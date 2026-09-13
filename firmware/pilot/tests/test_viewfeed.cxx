// viewfeed: bibowire's socket half, held to viewfeed.hxx over loopback.
//
//   ctest --test-dir build-pilot -R viewfeed     g++ on Linux
//
// Not in a .bat: the real half is Linux-only (accept4, pipe2, poll, recvfrom,
// MSG_NOSIGNAL) and on MSVC every call refuses, which is all the #else half checks.
//
// Not proved: that a JPEG comes off a real sensor. /dev/video0 has one opener and
// the pilot on the board holds it while a viewer watches, so BIBO_CAM_DEV points at
// a missing device and the camera check accepts a reason. To use the real device:
//
//   BIBO_CAM_DEV=/dev/video0 ./test_viewfeed
//
// Every socket read has a deadline, so a feed that sends nothing fails rather than
// hangs.
#include "shared.hxx"

#include "car.hxx"
#include "viewfeed.hxx"

#include <cstdio>

#if defined(__linux__)
#include <arpa/inet.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

static Int32 failures = 0;
static Int32 checks = 0;

static Void check(Bool ok, const Char* what)
{
    ++checks;
    if(ok)
    {
        std::printf("  ok    %s\n", what);
    }
    else
    {
        std::printf("  FAIL  %s\n", what);
        ++failures;
    }
}

static Void checkStr(const Str& got, const Char* want, const Char* what)
{
    const Bool ok = (got == want);
    check(ok, what);
    if(!ok)
    {
        std::printf("        got  \"%s\"\n        want \"%s\"\n", got.c_str(), want);
    }
}

#if defined(__linux__)

// A loopback bibowire client on the board's own codec, so no second copy of an
// offset can disagree with it.
struct Wire
{
    Int32 fd = -1;
    Vec<UInt8> in;
    Vec<UInt8> held;        // the frame most recently taken, kept alive
    bibowire::Frame f;      // points INTO `held`

    // A non-zero rcvBuf caps the receive buffer before connect, so the window scale
    // is negotiated with it. Without it loopback autotunes to megabytes and a
    // stalled viewer never pushes back.
    [[nodiscard]] Bool connect(UInt16 port, Int32 rcvBuf = 0)
    {
        fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if(fd < 0)
        {
            return false;
        }
        if(rcvBuf > 0)
        {
            static_cast<Void>(::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvBuf, sizeof(rcvBuf)));
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);
        if(::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            ::close(fd);
            fd = -1;
            return false;
        }
        return true;
    }

    Void raw(const UInt8* bytes, Size len)
    {
        if(fd >= 0)
        {
            static_cast<Void>(::send(fd, bytes, len, MSG_NOSIGNAL));
        }
    }

    Void put(bibowire::Type type, const UInt8* body, Size len, UInt16 seq)
    {
        Array<UInt8, 512> frame{};
        bibowire::Head h;
        h.type = type;
        h.ver = 1;
        h.seq = seq;
        bibowire::Body b;
        b.bytes = body;
        b.len = len;
        const Size total = bibowire::put(h, b, frame.data(), frame.size());
        raw(frame.data(), total);
    }

    Void subscribe(UInt32 session, UInt32 typeMask, UInt16 divisor, UInt16 seq)
    {
        bibowire::Subscribe m;
        m.sessionId = session;
        m.typeMask = typeMask;
        m.scanDivisor = divisor;
        Array<UInt8, 32> body{};
        const Size len = bibowire::writeSubscribe(m, body.data(), body.size());
        put(bibowire::Type::TYPE_SUBSCRIBE, body.data(), len, seq);
    }

    Void hello(UInt16 udpPort, UInt16 wantControl, const Str& name)
    {
        bibowire::Hello m;
        m.viewerUdpPort = udpPort;
        m.wantControl = wantControl;
        m.name = name;
        Array<UInt8, 128> body{};
        const Size len = bibowire::writeHello(m, body.data(), body.size());
        put(bibowire::Type::TYPE_HELLO, body.data(), len, 0);
    }

    // One decoded frame, or false after `ms` with nothing whole. `f` is valid
    // until the next call.
    [[nodiscard]] Bool next(Int32 ms)
    {
        const TimePoint start = monoNow();
        for(;;)
        {
            if(!in.empty())
            {
                bibowire::Frame got;
                Size used = 0;
                const bibowire::Take t = bibowire::take(in.data(), in.size(), &got, &used);
                if(t == bibowire::Take::TAKE_FRAME)
                {
                    held.assign(in.begin(), in.begin() + static_cast<ISize>(used));
                    in.erase(in.begin(), in.begin() + static_cast<ISize>(used));
                    f.head = got.head;
                    f.body.bytes = held.data() + bibowire::HEAD_BYTES;
                    f.body.len = got.body.len;
                    return true;
                }
                if(t == bibowire::Take::TAKE_RESYNC && used > 0u)
                {
                    in.erase(in.begin(), in.begin() + static_cast<ISize>(used));
                    continue;
                }
            }
            const Float64 left = static_cast<Float64>(ms) - elapsedMs(start);
            if(left <= 0.0 || fd < 0)
            {
                return false;
            }
            pollfd p{};
            p.fd = fd;
            p.events = POLLIN;
            if(::poll(&p, 1, static_cast<Int32>(left)) <= 0)
            {
                return false;
            }
            Array<UInt8, 8192> chunk{};
            const ISize n = ::recv(fd, chunk.data(), chunk.size(), 0);
            if(n <= 0)
            {
                ::close(fd);
                fd = -1;
                return false;
            }
            in.insert(in.end(), chunk.data(), chunk.data() + n);
        }
    }

    // The next frame of `type`, skipping others, such as the board's PINGs.
    [[nodiscard]] Bool nextOf(bibowire::Type type, Int32 ms)
    {
        const TimePoint start = monoNow();
        while(elapsedMs(start) < static_cast<Float64>(ms))
        {
            const Float64 left = static_cast<Float64>(ms) - elapsedMs(start);
            if(!next(static_cast<Int32>(left)))
            {
                return false;
            }
            if(f.head.type == type)
            {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] Bool closed(Int32 ms)
    {
        const TimePoint start = monoNow();
        while(fd >= 0 && elapsedMs(start) < static_cast<Float64>(ms))
        {
            pollfd p{};
            p.fd = fd;
            p.events = POLLIN;
            if(::poll(&p, 1, 50) <= 0)
            {
                continue;
            }
            Array<UInt8, 4096> chunk{};
            const ISize n = ::recv(fd, chunk.data(), chunk.size(), 0);
            if(n <= 0)
            {
                ::close(fd);
                fd = -1;
            }
        }
        return fd < 0;
    }

    // One line of plain ASCII, for the wrong-service check.
    [[nodiscard]] Str asciiLine(Int32 ms)
    {
        const TimePoint start = monoNow();
        for(;;)
        {
            for(Size i = 0; i < in.size(); ++i)
            {
                if(in[i] == '\n')
                {
                    Str line(in.begin(), in.begin() + static_cast<ISize>(i));
                    in.erase(in.begin(), in.begin() + static_cast<ISize>(i + 1u));
                    return line;
                }
            }
            const Float64 left = static_cast<Float64>(ms) - elapsedMs(start);
            if(left <= 0.0 || fd < 0)
            {
                return Str();
            }
            pollfd p{};
            p.fd = fd;
            p.events = POLLIN;
            if(::poll(&p, 1, static_cast<Int32>(left)) <= 0)
            {
                return Str();
            }
            Array<UInt8, 1024> chunk{};
            const ISize n = ::recv(fd, chunk.data(), chunk.size(), 0);
            if(n <= 0)
            {
                ::close(fd);
                fd = -1;
                return Str();
            }
            in.insert(in.end(), chunk.data(), chunk.data() + n);
        }
    }

    Void close()
    {
        if(fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
    }
};

// The viewer's UDP end: sends CONTROL, receives CTLSTATE.
struct Datagram
{
    Int32 fd = -1;
    UInt16 port = 0;

    [[nodiscard]] Bool open()
    {
        fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if(fd < 0)
        {
            return false;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        socklen_t len = sizeof(addr);
        if(::bind(fd, reinterpret_cast<const sockaddr*>(&addr), len) < 0
           || ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) < 0)
        {
            close();
            return false;
        }
        port = ntohs(addr.sin_port);
        return true;
    }

    // A CONTROL carrying the mode the viewer believes and a throttle. control()
    // leaves assumedMode at 0, MANUAL.
    Void controlAs(UInt16 to, UInt32 session, UInt32 seq, UInt16 buttons, UInt8 mode, Int16 throttle, UInt8 epoch)
    {
        bibowire::Control m;
        m.sessionId = session;
        m.seq = seq;
        m.buttons = buttons;
        m.assumedMode = mode;
        m.throttleMilli = throttle;
        m.armEpoch = epoch;
        send(to, m);
    }

    Void control(UInt16 to, UInt32 session, UInt32 seq, UInt16 buttons)
    {
        bibowire::Control m;
        m.sessionId = session;
        m.seq = seq;
        m.buttons = buttons;
        send(to, m);
    }

    Void send(UInt16 to, const bibowire::Control& m)
    {
        Array<UInt8, 64> body{};
        const Size len = bibowire::writeControl(m, body.data(), body.size());
        Array<UInt8, 128> frame{};
        bibowire::Head h;
        h.type = bibowire::Type::TYPE_CONTROL;
        h.ver = 1;
        h.seq = static_cast<UInt16>(m.seq);
        bibowire::Body b;
        b.bytes = body.data();
        b.len = len;
        const Size total = bibowire::put(h, b, frame.data(), frame.size());
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(to);
        static_cast<Void>(::sendto(
            fd,
            frame.data(),
            total,
            0,
            reinterpret_cast<const sockaddr*>(&addr),
            sizeof(addr)
        ));
    }

    // Drains the socket and keeps the newest CTLSTATE: they queue up between
    // checks, so the first one is the past.
    [[nodiscard]] Bool newest(bibowire::CtlState* out)
    {
        Bool any = false;
        for(;;)
        {
            Array<UInt8, 1500> buf{};
            const ISize n = ::recv(fd, buf.data(), buf.size(), MSG_DONTWAIT);
            if(n <= 0)
            {
                return any;
            }
            bibowire::Frame got;
            Size used = 0;
            const bibowire::Take t = bibowire::take(buf.data(), static_cast<Size>(n), &got, &used);
            if(t != bibowire::Take::TAKE_FRAME || got.head.type != bibowire::Type::TYPE_CTLSTATE)
            {
                continue;
            }
            bibowire::CtlState one;
            if(bibowire::readCtlState(got.body, got.head.ver, &one))
            {
                *out = one;
                any = true;
            }
        }
    }

    // The newest CTLSTATE, waiting up to `ms` for at least one to exist.
    [[nodiscard]] Bool state(Int32 ms, bibowire::CtlState* out)
    {
        const TimePoint start = monoNow();
        for(;;)
        {
            if(newest(out))
            {
                return true;
            }
            const Float64 left = static_cast<Float64>(ms) - elapsedMs(start);
            if(left <= 0.0)
            {
                return false;
            }
            pollfd p{};
            p.fd = fd;
            p.events = POLLIN;
            if(::poll(&p, 1, static_cast<Int32>(left > 50.0 ? 50.0 : left)) < 0)
            {
                return false;
            }
        }
    }

    // Drains until the newest CTLSTATE satisfies `ok` or `ms` pass. The board
    // applies CONTROL on its own thread, so reading once would race it.
    template<typename Pred>
    [[nodiscard]] Bool stateWhere(Int32 ms, bibowire::CtlState* out, Pred ok)
    {
        const TimePoint start = monoNow();
        for(;;)
        {
            if(newest(out) && ok(*out))
            {
                return true;
            }
            if(elapsedMs(start) >= static_cast<Float64>(ms))
            {
                return false;
            }
            sleepMs(10);
        }
    }

    Void close()
    {
        if(fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
    }
};

static Bool clientsReach(Size want, Int32 ms)
{
    const TimePoint start = monoNow();
    while(viewfeed::clients() != want && elapsedMs(start) < static_cast<Float64>(ms))
    {
        sleepMs(5);
    }
    return viewfeed::clients() == want;
}

// A revolution of `count` points, the shape the pilot publishes.
[[nodiscard]] static bibowire::Scan aScan(Size count, UInt32 rev)
{
    bibowire::Scan s;
    s.tMonoUs = 1000;
    s.revIndex = rev;
    s.freqMilliHz = 10000;
    s.health = 0;
    s.motor = 1;
    s.points.reserve(count);
    s.quality.reserve(count);
    for(Size i = 0; i < count; ++i)
    {
        bibowire::ScanPoint p;
        p.angleCentiDeg = static_cast<UInt16>((i * 35u) % 36000u);
        p.distMm = static_cast<UInt16>(1000 + (i % 500u));
        s.points.push_back(p);
        s.quality.push_back(static_cast<UInt8>(i % 64u));
    }
    return s;
}

[[nodiscard]] static viewfeed::Policy aPolicy()
{
    viewfeed::Policy p;
    p.boardName = "bibobox";
    p.boardBuild = "test";
    p.bootId = 0x0B00B1E5u;
    p.capabilities = 0x07u;
    return p;
}

// Welcomes one viewer and hands back its session id, or 0.
[[nodiscard]] static UInt32 handshake(Wire& w, UInt16 udp, UInt16 wantControl)
{
    w.hello(udp, wantControl, "test");
    if(!w.nextOf(bibowire::Type::TYPE_WELCOME, 1000))
    {
        return 0;
    }
    bibowire::Welcome m;
    if(!bibowire::readWelcome(w.f.body, w.f.head.ver, &m))
    {
        return 0;
    }
    return m.sessionId;
}

// The next EVENT carrying the board's saved trim, skipping other EVENTs, or false.
[[nodiscard]] static Bool nextTrim(Wire& w, Str& text)
{
    for(Int32 tries = 0; tries < 8; ++tries)
    {
        if(!w.nextOf(bibowire::Type::TYPE_EVENT, 1000))
        {
            return false;
        }
        bibowire::Event e;
        if(bibowire::readEvent(w.f.body, w.f.head.ver, &e) && e.code == bibowire::EVENT_CODE_TRIM)
        {
            text = e.text;
            return true;
        }
    }
    return false;
}

Int32 main()
{
    std::printf("\nviewfeed - bibowire's socket half, over loopback\n\n");
    // Overwrite 0, so an operator's own BIBO_CAM_DEV wins.
    static_cast<Void>(::setenv("BIBO_CAM_DEV", "/dev/bibo-no-such-video", 0));
    const viewfeed::Policy policy = aPolicy();
    check(viewfeed::start(0, policy), "the feed starts on an ephemeral port");
    check(viewfeed::port() != 0, "and says which");
    check(!viewfeed::start(0, policy), "a second start() without a stop() is refused");
    check(viewfeed::clients() == 0, "no viewers yet");
    viewfeed::publishScan(aScan(100, 1));
    viewfeed::publishDecide(bibowire::Decide());
    check(true, "publish() with nobody connected is harmless");
    const UInt16 port = viewfeed::port();
    // The board's own ages before anything has happened: 0 would read as perfectly
    // fresh, the most dangerous value on this wire.
    {
        Datagram udp;
        check(udp.open(), "a viewer binds its control socket");
        Wire fresh;
        check(fresh.connect(port), "and connects");
        const UInt32 session = handshake(fresh, udp.port, 1);
        check(session != 0u, "and takes control");
        bibowire::CtlState st;
        check(udp.state(2000, &st), "CTLSTATE arrives on UDP");
        check(st.scanAgeMs != 0u, "with NO revolution ever, scanAgeMs is NOT 0");
        std::printf(
            "        (scanAgeMs %u ms with no scan ever published)\n",
            static_cast<unsigned>(st.scanAgeMs)
        );
        check(st.controlAgeMs == bibowire::CONTROL_AGE_NEVER, "controlAgeMs is NEVER, not 0");
        check(st.picoSilentMs == bibowire::PICO_SILENT_ABSENT, "picoSilentMs is ABSENT, not 0");
        check(st.escUs == bibowire::ESC_ABSENT, "and the ESC pulse is unknown, not 0");
        viewfeed::publishScan(aScan(120, 1));
        sleepMs(150);
        check(udp.state(2000, &st), "after a revolution CTLSTATE still arrives");
        check(st.scanAgeMs < 2000u, "and scanAgeMs is a real, recent age");
        udp.control(port, session, 1, bibowire::BUTTON_ENABLE);
        const Bool applied = udp.stateWhere(2000, &st, [](const bibowire::CtlState& s) {
            return s.ackSeq == 1u;
        });
        check(applied, "CTLSTATE after a CONTROL names the seq the board applied");
        check(st.controlAgeMs != bibowire::CONTROL_AGE_NEVER, "controlAgeMs is a measurement now");
        fresh.close();
        udp.close();
        check(clientsReach(0, 2000), "the viewer leaves");
    }
    // The handshake sends state before any scan, so a viewer has something true to
    // draw when the picture appears.
    {
        bibowire::LidarInfo info;
        info.model = 0x41;
        info.baud = 460800;
        viewfeed::publishLidarInfo(info);
        bibowire::BoardState board;
        board.upS = 12;
        board.revolutions = 7;
        viewfeed::publishBoard(board);
        sleepMs(60);
        Wire a;
        check(a.connect(port), "a viewer connects");
        a.hello(0, 0, "observer");
        check(a.next(1000), "and gets an answer");
        check(a.f.head.type == bibowire::Type::TYPE_WELCOME, "the first frame is WELCOME");
        bibowire::Welcome w;
        check(bibowire::readWelcome(a.f.body, a.f.head.ver, &w), "which reads whole");
        check(w.sessionId != 0u, "with a session id that is never 0");
        check(w.bootId == 0x0B00B1E5u, "and the pilot process's bootId");
        check(w.accepted == 2u, "an observer is accepted as one");
        check(w.staleMs == 150u && w.deadMs == 300u, "the deadman numbers come from the board");
        checkStr(w.boardName, "bibobox", "and the board names itself");
        check(a.next(1000), "a second frame follows");
        check(a.f.head.type == bibowire::Type::TYPE_LIDAR_INFO, "LIDAR_INFO, before any scan");
        check(a.next(1000), "a third frame follows");
        check(a.f.head.type == bibowire::Type::TYPE_BOARD, "BOARD, still before any scan");
        bibowire::BoardState got;
        check(bibowire::readBoard(a.f.body, a.f.head.ver, &got), "the board state reads whole");
        check(got.upS == 12u && got.revolutions == 7u, "carrying what the pilot published");
        check(got.clients == 1u, "and what only the socket half can know");
        viewfeed::publishScan(aScan(500, 42));
        check(a.nextOf(bibowire::Type::TYPE_SCAN, 1000), "and THEN the scan arrives");
        bibowire::Scan s;
        check(bibowire::readScan(a.f.body, a.f.head.ver, &s), "which reads whole");
        check(s.points.size() == 500u, "with every point");
        check(s.revIndex == 42u, "and the revolution it was");
        check(s.droppedSinceLast == 0u, "nothing dropped for a viewer that is reading");
        // FLAG_ESTOP and FLAG_DEADMAN ride every board frame. With nobody holding the
        // slot the deadman does not apply, so its bit is clear.
        check(
            (a.f.head.flags & bibowire::FLAG_DEADMAN) == 0u,
            "with no holder the deadman does not apply, and FLAG_DEADMAN is clear"
        );
        check((a.f.head.flags & bibowire::FLAG_ESTOP) == 0u, "and no e-stop is latched");
        a.close();
        check(clientsReach(0, 2000), "and the count goes back to nothing when it leaves");
    }
    // An observer must not arm a deadman that would stop a car nobody is driving.
    {
        Datagram udp;
        check(udp.open(), "an observer binds a control socket it will not use");
        Wire watcher;
        check(watcher.connect(port), "and connects");
        watcher.hello(udp.port, 0, "watcher");
        check(watcher.nextOf(bibowire::Type::TYPE_WELCOME, 1000), "it is welcomed");
        bibowire::Welcome w;
        check(
            bibowire::readWelcome(watcher.f.body, watcher.f.head.ver, &w),
            "the WELCOME reads whole"
        );
        check(w.accepted == 2u, "as an observer");
        check(w.refusal == 0u, "with NO refusal - it never asked for the wheel");
        // featureMask uses bit = tag - 0x10.
        check(w.featureMask != 0u, "WELCOME names what this board will send");
        const UInt32 scanBit = 1u << (0x10u - 0x10u);
        const UInt32 decideBit = 1u << (0x11u - 0x10u);
        check((w.featureMask & scanBit) != 0u, "including SCAN");
        check((w.featureMask & decideBit) != 0u, "and DECIDE, which does not collide with SCHEMA");
        bibowire::CtlState st;
        check(udp.state(2000, &st), "the observer still gets CTLSTATE");
        check(st.holder == 0u, "which says the wheel is NOBODY's");
        check(st.deadman == 0u, "the deadman is not armed on an observer's account");
        check(st.refuse == bibowire::Refuse::REFUSE_NONE, "and nothing is being refused");
        viewfeed::publishScan(aScan(200, 900));
        check(watcher.nextOf(bibowire::Type::TYPE_SCAN, 2000), "and it is served revolutions");
        watcher.close();
        udp.close();
        check(clientsReach(0, 2000), "the observer leaves");
    }
    {
        Wire loud;
        check(loud.connect(port), "a viewer connects");
        bibowire::Hello m;
        m.featureMask = 0xFFFFFFFFu;
        m.viewerUdpPort = 0;
        m.wantControl = 0;
        m.name = "all-bits";
        Array<UInt8, 128> body{};
        const Size len = bibowire::writeHello(m, body.data(), body.size());
        loud.put(bibowire::Type::TYPE_HELLO, body.data(), len, 0);
        check(
            loud.nextOf(bibowire::Type::TYPE_WELCOME, 1000),
            "HELLO with ALL bits set is answered"
        );
        bibowire::Welcome w;
        check(bibowire::readWelcome(loud.f.body, loud.f.head.ver, &w), "the WELCOME reads whole");
        check(w.accepted == 2u, "and accepted, not refused for an unknown bit");
        check(clientsReach(1, 2000), "and counted before anything is published to it");
        viewfeed::publishScan(aScan(64, 901));
        check(loud.nextOf(bibowire::Type::TYPE_SCAN, 2000), "and it is served telemetry");
        loud.close();
        check(clientsReach(0, 2000), "it leaves");
    }
    {
        Wire nc;
        check(nc.connect(port), "somebody dials the binary port by hand");
        const Str typed = "hello?\n";
        nc.raw(reinterpret_cast<const UInt8*>(typed.data()), typed.size());
        const Str said = nc.asciiLine(1000);
        check(said.find("bibowire v1 binary on 8020") != Str::npos, "and is answered in words");
        check(nc.closed(2000), "then closed");
        nc.close();
    }
    {
        Wire big;
        check(big.connect(port), "a viewer connects");
        // A header claiming more than MAX_INBOUND_PAYLOAD: the board must answer from
        // the header alone, since the body would never fit its receive ring.
        Array<UInt8, bibowire::HEAD_BYTES> head{};
        bibowire::wr16(head.data(), bibowire::MAGIC);
        bibowire::wr8(head.data() + 2u, static_cast<UInt8>(bibowire::Type::TYPE_HELLO));
        bibowire::wr8(head.data() + 3u, 1);
        bibowire::wr16(head.data() + 4u, 0);
        bibowire::wr16(head.data() + 6u, 0);
        bibowire::wr32(head.data() + 8u, 200000u);
        big.raw(head.data(), head.size());
        check(big.nextOf(bibowire::Type::TYPE_BYE, 1000), "the board answers BYE");
        bibowire::Bye m;
        check(bibowire::readBye(big.f.body, big.f.head.ver, &m), "which reads whole");
        check(m.reason == bibowire::Reason::REASON_TOO_BIG, "with reason TOO_BIG");
        check(!m.text.empty(), "and a sentence saying what the bound is");
        check(big.closed(2000), "and the connection is closed");
        big.close();
    }
    {
        Wire twice;
        check(twice.connect(port), "a viewer connects");
        check(handshake(twice, 0, 0) != 0u, "and is welcomed");
        twice.hello(0, 0, "again");
        check(twice.closed(2000), "a second HELLO closes it");
        twice.close();
        check(clientsReach(0, 2000), "and it is no longer counted");
    }
    {
        Datagram udpA;
        Datagram udpB;
        check(udpA.open() && udpB.open(), "two viewers bind their control sockets");
        Wire driver;
        check(driver.connect(port), "the first viewer connects");
        const UInt32 sessionA = handshake(driver, udpA.port, 1);
        check(sessionA != 0u, "and asks for control");
        Wire second;
        check(second.connect(port), "a second viewer connects");
        second.hello(udpB.port, 1, "second");
        check(second.nextOf(bibowire::Type::TYPE_WELCOME, 1000), "and is answered, not ignored");
        bibowire::Welcome w;
        check(
            bibowire::readWelcome(second.f.body, second.f.head.ver, &w),
            "the WELCOME reads whole"
        );
        check(w.accepted == 2u, "accepted as an OBSERVER");
        check(w.refusal == 2u, "with refusal 2, control taken");
        check(!w.text.empty(), "and a sentence naming who holds it");
        std::printf("        (\"%s\")\n", w.text.c_str());
        const UInt32 sessionB = w.sessionId;
        check(sessionB != sessionA, "the two sessions are different");
        const viewfeed::Counters before = viewfeed::counters();
        for(UInt32 i = 1; i <= 8u; ++i)
        {
            udpA.control(port, sessionA, i, bibowire::BUTTON_ENABLE);
            sleepMs(10);
        }
        bibowire::CtlState st;
        const Bool eight = udpA.stateWhere(2000, &st, [](const bibowire::CtlState& s) {
            return s.ackSeq == 8u;
        });
        check(eight, "the holder's CTLSTATE carries the seq the board APPLIED");
        check(st.holder == 1u, "and tells it the wheel is its own");
        // An observer's CONTROL must not feed the holder's deadman.
        for(UInt32 i = 1; i <= 5u; ++i)
        {
            udpB.control(port, sessionB, i, bibowire::BUTTON_ENABLE);
            sleepMs(10);
        }
        sleepMs(150);
        const viewfeed::Counters after = viewfeed::counters();
        check(after.rxControlStale > before.rxControlStale, "the observer's datagrams are counted");
        check(udpA.state(1000, &st), "the holder still hears CTLSTATE");
        check(st.ackSeq == 8u, "and the observer moved nothing");
        check(udpB.state(1000, &st), "the observer hears CTLSTATE too");
        check(st.holder == 2u, "which tells it another viewer is driving");
        bibowire::Leave leave;
        leave.sessionId = sessionA;
        Array<UInt8, 16> body{};
        const Size len = bibowire::writeLeave(leave, body.data(), body.size());
        driver.put(bibowire::Type::TYPE_LEAVE, body.data(), len, 1);
        check(driver.closed(2000), "LEAVE closes the driver's connection");
        const Bool released = udpB.stateWhere(2000, &st, [](const bibowire::CtlState& s) {
            return s.holder == 0u;
        });
        check(released, "the observer hears that the wheel is nobody's again");
        driver.close();
        second.close();
        udpA.close();
        udpB.close();
        check(clientsReach(0, 2000), "both viewers are gone");
    }
    {
        Datagram udp;
        check(udp.open(), "a viewer binds its control socket");
        Wire first;
        check(first.connect(port), "it connects");
        const UInt32 old = handshake(first, udp.port, 1);
        check(old != 0u, "and takes control");
        udp.control(port, old, 1, bibowire::BUTTON_ENABLE);
        bibowire::CtlState st;
        const Bool first1 = udp.stateWhere(2000, &st, [](const bibowire::CtlState& s) {
            return s.ackSeq == 1u;
        });
        check(first1, "its control is applied and acknowledged");
        first.close();
        check(clientsReach(0, 2000), "then the hotspot blips and it is gone");
        Wire again;
        check(again.connect(port), "it reconnects");
        const UInt32 fresh = handshake(again, udp.port, 1);
        check(fresh != 0u, "and is welcomed");
        check(fresh != old, "with a NEW session id - nothing is resumed");
        // Still in flight from before the blip, with a higher seq: only the session
        // id stops it driving the car with a second-old stick position.
        const viewfeed::Counters before = viewfeed::counters();
        udp.control(port, old, 99, bibowire::BUTTON_ENABLE);
        sleepMs(200);
        const viewfeed::Counters after = viewfeed::counters();
        check(
            after.rxControlStale > before.rxControlStale,
            "the stale session's datagram is counted"
        );
        check(udp.state(1000, &st), "CTLSTATE still arrives");
        check(st.ackSeq == 0u, "and NOTHING from the old session was applied");
        udp.control(port, fresh, 1, bibowire::BUTTON_ENABLE);
        const Bool reset = udp.stateWhere(2000, &st, [](const bibowire::CtlState& s) {
            return s.ackSeq == 1u;
        });
        check(reset, "the new session's seq 1 is applied - the mark is reset per session");
        again.close();
        udp.close();
        check(clientsReach(0, 2000), "the viewer leaves");
    }
    {
        Wire quiet;
        check(quiet.connect(port), "a viewer asks for control");
        // It names a UDP port and never sends: a failure the viewer cannot see itself.
        check(handshake(quiet, 40000, 1) != 0u, "and is given the wheel");
        check(
            quiet.nextOf(bibowire::Type::TYPE_EVENT, 3000),
            "an EVENT arrives within a few seconds"
        );
        bibowire::Event e;
        check(bibowire::readEvent(quiet.f.body, quiet.f.head.ver, &e), "which reads whole");
        check(e.severity == bibowire::Severity::SEVERITY_WARN, "as a warning");
        check(e.text.find("no control datagrams") != Str::npos, "naming the symptom");
        check(e.text.find("will not move") != Str::npos, "and what it means for the car");
        std::printf("        (\"%s\")\n", e.text.c_str());
        quiet.close();
        check(clientsReach(0, 2000), "the viewer leaves");
    }
    // A stalled viewer is coalesced, then dropped, while another keeps receiving: the
    // pilot's tick depends on it. TINY_RCVBUF holds less than one 1024-point
    // revolution, so the board is pushed back at once.
    {
        constexpr Int32 TINY_RCVBUF = 2048;
        Wire lag;
        check(lag.connect(port, TINY_RCVBUF), "a viewer with a tiny receive buffer connects");
        check(handshake(lag, 0, 0) != 0u, "and is welcomed");
        check(clientsReach(1, 2000), "and is counted");
        // A burst it cannot absorb, short enough to finish before the first PING,
        // whose vital frame would close the client on BEHIND_MS.
        for(UInt32 i = 0; i < 24u; ++i)
        {
            viewfeed::publishScan(aScan(1024, 200u + i));
        }
        sleepMs(120);
        UInt16 told = 0;
        UInt32 got = 0;
        while(got < 4u && lag.nextOf(bibowire::Type::TYPE_SCAN, 500))
        {
            bibowire::Scan s;
            if(!bibowire::readScan(lag.f.body, lag.f.head.ver, &s))
            {
                break;
            }
            ++got;
            if(s.droppedSinceLast > told)
            {
                told = s.droppedSinceLast;
            }
        }
        check(got > 0u, "it still receives whole revolutions");
        check(told > 0u, "and is TOLD how many were coalesced away for it");
        std::printf("        (droppedSinceLast reached %u)\n", static_cast<unsigned>(told));
        lag.close();
        check(clientsReach(0, 3000), "it leaves");
        Wire slow;
        Wire fast;
        check(slow.connect(port, TINY_RCVBUF), "a stalling viewer connects");
        check(handshake(slow, 0, 0) != 0u, "and is welcomed");
        check(fast.connect(port), "a reading viewer connects");
        check(handshake(fast, 0, 0) != 0u, "and is welcomed too");
        check(clientsReach(2, 2000), "two viewers");
        const TimePoint stall = monoNow();
        Bool slowGone = false;
        UInt32 seen = 0;
        UInt32 rev = 300;
        while(elapsedMs(stall) < 8000.0 && !slowGone)
        {
            for(Int32 i = 0; i < 4; ++i)
            {
                viewfeed::publishScan(aScan(1024, rev));
                ++rev;
            }
            if(fast.nextOf(bibowire::Type::TYPE_SCAN, 500))
            {
                ++seen;
            }
            slowGone = viewfeed::clients() <= 1u;
            sleepMs(20);
        }
        const Float64 tookMs = elapsedMs(stall);
        check(slowGone, "the viewer that stopped reading is dropped");
        // The PING timeout cannot fire before 1000 + 4000 ms, so a sooner drop is
        // backpressure.
        check(tookMs < 4000.0, "for being BEHIND, not by the PING timeout");
        check(seen > 0u, "while the reading viewer kept receiving revolutions");
        check(slow.closed(2000), "and the stalled socket is closed");
        std::printf(
            "        (dropped after %.0f ms, the reader saw %u revolutions)\n",
            tookMs,
            static_cast<unsigned>(seen)
        );
        slow.close();
        fast.close();
        check(clientsReach(0, 3000), "both are gone");
    }
    check(
        bibowire::classOf(bibowire::Type::TYPE_CAMERA) == bibowire::Class::CLASS_BULK,
        "CAMERA is BULK - discarded first, always"
    );
    check(
        bibowire::classOf(bibowire::Type::TYPE_SCAN) == bibowire::Class::CLASS_LIVE,
        "SCAN is LIVE - drop-oldest, newest revolution wins"
    );
    check(
        bibowire::classOf(bibowire::Type::TYPE_BOARD) == bibowire::Class::CLASS_VITAL,
        "BOARD is VITAL - never dropped"
    );
    check(
        bibowire::classOf(bibowire::Type::TYPE_EVENT) == bibowire::Class::CLASS_VITAL,
        "and so is the sentence that says why"
    );
    // The zero-mask trap: a typeMask of 0 means "never asked" and is read as
    // everything except CAMERA.
    {
        const UInt32 cameraBit = 1u << (0x20u - 0x10u);
        check(cameraBit == (1u << 16u), "CAMERA's subscription bit is bit 16");
        Wire zero;
        check(zero.connect(port), "a viewer that never subscribes connects");
        check(handshake(zero, 0, 0) != 0u, "and is welcomed");
        check(clientsReach(1, 2000), "and is counted");
        // Served what a zero mask includes, so the camera check below is not vacuous.
        viewfeed::publishScan(aScan(64, 910));
        check(zero.nextOf(bibowire::Type::TYPE_SCAN, 2000), "a zero mask still receives SCAN");
        bibowire::BoardState state;
        state.upS = 21;
        viewfeed::publishBoard(state);
        check(zero.nextOf(bibowire::Type::TYPE_BOARD, 2000), "and still receives BOARD");
        // Long enough for a capture to start, fail and be explained several times over.
        Bool sawCamera = false;
        const TimePoint watch = monoNow();
        while(elapsedMs(watch) < 1500.0 && !sawCamera)
        {
            if(!zero.next(200))
            {
                continue;
            }
            sawCamera = zero.f.head.type == bibowire::Type::TYPE_CAMERA;
        }
        check(!sawCamera, "but a zero mask receives NO CAMERA - it never asked for one");
        zero.close();
        check(clientsReach(0, 2000), "it leaves");
    }
    {
        const UInt32 cameraBit = 1u << (0x20u - 0x10u);
        Wire watcher;
        check(watcher.connect(port), "a viewer that wants pictures connects");
        watcher.hello(0, 0, "camera");
        check(watcher.nextOf(bibowire::Type::TYPE_WELCOME, 1000), "and is welcomed");
        bibowire::Welcome w;
        check(
            bibowire::readWelcome(watcher.f.body, watcher.f.head.ver, &w),
            "the WELCOME reads whole"
        );
        // featureMask is what the board will send on request, so it includes the
        // camera while it is off.
        check(
            (w.featureMask & cameraBit) != 0u,
            "the board ADVERTISES the camera it will send on request"
        );
        check(w.sessionId != 0u, "with a session id");
        watcher.subscribe(w.sessionId, cameraBit, 1, 1);
        Bool sawCamera = false;
        Bool sawWhy = false;
        Str why;
        const TimePoint asked = monoNow();
        while(elapsedMs(asked) < 6000.0 && !sawCamera && !sawWhy)
        {
            if(!watcher.next(500))
            {
                continue;
            }
            if(watcher.f.head.type == bibowire::Type::TYPE_CAMERA)
            {
                sawCamera = true;
                break;
            }
            if(watcher.f.head.type != bibowire::Type::TYPE_EVENT)
            {
                continue;
            }
            bibowire::Event e;
            if(bibowire::readEvent(watcher.f.body, watcher.f.head.ver, &e)
               && e.text.find("camera") != Str::npos)
            {
                sawWhy = true;
                why = e.text;
            }
        }
        check(
            sawCamera || sawWhy,
            "asking for the camera is answered - a picture, or a reason, never silence"
        );
        if(sawCamera)
        {
            bibowire::Camera shot;
            check(
                bibowire::readCamera(watcher.f.body, watcher.f.head.ver, &shot),
                "the CAMERA frame reads whole"
            );
            check(shot.codec == 1u, "codec 1, JPEG, echoed on every frame");
            check(shot.width != 0u && shot.height != 0u, "with a real size, never a plausible 0");
            check(!shot.data.empty(), "and bytes in it");
            check(shot.tMonoUs != 0u, "and a stamp from the board's own clock");
            std::printf(
                "        (a %ux%u frame, %u bytes, index %u)\n",
                static_cast<unsigned>(shot.width),
                static_cast<unsigned>(shot.height),
                static_cast<unsigned>(shot.data.size()),
                static_cast<unsigned>(shot.frameIndex)
            );
        }
        if(sawWhy)
        {
            check(
                why.find("absent") != Str::npos || why.find("busy") != Str::npos
                    || why.find("holding") != Str::npos,
                "and the reason says what is wrong with the device"
            );
            std::printf("        (\"%s\")\n", why.c_str());
        }
        watcher.close();
        check(clientsReach(0, 2000), "the viewer leaves and the device is released");
    }
    {
        Array<Wire, 4> four{};
        Bool allIn = true;
        for(Size i = 0; i < 4u; ++i)
        {
            allIn = allIn && four[i].connect(port) && handshake(four[i], 0, 0) != 0u;
        }
        check(allIn, "four viewers connect and are welcomed");
        check(clientsReach(4, 3000), "and all four are counted");
        Wire fifth;
        check(fifth.connect(port), "a fifth connects");
        fifth.hello(0, 0, "fifth");
        check(fifth.nextOf(bibowire::Type::TYPE_WELCOME, 1000), "and is ANSWERED, not ignored");
        bibowire::Welcome w;
        check(bibowire::readWelcome(fifth.f.body, fifth.f.head.ver, &w), "the WELCOME reads whole");
        check(w.accepted == 0u, "refused");
        check(w.refusal == 3u, "with refusal 3, too many viewers");
        check(w.text.find("127.0.0.1") != Str::npos, "naming the addresses already connected");
        check(fifth.nextOf(bibowire::Type::TYPE_BYE, 1000), "and a BYE follows");
        bibowire::Bye bye;
        check(bibowire::readBye(fifth.f.body, fifth.f.head.ver, &bye), "which reads whole");
        check(bye.reason == bibowire::Reason::REASON_REFUSED, "with reason REFUSED");
        check(fifth.closed(2000), "then the socket closes");
        fifth.close();
        for(Size i = 0; i < 4u; ++i)
        {
            four[i].close();
        }
        check(clientsReach(0, 3000), "and the room empties");
    }
    {
        Wire ponger;
        check(ponger.connect(port), "a viewer connects");
        check(handshake(ponger, 0, 0) != 0u, "and is welcomed");
        check(ponger.nextOf(bibowire::Type::TYPE_PING, 2500), "the board pings it");
        bibowire::Ping p;
        check(bibowire::readPing(ponger.f.body, ponger.f.head.ver, &p), "the PING reads whole");
        Array<UInt8, 32> body{};
        const Size len = bibowire::writePing(p, body.data(), body.size());
        ponger.put(bibowire::Type::TYPE_PONG, body.data(), len, 1);
        // The viewer's latency readout needs its own token echoed byte for byte.
        bibowire::Ping mine;
        mine.token = 0xDEADBEEFCAFEF00Dull;
        mine.senderMonoUs = 12345u;
        Array<UInt8, 32> pbody{};
        const Size plen = bibowire::writePing(mine, pbody.data(), pbody.size());
        ponger.put(bibowire::Type::TYPE_PING, pbody.data(), plen, 2);
        check(ponger.nextOf(bibowire::Type::TYPE_PONG, 2000), "a viewer's PING is answered");
        bibowire::Ping back;
        check(bibowire::readPing(ponger.f.body, ponger.f.head.ver, &back), "the PONG reads whole");
        check(back.token == 0xDEADBEEFCAFEF00Dull, "and the token is echoed VERBATIM");
        check(back.senderMonoUs != 12345u, "with the BOARD's own clock in senderMonoUs");
        sleepMs(1500);
        check(viewfeed::clients() == 1u, "a viewer that PONGs is kept");
        // A phone out of range stops ACKing without a FIN; only the PING timeout notices.
        const TimePoint waited = monoNow();
        const Bool timedOut = ponger.closed(7000);
        check(timedOut, "a viewer that stops answering PING is closed");
        std::printf("        (closed %.0f ms after it went quiet)\n", elapsedMs(waited));
        ponger.close();
        check(clientsReach(0, 2000), "and it is gone");
    }
    {
        Wire last;
        check(last.connect(port), "one more viewer connects");
        check(handshake(last, 0, 0) != 0u, "and is welcomed");
        viewfeed::stop();
        check(last.closed(2000), "stop() closes it");
        check(
            viewfeed::clients() == 0 && viewfeed::port() == 0,
            "and reports no viewers and no port"
        );
        last.close();
        viewfeed::publishScan(aScan(10, 1));
        viewfeed::stop();
        check(true, "publish() and stop() are harmless with nothing running");
        check(viewfeed::start(port, aPolicy()), "the same port can be taken again straight after");
        viewfeed::stop();
    }
    // Tuning COMMANDs over a real socket. No BOARD is published, so picoDown() is false
    // ("not said yet" is not "no Pico") and nothing is refused with result 4; nothing
    // calls applied(), so the car reads as disarmed.
    {
        check(viewfeed::start(0, aPolicy()), "the feed starts for the tuning tests");
        const UInt16 port = viewfeed::port();
        Wire w;
        check(w.connect(port), "a viewer connects to tune");
        const UInt32 session = handshake(w, 0, 0);
        check(session != 0, "and is welcomed - an OBSERVER, because tuning is not driving");
        Array<UInt8, 32> body{};
        viewfeed::Tune t;
        {
            bibowire::Command m;
            m.sessionId = session;
            m.cmdId = 7;
            m.verb = bibowire::Verb::VERB_SET_SLEW;
            m.arg0 = bibowire::SLEW_AXIS_THROTTLE;
            m.arg1 = 37;
            const Size len = bibowire::writeCommand(m, body.data(), body.size());
            w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 20);
            check(w.nextOf(bibowire::Type::TYPE_CMDACK, 1000), "a tuning COMMAND is answered");
            bibowire::CmdAck ack;
            check(bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack), "and the ack decodes");
            check(ack.cmdId == 7, "against the cmdId that was sent");
            check(ack.result == 0, "and the value was accepted");
            check(!ack.text.empty(), "with a sentence naming what the car took");
            check(viewfeed::tune(&t), "the pilot's tick finds it queued");
            check(t.verb == bibowire::Verb::VERB_SET_SLEW, "the verb survived");
            check(t.arg0 == bibowire::SLEW_AXIS_THROTTLE, "the AXIS survived - 2, not 1");
            check(t.arg1 == 37, "and the rate survived");
            check(!viewfeed::tune(&t), "and the queue is empty once taken");
        }
        // A refusal that still queued the value would hand the car a number the
        // operator was told it would not take.
        {
            bibowire::Command m;
            m.sessionId = session;
            m.cmdId = 8;
            m.verb = bibowire::Verb::VERB_SET_SLEW;
            m.arg0 = bibowire::SLEW_AXIS_STEER;
            m.arg1 = 500;   // past SLEW_US_MAX
            const Size len = bibowire::writeCommand(m, body.data(), body.size());
            w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 21);
            check(w.nextOf(bibowire::Type::TYPE_CMDACK, 1000), "an out-of-range slew is answered");
            bibowire::CmdAck ack;
            check(bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack), "the ack decodes");
            check(ack.result == 1, "and it is refused");
            check(!viewfeed::tune(&t), "and NOTHING was queued for the car");
        }
        {
            bibowire::Command m;
            m.sessionId = session;
            m.cmdId = 9;
            m.verb = bibowire::Verb::VERB_SET_SERVO_LIMITS;
            m.arg1 = 1600;
            m.arg2 = 1400;
            const Size len = bibowire::writeCommand(m, body.data(), body.size());
            w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 22);
            check(
                w.nextOf(bibowire::Type::TYPE_CMDACK, 1000),
                "reversed servo limits are answered"
            );
            bibowire::CmdAck ack;
            check(bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack), "the ack decodes");
            check(ack.result == 1, "and refused - both ends are in range, the ORDER is not");
            check(!viewfeed::tune(&t), "and nothing was queued");
        }
        {
            bibowire::Command m;
            m.sessionId = session ^ 0xFFFFFFFFu;
            m.cmdId = 10;
            m.verb = bibowire::Verb::VERB_SET_SERVO_TRIM;
            m.arg1 = 1487;
            const Size len = bibowire::writeCommand(m, body.data(), body.size());
            w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 23);
            check(w.nextOf(bibowire::Type::TYPE_CMDACK, 1000), "a foreign session is answered");
            bibowire::CmdAck ack;
            check(bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack), "the ack decodes");
            check(ack.result != 0, "and refused before the trim is ever looked at");
            check(!viewfeed::tune(&t), "and nothing was queued");
        }
        {
            bibowire::Command m;
            m.sessionId = session;
            m.cmdId = 11;
            m.verb = bibowire::Verb::VERB_SET_ESC_REVERSE;
            m.arg1 = 1350;
            Size len = bibowire::writeCommand(m, body.data(), body.size());
            w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 24);
            check(w.nextOf(bibowire::Type::TYPE_CMDACK, 1000), "a reverse limit is answered");
            bibowire::CmdAck ack;
            check(
                bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack) && ack.result == 0,
                "and 1350 is taken"
            );
            check(
                viewfeed::tune(&t) && t.verb == bibowire::Verb::VERB_SET_ESC_REVERSE && t.arg1 == 1350,
                "and reaches the tick intact"
            );
            m.cmdId = 12;
            m.arg1 = bibowire::ESC_NEUTRAL_US;
            len = bibowire::writeCommand(m, body.data(), body.size());
            w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 25);
            check(
                w.nextOf(bibowire::Type::TYPE_CMDACK, 1000),
                "neutral as a reverse limit is answered"
            );
            check(
                bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack) && ack.result == 0,
                "and taken - it is reverse off"
            );
            check(
                viewfeed::tune(&t) && t.arg1 == bibowire::ESC_NEUTRAL_US,
                "and queued as neutral"
            );
            m.cmdId = 13;
            m.arg1 = 1600;
            len = bibowire::writeCommand(m, body.data(), body.size());
            w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 26);
            check(
                w.nextOf(bibowire::Type::TYPE_CMDACK, 1000),
                "a reverse limit above neutral is answered"
            );
            check(
                bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack) && ack.result == 1,
                "and refused"
            );
            check(!viewfeed::tune(&t), "and nothing was queued");
        }
        viewfeed::stop();
        check(!viewfeed::tune(&t), "a stopped feed has no trim waiting for the tick");
    }
    // The mode gate over a socket, where the board supplies its own mode; a codec test
    // chooses both sides of the comparison itself. A viewer holding W that believes
    // MANUAL while the pilot DRIVEs must have its throttle ignored.
    {
        check(viewfeed::start(0, aPolicy()), "the feed starts for the mode gate");
        const UInt16 port = viewfeed::port();
        // The board learns its mode from the pilot's BOARD, never from the viewer.
        bibowire::BoardState board;
        board.pilotMode = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_DRIVE);
        viewfeed::publishBoard(board);
        sleepMs(60);
        Datagram udp;
        check(udp.open(), "a viewer binds its control socket");
        Wire w;
        check(w.connect(port), "and connects");
        const UInt32 session = handshake(w, udp.port, 1);
        check(session != 0u, "and takes the control slot");
        // The epoch is read, not guessed: it bumps on every holder change, and
        // deadman::step checks enable, then epoch, then mode.
        bibowire::CtlState st;
        check(udp.state(2000, &st), "CTLSTATE arrives, carrying the board's epoch");
        const UInt8 epoch = st.armEpoch;
        // Streams while it waits: CONTROL_SLOT_MS of silence releases the slot and
        // bumps the epoch, and the next datagram would be refused as NOT_HOLDER.
        const auto pump = [&](UInt8 mode, UInt32 first, bibowire::Refuse want) {
            for(Int32 i = 0; i < 40; ++i)
            {
                const UInt32 seq = first + static_cast<UInt32>(i);
                udp.controlAs(
                    port,
                    session,
                    seq,
                    bibowire::BUTTON_ENABLE,
                    mode,
                    400,
                    epoch
                );
                sleepMs(50);
                if(udp.newest(&st) && st.ackSeq >= first && st.refuse == want)
                {
                    return true;
                }
            }
            return false;
        };
        // ENABLE on every one, or REFUSE_NOT_ARMED comes before the mode.
        const UInt8 manual = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL);
        const Bool refused = pump(manual, 1u, bibowire::Refuse::REFUSE_MODE);
        check(refused, "a viewer that believes MANUAL while the pilot DRIVES is REFUSE_MODE");
        check(
            st.deadman != static_cast<UInt8>(bibowire::deadman::State::STATE_LIVE),
            "and the deadman is not LIVE, so no throttle is consented to"
        );
        // The positive case, so a gate that refused everything would fail.
        const UInt8 drive = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_DRIVE);
        const Bool agreed = pump(drive, 100u, bibowire::Refuse::REFUSE_NONE);
        check(agreed, "and agreeing about the mode is refused for no reason at all");
        w.close();
        udp.close();
        viewfeed::stop();
        // lastBoard outlives stop() and start(), so the mode is put back for later
        // sections.
        viewfeed::publishBoard(bibowire::BoardState());
    }
    // ARM and DISARM: every refusal, and every road that takes an ARM away. Above all,
    // a stream that stalls and resumes must not find the car armed again.
    {
        check(viewfeed::start(0, aPolicy()), "the feed starts for arming");
        const UInt16 port = viewfeed::port();
        const UInt8 manual = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL);
        // An ARM is granted only over picoLink 1: 2 is up but silent and 0 is down.
        bibowire::BoardState board;
        board.pilotMode = manual;
        board.picoLink = 1u;
        viewfeed::publishBoard(board);
        sleepMs(60);
        Datagram udp;
        check(udp.open(), "a driver binds its control socket");
        Wire w;
        check(w.connect(port), "and connects");
        const UInt32 session = handshake(w, udp.port, 1);
        check(session != 0u, "and takes the control slot");
        Wire obs;
        check(obs.connect(port), "an observer connects beside it");
        const UInt32 watcher = handshake(obs, 0, 0);
        check(watcher != 0u, "and is welcomed as one");
        bibowire::CtlState st;
        check(udp.state(2000, &st), "CTLSTATE arrives, carrying the board's epoch");
        UInt32 seq = 0;
        UInt32 cmdId = 0;
        Array<UInt8, 32> body{};
        // The driver's stream, stamped with the newest epoch the board has said, so a
        // bump is followed rather than refused.
        const auto stream = [&](Int32 ms) {
            for(Int32 t = 0; t < ms; t += 40)
            {
                static_cast<Void>(udp.newest(&st));
                udp.controlAs(
                    port,
                    session,
                    ++seq,
                    bibowire::BUTTON_ENABLE,
                    manual,
                    0,
                    st.armEpoch
                );
                sleepMs(40);
            }
            static_cast<Void>(udp.newest(&st));
        };
        const auto command = [&](Wire& on, UInt32 sess, bibowire::Verb verb, UInt8 epoch, bibowire::CmdAck* ack) {
            bibowire::Command m;
            m.sessionId = sess;
            m.cmdId = static_cast<decltype(m.cmdId)>(++cmdId);
            m.verb = verb;
            m.armEpoch = epoch;
            const Size len = bibowire::writeCommand(m, body.data(), body.size());
            on.put(bibowire::Type::TYPE_COMMAND, body.data(), len, static_cast<UInt16>(cmdId));
            return on.nextOf(bibowire::Type::TYPE_CMDACK, 1000)
                && bibowire::readCmdAck(on.f.body, on.f.head.ver, ack)
                && ack->cmdId == m.cmdId;
        };
        // Polls drive(), streaming while it waits when asked, so a slot released by
        // silence cannot be what makes a check pass.
        const auto settles = [&](Bool want, Int32 ms, Bool streaming) {
            for(Int32 t = 0; t < ms; t += 20)
            {
                if(viewfeed::drive().armed == want)
                {
                    return true;
                }
                if(streaming)
                {
                    udp.controlAs(
                        port,
                        session,
                        ++seq,
                        bibowire::BUTTON_ENABLE,
                        manual,
                        0,
                        st.armEpoch
                    );
                }
                sleepMs(20);
            }
            return viewfeed::drive().armed == want;
        };
        const auto has = [](const Str& text, CharSeq word) {
            return text.find(word) != Str::npos;
        };
        bibowire::CmdAck ack;
        check(
            command(obs, watcher, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "an observer's ARM is answered"
        );
        check(
            ack.result == 1 && has(ack.text, "holding"),
            "and refused - you cannot arm a car you are not holding"
        );
        check(!viewfeed::drive().armed, "and nothing is armed");
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "the driver's ARM with no stream is answered"
        );
        check(
            ack.result == 1 && has(ack.text, "live"),
            "and refused for the stream, in those words"
        );
        stream(200);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "an ARM 200 ms into the stream is answered"
        );
        check(ack.result == 1 && !viewfeed::drive().armed, "and refused - REARM_STREAM_MS is 500");
        stream(450);
        const UInt8 wrong = static_cast<UInt8>(st.armEpoch + 1u);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, wrong, &ack),
            "an ARM under a stale epoch is answered"
        );
        check(ack.result == 1 && has(ack.text, "epoch"), "and refused, naming the epoch");
        stream(80);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "a proper ARM is answered"
        );
        check(ack.result == 0, "and GRANTED");
        check(viewfeed::drive().armed, "and drive() says so to the tick");
        check(
            command(w, session, bibowire::Verb::VERB_ESTOP, st.armEpoch, &ack),
            "ESTOP is answered"
        );
        check(!viewfeed::drive().armed, "and the ARM is gone the moment it latches");
        stream(120);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "an ARM while latched is answered"
        );
        check(ack.result == 1 && has(ack.text, "estop"), "and refused until CLEAR_ESTOP");
        check(
            command(w, session, bibowire::Verb::VERB_CLEAR_ESTOP, st.armEpoch, &ack),
            "CLEAR_ESTOP is answered"
        );
        check(!viewfeed::drive().armed, "and it does NOT re-arm the car");
        stream(120);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "an ARM after clearing is answered"
        );
        check(ack.result == 0 && viewfeed::drive().armed, "and granted - the third step");
        check(
            command(obs, watcher, bibowire::Verb::VERB_DISARM, 0, &ack),
            "an OBSERVER's DISARM is answered"
        );
        check(
            ack.result == 0 && !viewfeed::drive().armed,
            "and it disarms - making the car safer is not a privilege"
        );
        stream(120);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "re-armed for the link test"
        );
        check(ack.result == 0, "and granted");
        bibowire::BoardState noPico = board;
        noPico.picoLink = 0u;
        viewfeed::publishBoard(noPico);
        check(settles(false, 500, true), "a BOARD saying the Pico link is down takes the ARM");
        stream(120);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "an ARM with no Pico is answered"
        );
        check(ack.result == 4, "and refused - no arm into a closed port");
        check(
            command(w, session, bibowire::Verb::VERB_DISARM, st.armEpoch, &ack),
            "a DISARM with no Pico is answered"
        );
        check(
            ack.result == 4,
            "with result 4: disarmed here, the Pico's own deadman does the rest"
        );
        viewfeed::publishBoard(board);
        sleepMs(60);
        stream(120);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "re-armed for the deadman test"
        );
        check(ack.result == 0, "and granted");
        check(settles(false, 800, false), "a stream that stops past CONTROL_DEAD_MS takes the ARM");
        stream(600);
        check(!viewfeed::drive().armed, "and the stream RESUMING does not give it back");
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "only a fresh ARM does"
        );
        check(ack.result == 0 && viewfeed::drive().armed, "and it is granted");
        bibowire::BoardState driving = board;
        driving.pilotMode = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_DRIVE);
        viewfeed::publishBoard(driving);
        stream(120);
        check(
            command(w, session, bibowire::Verb::VERB_ARM, st.armEpoch, &ack),
            "an ARM while the autonomy drives is answered"
        );
        check(ack.result == 3, "and refused - a viewer arms only a car it is driving");
        viewfeed::publishBoard(board);
        sleepMs(60);
        stream(120);
        check(viewfeed::drive().armed, "still armed before the driver leaves");
        w.close();
        check(settles(false, 1500, false), "and the driver leaving takes the ARM with the slot");
        obs.close();
        udp.close();
        viewfeed::stop();
        check(!viewfeed::drive().armed, "a stopped feed reports nothing armed");
        viewfeed::publishBoard(bibowire::BoardState());
    }
    // Two frames in one read, each answered as itself: a frame's body points into the
    // receive ring, so moving the ring down before handling it would hand the first
    // frame the second's bytes. Every other section sends one frame per send().
    {
        check(viewfeed::start(0, aPolicy()), "the feed starts for back-to-back frames");
        const UInt16 port = viewfeed::port();
        Wire w;
        check(w.connect(port), "a viewer connects");
        const UInt32 session = handshake(w, 0, 0);
        check(session != 0u, "and is welcomed");
        // Two PINGs in one buffer, with distinct tokens so a wrong echo cannot pass.
        const Array<UInt64, 2> tokens = { 0x1111111111111111ull, 0x2222222222222222ull };
        Array<UInt8, 128> both{};
        Size at = 0;
        for(Size i = 0; i < tokens.size(); ++i)
        {
            bibowire::Ping m;
            m.token = tokens[i];
            m.senderMonoUs = tokens[i];
            Array<UInt8, 32> body{};
            const Size len = bibowire::writePing(m, body.data(), body.size());
            bibowire::Head h;
            h.type = bibowire::Type::TYPE_PING;
            h.ver = 1;
            h.seq = static_cast<UInt16>(40u + i);
            bibowire::Body b;
            b.bytes = body.data();
            b.len = len;
            at += bibowire::put(h, b, both.data() + at, both.size() - at);
        }
        check(at == 64u, "both PINGs fit one 64-byte write");
        w.raw(both.data(), at);
        Vec<UInt64> echoed;
        while(echoed.size() < 2u && w.nextOf(bibowire::Type::TYPE_PONG, 1500))
        {
            bibowire::Ping p;
            if(bibowire::readPing(w.f.body, w.f.head.ver, &p))
            {
                echoed.push_back(p.token);
            }
        }
        check(echoed.size() == 2u, "both PINGs sent in one write are answered");
        check(
            echoed.size() == 2u && echoed[0] == tokens[0],
            "the FIRST is answered with its own token, not the second frame's"
        );
        check(echoed.size() == 2u && echoed[1] == tokens[1], "and the second with its own");
        w.close();
        viewfeed::stop();
    }
    // The saved trim, told on WELCOME and on every save. The second viewer connects
    // after the first was told, so only WELCOME can reach it.
    {
        check(viewfeed::start(0, aPolicy()), "the feed starts for the trim report");
        const UInt16 port = viewfeed::port();
        viewfeed::publishTrim("SERVOTRIM 1485");
        Wire first;
        check(first.connect(port), "a viewer connects");
        check(handshake(first, 0, 0) != 0u, "and is welcomed");
        Str text;
        check(nextTrim(first, text), "it is told the trim the board has saved");
        checkStr(text, "SERVOTRIM 1485", "as the Pico's own line");
        Wire later;
        check(later.connect(port), "a second viewer connects after that");
        check(handshake(later, 0, 0) != 0u, "and is welcomed");
        text.clear();
        check(
            nextTrim(later, text),
            "and is told it too, on WELCOME - not left waiting for the next save"
        );
        checkStr(text, "SERVOTRIM 1485", "the same report");
        viewfeed::publishTrim("SERVOLIMITS 1230 1660; SERVOTRIM 1490");
        check(nextTrim(first, text), "a save is told to a viewer already connected");
        checkStr(text, "SERVOLIMITS 1230 1660; SERVOTRIM 1490", "with the whole saved set");
        check(nextTrim(later, text), "and to every other viewer");
        checkStr(text, "SERVOLIMITS 1230 1660; SERVOTRIM 1490", "the same set");
        viewfeed::publishTrim("");
        check(nextTrim(first, text), "a board with nothing saved still says so");
        check(text.empty(), "as an empty report, which is an answer");
        first.close();
        later.close();
        viewfeed::stop();
    }
    // The one exception to no trim while armed: ESC limits, only while the holder's
    // stream carries BUTTON_IDLE_TEST, ending the moment the bit does.
    {
        check(viewfeed::start(0, aPolicy()), "the feed starts for the idle test");
        const UInt16 port = viewfeed::port();
        Datagram udp;
        check(udp.open(), "a viewer binds its control socket");
        Wire w;
        check(w.connect(port), "and connects");
        const UInt32 session = handshake(w, udp.port, 1);
        check(session != 0u, "and takes control");
        viewfeed::Applied ap;
        ap.armed = 1;
        viewfeed::applied(ap);
        Array<UInt8, 32> body{};
        bibowire::CtlState st;
        bibowire::CmdAck ack;
        viewfeed::Tune t;
        bibowire::Command m;
        m.sessionId = session;
        udp.control(port, session, 1, bibowire::BUTTON_ENABLE);
        check(
            udp.stateWhere(2000, &st, [](const bibowire::CtlState& s) { return s.ackSeq == 1u; }),
            "a plain enabled stream is applied"
        );
        m.cmdId = 1;
        m.verb = bibowire::Verb::VERB_SET_ESC_LIMITS;
        m.arg1 = 1550;
        m.arg2 = 1900;
        Size len = bibowire::writeCommand(m, body.data(), body.size());
        w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 40);
        check(
            w.nextOf(bibowire::Type::TYPE_CMDACK, 1000) && bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack)
                && ack.result == 3,
            "armed, ESC limits are refused"
        );
        const UInt16 idle = static_cast<UInt16>(bibowire::BUTTON_ENABLE | bibowire::BUTTON_IDLE_TEST);
        udp.control(port, session, 2, idle);
        check(
            udp.stateWhere(2000, &st, [](const bibowire::CtlState& s) { return s.ackSeq == 2u; }),
            "an idle-test stream is applied"
        );
        m.cmdId = 2;
        len = bibowire::writeCommand(m, body.data(), body.size());
        w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 41);
        check(
            w.nextOf(bibowire::Type::TYPE_CMDACK, 1000) && bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack)
                && ack.result == 0,
            "under the idle test they are taken"
        );
        check(
            viewfeed::tune(&t) && t.verb == bibowire::Verb::VERB_SET_ESC_LIMITS && t.arg1 == 1550,
            "and queued for the pilot"
        );
        m.cmdId = 3;
        m.verb = bibowire::Verb::VERB_SET_SERVO_TRIM;
        m.arg1 = 1480;
        m.arg2 = 0;
        len = bibowire::writeCommand(m, body.data(), body.size());
        w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 42);
        check(
            w.nextOf(bibowire::Type::TYPE_CMDACK, 1000) && bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack)
                && ack.result == 3,
            "but nothing else is - the steering trim stays locked while armed"
        );
        check(!viewfeed::tune(&t), "and nothing else was queued");
        udp.control(port, session, 3, bibowire::BUTTON_ENABLE);
        check(
            udp.stateWhere(2000, &st, [](const bibowire::CtlState& s) { return s.ackSeq == 3u; }),
            "the stream drops the idle test"
        );
        m.cmdId = 4;
        m.verb = bibowire::Verb::VERB_SET_ESC_LIMITS;
        m.arg1 = 1560;
        m.arg2 = 1900;
        len = bibowire::writeCommand(m, body.data(), body.size());
        w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, 43);
        check(
            w.nextOf(bibowire::Type::TYPE_CMDACK, 1000) && bibowire::readCmdAck(w.f.body, w.f.head.ver, &ack)
                && ack.result == 3,
            "and with it gone, ESC limits are refused again"
        );
        w.close();
        udp.close();
        viewfeed::stop();
    }
    // A car program's run publishes LOOK (a dry run) or DRIVE. Its trim is not a
    // viewer's to change, and it arms through its own Car, not a viewer's epoch, so a
    // DISARM reaches it only through the estop latch.
    {
        check(viewfeed::start(0, aPolicy()), "the feed starts for a program's run");
        const UInt16 port = viewfeed::port();
        bibowire::BoardState board;
        board.pilotMode = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_DRIVE);
        board.picoLink = 1u;
        viewfeed::publishBoard(board);
        sleepMs(60);
        Wire w;
        check(w.connect(port), "a viewer connects to a program's run");
        const UInt32 session = handshake(w, 0, 0);
        check(session != 0u, "and is welcomed");
        Array<UInt8, 32> body{};
        UInt32 cmdId = 0;
        const auto command = [&](bibowire::Verb verb, UInt16 arg1, bibowire::CmdAck* ack) {
            bibowire::Command m;
            m.sessionId = session;
            m.cmdId = ++cmdId;
            m.verb = verb;
            m.arg1 = arg1;
            const Size len = bibowire::writeCommand(m, body.data(), body.size());
            w.put(bibowire::Type::TYPE_COMMAND, body.data(), len, static_cast<UInt16>(cmdId));
            return w.nextOf(bibowire::Type::TYPE_CMDACK, 1000)
                && bibowire::readCmdAck(w.f.body, w.f.head.ver, ack)
                && ack->cmdId == m.cmdId;
        };
        const auto says = [](const bibowire::CmdAck& ack, CharSeq word) {
            return ack.text.find(word) != Str::npos;
        };
        // The idle-test section left the car reported armed; start() does not clear it.
        viewfeed::applied(viewfeed::Applied());
        // A Car ends its run when this copy is older than VIEWER_STUCK_MS, so a
        // loop that is only polling must never let it get that old.
        Int32 oldest = 0;
        Bool aged = false;
        for(Int32 i = 0; i < 40; ++i)
        {
            const Int32 age = viewfeed::drive().seenAgeMs;
            oldest = age > oldest ? age : oldest;
            aged = aged || age > 0;
            sleepMs(7);
        }
        check(oldest < bibo::VIEWER_STUCK_MS, "drive()'s copy stays younger than VIEWER_STUCK_MS");
        check(aged, "and its age is measured, not left at 0");
        using Verb = bibowire::Verb;
        bibowire::CmdAck ack;
        viewfeed::Tune t;
        const UInt8 look = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_LOOK);
        const UInt8 manual = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL);
        check(command(Verb::VERB_SET_SERVO_TRIM, 1487, &ack), "a program's trim is answered");
        check(ack.result == 3 && says(ack, "manual"), "and refused, naming manual");
        check(!viewfeed::tune(&t), "and nothing was queued");
        check(!viewfeed::drive().estopLatched, "no estop before the DISARM");
        check(command(Verb::VERB_DISARM, 0, &ack), "a program's DISARM is answered");
        check(ack.result == 0 && says(ack, "estop"), "and says it latched the estop");
        check(viewfeed::drive().estopLatched, "which drive() hands the program");
        check(command(Verb::VERB_CLEAR_ESTOP, 0, &ack), "CLEAR_ESTOP is answered");
        check(!viewfeed::drive().estopLatched, "and releases it");
        board.pilotMode = look;
        viewfeed::publishBoard(board);
        sleepMs(60);
        check(command(Verb::VERB_SET_SLEW, 40, &ack), "a dry run's trim is answered");
        check(ack.result == 3 && !viewfeed::tune(&t), "and refused too, with nothing queued");
        check(command(Verb::VERB_DISARM, 0, &ack), "a dry run's DISARM is answered");
        check(viewfeed::drive().estopLatched, "and latches the estop too");
        check(command(Verb::VERB_CLEAR_ESTOP, 0, &ack), "CLEAR_ESTOP is answered again");
        board.pilotMode = manual;
        viewfeed::publishBoard(board);
        sleepMs(60);
        check(command(Verb::VERB_DISARM, 0, &ack), "in manual a DISARM is answered");
        check(ack.result == 0 && !viewfeed::drive().estopLatched, "as before: no estop latched");
        check(command(Verb::VERB_SET_SERVO_TRIM, 1487, &ack), "in manual, trim is answered");
        check(ack.result == 0, "and taken");
        check(viewfeed::tune(&t) && t.arg1 == 1487, "and queued for the pilot");
        w.close();
        viewfeed::stop();
    }
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

#else

Int32 main()
{
    std::printf("\nviewfeed - bibowire's socket half\n\n");
    viewfeed::Policy policy;
    check(!viewfeed::start(0, policy), "start() refuses on a platform with no sockets");
    check(viewfeed::clients() == 0 && viewfeed::port() == 0, "and there is nobody connected");
    viewfeed::publishScan(bibowire::Scan());
    viewfeed::publishDecide(bibowire::Decide());
    viewfeed::publishBoard(bibowire::BoardState());
    bibowire::Control ignored;
    check(!viewfeed::control(&ignored), "and no control to read");
    viewfeed::stop();
    check(true, "publish() and stop() are harmless with nothing running");
    checkStr(Str(), "", "(the socket tests run on Linux only - see the header)");
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

#endif
