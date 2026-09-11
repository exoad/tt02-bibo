// bibowire's socket half, held to viewfeed.hxx over real sockets.
//
//   ctest --test-dir build-pilot -R viewfeed     g++ on Linux, loopback
//
// NOT IN firmware\verify.bat, AND NOT IN A .bat AT ALL: viewfeed.cxx's real
// half is Linux-only - accept4, pipe2, poll, recvfrom, MSG_NOSIGNAL - and on
// MSVC every function refuses. A laptop run could prove nothing but that
// start() returns false, which is what the #else half below checks and all it
// claims to check. docs/bibowire.md section 11 says this out loud rather than
// leaving it to be discovered, and pilot/CMakeLists.txt keeps it a ctest.
//
// WHAT IS CHECKED, in order. Cases 35-40 are section 11's own list for this
// file; the rest are the ones writing it turned up.
//
//   1.  start/stop, the port, and publish() with nobody connected.
//   1b. THE BOARD'S OWN AGES. scanAgeMs, controlAgeMs and picoSilentMs are
//       measurements, and with nothing measured yet they carry a real elapsed
//       time or their ABSENT sentinel - never 0, which would read as
//       "perfectly fresh" and is the most dangerous value on this wire.
//   2.  The handshake: HELLO is answered by WELCOME, then LIDAR_INFO, then
//       BOARD, then the first SCAN. STATE BEFORE SCAN, ALWAYS - the ordering
//       the viewer needs to have something true to draw the moment a picture
//       appears.
//   3.  A person with nc: a connection whose first bytes are not the magic
//       gets one plain ASCII line before its BYE.
//   4.  (37) An inbound frame claiming more than MAX_INBOUND_PAYLOAD is closed
//       with BYE(TOO_BIG), and nothing is allocated for the claim.
//   5.  A second HELLO on a live connection closes it.
//   6.  (38) A second viewer asking for control is accepted as an OBSERVER
//       with refusal 2, and its CONTROL datagrams are counted and discarded.
//   7.  (39) A datagram carrying the PREVIOUS session's id is rejected after a
//       reconnect - the case that would otherwise drive the car with a
//       second-old stick position while the socket looked perfect.
//   8.  (40) The reverse-path probe says so in words when no CONTROL datagram
//       arrives within 1000 ms of WELCOME.
//   9.  (35) A client that stops reading is coalesced to ONE queued SCAN and
//       then dropped, WHILE ANOTHER KEEPS RECEIVING - the property the pilot's
//       tick depends on, and the one a shared queue would break.
//   10. (36) The drop ORDER: BULK before LIVE before VITAL, and a client the
//       vital frames cannot reach is closed rather than waited for.
//   11. A fifth viewer is refused by name, with the four already connected in
//       the sentence.
//   12. PING/PONG: the board pings once a second, and a viewer that never
//       answers is closed with BYE(TIMEOUT).
//   13. LEAVE releases the control slot on that tick.
//   14. THE ZERO-MASK TRAP. `typeMask = 0` means "never asked", which this
//       board reads as EVERYTHING - and CAMERA is the one type excluded from
//       that default, because ~45 KB a frame handed to a viewer that never
//       mentioned it would take the bandwidth the scan needs.
//   15. Asking for the camera is ANSWERED - a picture, or a sentence saying
//       why there is none. Never a blank panel and silence.
//
// WHAT THIS SUITE DOES NOT PROVE, said out loud rather than left to be assumed.
// /dev/video0 is SINGLE-OPENER - measured: a second streamer gets
// "VIDIOC_REQBUFS returned -1 (Device or resource busy)" and writes zero bytes
// - and the phone dashboard (tools/status/status_server.py) opens the same
// device. A ctest that grabbed it would fight the dashboard on the very board
// it runs on and would pass or fail depending on whether somebody had a tab
// open. So BIBO_CAM_DEV is pointed at a device that does not exist for the
// whole run, and what is checked is the BOARD's behaviour: the subscription
// gate, and that an absence arrives with a reason. That a JPEG actually comes
// off the sensor and reaches a viewer is NOT checked here.
//
// It can be, by hand, against real hardware - the setenv below does not
// overwrite, so
//
//   BIBO_CAM_DEV=/dev/video0 ./test_viewfeed
//
// runs this same suite against the real device, where check 15 accepts either
// the picture or the reason.
//
// Every socket read here has a deadline, so a feed that sends nothing fails the
// check rather than hanging the test.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"

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

// A bibowire client on loopback, with every read under a deadline. It speaks
// the protocol through the SAME codec the board does, so a disagreement about
// a field's offset cannot hide inside this file.
struct Wire
{
    Int32 fd = -1;
    Vec<UInt8> in;
    Vec<UInt8> held;        // the frame most recently taken, kept alive
    bibowire::Frame f;      // points INTO `held`

    // `rcvBuf` non-zero caps this end's receive buffer, BEFORE connect so the
    // window scale is negotiated with it. That is what makes backpressure a
    // thing this test can create on demand: loopback otherwise autotunes to
    // megabytes and swallows everything a stalled viewer is sent.
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

    // The next frame of `type`, skipping anything else that arrives first -
    // PING and CTLSTATE turn up on their own schedule and are not what a
    // particular check is asking about.
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

    Void control(UInt16 to, UInt32 session, UInt32 seq, UInt16 buttons)
    {
        bibowire::Control m;
        m.sessionId = session;
        m.seq = seq;
        m.buttons = buttons;
        Array<UInt8, 64> body{};
        const Size len = bibowire::writeControl(m, body.data(), body.size());
        Array<UInt8, 128> frame{};
        bibowire::Head h;
        h.type = bibowire::Type::TYPE_CONTROL;
        h.ver = 1;
        h.seq = static_cast<UInt16>(seq);
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

    // Drains the socket to EMPTY and keeps the NEWEST CTLSTATE in it.
    //
    // THIS IS THE RULE THE BOARD ITSELF APPLIES TO CONTROL, and this suite got
    // it wrong first time in exactly the way the protocol warns about. CTLSTATE
    // arrives at 20 Hz, so by the time a check runs there is a queue of them;
    // a reader that returns the FIRST is reading the past, and five assertions
    // failed on the board asserting values the board had long since moved on
    // from. A stale datagram read as current is this protocol's whole subject.
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

    // Keeps draining until the newest CTLSTATE satisfies `ok`, or `ms` passes.
    // The board applies a CONTROL on its own thread, so a check that reads once
    // and asserts is racing it; this waits for the state to arrive rather than
    // sleeping a guessed interval and hoping.
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

Int32 main()
{
    std::printf("\nviewfeed - bibowire's socket half, over loopback\n\n");

    // The camera device, pointed somewhere that does not exist for the whole
    // run - see "WHAT THIS SUITE DOES NOT PROVE" in the header for why, and for
    // how to point it at the real one by hand. Overwrite is 0 on purpose: an
    // operator who sets BIBO_CAM_DEV wins, and gets the real device.
    static_cast<Void>(::setenv("BIBO_CAM_DEV", "/dev/bibo-no-such-video", 0));

    // ---- 1. start, stop, and a publish with nobody there ---------------------------
    const viewfeed::Policy policy = aPolicy();
    check(viewfeed::start(0, policy), "the feed starts on an ephemeral port");
    check(viewfeed::port() != 0, "and says which");
    check(!viewfeed::start(0, policy), "a second start() without a stop() is refused");
    check(viewfeed::clients() == 0, "no viewers yet");
    viewfeed::publishScan(aScan(100, 1));
    viewfeed::publishDecide(bibowire::Decide());
    check(true, "publish() with nobody connected is harmless");

    const UInt16 port = viewfeed::port();

    // ---- 1b. the board's own ages, MEASURED and never a placeholder ------------------
    // A field left at 0 reads as PERFECTLY FRESH, which is the most dangerous
    // value this protocol can carry: it turns "no revolution has ever arrived"
    // into "the picture in front of you is current". scanAgeMs, controlAgeMs
    // and picoSilentMs are the board measuring ITSELF, and this checks they say
    // so BEFORE anything has happened rather than only once it is all working.
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

        // And once a revolution HAS reached the board the age becomes a real,
        // small one - the field is a measurement in both directions.
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

    // ---- 2. the handshake, and STATE BEFORE SCAN ------------------------------------
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

        // Bits 0 and 1 are MAINTAINED on every board->viewer frame, so "the car
        // is stopped" is derivable from any frame that arrives - including one
        // whose body a reader has no name for. Maintained is not the same as
        // set: with NOBODY holding the control slot the deadman does not apply
        // at all (section 6), so the honest value of the bit here is CLEAR.
        // A board that set it anyway would be reporting a stop that is not
        // happening, which is the same lie in the other direction.
        check(
            (a.f.head.flags & bibowire::FLAG_DEADMAN) == 0u,
            "with no holder the deadman does not apply, and FLAG_DEADMAN is clear"
        );
        check((a.f.head.flags & bibowire::FLAG_ESTOP) == 0u, "and no e-stop is latched");

        a.close();
        check(clientsReach(0, 2000), "and the count goes back to nothing when it leaves");
    }

    // ---- 2b. an OBSERVER is a first-class viewer ---------------------------------------
    // wantControl = 0 is the common case today: control is out of scope until
    // the Pico is wired, so the viewer connects to watch. Section 6 line 631 is
    // explicit that with nobody holding the slot bibowire's deadman does not
    // apply and the pilot runs under its own rules - so an observer must not
    // arm a timer that would stop a car nobody is driving.
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

        // The featureMask convention, pinned so the next reader is not left
        // guessing as the viewer's author was.
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

        // And it is served the telemetry, which is the whole reason it is here.
        viewfeed::publishScan(aScan(200, 900));
        check(watcher.nextOf(bibowire::Type::TYPE_SCAN, 2000), "and it is served revolutions");

        watcher.close();
        udp.close();
        check(clientsReach(0, 2000), "the observer leaves");
    }

    // ---- 2c. a HELLO with every feature bit set is ACCEPTED ------------------------------
    // The value a viewer sends when the spec named no bit for it. Refusing an
    // unrecognised bit would refuse the most sensible thing a viewer can say.
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

    // ---- 3. a person with nc gets a sentence, not a hex dump -------------------------
    {
        Wire nc;
        check(nc.connect(port), "somebody dials the binary port by hand");
        const Str typed = "hello?\n";
        nc.raw(reinterpret_cast<const UInt8*>(typed.data()), typed.size());
        const Str said = nc.asciiLine(1000);
        check(said.find("bibowire v1 binary on 8020") != Str::npos, "and is answered in words");
        check(said.find("8011") != Str::npos, "naming where the text feed is");
        check(nc.closed(2000), "then closed");
        nc.close();
    }

    // ---- 4. (37) an oversized claim is refused without allocating for it -------------
    {
        Wire big;
        check(big.connect(port), "a viewer connects");
        // A header claiming 200000 payload bytes. It is a CLAIM, and the board
        // must answer it from the header alone - the frame it describes could
        // never fit the 1024-byte receive ring, so a board that waited for the
        // body would wait forever.
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

    // ---- 5. a second HELLO on a live connection is a protocol error ------------------
    {
        Wire twice;
        check(twice.connect(port), "a viewer connects");
        check(handshake(twice, 0, 0) != 0u, "and is welcomed");
        twice.hello(0, 0, "again");
        check(twice.closed(2000), "a second HELLO closes it");
        twice.close();
        check(clientsReach(0, 2000), "and it is no longer counted");
    }

    // ---- 6. (38) the second asker drives nothing -------------------------------------
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

        // The holder's stream is applied and feeds the deadman.
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

        // The observer's are counted and discarded, and above all do not feed
        // the timer. This is Design 3's fatal flaw, pinned by a test.
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

        // ---- 13. LEAVE releases the slot on that tick -------------------------------
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

    // ---- 7. (39) a datagram from the PREVIOUS session ---------------------------------
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

        // The datagram still in flight from before the blip. Its seq is higher
        // than anything this session has applied, so seq alone would let it
        // through; the session id is what stops it driving the car with a
        // second-old stick position.
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

        // And the high-water mark was reset by the handshake, so a viewer that
        // restarts at seq 1 is heard rather than frozen out.
        udp.control(port, fresh, 1, bibowire::BUTTON_ENABLE);
        const Bool reset = udp.stateWhere(2000, &st, [](const bibowire::CtlState& s) {
            return s.ackSeq == 1u;
        });
        check(reset, "the new session's seq 1 is applied - the mark is reset per session");

        again.close();
        udp.close();
        check(clientsReach(0, 2000), "the viewer leaves");
    }

    // ---- 8. (40) the reverse path is MEASURED, not assumed ----------------------------
    {
        Wire quiet;
        check(quiet.connect(port), "a viewer asks for control");
        // It names a UDP port and then never sends a datagram - the asymmetric
        // failure the two-transport shape creates, and the one a viewer cannot
        // diagnose on its own.
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

    // ---- 9. (35) one viewer stalls, the other keeps seeing the car ---------------------
    //
    // THE FIRST VERSION OF THIS TEST PROVED NOTHING, and how it failed is worth
    // keeping. It published 2.5 MB at a viewer that never read, expecting the
    // ring to coalesce - and loopback autotuned its buffers into the megabytes
    // and swallowed every byte. The socket never pushed back, so the ring never
    // filled, nothing was ever coalesced, droppedSinceLast stayed 0, and the
    // stalled viewer was finally closed by the PING timeout at twelve seconds
    // rather than by backpressure at five hundred milliseconds. Three
    // assertions were green about a mechanism that had never once run.
    //
    // A small SO_RCVBUF set BEFORE connect is what makes the condition real:
    // one 1024-point revolution is 5160 bytes and will not fit, so the board is
    // pushing back within milliseconds and the drop classes have to decide.
    {
        constexpr Int32 TINY_RCVBUF = 2048;

        // ---- 9a. a viewer that falls behind is COALESCED to the newest ----------
        Wire lag;
        check(lag.connect(port, TINY_RCVBUF), "a viewer with a tiny receive buffer connects");
        check(handshake(lag, 0, 0) != 0u, "and is welcomed");
        check(clientsReach(1, 2000), "and is counted");

        // A burst it cannot absorb, with no reads at all. Kept short so this
        // finishes well inside the first PING, whose own vital frame would
        // otherwise close the client on BEHIND_MS before it can be read from.
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

        // ---- 9b. one stalls and is dropped, the other keeps receiving -----------
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
        // The PING timeout cannot fire before 1000 + 4000 ms, so a drop sooner
        // than that is BACKPRESSURE and not the half-open check doing this
        // check's job by accident. Telling those two apart is the whole point:
        // the first version could not, and passed anyway.
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

    // ---- 10. (36) the drop order, and the class ranks it comes from ---------------------
    // The ordering itself is bibowire's: BULK is discarded before LIVE, and
    // VITAL is never discarded at all. v1 has no BULK producer - CAMERA is
    // reserved and nothing publishes one - so the socket half cannot be made to
    // queue one from out here; what IS checked over the socket is the half that
    // has a producer, which is that LIVE coalesces (above) and that a client the
    // vital frames cannot reach is CLOSED rather than waited for (above).
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

    // ---- 14. THE ZERO-MASK TRAP -----------------------------------------------------
    //
    // A zero typeMask means "never asked", and this board reads that as
    // EVERYTHING: a viewer that never subscribes is not a viewer that wants
    // nothing. That is right for a 2.5 KB revolution and it would be a disaster
    // for a camera - 45 KB a frame, measured, at whatever rate the device runs.
    // EVERY viewer written before the producer existed sends a zero mask, so if
    // CAMERA sat in that default they would all start receiving a megabyte a
    // second they never asked for, out of the bandwidth the scan needs.
    //
    // So CAMERA is the one type excluded from the default, and this is the check
    // that pins it. The mask convention is `bit = tag - 0x10`, so CAMERA (0x20)
    // is bit 16.
    {
        const UInt32 cameraBit = 1u << (0x20u - 0x10u);
        check(cameraBit == (1u << 16u), "CAMERA's subscription bit is bit 16");

        Wire zero;
        check(zero.connect(port), "a viewer that never subscribes connects");
        check(handshake(zero, 0, 0) != 0u, "and is welcomed");
        check(clientsReach(1, 2000), "and is counted");

        // It IS served what a zero mask includes, so this is a check about the
        // camera and not about a viewer that is being sent nothing at all.
        viewfeed::publishScan(aScan(64, 910));
        check(zero.nextOf(bibowire::Type::TYPE_SCAN, 2000), "a zero mask still receives SCAN");
        bibowire::BoardState state;
        state.upS = 21;
        viewfeed::publishBoard(state);
        check(zero.nextOf(bibowire::Type::TYPE_BOARD, 2000), "and still receives BOARD");

        // And NOT the camera. Long enough that a capture would have been
        // started, failed and explained several times over if the mask let it.
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

    // ---- 15. asking for the camera is ANSWERED --------------------------------------
    //
    // A picture, or a sentence saying why there is none. Never a blank panel and
    // silence: an absence with a reason beats a silent nothing, and this repo is
    // named after the failure of reporting success while measuring nothing.
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
        // featureMask is "what this board WILL send", so the camera belongs in
        // it even though it is off: it is how a viewer discovers the bit is
        // worth setting at all, rather than having to read this source.
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

    // ---- 11. a fifth viewer is refused BY NAME -------------------------------------------
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

    // ---- 12. PING, and a viewer that never answers ------------------------------------
    {
        Wire ponger;
        check(ponger.connect(port), "a viewer connects");
        check(handshake(ponger, 0, 0) != 0u, "and is welcomed");
        check(ponger.nextOf(bibowire::Type::TYPE_PING, 2500), "the board pings it");
        bibowire::Ping p;
        check(bibowire::readPing(ponger.f.body, ponger.f.head.ver, &p), "the PING reads whole");

        // Answered, so it stays. The 4000 ms timeout is the other half, and it
        // is checked below on a viewer that says nothing at all.
        Array<UInt8, 32> body{};
        const Size len = bibowire::writePing(p, body.data(), body.size());
        ponger.put(bibowire::Type::TYPE_PONG, body.data(), len, 1);

        // The other direction, and the field the viewer's latency readout is
        // built on: the token must come back BYTE FOR BYTE. A board that
        // regenerated it would leave the viewer measuring nothing while its
        // display filled with plausible milliseconds.
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

        // And one that does not: BYE(TIMEOUT) inside 4000 ms of the PING going
        // out. A phone that walks out of range stops ACKing without a FIN, and
        // nothing but this notices.
        const TimePoint waited = monoNow();
        const Bool timedOut = ponger.closed(7000);
        check(timedOut, "a viewer that stops answering PING is closed");
        std::printf("        (closed %.0f ms after it went quiet)\n", elapsedMs(waited));
        ponger.close();
        check(clientsReach(0, 2000), "and it is gone");
    }

    // ---- stop() -----------------------------------------------------------------------
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
