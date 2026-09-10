#include "link.hxx"

// Winsock before anything that might drag in <windows.h>: winsock2.h and the
// original winsock.h define the same symbols, and the loser is whichever one
// arrives second. Nothing above this line includes a Windows header - link.hxx
// is the vocabulary, the codec and the scene, none of which know what a socket
// is - so this is the first and the order is safe.
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>

// <winnt.h>, underneath those, defines SEVERITY_ERROR as a macro - and
// bibowire::Severity has a member of that name. link.hxx is included above, so
// the enum is already declared and this file compiles either way; the undef is
// here so that the day somebody writes SEVERITY_ERROR below this line, it means
// what it says rather than expanding to 1.
#undef SEVERITY_SUCCESS
#undef SEVERITY_ERROR

#include <cmath>
#include <cstdio>
#include <cstring>

namespace link
{

  namespace
  {

    // The lidar's height above the ground, matching what the stand-in cloud
    // this replaces was drawn at. NOT a mounting correction: docs/conventions.md
    // records the lidar-to-vehicle transform as ASSUMED rather than measured, so
    // rotating or offsetting the cloud here would bake a guess into every point
    // and every sensor fused with it afterwards - and it would look like a
    // sensor fault rather than a bad constant. When the transform is measured,
    // it applies here, once.
    constexpr Float32 LIDAR_HEIGHT_M = 0.16f;

    // Centi-degrees to radians. The wire has no floating point on it at all, so
    // this multiply is the first and only place a scan becomes inexact.
    constexpr Float32 CENTI_DEG_TO_RAD = 3.14159265358979f / 18000.0f;

    constexpr Float32 MM_TO_M = 0.001f;

    // Section 3: the viewer keeps a 512 KiB receive ring. One allocation at
    // connect, reused until the socket closes; nothing in the receive path ever
    // sizes an allocation from a number a stranger on a hotspot wrote.
    // Size{512}, not 512u: the product is worked out in the type of its
    // operands and only then widened, so a 32-bit multiply that overflows has
    // already lost the bits by the time it becomes a Size. Half a megabyte is
    // nowhere near that, and this line is not the bug - it is the SHAPE of the
    // bug, and the shape is what gets copied to the place that does overflow.
    constexpr Size RX_BYTES = Size{512} * 1024;

    // Section 2: viewer SO_RCVBUF 256 KiB.
    constexpr Int32 RCVBUF_BYTES = 256 * 1024;

    // How long the worker blocks in one select(). It bounds how long a
    // Disconnect waits, and at 20 Hz it is far below the 50 ms cadence of the
    // fastest thing on the wire.
    constexpr Int32 POLL_MS = 20;

    // A frame this viewer SENDS. HELLO with a 31-byte name is 72 bytes and PONG
    // is 32; MAX_INBOUND_PAYLOAD is 256, so nothing the board will accept from us
    // comes close to this.
    constexpr Size TX_BYTES = 512;

    // The keepalive of section 2, in the shape Windows takes it. TCP_KEEPCNT has
    // no Windows equivalent - the count is fixed at 10 - so the two numbers that
    // can be set are set and the third is written down rather than pretended.
    constexpr UInt32 KEEPALIVE_IDLE_MS = 2000;
    constexpr UInt32 KEEPALIVE_INTERVAL_MS = 1000;

    // How long a send may spend waiting for a writable socket before the
    // connection is declared broken. Everything this viewer sends is one small
    // frame, so a socket that cannot take 72 bytes in half a second is not slow,
    // it is gone.
    constexpr Int64 SEND_BUDGET_MS = 500;

    // ---- ages ---------------------------------------------------------------

    // A negative age is a bug upstream, and the safe reading of a bug is "old".
    // Treating it as freshness is how a sign error becomes a stale picture drawn
    // as live - the same rule bibowire::deadman applies to nowMs < lastControlMs.
    [[nodiscard]] Int64 ageFrom(Int64 nowMs, Int64 atMs)
    {
        const Int64 age = nowMs - atMs;
        return age < 0 ? (GONE_MS + 1) : age;
    }

    // The larger of two ages: the one measured from local arrival, and the one
    // measured through the board's own clock. See Session::offsetMs - the offset
    // can only ever be corrected toward "older", by the fact that the bytes have
    // not arrived yet.
    [[nodiscard]] Int64 ageOf(const Session& s, Int64 nowMs, Int64 atMs, UInt64 boardUs)
    {
        const Int64 local = ageFrom(nowMs, atMs);
        if(!s.haveOffset || boardUs == 0u)
        {
            return local;
        }
        const Int64 boardMs = static_cast<Int64>(boardUs / 1000u);
        const Int64 through = nowMs - (boardMs + s.offsetMs);
        return through > local ? through : local;
    }

    Void noteOffset(Session& s, Int64 nowMs, UInt64 boardUs)
    {
        if(boardUs == 0u)
        {
            return;
        }
        const Int64 boardMs = static_cast<Int64>(boardUs / 1000u);
        const Int64 offset = nowMs - boardMs;
        // The MINIMUM, because on a hotspot the mean is dominated by stalls and
        // the smallest sample is the one that travelled closest to the true path.
        if(!s.haveOffset || offset < s.offsetMs)
        {
            s.haveOffset = true;
            s.offsetMs = offset;
        }
    }

    [[nodiscard]] const RttSample* bestSample(const Session& s)
    {
        if(s.rtts.empty())
        {
            return nullptr;
        }
        const RttSample* best = &s.rtts[0];
        for(const RttSample& r : s.rtts)
        {
            if(r.rttMs < best->rttMs)
            {
                best = &r;
            }
        }
        return best;
    }

    // A round trip is the only measurement here that can separate the clock
    // offset from the transit time, so it gives the better offset: the board
    // stamped its PONG at boardUs, that stamp took half the round trip to get
    // here, and the difference is the offset. Computed from the FASTEST of the
    // last sixteen, because that is the sample least polluted by a stall.
    //
    // Folded into the same minimum as the arrival floor, and only ever downward.
    // A smaller offset yields a LARGER age, so this can make the picture look
    // older than local arrival suggested and never younger - which is the whole
    // of "the one place the design deliberately distrusts its own cleverness".
    Void noteRttOffset(Session& s)
    {
        const RttSample* best = bestSample(s);
        if(best == nullptr || best->boardUs == 0u)
        {
            return;
        }
        const Int64 boardMs = static_cast<Int64>(best->boardUs / 1000u);
        const Int64 candidate = best->atMs - boardMs - (best->rttMs / 2);
        if(!s.haveOffset || candidate < s.offsetMs)
        {
            s.haveOffset = true;
            s.offsetMs = candidate;
        }
    }

    // ---- text ---------------------------------------------------------------

    // Integer digits, never printf's %.1f: the decimal point honours the locale,
    // a machine set to a comma decimal writes "3,2", and that is the bug
    // proto.cxx and scanwire.cxx both already met. Nothing on this wire is a
    // float and nothing this module prints from it becomes one.
    [[nodiscard]] Str secondsText(Int64 ms)
    {
        Array<Char, 32> t = {};
        const Int64 whole = ms / 1000;
        const Int64 tenth = (ms % 1000) / 100;
        std::snprintf(t.data(), t.size(), "%lld.%lld s", whole, tenth);
        return Str(t.data());
    }

    [[nodiscard]] Str numberText(Int64 v)
    {
        Array<Char, 32> t = {};
        std::snprintf(t.data(), t.size(), "%lld", v);
        return Str(t.data());
    }

    // Does this sentence talk about the camera?
    //
    // A HEURISTIC, and deliberately a visible one. EVENT carries a `code`
    // byte, but bibowire defines no code for the camera anywhere - not in the
    // document, not in the header, not in its 312 checks - so there is nothing
    // structured to match on and the alternative is showing an empty window
    // beside a note list the operator has to read for themselves.
    //
    // ASCII by construction: section 5 says EVENT text is ASCII, so this
    // comparison has no locale in it and does not call std::tolower, whose
    // answer depends on one.
    [[nodiscard]] Bool mentionsCamera(const Str& text)
    {
        Str lower;
        lower.reserve(text.size());
        for(const Char c : text)
        {
            const Bool upper = c >= 'A' && c <= 'Z';
            lower.push_back(upper ? static_cast<Char>(c - 'A' + 'a') : c);
        }
        return lower.find("camera") != Str::npos;
    }

    // ---- the connection -----------------------------------------------------

    // SOCKET is UINT_PTR and INVALID_SOCKET is ~0, so these stay out of link.hxx
    // and every file that draws a panel is spared <winsock2.h>.
    struct Conn
    {
        SOCKET tcp = INVALID_SOCKET;
        SOCKET udp = INVALID_SOCKET;
        UInt16 udpPort = 0;
        UInt16 txSeq = 0;
        UInt64 pingToken = 0;
        Bool udpFiltered = false;
        Vec<UInt8> rx;
        Size rxUsed = 0;

        // What THIS connection has told the board it wants. 0 means "never
        // asked", which is not the same fact as "asked for nothing" - see
        // syncSubscription. Reset with the connection, because the board keeps
        // no subscription across a session either.
        UInt32 sentMask = 0;
    };

    Void dropConn(Conn& c)
    {
        if(c.tcp != INVALID_SOCKET)
        {
            ::closesocket(c.tcp);
            c.tcp = INVALID_SOCKET;
        }
        if(c.udp != INVALID_SOCKET)
        {
            ::closesocket(c.udp);
            c.udp = INVALID_SOCKET;
        }
        c.rxUsed = 0;
        c.txSeq = 0;
        c.udpPort = 0;
        c.udpFiltered = false;
        c.sentMask = 0;
    }

    Void setNonBlocking(SOCKET fd)
    {
        u_long mode = 1;
        ::ioctlsocket(fd, FIONBIO, &mode);
    }

    [[nodiscard]] Bool waitWritable(SOCKET fd, Int64 budgetMs)
    {
        fd_set writes;
        fd_set fails;
        FD_ZERO(&writes);
        FD_ZERO(&fails);
        FD_SET(fd, &writes);
        FD_SET(fd, &fails);

        timeval tv;
        tv.tv_sec = static_cast<long>(budgetMs / 1000);
        tv.tv_usec = static_cast<long>((budgetMs % 1000) * 1000);

        const Int32 ready = ::select(0, nullptr, &writes, &fails, &tv);
        if(ready <= 0 || FD_ISSET(fd, &fails))
        {
            return false;
        }
        return FD_ISSET(fd, &writes) != 0;
    }

    [[nodiscard]] Bool sendAll(SOCKET fd, const UInt8* buf, Size len)
    {
        Size sent = 0;
        const Int64 deadline = monoMs() + SEND_BUDGET_MS;
        while(sent < len)
        {
            const Char* at = reinterpret_cast<const Char*>(buf + sent);
            const Int32 want = static_cast<Int32>(len - sent);
            const Int32 n = ::send(fd, at, want, 0);
            if(n > 0)
            {
                sent += static_cast<Size>(n);
                continue;
            }
            if(n == 0 || ::WSAGetLastError() != WSAEWOULDBLOCK)
            {
                return false;
            }
            const Int64 left = deadline - monoMs();
            if(left <= 0 || !waitWritable(fd, left))
            {
                return false;
            }
        }
        return true;
    }

    // One typed frame out. The header is assembled HERE and nowhere else, so no
    // call site ever picks its own seq - the per-connection counter is the
    // frame-header ring's and the loss accounting's, and two writers would make
    // it neither.
    [[nodiscard]] Bool sendFrame(Conn& c, bibowire::Type type, const UInt8* body, Size bodyLen)
    {
        bibowire::Head head;
        head.type = type;
        head.ver = 1;
        head.seq = c.txSeq;

        Array<UInt8, TX_BYTES> frame = {};
        const bibowire::Body payload = { body, bodyLen };
        const Size n = bibowire::put(head, payload, frame.data(), frame.size());
        if(n == 0)
        {
            return false;
        }
        ++c.txSeq;
        return sendAll(c.tcp, frame.data(), n);
    }

    // ---- dialling -----------------------------------------------------------

    [[nodiscard]] Bool finishConnect(SOCKET fd, Int64 budgetMs)
    {
        if(!waitWritable(fd, budgetMs))
        {
            return false;
        }
        // Writable is not the same fact as connected: a refused connection is
        // reported by SO_ERROR, and a socket that skipped this check would send
        // HELLO into a connection that never happened.
        Int32 err = 0;
        Int32 len = static_cast<Int32>(sizeof(err));
        Char* slot = reinterpret_cast<Char*>(&err);
        if(::getsockopt(fd, SOL_SOCKET, SO_ERROR, slot, &len) != 0)
        {
            return false;
        }
        return err == 0;
    }

    Void tuneTcp(SOCKET fd)
    {
        Int32 on = 1;
        const Char* onBytes = reinterpret_cast<const Char*>(&on);
        // A 40-byte CMDACK must not sit in Nagle's queue behind the 2540-byte
        // SCAN it follows.
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, onBytes, static_cast<Int32>(sizeof(on)));
        ::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, onBytes, static_cast<Int32>(sizeof(on)));

        Int32 rcvbuf = RCVBUF_BYTES;
        const Char* rcvBytes = reinterpret_cast<const Char*>(&rcvbuf);
        ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, rcvBytes, static_cast<Int32>(sizeof(rcvbuf)));

        // A phone that walks out of range stops ACKing without ever sending a
        // FIN, and the default keepalive is two hours. Windows fixes the probe
        // COUNT at 10 and offers no TCP_KEEPCNT, so the two numbers that can be
        // set are set; the viewer's own 3000 ms silence redial is what actually
        // catches this case first anyway.
        tcp_keepalive ka = {};
        ka.onoff = 1;
        ka.keepalivetime = KEEPALIVE_IDLE_MS;
        ka.keepaliveinterval = KEEPALIVE_INTERVAL_MS;
        DWORD got = 0;
        ::WSAIoctl(
            fd,
            SIO_KEEPALIVE_VALS,
            &ka,
            static_cast<DWORD>(sizeof(ka)),
            nullptr,
            0,
            &got,
            nullptr,
            nullptr
        );
    }

    // BY NAME, every single attempt, and never a cached address: the field
    // network is a phone hotspot whose DHCP hands out a different address every
    // outing, and bibobox.local over mDNS is the thing that stays true.
    [[nodiscard]] Bool dialTcp(Conn& c, const Str& host, UInt16 port, Str* why)
    {
        addrinfo hints = {};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

        Array<Char, 16> portText = {};
        std::snprintf(portText.data(), portText.size(), "%u", static_cast<UInt32>(port));

        addrinfo* found = nullptr;
        const Int32 rc = ::getaddrinfo(host.c_str(), portText.data(), &hints, &found);
        if(rc != 0 || found == nullptr)
        {
            *why = "cannot resolve " + host + " - is the board on this network?";
            return false;
        }

        const Int64 deadline = monoMs() + CONNECT_MS;
        *why = "no answer from " + host + " within " + numberText(CONNECT_MS) + " ms";

        for(addrinfo* a = found; a != nullptr; a = a->ai_next)
        {
            const Int64 left = deadline - monoMs();
            if(left <= 0)
            {
                break;
            }
            const SOCKET fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
            if(fd == INVALID_SOCKET)
            {
                continue;
            }
            setNonBlocking(fd);

            const Int32 len = static_cast<Int32>(a->ai_addrlen);
            const Int32 answer = ::connect(fd, a->ai_addr, len);
            Bool up = answer == 0;
            if(!up && ::WSAGetLastError() == WSAEWOULDBLOCK)
            {
                up = finishConnect(fd, left);
            }
            if(!up)
            {
                ::closesocket(fd);
                continue;
            }

            tuneTcp(fd);
            c.tcp = fd;
            c.rx.assign(RX_BYTES, 0);
            c.rxUsed = 0;
            ::freeaddrinfo(found);
            return true;
        }

        ::freeaddrinfo(found);
        return false;
    }

    // Bound before HELLO, because HELLO has to carry the port the board will
    // send CTLSTATE to. An observer never sends a datagram, so this socket is
    // receive-only today - it is opened anyway so the reverse path exists the
    // day the control seam is filled in, and so a board that sends CTLSTATE to
    // an observer is heard rather than silently ignored.
    [[nodiscard]] Bool openUdp(Conn& c)
    {
        const SOCKET fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(fd == INVALID_SOCKET)
        {
            return false;
        }
        setNonBlocking(fd);

        sockaddr_in any = {};
        any.sin_family = AF_INET;
        any.sin_addr.s_addr = INADDR_ANY;
        any.sin_port = 0;
        const sockaddr* anyAddr = reinterpret_cast<const sockaddr*>(&any);
        if(::bind(fd, anyAddr, static_cast<Int32>(sizeof(any))) != 0)
        {
            ::closesocket(fd);
            return false;
        }

        sockaddr_in got = {};
        Int32 gotLen = static_cast<Int32>(sizeof(got));
        sockaddr* gotAddr = reinterpret_cast<sockaddr*>(&got);
        if(::getsockname(fd, gotAddr, &gotLen) != 0)
        {
            ::closesocket(fd);
            return false;
        }

        c.udp = fd;
        c.udpPort = ::ntohs(got.sin_port);
        return true;
    }

    // Point the UDP socket at the board and nowhere else, once WELCOME has named
    // the port. CTLSTATE carries no sessionId, so this filter is the only thing
    // between the honesty line and anybody else on the hotspot who fancies
    // telling this viewer the car is armed.
    Void filterUdpToBoard(Conn& c, UInt16 boardPort)
    {
        if(c.udp == INVALID_SOCKET || c.udpFiltered)
        {
            return;
        }
        sockaddr_storage peer = {};
        Int32 len = static_cast<Int32>(sizeof(peer));
        sockaddr* peerAddr = reinterpret_cast<sockaddr*>(&peer);
        if(::getpeername(c.tcp, peerAddr, &len) != 0)
        {
            return;
        }
        if(peer.ss_family == AF_INET)
        {
            reinterpret_cast<sockaddr_in*>(&peer)->sin_port = ::htons(boardPort);
        }
        else if(peer.ss_family == AF_INET6)
        {
            reinterpret_cast<sockaddr_in6*>(&peer)->sin6_port = ::htons(boardPort);
        }
        else
        {
            return;
        }
        // A UDP socket bound to AF_INET cannot be connected to an AF_INET6 peer;
        // when that happens the filter is simply not applied and the datagrams
        // are still CRC-checked like everything else.
        c.udpFiltered = ::connect(c.udp, peerAddr, len) == 0;
    }

    // ---- pumping ------------------------------------------------------------

    [[nodiscard]] Bool pumpTcp(Conn& c, Session& s)
    {
        for(;;)
        {
            if(c.rxUsed >= c.rx.size())
            {
                // 512 KiB with no frame in it. MAX_PAYLOAD is 256 KiB, so this
                // is not a big frame - it is a peer writing something that will
                // never parse.
                return false;
            }
            Char* at = reinterpret_cast<Char*>(c.rx.data() + c.rxUsed);
            const Int32 want = static_cast<Int32>(c.rx.size() - c.rxUsed);
            const Int32 n = ::recv(c.tcp, at, want, 0);
            if(n == 0)
            {
                return false;                     // orderly close from the board
            }
            if(n < 0)
            {
                if(::WSAGetLastError() == WSAEWOULDBLOCK)
                {
                    return true;
                }
                return false;
            }

            c.rxUsed += static_cast<Size>(n);
            const Size used = ingestBytes(s, c.rx.data(), c.rxUsed, monoMs());
            if(used > 0)
            {
                std::memmove(c.rx.data(), c.rx.data() + used, c.rxUsed - used);
                c.rxUsed -= used;
            }
            if(n < want)
            {
                return true;                      // the socket is drained
            }
        }
    }

    Void pumpUdp(Conn& c, Session& s)
    {
        if(c.udp == INVALID_SOCKET)
        {
            return;
        }
        Array<UInt8, bibowire::MAX_DATAGRAM> dgram = {};
        for(;;)
        {
            Char* at = reinterpret_cast<Char*>(dgram.data());
            const Int32 cap = static_cast<Int32>(dgram.size());
            const Int32 n = ::recvfrom(c.udp, at, cap, 0, nullptr, nullptr);
            if(n <= 0)
            {
                return;
            }
            // One datagram is one frame. take() checks the CRC, so a truncated
            // or forged datagram is dropped here rather than believed.
            bibowire::Frame f;
            Size used = 0;
            const Size len = static_cast<Size>(n);
            if(bibowire::take(dgram.data(), len, &f, &used) == bibowire::Take::TAKE_FRAME)
            {
                ingestFrame(s, f, monoMs());
            }
        }
    }

    [[nodiscard]] Bool flushPongs(Conn& c, Session& s)
    {
        Bool ok = true;
        for(const bibowire::Ping& ping : s.pongsDue)
        {
            Array<UInt8, 64> body = {};
            bibowire::Ping pong = ping;
            // The token is echoed VERBATIM; senderMonoUs is our own clock, which
            // the board only ever compares with itself.
            pong.senderMonoUs = static_cast<UInt64>(monoMs()) * 1000u;
            const Size n = bibowire::writePing(pong, body.data(), body.size());
            if(n == 0 || !sendFrame(c, bibowire::Type::TYPE_PONG, body.data(), n))
            {
                ok = false;
                break;
            }
        }
        s.pongsDue.clear();
        return ok;
    }

    // This viewer's OWN PING. Answering the board's proves the board's round
    // trip; the number an operator reads off this panel has to be the one
    // measured from here, or it is a far-end measurement wearing this end's
    // label.
    [[nodiscard]] Bool sendPing(Conn& c, Session& s, Int64 nowMs)
    {
        ++c.pingToken;
        bibowire::Ping ping;
        // A counter rather than the clock: two PINGs in the same millisecond
        // must not look like one another when their answers come back.
        ping.token = c.pingToken;
        ping.senderMonoUs = static_cast<UInt64>(nowMs) * 1000u;

        Array<UInt8, 64> body = {};
        const Size n = bibowire::writePing(ping, body.data(), body.size());
        if(n == 0)
        {
            return false;
        }
        notePingSent(s, ping.token, nowMs);
        return sendFrame(c, bibowire::Type::TYPE_PING, body.data(), n);
    }

    [[nodiscard]] Bool sendHello(Conn& c, const Str& name)
    {
        bibowire::Hello hello;
        hello.protoMajor = bibowire::PROTO_MAJOR;
        hello.protoMinor = bibowire::PROTO_MINOR;
        // NO CONVENTION FOR THIS FIELD EXISTS. The spec calls it "types this
        // viewer understands", the codec carries it verbatim and its 312 checks
        // say nothing about what a bit means, so any value here is a guess. All
        // bits set is the only guess that cannot be read as "this viewer
        // understands nothing" by a board that has not been written yet; when
        // the socket half pins the convention, this line is the one to change.
        hello.featureMask = 0xFFFFFFFFu;
        hello.viewerBuild = 0;
        hello.viewerUdpPort = c.udpPort;
        // OBSERVER. This viewer cannot drive - see the SEAM in link.hxx - and a
        // program that asks for the control slot it cannot use would take the
        // wheel away from a viewer that can.
        hello.wantControl = 0;
        hello.controlHz = 0;
        hello.name = name;

        Array<UInt8, 128> body = {};
        const Size n = bibowire::writeHello(hello, body.data(), body.size());
        if(n == 0)
        {
            return false;
        }
        return sendFrame(c, bibowire::Type::TYPE_HELLO, body.data(), n);
    }

    // SUBSCRIBE, and only when it would say something new.
    //
    // THIS ONE FRAME IS THE WHOLE COST CONTROL. The board never sends a type
    // the mask did not claim, so it is the entire difference between a link
    // carrying 2.5 KB per revolution and one carrying about a megabyte a
    // second, and between a board that opens /dev/video0 and one that leaves
    // it alone.
    [[nodiscard]] Bool syncSubscription(Conn& c, Session& s, Bool wantCam)
    {
        // HELLO is the first bytes on the connection and nothing else goes out
        // until WELCOME has answered it.
        if(!s.haveWelcome)
        {
            return true;
        }

        const UInt32 want = subscriptionMask(wantCam);

        // NEVER ASKED, AND NOTHING WANTED: say nothing at all. The board's
        // default already sends the telemetry this viewer draws, so a
        // SUBSCRIBE here would change nothing except to make this viewer's
        // first act on every connection a frame nobody needed - and it would
        // change the behaviour of a viewer whose camera window has never been
        // opened, which is every viewer until somebody opens one.
        if(c.sentMask == 0u && !wantCam)
        {
            return true;
        }
        if(c.sentMask == want)
        {
            return true;
        }

        bibowire::Subscribe sub;
        sub.sessionId = s.welcome.sessionId;
        sub.typeMask = want;
        // 1 = every revolution. The camera does NOT quietly buy itself room by
        // thinning the scan: which of the two matters is the operator's
        // decision, and halving the scan the moment a window opened would be
        // exactly the silent degradation the drop classes exist to make
        // visible instead.
        sub.scanDivisor = 1;

        Array<UInt8, 32> body = {};
        const Size n = bibowire::writeSubscribe(sub, body.data(), body.size());
        if(n == 0)
        {
            return false;
        }
        if(!sendFrame(c, bibowire::Type::TYPE_SUBSCRIBE, body.data(), n))
        {
            return false;
        }

        c.sentMask = want;
        s.cameraSubscribed = wantCam;
        return true;
    }

    Void sendLeave(Conn& c, const Session& s)
    {
        if(c.tcp == INVALID_SOCKET || !s.haveWelcome)
        {
            return;
        }
        bibowire::Leave leave;
        leave.sessionId = s.welcome.sessionId;
        Array<UInt8, 32> body = {};
        const Size n = bibowire::writeLeave(leave, body.data(), body.size());
        if(n == 0)
        {
            return;
        }
        // A deliberate close says so rather than letting the board find out by
        // FIN. It costs one 24-byte frame and it is the difference between "the
        // operator left" and "the link died" in the board's own journal.
        //
        // Best effort, and the answer is consumed rather than cast away: this
        // socket is closing either way, and the board's own timers cover a LEAVE
        // that never made it onto the wire.
        if(!sendFrame(c, bibowire::Type::TYPE_LEAVE, body.data(), n))
        {
            return;
        }
    }

    // ---- the sentence the panel shows ---------------------------------------

    [[nodiscard]] Str scanPhrase(const Session& s, Int64 nowMs)
    {
        if(!s.haveScan)
        {
            return "no scan yet";
        }
        const Int64 age = ageOf(s, nowMs, s.scanAtMs, s.scanBoardUs);
        if(age > GONE_MS)
        {
            // The link is healthy and the sensor is not. Those are different
            // facts and this is the sentence that keeps them apart.
            return "no scan for " + secondsText(age);
        }
        Array<Char, 96> t = {};
        const UInt32 hz = s.freqMilliHz;
        std::snprintf(
            t.data(),
            t.size(),
            "rev %u at %u.%03u Hz",
            s.revIndex,
            hz / 1000u,
            hz % 1000u
        );
        return Str(t.data());
    }

    [[nodiscard]] Str liveStatus(const Session& s, const Str& host, Int64 nowMs)
    {
        if(!s.haveWelcome)
        {
            return "handshaking with " + host + " - HELLO sent, waiting for WELCOME";
        }
        Str out = "live - ";
        out += s.welcome.boardName.empty() ? host : s.welcome.boardName;
        if(s.welcome.accepted == 0u)
        {
            out += " REFUSED";
        }
        out += ", " + scanPhrase(s, nowMs);
        if(!s.welcome.text.empty())
        {
            out += " - " + s.welcome.text;
        }
        return out;
    }

    Void publish(Client& c, Phase phase, const Str& status, Int32 retryInMs, const Session& s)
    {
        LockGuard<Mutex> held(c.lock);
        c.shared.phase = phase;
        c.shared.status = status;
        c.shared.retryInMs = retryInMs;
        c.shared.state = s;
    }

    // Sleeps in slices so a Disconnect is answered in one POLL_MS rather than in
    // four seconds, and republishes the countdown as it goes - "retrying in
    // 1840 ms" is a thing an operator can wait for; a frozen panel is not.
    Void waitToRetry(Client& c, const Session& s, Int32 waitMs, const Str& why)
    {
        const Int64 until = monoMs() + waitMs;
        for(;;)
        {
            const Int64 left = until - monoMs();
            if(left <= 0 || c.quit.load())
            {
                return;
            }
            const Str text = "retrying in " + numberText(left) + " ms - " + why;
            publish(c, Phase::PHASE_RETRYING, text, static_cast<Int32>(left), s);
            ::Sleep(static_cast<DWORD>(left < POLL_MS ? left : POLL_MS));
        }
    }

    Void runWorker(Client* c)
    {
        Conn conn;
        Session live;
        Int32 attempt = 0;
        // Seeded from the clock, so two viewers started from the same script do
        // not draw the same jitter and hammer the board in lockstep - which is
        // the whole reason the jitter is there.
        UInt32 seed = static_cast<UInt32>(monoMs()) ^ 0xB1B0B0C5u;

        while(!c->quit.load())
        {
            // A new connection RESUMES NOTHING. There is no session-resumption
            // path in this protocol at all: the only state worth resuming is the
            // live picture, which is worthless by the time the link is back.
            clearSession(live);
            publish(*c, Phase::PHASE_RESOLVING, "resolving " + c->host, 0, live);

            Str why;
            if(!dialTcp(conn, c->host, c->port, &why))
            {
                dropConn(conn);
                ++attempt;
                seed = stir(seed);
                waitToRetry(*c, live, jittered(backoffBaseMs(attempt), seed), why);
                continue;
            }

            if(!openUdp(conn))
            {
                // Telemetry is unaffected: CTLSTATE is the only thing that rides
                // UDP toward this viewer, so what degrades is the honesty line
                // and nothing else. Said in words rather than left as a panel
                // row that is quietly always "--".
                Note note;
                note.severity = bibowire::Severity::SEVERITY_WARN;
                note.text = "no UDP socket - CTLSTATE cannot arrive";
                note.atMs = monoMs();
                live.notes.push_back(note);
            }
            publish(*c, Phase::PHASE_CONNECTING, "connected to " + c->host, 0, live);

            // HELLO IS THE FIRST BYTES ON THE CONNECTION, and nothing else is
            // sent until WELCOME arrives.
            if(!sendHello(conn, "bibo viewer"))
            {
                dropConn(conn);
                ++attempt;
                seed = stir(seed);
                waitToRetry(
                    *c,
                    live,
                    jittered(backoffBaseMs(attempt), seed),
                    "could not send HELLO"
                );
                continue;
            }

            live.lastFrameMs = monoMs();
            Int64 nextPingMs = live.lastFrameMs + PING_PERIOD_MS;
            why = "the board closed the connection";

            while(!c->quit.load())
            {
                fd_set reads;
                FD_ZERO(&reads);
                FD_SET(conn.tcp, &reads);
                if(conn.udp != INVALID_SOCKET)
                {
                    FD_SET(conn.udp, &reads);
                }
                timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = POLL_MS * 1000;
                const Int32 ready = ::select(0, &reads, nullptr, nullptr, &tv);
                if(ready == SOCKET_ERROR)
                {
                    why = "the connection failed";
                    break;
                }

                const Bool hadWelcome = live.haveWelcome;
                if(ready > 0 && FD_ISSET(conn.tcp, &reads) && !pumpTcp(conn, live))
                {
                    break;
                }
                if(ready > 0 && conn.udp != INVALID_SOCKET && FD_ISSET(conn.udp, &reads))
                {
                    pumpUdp(conn, live);
                }
                if(!flushPongs(conn, live))
                {
                    why = "could not answer a PING";
                    break;
                }

                if(!hadWelcome && live.haveWelcome)
                {
                    // A handshake that completed is a schedule that starts over:
                    // the next failure is a fresh one, not the tail of an old
                    // outage.
                    attempt = 0;
                    filterUdpToBoard(conn, live.welcome.controlUdpPort);
                }

                // Re-asserted every pass, because the answer can change at any
                // moment: the operator closes the camera window mid-outing, or
                // the link drops and comes back and the new session has been
                // told nothing. `sentMask` went with the old connection, so a
                // window that was open before the drop is asked for again.
                if(!syncSubscription(conn, live, c->cameraOn.load()))
                {
                    why = "could not send SUBSCRIBE";
                    break;
                }

                if(live.haveBye)
                {
                    why = live.byeText.empty() ? Str("the board said BYE") : live.byeText;
                    break;
                }

                const Int64 now = monoMs();
                if(now - live.lastFrameMs > SILENCE_MS)
                {
                    // No frame of ANY type. At 5 Hz BOARD and 1 Hz PING this is
                    // not a quiet moment.
                    why = "no frame for " + numberText(now - live.lastFrameMs) + " ms";
                    break;
                }

                // Only once WELCOME has arrived: HELLO is the first bytes on the
                // connection and NOTHING else goes out until the board has
                // answered it.
                if(live.haveWelcome && now >= nextPingMs)
                {
                    nextPingMs = now + PING_PERIOD_MS;
                    if(!sendPing(conn, live, now))
                    {
                        why = "could not send a PING";
                        break;
                    }
                }

                const Phase phase = live.haveWelcome ? Phase::PHASE_LIVE : Phase::PHASE_HANDSHAKING;
                publish(*c, phase, liveStatus(live, c->host, now), 0, live);
            }

            if(c->quit.load())
            {
                sendLeave(conn, live);
                dropConn(conn);
                break;
            }

            dropConn(conn);
            ++attempt;
            seed = stir(seed);
            waitToRetry(*c, live, jittered(backoffBaseMs(attempt), seed), why);
        }
    }

  }

  // ---- the clock -------------------------------------------------------------

  Int64 monoMs()
  {
      static const TimePoint BASE = monoNow();
      return static_cast<Int64>(elapsedMs(BASE));
  }

  // ---- names -----------------------------------------------------------------

  CharSeq phaseName(Phase p)
  {
      switch(p)
      {
      case Phase::PHASE_IDLE:
          return "idle";
      case Phase::PHASE_RESOLVING:
          return "resolving";
      case Phase::PHASE_CONNECTING:
          return "connecting";
      case Phase::PHASE_HANDSHAKING:
          return "handshaking";
      case Phase::PHASE_LIVE:
          return "live";
      case Phase::PHASE_RETRYING:
          return "retrying";
      }
      return "?";
  }

  CharSeq modeName(UInt8 mode)
  {
      switch(mode)
      {
      case 0:
          return "cruise";
      case 1:
          return "slow";
      case 2:
          return "stop";
      case 3:
          return "reverse";
      case 4:
          return "blind";
      default:
          break;
      }
      return "?";
  }

  CharSeq sourceName(UInt8 source)
  {
      switch(source)
      {
      case 0:
          return "manual";
      case 1:
          return "look";
      case 2:
          return "drive";
      default:
          break;
      }
      return "?";
  }

  // ---- the pure half ---------------------------------------------------------

  Void clearSession(Session& s)
  {
      const Bool keepFlag = s.haveBootId;
      const UInt32 keepBoot = s.bootId;
      s = Session();
      s.haveBootId = keepFlag;
      s.bootId = keepBoot;
  }

  Void clearAll(Session& s)
  {
      s = Session();
  }

  Opt<Revolution> Session::revolution(Int64 nowMs) const
  {
      if(!haveScan)
      {
          return {};
      }
      const Int64 age = ageOf(*this, nowMs, scanAtMs, scanBoardUs);
      if(age > GONE_MS)
      {
          return {};
      }
      Revolution r;
      r.cloud = cloud;
      r.revIndex = revIndex;
      r.freqMilliHz = freqMilliHz;
      r.droppedSinceLast = droppedSinceLast;
      r.health = scanHealth;
      r.motor = scanMotor;
      r.ageMs = age;
      r.stale = age > FRESH_MS;
      return r;
  }

  Opt<Decision> Session::decision(Int64 nowMs) const
  {
      if(!haveDecide)
      {
          return {};
      }
      const Int64 age = ageOf(*this, nowMs, decideAtMs, decideBoardUs);
      if(age > GONE_MS)
      {
          return {};
      }
      Decision d;
      d.decide = decide;
      d.ageMs = age;
      d.stale = age > FRESH_MS;
      return d;
  }

  Opt<Board> Session::boardState(Int64 nowMs) const
  {
      if(!haveBoard)
      {
          return {};
      }
      const Int64 age = ageOf(*this, nowMs, boardAtMs, board.tMonoUs);
      if(age > GONE_MS)
      {
          return {};
      }
      Board b;
      b.state = board;
      b.ageMs = age;
      b.stale = age > FRESH_MS;
      return b;
  }

  Opt<Control> Session::controlState(Int64 nowMs) const
  {
      if(!haveControl)
      {
          return {};
      }
      const Int64 age = ageOf(*this, nowMs, controlAtMs, control.tMonoUs);
      if(age > GONE_MS)
      {
          return {};
      }
      Control c;
      c.state = control;
      c.ageMs = age;
      c.stale = age > FRESH_MS;
      return c;
  }

  Opt<CameraShot> Session::cameraShot(Int64 nowMs) const
  {
      if(!haveCamera)
      {
          return {};
      }
      const Int64 age = ageOf(*this, nowMs, cameraAtMs, camera.tMonoUs);
      if(age > GONE_MS)
      {
          // No picture at all, rather than a dimmer one. A photograph of a
          // corridor is equally convincing whether it was taken now or forty
          // seconds ago - there is nothing in the image itself for a person to
          // read the age off, which makes absence matter MORE here than it
          // does for the cloud.
          return {};
      }
      CameraShot shot;
      shot.frameIndex = camera.frameIndex;
      shot.width = camera.width;
      shot.height = camera.height;
      shot.codec = camera.codec;
      shot.bytes = camera.data;
      shot.ageMs = age;
      shot.stale = age > FRESH_MS;
      return shot;
  }

  Opt<Int64> Session::rttMs() const
  {
      if(!haveRtt)
      {
          return {};
      }
      return lastRttMs;
  }

  Opt<Int64> Session::bestRttMs() const
  {
      const RttSample* best = bestSample(*this);
      if(best == nullptr)
      {
          return {};
      }
      return best->rttMs;
  }

  Opt<Int64> Session::oneWayMs() const
  {
      const Opt<Int64> best = bestRttMs();
      if(!best.has_value())
      {
          return {};
      }
      return *best / 2;
  }

  Void notePingSent(Session& s, UInt64 token, Int64 nowMs)
  {
      PingOut out;
      out.token = token;
      out.sentMs = nowMs;
      s.pingsOut.push_back(out);
      // An unanswered PING is the silence watchdog's business. This list exists
      // to match tokens, not to accumulate evidence of a link that has already
      // stopped answering.
      while(s.pingsOut.size() > MAX_PINGS_OUT)
      {
          s.pingsOut.erase(s.pingsOut.begin());
      }
  }

  Void ingestFrame(Session& s, const bibowire::Frame& f, Int64 nowMs)
  {
      // EVERY frame, including one whose body this build has no name for. The
      // silence watchdog asks "has anything arrived", and a reader that only
      // counted the messages it understood would redial a perfectly live board
      // the day it learns a new type.
      ++s.frames;
      s.lastFrameMs = nowMs;

      const UInt8 ver = f.head.ver;
      switch(f.head.type)
      {
      case bibowire::Type::TYPE_WELCOME:
      {
          bibowire::Welcome m;
          if(!bibowire::readWelcome(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          // A DIFFERENT bootId means the pilot restarted, and everything this
          // viewer knows is about a car that no longer exists. Cleared BEFORE a
          // single point is drawn - this is the specific defence against the
          // most convincing stale picture there is: a healthy new socket to a
          // restarted car, still showing the previous run.
          if(s.haveBootId && s.bootId != m.bootId)
          {
              clearAll(s);
              s.frames = 1;
              s.lastFrameMs = nowMs;
              Note note;
              note.severity = bibowire::Severity::SEVERITY_WARN;
              note.text = "board restarted - cleared scan, decision and state";
              note.atMs = nowMs;
              s.notes.push_back(note);
          }
          s.haveBootId = true;
          s.bootId = m.bootId;
          s.haveWelcome = true;
          s.welcome = m;
          noteOffset(s, nowMs, m.boardMonoUs);

          // The viewer applies the version rule to WELCOME too, and shows the
          // sentence rather than a spinner. An explanation that says only
          // "incompatible" sends a person to read source in a field.
          if(!bibowire::versionOk(m.protoMajor))
          {
              s.haveBye = true;
              s.byeReason = bibowire::Reason::REASON_VERSION;
              s.byeText = "board speaks bibowire " + numberText(m.protoMajor) + "."
                          + numberText(m.protoMinor) + ", viewer speaks "
                          + numberText(bibowire::PROTO_MAJOR) + "."
                          + numberText(bibowire::PROTO_MINOR)
                          + " - rebuild the viewer from the same commit as the board";
          }
          return;
      }

      case bibowire::Type::TYPE_LIDAR_INFO:
      {
          bibowire::LidarInfo m;
          if(!bibowire::readLidarInfo(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          s.haveLidar = true;
          s.lidar = m;
          return;
      }

      case bibowire::Type::TYPE_SCAN:
      {
          bibowire::Scan m;
          if(!bibowire::readScan(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          // GAPS ARE COUNTED, NEVER SMOOTHED. revIndex is monotonic, so what is
          // missing is knowable exactly; an interpolated sweep between two
          // revolutions a second apart is a lie the eye cannot detect.
          if(s.haveScan && m.revIndex > s.revIndex + 1u)
          {
              s.missedRevs += m.revIndex - s.revIndex - 1u;
              Array<Char, 64> line = {};
              std::snprintf(
                  line.data(),
                  line.size(),
                  "revolutions %u-%u missing",
                  s.revIndex + 1u,
                  m.revIndex - 1u
              );
              s.gapText = line.data();
          }

          s.cloud.clear();
          s.cloud.reserve(m.points.size());
          for(const bibowire::ScanPoint& p : m.points)
          {
              if(p.distMm == 0u)
              {
                  continue;                   // NO RETURN, not zero range
              }
              const Float32 a = static_cast<Float32>(p.angleCentiDeg) * CENTI_DEG_TO_RAD;
              const Float32 r = static_cast<Float32>(p.distMm) * MM_TO_M;
              // scene.hxx's frame: X right, Y forward, Z up, metres, with the
              // car at the origin pointing along +Y - so bearing 0 is +Y and a
              // growing angle swings toward +X, which is exactly how the
              // stand-in cloud this replaces was laid out.
              s.cloud.push_back(scene::Vec3{ r * std::sin(a), r * std::cos(a), LIDAR_HEIGHT_M });
          }

          s.haveScan = true;
          s.revIndex = m.revIndex;
          s.freqMilliHz = m.freqMilliHz;
          s.droppedSinceLast = m.droppedSinceLast;
          s.scanHealth = m.health;
          s.scanMotor = m.motor;
          s.scanAtMs = nowMs;
          s.scanBoardUs = m.tMonoUs;
          noteOffset(s, nowMs, m.tMonoUs);
          return;
      }

      case bibowire::Type::TYPE_DECIDE:
      {
          bibowire::Decide m;
          if(!bibowire::readDecide(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          // A DECIDE naming a SCAN this viewer never received is DISCARDED, not
          // drawn. The corridor belongs to one revolution, and drawing it over a
          // different one produces a picture that is individually plausible and
          // jointly false. revIndex 0 is the BLIND tick, which has no revolution
          // behind it by definition and is kept.
          if(m.revIndex != 0u && (!s.haveScan || m.revIndex != s.revIndex))
          {
              ++s.orphanDecides;
              return;
          }
          s.haveDecide = true;
          s.decide = m;
          s.decideAtMs = nowMs;
          // DECIDE carries no timestamp of its own; it is tied to the SCAN it
          // describes, so it borrows that revolution's board clock and shares
          // its age exactly.
          s.decideBoardUs = m.revIndex == 0u ? 0u : s.scanBoardUs;
          return;
      }

      case bibowire::Type::TYPE_BOARD:
      {
          bibowire::BoardState m;
          if(!bibowire::readBoard(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          s.haveBoard = true;
          s.board = m;
          s.boardAtMs = nowMs;
          noteOffset(s, nowMs, m.tMonoUs);
          return;
      }

      case bibowire::Type::TYPE_CTLSTATE:
      {
          bibowire::CtlState m;
          if(!bibowire::readCtlState(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          s.haveControl = true;
          s.control = m;
          s.controlAtMs = nowMs;
          noteOffset(s, nowMs, m.tMonoUs);
          return;
      }

      case bibowire::Type::TYPE_CAMERA:
      {
          bibowire::Camera m;
          if(!bibowire::readCamera(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          // GAPS ARE COUNTED, NEVER SMOOTHED - the scan's rule, applied here
          // for the same reason. frameIndex is monotonic, so what is missing
          // is knowable exactly, and a window that simply showed the next
          // picture would hide a link dropping half the stream.
          //
          // CAMERA is CLASS_BULK and is discarded before any scan or state
          // frame, so on a bad hotspot this is the counter that moves FIRST.
          // That is the design working, not a fault - what degrades is the
          // camera and not the car's picture of the world.
          if(s.haveCamera && m.frameIndex > s.camera.frameIndex + 1u)
          {
              s.missedCameraFrames += m.frameIndex - s.camera.frameIndex - 1u;
              Array<Char, 64> line = {};
              std::snprintf(
                  line.data(),
                  line.size(),
                  "frames %u-%u missing",
                  s.camera.frameIndex + 1u,
                  m.frameIndex - 1u
              );
              s.cameraGapText = line.data();
          }

          s.haveCamera = true;
          s.cameraAtMs = nowMs;
          ++s.cameraFrames;
          noteOffset(s, nowMs, m.tMonoUs);
          // Moved rather than copied: the body is a whole JPEG, tens of
          // kilobytes, and this runs on the network thread several times a
          // second.
          s.camera = std::move(m);
          return;
      }

      case bibowire::Type::TYPE_EVENT:
      {
          bibowire::Event m;
          if(!bibowire::readEvent(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          // Every lidar::reason() and every carlink::detail() the board writes
          // for a person arrives here VERBATIM, and is shown that way. A
          // protocol that keeps only the codes is how a project loses the one
          // thing that makes a fault diagnosable.
          Note note;
          note.severity = m.severity;
          note.text = m.text;
          note.atMs = nowMs;
          if(m.droppedSince > 0u)
          {
              note.text += " (+" + numberText(m.droppedSince) + " suppressed)";
          }
          // Kept where the camera window can reach it, as well as in the note
          // list. "another program holds /dev/video0" is the sentence that
          // turns a blank rectangle into an answer, and an operator should not
          // have to find it in a scrolling list to learn why there is no
          // picture. See link.hxx: matching on the text is a heuristic, and a
          // deliberate one.
          if(mentionsCamera(note.text))
          {
              s.haveCameraNote = true;
              s.cameraNoteText = note.text;
              s.cameraNoteAtMs = nowMs;
          }

          s.notes.push_back(note);
          while(s.notes.size() > MAX_EVENTS)
          {
              s.notes.erase(s.notes.begin());
          }
          return;
      }

      case bibowire::Type::TYPE_PING:
      {
          bibowire::Ping m;
          if(!bibowire::readPing(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          s.pongsDue.push_back(m);
          return;
      }

      case bibowire::Type::TYPE_BYE:
      {
          bibowire::Bye m;
          if(!bibowire::readBye(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          s.haveBye = true;
          s.byeReason = m.reason;
          s.byeText = m.text;
          return;
      }

      case bibowire::Type::TYPE_PONG:
      {
          bibowire::Ping m;
          if(!bibowire::readPing(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          // Matched by TOKEN, which the protocol echoes verbatim. A PONG for a
          // PING this connection never sent, or a second copy of one already
          // accounted for, must not be able to invent a round trip - so an
          // unmatched token is dropped rather than timed against the newest
          // send.
          for(Size i = 0; i < s.pingsOut.size(); ++i)
          {
              if(s.pingsOut[i].token != m.token)
              {
                  continue;
              }
              const Int64 rtt = nowMs - s.pingsOut[i].sentMs;
              s.pingsOut.erase(s.pingsOut.begin() + static_cast<ISize>(i));
              if(rtt < 0)
              {
                  // A clock that went backwards is a bug, not a fast link.
                  return;
              }
              RttSample sample;
              sample.rttMs = rtt;
              sample.atMs = nowMs;
              sample.boardUs = m.senderMonoUs;
              s.rtts.push_back(sample);
              while(s.rtts.size() > RTT_SAMPLES)
              {
                  s.rtts.erase(s.rtts.begin());
              }
              s.haveRtt = true;
              s.lastRttMs = rtt;
              noteRttOffset(s);
              return;
          }
          return;
      }

      default:
          break;
      }

      // AN UNKNOWN TYPE IS SKIPPED BY EXACTLY len AND COUNTED, NEVER FATAL. That
      // is what the length prefix is for, and it is the whole reason an older
      // viewer can keep watching a newer board.
      ++s.unknownFrames;
  }

  Size ingestBytes(Session& s, const UInt8* buf, Size len, Int64 nowMs)
  {
      Size at = 0;
      while(at < len)
      {
          bibowire::Frame f;
          Size used = 0;
          const bibowire::Take got = bibowire::take(buf + at, len - at, &f, &used);
          if(got == bibowire::Take::TAKE_FRAME)
          {
              ingestFrame(s, f, nowMs);
              at += used;
              continue;
          }
          if(got == bibowire::Take::TAKE_RESYNC)
          {
              // Junk on a checksummed stream that is never counted is a fault
              // nobody discovers.
              s.resyncBytes += static_cast<UInt32>(used);
              at += used;
              continue;
          }
          if(got == bibowire::Take::TAKE_NEED_MORE)
          {
              break;
          }
          // TOO_BIG or BAD_FLAG: a terminal answer about a frame at the stream
          // position. Counted, and the byte is stepped over so the caller's ring
          // can never wedge on it - the connection is torn down by the silence
          // watchdog if the peer keeps it up.
          ++s.refusedFrames;
          at += 1;
      }
      return at;
  }

  // ---- the reconnect schedule ------------------------------------------------

  // xorshift32, written here rather than taken from <random>: the schedule is a
  // safety-adjacent behaviour and a test that cannot reproduce a failure is a
  // test nobody will fix.
  UInt32 stir(UInt32 seed)
  {
      UInt32 x = seed == 0u ? 0x9E3779B9u : seed;
      x ^= x << 13;
      x ^= x >> 17;
      x ^= x << 5;
      return x;
  }

  Int32 backoffBaseMs(Int32 attempt)
  {
      // 250, 500, 1 s, 2 s, 4 s, then 4 s forever. NEVER GIVING UP, because
      // outdoors the link comes back when the phone stops moving, and a viewer
      // that stopped trying makes that recovery a manual step in a field.
      const Int32 step = attempt < 1 ? 1 : attempt;
      if(step >= BACKOFF_STEPS)
      {
          return 4000;
      }
      Int32 ms = 250;
      for(Int32 i = 1; i < step; ++i)
      {
          ms *= 2;
      }
      return ms;
  }

  Int32 jittered(Int32 baseMs, UInt32 roll)
  {
      const Int32 span = (baseMs * JITTER_PERCENT) / 100;
      if(span <= 0)
      {
          return baseMs;
      }
      const Int32 spread = (span * 2) + 1;
      const Int32 delta = static_cast<Int32>(roll % static_cast<UInt32>(spread)) - span;
      return baseMs + delta;
  }

  // ---- the socket half -------------------------------------------------------

  Bool open(Client& c, CharSeq host, UInt16 port)
  {
      if(c.running.load())
      {
          return false;
      }
      WSADATA wsa;
      if(::WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
      {
          LockGuard<Mutex> held(c.lock);
          c.shared.phase = Phase::PHASE_IDLE;
          c.shared.status = "Winsock would not start";
          return false;
      }

      c.host = host == nullptr ? Str("bibobox.local") : Str(host);
      c.port = port;
      c.quit.store(false);
      c.running.store(true);
      {
          LockGuard<Mutex> held(c.lock);
          c.shared = Snapshot();
          c.shared.phase = Phase::PHASE_RESOLVING;
          c.shared.status = "resolving " + c.host;
      }
      c.worker = Thread(runWorker, &c);
      return true;
  }

  Void close(Client& c)
  {
      if(!c.running.load())
      {
          return;
      }
      c.quit.store(true);
      if(c.worker.joinable())
      {
          c.worker.join();
      }
      c.running.store(false);
      ::WSACleanup();

      LockGuard<Mutex> held(c.lock);
      c.shared = Snapshot();
  }

  Bool isOpen(const Client& c)
  {
      return c.running.load();
  }

  Snapshot snapshot(Client& c)
  {
      LockGuard<Mutex> held(c.lock);
      return c.shared;
  }

  // ---- the subscription --------------------------------------------------

  UInt32 typeBit(bibowire::Type type)
  {
      // bit = tag - 0x10 for the board->viewer telemetry range, which is what
      // "a bit per telemetry type" means. Settled in viewfeed.cxx and asserted
      // by its suite; restated here rather than invented, and the viewer's own
      // suite pins the same answers so the two ends cannot drift apart in
      // silence. A type outside the range has no bit and is always sent.
      const UInt8 tag = static_cast<UInt8>(type);
      return tag >= 0x10u && tag <= 0x2Fu ? (1u << (tag - 0x10u)) : 0u;
  }

  UInt32 subscriptionMask(Bool withCamera)
  {
      // Every telemetry type this viewer actually draws, named one at a time.
      // Spelled out rather than left as 0, because 0 means EVERYTHING to the
      // board - so an unsubscribe written as 0 would ask for MORE than it
      // started with, which is the opposite of what the caller meant and the
      // exact bug that would only show up as a bandwidth figure.
      UInt32 mask = typeBit(bibowire::Type::TYPE_SCAN)
                    | typeBit(bibowire::Type::TYPE_DECIDE)
                    | typeBit(bibowire::Type::TYPE_BOARD)
                    | typeBit(bibowire::Type::TYPE_LIDAR_INFO)
                    | typeBit(bibowire::Type::TYPE_EVENT)
                    | typeBit(bibowire::Type::TYPE_CTLSTATE)
                    | typeBit(bibowire::Type::TYPE_CMDACK);
      if(withCamera)
      {
          mask |= typeBit(bibowire::Type::TYPE_CAMERA);
      }
      return mask;
  }

  Void wantCamera(Client& c, Bool on)
  {
      c.cameraOn.store(on);
  }

  Bool cameraWanted(const Client& c)
  {
      return c.cameraOn.load();
  }

}
