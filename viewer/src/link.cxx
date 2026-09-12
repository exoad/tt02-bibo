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

        // ---- CONTROL, which belongs to ONE connection ---------------------
        //
        // The seq is per SESSION and starts again at 1 on every reconnect,
        // which is not an oversight: section 6 says the board's high-water
        // mark is reset by the handshake, and comparison is on the signed
        // difference precisely so a viewer that begins again at 1 is accepted
        // rather than frozen out by a huge stale number.
        UInt32 ctlSeq = 0;

        // The frame header's own counter for the UDP stream, kept apart from
        // txSeq. The frame-header ring reads seqs per STREAM, and pushing two
        // streams through one counter would make the TCP side look like it was
        // losing every frame the UDP side sent.
        UInt16 udpTxSeq = 0;

        // Where the board is, learned from the TCP peer - the only address
        // this end can be sure belongs to the board. Kept even when connect()
        // on the UDP socket did not take, so a datagram can still be addressed
        // explicitly rather than not sent at all.
        sockaddr_storage boardAddr = {};
        Int32 boardAddrLen = 0;
        Bool haveBoardAddr = false;

        // What THIS connection has told the board it wants. 0 means "never
        // asked", which is not the same fact as "asked for nothing" - see
        // syncSubscription. Reset with the connection, because the board keeps
        // no subscription across a session either.
        UInt32 sentMask = 0;

        // And what rate it was told, so a viewer that changes the slider
        // re-sends rather than waiting for a mask change that never comes.
        UInt16 sentFps = 0;
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
        c.ctlSeq = 0;
        c.udpTxSeq = 0;
        c.boardAddrLen = 0;
        c.haveBoardAddr = false;
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
            // KEPT, and only for AF_INET: this socket is created AF_INET, so an
            // IPv6 board is an address it cannot send to at all. Recording one
            // would leave sendto failing forever on every datagram instead of
            // the fallback noticing there is no reverse path and moving CONTROL
            // onto TCP, where the board accepts it always.
            c.boardAddr = peer;
            c.boardAddrLen = len;
            c.haveBoardAddr = true;
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

    [[nodiscard]] Bool sendHello(Conn& c, const Str& name, Bool wantSlot)
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
        // OBSERVER UNLESS THE OPERATOR ASKED, and this is the ONLY place the
        // question is ever put: viewfeed.cxx grants the slot in onHello and
        // nowhere else, so a viewer that did not ask here is an observer for
        // the whole life of this connection however many buttons it grows.
        //
        // Default off, because taking the slot arms a deadman over whatever the
        // car is doing - including an autonomous run somebody else started.
        hello.wantControl = wantSlot ? 1u : 0u;
        // Informational, and only when there is a stream to describe. 20 Hz is
        // CONTROL_PERIOD_MS turned into a rate, from the protocol's own header
        // rather than typed again: what this viewer INTENDS before WELCOME has
        // told it the board's period.
        hello.controlHz = wantSlot
            ? static_cast<UInt16>(1000 / bibowire::CONTROL_PERIOD_MS)
            : 0u;
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
    [[nodiscard]] Bool syncSubscription(Conn& c, Session& s, Bool wantCam, Int32 wantFps)
    {
        // HELLO is the first bytes on the connection and nothing else goes out
        // until WELCOME has answered it.
        if(!s.haveWelcome)
        {
            return true;
        }

        const UInt32 want = subscriptionMask(wantCam);

        // A RATE IS ONLY MEANINGFUL ALONGSIDE A SUBSCRIPTION. With the camera
        // off there is nothing to set a rate for, so it is sent as 0 - "did not
        // ask" - rather than carrying the last slider position on a frame that
        // switches the camera off.
        UInt16 fps = 0;
        if(wantCam && wantFps > 0)
        {
            const Int32 ceiling = static_cast<Int32>(bibowire::CAM_FPS_MAX);
            fps = static_cast<UInt16>(wantFps > ceiling ? ceiling : wantFps);
        }

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
        if(c.sentMask == want && c.sentFps == fps)
        {
            return true;
        }

        bibowire::Subscribe sub;
        sub.sessionId = s.welcome.sessionId;
        sub.typeMask = want;
        sub.camFps = fps;
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
        c.sentFps = fps;
        s.cameraSubscribed = wantCam;
        return true;
    }

    // The epoch this viewer BELIEVES is in force, from the freshest thing that
    // carries one. CTLSTATE is newest at 20 Hz, then BOARD at 5, then the
    // WELCOME that opened the session.
    //
    // None of the tuning verbs turn on the epoch - they are refused by ARM
    // STATE, not by generation - but the field is on the wire either way and
    // sending a stale one would be inventing a number. When ARM itself is wired
    // up through this same path the value will matter, and it will already be
    // the right one.
    [[nodiscard]] UInt8 currentEpoch(const Session& s)
    {
        if(s.haveControl)
        {
            return s.control.armEpoch;
        }
        if(s.haveBoard)
        {
            return s.board.armEpoch;
        }
        return s.welcome.armEpoch;
    }

    // Everything the UI thread has asked for, onto the wire - or dropped, if
    // there is no wire to put it on.
    //
    // The queue is emptied EITHER WAY, and that is the drop decision written as
    // code (link.hxx says why at length): a tuning command that waited out a
    // reconnect would be applied to the car minutes after the person who asked
    // for it stopped expecting it. Emptied under the lock and sent outside it,
    // so a slow socket cannot block the UI thread's next click.
    [[nodiscard]] Bool flushCommands(Conn& c, Client& owner, Session& s)
    {
        Vec<bibowire::Command> outbound;
        {
            LockGuard<Mutex> held(owner.cmdLock);
            if(owner.pending.empty())
            {
                return true;
            }
            outbound.swap(owner.pending);
        }

        // HELLO is the first bytes on the connection and nothing else goes out
        // until WELCOME has answered it - syncSubscription's rule, and the same
        // reason. Before that there is no sessionId to stamp, so these are not
        // merely early, they are unsendable.
        if(!s.haveWelcome)
        {
            owner.commandsDropped.fetch_add(static_cast<UInt32>(outbound.size()));
            return true;
        }

        // Indexed rather than a range-for, so the failure path below can count
        // what is actually LEFT. Counting the whole batch there would report
        // commands the board has already acknowledged as dropped, which is a
        // counter that lies in the safe-looking direction.
        for(Size i = 0; i < outbound.size(); ++i)
        {
            bibowire::Command cmd = outbound[i];
            cmd.sessionId = s.welcome.sessionId;
            cmd.armEpoch = currentEpoch(s);

            Array<UInt8, 64> body = {};
            const Size n = bibowire::writeCommand(cmd, body.data(), body.size());
            if(n == 0)
            {
                // The codec refused to encode it. That is a bug in the caller's
                // arguments rather than a link fault, so it is counted as a
                // drop and the connection is left alone.
                owner.commandsDropped.fetch_add(1u);
                continue;
            }
            if(!sendFrame(c, bibowire::Type::TYPE_COMMAND, body.data(), n))
            {
                // This one and everything behind it die with the connection, by
                // the same rule: the caller is told by the counter, not by a
                // retry onto a socket that has just failed.
                const Size left = outbound.size() - i;
                owner.commandsDropped.fetch_add(static_cast<UInt32>(left));
                return false;
            }
        }
        return true;
    }

    // ---- what the UI thread is told, in ONE go ------------------------------
    //
    // The panel's sentence and the control tally beside it are published
    // together, so they can never be read from two different passes: "live -
    // bibobox, rev 41" above "sent 0" would be two true statements that are
    // false as a pair, which is the shape of bug this repo keeps naming.
    struct Report
    {
        Phase phase = Phase::PHASE_IDLE;
        Str status;
        Int32 retryInMs = 0;
        Bool controlOnTcp = false;
        UInt32 controlSent = 0;
        UInt32 controlFailed = 0;
        UInt32 controlSeq = 0;
    };

    // ---- CONTROL ------------------------------------------------------------

    // One datagram onto the wire. `send` when the socket was connected to the
    // board (the filter that keeps strangers out), `sendto` when it could not
    // be - an address is better than not sending at all.
    [[nodiscard]] Bool sendControlUdp(Conn& c, const UInt8* frame, Size len)
    {
        if(c.udp == INVALID_SOCKET)
        {
            return false;
        }
        const Char* at = reinterpret_cast<const Char*>(frame);
        const Int32 want = static_cast<Int32>(len);
        if(c.udpFiltered)
        {
            return ::send(c.udp, at, want, 0) == want;
        }
        if(!c.haveBoardAddr)
        {
            return false;
        }
        const sockaddr* to = reinterpret_cast<const sockaddr*>(&c.boardAddr);
        return ::sendto(c.udp, at, want, 0, to, c.boardAddrLen) == want;
    }

    // EVERY PERIOD, CHANGED OR NOT, for as long as this viewer holds the slot.
    // Section 5 says "sent every 50 ms unconditionally" and section 6 says why:
    // the constant stream is what makes silence mean something, and there is no
    // separate heartbeat because a separate heartbeat is a thing that can keep
    // beating while the control path is dead.
    [[nodiscard]] Bool sendControl(Conn& c, Client& owner, const Session& s, Report& say)
    {
        // An OBSERVER sends nothing, and that is not a failure. Its datagrams
        // would be counted in the board's rxControlStale and discarded, and -
        // worse - a viewer streaming into a slot it does not hold is a viewer
        // whose own panel would look like it was driving.
        if(!s.haveWelcome || !holdsSlot(s))
        {
            return true;
        }

        ControlStamp at;
        at.sessionId = s.welcome.sessionId;
        at.seq = nextControlSeq(c.ctlSeq);
        at.armEpoch = currentEpoch(s);
        // The VIEWER's clock, which the board only ever compares with itself.
        at.tMonoUs = static_cast<UInt64>(monoMs()) * 1000u;

        const bibowire::Control m = buildControl(controlIntent(owner), at);
        Array<UInt8, 64> body = {};
        const Size n = bibowire::writeControl(m, body.data(), body.size());
        if(n == 0)
        {
            // The codec refused to encode it - a steer or throttle outside
            // +-1000 - which is a bug at THIS end rather than a link fault. The
            // connection is left alone and the refusal is counted, because a
            // stream that silently stopped encoding is the failure this repo
            // names as an absence.
            ++say.controlFailed;
            return true;
        }

        c.ctlSeq = at.seq;
        say.controlSeq = at.seq;

        if(say.controlOnTcp)
        {
            // The same frame on a different socket. The board accepts CONTROL
            // on TCP always, with identical rules, identical deadman and
            // identical session and seq checks, so there is nothing to
            // negotiate and no second code path. A TCP send that fails IS the
            // connection failing - unlike the datagram below.
            if(!sendFrame(c, bibowire::Type::TYPE_CONTROL, body.data(), n))
            {
                ++say.controlFailed;
                return false;
            }
            ++say.controlSent;
            return true;
        }

        bibowire::Head head;
        head.type = bibowire::Type::TYPE_CONTROL;
        head.ver = 1;
        head.seq = c.udpTxSeq;

        Array<UInt8, 96> frame = {};
        const bibowire::Body payload = { body.data(), n };
        const Size total = bibowire::put(head, payload, frame.data(), frame.size());
        if(total == 0)
        {
            ++say.controlFailed;
            return true;
        }
        ++c.udpTxSeq;

        // A DATAGRAM THAT DID NOT LEAVE IS NOT A DEAD CONNECTION, and this is
        // the one send in this file that may not tear the session down. Section
        // 7 is explicit that the viewer infers nothing from send() returning: a
        // board whose UDP socket is not up answers with ICMP port-unreachable,
        // which Windows reports on the NEXT send, and a viewer that redialled
        // over it would throw away a perfectly good telemetry stream. It is
        // counted, the fallback notices a reverse path that never worked, and
        // the deadman is what notices for real.
        if(!sendControlUdp(c, frame.data(), total))
        {
            ++say.controlFailed;
            return true;
        }
        ++say.controlSent;
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

    Void publish(Client& c, const Report& say, const Session& s)
    {
        LockGuard<Mutex> held(c.lock);
        c.shared.phase = say.phase;
        c.shared.status = say.status;
        c.shared.retryInMs = say.retryInMs;
        c.shared.controlOnTcp = say.controlOnTcp;
        c.shared.controlSent = say.controlSent;
        c.shared.controlFailed = say.controlFailed;
        c.shared.controlSeq = say.controlSeq;
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
            Report say;
            say.phase = Phase::PHASE_RETRYING;
            say.status = "retrying in " + numberText(left) + " ms - " + why;
            say.retryInMs = static_cast<Int32>(left);
            // The control tally is left at ZERO rather than carried across, and
            // that is the honest reading rather than a lost field: there is no
            // stream while there is no connection, and "sent 412" frozen on a
            // panel during a four-second retry is a count of a thing that
            // stopped four seconds ago.
            publish(c, say, s);
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

            // One connection's worth of what the UI is told, INCLUDING the
            // control tally - which starts at zero here for the same reason the
            // session does: a stream belongs to a connection.
            Report say;
            say.phase = Phase::PHASE_RESOLVING;
            say.status = "resolving " + c->host;
            publish(*c, say, live);

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

                // AND CONTROL GOES STRAIGHT TO TCP. The fallback below measures
                // a reverse path for 1000 ms before deciding; here there is no
                // socket for a datagram to leave by at all, so the measurement
                // has nothing to measure and the answer is already known.
                say.controlOnTcp = true;
            }
            say.phase = Phase::PHASE_CONNECTING;
            say.status = "connected to " + c->host;
            publish(*c, say, live);

            // HELLO IS THE FIRST BYTES ON THE CONNECTION, and nothing else is
            // sent until WELCOME arrives.
            // AND THE CONTROL SLOT IS ASKED FOR HERE OR NOT AT ALL. The board
            // grants it in its HELLO handler and in no other place, so this one
            // byte decides whether this whole connection can drive.
            if(!sendHello(conn, "bibo viewer", c->wantSlot.load()))
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

            // THE CONTROL DEADLINE, IN nextPingMs's SHAPE AND NOT A THREAD. One
            // select() loop honours both, which is what keeps the cadence
            // answerable to the same POLL_MS slice everything else here is: a
            // second thread ticking at 50 ms would be a second thing that can
            // still look alive while this one is wedged, and the whole point of
            // the stream is that its silence means something.
            //
            // Zero until WELCOME, which is also what arms it: there is no
            // session to stamp on a datagram before then, and no slot either.
            Int64 nextControlMs = 0;
            Int64 welcomeAtMs = 0;
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

                    // WELCOME starts both control clocks: section 4's
                    // reverse-path window, and the stream's own cadence - which
                    // is the BOARD's controlPeriodMs and not a number compiled
                    // into this viewer months earlier. The first datagram goes
                    // on this pass rather than a period later, because the
                    // board's probe wants five of them inside 1000 ms.
                    welcomeAtMs = monoMs();
                    nextControlMs = welcomeAtMs;
                }

                // Re-asserted every pass, because the answer can change at any
                // moment: the operator closes the camera window mid-outing, or
                // the link drops and comes back and the new session has been
                // told nothing. `sentMask` went with the old connection, so a
                // window that was open before the drop is asked for again.
                if(!syncSubscription(conn, live, c->cameraOn.load(), c->cameraFps.load()))
                {
                    why = "could not send SUBSCRIBE";
                    break;
                }

                // After SUBSCRIBE and in the same pass, so a command typed while
                // the link was live is on the wire within one POLL_MS rather
                // than waiting for a frame to arrive first.
                if(!flushCommands(conn, *c, live))
                {
                    why = "could not send a COMMAND";
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

                // SECTION 4'S TCP FALLBACK, MEASURED AND THEN LATCHED. CTLSTATE
                // is the only thing that rides UDP toward this viewer, so a
                // silent 1000 ms after WELCOME is a measurement that UDP is not
                // working in at least one direction rather than a guess - this
                // repo's rule about reverse paths, applied to the link itself.
                //
                // It latches for the life of the connection, and that is the
                // part worth naming: while the fallback is active the board
                // MIRRORS CTLSTATE onto TCP, so a test that kept asking "has a
                // CTLSTATE arrived lately" would flip straight back to UDP the
                // moment the mirror answered, and then flap once a second
                // between two transports while the car was being driven.
                const Int64 probeMs = static_cast<Int64>(bibowire::REVERSE_PROBE_MS);
                if(!say.controlOnTcp && welcomeAtMs > 0 && !live.haveControl
                   && now - welcomeAtMs > probeMs)
                {
                    say.controlOnTcp = true;
                    Note note;
                    note.severity = bibowire::Severity::SEVERITY_WARN;
                    note.text = "no CTLSTATE for " + numberText(now - welcomeAtMs)
                                + " ms - degraded control (TCP)";
                    note.atMs = now;
                    live.notes.push_back(note);
                }

                // EVERY PERIOD, CHANGED OR NOT, and a period that is the
                // BOARD's. sendControl decides there is nothing to send when
                // this viewer is an observer; the deadline rolls either way, so
                // a slot taken on a later connection starts its stream on the
                // same schedule rather than whenever a key was first pressed.
                if(live.haveWelcome && now >= nextControlMs)
                {
                    nextControlMs = now + controlPeriodMs(live);
                    if(!sendControl(conn, *c, live, say))
                    {
                        why = "could not send a CONTROL";
                        break;
                    }
                }

                say.phase = live.haveWelcome ? Phase::PHASE_LIVE : Phase::PHASE_HANDSHAKING;
                say.status = liveStatus(live, c->host, now);
                say.retryInMs = 0;
                publish(*c, say, live);
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

  // ---- what a feed is delivering at ------------------------------------------

  Void noteArrival(Cadence& c, Int64 nowMs)
  {
      if(!c.have)
      {
          // THE FIRST FRAME STARTS THE CLOCK AND NOTHING ELSE. There is no gap
          // before a feed's first arrival, and inventing one would measure the
          // moment this viewer happened to connect rather than anything the
          // board is doing.
          c.have = true;
          c.lastAtMs = nowMs;
          return;
      }

      const Int64 gap = nowMs - c.lastAtMs;
      c.lastAtMs = nowMs;

      // A clock that went backwards, or two frames stamped inside one
      // millisecond. Neither is an interval this feed delivered at, and a
      // negative one would poison the maximum in the wrong direction.
      if(gap <= 0)
      {
          return;
      }

      c.gaps[c.at] = gap;
      c.at = (c.at + 1u) % CADENCE_SAMPLES;
      if(c.count < CADENCE_SAMPLES)
      {
          ++c.count;
      }
  }

  Int64 worstGapMs(const Cadence& c)
  {
      Int64 worst = 0;
      // `count` and not CADENCE_SAMPLES: the unfilled tail of a fresh ring is
      // zeros, and while zeros cannot raise a maximum, reading them would make
      // this loop's correctness depend on that coincidence.
      for(Size i = 0; i < c.count; ++i)
      {
          if(c.gaps[i] > worst)
          {
              worst = c.gaps[i];
          }
      }
      return worst;
  }

  Int64 staleBandMs(const Cadence& c)
  {
      const Int64 worst = worstGapMs(c);
      if(worst <= 0)
      {
          // Nothing measured yet, so the feed is held to section 7's number
          // until it has earned a different one.
          return FRESH_MS;
      }

      const Int64 band = (worst * STALE_SLACK_NUM) / STALE_SLACK_DEN;
      if(band < FRESH_MS)
      {
          // MEASURING CAN NEVER TIGHTEN THE BAND. A feed arriving every 20 ms
          // is not thereby promised to be called stale at 30, because the
          // number that matters to a person reading the screen is section 7's
          // and this function may only ever widen it for a feed that is slower.
          return FRESH_MS;
      }
      if(band > STALE_CEIL_MS)
      {
          // And never so wide that "stale" stops existing. Past this the feed
          // would go straight from live to gone, and the band that warns
          // somebody the picture is aging is the one thing between those two.
          return STALE_CEIL_MS;
      }
      return band;
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
      r.staleAtMs = staleBandMs(scanRate);
      r.worstGapMs = worstGapMs(scanRate);
      r.stale = age > r.staleAtMs;
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
      // The SCAN's band, because a DECIDE is tied to the revolution it
      // describes and shares its age exactly - the same reason it borrows that
      // revolution's board clock a few lines up in ingestFrame.
      d.stale = age > staleBandMs(scanRate);
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
      b.stale = age > staleBandMs(boardRate);
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
      c.stale = age > staleBandMs(controlRate);
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
      // THE WHOLE POINT OF THE BAND, and the bug it was written for: at the
      // board's 2 fps default a picture is ~500 ms old the instant before its
      // successor arrives, so a fixed FRESH_MS of 400 called a camera that was
      // perfectly on time STALE for the last 100 ms of every single frame.
      // Measured against the real board that was 57 frames out of 57.
      shot.staleAtMs = staleBandMs(cameraRate);
      shot.worstGapMs = worstGapMs(cameraRate);
      shot.stale = age > shot.staleAtMs;
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
          noteArrival(s.scanRate, nowMs);
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
          noteArrival(s.boardRate, nowMs);
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
          noteArrival(s.controlRate, nowMs);
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
          noteArrival(s.cameraRate, nowMs);
          noteOffset(s, nowMs, m.tMonoUs);
          // Moved rather than copied: the body is a whole JPEG, tens of
          // kilobytes, and this runs on the network thread several times a
          // second.
          s.camera = std::move(m);
          return;
      }

      case bibowire::Type::TYPE_CMDACK:
      {
          bibowire::CmdAck m;
          if(!bibowire::readCmdAck(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          // KEPT WITH ITS SENTENCE, VERBATIM. result = 1 or 3 is a refusal, and
          // the board explains it in words - "refused while armed" is the whole
          // difference between a slider that appears broken and one that is
          // doing exactly what the protocol says. A viewer that kept only the
          // result byte would leave an operator with a number and no reason.
          Ack a;
          a.ack = m;
          a.atMs = nowMs;
          s.acks.push_back(a);
          while(s.acks.size() > MAX_ACKS)
          {
              s.acks.erase(s.acks.begin());
          }
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

  Void wantCameraFps(Client& c, Int32 fps)
  {
      // Clamped HERE as well as in syncSubscription and again on the board.
      // Not redundancy for its own sake: this is the value the UI reads back to
      // show what was asked for, and a readout that echoed 60 while the wire
      // carried 15 would be a number describing nothing.
      const Int32 ceiling = static_cast<Int32>(bibowire::CAM_FPS_MAX);
      const Int32 held = fps < 0 ? 0 : (fps > ceiling ? ceiling : fps);
      c.cameraFps.store(held);
  }

  Int32 cameraFpsWanted(const Client& c)
  {
      return c.cameraFps.load();
  }

  // ---- COMMAND ---------------------------------------------------------------

  Void sendCommand(Client& c, bibowire::Verb verb, UInt8 arg0, UInt16 arg1, UInt16 arg2)
  {
      LockGuard<Mutex> held(c.cmdLock);

      bibowire::Command cmd;
      cmd.cmdId = c.nextCmdId;
      cmd.verb = verb;
      cmd.arg0 = arg0;
      cmd.arg1 = arg1;
      cmd.arg2 = arg2;
      // sessionId and armEpoch are stamped by the worker at the moment of
      // sending - see link.hxx. Left at their defaults here on purpose, so a
      // reader of this function cannot mistake a snapshot for the connection.

      ++c.nextCmdId;
      if(c.nextCmdId == 0u)
      {
          // NEVER 0. Unreachable at any human rate - it is 4.2 billion
          // deliberate acts - and written anyway, because the alternative is a
          // rule enforced by an arithmetic coincidence.
          c.nextCmdId = 1u;
      }

      c.pending.push_back(cmd);
      while(c.pending.size() > MAX_PENDING_COMMANDS)
      {
          // The OLDEST goes, and it is counted. A queue this deep means the
          // worker is not draining, and in that case the newest intent is the
          // one worth keeping - the same newest-wins rule the rest of this
          // protocol follows.
          c.pending.erase(c.pending.begin());
          c.commandsDropped.fetch_add(1u);
      }
  }

  UInt32 commandsDropped(const Client& c)
  {
      return c.commandsDropped.load();
  }

  CharSeq ackResultName(UInt8 result)
  {
      switch(result)
      {
      case 0:
          return "ok";
      case 1:
          return "refused";
      case 2:
          return "unknown verb";
      case 3:
          return "not in this state";
      case 4:
          return "no Pico";
      default:
          break;
      }
      return "?";
  }

  Opt<Ack> Session::newestAck() const
  {
      if(acks.empty())
      {
          return {};
      }
      return acks[acks.size() - 1u];
  }

  // ---- CONTROL ---------------------------------------------------------------

  bibowire::Control buildControl(const Intent& in, const ControlStamp& at)
  {
      bibowire::Control m;
      m.sessionId = at.sessionId;
      m.seq = at.seq;
      m.tMonoUs = at.tMonoUs;

      // NEUTRAL WHEN THE OPERATOR IS NOT DRIVING, and it is written HERE as
      // well as in the pane on purpose. Steering is applied by the board even
      // while throttle is refused - section 6, and it is right to, because a
      // car that snaps to centre mid-corner changes its line at the moment it
      // stopped being commanded - so a viewer that kept sending a steer angle
      // after its operator switched driving off would still be steering the
      // car. This is the last place before the wire, which makes it the one
      // place the rule cannot be bypassed by a caller that forgot.
      // Cast written out rather than left to the assignment. Both arms are in
      // range - an Int16 or a literal 0 - so nothing is lost either way, but a
      // ternary mixing Int16 with an int literal promotes to int and narrows
      // back implementation-defined on the way in. These are the two fields that
      // carry steering and throttle; they are the last two in this program worth
      // leaving to a conversion nobody wrote down.
      m.steerMilli = static_cast<Int16>(in.driving ? in.steerMilli : 0);
      m.throttleMilli = static_cast<Int16>(in.driving ? in.throttleMilli : 0);

      // ESTOP SURVIVES THE ENABLE BEING OFF; ENABLE CANNOT SURVIVE IT. The
      // stop is the one thing that must work in every state this viewer can be
      // in, and the consent is the one thing that must never be asserted by
      // accident - so they are masked in opposite directions.
      m.buttons = in.driving
          ? in.buttons
          : static_cast<UInt16>(in.buttons & bibowire::BUTTON_ESTOP);

      m.armEpoch = at.armEpoch;
      m.assumedMode = in.assumedMode;
      return m;
  }

  UInt32 nextControlSeq(UInt32 previous)
  {
      const UInt32 next = previous + 1u;
      // 0 IS NOT A SEQ. The board's newest-wins test is a signed difference, so
      // 0 is a perfectly ordinary number to it - but section 5 starts the
      // stream at 1, and CTLSTATE's ackSeq uses 0 for "none applied yet", so a
      // datagram numbered 0 is one the board could never report having run.
      return next == 0u ? 1u : next;
  }

  Bool holdsSlot(const Session& s)
  {
      // 1 is "control is yours", 2 is "observing" - and a WELCOME that refused
      // the connection outright (0) is neither. Asked of the BOARD's answer and
      // never of what this end wanted, because those are different facts and
      // the difference is a car.
      return s.haveWelcome && s.welcome.accepted == 1u;
  }

  Int64 controlPeriodMs(const Session& s)
  {
      const Int64 fallback = static_cast<Int64>(bibowire::CONTROL_PERIOD_MS);
      if(!s.haveWelcome || s.welcome.controlPeriodMs == 0u)
      {
          // A board that sends 0 does not get to make this viewer spin: a
          // period of zero is not a faster stream, it is a busy loop that would
          // saturate the link the stream is trying to survive on.
          return fallback;
      }
      return static_cast<Int64>(s.welcome.controlPeriodMs);
  }

  Void setControl(Client& c, const Intent& in)
  {
      LockGuard<Mutex> held(c.ctlLock);
      c.intent = in;
  }

  Intent controlIntent(Client& c)
  {
      LockGuard<Mutex> held(c.ctlLock);
      return c.intent;
  }

  Void wantControlSlot(Client& c, Bool on)
  {
      c.wantSlot.store(on);
  }

  Bool controlSlotWanted(const Client& c)
  {
      return c.wantSlot.load();
  }

  Bool reconnect(Client& c)
  {
      if(!c.running.load())
      {
          return false;
      }
      // Copied BEFORE the close, because open() takes them as arguments and
      // this is the one call site where the source and the destination are the
      // same object.
      const Str host = c.host;
      const UInt16 port = c.port;
      close(c);
      return open(c, host.c_str(), port);
  }

}
