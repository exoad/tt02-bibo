#include "link.hxx"

// Names no Windows header, so it may sit above the Winsock block.
#include "vlog.hxx"

// Winsock before anything that might include <windows.h>: winsock2.h and the old
// winsock.h define the same symbols, and whichever arrives second loses. Nothing
// above this line includes a Windows header.
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>

// <winnt.h> defines SEVERITY_ERROR as a macro, and bibowire::Severity has a
// member of that name, so later uses must mean the enum.
#undef SEVERITY_SUCCESS
#undef SEVERITY_ERROR

#include <cmath>
#include <cstdio>
#include <cstring>

namespace link
{
  namespace
  {
    // The lidar's height above the ground. Not a mounting correction:
    // docs/hardware.md records the lidar-to-vehicle transform as not
    // established, and rotating or offsetting the cloud would bake a guess into
    // every point. When the transform is measured, it applies here, once.
    constexpr Float32 LIDAR_HEIGHT_M = 0.16f;

    // The wire has no floating point, so this multiply is where a scan first
    // becomes inexact.
    constexpr Float32 CENTI_DEG_TO_RAD = 3.14159265358979f / 18000.0f;

    constexpr Float32 MM_TO_M = 0.001f;

    // Section 3: a 512 KiB receive ring, allocated once per connection, so no
    // allocation is sized from a number off the network. Size{512}, not 512u:
    // the product is computed in its operands' type before it is widened.
    constexpr Size RX_BYTES = Size{512} * 1024;

    // Section 2: viewer SO_RCVBUF 256 KiB.
    constexpr Int32 RCVBUF_BYTES = 256 * 1024;

    // One select() slice. It bounds how long a Disconnect waits and is far below
    // the 50 ms cadence of the fastest thing on the wire.
    constexpr Int32 POLL_MS = 20;

    // A frame this viewer sends. HELLO with a 31-byte name is 72 bytes and PONG
    // 32; the board accepts no more than MAX_INBOUND_PAYLOAD.
    constexpr Size TX_BYTES = 512;

    // Section 2's keepalive as Windows takes it. Windows has no TCP_KEEPCNT; its
    // probe count is fixed at 10.
    constexpr UInt32 KEEPALIVE_IDLE_MS = 2000;
    constexpr UInt32 KEEPALIVE_INTERVAL_MS = 1000;

    // How long a send may wait for a writable socket before the connection is
    // declared broken. Everything sent is one small frame, so a socket that
    // cannot take it in this long is gone.
    constexpr Int64 SEND_BUDGET_MS = 500;

    // The round-trip log's summary window, fine enough to line up with the
    // board's 1 Hz PING and its "no PONG" in the board's journal.
    constexpr Int64 SUMMARY_MS = 1000;

    // A negative age is a bug upstream, and the safe reading of a bug is "old":
    // the rule bibowire::deadman applies to nowMs < lastControlMs.
    [[nodiscard]] Int64 ageFrom(Int64 nowMs, Int64 atMs)
    {
        const Int64 age = nowMs - atMs;
        return age < 0 ? (GONE_MS + 1) : age;
    }

    // The larger of the age since local arrival and the age through the board's
    // clock (Session::offsetMs).
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

    // Only a round trip separates clock offset from transit time: the board
    // stamped its PONG at boardUs, and the stamp took half the round trip to
    // arrive. Taken from the fastest kept sample and folded into the arrival
    // floor's minimum, so it can only make a value older, never younger.
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

    // Integer digits, never "%.1f" (the locale trap, vlog.hxx).
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

    // Names for the round-trip log only; nothing drawn or decided reads them.
    // bibowire.cxx's spellings are private to the codec, which is shared with
    // the board. Every log line prints the number beside the name, so an unknown
    // value shows as "?" with its number.
    [[nodiscard]] CharSeq verbText(bibowire::Verb v)
    {
        switch(v)
        {
        case bibowire::Verb::VERB_NONE:
            return "none";
        case bibowire::Verb::VERB_ARM:
            return "arm";
        case bibowire::Verb::VERB_DISARM:
            return "disarm";
        case bibowire::Verb::VERB_ESTOP:
            return "estop";
        case bibowire::Verb::VERB_CLEAR_ESTOP:
            return "clear_estop";
        case bibowire::Verb::VERB_MOTOR_ON:
            return "motor_on";
        case bibowire::Verb::VERB_MOTOR_OFF:
            return "motor_off";
        case bibowire::Verb::VERB_SET_MODE:
            return "set_mode";
        case bibowire::Verb::VERB_SET_ESC_LIMITS:
            return "set_esc_limits";
        case bibowire::Verb::VERB_SET_SERVO_LIMITS:
            return "set_servo_limits";
        case bibowire::Verb::VERB_SET_SERVO_TRIM:
            return "set_servo_trim";
        case bibowire::Verb::VERB_SET_SLEW:
            return "set_slew";
        case bibowire::Verb::VERB_SET_ESC_REVERSE:
            return "set_esc_reverse";
        default:
            break;
        }
        return "?";
    }

    [[nodiscard]] CharSeq reasonText(bibowire::Reason r)
    {
        switch(r)
        {
        case bibowire::Reason::REASON_NONE:
            return "none";
        case bibowire::Reason::REASON_VERSION:
            return "version";
        case bibowire::Reason::REASON_TOO_BIG:
            return "too_big";
        case bibowire::Reason::REASON_BAD_CRC:
            return "bad_crc";
        case bibowire::Reason::REASON_BAD_SESSION:
            return "bad_session";
        case bibowire::Reason::REASON_TIMEOUT:
            return "timeout";
        case bibowire::Reason::REASON_SHUTDOWN:
            return "shutdown";
        case bibowire::Reason::REASON_SUPERSEDED:
            return "superseded";
        case bibowire::Reason::REASON_REFUSED:
            return "refused";
        case bibowire::Reason::REASON_BAD_FLAG:
            return "bad_flag";
        default:
            break;
        }
        return "?";
    }

    [[nodiscard]] CharSeq severityText(bibowire::Severity s)
    {
        switch(s)
        {
        case bibowire::Severity::SEVERITY_INFO:
            return "info";
        case bibowire::Severity::SEVERITY_WARN:
            return "warn";
        case bibowire::Severity::SEVERITY_ERROR:
            return "error";
        default:
            break;
        }
        return "?";
    }

    [[nodiscard]] CharSeq typeText(UInt8 tag)
    {
        return bibowire::knownType(tag) ? bibowire::typeName(static_cast<bibowire::Type>(tag)) : "?";
    }

    // A socket error as its number and Windows' own sentence for it.
    [[nodiscard]] Str wsaText(Int32 code)
    {
        Array<Char, 192> t = {};
        const DWORD flags = FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS;
        const DWORD n = ::FormatMessageA(
            flags,
            nullptr,
            static_cast<DWORD>(code),
            0,
            t.data(),
            static_cast<DWORD>(t.size()),
            nullptr
        );
        Size end = n;
        while(end > 0 && (t[end - 1] == '\r' || t[end - 1] == '\n' || t[end - 1] == ' '))
        {
            --end;
        }
        Str out = numberText(code);
        if(end > 0)
        {
            out += " ";
            out += Str(t.data(), end);
        }
        return out;
    }

    // An address the way a person types one: 100.101.3.7:8020, [fd7a::1]:8020.
    [[nodiscard]] Str addressText(const sockaddr* a)
    {
        Array<Char, 64> host = {};
        Array<Char, 96> t = {};
        if(a->sa_family == AF_INET)
        {
            const sockaddr_in* v4 = reinterpret_cast<const sockaddr_in*>(a);
            ::inet_ntop(AF_INET, &v4->sin_addr, host.data(), host.size());
            const UInt32 port = ::ntohs(v4->sin_port);
            std::snprintf(t.data(), t.size(), "%s:%u", host.data(), port);
            return Str(t.data());
        }
        if(a->sa_family == AF_INET6)
        {
            const sockaddr_in6* v6 = reinterpret_cast<const sockaddr_in6*>(a);
            ::inet_ntop(AF_INET6, &v6->sin6_addr, host.data(), host.size());
            const UInt32 port = ::ntohs(v6->sin6_port);
            std::snprintf(t.data(), t.size(), "[%s]:%u", host.data(), port);
            return Str(t.data());
        }
        return "address family " + numberText(a->sa_family);
    }

    // One end of a connected socket. Through Tailscale the peer is a 100.x
    // address, so both ends show which path a connection took.
    [[nodiscard]] Str endpointText(SOCKET fd, Bool peer)
    {
        sockaddr_storage at = {};
        Int32 len = static_cast<Int32>(sizeof(at));
        sockaddr* atAddr = reinterpret_cast<sockaddr*>(&at);
        const Int32 rc = peer ? ::getpeername(fd, atAddr, &len) : ::getsockname(fd, atAddr, &len);
        if(rc != 0)
        {
            return "unknown (" + wsaText(::WSAGetLastError()) + ")";
        }
        return addressText(atAddr);
    }

    // Whether a sentence mentions the camera (Session::haveCameraNote's
    // heuristic). EVENT text is ASCII (section 5), so this lowercases by hand,
    // not with the locale's std::tolower.
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

    // What one send did, so the log can say how much the socket took, how often
    // it would not, and how long that took.
    struct SendNote
    {
        Bool ok = false;
        Size offered = 0;
        Size taken = 0;
        UInt32 calls = 0;
        UInt32 shortCalls = 0;
        UInt32 wouldBlock = 0;
        Int32 error = 0;        // the WSA code that ended it; 0 when nothing did
        Int64 tookMs = 0;
    };

    // One worker pass by where its time went. `logMs`, the drain of Heard into
    // the file, is measured apart so the log cannot hide its own cost.
    struct PassSplit
    {
        Int64 selectMs = 0;
        Int64 recvMs = 0;
        Int64 logMs = 0;
        Int64 sendMs = 0;
        Int64 publishMs = 0;
    };

    // One SUMMARY_MS window of the socket half, written as the summary line and
    // zeroed.
    struct Wire
    {
        Int64 windowMs = 0;
        UInt64 rxBytes = 0;
        UInt64 txBytes = 0;
        UInt32 recvCalls = 0;
        UInt32 recvWouldBlock = 0;
        UInt32 recvZero = 0;
        UInt32 recvErrors = 0;
        Int64 worstRecvGapMs = 0;
        UInt32 sends = 0;
        UInt32 sendShort = 0;
        UInt32 sendWouldBlock = 0;
        UInt32 sendFailed = 0;
        Int64 worstSendMs = 0;
        UInt32 udpTx = 0;
        UInt32 udpTxFailed = 0;
        UInt32 udpRx = 0;
        UInt32 udpRxBad = 0;
        UInt32 udpRxErrors = 0;
        UInt32 passes = 0;
        Int64 worstPassMs = 0;
        PassSplit worstSplit;
        Size worstPongsDue = 0;
    };

    // A COMMAND sent and not yet answered, so its CMDACK line can say how long
    // the board took.
    struct CmdOut
    {
        UInt32 cmdId = 0;
        Int64 sentMs = 0;
    };

    struct Journal
    {
        Wire wire;
        SendNote lastSend;
        Heard heard;

        // Frame counts at the previous summary, so a line shows this window's
        // arrivals rather than the connection's.
        Array<UInt32, 256> byTypeAtSummary = {};

        Int64 openedMs = 0;
        Int64 helloAtMs = 0;
        Int64 lastRecvMs = 0;       // the last recv() that returned bytes
        UInt64 rxTotal = 0;
        UInt64 txTotal = 0;

        // Why recv or select ended the connection, when one did: the detail
        // behind the panel's `why`.
        Str endedBy;

        // The first CONTROL datagram failure is logged and the rest counted; the
        // last receive error goes into the summary.
        Int32 udpError = 0;
        Int32 udpRxError = 0;
        Bool saidUdpFailure = false;

        // The first of each kind is logged once per connection, then counted.
        Bool saidRefused = false;
        Bool saidUnknown = false;
        Bool saidResync = false;
        Bool saidFraming = false;
        Bool saidSeqJump = false;
        Bool saidOverflow = false;
        Bool saidBye = false;

        // CONTROL as last logged, so a line is written only on change.
        Bool haveControlLine = false;
        bibowire::Control controlLine;
        Bool controlLineOnTcp = false;

        Vec<CmdOut> commandsOut;
    };

    // Kept out of link.hxx so no file that draws a panel needs <winsock2.h>.
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

        // Per session, restarting at 1 on every reconnect: section 6 resets the
        // board's high-water mark at the handshake and compares the signed
        // difference.
        UInt32 ctlSeq = 0;

        // The frame header counter for the UDP stream, apart from txSeq: seqs
        // are read per stream, and one counter would make TCP look like it lost
        // every frame UDP sent.
        UInt16 udpTxSeq = 0;

        // The board's address, learned from the TCP peer. Kept even when
        // connect() on the UDP socket failed, so CONTROL can still go by sendto.
        sockaddr_storage boardAddr = {};
        Int32 boardAddrLen = 0;
        Bool haveBoardAddr = false;

        // What this connection has asked the board for. 0 is "never asked",
        // which differs from asking for nothing (syncSubscription). The board
        // keeps no subscription across sessions either.
        UInt32 sentMask = 0;

        // And the rate, so a slider change re-sends without a mask change.
        UInt16 sentFps = 0;

        // Reset with the connection, so no log tally blurs across a reconnect.
        Journal journal;
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
        c.journal = Journal();
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

    // `note` records what the socket did, whether or not the send succeeded.
    [[nodiscard]] Bool sendAll(SOCKET fd, const UInt8* buf, Size len, SendNote& note)
    {
        note = SendNote();
        note.offered = len;
        const Int64 started = monoMs();
        const Int64 deadline = started + SEND_BUDGET_MS;
        Size sent = 0;
        while(sent < len)
        {
            const Char* at = reinterpret_cast<const Char*>(buf + sent);
            const Int32 want = static_cast<Int32>(len - sent);
            const Int32 n = ::send(fd, at, want, 0);
            ++note.calls;
            if(n > 0)
            {
                if(n < want)
                {
                    ++note.shortCalls;
                }
                sent += static_cast<Size>(n);
                note.taken = sent;
                continue;
            }
            const Int32 err = n == 0 ? 0 : ::WSAGetLastError();
            if(n == 0 || err != WSAEWOULDBLOCK)
            {
                note.error = err;
                note.tookMs = monoMs() - started;
                return false;
            }
            ++note.wouldBlock;
            const Int64 left = deadline - monoMs();
            if(left <= 0 || !waitWritable(fd, left))
            {
                // The budget ran out with the socket still full; no call
                // returned an error.
                note.error = WSAEWOULDBLOCK;
                note.tookMs = monoMs() - started;
                return false;
            }
        }
        note.ok = true;
        note.tookMs = monoMs() - started;
        return true;
    }

    Void noteSend(Journal& j, const SendNote& note)
    {
        ++j.wire.sends;
        j.wire.txBytes += note.taken;
        j.txTotal += note.taken;
        j.wire.sendShort += note.shortCalls;
        j.wire.sendWouldBlock += note.wouldBlock;
        if(!note.ok)
        {
            ++j.wire.sendFailed;
        }
        if(note.tookMs > j.wire.worstSendMs)
        {
            j.wire.worstSendMs = note.tookMs;
        }
    }

    [[nodiscard]] Str sendText(const SendNote& note)
    {
        Array<Char, 128> t = {};
        std::snprintf(
            t.data(),
            t.size(),
            "%s %zu/%zu B in %u call(s), %u short, %u would-block, %lld ms",
            note.ok ? "ok" : "FAILED",
            note.taken,
            note.offered,
            note.calls,
            note.shortCalls,
            note.wouldBlock,
            note.tookMs
        );
        Str out = t.data();
        if(!note.ok && note.error != 0)
        {
            out += ", error " + wsaText(note.error);
        }
        return out;
    }

    // One typed frame out on TCP. The header is assembled here and nowhere else,
    // so no call site picks its own seq.
    [[nodiscard]] Bool sendFrame(Conn& c, bibowire::Type type, const UInt8* body, Size bodyLen)
    {
        // Cleared first, so a frame the codec refused reads as nothing offered
        // rather than as the previous send's success.
        c.journal.lastSend = SendNote();
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
        const Bool ok = sendAll(c.tcp, frame.data(), n, c.journal.lastSend);
        noteSend(c.journal, c.journal.lastSend);
        return ok;
    }

    // `err` is SO_ERROR when the socket has one and WSAETIMEDOUT when the budget
    // ran out, so a refused port and a silent board log differently.
    [[nodiscard]] Bool finishConnect(SOCKET fd, Int64 budgetMs, Int32& err)
    {
        const Bool writable = waitWritable(fd, budgetMs);
        // Writable is not connected: a refused connection is reported by
        // SO_ERROR.
        Int32 soError = 0;
        Int32 len = static_cast<Int32>(sizeof(soError));
        Char* slot = reinterpret_cast<Char*>(&soError);
        if(::getsockopt(fd, SOL_SOCKET, SO_ERROR, slot, &len) != 0)
        {
            err = ::WSAGetLastError();
            return false;
        }
        if(soError != 0)
        {
            err = soError;
            return false;
        }
        err = writable ? 0 : WSAETIMEDOUT;
        return writable;
    }

    Void tuneTcp(SOCKET fd)
    {
        Int32 on = 1;
        const Char* onBytes = reinterpret_cast<const Char*>(&on);
        // A small frame must not wait in Nagle's queue behind a SCAN.
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, onBytes, static_cast<Int32>(sizeof(on)));
        ::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, onBytes, static_cast<Int32>(sizeof(on)));
        Int32 rcvbuf = RCVBUF_BYTES;
        const Char* rcvBytes = reinterpret_cast<const Char*>(&rcvbuf);
        ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, rcvBytes, static_cast<Int32>(sizeof(rcvbuf)));
        // A phone that walks out of range stops ACKing without a FIN, and the
        // default keepalive is two hours. SILENCE_MS usually catches it first.
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

    // Resolves the name on every attempt, never a cached address (CONNECT_MS).
    [[nodiscard]] Bool dialTcp(Conn& c, const Str& host, UInt16 port, Str* why)
    {
        const Int64 dialMs = monoMs();
        vlog::line("dial %s port %u: resolving", host.c_str(), static_cast<UInt32>(port));
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
            vlog::line(
                "resolve %s FAILED after %lld ms: getaddrinfo %s",
                host.c_str(),
                monoMs() - dialMs,
                wsaText(rc).c_str()
            );
            *why = "cannot resolve " + host + " - is the board on this network?";
            return false;
        }
        // Every address the name produced, logged before any is tried.
        for(addrinfo* a = found; a != nullptr; a = a->ai_next)
        {
            vlog::line("resolved %s -> %s", host.c_str(), addressText(a->ai_addr).c_str());
        }
        const Int64 deadline = monoMs() + CONNECT_MS;
        *why = "no answer from " + host + " within " + numberText(CONNECT_MS) + " ms";
        for(addrinfo* a = found; a != nullptr; a = a->ai_next)
        {
            const Str target = addressText(a->ai_addr);
            const Int64 left = deadline - monoMs();
            if(left <= 0)
            {
                vlog::line("connect %s skipped: the dial deadline is spent", target.c_str());
                break;
            }
            const Int64 triedMs = monoMs();
            const SOCKET fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
            if(fd == INVALID_SOCKET)
            {
                const Str err = wsaText(::WSAGetLastError());
                vlog::line("connect %s: socket() FAILED %s", target.c_str(), err.c_str());
                continue;
            }
            setNonBlocking(fd);
            const Int32 len = static_cast<Int32>(a->ai_addrlen);
            const Int32 answer = ::connect(fd, a->ai_addr, len);
            Int32 err = answer == 0 ? 0 : ::WSAGetLastError();
            Bool up = answer == 0;
            if(!up && err == WSAEWOULDBLOCK)
            {
                up = finishConnect(fd, left, err);
            }
            if(!up)
            {
                vlog::line(
                    "connect %s FAILED after %lld ms: %s",
                    target.c_str(),
                    monoMs() - triedMs,
                    wsaText(err).c_str()
                );
                ::closesocket(fd);
                continue;
            }
            tuneTcp(fd);
            c.tcp = fd;
            c.rx.assign(RX_BYTES, 0);
            c.rxUsed = 0;
            c.journal.openedMs = monoMs();
            vlog::line(
                "connected %s -> %s in %lld ms, %lld ms after the dial began",
                endpointText(fd, false).c_str(),
                endpointText(fd, true).c_str(),
                monoMs() - triedMs,
                monoMs() - dialMs
            );
            ::freeaddrinfo(found);
            return true;
        }
        vlog::line(
            "dial %s FAILED after %lld ms: %s",
            host.c_str(),
            monoMs() - dialMs,
            why->c_str()
        );
        ::freeaddrinfo(found);
        return false;
    }

    // Bound before HELLO, which carries the port the board sends CTLSTATE to.
    // CONTROL datagrams leave by the same socket.
    [[nodiscard]] Bool openUdp(Conn& c)
    {
        const SOCKET fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(fd == INVALID_SOCKET)
        {
            vlog::line("UDP socket FAILED: %s", wsaText(::WSAGetLastError()).c_str());
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
            vlog::line("UDP bind FAILED: %s", wsaText(::WSAGetLastError()).c_str());
            ::closesocket(fd);
            return false;
        }
        sockaddr_in got = {};
        Int32 gotLen = static_cast<Int32>(sizeof(got));
        sockaddr* gotAddr = reinterpret_cast<sockaddr*>(&got);
        if(::getsockname(fd, gotAddr, &gotLen) != 0)
        {
            vlog::line("UDP getsockname FAILED: %s", wsaText(::WSAGetLastError()).c_str());
            ::closesocket(fd);
            return false;
        }
        c.udp = fd;
        c.udpPort = ::ntohs(got.sin_port);
        vlog::line("UDP bound on local port %u", static_cast<UInt32>(c.udpPort));
        return true;
    }

    // Point the UDP socket at the board once WELCOME has named the port.
    // CTLSTATE carries no sessionId, so this filter is all that stops anyone
    // else on the hotspot telling this viewer the car is armed.
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
            vlog::line(
                "UDP filter not applied: getpeername %s",
                wsaText(::WSAGetLastError()).c_str()
            );
            return;
        }
        if(peer.ss_family == AF_INET)
        {
            reinterpret_cast<sockaddr_in*>(&peer)->sin_port = ::htons(boardPort);
            // Only for AF_INET: this socket cannot send to an IPv6 board, and
            // recording one would fail every sendto instead of letting the
            // fallback move CONTROL onto TCP.
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
            vlog::line(
                "UDP filter not applied: TCP peer family %u",
                static_cast<UInt32>(peer.ss_family)
            );
            return;
        }
        // An AF_INET socket cannot connect to an AF_INET6 peer; then there is no
        // filter, and datagrams are still CRC-checked.
        c.udpFiltered = ::connect(c.udp, peerAddr, len) == 0;
        Str result = "connected - datagrams from anywhere else are dropped";
        if(!c.udpFiltered)
        {
            result = "NOT connected (" + wsaText(::WSAGetLastError()) + ")";
            result += c.haveBoardAddr ? " - CONTROL goes by sendto" : " - no address to send CONTROL to";
        }
        vlog::line(
            "UDP pointed at the board %s: %s",
            addressText(peerAddr).c_str(),
            result.c_str()
        );
    }

    [[nodiscard]] Bool pumpTcp(Conn& c, Session& s)
    {
        Journal& j = c.journal;
        for(;;)
        {
            if(c.rxUsed >= c.rx.size())
            {
                // A full ring holds more than MAX_PAYLOAD, so this is not a big
                // frame but bytes that will never parse.
                j.endedBy = "the 512 KiB receive ring filled with no frame in it";
                return false;
            }
            Char* at = reinterpret_cast<Char*>(c.rx.data() + c.rxUsed);
            const Int32 want = static_cast<Int32>(c.rx.size() - c.rxUsed);
            const Int32 n = ::recv(c.tcp, at, want, 0);
            ++j.wire.recvCalls;
            if(n == 0)
            {
                ++j.wire.recvZero;
                j.endedBy = "recv returned 0 - the board closed its end (FIN)";
                return false;                     // orderly close from the board
            }
            if(n < 0)
            {
                const Int32 err = ::WSAGetLastError();
                if(err == WSAEWOULDBLOCK)
                {
                    ++j.wire.recvWouldBlock;
                    return true;
                }
                ++j.wire.recvErrors;
                j.endedBy = "recv failed " + wsaText(err);
                return false;
            }
            // The gap between receives that returned bytes: it tells board data
            // that stopped arriving from data that arrived and was not acted on.
            const Int64 nowMs = monoMs();
            if(j.lastRecvMs > 0 && nowMs - j.lastRecvMs > j.wire.worstRecvGapMs)
            {
                j.wire.worstRecvGapMs = nowMs - j.lastRecvMs;
            }
            j.lastRecvMs = nowMs;
            j.wire.rxBytes += static_cast<UInt64>(n);
            j.rxTotal += static_cast<UInt64>(n);
            c.rxUsed += static_cast<Size>(n);
            const Size used = ingestBytes(s, c.rx.data(), c.rxUsed, nowMs, &j.heard);
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
                // WOULDBLOCK is a drained socket; anything else is counted.
                // Windows reports an ICMP port-unreachable for an earlier
                // datagram as 10054 on the next receive: the board's UDP port
                // is not open.
                const Int32 err = n < 0 ? ::WSAGetLastError() : 0;
                if(n < 0 && err != WSAEWOULDBLOCK)
                {
                    ++c.journal.wire.udpRxErrors;
                    c.journal.udpRxError = err;
                }
                return;
            }
            ++c.journal.wire.udpRx;
            // One datagram is one frame; take() checks its CRC.
            bibowire::Frame f;
            Size used = 0;
            const Size len = static_cast<Size>(n);
            if(bibowire::take(dgram.data(), len, &f, &used) == bibowire::Take::TAKE_FRAME)
            {
                ingestFrame(s, f, monoMs(), &c.journal.heard);
            }
            else
            {
                ++c.journal.wire.udpRxBad;
            }
        }
    }

    // Every PONG, with how long after its PING it left and what the socket did
    // with it: the line the board's "no PONG" is held against.
    Void logPong(const Journal& j, UInt64 token, Bool encoded)
    {
        if(!encoded)
        {
            vlog::line("PONG token=%llu NOT SENT: the codec refused to encode it", token);
            return;
        }
        Str after = "its PING's arrival was not recorded";
        for(const HeardPing& p : j.heard.pings)
        {
            if(p.token == token)
            {
                after = numberText(monoMs() - p.atMs) + " ms after its PING was decoded";
                break;
            }
        }
        vlog::line("PONG token=%llu %s: %s", token, after.c_str(), sendText(j.lastSend).c_str());
    }

    [[nodiscard]] Bool flushPongs(Conn& c, Session& s)
    {
        if(s.pongsDue.size() > c.journal.wire.worstPongsDue)
        {
            c.journal.wire.worstPongsDue = s.pongsDue.size();
        }
        Bool ok = true;
        for(const bibowire::Ping& ping : s.pongsDue)
        {
            Array<UInt8, 64> body = {};
            bibowire::Ping pong = ping;
            // The token is echoed verbatim; senderMonoUs is this viewer's clock,
            // which the board only compares with itself.
            pong.senderMonoUs = static_cast<UInt64>(monoMs()) * 1000u;
            const Size n = bibowire::writePing(pong, body.data(), body.size());
            const Bool sent = n != 0 && sendFrame(c, bibowire::Type::TYPE_PONG, body.data(), n);
            logPong(c.journal, ping.token, n != 0);
            if(!sent)
            {
                ok = false;
                break;
            }
        }
        s.pongsDue.clear();
        return ok;
    }

    [[nodiscard]] Bool sendPing(Conn& c, Session& s, Int64 nowMs)
    {
        ++c.pingToken;
        bibowire::Ping ping;
        // A counter, not the clock, so two PINGs in one millisecond stay
        // distinct.
        ping.token = c.pingToken;
        ping.senderMonoUs = static_cast<UInt64>(nowMs) * 1000u;
        Array<UInt8, 64> body = {};
        const Size n = bibowire::writePing(ping, body.data(), body.size());
        if(n == 0)
        {
            return false;
        }
        notePingSent(s, ping.token, nowMs);
        const Bool sent = sendFrame(c, bibowire::Type::TYPE_PING, body.data(), n);
        vlog::line("PING token=%llu: %s", ping.token, sendText(c.journal.lastSend).c_str());
        return sent;
    }

    [[nodiscard]] Bool sendHello(Conn& c, const Str& name, Bool wantSlot)
    {
        bibowire::Hello hello;
        hello.protoMajor = bibowire::PROTO_MAJOR;
        hello.protoMinor = bibowire::PROTO_MINOR;
        // Every bit set, under bibowire::typeBit's convention.
        hello.featureMask = 0xFFFFFFFFu;
        hello.viewerBuild = 0;
        hello.viewerUdpPort = c.udpPort;
        // The only place the control slot is asked for.
        hello.wantControl = wantSlot ? 1u : 0u;
        // Informational: the rate this viewer intends before WELCOME gives the
        // board's period.
        hello.controlHz = wantSlot
            ? static_cast<UInt16>(1000 / bibowire::CONTROL_PERIOD_MS)
            : 0u;
        hello.name = name;
        Array<UInt8, 128> body = {};
        const Size n = bibowire::writeHello(hello, body.data(), body.size());
        if(n == 0)
        {
            vlog::line("HELLO NOT SENT: the codec refused to encode it");
            return false;
        }
        const Bool sent = sendFrame(c, bibowire::Type::TYPE_HELLO, body.data(), n);
        c.journal.helloAtMs = monoMs();
        vlog::line(
            "HELLO name=\"%s\" wantControl=%u udpPort=%u controlHz=%u proto %u.%u: %s",
            hello.name.c_str(),
            static_cast<UInt32>(hello.wantControl),
            static_cast<UInt32>(hello.viewerUdpPort),
            static_cast<UInt32>(hello.controlHz),
            static_cast<UInt32>(hello.protoMajor),
            static_cast<UInt32>(hello.protoMinor),
            sendText(c.journal.lastSend).c_str()
        );
        return sent;
    }

    // SUBSCRIBE, only when it would say something new. The board sends only the
    // types the mask claims, so this frame decides between kilobytes a
    // revolution and a megabyte a second, and whether the board opens
    // /dev/video0.
    [[nodiscard]] Bool syncSubscription(Conn& c, Session& s, Bool wantCam, Int32 wantFps)
    {
        if(!s.haveWelcome)
        {
            return true;
        }
        const UInt32 want = subscriptionMask(wantCam);
        // With the camera off the rate is sent as 0 (did not ask), not the last
        // slider position.
        UInt16 fps = 0;
        if(wantCam && wantFps > 0)
        {
            const Int32 ceiling = static_cast<Int32>(bibowire::CAM_FPS_MAX);
            fps = static_cast<UInt16>(wantFps > ceiling ? ceiling : wantFps);
        }
        // Never asked and nothing wanted: send nothing. The board's default
        // already sends what this viewer draws.
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
        // 1 = every revolution. The camera never thins the scan to make room:
        // which matters more is the operator's decision.
        sub.scanDivisor = 1;
        Array<UInt8, 32> body = {};
        const Size n = bibowire::writeSubscribe(sub, body.data(), body.size());
        if(n == 0)
        {
            return false;
        }
        const Bool sent = sendFrame(c, bibowire::Type::TYPE_SUBSCRIBE, body.data(), n);
        vlog::line(
            "SUBSCRIBE mask=0x%08X camFps=%u scanDivisor=%u sessionId=%u: %s",
            sub.typeMask,
            static_cast<UInt32>(sub.camFps),
            static_cast<UInt32>(sub.scanDivisor),
            sub.sessionId,
            sendText(c.journal.lastSend).c_str()
        );
        if(!sent)
        {
            return false;
        }
        c.sentMask = want;
        c.sentFps = fps;
        s.cameraSubscribed = wantCam;
        return true;
    }

    // The epoch this viewer believes is in force, from the freshest source:
    // CTLSTATE (20 Hz), then BOARD (5 Hz), then WELCOME.
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

    // Everything the UI thread queued, onto the wire or dropped. The queue is
    // emptied either way: that is sendCommand's drop rule. Swapped out under the
    // lock and sent outside it, so a slow socket cannot block the UI thread.
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
        // Before WELCOME there is no sessionId to stamp, so these are
        // unsendable.
        if(!s.haveWelcome)
        {
            for(const bibowire::Command& cmd : outbound)
            {
                vlog::line(
                    "COMMAND cmdId=%u verb=%u (%s) DROPPED: no WELCOME on this connection yet",
                    cmd.cmdId,
                    static_cast<UInt32>(cmd.verb),
                    verbText(cmd.verb)
                );
            }
            owner.commandsDropped.fetch_add(static_cast<UInt32>(outbound.size()));
            return true;
        }
        // Indexed, so a failure counts only the commands left, never ones
        // already sent.
        for(Size i = 0; i < outbound.size(); ++i)
        {
            bibowire::Command cmd = outbound[i];
            cmd.sessionId = s.welcome.sessionId;
            cmd.armEpoch = currentEpoch(s);
            Array<UInt8, 64> body = {};
            const Size n = bibowire::writeCommand(cmd, body.data(), body.size());
            if(n == 0)
            {
                // The codec refused the arguments: a caller bug, not a link
                // fault, so it is dropped and the connection kept.
                vlog::line(
                    "COMMAND cmdId=%u verb=%u (%s) DROPPED: the codec refused its arguments",
                    cmd.cmdId,
                    static_cast<UInt32>(cmd.verb),
                    verbText(cmd.verb)
                );
                owner.commandsDropped.fetch_add(1u);
                continue;
            }
            const Bool sent = sendFrame(c, bibowire::Type::TYPE_COMMAND, body.data(), n);
            vlog::line(
                "COMMAND cmdId=%u verb=%u (%s) args %u/%u/%u armEpoch=%u sessionId=%u: %s",
                cmd.cmdId,
                static_cast<UInt32>(cmd.verb),
                verbText(cmd.verb),
                static_cast<UInt32>(cmd.arg0),
                static_cast<UInt32>(cmd.arg1),
                static_cast<UInt32>(cmd.arg2),
                static_cast<UInt32>(cmd.armEpoch),
                cmd.sessionId,
                sendText(c.journal.lastSend).c_str()
            );
            if(!sent)
            {
                // This one and everything behind it are dropped with the
                // connection, never retried on a socket that just failed.
                const Size left = outbound.size() - i;
                if(left > 1u)
                {
                    vlog::line(
                        "%zu COMMAND(s) queued behind it DROPPED with the connection",
                        left - 1u
                    );
                }
                owner.commandsDropped.fetch_add(static_cast<UInt32>(left));
                return false;
            }
            CmdOut out;
            out.cmdId = cmd.cmdId;
            out.sentMs = monoMs();
            c.journal.commandsOut.push_back(out);
            while(c.journal.commandsOut.size() > MAX_PENDING_COMMANDS)
            {
                c.journal.commandsOut.erase(c.journal.commandsOut.begin());
            }
        }
        return true;
    }

    // What the UI thread is told, published in one go so the panel's sentence
    // and the control tally beside it always come from the same pass.
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

    // One CONTROL datagram: `send` when the socket is connected to the board,
    // `sendto` when it could not be. A failure leaves its WSA code in the
    // journal.
    [[nodiscard]] Bool sendControlUdp(Conn& c, const UInt8* frame, Size len)
    {
        if(c.udp == INVALID_SOCKET)
        {
            c.journal.udpError = WSAENOTSOCK;
            return false;
        }
        const Char* at = reinterpret_cast<const Char*>(frame);
        const Int32 want = static_cast<Int32>(len);
        Int32 n = 0;
        if(c.udpFiltered)
        {
            n = ::send(c.udp, at, want, 0);
        }
        else if(c.haveBoardAddr)
        {
            const sockaddr* to = reinterpret_cast<const sockaddr*>(&c.boardAddr);
            n = ::sendto(c.udp, at, want, 0, to, c.boardAddrLen);
        }
        else
        {
            // No known board address, which Winsock names WSAEDESTADDRREQ.
            c.journal.udpError = WSAEDESTADDRREQ;
            return false;
        }
        if(n != want)
        {
            c.journal.udpError = n < 0 ? ::WSAGetLastError() : 0;
            return false;
        }
        return true;
    }

    // Logs CONTROL on change, never per datagram: the stream repeats every
    // period, and the log needs the moments the operator, the epoch or the
    // transport changed it.
    Void noteControl(Journal& j, const bibowire::Control& m, Bool onTcp)
    {
        const bibowire::Control& was = j.controlLine;
        const Bool same = j.haveControlLine
                          && was.buttons == m.buttons
                          && was.steerMilli == m.steerMilli
                          && was.throttleMilli == m.throttleMilli
                          && was.assumedMode == m.assumedMode
                          && was.armEpoch == m.armEpoch
                          && j.controlLineOnTcp == onTcp;
        if(same)
        {
            return;
        }
        j.haveControlLine = true;
        j.controlLine = m;
        j.controlLineOnTcp = onTcp;
        vlog::line(
            "CONTROL now buttons=0x%04X%s%s steer=%d throttle=%d assumedMode=%u armEpoch=%u via %s, seq %u",
            static_cast<UInt32>(m.buttons),
            (m.buttons & bibowire::BUTTON_ENABLE) != 0u ? " ENABLE" : "",
            (m.buttons & bibowire::BUTTON_ESTOP) != 0u ? " ESTOP" : "",
            static_cast<Int32>(m.steerMilli),
            static_cast<Int32>(m.throttleMilli),
            static_cast<UInt32>(m.assumedMode),
            static_cast<UInt32>(m.armEpoch),
            onTcp ? "TCP" : "UDP",
            m.seq
        );
    }

    // Called every period, changed or not, while this viewer holds the slot.
    // There is no separate heartbeat: one could keep beating while the control
    // path is dead.
    [[nodiscard]] Bool sendControl(Conn& c, Client& owner, const Session& s, Report& say)
    {
        // An observer sends nothing, and that is not a failure.
        if(!s.haveWelcome || !holdsSlot(s))
        {
            return true;
        }
        ControlStamp at;
        at.sessionId = s.welcome.sessionId;
        at.seq = nextControlSeq(c.ctlSeq);
        at.armEpoch = currentEpoch(s);
        // The viewer's clock, which the board only compares with itself.
        at.tMonoUs = static_cast<UInt64>(monoMs()) * 1000u;
        const bibowire::Control m = buildControl(controlIntent(owner), at);
        Array<UInt8, 64> body = {};
        const Size n = bibowire::writeControl(m, body.data(), body.size());
        if(n == 0)
        {
            // The codec refused it (steer or throttle outside +-1000): a bug at
            // this end, not a link fault, so the connection stays and the
            // refusal is counted rather than silently ending the stream.
            ++say.controlFailed;
            return true;
        }
        c.ctlSeq = at.seq;
        say.controlSeq = at.seq;
        noteControl(c.journal, m, say.controlOnTcp);
        if(say.controlOnTcp)
        {
            // The same frame on TCP, where the board applies identical rules,
            // deadman, session and seq checks. A failed TCP send is the
            // connection failing, unlike a datagram.
            if(!sendFrame(c, bibowire::Type::TYPE_CONTROL, body.data(), n))
            {
                const Str sent = sendText(c.journal.lastSend);
                vlog::line("CONTROL seq=%u on TCP FAILED: %s", at.seq, sent.c_str());
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
        // A datagram that did not leave is not a dead connection, and must not
        // end the session: section 7 infers nothing from send(), and Windows
        // reports a closed board port's ICMP on the next send. It is counted;
        // the TCP fallback and the deadman are what notice.
        if(!sendControlUdp(c, frame.data(), total))
        {
            ++say.controlFailed;
            ++c.journal.wire.udpTxFailed;
            // The first failure in words, the rest counted, so a line per
            // datagram cannot bury the log.
            if(!c.journal.saidUdpFailure)
            {
                c.journal.saidUdpFailure = true;
                vlog::line(
                    "CONTROL datagram seq=%u did not leave: %s - further failures are only counted",
                    at.seq,
                    wsaText(c.journal.udpError).c_str()
                );
            }
            return true;
        }
        ++c.journal.wire.udpTx;
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
        // A deliberate close says so, so the board's journal can tell "the
        // operator left" from "the link died". Best effort: the socket closes
        // either way and the board's timers cover a lost LEAVE.
        const Bool sent = sendFrame(c, bibowire::Type::TYPE_LEAVE, body.data(), n);
        vlog::line(
            "LEAVE sessionId=%u %s: %s",
            leave.sessionId,
            sent ? "sent" : "NOT sent",
            sendText(c.journal.lastSend).c_str()
        );
    }

    [[nodiscard]] Str scanPhrase(const Session& s, Int64 nowMs)
    {
        if(!s.haveScan)
        {
            return "no scan yet";
        }
        const Int64 age = ageOf(s, nowMs, s.scanAtMs, s.scanBoardUs);
        if(age > GONE_MS)
        {
            // A live link with a silent sensor, said as such.
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

    // Sleeps in POLL_MS slices so a Disconnect is answered promptly, and
    // republishes the countdown as it goes.
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
            // The control tally is published as zero: there is no stream without
            // a connection.
            publish(c, say, s);
            ::Sleep(static_cast<DWORD>(left < POLL_MS ? left : POLL_MS));
        }
    }

    // What the decode told the log this pass, into the file. Board PINGs stay in
    // the Heard until flushPongs has answered them, because each PONG's line
    // needs its PING's arrival time.
    Void drainHeard(Conn& c, const Session& s)
    {
        Journal& j = c.journal;
        Heard& h = j.heard;
        for(const HeardPing& p : h.pings)
        {
            vlog::line(
                "board PING token=%llu seq=%u - the link was quiet %lld ms before it",
                p.token,
                static_cast<UInt32>(p.seq),
                p.quietMs
            );
        }
        for(const HeardPong& p : h.pongs)
        {
            if(p.matched)
            {
                vlog::line("PONG received token=%llu rtt %lld ms", p.token, p.rttMs);
            }
            else
            {
                vlog::line(
                    "PONG received token=%llu matches no outstanding PING - ignored",
                    p.token
                );
            }
        }
        for(const Ack& a : h.acks)
        {
            Str after = "its send was not recorded";
            for(Size i = 0; i < j.commandsOut.size(); ++i)
            {
                if(j.commandsOut[i].cmdId == a.ack.cmdId)
                {
                    after = numberText(a.atMs - j.commandsOut[i].sentMs) + " ms after it was sent";
                    j.commandsOut.erase(j.commandsOut.begin() + static_cast<ISize>(i));
                    break;
                }
            }
            vlog::line(
                "CMDACK cmdId=%u verb=%u (%s) result=%u (%s) armEpoch=%u, %s: \"%s\"",
                a.ack.cmdId,
                static_cast<UInt32>(a.ack.verb),
                verbText(a.ack.verb),
                static_cast<UInt32>(a.ack.result),
                ackResultName(a.ack.result),
                static_cast<UInt32>(a.ack.armEpoch),
                after.c_str(),
                a.ack.text.c_str()
            );
        }
        for(const Note& e : h.events)
        {
            vlog::line("EVENT %s: %s", severityText(e.severity), e.text.c_str());
        }
        h.pongs.clear();
        h.acks.clear();
        h.events.clear();
        // The first of each kind in words, the rest only in the summary's
        // totals, or a bad stream would log a line per byte.
        if(h.bodiesRefused > 0u && !j.saidRefused)
        {
            j.saidRefused = true;
            vlog::line(
                "reader REFUSED a %s frame (tag 0x%02X): its body would not decode",
                typeText(h.lastRefusedType),
                static_cast<UInt32>(h.lastRefusedType)
            );
        }
        if(h.unknownTypes > 0u && !j.saidUnknown)
        {
            j.saidUnknown = true;
            vlog::line(
                "reader skipped a frame of UNKNOWN tag 0x%02X",
                static_cast<UInt32>(h.lastUnknownType)
            );
        }
        if(h.resyncs > 0u && !j.saidResync)
        {
            j.saidResync = true;
            vlog::line(
                "reader RESYNCED past %u byte(s) of junk - a corrupted frame lands here",
                h.resyncBytes
            );
        }
        if(h.framingRefused > 0u && !j.saidFraming)
        {
            j.saidFraming = true;
            vlog::line("reader refused a frame header on TCP (TOO_BIG or BAD_FLAG)");
        }
        if(h.seqJumps > 0u && !j.saidSeqJump)
        {
            j.saidSeqJump = true;
            vlog::line(
                "board TCP seq jumped %u -> %u",
                static_cast<UInt32>(h.jumpFrom),
                static_cast<UInt32>(h.jumpTo)
            );
        }
        if(h.overflow > 0u && !j.saidOverflow)
        {
            j.saidOverflow = true;
            vlog::line(
                "more than %zu frames of one kind in one pass - the rest were not logged",
                HEARD_MAX
            );
        }
        if(s.haveBye && !j.saidBye)
        {
            j.saidBye = true;
            vlog::line(
                "BYE reason=%u (%s): \"%s\"",
                static_cast<UInt32>(s.byeReason),
                reasonText(s.byeReason),
                s.byeText.c_str()
            );
        }
    }

    // The kernel's account of the TCP connection: unacknowledged bytes,
    // retransmissions, timeouts and both windows. If the board stops hearing
    // this viewer while these show every byte sent and acknowledged, the fault
    // is not in this process.
    Void appendTcpInfo(Str& out, SOCKET fd)
    {
        u_long queued = 0;
        if(::ioctlsocket(fd, FIONREAD, &queued) == 0)
        {
            vlog::append(out, " | kernel rx queue %u B", static_cast<UInt32>(queued));
        }
#ifdef SIO_TCP_INFO
        DWORD version = 0;
        TCP_INFO_v0 info = {};
        DWORD got = 0;
        const Int32 rc = ::WSAIoctl(
            fd,
            SIO_TCP_INFO,
            &version,
            static_cast<DWORD>(sizeof(version)),
            &info,
            static_cast<DWORD>(sizeof(info)),
            &got,
            nullptr,
            nullptr
        );
        if(rc != 0)
        {
            out += " | tcp_info unavailable: " + wsaText(::WSAGetLastError());
            return;
        }
        vlog::append(
            out,
            " | tcp rtt %u us (min %u), in flight %u B, retrans %u B (%u fast), timeouts %u",
            static_cast<UInt32>(info.RttUs),
            static_cast<UInt32>(info.MinRttUs),
            static_cast<UInt32>(info.BytesInFlight),
            static_cast<UInt32>(info.BytesRetrans),
            static_cast<UInt32>(info.FastRetrans),
            static_cast<UInt32>(info.TimeoutEpisodes)
        );
        vlog::append(
            out,
            ", dup acks %u, cwnd %u, sndwnd %u, rcvwnd %u, rcvbuf %u, kernel in %llu B out %llu B",
            static_cast<UInt32>(info.DupAcksIn),
            static_cast<UInt32>(info.Cwnd),
            static_cast<UInt32>(info.SndWnd),
            static_cast<UInt32>(info.RcvWnd),
            static_cast<UInt32>(info.RcvBuf),
            static_cast<UInt64>(info.BytesIn),
            static_cast<UInt64>(info.BytesOut)
        );
#else
        // Said, so absent numbers are not mistaken for zeros.
        out += " | tcp_info: SIO_TCP_INFO is not in this Windows SDK";
#endif
    }

    // Once per SUMMARY_MS, the socket half in one line, so a search for the
    // moment the board said "no PONG" lands on everything this end knew then.
    Void logSummary(Conn& c, const Session& s, const Report& say, Int64 nowMs)
    {
        Journal& j = c.journal;
        Wire& w = j.wire;
        const Heard& h = j.heard;
        Str out;
        out.reserve(1024);
        vlog::append(
            out,
            "summary %lld ms: tcp rx %llu B, tx %llu B",
            nowMs - w.windowMs,
            w.rxBytes,
            w.txBytes
        );
        out += " | frames in";
        Bool any = false;
        for(Size i = 0; i < h.byType.size(); ++i)
        {
            const UInt32 got = h.byType[i] - j.byTypeAtSummary[i];
            if(got == 0u)
            {
                continue;
            }
            any = true;
            const UInt8 tag = static_cast<UInt8>(i);
            if(bibowire::knownType(tag))
            {
                vlog::append(out, " %s=%u", typeText(tag), got);
            }
            else
            {
                vlog::append(out, " 0x%02X=%u", static_cast<UInt32>(tag), got);
            }
        }
        if(!any)
        {
            out += " NONE";
        }
        j.byTypeAtSummary = h.byType;
        const Str lastByte = j.lastRecvMs > 0
            ? numberText(nowMs - j.lastRecvMs) + " ms ago"
            : Str("never");
        vlog::append(
            out,
            " | recv %u calls (%u would-block, %u zero, %u error), worst gap %lld ms, last byte %s",
            w.recvCalls,
            w.recvWouldBlock,
            w.recvZero,
            w.recvErrors,
            w.worstRecvGapMs,
            lastByte.c_str()
        );
        vlog::append(
            out,
            " | loop %u passes, worst %lld ms (select %lld recv %lld log %lld send %lld publish %lld)",
            w.passes,
            w.worstPassMs,
            w.worstSplit.selectMs,
            w.worstSplit.recvMs,
            w.worstSplit.logMs,
            w.worstSplit.sendMs,
            w.worstSplit.publishMs
        );
        // No outbound queue: sendAll blocks up to SEND_BUDGET_MS, so a backlog
        // shows as a slow send, and unacknowledged bytes are in tcp_info.
        vlog::append(
            out,
            " | send %u (short %u, would-block %u, failed %u), worst %lld ms, no outbound queue",
            w.sends,
            w.sendShort,
            w.sendWouldBlock,
            w.sendFailed,
            w.worstSendMs
        );
        vlog::append(
            out,
            " | udp out %u (failed %u), in %u (bad %u, errors %u, last %d)",
            w.udpTx,
            w.udpTxFailed,
            w.udpRx,
            w.udpRxBad,
            w.udpRxErrors,
            j.udpRxError
        );
        vlog::append(
            out,
            " | control %s seq %u, %u sent %u failed this connection%s",
            say.controlOnTcp ? "TCP" : "UDP",
            c.ctlSeq,
            say.controlSent,
            say.controlFailed,
            holdsSlot(s) ? "" : " (slot not held - none sent)"
        );
        if(s.haveControl)
        {
            const UInt8 dm = s.control.deadman;
            CharSeq dmName = "?";
            if(dm <= static_cast<UInt8>(bibowire::deadman::State::STATE_ESTOP))
            {
                dmName = bibowire::deadman::stateName(static_cast<bibowire::deadman::State>(dm));
            }
            vlog::append(
                out,
                " | ctlstate deadman %u (%s), refuse %s, ackSeq %u, armEpoch %u",
                static_cast<UInt32>(dm),
                dmName,
                bibowire::refuseName(s.control.refuse),
                s.control.ackSeq,
                static_cast<UInt32>(s.control.armEpoch)
            );
            vlog::append(
                out,
                ", armed %u, holder %u, age %lld ms",
                static_cast<UInt32>(s.control.armed),
                static_cast<UInt32>(s.control.holder),
                nowMs - s.controlAtMs
            );
        }
        else
        {
            out += " | ctlstate NEVER";
        }
        vlog::append(
            out,
            " | pongsDue worst %zu, own PINGs unanswered %zu",
            w.worstPongsDue,
            s.pingsOut.size()
        );
        vlog::append(
            out,
            " | reader totals: refused %u, unknown %u, resync %u B in %u, framing %u, seq jumps %u",
            h.bodiesRefused,
            h.unknownTypes,
            h.resyncBytes,
            h.resyncs,
            h.framingRefused,
            h.seqJumps
        );
        appendTcpInfo(out, c.tcp);
        vlog::line("%s", out.c_str());
        w = Wire();
        w.windowMs = nowMs;
    }

    Void notePass(Wire& w, const PassSplit& split)
    {
        ++w.passes;
        const Int64 total = split.selectMs + split.recvMs + split.logMs + split.sendMs + split.publishMs;
        if(total > w.worstPassMs)
        {
            w.worstPassMs = total;
            w.worstSplit = split;
        }
    }

    Void logWelcome(const Conn& c, const Session& s)
    {
        const bibowire::Welcome& w = s.welcome;
        CharSeq slot = "REFUSED";
        if(w.accepted == 1u)
        {
            slot = "control is yours";
        }
        else if(w.accepted == 2u)
        {
            slot = "observer";
        }
        const Int64 afterMs = c.journal.helloAtMs > 0 ? monoMs() - c.journal.helloAtMs : -1;
        vlog::line(
            "WELCOME %lld ms after HELLO: accepted=%u (%s) refusal=%u sessionId=%u armEpoch=%u",
            afterMs,
            static_cast<UInt32>(w.accepted),
            slot,
            static_cast<UInt32>(w.refusal),
            w.sessionId,
            static_cast<UInt32>(w.armEpoch)
        );
        vlog::line(
            "WELCOME bootId=0x%08X caps=0x%02X proto %u.%u board=\"%s\" text=\"%s\"",
            w.bootId,
            static_cast<UInt32>(w.capabilities),
            static_cast<UInt32>(w.protoMajor),
            static_cast<UInt32>(w.protoMinor),
            w.boardName.c_str(),
            w.text.c_str()
        );
        vlog::line(
            "WELCOME controlUdpPort=%u controlPeriod %u ms, stale %u ms, dead %u ms",
            static_cast<UInt32>(w.controlUdpPort),
            static_cast<UInt32>(w.controlPeriodMs),
            static_cast<UInt32>(w.staleMs),
            static_cast<UInt32>(w.deadMs)
        );
    }

    Void logClose(const Conn& c, const Session& s, const Str& why, Int64 nowMs, Bool quitting)
    {
        const Journal& j = c.journal;
        const Str reason = quitting ? Str("this viewer disconnected") : why;
        const Int64 livedMs = j.openedMs > 0 ? nowMs - j.openedMs : 0;
        vlog::line(
            "connection CLOSED: %s%s%s - it lived %lld ms, rx %llu B, tx %llu B, %u frames",
            reason.c_str(),
            j.endedBy.empty() ? "" : " - ",
            j.endedBy.c_str(),
            livedMs,
            j.rxTotal,
            j.txTotal,
            s.frames
        );
        if(!j.lastSend.ok && j.lastSend.offered > 0u)
        {
            vlog::line("the last send before the close: %s", sendText(j.lastSend).c_str());
        }
    }

    // The next reconnect wait, logged with its attempt and base.
    [[nodiscard]] Int32 retryDelay(Int32 attempt, UInt32 seed, const Str& why)
    {
        const Int32 base = backoffBaseMs(attempt);
        const Int32 wait = jittered(base, seed);
        vlog::line(
            "reconnect attempt %d in %d ms (base %d) - %s",
            attempt,
            wait,
            base,
            why.c_str()
        );
        return wait;
    }

    Void runWorker(Client* c)
    {
        vlog::nameThread("net");
        vlog::line("worker started for %s port %u", c->host.c_str(), static_cast<UInt32>(c->port));
        Conn conn;
        Session live;
        Int32 attempt = 0;
        // Seeded from the clock, so two viewers started together do not share a
        // jitter sequence.
        UInt32 seed = static_cast<UInt32>(monoMs()) ^ 0xB1B0B0C5u;
        while(!c->quit.load())
        {
            // A new connection resumes nothing: the live picture is worthless by
            // the time the link is back.
            clearSession(live);
            // The Report, control tally included, starts at zero per connection.
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
                waitToRetry(*c, live, retryDelay(attempt, seed, why), why);
                continue;
            }
            if(!openUdp(conn))
            {
                // Only CTLSTATE rides UDP toward this viewer, so telemetry is
                // unaffected; said in a note rather than a row always "--".
                Note note;
                note.severity = bibowire::Severity::SEVERITY_WARN;
                note.text = "no UDP socket - CTLSTATE cannot arrive";
                note.atMs = monoMs();
                live.notes.push_back(note);
                // With no socket there is no reverse path to probe, so CONTROL
                // goes straight to TCP.
                say.controlOnTcp = true;
                vlog::line("CONTROL goes to TCP from the first datagram: there is no UDP socket");
            }
            say.phase = Phase::PHASE_CONNECTING;
            say.status = "connected to " + c->host;
            publish(*c, say, live);
            // HELLO is the first bytes on the connection, and nothing else is
            // sent until WELCOME answers it.
            if(!sendHello(conn, "bibo viewer", c->wantSlot.load()))
            {
                dropConn(conn);
                ++attempt;
                seed = stir(seed);
                waitToRetry(
                    *c,
                    live,
                    retryDelay(attempt, seed, "could not send HELLO"),
                    "could not send HELLO"
                );
                continue;
            }
            live.lastFrameMs = monoMs();
            Int64 nextPingMs = live.lastFrameMs + PING_PERIOD_MS;
            // The control deadline, a timestamp like nextPingMs rather than a
            // thread: a second thread could look alive while this loop is wedged,
            // and the stream's silence must mean something. Zero until WELCOME
            // arms it; before then there is no session to stamp and no slot.
            Int64 nextControlMs = 0;
            Int64 welcomeAtMs = 0;
            why = "the board closed the connection";
            conn.journal.wire.windowMs = monoMs();
            while(!c->quit.load())
            {
                // Where each pass spends its time: a slow `send` is a full
                // socket, a slow `publish` is a wait on the UI's lock.
                const Int64 passMs = monoMs();
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
                    conn.journal.endedBy = "select failed " + wsaText(::WSAGetLastError());
                    why = "the connection failed";
                    break;
                }
                const Int64 selectedMs = monoMs();
                const Bool hadWelcome = live.haveWelcome;
                // Pumped, logged, then judged: the frames that arrived just
                // before a close are the ones most worth logging.
                const Bool tcpReadable = ready > 0 && FD_ISSET(conn.tcp, &reads);
                const Bool tcpAlive = !tcpReadable || pumpTcp(conn, live);
                const Bool udpReadable = ready > 0 && conn.udp != INVALID_SOCKET
                                         && FD_ISSET(conn.udp, &reads);
                if(tcpAlive && udpReadable)
                {
                    pumpUdp(conn, live);
                }
                const Int64 receivedMs = monoMs();
                drainHeard(conn, live);
                const Int64 loggedMs = monoMs();
                if(!tcpAlive)
                {
                    break;
                }
                const Bool answered = flushPongs(conn, live);
                // Answered, so the PINGs' arrival times have done their job.
                conn.journal.heard.pings.clear();
                if(!answered)
                {
                    why = "could not answer a PING";
                    break;
                }
                if(!hadWelcome && live.haveWelcome)
                {
                    // A completed handshake restarts the backoff schedule.
                    attempt = 0;
                    filterUdpToBoard(conn, live.welcome.controlUdpPort);
                    // WELCOME starts section 4's reverse-path window and the
                    // control cadence. The first datagram goes on this pass: the
                    // board's probe wants several inside REVERSE_PROBE_MS.
                    welcomeAtMs = monoMs();
                    nextControlMs = welcomeAtMs;
                    logWelcome(conn, live);
                    if(!holdsSlot(live))
                    {
                        vlog::line("no control slot on this connection - no CONTROL is sent");
                    }
                }
                // Every pass: the camera window can change at any moment, and a
                // new connection's sentMask is reset, so an open window is
                // asked for again.
                if(!syncSubscription(conn, live, c->cameraOn.load(), c->cameraFps.load()))
                {
                    why = "could not send SUBSCRIBE";
                    break;
                }
                // Same pass as SUBSCRIBE, so a command is on the wire within one
                // POLL_MS.
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
                    why = "no frame for " + numberText(now - live.lastFrameMs) + " ms";
                    break;
                }
                if(live.haveWelcome && now >= nextPingMs)
                {
                    nextPingMs = now + PING_PERIOD_MS;
                    if(!sendPing(conn, live, now))
                    {
                        why = "could not send a PING";
                        break;
                    }
                }
                // Section 4's TCP fallback, measured then latched. Only CTLSTATE
                // rides UDP toward this viewer, so none within REVERSE_PROBE_MS
                // of WELCOME shows UDP failing in at least one direction. It
                // latches for the connection: the board then mirrors CTLSTATE
                // onto TCP, so re-testing would flap between transports while
                // driving.
                const Int64 probeMs = static_cast<Int64>(bibowire::REVERSE_PROBE_MS);
                if(!say.controlOnTcp && welcomeAtMs > 0 && !live.haveControl
                   && now - welcomeAtMs > probeMs)
                {
                    say.controlOnTcp = true;
                    vlog::line(
                        "CONTROL FALLS BACK TO TCP: no CTLSTATE %lld ms after WELCOME (window %lld ms)",
                        now - welcomeAtMs,
                        probeMs
                    );
                    vlog::line(
                        "  UDP until then: CONTROL %u sent, %u failed",
                        say.controlSent,
                        say.controlFailed
                    );
                    Note note;
                    note.severity = bibowire::Severity::SEVERITY_WARN;
                    note.text = "no CTLSTATE for " + numberText(now - welcomeAtMs)
                                + " ms - degraded control (TCP)";
                    note.atMs = now;
                    live.notes.push_back(note);
                }
                // Every period, changed or not, at the board's period. The
                // deadline rolls even for an observer, which sends nothing.
                if(live.haveWelcome && now >= nextControlMs)
                {
                    nextControlMs = now + controlPeriodMs(live);
                    if(!sendControl(conn, *c, live, say))
                    {
                        why = "could not send a CONTROL";
                        break;
                    }
                }
                const Int64 sentMs = monoMs();
                say.phase = live.haveWelcome ? Phase::PHASE_LIVE : Phase::PHASE_HANDSHAKING;
                say.status = liveStatus(live, c->host, now);
                say.retryInMs = 0;
                publish(*c, say, live);
                const Int64 publishedMs = monoMs();
                PassSplit split;
                split.selectMs = selectedMs - passMs;
                split.recvMs = receivedMs - selectedMs;
                split.logMs = loggedMs - receivedMs;
                split.sendMs = sentMs - loggedMs;
                split.publishMs = publishedMs - sentMs;
                notePass(conn.journal.wire, split);
                if(publishedMs - conn.journal.wire.windowMs >= SUMMARY_MS)
                {
                    logSummary(conn, live, say, publishedMs);
                }
            }
            // LEAVE first when this viewer is leaving, so the close line follows
            // everything sent; then the partial summary window and the close.
            const Bool quitting = c->quit.load();
            if(quitting)
            {
                sendLeave(conn, live);
            }
            const Int64 closedMs = monoMs();
            logSummary(conn, live, say, closedMs);
            logClose(conn, live, why, closedMs, quitting);
            dropConn(conn);
            if(quitting)
            {
                vlog::line("worker stopping: this viewer disconnected");
                break;
            }
            ++attempt;
            seed = stir(seed);
            waitToRetry(*c, live, retryDelay(attempt, seed, why), why);
        }
    }
  }

  Int64 monoMs()
  {
      static const TimePoint BASE = monoNow();
      return static_cast<Int64>(elapsedMs(BASE));
  }

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

  Void noteArrival(Cadence& c, Int64 nowMs)
  {
      if(!c.have)
      {
          c.have = true;
          c.lastAtMs = nowMs;
          return;
      }
      const Int64 gap = nowMs - c.lastAtMs;
      c.lastAtMs = nowMs;
      // A clock that went backwards, or two frames in one millisecond: not an
      // interval the feed delivered at.
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
      // `count`, not CADENCE_SAMPLES: a fresh ring's tail is unfilled.
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
          return FRESH_MS;
      }
      const Int64 band = (worst * STALE_SLACK_NUM) / STALE_SLACK_DEN;
      if(band < FRESH_MS)
      {
          return FRESH_MS;
      }
      if(band > STALE_CEIL_MS)
      {
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
          return {};
      }
      CameraShot shot;
      shot.frameIndex = camera.frameIndex;
      shot.width = camera.width;
      shot.height = camera.height;
      shot.codec = camera.codec;
      shot.bytes = camera.data;
      shot.ageMs = age;
      shot.staleAtMs = staleBandMs(cameraRate);
      shot.worstGapMs = worstGapMs(cameraRate);
      shot.worstCaptureMs = cameraWorstCaptureMs;
      shot.stale = age > shot.staleAtMs;
      return shot;
  }

  Opt<Odometry> Session::odometry(Int64 nowMs) const
  {
      if(!haveOdom)
      {
          return {};
      }
      const Int64 age = ageOf(*this, nowMs, odomAtMs, odom.tMonoUs);
      if(age > GONE_MS)
      {
          return {};
      }
      Odometry o;
      o.odom = odom;
      o.ageMs = age;
      o.stale = age > staleBandMs(odomRate);
      return o;
  }

  Opt<TagsSeen> Session::tagsSeen(Int64 nowMs) const
  {
      if(!haveTags)
      {
          return {};
      }
      const Int64 age = ageOf(*this, nowMs, tagsAtMs, tags.tMonoUs);
      if(age > GONE_MS)
      {
          return {};
      }
      TagsSeen seen;
      seen.tags = tags;
      seen.ageMs = age;
      seen.stale = age > staleBandMs(tagRate);
      return seen;
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
      while(s.pingsOut.size() > MAX_PINGS_OUT)
      {
          s.pingsOut.erase(s.pingsOut.begin());
      }
  }

  Void ingestFrame(Session& s, const bibowire::Frame& f, Int64 nowMs)
  {
      // Every frame, unknown types included: the silence watchdog must not
      // redial a live board that has learned a new type.
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
          // A different bootId means the pilot restarted: clear everything
          // before a point is drawn, or a healthy new socket to a restarted car
          // still shows the previous run.
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
          // The version rule applies to WELCOME too, with a sentence naming both
          // versions and what to do.
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
          // Gaps are counted, never interpolated: revIndex is monotonic, and a
          // smoothed sweep is a lie the eye cannot detect.
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
              // scene.hxx's frame with the car at the origin facing +Y: bearing 0
              // is +Y and a growing angle swings toward +X, the car's right.
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
          // A DECIDE naming a scan this viewer never received is discarded:
          // drawn over another revolution it is plausible and false. revIndex 0
          // is the BLIND tick, which has no revolution, and is kept.
          if(m.revIndex != 0u && (!s.haveScan || m.revIndex != s.revIndex))
          {
              ++s.orphanDecides;
              return;
          }
          s.haveDecide = true;
          s.decide = m;
          s.decideAtMs = nowMs;
          // DECIDE has no timestamp; it borrows its revolution's board clock.
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
          // Counted like the scan's gaps. CAMERA is CLASS_BULK, dropped before
          // any scan or state frame, so on a bad hotspot this counter moves
          // first, by design.
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
          // The capture gap on the board's clock. The first frame only starts
          // it, or the board's whole uptime would read as a stall. A timestamp
          // that went backwards is a restarted board, skipped rather than
          // recorded.
          if(s.haveCamera && m.tMonoUs > s.cameraBoardUs)
          {
              const Int64 capturedMs = static_cast<Int64>((m.tMonoUs - s.cameraBoardUs) / 1000u);
              if(capturedMs > s.cameraWorstCaptureMs)
              {
                  s.cameraWorstCaptureMs = capturedMs;
              }
          }
          s.cameraBoardUs = m.tMonoUs;
          s.haveCamera = true;
          s.cameraAtMs = nowMs;
          ++s.cameraFrames;
          noteArrival(s.cameraRate, nowMs);
          noteOffset(s, nowMs, m.tMonoUs);
          // Moved, not copied: a whole JPEG, several times a second on the
          // network thread.
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
          // The board's sentences are shown verbatim; codes alone do not make a
          // fault diagnosable.
          Note note;
          note.severity = m.severity;
          note.text = m.text;
          note.atMs = nowMs;
          // The saved trim is state, not news: kept for the Trim pane and still
          // listed as a note.
          if(m.code == bibowire::EVENT_CODE_TRIM)
          {
              s.haveBoardTrim = true;
              s.boardTrimText = m.text;
              s.boardTrimAtMs = nowMs;
              ++s.boardTrimCount;
              note.text = m.text.empty()
                  ? Str("the board has no saved trim - the Pico is on its compiled values")
                  : "trim saved on the board: " + m.text;
          }
          if(m.droppedSince > 0u)
          {
              note.text += " (+" + numberText(m.droppedSince) + " suppressed)";
          }
          if(mentionsCamera(note.text))
          {
              s.haveCameraNote = true;
              s.cameraNoteText = note.text;
              s.cameraNoteAtMs = nowMs;
          }
          if(m.code == bibowire::EVENT_CODE_BUNDLE)
          {
              s.haveBundleNote = true;
              s.bundleNote = note;
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
      case bibowire::Type::TYPE_BUNDLE:
      {
          bibowire::Bundle m;
          if(!bibowire::readBundle(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          // count 0 is the empty list, announced whole in one frame.
          if(m.count == 0u)
          {
              s.bundles.clear();
              s.bundlesArriving.clear();
              s.haveBundles = true;
              s.bundleGeneration = m.generation;
              s.bundlesAtMs = nowMs;
              return;
          }
          if(m.index >= m.count)
          {
              ++s.refusedFrames;
              return;
          }
          if(s.bundlesArriving.size() != m.count || s.bundlesArrivingGeneration != m.generation)
          {
              s.bundlesArriving.assign(m.count, bibowire::Bundle());
              s.bundlesArrivingGeneration = m.generation;
          }
          // By index, so a repeated or reordered frame cannot lengthen the list.
          s.bundlesArriving[m.index] = m;
          Bool whole = true;
          for(const bibowire::Bundle& b : s.bundlesArriving)
          {
              whole = whole && b.generation == m.generation;
          }
          if(whole)
          {
              s.bundles = s.bundlesArriving;
              s.bundlesArriving.clear();
              s.haveBundles = true;
              s.bundleGeneration = m.generation;
              s.bundlesAtMs = nowMs;
          }
          return;
      }
      case bibowire::Type::TYPE_TAGS:
      {
          bibowire::Tags m;
          if(!bibowire::readTags(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          s.haveTags = true;
          s.tagsAtMs = nowMs;
          ++s.tagFrames;
          noteArrival(s.tagRate, nowMs);
          s.tags = std::move(m);
          return;
      }
      case bibowire::Type::TYPE_ODOM:
      {
          bibowire::Odom m;
          if(!bibowire::readOdom(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          s.haveOdom = true;
          s.odomAtMs = nowMs;
          ++s.odomFrames;
          noteArrival(s.odomRate, nowMs);
          s.odom = m;
          return;
      }
      case bibowire::Type::TYPE_BUNDLE_STATE:
      {
          bibowire::BundleState m;
          if(!bibowire::readBundleState(f.body, ver, &m))
          {
              ++s.refusedFrames;
              return;
          }
          s.haveBundleState = true;
          s.bundleState = m;
          s.bundleStateAtMs = nowMs;
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
      // An unknown type is skipped by exactly its len and counted, never fatal,
      // so an older viewer can keep watching a newer board.
      ++s.unknownFrames;
  }

  namespace
  {
    template<typename T>
    Void keepHeard(Vec<T>& into, const T& item, UInt32& overflow)
    {
        if(into.size() >= HEARD_MAX)
        {
            ++overflow;
            return;
        }
        into.push_back(item);
    }
  }

  // A wrapper around the decode, not a change to it: the three-argument
  // ingestFrame is what the suite tests, and threading the log through its early
  // returns would put the log inside the code it watches. Everything here is
  // read before or after, from the frame and the session.
  Void ingestFrame(Session& s, const bibowire::Frame& f, Int64 nowMs, Heard* heard)
  {
      if(heard == nullptr)
      {
          ingestFrame(s, f, nowMs);
          return;
      }
      const bibowire::Type type = f.head.type;
      const UInt8 tag = static_cast<UInt8>(type);
      ++heard->byType[tag];
      // Read before ingesting, which overwrites lastFrameMs and, for a PONG,
      // removes the PING it matches. The token is decoded a second time.
      const Int64 quietMs = nowMs - s.lastFrameMs;
      const UInt32 refusedBefore = s.refusedFrames;
      const UInt32 unknownBefore = s.unknownFrames;
      const Bool keepalive = type == bibowire::Type::TYPE_PING || type == bibowire::Type::TYPE_PONG;
      bibowire::Ping echo;
      const Bool haveEcho = keepalive && bibowire::readPing(f.body, f.head.ver, &echo);
      HeardPong pong;
      pong.token = echo.token;
      if(haveEcho && type == bibowire::Type::TYPE_PONG)
      {
          for(const PingOut& out : s.pingsOut)
          {
              if(out.token == echo.token)
              {
                  pong.matched = true;
                  pong.rttMs = nowMs - out.sentMs;
                  break;
              }
          }
      }
      ingestFrame(s, f, nowMs);
      // Up, not changed: a restarted board's WELCOME clears the counters, and
      // that is not a refusal.
      if(s.refusedFrames > refusedBefore)
      {
          ++heard->bodiesRefused;
          heard->lastRefusedType = tag;
          return;
      }
      if(s.unknownFrames > unknownBefore)
      {
          ++heard->unknownTypes;
          heard->lastUnknownType = tag;
          return;
      }
      if(type == bibowire::Type::TYPE_PING && haveEcho)
      {
          HeardPing ping;
          ping.token = echo.token;
          ping.seq = f.head.seq;
          ping.atMs = nowMs;
          ping.quietMs = quietMs;
          keepHeard(heard->pings, ping, heard->overflow);
      }
      else if(type == bibowire::Type::TYPE_PONG && haveEcho)
      {
          keepHeard(heard->pongs, pong, heard->overflow);
      }
      else if(type == bibowire::Type::TYPE_CMDACK && !s.acks.empty())
      {
          // Not refused, so the case pushed it, and the newest is last.
          keepHeard(heard->acks, s.acks.back(), heard->overflow);
      }
      else if(type == bibowire::Type::TYPE_EVENT && !s.notes.empty())
      {
          keepHeard(heard->events, s.notes.back(), heard->overflow);
      }
  }

  Size ingestBytes(Session& s, const UInt8* buf, Size len, Int64 nowMs)
  {
      return ingestBytes(s, buf, len, nowMs, nullptr);
  }

  Size ingestBytes(Session& s, const UInt8* buf, Size len, Int64 nowMs, Heard* heard)
  {
      Size at = 0;
      while(at < len)
      {
          bibowire::Frame f;
          Size used = 0;
          const bibowire::Take got = bibowire::take(buf + at, len - at, &f, &used);
          if(got == bibowire::Take::TAKE_FRAME)
          {
              // TCP only: its seqs form a sequence, datagrams arrive outside it.
              if(heard != nullptr)
              {
                  const UInt16 expected = static_cast<UInt16>(heard->lastSeq + 1u);
                  if(heard->haveSeq && f.head.seq != expected)
                  {
                      ++heard->seqJumps;
                      heard->jumpFrom = heard->lastSeq;
                      heard->jumpTo = f.head.seq;
                  }
                  heard->haveSeq = true;
                  heard->lastSeq = f.head.seq;
              }
              ingestFrame(s, f, nowMs, heard);
              at += used;
              continue;
          }
          if(got == bibowire::Take::TAKE_RESYNC)
          {
              s.resyncBytes += static_cast<UInt32>(used);
              if(heard != nullptr)
              {
                  ++heard->resyncs;
                  heard->resyncBytes += static_cast<UInt32>(used);
              }
              at += used;
              continue;
          }
          if(got == bibowire::Take::TAKE_NEED_MORE)
          {
              break;
          }
          // TOO_BIG or BAD_FLAG. Counted and stepped over by one byte, so the
          // ring cannot wedge; a peer that keeps it up meets the silence
          // watchdog.
          ++s.refusedFrames;
          if(heard != nullptr)
          {
              ++heard->framingRefused;
          }
          at += 1;
      }
      return at;
  }

  // xorshift32 rather than <random>, so a schedule failure is reproducible in a
  // test.
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
      // Never gives up: outdoors the link comes back when the phone stops
      // moving, and recovery must not be a manual step.
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

  Bool open(Client& c, CharSeq host, UInt16 port)
  {
      if(c.running.load())
      {
          vlog::line("link open ignored: a worker is already running");
          return false;
      }
      const Str name = host == nullptr ? Str("bibobox.local") : Str(host);
      vlog::line(
          "link open: %s port %u, control slot wanted: %s",
          name.c_str(),
          static_cast<UInt32>(port),
          c.wantSlot.load() ? "yes" : "no"
      );
      WSADATA wsa;
      if(::WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
      {
          vlog::line("link open FAILED: WSAStartup refused");
          LockGuard<Mutex> held(c.lock);
          c.shared.phase = Phase::PHASE_IDLE;
          c.shared.status = "Winsock would not start";
          return false;
      }
      c.host = name;
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
      vlog::line("link close: stopping the worker");
      const Int64 askedMs = monoMs();
      c.quit.store(true);
      if(c.worker.joinable())
      {
          c.worker.join();
      }
      vlog::line("link closed: the worker joined in %lld ms", monoMs() - askedMs);
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

  UInt32 subscriptionMask(Bool withCamera)
  {
      // Every telemetry type this viewer draws, named one at a time.
      UInt32 mask = bibowire::typeBit(bibowire::Type::TYPE_SCAN)
                    | bibowire::typeBit(bibowire::Type::TYPE_DECIDE)
                    | bibowire::typeBit(bibowire::Type::TYPE_BOARD)
                    | bibowire::typeBit(bibowire::Type::TYPE_LIDAR_INFO)
                    | bibowire::typeBit(bibowire::Type::TYPE_EVENT)
                    | bibowire::typeBit(bibowire::Type::TYPE_CTLSTATE)
                    | bibowire::typeBit(bibowire::Type::TYPE_CMDACK)
                    | bibowire::typeBit(bibowire::Type::TYPE_BUNDLE)
                    | bibowire::typeBit(bibowire::Type::TYPE_BUNDLE_STATE)
                    | bibowire::typeBit(bibowire::Type::TYPE_TAGS)
                    | bibowire::typeBit(bibowire::Type::TYPE_ODOM);
      if(withCamera)
      {
          mask |= bibowire::typeBit(bibowire::Type::TYPE_CAMERA);
      }
      return mask;
  }

  Void wantCamera(Client& c, Bool on)
  {
      c.cameraOn.store(on);
  }

  Void wantCameraFps(Client& c, Int32 fps)
  {
      // Clamped here as well as on the wire, because the UI reads this value
      // back to show what was asked for.
      const Int32 ceiling = static_cast<Int32>(bibowire::CAM_FPS_MAX);
      const Int32 held = fps < 0 ? 0 : (fps > ceiling ? ceiling : fps);
      c.cameraFps.store(held);
  }

  Int32 cameraFpsWanted(const Client& c)
  {
      return c.cameraFps.load();
  }

  Void sendCommand(Client& c, bibowire::Verb verb, UInt8 arg0, UInt16 arg1, UInt16 arg2)
  {
      bibowire::Command cmd;
      Size waiting = 0;
      UInt32 evicted = 0;
      {
          LockGuard<Mutex> held(c.cmdLock);
          cmd.cmdId = c.nextCmdId;
          cmd.verb = verb;
          cmd.arg0 = arg0;
          cmd.arg1 = arg1;
          cmd.arg2 = arg2;
          ++c.nextCmdId;
          if(c.nextCmdId == 0u)
          {
              c.nextCmdId = 1u;
          }
          c.pending.push_back(cmd);
          while(c.pending.size() > MAX_PENDING_COMMANDS)
          {
              // The oldest goes, counted: a queue this deep means the worker is
              // stuck, and the newest intent is the one worth keeping.
              c.pending.erase(c.pending.begin());
              c.commandsDropped.fetch_add(1u);
              ++evicted;
          }
          waiting = c.pending.size();
      }
      // Logged after the lock is released, so a slow disk never delays the
      // worker.
      vlog::line(
          "COMMAND queued cmdId=%u verb=%u (%s) args %u/%u/%u, %zu waiting for the worker",
          cmd.cmdId,
          static_cast<UInt32>(verb),
          verbText(verb),
          static_cast<UInt32>(arg0),
          static_cast<UInt32>(arg1),
          static_cast<UInt32>(arg2),
          waiting
      );
      if(evicted > 0u)
      {
          vlog::line(
              "COMMAND queue over %zu - the oldest %u DROPPED",
              MAX_PENDING_COMMANDS,
              evicted
          );
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

  bibowire::Control buildControl(const Intent& in, const ControlStamp& at)
  {
      bibowire::Control m;
      m.sessionId = at.sessionId;
      m.seq = at.seq;
      m.tMonoUs = at.tMonoUs;
      // Neutral when the operator is not driving, enforced here, the last place
      // before the wire. The board applies steering even while throttle is
      // refused (section 6), so a steer angle sent after driving was switched
      // off would still steer the car. The casts are explicit because the
      // ternary mixes Int16 with an int literal.
      m.steerMilli = static_cast<Int16>(in.driving ? in.steerMilli : 0);
      m.throttleMilli = static_cast<Int16>(in.driving ? in.throttleMilli : 0);
      // ESTOP survives driving being off; ENABLE cannot. The stop must work in
      // every state and consent must never be asserted by accident.
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
      return next == 0u ? 1u : next;
  }

  Bool holdsSlot(const Session& s)
  {
      return s.haveWelcome && s.welcome.accepted == 1u;
  }

  Int64 controlPeriodMs(const Session& s)
  {
      const Int64 fallback = static_cast<Int64>(bibowire::CONTROL_PERIOD_MS);
      if(!s.haveWelcome || s.welcome.controlPeriodMs == 0u)
      {
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
      // Copied before close(): open() is passed the same fields it assigns.
      const Str host = c.host;
      const UInt16 port = c.port;
      vlog::line("link reconnect: %s port %u", host.c_str(), static_cast<UInt32>(port));
      close(c);
      return open(c, host.c_str(), port);
  }
}
