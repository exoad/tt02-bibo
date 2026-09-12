#include "link.hxx"

// The round-trip log. It names no Windows header, so it sits with link.hxx above
// the Winsock block rather than below it and the order there stays safe.
#include "vlog.hxx"

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

    // How often the worker writes its summary line to the round-trip log. A
    // second, because the board's PING is 1 Hz and its "no PONG" is judged in
    // seconds: a coarser line would put the moment the link stopped answering
    // inside a window too wide to line up against the board's journal.
    constexpr Int64 SUMMARY_MS = 1000;

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

    // ---- names, for the round-trip log only ---------------------------------
    //
    // bibowire.cxx spells verbs, reasons and severities too, but inside its own
    // anonymous namespace, so nothing outside the codec can reach them - and the
    // codec is shared with the board, so it is not edited for a viewer's log.
    // Spelled again HERE FOR THE LOG ONLY: nothing drawn or decided reads these,
    // and every line that uses one prints the number beside it, so a verb this
    // table has never heard of is a "?" with a value rather than a wrong word.

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

    // A socket error as its number AND Windows' own sentence for it. The number
    // is what a search finds; the sentence is what a person reads without one.
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
    // address and the local end is the tailnet interface; seeing both is what
    // says which path a connection actually took.
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

    // ---- what the round-trip log keeps about one connection -----------------

    // What ONE send did, kept so the callers that matter - the PONG above all,
    // whose answer is the case this log was written for - can say it in words:
    // how much the socket took, how often it would not, and how long that took.
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

    // One pass of the worker loop, by where its time went. `logMs` is the drain
    // of Heard into the file, measured apart so the log can never hide its own
    // cost inside the numbers it reports.
    struct PassSplit
    {
        Int64 selectMs = 0;
        Int64 recvMs = 0;
        Int64 logMs = 0;
        Int64 sendMs = 0;
        Int64 publishMs = 0;
    };

    // ONE SECOND of the socket half, written as the summary line and zeroed.
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

    // A COMMAND on the wire and not yet answered, so its CMDACK's line can say
    // how long the board took - the command's own round trip.
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

        // The frame counts as they stood at the previous summary, so a line can
        // say what arrived in THIS second rather than since the connect.
        Array<UInt32, 256> byTypeAtSummary = {};

        Int64 openedMs = 0;
        Int64 helloAtMs = 0;
        Int64 lastRecvMs = 0;       // the last recv() that returned bytes
        UInt64 rxTotal = 0;
        UInt64 txTotal = 0;

        // Why recv or select ended the connection, when one of them did. The
        // panel's `why` stays the sentence it always was; this is the detail.
        Str endedBy;

        // UDP errors: the first CONTROL failure is said in a line, the rest are
        // counted, and the last receive error is carried into the summary.
        Int32 udpError = 0;
        Int32 udpRxError = 0;
        Bool saidUdpFailure = false;

        // FIRST OF EACH KIND, said once per connection and counted after.
        Bool saidRefused = false;
        Bool saidUnknown = false;
        Bool saidResync = false;
        Bool saidFraming = false;
        Bool saidSeqJump = false;
        Bool saidOverflow = false;
        Bool saidBye = false;

        // CONTROL as last written to the log, so the next line is written only
        // when something on the wire has actually changed.
        Bool haveControlLine = false;
        bibowire::Control controlLine;
        Bool controlLineOnTcp = false;

        Vec<CmdOut> commandsOut;
    };

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

        // Everything the round-trip log knows about this connection. Reset with
        // it, because a tally that ran across a reconnect would blur the exact
        // moment this log exists to catch.
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

    // `note` is told what the socket did, whatever the answer - see SendNote.
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
                // The budget ran out with the socket still full. WOULDBLOCK is
                // the honest name for what ended it - no call returned an error.
                note.error = WSAEWOULDBLOCK;
                note.tookMs = monoMs() - started;
                return false;
            }
        }
        note.ok = true;
        note.tookMs = monoMs() - started;
        return true;
    }

    // One send, into this second's tally.
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

    // What one send did, in words.
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

    // One typed frame out. The header is assembled HERE and nowhere else, so no
    // call site ever picks its own seq - the per-connection counter is the
    // frame-header ring's and the loss accounting's, and two writers would make
    // it neither.
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

    // ---- dialling -----------------------------------------------------------

    // `err` is SO_ERROR when the socket has one, and WSAETIMEDOUT when the budget
    // simply ran out - so a refused port and a board that never answered are
    // logged as the two different facts they are.
    [[nodiscard]] Bool finishConnect(SOCKET fd, Int64 budgetMs, Int32& err)
    {
        const Bool writable = waitWritable(fd, budgetMs);
        // Writable is not the same fact as connected: a refused connection is
        // reported by SO_ERROR, and a socket that skipped this check would send
        // HELLO into a connection that never happened.
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

        // EVERY address the name produced, before any is tried. A name can
        // answer with more than one, and a dial that failed on the first and
        // succeeded on the second reads very differently from one that never
        // had a second choice.
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
            vlog::line(
                "UDP filter not applied: getpeername %s",
                wsaText(::WSAGetLastError()).c_str()
            );
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
            vlog::line(
                "UDP filter not applied: TCP peer family %u",
                static_cast<UInt32>(peer.ss_family)
            );
            return;
        }
        // A UDP socket bound to AF_INET cannot be connected to an AF_INET6 peer;
        // when that happens the filter is simply not applied and the datagrams
        // are still CRC-checked like everything else.
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

    // ---- pumping ------------------------------------------------------------

    [[nodiscard]] Bool pumpTcp(Conn& c, Session& s)
    {
        Journal& j = c.journal;
        for(;;)
        {
            if(c.rxUsed >= c.rx.size())
            {
                // 512 KiB with no frame in it. MAX_PAYLOAD is 256 KiB, so this
                // is not a big frame - it is a peer writing something that will
                // never parse.
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

            // THE GAP BETWEEN TWO RECEIVES THAT RETURNED BYTES. This is the
            // number that says whether board->viewer data stopped arriving at
            // the socket or arrived and was not acted on: the decode below and
            // the PONG after it both hang off this moment.
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
                // WOULDBLOCK is a drained socket. Anything else is counted:
                // Windows reports an ICMP port-unreachable for an EARLIER
                // datagram here, as 10054 on the next receive - which is the
                // board saying its UDP port is not open.
                const Int32 err = n < 0 ? ::WSAGetLastError() : 0;
                if(n < 0 && err != WSAEWOULDBLOCK)
                {
                    ++c.journal.wire.udpRxErrors;
                    c.journal.udpRxError = err;
                }
                return;
            }
            ++c.journal.wire.udpRx;
            // One datagram is one frame. take() checks the CRC, so a truncated
            // or forged datagram is dropped here rather than believed.
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

    // EVERY PONG, with how long after its PING it left and exactly what the
    // socket did with it. This is the line the board's "no PONG" is held
    // against: a PONG logged here as taken whole by the socket and never seen by
    // the board is a different fault from one this viewer never sent.
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
            // The token is echoed VERBATIM; senderMonoUs is our own clock, which
            // the board only ever compares with itself.
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
        const Bool sent = sendFrame(c, bibowire::Type::TYPE_PING, body.data(), n);
        vlog::line("PING token=%llu: %s", ping.token, sendText(c.journal.lastSend).c_str());
        return sent;
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
                // This one and everything behind it die with the connection, by
                // the same rule: the caller is told by the counter, not by a
                // retry onto a socket that has just failed.
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

            // Remembered, so its CMDACK's line can say how long the board took.
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
    //
    // A failure leaves its WSA code in the journal for the log to name.
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
            // No address the board is known to be at. Not a socket error at
            // all, so it is named as the one Winsock has for exactly this.
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

    // CONTROL ON CHANGE, never per datagram. The stream is twenty identical
    // frames a second, and what a person reading the log needs is the moment the
    // operator's hand did something - or the moment the wire stopped agreeing
    // with it: an epoch that moved, a transport that fell back.
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
        noteControl(c.journal, m, say.controlOnTcp);

        if(say.controlOnTcp)
        {
            // The same frame on a different socket. The board accepts CONTROL
            // on TCP always, with identical rules, identical deadman and
            // identical session and seq checks, so there is nothing to
            // negotiate and no second code path. A TCP send that fails IS the
            // connection failing - unlike the datagram below.
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
            ++c.journal.wire.udpTxFailed;
            // THE FIRST ONE IN WORDS, the rest in the summary's count: at 20 Hz
            // a line per failure would bury the keepalive lines this log is for.
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
        // A deliberate close says so rather than letting the board find out by
        // FIN. It costs one 24-byte frame and it is the difference between "the
        // operator left" and "the link died" in the board's own journal.
        //
        // Best effort, and the answer is consumed rather than cast away: this
        // socket is closing either way, and the board's own timers cover a LEAVE
        // that never made it onto the wire.
        const Bool sent = sendFrame(c, bibowire::Type::TYPE_LEAVE, body.data(), n);
        vlog::line(
            "LEAVE sessionId=%u %s: %s",
            leave.sessionId,
            sent ? "sent" : "NOT sent",
            sendText(c.journal.lastSend).c_str()
        );
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

    // ---- the round-trip log, the worker's side ------------------------------

    // Everything the decode told the log this pass, into the file. Board PINGs
    // STAY in the Heard until flushPongs has answered them, because each PONG's
    // line needs the moment its PING arrived.
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

        // THE FIRST OF EACH KIND in words, and every one after it only in the
        // summary's totals - a stream going bad would otherwise write a line
        // for every byte it could not read.
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

    // The kernel's own account of the TCP connection - what no counter in this
    // file can see: bytes the board has not acknowledged, retransmissions,
    // timeout episodes, the windows both ends are offering. When the board stops
    // hearing this viewer while these say every byte left and was acknowledged,
    // the fault is not in this process.
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
        // SAID rather than left out, so a reader never wonders whether these
        // numbers were zero or simply never asked for.
        out += " | tcp_info: SIO_TCP_INFO is not in this Windows SDK";
#endif
    }

    // ONCE A SECOND, the socket half in ONE line. One rather than several, so a
    // search for the moment the board said "no PONG" lands on everything this
    // end knew about that second at once.
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
        // NO OUTBOUND QUEUE, said rather than left as a missing number: sendAll
        // blocks for up to SEND_BUDGET_MS until the socket takes the whole
        // frame, so what would be a backlog here shows up as a slow send instead
        // - and the kernel's own unacknowledged bytes are in tcp_info below.
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

    // The reconnect schedule's next wait, said out loud - the attempt, the base
    // and the jittered answer - because "why did it take four seconds to come
    // back" is a question the log should answer without arithmetic.
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
                waitToRetry(*c, live, retryDelay(attempt, seed, why), why);
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
                vlog::line("CONTROL goes to TCP from the first datagram: there is no UDP socket");
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
                    retryDelay(attempt, seed, "could not send HELLO"),
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

            // The summary's first second starts with the connection.
            conn.journal.wire.windowMs = monoMs();

            while(!c->quit.load())
            {
                // WHERE EACH PASS SPENDS ITS TIME, in the same milliseconds as
                // everything else here. A pass that took 500 ms in `send` is a
                // socket that would not take a frame; one that took it in
                // `publish` is this thread waiting on the UI's lock - and those
                // are different fixes.
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

                // PUMPED, THEN DRAINED INTO THE LOG, THEN JUDGED. The frames
                // that arrived just before a close are the ones most worth
                // reading, and breaking first would throw their lines away.
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

                    logWelcome(conn, live);
                    if(!holdsSlot(live))
                    {
                        // One of the sentences behind "I cannot drive": an
                        // observer's CONTROL stream is never sent at all.
                        vlog::line("no control slot on this connection - no CONTROL is sent");
                    }
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

            // The LEAVE first when this viewer is the one leaving, so the close
            // line below comes after everything that went on the wire. Then the
            // partial second before the close - the second this log exists to
            // catch - and the close itself, with its reason.
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
      shot.worstCaptureMs = cameraWorstCaptureMs;
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

          // THE GAP BETWEEN TWO CAPTURES, on the board's clock. The first frame
          // only starts it - there is no gap before a feed's first picture, and
          // inventing one from a zero initial timestamp would report the whole
          // uptime of the board as a stall. noteArrival states that rule for
          // arrivals; this is the same rule for the other clock.
          //
          // A timestamp that went BACKWARDS is a restarted board, not a
          // negative gap: skipped rather than recorded, because the safe
          // reading of a clock that moved the wrong way is "measure again".
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

  namespace
  {

    // Bounded, and what does not fit is counted rather than silently lost.
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

  // A WRAPPER AROUND THE DECODE, not a change to it. The three-argument
  // ingestFrame above is exactly what the suite holds to an answer, and threading
  // a log through its fourteen early returns would put the log's bookkeeping
  // inside the code it is meant to watch. Everything here is read before or
  // after, from the frame and the session.
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

      // READ BEFORE the frame is ingested: ingesting overwrites lastFrameMs, and
      // for a PONG it removes the very PING the answer is matched against.
      const Int64 quietMs = nowMs - s.lastFrameMs;
      const UInt32 refusedBefore = s.refusedFrames;
      const UInt32 unknownBefore = s.unknownFrames;

      // The token decoded a SECOND time - sixteen bytes, once a second - rather
      // than threading the log through the decode it is watching.
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

      // UP, not CHANGED: a WELCOME from a restarted board clears the whole
      // session, its counters included, and that is not a refusal.
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
              // Here and never for a datagram: the TCP stream is the one whose
              // seqs run as a sequence, and a datagram arrives outside it.
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
              // Junk on a checksummed stream that is never counted is a fault
              // nobody discovers.
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
          // TOO_BIG or BAD_FLAG: a terminal answer about a frame at the stream
          // position. Counted, and the byte is stepped over so the caller's ring
          // can never wedge on it - the connection is torn down by the silence
          // watchdog if the peer keeps it up.
          ++s.refusedFrames;
          if(heard != nullptr)
          {
              ++heard->framingRefused;
          }
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
          // sessionId and armEpoch are stamped by the worker at the moment of
          // sending - see link.hxx. Left at their defaults here on purpose, so a
          // reader of this function cannot mistake a snapshot for the connection.

          ++c.nextCmdId;
          if(c.nextCmdId == 0u)
          {
              // NEVER 0. Unreachable at any human rate - it is 4.2 billion
              // deliberate acts - and written anyway, because the alternative is
              // a rule enforced by an arithmetic coincidence.
              c.nextCmdId = 1u;
          }

          c.pending.push_back(cmd);
          while(c.pending.size() > MAX_PENDING_COMMANDS)
          {
              // The OLDEST goes, and it is counted. A queue this deep means the
              // worker is not draining, and in that case the newest intent is
              // the one worth keeping - the same newest-wins rule the rest of
              // this protocol follows.
              c.pending.erase(c.pending.begin());
              c.commandsDropped.fetch_add(1u);
              ++evicted;
          }
          waiting = c.pending.size();
      }

      // Written AFTER the lock is released: a slow disk must never be the thing
      // the worker waits on to take the next command off the queue.
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
      vlog::line("link reconnect: %s port %u", host.c_str(), static_cast<UInt32>(port));
      close(c);
      return open(c, host.c_str(), port);
  }

}
