#include "viewfeed.hxx"

#include <cstdio>

#if defined(__linux__)

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace viewfeed
{

  namespace
  {

    // feed.cxx:26's number, unchanged. A client whose oldest unsent VITAL byte
    // is older than this has gone, whatever its socket says.
    constexpr Int64 BEHIND_MS = 500;

    // The longest the loop sleeps with nothing to do. Smaller than feed.cxx's
    // 250 because CTLSTATE owes a datagram every 50 ms and a loop that sleeps
    // through four of them is a loop that reports the deadman late.
    constexpr Int32 POLL_MS = 20;

    // The bounded ring, per client: 96 KiB or 12 frames, whichever fills first.
    constexpr Size RING_BYTES = 96u * 1024u;
    constexpr Size RING_FRAMES = 12;

    // A revolution older than this at SEND time is dropped before it is ever
    // queued. Sending it would spend the bandwidth the current revolution needs
    // in order to show something already wrong.
    constexpr Int64 LIVE_STALE_MS = 200;

    // Half-open connections, covered a second and a third time over: the
    // board's own PING, and SO_KEEPALIVE below. A phone that walks out of range
    // stops ACKing without ever sending a FIN.
    constexpr Int64 PING_EVERY_MS = 1000;
    constexpr Int64 PONG_WAIT_MS = 4000;

    constexpr Int64 CTLSTATE_EVERY_MS = 50;
    constexpr Int64 BOARD_EVERY_MS = 200;

    // 10 events a second, with the suppressed count carried in the next one.
    constexpr Int64 EVENT_WINDOW_MS = 100;

    // Per TCP client. A header claiming more than MAX_INBOUND_PAYLOAD never
    // gets this far - see consume() - so nothing sizes an allocation from a
    // number a stranger on a hotspot wrote.
    constexpr Size INBUF_BYTES = 1024;

    // Deliberately SMALL. A deep queue of control datagrams is a queue of stale
    // steering commands, and the board wants the newest, not the most.
    constexpr Int32 UDP_RCVBUF = 64 * 1024;

    // The largest frame v1 ever builds: a 1024-point SCAN. MAX_PAYLOAD is
    // 256 KiB and is sized for a JPEG nobody sends yet, so preallocating that
    // would be 256 KiB held for a frame that cannot occur.
    constexpr Size SCAN_BODY_MAX = 24u + 5u * bibowire::MAX_SCAN_POINTS;
    constexpr Size ENCODE_BYTES = bibowire::FRAME_OVERHEAD + SCAN_BODY_MAX;
    constexpr Size BODY_CAP = ENCODE_BYTES - bibowire::FRAME_OVERHEAD;

    // The last 256 frame headers PER DIRECTION, dumped on any abnormal close so
    // a disconnect can be post-mortemed from a phone over ssh.
    //
    // Section 8 asks for these in "a fixed 4 KiB array", which does not divide:
    // 512 entries carrying type, len, seq, tMonoUs and a CRC verdict is 16 bytes
    // each however they are packed, and 4 KiB buys 8-byte entries. The DEPTH and
    // the FIELDS are what a post-mortem reads, so they are what is kept; the
    // array is 8 KiB and this comment is the correction rather than a silently
    // shortened ring.
    constexpr Size NOTE_RING = 256;

    struct Note
    {
        UInt64 tMonoUs = 0;
        UInt32 len = 0;
        UInt16 seq = 0;
        UInt8 type = 0;
        UInt8 verdict = 0;   // 0 ok, 1 bad crc / resync, 2 refused
    };

    constexpr UInt8 NOTE_OK = 0;
    constexpr UInt8 NOTE_JUNK = 1;
    constexpr UInt8 NOTE_REFUSED = 2;

    // What a client is: a socket that has not said HELLO yet, or a viewer.
    enum class Stage
    {
        STAGE_WAIT_HELLO = 0,
        STAGE_LIVE,
    };

    // One encoded frame, waiting for a socket that would not take it yet.
    struct Queued
    {
        Vec<UInt8> bytes;
        bibowire::Class cls = bibowire::Class::CLASS_VITAL;
        bibowire::Type type = bibowire::Type::TYPE_PING;
        TimePoint at;
    };

    struct Client
    {
        Int32 fd = -1;
        Str peer;     // ip:port, for the log
        Str peerIp;   // ip alone, for the CTLSTATE datagram and the refusal sentence
        Stage stage = Stage::STAGE_WAIT_HELLO;

        // Fixed, never grown: the receive path has no heap in it at all.
        Array<UInt8, INBUF_BYTES> in{};
        Size inLen = 0;

        Deque<Queued> out;
        Size outBytes = 0;
        Size sentOfHead = 0;   // bytes of out.front() the socket has taken

        UInt32 sessionId = 0;
        Bool holder = false;
        Bool wantedControl = false;
        UInt16 udpPort = 0;
        Str name;

        UInt16 txSeq = 0;
        UInt16 droppedLive = 0;   // revolutions discarded for THIS client

        // The reverse-path probe: at least REVERSE_PROBE_MIN datagrams within
        // REVERSE_PROBE_MS of WELCOME, or the board says so in words.
        TimePoint welcomedAt;
        UInt32 datagrams = 0;
        Bool probeSaid = false;

        // The viewer fell back to CONTROL over TCP, so CTLSTATE is mirrored
        // there as well.
        Bool tcpControl = false;

        // What the viewer asked for. A zero mask is "never asked", which is
        // everything - a viewer that never subscribes is not a viewer that
        // wants nothing.
        UInt32 typeMask = 0;
        UInt16 scanDivisor = 1;
        UInt32 revSeen = 0;

        // What the viewer said it understands. ADVISORY, and recorded for the
        // log rather than acted on - see typeBit(). An unknown bit here is
        // ignored, never refused: the whole point of the length prefix is that
        // a type a reader has no name for is skipped, so a viewer claiming one
        // this board has never heard of costs nothing.
        UInt32 features = 0;

        TimePoint lastPingAt;
        TimePoint pingSentAt;
        UInt64 pingToken = 0;
        Bool pingOut = false;

        TimePoint lastCtlAt;

        Str dropWhy;
        Bool abnormal = false;   // dump the frame-header ring on the way out

        Size at = 0;   // where this client's fd sat in the pollfd vector
    };

    enum class What
    {
        WHAT_SCAN = 0,
        WHAT_DECIDE,
        WHAT_BOARD,
        WHAT_LIDAR,
        WHAT_EVENT,
    };

    struct Item
    {
        What what = What::WHAT_SCAN;
        bibowire::Scan scan;
        bibowire::Decide decide;
        bibowire::BoardState board;
        bibowire::LidarInfo lidar;
        bibowire::Event event;
        TimePoint at;
    };

    // Everything the two threads share. Every member is touched under `m`
    // except wakeFd, `count` and the seqlock, which are atomic so the pilot's
    // tick never takes this lock.
    struct Shared
    {
        Mutex m;
        Deque<Item> items;
        Bool quit = false;
        Int32 wakeFd = -1;
        Atomic<Size> count{ 0 };

        // The seqlock over two slots. The UDP receive path writes
        // slot[seq & 1] and then STORES seq with release; control() loads with
        // acquire, reads, re-loads, and retries if it moved.
        Atomic<UInt32> ctlSeq{ 0 };
        Array<bibowire::Control, 2> ctlSlot{};

        Mutex appliedM;
        Applied applied;

        Mutex tallyM;
        Counters tally;
    };

    Shared sh;
    Policy policy;
    Int32 listenFd = -1;
    Int32 udpFd = -1;
    Array<Int32, 2> wake{ -1, -1 };
    UInt16 boundPort = 0;
    Bool running = false;
    Thread worker;
    TimePoint startedAt;

    // The board's own state, owned by this thread.
    UInt32 armEpoch = 0;
    Bool estopLatched = false;
    Bool haveHolder = false;
    UInt32 holderSession = 0;
    TimePoint lastControlAt;
    Bool everControl = false;
    UInt32 appliedSeq = 0;
    Bool holderGone = false;   // positive evidence the driver left

    // The newest state a client accepted a moment from now is owed BEFORE it is
    // shown a single point. State before scan, always.
    bibowire::BoardState lastBoard;
    Bool haveBoard = false;
    bibowire::LidarInfo lastLidar;
    Bool haveLidar = false;
    TimePoint lastBoardAt;
    Bool boardSent = false;

    // When the last revolution reached this module. THE BOARD MEASURING ITSELF,
    // which is what catches a healthy link carrying dead data - the one lie a
    // viewer cannot detect from its own clock.
    TimePoint lastScanAt;
    Bool haveScan = false;

    TimePoint lastEventAt;
    UInt16 eventsSuppressed = 0;
    Bool eventWindowOpen = false;

    Array<Note, NOTE_RING> notesIn{};
    Array<Note, NOTE_RING> notesOut{};
    Size notesInAt = 0;
    Size notesOutAt = 0;

    Array<UInt8, ENCODE_BYTES> scratch{};

    // ---- small helpers -----------------------------------------------------

    [[nodiscard]] UInt64 monoUs()
    {
        return static_cast<UInt64>(elapsedMs(startedAt) * 1000.0);
    }

    Void wakeLoop()
    {
        const Char one = 1;
        if(::write(sh.wakeFd, &one, 1) < 0)
        {
            // Deliberately nothing: a full pipe means the loop is awake
            // already. See feed.cxx for the same silence and the same reason.
        }
    }

    Void post(Item item)
    {
        {
            LockGuard<Mutex> lock(sh.m);
            sh.items.push_back(std::move(item));
        }
        wakeLoop();
    }

    Void countFrame(Bool dropped)
    {
        LockGuard<Mutex> lock(sh.tallyM);
        if(dropped)
        {
            ++sh.tally.txDroppedFrames;
        }
        else
        {
            ++sh.tally.txFrames;
        }
    }

    Void countEncode(Float64 ns)
    {
        LockGuard<Mutex> lock(sh.tallyM);
        const UInt32 was = sh.tally.encodeAvgNs;
        // A running mean that costs one multiply and never allocates. The exact
        // average of every frame ever sent is not the useful number; what this
        // viewer is costing now is.
        sh.tally.encodeAvgNs = was == 0
            ? static_cast<UInt32>(ns)
            : static_cast<UInt32>((static_cast<Float64>(was) * 7.0 + ns) / 8.0);
        if(static_cast<UInt32>(ns) > sh.tally.encodeMaxNs)
        {
            sh.tally.encodeMaxNs = static_cast<UInt32>(ns);
        }
    }

    // A session id that is random enough that a datagram from a previous
    // session cannot be mistaken for this one, and NEVER 0.
    [[nodiscard]] UInt32 freshSession()
    {
        static UInt32 state = 0;
        if(state == 0u)
        {
            state = static_cast<UInt32>(monoUs()) ^ 0x9E3779B9u;
        }
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        state ^= static_cast<UInt32>(monoUs());
        return state == 0u ? 1u : state;
    }

    enum class Dir
    {
        DIR_IN = 0,
        DIR_OUT,
    };

    // The ring and its cursor were two parameters saying one thing - which
    // direction - in a 103-column signature. docs/conventions.md is right that
    // the length was the symptom rather than the problem.
    Void note(Dir dir, const bibowire::Head& h, Size len, UInt8 verdict)
    {
        Note n;
        n.tMonoUs = monoUs();
        n.len = static_cast<UInt32>(len);
        n.seq = h.seq;
        n.type = static_cast<UInt8>(h.type);
        n.verdict = verdict;
        Array<Note, NOTE_RING>& ring = dir == Dir::DIR_IN ? notesIn : notesOut;
        Size& at = dir == Dir::DIR_IN ? notesInAt : notesOutAt;
        ring[at] = n;
        at = (at + 1u) % NOTE_RING;
    }

    // Written to the journal on any abnormal close, which is what makes a
    // disconnect something a person can post-mortem with journalctl from a
    // phone rather than something they have to reproduce.
    Void dumpNotes(const Client& c)
    {
        std::printf("viewfeed: frame ring for %s (%s)\n", c.peer.c_str(), c.dropWhy.c_str());
        for(Size d = 0; d < 2u; ++d)
        {
            const Array<Note, NOTE_RING>& ring = d == 0u ? notesIn : notesOut;
            const Size at = d == 0u ? notesInAt : notesOutAt;
            for(Size i = 0; i < NOTE_RING; ++i)
            {
                const Note& n = ring[(at + i) % NOTE_RING];
                if(n.tMonoUs == 0u && n.type == 0u)
                {
                    continue;
                }
                std::printf(
                    "  %s %-14s seq %5u len %6u at %llu us%s\n",
                    d == 0u ? "rx" : "tx",
                    bibowire::typeName(static_cast<bibowire::Type>(n.type)),
                    static_cast<unsigned>(n.seq),
                    static_cast<unsigned>(n.len),
                    static_cast<unsigned long long>(n.tMonoUs),
                    n.verdict == NOTE_OK ? "" : (n.verdict == NOTE_JUNK ? "  JUNK" : "  REFUSED")
                );
            }
        }
    }

    // ---- the deadman, and who is holding the wheel -------------------------

    [[nodiscard]] bibowire::deadman::Output deadmanNow()
    {
        bibowire::deadman::Inputs in;
        in.nowMs = static_cast<Int64>(elapsedMs(startedAt));
        // A LEAVE, a FIN or an RST from the holder is POSITIVE EVIDENCE the
        // driver is gone. Spending a sixth of a second rediscovering that by
        // timeout is a sixth of a second of a car driving on a command from a
        // viewer that is provably not there, so the age is forced past DEAD
        // rather than allowed to run down.
        const Int64 gone = in.nowMs - static_cast<Int64>(bibowire::CONTROL_DEAD_MS);
        in.lastControlMs = holderGone || !everControl
            ? gone
            : static_cast<Int64>(elapsedMs(startedAt) - elapsedMs(lastControlAt));
        in.haveHolder = haveHolder;
        in.estopLatched = estopLatched;

        UInt32 seq = sh.ctlSeq.load(std::memory_order_acquire);
        if(seq != 0u)
        {
            const bibowire::Control& c = sh.ctlSlot[seq & 1u];
            in.enable = (c.buttons & bibowire::BUTTON_ENABLE) != 0u;
            in.epochMatches = c.armEpoch == static_cast<UInt8>(armEpoch);
            in.modeAgrees = true;
        }
        return bibowire::deadman::step(in);
    }

    [[nodiscard]] UInt16 outFlags()
    {
        const bibowire::deadman::Output d = deadmanNow();
        UInt16 flags = 0;
        if(estopLatched)
        {
            flags = static_cast<UInt16>(flags | bibowire::FLAG_ESTOP);
        }
        if(d.state != bibowire::deadman::State::STATE_LIVE)
        {
            flags = static_cast<UInt16>(flags | bibowire::FLAG_DEADMAN);
        }
        return flags;
    }

    [[nodiscard]] UInt8 deadmanByte()
    {
        const bibowire::deadman::Output d = deadmanNow();
        switch(d.state)
        {
        case bibowire::deadman::State::STATE_LIVE:
            return 0;
        case bibowire::deadman::State::STATE_SOFT:
            return 1;
        case bibowire::deadman::State::STATE_DEAD:
            return 2;
        default:
            return 3;
        }
    }

    // The epoch is bumped on a deadman disarm, an e-stop, a Pico link loss and
    // a CONTROL-SLOT CHANGE - which is what disarms a displaced viewer for
    // free, and why handing the wheel over cannot leave the old holder's
    // throttle believed.
    Void bumpEpoch()
    {
        armEpoch = (armEpoch + 1u) & 0xFFu;
    }

    // ---- encoding, framing and the drop classes ----------------------------

    [[nodiscard]] Size framed(bibowire::Type type, Size bodyLen, UInt16 seq, UInt16 flags)
    {
        bibowire::Head h;
        h.type = type;
        h.ver = 1;
        h.flags = flags;
        h.seq = seq;
        bibowire::Body b;
        b.bytes = scratch.data() + bibowire::HEAD_BYTES;
        b.len = bodyLen;
        // The body was written straight into the frame buffer, so put() has
        // nothing to copy and the framing costs a header and a CRC.
        return bibowire::put(h, b, scratch.data(), scratch.size());
    }

    [[nodiscard]] UInt8* bodyAt()
    {
        return scratch.data() + bibowire::HEAD_BYTES;
    }

    // The newest BULK, else the oldest LIVE. Never the head when the socket has
    // already taken part of it - half a frame followed by a different frame is
    // a stream the far end cannot resynchronise without help.
    [[nodiscard]] Bool discardOne(Client& c)
    {
        const Size first = c.sentOfHead > 0u ? 1u : 0u;
        for(Size i = c.out.size(); i > first; --i)
        {
            if(c.out[i - 1u].cls == bibowire::Class::CLASS_BULK)
            {
                c.outBytes -= c.out[i - 1u].bytes.size();
                c.out.erase(c.out.begin() + static_cast<ISize>(i - 1u));
                countFrame(true);
                return true;
            }
        }
        for(Size i = first; i < c.out.size(); ++i)
        {
            if(c.out[i].cls == bibowire::Class::CLASS_LIVE)
            {
                if(c.out[i].type == bibowire::Type::TYPE_SCAN)
                {
                    ++c.droppedLive;
                }
                c.outBytes -= c.out[i].bytes.size();
                c.out.erase(c.out.begin() + static_cast<ISize>(i));
                countFrame(true);
                return true;
            }
        }
        return false;
    }

    // Puts the frame now sitting in `scratch` on this client's ring, under the
    // classes of section 7.
    Void enqueue(Client& c, bibowire::Type type, Size total)
    {
        if(!c.dropWhy.empty() || total == 0u)
        {
            return;
        }
        const bibowire::Class cls = bibowire::classOf(type);

        // LIVE is drop-oldest, DEPTH 1: the newest revolution wins, and the
        // count travels in the next SCAN's droppedSinceLast so the viewer knows
        // what it missed rather than believing it saw everything.
        if(cls == bibowire::Class::CLASS_LIVE)
        {
            const Size first = c.sentOfHead > 0u ? 1u : 0u;
            for(Size i = c.out.size(); i > first; --i)
            {
                if(c.out[i - 1u].type != type)
                {
                    continue;
                }
                if(type == bibowire::Type::TYPE_SCAN)
                {
                    ++c.droppedLive;
                }
                c.outBytes -= c.out[i - 1u].bytes.size();
                c.out.erase(c.out.begin() + static_cast<ISize>(i - 1u));
                countFrame(true);
            }
        }

        while(c.outBytes + total > RING_BYTES || c.out.size() + 1u > RING_FRAMES)
        {
            if(!discardOne(c))
            {
                break;
            }
        }

        if(c.outBytes + total > RING_BYTES || c.out.size() + 1u > RING_FRAMES)
        {
            // A viewer that cannot absorb 120 bytes of state has gone, whatever
            // its socket says. VITAL is never dropped, so the client is what
            // gives way.
            if(cls == bibowire::Class::CLASS_VITAL)
            {
                c.dropWhy = "the vital ring filled";
                c.abnormal = true;
                return;
            }
            if(type == bibowire::Type::TYPE_SCAN)
            {
                ++c.droppedLive;
            }
            countFrame(true);
            return;
        }

        Queued q;
        q.bytes.assign(scratch.data(), scratch.data() + total);
        q.cls = cls;
        q.type = type;
        q.at = monoNow();
        c.outBytes += total;
        c.out.push_back(std::move(q));

        bibowire::Head h;
        h.type = type;
        h.seq = c.txSeq;
        note(Dir::DIR_OUT, h, total, NOTE_OK);
        ++c.txSeq;
    }

    // Builds one frame for one client and queues it. `write` fills the body and
    // returns its length, or 0 when the message could not be represented - a
    // board that emitted a frame its own reader would refuse would have moved a
    // bug from the encoder into somebody else's decoder.
    template<typename Fill>
    Void emit(Client& c, bibowire::Type type, Fill fill)
    {
        const TimePoint before = monoNow();
        const Size bodyLen = fill(bodyAt(), BODY_CAP);
        if(bodyLen == 0u)
        {
            return;
        }
        const Size total = framed(type, bodyLen, c.txSeq, outFlags());
        countEncode(elapsedMs(before) * 1000000.0);
        enqueue(c, type, total);
    }

    // ---- the mask convention, written down because nothing else writes it ----
    //
    // Section 5 calls SUBSCRIBE.typeMask "a bit per telemetry type" and names no
    // bit anywhere; HELLO.featureMask and WELCOME.featureMask are given no
    // convention at all, in the document, in bibowire.hxx or in its 312 checks.
    // A viewer reading that has nothing to send but every bit set. So the rule
    // is settled HERE, and the suite asserts it:
    //
    //   bit = tag - 0x10, for tags 0x10..0x2F - the board->viewer range, which
    //   is exactly what "telemetry type" means. One mapping serves all three
    //   fields, so a reader who learns it once has learned it everywhere.
    //
    // The obvious mapping - `1u << (tag & 0x1F)` - is the bug this replaced:
    // DECIDE (0x11) and SCHEMA (0xF1) land on the same bit, so subscribing to
    // one silently subscribes to the other.
    //
    // A type OUTSIDE that range has no bit and is always sent. WELCOME, BYE,
    // PING and CMDACK are the plumbing that carries the mask negotiation
    // itself; a mask that could switch them off would be a mask that could
    // switch off the way to change it.
    [[nodiscard]] constexpr UInt32 typeBit(bibowire::Type type)
    {
        const UInt8 tag = static_cast<UInt8>(type);
        return tag >= 0x10u && tag <= 0x2Fu ? (1u << (tag - 0x10u)) : 0u;
    }

    // What this board actually sends, for WELCOME.featureMask - which section 4
    // defines as "what this board will send", and which is therefore a fact
    // about the build rather than an echo of what the viewer asked for.
    [[nodiscard]] UInt32 boardFeatures()
    {
        return typeBit(bibowire::Type::TYPE_SCAN) | typeBit(bibowire::Type::TYPE_DECIDE)
             | typeBit(bibowire::Type::TYPE_BOARD) | typeBit(bibowire::Type::TYPE_LIDAR_INFO)
             | typeBit(bibowire::Type::TYPE_EVENT) | typeBit(bibowire::Type::TYPE_CTLSTATE)
             | typeBit(bibowire::Type::TYPE_CMDACK);
    }

    [[nodiscard]] Bool wants(const Client& c, bibowire::Type type)
    {
        if(c.stage != Stage::STAGE_LIVE || !c.dropWhy.empty())
        {
            return false;
        }
        const UInt32 bit = typeBit(type);
        if(bit == 0u)
        {
            return true;
        }
        // A viewer that never subscribed is not a viewer that wants nothing.
        if(c.typeMask == 0u)
        {
            return true;
        }
        return (c.typeMask & bit) != 0u;
    }

    // ---- the socket ends ---------------------------------------------------

    [[nodiscard]] Bool flush(Client& c)
    {
        while(!c.out.empty())
        {
            Queued& q = c.out.front();
            const ISize n = ::send(
                c.fd,
                q.bytes.data() + c.sentOfHead,
                q.bytes.size() - c.sentOfHead,
                MSG_NOSIGNAL | MSG_DONTWAIT
            );
            if(n > 0)
            {
                c.sentOfHead += static_cast<Size>(n);
                if(c.sentOfHead >= q.bytes.size())
                {
                    c.outBytes -= q.bytes.size();
                    c.sentOfHead = 0;
                    c.out.pop_front();
                    countFrame(false);
                }
                continue;
            }
            if(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                break;
            }
            c.dropWhy = n == 0 ? Str("closed") : Str(std::strerror(errno));
            return false;
        }
        return true;
    }

    // The oldest VITAL frame still owed, and how long it has been owed. This is
    // the test that closes a viewer which is up, connected, and not reading.
    Void checkBehind(Client& c)
    {
        for(const Queued& q : c.out)
        {
            if(q.cls != bibowire::Class::CLASS_VITAL)
            {
                continue;
            }
            if(elapsedMs(q.at) > static_cast<Float64>(BEHIND_MS))
            {
                Array<Char, 64> why{};
                std::snprintf(
                    why.data(),
                    why.size(),
                    "%.0f ms behind on a vital frame",
                    elapsedMs(q.at)
                );
                c.dropWhy = why.data();
                c.abnormal = true;
            }
            return;
        }
    }

    Void sayBye(Client& c, bibowire::Reason reason, const Str& text)
    {
        bibowire::Bye m;
        m.reason = reason;
        m.text = text;
        emit(c, bibowire::Type::TYPE_BYE, [&m](UInt8* out, Size cap) {
            return bibowire::writeBye(m, out, cap);
        });
        // Pushed as far as the socket will take it now, because the next thing
        // that happens to this client is a close.
        static_cast<Void>(flush(c));
    }

    Void refuse(Client& c, bibowire::Reason reason, const Str& text, const Str& why)
    {
        sayBye(c, reason, text);
        c.dropWhy = why;
        c.abnormal = true;
        LockGuard<Mutex> lock(sh.tallyM);
        ++sh.tally.refused;
    }

    // ---- the handshake -----------------------------------------------------

    [[nodiscard]] Size liveClients(const Vec<Client>& clients)
    {
        Size n = 0;
        for(const Client& c : clients)
        {
            if(c.stage == Stage::STAGE_LIVE && c.dropWhy.empty())
            {
                ++n;
            }
        }
        return n;
    }

    [[nodiscard]] Str holderName(const Vec<Client>& clients)
    {
        for(const Client& c : clients)
        {
            if(c.holder)
            {
                return c.name.empty() ? c.peer : c.name + " (" + c.peer + ")";
            }
        }
        return Str("nobody");
    }

    // The pilot fills the CAR's state; these are the fields only this module
    // can know - the deadman it computes, the epoch it owns, who is holding the
    // wheel, and what serving this viewer cost. The dashboard's JSON carries
    // none of them, so section 5's "one struct, one tick" rule is untouched:
    // the pilot still fills what the phone and the viewer both read.
    [[nodiscard]] bibowire::BoardState boardFor(const Client& c)
    {
        bibowire::BoardState b = lastBoard;
        b.deadman = deadmanByte();
        b.armEpoch = static_cast<UInt8>(armEpoch);
        b.controlHolder = c.holder ? 1u : (haveHolder ? 2u : 0u);
        b.clients = static_cast<UInt8>(sh.count.load());
        LockGuard<Mutex> lock(sh.tallyM);
        b.txDroppedFrames = static_cast<UInt32>(sh.tally.txDroppedFrames);
        b.rxControl = static_cast<UInt32>(sh.tally.rxControl);
        b.rxControlStale = static_cast<UInt32>(sh.tally.rxControlStale);
        b.encodeAvgNs = sh.tally.encodeAvgNs;
        b.encodeMaxNs = sh.tally.encodeMaxNs;
        return b;
    }

    Void sendState(Client& c)
    {
        // LIDAR_INFO then BOARD, then the next SCAN. State before scan, always,
        // so the viewer has something TRUE to draw the moment the picture
        // appears rather than a corridor overlay with no board behind it.
        if(haveLidar)
        {
            emit(c, bibowire::Type::TYPE_LIDAR_INFO, [](UInt8* out, Size cap) {
                return bibowire::writeLidarInfo(lastLidar, out, cap);
            });
        }
        if(haveBoard)
        {
            const bibowire::BoardState b = boardFor(c);
            emit(c, bibowire::Type::TYPE_BOARD, [&b](UInt8* out, Size cap) {
                return bibowire::writeBoard(b, out, cap);
            });
        }
    }

    Void onHello(Client& c, const bibowire::Body& body, UInt8 ver, Vec<Client>& clients)
    {
        // A second HELLO on a live connection is a protocol error and closes
        // it. There is one handshake per socket; a second one is either a
        // confused viewer or somebody else's bytes.
        if(c.stage != Stage::STAGE_WAIT_HELLO)
        {
            refuse(
                c,
                bibowire::Reason::REASON_REFUSED,
                "a second HELLO on a live connection",
                "second HELLO"
            );
            return;
        }

        bibowire::Hello h;
        if(!bibowire::readHello(body, ver, &h))
        {
            refuse(c, bibowire::Reason::REASON_REFUSED, "HELLO did not read whole", "bad HELLO");
            return;
        }
        c.name = h.name;
        c.udpPort = h.viewerUdpPort;
        c.wantedControl = h.wantControl != 0u;
        // Recorded, and every bit of it accepted. A viewer with no convention
        // to follow sends all ones, which is the only value that cannot be
        // misread as "understands nothing" - so refusing an unrecognised bit
        // would refuse the most sensible thing a viewer can say.
        c.features = h.featureMask;

        // protoMajor must be EQUAL. Not >=, not "compatible" - and the answer
        // names both numbers and the board's build, because a refusal that says
        // only "incompatible" sends a person to read source in a field.
        if(!bibowire::versionOk(h.protoMajor))
        {
            const bibowire::Bye m = bibowire::versionRefusal(h, policy.boardBuild.c_str());
            refuse(c, m.reason, m.text, "version");
            return;
        }

        // The board answers EVERY well-formed HELLO, including one it refuses.
        // Silence is never an answer, because on this link silence already
        // means four other things.
        bibowire::Welcome w;
        w.sessionId = freshSession();
        w.bootId = policy.bootId;
        w.boardMonoUs = monoUs();
        w.armEpoch = static_cast<UInt8>(armEpoch);
        w.capabilities = policy.capabilities;
        w.featureMask = boardFeatures();
        w.boardName = policy.boardName;

        if(liveClients(clients) >= bibowire::MAX_CLIENTS)
        {
            Str said = "four viewers already: ";
            for(const Client& other : clients)
            {
                if(other.stage == Stage::STAGE_LIVE && other.dropWhy.empty())
                {
                    said += other.peer + " ";
                }
            }
            w.accepted = 0;
            w.refusal = 3;
            w.text = said;
            c.sessionId = w.sessionId;
            emit(c, bibowire::Type::TYPE_WELCOME, [&w](UInt8* out, Size cap) {
                return bibowire::writeWelcome(w, out, cap);
            });
            refuse(c, bibowire::Reason::REASON_REFUSED, said, "too many viewers");
            return;
        }

        if(c.wantedControl && !haveHolder)
        {
            c.holder = true;
            haveHolder = true;
            holderSession = w.sessionId;
            holderGone = false;
            everControl = false;
            appliedSeq = 0;
            // A control-slot change bumps the epoch, which disarms whoever held
            // it before without a second mechanism.
            bumpEpoch();
            w.armEpoch = static_cast<UInt8>(armEpoch);
            w.accepted = 1;
            w.text = "control is yours";
        }
        else if(c.wantedControl)
        {
            w.accepted = 2;
            w.refusal = 2;
            w.text = "control is held by " + holderName(clients) + " - you are an observer";
        }
        else
        {
            w.accepted = 2;
            w.text = "observing";
        }

        c.sessionId = w.sessionId;
        c.stage = Stage::STAGE_LIVE;
        c.welcomedAt = monoNow();
        c.lastPingAt = monoNow();
        c.lastCtlAt = monoNow();

        emit(c, bibowire::Type::TYPE_WELCOME, [&w](UInt8* out, Size cap) {
            return bibowire::writeWelcome(w, out, cap);
        });
        sendState(c);

        std::printf(
            "viewfeed: %s welcomed as %s, session %08x\n",
            c.peer.c_str(),
            w.accepted == 1u ? "the driver" : "an observer",
            static_cast<unsigned>(c.sessionId)
        );
        LockGuard<Mutex> lock(sh.tallyM);
        ++sh.tally.accepted;
    }

    // The slot is released by LEAVE, by close, or by CONTROL_SLOT_MS of
    // silence. 1000 ms and not 300: the car has ALREADY been stopped by the
    // deadman at 300, and handing the wheel to somebody else 300 ms into a
    // stall - while the first operator is still holding the throttle and about
    // to come back - would be worse than the stall.
    Void releaseSlot(Client& c, CharSeq why)
    {
        if(!c.holder)
        {
            return;
        }
        c.holder = false;
        haveHolder = false;
        holderSession = 0;
        holderGone = true;
        bumpEpoch();
        std::printf("viewfeed: control released by %s (%s)\n", c.peer.c_str(), why);
    }

    // ---- what a viewer sends -----------------------------------------------

    Void onControlFrame(Client& c, const bibowire::Control& m);

    Void onCommand(Client& c, const bibowire::Body& body, UInt8 ver)
    {
        bibowire::Command cmd;
        if(!bibowire::readCommand(body, ver, &cmd))
        {
            return;
        }
        bibowire::CmdAck ack;
        ack.cmdId = cmd.cmdId;
        ack.verb = cmd.verb;

        if(cmd.sessionId != c.sessionId)
        {
            ack.result = 1;
            ack.text = "that COMMAND carries another session's id";
        }
        else if(cmd.verb == bibowire::Verb::VERB_ESTOP)
        {
            // Emergency stop has two paths on purpose, and either LATCHES. This
            // one rides TCP and is acknowledged; the CONTROL button bit rides
            // UDP and needs no round trip.
            estopLatched = true;
            bumpEpoch();
            ack.result = 0;
            ack.text = "estop latched - CLEAR_ESTOP while disarmed, then ARM again";
        }
        else if(cmd.verb == bibowire::Verb::VERB_CLEAR_ESTOP)
        {
            estopLatched = false;
            bumpEpoch();
            ack.result = 0;
            ack.text = "estop cleared - the car is disarmed and must be armed deliberately";
        }
        else
        {
            // The verbs that reach the CAR are refused rather than answered
            // with an OK nothing acted on. The pilot does not drive from
            // bibowire in this build, and a board that sent an arm into a
            // closed port and reported success is the failure this whole
            // protocol is shaped against.
            ack.result = 3;
            ack.text = "the pilot does not take bibowire commands yet - it drives from reactive::step";
        }
        ack.armEpoch = static_cast<UInt8>(armEpoch);
        emit(c, bibowire::Type::TYPE_CMDACK, [&ack](UInt8* out, Size cap) {
            return bibowire::writeCmdAck(ack, out, cap);
        });
    }

    Void onFrame(Client& c, const bibowire::Frame& f, Vec<Client>& clients)
    {
        note(Dir::DIR_IN, f.head, f.body.len, NOTE_OK);

        // Nothing but HELLO is read from a socket that has not been welcomed.
        if(c.stage == Stage::STAGE_WAIT_HELLO && f.head.type != bibowire::Type::TYPE_HELLO)
        {
            refuse(
                c,
                bibowire::Reason::REASON_REFUSED,
                "HELLO must be the first frame",
                "no HELLO"
            );
            return;
        }

        switch(f.head.type)
        {
        case bibowire::Type::TYPE_HELLO:
            onHello(c, f.body, f.head.ver, clients);
            break;
        case bibowire::Type::TYPE_LEAVE:
        {
            bibowire::Leave m;
            if(bibowire::readLeave(f.body, f.head.ver, &m) && m.sessionId == c.sessionId)
            {
                releaseSlot(c, "LEAVE");
                c.dropWhy = "LEAVE";
            }
            break;
        }
        case bibowire::Type::TYPE_PING:
        {
            bibowire::Ping m;
            if(!bibowire::readPing(f.body, f.head.ver, &m))
            {
                break;
            }
            // THE TOKEN IS ECHOED VERBATIM - `m.token` is not touched. It is
            // the viewer's only correlator, and a PONG that regenerated it
            // would silently destroy the round-trip time the viewer computes
            // its clock offset and its staleness floor from.
            //
            // senderMonoUs is REPLACED, and deliberately: the field is the
            // SENDER's clock, the board is this frame's sender, and section 7's
            // clock-offset rule needs the board's own timestamp to work
            // against. Only `token` is marked "echoed verbatim" in section 5,
            // and it is the field that carries the correlation.
            m.senderMonoUs = monoUs();
            // Answered HERE, in the same pass the PING arrived, rather than
            // from the periodic work below: a reply delayed behind a queue is a
            // round-trip time that measures this board's scheduler instead of
            // the link.
            emit(c, bibowire::Type::TYPE_PONG, [&m](UInt8* out, Size cap) {
                return bibowire::writePing(m, out, cap);
            });
            break;
        }
        case bibowire::Type::TYPE_PONG:
        {
            bibowire::Ping m;
            if(bibowire::readPing(f.body, f.head.ver, &m) && m.token == c.pingToken)
            {
                c.pingOut = false;
            }
            break;
        }
        case bibowire::Type::TYPE_CONTROL:
        {
            // The board accepts CONTROL on TCP ALWAYS, with identical rules,
            // identical deadman and identical session and seq checks. It is the
            // same frame on a different socket, so there is nothing to
            // negotiate and no second code path.
            bibowire::Control m;
            if(bibowire::readControl(f.body, f.head.ver, &m))
            {
                c.tcpControl = true;
                onControlFrame(c, m);
            }
            break;
        }
        case bibowire::Type::TYPE_COMMAND:
            onCommand(c, f.body, f.head.ver);
            break;
        case bibowire::Type::TYPE_SUBSCRIBE:
        {
            bibowire::Subscribe m;
            if(bibowire::readSubscribe(f.body, f.head.ver, &m) && m.sessionId == c.sessionId)
            {
                c.typeMask = m.typeMask;
                c.scanDivisor = m.scanDivisor == 0u ? 1u : m.scanDivisor;
            }
            break;
        }
        case bibowire::Type::TYPE_DESCRIBE:
        {
            bibowire::Describe m;
            if(!bibowire::readDescribe(f.body, f.head.ver, &m))
            {
                break;
            }
            bibowire::Schema s;
            s.text = m.type == 0u
                ? bibowire::schemaAll()
                : bibowire::schemaLine(static_cast<bibowire::Type>(m.type));
            emit(c, bibowire::Type::TYPE_SCHEMA, [&s](UInt8* out, Size cap) {
                return bibowire::writeSchema(s, out, cap);
            });
            break;
        }
        case bibowire::Type::TYPE_BYE:
            c.dropWhy = "the viewer said BYE";
            releaseSlot(c, "BYE");
            break;
        default:
            // An unknown type was skipped by exactly `len` before it reached
            // here. That is what the length prefix is FOR, and it is the whole
            // extensibility story: an older board keeps serving a newer viewer.
            break;
        }
    }

    // The one case that answers in words. The moment a person is most confused
    // by a binary port is the moment they connect to it by hand.
    Void wrongService(Client& c)
    {
        const Str line =
            "ERR bibowire v1 binary on 8020; scanwire text is on 8011; "
            "run `biboctl watch` on the board to read this port\n";
        static_cast<Void>(::send(c.fd, line.data(), line.size(), MSG_NOSIGNAL | MSG_DONTWAIT));
        refuse(c, bibowire::Reason::REASON_REFUSED, "not bibowire", "not speaking bibowire");
    }

    Void consume(Client& c, Vec<Client>& clients)
    {
        while(c.dropWhy.empty())
        {
            if(c.stage == Stage::STAGE_WAIT_HELLO && c.inLen >= 2u)
            {
                if(!(c.in[0] == bibowire::MAGIC_LO && c.in[1] == bibowire::MAGIC_HI))
                {
                    wrongService(c);
                    return;
                }
            }

            // THE SINGLE MOST IMPORTANT BOUND IN THE DESIGN, and it is checked
            // on the HEADER rather than after a frame arrives: a claim of
            // 200000 bytes would never fit the 1024-byte ring, so take() would
            // answer NEED_MORE forever while the peer said nothing more.
            // Nothing is allocated for the claim, here or anywhere.
            if(c.inLen >= bibowire::HEAD_BYTES && c.in[0] == bibowire::MAGIC_LO
               && c.in[1] == bibowire::MAGIC_HI)
            {
                const UInt32 plen = bibowire::rd32(c.in.data() + 8u);
                if(plen > bibowire::MAX_INBOUND_PAYLOAD)
                {
                    bibowire::Head h;
                    h.type = static_cast<bibowire::Type>(c.in[2]);
                    note(Dir::DIR_IN, h, plen, NOTE_REFUSED);
                    Array<Char, 128> text{};
                    std::snprintf(
                        text.data(),
                        text.size(),
                        "a frame claiming %u payload bytes; this board accepts %u",
                        static_cast<unsigned>(plen),
                        static_cast<unsigned>(bibowire::MAX_INBOUND_PAYLOAD)
                    );
                    refuse(
                        c,
                        bibowire::Reason::REASON_TOO_BIG,
                        text.data(),
                        "oversized inbound frame"
                    );
                    return;
                }
            }

            bibowire::Frame f;
            Size used = 0;
            const bibowire::Take t = bibowire::take(c.in.data(), c.inLen, &f, &used);
            if(t == bibowire::Take::TAKE_NEED_MORE)
            {
                // A viewer that fills the ring without ever completing a frame
                // is not speaking this protocol, whatever its first two bytes
                // said.
                if(c.inLen >= c.in.size())
                {
                    refuse(
                        c,
                        bibowire::Reason::REASON_BAD_CRC,
                        "1024 bytes and no whole frame",
                        "no frame in a full ring"
                    );
                }
                return;
            }
            if(t == bibowire::Take::TAKE_TOO_BIG)
            {
                refuse(
                    c,
                    bibowire::Reason::REASON_TOO_BIG,
                    "a payload over the protocol maximum",
                    "oversized inbound frame"
                );
                return;
            }
            if(t == bibowire::Take::TAKE_BAD_FLAG)
            {
                refuse(
                    c,
                    bibowire::Reason::REASON_BAD_FLAG,
                    "FLAG_MORE must be 0 in v1",
                    "framing flag"
                );
                return;
            }

            if(used > 0u && used <= c.inLen)
            {
                if(t == bibowire::Take::TAKE_RESYNC)
                {
                    // Junk on a checksummed stream that is never counted is a
                    // fault nobody discovers.
                    bibowire::Head h;
                    note(Dir::DIR_IN, h, used, NOTE_JUNK);
                    LockGuard<Mutex> lock(sh.tallyM);
                    sh.tally.resyncBytes += used;
                }
                std::memmove(c.in.data(), c.in.data() + used, c.inLen - used);
                c.inLen -= used;
            }
            if(t == bibowire::Take::TAKE_FRAME)
            {
                // The body points INTO c.in, and onFrame reads it before
                // anything refills the ring.
                onFrame(c, f, clients);
            }
        }
    }

    Void readFrom(Client& c, Vec<Client>& clients)
    {
        const Size room = c.in.size() - c.inLen;
        if(room == 0u)
        {
            refuse(
                c,
                bibowire::Reason::REASON_BAD_CRC,
                "1024 bytes and no whole frame",
                "no frame in a full ring"
            );
            return;
        }
        const ISize n = ::recv(c.fd, c.in.data() + c.inLen, room, MSG_DONTWAIT);
        if(n == 0)
        {
            // A FIN from the holder is positive evidence the driver is gone.
            releaseSlot(c, "the socket closed");
            c.dropWhy = "left";
            return;
        }
        if(n < 0)
        {
            if(errno != EAGAIN && errno != EWOULDBLOCK)
            {
                releaseSlot(c, "the socket failed");
                c.dropWhy = std::strerror(errno);
                c.abnormal = true;
            }
            return;
        }
        c.inLen += static_cast<Size>(n);
        consume(c, clients);
    }

    // ---- CONTROL, from either transport ------------------------------------

    Void onControlFrame(Client& c, const bibowire::Control& m)
    {
        {
            LockGuard<Mutex> lock(sh.tallyM);
            ++sh.tally.rxControl;
        }
        ++c.datagrams;

        bibowire::control::Gate g;
        g.sessionId = c.sessionId;
        g.highestSeq = appliedSeq;
        g.haveHolder = haveHolder;
        g.fromHolder = c.holder && c.sessionId == holderSession;
        g.armEpoch = static_cast<UInt8>(armEpoch);
        g.pilotMode = m.assumedMode;

        const bibowire::control::Outcome o = bibowire::control::apply(g, m);
        if(o.verdict != bibowire::control::Verdict::VERDICT_APPLIED)
        {
            // An observer's datagrams, a stale session's datagrams and a second
            // viewer's datagrams are counted and DISCARDED, and above all they
            // do NOT feed the timer.
            LockGuard<Mutex> lock(sh.tallyM);
            ++sh.tally.rxControlStale;
            return;
        }

        // The ESTOP bit rides the 20 Hz stream so it lands within one 50 ms
        // window and needs no round trip. It latches, like its TCP twin.
        if((m.buttons & bibowire::BUTTON_ESTOP) != 0u && !estopLatched)
        {
            estopLatched = true;
            bumpEpoch();
        }

        appliedSeq = o.highestSeq;
        lastControlAt = monoNow();
        everControl = true;
        holderGone = false;

        // The seqlock: the slot is written FIRST and the seq stored with
        // release, so the tick either sees the old command whole or the new one
        // whole and never a mixture of the two.
        const UInt32 slot = o.highestSeq & 1u;
        sh.ctlSlot[slot] = m;
        sh.ctlSeq.store(o.highestSeq, std::memory_order_release);
    }

    Void serveUdp(Vec<Client>& clients)
    {
        // DRAINED TO EMPTY every pass, keeping only the newest seq. There is no
        // backlog of stale steering to apply when a stall clears, because there
        // is no queue.
        for(;;)
        {
            Array<UInt8, bibowire::MAX_DATAGRAM> buf{};
            sockaddr_in from{};
            socklen_t len = sizeof(from);
            const ISize n = ::recvfrom(
                udpFd,
                buf.data(),
                buf.size(),
                MSG_DONTWAIT,
                reinterpret_cast<sockaddr*>(&from),
                &len
            );
            if(n <= 0)
            {
                return;
            }

            bibowire::Frame f;
            Size used = 0;
            const bibowire::Take t = bibowire::take(buf.data(), static_cast<Size>(n), &f, &used);
            if(t != bibowire::Take::TAKE_FRAME || f.head.type != bibowire::Type::TYPE_CONTROL)
            {
                LockGuard<Mutex> lock(sh.tallyM);
                ++sh.tally.rxControlStale;
                continue;
            }
            bibowire::Control m;
            if(!bibowire::readControl(f.body, f.head.ver, &m))
            {
                LockGuard<Mutex> lock(sh.tallyM);
                ++sh.tally.rxControlStale;
                continue;
            }

            // The session id is what makes a datagram from BEFORE a reconnect
            // harmless: after a hotspot blip the viewer comes back with a new
            // one, and anything still in flight from the old session is counted
            // and thrown away rather than driving the car with a second-old
            // stick position.
            Client* owner = nullptr;
            for(Client& c : clients)
            {
                if(c.stage == Stage::STAGE_LIVE && c.sessionId == m.sessionId)
                {
                    owner = &c;
                    break;
                }
            }
            if(owner == nullptr)
            {
                LockGuard<Mutex> lock(sh.tallyM);
                ++sh.tally.rxControl;
                ++sh.tally.rxControlStale;
                continue;
            }
            // Where CTLSTATE goes back, learned from the datagram that arrived
            // rather than trusted from HELLO alone: a viewer behind a NAT is
            // reachable at the port its packets came from.
            if(owner->udpPort == 0u)
            {
                owner->udpPort = ntohs(from.sin_port);
            }
            onControlFrame(*owner, m);
        }
    }

    Void sendCtlState(Client& c)
    {
        const bibowire::deadman::Output d = deadmanNow();
        Applied a;
        {
            LockGuard<Mutex> lock(sh.appliedM);
            a = sh.applied;
        }

        bibowire::CtlState s;
        s.tMonoUs = monoUs();
        s.ackSeq = c.holder ? appliedSeq : 0u;
        s.controlAgeMs = c.holder && everControl
            ? static_cast<UInt32>(elapsedMs(lastControlAt))
            : bibowire::CONTROL_AGE_NEVER;
        s.steerNowMilli = a.steerNowMilli;
        s.throttleMilli = a.throttleMilli;
        s.escUs = a.escUs;
        s.neutralInMs = static_cast<UInt16>(d.neutralInMs);
        s.disarmInMs = static_cast<UInt16>(d.disarmInMs);
        s.armed = a.armed;
        s.armEpoch = static_cast<UInt8>(armEpoch);
        s.deadman = deadmanByte();
        s.refuse = d.refuse;
        s.holder = c.holder ? 1u : (haveHolder ? 2u : 0u);

        // ---- the three ages, MEASURED ------------------------------------
        //
        // These are what catch the second lie of section 7: the link is
        // perfect and the data behind it is dead. They are the board measuring
        // itself rather than the viewer inferring, and the one value none of
        // them may take is 0-when-unknown, because 0 reads as PERFECTLY FRESH.
        //
        // With no revolution yet the age is how long this feed has been up:
        // true, large, and growing, so the viewer's own rule renders "no scan
        // for 3.2 s" on an empty background instead of a confident wall that is
        // no longer there. scanAgeMs has no ABSENT sentinel in the wire format,
        // which is why it is an honest elapsed time rather than an invented one.
        s.scanAgeMs = haveScan
            ? static_cast<UInt32>(elapsedMs(lastScanAt))
            : static_cast<UInt32>(elapsedMs(startedAt));

        // The pilot measures the Pico's silence every tick and it rides BOARD,
        // so it is reused here rather than measured a second time - two
        // measurements of one fact are two things that can disagree. Falling
        // back to the ABSENT sentinel and never to 0: "0 ms silent" is a board
        // that just spoke, which is the opposite of what is known about one
        // nobody has heard from.
        s.picoSilentMs = haveBoard ? lastBoard.picoSilentMs : a.picoSilentMs;
        s.pilotMode = haveBoard ? lastBoard.pilotMode : a.pilotMode;
        s.lastCmdId = a.lastCmdId;

        // The reverse-path probe's verdict, on the wire twenty times a second
        // rather than in a log nobody in a field can read.
        if(c.wantedControl && c.holder && c.datagrams < bibowire::REVERSE_PROBE_MIN
           && elapsedMs(c.welcomedAt) > static_cast<Float64>(bibowire::REVERSE_PROBE_MS))
        {
            s.refuse = bibowire::Refuse::REFUSE_NO_UDP;
        }

        const Size bodyLen = bibowire::writeCtlState(s, bodyAt(), BODY_CAP);
        if(bodyLen == 0u)
        {
            return;
        }
        const Size total = framed(bibowire::Type::TYPE_CTLSTATE, bodyLen, c.txSeq, outFlags());
        if(total == 0u)
        {
            return;
        }

        if(c.udpPort != 0u)
        {
            sockaddr_in to{};
            to.sin_family = AF_INET;
            to.sin_port = htons(c.udpPort);
            if(::inet_pton(AF_INET, c.peerIp.c_str(), &to.sin_addr) == 1)
            {
                static_cast<Void>(::sendto(
                    udpFd,
                    scratch.data(),
                    total,
                    MSG_DONTWAIT,
                    reinterpret_cast<const sockaddr*>(&to),
                    sizeof(to)
                ));
            }
        }

        // Each datagram is its own frame in this direction's sequence, so the
        // counter advances rather than repeating - a header ring in which every
        // CTLSTATE carries the same seq is a ring that cannot order itself.
        ++c.txSeq;

        // Mirrored onto TCP while the viewer is falling back to TCP control.
        //
        // Section 12.6 leaves this open, and it is answered here: the mirror is
        // queued as LIVE rather than VITAL. CTLSTATE at 20 Hz that can never be
        // dropped would turn a two-second stall into a closed connection - the
        // exact moment the operator most needs the link - and the newest
        // CTLSTATE is the only one worth having anyway, which is what LIVE
        // means.
        if(c.tcpControl)
        {
            const Size mirrored = framed(bibowire::Type::TYPE_CTLSTATE, bodyLen, c.txSeq, outFlags());
            if(mirrored != 0u)
            {
                const Size first = c.sentOfHead > 0u ? 1u : 0u;
                for(Size i = c.out.size(); i > first; --i)
                {
                    if(c.out[i - 1u].type == bibowire::Type::TYPE_CTLSTATE)
                    {
                        c.outBytes -= c.out[i - 1u].bytes.size();
                        c.out.erase(c.out.begin() + static_cast<ISize>(i - 1u));
                        countFrame(true);
                    }
                }
                Queued q;
                q.bytes.assign(scratch.data(), scratch.data() + mirrored);
                q.cls = bibowire::Class::CLASS_LIVE;
                q.type = bibowire::Type::TYPE_CTLSTATE;
                q.at = monoNow();
                if(c.outBytes + mirrored <= RING_BYTES && c.out.size() + 1u <= RING_FRAMES)
                {
                    c.outBytes += mirrored;
                    c.out.push_back(std::move(q));
                    ++c.txSeq;
                }
            }
        }
    }

    // ---- what the owner published ------------------------------------------

    Void deliver(Vec<Client>& clients, const Item& item)
    {
        switch(item.what)
        {
        case What::WHAT_SCAN:
            // A revolution REACHED THE BOARD, which is what scanAgeMs is the
            // age of - so it is stamped before the staleness rule below can
            // discard the frame. A dropped revolution is still a revolution
            // the sensor produced, and saying otherwise would make a working
            // lidar look dead every time a viewer fell behind.
            lastScanAt = item.at;
            haveScan = true;
            // A revolution more than 200 ms old at SEND time is dropped before
            // it is ever queued. This is measured HERE, on the thread that
            // would do the sending, because the age that matters is the one at
            // the moment the bytes would go out.
            if(elapsedMs(item.at) > static_cast<Float64>(LIVE_STALE_MS))
            {
                for(Client& c : clients)
                {
                    if(wants(c, bibowire::Type::TYPE_SCAN))
                    {
                        ++c.droppedLive;
                    }
                }
                countFrame(true);
                return;
            }
            for(Client& c : clients)
            {
                if(!wants(c, bibowire::Type::TYPE_SCAN))
                {
                    continue;
                }
                ++c.revSeen;
                if(c.scanDivisor > 1u && (c.revSeen % c.scanDivisor) != 0u)
                {
                    continue;
                }
                // droppedSinceLast is per CLIENT, and it is zeroed only once
                // the frame carrying it has been built - a count that is
                // cleared before it is reported is a count nobody ever sees.
                bibowire::Scan s = item.scan;
                s.droppedSinceLast = c.droppedLive;
                s.scanDivisor = c.scanDivisor;
                emit(c, bibowire::Type::TYPE_SCAN, [&s](UInt8* out, Size cap) {
                    return bibowire::writeScan(s, out, cap);
                });
                c.droppedLive = 0;
            }
            break;
        case What::WHAT_DECIDE:
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_DECIDE))
                {
                    emit(c, bibowire::Type::TYPE_DECIDE, [&item](UInt8* out, Size cap) {
                        return bibowire::writeDecide(item.decide, out, cap);
                    });
                }
            }
            break;
        case What::WHAT_BOARD:
            lastBoard = item.board;
            haveBoard = true;
            // 5 Hz on the wire from a pilot that fills the struct every tick.
            // The RATE is this module's business; the CONTENT is one struct the
            // pilot filled once, which is what keeps the phone and the viewer
            // from disagreeing about what the car thinks.
            if(boardSent && elapsedMs(lastBoardAt) < static_cast<Float64>(BOARD_EVERY_MS))
            {
                return;
            }
            lastBoardAt = monoNow();
            boardSent = true;
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_BOARD))
                {
                    const bibowire::BoardState b = boardFor(c);
                    emit(c, bibowire::Type::TYPE_BOARD, [&b](UInt8* out, Size cap) {
                        return bibowire::writeBoard(b, out, cap);
                    });
                }
            }
            break;
        case What::WHAT_LIDAR:
            lastLidar = item.lidar;
            haveLidar = true;
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_LIDAR_INFO))
                {
                    emit(c, bibowire::Type::TYPE_LIDAR_INFO, [](UInt8* out, Size cap) {
                        return bibowire::writeLidarInfo(lastLidar, out, cap);
                    });
                }
            }
            break;
        case What::WHAT_EVENT:
        {
            // Rate-limited to 10/s, with the suppressed count carried in the
            // next one - so the viewer knows events were dropped rather than
            // believing it saw them all.
            if(eventWindowOpen && elapsedMs(lastEventAt) < static_cast<Float64>(EVENT_WINDOW_MS))
            {
                ++eventsSuppressed;
                return;
            }
            lastEventAt = monoNow();
            eventWindowOpen = true;
            bibowire::Event e = item.event;
            e.droppedSince = eventsSuppressed;
            eventsSuppressed = 0;
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_EVENT))
                {
                    emit(c, bibowire::Type::TYPE_EVENT, [&e](UInt8* out, Size cap) {
                        return bibowire::writeEvent(e, out, cap);
                    });
                }
            }
            break;
        }
        }
    }

    // ---- the clients -------------------------------------------------------

    Void reap(Vec<Client>& clients)
    {
        Size kept = 0;
        for(Size i = 0; i < clients.size(); ++i)
        {
            Client& c = clients[i];
            if(c.dropWhy.empty())
            {
                if(kept != i)
                {
                    clients[kept] = std::move(c);
                }
                ++kept;
                continue;
            }
            releaseSlot(c, "the client went");
            if(c.abnormal)
            {
                dumpNotes(c);
            }
            std::printf("viewfeed: %s dropped: %s\n", c.peer.c_str(), c.dropWhy.c_str());
            ::close(c.fd);
        }
        clients.resize(kept);
    }

    Void acceptAll(Vec<Client>& clients)
    {
        for(;;)
        {
            sockaddr_in peer{};
            socklen_t len = sizeof(peer);
            const Int32 fd = ::accept4(listenFd, reinterpret_cast<sockaddr*>(&peer), &len, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if(fd < 0)
            {
                return;
            }
            // Unconditionally: a 40-byte CMDACK must not sit in Nagle's queue
            // behind the 2540-byte SCAN it follows.
            const Int32 yes = 1;
            static_cast<Void>(::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)));
            // A phone that walks out of range stops ACKing without a FIN, and
            // the default keepalive is two hours.
            static_cast<Void>(::setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes)));
            const Int32 idle = 2;
            const Int32 intvl = 1;
            const Int32 cnt = 3;
            static_cast<Void>(::setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle)));
            static_cast<Void>(::setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl)));
            static_cast<Void>(::setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt)));

            Array<Char, INET_ADDRSTRLEN> ip{};
            static_cast<Void>(::inet_ntop(AF_INET, &peer.sin_addr, ip.data(), ip.size()));
            Array<Char, 64> name{};
            std::snprintf(
                name.data(),
                name.size(),
                "%s:%u",
                ip.data(),
                static_cast<unsigned>(ntohs(peer.sin_port))
            );

            Client c;
            c.fd = fd;
            c.peer = name.data();
            c.peerIp = ip.data();
            c.lastPingAt = monoNow();
            c.lastCtlAt = monoNow();
            std::printf("viewfeed: connection from %s\n", c.peer.c_str());
            clients.push_back(std::move(c));
        }
    }

    // PING every second, and BYE(TIMEOUT) when no PONG comes back inside four.
    // The car stopped 2700 ms before any of that mattered; this is about not
    // leaving a dead socket believed.
    Void keepLive(Client& c)
    {
        if(c.stage != Stage::STAGE_LIVE)
        {
            return;
        }
        if(c.pingOut && elapsedMs(c.pingSentAt) > static_cast<Float64>(PONG_WAIT_MS))
        {
            refuse(c, bibowire::Reason::REASON_TIMEOUT, "no PONG within 4000 ms", "no PONG");
            return;
        }
        if(elapsedMs(c.lastPingAt) < static_cast<Float64>(PING_EVERY_MS))
        {
            return;
        }
        c.lastPingAt = monoNow();
        if(c.pingOut)
        {
            return;
        }
        bibowire::Ping m;
        m.token = monoUs() ^ (static_cast<UInt64>(c.sessionId) << 32u);
        m.senderMonoUs = monoUs();
        c.pingToken = m.token;
        c.pingOut = true;
        c.pingSentAt = monoNow();
        emit(c, bibowire::Type::TYPE_PING, [&m](UInt8* out, Size cap) {
            return bibowire::writePing(m, out, cap);
        });
    }

    // The reverse path, MEASURED rather than assumed because the other
    // direction is fine. This is the nastiest failure the two-transport shape
    // creates, and it is the one a viewer cannot diagnose on its own.
    Void probeReversePath(Client& c)
    {
        if(!c.wantedControl || !c.holder || c.probeSaid)
        {
            return;
        }
        if(elapsedMs(c.welcomedAt) < static_cast<Float64>(bibowire::REVERSE_PROBE_MS))
        {
            return;
        }
        c.probeSaid = true;
        if(c.datagrams >= bibowire::REVERSE_PROBE_MIN)
        {
            return;
        }
        Array<Char, 200> text{};
        std::snprintf(
            text.data(),
            text.size(),
            "no control datagrams on UDP %u from %s - telemetry is up and the car will not move; "
            "the viewer is falling back to TCP control",
            static_cast<unsigned>(boundPort),
            c.peerIp.c_str()
        );
        bibowire::Event e;
        e.tMonoUs = monoUs();
        e.severity = bibowire::Severity::SEVERITY_WARN;
        e.text = text.data();
        emit(c, bibowire::Type::TYPE_EVENT, [&e](UInt8* out, Size cap) {
            return bibowire::writeEvent(e, out, cap);
        });
        std::printf("viewfeed: %s\n", text.data());
    }

    // ---- the thread --------------------------------------------------------

    Void loop()
    {
        Vec<Client> clients;
        Size announced = 0;

        for(;;)
        {
            Vec<pollfd> fds;
            fds.push_back(pollfd{ listenFd, POLLIN, 0 });
            fds.push_back(pollfd{ wake[0], POLLIN, 0 });
            fds.push_back(pollfd{ udpFd, POLLIN, 0 });
            for(Client& c : clients)
            {
                c.at = fds.size();
                const Int16 want = static_cast<Int16>(POLLIN | (c.out.empty() ? 0 : POLLOUT));
                fds.push_back(pollfd{ c.fd, want, 0 });
            }

            if(::poll(fds.data(), fds.size(), POLL_MS) < 0 && errno != EINTR)
            {
                std::printf("viewfeed: poll failed: %s\n", std::strerror(errno));
                break;
            }

            const Size before = clients.size();
            if((fds[0].revents & POLLIN) != 0)
            {
                acceptAll(clients);
            }
            if((fds[1].revents & POLLIN) != 0)
            {
                Array<Char, 64> sink{};
                while(::read(wake[0], sink.data(), sink.size()) > 0)
                {
                }
            }
            if((fds[2].revents & POLLIN) != 0)
            {
                serveUdp(clients);
            }

            Deque<Item> items;
            Bool quit = false;
            {
                LockGuard<Mutex> lock(sh.m);
                items.swap(sh.items);
                quit = sh.quit;
            }
            if(quit)
            {
                break;
            }

            // Client sockets first, in the order the pollfds were built, so a
            // HELLO that arrived this pass is welcomed before the revolution
            // published this pass is handed out. Only the clients that existed
            // before accept() have an entry.
            for(Size i = 0; i < before && i < clients.size(); ++i)
            {
                Client& c = clients[i];
                if(!c.dropWhy.empty())
                {
                    continue;
                }
                const Int16 ev = fds[c.at].revents;
                if((ev & (POLLERR | POLLHUP | POLLNVAL)) != 0 && (ev & POLLIN) == 0)
                {
                    releaseSlot(c, "the socket hung up");
                    c.dropWhy = "hung up";
                    continue;
                }
                if((ev & POLLIN) != 0)
                {
                    readFrom(c, clients);
                }
            }

            for(const Item& item : items)
            {
                deliver(clients, item);
            }

            // The control slot, released by silence rather than by a close.
            if(haveHolder && everControl
               && elapsedMs(lastControlAt) > static_cast<Float64>(bibowire::CONTROL_SLOT_MS))
            {
                for(Client& c : clients)
                {
                    releaseSlot(c, "1000 ms of control silence");
                }
            }

            for(Client& c : clients)
            {
                if(!c.dropWhy.empty())
                {
                    continue;
                }
                keepLive(c);
                probeReversePath(c);
                if(c.stage == Stage::STAGE_LIVE
                   && elapsedMs(c.lastCtlAt) >= static_cast<Float64>(CTLSTATE_EVERY_MS))
                {
                    c.lastCtlAt = monoNow();
                    sendCtlState(c);
                }
                if(!c.out.empty())
                {
                    static_cast<Void>(flush(c));
                }
                if(c.dropWhy.empty())
                {
                    checkBehind(c);
                }
            }

            reap(clients);

            sh.count.store(liveClients(clients));
            if(clients.size() != announced)
            {
                announced = clients.size();
                if(policy.onClients)
                {
                    policy.onClients(sh.count.load());
                }
            }
        }

        for(Client& c : clients)
        {
            sayBye(c, bibowire::Reason::REASON_SHUTDOWN, "the pilot is stopping");
            ::close(c.fd);
        }
        sh.count.store(0);
    }

    [[nodiscard]] Int32 listenOn(UInt16 port)
    {
        const Int32 fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if(fd < 0)
        {
            std::printf("viewfeed: cannot create a socket: %s\n", std::strerror(errno));
            return -1;
        }
        const Int32 yes = 1;
        static_cast<Void>(::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);
        if(::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            std::printf(
                "viewfeed: cannot bind 0.0.0.0:%u: %s\n",
                static_cast<unsigned>(port),
                std::strerror(errno)
            );
            ::close(fd);
            return -1;
        }
        if(::listen(fd, 8) < 0)
        {
            std::printf("viewfeed: cannot listen: %s\n", std::strerror(errno));
            ::close(fd);
            return -1;
        }
        return fd;
    }

    [[nodiscard]] Int32 bindUdp(UInt16 port)
    {
        const Int32 fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if(fd < 0)
        {
            std::printf("viewfeed: cannot create a datagram socket: %s\n", std::strerror(errno));
            return -1;
        }
        const Int32 yes = 1;
        static_cast<Void>(::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)));
        // Deliberately small. A deep queue of control datagrams is a queue of
        // STALE STEERING COMMANDS, and the board wants the newest, not the most.
        const Int32 rcv = UDP_RCVBUF;
        static_cast<Void>(::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv)));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);
        if(::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            std::printf(
                "viewfeed: cannot bind udp 0.0.0.0:%u: %s\n",
                static_cast<unsigned>(port),
                std::strerror(errno)
            );
            ::close(fd);
            return -1;
        }
        return fd;
    }

  }

  Bool start(UInt16 port, const Policy& p)
  {
      if(running)
      {
          std::printf("viewfeed: already running on port %u\n", static_cast<unsigned>(boundPort));
          return false;
      }
      if(::pipe2(wake.data(), O_NONBLOCK | O_CLOEXEC) < 0)
      {
          std::printf("viewfeed: cannot create the wake pipe: %s\n", std::strerror(errno));
          return false;
      }
      startedAt = monoNow();
      listenFd = listenOn(port);
      if(listenFd < 0)
      {
          ::close(wake[0]);
          ::close(wake[1]);
          wake = { -1, -1 };
          return false;
      }

      sockaddr_in bound{};
      socklen_t len = sizeof(bound);
      if(::getsockname(listenFd, reinterpret_cast<sockaddr*>(&bound), &len) == 0)
      {
          boundPort = ntohs(bound.sin_port);
      }
      else
      {
          boundPort = port;
      }

      // The SAME NUMBER for both, always. A UDP socket that quietly landed
      // somewhere else would make the control path's address a thing a viewer
      // has to discover rather than a thing it knows.
      udpFd = bindUdp(boundPort);
      if(udpFd < 0)
      {
          ::close(listenFd);
          ::close(wake[0]);
          ::close(wake[1]);
          listenFd = -1;
          wake = { -1, -1 };
          boundPort = 0;
          return false;
      }

      policy = p;
      {
          LockGuard<Mutex> lock(sh.m);
          sh.items.clear();
          sh.quit = false;
          sh.wakeFd = wake[1];
      }
      {
          LockGuard<Mutex> lock(sh.tallyM);
          sh.tally = Counters();
      }
      sh.count.store(0);
      sh.ctlSeq.store(0, std::memory_order_release);
      armEpoch = 0;
      estopLatched = false;
      haveHolder = false;
      holderSession = 0;
      everControl = false;
      appliedSeq = 0;
      holderGone = false;
      haveBoard = false;
      haveLidar = false;
      boardSent = false;
      haveScan = false;
      eventWindowOpen = false;
      eventsSuppressed = 0;
      notesIn = {};
      notesOut = {};
      notesInAt = 0;
      notesOutAt = 0;
      running = true;
      worker = Thread(loop);
      std::printf(
          "viewfeed: bibowire v%u.%u on tcp+udp 0.0.0.0:%u\n",
          static_cast<unsigned>(bibowire::PROTO_MAJOR),
          static_cast<unsigned>(bibowire::PROTO_MINOR),
          static_cast<unsigned>(boundPort)
      );
      return true;
  }

  UInt16 port()
  {
      return running ? boundPort : static_cast<UInt16>(0);
  }

  Size clients()
  {
      return sh.count.load();
  }

  Void publishScan(bibowire::Scan s)
  {
      // Nobody to be live for: not even the lock. This is what keeps an
      // unwatched pilot's tick the price it was before this module existed.
      if(!running || sh.count.load() == 0)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_SCAN;
      item.scan = std::move(s);
      item.at = monoNow();
      post(std::move(item));
  }

  Void publishDecide(const bibowire::Decide& d)
  {
      if(!running || sh.count.load() == 0)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_DECIDE;
      item.decide = d;
      item.at = monoNow();
      post(std::move(item));
  }

  Void publishBoard(const bibowire::BoardState& b)
  {
      // NOT gated on a client being connected, unlike the rest: the newest
      // BOARD is what the NEXT viewer is owed before it is shown a point, and a
      // board state dropped at the door is a viewer that connects into silence
      // until the pilot's next second comes round.
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_BOARD;
      item.board = b;
      item.at = monoNow();
      post(std::move(item));
  }

  Void publishLidarInfo(const bibowire::LidarInfo& i)
  {
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_LIDAR;
      item.lidar = i;
      item.at = monoNow();
      post(std::move(item));
  }

  Void publishEvent(bibowire::Severity severity, UInt8 code, const Str& text)
  {
      if(!running || sh.count.load() == 0)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_EVENT;
      item.event.tMonoUs = monoUs();
      item.event.severity = severity;
      item.event.code = code;
      item.event.text = text.size() > bibowire::MAX_EVENT_TEXT
          ? text.substr(0, bibowire::MAX_EVENT_TEXT)
          : text;
      item.at = monoNow();
      post(std::move(item));
  }

  Bool control(bibowire::Control* out)
  {
      if(out == nullptr || !running)
      {
          return false;
      }
      for(Int32 tries = 0; tries < 8; ++tries)
      {
          const UInt32 seq = sh.ctlSeq.load(std::memory_order_acquire);
          if(seq == 0u)
          {
              return false;
          }
          const bibowire::Control got = sh.ctlSlot[seq & 1u];
          if(sh.ctlSeq.load(std::memory_order_acquire) == seq)
          {
              *out = got;
              return true;
          }
      }
      // Eight consecutive writes during one read is a control stream running
      // far faster than 20 Hz, which is not a thing this protocol produces.
      // Reporting nothing is the safe answer: no command beats half of one.
      return false;
  }

  Void applied(const Applied& a)
  {
      if(!running)
      {
          return;
      }
      LockGuard<Mutex> lock(sh.appliedM);
      sh.applied = a;
  }

  Counters counters()
  {
      LockGuard<Mutex> lock(sh.tallyM);
      return sh.tally;
  }

  Void stop()
  {
      if(!running)
      {
          return;
      }
      {
          LockGuard<Mutex> lock(sh.m);
          sh.quit = true;
      }
      wakeLoop();
      worker.join();
      running = false;
      ::close(listenFd);
      ::close(udpFd);
      ::close(wake[0]);
      ::close(wake[1]);
      listenFd = -1;
      udpFd = -1;
      wake = { -1, -1 };
      boundPort = 0;
      policy = Policy();
  }

}

#else

// Not a stub that listens and reports an empty room: a module that says it
// cannot do the job here. bibowire's socket half needs Linux sockets, and every
// call behaves as it would with no viewer connected - which is to say, does
// nothing, and says so from start().
namespace viewfeed
{

  Bool start(UInt16 port, const Policy& p)
  {
      static_cast<Void>(p);
      std::printf(
          "viewfeed: no bibowire socket on this platform (port %u asked for)\n",
          static_cast<unsigned>(port)
      );
      return false;
  }

  UInt16 port()
  {
      return 0;
  }

  Size clients()
  {
      return 0;
  }

  Void publishScan(bibowire::Scan s)
  {
      static_cast<Void>(s);
  }

  Void publishDecide(const bibowire::Decide& d)
  {
      static_cast<Void>(d);
  }

  Void publishBoard(const bibowire::BoardState& b)
  {
      static_cast<Void>(b);
  }

  Void publishLidarInfo(const bibowire::LidarInfo& i)
  {
      static_cast<Void>(i);
  }

  Void publishEvent(bibowire::Severity severity, UInt8 code, const Str& text)
  {
      static_cast<Void>(severity);
      static_cast<Void>(code);
      static_cast<Void>(text);
  }

  Bool control(bibowire::Control* out)
  {
      static_cast<Void>(out);
      return false;
  }

  Void applied(const Applied& a)
  {
      static_cast<Void>(a);
  }

  Counters counters()
  {
      return Counters();
  }

  Void stop()
  {
  }

}

#endif
