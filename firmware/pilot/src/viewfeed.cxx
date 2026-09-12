#include "viewfeed.hxx"

#include <cstdio>

#if defined(__linux__)

#include <arpa/inet.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
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

    // The largest frame the SCAN path ever builds: a 1024-point revolution.
    // MAX_PAYLOAD is 256 KiB and is sized for a JPEG, which this file now does
    // send - but a camera frame is NOT built here. It gets its own buffer,
    // allocated when the device opens and released when it closes, so a board
    // nobody has subscribed a camera on still holds exactly these 5 KiB and not
    // a quarter of a megabyte for a frame that is not occurring.
    constexpr Size SCAN_BODY_MAX = 24u + 5u * bibowire::MAX_SCAN_POINTS;
    constexpr Size ENCODE_BYTES = bibowire::FRAME_OVERHEAD + SCAN_BODY_MAX;
    constexpr Size BODY_CAP = ENCODE_BYTES - bibowire::FRAME_OVERHEAD;

    // ---- the camera's numbers ----------------------------------------------

    // JPEG start-of-image. THE ONLY FRAME BOUNDARY MJPEG GIVES: a JPEG's own
    // end marker can occur inside its payload, so a frame is whole only once
    // the NEXT one has begun. Costs exactly one frame of latency and is the
    // reason a viewer is never handed half a picture. status_server.py's
    // CAM_SOI, and its comment, unchanged.
    constexpr Array<UInt8, 3> CAM_SOI = { 0xFFu, 0xD8u, 0xFFu };

    // Bytes with no boundary in them are not a picture. Rather than grow
    // without bound, the reader drops back to hunting for the next marker.
    constexpr Size CAM_MAX_PARTIAL = 4u * 1024u * 1024u;

    // The CAMERA body is a 24-byte fixed header, then the bytes, padded to 4.
    // 32 is that rounded up past its padding: writeCamera REFUSES rather than
    // overruns when the buffer is short, so this only has to have slack, and
    // CAMERA_FIXED itself is private to bibowire.cxx and stays that way.
    constexpr Size CAM_BODY_OVERHEAD = 32;

    // A frame fits WHOLE or it is not sent. MAX_PAYLOAD is 262128 and a 640x480
    // MJPEG frame measured ~45 KB, so this is five times the headroom actually
    // needed - and section 5's FLAG_MORE stays unused, because v1 refuses it.
    constexpr Size CAM_MAX_JPEG = bibowire::MAX_PAYLOAD - CAM_BODY_OVERHEAD;

    // A capture that dies inside a second earns a longer wait, to a ceiling of
    // four. status_server.py's backoff and its reason: retrying twice a second
    // for as long as somebody leaves a subscription open is thousands of spawns
    // an hour against a board whose whole job is elsewhere.
    constexpr Int32 CAM_FAIL_CEILING = 8;
    constexpr Float64 CAM_RETRY_STEP_MS = 500.0;
    constexpr Float64 CAM_RETRY_MAX_MS = 4000.0;

    // How much of v4l2-ctl's stderr is kept to explain a failure with. The
    // sentence that matters - "VIDIOC_REQBUFS returned -1 (Device or resource
    // busy)" - is the first thing it says.
    constexpr Size CAM_DIAG_BYTES = 512;

    // THE DEFAULT RATE, AND WHY IT IS THIS LOW.
    //
    // Measured on this board: 640x480 MJPG comes off /dev/video0 at 25 fps and
    // 1121 KB/s, about 45 KB a frame. docs/bibowire.md section 10 assumes a
    // camera costs ~200 KB/s and says plainly that even that "does not fit
    // alongside the scan on this hotspot"; viewfeed.hxx sizes the whole design
    // against a 220 kbit/s link, which is 27 KB/s - less than ONE frame a
    // second.
    //
    // So no cap makes this fit, and that is not what the cap is for. CLASS_BULK
    // is what decides what the link actually carries: camera frames are
    // discarded before any scan or state frame, so whatever cannot get through
    // is dropped at the ring rather than delaying the car's picture. The cap
    // decides what the board OFFERS. Offering 25 fps would spend capture,
    // encode and CRC on twenty-three frames in twenty-five that the ring throws
    // away unread - real CPU out of the same core the control loop runs on, to
    // produce nothing a viewer ever sees.
    //
    // Two frames a second is ~90 KB/s offered: a small multiple of what a good
    // hotspot moment absorbs, so the picture updates when the link allows and
    // degrades to a slideshow when it does not, with nothing wasted either way.
    // BIBO_CAM_FPS raises it on a link that can take it - a bench cable will -
    // and the number is deliberately NOT tuned for the bench.
    constexpr Float64 CAM_FPS_DEFAULT = 2.0;
    constexpr UInt16 CAM_WIDTH_DEFAULT = 640;
    constexpr UInt16 CAM_HEIGHT_DEFAULT = 480;

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

        // The camera rate THIS viewer asked for, already clamped to
        // bibowire::CAM_FPS_MAX when it was read off the wire. 0 means it did
        // not ask, which is every viewer written before the field existed, and
        // leaves the board's own conservative default standing.
        UInt16 camFps = 0;

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

    // How many accepted tuning requests may wait for the tick. A slider dragged
    // across its range is a burst of discrete COMMANDs and the tick takes only
    // a couple a pass, so there has to be SOME slack - but this filling up does
    // not mean a fast viewer, it means the tick stopped draining, and an
    // unbounded queue would answer a dead control loop by growing until the
    // board ran out of memory.
    constexpr Size TUNE_MAX = 32;

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

        // The tuning handoff: a mutex and a QUEUE, where control above is a
        // seqlock. The difference is the whole reason both exist. CONTROL is a
        // 20 Hz stream whose old values are worthless, so keeping only the
        // newest is correct. A tuning request is a discrete act that has
        // already been acknowledged to the operator by name and value, so
        // keeping only the newest would silently lose one of two sliders moved
        // together and make that acknowledgement a lie.
        //
        // Drop-OLDEST if it ever fills, for the same reason: the newest value
        // is the operator's current intent and the one their slider is showing
        // them. Counted, because a drop here is a promise broken and a silent
        // one would be this repo's recurring failure with a slider on it.
        Mutex tuneM;
        Deque<Tune> tunes;
        UInt64 tuneDropped = 0;
        Bool tuneDropSaid = false;

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

    // One read off the capture's pipe. At file scope rather than on the stack
    // because a 64 KiB local zero-initialised on every read, fifty times a
    // second, is a memset nobody asked for.
    Array<UInt8, 65536> camChunk{};

    // ---- the camera's configuration ----------------------------------------
    //
    // BIBO_CAM_DEV, BIBO_CAM_SIZE and BIBO_CAM_FPS - THE SAME NAMES
    // status_server.py reads, deliberately. The two programs cannot both hold
    // the device, so they are never both capturing; letting them disagree about
    // which device or which size would be a second way to be confused about one
    // camera. Only the RATE differs in spirit, and only because this link is
    // the field hotspot rather than the phone on the same board.
    // BY ID, NOT BY MINOR NUMBER. /dev/videoN is assigned in enumeration order
    // and is NOT stable: this camera fell off the bus mid-stream on 2026-09-10
    // (uvcvideo "Failed to resubmit video URB (-19)", which is ENODEV), came
    // back as USB device 6, and took /dev/video1 - so /dev/video0 ceased to
    // exist and both this module and the phone dashboard reported a dead camera
    // that was sitting right there working. It had re-enumerated THREE times in
    // four minutes, with twelve URB failures, all while streaming: a recurring
    // fact about the hardware rather than a one-off.
    //
    // The by-id path is built from the device's own strings and survives that -
    // the same answer the lidar has always used through /dev/serial/by-id
    // rather than ttyUSB0. Falls back to the old name so a board without the
    // symlink, or a different camera, still works.
    //
    // A FUNCTION, not an initialiser with a branch in it: the first version of
    // this put an `if` directly in CamCfg's member initialiser, which is a type
    // definition where no statement may appear. MSVC never said so, because
    // every line here is inside the __linux__ half it does not compile - so
    // firmware\verify.bat passed and only g++ on the board caught it.
    [[nodiscard]] inline Str cameraDevDefault()
    {
        const Str byId =
            "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0";
        return ::access(byId.c_str(), F_OK) == 0 ? byId : Str("/dev/video0");
    }

    struct CamCfg
    {
        // RESOLVED PER ATTEMPT, not once. startCamera fills `dev` in on every
        // open, because this camera re-enumerates WHILE STREAMING and the by-id
        // symlink exists only while the device does. A path resolved once at
        // boot latches whatever was true then: start the board with the camera
        // absent and the retry loop below would ask for /dev/video0 forever,
        // never looking again when the symlink appeared. The retry was never the
        // broken part - retrying a name that stopped existing is.
        //
        // `devOverride` is BIBO_CAM_DEV when somebody set it and empty
        // otherwise. An override is never re-derived: a device path that moved
        // behind the operator's back would be worse than the bug it replaced.
        Str dev;
        Str devOverride;
        UInt16 width = CAM_WIDTH_DEFAULT;
        UInt16 height = CAM_HEIGHT_DEFAULT;
        Float64 periodMs = 1000.0 / CAM_FPS_DEFAULT;   // 0 means uncapped
    };

    CamCfg camCfg;

    // The capture, owned entirely by this module's thread. Every buffer here is
    // empty and every fd is -1 while nobody subscribes.
    struct Cam
    {
        Int32 pid = -1;
        Int32 outFd = -1;
        Int32 errFd = -1;
        Vec<UInt8> partial;      // bytes since the last start-of-image
        Vec<UInt8> encode;       // one framed CAMERA: header, body, CRC
        Str diag;                // what v4l2-ctl said on stderr, bounded
        UInt32 frameIndex = 0;   // MONOTONIC for the life of the feed
        TimePoint startedAt;
        TimePoint lastSentAt;
        Bool everSent = false;
        Bool everFrame = false;  // this capture delivered at least one picture
        Bool said = false;       // this failure episode has been explained once
        Int32 fails = 0;
        TimePoint failedAt;
        Float64 waitMs = 0.0;
        Bool waiting = false;
    };

    Cam cam;

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

    // The body is expected to be sitting at `buf + HEAD_BYTES` already, so
    // put() has nothing to copy and the framing costs a header and a CRC. That
    // matters most for the camera, where the body is 45 KB and this is called
    // once PER CLIENT - the seq and the flags differ per client, so the header
    // and CRC are rewritten in place over one body rather than the body being
    // re-encoded four times.
    [[nodiscard]] Size framedIn(UInt8* at, Size cap, const bibowire::Head& h, Size len)
    {
        bibowire::Body b;
        b.bytes = at + bibowire::HEAD_BYTES;
        b.len = len;
        return bibowire::put(h, b, at, cap);
    }

    [[nodiscard]] Size framed(bibowire::Type type, Size bodyLen, UInt16 seq, UInt16 flags)
    {
        bibowire::Head h;
        h.type = type;
        h.ver = 1;
        h.flags = flags;
        h.seq = seq;
        return framedIn(scratch.data(), scratch.size(), h, bodyLen);
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

    // Puts a built frame on this client's ring, under the classes of section 7.
    // `bytes` is whichever buffer it was framed in: `scratch` for everything the
    // pilot publishes, the camera's own buffer for a JPEG too big to live there.
    Void enqueueFrom(Client& c, bibowire::Type type, const UInt8* bytes, Size total)
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
        q.bytes.assign(bytes, bytes + total);
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

    // The frame now sitting in `scratch`, which is every frame but a camera's.
    Void enqueue(Client& c, bibowire::Type type, Size total)
    {
        enqueueFrom(c, type, scratch.data(), total);
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
    //
    // CAMERA is in here, and it is the one bit that is ADVERTISED BUT NOT ON.
    // featureMask is "what this board will send", not "what this board is
    // sending" - and a viewer has no other way to discover that asking for bit
    // 16 would get it a picture. Leaving it out would make the camera a thing
    // you have to read this source to find. Whether the DEVICE is there is a
    // different question, answered by an EVENT when a subscription actually
    // tries to open it, because that is the moment it can be answered honestly.
    [[nodiscard]] UInt32 boardFeatures()
    {
        return typeBit(bibowire::Type::TYPE_SCAN) | typeBit(bibowire::Type::TYPE_DECIDE)
             | typeBit(bibowire::Type::TYPE_BOARD) | typeBit(bibowire::Type::TYPE_LIDAR_INFO)
             | typeBit(bibowire::Type::TYPE_EVENT) | typeBit(bibowire::Type::TYPE_CTLSTATE)
             | typeBit(bibowire::Type::TYPE_CMDACK) | typeBit(bibowire::Type::TYPE_CAMERA);
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
        // THE ONE EXCEPTION TO THE ZERO MASK, AND THE REASON IT EXISTS.
        //
        // Every other type follows the rule below: a viewer that never
        // subscribed is not a viewer that wants nothing, so it gets everything.
        // That is right for a 2.5 KB revolution and it is WRONG for a camera.
        // A megabyte a second is not a sensible thing to hand somebody who
        // never mentioned it - it would arrive at every viewer written before
        // this producer existed, take the bandwidth the scan needs, and spin up
        // a capture on a board nobody is watching a picture on.
        //
        // So CAMERA is sent ONLY on an explicit bit. Section 10 already says the
        // camera "arrives switched off; a viewer that wants it asks", and this
        // is that sentence in code. The bit is `tag - 0x10` like every other, so
        // CAMERA (0x20) is bit 16 - nothing new to learn, just the one default
        // that is off. test_viewfeed asserts a zero-mask subscriber gets scan
        // and state and NOT camera.
        if(type == bibowire::Type::TYPE_CAMERA)
        {
            return (c.typeMask & bit) != 0u;
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
                    // Cleared only now, because only now has the viewer been
                    // TOLD. A count cleared at encode time is a count the
                    // viewer never receives, and the whole purpose of the field
                    // is that gaps are counted rather than smoothed over.
                    const bibowire::Type sent = q.type;
                    c.outBytes -= q.bytes.size();
                    c.sentOfHead = 0;
                    c.out.pop_front();
                    countFrame(false);
                    if(sent == bibowire::Type::TYPE_SCAN)
                    {
                        c.droppedLive = 0;
                    }
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

        // COUNTED THE MOMENT IT IS WELCOMED, not at the end of this pass. The
        // count is what publish() checks before it takes the lock, and what the
        // BOARD frame reports as `clients` - so a client counted a pass late is
        // a client whose own first BOARD says it is not there, and a
        // revolution published in that window is dropped at the door of a feed
        // that does have a viewer. Both were found on the board: the BOARD
        // frame said 0 clients to the very viewer reading it.
        sh.count.store(liveClients(clients));

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

    // ---- tuning ------------------------------------------------------------
    //
    // Four verbs that reach the car's TRIM rather than its motion: the servo's
    // end stops, its centre, the throttle's working range, and how fast either
    // output may move. They used to live in a hub that is gone, and
    // docs/bibowire.md section 5 is the contract they arrive under.
    //
    // NOTHING HERE TOUCHES THE SERIAL PORT. This thread validates, answers the
    // operator with a CMDACK naming the value that was taken, and queues the
    // request for the pilot's tick - the same split every other message in this
    // file keeps, and the reason a stalled Pico cannot stall the socket.

    [[nodiscard]] Bool isTuningVerb(bibowire::Verb v)
    {
        return v == bibowire::Verb::VERB_SET_ESC_LIMITS
            || v == bibowire::Verb::VERB_SET_SERVO_LIMITS
            || v == bibowire::Verb::VERB_SET_SERVO_TRIM
            || v == bibowire::Verb::VERB_SET_SLEW;
    }

    // The car's arm state as the PILOT last reported it, which is the only
    // channel the board has for the fact.
    //
    // READ THE HONESTY NOTE ON Applied IN THE HEADER BEFORE TRUSTING THIS. The
    // pilot does not drive from CONTROL in this build and nothing calls
    // applied(), so this reads 0 - disarmed - for the whole run, and the
    // refusal below never fires today. That makes it a guard that is CORRECT
    // and not yet LOAD-BEARING, and it must not be the only thing standing
    // between a live throttle and a new limit: the Pico re-clamps and refuses
    // on its own side, which is the check that is actually running.
    [[nodiscard]] Bool armedNow()
    {
        LockGuard<Mutex> lock(sh.appliedM);
        return sh.applied.armed != 0u;
    }

    // POSITIVE EVIDENCE ONLY. haveBoard is false until the pilot's first
    // publishBoard, and "has not said yet" is not "there is no Pico" - refusing
    // then would be this module inventing a fact it does not have. picoLink 0
    // IS the pilot saying the port is closed or that this run is --dry, and
    // that is a fact worth refusing on. Touched on this thread alone, like
    // lastBoard itself, so there is no lock here for a reason.
    [[nodiscard]] Bool picoDown()
    {
        return haveBoard && lastBoard.picoLink == 0u;
    }

    [[nodiscard]] Bool within(UInt16 v, UInt16 lo, UInt16 hi)
    {
        return v >= lo && v <= hi;
    }

    Void queueTune(const bibowire::Command& cmd)
    {
        LockGuard<Mutex> lock(sh.tuneM);
        while(sh.tunes.size() >= TUNE_MAX)
        {
            sh.tunes.pop_front();
            ++sh.tuneDropped;
            if(!sh.tuneDropSaid)
            {
                // Once, not per drop: a tick that has stopped draining will
                // drop every request after this one, and a line each would bury
                // the one line that says why.
                sh.tuneDropSaid = true;
                std::printf("viewfeed: tuning queue full - the tick is not draining it\n");
            }
        }
        Tune t;
        t.verb = cmd.verb;
        t.arg0 = cmd.arg0;
        t.arg1 = cmd.arg1;
        t.arg2 = cmd.arg2;
        sh.tunes.push_back(t);
    }

    // Fills `ack` for one tuning verb, and queues the request when it is taken.
    Void onTune(const bibowire::Command& cmd, bibowire::CmdAck* ack)
    {
        // ARMED IS CHECKED FIRST, before any talk of ranges. An operator told
        // "1000..2000 us" by a car that was never going to accept the number
        // has been answered a question they did not ask.
        if(armedNow())
        {
            ack->result = 3;
            ack->text = "the car is armed - disarm before changing its trim";
            return;
        }
        if(picoDown())
        {
            ack->result = 4;
            ack->text = "no Pico - trim lives in its RAM and there is nothing to send this to";
            return;
        }

        Array<Char, 160> buf{};
        const unsigned a1 = static_cast<unsigned>(cmd.arg1);
        const unsigned a2 = static_cast<unsigned>(cmd.arg2);

        if(cmd.verb == bibowire::Verb::VERB_SET_SERVO_LIMITS
            || cmd.verb == bibowire::Verb::VERB_SET_ESC_LIMITS)
        {
            const Bool esc = cmd.verb == bibowire::Verb::VERB_SET_ESC_LIMITS;
            const UInt16 lo = esc ? bibowire::ESC_US_HARD_MIN : bibowire::SERVO_US_HARD_MIN;
            const UInt16 hi = esc ? bibowire::ESC_US_HARD_MAX : bibowire::SERVO_US_HARD_MAX;
            const Char* what = esc ? "esc" : "servo";
            if(!within(cmd.arg1, lo, hi) || !within(cmd.arg2, lo, hi))
            {
                // The accepted range is NAMED. "Out of range" alone sends
                // somebody to read source in a field; two numbers turn the
                // refusal into the next thing to type.
                std::snprintf(
                    buf.data(),
                    buf.size(),
                    "%s limits must be %u..%u us at both ends",
                    what,
                    static_cast<unsigned>(lo),
                    static_cast<unsigned>(hi)
                );
                ack->result = 1;
                ack->text = Str(buf.data());
                return;
            }
            // min BELOW max, tested on the values that will actually be sent.
            // THIS BOARD REFUSES RATHER THAN CLAMPS, and the range test above
            // has already turned every out-of-range endpoint into a result = 1,
            // so the pair cannot be collapsed into equality between there and
            // here - the relation that holds now is the relation the Pico is
            // handed. The day a clamp is added above, this test runs again
            // AFTER it, because a clamp is exactly what can make two accepted
            // numbers equal.
            if(cmd.arg1 >= cmd.arg2)
            {
                std::snprintf(
                    buf.data(),
                    buf.size(),
                    "%s limits need min below max, not %u and %u",
                    what,
                    a1,
                    a2
                );
                ack->result = 1;
                ack->text = Str(buf.data());
                return;
            }
            queueTune(cmd);
            std::snprintf(
                buf.data(),
                buf.size(),
                "%s limits set to %u..%u us - the Pico holds them in RAM until it reboots",
                what,
                a1,
                a2
            );
            ack->result = 0;
            ack->text = Str(buf.data());
            return;
        }

        if(cmd.verb == bibowire::Verb::VERB_SET_SERVO_TRIM)
        {
            if(!within(cmd.arg1, bibowire::SERVO_US_HARD_MIN, bibowire::SERVO_US_HARD_MAX))
            {
                std::snprintf(
                    buf.data(),
                    buf.size(),
                    "servo centre must be %u..%u us",
                    static_cast<unsigned>(bibowire::SERVO_US_HARD_MIN),
                    static_cast<unsigned>(bibowire::SERVO_US_HARD_MAX)
                );
                ack->result = 1;
                ack->text = Str(buf.data());
                return;
            }
            queueTune(cmd);
            std::snprintf(
                buf.data(),
                buf.size(),
                "servo centre set to %u us - the Pico holds it in RAM until it reboots",
                a1
            );
            ack->result = 0;
            ack->text = Str(buf.data());
            return;
        }

        if(cmd.verb == bibowire::Verb::VERB_SET_SLEW)
        {
            if(cmd.arg0 > bibowire::SLEW_AXIS_THROTTLE)
            {
                ack->result = 1;
                ack->text = "slew axis must be 0 both, 1 steer or 2 throttle";
                return;
            }
            if(!within(cmd.arg1, bibowire::SLEW_US_MIN, bibowire::SLEW_US_MAX))
            {
                std::snprintf(
                    buf.data(),
                    buf.size(),
                    "slew must be %u..%u us per tick",
                    static_cast<unsigned>(bibowire::SLEW_US_MIN),
                    static_cast<unsigned>(bibowire::SLEW_US_MAX)
                );
                ack->result = 1;
                ack->text = Str(buf.data());
                return;
            }
            const Char* axis = cmd.arg0 == bibowire::SLEW_AXIS_STEER
                ? "steer"
                : (cmd.arg0 == bibowire::SLEW_AXIS_THROTTLE ? "throttle" : "steer and throttle");
            queueTune(cmd);
            // BOTH UNITS. us-per-tick is what the wire carries; us-per-second
            // is what an operator thinks in, and nobody should have to know
            // that a tick is 20 ms to read their own acknowledgement.
            std::snprintf(
                buf.data(),
                buf.size(),
                "%s slew set to %u us per tick - %u us/s at %u ticks a second",
                axis,
                a1,
                a1 * static_cast<unsigned>(bibowire::SLEW_TICKS_PER_S),
                static_cast<unsigned>(bibowire::SLEW_TICKS_PER_S)
            );
            ack->result = 0;
            ack->text = Str(buf.data());
            return;
        }

        // Unreachable while isTuningVerb and the branches above agree about
        // which verbs are tuning verbs. Answered rather than left silent for
        // the day they stop agreeing: a COMMAND with no CMDACK is the one thing
        // this protocol promises cannot happen.
        ack->result = 2;
        ack->text = "that is not a verb this board tunes";
    }

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
        else if(isTuningVerb(cmd.verb))
        {
            // Trim is the ONE family of verbs this board really does forward,
            // and it is safe to forward for the reason the refusal below is not:
            // it changes what the outputs are allowed to do, not what they are
            // doing, and it is refused outright while the car is armed.
            onTune(cmd, &ack);
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
                // CLAMPED, NEVER REFUSED. A viewer asking for more than this
                // board will give gets the most it will give, because the
                // alternative - dropping the whole SUBSCRIBE - would turn a
                // request for a faster picture into no picture at all.
                c.camFps = m.camFps > bibowire::CAM_FPS_MAX
                    ? bibowire::CAM_FPS_MAX
                    : m.camFps;
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

    // ---- the camera --------------------------------------------------------
    //
    // THE FIRST AND ONLY BULK PRODUCER. v1 had none: CAMERA was a reserved tag,
    // classOf answered CLASS_BULK, and nothing ever queued one. Everything
    // below exists because the device and the link are each already spoken for.
    //
    // /dev/video0 IS SINGLE-OPENER, MEASURED. A second streamer gets
    // "VIDIOC_REQBUFS returned -1 (Device or resource busy)" and writes zero
    // bytes. The open() itself SUCCEEDS - the refusal arrives later, at buffer
    // setup - which is why this cannot be answered by probing the node first.
    // The phone dashboard (tools/status/status_server.py, class Camera) opens
    // the same device and parks it five seconds after nobody is watching, so
    // the two CANNOT both hold it and whichever loses has to say which one lost.
    //
    // NOTHING RE-ENCODES, and nothing here could: there is no ffmpeg, no cv2,
    // no v4l2 binding, no PIL and no numpy on this board. One long-lived
    // v4l2-ctl streams mmap'd buffers into a pipe and this module looks for
    // frame boundaries - status_server.py's proven path, for its reasons - so
    // the JPEGs the sensor produced are the JPEGs the viewer renders.
    //
    // AND IT IS CLASS_BULK, which is what makes it safe to add at all: section
    // 7's drop machinery discards a camera frame before any scan or state
    // frame, so on a stalling hotspot the picture degrades and the car's
    // picture of the world does not.

    [[nodiscard]] Str envOr(CharSeq name, const Str& fallback)
    {
        const Char* v = std::getenv(name);
        return (v != nullptr && v[0] != '\0') ? Str(v) : fallback;
    }

    Void readCamCfg()
    {
        camCfg = CamCfg();
        camCfg.devOverride = envOr("BIBO_CAM_DEV", "");
        // Seeded so the "absent" sentence has a name to print before the first
        // open; startCamera re-resolves it on every attempt regardless.
        camCfg.dev = camCfg.devOverride.empty() ? cameraDevDefault() : camCfg.devOverride;

        const Str size = envOr("BIBO_CAM_SIZE", "640x480");
        const Size x = size.find('x');
        if(x != Str::npos)
        {
            const long w = std::strtol(size.substr(0, x).c_str(), nullptr, 10);
            const long h = std::strtol(size.substr(x + 1u).c_str(), nullptr, 10);
            if(w > 0 && h > 0 && w <= 0xFFFF && h <= 0xFFFF)
            {
                camCfg.width = static_cast<UInt16>(w);
                camCfg.height = static_cast<UInt16>(h);
            }
        }

        const Str fps = envOr("BIBO_CAM_FPS", "");
        if(!fps.empty())
        {
            const Float64 v = std::strtod(fps.c_str(), nullptr);
            // 0 is UNCAPPED, and is a deliberate thing to be able to ask for on
            // a bench cable. A negative or unreadable value is not, and falls
            // back to the default rather than becoming a division by something
            // absurd.
            if(v >= 0.0 && v <= 240.0)
            {
                camCfg.periodMs = v > 0.0 ? 1000.0 / v : 0.0;
            }
        }
    }

    // The picture's OWN dimensions, off its SOF marker.
    //
    // v4l2-ctl NEGOTIATES the format: a device is free to answer a 640x480
    // request with something else, and there is nothing in the bytes that would
    // make that visible. A header repeating what was ASKED FOR would then
    // describe a picture that is not the one attached - this repo's named
    // failure with a resolution on it - so the size is read off the frame and
    // the request is only the fallback for a frame with no SOF in it.
    [[nodiscard]] Bool jpegSize(const UInt8* d, Size n, UInt16* w, UInt16* h)
    {
        if(d == nullptr || w == nullptr || h == nullptr || n < 4u)
        {
            return false;
        }
        if(d[0] != 0xFFu || d[1] != 0xD8u)
        {
            return false;
        }
        Size p = 2u;
        while(p + 4u <= n)
        {
            if(d[p] != 0xFFu)
            {
                return false;
            }
            const UInt8 m = d[p + 1u];
            // Fill bytes, and the markers that carry no segment at all.
            if(m == 0xFFu)
            {
                ++p;
                continue;
            }
            if(m == 0x01u || (m >= 0xD0u && m <= 0xD8u))
            {
                p += 2u;
                continue;
            }
            // Start of scan, or end of image: the pixel data begins and there
            // was no frame header before it.
            if(m == 0xD9u || m == 0xDAu)
            {
                return false;
            }
            const Size segLen = (static_cast<Size>(d[p + 2u]) << 8u) | static_cast<Size>(d[p + 3u]);
            if(segLen < 2u)
            {
                return false;
            }
            // Every SOFn except DHT (0xC4), JPG (0xC8) and DAC (0xCC).
            const Bool sof = m >= 0xC0u && m <= 0xCFu && m != 0xC4u && m != 0xC8u && m != 0xCCu;
            if(sof)
            {
                if(p + 9u > n)
                {
                    return false;
                }
                *h = static_cast<UInt16>((static_cast<UInt32>(d[p + 5u]) << 8u) | d[p + 6u]);
                *w = static_cast<UInt16>((static_cast<UInt32>(d[p + 7u]) << 8u) | d[p + 8u]);
                return *w != 0u && *h != 0u;
            }
            p += 2u + segLen;
        }
        return false;
    }

    // Where the next start-of-image begins at or after `from`, else b.size().
    [[nodiscard]] Size findSoi(const Vec<UInt8>& b, Size from)
    {
        if(b.size() < CAM_SOI.size())
        {
            return b.size();
        }
        for(Size i = from; i + CAM_SOI.size() <= b.size(); ++i)
        {
            if(b[i] == CAM_SOI[0] && b[i + 1u] == CAM_SOI[1] && b[i + 2u] == CAM_SOI[2])
            {
                return i;
            }
        }
        return b.size();
    }

    [[nodiscard]] Bool anyWantsCamera(const Vec<Client>& clients)
    {
        for(const Client& c : clients)
        {
            if(wants(c, bibowire::Type::TYPE_CAMERA))
            {
                return true;
            }
        }
        return false;
    }

    // How often a picture is offered, from what the SUBSCRIBERS asked for.
    //
    // THE FASTEST REQUEST WINS AND EVERY SUBSCRIBER GETS EVERY OFFERED FRAME.
    // That is a deliberate choice over pacing each viewer separately, and the
    // reason is frameIndex: it is monotonic and the viewer counts its gaps as
    // dropped pictures, so a viewer held to a slower rate than the capture
    // would be shown "frames 91-94 missing" for frames the board decided on
    // purpose not to send it - a made-up fault, which is worse than the thing
    // it would be reporting.
    //
    // The cost is stated rather than hidden: two viewers asking for different
    // rates both get the higher one. That is safe here and nowhere else,
    // because CAMERA is CLASS_BULK - a link that cannot carry the rate discards
    // pictures ahead of every scan and state frame, so the viewer that wanted
    // less loses camera frames and never the car's view of the room.
    //
    // 0 is UNCAPPED and propagates as such: it is the one value that must not
    // be treated as "slowest", since a bench cable asking for everything the
    // device produces is a deliberate thing to be able to ask for.
    [[nodiscard]] Float64 offeredCamPeriodMs(const Vec<Client>& clients)
    {
        Bool asked = false;
        Float64 best = 0.0;
        for(const Client& c : clients)
        {
            if(!wants(c, bibowire::Type::TYPE_CAMERA))
            {
                continue;
            }
            // A subscriber that named no rate is content with the board's
            // default, so it is the default that enters the comparison for it.
            const Float64 per = c.camFps == 0u
                ? camCfg.periodMs
                : 1000.0 / static_cast<Float64>(c.camFps);
            if(!asked || per < best)
            {
                asked = true;
                best = per;
            }
            if(per == 0.0)
            {
                return 0.0;
            }
        }
        return asked ? best : camCfg.periodMs;
    }

    // Said to THE CAMERA'S SUBSCRIBERS, which is who is looking at the blank
    // panel. An absence with a reason beats a silent nothing, and a viewer that
    // asked for a picture and got neither picture nor sentence is the exact
    // failure this repo is named after.
    Void sayCamera(Vec<Client>& clients, bibowire::Severity severity, const Str& text)
    {
        bibowire::Event e;
        e.tMonoUs = monoUs();
        e.severity = severity;
        e.text = text.size() > bibowire::MAX_EVENT_TEXT
            ? text.substr(0, bibowire::MAX_EVENT_TEXT)
            : text;
        for(Client& c : clients)
        {
            if(!wants(c, bibowire::Type::TYPE_CAMERA))
            {
                continue;
            }
            emit(c, bibowire::Type::TYPE_EVENT, [&e](UInt8* out, Size cap) {
                return bibowire::writeEvent(e, out, cap);
            });
        }
        std::printf("viewfeed: %s\n", e.text.c_str());
    }

    // EBUSY IS SAID IN WORDS, AND IT NAMES THE LIKELY HOLDER - which is the
    // phone dashboard, because that is the only other thing on this board that
    // opens the device. status_server.py asks the same question in the same
    // words when its own capture ends early.
    [[nodiscard]] Str cameraWhy(const Str& diag, Int32 status)
    {
        // THE REAL EBUSY SIGNATURE, MEASURED ON THIS BOARD RATHER THAN ASSUMED.
        //
        // This was written expecting a losing v4l2-ctl to say "VIDIOC_REQBUFS
        // returned -1 (Device or resource busy)" on stderr, and to key off that
        // text. IT DOES NOT. Held against a second streamer on this board it
        // writes ZERO bytes to stdout, ZERO to stderr, and exits 255 - so the
        // stderr text is a bonus that usually is not there, and the EXIT STATUS
        // is the signal that is. A capture that produced no picture and exited
        // non-zero is the device being held by somebody else, and it is said in
        // words rather than left as a blank panel.
        const Bool exited = status >= 0 && WIFEXITED(status);
        const Int32 code = exited ? static_cast<Int32>(WEXITSTATUS(status)) : -1;

        if(code == 127)
        {
            // The child's own "exec failed" exit, from startCamera below.
            return "cannot start v4l2-ctl - it is not installed on this board";
        }
        if(code > 0 || diag.find("busy") != Str::npos || diag.find("Busy") != Str::npos)
        {
            // status_server.py's sentence for exactly this, with the likely
            // holder named: the phone dashboard is the only other thing on this
            // board that opens the device, and /dev/video0 is single-opener.
            return "camera stream ended - is something else holding " + camCfg.dev
                 + "? the phone dashboard opens the same device and parks it 5 s after "
                   "nobody is watching";
        }
        if(!diag.empty())
        {
            // One line of it. v4l2-ctl is chatty and EVENT carries a sentence.
            Str said = diag;
            const Size nl = said.find('\n');
            if(nl != Str::npos)
            {
                said = said.substr(0, nl);
            }
            return "camera: " + said;
        }
        return "camera stream ended with no picture and nothing said - is something else "
               "holding " + camCfg.dev + "?";
    }

    // A capture that dies inside a second, over and over, is a device that is
    // not going to work this second. Retrying twice a second for as long as
    // somebody leaves a subscription open is thousands of spawns an hour
    // against a board whose whole job is elsewhere.
    Void backOff()
    {
        cam.fails = cam.fails + 1 > CAM_FAIL_CEILING ? CAM_FAIL_CEILING : cam.fails + 1;
        const Float64 wait = CAM_RETRY_STEP_MS * static_cast<Float64>(1 + cam.fails);
        cam.waitMs = wait > CAM_RETRY_MAX_MS ? CAM_RETRY_MAX_MS : wait;
        cam.failedAt = monoNow();
        cam.waiting = true;
    }

    // Ends the capture and hands back the child's exit status, or -1.
    //
    // SIGKILL AND NOT SIGTERM, deliberately. v4l2-ctl has nothing to flush: the
    // kernel releases the V4L2 buffers and the device when the process exits,
    // however it exits. A polite signal would buy nothing and cost a wait, and
    // the wait is the problem - this thread owes every viewer a CTLSTATE every
    // 50 ms, so a loop spinning on a courteous exit trades the control clock for
    // a courtesy nobody receives. SIGKILL cannot be caught, so the waitpid
    // returns promptly.
    //
    // Killing a child that has ALREADY exited is harmless and does not destroy
    // the answer: a process that has exited keeps its status until it is reaped,
    // so the code below still reports why it stopped.
    //
    // AND IT IS REAPED. A killed child is not a gone child: it holds a
    // process-table slot until its parent waits on it, and this parent is a
    // long-lived service that starts a capture every time somebody subscribes.
    // status_server.py's kill() carries the same one-line fix, because the
    // failure it prevents - a board that cannot fork, including the child sshd
    // needs to answer a connection - locks you out of the machine.
    [[nodiscard]] Int32 killCamera()
    {
        if(cam.pid < 0)
        {
            return -1;
        }
        static_cast<Void>(::kill(cam.pid, SIGKILL));
        Int32 status = -1;
        while(::waitpid(cam.pid, &status, 0) < 0)
        {
            if(errno != EINTR)
            {
                cam.pid = -1;
                return -1;
            }
        }
        cam.pid = -1;
        return status;
    }

    Void closeCamera(CharSeq why)
    {
        static_cast<Void>(killCamera());
        if(cam.outFd >= 0)
        {
            ::close(cam.outFd);
            cam.outFd = -1;
        }
        if(cam.errFd >= 0)
        {
            ::close(cam.errFd);
            cam.errFd = -1;
        }
        // RELEASED, not merely cleared. An unwatched camera must cost the board
        // nothing, and a 45 KB partial frame plus an encode buffer held against
        // a subscription that ended is precisely the cost this gate exists to
        // avoid. clear() would keep every byte of both.
        Vec<UInt8>().swap(cam.partial);
        Vec<UInt8>().swap(cam.encode);
        cam.diag.clear();
        cam.everFrame = false;
        if(why != nullptr)
        {
            std::printf("viewfeed: camera released - %s\n", why);
        }
    }

    // One whole JPEG, offered to whoever asked for pictures.
    Void offerCamera(Vec<Client>& clients, const UInt8* jpeg, Size len)
    {
        cam.everFrame = true;

        // THE CAP IS APPLIED HERE AND NOT AT THE DEVICE. A device asked for a
        // lower rate with --set-parm may simply ignore the request and leave
        // the rate unchanged, and nothing in the picture would say so. Dropping
        // on this side cannot fail silently: what is not sent is not sent.
        // status_server.py caps in the same place for the same reason.
        // THE RATE THE SUBSCRIBERS ASKED FOR, and this board's own default only
        // when nobody asked. The viewer is the end that knows whether it is on
        // a LAN or a phone hotspot; this end knows only that it has a camera
        // and a socket, which is why the number could never be chosen well from
        // here alone.
        const Float64 offerMs = offeredCamPeriodMs(clients);
        if(cam.everSent && offerMs > 0.0 && elapsedMs(cam.lastSentAt) < offerMs)
        {
            return;
        }
        if(len == 0u || len > CAM_MAX_JPEG)
        {
            return;
        }

        const Size need = bibowire::FRAME_OVERHEAD + CAM_BODY_OVERHEAD + len;
        if(cam.encode.size() < need)
        {
            cam.encode.resize(need);
        }

        UInt16 w = camCfg.width;
        UInt16 h = camCfg.height;
        static_cast<Void>(jpegSize(jpeg, len, &w, &h));

        bibowire::Camera m;
        m.tMonoUs = monoUs();
        m.frameIndex = cam.frameIndex;
        m.width = w;
        m.height = h;
        // Echoed on every frame so a capture is self-describing, which is
        // section 10's rule for this type rather than an invented one.
        m.codec = 1;
        m.flags = 0;
        m.data.assign(jpeg, jpeg + len);

        const TimePoint before = monoNow();
        const Size bodyLen = bibowire::writeCamera(
            m,
            cam.encode.data() + bibowire::HEAD_BYTES,
            cam.encode.size() - bibowire::HEAD_BYTES
        );
        if(bodyLen == 0u)
        {
            return;
        }
        countEncode(elapsedMs(before) * 1000000.0);

        // The BODY is built once; only the header and the CRC are rewritten per
        // client, because seq and flags are per client and 45 KB is too much to
        // re-encode four times for the sake of two fields.
        Bool any = false;
        for(Client& c : clients)
        {
            if(!wants(c, bibowire::Type::TYPE_CAMERA))
            {
                continue;
            }
            bibowire::Head h;
            h.type = bibowire::Type::TYPE_CAMERA;
            h.ver = 1;
            h.flags = outFlags();
            h.seq = c.txSeq;
            const Size total = framedIn(cam.encode.data(), cam.encode.size(), h, bodyLen);
            if(total == 0u)
            {
                continue;
            }
            enqueueFrom(c, bibowire::Type::TYPE_CAMERA, cam.encode.data(), total);
            any = true;
        }
        if(any)
        {
            cam.lastSentAt = monoNow();
            cam.everSent = true;
            // MONOTONIC, and never rewound across a capture restart. A viewer
            // that sees the number JUMP has missed frames and can say so; one
            // that sees it go backwards is being shown pictures it already has,
            // labelled as new. status_server.py documents the same hazard from
            // the other side - a sequence that did not rewind while the frame
            // behind it did.
            ++cam.frameIndex;
            cam.said = false;
        }
    }

    // The capture ended. Work out why, say it once, and set the backoff.
    Void endCamera(Vec<Client>& clients)
    {
        const Float64 ran = elapsedMs(cam.startedAt);
        const Bool delivered = cam.everFrame;
        const Str diag = cam.diag;
        // Reaped HERE, before closeCamera, because the exit status is the thing
        // that says WHY the capture stopped and closeCamera would discard it.
        const Int32 status = killCamera();

        // Explained ONCE per failure episode, and only when the capture
        // produced no picture at all. A capture that ran, delivered frames and
        // then ended is a cable moving or a device resetting, and the backoff
        // handles it without a sentence per retry.
        if(!delivered && !cam.said)
        {
            cam.said = true;
            sayCamera(clients, bibowire::Severity::SEVERITY_WARN, cameraWhy(diag, status));
        }
        closeCamera(nullptr);
        if(ran < 1000.0 || !delivered)
        {
            backOff();
        }
        else
        {
            cam.fails = 0;
        }
    }

    Void pumpCamera(Vec<Client>& clients)
    {
        // stderr FIRST: it is where the answer lives on the run where stdout
        // stays empty, which is exactly the run this has to explain.
        for(;;)
        {
            Array<Char, 256> chunk{};
            const ISize n = ::read(cam.errFd, chunk.data(), chunk.size());
            if(n <= 0)
            {
                break;
            }
            if(cam.diag.size() < CAM_DIAG_BYTES)
            {
                cam.diag.append(chunk.data(), static_cast<Size>(n));
            }
        }

        Bool ended = false;
        for(;;)
        {
            const ISize n = ::read(cam.outFd, camChunk.data(), camChunk.size());
            if(n > 0)
            {
                cam.partial.insert(cam.partial.end(), camChunk.data(), camChunk.data() + n);
                continue;
            }
            if(n == 0)
            {
                ended = true;
                break;
            }
            if(errno == EINTR)
            {
                continue;
            }
            if(errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }
            ended = true;
            break;
        }

        // A frame is whole only once the NEXT one has begun - start-of-image is
        // the only boundary MJPEG gives, because a JPEG's own end marker can
        // occur inside its payload. This costs exactly one frame of latency and
        // is the reason a viewer is never handed half a picture.
        for(;;)
        {
            const Size start = findSoi(cam.partial, 0);
            if(start == cam.partial.size())
            {
                break;
            }
            const Size next = findSoi(cam.partial, start + CAM_SOI.size());
            if(next == cam.partial.size())
            {
                // Anything before the first marker is not part of a picture.
                if(start > 0u)
                {
                    cam.partial.erase(
                        cam.partial.begin(),
                        cam.partial.begin() + static_cast<ISize>(start)
                    );
                }
                break;
            }
            offerCamera(clients, cam.partial.data() + start, next - start);
            cam.partial.erase(cam.partial.begin(), cam.partial.begin() + static_cast<ISize>(next));
        }

        if(cam.partial.size() > CAM_MAX_PARTIAL)
        {
            // Not a picture. Rather than grow without bound, drop back to
            // hunting for the next marker - the deliberate resync the scan feed
            // already does on an impossible line.
            cam.partial.clear();
            std::printf("viewfeed: camera resyncing - no frame boundary in 4 MiB\n");
        }

        if(ended)
        {
            endCamera(clients);
        }
    }

    Void startCamera(Vec<Client>& clients)
    {
        // RE-RESOLVE ON EVERY ATTEMPT. See CamCfg for why: the device renames
        // itself when it re-enumerates, so a name resolved at boot goes stale
        // the first time the cable twitches.
        if(camCfg.devOverride.empty())
        {
            camCfg.dev = cameraDevDefault();
        }

        if(::access(camCfg.dev.c_str(), F_OK) != 0)
        {
            if(!cam.said)
            {
                cam.said = true;
                sayCamera(
                    clients,
                    bibowire::Severity::SEVERITY_WARN,
                    camCfg.dev + " absent - camera unplugged"
                );
            }
            backOff();
            return;
        }

        Array<Int32, 2> outPipe{ -1, -1 };
        Array<Int32, 2> errPipe{ -1, -1 };
        if(::pipe2(outPipe.data(), O_CLOEXEC) < 0)
        {
            backOff();
            return;
        }
        if(::pipe2(errPipe.data(), O_CLOEXEC) < 0)
        {
            ::close(outPipe[0]);
            ::close(outPipe[1]);
            backOff();
            return;
        }

        Array<Char, 128> fmt{};
        std::snprintf(
            fmt.data(),
            fmt.size(),
            "--set-fmt-video=width=%u,height=%u,pixelformat=MJPG",
            static_cast<unsigned>(camCfg.width),
            static_cast<unsigned>(camCfg.height)
        );

        // Built BEFORE the fork, because building it after would allocate, and
        // between fork and exec this process may not allocate. Str::data() is
        // non-const in C++20, so execvp's char*const* needs no cast.
        Vec<Str> args;
        args.push_back("v4l2-ctl");
        args.push_back("-d");
        args.push_back(camCfg.dev);
        args.push_back(fmt.data());
        args.push_back("--stream-mmap");
        args.push_back("--stream-count=0");
        args.push_back("--stream-to=-");
        Vec<Char*> argv;
        for(Str& a : args)
        {
            argv.push_back(a.data());
        }
        argv.push_back(nullptr);

        const pid_t pid = ::fork();
        if(pid < 0)
        {
            const Str why = Str("cannot start v4l2-ctl: ") + std::strerror(errno);
            ::close(outPipe[0]);
            ::close(outPipe[1]);
            ::close(errPipe[0]);
            ::close(errPipe[1]);
            if(!cam.said)
            {
                cam.said = true;
                sayCamera(clients, bibowire::Severity::SEVERITY_ERROR, why);
            }
            backOff();
            return;
        }
        if(pid == 0)
        {
            // BETWEEN fork AND exec, NOTHING BUT ASYNC-SIGNAL-SAFE CALLS. This
            // process has other threads - the pilot's control tick among them -
            // and a lock any of them held at the instant of the fork is held
            // forever in this child. dup2, execvp and _exit are the whole list
            // used here, and no allocation happens on this path.
            //
            // Every other descriptor this process owns - both listening
            // sockets, the UDP socket, the wake pipe and every client - was
            // opened CLOEXEC, so execvp closes them. A capture holding a copy of
            // the listening socket would keep port 8020 bound after the pilot
            // exited.
            if(::dup2(outPipe[1], STDOUT_FILENO) >= 0 && ::dup2(errPipe[1], STDERR_FILENO) >= 0)
            {
                ::execvp("v4l2-ctl", argv.data());
            }
            // Only reached when exec failed. The parent diagnoses it from the
            // empty pipe rather than from anything written here.
            ::_exit(127);
        }

        ::close(outPipe[1]);
        ::close(errPipe[1]);
        cam.pid = static_cast<Int32>(pid);
        cam.outFd = outPipe[0];
        cam.errFd = errPipe[0];
        // Non-blocking on THIS end only: the child's end must stay blocking or
        // v4l2-ctl gets EAGAIN on a full pipe and treats it as a write error.
        static_cast<Void>(::fcntl(cam.outFd, F_SETFL, O_NONBLOCK));
        static_cast<Void>(::fcntl(cam.errFd, F_SETFL, O_NONBLOCK));
        cam.startedAt = monoNow();
        cam.everFrame = false;
        cam.diag.clear();
        std::printf(
            "viewfeed: camera opened - %s %ux%u MJPG\n",
            camCfg.dev.c_str(),
            static_cast<unsigned>(camCfg.width),
            static_cast<unsigned>(camCfg.height)
        );
    }

    // Opened while at least one viewer subscribes to CAMERA, released when the
    // last one stops. The same bargain the lidar and the dashboard already
    // keep, and the reason an unwatched camera costs the board nothing: no
    // process, no pipes, no buffers, and the device handed straight back to
    // whoever wants it next.
    Void tendCamera(Vec<Client>& clients)
    {
        if(!anyWantsCamera(clients))
        {
            if(cam.pid >= 0 || cam.outFd >= 0)
            {
                closeCamera("the last subscriber went");
            }
            // A fresh slate for the next person to look, rather than serving
            // out a backoff earned by a camera that was unplugged an hour ago.
            cam.fails = 0;
            cam.waiting = false;
            cam.said = false;
            return;
        }
        if(cam.outFd >= 0)
        {
            pumpCamera(clients);
            return;
        }
        if(cam.waiting && elapsedMs(cam.failedAt) < cam.waitMs)
        {
            return;
        }
        cam.waiting = false;
        startCamera(clients);
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
                // The count is stamped from what has accumulated SO FAR and is
                // NOT cleared here. Clearing it after emit() was a bug the
                // board found and no amount of reading had: the drop happens
                // INSIDE emit(), when enqueue() coalesces this frame over the
                // one already queued, so zeroing the counter on the next line
                // destroyed every increment the instant it was made and
                // droppedSinceLast could never be anything but 0. It is cleared
                // in flush(), when the frame carrying it has actually gone.
                bibowire::Scan s = item.scan;
                s.droppedSinceLast = c.droppedLive;
                s.scanDivisor = c.scanDivisor;
                emit(c, bibowire::Type::TYPE_SCAN, [&s](UInt8* out, Size cap) {
                    return bibowire::writeScan(s, out, cap);
                });
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

            // THE KERNEL'S SEND BUFFER HAS TO BE SMALL OR THE RING IS NOT THE
            // BOUND. Section 7 promises that a stalled viewer's pending bytes
            // stay at about one SCAN plus one of each vital frame, "regardless
            // of how long the stall lasts". That is only true if send() starts
            // refusing while the queue is still ours to manage: with the
            // default socket buffer - megabytes on loopback, and autotuned
            // upward on a real link - send() keeps succeeding and revolutions
            // pile up INSIDE THE KERNEL, where the drop classes cannot coalesce
            // them, BEHIND_MS cannot age them and a viewer is handed seconds of
            // stale pictures in order. Found on the board: a viewer that read
            // nothing for five seconds was killed by the PING timeout with the
            // ring empty the whole time, because 2.5 MB had gone into the
            // socket. 32 KiB is about six revolutions, so what is beyond this
            // module's reach stays under a second even at full rate.
            const Int32 sndBuf = 32 * 1024;
            static_cast<Void>(::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndBuf, sizeof(sndBuf)));

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

            // The capture's pipe, appended AFTER the clients so nothing
            // disturbs the c.at indices just taken. Its revents are never
            // examined: tendCamera drains this fd to EAGAIN every pass anyway,
            // so the entry exists only to wake the loop promptly rather than
            // leave 45 KB sitting in a pipe for the rest of the 20 ms timeout.
            if(cam.outFd >= 0)
            {
                fds.push_back(pollfd{ cam.outFd, POLLIN, 0 });
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

            // AFTER the reads, so a SUBSCRIBE that arrived this pass opens the
            // device this pass, and BEFORE the flush below, so a frame read out
            // of the pipe this pass goes out on this pass's send().
            tendCamera(clients);

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
      {
          // A previous run's trim is not this run's. The Pico was rebooted or
          // reopened between the two as often as not, and forwarding a value
          // the operator asked for before the restart would be this module
          // acting on an intent that has expired.
          LockGuard<Mutex> lock(sh.tuneM);
          sh.tunes.clear();
          sh.tuneDropped = 0;
          sh.tuneDropSaid = false;
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
      cam = Cam();
      readCamCfg();
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

  Bool tune(Tune* out)
  {
      if(out == nullptr || !running)
      {
          return false;
      }
      LockGuard<Mutex> lock(sh.tuneM);
      if(sh.tunes.empty())
      {
          return false;
      }
      // FRONT, not back. These come out in the order the operator performed
      // them, because two limits set a moment apart are two acts and the second
      // is not a correction of the first.
      *out = sh.tunes.front();
      sh.tunes.pop_front();
      return true;
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
      // Nothing may outlive this call holding the camera open. The capture is a
      // CHILD PROCESS, so it survives its parent unless something says
      // otherwise, and a v4l2-ctl still on /dev/video0 is exactly why the phone
      // dashboard would find the device busy after the pilot exited. Safe here
      // because the thread that owns `cam` has been joined.
      closeCamera("the feed is stopping");
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

  Bool tune(Tune* out)
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
