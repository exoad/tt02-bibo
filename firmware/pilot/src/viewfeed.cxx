#include "viewfeed.hxx"

#include <cstdio>

#if defined(__linux__)

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

namespace viewfeed
{
  namespace
  {
    // A client whose oldest unsent VITAL byte is older than this has gone,
    // whatever its socket says.
    constexpr Int64 BEHIND_MS = 500;

    // The longest the loop sleeps with nothing to do. CTLSTATE owes a datagram
    // every CTLSTATE_EVERY_MS, and a loop that sleeps through several of them
    // reports the deadman late.
    constexpr Int32 POLL_MS = 20;

    // Per client; whichever fills first.
    constexpr Size RING_BYTES = 96u * 1024u;
    constexpr Size RING_FRAMES = 12;

    // A revolution older than this at SEND time is dropped before it is
    // queued: sending it spends the bandwidth the current one needs.
    constexpr Int64 LIVE_STALE_MS = 200;

    // A phone that walks out of range stops ACKing without sending a FIN. The
    // board's PING catches that, as does SO_KEEPALIVE.
    constexpr Int64 PING_EVERY_MS = 1000;
    constexpr Int64 PONG_WAIT_MS = 4000;

    // No camera frame is queued for a client whose PING is this late (see
    // offerCamera). Far past any healthy round trip and far short of
    // PONG_WAIT_MS, so it trips only on a path that is really queueing.
    constexpr Int64 CAM_HOLD_PONG_MS = 300;

    constexpr Int64 CTLSTATE_EVERY_MS = 50;
    constexpr Int64 BOARD_EVERY_MS = 200;

    // One event per window; the suppressed count rides the next one.
    constexpr Int64 EVENT_WINDOW_MS = 100;

    // Per TCP client. A header claiming more than MAX_INBOUND_PAYLOAD is
    // refused before this fills (consume()).
    constexpr Size INBUF_BYTES = 1024;

    // Deliberately SMALL: a deep queue of control datagrams is a queue of stale
    // steering, and the board wants the newest.
    constexpr Int32 UDP_RCVBUF = 64 * 1024;

    // The largest SCAN body. A camera frame gets its own buffer, held only
    // while the device is open, so an unwatched board holds just this.
    constexpr Size SCAN_BODY_MAX = 24u + 5u * bibowire::MAX_SCAN_POINTS;
    constexpr Size ENCODE_BYTES = bibowire::FRAME_OVERHEAD + SCAN_BODY_MAX;
    constexpr Size BODY_CAP = ENCODE_BYTES - bibowire::FRAME_OVERHEAD;

    // JPEG start-of-image, the only frame boundary MJPEG gives: an end marker
    // can occur inside a payload, so a frame is whole only once the next one
    // has begun. One frame of latency, and never half a picture.
    constexpr Array<UInt8, 3> CAM_SOI = { 0xFFu, 0xD8u, 0xFFu };

    // Past this with no boundary, the bytes are dropped and the reader hunts
    // for the next marker.
    constexpr Size CAM_MAX_PARTIAL = 4u * 1024u * 1024u;

    // CAMERA's 24-byte fixed header with slack for padding. writeCamera refuses
    // rather than overruns, and CAMERA_FIXED stays private to bibowire.cxx.
    constexpr Size CAM_BODY_OVERHEAD = 32;

    // A frame fits whole or is not sent: v1 refuses FLAG_MORE.
    constexpr Size CAM_MAX_JPEG = bibowire::MAX_PAYLOAD - CAM_BODY_OVERHEAD;

    // A capture that keeps dying waits longer each time, up to
    // CAM_RETRY_MAX_MS, rather than spawning thousands of times an hour.
    constexpr Int32 CAM_FAIL_CEILING = 8;
    constexpr Float64 CAM_RETRY_STEP_MS = 500.0;
    constexpr Float64 CAM_RETRY_MAX_MS = 4000.0;

    // v4l2-ctl's stderr kept to explain a failure; what matters comes first.
    constexpr Size CAM_DIAG_BYTES = 512;

    // The rate offered when no subscriber asks. 640x480 MJPG is ~45 KB a frame
    // and a hotspot carries less than one a second, so no cap makes it fit:
    // CLASS_BULK decides what the link carries. The cap decides what the board
    // OFFERS, because a frame the ring throws away still costs capture, encode
    // and CRC on the control loop's core. BIBO_CAM_FPS raises it.
    constexpr Float64 CAM_FPS_DEFAULT = 2.0;
    constexpr UInt16 CAM_WIDTH_DEFAULT = 640;
    constexpr UInt16 CAM_HEIGHT_DEFAULT = 480;

    // Frame headers kept per direction, dumped on an abnormal close.
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

    enum class Stage
    {
        STAGE_WAIT_HELLO = 0,
        STAGE_LIVE,
    };

    // An encoded frame the socket has not taken yet.
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

        // Fixed: the receive path has no heap in it.
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

        // For the reverse-path probe (bibowire::REVERSE_PROBE_MS).
        TimePoint welcomedAt;
        UInt32 datagrams = 0;
        Bool probeSaid = false;

        // The viewer fell back to CONTROL over TCP, so CTLSTATE is mirrored
        // there as well.
        Bool tcpControl = false;

        // A zero mask is "never asked", which means everything but CAMERA.
        UInt32 typeMask = 0;
        UInt16 scanDivisor = 1;
        UInt32 revSeen = 0;

        // Clamped to bibowire::CAM_FPS_MAX on receipt; 0 did not ask.
        UInt16 camFps = 0;

        // What the viewer says it understands: logged, never acted on, and an
        // unknown bit is never refused.
        UInt32 features = 0;

        TimePoint lastPingAt;
        TimePoint pingSentAt;
        UInt64 pingToken = 0;
        Bool pingOut = false;

        // Camera frames held back for a late PING, counted and said once, so a
        // thinning picture does not read as a broken camera.
        UInt64 camHeld = 0;
        Bool camHoldSaid = false;

        // The link log's window, reset every line (logLink).
        TimePoint linkLogAt;
        TimePoint lastRxAt;
        Float64 winRxGapMs = 0.0;     // longest wait between two reads that got bytes
        UInt64 winRxBytes = 0;
        UInt64 winTxBytes = 0;        // what send() TOOK, not what was queued
        UInt32 winSends = 0;
        UInt32 winEagain = 0;
        UInt32 winPing = 0;
        UInt32 winPong = 0;
        UInt32 winPongMatched = 0;    // answered the PING outstanding; the rest were stale
        UInt32 winControl = 0;        // every CONTROL, either transport
        UInt32 winControlTcp = 0;
        UInt32 winCommand = 0;
        Int64 lastRttMs = -1;

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
        WHAT_TRIM,
        WHAT_BUNDLES,
        WHAT_BUNDLE_STATE,
        WHAT_TAGS,
        WHAT_ODOM,
        WHAT_POSE,
    };

    struct Item
    {
        What what = What::WHAT_SCAN;
        bibowire::Scan scan;
        bibowire::Decide decide;
        bibowire::BoardState board;
        bibowire::LidarInfo lidar;
        bibowire::Event event;
        Vec<bibowire::Bundle> bundles;
        UInt32 bundleGeneration = 0;
        bibowire::BundleState bundleState;
        bibowire::Tags tags;
        bibowire::Odom odom;
        bibowire::Pose pose;
        TimePoint at;
    };

    // Tuning requests waiting for the tick. A dragged slider is a burst, so
    // there is slack; a full queue means the tick stopped draining, and an
    // unbounded one would grow until the board ran out of memory.
    constexpr Size TUNE_MAX = 32;

    // Loads a viewer may have outstanding. Far smaller than TUNE_MAX: a
    // dragged slider makes requests in bursts, a person pressing Load does not.
    constexpr Size BUNDLE_MAX = 8;

    // What drive() computes the deadman from, and all it reads of this thread.
    // The loop copies it under Shared::driveM before it flushes, so an answer a
    // viewer has read is what the tick, or a Car's minder, sees too.
    struct DriveSeen
    {
        TimePoint at;                  // when this thread took the copy
        Bool haveHolder = false;
        Bool estopLatched = false;
        Bool controlGone = true;       // no CONTROL yet, or the holder left
        TimePoint lastControlAt;
        Bool enable = false;
        Bool epochMatches = false;
        Bool modeAgrees = false;
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

        // The seqlock over two slots: the writer fills slot[seq & 1] FIRST and
        // then stores seq with release; control() loads with acquire, reads,
        // re-loads, and retries if it moved.
        Atomic<UInt32> ctlSeq{ 0 };
        Array<bibowire::Control, 2> ctlSlot{};

        Mutex appliedM;
        Applied applied;

        // Whether a viewer's COMMAND ARM stands. Written by onArm and
        // bumpEpoch, read by the tick through drive(); atomic so the tick
        // never takes a lock.
        Atomic<Bool> operatorArmed{ false };

        // The tuning queue (see tune()). Drop-OLDEST when full, because the
        // newest is the operator's current intent; every drop is counted.
        Mutex tuneM;
        Deque<Tune> tunes;
        UInt64 tuneDropped = 0;
        Bool tuneDropSaid = false;

        // Loads and unloads waiting for the tick, drop-oldest like the above.
        Mutex bundleM;
        Deque<BundleRequest> bundleQ;
        UInt64 bundleDropped = 0;

        Mutex tallyM;
        Counters tally;

        Mutex driveM;
        DriveSeen driveSeen;

        // The local camera subscriber (wantCamera) and the newest frame for
        // it, copied out of the capture under camM: the detector's thread
        // never touches `cam`, which the feed's thread owns.
        Atomic<Bool> camLocal{ false };
        Atomic<UInt16> camLocalFps{ 0 };
        Mutex camM;
        CondVar camCv;
        UInt32 camSeq = 0;
        CameraFrame camLatest;
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

    // The link log (logLink). The loop's longest pass is kept per second so a
    // stall on THIS side shows in the same line as a stall on the path.
    Bool linkLog = true;
    TimePoint loopWindowAt;
    Float64 loopWorstMs = 0.0;
    Float64 loopWorstShown = 0.0;

    // The board's own state, owned by this thread.
    UInt32 armEpoch = 0;
    Bool estopLatched = false;
    Bool haveHolder = false;
    UInt32 holderSession = 0;
    TimePoint lastControlAt;
    TimePoint streamSince;     // when the holder's current unbroken stream began
    Bool everControl = false;
    UInt32 appliedSeq = 0;
    Bool holderGone = false;   // positive evidence the driver left

    // The newest state, owed to a newly welcomed client before its first scan.
    bibowire::BoardState lastBoard;
    Bool haveBoard = false;
    bibowire::LidarInfo lastLidar;
    Bool haveLidar = false;

    // The bundles this board can run. Owed to a new client for the same reason
    // the board state is: without it a viewer that connects mid-run has an
    // empty master window and no way to tell that from a board with none.
    Vec<bibowire::Bundle> lastBundles;
    UInt32 lastBundleGeneration = 0;
    Bool haveBundles = false;
    bibowire::BundleState lastBundleState;
    Bool haveBundleState = false;

    // The saved trim, owed to a viewer at WELCOME rather than at the next save.
    Str lastTrim;
    Bool haveTrim = false;
    TimePoint lastBoardAt;
    Bool boardSent = false;

    // When the last revolution reached this module: the board measuring itself
    // catches a healthy link carrying dead data, which a viewer cannot.
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

    // One read off the capture's pipe, at file scope so a 64 KiB local is not
    // zero-initialised on every read.
    Array<UInt8, 65536> camChunk{};

    // By id, not /dev/videoN: this camera re-enumerates while streaming and
    // comes back under another minor number, and the by-id path survives that.
    // Falls back to /dev/video0 for a board without the symlink.
    [[nodiscard]] inline Str cameraDevDefault()
    {
        const Str byId =
            "/dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0";
        return ::access(byId.c_str(), F_OK) == 0 ? byId : Str("/dev/video0");
    }

    struct CamCfg
    {
        // Resolved on every open (startCamera), not once: the by-id symlink
        // exists only while the device does, so a path resolved at boot would
        // latch whatever was true then. `devOverride` is BIBO_CAM_DEV, or
        // empty, and is never re-derived.
        Str dev;
        Str devOverride;
        UInt16 width = CAM_WIDTH_DEFAULT;
        UInt16 height = CAM_HEIGHT_DEFAULT;
        Float64 periodMs = 1000.0 / CAM_FPS_DEFAULT;   // 0 means uncapped
    };

    CamCfg camCfg;

    // The capture, owned by this module's thread. Every buffer is empty and
    // every fd -1 while nobody subscribes.
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

    [[nodiscard]] UInt64 monoUs()
    {
        return static_cast<UInt64>(elapsedMs(startedAt) * 1000.0);
    }

    Void wakeLoop()
    {
        const Char one = 1;
        if(::write(sh.wakeFd, &one, 1) < 0)
        {
            // A full pipe means the loop is awake already.
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
        // A running mean: what serving costs now, not since start.
        sh.tally.encodeAvgNs = was == 0
            ? static_cast<UInt32>(ns)
            : static_cast<UInt32>((static_cast<Float64>(was) * 7.0 + ns) / 8.0);
        if(static_cast<UInt32>(ns) > sh.tally.encodeMaxNs)
        {
            sh.tally.encodeMaxNs = static_cast<UInt32>(ns);
        }
    }

    // Random enough that a previous session's datagram is not mistaken for
    // this one, and NEVER 0.
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

    // Written to the journal on any abnormal close, so a disconnect can be
    // examined with journalctl rather than reproduced.
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

    // Whether the mode a viewer BELIEVES is running is the pilot's, from BOARD.
    // Positive evidence only: before the first BOARD an unknown mode agrees.
    // This thread alone touches lastBoard, so there is no lock.
    [[nodiscard]] Bool modeAgreesWith(UInt8 assumed)
    {
        return !haveBoard || assumed == lastBoard.pilotMode;
    }

    // What the deadman reads, as this thread sees it now.
    [[nodiscard]] DriveSeen seenNow()
    {
        DriveSeen s;
        s.at = monoNow();
        s.haveHolder = haveHolder;
        s.estopLatched = estopLatched;
        s.controlGone = holderGone || !everControl;
        s.lastControlAt = lastControlAt;
        const UInt32 seq = sh.ctlSeq.load(std::memory_order_acquire);
        if(seq != 0u)
        {
            const bibowire::Control& c = sh.ctlSlot[seq & 1u];
            s.enable = (c.buttons & bibowire::BUTTON_ENABLE) != 0u;
            s.epochMatches = c.armEpoch == static_cast<UInt8>(armEpoch);
            s.modeAgrees = modeAgreesWith(c.assumedMode);
        }
        return s;
    }

    // The deadman at this moment, from s and the clock alone, so any thread may
    // call it.
    [[nodiscard]] bibowire::deadman::Output deadmanOf(const DriveSeen& s)
    {
        bibowire::deadman::Inputs in;
        in.nowMs = static_cast<Int64>(elapsedMs(startedAt));
        // A LEAVE, FIN or RST from the holder is positive evidence the driver
        // is gone, so the age is forced past DEAD rather than left to time out.
        const Int64 gone = in.nowMs - static_cast<Int64>(bibowire::CONTROL_DEAD_MS);
        in.lastControlMs = s.controlGone
            ? gone
            : static_cast<Int64>(elapsedMs(startedAt) - elapsedMs(s.lastControlAt));
        in.haveHolder = s.haveHolder;
        in.estopLatched = s.estopLatched;
        in.enable = s.enable;
        in.epochMatches = s.epochMatches;
        in.modeAgrees = s.modeAgrees;
        return bibowire::deadman::step(in);
    }

    [[nodiscard]] bibowire::deadman::Output deadmanNow()
    {
        return deadmanOf(seenNow());
    }

    // Copies what the deadman reads for drive(), which runs on another thread.
    Void shareDrive()
    {
        const DriveSeen s = seenNow();
        LockGuard<Mutex> lock(sh.driveM);
        sh.driveSeen = s;
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

    [[nodiscard]] UInt8 deadmanByteOf(const bibowire::deadman::Output& d)
    {
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

    [[nodiscard]] UInt8 deadmanByte()
    {
        return deadmanByteOf(deadmanNow());
    }

    // Bumped on a deadman trip, an estop, a Pico link loss, a DISARM and a
    // control-slot change, which disarms a displaced holder for free. The ARM
    // goes with it: moving the epoch IS disarming, so there is no second list
    // of things that disarm to fall out of step.
    Void bumpEpoch()
    {
        armEpoch = (armEpoch + 1u) & 0xFFu;
        sh.operatorArmed.store(false, std::memory_order_release);
    }

    // Frames a body already sitting at `at + HEAD_BYTES`, so put() copies
    // nothing. The camera relies on it: one body, with only the per-client
    // header and CRC rewritten over it.
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

    // The newest BULK, else the oldest LIVE. Never a head the socket has partly
    // taken: half a frame followed by another frame breaks the stream.
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

    // Puts a built frame on this client's ring under the drop classes. `bytes`
    // is `scratch`, or the camera's own buffer.
    Void enqueueFrom(Client& c, bibowire::Type type, const UInt8* bytes, Size total)
    {
        if(!c.dropWhy.empty() || total == 0u)
        {
            return;
        }
        const bibowire::Class cls = bibowire::classOf(type);
        // LIVE is drop-oldest, depth 1.
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
            // VITAL is never dropped, so the client gives way.
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

    // Builds one frame for one client and queues it. `fill` writes the body and
    // returns its length, or 0 when the message cannot be represented.
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

    // WELCOME.featureMask: what this build will send (section 4), not an echo
    // of the request. CAMERA is advertised though off until subscribed, or no
    // viewer could discover it; whether the device is there is answered by an
    // EVENT when a subscription opens it.
    [[nodiscard]] UInt32 boardFeatures()
    {
        return bibowire::typeBit(bibowire::Type::TYPE_SCAN)
             | bibowire::typeBit(bibowire::Type::TYPE_DECIDE)
             | bibowire::typeBit(bibowire::Type::TYPE_BOARD)
             | bibowire::typeBit(bibowire::Type::TYPE_LIDAR_INFO)
             | bibowire::typeBit(bibowire::Type::TYPE_EVENT)
             | bibowire::typeBit(bibowire::Type::TYPE_CTLSTATE)
             | bibowire::typeBit(bibowire::Type::TYPE_CMDACK)
             | bibowire::typeBit(bibowire::Type::TYPE_BUNDLE)
             | bibowire::typeBit(bibowire::Type::TYPE_BUNDLE_STATE)
             | bibowire::typeBit(bibowire::Type::TYPE_CAMERA)
             | bibowire::typeBit(bibowire::Type::TYPE_TAGS);
    }

    [[nodiscard]] Bool wants(const Client& c, bibowire::Type type)
    {
        if(c.stage != Stage::STAGE_LIVE || !c.dropWhy.empty())
        {
            return false;
        }
        const UInt32 bit = bibowire::typeBit(type);
        if(bit == 0u)
        {
            return true;
        }
        // CAMERA only on its explicit bit, even for a zero mask: a new type
        // arrives switched off (section 11), and a megabyte a second must not
        // reach a viewer that never asked or start a capture nobody watches.
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
                c.winTxBytes += static_cast<UInt64>(n);
                ++c.winSends;
                if(c.sentOfHead >= q.bytes.size())
                {
                    // droppedLive is cleared only now, when the SCAN carrying
                    // it has gone: coalescing in enqueue() counts drops after
                    // the frame is built, so clearing earlier would lose them.
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
                ++c.winEagain;
                break;
            }
            c.dropWhy = n == 0 ? Str("closed") : Str(std::strerror(errno));
            return false;
        }
        return true;
    }

    // Closes a viewer that is connected and not reading: its oldest owed VITAL
    // frame is older than BEHIND_MS.
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
        // Pushed now, because the next thing that happens to this client is a
        // close.
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

    // The pilot's BOARD with the fields only this module knows: the deadman,
    // the epoch, the holder, and what serving this viewer cost.
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

    // The saved trim (publishTrim), never through the event rate limiter.
    Void emitTrim(Client& c)
    {
        bibowire::Event e;
        e.tMonoUs = monoUs();
        e.severity = bibowire::Severity::SEVERITY_INFO;
        e.code = bibowire::EVENT_CODE_TRIM;
        e.text = lastTrim;
        emit(c, bibowire::Type::TYPE_EVENT, [&e](UInt8* out, Size cap) {
            return bibowire::writeEvent(e, out, cap);
        });
    }

    // The whole list to one client, one frame per bundle.
    //
    // generation, index and count are stamped HERE rather than trusted from the
    // caller. A viewer has the list once it holds `count` frames carrying one
    // generation, so if those three could disagree it would wait for a frame
    // that never comes. An EMPTY list is still announced - one frame with count
    // 0 - because "this board runs no bundles" and "this board has not said
    // yet" are different answers and the master window shows different things.
    Void sendBundles(Client& c)
    {
        const Size n = lastBundles.size();
        if(n == 0u)
        {
            bibowire::Bundle empty;
            empty.generation = lastBundleGeneration;
            emit(c, bibowire::Type::TYPE_BUNDLE, [&empty](UInt8* out, Size cap) {
                return bibowire::writeBundle(empty, out, cap);
            });
            return;
        }
        for(Size i = 0; i < n; ++i)
        {
            bibowire::Bundle b = lastBundles[i];
            b.generation = lastBundleGeneration;
            b.index = static_cast<UInt16>(i);
            b.count = static_cast<UInt16>(n);
            emit(c, bibowire::Type::TYPE_BUNDLE, [&b](UInt8* out, Size cap) {
                return bibowire::writeBundle(b, out, cap);
            });
        }
    }

    Void sendState(Client& c)
    {
        // State before scan, always: LIDAR_INFO, BOARD and the trim, then the
        // next SCAN.
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
        if(haveTrim)
        {
            emitTrim(c);
        }
        if(haveBundles)
        {
            sendBundles(c);
        }
        if(haveBundleState)
        {
            emit(c, bibowire::Type::TYPE_BUNDLE_STATE, [](UInt8* out, Size cap) {
                return bibowire::writeBundleState(lastBundleState, out, cap);
            });
        }
    }

    Void onHello(Client& c, const bibowire::Body& body, UInt8 ver, Vec<Client>& clients)
    {
        // One handshake per socket.
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
        // Every bit accepted: a viewer with no convention sends all ones.
        c.features = h.featureMask;
        if(!bibowire::versionOk(h.protoMajor))
        {
            const bibowire::Bye m = bibowire::versionRefusal(h, policy.boardBuild.c_str());
            refuse(c, m.reason, m.text, "version");
            return;
        }
        // Every well-formed HELLO is answered, even a refused one: on this link
        // silence already means too many other things.
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
        // Counted now, not at the end of the pass: publish() checks the count
        // before it takes the lock, and this client's own first BOARD reports
        // it as `clients`.
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

    // The slot is released by LEAVE, by a close, or by
    // bibowire::CONTROL_SLOT_MS of silence.
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

    Void onControlFrame(Client& c, const bibowire::Control& m);

    // Tuning verbs change the car's trim, not its motion (section 5). This
    // thread validates, answers with a CMDACK naming the value taken, and
    // queues the request for the tick: nothing here touches the serial port, so
    // a stalled Pico cannot stall the socket.
    [[nodiscard]] Bool isTuningVerb(bibowire::Verb v)
    {
        return v == bibowire::Verb::VERB_SET_ESC_LIMITS
            || v == bibowire::Verb::VERB_SET_SERVO_LIMITS
            || v == bibowire::Verb::VERB_SET_SERVO_TRIM
            || v == bibowire::Verb::VERB_SET_SLEW
            || v == bibowire::Verb::VERB_SET_ESC_REVERSE;
    }

    // The car's arm state as the pilot last reported it through applied().
    [[nodiscard]] Bool armedNow()
    {
        LockGuard<Mutex> lock(sh.appliedM);
        return sh.applied.armed != 0u;
    }

    // The one exception to "not while armed" (bibowire::BUTTON_IDLE_TEST): ESC
    // limits, while the holder's fresh newest CONTROL carries ENABLE and
    // BUTTON_IDLE_TEST. A stale stream or a departed holder allows nothing.
    [[nodiscard]] Bool idleTestAllows(bibowire::Verb v)
    {
        if(v != bibowire::Verb::VERB_SET_ESC_LIMITS || !everControl || holderGone)
        {
            return false;
        }
        if(elapsedMs(lastControlAt) > static_cast<Float64>(bibowire::CONTROL_DEAD_MS))
        {
            return false;
        }
        const UInt32 seq = sh.ctlSeq.load(std::memory_order_acquire);
        if(seq == 0u)
        {
            return false;
        }
        const bibowire::Control& c = sh.ctlSlot[seq & 1u];
        const UInt16 both = static_cast<UInt16>(bibowire::BUTTON_IDLE_TEST | bibowire::BUTTON_ENABLE);
        return (c.buttons & both) == both;
    }

    // Positive evidence only: "has not reported yet" is not "no Pico". picoLink
    // 0 is the pilot saying the port is closed or the run is --dry.
    [[nodiscard]] Bool picoDown()
    {
        return haveBoard && lastBoard.picoLink == 0u;
    }

    // Positive evidence, like picoDown(): the pilot said it is not in MANUAL, so
    // a car program or the autonomy drives and a viewer only watches.
    [[nodiscard]] Bool notManual()
    {
        const UInt8 manual = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL);
        return haveBoard && lastBoard.pilotMode != manual;
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
                // Once, not per drop, so the line that says why is not buried.
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

    [[nodiscard]] Bool isBundleVerb(bibowire::Verb v)
    {
        return v == bibowire::Verb::VERB_LOAD_BUNDLE
            || v == bibowire::Verb::VERB_STOP_BUNDLE;
    }

    Void queueBundle(const bibowire::Command& cmd)
    {
        LockGuard<Mutex> lock(sh.bundleM);
        while(sh.bundleQ.size() >= BUNDLE_MAX)
        {
            sh.bundleQ.pop_front();
            ++sh.bundleDropped;
        }
        BundleRequest r;
        r.load = cmd.verb == bibowire::Verb::VERB_LOAD_BUNDLE;
        r.index = cmd.arg0;
        r.generation = cmd.arg1;
        sh.bundleQ.push_back(r);
    }

    // Fills `ack` for a load or an unload, and queues it when it is taken.
    //
    // NOT GATED ON notManual(), where trim and ARM are. A bundle is what the
    // board runs, so refusing a load while one drives would make the master
    // window dead on the only host that has bundles (docs/bundles.md section
    // 8). ESTOP is not consulted either: a load changes the chain, and the
    // chain can only ever REDUCE authority, so no load moves a car that a
    // latched estop is holding still.
    //
    // Reads lastBundles and lastBundleGeneration with no lock, exactly as
    // notManual() reads lastBoard: onCommand and the deliver() that writes
    // them both run on this module's one thread.
    Void onBundle(const bibowire::Command& cmd, bibowire::CmdAck* ack)
    {
        if(!haveBundles)
        {
            ack->result = 4;
            ack->text = "this board has not published its bundle list yet";
            return;
        }
        // THE GENERATION IS WHAT MAKES AN INDEX SAFE: a list replaced while a
        // viewer had its window open must not load whatever now sits there.
        // Compared as 16 bits because COMMAND's arg1 is 16 bits wide.
        if(cmd.arg1 != static_cast<UInt16>(lastBundleGeneration))
        {
            ack->result = 1;
            ack->text = "that list is out of date - the board has published a newer one";
            return;
        }
        if(static_cast<Size>(cmd.arg0) >= lastBundles.size())
        {
            ack->result = 1;
            ack->text = "no bundle at that position in the list";
            return;
        }
        const bibowire::Bundle& b = lastBundles[cmd.arg0];
        const Bool loading = cmd.verb == bibowire::Verb::VERB_LOAD_BUNDLE;
        // Ready is the board's own measurement of what this car has; a bundle
        // never declares itself loadable.
        if(loading && b.ready == 0u)
        {
            ack->result = 1;
            ack->text = b.name + " cannot run on this car - something it needs is missing";
            return;
        }
        queueBundle(cmd);
        ack->result = 0;
        ack->text = (loading ? "loading " : "unloading ") + b.name;
    }

    // Fills `ack` for one tuning verb, and queues the request when it is taken.
    Void onTune(const bibowire::Command& cmd, bibowire::CmdAck* ack)
    {
        // Mode, arm and Pico before ranges: a range is no answer from a car that
        // would refuse any number.
        if(notManual())
        {
            ack->result = 3;
            ack->text = "not in manual - trim is changed only while bibo-pilot runs in manual";
            return;
        }
        if(armedNow() && !idleTestAllows(cmd.verb))
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
                // The accepted range is named, so the refusal says what to type.
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
            // min below max, on the values the Pico is handed. Should a clamp
            // ever be added above, this test must run after it: a clamp can
            // make two accepted numbers equal.
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
                "%s limits set to %u..%u us - saved on the board and re-sent whenever the Pico connects",
                what,
                a1,
                a2
            );
            ack->result = 0;
            ack->text = Str(buf.data());
            return;
        }

        if(cmd.verb == bibowire::Verb::VERB_SET_ESC_REVERSE)
        {
            // Neutral is in range and turns reverse off. Above it is refused,
            // not clamped: nothing named reverse may produce a forward pulse.
            if(!within(cmd.arg1, bibowire::ESC_US_HARD_MIN, bibowire::ESC_NEUTRAL_US))
            {
                std::snprintf(
                    buf.data(),
                    buf.size(),
                    "the reverse limit must be %u..%u us - %u is reverse off",
                    static_cast<unsigned>(bibowire::ESC_US_HARD_MIN),
                    static_cast<unsigned>(bibowire::ESC_NEUTRAL_US),
                    static_cast<unsigned>(bibowire::ESC_NEUTRAL_US)
                );
                ack->result = 1;
                ack->text = Str(buf.data());
                return;
            }
            queueTune(cmd);
            if(cmd.arg1 == bibowire::ESC_NEUTRAL_US)
            {
                ack->text = "reverse is off - S stops at neutral and goes no further - saved on the board";
            }
            else
            {
                std::snprintf(
                    buf.data(),
                    buf.size(),
                    "reverse limit set to %u us - S brakes, then reverses down to it - saved on the board",
                    a1
                );
                ack->text = Str(buf.data());
            }
            ack->result = 0;
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
                "servo centre set to %u us - saved on the board and re-sent whenever the Pico connects",
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
            // Both units: the wire's us per tick and an operator's us per second.
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

        // Unreachable while isTuningVerb and these branches agree; answered
        // anyway, because every COMMAND gets a CMDACK.
        ack->result = 2;
        ack->text = "that is not a verb this board tunes";
    }

    // ARM (section 6). Refused unless the estop is clear, the request comes
    // from the holder, the pilot is in MANUAL, the Pico is up and answering, the
    // viewer has the current epoch, and its CONTROL stream has been live for
    // REARM_STREAM_MS; each refusal says which. This thread only decides: the
    // tick sends ESC ARM when it sees the flag rise.
    Void onArm(const Client& c, const bibowire::Command& cmd, bibowire::CmdAck* ack)
    {
        Array<Char, 160> buf{};
        if(estopLatched)
        {
            ack->result = 1;
            ack->text = "estop is latched - CLEAR_ESTOP first, then ARM";
            return;
        }
        // Result 1, not 2: the viewer labels 2 "unknown verb".
        if(!(c.holder && c.sessionId == holderSession))
        {
            ack->result = 1;
            ack->text = "you cannot arm a car you are not holding - connect as the driver";
            return;
        }
        // The reverse of picoDown(): arming before the pilot has reported would
        // send an arm into a port nobody has seen open.
        if(!haveBoard)
        {
            ack->result = 4;
            ack->text = "the pilot has not reported the car yet - ARM again in a moment";
            return;
        }
        if(notManual())
        {
            ack->result = 3;
            ack->text = "this pilot is not in manual - a viewer arms only a car it is driving";
            return;
        }
        if(lastBoard.picoLink != 1u)
        {
            ack->result = 4;
            ack->text = lastBoard.picoLink == 0u
                ? "no Pico link - the board will not send an arm into a closed port"
                : "the Pico is not answering - the board will not arm a car it cannot hear";
            return;
        }
        if(cmd.armEpoch != static_cast<UInt8>(armEpoch))
        {
            std::snprintf(
                buf.data(),
                buf.size(),
                "the arm epoch is %u and this ARM was sent under %u - something disarmed the car; ARM again",
                static_cast<unsigned>(armEpoch),
                static_cast<unsigned>(cmd.armEpoch)
            );
            ack->result = 1;
            ack->text = Str(buf.data());
            return;
        }
        // The stream must be live now, not merely once.
        const Bool streaming = everControl && !holderGone
            && elapsedMs(lastControlAt) <= static_cast<Float64>(bibowire::CONTROL_STALE_MS);
        const Int32 liveMs = streaming ? static_cast<Int32>(elapsedMs(streamSince)) : 0;
        if(liveMs < bibowire::REARM_STREAM_MS)
        {
            std::snprintf(
                buf.data(),
                buf.size(),
                "your CONTROL stream has been live %d ms and arming needs %d - hold the slot and let it run",
                liveMs,
                bibowire::REARM_STREAM_MS
            );
            ack->result = 1;
            ack->text = Str(buf.data());
            return;
        }
        sh.operatorArmed.store(true, std::memory_order_release);
        std::snprintf(
            buf.data(),
            buf.size(),
            "armed under epoch %u - an estop, the deadman, a lost Pico link or leaving the slot disarms",
            static_cast<unsigned>(armEpoch)
        );
        ack->result = 0;
        ack->text = Str(buf.data());
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
        else if(cmd.verb == bibowire::Verb::VERB_ARM)
        {
            onArm(c, cmd, &ack);
        }
        else if(cmd.verb == bibowire::Verb::VERB_DISARM)
        {
            // Always succeeds, from any session: making the car safer is not a
            // privilege. The tick sends ESC DISARM when it sees the flag fall.
            bumpEpoch();
            if(notManual())
            {
                // A car program arms the car itself, so the epoch never reaches
                // it. The estop latch does: the program sends STOP and ends.
                estopLatched = true;
                ack.result = 0;
                ack.text = "estop latched - outside manual, DISARM stops the car as ESTOP does";
            }
            else if(picoDown())
            {
                ack.result = 4;
                ack.text = "disarmed locally; the Pico did not answer, its own watchdog"
                           " will stop the car";
            }
            else
            {
                ack.result = 0;
                ack.text = "disarmed - ARM again to drive";
            }
        }
        else if(cmd.verb == bibowire::Verb::VERB_ESTOP)
        {
            // Two estop paths, and either LATCHES: this one over TCP,
            // acknowledged; CONTROL's button bit over UDP, with no round trip.
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
            onTune(cmd, &ack);
        }
        else if(isBundleVerb(cmd.verb))
        {
            onBundle(cmd, &ack);
        }
        else
        {
            // MOTOR and SET_MODE are refused, never answered with an OK nothing
            // acted on.
            ack.result = 3;
            ack.text = "this board does not act on that verb yet";
        }
        ack.armEpoch = static_cast<UInt8>(armEpoch);
        // Before the printf below, which can block on a stalled console.
        shareDrive();
        if(linkLog)
        {
            // Every command and its answer, so a refusal outlives the viewer's
            // window.
            std::printf(
                "viewfeed: cmd %s id=%u verb=%u args=%u,%u,%u epoch=%u -> result=%u epoch=%u \"%s\"\n",
                c.peer.c_str(),
                static_cast<unsigned>(cmd.cmdId),
                static_cast<unsigned>(cmd.verb),
                static_cast<unsigned>(cmd.arg0),
                static_cast<unsigned>(cmd.arg1),
                static_cast<unsigned>(cmd.arg2),
                static_cast<unsigned>(cmd.armEpoch),
                static_cast<unsigned>(ack.result),
                static_cast<unsigned>(ack.armEpoch),
                ack.text.c_str()
            );
        }
        emit(c, bibowire::Type::TYPE_CMDACK, [&ack](UInt8* out, Size cap) {
            return bibowire::writeCmdAck(ack, out, cap);
        });
    }

    Void onFrame(Client& c, const bibowire::Frame& f, Vec<Client>& clients)
    {
        note(Dir::DIR_IN, f.head, f.body.len, NOTE_OK);
        switch(f.head.type)
        {
        case bibowire::Type::TYPE_PING:
            ++c.winPing;
            break;
        case bibowire::Type::TYPE_PONG:
            ++c.winPong;
            break;
        case bibowire::Type::TYPE_CONTROL:
            ++c.winControlTcp;
            break;
        case bibowire::Type::TYPE_COMMAND:
            ++c.winCommand;
            break;
        default:
            break;
        }
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
            // The token is echoed verbatim: it is the viewer's only correlator
            // for its round-trip time. senderMonoUs is replaced, because the
            // board is this frame's sender (section 7's clock-offset rule).
            m.senderMonoUs = monoUs();
            // Answered in this pass, so the round trip measures the link and
            // not this board's queue.
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
                // Clearing pingOut also resumes a held camera (CAM_HOLD_PONG_MS).
                c.lastRttMs = static_cast<Int64>(elapsedMs(c.pingSentAt));
                ++c.winPongMatched;
                c.pingOut = false;
            }
            break;
        }
        case bibowire::Type::TYPE_CONTROL:
        {
            // Accepted on TCP always, under identical rules: the same frame on
            // another socket.
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
                // Clamped, never refused (bibowire::CAM_FPS_MAX).
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
            // An unknown type was skipped by its `len`, so an older board keeps
            // serving a newer viewer.
            break;
        }
    }

    // The one refusal in plain text, for a person connecting by hand.
    Void wrongService(Client& c)
    {
        const Str line = "ERR bibowire v1 binary on 8020; connect with the bibo viewer\n";
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
            // MAX_INBOUND_PAYLOAD, checked on the HEADER: a claim larger than
            // the ring would leave take() answering NEED_MORE forever. Nothing
            // is allocated for the claim.
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
                // A full ring with no whole frame is not this protocol.
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
            // Handled FIRST, compacted SECOND: f.body points into c.in, and the
            // memmove would overwrite it with the next frame from the same read.
            if(t == bibowire::Take::TAKE_FRAME)
            {
                onFrame(c, f, clients);
            }
            if(used > 0u && used <= c.inLen)
            {
                if(t == bibowire::Take::TAKE_RESYNC)
                {
                    // Counted: uncounted junk is a fault nobody discovers.
                    bibowire::Head h;
                    note(Dir::DIR_IN, h, used, NOTE_JUNK);
                    LockGuard<Mutex> lock(sh.tallyM);
                    sh.tally.resyncBytes += used;
                }
                std::memmove(c.in.data(), c.in.data() + used, c.inLen - used);
                c.inLen -= used;
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
        c.winRxBytes += static_cast<UInt64>(n);
        // An unset TimePoint is the clock's epoch, so the first read has no gap.
        const Float64 gap = c.lastRxAt == TimePoint() ? 0.0 : elapsedMs(c.lastRxAt);
        if(gap > c.winRxGapMs)
        {
            c.winRxGapMs = gap;
        }
        c.lastRxAt = monoNow();
        consume(c, clients);
    }

    // CONTROL, from either transport.
    Void onControlFrame(Client& c, const bibowire::Control& m)
    {
        {
            LockGuard<Mutex> lock(sh.tallyM);
            ++sh.tally.rxControl;
        }
        ++c.datagrams;
        ++c.winControl;
        bibowire::control::Gate g;
        g.sessionId = c.sessionId;
        g.highestSeq = appliedSeq;
        g.haveHolder = haveHolder;
        g.fromHolder = c.holder && c.sessionId == holderSession;
        g.armEpoch = static_cast<UInt8>(armEpoch);
        // The BOARD's mode, not the datagram's: comparing the datagram with
        // itself could never refuse. Before the first BOARD an unknown mode
        // agrees.
        g.pilotMode = haveBoard ? lastBoard.pilotMode : m.assumedMode;
        const bibowire::control::Outcome o = bibowire::control::apply(g, m);
        if(o.verdict != bibowire::control::Verdict::VERDICT_APPLIED)
        {
            // Counted and discarded, and it does NOT feed the timer.
            LockGuard<Mutex> lock(sh.tallyM);
            ++sh.tally.rxControlStale;
            return;
        }
        // The ESTOP bit needs no round trip, and latches like its TCP twin.
        if((m.buttons & bibowire::BUTTON_ESTOP) != 0u && !estopLatched)
        {
            estopLatched = true;
            bumpEpoch();
            shareDrive();
        }
        appliedSeq = o.highestSeq;
        // When this stream began, for ARM's REARM_STREAM_MS. A gap the deadman
        // calls dead starts a new stream, which has to earn it again.
        const Bool deadGap = everControl
            && elapsedMs(lastControlAt) > static_cast<Float64>(bibowire::CONTROL_DEAD_MS);
        if(!everControl || holderGone || deadGap)
        {
            streamSince = monoNow();
        }
        // That gap disarms HERE, before lastControlAt moves. The loop's check
        // runs after the reads, so a datagram after a dead gap would make the
        // deadman LIVE before the loop saw it DEAD, and the tick would re-arm a
        // car off a stall - which must never happen on its own (section 6).
        if(deadGap && sh.operatorArmed.load(std::memory_order_acquire))
        {
            bumpEpoch();
        }
        lastControlAt = monoNow();
        everControl = true;
        holderGone = false;
        // What is stored is what the gate ALLOWED, not what arrived, so the
        // ungated throttle is unreachable from control().
        bibowire::Control gated = m;
        gated.steerMilli = o.steerMilli;
        gated.throttleMilli = o.throttleMilli;
        // Slot first, then seq (Shared::ctlSeq).
        const UInt32 slot = o.highestSeq & 1u;
        sh.ctlSlot[slot] = gated;
        sh.ctlSeq.store(o.highestSeq, std::memory_order_release);
    }

    Void serveUdp(Vec<Client>& clients)
    {
        // Drained to empty every pass.
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
            // A datagram from before a reconnect has no live session to own it.
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
            // Where CTLSTATE goes, learned from the datagram when HELLO gave no
            // port: a viewer behind a NAT is reachable where its packets came
            // from.
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
        // The board's own measured ages, which catch a perfect link carrying
        // dead data. None may read 0 when unknown, because 0 means perfectly
        // fresh: scanAgeMs has no ABSENT sentinel, so before any revolution it
        // is how long the feed has been up.
        s.scanAgeMs = haveScan
            ? static_cast<UInt32>(elapsedMs(lastScanAt))
            : static_cast<UInt32>(elapsedMs(startedAt));
        // The pilot's measurement from BOARD, not a second one that could
        // disagree; the ABSENT sentinel, never 0, before there is one.
        s.picoSilentMs = haveBoard ? lastBoard.picoSilentMs : a.picoSilentMs;
        s.pilotMode = haveBoard ? lastBoard.pilotMode : a.pilotMode;
        s.lastCmdId = a.lastCmdId;
        // The reverse-path probe's verdict, on the wire and not only in a log.
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
        // Each datagram is its own frame, so the seq advances and the header
        // ring can order itself.
        ++c.txSeq;
        // Mirrored onto TCP while the viewer falls back to TCP control, as LIVE
        // rather than VITAL: an undroppable 20 Hz stream would turn a stall into
        // a closed connection, and only the newest CTLSTATE matters.
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

    // The camera, the only BULK producer. One long-lived v4l2-ctl streams
    // MJPEG into a pipe and this module finds the frame boundaries; nothing
    // re-encodes, and the board has nothing to re-encode with. The device is
    // single-opener, and a second opener's open() succeeds and fails later at
    // buffer setup, so a busy device cannot be probed for: whichever capture
    // loses has to say so.
    [[nodiscard]] Str envOr(CharSeq name, const Str& fallback)
    {
        const Char* v = std::getenv(name);
        return (v != nullptr && v[0] != '\0') ? Str(v) : fallback;
    }

    Void readCamCfg()
    {
        camCfg = CamCfg();
        camCfg.devOverride = envOr("BIBO_CAM_DEV", "");
        // Seeded so the "absent" sentence has a name before the first open.
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
            // 0 is UNCAPPED, on purpose; a negative or unreadable value keeps
            // the default.
            if(v >= 0.0 && v <= 240.0)
            {
                camCfg.periodMs = v > 0.0 ? 1000.0 / v : 0.0;
            }
        }
    }

    // The picture's OWN dimensions, off its SOF marker: v4l2-ctl negotiates the
    // format and a device may deliver other than what was asked, so the
    // request is only the fallback.
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

    [[nodiscard]] Bool anyClientWantsCamera(const Vec<Client>& clients)
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

    // A viewer, or the local subscriber: either keeps the device open.
    [[nodiscard]] Bool anyWantsCamera(const Vec<Client>& clients)
    {
        return sh.camLocal.load() || anyClientWantsCamera(clients);
    }

    // How often a picture is offered: the fastest subscriber's rate, and every
    // subscriber gets every offered frame. Pacing viewers separately would
    // leave frameIndex gaps that a viewer reports as missing pictures the board
    // withheld on purpose. Safe only because CAMERA is CLASS_BULK. 0 is
    // uncapped and wins, never "slowest".
    [[nodiscard]] Float64 offeredCamPeriodMs(const Vec<Client>& clients)
    {
        Bool asked = false;
        Float64 best = 0.0;
        if(sh.camLocal.load())
        {
            const UInt16 fps = sh.camLocalFps.load();
            asked = true;
            best = fps == 0u ? camCfg.periodMs : 1000.0 / static_cast<Float64>(fps);
        }
        for(const Client& c : clients)
        {
            if(!wants(c, bibowire::Type::TYPE_CAMERA))
            {
                continue;
            }
            // A subscriber that named no rate enters with the board's default.
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

    // Said to the camera's subscribers, who are looking at the blank panel.
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

    // Why the capture ended, in words. A v4l2-ctl that loses the device usually
    // writes nothing at all and exits non-zero, so the EXIT STATUS, not the
    // stderr text, is the busy signal.
    [[nodiscard]] Str cameraWhy(const Str& diag, Int32 status)
    {
        const Bool exited = status >= 0 && WIFEXITED(status);
        const Int32 code = exited ? static_cast<Int32>(WEXITSTATUS(status)) : -1;
        if(code == 127)
        {
            // The child's own "exec failed" exit, from startCamera below.
            return "cannot start v4l2-ctl - it is not installed on this board";
        }
        if(code > 0 || diag.find("busy") != Str::npos || diag.find("Busy") != Str::npos)
        {
            // The only other openers on this board: a second pilot, or the
            // v4l2-ctl a killed one left behind (a child outlives its parent).
            return "camera stream ended - is something else holding " + camCfg.dev
                 + "? a second pilot, or a v4l2-ctl left running by one that died";
        }
        if(!diag.empty())
        {
            // One line: EVENT carries a sentence.
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

    Void backOff()
    {
        cam.fails = cam.fails + 1 > CAM_FAIL_CEILING ? CAM_FAIL_CEILING : cam.fails + 1;
        const Float64 wait = CAM_RETRY_STEP_MS * static_cast<Float64>(1 + cam.fails);
        cam.waitMs = wait > CAM_RETRY_MAX_MS ? CAM_RETRY_MAX_MS : wait;
        cam.failedAt = monoNow();
        cam.waiting = true;
    }

    // Ends the capture and returns the child's exit status, or -1.
    //
    // SIGKILL, not SIGTERM: v4l2-ctl has nothing to flush, and waiting on a
    // polite exit would stall the CTLSTATE clock. An already-exited child
    // keeps its status until reaped. And it is always reaped: every zombie
    // holds a process-table slot, and a board that cannot fork cannot even
    // answer ssh.
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
        // Released, not cleared: clear() keeps the capacity, and an unwatched
        // camera must cost nothing.
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
        // The cap is applied here, not at the device, which may silently ignore
        // a lower rate. The rate is the subscribers': the viewer knows its link.
        const Float64 offerMs = offeredCamPeriodMs(clients);
        if(cam.everSent && offerMs > 0.0 && elapsedMs(cam.lastSentAt) < offerMs)
        {
            return;
        }
        if(len == 0u || len > CAM_MAX_JPEG)
        {
            return;
        }
        UInt16 w = camCfg.width;
        UInt16 h = camCfg.height;
        static_cast<Void>(jpegSize(jpeg, len, &w, &h));
        const UInt64 tUs = monoUs();
        Bool any = false;
        // The local subscriber first, under the same frameIndex a viewer
        // gets, so a detection on these bytes names the picture it was made on.
        if(sh.camLocal.load())
        {
            {
                LockGuard<Mutex> lock(sh.camM);
                sh.camLatest.tMonoUs = tUs;
                sh.camLatest.frameIndex = cam.frameIndex;
                sh.camLatest.width = w;
                sh.camLatest.height = h;
                sh.camLatest.jpeg.assign(jpeg, jpeg + len);
                ++sh.camSeq;
            }
            sh.camCv.notify_all();
            any = true;
        }
        // Encoded once for every viewer, and not at all when none watches:
        // the detector alone must not cost the feed a copy and a CRC per frame.
        Size bodyLen = 0;
        if(anyClientWantsCamera(clients))
        {
            const Size need = bibowire::FRAME_OVERHEAD + CAM_BODY_OVERHEAD + len;
            if(cam.encode.size() < need)
            {
                cam.encode.resize(need);
            }
            bibowire::Camera m;
            m.tMonoUs = tUs;
            m.frameIndex = cam.frameIndex;
            m.width = w;
            m.height = h;
            m.codec = 1;
            m.flags = 0;
            m.data.assign(jpeg, jpeg + len);
            const TimePoint before = monoNow();
            bodyLen = bibowire::writeCamera(
                m,
                cam.encode.data() + bibowire::HEAD_BYTES,
                cam.encode.size() - bibowire::HEAD_BYTES
            );
            countEncode(elapsedMs(before) * 1000000.0);
        }
        for(Client& c : clients)
        {
            if(bodyLen == 0u || !wants(c, bibowire::Type::TYPE_CAMERA))
            {
                continue;
            }
            // Paced by the keepalive. Behind Tailscale's userspace networking,
            // the only mode this board allows, the socket never stops taking
            // bytes, so the ring never fills and pictures queue inside the proxy
            // with the PING behind them until the viewer is dropped. While this
            // client's PING is overdue it gets no new pictures; the backlog
            // drains, the PONG lands, and the camera resumes.
            if(c.pingOut && elapsedMs(c.pingSentAt) > static_cast<Float64>(CAM_HOLD_PONG_MS))
            {
                ++c.camHeld;
                if(!c.camHoldSaid)
                {
                    c.camHoldSaid = true;
                    std::printf(
                        "viewfeed: camera to %s held while its PING is over %lld ms late - "
                        "the path is slower than the picture; it resumes when the PONG lands\n",
                        c.peer.c_str(),
                        static_cast<long long>(CAM_HOLD_PONG_MS)
                    );
                }
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
            // Monotonic, never rewound across a capture restart: a jump reads as
            // missed frames, going backwards as old pictures labelled new.
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
        // Reaped here: the exit status says why, and closeCamera discards it.
        const Int32 status = killCamera();
        // Explained once per failure episode, and only when no picture came: a
        // capture that delivered and then ended is left to the backoff.
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
        // stderr first: when stdout stays empty, it holds the answer.
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
        // A frame is whole once the next one has begun (CAM_SOI).
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
        // Re-resolved on every attempt (CamCfg::dev).
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
        // Built BEFORE the fork: nothing may allocate between fork and exec.
        // Str::data() is non-const in C++20, so execvp needs no cast.
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
            // Between fork and exec, only async-signal-safe calls: a lock another
            // thread held at the fork is held forever in this child. Every
            // other descriptor was opened CLOEXEC, so the capture cannot keep
            // port 8020 bound after the pilot exits.
            if(::dup2(outPipe[1], STDOUT_FILENO) >= 0 && ::dup2(errPipe[1], STDERR_FILENO) >= 0)
            {
                ::execvp("v4l2-ctl", argv.data());
            }
            // Only when exec failed; cameraWhy reads the 127.
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

    // Opened while at least one viewer subscribes to CAMERA and released when
    // the last one stops: an unwatched camera costs no process, pipe or buffer,
    // and leaves the device free.
    Void tendCamera(Vec<Client>& clients)
    {
        if(!anyWantsCamera(clients))
        {
            if(cam.pid >= 0 || cam.outFd >= 0)
            {
                closeCamera("the last subscriber went");
            }
            // A fresh slate, with no backoff left from an old failure.
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

    // What the pilot published.
    Void deliver(Vec<Client>& clients, const Item& item)
    {
        switch(item.what)
        {
        case What::WHAT_SCAN:
            // Stamped before the staleness rule can drop the frame: a dropped
            // revolution is still one the sensor produced.
            lastScanAt = item.at;
            haveScan = true;
            // LIVE_STALE_MS, measured on the thread that would send it.
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
                // Per client, stamped from what has accumulated so far, and NOT
                // cleared here: flush() clears it once this frame has gone.
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
            // A Pico link that went down moves the epoch, which disarms: the
            // Pico that comes back is disarmed on its own side.
            if(haveBoard && lastBoard.picoLink != 0u && item.board.picoLink == 0u)
            {
                bumpEpoch();
            }
            lastBoard = item.board;
            haveBoard = true;
            // The rate is this module's; the content is the pilot's one struct,
            // so the console and the viewer agree.
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
        case What::WHAT_BUNDLES:
            lastBundles = item.bundles;
            lastBundleGeneration = item.bundleGeneration;
            haveBundles = true;
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_BUNDLE))
                {
                    sendBundles(c);
                }
            }
            break;
        case What::WHAT_TAGS:
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_TAGS))
                {
                    emit(c, bibowire::Type::TYPE_TAGS, [&item](UInt8* out, Size cap) {
                        return bibowire::writeTags(item.tags, out, cap);
                    });
                }
            }
            break;
        case What::WHAT_ODOM:
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_ODOM))
                {
                    emit(c, bibowire::Type::TYPE_ODOM, [&item](UInt8* out, Size cap) {
                        return bibowire::writeOdom(item.odom, out, cap);
                    });
                }
            }
            break;
        case What::WHAT_POSE:
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_POSE))
                {
                    emit(c, bibowire::Type::TYPE_POSE, [&item](UInt8* out, Size cap) {
                        return bibowire::writePose(item.pose, out, cap);
                    });
                }
            }
            break;
        case What::WHAT_BUNDLE_STATE:
            lastBundleState = item.bundleState;
            haveBundleState = true;
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_BUNDLE_STATE))
                {
                    emit(c, bibowire::Type::TYPE_BUNDLE_STATE, [](UInt8* out, Size cap) {
                        return bibowire::writeBundleState(lastBundleState, out, cap);
                    });
                }
            }
            break;
        case What::WHAT_EVENT:
        {
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
        case What::WHAT_TRIM:
            lastTrim = item.event.text;
            haveTrim = true;
            for(Client& c : clients)
            {
                if(wants(c, bibowire::Type::TYPE_EVENT))
                {
                    emitTrim(c);
                }
            }
            break;
        }
    }

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
            if(c.camHeld > 0u)
            {
                std::printf(
                    "viewfeed: %s had %llu camera frames held for a late PONG\n",
                    c.peer.c_str(),
                    static_cast<unsigned long long>(c.camHeld)
                );
            }
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
            // No Nagle: a small CMDACK must not wait behind the SCAN it follows.
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
            // The kernel's send buffer must be small or the ring is not the
            // bound: with the default, send() keeps succeeding and revolutions
            // pile up in the kernel, out of reach of the drop classes and
            // BEHIND_MS. 32 KiB is about six revolutions.
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

    // PING every PING_EVERY_MS; BYE(TIMEOUT) after PONG_WAIT_MS without a PONG.
    // The deadman stopped the car long before; this closes a dead socket.
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

    // The reverse-path probe (bibowire::REVERSE_PROBE_MS): the failure a viewer
    // cannot diagnose on its own.
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

    // The link log: one line a second per viewer, on unless BIBO_LINK_LOG=0.
    // Left to right it is the round trip: what came in, what went out and what
    // is stuck on this side, the keepalive, and what the deadman made of it.
    //
    //   rx        bytes read this second, frames by type, and quiet = the longest
    //             wait between two reads that returned bytes
    //   tx        bytes send() actually TOOK, calls, and calls that would block
    //   queued    this process's ring for the client, and its oldest frame's age
    //   kernel    TIOCOUTQ - bytes the kernel holds unacknowledged. Behind a
    //             userspace proxy this stays near 0 however stuck the path is,
    //             which is itself the diagnosis
    //   ping-out  how long the board's PING has been unanswered (-1: none out)
    //   rtt       the last PING's round trip
    //   loop      the longest pass of this thread in the previous second
    //
    // The viewer writes the other half with UTC timestamps, and journalctl's
    // are UTC as well, so the two logs line up by time.
    [[nodiscard]] Bool logLink(Client& c)
    {
        if(!linkLog || c.stage != Stage::STAGE_LIVE || !c.dropWhy.empty())
        {
            return false;
        }
        if(elapsedMs(c.linkLogAt) < 1000.0)
        {
            return false;
        }
        c.linkLogAt = monoNow();
        Int32 kernel = -1;
        if(::ioctl(c.fd, TIOCOUTQ, &kernel) != 0)
        {
            kernel = -1;
        }
        const Int64 oldestMs = c.out.empty() ? 0 : static_cast<Int64>(elapsedMs(c.out.front().at));
        const Int64 pingOutMs = c.pingOut ? static_cast<Int64>(elapsedMs(c.pingSentAt)) : -1;
        const bibowire::deadman::Output d = deadmanNow();
        // quiet only closes when bytes arrive; lastrx is measured now.
        const Int64 lastRxMs = c.lastRxAt == TimePoint() ? -1 : static_cast<Int64>(elapsedMs(c.lastRxAt));
        std::printf(
            "viewfeed: link %s s=%08x %s rx=%lluB lastrx=%lldms ping=%u pong=%u/%u ctl=%u(tcp %u) cmd=%u quiet=%lldms"
            " | tx=%lluB sends=%u eagain=%u queued=%lluB/%lluf oldest=%lldms kernel=%dB"
            " | ping-out=%lldms rtt=%lldms | deadman=%s refuse=%s epoch=%u armed=%d estop=%d"
            " camheld=%llu loop=%lldms\n",
            c.peer.c_str(),
            static_cast<unsigned>(c.sessionId),
            c.holder ? "driver" : "observer",
            static_cast<unsigned long long>(c.winRxBytes),
            static_cast<long long>(lastRxMs),
            static_cast<unsigned>(c.winPing),
            static_cast<unsigned>(c.winPongMatched),
            static_cast<unsigned>(c.winPong),
            static_cast<unsigned>(c.winControl),
            static_cast<unsigned>(c.winControlTcp),
            static_cast<unsigned>(c.winCommand),
            static_cast<long long>(c.winRxGapMs),
            static_cast<unsigned long long>(c.winTxBytes),
            static_cast<unsigned>(c.winSends),
            static_cast<unsigned>(c.winEagain),
            static_cast<unsigned long long>(c.outBytes),
            static_cast<unsigned long long>(c.out.size()),
            static_cast<long long>(oldestMs),
            static_cast<int>(kernel),
            static_cast<long long>(pingOutMs),
            static_cast<long long>(c.lastRttMs),
            bibowire::deadman::stateName(d.state),
            bibowire::refuseName(d.refuse),
            static_cast<unsigned>(armEpoch),
            sh.operatorArmed.load(std::memory_order_acquire) ? 1 : 0,
            estopLatched ? 1 : 0,
            static_cast<unsigned long long>(c.camHeld),
            static_cast<long long>(loopWorstShown)
        );
        c.winRxGapMs = 0.0;
        c.winRxBytes = 0;
        c.winTxBytes = 0;
        c.winSends = 0;
        c.winEagain = 0;
        c.winPing = 0;
        c.winPong = 0;
        c.winPongMatched = 0;
        c.winControl = 0;
        c.winControlTcp = 0;
        c.winCommand = 0;
        return true;
    }

    Void loop()
    {
        Vec<Client> clients;
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
            // After the clients, so the c.at indices stand. Its revents are
            // never read: it only wakes the loop, and tendCamera drains the
            // pipe every pass.
            if(cam.outFd >= 0)
            {
                fds.push_back(pollfd{ cam.outFd, POLLIN, 0 });
            }
            if(::poll(fds.data(), fds.size(), POLL_MS) < 0 && errno != EINTR)
            {
                std::printf("viewfeed: poll failed: %s\n", std::strerror(errno));
                break;
            }
            // The pass's work, timed from after poll() so the sleep is not a
            // stall, and kept per second for logLink.
            const TimePoint workStart = monoNow();
            if(elapsedMs(loopWindowAt) >= 1000.0)
            {
                loopWorstShown = loopWorstMs;
                loopWorstMs = 0.0;
                loopWindowAt = workStart;
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
            // Client sockets before the published items, so a HELLO from this
            // pass is welcomed before this pass's revolution goes out. Only the
            // clients from before accept() have a pollfd.
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
            // After the reads, so a SUBSCRIBE opens the device this pass, and
            // before the flush, so a frame read this pass is sent this pass.
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
            // The deadman tripping disarms. The tick has already sent STOP, so
            // the operator ARMs again rather than finding throttle back the
            // moment the stream resumes.
            if(sh.operatorArmed.load(std::memory_order_acquire) && deadmanByte() >= 2u)
            {
                bumpEpoch();
            }
            // Before the flush, so an answer a viewer reads is already what
            // drive() reports.
            shareDrive();
            for(Client& c : clients)
            {
                if(!c.dropWhy.empty())
                {
                    continue;
                }
                keepLive(c);
                probeReversePath(c);
                const Float64 worked = elapsedMs(workStart);
                if(worked > loopWorstMs)
                {
                    loopWorstMs = worked;
                }
                static_cast<Void>(logLink(c));
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
      // The same port for both, so a viewer knows where control goes.
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
          // A previous run's tuning requests are intents that have expired.
          LockGuard<Mutex> lock(sh.tuneM);
          sh.tunes.clear();
          sh.tuneDropped = 0;
          sh.tuneDropSaid = false;
      }
      {
          // A previous run's loads are expired intents in the same way.
          LockGuard<Mutex> lock(sh.bundleM);
          sh.bundleQ.clear();
          sh.bundleDropped = 0;
      }
      sh.count.store(0);
      sh.ctlSeq.store(0, std::memory_order_release);
      armEpoch = 0;
      estopLatched = false;
      haveHolder = false;
      {
          LockGuard<Mutex> lock(sh.driveM);
          sh.driveSeen = DriveSeen();
          sh.driveSeen.at = monoNow();
      }
      // A new start is a new car: no ARM survives from the previous run.
      sh.operatorArmed.store(false, std::memory_order_release);
      linkLog = envOr("BIBO_LINK_LOG", "1") != "0";
      holderSession = 0;
      everControl = false;
      appliedSeq = 0;
      holderGone = false;
      haveBoard = false;
      haveLidar = false;
      haveTrim = false;
      lastTrim.clear();
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
      // Nobody connected: not even the lock.
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
      // NOT gated on a client: the next viewer is owed the newest BOARD.
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

  Void publishBundles(UInt32 generation, Vec<bibowire::Bundle> list)
  {
      // NOT gated on a client, like the board state: the next viewer is owed
      // the newest list, and it is what its master window is made of.
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_BUNDLES;
      item.bundles = std::move(list);
      item.bundleGeneration = generation;
      item.at = monoNow();
      post(std::move(item));
  }

  Void publishBundleState(const bibowire::BundleState& s)
  {
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_BUNDLE_STATE;
      item.bundleState = s;
      item.at = monoNow();
      post(std::move(item));
  }

  Void publishTags(const bibowire::Tags& t)
  {
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_TAGS;
      item.tags = t;
      item.at = monoNow();
      post(std::move(item));
  }

  Void publishOdom(const bibowire::Odom& m)
  {
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_ODOM;
      item.odom = m;
      item.at = monoNow();
      post(std::move(item));
  }

  Void publishPose(const bibowire::Pose& m)
  {
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_POSE;
      item.pose = m;
      item.at = monoNow();
      post(std::move(item));
  }

  Void wantCamera(Bool on, UInt16 fps)
  {
      sh.camLocalFps.store(fps);
      sh.camLocal.store(on);
      // Woken so tendCamera opens or closes the device this pass rather
      // than on the next timer or packet.
      if(running)
      {
          wakeLoop();
      }
  }

  Bool waitCameraFrame(UInt32* seen, CameraFrame* out, Int32 waitMs)
  {
      if(seen == nullptr || out == nullptr)
      {
          return false;
      }
      UniqueLock<Mutex> lock(sh.camM);
      if(sh.camSeq == *seen)
      {
          const UInt32 was = *seen;
          static_cast<Void>(sh.camCv.wait_for(lock, Millis(waitMs > 0 ? waitMs : 0), [was] {
              return sh.camSeq != was;
          }));
      }
      if(sh.camSeq == *seen)
      {
          return false;
      }
      *seen = sh.camSeq;
      *out = sh.camLatest;
      return true;
  }

  Bool cameraPresent()
  {
      const Str dev = camCfg.devOverride.empty() ? cameraDevDefault() : camCfg.devOverride;
      return ::access(dev.c_str(), F_OK) == 0;
  }

  Void publishTrim(const Str& report)
  {
      if(!running)
      {
          return;
      }
      // Cut at whole lines, never mid-number: "SLEW THROTTLE 2" cut from 200
      // would read as a setting.
      Str text = report;
      while(text.size() > bibowire::MAX_EVENT_TEXT)
      {
          const Size cut = text.rfind("; ");
          text = cut == Str::npos ? Str() : text.substr(0, cut);
      }
      Item item;
      item.what = What::WHAT_TRIM;
      item.event.text = text;
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
      // Eight writes during one read is no stream this protocol produces; no
      // command beats half of one.
      return false;
  }

  Drive drive()
  {
      Drive d;
      if(!running)
      {
          // haveHolder false: the pilot runs under its own blind and silence
          // rules.
          return d;
      }
      DriveSeen seen;
      {
          LockGuard<Mutex> lock(sh.driveM);
          seen = sh.driveSeen;
      }
      const bibowire::deadman::Output o = deadmanOf(seen);
      d.seenAgeMs = static_cast<Int32>(std::min(elapsedMs(seen.at), 1.0e6));
      d.haveHolder = seen.haveHolder;
      d.estopLatched = seen.estopLatched;
      d.deadman = deadmanByteOf(o);
      d.refuse = o.refuse;
      d.neutralInMs = o.neutralInMs;
      d.disarmInMs = o.disarmInMs;
      d.armed = sh.operatorArmed.load(std::memory_order_acquire);
      return d;
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
      // Oldest first, in the order the operator acted.
      *out = sh.tunes.front();
      sh.tunes.pop_front();
      return true;
  }

  Bool bundleRequest(BundleRequest* out)
  {
      if(out == nullptr || !running)
      {
          return false;
      }
      LockGuard<Mutex> lock(sh.bundleM);
      if(sh.bundleQ.empty())
      {
          return false;
      }
      *out = sh.bundleQ.front();
      sh.bundleQ.pop_front();
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
      // The capture is a child process and would outlive this one, leaving the
      // device busy for the next pilot. Safe here: the thread that owns `cam`
      // has been joined.
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

// No Linux sockets: start() refuses, saying so, and every other call does
// nothing, as with no viewer connected.
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

  Void publishTrim(const Str& report)
  {
      static_cast<Void>(report);
  }

  Void publishEvent(bibowire::Severity severity, UInt8 code, const Str& text)
  {
      static_cast<Void>(severity);
      static_cast<Void>(code);
      static_cast<Void>(text);
  }

  Void publishBundles(UInt32 generation, Vec<bibowire::Bundle> list)
  {
      static_cast<Void>(generation);
      static_cast<Void>(list);
  }

  Void publishBundleState(const bibowire::BundleState& s)
  {
      static_cast<Void>(s);
  }

  Void publishTags(const bibowire::Tags& t)
  {
      static_cast<Void>(t);
  }

  Void publishOdom(const bibowire::Odom& m)
  {
      static_cast<Void>(m);
  }

  Void publishPose(const bibowire::Pose& m)
  {
      static_cast<Void>(m);
  }

  Void wantCamera(Bool on, UInt16 fps)
  {
      static_cast<Void>(on);
      static_cast<Void>(fps);
  }

  Bool waitCameraFrame(UInt32* seen, CameraFrame* out, Int32 waitMs)
  {
      static_cast<Void>(seen);
      static_cast<Void>(out);
      static_cast<Void>(waitMs);
      return false;
  }

  Bool cameraPresent()
  {
      return false;
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

  Bool bundleRequest(BundleRequest* out)
  {
      static_cast<Void>(out);
      return false;
  }

  // haveHolder false: with no holder the deadman does not apply and the pilot
  // runs under its own blind and silence rules.
  Drive drive()
  {
      return Drive();
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
