// The viewer's half of bibowire: one TCP connection to the pilot on the Orange
// Pi, decoded on its own thread, handed to the frame loop newest-wins.
//
// Framing, CRC and message bodies all come from firmware/pilot/src/bibowire.cxx,
// the same object file the board compiles (docs/bibowire.md section 12), so the
// two ends cannot disagree about a field while both still compile. This module
// owns sockets, a thread, a reconnect schedule and staleness.
//
// Session is pure: bytes and a millisecond in, decoded state out, so the decode
// path is tested against hand-built frames (viewer/tests/test_link.cxx). Client
// is the socket half; it owns a Session and publishes copies under a lock.
//
// The Session accessors take the caller's clock and return an empty Opt when the
// value is too old to draw. There is deliberately no way to reach the data
// without naming a time: a staleness flag the renderer must remember to check is
// how a healthy link makes a dead sensor look alive.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"
#include "scene.hxx"

namespace link
{
  // The viewer's one monotonic millisecond clock, based at the first call. The
  // network thread stamps arrivals with it and the frame loop asks in it, so
  // every age is a subtraction within one origin.
  [[nodiscard]] Int64 monoMs();

  // docs/bibowire.md section 7: drawn normally up to FRESH_MS, desaturated with
  // its age printed up to GONE_MS, not drawn at all beyond it. A greyed-out
  // picture is still read as current.
  constexpr Int64 FRESH_MS = 400;
  constexpr Int64 GONE_MS = 1500;

  // Each feed's stale band is derived from its own measured arrival cadence: a
  // feed keeping its observed rate, jitter included, is never called stale, and
  // one that has stopped still goes stale and then disappears. A fixed FRESH_MS
  // is wrong for a feed slower than it (the camera) and sits on the jitter of
  // one near it (the scan's pauses). The floor is FRESH_MS; STALE_CEIL_MS keeps
  // every band strictly below GONE_MS, so stale is a band every feed passes
  // through before it vanishes.
  constexpr Int64 STALE_CEIL_MS = 1200;

  // A feed is stale past STALE_SLACK_NUM / STALE_SLACK_DEN of its widest gap.
  constexpr Int64 STALE_SLACK_NUM = 3;
  constexpr Int64 STALE_SLACK_DEN = 2;

  // The gaps the band is measured over. The worst, not the mean: the mean hides
  // the stalls the band must cover. Sliding, so an old stall stops widening it.
  constexpr Size CADENCE_SAMPLES = 64;

  // Redial when no frame of any type has arrived for this long; BOARD at 5 Hz
  // and PING at 1 Hz make that much silence a dead link.
  constexpr Int64 SILENCE_MS = 3000;

  // The connect deadline of section 2. The name is resolved on every attempt,
  // never cached: the field hotspot's DHCP hands out a new address each outing.
  constexpr Int64 CONNECT_MS = 3000;

  // 250 ms doubling to 4000 over BACKOFF_STEPS, then 4000 forever, each with
  // +-JITTER_PERCENT so viewers returning together do not retry in lockstep.
  constexpr Int32 BACKOFF_STEPS = 5;
  constexpr Int32 JITTER_PERCENT = 20;

  constexpr Size MAX_EVENTS = 64;

  // CMDACKs kept for the pane. A refusal matters until someone has read it.
  constexpr Size MAX_ACKS = 8;

  // Commands queued for the worker. Each is a person's act, so the bound guards
  // against a stuck worker, not a stream.
  constexpr Size MAX_PENDING_COMMANDS = 32;

  // Round trips kept. Latency is their minimum, never the mean: on a hotspot the
  // mean measures stalls, not the path. Half the minimum is the one-way delay.
  constexpr Size RTT_SAMPLES = 16;

  // The viewer's own PING. Answering the board's PING measures the board's round
  // trip, not this end's.
  constexpr Int64 PING_PERIOD_MS = 1000;

  // An unanswered PING is the silence watchdog's business, not a list to grow.
  constexpr Size MAX_PINGS_OUT = 8;

  enum class Phase
  {
      PHASE_IDLE = 0,
      PHASE_RESOLVING,
      PHASE_CONNECTING,
      PHASE_HANDSHAKING,
      PHASE_LIVE,
      PHASE_RETRYING,
  };

  // Arrival times only, never the board's clock: the band asks how long this
  // viewer waits between frames, not how old the board thinks they are.
  struct Cadence
  {
      Bool have = false;
      Int64 lastAtMs = 0;

      // A ring rather than a running maximum, so the band narrows again once a
      // stall stops happening.
      Array<Int64, CADENCE_SAMPLES> gaps = {};
      Size count = 0;
      Size at = 0;
  };

  // Pure. The first arrival only starts the clock: there is no gap before a
  // feed's first frame.
  Void noteArrival(Cadence& c, Int64 nowMs);

  // The widest gap in the window, or 0 when nothing has been measured yet.
  [[nodiscard]] Int64 worstGapMs(const Cadence& c);

  // The age past which this feed is stale: FRESH_MS until a cadence is known,
  // never below FRESH_MS and never above STALE_CEIL_MS.
  [[nodiscard]] Int64 staleBandMs(const Cadence& c);

  struct Revolution
  {
      // The scene's frame and units: X right, Y forward, Z up, metres.
      Vec<scene::Vec3> cloud;

      UInt32 revIndex = 0;
      UInt16 freqMilliHz = 0;
      UInt16 droppedSinceLast = 0;
      UInt8 health = bibowire::HEALTH_ABSENT;
      UInt8 motor = 0;
      Int64 ageMs = 0;

      // The band this feed was held to and the gap it came from, published so
      // the staleness rule can be read off the running system.
      Int64 staleAtMs = FRESH_MS;
      Int64 worstGapMs = 0;

      // Past the band and still inside GONE_MS: drawable, but never as current.
      Bool stale = false;
  };

  struct Decision
  {
      bibowire::Decide decide;
      Int64 ageMs = 0;
      Bool stale = false;
  };

  struct Board
  {
      bibowire::BoardState state;
      Int64 ageMs = 0;
      Bool stale = false;
  };

  // What the board says it is doing, as opposed to what it was asked. Arrives on
  // UDP at 20 Hz, so a network that blocks UDP never delivers it.
  struct Control
  {
      bibowire::CtlState state;
      Int64 ageMs = 0;
      Bool stale = false;
  };

  // What the board's detector found in one camera frame, fresh enough to show.
  // Paired with a CameraShot by frameIndex: the same index is the same bytes.
  struct TagsSeen
  {
      bibowire::Tags tags;
      Int64 ageMs = 0;
      Bool stale = false;
  };

  // One JPEG from the car's camera, fresh enough to draw. The bytes are carried
  // verbatim: jpeg.cxx decodes on the UI thread, keeping a third-party parser off
  // the network thread.
  struct CameraShot
  {
      UInt32 frameIndex = 0;
      UInt16 width = 0;
      UInt16 height = 0;

      // 1 is JPEG and nothing else is defined; carried so another codec is
      // refused by name instead of fed to the decoder.
      UInt8 codec = 0;

      Vec<UInt8> bytes;
      Int64 ageMs = 0;

      // As in Revolution.
      Int64 staleAtMs = FRESH_MS;
      Int64 worstGapMs = 0;

      // The widest gap between captures by the board's clock, against
      // worstGapMs between arrivals by this viewer's. Gappy arrivals with steady
      // captures mean frames were made and lost on the way.
      Int64 worstCaptureMs = 0;

      Bool stale = false;
  };

  struct Note
  {
      bibowire::Severity severity = bibowire::Severity::SEVERITY_INFO;
      Str text;
      Int64 atMs = 0;
  };

  // One CMDACK and when it landed, kept with its text: a tuning verb is refused
  // while armed (docs/bibowire.md section 5, result 3) and only the board's
  // sentence tells the operator why the slider did nothing.
  struct Ack
  {
      bibowire::CmdAck ack;
      Int64 atMs = 0;
  };

  // A CMDACK result byte for a person. The codec defines no names, so this is
  // the one place the viewer spells them.
  [[nodiscard]] CharSeq ackResultName(UInt8 result);

  // One round trip, with what turns it into a clock offset: when the PONG landed
  // here, and the board's clock when it sent it.
  struct RttSample
  {
      Int64 rttMs = 0;
      Int64 atMs = 0;
      UInt64 boardUs = 0;
  };

  // A PING not yet answered. Matched by token, which the protocol echoes, so a
  // duplicate PONG or one for a PING never sent cannot invent a round trip.
  struct PingOut
  {
      UInt64 token = 0;
      Int64 sentMs = 0;
  };

  // Everything one connection has told this viewer. No socket and no clock:
  // every entry point takes the caller's nowMs.
  struct Session
  {
      Bool haveWelcome = false;
      bibowire::Welcome welcome;

      Bool haveLidar = false;
      bibowire::LidarInfo lidar;

      Bool haveScan = false;
      Vec<scene::Vec3> cloud;
      UInt32 revIndex = 0;
      UInt16 freqMilliHz = 0;
      UInt16 droppedSinceLast = 0;
      UInt8 scanHealth = bibowire::HEALTH_ABSENT;
      UInt8 scanMotor = 0;
      Int64 scanAtMs = 0;
      UInt64 scanBoardUs = 0;

      Bool haveDecide = false;
      bibowire::Decide decide;
      Int64 decideAtMs = 0;
      UInt64 decideBoardUs = 0;

      Bool haveBoard = false;
      bibowire::BoardState board;
      Int64 boardAtMs = 0;

      Bool haveControl = false;
      bibowire::CtlState control;
      Int64 controlAtMs = 0;

      // Arrives only while subscribed: CAMERA is CLASS_BULK at about 1 MB/s,
      // bandwidth the scan needs when the window is closed.
      Bool haveCamera = false;
      bibowire::Camera camera;
      Int64 cameraAtMs = 0;

      // Counted, never smoothed: frameIndex is monotonic, so every missed frame
      // is known exactly.
      UInt32 cameraFrames = 0;
      UInt32 missedCameraFrames = 0;
      Str cameraGapText;

      // The board's capture clock, deliberately not a Cadence, which is arrival
      // times only. With missedCameraFrames it classifies a dropout:
      //   frames missing, capture steady  -> lost in transit or at the ring
      //   no frames missing, capture gap  -> the camera or pumpCamera stalled
      UInt64 cameraBoardUs = 0;
      Int64 cameraWorstCaptureMs = 0;

      // What this viewer has sent, not what the board confirmed: SUBSCRIBE has
      // no acknowledgement. A subscription belongs to one connection.
      Bool cameraSubscribed = false;

      // The board's last EVENT about the camera, for the camera window. Matched
      // on the text, a heuristic: bibowire defines no EVENT code for the camera
      // yet. When it does, match the code here instead.
      Bool haveCameraNote = false;
      Str cameraNoteText;
      Int64 cameraNoteAtMs = 0;

      // TAGS: the detector's findings on one camera frame. Always subscribed,
      // since the board sends it only while the apriltag bundle runs and a
      // frame with nothing in it is 24 bytes.
      Bool haveTags = false;
      bibowire::Tags tags;
      Int64 tagsAtMs = 0;
      UInt32 tagFrames = 0;

      // The trim the board has saved: the Pico's lines joined by "; ", empty
      // when nothing is saved (EVENT_CODE_TRIM). boardTrimAtMs and
      // boardTrimCount name one report, so the Trim pane takes each report
      // once, even a repeat of the same text.
      Bool haveBoardTrim = false;
      Str boardTrimText;
      Int64 boardTrimAtMs = 0;
      UInt32 boardTrimCount = 0;

      // The board's bundle list, whole for one generation (docs/bundles.md
      // section 5). Frames are placed by index into `bundlesArriving` until
      // every slot carries one generation, then swapped in as one, so the
      // master window never shows half of one list and half of another. A
      // frame from a newer generation starts the collection over.
      Bool haveBundles = false;
      Vec<bibowire::Bundle> bundles;
      UInt32 bundleGeneration = 0;
      Int64 bundlesAtMs = 0;
      Vec<bibowire::Bundle> bundlesArriving;
      UInt32 bundlesArrivingGeneration = 0;

      // The aggregate: how many are loaded and the last thing that happened.
      Bool haveBundleState = false;
      bibowire::BundleState bundleState;
      Int64 bundleStateAtMs = 0;

      // The board's last EVENT about a bundle (EVENT_CODE_BUNDLE), kept where
      // the master window can show it after it has scrolled off the notes.
      Bool haveBundleNote = false;
      Note bundleNote;

      // One cadence per feed, so the camera and the scan are never held to each
      // other's rate. DECIDE shares its revolution's age, so it uses scanRate.
      Cadence scanRate;
      Cadence cameraRate;
      Cadence tagRate;
      Cadence boardRate;
      Cadence controlRate;

      // The last frame of any type; the silence watchdog reads only this. A link
      // delivering BOARD but no SCAN is a live link with a dead sensor, and the
      // two must not share a timer.
      Int64 lastFrameMs = 0;

      // The arrival floor (section 7): the smallest (localMs - boardMs) this
      // session, from the fastest sample. An age through it is compared with the
      // age since arrival and the larger wins, so a drifting offset can only
      // make a value older.
      Bool haveOffset = false;
      Int64 offsetMs = 0;

      // Survives reconnects: the same bootId means the board kept running, a
      // different one means the pilot restarted and everything here is void.
      Bool haveBootId = false;
      UInt32 bootId = 0;

      // A BYE's reason and its sentence, shown verbatim.
      Bool haveBye = false;
      bibowire::Reason byeReason = bibowire::Reason::REASON_NONE;
      Str byeText;

      Vec<Note> notes;

      // CMDACKs for this viewer's commands, newest last, at most MAX_ACKS. A
      // cmdId belongs to one connection.
      Vec<Ack> acks;

      UInt32 frames = 0;
      UInt32 unknownFrames = 0;
      UInt32 refusedFrames = 0;
      UInt32 resyncBytes = 0;
      UInt32 orphanDecides = 0;
      UInt32 missedRevs = 0;
      Str gapText;

      // PINGs parsed and not yet answered. The pure half cannot send, so the
      // socket half drains this every pass.
      Vec<bibowire::Ping> pongsDue;

      Vec<RttSample> rtts;
      Vec<PingOut> pingsOut;
      Bool haveRtt = false;
      Int64 lastRttMs = 0;

      // Empty when there is nothing fresh enough to draw.
      [[nodiscard]] Opt<Revolution> revolution(Int64 nowMs) const;
      [[nodiscard]] Opt<Decision> decision(Int64 nowMs) const;
      [[nodiscard]] Opt<Board> boardState(Int64 nowMs) const;
      [[nodiscard]] Opt<Control> controlState(Int64 nowMs) const;

      // Empty past GONE_MS, like the scan: a photograph carries no age a person
      // can read, so a frozen one looks exactly as live as a new one.
      [[nodiscard]] Opt<CameraShot> cameraShot(Int64 nowMs) const;

      // Empty past GONE_MS, like the picture it belongs to.
      [[nodiscard]] Opt<TagsSeen> tagsSeen(Int64 nowMs) const;

      // The newest CMDACK, empty until the board has answered anything.
      [[nodiscard]] Opt<Ack> newestAck() const;

      // The measured latency, empty until a PONG has come back. The network's
      // number, not a feed's age: a fast round trip can carry an old scan.
      [[nodiscard]] Opt<Int64> rttMs() const;
      [[nodiscard]] Opt<Int64> bestRttMs() const;
      [[nodiscard]] Opt<Int64> oneWayMs() const;
  };

  // Pure. Records a PING about to be sent, so its PONG can be matched by token.
  Void notePingSent(Session& s, UInt64 token, Int64 nowMs);

  // Forgets everything about a connection. `bootId` and its flag survive, so the
  // next WELCOME can be compared against the last one.
  Void clearSession(Session& s);

  // Forgets the bootId as well: a deliberate disconnect ends the comparison.
  Void clearAll(Session& s);

  // What the decode tells the round-trip log, still pure: counters and copies,
  // no clock, file or socket. The worker passes one in and drains it every pass.
  // A PING's header seq, the quiet before it and an unmatched PONG's token are
  // known only inside the decode. SCAN, CAMERA, BOARD, DECIDE and CTLSTATE are
  // only counted: a line each would load the network thread the log watches.
  struct HeardPing
  {
      UInt64 token = 0;
      UInt16 seq = 0;       // the frame header's, so a gap in the board's stream shows
      Int64 atMs = 0;       // when it was decoded, on monoMs()
      Int64 quietMs = 0;    // since the frame before it, of any type
  };

  struct HeardPong
  {
      UInt64 token = 0;

      // False when no outstanding PING has this token: a duplicate, or so late
      // its PING was pushed out of the bounded list.
      Bool matched = false;
      Int64 rttMs = 0;
  };

  // Per kind per drain. Anything past it is counted in `overflow`.
  constexpr Size HEARD_MAX = 64;

  struct Heard
  {
      // Every frame taken, TCP and UDP, by tag byte, unknown tags included.
      // Cumulative for the connection.
      Array<UInt32, 256> byType = {};

      // A known type whose body would not decode, and the latest one's tag.
      UInt32 bodiesRefused = 0;
      UInt8 lastRefusedType = 0;

      // A tag this build does not know, and the latest one.
      UInt32 unknownTypes = 0;
      UInt8 lastUnknownType = 0;

      // What the TCP reader stepped over. A corrupted frame lands in
      // `resyncBytes`: take() cannot tell a bad CRC from junk.
      // `framingRefused` counts its TOO_BIG and BAD_FLAG answers.
      UInt32 resyncs = 0;
      UInt32 resyncBytes = 0;
      UInt32 framingRefused = 0;

      // TCP header seqs that were not the previous plus one, with the latest
      // pair. Counted, not interpreted: what a jump means depends on whether the
      // board numbers a frame before or after its ring may drop it.
      UInt32 seqJumps = 0;
      Bool haveSeq = false;
      UInt16 lastSeq = 0;
      UInt16 jumpFrom = 0;
      UInt16 jumpTo = 0;

      Vec<HeardPing> pings;
      Vec<HeardPong> pongs;
      Vec<Ack> acks;
      Vec<Note> events;
      UInt32 overflow = 0;
  };

  // Pure. One decoded frame into the session.
  Void ingestFrame(Session& s, const bibowire::Frame& f, Int64 nowMs);

  // The same, also filling `heard`, which may be null.
  Void ingestFrame(Session& s, const bibowire::Frame& f, Int64 nowMs, Heard* heard);

  // Pure. Ingests every whole frame in `buf` and returns the bytes to retire.
  // Junk is resynced past and counted.
  [[nodiscard]] Size ingestBytes(Session& s, const UInt8* buf, Size len, Int64 nowMs);

  // The same, also giving `heard` the resyncs and seq jumps only the byte reader
  // sees.
  [[nodiscard]] Size ingestBytes(Session& s, const UInt8* buf, Size len, Int64 now, Heard* heard);

  // docs/bibowire.md section 6: CONTROL goes out every period while this viewer
  // holds the slot, changed or not. The cadence is the consent the deadman
  // watches, so "nothing changed" and "the link died" must differ on the wire.
  // The UI thread publishes a level the worker samples, never a queue of edges.
  struct Intent
  {
      // The operator's intent to drive; BUTTON_ENABLE follows it.
      // bibowire::deadman::step only reaches STATE_LIVE with enable set, else
      // the car stays at REFUSE_NOT_ARMED. Clearing it is a soft stop (throttle
      // zero, steering held, slot kept), so the stream continues while it is
      // false: stopping is the deadman's job.
      Bool driving = false;

      Int16 steerMilli = 0;
      Int16 throttleMilli = 0;

      // b0 ESTOP, b1 ENABLE, b2 MOTOR_WANTED, built by drive.hxx from the keys
      // and carried verbatim.
      UInt16 buttons = 0;

      // The mode the operator believes is active, from this viewer's own
      // selection and never echoed from CTLSTATE's pilotMode: the board compares
      // the two (REFUSE_MODE), and an echo would make that check always pass.
      // The suite pins it.
      UInt8 assumedMode = 0;
  };

  // Connection facts, stamped by the worker at send time rather than carried on
  // the Intent: the UI thread's snapshot may belong to an earlier session.
  struct ControlStamp
  {
      UInt32 sessionId = 0;
      UInt32 seq = 0;
      UInt8 armEpoch = 0;
      UInt64 tMonoUs = 0;
  };

  // Pure. What this viewer would put on the wire right now.
  [[nodiscard]] bibowire::Control buildControl(const Intent& in, const ControlStamp& at);

  // Pure. Strictly increasing from 1 and never 0, including across the wrap:
  // section 5 starts the stream at 1 and CTLSTATE's ackSeq uses 0 for none.
  [[nodiscard]] UInt32 nextControlSeq(UInt32 previous);

  // Whether the board granted this viewer the control slot: WELCOME's
  // `accepted`, 1 control, 2 observer. Only the board's answer counts; a viewer
  // that assumed the slot would stream CONTROL the board discards
  // (rxControlStale). CTLSTATE's `holder` is the live confirmation.
  [[nodiscard]] Bool holdsSlot(const Session& s);

  // The board's period from WELCOME (section 4), or bibowire::CONTROL_PERIOD_MS
  // before WELCOME or when the board sends 0, which would be a busy loop.
  [[nodiscard]] Int64 controlPeriodMs(const Session& s);

  // The reconnect schedule, pure.
  [[nodiscard]] UInt32 stir(UInt32 seed);
  [[nodiscard]] Int32 backoffBaseMs(Int32 attempt);
  [[nodiscard]] Int32 jittered(Int32 baseMs, UInt32 roll);

  struct Snapshot
  {
      Phase phase = Phase::PHASE_IDLE;

      // The Connection panel's sentence: connecting, handshaking, live,
      // retrying in N ms, and the BYE reason when there is one.
      Str status = "not connected";
      Int32 retryInMs = 0;
      Session state;

      // Section 4's TCP fallback: no CTLSTATE within REVERSE_PROBE_MS of WELCOME
      // means UDP is not getting through, so CONTROL moves onto TCP at the same
      // rate, where the board applies identical rules and deadman. Published so
      // the operator can see why control feels late.
      Bool controlOnTcp = false;

      UInt32 controlSent = 0;
      UInt32 controlFailed = 0;

      // The seq last sent, for the pane to set against CTLSTATE's ackSeq.
      UInt32 controlSeq = 0;
  };

  struct Client
  {
      Str host;
      UInt16 port = bibowire::PORT;

      Thread worker;
      Mutex lock;
      Snapshot shared;

      // `quit` is the only thing the UI thread writes while the worker runs, and
      // the worker checks it between blocking waits, so close() is bounded by
      // one poll slice.
      Atomic<Bool> quit = false;
      Atomic<Bool> running = false;

      // Whether the camera window is open. The worker sends SUBSCRIBE whenever it
      // differs from what this connection last asked for.
      Atomic<Bool> cameraOn = false;

      // Frames per second to ask for, 0 to leave the board's default. The viewer
      // asks because only it knows what its link carries; the board still clamps
      // to bibowire::CAM_FPS_MAX.
      Atomic<Int32> cameraFps = 0;

      // COMMANDs not yet sent. A queue, not a latch like cameraOn: two commands
      // are two acts with two CMDACKs. `cmdLock` guards the queue and
      // `nextCmdId`, which must be read, incremented and stamped as one step, so
      // an Atomic would not do.
      Mutex cmdLock;
      Vec<bibowire::Command> pending;

      // Never 0: the protocol reserves it, and CTLSTATE's lastCmdId uses 0 for
      // none applied. Wraps to 1.
      UInt32 nextCmdId = 1;

      // Commands that never reached the wire (see sendCommand). Atomic because
      // the UI thread reads it while the worker writes it.
      Atomic<UInt32> commandsDropped = 0;

      // A mutex and a struct, not five atomics: the fields are one act, and
      // separate atomics would let the worker pair this frame's steer with the
      // last frame's throttle.
      Mutex ctlLock;
      Intent intent;

      // Read only when dialling: HELLO is the one place the slot is asked for
      // (viewfeed.cxx grants it in onHello alone), so a change takes effect on
      // the next connection. Default true, the operator's choice: holding the
      // slot makes this viewer's cadence the consent the deadman watches in any
      // mode, autonomous runs included, so losing the stream stops the car. The
      // slot moves nothing by itself - the car stays disarmed until ARM - and a
      // second viewer only observes.
      Atomic<Bool> wantSlot = true;
  };

  // Starts the worker. Returns false when one is already running.
  Bool open(Client& c, CharSeq host, UInt16 port);

  // Stops the worker and joins it. Safe to call when nothing is running.
  Void close(Client& c);

  [[nodiscard]] Bool isOpen(const Client& c);

  // Newest-wins: a copy of the most recently published state. The lock is held
  // for a copy, never across a socket call.
  [[nodiscard]] Snapshot snapshot(Client& c);

  // bibowire::typeBit bits to ask for, never zero: 0 means everything to the
  // board, so turning the camera off names every other type explicitly.
  [[nodiscard]] UInt32 subscriptionMask(Bool withCamera);

  // Safe from the UI thread and before a connection exists: the worker sends
  // SUBSCRIBE once WELCOME has arrived, and again after every reconnect.
  Void wantCamera(Client& c, Bool on);

  // Ask for a camera rate in frames per second, or 0 for the board's default.
  // Clamped to bibowire::CAM_FPS_MAX here and again by the board. Re-sent like
  // wantCamera whenever it differs from what the connection was told.
  Void wantCameraFps(Client& c, Int32 fps);

  [[nodiscard]] Int32 cameraFpsWanted(const Client& c);

  // One discrete act, sent once on TCP and answered by one CMDACK. Safe from the
  // UI thread and before a connection exists: this enqueues, and the worker
  // sends after WELCOME. `cmdId` is stamped here, monotonic across the viewer's
  // run and never 0. `sessionId` and `armEpoch` are stamped by the worker at
  // send time, so a command carries the session it goes out on.
  //
  // A command queued while the link is down is dropped, not held: a stale tuning
  // value applied the instant a reconnect succeeds, with nobody watching, is
  // worse than a repeated click. A dropped tuning verb loses nothing the car had:
  // the board keeps the trim it accepted and replays it whenever it opens the
  // Pico (docs/bibowire.md section 5), and reports it on WELCOME for
  // trimview::follow to take. Drops are counted in commandsDropped.
  Void sendCommand(Client& c, bibowire::Verb verb, UInt8 arg0, UInt16 arg1, UInt16 arg2);

  // Commands that never reached the wire.
  [[nodiscard]] UInt32 commandsDropped(const Client& c);

  // CONTROL is sent every controlPeriodMs while this viewer holds the slot, on
  // the client's UDP socket, or on TCP when section 4's probe says UDP is not
  // getting through. There is no viewer-side bibowire::deadman::step: countdowns
  // come from CTLSTATE's neutralInMs and disarmInMs, computed by the board with
  // the function that trips, so the margin shown is the car's.
  //
  // setControl publishes the level the worker samples. Call it every frame, not
  // on change: section 6 forbids change-only control.
  Void setControl(Client& c, const Intent& in);

  [[nodiscard]] Intent controlIntent(Client& c);

  // Ask for the control slot on the next connection. Client::wantSlot has the
  // deadman trade-off of the default.
  Void wantControlSlot(Client& c, Bool on);

  [[nodiscard]] Bool controlSlotWanted(const Client& c);

  // Close, then open the same host, because the slot is asked for in HELLO.
  // Returns false when nothing was running. Nothing survives: a new session and
  // sessionId, a fresh epoch, and a car stopped at DEAD stays stopped until armed
  // again (section 7).
  Bool reconnect(Client& c);
}
