// The viewer's half of bibowire: one TCP connection to the pilot on the Orange
// Pi, decoded on its own thread, handed to the frame loop newest-wins.
//
// ---------------------------------------------------------------------------
// THE CODEC IS NOT HERE, AND THAT IS THE POINT
//
// Every byte that crosses this link is framed, checksummed and read by
// firmware/pilot/src/bibowire.cxx - THE SAME OBJECT FILE the board's program
// compiles, with the same 312 checks behind it. docs/bibowire.md section 11
// requires exactly that, so the two ends cannot drift into disagreeing about a
// field's offset while both still compile. Nothing in this module hand-rolls
// framing, a CRC or a message body; what it owns is sockets, a thread, a
// reconnect schedule and the question "is what I am about to draw still true".
//
// ---------------------------------------------------------------------------
// THE TWO HALVES, AND WHY THEY ARE SEPARATE
//
// `Session` is PURE - no socket, no clock, no thread. Bytes and a millisecond
// go in, decoded state comes out. That is what lets the whole decode path be
// exercised against hand-built frames on a laptop with no board anywhere
// (viewer/tests/test_link.cxx), which matters here more than usual: the pilot
// that serves port 8020 is being written in parallel and has never run.
//
// `Client` is the socket half: a thread, a connection, a backoff schedule.
// It owns a `Session` and publishes copies of it under a lock.
//
// ---------------------------------------------------------------------------
// STALENESS IS INSIDE THE ACCESSOR, NOT BESIDE IT
//
// revolution(), decision() and board() take the caller's clock and return
// Opt<T>, empty when the value is too old to draw. There is deliberately no
// way to reach the underlying revolution without naming a time, because the
// alternative - a field the renderer is trusted to check - is this repo's named
// recurring bug class (a healthy link making a dead sensor look alive) with the
// check written on the wrong side of the seam. Absence is the only rendering a
// person cannot misread.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"
#include "scene.hxx"

namespace link
{

  // One monotonic millisecond clock for the viewer, based at the first call.
  // The network thread stamps arrivals with it and the frame loop asks its
  // questions in it; two clocks would make every age a subtraction between
  // different origins.
  [[nodiscard]] Int64 monoMs();

  // ---- the numbers this module owns ------------------------------------------
  //
  // docs/bibowire.md section 7: drawn normally to 400 ms, drawn desaturated with
  // its age printed to 1500, NOT DRAWN AT ALL beyond it. A greyed-out picture is
  // still a picture and people read pictures as current whatever colour they
  // are.
  constexpr Int64 FRESH_MS = 400;
  constexpr Int64 GONE_MS = 1500;

  // ---- the staleness band, MEASURED rather than assumed ----------------------
  //
  // FRESH_MS is right for a feed arriving FASTER than it. A 10 Hz revolution is
  // ~100 ms old when it lands and nothing about 400 is arbitrary for it. It is
  // wrong as a constant for a feed arriving SLOWER, and the camera proved it on
  // the board: at the pilot's 2 fps default a picture is 500 ms old the instant
  // before its successor lands, so it read STALE for the last 100 ms of every
  // frame and the window flickered twice a second. Measured against the real
  // board, 57 camera frames out of 57 crossed FRESH_MS while the camera was
  // delivering exactly what it had promised. That is two constants chosen in
  // different files disagreeing - not a camera that was ever late.
  //
  // The scan had the same disease from the other end. Its mean interval on this
  // board is 103 ms, comfortably inside 400 - but the board goes quiet for about
  // 400 ms every five or six seconds (SCAN and BOARD stop together, and the
  // board's OWN tMonoUs deltas show the gap, so it is the pilot pausing and not
  // the network), which puts the threshold exactly on the feed's jitter. A
  // threshold sitting on the noise floor is a coin toss rendered as a colour.
  //
  // So the band is derived from what the feed is ACTUALLY DELIVERING, measured
  // here from arrival times. The property it holds: a feed keeping its own
  // observed cadence, INCLUDING that cadence's jitter, is never called stale;
  // one that has genuinely stopped still goes stale and then disappears.
  //
  // The two bounds are what keep it honest. The floor holds a fast feed to
  // FRESH_MS, so measuring cannot make the scan's band SMALLER than the number
  // section 7 fixed. The ceiling keeps every band strictly below GONE_MS, so
  // "stale" stays a band every feed passes THROUGH on its way to vanishing and
  // never one it can skip - which is the part of section 7 that must survive
  // this change, because a picture with no age written on it is worse than a
  // stale point cloud.
  constexpr Int64 STALE_CEIL_MS = 1200;

  // 1.5x the worst recent gap. A feed whose widest real gap is W is not late
  // until meaningfully past W, and half again is the allowance.
  constexpr Int64 STALE_SLACK_NUM = 3;
  constexpr Int64 STALE_SLACK_DEN = 2;

  // The window the band is measured over. The WORST of the recent gaps, not the
  // mean: the mean of the scan's intervals is 103 ms and its real gap is 400, so
  // a band built from the mean is a band that flickers on every stall. Bounded
  // and sliding, so a stall that has stopped happening stops widening the band -
  // an all-time worst would never narrow again and the feed could die quietly
  // inside a band its worst moment bought it an hour ago.
  constexpr Size CADENCE_SAMPLES = 64;

  // Redial when no frame OF ANY TYPE has arrived in this long. At 5 Hz BOARD and
  // 1 Hz PING, silence that long is not a quiet moment. The car stopped 2700 ms
  // before it mattered.
  constexpr Int64 SILENCE_MS = 3000;

  // The connect deadline of section 2, and the name is resolved on every attempt
  // rather than cached: the field network is a phone hotspot whose DHCP hands
  // out a different address every outing.
  constexpr Int64 CONNECT_MS = 3000;

  // 250, 500, 1000, 2000, 4000, then 4000 forever - each with +-20 % jitter,
  // never giving up. Jitter because the viewer and the phone dashboard come back
  // at the same instant when the hotspot returns, and two clients synchronised
  // on one schedule hammer the board in lockstep.
  constexpr Int32 BACKOFF_STEPS = 5;
  constexpr Int32 JITTER_PERCENT = 20;

  // The prose channel is bounded. An unbounded one is a leak with a good excuse.
  constexpr Size MAX_EVENTS = 64;

  // The board's answers to this viewer's COMMANDs, kept so a pane can show what
  // the car actually said. A FEW, not all of them: a refusal matters for as long
  // as it takes somebody to read it, and the useful question is "what did the
  // last thing I did do", never "what did I do a hundred commands ago".
  constexpr Size MAX_ACKS = 8;

  // Commands waiting for the worker to put them on the wire. Bounded for the
  // same reason as everything else here, and generously: every one of these is
  // a deliberate act by a person, so the bound is a guard against a stuck
  // worker rather than against a stream.
  constexpr Size MAX_PENDING_COMMANDS = 32;

  // THE LAST 16 ROUND TRIPS, AND THE MINIMUM OF THEM - never the mean. On a
  // hotspot the mean is dominated by stalls, so it measures the worst moment of
  // the last sixteen seconds rather than the path; the minimum is the closest
  // thing to the true one. Half of it is the one-way delay.
  constexpr Size RTT_SAMPLES = 16;

  // This viewer sends its OWN PING at 1 Hz. Answering the board's PING proves
  // the board's round trip, not ours, and a latency readout built from it would
  // be a number measured on the far end wearing this end's label.
  constexpr Int64 PING_PERIOD_MS = 1000;

  // Outstanding PINGs are bounded too: an unanswered one is the silence
  // watchdog's business, not a list to grow.
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

  [[nodiscard]] CharSeq phaseName(Phase p);

  // reactive::Mode, spelled for a person. The pilot owns these numbers and
  // bibowire carries them as a UInt8; the codec exports no name for them, so
  // this is the one place the viewer spells them and the one place to fix if
  // reactive.hxx ever gains a sixth.
  [[nodiscard]] CharSeq modeName(UInt8 mode);

  // 0 manual, 1 look, 2 drive - WHO produced a DECIDE's numbers.
  [[nodiscard]] CharSeq sourceName(UInt8 source);

  // ---- what a feed is actually delivering at ----------------------------------

  // Arrival times only. It never looks at the board's clock, which is what makes
  // it answer the question the band actually asks - "how long does this viewer
  // wait between pictures" - rather than "how old does the board think they
  // are". Those are different numbers and only the first one flickers.
  struct Cadence
  {
      Bool have = false;
      Int64 lastAtMs = 0;

      // The last CADENCE_SAMPLES gaps, oldest overwritten first. A ring rather
      // than a running maximum, so the band NARROWS again once a stall stops
      // happening.
      Array<Int64, CADENCE_SAMPLES> gaps = {};
      Size count = 0;
      Size at = 0;
  };

  // Pure. One arrival. The FIRST one only starts the clock - there is no gap
  // before a feed's first frame, and inventing one would be a measurement of
  // when the viewer happened to connect.
  Void noteArrival(Cadence& c, Int64 nowMs);

  // The widest gap in the window, or 0 when nothing has been measured yet.
  [[nodiscard]] Int64 worstGapMs(const Cadence& c);

  // The age past which THIS feed is stale, from what it has been delivering.
  // FRESH_MS until a cadence is known, so a feed is held to section 7's number
  // until it has earned a different one, and never wider than STALE_CEIL_MS so
  // stale always sits strictly below GONE_MS.
  [[nodiscard]] Int64 staleBandMs(const Cadence& c);

  // ---- what the frame loop is allowed to draw --------------------------------

  struct Revolution
  {
      // Already in the scene's frame and units: X right, Y forward, Z up,
      // metres. The conversion happens once, where the units are known.
      Vec<scene::Vec3> cloud;

      UInt32 revIndex = 0;
      UInt16 freqMilliHz = 0;
      UInt16 droppedSinceLast = 0;
      UInt8 health = bibowire::HEALTH_ABSENT;
      UInt8 motor = 0;
      Int64 ageMs = 0;

      // The band this feed was actually held to, and the widest recent gap it
      // was derived from. Carried out rather than kept private: a staleness
      // rule nobody can read off the running system is the same species of bug
      // as a test that measures nothing.
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

  // What the board said it is DOING, as opposed to what it was asked. Arrives on
  // UDP at 20 Hz, so it may never arrive at all on a network that blocks it -
  // which is exactly why `have` is a field and not an assumption.
  struct Control
  {
      bibowire::CtlState state;
      Int64 ageMs = 0;
      Bool stale = false;
  };

  // One JPEG from the car's camera, still true enough to draw.
  //
  // The bytes are carried VERBATIM and are not decoded here: this module owns
  // the wire and the question "is this still true", and a JPEG decoder in it
  // would put a third-party parser on the network thread. jpeg.cxx decodes, on
  // the UI thread, once per frame index.
  struct CameraShot
  {
      UInt32 frameIndex = 0;
      UInt16 width = 0;
      UInt16 height = 0;

      // Echoed on every frame so a capture is self-describing - 1 is JPEG and
      // nothing else is defined. Carried rather than assumed, so a frame in
      // some future codec is refused by name instead of being fed to a decoder
      // that will find out the hard way.
      UInt8 codec = 0;

      Vec<UInt8> bytes;
      Int64 ageMs = 0;

      // As Revolution above: the band this picture was judged against and the
      // measured gap behind it. The camera window shows both, because "stale"
      // with no number beside it is the claim that flickered.
      Int64 staleAtMs = FRESH_MS;
      Int64 worstGapMs = 0;

      Bool stale = false;
  };

  struct Note
  {
      bibowire::Severity severity = bibowire::Severity::SEVERITY_INFO;
      Str text;
      Int64 atMs = 0;
  };

  // One CMDACK, with the moment it landed.
  //
  // THE SENTENCE IS THE WHOLE POINT. A tuning verb is refused while the car is
  // armed (docs/bibowire.md section 5, result = 3) and the board says why in
  // words; a viewer that dropped the text would leave an operator dragging a
  // slider that does nothing, with no way to find out that the car had answered
  // at all. Silence after a refused command is this repo's recurring bug class
  // wearing a UI - so the ack is kept, not logged and forgotten.
  struct Ack
  {
      bibowire::CmdAck ack;
      Int64 atMs = 0;
  };

  // What a CMDACK's `result` byte means, for a person. The codec carries the
  // number and defines no name for it, so this is the one place the viewer
  // spells them and the one place to fix if the protocol gains a fifth.
  [[nodiscard]] CharSeq ackResultName(UInt8 result);

  // One measured round trip, kept with the two numbers needed to turn it into a
  // clock offset: when the PONG landed here, and what the board's clock said
  // when it sent it.
  struct RttSample
  {
      Int64 rttMs = 0;
      Int64 atMs = 0;
      UInt64 boardUs = 0;
  };

  // A PING sent and not yet answered. Matching is by TOKEN, which the protocol
  // echoes verbatim, so a duplicate PONG or one for a PING this connection never
  // sent cannot invent a round trip.
  struct PingOut
  {
      UInt64 token = 0;
      Int64 sentMs = 0;
  };

  // ---- the pure half ---------------------------------------------------------

  // Everything one connection has told this viewer. No socket, no clock: every
  // entry point takes the caller's `nowMs`, which is what makes the decode path
  // testable against recorded bytes.
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

      // ---- the camera --------------------------------------------------
      //
      // Arrives ONLY while this viewer has subscribed to it. CAMERA is
      // CLASS_BULK and the stream is about 1 MB/s at 640x480, so a viewer that
      // received it whether or not anybody was looking would spend the scan's
      // bandwidth on a window that is closed.
      Bool haveCamera = false;
      bibowire::Camera camera;
      Int64 cameraAtMs = 0;

      // Counted, never smoothed - frameIndex is monotonic, so what is missing
      // is knowable exactly. The same rule the scan's missedRevs follows, and
      // for the same reason: showing the next picture as though nothing were
      // dropped hides a link losing half the stream.
      UInt32 cameraFrames = 0;
      UInt32 missedCameraFrames = 0;
      Str cameraGapText;

      // WHAT THIS VIEWER HAS SENT, not what the board has confirmed. There is
      // no acknowledgement for SUBSCRIBE in the protocol, so this is the
      // honest name for it: the difference between "we have not asked yet" and
      // "we asked and nothing came back" is a real distinction for a person
      // staring at an empty rectangle, and it is the only part of it this end
      // can actually know. Cleared with the rest of the session, because a
      // subscription belongs to one connection.
      Bool cameraSubscribed = false;

      // The board's last sentence ABOUT THE CAMERA, kept apart from the
      // general note list so the camera window can show it beside the empty
      // rectangle it explains - "the phone dashboard has /dev/video0" is the
      // one thing that turns a blank window into an answer.
      //
      // Matched on the TEXT, which is a heuristic and is written down as one.
      // EVENT carries a `code` byte, but bibowire defines no code for the
      // camera anywhere - not in the document, not in the header, not in its
      // 312 checks - so there is nothing structured to match on yet. When the
      // board-side producer lands and claims a code, THIS is the line to
      // change, and it is one line.
      Bool haveCameraNote = false;
      Str cameraNoteText;
      Int64 cameraNoteAtMs = 0;

      // WHAT EACH FEED IS DELIVERING AT, one per feed and deliberately not one
      // shared number. The camera runs at 2 fps and the scan at 10 Hz on the
      // same connection, so a single cadence would hold each of them to the
      // other's clock - which is the exact mistake that made a camera borrow a
      // threshold built for a feed arriving five times faster.
      //
      // DECIDE has none of its own: it is tied to the revolution it describes
      // and shares that revolution's age exactly, so it is judged by the scan's
      // band for the same reason it borrows the scan's board clock.
      Cadence scanRate;
      Cadence cameraRate;
      Cadence boardRate;
      Cadence controlRate;

      // The last frame of ANY type. The silence watchdog reads this and nothing
      // else: a link that is delivering BOARD but no SCAN is a live link with a
      // dead sensor, and those two facts must not share one timer.
      Int64 lastFrameMs = 0;

      // THE ARRIVAL FLOOR (section 7). The smallest (localMs - boardMs) seen
      // this session, which is the sample that travelled fastest and so the
      // closest thing to the true offset. An age computed through it is compared
      // with the age since local arrival and THE LARGER WINS, so a clock offset
      // that drifts optimistic can only ever be corrected upward by the fact
      // that the bytes have not arrived yet.
      Bool haveOffset = false;
      Int64 offsetMs = 0;

      // Carried ACROSS reconnects by the Client, and the whole point of it: the
      // same bootId means the board kept running, a different one means the
      // pilot restarted and everything below is about a car that no longer
      // exists.
      Bool haveBootId = false;
      UInt32 bootId = 0;

      // A BYE's reason and its sentence. The sentence is the part a person can
      // act on, so it is kept verbatim and shown rather than logged.
      Bool haveBye = false;
      bibowire::Reason byeReason = bibowire::Reason::REASON_NONE;
      Str byeText;

      Vec<Note> notes;

      // WHAT THE BOARD SAID ABOUT THIS VIEWER'S COMMANDS, newest last. Bounded
      // at MAX_ACKS. Cleared with the rest of the session, because a cmdId
      // belongs to one connection: the counter that issued it is the viewer's
      // and the session that carried it is gone.
      Vec<Ack> acks;

      // Counted, never smoothed. A number nobody can read off the running system
      // is the same species of bug as a test that measures nothing.
      UInt32 frames = 0;
      UInt32 unknownFrames = 0;
      UInt32 refusedFrames = 0;
      UInt32 resyncBytes = 0;
      UInt32 orphanDecides = 0;
      UInt32 missedRevs = 0;
      Str gapText;

      // PINGs that have been parsed and not yet answered. The pure half cannot
      // send, so it records what the socket half owes; the socket half drains
      // this every pass.
      Vec<bibowire::Ping> pongsDue;

      // The round trips this viewer measured, and the PINGs still waiting for an
      // answer. Cleared with the rest of the session, because a round trip
      // belongs to one connection.
      Vec<RttSample> rtts;
      Vec<PingOut> pingsOut;
      Bool haveRtt = false;
      Int64 lastRttMs = 0;

      // Empty when there is nothing true to draw. The staleness test lives HERE
      // rather than at the call site - see the header comment.
      [[nodiscard]] Opt<Revolution> revolution(Int64 nowMs) const;
      [[nodiscard]] Opt<Decision> decision(Int64 nowMs) const;
      [[nodiscard]] Opt<Board> boardState(Int64 nowMs) const;
      [[nodiscard]] Opt<Control> controlState(Int64 nowMs) const;

      // Empty when the newest frame is too old to draw, exactly like the scan.
      // A frozen last picture drawn as though it were live is the precise lie
      // section 7 is written to prevent, and it is worse for a camera than for
      // the cloud: a photograph of a corridor looks equally convincing whether
      // it was taken now or forty seconds ago.
      [[nodiscard]] Opt<CameraShot> cameraShot(Int64 nowMs) const;

      // The measured latency, empty until a PONG has actually come back. This is
      // the NETWORK's number and it answers a different question from the ages
      // above: the round trip can be 8 ms while the scan behind it is two
      // seconds old, which is precisely the pair of lies section 7 separates.
      // The newest answer the board gave, empty until it has answered anything.
      // Empty is a REAL state and not a formality: between sending a command and
      // its CMDACK there is nothing true to show, and a pane that displayed the
      // previous verb's answer there would be reporting the wrong command's
      // result at precisely the moment somebody is watching for one.
      [[nodiscard]] Opt<Ack> newestAck() const;

      [[nodiscard]] Opt<Int64> rttMs() const;
      [[nodiscard]] Opt<Int64> bestRttMs() const;
      [[nodiscard]] Opt<Int64> oneWayMs() const;
  };

  // Pure. Records a PING this viewer is about to put on the wire, so the PONG
  // that comes back can be matched to it by token.
  Void notePingSent(Session& s, UInt64 token, Int64 nowMs);

  // Forgets everything about a connection. `bootId` and its flag SURVIVE, which
  // is what lets the next WELCOME be compared against the last one.
  Void clearSession(Session& s);

  // Forgets the bootId as well: a deliberate disconnect ends the comparison.
  Void clearAll(Session& s);

  // Pure. One decoded frame into the session.
  Void ingestFrame(Session& s, const bibowire::Frame& f, Int64 nowMs);

  // Pure. Takes as many whole frames as `buf` holds, ingesting each; returns the
  // number of bytes the caller should retire. Junk is resynced past and counted,
  // never skipped in silence.
  [[nodiscard]] Size ingestBytes(Session& s, const UInt8* buf, Size len, Int64 nowMs);

  // ---- CONTROL, the pure half ------------------------------------------------
  //
  // THE CADENCE IS THE CONSENT, NOT THE CONTENT. docs/bibowire.md section 6: a
  // CONTROL goes out every CONTROL_PERIOD_MS for as long as this viewer holds
  // the slot, changed or not. A protocol that sent control on change would make
  // "nothing changed" and "the link died" the same event on the wire, which is
  // this repo's recurring bug class pointed at the one mechanism that stops a
  // car. So what the UI thread publishes is a LEVEL that the worker samples on
  // its own schedule, never a queue of edges: a key that is held down is held
  // down twenty times a second, and a key that is not is silence with a value
  // in it.

  struct Intent
  {
      // THE OPERATOR'S INTENT TO DRIVE, and BUTTON_ENABLE follows it exactly.
      // bibowire::deadman::step only reaches STATE_LIVE when `enable` is set,
      // so a stream without it leaves the car at REFUSE_NOT_ARMED with the keys
      // looking dead. Clearing it is a SOFT stop - throttle to zero, steering
      // held, the slot kept - which is why the stream CONTINUES while this is
      // false rather than stopping: stopping is what the deadman is for.
      Bool driving = false;

      Int16 steerMilli = 0;
      Int16 throttleMilli = 0;

      // b0 ESTOP, b1 ENABLE, b2 MOTOR_WANTED. Built by drive.hxx from the keys
      // and carried verbatim, so this module never decides what a key means.
      UInt16 buttons = 0;

      // WHAT THE OPERATOR BELIEVES IS ACTIVE - from this viewer's OWN mode
      // selection, and NEVER echoed from CTLSTATE's pilotMode. The board
      // compares the two to catch somebody driving under a false belief (its
      // `REFUSE_MODE`), and a viewer that reflected the board's answer back
      // would make that comparison always true and delete the check. That exact
      // bug was found and fixed on the BOARD side of this comparison
      // (viewfeed.cxx, onControlFrame); this is the other end of it, and the
      // suite pins it.
      UInt8 assumedMode = 0;
  };

  // The three facts only the CONNECTION knows, stamped by the worker at the
  // moment of sending rather than carried on the Intent - sendCommand's rule and
  // the same reason: a snapshot taken on the UI thread is the session the
  // operator typed into, which may not be the one the frame goes out on.
  struct ControlStamp
  {
      UInt32 sessionId = 0;
      UInt32 seq = 0;
      UInt8 armEpoch = 0;
      UInt64 tMonoUs = 0;
  };

  // Pure. What this viewer would put on the wire right now.
  [[nodiscard]] bibowire::Control buildControl(const Intent& in, const ControlStamp& at);

  // Pure. STRICTLY INCREASING FROM 1, and never 0 - section 5 starts the stream
  // at 1 and the board's newest-wins comparison is on the difference, so a 0
  // would be a datagram the board could not tell from "no seq at all". At 20 Hz
  // the wrap is 6.8 years away and is written anyway, because a rule held by an
  // arithmetic coincidence is a rule nobody can point at.
  [[nodiscard]] UInt32 nextControlSeq(UInt32 previous);

  // Whether the BOARD said this viewer has the control slot. WELCOME's
  // `accepted` is the authority: 1 is "control is yours", 2 is observer.
  //
  // NOT a question this end can answer on its own, which is the whole point.
  // The slot is asked for in HELLO and granted (or not) in the answer, so a
  // viewer that decided locally that it was driving would send a stream the
  // board counts in rxControlStale and discards, while its own UI showed a car
  // it was not connected to. CTLSTATE's `holder` is the live confirmation and
  // the pane shows that too.
  [[nodiscard]] Bool holdsSlot(const Session& s);

  // THE BOARD'S OWN CADENCE, not a constant compiled into this viewer months
  // earlier - section 4 says WELCOME carries controlPeriodMs, staleMs and deadMs
  // for exactly this reason. bibowire::CONTROL_PERIOD_MS is the fallback for
  // "no WELCOME yet", and a board that sends 0 does not get to make this viewer
  // spin: a period of zero is not a faster stream, it is a busy loop.
  [[nodiscard]] Int64 controlPeriodMs(const Session& s);

  // ---- the reconnect schedule, as three pure functions -----------------------

  [[nodiscard]] UInt32 stir(UInt32 seed);
  [[nodiscard]] Int32 backoffBaseMs(Int32 attempt);
  [[nodiscard]] Int32 jittered(Int32 baseMs, UInt32 roll);

  // ---- the socket half -------------------------------------------------------

  struct Snapshot
  {
      Phase phase = Phase::PHASE_IDLE;

      // The truth, in a sentence, for the Connection panel: connecting,
      // handshaking, live, retrying in N ms, and the BYE reason when there is
      // one.
      Str status = "not connected";
      Int32 retryInMs = 0;
      Session state;

      // ---- how CONTROL is actually leaving this machine --------------------
      //
      // Section 4's TCP fallback, made visible. When no CTLSTATE datagram has
      // arrived within REVERSE_PROBE_MS of WELCOME, UDP is not getting through
      // and CONTROL moves onto the TCP connection at the same rate - the board
      // accepts it there always, with identical rules and identical deadman.
      // The banner is part of the contract and not a nicety: driving degraded
      // is still driving, and an operator who cannot tell is an operator who
      // will not know why the wheel feels late behind a 2.5 KB SCAN.
      Bool controlOnTcp = false;

      // COUNTED, NEVER SMOOTHED, and published so a pane can show them. A
      // control stream nobody can measure is the exact failure this repo keeps
      // finding, and here it would be measured in metres of car.
      UInt32 controlSent = 0;
      UInt32 controlFailed = 0;

      // The seq this viewer last put on the wire, so the pane can hold it up
      // against CTLSTATE's ackSeq - "sent 412, applied 411" is a link working,
      // and "sent 412, applied 96" is one that stopped three hundred datagrams
      // ago while every socket still looks perfect.
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
      // the worker checks it between every blocking wait - so a Disconnect is
      // bounded by one poll slice rather than by a socket timeout.
      Atomic<Bool> quit = false;
      Atomic<Bool> running = false;

      // Set by the UI thread from whether the camera window is open, read by
      // the worker, which sends SUBSCRIBE whenever it differs from what this
      // connection last asked for. An atomic rather than a lock because it is
      // one bit written once a frame and read once a poll slice.
      Atomic<Bool> cameraOn = false;

      // FRAMES PER SECOND THIS VIEWER IS ASKING FOR, 0 meaning "do not ask" -
      // in which case the board keeps its own conservative default and this
      // viewer behaves exactly as it did before the field existed.
      //
      // The viewer asks because only the viewer knows what its link is
      // carrying: the board cannot tell a LAN from a phone hotspot from the
      // far end, and the number that is right for one is ruinous for the
      // other. The board still decides - it clamps to bibowire::CAM_FPS_MAX -
      // so this is a request and never a command.
      Atomic<Int32> cameraFps = 0;

      // ---- COMMANDs the UI thread has asked for and the worker has not sent --
      //
      // The wantCamera pattern, with a queue instead of a bit, because these do
      // not COLLAPSE: two camera-on requests are one fact, but "set the servo
      // limits" followed by "set the trim" are two acts and each gets its own
      // CMDACK. A latch would silently lose the first of them.
      //
      // `cmdLock` guards both the queue and `nextCmdId`. The counter is not an
      // Atomic because it must be read, incremented and stamped onto a Command
      // as ONE step - an atomic would make each of those safe and the trio
      // still able to issue one id twice.
      Mutex cmdLock;
      Vec<bibowire::Command> pending;

      // Viewer-monotonic and NEVER 0 - the protocol reserves 0, and CTLSTATE's
      // lastCmdId uses it to mean "none applied", so a command numbered 0 would
      // be a command the board could not report having run. Starts at 1 and
      // skips back to 1 rather than to 0 on the wrap that will never happen.
      UInt32 nextCmdId = 1;

      // COUNTED, NOT SWALLOWED. Commands are DROPPED rather than held when the
      // link is down (see sendCommand), and a drop nobody can see is the exact
      // failure this repo keeps finding - so the pane can show that a command
      // never left. Atomic because the UI thread reads it while the worker
      // writes it.
      Atomic<UInt32> commandsDropped = 0;

      // ---- what the operator is asking the car to do -----------------------
      //
      // A MUTEX AND A STRUCT, not five atomics, and the difference matters. The
      // five fields are ONE act - "left, no throttle, enabled, in manual" - and
      // five independent atomics would let the worker read a steer from this
      // frame beside a throttle from the last one. Two halves each locally
      // correct and broken as a pair is a failure this repo has a name for, and
      // this is the one place in the viewer where it would be measured in
      // metres of car. The lock is held for a struct copy, once a UI frame and
      // once a poll slice.
      Mutex ctlLock;
      Intent intent;

      // ASKED FOR IN HELLO AND NOWHERE ELSE. bibowire v1 has no message that
      // takes the control slot mid-session: viewfeed.cxx grants it in onHello
      // and in no other place (`c.holder = true` appears exactly once), so this
      // is read when the connection is DIALLED and changing it later changes
      // nothing until the next one. The pane says that in words rather than
      // leaving a checkbox that appears to do nothing.
      //
      // DEFAULT FALSE, and that is a safety property rather than a default.
      // Section 6: the moment a viewer takes the slot, in ANY mode including
      // drive, its cadence becomes the consent the deadman watches and losing
      // it stops the car. Merely opening this viewer must not arm a deadman
      // over somebody else's autonomous run.
      Atomic<Bool> wantSlot = false;
  };

  // Starts the worker. Returns false when one is already running.
  Bool open(Client& c, CharSeq host, UInt16 port);

  // Stops the worker and joins it. Safe to call when nothing is running.
  Void close(Client& c);

  [[nodiscard]] Bool isOpen(const Client& c);

  // Newest-wins: a copy of the most recently published state. The frame loop
  // holds the lock for a memcpy and never for a socket.
  [[nodiscard]] Snapshot snapshot(Client& c);

  // ---- the subscription ------------------------------------------------------
  //
  // THE MASK CONVENTION, and it is not this file's invention: bit = tag - 0x10
  // for tags 0x10..0x2F, settled in firmware/pilot/src/viewfeed.cxx and
  // asserted by its suite. So CAMERA (0x20) is bit 16.
  //
  // The obvious mapping - `1u << (tag & 0x1F)` - is a known bug and not a
  // simplification: DECIDE (0x11) and SCHEMA (0xF1) collide on it, so
  // subscribing to one would silently subscribe to the other. A type outside
  // the telemetry range has no bit and is always sent.
  [[nodiscard]] UInt32 typeBit(bibowire::Type type);

  // What this viewer asks for. NEVER ZERO, and that is the point: a zero mask
  // means "everything" to the board, so an unsubscribe spelled as 0 would ask
  // for more than it started with rather than less. Turning the camera off
  // names every other type explicitly instead.
  [[nodiscard]] UInt32 subscriptionMask(Bool withCamera);

  // Ask the board for the camera, or stop asking. Safe from the UI thread and
  // safe before a connection exists - the worker sends SUBSCRIBE once WELCOME
  // has arrived, and again after every reconnect, because a new connection has
  // subscribed to nothing.
  Void wantCamera(Client& c, Bool on);

  [[nodiscard]] Bool cameraWanted(const Client& c);

  // Ask the board for a camera RATE, in frames per second, or 0 to stop asking
  // and let the board's own default stand. Clamped to bibowire::CAM_FPS_MAX on
  // the way out, and again by the board, which is the end that owns the
  // decision. Safe from the UI thread and before a connection exists: the
  // worker re-sends SUBSCRIBE whenever this differs from what the current
  // connection was told, including after a reconnect, because a new connection
  // has asked for nothing.
  Void wantCameraFps(Client& c, Int32 fps);

  [[nodiscard]] Int32 cameraFpsWanted(const Client& c);

  // ---- COMMAND ---------------------------------------------------------------
  //
  // One discrete act, sent once on TCP and answered by exactly one CMDACK.
  //
  // Safe from the UI thread and safe before a connection exists, exactly like
  // wantCamera: this enqueues, and the worker sends once WELCOME has arrived.
  // `cmdId` is stamped HERE, so it is monotonic across the whole run of the
  // viewer rather than per connection, and it is never 0.
  //
  // `sessionId` and `armEpoch` are deliberately NOT arguments and are stamped by
  // the worker at the moment of sending. They are facts about the connection,
  // and the UI thread's copy of them is a snapshot that may be one frame old -
  // stamping them at enqueue time would let a command carry the session it was
  // typed into rather than the one it is sent on.
  //
  // A COMMAND QUEUED WHILE THE LINK IS DOWN IS DROPPED, NOT HELD, and that is a
  // decision rather than an oversight. Holding it would mean a tuning value the
  // operator set minutes ago - and has very likely since changed their mind
  // about, or moved the slider past - being applied to the car at the instant a
  // reconnect succeeds, with nobody watching the moment it lands. These verbs
  // re-tune the limits a throttle is clamped to, they do not survive a Pico
  // reboot anyway (docs/bibowire.md section 5), and the pane's sliders still
  // hold what the operator wants, so re-sending is one click on a live link.
  // A surprise on reconnect is strictly worse than a command that must be
  // repeated. The drop is COUNTED - commandsDropped - because a drop nobody can
  // see is the failure this repo keeps finding.
  Void sendCommand(Client& c, bibowire::Verb verb, UInt8 arg0, UInt16 arg1, UInt16 arg2);

  // How many commands never reached the wire because there was no connection to
  // put them on. Shown, not just counted.
  [[nodiscard]] UInt32 commandsDropped(const Client& c);

  // ---- CONTROL, the socket half ----------------------------------------------
  //
  // WHAT NOW EXISTS, AND WHAT STILL DOES NOT. This block replaced a SEAM
  // comment that said driving was deliberately absent, and the honest version
  // of that paragraph is worth keeping rather than deleting:
  //
  //   - EXISTS: CONTROL is built, framed and sent every controlPeriodMs while
  //     this viewer holds the slot, on the UDP socket the client already binds,
  //     and on TCP instead when section 4's CTLSTATE probe says UDP is not
  //     getting through. seq is strictly increasing from 1, sessionId comes
  //     from WELCOME and armEpoch from the freshest thing the board has said.
  //   - EXISTS: the slot is ASKED FOR, opt-in and default off, in HELLO.
  //   - DOES NOT EXIST: taking the slot without reconnecting. bibowire v1 has
  //     no message for it and the board grants it in onHello alone.
  //   - DOES NOT EXIST: a viewer-side copy of bibowire::deadman::step. The
  //     countdowns an operator reads come from CTLSTATE's neutralInMs and
  //     disarmInMs, which the BOARD computes with the same pure function that
  //     does the tripping - so there is one arithmetic rather than two that can
  //     disagree, and nothing here can render a margin the car does not have.
  //   - NOT TESTED ANYWHERE: any of this against a car. The Pico has never been
  //     connected to the board, no CONTROL datagram has ever moved a wheel, and
  //     a safety mechanism that has run only in a suite is a safety mechanism
  //     nobody has watched fail. viewer/tests/test_link.cxx pins the shapes -
  //     the seq rule, the round trip, the key mapping, the ENABLE bit - and
  //     names in its own header what it cannot reach.

  // The level the worker samples. Safe from the UI thread, safe before a
  // connection exists, and called EVERY FRAME from the pane rather than on
  // change: this is a level and not an edge, and a "send it when it changes"
  // path here would be the one bug section 6 is written to prevent.
  Void setControl(Client& c, const Intent& in);

  [[nodiscard]] Intent controlIntent(Client& c);

  // Ask for the control slot on the NEXT connection - HELLO carries it and
  // nothing else can. Default false; see Client::wantSlot for why that is a
  // safety property and not a preference.
  Void wantControlSlot(Client& c, Bool on);

  [[nodiscard]] Bool controlSlotWanted(const Client& c);

  // Drop this connection and dial the same host again, because asking for the
  // slot is a HELLO-time decision and a HELLO belongs to a connection. Returns
  // false when there was nothing running to restart.
  //
  // It is a close and an open, in that order, and it is deliberately not
  // dressed up as anything smaller: the session is gone, the sessionId is new,
  // the board issues a fresh epoch, and the car - which stopped at DEAD when
  // the old stream stopped - stays stopped until somebody arms it again. That
  // is section 7's reconnect paragraph, and a "reconnect" that hid any of it
  // would be hiding the part an operator has to know.
  Bool reconnect(Client& c);

}
