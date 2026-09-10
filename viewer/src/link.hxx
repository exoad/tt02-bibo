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

      // Past FRESH_MS and still inside GONE_MS: drawable, but never as current.
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
      Bool stale = false;
  };

  struct Note
  {
      bibowire::Severity severity = bibowire::Severity::SEVERITY_INFO;
      Str text;
      Int64 atMs = 0;
  };

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

  // ---------------------------------------------------------------------------
  // SEAM: sending CONTROL and COMMAND - driving the car - goes here.
  //
  // Deliberately absent, not forgotten. The Pico is not connected to the board,
  // so nothing on that path can be exercised end to end today, and a control
  // path that has never moved a wheel is a safety mechanism nobody has tested.
  // What lands here when it can be tested: CONTROL at 20 Hz on the UDP socket
  // this client already binds (it carries `sessionId` from WELCOME, a strictly
  // increasing `seq` and the `armEpoch` the viewer believes), COMMAND on TCP for
  // the discrete acts, and the viewer's own copy of bibowire::deadman::step so
  // the operator watches the same arithmetic that will do the tripping.
  //
  // Until then this viewer connects as an OBSERVER - HELLO carries
  // wantControl = 0 - which is the honest description of a program that cannot
  // drive, and it means bibowire's deadman is not armed on our account: with no
  // control holder the pilot runs under its own rules exactly as it does with
  // nobody watching (docs/bibowire.md section 6).
  // ---------------------------------------------------------------------------

}
