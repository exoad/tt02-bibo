// bibowire's socket half on the board: a TCP server and a UDP socket on 8020,
// on a thread of its own, serving the Windows viewer.
//
// ---------------------------------------------------------------------------
// WHAT THIS IS, AND WHAT IT IS NOT
//
// bibowire.hxx is the codec - pure, no sockets, no clock, compiled into both
// the pilot and the viewer. This file is the other half: the sockets, the
// per-client queues, the drop classes, the session, the handshake and the
// frame-header ring. Nothing here decides a byte's LAYOUT; every frame is
// built by a bibowire writeX and framed by bibowire::put, so the board and the
// viewer cannot disagree about a field's offset while both still compile.
//
// It is shaped on feed.cxx and is deliberately SEPARATE from it. feed.cxx
// moves LINES for the phone dashboard and the hub on scanwire::PORT, and it
// must keep doing exactly that - the text feed is the field-debugging story
// (docs/bibowire.md section 8) and deleting it to save a socket would trade a
// thing a person can read from a phone for one they cannot.
//
// ---------------------------------------------------------------------------
// publish() NEVER BLOCKS THE CALLER, AND THE TICK NEVER ENCODES
//
// Every publish below takes a mutex for a push, writes one byte to a self-pipe
// to wake poll(), and returns. The ENCODE, the CRC and the send() all happen on
// this module's thread, so the pilot's tick pays a queue push whether the
// viewer is fast, slow or absent - and with nobody connected it pays nothing at
// all, because the message is dropped at the door before the lock is taken.
//
// The one thing the tick does pay for is filling the message struct: a Scan
// carries 500 points converted from reactive::Ray's two Float32 to the wire's
// centi-degrees and whole millimetres. That loop is the pilot's, the framing is
// this thread's, and section 7's rule - "the tick never encodes, never CRCs,
// never calls send(), and never waits for a viewer" - is what that split is for.
//
// ---------------------------------------------------------------------------
// A SLOW VIEWER DEGRADES IN A FIXED ORDER, AND NEVER THE CAR'S PICTURE FIRST
//
// Each client owns a bounded ring - 96 KiB or 12 frames, whichever fills first
// - and every telemetry type has a class:
//
//   CLASS_BULK   discarded FIRST, always. A camera, the day there is one.
//   CLASS_LIVE   drop-oldest, depth 1. The newest revolution wins and the count
//                travels in the next SCAN's droppedSinceLast, so the viewer
//                knows what it missed rather than believing it saw everything.
//                A revolution more than 200 ms old at send time is dropped
//                BEFORE it is ever queued - sending it would spend the
//                bandwidth the current revolution needs, to show something
//                already wrong.
//   CLASS_VITAL  never dropped. If the vital ring fills, the CLIENT IS CLOSED:
//                a viewer that cannot absorb 120 bytes of state has gone,
//                whatever its socket says. So is one whose oldest unsent vital
//                byte is more than BEHIND_MS old - feed.cxx's 500, unchanged.
//
// That ordering is the reason a camera is safe to add to a 220 kbit/s link at
// all: what degrades is the camera, not the car's picture of the world.
//
// ---------------------------------------------------------------------------
// TWO TRANSPORTS, BECAUSE THEY FAIL IN OPPOSITE DIRECTIONS
//
// Telemetry rides TCP, where a 2.5 KB revolution arrives whole or not at all.
// Control rides UDP, where this module drains the socket to EMPTY every pass
// and keeps only the newest sequence number - so a five-second hotspot stall
// leaves no backlog of stale steering to apply when it clears. There is no
// queue to drain, which is the whole point.
//
// The handoff to the pilot's tick is a seqlock over two slots: this thread
// writes the decoded CONTROL and stores its seq with release ordering, and
// control() below loads with acquire, reads, re-loads and retries if it moved.
// The tick never takes this module's mutex and can never read half a command.
//
// ---------------------------------------------------------------------------
// LINUX ONLY, LIKE feed AND link
//
// The real half is accept4, pipe2, poll, recvfrom and MSG_NOSIGNAL. Elsewhere
// start() REFUSES, saying so, and every other call does nothing - the same rule
// feed.hxx states, and for the same reason: a stub that listened and reported
// an empty room would be this repo's named recurring bug with a socket on it.
// tests/test_viewfeed.cxx is a ctest run on the board for that reason and is
// deliberately NOT in firmware\verify.bat, where it could only ever prove that
// start() returns false.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"

namespace viewfeed
{

  // What the board says about itself in WELCOME, and how it names itself in a
  // refusal a person has to act on.
  struct Policy
  {
      // The name the viewer shows in its connection banner.
      Str boardName;

      // This build's git short hash. It goes into the version-refusal sentence
      // verbatim, because "incompatible" alone sends somebody to read source in
      // a field and two build stamps turn it into an action.
      Str boardBuild;

      // New for every pilot PROCESS, and carried in every WELCOME. A viewer
      // that reconnects and sees a different one throws away everything it knew
      // before drawing a point - which is the defence against the most
      // convincing stale picture there is, a healthy new socket to a restarted
      // car still showing the previous run.
      UInt32 bootId = 0;

      // b0 canDrive, b1 hasLidar, b2 hasPico, b3 hasBattery.
      UInt8 capabilities = 0;

      // Called on this module's thread whenever the client count changes. Must
      // return quickly and must not call back into viewfeed - same contract as
      // feed::Policy::onClients.
      Fn<Void(Size clients)> onClients;
  };

  // What the PILOT did with the newest CONTROL, for CTLSTATE's honesty line.
  // Every field here is what the board DID, never what it was asked.
  //
  // The pilot does not drive from CONTROL yet, so nothing calls applied() in
  // this build and CTLSTATE carries the absent sentinels for the fields below.
  // That is deliberate and is the opposite of the failure this repo keeps
  // finding: an unmeasured quantity reads as `n/a` on the wire rather than as a
  // zero somebody would draw as a measurement.
  // There is deliberately NO scanAgeMs here. This module sees every
  // publishScan, so it measures that age ITSELF - and a field the pilot could
  // forget to set is a field that would default to 0, which on this wire reads
  // as "the picture is perfectly fresh". That is the most dangerous value the
  // protocol can carry and this repo's named recurring failure with a steering
  // wheel attached, so the opportunity to make the mistake is removed rather
  // than documented. Every remaining field here defaults to its ABSENT
  // sentinel for the same reason.
  struct Applied
  {
      Int16 steerNowMilli = 0;                                  // where the wheels ACTUALLY are
      Int16 throttleMilli = 0;                                  // what was SENT to the Pico
      UInt16 escUs = bibowire::ESC_ABSENT;
      UInt8 armed = 0;
      UInt8 pilotMode = 0;
      UInt32 picoSilentMs = bibowire::PICO_SILENT_ABSENT;
      UInt32 lastCmdId = 0;
  };

  // What this module measured about itself, for BOARD and for the exit summary.
  // A performance claim nobody can read off the running system is the same
  // species of bug as a test that measures nothing.
  struct Counters
  {
      UInt64 accepted = 0;
      UInt64 refused = 0;
      UInt64 txFrames = 0;
      UInt64 txDroppedFrames = 0;
      UInt64 rxControl = 0;
      UInt64 rxControlStale = 0;
      UInt64 resyncBytes = 0;
      UInt32 encodeAvgNs = 0;
      UInt32 encodeMaxNs = 0;
  };

  // Binds TCP and UDP on 0.0.0.0:port and starts the thread. There is NO
  // fallback port and no relay - scanwire's two-port dance exists because
  // scanfeed idles on 8011 under systemd, and nothing idles on 8020; a second
  // protocol quietly moving next door is how a board ends up with two feeds
  // nobody can tell apart. false, with the reason printed, when either socket
  // could not be made, bound or listened on, and false when called twice
  // without a stop().
  [[nodiscard]] Bool start(UInt16 port, const Policy& p);

  // The port actually bound; 0 when not started. 0 asks the system for one,
  // which is what the test does.
  [[nodiscard]] UInt16 port();

  // Viewers connected and past their handshake, right now.
  [[nodiscard]] Size clients();

  // One revolution, to every client that has been welcomed. By value so a
  // caller that is done with it moves it in. Dropped without a trace when
  // nobody is connected.
  Void publishScan(bibowire::Scan s);

  // The decision for the revolution just published. Sent immediately after the
  // SCAN it describes, so a viewer that has the one has the other.
  Void publishDecide(const bibowire::Decide& d);

  // The board's own state. Rate-limited to 5 Hz on the wire, and REMEMBERED:
  // the newest one is what a client accepted a moment from now is told before
  // it is shown a single point.
  Void publishBoard(const bibowire::BoardState& b);

  // The device's identity. Remembered for the same reason, and re-sent on
  // change. State before scan, always.
  Void publishLidarInfo(const bibowire::LidarInfo& i);

  // The prose channel: every lidar::reason(), every carlink::detail(), every
  // refusal sentence the board already writes for a person, verbatim.
  // Rate-limited to 10/s, with the suppressed count carried in the next one so
  // a viewer knows events were dropped rather than believing it saw them all.
  Void publishEvent(bibowire::Severity severity, UInt8 code, const Str& text);

  // ---------------------------------------------------------------------------
  // THE CAMERA HAS NO publish() AND THAT IS DELIBERATE
  //
  // Every other telemetry type above arrives here from the pilot's tick. The
  // camera does not: this module opens /dev/video0 itself, through one
  // long-lived v4l2-ctl streaming MJPEG into a pipe, and only while at least one
  // viewer has explicitly subscribed to CAMERA. There is nothing for the pilot
  // to call and nothing for it to forget to stop.
  //
  // IT IS OFF UNLESS ASKED FOR, BY AN EXPLICIT BIT. A zero typeMask means
  // "never asked", which everywhere else means everything - and for a camera
  // that would hand a megabyte a second to every viewer written before this
  // existed. CAMERA is the one type excluded from that default: bit 16
  // (`tag - 0x10`, tag 0x20), set by SUBSCRIBE, or no pictures.
  //
  // THE DEVICE IS SINGLE-OPENER, and the phone dashboard
  // (tools/status/status_server.py) opens the same one, so the two can never
  // both hold it. When the capture cannot start or produces nothing, the
  // subscribers are told in an EVENT that names the likely holder rather than
  // being left with a blank panel - an absence with a reason.
  //
  // CAMERA is CLASS_BULK, so it is discarded before any scan or state frame.
  // That is what makes it safe to add to this link: what degrades is the
  // picture, never the car's picture of the world.
  //
  // BIBO_CAM_DEV, BIBO_CAM_SIZE and BIBO_CAM_FPS override the device, the
  // requested format and the rate cap, read once per start(). They are the same
  // names status_server.py reads. The default rate is deliberately low - see
  // CAM_FPS_DEFAULT in viewfeed.cxx for the measured numbers behind it.

  // The newest CONTROL the holder has sent, or false when there is none. Reads
  // a seqlock: no mutex, no allocation, and it cannot return a half-written
  // command. This is the control loop's ENTIRE interaction with this module.
  [[nodiscard]] Bool control(bibowire::Control* out);

  // What the pilot did with it, for the next CTLSTATE. See Applied.
  Void applied(const Applied& a);

  [[nodiscard]] Counters counters();

  // BYE(SHUTDOWN) to every client, then closes them and both sockets and joins
  // the thread. Safe when not started.
  Void stop();

}
