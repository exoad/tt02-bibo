// bibowire's socket half on the board: TCP and UDP on one port, on a thread of
// its own, serving the viewer. bibowire.hxx decides every byte's layout; this
// file owns the sockets, the per-client queues, the session and the handshake.
//
// publish() never blocks and never encodes: it pushes under a mutex and wakes
// poll() through a self-pipe, and the encode, CRC and send() run on this
// module's thread.
//
// Each client has a bounded ring (RING_BYTES or RING_FRAMES, whichever fills
// first), and a slow viewer degrades by class:
//   CLASS_BULK   discarded first.
//   CLASS_LIVE   drop-oldest, depth 1; the count rides the next SCAN's
//                droppedSinceLast. A revolution older than LIVE_STALE_MS at
//                send time is never queued.
//   CLASS_VITAL  never dropped. A client whose vital frame cannot be queued, or
//                whose oldest vital frame is older than BEHIND_MS, is closed.
//
// Telemetry rides TCP. Control rides UDP, drained to empty every pass keeping
// only the newest seq, so a stall leaves no backlog of stale steering.
//
// The camera has no publish(): this module runs v4l2-ctl itself while a viewer
// subscribes to CAMERA by its explicit typeMask bit, which a zero mask never
// includes. BIBO_CAM_DEV, BIBO_CAM_SIZE and BIBO_CAM_FPS override the device,
// the format and the rate cap, read at start().
//
// Linux only. Elsewhere start() refuses, saying so, and every other call does
// nothing - never a stub that listens and reports an empty room. That is why
// tests/test_viewfeed.cxx is a CMake test and not a tools\test.bat suite,
// where it could only prove that start() refuses.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"

namespace viewfeed
{
  // What the board says about itself in WELCOME.
  struct Policy
  {
      Str boardName;

      // This build's git short hash, quoted in the version refusal.
      Str boardBuild;

      // New for every pilot process. A viewer that sees it change discards
      // everything it knew, so a restarted car never shows the previous run.
      UInt32 bootId = 0;

      // b0 canDrive, b1 hasLidar, b2 hasPico, b3 hasBattery.
      UInt8 capabilities = 0;
  };

  // What the pilot DID with the newest CONTROL, for CTLSTATE - never what it
  // was asked. Optional fields default to their ABSENT sentinels. There is no
  // scanAgeMs: this module measures that itself, because a field the pilot
  // forgot would read 0, which on the wire means perfectly fresh.
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

  // What this module measured about itself, for BOARD and the exit summary.
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

  // Binds TCP and UDP on 0.0.0.0:port, with no fallback port, and starts the
  // thread. false, with the reason printed, when a socket cannot be made, bound
  // or listened on, and when already started.
  [[nodiscard]] Bool start(UInt16 port, const Policy& p);

  // The port bound; 0 when not started. start(0) lets the system choose.
  [[nodiscard]] UInt16 port();

  // Viewers past their handshake.
  [[nodiscard]] Size clients();

  // One revolution to every welcomed client; dropped when nobody is connected.
  Void publishScan(bibowire::Scan s);

  // Sent immediately after the SCAN it describes.
  Void publishDecide(const bibowire::Decide& d);

  // Rate-limited to 5 Hz on the wire, and remembered: a newly welcomed client
  // is sent the newest before any scan.
  Void publishBoard(const bibowire::BoardState& b);

  // Remembered like the board state, and re-sent on change.
  Void publishLidarInfo(const bibowire::LidarInfo& i);

  // The saved trim as trimfile::report writes it, empty when nothing is saved.
  // Remembered: sent to each viewer after WELCOME and to every viewer on each
  // call, as an EVENT under bibowire::EVENT_CODE_TRIM that bypasses the event
  // rate limiter.
  Void publishTrim(const Str& report);

  // The prose channel (bibowire::Event). Rate-limited to 10/s; the suppressed
  // count rides the next one.
  Void publishEvent(bibowire::Severity severity, UInt8 code, const Str& text);

  // The bundles this board can run, as one list, one frame per bundle.
  // Remembered like the board state and replayed to a newly welcomed client
  // before its first scan, so a viewer joining mid-run has a master window
  // rather than an empty one.
  //
  // GENERATION, INDEX AND COUNT ARE STAMPED HERE, not taken from the entries: a
  // viewer holds the list once it has `count` frames carrying one generation,
  // so if those three could disagree it would wait for a frame that never
  // comes. Fill in id, name, about, needs and ready; the rest is this module's.
  //
  // An empty list is still announced, as a single frame with count 0 - "this
  // board runs no bundles" and "this board has not said yet" are different
  // answers, and the master window shows different things for them.
  Void publishBundles(UInt32 generation, Vec<bibowire::Bundle> list);

  // Which bundle is running, and how the last one ended. Remembered and
  // replayed the same way.
  Void publishBundleState(const bibowire::BundleState& s);

  // What the detector found in one camera frame, to every viewer subscribed
  // to TAGS. LIVE and not remembered: a detection is about one picture, and
  // replaying an old one to a new viewer would pair it with no picture at all.
  Void publishTags(const bibowire::Tags& t);

  // The wheel encoder as the Pico last reported it, to every viewer
  // subscribed to ODOM. LIVE and not remembered: the count is only worth
  // reading against the one before it.
  Void publishOdom(const bibowire::Odom& m);

  // A LOCAL subscriber to the camera, so the capture runs while no viewer
  // watches: the detector, loaded as a bundle, is the one there is. fps 0
  // asks for the board's default rate, as a viewer naming none does. The
  // device opens on the feed's next pass and closes when the last subscriber,
  // local or viewer, goes. Any thread.
  Void wantCamera(Bool on, UInt16 fps);

  // One captured JPEG, copied out for the local subscriber. frameIndex is
  // the CAMERA frame's own: the picture a viewer holds under that index is
  // these bytes, so a detection on them lands on the right picture.
  struct CameraFrame
  {
      UInt64 tMonoUs = 0;
      UInt32 frameIndex = 0;
      UInt16 width = 0;
      UInt16 height = 0;
      Vec<UInt8> jpeg;
  };

  // The newest captured frame once one newer than *seen exists, or false
  // after waitMs. *seen is this module's count of frames offered (0 before
  // any), updated on success, so a caller starting at 0 is handed the first
  // frame captured after it asked and never a stale one twice. The feed's
  // thread copies under a lock and never waits on the caller.
  [[nodiscard]] Bool waitCameraFrame(UInt32* seen, CameraFrame* out, Int32 waitMs);

  // Whether the camera device is there right now, by the path the capture
  // opens (BIBO_CAM_DEV, else the by-id default). A stat, never an open, so
  // it cannot disturb a running capture. Any thread.
  [[nodiscard]] Bool cameraPresent();

  // The newest CONTROL from the holder, or false when there is none. A seqlock:
  // no mutex, and never a half-written command. Steer and throttle are what
  // control::apply allowed - on an epoch or mode disagreement the throttle is
  // 0 - so a caller cannot drive on a value CTLSTATE reports as refused.
  [[nodiscard]] Bool control(bibowire::Control* out);

  // Whether anyone holds the wheel, and the deadman's verdict. control() says
  // nothing about whether a command may be obeyed; the deadman needs state
  // only this module has, so it is computed here by the function that fills
  // CTLSTATE.
  struct Drive
  {
      Bool haveHolder = false;

      // A viewer's ESTOP - or, outside MANUAL, its DISARM - until CLEAR_ESTOP.
      // What a car program obeys.
      Bool estopLatched = false;

      // 0 live, 1 soft, 2 dead, 3 estop latched: CTLSTATE's byte, from the same
      // mapping.
      UInt8 deadman = 0;
      bibowire::Refuse refuse = bibowire::Refuse::REFUSE_NONE;

      // The deadman's own countdown, so what a viewer shows is what trips.
      Int32 neutralInMs = 0;
      Int32 disarmInMs = 0;

      // A viewer's COMMAND ARM. Granted under one arm epoch and gone when the
      // epoch moves: an estop, the deadman tripping, a lost Pico link, the slot
      // changing hands, or a DISARM. The pilot sends ESC ARM when this rises and
      // ESC DISARM when it falls, and in MANUAL no throttle passes without it.
      Bool armed = false;

      // The age of the copy this was computed from. The feed's thread refreshes
      // it every pass, so a large age means that thread is stuck and no ESTOP
      // can arrive. 0 while the feed is not running.
      Int32 seenAgeMs = 0;
  };

  // Safe from any thread. Reads a copy the feed's thread takes before it
  // answers a viewer, so an ESTOP a viewer was told is latched is latched here.
  [[nodiscard]] Drive drive();

  // One accepted tuning request, as COMMAND carried it. Not translated into a
  // Pico line here: the pilot's tick owns the serial port.
  struct Tune
  {
      bibowire::Verb verb = bibowire::Verb::VERB_NONE;
      UInt8 arg0 = 0;
      UInt16 arg1 = 0;
      UInt16 arg2 = 0;
  };

  // Pops the OLDEST tuning request, or false when there is none; the tick calls
  // it until false. Outside MANUAL tuning is refused and nothing is queued.
  //
  // A queue, where control() is a seqlock: two sliders moved together are two
  // acts, each already acknowledged by a CMDACK, and keeping only the newest
  // would silently lose one.
  [[nodiscard]] Bool tune(Tune* out);

  // One accepted load or unload, as COMMAND carried it. The chain belongs to
  // the pilot, not to this module, so nothing here touches it: this thread
  // validates the request against the list it published and queues it for the
  // tick, which applies it BETWEEN passes.
  struct BundleRequest
  {
      Bool load = false;        // false: unload
      UInt16 index = 0;         // into the BUNDLE list the board published
      UInt32 generation = 0;    // the list the viewer was looking at
  };

  // Pops the OLDEST load or unload, or false when there is none; the tick
  // calls it until false, as it does tune().
  //
  // PERMITTED OUTSIDE MANUAL, where trim and ARM are not. A bundle is what the
  // board runs, so refusing a load while one drives would make the master
  // window dead on the only host that has bundles - docs/bundles.md section 8.
  [[nodiscard]] Bool bundleRequest(BundleRequest* out);

  // What the pilot did with the newest CONTROL, for the next CTLSTATE.
  Void applied(const Applied& a);

  [[nodiscard]] Counters counters();

  // BYE(SHUTDOWN) to every client, closes every socket and joins the thread.
  // Safe when not started.
  Void stop();
}
