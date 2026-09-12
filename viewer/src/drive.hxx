// The Drive window: WASD on a keyboard, turned into the CONTROL stream of
// docs/bibowire.md section 6, and the board's own account of what it did with
// it.
//
// ---------------------------------------------------------------------------
// `driveview`, NOT `drive`
//
// camera.cxx's reason and trim.cxx's, arrived at a third time: main.cxx holds a
// `driveview::View drive;`, and a variable named `drive` would hide a namespace
// of that name for the rest of the function. `bibo::drive` is also a firmware
// namespace (the chassis outputs), which is a second reason not to spend the
// word here.
//
// ---------------------------------------------------------------------------
// NOTHING HERE HAS EVER MOVED A WHEEL
//
// Said first because it is the most important thing about this file. The Pico
// has never been connected to the board, no CONTROL datagram has ever reached a
// car, and every claim below is a claim about a shape rather than about a
// vehicle. viewer/tests/test_link.cxx pins the parts that are pure - the key
// mapping, the ENABLE bit, the seq rule, the round trip - and says in its own
// header what it cannot reach.
//
// ---------------------------------------------------------------------------
// THREE GATES, EACH OFF BY DEFAULT, AND THAT IS THE DESIGN
//
// 1. The CONTROL SLOT is asked for in HELLO and nowhere else (the board grants
//    it in its HELLO handler alone), so it is a decision taken when a
//    connection is dialled. Default off: section 6 says the moment a viewer
//    takes the slot, in ANY mode including a fully autonomous drive, its
//    cadence becomes the consent the deadman watches and losing it stops the
//    car. Opening a viewer must not arm a deadman over somebody else's run.
// 2. ENABLE is a toggle in this window, default off. It is the operator saying
//    "I am driving now", and bibowire::deadman::step only reaches STATE_LIVE
//    while it is set.
// 3. The board's own `capabilities` bit 0 - canDrive - gates the whole pane. A
//    pilot started `--dry` says so in WELCOME, and this window says why it is
//    disabled rather than accepting keys that go nowhere.
//
// ---------------------------------------------------------------------------
// STEERING IS BANG-BANG, AND THE SMOOTHING IS SOMEBODY ELSE'S JOB
//
// A is -1000, D is +1000, neither or both is 0. There is no ramp here and no
// proportional feel, because the Pico already has one: `SLEW` limits how fast
// the servo may move, the Trim pane is where it is tuned, and a second ramp in
// the viewer would be two filters in series that nobody could tell apart when
// the steering felt wrong. A key has no travel; pretending it does is inventing
// data at the one end of this link that has none.
#pragma once

#include "shared.hxx"

#include "link.hxx"

namespace driveview
{

  // ---------------------------------------------------------------------------
  // WHY A DIGITAL THROTTLE NEEDS A CAP
  //
  // W is a switch. Without a cap it is full throttle the instant it goes down,
  // on a car whose ESC band is narrow and whose gearing was changed under it -
  // docs/conventions.md records that the brushless 21.5T at 10.71:1 breaks
  // static friction at a LOWER pulse than the 1541 the firmware still calls
  // idle. So the cap is the whole of the throttle's resolution, and the useful
  // default is a crawl.
  //
  // 100, and the arithmetic behind it: the old hub's forward key sent 1547 us
  // absolute, and against the committed 1541..1600 band (cal.hxx, mirrored in
  // trim.hxx) 1547 is (1547-1541)/59 of the way up, which is 102 milli. The
  // default is the round number beside it. THE MAPPING IS THE BOARD'S, not
  // this file's: the Pico turns throttleMilli into microseconds between the
  // limits the Trim pane set, so moving those limits moves what 100 means. That
  // is why the number here is a fraction and not a microsecond count.
  constexpr Int32 THROTTLE_CAP_DEFAULT = 100;

  // Full scale on this wire. bibowire refuses anything outside +-1000.
  constexpr Int32 THROTTLE_CAP_MAX = 1000;
  constexpr Int16 STEER_FULL = 1000;

  // What the keyboard said this frame. A struct rather than five arguments so
  // the pure functions below take one thing and the suite builds cases by name.
  struct Keys
  {
      Bool left = false;      // A
      Bool right = false;     // D
      Bool forward = false;   // W
      Bool brake = false;     // S
      Bool estop = false;     // Space
  };

  // ---- the mapping, pure and inline so the suite can reach it ----------------
  //
  // trim.hxx's reason: drive.cxx names ImGui, which cannot be linked into a
  // console test, and the arithmetic that decides what a key MEANS is the part
  // that has to be held to an answer. Defined here, viewer/tests reaches it
  // without linking this module at all.

  // BANG-BANG. Both keys down is 0 and not "the last one wins": a hand resting
  // on A while reaching for D is the case this is for, and a car that picked
  // one of them would turn while its operator believed it was straight.
  [[nodiscard]] inline Int16 steerFrom(const Keys& k)
  {
      if(k.left == k.right)
      {
          return 0;
      }
      return k.left ? static_cast<Int16>(-STEER_FULL) : static_cast<Int16>(STEER_FULL);
  }

  // S BEATS W, ALWAYS. A brake that W can override is not a brake - and W is
  // already held when somebody reaches for S, so "both down" is precisely the
  // moment the rule is for.
  //
  // Zero rather than negative: the band this project commands is forward-only
  // (bibowire::ESC_US_HARD_MIN is 1500 and the board refuses below it), so
  // there is no reverse to ask for and asking would be refused at the far end.
  [[nodiscard]] inline Int16 throttleFrom(const Keys& k, Int32 capMilli)
  {
      if(k.brake || !k.forward)
      {
          return 0;
      }
      if(capMilli <= 0)
      {
          return 0;
      }
      const Int32 held = capMilli > THROTTLE_CAP_MAX ? THROTTLE_CAP_MAX : capMilli;
      return static_cast<Int16>(held);
  }

  // ENABLE ON EVERY DATAGRAM WHILE THE OPERATOR INTENDS TO DRIVE. Not optional
  // and not an edge: deadman::step reaches STATE_LIVE only while `enable` is
  // set, so a stream that carried it once would be a car that went SOFT 150 ms
  // later with the keys looking dead.
  //
  // MOTOR_WANTED is deliberately not set. That bit is the LIDAR's motor
  // (section 6), which belongs to the scan and not to driving.
  [[nodiscard]] inline UInt16 buttonsFrom(const Keys& k, Bool driving)
  {
      UInt16 bits = 0;
      if(k.estop)
      {
          bits = static_cast<UInt16>(bits | bibowire::BUTTON_ESTOP);
      }
      if(driving)
      {
          bits = static_cast<UInt16>(bits | bibowire::BUTTON_ENABLE);
      }
      return bits;
  }

  // 0 manual, 1 look, 2 drive - bibowire::PilotMode's numbers. Anything else
  // folds to MANUAL, which is the mode whose stick values the board actually
  // reads, so an out-of-range selection cannot become a belief about a mode
  // nobody is in.
  [[nodiscard]] inline UInt8 modeOf(Int32 choice)
  {
      const UInt8 drive = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_DRIVE);
      if(choice < 0 || choice > static_cast<Int32>(drive))
      {
          return static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL);
      }
      return static_cast<UInt8>(choice);
  }

  // ---- the window ------------------------------------------------------------

  struct View
  {
      // Bound to ImGui::Begin's close button. Unlike the camera's, closing this
      // costs the board nothing and RELEASES NOTHING: the control slot belongs
      // to the connection, not to the window. What closing does is stop the
      // keys - the intent goes neutral with ENABLE clear, which is a SOFT stop
      // the board reaches in 150 ms, rather than silence, which is a full one
      // at 300. Said here because "I closed a window and the car stopped" has
      // to be a thing somebody can predict.
      Bool open = false;

      // THE OPERATOR'S HAND ON THE WHEEL. Off at startup and off again whenever
      // the pane is disabled for any reason, because this is the bit the
      // deadman treats as consent.
      Bool enabled = false;

      // These outlive the window being closed and reopened - camera.hxx's rule,
      // and the same reason: they describe how this operator drives, and
      // closing a window is not a decision to re-enter a setup.
      Int32 throttleCapMilli = THROTTLE_CAP_DEFAULT;

      // WHAT THE OPERATOR BELIEVES IS ACTIVE. bibowire::PilotMode as an Int32
      // because ImGui::Combo writes an int, folded through modeOf() on the way
      // to the wire.
      //
      // NEVER ASSIGNED FROM CTLSTATE'S pilotMode. The board compares the two to
      // catch a viewer driving under a false belief, and a viewer that copied
      // the board's answer into its own claim would make that comparison always
      // true and delete the check - a bug that was just fixed on the board side
      // of the same comparison. The pane shows both numbers side by side and
      // says when they disagree; it never reconciles them.
      Int32 assumedMode = 0;

      // COMMANDs this pane has sent, counted and shown. A verb the board never
      // answered is a fact worth seeing.
      UInt32 sent = 0;
  };

  // What the keys and the pane's own settings add up to. Pure, so the suite
  // holds it to an answer without a window or a socket.
  [[nodiscard]] inline link::Intent intentFrom(const Keys& k, const View& v)
  {
      link::Intent in;
      in.driving = v.enabled;
      // NEUTRAL ON BOTH AXES when not driving. link::buildControl enforces this
      // again at the wire; it is here as well so that what this pane DISPLAYS
      // and what it sends are the same object rather than two descriptions of
      // one intention.
      in.steerMilli = v.enabled ? steerFrom(k) : static_cast<Int16>(0);
      in.throttleMilli = v.enabled ? throttleFrom(k, v.throttleCapMilli) : static_cast<Int16>(0);
      // ESTOP IS NOT GATED BY THE ENABLE. The one thing that must work in every
      // state this window can be in is the stop.
      in.buttons = buttonsFrom(k, v.enabled);
      in.assumedMode = modeOf(v.assumedMode);
      return in;
  }

  // The DPI multiplier the layout uses. Called once, after ImGui exists. This
  // window owns no graphics resource, so there is no shutdown to match
  // camview's.
  Void init(Float32 scale);

  // One frame. Reads the keyboard when this window has focus and nothing is
  // being typed into, publishes the intent to the client EVERY frame - it is a
  // level and not an edge - and shows the board's own account of what it did.
  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs);

}
