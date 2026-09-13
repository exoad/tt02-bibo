// The Drive window: WASD turned into the CONTROL stream (docs/bibowire.md
// section 6), and the board's own account of what it did with it.
//
// `driveview`, not `drive`: main.cxx holds a `driveview::View drive;`, and
// bibo::drive is a firmware namespace.
//
// THREE GATES
// 1. The CONTROL SLOT is asked for in HELLO and nowhere else, so it is decided
//    when a connection is dialled. Asked for by default (link::Client::wantSlot
//    has the trade); holding it moves nothing while the car is disarmed.
// 2. ENABLE, a toggle in this window, default off: the operator saying "I am
//    driving now". bibowire::deadman::step reaches STATE_LIVE only while it is set.
// 3. WELCOME's capabilities bit 0 (canDrive) gates the whole pane. A pilot
//    started `--dry` clears it, and the window says why it is disabled.
//
// A and D move the steering toward full lock at `steerRateMilliPerS`; letting go
// springs it back to centre at the same rate; C centres it at once. The Pico's
// SLEW limits the servo in series, so the window shows this pane's value beside
// CTLSTATE's steerNowMilli to tell the two rates apart.
//
// EVERY STOP CENTRES AT ONCE: blocked, enable off, window closed or collapsed,
// ESTOP all put the steering straight to 0, so the next enable starts straight.
// Lost keyboard focus is not a stop: every key reads as up, so the wheel springs
// back as on a release.
#pragma once

#include "shared.hxx"

#include "link.hxx"

namespace driveview
{
  // W is a switch, so the cap is the throttle's whole resolution, and the
  // default is a crawl: docs/hardware.md records the brushless drivetrain
  // breaking static friction below the 1541 the firmware calls idle. The board
  // maps throttleMilli between the forward limits the Pico reports (set in
  // Trim), so this is a fraction, not microseconds; against the committed
  // 1541..1600 band, 100 is about a 1547 us crawl.
  constexpr Int32 THROTTLE_CAP_DEFAULT = 100;

  // Full scale on this wire. bibowire refuses anything outside +-1000.
  constexpr Int32 THROTTLE_CAP_MAX = 1000;
  constexpr Int16 STEER_FULL = 1000;

  // How fast A and D move the held steering, milli of full scale per second.
  // The default is centre to full lock in 667 ms. The slider and settings.cxx
  // both clamp to MIN..MAX.
  constexpr Int32 STEER_RATE_DEFAULT = 1500;
  constexpr Int32 STEER_RATE_MIN = 250;
  constexpr Int32 STEER_RATE_MAX = 5000;

  // The longest frame the ramp believes. A window drag or a breakpoint can
  // stall a frame for seconds, and trusting that dt would jump to full lock in
  // one frame with a key held.
  constexpr Int32 STEER_FRAME_MS_MAX = 100;

  // What the keyboard said this frame.
  struct Keys
  {
      Bool left = false;      // A
      Bool right = false;     // D
      Bool forward = false;   // W
      Bool brake = false;     // S
      Bool estop = false;     // Space
      Bool centre = false;    // C
  };

  // The key mapping is pure and inline so viewer/tests can check what a key
  // MEANS without linking ImGui.
  [[nodiscard]] inline Int32 clampMilli(Int32 value, Int32 lo, Int32 hi)
  {
      if(value < lo)
      {
          return lo;
      }
      if(value > hi)
      {
          return hi;
      }
      return value;
  }

  // Which way the keys push: full left, full right, or 0. steerHeldStep moves
  // toward it. Both keys down is 0, not last-wins: a hand resting on A while
  // reaching for D must not turn the wheel.
  [[nodiscard]] inline Int16 steerFrom(const Keys& k)
  {
      if(k.left == k.right)
      {
          return 0;
      }
      return k.left ? static_cast<Int16>(-STEER_FULL) : static_cast<Int16>(STEER_FULL);
  }

  // One frame of the held steering: `held` is last frame's value, `dtMs` this
  // frame's length.
  // C BEATS A AND D: it is pressed while a steering key is still down.
  // NO PUSH SPRINGS BACK toward 0 at the same rate and stops AT 0, never past
  // centre.
  // AT LEAST ONE MILLI whenever time has passed, or a slow rate on a fast frame
  // rounds to no movement at all.
  [[nodiscard]] inline Int16 steerHeldStep(Int16 held, const Keys& k, Int32 rateMilliPerS, Int32 dtMs)
  {
      if(k.centre)
      {
          return 0;
      }
      const Int32 from = clampMilli(held, -STEER_FULL, STEER_FULL);
      const Int32 dt = clampMilli(dtMs, 0, STEER_FRAME_MS_MAX);
      const Int32 rate = clampMilli(rateMilliPerS, STEER_RATE_MIN, STEER_RATE_MAX);
      Int32 step = (rate * dt) / 1000;
      if(step < 1 && dt > 0)
      {
          step = 1;
      }
      const Int16 push = steerFrom(k);
      if(push == 0)
      {
          if(from > 0)
          {
              return static_cast<Int16>(from > step ? from - step : 0);
          }
          return static_cast<Int16>(-from > step ? from + step : 0);
      }
      const Int32 to = push > 0 ? from + step : from - step;
      return static_cast<Int16>(clampMilli(to, -STEER_FULL, STEER_FULL));
  }

  // Milliseconds from centre to full lock at `rateMilliPerS`, for the readout;
  // -1 for a rate that never arrives.
  [[nodiscard]] inline Int32 steerLockMs(Int32 rateMilliPerS)
  {
      if(rateMilliPerS <= 0)
      {
          return -1;
      }
      return (static_cast<Int32>(STEER_FULL) * 1000) / rateMilliPerS;
  }

  // S is brake, then reverse: held, it asks for minus the cap. The ESC is in
  // Forward/Reverse/Brake mode, so the first push below neutral BRAKES and,
  // once S is released and the pulse is back at neutral, the next press
  // REVERSES (the transmitter's double tap). Only the SIGN reaches the ESC: the
  // pilot sends the Trim pane's reverse limit, which is off until set, so until
  // then S is a plain stop.
  //
  // S BEATS W, ALWAYS: W is already held when somebody reaches for S.
  [[nodiscard]] inline Int16 throttleFrom(const Keys& k, Int32 capMilli)
  {
      if(capMilli <= 0)
      {
          return 0;
      }
      const Int32 held = capMilli > THROTTLE_CAP_MAX ? THROTTLE_CAP_MAX : capMilli;
      if(k.brake)
      {
          return static_cast<Int16>(-held);
      }
      if(!k.forward)
      {
          return 0;
      }
      return static_cast<Int16>(held);
  }

  // ENABLE ON EVERY DATAGRAM while driving, not as an edge: deadman::step
  // reaches STATE_LIVE only while `enable` is set, so a stream that carried it
  // once would go SOFT 150 ms later.
  //
  // MOTOR_WANTED is never set here: it is the LIDAR's motor (section 6).
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
  // folds to MANUAL, the mode whose stick values the board actually reads.
  [[nodiscard]] inline UInt8 modeOf(Int32 choice)
  {
      const UInt8 drive = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_DRIVE);
      if(choice < 0 || choice > static_cast<Int32>(drive))
      {
          return static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL);
      }
      return static_cast<UInt8>(choice);
  }

  struct View
  {
      // Bound to ImGui::Begin's close button. Closing RELEASES NOTHING (the
      // control slot belongs to the connection) but stops the keys: the intent
      // goes neutral with ENABLE clear, a SOFT stop the board reaches in 150 ms,
      // rather than silence, a full stop at 300.
      Bool open = false;

      // The operator's consent, which the deadman reads. Off at startup and
      // whenever the pane is disabled. TICKED BY ARM once the board confirms the
      // car armed, since ARM was the consent; cleared when the board says it is
      // no longer armed. Unticking by hand is a soft stop until the next ARM.
      Bool enabled = false;

      // The armed state the board last reported, so ARM's confirmation is an
      // EDGE; as a level it would re-tick an enable unticked on purpose. Never
      // saved.
      Bool boardArmed = false;

      // Handed in by main.cxx from the Trim pane each frame. idleTest puts
      // BUTTON_IDLE_TEST on CONTROL (intentFrom); reverseOff is only for the
      // warning shown when S is pressed with nowhere below neutral to go.
      Bool idleTest = false;
      Bool reverseOff = false;

      // Settings: they outlive the window and are saved.
      Int32 throttleCapMilli = THROTTLE_CAP_DEFAULT;

      // What the OPERATOR believes is active: bibowire::PilotMode as an Int32
      // because ImGui::Combo writes an int, folded through modeOf() for the wire.
      //
      // NEVER ASSIGNED FROM CTLSTATE'S pilotMode. The board compares the two to
      // catch a viewer driving under a false belief, and copying its answer here
      // would make that check always pass. The pane shows both and never
      // reconciles them.
      Int32 assumedMode = 0;

      Int32 steerRateMilliPerS = STEER_RATE_DEFAULT;

      // Where the operator has left the wheel, -1000..+1000. NEVER SAVED: a
      // restored angle would turn the car the moment enable is ticked.
      // drawWindow zeroes it on every stop.
      Int16 steerHeldMilli = 0;

      // COMMANDs this pane has sent, shown in the window.
      UInt32 sent = 0;
  };

  // The saved fields clamped to their widgets' ranges, for settings.cxx. The
  // mode is FOLDED by modeOf, not clamped: a garbage 9 clamped would assert
  // DRIVE, and must become MANUAL.
  inline Void settle(View& v)
  {
      v.throttleCapMilli = clampMilli(v.throttleCapMilli, 0, THROTTLE_CAP_MAX);
      v.steerRateMilliPerS = clampMilli(v.steerRateMilliPerS, STEER_RATE_MIN, STEER_RATE_MAX);
      v.assumedMode = static_cast<Int32>(modeOf(v.assumedMode));
  }

  // What the keys and the pane's settings add up to. Pure, for the suite.
  // Steering comes from steerHeldMilli, already stepped this frame, so a key is
  // not counted twice; `k` decides throttle and ESTOP.
  [[nodiscard]] inline link::Intent intentFrom(const Keys& k, const View& v)
  {
      link::Intent in;
      in.driving = v.enabled;
      // NEUTRAL ON BOTH AXES when not driving. link::buildControl enforces it
      // again at the wire; doing it here too keeps what the pane displays and
      // what it sends the same object.
      const Int32 held = clampMilli(v.steerHeldMilli, -STEER_FULL, STEER_FULL);
      in.steerMilli = v.enabled ? static_cast<Int16>(held) : static_cast<Int16>(0);
      in.throttleMilli = v.enabled ? throttleFrom(k, v.throttleCapMilli) : static_cast<Int16>(0);
      // ESTOP IS NOT GATED BY THE ENABLE: the stop must work in every state.
      in.buttons = buttonsFrom(k, v.enabled);
      // The idle test rides only with ENABLE. The throttle is zeroed beside it
      // because the pilot ignores the field then.
      if(v.enabled && v.idleTest)
      {
          in.buttons = static_cast<UInt16>(in.buttons | bibowire::BUTTON_IDLE_TEST);
          in.throttleMilli = 0;
      }
      in.assumedMode = modeOf(v.assumedMode);
      return in;
  }

  // The DPI multiplier the layout uses. Called once, after ImGui exists. This
  // window owns no graphics resource, so there is no shutdown.
  Void init(Float32 scale);

  // One frame. Reads the keyboard when this window has focus and nothing is
  // being typed into, publishes the intent EVERY frame (a level, not an edge),
  // and shows the board's own account of what it did.
  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs);
}
