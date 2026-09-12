// The Trim window: the car's steering and throttle limits, its centre, and how
// fast either output is allowed to move - set from here, over bibowire COMMAND.
//
// ---------------------------------------------------------------------------
// `trimview`, NOT `trim`
//
// camera.cxx's reason, applied before it becomes a problem: main.cxx holds a
// `trimview::View trim;`, and a variable named `trim` would hide a namespace of
// that name for the rest of the function. The two modules are spelled the same
// way so the pattern is visible rather than incidental.
//
// ---------------------------------------------------------------------------
// WHAT THIS PANE CAN AND CANNOT PROMISE
//
// The Pico holds these in RAM and forgets them on a reboot, so the Pico is not
// where they are kept. THE BOARD saves every tuning change it accepts to a file
// on the Pi and re-sends the whole set whenever the Pico connects, and THIS
// LAPTOP saves the pane's values too (settings.hxx, beside bibo.exe). Two copies
// can drift apart - a change made from another viewer, or one the board refused
// - which is why the window has a "send all to the car" button, and why it says
// in one line where the values live rather than leaving that to this comment.
//
// The tuning verbs are also REFUSED WHILE THE CAR IS ARMED, with result = 3.
// Re-tuning the range a live throttle is being clamped to is the one way this
// window could hurt somebody. So the controls are disabled with the reason
// spelled out, rather than left draggable to produce a refusal - and the board's
// refusal is still shown when one arrives, because the disable is a courtesy and
// the board is the authority.
//
// ---------------------------------------------------------------------------
// EVERY MICROSECOND HERE IS AN INTEGER, AND THAT IS NOT A PREFERENCE
//
// There is not one Float32 slider in this file. ImGui prints a float through a
// printf format whose decimal point honours the locale; a machine set to a comma
// decimal writes "1,07", and this project has been bitten by exactly that three
// times (proto.cxx, scanwire.cxx, and the guide percentages in camera.hxx). The
// wire has no floating point on it either - bibowire carries µs as UInt16 - so an
// integer here is also the value that actually travels, with no conversion step
// to be wrong in.
//
// The one place a decimal point is unavoidable is the derived TIME readout, and
// it is built by integer arithmetic in centiseconds and printed as two integers
// with a '.' between them. Same rule, same reason, and no %f anywhere.
#pragma once

#include "shared.hxx"

#include "link.hxx"

namespace trimview
{

  // ---------------------------------------------------------------------------
  // THE CAR'S COMMITTED NUMBERS, MIRRORED - AND THIS IS A CROSS-BOUNDARY COPY
  //
  // The authority is firmware/lib/chassis/cal.hxx. It is NOT on this program's
  // include path (the viewer sees viewer/src, shared/ and firmware/pilot/src, and
  // nothing under firmware/lib), so these are retyped, which makes them the shape
  // of bug this project has a name for: two halves each locally correct and
  // broken as a pair. IF cal.hxx MOVES, THESE MOVE WITH IT.
  //
  // They are only the pane's STARTING POSITION, and only on a laptop with no
  // saved settings file. Nothing here is read back from the car - the protocol
  // has no "tell me your current limits" message - so these say "what the car
  // was last calibrated to", never "what the Pico is using right now". The
  // window says that too.
  constexpr Int32 STEER_MIN_DEFAULT = 1230;    // cal.hxx STEER_CAL_LEFT
  constexpr Int32 STEER_CENTRE_DEFAULT = 1480; // cal.hxx STEER_CAL_CENTER
  constexpr Int32 STEER_MAX_DEFAULT = 1660;    // cal.hxx STEER_CAL_RIGHT

  // MEASURED ON A MOTOR THAT IS GONE. cal.hxx is explicit: these came off a
  // brushed 1060 and a 540, and since 2026-09-06 the car is a QuicRun 10BL160
  // with a 21.5T brushless and a 17T pinion, which maps 1500..2000 almost
  // linearly instead of needing 41 µs of dead zone. So 1541 is probably already
  // creeping and 1600 is no longer a crawl. They stay because they are NARROW -
  // a 59 µs band cannot launch the car - and the pane presents them as a stale
  // starting point rather than as measured truth for this drivetrain.
  constexpr Int32 ESC_MIN_DEFAULT = 1541;      // cal.hxx THROTTLE_CAL_MIN
  constexpr Int32 ESC_MAX_DEFAULT = 1600;      // cal.hxx THROTTLE_CAL_MAX

  constexpr Int32 STEER_SLEW_DEFAULT = 8;      // cal.hxx SLEW_CAL_STEER
  constexpr Int32 THROTTLE_SLEW_DEFAULT = 8;   // cal.hxx SLEW_CAL_THROTTLE

  // ---- the derived arithmetic, which is the feature -------------------------
  //
  // µs-per-tick is a unit nobody has intuition about. µs-per-second is better and
  // a TIME is the one an operator actually thinks in: "lock to lock in a second"
  // is a thing a person can picture, "8 µs per 20 ms tick" is not.
  //
  // THESE ARE inline IN THE HEADER ON PURPOSE. It is orient.cxx's reason arrived
  // at from the other direction: the arithmetic a control's meaning depends on
  // must be reachable from viewer/tests, and trim.cxx names ImGui, which cannot
  // be linked into a console test. Defined here, the suite holds them to
  // hand-computed answers without linking this module at all.

  // At SLEW_TICKS_PER_S ticks a second. The Pico's tick is 20 ms, so 8 µs a tick
  // is 400 µs a second.
  [[nodiscard]] inline Int32 slewUsPerSec(Int32 usPerTick)
  {
      return usPerTick * static_cast<Int32>(bibowire::SLEW_TICKS_PER_S);
  }

  // HUNDREDTHS OF A SECOND to travel `spanUs` at `usPerTick`, or -1 when the
  // question has no answer - a span of zero, or a rate of zero, which would
  // otherwise be a divide by zero dressed up as "instant".
  //
  // Integer throughout and in centiseconds rather than seconds, so the one
  // decimal point this pane prints comes from splitting an integer and never
  // from a locale's idea of a radix character.
  [[nodiscard]] inline Int64 crossCentis(Int32 spanUs, Int32 usPerTick)
  {
      const Int32 rate = slewUsPerSec(usPerTick);
      if(spanUs <= 0 || rate <= 0)
      {
          return -1;
      }
      return (static_cast<Int64>(spanUs) * 100) / static_cast<Int64>(rate);
  }

  // ---- the window -----------------------------------------------------------

  struct View
  {
      // Bound to ImGui::Begin's close button, so the X in the corner closes it.
      // Unlike the camera's, this costs the board NOTHING while it is open -
      // there is no subscription behind it and no stream to switch off. It is
      // closed at startup only because a pane of limits is not what somebody
      // wants on screen while watching a point cloud.
      Bool open = false;

      // What the sliders are holding. These outlive the window being closed and
      // reopened, for camera.hxx's reason: they describe a SETUP, and closing a
      // window is not a decision to re-enter it.
      //
      // They are what the OPERATOR HAS ASKED FOR, never what the car has. The
      // board sends no reading of its own limits, so the difference matters and
      // the pane says which one it is showing.
      Int32 steerMinUs = STEER_MIN_DEFAULT;
      Int32 steerMaxUs = STEER_MAX_DEFAULT;
      Int32 steerTrimUs = STEER_CENTRE_DEFAULT;
      Int32 escMinUs = ESC_MIN_DEFAULT;
      Int32 escMaxUs = ESC_MAX_DEFAULT;

      // µs per 20 ms tick, SLEW_US_MIN..SLEW_US_MAX. Logarithmic on the slider,
      // because the interesting end is 1..20 and a linear 1..200 spends nine
      // tenths of its travel on rates that are all "immediately".
      Int32 steerSlewUs = STEER_SLEW_DEFAULT;
      Int32 throttleSlewUs = THROTTLE_SLEW_DEFAULT;

      // Counted and shown. A command this pane sent that the board never
      // answered is a fact worth seeing.
      UInt32 sent = 0;
  };

  // ---- clamping -------------------------------------------------------------
  //
  // Ctrl+click on an ImGui slider is a TEXT BOX, so the slider's own range is
  // not a guarantee about the value behind it. Everything is re-clamped before
  // it can be sent, which also keeps min below max - a pair the widgets cannot
  // enforce between them because each only knows its own number.
  //
  // IN THE HEADER since settings.cxx arrived: a saved file is a second way for
  // an out-of-range number to reach these fields, and the loader must clamp to
  // exactly the ranges the sliders use. One copy of the ranges, or two that
  // drift.

  [[nodiscard]] inline Int32 clampTo(Int32 value, Int32 lo, Int32 hi)
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

  inline Void settleSteer(View& v)
  {
      const Int32 hardLo = static_cast<Int32>(bibowire::SERVO_US_HARD_MIN);
      const Int32 hardHi = static_cast<Int32>(bibowire::SERVO_US_HARD_MAX);
      v.steerMinUs = clampTo(v.steerMinUs, hardLo, hardHi - 1);
      v.steerMaxUs = clampTo(v.steerMaxUs, v.steerMinUs + 1, hardHi);
      // THE CENTRE IS BOUNDED BY THE ENDS, not by the servo's range. A trim
      // outside the limits is a neutral the car can never reach, and cal.hxx is
      // explicit that 1500 has nothing to say about where a TT-02's wheels
      // point straight - so the ends are the only meaningful bound here.
      v.steerTrimUs = clampTo(v.steerTrimUs, v.steerMinUs, v.steerMaxUs);
  }

  inline Void settleEsc(View& v)
  {
      const Int32 hardLo = static_cast<Int32>(bibowire::ESC_US_HARD_MIN);
      const Int32 hardHi = static_cast<Int32>(bibowire::ESC_US_HARD_MAX);
      v.escMinUs = clampTo(v.escMinUs, hardLo, hardHi - 1);
      v.escMaxUs = clampTo(v.escMaxUs, v.escMinUs + 1, hardHi);
  }

  inline Void settleSlew(View& v)
  {
      const Int32 slewLo = static_cast<Int32>(bibowire::SLEW_US_MIN);
      const Int32 slewHi = static_cast<Int32>(bibowire::SLEW_US_MAX);
      v.steerSlewUs = clampTo(v.steerSlewUs, slewLo, slewHi);
      v.throttleSlewUs = clampTo(v.throttleSlewUs, slewLo, slewHi);
  }

  // Every field, in the order the pairs depend on each other.
  inline Void settleAll(View& v)
  {
      settleSteer(v);
      settleEsc(v);
      settleSlew(v);
  }

  // The DPI multiplier the layout uses. Called once, after ImGui exists. There is
  // no graphics device here - this window owns no texture - which is why there is
  // no shutdown to match it.
  Void init(Float32 scale);

  // One frame. Draws the window when it is open and sends a COMMAND for whatever
  // the operator just finished changing.
  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs);

}
