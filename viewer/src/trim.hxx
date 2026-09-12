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
// These values live in the Pico's RAM. They do NOT survive a reboot or a
// reflash - docs/bibowire.md section 5 says so in as many words - and
// firmware/lib/chassis/cal.hxx is the file that does. A pane that let somebody
// tune for an hour without saying that is a pane that loses their afternoon, so
// it is written on the window itself and not in this comment alone.
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
  // They are only the pane's STARTING POSITION. Nothing here is read back from
  // the car - the protocol has no "tell me your current limits" message - so
  // these say "what the car was last calibrated to", never "what the Pico is
  // using right now". The window says that too.
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

  // The DPI multiplier the layout uses. Called once, after ImGui exists. There is
  // no graphics device here - this window owns no texture - which is why there is
  // no shutdown to match it.
  Void init(Float32 scale);

  // One frame. Draws the window when it is open and sends a COMMAND for whatever
  // the operator just finished changing.
  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs);

}
