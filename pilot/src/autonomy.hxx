// One tick of the driving, as it will run on the companion board.
//
// ---------------------------------------------------------------------------
// WHY THIS IS ON THE PI AND NOT THE CAR
//
// The Pico holds the things that must not stop: the servo and ESC pulses, the
// slew limits, the deadman. Those are cheap, they are already written, and a
// board that is only ever asked to hold a pulse steady cannot be made late by
// anything the autonomy does.
//
// This side holds the thinking - odometry integration, the speed controller,
// pure pursuit, and eventually whatever the lidar becomes. It is the work whose
// cost is unpredictable, which is exactly the work that must not share a core
// with a 50 Hz servo update.
//
// The split is not about compute. An RP2350 could do pursuit arithmetic all day.
// It is about WHAT HAPPENS WHEN SOMETHING TAKES TOO LONG: on the Pi a late tick
// is a late command and the car keeps its last one under a deadman; on the Pico
// a late tick is a servo that stops being told anything.
//
// ---------------------------------------------------------------------------
// THE MATHS IS THE FIRMWARE'S, NOT A SECOND COPY
//
// geom, kinematics, pursuit, control and odom are pure headers that say in
// their own comments that they compile for the Pico, the Orange Pi and the host
// test from one copy. This file is the first thing to take them up on it.
//
// That matters more than it saves: two implementations of pure pursuit, one on
// each board, would agree until the day somebody fixed a sign in one of them.
//
// ---------------------------------------------------------------------------
// STUBBED, IN THE plan.hxx SENSE
//
// step() returns STATUS_NOT_IMPLEMENTED and touches no output. It does not
// return a plausible steering angle, because a plausible steering angle is
// indistinguishable from a working controller until the car is moving.
//
// What IS settled here: what the tick needs, what it produces, and what it is
// allowed to do when something is missing. Those outlive the implementation.
#pragma once

#include "shared.hxx"

#include "control.hxx"
#include "geom.hxx"
#include "kinematics.hxx"
#include "plan.hxx"
#include "pursuit.hxx"

namespace autonomy
{

  enum class Status
  {
      STATUS_OK = 0,

      // The stub. Never mistake it for a result.
      STATUS_NOT_IMPLEMENTED,

      STATUS_NO_LINK,        // the car is not reachable
      STATUS_NO_PATH,        // nothing to follow
      STATUS_NO_ODOM,        // no wheel data, so no idea how fast we are going
      STATUS_ARRIVED,        // the path is finished - not a fault
      STATUS_BAD_TUNING,

      // The car has not spoken within Config::silenceMs. The correct response
      // is to stop it, and the caller owns that decision - see the note on
      // link::silentForMs.
      STATUS_STALE,
  };

  [[nodiscard]] CharSeq why(Status s);

  // What the tick is told, gathered from the car's replies.
  //
  // ODOMETRY COMES FROM THE PICO, not from here. The encoder is wired to the
  // car and counting edges over a serial link would lose them; the board counts
  // and reports totals, and this side turns totals into a pose. That is also
  // why `ticks` is cumulative rather than a delta - a delta computed on the
  // sender cannot survive a dropped line, and a total can.
  struct Inputs
  {
      UInt32 ticks   = 0;      // cumulative wheel encoder count
      UInt64 atUs    = 0;      // when the board sampled it, by ITS clock
      Bool   haveOdom = false; // false until the encoder exists at all

      // Where the car believes it is. Integrated on this side from the ticks
      // above and the steering actually commanded.
      bibo::geom::Pose pose;
  };

  // What the tick decides. Microseconds and fractions, ready for proto.
  struct Outputs
  {
      Float32 steer = 0.0f;    // -1..1, as the board's STEER wants
      Int32   escUs = 0;       // absolute pulse, as the board's ESC wants
      Bool    stop  = false;   // send STOP instead of the two above
  };

  // The tunings this side owns.
  //
  // Separate from plan::Limits and control::Pid rather than duplicating them -
  // those are the firmware's and are configured through their own headers. This
  // holds only what belongs to the loop itself.
  struct Config
  {
      // How often the tick runs. 50 Hz matches the servo's own 20 ms frame, so
      // a command is ready exactly when the board can use one; faster only adds
      // commands the servo will never see.
      Float32 tickHz = 50.0f;

      // Longest the car may be silent before STATUS_STALE. Deliberately the
      // same default as link::Config::silenceMs, and deliberately a separate
      // number: one is about a dead cable and this is about a dead loop.
      Int32 silenceMs = 500;

      // Stop rather than coast when the path runs out. A car that coasts to a
      // halt at the end of a route ends up somewhere the route did not go.
      Bool stopOnArrival = true;
  };

  [[nodiscard]] const Config& tuning();

  // Refuses a Config that cannot be run, rather than clamping it into one that
  // can - a tick rate of zero is a mistake to report, not to round up to one.
  [[nodiscard]] Bool configure(const Config& c);

  // STUB. Will decide one tick's steering and throttle.
  //
  // `out` is untouched on anything but STATUS_OK, so a caller that ignores the
  // Status gets whatever it initialized - its own value, not one this file
  // invented.
  [[nodiscard]] Status step(const Inputs& in,
                            const bibo::pursuit::Path* path,
                            Outputs* out);

}
