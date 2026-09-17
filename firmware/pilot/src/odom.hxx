// odom - dead reckoning from the wheel encoder and the steering angle: the
// odometry bundle's working half. PURE: counts and a steering fraction in, a
// pose out; no clock, no device, no wire. app/main.cxx feeds it one step per
// Pico reply and publishes what it says as POSE.
//
// The model is a bicycle: the rear axle moves along the heading by the
// distance the ticks say, and the heading turns by distance / wheelbase x
// tan(steering angle). It is honest about what it is - integration of two
// nominal numbers - so the sigmas grow with distance and never shrink. The
// day the lidar corrects it, the correction lands on the pose here and the
// wire does not change.
//
// THE FRAME: the world is the car's own frame at the moment the bundle was
// loaded - X right, Y forward, metres in the viewer and millimetres on the
// wire - and the heading is the turn from +Y, COUNTER-CLOCKWISE POSITIVE seen
// from above. Steering is positive to the right (bibowire.hxx), so a right
// turn lowers the heading. scene.hxx's axes and the wire's POSE say the same.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"

namespace odom
{
  // MEASURED 2026-09-16 on the bench: both wheels of an axle turned one
  // revolution moved the count 64 either way (docs/hardware.md).
  constexpr Int32 TICKS_PER_WHEEL_TURN = 64;

  // NOMINAL, not yet measured: the TT-02's kit tyre is 66 mm across, and the
  // wheelbase is the kit's 257 mm. Steering lock is a guess at the wheel
  // angle at full travel; the trim's servo limits set the travel, not the
  // angle. Roll the car a measured distance and turn it a measured circle to
  // replace these with numbers (docs/bundles.md).
  constexpr Float32 WHEEL_DIAMETER_MM = 66.0f;
  constexpr Float32 WHEELBASE_MM = 257.0f;
  constexpr Float32 STEER_LOCK_RAD = 0.49f;

  // How the sigmas grow: a share of the distance for position, and radians
  // per metre for the heading. Guesses shaped like the truth: a scale error
  // is proportional, and a heading error comes from the lock guess, per turn.
  constexpr Float32 SIGMA_XY_SHARE = 0.03f;
  constexpr Float32 SIGMA_HEADING_RAD_PER_M = 0.05f;

  // POSE.source values. 0 is "none" on the wire.
  constexpr UInt8 SOURCE_DEAD_RECKONING = 1;

  struct Config
  {
      Float32 mmPerTick = 0.0f;
      Float32 wheelbaseMm = WHEELBASE_MM;
      Float32 steerLockRad = STEER_LOCK_RAD;
  };

  // The nominal numbers above, as a Config.
  [[nodiscard]] Config defaults();

  struct State
  {
      Float64 xMm = 0.0;
      Float64 yMm = 0.0;
      Float64 headingRad = 0.0;
      Float64 distanceMm = 0.0;    // unsigned: reverse adds to it too
      Int32 lastTicks = 0;
      Bool primed = false;         // a first count has been seen
      UInt32 steps = 0;            // replies integrated
  };

  // The first count after a load: where the wheel is, no motion. A step
  // before priming primes instead, so a bundle loaded mid-drive does not
  // read the whole count so far as a leap.
  Void prime(State& s, Int32 ticks);

  // One reply: the count now and the steering fraction the wheels ARE at
  // (steer_now, thousandths, positive right). Integrates the ticks since the
  // last step along the heading at the middle of the turn.
  Void step(State& s, Int32 ticks, Int32 steerMilli, const Config& cfg);

  // The state as the wire carries it. valid is 1 once primed.
  [[nodiscard]] bibowire::Pose toPose(const State& s, UInt64 tMonoUs);

  // The heading wrapped into (-pi, pi], for the wire and for tests.
  [[nodiscard]] Float64 wrapRad(Float64 rad);
}
