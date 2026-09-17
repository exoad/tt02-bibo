// The dead reckoning, by hand: straight, a circle that closes, reverse that
// returns, and the frame's sign conventions, which are the part a person
// gets wrong.
#include "odom.hxx"

#include <cmath>
#include <cstdio>

namespace
{
  Int32 failures = 0;
  Int32 checks = 0;

  Void check(Bool ok, const Char* what)
  {
      ++checks;
      if(!ok)
      {
          ++failures;
          std::printf("  FAIL: %s\n", what);
      }
  }

  [[nodiscard]] Bool near(Float64 a, Float64 b, Float64 tol)
  {
      return std::fabs(a - b) <= tol;
  }

  // n replies of `perStep` ticks each at one steering fraction.
  Void drive(odom::State& s, Int32 n, Int32 perStep, Int32 steerMilli, const odom::Config& cfg)
  {
      for(Int32 i = 0; i < n; ++i)
      {
          odom::step(s, s.lastTicks + perStep, steerMilli, cfg);
      }
  }

  Void testDefaults()
  {
      std::printf("-- the nominal numbers --\n");
      const odom::Config c = odom::defaults();
      // 66 mm x pi / 64 ticks: a little over 3 mm a tick.
      check(near(c.mmPerTick, 3.24, 0.01), "a tick is about 3.24 mm with the kit tyre");
      check(c.wheelbaseMm == odom::WHEELBASE_MM, "the wheelbase is the TT-02's");
  }

  Void testPrimeAndStraight()
  {
      std::printf("-- priming, then straight ahead --\n");
      const odom::Config c = odom::defaults();
      odom::State s;
      bibowire::Pose p = odom::toPose(s, 5);
      check(
          p.valid == 0u && p.source == odom::SOURCE_DEAD_RECKONING,
          "not valid before a count, but says what it is"
      );
      // A bundle loaded with 5,000 ticks already on the count does not leap.
      odom::step(s, 5000, 0, c);
      check(
          s.primed && s.xMm == 0.0 && s.yMm == 0.0 && s.steps == 0u,
          "the first count primes and moves nothing"
      );
      // 640 ticks straight is ten wheel turns: 66 pi x 10 = 2,073 mm along +Y.
      drive(s, 10, 64, 0, c);
      check(near(s.yMm, 2073.5, 1.0), "ten wheel turns ahead is 2.07 m along +Y");
      check(
          near(s.xMm, 0.0, 1.0e-6) && near(s.headingRad, 0.0, 1.0e-9),
          "no drift sideways, no turn"
      );
      check(near(s.distanceMm, 2073.5, 1.0), "and the distance says the same");
      p = odom::toPose(s, 7);
      check(p.valid == 1u && p.tMonoUs == 7, "the pose is valid and stamped");
      check(
          p.yMm >= 2072 && p.yMm <= 2075 && p.xMm == 0 && p.headingMilliRad == 0,
          "in millimetres and milliradians"
      );
      check(
          p.sigmaXyMm == static_cast<UInt32>(std::lround(s.distanceMm * odom::SIGMA_XY_SHARE)),
          "the position sigma is a share of the distance"
      );
      check(
          p.sigmaHeadingMilliRad > 0u && p.sigmaHeadingMilliRad < 200u,
          "the heading sigma grows per metre"
      );
  }

  Void testReverseReturns()
  {
      std::printf("-- reverse goes back along the same line --\n");
      const odom::Config c = odom::defaults();
      odom::State s;
      odom::prime(s, 0);
      drive(s, 10, 64, 0, c);
      drive(s, 10, -64, 0, c);
      check(
          near(s.yMm, 0.0, 1.0e-6) && near(s.xMm, 0.0, 1.0e-6),
          "forward then the same in reverse is home"
      );
      check(near(s.distanceMm, 2.0 * 2073.5, 2.0), "and the distance counted both legs");
  }

  Void testSigns()
  {
      std::printf("-- steering right turns clockwise, which is toward +X --\n");
      const odom::Config c = odom::defaults();
      odom::State right;
      odom::prime(right, 0);
      drive(right, 20, 16, 1000, c);
      check(right.headingRad < 0.0, "full right lock lowers the heading");
      check(right.xMm > 0.0 && right.yMm > 0.0, "and the car goes forward and to the right, +X");
      odom::State left;
      odom::prime(left, 0);
      drive(left, 20, 16, -1000, c);
      check(left.headingRad > 0.0 && left.xMm < 0.0, "full left raises it and goes to -X");
      check(
          near(left.xMm, -right.xMm, 1.0e-6) && near(left.yMm, right.yMm, 1.0e-6),
          "mirror images"
      );
      // Reversing with right lock: the nose swings the other way, heading rises.
      odom::State back;
      odom::prime(back, 0);
      drive(back, 20, -16, 1000, c);
      check(
          back.headingRad > 0.0 && back.yMm < 0.0,
          "reverse with right lock raises the heading, going backward"
      );
  }

  Void testCircleCloses()
  {
      std::printf("-- a full circle at constant lock comes back to the start --\n");
      const odom::Config c = odom::defaults();
      // Turning radius L / tan(lock); a lap is 2 pi R of arc.
      const Float64 radius = c.wheelbaseMm / std::tan(c.steerLockRad);
      const Float64 lap = 2.0 * 3.14159265358979323846 * radius;
      const Int32 ticksPerLap = static_cast<Int32>(std::lround(lap / c.mmPerTick));
      odom::State s;
      odom::prime(s, 0);
      // One tick a step, so the midpoint integration is nearly exact.
      drive(s, ticksPerLap, 1, 1000, c);
      check(
          near(s.xMm, 0.0, 15.0) && near(s.yMm, 0.0, 15.0),
          "within 15 mm of home after a 3 m lap"
      );
      check(near(odom::wrapRad(s.headingRad), 0.0, 0.02), "and pointing the way it started");
      check(s.distanceMm > lap - 5.0 && s.distanceMm < lap + 5.0, "having gone one lap");
  }

  Void testWrap()
  {
      std::printf("-- the heading wraps --\n");
      check(near(odom::wrapRad(4.0), 4.0 - 6.283185307, 1.0e-9), "past pi comes round negative");
      check(near(odom::wrapRad(-4.0), -4.0 + 6.283185307, 1.0e-9), "past -pi comes round positive");
      check(odom::wrapRad(3.0) == 3.0, "inside stays");
  }
}

int main()
{
    std::printf("odom suite\n");
    testDefaults();
    testPrimeAndStraight();
    testReverseReturns();
    testSigns();
    testCircleCloses();
    testWrap();
    std::printf("%s: %d checks, %d failures\n", failures == 0 ? "PASS" : "FAIL", checks, failures);
    return failures == 0 ? 0 : 1;
}
