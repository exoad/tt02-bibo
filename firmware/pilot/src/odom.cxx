#include "odom.hxx"

#include <algorithm>
#include <cmath>

namespace odom
{
  namespace
  {
    constexpr Float64 PI = 3.14159265358979323846;
  }

  Config defaults()
  {
      Config c;
      c.mmPerTick = MM_PER_TICK;
      c.wheelbaseMm = WHEELBASE_MM;
      c.steerLockRad = STEER_LOCK_RAD;
      return c;
  }

  Float64 wrapRad(Float64 rad)
  {
      while(rad > PI)
      {
          rad -= 2.0 * PI;
      }
      while(rad <= -PI)
      {
          rad += 2.0 * PI;
      }
      return rad;
  }

  Void prime(State& s, Int32 ticks)
  {
      s.lastTicks = ticks;
      s.primed = true;
  }

  Void step(State& s, Int32 ticks, Int32 steerMilli, const Config& cfg)
  {
      if(!s.primed)
      {
          prime(s, ticks);
          return;
      }
      const Int32 dTicks = ticks - s.lastTicks;
      s.lastTicks = ticks;
      ++s.steps;
      if(dTicks == 0)
      {
          return;
      }
      const Float64 d = static_cast<Float64>(dTicks) * static_cast<Float64>(cfg.mmPerTick);
      const Float64 fraction = static_cast<Float64>(std::clamp(steerMilli, -1000, 1000)) / 1000.0;
      const Float64 delta = fraction * static_cast<Float64>(cfg.steerLockRad);
      // Positive steering is a right turn, which is clockwise, which is a
      // falling heading.
      const Float64 dHeading = -(d / static_cast<Float64>(cfg.wheelbaseMm)) * std::tan(delta);
      const Float64 mid = s.headingRad + (0.5 * dHeading);
      s.xMm += d * -std::sin(mid);
      s.yMm += d * std::cos(mid);
      s.headingRad = wrapRad(s.headingRad + dHeading);
      s.distanceMm += std::fabs(d);
  }

  bibowire::Pose toPose(const State& s, UInt64 tMonoUs)
  {
      bibowire::Pose p;
      p.tMonoUs = tMonoUs;
      p.xMm = static_cast<Int32>(std::lround(s.xMm));
      p.yMm = static_cast<Int32>(std::lround(s.yMm));
      p.headingMilliRad = static_cast<Int32>(std::lround(s.headingRad * 1000.0));
      p.sigmaXyMm = static_cast<UInt32>(std::lround(s.distanceMm * SIGMA_XY_SHARE));
      const Float64 metres = s.distanceMm / 1000.0;
      p.sigmaHeadingMilliRad = static_cast<UInt32>(std::lround(
          metres * SIGMA_HEADING_RAD_PER_M * 1000.0
      ));
      p.source = SOURCE_DEAD_RECKONING;
      p.valid = s.primed ? 1u : 0u;
      return p;
  }
}
