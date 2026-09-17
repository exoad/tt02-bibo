#include "motion.hxx"

#include <cmath>

#include "odom.hxx"

namespace motion
{
  Void feed(State& s, const bibowire::Odom& m)
  {
      s.have = true;
      if(s.primed && m.seq == s.lastSeq)
      {
          return;
      }
      if(s.primed)
      {
          const Int32 dTicks = m.ticks - s.lastTicks;
          s.travelM += static_cast<Float64>(dTicks) * static_cast<Float64>(odom::MM_PER_TICK) / 1000.0;
      }
      s.primed = true;
      s.lastTicks = m.ticks;
      s.lastSeq = m.seq;
      s.moving = m.ticksPerS != 0;
      s.dir = m.ticksPerS > 0 ? 1 : (m.ticksPerS < 0 ? -1 : 0);
      s.speedMps = std::fabs(static_cast<Float32>(m.ticksPerS)) * odom::MM_PER_TICK / 1000.0f;
  }

  Void lost(State& s)
  {
      s.have = false;
      s.moving = false;
      s.dir = 0;
      s.speedMps = 0.0f;
  }

  Str describe(const State& s)
  {
      if(!s.have)
      {
          return "--";
      }
      if(!s.moving)
      {
          return "stopped";
      }
      const Int32 cmps = static_cast<Int32>(std::lround(s.speedMps * 100.0f));
      const Str whole = std::to_string(cmps / 100);
      const Str hundredths = (cmps % 100 < 10 ? "0" : "") + std::to_string(cmps % 100);
      return Str(s.dir < 0 ? "reverse " : "forward ") + whole + "." + hundredths + " m/s";
  }
}
