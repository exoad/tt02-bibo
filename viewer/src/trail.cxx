#include "trail.hxx"

#include <cmath>

namespace trail
{
  namespace
  {
    [[nodiscard]] Float64 between(const bibowire::Pose& a, const bibowire::Pose& b)
    {
        const Float64 dx = static_cast<Float64>(b.xMm) - static_cast<Float64>(a.xMm);
        const Float64 dy = static_cast<Float64>(b.yMm) - static_cast<Float64>(a.yMm);
        return std::sqrt((dx * dx) + (dy * dy));
    }
  }

  Bool Trail::add(const bibowire::Pose& p)
  {
      if(!points.empty())
      {
          const Float64 d = between(points.back(), p);
          if(d < static_cast<Float64>(MIN_STEP_MM))
          {
              return false;
          }
          distanceMm += d;
      }
      points.push_back(p);
      if(points.size() > MAX_POINTS)
      {
          points.erase(points.begin());
      }
      return true;
  }

  Void Trail::mark(const bibowire::Pose& p)
  {
      marks.push_back(p);
  }

  Void Trail::clear()
  {
      points.clear();
      marks.clear();
      distanceMm = 0.0;
  }

  Point inCarFrame(const bibowire::Pose& now, const bibowire::Pose& p)
  {
      const Float64 dx = (static_cast<Float64>(p.xMm) - static_cast<Float64>(now.xMm)) / 1000.0;
      const Float64 dy = (static_cast<Float64>(p.yMm) - static_cast<Float64>(now.yMm)) / 1000.0;
      // Undo the car's heading: a point ahead of a car facing world -X (heading
      // +pi/2) is at world -X, and must come out on the car's +Y.
      const Float64 h = static_cast<Float64>(now.headingMilliRad) / 1000.0;
      const Float64 c = std::cos(h);
      const Float64 s = std::sin(h);
      Point out;
      out.x = static_cast<Float32>((dx * c) + (dy * s));
      out.y = static_cast<Float32>((-dx * s) + (dy * c));
      return out;
  }

  Vec<Point> allInCarFrame(const bibowire::Pose& now, const Poses& list)
  {
      Vec<Point> out;
      out.reserve(list.size());
      for(const bibowire::Pose& p : list)
      {
          out.push_back(inCarFrame(now, p));
      }
      return out;
  }
}
