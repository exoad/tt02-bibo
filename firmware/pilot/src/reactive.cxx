#include "reactive.hxx"

#include <cmath>

namespace reactive
{
  namespace
  {
    Config cfg;

    constexpr Float32 PI_F = 3.14159265358979323846f;

    // The largest minHits configure() accepts. A fixed buffer rather than an
    // allocation: this runs every tick, and the number is a handful.
    constexpr Int32 HITS_CAP = 32;

    [[nodiscard]] Float32 toRad(const Float32 deg)
    {
        return deg * (PI_F / 180.0f);
    }

    // Relative to straight ahead, folded into -180..180 with POSITIVE MEANING
    // RIGHT - the sense the device counts in.
    [[nodiscard]] Float32 bearingOf(const Float32 rawDeg)
    {
        Float32 b = rawDeg - cfg.forwardDeg;
        while(b > 180.0f)
        {
            b -= 360.0f;
        }
        while(b < -180.0f)
        {
            b += 360.0f;
        }
        return b;
    }

    [[nodiscard]] Float32 clampF(const Float32 v, const Float32 lo, const Float32 hi)
    {
        if(v < lo)
        {
            return lo;
        }
        if(v > hi)
        {
            return hi;
        }
        return v;
    }

    // The `want` smallest values it is shown, so nth() is the Nth-nearest return
    // (trap 3). want == 1 is a plain minimum, the behaviour being avoided.
    struct Nearest
    {
        Float32 v[HITS_CAP] = {};
        Int32   n = 0;
        Int32   want = 1;

        Void add(const Float32 x)
        {
            if(n < want)
            {
                Int32 i = n++;
                while(i > 0 && v[i - 1] > x)
                {
                    v[i] = v[i - 1];
                    --i;
                }
                v[i] = x;
                return;
            }
            // Full, and this is no nearer than the furthest kept.
            if(n == 0 || x >= v[n - 1])
            {
                return;
            }
            Int32 i = n - 1;
            while(i > 0 && v[i - 1] > x)
            {
                v[i] = v[i - 1];
                --i;
            }
            v[i] = x;
        }

        [[nodiscard]] Bool full() const
        {
            return n >= want;
        }

        [[nodiscard]] Float32 nth() const
        {
            return n > 0 ? v[n - 1] : 0.0f;
        }
    };
  }

  CharSeq why(const Status s)
  {
      switch(s)
      {
      case Status::STATUS_OK:
          return "ok";
      case Status::STATUS_BLIND:
          return "too few lidar returns to drive on";
      case Status::STATUS_BAD_TUNING:
          return "tuning refused";
      }
      return "unknown";
  }

  CharSeq modeName(const Mode m)
  {
      switch(m)
      {
      case Mode::MODE_CRUISE:
          return "cruise";
      case Mode::MODE_SLOW:
          return "slow";
      case Mode::MODE_STOP:
          return "stop";
      case Mode::MODE_REVERSE:
          return "reverse";
      case Mode::MODE_BLIND:
          return "blind";
      }
      return "unknown";
  }

  const Config& tuning()
  {
      return cfg;
  }

  Bool configure(const Config& c)
  {
      if(!(c.clearMm > c.slowMm && c.slowMm > c.stopMm && c.stopMm > c.reverseMm
           && c.reverseMm > 0.0f))
      {
          return false;
      }
      if(c.halfWidthMm <= 0.0f || c.hysteresisMm < 0.0f)
      {
          return false;
      }
      if(c.frontArcDeg <= 0.0f || c.frontArcDeg > 90.0f)
      {
          return false;
      }
      if(!(c.sideNearDeg >= 0.0f && c.sideFarDeg > c.sideNearDeg && c.sideFarDeg <= 180.0f))
      {
          return false;
      }
      if(c.cruise <= 0.0f || c.cruise > 1.0f)
      {
          return false;
      }
      if(c.crawl <= 0.0f || c.crawl > c.cruise)
      {
          return false;
      }
      if(c.reverseThrottle <= 0.0f || c.reverseThrottle > 1.0f)
      {
          return false;
      }
      if(c.steerGain <= 0.0f)
      {
          return false;
      }
      if(c.minValid < 1 || c.minHits < 1 || c.minHits > HITS_CAP)
      {
          return false;
      }
      if(c.reverseMs <= 0 || c.reverseMaxMs < c.reverseMs)
      {
          return false;
      }
      cfg = c;
      return true;
  }

  Status step(const Ray* rays, const Size count, const Int32 dtMs, State* st, Outputs* out)
  {
      if(st == nullptr || out == nullptr)
      {
          return Status::STATUS_BLIND;
      }
      // The safe answer FIRST, so every early return below leaves a stop behind.
      out->steer = 0.0f;
      out->throttle = 0.0f;
      out->stop = true;
      out->clearanceMm = 0.0f;
      out->corridorHits = 0;
      out->mode = Mode::MODE_BLIND;
      Int32   valid = 0;
      Int32   hits = 0;
      Nearest ahead;
      ahead.want = cfg.minHits;
      // Side room starts at clearMm, not infinity: that keeps the steering ratio
      // bounded, so one side with no returns cannot produce full lock alone.
      Float32 leftRoom = cfg.clearMm;
      Float32 rightRoom = cfg.clearMm;
      for(Size i = 0; i < count && rays != nullptr; ++i)
      {
          const Float32 d = rays[i].distMm;
          // No return (trap 2): not a distance, and not a usable return either.
          if(!(d > 0.0f))
          {
              continue;
          }
          ++valid;
          const Float32 b = bearingOf(rays[i].angleDeg);
          const Float32 ab = std::fabs(b);
          if(ab > cfg.frontArcDeg)
          {
              continue;
          }
          const Float32 rad = toRad(b);
          const Float32 lateral = d * std::sin(rad);
          const Float32 forward = d * std::cos(rad);
          if(forward <= 0.0f)
          {
              continue;
          }
          if(std::fabs(lateral) <= cfg.halfWidthMm)
          {
              ahead.add(forward);
              ++hits;
          }
          if(ab >= cfg.sideNearDeg && ab <= cfg.sideFarDeg)
          {
              if(b < 0.0f)
              {
                  if(d < leftRoom)
                  {
                      leftRoom = d;
                  }
              }
              else if(d < rightRoom)
              {
                  rightRoom = d;
              }
          }
      }
      if(valid < cfg.minValid)
      {
          if(st->mode != Mode::MODE_BLIND)
          {
              st->mode = Mode::MODE_BLIND;
              st->modeMs = 0;
          }
          else
          {
              st->modeMs += (dtMs > 0 ? dtMs : 0);
          }
          out->mode = Mode::MODE_BLIND;
          return Status::STATUS_BLIND;
      }
      // Fewer than minHits corridor returns is an OPEN corridor: the price of not
      // braking for one speck is that a real obstacle needs minHits rays.
      const Float32 clearance = ahead.full() ? ahead.nth() : cfg.clearMm;
      // A normalised difference, bounded whatever the units: +-1 with a wall hard
      // against one side and the other clear, 0 for equal room.
      const Float32 sum = leftRoom + rightRoom;
      const Float32 toward =
          (sum > 0.0f) ? clampF(cfg.steerGain * ((rightRoom - leftRoom) / sum), -1.0f, 1.0f)
                       : 0.0f;
      st->modeMs += (dtMs > 0 ? dtMs : 0);
      Mode next = st->mode;
      if(st->mode == Mode::MODE_REVERSE)
      {
          if(st->modeMs >= cfg.reverseMaxMs)
          {
              // Wedged: stop rather than grind backwards into whatever is behind.
              next = Mode::MODE_STOP;
              st->wedged = true;
          }
          else if(st->modeMs >= cfg.reverseMs && clearance > cfg.stopMm + cfg.hysteresisMm)
          {
              next = Mode::MODE_SLOW;
          }
      }
      else
      {
          // Cleared by the same open test that lets the car leave a stop, so
          // giving up lasts exactly as long as its cause.
          if(clearance > cfg.stopMm + cfg.hysteresisMm)
          {
              st->wedged = false;
          }
          if(clearance <= cfg.reverseMm)
          {
              next = st->wedged ? Mode::MODE_STOP : Mode::MODE_REVERSE;
          }
          else if(clearance <= cfg.stopMm)
          {
              next = Mode::MODE_STOP;
          }
          else if(clearance <= cfg.slowMm)
          {
              next = Mode::MODE_SLOW;
          }
          else
          {
              next = Mode::MODE_CRUISE;
          }
          // Leaving a stop needs hysteresisMm more clearance than entering it.
          if(st->mode == Mode::MODE_STOP && next != Mode::MODE_STOP
             && next != Mode::MODE_REVERSE && clearance < cfg.stopMm + cfg.hysteresisMm)
          {
              next = Mode::MODE_STOP;
          }
      }
      if(next != st->mode)
      {
          st->mode = next;
          st->modeMs = 0;
          if(next == Mode::MODE_REVERSE)
          {
              // THE SIGN THAT IS EASY TO GET WRONG. Backing up, the nose swings
              // OPPOSITE the way the wheels point, so to end up facing the side
              // with room the wheels go the other way. Full lock: a gentle
              // reverse only backs into the same corner more slowly.
              st->reverseSteer = (toward >= 0.0f) ? -1.0f : 1.0f;
          }
      }
      out->mode = st->mode;
      out->clearanceMm = clearance;
      out->corridorHits = hits;
      switch(st->mode)
      {
      case Mode::MODE_CRUISE:
          out->steer = toward;
          out->throttle = cfg.cruise;
          out->stop = false;
          break;
      case Mode::MODE_SLOW:
      {
          // Linear from crawl at stopMm to cruise at slowMm. Not a curve: nothing
          // here knows the car's actual speed.
          const Float32 span = cfg.slowMm - cfg.stopMm;
          const Float32 t = (span > 0.0f) ? clampF((clearance - cfg.stopMm) / span, 0.0f, 1.0f)
                                          : 0.0f;
          out->steer = toward;
          out->throttle = cfg.crawl + t * (cfg.cruise - cfg.crawl);
          out->stop = false;
          break;
      }
      case Mode::MODE_STOP:
          // The wheels still point toward the room, so the first motion goes the
          // right way.
          out->steer = toward;
          out->throttle = 0.0f;
          out->stop = true;
          break;
      case Mode::MODE_REVERSE:
          out->steer = st->reverseSteer;
          out->throttle = -cfg.reverseThrottle;
          out->stop = false;
          break;
      case Mode::MODE_BLIND:
          out->stop = true;
          break;
      }
      out->steer = clampF(out->steer, -1.0f, 1.0f);
      out->throttle = clampF(out->throttle, -1.0f, 1.0f);
      return Status::STATUS_OK;
  }
}
