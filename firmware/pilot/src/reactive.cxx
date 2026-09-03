#include "reactive.hxx"

#include <cmath>

namespace reactive
{

  namespace
  {

    Config cfg;

    constexpr Float32 PI_F = 3.14159265358979323846f;

    // The largest minHits this file will hold, and therefore the largest
    // configure() will accept. A fixed buffer rather than an allocation: this
    // runs every tick and the number is a handful, not a parameter that grows.
    constexpr Int32 HITS_CAP = 32;

    [[nodiscard]] Float32 toRad(const Float32 deg)
    {
        return deg * (PI_F / 180.0f);
    }

    // The bearing of a raw lidar angle relative to straight ahead, folded into
    // -180..180 with POSITIVE MEANING RIGHT - the same sense the device counts
    // in, so the two never disagree.
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

    /*
     * Holds the N smallest values it is shown.
     *
     * The point of the whole structure is that nth() is the Nth-nearest return
     * rather than the nearest, which is what stops one spurious point braking
     * the car - see trap 3 in the header. With want == 1 it degenerates to a
     * plain minimum, which is exactly the behaviour being avoided.
     */
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

            // Full, and this is no nearer than the furthest we are keeping.
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
      /*
       * ORDERING IS THE POINT of most of this. A tuning where stopMm is further
       * out than slowMm produces a car that brakes and accelerates at the same
       * wall, which reads as a bug in the controller rather than in the number
       * somebody typed. Refused here rather than clamped, so the mistake is
       * visible at the moment it is made.
       */
      if(!(c.clearMm > c.slowMm && c.slowMm > c.stopMm && c.stopMm > c.reverseMm
           && c.reverseMm > 0.0f))
      {
          return false;
      }
      if(c.halfWidthMm <= 0.0f || c.hysteresisMm < 0.0f)
      {
          return false;
      }

      // Beyond 90 degrees "ahead" stops meaning anything: cos goes negative and
      // a thing behind the car would resolve as a thing in front of it.
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

      /*
       * The safe answer is written FIRST, so every path out of this function -
       * including the early returns below - leaves a stop behind rather than
       * whatever the caller happened to have in the struct.
       */
      out->steer = 0.0f;
      out->throttle = 0.0f;
      out->stop = true;
      out->clearanceMm = 0.0f;
      out->corridorHits = 0;
      out->mode = Mode::MODE_BLIND;

      // ---- read the scan ---------------------------------------------------
      Int32   valid = 0;
      Int32   hits = 0;
      Nearest ahead;
      ahead.want = cfg.minHits;

      /*
       * Side room starts at clearMm rather than at infinity. An empty side is
       * "as good as it gets", not "infinitely good" - which keeps the steering
       * ratio below bounded and stops one side with no returns from producing
       * full lock on its own.
       */
      Float32 leftRoom = cfg.clearMm;
      Float32 rightRoom = cfg.clearMm;

      for(Size i = 0; i < count && rays != nullptr; ++i)
      {
          const Float32 d = rays[i].distMm;

          // NO RETURN. Not a distance of zero, and not a vote toward the scan
          // being trustworthy either - see trap 2.
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

      // ---- blindness, which is its own answer ------------------------------
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

      /*
       * Fewer than minHits corridor returns is an OPEN corridor, not a near
       * one. This is the other half of trap 3: the price of not braking for a
       * single speck is that a genuine obstacle has to be seen by minHits rays
       * before it counts.
       */
      const Float32 clearance = ahead.full() ? ahead.nth() : cfg.clearMm;

      // ---- which way is there more room ------------------------------------
      //
      // A normalised difference, so it is bounded whatever the units: a wall
      // hard against one side with the other clear approaches +-1, and equal
      // room on both sides is exactly 0.
      const Float32 sum = leftRoom + rightRoom;
      const Float32 toward =
          (sum > 0.0f) ? clampF(cfg.steerGain * ((rightRoom - leftRoom) / sum), -1.0f, 1.0f)
                       : 0.0f;

      // ---- pick a mode ------------------------------------------------------
      st->modeMs += (dtMs > 0 ? dtMs : 0);

      Mode next = st->mode;

      if(st->mode == Mode::MODE_REVERSE)
      {
          if(st->modeMs >= cfg.reverseMaxMs)
          {
              // Wedged. Stopping is the honest outcome; grinding backwards into
              // whatever is behind the car is not. The flag is what makes it
              // STICK - see State::wedged.
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
          // One condition clears it, and it is the same "genuinely open" test
          // that lets the car leave a stop - so giving up lasts exactly as long
          // as the situation that caused it.
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

          // Leaving a stop costs more clearance than entering it did, so a car
          // parked exactly at the threshold does not stutter.
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
              /*
               * THE SIGN THAT IS EASY TO GET WRONG. Backing up, the nose swings
               * OPPOSITE the way the wheels point - so to end up facing the side
               * with room, the wheels go the other way. Full lock rather than a
               * proportional angle: a reverse is a manoeuvre to commit to, and a
               * gentle one just backs into the same corner more slowly.
               *
               * Decided ONCE, here, and held in the state for the whole
               * manoeuvre. Re-deciding it every tick with the nose against a
               * wall makes the wheels saw back and forth.
               */
              st->reverseSteer = (toward >= 0.0f) ? -1.0f : 1.0f;
          }
      }

      // ---- what that means for the two outputs -----------------------------
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
          // Linear between the two thresholds: crawling at stopMm, cruising at
          // slowMm. Not a curve, because nothing here knows the car's actual
          // speed and a curve would only be a more confident guess.
          const Float32 span = cfg.slowMm - cfg.stopMm;
          const Float32 t = (span > 0.0f) ? clampF((clearance - cfg.stopMm) / span, 0.0f, 1.0f)
                                          : 0.0f;
          out->steer = toward;
          out->throttle = cfg.crawl + t * (cfg.cruise - cfg.crawl);
          out->stop = false;
          break;
      }

      case Mode::MODE_STOP:
          // Wheels pre-positioned toward the room even while stopped, so the
          // first moment of motion is already going the right way.
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
