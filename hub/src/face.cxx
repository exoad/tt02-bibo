#include "face.hxx"

#include <algorithm>
#include <cmath>

namespace face
{

  namespace
  {

    constexpr Float32 PI_F = 3.14159265f;

    // The longest slice a spring is ever integrated in one go. Semi-implicit
    // Euler is stable while freq * dt stays well under 2, and the fastest spring
    // here is 28 rad/s - so 5 ms is 0.14, with room to spare for a config that
    // scales the frequencies up. A 200 ms hitch integrated in one leap does not
    // settle, it detonates.
    constexpr Float32 SUBSTEP_MS = 5.0f;

    // The frequency scale is 1 at this transitionMs. See Config::transitionMs.
    constexpr Float32 SPRING_REFERENCE_MS = 260.0f;

    // ---- the blink -------------------------------------------------------
    // 40 ms down, 90 ms up out of the 130 ms default. Down is the fast half:
    // a real lid slams shut and creeps back, and 50/50 reads as a shutter.
    constexpr Float32 BLINK_CLOSE_RATIO = 40.0f / 130.0f;

    // The reopen is a damped cosine rather than a curve that merely arrives, so
    // the lid goes a little PAST open and settles back. REOPEN_END is the value
    // that curve would land on at u = 1, subtracted off linearly so it lands on
    // exactly 1 instead and the end of a blink is not a step.
    constexpr Float32 REOPEN_DECAY = 4.0f;
    constexpr Float32 REOPEN_RING = 4.2f;
    constexpr Float32 REOPEN_END = 1.00898f;

    // ---- the wink beat ---------------------------------------------------
    constexpr Float32 WINK_CLOSE_MS = 70.0f;
    constexpr Float32 WINK_HOLD_MS = 90.0f;
    constexpr Float32 WINK_OPEN_MS = 150.0f;
    constexpr Float32 WINK_TOTAL_MS = WINK_CLOSE_MS + WINK_HOLD_MS + WINK_OPEN_MS;

    // How long after one wink ends before the face does it again, while WINK is
    // the chosen expression. A wink you have to re-press to see is a wink nobody
    // sees.
    constexpr Float32 WINK_REPEAT_S = 1.4f;

    // What the OTHER eye does about it: a widen and a small tip, peaking during
    // the hold. Without this a wink reads as one screen switching off.
    constexpr Float32 WINK_PARTNER_WIDEN = 0.075f;
    constexpr Float32 WINK_PARTNER_TILT = -3.5f;

    // ---- anticipation ----------------------------------------------------
    // The target is briefly pulled to the WRONG SIDE of where the eye already
    // is, not merely short of where it is going. That distinction is the whole
    // mechanism: an offset on the destination only softens the arrival, and what
    // makes a move read as having weight is the eye moving backwards first.
    constexpr Float32 ANTICIPATE_MS = 80.0f;
    constexpr Float32 ANTICIPATE_GAIN = 0.22f;
    constexpr Float32 ANTICIPATE_MIN_MOVE_PX = 45.0f;   // below this it is not a big move
    constexpr Float32 ANTICIPATE_MAX_PX = 26.0f;

    // ---- squash and stretch ----------------------------------------------
    // Width from the HEIGHT'S VELOCITY. A blink close runs 200 px of height away
    // in 40 ms, which is 5000 px per second, and the gain is set so that reads as
    // roughly a tenth wider. Clamped, because a hitch can hand this any rate at
    // all.
    constexpr Float32 SQUASH_GAIN = 0.0000225f;
    constexpr Float32 SQUASH_MAX = 0.14f;

    // ---- the gaze --------------------------------------------------------
    // A saccade is a flick and a settle. The spring is stiff and lightly damped,
    // which is what puts the settle in; the hold between them is what keeps the
    // face from looking nervous.
    constexpr Float32 SACCADE_REACH_PX = 14.0f;
    constexpr Float32 SACCADE_FREQ = 34.0f;
    constexpr Float32 SACCADE_DAMP = 0.52f;
    constexpr Float32 SACCADE_HOLD_MIN_S = 0.55f;
    constexpr Float32 SACCADE_HOLD_MAX_S = 2.20f;

    // The micro-twitches BETWEEN saccades: a velocity kick in pixels per second,
    // not a new target, so the eye jitters and comes back rather than drifting
    // somewhere new. A still face is allowed to be still; a frozen one is not.
    constexpr Float32 MICRO_HOLD_MIN_S = 0.35f;
    constexpr Float32 MICRO_HOLD_MAX_S = 1.30f;
    constexpr Float32 MICRO_KICK_PX = 34.0f;

    // The breathing drift. Slow enough that nobody can point at it and say what
    // is moving, fast enough that a still face looks alive. The height one stays
    // a FRACTION because it is a multiplier on whatever the eye's height is -
    // four pixels of sag is a lot on a sleepy slit and nothing on a wide stare.
    constexpr Float32 IDLE_RATE = 0.90f;    // radians per second
    constexpr Float32 IDLE_X_PX = 4.0f;
    constexpr Float32 IDLE_Y_PX = 5.0f;
    constexpr Float32 IDLE_HEIGHT = 0.045f;

    // SCANNING's sweep. Deliberately its own clock rather than the idle one, so
    // it keeps its rhythm while the drift wanders underneath it. 52 px each way
    // takes the narrow scanning bar most of the width of its panel.
    constexpr Float32 SCAN_RATE = 1.55f;
    constexpr Float32 SCAN_REACH_PX = 52.0f;

    // The guardrails. Overshoot is the point of the whole file, so these are the
    // line between "it rang past the target" and "half the eye is off the panel".
    constexpr Float32 MIN_W_PX = 6.0f;
    constexpr Float32 MAX_TILT = 45.0f;

    constexpr Float32 HALF_W = static_cast<Float32>(PANEL_W) * 0.5f;
    constexpr Float32 HALF_H = static_cast<Float32>(PANEL_H) * 0.5f;

    // The rotated rectangle's half-extents. A tilted rectangle needs more room
    // than an upright one, and how much more is the only arithmetic between
    // "it overshot" and "it went off the glass".
    Void extents(const Eye& e, Float32* ex, Float32* ey) noexcept
    {
        const Float32 a = e.tilt * 3.14159265f / 180.0f;
        const Float32 cs = std::fabs(std::cos(a));
        const Float32 sn = std::fabs(std::sin(a));
        *ex = e.w * 0.5f * cs + e.h * 0.5f * sn;
        *ey = e.w * 0.5f * sn + e.h * 0.5f * cs;
    }

    [[nodiscard]] Float32 clampF(Float32 v, Float32 lo, Float32 hi) noexcept
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

    [[nodiscard]] Float32 mix(Float32 a, Float32 b, Float32 k) noexcept
    {
        return a + (b - a) * k;
    }

    [[nodiscard]] Float32 smoothstep01(Float32 t) noexcept
    {
        const Float32 x = clampF(t, 0.0f, 1.0f);
        return x * x * (3.0f - 2.0f * x);
    }

    // One spring parameter, one sub-step. Semi-implicit Euler: the velocity is
    // updated FIRST and the position moved by the NEW velocity, which is what
    // makes it stable at the step sizes here where plain Euler would not be.
    Void integrate(Float32* pos, Float32* vel, Float32 target, const Spring& sp, Float32 dtS)
    {
        const Float32 k = sp.freq * sp.freq;
        const Float32 c = 2.0f * sp.damp * sp.freq;
        const Float32 accel = -k * (*pos - target) - c * (*vel);
        *vel += accel * dtS;
        *pos += (*vel) * dtS;
    }

    Void integrateEye(Eye* pos, EyeRate* vel, const Eye& want, const Spring& sp, Float32 dtS)
    {
        integrate(&pos->w, &vel->w, want.w, sp, dtS);
        integrate(&pos->h, &vel->h, want.h, sp, dtS);
        integrate(&pos->x, &vel->x, want.x, sp, dtS);
        integrate(&pos->y, &vel->y, want.y, sp, dtS);
        integrate(&pos->tilt, &vel->tilt, want.tilt, sp, dtS);
    }

    // xorshift32. Small, fast, no state outside the State, and reproducible -
    // which is the whole reason it is here rather than rand().
    [[nodiscard]] UInt32 nextBits(State* st) noexcept
    {
        UInt32 x = st->rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        st->rng = (x == 0u) ? 0x1BADB002u : x;
        return st->rng;
    }

    [[nodiscard]] Float32 nextUnit(State* st) noexcept
    {
        return static_cast<Float32>(nextBits(st) >> 8) / 16777216.0f;
    }

    [[nodiscard]] Float32 nextRange(State* st, Float32 lo, Float32 hi) noexcept
    {
        return lo + (hi - lo) * nextUnit(st);
    }

    // A lid coming back up, overshooting and settling. Shared by the blink's
    // reopen and the wink's, because they are the same motion at two speeds.
    [[nodiscard]] Float32 reopenCurve(Float32 u) noexcept
    {
        const Float32 t = clampF(u, 0.0f, 1.0f);
        const Float32 raw = 1.0f - std::exp(-REOPEN_DECAY * t) * std::cos(REOPEN_RING * t);
        return raw - (REOPEN_END - 1.0f) * t;
    }

    // The blink curve: 1 at both ends, 0 at the shut point and a little OVER 1
    // on the way back. MULTIPLIED into the height by the caller - see the trap
    // at the top of face.hxx.
    [[nodiscard]] Float32 blinkFactor(Float32 u) noexcept
    {
        if(u <= 0.0f || u >= 1.0f)
        {
            return 1.0f;
        }
        if(u < BLINK_CLOSE_RATIO)
        {
            return 1.0f - smoothstep01(u / BLINK_CLOSE_RATIO);
        }
        return reopenCurve((u - BLINK_CLOSE_RATIO) / (1.0f - BLINK_CLOSE_RATIO));
    }

    // The wink's own curve, in MILLISECONDS rather than a fraction, because its
    // three phases are three fixed durations and not three fractions of one.
    [[nodiscard]] Float32 winkFactor(Float32 t) noexcept
    {
        if(t < 0.0f || t >= WINK_TOTAL_MS)
        {
            return 1.0f;
        }
        if(t < WINK_CLOSE_MS)
        {
            return 1.0f - smoothstep01(t / WINK_CLOSE_MS);
        }
        if(t < WINK_CLOSE_MS + WINK_HOLD_MS)
        {
            return 0.0f;   // the hold. A blink landing in here changes nothing.
        }
        return reopenCurve((t - WINK_CLOSE_MS - WINK_HOLD_MS) / WINK_OPEN_MS);
    }

    // How much of the partner eye's sympathy beat is showing: 0 at both ends of
    // the wink, 1 in the middle of it.
    [[nodiscard]] Float32 winkPartner(Float32 t) noexcept
    {
        if(t < 0.0f || t >= WINK_TOTAL_MS)
        {
            return 0.0f;
        }
        return std::sin(PI_F * (t / WINK_TOTAL_MS));
    }

    // How ALIVE an expression is: what fraction of the saccades, the twitches
    // and the idle drift it gets. A dead face that is still wandering its eyes
    // around is the most obvious way to make DEAD read as "not implemented yet".
    [[nodiscard]] Float32 liveliness(Expression e) noexcept
    {
        if(e == Expression::EXPRESSION_DEAD)
        {
            return 0.0f;
        }
        if(e == Expression::EXPRESSION_SLEEPY)
        {
            return 0.30f;
        }
        if(e == Expression::EXPRESSION_FOCUSED || e == Expression::EXPRESSION_SCANNING)
        {
            // Both are LOOKING at something. A focused eye that keeps glancing
            // away is not focused, and a scanner has its own sweep already.
            return 0.25f;
        }
        return 1.0f;
    }

    [[nodiscard]] Float32 scanWeight(Expression e) noexcept
    {
        return (e == Expression::EXPRESSION_SCANNING) ? 1.0f : 0.0f;
    }

  }

  CharSeq name(Expression e) noexcept
  {
      switch(e)
      {
      case Expression::EXPRESSION_NEUTRAL:   return "Neutral";
      case Expression::EXPRESSION_HAPPY:     return "Happy";
      case Expression::EXPRESSION_SURPRISED: return "Surprised";
      case Expression::EXPRESSION_SLEEPY:    return "Sleepy";
      case Expression::EXPRESSION_FOCUSED:   return "Focused";
      case Expression::EXPRESSION_CONFUSED:  return "Confused";
      case Expression::EXPRESSION_SAD:       return "Sad";
      case Expression::EXPRESSION_ANGRY:     return "Angry";
      case Expression::EXPRESSION_SCANNING:  return "Scanning";
      case Expression::EXPRESSION_THINKING:  return "Thinking";
      case Expression::EXPRESSION_WINK:      return "Wink";
      case Expression::EXPRESSION_DEAD:      return "Dead";
      }
      return "Neutral";
  }

  // ---------------------------------------------------------------------------
  // HOW each expression moves. This table is half of what makes them read as
  // twelve moods rather than twelve rectangles: SURPRISED arrives in three
  // frames and rings, SLEEPY sags into place over most of a second, FOCUSED
  // snaps and then holds absolutely still.
  //
  // Damping below 1 overshoots. Nothing here is at 1 except the one that is
  // meant to look like it has stopped.
  // ---------------------------------------------------------------------------
  Spring springOf(Expression e) noexcept
  {
      switch(e)
      {
      case Expression::EXPRESSION_SURPRISED: return Spring{ 28.0f, 0.42f };
      case Expression::EXPRESSION_HAPPY:     return Spring{ 21.0f, 0.48f };
      case Expression::EXPRESSION_WINK:      return Spring{ 22.0f, 0.45f };
      case Expression::EXPRESSION_ANGRY:     return Spring{ 24.0f, 0.55f };
      case Expression::EXPRESSION_CONFUSED:  return Spring{ 18.0f, 0.50f };
      case Expression::EXPRESSION_FOCUSED:   return Spring{ 22.0f, 0.90f };
      case Expression::EXPRESSION_SCANNING:  return Spring{ 20.0f, 0.75f };
      case Expression::EXPRESSION_THINKING:  return Spring{ 13.0f, 0.65f };
      case Expression::EXPRESSION_SAD:       return Spring{ 10.0f, 0.80f };
      case Expression::EXPRESSION_SLEEPY:    return Spring{ 7.0f, 0.95f };
      case Expression::EXPRESSION_DEAD:      return Spring{ 6.0f, 1.00f };
      case Expression::EXPRESSION_NEUTRAL:   break;
      }
      return Spring{ 17.0f, 0.60f };
  }

  // ---------------------------------------------------------------------------
  // The twelve faces, in five numbers each.
  //
  // Designated initializers throughout, because five bare floats in a row say
  // nothing about which is a width and which an angle, and the struct's defaults
  // ARE the neutral face - so each case below says only what makes that
  // expression different, which is also the shortest way to write it.
  //
  // The one thing worth reading twice is WHERE THE SLIT SITS on the half-closed
  // faces. A happy squint closes from the BOTTOM up, so what is left of the eye
  // sits HIGH; a sleepy droop closes from the TOP down, so what is left sits
  // LOW. Same height, opposite y, and that sign is the whole difference between
  // the two - get it backwards and HAPPY reads as tired.
  //
  // WINK's resting pose is SYMMETRIC, which looks like a bug and is not: the
  // wink is a beat rather than a pose now, so what the table holds is the
  // friendly half-squint the face wears between winks.
  // ---------------------------------------------------------------------------
  Pose posed(Expression e) noexcept
  {
      Pose p;   // neutral, straight out of the defaults in face.hxx

      switch(e)
      {
      case Expression::EXPRESSION_NEUTRAL:
          // 150 x 200 centered in a 240 x 320 panel: 1 : 1.33 upright, with 45 px
          // of margin at the sides and 60 px top and bottom. Straight out of the
          // defaults in face.hxx.
          break;

      case Expression::EXPRESSION_HAPPY:
          // The slit sits HIGH: a happy squint closes from the bottom up, so
          // what is left of the eye is its top half. The height comes down and
          // the width goes UP, which makes a wide bar rather than a small square.
          p.left = Eye{ .w = 190.0f, .h = 60.0f, .y = -66.0f };
          p.right = p.left;
          break;

      case Expression::EXPRESSION_SURPRISED:
          // Taller AND wider, and nearly the full height of the panel. 168 x 290
          // is 1 : 1.73, so it goes up rather than round.
          p.left = Eye{ .w = 168.0f, .h = 290.0f };
          p.right = p.left;
          break;

      case Expression::EXPRESSION_SLEEPY:
          // Low, thin and tipped outward, arriving on the slowest spring in the
          // table - sleepiness is as much about the sag as about the shape.
          p.left = Eye{ .w = 172.0f, .h = 38.0f, .y = 78.0f, .tilt = -8.0f };
          p.right = p.left;
          break;

      case Expression::EXPRESSION_FOCUSED:
          // A level slit that arrives and stops - see its spring, the most damped
          // in the table. FOCUSED is not "a little bit angry", it is an
          // expression with no ring in it at all.
          p.left = Eye{ .w = 186.0f, .h = 70.0f, .tilt = 3.0f };
          p.right = p.left;
          break;

      case Expression::EXPRESSION_CONFUSED:
          // One eye tall, one squashed to a bar, tilted opposite ways, and both
          // looking off-axis. The asymmetry IS the expression - symmetric
          // confusion is just mild surprise.
          p.left = Eye{ .w = 126.0f, .h = 250.0f, .x = -26.0f, .y = -12.0f, .tilt = -12.0f };
          p.right = Eye{ .w = 126.0f, .h = 92.0f, .x = -26.0f, .y = 22.0f, .tilt = 12.0f };
          break;

      case Expression::EXPRESSION_SAD:
          // Inner ends UP, which is a negative tilt by this module's convention,
          // and with no brows to help it is the entire sadness cue.
          p.left = Eye{ .w = 148.0f, .h = 106.0f, .y = 26.0f, .tilt = -26.0f };
          p.right = p.left;
          break;

      case Expression::EXPRESSION_ANGRY:
          // The exact mirror of SAD in the cue they share, and it arrives more
          // than twice as fast: anger is sudden and sadness is not, which the
          // springs say more clearly than the shapes do. At 28 degrees a 180 px
          // bar sweeps 109 px each side, which is why it is not wider.
          p.left = Eye{ .w = 180.0f, .h = 124.0f, .y = -12.0f, .tilt = 28.0f };
          p.right = p.left;
          break;

      case Expression::EXPRESSION_SCANNING:
          // Narrow and upright - an instrument rather than an eye. The identity
          // of this one is the SWEEP rather than the shape: 96 px of eye with
          // 52 px of travel each way fills the panel with movement.
          p.left = Eye{ .w = 96.0f, .h = 194.0f };
          p.right = p.left;
          break;

      case Expression::EXPRESSION_THINKING:
          p.left = Eye{ .w = 112.0f, .h = 172.0f, .x = 40.0f, .y = -60.0f, .tilt = -6.0f };
          p.right = p.left;
          p.right.tilt = 2.0f;
          break;

      case Expression::EXPRESSION_WINK:
          // Symmetric ON PURPOSE - the wink itself is the beat in step(), and
          // this is the friendly face it wears between them.
          p.left = Eye{ .w = 160.0f, .h = 122.0f, .y = -14.0f };
          p.right = p.left;
          break;

      case Expression::EXPRESSION_DEAD:
          // A 12 px bar, and nothing else about it moves: liveliness() gives it
          // no drift, no twitch and no saccade, and its spring is the only
          // critically damped one in the table. Everything else here bounces;
          // this is the one that must not.
          p.left = Eye{ .w = 190.0f, .h = 12.0f };
          p.right = p.left;
          break;
      }

      return p;
  }

  Bool insidePanel(const Eye& e) noexcept
  {
      Float32 ex = 0.0f;
      Float32 ey = 0.0f;
      extents(e, &ex, &ey);
      return e.w > 0.0f
          && e.h >= 0.0f
          && std::fabs(e.x) + ex <= HALF_W + 0.001f
          && std::fabs(e.y) + ey <= HALF_H + 0.001f;
  }

  namespace
  {

    // The guardrail, and the only place the panel's edges are enforced.
    //
    // Overshoot is the point of this module, so "do not exceed the target" is
    // not the rule - a spring that could not go past its target would not be a
    // spring. The rule is that whatever it rings out to still fits on 240 x 320.
    //
    // Order matters. Shrink the HEIGHT first, then the width, then re-center:
    // shrinking the width can only reduce the vertical extent, so doing height
    // first cannot be undone by doing width second. Doing it the other way round
    // needs a second pass and one is enough.
    Void fitToPanel(Eye* e) noexcept
    {
        e->tilt = clampF(e->tilt, -MAX_TILT, MAX_TILT);
        e->w = clampF(e->w, MIN_W_PX, static_cast<Float32>(PANEL_W));
        e->h = clampF(e->h, 0.0f, static_cast<Float32>(PANEL_H));

        const Float32 a = e->tilt * PI_F / 180.0f;
        const Float32 cs = std::fabs(std::cos(a));   // >= 0.707: tilt is capped at 45
        const Float32 sn = std::fabs(std::sin(a));

        Float32 ey = e->w * 0.5f * sn + e->h * 0.5f * cs;
        if(ey > HALF_H)
        {
            e->h = std::max(0.0f, (HALF_H - e->w * 0.5f * sn) / cs * 2.0f);
            ey = e->w * 0.5f * sn + e->h * 0.5f * cs;
        }

        Float32 ex = e->w * 0.5f * cs + e->h * 0.5f * sn;
        if(ex > HALF_W)
        {
            e->w = std::max(MIN_W_PX, (HALF_W - e->h * 0.5f * sn) / cs * 2.0f);
            ex = e->w * 0.5f * cs + e->h * 0.5f * sn;
            ey = e->w * 0.5f * sn + e->h * 0.5f * cs;
        }

        const Float32 roomX = std::max(0.0f, HALF_W - ex);
        const Float32 roomY = std::max(0.0f, HALF_H - ey);
        e->x = clampF(e->x, -roomX, roomX);
        e->y = clampF(e->y, -roomY, roomY);
    }

  }

  Void wink(State* st) noexcept
  {
      if(st == nullptr)
      {
          return;
      }
      if(st->winkT >= 0.0f)
      {
          return;   // one beat at a time; restarting it mid-hold is a stutter
      }
      st->winkT = 0.0f;
  }

  Void set(State* st, Expression e) noexcept
  {
      if(st == nullptr)
      {
          return;
      }

      // ANTICIPATION. Before a big height change, lean the target briefly the
      // OTHER way: an eye about to fly open dips first, an eye about to sag
      // lifts first. Neither is visible AS itself - what is visible is that the
      // move afterwards has weight.
      const Pose    want = posed(e);
      const Float32 dL = want.left.h - st->shape.left.h;
      const Float32 dR = want.right.h - st->shape.right.h;

      // The anticipation POINT, not an offset: where the height is pulled to
      // before it is allowed to go anywhere. Small moves get none, because an
      // eye that backs up before every twitch reads as broken rather than alive.
      st->anticipLeftH = st->shape.left.h;
      st->anticipRightH = st->shape.right.h;
      const Float32 tall = static_cast<Float32>(PANEL_H);
      if(std::fabs(dL) > ANTICIPATE_MIN_MOVE_PX)
      {
          const Float32 back =
              clampF(-dL * ANTICIPATE_GAIN, -ANTICIPATE_MAX_PX, ANTICIPATE_MAX_PX);
          st->anticipLeftH = clampF(st->shape.left.h + back, 0.0f, tall);
      }
      if(std::fabs(dR) > ANTICIPATE_MIN_MOVE_PX)
      {
          const Float32 back =
              clampF(-dR * ANTICIPATE_GAIN, -ANTICIPATE_MAX_PX, ANTICIPATE_MAX_PX);
          st->anticipRightH = clampF(st->shape.right.h + back, 0.0f, tall);
      }
      st->anticipMs = ANTICIPATE_MS;

      st->source = st->target;
      st->target = e;
      st->blend = 0.0f;

      // There is no `from` pose to record and no progress to reset. The springs
      // are already where the face is and already carry its velocity, so
      // interrupting a move CONTINUES it - which is the one thing the eased
      // version this replaced had to be written carefully to fake.
      if(e == Expression::EXPRESSION_WINK)
      {
          wink(st);
          st->winkAgainS = WINK_REPEAT_S;
      }
  }

  Void blink(State* st) noexcept
  {
      if(st == nullptr)
      {
          return;
      }
      if(st->blinkT >= 0.0f)
      {
          return;   // already blinking - restarting one mid-close is a stutter
      }
      st->blinkT = 0.0f;
  }

  Bool blinking(const State& st) noexcept
  {
      return st.blinkT >= 0.0f;
  }

  Bool winking(const State& st) noexcept
  {
      return st.winkT >= 0.0f;
  }

  Void step(State* st, const Config& cfg, Float32 dtMs) noexcept
  {
      if(st == nullptr)
      {
          return;
      }

      // Speed scales the CLOCK, not any of the durations, so 0.25x and 3x are
      // the same animation played slower and faster rather than a different one.
      // Written as `!(dt > 0)` so a NaN dt freezes rather than poisoning the
      // pose - every field below is multiplied by something derived from it.
      const Float32 dt = dtMs * cfg.speed;
      if(!(dt > 0.0f))
      {
          return;
      }
      const Float32 dtS = dt * 0.001f;

      if(!st->seeded)
      {
          st->seeded = true;
          st->shape = posed(st->target);
          st->nextBlinkS = nextRange(st, cfg.blinkEveryMinS, cfg.blinkEveryMaxS);
          st->saccadeHoldS = nextRange(st, SACCADE_HOLD_MIN_S, SACCADE_HOLD_MAX_S);
          st->microHoldS = nextRange(st, MICRO_HOLD_MIN_S, MICRO_HOLD_MAX_S);
      }

      // ---- the springs ------------------------------------------------------
      const Float32 scale =
          SPRING_REFERENCE_MS / ((cfg.transitionMs > 40.0f) ? cfg.transitionMs : 40.0f);

      Spring sp = springOf(st->target);
      sp.freq *= scale;

      Pose want = posed(st->target);

      // The anticipation moves the TARGET, never the position, which is why it
      // never shows as a jump: the spring smooths the step for free. It blends
      // out over its window, so the eye backs up, stops backing up, and is
      // already moving the right way by the time the window closes.
      if(st->anticipMs > 0.0f)
      {
          const Float32 f = st->anticipMs / ANTICIPATE_MS;
          want.left.h = mix(want.left.h, st->anticipLeftH, f);
          want.right.h = mix(want.right.h, st->anticipRightH, f);
          st->anticipMs -= dt;
          if(st->anticipMs < 0.0f)
          {
              st->anticipMs = 0.0f;
          }
      }

      // SUB-STEPPED. One long frame integrated in a single leap is how a spring
      // goes to infinity; the slices are equal, so the result is a function of
      // dt alone and the run stays reproducible.
      const Int32   steps = std::max(1, static_cast<Int32>(std::ceil(dt / SUBSTEP_MS)));
      const Float32 sub = dtS / static_cast<Float32>(steps);
      for(Int32 i = 0; i < steps; ++i)
      {
          integrateEye(&st->shape.left, &st->rate.left, want.left, sp, sub);
          integrateEye(&st->shape.right, &st->rate.right, want.right, sp, sub);
      }

      // The blend is no longer the shape's business - the springs own that - but
      // the sweep and the liveliness still cross-fade on it, so leaving SCANNING
      // fades the sweep out instead of dropping it on one frame.
      if(st->blend < 1.0f)
      {
          st->blend += dt / SPRING_REFERENCE_MS;
          if(st->blend > 1.0f)
          {
              st->blend = 1.0f;
          }
      }
      const Float32 k = smoothstep01(st->blend);

      // ---- the blink --------------------------------------------------------
      //
      // Its own timeline, laid over the springs rather than folded into them.
      // That is what lets a blink land in the middle of a move without either
      // one jumping: the springs own the RESTING height and the blink owns a
      // multiplier on it, and neither writes the other's number.
      const Float32 blinkSpan = (cfg.blinkMs > 1.0f) ? cfg.blinkMs : 1.0f;

      if(cfg.autoBlink && st->blinkT < 0.0f)
      {
          st->nextBlinkS -= dtS;
          if(st->nextBlinkS <= 0.0f)
          {
              st->blinkT = 0.0f;
          }
      }

      if(st->blinkT >= 0.0f)
      {
          st->blinkT += dt;
          if(st->blinkT >= blinkSpan)
          {
              // Redrawn at the END, so the interval is a gap between blinks
              // rather than a period a long blink could eat into. A forced blink
              // reschedules the automatic one for free.
              st->blinkT = -1.0f;
              st->nextBlinkS = nextRange(st, cfg.blinkEveryMinS, cfg.blinkEveryMaxS);
          }
      }

      const Float32 lid = (st->blinkT >= 0.0f) ? blinkFactor(st->blinkT / blinkSpan) : 1.0f;

      // ---- the wink beat ----------------------------------------------------
      if(st->winkT >= 0.0f)
      {
          st->winkT += dt;
          if(st->winkT >= WINK_TOTAL_MS)
          {
              st->winkT = -1.0f;
              st->winkAgainS = WINK_REPEAT_S;
          }
      }
      else if(st->target == Expression::EXPRESSION_WINK)
      {
          st->winkAgainS -= dtS;
          if(st->winkAgainS <= 0.0f)
          {
              st->winkT = 0.0f;
          }
      }

      const Float32 winkMul = winkFactor(st->winkT);
      const Float32 partner = winkPartner(st->winkT);

      // ---- the gaze ---------------------------------------------------------
      st->saccadeHoldS -= dtS;
      if(st->saccadeHoldS <= 0.0f)
      {
          st->gazeToX = nextRange(st, -SACCADE_REACH_PX, SACCADE_REACH_PX);
          st->gazeToY = nextRange(st, -SACCADE_REACH_PX * 0.6f, SACCADE_REACH_PX * 0.6f);
          st->saccadeHoldS = nextRange(st, SACCADE_HOLD_MIN_S, SACCADE_HOLD_MAX_S);
      }

      // A twitch is a KICK to the velocity rather than a new target: the eye
      // jolts and the spring pulls it straight back, which is what a real
      // micro-movement looks like and what a second target would not be.
      st->microHoldS -= dtS;
      if(st->microHoldS <= 0.0f)
      {
          st->gazeVelX += nextRange(st, -MICRO_KICK_PX, MICRO_KICK_PX);
          st->gazeVelY += nextRange(st, -MICRO_KICK_PX * 0.6f, MICRO_KICK_PX * 0.6f);
          st->microHoldS = nextRange(st, MICRO_HOLD_MIN_S, MICRO_HOLD_MAX_S);
      }

      const Spring gazeSp{ SACCADE_FREQ * scale, SACCADE_DAMP };
      for(Int32 i = 0; i < steps; ++i)
      {
          integrate(&st->gazeX, &st->gazeVelX, st->gazeToX, gazeSp, sub);
          integrate(&st->gazeY, &st->gazeVelY, st->gazeToY, gazeSp, sub);
      }
      st->gazeX = clampF(st->gazeX, -HALF_W, HALF_W);
      st->gazeY = clampF(st->gazeY, -HALF_H, HALF_H);

      // ---- the drift and the sweep -----------------------------------------
      st->idlePhase += dtS * IDLE_RATE;
      st->scanPhase += dtS * SCAN_RATE;
      if(st->idlePhase > 2.0f * PI_F * 1024.0f)
      {
          st->idlePhase -= 2.0f * PI_F * 1024.0f;   // a Float32 loses its fraction eventually
      }
      if(st->scanPhase > 2.0f * PI_F * 1024.0f)
      {
          st->scanPhase -= 2.0f * PI_F * 1024.0f;
      }

      const Float32 live = mix(liveliness(st->source), liveliness(st->target), k);
      const Float32 scan = mix(scanWeight(st->source), scanWeight(st->target), k);
      const Float32 idleOn = cfg.idleMotion ? 1.0f : 0.0f;

      const Float32 driftX = std::sin(st->idlePhase * 0.61f) * IDLE_X_PX;
      const Float32 driftY = std::sin(st->idlePhase) * IDLE_Y_PX;
      const Float32 driftH = (0.5f + 0.5f * std::sin(st->idlePhase * 0.83f)) * IDLE_HEIGHT;

      // NOT gated on idleMotion: a sweep is what SCANNING IS, and a scanning
      // face that stops scanning is a different expression rather than a calmer
      // one.
      const Float32 sweepX = std::sin(st->scanPhase) * SCAN_REACH_PX * scan;

      const Float32 offX = (st->gazeX + driftX) * live * idleOn + sweepX;
      const Float32 offY = (st->gazeY + driftY) * live * idleOn;
      const Float32 hMul = lid * (1.0f - driftH * live * idleOn);

      // ---- compose ----------------------------------------------------------
      const Float32 tall = static_cast<Float32>(PANEL_H);
      const Float32 hL = clampF(st->shape.left.h * hMul, 0.0f, tall);
      const Float32 hR = clampF(st->shape.right.h * hMul * winkMul, 0.0f, tall);

      // SQUASH AND STRETCH, from the height's RATE rather than its value: an eye
      // collapsing fast spreads, and one snapping open pinches. The rate is
      // measured on the DRAWN height, so a blink and a wink squash the shape as
      // surely as a change of expression does.
      Float32 sqL = 1.0f;
      Float32 sqR = 1.0f;
      if(st->havePrev)
      {
          const Float32 rateL = (hL - st->prevLeftH) / dtS;
          const Float32 rateR = (hR - st->prevRightH) / dtS;
          sqL = clampF(1.0f - SQUASH_GAIN * rateL, 1.0f - SQUASH_MAX, 1.0f + SQUASH_MAX);
          sqR = clampF(1.0f - SQUASH_GAIN * rateR, 1.0f - SQUASH_MAX, 1.0f + SQUASH_MAX);
      }
      st->prevLeftH = hL;
      st->prevRightH = hR;
      st->havePrev = true;

      st->pose.left.h = hL;
      st->pose.right.h = hR;

      // The partner beat lands on the LEFT eye, because the RIGHT one is the one
      // that winks.
      const Float32 widen = 1.0f + WINK_PARTNER_WIDEN * partner;
      st->pose.left.w = st->shape.left.w * sqL * widen;
      st->pose.right.w = st->shape.right.w * sqR;

      st->pose.left.x = st->shape.left.x + offX;
      st->pose.left.y = st->shape.left.y + offY;
      st->pose.right.x = st->shape.right.x + offX;
      st->pose.right.y = st->shape.right.y + offY;

      st->pose.left.tilt = st->shape.left.tilt + WINK_PARTNER_TILT * partner;
      st->pose.right.tilt = st->shape.right.tilt;

      // LAST, and on the drawn pose only. The springs are left free to ring
      // wherever they like; what is guaranteed is that the rectangle handed to a
      // renderer is one the panel can hold. Clamping the SPRING instead would
      // eat its velocity at the edge and turn an overshoot into a thud.
      fitToPanel(&st->pose.left);
      fitToPanel(&st->pose.right);
  }

}
