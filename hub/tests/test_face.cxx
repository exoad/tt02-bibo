// The car's face, from hub/src/face.hxx.
//
//   tests\build_face_test.bat run
//
// The module is pure - a State, a Config and a clock in milliseconds - so all of
// this runs with no window, no device and no ImGui.
//
// Two rules matter more than the rest, and both are the kind that look right on
// a still frame and are wrong the moment anything moves:
//
//   * A blink MULTIPLIES the eye's height rather than replacing it. Written the
//     obvious way, a SLEEPY face snaps to full height every few seconds and a
//     wink un-winks itself mid-hold. testSleepyTrap and testWinkThroughBlink
//     are ten and thirty seconds of that, in a loop.
//   * The springs OVERSHOOT on purpose, so "does not exceed the target" is not
//     the assertion any more - "settles, in bounded time, without ever leaving
//     a legal rectangle" is. testSpringSettles and testHammered are that,
//     including the case of an expression changed on every single frame.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "../src/face.hxx"

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
          std::printf("  FAIL  %s\n", what);
      }
      else
      {
          std::printf("  ok    %s\n", what);
      }
  }

  Void checkNear(Float32 got, Float32 want, const Char* what)
  {
      ++checks;
      if(std::fabs(got - want) > 0.002f)
      {
          ++failures;
          std::printf(
              "  FAIL  %s: got %.4f, want %.4f\n",
              what,
              static_cast<Float64>(got),
              static_cast<Float64>(want)
          );
      }
      else
      {
          std::printf("  ok    %s = %.3f\n", what, static_cast<Float64>(got));
      }
  }

  // Half a panel pixel. checkNear's 0.002 is right for the normalized deltas
  // below and absurd for a number that counts pixels on a 240 x 320 screen.
  Void checkPx(Float32 got, Float32 want, const Char* what)
  {
      ++checks;
      if(std::fabs(got - want) > 0.5f)
      {
          ++failures;
          std::printf(
              "  FAIL  %s: got %.2f px, want %.2f px\n",
              what,
              static_cast<Float64>(got),
              static_cast<Float64>(want)
          );
      }
      else
      {
          std::printf("  ok    %s = %.1f px\n", what, static_cast<Float64>(got));
      }
  }

  constexpr Int32 EXPR_N = face::EXPRESSION_COUNT;
  constexpr Float32 PANEL_W = static_cast<Float32>(face::PANEL_W);
  constexpr Float32 PANEL_H = static_cast<Float32>(face::PANEL_H);

  [[nodiscard]] face::Expression at(Int32 i)
  {
      return static_cast<face::Expression>(i);
  }

  // One frame at 60 fps, which is what the hub actually hands step().
  constexpr Float32 FRAME_MS = 1000.0f / 60.0f;

  // NORMALIZED, even though the module is in pixels: a width in pixels of 240,
  // a height in pixels of 320 and a tilt in degrees are three different units,
  // and summing them raw would let two degrees of tilt count for less than one
  // pixel of width. Divided by their own ranges, "these two poses are distinct"
  // means something.
  [[nodiscard]] Float32 eyeDelta(const face::Eye& a, const face::Eye& b)
  {
      Float32 d = 0.0f;
      d += std::fabs(a.w - b.w) / PANEL_W;
      d += std::fabs(a.h - b.h) / PANEL_H;
      d += std::fabs(a.x - b.x) / PANEL_W;
      d += std::fabs(a.y - b.y) / PANEL_H;
      d += std::fabs(a.tilt - b.tilt) / 45.0f;
      return d;
  }

  [[nodiscard]] Float32 poseDelta(const face::Pose& a, const face::Pose& b)
  {
      return eyeDelta(a.left, b.left) + eyeDelta(a.right, b.right);
  }

  // Everything a 240 x 320 panel would refuse to draw, in one place. This is
  // the module's own containment claim, asked of it rather than reimplemented -
  // a second copy of the extents arithmetic in the test would agree with a
  // broken one.
  [[nodiscard]] Bool legal(const face::Pose& p)
  {
      return face::insidePanel(p.left)
          && face::insidePanel(p.right)
          && std::fabs(p.left.tilt) <= 45.0f
          && std::fabs(p.right.tilt) <= 45.0f;
  }

  // A config with nothing automatic in it, for the tests that are about one
  // mechanism. Every default is left where it is - only the generators of motion
  // that would otherwise be laid over the thing under test are off.
  [[nodiscard]] face::Config quiet()
  {
      face::Config cfg;
      cfg.autoBlink = false;
      cfg.idleMotion = false;
      return cfg;
  }

  // Run a state until its springs have stopped, for tests that want a settled
  // starting point rather than a moving one.
  Void settle(face::State* st, const face::Config& cfg, Int32 frames = 180)
  {
      for(Int32 i = 0; i < frames; ++i)
      {
          face::step(st, cfg, FRAME_MS);
      }
  }

  Void testNames()
  {
      std::printf("names\n");

      Bool allNamed = true;
      Bool allDistinct = true;
      for(Int32 i = 0; i < EXPR_N; ++i)
      {
          CharSeq a = face::name(at(i));
          if(a == nullptr || a[0] == '\0')
          {
              allNamed = false;
          }
          for(Int32 j = i + 1; j < EXPR_N; ++j)
          {
              if(Str(a) == Str(face::name(at(j))))
              {
                  allDistinct = false;
              }
          }
      }

      check(allNamed, "every expression has a name");
      check(allDistinct, "no two expressions share a name");

      // The enum and the count have to agree, and nothing else checks it: adding
      // a thirteenth face and forgetting the constant would silently hide it
      // from every grid and every loop in this file.
      check(
          Str(face::name(at(EXPR_N - 1))) == Str("Dead"),
          "EXPRESSION_COUNT reaches the last member"
      );
  }

  Void testDistinctPoses()
  {
      std::printf("twelve distinct faces\n");

      Int32 collisions = 0;
      Float32 closest = 1.0e9f;
      for(Int32 i = 0; i < EXPR_N; ++i)
      {
          for(Int32 j = i + 1; j < EXPR_N; ++j)
          {
              const Float32 d = poseDelta(face::posed(at(i)), face::posed(at(j)));
              if(d < 0.20f)
              {
                  ++collisions;
                  std::printf(
                      "        %s and %s differ by only %.4f\n",
                      face::name(at(i)),
                      face::name(at(j)),
                      static_cast<Float64>(d)
                  );
              }
              if(d < closest)
              {
                  closest = d;
              }
          }
      }

      check(collisions == 0, "no two resting poses are the same face");
      std::printf("        closest pair differs by %.3f\n", static_cast<Float64>(closest));

      // CONFUSED and THINKING carry their meaning in the asymmetry, so a
      // copy-paste that made both eyes identical would remove the expression and
      // leave the name behind. WINK is deliberately NOT in this list any more -
      // its asymmetry moved into the beat, and testWinkBeat is where it is now.
      const face::Pose confused = face::posed(face::Expression::EXPRESSION_CONFUSED);
      const face::Pose thinking = face::posed(face::Expression::EXPRESSION_THINKING);

      check(eyeDelta(confused.left, confused.right) > 0.4f, "CONFUSED is asymmetric");
      check(eyeDelta(thinking.left, thinking.right) > 0.05f, "THINKING is asymmetric");
      check(thinking.left.y < -0.3f, "THINKING looks up");
      check(std::fabs(thinking.left.x) > 0.3f, "...and to one side");

      // The sign that separates a squint from a droop, and the one thing in the
      // table that is easy to get backwards.
      const face::Pose happy = face::posed(face::Expression::EXPRESSION_HAPPY);
      const face::Pose sleepy = face::posed(face::Expression::EXPRESSION_SLEEPY);
      check(happy.left.y < 0.0f, "a happy squint leaves the slit HIGH");
      check(sleepy.left.y > 0.0f, "a sleepy droop leaves it LOW");

      // Tilt is the only cue anger and sadness have, so it has to point opposite
      // ways. Positive is inner-end-down.
      check(face::posed(face::Expression::EXPRESSION_ANGRY).left.tilt > 10.0f, "ANGRY tips in");
      check(face::posed(face::Expression::EXPRESSION_SAD).left.tilt < -10.0f, "SAD tips out");
  }

  Void testShapeRule()
  {
      std::printf("every resting pose is a rounded RECTANGLE\n");

      // Every RESTING pose has to fit the panel on its own, before any spring
      // has rung past it. The table is hand-written in pixels, so this is the
      // check that a tilt somebody nudged by four degrees did not quietly push a
      // corner off the glass - which is invisible until it is on the car.
      Bool ok = true;
      for(Int32 i = 0; i < EXPR_N; ++i)
      {
          if(!legal(face::posed(at(i))))
          {
              ok = false;
              std::printf("        %s does not fit 240 x 320\n", face::name(at(i)));
          }
      }
      check(ok, "every resting pose fits inside the panel");

      // THE SHAPE RULE, and it is here because it is the one thing that made an
      // earlier version wrong on sight: a rectangle whose sides are equal is a
      // square, and a square at a generous corner radius is a CIRCLE. The corner
      // is a renderer constant now, so this half of the defence is all this
      // module can hold - and it is the half that matters, because the corner
      // cannot make a tall rectangle round on its own.
      Bool square = false;
      for(Int32 i = 0; i < EXPR_N; ++i)
      {
          const face::Pose p = face::posed(at(i));
          const Array<face::Eye, 2> eyes = { p.left, p.right };
          for(const face::Eye& e : eyes)
          {
              if(e.h < 32.0f)
              {
                  continue;   // a shut eye is a bar; proportion says nothing
              }
              const Float32 ratio = std::fmax(e.w, e.h) / std::fmin(e.w, e.h);
              if(ratio < 1.20f)
              {
                  square = true;
                  std::printf(
                      "        %s is %.2f : 1 - too close to square\n",
                      face::name(at(i)),
                      static_cast<Float64>(ratio)
                  );
              }
          }
      }
      check(!square, "no open eye is within 20% of square");

      const face::Eye rest = face::posed(face::Expression::EXPRESSION_NEUTRAL).left;
      const Float32 ratio = rest.h / rest.w;
      check(rest.h > rest.w, "the resting eye stands up");
      check(ratio > 1.28f && ratio < 1.42f, "at about 1 : 1.33");
      checkPx(rest.w, 150.0f, "resting width");
      checkPx(rest.h, 200.0f, "resting height");

      // A real margin inside the panel on every side, at rest. An eye that
      // touches its own bezel has nowhere to go when a spring rings.
      check(rest.w < PANEL_W - 60.0f, "with room at the sides");
      check(rest.h < PANEL_H - 80.0f, "and room above and below");
  }

  Void testSprings()
  {
      std::printf("the springs are tuned per expression\n");

      // If they were all the same there would be no point exposing them, and the
      // face would move the same way whatever it was feeling.
      const face::Spring surprised = face::springOf(face::Expression::EXPRESSION_SURPRISED);
      const face::Spring sleepy = face::springOf(face::Expression::EXPRESSION_SLEEPY);
      const face::Spring focused = face::springOf(face::Expression::EXPRESSION_FOCUSED);
      const face::Spring dead = face::springOf(face::Expression::EXPRESSION_DEAD);

      check(surprised.freq > sleepy.freq * 3.0f, "SURPRISED is far quicker than SLEEPY");
      check(surprised.damp < 0.5f, "and rings while it gets there");
      check(focused.damp > 0.85f, "FOCUSED arrives and holds still");
      check(dead.damp >= 1.0f, "DEAD does not bounce at all");

      Bool sane = true;
      for(Int32 i = 0; i < EXPR_N; ++i)
      {
          const face::Spring sp = face::springOf(at(i));
          if(sp.freq < 4.0f || sp.freq > 40.0f || sp.damp < 0.30f || sp.damp > 1.20f)
          {
              sane = false;
              std::printf(
                  "        %s: %.1f rad/s, zeta %.2f\n",
                  face::name(at(i)),
                  static_cast<Float64>(sp.freq),
                  static_cast<Float64>(sp.damp)
              );
          }
      }
      check(sane, "every spring is inside a range that can settle");
  }

  Void testZeroDt()
  {
      std::printf("a dt of zero\n");

      face::State st;
      const face::Config cfg;
      face::set(&st, face::Expression::EXPRESSION_SURPRISED);

      const face::Pose before = st.pose;
      const UInt32 rngBefore = st.rng;

      for(Int32 i = 0; i < 100; ++i)
      {
          face::step(&st, cfg, 0.0f);
      }

      checkNear(poseDelta(st.pose, before), 0.0f, "the pose does not move");
      check(st.rng == rngBefore, "the generator is not advanced");
  }

  Void testSpeedZero()
  {
      std::printf("speed 0 freezes everything\n");

      face::State st;
      face::Config cfg;
      cfg.speed = 0.0f;
      face::set(&st, face::Expression::EXPRESSION_SCANNING);

      const face::Pose before = st.pose;
      const UInt32 rngBefore = st.rng;

      for(Int32 i = 0; i < 600; ++i)
      {
          face::step(&st, cfg, FRAME_MS);
      }

      checkNear(poseDelta(st.pose, before), 0.0f, "ten seconds of frames move nothing");
      check(st.rng == rngBefore, "and draw no randomness");
      check(!face::blinking(st), "and never blink");
  }

  Void testSpringSettles()
  {
      std::printf("a spring overshoots, and then it settles\n");

      const face::Config cfg = quiet();

      // Every expression, from neutral, has to be within a whisker of its target
      // inside a second and a quarter - including the slowest one in the table.
      // A spring has no arrival time of its own, so this is the substitute for
      // the "reaches the target within transitionMs" that the eased version had.
      Int32 slow = 0;
      Float32 worstAt = 0.0f;
      for(Int32 i = 0; i < EXPR_N; ++i)
      {
          face::State st;
          settle(&st, cfg);
          face::set(&st, at(i));

          const face::Pose want = face::posed(at(i));
          Float32 arrivedAt = -1.0f;
          for(Int32 f = 0; f < 120; ++f)   // 2 s
          {
              face::step(&st, cfg, FRAME_MS);
              if(arrivedAt < 0.0f && poseDelta(st.pose, want) < 0.01f)
              {
                  arrivedAt = static_cast<Float32>(f + 1) * FRAME_MS;
              }
          }
          // 1.5 s, which is DEAD and SLEEPY - the two deliberately heavy ones -
          // with room over. Everything else is inside 400 ms; this bound is here
          // to catch a spring that never settles at all, not to police taste.
          if(arrivedAt < 0.0f || arrivedAt > 1500.0f)
          {
              ++slow;
              std::printf("        %s had not settled in 1.5 s\n", face::name(at(i)));
          }
          if(arrivedAt > worstAt)
          {
              worstAt = arrivedAt;
          }
      }
      std::printf("        slowest to settle: %.0f ms\n", static_cast<Float64>(worstAt));
      check(slow == 0, "every expression settles inside 1.5 s");

      // ...and it OVERSHOOTS on the way, which is the whole reason the springs
      // replaced an eased transition. SURPRISED is the bounciest in the table.
      face::State st;
      settle(&st, cfg);
      face::set(&st, face::Expression::EXPRESSION_SURPRISED);
      const Float32 want = face::posed(face::Expression::EXPRESSION_SURPRISED).left.h;

      Float32 tallest = 0.0f;
      Float32 dipped = 1.0e9f;   // a min-sentinel: everything here is in PANEL PIXELS, so it is big
      for(Int32 f = 0; f < 40; ++f)
      {
          face::step(&st, cfg, FRAME_MS);
          tallest = std::fmax(tallest, st.pose.left.h);
          if(f < 8)
          {
              dipped = std::fmin(dipped, st.pose.left.h);
          }
      }
      check(tallest > want + 0.01f, "SURPRISED goes PAST its target and comes back");

      // ANTICIPATION: before flying open, the eye dips the other way first. The
      // dip is small and brief and nobody sees it AS a dip - what they see is
      // that the move afterwards has weight.
      check(
          dipped < face::posed(face::Expression::EXPRESSION_NEUTRAL).left.h,
          "and dips the other way before it goes"
      );
  }

  Void testSquashAndStretch()
  {
      std::printf("squash and stretch\n");

      face::State st;
      const face::Config cfg = quiet();
      settle(&st, cfg);

      const Float32 restW = st.pose.left.w;

      // A blink is the fastest height change there is, so it is where the width
      // has to react most. Collapsing spreads the eye.
      face::blink(&st);
      Float32 widest = 0.0f;
      for(Int32 f = 0; f < 12; ++f)
      {
          face::step(&st, cfg, 4.0f);
          widest = std::fmax(widest, st.pose.left.w);
      }
      check(widest > restW * 1.02f, "an eye collapsing fast gets wider");
      check(widest < restW * 1.20f, "...but not by much - this is weight, not a gag");

      settle(&st, cfg);
      checkPx(st.pose.left.w, restW, "and it comes back to its own width");
  }

  Void testBlinkShape()
  {
      std::printf("the blink itself\n");

      face::State st;
      const face::Config cfg = quiet();
      settle(&st, cfg);

      const Float32 rest = st.pose.left.h;
      check(!face::blinking(st), "not blinking to begin with");

      face::blink(&st);

      Float32 lowest = 1.0e9f;   // a min-sentinel: everything here is in PANEL PIXELS, so it is big
      Float32 elapsed = 0.0f;
      Bool everBlinking = false;

      // 2 ms steps, because a 130 ms blink sampled at 16 ms is eight samples and
      // the shut point can fall between two of them.
      while(elapsed < cfg.blinkMs)
      {
          face::step(&st, cfg, 2.0f);
          elapsed += 2.0f;
          if(face::blinking(st))
          {
              everBlinking = true;
          }
          lowest = std::fmin(lowest, st.pose.left.h);
      }

      check(everBlinking, "blinking() says so while it runs");
      check(lowest < 1.0f, "the eye actually closes");
      check(!face::blinking(st), "and it is over within blinkMs");
      checkPx(st.pose.left.h, rest, "height comes back exactly where it was");

      // Nothing but the height and its squash moves. A blink that also pulled the
      // eye about, or tipped it, would be a transition wearing a blink's name.
      const face::Pose neutral = face::posed(face::Expression::EXPRESSION_NEUTRAL);
      checkPx(st.pose.left.tilt, neutral.left.tilt, "the tilt is untouched");
      checkPx(st.pose.left.y, neutral.left.y, "and so is the position");

      // The asymmetry: 40 ms down and 90 ms up, so at the same fraction through
      // each half the closing side is much further along.
      face::State a;
      face::State b;
      settle(&a, cfg);
      settle(&b, cfg);
      face::blink(&a);
      face::blink(&b);
      face::step(&a, cfg, 20.0f);    // half way down
      face::step(&b, cfg, 85.0f);    // half way back up
      check(a.pose.left.h < b.pose.left.h, "the close is faster than the open");

      // The reopen goes a LITTLE past open before settling. That is
      // follow-through, and it is the reason testSleepyTrap allows a margin
      // rather than demanding the height never exceeds its resting value.
      face::State c;
      settle(&c, cfg);
      const Float32 cRest = c.pose.left.h;
      face::blink(&c);
      Float32 peak = 0.0f;
      for(Int32 f = 0; f < 70; ++f)
      {
          face::step(&c, cfg, 2.0f);
          peak = std::fmax(peak, c.pose.left.h);
      }
      check(peak > cRest * 1.005f, "and it opens a little past open");
      check(peak < cRest * 1.10f, "by a few percent, not a bounce");
  }

  Void testWinkBeat()
  {
      std::printf("the wink is a beat, not a pose\n");

      face::State st;
      const face::Config cfg = quiet();
      settle(&st, cfg);

      face::set(&st, face::Expression::EXPRESSION_WINK);
      check(face::winking(st), "selecting WINK starts the beat at once");

      Float32 lowestRight = 1.0e9f;   // a min-sentinel: everything here is in PANEL PIXELS, so it is big
      Float32 lowestLeft = 1.0e9f;   // a min-sentinel: everything here is in PANEL PIXELS, so it is big
      Float32 widestLeft = 0.0f;
      Float32 mostAsymmetric = 0.0f;
      Float32 elapsed = 0.0f;
      while(elapsed < 320.0f)
      {
          face::step(&st, cfg, 4.0f);
          elapsed += 4.0f;
          lowestRight = std::fmin(lowestRight, st.pose.right.h);
          lowestLeft = std::fmin(lowestLeft, st.pose.left.h);
          widestLeft = std::fmax(widestLeft, st.pose.left.w);
          mostAsymmetric = std::fmax(mostAsymmetric, eyeDelta(st.pose.left, st.pose.right));
      }

      std::printf("        left eye bottomed out at %.1f px\n", static_cast<Float64>(lowestLeft));
      checkPx(lowestRight, 0.0f, "the right eye shuts completely");
      check(lowestLeft > 40.0f, "and the left one never does");
      check(mostAsymmetric > 0.3f, "so the face is strongly asymmetric mid-beat");

      // The partner reaction. Without it a wink reads as one screen switching off
      // rather than as something the face did on purpose.
      const Float32 restW = face::posed(face::Expression::EXPRESSION_WINK).left.w;
      check(widestLeft > restW * 1.03f, "the other eye widens in sympathy");

      // ...and it REOPENS. A wink that leaves the eye shut is a broken panel.
      settle(&st, cfg, 60);
      check(st.pose.right.h > 90.0f, "and the eye comes back open afterwards");

      // The resting pose is symmetric ON PURPOSE - everything asymmetric about
      // WINK is in the beat above.
      const face::Pose rest = face::posed(face::Expression::EXPRESSION_WINK);
      checkNear(eyeDelta(rest.left, rest.right), 0.0f, "WINK's resting pose is symmetric");
  }

  Void testWinkThroughBlink()
  {
      std::printf("a wink and a blink compose rather than fight\n");

      face::State st;
      face::Config cfg = quiet();
      settle(&st, cfg);

      face::wink(&st);

      // Into the HOLD - 70 ms of close plus a little.
      for(Int32 f = 0; f < 25; ++f)
      {
          face::step(&st, cfg, 4.0f);
      }
      check(face::winking(st), "still mid-wink");
      checkPx(st.pose.right.h, 0.0f, "and the winking eye is shut");

      // Now blink INTO the hold. The blink multiplies, so it changes nothing on
      // an eye already at zero - and it must still close the other one.
      //
      // The window is 48 ms, which keeps the wink inside its 90 ms hold: past
      // that the wink's own reopen legitimately raises the eye, and a test that
      // ran through it would be failing the wink for working.
      face::blink(&st);
      Float32 worstRight = 0.0f;
      Float32 lowestLeft = 1.0e9f;   // a min-sentinel: everything here is in PANEL PIXELS, so it is big
      for(Int32 f = 0; f < 12; ++f)
      {
          face::step(&st, cfg, 4.0f);
          worstRight = std::fmax(worstRight, st.pose.right.h);
          lowestLeft = std::fmin(lowestLeft, st.pose.left.h);
      }
      check(face::winking(st), "and the wink is still in its hold");
      checkPx(worstRight, 0.0f, "the blink never reopens the winking eye");
      check(lowestLeft < 1.0f, "and it does close the other one");
  }

  Void testSleepyTrap()
  {
      std::printf("the trap: a blink multiplies, it does not replace\n");

      face::State st;
      face::Config cfg;
      cfg.idleMotion = false;   // so the only things touching the height are the springs and blinks

      face::set(&st, face::Expression::EXPRESSION_SLEEPY);
      const Float32 want = face::posed(face::Expression::EXPRESSION_SLEEPY).left.h;

      // Past the arrival before measuring: on the way down from NEUTRAL's 0.70
      // the eye is legitimately taller than sleepy, and counting that as an
      // escape would make this fail for the one reason it is not about.
      settle(&st, cfg);

      // The reopen overshoots by a few percent by design, so the bound is the
      // resting height plus that - NOT the resting height exactly. The version
      // this is guarding against does not miss by a few percent; it hands back
      // the FULL height, which on a 38 px sleepy slit is eight times too much.
      const Float32 bound = want * 1.15f;

      Float32 highest = 0.0f;
      Int32 blinks = 0;
      Bool wasBlinking = false;
      for(Int32 i = 0; i < 600; ++i)   // 10 s
      {
          face::step(&st, cfg, FRAME_MS);
          if(face::blinking(st) && !wasBlinking)
          {
              ++blinks;
          }
          wasBlinking = face::blinking(st);
          highest = std::fmax(highest, st.pose.left.h);
      }

      std::printf(
          "        %d blink(s) in ten seconds, tallest %.1f px (sleepy is %.1f px)\n",
          blinks,
          static_cast<Float64>(highest),
          static_cast<Float64>(want)
      );
      check(blinks >= 1, "it blinked at least once");
      check(highest <= bound, "a sleepy eye never opens past sleepy");
      checkPx(st.pose.left.h, want, "and is still sleepy at the end");

      // ...and ten thousand frames of it, which is the run the header promises.
      for(Int32 i = 0; i < 10000; ++i)
      {
          face::step(&st, cfg, FRAME_MS);
          if(st.pose.left.h > bound)
          {
              check(false, "the height escaped during the long run");
              return;
          }
      }
      check(true, "ten thousand frames and it never escapes");
  }

  Void testScanningComposes()
  {
      std::printf("scanning keeps sweeping through a blink\n");

      face::State st;
      face::Config cfg;
      cfg.blinkEveryMinS = 0.30f;
      cfg.blinkEveryMaxS = 0.45f;   // blink constantly, so the overlap is certain

      face::set(&st, face::Expression::EXPRESSION_SCANNING);
      settle(&st, cfg, 60);

      Float32 minX = 1.0e9f;   // a min-sentinel: everything here is in PANEL PIXELS, so it is big
      Float32 maxX = -1.0e9f;
      Float32 movedWhileBlinking = 0.0f;
      Float32 lastX = st.pose.left.x;
      for(Int32 i = 0; i < 600; ++i)
      {
          face::step(&st, cfg, FRAME_MS);
          const Float32 g = st.pose.left.x;
          if(face::blinking(st))
          {
              movedWhileBlinking += std::fabs(g - lastX);
          }
          lastX = g;
          minX = std::fmin(minX, g);
          maxX = std::fmax(maxX, g);
      }

      check(maxX - minX > 80.0f, "the eyes sweep a wide arc");
      check(movedWhileBlinking > 8.0f, "and keep moving while they are shut");
  }

  Void testAutoBlinkWindow()
  {
      std::printf("auto-blink fires inside its window and no faster\n");

      face::State st;
      const face::Config cfg;

      // The interval is redrawn when a blink ENDS, so the gap being measured is
      // end-to-start. Start-to-start is that plus one blink, and the upper bound
      // below allows for it.
      const Float32 slack = FRAME_MS * 0.001f + cfg.blinkMs * 0.001f;

      Float32 clock = 0.0f;
      Float32 lastStart = -1.0f;
      Int32 starts = 0;
      Int32 tooSoon = 0;
      Int32 tooLate = 0;
      Bool wasBlinking = false;

      for(Int32 i = 0; i < 60 * 120; ++i)   // two minutes
      {
          face::step(&st, cfg, FRAME_MS);
          clock += FRAME_MS * 0.001f;
          const Bool now = face::blinking(st);
          if(now && !wasBlinking)
          {
              ++starts;
              if(lastStart >= 0.0f)
              {
                  const Float32 gap = clock - lastStart;
                  if(gap < cfg.blinkEveryMinS)
                  {
                      ++tooSoon;
                  }
                  if(gap > cfg.blinkEveryMaxS + slack)
                  {
                      ++tooLate;
                  }
              }
              lastStart = clock;
          }
          wasBlinking = now;
      }

      std::printf("        %d blink(s) in two minutes\n", starts);
      check(starts >= 15, "it blinks often enough to look alive");
      check(tooSoon == 0, "never sooner than blinkEveryMinS");
      check(tooLate == 0, "never later than blinkEveryMaxS");

      // Off means off. Not "rarely".
      face::State quietSt;
      face::Config off;
      off.autoBlink = false;
      Bool everBlinked = false;
      for(Int32 i = 0; i < 60 * 120; ++i)
      {
          face::step(&quietSt, off, FRAME_MS);
          if(face::blinking(quietSt))
          {
              everBlinked = true;
          }
      }
      check(!everBlinked, "autoBlink off means no blink in two minutes");
  }

  Void testHammered()
  {
      std::printf("hammered: a new expression on every single frame\n");

      // The nastiest thing a spring can be asked to do. Nothing here may leave a
      // legal rectangle, and once the hammering stops it has to settle like any
      // other move - a spring that has been driven for a minute and cannot come
      // back has been accumulating energy it should not have.
      face::State st;
      const face::Config cfg;

      Bool everIllegal = false;
      for(Int32 i = 0; i < 60 * 60; ++i)   // one minute, one set() per frame
      {
          face::set(&st, at(i % EXPR_N));
          face::step(&st, cfg, FRAME_MS);
          if(!legal(st.pose))
          {
              everIllegal = true;
          }
      }
      check(!everIllegal, "never puts a corner off the 240 x 320 panel");

      face::set(&st, face::Expression::EXPRESSION_NEUTRAL);
      settle(&st, quiet());
      checkNear(
          poseDelta(st.pose, face::posed(face::Expression::EXPRESSION_NEUTRAL)),
          0.0f,
          "and settles cleanly once it is left alone"
      );

      // The other way a spring detonates: one enormous frame. step() sub-steps,
      // so a stall must be survivable rather than fatal.
      face::State stalled;
      face::set(&stalled, face::Expression::EXPRESSION_SURPRISED);
      face::step(&stalled, cfg, 2000.0f);
      check(legal(stalled.pose), "a two-second frame does not blow it up");
      settle(&stalled, quiet());
      check(legal(stalled.pose), "and it is still sane afterwards");
  }

  Void testEverythingStaysInRange()
  {
      std::printf("nothing ever leaves its range, and nothing teleports\n");

      face::State st;
      const face::Config cfg;

      Bool everIllegal = false;
      Float32 worstX = 0.0f;
      Float32 worstY = 0.0f;
      Float32 biggestStep = 0.0f;
      Float32 last = st.pose.left.h;

      for(Int32 i = 0; i < 60 * 60; ++i)   // 60 s at 60 fps
      {
          // Change expression on a cadence that is NOT a multiple of the blink
          // window, so transitions and blinks land on top of each other.
          if((i % 47) == 0)
          {
              face::set(&st, at((i / 47) % EXPR_N));
          }
          face::step(&st, cfg, FRAME_MS);

          if(!legal(st.pose))
          {
              everIllegal = true;
          }

          worstX = std::fmax(worstX, std::fabs(st.pose.left.x));
          worstX = std::fmax(worstX, std::fabs(st.pose.right.x));
          worstY = std::fmax(worstY, std::fabs(st.pose.left.y));
          worstY = std::fmax(worstY, std::fabs(st.pose.right.y));

          const Float32 v = st.pose.left.h;
          biggestStep = std::fmax(biggestStep, std::fabs(v - last));
          last = v;
      }

      std::printf(
          "        furthest an eye's center ever got: %.1f, %.1f px\n",
          static_cast<Float64>(worstX),
          static_cast<Float64>(worstY)
      );
      check(!everIllegal, "every frame fits inside 240 x 320");
      check(worstX < PANEL_W * 0.5f, "and no center leaves the panel sideways");
      check(worstY < PANEL_H * 0.5f, "or up and down");

      // The blink close is the fastest thing here and its bound is arithmetic:
      // 40 ms, of which one 16.7 ms frame is 42%, and smoothstep's steepest slope
      // is 1.5 - so an eye at the panel's full 320 px can lose 0.63 of itself,
      // about 200 px, in one frame and no more. This is a "nothing teleports"
      // bound rather than the blink trap; testSleepyTrap is the one that catches
      // a blink written the wrong way.
      std::printf(
          "        biggest single-frame move: %.1f px\n",
          static_cast<Float64>(biggestStep)
      );
      check(biggestStep < 240.0f, "no frame moves the height more than a blink can");
  }

  Void testDeterminism()
  {
      std::printf("the seeded generator makes a run reproducible\n");

      const face::Config cfg;

      face::State a;
      face::State b;
      for(Int32 i = 0; i < 5000; ++i)
      {
          face::step(&a, cfg, FRAME_MS);
          face::step(&b, cfg, FRAME_MS);
      }
      checkNear(poseDelta(a.pose, b.pose), 0.0f, "same seed, same five thousand frames");
      check(a.rng == b.rng, "and the generators are in step");

      // A different seed has to give a different run, or the seed is decoration.
      face::State c;
      c.rng = 0xC0FFEE11u;
      for(Int32 i = 0; i < 5000; ++i)
      {
          face::step(&c, cfg, FRAME_MS);
      }
      check(c.rng != a.rng, "a different seed is a different run");

      // Replaying from a copied State reproduces exactly, which is what makes a
      // bad frame in the viewer something anybody can get back. The springs make
      // this stricter than it was: the velocities are state too, and a copy that
      // dropped them would drift within a dozen frames.
      face::State snap = a;
      for(Int32 i = 0; i < 500; ++i)
      {
          face::step(&a, cfg, FRAME_MS);
          face::step(&snap, cfg, FRAME_MS);
      }
      checkNear(poseDelta(a.pose, snap.pose), 0.0f, "a copied State replays the same");
  }

  Void testIdleMotion()
  {
      std::printf("idle motion, and turning it off\n");

      face::State moving;
      face::Config on;
      on.autoBlink = false;

      Float32 lo = 1.0e9f;   // a min-sentinel: everything here is in PANEL PIXELS, so it is big
      Float32 hi = -1.0e9f;
      for(Int32 i = 0; i < 600; ++i)
      {
          face::step(&moving, on, FRAME_MS);
          lo = std::fmin(lo, moving.pose.left.y);
          hi = std::fmax(hi, moving.pose.left.y);
      }
      check(hi - lo > 0.02f, "a still face still drifts");

      face::State frozen;
      face::Config off = on;
      off.idleMotion = false;
      const face::Pose want = face::posed(face::Expression::EXPRESSION_NEUTRAL);
      settle(&frozen, off);

      Bool everMoved = false;
      for(Int32 i = 0; i < 600; ++i)
      {
          face::step(&frozen, off, FRAME_MS);
          if(poseDelta(frozen.pose, want) > 0.0005f)
          {
              everMoved = true;
          }
      }
      check(!everMoved, "idleMotion off holds the whole face exactly still");
  }

  Void testDeadIsDead()
  {
      std::printf("dead is dead\n");

      face::State st;
      const face::Config cfg;
      face::set(&st, face::Expression::EXPRESSION_DEAD);
      settle(&st, cfg);

      const face::Pose rested = st.pose;
      Float32 wander = 0.0f;
      for(Int32 i = 0; i < 1200; ++i)
      {
          face::step(&st, cfg, FRAME_MS);
          wander = std::fmax(wander, std::fabs(st.pose.left.x - rested.left.x));
          wander = std::fmax(wander, std::fabs(st.pose.left.y - rested.left.y));
      }
      checkPx(wander, 0.0f, "a dead face does not wander its eyes");
      check(rested.left.h < 32.0f, "and it is a flat bar");
  }

}

Int32 main()
{
    std::printf("face tests\n\n");

    testNames();
    testDistinctPoses();
    testShapeRule();
    testSprings();
    testZeroDt();
    testSpeedZero();
    testSpringSettles();
    testSquashAndStretch();
    testBlinkShape();
    testWinkBeat();
    testWinkThroughBlink();
    testSleepyTrap();
    testScanningComposes();
    testAutoBlinkWindow();
    testHammered();
    testEverythingStaysInRange();
    testDeterminism();
    testIdleMotion();
    testDeadIsDead();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
