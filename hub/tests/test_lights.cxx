// The vehicle lighting rules, from docs/conventions.md "Lighting behavior".
//
//   tests\build_lights_test.bat run
//
// These exist because three of the rules look right and are not, and all three
// are the kind that a person glancing at a blinking LED would accept:
//
//   * The indicator OVERRIDES the brake on its own side. Braking while
//     signalling right means the right rear alternates and the left stays
//     solid. Get it wrong and both stay solid, which looks fine and is wrong.
//   * Hazards are both sides IN PHASE. Alternating is what a film prop does.
//   * Front and rear on one side share ONE clock. Per-lamp timers drift, and
//     drift is invisible for the first minute.
//
// No hardware and no ImGui. Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "../src/lights.hxx"

#include <cmath>
#include <cstdio>

namespace
{

  Int32 failures = 0;
  Int32 checks   = 0;

  Void check(Bool ok, const Char* what)
  {
      ++checks;
      if(!ok)
      {
          ++failures;
          std::printf("  FAIL  %s\n", what);;
      }
      else
      {
          std::printf("  ok    %s\n", what);;
      }
  }

  Void checkNear(Float32 got, Float32 want, const Char* what)
  {
      ++checks;
      if(std::fabs(got - want) > 0.001f)
      {
          ++failures;
          std::printf("  FAIL  %s: got %.3f, want %.3f\n",
                      what, static_cast<Float64>(got), static_cast<Float64>(want));
      }
      else
      {
          std::printf("  ok    %s = %.2f\n", what, static_cast<Float64>(got));
      }
  }

  // A time inside the ON phase, and one inside the OFF phase, of the same cycle.
  constexpr Float64 T_ON  = 0.100;
  constexpr Float64 T_OFF = 0.500;

  Void testTiming()
  {
      std::printf("flasher timing\n");

      checkNear(static_cast<Float32>(lights::BLINK_PERIOD_S), 0.600f, "period (s)");

      // 100 flashes per minute. This asserted 0.667 s / 1.5 Hz until
      // 2026-08-30, which was the value BEFORE the rate was corrected: 667 ms
      // is 89.96 fpm and the normally-closed band in SAE J945 has a floor of
      // 90. The source moved and the test did not, so it failed for weeks
      // saying the right answer was wrong.
      //
      // Checked in flashes per minute rather than Hz because that is the unit
      // the standard is written in, and 89.96 vs 90 is a distinction Hz hides.
      const Float32 fpm = static_cast<Float32>(60.0 / lights::BLINK_PERIOD_S);
      checkNear(fpm, 100.0f, "100 flashes per minute");
      check(fpm >= 90.0f && fpm <= 120.0f, "inside the SAE J945 band");

      check(lights::blinkPhase(T_ON),  "on inside the on phase");
      check(!lights::blinkPhase(T_OFF), "off inside the off phase");

      // And it repeats, rather than running once and latching.
      check(lights::blinkPhase(T_ON + lights::BLINK_PERIOD_S * 5.0),
            "still on one whole cycle later");
      check(!lights::blinkPhase(T_OFF + lights::BLINK_PERIOD_S * 5.0),
            "still off one whole cycle later");
  }

  Void testOneClock()
  {
      std::printf("one clock for the whole vehicle\n");

      lights::Input in;
      in.turn = lights::Turn::TURN_LEFT;

      // Sampled across a whole cycle: front and rear on a side must never
      // disagree, at any instant.
      Bool everDiffered = false;
      for(Int32 i = 0; i < 200; ++i)
      {
          const Float64 t = static_cast<Float64>(i) * (lights::BLINK_PERIOD_S / 37.0);
          const lights::Lamps l = lights::solve(in, t);
          if(l.indFL != l.indRL)
          {
              everDiffered = true;
          }
      }
      check(!everDiffered, "front and rear left never disagree");
  }

  Void testHazard()
  {
      std::printf("hazards\n");

      lights::Input in;
      in.turn = lights::Turn::TURN_HAZARD;

      Bool everDiffered = false;
      Bool sawOn = false, sawOff = false;
      for(Int32 i = 0; i < 200; ++i)
      {
          const Float64 t = static_cast<Float64>(i) * (lights::BLINK_PERIOD_S / 37.0);
          const lights::Lamps l = lights::solve(in, t);
          if(l.indFL != l.indFR || l.indRL != l.indRR)
          {
              everDiffered = true;
          }
          if(l.indFL > 0.5f)
          {
              sawOn = true;
          }
          else
          {
              sawOff = true;
          }
      }

      check(!everDiffered, "both sides in phase, never alternating");
      check(sawOn && sawOff, "and they do actually blink");
  }

  Void testTailAndBrake()
  {
      std::printf("tail and brake share one red lamp\n");

      lights::Input in;
      checkNear(lights::solve(in, T_OFF).tailL, 0.0f, "dark with everything off");

      in.head = lights::Head::HEAD_ON;
      checkNear(lights::solve(in, T_OFF).tailL, 0.30f, "tail at 30% with lights on");

      in.brake = true;
      checkNear(lights::solve(in, T_OFF).tailL, 1.00f, "brake at 100%");

      // Braking with the lights OFF still lights the brake: a brake lamp does not
      // depend on the headlight switch.
      in.head = lights::Head::HEAD_OFF;
      checkNear(lights::solve(in, T_OFF).tailL, 1.00f, "brake works with lights off");

      lights::Input drl;
      drl.head = lights::Head::HEAD_DRL;
      checkNear(lights::solve(drl, T_OFF).headL, 0.45f, "DRL is the headlamp dimmed");
      drl.head = lights::Head::HEAD_ON;
      checkNear(lights::solve(drl, T_OFF).headL, 1.00f, "headlight full");
  }

  // The rule this used to test was REMOVED, and the test is inverted rather than
  // deleted. A rule that was once there and is deliberately gone needs a test
  // saying so, or the next person who knows how cars work puts it back.
  Void testNoOverride()
  {
      std::printf("the indicator does NOT interrupt the brake\n");

      lights::Input in;
      in.brake = true;
      in.turn  = lights::Turn::TURN_RIGHT;

      const lights::Lamps on  = lights::solve(in, T_ON);
      const lights::Lamps off = lights::solve(in, T_OFF);

      // On a car whose rear indicator and brake share ONE bulb, the indicator has
      // to interrupt the brake to be seen at all. This car has separate LEDs for
      // each, so interrupting is pure loss: it makes a brake light blink, which
      // is the one thing a brake light must never do.
      checkNear(on.tailR,  1.00f, "right rear stays solid while its indicator is lit");
      checkNear(off.tailR, 1.00f, "and between flashes");
      checkNear(on.tailL,  1.00f, "left rear solid too");
      checkNear(off.tailL, 1.00f, "left rear solid between flashes");

      // The indicator still blinks. It is simply beside the brake rather than
      // instead of it.
      checkNear(on.indRR,  1.0f, "the right indicator blinks independently");
      checkNear(off.indRR, 0.0f, "dark between flashes");
      checkNear(on.indRL,  0.0f, "and the other side is not indicating");

      // Hazards while braking: all four reds stay lit, both indicators blink.
      lights::Input haz;
      haz.brake = true;
      haz.turn  = lights::Turn::TURN_HAZARD;
      const lights::Lamps h = lights::solve(haz, T_ON);
      check(h.tailL == 1.00f && h.tailR == 1.00f,
            "hazards while braking leave both brake lamps solid");
      check(h.indRL == 1.0f && h.indRR == 1.0f,
            "and blink both indicators together");
  }

  Void testReverse()
  {
      std::printf("reverse\n");

      lights::Input in;
      checkNear(lights::solve(in, T_ON).revL, 0.0f, "off by default");

      in.reverse = true;
      in.turn    = lights::Turn::TURN_HAZARD;
      in.brake   = true;

      // Nothing interrupts reverse: it reports the gearbox, which no other signal
      // contradicts.
      checkNear(lights::solve(in, T_ON).revL,  1.0f, "lit through a flash");
      checkNear(lights::solve(in, T_OFF).revR, 1.0f, "lit between flashes");
  }


  // ===========================================================================
  // detect(): steering and throttle in, an Input out.
  //
  // Every one of these is a rule that looks obviously right written down and is
  // obviously wrong the first time it is on a car.
  // ===========================================================================

  // A tidy way to push one sample through and see what came out.
  lights::Input step(lights::AutoState& st, Float32 steer, Int32 esc,
                     Float64 t, Bool armed = true)
  {
      lights::Drive d;
      d.steer      = steer;
      d.throttleUs = esc;
      d.armed      = armed;
      return lights::detect(st, d, t);
  }

  Void testTurnThreshold()
  {
      std::printf("\n-- detect: indicator thresholds --\n");

      lights::AutoState st;

      check(step(st, 0.0f, 1500, 0.0).turn == lights::Turn::TURN_OFF,
            "straight ahead does not indicate");
      check(step(st, 0.30f, 1500, 0.1).turn == lights::Turn::TURN_OFF,
            "a gentle correction does not indicate");

      check(step(st, 0.60f, 1500, 0.2).turn == lights::Turn::TURN_RIGHT,
            "a deliberate right turn indicates right");

      lights::AutoState st2;
      check(step(st2, -0.60f, 1500, 0.2).turn == lights::Turn::TURN_LEFT,
            "and left is left");
  }

  Void testTurnHysteresis()
  {
      std::printf("\n-- detect: indicator hysteresis --\n");

      lights::AutoState st;

      // On, then unwind the steering slowly. It must NOT drop out the moment the
      // wheel passes back through the trigger point - that is the chatter the
      // second threshold exists to prevent.
      check(step(st, 0.60f, 1500, 0.0).turn == lights::Turn::TURN_RIGHT, "on at 0.60");

      const Float64 late = 1.0;   // past the minimum flash, so only the band decides
      check(step(st, 0.40f, 1500, late).turn == lights::Turn::TURN_RIGHT,
            "still on at 0.40 - below the ON threshold, above the OFF one");
      check(step(st, 0.30f, 1500, late).turn == lights::Turn::TURN_RIGHT,
            "still on at 0.30");
      check(step(st, 0.20f, 1500, late).turn == lights::Turn::TURN_OFF,
            "off at 0.20, through the lower threshold");
  }

  Void testTurnMinimumFlash()
  {
      std::printf("\n-- detect: one complete flash --\n");

      lights::AutoState st;

      // Flick the wheel and centre it again immediately. The flasher clock is
      // free-running, so without a hold this shows a fragment of a cycle or
      // nothing at all, depending on when it happened to land.
      check(step(st, 0.60f, 1500, 0.0).turn == lights::Turn::TURN_RIGHT, "flick triggers");
      check(step(st, 0.0f, 1500, 0.01).turn == lights::Turn::TURN_RIGHT,
            "and holds though the wheel is already back");
      check(step(st, 0.0f, 1500, lights::BLINK_PERIOD_S * 0.5).turn == lights::Turn::TURN_RIGHT,
            "still holding halfway through the period");

      check(step(st, 0.0f, 1500, lights::BLINK_PERIOD_S + 0.01).turn == lights::Turn::TURN_OFF,
            "and releases after one full period");
  }

  Void testTurnDirectionChange()
  {
      std::printf("\n-- detect: a change of side is immediate --\n");

      lights::AutoState st;

      check(step(st, 0.60f, 1500, 0.0).turn == lights::Turn::TURN_RIGHT, "right first");

      // Straight through the hold. Indicating LEFT while the wheels go right is
      // the one failure an indicator must not have, so the hold does not apply.
      check(step(st, -0.60f, 1500, 0.05).turn == lights::Turn::TURN_LEFT,
            "swings to left at once, inside the minimum flash");
  }

  Void testBrakeOnLiftOff()
  {
      std::printf("\n-- detect: brake --\n");

      lights::AutoState st;

      // The first sample is a baseline. Connecting to a board that is already
      // holding throttle must not read as a drop from nothing.
      check(!step(st, 0.0f, 1580, 0.0).brake, "the first sample never brakes");
      check(!step(st, 0.0f, 1580, 0.1).brake, "holding a steady throttle does not brake");
      check(!step(st, 0.0f, 1590, 0.2).brake, "accelerating does not brake");

      check(step(st, 0.0f, 1560, 0.3).brake, "a drop lights the brake");
      check(step(st, 0.0f, 1560, 0.4).brake, "and it stays lit while it is held");

      check(!step(st, 0.0f, 1560, 0.3 + 0.36).brake,
            "and goes out after the hold, once nothing more has dropped");
  }

  Void testBrakeSurvivesOneFrame()
  {
      std::printf("\n-- detect: the brake outlives its trigger --\n");

      lights::AutoState st;
      step(st, 0.0f, 1600, 0.0);

      // One single-step drop, of the size the slew limiter actually produces,
      // then perfectly steady. The lamp has to stay on: a brake light lit for one
      // frame is a brake light nobody sees.
      check(step(st, 0.0f, 1592, 0.1).brake, "one slew step is enough to trigger");
      check(step(st, 0.0f, 1592, 0.2).brake, "still on 100 ms later with no further drop");
      check(step(st, 0.0f, 1592, 0.3).brake, "and 200 ms later");
      check(!step(st, 0.0f, 1592, 0.5).brake, "out once the hold expires");
  }

  Void testAutoTail()
  {
      std::printf("\n-- detect: tails and reverse --\n");

      lights::AutoState st;

      check(step(st, 0.0f, 1500, 0.0, false).head == lights::Head::HEAD_OFF,
            "disarmed: no running lights");
      check(step(st, 0.0f, 1500, 0.1, true).head == lights::Head::HEAD_DRL,
            "armed: running lights, so the brake has something to be brighter than");

      // Forward-only car, so this is never true - see chassis.h.
      check(!step(st, 0.0f, 1500, 0.2, true).reverse, "reverse is never claimed");
  }

  Void testDetectFeedsSolve()
  {
      std::printf("\n-- detect -> solve --\n");

      // The point of the whole exercise: what detect() produces must be something
      // solve() renders sensibly, including the override that makes an indicating
      // side interrupt its own brake lamp.
      lights::AutoState st;
      step(st, 0.0f, 1600, 0.0);

      const lights::Input in = step(st, 0.60f, 1560, 0.1);
      check(in.turn == lights::Turn::TURN_RIGHT && in.brake,
            "braking into a right-hander is both at once");

      const lights::Lamps on = lights::solve(in, T_ON);
      checkNear(on.tailL, lights::BRAKE_LEVEL, "both brake lamps are hard on");
      checkNear(on.tailR, lights::BRAKE_LEVEL, "including the indicating side");
      checkNear(on.indRR, 1.0f, "and the indicator blinks beside it");
  }

} // namespace

Int32 main()
{
    std::printf("vehicle lighting tests\n\n");

    testTiming();
    testOneClock();
    testHazard();
    testTailAndBrake();
    testNoOverride();
    testReverse();

    testTurnThreshold();
    testTurnHysteresis();
    testTurnMinimumFlash();
    testTurnDirectionChange();
    testBrakeOnLiftOff();
    testBrakeSurvivesOneFrame();
    testAutoTail();
    testDetectFeedsSolve();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
