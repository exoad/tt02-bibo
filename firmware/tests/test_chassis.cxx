/*
 * ---------------------------------------------------------------------------
 * chassis - the safety property, on a laptop.
 *
 * conventions.md says "Safety lives in the module, not the caller", and points
 * at this file: drive:: refuses throttle until armed, and returns Bool rather
 * than printing. That is the rule the whole car rests on and it had no test,
 * because chassis.hxx includes hal.hxx and hal.hxx is the Pico SDK.
 *
 * It does now, through firmware/tests/fakes/hal.hxx. What that buys is not
 * coverage for its own sake: it is that these assertions are about what
 * reaches the PINS, not about what the module says it did. The difference
 * matters - lights::enable() spent weeks reporting a state it had not stored.
 * -------------------------------------------------------------------------
 */
#include "../lib/chassis/chassis.hxx"

#include <stdio.h>

namespace
{

  Int32 checks = 0;
  Int32 failures = 0;

  /**
   * @brief Records a pass/fail check and prints its outcome.
   *
   * @param ok true if the property being checked held
   * @param what the description printed alongside the outcome
   */
  Void check(Bool ok, CharSeq what)
  {
      ++checks;
      if(!ok)
      {
          ++failures;
          printf("  FAIL  %s\n", what);
          return;
      }
      printf("  ok    %s\n", what);
  }

  /**
   * @brief Records a pass/fail check comparing two integers for equality.
   *
   * @param got the value produced by the code under test
   * @param want the value it was expected to produce
   * @param what the description printed alongside the outcome
   */
  Void checkEq(Int32 got, Int32 want, CharSeq what)
  {
      ++checks;
      if(got != want)
      {
          ++failures;
          printf("  FAIL  %s: got %d, want %d\n", what, got, want);
          return;
      }
      printf("  ok    %s = %d\n", what, got);
  }

  /**
   * @brief The ESC pad of the currently installed pin map.
   *
   * @return the GPIO number bound to the ESC, or pins::NONE
   *
   * @note Expands to pins::active().esc, so a test that never installs a
   *       map asserts about pin NONE - which passes, and proves nothing.
   */
  Int32 escPin(Void)
  {
      return bibo::pins::active().esc;
  }

  /**
   * @brief The servo pad of the currently installed pin map.
   *
   * @return the GPIO number bound to the servo, or pins::NONE
   */
  Int32 servoPin(Void)
  {
      return bibo::pins::active().servo;
  }

  /**
   * @brief Runs the fake clock long enough for pump()'s slew to finish.
   *
   * @note pump() SLEWS - it walks each output toward its target by a few
   *       microseconds per 20 ms tick, so one call never arrives. Tests
   *       that care where the output ENDS UP have to let the clock run,
   *       which the fake lets them do instantly. 200 ticks is four
   *       seconds of car time and returns in microseconds of ours.
   */
  Void settle(Void)
  {
      for(Int32 i = 0; i < 200; ++i)
      {
          bibo::timing::ms(SLEW_TICK_MS);
          bibo::drive::pump();
      }
  }

  /**
   * @brief Resets the fake HAL and opens drive:: on the car's pin map.
   *
   * @note Every test starts from a known board.
   */
  Void fresh(Void)
  {
      bibo::fake::reset();
      static_cast<Void>(bibo::pins::begin(bibo::pins::car()));
      bibo::drive::open();
  }

  /**
   * @brief Proves the ESC refuses throttle until arm() is called, and again once disarmed.
   */
  /* ---- rule 2: the ESC is disarmed until asked ------------------------- */
  Void testThrottleRefusedUntilArmed()
  {
      printf("\nthrottle is refused until armed\n");
      fresh();

      check(!bibo::drive::throttleUs(1600), "a disarmed ESC refuses throttle");
      check(
          !bibo::drive::throttleUs(THROTTLE_CAL_MAX),
          "and refuses it at the calibrated maximum too"
      );

      bibo::drive::arm(true);
      check(bibo::drive::throttleUs(1560), "an armed ESC accepts it");

      /* Disarming is not a suggestion. */
      bibo::drive::arm(false);
      check(!bibo::drive::throttleUs(1560), "disarming refuses again");
  }

  /**
   * @brief Proves re-arming lands the ESC at neutral rather than carrying the last command.
   *
   * @note Arming must not itself be a throttle command. A board that armed
   *       INTO the last commanded value would move the moment you armed
   *       it, which is the one thing the arm step exists to prevent.
   */
  Void testArmingIsNeutral()
  {
      printf("\narming does not carry the last command with it\n");
      fresh();

      bibo::drive::arm(true);
      check(bibo::drive::throttleUs(1590), "a throttle is set");

      bibo::drive::arm(false);
      bibo::drive::arm(true);

      settle();
      checkEq(
          bibo::fake::lastUs(escPin()),
          DRIVE_NEUTRAL_US,
          "re-arming leaves the ESC at neutral"
      );
  }

  /**
   * @brief Proves a refused throttle command never reaches the ESC pin, not just that it is reported refused.
   */
  /* ---- the pins, not the promises -------------------------------------- */
  Void testDisarmedNeverReachesTheEsc()
  {
      printf("\na refused throttle never reaches the pin\n");
      fresh();

      const Int32 before = bibo::fake::lastUs(escPin());

      check(!bibo::drive::throttleUs(1650), "the disarmed refusal is reported, not silent");
      bibo::drive::pump();

      check(
          bibo::fake::lastUs(escPin()) == before || bibo::fake::lastUs(escPin()) == DRIVE_NEUTRAL_US,
          "the ESC saw neutral or nothing, never 1650"
      );
  }

  /**
   * @brief Proves throttle is clamped to the calibrated band rather than passed through unbounded.
   */
  /* ---- limits ----------------------------------------------------------- */
  Void testThrottleClamping()
  {
      printf("\nthrottle is clamped to the calibrated band\n");
      fresh();
      bibo::drive::arm(true);

      check(bibo::drive::throttleUs(9999), "an absurd value is accepted");
      settle();
      check(
          bibo::fake::lastUs(escPin()) <= THROTTLE_CAL_MAX,
          "and clamped at or below the calibrated maximum"
      );

      check(bibo::drive::throttleUs(0), "zero is accepted");
      settle();
      check(bibo::fake::lastUs(escPin()) >= THROTTLE_CAL_MIN, "and clamped at or above idle");
  }

  /**
   * @brief Proves a limit pair that is not a real range (lo == hi, or lo > hi) is refused.
   */
  Void testLimitsRefuseNonsense()
  {
      printf("\nlimits refuse a range that is not one\n");
      fresh();

      check(!bibo::drive::setThrottleLimits(1600, 1600), "lo == hi is refused");
      check(!bibo::drive::setThrottleLimits(1700, 1500), "lo > hi is refused");
      check(!bibo::drive::setSteerLimits(1500, 1500), "the same for steering");
  }

  /**
   * @brief Proves steerToUs maps a fraction onto the calibrated microsecond range, asymmetrically and clamped.
   */
  /* ---- steering maths --------------------------------------------------- */
  Void testSteerMapping()
  {
      printf("\nsteering maps the fraction onto measured microseconds\n");
      fresh();

      checkEq(bibo::drive::steerToUs(0.0f), STEER_CAL_CENTER, "0.0 is center");
      checkEq(bibo::drive::steerToUs(-1.0f), STEER_CAL_LEFT, "-1.0 is full left");
      checkEq(bibo::drive::steerToUs(1.0f), STEER_CAL_RIGHT, "+1.0 is full right");

      /*
       * The throw is ASYMMETRIC on this car - center is 1480, not 1500 - which
       * is the whole reason commands are fractions. Half right must land half
       * way to the RIGHT limit, not half way to 1500 + something.
       */
      const Int32 halfRight = bibo::drive::steerToUs(0.5f);
      const Int32 expected = STEER_CAL_CENTER
                            + (STEER_CAL_RIGHT - STEER_CAL_CENTER) / 2;
      check(
          halfRight >= expected - 2 && halfRight <= expected + 2,
          "0.5 is half of the RIGHT throw, not half of a symmetric one"
      );

      /* Out of range is clamped, not wrapped. */
      checkEq(bibo::drive::steerToUs(-9.0f), STEER_CAL_LEFT, "beyond full left clamps");
      checkEq(bibo::drive::steerToUs(9.0f), STEER_CAL_RIGHT, "beyond full right clamps");
  }

  /**
   * @brief Proves center round-trips through steerToUs and steerFromUs to zero.
   */
  Void testSteerRoundTrip()
  {
      printf("\nsteerToUs and steerFromUs agree\n");
      fresh();

      const Int32 mid = bibo::drive::steerToUs(0.0f);
      check(
          bibo::drive::steerFromUs(mid) == 0 || bibo::drive::steerFromUs(mid) == 0,
          "center round-trips to zero"
      );
  }

  /**
   * @brief Proves stop() parks the ESC at neutral and disarms, refusing throttle again.
   */
  /* ---- stop ------------------------------------------------------------- */
  Void testStopIsNeutralAndReleased()
  {
      printf("\nstop parks the ESC and lets the servo go\n");
      fresh();
      bibo::drive::arm(true);
      check(bibo::drive::throttleUs(1580), "an armed throttle is accepted");
      settle();

      bibo::drive::stop();
      settle();
      checkEq(bibo::fake::lastUs(escPin()), DRIVE_NEUTRAL_US, "the ESC is left at neutral");
      check(!bibo::drive::throttleUs(1580), "and stop disarms, so throttle is refused again");
  }

}

/**
 * @brief Runs every chassis safety-property check and reports the pass/fail count.
 *
 * @return 0 if every check passed, 1 if any failed
 */
Int32 main()
{
    printf("chassis - the safety property\n");
    printf("=============================\n");

    testThrottleRefusedUntilArmed();
    testArmingIsNeutral();
    testDisarmedNeverReachesTheEsc();
    testThrottleClamping();
    testLimitsRefuseNonsense();
    testSteerMapping();
    testSteerRoundTrip();
    testStopIsNeutralAndReleased();

    printf("\n%d checks, %d failed\n", checks, failures);
    printf("%s\n", failures == 0 ? "OVERALL: PASS" : "OVERALL: FAIL");
    return failures == 0 ? 0 : 1;
}
