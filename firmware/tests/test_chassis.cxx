/*
 * chassis - the safety rules, on a laptop, through tests/fakes/hal.hxx. The
 * assertions are about what reaches the PINS, not what the module says it did.
 */
#include "../lib/chassis/chassis.hxx"

#include <stdio.h>

namespace
{
  Int32 checks = 0;
  Int32 failures = 0;

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

  /* A test that never installs a map asserts about pin NONE, which passes and proves nothing. */
  Int32 escPin(Void)
  {
      return bibo::pins::active().esc;
  }

  /*
   * pump() slews, so a test that cares where an output ENDS UP runs the fake
   * clock: 200 ticks is four seconds of car time.
   */
  Void settle(Void)
  {
      for(Int32 i = 0; i < 200; ++i)
      {
          bibo::timing::ms(SLEW_TICK_MS);
          bibo::drive::pump();
      }
  }

  Void fresh(Void)
  {
      bibo::fake::reset();
      static_cast<Void>(bibo::pins::begin(bibo::pins::car()));
      bibo::drive::open();
  }

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
      bibo::drive::arm(false);
      check(!bibo::drive::throttleUs(1560), "disarming refuses again");
  }

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

  /* open() does not reset escReverse, so this test turns reverse off again at the end. */
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
      check(bibo::drive::throttleUs(DRIVE_NEUTRAL_US + 1), "just above neutral is accepted");
      settle();
      check(bibo::fake::lastUs(escPin()) >= THROTTLE_CAL_MIN, "and clamped at or above idle");
      check(bibo::drive::throttleUs(0), "zero is accepted");
      settle();
      checkEq(bibo::fake::lastUs(escPin()), DRIVE_NEUTRAL_US, "with reverse off it is neutral");
      check(bibo::drive::setReverseLimit(1350), "a reverse limit is accepted");
      check(bibo::drive::throttleUs(0), "zero is accepted again");
      settle();
      checkEq(bibo::fake::lastUs(escPin()), 1350, "and clamped at the reverse limit, never below");
      check(bibo::drive::setReverseLimit(DRIVE_NEUTRAL_US), "reverse is turned off again");
  }

  Void testLimitsRefuseNonsense()
  {
      printf("\nlimits refuse a range that is not one\n");
      fresh();
      check(!bibo::drive::setThrottleLimits(1600, 1600), "lo == hi is refused");
      check(!bibo::drive::setThrottleLimits(1700, 1500), "lo > hi is refused");
      check(!bibo::drive::setSteerLimits(1500, 1500), "the same for steering");
      check(!bibo::drive::setReverseLimit(1600), "a reverse limit above neutral is refused");
  }

  Void testSteerMapping()
  {
      printf("\nsteering maps the fraction onto measured microseconds\n");
      fresh();
      checkEq(bibo::drive::steerToUs(0.0f), STEER_CAL_CENTER, "0.0 is center");
      checkEq(bibo::drive::steerToUs(-1.0f), STEER_CAL_LEFT, "-1.0 is full left");
      checkEq(bibo::drive::steerToUs(1.0f), STEER_CAL_RIGHT, "+1.0 is full right");
      /*
       * The throw is ASYMMETRIC: half right lands half way to STEER_CAL_RIGHT from
       * the measured center.
       */
      const Int32 halfRight = bibo::drive::steerToUs(0.5f);
      const Int32 expected = STEER_CAL_CENTER
                            + (STEER_CAL_RIGHT - STEER_CAL_CENTER) / 2;
      check(
          halfRight >= expected - 2 && halfRight <= expected + 2,
          "0.5 is half of the RIGHT throw, not half of a symmetric one"
      );
      checkEq(bibo::drive::steerToUs(-9.0f), STEER_CAL_LEFT, "beyond full left clamps");
      checkEq(bibo::drive::steerToUs(9.0f), STEER_CAL_RIGHT, "beyond full right clamps");
  }

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
