/*
 * The car-side encoder module against the fake hal: the lines are set by hand,
 * the interrupt is fired by hand, and what the loop reads is asserted. The
 * decoder itself is test_hall's business; this suite is about the binding -
 * one read of the port, the miss check and its pending-interrupt exclusion,
 * the speed window, zeroing.
 */
#include "../lib/encoder.hxx"

#include <stdio.h>

namespace
{
  Int32 failures = 0;
  Int32 checks = 0;

  Void check(const Bool ok, const CharSeq what)
  {
      ++checks;
      if(!ok)
      {
          ++failures;
          printf("  FAIL: %s\n", what);
      }
  }

  constexpr Int32 A = 11;
  constexpr Int32 B = 12;
  constexpr Int32 C = 13;

  /* The lines to one of the six states, as the motor would leave them. */
  Void setState(const UInt8 abc)
  {
      bibo::fake::setLine(A, (abc & 4u) != 0u);
      bibo::fake::setLine(B, (abc & 2u) != 0u);
      bibo::fake::setLine(C, (abc & 1u) != 0u);
  }

  /* One step: the lines change and the interrupt sees them, msLater on. */
  Void step(const UInt8 abc, const UInt32 msLater)
  {
      bibo::timing::ms(msLater);
      setState(abc);
      bibo::fake::edge();
  }

  Void testOpen(Void)
  {
      printf("-- open: three inputs pulled down, watched, primed from the lines --\n");
      bibo::fake::reset();
      setState(1);
      bibo::encoder::open(A, B, C);
      Int32 inputs = 0;
      Int32 watched = 0;
      for(Size i = 0; i < bibo::fake::count; ++i)
      {
          inputs += bibo::fake::events[i].kind == bibo::fake::EVENT_GPIO_INPUT ? 1 : 0;
          watched += bibo::fake::events[i].kind == bibo::fake::EVENT_GPIO_WATCH ? 1 : 0;
      }
      check(inputs == 3 && watched == 3, "each line is opened as an input and watched for edges");
      const bibo::encoder::Snapshot s = bibo::encoder::read();
      check(s.ticks == 0 && s.edges == 0 && s.state == 1, "primed from the lines, no tick counted");
      check(s.skips == 0 && s.invalid == 0 && s.misses == 0, "and nothing wrong yet");
  }

  Void testForwardAndSpeed(Void)
  {
      printf("-- a turn forward: six ticks, the speed two ways --\n");
      bibo::fake::reset();
      setState(1);
      bibo::encoder::open(A, B, C);
      static constexpr UInt8 SEQ[6] = { 3, 2, 6, 4, 5, 1 };
      for(const UInt8 abc : SEQ)
      {
          step(abc, 10);
          bibo::encoder::pump();
      }
      bibo::encoder::Snapshot s = bibo::encoder::read();
      check(s.ticks == 6 && s.edges == 6, "six edges are six ticks, forward positive");
      check(s.skips == 0 && s.invalid == 0 && s.repeats == 0 && s.misses == 0, "with no errors");
      check(s.periodTps == 100, "10 ms between edges is 100 ticks a second by period");
      check(s.usePeriod, "and below PERIOD_BELOW_TICKS a window the report leads with the period");
      check(s.ticksPerS == 100, "so ticksPerS is the period's number");
      /* The first window closed at 50 ms from open, on the five ticks by then. */
      step(3, 10);
      bibo::encoder::pump();
      s = bibo::encoder::read();
      check(s.windowTps == 100, "the 50 ms window scales its five ticks to 100 a second");
      /* Nothing for a quarter second is a stop. */
      bibo::timing::ms(300);
      bibo::encoder::pump();
      s = bibo::encoder::read();
      check(s.stopped && s.ticksPerS == 0, "no edge for STOPPED_US reads stopped, speed 0");
      check(s.ticks == 7, "and the count stands");
  }

  Void testReverse(Void)
  {
      printf("-- a turn backward counts down --\n");
      bibo::fake::reset();
      setState(1);
      bibo::encoder::open(A, B, C);
      static constexpr UInt8 SEQ[6] = { 5, 4, 6, 2, 3, 1 };
      for(const UInt8 abc : SEQ)
      {
          step(abc, 10);
      }
      const bibo::encoder::Snapshot s = bibo::encoder::read();
      check(s.ticks == -6 && s.edges == 6, "six edges the other way are minus six");
      check(s.periodTps == -100, "and the period speed is signed");
  }

  Void testMissCheck(Void)
  {
      printf("-- the loop's miss check, and the pending interrupt it excludes --\n");
      bibo::fake::reset();
      setState(1);
      bibo::encoder::open(A, B, C);
      /* The lines move and no interrupt fires: the loop notices and feeds it. */
      bibo::timing::ms(10);
      setState(3);
      bibo::encoder::pump();
      bibo::encoder::Snapshot s = bibo::encoder::read();
      check(s.misses == 1, "a change the interrupt did not see is a miss");
      check(
          s.ticks == 1 && s.state == 3,
          "and is fed, so the count and the state are right anyway"
      );
      /* The lines move with the interrupt still pending: not a miss, the
       * interrupt is about to see it. */
      bibo::timing::ms(10);
      setState(2);
      bibo::fake::pending = 1u << static_cast<UInt32>(B);
      bibo::encoder::pump();
      s = bibo::encoder::read();
      check(
          s.misses == 1 && s.ticks == 1,
          "a pending edge is left to its interrupt, not counted as a miss"
      );
      bibo::fake::pending = 0;
      bibo::fake::edge();
      s = bibo::encoder::read();
      check(s.ticks == 2 && s.state == 2, "which then counts it");
      bibo::encoder::pump();
      s = bibo::encoder::read();
      check(s.misses == 1, "and the loop, agreeing with the state, adds nothing");
  }

  Void testInvalidAndZero(Void)
  {
      printf("-- an unplugged cable, then zero --\n");
      bibo::fake::reset();
      setState(1);
      bibo::encoder::open(A, B, C);
      step(3, 10);
      step(0, 10);
      bibo::encoder::Snapshot s = bibo::encoder::read();
      check(s.invalid == 1 && s.ticks == 1, "000 is invalid, and the count keeps what it had");
      step(3, 10);
      step(2, 10);
      s = bibo::encoder::read();
      check(s.ticks == 2 && s.invalid == 1, "a valid state re-primes and counting resumes");
      bibo::encoder::zero();
      s = bibo::encoder::read();
      check(
          s.ticks == 0 && s.edges == 0 && s.invalid == 0 && s.misses == 0,
          "zero clears every count"
      );
      check(s.state == 2, "and keeps the state");
      step(6, 10);
      s = bibo::encoder::read();
      check(
          s.ticks == 1 && s.skips == 0,
          "so the next edge is one step, not a priming and not a skip"
      );
  }
}

int main(Void)
{
    printf("encoder suite\n");
    testOpen();
    testForwardAndSpeed();
    testReverse();
    testMissCheck();
    testInvalidAndZero();
    printf("%s: %d checks, %d failures\n", failures == 0 ? "PASS" : "FAIL", checks, failures);
    return failures == 0 ? 0 : 1;
}
