/*
 * hall - a sensored brushless motor's three hall lines as a quadrature-like
 * encoder: six states around a mechanical turn of a two-pole rotor, one bit
 * changing at each step, so the direction is which neighbour the state moved
 * to and a skipped state is a fact, not a guess.
 *
 * PURE: no SDK, no GPIO, no clock, and plain arrays, since Array is the host's.
 * The state arrives as three bits and the time as microseconds the caller read,
 * so firmware/tests/test_hall.cxx drives the decoder on the host and
 * encoder/main.cxx only wires it to the pins.
 *
 * Which way round the sequence is "forward" is a convention here (ascending
 * through SEQUENCE is +1); the bench decides whether that is the car's forward,
 * and FORWARD_ASCENDING is the one place to flip it.
 */
#pragma once

#include "shared.hxx"

namespace hall
{
  /* The 120-degree hall order as ABC bits: each step changes exactly one bit. */
  constexpr UInt8 SEQUENCE[6] = { 1, 3, 2, 6, 4, 5 };

  /* State to its place in SEQUENCE; 0 and 7 are no state at all. */
  constexpr Int8 INDEX_OF[8] = { -1, 0, 2, 1, 4, 5, 3, -1 };

  constexpr Bool FORWARD_ASCENDING = true;

  /* Per window of this many milliseconds, for the windowed speed. */
  constexpr UInt32 WINDOW_MS = 50;

  /*
   * Below this many ticks in a window the windowed speed is too coarse (each
   * tick is 20 ticks per second at 50 ms) and the edge period is reported
   * instead; above it the period is a single noisy edge and the window wins.
   */
  constexpr Int32 PERIOD_BELOW_TICKS = 10;

  /* No edge for this long is a stopped motor: both speeds read 0. */
  constexpr UInt32 STOPPED_US = 250000;

  struct Decoder
  {
      Int32 ticks = 0;         /* signed, +1 per step forward */
      UInt32 edges = 0;        /* valid steps, either way */
      UInt32 skips = 0;        /* a state two or three steps away: an edge was missed */
      UInt32 invalid = 0;      /* 0 or 7: a line floating or a wire off */
      UInt32 repeats = 0;      /* an edge that left the state as it was: a glitch */
      UInt8 state = 0;         /* the newest ABC bits */
      Bool primed = false;     /* a valid state has been seen, so a step can be judged */
      Int8 lastDir = 0;        /* -1, 0, +1 */
      UInt64 lastEdgeUs = 0;
      UInt32 periodUs = 0;     /* between the last two valid steps; 0 before there are two */

      [[nodiscard]] UInt32 errors() const
      {
          return skips + invalid;
      }

      /* One reading of the three lines, at `nowUs`. Safe to call from an interrupt. */
      Void feed(const UInt8 next, const UInt64 nowUs)
      {
          const UInt8 bits = next & 7u;
          const Int8 idx = INDEX_OF[bits];
          if(idx < 0)
          {
              ++invalid;
              state = bits;
              primed = false;
              return;
          }
          if(primed && bits == state)
          {
              ++repeats;
              return;
          }
          if(!primed)
          {
              primed = true;
              state = bits;
              lastEdgeUs = nowUs;
              return;
          }
          const Int32 was = INDEX_OF[state];
          const Int32 step = ((idx - was) + 6) % 6;
          state = bits;
          if(step == 1 || step == 5)
          {
              const Int8 dir = (step == 1) == FORWARD_ASCENDING ? 1 : -1;
              ticks += dir;
              lastDir = dir;
              ++edges;
              const UInt64 gap = nowUs - lastEdgeUs;
              periodUs = gap > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<UInt32>(gap);
              lastEdgeUs = nowUs;
              return;
          }
          /* Two or three steps at once: an edge went by unseen. Not a tick, since
           * the direction of a half-turn jump is anyone's guess. */
          ++skips;
          lastEdgeUs = nowUs;
      }
  };

  /* The two speeds and which one to trust, in ticks per second. */
  struct Speed
  {
      Int32 windowTps = 0;     /* the window's ticks scaled to a second */
      Int32 periodTps = 0;     /* one second over the last edge period, signed */
      Bool stopped = false;    /* no edge for STOPPED_US */
      Bool usePeriod = false;  /* which the report should lead with */
  };

  [[nodiscard]] inline Speed speed(const Int32 windowTicks, const Decoder& d, const UInt64 nowUs)
  {
      Speed s;
      s.windowTps = windowTicks * static_cast<Int32>(1000u / WINDOW_MS);
      const UInt64 sinceEdge = nowUs - d.lastEdgeUs;
      s.stopped = !d.primed || sinceEdge > STOPPED_US;
      if(!s.stopped && d.periodUs > 0u)
      {
          s.periodTps = static_cast<Int32>(1000000u / d.periodUs) * d.lastDir;
      }
      if(s.stopped)
      {
          s.periodTps = 0;
          s.windowTps = 0;
      }
      const Int32 magnitude = windowTicks < 0 ? -windowTicks : windowTicks;
      s.usePeriod = !s.stopped && magnitude < PERIOD_BELOW_TICKS && d.periodUs > 0u;
      return s;
  }
}
