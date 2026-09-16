/*
 * encoder - the motor's three hall sensors as a wheel encoder, on the car's own
 * Pico. hall.hxx judges the six-state sequence and never sees a GPIO; this file
 * binds it to three input lines through interrupts and hands the loop a
 * snapshot. Header-only, file-scope state: one encoder per board.
 *
 * Every edge is an interrupt that reads all three lines at once, so a state is
 * judged whole and never from a stale pin. A two-pole motor makes six edges a
 * turn, 2,000 a second at 20,000 RPM; the interrupt costs about a microsecond,
 * so ten times that still leaves the core mostly idle. As a check on that
 * promise, pump() compares the lines it can see with the state the interrupts
 * last recorded, and a difference is COUNTED (misses) rather than corrected
 * quietly - with one exclusion: an edge whose interrupt is pending, raised in
 * the moment between the loop's read and its check, is not a miss, the
 * interrupt is about to see it. Nothing acts on the count; it is a number a
 * person reads on the bench.
 *
 * The lines are pulled DOWN, so an unplugged sensor cable reads 000 - an
 * invalid state that shows as `invalid` - rather than floating into counts.
 */
#pragma once

#include "hal.hxx"
#include "hall.hxx"

namespace bibo::encoder
{
  /* What the loop reads: the decoder's counts and the speed two ways. */
  struct Snapshot
  {
      Int32  ticks = 0;
      Int32  ticksPerS = 0;    /* the estimate the report leads with */
      Int32  windowTps = 0;
      Int32  periodTps = 0;
      Bool   usePeriod = false;
      Bool   stopped = false;
      UInt32 edges = 0;
      UInt32 skips = 0;
      UInt32 invalid = 0;
      UInt32 repeats = 0;
      UInt32 misses = 0;
      UInt8  state = 0;        /* the newest ABC bits */
  };

  /* Written by the interrupt, read by the loop under a disabled-interrupt window. */
  inline hall::Decoder decoder;
  inline volatile UInt32 misses = 0;
  inline Pin pinA = -1;
  inline Pin pinB = -1;
  inline Pin pinC = -1;
  inline Int32 windowStartTicks = 0;
  inline Int32 windowTicks = 0;
  inline timing::Deadline window = 0;
  inline Bool opened = false;

  /* The three lines as ABC bits, from ONE read of the port. */
  inline UInt8 lines(Void)
  {
      const UInt32 all = gpio::readAll();
      return static_cast<UInt8>(
          (((all >> static_cast<UInt32>(pinA)) & 1u) << 2) | (((all >> static_cast<UInt32>(pinB)) & 1u) << 1)
          | ((all >> static_cast<UInt32>(pinC)) & 1u)
      );
  }

  inline Void onEdge(Void)
  {
      decoder.feed(lines(), timing::nowUs());
  }

  /**
   * Binds the lines and starts counting. The decoder starts from the lines as
   * they are, so the first edge is a step and not a priming.
   */
  inline Void open(const Pin a, const Pin b, const Pin c)
  {
      pinA = a;
      pinB = b;
      pinC = c;
      gpio::openInputDown(a);
      gpio::openInputDown(b);
      gpio::openInputDown(c);
      decoder = hall::Decoder();
      decoder.feed(lines(), timing::nowUs());
      misses = 0;
      windowStartTicks = 0;
      windowTicks = 0;
      window = timing::armMs(hall::WINDOW_MS);
      gpio::watchEdges(a, onEdge);
      gpio::watchEdges(b, onEdge);
      gpio::watchEdges(c, onEdge);
      opened = true;
  }

  /** Every pass of the loop: the miss check and the speed window. */
  inline Void pump(Void)
  {
      if(!opened)
      {
          return;
      }
      {
          const UInt32 saved = sync::disable();
          const UInt8 live = lines();
          const Bool pending = gpio::edgePending(pinA) || gpio::edgePending(pinB) || gpio::edgePending(pinC);
          if(decoder.primed && live != decoder.state && !pending)
          {
              /* Fed so the decoder judges it (a skip, most likely) rather than
               * staying wrong until the next edge. */
              ++misses;
              decoder.feed(live, timing::nowUs());
          }
          sync::restore(saved);
      }
      if(timing::reached(window))
      {
          window = timing::armMs(hall::WINDOW_MS);
          const UInt32 saved = sync::disable();
          const Int32 now = decoder.ticks;
          sync::restore(saved);
          windowTicks = now - windowStartTicks;
          windowStartTicks = now;
      }
  }

  [[nodiscard]] inline Snapshot read(Void)
  {
      const UInt32 saved = sync::disable();
      const hall::Decoder snap = decoder;
      const UInt32 missed = misses;
      sync::restore(saved);
      const hall::Speed s = hall::speed(windowTicks, snap, timing::nowUs());
      Snapshot out;
      out.ticks = snap.ticks;
      out.windowTps = s.windowTps;
      out.periodTps = s.periodTps;
      out.usePeriod = s.usePeriod;
      out.stopped = s.stopped;
      out.ticksPerS = s.usePeriod ? s.periodTps : s.windowTps;
      out.edges = snap.edges;
      out.skips = snap.skips;
      out.invalid = snap.invalid;
      out.repeats = snap.repeats;
      out.misses = missed;
      out.state = snap.state;
      return out;
  }

  /** Every count back to zero; the state the lines are in is kept. */
  inline Void zero(Void)
  {
      const UInt32 saved = sync::disable();
      const UInt8 state = decoder.state;
      decoder = hall::Decoder();
      decoder.feed(state, timing::nowUs());
      misses = 0;
      windowStartTicks = 0;
      windowTicks = 0;
      sync::restore(saved);
  }
}
