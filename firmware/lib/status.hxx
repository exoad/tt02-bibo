/* ---------------------------------------------------------------------------
 * status - the onboard LED as something a person can read across a room.
 *
 * Solid, off, or blinking at a rate you choose. That is the whole surface, and
 * it is enough: a rate carries information a single lamp cannot, and it carries
 * it without a laptop, a terminal, or the port being open at all. On a car that
 * has driven itself under a table, that is sometimes the only channel left.
 *
 * ---- why this is not just led::write ----------------------------------------
 *
 * A blink is a TIMER, and a timer somebody hand-rolls in main() is a timer that
 * gets copied into the next program with its bugs. The half-period arithmetic,
 * the "what is it doing right now" state, and the tolerance for a wireless chip
 * that never came up are all things every program wants and none should own.
 *
 * ---- calling it -----------------------------------------------------------
 *
 *     status::open();                    once, at startup
 *     status::blink(2.0f);               two full cycles a second
 *     while(true) { status::tick(); }    often, from the main loop
 *
 * status::tick() must be called regularly or the blink stalls mid-cycle - it does
 * not run on an interrupt, on purpose. An interrupt handler that drives a
 * peripheral over a bus is a good way to find out what your bus does when it is
 * re-entered.
 *
 * ---- one copy -------------------------------------------------------------
 *
 * File-scope state, so this belongs to a single translation unit - the same
 * deal chassis.h makes, and for the same reason.
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.hxx"

namespace bibo
{

  namespace status
  {

    /* 0 means not blinking: solid at whatever status::solid() last set. */
    static Float32 hzNow  = 0.0f;
    static Bool    lit    = false;
    static UInt64  nextUs = 0;

    /*
     * Half a period per toggle, so `hz` counts full on-off cycles per second
     * rather than edges. One flash a second is status::blink(1.0f), which is what
     * anybody watching would call it.
     */
    static UInt64 halfPeriodUs(Float32 hz)
    {
        return static_cast<UInt64>(500000.0f / hz);
    }

    /*
     * Brings up the LED. Returns false if the wireless chip did not start, which is
     * worth reporting rather than swallowing: everything here keeps working, it
     * just cannot be seen, and a silent lamp reads as a stopped program.
     */
    static Bool open(Void)
    {
        const Bool ok = led::open();
        hzNow  = 0.0f;
        lit    = false;
        nextUs = 0;
        led::write(false);
        return ok;
    }

    /* Stops any blink and holds the lamp. */
    static Void solid(Bool on)
    {
        hzNow = 0.0f;
        lit   = on;
        led::write(on);
    }

    /* Blinks at `hz` full cycles per second. Zero or less is solid off. */
    static Void blink(Float32 hz)
    {
        if(hz <= 0.0f)
        {
            solid(false);
            return;
        }
        hzNow  = hz;
        nextUs = timing::nowUs() + halfPeriodUs(hz);
    }

    /* Call often. Cheap when there is nothing to do. */
    static Void tick(Void)
    {
        if(hzNow <= 0.0f)
        {
            return;
        }
        if(timing::nowUs() < nextUs)
        {
            return;
        }

        lit = !lit;
        led::write(lit);
        nextUs = timing::nowUs() + halfPeriodUs(hzNow);
    }

    /* What the lamp is doing this instant, for a program that reports its own
     * state. */
    static Bool isLit(Void)
    {
        return lit;
    }

    /* The blink rate, or 0 when solid. Authoritative over status::isLit(): a
     * non-zero rate means blinking whichever half of the cycle you happened to
     * sample. */
    static Float32 rate(Void)
    {
        return hzNow;
    }

    /*
     * A short attention-getting burst, blocking.
     *
     * For power-on, before any host could possibly be listening - which is the one
     * moment a blocking flash costs nothing, and the one moment somebody genuinely
     * wants to know the program started.
     */
    static Void hello(Int32 flashes, UInt32 msEach)
    {
        for(Int32 i = 0; i < flashes; ++i)
        {
            led::write(true);
            timing::ms(msEach);
            led::write(false);
            timing::ms(msEach);
        }
        lit = false;
    }


  } // namespace status

} // namespace bibo