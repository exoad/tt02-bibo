/*
 * ---------------------------------------------------------------------------
 * status - the onboard LED as a heartbeat: a few flashes at power-on, then a
 * steady blink while the main loop runs.
 *
 *     status::open();                    once, at startup
 *     status::blink(0.5f);               one cycle every two seconds
 *     for(;;) { status::tick(); }        often, from the main loop
 *
 * tick() is polled rather than run from an interrupt, so the LED's bus is never
 * re-entered. File-scope state, so this belongs to a single translation unit.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "hal.hxx"

namespace bibo::status
{

    /* 0 means not blinking. */
    inline Float32 hzNow = 0.0f;
    inline Bool    lit = false;
    inline UInt64  nextUs = 0;

    /**
     * @brief Microseconds the lamp holds each state at a given blink rate.
     *
     * @param hz full on-off cycles per second; must be greater than zero
     * @return microseconds to hold the lamp before the next toggle
     */
    inline UInt64 halfPeriodUs(const Float32 hz)
    {
        return 500000.0f / hz;
    }

    /**
     * @brief Brings up the status LED and parks it dark.
     *
     * @return true when the lamp is usable; false when the Pico 2 W's wireless
     *         chip did not start, which led::present() reports from then on
     */
    inline Bool open(Void)
    {
        const Bool ok = led::open();
        hzNow = 0.0f;
        lit = false;
        nextUs = 0;
        led::write(false);
        return ok;
    }

    /**
     * @brief Starts the lamp blinking at a rate.
     *
     * @param hz full on-off cycles per second; zero or less parks it dark
     *
     * @note Only arms the next toggle. Nothing blinks unless tick() is called
     *       from the program's loop.
     */
    inline Void blink(const Float32 hz)
    {
        if(hz <= 0.0f)
        {
            hzNow = 0.0f;
            lit = false;
            led::write(false);
            return;
        }
        hzNow = hz;
        nextUs = timing::nowUs() + halfPeriodUs(hz);
    }

    /**
     * @brief Advances the blink, toggling the lamp when its half-period expires.
     *
     * Cheap when there is nothing to do, so it can run on every loop pass.
     */
    inline Void tick(Void)
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

    /**
     * @brief A short burst of flashes, for power-on.
     *
     * @param flashes how many on-off flashes to give
     * @param msEach milliseconds the lamp holds each half of a flash
     *
     * @warning BLOCKS for `flashes * msEach * 2` milliseconds, so it belongs
     *          before the main loop and nowhere the car can be moving.
     */
    inline Void hello(const Int32 flashes, const UInt32 msEach)
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

}
