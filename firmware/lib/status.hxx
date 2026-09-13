/*
 * status - the onboard LED as a heartbeat: a few flashes at power-on, then a
 * steady blink. tick() is polled from the main loop rather than run from an
 * interrupt, so the LED's bus is never re-entered. File-scope state, so this
 * belongs to a single translation unit.
 */
#pragma once

#include "hal.hxx"

namespace bibo::status
{
    /* 0 means not blinking. */
    inline Float32 hzNow = 0.0f;
    inline Bool    lit = false;
    inline UInt64  nextUs = 0;

    /** hz is full on-off cycles per second and must be above zero. */
    inline UInt64 halfPeriodUs(const Float32 hz)
    {
        return 500000.0f / hz;
    }

    /**
     * Parks the lamp dark. False when the Pico 2 W's wireless chip did not start,
     * which led::present() then reports.
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

    /** Zero or less parks it dark. Only arms the next toggle: nothing blinks unless tick() runs. */
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
     * BLOCKS for flashes * msEach * 2 ms, so it belongs before the main loop and
     * nowhere the car can be moving.
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
