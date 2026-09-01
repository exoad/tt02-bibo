/*
 * ---------------------------------------------------------------------------
 * status - the onboard LED as something a person can read across a room.
 *
 * Solid, off, or blinking at a rate you choose. A rate carries information a
 * single lamp cannot, and carries it with no laptop and no port open at all.
 *
 *     status::open();                    once, at startup
 *     status::blink(2.0f);               two full cycles a second
 *     while(true) { status::tick(); }    often, from the main loop
 *
 * tick() must be called regularly or the blink stalls mid-cycle - it does not
 * run on an interrupt on purpose: a bus driven from a handler gets re-entered.
 *
 * File-scope state, so this belongs to a single translation unit.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "hal.hxx"

namespace bibo::status
{

    /* 0 means not blinking: solid at whatever status::solid() last set. */
    inline Float32 hzNow  = 0.0f;
    inline Bool    lit    = false;
    inline UInt64  nextUs = 0;

    /**
     * @brief Microseconds the lamp holds each state at a given blink rate.
     *
     * Half a period per toggle, so `hz` counts full on-off cycles per second
     * rather than edges. One flash a second is status::blink(1.0f), which is
     * what anybody watching would call it.
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
     * @return true when the lamp is usable; false when the wireless chip did
     *         not start
     *
     * @note A false is worth reporting rather than swallowing. Everything in
     *       this module keeps working without the lamp - it simply cannot be
     *       seen, and a dark lamp reads as a stopped program.
     */
    inline Bool open(Void)
    {
        const Bool ok = led::open();
        hzNow  = 0.0f;
        lit    = false;
        nextUs = 0;
        led::write(false);
        return ok;
    }

    /**
     * @brief Stops any blink and holds the lamp.
     *
     * @param on true to hold it lit, false to hold it dark
     */
    inline Void solid(const Bool on)
    {
        hzNow = 0.0f;
        lit   = on;
        led::write(on);
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
            solid(false);
            return;
        }
        hzNow  = hz;
        nextUs = timing::nowUs() + halfPeriodUs(hz);
    }

    /**
     * @brief Advances the blink, toggling the lamp when its half-period expires.
     *
     * Call often, from the program's main loop. Cheap when there is nothing to
     * do: it returns immediately when the lamp is solid, and again when the
     * next toggle is still in the future.
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
     * @brief Whether the lamp is lit this instant.
     *
     * For a program that reports its own state.
     *
     * @return true when the lamp is currently on
     *
     * @note While blinking, this is whichever half of the cycle you happened to
     *       sample. rate() is the authoritative answer to "is it blinking".
     */
    inline Bool isLit(Void)
    {
        return lit;
    }

    /**
     * @brief The blink rate.
     *
     * @return full cycles per second, or 0 when the lamp is solid
     */
    inline Float32 rate(Void)
    {
        return hzNow;
    }

    /**
     * @brief A short attention-getting burst of flashes.
     *
     * For power-on, before any host could be listening - the one moment a
     * blocking flash costs nothing, and the one moment somebody genuinely wants
     * to know the program started.
     *
     * @param flashes how many on-off flashes to give
     * @param msEach milliseconds the lamp holds each half of a flash
     *
     * @warning BLOCKS for `flashes * msEach * 2` milliseconds. Nothing else
     *          runs during it, so it does not belong anywhere the car is
     *          already moving.
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
