/*
 * boot - serial::open(), pins::begin(), and something visible when the map is
 * refused. Every image started with the same eight lines.
 *
 * Not in pins.hxx: that file has no hardware dependency, which is what lets
 * firmware/tests compile it on the host and check the conflict logic without a
 * board. Reaching for serial and led from there would cost that.
 *
 * halt() is separate from begin() because the right answer differs. A sketch
 * should stop; the app keeps going, since a car whose lamps are unbound is
 * still a car whose watchdog should run. begin() reports; the caller decides.
 */
#pragma once

#include "hal.hxx"
#include "pins.hxx"

namespace bibo::boot
{

    /**
     * @brief Opens the serial console and installs the pin map.
     *
     * The first call an image makes. Everything in lib/ reads its GPIO numbers
     * from the map installed here, so nothing below this line works until it
     * has run.
     *
     * On refusal it has already printed the reason on the serial line, so a
     * caller that only wants to stop can ignore the text, and one that wants to
     * add to it still can.
     *
     * @param wiring the pin map this image is claiming - pins::car() for the
     *               car, or one the image builds itself
     * @return true when the map was installed; false when it was refused for a
     *         pin conflict, with the reason already on the serial line
     */
    [[nodiscard]] static Bool begin(const pins::Map& wiring)
    {
        serial::open();

        if(!pins::begin(wiring))
        {
            serial::printf("ERR %s\n", pins::conflictText());
            return false;
        }
        return true;
    }

    /**
     * @brief Stops, visibly, forever.
     *
     * NEVER RETURNS, and that is the point. `return 1` from main on an RP2350
     * leaves the board powered and doing nothing, which looks exactly like a
     * board that failed to flash - the one diagnosis that sends you back to the
     * toolchain when the problem is a wire. A blink says the firmware ran, got
     * as far as checking, and refused.
     *
     * @note The 120 ms period is deliberately NOT the status:: heartbeat. This
     *       is not a health signal, it is a stop, and it must not be mistaken
     *       for one at a glance.
     */
    inline Void halt(Void)
    {
        while(true)
        {
            led::write(true);
            timing::ms(120);
            led::write(false);
            timing::ms(120);
        }
    }

}
