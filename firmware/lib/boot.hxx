/* ---------------------------------------------------------------------------
 * boot - the eight lines every program started with.
 *
 * serial::open(), pins::begin(), and something visible when the map is refused.
 * main.cxx, range-view.cxx and speaker.cxx each wrote their own copy, and the
 * copies had already drifted: two of the three printed a conflict message that
 * was wrong in the out-of-range case, and the three disagreed about what to do
 * afterwards.
 *
 * WHY THIS IS NOT IN pins.hxx
 *
 * pins.hxx has no hardware dependency at all - it is a map, a validator and
 * some names - which is what lets firmware/tests compile it on the host with
 * MSVC and check the conflict logic without a board. Reaching for serial and
 * led from there would cost that, and the test is worth more than the file
 * count.
 *
 * WHY halt() IS SEPARATE FROM begin()
 *
 * Because the right answer differs and both are defensible. A SKETCH should
 * stop: returning from main on a Pico leaves the board powered, silent and
 * indistinguishable from a crash, so speaker.cxx blinks instead and that is
 * the better instinct. The APP keeps going: a car whose lamps are unbound is
 * still a car whose watchdog should run. begin() reports; the caller decides.
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.hxx"
#include "pins.hxx"

namespace bibo
{

  namespace boot
  {

    /*
     * Opens the serial console and installs the pin map.
     *
     * Returns false when the map was refused, having already said why on the
     * serial line - so a caller that only wants to stop can ignore the string
     * and a caller that wants to add to it still can.
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

    /*
     * Stops, visibly, forever.
     *
     * NEVER RETURNS, and that is the point. `return 1` from main on an RP2350
     * leaves the board powered and doing nothing, which looks exactly like a
     * board that failed to flash - the one diagnosis that sends you back to the
     * toolchain when the problem is a wire. A blink says the firmware ran, got
     * as far as checking, and refused.
     *
     * 120 ms is deliberately not the status:: heartbeat: this is not a health
     * signal, it is a stop, and it should not be mistaken for one.
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

  } // namespace boot

} // namespace bibo
