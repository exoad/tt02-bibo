/*
 * 01 - The onboard LED
 *
 * TEACHES: that the Pico 2 W's own LED is not a GPIO, and how to drive it.
 * NEEDS:   nothing. No breadboard, no wiring.
 *
 * ---------------------------------------------------------------------------
 * THE GOTCHA
 *
 * Every Pico 1 example on the internet says gpio_put(25, 1). On a Pico 2 W that
 * compiles, runs, and does absolutely nothing - the LED is wired to the
 * CYW43439 wireless chip, not to a pin on the RP2350. The chip has to be woken
 * up first, which is what ledOpen() does.
 *
 * ledOpen() returns false if the chip does not start. Check it. If you do not,
 * ledWrite() below quietly does nothing and you will go looking for a fault in
 * hardware that is fine.
 */

#include "pico2w.h"

Int32 main(Void)
{
    /* Always first. Without it the board never appears on USB and the only way
     * to reflash is holding BOOTSEL while plugging the cable in. */
    serialOpen();

    if(!ledOpen())
    {
        return 1;
    }

    ledWrite(true);

    /*
     * main() returning on a microcontroller does not "finish" - there is no
     * operating system to hand back to. A program meant to leave the LED on has
     * to keep running. sleepMs rather than an empty loop so the core idles.
     */
    while(true)
    {
        sleepMs(1000);
    }

    return 0;
}
