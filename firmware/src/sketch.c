/*
 * Turn the Pico 2 W's onboard LED on. That is the whole program.
 *
 * THE LED IS NOT A GPIO. On the Pico 2 W it is wired to the CYW43439 wireless
 * chip, so the classic `gpio_put(25, 1)` from any Pico 1 example compiles here,
 * runs, and does nothing at all. The chip has to be brought up first, and that
 * is what ledOpen() does.
 *
 * ledOpen() CAN FAIL - it is a real peripheral on a real bus. If it does and you
 * ignore the return, ledWrite() below silently does nothing and you spend the
 * evening looking at your wiring for a fault that is not there.
 *
 * No wiring needed. Nothing on the breadboard is involved.
 */

#include "pico2w.h"

Int32 main(Void)
{
    if(!ledOpen())
    {
        /* Nothing else to try: without the chip there is no LED to drive. */
        return 1;
    }

    ledWrite(true);

    /*
     * Hold here rather than returning.
     *
     * main() returning on a microcontroller does not "finish" - there is no
     * operating system to hand back to, and what happens next is not something
     * to rely on. A program that is meant to leave the LED on should be a
     * program that is still running with the LED on.
     *
     * sleepMs rather than a bare `while(true) {}` so the core idles instead of
     * spinning at full tilt to accomplish nothing.
     */
    while(true)
    {
        sleepMs(1000);
    }

    return 0;
}
