/*
 * 02 - Blink
 *
 * TEACHES: the shape of every embedded program - set up once, then loop forever.
 * NEEDS:   nothing.
 *
 * ---------------------------------------------------------------------------
 * WHY THE DELAY IS NOT DECORATION
 *
 * Take the sleepMs calls out and the LED does not "blink very fast" - it looks
 * dimly on. The RP2350 runs at 150 MHz, so the loop would toggle the LED tens of
 * millions of times a second and your eye would average it into a steady glow.
 *
 * That is not a bug, it is the whole idea behind PWM, which sketch 06 uses on
 * purpose to control brightness.
 *
 * TRY: change DELAY_MS to 50, then to 2. Watch where "blinking" stops being
 * blinking and becomes "on but dim".
 */

#include "pico2w.h"

#define DELAY_MS 400

Int32 main(Void)
{
    serialOpen();

    if(!ledOpen())
    {
        return 1;
    }

    while(true)
    {
        ledWrite(true);
        sleepMs(DELAY_MS);

        ledWrite(false);
        sleepMs(DELAY_MS);
    }

    return 0;
}
