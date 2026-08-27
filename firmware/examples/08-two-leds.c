/*
 * 08 - Two LEDs, and arrays of pins
 *
 * TEACHES: driving several pins from one array instead of copy-pasting code.
 * NEEDS:   two LEDs and two resistors. Three or four is better - the code does
 *          not care how many, which is the point.
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   GP26 (physical pin 31) ---[ resistor ]--- LED --- GND
 *   GP27 (physical pin 32) ---[ resistor ]--- LED --- GND
 *
 * Both grounds can share one rail on the breadboard - that is what the long
 * strips down the side are for. Every ground pin on the Pico is the same
 * ground, so any of pins 3, 8, 13, 18, 23, 28, 33 or 38 will do.
 *
 * Want more? GP22 (pin 29) and GP21 (pin 27) are free too. Add them to the
 * array below and the sequences pick them up without another line of code.
 *
 * ---------------------------------------------------------------------------
 * WHY AN ARRAY
 *
 * With two LEDs you could just write two variables. With five you would be
 * copying the same four lines five times, and the day you move a pin you would
 * have to find every copy. The array is not showing off - it is the difference
 * between a program you can change and one you have to rewrite.
 *
 * COUNT is derived from the array with sizeof rather than written out, so it
 * cannot disagree with reality. Adding a pin is a one-line change.
 */

#include "pico2w.h"

static const Pin LEDS[] = { 26, 27 };

#define COUNT ((Int32) (sizeof(LEDS) / sizeof(LEDS[0])))
#define STEP_MS 150

Int32 main(Void)
{
    serialOpen();

    for(Int32 i = 0; i < COUNT; ++i)
    {
        gpioOpen(LEDS[i], PIN_DIR_OUT);
    }

    while(true)
    {
        /* ---- a chase: one lit at a time, running along the row ---- */
        for(Int32 i = 0; i < COUNT; ++i)
        {
            for(Int32 k = 0; k < COUNT; ++k)
            {
                gpioWrite(LEDS[k], k == i);
            }
            sleepMs(STEP_MS);
        }

        /* ---- and back, so it bounces rather than jumping to the start ---- */
        for(Int32 i = COUNT - 2; i > 0; --i)
        {
            for(Int32 k = 0; k < COUNT; ++k)
            {
                gpioWrite(LEDS[k], k == i);
            }
            sleepMs(STEP_MS);
        }

        /* ---- all together, twice ---- */
        for(Int32 f = 0; f < 2; ++f)
        {
            for(Int32 k = 0; k < COUNT; ++k)
            {
                gpioWrite(LEDS[k], true);
            }
            sleepMs(STEP_MS);

            for(Int32 k = 0; k < COUNT; ++k)
            {
                gpioWrite(LEDS[k], false);
            }
            sleepMs(STEP_MS);
        }
    }

    return 0;
}
