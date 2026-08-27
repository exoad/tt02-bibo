/*
 * 99 - Diagnostic: is it the code, the pin, or the LED?
 *
 * Not a lesson. A test that splits the problem in three, because "the LED does
 * not light" has three quite different causes and guessing between them wastes
 * an evening.
 *
 * It drives FOUR free pins in turn, three seconds each, and says over the
 * serial link which one is live right now. Move your LED's wire from pin to pin
 * while it runs:
 *
 *   GP28 is physical pin 34   <- the diagram's pin. Try this one first.
 *   GP3  is physical pin 5
 *   GP6  is physical pin 9
 *   GP7  is physical pin 10
 *   GP22 is physical pin 29
 *
 * WHAT THE RESULT MEANS
 *
 *   lights on every pin      the code and the pins are fine; the earlier sketch
 *                            or its wiring was the problem
 *   lights only on GP28       the wire is on GP28, exactly as the diagram says.
 *                            Nothing is broken; the sketch was aimed elsewhere.
 *   lights on some, not one   that pin is damaged - use another, nothing else
 *                            is wrong. This is what a no-resistor LED does.
 *   lights on none            the LED is backwards, the resistor is across it
 *                            instead of in series, or the ground is not
 *                            actually reaching a ground pin
 *
 * The onboard LED follows along, so if THAT blinks the program is definitely
 * running and the fault is on the breadboard.
 *
 * Remember the LED has a direction: LONG leg toward the pin, SHORT leg toward
 * ground. Backwards it will not light and will not be harmed.
 */

#include "pico2w.h"

/* GP28 FIRST, because that is the pin the Electronic Clinic diagram uses and
 * therefore the one most likely to already have your LED on it. GP3 second,
 * since that is where you thought it was. */
static const Pin PINS[] = { 28, 3, 6, 7, 22 };
static const Int32 PHYSICAL[] = { 34, 5, 9, 10, 29 };

#define COUNT ((Int32) (sizeof(PINS) / sizeof(PINS[0])))
#define HOLD_MS 3000

Int32 main(Void)
{
    serialOpen();
    serialWaitForHost(3000);

    serialPrintLine("");
    serialPrintLine("=== 99-diagnose-gp3 ===");
    serialPrintLine("Move the LED wire between pins while this runs.");
    serialPrintLine("");

    for(Int32 i = 0; i < COUNT; ++i)
    {
        gpioOpen(PINS[i], PIN_DIR_OUT);
        gpioWrite(PINS[i], false);
    }

    const Bool haveOnboard = ledOpen();
    UInt32     round       = 0;

    while(true)
    {
        for(Int32 i = 0; i < COUNT; ++i)
        {
            /* Exactly one pin high at a time, so there is no ambiguity about
             * which one is being tested. */
            for(Int32 k = 0; k < COUNT; ++k)
            {
                gpioWrite(PINS[k], k == i);
            }

            if(haveOnboard)
            {
                ledWrite((i % 2) == 0);
            }

            serialPrintf("round %u: GP%d (physical pin %d) is HIGH now\n",
                         round, (Int32) PINS[i], PHYSICAL[i]);

            sleepMs(HOLD_MS);
        }
        ++round;
    }

    return 0;
}
