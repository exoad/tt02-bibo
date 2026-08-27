/*
 * 09 - The car's actual indicator, on your breadboard
 *
 * TEACHES: doing two things at once without sleepMs, using a clock instead.
 * NEEDS:   two LEDs and two resistors, as in sketch 08.
 *
 * ---------------------------------------------------------------------------
 * THIS IS REAL PROJECT CODE
 *
 * The timing below is the TT-02's indicator specification from
 * docs/conventions.md, and hub/src/lights.cpp implements the same rule with 25
 * tests over it. 400 ms on, 267 ms off - a 1.5 Hz flash at 60% duty, which is
 * what a road car does. Even flashing looks subtly wrong beside a real one.
 *
 * WIRING - the front and rear lamp of ONE side:
 *
 *   GP26 (pin 31) ---[ resistor ]--- LED --- GND     "front"
 *   GP27 (pin 32) ---[ resistor ]--- LED --- GND     "rear"
 *
 * They share one clock on purpose. Give each lamp its own timer and they drift
 * apart - invisible for the first minute, obviously wrong after five.
 *
 * ---------------------------------------------------------------------------
 * THE REAL LESSON: STOP USING sleepMs FOR TIMING
 *
 * Every sketch so far has slept to make time pass. That works while the program
 * has one job. It falls apart the moment it has two, because sleeping is the
 * program refusing to do anything else - a sketch asleep for 400 ms cannot read
 * a sensor, answer the host, or feed a watchdog.
 *
 * So the loop below never sleeps for the flash. It reads the clock, works out
 * which part of the cycle it is in, and sets the lamps accordingly. The loop
 * spins freely and could do ten other things in the same pass.
 *
 * That shape - look at the time, decide, repeat - is how the car's firmware has
 * to be written, because steering, throttle, lidar and the watchdog all need
 * servicing and none of them can wait 400 ms for the indicators.
 *
 * TRY: add a serialPrintf inside the loop. It runs thousands of times a second,
 * which is the proof that nothing is blocked.
 */

#include "pico2w.h"

#define LAMP_FRONT 26
#define LAMP_REAR 27

/* From docs/conventions.md. 1.5 Hz at 60% duty. */
#define BLINK_ON_MS 400
#define BLINK_OFF_MS 267
#define BLINK_PERIOD_MS (BLINK_ON_MS + BLINK_OFF_MS)

Int32 main(Void)
{
    serialOpen();

    gpioOpen(LAMP_FRONT, PIN_DIR_OUT);
    gpioOpen(LAMP_REAR, PIN_DIR_OUT);

    const Bool haveOnboard = ledOpen();
    Bool       lastLit     = false;

    while(true)
    {
        /*
         * ONE clock, read once, driving both lamps. This is what keeps them in
         * step - and it is why the phase is derived from an absolute time
         * rather than accumulated, so it cannot drift at all.
         */
        const UInt32 phase = nowMs() % BLINK_PERIOD_MS;
        const Bool   lit   = (phase < BLINK_ON_MS);

        gpioWrite(LAMP_FRONT, lit);
        gpioWrite(LAMP_REAR, lit);

        if(haveOnboard)
        {
            ledWrite(lit);
        }

        /* Only speak on a change, not thousands of times a second. */
        if(lit != lastLit)
        {
            serialPrintLine(lit ? "lamps on" : "lamps off");
            lastLit = lit;
        }

        /*
         * No sleep for the flash itself. A short one only to keep the USB stack
         * and the rest of the system from being starved by a tight spin - one
         * millisecond is 400 passes per flash, far more than enough.
         */
        sleepMs(1);
    }

    return 0;
}
