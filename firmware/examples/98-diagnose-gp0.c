/*
 * 98 - Diagnostic: is GP0 alive, and does PWM reach it?
 *
 * Not a lesson. A test for one question: the drive code says it is writing
 * 1300 us to GP0 and the servo does not move. That has three quite different
 * causes and no amount of staring at the servo tells them apart.
 *
 *   the pin is dead              nothing ever comes out of GP0
 *   the pin works, PWM does not  GPIO toggling works, the PWM peripheral does
 *                                not reach this pin
 *   both work                    the fault is downstream - the servo, its
 *                                supply, or the wire between
 *
 * WIRING
 *
 *   GP0 is physical pin 1, the corner nearest the USB socket.
 *
 *   GP0  ->  resistor (220R to 1k)  ->  LED long leg
 *   LED short leg  ->  any GND pin
 *
 *   The resistor can be on either side of the LED. The LED has a direction:
 *   LONG leg toward the pin, SHORT leg toward ground. Backwards it will not
 *   light and will not be harmed.
 *
 *   TAKE THE SERVO OFF GP0 FIRST. Not because it would be damaged, but because
 *   a servo and an LED on the same pin make the result ambiguous.
 *
 * WHAT YOU SHOULD SEE, in a loop
 *
 *   PHASE 1, 6 s   plain on/off, 2 Hz     obvious blinking
 *   PHASE 2, 6 s   PWM, 2 Hz, duty ramp   smooth fade up and down
 *   PHASE 3, 8 s   servo pulses, 50 Hz    dim but STEADY glow, brightening
 *                                         very slightly across the sweep
 *
 * WHAT THE RESULT MEANS
 *
 *   nothing in any phase       GP0 is not driving, or the LED is backwards, or
 *                              the ground is not reaching a ground pin. Move the
 *                              same LED to GP3 to tell those apart - if it
 *                              blinks there, GP0 is the problem.
 *   phase 1 only               the pin toggles but PWM does not reach it. That
 *                              would be a real firmware fault and the servo is
 *                              innocent.
 *   all three phases           GP0 is fine and so is the PWM path. The servo
 *                              was being driven correctly all along, and the
 *                              fault is the servo or its 5 V supply.
 *
 * Phase 3 is deliberately hard to read: a servo pulse is only 5-10% duty, so
 * the LED sits dim and barely changes. That IS the signal. If phase 2 fades and
 * phase 3 glows at all, the peripheral is doing exactly what the drive code
 * asks of it.
 *
 * The onboard LED counts the phase - one, two, or three blinks between phases -
 * so if THAT is running you know the program is alive and the fault is on the
 * breadboard.
 */

#include "pico2w.h"

#define TEST_PIN 0

/* Long enough to be sure of what you are looking at, short enough that the loop
 * comes back round while you are still watching it. */
#define PHASE1_MS 6000
#define PHASE2_MS 6000
#define PHASE3_MS 8000

/* Counts out the phase number on the onboard LED, which is on the wireless chip
 * rather than a GPIO and so cannot be the thing that is broken. */
static Void markPhase(Bool haveOnboard, Int32 phase)
{
    if(!haveOnboard)
    {
        return;
    }
    for(Int32 i = 0; i < phase; ++i)
    {
        ledWrite(true);
        sleepMs(120);
        ledWrite(false);
        sleepMs(180);
    }
    sleepMs(400);
}

Int32 main(Void)
{
    serialOpen();
    serialWaitForHost(3000);

    serialPrintLine("");
    serialPrintLine("=== 98-diagnose-gp0 ===");
    serialPrintLine("LED on GP0 (physical pin 1) through a resistor to GND.");
    serialPrintLine("Servo OFF GP0 for this test.");
    serialPrintLine("");

    const Bool haveOnboard = ledOpen();
    UInt32     round       = 0;

    while(true)
    {
        /* ---- PHASE 1: is the pin alive at all? -------------------------- */
        markPhase(haveOnboard, 1);
        serialPrintf("round %u  PHASE 1  plain on/off at 2 Hz - "
                     "expect obvious blinking\n", round);

        gpioOpen(TEST_PIN, PIN_DIR_OUT);
        for(UInt32 t = 0; t < PHASE1_MS; t += 500)
        {
            gpioWrite(TEST_PIN, true);
            sleepMs(250);
            gpioWrite(TEST_PIN, false);
            sleepMs(250);
        }

        /* ---- PHASE 2: does the PWM peripheral reach this pin? ------------
         *
         * 200 Hz is far above what an eye resolves, so the LED reads as one
         * steady brightness and the SLOW ramp of the duty cycle is what you
         * see. Blinking here would prove nothing phase 1 did not. */
        markPhase(haveOnboard, 2);
        serialPrintf("round %u  PHASE 2  PWM duty ramp - "
                     "expect a smooth fade up and down\n", round);

        pwmOpen(TEST_PIN, 200);
        for(UInt32 t = 0; t < PHASE2_MS; t += 40)
        {
            /* A triangle, so it fades both ways and neither end is mistaken
             * for the LED having gone out. */
            const UInt32  phase = (t % 2000);
            const Float32 up    = (Float32) phase / 1000.0f;
            pwmWrite(TEST_PIN, (phase < 1000) ? up : (2.0f - up));
            sleepMs(40);
        }
        pwmWrite(TEST_PIN, 0.0f);

        /* ---- PHASE 3: the exact path the drive code uses ----------------- */
        markPhase(haveOnboard, 3);
        serialPrintf("round %u  PHASE 3  servo pulses 1000-2000 us at 50 Hz - "
                     "expect a dim steady glow\n", round);

        servoOpen(TEST_PIN);
        for(UInt32 t = 0; t < PHASE3_MS; t += 100)
        {
            /* 1000 to 2000 and back, the full span the servo would ever see. */
            const UInt32 phase = (t % 4000);
            const UInt32 us    = (phase < 2000)
                               ? (1000u + (phase / 2u))
                               : (2000u - ((phase - 2000u) / 2u));
            servoWriteUs(TEST_PIN, us);

            if((t % 1000) == 0)
            {
                serialPrintf("   holding %u us\n", us);
            }
            sleepMs(100);
        }

        /* Left LOW between rounds so the gap is unmistakable.
         *
         * servoRelease rather than servoWriteUs(0): servoWriteUs CLAMPS to the
         * 1000-2000 us band, so asking it for zero gets you a 1000 us pulse
         * train and an LED that never goes out. */
        servoRelease(TEST_PIN);
        serialPrintLine("");

        ++round;
    }

    return 0;
}
