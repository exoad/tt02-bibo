/*
 * 10 - Moving the steering servo
 *
 * TEACHES: hobby servo control, and it is the car's actual next milestone.
 * NEEDS:   the Power HD 1501MG servo, and power for it. Read ALL of this first.
 *
 * ===========================================================================
 *  SAFETY - these are from docs/wiring.md and each one has bitten somebody
 * ===========================================================================
 *
 *  1. THE CAR GOES ON A STAND, WHEELS OFF THE GROUND, for every first run of
 *     new code. No exceptions. This is a program that moves a steering rack.
 *
 *  2. COMMON GROUND between the Pico and the ESC is MANDATORY. Signal and
 *     ground cross between the two power domains; power never does. A missing
 *     common ground presents as erratic or absent servo response and is NOT a
 *     code bug - you will debug software for an hour over a missing wire.
 *
 *  3. NEVER connect BEC 5 V to the Pico while USB is attached. During
 *     development the Pico is USB-powered. Back-feeding the 5 V rail from the
 *     BEC with USB also connected risks both.
 *
 *  4. THE SERVO DOES NOT RUN OFF THE PICO. It wants ~6 V and stalls at several
 *     amps; the Pico's 3V3 rail cannot supply a fraction of that. Servo power
 *     comes from the ESC's BEC.
 *
 * ---------------------------------------------------------------------------
 * WIRING the Power HD 1501MG
 *
 *   The cable is black/white. The conductor WITH THE WHITE STRIPE is signal.
 *   The MIDDLE pin is +5 V - servo convention, always.
 *   The remaining outer conductor is ground.
 *
 *   signal (white stripe) --- GP0, physical pin 1
 *   middle                --- BEC 5 V, NOT the Pico
 *   ground                --- BEC ground AND Pico GND, joined  <- rule 2
 *
 * ---------------------------------------------------------------------------
 * HOW A HOBBY SERVO IS COMMANDED
 *
 * Not by voltage, and not by PWM duty in the usual sense. A servo listens for a
 * PULSE, repeated 50 times a second, and it is the WIDTH of that pulse that
 * says where to go:
 *
 *   1000 us  full one way
 *   1500 us  centre
 *   2000 us  full the other way
 *
 * The gap between pulses carries no information. That is why servoOpen() sets
 * 50 Hz and servoWriteUs() takes microseconds rather than a percentage.
 *
 * servoWriteUs CLAMPS to 1000-2000. Driving a servo past its travel stalls it
 * against its own end stop, where it draws locked-rotor current and cooks
 * itself - quietly, with no sound to warn you.
 *
 * The 1501MG's deadband is 4 us, so commands finer than that do nothing. There
 * is no point stepping in ones.
 *
 * ---------------------------------------------------------------------------
 * THE SAME CODE DRIVES THE ESC
 *
 * An ESC speaks the identical protocol on GP1: 1500 us is neutral, above is
 * forward, below is reverse. DO NOT point this sketch at the ESC with the
 * wheels on the ground.
 */

#include "pico2w.h"

#define STEER_PIN 0

/* Deliberately narrower than the full 1000-2000 travel. A steering rack has
 * mechanical limits well inside what the servo can reach, and finding them by
 * driving into them is how linkages get bent. Widen it once you have watched
 * where the wheels actually stop. */
#define SWEEP_MIN_US 1300
#define SWEEP_MAX_US 1700
#define STEP_US 5
#define STEP_MS 15

Int32 main(Void)
{
    serialOpen();
    serialWaitForHost(3000);

    serialPrintLine("servo sweep - WHEELS OFF THE GROUND");

    servoOpen(STEER_PIN);

    /* Centre first and pause. If the servo jumps hard here, the linkage is
     * already loaded and something is mechanically wrong - stop and look. */
    servoCenter(STEER_PIN);
    sleepMs(1000);

    while(true)
    {
        for(UInt32 us = SWEEP_MIN_US; us <= SWEEP_MAX_US; us += STEP_US)
        {
            servoWriteUs(STEER_PIN, us);
            sleepMs(STEP_MS);
        }

        for(UInt32 us = SWEEP_MAX_US; us >= SWEEP_MIN_US; us -= STEP_US)
        {
            servoWriteUs(STEER_PIN, us);
            sleepMs(STEP_MS);
        }

        serialPrintf("swept %u..%u us   chip %.1f C\n",
                     SWEEP_MIN_US, SWEEP_MAX_US, (Float64) tempC());
    }

    return 0;
}
