/*
 * 06 - Fading an LED with PWM
 *
 * TEACHES: PWM - how a pin that can only be on or off produces brightness.
 * NEEDS:   the LED from sketch 03, on GP28 through its resistor.
 *
 * ---------------------------------------------------------------------------
 * THE IDEA
 *
 * A GPIO has two states. There is no "half on". So instead the pin is switched
 * on and off very fast, and the FRACTION of the time it spends on - the DUTY
 * CYCLE - is what your eye averages into brightness.
 *
 *   duty 0.0  = off the whole cycle      = dark
 *   duty 0.5  = on half the cycle        = about half brightness
 *   duty 1.0  = on the whole cycle       = full
 *
 * This is exactly what sketch 02 warned about: blink fast enough and it stops
 * looking like blinking. Here that is the point rather than the bug.
 *
 * 1 kHz is chosen because it is far above what an eye can follow. Drop it to
 * 30 Hz and you will see flicker; drop it to 5 Hz and it is just blinking again.
 *
 * ---------------------------------------------------------------------------
 * WHY BRIGHTNESS DOES NOT LOOK LINEAR
 *
 * Ramp the duty evenly and the fade looks wrong - it rushes at the dark end and
 * crawls at the bright end. Human brightness perception is roughly logarithmic,
 * so an even ramp in POWER is an uneven ramp in APPEARANCE.
 *
 * Squaring the fraction, as below, is a cheap approximation that looks far more
 * even. TRY: replace `f * f` with `f` and watch the difference.
 *
 * NOTE: a pin used for PWM is under the PWM hardware's control, not gpioWrite's.
 * Call gpioOpen on it again if you want a plain output back.
 */

#include "pico2w.h"

#define LED_PIN 28
#define PWM_HZ 1000
#define STEPS 100

Int32 main(Void)
{
    serialOpen();

    pwmOpen(LED_PIN, PWM_HZ);

    while(true)
    {
        for(Int32 i = 0; i <= STEPS; ++i)
        {
            const Float32 f = (Float32) i / (Float32) STEPS;
            pwmWrite(LED_PIN, f * f);
            sleepMs(10);
        }

        for(Int32 i = STEPS; i >= 0; --i)
        {
            const Float32 f = (Float32) i / (Float32) STEPS;
            pwmWrite(LED_PIN, f * f);
            sleepMs(10);
        }
    }

    return 0;
}
