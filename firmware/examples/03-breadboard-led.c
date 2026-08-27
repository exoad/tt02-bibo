/*
 * 03 - An LED on the breadboard
 *
 * TEACHES: what a GPIO actually is - a pin you can drive high or low.
 * NEEDS:   one LED, one resistor (220R to 1k), two jumper wires.
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   GP28 is physical pin 34. The nearest ground is physical pin 38.
 *
 *   GP28 (pin 34) ---[ resistor ]--- LED long leg (anode, +)
 *                                    LED short leg (cathode, -) --- GND (pin 38)
 *
 * THE RESISTOR IS NOT OPTIONAL. An LED is not a resistor: past its forward
 * voltage it will draw as much current as the source can give, and a bare LED
 * across a 3.3 V pin is a short. It limits current whichever side of the LED it
 * sits on, so either order works.
 *
 * THE LED HAS A DIRECTION. The long leg is the anode and goes toward the pin,
 * the short leg toward ground. Backwards it simply does not light - it will not
 * be damaged, so if nothing happens, try it the other way round first.
 *
 * ---------------------------------------------------------------------------
 * WHY gpioOpen TAKES A DIRECTION
 *
 * A pin is not an output until you say so. The SDK wants two calls for this
 * (gpio_init then gpio_set_dir) and forgetting the second is the single most
 * common first-hour mistake: gpio_put on a pin that is still an input does
 * nothing at all, silently. gpioOpen does both so it cannot be half-done.
 *
 * PINS TO LEAVE ALONE once the car is wired, from docs/wiring.md:
 *   GP0/GP1 servo and ESC, GP4/GP5 I2C, GP10-GP13 ToF, GP15 encoder,
 *   GP16-GP19 SD card. GP28 is free, which is why it is used here.
 */

#include "pico2w.h"

#define LED_PIN 28
#define DELAY_MS 300

Int32 main(Void)
{
    serialOpen();

    gpioOpen(LED_PIN, PIN_DIR_OUT);

    /* The onboard LED blinks in the opposite phase, so you can see both and
     * tell at a glance that the program is the thing running. */
    const Bool haveOnboard = ledOpen();

    while(true)
    {
        gpioWrite(LED_PIN, true);
        if(haveOnboard)
        {
            ledWrite(false);
        }
        sleepMs(DELAY_MS);

        gpioWrite(LED_PIN, false);
        if(haveOnboard)
        {
            ledWrite(true);
        }
        sleepMs(DELAY_MS);
    }

    return 0;
}
