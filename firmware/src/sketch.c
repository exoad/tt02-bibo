/*
 * Blink an LED on a breadboard.
 *
 * WIRING - GP28 is physical pin 34, ground is physical pin 38:
 *
 *   GP28 (34) --[ 220R-1k ]-- LED long leg
 *                             LED short leg -- GND (38)
 *
 * The resistor can go on either side of the LED. Without one the LED
 * is a short across a 3.3 V pin and both are at risk.
 *
 * GP28 is free. Do not blink GP0/GP1 (servo, ESC), GP4/GP5 (I2C),
 * GP10-13 (ToF), GP15 (encoder) or GP16-19 (SD) once the car is wired.
 *
 * pico2w.h is the SDK in this project's own naming. Everything it
 * offers is in the completion list: type gpio, servo, adc or serial.
 */

#include "pico2w.h"

#define LED_PIN 28
#define DELAY_MS 400

Int32 main(Void)
{
    gpioOpen(LED_PIN, PIN_DIR_OUT);

    while(true)
    {
        gpioWrite(LED_PIN, true);
        sleepMs(DELAY_MS);
        gpioWrite(LED_PIN, false);
        sleepMs(DELAY_MS);
    }

    return 0;
}
