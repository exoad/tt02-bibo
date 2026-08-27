/*
 * A scratch program, edited from the hub's Code view and flashed onto the board.
 *
 * This file is OVERWRITTEN by the GUI every time you press Build & Flash. It is
 * checked in so the target always compiles from a clean clone, but treat it as
 * scratch space, not as firmware: anything worth keeping gets its own .c file
 * and its own target in CMakeLists.txt.
 *
 * pico2w.h is the SDK wrapped in this project's own naming. Everything it
 * offers is in the editor's completion list: start typing gpio, led, servo,
 * adc, serial, pwm or watchdog.
 *
 * TWO LEDs blink below, and they are not the same kind of thing:
 *
 *   the ONBOARD LED is on the CYW43439 wireless chip, not on a GPIO. It needs
 *   ledOpen() first and cannot be reached with gpioWrite() at any pin number.
 *   The classic gpio_put(25, 1) from a Pico 1 example compiles here, runs, and
 *   does nothing at all.
 *
 *   the BREADBOARD LED is a plain GPIO with a resistor in series, which is the
 *   thing that actually teaches you what a GPIO is.
 *
 * WIRING for the breadboard one (docs/wiring.md has the full map):
 *
 *   GP28 is physical pin 34. Nearest ground is physical pin 38.
 *
 *   GP28 (34) ---[ resistor 220R-1k ]--- LED long leg (anode)
 *                                        LED short leg (cathode) --- GND (38)
 *
 *   The resistor may sit on either side of the LED; it limits the current
 *   either way. Without one the LED is a short across a 3.3 V pin and both it
 *   and the pin are at risk.
 *
 * GP28 is free in this project. GP0/GP1 are the servo and ESC, GP4/GP5 are I2C,
 * GP10-GP13 are the ToF XSHUT lines, GP15 is the encoder and GP16-GP19 are SPI
 * for the SD card, so do not blink any of those once the car is wired.
 */

#include "pico2w.h"

#define LED_PIN 3
#define DELAY_MS 400

Int32 main(Void)
{
    /*
     * FIRST, and in every sketch you write, even one that prints nothing.
     *
     * This starts the USB stack. Without it the board runs fine and never
     * enumerates - no COM port for the flasher to reboot at 1200 baud - and the
     * only way to flash it again is holding BOOTSEL while plugging the cable.
     * See the note above serialOpen() in pico2w.h.
     */
    serialOpen();

    gpioOpen(LED_PIN, PIN_DIR_OUT);

    /* The wireless chip is a real peripheral and can fail to start. If it does,
     * the breadboard LED still blinks - losing one LED should not cost the
     * other. */
    const Bool haveOnboard = ledOpen();

    while(true)
    {
        gpioWrite(LED_PIN, true);
        if(haveOnboard)
        {
            ledWrite(false);   /* opposite phase, so both are visible at once */
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
