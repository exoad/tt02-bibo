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
 * ONE LED blinks below: the breadboard one on LED_PIN, and nothing else.
 *
 * The onboard LED is deliberately left alone - see the block at the bottom of
 * main(). It is on the CYW43439 wireless chip rather than on a GPIO, so it is
 * reached with ledWrite() and NOT with gpioWrite() at any pin number. Blinking
 * both at once is a good demo and a bad diagnostic: when you are trying to work
 * out whether GP3 is doing anything, a second LED blinking on its own schedule
 * is the one thing guaranteed to confuse the answer.
 *
 * So if the LED below does not blink now, the fault is in the wiring or the
 * hardware, and nothing on this board will blink to suggest otherwise.
 *
 * WIRING (docs/wiring.md has the full map):
 *
 *   GP3 is physical pin 5. Nearest ground is physical pin 3, right beside it.
 *
 *   GP3 (5) ---[ resistor 220R-1k ]--- LED long leg (anode)
 *                                      LED short leg (cathode) --- GND (3)
 *
 *   The resistor may sit on either side of the LED; it limits the current
 *   either way. Without one the LED is a short across a 3.3 V pin and both it
 *   and the pin are at risk.
 *
 *   The LONG leg is the one that must end up connected back to GP3, whether the
 *   resistor is between them or not. An LED fitted the other way round is not
 *   damaged, it simply never lights - which looks exactly like a dead pin.
 *
 * GP3 is free in this project. GP0/GP1 are the servo and ESC, GP4/GP5 are I2C,
 * GP10-GP13 are the ToF XSHUT lines, GP15 is the encoder and GP16-GP19 are SPI
 * for the SD card, so do not blink any of those once the car is wired.
 */

#include "pico2w.h"

#define LED_PIN 3 
#define DELAY_MS 100

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

    while(true)
    {
        gpioWrite(LED_PIN, true);
        sleepMs(DELAY_MS);
        gpioWrite(LED_PIN, false);
        sleepMs(DELAY_MS);
    }

    return 0;
}

/*
 * The onboard LED, kept here rather than deleted, because it is worth knowing
 * how to reach and it is not obvious. Paste it back into main() to use it:
 *
 *     const Bool haveOnboard = ledOpen();   // before the loop; can fail, as
 *                                           // the wireless chip is a real
 *                                           // peripheral that has to start up
 *
 *     ledWrite(true);                       // inside the loop
 *     ledWrite(false);
 *
 * gpio_put(25, 1), copied from any Pico 1 example, compiles here and runs here
 * and does nothing whatsoever. On this board that pin is not the LED.
 */
