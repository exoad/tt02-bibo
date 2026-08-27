/*
 * 11 - Blink the external LED on GP3. ONLY that.
 *
 * The onboard LED is deliberately never touched here. It stays dark. If
 * anything lights up, it is your LED.
 *
 * ---------------------------------------------------------------------------
 * WIRING - GP3 IS PHYSICAL PIN 5
 *
 *   GP3 (physical pin 5) ---[ resistor ]--- LED long leg (anode)
 *                                           LED short leg --- GND
 *
 * Counting from the USB end, down the left-hand side:
 *
 *   pin 1  GP0
 *   pin 2  GP1
 *   pin 3  GND      <- NOT GP3. This one catches everybody.
 *   pin 4  GP2
 *   pin 5  GP3      <- this is the one
 *   pin 6  GP4
 *
 * Ground: pin 3 is right there, two up from GP3, and is the easiest to reach.
 *
 * ---------------------------------------------------------------------------
 * HOW TO TELL IT IS RUNNING WITHOUT THE ONBOARD LED
 *
 * It prints. Open the Pico 2 W tab in the hub, connect, and you will see a line
 * every time the pin changes:
 *
 *     GP3 HIGH  (physical pin 5)  - LED should be ON
 *     GP3 LOW   (physical pin 5)  - LED should be OFF
 *
 * If those lines appear and the LED does not light, the code is doing its job
 * and the fault is on the breadboard: LED backwards, resistor bridging the
 * legs instead of in the path, ground not on a ground pin, or the wire on the
 * wrong physical pin.
 *
 * If those lines do NOT appear, the board is not running this program - check
 * that you flashed THIS sketch and not another one.
 *
 * SLOW ON PURPOSE. One second each way, so there is no chance of mistaking a
 * fast blink for "dimly on".
 */

#include "pico2w.h"

#define LED_PIN 3
#define DELAY_MS 1000

Int32 main(Void)
{
    /* Always first - without it the board never appears on USB and you would
     * have to hold BOOTSEL to flash it again. */
    serialOpen();
    serialWaitForHost(3000);

    serialPrintLine("");
    serialPrintLine("=== 11-led-gp3: blinking GP3 (physical pin 5) only ===");
    serialPrintLine("The onboard LED is not used by this sketch.");
    serialPrintLine("");

    gpioOpen(LED_PIN, PIN_DIR_OUT);

    while(true)
    {
        gpioWrite(LED_PIN, true);
        serialPrintLine("GP3 HIGH  (physical pin 5)  - LED should be ON");
        sleepMs(DELAY_MS);

        gpioWrite(LED_PIN, false);
        serialPrintLine("GP3 LOW   (physical pin 5)  - LED should be OFF");
        sleepMs(DELAY_MS);
    }

    return 0;
}
