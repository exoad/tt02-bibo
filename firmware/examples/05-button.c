/*
 * 05 - Reading a button
 *
 * TEACHES: inputs, pull-ups, and why a button reads BACKWARDS.
 * NEEDS:   a push button (or just a jumper wire you touch to ground).
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   GP14 is physical pin 19. Ground is physical pin 18, right beside it.
 *
 *   GP14 (pin 19) --- button --- GND (pin 18)
 *
 * No resistor needed: the internal pull-up does that job.
 *
 * If you have no button, touch a jumper wire between pin 19 and pin 18. That is
 * a button.
 *
 * ---------------------------------------------------------------------------
 * WHY PRESSED READS AS false - THE THING THAT CONFUSES EVERYONE
 *
 * A pin that is not connected to anything is FLOATING. It is not 0 and it is
 * not 1; it picks up noise and reads randomly. So an input needs to be pulled
 * somewhere by default.
 *
 * PIN_PULL_UP connects a weak resistor from the pin to 3.3 V inside the chip.
 * So:
 *
 *   button NOT pressed -> the pull-up wins            -> reads HIGH (true)
 *   button pressed     -> the pin is tied to ground   -> reads LOW  (false)
 *
 * This is called ACTIVE LOW, and it is the normal way to wire a button. It is
 * why the code below says `!gpioRead(...)` to mean "pressed".
 *
 * TRY: comment out the gpioPull line. The reading becomes noise, and you will
 * see the LED flicker with nothing touching the board.
 */

#include "pico2w.h"

#define BUTTON_PIN 14

Int32 main(Void)
{
    serialOpen();
    serialWaitForHost(3000);

    gpioOpen(BUTTON_PIN, PIN_DIR_IN);
    gpioPull(BUTTON_PIN, PIN_PULL_UP);

    const Bool haveLed = ledOpen();
    Bool       wasDown = false;

    while(true)
    {
        /* Active low: the pin is pulled to ground when the button is down. */
        const Bool isDown = !gpioRead(BUTTON_PIN);

        if(haveLed)
        {
            ledWrite(isDown);
        }

        /*
         * Only print on a CHANGE, not every pass. Printing every loop would
         * produce thousands of identical lines a second and tell you nothing.
         */
        if(isDown != wasDown)
        {
            serialPrintLine(isDown ? "pressed" : "released");
            wasDown = isDown;
        }

        /*
         * 10 ms is also a crude debounce. A mechanical switch does not close
         * cleanly - the contacts bounce for a millisecond or two and a fast loop
         * sees one press as several. Sampling slowly steps over most of it.
         */
        sleepMs(10);
    }

    return 0;
}
