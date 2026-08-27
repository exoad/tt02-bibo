/*
 * A scratch program, edited from the hub's Code view and flashed onto the board.
 *
 * This file is a SLOT. The GUI overwrites it with whichever sketch is open when
 * you press Build & Flash. Anything worth keeping goes in firmware/examples/.
 *
 * ---------------------------------------------------------------------------
 * RIGHT NOW: fill the screen yellow. That is the whole program.
 *
 * Deliberately the smallest thing that can possibly show something. A blank
 * screen has too many candidate causes, and this removes most of them at once:
 * no back buffer, no font, no text, no animation. Bring the panel up, push one
 * colour at it, stop.
 *
 * WHAT EACH OUTCOME MEANS
 *
 *   yellow screen           the wiring, the SPI mode, the panel type and the
 *                           init sequence are ALL correct. Everything past
 *                           this point is drawing, and drawing is the easy
 *                           part.
 *
 *   black, LED blinking     the program runs and the panel is not listening.
 *                           Try the other PANEL_ define in st77xx.h, then
 *                           check DC and RES - swapping those two is the
 *                           commonest wiring mistake and looks exactly like
 *                           this.
 *
 *   black, LED dead         the program is not running at all. Hold BOOTSEL
 *                           while plugging the cable in, and flash again.
 *
 *   blue instead of yellow  red and blue are swapped: change the MADCTL byte
 *                           in st77xx.h from 0x00 to 0x08.
 *
 *   a dark negative of it   inversion is wrong: flip PANEL_INVERT.
 *
 * Yellow rather than red or white, for a reason. It is bright, it is nothing
 * like the black of a panel that is merely backlit, and it drives two of the
 * three colour channels - so a stuck or miswired channel changes it visibly
 * instead of leaving it looking plausible.
 *
 * ---------------------------------------------------------------------------
 * WIRING
 *
 *   Display   Pico
 *   GND       GND
 *   VCC       3V3          these modules are 3.3 V parts
 *   SCL/SCK   GP18         SPI0 SCK - fixed by the silicon, not a free choice
 *   SDA/MOSI  GP19         SPI0 TX
 *   RES       GP20
 *   DC        GP21         low = command, high = pixel data
 *   CS        GP17
 *   BLK       3V3          always on
 */

#include "pico2w.h"
#include "st77xx.h"

/*
 * st77xx.h rather than gfx.h, on purpose. gfx.h carries a 112 KB back buffer
 * and a font - both worth having, and both two more things that could be the
 * reason nothing appears. A diagnostic should contain as little as possible
 * that is not the thing being diagnosed.
 */

#define BEAT_MS 500

Int32 main(Void)
{
    /*
     * FIRST, always. This starts the USB stack; without it the board runs
     * perfectly and never enumerates, and the only way back in is holding
     * BOOTSEL while plugging the cable in.
     */
    serialOpen();

    const Bool haveLed = ledOpen();

    /*
     * The LED comes on BEFORE the panel is touched, so a board that hangs
     * during bring-up still looks different from a board that is not running
     * at all. That distinction is most of why this sketch exists.
     */
    if(haveLed)
    {
        ledWrite(true);
    }

    const Bool haveTft = tftInit();

    if(!haveTft)
    {
        /* The only failure that can be detected from here: the SCK pin does
         * not belong to an SPI controller, so no bus was ever created. */
        while(true)
        {
            serialPrintLine("tft: bad SPI pins - check PIN_TFT_SCK in st77xx.h");
            sleepMs(1000);
        }
    }

    tftFill(TFT_YELLOW);

    serialPrintf("tft: filled %dx%d with yellow\n", PANEL_W, PANEL_H);

    /*
     * Then nothing but a heartbeat. The screen is never touched again, so
     * whatever is on it from here is what tftFill() put there - if it changes
     * or flickers later, that is the panel or the wiring, not this code.
     */
    UInt32 beats = 0;
    while(true)
    {
        if(haveLed)
        {
            ledWrite((beats & 1u) != 0u);
        }

        if((beats % 10u) == 0u)
        {
            serialPrintf("alive %u s\n", beats / 2u);
        }

        ++beats;
        sleepMs(BEAT_MS);
    }

    return 0;
}
