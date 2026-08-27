/*
 * A scratch program, edited from the hub's Code view and flashed onto the board.
 *
 * This file is OVERWRITTEN by the GUI every time you press Build & Flash. It is
 * checked in so the target always compiles from a clean clone, but treat it as
 * scratch space, not as firmware: anything worth keeping gets its own .c file
 * and its own target in CMakeLists.txt.
 *
 * ---------------------------------------------------------------------------
 * RIGHT NOW: bringing up the SPI colour display.
 *
 *   Display   Pico          why
 *   GND       GND
 *   VCC       3V3           these modules are 3.3 V parts
 *   SCL/SCK   GP18          SPI0 SCK - fixed by the silicon, not a free choice
 *   SDA/MOSI  GP19          SPI0 TX
 *   RES       GP20
 *   DC        GP21          low = command, high = pixel data
 *   CS        GP17
 *   BLK       3V3           always on
 *
 * st77xx.h has the driver and, at the top, the ONE line that says whether this
 * is an ST7789 or an ST7735. If the screen lights but shows nothing sensible,
 * flip it and reflash. That is the fastest way to find out which you own.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS DRAWS A TEST PATTERN FIRST
 *
 * These panels are WRITE-ONLY. Nothing can be read back, no command returns an
 * acknowledgement, and tftInit() can only fail if the SPI pins are wrong - so
 * there is no return code anywhere that means "the display is working".
 *
 * The picture is the test. Eight known colour bars in a known order, with the
 * corners marked, tells you in one glance whether the bus works, whether the
 * bytes are in the right order, whether the geometry is right and whether the
 * colours are inverted. A "hello world" that just prints text tells you almost
 * none of that, and when it fails it fails silently.
 *
 * WHAT YOU SHOULD SEE, in order down the screen:
 *
 *   white  yellow  cyan  green  magenta  red  blue  black
 *
 * That is the standard bar order, deliberately, so it can be checked against
 * any reference. Then:
 *
 *   colours in the WRONG ORDER (blue first)   red and blue are swapped:
 *                                             change MADCTL in st77xx.h to 0x08
 *   everything a photographic negative        flip PANEL_INVERT
 *   bars present but shifted or wrapped       wrong PANEL_W/H or the offsets
 *   nothing at all, backlight on              wrong panel type, or DC/RES swapped
 *   snow, tearing, intermittent               SPI too fast - drop PANEL_HZ
 */

#include "pico2w.h"
#include "st77xx.h"

#define STATUS_MS 500

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

    const Bool haveTft = tftInit();

    if(!haveTft)
    {
        /*
         * The only failure this driver can actually detect: the SCK pin does not
         * belong to an SPI controller, so no bus was ever created. Everything
         * else has to be judged by looking at the screen.
         */
        while(true)
        {
            serialPrintLine("TFT: bad SPI pins - check PIN_TFT_SCK in st77xx.h");
            sleepMs(1000);
        }
    }

    serialPrintLine("TFT: initialised, drawing test pattern");

    /* ---- the colour bars -------------------------------------------------- */

    const UInt16 BARS[8] = {
        TFT_WHITE, TFT_YELLOW, TFT_CYAN,  TFT_GREEN,
        TFT_MAGENTA, TFT_RED,  TFT_BLUE,  TFT_BLACK,
    };

    /* Computed rather than hard-coded so this is right on both panel sizes.
     * The last bar takes the remainder, because 240/8 divides evenly and
     * 160/8 does too but the next panel someone plugs in might not. */
    const Int32 barH = PANEL_H / 8;
    for(Int32 i = 0; i < 8; ++i)
    {
        const Int32 h = (i == 7) ? (PANEL_H - (barH * 7)) : barH;
        tftRect(0, barH * i, PANEL_W, h, BARS[i]);
    }

    /* Corner marks. If one is missing, the visible area is not the size the
     * driver thinks it is - which is the offset problem, and it is otherwise
     * genuinely hard to spot. */
    tftRect(0, 0, 8, 8, TFT_RED);
    tftRect(PANEL_W - 8, 0, 8, 8, TFT_GREEN);
    tftRect(0, PANEL_H - 8, 8, 8, TFT_BLUE);
    tftRect(PANEL_W - 8, PANEL_H - 8, 8, 8, TFT_WHITE);

    /* ---- something to read ------------------------------------------------ */

    tftRect(0, PANEL_H / 2 - 26, PANEL_W, 52, TFT_BLACK);
    tftText(6, PANEL_H / 2 - 20, "TT02-AUTO", TFT_ORANGE, TFT_BLACK, 2);
    tftText(6, PANEL_H / 2 + 2,  "DISPLAY OK", TFT_WHITE, TFT_BLACK, 1);

    Utf8 line[32];
#if PANEL_ST7789
    tftText(6, PANEL_H / 2 + 14, "ST7789 240X240", TFT_GREY, TFT_BLACK, 1);
#else
    tftText(6, PANEL_H / 2 + 14, "ST7735 128X160", TFT_GREY, TFT_BLACK, 1);
#endif

    /* ---- alive ------------------------------------------------------------ */

    /*
     * A counter, so a frozen board and a working one look different. A static
     * picture proves the display was written to ONCE; it says nothing about
     * whether the program is still running, and those two failures look
     * identical on a screen that holds its last image.
     */
    Int32 ticks = 0;
    while(true)
    {
        snprintf(line, sizeof(line), "UP %d S", ticks / 2);

        /* Only the strip that changes is redrawn. Repainting the whole screen
         * twice a second would work and would also flicker, because there is no
         * back buffer here - what you draw is what is on the glass. */
        tftText(6, PANEL_H - 20, line, TFT_CYAN, TFT_BLACK, 1);

        /* The onboard LED as a heartbeat, deliberately NOT a second thing to
         * debug: if the screen is blank but this blinks, the program is running
         * and the fault is in the display or its wiring. */
        if(ledOpen())
        {
            ledWrite((ticks & 1) != 0);
        }

        if((ticks % 20) == 0)
        {
            serialPrintf("up %d s, panel %dx%d\n", ticks / 2, PANEL_W, PANEL_H);
        }

        ++ticks;
        sleepMs(STATUS_MS);
    }

    return 0;
}
