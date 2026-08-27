/*
 * A scratch program, edited from the hub's Code view and flashed onto the board.
 *
 * This file is a SLOT. The GUI overwrites it with whichever sketch is open when
 * you press Build & Flash, because it is the file CMakeLists.txt compiles for
 * the `sketch` target. Anything you want to keep goes in firmware/examples/ and
 * gets its own name - see the README there.
 *
 * ---------------------------------------------------------------------------
 * RIGHT NOW: bringing up the SPI colour display, drawn with gfx.h.
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
 * st77xx.h is the driver and holds the two lines that describe your panel:
 * PANEL_ST7789 / PANEL_ST7735, and PANEL_ST7789_SIZE for which glass.
 *
 * gfx.h is what this file actually draws with - shapes, text, colour, and a
 * back buffer. Nothing reaches the screen until gfxPresent().
 *
 * ---------------------------------------------------------------------------
 * WHY THIS STILL DRAWS A TEST PATTERN
 *
 * The panel is WRITE-ONLY. Nothing reads back, no command acknowledges, and
 * gfxInit() can only fail if the SPI pins are wrong - so no return code
 * anywhere means "the display is working". The picture is the test.
 *
 * WHAT YOU SHOULD SEE, in order down the screen:
 *
 *   white  yellow  cyan  green  magenta  red  blue  black
 *
 * That is the standard bar order, so it can be checked against any reference.
 * Then, if it is not that:
 *
 *   colours in the WRONG ORDER (blue first)   red and blue swapped: change the
 *                                             MADCTL byte in st77xx.h to 0x08
 *   everything a photographic negative        flip PANEL_INVERT
 *   bars present but shifted or wrapping      wrong PANEL_ST7789_SIZE
 *   a corner mark missing                     same - the visible area is not
 *                                             the size the driver thinks
 *   nothing at all, backlight on              wrong panel type, or DC/RES swapped
 *   snow, tearing, intermittent               SPI too fast - drop PANEL_HZ
 */

#include "pico2w.h"
#include "gfx.h"

#define STATUS_MS 500

Int32 main(Void)
{
    /*
     * FIRST, and in every sketch you write, even one that prints nothing.
     *
     * This starts the USB stack. Without it the board runs fine and never
     * enumerates - no COM port for the flasher to reboot at 1200 baud - and the
     * only way to flash it again is holding BOOTSEL while plugging the cable.
     */
    serialOpen();

    if(!gfxInit())
    {
        /*
         * The only failure that can be detected: the SCK pin does not belong to
         * an SPI controller, so no bus was ever created. Everything else has to
         * be judged by looking at the screen.
         */
        while(true)
        {
            serialPrintLine("gfx: bad SPI pins - check PIN_TFT_SCK in st77xx.h");
            sleepMs(1000);
        }
    }

    serialPrintf("gfx: %dx%d ready\n", PANEL_W, PANEL_H);

    /* ---- the test pattern, drawn once ------------------------------------
     *
     * Outside the loop deliberately. It never changes, and gfxPresent() only
     * pushes the rows that were TOUCHED since the last one - so redrawing this
     * every frame would turn a twenty-row update into a whole-screen one for no
     * visible difference.
     */

    const UInt16 BARS[8] = {
        GFX_WHITE, GFX_YELLOW, GFX_CYAN,  GFX_GREEN,
        GFX_MAGENTA, GFX_RED,  GFX_BLUE,  GFX_BLACK,
    };

    /* Computed rather than hard-coded, so this is right on every panel size.
     * The last bar takes the remainder - 240 and 160 both divide by 8, but the
     * next panel plugged in might not. */
    const Int32 barH = PANEL_H / 8;
    for(Int32 i = 0; i < 8; ++i)
    {
        const Int32 h = (i == 7) ? (PANEL_H - (barH * 7)) : barH;
        gfxRectFill(0, barH * i, PANEL_W, h, BARS[i]);
    }

    /* Corner marks. A missing one means the visible area is not the size the
     * driver thinks it is, which is otherwise genuinely hard to spot. */
    gfxRectFill(0, 0, 8, 8, GFX_RED);
    gfxRectFill(PANEL_W - 8, 0, 8, 8, GFX_GREEN);
    gfxRectFill(0, PANEL_H - 8, 8, 8, GFX_BLUE);
    gfxRectFill(PANEL_W - 8, PANEL_H - 8, 8, 8, GFX_WHITE);

    /* ---- a card over the bars --------------------------------------------
     *
     * This is the part the bars cannot show you: that drawing ON TOP of what is
     * already there works at all. Straight to the panel each of these would be
     * a separate round trip with the half-finished states visible; in the back
     * buffer it is just memory, and the viewer only ever sees the result.
     */
    const Int32 cardY = (PANEL_H / 2) - 30;
    const Int32 cardH = 60;

    gfxRoundRectFill(8, cardY, PANEL_W - 16, cardH, 8, GFX_BLACK);
    gfxRoundRect(8, cardY, PANEL_W - 16, cardH, 8, GFX_GREY);

    gfxTextTransparent();          /* over the card, not over its own box */
    gfxTextColour(GFX_ORANGE);
    gfxTextSize(2);
    gfxTextAt(18, cardY + 10, "TT02-AUTO");

    gfxTextColour(GFX_WHITE);
    gfxTextSize(1);
    gfxTextAt(18, cardY + 32, "DISPLAY OK");

    gfxTextColour(GFX_GREY);
    gfxTextAligned(PANEL_W - 18, cardY + 32,
#if PANEL_ST7789
                   "ST7789",
#else
                   "ST7735",
#endif
                   GFX_ALIGN_RIGHT);

    gfxPresent();

    /* ---- alive ------------------------------------------------------------
     *
     * A counter, because a static picture proves the screen was written to ONCE
     * and says nothing about whether the program is still running - and on a
     * panel that holds its last image those two failures look identical.
     *
     * An OPAQUE text background rather than clearing the strip first: the new
     * digits erase the old as they draw, in one pass, with nothing in between.
     */
    Int32 ticks = 0;
    while(true)
    {
        gfxTextColour(GFX_CYAN);
        gfxTextBackground(GFX_BLACK);
        gfxTextSize(1);

        gfxCursor(6, PANEL_H - 20);
        gfxPrintf("UP %d S    ", ticks / 2);

        /* Only the rows the text touched are sent - about twenty of them
         * instead of all 240, which is the whole point of tracking them. */
        gfxPresent();

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
