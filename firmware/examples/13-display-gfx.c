/*
 * Drawing with gfx.h - the 2D API over the panel.
 *
 * Flash 12-display-spi.c FIRST. That one proves the wiring with colour bars,
 * and until it does there is nothing here that can tell you why a blank screen
 * is blank. This sketch assumes the panel already works.
 *
 * ---------------------------------------------------------------------------
 * WHAT gfx.h GIVES YOU OVER st77xx.h
 *
 * st77xx.h is the driver: chip selects, command bytes, address windows. Real,
 * necessary, and not what you want to think about while drawing a dial.
 *
 * gfx.h is the canvas. Shapes, text, colour, clipping - and a BACK BUFFER, so
 * you draw a whole frame in RAM and then show it in one go. That last part is
 * what makes animation possible at all: drawing straight to the glass means the
 * viewer watches you clear the screen and redraw it, every frame, as a flash.
 *
 * The shape of every frame:
 *
 *     gfxClear(...)        start from a known background
 *     ...draw...           in any order; later covers earlier
 *     gfxPresent()         push it. Nothing is visible until this runs.
 *
 * Forget gfxPresent() and the screen simply never changes, which is the one
 * mistake this API makes easy - so it is worth knowing in advance.
 */

#include "pico2w.h"
#include "gfx.h"

#define FRAME_MS 33          /* about 30 fps, which the panel can just about do */

Int32 main(Void)
{
    serialOpen();

    if(!gfxInit())
    {
        while(true)
        {
            serialPrintLine("gfx: bad SPI pins - check PIN_TFT_SCK in st77xx.h");
            sleepMs(1000);
        }
    }

    serialPrintf("gfx: %dx%d ready\n", PANEL_W, PANEL_H);

    Int32 frame = 0;

    while(true)
    {
        /* ---- background ---------------------------------------------------
         * A vertical gradient, one span per row. Cheap, and it makes everything
         * drawn on top of it read as being ON something rather than floating. */
        for(Int32 y = 0; y < PANEL_H; ++y)
        {
            const UInt8 t = (UInt8) ((y * 255) / PANEL_H);
            gfxHLine(0, y, PANEL_W, gfxBlend(GFX_NAVY, GFX_BLACK, t));
        }

        /* ---- a rotating hue bar ------------------------------------------
         * gfxHsv is integer, so sweeping hue costs nothing. */
        for(Int32 x = 0; x < PANEL_W; ++x)
        {
            const Int32 hue = ((x * 360) / PANEL_W) + (frame * 2);
            gfxVLine(x, 8, 10, gfxHsv(hue, 220, 200));
        }

        /* ---- a card, to show rounded rectangles and layering -------------- */
        const Int32 cardX = 12;
        const Int32 cardY = 30;
        const Int32 cardW = PANEL_W - 24;
        const Int32 cardH = 62;

        gfxRoundRectFill(cardX, cardY, cardW, cardH, 8, GFX_DARKGREY);
        gfxRoundRect(cardX, cardY, cardW, cardH, 8, GFX_GREY);

        gfxTextColour(GFX_ORANGE);
        gfxTextTransparent();          /* over the card, not over a black box */
        gfxTextSize(2);
        gfxTextAt(cardX + 10, cardY + 10, "TT02-AUTO");

        gfxTextColour(GFX_WHITE);
        gfxTextSize(1);
        gfxCursor(cardX + 10, cardY + 34);
        gfxPrintf("FRAME %d", frame);

        /* Right-aligned, so a number that changes width does not shuffle the
         * layout about - which is the entire reason alignment exists. */
        gfxTextColour(GFX_CYAN);
        gfxTextAligned(cardX + cardW - 10, cardY + 34, "GFX", GFX_ALIGN_RIGHT);

        /* ---- a bouncing ball, to show it is really animating -------------- */
        const Int32 span = PANEL_W - 40;
        Int32       p    = (frame * 3) % (span * 2);
        if(p >= span)
        {
            p = (span * 2) - p;        /* fold, so it comes back rather than jumps */
        }
        gfxCircleFill(20 + p, PANEL_H - 60, 12, GFX_YELLOW);
        gfxCircle(20 + p, PANEL_H - 60, 12, GFX_WHITE);

        /* ---- clipping ------------------------------------------------------
         * Everything below is confined to this strip, so the triangle can be
         * drawn far too large and simply be cut off - which is what clipping is
         * for, and why animation does not need a bounds check per shape. */
        gfxClip(0, PANEL_H - 34, PANEL_W, 34);
        gfxRectFill(0, PANEL_H - 34, PANEL_W, 34, GFX_BLACK);

        const Int32 tipX = (frame * 4) % (PANEL_W + 80) - 40;
        gfxTriangleFill(tipX, PANEL_H - 60,
                        tipX + 40, PANEL_H - 4,
                        tipX - 40, PANEL_H - 4,
                        GFX_PURPLE);

        gfxTextColour(GFX_GREY);
        gfxTextTransparent();
        gfxTextAt(6, PANEL_H - 14, "CLIPPED");
        gfxClipReset();

        /* Nothing above has touched the panel. THIS is what does. */
        gfxPresent();

        ++frame;
        sleepMs(FRAME_MS);
    }

    return 0;
}
