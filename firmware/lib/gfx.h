/*
 * A 2D drawing API over a Screen: shapes, text, colour and clipping.
 *
 * st77xx.h is the DRIVER - chip selects, command bytes, address windows - and
 * it hands back a Screen. This is what you draw with, and every call takes the
 * screen it draws into:
 *
 *     Screen screen;
 *     gfxOpen(&screen, 240, 280, 0, 20);
 *
 *     while(true)
 *     {
 *         gfxClear(&screen, GFX_NAVY);
 *         gfxRectFill(&screen, 10, 10, 100, 40, GFX_ORANGE);
 *         gfxCircleFill(&screen, 120, 120, 30, GFX_CYAN);
 *
 *         gfxTextColour(&screen, GFX_WHITE);
 *         gfxTextSize(&screen, 2);
 *         gfxTextAt(&screen, 10, 60, "HELLO");
 *
 *         gfxPresent(&screen);
 *     }
 *
 * The colour, the text size and the clip rectangle live in the Screen, not in
 * this file. They belong to the thing being drawn on - a clip rectangle is as
 * much a property of a screen as its width is - and keeping them there is what
 * makes two screens possible rather than one global canvas.
 *
 * ---------------------------------------------------------------------------
 * THE BACK BUFFER, AND WHY IT IS WORTH THE RAM
 *
 * gfxOpen() attaches one. Every call above then draws into memory and
 * gfxPresent() sends it in one burst. 240x320 at 16bpp is 153,600 bytes - 29%
 * of the RP2350's 520 KB - and it buys three things:
 *
 *   NO FLICKER. Without a buffer you draw on the glass, so clearing and
 *   redrawing means the viewer SEES the clear, every frame, as a flash.
 *
 *   OVERDRAW IS FREE. Background, then a shape on top, then text on top of
 *   that. Straight to the panel each is a round trip with the half-finished
 *   states visible.
 *
 *   IT IS FAST. One address window and one burst, and gfxPresent only pushes
 *   the ROWS THAT CHANGED - a ticking counter costs a fraction of a frame.
 *
 * A sketch that wants the RAM back uses tftOpen() and the tft* calls instead;
 * those go straight to the panel and allocate nothing.
 *
 * ---------------------------------------------------------------------------
 * COORDINATES
 *
 * x right, y DOWN, origin top-left, like every other screen API and unlike
 * school maths. Everything is clipped: drawing off the edge is not an error and
 * never corrupts the opposite side, which is what makes animation writable
 * without a bounds check at every call site.
 */
#ifndef TT02_GFX_H
#define TT02_GFX_H

#include "drivers/display.h"

#include <stdarg.h>
#include <stdio.h>

/* ---- colour --------------------------------------------------------------
 *
 * 16-bit 5-6-5, the panel's own format.
 */
#define GFX_RGB(r, g, b) TFT_RGB(r, g, b)

#define GFX_BLACK    TFT_RGB(0, 0, 0)
#define GFX_WHITE    TFT_RGB(255, 255, 255)
#define GFX_RED      TFT_RGB(255, 0, 0)
#define GFX_GREEN    TFT_RGB(0, 255, 0)
#define GFX_BLUE     TFT_RGB(0, 0, 255)
#define GFX_YELLOW   TFT_RGB(255, 255, 0)
#define GFX_CYAN     TFT_RGB(0, 255, 255)
#define GFX_MAGENTA  TFT_RGB(255, 0, 255)
#define GFX_ORANGE   TFT_RGB(255, 140, 0)
#define GFX_GREY     TFT_RGB(128, 128, 128)
#define GFX_DARKGREY TFT_RGB(64, 64, 64)
#define GFX_NAVY     TFT_RGB(12, 16, 32)
#define GFX_PURPLE   TFT_RGB(160, 90, 220)

/*
 * Mixes two colours. `t` is 0 for all of `a`, 255 for all of `b`.
 *
 * Per channel after unpacking, because 5-6-5 cannot be averaged as one integer:
 * the channels would carry into each other and a half-way blend of red and blue
 * would come out an unrelated colour.
 */
static inline UInt16 gfxBlend(UInt16 a, UInt16 b, UInt8 t)
{
    const UInt32 ar = (a >> 11) & 0x1F;
    const UInt32 ag = (a >> 5)  & 0x3F;
    const UInt32 ab = a & 0x1F;

    const UInt32 br = (b >> 11) & 0x1F;
    const UInt32 bg = (b >> 5)  & 0x3F;
    const UInt32 bb = b & 0x1F;

    const UInt32 it = 255u - (UInt32) t;
    const UInt32 r  = ((ar * it) + (br * (UInt32) t)) / 255u;
    const UInt32 g  = ((ag * it) + (bg * (UInt32) t)) / 255u;
    const UInt32 bl = ((ab * it) + (bb * (UInt32) t)) / 255u;

    return (UInt16) ((r << 11) | (g << 5) | bl);
}

static UInt16 gfxDim(UInt16 c, UInt8 amount)
{
    return gfxBlend(c, GFX_BLACK, amount);
}

static inline UInt16 gfxLighten(UInt16 c, UInt8 amount)
{
    return gfxBlend(c, GFX_WHITE, amount);
}

/*
 * Hue 0-359, saturation and value 0-255. Integer throughout - no float, no
 * table - so a rainbow sweep costs nothing on a chip that would rather not.
 */
static UInt16 gfxHsv(Int32 hue, UInt8 sat, UInt8 val)
{
    hue = ((hue % 360) + 360) % 360;

    const UInt32 region = (UInt32) (hue / 60);
    const UInt32 rem    = (UInt32) ((hue - (Int32) (region * 60u)) * 255 / 60);

    const UInt32 p = ((UInt32) val * (255u - sat)) / 255u;
    const UInt32 q = ((UInt32) val * (255u - ((sat * rem) / 255u))) / 255u;
    const UInt32 t = ((UInt32) val * (255u - ((sat * (255u - rem)) / 255u))) / 255u;

    UInt32 r = 0;
    UInt32 g = 0;
    UInt32 b = 0;
    switch(region)
    {
    case 0:  r = val; g = t;   b = p;   break;
    case 1:  r = q;   g = val; b = p;   break;
    case 2:  r = p;   g = val; b = t;   break;
    case 3:  r = p;   g = q;   b = val; break;
    case 4:  r = t;   g = p;   b = val; break;
    default: r = val; g = p;   b = q;   break;
    }
    return GFX_RGB((UInt8) r, (UInt8) g, (UInt8) b);
}

/* ---- the back buffer -----------------------------------------------------
 *
 * ONE buffer, sized for the largest panel this controller drives, because a
 * static array has to be sized when the program is compiled and the panel is
 * not known until gfxOpen() runs.
 *
 * One buffer means one BUFFERED screen. A second screen can still be opened
 * with tftOpen() and drawn on directly; it simply does not get this. Saying so
 * plainly beats pretending otherwise and running off the end of it.
 *
 * The STRIDE is PANEL_MAX_W and not the screen's width, deliberately: the row a
 * pixel lives on has to be computed the same way every time, and rebasing the
 * stride on a resize would invalidate everything already drawn.
 */
static UInt16 gfxBuf[PANEL_MAX_W * PANEL_MAX_H];
static Bool   gfxBufTaken = false;

/* ---- the safe area -------------------------------------------------------
 *
 * Rounded corners are cut into the glass, so the outermost pixels of a panel
 * are addressable and invisible. Anything at a corner is lost, and text along
 * an edge vanishes into the curve.
 *
 * Set the inset once and lay out against these instead of against 0 and
 * width/height. The full rectangle is still reachable - gfxClear() fills it,
 * and a background should - but anything that has to be READ belongs inside.
 *
 *     gfxSafeInset(&screen, 12);
 *     gfxTextAt(&screen, gfxSafeLeft(&screen), gfxSafeTop(&screen), "HELLO");
 *
 * 12 is a reasonable start for a 1.69 inch 240x280. Turn on gfxSafeOutline()
 * for a frame to check against, then take it out.
 */
static Void gfxSafeInset(Screen* s, Int32 inset)
{
    const Int32 most = ((s->width < s->height) ? s->width : s->height) / 3;
    s->safeInset = (inset < 0) ? 0 : ((inset > most) ? most : inset);
}

static Int32 gfxSafeLeft(const Screen* s)
{
    return s->safeInset;
}

static Int32 gfxSafeTop(const Screen* s)
{
    return s->safeInset;
}

static Int32 gfxSafeRight(const Screen* s)
{
    return s->width - s->safeInset;
}

static Int32 gfxSafeBottom(const Screen* s)
{
    return s->height - s->safeInset;
}

static Int32 gfxSafeWidth(const Screen* s)
{
    return s->width - (2 * s->safeInset);
}

static Int32 gfxSafeHeight(const Screen* s)
{
    return s->height - (2 * s->safeInset);
}

/* ---- clipping ------------------------------------------------------------ */

static Void gfxClip(Screen* s, Int32 x, Int32 y, Int32 w, Int32 h)
{
    if(x < 0)
    {
        w += x;
        x = 0;
    }
    if(y < 0)
    {
        h += y;
        y = 0;
    }
    if(x + w > s->width)
    {
        w = s->width - x;
    }
    if(y + h > s->height)
    {
        h = s->height - y;
    }
    if(w < 0)
    {
        w = 0;
    }
    if(h < 0)
    {
        h = 0;
    }

    s->clipX = x;
    s->clipY = y;
    s->clipW = w;
    s->clipH = h;
}

static Void gfxClipReset(Screen* s)
{
    gfxClip(s, 0, 0, s->width, s->height);
}

/* ---- the two primitives everything is built from -------------------------
 *
 * A horizontal run is the fast path in both modes: a memory fill when buffered,
 * and ONE address window when not. Every shape below is expressed as runs for
 * that reason - a filled circle drawn as pixels is a thousand SPI transactions,
 * and drawn as spans it is thirty.
 */
static Void gfxSpan(Screen* s, Int32 x, Int32 y, Int32 len, UInt16 colour)
{
    if(len <= 0 || y < s->clipY || y >= s->clipY + s->clipH)
    {
        return;
    }
    if(x < s->clipX)
    {
        len -= (s->clipX - x);
        x = s->clipX;
    }
    if(x + len > s->clipX + s->clipW)
    {
        len = (s->clipX + s->clipW) - x;
    }
    if(len <= 0)
    {
        return;
    }

    if(s->buf != NULL)
    {
        UInt16* p = &s->buf[(y * PANEL_MAX_W) + x];
        for(Int32 i = 0; i < len; ++i)
        {
            p[i] = colour;
        }
        if(y < s->dirtyTop)
        {
            s->dirtyTop = y;
        }
        if(y > s->dirtyBot)
        {
            s->dirtyBot = y;
        }
    }
    else
    {
        tftRect(s, x, y, len, 1, colour);
    }
}

static inline Void gfxPixel(Screen* s, Int32 x, Int32 y, UInt16 colour)
{
    gfxSpan(s, x, y, 1, colour);
}

/* Reads a pixel back. Only possible with the buffer - the panel itself cannot
 * be read - so this returns black without one rather than lying. */
static inline UInt16 gfxPeek(const Screen* s, Int32 x, Int32 y)
{
    if(s->buf == NULL || x < 0 || y < 0 || x >= s->width || y >= s->height)
    {
        return GFX_BLACK;
    }
    return s->buf[(y * PANEL_MAX_W) + x];
}

/* Alpha, which needs to read what is already there and so needs the buffer. */
static inline Void gfxPixelBlend(Screen* s, Int32 x, Int32 y, UInt16 colour, UInt8 alpha)
{
    gfxPixel(s, x, y, gfxBlend(gfxPeek(s, x, y), colour, alpha));
}

/* ---- present ------------------------------------------------------------- */

static inline Void gfxPresent(Screen* s)
{
    if(s->buf == NULL || s->dirtyBot < s->dirtyTop)
    {
        return;                     /* nothing changed; do not touch the panel */
    }

    const Int32 y = s->dirtyTop;
    const Int32 h = (s->dirtyBot - s->dirtyTop) + 1;
    const Int32 w = s->width;

    /* The buffer holds native-endian UInt16 and the panel wants big-endian, so
     * this cannot be one memcpy of the frame - the bytes go out a row at a time
     * through a swap. Still one address window and ONE transaction for the lot,
     * which is where nearly all of the win is. */
    UInt8 row[PANEL_MAX_W * 2];
    tftBeginPixels(s, 0, y, w, h);
    for(Int32 r = 0; r < h; ++r)
    {
        const UInt16* src = &s->buf[((y + r) * PANEL_MAX_W)];
        for(Int32 i = 0; i < w; ++i)
        {
            row[i * 2]     = (UInt8) (src[i] >> 8);
            row[i * 2 + 1] = (UInt8) (src[i] & 0xFF);
        }
        spiWrite(s->sck, row, (Size) (w * 2));
    }
    tftEndPixels(s);

    s->dirtyTop = s->height;
    s->dirtyBot = -1;
}

static Void gfxClear(Screen* s, UInt16 colour)
{
    for(Int32 y = 0; y < s->height; ++y)
    {
        gfxSpan(s, 0, y, s->width, colour);
    }
}

/* ---- rectangles ---------------------------------------------------------- */

static Void gfxRectFill(Screen* s, Int32 x, Int32 y, Int32 w, Int32 h, UInt16 colour)
{
    for(Int32 r = 0; r < h; ++r)
    {
        gfxSpan(s, x, y + r, w, colour);
    }
}

static Void gfxRect(Screen* s, Int32 x, Int32 y, Int32 w, Int32 h, UInt16 colour)
{
    if(w <= 0 || h <= 0)
    {
        return;
    }
    gfxSpan(s, x, y, w, colour);
    gfxSpan(s, x, y + h - 1, w, colour);
    for(Int32 r = 1; r < h - 1; ++r)
    {
        gfxPixel(s, x, y + r, colour);
        gfxPixel(s, x + w - 1, y + r, colour);
    }
}

/* ---- lines --------------------------------------------------------------- */

static Void gfxHLine(Screen* s, Int32 x, Int32 y, Int32 w, UInt16 colour)
{
    gfxSpan(s, x, y, w, colour);
}

static Void gfxVLine(Screen* s, Int32 x, Int32 y, Int32 h, UInt16 colour)
{
    for(Int32 i = 0; i < h; ++i)
    {
        gfxPixel(s, x, y + i, colour);
    }
}

/*
 * Bresenham. Integer only - no division and no floating point in the loop -
 * which is why it has survived since 1962 and is still right on a
 * microcontroller.
 */
static Void gfxLine(Screen* s, Int32 x0, Int32 y0, Int32 x1, Int32 y1, UInt16 colour)
{
    const Int32 dx = x1 > x0 ? x1 - x0 : x0 - x1;
    const Int32 dy = y1 > y0 ? y1 - y0 : y0 - y1;
    const Int32 sx = x0 < x1 ? 1 : -1;
    const Int32 sy = y0 < y1 ? 1 : -1;

    /* The axis-aligned cases are common enough - borders, grids, axes - to be
     * worth the span path instead of stepping pixel by pixel. */
    if(dy == 0)
    {
        gfxSpan(s, (x0 < x1) ? x0 : x1, y0, dx + 1, colour);
        return;
    }
    if(dx == 0)
    {
        gfxVLine(s, x0, (y0 < y1) ? y0 : y1, dy + 1, colour);
        return;
    }

    Int32 err = dx - dy;
    Int32 x   = x0;
    Int32 y   = y0;

    while(true)
    {
        gfxPixel(s, x, y, colour);
        if(x == x1 && y == y1)
        {
            break;
        }
        const Int32 e2 = err * 2;
        if(e2 > -dy)
        {
            err -= dy;
            x   += sx;
        }
        if(e2 < dx)
        {
            err += dx;
            y   += sy;
        }
    }
}

/* ---- circles ------------------------------------------------------------- */

static Void gfxCircle(Screen* s, Int32 cx, Int32 cy, Int32 r, UInt16 colour)
{
    if(r < 0)
    {
        return;
    }
    Int32 x = 0;
    Int32 y = r;
    Int32 d = 1 - r;
    while(x <= y)
    {
        gfxPixel(s, cx + x, cy + y, colour);
        gfxPixel(s, cx - x, cy + y, colour);
        gfxPixel(s, cx + x, cy - y, colour);
        gfxPixel(s, cx - x, cy - y, colour);
        gfxPixel(s, cx + y, cy + x, colour);
        gfxPixel(s, cx - y, cy + x, colour);
        gfxPixel(s, cx + y, cy - x, colour);
        gfxPixel(s, cx - y, cy - x, colour);
        ++x;
        if(d < 0)
        {
            d += (2 * x) + 1;
        }
        else
        {
            --y;
            d += (2 * (x - y)) + 1;
        }
    }
}

static Void gfxCircleFill(Screen* s, Int32 cx, Int32 cy, Int32 r, UInt16 colour)
{
    if(r < 0)
    {
        return;
    }
    Int32 x = 0;
    Int32 y = r;
    Int32 d = 1 - r;
    while(x <= y)
    {
        gfxSpan(s, cx - x, cy + y, (2 * x) + 1, colour);
        gfxSpan(s, cx - x, cy - y, (2 * x) + 1, colour);
        gfxSpan(s, cx - y, cy + x, (2 * y) + 1, colour);
        gfxSpan(s, cx - y, cy - x, (2 * y) + 1, colour);
        ++x;
        if(d < 0)
        {
            d += (2 * x) + 1;
        }
        else
        {
            --y;
            d += (2 * (x - y)) + 1;
        }
    }
}

/* ---- rounded rectangles --------------------------------------------------
 *
 * Filled as three bands plus four corner discs. A disc of radius r centred r in
 * from each edge cannot reach past it, so the overdraw is free of side effects
 * and the code stays short.
 */
static Void gfxRoundRectFill(Screen* s, Int32 x, Int32 y, Int32 w, Int32 h, Int32 r, UInt16 colour)
{
    if(w <= 0 || h <= 0)
    {
        return;
    }
    const Int32 maxR = ((w < h) ? w : h) / 2;
    if(r > maxR)
    {
        r = maxR;
    }
    if(r <= 0)
    {
        gfxRectFill(s, x, y, w, h, colour);
        return;
    }

    gfxRectFill(s, x + r, y, w - (2 * r), h, colour);
    gfxRectFill(s, x, y + r, r, h - (2 * r), colour);
    gfxRectFill(s, x + w - r, y + r, r, h - (2 * r), colour);

    gfxCircleFill(s, x + r,         y + r,         r, colour);
    gfxCircleFill(s, x + w - r - 1, y + r,         r, colour);
    gfxCircleFill(s, x + r,         y + h - r - 1, r, colour);
    gfxCircleFill(s, x + w - r - 1, y + h - r - 1, r, colour);
}

static Void gfxRoundRect(Screen* s, Int32 x, Int32 y, Int32 w, Int32 h, Int32 r, UInt16 colour)
{
    if(w <= 0 || h <= 0)
    {
        return;
    }
    const Int32 maxR = ((w < h) ? w : h) / 2;
    if(r > maxR)
    {
        r = maxR;
    }
    if(r <= 0)
    {
        gfxRect(s, x, y, w, h, colour);
        return;
    }

    gfxSpan(s, x + r, y, w - (2 * r), colour);
    gfxSpan(s, x + r, y + h - 1, w - (2 * r), colour);
    gfxVLine(s, x, y + r, h - (2 * r), colour);
    gfxVLine(s, x + w - 1, y + r, h - (2 * r), colour);

    Int32 cx = 0;
    Int32 cy = r;
    Int32 d  = 1 - r;
    while(cx <= cy)
    {
        gfxPixel(s, x + w - r - 1 + cx, y + h - r - 1 + cy, colour);
        gfxPixel(s, x + r - cx,         y + h - r - 1 + cy, colour);
        gfxPixel(s, x + w - r - 1 + cy, y + h - r - 1 + cx, colour);
        gfxPixel(s, x + r - cy,         y + h - r - 1 + cx, colour);

        gfxPixel(s, x + w - r - 1 + cx, y + r - cy, colour);
        gfxPixel(s, x + r - cx,         y + r - cy, colour);
        gfxPixel(s, x + w - r - 1 + cy, y + r - cx, colour);
        gfxPixel(s, x + r - cy,         y + r - cx, colour);

        ++cx;
        if(d < 0)
        {
            d += (2 * cx) + 1;
        }
        else
        {
            --cy;
            d += (2 * (cx - cy)) + 1;
        }
    }
}

/* ---- triangles ----------------------------------------------------------- */

static Void gfxTriangle(Screen* s, Int32 x0, Int32 y0, Int32 x1, Int32 y1, Int32 x2, Int32 y2, UInt16 colour)
{
    gfxLine(s, x0, y0, x1, y1, colour);
    gfxLine(s, x1, y1, x2, y2, colour);
    gfxLine(s, x2, y2, x0, y0, colour);
}

/*
 * Scanline fill. Vertices sorted by y, then the triangle walked as two halves
 * that share the middle vertex, filling a span between the active edges.
 */
static Void gfxTriangleFill(Screen* s, Int32 x0, Int32 y0, Int32 x1, Int32 y1, Int32 x2, Int32 y2, UInt16 colour)
{
    Int32 tx = 0;
    Int32 ty = 0;

    /* Three compare-and-swaps is a full sort for three items, and the middle
     * one is what splits the triangle into its two scanline halves. */
    if(y0 > y1)
    {
        tx = x0; x0 = x1; x1 = tx;
        ty = y0; y0 = y1; y1 = ty;
    }
    if(y1 > y2)
    {
        tx = x1; x1 = x2; x2 = tx;
        ty = y1; y1 = y2; y2 = ty;
    }
    if(y0 > y1)
    {
        tx = x0; x0 = x1; x1 = tx;
        ty = y0; y0 = y1; y1 = ty;
    }

    if(y0 == y2)
    {
        /* Degenerate: all three on one row. Draw the extent and stop, rather
         * than dividing by a zero height below. */
        Int32 lo = x0;
        Int32 hi = x0;
        if(x1 < lo)
        {
            lo = x1;
        }
        if(x1 > hi)
        {
            hi = x1;
        }
        if(x2 < lo)
        {
            lo = x2;
        }
        if(x2 > hi)
        {
            hi = x2;
        }
        gfxSpan(s, lo, y0, (hi - lo) + 1, colour);
        return;
    }

    for(Int32 y = y0; y <= y2; ++y)
    {
        const Bool second = (y > y1);

        const Int32 aY0 = y0;
        const Int32 aY1 = y2;
        const Int32 aX0 = x0;
        const Int32 aX1 = x2;

        const Int32 bY0 = second ? y1 : y0;
        const Int32 bY1 = second ? y2 : y1;
        const Int32 bX0 = second ? x1 : x0;
        const Int32 bX1 = second ? x2 : x1;

        const Int32 aDen = (aY1 - aY0);
        const Int32 bDen = (bY1 - bY0);

        const Int32 ax = aX0 + (((aX1 - aX0) * (y - aY0)) / ((aDen == 0) ? 1 : aDen));
        const Int32 bx = (bDen == 0) ? bX1
                                     : (bX0 + (((bX1 - bX0) * (y - bY0)) / bDen));

        const Int32 lo = (ax < bx) ? ax : bx;
        const Int32 hi = (ax < bx) ? bx : ax;
        gfxSpan(s, lo, y, (hi - lo) + 1, colour);
    }
}

/* ---- text ----------------------------------------------------------------
 *
 * Set the colour and size on the screen once, then draw. Two ways to place it:
 * gfxTextAt() for an absolute position, or a cursor with gfxPrint() for a
 * readout that flows down the screen.
 */

static inline Void gfxTextColour(Screen* s, UInt16 fg)
{
    s->fg = fg;
}

/* An opaque background, which is what you want for a value that changes: the
 * new text erases the old as it draws, with no flicker and no clear step. */
static inline Void gfxTextBackground(Screen* s, UInt16 bg)
{
    s->bg      = bg;
    s->bgSolid = true;
}

/* Leave whatever is behind the glyph alone - for text over a picture. */
static inline Void gfxTextTransparent(Screen* s)
{
    s->bgSolid = false;
}

static inline Void gfxTextSize(Screen* s, Int32 scale)
{
    s->textScale = (scale < 1) ? 1 : ((scale > 8) ? 8 : scale);
}

static inline Void gfxCursor(Screen* s, Int32 x, Int32 y)
{
    s->cursorX = x;
    s->cursorY = y;
}

static inline Int32 gfxTextHeight(const Screen* s)
{
    return 8 * s->textScale;
}

static inline Int32 gfxCharWidth(const Screen* s)
{
    return 6 * s->textScale;
}

static inline Int32 gfxTextWidth(const Screen* s, const Utf8* str)
{
    Int32 n = 0;
    while(str != NULL && str[n] != '\0')
    {
        ++n;
    }
    return n * gfxCharWidth(s);
}

static inline Void gfxCharAt(Screen* s, Int32 x, Int32 y, Utf8 ch)
{
    Utf8 c = ch;
    if(c >= 'a' && c <= 'z')
    {
        c = (Utf8) (c - 'a' + 'A');
    }
    if(c < 32 || c > 90)
    {
        c = '?';
    }

    const UInt8* const glyph = FONT5X7[c - 32];

    for(Int32 col = 0; col < 6; ++col)
    {
        const UInt8 bits = (col < 5) ? glyph[col] : 0x00;

        for(Int32 row = 0; row < 8; ++row)
        {
            const Bool on = (row < 7) && (((bits >> row) & 1u) != 0u);
            if(!on && !s->bgSolid)
            {
                continue;                   /* see through to what is behind */
            }
            gfxRectFill(s, x + (col * s->textScale), y + (row * s->textScale),
                        s->textScale, s->textScale, on ? s->fg : s->bg);
        }
    }
}

static inline Void gfxTextAt(Screen* s, Int32 x, Int32 y, const Utf8* str)
{
    Int32 cx = x;
    while(str != NULL && *str != '\0')
    {
        gfxCharAt(s, cx, y, *str);
        cx += gfxCharWidth(s);
        ++str;
    }
}

typedef enum GfxAlign
{
    GFX_ALIGN_LEFT = 0,
    GFX_ALIGN_CENTRE,
    GFX_ALIGN_RIGHT
} GfxAlign;

/* `x` is the left edge, the centre or the right edge depending on `align` -
 * which is what makes a value that changes width stay put. */
static inline Void gfxTextAligned(Screen* s, Int32 x, Int32 y, const Utf8* str, GfxAlign align)
{
    const Int32 w  = gfxTextWidth(s, str);
    Int32       at = x;
    if(align == GFX_ALIGN_CENTRE)
    {
        at = x - (w / 2);
    }
    else if(align == GFX_ALIGN_RIGHT)
    {
        at = x - w;
    }
    gfxTextAt(s, at, y, str);
}

/* Draws at the cursor and advances it, so consecutive calls flow. */
static inline Void gfxPrint(Screen* s, const Utf8* str)
{
    gfxTextAt(s, s->cursorX, s->cursorY, str);
    s->cursorX += gfxTextWidth(s, str);
}

static inline Void gfxPrintLine(Screen* s, const Utf8* str)
{
    gfxTextAt(s, s->cursorX, s->cursorY, str);
    s->cursorY += gfxTextHeight(s) + (2 * s->textScale);
}

/* printf into the cursor. The buffer is deliberately small: this is a 240 pixel
 * screen and forty characters already overflow it at size 1. */
static inline Void gfxPrintf(Screen* s, const Utf8* fmt, ...)
{
    Utf8    buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    gfxPrint(s, buf);
}

/*
 * Draws the safe area's boundary. A calibration aid, not decoration: run it,
 * look at the panel, and change the inset until the frame is fully visible with
 * a little to spare. Then take the call out.
 */
static inline Void gfxSafeOutline(Screen* s, UInt16 colour)
{
    gfxRect(s, gfxSafeLeft(s), gfxSafeTop(s),
            gfxSafeWidth(s), gfxSafeHeight(s), colour);
}

/* ---- start --------------------------------------------------------------- */

/*
 * Brings up the panel AND attaches the back buffer.
 *
 * Returns what tftOpen returns, which can only tell you the SPI pins were valid
 * - see the note in st77xx.h about the panel being write-only.
 */
static inline Bool gfxOpen(Screen* s, Int32 w, Int32 h, Int32 xoff, Int32 yoff)
{
    if(!tftOpen(s, w, h, xoff, yoff))
    {
        return false;
    }

    /* One buffer, so the FIRST screen to ask gets it. A second screen still
     * works and simply draws straight at its panel - which is the honest
     * outcome, and better than two screens quietly sharing one buffer. */
    if(!gfxBufTaken)
    {
        gfxBufTaken = true;
        s->buf      = gfxBuf;
    }

    gfxClipReset(s);
    gfxClear(s, GFX_BLACK);
    gfxPresent(s);
    return true;
}

#endif
