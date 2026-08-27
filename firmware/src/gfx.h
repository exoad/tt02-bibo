/*
 * A 2D drawing API over the panel: shapes, text, colour and clipping.
 *
 * st77xx.h is the DRIVER - it knows about chip selects, command bytes and
 * address windows. This is the part you actually draw with, and it is meant to
 * feel like every other 2D canvas you have used:
 *
 *     gfxInit();
 *
 *     while(true)
 *     {
 *         gfxClear(GFX_NAVY);
 *         gfxRectFill(10, 10, 100, 40, GFX_ORANGE);
 *         gfxCircleFill(120, 120, 30, GFX_CYAN);
 *
 *         gfxTextColour(GFX_WHITE);
 *         gfxTextSize(2);
 *         gfxTextAt(10, 60, "HELLO");
 *
 *         gfxPresent();
 *     }
 *
 * ---------------------------------------------------------------------------
 * THE BACK BUFFER, AND WHY IT IS WORTH 112 KB
 *
 * By default every call above draws into RAM, and gfxPresent() sends the whole
 * thing to the panel in one burst. 240 x 240 x 16bpp is 115,200 bytes - 22% of
 * the RP2350's 520 KB - and it buys three things that matter:
 *
 *   NO FLICKER. Without a buffer you are drawing on the glass. Clearing the
 *   screen and redrawing means the viewer SEES the clear, every frame, as a
 *   flash. With one, they only ever see finished frames.
 *
 *   OVERDRAW IS FREE. You can draw a background, then a shape on top of it,
 *   then text on top of that. Straight to the panel each of those is a separate
 *   round trip and the intermediate states are visible.
 *
 *   IT IS FAST. One address window and one 115 KB burst, rather than a window
 *   setup per rectangle. At 24 MHz a full frame is 38 ms - about 26 fps - and
 *   gfxPresent only pushes the ROWS THAT CHANGED, so a ticking clock in the
 *   corner costs a fraction of that.
 *
 * If you need the RAM back, set GFX_BUFFERED to 0. Every function below still
 * works and draws straight to the panel; you lose the three things above and
 * gain 112 KB. The API does not change, which is the point.
 *
 * ---------------------------------------------------------------------------
 * COORDINATES
 *
 * x right, y DOWN, origin top-left, like every other screen API and unlike
 * school maths. (0, 0) is the top-left pixel; (PANEL_W - 1, PANEL_H - 1) is the
 * bottom-right.
 *
 * Everything is clipped. Drawing off the edge is not an error and never
 * corrupts the opposite side - it just does not appear. That is what makes
 * scrolling and animation writable without a bounds check at every call site.
 */
#ifndef TT02_GFX_H
#define TT02_GFX_H

#include "st77xx.h"

#include <stdarg.h>
#include <stdio.h>

/* 1 = draw into RAM and push on gfxPresent(). 0 = draw straight to the panel. */
#define GFX_BUFFERED 1

/* ---- colour --------------------------------------------------------------
 *
 * 16-bit 5-6-5, the panel's own format. Green gets the spare bit because the
 * eye resolves more detail in green than in red or blue.
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
 * Done per channel after unpacking, because 5-6-5 cannot be averaged as a
 * single integer - the channels would carry into each other and a half-way
 * blend of red and blue would come out an unrelated colour.
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

/* Toward black. `amount` of 255 is black, 0 leaves it alone. */
static inline UInt16 gfxDim(UInt16 c, UInt8 amount)
{
    return gfxBlend(c, GFX_BLACK, amount);
}

/* Toward white. */
static inline UInt16 gfxLighten(UInt16 c, UInt8 amount)
{
    return gfxBlend(c, GFX_WHITE, amount);
}

/*
 * Hue 0-359, saturation and value 0-255. Integer throughout - no float, no
 * table - so a rainbow sweep costs nothing on a chip that would rather not.
 */
static inline UInt16 gfxHsv(Int32 hue, UInt8 sat, UInt8 val)
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

/* ---- state ---------------------------------------------------------------
 *
 * Module-level, like a classic canvas context: you set a colour and a size and
 * then draw, rather than passing eight arguments to every call. There is one
 * screen, so there is one context.
 */

#if GFX_BUFFERED
static UInt16 gfxBuf[PANEL_W * PANEL_H];

/* Rows touched since the last present. Pushing only these is what makes a small
 * update cheap: a clock ticking in the corner sends twenty rows, not all 240. */
static Int32 gfxDirtyTop = PANEL_H;
static Int32 gfxDirtyBot = -1;
#endif

static Int32 gfxClipX = 0;
static Int32 gfxClipY = 0;
static Int32 gfxClipW = PANEL_W;
static Int32 gfxClipH = PANEL_H;

static UInt16 gfxFg       = GFX_WHITE;
static UInt16 gfxBg       = GFX_BLACK;
static Bool   gfxBgSolid  = true;
static Int32  gfxScale    = 1;
static Int32  gfxCurX     = 0;
static Int32  gfxCurY     = 0;

/* ---- clipping ------------------------------------------------------------ */

/* Restricts drawing to a rectangle. Intersected with the screen, so a clip
 * larger than the panel is harmless rather than a way to write out of bounds. */
static inline Void gfxClip(Int32 x, Int32 y, Int32 w, Int32 h)
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
    if(x + w > PANEL_W)
    {
        w = PANEL_W - x;
    }
    if(y + h > PANEL_H)
    {
        h = PANEL_H - y;
    }
    if(w < 0)
    {
        w = 0;
    }
    if(h < 0)
    {
        h = 0;
    }

    gfxClipX = x;
    gfxClipY = y;
    gfxClipW = w;
    gfxClipH = h;
}

static inline Void gfxClipReset(Void)
{
    gfxClip(0, 0, PANEL_W, PANEL_H);
}

/* ---- the two primitives everything else is built from --------------------
 *
 * A horizontal run is the fast path in both modes: a memory fill when buffered,
 * and ONE address window when not. Every shape below is expressed as runs for
 * that reason - a filled circle drawn as pixels is a thousand SPI transactions,
 * and drawn as spans it is thirty.
 */

static inline Void gfxSpan(Int32 x, Int32 y, Int32 len, UInt16 colour)
{
    if(len <= 0 || y < gfxClipY || y >= gfxClipY + gfxClipH)
    {
        return;
    }
    if(x < gfxClipX)
    {
        len -= (gfxClipX - x);
        x = gfxClipX;
    }
    if(x + len > gfxClipX + gfxClipW)
    {
        len = (gfxClipX + gfxClipW) - x;
    }
    if(len <= 0)
    {
        return;
    }

#if GFX_BUFFERED
    UInt16* p = &gfxBuf[(y * PANEL_W) + x];
    for(Int32 i = 0; i < len; ++i)
    {
        p[i] = colour;
    }
    if(y < gfxDirtyTop)
    {
        gfxDirtyTop = y;
    }
    if(y > gfxDirtyBot)
    {
        gfxDirtyBot = y;
    }
#else
    tftRect(x, y, len, 1, colour);
#endif
}

static inline Void gfxPixel(Int32 x, Int32 y, UInt16 colour)
{
    gfxSpan(x, y, 1, colour);
}

/* Reads a pixel back. Only possible with the buffer - the panel itself cannot
 * be read - so this returns black in unbuffered mode rather than lying. */
static inline UInt16 gfxPeek(Int32 x, Int32 y)
{
#if GFX_BUFFERED
    if(x < 0 || y < 0 || x >= PANEL_W || y >= PANEL_H)
    {
        return GFX_BLACK;
    }
    return gfxBuf[(y * PANEL_W) + x];
#else
    (Void) x;
    (Void) y;
    return GFX_BLACK;
#endif
}

/* Alpha, which needs to read what is already there and so needs the buffer. */
static inline Void gfxPixelBlend(Int32 x, Int32 y, UInt16 colour, UInt8 alpha)
{
    gfxPixel(x, y, gfxBlend(gfxPeek(x, y), colour, alpha));
}

/* ---- present ------------------------------------------------------------- */

static inline Void gfxPresent(Void)
{
#if GFX_BUFFERED
    if(gfxDirtyBot < gfxDirtyTop)
    {
        return;                     /* nothing changed; do not touch the panel */
    }

    const Int32 y = gfxDirtyTop;
    const Int32 h = (gfxDirtyBot - gfxDirtyTop) + 1;

    /* The buffer holds native-endian UInt16 and the panel wants big-endian, so
     * this cannot be one memcpy of the whole frame - the bytes go out a row at
     * a time through a swap. Still one address window and ONE transaction for
     * the lot, which is where nearly all of the win is. */
    UInt8 row[PANEL_W * 2];
    tftBeginPixels(0, y, PANEL_W, h);
    for(Int32 r = 0; r < h; ++r)
    {
        const UInt16* src = &gfxBuf[((y + r) * PANEL_W)];
        for(Int32 i = 0; i < PANEL_W; ++i)
        {
            row[i * 2]     = (UInt8) (src[i] >> 8);
            row[i * 2 + 1] = (UInt8) (src[i] & 0xFF);
        }
        spiWrite(PIN_TFT_SCK, row, sizeof(row));
    }
    tftEndPixels();

    gfxDirtyTop = PANEL_H;
    gfxDirtyBot = -1;
#endif
}

static inline Void gfxClear(UInt16 colour)
{
    for(Int32 y = 0; y < PANEL_H; ++y)
    {
        gfxSpan(0, y, PANEL_W, colour);
    }
}

/* ---- rectangles ---------------------------------------------------------- */

static inline Void gfxRectFill(Int32 x, Int32 y, Int32 w, Int32 h, UInt16 colour)
{
    for(Int32 r = 0; r < h; ++r)
    {
        gfxSpan(x, y + r, w, colour);
    }
}

static inline Void gfxRect(Int32 x, Int32 y, Int32 w, Int32 h, UInt16 colour)
{
    if(w <= 0 || h <= 0)
    {
        return;
    }
    gfxSpan(x, y, w, colour);
    gfxSpan(x, y + h - 1, w, colour);
    for(Int32 r = 1; r < h - 1; ++r)
    {
        gfxPixel(x, y + r, colour);
        gfxPixel(x + w - 1, y + r, colour);
    }
}

/* ---- lines --------------------------------------------------------------- */

static inline Void gfxHLine(Int32 x, Int32 y, Int32 w, UInt16 colour)
{
    gfxSpan(x, y, w, colour);
}

static inline Void gfxVLine(Int32 x, Int32 y, Int32 h, UInt16 colour)
{
    for(Int32 i = 0; i < h; ++i)
    {
        gfxPixel(x, y + i, colour);
    }
}

/*
 * Bresenham. Integer only - no division and no floating point in the loop -
 * which is why it has survived since 1962 and why it is still the right choice
 * on a microcontroller.
 */
static inline Void gfxLine(Int32 x0, Int32 y0, Int32 x1, Int32 y1, UInt16 colour)
{
    const Int32 dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    const Int32 dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    const Int32 sx = (x0 < x1) ? 1 : -1;
    const Int32 sy = (y0 < y1) ? 1 : -1;

    /* The axis-aligned cases are common enough - borders, grids, axes - to be
     * worth taking the span path instead of stepping pixel by pixel. */
    if(dy == 0)
    {
        gfxSpan((x0 < x1) ? x0 : x1, y0, dx + 1, colour);
        return;
    }
    if(dx == 0)
    {
        gfxVLine(x0, (y0 < y1) ? y0 : y1, dy + 1, colour);
        return;
    }

    Int32 err = dx - dy;
    Int32 x   = x0;
    Int32 y   = y0;

    while(true)
    {
        gfxPixel(x, y, colour);
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

/*
 * Midpoint circle: one decision variable, integer, eight-way symmetry. The
 * filled version draws the same octants as SPANS rather than points, which is
 * the difference between one transaction per row and one per pixel.
 */
static inline Void gfxCircle(Int32 cx, Int32 cy, Int32 r, UInt16 colour)
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
        gfxPixel(cx + x, cy + y, colour);
        gfxPixel(cx - x, cy + y, colour);
        gfxPixel(cx + x, cy - y, colour);
        gfxPixel(cx - x, cy - y, colour);
        gfxPixel(cx + y, cy + x, colour);
        gfxPixel(cx - y, cy + x, colour);
        gfxPixel(cx + y, cy - x, colour);
        gfxPixel(cx - y, cy - x, colour);

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

static inline Void gfxCircleFill(Int32 cx, Int32 cy, Int32 r, UInt16 colour)
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
        gfxSpan(cx - x, cy + y, (2 * x) + 1, colour);
        gfxSpan(cx - x, cy - y, (2 * x) + 1, colour);
        gfxSpan(cx - y, cy + x, (2 * y) + 1, colour);
        gfxSpan(cx - y, cy - x, (2 * y) + 1, colour);

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
 * Filled as three bands plus four corner discs. The discs sit entirely inside
 * the rectangle - a disc of radius r centred r in from each edge cannot reach
 * past it - so the overdraw is free of side effects and the code stays short.
 */
static inline Void gfxRoundRectFill(Int32 x, Int32 y, Int32 w, Int32 h, Int32 r,
                                    UInt16 colour)
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
        gfxRectFill(x, y, w, h, colour);
        return;
    }

    gfxRectFill(x + r, y, w - (2 * r), h, colour);
    gfxRectFill(x, y + r, r, h - (2 * r), colour);
    gfxRectFill(x + w - r, y + r, r, h - (2 * r), colour);

    gfxCircleFill(x + r,         y + r,         r, colour);
    gfxCircleFill(x + w - r - 1, y + r,         r, colour);
    gfxCircleFill(x + r,         y + h - r - 1, r, colour);
    gfxCircleFill(x + w - r - 1, y + h - r - 1, r, colour);
}

static inline Void gfxRoundRect(Int32 x, Int32 y, Int32 w, Int32 h, Int32 r,
                                UInt16 colour)
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
        gfxRect(x, y, w, h, colour);
        return;
    }

    gfxSpan(x + r, y, w - (2 * r), colour);
    gfxSpan(x + r, y + h - 1, w - (2 * r), colour);
    gfxVLine(x, y + r, h - (2 * r), colour);
    gfxVLine(x + w - 1, y + r, h - (2 * r), colour);

    Int32 cx = 0;
    Int32 cy = r;
    Int32 d  = 1 - r;
    while(cx <= cy)
    {
        gfxPixel(x + w - r - 1 + cx, y + h - r - 1 + cy, colour);
        gfxPixel(x + r - cx,         y + h - r - 1 + cy, colour);
        gfxPixel(x + w - r - 1 + cy, y + h - r - 1 + cx, colour);
        gfxPixel(x + r - cy,         y + h - r - 1 + cx, colour);

        gfxPixel(x + w - r - 1 + cx, y + r - cy, colour);
        gfxPixel(x + r - cx,         y + r - cy, colour);
        gfxPixel(x + w - r - 1 + cy, y + r - cx, colour);
        gfxPixel(x + r - cy,         y + r - cx, colour);

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

static inline Void gfxTriangle(Int32 x0, Int32 y0, Int32 x1, Int32 y1,
                               Int32 x2, Int32 y2, UInt16 colour)
{
    gfxLine(x0, y0, x1, y1, colour);
    gfxLine(x1, y1, x2, y2, colour);
    gfxLine(x2, y2, x0, y0, colour);
}

/*
 * Scanline fill. Vertices are sorted by y, then the triangle is walked as two
 * halves that share the middle vertex, filling a span between the two active
 * edges on each row.
 */
static inline Void gfxTriangleFill(Int32 x0, Int32 y0, Int32 x1, Int32 y1,
                                   Int32 x2, Int32 y2, UInt16 colour)
{
    Int32 tx = 0;
    Int32 ty = 0;

    /* Sort the three vertices by y, so y0 is the top and y2 the bottom. Three
     * compare-and-swaps is a full sort for three items, and the middle one is
     * what splits the triangle into its two scanline halves. */
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
        gfxSpan(lo, y0, (hi - lo) + 1, colour);
        return;
    }

    for(Int32 y = y0; y <= y2; ++y)
    {
        const Bool second = (y > y1);

        /* The long edge runs the whole height; the short one is whichever half
         * this row is in. */
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
        gfxSpan(lo, y, (hi - lo) + 1, colour);
    }
}

/* ---- text ----------------------------------------------------------------
 *
 * A context, like a canvas: set the colour and size once, then draw. Two ways
 * to place it - gfxTextAt() for an absolute position, or a cursor with
 * gfxPrint() for a running readout that flows down the screen.
 */

static inline Void gfxTextColour(UInt16 fg)
{
    gfxFg = fg;
}

/* An opaque background, which is what you want for a value that changes: the
 * new text erases the old as it draws, with no flicker and no clear step. */
static inline Void gfxTextBackground(UInt16 bg)
{
    gfxBg      = bg;
    gfxBgSolid = true;
}

/* Leave whatever is behind the glyph alone - for text over a picture. */
static inline Void gfxTextTransparent(Void)
{
    gfxBgSolid = false;
}

static inline Void gfxTextSize(Int32 scale)
{
    gfxScale = (scale < 1) ? 1 : ((scale > 8) ? 8 : scale);
}

static inline Void gfxCursor(Int32 x, Int32 y)
{
    gfxCurX = x;
    gfxCurY = y;
}

static inline Int32 gfxTextHeight(Void)
{
    return 8 * gfxScale;
}

static inline Int32 gfxCharWidth(Void)
{
    return 6 * gfxScale;
}

static inline Int32 gfxTextWidth(const Utf8* s)
{
    Int32 n = 0;
    while(s != NULL && s[n] != '\0')
    {
        ++n;
    }
    return n * gfxCharWidth();
}

/* One glyph at an absolute position, honouring the current colours and size. */
static inline Void gfxCharAt(Int32 x, Int32 y, Utf8 ch)
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
            if(!on && !gfxBgSolid)
            {
                continue;                       /* see through to what is behind */
            }
            gfxRectFill(x + (col * gfxScale), y + (row * gfxScale),
                        gfxScale, gfxScale, on ? gfxFg : gfxBg);
        }
    }
}

static inline Void gfxTextAt(Int32 x, Int32 y, const Utf8* s)
{
    Int32 cx = x;
    while(s != NULL && *s != '\0')
    {
        gfxCharAt(cx, y, *s);
        cx += gfxCharWidth();
        ++s;
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
static inline Void gfxTextAligned(Int32 x, Int32 y, const Utf8* s, GfxAlign align)
{
    const Int32 w = gfxTextWidth(s);
    Int32       at = x;
    if(align == GFX_ALIGN_CENTRE)
    {
        at = x - (w / 2);
    }
    else if(align == GFX_ALIGN_RIGHT)
    {
        at = x - w;
    }
    gfxTextAt(at, y, s);
}

/* Draws at the cursor and advances it, so consecutive calls flow. */
static inline Void gfxPrint(const Utf8* s)
{
    gfxTextAt(gfxCurX, gfxCurY, s);
    gfxCurX += gfxTextWidth(s);
}

static inline Void gfxPrintLine(const Utf8* s)
{
    gfxTextAt(gfxCurX, gfxCurY, s);
    gfxCurY += gfxTextHeight() + (2 * gfxScale);
}

/* printf into the current cursor. The buffer is deliberately small: this is a
 * 240 pixel screen and forty characters already overflow it at size 1. */
static inline Void gfxPrintf(const Utf8* fmt, ...)
{
    Utf8    buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    gfxPrint(buf);
}

/* ---- start --------------------------------------------------------------- */

/*
 * Brings up the panel and clears it. Returns what tftInit() returns, which can
 * only tell you the SPI pins were valid - see the note in st77xx.h about the
 * panel being write-only.
 */
static inline Bool gfxInit(Void)
{
    if(!tftInit())
    {
        return false;
    }
    gfxClipReset();
    gfxClear(GFX_BLACK);
    gfxPresent();
    return true;
}

#endif
