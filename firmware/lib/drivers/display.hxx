/*
 * The ST7789 / ST7735 panel driver: pins, commands, and one tft::Screen.
 *
 * ---------------------------------------------------------------------------
 * HOW THE TWO LAYERS FIT TOGETHER
 *
 * This file OWNS the hardware and hands back a tft::Screen. gfx.h DRAWS into one.
 *
 *     tft::Screen screen;
 *     tft::open(&screen, 240, 280, 0, 20);      the panel is now yours
 *     tft::fill(&screen, TFT_YELLOW);
 *
 * Every call takes the screen it acts on, so nothing is implicit and the
 * panel's size, pins and state live in one visible place rather than in file
 * statics. The practical payoff is that a sketch reads top-down - what is
 * connected, then what it is - instead of inheriting it from a header you have
 * to go and open.
 *
 * gfx.h layers on the SAME tft::Screen rather than introducing a second type:
 *
 *     tft::Screen screen;
 *     gfx::open(&screen, 240, 280, 0, 20);      panel + back buffer + context
 *     gfx::rectFill(&screen, 10, 10, 100, 40, GFX_ORANGE);
 *     gfx::present(&screen);
 *
 * ---------------------------------------------------------------------------
 * WHICH PANEL HAVE I GOT?
 *
 * The boards labelled GND / VCC / SCL / SDA / RES / DC / CS / BLK ship with
 * either controller behind them and the silkscreen rarely says. They take the
 * same wiring and almost the same commands, so this drives both and you pick
 * with the one define below.
 *
 *   ST7789   240x240, 240x280, 240x320 or 172x320, and INVERTED at the glass
 *   ST7735   128x160 or 128x128, not inverted
 *
 * SIZE IS AN ARGUMENT, not a define. The controller always has 240x320 of RAM
 * whatever glass is on the front, so a shorter panel shows a WINDOW into that
 * RAM and the offset says where the window starts. A 240x280 begins 20 rows
 * down; get that wrong and the picture is shifted off the top by 20 pixels
 * while everything else looks convincing.
 *
 * The symptom table, because these are what actually happens:
 *
 *   backlight on, screen black      wrong controller, or DC and RES swapped
 *   only part of the screen painted wrong size passed to tft::open
 *   image shifted, a band unpainted wrong offset
 *   colours inverted                wrong PANEL_INVERT
 *   blue where yellow should be     red and blue swapped - MADCTL 0x00 -> 0x08
 *   noise, tearing, intermittent    SPI too fast - drop PANEL_HZ
 *
 * ---------------------------------------------------------------------------
 * WIRING as built on this desk
 *
 *   Display   Pico          why
 *   GND       GND
 *   VCC       3V3           these are 3.3 V parts; do not feed them 5 V
 *   SCL/SCK   GP18          SPI0 SCK - fixed by the silicon, not a free choice
 *   SDA/MOSI  GP19          SPI0 TX
 *   RES       GP20          plain GPIO
 *   DC        GP21          plain GPIO: low = command, high = pixel data
 *   CS        GP17          plain GPIO, driven by us rather than the hardware
 *   BLK       3V3           always on. Move to a GPIO for PWM brightness.
 *
 * There is no MISO. These panels are write-only in practice: nothing can be
 * read back, which is why there is no "read pixel" here and why every drawing
 * routine has to know what it drew.
 *
 * GP17/GP18/GP19 are the SPI pins docs/wiring.md reserves for the MicroSD card.
 * That is FINE and is the point of a bus - SCK and MOSI are shared and the two
 * devices are told apart by their separate CS lines.
 */
#pragma once

#include "../hal.hxx"

namespace bibo
{

namespace tft
{

/* ===== the one line to change ============================================= */
#define PANEL_ST7789 1
#define PANEL_ST7735 0
/* ========================================================================== */

#if PANEL_ST7789
  /* ST7789 panels are wired inverted at the glass, so "invert on" is what
   * produces correct colours. This looks backwards and is not. */
  #define PANEL_INVERT  1
#else
  #define PANEL_INVERT  0
#endif

/*
 * The CEILING, not the size. It bounds the one stack buffer below and gfx.h's
 * frame buffer, both of which must be sized when the program is compiled rather
 * than when it runs. 240x320 is the largest this controller drives.
 */
#define PANEL_MAX_W   240
#define PANEL_MAX_H   320

/* Default pins. tft::openOn() takes them explicitly if a board differs. */
#define PIN_TFT_SCK   18
#define PIN_TFT_MOSI  19
#define PIN_TFT_CS    17
#define PIN_TFT_DC    21
#define PIN_TFT_RES   20

/*
 * 24 MHz. Comfortably inside what these panels manage and slow enough that a
 * long jumper wire is not the reason it fails. Raise it once it works, never
 * before - a display that has never worked and one that is too fast look
 * identical.
 */
#define PANEL_HZ      24000000u

/* 16 bits per pixel, 5 red / 6 green / 5 blue, which is what COLMOD is set to
 * below. Green gets the spare bit because the eye resolves most detail there. */
#define TFT_RGB(r, g, b) \
    (static_cast<UInt16>((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

#define TFT_BLACK   TFT_RGB(0, 0, 0)
#define TFT_WHITE   TFT_RGB(255, 255, 255)
#define TFT_RED     TFT_RGB(255, 0, 0)
#define TFT_GREEN   TFT_RGB(0, 255, 0)
#define TFT_BLUE    TFT_RGB(0, 0, 255)
#define TFT_YELLOW  TFT_RGB(255, 255, 0)
#define TFT_CYAN    TFT_RGB(0, 255, 255)
#define TFT_MAGENTA TFT_RGB(255, 0, 255)
#define TFT_GREY    TFT_RGB(128, 128, 128)
#define TFT_ORANGE  TFT_RGB(255, 140, 0)

/* ---- the tft::Screen -----------------------------------------------------------
 *
 * Everything about one panel, in one place. Passed to every call in this file
 * and in gfx.h, which is what lets a sketch say what it has rather than inherit
 * it from a header.
 *
 * The gfx fields live here rather than in gfx.h because they belong to the
 * SCREEN, not to the library: a clip rectangle and a text colour are as much
 * part of "the thing being drawn on" as its width is. They sit inert until
 * gfx::open() claims them.
 */
typedef struct Screen
{
    /* geometry, in pixels */
    Int32 width;
    Int32 height;
    Int32 xoff;
    Int32 yoff;

    /* pins */
    Pin sck;
    Pin mosi;
    Pin cs;
    Pin dc;
    Pin res;

    /* ---- gfx.h's business ------------------------------------------------ */

    /* The back buffer, or NULL to draw straight at the panel. */
    UInt16* buf;

    /* Rows touched since the last present. Pushing only these is what makes a
     * small update cheap - a clock ticking in a corner sends twenty rows. */
    Int32 dirtyTop;
    Int32 dirtyBot;

    Int32 clipX;
    Int32 clipY;
    Int32 clipW;
    Int32 clipH;

    /* ---- the safe area --------------------------------------------------
     *
     * These panels have ROUNDED CORNERS cut into the glass. The controller
     * still addresses the full rectangle and will happily accept pixels for
     * the corners; you simply cannot see them, and text placed near an edge
     * disappears into the curve.
     *
     * `safeInset` is how far in from each edge is actually visible. It is a
     * property of the GLASS, not of the driver, so it lives on the screen
     * beside the size and offsets - the same category of fact.
     *
     * Zero means "the whole rectangle is visible", which is right for a panel
     * with square corners and wrong for every one of these.
     */
    Int32 safeInset;

    UInt16 fg;
    UInt16 bg;
    Bool   bgSolid;
    Int32  textScale;
    Int32  cursorX;
    Int32  cursorY;
} Screen;

static Int32 width(const Screen* s)
{
    return (s != NULL) ? s->width : 0;
}

static Int32 height(const Screen* s)
{
    return (s != NULL) ? s->height : 0;
}

/* ---- the wire ------------------------------------------------------------
 *
 * CS LOW means "this transaction is mine"; raising it ENDS the transaction.
 *
 * That is why these are bracketed rather than each call driving CS itself. A
 * command and its parameters are ONE transaction: raise CS between them and the
 * controller treats the parameters as a fresh transaction beginning with no
 * command, and throws them away.
 *
 * The failure that produces is deeply unhelpful, and cost a day. SLPOUT takes
 * no parameters, so the panel still wakes and the backlight still comes on -
 * but COLMOD, MADCTL, CASET, RASET and RAMWR all lose their data and nothing is
 * ever drawn. A lit blank screen and a dead wire look identical.
 */
static Void select(const Screen* s)
{
    gpio::write(s->cs, false);
}

static Void deselect(const Screen* s)
{
    gpio::write(s->cs, true);
}

/* A command and its parameters, as one transaction. `n` may be 0. */
static Void write(const Screen* s, UInt8 cmd, const UInt8* params, Size n)
{
    select(s);

    gpio::write(s->dc, false);              /* low = this byte is a command */
    spi::writeByte(s->sck, cmd);

    if(params != NULL && n > 0)
    {
        gpio::write(s->dc, true);           /* high = these are its parameters */
        spi::write(s->sck, params, n);
    }

    deselect(s);
}

static Void cmd(const Screen* s, UInt8 c)
{
    write(s, c, NULL, 0);
}

static Void cmd1(const Screen* s, UInt8 c, UInt8 p)
{
    write(s, c, &p, 1);
}

/*
 * The write window. Every pixel push is "set a rectangle, then stream pixels
 * into it", which is why there is no per-pixel addressing anywhere below - the
 * controller advances its own cursor and wraps at the right edge.
 */
static Void window(const Screen* s, Int32 x, Int32 y, Int32 w, Int32 h)
{
    const Int32 x0 = x + s->xoff;
    const Int32 y0 = y + s->yoff;
    const Int32 x1 = x0 + w - 1;
    const Int32 y1 = y0 + h - 1;

    UInt8 buf[4];

    buf[0] = static_cast<UInt8>(x0 >> 8);
    buf[1] = static_cast<UInt8>(x0 & 0xFF);
    buf[2] = static_cast<UInt8>(x1 >> 8);
    buf[3] = static_cast<UInt8>(x1 & 0xFF);
    write(s, 0x2A, buf, 4);            /* CASET - column address */

    buf[0] = static_cast<UInt8>(y0 >> 8);
    buf[1] = static_cast<UInt8>(y0 & 0xFF);
    buf[2] = static_cast<UInt8>(y1 >> 8);
    buf[3] = static_cast<UInt8>(y1 & 0xFF);
    write(s, 0x2B, buf, 4);            /* RASET - row address */
}

/*
 * Opens a pixel write: sets the window, issues RAMWR, and LEAVES CS low with DC
 * high so pixels can follow in the same transaction. Close with tft::endPixels().
 * RAMWR is exactly the command whose data must not be separated from it, since
 * its "parameters" are the whole image.
 */
static Void beginPixels(const Screen* s, Int32 x, Int32 y, Int32 w, Int32 h)
{
    window(s, x, y, w, h);

    select(s);
    gpio::write(s->dc, false);
    spi::writeByte(s->sck, 0x2C);           /* RAMWR */
    gpio::write(s->dc, true);
    /* CS stays LOW; the caller streams pixels now. */
}

static Void endPixels(const Screen* s)
{
    deselect(s);
}

/* ---- drawing -------------------------------------------------------------
 *
 * These go STRAIGHT to the panel and ignore any back buffer. They are the
 * driver's own drawing, for bring-up and for sketches that want nothing else.
 * gfx.h is the layer that buffers.
 */

/*
 * Filled rectangle, streamed from a small buffer rather than a framebuffer.
 * Clipped rather than trusted: an off-screen rectangle draws nothing instead of
 * wrapping around and corrupting the far edge.
 */
static Void rect(const Screen* s, Int32 x, Int32 y, Int32 w, Int32 h, UInt16 colour)
{
    if(w <= 0 || h <= 0)
    {
        return;
    }
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
    if(w <= 0 || h <= 0)
    {
        return;
    }

    UInt8       line[PANEL_MAX_W * 2];
    const UInt8 hi = static_cast<UInt8>(colour >> 8);
    const UInt8 lo = static_cast<UInt8>(colour & 0xFF);
    for(Int32 i = 0; i < w; ++i)
    {
        line[i * 2]     = hi;
        line[i * 2 + 1] = lo;
    }

    beginPixels(s, x, y, w, h);
    for(Int32 r = 0; r < h; ++r)
    {
        spi::write(s->sck, line, static_cast<Size>(w * 2));
    }
    endPixels(s);
}

static Void fill(const Screen* s, UInt16 colour)
{
    rect(s, 0, 0, s->width, s->height, colour);
}

static Void pixel(const Screen* s, Int32 x, Int32 y, UInt16 colour)
{
    rect(s, x, y, 1, 1, colour);
}

/* ---- text ----------------------------------------------------------------
 *
 * A 5x7 font, five bytes per glyph, one byte per column, bit 0 at the top.
 *
 * Covers space through Z: digits, capitals and the punctuation a status readout
 * needs. Lowercase folds to uppercase rather than shipping a second set of
 * glyphs - this is for labels on a 240 pixel screen, not for prose, and half a
 * font that is CORRECT beats a whole one that is guessed at.
 */
static const UInt8 FONT5X7[59][5] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00 },   /* 32 space */
    { 0x00, 0x00, 0x5F, 0x00, 0x00 },   /* !  */
    { 0x00, 0x07, 0x00, 0x07, 0x00 },   /* "  */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14 },   /* #  */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12 },   /* $  */
    { 0x23, 0x13, 0x08, 0x64, 0x62 },   /* %  */
    { 0x36, 0x49, 0x55, 0x22, 0x50 },   /* &  */
    { 0x00, 0x05, 0x03, 0x00, 0x00 },   /* '  */
    { 0x00, 0x1C, 0x22, 0x41, 0x00 },   /* (  */
    { 0x00, 0x41, 0x22, 0x1C, 0x00 },   /* )  */
    { 0x14, 0x08, 0x3E, 0x08, 0x14 },   /* *  */
    { 0x08, 0x08, 0x3E, 0x08, 0x08 },   /* +  */
    { 0x00, 0x50, 0x30, 0x00, 0x00 },   /* ,  */
    { 0x08, 0x08, 0x08, 0x08, 0x08 },   /* -  */
    { 0x00, 0x60, 0x60, 0x00, 0x00 },   /* .  */
    { 0x20, 0x10, 0x08, 0x04, 0x02 },   /* /  */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E },   /* 0  */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 },   /* 1  */
    { 0x42, 0x61, 0x51, 0x49, 0x46 },   /* 2  */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 },   /* 3  */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 },   /* 4  */
    { 0x27, 0x45, 0x45, 0x45, 0x39 },   /* 5  */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 },   /* 6  */
    { 0x01, 0x71, 0x09, 0x05, 0x03 },   /* 7  */
    { 0x36, 0x49, 0x49, 0x49, 0x36 },   /* 8  */
    { 0x06, 0x49, 0x49, 0x29, 0x1E },   /* 9  */
    { 0x00, 0x36, 0x36, 0x00, 0x00 },   /* :  */
    { 0x00, 0x56, 0x36, 0x00, 0x00 },   /* ;  */
    { 0x08, 0x14, 0x22, 0x41, 0x00 },   /* <  */
    { 0x14, 0x14, 0x14, 0x14, 0x14 },   /* =  */
    { 0x00, 0x41, 0x22, 0x14, 0x08 },   /* >  */
    { 0x02, 0x01, 0x51, 0x09, 0x06 },   /* ?  */
    { 0x32, 0x49, 0x79, 0x41, 0x3E },   /* @  */
    { 0x7E, 0x11, 0x11, 0x11, 0x7E },   /* A  */
    { 0x7F, 0x49, 0x49, 0x49, 0x36 },   /* B  */
    { 0x3E, 0x41, 0x41, 0x41, 0x22 },   /* C  */
    { 0x7F, 0x41, 0x41, 0x22, 0x1C },   /* D  */
    { 0x7F, 0x49, 0x49, 0x49, 0x41 },   /* E  */
    { 0x7F, 0x09, 0x09, 0x09, 0x01 },   /* F  */
    { 0x3E, 0x41, 0x49, 0x49, 0x7A },   /* G  */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F },   /* H  */
    { 0x00, 0x41, 0x7F, 0x41, 0x00 },   /* I  */
    { 0x20, 0x40, 0x41, 0x3F, 0x01 },   /* J  */
    { 0x7F, 0x08, 0x14, 0x22, 0x41 },   /* K  */
    { 0x7F, 0x40, 0x40, 0x40, 0x40 },   /* L  */
    { 0x7F, 0x02, 0x0C, 0x02, 0x7F },   /* M  */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F },   /* N  */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E },   /* O  */
    { 0x7F, 0x09, 0x09, 0x09, 0x06 },   /* P  */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E },   /* Q  */
    { 0x7F, 0x09, 0x19, 0x29, 0x46 },   /* R  */
    { 0x46, 0x49, 0x49, 0x49, 0x31 },   /* S  */
    { 0x01, 0x01, 0x7F, 0x01, 0x01 },   /* T  */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F },   /* U  */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F },   /* V  */
    { 0x3F, 0x40, 0x38, 0x40, 0x3F },   /* W  */
    { 0x63, 0x14, 0x08, 0x14, 0x63 },   /* X  */
    { 0x07, 0x08, 0x70, 0x08, 0x07 },   /* Y  */
    { 0x61, 0x51, 0x49, 0x45, 0x43 },   /* Z  */
};

/* Biggest text the driver will draw. Bounds the stack buffer below, which is
 * what lets a glyph go out in ONE transfer. */
#define TFT_MAX_SCALE 4

/*
 * One character, streamed as a single window.
 *
 * The obvious implementation calls tft::rect once per pixel block: 48 transfers a
 * character, each two commands, eight address bytes and a pair of GPIO toggles
 * to move as little as four bytes of colour. Building the glyph into a buffer
 * costs at most 1536 bytes of stack and turns a character into one address
 * setup and one burst.
 */
static Void drawChar(const Screen* s, Int32 x, Int32 y, Utf8 ch, UInt16 fg, UInt16 bg, Int32 scale)
{
    if(scale < 1)
    {
        scale = 1;
    }
    if(scale > TFT_MAX_SCALE)
    {
        scale = TFT_MAX_SCALE;
    }

    Utf8 c = ch;
    if(c >= 'a' && c <= 'z')
    {
        c = static_cast<Utf8>(c - 'a' + 'A');
    }
    if(c < 32 || c > 90)
    {
        c = '?';
    }

    const UInt8* const glyph = FONT5X7[c - 32];

    const Int32 w = 6 * scale;
    const Int32 h = 8 * scale;

    /* Off-screen entirely: nothing to do. Partial overlap is not clipped here -
     * the caller lays text out, and half a character is worse to look at than a
     * missing one. */
    if(x < 0 || y < 0 || x + w > s->width || y + h > s->height)
    {
        return;
    }

    UInt8 cell[6 * TFT_MAX_SCALE * 8 * TFT_MAX_SCALE * 2];

    const UInt8 fgHi = static_cast<UInt8>(fg >> 8);
    const UInt8 fgLo = static_cast<UInt8>(fg & 0xFF);
    const UInt8 bgHi = static_cast<UInt8>(bg >> 8);
    const UInt8 bgLo = static_cast<UInt8>(bg & 0xFF);

    Size at = 0;
    for(Int32 row = 0; row < h; ++row)
    {
        const Int32 gr = row / scale;
        for(Int32 col = 0; col < w; ++col)
        {
            const Int32 gc = col / scale;

            /* The sixth column is the gap between characters, DRAWN rather than
             * skipped so overwriting text leaves no comb of old pixels. */
            const UInt8 bits = (gc < 5) ? glyph[gc] : 0x00;
            const Bool  on   = (gr < 7) && (((bits >> gr) & 1u) != 0u);

            cell[at++] = on ? fgHi : bgHi;
            cell[at++] = on ? fgLo : bgLo;
        }
    }

    beginPixels(s, x, y, w, h);
    spi::write(s->sck, cell, at);
    endPixels(s);
}

static Void text(const Screen* s, Int32 x, Int32 y, const Utf8* str, UInt16 fg, UInt16 bg, Int32 scale)
{
    Int32 cx = x;
    while(str != NULL && *str != '\0')
    {
        drawChar(s, cx, y, *str, fg, bg, scale);
        cx += 6 * scale;
        ++str;
    }
}

/* ---- bring-up ------------------------------------------------------------ */

/*
 * Brings up a panel on the given pins and fills `s` in.
 *
 * Returns false only if the SPI pins do not form a bus. Everything after that
 * is write-only and cannot be checked, which is exactly why a first sketch
 * draws a test pattern rather than trusting a return code.
 */
static Bool openOn(Screen* s, Int32 w, Int32 h, Int32 xoff, Int32 yoff, Pin sck, Pin mosi, Pin cs, Pin dc, Pin res)
{
    if(s == NULL)
    {
        return false;
    }

    /* Clamped to what the controller can address, so a typo produces a wrong
     * picture rather than a window running off the end of its RAM. */
    s->width  = (w <= 0) ? PANEL_MAX_W : ((w > PANEL_MAX_W) ? PANEL_MAX_W : w);
    s->height = (h <= 0) ? PANEL_MAX_H : ((h > PANEL_MAX_H) ? PANEL_MAX_H : h);
    s->xoff   = (xoff < 0) ? 0 : xoff;
    s->yoff   = (yoff < 0) ? 0 : yoff;

    s->sck  = sck;
    s->mosi = mosi;
    s->cs   = cs;
    s->dc   = dc;
    s->res  = res;

    /* gfx.h's fields, left inert until gfx::open() claims them. A screen used
     * only through this file never allocates a back buffer. */
    s->buf       = NULL;
    s->dirtyTop  = s->height;
    s->dirtyBot  = -1;
    s->clipX     = 0;
    s->clipY     = 0;
    s->clipW     = s->width;
    s->clipH     = s->height;
    s->safeInset = 0;
    s->fg        = TFT_WHITE;
    s->bg        = TFT_BLACK;
    s->bgSolid   = true;
    s->textScale = 1;
    s->cursorX   = 0;
    s->cursorY   = 0;

    gpio::open(s->dc, PIN_DIR_OUT);
    gpio::open(s->res, PIN_DIR_OUT);

    if(!spi::open(s->sck, s->mosi, s->cs, PANEL_HZ))
    {
        return false;
    }

    /* Mode 3. spi_init leaves mode 0, and an ST7789 on the wrong mode reads
     * every byte shifted and behaves exactly as though nothing was sent. */
    spi::mode(s->sck, true, true);

    /* Hardware reset. The delays are from the datasheet and are not padding:
     * talking to the controller before it has finished resetting is the classic
     * way to get a panel that works only every other power-up. */
    gpio::write(s->res, true);
    timing::ms(50);
    gpio::write(s->res, false);
    timing::ms(50);
    gpio::write(s->res, true);
    timing::ms(150);

    cmd(s, 0x01);              /* SWRESET */
    timing::ms(150);
    cmd(s, 0x11);              /* SLPOUT - leave sleep */
    timing::ms(255);

#if PANEL_ST7735
    {
        /* Frame rate and power control. These are the values in every ST7735
         * bring-up in existence; they are panel timings rather than anything
         * derivable. */
        UInt8 b[16];

        b[0] = 0x01; b[1] = 0x2C; b[2] = 0x2D; write(s, 0xB1, b, 3);
        b[0] = 0x01; b[1] = 0x2C; b[2] = 0x2D; write(s, 0xB2, b, 3);
        b[0] = 0x01; b[1] = 0x2C; b[2] = 0x2D;
        b[3] = 0x01; b[4] = 0x2C; b[5] = 0x2D;
        write(s, 0xB3, b, 6);

        cmd1(s, 0xB4, 0x07);                                /* INVCTR   */
        b[0] = 0xA2; b[1] = 0x02; b[2] = 0x84;
        write(s, 0xC0, b, 3);
        cmd1(s, 0xC1, 0xC5);
        b[0] = 0x0A; b[1] = 0x00; write(s, 0xC2, b, 2);
        b[0] = 0x8A; b[1] = 0x2A; write(s, 0xC3, b, 2);
        b[0] = 0x8A; b[1] = 0xEE; write(s, 0xC4, b, 2);
        cmd1(s, 0xC5, 0x0E);                                /* VMCTR1   */
    }
    cmd1(s, 0x3A, 0x05);       /* COLMOD - 16 bit on ST7735 */
#else
    cmd1(s, 0x3A, 0x55);       /* COLMOD - 16 bit on ST7789 */
    timing::ms(10);

    /* The ST7789 power and porch settings. The datasheet defaults work on some
     * modules and not others; these are the values that work on all of them. */
    {
        UInt8 p[5];
        p[0] = 0x0C; p[1] = 0x0C; p[2] = 0x00; p[3] = 0x33; p[4] = 0x33;
        write(s, 0xB2, p, 5);       /* PORCTRL  */
        cmd1(s, 0xB7, 0x35);        /* GCTRL    */
        cmd1(s, 0xBB, 0x19);        /* VCOMS    */
        cmd1(s, 0xC0, 0x2C);        /* LCMCTRL  */
        cmd1(s, 0xC2, 0x01);        /* VDVVRHEN */
        cmd1(s, 0xC3, 0x12);        /* VRHS     */
        cmd1(s, 0xC4, 0x20);        /* VDVS     */
        cmd1(s, 0xC6, 0x0F);        /* FRCTRL2 - 60 Hz */
        p[0] = 0xA4; p[1] = 0xA1;
        write(s, 0xD0, p, 2);       /* PWCTRL1  */
    }
#endif

    /* MADCTL: row/column order and RGB-versus-BGR. 0x00 is the identity, which
     * is right for the common modules. If red and blue come out swapped, this
     * is the byte to change - try 0x08. */
    cmd1(s, 0x36, 0x00);

#if PANEL_INVERT
    cmd(s, 0x21);              /* INVON  */
#else
    cmd(s, 0x20);              /* INVOFF */
#endif

    cmd(s, 0x13);              /* NORON - normal display */
    timing::ms(10);
    cmd(s, 0x29);              /* DISPON */
    timing::ms(120);

    fill(s, TFT_BLACK);
    return true;
}

/* The same, on this project's pins. What a sketch normally calls. */
static Bool open(Screen* s, Int32 w, Int32 h, Int32 xoff, Int32 yoff)
{
    return openOn(s, w, h, xoff, yoff,
                     PIN_TFT_SCK, PIN_TFT_MOSI, PIN_TFT_CS,
                     PIN_TFT_DC, PIN_TFT_RES);
}


} // namespace tft

} // namespace bibo
