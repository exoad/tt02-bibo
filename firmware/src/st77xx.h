/*
 * A small driver for the ST7789 and ST7735 SPI colour panels.
 *
 * ---------------------------------------------------------------------------
 * WHICH PANEL HAVE I GOT?
 *
 * The little boards labelled GND / VCC / SCL / SDA / RES / DC / CS / BLK are
 * sold with either controller behind them and the silkscreen rarely says. They
 * take the same wiring and almost the same commands, so this driver does both
 * and you pick with ONE line below.
 *
 *   ST7789   usually 240x240 (1.3") or 240x320 (2.0"), and INVERTED by default
 *   ST7735   usually 128x160 (1.8") or 128x128 (1.44"), not inverted
 *
 * Start with the default. If the screen lights up but shows nothing sensible,
 * change PANEL and reflash - that is a ten-second experiment and it is the
 * fastest way to find out what you own.
 *
 * The symptom table, because these are the four things that actually happen:
 *
 *   backlight on, screen black      wrong panel, or DC/RES on the wrong pin
 *   colours inverted (white->black) wrong PANEL_INVERT for this board
 *   image shifted by a few pixels   wrong PANEL_XOFF / PANEL_YOFF
 *   noise, or flickers and tears    SPI too fast - drop PANEL_HZ to 8000000
 *
 * ---------------------------------------------------------------------------
 * WIRING as built on this desk
 *
 *   Display   Pico          why
 *   GND       GND
 *   VCC       3V3           these modules are 3.3 V parts; do not feed them 5 V
 *   SCL/SCK   GP18          SPI0 SCK - fixed by the silicon, not a free choice
 *   SDA/MOSI  GP19          SPI0 TX
 *   RES       GP20          plain GPIO
 *   DC        GP21          plain GPIO: low = command, high = pixel data
 *   CS        GP17          plain GPIO, driven by us rather than the hardware
 *   BLK       3V3           always on. Move to a GPIO for PWM brightness.
 *
 * There is no MISO. These panels are write-only in practice: you cannot read
 * back what is on the screen, which is why there is no "read pixel" here and
 * why every drawing routine has to know what it drew.
 *
 * GP17/GP18/GP19 are the SPI pins docs/wiring.md reserves for the MicroSD card.
 * That is FINE and is the point of a bus - SCK and MOSI are shared, and the two
 * devices are told apart by their separate CS lines. The card will want its own
 * CS on a different pin when it arrives.
 */
#ifndef TT02_ST77XX_H
#define TT02_ST77XX_H

#include "pico2w.h"

/* ===== the one line to change ============================================= */
#define PANEL_ST7789 1
#define PANEL_ST7735 0
/* ========================================================================== */

/*
 * ST7789 SIZE. The controller is the same part in every one of these; only the
 * glass differs, and the module rarely says which it is. Set it to the number
 * printed on the listing you bought - or try them: a wrong size draws, it just
 * draws in the wrong place, which is a far better clue than a blank screen.
 *
 *   1  240x240   1.3 inch square
 *   2  240x280   1.69 inch tall
 *   3  240x320   2.0 inch
 *   4  172x320   1.47 inch
 *
 * The offsets are not decoration. The controller has 240x320 of RAM regardless,
 * so a shorter panel shows a WINDOW into it - a 240x280 starts 20 rows down,
 * and without that offset the top of the image is off the top of the glass.
 */
#define PANEL_ST7789_SIZE 1

#if PANEL_ST7789
  #if PANEL_ST7789_SIZE == 2
    #define PANEL_W     240
    #define PANEL_H     280
    #define PANEL_XOFF  0
    #define PANEL_YOFF  20
  #elif PANEL_ST7789_SIZE == 3
    #define PANEL_W     240
    #define PANEL_H     320
    #define PANEL_XOFF  0
    #define PANEL_YOFF  0
  #elif PANEL_ST7789_SIZE == 4
    #define PANEL_W     172
    #define PANEL_H     320
    #define PANEL_XOFF  34
    #define PANEL_YOFF  0
  #else
    #define PANEL_W     240
    #define PANEL_H     240
    #define PANEL_XOFF  0
    #define PANEL_YOFF  0
  #endif
  /* ST7789 panels are wired inverted at the glass, so "invert on" is what
   * produces correct colours. This looks backwards and is not. */
  #define PANEL_INVERT  1
#else
  #define PANEL_W       128
  #define PANEL_H       160
  #define PANEL_XOFF    0
  #define PANEL_YOFF    0
  #define PANEL_INVERT  0
#endif

#define PIN_TFT_SCK   18
#define PIN_TFT_MOSI  19
#define PIN_TFT_CS    17
#define PIN_TFT_DC    21
#define PIN_TFT_RES   20

/* 24 MHz. Comfortably inside what these panels manage and slow enough that a
 * long jumper wire is not the reason it fails. Raise it once it works, never
 * before - a display that has never worked and a display that is too fast look
 * identical. */
#define PANEL_HZ      24000000u

/* 16 bits per pixel, 5 red / 6 green / 5 blue, which is what COLMOD is set to
 * below. Green gets the spare bit because the eye has most resolution there. */
#define TFT_RGB(r, g, b) \
    ((UInt16) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

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

/* ---- the wire ------------------------------------------------------------ */

/*
 * CS LOW means "this transaction is mine"; raising it ENDS the transaction.
 *
 * That is the whole reason these are bracketed rather than each call driving CS
 * itself. A command and its parameters are ONE transaction: raise CS between
 * them and the controller treats the parameters as a fresh transaction that
 * begins with no command, and throws them away.
 *
 * The failure that produces is deeply unhelpful. SLPOUT takes no parameters, so
 * the panel still wakes up and the backlight still comes on - but COLMOD,
 * MADCTL, CASET, RASET and RAMWR all lose their data, so nothing is ever drawn.
 * A lit, blank screen and a dead wire look identical, and this is the reason
 * for it.
 */
static inline Void tftSelect(Void)
{
    gpioWrite(PIN_TFT_CS, false);
}

static inline Void tftDeselect(Void)
{
    gpioWrite(PIN_TFT_CS, true);
}

/* A command and its parameters, as one transaction. `n` may be 0. */
static inline Void tftWrite(UInt8 cmd, const UInt8* params, Size n)
{
    tftSelect();

    gpioWrite(PIN_TFT_DC, false);          /* low = this byte is a command */
    spiWriteByte(PIN_TFT_SCK, cmd);

    if(params != NULL && n > 0)
    {
        gpioWrite(PIN_TFT_DC, true);       /* high = these are its parameters */
        spiWrite(PIN_TFT_SCK, params, n);
    }

    tftDeselect();
}

static inline Void tftCmd(UInt8 c)
{
    tftWrite(c, NULL, 0);
}

static inline Void tftCmd1(UInt8 c, UInt8 p)
{
    tftWrite(c, &p, 1);
}

/*
 * The write window. Every pixel push is "set a rectangle, then stream pixels
 * into it", which is why there is no per-pixel addressing anywhere below - the
 * controller advances its own cursor and wraps at the right edge.
 */
static inline Void tftWindow(Int32 x, Int32 y, Int32 w, Int32 h)
{
    const Int32 x0 = x + PANEL_XOFF;
    const Int32 y0 = y + PANEL_YOFF;
    const Int32 x1 = x0 + w - 1;
    const Int32 y1 = y0 + h - 1;

    UInt8 buf[4];

    buf[0] = (UInt8) (x0 >> 8);
    buf[1] = (UInt8) (x0 & 0xFF);
    buf[2] = (UInt8) (x1 >> 8);
    buf[3] = (UInt8) (x1 & 0xFF);
    tftWrite(0x2A, buf, 4);                /* CASET - column address */

    buf[0] = (UInt8) (y0 >> 8);
    buf[1] = (UInt8) (y0 & 0xFF);
    buf[2] = (UInt8) (y1 >> 8);
    buf[3] = (UInt8) (y1 & 0xFF);
    tftWrite(0x2B, buf, 4);                /* RASET - row address */
}

/*
 * Opens a pixel write: sets the window, issues RAMWR, and LEAVES CS low with DC
 * high so the pixels can follow in the same transaction. Close with
 * tftEndPixels() - and note that RAMWR is exactly the command that must not be
 * separated from its data, since its "parameters" are the whole image.
 */
static inline Void tftBeginPixels(Int32 x, Int32 y, Int32 w, Int32 h)
{
    tftWindow(x, y, w, h);

    tftSelect();
    gpioWrite(PIN_TFT_DC, false);
    spiWriteByte(PIN_TFT_SCK, 0x2C);       /* RAMWR */
    gpioWrite(PIN_TFT_DC, true);
    /* CS stays LOW; the caller streams pixels now. */
}

static inline Void tftEndPixels(Void)
{
    tftDeselect();
}

/* ---- drawing ------------------------------------------------------------- */

/*
 * Filled rectangle. Streamed from a small buffer rather than a framebuffer: a
 * 240x240 screen at 16bpp is 115 KB, which the RP2350 could hold, but a driver
 * that needs a fifth of RAM before it draws anything is a bad neighbour to
 * every sketch that uses it.
 *
 * Clipped rather than trusted. An off-screen rectangle should draw nothing, not
 * wrap around and corrupt the far edge.
 */
static inline Void tftRect(Int32 x, Int32 y, Int32 w, Int32 h, UInt16 colour)
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
    if(x + w > PANEL_W)
    {
        w = PANEL_W - x;
    }
    if(y + h > PANEL_H)
    {
        h = PANEL_H - y;
    }
    if(w <= 0 || h <= 0)
    {
        return;
    }

    UInt8 line[PANEL_W * 2];
    const UInt8 hi = (UInt8) (colour >> 8);
    const UInt8 lo = (UInt8) (colour & 0xFF);
    for(Int32 i = 0; i < w; ++i)
    {
        line[i * 2]     = hi;
        line[i * 2 + 1] = lo;
    }

    tftBeginPixels(x, y, w, h);
    for(Int32 r = 0; r < h; ++r)
    {
        spiWrite(PIN_TFT_SCK, line, (Size) (w * 2));
    }
    tftEndPixels();
}

static inline Void tftFill(UInt16 colour)
{
    tftRect(0, 0, PANEL_W, PANEL_H, colour);
}

static inline Void tftPixel(Int32 x, Int32 y, UInt16 colour)
{
    tftRect(x, y, 1, 1, colour);
}

/* ---- text ---------------------------------------------------------------- */

/*
 * A 5x7 font, five bytes per glyph, one byte per column, bit 0 at the top.
 *
 * Covers space through Z: digits, capitals and the punctuation a status
 * readout needs. Lowercase is folded to uppercase rather than shipped as a
 * second set of glyphs - this is for labels on a 240 pixel screen, not for
 * prose, and half a font that is CORRECT beats a whole one that is guessed at.
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

/* Biggest text this will draw. Bounds the stack buffer in tftChar below, which
 * is what lets a glyph go out in ONE transfer. */
#define TFT_MAX_SCALE 4

/*
 * One character, streamed as a single window.
 *
 * The obvious implementation calls tftRect once per pixel-block, which is 48
 * transfers per character - each one two commands, eight address bytes and a
 * pair of GPIO toggles, to move as little as four bytes of colour. It works and
 * it is visibly slow: the overhead is thirty times the payload.
 *
 * Building the glyph into a buffer and pushing it in one go costs at most
 * 24 * 32 * 2 = 1536 bytes of stack at the largest scale, and turns a whole
 * character into one address setup and one burst.
 */
static inline Void tftChar(Int32 x, Int32 y, Utf8 ch, UInt16 fg, UInt16 bg,
                           Int32 scale)
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
        c = (Utf8) (c - 'a' + 'A');
    }
    if(c < 32 || c > 90)
    {
        c = '?';
    }

    const UInt8* const glyph = FONT5X7[c - 32];

    const Int32 w = 6 * scale;
    const Int32 h = 8 * scale;

    /* Off-screen entirely: nothing to do, and no window to set. Partial
     * overlap is not clipped here - the caller lays text out, and a half
     * character is worse to look at than a missing one. */
    if(x < 0 || y < 0 || x + w > PANEL_W || y + h > PANEL_H)
    {
        return;
    }

    UInt8 cell[6 * TFT_MAX_SCALE * 8 * TFT_MAX_SCALE * 2];

    const UInt8 fgHi = (UInt8) (fg >> 8);
    const UInt8 fgLo = (UInt8) (fg & 0xFF);
    const UInt8 bgHi = (UInt8) (bg >> 8);
    const UInt8 bgLo = (UInt8) (bg & 0xFF);

    Size at = 0;
    for(Int32 row = 0; row < h; ++row)
    {
        const Int32 gr = row / scale;
        for(Int32 col = 0; col < w; ++col)
        {
            const Int32 gc = col / scale;

            /* The sixth column is the gap between characters, and it is DRAWN
             * rather than skipped so that overwriting text does not leave a comb
             * of old pixels standing between the new ones. */
            const UInt8 bits = (gc < 5) ? glyph[gc] : 0x00;
            const Bool  on   = (gr < 7) && (((bits >> gr) & 1u) != 0u);

            cell[at++] = on ? fgHi : bgHi;
            cell[at++] = on ? fgLo : bgLo;
        }
    }

    tftBeginPixels(x, y, w, h);
    spiWrite(PIN_TFT_SCK, cell, at);
    tftEndPixels();
}

static inline Void tftText(Int32 x, Int32 y, const Utf8* s, UInt16 fg, UInt16 bg,
                           Int32 scale)
{
    Int32 cx = x;
    while(*s != '\0')
    {
        tftChar(cx, y, *s, fg, bg, scale);
        cx += 6 * scale;
        ++s;
    }
}

/* ---- bring-up ------------------------------------------------------------ */

/*
 * Resets and configures the panel. Returns false only if the SPI pins do not
 * form a bus - everything after that is write-only and cannot be checked,
 * which is exactly why the first sketch draws a test pattern instead of
 * trusting a return code.
 */
static inline Bool tftInit(Void)
{
    gpioOpen(PIN_TFT_DC, PIN_DIR_OUT);
    gpioOpen(PIN_TFT_RES, PIN_DIR_OUT);

    if(!spiOpen(PIN_TFT_SCK, PIN_TFT_MOSI, PIN_TFT_CS, PANEL_HZ))
    {
        return false;
    }

    /* Mode 3. spi_init leaves mode 0, and an ST7789 on the wrong mode reads
     * every byte shifted and behaves exactly as though nothing was sent. */
    spiMode(PIN_TFT_SCK, true, true);

    /* Hardware reset. The 120 ms is from the datasheet and is not padding:
     * talking to the controller before it has finished resetting is the classic
     * way to get a panel that works only every other power-up. */
    gpioWrite(PIN_TFT_RES, true);
    sleepMs(50);
    gpioWrite(PIN_TFT_RES, false);
    sleepMs(50);
    gpioWrite(PIN_TFT_RES, true);
    sleepMs(150);

    tftCmd(0x01);                 /* SWRESET */
    sleepMs(150);
    tftCmd(0x11);                 /* SLPOUT - leave sleep */
    sleepMs(255);

#if PANEL_ST7735
    {
        /* Frame rate and power control. These values are the ones in every
         * ST7735 bring-up in existence, including Adafruit's; they are panel
         * timings rather than anything derivable. */
        UInt8 b[16];

        b[0] = 0x01; b[1] = 0x2C; b[2] = 0x2D; tftWrite(0xB1, b, 3);
        b[0] = 0x01; b[1] = 0x2C; b[2] = 0x2D; tftWrite(0xB2, b, 3);
        b[0] = 0x01; b[1] = 0x2C; b[2] = 0x2D;
        b[3] = 0x01; b[4] = 0x2C; b[5] = 0x2D;
        tftWrite(0xB3, b, 6);

        tftCmd1(0xB4, 0x07);                                 /* INVCTR */
        b[0] = 0xA2; b[1] = 0x02; b[2] = 0x84; tftWrite(0xC0, b, 3);
        tftCmd1(0xC1, 0xC5);
        b[0] = 0x0A; b[1] = 0x00; tftWrite(0xC2, b, 2);
        b[0] = 0x8A; b[1] = 0x2A; tftWrite(0xC3, b, 2);
        b[0] = 0x8A; b[1] = 0xEE; tftWrite(0xC4, b, 2);
        tftCmd1(0xC5, 0x0E);                                 /* VMCTR1 */
    }
    tftCmd1(0x3A, 0x05);                 /* COLMOD - 16 bit on ST7735 */
#else
    tftCmd1(0x3A, 0x55);                 /* COLMOD - 16 bit on ST7789 */
    sleepMs(10);

    /* PORCTRL, GCTRL, VCOMS, LCMCTRL, VDVVRHEN, VRHS, VDVS, PWCTRL1 - the
     * ST7789 power and porch settings. The datasheet defaults work on some
     * modules and not others; these are the values that work on all of them. */
    {
        UInt8 p[5];
        p[0] = 0x0C; p[1] = 0x0C; p[2] = 0x00; p[3] = 0x33; p[4] = 0x33;
        tftWrite(0xB2, p, 5);            /* PORCTRL */
        tftCmd1(0xB7, 0x35);             /* GCTRL   */
        tftCmd1(0xBB, 0x19);             /* VCOMS   */
        tftCmd1(0xC0, 0x2C);             /* LCMCTRL */
        tftCmd1(0xC2, 0x01);             /* VDVVRHEN */
        tftCmd1(0xC3, 0x12);             /* VRHS    */
        tftCmd1(0xC4, 0x20);             /* VDVS    */
        tftCmd1(0xC6, 0x0F);             /* FRCTRL2 - 60 Hz */
        p[0] = 0xA4; p[1] = 0xA1;
        tftWrite(0xD0, p, 2);            /* PWCTRL1 */
    }
#endif

    /* MADCTL: row/column order and RGB-versus-BGR. 0x00 is the identity, which
     * is right for the common modules. If red and blue come out swapped, this
     * is the byte to change - try 0x08. */
    tftCmd1(0x36, 0x00);

#if PANEL_INVERT
    tftCmd(0x21);                 /* INVON  */
#else
    tftCmd(0x20);                 /* INVOFF */
#endif

    tftCmd(0x13);                 /* NORON - normal display */
    sleepMs(10);
    tftCmd(0x29);                 /* DISPON */
    sleepMs(120);

    tftFill(TFT_BLACK);
    return true;
}

#endif
