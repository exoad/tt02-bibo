/**
 * @file display.hxx
 * @brief The ST7789 / ST7735 panel driver: pins, commands, and one tft::Screen.
 *
 * ---------------------------------------------------------------------------
 * HOW THE TWO LAYERS FIT TOGETHER
 *
 * This file OWNS the hardware and hands back a tft::Screen. gfx.h DRAWS into one.
 *
 *     tft::Screen screen;
 *     tft::open(&screen, 240, 280, 0, 20);      the panel is now yours
 *     tft::fill(&screen, YELLOW);
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
 *     gfx::rectFill(&screen, 10, 10, 100, 40, ORANGE);
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
#include "../pins.hxx"

namespace bibo
{

  namespace tft
  {

    /* ===== the one line to change ============================================= */
    /**
     * @brief Selects the panel controller this file drives.
     *
     * Set exactly one of PANEL_ST7789 / PANEL_ST7735 to 1 and the other to 0.
     */
#define PANEL_ST7789 1
#define PANEL_ST7735 0
    /* ========================================================================== */

#if PANEL_ST7789
      /**
       * @brief Whether the controller must invert colors for this panel's glass.
       *
       * ST7789 panels are wired inverted at the glass, so "invert on" is what
       * produces correct colors. This looks backwards and is not.
       */
  #define PANEL_INVERT  1
#else
  #define PANEL_INVERT  0
#endif

    /**
     * @brief The largest panel this driver can address, not the panel's actual
     * size.
     *
     * It bounds the one stack buffer below and gfx.h's frame buffer, both of
     * which must be sized when the program is compiled rather than when it
     * runs. 240x320 is the largest this controller drives.
     */
#define PANEL_MAX_W   240
#define PANEL_MAX_H   320

    /**
     * @brief Default SPI and control pins for the panel.
     *
     * tft::openOn() takes them explicitly if a board differs.
     */
#define PIN_TFT_SCK   18
#define PIN_TFT_MOSI  19
#define PIN_TFT_CS    17
#define PIN_TFT_DC    21
#define PIN_TFT_RES   20

    /**
     * @brief SPI clock rate for the panel, in Hz.
     *
     * 24 MHz. Comfortably inside what these panels manage and slow enough
     * that a long jumper wire is not the reason it fails. Raise it once it
     * works, never before - a display that has never worked and one that is
     * too fast look identical.
     */
#define PANEL_HZ      24000000u

    /**
     * @brief Packs 8-bit red, green and blue into one RGB565 pixel value.
     *
     * 16 bits per pixel, 5 red / 6 green / 5 blue, which is what COLMOD is
     * set to below. Green gets the spare bit because the eye resolves most
     * detail there.
     *
     * constexpr rather than a macro. It was `#define TFT_RGB(r, g, b)` and
     * the colors below are still computed at compile time either way - what
     * a function adds is argument types, a return type, and evaluating each
     * argument exactly once. A macro doing arithmetic evaluates whatever you
     * hand it as many times as its body mentions it.
     *
     * @param r red, 0-255; only the top 5 bits are kept
     * @param g green, 0-255; only the top 6 bits are kept
     * @param b blue, 0-255; only the top 5 bits are kept
     * @return the packed RGB565 pixel value
     */
    constexpr UInt16 rgb(const UInt32 r, const UInt32 g, const UInt32 b)
    {
        return static_cast<UInt16>(((r & 0xF8u) << 8)
                                 | ((g & 0xFCu) << 3)
                                 | (b >> 3));
    }

    /** @brief Common colors, in RGB565, computed at compile time. */
    constexpr UInt16 BLACK = rgb(0, 0, 0);
    constexpr UInt16 WHITE = rgb(255, 255, 255);
    constexpr UInt16 RED = rgb(255, 0, 0);
    constexpr UInt16 GREEN = rgb(0, 255, 0);
    constexpr UInt16 BLUE = rgb(0, 0, 255);
    constexpr UInt16 YELLOW = rgb(255, 255, 0);
    constexpr UInt16 CYAN = rgb(0, 255, 255);
    constexpr UInt16 MAGENTA = rgb(255, 0, 255);
    constexpr UInt16 GREY = rgb(128, 128, 128);
    constexpr UInt16 ORANGE = rgb(255, 140, 0);
    /**
     * @brief Everything about one physical panel, in one place.
     *
     * Passed to every call in this file and in gfx.h, which is what lets a
     * sketch say what it has rather than inherit it from a header.
     *
     * This holds PANEL facts only - geometry, pins, safe area. The drawing
     * state (back buffer, clip rectangle, text color) is not here: it belongs
     * to gfx::Canvas, so this struct stays the hardware and gfx.h stays what
     * draws on it.
     */
    struct Screen
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

        /* The backlight, or PIN_NONE when it is tied to 3V3. */
        Pin blk;

        /**
         * @brief How far in from each edge is actually visible.
         *
         * These panels have ROUNDED CORNERS cut into the glass. The
         * controller still addresses the full rectangle and will happily
         * accept pixels for the corners; you simply cannot see them, and
         * text placed near an edge disappears into the curve.
         *
         * This is a property of the GLASS, not of the driver, so it lives on
         * the screen beside the size and offsets - the same category of
         * fact.
         *
         * Zero means "the whole rectangle is visible", which is right for a
         * panel with square corners and wrong for every one of these.
         */
        Int32 safeInset;
    };

    /**
     * @brief The panel's width in pixels.
     *
     * @param s the screen to ask, or nullptr
     * @return s->width, or 0 when s is nullptr
     */
    inline Int32 width(const Screen* s)
    {
        return s != nullptr ? s->width : 0;
    }

    /**
     * @brief The panel's height in pixels.
     *
     * @param s the screen to ask, or nullptr
     * @return s->height, or 0 when s is nullptr
     */
    inline Int32 height(const Screen* s)
    {
        return s != nullptr ? s->height : 0;
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
    /**
     * @brief Pulls CS low, claiming the SPI bus for this panel's transaction.
     *
     * @param s the screen whose CS pin is asserted
     */
    inline Void select(const Screen* s)
    {
        gpio::write(s->cs, false);
    }

    /**
     * @brief Raises CS, ending the current transaction.
     *
     * @param s the screen whose CS pin is released
     */
    inline Void deselect(const Screen* s)
    {
        gpio::write(s->cs, true);
    }

    /**
     * @brief Sends one command and its parameter bytes as a single SPI
     * transaction.
     *
     * @param s the screen to write to
     * @param cmd the controller command byte
     * @param params the command's parameter bytes, or nullptr when it takes
     *        none
     * @param n how many bytes `params` holds; may be 0
     */
    inline Void write(const Screen* s, const UInt8 cmd, const UInt8* params, const Size n)
    {
        select(s);

        gpio::write(s->dc, false);              /* low = this byte is a command */
        spi::writeByte(s->sck, cmd);

        if(params != nullptr && n > 0)
        {
            gpio::write(s->dc, true);           /* high = these are its parameters */
            spi::write(s->sck, params, n);
        }

        deselect(s);
    }

    /**
     * @brief Sends a command with no parameters.
     *
     * @param s the screen to write to
     * @param c the controller command byte
     */
    inline Void cmd(const Screen* s, const UInt8 c)
    {
        write(s, c, nullptr, 0);
    }

    /**
     * @brief Sends a command with a single parameter byte.
     *
     * @param s the screen to write to
     * @param c the controller command byte
     * @param p the one parameter byte
     */
    inline Void cmd1(const Screen* s, const UInt8 c, const UInt8 p)
    {
        write(s, c, &p, 1);
    }

    /**
     * @brief Sets the panel's write window, the rectangle the next pixel
     * burst fills.
     *
     * Every pixel push is "set a rectangle, then stream pixels into it",
     * which is why there is no per-pixel addressing anywhere below - the
     * controller advances its own cursor and wraps at the right edge.
     *
     * @param s the screen to address
     * @param x left edge of the window, in panel pixels before offset
     * @param y top edge of the window, in panel pixels before offset
     * @param w window width in pixels
     * @param h window height in pixels
     */
    inline Void window(const Screen* s, const Int32 x, const Int32 y, const Int32 w, const Int32 h)
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

    /**
     * @brief Opens a pixel write: sets the window, issues RAMWR, and leaves
     * CS low with DC high so pixels can follow in the same transaction.
     *
     * RAMWR is exactly the command whose data must not be separated from it,
     * since its "parameters" are the whole image.
     *
     * @param s the screen to address
     * @param x left edge of the region to fill, in panel pixels before offset
     * @param y top edge of the region to fill, in panel pixels before offset
     * @param w region width in pixels
     * @param h region height in pixels
     *
     * @note Must be closed with tft::endPixels() once the pixels have been
     * streamed.
     */
    inline Void beginPixels(const Screen* s, const Int32 x, const Int32 y, const Int32 w, const Int32 h)
    {
        window(s, x, y, w, h);

        select(s);
        gpio::write(s->dc, false);
        spi::writeByte(s->sck, 0x2C);           /* RAMWR */
        gpio::write(s->dc, true);
        /* CS stays LOW; the caller streams pixels now. */
    }

    /**
     * @brief Closes a pixel write opened by tft::beginPixels(), releasing CS.
     *
     * @param s the screen to address
     */
    inline Void endPixels(const Screen* s)
    {
        deselect(s);
    }

    /* ---- drawing -------------------------------------------------------------
     *
     * These go STRAIGHT to the panel and ignore any back buffer. They are the
     * driver's own drawing, for bring-up and for sketches that want nothing else.
     * gfx.h is the layer that buffers.
     */

    /**
     * @brief Internal drawing primitives that predate gfx.hxx; NOT the
     * interface.
     *
     * These predate gfx.hxx and are what it was built on top of. They stay
     * because gfx::span still pushes a run through rect(), but they are NOT
     * the way to draw: every one of them has a gfx:: counterpart that clips,
     * buffers and batches, and calling these instead bypasses all three.
     */
    namespace detail
    {

      /**
       * @brief Fills a rectangle directly, streamed from a small buffer
       * rather than a framebuffer.
       *
       * Clipped rather than trusted: an off-screen rectangle draws nothing
       * instead of wrapping around and corrupting the far edge.
       *
       * @param s the screen to draw on
       * @param x left edge, in screen pixels; may be negative or off-screen
       * @param y top edge, in screen pixels; may be negative or off-screen
       * @param w rectangle width in pixels
       * @param h rectangle height in pixels
       * @param colour the RGB565 fill color
       */
      inline Void rect(const Screen* s, Int32 x, Int32 y, Int32 w, Int32 h, const UInt16 colour)
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

      /**
       * @brief Fills the entire screen with one color.
       *
       * @param s the screen to draw on
       * @param colour the RGB565 fill color
       */
      inline Void fill(const Screen* s, const UInt16 colour)
      {
        rect(s, 0, 0, s->width, s->height, colour);
      }

      /**
       * @brief Sets one pixel.
       *
       * @param s the screen to draw on
       * @param x pixel column, in screen pixels
       * @param y pixel row, in screen pixels
       * @param colour the RGB565 pixel color
       */
      inline Void pixel(const Screen* s, const Int32 x, const Int32 y, const UInt16 colour)
      {
        rect(s, x, y, 1, 1, colour);
      }

      /* ---- text ----------------------------------------------------------------
      */
      /**
       * @brief A 5x7 pixel font, five bytes per glyph, one byte per column,
       * bit 0 at the top.
       *
       * Covers space through Z: digits, capitals and the punctuation a
       * status readout needs. Lowercase folds to uppercase rather than
       * shipping a second set of glyphs - this is for labels on a 240 pixel
       * screen, not for prose, and half a font that is CORRECT beats a whole
       * one that is guessed at.
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

      /**
       * @brief Largest text scale factor the driver will draw.
       *
       * Bounds the stack buffer below, which is what lets a glyph go out in
       * ONE transfer.
       */
#define TFT_MAX_SCALE 4

      /**
       * @brief Draws one character, streamed as a single window.
       *
       * The obvious implementation calls tft::rect once per pixel block: 48
       * transfers a character, each two commands, eight address bytes and a
       * pair of GPIO toggles to move as little as four bytes of color.
       * Building the glyph into a buffer costs at most 1536 bytes of stack
       * and turns a character into one address setup and one burst.
       *
       * @param s the screen to draw on
       * @param x left edge of the character cell, in screen pixels
       * @param y top edge of the character cell, in screen pixels
       * @param ch the character to draw; lowercase folds to uppercase and
       *        anything outside space-Z draws as '?'
       * @param fg foreground (glyph) color, RGB565
       * @param bg background (cell) color, RGB565
       * @param scale size multiplier, clamped to 1..TFT_MAX_SCALE
       */
      inline Void drawChar(const Screen* s, const Int32 x, const Int32 y, const Utf8 ch, const UInt16 fg, const UInt16 bg, Int32 scale)
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
                const UInt8 bits = gc < 5 ? glyph[gc] : 0x00;
                const Bool  on   = gr < 7 && ((bits >> gr) & 1u) != 0u;

                cell[at++] = on ? fgHi : bgHi;
                cell[at++] = on ? fgLo : bgLo;
            }
        }

        beginPixels(s, x, y, w, h);
        spi::write(s->sck, cell, at);
        endPixels(s);
      }

      /**
       * @brief Draws a string, one drawChar() per character, left to right.
       *
       * @param s the screen to draw on
       * @param x left edge of the first character, in screen pixels
       * @param y top edge of the text, in screen pixels
       * @param str the NUL-terminated string to draw, or nullptr to draw
       *        nothing
       * @param fg foreground (glyph) color, RGB565
       * @param bg background (cell) color, RGB565
       * @param scale size multiplier, clamped to 1..TFT_MAX_SCALE
       */
      inline Void text(const Screen* s, const Int32 x, const Int32 y, const Utf8* str, const UInt16 fg, const UInt16 bg, const Int32 scale)
      {
        Int32 cx = x;
        while(str != nullptr && *str != '\0')
        {
            drawChar(s, cx, y, *str, fg, bg, scale);
            cx += 6 * scale;
            ++str;
        }
      }

    }

    /* ---- bring-up ------------------------------------------------------------ */

    /**
     * @brief Brings up a panel on the given pins and fills `s` in.
     *
     * Returns false only if the SPI pins do not form a bus. Everything after
     * that is write-only and cannot be checked, which is exactly why a first
     * sketch draws a test pattern rather than trusting a return code.
     *
     * @param s the screen to initialize; must not be null
     * @param w panel width in pixels; clamped to PANEL_MAX_W
     * @param h panel height in pixels; clamped to PANEL_MAX_H
     * @param xoff column where the visible glass begins within controller RAM
     * @param yoff row where the visible glass begins within controller RAM
     * @param sck the SPI clock pin
     * @param mosi the SPI data-out pin
     * @param cs the chip-select pin for this panel
     * @param dc the data/command pin
     * @param res the hardware reset pin
     * @return true once the panel has been reset and configured; false only
     *         when the SPI pins do not form a valid bus
     *
     * @note Performs a hardware reset with datasheet-specified delays;
     * talking to the controller before it finishes resetting produces a
     * panel that works only every other power-up.
     */
    [[nodiscard]] static Bool openOn(Screen* s, const Int32 w, const Int32 h, const Int32 xoff, const Int32 yoff, const Pin sck, const Pin mosi, const Pin cs, const Pin dc, const Pin res)
    {
        if(s == nullptr)
        {
            return false;
        }

        /* Clamped to what the controller can address, so a typo produces a wrong
         * picture rather than a window running off the end of its RAM. */
        s->width  = w <= 0 ? PANEL_MAX_W : w > PANEL_MAX_W ? PANEL_MAX_W : w;
        s->height = h <= 0 ? PANEL_MAX_H : h > PANEL_MAX_H ? PANEL_MAX_H : h;
        s->xoff   = xoff < 0 ? 0 : xoff;
        s->yoff   = yoff < 0 ? 0 : yoff;

        s->sck  = sck;
        s->mosi = mosi;
        s->cs   = cs;
        s->dc   = dc;
        s->res  = res;

        /* Zero means "the whole rectangle is visible". A panel with rounded
         * corners is told its real inset by whoever knows the glass - see the
         * field's own note above. The DRAWING state that used to be zeroed here
         * belongs to gfx::Canvas now. */
        s->safeInset = 0;

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

            b[0] = 0x01;
            b[1] = 0x2C;
            b[2] = 0x2D;
            write(s, 0xB1, b, 3);
            b[0] = 0x01;
            b[1] = 0x2C;
            b[2] = 0x2D;
            write(s, 0xB2, b, 3);
            b[0] = 0x01;
            b[1] = 0x2C;
            b[2] = 0x2D;
            b[3] = 0x01;
            b[4] = 0x2C;
            b[5] = 0x2D;
            write(s, 0xB3, b, 6);

            cmd1(s, 0xB4, 0x07);                                /* INVCTR   */
            b[0] = 0xA2;
            b[1] = 0x02;
            b[2] = 0x84;
            write(s, 0xC0, b, 3);
            cmd1(s, 0xC1, 0xC5);
            b[0] = 0x0A;
            b[1] = 0x00;
            write(s, 0xC2, b, 2);
            b[0] = 0x8A;
            b[1] = 0x2A;
            write(s, 0xC3, b, 2);
            b[0] = 0x8A;
            b[1] = 0xEE;
            write(s, 0xC4, b, 2);
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
            p[0] = 0x0C;
            p[1] = 0x0C;
            p[2] = 0x00;
            p[3] = 0x33;
            p[4] = 0x33;
            write(s, 0xB2, p, 5);       /* PORCTRL  */
            cmd1(s, 0xB7, 0x35);        /* GCTRL    */
            cmd1(s, 0xBB, 0x19);        /* VCOMS    */
            cmd1(s, 0xC0, 0x2C);        /* LCMCTRL  */
            cmd1(s, 0xC2, 0x01);        /* VDVVRHEN */
            cmd1(s, 0xC3, 0x12);        /* VRHS     */
            cmd1(s, 0xC4, 0x20);        /* VDVS     */
            cmd1(s, 0xC6, 0x0F);        /* FRCTRL2 - 60 Hz */
            p[0] = 0xA4;
            p[1] = 0xA1;
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

        detail::fill(s, BLACK);
        return true;
    }

    /**
     * @brief Brings up the panel on this project's pins from pins::active().
     *
     * What a sketch normally calls.
     *
     * @param s the screen to initialize; must not be null
     * @param w panel width in pixels; clamped to PANEL_MAX_W
     * @param h panel height in pixels; clamped to PANEL_MAX_H
     * @param xoff column where the visible glass begins within controller RAM
     * @param yoff row where the visible glass begins within controller RAM
     * @return true once the panel has been reset and configured; false only
     *         when the SPI pins do not form a valid bus
     *
     * @note pins::begin() must have run first, so pins::active() has a real
     * map to read.
     */
    [[nodiscard]] static Bool open(Screen* s, const Int32 w, const Int32 h, const Int32 xoff, const Int32 yoff)
    {
        /* The pads THIS PROGRAM declared, not the defines above. The display's
         * wiring used to be fixed when the firmware was compiled, so a sketch
         * that moved the panel had to edit a driver; now it is five fields in a
         * pins::Map and no driver notices.
         *
         * openOn() still takes pins explicitly - for a program that wants pads
         * outside its own map, which is legitimate and rare. */
        const pins::Map& m = pins::active();

        if(!openOn(s, w, h, xoff, yoff,
                      m.tftSck, m.tftMosi, m.tftCs,
                      m.tftDc, m.tftRes))
        {
            return false;
        }

        s->blk = m.tftBlk;
        if(s->blk != pins::NONE)
        {
            /* 1 kHz: fast enough not to be seen, slow enough that the
             * backlight driver keeps up. */
            pwm::open(s->blk, 1000u);
            pwm::write(s->blk, 1.0f);
        }
        return true;
    }


    /* ===========================================================================
     * THE PANEL, as a component.
     *
     * This is the half of the split gfx does not do. gfx draws - shapes, text,
     * a back buffer, a clip rectangle - and knows nothing about the glass it
     * lands on. These are the things that are true of the PANEL and have no
     * meaning on a canvas: whether it is asleep, whether the controller inverts,
     * how bright the backlight is.
     *
     * A sketch uses gfx for a frame and reaches for these when it wants the
     * hardware itself.
     * ======================================================================== */

    /**
     * @brief Turns color inversion at the controller on or off.
     *
     * ST7789 glass is wired inverted, which is why PANEL_INVERT exists and
     * why open() already sets this - flipping it afterwards is for looking
     * at a panel you are not sure about.
     *
     * @param s the screen to address
     * @param on true to invert colors, false for normal
     */
    inline Void invert(const Screen* s, const Bool on)
    {
        cmd(s, on ? 0x21 : 0x20);   /* INVON / INVOFF */
    }

    /**
     * @brief Puts the controller into or out of its own low-power sleep
     * state.
     *
     * Sleep is the controller's own low-power state. It does NOT turn the
     * backlight off: on a board with BLK tied to 3V3 a sleeping panel is a
     * lit rectangle of nothing, which looks like a crash. Turn the backlight
     * down too if there is one.
     *
     * @param s the screen to address
     * @param on true to sleep the panel, false to wake it
     *
     * @note Waking (SLPOUT) needs a 120 ms settle before the panel responds;
     * sleeping (SLPIN) needs only 5 ms.
     */
    inline Void sleep(const Screen* s, const Bool on)
    {
        cmd(s, on ? 0x10 : 0x11);   /* SLPIN / SLPOUT */
        timing::ms(on ? 5u : 120u);         /* SLPOUT needs the long wait */
    }

    /**
     * @brief Blanks the panel's output without sleeping the controller.
     *
     * Faster to come back from than sleep().
     *
     * @param s the screen to address
     * @param on true to show the display, false to blank it
     */
    inline Void display(const Screen* s, const Bool on)
    {
        cmd(s, on ? 0x29 : 0x28);   /* DISPON / DISPOFF */
    }

    /**
     * @brief Whether this screen has a backlight pin to control.
     *
     * False on every board wired as shipped, where BLK is tied to 3V3.
     *
     * @param s the screen to ask
     * @return true when brightness() can actually change the backlight
     */
    [[nodiscard]] static Bool hasBacklight(const Screen* s)
    {
        return s->blk != pins::NONE;
    }

    /**
     * @brief Sets the backlight brightness.
     *
     * FALSE means BLK is tied to 3V3 - the panel is at full brightness and
     * there is no pad to change it. Reported rather than ignored, because a
     * brightness call that silently does nothing is indistinguishable from a
     * broken panel, and the fix is a wire rather than a line of code.
     *
     * @param s the screen to address
     * @param level 0.0 dark to 1.0 full; clamped into that range
     * @return true when a backlight pin exists and was written; false when
     *         there is no pad to dim
     */
    [[nodiscard]] static Bool brightness(const Screen* s, const Float32 level)
    {
        if(!hasBacklight(s))
        {
            return false;
        }
        pwm::write(s->blk, level < 0.0f ? 0.0f : level > 1.0f ? 1.0f : level);
        return true;
    }

  }

}
