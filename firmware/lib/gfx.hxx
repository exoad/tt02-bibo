/**
 * @file gfx.hxx
 * @brief A 2D drawing API over a tft::Screen: shapes, text, color and
 *        clipping.
 *
 * st77xx.h is the DRIVER - chip selects, command bytes, address windows - and
 * it hands back a tft::Screen. This is what you draw with, and every call takes the
 * screen it draws into, wrapped in a gfx::Canvas:
 *
 *     tft::Screen screen;
 *     gfx::Canvas cv = gfx::open(&screen, {240, 280, 0, 20});
 *
 *     while(true)
 *     {
 *         cv.clear(NAVY)
 *           .rectFill({10, 10, 100, 40}, ORANGE)
 *           .circleFill({120, 120, 30}, CYAN)
 *           .text({10, 60}, "HELLO", {.fg = WHITE, .size = 2})
 *           .present();
 *     }
 *
 * The color, the text size and the clip rectangle live in the gfx::Canvas, not
 * in the tft::Screen or in this file's own state - see the Canvas banner
 * further down for why. They belong to the thing being drawn on, and keeping
 * them off the panel is what makes two screens possible rather than one
 * global canvas.
 *
 * ---------------------------------------------------------------------------
 * namespace detail, BELOW, IS NOT THE INTERFACE. It holds the functions this
 * file has always drawn with, each still taking its canvas as a first
 * argument; gfx::Canvas's methods are thin wrappers over them. A sketch
 * should only ever name a Canvas method - never detail:: directly.
 *
 * ---------------------------------------------------------------------------
 * THE BACK BUFFER, AND WHY IT IS WORTH THE RAM
 *
 * gfx::open() attaches one. Every call above then draws into memory and
 * Canvas::present() sends it in one burst. 240x320 at 16bpp is 153,600 bytes - 29%
 * of the RP2350's 520 KB - and it buys three things:
 *
 *   NO FLICKER. Without a buffer you draw on the glass, so clearing and
 *   redrawing means the viewer SEES the clear, every frame, as a flash.
 *
 *   OVERDRAW IS FREE. Background, then a shape on top, then text on top of
 *   that. Straight to the panel each is a round trip with the half-finished
 *   states visible.
 *
 *   IT IS FAST. One address window and one burst, and Canvas::present only
 *   pushes the ROWS THAT CHANGED - a ticking counter costs a fraction of a
 *   frame.
 *
 * A sketch that wants the RAM back uses tft::open() and the tft* calls instead;
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
#pragma once

#include "drivers/display.hxx"

#include <stdarg.h>
#include <stdio.h>

namespace bibo::gfx
{

    /* ---- color -----------------------------------------------------------
     *
     * 16-bit 5-6-5, the panel's own format.
     */
    /**
     * @brief Packs 8-bit red, green and blue into the panel's format.
     *
     * The same packing as tft::rgb, named for the layer a sketch is
     * working in.
     *
     * @param r red, 0-255
     * @param g green, 0-255
     * @param b blue, 0-255
     * @return the color as a 16-bit 5-6-5 value
     */
    constexpr UInt16 rgb(const UInt32 r, const UInt32 g, const UInt32 b)
    {
        return tft::rgb(r, g, b);
    }

    constexpr UInt16 BLACK     = rgb(0, 0, 0);
    constexpr UInt16 WHITE     = rgb(255, 255, 255);
    constexpr UInt16 RED       = rgb(255, 0, 0);
    constexpr UInt16 GREEN     = rgb(0, 255, 0);
    constexpr UInt16 BLUE      = rgb(0, 0, 255);
    constexpr UInt16 YELLOW    = rgb(255, 255, 0);
    constexpr UInt16 CYAN      = rgb(0, 255, 255);
    constexpr UInt16 MAGENTA   = rgb(255, 0, 255);
    constexpr UInt16 ORANGE    = rgb(255, 140, 0);
    constexpr UInt16 GREY      = rgb(128, 128, 128);
    constexpr UInt16 DARKGREY  = rgb(64, 64, 64);
    constexpr UInt16 NAVY      = rgb(12, 16, 32);
    constexpr UInt16 PURPLE    = rgb(160, 90, 220);
    /**
     * @brief Mixes two colors.
     *
     * Per channel after unpacking, because 5-6-5 cannot be averaged as one
     * integer: the channels would carry into each other and a half-way blend
     * of red and blue would come out an unrelated color.
     *
     * @param a the color for t = 0
     * @param b the color for t = 255
     * @param t the mix: 0 for all of `a`, 255 for all of `b`
     * @return the blended color
     */
    inline UInt16 blend(const UInt16 a, const UInt16 b, const UInt8 t)
    {
        const UInt32 ar = (a >> 11) & 0x1F;
        const UInt32 ag = (a >> 5)  & 0x3F;
        const UInt32 ab = a & 0x1F;

        const UInt32 br = (b >> 11) & 0x1F;
        const UInt32 bg = (b >> 5)  & 0x3F;
        const UInt32 bb = b & 0x1F;

        const UInt32 it = 255u - static_cast<UInt32>(t);
        const UInt32 r  = (ar * it + br * static_cast<UInt32>(t)) / 255u;
        const UInt32 g  = (ag * it + bg * static_cast<UInt32>(t)) / 255u;
        const UInt32 bl = (ab * it + bb * static_cast<UInt32>(t)) / 255u;

        return static_cast<UInt16>((r << 11) | (g << 5) | bl);
    }

    /**
     * @brief Darkens a color toward black.
     *
     * @param c the color to darken
     * @param amount how far toward black: 0 unchanged, 255 for black
     * @return the darkened color
     */
    inline UInt16 dim(const UInt16 c, const UInt8 amount)
    {
        return blend(c, BLACK, amount);
    }

    /**
     * @brief Lightens a color toward white.
     *
     * @param c the color to lighten
     * @param amount how far toward white: 0 unchanged, 255 for white
     * @return the lightened color
     */
    inline UInt16 lighten(const UInt16 c, const UInt8 amount)
    {
        return blend(c, WHITE, amount);
    }

    /**
     * @brief Converts a hue/saturation/value color to the panel's format.
     *
     * Integer throughout - no float, no table - so a rainbow sweep costs
     * nothing on a chip that would rather not.
     *
     * @param hue the hue in degrees; wrapped into 0-359 from either side
     * @param sat saturation, 0-255
     * @param val value (brightness), 0-255
     * @return the color as a 16-bit 5-6-5 value
     */
    inline UInt16 hsv(Int32 hue, const UInt8 sat, const UInt8 val)
    {
        hue = (hue % 360 + 360) % 360;

        const auto region = static_cast<UInt32>(hue / 60);
        const auto rem    = static_cast<UInt32>((hue - static_cast<Int32>(region * 60u)) * 255 / 60);

        const UInt32 p = static_cast<UInt32>(val) * (255u - sat) / 255u;
        const UInt32 q = static_cast<UInt32>(val) * (255u - sat * rem / 255u) / 255u;
        const UInt32 t = static_cast<UInt32>(val) * (255u - sat * (255u - rem) / 255u) / 255u;

        UInt32 r = 0;
        UInt32 g = 0;
        UInt32 b = 0;
        switch(region)
        {
            case 0:
                r = val;
                g = t;
                b = p;
                break;
            case 1:
                r = q;
                g = val;
                b = p;
                break;
            case 2:
                r = p;
                g = val;
                b = t;
                break;
            case 3:
                r = p;
                g = q;
                b = val;
                break;
            case 4:
                r = t;
                g = p;
                b = val;
                break;
            default:
                r = val;
                g = p;
                b = q;
                break;
        }
        return rgb(r, g, b);
    }

    /* ---- the back buffer -----------------------------------------------------
     *
     * ONE buffer, sized for the largest panel this controller drives, because a
     * static array has to be sized when the program is compiled and the panel is
     * not known until gfx::open() runs.
     *
     * One buffer means one BUFFERED screen. A second screen can still be opened
     * with tft::open() and drawn on directly; it simply does not get this. Saying so
     * plainly beats pretending otherwise and running off the end of it.
     *
     * The STRIDE is PANEL_MAX_W and not the screen's width, deliberately: the row a
     * pixel lives on has to be computed the same way every time, and rebasing the
     * stride on a resize would invalidate everything already drawn.
     */
    inline UInt16 buf[PANEL_MAX_W * PANEL_MAX_H];
    inline Bool   bufTaken = false;

    /* ===========================================================================
     * THE VALUE TYPES.
     *
     * A point is a point and a rectangle is a rectangle. These existed as loose
     * Int32 parameters - x, y, w, h - four at a time through forty signatures,
     * which is how `triangleFill` came to take seven numbers in a row where a
     * transposed pair is a silent bug rather than a compile error.
     * ======================================================================== */
    /**
     * @brief A pixel position: x right, y down, from the panel's top-left.
     */
    struct Point
    {
        Int32 x;
        Int32 y;
    };

    /**
     * @brief A pixel rectangle: top-left corner plus width and height.
     */
    struct Box
    {
        Int32 x;
        Int32 y;
        Int32 w;
        Int32 h;
    };

    /* ---------------------------------------------------------------------------
     * Paint - how a thing is drawn, as a VALUE.
     *
     * Flutter's idea, and it earns its place here for the same reason it does
     * there: style used to be sticky state on the screen, so `textColour()` set a
     * color that stayed set, and a function that drew red text left the next
     * caller drawing red text. That is a bug you find by looking at the screen
     * rather than at the code.
     *
     * A Paint is passed, used, and forgotten. Two paints side by side describe two
     * appearances without either one leaking into the other.
     * ------------------------------------------------------------------------ */
    /**
     * @brief Where a piece of text sits relative to the point it is drawn at.
     */
    enum Align
    {
        ALIGN_LEFT = 0,
        ALIGN_CENTRE,
        ALIGN_RIGHT
    };

    /**
     * @brief How a piece of text is drawn, as a value rather than sticky
     *        state.
     *
     * Passed to a text call and then forgotten - see the rationale above.
     */
    struct Paint
    {
        UInt16 fg      = WHITE;
        UInt16 bg      = BLACK;
        Bool   bgSolid = false;  /* false: glyph only, leave the background */
        Int32  size    = 1;      /* integer scale of the 5x7 font, >= 1 */
    };

    /* ---------------------------------------------------------------------------
     * Canvas - the thing you draw on.
     *
     * Holds a panel and the state that BELONGS TO DRAWING: the back buffer, which
     * rows are dirty, the clip rectangle, and the text cursor. All of that used to
     * live in tft::Screen under a comment that said "gfx.h's business", which is
     * a fair description of a field in the wrong struct.
     *
     * The panel keeps what is a fact about the HARDWARE - its size, its pins, and
     * the safe inset, which is a property of the glass. A Canvas is passed one and
     * does not own it: open two canvases on one panel and they share it, which is
     * the same thing two painters sharing a wall would mean.
     * ------------------------------------------------------------------------ */
    /**
     * @brief The thing you draw on: a panel plus everything drawing needs.
     *
     * Every drawing method returns *this so calls chain, and every query is
     * const and does not chain, since a query answers rather than draws -
     * see the header comment at the top of this file for what a chain of
     * calls looks like.
     */
    struct Canvas
    {
        tft::Screen* panel;

        /* The back buffer, or nullptr to draw straight at the panel. */
        UInt16* buf;

        /* Rows touched since the last present. Pushing only these is what makes a
         * small update cheap - a clock ticking in a corner sends twenty rows. */
        Int32 dirtyTop;
        Int32 dirtyBot;

        Int32 clipX;
        Int32 clipY;
        Int32 clipW;
        Int32 clipH;

        /* Sticky only for print()/printLine(), which are a stream and need a
         * position to continue from. Everything else takes a Paint. */
        Int32  cursorX;
        Int32  cursorY;
        UInt16 fg;
        UInt16 bg;
        Bool   bgSolid;
        Int32  textScale;

        /* Did the panel come up. False means the SPI pads were not a bus - and
         * that is ALL it can mean, because the panel is write-only and cannot be
         * asked whether it is there. See tft::open. */
        Bool opened;

        /* ---- the drawing surface ---------------------------------------------
         *
         * Every one of these returns *this, so a frame reads as a sequence of
         * things drawn rather than forty repetitions of the canvas's name. The
         * implementations are in detail:: below; these are the API.
         * ------------------------------------------------------------------- */
        /**
         * @brief Fills the whole canvas with one color.
         *
         * @param colour the fill color
         */
        Canvas& clear(UInt16 colour);

        /**
         * @brief Pushes whatever has changed since the last present to the
         *        panel.
         *
         * A no-op when nothing is dirty, so calling it every frame
         * regardless of whether anything was drawn costs nothing.
         */
        Canvas& present();

        /**
         * @brief Draws a single pixel.
         *
         * @param p the pixel's position
         * @param colour the color to draw with
         */
        Canvas& pixel(Point p, UInt16 colour);

        /**
         * @brief Draws a line between two points.
         *
         * @param a one endpoint
         * @param b the other endpoint
         * @param colour the color to draw with
         */
        Canvas& line(Point a, Point b, UInt16 colour);

        /**
         * @brief Draws a rectangle's outline.
         *
         * @param b the rectangle
         * @param colour the color to draw with
         */
        Canvas& rect(const Box& b, UInt16 colour);

        /**
         * @brief Draws a filled rectangle.
         *
         * @param b the rectangle
         * @param colour the fill color
         */
        Canvas& rectFill(const Box& b, UInt16 colour);

        /**
         * @brief Draws a rounded rectangle's outline.
         *
         * @param b the rectangle
         * @param radius the corner radius; clamped to half the shorter side
         * @param colour the color to draw with
         */
        Canvas& roundRect(const Box& b, Int32 radius, UInt16 colour);

        /**
         * @brief Draws a filled rounded rectangle.
         *
         * @param b the rectangle
         * @param radius the corner radius; clamped to half the shorter side
         * @param colour the fill color
         */
        Canvas& roundRectFill(const Box& b, Int32 radius, UInt16 colour);

        /**
         * @brief Draws a circle's outline.
         *
         * @param centre the circle's center
         * @param radius the radius; negative draws nothing
         * @param colour the color to draw with
         */
        Canvas& circle(Point centre, Int32 radius, UInt16 colour);

        /**
         * @brief Draws a filled circle.
         *
         * @param centre the circle's center
         * @param radius the radius; negative draws nothing
         * @param colour the fill color
         */
        Canvas& circleFill(Point centre, Int32 radius, UInt16 colour);

        /**
         * @brief Draws a triangle's outline as three lines.
         *
         * @param a the first vertex
         * @param b the second vertex
         * @param cc the third vertex
         * @param colour the color to draw with
         */
        Canvas& triangle(Point a, Point b, Point cc, UInt16 colour);

        /**
         * @brief Draws a filled triangle.
         *
         * @param a the first vertex
         * @param b the second vertex
         * @param cc the third vertex
         * @param colour the fill color
         */
        Canvas& triangleFill(Point a, Point b, Point cc, UInt16 colour);

        /* Text takes a Paint rather than leaving a color set behind it. */
        /**
         * @brief Draws left-aligned text at a position.
         *
         * @param at the top-left corner of the first glyph
         * @param str the text to draw; nothing is drawn for nullptr
         * @param paint the color, background and size to draw it with
         */
        Canvas& text(Point at, const Utf8* str, const Paint& paint);

        /**
         * @brief Draws text aligned to a position.
         *
         * @param at the anchor point; which edge of the text sits there
         *           depends on `align`
         * @param str the text to draw; nothing is drawn for nullptr
         * @param paint the color, background and size to draw it with
         * @param align which edge of the text is placed at `at`
         */
        Canvas& text(Point at, const Utf8* str, const Paint& paint, Align align);

        /**
         * @brief Formats text and draws it left-aligned at a position.
         *
         * @param at the top-left corner of the first glyph
         * @param paint the color, background and size to draw it with
         * @param fmt a printf-style format string
         *
         * @note The formatted text is truncated at 63 characters.
         */
        Canvas& printf(Point at, const Paint& paint, const Utf8* fmt, ...);

        /* Clipping and the safe area. */
        /**
         * @brief Restricts drawing to a rectangle.
         *
         * @param b the rectangle to clip to; clamped to the panel's bounds
         */
        Canvas& clip(const Box& b);

        /**
         * @brief Removes any clip rectangle, allowing the whole panel again.
         */
        Canvas& clipReset();

        /**
         * @brief Sets how far the safe area is inset from the panel's edges.
         *
         * @param px the inset in pixels; clamped to a third of the shorter
         *           side
         *
         * @note The panel's rounded corners cut into its outermost pixels;
         *       lay out anything that must be read inside the safe area
         *       rather than against 0 and width/height.
         */
        Canvas& safeInset(Int32 px);

        /**
         * @brief Draws the safe area's boundary, as a calibration aid.
         *
         * @param colour the color to draw it with
         */
        Canvas& safeOutline(UInt16 colour);

        /* Queries - const, and not chainable, because they answer rather than
         * draw. */
        /**
         * @brief Whether the panel came up.
         *
         * @return true if open() succeeded
         *
         * @note A canvas that did not open still draws, into its buffer,
         *       harmlessly - so this is the only way to find out.
         */
        [[nodiscard]] Bool  ok() const;

        /**
         * @brief The panel's width in pixels.
         */
        [[nodiscard]] Int32 width() const;

        /**
         * @brief The panel's height in pixels.
         */
        [[nodiscard]] Int32 height() const;

        /**
         * @brief The safe area, inset from the panel's edges.
         *
         * @return the safe rectangle, in the same coordinates as everything
         *         else drawn
         */
        [[nodiscard]] Box   safe() const;

        /**
         * @brief The width a piece of text would draw at, in pixels.
         *
         * @param str the text to measure
         * @param paint only its size is used
         * @return the width in pixels, or 0 for nullptr
         */
        Int32 textWidth(const Utf8* str, const Paint& paint) const;

        /**
         * @brief The height one line of text draws at, in pixels.
         *
         * @param paint only its size is used
         * @return the height in pixels
         */
        [[nodiscard]] Int32 textHeight(const Paint& paint) const;
    };

    /* ===========================================================================
     * detail - the implementations.
     *
     * These are the functions this file has always had, still taking their
     * canvas as a first parameter. They are not the API: Canvas's methods are,
     * and they are the only thing a sketch should name.
     * ======================================================================== */
  namespace detail
  {

      /* ---- the safe area -------------------------------------------------------
    *
    * Rounded corners are cut into the glass, so the outermost pixels of a panel
    * are addressable and invisible. Anything at a corner is lost, and text along
    * an edge vanishes into the curve.
    *
    * Set the inset once and lay out against these instead of against 0 and
    * width/height. The full rectangle is still reachable - Canvas::clear() fills
    * it, and a background should - but anything that has to be READ belongs
    * inside.
    *
    *     cv.safeInset(12);
    *     const gfx::Box area = cv.safe();
    *     cv.text({area.x, area.y}, "HELLO", {});
    *
    * 12 is a reasonable start for a 1.69 inch 240x280. Turn on
    * Canvas::safeOutline() for a frame to check against, then take it out.
    */
      /** @brief Sets the safe inset on the canvas's panel. See above. */
      inline Void safeInset(const Canvas* cv, const Int32 inset)
      {
          const Int32 most = (cv->panel->width < cv->panel->height ? cv->panel->width : cv->panel->height) / 3;
          cv->panel->safeInset = inset < 0 ? 0 : inset > most ? most : inset;
      }

      /** @brief The safe area's left edge. */
      inline Int32 safeLeft(const Canvas* cv)
      {
          return cv->panel->safeInset;
      }

      /** @brief The safe area's top edge. */
      inline Int32 safeTop(const Canvas* cv)
      {
          return cv->panel->safeInset;
      }

      /** @brief The safe area's right edge. */
      inline Int32 safeRight(const Canvas* cv)
      {
          return cv->panel->width - cv->panel->safeInset;
      }

      /** @brief The safe area's bottom edge. */
      inline Int32 safeBottom(const Canvas* cv)
      {
          return cv->panel->height - cv->panel->safeInset;
      }

      /** @brief The safe area's width. */
      inline Int32 safeWidth(const Canvas* cv)
      {
          return cv->panel->width - 2 * cv->panel->safeInset;
      }

      /** @brief The safe area's height. */
      inline Int32 safeHeight(const Canvas* cv)
      {
          return cv->panel->height - 2 * cv->panel->safeInset;
      }

      /* ---- clipping ------------------------------------------------------------ */

      /** @brief Clamps and sets the clip rectangle. See Canvas::clip. */
      inline Void clip(Canvas* cv, Int32 x, Int32 y, Int32 w, Int32 h)
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
          if(x + w > cv->panel->width)
          {
              w = cv->panel->width - x;
          }
          if(y + h > cv->panel->height)
          {
              h = cv->panel->height - y;
          }
          if(w < 0)
          {
              w = 0;
          }
          if(h < 0)
          {
              h = 0;
          }

          cv->clipX = x;
          cv->clipY = y;
          cv->clipW = w;
          cv->clipH = h;
      }

      /** @brief Resets the clip rectangle to the whole panel. */
      inline Void clipReset(Canvas* cv)
      {
          clip(cv, 0, 0, cv->panel->width, cv->panel->height);
      }

      /* ---- the two primitives everything is built from -------------------------
    *
    * A horizontal run is the fast path in both modes: a memory fill when buffered,
    * and ONE address window when not. Every shape below is expressed as runs for
    * that reason - a filled circle drawn as pixels is a thousand SPI transactions,
    * and drawn as spans it is thirty.
    */
      /**
       * @brief Fills a horizontal run of pixels, clipped to the canvas.
       *
       * The primitive every shape below is built from.
       */
      inline Void span(Canvas* cv, Int32 x, const Int32 y, Int32 len, const UInt16 colour)
      {
          if(len <= 0 || y < cv->clipY || y >= cv->clipY + cv->clipH)
          {
              return;
          }
          if(x < cv->clipX)
          {
              len -= cv->clipX - x;
              x = cv->clipX;
          }
          if(x + len > cv->clipX + cv->clipW)
          {
              len = cv->clipX + cv->clipW - x;
          }
          if(len <= 0)
          {
              return;
          }

          if(cv->buf != nullptr)
          {
              UInt16* p = &cv->buf[y * PANEL_MAX_W + x];
              for(Int32 i = 0; i < len; ++i)
              {
                  p[i] = colour;
              }
              if(y < cv->dirtyTop)
              {
                  cv->dirtyTop = y;
              }
              if(y > cv->dirtyBot)
              {
                  cv->dirtyBot = y;
              }
          }
          else
          {
              tft::detail::rect(cv->panel, x, y, len, 1, colour);
          }
      }

      /** @brief Draws a single pixel, via span(). */
      inline Void pixel(Canvas* cv, const Int32 x, const Int32 y, const UInt16 colour)
      {
          span(cv, x, y, 1, colour);
      }

      /* Reads a pixel back. Only possible with the buffer - the panel itself cannot
    * be read - so this returns black without one rather than lying. */
      /** @brief Reads back a pixel's color, or BLACK without a back buffer. */
      inline UInt16 peek(const Canvas* cv, const Int32 x, const Int32 y)
      {
          if(cv->buf == nullptr || x < 0 || y < 0 || x >= cv->panel->width || y >= cv->panel->height)
          {
              return BLACK;
          }
          return cv->buf[y * PANEL_MAX_W + x];
      }

      /* Alpha, which needs to read what is already there and so needs the buffer. */
      /** @brief Blends a color over the pixel already there. Needs the buffer. */
      inline Void pixelBlend(Canvas* cv, const Int32 x, const Int32 y, const UInt16 colour, const UInt8 alpha)
      {
          pixel(cv, x, y, blend(peek(cv, x, y), colour, alpha));
      }

      /* ---- present ------------------------------------------------------------- */

      /** @brief Pushes the dirty rows to the panel. See Canvas::present. */
      inline Void present(Canvas* cv)
      {
          if(cv->buf == nullptr || cv->dirtyBot < cv->dirtyTop)
          {
              return;                     /* nothing changed; do not touch the panel */
          }

          const Int32 y = cv->dirtyTop;
          const Int32 h = cv->dirtyBot - cv->dirtyTop + 1;
          const Int32 w = cv->panel->width;

          /* The buffer holds native-endian UInt16 and the panel wants big-endian, so
       * this cannot be one memcpy of the frame - the bytes go out a row at a time
       * through a swap. Still one address window and ONE transaction for the lot,
       * which is where nearly all of the win is. */
          UInt8 row[PANEL_MAX_W * 2];
          tft::beginPixels(cv->panel, 0, y, w, h);
          for(Int32 r = 0; r < h; ++r)
          {
              const UInt16* src = &cv->buf[((y + r) * PANEL_MAX_W)];
              for(Int32 i = 0; i < w; ++i)
              {
                  row[i * 2]     = static_cast<UInt8>(src[i] >> 8);
                  row[i * 2 + 1] = static_cast<UInt8>(src[i] & 0xFF);
              }
              spi::write(cv->panel->sck, row, static_cast<Size>(w * 2));
          }
          tft::endPixels(cv->panel);

          cv->dirtyTop = cv->panel->height;
          cv->dirtyBot = -1;
      }

      /** @brief Fills the whole canvas. See Canvas::clear. */
      inline Void clear(Canvas* cv, const UInt16 colour)
      {
          for(Int32 y = 0; y < cv->panel->height; ++y)
          {
              span(cv, 0, y, cv->panel->width, colour);
          }
      }

      /* ---- rectangles ---------------------------------------------------------- */

      /** @brief Fills a rectangle, as a stack of spans. */
      inline Void rectFill(Canvas* cv, const Int32 x, const Int32 y, const Int32 w, const Int32 h, const UInt16 colour)
      {
          for(Int32 r = 0; r < h; ++r)
          {
              span(cv, x, y + r, w, colour);
          }
      }

      /** @brief Draws a rectangle's outline. */
      inline Void rect(Canvas* cv, const Int32 x, const Int32 y, const Int32 w, const Int32 h, const UInt16 colour)
      {
          if(w <= 0 || h <= 0)
          {
              return;
          }
          span(cv, x, y, w, colour);
          span(cv, x, y + h - 1, w, colour);
          for(Int32 r = 1; r < h - 1; ++r)
          {
              pixel(cv, x, y + r, colour);
              pixel(cv, x + w - 1, y + r, colour);
          }
      }

      /* ---- lines --------------------------------------------------------------- */

      /** @brief Draws a horizontal line, via span(). */
      inline Void hLine(Canvas* cv, const Int32 x, const Int32 y, const Int32 w, const UInt16 colour)
      {
          span(cv, x, y, w, colour);
      }

      /** @brief Draws a vertical line, pixel by pixel. */
      inline Void vLine(Canvas* cv, const Int32 x, const Int32 y, const Int32 h, const UInt16 colour)
      {
          for(Int32 i = 0; i < h; ++i)
          {
              pixel(cv, x, y + i, colour);
          }
      }

      /*
    * Bresenham. Integer only - no division and no floating point in the loop -
    * which is why it has survived since 1962 and is still right on a
    * microcontroller.
    */
      /** @brief Draws a line between two points. See above. */
      inline Void line(Canvas* cv, const Int32 x0, const Int32 y0, const Int32 x1, const Int32 y1, const UInt16 colour)
      {
          const Int32 dx = x1 > x0 ? x1 - x0 : x0 - x1;
          const Int32 dy = y1 > y0 ? y1 - y0 : y0 - y1;
          const Int32 sx = x0 < x1 ? 1 : -1;
          const Int32 sy = y0 < y1 ? 1 : -1;

          /* The axis-aligned cases are common enough - borders, grids, axes - to be
       * worth the span path instead of stepping pixel by pixel. */
          if(dy == 0)
          {
              span(cv, x0 < x1 ? x0 : x1, y0, dx + 1, colour);
              return;
          }
          if(dx == 0)
          {
              vLine(cv, x0, y0 < y1 ? y0 : y1, dy + 1, colour);
              return;
          }

          Int32 err = dx - dy;
          Int32 x   = x0;
          Int32 y   = y0;

          while(true)
          {
              pixel(cv, x, y, colour);
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

      /** @brief Draws a circle's outline (midpoint algorithm). */
      inline Void circle(Canvas* cv, const Int32 cx, const Int32 cy, const Int32 r, const UInt16 colour)
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
              pixel(cv, cx + x, cy + y, colour);
              pixel(cv, cx - x, cy + y, colour);
              pixel(cv, cx + x, cy - y, colour);
              pixel(cv, cx - x, cy - y, colour);
              pixel(cv, cx + y, cy + x, colour);
              pixel(cv, cx - y, cy + x, colour);
              pixel(cv, cx + y, cy - x, colour);
              pixel(cv, cx - y, cy - x, colour);
              ++x;
              if(d < 0)
              {
                  d += 2 * x + 1;
              }
              else
              {
                  --y;
                  d += 2 * (x - y) + 1;
              }
          }
      }

      /** @brief Draws a filled circle, as horizontal spans. */
      inline Void circleFill(Canvas* cv, const Int32 cx, const Int32 cy, const Int32 r, const UInt16 colour)
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
              span(cv, cx - x, cy + y, 2 * x + 1, colour);
              span(cv, cx - x, cy - y, 2 * x + 1, colour);
              span(cv, cx - y, cy + x, 2 * y + 1, colour);
              span(cv, cx - y, cy - x, 2 * y + 1, colour);
              ++x;
              if(d < 0)
              {
                  d += 2 * x + 1;
              }
              else
              {
                  --y;
                  d += 2 * (x - y) + 1;
              }
          }
      }

      /* ---- rounded rectangles --------------------------------------------------
    *
    * Filled as three bands plus four corner discs. A disc of radius r centered r in
    * from each edge cannot reach past it, so the overdraw is free of side effects
    * and the code stays short.
    */
      /** @brief Draws a filled rounded rectangle. See above. */
      inline Void roundRectFill(Canvas* cv, const Int32 x, const Int32 y, const Int32 w, const Int32 h, Int32 r, const UInt16 colour)
      {
          if(w <= 0 || h <= 0)
          {
              return;
          }
          const Int32 maxR = (w < h ? w : h) / 2;
          if(r > maxR)
          {
              r = maxR;
          }
          if(r <= 0)
          {
              rectFill(cv, x, y, w, h, colour);
              return;
          }

          rectFill(cv, x + r, y, w - 2 * r, h, colour);
          rectFill(cv, x, y + r, r, h - 2 * r, colour);
          rectFill(cv, x + w - r, y + r, r, h - 2 * r, colour);

          circleFill(cv, x + r,         y + r,         r, colour);
          circleFill(cv, x + w - r - 1, y + r,         r, colour);
          circleFill(cv, x + r,         y + h - r - 1, r, colour);
          circleFill(cv, x + w - r - 1, y + h - r - 1, r, colour);
      }

      /** @brief Draws a rounded rectangle's outline: straight sides, arced corners. */
      inline Void roundRect(Canvas* cv, const Int32 x, const Int32 y, const Int32 w, const Int32 h, Int32 r, const UInt16 colour)
      {
          if(w <= 0 || h <= 0)
          {
              return;
          }
          const Int32 maxR = (w < h ? w : h) / 2;
          if(r > maxR)
          {
              r = maxR;
          }
          if(r <= 0)
          {
              rect(cv, x, y, w, h, colour);
              return;
          }

          span(cv, x + r, y, w - 2 * r, colour);
          span(cv, x + r, y + h - 1, w - 2 * r, colour);
          vLine(cv, x, y + r, h - 2 * r, colour);
          vLine(cv, x + w - 1, y + r, h - 2 * r, colour);

          Int32 cx = 0;
          Int32 cy = r;
          Int32 d  = 1 - r;
          while(cx <= cy)
          {
              pixel(cv, x + w - r - 1 + cx, y + h - r - 1 + cy, colour);
              pixel(cv, x + r - cx,         y + h - r - 1 + cy, colour);
              pixel(cv, x + w - r - 1 + cy, y + h - r - 1 + cx, colour);
              pixel(cv, x + r - cy,         y + h - r - 1 + cx, colour);

              pixel(cv, x + w - r - 1 + cx, y + r - cy, colour);
              pixel(cv, x + r - cx,         y + r - cy, colour);
              pixel(cv, x + w - r - 1 + cy, y + r - cx, colour);
              pixel(cv, x + r - cy,         y + r - cx, colour);

              ++cx;
              if(d < 0)
              {
                  d += 2 * cx + 1;
              }
              else
              {
                  --cy;
                  d += 2 * (cx - cy) + 1;
              }
          }
      }

      /* ---- triangles ----------------------------------------------------------- */

      /** @brief Draws a triangle's outline as three lines. */
      inline Void triangle(Canvas* cv, const Int32 x0, const Int32 y0, const Int32 x1, const Int32 y1, const Int32 x2, const Int32 y2, const UInt16 colour)
      {
          line(cv, x0, y0, x1, y1, colour);
          line(cv, x1, y1, x2, y2, colour);
          line(cv, x2, y2, x0, y0, colour);
      }

      /*
    * Scanline fill. Vertices sorted by y, then the triangle walked as two halves
    * that share the middle vertex, filling a span between the active edges.
    */
      /** @brief Draws a filled triangle. See above. */
      inline Void triangleFill(Canvas* cv, Int32 x0, Int32 y0, Int32 x1, Int32 y1, Int32 x2, Int32 y2, const UInt16 colour)
      {
          Int32 tx = 0;
          Int32 ty = 0;

          /* Three compare-and-swaps is a full sort for three items, and the middle
       * one is what splits the triangle into its two scanline halves. */
          if(y0 > y1)
          {
              tx = x0;
              x0 = x1;
              x1 = tx;
              ty = y0;
              y0 = y1;
              y1 = ty;
          }
          if(y1 > y2)
          {
              tx = x1;
              x1 = x2;
              x2 = tx;
              ty = y1;
              y1 = y2;
              y2 = ty;
          }
          if(y0 > y1)
          {
              tx = x0;
              x0 = x1;
              x1 = tx;
              ty = y0;
              y0 = y1;
              y1 = ty;
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
              span(cv, lo, y0, hi - lo + 1, colour);
              return;
          }

          for(Int32 y = y0; y <= y2; ++y)
          {
              const Bool second = y > y1;

              const Int32 aY0 = y0;
              const Int32 aY1 = y2;
              const Int32 aX0 = x0;
              const Int32 aX1 = x2;

              const Int32 bY0 = second ? y1 : y0;
              const Int32 bY1 = second ? y2 : y1;
              const Int32 bX0 = second ? x1 : x0;
              const Int32 bX1 = second ? x2 : x1;

              const Int32 aDen = aY1 - aY0;
              const Int32 bDen = bY1 - bY0;

              const Int32 ax = aX0 + (aX1 - aX0) * (y - aY0) / (aDen == 0 ? 1 : aDen);
              const Int32 bx = bDen == 0 ? bX1
                                   : bX0 + (bX1 - bX0) * (y - bY0) / bDen;

              const Int32 lo = ax < bx ? ax : bx;
              const Int32 hi = ax < bx ? bx : ax;
              span(cv, lo, y, hi - lo + 1, colour);
          }
      }

      /* ---- text ------------------------------------------------------------
    *
    * Set the color and size with a Paint, then draw with Canvas::text() or
    * Canvas::printf(). cursor(), textAt() and friends below are the
    * lower-level pieces those are built from; textAligned() is reached from
    * Canvas::text()'s aligned overload.
    */

      /** @brief Sets the foreground color used for text drawn from here on. */
      inline Void textColour(Canvas* cv, const UInt16 fg)
      {
          cv->fg = fg;
      }

      /* An opaque background, which is what you want for a value that changes: the
    * new text erases the old as it draws, with no flicker and no clear step. */
      /** @brief Sets an opaque background color for text drawn from here on. */
      inline Void textBackground(Canvas* cv, const UInt16 bg)
      {
          cv->bg      = bg;
          cv->bgSolid = true;
      }

      /* Leave whatever is behind the glyph alone - for text over a picture. */
      /** @brief Makes text drawn from here on leave its background untouched. */
      inline Void textTransparent(Canvas* cv)
      {
          cv->bgSolid = false;
      }

      /** @brief Sets the integer font scale, clamped to 1-8. */
      inline Void textSize(Canvas* cv, const Int32 scale)
      {
          cv->textScale = scale < 1 ? 1 : scale > 8 ? 8 : scale;
      }

      /** @brief Moves the text cursor used by print()/printLine(). */
      inline Void cursor(Canvas* cv, const Int32 x, const Int32 y)
      {
          cv->cursorX = x;
          cv->cursorY = y;
      }

      /** @brief The height of one line of text at the current scale. */
      inline Int32 textHeight(const Canvas* cv)
      {
          return 8 * cv->textScale;
      }

      /** @brief The width of one glyph at the current scale. */
      inline Int32 charWidth(const Canvas* cv)
      {
          return 6 * cv->textScale;
      }

      /** @brief The width a string would draw at, or 0 for nullptr. */
      inline Int32 textWidth(const Canvas* cv, const Utf8* str)
      {
          Int32 n = 0;
          while(str != nullptr && str[n] != '\0')
          {
              ++n;
          }
          return n * charWidth(cv);
      }

      /** @brief Draws one glyph. Lowercase is folded to upper; the rest is '?'. */
      inline Void charAt(Canvas* cv, const Int32 x, const Int32 y, const Utf8 ch)
      {
          Utf8 c = ch;
          if(c >= 'a' && c <= 'z')
          {
              c = static_cast<Utf8>(c - 'a' + 'A');
          }
          if(c < 32 || c > 90)
          {
              c = '?';
          }

          const UInt8* const glyph = tft::detail::FONT5X7[c - 32];

          for(Int32 col = 0; col < 6; ++col)
          {
              const UInt8 bits = col < 5 ? glyph[col] : 0x00;

              for(Int32 row = 0; row < 8; ++row)
              {
                  const Bool on = row < 7 && ((bits >> row) & 1u) != 0u;
                  if(!on && !cv->bgSolid)
                  {
                      continue;                   /* see through to what is behind */
                  }
                  rectFill(cv, x + col * cv->textScale, y + row * cv->textScale,
                           cv->textScale, cv->textScale, on ? cv->fg : cv->bg);
              }
          }
      }

      /** @brief Draws a string left-aligned at a position. */
      inline Void textAt(Canvas* cv, const Int32 x, const Int32 y, const Utf8* str)
      {
          Int32 cx = x;
          while(str != nullptr && *str != '\0')
          {
              charAt(cv, cx, y, *str);
              cx += charWidth(cv);
              ++str;
          }
      }


      /* `x` is the left edge, the center or the right edge depending on `align` -
    * which is what makes a value that changes width stay put. */
      /** @brief Draws a string aligned to a position. See Canvas::text. */
      inline Void textAligned(Canvas* cv, const Int32 x, const Int32 y, const Utf8* str, const Align align)
      {
          const Int32 w  = textWidth(cv, str);
          Int32       at = x;
          if(align == ALIGN_CENTRE)
          {
              at = x - w / 2;
          }
          else if(align == ALIGN_RIGHT)
          {
              at = x - w;
          }
          textAt(cv, at, y, str);
      }

      /* Draws at the cursor and advances it, so consecutive calls flow. */
      /**
       * @brief Draws at the cursor and advances it horizontally.
       *
       * @note Not reached from any Canvas method today; see the text banner
       *       above.
       */
      inline Void print(Canvas* cv, const Utf8* str)
      {
          textAt(cv, cv->cursorX, cv->cursorY, str);
          cv->cursorX += textWidth(cv, str);
      }

      /**
       * @brief Draws at the cursor and drops it to the next line.
       *
       * @note Not reached from any Canvas method today; see the text banner
       *       above.
       */
      inline Void printLine(Canvas* cv, const Utf8* str)
      {
          textAt(cv, cv->cursorX, cv->cursorY, str);
          cv->cursorY += textHeight(cv) + 2 * cv->textScale;
      }

      /* printf into the cursor. The buffer is deliberately small: this is a 240 pixel
    * screen and forty characters already overflow it at size 1. */
      /**
       * @brief Formats text and draws it at the cursor. See print().
       *
       * @note Not reached from any Canvas method today; Canvas::printf
       *       formats and calls textAt() directly instead.
       */
      inline Void printf(Canvas* cv, const Utf8* fmt, ...)
      {
          Utf8    buf[64];
          va_list ap;
          va_start(ap, fmt);
          vsnprintf(buf, sizeof(buf), fmt, ap);
          va_end(ap);
          print(cv, buf);
      }

      /*
    * Draws the safe area's boundary. A calibration aid, not decoration: run it,
    * look at the panel, and change the inset until the frame is fully visible with
    * a little to spare. Then take the call out.
    */
      /** @brief Draws the safe area's boundary. See Canvas::safeOutline. */
      inline Void safeOutline(Canvas* cv, const UInt16 colour)
      {
          rect(cv, safeLeft(cv), safeTop(cv),
               safeWidth(cv), safeHeight(cv), colour);
      }

      /* ---- start --------------------------------------------------------------- */

      /**
       * @brief Brings up the panel and attaches the back buffer.
       *
       * @param cv the canvas to initialize
       * @param panel the panel to open; not owned
       * @param w the panel's width in pixels
       * @param h the panel's height in pixels
       * @param xoff the panel's column offset into the controller's RAM
       * @param yoff the panel's row offset into the controller's RAM
       * @return what tft::open returns, which can only tell you the SPI pins
       *         were valid - see the note in st77xx.h about the panel being
       *         write-only
       */
      [[nodiscard]] static Bool open(Canvas* cv, tft::Screen* panel, const Int32 w, const Int32 h, const Int32 xoff, const Int32 yoff)
      {
          cv->panel = panel;

          /* The drawing state starts here rather than in the driver. tft::openOn
       * used to zero these, which meant a panel arrived pre-loaded with a text
       * color and a clip rectangle it had no business having an opinion on. */
          cv->buf       = nullptr;
          cv->dirtyTop  = h;
          cv->dirtyBot  = -1;
          cv->clipX     = 0;
          cv->clipY     = 0;
          cv->clipW     = w;
          cv->clipH     = h;
          cv->cursorX   = 0;
          cv->cursorY   = 0;
          cv->fg        = WHITE;
          cv->bg        = BLACK;
          cv->bgSolid   = true;
          cv->textScale = 1;

          if(!tft::open(cv->panel, w, h, xoff, yoff))
          {
              return false;
          }

          /* One buffer, so the FIRST screen to ask gets it. A second screen still
       * works and simply draws straight at its panel - which is the honest
       * outcome, and better than two screens quietly sharing one buffer. */
          if(!bufTaken)
          {
              bufTaken = true;
              cv->buf      = buf;
          }

          clipReset(cv);
          clear(cv, BLACK);
          present(cv);
          return true;
      }

  }

    /* ===========================================================================
     * Canvas, the API.
     *
     * Thin, and deliberately so: each one forwards to detail:: and returns *this.
     * The value is not in what they do, it is in what a frame LOOKS like once
     * they exist - see the header comment at the top of this file.
     * ======================================================================== */
    inline Canvas& Canvas::clear(const UInt16 colour)
    {
        detail::clear(this, colour);
        return *this;
    }
    inline Canvas& Canvas::present()
    {
        detail::present(this);
        return *this;
    }
    inline Canvas& Canvas::clipReset()
    {
        detail::clipReset(this);
        return *this;
    }
    inline Canvas& Canvas::safeInset(const Int32 px)
    {
        detail::safeInset(this, px);
        return *this;
    }
    inline Canvas& Canvas::safeOutline(const UInt16 colour)
    {
        detail::safeOutline(this, colour);
        return *this;
    }
    inline Canvas& Canvas::clip(const Box& b)
    {
        detail::clip(this, b.x, b.y, b.w, b.h);
        return *this;
    }

    inline Canvas& Canvas::pixel(const Point p, const UInt16 colour)
    {
        detail::pixel(this, p.x, p.y, colour);
        return *this;
    }

    inline Canvas& Canvas::line(const Point a, const Point b, const UInt16 colour)
    {
        detail::line(this, a.x, a.y, b.x, b.y, colour);
        return *this;
    }

    inline Canvas& Canvas::rect(const Box& b, const UInt16 colour)
    {
        detail::rect(this, b.x, b.y, b.w, b.h, colour);
        return *this;
    }

    inline Canvas& Canvas::rectFill(const Box& b, const UInt16 colour)
    {
        detail::rectFill(this, b.x, b.y, b.w, b.h, colour);
        return *this;
    }

    inline Canvas& Canvas::roundRect(const Box& b, const Int32 radius, const UInt16 colour)
    {
        detail::roundRect(this, b.x, b.y, b.w, b.h, radius, colour);
        return *this;
    }

    inline Canvas& Canvas::roundRectFill(const Box& b, const Int32 radius, const UInt16 colour)
    {
        detail::roundRectFill(this, b.x, b.y, b.w, b.h, radius, colour);
        return *this;
    }

    inline Canvas& Canvas::circle(const Point centre, const Int32 radius, const UInt16 colour)
    {
        detail::circle(this, centre.x, centre.y, radius, colour);
        return *this;
    }

    inline Canvas& Canvas::circleFill(const Point centre, const Int32 radius, const UInt16 colour)
    {
        detail::circleFill(this, centre.x, centre.y, radius, colour);
        return *this;
    }

    inline Canvas& Canvas::triangle(const Point a, const Point b, const Point cc, const UInt16 colour)
    {
        detail::triangle(this, a.x, a.y, b.x, b.y, cc.x, cc.y, colour);
        return *this;
    }

    inline Canvas& Canvas::triangleFill(const Point a, const Point b, const Point cc, const UInt16 colour)
    {
        detail::triangleFill(this, a.x, a.y, b.x, b.y, cc.x, cc.y, colour);
        return *this;
    }

    /* A Paint is applied and then left behind: the sticky fields still exist
     * because print()/printLine() stream from a cursor, but nothing OUTSIDE this
     * file can observe them, so two paints cannot leak into each other. */
    /**
     * @brief Copies a Paint's fields onto the canvas's sticky text state.
     *
     * @param cv the canvas to set the state on
     * @param p the appearance to apply
     */
    inline Void applyPaint(Canvas* cv, const Paint& p)
    {
        detail::textColour(cv, p.fg);
        detail::textSize(cv, p.size > 0 ? p.size : 1);
        if(p.bgSolid)
        {
            detail::textBackground(cv, p.bg);
        }
        else
        {
            detail::textTransparent(cv);
        }
    }

    inline Canvas& Canvas::text(const Point at, const Utf8* str, const Paint& paint)
    {
        applyPaint(this, paint);
        detail::textAt(this, at.x, at.y, str);
        return *this;
    }

    inline Canvas& Canvas::text(const Point at, const Utf8* str, const Paint& paint, const Align align)
    {
        applyPaint(this, paint);
        detail::textAligned(this, at.x, at.y, str, align);
        return *this;
    }

    inline Canvas& Canvas::printf(const Point at, const Paint& paint, const Utf8* fmt, ...)
    {
        Utf8    buf[64];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);

        applyPaint(this, paint);
        detail::textAt(this, at.x, at.y, buf);
        return *this;
    }

    inline Bool Canvas::ok() const
    {
        return opened;
    }

    inline Int32 Canvas::width() const
    {
        return panel->width;
    }

    inline Int32 Canvas::height() const
    {
        return panel->height;
    }

    inline Box Canvas::safe() const
    {
        return Box{ .x = detail::safeLeft(this), .y = detail::safeTop(this),
            .w = detail::safeWidth(this), .h = detail::safeHeight(this) };
    }

    inline Int32 Canvas::textWidth(const Utf8* str, const Paint& paint) const
    {
        const Int32 scale = paint.size > 0 ? paint.size : 1;
        return str == nullptr ? 0 : static_cast<Int32>(text::len(str)) * 6 * scale;
    }

    inline Int32 Canvas::textHeight(const Paint& paint) const
    {
        return 8 * (paint.size > 0 ? paint.size : 1);
    }

    /* ---------------------------------------------------------------------------
     * Bring a canvas up on a panel.
     *
     * The panel is PASSED IN and not owned: gfx is the drawing layer and tft is
     * the hardware, and this is the one line where they meet. A sketch that wants
     * the panel's own controls - brightness, inversion, sleep - still has it.
     * ------------------------------------------------------------------------ */
    /**
     * @brief A panel's geometry: its size and its offset into controller RAM.
     */
    struct PanelSize
    {
        Int32 w;
        Int32 h;
        Int32 xoff;
        Int32 yoff;
    };

    /**
     * @brief Opens a canvas on a panel.
     *
     * @param panel the panel to draw on; passed in and not owned - see above
     * @param g the panel's size and offset
     * @return a Canvas ready to draw with; check ok() before trusting it
     */
    inline Canvas open(tft::Screen* panel, const PanelSize& g)
    {
        Canvas cv{};
        cv.opened = detail::open(&cv, panel, g.w, g.h, g.xoff, g.yoff);
        return cv;
    }


}
