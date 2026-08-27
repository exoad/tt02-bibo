// The reference library. See reference.hpp for why the pages are drawn rather
// than shipped as images.
//
// Each page is a plain function over a Canvas, working in its own page units.
// The table at the bottom is the only thing that has to change when one is
// added.

#include "shared.hpp"
#include "reference.hpp"

#include "board_view.hpp"
#include "icons.hpp"
#include "theme.hpp"

#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ref {
namespace {

// ============================================================== palette ==
//
// Deliberately the app's own colours rather than a datasheet's. A reference you
// read twenty times a day should look like the tool it lives in, not like a PDF
// pasted into it.
constexpr ImU32 INK        = IM_COL32(0xE8, 0xE4, 0xDA, 0xFF);
constexpr ImU32 INK_DIM    = IM_COL32(0x92, 0x8C, 0x82, 0xFF);
constexpr ImU32 INK_FAINT  = IM_COL32(0x60, 0x5C, 0x56, 0xFF);
constexpr ImU32 PAPER      = IM_COL32(0x14, 0x15, 0x18, 0xFF);
constexpr ImU32 RULE       = IM_COL32(0x33, 0x34, 0x38, 0xFF);

constexpr ImU32 C_GPIO     = IM_COL32(0x6D, 0xA8, 0x5E, 0xFF);   // free to use
constexpr ImU32 C_USED     = IM_COL32(0xD8, 0x9E, 0x3C, 0xFF);   // spoken for
constexpr ImU32 C_GROUND   = IM_COL32(0x6A, 0x6A, 0x6A, 0xFF);
constexpr ImU32 C_POWER    = IM_COL32(0xC0, 0x4B, 0x3F, 0xFF);
constexpr ImU32 C_SYS      = IM_COL32(0x51, 0x7C, 0xA8, 0xFF);
constexpr ImU32 C_WIRE     = IM_COL32(0x8A, 0x86, 0x7E, 0xFF);
constexpr ImU32 C_NOTE     = IM_COL32(0xB5, 0x8F, 0x4A, 0xFF);

inline Float32 minf(Float32 a, Float32 b)
{
    return a < b ? a : b;
}

inline Float32 maxf(Float32 a, Float32 b)
{
    return a > b ? a : b;
}

inline Float32 clampf(Float32 v, Float32 lo, Float32 hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// The font every page draws with. One face, one place to change it.
ImFont* pageFont()
{
    return ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
}

Float32 pageFontBase()
{
    ImFont* f = pageFont();
    return (f != nullptr && f->LegacySize > 0.0f) ? f->LegacySize
                                                  : ImGui::GetFontSize();
}

} // namespace

// ============================================================== Canvas ==

Void Canvas::text(Float32 x, Float32 y, ImU32 col, const Char* s, Float32 pt,
                  Align align) const
{
    if(dl == nullptr || s == nullptr || *s == '\0')
    {
        return;
    }

    ImFont* f = pageFont();

    // Floored at 9 physical pixels. A diagram whose labels have gone illegible
    // is worse than one that has stopped being exactly to scale, and at low
    // zoom the second is a much smaller loss than the first.
    const Float32 px = maxf(9.0f, pt * scale);
    const Float32 w  = f->CalcTextSizeA(px, FLT_MAX, 0.0f, s).x;

    ImVec2 p = at(x, y);
    if(align == Align::ALIGN_CENTRE)
    {
        p.x -= w * 0.5f;
    }
    else if(align == Align::ALIGN_RIGHT)
    {
        p.x -= w;
    }
    p.y -= px * 0.5f;   // vertically centred on y, which is what callers mean

    dl->AddText(f, px, p, col, s);
}

Float32 Canvas::textWidth(const Char* s, Float32 pt) const
{
    if(s == nullptr || *s == '\0')
    {
        return 0.0f;
    }
    ImFont*       f  = pageFont();
    const Float32 px = maxf(9.0f, pt * scale);
    const Float32 w  = f->CalcTextSizeA(px, FLT_MAX, 0.0f, s).x;
    return (scale > 0.0f) ? (w / scale) : w;   // back into page units
}

Void Canvas::rect(Float32 x0, Float32 y0, Float32 x1, Float32 y1, ImU32 col,
                  Float32 rounding, Float32 thickness) const
{
    dl->AddRect(at(x0, y0), at(x1, y1), col, len(rounding), 0,
                maxf(1.0f, len(thickness)));
}

Void Canvas::rectFilled(Float32 x0, Float32 y0, Float32 x1, Float32 y1,
                        ImU32 col, Float32 rounding) const
{
    dl->AddRectFilled(at(x0, y0), at(x1, y1), col, len(rounding));
}

Void Canvas::line(Float32 x0, Float32 y0, Float32 x1, Float32 y1, ImU32 col,
                  Float32 thickness) const
{
    dl->AddLine(at(x0, y0), at(x1, y1), col, maxf(1.0f, len(thickness)));
}

Void Canvas::circle(Float32 x, Float32 y, Float32 r, ImU32 col, Bool filled,
                    Float32 thickness) const
{
    if(filled)
    {
        dl->AddCircleFilled(at(x, y), len(r), col, 0);
    }
    else
    {
        dl->AddCircle(at(x, y), len(r), col, 0, maxf(1.0f, len(thickness)));
    }
}

namespace {

// A heading and a rule, the same on every page.
Float32 heading(const Canvas& c, Float32 x, Float32 y, Float32 w, const Char* s)
{
    c.text(x, y, INK, s, 15.0f);
    c.line(x, y + 12.0f, x + w, y + 12.0f, RULE, 1.0f);
    return y + 26.0f;
}

// A boxed aside. These carry the things that are only ever learned the hard
// way, so they are given a colour of their own and never mixed into body text.
Float32 note(const Canvas& c, Float32 x, Float32 y, Float32 w,
             const Char* const* lines, Int32 n)
{
    const Float32 h = 12.0f + (static_cast<Float32>(n) * 15.0f);
    c.rectFilled(x, y, x + w, y + h, IM_COL32(0x2A, 0x22, 0x12, 0xFF), 3.0f);
    c.line(x, y, x, y + h, C_NOTE, 2.0f);
    for(Int32 i = 0; i < n; ++i)
    {
        c.text(x + 10.0f, y + 13.0f + (static_cast<Float32>(i) * 15.0f),
               (i == 0) ? C_NOTE : INK_DIM, lines[i], 11.0f);
    }
    return y + h + 12.0f;
}

// ======================================================== page: pinout ==

ImU32 roleColour(board::PinRole r, Bool assigned)
{
    switch(r)
    {
    case board::PinRole::PIN_ROLE_GROUND: return C_GROUND;
    case board::PinRole::PIN_ROLE_POWER:  return C_POWER;
    case board::PinRole::PIN_ROLE_SYS:    return C_SYS;
    case board::PinRole::PIN_ROLE_GPIO:   break;
    }
    return assigned ? C_USED : C_GPIO;
}

constexpr Float32 PIN_ROWS   = 20.0f;
constexpr Float32 PIN_PITCH  = 27.0f;
constexpr Float32 BOARD_X0   = 268.0f;
constexpr Float32 BOARD_X1   = 392.0f;
constexpr Float32 BOARD_Y0   = 74.0f;

Void drawPinout(const Canvas& c)
{
    const Float32 y0 = heading(c, 20.0f, 26.0f, 620.0f,
                               "Raspberry Pi Pico 2 W  -  pinout and this car's wiring");
    c.text(20.0f, y0 + 2.0f, INK_DIM,
           "Green is free. Amber is already spoken for on this vehicle.", 11.0f);

    const Float32 by1 = BOARD_Y0 + (PIN_ROWS * PIN_PITCH);

    // The board itself, with the USB end marked - which is the only thing that
    // tells you which way round you are holding it.
    c.rectFilled(BOARD_X0, BOARD_Y0 - 22.0f, BOARD_X1, by1 + 22.0f,
                 IM_COL32(0x1E, 0x22, 0x2A, 0xFF), 6.0f);
    c.rect(BOARD_X0, BOARD_Y0 - 22.0f, BOARD_X1, by1 + 22.0f, RULE, 6.0f, 1.0f);

    const Float32 midX = (BOARD_X0 + BOARD_X1) * 0.5f;
    c.rectFilled(midX - 17.0f, BOARD_Y0 - 30.0f, midX + 17.0f, BOARD_Y0 - 14.0f,
                 IM_COL32(0x8A, 0x8A, 0x90, 0xFF), 2.0f);
    c.text(midX, BOARD_Y0 - 38.0f, INK_DIM, "USB", 10.0f,
           Canvas::Align::ALIGN_CENTRE);
    c.text(midX, by1 + 34.0f, INK_FAINT, "Pico 2 W  /  RP2350", 10.0f,
           Canvas::Align::ALIGN_CENTRE);

    for(Int32 i = 0; i < board::pinCount(); ++i)
    {
        const board::PinRef p = board::pinAt(i);
        const Bool  left  = board::pinOnLeft(p.phys);

        // 1..20 run down the left edge; 21..40 run UP the right, so the right
        // column is indexed from the bottom.
        const Int32 row = left ? (p.phys - 1) : (40 - p.phys);
        const Float32 y = BOARD_Y0 + (static_cast<Float32>(row) * PIN_PITCH)
                        + (PIN_PITCH * 0.5f);

        const Bool  assigned = (p.use != nullptr);
        const ImU32 col      = roleColour(p.role, assigned);

        // The pad, straddling the board edge the way a real header does.
        const Float32 padOut = left ? (BOARD_X0 - 9.0f) : (BOARD_X1 + 9.0f);
        const Float32 padIn  = left ? (BOARD_X0 + 9.0f) : (BOARD_X1 - 9.0f);
        c.rectFilled(minf(padOut, padIn), y - 8.0f, maxf(padOut, padIn), y + 8.0f,
                     col, 2.0f);

        Char num[8];
        std::snprintf(num, sizeof(num), "%d", p.phys);
        c.text((padOut + padIn) * 0.5f, y, PAPER, num, 10.0f,
               Canvas::Align::ALIGN_CENTRE);

        // Name, then the project's use further out. Right column mirrors.
        const Float32 nameX = left ? (BOARD_X0 - 16.0f) : (BOARD_X1 + 16.0f);
        const Canvas::Align nameAlign = left ? Canvas::Align::ALIGN_RIGHT
                                             : Canvas::Align::ALIGN_LEFT;
        c.text(nameX, y, INK, p.name, 12.0f, nameAlign);

        if(assigned)
        {
            const Float32 w  = c.textWidth(p.name, 12.0f);
            const Float32 ux = left ? (nameX - w - 10.0f) : (nameX + w + 10.0f);
            c.text(ux, y, C_USED, p.use, 10.0f, nameAlign);
        }
    }

    // Legend.
    const Float32 ly = by1 + 58.0f;
    struct Swatch
    {
        ImU32       col;
        const Char* what;
    };
    const Swatch SW[] = {
        { C_GPIO,   "free GPIO" },
        { C_USED,   "used by this car" },
        { C_GROUND, "ground" },
        { C_POWER,  "power" },
        { C_SYS,    "system" },
    };
    Float32 lx = 20.0f;
    for(const Swatch& s : SW)
    {
        c.rectFilled(lx, ly - 6.0f, lx + 12.0f, ly + 6.0f, s.col, 2.0f);
        c.text(lx + 18.0f, ly, INK_DIM, s.what, 11.0f);
        lx += 20.0f + c.textWidth(s.what, 11.0f) + 22.0f;
    }

    const Char* const NOTES[] = {
        "The two columns do not both count downward.",
        "Pads 1-20 run DOWN the left edge. 21-40 run UP the right, so 40 is top-right,",
        "not bottom-right. Miscounting this is the classic way to wire a board backwards.",
    };
    note(c, 20.0f, ly + 22.0f, 620.0f, NOTES, 3);
}

// =================================================== page: LED on a pin ==

Void drawLedCircuit(const Canvas& c)
{
    const Float32 y0 = heading(c, 20.0f, 26.0f, 620.0f,
                               "An LED on a GPIO  -  the whole circuit");

    const Float32 midY = y0 + 96.0f;

    // Pico edge.
    c.rectFilled(30.0f, y0 + 20.0f, 108.0f, y0 + 172.0f,
                 IM_COL32(0x1E, 0x22, 0x2A, 0xFF), 5.0f);
    c.rect(30.0f, y0 + 20.0f, 108.0f, y0 + 172.0f, RULE, 5.0f, 1.0f);
    c.text(69.0f, y0 + 34.0f, INK_DIM, "Pico 2 W", 11.0f,
           Canvas::Align::ALIGN_CENTRE);

    c.rectFilled(100.0f, midY - 8.0f, 118.0f, midY + 8.0f, C_GPIO, 2.0f);
    c.text(94.0f, midY, INK, "GP3", 12.0f, Canvas::Align::ALIGN_RIGHT);
    c.text(94.0f, midY + 15.0f, INK_FAINT, "pin 5", 10.0f,
           Canvas::Align::ALIGN_RIGHT);

    const Float32 gndY = y0 + 152.0f;
    c.rectFilled(100.0f, gndY - 8.0f, 118.0f, gndY + 8.0f, C_GROUND, 2.0f);
    c.text(94.0f, gndY, INK, "GND", 12.0f, Canvas::Align::ALIGN_RIGHT);
    c.text(94.0f, gndY + 15.0f, INK_FAINT, "pin 3", 10.0f,
           Canvas::Align::ALIGN_RIGHT);

    // GPIO -> resistor.
    c.line(118.0f, midY, 190.0f, midY, C_WIRE, 2.0f);

    // The resistor, drawn with its bands, because reading them off the part is
    // half the skill and the colours are on the sibling page.
    c.rectFilled(190.0f, midY - 13.0f, 268.0f, midY + 13.0f,
                 IM_COL32(0x9C, 0x82, 0x5A, 0xFF), 4.0f);
    const ImU32 BANDS[] = { IM_COL32(0xC0, 0x39, 0x2B, 0xFF),
                            IM_COL32(0xC0, 0x39, 0x2B, 0xFF),
                            IM_COL32(0x8B, 0x5A, 0x2B, 0xFF) };
    Float32 bx = 204.0f;
    for(ImU32 b : BANDS)
    {
        c.rectFilled(bx, midY - 13.0f, bx + 7.0f, midY + 13.0f, b, 0.0f);
        bx += 15.0f;
    }
    c.text(229.0f, midY - 26.0f, INK, "220R - 1k", 11.0f,
           Canvas::Align::ALIGN_CENTRE);
    c.text(229.0f, midY + 28.0f, INK_FAINT, "red red brown = 220", 10.0f,
           Canvas::Align::ALIGN_CENTRE);

    c.line(268.0f, midY, 340.0f, midY, C_WIRE, 2.0f);

    // The LED: triangle plus bar, with the legs drawn to LENGTH, because the
    // long leg is the entire orientation cue on a real part.
    const Float32 lx = 340.0f;
    c.dl->AddTriangleFilled(c.at(lx, midY - 18.0f), c.at(lx, midY + 18.0f),
                            c.at(lx + 32.0f, midY), IM_COL32(0xD8, 0x5A, 0x4A, 0xFF));
    c.line(lx + 32.0f, midY - 18.0f, lx + 32.0f, midY + 18.0f,
           IM_COL32(0xE8, 0x8A, 0x7A, 0xFF), 3.0f);

    c.text(lx + 16.0f, midY - 30.0f, INK, "LED", 11.0f,
           Canvas::Align::ALIGN_CENTRE);
    c.text(lx - 4.0f, midY + 34.0f, C_GPIO, "long leg (anode, +)", 10.0f,
           Canvas::Align::ALIGN_CENTRE);
    c.text(lx + 74.0f, midY + 34.0f, INK_DIM, "short leg (cathode, -)", 10.0f,
           Canvas::Align::ALIGN_CENTRE);

    // Cathode back down to ground.
    c.line(lx + 32.0f, midY, 470.0f, midY, C_WIRE, 2.0f);
    c.line(470.0f, midY, 470.0f, gndY, C_WIRE, 2.0f);
    c.line(470.0f, gndY, 118.0f, gndY, C_WIRE, 2.0f);

    // Current direction, so the drawing says which way round it all goes.
    c.text(560.0f, midY, INK_FAINT, "current flows", 10.0f,
           Canvas::Align::ALIGN_CENTRE);
    c.text(560.0f, midY + 14.0f, INK_FAINT, "GP3 -> GND", 10.0f,
           Canvas::Align::ALIGN_CENTRE);

    const Char* const NOTES[] = {
        "Backwards is not broken - it is dark, which looks exactly like a dead pin.",
        "An LED only conducts one way. Fitted the wrong way round it is undamaged and",
        "simply never lights, so an LED that does nothing is worth turning round BEFORE",
        "you start suspecting the pin, the sketch or the board.",
    };
    Float32 ny = note(c, 20.0f, y0 + 200.0f, 620.0f, NOTES, 4);

    const Char* const NOTES2[] = {
        "The resistor may sit on either side of the LED.",
        "It limits the current through the loop, and a series loop has one current",
        "everywhere in it, so before or after makes no difference at all. Without one",
        "the LED is close to a short across a 3.3 V pin, which risks both parts.",
    };
    note(c, 20.0f, ny, 620.0f, NOTES2, 4);
}

// ==================================================== page: breadboard ==

Void drawBreadboard(const Canvas& c)
{
    const Float32 y0 = heading(c, 20.0f, 26.0f, 620.0f,
                               "The breadboard  -  what is connected to what");

    const Float32 bx0   = 30.0f;
    const Float32 by0   = y0 + 16.0f;
    const Float32 pitch = 17.0f;

    // 24 columns, not a realistic 30-odd. The width has to leave room for the
    // annotations at the right of the page, and a breadboard drawing that runs
    // under its own labels teaches nothing that a shorter one does not.
    const Int32   cols  = 24;
    const Float32 bx1   = bx0 + (static_cast<Float32>(cols) * pitch) + 10.0f;

    // Tall enough for both banks of five plus the rails: the last row sits at
    // by0 + 253, so anything under 267 puts holes outside the board.
    const Float32 bh = 286.0f;

    c.rectFilled(bx0, by0, bx1, by0 + bh, IM_COL32(0x24, 0x24, 0x26, 0xFF), 4.0f);
    c.rect(bx0, by0, bx1, by0 + bh, RULE, 4.0f, 1.0f);

    const auto holes = [&](Float32 y, Int32 n, ImU32 col)
    {
        for(Int32 i = 0; i < n; ++i)
        {
            c.circle(bx0 + 14.0f + (static_cast<Float32>(i) * pitch), y, 3.2f,
                     col, true);
        }
    };

    // Power rails: one long connection each, the length of the board.
    c.line(bx0 + 10.0f, by0 + 16.0f, bx1 - 10.0f, by0 + 16.0f, C_POWER, 2.0f);
    holes(by0 + 26.0f, cols, IM_COL32(0x50, 0x3A, 0x3A, 0xFF));
    c.line(bx0 + 10.0f, by0 + 46.0f, bx1 - 10.0f, by0 + 46.0f, C_SYS, 2.0f);
    holes(by0 + 36.0f, cols, IM_COL32(0x3A, 0x42, 0x50, 0xFF));
    c.text(bx1 + 12.0f, by0 + 24.0f, C_POWER, "+ rail: joined end to end", 10.0f);
    c.text(bx1 + 12.0f, by0 + 38.0f, C_SYS,   "- rail: joined end to end", 10.0f);

    // Main rows: five holes joined, and the centre channel splits left from
    // right. This is the fact the whole board is built around.
    const Float32 topRows = by0 + 74.0f;
    for(Int32 r = 0; r < 5; ++r)
    {
        holes(topRows + (static_cast<Float32>(r) * pitch), cols,
              IM_COL32(0x44, 0x44, 0x48, 0xFF));
    }
    const Float32 chanY = topRows + (5.0f * pitch) + 6.0f;
    c.rectFilled(bx0 + 6.0f, chanY - 7.0f, bx1 - 6.0f, chanY + 7.0f,
                 IM_COL32(0x18, 0x18, 0x1A, 0xFF), 2.0f);
    const Float32 botRows = chanY + 20.0f;
    for(Int32 r = 0; r < 5; ++r)
    {
        holes(botRows + (static_cast<Float32>(r) * pitch), cols,
              IM_COL32(0x44, 0x44, 0x48, 0xFF));
    }

    // Highlight one column of five, which is the unit of connection.
    const Float32 hx = bx0 + 14.0f + (6.0f * pitch);
    c.rectFilled(hx - 7.0f, topRows - 8.0f, hx + 7.0f, topRows + (4.0f * pitch) + 8.0f,
                 IM_COL32(0x2E, 0x44, 0x2E, 0xFF), 4.0f);
    for(Int32 r = 0; r < 5; ++r)
    {
        c.circle(hx, topRows + (static_cast<Float32>(r) * pitch), 3.2f, C_GPIO, true);
    }
    c.text(hx, topRows - 18.0f, C_GPIO, "these five are one node", 10.0f,
           Canvas::Align::ALIGN_CENTRE);

    c.text(bx1 + 12.0f, chanY - 7.0f, INK_DIM, "the channel splits", 10.0f);
    c.text(bx1 + 12.0f, chanY + 7.0f, INK_DIM, "left from right", 10.0f);
    c.text(bx1 + 12.0f, chanY + 26.0f, INK_FAINT, "a chip straddles it", 10.0f);
    c.text(bx1 + 12.0f, chanY + 39.0f, INK_FAINT, "so its two rows of", 10.0f);
    c.text(bx1 + 12.0f, chanY + 52.0f, INK_FAINT, "pins stay separate", 10.0f);

    const Char* const NOTES[] = {
        "Push components ALL the way in. They are meant to be stiff.",
        "The contacts are sprung metal and they grip hard on purpose - a board whose",
        "parts slid out under their own weight would be useless. A Pico needs real force,",
        "and a part resting on the surface with its legs barely in the hole reads exactly",
        "like a dead pin, a bad sketch or a broken board. Seat it, then debug.",
    };
    note(c, 20.0f, by0 + bh + 16.0f, 620.0f, NOTES, 5);
}

// ========================================================== page: I2C ==

Void drawI2c(const Canvas& c)
{
    const Float32 y0 = heading(c, 20.0f, 26.0f, 620.0f,
                               "I2C  -  five devices, two wires");

    const Float32 sda = y0 + 118.0f;
    const Float32 scl = y0 + 142.0f;

    // Pico.
    c.rectFilled(30.0f, y0 + 74.0f, 118.0f, y0 + 186.0f,
                 IM_COL32(0x1E, 0x22, 0x2A, 0xFF), 5.0f);
    c.rect(30.0f, y0 + 74.0f, 118.0f, y0 + 186.0f, RULE, 5.0f, 1.0f);
    c.text(74.0f, y0 + 88.0f, INK_DIM, "Pico 2 W", 11.0f,
           Canvas::Align::ALIGN_CENTRE);
    c.text(74.0f, y0 + 102.0f, INK_FAINT, "controller", 10.0f,
           Canvas::Align::ALIGN_CENTRE);

    c.rectFilled(110.0f, sda - 7.0f, 126.0f, sda + 7.0f, C_USED, 2.0f);
    c.rectFilled(110.0f, scl - 7.0f, 126.0f, scl + 7.0f, C_USED, 2.0f);
    c.text(104.0f, sda, INK, "GP4", 11.0f, Canvas::Align::ALIGN_RIGHT);
    c.text(104.0f, scl, INK, "GP5", 11.0f, Canvas::Align::ALIGN_RIGHT);

    // The two wires, the length of the page.
    c.line(126.0f, sda, 610.0f, sda, IM_COL32(0xC9, 0xA2, 0x4E, 0xFF), 2.0f);
    c.line(126.0f, scl, 610.0f, scl, IM_COL32(0x7E, 0x9C, 0xC0, 0xFF), 2.0f);
    c.text(614.0f, sda, IM_COL32(0xC9, 0xA2, 0x4E, 0xFF), "SDA", 11.0f);
    c.text(614.0f, scl, IM_COL32(0x7E, 0x9C, 0xC0, 0xFF), "SCL", 11.0f);
    c.text(140.0f, sda - 14.0f, INK_FAINT, "data", 10.0f);
    c.text(140.0f, scl + 14.0f, INK_FAINT, "clock", 10.0f);

    // Pull-ups to 3V3.
    const Float32 pux = 176.0f;
    c.line(pux, y0 + 44.0f, 236.0f, y0 + 44.0f, C_POWER, 2.0f);
    c.text(240.0f, y0 + 44.0f, C_POWER, "3V3", 11.0f);
    for(Int32 i = 0; i < 2; ++i)
    {
        const Float32 x = pux + (static_cast<Float32>(i) * 30.0f);
        const Float32 to = (i == 0) ? sda : scl;
        c.line(x, y0 + 44.0f, x, y0 + 62.0f, C_WIRE, 1.5f);
        c.rectFilled(x - 6.0f, y0 + 62.0f, x + 6.0f, y0 + 92.0f,
                     IM_COL32(0x9C, 0x82, 0x5A, 0xFF), 2.0f);
        c.line(x, y0 + 92.0f, x, to, C_WIRE, 1.5f);
        c.circle(x, to, 3.4f,
                 (i == 0) ? IM_COL32(0xC9, 0xA2, 0x4E, 0xFF)
                          : IM_COL32(0x7E, 0x9C, 0xC0, 0xFF), true);
    }
    c.text(206.0f, y0 + 30.0f, INK_DIM, "pull-ups, ~4.7k", 10.0f,
           Canvas::Align::ALIGN_CENTRE);

    // Stated outright, because the whole diagram is unreadable without it and
    // it is exactly the convention a beginner has not met yet.
    c.text(640.0f, y0 + 30.0f, INK_FAINT,
           "a dot is a join - wires that merely cross do not connect", 10.0f,
           Canvas::Align::ALIGN_RIGHT);

    // Devices hanging off the same pair.
    struct Dev
    {
        Float32     x;
        const Char* what;
        const Char* addr;
    };
    const Dev DEVS[] = {
        { 300.0f, "display",  "0x3C" },
        { 400.0f, "ToF",      "0x29" },
        { 500.0f, "IMU",      "0x68" },
    };
    for(const Dev& d : DEVS)
    {
        c.rectFilled(d.x - 34.0f, scl + 40.0f, d.x + 34.0f, scl + 92.0f,
                     IM_COL32(0x22, 0x26, 0x2C, 0xFF), 4.0f);
        c.rect(d.x - 34.0f, scl + 40.0f, d.x + 34.0f, scl + 92.0f, RULE, 4.0f, 1.0f);
        c.text(d.x, scl + 58.0f, INK, d.what, 11.0f, Canvas::Align::ALIGN_CENTRE);
        c.text(d.x, scl + 76.0f, C_GPIO, d.addr, 11.0f, Canvas::Align::ALIGN_CENTRE);

        // The SDA drop crosses SCL on its way down and does NOT join it. The
        // dot at the top is what says so: junction dot means connected, plain
        // crossing means the wires pass.
        c.line(d.x - 12.0f, sda, d.x - 12.0f, scl + 40.0f,
               IM_COL32(0xC9, 0xA2, 0x4E, 0xFF), 1.5f);
        c.line(d.x + 12.0f, scl, d.x + 12.0f, scl + 40.0f,
               IM_COL32(0x7E, 0x9C, 0xC0, 0xFF), 1.5f);
        c.circle(d.x - 12.0f, sda, 3.4f, IM_COL32(0xC9, 0xA2, 0x4E, 0xFF), true);
        c.circle(d.x + 12.0f, scl, 3.4f, IM_COL32(0x7E, 0x9C, 0xC0, 0xFF), true);
    }

    const Char* const NOTES[] = {
        "Every device shares both wires. The ADDRESS is what picks one out.",
        "So the first thing to write is a scanner: walk 0x08 to 0x77, ping each address,",
        "print the ones that answer. It needs no driver for any particular chip, it tells",
        "you which chips you actually have, and it turns \"is my wiring right\" into a",
        "yes or no - which is the question you cannot otherwise answer.",
    };
    Float32 ny = note(c, 20.0f, scl + 116.0f, 620.0f, NOTES, 5);

    const Char* const NOTES2[] = {
        "Two devices at the same address cannot share a bus.",
        "This is why the four ToF sensors have XSHUT lines on GP10-GP13: they all ship",
        "at 0x29, so you hold three in reset, re-address the fourth, and repeat.",
    };
    note(c, 20.0f, ny, 620.0f, NOTES2, 3);
}

// ============================================== page: resistor colours ==

Void drawResistorCode(const Canvas& c)
{
    const Float32 y0 = heading(c, 20.0f, 26.0f, 620.0f,
                               "Resistor colour code");

    struct Band
    {
        const Char* name;
        ImU32       col;
        ImU32       ink;
        const Char* digit;
        const Char* mult;
    };
    const Band BANDS[] = {
        { "black",  IM_COL32(0x18, 0x18, 0x18, 0xFF), INK,   "0", "x1" },
        { "brown",  IM_COL32(0x6B, 0x42, 0x22, 0xFF), INK,   "1", "x10" },
        { "red",    IM_COL32(0xB0, 0x30, 0x26, 0xFF), INK,   "2", "x100" },
        { "orange", IM_COL32(0xD0, 0x70, 0x20, 0xFF), PAPER, "3", "x1k" },
        { "yellow", IM_COL32(0xD8, 0xC0, 0x30, 0xFF), PAPER, "4", "x10k" },
        { "green",  IM_COL32(0x3C, 0x8A, 0x40, 0xFF), INK,   "5", "x100k" },
        { "blue",   IM_COL32(0x33, 0x5C, 0xA8, 0xFF), INK,   "6", "x1M" },
        { "violet", IM_COL32(0x7A, 0x46, 0xA0, 0xFF), INK,   "7", "x10M" },
        { "grey",   IM_COL32(0x86, 0x86, 0x86, 0xFF), INK,   "8", "-" },
        { "white",  IM_COL32(0xDC, 0xDC, 0xDC, 0xFF), PAPER, "9", "-" },
    };

    Float32 y = y0 + 12.0f;
    c.text(20.0f,  y, INK_DIM, "colour", 10.0f);
    c.text(150.0f, y, INK_DIM, "digit",  10.0f);
    c.text(230.0f, y, INK_DIM, "as multiplier", 10.0f);
    y += 16.0f;

    for(const Band& b : BANDS)
    {
        c.rectFilled(20.0f, y - 9.0f, 132.0f, y + 9.0f, b.col, 3.0f);
        c.rect(20.0f, y - 9.0f, 132.0f, y + 9.0f, RULE, 3.0f, 1.0f);
        c.text(26.0f,  y, b.ink,  b.name,  11.0f);
        c.text(150.0f, y, INK,    b.digit, 11.0f);
        c.text(230.0f, y, INK_DIM, b.mult, 11.0f);
        y += 22.0f;
    }

    // A worked example, since the table alone never quite lands.
    const Float32 ex = 360.0f;
    c.text(ex, y0 + 12.0f, INK, "reading one", 12.0f);

    c.rectFilled(ex, y0 + 40.0f, ex + 200.0f, y0 + 88.0f,
                 IM_COL32(0x9C, 0x82, 0x5A, 0xFF), 5.0f);
    const ImU32 EX_BANDS[] = { IM_COL32(0xB0, 0x30, 0x26, 0xFF),
                               IM_COL32(0xB0, 0x30, 0x26, 0xFF),
                               IM_COL32(0x6B, 0x42, 0x22, 0xFF) };
    Float32 bx = ex + 24.0f;
    for(ImU32 b : EX_BANDS)
    {
        c.rectFilled(bx, y0 + 40.0f, bx + 14.0f, y0 + 88.0f, b, 0.0f);
        bx += 30.0f;
    }
    c.rectFilled(ex + 168.0f, y0 + 40.0f, ex + 180.0f, y0 + 88.0f,
                 IM_COL32(0xC0, 0xA0, 0x50, 0xFF), 0.0f);

    c.text(ex, y0 + 104.0f, INK_DIM, "red  red  brown", 11.0f);
    c.text(ex, y0 + 122.0f, INK,     "2    2    x10   = 220 ohm", 11.0f);
    c.text(ex, y0 + 146.0f, INK_FAINT,
           "The gold band on its own is tolerance,", 10.0f);
    c.text(ex, y0 + 160.0f, INK_FAINT,
           "and it marks the END you read TOWARD.", 10.0f);

    const Char* const NOTES[] = {
        "For an LED on 3.3 V, anything from 220 ohm to 1k is fine.",
        "Higher means dimmer and safer; lower means brighter and hotter. This is not a",
        "precise calculation for indicator LEDs, and treating it as one wastes an evening.",
    };
    note(c, ex - 340.0f, y + 16.0f, 620.0f, NOTES, 3);
}

// ============================================================== table ==
//
// THE place to add a page.
constexpr Page PAGES[] = {
    {
        .category = "Boards",
        .title    = "Pico 2 W pinout",
        .blurb    = "All forty pads, with this car's wiring alongside",
        .natural  = ImVec2(660.0f, 780.0f),
        .draw     = drawPinout,
    },
    {
        .category = "Circuits",
        .title    = "LED on a GPIO",
        .blurb    = "Resistor, polarity, and why backwards looks like broken",
        .natural  = ImVec2(660.0f, 470.0f),
        .draw     = drawLedCircuit,
    },
    {
        .category = "Circuits",
        .title    = "Breadboard",
        .blurb    = "Which holes are joined, and how hard to push",
        .natural  = ImVec2(660.0f, 480.0f),
        .draw     = drawBreadboard,
    },
    {
        .category = "Buses",
        .title    = "I2C",
        .blurb    = "Two wires, many devices, addresses and pull-ups",
        .natural  = ImVec2(660.0f, 540.0f),
        .draw     = drawI2c,
    },
    {
        .category = "Parts",
        .title    = "Resistor colours",
        .blurb    = "The bands, and a worked example",
        .natural  = ImVec2(660.0f, 330.0f),
        .draw     = drawResistorCode,
    },
};

constexpr Int32 PAGE_N = static_cast<Int32>(sizeof(PAGES) / sizeof(PAGES[0]));

Bool matches(const Page& p, const Str& needle)
{
    if(needle.empty())
    {
        return true;
    }

    // Case-insensitive substring over everything the reader can see, so typing
    // "led" finds the page whether the word is in its title or its blurb.
    Str hay = Str(p.category) + " " + p.title + " " + p.blurb;
    for(Char& ch : hay)
    {
        ch = static_cast<Char>(std::tolower(static_cast<UInt8>(ch)));
    }
    Str low = needle;
    for(Char& ch : low)
    {
        ch = static_cast<Char>(std::tolower(static_cast<UInt8>(ch)));
    }
    return hay.find(low) != Str::npos;
}

} // namespace

Int32 pageCount()
{
    return PAGE_N;
}

const Page& page(Int32 i)
{
    return PAGES[(i < 0 || i >= PAGE_N) ? 0 : i];
}

// ================================================================ panel ==

Void draw(State& st, const ImVec2& size)
{
    if(size.x <= 8.0f || size.y <= 8.0f)
    {
        return;
    }

    st.selected = (st.selected < 0 || st.selected >= PAGE_N) ? 0 : st.selected;

    const Float32 SPLIT = 5.0f;
    const Float32 dw    = st.drawerOpen
                        ? clampf(st.drawerW, 120.0f, maxf(140.0f, size.x - 220.0f))
                        : 0.0f;

    // ---- the drawer ------------------------------------------------------
    if(st.drawerOpen)
    {
        ImGui::BeginChild("##refdrawer", ImVec2(dw, size.y), ImGuiChildFlags_None);

        ImGui::SetNextItemWidth(-1.0f);
        Char buf[64];
        std::snprintf(buf, sizeof(buf), "%s", st.filter.c_str());
        if(ImGui::InputTextWithHint("##reffilter", "filter", buf, sizeof(buf)))
        {
            st.filter = buf;
        }

        ImGui::Separator();

        // Grouped by category, and the heading is only emitted when a page
        // under it actually survived the filter - otherwise filtering leaves a
        // column of empty headings, which reads as broken.
        const Char* lastCat = nullptr;
        Int32       shown   = 0;
        for(Int32 i = 0; i < PAGE_N; ++i)
        {
            const Page& p = PAGES[i];
            if(!matches(p, st.filter))
            {
                continue;
            }

            if(lastCat == nullptr || std::strcmp(lastCat, p.category) != 0)
            {
                if(lastCat != nullptr)
                {
                    ImGui::Spacing();
                }
                ImGui::TextDisabled("%s", p.category);
                lastCat = p.category;
            }

            ImGui::PushID(i);
            if(ImGui::Selectable(p.title, i == st.selected))
            {
                st.selected = i;
                st.fitted   = false;   // a new page starts fitted, always
            }
            ImGui::PopID();
            ++shown;
        }

        if(shown == 0)
        {
            ImGui::TextDisabled("  nothing matches");
        }

        ImGui::EndChild();
        ImGui::SameLine(0.0f, 0.0f);

        // ---- splitter ----------------------------------------------------
        ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, RULE);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, RULE);
        ImGui::Button("##refsplit", ImVec2(SPLIT, size.y));
        ImGui::PopStyleColor(3);

        if(ImGui::IsItemActive())
        {
            st.drawerW = clampf(dw + ImGui::GetIO().MouseDelta.x, 120.0f,
                                maxf(140.0f, size.x - 220.0f));
        }
        if(ImGui::IsItemHovered() || ImGui::IsItemActive())
        {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        }

        ImGui::SameLine(0.0f, 0.0f);
    }

    // ---- the viewer ------------------------------------------------------
    const Float32 vw = size.x - dw - (st.drawerOpen ? SPLIT : 0.0f);

    ImGui::BeginChild("##refview", ImVec2(vw, size.y), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar
                      | ImGuiWindowFlags_NoScrollWithMouse);

    const Page&  p  = PAGES[st.selected];
    const ImVec2 o  = ImGui::GetCursorScreenPos();
    const ImVec2 av = ImGui::GetContentRegionAvail();
    ImDrawList*  dl = ImGui::GetWindowDrawList();

    dl->AddRectFilled(o, ImVec2(o.x + av.x, o.y + av.y), PAPER);

    // A header strip that stays put while the page pans under it - the title of
    // what you are looking at should not be something you can scroll away.
    const Float32 hdr = 40.0f;
    dl->AddRectFilled(o, ImVec2(o.x + av.x, o.y + hdr),
                      IM_COL32(0x1B, 0x1C, 0x20, 0xFF));
    dl->AddLine(ImVec2(o.x, o.y + hdr), ImVec2(o.x + av.x, o.y + hdr), RULE, 1.0f);

    {
        ImFont* f = pageFont();
        const Float32 fs = pageFontBase();
        dl->AddText(f, fs * 1.15f, ImVec2(o.x + 12.0f, o.y + 6.0f), INK, p.title);
        dl->AddText(f, fs * 0.92f, ImVec2(o.x + 12.0f, o.y + 22.0f), INK_DIM,
                    p.blurb);
    }

    // Toolbar, right-aligned in the header strip.
    ImGui::SetCursorScreenPos(ImVec2(o.x + av.x - 168.0f, o.y + 8.0f));
    if(ui::iconButton(st.drawerOpen ? ui::Icon::ICON_CLEAR : ui::Icon::ICON_OPEN,
                      st.drawerOpen ? "Hide" : "List", ImVec2(64.0f, 24.0f)))
    {
        st.drawerOpen = !st.drawerOpen;
    }
    ImGui::SameLine(0.0f, 4.0f);
    if(ui::button("Fit", ImVec2(44.0f, 24.0f)))
    {
        st.fitted = false;
    }

    const ImVec2 pageOrigin = ImVec2(o.x, o.y + hdr);
    const ImVec2 pageAvail  = ImVec2(av.x, maxf(1.0f, av.y - hdr));

    // Fit-to-panel, recomputed while the page has not been zoomed. Once it has,
    // the user's zoom is authoritative and a resize must not throw it away.
    const Float32 fit = minf(pageAvail.x / maxf(1.0f, p.natural.x),
                             pageAvail.y / maxf(1.0f, p.natural.y));
    if(!st.fitted)
    {
        st.zoom   = fit;
        st.pan    = ImVec2(0.0f, 0.0f);
        st.fitted = true;
    }

    // Wheel zooms about the cursor, so the thing under the pointer stays under
    // it - the only zoom that does not feel like the page is running away.
    const Bool over = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)
                   && ImGui::GetMousePos().y > pageOrigin.y;
    if(over)
    {
        const Float32 wheel = ImGui::GetIO().MouseWheel;
        if(wheel != 0.0f)
        {
            const Float32 before = st.zoom;
            st.zoom = clampf(st.zoom * std::pow(1.12f, wheel), fit * 0.5f, 6.0f);

            // Keep the point under the cursor under the cursor. With
            //     screen = origin + pan + page * zoom
            // holding `page` fixed across a zoom change of k gives
            //     pan' = (m - origin) - ((m - origin) - pan) * k
            const ImVec2  m = ImGui::GetMousePos();
            const Float32 k = (before > 0.0f) ? (st.zoom / before) : 1.0f;
            st.pan.x = (m.x - pageOrigin.x) - (((m.x - pageOrigin.x) - st.pan.x) * k);
            st.pan.y = (m.y - pageOrigin.y) - (((m.y - pageOrigin.y) - st.pan.y) * k);
        }

        if(ImGui::IsMouseDragging(ImGuiMouseButton_Left, 2.0f))
        {
            const ImVec2 d = ImGui::GetIO().MouseDelta;
            st.pan.x += d.x;
            st.pan.y += d.y;
        }
    }

    // Centre the page when it is smaller than the space, which is the common
    // case at fit and looks wrong pinned to a corner.
    const Float32 drawnW = p.natural.x * st.zoom;
    const Float32 drawnH = p.natural.y * st.zoom;
    const Float32 cx = (drawnW < pageAvail.x) ? ((pageAvail.x - drawnW) * 0.5f) : 0.0f;
    const Float32 cy = (drawnH < pageAvail.y) ? ((pageAvail.y - drawnH) * 0.5f) : 0.0f;

    dl->PushClipRect(pageOrigin,
                     ImVec2(pageOrigin.x + pageAvail.x, pageOrigin.y + pageAvail.y),
                     true);

    Canvas c;
    c.dl     = dl;
    c.scale  = st.zoom;
    c.origin = ImVec2(pageOrigin.x + st.pan.x + cx, pageOrigin.y + st.pan.y + cy);

    if(p.draw != nullptr)
    {
        p.draw(c);
    }

    dl->PopClipRect();

    ImGui::EndChild();
}

} // namespace ref
