// A browsable library of reference diagrams: pinouts, wiring recipes, bus
// topologies - the things you otherwise keep a browser tab open for.
//
// ---------------------------------------------------------------------------
// WHY THE DIAGRAMS ARE DRAWN AND NOT IMAGES
//
// A screenshot of a pinout is the easy version and the wrong one. It cannot be
// searched, it goes blurry the moment the panel is resized or zoomed, it has to
// be shipped as a binary asset with someone else's licence attached, and - the
// one that actually matters - it cannot show THIS project's pin assignments
// beside the physical pins. A drawing compiled from the same facts as
// docs/wiring.md cannot disagree with it.
//
// So every page here is vector, drawn into an ImDrawList at whatever size it is
// given, and legible at all of them.
//
// ---------------------------------------------------------------------------
// ADDING A PAGE
//
// Write a function taking a Canvas, add one entry to the table in reference.cpp.
// That is the whole procedure; nothing else in the app needs to know. Pages are
// deliberately plain functions rather than a class hierarchy, because a diagram
// has no state and never will.
//
// Draw in page coordinates - the units in Page::natural - and let Canvas do the
// mapping. A page never sees a screen coordinate or the zoom level.
#pragma once

#include "shared.hpp"

#include "imgui.h"

namespace ref {

// A drawing surface in PAGE coordinates, which are fixed and independent of the
// panel size, the zoom and the DPI. `at` maps them to the screen; `len` scales a
// distance. Text is given a page-space size too, so a diagram scales as one
// thing rather than drifting apart from its own labels.
struct Canvas
{
    ImDrawList* dl     = nullptr;
    ImVec2      origin;
    Float32     scale  = 1.0f;

    [[nodiscard]] ImVec2 at(Float32 x, Float32 y) const
    {
        return ImVec2(origin.x + (x * scale), origin.y + (y * scale));
    }

    [[nodiscard]] Float32 len(Float32 v) const
    {
        return v * scale;
    }

    // Where the text sits relative to (x, y): the left edge, the centre or the
    // right edge. Pinouts are two columns facing each other and half of every
    // diagram here wants to be right-aligned, so this is not a luxury.
    enum class Align
    {
        ALIGN_LEFT = 0,
        ALIGN_CENTRE,
        ALIGN_RIGHT
    };

    // `pt` is a height in page units. Clamped to stay readable when a page is
    // fitted into a small panel - a diagram whose labels have become illegible
    // is worse than one that has stopped being to scale.
    Void text(Float32 x, Float32 y, ImU32 col, const Char* s, Float32 pt = 11.0f,
              Align align = Align::ALIGN_LEFT) const;

    // Width the string would occupy, in PAGE units, so a caller can size a box
    // around it without leaving page space.
    [[nodiscard]] Float32 textWidth(const Char* s, Float32 pt = 11.0f) const;

    Void rect(Float32 x0, Float32 y0, Float32 x1, Float32 y1, ImU32 col,
              Float32 rounding = 0.0f, Float32 thickness = 1.0f) const;

    Void rectFilled(Float32 x0, Float32 y0, Float32 x1, Float32 y1, ImU32 col,
                    Float32 rounding = 0.0f) const;

    Void line(Float32 x0, Float32 y0, Float32 x1, Float32 y1, ImU32 col,
              Float32 thickness = 1.0f) const;

    Void circle(Float32 x, Float32 y, Float32 r, ImU32 col, Bool filled = true,
                Float32 thickness = 1.0f) const;
};

// One page in the library.
struct Page
{
    const Char* category = nullptr;   // groups the drawer list
    const Char* title    = nullptr;
    const Char* blurb    = nullptr;   // one line, under the title in the header
    ImVec2      natural;              // page size in page units
    Void      (*draw)(const Canvas& c) = nullptr;
};

[[nodiscard]] Int32       pageCount();
[[nodiscard]] const Page& page(Int32 i);

// Everything the panel remembers between frames. Held by the caller so the
// panel is re-entrant and two of them could show different pages.
struct State
{
    Int32   selected   = 0;
    Bool    drawerOpen = true;
    Float32 drawerW    = 186.0f;

    // Fit-to-panel until the user zooms, then whatever they set. `fitted` is
    // how the difference is remembered: a page that has been zoomed must not
    // silently re-fit itself the next time the panel is resized.
    Bool    fitted     = false;
    Float32 zoom       = 1.0f;
    ImVec2  pan;

    Str     filter;
};

// Draws the whole panel - drawer, splitter and viewer - into `size`.
Void draw(State& st, const ImVec2& size);

} // namespace ref
