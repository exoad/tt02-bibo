// The floating workspace: the central region as a pannable, zoomable board of
// movable panels, instead of a tab bar showing one view at a time.
//
// ---------------------------------------------------------------------------
// WHY THIS IS HAND-ROLLED AND NOT ImGui DOCKING
//
// The vendored Dear ImGui is master, not the docking branch, so there are no
// dock nodes to build this out of. Real top-level ImGui windows were the other
// candidate and were rejected for a specific reason: they float over the whole
// viewport, cannot be clipped to a sub-region, and cannot be panned or zoomed as
// a GROUP - which is the entire point of a board you arrange things on.
//
// So the panels here are drawn, and their geometry is ours.
//
// ---------------------------------------------------------------------------
// WHAT ZOOM DOES, precisely
//
// It scales panel POSITIONS AND SIZES, not rasterised pixels. Zooming out does
// not shrink the text to mush; it makes each panel physically smaller and lets
// its content re-lay-out into the space, which for this app is exactly right -
// the lidar map re-fits its range, the board view rescales, the editor shows
// fewer columns. Text stays legible at every zoom because it is never scaled.
//
// The honest consequence: at low zoom you fit more panels on screen with less in
// each, rather than seeing the same content smaller. That is a board, not a
// photograph of a board.
//
// ---------------------------------------------------------------------------
// COORDINATES
//
// Panel rects are in CANVAS space, which is independent of zoom, pan and window
// size, and is what gets persisted. Screen space is
//     screen = origin + canvas * zoom + pan
// and nothing stores a screen coordinate between frames.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hpp"

namespace ws {

// One panel per central view, in the same order the tab bar used, so a saved
// layout and a --view argument agree about what "3" means.
enum class Which
{
    WHICH_MAP_2D = 0,
    WHICH_MAP_3D,
    WHICH_RECORD,
    WHICH_CODE,
    WHICH_BOARD,

    // Appended rather than slotted in beside the other static view, so every
    // index above keeps the meaning a saved layout and a --view argument
    // already gave it.
    WHICH_REFERENCE,
    WHICH_RANGE,

    WHICH_COUNT
};

inline constexpr Int32 PANEL_COUNT = static_cast<Int32>(Which::WHICH_COUNT);

// Zoom limits. Below the minimum a panel is too small to aim at; above the
// maximum you are looking at one panel and should be using the tabbed layout,
// which is what it is for.
inline constexpr Float32 ZOOM_MIN  = 0.30f;
inline constexpr Float32 ZOOM_MAX  = 2.50f;
inline constexpr Float32 ZOOM_STEP = 1.10f;   // multiplicative, per wheel notch

// Canvas-space minimums. A panel dragged smaller than this stops being usable
// before it stops being visible, so the limit is about the content and not the
// pixels.
inline constexpr Float32 PANEL_MIN_W = 220.0f;
inline constexpr Float32 PANEL_MIN_H = 140.0f;

struct Rect
{
    Float32 x = 0.0f;
    Float32 y = 0.0f;
    Float32 w = 0.0f;
    Float32 h = 0.0f;
};

struct Panel
{
    Rect  rect;
    Bool  open      = true;
    Bool  collapsed = false;

    // Paint order. Higher is nearer the front; `bringToFront` keeps these a
    // dense 0..N-1 so they can never drift apart over a long session.
    Int32 z = 0;
};

struct Canvas
{
    Float32 zoom = 1.0f;
    Float32 panX = 0.0f;
    Float32 panY = 0.0f;
};

// ---------------------------------------------------------------------------
// Pure geometry. No ImGui, and tested in tests/test_workspace.cpp.
// ---------------------------------------------------------------------------

[[nodiscard]] Float32 clampZoom(Float32 z);

// Canvas rect -> screen rect, given the viewport's top-left.
[[nodiscard]] Rect toScreen(const Rect& r, const Canvas& c,
                            Float32 originX, Float32 originY);

// Screen point -> canvas point. The inverse of the above, and the thing that
// makes zoom-at-the-cursor work.
Void toCanvas(Float32 sx, Float32 sy, const Canvas& c,
              Float32 originX, Float32 originY, Float32& cx, Float32& cy);

[[nodiscard]] Bool contains(const Rect& r, Float32 x, Float32 y);

// Index of the FRONT-MOST open panel whose screen rect contains the point, or
// -1. `heights` supplies each panel's drawn height so a collapsed panel is hit
// on its title bar only.
[[nodiscard]] Int32 hitTest(const Panel* panels, Int32 count, const Canvas& c,
                            Float32 originX, Float32 originY,
                            const Float32* screenH, Float32 sx, Float32 sy);

// Makes `idx` front-most and closes the gap the move leaves behind, so z stays
// a dense permutation. A no-op if it is already there.
Void bringToFront(Panel* panels, Int32 count, Int32 idx);

// Zooms about a fixed screen point, so the thing under the cursor stays under
// the cursor. Doing this by adjusting pan afterwards is the only version that
// feels right; zooming about the origin makes the board slide away from you.
Void zoomAt(Canvas& c, Float32 factor, Float32 sx, Float32 sy,
            Float32 originX, Float32 originY);

// Zoom and pan that bring every open panel into `viewW` x `viewH`. Leaves the
// canvas untouched when nothing is open.
Void fitAll(const Panel* panels, Int32 count, Canvas& c,
            Float32 viewW, Float32 viewH);

// The layout panels start from: a readable arrangement rather than a pile at
// the origin, which is what a default of {0,0} for everything would give.
Void defaultLayout(Panel* panels, Int32 count);

} // namespace ws
