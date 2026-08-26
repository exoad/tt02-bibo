// The floating workspace's geometry.
//
//   tests\build_workspace_test.bat run
//
// These exist because a board you arrange things on is all coordinate maths,
// and the failures are the kind you cannot see in a screenshot:
//
//   * ZOOM MUST BE ABOUT THE CURSOR. Zooming about the origin makes the board
//     slide away from what you were looking at, which feels broken long before
//     anyone can say why.
//   * The z-order must stay a DENSE permutation. Assigning "front" a big number
//     works for a session and drifts; clicking panels for an hour must leave the
//     order exactly as sound as it started.
//   * A COLLAPSED panel must only be hit on its title bar, or it keeps eating
//     clicks meant for whatever sits behind the space its body used to fill.
//
// No hardware, no window. Exits 0 on PASS, 1 on FAIL.

#include "shared.hpp"
#include "workspace.hpp"

#include <cmath>
#include <cstdio>

namespace {

Int32 failures = 0;
Int32 checks   = 0;

Void check(Bool ok, const Char* what)
{
    ++checks;
    if(!ok)
    {
        ++failures;
        std::printf("  FAIL  %s\n", what);
    }
    else
    {
        std::printf("  ok    %s\n", what);
    }
}

Void checkNear(Float32 got, Float32 want, const Char* what)
{
    ++checks;
    if(std::fabs(got - want) > 0.01f)
    {
        ++failures;
        std::printf("  FAIL  %s\n         got  %.3f\n         want %.3f\n",
                    what, static_cast<Float64>(got), static_cast<Float64>(want));
    }
    else
    {
        std::printf("  ok    %s\n", what);
    }
}

Void testTransform()
{
    std::printf("\n-- canvas <-> screen --\n");

    ws::Canvas c;
    c.zoom = 2.0f;
    c.panX = 30.0f;
    c.panY = -10.0f;

    ws::Rect r;
    r.x = 10.0f;
    r.y = 20.0f;
    r.w = 100.0f;
    r.h = 50.0f;

    const ws::Rect s = ws::toScreen(r, c, 5.0f, 7.0f);
    checkNear(s.x, 5.0f + 10.0f * 2.0f + 30.0f, "screen x");
    checkNear(s.y, 7.0f + 20.0f * 2.0f - 10.0f, "screen y");
    checkNear(s.w, 200.0f, "screen w scales with zoom");
    checkNear(s.h, 100.0f, "screen h scales with zoom");

    // Round trip.
    Float32 cx = 0.0f;
    Float32 cy = 0.0f;
    ws::toCanvas(s.x, s.y, c, 5.0f, 7.0f, cx, cy);
    checkNear(cx, r.x, "round trip x");
    checkNear(cy, r.y, "round trip y");
}

Void testZoom()
{
    std::printf("\n-- zoom --\n");

    checkNear(ws::clampZoom(99.0f), ws::ZOOM_MAX, "zoom clamps high");
    checkNear(ws::clampZoom(0.0001f), ws::ZOOM_MIN, "zoom clamps low");

    // The point under the cursor must not move. This is the whole test.
    ws::Canvas c;
    c.zoom = 1.0f;
    c.panX = 0.0f;
    c.panY = 0.0f;

    const Float32 sx = 400.0f;
    const Float32 sy = 300.0f;

    Float32 beforeX = 0.0f;
    Float32 beforeY = 0.0f;
    ws::toCanvas(sx, sy, c, 0.0f, 0.0f, beforeX, beforeY);

    ws::zoomAt(c, 1.10f, sx, sy, 0.0f, 0.0f);

    Float32 afterX = 0.0f;
    Float32 afterY = 0.0f;
    ws::toCanvas(sx, sy, c, 0.0f, 0.0f, afterX, afterY);

    checkNear(afterX, beforeX, "the canvas point under the cursor is unmoved (x)");
    checkNear(afterY, beforeY, "the canvas point under the cursor is unmoved (y)");
    checkNear(c.zoom, 1.10f, "zoom applied");

    // Repeated zoom out must land exactly on the clamp and then stop moving.
    for(Int32 i = 0; i < 60; ++i)
    {
        ws::zoomAt(c, 1.0f / 1.10f, sx, sy, 0.0f, 0.0f);
    }
    checkNear(c.zoom, ws::ZOOM_MIN, "repeated zoom out reaches the floor");

    const Float32 pinnedX = c.panX;
    ws::zoomAt(c, 1.0f / 1.10f, sx, sy, 0.0f, 0.0f);
    checkNear(c.panX, pinnedX, "at the clamp, pan stops drifting");
}

Void testZOrder()
{
    std::printf("\n-- z order --\n");

    ws::Panel p[ws::PANEL_COUNT];
    ws::defaultLayout(p, ws::PANEL_COUNT);

    check(p[0].z == 0 && p[4].z == 4, "default layout gives a dense order");

    ws::bringToFront(p, ws::PANEL_COUNT, 0);
    check(p[0].z == ws::PANEL_COUNT - 1, "the raised panel is front-most");

    // Dense permutation: every z in 0..N-1 exactly once.
    auto dense = [&p]()
    {
        Bool seen[ws::PANEL_COUNT] = {};
        for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
        {
            if(p[i].z < 0 || p[i].z >= ws::PANEL_COUNT)
            {
                return false;
            }
            if(seen[p[i].z])
            {
                return false;
            }
            seen[p[i].z] = true;
        }
        return true;
    };
    check(dense(), "still a dense permutation after one raise");

    // Hammer it: this is the drift the dense scheme exists to prevent.
    for(Int32 i = 0; i < 500; ++i)
    {
        ws::bringToFront(p, ws::PANEL_COUNT, i % ws::PANEL_COUNT);
    }
    check(dense(), "still dense after 500 raises");

    // Raising the front-most is a no-op, not a shuffle.
    const Int32 front = ws::PANEL_COUNT - 1;
    Int32 which = -1;
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        if(p[i].z == front)
        {
            which = i;
        }
    }
    Int32 before[ws::PANEL_COUNT];
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        before[i] = p[i].z;
    }
    ws::bringToFront(p, ws::PANEL_COUNT, which);
    Bool same = true;
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        if(before[i] != p[i].z)
        {
            same = false;
        }
    }
    check(same, "raising the front-most panel changes nothing");
}

Void testHitTest()
{
    std::printf("\n-- hit testing --\n");

    ws::Panel p[ws::PANEL_COUNT];
    ws::defaultLayout(p, ws::PANEL_COUNT);

    ws::Canvas c;   // identity

    // A point inside panel 0 only.
    Int32 hit = ws::hitTest(p, ws::PANEL_COUNT, c, 0.0f, 0.0f, nullptr, 10.0f, 10.0f);
    check(hit == 0, "point inside the first panel hits it");

    // Empty canvas.
    hit = ws::hitTest(p, ws::PANEL_COUNT, c, 0.0f, 0.0f, nullptr, -50.0f, -50.0f);
    check(hit == -1, "a point outside every panel hits nothing");

    // Overlap: the front-most wins.
    p[1].rect = p[0].rect;             // exactly on top of panel 0
    ws::bringToFront(p, ws::PANEL_COUNT, 1);
    hit = ws::hitTest(p, ws::PANEL_COUNT, c, 0.0f, 0.0f, nullptr, 10.0f, 10.0f);
    check(hit == 1, "overlapping panels resolve to the front-most");

    ws::bringToFront(p, ws::PANEL_COUNT, 0);
    hit = ws::hitTest(p, ws::PANEL_COUNT, c, 0.0f, 0.0f, nullptr, 10.0f, 10.0f);
    check(hit == 0, "and follow the order when it changes");

    // A closed panel is not hit at all.
    p[0].open = false;
    hit = ws::hitTest(p, ws::PANEL_COUNT, c, 0.0f, 0.0f, nullptr, 10.0f, 10.0f);
    check(hit == 1, "a closed panel is skipped");
    p[0].open = true;

    // A collapsed panel is hit on its title bar only.
    ws::defaultLayout(p, ws::PANEL_COUNT);
    Float32 heights[ws::PANEL_COUNT];
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        heights[i] = p[i].rect.h;
    }
    heights[0] = 24.0f;                // panel 0 collapsed to its bar

    hit = ws::hitTest(p, ws::PANEL_COUNT, c, 0.0f, 0.0f, heights, 10.0f, 10.0f);
    check(hit == 0, "a collapsed panel is still hit on its bar");

    hit = ws::hitTest(p, ws::PANEL_COUNT, c, 0.0f, 0.0f, heights, 10.0f, 200.0f);
    check(hit == -1, "and NOT hit below it, where its body used to be");
}

Void testFit()
{
    std::printf("\n-- fit --\n");

    ws::Panel p[ws::PANEL_COUNT];
    ws::defaultLayout(p, ws::PANEL_COUNT);

    ws::Canvas c;
    ws::fitAll(p, ws::PANEL_COUNT, c, 1200.0f, 700.0f);

    check(c.zoom >= ws::ZOOM_MIN && c.zoom <= ws::ZOOM_MAX, "fit stays within the clamps");

    // Every open panel must land inside the viewport, with a little slack for
    // the padding fit() reserves.
    Bool allIn = true;
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        const ws::Rect s = ws::toScreen(p[i].rect, c, 0.0f, 0.0f);
        if(s.x < -1.0f || s.y < -1.0f
           || s.x + s.w > 1201.0f || s.y + s.h > 701.0f)
        {
            allIn = false;
        }
    }
    check(allIn, "every panel lands inside the viewport");

    // Nothing open: the canvas must be left alone rather than reset to junk.
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        p[i].open = false;
    }
    ws::Canvas keep = c;
    ws::fitAll(p, ws::PANEL_COUNT, c, 1200.0f, 700.0f);
    check(c.zoom == keep.zoom && c.panX == keep.panX,
          "fitting nothing leaves the canvas untouched");

    // A degenerate viewport must not divide by zero.
    ws::defaultLayout(p, ws::PANEL_COUNT);
    keep = c;
    ws::fitAll(p, ws::PANEL_COUNT, c, 0.0f, 0.0f);
    check(c.zoom == keep.zoom, "a zero-size viewport is ignored");
}

} // namespace

int main()
{
    std::printf("workspace geometry tests\n");

    testTransform();
    testZoom();
    testZOrder();
    testHitTest();
    testFit();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
