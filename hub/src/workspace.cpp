#include "workspace.hpp"

#include <algorithm>

namespace ws {

Float32 clampZoom(Float32 z)
{
    if(z < ZOOM_MIN)
    {
        return ZOOM_MIN;
    }
    if(z > ZOOM_MAX)
    {
        return ZOOM_MAX;
    }
    return z;
}

Rect toScreen(const Rect& r, const Canvas& c, Float32 originX, Float32 originY)
{
    Rect out;
    out.x = originX + r.x * c.zoom + c.panX;
    out.y = originY + r.y * c.zoom + c.panY;
    out.w = r.w * c.zoom;
    out.h = r.h * c.zoom;
    return out;
}

Void toCanvas(Float32 sx, Float32 sy, const Canvas& c,
              Float32 originX, Float32 originY, Float32& cx, Float32& cy)
{
    const Float32 z = (c.zoom > 0.0001f) ? c.zoom : 0.0001f;
    cx = (sx - originX - c.panX) / z;
    cy = (sy - originY - c.panY) / z;
}

Bool contains(const Rect& r, Float32 x, Float32 y)
{
    return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}

Int32 hitTest(const Panel* panels, Int32 count, const Canvas& c,
              Float32 originX, Float32 originY,
              const Float32* screenH, Float32 sx, Float32 sy)
{
    Int32 best  = -1;
    Int32 bestZ = -1;

    for(Int32 i = 0; i < count; ++i)
    {
        if(!panels[i].open)
        {
            continue;
        }

        Rect s = toScreen(panels[i].rect, c, originX, originY);

        // A collapsed panel is only its title bar, so it must only be hit
        // there - otherwise it would keep swallowing clicks meant for whatever
        // is behind the space its body used to occupy.
        if(screenH != nullptr)
        {
            s.h = screenH[i];
        }

        if(contains(s, sx, sy) && panels[i].z > bestZ)
        {
            best  = i;
            bestZ = panels[i].z;
        }
    }
    return best;
}

Void bringToFront(Panel* panels, Int32 count, Int32 idx)
{
    if(idx < 0 || idx >= count)
    {
        return;
    }

    const Int32 was = panels[idx].z;

    // Already there.
    if(was == count - 1)
    {
        return;
    }

    // Everything that was in front of it slides back one, which keeps z a dense
    // 0..N-1 permutation. Simply assigning a big number would work today and
    // drift into overflow over a long session of clicking.
    for(Int32 i = 0; i < count; ++i)
    {
        if(panels[i].z > was)
        {
            --panels[i].z;
        }
    }
    panels[idx].z = count - 1;
}

Void zoomAt(Canvas& c, Float32 factor, Float32 sx, Float32 sy,
            Float32 originX, Float32 originY)
{
    // The canvas point under the cursor before the change...
    Float32 cx = 0.0f;
    Float32 cy = 0.0f;
    toCanvas(sx, sy, c, originX, originY, cx, cy);

    const Float32 before = c.zoom;
    c.zoom = clampZoom(c.zoom * factor);

    if(c.zoom == before)
    {
        return;   // clamped: leave pan alone rather than drifting at the limit
    }

    // ...must still be under it afterwards.
    c.panX = sx - originX - cx * c.zoom;
    c.panY = sy - originY - cy * c.zoom;
}

Void fitAll(const Panel* panels, Int32 count, Canvas& c,
            Float32 viewW, Float32 viewH)
{
    Float32 minX = 0.0f;
    Float32 minY = 0.0f;
    Float32 maxX = 0.0f;
    Float32 maxY = 0.0f;
    Bool    any  = false;

    for(Int32 i = 0; i < count; ++i)
    {
        if(!panels[i].open)
        {
            continue;
        }
        const Rect& r = panels[i].rect;

        if(!any)
        {
            minX = r.x;
            minY = r.y;
            maxX = r.x + r.w;
            maxY = r.y + r.h;
            any  = true;
            continue;
        }
        minX = std::min(minX, r.x);
        minY = std::min(minY, r.y);
        maxX = std::max(maxX, r.x + r.w);
        maxY = std::max(maxY, r.y + r.h);
    }

    if(!any || viewW <= 1.0f || viewH <= 1.0f)
    {
        return;
    }

    const Float32 pad = 24.0f;
    const Float32 w   = std::max(1.0f, maxX - minX + pad * 2.0f);
    const Float32 h   = std::max(1.0f, maxY - minY + pad * 2.0f);

    c.zoom = clampZoom(std::min(viewW / w, viewH / h));

    // Centre what is left over, so a fit that hits the zoom clamp still puts the
    // content in the middle rather than in a corner.
    const Float32 usedW = (maxX - minX) * c.zoom;
    const Float32 usedH = (maxY - minY) * c.zoom;
    c.panX = (viewW - usedW) * 0.5f - minX * c.zoom;
    c.panY = (viewH - usedH) * 0.5f - minY * c.zoom;
}

Void defaultLayout(Panel* panels, Int32 count)
{
    // A readable starting arrangement: the two maps side by side across the top
    // because they are the two most looked at, the recorder under 2D since it IS
    // a 2D view, the editor wide underneath, and the board off to the right.
    //
    // Canvas units are roughly logical pixels at zoom 1.
    static const Rect DEFAULTS[PANEL_COUNT] = {
        {   0.0f,   0.0f, 640.0f, 460.0f },   // 2D
        { 660.0f,   0.0f, 640.0f, 460.0f },   // 3D
        {   0.0f, 480.0f, 640.0f, 400.0f },   // Record
        { 660.0f, 480.0f, 780.0f, 400.0f },   // Code
        { 1320.0f,   0.0f, 460.0f, 460.0f },  // Pico 2 W
    };

    for(Int32 i = 0; i < count && i < PANEL_COUNT; ++i)
    {
        panels[i].rect      = DEFAULTS[i];
        panels[i].open      = true;
        panels[i].collapsed = false;
        panels[i].z         = i;
    }
}

} // namespace ws
