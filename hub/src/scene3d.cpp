#include "scene3d.hpp"

#include "icons.hpp"
#include "scene_gpu.hpp"
#include "vehicle.hpp"
#include "theme.hpp"

// NOMINMAX before windows.h, or its min/max MACROS shadow std::min/std::max and
// every call to them becomes a syntax error. Only needed here because this file
// is the one part of the renderer that touches the filesystem.
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace scene3d {

namespace {

constexpr Float32 PI_F = 3.14159265358979323846f;

// The device's own limits, repeated here rather than reached for across a
// header: the 3D view discards exactly what the flat map discards, and if that
// ever stops being true it should be a visible edit in both places.
constexpr Float32 MIN_VALID_MM = 50.0f;
constexpr Float32 MAX_VALID_MM = 12000.0f;

// The car and the sensor, from vehicle.hpp - one definition for the whole app.
constexpr Float32 EGO_LEN_MM       = vehicle::CAR_LEN_MM;
constexpr Float32 EGO_WID_MM       = vehicle::CAR_WID_MM;
constexpr Float32 EGO_HEIGHT_MM    = vehicle::CAR_HEIGHT_MM;
constexpr Float32 EGO_WHEELBASE_MM = vehicle::CAR_WHEELBASE_MM;
constexpr Float32 EGO_TREAD_MM     = vehicle::CAR_TREAD_MM;
constexpr Float32 EGO_WHEEL_D_MM   = vehicle::CAR_TYRE_DIA_MM;
constexpr Float32 EGO_WHEEL_W_MM   = vehicle::CAR_TYRE_WID_MM;
constexpr Float32 EGO_SENSOR_AHEAD_MM = vehicle::C1_MOUNT_AHEAD_MM;

// How tall a return stands: up to the SCAN PLANE, and no further.
//
// The C1 is a planar scanner. It measures one horizontal slice and knows
// nothing whatsoever about height, so every column is the same height - varying
// it by range or by quality would be inventing a third dimension out of a
// two-dimensional instrument, which is the one thing a 3D view of a 2D sensor
// must not do.
//
// But the height is no longer ARBITRARY. It was 90 mm, chosen because it looked
// right. It is now the height of the plane the beam actually sweeps, so the top
// of every column is a real measured surface and the sensor drawn on the car
// sits exactly level with it. That is one fewer invented number, and it makes
// the picture self-consistent: you can see the slice the device is taking.
constexpr Float32 COLUMN_MM   = vehicle::C1_SCAN_Z_MM;
constexpr Float32 COLUMN_W_MM = 46.0f;
constexpr Float32 WALL_H_MM   = 260.0f;

// ---------------------------------------------------------------------------
// Vector helpers
// ---------------------------------------------------------------------------
Vec3 sub(const Vec3& a, const Vec3& b) { return Vec3{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z }; }
Vec3 add(const Vec3& a, const Vec3& b) { return Vec3{ a.x + b.x, a.y + b.y, a.z + b.z }; }
Vec3 mul(const Vec3& a, Float32 k)     { return Vec3{ a.x * k, a.y * k, a.z * k }; }
Float32 dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x };
}

Vec3 norm(const Vec3& a)
{
    const Float32 l = std::sqrt(dot(a, a));
    return (l > 1e-6f) ? mul(a, 1.0f / l) : Vec3{ 0.0f, 1.0f, 0.0f };
}

// ---------------------------------------------------------------------------
// The projection, built once per frame.
//
// Two forms of the same camera: an MVP matrix for the GPU, and the basis
// vectors, because billboarding a line and sizing it in PIXELS both need to
// know where the eye is and how wide a pixel is at a given depth.
// ---------------------------------------------------------------------------
struct View
{
    Vec3    eye, right, up, fwd;
    ImVec2  p0, p1;              // the widget rect, screen space
    Float32 focal  = 1.0f;       // pixels per unit at unit depth
    Float32 nearMm = 40.0f;
    Float32 farMm  = 60000.0f;
    Float32 mvp[16] = {};

    [[nodiscard]] Float32 depthOf(const Vec3& p) const
    {
        return dot(sub(p, eye), fwd);
    }

    // Half-width, in WORLD units, of something that should be `px` pixels wide
    // where it sits. Without this every line would be a fixed physical width and
    // the far side of the grid would vanish.
    [[nodiscard]] Float32 pxToWorld(const Vec3& at, Float32 px) const
    {
        const Float32 d = depthOf(at);
        return (d > nearMm) ? (px * d / focal) : (px * nearMm / focal);
    }

    // Screen position, for the few things that are still 2D - labels, and the
    // HUD drawn over the composited image.
    Bool project(const Vec3& p, ImVec2& out) const
    {
        const Float32 x = p.x * mvp[0] + p.y * mvp[4] + p.z * mvp[8]  + mvp[12];
        const Float32 y = p.x * mvp[1] + p.y * mvp[5] + p.z * mvp[9]  + mvp[13];
        const Float32 w = p.x * mvp[3] + p.y * mvp[7] + p.z * mvp[11] + mvp[15];
        if(w <= 1e-4f)
            return false;
        out = ImVec2(p0.x + (x / w * 0.5f + 0.5f) * (p1.x - p0.x),
                     p0.y + (0.5f - y / w * 0.5f) * (p1.y - p0.y));
        return true;
    }
};

View makeView(const Camera& c, const ImVec2& p0, const ImVec2& p1,
              Float32 worldYawDeg)
{
    View v;
    const Float32 cp = std::cos(c.pitch), sp = std::sin(c.pitch);
    v.eye = add(c.target, Vec3{ c.dist * cp * std::sin(c.yaw),
                               -c.dist * cp * std::cos(c.yaw),
                                c.dist * sp });

    v.fwd   = norm(sub(c.target, v.eye));
    v.right = norm(cross(v.fwd, Vec3{ 0.0f, 0.0f, 1.0f }));
    v.up    = cross(v.right, v.fwd);

    v.p0 = p0;
    v.p1 = p1;

    const Float32 w = std::max(1.0f, p1.x - p0.x);
    const Float32 h = std::max(1.0f, p1.y - p0.y);
    v.focal = (h * 0.5f) / std::tan(c.fovY * 0.5f);

    // Row-vector, left-handed, depth in [0,1] - what D3D wants and what
    // scene_gpu's shader assumes.
    const Float32 ys = 1.0f / std::tan(c.fovY * 0.5f);
    const Float32 xs = ys * (h / w);
    const Float32 zn = v.nearMm, zf = v.farMm;
    const Float32 q  = zf / (zf - zn);

    const Float32 view[16] = {
        v.right.x, v.up.x, v.fwd.x, 0.0f,
        v.right.y, v.up.y, v.fwd.y, 0.0f,
        v.right.z, v.up.z, v.fwd.z, 0.0f,
        -dot(v.eye, v.right), -dot(v.eye, v.up), -dot(v.eye, v.fwd), 1.0f,
    };
    const Float32 proj[16] = {
        xs,   0.0f, 0.0f,     0.0f,
        0.0f, ys,   0.0f,     0.0f,
        0.0f, 0.0f, q,        1.0f,
        0.0f, 0.0f, -zn * q,  0.0f,
    };

    // Rotation about the vertical, applied BEFORE the view - a model matrix for
    // the whole scene. See DrawArgs::worldYawDeg.
    //
    // Sign, derived rather than guessed: a return at bearing b lands at
    // (sin b, cos b), so bearing 0 is +y and bearing 90 is +x - increasing
    // bearing sweeps forward toward right, which is CLOCKWISE seen from above
    // and therefore a NEGATIVE rotation about +z. When the sensor turns and a
    // fixed feature's measured bearing rises by d, the feature has moved by -d
    // about +z, so undoing it - putting the world back where it was - is +d.
    const Float32 wy = worldYawDeg * (PI_F / 180.0f);
    const Float32 cw = std::cos(wy), sw = std::sin(wy);
    const Float32 model[16] = {
         cw,   sw,   0.0f, 0.0f,
        -sw,   cw,   0.0f, 0.0f,
         0.0f, 0.0f, 1.0f, 0.0f,
         0.0f, 0.0f, 0.0f, 1.0f,
    };

    Float32 mv[16] = {};
    for(Int32 r = 0; r < 4; ++r)
        for(Int32 col = 0; col < 4; ++col)
        {
            Float32 sum = 0.0f;
            for(Int32 k = 0; k < 4; ++k)
                sum += model[r * 4 + k] * view[k * 4 + col];
            mv[r * 4 + col] = sum;
        }

    for(Int32 r = 0; r < 4; ++r)
        for(Int32 col = 0; col < 4; ++col)
        {
            Float32 sum = 0.0f;
            for(Int32 k = 0; k < 4; ++k)
                sum += mv[r * 4 + k] * proj[k * 4 + col];
            v.mvp[r * 4 + col] = sum;
        }

    // The eye and the basis stay in SCENE space, unrotated: they are what
    // billboarding and pixel-width sizing are measured against, and those are
    // properties of the camera rather than of the frame the scene is drawn in.
    return v;
}

// ---------------------------------------------------------------------------
// Emission.
//
// These keep the names and shapes the geometry code already called, so every
// mode below is unchanged by the move to a depth buffer - what changed is that
// they now hand WORLD-space triangles to the GPU instead of screen-space
// polygons to a sorted queue.
//
// Opaque or blended is decided by the fill's own alpha rather than by a flag at
// every call site: a translucent colour IS the statement that this surface is
// see-through, and having to say it twice is a way to say it inconsistently.
// ---------------------------------------------------------------------------
scenegpu::Vertex vtx(const Vec3& p, ImU32 col, const ImVec2& uv)
{
    scenegpu::Vertex v;
    v.x = p.x; v.y = p.y; v.z = p.z;
    v.col = col;
    v.u = uv.x; v.v = uv.y;
    return v;
}

Void emitTri(const Vec3& a, const Vec3& b, const Vec3& c, ImU32 col,
             const ImVec2* uv, ImTextureID tex)
{
    const ImVec2 z(0.5f, 0.5f);
    const scenegpu::Vertex va = vtx(a, col, uv ? uv[0] : z);
    const scenegpu::Vertex vb = vtx(b, col, uv ? uv[1] : z);
    const scenegpu::Vertex vc = vtx(c, col, uv ? uv[2] : z);

    if((col >> IM_COL32_A_SHIFT) >= 0xFFu)
        scenegpu::addOpaque(va, vb, vc, tex);
    else
        scenegpu::addBlended(va, vb, vc, tex);
}

// The same, with a colour per vertex. A gradient needs one: a triangle painted
// one flat colour cannot fade, and a ring of them fading in steps reads as
// wedges - which is exactly how the ride view's ground first came out.
Void emitTriC(const Vec3& a, const Vec3& b, const Vec3& c,
              ImU32 ca, ImU32 cb, ImU32 cc)
{
    const ImVec2 z(0.5f, 0.5f);
    const scenegpu::Vertex va = vtx(a, ca, z);
    const scenegpu::Vertex vb = vtx(b, cb, z);
    const scenegpu::Vertex vc = vtx(c, cc, z);

    // Blended if ANY vertex is: the triangle is see-through somewhere.
    if((ca >> IM_COL32_A_SHIFT) >= 0xFFu && (cb >> IM_COL32_A_SHIFT) >= 0xFFu
       && (cc >> IM_COL32_A_SHIFT) >= 0xFFu)
        scenegpu::addOpaque(va, vb, vc, 0);
    else
        scenegpu::addBlended(va, vb, vc, 0);
}

// A camera-facing quad along a segment, `px` pixels wide wherever it is. This is
// how every line in the scene is drawn now: a real line primitive would be one
// pixel wide regardless of DPI, and would not be depth-tested against the solid
// geometry the way a triangle is.
Void ribbon(const View& v, const Vec3& a, const Vec3& b, ImU32 col, Float32 px)
{
    const Vec3 dir = sub(b, a);
    if(dot(dir, dir) < 1e-6f)
        return;

    const Vec3 mid{ (a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f, (a.z + b.z) * 0.5f };
    Vec3 side = cross(norm(dir), norm(sub(v.eye, mid)));
    if(dot(side, side) < 1e-9f)
        return;                                  // the line points at the eye
    side = mul(norm(side), v.pxToWorld(mid, px) * 0.5f);

    const Vec3 q0 = sub(a, side), q1 = add(a, side);
    const Vec3 q2 = add(b, side), q3 = sub(b, side);
    emitTri(q0, q1, q2, col, nullptr, 0);
    emitTri(q0, q2, q3, col, nullptr, 0);
}

// The old signature, kept so the geometry code did not have to change. `dl` is
// unused: nothing in the scene draws to the ImGui list any more.
Void line3(ImDrawList* dl, const View& v, const Vec3& a, const Vec3& b, ImU32 col,
           Float32 w)
{
    static_cast<Void>(dl);
    ribbon(v, a, b, col, w);
}

// A convex face, optionally outlined. The outline is nudged toward the eye by a
// fraction of its depth so it cannot z-fight with the face it belongs to.
Void pushFace(const View& v, const Vec3* w, Int32 n, ImU32 fill, ImU32 edge,
              Float32 edgeW, ImTextureID tex = 0, const ImVec2* uv = nullptr)
{
    if(n < 3 || n > 4)
        return;

    if((fill >> IM_COL32_A_SHIFT) != 0u)
    {
        for(Int32 i = 1; i + 1 < n; ++i)
        {
            const ImVec2 tuv[3] = { uv ? uv[0] : ImVec2(0.5f, 0.5f),
                                    uv ? uv[i] : ImVec2(0.5f, 0.5f),
                                    uv ? uv[i + 1] : ImVec2(0.5f, 0.5f) };
            emitTri(w[0], w[i], w[i + 1], fill, uv ? tuv : nullptr, tex);
        }
    }

    if(edgeW <= 0.0f || (edge >> IM_COL32_A_SHIFT) == 0u)
        return;

    const auto lift = [&](const Vec3& p) {
        const Vec3 toEye = norm(sub(v.eye, p));
        return add(p, mul(toEye, v.pxToWorld(p, 1.0f) * 1.5f));
    };

    for(Int32 i = 0; i < n; ++i)
    {
        const Vec3 a2 = lift(w[i]);
        const Vec3 b2 = lift(w[(i + 1) % n]);
        ribbon(v, a2, b2, edge, edgeW);
    }
}

// ---------------------------------------------------------------------------
// A box standing on the ground, centred on (x,y) in scene axes.
// ---------------------------------------------------------------------------
Void column(const View& v, Float32 x, Float32 y, Float32 half, Float32 h,
            ImU32 side, ImU32 top, ImU32 edge, Float32 edgeW)
{
    const Float32 x0 = x - half, x1 = x + half;
    const Float32 y0 = y - half, y1 = y + half;

    const Vec3 b00{ x0, y0, 0.0f }, b10{ x1, y0, 0.0f };
    const Vec3 b11{ x1, y1, 0.0f }, b01{ x0, y1, 0.0f };
    const Vec3 t00{ x0, y0, h },    t10{ x1, y0, h };
    const Vec3 t11{ x1, y1, h },    t01{ x0, y1, h };

    // All four sides. Picking the two facing the camera was a painter's-algorithm
    // economy; the depth buffer hides the far ones for free and correctly, and
    // guessing which two are visible is exactly the kind of per-primitive
    // visibility decision this renderer stopped making.
    const Vec3 sN[4] = { b01, b11, t11, t01 };
    const Vec3 sS[4] = { b00, b10, t10, t00 };
    const Vec3 sE[4] = { b10, b11, t11, t10 };
    const Vec3 sW[4] = { b00, b01, t01, t00 };

    pushFace(v, sN, 4, side, edge, edgeW);
    pushFace(v, sS, 4, side, edge, edgeW);
    pushFace(v, sE, 4, side, edge, edgeW);
    pushFace(v, sW, 4, side, edge, edgeW);

    const Vec3 tp[4] = { t00, t10, t11, t01 };
    pushFace(v, tp, 4, top, edge, edgeW);
}

// ---------------------------------------------------------------------------
// The ground: range rings and radials, on z = 0.
// ---------------------------------------------------------------------------
// The ground grid follows the MODE, the same way the flat map's does: rings
// where the quantity is a range and a bearing, squares where it is a length.
// A wall panel judged against concentric circles is a straight edge argued with
// by curves.
Bool sceneWantsSquares(SceneMode m)
{
    return m == SceneMode::SCENE_MODE_WALLS;
}

Void drawGroundSquares(ImDrawList* dl, const View& v, Float32 dpi)
{
    constexpr Float32 STEP = 1000.0f;      // one metre
    constexpr Int32   N    = 12;
    const Float32 far2 = STEP * N;

    for(Int32 i = -N; i <= N; ++i)
    {
        const Float32 t = static_cast<Float32>(i) * STEP;
        const Bool major = (i % 5) == 0;
        const ImU32 col = (i == 0)
            ? ((ui::ansi::GRID_MAJOR & 0x00FFFFFFu) | (0xB0u << IM_COL32_A_SHIFT))
            : (major ? ((ui::ansi::GRID_MAJOR & 0x00FFFFFFu) | (0x80u << IM_COL32_A_SHIFT))
                     : ((ui::ansi::GRID       & 0x00FFFFFFu) | (0xB0u << IM_COL32_A_SHIFT)));
        const Float32 w = ((i == 0) ? 1.6f : (major ? 1.3f : 1.0f)) * dpi;

        line3(dl, v, Vec3{ t, -far2, 0.0f }, Vec3{ t, far2, 0.0f }, col, w);
        line3(dl, v, Vec3{ -far2, t, 0.0f }, Vec3{ far2, t, 0.0f }, col, w);
    }

    // The heading, in its own colour, so which way is forward survives an orbit.
    line3(dl, v, Vec3{ 0.0f, 0.0f, 0.0f }, Vec3{ 0.0f, far2, 0.0f },
          (ui::ansi::BRCYAN & 0x00FFFFFFu) | (0x99u << IM_COL32_A_SHIFT), 1.6f * dpi);
}

// ---------------------------------------------------------------------------
// SCENE_MODE_FULL - the ride view.
//
// Modelled on the display an autonomous car shows its PASSENGERS, which is a
// different thing from the displays above it. Those are instruments: grids to
// measure against, a number per mode, a colour per meaning. This one is a
// picture of the situation, and the design rules that follow from that are:
//
//   * No wireframe. A grid of lines is how you read a measurement off a plot;
//     it is not how you show somebody where the car is.
//   * Almost no text.
//   * Few colours. A near-white for real things, one accent for intent, and a
//     dark ground. Waymo's screen is essentially two colours and it is legible
//     from the back seat.
//   * Soft, rounded, matte. Sharp wireframe boxes read as debug output.
//
// THE ONE THING NOT COPIED: Waymo draws little pedestrians, cyclists and cars,
// because Waymo has a classifier that earns those shapes. A planar lidar with
// no classifier cannot tell a chair leg from an ankle, so every detection here
// is the same plain box. Drawing a person because something is person-sized
// would be the display inventing a fact.
// ---------------------------------------------------------------------------
// Uniform, and modest. The C1 measures one horizontal slice, so a box's
// height is a drawing convention either way - and a convention near the car's
// own 135 mm reads as "things about this big", where 380 mm read as a wall.
constexpr Float32 RIDE_BOX_H_MM = 240.0f;

// The floor: a filled disc that fades out with range, no lines. Its edge is
// where the sensor stops being able to see, which is a real boundary and the
// only one this view draws.
Void drawRideGround(const View& v, const DrawArgs& a)
{
    // Everything here is world-space and emitted straight through; the view is
    // only in the signature so this reads like its neighbours.
    static_cast<Void>(v);
    constexpr Int32   SEG   = 64;
    constexpr Float32 R_IN  = 1400.0f;
    constexpr Float32 R_MID = 5000.0f;
    constexpr Float32 R_OUT = 11000.0f;

    const ImU32 c0 = IM_COL32(0x1E, 0x26, 0x33, 0xFF);
    const ImU32 c1 = IM_COL32(0x12, 0x18, 0x22, 0xFF);
    const ImU32 c2 = IM_COL32(0x08, 0x0B, 0x10, 0x00);

    // Inner colour on the inner edge, outer on the outer, interpolated across
    // each triangle. Per-VERTEX, not per-triangle: colouring whole triangles
    // gives a ring of flat wedges, which is what this looked like first time.
    const auto ring = [&](Float32 r0, Float32 r1, ImU32 a0, ImU32 a1) {
        Vec3 p0{ r0, 0.0f, 0.0f }, p1{ r1, 0.0f, 0.0f };
        for(Int32 i = 1; i <= SEG; ++i)
        {
            const Float32 t = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
            const Vec3 q0{ r0 * std::cos(t), r0 * std::sin(t), 0.0f };
            const Vec3 q1{ r1 * std::cos(t), r1 * std::sin(t), 0.0f };
            emitTriC(p0, q0, q1, a0, a0, a1);
            emitTriC(p0, q1, p1, a0, a1, a1);
            p0 = q0; p1 = q1;
        }
    };

    // The disc under the car is its own, slightly lighter, so the car is always
    // standing on something even in an empty room.
    Vec3 prev{ R_IN, 0.0f, 0.0f };
    for(Int32 i = 1; i <= SEG; ++i)
    {
        const Float32 t = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
        const Vec3 cur{ R_IN * std::cos(t), R_IN * std::sin(t), 0.0f };
        emitTriC(Vec3{ 0.0f, 0.0f, 0.0f }, prev, cur, c0, c0, c0);
        prev = cur;
    }
    ring(R_IN,  R_MID, c0, c1);
    ring(R_MID, R_OUT, c1, c2);

    static_cast<Void>(a);
}

// A detection, as a soft chamfered box.
//
// Chamfered rather than square: eight sides in plan instead of four costs four
// triangles and is the whole difference between "a rounded object" and "a debug
// AABB". Nothing else about it is rounded, and nothing else needs to be.
Void drawDetection(const View& v, const Detection& d, Float32 dpi)
{
    const Float32 px = -d.uy, py = d.ux;
    const Float32 hl = std::max(d.halfL, 40.0f);
    const Float32 hw = std::max(d.halfW, 40.0f);

    // The chamfer, capped so a small object is not chamfered into a diamond.
    const Float32 k = std::min(std::min(hl, hw) * 0.45f, 90.0f);

    // Eight corners, going round the plan.
    const Float32 ol[8] = {  hl - k,  hl,      hl,     hl - k,
                            -hl + k, -hl,     -hl,    -hl + k };
    const Float32 ow[8] = { -hw,     -hw + k,  hw - k, hw,
                             hw,      hw - k, -hw + k, -hw };

    Vec3 lo[8], hi[8];
    for(Int32 i = 0; i < 8; ++i)
    {
        const Float32 x = d.cx + d.ux * ol[i] + px * ow[i];
        const Float32 y = d.cy + d.uy * ol[i] + py * ow[i];
        lo[i] = toScene(x, y, 0.0f);
        hi[i] = toScene(x, y, RIDE_BOX_H_MM);
    }

    // In the car's way, or merely present. Two states, because two is what this
    // display can honestly distinguish.
    const ImU32 side = d.inPath ? IM_COL32(0xE8, 0x7A, 0x5A, 0x5A)
                                : IM_COL32(0xC8, 0xD4, 0xE2, 0x3E);
    const ImU32 top  = d.inPath ? IM_COL32(0xFF, 0x9A, 0x74, 0xD8)
                                : IM_COL32(0xE8, 0xF0, 0xFA, 0xC0);

    for(Int32 i = 0; i < 8; ++i)
    {
        const Int32 j = (i + 1) % 8;
        const Vec3 q[4] = { lo[i], lo[j], hi[j], hi[i] };
        pushFace(v, q, 4, side, 0u, 0.0f);
    }

    const Vec3 mid{ toScene(d.cx, d.cy, RIDE_BOX_H_MM) };
    for(Int32 i = 0; i < 8; ++i)
        emitTri(mid, hi[i], hi[(i + 1) % 8], top, nullptr, 0);

    static_cast<Void>(dpi);
}

// Where the car would go. The one saturated colour on the screen, because it is
// the one thing here that is an INTENTION rather than an observation.
Void drawPathRibbon(const View& v, const DrawArgs& a)
{
    static_cast<Void>(v);
    if(a.corridorHalfW <= 0.0f || a.corridorFree <= 0.0f)
        return;

    const Float32 y0 = EGO_LEN_MM * 0.5f;
    const Float32 y1 = std::max(y0 + 60.0f, a.corridorFree);
    const Float32 hw = a.corridorHalfW;

    constexpr Int32 STEPS = 14;
    for(Int32 i = 0; i < STEPS; ++i)
    {
        const Float32 t0 = static_cast<Float32>(i)     / STEPS;
        const Float32 t1 = static_cast<Float32>(i + 1) / STEPS;
        const Float32 ya = y0 + (y1 - y0) * t0;
        const Float32 yb = y0 + (y1 - y0) * t1;

        // Fades out along its length: the far end of a planned path is less
        // certain than the near end, and the ribbon should not pretend
        // otherwise.
        const UInt32 aa = static_cast<UInt32>(0xB0u * (1.0f - t0 * 0.75f));
        const UInt32 ab = static_cast<UInt32>(0xB0u * (1.0f - t1 * 0.75f));
        const ImU32 ca = IM_COL32(0x35, 0xC8, 0xE8, aa);
        const ImU32 cb = IM_COL32(0x35, 0xC8, 0xE8, ab);

        const Vec3 p0{ -hw, ya, 6.0f }, p1{ hw, ya, 6.0f };
        const Vec3 p2{  hw, yb, 6.0f }, p3{ -hw, yb, 6.0f };
        emitTri(p0, p1, p2, ca, nullptr, 0);
        emitTri(p0, p2, p3, cb, nullptr, 0);
    }
}

Void drawGround(ImDrawList* dl, const View& v, Float32 dpi)
{
    constexpr Int32 SEG = 72;

    for(Int32 r = 1; r <= 12; ++r)
    {
        const Float32 rad = static_cast<Float32>(r) * 1000.0f;
        const Bool major = (r % 5) == 0;
        const ImU32 col = major ? (ui::ansi::GRID_MAJOR & 0x00FFFFFFu) | (0x88u << IM_COL32_A_SHIFT)
                                : (ui::ansi::GRID       & 0x00FFFFFFu) | (0xAAu << IM_COL32_A_SHIFT);

        Vec3 prev{ rad, 0.0f, 0.0f };
        for(Int32 i = 1; i <= SEG; ++i)
        {
            const Float32 a = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
            const Vec3 cur{ rad * std::cos(a), rad * std::sin(a), 0.0f };
            line3(dl, v, prev, cur, col, (major ? 1.4f : 1.0f) * dpi);
            prev = cur;
        }
    }

    // Radials every 45 deg, and the forward axis in the heading colour so the
    // car's facing survives being orbited behind.
    for(Int32 b = 0; b < 360; b += 45)
    {
        const Float32 a = static_cast<Float32>(b) * (PI_F / 180.0f);
        const Bool fwd = (b == 90);      // +y is forward
        line3(dl, v, Vec3{ 0.0f, 0.0f, 0.0f },
              Vec3{ 12000.0f * std::cos(a), 12000.0f * std::sin(a), 0.0f },
              fwd ? ((ui::ansi::BRCYAN & 0x00FFFFFFu) | (0x99u << IM_COL32_A_SHIFT))
                  : ((ui::ansi::AXIS   & 0x00FFFFFFu) | (0xFFu << IM_COL32_A_SHIFT)),
              (fwd ? 1.6f : 1.0f) * dpi);
    }
}

// ---------------------------------------------------------------------------
// The car model.
//
// assets/models/car.obj - "sedan-sports" from Kenney's Car Kit, CC0. Downloaded
// rather than modelled: a hand-lofted shell got the proportions of a touring car
// roughly right and never looked like one, and there is no reason to keep
// approximating a thing that exists under a public-domain licence.
//
// The loader is deliberately small. It reads `v`, `g` and triangular `f`, which
// is everything this file contains (2088 faces, all triangles, one material) and
// is not a general OBJ parser - anything it does not understand it skips, so a
// different model would degrade to a partial mesh rather than to garbage.
//
// Colour comes from the GROUP NAMES, not from the material: the kit textures
// everything from one atlas, and a texture would be the wrong answer here
// anyway. This is a schematic, and `body` / `wheel-*` / `spoiler` is exactly the
// distinction a schematic wants.
// ---------------------------------------------------------------------------
enum CarPart { CAR_PART_BODY = 0, CAR_PART_WHEEL, CAR_PART_SPOILER, CAR_PART_COUNT };

struct CarTri
{
    Vec3    a, b, c;
    Vec3    n;
    ImVec2  ta, tb, tc;         // texture coordinates
    Int32   part = CAR_PART_BODY;
};

struct CarModel
{
    Vec<CarTri> tris;
    ImTextureID         tex = 0;
    Bool loaded = false;
};

// Both go through ui::assetPath, so the model follows the same two-layout rule
// the icon atlas does and a copied exe keeps its car.
Bool carModelPath(Char* out, Size cap)
{
    return ui::assetPath("models\\car.obj", out, cap);
}

Bool carTexturePath(Char* out, Size cap)
{
    return ui::assetPath("models\\colormap.png", out, cap);
}

Int32 partForGroup(const Char* g)
{
    if(std::strstr(g, "wheel")   != nullptr) return CAR_PART_WHEEL;
    if(std::strstr(g, "spoiler") != nullptr) return CAR_PART_SPOILER;
    return CAR_PART_BODY;
}

Bool loadCarObj(CarModel& m)
{
    m.tris.clear();
    m.loaded = false;

    Char path[MAX_PATH];
    if(!carModelPath(path, sizeof(path)))
        return false;

    FILE* f = std::fopen(path, "rb");
    if(f == nullptr)
        return false;

    Vec<Vec3> raw;
    raw.reserve(1400);

    Vec<ImVec2> uvs;
    uvs.reserve(1400);

    struct Face { Int32 i0, i1, i2; Int32 t0, t1, t2; Int32 part; };
    Vec<Face> faces;
    faces.reserve(2200);

    Int32 part = CAR_PART_BODY;
    Char  line[512];

    while(std::fgets(line, sizeof(line), f) != nullptr)
    {
        if(line[0] == 'v' && line[1] == ' ')
        {
            // `v x y z [r g b]` - the kit writes vertex colours, all white, and
            // they are ignored.
            Float32 x = 0.0f, y = 0.0f, z = 0.0f;
            if(std::sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3)
                raw.push_back(Vec3{ x, y, z });
        }
        else if(line[0] == 'v' && line[1] == 't' && line[2] == ' ')
        {
            // OBJ's V axis runs up from the bottom; every graphics API this app
            // touches runs it down from the top, so it is flipped once here
            // rather than at every use.
            Float32 u = 0.0f, vv = 0.0f;
            if(std::sscanf(line + 3, "%f %f", &u, &vv) == 2)
                uvs.push_back(ImVec2(u, 1.0f - vv));
        }
        else if(line[0] == 'g' && line[1] == ' ')
        {
            Char name[128] = {};
            if(std::sscanf(line + 2, "%127s", name) == 1)
                part = partForGroup(name);
        }
        else if(line[0] == 'f' && line[1] == ' ')
        {
            // `f a/ta/na b/tb/nb c/tc/nc`. Only the position index is wanted,
            // and only triangles appear in this file - a polygon would be
            // skipped rather than mis-triangulated.
            Int32 idx[3] = { 0, 0, 0 };
            Int32 tex[3] = { 0, 0, 0 };
            Int32 got = 0;
            const Char* p = line + 2;
            while(got < 3)
            {
                while(*p == ' ') ++p;
                if(*p == 0 || *p == '\n' || *p == '\r')
                    break;

                idx[got] = std::atoi(p);

                // `a/b/c` - b is the texture index, and may be absent (`a//c`).
                const Char* slash = p;
                while(*slash != 0 && *slash != ' ' && *slash != '\n'
                      && *slash != '\r' && *slash != '/') ++slash;
                if(*slash == '/' && slash[1] != '/')
                    tex[got] = std::atoi(slash + 1);

                ++got;
                while(*p != 0 && *p != ' ' && *p != '\n' && *p != '\r') ++p;
            }
            if(got == 3)
                faces.push_back(Face{ idx[0], idx[1], idx[2],
                                      tex[0], tex[1], tex[2], part });
        }
    }
    std::fclose(f);

    if(raw.empty() || faces.empty())
        return false;

    // ---- fit to the real chassis -----------------------------------------
    //
    // PER AXIS, not uniform, and that is a deliberate distortion. The model's
    // proportions are a generic sports saloon; the numbers the rest of this app
    // uses - the corridor width, the Fit erosion, the footprint on the flat map
    // - are the TT-02's actual 430 x 190 x 135. Scaling uniformly would leave
    // the car ~15% wider than the corridor drawn alongside it, and a picture
    // that disagrees with the measurement beside it is worse than a picture with
    // a slightly wrong roofline.
    Float32 lo[3] = {  1e9f,  1e9f,  1e9f };
    Float32 hi[3] = { -1e9f, -1e9f, -1e9f };
    for(const Vec3& v : raw)
    {
        const Float32 c[3] = { v.x, v.y, v.z };
        for(Int32 k = 0; k < 3; ++k)
        {
            if(c[k] < lo[k]) lo[k] = c[k];
            if(c[k] > hi[k]) hi[k] = c[k];
        }
    }

    const Float32 ex = std::max(hi[0] - lo[0], 1e-4f);   // model x = width
    const Float32 ey = std::max(hi[1] - lo[1], 1e-4f);   // model y = height (up)
    const Float32 ez = std::max(hi[2] - lo[2], 1e-4f);   // model z = length

    const Float32 sx = EGO_WID_MM    / ex;
    const Float32 sy = EGO_HEIGHT_MM / ey;
    const Float32 sz = EGO_LEN_MM    / ez;

    const Float32 cx = (lo[0] + hi[0]) * 0.5f;
    const Float32 cz = (lo[2] + hi[2]) * 0.5f;

    // Model axes are x=right, y=UP, z=LENGTH with +z forward (checked against
    // the wheel groups: wheel-front-* sit at +z). Scene axes are x=right,
    // y=forward, z=up.
    //
    // Swapping two axes is a REFLECTION - it flips triangle winding, and the
    // back-face test below depends on winding. Negating x as well puts the
    // determinant back to +1, and mirroring a symmetric car left-to-right is
    // invisible.
    const auto toScene3 = [&](const Vec3& v) {
        return Vec3{ -(v.x - cx) * sx,
                      (v.z - cz) * sz,
                      (v.y - lo[1]) * sy };
    };

    const Int32 nRaw = static_cast<Int32>(raw.size());
    m.tris.reserve(faces.size());

    for(const Face& fc : faces)
    {
        // OBJ indices are 1-based, and negative means "counting back from the
        // end". Both forms are resolved here; out-of-range drops the face.
        const auto resolve = [&](Int32 i) -> Int32 {
            if(i > 0)  return i - 1;
            if(i < 0)  return nRaw + i;
            return -1;
        };

        const Int32 i0 = resolve(fc.i0), i1 = resolve(fc.i1), i2 = resolve(fc.i2);
        if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= nRaw || i1 >= nRaw || i2 >= nRaw)
            continue;

        CarTri t;
        t.a = toScene3(raw[static_cast<Size>(i0)]);
        t.b = toScene3(raw[static_cast<Size>(i1)]);
        t.c = toScene3(raw[static_cast<Size>(i2)]);
        t.n = norm(cross(sub(t.b, t.a), sub(t.c, t.a)));
        t.part = fc.part;

        const Int32 nUv = static_cast<Int32>(uvs.size());
        const auto uvAt = [&](Int32 i) -> ImVec2 {
            const Int32 k = (i > 0) ? i - 1 : (i < 0 ? nUv + i : -1);
            return (k >= 0 && k < nUv) ? uvs[static_cast<Size>(k)] : ImVec2(0.5f, 0.5f);
        };
        t.ta = uvAt(fc.t0);
        t.tb = uvAt(fc.t1);
        t.tc = uvAt(fc.t2);

        m.tris.push_back(t);
    }

    m.loaded = !m.tris.empty();
    return m.loaded;
}

const CarModel& carModel()
{
    static CarModel m;
    static Bool tried = false;
    if(!tried)
    {
        tried = true;
        static_cast<Void>(loadCarObj(m));
    }

    // The texture is loaded lazily and retried until it succeeds, because the
    // D3D device is not up when the first frame's model load happens. Once it
    // is non-zero this costs one comparison.
    if(m.loaded && m.tex == 0)
    {
        Char tp[MAX_PATH];
        if(carTexturePath(tp, sizeof(tp)))
            m.tex = ui::loadTexture(ui::device(), tp);
    }
    return m;
}

// Flat-shaded, back-face culled. No per-triangle outline: at 2088 faces an
// outline is a wireframe, and the shading is what carries the form.
Void drawCarModel(const View& v, const CarModel& m, Float32 dpi)
{
    static_cast<Void>(dpi);

    // Above, ahead and to one side. A single fixed light in WORLD space, not
    // camera space, so orbiting moves the highlight across the body the way it
    // would on a real object rather than pinning it to the screen.
    const Vec3 LIGHT = norm(Vec3{ 0.38f, 0.42f, 0.82f });

    // The contact shadow. Nothing sells "this object is sitting on that floor"
    // like a soft dark patch under it, and without one the car appears to hover
    // - which on a display whose whole subject is where things are on the
    // ground is the wrong impression to give.
    {
        const Float32 rx = EGO_WID_MM * 0.62f;
        const Float32 ry = EGO_LEN_MM * 0.56f;
        constexpr Int32 SEG = 24;

        Vec3 prev{ rx, 0.0f, 2.0f };
        for(Int32 i = 1; i <= SEG; ++i)
        {
            const Float32 t = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
            const Vec3 cur{ rx * std::cos(t), ry * std::sin(t), 2.0f };
            const Vec3 tri[3] = { Vec3{ 0.0f, 0.0f, 2.0f }, prev, cur };
            emitTri(tri[0], tri[1], tri[2], IM_COL32(0, 0, 0, 0x66), nullptr, 0);
            prev = cur;
        }
    }

    const ImU32 BASE[CAR_PART_COUNT] = {
        IM_COL32(0x4A, 0x58, 0x68, 0xFF),   // body
        IM_COL32(0x1C, 0x1E, 0x21, 0xFF),   // wheels
        IM_COL32(0x30, 0x3A, 0x46, 0xFF),   // spoiler
    };

    for(const CarTri& t : m.tris)
    {
        // No back-face rejection. The depth buffer resolves it per pixel, and
        // trusting a downloaded mesh's winding was one bug waiting to happen for
        // no gain at 2088 triangles.
        // A stylised material rather than one lambert term, because the point
        // of this view is that it reads as an OBJECT at a glance:
        //
        //   key  a directional light. |n.l|, not max(0, n.l) - with both faces
        //        of the shell reachable, a triangle whose winding happens to
        //        point inward would otherwise render black.
        //   sky  a hemispheric term. Surfaces facing up are lit by the sky and
        //        surfaces facing down are not; this is what stops a flat-shaded
        //        model looking like folded paper.
        //   rim  a fresnel edge. Grazing faces catch light, which separates the
        //        silhouette from a dark background without an outline.
        const Vec3 toEye = norm(sub(v.eye, Vec3{
            (t.a.x + t.b.x + t.c.x) / 3.0f,
            (t.a.y + t.b.y + t.c.y) / 3.0f,
            (t.a.z + t.b.z + t.c.z) / 3.0f }));

        const Float32 key = std::fabs(dot(t.n, LIGHT));
        const Float32 sky = 0.5f + 0.5f * t.n.z;
        const Float32 rim = 1.0f - std::fabs(dot(t.n, toEye));

        Float32 lit = 0.26f + 0.52f * key + 0.26f * sky + 0.22f * rim * rim;
        if(lit > 1.35f) lit = 1.35f;

        // With the atlas bound, the vertex colour is a LIGHT level, not a
        // paint: white x lit, so the texture's own colours come through.
        const ImU32 base = (m.tex != 0) ? IM_COL32(0xFF, 0xFF, 0xFF, 0xFF)
                                        : BASE[t.part];
        const auto ch = [&](Int32 shift) {
            const Float32 x = static_cast<Float32>((base >> shift) & 0xFFu) * lit;
            return static_cast<Int32>(x > 255.0f ? 255.0f : x);
        };
        const ImU32 col = IM_COL32(ch(IM_COL32_R_SHIFT), ch(IM_COL32_G_SHIFT),
                                   ch(IM_COL32_B_SHIFT), 0xFF);

        const Vec3   tri[3] = { t.a, t.b, t.c };
        const ImVec2 uv[3]  = { t.ta, t.tb, t.tc };

        // Textured when the atlas is there, shaded-flat when it is not. The
        // shading multiplies the SAMPLED colour either way - the tint is passed
        // as the vertex colour, which ImGui multiplies into the texture - so the
        // car keeps its form in both cases rather than turning into a flat
        // sticker the moment a texture appears.
        pushFace(v, tri, 3, col, 0u, 0.0f, m.tex, m.tex != 0 ? uv : nullptr);
    }
}

// ---------------------------------------------------------------------------
// The car.
//
// Hand-authored from the TT-02's own dimensions, not a scanned or CAD model:
// there is no TT-02 mesh in this repository and I am not going to fetch one, so
// what this is - a lofted low-poly touring shell built to the kit's wheelbase,
// tread and overall size - is stated rather than implied.
//
// Built as a series of CROSS-SECTIONS along the length, lofted together. That is
// how a car body actually varies: the bonnet is low and narrow, the screen rakes
// up to a roof that is inset from the shoulders, the tail drops away again. An
// extruded plan outline - which is what this was - cannot express any of that,
// and read as a wedge from every angle.
//
// Heights are fractions of EGO_HEIGHT_MM, widths of the half-width. Front is +y.
// ---------------------------------------------------------------------------
struct Station
{
    Float32 y;      // along the car, fraction of half-length, +1 = nose
    Float32 wl;     // half-width at the shoulder line
    Float32 wu;     // half-width at the roof line
    Float32 zl;     // sill height
    Float32 zu;     // roof height
};

// Nine stations. The shape of a 1/10 touring shell: a short low nose, a bonnet
// rising to the cowl, a raked screen, a roof over the middle, a fastback rear.
constexpr Station STATIONS[9] = {
    {  1.00f, 0.34f, 0.28f, 0.05f, 0.26f },   // nose
    {  0.86f, 0.66f, 0.54f, 0.05f, 0.38f },
    {  0.66f, 0.90f, 0.72f, 0.05f, 0.44f },   // bonnet
    {  0.40f, 1.00f, 0.80f, 0.05f, 0.50f },   // cowl - base of the screen
    {  0.16f, 0.97f, 0.64f, 0.05f, 0.94f },   // roof, front
    { -0.24f, 0.95f, 0.62f, 0.05f, 1.00f },   // roof, rear
    { -0.52f, 1.00f, 0.76f, 0.05f, 0.66f },   // base of the rear screen
    { -0.80f, 0.99f, 0.82f, 0.05f, 0.54f },   // boot
    { -1.00f, 0.78f, 0.64f, 0.06f, 0.48f },   // tail
};

constexpr Int32 STATION_N = 9;
constexpr Int32 SECTION_N = 6;    // points around one cross-section

// One cross-section, as a closed loop of six points. Six is the fewest that can
// carry a shoulder: bottom, shoulder, roof - mirrored. Four would make every
// section a trapezoid and lose the tumblehome that says "car".
Void sectionLoop(const Station& s, Float32 hw, Float32 hl, Float32 hz, Vec3* out)
{
    const Float32 y  = s.y * hl;
    const Float32 zl = s.zl * hz;
    const Float32 zu = s.zu * hz;
    const Float32 zm = zl + (zu - zl) * 0.55f;      // the shoulder

    out[0] = Vec3{ -s.wl * hw, y, zl };
    out[1] = Vec3{ -s.wl * hw, y, zm };
    out[2] = Vec3{ -s.wu * hw, y, zu };
    out[3] = Vec3{  s.wu * hw, y, zu };
    out[4] = Vec3{  s.wl * hw, y, zm };
    out[5] = Vec3{  s.wl * hw, y, zl };
}

// A wheel: a cylinder about the x axis, which is how a wheel is actually
// oriented. Ten segments - enough to read as round at any zoom this view
// reaches, and 4 wheels x 10 is still only 40 faces.
Void wheel(const View& v, Float32 cx, Float32 cy, Float32 r, Float32 halfW,
           ImU32 tread, ImU32 rim, Float32 dpi)
{
    constexpr Int32 SEG = 10;

    Vec3 in[SEG], out[SEG];
    for(Int32 i = 0; i < SEG; ++i)
    {
        const Float32 a = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
        const Float32 y = cy + r * std::cos(a);
        const Float32 z = r  + r * std::sin(a);      // sits ON the ground
        in[i]  = Vec3{ cx - halfW, y, z };
        out[i] = Vec3{ cx + halfW, y, z };
    }

    for(Int32 i = 0; i < SEG; ++i)
    {
        const Int32 j = (i + 1) % SEG;
        const Vec3 q[4] = { in[i], in[j], out[j], out[i] };
        pushFace(v, q, 4, tread, 0u, 0.0f);
    }

    // The outboard face. The inboard one is inside the bodywork and the depth
    // buffer would discard it anyway, so it is simply not generated.
    const Vec3* side = (cx > 0.0f) ? out : in;
    const Vec3 hub{ (cx > 0.0f) ? cx + halfW : cx - halfW, cy, r };
    for(Int32 i = 0; i < SEG; ++i)
    {
        const Int32 j = (i + 1) % SEG;
        const Vec3 t[3] = { hub, side[i], side[j] };
        pushFace(v, t, 3, rim, 0u, 0.0f);
    }

    // The rim's edge, so a wheel has an outline like everything else.
    for(Int32 i = 0; i < SEG; ++i)
    {
        const Int32 j = (i + 1) % SEG;
        const Vec3 e[3] = { side[i], side[j], side[j] };
        pushFace(v, e, 3, 0u, IM_COL32(0xB0, 0xB4, 0xB8, 0xC0), 1.0f * dpi);
    }
}

// The RPLIDAR C1, to scale: a 55.6 mm square base with the rotating head on
// top, 41.3 mm to the crown. Small - about an eighth of the car's length - and
// that smallness is the point of the mode.
// ---------------------------------------------------------------------------
// Lamps.
//
// Positioned from the car's own dimensions rather than from the mesh: the
// downloaded model has no addressable lamp geometry, and hunting for it in a
// generic saloon would tie the lighting to one particular OBJ.
//
// Every lamp is drawn twice - a dark lens that is always there, and a lit face
// on top scaled by brightness. That is how a real lamp looks: an unlit lamp is
// not invisible, it is a dark glass rectangle, and drawing nothing when a lamp
// is off makes the car change shape every time it blinks.
// ---------------------------------------------------------------------------
Void drawLamp(const View& v, Float32 x, Float32 y, Float32 z,
              Float32 halfW, Float32 halfH, Bool facingFront,
              ImU32 hue, Float32 level)
{
    // Pushed a hair proud of the bodywork so the depth buffer cannot z-fight
    // the lens against the panel it is set into.
    const Float32 out = facingFront ? 1.5f : -1.5f;

    const Vec3 q[4] = {
        Vec3{ x - halfW, y + out, z - halfH },
        Vec3{ x + halfW, y + out, z - halfH },
        Vec3{ x + halfW, y + out, z + halfH },
        Vec3{ x - halfW, y + out, z + halfH },
    };

    // The lens: always drawn, always dark.
    pushFace(v, q, 4, IM_COL32(0x14, 0x16, 0x1A, 0xFF), 0u, 0.0f);

    if(level <= 0.01f)
        return;

    const Int32 r = static_cast<Int32>(((hue >> IM_COL32_R_SHIFT) & 0xFFu) * level);
    const Int32 g = static_cast<Int32>(((hue >> IM_COL32_G_SHIFT) & 0xFFu) * level);
    const Int32 b = static_cast<Int32>(((hue >> IM_COL32_B_SHIFT) & 0xFFu) * level);

    const Vec3 lit[4] = {
        Vec3{ x - halfW, y + out * 1.6f, z - halfH },
        Vec3{ x + halfW, y + out * 1.6f, z - halfH },
        Vec3{ x + halfW, y + out * 1.6f, z + halfH },
        Vec3{ x - halfW, y + out * 1.6f, z + halfH },
    };
    pushFace(v, lit, 4, IM_COL32(r, g, b, 0xFF), 0u, 0.0f);

    // A soft bloom in front of the lens, so a lit lamp reads at a distance the
    // way a real one does - by spilling, not by being a brighter rectangle.
    const Float32 gw = halfW * 2.1f, gh = halfH * 2.4f;
    const Vec3 glow[4] = {
        Vec3{ x - gw, y + out * 2.2f, z - gh },
        Vec3{ x + gw, y + out * 2.2f, z - gh },
        Vec3{ x + gw, y + out * 2.2f, z + gh },
        Vec3{ x - gw, y + out * 2.2f, z + gh },
    };
    const Int32 ga = static_cast<Int32>(70.0f * level);
    pushFace(v, glow, 4, IM_COL32(r, g, b, ga), 0u, 0.0f);
}

// The whole cluster set, front and rear.
//
// The rear layout follows the note in docs/conventions.md: the Impreza's cluster carries
// a red main lamp, an amber indicator, and the reverse lamp NESTED INSIDE the
// indicator housing - so the reverse lamp is drawn small and inboard of the
// amber, not as a fourth unit in a row.
Void drawLamps(const View& v, const lights::Lamps& L)
{
    const Float32 hw = EGO_WID_MM * 0.5f;
    const Float32 hl = EGO_LEN_MM * 0.5f;
    const Float32 hz = EGO_HEIGHT_MM;

    const ImU32 WHITE = IM_COL32(0xFF, 0xF4, 0xD8, 0xFF);
    const ImU32 AMBER = IM_COL32(0xFF, 0xA8, 0x18, 0xFF);
    const ImU32 RED   = IM_COL32(0xFF, 0x2A, 0x1E, 0xFF);

    // ---- front ----
    const Float32 fz = hz * 0.30f;
    drawLamp(v, -hw * 0.52f, hl, fz, 20.0f, 9.0f, true, WHITE, L.headL);
    drawLamp(v,  hw * 0.52f, hl, fz, 20.0f, 9.0f, true, WHITE, L.headR);
    drawLamp(v, -hw * 0.82f, hl, fz, 9.0f,  7.0f, true, AMBER, L.indFL);
    drawLamp(v,  hw * 0.82f, hl, fz, 9.0f,  7.0f, true, AMBER, L.indFR);

    // ---- rear ----
    const Float32 rz = hz * 0.42f;
    drawLamp(v, -hw * 0.50f, -hl, rz, 18.0f, 10.0f, false, RED, L.tailL);
    drawLamp(v,  hw * 0.50f, -hl, rz, 18.0f, 10.0f, false, RED, L.tailR);
    drawLamp(v, -hw * 0.80f, -hl, rz, 10.0f,  9.0f, false, AMBER, L.indRL);
    drawLamp(v,  hw * 0.80f, -hl, rz, 10.0f,  9.0f, false, AMBER, L.indRR);

    // Nested inside the indicator housing - small, inboard, low.
    drawLamp(v, -hw * 0.80f, -hl, rz - 5.0f, 4.5f, 3.5f, false, WHITE, L.revL);
    drawLamp(v,  hw * 0.80f, -hl, rz - 5.0f, 4.5f, 3.5f, false, WHITE, L.revR);
}

Void drawSensor(const View& v, Float32 dpi, Float32 atX, Float32 atY, Float32 atZ)
{
    constexpr Float32 BASE_MM  = vehicle::C1_BASE_MM;
    constexpr Float32 TALL_MM  = vehicle::C1_TALL_MM;
    constexpr Float32 PLINTH_H = 16.0f;    // the fixed lower half
    constexpr Float32 HEAD_R   = 24.0f;    // the spinning head

    const Float32 hb = BASE_MM * 0.5f;

    const ImU32 body = IM_COL32(0x2A, 0x2E, 0x34, 0xFF);
    const ImU32 top  = IM_COL32(0x3A, 0x40, 0x48, 0xFF);
    const ImU32 head = IM_COL32(0x1E, 0x22, 0x28, 0xFF);
    const ImU32 edge = IM_COL32(0x9A, 0xA6, 0xB4, 0xB0);

    // Plinth.
    const auto at = [&](Float32 x, Float32 y, Float32 z) {
        return Vec3{ atX + x, atY + y, atZ + z };
    };

    const Vec3 b0 = at(-hb, -hb, 0.0f), b1 = at(hb, -hb, 0.0f);
    const Vec3 b2 = at( hb,  hb, 0.0f), b3 = at(-hb, hb, 0.0f);
    const Vec3 t0 = at(-hb, -hb, PLINTH_H), t1 = at(hb, -hb, PLINTH_H);
    const Vec3 t2 = at( hb,  hb, PLINTH_H), t3 = at(-hb, hb, PLINTH_H);

    const Vec3 sides[4][4] = {
        { b0, b1, t1, t0 }, { b1, b2, t2, t1 },
        { b2, b3, t3, t2 }, { b3, b0, t0, t3 },
    };
    for(Int32 i = 0; i < 4; ++i)
        pushFace(v, sides[i], 4, body, edge, 1.0f * dpi);

    const Vec3 cap[4] = { t0, t1, t2, t3 };
    pushFace(v, cap, 4, top, edge, 1.0f * dpi);

    // The head, as a short cylinder. This is the part that actually turns, and
    // drawing it as its own piece is the only visual cue this view gives that
    // the device is a rotating scanner rather than a box.
    constexpr Int32 SEG = 16;
    Vec3 lo[SEG], hi[SEG];
    for(Int32 i = 0; i < SEG; ++i)
    {
        const Float32 a = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
        const Float32 x = HEAD_R * std::cos(a);
        const Float32 y = HEAD_R * std::sin(a);
        lo[i] = at(x, y, PLINTH_H);
        hi[i] = at(x, y, TALL_MM);
    }
    for(Int32 i = 0; i < SEG; ++i)
    {
        const Int32 j = (i + 1) % SEG;
        const Vec3 q[4] = { lo[i], lo[j], hi[j], hi[i] };
        pushFace(v, q, 4, head, 0u, 0.0f);
    }

    const Vec3 crown = at(0.0f, 0.0f, TALL_MM);
    for(Int32 i = 0; i < SEG; ++i)
        emitTri(crown, hi[i], hi[(i + 1) % SEG], top, nullptr, 0);

    // The emitter window, marking which way bearing 0 points.
    const Vec3 w0 = at(-10.0f, HEAD_R * 0.98f, PLINTH_H + 6.0f);
    const Vec3 w1 = at( 10.0f, HEAD_R * 0.98f, PLINTH_H + 6.0f);
    const Vec3 w2 = at( 10.0f, HEAD_R * 0.98f, TALL_MM - 6.0f);
    const Vec3 w3 = at(-10.0f, HEAD_R * 0.98f, TALL_MM - 6.0f);
    const Vec3 win[4] = { w0, w1, w2, w3 };
    pushFace(v, win, 4, IM_COL32(0x8A, 0x1E, 0x1E, 0xFF), 0u, 0.0f);
}

Void drawCarFallback(const View& v, Float32 dpi)
{
    const Float32 hw = EGO_WID_MM * 0.5f;
    const Float32 hl = EGO_LEN_MM * 0.5f;
    const Float32 hz = EGO_HEIGHT_MM;

    // The car is the one object in the scene that is NOT measured data, so it is
    // drawn as a solid shell while every return stays an outline. You should
    // never have to wonder which of the two the sensor actually saw.
    const ImU32 shell = IM_COL32(0x2A, 0x33, 0x3E, 0xFF);
    const ImU32 upper = IM_COL32(0x36, 0x41, 0x4E, 0xFF);
    const ImU32 glass = IM_COL32(0x18, 0x2A, 0x38, 0xFF);
    const ImU32 edge  = IM_COL32(0xC8, 0xD2, 0xDC, 0xC0);
    const Float32 ew  = 1.1f * dpi;

    // Wheels first - they are inboard of the shoulders and mostly hidden by it,
    // and the depth sort will interleave them correctly anyway.
    const Float32 wr = EGO_WHEEL_D_MM * 0.5f;
    const Float32 ww = EGO_WHEEL_W_MM * 0.5f;
    const Float32 ax = EGO_WHEELBASE_MM * 0.5f;
    const Float32 tx = EGO_TREAD_MM * 0.5f;
    for(Int32 sx = -1; sx <= 1; sx += 2)
        for(Int32 sy = -1; sy <= 1; sy += 2)
            wheel(v, static_cast<Float32>(sx) * tx, static_cast<Float32>(sy) * ax,
                  wr, ww, IM_COL32(0x1E, 0x1E, 0x1E, 0xFF),
                  IM_COL32(0x8A, 0x90, 0x96, 0xFF), dpi);

    // The shell, lofted station to station.
    Vec3 loop[STATION_N][SECTION_N];
    for(Int32 i = 0; i < STATION_N; ++i)
        sectionLoop(STATIONS[i], hw, hl, hz, loop[i]);

    for(Int32 i = 0; i < STATION_N - 1; ++i)
    {
        for(Int32 k = 0; k < SECTION_N; ++k)
        {
            const Int32 k2 = (k + 1) % SECTION_N;

            // The roof panel (the segment across the top) gets the lighter
            // colour, and the two screen bays get glass - which is the whole
            // reason for lofting rather than extruding: those panels only exist
            // because the sections differ.
            ImU32 col = shell;
            if(k == 2)                       col = upper;   // roof / bonnet top
            if(k == 2 && (i == 3 || i == 5)) col = glass;   // screens

            const Vec3 q[4] = { loop[i][k], loop[i][k2],
                                loop[i + 1][k2], loop[i + 1][k] };
            pushFace(v, q, 4, col, edge, ew);
        }
    }

    // Caps, so the nose and tail are closed rather than open tubes.
    //
    // The fan triangles carry NO edge. Outlining each one drew a star across
    // both ends of the car - the spokes of the fan are an artefact of how the
    // cap is triangulated, not lines that exist on the shell. The rim gets its
    // outline separately, from the loop itself.
    for(Int32 e = 0; e < 2; ++e)
    {
        const Vec3* L = loop[e == 0 ? 0 : STATION_N - 1];
        const Vec3 mid{ 0.0f, L[0].y, (L[0].z + L[2].z) * 0.5f };

        for(Int32 k = 0; k < SECTION_N; ++k)
        {
            const Int32 k2 = (k + 1) % SECTION_N;
            const Vec3 t[3] = { mid, L[k], L[k2] };
            pushFace(v, t, 3, shell, 0u, 0.0f);
        }
        for(Int32 k = 0; k < SECTION_N; ++k)
        {
            const Int32 k2 = (k + 1) % SECTION_N;
            const Vec3 o2[3] = { L[k], L[k2], L[k2] };
            pushFace(v, o2, 3, 0u, edge, ew);
        }
    }

    // Lights. Two amber at the nose, two red at the tail - the cheapest possible
    // way to make which end is which readable from any orbit angle, which
    // matters more here than on the flat map because the map always faced up.
    const auto lamp = [&](Float32 x, Float32 y, Float32 z, ImU32 col) {
        const Float32 s = 22.0f;
        const Vec3 q[4] = { Vec3{ x - s, y, z - s * 0.6f },
                            Vec3{ x + s, y, z - s * 0.6f },
                            Vec3{ x + s, y, z + s * 0.6f },
                            Vec3{ x - s, y, z + s * 0.6f } };
        pushFace(v, q, 4, col, 0u, 0.0f);
    };
    lamp(-hw * 0.42f, hl * 0.99f, hz * 0.22f, IM_COL32(0xFF, 0xE0, 0x90, 0xFF));
    lamp( hw * 0.42f, hl * 0.99f, hz * 0.22f, IM_COL32(0xFF, 0xE0, 0x90, 0xFF));
    lamp(-hw * 0.50f, -hl * 0.99f, hz * 0.34f, IM_COL32(0xE0, 0x28, 0x20, 0xFF));
    lamp( hw * 0.50f, -hl * 0.99f, hz * 0.34f, IM_COL32(0xE0, 0x28, 0x20, 0xFF));

    // The rear wing a touring shell carries, on two posts. Cheap, and it settles
    // the front/back question from above, where the lights are edge-on.
    {
        const Float32 y  = -hl * 0.92f;
        const Float32 z  = hz * 0.78f;
        const Float32 sw = hw * 0.86f;
        const Vec3 q[4] = { Vec3{ -sw, y - 26.0f, z }, Vec3{ sw, y - 26.0f, z },
                            Vec3{  sw, y + 26.0f, z }, Vec3{ -sw, y + 26.0f, z } };
        pushFace(v, q, 4, upper, edge, ew);

        for(Int32 sx = -1; sx <= 1; sx += 2)
        {
            const Float32 px = static_cast<Float32>(sx) * sw * 0.7f;
            const Vec3 p[4] = { Vec3{ px, y, hz * 0.52f }, Vec3{ px, y, z },
                                Vec3{ px, y + 10.0f, z }, Vec3{ px, y + 10.0f, hz * 0.52f } };
            pushFace(v, p, 4, shell, edge, ew * 0.8f);
        }
    }
}

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------
// THE CHASSIS FILTER IS GONE, and its removal is the point.
//
// Returns inside the car's footprint used to be discarded here, because in the
// painter's-algorithm renderer they drew as columns standing up THROUGH the
// model. That fixed a picture and broke a measurement: a hand cupped around the
// sensor sits well inside a 430 x 190 mm footprint, so the filter deleted the
// easiest way there is to check the device is alive. It was reported within a
// day, which is about how long a display that hides real returns deserves.
//
// The depth buffer makes it unnecessary. A return behind the car is occluded
// because it IS behind the car, per pixel; one in front draws in front. Nothing
// is guessed and nothing is thrown away. Where the car would hide something you
// want to see, hide the CAR - see DrawArgs::showCar.
Int32 drawReturns(const View& v, const DrawArgs& a, Bool solid, Int32* hidden)
{
    if(hidden != nullptr)
        *hidden = 0;
    if(a.points == nullptr)
        return 0;

    const Float32 deg2rad = PI_F / 180.0f;
    Int32 n = 0;

    for(const LidarPoint& p : *a.points)
    {
        if(!(p.distMm >= MIN_VALID_MM) || p.distMm > MAX_VALID_MM)
            continue;

        // 2D map bearing: 0 = forward, clockwise. Scene: +y forward, +x right.
        const Float32 ang = p.angleDeg * deg2rad;
        const Float32 x = p.distMm * std::sin(ang);
        const Float32 y = p.distMm * std::cos(ang);

        ++n;

        if(solid)
        {
            column(v, x, y, COLUMN_W_MM * 0.5f, COLUMN_MM,
                   IM_COL32(0xC8, 0xC8, 0xC8, 0x9A),
                   IM_COL32(0xFF, 0xFF, 0xFF, 0xD0),
                   IM_COL32(0xFF, 0xFF, 0xFF, 0x50), 1.0f * a.dpi);
        }
        else
        {
            // A pin: a stalk, and a head at the top. The head used to be a
            // screen-space circle drawn straight to the ImGui list - which is
            // precisely why the car covered pins that were in front of it, since
            // nothing drawn that way is depth-tested against anything. It is a
            // camera-facing quad in the scene now, like everything else.
            const Vec3 g{ x, y, 0.0f }, t{ x, y, COLUMN_MM };
            ribbon(v, g, t, IM_COL32(0xFF, 0xFF, 0xFF, 0x55), 1.0f * a.dpi);

            const Float32 hr = v.pxToWorld(t, 3.0f * a.dpi) * 0.5f;
            const Vec3 hx = mul(v.right, hr);
            const Vec3 hy = mul(v.up, hr);
            const Vec3 head[4] = { sub(sub(t, hx), hy), add(sub(t, hy), hx),
                                   add(add(t, hx), hy), sub(add(t, hy), hx) };
            pushFace(v, head, 4, IM_COL32(0xFF, 0xFF, 0xFF, 0xFF), 0u, 0.0f);
        }
    }
    return n;
}

Int32 drawWalls(const View& v, const DrawArgs& a)
{
    if(a.walls == nullptr)
        return 0;

    const ImU32 face = (ui::ansi::BRCYAN & 0x00FFFFFFu) | (0x2Eu << IM_COL32_A_SHIFT);
    const ImU32 edge = (ui::ansi::BRCYAN & 0x00FFFFFFu) | (0xE0u << IM_COL32_A_SHIFT);

    for(const mapgeo::WallSeg& w : *a.walls)
    {
        const Vec3 a0 = toScene(w.a.x, w.a.y, 0.0f);
        const Vec3 b0 = toScene(w.b.x, w.b.y, 0.0f);
        const Vec3 a1 = toScene(w.a.x, w.a.y, WALL_H_MM);
        const Vec3 b1 = toScene(w.b.x, w.b.y, WALL_H_MM);

        const Vec3 q[4] = { a0, b0, b1, a1 };
        pushFace(v, q, 4, face, edge, 1.6f * a.dpi);
    }
    return static_cast<Int32>(a.walls->size());
}

Void drawFitFloor(const View& v, const DrawArgs& a)
{
    if(a.reach == nullptr || a.reachN < 3)
        return;

    const ImU32 face = (ui::ansi::BRGREEN & 0x00FFFFFFu) | (0x2Cu << IM_COL32_A_SHIFT);
    const ImU32 edge = (ui::ansi::BRGREEN & 0x00FFFFFFu) | (0xC0u << IM_COL32_A_SHIFT);

    const Vec3 hub{ 0.0f, 0.0f, 1.0f };   // a hair above the grid, not in it

    for(Int32 i = 0; i < a.reachN; ++i)
    {
        const Int32 j = (i + 1) % a.reachN;

        // Bin centres, in the same convention the clearance map uses: bearing 0
        // is forward, and forward is +y here.
        const Float32 ai = (static_cast<Float32>(i) + 0.5f) * a.reachBinDeg * (PI_F / 180.0f);
        const Float32 aj = (static_cast<Float32>(j) + 0.5f) * a.reachBinDeg * (PI_F / 180.0f);

        const Vec3 pi{ a.reach[i] * std::sin(ai), a.reach[i] * std::cos(ai), 1.0f };
        const Vec3 pj{ a.reach[j] * std::sin(aj), a.reach[j] * std::cos(aj), 1.0f };

        const Vec3 t[3] = { hub, pi, pj };
        pushFace(v, t, 3, face, 0u, 0.0f);

        const Vec3 e[3] = { pi, pj, pj };
        pushFace(v, e, 3, 0u, edge, 1.6f * a.dpi);
    }
}

// How the frame of reference reads in the corner.
//
// In a world frame the heading is a MEASUREMENT, so it is shown with its
// confidence. Below about a third, the room is not distinctive enough to
// identify a direction - a corridor, a bare wall, anything close to
// rotationally symmetric - and saying "unsure" is the only honest output.
const Char* lockNote(const DrawArgs& a, Float32 yaw, Char* buf, Size cap)
{
    if(a.worldHeadingOk < 0.0f)
    {
        std::snprintf(buf, cap, "car lock");
        return buf;
    }
    // Folded to (-180, 180] and snapped through zero, so a heading of -0.3 deg
    // prints as "0" rather than "-0" - which reads as a bug even though it is
    // just a sign bit on a rounded number.
    while(yaw >  180.0f) yaw -= 360.0f;
    while(yaw <= -180.0f) yaw += 360.0f;
    if(yaw > -0.5f && yaw < 0.5f)
        yaw = 0.0f;

    if(a.worldHeadingOk < 0.33f)
        std::snprintf(buf, cap, "world lock, heading unsure (%.0f%%)",
                      static_cast<Float64>(a.worldHeadingOk * 100.0f));
    else
        std::snprintf(buf, cap, "world lock, %.0f deg (%.0f%%)",
                      static_cast<Float64>(yaw),
                      static_cast<Float64>(a.worldHeadingOk * 100.0f));
    return buf;
}

Void say(const DrawArgs& a, const Char* fmt, ...)
{
    if(a.diag == nullptr || a.diagCap == 0)
        return;
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(a.diag, a.diagCap, fmt, ap);
    va_end(ap);
}

const SceneModeInfo SCENE_INFO[static_cast<Size>(SceneMode::SCENE_MODE_COUNT)] = {
    { "Cloud",
      "Every in-spec return as a pin standing on the ground plane.",
      "The scan, in the space it was taken in. The pins are all the same "
      "height because the C1 measures one horizontal slice and knows nothing "
      "about height - that height is a drawing convention, not data." },

    { "Blocks",
      "The same returns as solid columns.",
      "Reads as geometry rather than as a plot: at a low camera angle a run of "
      "columns occludes what is behind it, which is what the sensor sees too." },

    { "Walls",
      "Fitted straight surfaces, extruded into standing panels.",
      "Where 3D earns its place. A wall drawn as a panel you can orbit behind "
      "is a surface; the same wall on the flat map is a line segment." },

    { "Fit",
      "The floor the car can actually reach, laid on the ground.",
      "The 2D Fit polygon, seen from where the car sits. Orbit down to eye "
      "level and the drivable floor is the part you can still see." },

    { "Full",
      "The ride view: the floor, the things on it as soft boxes, and the path "
      "ahead. Almost no text.",
      "Modelled on what an autonomous car shows its passengers - a picture of "
      "the situation rather than an instrument. Every detection is the same "
      "plain box because a planar lidar has no classifier: a display that drew "
      "a pedestrian would be inventing one." },
};

} // namespace

// ---------------------------------------------------------------------------

Void Camera::orbit(Float32 dYaw, Float32 dPitch)
{
    yaw += dYaw;
    while(yaw >  PI_F) yaw -= 2.0f * PI_F;
    while(yaw < -PI_F) yaw += 2.0f * PI_F;

    // Never past vertical either way: at the poles the up vector degenerates
    // and the ground plane flips over, and no drag should be able to do that.
    pitch += dPitch;
    if(pitch <  0.02f) pitch = 0.02f;
    if(pitch >  1.52f) pitch = 1.52f;
}

Void Camera::pan(Float32 dRight, Float32 dUp)
{
    // Locked to the car means locked. See Camera::lockToCar.
    if(lockToCar)
        return;

    // In the ground plane, so panning never lifts the scene off the floor.
    const Float32 s = std::sin(yaw), c = std::cos(yaw);
    target.x += dRight * c + dUp * s;
    target.y += dRight * -s + dUp * c;
}

Void Camera::zoom(Float32 factor)
{
    dist *= factor;
    if(dist <  400.0f)   dist = 400.0f;
    if(dist > 26000.0f)  dist = 26000.0f;
}

const SceneModeInfo& sceneModeInfo(SceneMode m) noexcept
{
    const Size i = static_cast<Size>(m);
    if(i >= static_cast<Size>(SceneMode::SCENE_MODE_COUNT))
        return SCENE_INFO[0];
    return SCENE_INFO[i];
}

const Char* sceneModeName(SceneMode m) noexcept
{
    return sceneModeInfo(m).name;
}

Void draw(const Camera& cam, const DrawArgs& a)
{
    if(a.dl == nullptr || (a.p1.x - a.p0.x) < 32.0f || (a.p1.y - a.p0.y) < 32.0f)
        return;

    const View v = makeView(cam, a.p0, a.p1, a.worldYawDeg);

    a.dl->PushClipRect(a.p0, a.p1, true);

    // The scene is rendered off-screen with a depth buffer and composited as one
    // image. If that cannot be set up, the viewport is left black rather than
    // falling back to the painter's algorithm this replaced - a renderer that
    // silently degrades to the wrong answer is worse than one that shows
    // nothing and says so.
    const Int32 pw = static_cast<Int32>(a.p1.x - a.p0.x);
    const Int32 ph = static_cast<Int32>(a.p1.y - a.p0.y);

    if(!scenegpu::begin(pw, ph))
    {
        a.dl->AddRectFilled(a.p0, a.p1, ui::ansi::BLACK);
        say(a, "3D renderer unavailable");
        a.dl->PopClipRect();
        return;
    }

    // The ride view gets a surface; the analytical modes get a ruler.
    if(a.mode == SceneMode::SCENE_MODE_FULL)
        drawRideGround(v, a);
    else if(sceneWantsSquares(a.mode))
        drawGroundSquares(a.dl, v, a.dpi);
    else
        drawGround(a.dl, v, a.dpi);

    Int32 n = 0;
    Int32 hidden = 0;
    Char  lockBuf[64] = {};
    switch(a.mode)
    {
    case SceneMode::SCENE_MODE_BLOCKS:
        n = drawReturns(v, a, true, &hidden);
        say(a, "%d returns as columns%s  |  %s, orbit %.0f deg, %.1f m out",
            n, "",
            lockNote(a, a.worldYawDeg, lockBuf, sizeof(lockBuf)),
            static_cast<Float64>(cam.yaw * 180.0f / PI_F),
            static_cast<Float64>(cam.dist / 1000.0f));
        break;

    case SceneMode::SCENE_MODE_WALLS:
        n = drawWalls(v, a);
        drawReturns(v, a, false, &hidden);
        say(a, "%d wall%s as panels%s  |  %s, orbit %.0f deg, %.1f m out",
            n, n == 1 ? "" : "s", "",
            lockNote(a, a.worldYawDeg, lockBuf, sizeof(lockBuf)),
            static_cast<Float64>(cam.yaw * 180.0f / PI_F),
            static_cast<Float64>(cam.dist / 1000.0f));
        break;

    case SceneMode::SCENE_MODE_FIT:
        drawFitFloor(v, a);
        n = drawReturns(v, a, false, &hidden);
        say(a, "drivable floor from %d returns  |  %s, orbit %.0f deg, %.1f m out",
            n, lockNote(a, a.worldYawDeg, lockBuf, sizeof(lockBuf)),
            static_cast<Float64>(cam.yaw * 180.0f / PI_F),
            static_cast<Float64>(cam.dist / 1000.0f));
        break;

    case SceneMode::SCENE_MODE_FULL:
        // The ride view. No returns as columns and no wall panels: this mode
        // shows the CONCLUSIONS - the floor, the things on it, and where the car
        // is going - and the evidence for them is what the other four are for.
        drawPathRibbon(v, a);
        for(Int32 i = 0; i < a.objectN; ++i)
            drawDetection(v, a.objects[i], a.dpi);
        n = a.objectN;
        say(a, "%d object%s  |  %.2f m ahead", n, n == 1 ? "" : "s",
            static_cast<Float64>(a.aheadMm / 1000.0f));
        break;

    case SceneMode::SCENE_MODE_CLOUD:
    case SceneMode::SCENE_MODE_COUNT:
    default:
        n = drawReturns(v, a, false, &hidden);
        say(a, "%d returns%s  |  %s, orbit %.0f deg, %.1f m out",
            n, "",
            lockNote(a, a.worldYawDeg, lockBuf, sizeof(lockBuf)),
            static_cast<Float64>(cam.yaw * 180.0f / PI_F),
            static_cast<Float64>(cam.dist / 1000.0f));
        break;
    }

    // The downloaded model when it is there, the hand-built shell when it is
    // not. A missing asset must cost fidelity, never the car: without something
    // at the origin the whole scene loses its frame of reference.
    if(a.ego == EgoView::EGO_VIEW_SENSOR)
    {
        // On its own, standing on the floor. The whole point of this mode is
        // the SIZE: 56 mm against a 442 mm car.
        drawSensor(v, a.dpi, 0.0f, 0.0f, 0.0f);
    }
    else
    {
        const CarModel& car = carModel();
        if(car.loaded)
            drawCarModel(v, car, a.dpi);
        else
            drawCarFallback(v, a.dpi);

        // And the sensor ON it, at its mount. Every measurement in this scene
        // is in the sensor's frame, so where the sensor sits on the car is the
        // relationship the whole picture is built on - and leaving it out made
        // the car look like the thing doing the measuring.
        drawSensor(v, a.dpi,
                   vehicle::C1_MOUNT_LATERAL_MM,
                   vehicle::C1_MOUNT_AHEAD_MM,
                   vehicle::C1_MOUNT_BASE_MM);

        drawLamps(v, a.lamps);
    }

    const ImTextureID img = scenegpu::end(v.mvp);
    if(img != 0)
        a.dl->AddImage(img, a.p0, a.p1);
    else
        a.dl->AddRectFilled(a.p0, a.p1, ui::ansi::BLACK);

    a.dl->PopClipRect();
}

} // namespace scene3d
