#include "scene3d.hxx"

#include "icons.hxx"
#include "scene_gpu.hxx"
#include "vehicle.hxx"
#include "theme.hxx"

// NOMINMAX before windows.h, or its min/max MACROS shadow std::min/std::max and
// every call becomes a syntax error. Needed only here: this file loads the model.
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace scene3d
{

  namespace
  {

    constexpr Float32 PI_F = 3.14159265358979323846f;

    // Repeated, not shared: the 3D view discards what the flat map discards.
    constexpr Float32 MIN_VALID_MM = 50.0f;
    constexpr Float32 MAX_VALID_MM = 12000.0f;

    // The car and the sensor, from vehicle.hxx - one definition for the whole app.
    constexpr Float32 EGO_LEN_MM = vehicle::CAR_LEN_MM;
    constexpr Float32 EGO_WID_MM = vehicle::CAR_WID_MM;
    constexpr Float32 EGO_HEIGHT_MM = vehicle::CAR_HEIGHT_MM;
    constexpr Float32 EGO_WHEELBASE_MM = vehicle::CAR_WHEELBASE_MM;
    constexpr Float32 EGO_TREAD_MM = vehicle::CAR_TREAD_MM;
    constexpr Float32 EGO_WHEEL_D_MM = vehicle::CAR_TIRE_DIA_MM;
    constexpr Float32 EGO_WHEEL_W_MM = vehicle::CAR_TIRE_WID_MM;
    constexpr Float32 EGO_SENSOR_AHEAD_MM = vehicle::C1_MOUNT_AHEAD_MM;

    // How tall a return stands: up to the SCAN PLANE, and no further. The C1 is a
    // planar scanner - one horizontal slice, no height - so every column is the
    // same height, and varying it would invent a third dimension. Using the plane
    // the beam sweeps makes the top of every column a real measured surface.
    constexpr Float32 COLUMN_MM = vehicle::C1_SCAN_Z_MM;
    constexpr Float32 COLUMN_W_MM = 46.0f;
    constexpr Float32 WALL_H_MM = 260.0f;

    // ---- vector helpers -------------------------------------------------------
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

    // The projection, built once per frame, in two forms of the same camera: an MVP
    // matrix for the GPU, and the basis vectors, because billboarding a line and
    // sizing it in PIXELS both need the eye and the pixel width at a given depth.
    struct View
    {
        Vec3    eye, right, up, fwd;
        ImVec2  p0, p1;              // the widget rect, screen space
        Float32 focal = 1.0f;       // pixels per unit at unit depth
        Float32 nearMm = 40.0f;
        Float32 farMm = 60000.0f;
        Array<Float32, 16> mvp= {};

        [[nodiscard]] Float32 depthOf(const Vec3& p) const
        {
            return dot(sub(p, eye), fwd);
        }

        // Half-width in WORLD units of something `px` pixels wide where it sits;
        // without it every line has a fixed physical width and the far grid goes.
        [[nodiscard]] Float32 pxToWorld(const Vec3& at, Float32 px) const
        {
            const Float32 d = depthOf(at);
            return (d > nearMm) ? (px * d / focal) : (px * nearMm / focal);
        }

        // Screen position, for the few things still 2D - labels and the HUD.
        Bool project(const Vec3& p, ImVec2& out) const
        {
            const Float32 x = p.x * mvp[0] + p.y * mvp[4] + p.z * mvp[8]  + mvp[12];
            const Float32 y = p.x * mvp[1] + p.y * mvp[5] + p.z * mvp[9]  + mvp[13];
            const Float32 w = p.x * mvp[3] + p.y * mvp[7] + p.z * mvp[11] + mvp[15];
            if(w <= 1e-4f)
            {
                return false;
            }
            out = ImVec2(
                p0.x + (x / w * 0.5f + 0.5f) * (p1.x - p0.x),
                p0.y + (0.5f - y / w * 0.5f) * (p1.y - p0.y)
            );
            return true;
        }
    };

    View makeView(const Camera& c, const ImVec2& p0, const ImVec2& p1, Float32 worldYawDeg)
    {
        View v;
        const Float32 cp = std::cos(c.pitch), sp = std::sin(c.pitch);
        v.eye = add(
            c.target,
            Vec3{ c.dist * cp * std::sin(c.yaw), -c.dist * cp * std::cos(c.yaw), c.dist * sp }
        );

        v.fwd = norm(sub(c.target, v.eye));
        v.right = norm(cross(v.fwd, Vec3{ 0.0f, 0.0f, 1.0f }));
        v.up = cross(v.right, v.fwd);

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
        const Float32 q = zf / (zf - zn);

        const Array<Float32, 16> view = {
            v.right.x, v.up.x, v.fwd.x, 0.0f,
            v.right.y, v.up.y, v.fwd.y, 0.0f,
            v.right.z, v.up.z, v.fwd.z, 0.0f,
            -dot(v.eye, v.right), -dot(v.eye, v.up), -dot(v.eye, v.fwd), 1.0f,
        };
        const Array<Float32, 16> proj = {
            xs,   0.0f, 0.0f,     0.0f,
            0.0f, ys,   0.0f,     0.0f,
            0.0f, 0.0f, q,        1.0f,
            0.0f, 0.0f, -zn * q,  0.0f,
        };

        // Rotation about the vertical, applied BEFORE the view - a model matrix for
        // the whole scene. See DrawArgs::worldYawDeg. Sign, derived not guessed: a
        // return at bearing b lands at (sin b, cos b), so rising bearing sweeps
        // CLOCKWISE from above, a NEGATIVE rotation about +z. A fixed feature whose
        // measured bearing rises by d moved by -d, so undoing it is +d.
        const Float32 wy = worldYawDeg * (PI_F / 180.0f);
        const Float32 cw = std::cos(wy), sw = std::sin(wy);
        const Array<Float32, 16> model = {
             cw,   sw,   0.0f, 0.0f,
            -sw,   cw,   0.0f, 0.0f,
             0.0f, 0.0f, 1.0f, 0.0f,
             0.0f, 0.0f, 0.0f, 1.0f,
        };

        Array<Float32, 16> mv= {};
        for(Int32 r = 0; r < 4; ++r)
        {
            for(Int32 col = 0; col < 4; ++col)
            {
                Float32 sum = 0.0f;
                for(Int32 k = 0; k < 4; ++k)
                {
                    sum += model[r * 4 + k] * view[k * 4 + col];
                }
                mv[r * 4 + col] = sum;
            }
        }

        for(Int32 r = 0; r < 4; ++r)
        {
            for(Int32 col = 0; col < 4; ++col)
            {
                Float32 sum = 0.0f;
                for(Int32 k = 0; k < 4; ++k)
                {
                    sum += mv[r * 4 + k] * proj[k * 4 + col];
                }
                v.mvp[r * 4 + col] = sum;
            }
        }

        // Eye and basis stay in SCENE space, unrotated: billboarding and pixel
        // sizing are camera properties, not world-frame ones.
        return v;
    }

    // ---- emission -------------------------------------------------------------
    // These hand WORLD-space triangles to the GPU. Opaque or blended is decided by
    // the fill's own alpha, not by a flag at every call site.
    scenegpu::Vertex vtx(const Vec3& p, ImU32 col, const ImVec2& uv)
    {
        scenegpu::Vertex v;
        v.x = p.x;
        v.y = p.y;
        v.z = p.z;
        v.col = col;
        v.u = uv.x;
        v.v = uv.y;
        return v;
    }

    Void emitTri(const Vec3& a, const Vec3& b, const Vec3& c, ImU32 col, const ImVec2* uv, ImTextureID tex)
    {
        const ImVec2 z(0.5f, 0.5f);
        const scenegpu::Vertex va = vtx(a, col, uv ? uv[0] : z);
        const scenegpu::Vertex vb = vtx(b, col, uv ? uv[1] : z);
        const scenegpu::Vertex vc = vtx(c, col, uv ? uv[2] : z);

        if((col >> IM_COL32_A_SHIFT) >= 0xFFu)
        {
            scenegpu::addOpaque(va, vb, vc, tex);
        }
        else
        {
            scenegpu::addBlended(va, vb, vc, tex);
        }
    }

    // The same, per vertex. A gradient needs it: flat triangles read as wedges.
    Void emitTriC(const Vec3& a, const Vec3& b, const Vec3& c, ImU32 ca, ImU32 cb, ImU32 cc)
    {
        const ImVec2 z(0.5f, 0.5f);
        const scenegpu::Vertex va = vtx(a, ca, z);
        const scenegpu::Vertex vb = vtx(b, cb, z);
        const scenegpu::Vertex vc = vtx(c, cc, z);

        // Blended if ANY vertex is: the triangle is see-through somewhere.
        if((ca >> IM_COL32_A_SHIFT) >= 0xFFu && (cb >> IM_COL32_A_SHIFT) >= 0xFFu
           && (cc >> IM_COL32_A_SHIFT) >= 0xFFu)
        {
            scenegpu::addOpaque(va, vb, vc, 0);
        }
        else
        {
            scenegpu::addBlended(va, vb, vc, 0);
        }
    }

    // A camera-facing quad along a segment, `px` pixels wide wherever it is - how
    // every line in the scene is drawn. A real line primitive would be one pixel
    // wide whatever the DPI, and would not depth-test against solid geometry.
    Void ribbon(const View& v, const Vec3& a, const Vec3& b, ImU32 col, Float32 px)
    {
        const Vec3 dir = sub(b, a);
        if(dot(dir, dir) < 1e-6f)
        {
            return;
        }

        const Vec3 mid{ (a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f, (a.z + b.z) * 0.5f };
        Vec3 side = cross(norm(dir), norm(sub(v.eye, mid)));
        if(dot(side, side) < 1e-9f)
        {
            return;                                  // the line points at the eye
        }
        side = mul(norm(side), v.pxToWorld(mid, px) * 0.5f);

        const Vec3 q0 = sub(a, side), q1 = add(a, side);
        const Vec3 q2 = add(b, side), q3 = sub(b, side);
        emitTri(q0, q1, q2, col, nullptr, 0);
        emitTri(q0, q2, q3, col, nullptr, 0);
    }

    // `dl` is unused: nothing in the scene draws to the ImGui list any more.
    Void line3(ImDrawList* dl, const View& v, const Vec3& a, const Vec3& b, ImU32 col, Float32 w)
    {
        static_cast<Void>(dl);
        ribbon(v, a, b, col, w);
    }

    // How a face is painted: fill, outline, outline width.
    struct FaceStyle
    {
        ImU32   fill = 0u;
        ImU32   edge = 0u;
        Float32 edgeW = 0.0f;
    };

    // Optional texturing. Defaulted, because most faces are flat color.
    struct FaceTex
    {
        ImTextureID   tex = 0;
        const ImVec2* uv = nullptr;
    };

    // A convex face, optionally outlined. The outline is nudged toward the eye by a
    // fraction of its depth so it cannot z-fight with the face it belongs to.
    // Templated on the point count so the count and the array cannot disagree, and
    // the 3-or-4 guard is a static_assert at the call site.
    template<Size N>
    Void pushFace(const View& v, const Array<Vec3, N>& w, const FaceStyle& st, FaceTex tx = {})
    {
        static_assert(N >= 3 && N <= 4, "a face is a triangle or a quad");

        const Int32         n = static_cast<Int32>(N);
        const ImU32         fill = st.fill;
        const ImU32         edge = st.edge;
        const Float32       edgeW = st.edgeW;
        const ImTextureID   tex = tx.tex;
        const ImVec2* const uv = tx.uv;

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
        {
            return;
        }

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

    // A box standing on the ground, centered on (x,y) in scene axes.
    Void column(const View& v, Float32 x, Float32 y, Float32 half, Float32 h, ImU32 side, ImU32 top, ImU32 edge, Float32 edgeW)
    {
        const Float32 x0 = x - half, x1 = x + half;
        const Float32 y0 = y - half, y1 = y + half;

      const Vec3 b00{ x0, y0, 0.0f }, b10{ x1, y0, 0.0f };
      const Vec3 b11{ x1, y1, 0.0f }, b01{ x0, y1, 0.0f };
      const Vec3 t00{ x0, y0, h },    t10{ x1, y0, h };
      const Vec3 t11{ x1, y1, h },    t01{ x0, y1, h };

        // All four sides: the depth buffer hides the far ones for free.
        const Array<Vec3, 4> sN = { b01, b11, t11, t01 };
        const Array<Vec3, 4> sS = { b00, b10, t10, t00 };
        const Array<Vec3, 4> sE = { b10, b11, t11, t10 };
        const Array<Vec3, 4> sW = { b00, b01, t01, t00 };

        pushFace(v, sN, { side, edge, edgeW });
        pushFace(v, sS, { side, edge, edgeW });
        pushFace(v, sE, { side, edge, edgeW });
        pushFace(v, sW, { side, edge, edgeW });

        const Array<Vec3, 4> tp = { t00, t10, t11, t01 };
        pushFace(v, tp, { top, edge, edgeW });
    }

    // The ground - range rings and radials on z = 0 - follows the MODE the way the
    // flat map's does: rings where the quantity is a range and a bearing, squares
    // where it is a length.
    Bool sceneWantsSquares(SceneMode m)
    {
        return m == SceneMode::SCENE_MODE_WALLS;
    }

    Void drawGroundSquares(ImDrawList* dl, const View& v, Float32 dpi)
    {
        constexpr Float32 STEP = 1000.0f;      // one meter
        constexpr Int32   N = 12;
        const Float32 far2 = STEP * N;

        for(Int32 i = -N; i <= N; ++i)
        {
            const Float32 t = static_cast<Float32>(i) * STEP;
            const Bool major = (i % 5) == 0;
            const ImU32 col = (i == 0)
                ? ((ui::ansi::GRID_MAJOR & 0x00FFFFFFu) | (0xB0u << IM_COL32_A_SHIFT))
                : (
                    major ? ((ui::ansi::GRID_MAJOR & 0x00FFFFFFu) | (0x80u << IM_COL32_A_SHIFT)) : (
                        (ui::ansi::GRID & 0x00FFFFFFu) | (0xB0u << IM_COL32_A_SHIFT)
                    )
                );
            const Float32 w = ((i == 0) ? 1.6f : (major ? 1.3f : 1.0f)) * dpi;

            line3(dl, v, Vec3{ t, -far2, 0.0f }, Vec3{ t, far2, 0.0f }, col, w);
            line3(dl, v, Vec3{ -far2, t, 0.0f }, Vec3{ far2, t, 0.0f }, col, w);
        }

        // The heading, in its own color, so which way is forward survives an orbit.
      line3(
          dl,
          v,
          Vec3{ 0.0f, 0.0f, 0.0f },
          Vec3{ 0.0f, far2, 0.0f },
          (ui::ansi::BRCYAN & 0x00FFFFFFu) | (0x99u << IM_COL32_A_SHIFT),
          1.6f * dpi
      );
    }

    // SCENE_MODE_FULL - the ride view. Modelled on the display an autonomous car
    // shows its PASSENGERS: a picture of the situation rather than an instrument.
    // No wireframe, almost no text, few colors, soft and matte.
    //
    // A planar lidar has no classifier and cannot tell a chair leg from an ankle,
    // so every detection is the same plain box: drawing a person because something
    // is person-sized would be the display inventing a fact.
    //
    // Box height is a drawing convention either way; near the car's own 135 mm it
    // reads as "things about this big", where 380 mm read as a wall.
    constexpr Float32 RIDE_BOX_H_MM = 240.0f;

    // The floor: a filled disc fading with range. Its edge is where the sensor
    // stops seeing - the only real boundary this view draws.
    Void drawRideGround(const View& v, const DrawArgs& a)
    {
        // World-space and emitted straight through; `v` is only in the signature.
        static_cast<Void>(v);
        constexpr Int32   SEG = 64;
        constexpr Float32 R_IN = 1400.0f;
        constexpr Float32 R_MID = 5000.0f;
        constexpr Float32 R_OUT = 11000.0f;

        const ImU32 c0 = IM_COL32(0x1E, 0x26, 0x33, 0xFF);
        const ImU32 c1 = IM_COL32(0x12, 0x18, 0x22, 0xFF);
        const ImU32 c2 = IM_COL32(0x08, 0x0B, 0x10, 0x00);

        // Per-VERTEX, not per-triangle, or the ring comes out as flat wedges.
        const auto ring = [&](Float32 r0, Float32 r1, ImU32 a0, ImU32 a1) {
            Vec3 p0{ r0, 0.0f, 0.0f }, p1{ r1, 0.0f, 0.0f };
            for(Int32 i = 1; i <= SEG; ++i)
            {
                const Float32 t = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
                const Vec3 q0{ r0 * std::cos(t), r0 * std::sin(t), 0.0f };
                const Vec3 q1{ r1 * std::cos(t), r1 * std::sin(t), 0.0f };
                emitTriC(p0, q0, q1, a0, a0, a1);
                emitTriC(p0, q1, p1, a0, a1, a1);
                p0 = q0;
                p1 = q1;
            }
        };

        // The disc under the car is lighter, so it stands on something.
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

    // A detection, as a soft chamfered box: eight sides in plan instead of four is
    // four triangles, and the difference between an object and a debug AABB.
    Void drawDetection(const View& v, const Detection& d, Float32 dpi)
    {
        const Float32 px = -d.uy, py = d.ux;
        const Float32 hl = std::max(d.halfL, 40.0f);
        const Float32 hw = std::max(d.halfW, 40.0f);

        // The chamfer, capped so a small object is not chamfered into a diamond.
        const Float32 k = std::min(std::min(hl, hw) * 0.45f, 90.0f);

        // Eight corners, going round the plan.
        const Array<Float32, 8> ol = {  hl - k,  hl,      hl,     hl - k,
                                       -hl + k, -hl,     -hl,    -hl + k };
        const Array<Float32, 8> ow = { -hw,     -hw + k,  hw - k, hw,
                                        hw,      hw - k, -hw + k, -hw };

        Array<Vec3, 8> lo;
        Array<Vec3, 8> hi;
        for(Int32 i = 0; i < 8; ++i)
        {
            const Float32 x = d.cx + d.ux * ol[i] + px * ow[i];
            const Float32 y = d.cy + d.uy * ol[i] + py * ow[i];
            lo[i] = toScene(x, y, 0.0f);
            hi[i] = toScene(x, y, RIDE_BOX_H_MM);
        }

        // In the car's way, or merely present - two states, all this can tell.
        const ImU32 side = d.inPath ? IM_COL32(0xE8, 0x7A, 0x5A, 0x5A)
                                    : IM_COL32(0xC8, 0xD4, 0xE2, 0x3E);
        const ImU32 top = d.inPath ? IM_COL32(0xFF, 0x9A, 0x74, 0xD8)
                                    : IM_COL32(0xE8, 0xF0, 0xFA, 0xC0);

        for(Int32 i = 0; i < 8; ++i)
        {
            const Int32 j = (i + 1) % 8;
            const Array<Vec3, 4> q = { lo[i], lo[j], hi[j], hi[i] };
            pushFace(v, q, { side, 0u, 0.0f });
        }

        const Vec3 mid{ toScene(d.cx, d.cy, RIDE_BOX_H_MM) };
        for(Int32 i = 0; i < 8; ++i)
        {
            emitTri(mid, hi[i], hi[(i + 1) % 8], top, nullptr, 0);
        }

        static_cast<Void>(dpi);
    }

    // Where the car would go: the one saturated color, and the one INTENTION.
    Void drawPathRibbon(const View& v, const DrawArgs& a)
    {
        static_cast<Void>(v);
        if(a.corridorHalfW <= 0.0f || a.corridorFree <= 0.0f)
        {
            return;
        }

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

            // Fades along its length: the far end of a plan is less certain.
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

        // Radials every 45 deg; forward in the heading color, so facing survives.
        for(Int32 b = 0; b < 360; b += 45)
        {
            const Float32 a = static_cast<Float32>(b) * (PI_F / 180.0f);
            const Bool fwd = (b == 90);      // +y is forward
            line3(
                dl,
                v,
                Vec3{ 0.0f, 0.0f, 0.0f },
                Vec3{ 12000.0f * std::cos(a), 12000.0f * std::sin(a), 0.0f },
                fwd ? ((ui::ansi::BRCYAN & 0x00FFFFFFu) | (0x99u << IM_COL32_A_SHIFT)) : (
                    (ui::ansi::AXIS & 0x00FFFFFFu) | (0xFFu << IM_COL32_A_SHIFT)
                ),
                (fwd ? 1.6f : 1.0f) * dpi
            );
        }
    }

    // The car model: assets/models/car.obj - "sedan-sports", Kenney's Car Kit, CC0.
    //
    // The loader reads `v`, `g` and triangular `f`, which is everything this file
    // contains (2088 faces, all triangles, one material). NOT a general OBJ parser:
    // what it does not understand it skips, so another model degrades to a partial
    // mesh rather than to garbage. Color comes from the GROUP NAMES, not the
    // material - `body` / `wheel-*` / `spoiler` is what a schematic wants.
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

    // Both via ui::assetPath, so a copied exe keeps its car - as the icon atlas.
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
        if(std::strstr(g, "wheel")   != nullptr)
        {
            return CAR_PART_WHEEL;
        }
        if(std::strstr(g, "spoiler") != nullptr)
        {
            return CAR_PART_SPOILER;
        }
        return CAR_PART_BODY;
    }

    Bool loadCarObj(CarModel& m)
    {
        m.tris.clear();
        m.loaded = false;

        Array<Char, MAX_PATH> path;
        if(!carModelPath(path.data(), path.size()))
        {
            return false;
        }

        FILE* f = std::fopen(path.data(), "rb");
        if(f == nullptr)
        {
            return false;
        }

        Vec<Vec3> raw;
        raw.reserve(1400);

        Vec<ImVec2> uvs;
        uvs.reserve(1400);

        struct Face { Int32 i0, i1, i2; Int32 t0, t1, t2; Int32 part; };
        Vec<Face> faces;
        faces.reserve(2200);

        Int32 part = CAR_PART_BODY;
        Array<Char, 512> line;

        while(std::fgets(line.data(), static_cast<Int32>(line.size()), f) != nullptr)
        {
            if(line[0] == 'v' && line[1] == ' ')
            {
                // `v x y z [r g b]` - the kit's vertex colors are all white.
                Float32 x = 0.0f, y = 0.0f, z = 0.0f;
                if(std::sscanf(line.data() + 2, "%f %f %f", &x, &y, &z) == 3)
                {
                    raw.push_back(Vec3{ x, y, z });
                }
            }
            else if(line[0] == 'v' && line[1] == 't' && line[2] == ' ')
            {
                // OBJ's V runs up, every API here runs it down. Flipped once.
                Float32 u = 0.0f, vv = 0.0f;
                if(std::sscanf(line.data() + 3, "%f %f", &u, &vv) == 2)
                {
                    uvs.push_back(ImVec2(u, 1.0f - vv));
                }
            }
            else if(line[0] == 'g' && line[1] == ' ')
            {
                Array<Char, 128> name= {};
                if(std::sscanf(line.data() + 2, "%127s", name.data()) == 1)
                {
                    part = partForGroup(name.data());
                }
            }
            else if(line[0] == 'f' && line[1] == ' ')
            {
                // `f a/ta/na ...`. Only triangles here; a polygon is skipped.
                Array<Int32, 3> idx= { 0, 0, 0 };
                Array<Int32, 3> tex= { 0, 0, 0 };
                Int32 got = 0;
                const Char* p = line.data() + 2;
                while(got < 3)
                {
                    while(*p == ' ')
                    {
                        ++p;
                    }
                    if(*p == 0 || *p == '\n' || *p == '\r')
                    {
                        break;
                    }

                    idx[got] = std::atoi(p);

                    // `a/b/c` - b is the texture index, and may be absent (`a//c`).
                    const Char* slash = p;
                    while(*slash != 0 && *slash != ' ' && *slash != '\n'
                          && *slash != '\r' && *slash != '/')
                    {
                        ++slash;
                    }
                    if(*slash == '/' && slash[1] != '/')
                    {
                        tex[got] = std::atoi(slash + 1);
                    }

                    ++got;
                    while(*p != 0 && *p != ' ' && *p != '\n' && *p != '\r')
                    {
                        ++p;
                    }
                }
                if(got == 3)
                {
                    faces.push_back(Face{ idx[0], idx[1], idx[2], tex[0], tex[1], tex[2], part });
                }
            }
        }
        std::fclose(f);

        if(raw.empty() || faces.empty())
        {
            return false;
        }

        // ---- fit to the real chassis -----------------------------------------
        //
        // PER AXIS, not uniform, and the distortion is deliberate: the app measures
        // against the TT-02's actual 430 x 190 x 135, and a uniform scale leaves
        // the car ~15% wider than the corridor drawn beside it.
        Array<Float32, 3> lo= {  1e9f,  1e9f,  1e9f };
        Array<Float32, 3> hi= { -1e9f, -1e9f, -1e9f };
        for(const Vec3& v : raw)
        {
            const Array<Float32, 3> c= { v.x, v.y, v.z };
            for(Int32 k = 0; k < 3; ++k)
            {
                if(c[k] < lo[k])
                {
                    lo[k] = c[k];
                }
                if(c[k] > hi[k])
                {
                    hi[k] = c[k];
                }
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
        // y=forward, z=up. Swapping two axes is a REFLECTION and flips triangle
        // winding, so x is negated as well to put the determinant back to +1;
        // mirroring a symmetric car left-to-right is invisible.
        const auto toScene3 = [&](const Vec3& v) {
            return Vec3{ -(v.x - cx) * sx,
                          (v.z - cz) * sz,
                          (v.y - lo[1]) * sy };
        };

        const Int32 nRaw = static_cast<Int32>(raw.size());
        m.tris.reserve(faces.size());

        for(const Face& fc : faces)
        {
            // OBJ indices are 1-based; negative counts back. Out-of-range drops.
            const auto resolve = [&](Int32 i) -> Int32 {
                if(i > 0)
                {
                    return i - 1;
                }
                if(i < 0)
                {
                    return nRaw + i;
                }
                return -1;
            };

            const Int32 i0 = resolve(fc.i0), i1 = resolve(fc.i1), i2 = resolve(fc.i2);
            if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= nRaw || i1 >= nRaw || i2 >= nRaw)
            {
                continue;
            }

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

        // Lazy and retried: the D3D device is not up at the first model load.
        if(m.loaded && m.tex == 0)
        {
            Array<Char, MAX_PATH> tp;
            if(carTexturePath(tp.data(), tp.size()))
            {
                m.tex = ui::loadTexture(ui::device(), tp.data());
            }
        }
        return m;
    }

    // Flat-shaded. No per-triangle outline: at 2088 faces that is a wireframe.
    Void drawCarModel(const View& v, const CarModel& m, Float32 dpi)
    {
        static_cast<Void>(dpi);

        // One fixed light in WORLD space, so orbiting moves the highlight.
        const Vec3 LIGHT = norm(Vec3{ 0.38f, 0.42f, 0.82f });

        // The contact shadow: without it the car hovers, on a display whose whole
        // subject is where things are on the ground.
        {
            const Float32 rx = EGO_WID_MM * 0.62f;
            const Float32 ry = EGO_LEN_MM * 0.56f;
            constexpr Int32 SEG = 24;

            Vec3 prev{ rx, 0.0f, 2.0f };
            for(Int32 i = 1; i <= SEG; ++i)
            {
                const Float32 t = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
                const Vec3 cur{ rx * std::cos(t), ry * std::sin(t), 2.0f };
                const Array<Vec3, 3> tri = { Vec3{ 0.0f, 0.0f, 2.0f }, prev, cur };
                emitTri(tri[0], tri[1], tri[2], IM_COL32(0, 0, 0, 0x66), nullptr, 0);
                prev = cur;
            }
        }

        constexpr ImU32 BASE[CAR_PART_COUNT] = {
            IM_COL32(0x4A, 0x58, 0x68, 0xFF),   // body
            IM_COL32(0x1C, 0x1E, 0x21, 0xFF),   // wheels
            IM_COL32(0x30, 0x3A, 0x46, 0xFF),   // spoiler
        };

        for(const CarTri& t : m.tris)
        {
            // No back-face rejection: the depth buffer resolves it per pixel, and a
            // downloaded mesh's winding is not worth trusting. A stylised material,
            // not one lambert term:
            //   key  |n.l|, NOT max(0, n.l) - with both faces reachable, an
            //        inward-winding triangle would render black.
            //   sky  hemispheric; stops a flat-shaded model looking like paper.
            //   rim  fresnel, separating the silhouette from a dark background.
            const Vec3 toEye = norm(sub(
                v.eye,
                Vec3{ (t.a.x + t.b.x + t.c.x) / 3.0f, (t.a.y + t.b.y + t.c.y) / 3.0f, (t.a.z + t.b.z + t.c.z) / 3.0f }
            ));

            const Float32 key = std::fabs(dot(t.n, LIGHT));
            const Float32 sky = 0.5f + 0.5f * t.n.z;
            const Float32 rim = 1.0f - std::fabs(dot(t.n, toEye));

            Float32 lit = 0.26f + 0.52f * key + 0.26f * sky + 0.22f * rim * rim;
            if(lit > 1.35f)
            {
                lit = 1.35f;
            }

            // With the atlas bound the vertex color is a LIGHT level, not paint.
            const ImU32 base = (m.tex != 0) ? IM_COL32(0xFF, 0xFF, 0xFF, 0xFF)
                                            : BASE[t.part];
            const auto ch = [&](Int32 shift) {
                const Float32 x = static_cast<Float32>((base >> shift) & 0xFFu) * lit;
                return static_cast<Int32>(x > 255.0f ? 255.0f : x);
            };
            const ImU32 col = IM_COL32(
                ch(IM_COL32_R_SHIFT),
                ch(IM_COL32_G_SHIFT),
                ch(IM_COL32_B_SHIFT),
                0xFF
            );

            const Array<Vec3, 3> tri = { t.a, t.b, t.c };
            const ImVec2 uv[3] = { t.ta, t.tb, t.tc };

            // Textured when the atlas is there, shaded-flat when not: the shading
            // multiplies the SAMPLED color either way, so the form survives both.
            pushFace(v, tri, { col, 0u, 0.0f }, { m.tex, m.tex != 0 ? uv : nullptr });
        }
    }

    // The fallback car: hand-authored from the TT-02's own dimensions, not a
    // scanned or CAD model. Built as CROSS-SECTIONS along the length, lofted
    // together, because that is how a body varies - low narrow bonnet, raked
    // screen, roof inset from the shoulders, tail dropping away. An extruded plan
    // outline reads as a wedge.
    //
    // Heights are fractions of EGO_HEIGHT_MM, widths of the half-width. Front is +y.
    struct Station
    {
        Float32 y;      // along the car, fraction of half-length, +1 = nose
        Float32 wl;     // half-width at the shoulder line
        Float32 wu;     // half-width at the roof line
        Float32 zl;     // sill height
        Float32 zu;     // roof height
    };

    // Nine stations: nose, bonnet, cowl, raked screen, roof, fastback rear.
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

    // One cross-section, a closed loop of six points - the fewest that can carry a
    // shoulder (bottom, shoulder, roof, mirrored). Four is a trapezoid.
    Void sectionLoop(const Station& s, Float32 hw, Float32 hl, Float32 hz, Vec3* out)
    {
        const Float32 y = s.y * hl;
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

    // A wheel: a cylinder about the x axis. Ten segments reads as round here.
    Void wheel(const View& v, Float32 cx, Float32 cy, Float32 r, Float32 halfW, ImU32 tread, ImU32 rim, Float32 dpi)
    {
        constexpr Int32 SEG = 10;

        Array<Vec3, SEG> in;
        Array<Vec3, SEG> out;
        for(Int32 i = 0; i < SEG; ++i)
        {
            const Float32 a = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
            const Float32 y = cy + r * std::cos(a);
            const Float32 z = r  + r * std::sin(a);      // sits ON the ground
            in[i] = Vec3{ cx - halfW, y, z };
            out[i] = Vec3{ cx + halfW, y, z };
        }

        for(Int32 i = 0; i < SEG; ++i)
        {
            const Int32 j = (i + 1) % SEG;
            const Array<Vec3, 4> q = { in[i], in[j], out[j], out[i] };
            pushFace(v, q, { tread, 0u, 0.0f });
        }

        // The outboard face only: the inboard one is inside the bodywork.
        const Vec3* side = (cx > 0.0f) ? out.data() : in.data();
        const Vec3 hub{ (cx > 0.0f) ? cx + halfW : cx - halfW, cy, r };
        for(Int32 i = 0; i < SEG; ++i)
        {
            const Int32 j = (i + 1) % SEG;
            const Array<Vec3, 3> t = { hub, side[i], side[j] };
            pushFace(v, t, { rim, 0u, 0.0f });
        }

        // The rim's edge, so a wheel has an outline like everything else.
        for(Int32 i = 0; i < SEG; ++i)
        {
            const Int32 j = (i + 1) % SEG;
            const Array<Vec3, 3> e = { side[i], side[j], side[j] };
            pushFace(v, e, { 0u, IM_COL32(0xB0, 0xB4, 0xB8, 0xC0), 1.0f * dpi });
        }
    }

    // Lamps, positioned from the car's own dimensions rather than from the mesh:
    // the downloaded model has no addressable lamp geometry, and hunting for it
    // would tie the lighting to one particular OBJ.
    //
    // Every lamp is drawn TWICE - a dark lens always there, and a lit face on top
    // scaled by brightness. An unlit lamp is a dark glass rectangle, not nothing;
    // drawing nothing makes the car change shape every time it blinks.
    struct Lamp3D
    {
        Vec3    at{ 0.0f, 0.0f, 0.0f };
        Float32 halfW = 0.0f;
        Float32 halfH = 0.0f;
        Bool    facingFront = true;
        ImU32   hue = 0u;
        Float32 level = 0.0f;
    };

    Void drawLamp(const View& v, const Lamp3D& L)
    {
        const Float32 x = L.at.x, y = L.at.y, z = L.at.z;
        const Float32 halfW = L.halfW, halfH = L.halfH;
        const Bool    facingFront = L.facingFront;
        const ImU32   hue = L.hue;
        const Float32 level = L.level;
        // A hair proud of the bodywork, or the lens z-fights the panel.
        const Float32 out = facingFront ? 1.5f : -1.5f;

        const Array<Vec3, 4> q = {
            Vec3{ x - halfW, y + out, z - halfH },
            Vec3{ x + halfW, y + out, z - halfH },
            Vec3{ x + halfW, y + out, z + halfH },
            Vec3{ x - halfW, y + out, z + halfH },
        };

        // The lens: always drawn, always dark.
        pushFace(v, q, { IM_COL32(0x14, 0x16, 0x1A, 0xFF), 0u, 0.0f });

        if(level <= 0.01f)
        {
            return;
        }

        const Int32 r = static_cast<Int32>(((hue >> IM_COL32_R_SHIFT) & 0xFFu) * level);
        const Int32 g = static_cast<Int32>(((hue >> IM_COL32_G_SHIFT) & 0xFFu) * level);
        const Int32 b = static_cast<Int32>(((hue >> IM_COL32_B_SHIFT) & 0xFFu) * level);

        const Array<Vec3, 4> lit = {
            Vec3{ x - halfW, y + out * 1.6f, z - halfH },
            Vec3{ x + halfW, y + out * 1.6f, z - halfH },
            Vec3{ x + halfW, y + out * 1.6f, z + halfH },
            Vec3{ x - halfW, y + out * 1.6f, z + halfH },
        };
        pushFace(v, lit, { IM_COL32(r, g, b, 0xFF), 0u, 0.0f });

        // A soft bloom: a lit lamp reads by spilling, not by being brighter.
        const Float32 gw = halfW * 2.1f, gh = halfH * 2.4f;
        const Array<Vec3, 4> glow = {
            Vec3{ x - gw, y + out * 2.2f, z - gh },
            Vec3{ x + gw, y + out * 2.2f, z - gh },
            Vec3{ x + gw, y + out * 2.2f, z + gh },
            Vec3{ x - gw, y + out * 2.2f, z + gh },
        };
        const Int32 ga = static_cast<Int32>(70.0f * level);
        pushFace(v, glow, { IM_COL32(r, g, b, ga), 0u, 0.0f });
    }

    // The whole cluster set. The rear layout follows docs/conventions.md: red main
    // lamp, amber indicator, and the reverse lamp NESTED INSIDE the indicator
    // housing - small and inboard of the amber, not a fourth unit in a row.
    Void drawLamps(const View& v, const lights::Lamps& L)
    {
        const Float32 hw = EGO_WID_MM * 0.5f;
        const Float32 hl = EGO_LEN_MM * 0.5f;
        const Float32 hz = EGO_HEIGHT_MM;

        const ImU32 WHITE = IM_COL32(0xFF, 0xF4, 0xD8, 0xFF);
        const ImU32 AMBER = IM_COL32(0xFF, 0xA8, 0x18, 0xFF);
        const ImU32 RED = IM_COL32(0xFF, 0x2A, 0x1E, 0xFF);

        // ---- front ----
        const Float32 fz = hz * 0.30f;
        drawLamp(
            v,
            { .at = Vec3{ -hw * 0.52f, hl, fz }, .halfW = 20.0f, .halfH = 9.0f, .facingFront = true, .hue = WHITE, .level = L.headL }
        );
        drawLamp(
            v,
            { .at = Vec3{ hw * 0.52f, hl, fz }, .halfW = 20.0f, .halfH = 9.0f, .facingFront = true, .hue = WHITE, .level = L.headR }
        );
        drawLamp(
            v,
            { .at = Vec3{ -hw * 0.82f, hl, fz }, .halfW = 9.0f, .halfH = 7.0f, .facingFront = true, .hue = AMBER, .level = L.indFL }
        );
        drawLamp(
            v,
            { .at = Vec3{ hw * 0.82f, hl, fz }, .halfW = 9.0f, .halfH = 7.0f, .facingFront = true, .hue = AMBER, .level = L.indFR }
        );

        // ---- rear ----
        const Float32 rz = hz * 0.42f;
        drawLamp(
            v,
            { .at = Vec3{ -hw * 0.50f, -hl, rz }, .halfW = 18.0f, .halfH = 10.0f, .facingFront = false, .hue = RED, .level = L.tailL }
        );
        drawLamp(
            v,
            { .at = Vec3{ hw * 0.50f, -hl, rz }, .halfW = 18.0f, .halfH = 10.0f, .facingFront = false, .hue = RED, .level = L.tailR }
        );
        drawLamp(
            v,
            { .at = Vec3{ -hw * 0.80f, -hl, rz }, .halfW = 10.0f, .halfH = 9.0f, .facingFront = false, .hue = AMBER, .level = L.indRL }
        );
        drawLamp(
            v,
            { .at = Vec3{ hw * 0.80f, -hl, rz }, .halfW = 10.0f, .halfH = 9.0f, .facingFront = false, .hue = AMBER, .level = L.indRR }
        );

        // Nested inside the indicator housing - small, inboard, low.
        drawLamp(
            v,
            { .at = Vec3{ -hw * 0.80f, -hl, rz - 5.0f }, .halfW = 4.5f, .halfH = 3.5f, .facingFront = false, .hue = WHITE, .level = L.revL }
        );
        drawLamp(
            v,
            { .at = Vec3{ hw * 0.80f, -hl, rz - 5.0f }, .halfW = 4.5f, .halfH = 3.5f, .facingFront = false, .hue = WHITE, .level = L.revR }
        );
    }

    Void drawSensor(const View& v, Float32 dpi, Float32 atX, Float32 atY, Float32 atZ)
    {
        constexpr Float32 BASE_MM = vehicle::C1_BASE_MM;
        constexpr Float32 TALL_MM = vehicle::C1_TALL_MM;
        constexpr Float32 PLINTH_H = 16.0f;    // the fixed lower half
        constexpr Float32 LAMP_HEAD_R = 24.0f;    // the spinning head

        const Float32 hb = BASE_MM * 0.5f;

        const ImU32 body = IM_COL32(0x2A, 0x2E, 0x34, 0xFF);
        const ImU32 top = IM_COL32(0x3A, 0x40, 0x48, 0xFF);
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

        // Double-braced: outer pair is the Array of faces, inner one each face's
        // four points. A single pair is one initializer too few.
        const Array<Array<Vec3, 4>, 4> sides = {{
            { b0, b1, t1, t0 }, { b1, b2, t2, t1 },
            { b2, b3, t3, t2 }, { b3, b0, t0, t3 },
        }};
        for(Int32 i = 0; i < 4; ++i)
        {
            pushFace(v, sides[i], { body, edge, 1.0f * dpi });
        }

        const Array<Vec3, 4> cap = { t0, t1, t2, t3 };
        pushFace(v, cap, { top, edge, 1.0f * dpi });

        // The head: the part that turns, and the only cue that this is a scanner.
        constexpr Int32 SEG = 16;
        Array<Vec3, SEG> lo;
        Array<Vec3, SEG> hi;
        for(Int32 i = 0; i < SEG; ++i)
        {
            const Float32 a = static_cast<Float32>(i) * (2.0f * PI_F / SEG);
            const Float32 x = LAMP_HEAD_R * std::cos(a);
            const Float32 y = LAMP_HEAD_R * std::sin(a);
            lo[i] = at(x, y, PLINTH_H);
            hi[i] = at(x, y, TALL_MM);
        }
        for(Int32 i = 0; i < SEG; ++i)
        {
            const Int32 j = (i + 1) % SEG;
            const Array<Vec3, 4> q = { lo[i], lo[j], hi[j], hi[i] };
            pushFace(v, q, { head, 0u, 0.0f });
        }

        const Vec3 crown = at(0.0f, 0.0f, TALL_MM);
        for(Int32 i = 0; i < SEG; ++i)
        {
            emitTri(crown, hi[i], hi[(i + 1) % SEG], top, nullptr, 0);
        }

        // The emitter window, marking which way bearing 0 points.
        const Vec3 w0 = at(-10.0f, LAMP_HEAD_R * 0.98f, PLINTH_H + 6.0f);
        const Vec3 w1 = at( 10.0f, LAMP_HEAD_R * 0.98f, PLINTH_H + 6.0f);
        const Vec3 w2 = at( 10.0f, LAMP_HEAD_R * 0.98f, TALL_MM - 6.0f);
        const Vec3 w3 = at(-10.0f, LAMP_HEAD_R * 0.98f, TALL_MM - 6.0f);
        const Array<Vec3, 4> win = { w0, w1, w2, w3 };
        pushFace(v, win, { IM_COL32(0x8A, 0x1E, 0x1E, 0xFF), 0u, 0.0f });
    }

    Void drawCarFallback(const View& v, Float32 dpi)
    {
        const Float32 hw = EGO_WID_MM * 0.5f;
        const Float32 hl = EGO_LEN_MM * 0.5f;
        const Float32 hz = EGO_HEIGHT_MM;

        // The car is NOT measured data: a solid shell, where returns are outlines.
        const ImU32 shell = IM_COL32(0x2A, 0x33, 0x3E, 0xFF);
        const ImU32 upper = IM_COL32(0x36, 0x41, 0x4E, 0xFF);
        const ImU32 glass = IM_COL32(0x18, 0x2A, 0x38, 0xFF);
        const ImU32 edge = IM_COL32(0xC8, 0xD2, 0xDC, 0xC0);
        const Float32 ew = 1.1f * dpi;

        // Wheels first: inboard of the shoulders, and the depth test interleaves.
        const Float32 wr = EGO_WHEEL_D_MM * 0.5f;
        const Float32 ww = EGO_WHEEL_W_MM * 0.5f;
        const Float32 ax = EGO_WHEELBASE_MM * 0.5f;
        const Float32 tx = EGO_TREAD_MM * 0.5f;
        for(Int32 sx = -1; sx <= 1; sx += 2)
        {
            for(Int32 sy = -1; sy <= 1; sy += 2)
            {
                wheel(
                    v,
                    static_cast<Float32>(sx) * tx,
                    static_cast<Float32>(sy) * ax,
                    wr,
                    ww,
                    IM_COL32(0x1E, 0x1E, 0x1E, 0xFF),
                    IM_COL32(0x8A, 0x90, 0x96, 0xFF),
                    dpi
                );
            }
        }

        // The shell, lofted station to station.
        Vec3 loop[STATION_N][SECTION_N];
        for(Int32 i = 0; i < STATION_N; ++i)
        {
            sectionLoop(STATIONS[i], hw, hl, hz, loop[i]);
        }

        for(Int32 i = 0; i < STATION_N - 1; ++i)
        {
            for(Int32 k = 0; k < SECTION_N; ++k)
            {
                const Int32 k2 = (k + 1) % SECTION_N;

                // Roof panel lighter, the two screen bays glass - panels that only
                // exist because the sections differ, which is why this is lofted.
                ImU32 col = shell;
                if(k == 2)                                      // roof / bonnet top
                {
                    col = upper;
                }
                if(k == 2 && (i == 3 || i == 5))                // screens
                {
                    col = glass;
                }

                const Array<Vec3, 4> q = { loop[i][k], loop[i][k2],
                                    loop[i + 1][k2], loop[i + 1][k] };
                pushFace(v, q, { col, edge, ew });
            }
        }

        // Caps, so the nose and tail are closed. The fan triangles carry NO edge:
        // outlining each drew a star across both ends, because the spokes are a
        // triangulation artifact. The rim is outlined separately, from the loop.
        for(Int32 e = 0; e < 2; ++e)
        {
            const Vec3* L = loop[e == 0 ? 0 : STATION_N - 1];
            const Vec3 mid{ 0.0f, L[0].y, (L[0].z + L[2].z) * 0.5f };

            for(Int32 k = 0; k < SECTION_N; ++k)
            {
                const Int32 k2 = (k + 1) % SECTION_N;
                const Array<Vec3, 3> t = { mid, L[k], L[k2] };
                pushFace(v, t, { shell, 0u, 0.0f });
            }
            for(Int32 k = 0; k < SECTION_N; ++k)
            {
                const Int32 k2 = (k + 1) % SECTION_N;
                const Array<Vec3, 3> o2 = { L[k], L[k2], L[k2] };
                pushFace(v, o2, { 0u, edge, ew });
            }
        }

        // Two amber at the nose, two red at the tail: which end is which, at any
        // orbit angle.
        const auto lamp = [&](Float32 x, Float32 y, Float32 z, ImU32 col) {
            const Float32 s = 22.0f;
            const Array<Vec3, 4> q = { Vec3{ x - s, y, z - s * 0.6f },
                                Vec3{ x + s, y, z - s * 0.6f },
                                Vec3{ x + s, y, z + s * 0.6f },
                                Vec3{ x - s, y, z + s * 0.6f } };
            pushFace(v, q, { col, 0u, 0.0f });
        };
        lamp(-hw * 0.42f, hl * 0.99f, hz * 0.22f, IM_COL32(0xFF, 0xE0, 0x90, 0xFF));
        lamp( hw * 0.42f, hl * 0.99f, hz * 0.22f, IM_COL32(0xFF, 0xE0, 0x90, 0xFF));
        lamp(-hw * 0.50f, -hl * 0.99f, hz * 0.34f, IM_COL32(0xE0, 0x28, 0x20, 0xFF));
        lamp( hw * 0.50f, -hl * 0.99f, hz * 0.34f, IM_COL32(0xE0, 0x28, 0x20, 0xFF));

        // The rear wing: settles front/back from above, where lights are edge-on.
        {
            const Float32 y = -hl * 0.92f;
            const Float32 z = hz * 0.78f;
            const Float32 sw = hw * 0.86f;
            const Array<Vec3, 4> q = { Vec3{ -sw, y - 26.0f, z }, Vec3{ sw, y - 26.0f, z },
                                Vec3{  sw, y + 26.0f, z }, Vec3{ -sw, y + 26.0f, z } };
            pushFace(v, q, { upper, edge, ew });

            for(Int32 sx = -1; sx <= 1; sx += 2)
            {
                const Float32 px = static_cast<Float32>(sx) * sw * 0.7f;
                const Array<Vec3, 4> p = { Vec3{ px, y, hz * 0.52f }, Vec3{ px, y, z },
                                    Vec3{ px, y + 10.0f, z }, Vec3{ px, y + 10.0f, hz * 0.52f } };
                pushFace(v, p, { shell, edge, ew * 0.8f });
            }
        }
    }

    // ---- modes ----------------------------------------------------------------
    // THERE IS NO CHASSIS FILTER, deliberately. Discarding returns inside the car's
    // footprint deleted the easiest check that the device is alive - a hand cupped
    // around the sensor sits well inside 430 x 190 mm. The depth buffer makes the
    // filter unnecessary; to see past the car, hide the CAR.
    Int32 drawReturns(const View& v, const DrawArgs& a, Bool solid, Int32* hidden)
    {
        if(hidden != nullptr)
        {
            *hidden = 0;
        }
        if(a.points == nullptr)
        {
            return 0;
        }

        const Float32 deg2rad = PI_F / 180.0f;
        Int32 n = 0;

        for(const LidarPoint& p : *a.points)
        {
            if(!(p.distMm >= MIN_VALID_MM) || p.distMm > MAX_VALID_MM)
            {
                continue;
            }

            // 2D map bearing: 0 = forward, clockwise. Scene: +y forward, +x right.
            const Float32 ang = p.angleDeg * deg2rad;
            const Float32 x = p.distMm * std::sin(ang);
            const Float32 y = p.distMm * std::cos(ang);

            ++n;

            if(solid)
            {
                column(
                    v,
                    x,
                    y,
                    COLUMN_W_MM * 0.5f,
                    COLUMN_MM,
                    IM_COL32(0xC8, 0xC8, 0xC8, 0x9A),
                    IM_COL32(0xFF, 0xFF, 0xFF, 0xD0),
                    IM_COL32(0xFF, 0xFF, 0xFF, 0x50),
                    1.0f * a.dpi
                );
            }
            else
            {
                // A pin: a stalk and a head. The head is a camera-facing quad IN THE
                // SCENE - as a screen-space circle on the ImGui list it was not
                // depth-tested, so the car covered pins that were in front of it.
                const Vec3 g{ x, y, 0.0f }, t{ x, y, COLUMN_MM };
                ribbon(v, g, t, IM_COL32(0xFF, 0xFF, 0xFF, 0x55), 1.0f * a.dpi);

                const Float32 hr = v.pxToWorld(t, 3.0f * a.dpi) * 0.5f;
                const Vec3 hx = mul(v.right, hr);
                const Vec3 hy = mul(v.up, hr);
                const Array<Vec3, 4> head = { sub(sub(t, hx), hy), add(sub(t, hy), hx),
                                       add(add(t, hx), hy), sub(add(t, hy), hx) };
                pushFace(v, head, { IM_COL32(0xFF, 0xFF, 0xFF, 0xFF), 0u, 0.0f });
            }
        }
        return n;
    }

    Int32 drawWalls(const View& v, const DrawArgs& a)
    {
        if(a.walls == nullptr)
        {
            return 0;
        }

        const ImU32 face = (ui::ansi::BRCYAN & 0x00FFFFFFu) | (0x2Eu << IM_COL32_A_SHIFT);
        const ImU32 edge = (ui::ansi::BRCYAN & 0x00FFFFFFu) | (0xE0u << IM_COL32_A_SHIFT);

        for(const mapgeo::WallSeg& w : *a.walls)
        {
            const Vec3 a0 = toScene(w.a.x, w.a.y, 0.0f);
            const Vec3 b0 = toScene(w.b.x, w.b.y, 0.0f);
            const Vec3 a1 = toScene(w.a.x, w.a.y, WALL_H_MM);
            const Vec3 b1 = toScene(w.b.x, w.b.y, WALL_H_MM);

            const Array<Vec3, 4> q = { a0, b0, b1, a1 };
            pushFace(v, q, { face, edge, 1.6f * a.dpi });
        }
        return static_cast<Int32>(a.walls->size());
    }

    Void drawFitFloor(const View& v, const DrawArgs& a)
    {
        if(a.reach == nullptr || a.reachN < 3)
        {
            return;
        }

        const ImU32 face = (ui::ansi::BRGREEN & 0x00FFFFFFu) | (0x2Cu << IM_COL32_A_SHIFT);
        const ImU32 edge = (ui::ansi::BRGREEN & 0x00FFFFFFu) | (0xC0u << IM_COL32_A_SHIFT);

        const Vec3 hub{ 0.0f, 0.0f, 1.0f };   // a hair above the grid, not in it

        for(Int32 i = 0; i < a.reachN; ++i)
        {
            const Int32 j = (i + 1) % a.reachN;

            // Bin centers, clearance-map convention: bearing 0 forward, +y here.
            const Float32 ai = (static_cast<Float32>(i) + 0.5f) * a.reachBinDeg * (PI_F / 180.0f);
            const Float32 aj = (static_cast<Float32>(j) + 0.5f) * a.reachBinDeg * (PI_F / 180.0f);

            const Vec3 pi{ a.reach[i] * std::sin(ai), a.reach[i] * std::cos(ai), 1.0f };
            const Vec3 pj{ a.reach[j] * std::sin(aj), a.reach[j] * std::cos(aj), 1.0f };

            const Array<Vec3, 3> t = { hub, pi, pj };
            pushFace(v, t, { face, 0u, 0.0f });

            const Array<Vec3, 3> e = { pi, pj, pj };
            pushFace(v, e, { 0u, edge, 1.6f * a.dpi });
        }
    }

    // How the frame of reference reads in the corner. In a world frame the heading
    // is a MEASUREMENT and carries its confidence: below about a third the room is
    // too near rotationally symmetric to name a direction, so it says "unsure".
    const Char* lockNote(const DrawArgs& a, Float32 yaw, Char* buf, Size cap)
    {
        if(a.worldHeadingOk < 0.0f)
        {
            std::snprintf(buf, cap, "car lock");
            return buf;
        }
        // Folded to (-180, 180] and snapped through zero, so -0.3 prints as "0".
        while(yaw >  180.0f)
        {
            yaw -= 360.0f;
        }
        while(yaw <= -180.0f)
        {
            yaw += 360.0f;
        }
        if(yaw > -0.5f && yaw < 0.5f)
        {
            yaw = 0.0f;
        }

        if(a.worldHeadingOk < 0.33f)
        {
            std::snprintf(
                buf,
                cap,
                "world lock, heading unsure (%.0f%%)",
                static_cast<Float64>(a.worldHeadingOk * 100.0f)
            );
        }
        else
        {
            std::snprintf(
                buf,
                cap,
                "world lock, %.0f deg (%.0f%%)",
                static_cast<Float64>(yaw),
                static_cast<Float64>(a.worldHeadingOk * 100.0f)
            );
        }
        return buf;
    }

    Void say(const DrawArgs& a, const Char* fmt, ...)
    {
        if(a.diag == nullptr || a.diagCap == 0)
        {
            return;
        }
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(a.diag, a.diagCap, fmt, ap);
        va_end(ap);
    }

    constexpr SceneModeInfo SCENE_INFO[static_cast<Size>(SceneMode::SCENE_MODE_COUNT)] = {
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

  }

  // ---------------------------------------------------------------------------

  Void Camera::orbit(Float32 dYaw, Float32 dPitch)
  {
      yaw += dYaw;
      while(yaw >  PI_F)
      {
          yaw -= 2.0f * PI_F;
      }
      while(yaw < -PI_F)
      {
          yaw += 2.0f * PI_F;
      }

      // Never past vertical: at the poles `up` degenerates and the ground flips.
      pitch += dPitch;
      if(pitch <  0.02f)
      {
          pitch = 0.02f;
      }
      if(pitch >  1.52f)
      {
          pitch = 1.52f;
      }
  }

  Void Camera::pan(Float32 dRight, Float32 dUp)
  {
      // Locked to the car means locked. See Camera::lockToCar.
      if(lockToCar)
      {
          return;
      }

      // In the ground plane, so panning never lifts the scene off the floor.
      const Float32 s = std::sin(yaw), c = std::cos(yaw);
      target.x += dRight * c + dUp * s;
      target.y += dRight * -s + dUp * c;
  }

  Void Camera::zoom(Float32 factor)
  {
      dist *= factor;
      if(dist <  400.0f)
      {
          dist = 400.0f;
      }
      if(dist > 26000.0f)
      {
          dist = 26000.0f;
      }
  }

  const SceneModeInfo& sceneModeInfo(SceneMode m) noexcept
  {
      const Size i = static_cast<Size>(m);
      if(i >= static_cast<Size>(SceneMode::SCENE_MODE_COUNT))
      {
          return SCENE_INFO[0];
      }
      return SCENE_INFO[i];
  }

  const Char* sceneModeName(SceneMode m) noexcept
  {
      return sceneModeInfo(m).name;
  }

  Void draw(const Camera& cam, const DrawArgs& a)
  {
      if(a.dl == nullptr || (a.p1.x - a.p0.x) < 32.0f || (a.p1.y - a.p0.y) < 32.0f)
      {
          return;
      }

      const View v = makeView(cam, a.p0, a.p1, a.worldYawDeg);

      a.dl->PushClipRect(a.p0, a.p1, true);

      // Rendered off-screen with a depth buffer and composited as one image. If
      // that cannot be set up the viewport goes black and says so, rather than
      // silently degrading to the wrong answer.
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
      {
          drawRideGround(v, a);
      }
      else if(sceneWantsSquares(a.mode))
      {
          drawGroundSquares(a.dl, v, a.dpi);
      }
      else
      {
          drawGround(a.dl, v, a.dpi);
      }

      Int32 n = 0;
      Int32 hidden = 0;
      Array<Char, 64> lockBuf= {};
      switch(a.mode)
      {
      case SceneMode::SCENE_MODE_BLOCKS:
          n = drawReturns(v, a, true, &hidden);
          say(
              a,
              "%d returns as columns%s | %s, orbit %.0f deg, %.1f m out",
              n,
              "",
              lockNote(a, a.worldYawDeg, lockBuf.data(), lockBuf.size()),
              static_cast<Float64>(cam.yaw * 180.0f / PI_F),
              static_cast<Float64>(cam.dist / 1000.0f)
          );
          break;

      case SceneMode::SCENE_MODE_WALLS:
          n = drawWalls(v, a);
          drawReturns(v, a, false, &hidden);
          say(
              a,
              "%d wall%s as panels%s | %s, orbit %.0f deg, %.1f m out",
              n,
              n == 1 ? "" : "s",
              "",
              lockNote(a, a.worldYawDeg, lockBuf.data(), lockBuf.size()),
              static_cast<Float64>(cam.yaw * 180.0f / PI_F),
              static_cast<Float64>(cam.dist / 1000.0f)
          );
          break;

      case SceneMode::SCENE_MODE_FIT:
          drawFitFloor(v, a);
          n = drawReturns(v, a, false, &hidden);
          say(
              a,
              "drivable floor from %d returns | %s, orbit %.0f deg, %.1f m out",
              n,
              lockNote(a, a.worldYawDeg, lockBuf.data(), lockBuf.size()),
              static_cast<Float64>(cam.yaw * 180.0f / PI_F),
              static_cast<Float64>(cam.dist / 1000.0f)
          );
          break;

      case SceneMode::SCENE_MODE_FULL:
          // No columns, no wall panels: this mode shows the CONCLUSIONS.
          drawPathRibbon(v, a);
          for(Int32 i = 0; i < a.objectN; ++i)
          {
              drawDetection(v, a.objects[i], a.dpi);
          }
          n = a.objectN;
          say(
              a,
              "%d object%s | %.2f m ahead",
              n,
              n == 1 ? "" : "s",
              static_cast<Float64>(a.aheadMm / 1000.0f)
          );
          break;

      case SceneMode::SCENE_MODE_CLOUD:
      case SceneMode::SCENE_MODE_COUNT:
      default:
          n = drawReturns(v, a, false, &hidden);
          say(
              a,
              "%d returns%s | %s, orbit %.0f deg, %.1f m out",
              n,
              "",
              lockNote(a, a.worldYawDeg, lockBuf.data(), lockBuf.size()),
              static_cast<Float64>(cam.yaw * 180.0f / PI_F),
              static_cast<Float64>(cam.dist / 1000.0f)
          );
          break;
      }

      // The downloaded model when there, the hand-built shell when not: a missing
      // asset costs fidelity, never the car - the origin needs something on it.
      if(a.ego == EgoView::EGO_VIEW_SENSOR)
      {
          // On its own. The point of this mode is the SIZE: 56 mm vs a 442 mm car.
          drawSensor(v, a.dpi, 0.0f, 0.0f, 0.0f);
      }
      else
      {
          const CarModel& car = carModel();
          if(car.loaded)
          {
              drawCarModel(v, car, a.dpi);
          }
          else
          {
              drawCarFallback(v, a.dpi);
          }

          // And the sensor ON it, at its mount: every measurement here is in the
          // sensor's frame, so where it sits on the car is what the picture is
          // built on.
          drawSensor(
              v,
              a.dpi,
              vehicle::C1_MOUNT_LATERAL_MM,
              vehicle::C1_MOUNT_AHEAD_MM,
              vehicle::C1_MOUNT_BASE_MM
          );

          drawLamps(v, a.lamps);
      }

      const ImTextureID img = scenegpu::end(v.mvp.data());
      if(img != 0)
      {
          a.dl->AddImage(img, a.p0, a.p1);
      }
      else
      {
          a.dl->AddRectFilled(a.p0, a.p1, ui::ansi::BLACK);
      }

      a.dl->PopClipRect();
  }

}
