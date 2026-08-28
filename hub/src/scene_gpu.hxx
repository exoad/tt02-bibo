// ---------------------------------------------------------------------------
// A real depth buffer for the 3D scene.
//
// WHY THIS EXISTS. The 3D view started as a painter's algorithm: project every
// face to screen space, sort by centroid depth, draw back to front into the same
// ImDrawList the rest of the UI uses. That is correct for a flat ground and for
// disjoint columns, and it is WRONG for a car - which is what it was asked to
// draw. Two failures, both reported from screenshots:
//
//   * The shell self-clipped. A car body is not convex: the greenhouse is inset
//     into the shoulders and the spoiler passes through the tail. Triangles that
//     interpenetrate have NO correct draw order, so no sort could have fixed it.
//   * The car covered point-cloud pins that were plainly in front of it. Those
//     pins were drawn with immediate ImDrawList calls that never entered the
//     sort queue at all, so they were always behind everything that did.
//
// Both are the same shape of problem - visibility decided per PRIMITIVE instead
// of per PIXEL - and both go away with a depth buffer. So the scene renders
// through D3D11 into an off-screen target and the result is composited back as a
// single ImGui image. Depth-correct, and it drops the whole sort.
//
// Off-screen rather than straight to the back buffer so the result layers with
// ImGui normally: the HUD, the labels and the floating panels all still draw
// over it without knowing anything about D3D.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

#include "imgui.h"

struct ID3D11Device;
struct ID3D11DeviceContext;

namespace scenegpu {

// Position in world space, colour already lit (flat shading is done on the CPU,
// where the face normals are), and a texture coordinate.
struct Vertex
{
    Float32 x = 0.0f, y = 0.0f, z = 0.0f;
    ImU32   col = 0xFFFFFFFFu;
    Float32 u = 0.0f, v = 0.0f;
};

Void init(ID3D11Device* device, ID3D11DeviceContext* context);
Void shutdown();

[[nodiscard]] Bool ready() noexcept;

// Starts a frame at this pixel size. Returns false if the target could not be
// made, in which case the caller must not emit anything.
Bool begin(Int32 widthPx, Int32 heightPx);

// OPAQUE triangles are depth-tested and depth-written, in any order.
//
// TRANSLUCENT ones are depth-tested but do not WRITE depth, and are sorted back
// to front among themselves before drawing. A depth buffer alone cannot resolve
// transparency - it decides one winner per pixel, and a see-through surface has
// no business being a winner - so the sort survives here and only here.
Void addOpaque(const Vertex& a, const Vertex& b, const Vertex& c, ImTextureID tex);
Void addBlended(const Vertex& a, const Vertex& b, const Vertex& c, ImTextureID tex);

// Draws everything and returns the texture to composite. `mvp` is row-major and
// row-vector: the shader computes mul(float4(pos, 1), mvp).
[[nodiscard]] ImTextureID end(const Float32* mvp);

} // namespace scenegpu
