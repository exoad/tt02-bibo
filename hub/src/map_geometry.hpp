// ---------------------------------------------------------------------------
// The map's pure geometry: no ImGui, no drawing, no globals.
//
// Split out of radar.cpp so it can be TESTED. The scene the sensor happens to be
// pointing at decides whether a corner exists, and a desk with three short walls
// on it will never produce one - so leaving this inside the renderer meant the
// only way to exercise it was to hope the room cooperated. It did not.
//
// Everything here works in the sensor frame, millimetres, x right and y DOWN -
// the same orientation the screen uses, so nothing has to be flipped on the way
// out.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hpp"

#include <cmath>

namespace mapgeo {

struct WorldPt { Float32 x, y; };

// A straight surface fitted to a run of returns.
struct WallSeg
{
    WorldPt a{ 0.0f, 0.0f }, b{ 0.0f, 0.0f };
    Float32 lenMm = 0.0f;
    Float32 deg   = 0.0f;    // 0 = up, undirected, [0,180)
};

// Where two walls meet.
struct Corner
{
    Float32 x = 0.0f, y = 0.0f;
    Float32 angDeg  = 0.0f;   // the included angle you would see
    Float32 rangeMm = 0.0f;
    Float32 a0 = 0.0f, a1 = 0.0f;   // arm directions, radians, for drawing
};

// Perpendicular distance from p to the line through a and b.
[[nodiscard]] inline Float32 perpDist(const WorldPt& p, const WorldPt& a, const WorldPt& b)
{
    const Float32 dx = b.x - a.x;
    const Float32 dy = b.y - a.y;
    const Float32 len = std::sqrt(dx * dx + dy * dy);
    if(len < 1e-3f)
    {
        const Float32 ex = p.x - a.x, ey = p.y - a.y;
        return std::sqrt(ex * ex + ey * ey);
    }
    return std::fabs(dy * (p.x - a.x) - dx * (p.y - a.y)) / len;
}

// Distance from p to the nearer end of segment [a,b].
[[nodiscard]] inline Float32 endGap(const WorldPt& p, const WorldPt& a, const WorldPt& b)
{
    const Float32 da = std::sqrt((p.x - a.x) * (p.x - a.x) + (p.y - a.y) * (p.y - a.y));
    const Float32 db = std::sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));
    return (da < db) ? da : db;
}

// The bearing of a segment, 0 = up, undirected, folded into [0,180).
[[nodiscard]] inline Float32 segBearingDeg(const WorldPt& a, const WorldPt& b)
{
    Float32 deg = std::atan2(b.x - a.x, -(b.y - a.y)) * 180.0f / 3.14159265358979323846f;
    while(deg <    0.0f) deg += 180.0f;
    while(deg >= 180.0f) deg -= 180.0f;
    return deg;
}

// ---------------------------------------------------------------------------
// Corners.
//
// Two tests, and they are what keep this honest:
//
//   TURN     the walls have to actually turn. A 5 deg join is one surface the
//            fitter happened to split in two, not a corner.
//   ENDPOINT the crossing has to be near an END of BOTH walls. Two walls on
//            opposite sides of a room have lines that meet somewhere out in the
//            car park, and that intersection is not a landmark.
// ---------------------------------------------------------------------------
inline constexpr Float32 CORNER_MIN_DEG = 35.0f;
inline constexpr Float32 CORNER_NEAR_MM = 220.0f;
inline constexpr Size    CORNER_MAX     = 32u;

Void findCorners(const Vec<WallSeg>& w, Vec<Corner>& out,
                 Float32 minRangeMm, Float32 maxRangeMm);

// ---------------------------------------------------------------------------
// Configuration space, in polar form.
//
// `clr[i]` is the free radius on bearing bin i, `seen[i]` whether that bin has
// evidence. `out[i]` becomes how far the CENTRE of a disc of radius halfW could
// travel along bearing i before touching anything.
//
// For an obstacle at (R, dth) off the bearing, the centre first touches it at
//
//     r = R cos(dth) - sqrt(halfW^2 - (R sin(dth))^2)
//
// and cannot touch it at all when |R sin(dth)| >= halfW: its perpendicular
// offset from the bearing already clears the disc. That second condition bounds
// how many bins can possibly block a bearing, which is what keeps this cheap.
// ---------------------------------------------------------------------------
Void computeReach(const Float32* clr, const Bool* seen, Int32 bins, Float32 binDeg,
                  Float32 halfW, Float32* out);

// ---------------------------------------------------------------------------
// Heading, from the scan alone.
//
// A world-locked view needs to know which way the sensor is pointing, and there
// is no odometry, no IMU and no compass on this machine. There is, however, a
// range profile: 120 bins of "how far away is the nearest thing on this
// bearing", which is a signature of the room. Turn the sensor and that
// signature slides along the bearing axis without changing shape.
//
// So: circular cross-correlation against a REFERENCE profile captured when the
// world frame was zeroed. The shift that best aligns them is the heading.
//
// Against a fixed reference rather than frame-to-frame ON PURPOSE. Integrating
// per-frame deltas accumulates drift forever; matching an absolute reference
// does not drift at all. The trade is that it fails when the room stops
// resembling the reference, which is a failure you can see and reason about,
// where drift is one you cannot.
//
// WHAT THIS CANNOT DO, and the caller must not pretend otherwise:
//
//   * Rotation only. Translation is not estimated. Slide the sensor sideways
//     and the profile changes shape rather than shifting, so the answer degrades
//     - which is what `score` is for.
//   * A rotationally symmetric room has no unique answer. A circular room gives
//     the same cost at every shift, and the score collapses accordingly.
//
// `score` is 0..1 separability: how much better the best shift is than a typical
// one. Near zero means "this profile does not identify a direction" and the
// caller should say so rather than draw a confident heading.
// ---------------------------------------------------------------------------
Bool estimateHeading(const Float32* ref, const Bool* refSeen,
                     const Float32* cur, const Bool* curSeen,
                     Int32 bins, Float32 binDeg,
                     Float32& outDeg, Float32& outScore);

// Shoelace area of a polar polygon, in m^2 given radii in mm.
[[nodiscard]] Float32 polarArea(const Float32* r, Int32 bins, Float32 binDeg);

} // namespace mapgeo
