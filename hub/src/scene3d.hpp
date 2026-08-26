// ---------------------------------------------------------------------------
// The 3D view: an orbit camera, the car in the middle, the scan on the ground.
//
// Software projection into an ImDrawList rather than a second D3D11 pass. The
// scene is a few thousand quads - a scan is ~500 returns and the car is about
// sixty faces - which is nothing for the vertex path already used by the 2D map,
// and the alternative is shaders, a depth buffer, render-to-texture and a second
// resize path for a scene that would still be sorted by hand.
//
// Depth is a painter's sort on primitive centroids. That is exact for the ground
// (it is flat), correct for the columns (they are disjoint), and can only be
// wrong where two faces of the car interpenetrate - which they do not, because
// it is convex per part.
//
// AXES, and they differ from the 2D map's on purpose:
//     x  right
//     y  FORWARD  (the 2D map's -y, since its y runs down the screen)
//     z  UP
// Everything crossing over goes through toScene(), so the flip lives in exactly
// one place.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hpp"
#include "lidar_source.hpp"
#include "lights.hpp"
#include "map_geometry.hpp"

#include "imgui.h"

#include <deque>
#include <vector>

namespace scene3d {

struct Vec3 { Float32 x = 0.0f, y = 0.0f, z = 0.0f; };

// A 2D map point (x right, y DOWN) at ground level, in scene axes.
[[nodiscard]] inline Vec3 toScene(Float32 mapX, Float32 mapY, Float32 up = 0.0f)
{
    return Vec3{ mapX, -mapY, up };
}

// ---------------------------------------------------------------------------
// The orbit camera. Angles in radians, distances in millimetres.
// ---------------------------------------------------------------------------
struct Camera
{
    Float32 yaw   = 0.0f;         // 0 looks along +y, i.e. from behind the car
    Float32 pitch = 0.42f;        // above the horizon
    Float32 dist  = 4200.0f;
    Vec3    target{ 0.0f, 0.0f, 120.0f };
    Float32 fovY  = 0.85f;        // radians

    // What the camera is anchored to.
    //
    // CAR: the target is pinned to the car, so it never leaves the middle of
    // the screen however far you orbit or zoom. Panning is inert - that is what
    // "locked" means, and a pan that silently un-locked the thing you locked
    // would be worse than one that does nothing.
    //
    // WORLD: the target is yours. Pan anywhere; the car can leave frame.
    //
    // Today the two differ only in whether the target follows the car, because
    // every scan is in the sensor's own frame and the car sits at its origin.
    // Once there is a pose estimate - odometry, or scan matching - CAR will also
    // inherit the car's heading and WORLD will hold a fixed bearing while the
    // car turns underneath it. The switch is here now because the ANCHORING
    // half is real now; the heading half is not faked in the meantime.
    Bool lockToCar = true;

    // Kept inside the sane range: past vertical the up vector degenerates and
    // the view flips, and no amount of dragging should be able to invert a
    // ground plane.
    Void orbit(Float32 dYaw, Float32 dPitch);
    Void pan(Float32 dRight, Float32 dUp);
    Void zoom(Float32 factor);
};

// What the 3D view is showing. Its own list, deliberately: these answer
// questions the flat map cannot, and the 2D modes answer questions that do not
// need a camera.
enum class SceneMode
{
    SCENE_MODE_CLOUD = 0,   // returns as pins standing on the ground
    SCENE_MODE_BLOCKS,      // returns as solid columns - the scan as geometry
    SCENE_MODE_WALLS,       // fitted surfaces extruded into standing panels
    SCENE_MODE_FIT,         // the floor the car can actually reach
    SCENE_MODE_FULL,        // all of it at once, with the numbers
    SCENE_MODE_COUNT
};

struct SceneModeInfo
{
    const Char* name;
    const Char* what;
    const Char* read;
};

[[nodiscard]] const SceneModeInfo& sceneModeInfo(SceneMode m) noexcept;
[[nodiscard]] const Char*          sceneModeName(SceneMode m) noexcept;

// One detected thing. Deliberately NOT classified: a planar lidar with no
// classifier cannot tell a chair from a person, and the ride view draws every
// detection as the same plain box for exactly that reason. Waymo's display
// shows pedestrian and cyclist figures because Waymo has a classifier to earn
// them with.
// What sits at the origin.
//
// CAR is the TT-02 the sensor is going to be bolted to. SENSOR is the RPLIDAR
// C1 on its own, to scale - 55.6 x 55.6 x 41.3 mm, from the datasheet figures
// in lidar/README.md.
//
// The distinction is worth a control rather than a constant because right now
// SENSOR is the TRUE picture: the C1 is on a desk and there is no car. Drawing
// a 430 mm shell around a 56 mm puck is a statement about the future, and there
// should be a way to ask what is actually there.
enum class EgoView
{
    EGO_VIEW_CAR = 0,
    EGO_VIEW_SENSOR,
};

struct Detection
{
    Float32 cx = 0.0f, cy = 0.0f;      // centre, mm, sensor frame (y DOWN)
    Float32 ux = 1.0f, uy = 0.0f;      // principal axis
    Float32 halfL = 0.0f, halfW = 0.0f;
    Float32 nearMm = 0.0f;
    Bool    inPath = false;
};

// Everything the renderer needs that is not the camera.
struct DrawArgs
{
    ImDrawList* dl   = nullptr;
    ImVec2      p0, p1;          // widget rect, screen space
    Float32     dpi  = 1.0f;
    SceneMode   mode = SceneMode::SCENE_MODE_CLOUD;

    const std::vector<LidarPoint>* points = nullptr;   // current revolution
    const std::vector<mapgeo::WallSeg>* walls = nullptr;

    // Reachable radius per bearing bin, for SCENE_MODE_FIT. Null when unknown.
    const Float32* reach   = nullptr;
    Int32          reachN  = 0;
    Float32        reachBinDeg = 0.0f;

    // The car is a REFERENCE object, not data. It is the only thing in the scene
    // that can hide a measurement, so when you want to see what is under it the
    // answer is to draw the sensor instead - not to delete the returns, which is
    // what the old chassis filter did.
    EgoView ego = EgoView::EGO_VIEW_CAR;

    // Numbers for SCENE_MODE_FULL's panel. The scene has no way to compute
    // these - they belong to the flat map's accumulators - so they are handed
    // in rather than recomputed differently here.
    Float32 hz        = 0.0f;
    Int32   returns   = 0;
    Float32 nearestMm = 0.0f;
    Float32 aheadMm   = 0.0f;

    // Detections and the corridor, for the ride view.
    const Detection* objects = nullptr;
    Int32            objectN = 0;
    Float32          corridorHalfW = 0.0f;
    Float32          corridorFree  = 0.0f;

    // Lamp brightness, already solved. The renderer does not know the RULES -
    // it is handed ten numbers and draws ten lamps - so the behaviour lives in
    // one tested place instead of being re-derived from what looks right.
    lights::Lamps lamps;

    // Rotation applied to the WHOLE scene, degrees, about the vertical.
    //
    // This is what makes World lock a frame of reference rather than a camera
    // preference. Everything drawn here is in the SENSOR's frame, so rotating
    // all of it by the sensor's heading puts it in the world's: the room stops
    // turning when you turn the device, and the car - which is fixed in the
    // sensor frame - is what visibly rotates instead. One rotation, applied
    // once, in the matrix. Zero in Car lock.
    Float32 worldYawDeg = 0.0f;

    // Confidence in worldYawDeg, 0..1, or NEGATIVE when there is no world frame
    // (Car lock). Reported rather than hidden: a heading matched against a
    // featureless room is a number with nothing behind it, and the display has
    // to be able to say so.
    Float32 worldHeadingOk = -1.0f;

    // Where the readout line goes. Never null.
    Char* diag    = nullptr;
    Size  diagCap = 0;
};

Void draw(const Camera& cam, const DrawArgs& a);

} // namespace scene3d
