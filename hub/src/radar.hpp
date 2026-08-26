// Interactive 2D map of recent scan revolutions, drawn with ImDrawList.
//
// World space is millimetres with the sensor at the origin, x right, y down on
// screen; a measurement at (angle, dist) maps to (dist*sin a, -dist*cos a), so
// 0 degrees points up and angle increases clockwise.
//
// Mouse gestures (all handled inside draw()):
//   left / middle drag ... pan
//   wheel ................ zoom about the cursor (ctrl = fine, shift = coarse)
//   double click left .... reset to auto-fit
//   right drag ........... measure; reports distance between the two points
#pragma once

#include <deque>
#include <vector>

#include "imgui.h"

#include "lidar_source.hpp"
#include "lights.hpp"
#include "scene3d.hpp"
#include "shared.hpp"

// How the returns are rendered. The geometry, grid, gestures and readouts are
// identical in every mode - only the marks change - so switching is a display
// choice, never a change of what is being measured.
enum class MapMode
{
    // Nine, and every one of them ANSWERS SOMETHING the dots do not already
    // say. That is the bar, and seven modes that did not clear it were removed
    // rather than kept for variety - see docs/log.md. A view that re-paints the
    // same returns in a different colour is not an analysis, and a toggle full
    // of them makes the ones that are harder to find.
    MAP_MODE_POINTS = 0,   // the raw returns. Nothing inferred - the ground truth
    MAP_MODE_DENSITY,      // what stayed put: hit counts in a fixed world grid
    MAP_MODE_MOTION,       // what did not: cells hit now that the map says were empty
    MAP_MODE_CLEARANCE,    // free radius per bearing: how far the BEAM reaches
    MAP_MODE_GAPS,         // openings wide enough to drive through, with widths
    MAP_MODE_WALLS,        // straight segments fitted to the returns
    MAP_MODE_CORNERS,      // where two walls meet: the point landmarks
    MAP_MODE_FIT,          // free space eroded by the chassis: where the CAR fits
    MAP_MODE_FULL,         // the field display: everything, with the numbers on it
    MAP_MODE_MINIMAL,      // for showing people. The room, the car, nothing else
    MAP_MODE_COUNT
};


// ---------------------------------------------------------------------------
// What kind of grid a mode is read against.
//
// Every mode used to get range rings, because the first mode did and the rest
// inherited it. But a grid is a measuring instrument, and which one is right
// depends on what is being measured:
//
//   RADIAL     rings and bearings. Correct when the quantity IS a range and a
//              bearing - Clearance, Gaps, Points.
//   CARTESIAN  metre squares, axis-aligned. Correct when the quantity is a
//              LENGTH or a right angle - Walls, Corners - and honest for
//              Density and Motion, whose cells are literally a fixed world grid.
//              Rings behind a wall you are trying to judge the straightness of
//              are just curves arguing with a line.
//   NONE       for the modes that are not being measured at all.
// ---------------------------------------------------------------------------
enum class GridStyle
{
    GRID_STYLE_RADIAL = 0,
    GRID_STYLE_CARTESIAN,
    GRID_STYLE_NONE,
};

[[nodiscard]] GridStyle mapModeGrid(MapMode m) noexcept;

// Short label for each mode, for the toggle in the UI.
[[nodiscard]] const Char* mapModeName(MapMode m) noexcept;

// What a mode draws and how to read it.
//
// These are not self-explanatory and were never meant to be: each answers a
// different question about the same revolution, and the label alone ("Density",
// "Validity") does not say which question. The toggle shows this on hover.
struct MapModeInfo
{
    const Char* name;
    const Char* what;    // what is on screen
    const Char* read;    // what it is for / how to interpret it
};

[[nodiscard]] const MapModeInfo& mapModeInfo(MapMode m) noexcept;

// The viewer's background for this mode.
//
// Always very dark - the returns must stay the brightest thing on screen - but
// tinted toward the mode's own palette, so which mode is active is legible from
// the map itself rather than only from the toggle. Pure black is still the
// answer for the modes that are about raw geometry.
[[nodiscard]] ImU32 mapModeBackground(MapMode m) noexcept;

class RadarView
{
public:
    // Flat map or 3D scene. The two are genuinely different views of the same
    // revolution with their own mode lists, not one view with a projection
    // switch, so the dimension is chosen first and the overlays follow it.
    Bool is3D = false;

    // Current render mode, per dimension. Kept separately so switching to 3D
    // and back does not lose which flat overlay was up.
    MapMode mode = MapMode::MAP_MODE_POINTS;
    scene3d::SceneMode scene = scene3d::SceneMode::SCENE_MODE_CLOUD;

    // The 3D camera. Persisted across frames because an orbit the user set is
    // state, and resetting it on every revolution would make it unusable.
    scene3d::Camera cam;

    // What is drawn at the origin, in BOTH dimensions. Shared deliberately: it
    // is a statement about the machine, not about a projection, and having the
    // flat map claim a car while the scene shows a bare sensor would be two
    // answers to one question.
    scene3d::EgoView ego = scene3d::EgoView::EGO_VIEW_CAR;

    // The lighting bench's state, set by the Vehicle panel. Lives here with the
    // other pure display state rather than being reached for across a namespace
    // boundary the renderer has no other reason to cross.
    lights::Input lighting;

    // ---- overlays -------------------------------------------------------
    Bool showGrid    = true;
    Bool showTrail   = true;
    Bool showLabels  = true;   // range-ring and compass labels
    Bool showNearest = true;   // ring around the closest return

    // ---- readouts, written by draw(), read by the caller's HUD ----------
    //
    // Plain fields rather than the trivial one-line getters these used to be.
    // The style guide asks for public fields where a getter would only forward,
    // and there is a second reason here: a member and its accessor cannot share
    // a name, so `radiusPx` had to be either the field or the function. Making
    // it the field keeps the name that reads best at the call site, and matches
    // `mode` and the overlay flags directly above, which were always public.
    ImVec2 centerPx = ImVec2(0.0f, 0.0f);
    Float32 radiusPx = 0.0f;

    Bool    cursorValid      = false;
    Float32 cursorRangeMm    = 0.0f;   // from the sensor
    Float32 cursorBearingDeg = 0.0f;

    Bool    hasNearest        = false;
    Float32 nearestMm         = 0.0f;
    Float32 nearestBearingDeg = 0.0f;

    Bool    measuring = false;
    Float32 measureMm = 0.0f;

    // One line of diagnostics from the ACTIVE mode, written by draw() and drawn
    // by the caller's HUD. Empty when the mode has nothing extra to say.
    //
    // This is where a mode reports the number it exists to produce - the widest
    // gap, the tightest sector, how much of the revolution came back unusable.
    // Without it a mode is a picture; with it the picture has a reading.
    Char diag[128] = {};

    // ---- data -----------------------------------------------------------
    Void push(const LidarFrame& frame);
    Void clear();

    // Consumes `size` of layout space, handles input, and draws.
    Void draw(const ImVec2& size);

    // The newest revolution, and whether there is one. The 3D renderer needs
    // the points without needing the whole trail, and it has no business
    // reaching into the deque.
    [[nodiscard]] Bool trailEmpty() const { return trail.empty(); }

    // The last revolution rate the source reported. The 3D Full panel shows it,
    // and it has no business reading a private field to do so.
    [[nodiscard]] Float32 hz() const { return lastHz; }
    [[nodiscard]] const std::vector<LidarPoint>& lastRevolution() const
    {
        static const std::vector<LidarPoint> none;
        return trail.empty() ? none : trail.back();
    }

    // ---- view control ---------------------------------------------------
    // Auto-fit keeps the sensor centred and eases the scale to contain the data.
    // Any pan or zoom drops out of it; fit() returns to it.
    Void fit();
    [[nodiscard]] Bool isAutoFit() const noexcept { return autoFit; }

    // Pins the view to a fixed radius in mm (drops auto-fit, recentres).
    Void setRangeMm(Float32 mm);

    // Distance from the view centre to the nearer edge of the widget, in mm.
    [[nodiscard]] Float32 visibleRangeMm() const noexcept;

    // ---- geometry, for HUD overlays drawn by the caller ------------------
    [[nodiscard]] ImVec2 toScreen(const ImVec2& worldMm) const noexcept;
    [[nodiscard]] ImVec2 toWorld(const ImVec2& screenPx) const noexcept;

private:
    std::deque<std::vector<LidarPoint>> trail;
    Bool hasData = false;

    // View: world point shown at the widget centre, and the zoom.
    ImVec2  viewCenterMm = ImVec2(0.0f, 0.0f);
    Float32 pxPerMm      = 0.0f;      // 0 until the first draw sizes it
    Bool    autoFit      = true;
    Float32 autoRangeMm  = 4000.0f;   // eased target radius while auto-fitting

    // Recent per-revolution fit distances. The auto-fit target is the largest
    // of these rather than the newest, so the view only shrinks once the scene
    // has genuinely stayed small - a single sparse revolution cannot pull it in
    // and then let it spring back.
    static constexpr Int32 FIT_HISTORY = 24;   // ~2.4 s at 10 Hz
    Float32 fitHist[FIT_HISTORY] = {};
    Int32   fitN = 0;

    // The auto-fit radius the view is committed to, snapped to a 1/2/5 x 10^n
    // ladder. Easing toward a continuously-moving target never settles - it
    // just chases - so the target itself is discrete and changes only when the
    // scene crosses a threshold. 0 means "not chosen yet".
    Float32 fitStepMm = 0.0f;

    // Rotation rate of the most recent revolution, so a mode can report the
    // duration of a sweep without the caller having to pass it in.
    Float32 lastHz = 0.0f;

    Bool    measureActive = false;
    ImVec2  measureFromMm = ImVec2(0.0f, 0.0f);
    ImVec2  measureToMm   = ImVec2(0.0f, 0.0f);
};
