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
#include "lidar_source.h"

class RadarView
{
public:
    // ---- overlays -------------------------------------------------------
    bool show_grid    = true;
    bool show_trail   = true;
    bool show_labels  = true;   // range-ring and compass labels
    bool show_nearest = true;   // ring around the closest return

    // ---- data -----------------------------------------------------------
    void push(const LidarFrame& frame);
    void clear();

    // Consumes `size` of layout space, handles input, and draws.
    void draw(const ImVec2& size);

    // ---- view control ---------------------------------------------------
    // Auto-fit keeps the sensor centred and eases the scale to contain the data.
    // Any pan or zoom drops out of it; fit() returns to it.
    void fit();
    bool is_auto_fit() const { return auto_fit_; }

    // Pins the view to a fixed radius in mm (drops auto-fit, recentres).
    void set_range_mm(float mm);

    // Distance from the view centre to the nearer edge of the widget, in mm.
    float visible_range_mm() const;

    // ---- geometry, for HUD overlays drawn by the caller ------------------
    ImVec2 to_screen(const ImVec2& world_mm) const;
    ImVec2 to_world(const ImVec2& screen_px) const;
    ImVec2 center_px() const { return center_px_; }
    float  radius_px() const { return radius_px_; }

    // ---- readouts, valid after draw() -----------------------------------
    bool  cursor_valid() const { return cursor_valid_; }
    float cursor_range_mm() const { return cursor_range_mm_; }     // from sensor
    float cursor_bearing_deg() const { return cursor_bearing_deg_; }

    bool  has_nearest() const { return has_nearest_; }
    float nearest_mm() const { return nearest_mm_; }
    float nearest_bearing_deg() const { return nearest_bearing_deg_; }

    bool  measuring() const { return measure_active_; }
    float measure_mm() const { return measure_mm_; }

private:
    std::deque<std::vector<LidarPoint>> trail_;
    bool  has_data_ = false;

    // View: world point shown at the widget centre, and the zoom.
    ImVec2 view_center_mm_ = ImVec2(0.0f, 0.0f);
    float  px_per_mm_      = 0.0f;    // 0 until the first draw sizes it
    bool   auto_fit_       = true;
    float  auto_range_mm_  = 4000.0f; // eased target radius while auto-fitting

    // Recent per-revolution fit distances. The auto-fit target is the largest
    // of these rather than the newest, so the view only shrinks once the scene
    // has genuinely stayed small - a single sparse revolution cannot pull it in
    // and then let it spring back.
    static constexpr int kFitHistory = 24;   // ~2.4 s at 10 Hz
    float fit_hist_[kFitHistory] = {};
    int   fit_n_ = 0;

    // Cached widget geometry from the last draw().
    ImVec2 center_px_ = ImVec2(0.0f, 0.0f);
    float  radius_px_ = 0.0f;

    // Readouts.
    bool   cursor_valid_       = false;
    float  cursor_range_mm_    = 0.0f;
    float  cursor_bearing_deg_ = 0.0f;

    bool   has_nearest_         = false;
    float  nearest_mm_          = 0.0f;
    float  nearest_bearing_deg_ = 0.0f;

    bool   measure_active_ = false;
    float  measure_mm_     = 0.0f;
    ImVec2 measure_from_mm_ = ImVec2(0.0f, 0.0f);
    ImVec2 measure_to_mm_   = ImVec2(0.0f, 0.0f);
};
