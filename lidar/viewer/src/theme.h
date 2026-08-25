// Styling for the viewer: stock Dear ImGui dark theme, a readable type scale,
// and the few data colours the map needs.
//
// This is deliberately NOT a design system. Widgets are plain ImGui widgets;
// the only thing customised is font size and a small amount of spacing.
#pragma once

#include "imgui.h"

namespace ui {

// Type scale in LOGICAL px. LoadFonts multiplies these by the DPI scale, so a
// PushFont base size is always the font's own LegacySize - never re-multiplied.
namespace size {
inline constexpr float small = 15.0f;   // captions, axis labels, HUD
inline constexpr float body  = 17.0f;   // default UI text
inline constexpr float title = 20.0f;   // section headings
inline constexpr float stat  = 24.0f;   // metric values
inline constexpr float big   = 30.0f;   // hero numerals
}

struct Fonts
{
    ImFont* small = nullptr;
    ImFont* body  = nullptr;
    ImFont* title = nullptr;
    ImFont* stat  = nullptr;
    ImFont* big   = nullptr;
};

extern Fonts fonts;

// Data colours for the map. These are a colormap, not UI chrome: the point ramp
// encodes distance, so it stays explicit rather than following the ImGui theme.
namespace plot {

inline constexpr ImU32 ramp_near = IM_COL32(0xFF, 0xC1, 0x6E, 0xFF);  // warm  = close
inline constexpr ImU32 ramp_mid  = IM_COL32(0x7A, 0xD9, 0xA5, 0xFF);
inline constexpr ImU32 ramp_far  = IM_COL32(0x62, 0xB0, 0xFF, 0xFF);  // cool  = distant

inline constexpr ImU32 grid       = IM_COL32(0x4A, 0x4A, 0x52, 0xFF);
inline constexpr ImU32 grid_major = IM_COL32(0x6A, 0x6A, 0x74, 0xFF);
inline constexpr ImU32 axis       = IM_COL32(0x3A, 0x3A, 0x42, 0xFF);
inline constexpr ImU32 label      = IM_COL32(0xA8, 0xA8, 0xB4, 0xFF);
inline constexpr ImU32 heading    = IM_COL32(0x62, 0xB0, 0xFF, 0xFF);
inline constexpr ImU32 hub        = IM_COL32(0x3D, 0x6E, 0xA8, 0xFF);
inline constexpr ImU32 hub_core   = IM_COL32(0xE8, 0xF2, 0xFF, 0xFF);
inline constexpr ImU32 nearest    = IM_COL32(0xFF, 0x8A, 0x5C, 0xFF);
inline constexpr ImU32 measure    = IM_COL32(0xFF, 0xD7, 0x6E, 0xFF);

// Status accents, used by the HUD and the clearance chart.
inline constexpr ImU32 ok      = IM_COL32(0x62, 0xC4, 0x82, 0xFF);
inline constexpr ImU32 warn    = IM_COL32(0xE8, 0xB3, 0x39, 0xFF);
inline constexpr ImU32 bad     = IM_COL32(0xE8, 0x6B, 0x5C, 0xFF);
inline constexpr ImU32 idle    = IM_COL32(0x88, 0x88, 0x92, 0xFF);

} // namespace plot

// ---------------------------------------------------------------------------
// Semantic colours for UI TEXT. This is the whole palette; there is no other.
//
// The rule: colour means something or it is not used. A readout is drawn in the
// default text colour unless its VALUE carries a state — connected, degraded,
// failed. Numbers are not tinted to tell them apart; their labels do that.
//
// Everything else (buttons, frames, headers, tabs, scrollbars) keeps stock
// ImGui dark. Do not tint a widget to make it stand out; place it better.
//
// `plot::` above is a different thing: those are DATA colours for the map, where
// hue encodes a measured quantity rather than a UI state.
// ---------------------------------------------------------------------------
namespace sem {

inline constexpr ImU32 good  = plot::ok;     // healthy, connected, in spec
inline constexpr ImU32 warn  = plot::warn;   // degraded, waiting, out of spec
inline constexpr ImU32 bad   = plot::bad;    // failed, disconnected, error
inline constexpr ImU32 muted = plot::idle;   // absent, idle, not applicable

} // namespace sem

// Loads Segoe UI at the sizes above, falling back to ImGui's built-in font if
// it is unavailable. Never leaves a null ImFont*. Call once after
// ImGui::CreateContext() and before the first frame.
void LoadFonts(float dpi_scale);

// Applies ImGui's dark theme plus modest spacing/rounding tweaks.
void ApplyStyle(float dpi_scale);

// DPI scale used for geometry (padding, radii, gaps). Never applied to a font
// base size - LoadFonts has already baked DPI into each font's LegacySize.
void  SetDpiScale(float scale);
float DpiScale();

// ---------------------------------------------------------------------------
// Aero gloss
//
// The signature mid-2000s look is a lit top half and a specular line near the
// top edge. Rather than reimplement widgets, these draw OVER a stock widget
// that has already been submitted, so `ImGui::Button` stays `ImGui::Button` and
// keeps all its normal behaviour, sizing and keyboard nav.
//
// Usage:
//     if (ImGui::Button("Connect")) { ... }
//     ui::GlossLastItem();
// ---------------------------------------------------------------------------

// Glosses the item just submitted. Skips itself when the item is clipped or
// degenerate. `strength` scales the effect; 0 disables it.
void GlossLastItem(float strength = 1.0f);

// Glosses an arbitrary rect - for panels and custom-drawn surfaces.
void GlossRect(const ImVec2& p_min, const ImVec2& p_max, float rounding,
               float strength = 1.0f);

// A vertical sky gradient for a window or panel background. Aero's other half:
// surfaces are lit from above rather than being one flat colour.
void SkyBackdrop(const ImVec2& p_min, const ImVec2& p_max, float rounding = 0.0f);

} // namespace ui
