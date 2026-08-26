// Toolbar icons.
//
// Fugue Icons 3.5.6 by Yusuke Kamiyamane, CC BY 3.0. See assets/icons/LICENSE.txt
// and the attribution in the repo README - the licence requires it.
//
// They are 16x16 pixel art, hinted at that size and available at no other, so
// they are drawn at INTEGER multiples of 16 physical pixels and never at a
// fractional DPI scale. A 16px source stretched to 24px is mush; 16px next to
// larger text is merely small, which is what toolbars of that era looked like
// anyway.
#pragma once

#include "imgui.h"

// For ui::Tint, which iconButton forwards to ui::button.
#include "theme.hpp"

#include "shared.hpp"

struct ID3D11Device;

namespace ui {

// Every icon the app uses. Adding one means adding the enum entry and the file
// name beside it in icons.cpp - there is no lookup by string at a call site,
// so a typo is a compile error rather than a missing picture.
enum class Icon
{
    // ---- central views ----
    ICON_RADAR = 0,      // the map view
    ICON_PROCESSOR,      // the Pico board view

    // ---- sidebar sections ----
    ICON_SYSTEM,
    ICON_SENSORS,
    ICON_VEHICLE,
    ICON_FIRMWARE,
    ICON_CONSOLE,

    // ---- actions ----
    ICON_PLUG_CONNECT,
    ICON_PLUG_DISCONNECT,
    ICON_REBOOT,
    ICON_BACKUP,
    ICON_RESET_VIEW,
    ICON_BUILD,          // compile
    ICON_FLASH,          // write to the board
    ICON_CLEAR,          // clear a log
    ICON_REFRESH,
    ICON_MOTOR_STOP,
    ICON_MOTOR_RUN,
    ICON_RECORD,
    ICON_PLAY,
    ICON_PAUSE,
    ICON_SAVE,
    ICON_OPEN,
    ICON_LAMP,
    ICON_LAMP_DIM,
    ICON_HAZARD,
    ICON_SEND,
    ICON_HELP,
    ICON_CODE,

    // ---- status LEDs ----
    ICON_STATUS_OK,
    ICON_STATUS_WARN,
    ICON_STATUS_BAD,
    ICON_STATUS_IDLE,

    // ---- telemetry tabs ----
    ICON_LIVE,
    ICON_SIGNAL,
    ICON_SCAN,
    ICON_DEVICE,

    // ---- subsystem / sensor rows ----
    ICON_LINK,           // a serial link
    ICON_SERVO,
    ICON_TOF,            // a ranging sensor
    ICON_ENCODER,
    ICON_IMU,
    ICON_STORAGE,
    ICON_NETWORK,
    ICON_MEASURE,

    // ---- map overlay modes ----
    // One per MapMode, in the same order. Several of these are genuinely apt
    // rather than decorative - a polyline for Contour, a pie for Sectors, an
    // open door for Gaps - which is why they are here now and were left out
    // when the only candidates would have been arbitrary.
    ICON_MODE_POINTS,
    ICON_MODE_DENSITY,
    ICON_MODE_MOTION,
    ICON_MODE_CLEARANCE,
    ICON_MODE_GAPS,
    ICON_MODE_WALLS,
    ICON_MODE_CORNERS,
    ICON_MODE_FIT,
    ICON_MODE_FULL,
    ICON_MODE_MINIMAL,

    // The 3D scene's own modes. Walls and Fit reuse the flat map's icons on
    // purpose: they are the SAME analysis seen from a different place, and
    // giving them separate marks would imply they were separate answers.
    ICON_SCENE_CLOUD,
    ICON_SCENE_BLOCKS,
    ICON_SCENE_WALLS,
    ICON_SCENE_FIT,
    ICON_SCENE_FULL,

    // The dimension switch itself.
    ICON_DIM_2D,
    ICON_DIM_3D,

    ICON_COUNT
};

// The overlay block must stay contiguous and in MapMode order: drawModeToggle()
// indexes it arithmetically rather than through a second table that could drift.
static_assert(static_cast<int>(Icon::ICON_MODE_MINIMAL)
              - static_cast<int>(Icon::ICON_MODE_POINTS) == 9,
              "icons.hpp: the ICON_MODE_* block must stay contiguous");

// ---------------------------------------------------------------------------
// Resolves `relative` (e.g. "icons" or "models/car.obj") under assets/.
//
// TWO LAYOUTS are tried, in order:
//
//   <exe dir>/assets/...       the exe shipped with its assets beside it
//   <exe dir>/../assets/...    build.bat's layout, with the exe in hub/build/
//
// Both exist in practice and the second used to be the ONLY one accepted, which
// meant an exe copied anywhere sensible - or built by CMake into its own
// directory - silently lost every icon and the car model, with no error and no
// missing-file message. A resolver that only understands the developer's own
// tree is a resolver that breaks the moment anybody ships anything.
//
// Returns false when neither exists, so callers can degrade rather than guess.
[[nodiscard]] Bool assetPath(const Char* relative, Char* out, Size cap);

// Decodes the PNGs into one atlas texture. Safe to call with a null device (the
// icons simply never become available and every draw below is a no-op), so a
// machine without the asset folder still runs.
Void loadIcons(ID3D11Device* device);

// ---------------------------------------------------------------------------
// A standalone texture from a PNG on disk, for anything that is not an icon.
//
// Exposed rather than duplicated: this file already owns the only WIC decode and
// the only CreateTexture2D in the app, and a second copy in the 3D renderer
// would be a second place for the pixel format and the premultiply rule to
// drift apart.
//
// Returns 0 on any failure - a missing texture must cost fidelity, not a crash.
// The caller does not own the result; releaseIcons() frees these too.
// ---------------------------------------------------------------------------
[[nodiscard]] ImTextureID loadTexture(ID3D11Device* device, const Char* path);

// The device loadIcons() was given, so anything else that needs to make a
// texture does not have to be handed one through five layers of UI code that
// have no business knowing about D3D. Null before loadIcons().
[[nodiscard]] ID3D11Device* device() noexcept;
Void releaseIcons();

// True once the atlas exists. Call sites do not need to check - the draw helpers
// degrade to drawing nothing - but layout code that reserves width does.
[[nodiscard]] Bool iconsReady() noexcept;

// Side length the icons are drawn at, in physical pixels: 16 times an integer
// scale chosen from the DPI. Never a fractional multiple.
[[nodiscard]] Float32 iconSize() noexcept;

// Draws `ic` as an ImGui item, advancing the cursor like any other widget.
Void icon(Icon ic);

// Draws `ic` into a draw list at `pos` (top-left), tinted. For custom-drawn
// chrome - the status strip, the map HUD - where there is no ImGui item.
Void iconAt(ImDrawList* dl, Icon ic, const ImVec2& pos, ImU32 tint = IM_COL32_WHITE);

// A normal ImGui button with an icon set into its left margin.
//
// The label stays centred, which is what leaves the margin free - so this needs
// no padded label string and no hand-rolled widget. Returns what Button returns.
[[nodiscard]] Bool iconButton(Icon ic, const Char* label,
                              const ImVec2& size = ImVec2(0, 0),
                              Tint tint = Tint::TINT_NONE);

// One cell of a mutually-exclusive row, with an icon in its left margin. The
// segmented-button equivalent of iconButton().
[[nodiscard]] Bool segmentedIconButton(Icon ic, const Char* label, Bool selected,
                                       const ImVec2& size = ImVec2(0, 0));

// icon() followed by SameLine(), which is nearly every call site.
Void iconLabel(Icon ic);

} // namespace ui
