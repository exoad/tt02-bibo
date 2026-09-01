// Styling for the viewer: stock Dear ImGui dark theme, a readable type scale, and
// the few data colors the map needs. Deliberately NOT a design system - widgets
// are plain ImGui widgets; only font size and a little spacing are customised.
#pragma once

#include "shared.hxx"

#include "imgui.h"

namespace ui
{

  // Type scale in LOGICAL px. loadFonts multiplies these by the DPI scale, so a
  // PushFont base size is always the font's own LegacySize - never re-multiplied.
  namespace size
  {
    // Pulled down a step for density; at 1.5x DPI the body face lands at ~22
    // physical px.
    inline constexpr Float32 SMALL = 13.0f;   // captions, axis labels, HUD
    inline constexpr Float32 BODY = 15.0f;   // default UI text
    inline constexpr Float32 TITLE = 17.0f;   // section headings
    inline constexpr Float32 STAT = 21.0f;   // metric values
    inline constexpr Float32 BIG = 26.0f;   // hero numerals
    inline constexpr Float32 CODE = 15.0f;   // source text in the editor
  }

  struct Fonts
  {
      ImFont* small = nullptr;
      ImFont* body = nullptr;
      ImFont* title = nullptr;
      ImFont* stat = nullptr;
      ImFont* big = nullptr;

      // Monospaced, for the code editor and anywhere a column must line up. Falls
      // back to `body` if no mono face is installed.
      ImFont* mono = nullptr;
  };

  extern Fonts fonts;

  // Dark Aero, on an industrial slate console. Three properties do the work:
  //   LOW CONTRAST GROUND - graphite, not black; panels sit in a narrow band
  //     around 15-22% with the type floating a little above it.
  //   TACTILE CONTROLS - 1px of light on a button's top edge, 1px of shadow on
  //     its bottom, inverted when pressed. See ui::bevel.
  //   LED ACCENTS - cyan for selection, green/amber/red for state.
  // The lidar VIEWPORT stays darker than the chrome: a display set into the
  // console, not part of the casing.

  // THE TERMINAL PALETTE. Pure xterm ANSI on pure black, and the LIDAR MAP's
  // palette only - the chrome stays industrial slate and the board view keeps
  // `plot::`. The rule for anything added here: it is one of the sixteen,
  // unmodified. No blends, no alpha ramps standing in for a color, no tints.
  namespace ansi
  {

    // ---- the sixteen -------------------------------------------------------
    inline constexpr ImU32 BLACK = IM_COL32(0x00, 0x00, 0x00, 0xFF);
    inline constexpr ImU32 RED = IM_COL32(0xCD, 0x00, 0x00, 0xFF);
    inline constexpr ImU32 GREEN = IM_COL32(0x00, 0xCD, 0x00, 0xFF);
    inline constexpr ImU32 YELLOW = IM_COL32(0xCD, 0xCD, 0x00, 0xFF);
    inline constexpr ImU32 BLUE = IM_COL32(0x00, 0x00, 0xEE, 0xFF);
    inline constexpr ImU32 MAGENTA = IM_COL32(0xCD, 0x00, 0xCD, 0xFF);
    inline constexpr ImU32 CYAN = IM_COL32(0x00, 0xCD, 0xCD, 0xFF);
    inline constexpr ImU32 WHITE = IM_COL32(0xE5, 0xE5, 0xE5, 0xFF);

    inline constexpr ImU32 GRAY = IM_COL32(0x7F, 0x7F, 0x7F, 0xFF);
    inline constexpr ImU32 BRRED = IM_COL32(0xFF, 0x00, 0x00, 0xFF);
    inline constexpr ImU32 BRGREEN = IM_COL32(0x00, 0xFF, 0x00, 0xFF);
    inline constexpr ImU32 BRYELLOW = IM_COL32(0xFF, 0xFF, 0x00, 0xFF);
    inline constexpr ImU32 BRBLUE = IM_COL32(0x5C, 0x5C, 0xFF, 0xFF);
    inline constexpr ImU32 BRMAGENTA = IM_COL32(0xFF, 0x00, 0xFF, 0xFF);
    inline constexpr ImU32 BRCYAN = IM_COL32(0x00, 0xFF, 0xFF, 0xFF);
    inline constexpr ImU32 BRWHITE = IM_COL32(0xFF, 0xFF, 0xFF, 0xFF);

    // ---- what each one means on the map ------------------------------------

    // Distance: red near, yellow mid, green far - a traffic light. NOT cyan at the
    // far end: cyan is the heading marker.
    inline constexpr ImU32 RAMP_NEAR = BRRED;
    inline constexpr ImU32 RAMP_MID = BRYELLOW;
    inline constexpr ImU32 RAMP_FAR = BRGREEN;

    // Furniture. GRID and AXIS are the ONE allowed departure from the sixteen: a
    // range grid at full brightness competes with the returns it measures. Still
    // neutral, not tinted.
    inline constexpr ImU32 GRID = IM_COL32(0x3A, 0x3A, 0x3A, 0xFF);
    inline constexpr ImU32 GRID_MAJOR = GRAY;
    inline constexpr ImU32 AXIS = IM_COL32(0x26, 0x26, 0x26, 0xFF);

    inline constexpr ImU32 LABEL = WHITE;
    inline constexpr ImU32 HEADING = BRCYAN;
    inline constexpr ImU32 NEAREST = BRRED;
    inline constexpr ImU32 MEASURE = BRYELLOW;

    // Status, on the map.
    inline constexpr ImU32 OK = BRGREEN;
    inline constexpr ImU32 WARN = BRYELLOW;
    inline constexpr ImU32 BAD = BRRED;
    inline constexpr ImU32 IDLE = GRAY;

  }

  namespace plot
  {

    // FORWARDS to ansi::; the names stay because plot::OK says what it means. There
    // is ONE palette in this program - do not reintroduce a second, softer set.

    inline constexpr ImU32 RAMP_NEAR = ansi::RAMP_NEAR;
    inline constexpr ImU32 RAMP_MID = ansi::RAMP_MID;
    inline constexpr ImU32 RAMP_FAR = ansi::RAMP_FAR;

    inline constexpr ImU32 GRID = ansi::GRID;
    inline constexpr ImU32 GRID_MAJOR = ansi::GRID_MAJOR;
    inline constexpr ImU32 AXIS = ansi::AXIS;
    inline constexpr ImU32 LABEL = ansi::LABEL;
    inline constexpr ImU32 HEADING = ansi::HEADING;
    inline constexpr ImU32 HUB = ansi::BRCYAN;
    inline constexpr ImU32 HUB_CORE = ansi::BRWHITE;
    inline constexpr ImU32 NEAREST = ansi::NEAREST;
    inline constexpr ImU32 MEASURE = ansi::MEASURE;

    inline constexpr ImU32 OK = ansi::OK;
    inline constexpr ImU32 WARN = ansi::WARN;
    inline constexpr ImU32 BAD = ansi::BAD;
    inline constexpr ImU32 IDLE = ansi::IDLE;

    inline constexpr ImU32 ACCENT = ansi::BRCYAN;

  }

  // The interface accent. Cyan is reserved for "the UI is pointing at this" -
  // selection, check marks, the active tab, a slider grab. Never a status.
  namespace accent
  {

    inline constexpr ImU32 CYAN = ansi::CYAN;
    inline constexpr ImU32 CYAN_HI = ansi::BRCYAN;

  }

  // Pin CATEGORY colors, for the board view. NOT statuses, and they must not
  // borrow the status palette: sem::WARN for power put a column of GND / VBUS /
  // VSYS labels in warning orange. They encode ONE axis - how much this pin has to
  // do with the project - so it is one hue at three weights.
  namespace pin
  {

    inline constexpr ImU32 ASSIGNED = ansi::BRCYAN;                       // ours: strongest
    inline constexpr ImU32 POWER = ansi::GRAY;                        // structural: quiet
    inline constexpr ImU32 FREE = IM_COL32(0x4A, 0x4A, 0x4A, 0xFF);  // unused: quietest

  }

  // Semantic colors for UI TEXT, and the whole palette for it. Color means
  // something or it is not used: a readout is default-colored unless its VALUE
  // carries a state, and everything else keeps stock ImGui dark. `plot::` above is
  // different - DATA colors, where hue encodes a measured quantity.
  namespace sem
  {

    // Forwarded to the viewport set: one green means one thing everywhere.
    inline constexpr ImU32 GOOD = plot::OK;     // healthy, connected, in spec
    inline constexpr ImU32 WARN = plot::WARN;   // degraded, waiting, out of spec
    inline constexpr ImU32 BAD = plot::BAD;    // failed, disconnected, error
    inline constexpr ImU32 MUTED = plot::IDLE;   // absent, idle, not applicable

  }

  // Loads Segoe UI at the sizes above, falling back to ImGui's built-in font if
  // it is unavailable. Never leaves a null ImFont*. Call once after
  // ImGui::CreateContext() and before the first frame.
  Void loadFonts(Float32 dpiScale);

  // Applies ImGui's dark theme plus modest spacing/rounding tweaks.
  Void applyStyle(Float32 dpiScale);

  // DPI scale used for geometry (padding, radii, gaps). Never applied to a font
  // base size - LoadFonts has already baked DPI into each font's LegacySize.
  Void  setDpiScale(Float32 scale);
  Float32 dpiScale();

  // How far the CURRENT geometry scale has been pushed past the one the fonts were
  // baked at. 1.0 normally.
  //
  // For the floating workspace's optical zoom: a zoomed panel raises the geometry
  // scale, but a font's size is baked into its LegacySize and does not follow.
  // Anything pushing an explicit font size (the code editor, for the mono face)
  // must multiply by this or the text stays put while the panel moves.
  [[nodiscard]] Float32 fontScale();

  // The USER's zoom, deliberately separate from dpiScale() - that is what Windows
  // says the display is, this is what the person wants. They MULTIPLY, and the
  // product is what every size derives from. Applied to fonts AND geometry
  // together: scaling only the type blows the layout apart.
  inline constexpr Float32 USER_SCALE_MIN = 0.75f;
  inline constexpr Float32 USER_SCALE_MAX = 2.00f;
  inline constexpr Float32 USER_SCALE_STEP = 0.05f;

  [[nodiscard]] Float32 userScale();

  // Clamped and snapped to USER_SCALE_STEP. Raises the changed flag only on a real
  // change, so calling this every frame with the same value costs nothing.
  Void setUserScale(Float32 s);

  // True once per change. The host reads this between frames - restyling from
  // inside a frame would leave half the widgets laid out at the old size.
  [[nodiscard]] Bool consumeUserScaleChanged();


  // Elevation shading: a soft vertical fade drawn OVER an already-submitted
  // widget, so ImGui::Button stays a real ImGui::Button and keeps its sizing,
  // hover, activation and keyboard nav.
  //
  //     if(ImGui::Button("connect")) { ... }
  //     ui::shadeLastItem();
  //
  // No-op when the item is clipped or degenerate. `strength` scales it; 0 disables.
  Void shadeLastItem(Float32 strength = 1.0f);

  // A button's semantic tint. Not decoration - a claim about what pressing it does:
  //   GOOD    it starts something, and starting it is the normal path
  //   WARN    it interrupts something, or the board will notice
  //   BAD     it destroys, overwrites or aborts
  //   ACCENT  the primary action of a group, where there is one
  // Deliberately weak: the base slate is mixed only about a third of the way
  // toward the hue, so the bevel and gloss drawn OVER it still read.
  enum class Tint
  {
      TINT_NONE = 0,
      TINT_GOOD,
      TINT_WARN,
      TINT_BAD,
      TINT_ACCENT,
  };

  // Pushes the three button colors for `t`. Balanced by popTint(); a no-op pair
  // for TINT_NONE, so callers do not have to branch.
  Void pushTint(Tint t);
  Void popTint(Tint t);

  // Bevels an arbitrary rect: 1px of light on the top edge, 1px of shadow on the
  // bottom, inverted when `pressed` so the surface visibly sinks. This is the
  // tactile-control effect; shadeRect() forwards to it un-pressed.
  Void bevelRect(const ImVec2& pMin, const ImVec2& pMax, Bool pressed, Float32 strength = 1.0f);

  // Draws a recessed bezel just inside a rect, so the region reads as a display
  // milled into the casing rather than as an area of it. The exact inverse of the
  // raised-key bevel above.
  Void screenInset(const ImVec2& pMin, const ImVec2& pMax, Float32 strength = 1.0f);

  // A raised plate, for custom-drawn chrome that is not an ImGui item: the HUD
  // readouts over the map, the chips in the board view.
  Void plate(const ImVec2& pMin, const ImVec2& pMax, ImU32 fill, Float32 rounding = 0.0f);

  // An indicator lamp: lit throws a halo onto the panel, unlit is a dark lamp in
  // a recessed socket. Use for every state dot in the chrome.
  Void led(ImDrawList* dl, const ImVec2& center, Float32 radius, ImU32 color, Bool lit);

  // Shades an arbitrary rect, for custom-drawn surfaces.
  Void shadeRect(const ImVec2& pMin, const ImVec2& pMax, Float32 strength = 1.0f);

  // ImGui::Button + shadeLastItem, since that pairing is nearly every call site.
  Bool button(const Char* label, const ImVec2& size = ImVec2(0, 0),
              Tint tint = Tint::TINT_NONE);

  // A compact outlined checkbox, sized to the type rather than to the row.
  // Replaces ImGui::Checkbox everywhere - see theme.cxx for why the stock one is
  // wrong at this type scale. Same signature and return contract.
  Bool checkbox(const Char* label, Bool* v);

  // ImGui::Combo with the drop-down arrow drawn inside the field rather than as a
  // button welded to its right. Same signature and return contract.
  Bool combo(const Char* label, Int32* current, const Char* const items[], Int32 count);

  // Where the accent edge goes on a selected cell.
  enum class Mark { MARK_UNDERLINE, MARK_LEFT_BAR, MARK_NONE };

  // One cell of a mutually-exclusive set: transparent and unoutlined when not
  // selected, plate fill plus an accent edge when it is. `selected` is the
  // caller's state; this does not own it. Underline suits a horizontal row,
  // LeftBar a vertical list. An unselected cell must NOT be outlined, or the set
  // reads as N controls instead of one.
  Bool segmentedButton(const Char* label, Bool selected, const ImVec2& size = ImVec2(0, 0),
                       Mark mark = Mark::MARK_UNDERLINE);

}
