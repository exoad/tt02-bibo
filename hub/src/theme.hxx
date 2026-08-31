// Styling for the viewer: stock Dear ImGui dark theme, a readable type scale,
// and the few data colours the map needs.
//
// This is deliberately NOT a design system. Widgets are plain ImGui widgets;
// the only thing customised is font size and a small amount of spacing.
#pragma once

#include "shared.hxx"

#include "imgui.h"

namespace ui
{

  // Type scale in LOGICAL px. loadFonts multiplies these by the DPI scale, so a
  // PushFont base size is always the font's own LegacySize - never re-multiplied.
  namespace size
  {
    // Pulled down a step for density. Still comfortably readable - at 1.5x DPI the
    // body face lands at ~22 physical px - but a workstation UI carries far more on
    // screen at once than the previous scale allowed, and the brief is dense.
    inline constexpr Float32 SMALL = 13.0f;   // captions, axis labels, HUD
    inline constexpr Float32 BODY  = 15.0f;   // default UI text
    inline constexpr Float32 TITLE = 17.0f;   // section headings
    inline constexpr Float32 STAT  = 21.0f;   // metric values
    inline constexpr Float32 BIG   = 26.0f;   // hero numerals
    inline constexpr Float32 CODE  = 15.0f;   // source text in the editor
  }

  struct Fonts
  {
      ImFont* small = nullptr;
      ImFont* body  = nullptr;
      ImFont* title = nullptr;
      ImFont* stat  = nullptr;
      ImFont* big   = nullptr;

      // Monospaced, for the code editor and anywhere else a column has to line up
      // with the column above it. Falls back to `body` if no mono face is
      // installed, which costs alignment but never crashes.
      ImFont* mono  = nullptr;
  };

  extern Fonts fonts;

  // ---------------------------------------------------------------------------
  // The palette: Tango, on cream. Frutiger Aero read as a LIGHT industrial tool.
  //
  // The era this app is dressed as was mostly a light era - NetBeans, Media
  // Center, the GNOME/Tango desktops - and the Tango palette was designed exactly
  // for that: saturated enough to carry meaning, dark enough to sit on an off-white
  // ground without glowing. Every hue below is a Tango stop or one step off it.
  //
  // Ground is CREAM, not white and not neutral grey. A warm off-white is what makes
  // the chrome read as of that period rather than as a modern flat-white app.
  //
  // Three anchors:
  //
  //   BLUE  #3465A4 - Tango Sky Blue. Selection, checks, the active tab, the
  //                   heading arrow. Never a status.
  //   GREEN #4E9A06 - Tango Chameleon, dark stop. Connected, scanning, in spec.
  //   ORANGE #CE5C00 - Tango Orange, dark stop. Waiting, degraded, out of spec.
  //
  // The dark stops matter: the bright Tango stops (#73D216, #F57900) are display
  // colours for filled shapes and are illegible as TEXT on cream. Anything that
  // prints as words uses the dark stop.
  // ---------------------------------------------------------------------------

  // ---------------------------------------------------------------------------
  // Dark Aero, on an industrial slate console.
  //
  // The reference points are the dark-graphite era of professional tools - UDK,
  // Maya, 3ds Max, Blender 2.7x, Photoshop CS5/6, FL Studio - crossed with what
  // gets called Dark Aero or Frutiger Ego: charcoal panels rather than nature and
  // sky, with the accents reading as glowing LEDs set into the casing.
  //
  // Three properties do the work, and none of them is a colour:
  //
  //   LOW CONTRAST GROUND. Graphite, not black. These tools sat in front of people
  //   for ten-hour sessions and a pure black ground with white text is the harshest
  //   pairing there is. Panels live in a narrow band around 15-22% and the type
  //   floats a little way above it.
  //
  //   TACTILE CONTROLS. A button is a physical key pushed out of the casing: one
  //   pixel of light along its top edge, one of shadow along its bottom, inverted
  //   when pressed so it visibly sinks. See ui::bevel.
  //
  //   LED ACCENTS. Cyan for selection, green/amber/red for state - saturated, and
  //   read as light emitted rather than paint applied, because on graphite that is
  //   what a bright small area looks like.
  //
  // The lidar VIEWPORT stays darker than the chrome around it: it is a display set
  // into the console, not part of the casing.
  // ---------------------------------------------------------------------------

  // Data colours for the viewport - the map, the board view, the HUD drawn on them.
  // ---------------------------------------------------------------------------
  // THE TERMINAL PALETTE. Pure xterm ANSI on pure black.
  //
  // This is the LIDAR MAP's palette and nothing else's. The chrome around it stays
  // industrial slate and the board view keeps `plot::`, because that one is a
  // picture of a physical object and a photograph of a PCB has no business being
  // drawn in sixteen colours.
  //
  // The map is different: it is a readout, and a readout wants maximum separation
  // between its few meanings with no ambiguity about which one you are looking at.
  // That is precisely the problem sixteen fully saturated colours on black were
  // designed for, and it is what a graphite substrate with gradients was working
  // against - every hue had to survive being laid over a tinted, unevenly lit
  // ground, so none of them could be itself.
  //
  // The rule for anything added here: it is one of the sixteen, unmodified. No
  // blends, no alpha ramps standing in for a colour, no tints of the background.
  // ---------------------------------------------------------------------------
  namespace ansi
  {

    // ---- the sixteen -------------------------------------------------------
    inline constexpr ImU32 BLACK     = IM_COL32(0x00, 0x00, 0x00, 0xFF);
    inline constexpr ImU32 RED       = IM_COL32(0xCD, 0x00, 0x00, 0xFF);
    inline constexpr ImU32 GREEN     = IM_COL32(0x00, 0xCD, 0x00, 0xFF);
    inline constexpr ImU32 YELLOW    = IM_COL32(0xCD, 0xCD, 0x00, 0xFF);
    inline constexpr ImU32 BLUE      = IM_COL32(0x00, 0x00, 0xEE, 0xFF);
    inline constexpr ImU32 MAGENTA   = IM_COL32(0xCD, 0x00, 0xCD, 0xFF);
    inline constexpr ImU32 CYAN      = IM_COL32(0x00, 0xCD, 0xCD, 0xFF);
    inline constexpr ImU32 WHITE     = IM_COL32(0xE5, 0xE5, 0xE5, 0xFF);

    inline constexpr ImU32 GREY      = IM_COL32(0x7F, 0x7F, 0x7F, 0xFF);
    inline constexpr ImU32 BRRED     = IM_COL32(0xFF, 0x00, 0x00, 0xFF);
    inline constexpr ImU32 BRGREEN   = IM_COL32(0x00, 0xFF, 0x00, 0xFF);
    inline constexpr ImU32 BRYELLOW  = IM_COL32(0xFF, 0xFF, 0x00, 0xFF);
    inline constexpr ImU32 BRBLUE    = IM_COL32(0x5C, 0x5C, 0xFF, 0xFF);
    inline constexpr ImU32 BRMAGENTA = IM_COL32(0xFF, 0x00, 0xFF, 0xFF);
    inline constexpr ImU32 BRCYAN    = IM_COL32(0x00, 0xFF, 0xFF, 0xFF);
    inline constexpr ImU32 BRWHITE   = IM_COL32(0xFF, 0xFF, 0xFF, 0xFF);

    // ---- what each one means on the map ------------------------------------

    // Distance: red near, yellow mid, green far. A traffic light, read the way
    // everybody already reads one, and the reason the ramp no longer ends in cyan -
    // cyan is the heading marker, and a far wall should not be the same colour as
    // the direction the car is pointing.
    inline constexpr ImU32 RAMP_NEAR = BRRED;
    inline constexpr ImU32 RAMP_MID  = BRYELLOW;
    inline constexpr ImU32 RAMP_FAR  = BRGREEN;

    // Furniture. The two greys below are the ONE allowed departure from the sixteen,
    // because a range grid drawn at full brightness competes with the returns it
    // exists to measure - and they are still neutral, not tinted.
    inline constexpr ImU32 GRID       = IM_COL32(0x3A, 0x3A, 0x3A, 0xFF);
    inline constexpr ImU32 GRID_MAJOR = GREY;
    inline constexpr ImU32 AXIS       = IM_COL32(0x26, 0x26, 0x26, 0xFF);

    inline constexpr ImU32 LABEL      = WHITE;
    inline constexpr ImU32 HEADING    = BRCYAN;
    inline constexpr ImU32 NEAREST    = BRRED;
    inline constexpr ImU32 MEASURE    = BRYELLOW;

    // Status, on the map.
    inline constexpr ImU32 OK    = BRGREEN;
    inline constexpr ImU32 WARN  = BRYELLOW;
    inline constexpr ImU32 BAD   = BRRED;
    inline constexpr ImU32 IDLE  = GREY;

  }

  namespace plot
  {

    // ---------------------------------------------------------------------------
    // FORWARDS. Every one of these is now a name for an ansi:: colour.
    //
    // The names stay because call sites read better for them - plot::OK says what
    // it means where ansi::BRGREEN says only what it looks like - but there is one
    // palette in this program and it is the sixteen above. This namespace used to
    // hold a second, softer set: amber-to-cyan ramps and steel-blue furniture, from
    // when the viewport was trying to look like an instrument panel. Two palettes
    // meant a green LED in the sidebar and a green return on the map were different
    // greens, which is exactly the sort of thing nobody notices and everybody feels.
    // ---------------------------------------------------------------------------

    inline constexpr ImU32 RAMP_NEAR = ansi::RAMP_NEAR;
    inline constexpr ImU32 RAMP_MID  = ansi::RAMP_MID;
    inline constexpr ImU32 RAMP_FAR  = ansi::RAMP_FAR;

    inline constexpr ImU32 GRID       = ansi::GRID;
    inline constexpr ImU32 GRID_MAJOR = ansi::GRID_MAJOR;
    inline constexpr ImU32 AXIS       = ansi::AXIS;
    inline constexpr ImU32 LABEL      = ansi::LABEL;
    inline constexpr ImU32 HEADING    = ansi::HEADING;
    inline constexpr ImU32 HUB        = ansi::BRCYAN;
    inline constexpr ImU32 HUB_CORE   = ansi::BRWHITE;
    inline constexpr ImU32 NEAREST    = ansi::NEAREST;
    inline constexpr ImU32 MEASURE    = ansi::MEASURE;

    inline constexpr ImU32 OK      = ansi::OK;
    inline constexpr ImU32 WARN    = ansi::WARN;
    inline constexpr ImU32 BAD     = ansi::BAD;
    inline constexpr ImU32 IDLE    = ansi::IDLE;

    inline constexpr ImU32 ACCENT  = ansi::BRCYAN;

  }

  // The interface accent. Cyan is reserved for "the UI is pointing at this" -
  // selection, check marks, the active tab, a slider grab. Deliberately NOT a
  // status: a thing being selected and a thing being healthy are different claims.
  namespace accent
  {

    inline constexpr ImU32 CYAN      = ansi::CYAN;
    inline constexpr ImU32 CYAN_HI   = ansi::BRCYAN;

  }

  // ---------------------------------------------------------------------------
  // Pin CATEGORY colours, for the board view.
  //
  // Not statuses, and they must not borrow the status palette - which is what they
  // used to do: an assigned pin took sem::GOOD and a power pin took sem::WARN, so a
  // column of GND / VBUS / VSYS labels sat in warning orange announcing a problem
  // that did not exist. Power is not a warning; an assigned pin is not "healthy".
  //
  // The three categories encode ONE axis - how much this pin has to do with the
  // project - so this is one hue at three weights, not three unrelated hues.
  // ---------------------------------------------------------------------------
  namespace pin
  {

    inline constexpr ImU32 ASSIGNED = ansi::BRCYAN;                       // ours: strongest
    inline constexpr ImU32 POWER    = ansi::GREY;                        // structural: quiet
    inline constexpr ImU32 FREE     = IM_COL32(0x4A, 0x4A, 0x4A, 0xFF);  // unused: quietest

  }

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
  namespace sem
  {

    // Chrome and viewport are both dark now, so these can go back to forwarding to
    // the viewport set - one green means one thing everywhere. The split that
    // existed while the chrome was light is gone with it.
    inline constexpr ImU32 GOOD  = plot::OK;     // healthy, connected, in spec
    inline constexpr ImU32 WARN  = plot::WARN;   // degraded, waiting, out of spec
    inline constexpr ImU32 BAD   = plot::BAD;    // failed, disconnected, error
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
  // It exists for the floating workspace's optical zoom. A zoomed panel raises the
  // geometry scale so its padding, radii and gaps grow - but a font's size is
  // already baked into its LegacySize and does not follow. Anything that pushes an
  // explicit font size (the code editor does, because it needs the mono face)
  // multiplies by this, and then the text grows WITH the panel instead of staying
  // put while everything around it moves.
  [[nodiscard]] Float32 fontScale();

  // ---------------------------------------------------------------------------
  // The USER's zoom, on top of whatever the monitor's DPI already is.
  //
  // Deliberately a separate number from dpiScale(). That one is what Windows says
  // the display is; this one is what the person sitting in front of it wants, and
  // the two are independent facts - a correct DPI scale can still be too small for
  // someone's eyes, their desk, or the distance they are sitting at. They
  // multiply, and the product is what every size in the app derives from.
  //
  // Applied to fonts AND geometry together, which is the whole point: scaling only
  // the type would blow the layout apart, and the density the UI was tuned for is
  // a RATIO, so it survives being multiplied.
  // ---------------------------------------------------------------------------
  inline constexpr Float32 USER_SCALE_MIN  = 0.75f;
  inline constexpr Float32 USER_SCALE_MAX  = 2.00f;
  inline constexpr Float32 USER_SCALE_STEP = 0.05f;

  [[nodiscard]] Float32 userScale();

  // Clamped and snapped to USER_SCALE_STEP. Raises the changed flag only on a real
  // change, so calling this every frame with the same value costs nothing.
  Void setUserScale(Float32 s);

  // True once per change. The host reads this between frames - restyling from
  // inside a frame would leave half the widgets laid out at the old size.
  [[nodiscard]] Bool consumeUserScaleChanged();


  // ---------------------------------------------------------------------------
  // Elevation shading
  //
  // A very soft vertical fade drawn OVER a widget that has already been
  // submitted, so ImGui::Button stays a real ImGui::Button and keeps its sizing,
  // hover, activation and keyboard nav. The effect is deliberately near the
  // threshold of visibility - it should read as "this surface is lit from above"
  // and never as a gradient or a gloss.
  //
  // Usage:
  //     if(ImGui::Button("connect")) { ... }
  //     ui::shadeLastItem();
  // ---------------------------------------------------------------------------

  // Shades the item just submitted. No-op when the item is clipped or degenerate.
  // `strength` scales it; 0 disables.
  Void shadeLastItem(Float32 strength = 1.0f);

  // ---------------------------------------------------------------------------
  // A button's semantic tint.
  //
  // The rule the rest of this palette follows applies here too: COLOUR MEANS
  // SOMETHING OR IT IS NOT USED. A tint is not decoration and not a way to make a
  // control stand out - it is a claim about what pressing the thing does.
  //
  //   GOOD    it starts something, and starting it is the normal path
  //   WARN    it interrupts something, or the board will notice
  //   BAD     it destroys, overwrites or aborts
  //   ACCENT  the primary action of a group, where there is one
  //
  // Deliberately weak. The base slate is mixed only about a third of the way
  // toward the hue, because a saturated button in an industrial-slate panel reads
  // as a different application, and because the bevel and gloss are drawn OVER
  // this - the tint has to leave room for them or the control stops looking
  // moulded and starts looking painted.
  // ---------------------------------------------------------------------------
  enum class Tint
  {
      TINT_NONE = 0,
      TINT_GOOD,
      TINT_WARN,
      TINT_BAD,
      TINT_ACCENT,
  };

  // Pushes the three button colours for `t`. Balanced by popTint(); a no-op pair
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

  // An indicator lamp. A lit one throws a halo onto the panel around it, which is
  // what makes it read as emitting rather than as a painted dot; an unlit one is a
  // dark lamp in a recessed socket. Use for every state dot in the chrome.
  Void led(ImDrawList* dl, const ImVec2& centre, Float32 radius, ImU32 colour, Bool lit);

  // Shades an arbitrary rect, for custom-drawn surfaces.
  Void shadeRect(const ImVec2& pMin, const ImVec2& pMax, Float32 strength = 1.0f);

  // ImGui::Button + shadeLastItem, since that pairing is nearly every call site.
  Bool button(const Char* label, const ImVec2& size = ImVec2(0, 0),
              Tint tint = Tint::TINT_NONE);

  // A compact outlined checkbox, sized to the type rather than to the row.
  // Replaces ImGui::Checkbox everywhere - see the note on the implementation for
  // why the stock one is wrong at this type scale. Same signature and same
  // return contract: true on the frame it was clicked.
  Bool checkbox(const Char* label, Bool* v);

  // ImGui::Combo with the drop-down arrow drawn inside the field rather than as a
  // button welded to its right. Same signature and return contract.
  Bool combo(const Char* label, Int32* current, const Char* const items[], Int32 count);

  // Where the accent edge goes on a selected cell.
  enum class Mark { MARK_UNDERLINE, MARK_LEFT_BAR, MARK_NONE };

  // One cell of a mutually-exclusive set: transparent and unoutlined when not
  // selected, plate fill plus an accent edge when it is. `selected` is the
  // caller's state; this does not own it.
  //
  // Underline suits a horizontal row of segments, LeftBar a vertical list of
  // rows. Use it anywhere a set of buttons is really one choice - an unselected
  // cell must not be outlined, or the set reads as N controls instead of one.
  Bool segmentedButton(const Char* label, Bool selected, const ImVec2& size = ImVec2(0, 0),
                       Mark mark = Mark::MARK_UNDERLINE);

}
