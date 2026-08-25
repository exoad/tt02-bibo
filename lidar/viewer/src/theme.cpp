// Stock Dear ImGui dark theme + a readable type scale.
//
// This file deliberately does NOT hand-author a palette. Colours come from
// ImGui::StyleColorsDark() and are left alone; the only edits are geometry
// (rounding / padding / spacing), tuned so the stock look still breathes at the
// larger font sizes in ui::size.
//
// Notes on Dear ImGui 1.92 (dynamic font atlas):
//  - PushFont() takes TWO arguments: PushFont(ImFont*, float base_size), where
//    base_size is a PRE-SCALE value that ImGui multiplies by
//    style.FontScaleMain * style.FontScaleDpi. Pass `font->LegacySize` to get
//    the pre-1.92 "use the size the font was added at" behaviour. Every ImFont
//    handed out below has a positive LegacySize equal to its intended
//    (already load-time-DPI-multiplied) size.
//  - AddFontFromFileTTF() asserts on a missing file unless the config carries
//    ImFontFlags_NoLoadError; we set it AND pre-check the file with fopen, so a
//    missing Segoe UI degrades to the built-in font instead of tripping
//    IM_ASSERT_USER_ERROR(0, "Could not load font file!").
//  - The atlas is dynamic: no Build() call, and re-baking for a scale change
//    happens automatically on the next frame.
//
// DPI OWNERSHIP (read before touching FontScaleDpi):
//  - LoadFonts(dpi) rasterises the atlas at logical_px * dpi and remembers dpi
//    in g_font_base_dpi. That is the ONLY place DPI is baked into type.
//  - style.FontScaleDpi belongs to the SHELL (main.cpp), not to this file. On
//    WM_DPICHANGED the shell keeps the existing atlas and sets
//        style.FontScaleDpi = new_dpi / <dpi LoadFonts was called with>
//    so the dynamic atlas re-bakes for the delta only.
//  - ApplyStyle() therefore PRESERVES whatever FontScaleDpi it finds instead of
//    resetting it to 1.0f, which is what makes it safely repeatable after a DPI
//    change. FontScaleMain is ours and stays at 1.0f.
//  - ui::DpiScale() is GEOMETRY ONLY. It is never applied to a font base size.

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "theme.h"

#include <algorithm>
#include <cstdio>

namespace ui {

Fonts fonts;

namespace {

// The DPI scale the atlas was last rasterised at, i.e. the argument of the most
// recent LoadFonts() call. Font base sizes (LegacySize) are in these units.
//
// Intentionally NOT declared in theme.h and intentionally not exposed as a free
// function: nothing outside this file needs it any more (main.cpp tracks the
// same value itself as g_font_dpi_base). If another translation unit ever does,
// add `namespace ui { float FontBaseDpi(); }` as a forward declaration there
// rather than growing the header.
float g_font_base_dpi = 1.0f;

// Geometry scale. Set by SetDpiScale(), read by DpiScale(). Never a font size.
float g_dpi_scale = 1.0f;

// ---------------------------------------------------------------- font files

#ifndef UI_FONT_DIR
#define UI_FONT_DIR "C:\\Windows\\Fonts\\"
#endif

constexpr const char* kSegoeRegular  = UI_FONT_DIR "segoeui.ttf";
constexpr const char* kSegoeSemibold = UI_FONT_DIR "seguisb.ttf";
constexpr const char* kSegoeBold     = UI_FONT_DIR "segoeuib.ttf";

bool FileReadable(const char* path)
{
    if (path == nullptr)
        return false;
    FILE* f = fopen(path, "rb");
    if (f == nullptr)
        return false;
    fclose(f);
    return true;
}

// Adds `path` at `size_px`. Returns nullptr (without asserting) when the file
// is missing or unreadable.
ImFont* TryAddFile(ImFontAtlas* atlas, const char* path, float size_px)
{
    if (!FileReadable(path))
        return nullptr;

    ImFontConfig cfg;
    cfg.SizePixels  = size_px;
    cfg.OversampleH = 2;
    cfg.OversampleV = 1;
    cfg.PixelSnapH  = false;
    cfg.Flags      |= ImFontFlags_NoLoadError; // never assert; we check the result
    return atlas->AddFontFromFileTTF(path, size_px, &cfg);
}

// Built-in font at a specific reference size, so LegacySize stays meaningful
// per role instead of defaulting to ImGui's 13px.
ImFont* AddBuiltinAt(ImFontAtlas* atlas, float size_px)
{
    ImFontConfig cfg;
    cfg.SizePixels = size_px;

#if !defined(IMGUI_DISABLE_DEFAULT_FONT) && !defined(IMGUI_DISABLE_DEFAULT_FONT_VECTOR)
    // The vector default scales cleanly to arbitrary sizes; prefer it.
    if (ImFont* f = atlas->AddFontDefaultVector(&cfg))
        return f;
#endif
#if !defined(IMGUI_DISABLE_DEFAULT_FONT)
    if (ImFont* f = atlas->AddFontDefault(&cfg))
        return f;
#endif
    return nullptr;
}

// First readable candidate wins, else the built-in font. Guaranteed non-null as
// long as ImGui has any default font compiled in; LoadFonts() applies one final
// safety net regardless.
ImFont* LoadRole(ImFontAtlas* atlas, float size_px,
                 const char* first, const char* second, const char* third)
{
    if (ImFont* f = TryAddFile(atlas, first, size_px))
        return f;
    if (ImFont* f = TryAddFile(atlas, second, size_px))
        return f;
    if (ImFont* f = TryAddFile(atlas, third, size_px))
        return f;
    return AddBuiltinAt(atlas, size_px);
}

// A border/separator thickness that survives ScaleAllSizes()' truncation.
float Hairline(float dpi_scale)
{
    const float v = (float)(int)(1.0f * dpi_scale);
    return v >= 1.0f ? v : 1.0f;
}

} // namespace

// ---------------------------------------------------------------------------

void LoadFonts(float dpi_scale)
{
    if (!(dpi_scale > 0.0f))
        dpi_scale = 1.0f;

    g_font_base_dpi = dpi_scale;

    ImGuiIO& io = ImGui::GetIO();
    ImFontAtlas* atlas = io.Fonts;

    // Safe to re-run before the first frame and between frames. The atlas is
    // only marked Locked inside NewFrame()..EndFrame() on backends without
    // dynamic texture support; touching it then would assert.
    if (!atlas->Locked)
        atlas->Clear();

    // The type scale from theme.h, in logical px, multiplied by the LOAD-TIME
    // DPI exactly once. These are the values that end up in ImFont::LegacySize
    // and therefore the base sizes callers push.
    const float sz_small = size::small * dpi_scale;
    const float sz_body  = size::body  * dpi_scale;
    const float sz_title = size::title * dpi_scale;
    const float sz_stat  = size::stat  * dpi_scale;
    const float sz_big   = size::big   * dpi_scale;

    // Regular for reading sizes, semibold/bold for the headline roles so the
    // hierarchy reads by weight as well as size.
    fonts.small = LoadRole(atlas, sz_small, kSegoeRegular,  kSegoeSemibold, nullptr);
    fonts.body  = LoadRole(atlas, sz_body,  kSegoeRegular,  kSegoeSemibold, nullptr);
    fonts.title = LoadRole(atlas, sz_title, kSegoeSemibold, kSegoeBold,     kSegoeRegular);
    fonts.stat  = LoadRole(atlas, sz_stat,  kSegoeSemibold, kSegoeBold,     kSegoeRegular);
    fonts.big   = LoadRole(atlas, sz_big,   kSegoeBold,     kSegoeSemibold, kSegoeRegular);

    // Safety net: nothing here may ever be null. Pushing a null ImFont* is an
    // outright crash, so this is load-bearing rather than defensive noise.
    ImFont* any = fonts.body;
    if (any == nullptr)
    {
        ImFont* const candidates[] = { fonts.small, fonts.title, fonts.stat, fonts.big };
        for (ImFont* c : candidates)
            if (c != nullptr) { any = c; break; }
    }
    if (any == nullptr)
        any = AddBuiltinAt(atlas, sz_body);
    if (any == nullptr && atlas->Fonts.Size > 0)
        any = atlas->Fonts[0];

    IM_ASSERT(any != nullptr && "No font could be loaded (default font compiled out?)");

    if (fonts.small == nullptr) fonts.small = any;
    if (fonts.body  == nullptr) fonts.body  = any;
    if (fonts.title == nullptr) fonts.title = any;
    if (fonts.stat  == nullptr) fonts.stat  = any;
    if (fonts.big   == nullptr) fonts.big   = any;

    // Callers derive their PushFont() base size from LegacySize, and a
    // zero/negative there silently degrades to "keep the current size". The
    // atlas copies ImFontConfig::SizePixels into LegacySize, but a shared
    // fallback carries the size of whichever role first loaded it - so re-stamp
    // anything non-positive.
    ImFont* const roles[] = { fonts.small, fonts.body, fonts.title, fonts.stat, fonts.big };
    const float   sizes[] = { sz_small,    sz_body,    sz_title,    sz_stat,    sz_big    };
    for (int i = 0; i < 5; ++i)
        if (roles[i] != nullptr && !(roles[i]->LegacySize > 0.0f))
            roles[i]->LegacySize = sizes[i];

    // Untagged text must be the readable body size, not ImGui's 13px default.
    // Fonts[0] is `small`, so this assignment is what makes body the default.
    io.FontDefault = fonts.body;

    // Base size for un-pushed text. PRE-SCALE value: FontScaleMain/FontScaleDpi
    // are applied on top of it by ImGui.
    ImGui::GetStyle().FontSizeBase = sz_body;
}

// ---------------------------------------------------------------------------

void ApplyStyle(float dpi_scale)
{
    if (!(dpi_scale > 0.0f))
        dpi_scale = 1.0f;

    ImGuiStyle& style = ImGui::GetStyle();

    // style.FontScaleDpi is the SHELL's: after WM_DPICHANGED it holds
    // new_dpi / font_base_dpi so the existing atlas re-bakes for the delta.
    // Capture it before the reset below and hand it straight back, otherwise a
    // second ApplyStyle() after a DPI change would silently revert all text to
    // the pre-change size. This makes ApplyStyle() safe in either order:
    //   - shell sets FontScaleDpi then calls ApplyStyle()  -> preserved here
    //   - shell calls ApplyStyle() then sets FontScaleDpi  -> shell wins
    const float shell_font_scale_dpi = style.FontScaleDpi;

    // Reset to defaults so ApplyStyle() is idempotent: ScaleAllSizes()
    // accumulates into style._MainScale otherwise, and repeated calls on a DPI
    // change would compound.
    style = ImGuiStyle();

    // Stock Dear ImGui dark. No hand-authored palette: every ImGuiCol_ below
    // this line is whatever upstream ships.
    ImGui::StyleColorsDark();

    // Default (un-pushed) text size follows the body font LoadFonts() actually
    // registered, so the two can never drift apart. Pre-scale value, and NOT
    // touched by ScaleAllSizes() - dpi is already baked into LegacySize.
    style.FontSizeBase = (fonts.body != nullptr && fonts.body->LegacySize > 0.0f)
                       ? fonts.body->LegacySize
                       : size::body * dpi_scale;

    // FontScaleMain is an app/user zoom knob we do not use.
    style.FontScaleMain = 1.0f;
    style.FontScaleDpi  = (shell_font_scale_dpi > 0.0f) ? shell_font_scale_dpi : 1.0f;

    // ------------------------------------------------------------- palette
    // Stock ImGui dark, with one deliberate override: every SURFACE background
    // is solid black. Widget colours (buttons, headers, tabs, sliders) are left
    // exactly as upstream ships them, so this still reads as a Dear ImGui app
    // rather than a themed one.
    //
    // "Surface" means the things a panel is drawn ON - window, child, popup,
    // title, menu bar, scrollbar trough, table rows. Opaque, not translucent:
    // stock ChildBg/PopupBg carry alpha, and translucent panels over a black
    // ground just produce muddy near-blacks that differ by a few counts.
    auto black = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);

    ImVec4* c = style.Colors;
    c[ImGuiCol_WindowBg]         = black;
    c[ImGuiCol_ChildBg]          = black;
    c[ImGuiCol_PopupBg]          = black;
    c[ImGuiCol_MenuBarBg]        = black;
    c[ImGuiCol_TitleBg]          = black;
    c[ImGuiCol_TitleBgActive]    = black;
    c[ImGuiCol_TitleBgCollapsed] = black;
    c[ImGuiCol_ScrollbarBg]      = black;
    c[ImGuiCol_TableRowBg]       = black;

    // Alternating table rows would be invisible on black, so the zebra is a
    // faint lift rather than a second colour.
    c[ImGuiCol_TableRowBgAlt]    = ImVec4(1.0f, 1.0f, 1.0f, 0.03f);

    // Frame backgrounds too - these are what the plots, combos and text fields
    // are drawn on, and they were the last thing left reading blue.
    c[ImGuiCol_FrameBg]        = black;
    // Hover/active still have to be visible, so they lift off black slightly
    // rather than staying pure. Without this, a combo gives no feedback at all.
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.16f, 0.16f, 0.16f, 1.0f);
    c[ImGuiCol_FrameBgActive]  = ImVec4(0.24f, 0.24f, 0.24f, 1.0f);

    // ------------------------------------------------------------ geometry
    // Square corners, stock padding, stock spacing, stock scrollbar - straight
    // from ImGuiStyle()'s defaults.
    //
    // ONE deviation: FrameBorderSize goes 0 -> 1. Stock ImGui relies on FrameBg
    // being lighter than the window to show where a widget is; with both black
    // that cue is gone and combos and inputs become invisible rectangles. The
    // border puts the edge back. Everything else is untouched.
    style.FrameBorderSize = 1.0f;

    // Convert the logical sizes into physical px, exactly once.
    style.ScaleAllSizes(dpi_scale);

    // ScaleAllSizes() truncates, which can zero out 1px lines at dpi < 1, and
    // ImGui requires SeparatorSize >= 1.0f.
    style.SeparatorSize           = Hairline(dpi_scale);
    style.SeparatorTextBorderSize = Hairline(dpi_scale);
    style.WindowBorderSize        = Hairline(dpi_scale);
    style.ChildBorderSize         = Hairline(dpi_scale);
    style.PopupBorderSize         = Hairline(dpi_scale);
}

// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------

void SetDpiScale(float scale)
{
    g_dpi_scale = (scale > 0.0f) ? scale : 1.0f;
}

float DpiScale()
{
    return g_dpi_scale;
}

} // namespace ui
