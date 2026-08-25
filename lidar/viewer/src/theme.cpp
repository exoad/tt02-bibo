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
    // Frutiger Aero: glossy, glassy, saturated sky-blue and cyan, generous
    // rounding. The look is mid-2000s Windows Aero / iOS-before-flat.
    //
    // Applied to CHROME ONLY - buttons, headers, tabs, frames, panels. The map,
    // the plots and the log keep flat dark backgrounds, because gloss and
    // translucency cost contrast and those surfaces carry the actual data.
    // A pretty instrument you cannot read is a worse instrument.
    //
    // Real frosted glass needs a blur pass the DX11 backend does not have, so
    // "glass" here is translucency plus a vertical gradient (see AeroGloss in
    // widgets drawn by app_ui) - which is most of the effect anyway.

    auto C = [](int r, int g, int b, int a) {
        return ImVec4(r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f);
    };

    ImVec4* c = style.Colors;

    // Deep blue-black ground, so the saturated chrome reads as lit from within.
    c[ImGuiCol_WindowBg]            = C(0x0C, 0x16, 0x24, 0xFF);
    c[ImGuiCol_ChildBg]             = C(0x10, 0x1E, 0x30, 0x66);
    c[ImGuiCol_PopupBg]             = C(0x0E, 0x1B, 0x2C, 0xF2);

    c[ImGuiCol_Text]                = C(0xE8, 0xF4, 0xFF, 0xFF);
    c[ImGuiCol_TextDisabled]        = C(0x7E, 0x9A, 0xB4, 0xFF);

    // Borders are a pale highlight, not a dark outline - edges catch the light.
    c[ImGuiCol_Border]              = C(0x6F, 0xC8, 0xF0, 0x59);
    c[ImGuiCol_BorderShadow]        = C(0x00, 0x00, 0x00, 0x00);

    // Inputs read as recessed glass.
    c[ImGuiCol_FrameBg]             = C(0x14, 0x2C, 0x46, 0xD9);
    c[ImGuiCol_FrameBgHovered]      = C(0x1E, 0x40, 0x63, 0xE6);
    c[ImGuiCol_FrameBgActive]       = C(0x27, 0x55, 0x80, 0xF2);

    c[ImGuiCol_TitleBg]             = C(0x0E, 0x22, 0x38, 0xFF);
    c[ImGuiCol_TitleBgActive]       = C(0x18, 0x3E, 0x62, 0xFF);
    c[ImGuiCol_TitleBgCollapsed]    = C(0x0C, 0x18, 0x28, 0xBF);
    c[ImGuiCol_MenuBarBg]           = C(0x12, 0x28, 0x40, 0xFF);

    c[ImGuiCol_ScrollbarBg]         = C(0x0A, 0x14, 0x20, 0x80);
    c[ImGuiCol_ScrollbarGrab]       = C(0x2E, 0x74, 0xA8, 0xCC);
    c[ImGuiCol_ScrollbarGrabHovered]= C(0x3E, 0x96, 0xD4, 0xE6);
    c[ImGuiCol_ScrollbarGrabActive] = C(0x55, 0xB6, 0xF0, 0xFF);

    // The aqua accent. Everything interactive lands on this hue.
    c[ImGuiCol_CheckMark]           = C(0x6F, 0xE4, 0xFF, 0xFF);
    c[ImGuiCol_SliderGrab]          = C(0x40, 0xB4, 0xE8, 0xFF);
    c[ImGuiCol_SliderGrabActive]    = C(0x76, 0xDC, 0xFF, 0xFF);

    c[ImGuiCol_Button]              = C(0x1C, 0x55, 0x86, 0xE6);
    c[ImGuiCol_ButtonHovered]       = C(0x2E, 0x83, 0xC4, 0xF2);
    c[ImGuiCol_ButtonActive]        = C(0x14, 0x44, 0x6E, 0xFF);

    c[ImGuiCol_Header]              = C(0x1A, 0x4C, 0x78, 0xCC);
    c[ImGuiCol_HeaderHovered]       = C(0x2A, 0x77, 0xB4, 0xE6);
    c[ImGuiCol_HeaderActive]        = C(0x36, 0x94, 0xD8, 0xF2);

    c[ImGuiCol_Separator]           = C(0x4E, 0x8C, 0xB4, 0x66);
    c[ImGuiCol_SeparatorHovered]    = C(0x6F, 0xC8, 0xF0, 0xB3);
    c[ImGuiCol_SeparatorActive]     = C(0x8A, 0xE0, 0xFF, 0xFF);

    c[ImGuiCol_ResizeGrip]          = C(0x40, 0xB4, 0xE8, 0x40);
    c[ImGuiCol_ResizeGripHovered]   = C(0x60, 0xD0, 0xFF, 0x99);
    c[ImGuiCol_ResizeGripActive]    = C(0x8A, 0xE0, 0xFF, 0xE6);

    c[ImGuiCol_Tab]                 = C(0x14, 0x36, 0x56, 0xE6);
    c[ImGuiCol_TabHovered]          = C(0x36, 0x94, 0xD8, 0xE6);
    c[ImGuiCol_TabSelected]         = C(0x22, 0x66, 0x9E, 0xFF);
    c[ImGuiCol_TabSelectedOverline] = C(0x8A, 0xE0, 0xFF, 0xFF);
    c[ImGuiCol_TabDimmed]           = C(0x0F, 0x22, 0x36, 0xE6);
    c[ImGuiCol_TabDimmedSelected]   = C(0x18, 0x44, 0x6C, 0xFF);

    // Data surfaces: flat and dark on purpose. See the note above.
    c[ImGuiCol_PlotLines]           = C(0x7C, 0xE0, 0xC8, 0xFF);
    c[ImGuiCol_PlotLinesHovered]    = C(0xA8, 0xFF, 0xE8, 0xFF);
    c[ImGuiCol_PlotHistogram]       = C(0x58, 0xC8, 0xF0, 0xFF);
    c[ImGuiCol_PlotHistogramHovered]= C(0x8A, 0xE4, 0xFF, 0xFF);

    c[ImGuiCol_TableHeaderBg]       = C(0x16, 0x33, 0x52, 0xFF);
    c[ImGuiCol_TableBorderStrong]   = C(0x3E, 0x74, 0x9C, 0x8C);
    c[ImGuiCol_TableBorderLight]    = C(0x28, 0x4C, 0x68, 0x66);
    c[ImGuiCol_TableRowBg]          = C(0x00, 0x00, 0x00, 0x00);
    c[ImGuiCol_TableRowBgAlt]       = C(0xFF, 0xFF, 0xFF, 0x0A);

    c[ImGuiCol_TextSelectedBg]      = C(0x36, 0x94, 0xD8, 0x59);
    c[ImGuiCol_NavCursor]           = C(0x8A, 0xE0, 0xFF, 0xFF);

    // ------------------------------------------------------------ geometry
    // Aero is soft-cornered and airy. Sizes are LOGICAL px here; ScaleAllSizes
    // converts the whole set exactly once, below.
    style.WindowRounding    = 8.0f;
    style.ChildRounding     = 7.0f;
    style.FrameRounding     = 6.0f;
    style.PopupRounding     = 7.0f;
    style.ScrollbarRounding = 9.0f;
    style.GrabRounding      = 6.0f;
    style.TabRounding       = 7.0f;

    style.WindowPadding     = ImVec2(10.0f, 9.0f);
    style.FramePadding      = ImVec2(9.0f, 5.0f);
    style.ItemSpacing       = ImVec2(9.0f, 7.0f);
    style.ItemInnerSpacing  = ImVec2(7.0f, 5.0f);
    style.CellPadding       = ImVec2(7.0f, 4.0f);
    style.ScrollbarSize     = 13.0f;
    style.GrabMinSize       = 11.0f;

    // A visible pale edge is a big part of the look: it is what makes a panel
    // read as a pane of glass rather than a flat rectangle.
    style.WindowBorderSize  = 1.0f;
    style.ChildBorderSize   = 1.0f;
    style.FrameBorderSize   = 1.0f;
    style.PopupBorderSize   = 1.0f;

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

void GlossRect(const ImVec2& p_min, const ImVec2& p_max, float rounding, float strength)
{
    if (strength <= 0.0f) return;

    const float w = p_max.x - p_min.x;
    const float h = p_max.y - p_min.y;
    if (w < 3.0f || h < 3.0f) return;          // too small to read as anything

    ImDrawList* dl = ImGui::GetWindowDrawList();

    const float a = (strength > 1.0f) ? 1.0f : strength;
    const float mid_y = p_min.y + h * 0.48f;

    // Rounding is clamped: a radius larger than half the short side makes
    // AddRectFilled draw a lozenge, which looks wrong on a half-height overlay.
    const float r = std::min(rounding, std::min(w, h) * 0.5f);

    // Lit top half. Drawn as a rounded rect with only the top corners rounded so
    // it follows the widget's own silhouette instead of overhanging it.
    dl->AddRectFilled(p_min, ImVec2(p_max.x, mid_y),
                      IM_COL32(255, 255, 255, (int)(30 * a)),
                      r, ImDrawFlags_RoundCornersTop);

    // Specular line just inside the top edge - the single detail that most says
    // "glass" rather than "gradient".
    dl->AddLine(ImVec2(p_min.x + r * 0.6f, p_min.y + 1.0f),
                ImVec2(p_max.x - r * 0.6f, p_min.y + 1.0f),
                IM_COL32(255, 255, 255, (int)(90 * a)), 1.0f);

    // Faint pooling of light along the bottom, as if the surface were convex.
    dl->AddRectFilled(ImVec2(p_min.x, p_max.y - h * 0.22f), p_max,
                      IM_COL32(120, 200, 255, (int)(16 * a)),
                      r, ImDrawFlags_RoundCornersBottom);
}

void GlossLastItem(float strength)
{
    if (!ImGui::IsItemVisible()) return;
    GlossRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax(),
              ImGui::GetStyle().FrameRounding, strength);
}

void SkyBackdrop(const ImVec2& p_min, const ImVec2& p_max, float rounding)
{
    const float w = p_max.x - p_min.x;
    const float h = p_max.y - p_min.y;
    if (w < 2.0f || h < 2.0f) return;

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // AddRectFilledMultiColor has no rounding, so when a rounded surface is
    // asked for the gradient is clipped to a rounded rect instead.
    const bool round = (rounding > 0.0f);
    if (round)
        dl->PushClipRect(p_min, p_max, true);

    dl->AddRectFilledMultiColor(p_min, p_max,
                                IM_COL32(0x1B, 0x3E, 0x60, 0xFF),   // lit top
                                IM_COL32(0x1B, 0x3E, 0x60, 0xFF),
                                IM_COL32(0x0A, 0x14, 0x22, 0xFF),   // deep base
                                IM_COL32(0x0A, 0x14, 0x22, 0xFF));

    // Horizon: a thin brighter band where the sky meets the ground. Aero
    // wallpapers almost always have one, and it stops the gradient reading as a
    // flat wash.
    const float y = p_min.y + h * 0.34f;
    dl->AddRectFilledMultiColor(ImVec2(p_min.x, y), ImVec2(p_max.x, y + h * 0.10f),
                                IM_COL32(0x4E, 0x9E, 0xD0, 0x00),
                                IM_COL32(0x4E, 0x9E, 0xD0, 0x00),
                                IM_COL32(0x4E, 0x9E, 0xD0, 0x2E),
                                IM_COL32(0x4E, 0x9E, 0xD0, 0x2E));

    if (round)
        dl->PopClipRect();
}

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
