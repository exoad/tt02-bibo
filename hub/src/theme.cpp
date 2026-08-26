// The app's look: Frutiger Aero colour on Unreal/IntelliJ geometry.
//
// StyleColorsDark() is called first only to guarantee every ImGuiCol_ has some
// value; essentially all of them are then overwritten below. The palette IS
// hand-authored - see the block in applyStyle() for what the three anchor
// colours are and why chrome is steel rather than neutral grey.
//
// Geometry (rounding / padding / spacing) is tuned so the look still breathes at
// the larger font sizes in ui::size.
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
//  - loadFonts(dpi) rasterises the atlas at logical_px * dpi and remembers dpi
//    in fontBaseDpi. That is the ONLY place DPI is baked into type.
//  - style.FontScaleDpi belongs to the SHELL (main.cpp), not to this file. On
//    WM_DPICHANGED the shell keeps the existing atlas and sets
//        style.FontScaleDpi = new_dpi / <dpi loadFonts was called with>
//    so the dynamic atlas re-bakes for the delta only.
//  - applyStyle() therefore PRESERVES whatever FontScaleDpi it finds instead of
//    resetting it to 1.0f, which is what makes it safely repeatable after a DPI
//    change. FontScaleMain is ours and stays at 1.0f.
//  - ui::dpiScale() is GEOMETRY ONLY. It is never applied to a font base size.

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "shared.hpp"
#include "theme.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace ui {

Fonts fonts;

namespace {

// The DPI scale the atlas was last rasterised at, i.e. the argument of the most
// recent loadFonts() call. Font base sizes (LegacySize) are in these units.
//
// Intentionally NOT declared in theme.h and intentionally not exposed as a free
// function: nothing outside this file needs it any more (main.cpp tracks the
// same value itself as fontDpiBase). If another translation unit ever does,
// add `namespace ui { float FontBaseDpi(); }` as a forward declaration there
// rather than growing the header.
Float32 fontBaseDpi = 1.0f;

// Geometry scale. Set by setDpiScale(), read by dpiScale(). Never a font size.
Float32 geometryDpiScale = 1.0f;

// ---------------------------------------------------------------- font files

#ifndef UI_FONT_DIR
#define UI_FONT_DIR "C:\\Windows\\Fonts\\"
#endif

constexpr const Char* SEGOE_REGULAR  = UI_FONT_DIR "segoeui.ttf";
constexpr const Char* SEGOE_SEMIBOLD = UI_FONT_DIR "seguisb.ttf";
constexpr const Char* SEGOE_BOLD     = UI_FONT_DIR "segoeuib.ttf";

// Monospace, best first. Cascadia Mono ships with Windows Terminal and modern
// Windows 11; Consolas is on every Windows since Vista; Lucida Console is the
// floor. One of these exists on any machine this will ever run on.
constexpr const Char* MONO_CASCADIA  = UI_FONT_DIR "CascadiaMono.ttf";
constexpr const Char* MONO_CONSOLAS  = UI_FONT_DIR "consola.ttf";
constexpr const Char* MONO_LUCIDA    = UI_FONT_DIR "lucon.ttf";

Bool fileReadable(const Char* path)
{
    if(path == nullptr)
        return false;
    FILE* f = fopen(path, "rb");
    if(f == nullptr)
        return false;
    fclose(f);
    return true;
}

// Adds `path` at `sizePx`. Returns nullptr (without asserting) when the file
// is missing or unreadable.
ImFont* tryAddFile(ImFontAtlas* atlas, const Char* path, Float32 sizePx)
{
    if(!fileReadable(path))
        return nullptr;

    ImFontConfig cfg;
    cfg.SizePixels  = sizePx;
    cfg.OversampleH = 2;
    cfg.OversampleV = 1;
    cfg.PixelSnapH  = false;
    cfg.Flags      |= ImFontFlags_NoLoadError; // never assert; we check the result
    return atlas->AddFontFromFileTTF(path, sizePx, &cfg);
}

// Built-in font at a specific reference size, so LegacySize stays meaningful
// per role instead of defaulting to ImGui's 13px.
ImFont* addBuiltinAt(ImFontAtlas* atlas, Float32 sizePx)
{
    ImFontConfig cfg;
    cfg.SizePixels = sizePx;

#if !defined(IMGUI_DISABLE_DEFAULT_FONT) && !defined(IMGUI_DISABLE_DEFAULT_FONT_VECTOR)
    // The vector default scales cleanly to arbitrary sizes; prefer it.
    if(ImFont* f = atlas->AddFontDefaultVector(&cfg))
        return f;
#endif
#if !defined(IMGUI_DISABLE_DEFAULT_FONT)
    if(ImFont* f = atlas->AddFontDefault(&cfg))
        return f;
#endif
    return nullptr;
}

// First readable candidate wins, else the built-in font. Guaranteed non-null as
// long as ImGui has any default font compiled in; LoadFonts() applies one final
// safety net regardless.
ImFont* loadRole(ImFontAtlas* atlas, Float32 sizePx,
                 const Char* first, const Char* second, const Char* third)
{
    if(ImFont* f = tryAddFile(atlas, first, sizePx))
        return f;
    if(ImFont* f = tryAddFile(atlas, second, sizePx))
        return f;
    if(ImFont* f = tryAddFile(atlas, third, sizePx))
        return f;
    return addBuiltinAt(atlas, sizePx);
}

// A border/separator thickness that survives ScaleAllSizes()' truncation.
Float32 hairline(Float32 dpiScale)
{
    const Float32 v = static_cast<Float32>(static_cast<Int32>((1.0f * dpiScale)));
    return v >= 1.0f ? v : 1.0f;
}

} // namespace

// ---------------------------------------------------------------------------

Void loadFonts(Float32 dpiScale)
{
    if(!(dpiScale > 0.0f))
        dpiScale = 1.0f;

    fontBaseDpi = dpiScale;

    ImGuiIO& io = ImGui::GetIO();
    ImFontAtlas* atlas = io.Fonts;

    // Safe to re-run before the first frame and between frames. The atlas is
    // only marked Locked inside NewFrame()..EndFrame() on backends without
    // dynamic texture support; touching it then would assert.
    if(!atlas->Locked)
        atlas->Clear();

    // The type scale from theme.h, in logical px, multiplied by the LOAD-TIME
    // DPI exactly once. These are the values that end up in ImFont::LegacySize
    // and therefore the base sizes callers push.
    const Float32 szSmall = size::SMALL * dpiScale;
    const Float32 szBody  = size::BODY  * dpiScale;
    const Float32 szTitle = size::TITLE * dpiScale;
    const Float32 szStat  = size::STAT  * dpiScale;
    const Float32 szBig   = size::BIG   * dpiScale;
    const Float32 szCode  = size::CODE  * dpiScale;

    // Regular for reading sizes, semibold/bold for the headline roles so the
    // hierarchy reads by weight as well as size.
    fonts.small = loadRole(atlas, szSmall, SEGOE_REGULAR,  SEGOE_SEMIBOLD, nullptr);
    fonts.body  = loadRole(atlas, szBody,  SEGOE_REGULAR,  SEGOE_SEMIBOLD, nullptr);
    fonts.title = loadRole(atlas, szTitle, SEGOE_SEMIBOLD, SEGOE_BOLD,     SEGOE_REGULAR);
    fonts.stat  = loadRole(atlas, szStat,  SEGOE_SEMIBOLD, SEGOE_BOLD,     SEGOE_REGULAR);
    fonts.big   = loadRole(atlas, szBig,   SEGOE_BOLD,     SEGOE_SEMIBOLD, SEGOE_REGULAR);
    fonts.mono  = loadRole(atlas, szCode,  MONO_CASCADIA,  MONO_CONSOLAS,  MONO_LUCIDA);

    // Safety net: nothing here may ever be null. Pushing a null ImFont* is an
    // outright crash, so this is load-bearing rather than defensive noise.
    ImFont* any = fonts.body;
    if(any == nullptr)
    {
        ImFont* const candidates[] = { fonts.small, fonts.title, fonts.stat, fonts.big, fonts.mono };
        for(ImFont* c : candidates)
            if(c != nullptr)
            {
                any = c;
                break;
            }
    }
    if(any == nullptr)
        any = addBuiltinAt(atlas, szBody);
    if(any == nullptr && atlas->Fonts.Size > 0)
        any = atlas->Fonts[0];

    IM_ASSERT(any != nullptr && "No font could be loaded (default font compiled out?)");

    if(fonts.small == nullptr) fonts.small = any;
    if(fonts.body  == nullptr) fonts.body  = any;
    if(fonts.title == nullptr) fonts.title = any;
    if(fonts.stat  == nullptr) fonts.stat  = any;
    if(fonts.big   == nullptr) fonts.big   = any;
    if(fonts.mono  == nullptr) fonts.mono  = any;

    // Callers derive their PushFont() base size from LegacySize, and a
    // zero/negative there silently degrades to "keep the current size". The
    // atlas copies ImFontConfig::SizePixels into LegacySize, but a shared
    // fallback carries the size of whichever role first loaded it - so re-stamp
    // anything non-positive.
    ImFont* const roles[] = { fonts.small, fonts.body, fonts.title, fonts.stat, fonts.big, fonts.mono };
    const Float32   sizes[] = { szSmall,    szBody,    szTitle,    szStat,    szBig,     szCode     };
    for(Int32 i = 0; i < 6; ++i)
        if(roles[i] != nullptr && !(roles[i]->LegacySize > 0.0f))
            roles[i]->LegacySize = sizes[i];

    // Untagged text must be the readable body size, not ImGui's 13px default.
    // Fonts[0] is `small`, so this assignment is what makes body the default.
    io.FontDefault = fonts.body;

    // Base size for un-pushed text. PRE-SCALE value: FontScaleMain/FontScaleDpi
    // are applied on top of it by ImGui.
    ImGui::GetStyle().FontSizeBase = szBody;
}

// ---------------------------------------------------------------------------

Void applyStyle(Float32 dpiScale)
{
    if(!(dpiScale > 0.0f))
        dpiScale = 1.0f;

    ImGuiStyle& style = ImGui::GetStyle();

    // style.FontScaleDpi is the SHELL's: after WM_DPICHANGED it holds
    // new_dpi / font_base_dpi so the existing atlas re-bakes for the delta.
    // Capture it before the reset below and hand it straight back, otherwise a
    // second applyStyle() after a DPI change would silently revert all text to
    // the pre-change size. This makes applyStyle() safe in either order:
    //   - shell sets FontScaleDpi then calls applyStyle()  -> preserved here
    //   - shell calls applyStyle() then sets FontScaleDpi  -> shell wins
    const Float32 shellFontScaleDpi = style.FontScaleDpi;

    // Reset to defaults so applyStyle() is idempotent: ScaleAllSizes()
    // accumulates into style._MainScale otherwise, and repeated calls on a DPI
    // change would compound.
    style = ImGuiStyle();

    // Stock Dear ImGui dark. No hand-authored palette: every ImGuiCol_ below
    // this line is whatever upstream ships.
    ImGui::StyleColorsDark();

    // Default (un-pushed) text size follows the body font loadFonts() actually
    // registered, so the two can never drift apart. Pre-scale value, and NOT
    // touched by ScaleAllSizes() - dpi is already baked into LegacySize.
    style.FontSizeBase = (fonts.body != nullptr && fonts.body->LegacySize > 0.0f)
                       ? fonts.body->LegacySize
                       : size::BODY * dpiScale;

    // FontScaleMain is an app/user zoom knob we do not use.
    style.FontScaleMain = 1.0f;
    style.FontScaleDpi  = (shellFontScaleDpi > 0.0f) ? shellFontScaleDpi : 1.0f;

    // ------------------------------------------------------------- palette
    // Industrial slate: the dark-graphite console of UDK / Maya / Blender 2.7x /
    // Photoshop CS5, with Dark Aero's LED accents set into it.
    //
    // GRAPHITE, NOT BLACK. This is the change that matters most and it is the
    // opposite of what the app used to do. A pure black ground with near-white
    // text is the highest-contrast pairing available and it is punishing over a
    // long session - which is exactly why every tool in that reference list sits
    // in a narrow low-contrast band instead. Panels live around 15-22% grey and
    // the type floats a little way above them rather than blazing off them.
    //
    // The one thing kept from black: the lidar VIEWPORT, which stays darker than
    // the chrome. It is a display set into the console, not part of the casing,
    // and the point cloud needs the contrast the chrome deliberately gives up.
    auto  slate = [](Float32 v, Float32 a = 1.0f) {
        // Faintly cool, the way graphite reads under workshop light.
        return ImVec4(v * 0.96f, v * 0.99f, v * 1.06f, a);
    };
    auto  grey  = [](Float32 v, Float32 a = 1.0f) { return ImVec4(v, v, v, a); };

    const ImVec4 accent   = ImGui::ColorConvertU32ToFloat4(accent::CYAN);
    const ImVec4 accentHi = ImGui::ColorConvertU32ToFloat4(accent::CYAN_HI);

    ImVec4* c = style.Colors;

    // ---- the casing ----
    c[ImGuiCol_WindowBg]         = slate(0.190f);
    c[ImGuiCol_ChildBg]          = slate(0.220f);
    c[ImGuiCol_PopupBg]          = slate(0.240f);
    c[ImGuiCol_MenuBarBg]        = slate(0.210f);
    c[ImGuiCol_TitleBg]          = slate(0.160f);
    c[ImGuiCol_TitleBgActive]    = slate(0.195f);
    c[ImGuiCol_TitleBgCollapsed] = slate(0.160f);
    c[ImGuiCol_TableRowBg]       = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_TableRowBgAlt]    = grey(1.0f, 0.022f);

    // ---- wells: a field is milled INTO the casing, so it is darker ----
    c[ImGuiCol_FrameBg]          = slate(0.120f);
    c[ImGuiCol_FrameBgHovered]   = slate(0.150f);
    c[ImGuiCol_FrameBgActive]    = slate(0.100f);

    // ---- keys: pushed OUT of the casing, so they are lighter ----
    // The bevel in ui::bevel is what sells this; these are the plate under it.
    c[ImGuiCol_Button]           = slate(0.300f);
    c[ImGuiCol_ButtonHovered]    = slate(0.360f);
    c[ImGuiCol_ButtonActive]     = slate(0.215f);   // sinks when pressed

    c[ImGuiCol_Header]           = slate(0.275f);
    c[ImGuiCol_HeaderHovered]    = slate(0.330f);
    c[ImGuiCol_HeaderActive]     = slate(0.370f);

    c[ImGuiCol_Tab]              = slate(0.225f);
    c[ImGuiCol_TabHovered]       = slate(0.325f);
    c[ImGuiCol_TabSelected]      = slate(0.295f);
    c[ImGuiCol_TabDimmed]        = slate(0.190f);
    c[ImGuiCol_TabDimmedSelected]= slate(0.245f);
    c[ImGuiCol_TabSelectedOverline] = accent;

    // ---- edges: a dark seam between parts, as machined panels have ----
    c[ImGuiCol_Border]           = grey(0.0f, 0.55f);
    c[ImGuiCol_BorderShadow]     = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_Separator]        = grey(0.0f, 0.42f);
    c[ImGuiCol_SeparatorHovered] = accent;
    c[ImGuiCol_SeparatorActive]  = accentHi;

    // ---- controls ----
    c[ImGuiCol_CheckMark]        = accentHi;
    c[ImGuiCol_SliderGrab]       = accent;
    c[ImGuiCol_SliderGrabActive] = accentHi;

    c[ImGuiCol_ScrollbarBg]          = slate(0.140f);
    c[ImGuiCol_ScrollbarGrab]        = slate(0.330f);
    c[ImGuiCol_ScrollbarGrabHovered] = slate(0.410f);
    c[ImGuiCol_ScrollbarGrabActive]  = slate(0.490f);
    c[ImGuiCol_ResizeGrip]           = grey(1.0f, 0.08f);
    c[ImGuiCol_ResizeGripHovered]    = grey(1.0f, 0.18f);
    c[ImGuiCol_ResizeGripActive]     = grey(1.0f, 0.30f);

    c[ImGuiCol_TableHeaderBg]     = slate(0.260f);
    c[ImGuiCol_TableBorderStrong] = grey(0.0f, 0.45f);
    c[ImGuiCol_TableBorderLight]  = grey(0.0f, 0.25f);
    c[ImGuiCol_TextSelectedBg]    = ImVec4(accent.x, accent.y, accent.z, 0.32f);
    c[ImGuiCol_InputTextCursor]   = accentHi;

    // ---- the rest of the palette -----------------------------------------
    c[ImGuiCol_PlotLines]            = slate(0.62f);
    c[ImGuiCol_PlotLinesHovered]     = accentHi;
    c[ImGuiCol_PlotHistogram]        = accent;
    c[ImGuiCol_PlotHistogramHovered] = accentHi;

    c[ImGuiCol_NavCursor]            = accent;
    c[ImGuiCol_DragDropTarget]       = accentHi;
    c[ImGuiCol_TextLink]             = accentHi;

    // Type sits ABOVE the panels, not blazing off them. #D6DBE0 on ~18% grey is
    // a comfortable long-session contrast; near-white on black was not.
    c[ImGuiCol_Text]                 = ImVec4(0.839f, 0.859f, 0.878f, 1.00f);
    c[ImGuiCol_TextDisabled]         = ImVec4(0.478f, 0.510f, 0.541f, 1.00f);

    // ------------------------------------------------------------ geometry
    // Very nearly square. 2px takes the bite off a hard corner without the
    // control reading as a rounded tile; containers are square outright. This is
    // the single change that does the most to move the app from "Material" to
    // "tool" - a 5px radius reads as a card no matter what colour it is.
    style.FrameRounding     = 2.0f;
    style.GrabRounding      = 2.0f;
    style.TabRounding       = 2.0f;
    style.ChildRounding     = 0.0f;
    style.WindowRounding    = 0.0f;
    style.PopupRounding     = 2.0f;
    style.ScrollbarRounding = 0.0f;

    // Widgets are outlined; containers are not. Both of these tools ring every
    // control and let panels run flush into one another, and the earlier
    // complaint was specifically about container borders - sidebars and boxes
    // ringed inside other ringed boxes. That stays fixed.
    style.FrameBorderSize  = 1.0f;    // scaled by ScaleAllSizes below
    style.WindowBorderSize = 0.0f;
    style.ChildBorderSize  = 0.0f;

    // DENSE. This is a workstation toolbar, not a settings page: the reference
    // tools pack controls close together and let the outlines and bevels do the
    // separating that whitespace does elsewhere. Every value here is smaller
    // than a general-purpose UI would use, on purpose.
    style.ItemSpacing      = ImVec2(5.0f, 3.0f);
    style.ItemInnerSpacing = ImVec2(4.0f, 3.0f);
    style.FramePadding     = ImVec2(6.0f, 3.0f);
    style.CellPadding      = ImVec2(5.0f, 2.0f);
    style.WindowPadding    = ImVec2(6.0f, 5.0f);
    style.IndentSpacing    = 14.0f;

    // A thin scrollbar. The stock 14px bar is a touch-sized affordance; a tool
    // that expects a mouse gives the space to the content.
    style.ScrollbarSize    = 10.0f;
    style.GrabMinSize      = 9.0f;

    // The selected tab is marked by its overline, so the overline has to be
    // thick enough to see against a 2px-rounded tab.
    style.TabBarOverlineSize = 3.0f;
    style.TabBorderSize      = 0.0f;
    style.TabBarBorderSize   = 1.0f;

    // Convert the logical sizes into physical px, exactly once.
    style.ScaleAllSizes(dpiScale);

    // ScaleAllSizes() truncates, which can zero out 1px lines at dpi < 1, and
    // ImGui requires SeparatorSize >= 1.0f.
    style.SeparatorSize           = hairline(dpiScale);
    style.SeparatorTextBorderSize = hairline(dpiScale);

    // Same reason: an outline that rounds to zero at some DPI would silently
    // undo the entire look on that machine.
    style.FrameBorderSize         = hairline(dpiScale);

    // Popups are the one exception: they float over arbitrary content and need
    // an edge to sit against, or a modal reads as text pasted onto the page.
    style.PopupBorderSize         = hairline(dpiScale);
}

// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------

Void shadeRect(const ImVec2& pMin, const ImVec2& pMax, Float32 strength)
{
    bevelRect(pMin, pMax, false, strength);
}

// ---------------------------------------------------------------------------
// The bevel
//
// This is what makes a control read as a physical key pushed out of the casing
// rather than a coloured rectangle: one pixel of light along the TOP edge, one
// pixel of shadow along the BOTTOM. Both invert when the key is pressed, so it
// visibly sinks into the panel instead of merely changing colour.
//
// Drawn OVER an item ImGui has already submitted, so ImGui::Button stays a real
// ImGui::Button and keeps its sizing, hover, activation and keyboard nav.
//
// One pixel means ONE PHYSICAL PIXEL, not one scaled unit. A bevel that grows
// with DPI stops reading as a machined edge and starts reading as a border,
// which is a different thing entirely - so this is the one measurement in the
// file that deliberately ignores the DPI scale.
// ---------------------------------------------------------------------------
Void bevelRect(const ImVec2& pMin, const ImVec2& pMax, Bool pressed, Float32 strength)
{
    if(strength <= 0.0f)
        return;

    const Float32 w = pMax.x - pMin.x;
    const Float32 h = pMax.y - pMin.y;
    if(w < 4.0f || h < 4.0f)
        return;

    const Float32 a = (strength > 1.0f) ? 1.0f : strength;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // GLOSS. Dark Aero's panels are glossy charcoal, not matte, so a raised
    // control carries a soft sheen over its top half - light falling on a
    // moulded surface. Kept low: this has to read as a finish, never as a
    // gradient, and it is the first thing that tips into looking cheap.
    //
    // Clipped rather than rounded because AddRectFilledMultiColor cannot round
    // its corners; at these alphas the square corners under a 2px radius are not
    // perceptible.
    if(!pressed)
    {
        dl->PushClipRect(pMin, pMax, true);
        const Float32 mid = pMin.y + h * 0.52f;
        const ImU32 top = IM_COL32(255, 255, 255, static_cast<Int32>(15 * a));
        const ImU32 nil = IM_COL32(255, 255, 255, 0);
        dl->AddRectFilledMultiColor(pMin, ImVec2(pMax.x, mid), top, top, nil, nil);
        dl->PopClipRect();
    }

    const ImU32 lit   = IM_COL32(255, 255, 255, static_cast<Int32>(42 * a));
    const ImU32 shade = IM_COL32(0, 0, 0, static_cast<Int32>(105 * a));

    const ImU32 top    = pressed ? shade : lit;
    const ImU32 bottom = pressed ? lit   : shade;

    // Inset by the frame rounding so the highlight does not stick out past the
    // rounded corners as a pair of stray dots.
    const Float32 r = ImGui::GetStyle().FrameRounding;
    dl->AddLine(ImVec2(pMin.x + r, pMin.y + 0.5f),
                ImVec2(pMax.x - r, pMin.y + 0.5f), top, 1.0f);
    dl->AddLine(ImVec2(pMin.x + r, pMax.y - 0.5f),
                ImVec2(pMax.x - r, pMax.y - 0.5f), bottom, 1.0f);

    // A pressed key also shadows itself along its top inner edge, the way a real
    // one does when it drops into the casing.
    if(pressed)
        dl->AddLine(ImVec2(pMin.x + r, pMin.y + 1.5f),
                    ImVec2(pMax.x - r, pMin.y + 1.5f),
                    IM_COL32(0, 0, 0, static_cast<Int32>(55 * a)), 1.0f);
}

// ---------------------------------------------------------------------------
// An indicator LED
//
// Not a filled circle: a lit LED on a dark console throws light onto the panel
// around it, and that halo is most of what makes it read as EMITTING rather than
// as a coloured dot painted on. Unlit ones get a recessed socket instead, so the
// two states differ in more than brightness.
// ---------------------------------------------------------------------------
Void led(ImDrawList* dl, const ImVec2& centre, Float32 radius, ImU32 colour, Bool lit)
{
    if(dl == nullptr || radius <= 0.0f)
        return;

    // The socket the lamp sits in, always drawn.
    dl->AddCircleFilled(centre, radius * 1.55f, IM_COL32(0, 0, 0, 110), 16);

    if(lit)
    {
        const ImU32 rgb = colour & 0x00FFFFFFu;
        dl->AddCircleFilled(centre, radius * 2.60f, rgb | (static_cast<ImU32>(26u) << IM_COL32_A_SHIFT), 16);
        dl->AddCircleFilled(centre, radius * 1.70f, rgb | (static_cast<ImU32>(52u) << IM_COL32_A_SHIFT), 16);
        dl->AddCircleFilled(centre, radius, colour, 16);
        // The hot spot, offset up-left as a domed lens catches the light.
        dl->AddCircleFilled(ImVec2(centre.x - radius * 0.28f, centre.y - radius * 0.28f),
                            radius * 0.36f, IM_COL32(255, 255, 255, 150), 12);
    }
    else
    {
        dl->AddCircleFilled(centre, radius, (colour & 0x00FFFFFFu)
                                            | (static_cast<ImU32>(60u) << IM_COL32_A_SHIFT), 16);
        dl->AddCircle(centre, radius, IM_COL32(255, 255, 255, 28), 16, 1.0f);
    }
}

// ---------------------------------------------------------------------------
// A screen recessed into the casing
//
// The two central views are displays set into the console, not areas of it, and
// on real gear that reads as a bezel: the panel casts a shadow onto the top and
// left inner edges of the cut-out and catches light along the bottom and right.
// That is the exact inverse of a raised key, which is what makes the two read as
// opposite mechanisms rather than as two rectangles.
//
// The inner shadow is a gradient rather than a line because a milled edge is not
// sharp - it falls off over a couple of millimetres, and at screen scale that is
// a few pixels.
// ---------------------------------------------------------------------------
Void screenInset(const ImVec2& pMin, const ImVec2& pMax, Float32 strength)
{
    if(strength <= 0.0f)
        return;

    const Float32 w = pMax.x - pMin.x;
    const Float32 h = pMax.y - pMin.y;
    if(w < 8.0f || h < 8.0f)
        return;

    const Float32 a  = (strength > 1.0f) ? 1.0f : strength;
    ImDrawList*   dl = ImGui::GetWindowDrawList();

    dl->PushClipRect(pMin, pMax, true);

    // Shadow falling in from the top and left edges.
    const Float32 fall = 10.0f * dpiScale();
    const ImU32 dark = IM_COL32(0, 0, 0, static_cast<Int32>(80 * a));
    const ImU32 nil  = IM_COL32(0, 0, 0, 0);

    dl->AddRectFilledMultiColor(pMin, ImVec2(pMax.x, pMin.y + fall), dark, dark, nil, nil);
    dl->AddRectFilledMultiColor(pMin, ImVec2(pMin.x + fall, pMax.y), dark, nil, nil, dark);

    // The bezel itself: one dark pixel top/left, one light pixel bottom/right.
    const ImU32 edgeDark = IM_COL32(0, 0, 0, static_cast<Int32>(160 * a));
    const ImU32 edgeLit  = IM_COL32(255, 255, 255, static_cast<Int32>(26 * a));

    dl->AddLine(ImVec2(pMin.x, pMin.y + 0.5f), ImVec2(pMax.x, pMin.y + 0.5f), edgeDark, 1.0f);
    dl->AddLine(ImVec2(pMin.x + 0.5f, pMin.y), ImVec2(pMin.x + 0.5f, pMax.y), edgeDark, 1.0f);
    dl->AddLine(ImVec2(pMin.x, pMax.y - 0.5f), ImVec2(pMax.x, pMax.y - 0.5f), edgeLit, 1.0f);
    dl->AddLine(ImVec2(pMax.x - 0.5f, pMin.y), ImVec2(pMax.x - 0.5f, pMax.y), edgeLit, 1.0f);

    dl->PopClipRect();
}

// A raised plate for custom-drawn chrome that is not an ImGui item - the HUD
// readouts over the map, the chips in the board view. Same key treatment the
// buttons get, so a label on the viewport belongs to the same machine.
Void plate(const ImVec2& pMin, const ImVec2& pMax, ImU32 fill, Float32 rounding)
{
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(pMin, pMax, fill, rounding);
    bevelRect(pMin, pMax, false, 0.85f);
    dl->AddRect(pMin, pMax, IM_COL32(0, 0, 0, 120), rounding, 0, 1.0f);
}

Void shadeLastItem(Float32 strength)
{
    if(!ImGui::IsItemVisible())
        return;
    bevelRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax(),
              ImGui::IsItemActive(), strength);
}

namespace {

// The base colour mixed toward the hue. Hovered and active go further, so a
// tinted button still brightens under the cursor rather than sitting flat.
ImVec4 mixToward(const ImVec4& base, ImU32 hue, Float32 k, Float32 lift)
{
    ImVec4 h = ImGui::ColorConvertU32ToFloat4(hue);

    // Rescale the hue to the base's own brightness FIRST. The semantic colours
    // are LED colours - picked to be read at a glance against a dark panel - and
    // mixing one in raw does not tint a button, it lights it up: 30% of a
    // 0xFFB02E amber over slate produced a solid tan block with the bevel and
    // the gloss no longer visible on it, which is a painted control, not a
    // moulded one.
    //
    // So change the HUE at (near) constant VALUE, and let `lift` add the small
    // deliberate amount of brightness rather than it arriving by accident.
    const Float32 lb = 0.2126f * base.x + 0.7152f * base.y + 0.0722f * base.z;
    const Float32 lh = 0.2126f * h.x    + 0.7152f * h.y    + 0.0722f * h.z;
    if(lh > 0.001f)
    {
        const Float32 s = (lb * lift) / lh;
        h.x *= s; h.y *= s; h.z *= s;
    }

    return ImVec4(base.x + (h.x - base.x) * k,
                  base.y + (h.y - base.y) * k,
                  base.z + (h.z - base.z) * k,
                  base.w);
}

ImU32 tintHue(Tint t)
{
    switch(t)
    {
    case Tint::TINT_GOOD:   return sem::GOOD;
    case Tint::TINT_WARN:   return sem::WARN;
    case Tint::TINT_BAD:    return sem::BAD;
    case Tint::TINT_ACCENT: return accent::CYAN;
    case Tint::TINT_NONE:
    default:                return 0u;
    }
}

} // namespace

Void pushTint(Tint t)
{
    if(t == Tint::TINT_NONE)
        return;

    const ImU32   hue = tintHue(t);
    const ImVec4* c   = ImGui::GetStyle().Colors;

    // The mix is strong (the hue is luminance-matched, so it costs no
    // brightness) and the lift is small. Hovered and active lift further, which
    // is where the "it responds" comes from - not from the tint.
    ImGui::PushStyleColor(ImGuiCol_Button,        mixToward(c[ImGuiCol_Button],        hue, 0.38f, 1.02f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, mixToward(c[ImGuiCol_ButtonHovered], hue, 0.48f, 1.10f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  mixToward(c[ImGuiCol_ButtonActive],  hue, 0.58f, 1.18f));
}

Void popTint(Tint t)
{
    if(t != Tint::TINT_NONE)
        ImGui::PopStyleColor(3);
}

Bool button(const Char* label, const ImVec2& size, Tint tint)
{
    pushTint(tint);
    const Bool clicked = ImGui::Button(label, size);
    popTint(tint);

    // AFTER the pop, so the bevel and gloss are drawn over the tinted fill
    // rather than being tinted themselves. The highlight on a moulded surface
    // is the colour of the light, not the colour of the plastic.
    shadeLastItem();
    return clicked;
}

// ---------------------------------------------------------------------------
// Checkbox
//
// Hand-rolled rather than ImGui::Checkbox because that widget sizes its box to
// the WHOLE frame height, and at this type scale that is a ~28 px square that
// reads as a tile rather than a control.
//
// A checkbox is a well, not a key: it is milled INTO the panel, so it takes the
// inverted bevel - shadow on the top edge, light on the bottom - which is the
// opposite of a button and is what makes the pair read as different mechanisms.
//
// Still a real ImGui item (an InvisibleButton over box + label), so hover,
// activation, keyboard nav, disabled state and SameLine all behave.
// ---------------------------------------------------------------------------
Bool checkbox(const Char* label, Bool* v)
{
    const ImGuiStyle& st = ImGui::GetStyle();
    const Float32 fh  = ImGui::GetFrameHeight();
    const Float32 box = ImGui::GetFontSize() * 0.86f;
    const Float32 dpi = dpiScale();

    // hide_text_after_double_hash: "##vis" is a box with no label and must not
    // reserve width for one.
    const ImVec2 tsz = ImGui::CalcTextSize(label, nullptr, true);
    const Float32 w  = box + ((tsz.x > 0.0f) ? st.ItemInnerSpacing.x + tsz.x : 0.0f);

    const ImVec2 p = ImGui::GetCursorScreenPos();

    ImGui::PushID(label);
    const Bool pressed = ImGui::InvisibleButton("##cb", ImVec2(w, fh));
    ImGui::PopID();

    if(pressed && v != nullptr) *v = !*v;

    const Bool on  = (v != nullptr) && *v;
    const Bool hov = ImGui::IsItemHovered();
    const Bool act = ImGui::IsItemActive();

    ImDrawList*  dl = ImGui::GetWindowDrawList();
    const ImVec2 b0(p.x, p.y + (fh - box) * 0.5f);
    const ImVec2 b1(b0.x + box, b0.y + box);

    // GetColorU32 rather than the raw style colour: it folds in style.Alpha, so
    // the box dims correctly inside BeginDisabled() the way a stock widget does.
    const ImU32 fill = ImGui::GetColorU32(act ? ImGuiCol_FrameBgActive
                                         : hov ? ImGuiCol_FrameBgHovered
                                               : ImGuiCol_FrameBg);
    dl->AddRectFilled(b0, b1, fill, st.FrameRounding);

    // Recessed: pressed == true gives shadow on top, light on the bottom.
    bevelRect(b0, b1, true, 1.0f);

    const Float32 bw = (st.FrameBorderSize > 0.0f) ? st.FrameBorderSize : hairline(dpi);
    const ImU32 edge = on  ? ImGui::GetColorU32(ImGuiCol_CheckMark)
                           : ImGui::GetColorU32(ImGuiCol_Border);
    dl->AddRect(b0, b1, edge, st.FrameRounding, 0, bw);

    if(on)
    {
        // Two strokes drawn inside the box with a margin, rather than ImGui's
        // RenderCheckMark - that one is thick enough at this size to fill the
        // square, which is what made the control read as a solid tile.
        const Float32 m = box * 0.24f;
        const ImVec2 a(b0.x + m,           b0.y + box * 0.52f);
        const ImVec2 b(b0.x + box * 0.42f, b1.y - m);
        const ImVec2 cpt(b1.x - m,         b0.y + m);
        const ImVec2 pts[3] = { a, b, cpt };
        dl->AddPolyline(pts, 3, ImGui::GetColorU32(ImGuiCol_CheckMark),
                        0, (box * 0.13f > 1.5f * dpi) ? box * 0.13f : 1.5f * dpi);
    }

    if(tsz.x > 0.0f)
    {
        // ImGui::FindRenderedTextEnd is imgui_internal only, and nothing else
        // here includes that header. The rule it implements is one line anyway:
        // everything from the first "##" onward is an ID, not a label.
        const Char* end = std::strstr(label, "##");
        if(end == nullptr) end = label + std::strlen(label);

        dl->AddText(ImVec2(b1.x + st.ItemInnerSpacing.x, p.y + (fh - tsz.y) * 0.5f),
                    ImGui::GetColorU32(ImGuiCol_Text), label, end);
    }

    return pressed;
}

// ---------------------------------------------------------------------------
// Combo
//
// ImGui fills a combo's arrow area with ImGuiCol_Button, so against a darker
// FrameBg the drop-down reads as a separate button welded onto the right of a
// field. Both reference tools draw the arrow INSIDE the field. Pushing the
// button colours to match the frame is the whole fix; the widget is otherwise
// stock, so it keeps its popup, keyboard handling and sizing.
// ---------------------------------------------------------------------------
Bool combo(const Char* label, Int32* current, const Char* const items[], Int32 count)
{
    const ImGuiStyle& st = ImGui::GetStyle();
    ImGui::PushStyleColor(ImGuiCol_Button,        st.Colors[ImGuiCol_FrameBg]);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, st.Colors[ImGuiCol_FrameBgHovered]);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  st.Colors[ImGuiCol_FrameBgActive]);
    const Bool changed = ImGui::Combo(label, current, items, count);
    ImGui::PopStyleColor(3);
    return changed;
}

// ---------------------------------------------------------------------------
// Segmented button
//
// One cell of a mutually-exclusive row. Unselected is transparent so the row
// reads as one strip rather than as N separate buttons; selected takes a plate
// fill and an accent underline along its bottom edge.
//
// The underline is the part that matters. Both reference tools mark the active
// item with a coloured edge rather than by flooding it, which is what keeps a
// selected cell from shouting louder than the content it is selecting.
// ---------------------------------------------------------------------------
Bool segmentedButton(const Char* label, Bool selected, const ImVec2& size, Mark mark)
{
    Bool hit;
    if(selected)
    {
        hit = ImGui::Button(label, size);
        shadeLastItem();

        const ImVec2 a = ImGui::GetItemRectMin();
        const ImVec2 b = ImGui::GetItemRectMax();
        const Float32  t = 2.0f * dpiScale();
        const ImU32  m = ImGui::GetColorU32(ImGuiCol_CheckMark);

        // Underline for a segment in a horizontal row, left bar for a row in a
        // vertical list - in both cases the mark runs along the edge the row is
        // stacked against, which is what makes a column of them scan as a list.
        if(mark == Mark::MARK_UNDERLINE)
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(a.x, b.y - t), b, m);
        else if(mark == Mark::MARK_LEFT_BAR)
            ImGui::GetWindowDrawList()->AddRectFilled(a, ImVec2(a.x + t, b.y), m);
    }
    else
    {
        // Unshaded when transparent - shading a transparent button paints a box
        // over nothing.
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
        hit = ImGui::Button(label, size);
        ImGui::PopStyleColor(2);
    }
    return hit;
}

// ---------------------------------------------------------------------------

Void setDpiScale(Float32 scale)
{
    geometryDpiScale = (scale > 0.0f) ? scale : 1.0f;
}

Float32 dpiScale()
{
    return geometryDpiScale;
}

// ---------------------------------------------------------------------------
// The user's zoom. See theme.hpp for why it is not folded into dpiScale().
// ---------------------------------------------------------------------------
namespace {
// Defaults to a step above 100%. The scale the UI was originally tuned at was
// measured on one machine, and "everything is a bit small" was the verdict from
// the person actually using it - so the default moves, and the control stays.
Float32 uiUserScale     = 1.20f;
Bool    uiUserScaleDirty = false;
}

Float32 userScale()
{
    return uiUserScale;
}

Void setUserScale(Float32 s)
{
    if(!(s > 0.0f))
        s = 1.0f;

    // Snapped before clamping so the ends of the range are reachable exactly.
    s = std::floor(s / USER_SCALE_STEP + 0.5f) * USER_SCALE_STEP;
    if(s < USER_SCALE_MIN) s = USER_SCALE_MIN;
    if(s > USER_SCALE_MAX) s = USER_SCALE_MAX;

    // Half a step, so float noise from the snap can never register as a change
    // and rebuild the style every frame.
    if(std::fabs(s - uiUserScale) < USER_SCALE_STEP * 0.5f)
        return;

    uiUserScale      = s;
    uiUserScaleDirty = true;
}

Bool consumeUserScaleChanged()
{
    const Bool d = uiUserScaleDirty;
    uiUserScaleDirty = false;
    return d;
}

} // namespace ui
