// The app's look: hand-authored palette on Unreal/IntelliJ geometry.
// StyleColorsDark() runs first only to guarantee every ImGuiCol_ has a value.
//
// Dear ImGui 1.92 (dynamic font atlas):
//  - PushFont(ImFont*, float base_size) - base_size is a PRE-SCALE value ImGui
//    multiplies by style.FontScaleMain * style.FontScaleDpi. Pass
//    `font->LegacySize`; every ImFont handed out below has a positive one.
//  - AddFontFromFileTTF() asserts on a missing file unless the config carries
//    ImFontFlags_NoLoadError; we set it AND pre-check with fopen, so a missing
//    Segoe UI degrades to the built-in font.
//  - The atlas is dynamic: no Build() call; a scale change re-bakes next frame.
//
// DPI OWNERSHIP (read before touching FontScaleDpi):
//  - loadFonts(dpi) rasterises at logical_px * dpi and remembers dpi in
//    fontBaseDpi. The ONLY place DPI is baked into type.
//  - style.FontScaleDpi belongs to the SHELL (main.cxx): on WM_DPICHANGED it keeps
//    the atlas and sets FontScaleDpi = new_dpi / <loadFonts' dpi>.
//  - applyStyle() therefore PRESERVES the FontScaleDpi it finds rather than
//    resetting it. FontScaleMain is ours and stays at 1.0f.
//  - ui::dpiScale() is GEOMETRY ONLY. Never applied to a font base size.

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "shared.hxx"
#include "theme.hxx"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace ui
{

  Fonts fonts;

  namespace
  {

    // The DPI the atlas was last rasterised at - loadFonts()'s argument. Font base
    // sizes (LegacySize) are in these units.
    Float32 fontBaseDpi = 1.0f;

    // Geometry scale. Set by setDpiScale(), read by dpiScale(). Never a font size.
    Float32 geometryDpiScale = 1.0f;

    // ---------------------------------------------------------------- font files

#ifndef UI_FONT_DIR
#define UI_FONT_DIR "C:\\Windows\\Fonts\\"
#endif

    constexpr const Char* SEGOE_REGULAR = UI_FONT_DIR "segoeui.ttf";
    constexpr const Char* SEGOE_SEMIBOLD = UI_FONT_DIR "seguisb.ttf";
    constexpr const Char* SEGOE_BOLD = UI_FONT_DIR "segoeuib.ttf";

    // Monospace, best first: Cascadia Mono (Windows 11 / Terminal), Consolas
    // (every Windows since Vista), Lucida Console as the floor.
    constexpr const Char* MONO_CASCADIA = UI_FONT_DIR "CascadiaMono.ttf";
    constexpr const Char* MONO_CONSOLAS = UI_FONT_DIR "consola.ttf";
    constexpr const Char* MONO_LUCIDA = UI_FONT_DIR "lucon.ttf";

    Bool fileReadable(const Char* path)
    {
        if(path == nullptr)
        {
            return false;
        }
        FILE* f = fopen(path, "rb");
        if(f == nullptr)
        {
            return false;
        }
        fclose(f);
        return true;
    }

    // Adds `path` at `sizePx`. Returns nullptr (without asserting) when the file
    // is missing or unreadable.
    ImFont* tryAddFile(ImFontAtlas* atlas, const Char* path, Float32 sizePx)
    {
        if(!fileReadable(path))
        {
            return nullptr;
        }

        ImFontConfig cfg;
        cfg.SizePixels = sizePx;
        cfg.OversampleH = 2;
        cfg.OversampleV = 1;
        cfg.PixelSnapH = false;
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
        {
            return f;
        }
#endif
#if !defined(IMGUI_DISABLE_DEFAULT_FONT)
        if(ImFont* f = atlas->AddFontDefault(&cfg))
        {
            return f;
        }
#endif
        return nullptr;
    }

    // First readable candidate wins, else the built-in font. Non-null as long as
    // ImGui has a default font compiled in; loadFonts() nets the rest.
    ImFont* loadRole(
        ImFontAtlas* atlas,
        Float32 sizePx,
        const Char* first,
        const Char* second,
        const Char* third
    )
    {
        if(ImFont* f = tryAddFile(atlas, first, sizePx))
        {
            return f;
        }
        if(ImFont* f = tryAddFile(atlas, second, sizePx))
        {
            return f;
        }
        if(ImFont* f = tryAddFile(atlas, third, sizePx))
        {
            return f;
        }
        return addBuiltinAt(atlas, sizePx);
    }

    // A border/separator thickness that survives ScaleAllSizes()' truncation.
    Float32 hairline(Float32 dpiScale)
    {
        const Float32 v = static_cast<Float32>(static_cast<Int32>((1.0f * dpiScale)));
        return v >= 1.0f ? v : 1.0f;
    }

  }

  // ---------------------------------------------------------------------------

  Void loadFonts(Float32 dpiScale)
  {
      if(!(dpiScale > 0.0f))
      {
          dpiScale = 1.0f;
      }

      fontBaseDpi = dpiScale;

      ImGuiIO& io = ImGui::GetIO();
      ImFontAtlas* atlas = io.Fonts;

      // Safe to re-run before the first frame and between frames. The atlas is
      // Locked inside NewFrame()..EndFrame() on backends without dynamic texture
      // support, and touching it then asserts.
      if(!atlas->Locked)
      {
          atlas->Clear();
      }

      // The type scale from theme.hxx, multiplied by the LOAD-TIME DPI exactly
      // once. These end up in ImFont::LegacySize - the base sizes callers push.
      const Float32 szSmall = size::SMALL * dpiScale;
      const Float32 szBody = size::BODY  * dpiScale;
      const Float32 szTitle = size::TITLE * dpiScale;
      const Float32 szStat = size::STAT  * dpiScale;
      const Float32 szBig = size::BIG   * dpiScale;
      const Float32 szCode = size::CODE  * dpiScale;

      // Regular for reading sizes, semibold/bold for headlines, so the hierarchy
      // reads by weight as well as size.
      fonts.small = loadRole(atlas, szSmall, SEGOE_REGULAR,  SEGOE_SEMIBOLD, nullptr);
      fonts.body = loadRole(atlas, szBody,  SEGOE_REGULAR,  SEGOE_SEMIBOLD, nullptr);
      fonts.title = loadRole(atlas, szTitle, SEGOE_SEMIBOLD, SEGOE_BOLD,     SEGOE_REGULAR);
      fonts.stat = loadRole(atlas, szStat,  SEGOE_SEMIBOLD, SEGOE_BOLD,     SEGOE_REGULAR);
      fonts.big = loadRole(atlas, szBig,   SEGOE_BOLD,     SEGOE_SEMIBOLD, SEGOE_REGULAR);
      fonts.mono = loadRole(atlas, szCode,  MONO_CASCADIA,  MONO_CONSOLAS,  MONO_LUCIDA);

      // Safety net: nothing here may ever be null - pushing a null ImFont* is an
      // outright crash.
      ImFont* any = fonts.body;
      if(any == nullptr)
      {
          ImFont* const candidates[] = { fonts.small, fonts.title, fonts.stat, fonts.big, fonts.mono };
          for(ImFont* c : candidates)
          {
              if(c != nullptr)
              {
                  any = c;
                  break;
              }
          }
      }
      if(any == nullptr)
      {
          any = addBuiltinAt(atlas, szBody);
      }
      if(any == nullptr && atlas->Fonts.Size > 0)
      {
          any = atlas->Fonts[0];
      }

      IM_ASSERT(any != nullptr && "No font could be loaded (default font compiled out?)");

      if(fonts.small == nullptr)
      {
          fonts.small = any;
      }
      if(fonts.body  == nullptr)
      {
          fonts.body = any;
      }
      if(fonts.title == nullptr)
      {
          fonts.title = any;
      }
      if(fonts.stat  == nullptr)
      {
          fonts.stat = any;
      }
      if(fonts.big   == nullptr)
      {
          fonts.big = any;
      }
      if(fonts.mono  == nullptr)
      {
          fonts.mono = any;
      }

      // A zero/negative LegacySize silently degrades PushFont to "keep the current
      // size", and a shared fallback font carries the size of whichever role first
      // loaded it - so re-stamp anything non-positive.
      const Array<ImFont*, 6> roles = { fonts.small, fonts.body, fonts.title,
                                        fonts.stat,  fonts.big,  fonts.mono };
      const Array<Float32, 6> sizes = { szSmall,     szBody,     szTitle,
                                        szStat,      szBig,      szCode };
      for(Int32 i = 0; i < 6; ++i)
      {
          if(roles[i] != nullptr && !(roles[i]->LegacySize > 0.0f))
          {
              roles[i]->LegacySize = sizes[i];
          }
      }

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
      {
          dpiScale = 1.0f;
      }

      ImGuiStyle& style = ImGui::GetStyle();

      // The SHELL's, captured before the reset below and handed straight back - or
      // a second applyStyle() after a DPI change silently reverts all text to the
      // pre-change size. Safe in either call order.
      const Float32 shellFontScaleDpi = style.FontScaleDpi;

      // Reset to defaults so applyStyle() is idempotent: ScaleAllSizes()
      // accumulates into style._MainScale, so repeated calls would compound.
      style = ImGuiStyle();

      ImGui::StyleColorsDark();

      // Default (un-pushed) text size follows the body font loadFonts() actually
      // registered, so the two cannot drift. Pre-scale value, and NOT touched by
      // ScaleAllSizes() - dpi is already baked into LegacySize.
      style.FontSizeBase = (fonts.body != nullptr && fonts.body->LegacySize > 0.0f)
                         ? fonts.body->LegacySize
                         : size::BODY * dpiScale;

      // FontScaleMain is an app/user zoom knob we do not use.
      style.FontScaleMain = 1.0f;
      style.FontScaleDpi = (shellFontScaleDpi > 0.0f) ? shellFontScaleDpi : 1.0f;

      // ------------------------------------------------------------- palette
      // GRUVBOX DARK, on the same skeuomorphic depth the chrome always had. The
      // palette changed; the physics did not. A field is still milled INTO the
      // casing and sits a step darker; a key is still pushed OUT of it and sits
      // a step lighter; bevelRect() still puts a pixel of light along the top
      // edge and a pixel of shadow along the bottom. Gruvbox's background ramp -
      // bg0_h, bg0, bg0_s, bg1, bg2, bg3, bg4 - IS that depth scale, which is
      // why it fits a 2010s dark utility panel better than a flat theme would.
      //
      // The editor's syntax colours (syn::gruv) were already this palette. This
      // brings the chrome around them to the same set, so a file no longer sits
      // in a differently-coloured window.
      //
      // NO ACCENT ON ANY EDGE. Borders, separators, the tab overline and the
      // selected-segment mark are all neutral now. The one interface accent -
      // accent::CYAN, which is gruvbox aqua despite its name - is reserved for
      // marks: a check, a caret, a drop target. Colour on an edge is what made
      // the old chrome busy, and an edge that changes colour on hover is an
      // edge asking to be looked at.
      auto hex = [](Int32 r, Int32 g, Int32 b, Float32 a = 1.0f) {
          return ImVec4(
              static_cast<Float32>(r) / 255.0f,
              static_cast<Float32>(g) / 255.0f,
              static_cast<Float32>(b) / 255.0f,
              a
          );
      };

      const ImVec4 bg0h = hex(0x1D, 0x20, 0x21);   // the deepest well
      const ImVec4 bg0 = hex(0x28, 0x28, 0x28);   // the casing
      const ImVec4 bg0s = hex(0x32, 0x30, 0x2F);   // a panel on the casing
      const ImVec4 bg1 = hex(0x3C, 0x38, 0x36);   // a key
      const ImVec4 bg2 = hex(0x50, 0x49, 0x45);   // a key under the pointer
      const ImVec4 bg3 = hex(0x66, 0x5C, 0x54);   // every hairline edge
      const ImVec4 bg4 = hex(0x7C, 0x6F, 0x64);
      const ImVec4 fg1 = hex(0xEB, 0xDB, 0xB2);   // text
      const ImVec4 fg2 = hex(0xD5, 0xC4, 0xA1);
      const ImVec4 fg3 = hex(0xBD, 0xAE, 0x93);
      const ImVec4 fg4 = hex(0xA8, 0x99, 0x84);   // muted text, neutral grabs
      const ImVec4 aqua = ImGui::ColorConvertU32ToFloat4(accent::CYAN_HI);
      const ImVec4 blue = hex(0x83, 0xA5, 0x98);

      ImVec4* c = style.Colors;

      // ---- the casing ----
      c[ImGuiCol_WindowBg] = bg0;
      c[ImGuiCol_ChildBg] = bg0;      // same as the window: one casing, not panels on a casing
      c[ImGuiCol_PopupBg] = bg1;
      c[ImGuiCol_MenuBarBg] = bg0;
      c[ImGuiCol_TitleBg] = bg0h;
      c[ImGuiCol_TitleBgActive] = bg0;
      c[ImGuiCol_TitleBgCollapsed] = bg0h;
      c[ImGuiCol_TableRowBg] = ImVec4(0, 0, 0, 0);
      c[ImGuiCol_TableRowBgAlt] = ImVec4(fg1.x, fg1.y, fg1.z, 0.03f);

      // ---- wells: a field is milled INTO the casing, so it is darker ----
      c[ImGuiCol_FrameBg] = bg0h;
      c[ImGuiCol_FrameBgHovered] = bg0;
      c[ImGuiCol_FrameBgActive] = bg0h;

      // ---- keys: pushed OUT of the casing, so they are lighter ----
      c[ImGuiCol_Button] = bg1;
      c[ImGuiCol_ButtonHovered] = bg2;
      c[ImGuiCol_ButtonActive] = bg0s;   // sinks when pressed

      c[ImGuiCol_Header] = bg1;
      c[ImGuiCol_HeaderHovered] = bg2;
      c[ImGuiCol_HeaderActive] = bg3;

      c[ImGuiCol_Tab] = bg0;
      c[ImGuiCol_TabHovered] = bg2;
      c[ImGuiCol_TabSelected] = bg1;
      c[ImGuiCol_TabDimmed] = bg0;
      c[ImGuiCol_TabDimmedSelected]= bg1;

      // The selected tab is the one with the key fill. It does not also get a
      // coloured line drawn on it.
      c[ImGuiCol_TabSelectedOverline] = ImVec4(0, 0, 0, 0);

      // ---- edges: one hairline tone, and it does not react ----
      c[ImGuiCol_Border] = bg3;
      c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
      c[ImGuiCol_Separator] = bg3;
      c[ImGuiCol_SeparatorHovered] = bg4;
      c[ImGuiCol_SeparatorActive] = fg4;

      // ---- controls ----
      c[ImGuiCol_CheckMark] = aqua;
      c[ImGuiCol_SliderGrab] = fg4;
      c[ImGuiCol_SliderGrabActive] = fg3;

      c[ImGuiCol_ScrollbarBg] = bg0h;
      c[ImGuiCol_ScrollbarGrab] = bg2;
      c[ImGuiCol_ScrollbarGrabHovered] = bg3;
      c[ImGuiCol_ScrollbarGrabActive] = bg4;
      c[ImGuiCol_ResizeGrip] = ImVec4(fg1.x, fg1.y, fg1.z, 0.08f);
      c[ImGuiCol_ResizeGripHovered] = ImVec4(fg1.x, fg1.y, fg1.z, 0.18f);
      c[ImGuiCol_ResizeGripActive] = ImVec4(fg1.x, fg1.y, fg1.z, 0.30f);

      c[ImGuiCol_TableHeaderBg] = bg1;
      c[ImGuiCol_TableBorderStrong] = bg3;
      c[ImGuiCol_TableBorderLight] = ImVec4(0, 0, 0, 0);   // rows are separated by their own alternation
      c[ImGuiCol_TextSelectedBg] = ImVec4(bg2.x, bg2.y, bg2.z, 0.90f);
      c[ImGuiCol_InputTextCursor] = fg1;

      // ---- the rest of the palette -----------------------------------------
      c[ImGuiCol_PlotLines] = fg4;
      c[ImGuiCol_PlotLinesHovered] = fg2;
      c[ImGuiCol_PlotHistogram] = bg4;
      c[ImGuiCol_PlotHistogramHovered] = fg4;

      c[ImGuiCol_NavCursor] = fg4;
      c[ImGuiCol_DragDropTarget] = aqua;
      c[ImGuiCol_TextLink] = blue;

      c[ImGuiCol_Text] = fg1;
      c[ImGuiCol_TextDisabled] = fg4;

      // ------------------------------------------------------------ geometry
      // SHARP. Every corner in the program, at zero, and set in ONE place so there
      // is no second answer - a widget that rounds itself stands out immediately.
      style.FrameRounding = 0.0f;
      style.GrabRounding = 0.0f;
      style.TabRounding = 0.0f;
      style.ChildRounding = 0.0f;
      style.WindowRounding = 0.0f;
      style.PopupRounding = 0.0f;
      style.ScrollbarRounding = 0.0f;

      // Widgets are outlined; containers are NOT - panels run flush into one
      // another, or you get boxes ringed inside other ringed boxes.
      style.FrameBorderSize = 0.0f;    // the bevel is the edge; no outline on top of it
      style.WindowBorderSize = 0.0f;
      style.ChildBorderSize = 0.0f;

      // DENSE, on purpose - every value is smaller than a general-purpose UI would
      // use, with the outlines and bevels doing the separating.
      // ONE SCALE, EVEN NUMBERS. These were 5/3, 6/3, 6/5, 14 - each chosen on
      // its own, none a multiple of any other, so a row of controls, a table,
      // and a panel edge all sat at slightly different distances and the whole
      // window read as unevenly spaced. Everything here is now 4 or 8 or a
      // multiple, which is what makes gaps line up across views without each
      // view having to be tuned by hand.
      style.ItemSpacing = ImVec2(8.0f, 6.0f);
      style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
      style.FramePadding = ImVec2(8.0f, 4.0f);
      // 4/2, not 6/3: the one pair that was off the scale, and a table row is
      // its text plus twice the y - at 3 the sidebar's sixteen rows ran 44 px
      // past what a 1000 px window holds.
      style.CellPadding = ImVec2(4.0f, 2.0f);
      style.WindowPadding = ImVec2(8.0f, 8.0f);
      style.IndentSpacing = 16.0f;

      // Sub-headings flush on the content edge, with the frame's vertical
      // padding above and below: ImGui's default (20,3) was a fourth left edge
      // in the sidebar and a pad off the 4/8 scale, and the row gap (6) put
      // the sidebar's three headings twelve pixels over what 1000 px holds.
      style.SeparatorTextPadding = ImVec2(0.0f, style.FramePadding.y);
      style.SeparatorTextAlign = ImVec2(0.0f, 0.5f);

      // A thin scrollbar; the stock 14px bar is a touch-sized affordance.
      style.ScrollbarSize = 10.0f;
      style.GrabMinSize = 10.0f;

      // The selected tab is marked only by its overline, so it has to be thick
      // enough to see.
      style.TabBarOverlineSize = 3.0f;
      style.TabBorderSize = 0.0f;
      style.TabBarBorderSize = 1.0f;

      // Convert the logical sizes into physical px, exactly once.
      style.ScaleAllSizes(dpiScale);

      // ScaleAllSizes() truncates, which can zero out 1px lines at dpi < 1, and
      // ImGui requires SeparatorSize >= 1.0f.
      style.SeparatorSize = hairline(dpiScale);
      style.SeparatorTextBorderSize = hairline(dpiScale);

      // Same reason: an outline rounding to zero at some DPI silently undoes the
      // whole look on that machine.
      style.FrameBorderSize = 0.0f;

      // Popups are the one bordered container: they float over arbitrary content
      // and need an edge, or a modal reads as text pasted onto the page.
      style.PopupBorderSize = hairline(dpiScale);
  }

  // ---------------------------------------------------------------------------

  // ---------------------------------------------------------------------------

  Void shadeRect(const ImVec2& pMin, const ImVec2& pMax, Float32 strength)
  {
      bevelRect(pMin, pMax, false, strength);
  }

  // The bevel: 1px of light along the TOP edge, 1px of shadow along the BOTTOM,
  // both inverted when pressed. Drawn OVER an already-submitted item, so
  // ImGui::Button stays a real ImGui::Button. One pixel means ONE PHYSICAL PIXEL -
  // the one measurement in the file that deliberately ignores the DPI scale.
  Void bevelRect(const ImVec2& pMin, const ImVec2& pMax, Bool pressed, Float32 strength)
  {
      if(strength <= 0.0f)
      {
          return;
      }

      const Float32 w = pMax.x - pMin.x;
      const Float32 h = pMax.y - pMin.y;
      if(w < 4.0f || h < 4.0f)
      {
          return;
      }

      const Float32 a = (strength > 1.0f) ? 1.0f : strength;
      ImDrawList* dl = ImGui::GetWindowDrawList();

      // GLOSS: a soft sheen over the top half of a raised control, kept low so it
      // reads as a finish and not a gradient. Clipped rather than rounded because
      // AddRectFilledMultiColor cannot round its corners.
      if(!pressed)
      {
          dl->PushClipRect(pMin, pMax, true);
          const Float32 mid = pMin.y + h * 0.52f;
          const ImU32 top = IM_COL32(255, 255, 255, static_cast<Int32>(15 * a));
          const ImU32 nil = IM_COL32(255, 255, 255, 0);
          dl->AddRectFilledMultiColor(pMin, ImVec2(pMax.x, mid), top, top, nil, nil);
          dl->PopClipRect();
      }

      const ImU32 lit = IM_COL32(255, 255, 255, static_cast<Int32>(42 * a));
      const ImU32 shade = IM_COL32(0, 0, 0, static_cast<Int32>(105 * a));

      const ImU32 top = pressed ? shade : lit;
      const ImU32 bottom = pressed ? lit   : shade;

      // Inset by the frame rounding so the highlight does not stick out past the
      // rounded corners as a pair of stray dots.
      const Float32 r = ImGui::GetStyle().FrameRounding;
      dl->AddLine(ImVec2(pMin.x + r, pMin.y + 0.5f), ImVec2(pMax.x - r, pMin.y + 0.5f), top, 1.0f);
      dl->AddLine(
          ImVec2(pMin.x + r, pMax.y - 0.5f),
          ImVec2(pMax.x - r, pMax.y - 0.5f),
          bottom,
          1.0f
      );

      // A pressed key also shadows itself along its top inner edge.
      if(pressed)
      {
          dl->AddLine(
              ImVec2(pMin.x + r, pMin.y + 1.5f),
              ImVec2(pMax.x - r, pMin.y + 1.5f),
              IM_COL32(0, 0, 0, static_cast<Int32>(55 * a)),
              1.0f
          );
      }
  }

  // An indicator LED. Not a filled circle: a lit one throws a halo onto the panel,
  // which is most of what makes it read as EMITTING. Unlit ones get a recessed
  // socket, so the two states differ in more than brightness.
  Void led(ImDrawList* dl, const ImVec2& center, Float32 radius, ImU32 color, Bool lit)
  {
      if(dl == nullptr || radius <= 0.0f)
      {
          return;
      }

      // The socket the lamp sits in, always drawn.
      dl->AddCircleFilled(center, radius * 1.55f, IM_COL32(0, 0, 0, 110), 16);

      if(lit)
      {
          const ImU32 rgb = color & 0x00FFFFFFu;
          dl->AddCircleFilled(
              center,
              radius * 2.60f,
              rgb | (static_cast<ImU32>(26u) << IM_COL32_A_SHIFT),
              16
          );
          dl->AddCircleFilled(
              center,
              radius * 1.70f,
              rgb | (static_cast<ImU32>(52u) << IM_COL32_A_SHIFT),
              16
          );
          dl->AddCircleFilled(center, radius, color, 16);
          // The hot spot, offset up-left as a domed lens catches the light.
          dl->AddCircleFilled(
              ImVec2(center.x - radius * 0.28f, center.y - radius * 0.28f),
              radius * 0.36f,
              IM_COL32(255, 255, 255, 150),
              12
          );
      }
      else
      {
          dl->AddCircleFilled(
              center,
              radius,
              (color & 0x00FFFFFFu) | (static_cast<ImU32>(60u) << IM_COL32_A_SHIFT),
              16
          );
          dl->AddCircle(center, radius, IM_COL32(255, 255, 255, 28), 16, 1.0f);
      }
  }

  // A screen recessed into the casing: shadow on the top and left inner edges,
  // light along the bottom and right - the exact inverse of a raised key. The
  // inner shadow is a gradient because a milled edge is not sharp.
  Void screenInset(const ImVec2& pMin, const ImVec2& pMax, Float32 strength)
  {
      if(strength <= 0.0f)
      {
          return;
      }

      const Float32 w = pMax.x - pMin.x;
      const Float32 h = pMax.y - pMin.y;
      if(w < 8.0f || h < 8.0f)
      {
          return;
      }

      const Float32 a = (strength > 1.0f) ? 1.0f : strength;
      ImDrawList*   dl = ImGui::GetWindowDrawList();

      dl->PushClipRect(pMin, pMax, true);

      // Shadow falling in from the top and left edges.
      const Float32 fall = 10.0f * dpiScale();
      const ImU32 dark = IM_COL32(0, 0, 0, static_cast<Int32>(80 * a));
      const ImU32 nil = IM_COL32(0, 0, 0, 0);

      dl->AddRectFilledMultiColor(pMin, ImVec2(pMax.x, pMin.y + fall), dark, dark, nil, nil);
      dl->AddRectFilledMultiColor(pMin, ImVec2(pMin.x + fall, pMax.y), dark, nil, nil, dark);

      // The bezel itself: one dark pixel top/left, one light pixel bottom/right.
      const ImU32 edgeDark = IM_COL32(0, 0, 0, static_cast<Int32>(160 * a));
      const ImU32 edgeLit = IM_COL32(255, 255, 255, static_cast<Int32>(26 * a));

      dl->AddLine(ImVec2(pMin.x, pMin.y + 0.5f), ImVec2(pMax.x, pMin.y + 0.5f), edgeDark, 1.0f);
      dl->AddLine(ImVec2(pMin.x + 0.5f, pMin.y), ImVec2(pMin.x + 0.5f, pMax.y), edgeDark, 1.0f);
      dl->AddLine(ImVec2(pMin.x, pMax.y - 0.5f), ImVec2(pMax.x, pMax.y - 0.5f), edgeLit, 1.0f);
      dl->AddLine(ImVec2(pMax.x - 0.5f, pMin.y), ImVec2(pMax.x - 0.5f, pMax.y), edgeLit, 1.0f);

      dl->PopClipRect();
  }

  // A raised plate for custom-drawn chrome that is not an ImGui item - the HUD
  // readouts over the map, the chips in the board view.
  Void plate(const ImVec2& pMin, const ImVec2& pMax, ImU32 fill, Float32 rounding)
  {
      ImDrawList* dl = ImGui::GetWindowDrawList();
      // Fill and bevel are the whole treatment - no outline on top of the edge.
      dl->AddRectFilled(pMin, pMax, fill, rounding);
      bevelRect(pMin, pMax, false, 0.85f);
  }

  Void shadeLastItem(Float32 strength)
  {
      if(!ImGui::IsItemVisible())
      {
          return;
      }
      bevelRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), ImGui::IsItemActive(), strength);
  }

  namespace
  {

    // The base color mixed toward the hue. Hovered and active go further, so a
    // tinted button still brightens under the cursor rather than sitting flat.
    ImVec4 mixToward(const ImVec4& base, ImU32 hue, Float32 k, Float32 lift)
    {
        ImVec4 h = ImGui::ColorConvertU32ToFloat4(hue);

        // Rescale the hue to the base's brightness FIRST: mixing an LED color in
        // raw lights the button up rather than tinting it - 30% of 0xFFB02E amber
        // over slate gave a solid tan block with the bevel and gloss invisible. So
        // change HUE at near-constant VALUE and let `lift` add the brightness.
        const Float32 lb = 0.2126f * base.x + 0.7152f * base.y + 0.0722f * base.z;
        const Float32 lh = 0.2126f * h.x    + 0.7152f * h.y    + 0.0722f * h.z;
        if(lh > 0.001f)
        {
            const Float32 s = (lb * lift) / lh;
            h.x *= s;
            h.y *= s;
            h.z *= s;
        }

        return ImVec4(
            base.x + (h.x - base.x) * k,
            base.y + (h.y - base.y) * k,
            base.z + (h.z - base.z) * k,
            base.w
        );
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

  }

  Void pushTint(Tint t)
  {
      if(t == Tint::TINT_NONE)
      {
          return;
      }

      const ImU32   hue = tintHue(t);
      const ImVec4* c = ImGui::GetStyle().Colors;

      // Strong mix (the hue is luminance-matched, so it costs no brightness), small
      // lift. Hovered and active lift further - that is where the response reads
      // from, not from the tint.
      ImGui::PushStyleColor(ImGuiCol_Button,        mixToward(
          c[ImGuiCol_Button],
          hue,
          0.38f,
          1.02f
      ));
      ImGui::PushStyleColor(
          ImGuiCol_ButtonHovered,
          mixToward(c[ImGuiCol_ButtonHovered], hue, 0.48f, 1.10f)
      );
      ImGui::PushStyleColor(
          ImGuiCol_ButtonActive,
          mixToward(c[ImGuiCol_ButtonActive], hue, 0.58f, 1.18f)
      );
  }

  Void popTint(Tint t)
  {
      if(t != Tint::TINT_NONE)
      {
          ImGui::PopStyleColor(3);
      }
  }

  Bool button(const Char* label, const ImVec2& size, Tint tint)
  {
      pushTint(tint);
      const Bool clicked = ImGui::Button(label, size);
      popTint(tint);

      // AFTER the pop, so the bevel and gloss are drawn over the tinted fill
      // rather than being tinted themselves.
      shadeLastItem();
      return clicked;
  }

  // Hand-rolled rather than ImGui::Checkbox, which sizes its box to the WHOLE
  // frame height - at this type scale a ~28px square that reads as a tile. A
  // checkbox is a well, not a key, so it takes the INVERTED bevel. Still a real
  // ImGui item (an InvisibleButton over box + label), so hover, activation,
  // keyboard nav, disabled and SameLine all behave.
  Bool checkbox(const Char* label, Bool* v)
  {
      const ImGuiStyle& st = ImGui::GetStyle();
      const Float32 fh = ImGui::GetFrameHeight();
      const Float32 box = ImGui::GetFontSize() * 0.86f;
      const Float32 dpi = dpiScale();

      // hide_text_after_double_hash: "##vis" is a box with no label and must not
      // reserve width for one.
      const ImVec2 tsz = ImGui::CalcTextSize(label, nullptr, true);
      const Float32 w = box + ((tsz.x > 0.0f) ? st.ItemInnerSpacing.x + tsz.x : 0.0f);

      const ImVec2 p = ImGui::GetCursorScreenPos();

      // Report a text baseline, the way ImGui's own checkbox does - an
      // InvisibleButton reports NONE, so a following SameLine() item drew at the
      // TOP of the row. This is the only public way to set it. It is a max(), and
      // it does not move the checkbox: ItemSize only shifts an item reporting a
      // baseline of its own, and the box is drawn from p, captured above.
      ImGui::AlignTextToFramePadding();

      ImGui::PushID(label);
      const Bool pressed = ImGui::InvisibleButton("##cb", ImVec2(w, fh));
      ImGui::PopID();

      if(pressed && v != nullptr)
      {
          *v = !*v;
      }

      const Bool on = (v != nullptr) && *v;
      const Bool hov = ImGui::IsItemHovered();
      const Bool act = ImGui::IsItemActive();

      ImDrawList*  dl = ImGui::GetWindowDrawList();
      const ImVec2 b0(p.x, p.y + (fh - box) * 0.5f);
      const ImVec2 b1(b0.x + box, b0.y + box);

      // GetColorU32 rather than the raw style color: it folds in style.Alpha, so
      // the box dims correctly inside BeginDisabled() the way a stock widget does.
      const ImU32 fill = ImGui::GetColorU32(
          act ? ImGuiCol_FrameBgActive : hov ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg
      );
      dl->AddRectFilled(b0, b1, fill, 0.0f);

      // Recessed: pressed == true gives shadow on top, light on the bottom.
      bevelRect(b0, b1, true, 1.0f);

      // The neutral hairline in both states: the check strokes below carry the
      // on-state, and a coloured outline is the one edge the palette forbids.
      const Float32 bw = (st.FrameBorderSize > 0.0f) ? st.FrameBorderSize : hairline(dpi);
      dl->AddRect(b0, b1, ImGui::GetColorU32(ImGuiCol_Border), 0.0f, 0, bw);

      if(on)
      {
          // Two strokes with a margin, rather than ImGui's RenderCheckMark, which
          // is thick enough at this size to fill the square.
          const Float32 m = box * 0.24f;
          const ImVec2 a(b0.x + m,           b0.y + box * 0.52f);
          const ImVec2 b(b0.x + box * 0.42f, b1.y - m);
          const ImVec2 cpt(b1.x - m,         b0.y + m);
          const ImVec2 pts[3] = { a, b, cpt };
          dl->AddPolyline(
              pts,
              3,
              ImGui::GetColorU32(ImGuiCol_CheckMark),
              0,
              (box * 0.13f > 1.5f * dpi) ? box * 0.13f : 1.5f * dpi
          );
      }

      if(tsz.x > 0.0f)
      {
          // ImGui::FindRenderedTextEnd is imgui_internal only. Its rule: everything
          // from the first "##" onward is an ID, not a label.
          const Char* end = std::strstr(label, "##");
          if(end == nullptr)
          {
              end = label + std::strlen(label);
          }

          dl->AddText(
              ImVec2(b1.x + st.ItemInnerSpacing.x, p.y + (fh - tsz.y) * 0.5f),
              ImGui::GetColorU32(ImGuiCol_Text),
              label,
              end
          );
      }

      return pressed;
  }

  // ImGui fills a combo's arrow area with ImGuiCol_Button, so against a darker
  // FrameBg the drop-down reads as a button welded to the field. Pushing the
  // button colors to match the frame is the whole fix; otherwise stock.
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

  Float32 buttonWidth(const ImVec2& size, Float32 autoW)
  {
      if(size.x > 0.0f)
      {
          return size.x;
      }
      if(size.x < 0.0f)
      {
          // CalcItemSize's floor, so a fill-right button in no room still exists.
          return std::max(4.0f, ImGui::GetContentRegionAvail().x + size.x);
      }
      return autoW;
  }

  Float32 labelAlign(Float32 labelX, Float32 w, Float32 textW)
  {
      const Float32 pad = ImGui::GetStyle().FramePadding.x;
      const Float32 inner = w - pad * 2.0f - textW;
      return (inner > 0.0f) ? std::clamp((labelX - pad) / inner, 0.0f, 1.0f) : 0.5f;
  }

  // One cell of a mutually-exclusive row. Unselected is transparent so the row
  // reads as one strip; selected takes the plate fill and bevel, nothing more -
  // a bar along one edge was a third treatment on a button and read as a tab
  // underline.
  Bool segmentedButton(const Char* label, Bool selected, const ImVec2& size, Float32 labelX)
  {
      // A cell in a strip is left-aligned at a FIXED inset when asked, so a
      // strip that wraps to two rows has its labels in one column; a label
      // centered per cell lands somewhere else in every cell.
      const Bool aligned = (labelX >= 0.0f);
      if(aligned)
      {
          const Float32 textW = ImGui::CalcTextSize(label, nullptr, true).x;
          const Float32 w = buttonWidth(size, textW + ImGui::GetStyle().FramePadding.x * 2.0f);
          ImGui::PushStyleVar(
              ImGuiStyleVar_ButtonTextAlign,
              ImVec2(labelAlign(labelX, w, textW), 0.5f)
          );
      }

      Bool hit;
      if(selected)
      {
          hit = ImGui::Button(label, size);
          shadeLastItem();
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

      if(aligned)
      {
          ImGui::PopStyleVar();
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

  Float32 fontScale()
  {
      if(fontBaseDpi <= 0.0f)
      {
          return 1.0f;
      }
      return geometryDpiScale / fontBaseDpi;
  }

  // The user's zoom. See theme.hxx for why it is not folded into dpiScale().
  namespace
  {
    // Defaults a step above 100%: the scale the UI was tuned at read as too small
    // in use.
    Float32 uiUserScale = 1.20f;
    Bool    uiUserScaleDirty = false;
  }

  Float32 userScale()
  {
      return uiUserScale;
  }

  Void setUserScale(Float32 s)
  {
      if(!(s > 0.0f))
      {
          s = 1.0f;
      }

      // Snapped before clamping so the ends of the range are reachable exactly.
      s = std::floor(s / USER_SCALE_STEP + 0.5f) * USER_SCALE_STEP;
      if(s < USER_SCALE_MIN)
      {
          s = USER_SCALE_MIN;
      }
      if(s > USER_SCALE_MAX)
      {
          s = USER_SCALE_MAX;
      }

      // Half a step, so float noise from the snap cannot rebuild the style every
      // frame.
      if(std::fabs(s - uiUserScale) < USER_SCALE_STEP * 0.5f)
      {
          return;
      }

      uiUserScale = s;
      uiUserScaleDirty = true;
  }

  Bool consumeUserScaleChanged()
  {
      const Bool d = uiUserScaleDirty;
      uiUserScaleDirty = false;
      return d;
  }

}
