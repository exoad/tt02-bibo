// See icons.hxx.
//
// Decoding goes through WIC, which ships with Windows. That is the whole reason
// this file has no third-party image library in it: the app is already Win32 +
// D3D11, so the platform's own PNG decoder costs nothing and adds no dependency
// to build, vendor or license.
//
// All the icons land in ONE atlas texture. Sixteen separate 16x16 textures would
// also work and would be simpler, but each one is a draw-call boundary in
// ImGui's draw list, and the point of a toolbar is that there are a lot of them.

#include "icons.hxx"

#include "theme.hxx"

#include <d3d11.h>
#include <wincodec.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#pragma comment(lib, "windowscodecs.lib")

namespace ui
{
  namespace
  {

    // Source art size. Not a preference - it is the size these were drawn at.
    constexpr Int32 SRC = 16;

    // Atlas layout: one row per icon is wasteful in height but trivial in memory at
    // this count, and keeps the UV maths to a single axis.
    constexpr Int32 COLS = 8;

    struct Entry
    {
        Icon        id;
        const Char* file;   // basename in assets/icons, without .png
    };

    // The names are Fugue's own, unaltered, so a reader can find the source art.
    constexpr Entry FILES[] = {
        { Icon::ICON_RADAR,           "radar" },
        { Icon::ICON_PROCESSOR,       "processor" },

        { Icon::ICON_SYSTEM,          "system-monitor" },
        { Icon::ICON_SENSORS,         "application-wave" },
        { Icon::ICON_VEHICLE,         "car" },
        { Icon::ICON_FIRMWARE,        "memory" },
        { Icon::ICON_CONSOLE,         "terminal" },

        { Icon::ICON_PLUG_CONNECT,    "plug-connect" },
        { Icon::ICON_PLUG_DISCONNECT, "plug-disconnect" },
        { Icon::ICON_REBOOT,          "arrow-circle-135-left" },
        { Icon::ICON_BACKUP,          "drive-download" },
        { Icon::ICON_RESET_VIEW,      "target" },
        { Icon::ICON_BUILD,           "hammer" },
        { Icon::ICON_FLASH,           "burn-small" },
        { Icon::ICON_CLEAR,           "broom" },
        { Icon::ICON_REFRESH,         "arrow-circle-135-left" },
        { Icon::ICON_MOTOR_STOP,      "control-stop" },
        { Icon::ICON_MOTOR_RUN,       "control" },
        { Icon::ICON_RECORD,          "control-record" },
        { Icon::ICON_PLAY,            "control" },
        { Icon::ICON_PAUSE,           "control-stop" },
        { Icon::ICON_SAVE,            "disk" },
        { Icon::ICON_OPEN,            "blue-document" },
        { Icon::ICON_LAMP,            "light-bulb" },
        { Icon::ICON_LAMP_DIM,        "status-away" },
        { Icon::ICON_HAZARD,          "exclamation" },
        { Icon::ICON_SEND,            "arrow-transition" },
        { Icon::ICON_HELP,            "information" },
        { Icon::ICON_REFERENCE,       "blue-document" },
        { Icon::ICON_CODE,            "document" },
        { Icon::ICON_DOC,             "ruler" },

        { Icon::ICON_STATUS_OK,       "status" },
        { Icon::ICON_STATUS_WARN,     "status-away" },
        { Icon::ICON_STATUS_BAD,      "status-busy" },
        { Icon::ICON_STATUS_IDLE,     "status-offline" },

        { Icon::ICON_LIVE,            "counter" },
        { Icon::ICON_SIGNAL,          "equalizer" },
        { Icon::ICON_SCAN,            "chart-up" },
        { Icon::ICON_DEVICE,          "processor" },

        { Icon::ICON_LINK,            "plug" },
        { Icon::ICON_SERVO,           "gear" },
        { Icon::ICON_TOF,             "binocular" },
        { Icon::ICON_ENCODER,         "counter" },
        { Icon::ICON_IMU,             "compass" },
        { Icon::ICON_STORAGE,         "card" },
        { Icon::ICON_NETWORK,         "network" },
        { Icon::ICON_MEASURE,         "ruler" },

        { Icon::ICON_MODE_POINTS,     "grid-dot" },
        { Icon::ICON_MODE_DENSITY,    "spectrum" },
        { Icon::ICON_MODE_MOTION,     "arrow-transition" },
        { Icon::ICON_MODE_CLEARANCE,  "shield" },
        { Icon::ICON_MODE_GAPS,       "door-open" },
        { Icon::ICON_MODE_WALLS,      "wall" },
        { Icon::ICON_MODE_CORNERS,    "node-design" },
        { Icon::ICON_MODE_FIT,        "car" },
        { Icon::ICON_MODE_FULL,       "dashboard" },
        { Icon::ICON_MODE_MINIMAL,    "eye" },

        { Icon::ICON_SCENE_CLOUD,     "asterisk" },
        { Icon::ICON_SCENE_BLOCKS,    "equalizer" },
        { Icon::ICON_SCENE_WALLS,     "wall" },
        { Icon::ICON_SCENE_FIT,       "car" },
        { Icon::ICON_SCENE_FULL,      "dashboard" },

        { Icon::ICON_DIM_2D,          "table" },
        { Icon::ICON_DIM_3D,          "node-design" },
    };

    constexpr Int32 COUNT = static_cast<Int32>(sizeof(FILES) / sizeof(FILES[0]));
    static_assert(COUNT == static_cast<Int32>(Icon::ICON_COUNT),
                  "icons.cxx: every Icon enumerator needs a file beside it");

    ID3D11ShaderResourceView* atlasSrv = nullptr;
    ID3D11Texture2D*          atlasTex = nullptr;

    // Extra textures handed out by loadTexture(). Freed with the atlas, so the
    // caller never owns one and cannot outlive the device.
    ID3D11Device* d3dDevice = nullptr;

    Vec<ID3D11Texture2D*>          extraTex;
    Vec<ID3D11ShaderResourceView*> extraSrv;
    Int32                     atlasW   = 0;
    Int32                     atlasH   = 0;

    // Where the app was launched from, so the assets resolve regardless of the
    // working directory - double-clicking the exe does not set one.
    Bool assetDir(Char* out, Size cap)
    {
        return assetPath("icons", out, cap);
    }

    // Decodes one PNG to 32-bit BGRA. Returns false without complaint on any
    // failure: a missing icon must degrade to no icon, never to a crash or a
    // dialog, because the app's job is talking to a lidar.
    Bool decodePng(IWICImagingFactory* wic, const Char* path, Vec<UInt8>& out, UInt32& w, UInt32& h)
    {
        WCHAR wide[MAX_PATH];
        if(::MultiByteToWideChar(CP_UTF8, 0, path, -1, wide, MAX_PATH) == 0)
        {
            return false;
        }

        IWICBitmapDecoder*     dec   = nullptr;
        IWICBitmapFrameDecode* frame = nullptr;
        IWICFormatConverter*   conv  = nullptr;
        Bool ok = false;

        if(SUCCEEDED(wic->CreateDecoderFromFilename(wide, nullptr, GENERIC_READ,
                                                    WICDecodeMetadataCacheOnDemand, &dec)) &&
           SUCCEEDED(dec->GetFrame(0, &frame)) &&
           SUCCEEDED(wic->CreateFormatConverter(&conv)) &&
           SUCCEEDED(conv->Initialize(frame, GUID_WICPixelFormat32bppPBGRA,
                                      WICBitmapDitherTypeNone, nullptr, 0.0,
                                      WICBitmapPaletteTypeCustom)) &&
           SUCCEEDED(conv->GetSize(&w, &h)))
        {
            out.resize(static_cast<Size>(w) * h * 4);
            ok = SUCCEEDED(conv->CopyPixels(nullptr, w * 4,
                                            static_cast<UINT>(out.size()), out.data()));
        }

        if(conv  != nullptr)
        {
            conv->Release();
        }
        if(frame != nullptr)
        {
            frame->Release();
        }
        if(dec   != nullptr)
        {
            dec->Release();
        }
        return ok;
    }

    // Integer scale only. See the note in icons.hxx: a fractional multiple of pixel
    // art is mush, and these exist at exactly one size.
    Int32 scaleSteps() noexcept
    {
        const Float32 d = dpiScale();
        if(d >= 2.75f)
        {
            return 3;
        }
        if(d >= 1.85f)
        {
            return 2;
        }
        return 1;
    }

    Void uvFor(Icon ic, ImVec2& uv0, ImVec2& uv1) noexcept
    {
        const Int32 i  = static_cast<Int32>(ic);
        const Int32 cx = (i % COLS) * SRC;
        const Int32 cy = (i / COLS) * SRC;
        uv0 = ImVec2(static_cast<Float32>(cx) / static_cast<Float32>(atlasW),
                     static_cast<Float32>(cy) / static_cast<Float32>(atlasH));
        uv1 = ImVec2(static_cast<Float32>(cx + SRC) / static_cast<Float32>(atlasW),
                     static_cast<Float32>(cy + SRC) / static_cast<Float32>(atlasH));
    }

  }

  Bool assetPath(const Char* relative, Char* out, Size cap)
  {
      if(relative == nullptr || out == nullptr || cap == 0)
      {
          return false;
      }

      Array<Char, MAX_PATH> exe;
      const DWORD n = ::GetModuleFileNameA(nullptr, exe.data(), MAX_PATH);
      if(n == 0 || n >= MAX_PATH)
      {
          return false;
      }

      Char* slash = std::strrchr(exe.data(), '\\');
      if(slash == nullptr)
      {
          return false;
      }
      *slash = 0;

      constexpr const Char* const LAYOUTS[] = {
          "%s\\assets\\%s",        // shipped: assets beside the exe
          "%s\\..\\assets\\%s",    // build.bat: exe one level down, in build/
      };

      for(const Char* fmt : LAYOUTS)
      {
          std::snprintf(out, cap, fmt, exe.data(), relative);
          if(::GetFileAttributesA(out) != INVALID_FILE_ATTRIBUTES)
          {
              return true;
          }
      }

      // Neither exists. Leave the FIRST candidate in `out` so a caller that logs
      // the path names the one it should have been, not the fallback it tried last.
      std::snprintf(out, cap, LAYOUTS[0], exe.data(), relative);
      return false;
  }

  Void loadIcons(ID3D11Device* device)
  {
      d3dDevice = device;

      releaseIcons();
      if(device == nullptr)
      {
          return;
      }

      Array<Char, MAX_PATH> dir;
      if(!assetDir(dir.data(), dir.size()))
      {
          return;
      }

      const Int32 rows = (COUNT + COLS - 1) / COLS;
      atlasW = COLS * SRC;
      atlasH = rows * SRC;

      Vec<UInt8> atlas(static_cast<Size>(atlasW) * atlasH * 4, 0);

      IWICImagingFactory* wic = nullptr;
      // The app never calls CoInitialize, so ask for the factory with the
      // apartment we are already in; WIC is happy either way for decoding.
      if(FAILED(::CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE)))
      {
          // Already initialized on this thread is fine and not an error for us.
      }
      if(FAILED(::CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                   IID_PPV_ARGS(&wic))))
      {
          return;
      }

      Vec<UInt8> px;
      Int32 loaded = 0;
      for(Int32 i = 0; i < COUNT; ++i)
      {
          Array<Char, MAX_PATH> path;
          std::snprintf(path.data(), path.size(), "%s\\%s.png", dir.data(), FILES[i].file);

          UInt32 w = 0, h = 0;
          if(!decodePng(wic, path.data(), px, w, h))
          {
              continue;
          }
          if(w != static_cast<UInt32>(SRC) || h != static_cast<UInt32>(SRC))
          {
              continue;   // not the art we expect; skip rather than stretch it
          }

          const Int32 dx = (static_cast<Int32>(FILES[i].id) % COLS) * SRC;
          const Int32 dy = (static_cast<Int32>(FILES[i].id) / COLS) * SRC;
          for(Int32 y = 0; y < SRC; ++y)
          {
              std::memcpy(&atlas[(static_cast<Size>(dy + y) * atlasW + dx) * 4],
                          &px[static_cast<Size>(y) * SRC * 4],
                          static_cast<Size>(SRC) * 4);
          }
          ++loaded;
      }
      wic->Release();

      if(loaded == 0)
      {
          atlasW = atlasH = 0;
          return;
      }

      D3D11_TEXTURE2D_DESC td = {};
      td.Width            = static_cast<UINT>(atlasW);
      td.Height           = static_cast<UINT>(atlasH);
      td.MipLevels        = 1;
      td.ArraySize        = 1;
      td.Format           = DXGI_FORMAT_B8G8R8A8_UNORM;
      td.SampleDesc.Count = 1;
      td.Usage            = D3D11_USAGE_IMMUTABLE;
      td.BindFlags        = D3D11_BIND_SHADER_RESOURCE;

      D3D11_SUBRESOURCE_DATA sd = {};
      sd.pSysMem     = atlas.data();
      sd.SysMemPitch = static_cast<UINT>(atlasW * 4);

      if(FAILED(device->CreateTexture2D(&td, &sd, &atlasTex)))
      {
          atlasW = atlasH = 0;
          return;
      }
      if(FAILED(device->CreateShaderResourceView(atlasTex, nullptr, &atlasSrv)))
      {
          atlasTex->Release();
          atlasTex = nullptr;
          atlasW = atlasH = 0;
      }
  }

  // See icons.hxx. Uses the same WIC decode the atlas does.
  ID3D11Device* device() noexcept
  {
      return d3dDevice;
  }

  ImTextureID loadTexture(ID3D11Device* device, const Char* path)
  {
      if(device == nullptr || path == nullptr)
      {
          return 0;
      }

      IWICImagingFactory* wic = nullptr;
      if(FAILED(::CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                                   IID_PPV_ARGS(&wic))) || wic == nullptr)
      {
          return 0;
      }

      Vec<UInt8> px;
      UInt32 w = 0, h = 0;
      const Bool ok = decodePng(wic, path, px, w, h);
      wic->Release();

      if(!ok || w == 0 || h == 0)
      {
          return 0;
      }

      D3D11_TEXTURE2D_DESC td = {};
      td.Width            = w;
      td.Height           = h;
      td.MipLevels        = 1;
      td.ArraySize        = 1;
      td.Format           = DXGI_FORMAT_B8G8R8A8_UNORM;
      td.SampleDesc.Count = 1;
      td.Usage            = D3D11_USAGE_IMMUTABLE;
      td.BindFlags        = D3D11_BIND_SHADER_RESOURCE;

      D3D11_SUBRESOURCE_DATA sd = {};
      sd.pSysMem     = px.data();
      sd.SysMemPitch = w * 4;

      ID3D11Texture2D* tex = nullptr;
      if(FAILED(device->CreateTexture2D(&td, &sd, &tex)))
      {
          return 0;
      }

      ID3D11ShaderResourceView* srv = nullptr;
      if(FAILED(device->CreateShaderResourceView(tex, nullptr, &srv)))
      {
          tex->Release();
          return 0;
      }

      extraTex.push_back(tex);
      extraSrv.push_back(srv);
      return reinterpret_cast<ImTextureID>(srv);
  }

  Void releaseIcons()
  {
      for(ID3D11ShaderResourceView* e : extraSrv)
      {
          if(e != nullptr)
          {
              e->Release();
          }
      }
      for(ID3D11Texture2D* t : extraTex)
      {
          if(t != nullptr)
          {
              t->Release();
          }
      }
      extraSrv.clear();
      extraTex.clear();


      if(atlasSrv != nullptr)
      {
          atlasSrv->Release();
          atlasSrv = nullptr;
      }
      if(atlasTex != nullptr)
      {
          atlasTex->Release();
          atlasTex = nullptr;
      }
      atlasW = atlasH = 0;
  }

  Bool iconsReady() noexcept
  {
      return atlasSrv != nullptr;
  }

  Float32 iconSize() noexcept
  {
      return static_cast<Float32>(SRC * scaleSteps());
  }

  Void icon(Icon ic)
  {
      const Float32 s = iconSize();
      if(!iconsReady() || ic >= Icon::ICON_COUNT)
      {
          ImGui::Dummy(ImVec2(s, s));
          return;
      }
      ImVec2 uv0, uv1;
      uvFor(ic, uv0, uv1);
      ImGui::Image(reinterpret_cast<ImTextureID>(atlasSrv), ImVec2(s, s), uv0, uv1);
  }

  Void iconAt(ImDrawList* dl, Icon ic, const ImVec2& pos, ImU32 tint)
  {
      if(!iconsReady() || dl == nullptr || ic >= Icon::ICON_COUNT)
      {
          return;
      }
      const Float32 s = iconSize();
      ImVec2 uv0, uv1;
      uvFor(ic, uv0, uv1);
      dl->AddImage(reinterpret_cast<ImTextureID>(atlasSrv), pos,
                   ImVec2(pos.x + s, pos.y + s), uv0, uv1, tint);
  }

  Bool iconButton(Icon ic, const Char* label, const ImVec2& size, Tint tint)
  {
      const ImGuiStyle& sty = ImGui::GetStyle();
      const Float32     sz  = iconSize();
      const Float32     gap = sty.ItemInnerSpacing.x;

      // An auto-sized button is exactly its text plus padding, so there is no
      // margin for a glyph to sit in and the fit test below rejected every one -
      // which it silently did for as long as this function has existed. Widen the
      // auto case, and push the label flush RIGHT so all of the new space lands on
      // the left where the icon goes. Centering it instead would split the space in
      // two and the icon would still not fit.
      //
      // Only the auto case. A caller who passed a width meant that width, and the
      // wide buttons that pass one look right with a centered label and the icon
      // out at the frame padding, which is the branch below.
      const Bool  autoW = (size.x == 0.0f) && iconsReady();
      ImVec2      sz2   = size;
      if(autoW)
      {
          sz2.x = ImGui::CalcTextSize(label).x + sty.FramePadding.x * 2.0f + sz + gap;
          ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(1.0f, 0.5f));
      }

      const Bool hit = button(label, sz2, tint);

      if(autoW)
      {
          ImGui::PopStyleVar();
      }

      if(iconsReady())
      {
          const ImVec2  a = ImGui::GetItemRectMin();
          const ImVec2  b = ImGui::GetItemRectMax();
          const Float32 x = a.x + sty.FramePadding.x;
          const Float32 y = a.y + ((b.y - a.y) - sz) * 0.5f;

          // Auto-width already reserved the room above. Otherwise only if the
          // glyph clears the centered label: on a narrow button it would sit on top
          // of the text, and the text is the part that has to survive.
          const Bool fits = autoW
              || (x + sz < (a.x + b.x) * 0.5f - ImGui::CalcTextSize(label).x * 0.5f);

          if(fits)
          {
              iconAt(ImGui::GetWindowDrawList(), ic, ImVec2(x, y));
          }
      }
      return hit;
  }

  Bool iconMenuItem(Icon ic, const Char* label, const Char* shortcut, Bool enabled, Bool selected)
  {
      // Enough leading spaces to clear the icon, measured rather than guessed -
      // the icon size and the space width move independently with DPI.
      Array<Char, 128> padded;
      const Float32 spaceW = ImGui::CalcTextSize(" ").x;
      Int32 n = 3;
      if(iconsReady() && spaceW > 0.0f)
      {
          n = static_cast<Int32>(std::ceil((iconSize()
                                            + ImGui::GetStyle().ItemInnerSpacing.x)
                                           / spaceW));
      }
      n = (n < 0) ? 0 : ((n > 32) ? 32 : n);

      std::snprintf(padded.data(), padded.size(), "%*s%s", n, "", label);

      const Bool hit = ImGui::MenuItem(padded.data(), shortcut, selected, enabled);

      if(iconsReady())
      {
          const ImVec2  a  = ImGui::GetItemRectMin();
          const ImVec2  b  = ImGui::GetItemRectMax();
          const Float32 sz = iconSize();
          const Float32 x  = a.x + ImGui::GetStyle().FramePadding.x;
          const Float32 y  = a.y + ((b.y - a.y) - sz) * 0.5f;

          // Dimmed with the label when the entry is disabled, or the icon would
          // be the one bright thing on a grayed-out row.
          const ImU32 tint = enabled
              ? IM_COL32_WHITE
              : ImGui::GetColorU32(ImGuiCol_TextDisabled);

          iconAt(ImGui::GetWindowDrawList(), ic, ImVec2(x, y), tint);
      }
      return hit;
  }

  Bool segmentedIconButton(Icon ic, const Char* label, Bool selected, const ImVec2& size)
  {
      const Bool hit = segmentedButton(label, selected, size);

      if(iconsReady())
      {
          const ImVec2  a    = ImGui::GetItemRectMin();
          const ImVec2  b    = ImGui::GetItemRectMax();
          const Float32 sz   = iconSize();
          const Float32 gap  = ImGui::GetStyle().ItemInnerSpacing.x;
          const Float32 half = ImGui::CalcTextSize(label).x * 0.5f;

          // Immediately left of the CENTERED label, not out at the frame padding.
          // These cells are wide, and an icon pinned to the far margin reads as
          // unrelated to the word in the middle of the same button.
          const Float32 x = (a.x + b.x) * 0.5f - half - gap - sz;
          const Float32 y = a.y + ((b.y - a.y) - sz) * 0.5f;

          // Only when it fits inside the cell; on a narrow one the text is the
          // part that has to survive.
          if(x > a.x + ImGui::GetStyle().FramePadding.x * 0.5f)
          {
              // Unselected cells are quiet, so their icons are too - otherwise a
              // row of twelve reads as twelve equally-loud things and the
              // selection stops being the thing you see first.
              iconAt(ImGui::GetWindowDrawList(), ic, ImVec2(x, y),
                     selected ? IM_COL32_WHITE : IM_COL32(255, 255, 255, 130));
          }
      }
      return hit;
  }

  Void iconLabel(Icon ic)
  {
      // Nudged down so a 16px icon sits on the text baseline rather than on the
      // top of the line box, which is where ImGui::Image would otherwise put it.
      const Float32 drop = (ImGui::GetTextLineHeight() - iconSize()) * 0.5f;
      if(drop > 0.0f)
      {
          ImGui::SetCursorPosY(ImGui::GetCursorPosY() + drop);
      }

      icon(ic);
      ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);

      if(drop > 0.0f)
      {
          ImGui::SetCursorPosY(ImGui::GetCursorPosY() - drop);
      }
  }

}
