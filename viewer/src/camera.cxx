#include "shared.hxx"

#include <cstdio>
#include <cstring>
#include <windows.h>
#include <d3d11.h>

// <rpcndr.h> does `#define small char`; near and far come from the same era.
// SEVERITY_SUCCESS and SEVERITY_ERROR are <winnt.h>'s HRESULT severity bits,
// and bibowire::Severity has a member by the second of those names - so
// `Severity::SEVERITY_ERROR` would become `Severity::1` and the protocol
// header would stop parsing hundreds of lines before anything of ours is read.
//
// Killed at the boundary, in the same place and for the same reason as in
// main.cxx: every one of these must be undefined BEFORE camera.hxx below,
// because that header includes link.hxx and link.hxx includes bibowire.hxx.
#undef small
#undef near
#undef far
#undef SEVERITY_SUCCESS
#undef SEVERITY_ERROR

#include "imgui.h"

#include "camera.hxx"
#include "jpeg.hxx"
#include "orient.hxx"

namespace camview
{

  namespace
  {

    // Handed over by init(). This module creates textures and never presents,
    // so it needs the device to make them and the context to fill them.
    ID3D11Device* device = nullptr;
    ID3D11DeviceContext* context = nullptr;
    Float32 uiScale = 1.0f;

    constexpr Float32 READOUT_COLUMN = 104.0f;

    Void readout(CharSeq label, CharSeq value)
    {
        ImGui::TextUnformatted(label);
        ImGui::SameLine(READOUT_COLUMN * uiScale);
        ImGui::TextUnformatted(value);
    }

    // Integer digits, never printf's "%.1f": the decimal point honours the
    // locale, a machine set to a comma decimal writes "3,2", and that is the
    // bug this project has already met three times.
    [[nodiscard]] Str secondsText(Int64 ms)
    {
        Array<Char, 32> t = {};
        const Int64 whole = ms / 1000;
        const Int64 tenth = (ms % 1000) / 100;
        std::snprintf(t.data(), t.size(), "%lld.%lld s", whole, tenth);
        return Str(t.data());
    }

    Void releaseTexture(View& v)
    {
        if(v.srv != nullptr)
        {
            v.srv->Release();
            v.srv = nullptr;
        }
        if(v.tex != nullptr)
        {
            v.tex->Release();
            v.tex = nullptr;
        }
        v.texW = 0;
        v.texH = 0;
        v.haveShown = false;
        v.shownIndex = 0;
        v.shownBytes = 0;
    }

    // DYNAMIC and CPU-writable, because this texture is rewritten from a new
    // JPEG several times a second and a DEFAULT one would need a staging copy
    // for every frame.
    [[nodiscard]] Bool makeTexture(View& v, Int32 width, Int32 height)
    {
        releaseTexture(v);
        if(device == nullptr || width <= 0 || height <= 0)
        {
            return false;
        }

        D3D11_TEXTURE2D_DESC desc;
        ZeroMemory(&desc, sizeof(desc));
        desc.Width = static_cast<UINT>(width);
        desc.Height = static_cast<UINT>(height);
        desc.MipLevels = 1;
        desc.ArraySize = 1;
        desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        desc.SampleDesc.Count = 1;
        desc.Usage = D3D11_USAGE_DYNAMIC;
        desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

        if(FAILED(device->CreateTexture2D(&desc, nullptr, &v.tex)) || v.tex == nullptr)
        {
            return false;
        }

        D3D11_SHADER_RESOURCE_VIEW_DESC srv;
        ZeroMemory(&srv, sizeof(srv));
        srv.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        srv.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
        srv.Texture2D.MipLevels = 1;

        if(FAILED(device->CreateShaderResourceView(v.tex, &srv, &v.srv)))
        {
            releaseTexture(v);
            return false;
        }

        v.texW = width;
        v.texH = height;
        return true;
    }

    // ROW BY ROW, because RowPitch is the driver's and is not width * 4. A
    // single memcpy of the whole buffer is the shape that works on the machine
    // it was written on and skews the picture diagonally on the next one.
    [[nodiscard]] Bool uploadPixels(View& v, const jpeg::Picture& pic)
    {
        if(context == nullptr || v.tex == nullptr)
        {
            return false;
        }

        D3D11_MAPPED_SUBRESOURCE mapped;
        ZeroMemory(&mapped, sizeof(mapped));
        if(FAILED(context->Map(v.tex, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)))
        {
            return false;
        }

        const Size rowBytes = static_cast<Size>(pic.width) * 4u;
        for(Int32 y = 0; y < pic.height; ++y)
        {
            UInt8* dst = static_cast<UInt8*>(mapped.pData) + (static_cast<Size>(y) * mapped.RowPitch);
            const UInt8* src = pic.rgba.data() + (static_cast<Size>(y) * rowBytes);
            std::memcpy(dst, src, rowBytes);
        }

        context->Unmap(v.tex, 0);
        return true;
    }

    // Decode one JPEG into the texture, creating or resizing it only when the
    // dimensions have actually moved.
    Void adopt(View& v, const link::CameraShot& shot)
    {
        v.haveShown = true;
        v.shownIndex = shot.frameIndex;
        v.shownBytes = shot.bytes.size();
        v.decodeWhy.clear();

        // The codec byte is echoed on every frame so a capture is
        // self-describing (docs/bibowire.md section 10). Anything but 1 is not
        // a JPEG, and handing it to a JPEG decoder to find out would be
        // guessing at a value the frame already stated.
        if(shot.codec != 1u)
        {
            ++v.decodeFailures;
            Array<Char, 96> t = {};
            const UInt32 codec = shot.codec;
            std::snprintf(
                t.data(),
                t.size(),
                "codec %u is not JPEG - nothing here can show it",
                codec
            );
            v.decodeWhy = Str(t.data());
            releaseTexture(v);
            // Marked shown AFTER the release, and that order is load-bearing:
            // releaseTexture clears these three, so setting them first would
            // leave this frame looking un-adopted and the refusal would be
            // recomputed at 60 Hz for as long as the camera sent that codec.
            v.haveShown = true;
            v.shownIndex = shot.frameIndex;
            v.shownBytes = shot.bytes.size();
            return;
        }

        jpeg::Picture pic;
        Str why;
        if(!jpeg::decode(shot.bytes.data(), shot.bytes.size(), &pic, &why))
        {
            ++v.decodeFailures;
            v.decodeWhy = why;
            return;
        }

        if(v.tex == nullptr || v.texW != pic.width || v.texH != pic.height)
        {
            if(!makeTexture(v, pic.width, pic.height))
            {
                ++v.decodeFailures;
                v.decodeWhy = "could not create a texture for the picture";
                return;
            }
        }

        if(!uploadPixels(v, pic))
        {
            ++v.decodeFailures;
            v.decodeWhy = "could not upload the picture to the graphics device";
        }
    }

    // The corner mapping lives in orient.cxx, which names no ImGui or D3D type
    // and can therefore be linked into viewer/tests. It was here first, where
    // nothing could reach it: the 90-degree case is a transpose, it is the part
    // most likely to be silently wrong, and "it looked right on screen" is not
    // a check anybody can re-run.

    // The controls, drawn whether or not there is a picture behind them.
    //
    // Deliberately ABOVE the picture and outside every early return: the rate
    // is the thing an operator reaches for when the window is empty or jerky,
    // and a control that appears only once a frame has arrived is missing at
    // exactly the moment it is wanted.
    Void drawControls(View& v)
    {
        static constexpr Array<CharSeq, 4> TURN_NAMES = { "0", "90", "180", "270" };

        ImGui::SetNextItemWidth(72.0f * uiScale);
        Int32 turns = v.turns;
        const Int32 turnCount = static_cast<Int32>(TURN_NAMES.size());
        if(ImGui::Combo("rotate", &turns, TURN_NAMES.data(), turnCount))
        {
            v.turns = turns;
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("degrees clockwise - the camera may be\nmounted sideways on the car");
        }

        ImGui::SameLine();
        ImGui::Checkbox("flip H", &v.flipX);
        ImGui::SameLine();
        ImGui::Checkbox("flip V", &v.flipY);

        ImGui::SetNextItemWidth(-96.0f * uiScale);
        Int32 fps = v.fps;
        const Int32 ceiling = static_cast<Int32>(bibowire::CAM_FPS_MAX);
        if(ImGui::SliderInt("rate", &fps, 0, ceiling, fps == 0 ? "board default" : "%d fps"))
        {
            v.fps = fps < 0 ? 0 : (fps > ceiling ? ceiling : fps);
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "frames a second to ask the board for.\n"
                "0 leaves the board's own default, which is\n"
                "two - honest for a hotspot, a slideshow on a LAN.\n"
                "Pictures are dropped before scans, so asking for\n"
                "more than the link can carry costs the camera\n"
                "and never the car's view of the room."
            );
        }
    }

    // The sentence for a window with no picture in it. Which of these is true
    // is the whole question an operator has when the rectangle is empty, and
    // an empty rectangle answers none of them.
    [[nodiscard]] Str whyNothing(link::Client& lk, const link::Snapshot& snap, Int64 nowMs)
    {
        if(!link::isOpen(lk))
        {
            return "not connected - the camera comes over the same link as the scan";
        }
        if(!snap.state.haveWelcome)
        {
            return "handshaking - nothing is asked for until the board has answered HELLO";
        }
        if(!snap.state.cameraSubscribed)
        {
            return "asking the board for the camera";
        }
        if(!snap.state.haveCamera)
        {
            return "subscribed - the board has not sent a frame yet";
        }
        // Subscribed, frames arrived, and the newest is too old to draw. The
        // link can be perfectly healthy while this is true, which is exactly
        // the pair of lies section 7 keeps apart.
        const Int64 age = nowMs - snap.state.cameraAtMs;
        return "no camera frame for " + secondsText(age < 0 ? 0 : age);
    }

  }

  Void init(ID3D11Device* dev, ID3D11DeviceContext* ctx, Float32 scale)
  {
      device = dev;
      context = ctx;
      uiScale = scale > 0.0f ? scale : 1.0f;
  }

  Void shutdown(View& v)
  {
      releaseTexture(v);
  }

  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs)
  {
      // Told every frame, from the one piece of state that decides it. Two
      // booleans - "the window is open" and "the board is sending" - that were
      // set in different places would drift, and the direction they drift in
      // costs a megabyte a second on a phone hotspot.
      link::wantCamera(lk, v.open);

      // Told every frame, beside the subscription and for the same reason: the
      // window's slider is the one piece of state that decides it, and a rate
      // cached anywhere else would drift from what the operator is looking at.
      // The worker re-sends SUBSCRIBE only when the number actually changes.
      link::wantCameraFps(lk, v.fps);

      if(!v.open)
      {
          // Nothing on screen and nothing on the wire: the texture goes back
          // with the subscription, because 1.2 MB of device memory holding a
          // picture nobody can see is the same waste as the bandwidth.
          releaseTexture(v);
          return;
      }

      ImGui::SetNextWindowPos(ImVec2(380.0f * uiScale, 16.0f * uiScale), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(420.0f * uiScale, 400.0f * uiScale), ImGuiCond_FirstUseEver);

      // `&v.open` is what puts the X in the title bar, and closing it is what
      // unsubscribes on the next frame.
      if(!ImGui::Begin("Camera", &v.open))
      {
          ImGui::End();
          return;
      }

      drawControls(v);
      ImGui::Separator();

      const Opt<link::CameraShot> shot = snap.state.cameraShot(nowMs);

      if(!shot.has_value())
      {
          // A frame this viewer never received, or one too old to draw. Either
          // way there is no picture, and the texture is released so a later
          // bug cannot draw the previous one.
          releaseTexture(v);
          ImGui::TextWrapped("%s", whyNothing(lk, snap, nowMs).c_str());

          // The board's own sentence, verbatim. It is the difference between
          // an empty rectangle and "the phone dashboard has the camera".
          if(snap.state.haveCameraNote)
          {
              ImGui::Separator();
              ImGui::TextWrapped("board: %s", snap.state.cameraNoteText.c_str());
          }
          ImGui::End();
          return;
      }

      // Decoded once per FRAME INDEX, not once per redraw. The byte length
      // joins the comparison so a board that restarts its counter at 0 while
      // frame 0 is still on screen is still a new picture.
      const Bool same = v.haveShown
                        && v.shownIndex == shot->frameIndex
                        && v.shownBytes == shot->bytes.size();
      if(!same)
      {
          adopt(v, *shot);
      }

      if(v.srv != nullptr && v.texW > 0 && v.texH > 0)
      {
          // THE DISPLAYED SHAPE SWAPS AT 90 AND 270. A 640x480 picture shown on
          // its side is 480x640, so the fit has to be computed from the turned
          // dimensions - fitting the unturned ones would stretch the picture
          // into a rectangle of the wrong shape and quietly change its aspect
          // ratio, which on a camera used for judging clearance is a lie about
          // the room.
          const Bool turned = orient::sideways(v.turns);
          const Float32 wide = static_cast<Float32>(turned ? v.texH : v.texW);
          const Float32 high = static_cast<Float32>(turned ? v.texW : v.texH);

          // Fit the width, keep the aspect ratio. The picture is what the
          // window is for, so it takes the space and the readouts sit under it.
          const Float32 avail = ImGui::GetContentRegionAvail().x;
          const Float32 shown = avail > 16.0f ? avail : 16.0f;
          const ImVec2 size = ImVec2(shown, shown * (high / wide));
          const ImTextureID id = static_cast<ImTextureID>(reinterpret_cast<UPtr>(v.srv));

          // DESATURATED when stale, with its age printed below - section 7's
          // middle band. A greyed-out picture is still a picture, which is why
          // the band above it is absence rather than a darker grey.
          const ImU32 tint = shot->stale ? IM_COL32(115, 115, 115, 255) : IM_COL32_WHITE;

          const Array<orient::Uv, 4> corner = orient::cornerUvs(v.turns, v.flipX, v.flipY);
          const ImVec2 uv0 = ImVec2(corner[0].u, corner[0].v);
          const ImVec2 uv1 = ImVec2(corner[1].u, corner[1].v);
          const ImVec2 uv2 = ImVec2(corner[2].u, corner[2].v);
          const ImVec2 uv3 = ImVec2(corner[3].u, corner[3].v);
          const ImVec2 at = ImGui::GetCursorScreenPos();
          const ImVec2 topLeft = at;
          const ImVec2 topRight = ImVec2(at.x + size.x, at.y);
          const ImVec2 botRight = ImVec2(at.x + size.x, at.y + size.y);
          const ImVec2 botLeft = ImVec2(at.x, at.y + size.y);

          ImGui::GetWindowDrawList()->AddImageQuad(
              ImTextureRef(id),
              topLeft,
              topRight,
              botRight,
              botLeft,
              uv0,
              uv1,
              uv2,
              uv3,
              tint
          );

          // The draw list does not move the cursor, so the layout is told how
          // much room the picture took. Without this the readouts below would
          // be drawn on top of it.
          ImGui::Dummy(size);
      }
      else
      {
          // A frame arrived and could not be shown. That is a different fact
          // from "no frame arrived" and it gets a different sentence.
          ImGui::TextWrapped("frame %u arrived and could not be shown", shot->frameIndex);
          if(!v.decodeWhy.empty())
          {
              ImGui::TextWrapped("%s", v.decodeWhy.c_str());
          }
      }

      ImGui::Separator();

      const Str age = secondsText(shot->ageMs) + (shot->stale ? " - STALE" : "");
      readout("age", age.c_str());

      // THE BAND, AND THE CADENCE IT CAME FROM. Shown rather than kept inside
      // the accessor, because "STALE" with no number beside it is exactly the
      // claim that used to appear twice a second on a camera that was never
      // late - and a staleness rule nobody can read off the running system is
      // a rule nobody can catch being wrong.
      if(shot->worstGapMs > 0)
      {
          Array<Char, 96> band = {};
          std::snprintf(
              band.data(),
              band.size(),
              "stale past %lld ms, from a %lld ms cadence",
              shot->staleAtMs,
              shot->worstGapMs
          );
          readout("band", band.data());
      }

      Array<Char, 64> what = {};
      std::snprintf(
          what.data(),
          what.size(),
          "%ux%u, %u bytes",
          static_cast<UInt32>(shot->width),
          static_cast<UInt32>(shot->height),
          static_cast<UInt32>(shot->bytes.size())
      );
      readout("frame", what.data());

      Array<Char, 48> num = {};
      std::snprintf(num.data(), num.size(), "%u", shot->frameIndex);
      readout("index", num.data());

      // GAPS ARE COUNTED, NEVER SMOOTHED - the same rule the scan follows.
      // frameIndex is monotonic, so what is missing is knowable exactly, and a
      // viewer that just showed the next picture would be hiding a link
      // dropping half the stream.
      if(snap.state.missedCameraFrames > 0u && !snap.state.cameraGapText.empty())
      {
          readout("missed", snap.state.cameraGapText.c_str());
      }

      if(v.decodeFailures > 0u)
      {
          Array<Char, 48> bad = {};
          std::snprintf(bad.data(), bad.size(), "%u", v.decodeFailures);
          readout("undecodable", bad.data());
      }

      if(snap.state.haveCameraNote)
      {
          ImGui::Separator();
          ImGui::TextWrapped("board: %s", snap.state.cameraNoteText.c_str());
      }

      ImGui::End();
  }

}
