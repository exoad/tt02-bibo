#include "shared.hxx"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <windows.h>
#include <d3d11.h>

// Windows macros that break our headers, undefined before camera.hxx reaches
// bibowire.hxx through link.hxx: <rpcndr.h>'s `small`, `near` and `far`, and
// <winnt.h>'s SEVERITY_SUCCESS and SEVERITY_ERROR, which would turn
// bibowire::Severity::SEVERITY_ERROR into Severity::1.
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
    // From init(): the device creates textures and the context fills them.
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

    // Integer digits, never "%.1f" (the locale trap, vlog.hxx).
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

    // DYNAMIC and CPU-writable: rewritten several times a second, where a
    // DEFAULT texture would need a staging copy per frame.
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

    // Row by row, because RowPitch is the driver's and need not be width * 4.
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

    // Decodes one JPEG into the texture, recreating it only when the dimensions
    // change.
    Void adopt(View& v, const link::CameraShot& shot)
    {
        v.haveShown = true;
        v.shownIndex = shot.frameIndex;
        v.shownBytes = shot.bytes.size();
        v.decodeWhy.clear();
        // Codec 1 is JPEG and nothing else is (docs/bibowire.md section 5): any
        // other value is refused, not handed to the decoder to guess.
        if(shot.codec != 1u)
        {
            ++v.decodeFailures;
            Array<Char, 96> t = {};
            const UInt32 codec = shot.codec;
            std::snprintf(t.data(), t.size(), "codec %u is not JPEG", codec);
            v.decodeWhy = Str(t.data());
            releaseTexture(v);
            // Marked shown AFTER the release, which clears these three;
            // otherwise the refusal is recomputed every frame.
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
                v.decodeWhy = "texture create failed";
                return;
            }
        }
        if(!uploadPixels(v, pic))
        {
            ++v.decodeFailures;
            v.decodeWhy = "texture upload failed";
        }
    }

    // OVERLAYS FOLLOW THE GUI, NEVER THE CAMERA. Every point below is a fraction
    // of the picture AS DRAWN, in window space, and rotate and flip never touch
    // them: marks the operator dragged into place against what they see must not
    // move when the mount setting is corrected. None is calibrated, so no band is
    // ever labelled with a distance.
    constexpr ImU32 OVERLAY_SHADE = IM_COL32(0, 0, 0, 165);
    constexpr ImU32 CROSS_COL = IM_COL32(255, 255, 255, 225);
    constexpr ImU32 BOX_COL = IM_COL32(120, 210, 255, 225);
    constexpr ImU32 THIRDS_COL = IM_COL32(235, 235, 235, 105);

    // Near, middle and far in reversing-camera colours: zones placed by eye, not
    // distances.
    constexpr ImU32 ZONE_NEAR = IM_COL32(244, 76, 62, 235);
    constexpr ImU32 ZONE_MID = IM_COL32(255, 196, 46, 235);
    constexpr ImU32 ZONE_FAR = IM_COL32(96, 226, 130, 235);

    // Where the picture landed on screen; overlays are placed against this only.
    struct Placed
    {
        ImVec2 at;
        ImVec2 size;
    };

    // The guide trapezoid as clamped fractions of the frame.
    struct Rails
    {
        Float32 centre = 0.5f;
        Float32 spread = 0.42f;
        Float32 converge = 0.12f;
        Float32 nearY = 1.0f;
        Float32 farY = 0.45f;
    };

    // A light stroke over a dark one, so a line shows over a bright sky or a
    // dark garage alike.
    Void strokeLine(ImDrawList* dl, const ImVec2& a, const ImVec2& b, ImU32 col, Float32 w)
    {
        dl->AddLine(a, b, OVERLAY_SHADE, w + (2.0f * uiScale));
        dl->AddLine(a, b, col, w);
    }

    // A fraction of the drawn rectangle to a pixel. No rotation and no mirror,
    // on purpose.
    [[nodiscard]] ImVec2 screenOf(const Placed& p, Float32 x, Float32 y)
    {
        return ImVec2(p.at.x + (x * p.size.x), p.at.y + (y * p.size.y));
    }

    [[nodiscard]] ImVec2 stepBy(const ImVec2& from, const ImVec2& dir, Float32 k)
    {
        return ImVec2(from.x + (dir.x * k), from.y + (dir.y * k));
    }

    [[nodiscard]] ImVec2 unitFrom(const ImVec2& from, const ImVec2& to)
    {
        const Float32 dx = to.x - from.x;
        const Float32 dy = to.y - from.y;
        const Float32 len = std::sqrt((dx * dx) + (dy * dy));
        if(len < 0.0001f)
        {
            return ImVec2(0.0f, 0.0f);
        }
        return ImVec2(dx / len, dy / len);
    }

    [[nodiscard]] Rails railsOf(const View& v)
    {
        Rails r;
        r.centre = pctToUnit(v.guideCentrePct, 10, 90);
        r.spread = pctToUnit(v.guideSpreadPct, 5, 60);
        r.converge = pctToUnit(v.guideConvergePct, 0, 40);
        r.nearY = pctToUnit(v.guideNearPct, 40, 100);
        r.farY = pctToUnit(v.guideFarPct, 5, 95);
        return r;
    }

    // One point on one rail. `side` is -1 for the left rail and +1 for the
    // right; `t` runs from 0 at the near end to 1 at the far end.
    [[nodiscard]] ImVec2 railPoint(const Placed& p, const Rails& r, Float32 side, Float32 t, Float32 steer, Int32 bendPct)
    {
        const Float32 nearX = r.centre + (side * r.spread);
        const Float32 farX = r.centre + (side * r.converge);
        const Float32 bend = bendAt(steer, bendPct, t);
        // The only place a guide point is made, so the bend and the trapezoid
        // cannot disagree. Both rails swing by the same amount: the corridor
        // turns without getting wider.
        return screenOf(
            p,
            nearX + ((farX - nearX) * t) + (bend * t * t),
            r.nearY + ((r.farY - r.nearY) * t)
        );
    }

    // Two rails converging toward the far end, in three colour bands, each
    // closed by a cross line.
    Void drawGuides(ImDrawList* dl, const Placed& p, const View& v, Float32 steer)
    {
        const Rails r = railsOf(v);
        const Array<ImU32, 3> zone = { ZONE_NEAR, ZONE_MID, ZONE_FAR };
        const Float32 w = 2.0f * uiScale;
        const Float32 swing = v.guideBend ? steer : 0.0f;
        for(Size i = 0; i < 3u; ++i)
        {
            const Float32 t0 = static_cast<Float32>(i) / 3.0f;
            const Float32 t1 = static_cast<Float32>(i + 1u) / 3.0f;
            const ImVec2 leftNear = railPoint(p, r, -1.0f, t0, swing, v.guideBendPct);
            const ImVec2 leftFar = railPoint(p, r, -1.0f, t1, swing, v.guideBendPct);
            const ImVec2 rightNear = railPoint(p, r, 1.0f, t0, swing, v.guideBendPct);
            const ImVec2 rightFar = railPoint(p, r, 1.0f, t1, swing, v.guideBendPct);
            strokeLine(dl, leftNear, leftFar, zone[i], w);
            strokeLine(dl, rightNear, rightFar, zone[i], w);
            strokeLine(dl, leftFar, rightFar, zone[i], w);
        }
    }

    // A centred, scaled copy of the picture's own outline, not a square: a
    // square would need a different fraction on each axis of a 4:3 picture.
    Void drawBox(ImDrawList* dl, const Placed& p, const View& v)
    {
        const Float32 half = pctToUnit(v.boxPct, 5, 48);
        const Float32 lo = 0.5f - half;
        const Float32 hi = 0.5f + half;
        const ImVec2 a = screenOf(p, lo, lo);
        const ImVec2 b = screenOf(p, hi, lo);
        const ImVec2 c = screenOf(p, hi, hi);
        const ImVec2 d = screenOf(p, lo, hi);
        const Float32 w = 1.6f * uiScale;
        strokeLine(dl, a, b, BOX_COL, w);
        strokeLine(dl, b, c, BOX_COL, w);
        strokeLine(dl, c, d, BOX_COL, w);
        strokeLine(dl, d, a, BOX_COL, w);
    }

    Void drawThirds(ImDrawList* dl, const Placed& p)
    {
        const Float32 w = 1.0f * uiScale;
        for(Size i = 1u; i < 3u; ++i)
        {
            const Float32 f = static_cast<Float32>(i) / 3.0f;
            strokeLine(dl, screenOf(p, f, 0.0f), screenOf(p, f, 1.0f), THIRDS_COL, w);
            strokeLine(dl, screenOf(p, 0.0f, f), screenOf(p, 1.0f, f), THIRDS_COL, w);
        }
    }

    // Arms measured in pixels off the shorter side, so the cross is square on
    // screen. The gap in the middle keeps the thing being lined up visible.
    Void drawCross(ImDrawList* dl, const Placed& p)
    {
        const ImVec2 mid = screenOf(p, 0.5f, 0.5f);
        const ImVec2 alongU = unitFrom(mid, screenOf(p, 1.0f, 0.5f));
        const ImVec2 alongV = unitFrom(mid, screenOf(p, 0.5f, 1.0f));
        const Float32 half = (p.size.x < p.size.y ? p.size.x : p.size.y) * 0.5f;
        const Float32 reach = half * 0.62f;
        const Float32 gap = half * 0.10f;
        const Float32 tick = half * 0.055f;
        const Float32 w = 1.6f * uiScale;
        Array<ImVec2, 4> arm = {};
        arm[0] = alongU;
        arm[1] = ImVec2(-alongU.x, -alongU.y);
        arm[2] = alongV;
        arm[3] = ImVec2(-alongV.x, -alongV.y);
        // Two ticks per arm. They mark nothing measurable; they let the eye judge
        // how far off centre something is.
        const Array<Float32, 2> where = { 0.45f, 0.78f };
        for(Size i = 0; i < 4u; ++i)
        {
            const ImVec2 d = arm[i];
            strokeLine(dl, stepBy(mid, d, gap), stepBy(mid, d, reach), CROSS_COL, w);
            const ImVec2 side = ImVec2(-d.y, d.x);
            for(Size k = 0; k < 2u; ++k)
            {
                const ImVec2 o = stepBy(mid, d, reach * where[k]);
                strokeLine(dl, stepBy(o, side, -tick), stepBy(o, side, tick), CROSS_COL, w);
            }
        }
    }

    // Detection boxes FOLLOW THE PICTURE, unlike every overlay above: the board
    // made them in camera pixels, so they take the same turns and flips the
    // pixels do (orient::place), or correcting the mount would move a box off
    // its tag. Grey when the detection is stale or names a different frame
    // from the one on the texture: drawn, but not as a fact about this picture.
    constexpr ImU32 TAG_COL = IM_COL32(90, 255, 120, 245);
    constexpr ImU32 TAG_OLD_COL = IM_COL32(190, 190, 190, 210);
    constexpr ImU32 TAG_FILL = IM_COL32(90, 255, 120, 55);
    constexpr ImU32 TAG_OLD_FILL = IM_COL32(190, 190, 190, 40);
    constexpr ImU32 TAG_PLATE = IM_COL32(0, 0, 0, 190);

    Void drawTags(const View& v, const ImVec2& at, const ImVec2& size, const link::TagsSeen& seen)
    {
        const bibowire::Tags& t = seen.tags;
        if(t.width == 0u || t.height == 0u)
        {
            return;
        }
        const Bool current = !seen.stale && v.haveShown && t.frameIndex == v.shownIndex;
        const ImU32 col = current ? TAG_COL : TAG_OLD_COL;
        const Float32 w = static_cast<Float32>(t.width);
        const Float32 h = static_cast<Float32>(t.height);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->PushClipRect(at, ImVec2(at.x + size.x, at.y + size.y), true);
        for(const bibowire::Tag& tag : t.tags)
        {
            Array<ImVec2, 4> pt = {};
            ImVec2 centre = ImVec2(0.0f, 0.0f);
            for(Size c = 0; c < 4u; ++c)
            {
                const Float32 u = static_cast<Float32>(tag.corners[c].xDeci) / (10.0f * w);
                const Float32 vv = static_cast<Float32>(tag.corners[c].yDeci) / (10.0f * h);
                const orient::Place p = orient::place(v.turns, v.flipX, v.flipY, u, vv);
                pt[c] = ImVec2(at.x + (p.x * size.x), at.y + (p.y * size.y));
                centre.x += pt[c].x * 0.25f;
                centre.y += pt[c].y * 0.25f;
            }
            // A tint inside, a thick dark-edged outline, a dot at each corner
            // and the id on a dark plate at half again the font size: a box
            // the picture's own detail cannot hide.
            dl->AddConvexPolyFilled(pt.data(), 4, current ? TAG_FILL : TAG_OLD_FILL);
            dl->AddPolyline(pt.data(), 4, OVERLAY_SHADE, ImDrawFlags_Closed, 7.0f * uiScale);
            dl->AddPolyline(pt.data(), 4, col, ImDrawFlags_Closed, 3.0f * uiScale);
            for(const ImVec2& corner : pt)
            {
                dl->AddCircleFilled(corner, 4.0f * uiScale, col);
            }
            // The id, and the range once the board has one.
            Array<Char, 24> label = {};
            if(tag.rangeMm > 0)
            {
                std::snprintf(
                    label.data(),
                    label.size(),
                    "%u  %d.%02dm",
                    static_cast<unsigned>(tag.id),
                    tag.rangeMm / 1000,
                    (tag.rangeMm % 1000) / 10
                );
            }
            else
            {
                std::snprintf(label.data(), label.size(), "%u", static_cast<unsigned>(tag.id));
            }
            ImFont* font = ImGui::GetFont();
            const Float32 big = ImGui::GetFontSize() * 1.5f;
            const ImVec2 extent = font->CalcTextSizeA(big, FLT_MAX, 0.0f, label.data());
            const ImVec2 text = ImVec2(centre.x - (extent.x * 0.5f), centre.y - (extent.y * 0.5f));
            const Float32 pad = 4.0f * uiScale;
            dl->AddRectFilled(
                ImVec2(text.x - pad, text.y - pad),
                ImVec2(text.x + extent.x + pad, text.y + extent.y + pad),
                TAG_PLATE,
                3.0f * uiScale
            );
            dl->AddText(font, big, text, col, label.data());
        }
        dl->PopClipRect();
    }

    Void drawOverlays(const View& v, const ImVec2& at, const ImVec2& size, Float32 steer)
    {
        if(!v.showCross && !v.showGuides && !v.showBox && !v.showThirds)
        {
            return;
        }
        Placed p;
        p.at = at;
        p.size = size;
        ImDrawList* dl = ImGui::GetWindowDrawList();
        // Clipped to the picture, so a wide spread cannot draw over the readouts.
        const ImVec2 corner = ImVec2(at.x + size.x, at.y + size.y);
        dl->PushClipRect(at, corner, true);
        // Faintest first, so the crosshair is never buried.
        if(v.showThirds)
        {
            drawThirds(dl, p);
        }
        if(v.showBox)
        {
            drawBox(dl, p, v);
        }
        if(v.showGuides)
        {
            drawGuides(dl, p, v, steer);
        }
        if(v.showCross)
        {
            drawCross(dl, p);
        }
        dl->PopClipRect();
    }

    // The overlay switches behind one button, so the row above the picture
    // stays short.
    //
    // THE BUTTON OPENS AN INLINE PANEL, NOT A POPUP. Clicking outside an ImGui
    // popup only dismisses it, so the operator's next click - on flip H or flip V
    // right after switching guides on - did nothing.
    Void drawOverlayToggle(View& v)
    {
        // "###" keeps one id while the label changes.
        if(ImGui::Button(v.overlayPanel ? "overlays -###overlays" : "overlays +###overlays"))
        {
            v.overlayPanel = !v.overlayPanel;
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("uncalibrated marks, placed by eye");
        }
        // Says whether anything is drawn without opening the panel.
        if(v.showCross || v.showGuides || v.showBox || v.showThirds)
        {
            ImGui::SameLine();
            ImGui::TextDisabled("on");
        }
    }

    Void drawOverlayPanel(View& v)
    {
        if(!v.overlayPanel)
        {
            return;
        }
        ImGui::Separator();
        ImGui::Checkbox("crosshair", &v.showCross);
        ImGui::Checkbox("reversing guides", &v.showGuides);
        ImGui::Checkbox("centre box", &v.showBox);
        ImGui::Checkbox("thirds", &v.showThirds);
        ImGui::Separator();
        ImGui::PushItemWidth(128.0f * uiScale);
        ImGui::SliderInt("centre", &v.guideCentrePct, 10, 90, "%d%%");
        ImGui::SliderInt("spread", &v.guideSpreadPct, 5, 60, "%d%%");
        ImGui::SliderInt("converge", &v.guideConvergePct, 0, 40, "%d%%");
        ImGui::SliderInt("near edge", &v.guideNearPct, 40, 100, "%d%%");
        ImGui::SliderInt("far edge", &v.guideFarPct, 5, 95, "%d%%");
        ImGui::Checkbox("bend with the wheels", &v.guideBend);
        ImGui::SliderInt("bend", &v.guideBendPct, 0, 100, "%d%%");
        ImGui::Separator();
        ImGui::SliderInt("box", &v.boxPct, 5, 48, "%d%%");
        ImGui::PopItemWidth();
    }

    // Drawn above the picture and outside every early return, so the rate is
    // reachable when the window is empty or jerky.
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
            ImGui::SetTooltip("degrees clockwise");
        }
        ImGui::SameLine();
        ImGui::Checkbox("flip H", &v.flipX);
        ImGui::SameLine();
        ImGui::Checkbox("flip V", &v.flipY);
        ImGui::SameLine();
        ImGui::Checkbox("tags", &v.showTags);
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("the board's AprilTag detections, following the picture");
        }
        ImGui::SameLine();
        drawOverlayToggle(v);
        ImGui::SetNextItemWidth(-96.0f * uiScale);
        Int32 fps = v.fps;
        const Int32 ceiling = static_cast<Int32>(bibowire::CAM_FPS_MAX);
        if(ImGui::SliderInt("rate", &fps, 0, ceiling, fps == 0 ? "board default" : "%d fps"))
        {
            v.fps = fps < 0 ? 0 : (fps > ceiling ? ceiling : fps);
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("0 = the board's default (2 fps)");
        }
        // Under the rate slider, so opening it never moves flip H and flip V.
        drawOverlayPanel(v);
    }

    // The sentence for a window with no picture in it.
    [[nodiscard]] Str whyNothing(link::Client& lk, const link::Snapshot& snap, Int64 nowMs)
    {
        if(!link::isOpen(lk))
        {
            return "not connected";
        }
        if(!snap.state.haveWelcome)
        {
            return "handshaking";
        }
        if(!snap.state.cameraSubscribed)
        {
            return "asking the board for the camera";
        }
        if(!snap.state.haveCamera)
        {
            return "waiting for the first frame";
        }
        // Frames arrived but the newest is too old to draw, however healthy the
        // link is (section 7).
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
      // Both told every frame from the window's own state, so the subscription
      // and rate cannot drift from what the operator sees. The worker re-sends
      // SUBSCRIBE only when a value changes.
      link::wantCamera(lk, v.open);
      link::wantCameraFps(lk, v.fps);
      if(!v.open)
      {
          // The texture goes back with the subscription.
          releaseTexture(v);
          return;
      }
      ImGui::SetNextWindowPos(ImVec2(380.0f * uiScale, 16.0f * uiScale), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(420.0f * uiScale, 400.0f * uiScale), ImGuiCond_FirstUseEver);
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
          // Never received or too old to draw. The texture is released so the
          // previous picture cannot be drawn.
          releaseTexture(v);
          ImGui::TextWrapped("%s", whyNothing(lk, snap, nowMs).c_str());
          // The board's own sentence, e.g. that a second pilot has the camera.
          if(snap.state.haveCameraNote)
          {
              ImGui::Separator();
              ImGui::TextWrapped("board: %s", snap.state.cameraNoteText.c_str());
          }
          ImGui::End();
          return;
      }
      // Decoded once per frame index. The byte length joins the test so a board
      // restarting its counter at 0 still shows a new picture.
      const Bool same = v.haveShown
                        && v.shownIndex == shot->frameIndex
                        && v.shownBytes == shot->bytes.size();
      if(!same)
      {
          adopt(v, *shot);
      }
      if(v.srv != nullptr && v.texW > 0 && v.texH > 0)
      {
          const Bool turned = orient::sideways(v.turns);
          const Float32 wide = static_cast<Float32>(turned ? v.texH : v.texW);
          const Float32 high = static_cast<Float32>(turned ? v.texW : v.texH);
          // Fit the width, keep the aspect ratio; the readouts sit under it.
          const Float32 avail = ImGui::GetContentRegionAvail().x;
          const Float32 shown = avail > 16.0f ? avail : 16.0f;
          const ImVec2 size = ImVec2(shown, shown * (high / wide));
          const ImTextureID id = static_cast<ImTextureID>(reinterpret_cast<UPtr>(v.srv));
          // Desaturated when stale, with its age below: section 7's middle band.
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
          // Overlays use the quad's `at` and `size` but deliberately NOT its turns
          // and flips. Guide steer is zero when the board has not reported the
          // wheels, so the guides sit straight rather than at an unreported angle.
          Float32 guideSteer = 0.0f;
          if(v.guideBend)
          {
              const Opt<link::Control> wheels = snap.state.controlState(nowMs);
              if(wheels.has_value())
              {
                  guideSteer = static_cast<Float32>(wheels->state.steerNowMilli) / 1000.0f;
              }
          }
          drawOverlays(v, at, size, guideSteer);
          if(v.showTags)
          {
              const Opt<link::TagsSeen> seen = snap.state.tagsSeen(nowMs);
              if(seen.has_value())
              {
                  drawTags(v, at, size, *seen);
              }
          }
          // The draw list does not move the cursor, so reserve the picture's
          // space or the readouts draw on top of it.
          ImGui::Dummy(size);
      }
      else
      {
          // A frame arrived and could not be shown: a different fact from no frame.
          ImGui::TextWrapped("frame %u could not be shown", shot->frameIndex);
          if(!v.decodeWhy.empty())
          {
              ImGui::TextWrapped("%s", v.decodeWhy.c_str());
          }
      }
      ImGui::Separator();
      const Str age = secondsText(shot->ageMs) + (shot->stale ? " - STALE" : "");
      readout("age", age.c_str());
      // The staleness band is shown beside STALE, so a wrong rule can be caught
      // on the running system.
      if(shot->worstGapMs > 0)
      {
          Array<Char, 96> band = {};
          std::snprintf(band.data(), band.size(), "%lld ms", shot->staleAtMs);
          readout("stale after", band.data());
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
      // Gaps are counted, never smoothed: frameIndex is monotonic, so what is
      // missing is known exactly.
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
      // The widest wait between pictures HERE beside the widest gap by the
      // BOARD's capture clock. Read with `missed` above:
      //
      //   missed frames, capture steady -> they were made and lost on the way
      //   no missed frames, capture gap -> the board stopped making them
      if(shot->worstCaptureMs > 0)
      {
          Array<Char, 96> gaps = {};
          std::snprintf(
              gaps.data(),
              gaps.size(),
              "%lld ms here, %lld ms at capture",
              shot->worstGapMs,
              shot->worstCaptureMs
          );
          readout("worst gap", gaps.data());
      }
      if(snap.state.refusedFrames > 0u)
      {
          Array<Char, 48> refused = {};
          std::snprintf(refused.data(), refused.size(), "%u", snap.state.refusedFrames);
          readout("refused", refused.data());
      }
      // The board's ring counts every frame type to every client, not only the
      // camera's, hence "board drops" rather than "camera dropped".
      const Opt<link::Board> ring = snap.state.boardState(nowMs);
      if(ring.has_value() && ring->state.txDroppedFrames > 0u)
      {
          Array<Char, 64> drops = {};
          std::snprintf(
              drops.data(),
              drops.size(),
              "%u",
              static_cast<UInt32>(ring->state.txDroppedFrames)
          );
          readout("board drops", drops.data());
      }
      if(snap.state.haveCameraNote)
      {
          ImGui::Separator();
          ImGui::TextWrapped("board: %s", snap.state.cameraNoteText.c_str());
      }
      ImGui::End();
  }
}
