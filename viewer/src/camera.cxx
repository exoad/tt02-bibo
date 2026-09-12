#include "shared.hxx"

#include <cmath>
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

    // ---- alignment overlays -------------------------------------------------
    //
    // EVERY POINT BELOW IS AUTHORED IN THE SENSOR FRAME - 0..1 across and down
    // the picture - and mapped to the screen through orient::displayFromImage.
    //
    // That is not a style choice. The picture is drawn with AddImageQuad onto
    // four corners that are ALWAYS axis-aligned; the rotation and the flips
    // live entirely in the uvs. So an overlay drawn straight onto the window
    // would sit still while the picture turned underneath it, and a reversing
    // guide that does not turn with the picture is describing a part of the
    // room it no longer points at. The arithmetic lives in orient.cxx, which
    // names no ImGui or D3D type, so viewer/tests can hold it to an answer at
    // all four turns and both flips.
    //
    // AND NOTHING HERE IS CALIBRATED. See the View, which says it at length;
    // the short form is that no band is ever labelled with a distance, because
    // there is no camera calibration and no measured camera-to-car transform in
    // this project to derive one from.
    constexpr ImU32 OVERLAY_SHADE = IM_COL32(0, 0, 0, 165);
    constexpr ImU32 CROSS_COL = IM_COL32(255, 255, 255, 225);
    constexpr ImU32 BOX_COL = IM_COL32(120, 210, 255, 225);
    constexpr ImU32 THIRDS_COL = IM_COL32(235, 235, 235, 105);

    // Near, middle and far, in the colours a reversing camera uses. COLOURS
    // ONLY: the operator dragged these bands to where they are, so they are
    // zones placed by eye and not distances anything derived.
    constexpr ImU32 ZONE_NEAR = IM_COL32(244, 76, 62, 235);
    constexpr ImU32 ZONE_MID = IM_COL32(255, 196, 46, 235);
    constexpr ImU32 ZONE_FAR = IM_COL32(96, 226, 130, 235);

    // Where the picture landed on screen and how it is turned - everything an
    // overlay needs to put a point of the sensor frame in the right pixel.
    struct Placed
    {
        ImVec2 at;
        ImVec2 size;
        Int32 turns = 0;
        Bool flipX = false;
        Bool flipY = false;
    };

    // The guide trapezoid as fractions of the frame, clamped, rather than the
    // raw percentages the sliders hold.
    struct Rails
    {
        Float32 centre = 0.5f;
        Float32 spread = 0.42f;
        Float32 converge = 0.12f;
        Float32 nearY = 1.0f;
        Float32 farY = 0.45f;
    };

    // A LIGHT STROKE OVER A DARK ONE, every line. A single-colour overlay
    // vanishes into a bright sky or a dark garage depending on which colour was
    // picked, and what is under it is whatever the car happens to be looking at.
    Void strokeLine(ImDrawList* dl, const ImVec2& a, const ImVec2& b, ImU32 col, Float32 w)
    {
        dl->AddLine(a, b, OVERLAY_SHADE, w + (2.0f * uiScale));
        dl->AddLine(a, b, col, w);
    }

    [[nodiscard]] ImVec2 screenOf(const Placed& p, Float32 iu, Float32 iv)
    {
        const orient::Pt d = orient::displayFromImage(p.turns, p.flipX, p.flipY, iu, iv);
        return ImVec2(p.at.x + (d.x * p.size.x), p.at.y + (d.y * p.size.y));
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

    // pctToUnit and bendAt moved to camera.hxx, at namespace scope, so
    // test_link.cxx can assert the bend's SIGN without linking a translation
    // unit that names ImGui. Every call below resolves to them unchanged.

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

        // THE SWING GOES HERE, not in drawGuides, and that is the whole point:
        // this is the one place a guide point becomes an image coordinate, so a
        // bend applied here rides orient::displayFromImage with everything else
        // and a rotated or flipped picture carries its guides correctly. Bending
        // the drawn screen points instead would put the curve in window space,
        // where it would sit still while the picture turned underneath it.
        //
        // BOTH RAILS BY THE SAME AMOUNT, so the corridor swings rather than
        // deforming - the car's path does not get wider because it is turning.
        //
        // t*t, not t: a reversing camera's guides barely move at the bumper and
        // sweep hardest at the far end, because that is where a given steering
        // angle has had the most distance to act.
        return screenOf(p, nearX + ((farX - nearX) * t) + (bend * t * t), r.nearY + ((r.farY - r.nearY) * t));
    }

    // THE REVERSING GUIDES: two rails converging toward a far end the operator
    // chooses, in three colour bands, each closed by a cross line.
    Void drawGuides(ImDrawList* dl, const Placed& p, const View& v, Float32 steer)
    {
        const Rails r = railsOf(v);
        const Array<ImU32, 3> zone = { ZONE_NEAR, ZONE_MID, ZONE_FAR };
        const Float32 w = 2.0f * uiScale;

        // The geometry - and its sign - is in camera.hxx where the suite can
        // assert it. This is only whether the operator asked for it.
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

    // A CENTRED FRACTION OF THE FRAME, deliberately not a square. It is a
    // scaled copy of the picture's own outline, which is what makes it useful
    // for centring the car on something. A square would need a different
    // fraction on each axis of a 4:3 picture, and which fraction it was would
    // then be a number nobody could read back off the screen.
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

    // THE ARMS FOLLOW THE SENSOR'S AXES AND ARE MEASURED IN PIXELS. Built from
    // frame fractions instead, a crosshair is a third longer across than down
    // on a 4:3 picture, which reads as a bug; measured off the shorter side it
    // is square on screen and still turns with the picture.
    //
    // The gap in the middle is the point of the whole thing: a solid cross
    // hides the one thing being lined up.
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

        // Two ticks along each arm. They mark NOTHING MEASURABLE - there is no
        // calibration here - they are there so the eye can judge how far off
        // centre something is rather than only whether it is off.
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

    Void drawOverlays(const View& v, const ImVec2& at, const ImVec2& size, Float32 steer)
    {
        if(!v.showCross && !v.showGuides && !v.showBox && !v.showThirds)
        {
            return;
        }

        Placed p;
        p.at = at;
        p.size = size;
        p.turns = v.turns;
        p.flipX = v.flipX;
        p.flipY = v.flipY;

        ImDrawList* dl = ImGui::GetWindowDrawList();

        // CLIPPED TO THE PICTURE. A spread dragged wide would otherwise run out
        // over the readouts below, and a guide drawn outside the picture is
        // marking something the camera cannot see.
        const ImVec2 corner = ImVec2(at.x + size.x, at.y + size.y);
        dl->PushClipRect(at, corner, true);

        // Faintest first, so the crosshair is never the thing that gets buried.
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

    // The switches, behind one button. Four toggles and six numbers do not fit
    // beside the rotate combo, and the row above the picture is what an operator
    // reaches for when the window is empty - it stays short.
    Void drawOverlayMenu(View& v)
    {
        if(ImGui::Button("overlays"))
        {
            ImGui::OpenPopup("overlays");
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "crosshair, reversing guides and boxes, for lining the car up.\n"
                "NONE OF IT IS CALIBRATED: there is no camera calibration and\n"
                "no measured camera-to-car transform in this project, so these\n"
                "lines carry no distance and mark no real width. They are marks\n"
                "you place by eye and then read the same way every time."
            );
        }

        if(ImGui::BeginPopup("overlays"))
        {
            ImGui::TextUnformatted("uncalibrated - these lines are not measurements");
            ImGui::Separator();
            ImGui::Checkbox("crosshair", &v.showCross);
            ImGui::Checkbox("reversing guides", &v.showGuides);
            ImGui::Checkbox("centre box", &v.showBox);
            ImGui::Checkbox("thirds", &v.showThirds);
            ImGui::Separator();
            ImGui::TextUnformatted("guides - drag until they match what you see");

            ImGui::PushItemWidth(128.0f * uiScale);
            ImGui::SliderInt("centre", &v.guideCentrePct, 10, 90, "%d%%");
            ImGui::SliderInt("spread", &v.guideSpreadPct, 5, 60, "%d%%");
            ImGui::SliderInt("converge", &v.guideConvergePct, 0, 40, "%d%%");
            ImGui::SliderInt("near edge", &v.guideNearPct, 40, 100, "%d%%");
            ImGui::SliderInt("far edge", &v.guideFarPct, 5, 95, "%d%%");

            ImGui::Checkbox("bend with the wheels", &v.guideBend);
            if(ImGui::IsItemHovered())
            {
                ImGui::SetTooltip(
                    "The guides sweep with the steering, the way a reversing\n"
                    "camera's do.\n\n"
                    "The sweep is NOT calculated from the car. There is no\n"
                    "wheelbase, no steering-angle map and no lens calibration\n"
                    "in this project, so this is a shape you tune until it\n"
                    "matches what the car actually does - and it still carries\n"
                    "no distance and marks no real width.\n\n"
                    "It follows where the wheels ARE, not what was asked for."
                );
            }
            ImGui::SliderInt("bend", &v.guideBendPct, 0, 100, "%d%%");

            ImGui::Separator();
            ImGui::SliderInt("box", &v.boxPct, 5, 48, "%d%%");
            ImGui::PopItemWidth();

            ImGui::EndPopup();
        }

        // So the row says whether anything is being drawn without the popup
        // having to be opened to find out.
        if(v.showCross || v.showGuides || v.showBox || v.showThirds)
        {
            ImGui::SameLine();
            ImGui::TextDisabled("on");
        }
    }

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
        ImGui::SameLine();
        drawOverlayMenu(v);

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

          // OVER THE PICTURE, FROM THE SAME PLACEMENT. Same `at` and `size` as
          // the quad above and the same turns and flips, so the guides move
          // with the picture's contents rather than with the window.
          // WHERE THE WHEELS ARE, for guides that bend with them - and zero when
          // the board has not said, so they sit straight rather than sweeping to
          // an angle nobody reported. steerNowMilli and not the commanded value:
          // the slew limiter means a request takes about a second to become an
          // angle, and guides drawn from the request would show a turn the car
          // has not made.
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

      // ---- WHERE A DROPOUT HAPPENED ------------------------------------------
      //
      // The two clocks side by side, which is the whole diagnosis. `worst gap`
      // is the widest wait THIS VIEWER had between pictures; `worst capture` is
      // the widest gap between two frames by the BOARD'S own clock. Read with
      // the `missed` row above:
      //
      //   missed frames, capture steady -> they were made and lost on the way
      //   no missed frames, capture gap -> the board stopped making them
      //
      // Every one of these numbers was already being computed and thrown away
      // at this boundary, which is why a dropout used to need a log tailed on
      // somebody else's machine to explain.
      if(shot->worstCaptureMs > 0)
      {
          Array<Char, 96> gaps = {};
          std::snprintf(
              gaps.data(),
              gaps.size(),
              "%lld ms waiting, %lld ms between captures",
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

      // THE BOARD'S OWN RING, and named for what it actually counts: this is
      // every frame type to every client, not the camera's alone. Labelling it
      // "camera dropped" would read as precise and be wrong - but a number that
      // climbs while pictures vanish still says the ring is where they went.
      const Opt<link::Board> ring = snap.state.boardState(nowMs);
      if(ring.has_value() && ring->state.txDroppedFrames > 0u)
      {
          Array<Char, 64> drops = {};
          std::snprintf(
              drops.data(),
              drops.size(),
              "%u (board, all frame types)",
              static_cast<UInt32>(ring->state.txDroppedFrames)
          );
          readout("dropped at the board", drops.data());
      }

      if(snap.state.haveCameraNote)
      {
          ImGui::Separator();
          ImGui::TextWrapped("board: %s", snap.state.cameraNoteText.c_str());
      }

      ImGui::End();
  }

}
