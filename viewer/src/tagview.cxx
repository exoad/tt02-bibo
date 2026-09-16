#include "tagview.hxx"

#include <cmath>
#include <cstdlib>

#include "imgui.h"

namespace tagview
{
  namespace
  {
    Float32 uiScale = 1.0f;

    // Integers only: a "%.1f" here would print a comma under some locales, and
    // the detector's time in tenths of a millisecond is what an NPU port is
    // measured against.
    Void drawTenths(const Char* what, UInt32 us)
    {
        ImGui::Text("%s %u.%u ms", what, static_cast<unsigned>(us / 1000u), static_cast<unsigned>((us % 1000u) / 100u));
    }

    // The mean edge length in whole pixels: the number a person reads off
    // to calibrate (tags.hxx, defaultCameraPath).
    [[nodiscard]] Int32 sidePx(const bibowire::Tag& tag)
    {
        Float32 total = 0.0f;
        for(Size c = 0; c < 4u; ++c)
        {
            const bibowire::TagCorner& a = tag.corners[c];
            const bibowire::TagCorner& b = tag.corners[(c + 1u) % 4u];
            const Float32 dx = static_cast<Float32>(a.xDeci - b.xDeci);
            const Float32 dy = static_cast<Float32>(a.yDeci - b.yDeci);
            total += std::sqrt((dx * dx) + (dy * dy));
        }
        return static_cast<Int32>(total / 40.0f);
    }

    Void drawFound(const bibowire::Tags& t)
    {
        if(t.tags.empty())
        {
            ImGui::TextDisabled("nothing in this frame");
            return;
        }
        const Bool ranged = (t.flags & bibowire::TAGS_FLAG_CALIBRATED) != 0u;
        if(!ImGui::BeginTable("tags", 6, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
        {
            return;
        }
        ImGui::TableSetupColumn("id");
        ImGui::TableSetupColumn("centre px");
        ImGui::TableSetupColumn("side px");
        ImGui::TableSetupColumn("range");
        ImGui::TableSetupColumn("bearing");
        ImGui::TableSetupColumn("margin");
        ImGui::TableHeadersRow();
        for(const bibowire::Tag& tag : t.tags)
        {
            Int32 cx = 0;
            Int32 cy = 0;
            for(const bibowire::TagCorner& c : tag.corners)
            {
                cx += c.xDeci;
                cy += c.yDeci;
            }
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%u", static_cast<unsigned>(tag.id));
            ImGui::TableNextColumn();
            ImGui::Text("%d, %d", cx / 40, cy / 40);
            ImGui::TableNextColumn();
            ImGui::Text("%d", sidePx(tag));
            ImGui::TableNextColumn();
            if(ranged && tag.rangeMm > 0)
            {
                ImGui::Text("%d.%02d m", tag.rangeMm / 1000, (tag.rangeMm % 1000) / 10);
            }
            else
            {
                ImGui::TextDisabled("-");
            }
            ImGui::TableNextColumn();
            if(ranged)
            {
                const Int32 c = tag.bearingCdeg;
                ImGui::Text("%s%d.%d deg", c < 0 ? "-" : "+", std::abs(c) / 100, (std::abs(c) % 100) / 10);
            }
            else
            {
                ImGui::TextDisabled("-");
            }
            ImGui::TableNextColumn();
            ImGui::Text("%d", tag.marginMilli / 1000);
        }
        ImGui::EndTable();
        if(!ranged)
        {
            ImGui::TextWrapped(
                "no range: the camera is not calibrated. Hold the tag a measured D metres from "
                "the lens, read its side S above, and write ~/.config/bibo/camera.txt on the "
                "board: fx and fy = S * D / tag, cx and cy = half the frame, size WxH, tag in metres"
            );
        }
    }
  }

  Void init(Float32 scale)
  {
      uiScale = scale > 0.0f ? scale : 1.0f;
  }

  Void drawWindow(View& v, const link::Snapshot& snap, Int64 nowMs)
  {
      if(!v.open)
      {
          return;
      }
      const Str title = Str("apriltag###bundle:") + ID_APRILTAG;
      ImGui::SetNextWindowSize(ImVec2(340.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
      if(!ImGui::Begin(title.c_str(), &v.open))
      {
          ImGui::End();
          return;
      }
      if(v.cam != nullptr)
      {
          ImGui::Checkbox("draw boxes on the camera picture", &v.cam->showTags);
          if(!v.cam->open)
          {
              ImGui::TextDisabled("the Camera window is closed; open it to see them");
          }
      }
      ImGui::Separator();
      const Opt<link::TagsSeen> seen = snap.state.tagsSeen(nowMs);
      if(!seen.has_value())
      {
          if(snap.state.tagFrames == 0u)
          {
              ImGui::TextWrapped(
                  "no detections yet: the board sends them only while this bundle is loaded "
                  "and its camera is capturing"
              );
          }
          else
          {
              ImGui::TextWrapped("no detection for over %lld ms", static_cast<long long>(link::GONE_MS));
          }
          if(snap.state.haveBundleNote && snap.state.bundleNote.text.find("apriltag") != Str::npos)
          {
              ImGui::Separator();
              ImGui::TextWrapped("board: %s", snap.state.bundleNote.text.c_str());
          }
          ImGui::End();
          return;
      }
      const bibowire::Tags& t = seen->tags;
      ImGui::Text(
          "frame %u  %ux%u  %s",
          static_cast<unsigned>(t.frameIndex),
          static_cast<unsigned>(t.width),
          static_cast<unsigned>(t.height),
          seen->stale ? "stale" : "live"
      );
      drawTenths("detect", t.detectUs);
      ImGui::SameLine();
      ImGui::TextDisabled("%u frames", static_cast<unsigned>(snap.state.tagFrames));
      drawFound(t);
      ImGui::End();
  }
}
