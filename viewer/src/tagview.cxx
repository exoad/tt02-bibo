#include "tagview.hxx"

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

    Void drawFound(const bibowire::Tags& t)
    {
        if(t.tags.empty())
        {
            ImGui::TextDisabled("nothing in this frame");
            return;
        }
        if(!ImGui::BeginTable("tags", 4, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
        {
            return;
        }
        ImGui::TableSetupColumn("id");
        ImGui::TableSetupColumn("centre px");
        ImGui::TableSetupColumn("margin");
        ImGui::TableSetupColumn("hamming");
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
            ImGui::Text("%d", tag.marginMilli / 1000);
            ImGui::TableNextColumn();
            ImGui::Text("%u", static_cast<unsigned>(tag.hamming));
        }
        ImGui::EndTable();
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
