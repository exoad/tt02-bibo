#include "odomview.hxx"

#include <cmath>

#include "imgui.h"

namespace odomview
{
  namespace
  {
    Float32 uiScale = 1.0f;

    // Whole units only, like tagview: "%.1f" prints a comma under some locales.
    Void drawMetres(const Char* what, Int32 mm)
    {
        const Int32 whole = std::abs(mm) / 1000;
        const Int32 cm = (std::abs(mm) % 1000) / 10;
        ImGui::Text("%s %s%d.%02d m", what, mm < 0 ? "-" : "", whole, cm);
    }

    Void drawDegrees(const Char* what, Int32 milliRad)
    {
        const Int32 deg = static_cast<Int32>(std::lround(static_cast<Float64>(milliRad) * 180.0 / 3141.59265));
        ImGui::Text("%s %d deg", what, deg);
    }
  }

  Void init(Float32 scale)
  {
      uiScale = scale > 0.0f ? scale : 1.0f;
  }

  Void update(View& v, const link::Snapshot& snap, Int64 nowMs)
  {
      const Opt<link::PoseSeen> seen = snap.state.poseSeen(nowMs);
      if(!seen.has_value() || seen->pose.valid == 0u)
      {
          return;
      }
      if(seen->pose.tMonoUs == v.lastPoseUs && v.posesSeen > 0u)
      {
          return;
      }
      // A stamp that went backward is a new run on the board: a reload of the
      // bundle, or a restart. The old trail belongs to the old frame.
      if(seen->pose.tMonoUs < v.lastPoseUs)
      {
          v.trail.clear();
      }
      v.lastPoseUs = seen->pose.tMonoUs;
      ++v.posesSeen;
      static_cast<Void>(v.trail.add(seen->pose));
  }

  Drawn toScene(const View& v, const link::Snapshot& snap, Int64 nowMs)
  {
      Drawn out;
      const Opt<link::PoseSeen> seen = snap.state.poseSeen(nowMs);
      if(!seen.has_value() || seen->pose.valid == 0u)
      {
          return out;
      }
      if(v.showTrail)
      {
          out.trail = trail::allInCarFrame(seen->pose, v.trail.points);
      }
      if(v.showMarks)
      {
          out.marks = trail::allInCarFrame(seen->pose, v.trail.marks);
      }
      return out;
  }

  Void drawWindow(View& v, const link::Snapshot& snap, Int64 nowMs)
  {
      if(!v.open)
      {
          return;
      }
      const Str title = Str("odometry###bundle:") + ID_ODOMETRY;
      ImGui::SetNextWindowSize(ImVec2(320.0f * uiScale, 0.0f), ImGuiCond_FirstUseEver);
      if(!ImGui::Begin(title.c_str(), &v.open))
      {
          ImGui::End();
          return;
      }
      ImGui::Checkbox("draw the trail", &v.showTrail);
      ImGui::SameLine();
      ImGui::Checkbox("and the marks", &v.showMarks);
      const Opt<link::PoseSeen> seen = snap.state.poseSeen(nowMs);
      const Bool have = seen.has_value() && seen->pose.valid != 0u;
      if(ImGui::Button("mark here") && have)
      {
          v.trail.mark(seen->pose);
      }
      ImGui::SameLine();
      if(ImGui::Button("clear the trail"))
      {
          v.trail.clear();
      }
      ImGui::Separator();
      if(!have)
      {
          if(snap.state.poseFrames == 0u)
          {
              ImGui::TextWrapped(
                  "no pose yet: the board reckons one from the wheel encoder while this bundle "
                  "is loaded and the Pico is talking"
              );
          }
          else
          {
              ImGui::TextWrapped("no pose for over %lld ms", static_cast<long long>(link::GONE_MS));
          }
          ImGui::End();
          return;
      }
      const bibowire::Pose& p = seen->pose;
      ImGui::Text("%s, from the wheel encoder", seen->stale ? "stale" : "live");
      drawMetres("x (right)", p.xMm);
      drawMetres("y (forward)", p.yMm);
      drawDegrees("heading", p.headingMilliRad);
      drawMetres("since load", static_cast<Int32>(std::lround(v.trail.distanceMm)));
      ImGui::TextDisabled(
          "uncertain by about %d cm and %d deg; %u points, %u marks",
          static_cast<int>(p.sigmaXyMm / 10u),
          static_cast<int>(std::lround(
              static_cast<Float64>(p.sigmaHeadingMilliRad) * 180.0 / 3141.59265
          )),
          static_cast<unsigned>(v.trail.points.size()),
          static_cast<unsigned>(v.trail.marks.size())
      );
      ImGui::TextWrapped(
          "The frame is where the car stood when the bundle was loaded. The trail is this "
          "viewer's memory; the board keeps only the newest pose."
      );
      ImGui::End();
  }
}
