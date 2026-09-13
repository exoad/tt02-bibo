// Camera picture rotation and flips as arithmetic, with no graphics device, so
// viewer/tests can check it.
//
// ImGui::Image takes uv0 and uv1 as opposite corners, which can mirror an axis
// but never transpose one, and a quarter turn is a transpose. So the picture is
// drawn as a quad with four independent corner coordinates, computed here.
#pragma once

#include "shared.hxx"

namespace orient
{
  // One corner of the source picture, in texture coordinates. Not ImVec2, so
  // tests of this module need no imgui.h.
  struct Uv
  {
      Float32 u = 0.0f;
      Float32 v = 0.0f;
  };

  // True at 90 and 270 degrees, where the displayed width and height swap and
  // the fit must use the turned dimensions or the aspect ratio is wrong.
  [[nodiscard]] Bool sideways(Int32 turns);

  // The source corners that `turns` quarter-turns CLOCKWISE plus the flips put
  // at the destination's top-left, top-right, bottom-right and bottom-left, in
  // that order. Any Int32 is accepted, negative included, folded modulo 4.
  [[nodiscard]] Array<Uv, 4> cornerUvs(Int32 turns, Bool flipX, Bool flipY);
}
