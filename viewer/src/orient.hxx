// How the camera picture is turned, as arithmetic - with no graphics device in
// it.
//
// ---------------------------------------------------------------------------
// WHY THIS IS ITS OWN MODULE
//
// The same seam jpeg.cxx exists for, and for the same reason. camera.cxx owns
// a D3D11 texture, an ImGui window and <windows.h>; none of those exist in a
// test process, so anything that lives there can only be checked by looking at
// it on a screen.
//
// The 90-degree case is exactly the kind of mapping that ships silently wrong.
// ImGui::Image CANNOT express it: it takes uv0 and uv1 as OPPOSITE CORNERS of
// an axis-aligned rectangle, which can mirror an axis but can never transpose
// one axis onto the other, and a quarter turn is precisely a transpose. So the
// picture is drawn as a quad with four independent corners, and which corner
// receives which coordinate is arithmetic nobody can eyeball. It lives here so
// viewer/tests can hold it to an answer.
#pragma once

#include "shared.hxx"

namespace orient
{

  // One corner of the source picture, in texture coordinates.
  //
  // Deliberately NOT ImVec2. Naming an ImGui type here would put imgui.h on the
  // include path of everything that tests this, which is the dependency this
  // module exists to avoid; camera.cxx converts at the one point of use.
  struct Uv
  {
      Float32 u = 0.0f;
      Float32 v = 0.0f;
  };

  // Does this rotation swap the displayed width and height? True at 90 and 270.
  //
  // A 640x480 picture shown on its side is 480x640, and a fit computed from the
  // unturned dimensions would stretch it into a rectangle of the wrong shape -
  // quietly changing the aspect ratio, which on a camera used for judging
  // clearance is a lie about the room rather than a cosmetic fault.
  [[nodiscard]] Bool sideways(Int32 turns);

  // The four source corners that `turns` quarter-turns CLOCKWISE and a pair of
  // flips put at the destination's top-left, top-right, bottom-right and
  // bottom-left, in that order.
  //
  // Out-of-range and NEGATIVE turns are folded rather than refused: this is fed
  // by a combo box today, and a function that only works for 0..3 is one that
  // breaks the first time somebody wires a "rotate left" button to it.
  [[nodiscard]] Array<Uv, 4> cornerUvs(Int32 turns, Bool flipX, Bool flipY);

}
