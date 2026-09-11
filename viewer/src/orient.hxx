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

  // One point on the DISPLAYED rectangle, normalised: 0,0 is its top-left
  // corner and 1,1 its bottom-right, x to the right and y DOWN. Multiply by the
  // rectangle's size on screen to get pixels.
  //
  // A separate type from Uv on purpose. They are both a pair of floats and they
  // mean opposite things - a Uv is a place in the SOURCE picture, a Pt is a
  // place on the SCREEN - and the one mistake this file is here to catch is
  // using one where the other belongs.
  struct Pt
  {
      Float32 x = 0.0f;
      Float32 y = 0.0f;
  };

  // Where a point of the SENSOR FRAME ends up on the displayed rectangle, under
  // the same turns and flips cornerUvs() applies to the picture itself.
  //
  // ---------------------------------------------------------------------------
  // WHY AN OVERLAY CANNOT BE DRAWN IN SCREEN SPACE
  //
  // camera.cxx draws the picture with AddImageQuad onto four corners that are
  // ALWAYS axis-aligned - the rotation lives entirely in the uvs. So a guide
  // line drawn straight onto the window would sit still while the picture
  // turned underneath it, and at 90 degrees it would be describing a part of
  // the room it does not point at.
  //
  // Reversing guides are a claim about where the CAR will go, so they have to
  // turn with the picture's contents. Authoring them here - in the frame, 0..1
  // across and down - and mapping them through the same transform is what makes
  // that true at every angle rather than at the one the author happened to test.
  //
  // This is cornerUvs read in the other direction, and the suite holds the two
  // to each other: feeding a corner uv in here must give back the display
  // corner that samples it. Neither can drift without the other noticing.
  //
  // Out-of-range and negative turns fold, exactly as they do above.
  [[nodiscard]] Pt displayFromImage(Int32 turns, Bool flipX, Bool flipY, Float32 u, Float32 v);

}
