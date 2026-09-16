// The apriltag bundle's window: what the board's detector found in the newest
// camera frame, and the switch for drawing it over the Camera pane.
//
// `tagview`, like `camview`: main.cxx holds a `tagview::View tags;`.
//
// Opened by bundleview on the load edge of net.exoad.tt02bibo.apriltag, and
// keyed on that id past ### so its saved position survives a rename. Read-only
// about the car: nothing here reaches the board, because a detection is a fact
// the board publishes and what the car does with one is a later bundle.
#pragma once

#include "shared.hxx"

#include "camera.hxx"
#include "link.hxx"

namespace tagview
{
  // The id this window belongs to, the wire's stable identity for the bundle.
  constexpr CharSeq ID_APRILTAG = "net.exoad.tt02bibo.apriltag";

  struct View
  {
      Bool open = false;

      // The Camera pane, whose showTags this window's checkbox sets. Borrowed
      // from main.cxx, set once before the first frame.
      camview::View* cam = nullptr;
  };

  // The DPI multiplier the layout uses. Called once, after ImGui exists.
  Void init(Float32 scale);

  // One frame: the window when open, nothing otherwise.
  Void drawWindow(View& v, const link::Snapshot& snap, Int64 nowMs);
}
