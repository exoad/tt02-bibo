// The Bundles window: what the board can run, what is loaded, and one window
// per loaded bundle.
//
// `bundleview`, not `bundle`: main.cxx holds a `bundleview::View bundles;`.
//
// THE LIST IS THE BOARD'S. Every row comes from the BUNDLE frames the board
// published (link::Session::bundles); this window invents nothing, greys a row
// the board says is not ready rather than hiding it, and names a bundle to the
// board by INDEX and GENERATION, never by string - docs/bundles.md section 5.
// A list the board has since replaced is refused by the board, and the refusal
// is shown here from its CMDACK.
//
// WHICH WINDOW AN ID GETS IS A LOOKUP. wasd's window is the Drive pane and
// trim's is the Trim pane - those already exist, reach the car through rules
// that were reviewed once, and are not rewritten here; loading either opens its
// pane. Any other id gets a read-only window keyed on the id
// ("forward###bundle:net.exoad.tt02bibo.forward"), so ImGui keeps its saved
// position across a rename of the name. The fallback stays read-only on
// purpose: a generic "a bundle declares its controls" path would be a second
// way around the deadman and the arm epoch.
#pragma once

#include "shared.hxx"

#include "drive.hxx"
#include "link.hxx"
#include "tagview.hxx"
#include "trim.hxx"

namespace bundleview
{
  struct View
  {
      Bool open = false;   // the master window

      // The panes that ARE two of the bundles' windows. Borrowed from main.cxx,
      // set once before the first frame, never null after.
      trimview::View* trim = nullptr;
      driveview::View* drive = nullptr;

      // apriltag's window, the third with a pane of its own.
      tagview::View* tags = nullptr;

      // Which ids were loaded last frame, so a load EDGE opens a window once
      // and a closed window stays closed until the bundle is loaded again.
      Set<Str> wasLoaded;
      Set<Str> closed;
  };

  // The DPI multiplier the layout uses. Called once, after ImGui exists.
  Void init(Float32 scale);

  // One frame: the master window when open, and a window per loaded bundle.
  // Call every frame whether or not the master is open, or a load edge could
  // be missed and a pane never open.
  Void drawWindow(View& v, link::Client& lk, const link::Snapshot& snap, Int64 nowMs);
}
