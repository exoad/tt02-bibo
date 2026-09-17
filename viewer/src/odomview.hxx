// odomview - the odometry bundle's window: where the board reckons the car is,
// how far it has gone, and the trail and marks the 3D view draws from it.
//
// `odomview`, like `tagview`: main.cxx holds an `odomview::View odom;`, calls
// update() once a frame so every new POSE lands on the trail whether or not
// the window is open, and hands the trail to the scene in the car's frame.
//
// Opened by bundleview on the load edge of net.exoad.tt02bibo.odometry, and
// keyed on that id past ### so its saved position survives a rename. The
// trail is the viewer's memory, not the board's: the board publishes one
// pose at a time, and a reconnect starts a new trail.
#pragma once

#include "shared.hxx"

#include "link.hxx"
#include "trail.hxx"

namespace odomview
{
  constexpr CharSeq ID_ODOMETRY = "net.exoad.tt02bibo.odometry";

  struct View
  {
      Bool open = false;
      Bool showTrail = true;
      Bool showMarks = true;
      trail::Trail trail;
      UInt64 lastPoseUs = 0;   // the newest pose on the trail, by its stamp
      UInt32 resetsSeen = 0;   // EVENT_CODE_ODOM_FRAME arrivals the trail has answered
      UInt32 posesSeen = 0;
  };

  // The DPI multiplier the layout uses. Called once, after ImGui exists.
  Void init(Float32 scale);

  // Every frame, window or not: a POSE newer than the last goes on the trail.
  Void update(View& v, const link::Snapshot& snap, Int64 nowMs);

  // The trail and the marks as the scene draws them: in the car's frame at
  // the newest pose, empty when there is no pose or the switch is off.
  struct Drawn
  {
      Vec<trail::Point> trail;
      Vec<trail::Point> marks;
  };

  [[nodiscard]] Drawn toScene(const View& v, const link::Snapshot& snap, Int64 nowMs);

  // One frame: the window when open, nothing otherwise.
  Void drawWindow(View& v, const link::Snapshot& snap, Int64 nowMs);
}
