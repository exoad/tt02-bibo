// trail - where the car has been, from POSE frames, and the marks a person
// drops along the way. PURE: poses in, points in the CAR'S frame out, so the
// 3D view (which is car-centred, like the cloud) draws the trail streaming out
// behind the car and the marks sliding past as it drives.
//
// The world frame is the board's (firmware/pilot/src/odom.hxx): X right, Y
// forward as the car stood when its odometry bundle was loaded, heading from
// +Y counter-clockwise. A trail point is kept when the car has moved a
// little since the last one, so a car standing still adds nothing, and the
// oldest go when the trail is full.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"

namespace trail
{
  // Millimetres of travel between kept points, and the most kept.
  constexpr Int32 MIN_STEP_MM = 20;
  constexpr Size MAX_POINTS = 4000;

  struct Point
  {
      Float32 x = 0.0f;   // metres, car frame: X right, Y forward
      Float32 y = 0.0f;
  };

  struct Trail
  {
      Vec<bibowire::Pose> points;   // world millimetres, oldest first
      Vec<bibowire::Pose> marks;
      Float64 distanceMm = 0.0;     // along the kept points

      // A pose as the board published it. Kept when it moved MIN_STEP_MM
      // from the last kept one, or is the first. False when it was not.
      Bool add(const bibowire::Pose& p);

      // A mark at a pose, usually the newest.
      Void mark(const bibowire::Pose& p);

      Void clear();
  };

  // The world point `p` seen from the car at `now`: metres, X right, Y forward.
  [[nodiscard]] Point inCarFrame(const bibowire::Pose& now, const bibowire::Pose& p);

  // Every point of `list`, in the car's frame at `now`, oldest first.
  using Poses = Vec<bibowire::Pose>;

  [[nodiscard]] Vec<Point> allInCarFrame(const bibowire::Pose& now, const Poses& list);
}
