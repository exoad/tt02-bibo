// The 3D view: an orbit camera and a perspective projection, drawn straight
// into ImGui's draw list. No engine and no shaders on purpose: a few hundred
// points and a box do not need a GPU pipeline.
#pragma once

#include "shared.hxx"

#include "imgui.h"

#include <cmath>

namespace scene
{
  // The world frame: X right, Y forward, Z up, metres. The car sits at the
  // origin facing +Y. The lidar-to-car transform is not established
  // (docs/hardware.md), so nothing rotates the cloud.
  struct Vec3
  {
      Float32 x;
      Float32 y;
      Float32 z;
  };

  // Which way the wheels point, inline so the suite can assert its SIGN.
  // Positive steer is right (chassis.hxx steerToUs sends a positive fraction
  // toward servoMax, cal.hxx STEER_CAL_RIGHT) and +X is the car's right, so a
  // positive angle swings toward +X; flip either and the arrow points the wrong
  // way while looking plausible.
  //
  // HEADING_MAX_RAD at full lock is a display convention, not the real steering
  // angle: no wheelbase or steering-angle map is measured, which is also why
  // this draws a heading and never a predicted path.
  constexpr Float32 HEADING_MAX_RAD = 0.52f;   // 30 degrees drawn at full lock

  [[nodiscard]] inline Vec3 headingDir(Float32 steer)
  {
      const Float32 a = steer * HEADING_MAX_RAD;
      return Vec3{ std::sin(a), std::cos(a), 0.0f };
  }

  // DISTANCE ramps the colour by range from the car.
  enum class PointColor
  {
      POINT_COLOR_UNIFORM,
      POINT_COLOR_DISTANCE
  };

  struct ViewOptions
  {
      Bool grid = true;
      Bool points = true;
      Bool car = true;
      Bool axes = true;

      // On by default, unlike the camera overlays, because it draws only when
      // the board has reported where the wheels are.
      Bool heading = true;
      Float32 pointSize = 4.0f;
      PointColor coloring = PointColor::POINT_COLOR_DISTANCE;
  };

  // Orbit camera around `target`. Up is always +Z, so the horizon never rolls.
  struct Camera
  {
      Float32 yaw = 0.0f;
      Float32 pitch = 0.0f;
      Float32 dist = 0.0f;
      Vec3 target = { 0.0f, 0.0f, 0.0f };
  };

  struct Scene
  {
      Camera cam;
      ViewOptions opt;
      Vec<Vec3> cloud;

      // Past the first freshness band the cloud is drawn desaturated and the
      // panel shows its age (docs/bibowire.md section 7). Past the second the
      // client hands over no points at all.
      Bool cloudStale = false;

      // Fractions of full lock, -1 to +1. `steerNow` is where the wheels ARE
      // (CTLSTATE, UDP), drawn solid; `steerWant` is what was ASKED FOR
      // (DECIDE, TCP), drawn as a ghost. They differ while the Pico's slew
      // limiter catches up.
      //
      // A flag each, because they arrive by different routes and blocked UDP
      // stops only one. Without its flag an arrow draws NOTHING: one defaulting
      // to centre would claim a steering angle the board never reported.
      Bool haveSteerNow = false;
      Bool haveSteerWant = false;
      Float32 steerNow = 0.0f;
      Float32 steerWant = 0.0f;
  };

  // Where on screen the view lives, in ImGui's pixel coordinates.
  struct Viewport
  {
      ImVec2 origin;
      ImVec2 size;
  };

  Void resetCamera(Camera& cam);

  // Mouse deltas in PIXELS; the rates live in scene.cxx.
  Void orbit(Camera& cam, Float32 dx, Float32 dy);
  Void pan(Camera& cam, Float32 dx, Float32 dy);
  Void zoom(Camera& cam, Float32 notches);

  Void draw(ImDrawList* dl, const Viewport& vp, const Scene& sc);
}
