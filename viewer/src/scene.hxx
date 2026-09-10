// The 3D view: a camera, a perspective projection, and everything drawn into
// ImGui's own draw list.
//
// There is no engine under this and there are no shaders in it. A point is
// turned into a screen position by subtracting the eye, taking three dot
// products and dividing by depth, and the result is handed to ImDrawList as a
// line or a disc. That is the whole renderer, and it is deliberate: the thing
// this program shows is a few hundred points and a box, and a GPU pipeline for
// that would be more code than the program.
#pragma once

#include "shared.hxx"

#include "imgui.h"

namespace scene
{

  // The world frame, and the only one this program has: X right, Y forward,
  // Z up, metres. The car sits at the origin pointing along +Y.
  //
  // +Y is where the lidar's 0 degrees will point once there is a measured
  // mounting offset. docs/conventions.md is explicit that the transform is NOT
  // established, so nothing here rotates the cloud to pretend otherwise.
  struct Vec3
  {
      Float32 x;
      Float32 y;
      Float32 z;
  };

  // How the cloud is tinted. UNIFORM is one colour for every point; DISTANCE
  // ramps by range from the car, which is what makes a near wall separate from
  // a far one when the view is flattened out.
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
      Float32 pointSize = 4.0f;
      PointColor coloring = PointColor::POINT_COLOR_DISTANCE;
  };

  // Orbit camera, spherical around `target`. Up is always +Z, so the horizon
  // never rolls and there is no orientation to get lost in - the one thing a
  // free camera buys you is the one thing nobody wants while looking at a car.
  struct Camera
  {
      Float32 yaw = 0.0f;
      Float32 pitch = 0.0f;
      Float32 dist = 0.0f;
      Vec3 target = { 0.0f, 0.0f, 0.0f };
  };

  // Everything the view draws, in one place, so `draw` takes three parameters
  // instead of seven.
  struct Scene
  {
      Camera cam;
      ViewOptions opt;
      Vec<Vec3> cloud;
  };

  // Where on screen the view lives, in ImGui's pixel coordinates.
  struct Viewport
  {
      ImVec2 origin;
      ImVec2 size;
  };

  Void resetCamera(Camera& cam);

  // Mouse deltas in PIXELS. The rates that turn them into radians and metres
  // live in scene.cxx, so a caller never has to know one.
  Void orbit(Camera& cam, Float32 dx, Float32 dy);
  Void pan(Camera& cam, Float32 dx, Float32 dy);
  Void zoom(Camera& cam, Float32 notches);

  // THE STAND-IN SCAN. The real one replaces this call and nothing else - see
  // the comment on the definition in scene.cxx.
  Void fillSyntheticCloud(Vec<Vec3>& out, Float64 seconds);

  Void draw(ImDrawList* dl, const Viewport& vp, const Scene& sc);

}
