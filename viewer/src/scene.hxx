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

#include <cmath>

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

  // ---------------------------------------------------------------------------
  // WHICH WAY THE WHEELS POINT - inline so the suite can assert its SIGN.
  //
  // Two facts, both written down elsewhere and neither inferred: +X is the
  // car's right (the frame note above, and link.cxx repeats it where it builds
  // the cloud), and positive steer is right (chassis.hxx's steerToUs sends a
  // positive fraction toward servoMax, which cal.hxx names STEER_CAL_RIGHT).
  // Flip either and this still compiles, still looks entirely plausible, and
  // draws a confident arrow the wrong way - which is worse than drawing none.
  // orient.cxx exists for the same species of error one dimension up.
  //
  // THE ANGLE IS A DISPLAY CONVENTION. Full lock draws as HEADING_MAX_RAD; it
  // is not the car's real steering angle, because no wheelbase and no
  // steering-angle map have ever been measured for this car. A predicted PATH
  // would need both, which is why this is a heading and not an arc.
  constexpr Float32 HEADING_MAX_RAD = 0.52f;   // 30 degrees drawn at full lock

  [[nodiscard]] inline Vec3 headingDir(Float32 steer)
  {
      const Float32 a = steer * HEADING_MAX_RAD;
      return Vec3{ std::sin(a), std::cos(a), 0.0f };
  }

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

      // The heading arrow. On by default, unlike the camera's overlays, because
      // it draws only when the board has actually said where the wheels are -
      // so it is absent rather than misleading when there is nothing to show.
      Bool heading = true;
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

      // The cloud is past its first freshness band and is drawn DESATURATED,
      // with its age printed in the panel beside it (docs/bibowire.md section
      // 7). It is not an option a person sets - it is a fact about the data, and
      // the renderer is told rather than left to work it out. A revolution older
      // still is not flagged, it is ABSENT: the client hands over no points at
      // all, because absence is the only rendering a person cannot misread.
      Bool cloudStale = false;

      // ---- where the car is pointed ------------------------------------------
      //
      // Fractions of full lock, -1 to +1. TWO numbers because they are two
      // different facts, and the difference between them is the thing the trim
      // pane tunes: the Pico's slew limiter means a commanded angle takes about
      // a second to become a real one, so an arrow drawn from the command alone
      // would show a turn the car has not made yet. `steerNow` is where the
      // wheels ARE and is drawn solid; `steerWant` is what was ASKED FOR and is
      // drawn as a ghost.
      //
      // A FLAG EACH, and not one between them. They arrive by different routes -
      // steerNow from CTLSTATE on UDP, steerWant from DECIDE on TCP - so on a
      // network that blocks UDP the first never comes at all while the second
      // keeps arriving. Sharing a flag would draw the solid arrow straight ahead
      // and call that "the wheels", when the truth is that the board has never
      // said. Told nothing, each of these draws NOTHING: an arrow defaulting to
      // centre is a confident claim about the one thing an operator steers by,
      // and that is this repo's named recurring failure with a wheel on it.
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

  // Mouse deltas in PIXELS. The rates that turn them into radians and metres
  // live in scene.cxx, so a caller never has to know one.
  Void orbit(Camera& cam, Float32 dx, Float32 dy);
  Void pan(Camera& cam, Float32 dx, Float32 dy);
  Void zoom(Camera& cam, Float32 notches);

  Void draw(ImDrawList* dl, const Viewport& vp, const Scene& sc);

}
