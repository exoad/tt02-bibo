#include "scene.hxx"

#include <algorithm>
#include <cmath>

namespace scene
{

  constexpr Float32 PI = 3.14159265358979f;
  constexpr Float32 TWO_PI = PI * 2.0f;

  // The clip plane, in metres. Anything nearer than this is behind the glass.
  constexpr Float32 NEAR_PLANE = 0.05f;

  // What `project` calls "in front of the eye". Deliberately far smaller than
  // NEAR_PLANE: `segment` clips an endpoint ONTO the near plane, and rounding
  // can leave it a hair inside. Testing against NEAR_PLANE there would reject
  // the point the clip had just built and the segment would vanish.
  constexpr Float32 MIN_DEPTH = 1.0e-4f;

  constexpr Float32 FOV_Y = 1.0472f;          // 60 degrees, vertical

  constexpr Float32 ORBIT_RATE = 0.0075f;     // radians per pixel
  constexpr Float32 PAN_RATE = 0.0016f;       // metres per pixel, per metre out
  constexpr Float32 ZOOM_STEP = 1.15f;        // per wheel notch
  constexpr Float32 DIST_MIN = 0.8f;
  constexpr Float32 DIST_MAX = 40.0f;

  // Just short of the pole. AT the pole the view axis and +Z are parallel, the
  // right vector is the cross product of two parallel vectors, and the basis
  // collapses to zero - the picture does not degrade, it disappears.
  constexpr Float32 PITCH_LIMIT = 1.5533f;    // 89 degrees

  constexpr Int32 GRID_HALF = 10;             // metres each way, 1 m spacing
  constexpr Float32 AXIS_LEN = 0.6f;

  // The TT-02 with its shell on, from docs/conventions.md: ~440 x 190 x 130 mm.
  constexpr Float32 CAR_HALF_WIDTH = 0.095f;
  constexpr Float32 CAR_HALF_LENGTH = 0.22f;
  constexpr Float32 CAR_FLOOR = 0.02f;
  constexpr Float32 CAR_ROOF = 0.15f;

  constexpr Float32 RAMP_MAX = 6.0f;          // metres at the far end of the ramp

  // ---- small vector helpers -----------------------------------------------

  static Vec3 sub(const Vec3& a, const Vec3& b)
  {
      return Vec3{ a.x - b.x, a.y - b.y, a.z - b.z };
  }

  static Float32 dot(const Vec3& a, const Vec3& b)
  {
      return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
  }

  static Vec3 cross(const Vec3& a, const Vec3& b)
  {
      return Vec3{ (a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z), (a.x * b.y) - (a.y * b.x) };
  }

  static Vec3 normalize(const Vec3& v)
  {
      const Float32 len = std::sqrt(dot(v, v));
      if(len < 1.0e-8f)
      {
          return Vec3{ 0.0f, 0.0f, 1.0f };
      }
      return Vec3{ v.x / len, v.y / len, v.z / len };
  }

  static Vec3 lerp(const Vec3& a, const Vec3& b, Float32 t)
  {
      return Vec3{ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
  }

  static ImU32 rgbaOf(Float32 r, Float32 g, Float32 b, Float32 a)
  {
      return IM_COL32(
          static_cast<int>(r * 255.0f),
          static_cast<int>(g * 255.0f),
          static_cast<int>(b * 255.0f),
          static_cast<int>(a * 255.0f)
      );
  }

  // ---- the projection ------------------------------------------------------

  // The camera resolved into something a projection can use: an eye position,
  // an orthonormal basis, and the focal length in PIXELS.
  struct Basis
  {
      Vec3 eye;
      Vec3 right;
      Vec3 up;
      Vec3 fwd;
      Float32 focal;
      ImVec2 center;
  };

  struct Projected
  {
      ImVec2 at;
      Float32 depth;      // metres along the view axis
  };

  static Basis basisFor(const Camera& cam, const Viewport& vp)
  {
      const Float32 cp = std::cos(cam.pitch);
      const Float32 sp = std::sin(cam.pitch);
      const Float32 cy = std::cos(cam.yaw);
      const Float32 sy = std::sin(cam.yaw);

      // Target to eye. Pitch is elevation and yaw swings around the car,
      // because up is pinned to +Z rather than carried by the camera.
      const Vec3 off = { cp * sy * cam.dist, -cp * cy * cam.dist, sp * cam.dist };

      Basis b;
      b.eye = Vec3{ cam.target.x + off.x, cam.target.y + off.y, cam.target.z + off.z };
      b.fwd = normalize(Vec3{ -off.x, -off.y, -off.z });
      b.right = normalize(cross(b.fwd, Vec3{ 0.0f, 0.0f, 1.0f }));
      b.up = cross(b.right, b.fwd);

      // From the viewport HEIGHT, not a constant: this is what keeps the scene
      // the same shape when the window is resized rather than letting the
      // apparent field of view breathe with it.
      b.focal = (vp.size.y * 0.5f) / std::tan(FOV_Y * 0.5f);
      b.center = ImVec2(vp.origin.x + vp.size.x * 0.5f, vp.origin.y + vp.size.y * 0.5f);
      return b;
  }

  static Projected project(const Basis& b, const Vec3& p)
  {
      const Vec3 v = sub(p, b.eye);
      const Float32 z = dot(v, b.fwd);

      Projected out;
      out.depth = z;
      if(z <= MIN_DEPTH)
      {
          out.at = ImVec2(0.0f, 0.0f);
          return out;
      }

      // The perspective divide, and the whole of it. Y is negated because the
      // screen's Y grows downward and the world's up does not.
      const Float32 inv = b.focal / z;
      out.at = ImVec2(b.center.x + dot(v, b.right) * inv, b.center.y - dot(v, b.up) * inv);
      return out;
  }

  // A world-space segment, clipped against the near plane.
  //
  // THE CLIP IS THE POINT. A line with one endpoint behind the eye has a
  // negative depth at that end, and dividing by it mirrors the endpoint through
  // the centre of the screen - so the line does not stop at the edge of the
  // view, it shoots across the whole window in the wrong direction. On a ground
  // grid that is every line near the horizon at once.
  static Void segment(ImDrawList* dl, const Basis& b, const Vec3& p0, const Vec3& p1, ImU32 col)
  {
      Vec3 a = p0;
      Vec3 c = p1;
      const Float32 za = dot(sub(a, b.eye), b.fwd);
      const Float32 zc = dot(sub(c, b.eye), b.fwd);

      if(za <= NEAR_PLANE && zc <= NEAR_PLANE)
      {
          return;
      }
      if(za < NEAR_PLANE)
      {
          a = lerp(a, c, (NEAR_PLANE - za) / (zc - za));
      }
      else if(zc < NEAR_PLANE)
      {
          c = lerp(c, a, (NEAR_PLANE - zc) / (za - zc));
      }

      const Projected qa = project(b, a);
      const Projected qc = project(b, c);
      dl->AddLine(qa.at, qc.at, col, 1.0f);
  }

  // ---- the camera ----------------------------------------------------------

  Void resetCamera(Camera& cam)
  {
      cam.yaw = 0.0f;
      // ~36 degrees, and measured rather than guessed: 23 degrees put the eye
      // close enough to the ground that the grid stacked up into a wall of
      // converging lines, the room's far wall sat on the horizon, and the car
      // was a smudge. This is the angle at which the grid reads as a PLANE.
      cam.pitch = 0.62f;
      // Close enough that the 440 mm car is a recognisable box - at nine metres
      // it was forty pixels long - and still wide enough for an 8 x 6 m room.
      cam.dist = 7.5f;
      cam.target = Vec3{ 0.0f, 0.0f, 0.25f };
  }

  Void orbit(Camera& cam, Float32 dx, Float32 dy)
  {
      cam.yaw += dx * ORBIT_RATE;
      cam.pitch += dy * ORBIT_RATE;
      cam.pitch = std::min(std::max(cam.pitch, -PITCH_LIMIT), PITCH_LIMIT);
  }

  Void pan(Camera& cam, Float32 dx, Float32 dy)
  {
      // Scaled by DISTANCE, so the ground keeps up with the cursor whether the
      // camera is one metre out or twenty.
      const Float32 k = cam.dist * PAN_RATE;
      const Float32 cy = std::cos(cam.yaw);
      const Float32 sy = std::sin(cam.yaw);

      // Screen right on the ground is (cos yaw, sin yaw); screen up, flattened
      // onto the ground, is the direction the camera looks along. Panning on
      // the GROUND rather than in the camera's own plane is what stops the grid
      // sliding out from under the car when the view is tilted over.
      cam.target.x -= ((cy * dx) + (sy * dy)) * k;
      cam.target.y -= ((sy * dx) - (cy * dy)) * k;
  }

  Void zoom(Camera& cam, Float32 notches)
  {
      // Multiplicative: one notch means the same FRACTION of the way in at
      // thirty metres as at one, which is the only version that feels the same
      // at both ends of the range.
      cam.dist *= std::pow(ZOOM_STEP, -notches);
      cam.dist = std::min(std::max(cam.dist, DIST_MIN), DIST_MAX);
  }

  // ---- the synthetic scan --------------------------------------------------

  // An axis-aligned rectangle on the ground, and a ray across it.
  struct Rect
  {
      Float32 minX;
      Float32 minY;
      Float32 maxX;
      Float32 maxY;
  };

  struct Ray
  {
      Float32 ox;
      Float32 oy;
      Float32 dx;
      Float32 dy;
  };

  constexpr Rect ROOM = { -4.0f, -3.0f, 4.0f, 3.0f };

  constexpr Array<Rect, 2> OBSTACLES = {
      Rect{ -2.6f, 0.4f, -1.4f, 1.6f },
      Rect{ 1.2f, -2.0f, 2.4f, -0.8f }
  };

  constexpr Int32 SCAN_POINTS = 480;          // about one C1 revolution
  constexpr Float32 LIDAR_HEIGHT = 0.16f;
  constexpr Float32 TURN_RATE = 0.22f;        // radians per second

  // Nearest positive hit of a ray against a rectangle, by the slab method.
  // `inside` asks for the EXIT rather than the entry, which is how a room's
  // walls are hit by a scanner standing in the middle of them.
  static Float32 hitRect(const Ray& ray, const Rect& box, Bool inside)
  {
      Float32 t0 = -1.0e30f;
      Float32 t1 = 1.0e30f;

      if(std::fabs(ray.dx) < 1.0e-6f)
      {
          if(ray.ox < box.minX || ray.ox > box.maxX)
          {
              return -1.0f;
          }
      }
      else
      {
          Float32 lo = (box.minX - ray.ox) / ray.dx;
          Float32 hi = (box.maxX - ray.ox) / ray.dx;
          if(lo > hi)
          {
              std::swap(lo, hi);
          }
          t0 = std::max(t0, lo);
          t1 = std::min(t1, hi);
      }

      if(std::fabs(ray.dy) < 1.0e-6f)
      {
          if(ray.oy < box.minY || ray.oy > box.maxY)
          {
              return -1.0f;
          }
      }
      else
      {
          Float32 lo = (box.minY - ray.oy) / ray.dy;
          Float32 hi = (box.maxY - ray.oy) / ray.dy;
          if(lo > hi)
          {
              std::swap(lo, hi);
          }
          t0 = std::max(t0, lo);
          t1 = std::min(t1, hi);
      }

      if(t1 < t0 || t1 < 0.0f)
      {
          return -1.0f;
      }
      const Float32 t = inside ? t1 : t0;
      return t > 0.0f ? t : -1.0f;
  }

  static Float32 rangeAt(const Ray& ray)
  {
      Float32 best = hitRect(ray, ROOM, true);
      for(const Rect& box : OBSTACLES)
      {
          const Float32 t = hitRect(ray, box, false);
          if(t > 0.0f && (best < 0.0f || t < best))
          {
              best = t;
          }
      }
      return best;
  }

  // ---------------------------------------------------------------------------
  // THE STAND-IN SCAN, AND THE ONE CALL THAT REPLACES IT.
  //
  // A room raycast from wherever the car is standing, so the cloud has the
  // shape a real revolution has: a closed outline with the shadows of two
  // obstacles cut out of it, at the lidar's own height, moving because the car
  // is turning. A ring of random points would have looked live and taught
  // nothing about whether the projection is right.
  //
  // WHEN THE PROTOCOL CLIENT LANDS this function goes away. Its one caller in
  // main.cxx fills sc.cloud from the board's scan message instead, and nothing
  // else in the viewer knows or cares where the points came from - the cloud is
  // already just a Vec<Vec3> in the car's own frame.
  // ---------------------------------------------------------------------------
  Void fillSyntheticCloud(Vec<Vec3>& out, Float64 seconds)
  {
      const Float32 t = static_cast<Float32>(seconds);
      const Float32 heading = t * TURN_RATE;

      // The car wanders a little as well as turning, so the walls change range
      // and not just bearing.
      Ray ray;
      ray.ox = std::sin(t * 0.25f) * 0.9f;
      ray.oy = std::cos(t * 0.17f) * 0.6f;
      ray.dx = 0.0f;
      ray.dy = 0.0f;

      out.clear();
      out.reserve(static_cast<Size>(SCAN_POINTS));

      for(Int32 i = 0; i < SCAN_POINTS; ++i)
      {
          const Float32 f = static_cast<Float32>(i) / static_cast<Float32>(SCAN_POINTS);
          const Float32 a = TWO_PI * f;

          // The ray is cast in the ROOM's frame, where the rectangles are axis
          // aligned; the hit comes back as a range, and a range at bearing `a`
          // is a point in the CAR's frame without any further rotation. That is
          // what lets the room stay axis-aligned while the car turns inside it.
          ray.dx = std::sin(a + heading);
          ray.dy = std::cos(a + heading);

          const Float32 range = rangeAt(ray);
          if(range <= 0.0f)
          {
              continue;
          }

          // Sensor noise, so the outline reads as measurements rather than as a
          // drawn shape.
          const Float32 n = std::sin((static_cast<Float32>(i) * 12.9898f) + t) * 0.012f;
          const Float32 r = range + n;
          out.push_back(Vec3{ r * std::sin(a), r * std::cos(a), LIDAR_HEIGHT });
      }
  }

  // ---- drawing -------------------------------------------------------------

  static Void drawGrid(ImDrawList* dl, const Basis& b)
  {
      const ImU32 minor = IM_COL32(64, 70, 80, 255);
      const ImU32 major = IM_COL32(104, 112, 126, 255);
      const Float32 e = static_cast<Float32>(GRID_HALF);

      for(Int32 i = -GRID_HALF; i <= GRID_HALF; ++i)
      {
          const Float32 v = static_cast<Float32>(i);
          const ImU32 col = (i == 0) ? major : minor;
          segment(dl, b, Vec3{ v, -e, 0.0f }, Vec3{ v, e, 0.0f }, col);
          segment(dl, b, Vec3{ -e, v, 0.0f }, Vec3{ e, v, 0.0f }, col);
      }
  }

  static Void drawAxes(ImDrawList* dl, const Basis& b)
  {
      const Vec3 o = { 0.0f, 0.0f, 0.0f };
      segment(dl, b, o, Vec3{ AXIS_LEN, 0.0f, 0.0f }, IM_COL32(228, 96, 96, 255));
      segment(dl, b, o, Vec3{ 0.0f, AXIS_LEN, 0.0f }, IM_COL32(112, 210, 122, 255));
      segment(dl, b, o, Vec3{ 0.0f, 0.0f, AXIS_LEN }, IM_COL32(108, 154, 240, 255));
  }

  static Void drawCar(ImDrawList* dl, const Basis& b)
  {
      const Float32 hw = CAR_HALF_WIDTH;
      const Float32 hl = CAR_HALF_LENGTH;
      const Float32 z0 = CAR_FLOOR;
      const Float32 z1 = CAR_ROOF;
      const ImU32 col = IM_COL32(238, 238, 242, 255);

      const Array<Vec3, 8> c = {
          Vec3{ -hw, -hl, z0 }, Vec3{ hw, -hl, z0 }, Vec3{ hw, hl, z0 }, Vec3{ -hw, hl, z0 },
          Vec3{ -hw, -hl, z1 }, Vec3{ hw, -hl, z1 }, Vec3{ hw, hl, z1 }, Vec3{ -hw, hl, z1 }
      };

      // Twelve edges as index pairs: the floor, the roof, then the uprights.
      static constexpr Array<Int32, 24> EDGES = {
          0, 1, 1, 2, 2, 3, 3, 0,
          4, 5, 5, 6, 6, 7, 7, 4,
          0, 4, 1, 5, 2, 6, 3, 7
      };

      for(Size i = 0; i < EDGES.size(); i += 2)
      {
          const Size from = static_cast<Size>(EDGES[i]);
          const Size to = static_cast<Size>(EDGES[i + 1]);
          segment(dl, b, c[from], c[to], col);
      }

      // A chevron on the roof, so which way the car faces survives any orbit.
      // A symmetrical box does not say +Y from behind, and a viewer that cannot
      // tell forward from backward is worse than one that shows no car at all.
      const Vec3 nose = { 0.0f, hl, z1 };
      segment(dl, b, Vec3{ -hw, hl - 0.09f, z1 }, nose, col);
      segment(dl, b, Vec3{ hw, hl - 0.09f, z1 }, nose, col);
  }

  static Void drawPoints(ImDrawList* dl, const Basis& b, const Viewport& vp, const Scene& sc)
  {
      const Float32 r = sc.opt.pointSize * 0.5f;
      const Float32 margin = r + 2.0f;
      const ImU32 flat = IM_COL32(122, 214, 255, 235);
      const Bool byRange = sc.opt.coloring == PointColor::POINT_COLOR_DISTANCE;

      for(const Vec3& p : sc.cloud)
      {
          const Projected q = project(b, p);
          if(q.depth <= NEAR_PLANE)
          {
              continue;
          }
          // Off-screen points are cheap to reject and expensive to submit: a
          // disc is a fan of triangles whether or not any of it is visible.
          if(q.at.x < vp.origin.x - margin || q.at.x > vp.origin.x + vp.size.x + margin)
          {
              continue;
          }
          if(q.at.y < vp.origin.y - margin || q.at.y > vp.origin.y + vp.size.y + margin)
          {
              continue;
          }

          ImU32 col = flat;
          if(byRange)
          {
              const Float32 range = std::sqrt((p.x * p.x) + (p.y * p.y));
              const Float32 u = std::min(std::max(range / RAMP_MAX, 0.0f), 1.0f);
              // Cyan near, amber far. Two ends of one ramp rather than a rainbow:
              // a hue wheel puts two very different ranges at the same colour.
              col = rgbaOf(0.35f + (0.65f * u), 0.85f - (0.30f * u), 1.00f - (0.72f * u), 0.92f);
          }
          dl->AddCircleFilled(q.at, r, col, 8);
      }
  }

  Void draw(ImDrawList* dl, const Viewport& vp, const Scene& sc)
  {
      if(vp.size.x < 1.0f || vp.size.y < 1.0f)
      {
          return;
      }

      const Basis b = basisFor(sc.cam, vp);

      // Painter's order, and there is no depth buffer: grid, then axes, then
      // the car, then the cloud. A point BEHIND the car therefore draws over
      // it. That is the honest cost of projecting into a 2D draw list, and at
      // this scale it is cheaper to accept than to sort - the car is a wire box
      // and you can see straight through it either way.
      if(sc.opt.grid)
      {
          drawGrid(dl, b);
      }
      if(sc.opt.axes)
      {
          drawAxes(dl, b);
      }
      if(sc.opt.car)
      {
          drawCar(dl, b);
      }
      if(sc.opt.points)
      {
          drawPoints(dl, b, vp, sc);
      }
  }

}
