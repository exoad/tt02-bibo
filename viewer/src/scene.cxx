#include "scene.hxx"

#include <algorithm>
#include <cmath>

namespace scene
{
  constexpr Float32 NEAR_PLANE = 0.05f;       // metres

  // What `project` treats as in front of the eye. Far smaller than NEAR_PLANE:
  // `segment` clips an endpoint onto the near plane and rounding can leave it a
  // hair inside, which a NEAR_PLANE test would reject.
  constexpr Float32 MIN_DEPTH = 1.0e-4f;

  constexpr Float32 FOV_Y = 1.0472f;          // 60 degrees, vertical

  constexpr Float32 ORBIT_RATE = 0.0075f;     // radians per pixel
  constexpr Float32 PAN_RATE = 0.0016f;       // metres per pixel, per metre out
  constexpr Float32 ZOOM_STEP = 1.15f;        // per wheel notch
  constexpr Float32 DIST_MIN = 0.8f;
  constexpr Float32 DIST_MAX = 40.0f;

  // Short of the pole, where the view axis parallels +Z and the right vector
  // collapses to zero.
  constexpr Float32 PITCH_LIMIT = 1.5533f;    // 89 degrees

  constexpr Int32 GRID_HALF = 10;             // metres each way, 1 m spacing
  constexpr Float32 AXIS_LEN = 0.6f;

  // The TT-02 with its shell on, ~440 x 190 x 130 mm (docs/hardware.md).
  constexpr Float32 CAR_HALF_WIDTH = 0.095f;
  constexpr Float32 CAR_HALF_LENGTH = 0.22f;
  constexpr Float32 CAR_FLOOR = 0.02f;
  constexpr Float32 CAR_ROOF = 0.15f;

  constexpr Float32 RAMP_MAX = 6.0f;          // metres at the far end of the ramp

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

  // The camera resolved into an eye, an orthonormal basis, and the focal
  // length in PIXELS.
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
      // Target to eye. Up is pinned to +Z, so pitch is elevation and yaw swings
      // around the car.
      const Vec3 off = { cp * sy * cam.dist, -cp * cy * cam.dist, sp * cam.dist };
      Basis b;
      b.eye = Vec3{ cam.target.x + off.x, cam.target.y + off.y, cam.target.z + off.z };
      b.fwd = normalize(Vec3{ -off.x, -off.y, -off.z });
      b.right = normalize(cross(b.fwd, Vec3{ 0.0f, 0.0f, 1.0f }));
      b.up = cross(b.right, b.fwd);
      // From the viewport height, so resizing the window keeps the field of view.
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
      // Y is negated because screen Y grows downward.
      const Float32 inv = b.focal / z;
      out.at = ImVec2(b.center.x + dot(v, b.right) * inv, b.center.y - dot(v, b.up) * inv);
      return out;
  }

  // A world-space segment, clipped against the near plane. Unclipped, an
  // endpoint behind the eye projects mirrored through the screen centre and the
  // line shoots across the window.
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

  Void resetCamera(Camera& cam)
  {
      cam.yaw = 0.0f;
      // ~36 degrees: much lower and the grid stacks into a wall of lines.
      cam.pitch = 0.62f;
      // The car still reads as a box, and an 8 x 6 m room still fits.
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
      // Scaled by distance so the ground keeps up with the cursor, and moved on
      // the ground plane rather than the camera's plane so a tilted view does
      // not slide the grid out from under the car.
      const Float32 k = cam.dist * PAN_RATE;
      const Float32 cy = std::cos(cam.yaw);
      const Float32 sy = std::sin(cam.yaw);
      cam.target.x -= ((cy * dx) + (sy * dy)) * k;
      cam.target.y -= ((sy * dx) - (cy * dy)) * k;
  }

  Void zoom(Camera& cam, Float32 notches)
  {
      // Multiplicative, so a notch is the same fraction at any distance.
      cam.dist *= std::pow(ZOOM_STEP, -notches);
      cam.dist = std::min(std::max(cam.dist, DIST_MIN), DIST_MAX);
  }

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

  // The heading arrow's drawn proportions; its angle is headingDir.
  constexpr Float32 ARROW_LEN = 0.70f;        // metres, about 1.6 car lengths
  constexpr Float32 ARROW_BARB = 0.12f;
  constexpr Float32 ARROW_BARB_RAD = 0.45f;

  static Void drawArrow(ImDrawList* dl, const Basis& b, Float32 steer, ImU32 col)
  {
      const Vec3 d = headingDir(steer);
      const Vec3 from = { 0.0f, CAR_HALF_LENGTH, CAR_FLOOR };
      const Vec3 tip = { from.x + (ARROW_LEN * d.x), from.y + (ARROW_LEN * d.y), CAR_FLOOR };
      segment(dl, b, from, tip, col);
      // Barbs in the ground plane, rotated from the direction vector so they
      // cannot disagree with the shaft.
      const Float32 cs = std::cos(ARROW_BARB_RAD);
      const Float32 sn = std::sin(ARROW_BARB_RAD);
      const Vec3 left = { (d.x * cs) - (d.y * sn), (d.x * sn) + (d.y * cs), 0.0f };
      const Vec3 right = { (d.x * cs) + (d.y * sn), (d.y * cs) - (d.x * sn), 0.0f };
      const Vec3 barbL = { tip.x - (ARROW_BARB * left.x), tip.y - (ARROW_BARB * left.y), CAR_FLOOR };
      const Vec3 barbR = { tip.x - (ARROW_BARB * right.x), tip.y - (ARROW_BARB * right.y), CAR_FLOOR };
      segment(dl, b, tip, barbL, col);
      segment(dl, b, tip, barbR, col);
  }

  // Ghost (asked) first, so the solid (actual) arrow is on top where they overlap.
  static Void drawHeading(ImDrawList* dl, const Basis& b, const Scene& sc)
  {
      if(sc.haveSteerWant)
      {
          drawArrow(dl, b, sc.steerWant, IM_COL32(120, 150, 190, 150));
      }
      if(sc.haveSteerNow)
      {
          drawArrow(dl, b, sc.steerNow, IM_COL32(120, 210, 255, 255));
      }
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
      // A roof chevron, because a symmetrical box does not show which way is
      // forward from every orbit angle.
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
      const Float32 tint = sc.cloudStale ? 0.35f : 1.0f;
      const Float32 dim = sc.cloudStale ? 0.55f : 1.0f;
      for(const Vec3& p : sc.cloud)
      {
          const Projected q = project(b, p);
          if(q.depth <= NEAR_PLANE)
          {
              continue;
          }
          // A disc costs a triangle fan even off screen, so reject it first.
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
              // Cyan near, amber far: one ramp, not a hue wheel, so no two
              // ranges share a colour.
              col = rgbaOf(0.35f + (0.65f * u), 0.85f - (0.30f * u), 1.00f - (0.72f * u), 0.92f);
          }
          if(sc.cloudStale)
          {
              // Stale keeps its shape and loses the ramp, so it cannot pass
              // for live at a glance.
              const Float32 g = 0.62f * dim;
              col = rgbaOf(g, g, g, 0.92f * tint);
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
      // No depth buffer, so painter's order: a point behind the wire-box car
      // draws over it, which is accepted rather than sorted.
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
      if(sc.opt.heading)
      {
          drawHeading(dl, b, sc);
      }
      if(sc.opt.points)
      {
          drawPoints(dl, b, vp, sc);
      }
  }
}
