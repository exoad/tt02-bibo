// Geometry tests for the Corners and Fit map modes.
//
//   tests\build_geom_test.bat run
//
// These exist because the SCENE decides whether these two modes produce
// anything. Corners found nothing the first time it ran on real hardware - not
// because it was broken, but because a desk with three short walls on it has no
// corners in it. That is an untestable situation to be in: a mode that finds
// nothing and a mode that cannot find anything look identical.
//
// So the geometry is fed inputs it cannot get from a desk, including the ones it
// is supposed to REJECT - which is the half that actually matters, since a
// corner detector that fires on two unrelated walls is worse than one that never
// fires at all.
//
// Exits 0 on PASS, 1 on FAIL.

#include "../src/map_geometry.hxx"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

  Int32 failures = 0;
  Int32 checks   = 0;

  Void check(Bool ok, const Char* what)
  {
      ++checks;
      if(!ok)
      {
          ++failures;
          std::printf("  FAIL  %s\n", what);
      }
      else
      {
          std::printf("  ok    %s\n", what);
      }
  }

  Void checkNear(Float32 got, Float32 want, Float32 tol, const Char* what)
  {
      ++checks;
      if(std::fabs(got - want) > tol)
      {
          ++failures;
          std::printf("  FAIL  %s: got %.3f, want %.3f +/- %.3f\n",
                      what, static_cast<Float64>(got), static_cast<Float64>(want),
                      static_cast<Float64>(tol));
      }
      else
      {
          std::printf("  ok    %s = %.3f\n", what, static_cast<Float64>(got));
      }
  }

  mapgeo::WallSeg wall(Float32 ax, Float32 ay, Float32 bx, Float32 by)
  {
      mapgeo::WallSeg w;
      w.a = mapgeo::WorldPt{ ax, ay };
      w.b = mapgeo::WorldPt{ bx, by };
      w.lenMm = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
      w.deg   = mapgeo::segBearingDeg(w.a, w.b);
      return w;
  }

  // ---------------------------------------------------------------------------

  Void testBearing()
  {
      std::printf("segBearingDeg\n");
      // y is DOWN, so "up" (bearing 0) is -y.
    checkNear(mapgeo::segBearingDeg({ 0, 0 }, { 0, -1000 }),   0.0f, 0.01f, "straight up");
    checkNear(mapgeo::segBearingDeg({ 0, 0 }, { 1000, 0 }),   90.0f, 0.01f, "to the right");
      // Undirected: reversing a wall must not change its bearing.
    checkNear(mapgeo::segBearingDeg({ 1000, 0 }, { 0, 0 }),   90.0f, 0.01f, "reversed is equal");
  }

  Void testCornerFound()
  {
      std::printf("findCorners: an L\n");

      // Two walls meeting at the origin-adjacent point (1000, -1000): one running
      // right along the top, one running down the right-hand side.
      Vec<mapgeo::WallSeg> w;
      w.push_back(wall(-500.0f, -1000.0f, 1000.0f, -1000.0f));
      w.push_back(wall(1000.0f, -1000.0f, 1000.0f,   500.0f));

      Vec<mapgeo::Corner> out;
      mapgeo::findCorners(w, out, 50.0f, 12000.0f);

      check(out.size() == 1u, "one corner");
      if(out.size() != 1u)
      {
          return;
      }

      checkNear(out[0].x, 1000.0f,  1.0f, "corner x");
      checkNear(out[0].y, -1000.0f, 1.0f, "corner y");
      checkNear(out[0].angDeg, 90.0f, 0.5f, "included angle is 90");
      checkNear(out[0].rangeMm, std::sqrt(2.0f) * 1000.0f, 1.0f, "range");

      // The arms must point back ALONG each wall, away from the vertex: one to
      // -x (back down the top wall), one to +y (down the side wall).
      const Float32 c0 = std::cos(out[0].a0), s0 = std::sin(out[0].a0);
      const Float32 c1 = std::cos(out[0].a1), s1 = std::sin(out[0].a1);
      check(c0 < -0.9f && std::fabs(s0) < 0.1f, "arm 0 runs back along the top wall");
      check(s1 >  0.9f && std::fabs(c1) < 0.1f, "arm 1 runs back along the side wall");
  }

  Void testCornerRejects()
  {
      std::printf("findCorners: what it must REJECT\n");

      Vec<mapgeo::Corner> out;

      // 1. Parallel walls. No intersection at all.
      {
          Vec<mapgeo::WallSeg> w;
          w.push_back(wall(-1000.0f, -1000.0f, 1000.0f, -1000.0f));
          w.push_back(wall(-1000.0f,  1000.0f, 1000.0f,  1000.0f));
          mapgeo::findCorners(w, out, 50.0f, 12000.0f);
          check(out.empty(), "parallel walls give no corner");
      }

      // 2. A shallow join: one surface the fitter split in two. 10 deg apart.
      {
          Vec<mapgeo::WallSeg> w;
          w.push_back(wall(-1000.0f, -1000.0f, 0.0f, -1000.0f));
          w.push_back(wall(0.0f, -1000.0f, 1000.0f, -1176.0f));   // ~10 deg
          mapgeo::findCorners(w, out, 50.0f, 12000.0f);
          check(out.empty(), "a 10 deg join is one wall, not a corner");
      }

      // 3. THE IMPORTANT ONE: two walls at 90 deg whose infinite lines cross a
      //    long way from either of them. Opposite sides of a room.
      {
          Vec<mapgeo::WallSeg> w;
          w.push_back(wall(-3000.0f, -3000.0f, -1000.0f, -3000.0f));  // horizontal, far left
          w.push_back(wall( 3000.0f,  1000.0f,  3000.0f,  3000.0f));  // vertical,  far right
          mapgeo::findCorners(w, out, 50.0f, 12000.0f);
          check(out.empty(), "lines crossing far from both walls give no corner");
      }

      // 4. The same pair, moved so the crossing IS at both their ends. Now it is
      //    a corner - which proves test 3 failed for the endpoint rule and not
      //    because the intersection maths never fires.
      {
          Vec<mapgeo::WallSeg> w;
          w.push_back(wall(-3000.0f, -3000.0f, -1000.0f, -3000.0f));
          w.push_back(wall(-1000.0f, -3000.0f, -1000.0f, -1000.0f));
          mapgeo::findCorners(w, out, 50.0f, 12000.0f);
          check(out.size() == 1u, "the same walls DO corner when their ends meet");
      }

      // 5. Out of the sensor's range window.
      {
          Vec<mapgeo::WallSeg> w;
          w.push_back(wall(13000.0f, -14000.0f, 14000.0f, -14000.0f));
          w.push_back(wall(14000.0f, -14000.0f, 14000.0f, -13000.0f));
          mapgeo::findCorners(w, out, 50.0f, 12000.0f);
          check(out.empty(), "a corner past 12 m is out of spec and dropped");
      }
  }

  Void testReach()
  {
      std::printf("computeReach: configuration space\n");

      constexpr Int32   BINS = 120;
      constexpr Float32 STEP = 360.0f / static_cast<Float32>(BINS);

      Array<Float32, BINS> clr;
      Array<Bool, BINS> seen;
      Array<Float32, BINS> out;

      // An empty circular room, 3 m radius.
      for(Int32 i = 0; i < BINS; ++i)
      {
          clr[i] = 3000.0f;
          seen[i] = true;
      }

      // A car 190 mm wide -> 95 mm half-width.
      mapgeo::computeReach(mapgeo::PolarScan{ clr.data(), seen.data(), BINS, STEP },
                           95.0f, out.data());

      // In an open circular room the centre can get to within halfW of the wall,
      // and no further. sqrt(3000^2 - 95^2) - ... is not the answer; the binding
      // obstacle is the one straight ahead, so r = 3000 - 95.
      checkNear(out[0], 3000.0f - 95.0f, 12.0f, "open room: reach = radius - halfW");

      // Now a slot: two obstacles 150 mm apart across bearing 0. A 190 mm car
      // cannot pass, so the reach on that bearing must collapse to the slot.
      for(Int32 i = 0; i < BINS; ++i)
      {
          clr[i] = 3000.0f;
          seen[i] = true;
      }
      {
          // Two returns at 1000 mm, +/-75 mm either side of the bearing-0 axis.
          // 75 mm off-axis at 1000 mm range is atan(75/1000) = 4.29 deg, which at
          // 3 deg bins is bin +/-1 (bin centres sit at +/-1.5 deg, +/-4.5 deg).
          const Float32 r = std::sqrt(1000.0f * 1000.0f + 75.0f * 75.0f);
          clr[1] = r;               // + side
          clr[BINS - 1] = r;        // - side
      }
      mapgeo::computeReach(mapgeo::PolarScan{ clr.data(), seen.data(), BINS, STEP },
                           95.0f, out.data());
      check(out[0] < 1100.0f, "a 150 mm slot blocks a 190 mm car");
      std::printf("        reach through the slot = %.0f mm\n",
                  static_cast<Float64>(out[0]));

      // A narrow car fits through the same slot.
      mapgeo::computeReach(mapgeo::PolarScan{ clr.data(), seen.data(), BINS, STEP },
                           40.0f, out.data());
      check(out[0] > 2000.0f, "an 80 mm car passes the same slot");
      std::printf("        reach for the narrow car = %.0f mm\n",
                  static_cast<Float64>(out[0]));

      // Reach can never exceed the raw clearance on its own bearing.
      for(Int32 i = 0; i < BINS; ++i) { clr[i] = 500.0f + static_cast<Float32>(i) * 20.0f;
                                        seen[i] = true; }
      mapgeo::computeReach(mapgeo::PolarScan{ clr.data(), seen.data(), BINS, STEP },
                           95.0f, out.data());
      Bool bounded = true;
      for(Int32 i = 0; i < BINS; ++i)
      {
          if(out[i] > clr[i] + 0.5f)
          {
              bounded = false;
          }
      }
      check(bounded, "reach never exceeds the free radius on its own bearing");
  }

  Void testHeading()
  {
      std::printf("estimateHeading\n");

      constexpr Int32   BINS = 120;
      constexpr Float32 STEP = 360.0f / static_cast<Float32>(BINS);

      Array<Float32, BINS> ref;
      Array<Float32, BINS> cur;
      Array<Bool, BINS>    refSeen;
      Array<Bool, BINS>    curSeen;

      const auto scanOf = [](const Array<Float32, BINS>& r,
                             const Array<Bool, BINS>& seen)
      {
          return mapgeo::PolarScan{ r.data(), seen.data(), BINS, STEP };
      };

      // An asymmetric room: a long wall one way, a short one the other, so the
      // profile actually identifies a direction.
      for(Int32 i = 0; i < BINS; ++i)
      {
          const Float32 a = static_cast<Float32>(i) * STEP * 3.14159265f / 180.0f;
          ref[i] = 2000.0f + 900.0f * std::cos(a) + 400.0f * std::sin(2.0f * a);
          refSeen[i] = true;
      }

      // Rotate it by a whole number of bins and see if the shift comes back.
      const Int32 SHIFT = 17;
      for(Int32 i = 0; i < BINS; ++i)
      {
          cur[(i + SHIFT) % BINS] = ref[i];
          curSeen[i] = true;
      }

      Float32 deg = 0.0f, score = 0.0f;
      check(mapgeo::estimateHeading(scanOf(ref, refSeen), scanOf(cur, curSeen), deg, score),
            "a shifted profile is matched");
      checkNear(deg, static_cast<Float32>(SHIFT) * STEP, 0.6f, "recovered angle");
      check(score > 0.3f, "and it is confident about it");
      std::printf("        score %.2f\n", static_cast<Float64>(score));

      // Zero shift must come back as zero, not as 360.
      Float32 d0 = 0.0f, s0 = 0.0f;
      static_cast<Void>(mapgeo::estimateHeading(scanOf(ref, refSeen),
                                                scanOf(ref, refSeen), d0, s0));
      check(d0 < 1.0f || d0 > 359.0f, "an unrotated profile reads as zero");

      // SUB-BIN. Rotating by half a bin must not quantise to a whole one - the
      // world snapping in 3 degree steps as you turn is what this refinement is
      // for.
      for(Int32 i = 0; i < BINS; ++i)
      {
          const Float32 a = (static_cast<Float32>(i) - 10.5f) * STEP * 3.14159265f / 180.0f;
          cur[i] = 2000.0f + 900.0f * std::cos(a) + 400.0f * std::sin(2.0f * a);
          curSeen[i] = true;
      }
      Float32 dh = 0.0f, sh = 0.0f;
      static_cast<Void>(mapgeo::estimateHeading(scanOf(ref, refSeen),
                                                scanOf(cur, curSeen), dh, sh));
      checkNear(dh, 10.5f * STEP, 1.2f, "half-bin rotation resolved below a bin");

      // THE FAILURE CASE THAT MATTERS. A circular room is the same from every
      // angle, so there is no correct answer - and the score has to say so rather
      // than the function returning a confident wrong one.
      for(Int32 i = 0; i < BINS; ++i)
      {
          ref[i] = 2500.0f; refSeen[i] = true;
          cur[i] = 2500.0f; curSeen[i] = true;
      }
      Float32 dc = 0.0f, sc = 1.0f;
      static_cast<Void>(mapgeo::estimateHeading(scanOf(ref, refSeen),
                                                scanOf(cur, curSeen), dc, sc));
      check(sc < 0.05f, "a circular room scores ~0: no direction to find");
      std::printf("        score %.3f\n", static_cast<Float64>(sc));

      // No overlap at all is a failure, not a lucky zero.
      for(Int32 i = 0; i < BINS; ++i)
      {
          refSeen[i] = true;
          curSeen[i] = false;
      }
      Float32 dn = 0.0f, sn = 0.0f;
      check(!mapgeo::estimateHeading(scanOf(ref, refSeen), scanOf(cur, curSeen), dn, sn),
            "no overlapping evidence returns false");
  }

  Void testArea()
  {
      std::printf("polarArea\n");

      constexpr Int32   BINS = 120;
      constexpr Float32 STEP = 360.0f / static_cast<Float32>(BINS);

      Array<Float32, BINS> r;
      for(Int32 i = 0; i < BINS; ++i)
      {
          r[i] = 1000.0f;  // a 1 m circle
      }

      // pi * 1^2 = 3.1416 m^2. A 120-gon is very slightly under.
      checkNear(mapgeo::polarArea(r.data(), BINS, STEP), 3.1416f, 0.01f, "unit circle area");

      for(Int32 i = 0; i < BINS; ++i)
      {
          r[i] = 0.0f;
      }
      checkNear(mapgeo::polarArea(r.data(), BINS, STEP), 0.0f, 1e-4f, "empty polygon");
  }

} // namespace

Int32 main()
{
    std::printf("map geometry tests\n\n");

    testBearing();
    testCornerFound();
    testCornerRejects();
    testReach();
    testHeading();
    testArea();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
