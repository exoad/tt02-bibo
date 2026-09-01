#include "map_geometry.hxx"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace mapgeo
{

  namespace
  {
    constexpr Float32 PI_F = 3.14159265358979323846f;
  }

  Void findCorners(const Vec<WallSeg>& w, Vec<Corner>& out, Float32 minRangeMm, Float32 maxRangeMm)
  {
      out.clear();

      const Int32 n = static_cast<Int32>(w.size());
      for(Int32 i = 0; i < n; ++i)
      {
          for(Int32 j = i + 1; j < n; ++j)
          {
              // Undirected bearings, so the turn lands in [0,90].
              Float32 turn = std::fabs(w[static_cast<Size>(i)].deg - w[static_cast<Size>(j)].deg);
              if(turn > 90.0f)
              {
                  turn = 180.0f - turn;
              }
              if(turn < CORNER_MIN_DEG)
              {
                  continue;
              }

              const WorldPt& p1 = w[static_cast<Size>(i)].a;
              const WorldPt& p2 = w[static_cast<Size>(i)].b;
              const WorldPt& p3 = w[static_cast<Size>(j)].a;
              const WorldPt& p4 = w[static_cast<Size>(j)].b;

              const Float32 d1x = p2.x - p1.x, d1y = p2.y - p1.y;
              const Float32 d2x = p4.x - p3.x, d2y = p4.y - p3.y;

              const Float32 den = d1x * d2y - d1y * d2x;
              if(std::fabs(den) < 1e-4f)
              {
                  continue;               // parallel; the turn test should already
              }
                                          // have caught this

              const Float32 t = ((p3.x - p1.x) * d2y - (p3.y - p1.y) * d2x) / den;
              const WorldPt hit{ p1.x + d1x * t, p1.y + d1y * t };

              if(endGap(hit, p1, p2) > CORNER_NEAR_MM || endGap(hit, p3, p4) > CORNER_NEAR_MM)
              {
                  continue;
              }

              const Float32 r = std::sqrt(hit.x * hit.x + hit.y * hit.y);
              if(!(r >= minRangeMm) || r > maxRangeMm)
              {
                  continue;
              }

              // Arms point back along each wall, away from the crossing, toward
              // whichever end is further from it - so the marker traces the two
              // surfaces that actually exist rather than a generic cross.
              const Float32 f1 = (endGap(hit, p1, p1) > endGap(hit, p2, p2)) ? -1.0f : 1.0f;
              const Float32 e1a = std::sqrt((hit.x - p1.x) * (hit.x - p1.x)
                                          + (hit.y - p1.y) * (hit.y - p1.y));
              const Float32 e1b = std::sqrt((hit.x - p2.x) * (hit.x - p2.x)
                                          + (hit.y - p2.y) * (hit.y - p2.y));
              const Float32 s1 = (e1a > e1b) ? -1.0f : 1.0f;

              const Float32 e2a = std::sqrt((hit.x - p3.x) * (hit.x - p3.x)
                                          + (hit.y - p3.y) * (hit.y - p3.y));
              const Float32 e2b = std::sqrt((hit.x - p4.x) * (hit.x - p4.x)
                                          + (hit.y - p4.y) * (hit.y - p4.y));
              const Float32 s2 = (e2a > e2b) ? -1.0f : 1.0f;
              static_cast<Void>(f1);

              Corner k;
              k.x = hit.x;
              k.y = hit.y;
              k.angDeg = 180.0f - turn;
              k.rangeMm = r;
              k.a0 = std::atan2(d1y * s1, d1x * s1);
              k.a1 = std::atan2(d2y * s2, d2x * s2);
              out.push_back(k);

              if(out.size() >= CORNER_MAX)
              {
                  return;
              }
          }
      }
  }

  Void computeReach(const PolarScan& in, Float32 halfW, Float32* out)
  {
      const Float32* clr = in.r;
      const Bool*    seen = in.seen;
      const Int32    bins = in.bins;
      const Float32  binDeg = in.binDeg;

      // Beyond this offset nothing in spec can block: at 72 deg the perpendicular
      // offset of anything further than ~halfW/sin(72) already clears the disc.
      const Int32 win = std::min(bins / 2, 24);

      for(Int32 i = 0; i < bins; ++i)
      {
          Float32 r = seen[i] ? clr[i] : 0.0f;

          for(Int32 k = -win; k <= win && r > 0.0f; ++k)
          {
              Int32 j = i + k;
              while(j < 0)
              {
                  j += bins;
              }
              while(j >= bins)
              {
                  j -= bins;
              }
              if(!seen[j])
              {
                  continue;
              }

              const Float32 a = static_cast<Float32>(k) * binDeg * (PI_F / 180.0f);
              const Float32 R = clr[j];
              const Float32 s = R * std::sin(a);
              if(std::fabs(s) >= halfW)
              {
                  continue;                       // already clears the disc
              }

              const Float32 rj = R * std::cos(a) - std::sqrt(halfW * halfW - s * s);
              if(rj < r)
              {
                  r = (rj > 0.0f) ? rj : 0.0f;
              }
          }

          out[i] = r;
      }
  }

  Bool estimateHeading(const PolarScan& refScan, const PolarScan& curScan, Float32& outDeg, Float32& outScore)
  {
      outDeg = 0.0f;
      outScore = 0.0f;

      const Float32* ref = refScan.r;
      const Bool*    refSeen = refScan.seen;
      const Float32* cur = curScan.r;
      const Bool*    curSeen = curScan.seen;
      const Int32    bins = refScan.bins;
      const Float32  binDeg = refScan.binDeg;

      if(ref == nullptr || cur == nullptr || refSeen == nullptr
         || curSeen == nullptr || bins < 8)
      {
          return false;
      }

      // Two profiles of different shapes cannot be correlated bin against
      // bin, and silently using the reference's count would read past the end
      // of the shorter one.
      if(curScan.bins != bins || curScan.binDeg != binDeg)
      {
          return false;
      }

      // Cost per candidate shift: mean absolute difference over the bearings both
      // profiles actually have evidence for. Bins either side has never seen
      // contribute nothing rather than contributing a zero, which would reward
      // alignments that happen to line up two blind spots.
      Vec<Float32> cost(static_cast<Size>(bins), 0.0f);

      for(Int32 k = 0; k < bins; ++k)
      {
          Float64 sum = 0.0;
          Int32   n = 0;

          for(Int32 i = 0; i < bins; ++i)
          {
              Int32 j = i + k;
              while(j >= bins)
              {
                  j -= bins;
              }

              if(!refSeen[i] || !curSeen[j])
              {
                  continue;
              }

              sum += std::fabs(static_cast<Float64>(cur[j]) - static_cast<Float64>(ref[i]));
              ++n;
          }

          // Too little overlap to mean anything. Marked unusable rather than
          // given a flattering low cost from three lucky bins.
          cost[static_cast<Size>(k)] = (n >= bins / 4)
              ? static_cast<Float32>(sum / n)
              : FLT_MAX;
      }

      Int32   best = -1;
      Float32 bestCost = FLT_MAX;
      for(Int32 k = 0; k < bins; ++k)
      {
          if(cost[static_cast<Size>(k)] < bestCost)
          {
              bestCost = cost[static_cast<Size>(k)];
              best = k;
          }
      }

      if(best < 0 || bestCost == FLT_MAX)
      {
          return false;
      }

      // Separability: the best against the MEDIAN, not against the worst. The
      // worst shift is an outlier and flatters every match; the median is what a
      // typical wrong answer costs, and being much better than typical is what
      // "this profile identifies a direction" actually means.
      Vec<Float32> sorted;
      sorted.reserve(static_cast<Size>(bins));
      for(Int32 k = 0; k < bins; ++k)
      {
          if(cost[static_cast<Size>(k)] != FLT_MAX)
          {
              sorted.push_back(cost[static_cast<Size>(k)]);
          }
      }
      std::sort(sorted.begin(), sorted.end());

      const Float32 median = sorted.empty() ? 0.0f : sorted[sorted.size() / 2];
      outScore = (median > 1e-3f) ? (1.0f - bestCost / median) : 0.0f;
      if(outScore < 0.0f)
      {
          outScore = 0.0f;
      }
      if(outScore > 1.0f)
      {
          outScore = 1.0f;
      }

      // Sub-bin refinement by parabola through the best and its two neighbors.
      // Without it the heading quantises to 3 deg steps and the world visibly
      // snaps as you turn.
      const Int32 km = (best + bins - 1) % bins;
      const Int32 kp = (best + 1) % bins;
      const Float32 c0 = cost[static_cast<Size>(km)];
      const Float32 c1 = cost[static_cast<Size>(best)];
      const Float32 c2 = cost[static_cast<Size>(kp)];

      Float32 delta = 0.0f;
      if(c0 != FLT_MAX && c2 != FLT_MAX)
      {
          const Float32 den = (c0 - 2.0f * c1 + c2);
          if(std::fabs(den) > 1e-6f)
          {
              delta = 0.5f * (c0 - c2) / den;
          }
          if(delta < -1.0f)
          {
              delta = -1.0f;
          }
          if(delta >  1.0f)
          {
              delta =  1.0f;
          }
      }

      Float32 deg = (static_cast<Float32>(best) + delta) * binDeg;
      while(deg >= 360.0f)
      {
          deg -= 360.0f;
      }
      while(deg <    0.0f)
      {
          deg += 360.0f;
      }

      outDeg = deg;
      return true;
  }

  Float32 polarArea(const Float32* r, Int32 bins, Float32 binDeg)
  {
      const Float32 step = binDeg * (PI_F / 180.0f);
      const Float64 k = 0.5 * static_cast<Float64>(std::sin(step));

      Float64 a = 0.0;
      for(Int32 i = 0; i < bins; ++i)
      {
          const Int32 j = (i + 1) % bins;
          a += k * static_cast<Float64>(r[i]) * static_cast<Float64>(r[j]);
      }
      return static_cast<Float32>(a / 1.0e6);      // mm^2 -> m^2
  }

}
