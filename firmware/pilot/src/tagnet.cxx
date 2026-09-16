#include "tagnet.hxx"

#include "tag36h11_codes.hxx"

#include <algorithm>
#include <cmath>

namespace tagnet
{
  namespace
  {
    constexpr Float32 PI = 3.14159265358979f;

    // Where (u, v) of the unit square lands, or false when off the picture.
    [[nodiscard]] Bool project(const Array<Float64, 9>& h, Float64 u, Float64 v, Int32* x, Int32* y)
    {
        const Float64 w = h[6] * u + h[7] * v + h[8];
        if(std::fabs(w) < 1e-9)
        {
            return false;
        }
        const Float64 px = (h[0] * u + h[1] * v + h[2]) / w;
        const Float64 py = (h[3] * u + h[4] * v + h[5]) / w;
        const Int32 xi = static_cast<Int32>(std::lround(px));
        const Int32 yi = static_cast<Int32>(std::lround(py));
        if(xi < 0 || yi < 0 || xi >= IN_W || yi >= IN_H)
        {
            return false;
        }
        *x = xi;
        *y = yi;
        return true;
    }

    [[nodiscard]] Float32 median(Vec<Float32> v)
    {
        std::sort(v.begin(), v.end());
        const Size n = v.size();
        return n % 2u == 1u ? v[n / 2u] : 0.5f * (v[n / 2u - 1u] + v[n / 2u]);
    }

    // The 6x6 grid rotated a quarter turn clockwise, k times, as a code.
    [[nodiscard]] UInt64 rotated(UInt64 code, Int32 k)
    {
        UInt64 out = 0;
        for(Int32 r = 0; r < 6; ++r)
        {
            for(Int32 c = 0; c < 6; ++c)
            {
                // Destination (r, c) takes source (5 - c, r) for one clockwise turn.
                Int32 sr = r;
                Int32 sc = c;
                for(Int32 i = 0; i < k; ++i)
                {
                    const Int32 nr = 5 - sc;
                    const Int32 nc = sr;
                    sr = nr;
                    sc = nc;
                }
                const UInt64 bit = (code >> (35 - (sr * 6 + sc))) & 1u;
                out = (out << 1) | bit;
            }
        }
        return out;
    }

    // One candidate set of four corners around a centre, best fit first.
    struct Candidate
    {
        Float32 error = 0.0f;
        Quad quad = {};
    };

    struct Near
    {
        Float32 dist = 0.0f;
        Float32 angle = 0.0f;
        Float32 x = 0.0f;
        Float32 y = 0.0f;
    };

    [[nodiscard]] Vec<Candidate> candidates(const Peak& centre, const Vec<Peak>& corners)
    {
        Vec<Near> near;
        for(const Peak& p : corners)
        {
            const Float32 d = std::hypot(p.x - centre.x, p.y - centre.y);
            if(d >= 1.5f && d <= MAX_SPAN)
            {
                near.push_back(Near{ d, std::atan2(p.y - centre.y, p.x - centre.x), p.x, p.y });
            }
        }
        std::sort(
            near.begin(),
            near.end(),
            [](const Near& a, const Near& b) { return a.dist < b.dist; }
        );
        if(near.size() > NEAREST)
        {
            near.resize(NEAREST);
        }
        Vec<Candidate> out;
        if(near.size() < 4u)
        {
            return out;
        }
        const Size n = near.size();
        for(Size a = 0; a < n; ++a)
        {
            for(Size b = a + 1u; b < n; ++b)
            {
                for(Size c = b + 1u; c < n; ++c)
                {
                    for(Size d = c + 1u; d < n; ++d)
                    {
                        Array<Near, 4> pick = { near[a], near[b], near[c], near[d] };
                        std::sort(
                            pick.begin(),
                            pick.end(),
                            [](const Near& l, const Near& r) { return l.angle < r.angle; }
                        );
                        Float32 lo = pick[0].dist;
                        Float32 hi = pick[0].dist;
                        for(const Near& p : pick)
                        {
                            lo = std::min(lo, p.dist);
                            hi = std::max(hi, p.dist);
                        }
                        if(hi / std::max(1e-6f, lo) > MAX_SPREAD)
                        {
                            continue;
                        }
                        Float32 err = 0.0f;
                        for(Size k = 0; k < 4u; ++k)
                        {
                            const Float32 gap = pick[k].angle - pick[(k + 3u) % 4u].angle;
                            const Float32 wrapped = gap < 0.0f ? gap + 2.0f * PI : gap;
                            err += std::fabs(wrapped - PI / 2.0f);
                        }
                        if(err > MAX_ANGLE_ERROR)
                        {
                            continue;
                        }
                        Candidate cand;
                        cand.error = err;
                        for(Size k = 0; k < 4u; ++k)
                        {
                            cand.quad[k] = Corner{ pick[k].x * STRIDE, pick[k].y * STRIDE };
                        }
                        out.push_back(cand);
                    }
                }
            }
        }
        std::sort(
            out.begin(),
            out.end(),
            [](const Candidate& l, const Candidate& r) { return l.error < r.error; }
        );
        if(out.size() > MAX_SETS_PER_CENTRE)
        {
            out.resize(MAX_SETS_PER_CENTRE);
        }
        return out;
    }
  }

  Bool homography(const Quad& src, const Quad& dst, Array<Float64, 9>& h)
  {
      Array<Array<Float64, 9>, 8> m = {};
      for(Size i = 0; i < 4u; ++i)
      {
          const Float64 x = src[i].x;
          const Float64 y = src[i].y;
          const Float64 u = dst[i].x;
          const Float64 v = dst[i].y;
          m[2u * i] = { x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y, u };
          m[2u * i + 1u] = { 0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y, v };
      }
      for(Size col = 0; col < 8u; ++col)
      {
          Size pivot = col;
          for(Size r = col + 1u; r < 8u; ++r)
          {
              if(std::fabs(m[r][col]) > std::fabs(m[pivot][col]))
              {
                  pivot = r;
              }
          }
          if(std::fabs(m[pivot][col]) < 1e-12)
          {
              return false;
          }
          std::swap(m[col], m[pivot]);
          for(Size r = 0; r < 8u; ++r)
          {
              if(r == col)
              {
                  continue;
              }
              const Float64 f = m[r][col] / m[col][col];
              for(Size c = col; c < 9u; ++c)
              {
                  m[r][c] -= f * m[col][c];
              }
          }
      }
      for(Size i = 0; i < 8u; ++i)
      {
          h[i] = m[i][8] / m[i][i];
      }
      h[8] = 1.0;
      return true;
  }

  Vec<Peak> peaks(const Float32* heat, Float32 threshold)
  {
      struct Cell
      {
          Int32 x;
          Int32 y;
          Float32 v;
      };
      Vec<Cell> cells;
      for(Int32 y = 0; y < OUT_H; ++y)
      {
          for(Int32 x = 0; x < OUT_W; ++x)
          {
              const Float32 v = heat[y * OUT_W + x];
              if(v < threshold)
              {
                  continue;
              }
              Bool top = true;
              for(Int32 dy = -1; dy <= 1 && top; ++dy)
              {
                  for(Int32 dx = -1; dx <= 1; ++dx)
                  {
                      const Int32 nx = x + dx;
                      const Int32 ny = y + dy;
                      if(nx < 0 || ny < 0 || nx >= OUT_W || ny >= OUT_H)
                      {
                          continue;
                      }
                      if(heat[ny * OUT_W + nx] > v)
                      {
                          top = false;
                          break;
                      }
                  }
              }
              if(top)
              {
                  cells.push_back(Cell{ x, y, v });
              }
          }
      }
      std::stable_sort(
          cells.begin(),
          cells.end(),
          [](const Cell& a, const Cell& b) { return a.v > b.v; }
      );
      Vec<Cell> kept;
      for(const Cell& c : cells)
      {
          Bool shadowed = false;
          for(const Cell& k : kept)
          {
              if(std::abs(c.x - k.x) <= PEAK_RADIUS && std::abs(c.y - k.y) <= PEAK_RADIUS)
              {
                  shadowed = true;
                  break;
              }
          }
          if(!shadowed)
          {
              kept.push_back(c);
          }
      }
      Vec<Peak> out;
      for(const Cell& c : kept)
      {
          Float32 mass = 0.0f;
          Float32 sx = 0.0f;
          Float32 sy = 0.0f;
          for(Int32 dy = -1; dy <= 1; ++dy)
          {
              for(Int32 dx = -1; dx <= 1; ++dx)
              {
                  const Int32 nx = c.x + dx;
                  const Int32 ny = c.y + dy;
                  if(nx < 0 || ny < 0 || nx >= OUT_W || ny >= OUT_H)
                  {
                      continue;
                  }
                  const Float32 v = heat[ny * OUT_W + nx];
                  mass += v;
                  sx += v * static_cast<Float32>(nx);
                  sy += v * static_cast<Float32>(ny);
              }
          }
          out.push_back(Peak{ sx / mass, sy / mass, c.v });
      }
      return out;
  }

  Bool sampleCode(const UInt8* grey, const Quad& quad, Sample* out)
  {
      const Quad unit = { { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 1.0f } } };
      Array<Float64, 9> h = {};
      if(!homography(unit, quad, h))
      {
          return false;
      }
      // The quad is the black border, eight cells across; its outer ring of
      // cells is black and the ring outside it (the tag's white margin) white.
      Vec<Float32> blacks;
      Vec<Float32> whites;
      for(Int32 i = 0; i < 8; ++i)
      {
          Int32 x = 0;
          Int32 y = 0;
          const Float64 along = (i + 0.5) / 8.0;
          if(project(h, along, 0.5 / 8.0, &x, &y))
          {
              blacks.push_back(grey[y * IN_W + x]);
          }
          if(project(h, 0.5 / 8.0, along, &x, &y))
          {
              blacks.push_back(grey[y * IN_W + x]);
          }
          if(project(h, along, -0.5 / 8.0, &x, &y))
          {
              whites.push_back(grey[y * IN_W + x]);
          }
          if(project(h, -0.5 / 8.0, along, &x, &y))
          {
              whites.push_back(grey[y * IN_W + x]);
          }
      }
      if(blacks.size() < 6u || whites.size() < 6u)
      {
          return false;
      }
      const Float32 threshold = 0.5f * (median(blacks) + median(whites));
      UInt64 bits = 0;
      Float32 total = 0.0f;
      for(Int32 r = 0; r < 6; ++r)
      {
          for(Int32 c = 0; c < 6; ++c)
          {
              Int32 x = 0;
              Int32 y = 0;
              if(!project(h, (c + 1.5) / 8.0, (r + 1.5) / 8.0, &x, &y))
              {
                  return false;
              }
              const Float32 v = grey[y * IN_W + x];
              bits = (bits << 1) | (v > threshold ? 1u : 0u);
              total += std::fabs(v - threshold);
          }
      }
      out->code = bits;
      out->margin = total / 36.0f;
      return true;
  }

  Bool matchCode(UInt64 code, UInt16* id, UInt8* hamming)
  {
      Int32 best = MAX_HAMMING + 1;
      for(Size i = 0; i < TAG36H11_COUNT; ++i)
      {
          for(Int32 k = 0; k < 4; ++k)
          {
              const UInt64 diff = rotated(TAG36H11_CODES[i], k) ^ code;
              Int32 errors = 0;
              for(UInt64 d = diff; d != 0u; d &= d - 1u)
              {
                  ++errors;
              }
              if(errors < best)
              {
                  best = errors;
                  *id = static_cast<UInt16>(i);
                  *hamming = static_cast<UInt8>(errors);
                  if(errors == 0)
                  {
                      return true;
                  }
              }
          }
      }
      return best <= MAX_HAMMING;
  }

  Vec<Found> detect(const UInt8* grey, const Float32* heat)
  {
      Vec<Found> out;
      const Vec<Peak> centres = peaks(heat, CENTRE_THRESHOLD);
      const Vec<Peak> corners = peaks(heat + OUT_W * OUT_H, CORNER_THRESHOLD);
      for(const Peak& centre : centres)
      {
          for(const Candidate& cand : candidates(centre, corners))
          {
              Sample s;
              if(!sampleCode(grey, cand.quad, &s))
              {
                  continue;
              }
              Found f;
              if(matchCode(s.code, &f.id, &f.hamming))
              {
                  f.corners = cand.quad;
                  f.margin = s.margin;
                  out.push_back(f);
                  break;
              }
          }
      }
      return out;
  }
}
