#include "shared.hxx"

#include "orient.hxx"

namespace orient
{
  namespace
  {
    // Turns folded into 0..3. C++'s % keeps the sign, so -1 % 4 is -1.
    [[nodiscard]] Int32 quarters(Int32 turns)
    {
        return ((turns % 4) + 4) % 4;
    }
  }

  Bool sideways(Int32 turns)
  {
      return (quarters(turns) % 2) != 0;
  }

  Array<Uv, 4> cornerUvs(Int32 turns, Bool flipX, Bool flipY)
  {
      // Top-left, top-right, bottom-right, bottom-left.
      Array<Uv, 4> src = {
          Uv{ 0.0f, 0.0f },
          Uv{ 1.0f, 0.0f },
          Uv{ 1.0f, 1.0f },
          Uv{ 0.0f, 1.0f },
      };
      // Flips apply in source space, before the rotation, so "flip
      // horizontally" mirrors the picture the same way at every angle.
      for(Uv& p : src)
      {
          if(flipX)
          {
              p.u = 1.0f - p.u;
          }
          if(flipY)
          {
              p.v = 1.0f - p.v;
          }
      }
      // Clockwise: destination corner i samples source corner i - turns.
      const Int32 steps = quarters(turns);
      Array<Uv, 4> out = {};
      for(Int32 i = 0; i < 4; ++i)
      {
          const Int32 from = ((i - steps) + 4) % 4;
          out[static_cast<Size>(i)] = src[static_cast<Size>(from)];
      }
      return out;
  }
}
