#include "shared.hxx"

#include "orient.hxx"

namespace orient
{

  namespace
  {

    // Turns folded into 0..3. Written to survive a negative, because C++'s `%`
    // keeps the sign of the left operand and -1 % 4 is -1, which would index
    // off the front of the corner array.
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
      // The source's own corners, in the same top-left, top-right,
      // bottom-right, bottom-left order the destination is written in.
      Array<Uv, 4> src = {
          Uv{ 0.0f, 0.0f },
          Uv{ 1.0f, 0.0f },
          Uv{ 1.0f, 1.0f },
          Uv{ 0.0f, 1.0f },
      };

      // THE FLIPS ARE APPLIED IN SOURCE SPACE, before the rotation permutes the
      // corners. That is what keeps "flip horizontally" meaning "mirror the
      // picture" at every angle, instead of meaning something that depends on
      // which way the picture happens to be turned at the time.
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

      // A quarter turn clockwise sends what was at the source's bottom-left to
      // the destination's top-left, so destination corner i samples source
      // corner i - turns.
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
