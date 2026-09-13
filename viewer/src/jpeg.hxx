// JPEG bytes in, RGBA pixels out, with no graphics device, so viewer/tests can
// decode a real frame without a window or a board.
//
// The decoder is stb_image from third_party/stb (gitignored and cloned; see
// THIRD_PARTY.md). Only its JPEG parser is built: CAMERA's codec byte defines
// 1 = JPEG and nothing else, and every other parser would be attack surface
// reachable from a payload anyone on the hotspot can send.
#pragma once

#include "shared.hxx"

namespace jpeg
{
  // Largest decoded side. A JPEG header can claim 30000x30000 in a few bytes,
  // which stb would try to allocate (3.6 GB); the header is checked against
  // this before any pixel is allocated.
  constexpr Int32 MAX_SIDE = 4096;

  struct Picture
  {
      Int32 width = 0;
      Int32 height = 0;

      // width * height * 4, RGBA, top row first: DXGI_FORMAT_R8G8B8A8_UNORM.
      Vec<UInt8> rgba;
  };

  // False on any refusal, with `why` set to a readable reason (stb's own when
  // it has one). Never partially fills `out`.
  [[nodiscard]] Bool decode(const UInt8* bytes, Size len, Picture* out, Str* why);
}
