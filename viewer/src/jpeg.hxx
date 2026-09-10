// JPEG bytes in, RGBA pixels out. Nothing else.
//
// ---------------------------------------------------------------------------
// WHY THIS IS ITS OWN MODULE AND NOT PART OF camera.cxx
//
// The same seam link.cxx already draws between its pure half and its socket
// half, for the same reason. camera.cxx owns a D3D11 device, an ImGui window
// and a texture; none of those exist in a test process, and a decoder that can
// only be reached through them is a decoder that can only be exercised by
// looking at it.
//
// This half takes a pointer and a length and returns pixels, so
// viewer/tests/test_link.cxx decodes a real JPEG on a laptop with no board, no
// window and no graphics device - which matters here more than usual, because
// the board-side producer that will send the first CAMERA frame is being
// written in parallel and has never run.
//
// ---------------------------------------------------------------------------
// THE DECODER IS stb_image, AND IT IS NOT IN THIS REPOSITORY
//
// third_party/stb is gitignored and cloned, the same arrangement Dear ImGui
// has - see THIRD_PARTY.md, which records the version and the licence. Only
// the JPEG decoder is compiled in: CAMERA's codec byte defines 1 = JPEG and
// nothing else, so building the PNG, GIF, PSD, TGA, BMP, HDR, PIC and PNM
// decoders would be eight more parsers reachable from a payload a stranger on
// a hotspot wrote, in exchange for no feature at all.
#pragma once

#include "shared.hxx"

namespace jpeg
{

  // The largest picture this viewer will decode, per side. The camera is
  // 640x480 and MAX_PAYLOAD bounds a frame at 256 KiB, but a JPEG header is
  // free to CLAIM 30000x30000 in a few bytes and the decoder would then try to
  // allocate 3.6 GB before failing. The dimensions are read and checked before
  // a single pixel is allocated, so a hostile header is refused rather than
  // survived.
  constexpr Int32 MAX_SIDE = 4096;

  struct Picture
  {
      Int32 width = 0;
      Int32 height = 0;

      // width * height * 4, RGBA, top row first - what D3D11 wants for
      // DXGI_FORMAT_R8G8B8A8_UNORM and what stb_image produces at reqComp 4.
      Vec<UInt8> rgba;
  };

  // False on any refusal, with `why` set to a sentence a person can act on -
  // stb_image's own failure reason where there is one. Never partially fills
  // `out`: a half-decoded picture drawn as a whole one is the same species of
  // lie as a stale frame drawn as live.
  [[nodiscard]] Bool decode(const UInt8* bytes, Size len, Picture* out, Str* why);

}
