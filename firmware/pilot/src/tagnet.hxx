// tagnet - the tag detector's CPU half when the network runs on the NPU: from
// the two heatmaps the network gives (tag centres, tag corners) to tags with
// ids and corners in pixels. The same steps as tools/npu/decode.py, which is
// the reference this must agree with: peaks, four-corner sets around each
// centre, the code sampled through the quad's homography, the family matched
// under all four rotations with up to two bit errors.
//
// PURE: no NPU, no camera, no library. The heatmaps and the picture arrive as
// arrays, so tests/test_tagnet.cxx draws a tag and its heatmaps by hand and
// checks the id on every platform, and npu.cxx is the only file that knows
// what VIPLite is.
#pragma once

#include "shared.hxx"

namespace tagnet
{
  // The network's input and output sizes, fixed at export (tools/npu/model.py).
  constexpr Int32 IN_W = 320;
  constexpr Int32 IN_H = 240;
  constexpr Int32 STRIDE = 4;
  constexpr Int32 OUT_W = IN_W / STRIDE;
  constexpr Int32 OUT_H = IN_H / STRIDE;
  constexpr Size HEAT_VALUES = static_cast<Size>(2 * OUT_W * OUT_H);

  // A peak worth considering, in HEATMAP pixels, refined to its 3x3 centroid.
  constexpr Float32 CENTRE_THRESHOLD = 0.35f;
  constexpr Float32 CORNER_THRESHOLD = 0.30f;
  constexpr Int32 PEAK_RADIUS = 3;

  // How far from a centre a corner may be (heatmap px), how many of the
  // nearest are tried, and how far a set may be from a square in angle (the
  // sum over its four gaps, radians) and in distance spread.
  constexpr Float32 MAX_SPAN = 60.0f;
  constexpr Size NEAREST = 8;
  constexpr Float32 MAX_ANGLE_ERROR = 1.6f;
  constexpr Float32 MAX_SPREAD = 2.2f;
  constexpr Size MAX_SETS_PER_CENTRE = 12;
  constexpr Int32 MAX_HAMMING = 2;

  struct Corner
  {
      Float32 x = 0.0f;   // INPUT pixels (IN_W x IN_H)
      Float32 y = 0.0f;
  };

  // Four corners, counter-clockwise in image coordinates as the library orders them.
  using Quad = Array<Corner, 4>;

  struct Found
  {
      UInt16 id = 0;
      UInt8 hamming = 0;

      // The black border's corners, counter-clockwise in image coordinates
      // as the library orders them.
      Quad corners = {};

      // Mean distance of the 36 sampled cells from the black/white threshold,
      // in grey levels: the library's decision_margin in spirit.
      Float32 margin = 0.0f;
  };

  // Peaks of one heatmap (OUT_H rows of OUT_W), strongest first, none within
  // PEAK_RADIUS of a stronger one, each at its 3x3 centroid. Exposed for tests.
  struct Peak
  {
      Float32 x = 0.0f;   // HEATMAP pixels
      Float32 y = 0.0f;
      Float32 score = 0.0f;
  };

  [[nodiscard]] Vec<Peak> peaks(const Float32* heat, Float32 threshold);

  // The 3x3 homography (row-major, h[8] = 1) taking the four `src` points to
  // the four `dst` points: the exact four-point solve. False when degenerate.
  [[nodiscard]] Bool homography(const Quad& src, const Quad& dst, Array<Float64, 9>& h);

  // The 6x6 code read through the quad (INPUT pixels) as 36 bits, top-left
  // highest, 1 for a WHITE cell, thresholded midway between the black border
  // and the white ring around it. False when the quad leaves the picture or
  // the border cannot be told from the ring. Exposed for tests.
  struct Sample
  {
      UInt64 code = 0;
      Float32 margin = 0.0f;
  };

  [[nodiscard]] Bool sampleCode(const UInt8* grey, const Quad& quad, Sample* out);

  // The family member the code names under any of the four rotations, with
  // at most MAX_HAMMING bit errors. False when none.
  [[nodiscard]] Bool matchCode(UInt64 code, UInt16* id, UInt8* hamming);

  // The whole thing: `heat` is the network's output, 2 x OUT_H x OUT_W with
  // the centre map first, values 0..1; `grey` is the picture the network
  // saw, IN_H rows of IN_W. At most one tag per centre peak.
  [[nodiscard]] Vec<Found> detect(const UInt8* grey, const Float32* heat);
}
