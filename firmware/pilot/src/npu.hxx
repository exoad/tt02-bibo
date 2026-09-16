// npu - one network binary on the board's NPU through VIPLite: load it, hand
// it a grey picture, get its output back as floats. Nothing here knows what
// the network is for; tagnet.cxx turns the floats into tags, and tags.cxx
// decides whether the NPU or the CPU detector runs at all.
//
// The tensor formats come from the binary itself (what tools/npu/nbg/*.json
// records), and this module honours whatever it finds: uint8 or int8 affine
// (value = (raw - zero) * scale) and int16 dynamic fixed point (value =
// raw / 2^pos), for the input as for the output. A binary asking for a
// format this module cannot fill is refused at open(), by name.
//
// Linux with VIPLite only (PILOT_HAVE_VIPLITE, found by CMake on the board).
// Elsewhere open() refuses, saying so.
#pragma once

#include "shared.hxx"

namespace npu
{
  struct Info
  {
      Str name;
      UInt32 layers = 0;
      Int32 inW = 0;             // the input's width, height and channels
      Int32 inH = 0;
      Int32 inC = 0;
      Int32 outW = 0;
      Int32 outH = 0;
      Int32 outC = 0;
      Str inFormat;              // "uint8", "int8", "int16", for a person
      Str outFormat;
  };

  [[nodiscard]] Bool available();

  // Loads and prepares `path`, allocating its buffers. False, with why, when
  // VIPLite is not there, the file is not a binary for this NPU, or its
  // tensors are of a shape or format this module does not fill: exactly one
  // input of one channel and one output.
  [[nodiscard]] Bool open(const Str& path, Info* info, Str& why);

  // One inference: `grey` is inH rows of inW, quantised into the input as the
  // binary wants; `out` receives outC x outH x outW floats, channel-major
  // with the width fastest, dequantised. `us` is the wall clock around the
  // NPU's run alone. False when not open or the run failed.
  [[nodiscard]] Bool run(const UInt8* grey, Float32* out, UInt32* us, Str& why);

  Void close();
}
