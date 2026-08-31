// ---------------------------------------------------------------------------
// Scan recording: capture revolutions to a file, and play them back.
//
// WHY THIS AND NOT MAPPING. What was asked for was "mapping or localization" -
// and a real map needs a POSE for every scan, which needs translation as well as
// rotation. There is no odometry on this machine yet (GP15 is marked "wheel
// encoder, not wired") and scan-profile matching recovers rotation only. Building
// a map from unlocated scans would produce a confident-looking picture that is
// wrong the moment the sensor moves, which is worse than no map.
//
// So this is the part that is honest today, and it is also the part that has to
// come first: you cannot develop a mapper against a live sensor. You develop it
// against a recording you can replay a hundred times and get the same answer.
// When the encoders are wired, the mapper reads these files.
//
// ---------------------------------------------------------------------------
// FILE FORMAT - "biborec 1", and "tt02rec 2" is still read. The version went
// back to 1 with the name because it is a new name for the same layout, not a
// new layout; the reader accepts either header and nothing else changed.
//
// PLAIN TEXT, and compact because of how it is
// written rather than because a compressor was run over it.
//
//   # biborec 1
//   # <comments, freely>
//   R <t_ms> <hz_centi> <count>
//   <angleDelta> <dist> <angleDelta> <dist> ...      (count pairs, wrapped)
//   R ...
//
//   angleDelta  hundredths of a degree from the previous point; the first in a
//               revolution is measured from 0. The C1 steps ~0.72 deg but the
//               real spacing wanders between 0.49 and 1.05, so these are MEASURED
//               and kept, not assumed uniform and thrown away.
//   dist        whole millimetres. 0 means no return on that bearing, which is a
//               fact about that bearing and is preserved as one.
//   t_ms        milliseconds from the start of the recording.
//   hz_centi    the device's reported rotation rate x100.
//
// The parser reads `count` pairs after each R line wherever they fall, so the
// wrapping is cosmetic and a writer may use any line length. That also makes the
// format streamable: a line at a time, which is what the Pico will want when it
// writes these to an SD card itself.
//
// WHY TEXT, AND WHY IT IS NOT BIGGER. Measured on 48 real revolutions:
//
//     binary, 2 x float32 per point     8.00 bytes/point   100%
//     this format                       6.65 bytes/point    83%
//     the same with delta-coded ranges   ~5.9 bytes/point    74%
//
// (Measured, not estimated: a 49-revolution capture of 24,915 points wrote
// 165,696 bytes.)
//
// Text WINS against the binary form it replaced, because the device reports
// whole millimetres - so "6789" carries exactly what a float32 did, in four
// bytes instead of four with none of the ambiguity. Delta-coding the ranges
// would save another 8 points and cost the one property this format exists for:
// you can open it, read it, and see that 6789 is a wall 6.8 m away. On an SD
// card pulled out of a car at the side of a track, that matters more than 8%.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"
#include "lidar_source.hxx"

namespace rec
{

  struct Rev
  {
      Float64                 tS = 0.0;     // seconds from the start
      Float32                 hz = 0.0f;
      Vec<LidarPoint> points;
  };

  class Recording
  {
  public:
      Void clear();

      // Appends a revolution at `tS` seconds from the start.
      Void append(const LidarFrame& f, Float64 tS);

      [[nodiscard]] Size       count() const
      {
          return revs.size();
      }
      [[nodiscard]] Bool       empty() const
      {
          return revs.empty();
      }
      [[nodiscard]] const Rev& at(Size i) const;

      // Wall-clock span from the first revolution to the last. Zero for fewer
      // than two - a single revolution has a timestamp but no duration.
      [[nodiscard]] Float64 durationS() const;

      // Total points held, for reporting a size without walking the file.
      [[nodiscard]] Size pointCount() const;

      // The revolution at or just before `tS`, for playback. Returns count()-1
      // past the end so a finished playback holds on the last frame rather than
      // snapping back.
      [[nodiscard]] Size indexAt(Float64 tS) const;

      // Both report failure through `err` rather than throwing. A recording that
      // will not save must say why - it may be the only copy of a run that took a
      // person twenty minutes to set up.
      //
      // load() also reads the original "TT02REC1" binary files. Keeping that path
      // costs twenty lines and means changing the format never cost anybody a
      // recording.
      Bool save(const Str& path, Str& err) const;
      Bool load(const Str& path, Str& err);

  private:
      Bool loadBinaryV1(const Str& path, Str& err);
      Bool loadTextV2(const Str& path, Str& err);

      Vec<Rev> revs;
  };

  // %LOCALAPPDATA%\bibo\recordings, created on demand. Empty if there is no
  // user profile.
  [[nodiscard]] Str dir();

  // Existing recordings, newest first, bare filenames.
  [[nodiscard]] Vec<Str> list();

  // A timestamped name: scan-YYYYMMDD-HHMMSS.biborec
  [[nodiscard]] Str makeName();

} // namespace rec
