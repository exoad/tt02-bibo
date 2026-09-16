// The AprilTag detector: a thread of its own that takes each camera frame
// viewfeed captured, finds tag36h11 tags in it, and publishes them as TAGS.
//
// It is the apriltag bundle's working half. The behaviour in the chain
// (chain::ID_APRILTAG) says nothing in a pass, and the host starts this when
// it sees that bundle loaded and stops it when it goes - the way trim's verbs
// live in viewfeed rather than in trim's behaviour, so the chain stays pure
// (chain.hxx). Nothing here drives: a detection is a fact about a picture,
// published for a viewer, and what the car does with one is a later bundle.
//
// Linux with the library only (PILOT_APRILTAG at configure time). Elsewhere
// start() and detect() refuse, saying so, and stats() reports nothing built:
// never a detector that runs and finds nothing.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"

namespace tags
{
  // The camera's intrinsics and the printed tag's size, which turn four
  // corners into a range and a bearing. Measured at `width` x `height`;
  // find() scales them to the frame it is handed. Uncalibrated means every
  // range is 0 and TAGS_FLAG_CALIBRATED is clear: the boxes still draw, and
  // a driver that needs a range does not move.
  struct Intrinsics
  {
      Bool calibrated = false;
      Float64 fx = 0.0;
      Float64 fy = 0.0;
      Float64 cx = 0.0;
      Float64 cy = 0.0;
      UInt16 width = 0;
      UInt16 height = 0;
      Float64 tagM = 0.0;   // the printed tag's outer black edge, in metres
  };

  // ~/.config/bibo/camera.txt, or BIBO_CAMERA_FILE. Beside trim.txt, and
  // written by a person, once, from one measurement: hold a tag D metres
  // from the lens, read its side S in pixels from the viewer's apriltag
  // window, and fx = fy = S * D / tag.
  [[nodiscard]] Str defaultCameraPath();

  // `key value` per line - fx, fy, cx, cy, tag (metres) and size WxH - with
  // '#' to end of line a comment. All six are required, so a file that is
  // half a calibration is no calibration; false with why, and `out`
  // untouched. Pure, so every platform's suite checks it.
  [[nodiscard]] Bool parseIntrinsics(const Str& text, Intrinsics* out, Str& why);
  [[nodiscard]] Bool loadIntrinsics(const Str& path, Intrinsics* out, Str& why);

  struct Config
  {
      // Asked of viewfeed's capture; 0 is the board's default rate.
      UInt16 fps = 10;

      // The library's quad_decimate: 2 searches for quads at half resolution
      // and refines corners at full, which on this board is a third of the
      // time of 1 for the same tags found at these sizes.
      Float32 decimate = 2.0f;

      // The library's own worker pool. 1 keeps the detector to one core.
      Int32 threads = 1;

      Intrinsics cal;

      // Which cores the thread is pinned to, by the kernel's cpu_capacity:
      // "big" (the highest set, the default), "little" (the lowest, which
      // frees the big cores for driving at about twice the time per frame),
      // or "any" (the scheduler's choice). BIBO_TAGS_CORES sets it.
      Str cores = "big";
  };

  struct Stats
  {
      Bool built = false;         // this build carries the detector
      Bool running = false;
      UInt64 frames = 0;          // frames detected on
      UInt64 undecodable = 0;     // frames the JPEG decoder refused
      UInt64 seen = 0;            // tags found, summed over frames
      UInt32 lastDetectUs = 0;
      UInt32 lastCount = 0;
      Str cores;                  // which cpus the thread runs on, in words
      Str why;                    // the last refusal or decode failure, in words
  };

  [[nodiscard]] Bool available();

  // One frame, synchronously: the JPEG to grey, the detector over it, and
  // `out` filled with what it found, width, height and detectUs included.
  // tMonoUs and frameIndex are the caller's to set, since only it knows which
  // picture this was. What the thread runs, exposed so a test can hand it a
  // picture. False, with why, when the bytes will not decode or the detector
  // is not built.
  [[nodiscard]] Bool find(const Vec<UInt8>& pic, const Config& cfg, bibowire::Tags* out, Str& why);

  // Starts the thread and subscribes to the camera. False, with why, when not
  // built, already running, or the thread cannot start.
  [[nodiscard]] Bool start(const Config& cfg, Str& why);

  [[nodiscard]] Bool running();
  [[nodiscard]] Stats stats();

  // The newest frame's detections and how long ago they were made, for
  // the chain's pass. False before the first, and always false when not
  // built. A copy under a lock, like the rest of this module's answers.
  [[nodiscard]] Bool latest(bibowire::Tags* out, Int32* ageMs);

  // Unsubscribes from the camera and joins the thread. Safe when not running.
  Void stop();
}
