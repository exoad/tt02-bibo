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

  // Unsubscribes from the camera and joins the thread. Safe when not running.
  Void stop();
}
