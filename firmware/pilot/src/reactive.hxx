// Reactive driving: one lidar scan in, one throttle and steering command out.
// The DUMB layer: no map, path, odometry or memory of where it has been. Pure,
// like proto.hxx: no serial, sockets, clock or lidar SDK. Time arrives as
// `dtMs` and the scan as a plain array, so every case below, including the ones
// that crash cars, runs on synthetic scans.
//
// THE THREE TRAPS
//   1. AN EMPTY SCAN IS NOT AN EMPTY ROOM. A lidar that is unplugged, stalled or
//      pointed at glass returns nothing, and a minimum distance starting from
//      infinity reads that as a clear road. Blindness is its own answer, and it
//      stops the car: STATUS_BLIND.
//   2. A ZERO DISTANCE IS NOT A ZERO DISTANCE. The C1's distMm == 0 means no
//      return, not an obstacle at the bumper. As a measurement it brakes at
//      random; as infinity it drives into what the lidar could not see. It is
//      DROPPED, and does not count toward the returns that make a scan trusted.
//   3. ONE BAD POINT IS NOT AN OBSTACLE. Dust and reflections make isolated
//      near returns, so the clearance is the Nth-nearest return, not the
//      nearest: Config::minHits.
//
// THE CORRIDOR, NOT THE CONE. What matters is whether something is in the box
// the car sweeps, not within some angle of ahead. A ray at bearing b and
// distance d sits `d*sin(b)` to the side and `d*cos(b)` ahead; it is in the
// corridor when that side offset is within halfWidthMm, and then its distance
// AHEAD is what counts.
//
// Config::forwardDeg names the raw angle that points ahead, because where the
// C1's zero mark faces depends on how it was bolted down.
#pragma once

#include "shared.hxx"

namespace reactive
{
  // One measurement, in the C1's units. No quality byte: filtering on it is the
  // driver's decision, not this file's.
  struct Ray
  {
      Float32 angleDeg = 0.0f;  // 0..360, raw, as the device reports it
      Float32 distMm = 0.0f;  // 0 means NO RETURN - see trap 2 above
  };

  enum class Status
  {
      STATUS_OK = 0,

      // Too few usable returns to drive on. Outputs are set to a full stop, so
      // a quiet lidar never leaves the car on its last throttle.
      STATUS_BLIND,

      // configure() refused the tuning; the previous one is still installed.
      STATUS_BAD_TUNING,
  };

  [[nodiscard]] CharSeq why(Status s);

  enum class Mode
  {
      MODE_CRUISE = 0,   // clear ahead, up to speed
      MODE_SLOW,         // something ahead, easing off
      MODE_STOP,         // too close to move forward
      MODE_REVERSE,      // backing out, steering to swing the nose clear
      MODE_BLIND,        // no usable scan
  };

  [[nodiscard]] CharSeq modeName(Mode m);

  struct Config
  {
      Float32 forwardDeg = 0.0f;

      // Half the car's width, plus a margin either side.
      Float32 halfWidthMm = 160.0f;

      // Returns beyond this bearing either way are ignored, so the corridor test
      // never meets things behind the car, where cos(b) goes negative.
      Float32 frontArcDeg = 75.0f;

      // Ordered clear > slow > stop > reverse, which configure() enforces: stop
      // further out than slow brakes and accelerates at the same wall. Roomy for
      // the brushless motor, since braking distance grows with the SQUARE of
      // speed; tighten them only from a measured stopping test.
      Float32 clearMm = 2500.0f;  // beyond this, nothing is in the way
      Float32 slowMm = 1600.0f;  // start easing off
      Float32 stopMm = 600.0f;  // no forward motion below this
      Float32 reverseMm = 250.0f;  // back out below this

      // Added to the threshold when LEAVING a mode, never when entering it, so a
      // car sitting exactly at stopMm does not alternate stop/go every tick.
      Float32 hysteresisMm = 120.0f;

      Float32 cruise = 0.25f;   // 0..1 of the calibrated range
      Float32 crawl = 0.15f;   // the slowest that still moves the car
      Float32 reverseThrottle = 0.20f;   // magnitude; the sign is added here

      // Scales the left/right room difference into a steering fraction. 1.0 is
      // full lock for a wall hard against one side and nothing on the other.
      Float32 steerGain = 0.9f;

      // The bearings the left/right room comparison looks at. Inside sideNearDeg
      // is excluded: a return dead ahead says nothing about which way to go and,
      // counted on both sides, cancels itself out.
      Float32 sideNearDeg = 12.0f;
      Float32 sideFarDeg = 80.0f;

      // Fewest usable returns in a whole scan before it is believed at all;
      // below this the answer is STATUS_BLIND.
      Int32 minValid = 40;

      // How many corridor returns must agree before a distance is acted on.
      // 1 brakes on any single point: trap 3.
      Int32 minHits = 3;

      // Once reversing, keep reversing at least this long, so the car does not
      // back up one tick and drive straight back into the same corner.
      Int32 reverseMs = 700;

      // Longest a single reverse may last, so a car wedged with its back to a
      // wall stops rather than grinding backwards forever.
      Int32 reverseMaxMs = 2500;
  };

  [[nodiscard]] const Config& tuning();

  // Refuses a Config that cannot be driven, rather than clamping it into one.
  [[nodiscard]] Bool configure(const Config& c);

  // What the controller remembers between ticks. Owned by the caller, so a test
  // can drive several independent cars.
  struct State
  {
      Mode  mode = Mode::MODE_STOP;
      Int32 modeMs = 0;

      // Which way to swing while reversing, held for the whole manoeuvre:
      // re-deciding every tick with the nose against a wall saws the wheels
      // back and forth and the car goes nowhere.
      Float32 reverseSteer = 0.0f;

      // Set when a reverse ran out of reverseMaxMs while still blocked, and
      // cleared only when the way ahead genuinely opens. Without it the stop
      // after a timed-out reverse sees the same blocked scan and reverses again,
      // an endless reverse/stop stutter.
      Bool wedged = false;
  };

  // What one tick decided, as fractions: the ESC calibration lives in
  // firmware/lib/chassis, not here.
  struct Outputs
  {
      Float32 steer = 0.0f;   // -1..1, left negative, as proto::steer wants
      Float32 throttle = 0.0f;   // -1..1, negative is reverse
      Bool    stop = true;    // stand still, whatever the two above say

      // For a console, a log, or an overlay; not needed to drive.
      Float32 clearanceMm = 0.0f;
      Int32   corridorHits = 0;
      Mode    mode = Mode::MODE_STOP;
  };

  // Decides one tick. `dtMs` is the time since the previous call, passed in so
  // the behaviour is reproducible in a test.
  //
  // ALWAYS writes `out`, a full stop on STATUS_BLIND, so a caller that ignores
  // the Status still gets a safe command.
  Status step(const Ray* rays, Size count, Int32 dtMs, State* st, Outputs* out);
}
