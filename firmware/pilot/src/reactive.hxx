// Reactive driving: one lidar scan in, one throttle and steering command out.
//
// ---------------------------------------------------------------------------
// WHAT THIS IS, AND WHAT IT DELIBERATELY IS NOT
//
// This is the DUMB layer. It has no map, no path, no odometry and no memory of
// where it has been. It looks at what is in front of the car right now and
// decides how fast to go and which way to point. That is the whole idea.
//
// autonomy.hxx is the other thing - pure pursuit along a planned path, needing
// a pose and an encoder. The two are separate files because they fail in
// different ways and are useful at different times: this one drives a car
// around a room today with nothing but a lidar, and keeps working on the day
// the encoder breaks.
//
// ---------------------------------------------------------------------------
// PURE, IN THE proto.hxx SENSE
//
// No serial, no sockets, no clock, no lidar SDK. Time arrives as `dtMs` from
// the caller and the scan arrives as a plain array. That is what lets the whole
// behaviour - including the cases below, which are the ones that crash cars -
// be exercised on a laptop with synthetic scans, months before the Orange Pi is
// plugged in and without putting a real car near a real wall.
//
// ---------------------------------------------------------------------------
// THE THREE TRAPS, WHICH ARE THE REASON THIS FILE IS CAREFUL
//
//   1. AN EMPTY SCAN IS NOT AN EMPTY ROOM. A lidar that is unplugged, stalled,
//      or pointed at glass returns nothing. The obvious loop - "minimum
//      distance over all returns", starting from infinity - reads that as a
//      clear road and applies full throttle. Blindness must be its own answer,
//      and it must stop the car. See STATUS_BLIND.
//
//   2. A ZERO DISTANCE IS NOT A ZERO DISTANCE. The C1 reports distMm == 0 to
//      mean "no return in this direction", not "an obstacle against the
//      bumper". Treating it as a measurement brakes the car at random; treating
//      it as infinity drives into the one thing the lidar could not see. It is
//      DROPPED, and it does not count toward the returns that make a scan
//      trustworthy.
//
//   3. ONE BAD POINT IS NOT AN OBSTACLE. Lidars produce isolated spurious
//      near-returns - dust, a reflection, a sensor artefact. A controller that
//      brakes on the single nearest return brakes constantly for nothing. So
//      the clearance is the Nth-nearest return, not the nearest: see
//      Config::minHits.
//
// ---------------------------------------------------------------------------
// THE CORRIDOR, NOT THE CONE
//
// What matters for stopping is whether something is in the box the car will
// sweep through, not whether something is within some angle of straight ahead.
// A wall two metres to the left at 60 degrees is far away in a cone model and
// irrelevant in reality.
//
// So each return is resolved into the car's frame: a ray at bearing b and
// distance d sits `d*sin(b)` to the side and `d*cos(b)` ahead. It is in the
// corridor when its sideways offset is inside the car's half width plus a
// margin, and then the number that matters is how far AHEAD it is.
//
// ---------------------------------------------------------------------------
// MOUNTING IS A TUNING, NOT AN ASSUMPTION
//
// The C1 reports 0..360 clockwise from its own zero mark, and which way that
// points depends on how it was bolted down. Config::forwardDeg says which raw
// angle is straight ahead, so the file never assumes the mark faces forward -
// a wrong guess there would steer the car confidently into the nearest wall.
#pragma once

#include "shared.hxx"

namespace reactive
{

  // One measurement, in the units the C1 already produces.
  //
  // Same shape as hub/src/lidar_source.hxx's LidarPoint minus the quality byte,
  // so the binding is a copy rather than a conversion. Quality is left out
  // because filtering on it is the driver's decision, not this file's.
  struct Ray
  {
      Float32 angleDeg = 0.0f;  // 0..360, raw, as the device reports it
      Float32 distMm = 0.0f;  // 0 means NO RETURN - see trap 2 above
  };

  enum class Status
  {
      STATUS_OK = 0,

      // Too few usable returns to drive on. Outputs are set to a full stop
      // rather than left alone, because the one thing that must not happen when
      // the lidar goes quiet is the car carrying on at its last throttle.
      STATUS_BLIND,

      // configure() refused the tuning; the previous one is still installed.
      STATUS_BAD_TUNING,
  };

  [[nodiscard]] CharSeq why(Status s);

  // What the car is doing, and why it looks the way it does from outside.
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
      // ---- mounting ------------------------------------------------------
      // The raw angle that points straight ahead. See the note above.
      Float32 forwardDeg = 0.0f;

      // ---- the corridor --------------------------------------------------
      // Half the car's width, plus whatever margin you want either side.
      Float32 halfWidthMm = 160.0f;

      // Returns beyond this bearing either way are ignored entirely. Stops the
      // corridor test from having to reason about things behind the car, where
      // cos(b) goes negative and "ahead" stops meaning anything.
      Float32 frontArcDeg = 75.0f;

      // ---- the distances that pick a mode --------------------------------
      // Ordered clear > slow > stop > reverse; configure() enforces it, because
      // a tuning where stop is further than slow produces a car that brakes and
      // accelerates at the same wall and looks like a bug in the controller.
      Float32 clearMm = 2000.0f;  // beyond this, nothing is in the way
      Float32 slowMm = 1200.0f;  // start easing off
      Float32 stopMm = 400.0f;  // no forward motion below this
      Float32 reverseMm = 250.0f;  // back out below this

      // Added to the threshold when LEAVING a mode, never when entering it.
      // Without it a car sitting exactly at stopMm alternates stop/go every
      // tick, which is both useless and hard on the drivetrain.
      Float32 hysteresisMm = 120.0f;

      // ---- what to send --------------------------------------------------
      Float32 cruise = 0.35f;   // 0..1 of full forward
      Float32 crawl = 0.15f;   // the slowest that still moves the car
      Float32 reverseThrottle = 0.20f;   // magnitude; the sign is added here

      // ---- steering ------------------------------------------------------
      // Scales the left/right room difference into a steering fraction. 1.0
      // means "a wall hard against one side and nothing on the other is full
      // lock"; less is gentler.
      Float32 steerGain = 0.9f;

      // The bearings the left/right room comparison looks at. Inside `nearDeg`
      // of straight ahead is excluded: a return dead ahead says nothing about
      // which way to go and, counted on both sides, cancels itself out.
      Float32 sideNearDeg = 12.0f;
      Float32 sideFarDeg = 80.0f;

      // ---- trust ---------------------------------------------------------
      // Fewest usable returns in a whole scan before it is believed at all.
      // Below this the answer is STATUS_BLIND rather than a distance.
      Int32 minValid = 40;

      // How many corridor returns must agree before a distance is acted on.
      // The clearance is the Nth-nearest, so N == 1 is "brake on any single
      // point" and is exactly the behaviour trap 3 describes.
      Int32 minHits = 3;

      // ---- commitment ----------------------------------------------------
      // Once reversing, keep reversing for at least this long. A car that backs
      // up for one tick, sees room, drives forward into the same corner and
      // reverses again is not avoiding anything.
      Int32 reverseMs = 700;

      // Longest a single reverse may last, so a car wedged with its back to a
      // wall stops rather than grinding backwards forever.
      Int32 reverseMaxMs = 2500;
  };

  [[nodiscard]] const Config& tuning();

  // Refuses a Config that cannot be driven rather than clamping it into one
  // that can - see the ordering note on the distances above.
  [[nodiscard]] Bool configure(const Config& c);

  // Everything the controller remembers between ticks, which is deliberately
  // almost nothing: the mode and how long it has been held. Owned by the
  // caller so that this file has no globals to reset and a test can drive
  // several independent cars.
  struct State
  {
      Mode  mode = Mode::MODE_STOP;
      Int32 modeMs = 0;

      // Which way the car chose to swing while reversing, held for the whole
      // manoeuvre. Re-deciding every tick with the nose against a wall makes
      // the wheels saw back and forth and the car go nowhere.
      Float32 reverseSteer = 0.0f;

      // Set when a reverse ran out of reverseMaxMs while still blocked, and
      // cleared only when the way ahead genuinely opens.
      //
      // WITHOUT IT, GIVING UP DOES NOT STICK. The timed-out reverse becomes a
      // stop, the very next tick sees the same blocked scan, and the car
      // reverses again - an endless reverse/stop stutter that is worse than
      // either. A car that has tried and failed should sit still until
      // something about the world changes.
      Bool wedged = false;
  };

  // What one tick decided. Fractions rather than microseconds: turning a
  // fraction into an ESC pulse is the drivetrain's job and its calibration
  // lives in firmware/lib/chassis, not here.
  struct Outputs
  {
      Float32 steer = 0.0f;   // -1..1, left negative, as proto::steer wants
      Float32 throttle = 0.0f;   // -1..1, negative is reverse
      Bool    stop = true;    // send STOP rather than the two above

      // For a console, a log, or an overlay. Not needed to drive.
      Float32 clearanceMm = 0.0f;
      Int32   corridorHits = 0;
      Mode    mode = Mode::MODE_STOP;
  };

  // Reads what is in front and decides one tick.
  //
  // `dtMs` is how long since the previous call - passed in rather than read
  // from a clock so the whole behaviour is reproducible in a test.
  //
  // ALWAYS writes `out`, including on STATUS_BLIND, where it writes a full
  // stop. A caller that ignores the Status still gets a safe command, which is
  // the opposite of the convention in autonomy.hxx and is deliberate: there,
  // not driving is the safe default; here, the car is already moving.
  Status step(const Ray* rays, Size count, Int32 dtMs, State* st, Outputs* out);

}
