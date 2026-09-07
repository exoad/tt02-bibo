// The companion board's lidar: Slamtec's C1 over USB serial, one revolution at
// a time, in the units reactive.hxx already speaks.
//
// ---------------------------------------------------------------------------
// WHAT THIS IS
//
// The seam between Slamtec's sl_lidar_sdk and the rest of this program. On
// one side is the SDK's vocabulary - ILidarDriver, sl_result, q14 angles and
// q2 millimetres; on the other is a Vec<reactive::Ray>. Nothing above this
// header names an SDK type, so the day the lidar is a different one, or the
// SDK is dropped for a hand-written parser of the C1's serial protocol, this
// header does not move.
//
// It is the same idea as hub/src/lidar_source.hxx, minus the thread. The hub
// wraps grabScanDataHq() in a worker because a UI thread cannot block; the
// autonomy loop CAN, and wants to - a driving tick with nothing new to look at
// has nothing to do, so the blocking grab is the tick's clock. Wrapping it in
// a second thread here would add a mutex and a stale-frame hazard to gain
// nothing.
//
// ---------------------------------------------------------------------------
// THE SDK IS LINUX-ONLY HERE, AND THAT IS SAID OUT LOUD
//
// The library is built on the Orange Pi at a path CMake is told about with
// -DPILOT_RPLIDAR_SDK. Without it, or on MSVC, every function below REFUSES:
// open() returns false with a reason that names the missing SDK, and grab()
// returns false having emptied the vector. Same rule as link.hxx - a stub that
// reported an empty room rather than a missing sensor would be trap 1 of
// reactive.hxx built into the program on purpose.
//
// The refusing path is what tests/build_pilot_test.bat compiles on the laptop,
// and what CMake compiles anywhere the option is unset. tests/test_pilot.cxx
// holds it to the promises below in both builds - it is the only place that
// does, since nothing else ever runs this program without a lidar SDK.
//
// ---------------------------------------------------------------------------
// ONE DEVICE, NOT A CLASS
//
// Free functions and file-local state, the way carlink and reactive are laid
// out, because there is one lidar on the car and one serial port it lives on.
// A second lidar would be a second design decision, not a second instance.
#pragma once

#include "shared.hxx"

#include "reactive.hxx"

namespace lidar
{

  // Whether an SDK is compiled into this program at all. False on MSVC and on
  // any Linux build without -DPILOT_RPLIDAR_SDK; a caller that gets false from
  // open() can ask this to tell "no lidar" from "no code to talk to one".
  [[nodiscard]] Bool available();

  // Opens the port, talks to the device, and remembers what it said. The motor
  // is NOT started: a lidar spinning before anyone has asked for a scan is a
  // lidar spinning on a bench while somebody reads its serial number.
  //
  // 460800 is what the C1 enumerates at; the A-series units use 115200 or
  // 256000 and are not what is bolted to this car.
  //
  // Idempotent: a second call while open is true. false is explained by
  // reason().
  [[nodiscard]] Bool open(const Str& port, Int32 baud = 460800);

  // Stops the scan, stops the motor, releases the port. Safe with nothing open.
  // Called for you by a successful open() when a previous session was killed
  // rather than closed - see the note in lidar.cxx.
  Void close();

  [[nodiscard]] Bool isOpen();

  // Why the most recent call that returned false did so. Empty after a call
  // that succeeded. A Str rather than a CharSeq because the useful reasons
  // carry the port name and the SDK's hex code, neither of which is a literal.
  [[nodiscard]] const Str& reason();

  // Spins the motor up and starts the scan the SDK considers typical for the
  // device. grab() has nothing to return until this has succeeded.
  [[nodiscard]] Bool motorOn();

  // Stops the scan, waits for the device to acknowledge, then cuts the motor -
  // in that order, because cutting the motor first can leave the scan running
  // and the next startScan confused. Safe when already off.
  [[nodiscard]] Bool motorOff();

  [[nodiscard]] Bool isSpinning();

  // Blocks until one full revolution has arrived, and REPLACES `out` with it.
  // The C1's q14 angle becomes degrees and its q2 distance becomes millimetres;
  // a distance of 0 is carried through unchanged and means what reactive.hxx
  // says it means - no return in that direction.
  //
  // Returns false, with `out` EMPTIED, when no revolution arrives within
  // `timeoutMs`, when the motor is off, or when nothing is open. Emptied rather
  // than left alone, and this is the opposite of carlink::drain's rule for a
  // reason: a caller that ignores the Bool and hands `out` to reactive::step
  // gets STATUS_BLIND and a stopped car, where a stale revolution would get a
  // car confidently driving on what the room looked like a second ago.
  //
  // Quality is not filtered. The C1 pairs a zero distance with a zero quality,
  // so the "no return" rule already drops those, and reactive::Config::minHits
  // is the defence against the isolated spurious point that survives it.
  //
  // One revolution is a single grab; timing out on one of them is routine and
  // not a fault. A long unbroken run of them is the cable coming out, and that
  // judgement belongs to whoever is counting.
  //
  // In particular the FIRST grab after motorOn() times out at the default,
  // every time: measured on the C1 on 2026-09-06, the motor takes over two
  // seconds to come up to speed and the SDK will not hand over a revolution
  // until it has one at a steady rate. motorOn() does not wait that out for
  // you, because the time is better spent by a caller that has other things to
  // set up - and a caller that has not should expect one blind tick.
  [[nodiscard]] Bool grab(Vec<reactive::Ray>& out, Int32 timeoutMs = 2000);

  // The device's identity as it reported it at open(): model, firmware,
  // hardware revision, serial. Empty when nothing is open.
  [[nodiscard]] Str info();

  // The device's self-report, re-read from the device when the motor is off
  // and repeated from the last reading when it is spinning - the SDK serialises
  // commands against the scan stream, and a health query mid-scan is a stall
  // in the middle of a revolution. Empty when nothing is open.
  [[nodiscard]] Str health();

}
