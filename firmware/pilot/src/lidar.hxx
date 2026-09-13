// The companion board's lidar: Slamtec's C1 over USB serial, one revolution at a
// time, as reactive::Ray. Nothing above this header names an sl_lidar_sdk type.
// Free functions and file-local state, because the car has one lidar.
//
// No thread of its own: the blocking grab() is the caller's tick clock, and a
// thread here would add a lock and a stale-frame hazard for nothing.
//
// The SDK is built on the Orange Pi and given to CMake with -DPILOT_RPLIDAR_SDK.
// Without it, or on MSVC, every function REFUSES: open() returns false with a
// reason naming the missing SDK, and grab() returns false with the vector
// emptied. A stub reporting an empty room instead of a missing sensor would be
// trap 1 of reactive.hxx. tests/test_pilot.cxx holds the refusing half to the
// promises below, in both builds.
#pragma once

#include "shared.hxx"

#include "reactive.hxx"

namespace lidar
{
  // Whether an SDK is compiled in: false on MSVC and on any Linux build without
  // -DPILOT_RPLIDAR_SDK. Tells "no lidar" from "no code to talk to one".
  [[nodiscard]] Bool available();

  // Opens the port, talks to the device, and remembers what it said. The motor
  // is NOT started, so reading a serial number does not spin the lidar. The
  // default baud is the C1's; the A-series units use other rates.
  //
  // The port is held EXCLUSIVELY while open (TIOCEXCL on Linux): a second
  // program - a car program against bibo-pilot, or the reverse - is refused
  // with "another program has <port>" rather than sharing one byte stream and
  // breaking both.
  //
  // Idempotent: a second call while open is true. false is explained by
  // reason(). A device that a killed session left spinning is parked here.
  [[nodiscard]] Bool open(const Str& port, Int32 baud = 460800);

  // Stops the scan, stops the motor, releases the port. Safe with nothing open.
  Void close();

  [[nodiscard]] Bool isOpen();

  // Why the most recent call that returned false did so. Empty after a call
  // that succeeded.
  [[nodiscard]] const Str& reason();

  // Why the most recent open() refused, for code to branch on instead of
  // matching reason()'s wording. REFUSAL_HELD usually means the pilot service
  // holds the port and has to be stopped first.
  enum class Refusal
  {
      REFUSAL_NONE,            // the last open() succeeded, or none was tried
      REFUSAL_NO_SDK,          // this build has no SDK (the refusing half)
      REFUSAL_NO_PORT,         // no such device - the cable
      REFUSAL_NO_PERMISSION,   // the device exists and this user may not open it
      REFUSAL_HELD,            // another program has it open exclusively
      REFUSAL_CANNOT_OPEN,     // the port would not open for some other reason
      REFUSAL_NOT_A_LIDAR,     // opened, and nothing answered - baud, or not a lidar
      REFUSAL_SDK,             // the SDK failed between the port and the device
  };

  // Set by every open() and by nothing else, so it still describes the last
  // open() after a later grab() has written its own reason().
  [[nodiscard]] Refusal refusal();

  // Spins the motor up and starts the scan the SDK considers typical for the
  // device. grab() has nothing to return until this has succeeded.
  [[nodiscard]] Bool motorOn();

  // Stops the scan, waits for the device to acknowledge, then cuts the motor -
  // in that order, because cutting the motor first can leave the scan running
  // and the next startScan confused. Safe when already off.
  [[nodiscard]] Bool motorOff();

  [[nodiscard]] Bool isSpinning();

  // Blocks until one full revolution has arrived, and REPLACES `out` with it:
  // the C1's q14 angle as degrees and its q2 distance as millimetres. A distance
  // of 0 is carried through and means no return in that direction.
  //
  // false, with `out` EMPTIED, when no revolution arrives within `timeoutMs`,
  // when the motor is off, or when nothing is open. Emptied, unlike
  // carlink::drain: a caller that ignores the Bool hands reactive::step a blind
  // scan and stops the car, rather than driving on a stale revolution.
  //
  // Quality is not filtered. The C1 pairs a zero distance with a zero quality,
  // so the no-return rule drops those, and reactive::Config::minHits handles
  // the isolated spurious point that survives.
  //
  // One timeout is routine; a long unbroken run of them is the cable coming
  // out, and judging that belongs to whoever is counting. The FIRST grab after
  // motorOn() times out at the default every time: the C1 takes over two
  // seconds to reach a steady speed, and motorOn() leaves that wait to the
  // caller.
  //
  // `quality` is optional and PARALLEL to `out`: quality[i] is the C1's 0..63
  // return strength for out[i], emptied on every path that empties `out`. A
  // pointer rather than a Ray field, so the driving code never holds a value it
  // must ignore.
  [[nodiscard]] Bool grab(Vec<reactive::Ray>& out, Int32 timeoutMs = 2000, Vec<UInt8>* quality = nullptr);

  // The device's identity as it reported it at open(): model, firmware,
  // hardware revision, serial. Empty when nothing is open.
  [[nodiscard]] Str info();

  // The same identity as numbers, for viewfeed to put on the wire.
  //
  // model, fwMajor, fwMinor, hwRev and serial are captured at open() and do
  // not change while it is open. health is the SDK's status - 0 good, 1
  // warning, 2 error - from the MOST RECENT reading: open() takes one, and
  // motorOn() and health() take another. -1 is "unknown": nothing is open,
  // or the device did not answer the query.
  struct Device
  {
      Int32 model = 0;
      Int32 fwMajor = 0;
      Int32 fwMinor = 0;
      Int32 hwRev = 0;
      Str   serial;   // 16 bytes as 32 upper-case hex digits; empty when not open
      Int32 health = -1;
  };

  // A default Device (health -1, serial empty) when nothing is open.
  [[nodiscard]] Device device();

  // The device's self-report: re-read when the motor is off, and repeated from
  // the last reading while it spins, because a health query mid-scan stalls a
  // revolution. Empty when nothing is open.
  [[nodiscard]] Str health();
}
