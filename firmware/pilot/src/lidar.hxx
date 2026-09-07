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
  // The port is held EXCLUSIVELY while open (TIOCEXCL on Linux), so a second
  // program - the pilot against the scan feed, or the reverse - is refused
  // here with "another program has <port>" rather than quietly sharing one
  // byte stream and breaking both. See guardFd in lidar.cxx for the day that
  // was measured.
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

  // WHY the most recent open() refused, as a value a program can branch on.
  // reason() is the sentence for a person; this is the same fact for code,
  // because matching on the sentence would tie a caller to its wording.
  // scanfeed branches on REFUSAL_HELD: a port held by another program is the
  // pilot driving, and the right answer is to relay to it, not to say no.
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

  // Set by every open(); untouched by every other call, so it describes the
  // last open() even after a later grab() has written its own reason().
  [[nodiscard]] Refusal refusal();

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
  //
  // `quality` is optional and PARALLEL to `out`: when it is given, quality[i]
  // is the C1's 0..63 return strength for out[i], and it is emptied on every
  // path that empties `out`, so the two can never disagree in length. A
  // pointer rather than a second overload because reactive::step does not want
  // it and the scan feed does, and a Ray carries no quality on purpose - the
  // driver reads distances, and a field it must ignore is a field it will one
  // day read by mistake.
  [[nodiscard]] Bool grab(Vec<reactive::Ray>& out, Int32 timeoutMs = 2000, Vec<UInt8>* quality = nullptr);

  // The device's identity as it reported it at open(): model, firmware,
  // hardware revision, serial. Empty when nothing is open.
  [[nodiscard]] Str info();

  // The same identity, as numbers, for a program that has to WRITE it rather
  // than print it - the scan feed puts these on the wire one field at a time
  // and a hub parses them back. info() is the sentence; this is the record.
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

  // The device's self-report, re-read from the device when the motor is off
  // and repeated from the last reading when it is spinning - the SDK serialises
  // commands against the scan stream, and a health query mid-scan is a stall
  // in the middle of a revolution. Empty when nothing is open.
  [[nodiscard]] Str health();

}
