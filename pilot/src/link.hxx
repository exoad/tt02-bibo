// The companion board's link to the car.
//
// Same lines as hub/src/pico_link.hxx moves, from the other side of the cable.
// The hub is a person's client; this is the autonomy's.
//
// ---------------------------------------------------------------------------
// THERE IS NO IMPLEMENTATION YET, AND THAT IS SAID OUT LOUD.
//
// open() returns RESULT_NO_PLATFORM on every host this currently compiles for,
// and every other call refuses. The Orange Pi is not here; the serial code it
// needs is POSIX termios, which cannot be written honestly on the Windows
// machine this repository is developed on, and cannot be tested at all without
// the board.
//
// The alternative was a stub that returns RESULT_OK and pretends the car is
// listening. That is the failure this project keeps finding in its own code -
// something that reports success while doing nothing - and it is far worse here
// than usual, because the thing silently not happening would be a STOP command.
//
// So: the shape is settled, the errors are named, and the one function that
// would lie instead refuses. When the board arrives, linkPosix.cxx implements
// this header and nothing above it changes.
//
// ---------------------------------------------------------------------------
// WHAT WILL IMPLEMENT IT
//
//   USB CDC   the Pico appears as /dev/ttyACM0. Same cable the hub uses, moved
//             from the laptop to the Pi. This is the one to write first: it is
//             the link that exists on a bench with no network.
//   UDP       the firmware's WIFI JOIN already carries the same text lines, so
//             a Pi on the same network can drive a car it is not wired to.
//             Untested end to end - the laptop here is on 5 GHz and the
//             CYW43439 is 2.4 GHz only.
#pragma once

#include "shared.hxx"

namespace link
{

  enum class Result
  {
      RESULT_OK = 0,

      // No transport is compiled in for this platform. The honest answer today
      // on every machine, and never a reason to retry.
      RESULT_NO_PLATFORM,

      RESULT_NO_PORT,        // the named device is not there
      RESULT_DENIED,         // it is there and we may not open it
      RESULT_NOT_OPEN,       // a call that needs a link, without one
      RESULT_WRITE_FAILED,
      RESULT_CLOSED,         // it went away mid-session
  };

  [[nodiscard]] CharSeq why(Result r);

  // How the link should behave once there is one. Settled now because the
  // numbers outlive whichever transport ends up carrying them.
  struct Config
  {
      // The device, or the car's address for UDP. "/dev/ttyACM0" on the Pi.
      Str where;

      // 115200 is what the firmware's USB CDC enumerates at. Ignored by UDP.
      Int32 baud = 115200;

      // How long a command may go unanswered before the link is called dead.
      //
      // NOT a timeout on any one reply - some commands answer immediately and
      // some do not answer at all. It is the longest the board may be entirely
      // silent while the car is moving, and the right response to exceeding it
      // is to stop the car, not to retry.
      Int32 silenceMs = 500;
  };

  // Opens the link. Idempotent: a second call while open is RESULT_OK.
  [[nodiscard]] Result open(const Config& cfg);

  // Closes it. Safe with no link.
  Void close();

  [[nodiscard]] Bool isOpen();

  // Sends one line. The newline is added here so no caller has to remember it -
  // a command without one is a command the board waits forever to finish.
  [[nodiscard]] Result send(const Str& line);

  // Takes any complete lines that have arrived, appending to `out`. A partial
  // line is held until the rest of it turns up rather than delivered short.
  //
  // Returns RESULT_OK with nothing appended when the board simply has not said
  // anything, which is the normal case and not a fault.
  [[nodiscard]] Result drain(Vec<Str>& out);

  // Milliseconds since the last line arrived, or -1 with no link and no data.
  // Compared against Config::silenceMs by whatever owns the safety decision -
  // this header does not stop the car on its own, because a transport that
  // decides policy is a transport nobody can test.
  [[nodiscard]] Int32 silentForMs();

} // namespace link
