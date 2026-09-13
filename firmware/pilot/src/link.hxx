// The companion board's link to the car: the Pico's USB CDC port, /dev/ttyACM0.
//
// ---------------------------------------------------------------------------
// WHAT IMPLEMENTS IT
//
//   Linux     link.cxx, behind `#if defined(__linux__)`: POSIX termios on a
//             serial device, raw 8N1, a reader thread that splits the byte
//             stream into lines. The Pico's CDC ignores the baud rate; 115200
//             is convention, not protocol.
//   others    the refusing path. open() returns RESULT_NO_PLATFORM and every
//             other call refuses, rather than a stub that reports success while
//             a STOP silently goes nowhere.
#pragma once

#include "shared.hxx"

// carlink, not link: `link` is a POSIX function in <unistd.h>, which glibc's
// <thread> drags in, and a namespace with a libc function's name will not
// compile on the board this file exists for. MSVC never noticed.
namespace carlink
{

  enum class Result
  {
      RESULT_OK = 0,

      // No transport is compiled in for this platform. The honest answer on
      // every machine but the Pi, and never a reason to retry.
      RESULT_NO_PLATFORM,

      // Three different absences at open(), told apart because each sends a
      // person to a different place. "Unplugged" is not "broken".
      RESULT_NO_PORT,        // the named device is not there: check the cable
      RESULT_DENIED,         // it is there and we may not open it: check groups
      RESULT_BUSY,           // it is there and another program holds it: close that
      RESULT_OPEN_FAILED,    // it is there, we opened it, and it is not a serial
                             // port we can configure - /dev/null is the example

      RESULT_NOT_OPEN,       // a call that needs a link, without one
      RESULT_WRITE_FAILED,
      RESULT_CLOSED,         // it went away mid-session
  };

  [[nodiscard]] CharSeq why(Result r);

  // The system's own words for the last failure - "open(/dev/ttyACM0): No such
  // file or directory" - or empty when nothing has failed. why() names the
  // kind; this carries the evidence, for a log line.
  [[nodiscard]] Str detail();

  // How the link behaves. Settled before the transport existed, because the
  // numbers outlive whichever transport ends up carrying them.
  struct Config
  {
      // The device. "/dev/ttyACM0" on the Pi.
      Str where;

      // 115200 is what the firmware's USB CDC enumerates at. Must be a rate
      // termios has a name for, or open() is RESULT_OPEN_FAILED.
      Int32 baud = 115200;

      // How long a command may go unanswered before the link is called dead.
      //
      // NOT a timeout on any one reply - some commands answer immediately and
      // some do not answer at all. It is the longest the board may be entirely
      // silent while the car is moving, and the right response to exceeding it
      // is to stop the car, not to retry.
      Int32 silenceMs = 500;
  };

  // Opens the link. Idempotent: a second call while open is RESULT_OK. After
  // the board went away (RESULT_CLOSED from send() or drain()) a call here
  // tears the dead link down and tries again - which is how a caller recovers
  // from an unplug without an extra step.
  //
  // The port is taken EXCLUSIVELY (TIOCEXCL), so a second program opening it
  // gets RESULT_BUSY rather than a share of the same byte stream. Two readers
  // on one port each see half the lines and both report a broken board.
  [[nodiscard]] Result open(const Config& cfg);

  // Closes it. Safe with no link, safe twice, and it joins the reader thread
  // before returning so nothing touches the device afterwards.
  Void close();

  // True while the link can carry lines. False before open(), after close(),
  // and once the reader has seen the device go away - a link that still holds
  // a descriptor to a board that is gone is not open in any sense a caller
  // cares about.
  [[nodiscard]] Bool isOpen();

  // How long send() waits, unless told otherwise, for the device to take a line.
  // A USB CDC port whose board has stopped reading fills its buffer and then
  // blocks writers forever, so a stall is counted as a drop.
  constexpr Int32 WRITE_WAIT_MS = 100;

  // Sends one line. The newline is added here so no caller has to remember it -
  // a command without one is a command the board waits forever to finish.
  //
  // Blocks for at most waitMs if the device will not take the bytes, then gives
  // up and counts a drop. A stalled write is a stale command; holding the
  // autonomy loop on it is worse than losing it, because the next tick will send
  // a fresher one. A caller with a sooner deadline - a throttle line that must
  // not arrive late - passes a shorter wait.
  //
  // A stall can leave the head of the line in the device's buffer, and the
  // board keeps collecting it. The next send() ends that fragment with a
  // newline before its own line, so the board rejects the fragment on its own
  // ("ERR unknown command") rather than the fragment with the next command -
  // possibly a STOP - stuck to its tail.
  //
  // Every call is counted exactly once, as transmitted or as dropped, so
  //     calls to send() == txLines() + dropped()
  // holds whenever anyone looks.
  // A line sent with no link is dropped, not skipped: a stop that went nowhere
  // has to show up in a number somewhere.
  [[nodiscard]] Result send(const Str& line, Int32 waitMs = WRITE_WAIT_MS);

  // Takes any complete lines that have arrived, appending to `out`. A partial
  // line is held until the rest of it turns up rather than delivered short.
  //
  // Returns RESULT_OK with nothing appended when the board simply has not said
  // anything, which is the normal case and not a fault. Returns RESULT_CLOSED
  // once the device has gone - and still appends whatever it said before it
  // went, because "STOP ok" followed by an unplug is a sequence worth seeing.
  [[nodiscard]] Result drain(Vec<Str>& out);

  // Writes "\nSTOP\n" to the open port with one write(2) and does nothing else:
  // no lock, no allocation, no counter, errno left as it was. Safe inside a
  // signal handler, which is what it is for. Nothing happens with no link. The
  // leading newline ends any line a send() was interrupted in the middle of.
  Void stopFromSignal();

  // Milliseconds since the last line arrived, or -1 with no link.
  //
  // Counted from open() until the board's first line, not from the first line:
  // a board that never speaks is exactly the case this number exists to catch,
  // and "no data yet" reported as -1 would hide it behind the same value as
  // "no link". Compared against Config::silenceMs by whatever owns the safety
  // decision - this header does not stop the car on its own, because a
  // transport that decides policy is a transport nobody can test.
  [[nodiscard]] Int32 silentForMs();

  // Lines that reached the device, lines that arrived from it, and lines send()
  // accepted that never reached it. Process-lifetime counts, across reopens.
  [[nodiscard]] UInt64 txLines();
  [[nodiscard]] UInt64 rxLines();
  [[nodiscard]] UInt64 dropped();

}
