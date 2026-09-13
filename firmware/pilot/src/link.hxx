// The companion board's link to the Pico's USB CDC port, /dev/ttyACM0.
//
//   Linux     link.cxx behind `#if defined(__linux__)`: POSIX termios, raw 8N1, and
//             a reader thread that splits the byte stream into lines.
//   others    the refusing path: open() returns RESULT_NO_PLATFORM and every other
//             call refuses, never a stub that reports success while a STOP goes
//             nowhere.
#pragma once

#include "shared.hxx"

// carlink, not link: glibc's <thread> pulls in <unistd.h>, whose POSIX link()
// will not share its name with a namespace. MSVC does not show the clash.
namespace carlink
{
  enum class Result
  {
      RESULT_OK = 0,

      // No transport is compiled in for this platform; never a reason to retry.
      RESULT_NO_PLATFORM,

      // Told apart at open() because each sends a person somewhere different.
      RESULT_NO_PORT,        // the named device is not there: check the cable
      RESULT_DENIED,         // it is there and we may not open it: check groups
      RESULT_BUSY,           // it is there and another program holds it: close that
      RESULT_OPEN_FAILED,    // it opened, and is not a serial port we can
                             // configure - /dev/null is the example
      RESULT_NOT_OPEN,       // a call that needs a link, without one
      RESULT_WRITE_FAILED,
      RESULT_CLOSED,         // it went away mid-session
  };

  [[nodiscard]] CharSeq why(Result r);

  // The system's own words for the last failure - "open(/dev/ttyACM0): No such
  // file or directory" - or empty when nothing has failed. why() names the kind.
  [[nodiscard]] Str detail();

  struct Config
  {
      // The device. "/dev/ttyACM0" on the Pi.
      Str where;

      // The Pico's CDC ignores the rate, but termios must have a name for it,
      // or open() is RESULT_OPEN_FAILED.
      Int32 baud = 115200;

      // The longest the board may be entirely silent while the car moves before
      // the link counts as dead. NOT a timeout on one reply: some commands never
      // answer. Exceeding it means stop the car, not retry.
      Int32 silenceMs = 500;
  };

  // Idempotent: a second call while open is RESULT_OK. After RESULT_CLOSED from
  // send() or drain(), a call tears the dead link down and opens again, so a
  // caller recovers from an unplug with open() alone.
  //
  // The port is taken EXCLUSIVELY (TIOCEXCL): a second program gets RESULT_BUSY,
  // rather than two readers each seeing half the lines.
  [[nodiscard]] Result open(const Config& cfg);

  // Safe with no link and safe twice. Joins the reader thread before returning,
  // so nothing touches the device afterwards.
  Void close();

  // False before open(), after close(), and once the reader has seen the device
  // go away, even while a descriptor is still held.
  [[nodiscard]] Bool isOpen();

  // send()'s default wait for the device to take a line. A CDC port whose board
  // has stopped reading blocks writers forever, so a stall counts as a drop.
  constexpr Int32 WRITE_WAIT_MS = 100;

  // Sends one line; the newline is added here, since without one the board waits
  // forever for the command to finish. Blocks at most waitMs, then gives up and
  // counts a drop: a stalled command is stale, and the next tick sends a fresher
  // one. A caller with a sooner deadline, a throttle line that must not arrive
  // late, passes a shorter wait.
  //
  // A stall can leave the head of the line in the device's buffer. The next
  // send() ends that fragment with a newline first, so the board rejects the
  // fragment alone ("ERR unknown command") instead of with the next command -
  // possibly a STOP - stuck to its tail.
  //
  // Every call counts exactly once, so calls to send() == txLines() + dropped().
  // A line sent with no link is dropped, not skipped: a STOP that went nowhere
  // shows up in a number.
  [[nodiscard]] Result send(const Str& line, Int32 waitMs = WRITE_WAIT_MS);

  // Appends any complete lines that have arrived to `out`, which is NOT cleared,
  // so a caller gathering from several sources keeps its earlier lines. A
  // partial line waits for the rest of it.
  //
  // RESULT_OK with nothing appended is the normal quiet case, not a fault.
  // RESULT_CLOSED once the device has gone, still appending what it said before
  // it went.
  [[nodiscard]] Result drain(Vec<Str>& out);

  // Writes "\nSTOP\n" to the open port with one write(2) and nothing else: no
  // lock, no allocation, no counter, errno left as it was. Safe inside a signal
  // handler, which is what it is for. Nothing happens with no link. The leading
  // newline ends any line a send() was interrupted in the middle of.
  Void stopFromSignal();

  // Milliseconds since the last line arrived, or -1 with no link. Counted from
  // open() until the board's first line, so a board that never speaks reads as
  // silent, not as "no link". Whatever owns the safety decision compares it with
  // Config::silenceMs; the link never stops the car itself.
  [[nodiscard]] Int32 silentForMs();

  // Lines that reached the device, lines that arrived from it, and lines send()
  // accepted that never reached it. Process-lifetime counts, across reopens.
  [[nodiscard]] UInt64 txLines();
  [[nodiscard]] UInt64 rxLines();
  [[nodiscard]] UInt64 dropped();
}
