// carrules - what the Car decides, with no clock, port or socket of its own: the
// flags, lidar rays to a Scan, the Pico's replies, the ESC and STEER lines, the
// Governor that picks what the minder sends each pass, and the Sender that holds
// a throttle line to the send gap as it goes out. Times arrive as numbers and the
// port as a function, so tests/test_carrules.cxx runs all of it on any compiler.
#pragma once

#include "shared.hxx"

#include "car.hxx"
#include "link.hxx"
#include "reactive.hxx"

namespace carrules
{
  struct Options
  {
      Bool drive = false;                              // --drive; otherwise a dry run
      Bool help = false;                               // --help
      Bool viewer = true;                              // false with --no-viewer
      Int32 seconds = 0;                               // --seconds; 0 is no limit
      Float32 forwardDeg = bibo::LIDAR_FORWARD_DEG;    // --forward
      Str lidarPort = bibo::LIDAR_PORT;                // --lidar
      Str picoPort = bibo::PICO_PORT;                  // --pico
  };

  // The flag list car.hxx gives, for --help.
  [[nodiscard]] Str usage();

  // Reads argv[1..] into out. false, with why naming the problem, for an unknown
  // flag, a missing or malformed value, or --drive while measured (the committed
  // LIDAR_FORWARD_MEASURED) is false and no --forward was given.
  [[nodiscard]] Bool parseArgs(Int32 argc, Char** argv, Bool measured, Options& out, Str& why);

  // Revolution number rev from lidar::grab, in the car's frame. Rays with no
  // return (distMm not above 0) are dropped, millimetres become metres, and each
  // raw angle becomes its bearing from forwardDeg, folded to -180..180.
  [[nodiscard]] bibo::Scan toScan(const Vec<reactive::Ray>& rays, Float32 forwardDeg, UInt32 rev);

  // What the Pico last said about itself. The Int32 fields come from the latest
  // OK drive line (firmware/app/main.cxx printDrive); -1 is "that line did not
  // carry the key".
  struct Board
  {
      Int32 armed = -1;
      Int32 servoOn = -1;
      Int32 escUs = -1;
      Int32 escMinUs = -1;
      Int32 escMaxUs = -1;
      Int32 escRevUs = -1;
      Int32 stale = -1;             // 1: the Pico's watchdog fired before this command
      // The wheel encoder, as the Pico counts its hall sensors: six ticks a
      // motor turn, forward positive. The error counts are saturated bytes.
      Int32 ticks = -1;
      Int32 ticksPerS = -1;
      Int32 hallSkips = -1;
      Int32 hallInvalid = -1;
      Bool stopAnswered = false;    // an OK stop has arrived
      UInt32 errors = 0;            // ERR lines
  };

  // Folds one line from the Pico into b. An OK drive line replaces every Int32
  // field. true for an OK drive line.
  Bool fold(Board& b, const Str& line);

  // The ESC pulse for throttle t within the Pico's reported band, whose bottom is
  // raised to bibowire::ESC_NEUTRAL_US: there is no reverse. t above 1 counts as 1.
  // ESC_NEUTRAL_US for t not above 0 or NaN, and for a band unknown or empty.
  [[nodiscard]] Int32 forwardPulse(Float32 t, Int32 escMinUs, Int32 escMaxUs);

  // The STEER line for steer -1..1: clamped, NaN counts as 0.
  [[nodiscard]] Str steerLine(Float32 steer);

  // The pulse in an "ESC <us>" line, or -1 for any other line.
  [[nodiscard]] Int32 pulseIn(const Str& line);

  // How a run ends.
  enum class End
  {
      END_NONE = 0,         // still running
      END_FINISHED,         // the program called finish()
      END_SIGNAL,           // SIGINT, SIGTERM or SIGHUP
      END_ESTOP,            // a viewer's ESTOP button
      END_SECONDS,          // --seconds ran out
      END_HELP,             // --help
      END_BAD_FLAGS,
      END_OPEN_FAILED,      // the lidar or the Pico would not open
      END_NO_REVOLUTION,    // no good revolution within FIRST_REVOLUTION_MS
      END_ARM_TIMEOUT,      // the Pico did not confirm within ARM_CONFIRM_MS
      END_LIDAR_LOST,       // no good revolution for LIDAR_LOST_MS
      END_LINK_LOST,        // the Pico's port closed
      END_PICO_DISARMED,    // armed=0 after arming: it rebooted, or someone sent STOP
      END_PICO_WATCHDOG,    // stale=1 after arming: its watchdog fired
      END_SEND_GAP,         // more than SEND_GAP_STOP_MS between two sends
      END_VIEWER_STUCK,     // the viewer's feed stuck for VIEWER_STUCK_MS
  };

  [[nodiscard]] CharSeq endName(End e);

  // finish()'s exit code, as car.hxx lists it.
  [[nodiscard]] Int32 exitCode(End e, UInt64 goodRevolutions);

  enum class Arm
  {
      ARM_IDLE = 0,       // not asked
      ARM_REQUESTED,      // the next pass sends ESC ARM and SERVO ON
      ARM_PENDING,        // sent; waiting for the Pico to confirm
      ARM_CONFIRMED,      // armed=1, servo_on=1 and a forward band, after OK stop
  };

  // What the minder knows at one pass. Milliseconds on one steady clock; -1 is never.
  struct Inputs
  {
      Int64 nowMs = 0;
      Vec<Str> replies;             // the Pico's lines since the last pass, in order
      Bool linkLost = false;        // the Pico's port closed
      Bool signal = false;          // SIGINT, SIGTERM or SIGHUP
      Bool estop = false;           // a viewer latched ESTOP
      Bool viewerStuck = false;     // the viewer's feed stuck for VIEWER_STUCK_MS
      Int64 scanMs = -1;            // the newest good revolution
      Float32 throttle = 0.0f;      // the last drive()
      Float32 steer = 0.0f;
      Int64 driveMs = -1;           // when drive() was last called
      Int64 sentMs = -1;            // the last line the port accepted
      Int64 gapMs = 0;              // the longest gap between two accepted lines since
                                    // the last pass (Sender::takeGapMs)
  };

  // Decides what the minder sends, pass by pass. Only the minder's thread uses it.
  class Governor
  {
  public:
      // dryRun: nothing is ever sent, and only the latches that need no Pico
      // apply. seconds: --seconds, 0 for no limit.
      Governor(Bool dryRun, Int32 seconds);

      // The lines that open the Pico: STOP, PING, then trim in order. Empty when dry.
      [[nodiscard]] Vec<Str> open(const Vec<Str>& trim);

      // The next pass arms. A dry run counts as confirmed on that pass.
      Void requestArm();

      // Ends the run for a reason found outside; the first reason stands.
      Void end(End why);

      // One pass: folds in.replies, applies the latches, returns the lines to send.
      // Once ended, exactly STOP every pass (nothing when dry).
      [[nodiscard]] Vec<Str> pass(const Inputs& in);

      [[nodiscard]] Arm armState() const;
      [[nodiscard]] End ended() const;
      [[nodiscard]] const Board& board() const;

  private:
      Bool dry = false;
      Int64 secondsMs = 0;
      Arm arm = Arm::ARM_IDLE;
      End reason = End::END_NONE;
      Board pico;
      Int64 armSentMs = -1;
      Int64 armedMs = -1;
      Int64 heardMs = -1;
  };

  // How long a forward pulse's write may still take at nowMs and finish within
  // SEND_GAP_STOP_MS of sentMs, when the port last accepted a line. 0 or less -
  // and always while nothing has been accepted (sentMs -1) - is too late to send.
  [[nodiscard]] Int64 pulseWaitMs(Int64 nowMs, Int64 sentMs);

  // The Pico's port as the minder sends to it. The clock and the send are handed
  // in, so a test can stall a write with no port. A line counts as sent when send
  // returns RESULT_OK, and the clock is read then. Only one thread uses it.
  class Sender
  {
  public:
      using ClockFn = Fn<Int64()>;
      using SendFn = Fn<carlink::Result(const Str& line, Int32 waitMs)>;

      Sender(ClockFn clockMs, SendFn send);

      // One line, waiting at most waitMs for the port to take it.
      carlink::Result send(const Str& line, Int32 waitMs = carlink::WRITE_WAIT_MS);

      // One pass's lines, in order. A forward pulse goes only if every line this
      // pass sent before it, the first included, went within SEND_GAP_STOP_MS of
      // the line before it, and only with the wait pulseWaitMs leaves, judged as
      // it goes: otherwise a line may have reached the Pico after its watchdog
      // fired, and the pulse would put throttle back. A pulse that cannot go, or
      // does not, is replaced by STOP, which ends the pass and sets ended to
      // END_LINK_LOST when the port had closed, END_SEND_GAP otherwise. Returns
      // the lines sent or tried, that STOP included.
      [[nodiscard]] Vec<Str> pass(const Vec<Str>& lines, End& ended);

      // When the port last accepted a line; -1 before the first.
      [[nodiscard]] Int64 sentMs() const;

      // The longest gap between two accepted lines, over the run.
      [[nodiscard]] Int64 worstGapMs() const;

      // The longest gap between two accepted lines since the last call.
      [[nodiscard]] Int64 takeGapMs();

  private:
      ClockFn clock;
      SendFn write;
      Int64 lastMs = -1;
      Int64 worstMs = 0;
      Int64 recentMs = 0;
      Int64 passMs = 0;      // the longest gap within the pass being sent
  };
}
