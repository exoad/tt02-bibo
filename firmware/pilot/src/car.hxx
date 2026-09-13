// car.hxx - the one header a car program includes.
//
// A car program runs on the Orange Pi (bibobox), because that is where the
// lidar is plugged in. The Car talks to the Pico for you, and the Pico moves the
// steering servo and the ESC. Start by copying programs/forward.cxx.
//
//   DECLARE   bibo::Car car(argc, argv);            what this run may do, from its flags
//   BIND      if(!car.arm()) return car.finish();   lidar seen, ESC armed, steering engaged
//   RUN       while(car.ok()) { bibo::Scan s = car.scan(); ...; car.drive(throttle, steer); }
//             return car.finish();
//
// ---- UNITS AND DIRECTIONS, EVERYWHERE IN THIS HEADER --------------------------
//   distance  metres, measured from the lidar's spin axis - NOT from the bumper
//   bearing   degrees, -180..180. 0 is straight ahead, POSITIVE IS RIGHT
//   steer     -1..1. NEGATIVE IS LEFT, 0 is the car's trimmed centre
//   throttle  0..1 of THIS car's forward range as trimmed (the Pico's
//             esc_min..esc_max). 0 is neutral. There is no reverse: below 0 is 0.
//
// ---- FLAGS, read by the constructor. Anything else is refused. ----------------
//   (none)          DRY RUN. The Pico is never opened, so the car cannot move.
//                   Everything else runs; the status line shows what would be sent.
//   --drive         really drive
//   --seconds N     stop the run N seconds after arm()
//   --forward DEG   the raw lidar angle that points straight ahead, for this run
//   --lidar PORT    default LIDAR_PORT
//   --pico PORT     default PICO_PORT
//   --no-viewer     do not serve the Windows viewer on bibowire::PORT
//   --help          print this list
//
// ---- WHAT THE CAR DOES WITHOUT BEING ASKED ------------------------------------
//   - Opens the Pico with STOP (in case an earlier run died armed), then PING,
//     then the saved trim from ~/.config/bibo/trim.txt. Nothing is armed yet.
//   - Arms only inside arm(), only after the lidar has delivered a revolution,
//     and only counts it done when the Pico itself reports armed, steering on,
//     and a forward range.
//   - Sends your latest drive() to the Pico every MINDER_MS from its own thread,
//     whatever your loop is doing, so the Pico's watchdog
//     (bibowire::PICO_DEADMAN_MS) never fires while this program is alive.
//   - Holds the throttle at neutral, steering where it is, while your last
//     drive() is older than DRIVE_FRESH_MS, the newest good revolution is older
//     than SCAN_FRESH_MS, or the Pico has said nothing for PICO_QUIET_MS. It
//     drives again as soon as all three are fresh.
//   - Sends STOP (neutral, disarm, steering released) and makes ok() false for
//     good on: Ctrl-C, SIGTERM, SIGHUP (the ssh session dropped), --seconds, a
//     viewer's ESTOP button, no good revolution for LIDAR_LOST_MS, the Pico's
//     port gone, the Pico reporting that it disarmed or that its watchdog fired,
//     more than SEND_GAP_STOP_MS between two lines sent to the Pico - judged
//     again as each throttle line goes out - or the viewer's feed stuck for
//     VIEWER_STUCK_MS, so its ESTOP could no longer arrive.
//   - Sends STOP on every way out: finish(), ~Car(), exit(), an uncaught
//     exception, and a crash (SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT). A stack
//     overflow is covered on the threads that have a signal stack: the one that
//     made the Car, and the minder, lidar and printer threads - not the viewer
//     feed's or the Pico reader's. After a Ctrl-C, a second Ctrl-C kills the
//     program outright.
//   - Left to the Pico's own watchdog (bibowire::PICO_DEADMAN_MS: throttle
//     neutral, steering held, still armed until the next run's opening STOP):
//     SIGKILL, a hung Orange Pi, and a stack overflow on a thread without a
//     signal stack. A power cut or a pulled USB cable turns the Pico off too, so
//     no watchdog runs and the ESC's own reaction to a lost signal decides.
//   - Stopping is not instant: the Pico slews the throttle back to neutral
//     rather than cutting it, and the lidar is up to one revolution behind.
//     Leave room for both in your distances.
//   - Prints one status line a second, from its own thread: a console that stops
//     taking output (a hung ssh session) loses lines, never the Pico's feed.
//     The viewer's feed prints too while a viewer is connected, so then a
//     stalled console can end the run (VIEWER_STUCK_MS).
//   - Serves the viewer on bibowire::PORT. A viewer can watch and press ESTOP;
//     it cannot arm, drive or change the trim.
//
// Do not install your own handlers for the signals above: they would replace
// the ones that send STOP.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"

namespace bibo
{

  // The raw lidar angle (the C1's own 0..360, clockwise seen from above) that
  // points straight ahead. Measure it once with a dry run - docs/start.md,
  // step 2 - set both lines, and commit. While LIDAR_FORWARD_MEASURED is false,
  // --drive is refused unless the run passes --forward DEG itself.
  constexpr Float32 LIDAR_FORWARD_DEG = 0.0f;
  constexpr Bool LIDAR_FORWARD_MEASURED = false;

  constexpr CharSeq LIDAR_PORT = "/dev/ttyUSB0";
  constexpr CharSeq PICO_PORT = "/dev/ttyACM0";

  // Half the car's width plus a margin: the strip Scan::ahead() looks along.
  constexpr Float32 CAR_HALF_WIDTH_M = 0.16f;

  // How far either side of straight ahead Scan::ahead() looks at all, so points
  // beside the lidar do not read as blocking the strip.
  constexpr Float32 SCAN_FRONT_ARC_DEG = 75.0f;

  // What ahead() and nearest() return when nothing is there.
  constexpr Float32 SCAN_FAR_M = 12.0f;

  // Fewest points a revolution needs before it is believed at all.
  constexpr Int32 SCAN_MIN_POINTS = 40;

  // How many points must agree before a distance counts: the SCAN_MIN_HITS-th
  // nearest is used, so one speck of dust is not a wall.
  constexpr Int32 SCAN_MIN_HITS = 3;

  // ---- timing, in milliseconds --------------------------------------------------

  // How often the Car sends the Pico your latest drive().
  constexpr Int32 MINDER_MS = 40;

  // More than this between two lines sent while arming or armed ends the run
  // with STOP, and a throttle line goes out only if it leaves within this of the
  // line before it: a later one could reach a Pico whose watchdog has already
  // neutralised the ESC, and make it lurch.
  constexpr Int32 SEND_GAP_STOP_MS = bibowire::PICO_DEADMAN_MS - bibowire::PICO_HOP_BUDGET_MS;

  static_assert(
      2 * MINDER_MS + bibowire::PICO_HOP_BUDGET_MS <= bibowire::PICO_DEADMAN_MS,
      "one late pass must neither end the run nor let the Pico's watchdog fire"
  );

  // Older than these and the throttle is held at neutral until they are fresh.
  constexpr Int32 DRIVE_FRESH_MS = 300;                          // your last drive()
  constexpr Int32 SCAN_FRESH_MS = 300;                           // the newest good revolution
  constexpr Int32 PICO_QUIET_MS = bibowire::PICO_DEADMAN_MS;     // the Pico's last line

  // No good revolution for this long after arm() ends the run with STOP.
  constexpr Int32 LIDAR_LOST_MS = 2000;

  // The viewer's feed refreshes what viewfeed::drive() reads on every pass. A copy
  // older than this means that thread is stuck, on a stalled console say, so a
  // viewer's ESTOP could no longer arrive, and the run ends with STOP.
  constexpr Int32 VIEWER_STUCK_MS = 500;

  // How long arm() waits for the first good revolution (spin-up alone is over
  // 2 s), and then for the Pico to confirm it is armed.
  constexpr Int32 FIRST_REVOLUTION_MS = 6000;
  constexpr Int32 ARM_CONFIRM_MS = 1000;

  // The longest scan() blocks.
  constexpr Int32 SCAN_WAIT_MS = 250;

  // One lidar return, already in the car's frame.
  struct Point
  {
      Float32 bearingDeg = 0.0f;   // -180..180 from straight ahead, positive right
      Float32 distanceM = 0.0f;    // > 0; directions with no return are left out
  };

  // One revolution of the lidar: about 500 points, about 10 a second.
  struct Scan
  {
      UInt32 revolution = 0;       // counts up from 1; 0 means nothing arrived
      Vec<Point> points;

      // No revolution, or fewer than SCAN_MIN_POINTS points: nothing to drive on.
      [[nodiscard]] Bool blind() const;

      // Metres AHEAD, along the car's axis, to the SCAN_MIN_HITS-th nearest point
      // that is within SCAN_FRONT_ARC_DEG of straight ahead and within halfWidthM
      // to either side. SCAN_FAR_M when that strip is clear. 0 when blind(), or
      // when halfWidthM is negative or NaN: a scan that cannot see reads as
      // something touching the car.
      [[nodiscard]] Float32 ahead(Float32 halfWidthM = CAR_HALF_WIDTH_M) const;

      // Metres, in a straight line, to the SCAN_MIN_HITS-th nearest point whose
      // bearing is within fromDeg..toDeg, both included. SCAN_FAR_M when there are
      // fewer than that; 0 when blind() or when fromDeg > toDeg.
      [[nodiscard]] Float32 nearest(Float32 fromDeg, Float32 toDeg) const;
  };

  // The car and its lidar, for one run. One per program: a second one refuses.
  class Car
  {
  public:
      // Reads the flags, opens the lidar and - with --drive - the Pico, and
      // starts the lidar motor and the viewer. Takes a few seconds. Never throws
      // and never exits: a failure is printed and makes ok() false.
      Car(Int32 argc, Char** argv);

      // Calls finish() if you did not.
      ~Car();

      Car(const Car&) = delete;
      Car& operator=(const Car&) = delete;

      // Waits up to FIRST_REVOLUTION_MS for the lidar's first good revolution,
      // then arms the ESC and engages the steering, and waits up to
      // ARM_CONFIRM_MS for the Pico to confirm both. Prints the forward range the
      // Pico is using and where it came from. false, with the reason printed,
      // otherwise - and STOP has been sent by then. In a dry run: waits for the lidar
      // the same way, then returns true without arming anything.
      [[nodiscard]] Bool arm();

      // Keep looping. false when opening or arming failed, and false for good
      // once the run has been stopped (see above). The car has already been
      // sent STOP by the time this is false.
      [[nodiscard]] Bool ok() const;

      // Blocks until a revolution newer than the last one returned, at most
      // SCAN_WAIT_MS. On timeout, or once ok() is false, returns a blind() scan.
      [[nodiscard]] Scan scan();

      // What the car should do from now on; see UNITS. Values are clamped and
      // NaN counts as 0. Call it after every scan(): an order older than
      // DRIVE_FRESH_MS is replaced by neutral. Before arm() has succeeded it
      // moves nothing, and says so once on the console.
      Void drive(Float32 throttle, Float32 steer);

      // Sends STOP, stops the lidar motor, closes the ports and the viewer, and
      // prints one line saying why the run ended. Returns the exit code for main:
      //   0  ended on request - your own finish(), Ctrl-C, SIGTERM, SIGHUP,
      //      --seconds, a viewer's ESTOP - after at least one good revolution; or --help
      //   1  anything else: a device would not open, arm() failed, the lidar or
      //      the Pico was lost, or nothing was measured
      //   2  the flags were wrong
      // Safe to call twice; a later call returns the same code.
      [[nodiscard]] Int32 finish();

  private:
      struct Inner;
      UniqPtr<Inner> inner;
  };

}
