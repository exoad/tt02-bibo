// The companion board's program: a lidar revolution in, two lines to the car
// out, for as long as it is left running.
//
//   pilot [--lidar PORT] [--pico PORT] [--dry] [--arm] [--forward DEG] [--seconds N]
//
// ---------------------------------------------------------------------------
// WHAT THIS IS
//
// The loop firmware/pilot exists for, in its first shape. reactive.hxx is the
// dumb layer - no map, no path, no odometry - and this is the program that
// gives it a real lidar and a real car. autonomy.hxx, the pursuit along a
// planned path, is still a stub and is not touched here; when it is real it
// gets its own loop or joins this one, and that is a decision for the day an
// encoder exists.
//
// Everything below is glue. The behaviour is reactive::step's, the protocol is
// proto's, the transport is carlink's and the sensor is lidar's; this file
// decides only how they are sequenced and what happens when one of them is
// late. Those are the decisions written down in the sections that follow.
//
// ---------------------------------------------------------------------------
// THE LIDAR IS THE CLOCK
//
// lidar::grab() blocks until one revolution has arrived, and that is the tick.
// The C1 turns at about 10 Hz, so the car gets a fresh decision every ~100 ms
// and nothing in between, because nothing new has been seen in between. There
// is no timer and no sleep in this loop, and dtMs handed to the module is what
// the wall clock says passed - not the nominal 100 ms - so a slow revolution
// counts as the longer interval it was.
//
// The number that shapes the loop is the BOARD'S deadman. firmware/app/main.cxx
// stops the car when no command has arrived for DEADMAN_MS, 400 ms. This loop
// waits at most REV_WAIT_MS, half of that, for a revolution and then sends
// anyway: a steer and a neutral, because a tick with no scan behind it has
// nothing to say about throttle. That keeps two different failures apart. The
// board's deadman firing means the Pi has gone quiet - a crash, a cable, a
// hung process - and is the board's business. A late revolution is THIS
// program's business, and it is answered by stopping the car on purpose, on
// time, rather than by coasting for up to 400 ms until the board notices.
//
// The wait is spent inside the SDK, not in a sleep, so a revolution that turns
// up at 150 ms is acted on at 150 ms.
//
// ---------------------------------------------------------------------------
// WHAT THE CAR IS TOLD
//
// Steering always, as a fraction. Throttle only for a forward decision from a
// scan the module trusted. That is the rule hub/src/app_ui.cxx reactiveSend()
// applies, and it is copied here deliberately rather than shared: the hub maps
// onto the Drive view's live idle..full, this maps onto the numbers in
// cal.hxx, and the two programs must be allowed to differ in exactly that.
//
// Reverse is sent as NEUTRAL. The board is forward-only - it refuses anything
// below 1500 us, and reverse on the QuicRun is a brake-then-reverse sequence
// nobody has written - so the module's MODE_REVERSE is a stop here and its
// reverse commitment becomes a pause. A known gap, stated: a car that stops at
// a wall is the honest version of a car that was told to back up and did not.
//
// The pulse range is THROTTLE_CAL_MIN..THROTTLE_CAL_MAX from
// firmware/lib/chassis/cal.hxx, 1541..1600. Read the comment there: they were
// measured on the brushed 1060, since replaced by a brushless that maps
// 1500..2000 almost linearly, so 1541 is probably already creeping and 1600 is
// no longer a crawl. They are used anyway BECAUSE they are narrow. A 59 us
// band cannot launch the car, and a first autonomous drive wants a car that
// cannot launch. Widen them in cal.hxx from a measured test, never here.
//
// The board also holds the car still on its own: a pulse from an unarmed ESC
// is refused with "ERR esc not armed". This program sends ESC ARM only with
// --arm, on the hub's rule that reactive drives nothing until a person has
// armed the car. Without the flag the loop still runs, still steers, and the
// board's refusals are printed once per tick - which is the right amount of
// noise for a car that was not supposed to move.
//
// ---------------------------------------------------------------------------
// --dry, AND WHY IT IS THE FIRST THING TO RUN
//
// Never opens the Pico. Prints what would have been sent, one line per tick,
// with the mode and clearance that produced it. A controller that has never
// been watched deciding should not be handed a car; this is how it is watched,
// with the sensor, the room and the decisions all real and only the thing that
// moves left out.
//
// ---------------------------------------------------------------------------
// THE FEED RIDES ALONG
//
// While it drives, the pilot serves the same wire tools/scanfeed.cxx does -
// src/feed.hxx on scanwire::PORT - so the hub and the board's own dashboard
// can watch the car see. Each revolution goes out as the F line it was and is
// followed by a D line saying what was decided about it; a blind tick sends
// only the D, so a viewer's mode and clearance stay live through the spin-up
// and through a lost lidar rather than freezing on the last good picture.
//
// A viewer may not stop the motor. scanfeed obeys MOTOR 0 because the lidar
// is spinning for the viewer alone; here it is spinning for the car, and a
// viewer that could stop it could stop the car seeing. The request is answered
// with the true state - MOTOR 1 - and logged once per client.
//
// The tick pays for none of it. publish() takes a mutex for a push and wakes
// the feed thread, which does the sends; a viewer that stalls is dropped by
// that thread, and a tick with no viewer connected pays for nothing at all.
// The same F and D text goes to scanwire::SCAN_FILE on tmpfs for the status
// page, a write and a rename. The exit summary prints what the two cost.
//
// The feed is on by default and --no-feed turns it off, the scan file with
// it: the flag means "no viewers", not "no sockets". scanwire::PORT is the
// address, and when it is already taken - scanfeed idling under systemd,
// which is the field case and one nobody on the board has root to stop -
// the feed falls back to scanwire::PILOT_PORT and says so. scanfeed, finding
// the lidar held, relays every viewer there, so the hub keeps dialing 8011
// and sees the car drive. A feed that could not bind either is said once
// and driven without, the file still written: the car does not wait for its
// audience.
//
// ---------------------------------------------------------------------------
// WHAT IS COUNTED
//
// Every revolution, every grab that timed out, every line the board answered
// and whether it was OK or ERR - and, from carlink, every line sent and every
// one dropped. The status line once a second and the summary at exit print
// them, because the failure this project keeps finding in its own code is the
// one that reports success while measuring nothing: a pilot that ran for a
// minute and saw zero revolutions exits 1 and says so.

#include "shared.hxx"

#include "feed.hxx"
#include "lidar.hxx"
#include "link.hxx"
#include "proto.hxx"
#include "reactive.hxx"
#include "scanwire.hxx"
#include "viewfeed.hxx"

// The car's measured numbers - see cal.hxx, and "WHAT THE CAR IS TOLD" above,
// for why the throttle pair is used despite being out of date.
#include "chassis/cal.hxx"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{

  // Half the board's DEADMAN_MS (firmware/app/main.cxx, 400). Not imported from
  // there - that file is firmware and includes the Pico SDK - so the pairing is
  // held by this comment and by the test that will one day time a car.
  constexpr Int32 REV_WAIT_MS = 200;

  constexpr Int32 STATUS_EVERY_MS = 1000;

  // Consecutive grab() timeouts, after the first revolution, before the lidar
  // is called lost: 2 s at REV_WAIT_MS, twenty times the normal interval
  // between revolutions, so a revolution that is merely late does not trip it
  // and a cable that is out does.
  constexpr Int32 LIDAR_LOST_TIMEOUTS = 10;

  // The narrow, stale, deliberately-kept band. See the header comment.
  constexpr Int32 ESC_MIN_US = THROTTLE_CAL_MIN;
  constexpr Int32 ESC_MAX_US = THROTTLE_CAL_MAX;
  static_assert(ESC_MAX_US > ESC_MIN_US, "cal.hxx throttle band is empty or inverted");

  // Where the status page (tools/status/status_server.py) reads the pilot's
  // last second from. One JSON object per write, rewritten whole once a second
  // through a rename so a reader never sees half a line; tmpfs, so it is gone
  // at reboot with the process it described. The page treats a file older than
  // three seconds as "pilot not running" - the file is a heartbeat, not a log.
  constexpr CharSeq STATUS_FILE = "/tmp/bibo-pilot.json";

  // What the board calls this build when it refuses a viewer's version. The
  // sentence is the useful part - "incompatible" alone sends a person to read
  // source in a field, and a stamp is what turns it into an action - so this is
  // the compiler's own date and time rather than a git hash: nothing hands one
  // in at build time, and a hand-typed hash is wrong the first day nobody
  // remembers to change it. Wire -DBOARD_BUILD and this becomes the commit.
#if defined(BOARD_BUILD_STAMP)
  constexpr CharSeq BOARD_BUILD = BOARD_BUILD_STAMP;
#else
  constexpr CharSeq BOARD_BUILD = __DATE__ " " __TIME__;
#endif

  // The whole file at once, through a rename, so a reader never sees half of
  // it. The heartbeat and the scan file are both written this way.
  Void writeWhole(CharSeq path, const Str& text)
  {
      const Str tmp = Str(path) + ".tmp";
      std::FILE* f = std::fopen(tmp.c_str(), "w");
      if(f == nullptr)
      {
          return;    // no /tmp here (a laptop): the page is a Linux thing
      }
      std::fputs(text.c_str(), f);
      std::fclose(f);
      static_cast<Void>(std::rename(tmp.c_str(), path));
  }

  [[nodiscard]] Float64 epochNow()
  {
      // Wall time, not the monotonic Clock everything else here uses: the page
      // compares it with its own time.time() to say how old the heartbeat is.
      return Duration<Float64>(WallClock::now().time_since_epoch()).count();
  }

  // Written from the signal handler, read from the loop. volatile sig_atomic_t
  // is the one type the standard promises is safe to touch in a handler; the
  // Atomic<> alias is not guaranteed lock-free and so is not.
  volatile std::sig_atomic_t interrupted = 0;

  // Int32 is int32_t, which is int on every host this builds for, so the
  // signature still matches std::signal's handler type.
  Void onInterrupt(Int32)
  {
      interrupted = 1;
  }

  struct Options
  {
      Str     lidarPort = "/dev/ttyUSB0";
      Str     picoPort = "/dev/ttyACM0";
      Bool    dry = false;
      Bool    arm = false;

      // MANUAL: a viewer's CONTROL writes steer and throttle instead of the
      // autonomy. A startup flag and not something a held key can cause,
      // because section 6 is explicit that the mode changes only through
      // COMMAND SET_MODE - and until that verb is honoured, slipping into
      // MANUAL because somebody took the control slot would be this program
      // inventing a mode change nobody asked for.
      Bool    manual = false;

      Float32 forwardDeg = 0.0f;
      Float64 seconds = -1.0;   // negative: until a signal
      Bool    feed = true;      // serve the scan feed on scanwire::PORT
  };

  Void usage()
  {
      std::printf(
          "pilot [--lidar PORT] [--pico PORT] [--dry] [--manual] [--arm] [--forward DEG] [--seconds N] [--no-feed]\n"
          "  --lidar PORT   the C1's serial device        (default /dev/ttyUSB0)\n"
          "  --pico PORT    the car's serial device       (default /dev/ttyACM0)\n"
          "  --dry          never open the Pico; print each decision instead\n"
          "  --manual       a viewer's CONTROL drives, not the autonomy\n"
          "  --arm          send ESC ARM once the link is up, so throttle is obeyed\n"
          "  --forward DEG  the raw lidar angle that is straight ahead (default 0)\n"
          "  --seconds N    run for N seconds, then stop  (default: until SIGINT)\n"
          "  --no-feed      no viewers: neither the scan feed on TCP %u (or %u) nor %s\n",
          static_cast<unsigned>(scanwire::PORT),
          static_cast<unsigned>(scanwire::PILOT_PORT),
          scanwire::SCAN_FILE
      );
  }

  // WHO WRITES steer and throttle, in one place.
  //
  // This was `opt.dry ? 1u : 2u` written out at three separate sites - the BOARD
  // frame, DECIDE's source, and CTLSTATE's Applied - and a fourth mode arriving
  // would have had to be remembered at all three. Two of them agreeing and one
  // not is the shape of bug this repo keeps finding, and it would show up as a
  // viewer being refused for a mode disagreement the board reported as agreeing.
  [[nodiscard]] UInt8 pilotModeOf(const Options& o)
  {
      if(o.dry)
      {
          return static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_LOOK);
      }
      if(o.manual)
      {
          return static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL);
      }
      return static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_DRIVE);
  }

  // The flag's value, or a refusal naming the flag. Advances `i` past it.
  [[nodiscard]] Bool takeValue(Int32 argc, Char** argv, Int32& i, Str& out)
  {
      if(i + 1 >= argc)
      {
          std::printf("%s needs a value\n", argv[i]);
          return false;
      }
      ++i;
      out = argv[i];
      return true;
  }

  // strtod, but the whole token must be the number: "--seconds 12x" is a typo
  // to report, not a 12 to run with.
  [[nodiscard]] Bool parseNumber(const Str& text, Float64& out)
  {
      if(text.empty())
      {
          return false;
      }
      Char* end = nullptr;
      out = std::strtod(text.c_str(), &end);
      return end != nullptr && *end == '\0';
  }

  [[nodiscard]] Bool parseOptions(Int32 argc, Char** argv, Options& o)
  {
      for(Int32 i = 1; i < argc; ++i)
      {
          const Str flag = argv[i];
          Str value;
          if(flag == "--dry")
          {
              o.dry = true;
          }
          else if(flag == "--arm")
          {
              o.arm = true;
          }
          else if(flag == "--manual")
          {
              o.manual = true;
          }
          else if(flag == "--no-feed")
          {
              o.feed = false;
          }
          else if(flag == "--lidar")
          {
              if(!takeValue(argc, argv, i, o.lidarPort))
              {
                  return false;
              }
          }
          else if(flag == "--pico")
          {
              if(!takeValue(argc, argv, i, o.picoPort))
              {
                  return false;
              }
          }
          else if(flag == "--forward")
          {
              Float64 deg = 0.0;
              if(!takeValue(argc, argv, i, value) || !parseNumber(value, deg))
              {
                  std::printf("--forward wants degrees, got '%s'\n", value.c_str());
                  return false;
              }
              o.forwardDeg = static_cast<Float32>(deg);
          }
          else if(flag == "--seconds")
          {
              if(!takeValue(argc, argv, i, value) || !parseNumber(value, o.seconds))
              {
                  std::printf("--seconds wants a number, got '%s'\n", value.c_str());
                  return false;
              }
          }
          else
          {
              std::printf("unknown argument '%s'\n\n", flag.c_str());
              return false;
          }
      }

      // INCOHERENT, not merely redundant. --dry never opens the Pico at all;
      // MANUAL means a viewer's CONTROL is what writes to it. Accepting both
      // would start a run that reports pilotMode MANUAL on the wire, invites a
      // viewer to take the control slot and hold a key, and then sends the car
      // nothing whatsoever - with every layer truthfully reporting success.
      if(o.dry && o.manual)
      {
          std::printf("--dry and --manual contradict: a dry run never opens the Pico for CONTROL to drive\n");
          return false;
      }
      return true;
  }

  // ---- the decision, as two lines ------------------------------------------

  // The module's 0..1 onto idle..full of the calibrated band, rounded to the
  // microsecond. The clamp is against a bug, not a tuning: nothing in
  // reactive::Config exceeds 1.0, and a value that did should hit the top of
  // the band rather than be multiplied past it.
  [[nodiscard]] Int32 escPulseFor(Float32 throttle)
  {
      if(throttle > 1.0f)
      {
          throttle = 1.0f;
      }
      constexpr Float32 span = static_cast<Float32>(ESC_MAX_US - ESC_MIN_US);
      return ESC_MIN_US + static_cast<Int32>(throttle * span + 0.5f);
  }

  // The throttle line: a pulse for a forward decision from a scan the module
  // trusted, NEUTRAL for everything else - blind, stop, reverse. The steering
  // line needs no such function; it is proto::steer(out.steer) every tick.
  //
  // `silent`: the car has not spoken within carlink::Config::silenceMs. The
  // transport reports it and does not act on it - see silentForMs - so the
  // policy sits here, and the policy is that a board that has stopped talking
  // is not given throttle. Steering is still sent: it costs nothing and is the
  // command that will show up as the first reply when the board comes back.
  [[nodiscard]] Str escLineFor(reactive::Status status, const reactive::Outputs& out, Bool silent, Bool mayDrive)
  {
      const Bool trusted = status == reactive::Status::STATUS_OK;
      const Bool forward = trusted && !out.stop && out.throttle > 0.0f;

      // `mayDrive` is --arm: whether THIS RUN is allowed to move the car. It is
      // intent, not measurement, and that is the right input here - the question
      // is what we are willing to send, not what the car currently is.
      //
      // Without it a run started with no --arm sent ESC <us> on every forward
      // decision and the Pico refused every one: measured at ten "ERR esc not
      // armed" a second, nineteen inside four seconds. Harmless to the car and
      // corrosive to everything that reads the link - it made replyErr useless
      // as a health signal, printed a fault line for correct behaviour, and
      // spent the port's bandwidth being told no. NEUTRAL is the honest line:
      // it is what we would command anyway, and it is accepted while disarmed.
      if(!forward || silent || !mayDrive)
      {
          return proto::command("ESC", "NEUTRAL");
      }
      return proto::escUs(escPulseFor(out.throttle));
  }

  // ---- the car end -----------------------------------------------------------

  struct Replies
  {
      UInt64 ok = 0;
      UInt64 err = 0;
      UInt64 other = 0;   // INFO, banner text, anything the board says unasked

      // ---- what the CAR said about itself ------------------------------------
      //
      // STEER goes out every tick and the Pico answers every one of them with
      // printDrive() - the whole drive state, armed= included. This program
      // counted that line and threw its contents away, which is the only reason
      // BoardState::picoArmed was hard-coded to "unknown" and the viewer's Car
      // panel could never say anything but "armed --".
      //
      // Reading it costs NOTHING. Not one extra byte goes down the port and no
      // poll is added: the line was already arriving fifty times a second and
      // was already being parsed far enough to be classified.
      //
      // -1 is "the car has not told us", which is a different fact from any
      // value it could report - and is what these stay at on a dry run, where
      // nothing is asked and nothing answers.
      Int32 armed = -1;
      Int32 escUs = -1;
      Int32 steerNowMilli = 0;
  };

  // What this program knows about the link that the transport does not.
  //
  // `heard` is the one that matters for safety. carlink::open() returns OK
  // after termios setup with no exchange, and silentForMs() counts from that
  // moment - so for the first silenceMs after every open and every reopen the
  // transport reports a board that is not yet silent, about a board that has
  // never spoken. Throttle waits for the first line instead.
  struct Link
  {
      Bool lost = false;        // declared gone; retried once a second
      Bool heard = false;       // a line has arrived since the port last opened
      Bool stallSaid = false;   // a stalled write has been mentioned once
  };

  // Everything the board said since the last call: OK and ERR counted, ERR
  // lines printed in full because the reason is the useful part. Returns
  // false when the link has gone, having still handed over the board's last
  // words.
  [[nodiscard]] Bool readReplies(Vec<Str>& scratch, Replies& tally, Link& link)
  {
      scratch.clear();
      const carlink::Result r = carlink::drain(scratch);
      if(!scratch.empty())
      {
          // Any line at all, PONG and banner text included: the question is
          // whether something is on the other end, not whether it agreed.
          link.heard = true;
      }
      for(const Str& line : scratch)
      {
          const proto::Reply reply = proto::read(line);
          switch(reply.kind)
          {
          case proto::Kind::KIND_OK:
              ++tally.ok;
              // "OK drive ..." is the answer to the STEER this tick already
              // sent, so the car's own arm state and pulse arrive for free.
              // Read BY NAME through proto::field, which matches on token
              // boundaries - so "esc" does not find "esc_min" and a firmware
              // that adds a field later is ignored rather than shifting
              // everything after it.
              if(reply.topic == "drive")
              {
                  // THE '=' IS PART OF THE KEY. proto::field compares the key
                  // and then reads the value from directly after it, so "armed"
                  // hands strtol the string "=0", which is not a number, and the
                  // call returns false EVERY TIME. Written that way first: the
                  // build stayed green, the suite stayed green, the live run was
                  // clean, and armed would have sat at -1 for the life of the
                  // process with the guard above it reading as protection.
                  Int32 v = 0;
                  if(proto::fieldInt(reply.rest, "armed=", v))
                  {
                      tally.armed = v;
                  }
                  if(proto::fieldInt(reply.rest, "esc=", v))
                  {
                      tally.escUs = v;
                  }
                  if(proto::fieldInt(reply.rest, "steer_now=", v))
                  {
                      tally.steerNowMilli = v;
                  }
              }
              break;
          case proto::Kind::KIND_ERR:
              ++tally.err;
              std::printf("pico: %s\n", reply.line.c_str());
              break;
          case proto::Kind::KIND_INFO:
          case proto::Kind::KIND_OTHER:
              ++tally.other;
              break;
          case proto::Kind::KIND_EMPTY:
              break;
          }
      }
      return r != carlink::Result::RESULT_CLOSED && r != carlink::Result::RESULT_NOT_OPEN;
  }

  // Sends, and says so when the line went nowhere. Once - the counters carry
  // the running total, and a message per dropped tick on a dead link would
  // bury the one line that explains why it died.
  //
  // Only CLOSED and NOT_OPEN are a lost link. WRITE_FAILED is a stall - the
  // device took nothing for WRITE_WAIT_MS - and the descriptor is still good,
  // so declaring it lost would have the once-a-second retry call open() on a
  // link that never dropped, get RESULT_OK back, print "link back" and re-send
  // ESC ARM, which resets the board's throttle target to neutral: a dip
  // mid-drive for a 100 ms hiccup. A stall is counted by dropped() and said
  // once here.
  Void sendLine(const Str& line, Link& link)
  {
      const carlink::Result r = carlink::send(line);
      if(r == carlink::Result::RESULT_OK)
      {
          return;
      }
      const Bool gone = r == carlink::Result::RESULT_CLOSED
                     || r == carlink::Result::RESULT_NOT_OPEN;
      if(gone && !link.lost)
      {
          link.lost = true;
          std::printf("pico link lost: %s - %s\n", carlink::why(r), carlink::detail().c_str());
      }
      else if(!gone && !link.stallSaid)
      {
          link.stallSaid = true;
          std::printf("pico write dropped: %s - %s\n", carlink::why(r), carlink::detail().c_str());
      }
  }

  [[nodiscard]] Bool openPico(const carlink::Config& cfg, Bool arm, Link& link)
  {
      // Whatever the last board said no longer counts, whether or not this
      // attempt succeeds: a failed reopen leaves no descriptor, and a board
      // behind a fresh descriptor has not been heard from yet.
      link.heard = false;
      const carlink::Result r = carlink::open(cfg);
      if(r != carlink::Result::RESULT_OK)
      {
          std::printf(
              "pico %s: %s - %s\n",
              cfg.where.c_str(),
              carlink::why(r),
              carlink::detail().c_str()
          );
          return false;
      }
      link.lost = false;
      // Something for the board to answer before the loop has said anything,
      // so a live board is heard by the first tick's readReplies rather than
      // a tick later. Every STEER and ESC line gets an OK as well; this one
      // simply goes out first, and has an answer even from an unarmed board.
      sendLine(proto::command("PING"), link);
      if(arm)
      {
          // Re-sent on every (re)open, not just the first: a board that was
          // replugged or rebooted came up disarmed, and the flag means "this
          // run is allowed to move the car", not "arm once".
          sendLine(proto::command("ESC", "ARM"), link);
      }
      return true;
  }

  // A one-line description of the decision, for the dry run and the log.
  [[nodiscard]] Str describe(reactive::Status status, const reactive::Outputs& out, Bool got)
  {
      Array<Char, 96> buf{};
      if(!got)
      {
          std::snprintf(
              buf.data(),
              buf.size(),
              "%-8s no revolution within %d ms",
              "blind",
              REV_WAIT_MS
          );
      }
      else if(status != reactive::Status::STATUS_OK)
      {
          std::snprintf(
              buf.data(),
              buf.size(),
              "%-8s %s",
              reactive::modeName(out.mode),
              reactive::why(status)
          );
      }
      else
      {
          std::snprintf(
              buf.data(),
              buf.size(),
              "%-8s clear %5.0f mm  hits %3d",
              reactive::modeName(out.mode),
              static_cast<Float64>(out.clearanceMm),
              out.corridorHits
          );
      }
      return Str(buf.data());
  }

  // ---- the viewers -------------------------------------------------------------

  // What a new client is told: INFO, HEALTH when the device answered, and the
  // motor state - which, while the pilot runs, is on.
  [[nodiscard]] Str greetingFor(const lidar::Device& d)
  {
      scanwire::Info info;
      info.model = d.model;
      info.fwMajor = d.fwMajor;
      info.fwMinor = d.fwMinor;
      info.hwRev = d.hwRev;
      info.serial = d.serial;
      Str hello = scanwire::formatInfo(info);
      // HEALTH carries only the three values the wire defines; -1 (the device
      // did not answer) is left out rather than sent as a line every reader
      // would have to reject.
      if(d.health >= 0 && d.health <= 2)
      {
          hello += scanwire::formatHealth(d.health);
      }
      hello += scanwire::formatMotor(true);
      return hello;
  }

  // The revolution as the feed sends it. `hz` is what this tick measured, so
  // the viewer sees the rate the car is deciding at.
  [[nodiscard]] Str frameLine(const Vec<reactive::Ray>& rays, const Vec<UInt8>& quality, Int32 dtMs)
  {
      scanwire::Frame f;
      f.hz = dtMs > 0 ? 1000.0f / static_cast<Float32>(dtMs) : 0.0f;
      f.samples.reserve(rays.size());
      for(Size i = 0; i < rays.size(); ++i)
      {
          scanwire::Sample s;
          s.angleDeg = rays[i].angleDeg;
          s.distMm = rays[i].distMm;
          s.quality = i < quality.size() ? quality[i] : static_cast<UInt8>(0);
          f.samples.push_back(s);
      }
      return scanwire::formatFrame(f);
  }

  // The decision as the feed sends it. "blind" for a tick with no revolution,
  // the same word describe() prints, whatever mode the module was left in.
  [[nodiscard]] Str driveLine(const reactive::Outputs& out, Bool got)
  {
      scanwire::Drive d;
      d.mode = got ? reactive::modeName(out.mode) : "blind";
      d.clearanceMm = static_cast<Int32>(out.clearanceMm + 0.5f);
      d.hits = out.corridorHits;
      d.steer = out.steer;
      d.throttle = out.throttle;
      d.stop = out.stop;
      return scanwire::formatDrive(d);
  }

  // What serving the viewers cost the tick, so "adds nothing" is a number in
  // the exit summary rather than a belief.
  struct Viewer
  {
      Bool    serving = false;   // feed::start succeeded
      UInt64  frames = 0;        // F lines published
      Float64 costMaxUs = 0.0;   // the longest publish + file write of any tick
      Float64 costSumUs = 0.0;
      UInt64  costTicks = 0;
      Bool    wire = false;      // viewfeed::start succeeded
  };

  // ---- the board's state, filled ONCE a tick and read TWICE --------------------

  // docs/bibowire.md section 5 puts one obligation on this file by name: the
  // dashboard's JSON and the BOARD frame must be filled from the SAME STRUCT in
  // the SAME TICK, so the phone and the viewer can never disagree about what the
  // car thinks. This is that struct. Everything the page prints and everything
  // the viewer is told about the board is derived from one instance of it,
  // filled once per tick below; a second, independently-filled source is exactly
  // what the rule forbids, and it is the shape a disagreement would take.
  struct Snapshot
  {
      Float64 ts = 0.0;            // wall clock, for the page's staleness test
      UInt64  monoUs = 0;          // the same instant on the monotonic clock
      UInt32  upS = 0;
      Str     mode;                // the reactive mode word, or "blind"
      Float32 clearanceMm = 0.0f;
      Int32   hits = 0;
      Float64 revPerS = 0.0;
      UInt64  timeouts = 0;
      UInt64  revolutions = 0;
      Bool    lidarLost = false;
      Int32   lidarHealth = -1;
      Bool    lidarSpinning = false;
      Bool    dry = false;
      UInt8   pilotMode = 2;       // who writes steer and throttle; see pilotModeOf
      Bool    picoOpen = false;
      Bool    picoHeard = false;
      Int32   picoSilentMs = -1;   // carlink's own number; -1 is no link
      Int32   picoArmed = -1;      // the car's own armed=; -1 is "it has not said"
      UInt64  replyOk = 0;
      UInt64  replyErr = 0;
      Str     pico;                // the printed phrase, shared by all three readers
  };

  // -1 (the device did not answer) becomes 255, the wire's "unknown". A health
  // nobody read must never arrive as 0, which means "good".
  [[nodiscard]] UInt8 healthByte(Int32 health)
  {
      return health >= 0 && health <= 2 ? static_cast<UInt8>(health) : bibowire::HEALTH_ABSENT;
  }

  // The phrase the console prints, the page shows and the viewer is told - one
  // string, so all three say the same thing about the car's link.
  [[nodiscard]] Str picoPhrase(const Snapshot& s)
  {
      Array<Char, 48> pico{};
      if(s.dry)
      {
          std::snprintf(pico.data(), pico.size(), "pico dry");
          return Str(pico.data());
      }
      // -1 is no descriptor, and "silent -1 ms" would be a number standing in
      // for a fact.
      Array<Char, 24> silence{};
      if(s.picoSilentMs < 0)
      {
          std::snprintf(silence.data(), silence.size(), "%-15s", "no link");
      }
      else
      {
          std::snprintf(silence.data(), silence.size(), "silent %5d ms", s.picoSilentMs);
      }
      std::snprintf(
          pico.data(),
          pico.size(),
          "pico %s  ok %llu  err %llu",
          silence.data(),
          static_cast<unsigned long long>(s.replyOk),
          static_cast<unsigned long long>(s.replyErr)
      );
      return Str(pico.data());
  }

  // The heartbeat the status page reads. Byte-for-byte the object it has always
  // been - the page is not changing - but now derived from the snapshot rather
  // than from the locals, which is the whole point.
  [[nodiscard]] Str jsonFrom(const Snapshot& s)
  {
      Array<Char, 320> json{};
      std::snprintf(
          json.data(),
          json.size(),
          "{\"ts\":%.3f,\"mode\":\"%s\",\"clearanceMm\":%.0f,\"hits\":%d,"
          "\"revPerS\":%.1f,\"timeouts\":%llu,\"revolutions\":%llu,"
          "\"lidarLost\":%s,\"pico\":\"%s\"}\n",
          s.ts,
          s.mode.c_str(),
          static_cast<Float64>(s.clearanceMm),
          s.hits,
          s.revPerS,
          static_cast<unsigned long long>(s.timeouts),
          static_cast<unsigned long long>(s.revolutions),
          s.lidarLost ? "true" : "false",
          s.pico.c_str()
      );
      return Str(json.data());
  }

  // The same snapshot, as the viewer is told it. The fields this program cannot
  // measure keep their ABSENT sentinels rather than being faked by a zero: 0 mV
  // is a real reading of a dead pack, and 0 centi-degrees is a real temperature.
  // viewfeed fills the few fields that are its own measurement - the deadman,
  // the epoch, the holder and its encode cost - because this file cannot know
  // them and the page does not show them.
  [[nodiscard]] bibowire::BoardState boardFrom(const Snapshot& s)
  {
      bibowire::BoardState b;
      b.tMonoUs = s.monoUs;
      b.upS = s.upS;
      b.cpuCentiC = bibowire::CPU_ABSENT;
      b.battMilliV = bibowire::BATT_ABSENT;
      b.picoLink = s.dry || !s.picoOpen ? 0u : (s.picoHeard ? 1u : 2u);
      // The CAR's answer, not this program's intention. Every STEER is answered
      // with armed=, so 2 ("unknown") now means only that no reply has been read
      // yet - one tick at startup, and the whole of a dry run, where there is no
      // port to ask down.
      b.picoArmed = s.picoArmed < 0 ? 2u : (s.picoArmed != 0 ? 1u : 0u);
      b.pilotMode = s.pilotMode;   // decided once per tick by pilotModeOf, not here
      b.lidarHealth = healthByte(s.lidarHealth);
      b.lidarSpinning = s.lidarSpinning ? 1u : 0u;
      b.picoSilentMs = s.picoSilentMs < 0
          ? bibowire::PICO_SILENT_ABSENT
          : static_cast<UInt32>(s.picoSilentMs);
      b.revolutions = static_cast<UInt32>(s.revolutions);
      b.timeouts = static_cast<UInt32>(s.timeouts);
      return b;
  }

  // ---- the operator's trim, as the Pico's parser reads it ----------------------

  // At most this many tuning lines per tick. A slider dragged across its range
  // is a burst of discrete COMMANDs, and this loop's promise is that it finishes
  // inside 20 ms - so the queue is drained at a rate the serial port can carry
  // rather than emptied in one pass. Nothing is lost by the cap: what is not
  // taken this tick is taken by the next, 20 ms later.
  constexpr Int32 TUNE_PER_TICK = 2;

  // One accepted tuning request as the line firmware/app/main.cxx's COMMANDS
  // table parses. Empty for a verb this build does not send, which the caller
  // drops rather than handing the port a bare verb with no argument.
  //
  // %u AND NOTHING ELSE. Every number on this path is a whole microsecond, so
  // the locale-sensitive decimal point proto::fixed3 exists to dodge never gets
  // near it - not because it is escaped, but because there is no float here to
  // print in the first place.
  [[nodiscard]] Str tuneLine(const viewfeed::Tune& t)
  {
      Array<Char, 48> args{};
      const unsigned a1 = static_cast<unsigned>(t.arg1);
      const unsigned a2 = static_cast<unsigned>(t.arg2);
      switch(t.verb)
      {
          case bibowire::Verb::VERB_SET_SERVO_LIMITS:
              std::snprintf(args.data(), args.size(), "%u %u", a1, a2);
              return proto::command("SERVOLIMITS", args.data());
          case bibowire::Verb::VERB_SET_ESC_LIMITS:
              std::snprintf(args.data(), args.size(), "%u %u", a1, a2);
              return proto::command("ESCLIMITS", args.data());
          case bibowire::Verb::VERB_SET_SERVO_TRIM:
              std::snprintf(args.data(), args.size(), "%u", a1);
              return proto::command("SERVOTRIM", args.data());
          case bibowire::Verb::VERB_SET_SLEW:
              // The bare `SLEW <us>` is the axis-less form the board has always
              // taken and is what one shared rate used to mean, so "both" is
              // not spelled out - it is the absence of an axis word.
              if(t.arg0 == bibowire::SLEW_AXIS_STEER)
              {
                  std::snprintf(args.data(), args.size(), "STEER %u", a1);
              }
              else if(t.arg0 == bibowire::SLEW_AXIS_THROTTLE)
              {
                  std::snprintf(args.data(), args.size(), "THROTTLE %u", a1);
              }
              else
              {
                  std::snprintf(args.data(), args.size(), "%u", a1);
              }
              return proto::command("SLEW", args.data());
          default:
              return Str();
      }
  }

  // ---- the revolution and the decision, as bibowire carries them ---------------

  // Degrees and millimetres to whole centi-degrees and whole millimetres, which
  // is LOSSLESS with respect to the device - the C1 does not measure finer. 360
  // degrees wraps to 0 and is never 36000, which is scanwire's rule and what the
  // encoder refuses. This is the same loop frameLine already runs, minus the
  // snprintf; the framing and the CRC happen on viewfeed's thread, not here.
  [[nodiscard]] bibowire::Scan scanFrom(const Vec<reactive::Ray>& r, const Vec<UInt8>& q)
  {
      bibowire::Scan s;
      const Size count = r.size() > bibowire::MAX_SCAN_POINTS ? bibowire::MAX_SCAN_POINTS : r.size();
      s.points.reserve(count);
      s.quality.reserve(count);
      for(Size i = 0; i < count; ++i)
      {
          Float32 deg = r[i].angleDeg;
          while(deg < 0.0f)
          {
              deg += 360.0f;
          }
          while(deg >= 360.0f)
          {
              deg -= 360.0f;
          }
          bibowire::ScanPoint p;
          p.angleCentiDeg = static_cast<UInt16>(deg * 100.0f + 0.5f);
          if(p.angleCentiDeg >= 36000u)
          {
              p.angleCentiDeg = 0;
          }
          const Float32 mm = r[i].distMm;
          p.distMm = mm <= 0.0f ? 0u : static_cast<UInt16>(mm > 65535.0f ? 65535.0f : mm + 0.5f);
          s.points.push_back(p);
          const UInt8 qual = i < q.size() ? q[i] : static_cast<UInt8>(0);
          s.quality.push_back(qual > 63u ? static_cast<UInt8>(63) : qual);
      }
      return s;
  }

  // Thousandths of -1..1, the units the wire uses everywhere and the ones that
  // have no locale and no NaN in them.
  [[nodiscard]] Int16 milliOf(Float32 fraction)
  {
      const Float32 clamped = fraction > 1.0f ? 1.0f : (fraction < -1.0f ? -1.0f : fraction);
      return static_cast<Int16>(clamped * 1000.0f + (clamped >= 0.0f ? 0.5f : -0.5f));
  }

  [[nodiscard]] bibowire::Decide decideFrom(const reactive::Outputs& o, Int32 modeMs, UInt32 rev)
  {
      bibowire::Decide d;
      // 0 is a BLIND tick with no revolution behind it, which is what the
      // caller passes when grab() timed out.
      d.revIndex = rev;
      d.clearanceMm = static_cast<UInt32>(o.clearanceMm < 0.0f ? 0.0f : o.clearanceMm + 0.5f);
      d.hits = static_cast<UInt16>(o.corridorHits < 0 ? 0 : o.corridorHits);
      d.steerMilli = milliOf(o.steer);
      d.throttleMilli = milliOf(o.throttle);
      d.mode = static_cast<UInt8>(o.mode);
      d.stop = o.stop ? 1u : 0u;
      d.modeMs = static_cast<UInt32>(modeMs < 0 ? 0 : modeMs);
      return d;
  }

  // The device's identity, as LIDAR_INFO carries it. The serial is 32 hex digits
  // as the device printed them; the wire carries the 16 raw bytes and the viewer
  // renders them back, so a serial that is not 32 hex digits arrives as zeros
  // rather than as somebody else's device.
  [[nodiscard]] bibowire::LidarInfo lidarInfoFrom(const lidar::Device& d)
  {
      bibowire::LidarInfo i;
      i.model = static_cast<UInt16>(d.model);
      i.fwMajor = static_cast<UInt8>(d.fwMajor);
      i.fwMinor = static_cast<UInt8>(d.fwMinor);
      i.hwRev = static_cast<UInt16>(d.hwRev);
      i.baud = 460800;
      if(d.serial.size() == 32u)
      {
          for(Size b = 0; b < 16u; ++b)
          {
              UInt32 byte = 0;
              for(Size n = 0; n < 2u; ++n)
              {
                  const Char ch = d.serial[b * 2u + n];
                  UInt32 nibble = 0;
                  if(ch >= '0' && ch <= '9')
                  {
                      nibble = static_cast<UInt32>(ch - '0');
                  }
                  else if(ch >= 'A' && ch <= 'F')
                  {
                      nibble = static_cast<UInt32>(ch - 'A') + 10u;
                  }
                  else if(ch >= 'a' && ch <= 'f')
                  {
                      nibble = static_cast<UInt32>(ch - 'a') + 10u;
                  }
                  byte = (byte << 4u) | nibble;
              }
              i.serial[b] = static_cast<UInt8>(byte);
          }
      }
      return i;
  }

}

Int32 main(Int32 argc, Char** argv)
{
    Options opt;
    if(!parseOptions(argc, argv, opt))
    {
        usage();
        return 2;
    }

    // Line-buffered even into a pipe: a status line a second is only a status
    // line if it arrives once a second, and over ssh or into a log stdout is
    // otherwise held back until exit - which for a run that never exits is
    // never.
    std::setvbuf(stdout, nullptr, _IOLBF, BUFSIZ);

    // Installed before anything is opened, not before the motor starts. The
    // handlers only store a flag, so there is nothing they could run too
    // early; what they must not be is late. lidar::open() blocks for two
    // seconds or more and openPico() sends ESC ARM, and a Ctrl-C between
    // those with the default handler would exit without the shutdown STOP,
    // leaving the board armed. From here on every exit is through the
    // shutdown below, or through a check of `interrupted` before arming.
    std::signal(SIGINT, onInterrupt);
    std::signal(SIGTERM, onInterrupt);

    // ---- the module's tuning ---------------------------------------------------
    reactive::Config tune = reactive::tuning();
    tune.forwardDeg = opt.forwardDeg;
    if(!reactive::configure(tune))
    {
        std::printf(
            "reactive tuning refused (forward %.1f deg)\n",
            static_cast<Float64>(opt.forwardDeg)
        );
        return 1;
    }

    // ---- the lidar, parked ------------------------------------------------------
    // Opened first because it is the thing most likely to be missing, and a
    // program that arms a car and then finds it has no eyes has done the two
    // steps in the wrong order.
    std::printf("lidar %s: opening\n", opt.lidarPort.c_str());
    if(!lidar::open(opt.lidarPort))
    {
        std::printf("lidar %s: %s\n", opt.lidarPort.c_str(), lidar::reason().c_str());
        return 1;
    }
    std::printf("lidar device  %s\n", lidar::info().c_str());
    std::printf("lidar health  %s\n", lidar::health().c_str());

    // A Ctrl-C that landed during the lidar's open. Nothing is armed and
    // nothing is spinning, so there is nothing to stop - only a port to give
    // back - and arming the car now would be doing the thing the person just
    // asked not to happen.
    if(interrupted != 0)
    {
        std::printf("interrupted before the car was opened\n");
        lidar::close();
        return 0;
    }

    // ---- the car -----------------------------------------------------------------
    carlink::Config linkCfg;
    linkCfg.where = opt.picoPort;
    Link link;
    if(opt.dry)
    {
        std::printf("pico: dry run - decisions are printed, nothing is sent\n");
    }
    else if(!openPico(linkCfg, opt.arm, link))
    {
        lidar::close();
        return 1;
    }
    else
    {
        std::printf(
            "pico %s: open%s\n",
            opt.picoPort.c_str(),
            opt.arm ? ", ESC ARM sent" : ", not armed"
        );
    }

    // A Ctrl-C that landed while the car was being opened is honored by NOT
    // starting the motor and falling through: the loop sees `interrupted` and
    // does not run, and the shutdown sends the STOP an armed board is owed.
    if(interrupted == 0)
    {
        if(!lidar::motorOn())
        {
            std::printf("lidar motor: %s\n", lidar::reason().c_str());
            lidar::close();
            carlink::close();
            return 1;
        }
        std::printf(
            "running: throttle band %d..%d us, revolution wait %d ms, forward %.1f deg%s\n",
            ESC_MIN_US,
            ESC_MAX_US,
            REV_WAIT_MS,
            static_cast<Float64>(opt.forwardDeg),
            opt.seconds < 0.0 ? "" : ", timed"
        );
    }

    // ---- the viewers -------------------------------------------------------------------
    // After the motor, so the greeting's MOTOR 1 is true when it is sent, and
    // never fatal: a port already taken is scanfeed idling under systemd, the
    // feed moves next door and scanfeed relays to it, and the car drives with
    // or without an audience either way.
    Viewer viewer;
    if(opt.feed && interrupted == 0)
    {
        feed::Policy policy;
        policy.greeting = greetingFor(lidar::device());
        policy.motor = feed::Motor::MOTOR_REFUSE;
        policy.motorOn = true;
        policy.refusal = "viewer asked for the motor; the pilot keeps it while driving";
        policy.fallbackPort = scanwire::PILOT_PORT;
        viewer.serving = feed::start(scanwire::PORT, policy);
        if(!viewer.serving)
        {
            std::printf("feed: not serving - driving without viewers\n");
        }
        else if(feed::port() == scanwire::PORT)
        {
            std::printf("feed: serving on port %u\n", static_cast<unsigned>(feed::port()));
        }
        else
        {
            std::printf(
                "feed: port %u is taken (scanfeed, most likely) - serving on %u, which scanfeed relays to\n",
                static_cast<unsigned>(scanwire::PORT),
                static_cast<unsigned>(feed::port())
            );
        }

        // And bibowire, for the Windows viewer, on 8020. A separate socket and
        // a separate module on purpose: feed.cxx moves LINES for the phone
        // dashboard and must keep doing exactly that, because `nc bibobox.local
        // 8011` from a phone is the field-debugging story this binary format
        // spends and has to pay back.
        //
        // Never fatal. A board that could not bind 8020 still drives, still
        // steers and still serves the dashboard; the viewer is an audience, and
        // the car does not wait for its audience.
        viewfeed::Policy wire;
        wire.boardName = "bibobox";
        wire.boardBuild = BOARD_BUILD;
        // Per PROCESS, not per session: a viewer that reconnects and sees a
        // different one throws away everything it knew before drawing a point.
        wire.bootId = static_cast<UInt32>(WallClock::now().time_since_epoch().count());
        wire.capabilities = static_cast<UInt8>((opt.dry ? 0u : 1u) | 2u | (opt.dry ? 0u : 4u));
        viewer.wire = viewfeed::start(bibowire::PORT, wire);
        if(!viewer.wire)
        {
            std::printf("viewfeed: not serving - driving without a viewer\n");
        }
        else
        {
            // State before scan, always: the device's identity is published
            // before the first revolution can be, so a viewer that connects
            // during the motor's two-second spin-up already knows what it is
            // watching.
            viewfeed::publishLidarInfo(lidarInfoFrom(lidar::device()));
        }
    }

    // ---- the loop ------------------------------------------------------------------
    reactive::State   state;
    reactive::Outputs out;
    reactive::Status  status = reactive::Status::STATUS_BLIND;
    Vec<reactive::Ray> rays;
    Vec<UInt8>         quality;
    Vec<Str>           lines;
    Replies            replies;

    // The last steering the operator actually commanded, in milli, kept ACROSS
    // ticks because section 6's SOFT state holds it rather than centring it: a
    // car that snaps straight mid-corner changes its line at the instant it
    // stopped being commanded, which is the worst moment to change it. Only
    // updated while the deadman is LIVE, so a stale link cannot keep writing it.
    Int16 heldSteerMilli = 0;
    Str                lastFrame;   // the latest F line, for the scan file

    UInt64 revolutions = 0;
    UInt64 timeouts = 0;
    UInt64 windowRevs = 0;   // revolutions since the last status line

    // Silent until proven otherwise. Starting at false would print "pico
    // silent" on the first tick of every run, about a board that has had one
    // tick to answer; starting at true means the first line printed is the
    // board being heard, which is the event worth a line.
    Bool boardSilent = true;

    // A lidar that dies mid-run. grab() times out forever on an unplugged C1
    // and the loop would otherwise run to --seconds sending NEUTRAL and exit 0
    // on the strength of whatever revolutions arrived before the cable came
    // out. Consecutive timeouts are counted only once a revolution has been
    // seen: spin-up after motorOn() produces around eleven of them (2.2 s) on
    // every run, and that is the sensor starting, not the sensor gone.
    Int32 blindRun = 0;          // consecutive timeouts since the last revolution
    Bool  lidarLost = false;     // a blind run reached LIDAR_LOST_TIMEOUTS; sticky

    const TimePoint start = monoNow();
    TimePoint lastTick = start;
    TimePoint lastStatus = start;
    Bool haveTick = false;

    while(interrupted == 0 && (opt.seconds < 0.0 || elapsedS(start) < opt.seconds))
    {
        // Empty on a timeout, and handed to step() anyway: an empty scan is the
        // module's STATUS_BLIND, which is a stop, which is what a tick with no
        // revolution behind it should send. lidar.hxx explains the emptying.
        const Bool got = lidar::grab(rays, REV_WAIT_MS, &quality);
        const TimePoint now = monoNow();
        const Duration<Float64, std::milli> sinceTick = now - lastTick;
        const Int32 dtMs = haveTick ? static_cast<Int32>(sinceTick.count()) : 0;
        lastTick = now;
        haveTick = true;
        if(got)
        {
            ++revolutions;
            ++windowRevs;
            blindRun = 0;
        }
        else
        {
            ++timeouts;
            // Said once per loss, not once per tick: the counter resets on
            // the next revolution, so a lidar that comes back and goes again
            // is reported again. The flag never resets - a run that lost its
            // eyes for two seconds is not a run that succeeded.
            if(revolutions > 0 && ++blindRun == LIDAR_LOST_TIMEOUTS)
            {
                lidarLost = true;
                std::printf(
                    "lidar: no revolution for %d ms - car held at neutral, run marked failed\n",
                    LIDAR_LOST_TIMEOUTS * REV_WAIT_MS
                );
            }
        }

        status = reactive::step(rays.data(), rays.size(), dtMs, &state, &out);

        // The viewers, before the car is told: the car's lines go to a serial
        // port that may stall for WRITE_WAIT_MS, and the feed's go to a queue
        // that cannot. Timed, so the summary can say what they cost. The scan
        // file carries the last revolution under this tick's decision, so the
        // dashboard's mode goes blind when the feed's does.
        if(opt.feed)
        {
            const TimePoint before = monoNow();
            const Str drive = driveLine(out, got);
            if(got)
            {
                lastFrame = frameLine(rays, quality, dtMs);
                ++viewer.frames;
                feed::publish(lastFrame);
            }
            feed::publish(drive);
            writeWhole(scanwire::SCAN_FILE, lastFrame + drive);

            // The same revolution and the same decision to the viewer, as
            // frames rather than lines. publish() is a push and a wake: the
            // encode, the CRC and the send all happen on viewfeed's thread, so
            // this tick pays a queue push whether the viewer is fast, slow or
            // absent - and with nobody connected it pays nothing at all.
            if(got)
            {
                bibowire::Scan scan = scanFrom(rays, quality);
                scan.tMonoUs = static_cast<UInt64>(elapsedS(start) * 1000000.0);
                scan.revIndex = static_cast<UInt32>(revolutions);
                scan.freqMilliHz = dtMs > 0 ? static_cast<UInt16>(1000000 / dtMs) : 0;
                scan.health = healthByte(lidar::device().health);
                scan.motor = 1;
                viewfeed::publishScan(std::move(scan));
            }
            const UInt32 rev = got ? static_cast<UInt32>(revolutions) : 0u;
            bibowire::Decide decide = decideFrom(out, state.modeMs, rev);
            decide.source = pilotModeOf(opt);
            viewfeed::publishDecide(decide);
            const Float64 costUs = elapsedMs(before) * 1000.0;
            viewer.costSumUs += costUs;
            ++viewer.costTicks;
            if(costUs > viewer.costMaxUs)
            {
                viewer.costMaxUs = costUs;
            }
        }

        // The board's silence, judged before deciding, so this tick's throttle
        // already reflects it. Announced on each change rather than each tick.
        //
        // Three silences, one policy. -1 is no descriptor - a reopen that
        // failed - and must read as silent, or a board that is gone would be
        // announced as speaking again. !heard is a port that opened under a
        // board that has not yet answered; see Link. The third is the one the
        // transport measures.
        if(!opt.dry)
        {
            const Int32 silent = carlink::silentForMs();
            const Bool nowSilent = !link.heard || silent < 0 || silent > linkCfg.silenceMs;
            if(nowSilent && !boardSilent)
            {
                if(link.heard && silent >= 0)
                {
                    std::printf(
                        "pico silent for %d ms - throttle held at neutral until it speaks\n",
                        silent
                    );
                }
                else
                {
                    std::printf(
                        "pico not heard since the port opened - throttle held at neutral\n"
                    );
                }
            }
            else if(!nowSilent && boardSilent)
            {
                std::printf("pico speaking - throttle follows the decision again\n");
            }
            boardSilent = nowSilent;
        }

        // ---- the board's state, filled ONCE ------------------------------------------
        // Read twice below: by the heartbeat the phone reads and by the BOARD
        // frame the viewer reads. See Snapshot - this is the obligation
        // docs/bibowire.md section 5 puts on this file, honoured by there being
        // one struct rather than two places that fill the same numbers.
        Snapshot snap;
        snap.ts = epochNow();
        snap.monoUs = static_cast<UInt64>(elapsedS(start) * 1000000.0);
        snap.upS = static_cast<UInt32>(elapsedS(start));
        snap.mode = got ? reactive::modeName(out.mode) : "blind";
        snap.clearanceMm = out.clearanceMm;
        snap.hits = out.corridorHits;
        const Float64 windowS = elapsedS(lastStatus);
        snap.revPerS = windowS > 0.0 ? static_cast<Float64>(windowRevs) / windowS : 0.0;
        snap.timeouts = timeouts;
        snap.revolutions = revolutions;
        snap.lidarLost = lidarLost;
        snap.lidarHealth = lidar::device().health;
        snap.lidarSpinning = lidar::isSpinning();
        snap.dry = opt.dry;
        snap.pilotMode = pilotModeOf(opt);
        snap.picoOpen = !opt.dry && !link.lost;
        snap.picoHeard = link.heard;
        snap.picoSilentMs = opt.dry ? -1 : carlink::silentForMs();
        snap.replyOk = replies.ok;
        snap.replyErr = replies.err;
        snap.picoArmed = replies.armed;
        snap.pico = picoPhrase(snap);
        if(opt.feed)
        {
            // Every tick. viewfeed holds it as the state the NEXT viewer is
            // owed before it is shown a point, and puts it on the wire at the
            // 5 Hz section 2 asks for - the rate is the socket's business, the
            // content is this one struct.
            viewfeed::publishBoard(boardFrom(snap));

            // WHAT THE CAR IS DOING, which is a different question from what
            // was decided - and until now nothing called this at all, so every
            // field of CTLSTATE carried its absent sentinel and viewfeed's
            // "refuse to re-trim an armed car" guard read armed = 0 forever.
            // A guard that cannot observe the thing it guards against is not a
            // safety mechanism, it is a comment; this is what makes it real.
            //
            // Three of these are MEASURED, read out of the Pico's own reply to
            // the STEER this loop already sends. throttleMilli is the decision
            // rather than a measurement, which is what the field means: what
            // was sent, not what the wheels did with it.
            viewfeed::Applied ap;
            ap.steerNowMilli = static_cast<Int16>(replies.steerNowMilli);
            ap.throttleMilli = static_cast<Int16>(out.throttle * 1000.0f);
            ap.escUs = replies.escUs < 0
                ? bibowire::ESC_ABSENT
                : static_cast<UInt16>(replies.escUs);
            // Unknown reads as NOT armed, and that is the permissive direction
            // for the tuning guard rather than the dangerous one: it lasts a
            // single tick before the first reply lands, and a dry run - where
            // it lasts forever - refuses tuning on "no Pico" long before this
            // is consulted.
            ap.armed = replies.armed > 0 ? 1u : 0u;
            ap.pilotMode = pilotModeOf(opt);
            ap.picoSilentMs = snap.picoSilentMs < 0
                ? bibowire::PICO_SILENT_ABSENT
                : static_cast<UInt32>(snap.picoSilentMs);
            viewfeed::applied(ap);
        }

        // ---- who writes steer and throttle this tick --------------------------
        //
        // In MANUAL a viewer's CONTROL writes them and the autonomy does not.
        // Section 6's chain, in its order:
        //
        //   ESTOP / DEAD  STOP to the Pico - neutral, disarm, release - and it
        //                 does NOT recover on its own, because a link that came
        //                 back is not the same fact as an operator who is ready.
        //   SOFT          throttle forced to 0, steering HELD at the last
        //                 commanded value rather than centred.
        //   LIVE          obeyed. What control() returns is already gated: on an
        //                 epoch or mode disagreement its throttle is 0 before it
        //                 ever reaches this loop.
        //
        // SOMETHING IS SENT EVERY TICK IN EVERY STATE, which section 6 calls
        // load-bearing and means literally: the Pico's own 400 ms deadman must
        // fire only when this program has stopped running, never routinely, or
        // it becomes a last resort nobody notices has gone off.
        //
        // No holder and no command are treated as the dead case, not as an idle
        // one - in MANUAL the autonomy is not driving, so if the operator is not
        // either then nobody is.
        Str steerLine;
        Str escLine;

        // WHAT WAS ACTUALLY COMMANDED, for the once-a-second line below.
        //
        // That line printed out.steer and out.throttle whatever the mode, and in
        // MANUAL the autonomy's numbers go nowhere at all - so a manual run with
        // no viewer connected reported "steer +0.37 thr 0.24" while the only
        // thing on the wire was STOP. A console that disagrees with the car is
        // the same failure as a panel that does, and this one was mine.
        //
        // `modeWord` empty means "use describe()", which is right for the
        // autonomy's modes and wrong for MANUAL: cruise / slow / blind describe
        // a decision nobody is acting on, where the deadman's state is the thing
        // an operator holding a key needs to see.
        Int32 sentSteerMilli = static_cast<Int32>(out.steer * 1000.0f);
        Int32 sentThrottleMilli = static_cast<Int32>(out.throttle * 1000.0f);
        Str modeWord;

        if(pilotModeOf(opt) == static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL))
        {
            const viewfeed::Drive dm = viewfeed::drive();
            bibowire::Control cmd;
            const Bool haveCmd = viewfeed::control(&cmd);

            // Named by the protocol module rather than spelled again here, so
            // the console, the viewer's panel and the board's CTLSTATE cannot
            // come to disagree about what a 2 means. The range guard is for a
            // byte that arrived wrong rather than for one this build can
            // produce, and the safe reading of a wrong one is "stopped".
            const bibowire::deadman::State ds = dm.deadman <= 3u
                ? static_cast<bibowire::deadman::State>(dm.deadman)
                : bibowire::deadman::State::STATE_DEAD;
            modeWord = Str("manual ") + bibowire::deadman::stateName(ds);
            if(!dm.haveHolder)
            {
                // Not a deadman state at all: with nobody holding the wheel the
                // timer does not apply (section 6), and printing "live" here
                // would read as a healthy link to a car nobody is driving.
                modeWord = "manual  nobody holding";
            }

            if(dm.estopLatched || dm.deadman >= 2u || !dm.haveHolder || !haveCmd)
            {
                heldSteerMilli = 0;
                escLine = proto::stop();
                sentSteerMilli = 0;
                sentThrottleMilli = 0;
            }
            else
            {
                if(dm.deadman == 0u)
                {
                    heldSteerMilli = cmd.steerMilli;
                }
                steerLine = proto::steer(static_cast<Float32>(heldSteerMilli) / 1000.0f);

                // Throttle only while LIVE, only while this run is allowed to
                // move the car, and only while the board is answering. The last
                // of those is L3 and it stays: a cable this program cannot hear
                // is not one to push throttle down.
                const Bool mayPush = dm.deadman == 0u && opt.arm && !boardSilent;
                escLine = mayPush && cmd.throttleMilli > 0
                    ? proto::escUs(escPulseFor(static_cast<Float32>(cmd.throttleMilli) / 1000.0f))
                    : proto::command("ESC", "NEUTRAL");

                sentSteerMilli = heldSteerMilli;
                sentThrottleMilli = mayPush ? cmd.throttleMilli : 0;
            }
        }
        else
        {
            steerLine = proto::steer(out.steer);
            escLine = escLineFor(status, out, boardSilent, opt.arm);
        }
        if(opt.dry)
        {
            std::printf(
                "dry  %-12s %-12s %s  dt %3d ms\n",
                steerLine.c_str(),
                escLine.c_str(),
                describe(status, out, got).c_str(),
                dtMs
            );
        }
        else
        {
            // Empty means "there is no such line this tick", which happens in
            // MANUAL's stopped states: STOP already neutralises, disarms and
            // releases, so a STEER beside it would be commanding a servo that
            // was just released. An empty line written to the port would be a
            // bare newline the Pico's parser has to classify, so it is skipped
            // rather than sent.
            if(!steerLine.empty())
            {
                sendLine(steerLine, link);
            }
            if(!escLine.empty())
            {
                sendLine(escLine, link);
            }

            // The operator's trim, after the car's motion and before the
            // replies are read - so the OK or ERR the Pico answers each of
            // these with is counted by the same readReplies below rather than
            // being left in the buffer to be read as an answer to the NEXT
            // tick's STEER.
            //
            // Inside the !opt.dry branch on purpose: a dry run has no port to
            // send this down. viewfeed refuses these verbs with "no Pico" in
            // that case anyway, because the BOARD frame a dry run publishes
            // says picoLink is down - so the queue should be empty here, and
            // this is the second of the two places that has to be true.
            for(Int32 sent = 0; sent < TUNE_PER_TICK; ++sent)
            {
                viewfeed::Tune t;
                if(!viewfeed::tune(&t))
                {
                    break;
                }
                const Str line = tuneLine(t);
                if(line.empty())
                {
                    continue;
                }
                sendLine(line, link);
            }
            if(!readReplies(lines, replies, link) && !link.lost)
            {
                link.lost = true;
                std::printf("pico link lost: %s\n", carlink::detail().c_str());
            }
        }

        // ---- once a second ----------------------------------------------------------
        if(elapsedMs(lastStatus) >= STATUS_EVERY_MS)
        {
            const Str what = modeWord.empty() ? describe(status, out, got) : modeWord;
            std::printf(
                "%6.1f s  %s  steer %+.2f  thr %.2f  %5.1f rev/s  timeouts %llu  %s\n",
                elapsedS(start),
                what.c_str(),
                static_cast<Float64>(sentSteerMilli) / 1000.0,
                static_cast<Float64>(sentThrottleMilli) / 1000.0,
                snap.revPerS,
                static_cast<unsigned long long>(snap.timeouts),
                snap.pico.c_str()
            );
            windowRevs = 0;
            lastStatus = now;

            // The same second, for the phone - and from the SAME STRUCT the
            // viewer's BOARD frame was filled from a few lines above, which is
            // the whole of section 5's obligation on this file. The console,
            // the page and the viewer cannot now disagree, because there is
            // only one place the numbers come from.
            writeWhole(STATUS_FILE, jsonFrom(snap));

            // A lost link is retried here, once a second, rather than every tick:
            // open() probes the device and a board that is being replugged does
            // not need ten attempts a second to notice it. The car has already
            // been stopped by its own deadman; what is owed is a quiet reconnect.
            if(!opt.dry && link.lost)
            {
                if(openPico(linkCfg, opt.arm, link))
                {
                    std::printf("pico %s: link back\n", opt.picoPort.c_str());
                }
            }
        }
    }

    // ---- shutdown -----------------------------------------------------------------------
    // STOP is neutral, disarm, release - everything off - and it goes before the
    // lidar is touched, because the car is the thing that can hurt somebody and
    // the lidar is not. Then the motor, then the ports.
    if(!opt.dry)
    {
        sendLine(proto::stop(), link);
        sleepMs(50);   // long enough for the board's reply to the STOP to land
        // The verdict is kept, not cast away: a board that was gone when the
        // STOP went out is the one thing worth saying on the way out, because
        // the car is then relying on its own deadman.
        if(!readReplies(lines, replies, link) && !link.lost)
        {
            link.lost = true;
            std::printf("pico link lost at STOP: %s\n", carlink::detail().c_str());
        }
    }

    const Float64 ran = elapsedS(start);
    std::printf(
        "%s: %llu revolutions, %llu timeouts in %.1f s (%.2f rev/s over the run)\n",
        interrupted != 0 ? "interrupted" : "done",
        static_cast<unsigned long long>(revolutions),
        static_cast<unsigned long long>(timeouts),
        ran,
        ran > 0.0 ? static_cast<Float64>(revolutions) / ran : 0.0
    );
    if(!opt.dry)
    {
        std::printf(
            "pico: %llu lines sent, %llu dropped, %llu ok, %llu err, %llu other\n",
            static_cast<unsigned long long>(carlink::txLines()),
            static_cast<unsigned long long>(carlink::dropped()),
            static_cast<unsigned long long>(replies.ok),
            static_cast<unsigned long long>(replies.err),
            static_cast<unsigned long long>(replies.other)
        );
    }

    if(viewer.costTicks > 0)
    {
        std::printf(
            "feed: %llu frames published to %s, viewer cost per tick avg %.0f us, max %.0f us\n",
            static_cast<unsigned long long>(viewer.frames),
            viewer.serving ? "the feed and the scan file" : "the scan file only",
            viewer.costSumUs / static_cast<Float64>(viewer.costTicks),
            viewer.costMaxUs
        );
    }

    // What bibowire cost, measured rather than asserted - the same argument the
    // line above makes for the text feed, and section 9's claim made readable
    // off the running system instead of believed.
    if(viewer.wire)
    {
        const viewfeed::Counters wireCount = viewfeed::counters();
        std::printf(
            "viewfeed: %llu viewers accepted, %llu refused, %llu frames sent, %llu dropped, "
            "%llu control datagrams (%llu stale), encode avg %u ns, max %u ns\n",
            static_cast<unsigned long long>(wireCount.accepted),
            static_cast<unsigned long long>(wireCount.refused),
            static_cast<unsigned long long>(wireCount.txFrames),
            static_cast<unsigned long long>(wireCount.txDroppedFrames),
            static_cast<unsigned long long>(wireCount.rxControl),
            static_cast<unsigned long long>(wireCount.rxControlStale),
            static_cast<unsigned>(wireCount.encodeAvgNs),
            static_cast<unsigned>(wireCount.encodeMaxNs)
        );
    }

    if(!lidar::motorOff())
    {
        std::printf("lidar motor off: %s\n", lidar::reason().c_str());
    }
    lidar::close();
    carlink::close();

    // The viewers last: they were watching a car that has now stopped, and
    // their sockets closing is how they learn it. The scan file goes with
    // them - a revolution from a pilot that has exited is not a picture of
    // anything, and the page reads its absence as "pilot not running".
    feed::stop();
    // BYE(SHUTDOWN) with a sentence, rather than a socket that simply stops
    // answering: on this link silence already means four other things, and the
    // one time the board knows why it is going is the one time it can say so.
    viewfeed::stop();
    static_cast<Void>(std::remove(scanwire::SCAN_FILE));

    // A signal is a person asking, and 0 is the answer to a request that was
    // carried out. A timed run that saw no revolution at all is the other case:
    // it did what it was told and measured nothing, and that must not look like
    // success to whatever launched it. Nor must a run whose lidar stopped
    // arriving partway through - see lidarLost - however many revolutions it
    // had counted before then.
    if(interrupted != 0)
    {
        return 0;
    }
    return revolutions > 0 && !lidarLost ? 0 : 1;
}
