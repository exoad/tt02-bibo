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
// WHAT IS COUNTED
//
// Every revolution, every grab that timed out, every line the board answered
// and whether it was OK or ERR - and, from carlink, every line sent and every
// one dropped. The status line once a second and the summary at exit print
// them, because the failure this project keeps finding in its own code is the
// one that reports success while measuring nothing: a pilot that ran for a
// minute and saw zero revolutions exits 1 and says so.

#include "shared.hxx"

#include "lidar.hxx"
#include "link.hxx"
#include "proto.hxx"
#include "reactive.hxx"

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

  Void writeStatus(const Str& json)
  {
      const Str tmp = Str(STATUS_FILE) + ".tmp";
      std::FILE* f = std::fopen(tmp.c_str(), "w");
      if(f == nullptr)
      {
          return;    // no /tmp here (a laptop): the page is a Linux thing
      }
      std::fputs(json.c_str(), f);
      std::fclose(f);
      static_cast<Void>(std::rename(tmp.c_str(), STATUS_FILE));
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
      Float32 forwardDeg = 0.0f;
      Float64 seconds = -1.0;   // negative: until a signal
  };

  Void usage()
  {
      std::printf(
          "pilot [--lidar PORT] [--pico PORT] [--dry] [--arm] [--forward DEG] [--seconds N]\n"
          "  --lidar PORT   the C1's serial device        (default /dev/ttyUSB0)\n"
          "  --pico PORT    the car's serial device       (default /dev/ttyACM0)\n"
          "  --dry          never open the Pico; print each decision instead\n"
          "  --arm          send ESC ARM once the link is up, so throttle is obeyed\n"
          "  --forward DEG  the raw lidar angle that is straight ahead (default 0)\n"
          "  --seconds N    run for N seconds, then stop  (default: until SIGINT)\n"
      );
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
  [[nodiscard]] Str escLineFor(reactive::Status status, const reactive::Outputs& out, Bool silent)
  {
      const Bool trusted = status == reactive::Status::STATUS_OK;
      const Bool forward = trusted && !out.stop && out.throttle > 0.0f;
      if(!forward || silent)
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

    // ---- the loop ------------------------------------------------------------------
    reactive::State   state;
    reactive::Outputs out;
    reactive::Status  status = reactive::Status::STATUS_BLIND;
    Vec<reactive::Ray> rays;
    Vec<Str>           lines;
    Replies            replies;

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
        const Bool got = lidar::grab(rays, REV_WAIT_MS);
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

        const Str steerLine = proto::steer(out.steer);
        const Str escLine = escLineFor(status, out, boardSilent);
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
            sendLine(steerLine, link);
            sendLine(escLine, link);
            if(!readReplies(lines, replies, link) && !link.lost)
            {
                link.lost = true;
                std::printf("pico link lost: %s\n", carlink::detail().c_str());
            }
        }

        // ---- once a second ----------------------------------------------------------
        if(elapsedMs(lastStatus) >= STATUS_EVERY_MS)
        {
            const Float64 windowS = elapsedS(lastStatus);
            Array<Char, 48> pico{};
            if(opt.dry)
            {
                std::snprintf(pico.data(), pico.size(), "pico dry");
            }
            else
            {
                // -1 is no descriptor, and "silent -1 ms" would be a number
                // standing in for a fact.
                const Int32 silent = carlink::silentForMs();
                Array<Char, 24> silence{};
                if(silent < 0)
                {
                    std::snprintf(silence.data(), silence.size(), "%-15s", "no link");
                }
                else
                {
                    std::snprintf(silence.data(), silence.size(), "silent %5d ms", silent);
                }
                std::snprintf(
                    pico.data(),
                    pico.size(),
                    "pico %s  ok %llu  err %llu",
                    silence.data(),
                    static_cast<unsigned long long>(replies.ok),
                    static_cast<unsigned long long>(replies.err)
                );
            }
            const Float64 revPerS = windowS > 0.0 ? static_cast<Float64>(windowRevs) / windowS : 0.0;
            std::printf(
                "%6.1f s  %s  steer %+.2f  thr %.2f  %5.1f rev/s  timeouts %llu  %s\n",
                elapsedS(start),
                describe(status, out, got).c_str(),
                static_cast<Float64>(out.steer),
                static_cast<Float64>(out.throttle),
                revPerS,
                static_cast<unsigned long long>(timeouts),
                pico.data()
            );
            windowRevs = 0;
            lastStatus = now;

            // The same second, for the phone. `pico` is the printed phrase, so
            // the page shows exactly what the console showed.
            Array<Char, 320> json{};
            std::snprintf(
                json.data(),
                json.size(),
                "{\"ts\":%.3f,\"mode\":\"%s\",\"clearanceMm\":%.0f,\"hits\":%d,"
                "\"revPerS\":%.1f,\"timeouts\":%llu,\"revolutions\":%llu,"
                "\"lidarLost\":%s,\"pico\":\"%s\"}\n",
                epochNow(),
                got ? reactive::modeName(out.mode) : "blind",
                static_cast<Float64>(out.clearanceMm),
                out.corridorHits,
                revPerS,
                static_cast<unsigned long long>(timeouts),
                static_cast<unsigned long long>(revolutions),
                lidarLost ? "true" : "false",
                pico.data()
            );
            writeStatus(json.data());

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

    if(!lidar::motorOff())
    {
        std::printf("lidar motor off: %s\n", lidar::reason().c_str());
    }
    lidar::close();
    carlink::close();

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
