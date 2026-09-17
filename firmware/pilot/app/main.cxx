// pilot - the service program on the companion board. Each lidar revolution is
// one tick: the decision goes to the viewer, and a STEER and an ESC line go to
// the Pico.
//
//   pilot [--lidar PORT] [--pico PORT] [--dry] [--manual] [--arm] [--forward DEG] [--seconds N]
//
// WHO DRIVES
//   DRIVE (default)  reactive::step decides. Throttle only for a forward decision
//                    from a scan it trusted, mapped onto cal.hxx's THROTTLE_CAL_MIN..
//                    THROTTLE_CAL_MAX, and only with --arm, which sends ESC ARM and
//                    SERVO ON whenever the Pico is opened. The autonomy's reverse is
//                    sent as neutral: this ESC's reverse is a brake-then-push sequence
//                    nothing here drives. A viewer's ESTOP or DISARM sends STOP and
//                    ends the run.
//   MANUAL           a viewer's CONTROL drives (docs/bibowire.md section 6). Only a
//                    viewer's COMMAND ARM lets throttle through, so
//                    bibo-pilot.service runs --manual without --arm and the car comes
//                    up held still.
//   --dry            never opens the Pico; prints each decision instead.
//
// TIMING
//   A tick waits up to REV_WAIT_MS for a revolution, and a tick without one sends
//   neutral. While it waits, the last lines are re-sent every PICO_KEEPALIVE_MS
//   (grabHeld), so the Pico's watchdog, bibowire::PICO_DEADMAN_MS, fires only when
//   this program has stopped. A lost link holds neutral instead.
//
// THE PICO
//   Opened with STOP, so a run that died armed is disarmed, then PING and the saved
//   trim; a lost link is reopened the same way, once a BOARD has reported it down.
//   Its replies are read through carrules::fold and forward pulses come from
//   carrules::forwardPulse, the same rules car programs use. Shutdown sends STOP
//   before the lidar is touched. A run that saw no revolution, or lost the lidar
//   partway, exits 1.
#include "shared.hxx"

#include "carrules.hxx"
#include "chain.hxx"
#include "lidar.hxx"
#include "link.hxx"
#include "proto.hxx"
#include "reactive.hxx"
#include "odom.hxx"
#include "tags.hxx"
#include "trimfile.hxx"
#include "viewfeed.hxx"

// THROTTLE_CAL_MIN/MAX: measured on a motor this car no longer has, and kept for
// the autonomy because the band is too narrow to launch the car. Re-measure on a
// stand before widening it in cal.hxx.
#include "chassis/cal.hxx"

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{
  constexpr Int32 REV_WAIT_MS = 200;

  // How often the held lines are re-sent while a tick waits. Above a normal
  // revolution's interval, so the common tick sends nothing extra.
  constexpr Int32 PICO_KEEPALIVE_MS = 80;

  static_assert(
      PICO_KEEPALIVE_MS + bibowire::PICO_HOP_BUDGET_MS <= bibowire::PICO_DEADMAN_MS,
      "a keepalive must reach the board before its watchdog fires, or a late revolution stops the car"
  );

  constexpr Int32 STATUS_EVERY_MS = 1000;

  // Consecutive timeouts, after the first revolution, before the lidar is lost.
  constexpr Int32 LIDAR_LOST_TIMEOUTS = 10;

  constexpr Int32 ESC_MIN_US = THROTTLE_CAL_MIN;
  constexpr Int32 ESC_MAX_US = THROTTLE_CAL_MAX;
  static_assert(ESC_MAX_US > ESC_MIN_US, "cal.hxx throttle band is empty or inverted");

  // The build a viewer is told: the build's own stamp, else the compile time.
#if defined(BOARD_BUILD_STAMP)
  constexpr CharSeq BOARD_BUILD = BOARD_BUILD_STAMP;
#else
  constexpr CharSeq BOARD_BUILD = __DATE__ " " __TIME__;
#endif

  // Set by the signal handler; sig_atomic_t is the type a handler may write.
  volatile std::sig_atomic_t interrupted = 0;

  // The camera's calibration, read once at startup (tags::defaultCameraPath).
  // Handed to the detector when a bundle starts it; without one the tags
  // carry no range and follow does not move.
  tags::Intrinsics camCal;

  // --no-lidar: grabHeld sleeps the slice instead of asking a lidar that is
  // not open, which would answer at once and turn the tick into a hot loop.
  Bool blind = false;

  Void onInterrupt(Int32)
  {
      interrupted = 1;
  }

  struct Options
  {
      Str     lidarPort = bibo::LIDAR_PORT;
      Str     picoPort = bibo::PICO_PORT;
      Bool    dry = false;
      Bool    arm = false;
      Bool    manual = false;   // a viewer's CONTROL drives; a startup flag only
      Bool    noLidar = false;  // the bench: no lidar opened, every scan empty
      Float32 forwardDeg = 0.0f;
      Float64 seconds = -1.0;   // negative: until a signal
  };

  Void usage()
  {
      std::printf(
          "pilot [--lidar PORT] [--pico PORT] [--dry] [--manual] [--arm]"
          " [--no-lidar] [--forward DEG] [--seconds N]\n"
          "  --lidar PORT   the C1's serial device        (default %s)\n"
          "  --pico PORT    the car's serial device       (default %s)\n"
          "  --dry          never open the Pico; print each decision instead\n"
          "  --manual       a viewer's CONTROL drives, not the autonomy\n"
          "  --no-lidar     open no lidar and run blind, every scan empty: the bench,\n"
          "                 for the camera and the viewer with the car elsewhere\n"
          "  --arm          send ESC ARM once the link is up, so throttle is obeyed;\n"
          "                 with --manual a viewer's ARM is still what lets throttle through\n"
          "  --forward DEG  the raw lidar angle that is straight ahead (default 0)\n"
          "  --seconds N    run for N seconds, then stop  (default: until SIGINT)\n",
          bibo::LIDAR_PORT,
          bibo::PICO_PORT
      );
  }

  // Who writes steer and throttle, for BOARD, DECIDE's source and CTLSTATE alike.
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

  // strtod, but the whole token must be the number: "12x" is refused.
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
          else if(flag == "--no-lidar")
          {
              o.noLidar = true;
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
      // A dry run never opens the Pico, so MANUAL would report a mode that
      // drives nothing.
      if(o.dry && o.manual)
      {
          std::printf("--dry and --manual contradict: a dry run never opens the Pico\n");
          return false;
      }
      return true;
  }

  // The autonomy's throttle line: a pulse within cal.hxx's band for a forward
  // decision from a trusted scan while this run may drive (--arm) and the Pico is
  // speaking, NEUTRAL otherwise. NEUTRAL is accepted while disarmed, so a run
  // without --arm is not refused every tick.
  [[nodiscard]] Str escLineFor(reactive::Status s, const reactive::Outputs& o, Bool quiet, Bool arm)
  {
      const Bool forward = s == reactive::Status::STATUS_OK && !o.stop && o.throttle > 0.0f;
      if(!forward || quiet || !arm)
      {
          return proto::command("ESC", "NEUTRAL");
      }
      return proto::escUs(carrules::forwardPulse(o.throttle, ESC_MIN_US, ESC_MAX_US));
  }

  // What the Pico has said. carrules::fold keeps the keys car programs also use;
  // the rest come from the same OK drive line (firmware/app/main.cxx printDrive).
  struct Replies
  {
      carrules::Board pico;      // armed, servo_on, esc, esc limits, esc_rev, stale, ERRs
      UInt64 ok = 0;
      UInt64 other = 0;          // INFO, banner text, anything unasked
      Int32  steerNowMilli = 0;
      Int32  servoUs = -1;       // -1: not reported
      Int32  servoMinUs = -1;
      Int32  servoMaxUs = -1;
  };

  // The link as this program sees it. `heard`: carlink::open() does no exchange,
  // so a freshly opened port is not trusted with throttle until a line arrives.
  struct Link
  {
      carlink::Config cfg;
      Bool lost = false;        // declared gone; retried once a second
      Bool heard = false;       // a line has arrived since the port last opened
      Bool stallSaid = false;   // a stalled write has been mentioned once
  };

  // key's value in an OK drive line's fields (the '=' is part of the key), or
  // fallback when the line lacks it.
  [[nodiscard]] Int32 driveKey(const Str& fields, const Char* key, Int32 fallback)
  {
      Int32 v = 0;
      return proto::fieldInt(fields, key, v) ? v : fallback;
  }

  // Everything the Pico said since the last call; ERR lines are printed in full.
  // false when the link has gone, after its last lines have still been read.
  [[nodiscard]] Bool readReplies(Vec<Str>& scratch, Replies& tally, Link& link)
  {
      scratch.clear();
      const carlink::Result r = carlink::drain(scratch);
      if(!scratch.empty())
      {
          link.heard = true;   // any line at all: something is on the other end
      }
      for(const Str& line : scratch)
      {
          const proto::Reply reply = proto::read(line);
          if(reply.kind == proto::Kind::KIND_OK)
          {
              ++tally.ok;
          }
          else if(reply.kind == proto::Kind::KIND_ERR)
          {
              std::printf("pico: %s\n", reply.line.c_str());
          }
          else if(reply.kind != proto::Kind::KIND_EMPTY)
          {
              ++tally.other;
          }
          if(carrules::fold(tally.pico, line))
          {
              tally.steerNowMilli = driveKey(reply.rest, "steer_now=", 0);
              tally.servoUs = driveKey(reply.rest, "servo=", -1);
              tally.servoMinUs = driveKey(reply.rest, "servo_min=", -1);
              tally.servoMaxUs = driveKey(reply.rest, "servo_max=", -1);
          }
      }
      return r != carlink::Result::RESULT_CLOSED && r != carlink::Result::RESULT_NOT_OPEN;
  }

  // Sends one line, and says once when it went nowhere. Only CLOSED and NOT_OPEN
  // mark the link lost. WRITE_FAILED is a stall on a good descriptor: treating it
  // as lost would reopen the port and re-send ESC ARM, dropping the throttle to
  // neutral mid-drive.
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

  // The lines the Pico was last sent, re-sent while a tick waits. Empty: nothing
  // yet, or a dry run.
  struct Hold
  {
      Str steer;
      Str esc;
  };

  // What is held once the link is lost, and after it reopens: neutral and no
  // STEER, so a throttle decided before the loss never reaches the Pico that
  // comes back.
  Void holdNeutral(Hold& hold)
  {
      hold.steer.clear();
      hold.esc = proto::command("ESC", "NEUTRAL");
  }

  // One revolution, waited for in PICO_KEEPALIVE_MS slices with the held lines
  // re-sent between them. The held lines and not a neutral: whether to go neutral
  // is the tick's decision, made at REV_WAIT_MS; a neutral here would cut the
  // throttle whenever a revolution ran late.
  // The odometry bundle's working half is the dead reckoning (odom.hxx),
  // stepped in the tick from the Pico's replies. Loading it starts a frame
  // where the car stands; unloading ends it. Called whenever the chain
  // changed.
  odom::State odomState;
  Bool odomOn = false;

  Void syncOdometry(const chain::Chain& live)
  {
      const Bool want = live.has(chain::ID_ODOMETRY);
      if(want == odomOn)
      {
          return;
      }
      odomOn = want;
      odomState = odom::State();
      const CharSeq said = want ? "odometry: a new frame, from where the car stands" : "odometry: stopped";
      std::printf("%s\n", said);
      viewfeed::publishEvent(
          bibowire::Severity::SEVERITY_INFO,
          bibowire::EVENT_CODE_ODOM_FRAME,
          said
      );
  }

  // The apriltag bundle's working half is a thread (tags.hxx); its behaviour
  // in the chain says nothing. Loading it starts the detector and unloading
  // it stops it, on the EDGE, so a load the detector refused is said once
  // and not every tick. Called whenever the chain changed.
  Void syncTags(const chain::Chain& live)
  {
      static Bool was = false;
      static Bool followWas = false;
      // follow drives on the detections, so loading it runs the detector too.
      const Bool follow = live.has(chain::ID_FOLLOW);
      if(follow && !followWas && !camCal.calibrated)
      {
          viewfeed::publishEvent(
              bibowire::Severity::SEVERITY_WARN,
              bibowire::EVENT_CODE_BUNDLE,
              "follow: the camera is not calibrated, so no tag has a range - it will aim "
              "and not move; see docs/bundles.md"
          );
      }
      followWas = follow;
      const Bool want = live.has(chain::ID_APRILTAG) || follow;
      if(want == was)
      {
          return;
      }
      was = want;
      if(!want)
      {
          const tags::Stats s = tags::stats();
          tags::stop();
          std::printf(
              "apriltag: detector stopped after %llu frames, %llu tags\n",
              static_cast<unsigned long long>(s.frames),
              static_cast<unsigned long long>(s.seen)
          );
          return;
      }
      tags::Config cfg;
      cfg.cal = camCal;
      const Char* cores = std::getenv("BIBO_TAGS_CORES");
      if(cores != nullptr && cores[0] != '\0')
      {
          cfg.cores = cores;
      }
      const Char* npuModel = std::getenv("BIBO_TAGS_NPU");
      if(npuModel != nullptr && npuModel[0] != '\0')
      {
          cfg.npuModel = npuModel;
      }
      Str why;
      if(!tags::start(cfg, why))
      {
          std::printf("apriltag: not started - %s\n", why.c_str());
          viewfeed::publishEvent(
              bibowire::Severity::SEVERITY_WARN,
              bibowire::EVENT_CODE_BUNDLE,
              "apriltag: loaded, but not detecting - " + why
          );
          return;
      }
      std::printf(
          "apriltag: detector running on the %s - tag36h11, decimate %.1f, %u fps asked of the camera, %s\n",
          tags::stats().backend.c_str(),
          static_cast<Float64>(cfg.decimate),
          static_cast<unsigned>(cfg.fps),
          camCal.calibrated ? "ranges from the calibration" : "no calibration so no ranges"
      );
      viewfeed::publishEvent(
          bibowire::Severity::SEVERITY_INFO,
          bibowire::EVENT_CODE_BUNDLE,
          "apriltag: detector running (tag36h11)"
      );
  }

  [[nodiscard]] Bool grabHeld(Vec<reactive::Ray>& out, Vec<UInt8>* q, const Hold& hold, Link& link)
  {
      Int32 waited = 0;
      while(waited < REV_WAIT_MS)
      {
          const Int32 left = REV_WAIT_MS - waited;
          const Int32 slice = left < PICO_KEEPALIVE_MS ? left : PICO_KEEPALIVE_MS;
          if(blind)
          {
              out.clear();
              if(q != nullptr)
              {
                  q->clear();
              }
              sleepMs(slice);
          }
          else if(lidar::grab(out, slice, q))
          {
              return true;
          }
          waited += slice;
          // The last slice's timeout is the tick's: the caller sends at once.
          if(waited >= REV_WAIT_MS)
          {
              break;
          }
          // A viewer's ESTOP does not wait for the tick: STOP goes in place of the
          // held lines. A dry run holds nothing and has no port.
          if(!hold.esc.empty() && viewfeed::drive().estopLatched)
          {
              sendLine(proto::stop(), link);
              continue;
          }
          if(!hold.steer.empty())
          {
              sendLine(hold.steer, link);
          }
          if(!hold.esc.empty())
          {
              sendLine(hold.esc, link);
          }
      }
      return false;
  }

  // Opens the Pico, and again after every lost link: STOP, so a run that died
  // armed is disarmed; PING, so the first tick hears it; the saved trim, before
  // anything is armed. With --arm, ESC ARM. Outside MANUAL also SERVO ON, because
  // the autonomy steers from its first tick; in MANUAL the viewer's ARM engages it.
  [[nodiscard]] Bool openPico(const Options& opt, const trimfile::Store& trim, Link& link)
  {
      // A board behind a fresh descriptor, or none, has not been heard from.
      link.heard = false;
      const carlink::Result r = carlink::open(link.cfg);
      if(r != carlink::Result::RESULT_OK)
      {
          std::printf(
              "pico %s: %s - %s\n",
              link.cfg.where.c_str(),
              carlink::why(r),
              carlink::detail().c_str()
          );
          return false;
      }
      link.lost = false;
      sendLine(proto::stop(), link);
      sendLine(proto::command("PING"), link);
      for(const Str& line : trimfile::lines(trim))
      {
          sendLine(line, link);
      }
      if(opt.arm)
      {
          sendLine(proto::command("ESC", "ARM"), link);
          if(!opt.manual)
          {
              sendLine(proto::command("SERVO", "ON"), link);
          }
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

  // What publishing to the viewers cost per tick, for the exit summary.
  struct Viewer
  {
      Float64 costMaxUs = 0.0;
      Float64 costSumUs = 0.0;
      UInt64  costTicks = 0;
      Bool    wire = false;      // viewfeed::start succeeded
  };

  // What the board says about itself, filled once a tick and read by both the
  // BOARD frame and the console line, so the two cannot disagree
  // (docs/bibowire.md section 5).
  struct Snapshot
  {
      UInt64  monoUs = 0;
      UInt32  upS = 0;
      Float64 revPerS = 0.0;
      UInt64  timeouts = 0;
      UInt64  revolutions = 0;
      Int32   lidarHealth = -1;
      Bool    lidarSpinning = false;
      Bool    dry = false;
      UInt8   pilotMode = 2;       // pilotModeOf
      Bool    picoOpen = false;
      Bool    picoHeard = false;
      Int32   picoSilentMs = -1;   // -1: no link
      Int32   picoArmed = -1;      // the Pico's own armed=; -1: not reported
      UInt64  replyOk = 0;
      UInt64  replyErr = 0;
      Str     pico;                // the console phrase, from the fields above
  };

  // -1 (the device did not answer) is the wire's "unknown", never 0 ("good").
  [[nodiscard]] UInt8 healthByte(Int32 health)
  {
      return health >= 0 && health <= 2 ? static_cast<UInt8>(health) : bibowire::HEALTH_ABSENT;
  }

  [[nodiscard]] Str picoPhrase(const Snapshot& s)
  {
      Array<Char, 48> pico{};
      if(s.dry)
      {
          std::snprintf(pico.data(), pico.size(), "pico dry");
          return Str(pico.data());
      }
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

  // The snapshot as the viewer is told it. What this program cannot measure keeps
  // its ABSENT sentinel; viewfeed fills its own fields.
  [[nodiscard]] bibowire::BoardState boardFrom(const Snapshot& s)
  {
      bibowire::BoardState b;
      b.tMonoUs = s.monoUs;
      b.upS = s.upS;
      b.cpuCentiC = bibowire::CPU_ABSENT;
      b.battMilliV = bibowire::BATT_ABSENT;
      b.picoLink = s.dry || !s.picoOpen ? 0u : (s.picoHeard ? 1u : 2u);
      b.picoArmed = s.picoArmed < 0 ? 2u : (s.picoArmed != 0 ? 1u : 0u);
      b.pilotMode = s.pilotMode;
      b.lidarHealth = healthByte(s.lidarHealth);
      b.lidarSpinning = s.lidarSpinning ? 1u : 0u;
      b.picoSilentMs = s.picoSilentMs < 0
          ? bibowire::PICO_SILENT_ABSENT
          : static_cast<UInt32>(s.picoSilentMs);
      b.revolutions = static_cast<UInt32>(s.revolutions);
      b.timeouts = static_cast<UInt32>(s.timeouts);
      return b;
  }

  // At most this many tuning lines a tick, so a dragged slider is spread over
  // ticks the serial port can carry; the rest wait for the next tick.
  constexpr Int32 TUNE_PER_TICK = 2;

  // One tuning request as firmware/app/main.cxx parses it; empty for a verb this
  // build does not send. Whole microseconds only, so no locale is involved.
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
          case bibowire::Verb::VERB_SET_ESC_REVERSE:
              std::snprintf(args.data(), args.size(), "%u", a1);
              return proto::command("ESCREVERSE", args.data());
          case bibowire::Verb::VERB_SET_SLEW:
              // No axis word sets both.
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

  // Whole centi-degrees and millimetres, as fine as the C1 measures. 360 degrees
  // wraps to 0, never 36000, which the encoder refuses.
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

  // Thousandths of -1..1, the wire's unit.
  [[nodiscard]] Int16 milliOf(Float32 fraction)
  {
      const Float32 clamped = fraction > 1.0f ? 1.0f : (fraction < -1.0f ? -1.0f : fraction);
      return static_cast<Int16>(clamped * 1000.0f + (clamped >= 0.0f ? 0.5f : -0.5f));
  }

  // rev 0 is a blind tick.
  [[nodiscard]] bibowire::Decide decideFrom(const reactive::Outputs& o, Int32 modeMs, UInt32 rev)
  {
      bibowire::Decide d;
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

  // The device's identity as LIDAR_INFO carries it: the 32 hex digits of the
  // serial as 16 bytes, or zeros when it is not 32 hex digits.
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

  // Where the loaded set lives, beside the trim: BIBO_BUNDLES_FILE, else
  // ~/.config/bibo/bundles.txt.
  [[nodiscard]] Str bundleSetPath()
  {
      const Char* explicitPath = std::getenv("BIBO_BUNDLES_FILE");
      if(explicitPath != nullptr && explicitPath[0] != 0)
      {
          return Str(explicitPath);
      }
      const Char* home = std::getenv("HOME");
      if(home != nullptr && home[0] != 0)
      {
          return Str(home) + "/.config/bibo/bundles.txt";
      }
      return Str("bibo-bundles.txt");
  }

  // Written on every change, so the set a restart comes up with is the set the
  // board had. A failure is said once per attempt and changes nothing else.
  Void saveBundleSet(const Str& path, const chain::Chain& live)
  {
      const Str text = chain::formatSet(live);
      trimfile::makeParents(path);
      std::FILE* f = std::fopen(path.c_str(), "wb");
      if(f == nullptr)
      {
          std::printf("bundles: NOT saved to %s: %s\n", path.c_str(), std::strerror(errno));
          return;
      }
      const Bool wrote = std::fwrite(text.data(), 1u, text.size(), f) == text.size();
      const Bool closed = std::fclose(f) == 0;
      if(!wrote || !closed)
      {
          std::printf("bundles: NOT saved to %s: %s\n", path.c_str(), std::strerror(errno));
      }
  }

  // THE BOOT RULE: what was loaded when the board last changed its set is
  // loaded again. That is safe to do blind because a load can only ever reduce
  // authority and the car comes up disarmed, so no saved set can move it; and
  // a saved set that clamps everything is undone from the viewer, whose Unload
  // is refused in no state. A missing file is an empty set, not a fault.
  Void loadBundleSet(const Str& path, chain::Chain& live)
  {
      std::FILE* f = std::fopen(path.c_str(), "rb");
      if(f == nullptr)
      {
          if(errno != ENOENT)
          {
              std::printf("bundles: cannot read %s: %s\n", path.c_str(), std::strerror(errno));
          }
          return;
      }
      Str text;
      Array<Char, 4096> buf{};
      for(;;)
      {
          const Size n = std::fread(buf.data(), 1u, buf.size(), f);
          if(n == 0u)
          {
              break;
          }
          text.append(buf.data(), n);
      }
      std::fclose(f);
      Vec<Str> ids;
      Str unknown;
      if(!chain::parseSet(text, ids, unknown))
      {
          std::printf(
              "bundles: %s names %s, which this build does not have\n",
              path.c_str(),
              unknown.c_str()
          );
      }
      for(const Str& id : ids)
      {
          Str why;
          if(!live.load(chain::make(id.c_str()), why))
          {
              std::printf("bundles: %s: %s\n", id.c_str(), why.c_str());
          }
      }
      if(live.size() > 0)
      {
          std::printf("bundles: %zu loaded from %s\n", live.size(), path.c_str());
      }
  }

  // The bundles this board can run, published to the viewer.
  //
  // THE CATALOG IS THE LIST, not the manifests on disk. chain::catalog() is
  // exactly what the chain can load, so it is what a viewer may be offered;
  // building the list from manifests instead would offer a Load button for a
  // behaviour this build does not have, and hide the ones it does.
  //
  // READY IS DECIDED HERE, not by the bundle. What this board actually has is
  // something only this program knows, and a bundle must never be able to
  // declare itself loadable - that is the difference between a row a viewer
  // greys out with a reason and a car that starts something it cannot run.
  Void publishBundleList(const Options& opt, const chain::Chain& live)
  {
      // Bumped on every publish: a viewer keeps only frames of one generation,
      // so a new list replaces the old rather than interleaving with it, and
      // LOAD_BUNDLE quoting a stale generation is refused instead of acting on
      // whatever now sits at that index.
      static UInt32 generation = 0;
      chain::Present have;
      // main() has already returned if a lidar was asked for and did not open,
      // and the Pico is open unless this is a dry run.
      have.lidar = !opt.noLidar;
      have.pico = !opt.dry;
      // Whether the device is there now, by the path viewfeed opens it. A
      // stat, not an open, so it cannot disturb a capture; a camera that
      // falls off the bus later is reported by the detector's frame count.
      have.camera = viewfeed::cameraPresent();
      Vec<bibowire::Bundle> list;
      for(const chain::Entry& e : chain::catalog())
      {
          bibowire::Bundle b;
          b.id = e.id;
          b.name = e.name;
          b.about = e.about;
          b.needs = static_cast<UInt8>(
              e.needs | (e.drives ? bibowire::BUNDLE_MAY_DRIVE : 0u)
          );
          // readyFor reads only the needs bits, never BUNDLE_MAY_DRIVE.
          b.ready = chain::readyFor(e.needs, have) ? 1u : 0u;
          b.loaded = live.has(e.id) ? 1u : 0u;
          list.push_back(b);
      }
      ++generation;
      std::printf("bundles: %zu in the catalog\n", list.size());
      viewfeed::publishBundles(generation, std::move(list));
  }

  // What one load or unload did, so the tick reports once however many
  // requests arrived together.
  struct BundleOutcome
  {
      Bool acted = false;
      Bool ok = false;
      Str id;
      Str text;
  };

  // One request from a viewer, applied to the chain. THE INDEX IS INTO
  // chain::catalog(), because publishBundleList builds the list from it in
  // order - that is what makes an index mean the same bundle on two runs.
  // viewfeed has already checked the generation and the range against the list
  // it published; this range-checks again rather than trusting it, because the
  // catalog is this side's own truth.
  [[nodiscard]] BundleOutcome applyBundle(const viewfeed::BundleRequest& req, chain::Chain& live)
  {
      BundleOutcome o;
      const Vec<chain::Entry>& all = chain::catalog();
      if(static_cast<Size>(req.index) >= all.size())
      {
          return o;
      }
      const chain::Entry& e = all[req.index];
      o.acted = true;
      o.id = e.id;
      if(req.load)
      {
          Str why;
          o.ok = live.load(chain::make(e.id), why);
          o.text = o.ok ? (Str(e.name) + " loaded") : (Str(e.name) + ": " + why);
      }
      else
      {
          o.ok = live.unload(e.id);
          o.text = o.ok ? (Str(e.name) + " unloaded") : (Str(e.name) + " was not loaded");
      }
      std::printf("bundles: %s\n", o.text.c_str());
      viewfeed::publishEvent(
          o.ok ? bibowire::Severity::SEVERITY_INFO : bibowire::Severity::SEVERITY_WARN,
          bibowire::EVENT_CODE_BUNDLE,
          o.text
      );
      return o;
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
    // Line-buffered even into a pipe, so the status line reaches ssh or the
    // journal once a second.
    std::setvbuf(stdout, nullptr, _IOLBF, BUFSIZ);
    // Before anything is opened: a Ctrl-C during the lidar's open or the Pico's
    // must still end through the shutdown STOP below.
    std::signal(SIGINT, onInterrupt);
    std::signal(SIGTERM, onInterrupt);
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
    // The lidar first: a car that cannot see is never armed.
    blind = opt.noLidar;
    if(opt.noLidar)
    {
        std::printf("lidar: none opened (--no-lidar) - every scan is empty, the car is not seen\n");
    }
    else
    {
        std::printf("lidar %s: opening\n", opt.lidarPort.c_str());
        if(!lidar::open(opt.lidarPort))
        {
            std::printf("lidar %s: %s\n", opt.lidarPort.c_str(), lidar::reason().c_str());
            return 1;
        }
        std::printf("lidar device  %s\n", lidar::info().c_str());
        std::printf("lidar health  %s\n", lidar::health().c_str());
    }
    if(interrupted != 0)
    {
        std::printf("interrupted before the car was opened\n");
        lidar::close();
        return 0;
    }
    Link link;
    link.cfg.where = opt.picoPort;
    // The trim the viewer's Trim pane last saved.
    const Str trimPath = trimfile::defaultPath();
    trimfile::Store trim;
    // A save the viewers have not been told about yet.
    Bool trimUnreported = false;
    // The trim preview (see the manual tick): the aim, since when, and whether the
    // preview engaged the servo, so only the preview releases it.
    TimePoint previewAt;
    Bool previewing = false;
    Float32 previewSteer = 0.0f;
    Bool previewServoOn = false;
    {
        Str why;
        if(!trimfile::load(trimPath, trim, why))
        {
            std::printf(
                "trim: cannot read %s: %s - the Pico keeps its compiled values\n",
                trimPath.c_str(),
                why.c_str()
            );
        }
        else
        {
            std::printf(
                "trim: %zu setting(s) from %s, replayed whenever the Pico is opened\n",
                trimfile::lines(trim).size(),
                trimPath.c_str()
            );
        }
    }
    {
        const Str camPath = tags::defaultCameraPath();
        Str why;
        if(tags::loadIntrinsics(camPath, &camCal, why))
        {
            std::printf(
                "camera: calibrated from %s - fx %.1f fy %.1f at %ux%u, tag %.3f m\n",
                camPath.c_str(),
                camCal.fx,
                camCal.fy,
                static_cast<unsigned>(camCal.width),
                static_cast<unsigned>(camCal.height),
                camCal.tagM
            );
        }
        else
        {
            std::printf("camera: not calibrated (%s) - tags carry no range\n", why.c_str());
        }
    }
    if(opt.dry)
    {
        std::printf("pico: dry run - decisions are printed, nothing is sent\n");
    }
    else if(!openPico(opt, trim, link))
    {
        lidar::close();
        return 1;
    }
    else
    {
        const CharSeq armed = !opt.arm ? ", not armed"
            : (opt.manual ? ", ESC ARM sent" : ", ESC ARM and SERVO ON sent");
        std::printf("pico %s: open, STOP sent%s\n", opt.picoPort.c_str(), armed);
    }
    // A Ctrl-C during the open skips the motor; the loop does not run and the
    // shutdown sends STOP.
    if(interrupted == 0)
    {
        if(!opt.noLidar && !lidar::motorOn())
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
    // The behaviours loaded right now. Empty until a viewer loads one, and it
    // is what BUNDLE::loaded reports, so the list a viewer sees is the chain
    // this program actually holds rather than a second copy of the answer.
    chain::Chain live;
    const Str bundlePath = bundleSetPath();
    loadBundleSet(bundlePath, live);
    // --manual is the promise that a viewer drives, so wasd is loaded whether
    // or not the file says so: it is the one bundle the flag means.
    if(opt.manual && !live.has(chain::ID_WASD))
    {
        Str why;
        if(!live.load(chain::makeWasd(), why))
        {
            std::printf("bundles: wasd: %s\n", why.c_str());
        }
    }
    // The viewers: never fatal, the car does not wait for them.
    Viewer viewer;
    if(interrupted == 0)
    {
        viewfeed::Policy wire;
        wire.boardName = "bibobox";
        wire.boardBuild = BOARD_BUILD;
        // Per process: a viewer that sees a new one forgets what it knew.
        wire.bootId = static_cast<UInt32>(WallClock::now().time_since_epoch().count());
        // b0 canDrive, b1 hasLidar, b2 hasPico, b4 canLoadBundles - the last so
        // a viewer knows to show a master window at all, and an older board
        // simply does not offer one.
        wire.capabilities = static_cast<UInt8>(
            (opt.dry ? 0u : 1u) | (opt.noLidar ? 0u : 2u) | (opt.dry ? 0u : 4u) | 16u
        );
        viewer.wire = viewfeed::start(bibowire::PORT, wire);
        if(!viewer.wire)
        {
            std::printf("viewfeed: not serving - driving without a viewer\n");
        }
        else
        {
            // Identity and trim before the first revolution, so a viewer that
            // connects during spin-up knows what it is watching.
            if(!opt.noLidar)
            {
                viewfeed::publishLidarInfo(lidarInfoFrom(lidar::device()));
            }
            viewfeed::publishTrim(trimfile::report(trim));
            publishBundleList(opt, live);
        }
    }
    // After the feed is up, so the detector's camera subscription is seen.
    syncTags(live);
    syncOdometry(live);
    reactive::State   state;
    reactive::Outputs out;
    reactive::Status  status = reactive::Status::STATUS_BLIND;
    Vec<reactive::Ray> rays;
    Vec<UInt8>         quality;
    Vec<Str>           lines;
    Replies            replies;
    // What the next tick's wait re-sends.
    Hold hold;
    // MANUAL's last commanded steering, held (not centred) while the deadman is
    // SOFT, and updated only while it is LIVE.
    Int16 heldSteerMilli = 0;
    UInt64 revolutions = 0;
    UInt64 timeouts = 0;
    UInt64 windowRevs = 0;   // revolutions since the last status line
    // Silent until heard, so the first change printed is the Pico speaking.
    Bool boardSilent = true;
    // Whether the Pico has been sent ESC ARM for a viewer's ARM, in MANUAL. Anything
    // that disarms it another way (STOP, a reopened link) clears this.
    Bool armSent = false;
    // Set when the link reopens: a viewer's ARM from before the loss must be seen
    // to fall before one counts again.
    Bool staleArm = false;
    // Consecutive timeouts since the last revolution, counted only after the first
    // one: spin-up times out several times on every run.
    Int32 blindRun = 0;
    Bool  lidarLost = false;     // a blind run reached LIDAR_LOST_TIMEOUTS; sticky
    const TimePoint start = monoNow();
    TimePoint lastTick = start;
    TimePoint lastStatus = start;
    Bool haveTick = false;
    // A viewer's ESTOP or DISARM outside MANUAL: STOP was sent and the run ends, as a
    // car program's does, so nothing re-arms the car on a later tick or reconnect.
    Bool estopped = false;
    // The chain's newest refusal, said once per distinct sentence, not per tick.
    Str chainRefusalSaid;
    // ODOM goes out once per OK drive reply that carried a count, never twice
    // for the same one: a silent Pico is then a silent feed, which the viewer
    // shows as gone, and not a count that looks held still.
    UInt64 odomSaidOk = 0;
    UInt8 odomSeq = 0;
    const odom::Config odomCfg = odom::defaults();
    while(interrupted == 0 && !estopped && (opt.seconds < 0.0 || elapsedS(start) < opt.seconds))
    {
        // Empty on a timeout, and handed to step() anyway: STATUS_BLIND is a stop.
        const Bool got = grabHeld(rays, &quality, hold, link);
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
        // The viewers before the car: publish() only queues, while the serial port
        // may stall. Timed for the exit summary.
        const TimePoint before = monoNow();
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
        // The Pico's silence, judged before deciding and announced on change. No
        // descriptor (-1) and a port not yet heard both count as silent.
        if(!opt.dry)
        {
            const Int32 silent = carlink::silentForMs();
            const Bool nowSilent = !link.heard || silent < 0 || silent > link.cfg.silenceMs;
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
        const carrules::Board& car = replies.pico;
        Snapshot snap;
        snap.monoUs = static_cast<UInt64>(elapsedS(start) * 1000000.0);
        snap.upS = static_cast<UInt32>(elapsedS(start));
        const Float64 windowS = elapsedS(lastStatus);
        snap.revPerS = windowS > 0.0 ? static_cast<Float64>(windowRevs) / windowS : 0.0;
        snap.timeouts = timeouts;
        snap.revolutions = revolutions;
        snap.lidarHealth = lidar::device().health;
        snap.lidarSpinning = lidar::isSpinning();
        snap.dry = opt.dry;
        snap.pilotMode = pilotModeOf(opt);
        snap.picoOpen = !opt.dry && !link.lost;
        snap.picoHeard = link.heard;
        snap.picoSilentMs = opt.dry ? -1 : carlink::silentForMs();
        snap.replyOk = replies.ok;
        snap.replyErr = car.errors;
        snap.picoArmed = car.armed;
        snap.pico = picoPhrase(snap);
        // Every tick; viewfeed sends it at its own rate.
        viewfeed::publishBoard(boardFrom(snap));
        // What the car is doing, from the Pico's replies; throttleMilli is what
        // was decided. viewfeed's refusal to re-trim an armed car reads armed from
        // here. Not reported reads as not armed.
        viewfeed::Applied ap;
        ap.steerNowMilli = static_cast<Int16>(replies.steerNowMilli);
        ap.throttleMilli = static_cast<Int16>(out.throttle * 1000.0f);
        ap.escUs = car.escUs < 0 ? bibowire::ESC_ABSENT : static_cast<UInt16>(car.escUs);
        ap.armed = car.armed > 0 ? 1u : 0u;
        ap.pilotMode = pilotModeOf(opt);
        ap.picoSilentMs = snap.picoSilentMs < 0
            ? bibowire::PICO_SILENT_ABSENT
            : static_cast<UInt32>(snap.picoSilentMs);
        viewfeed::applied(ap);
        if(car.ticks != -1 && replies.ok != odomSaidOk)
        {
            odomSaidOk = replies.ok;
            ++odomSeq;
            bibowire::Odom od;
            od.tMonoUs = snap.monoUs;
            od.ticks = car.ticks;
            od.ticksPerS = static_cast<Int16>(std::clamp(car.ticksPerS, -32768, 32767));
            od.skips = static_cast<UInt8>(std::clamp(car.hallSkips, 0, 255));
            od.invalid = static_cast<UInt8>(std::clamp(car.hallInvalid, 0, 255));
            od.seq = odomSeq;
            viewfeed::publishOdom(od);
            // The same reply steps the reckoning, with the steering the wheels
            // ARE at, so a pose is published for exactly the counts it used.
            if(odomOn)
            {
                odom::step(odomState, car.ticks, replies.steerNowMilli, odomCfg);
                viewfeed::publishPose(odom::toPose(odomState, snap.monoUs));
            }
        }
        // LOADS AND UNLOADS TAKE EFFECT BETWEEN PASSES, never inside one, so
        // no scan is half judged by two different sets of behaviours.
        {
            viewfeed::BundleRequest req;
            Bool changed = false;
            BundleOutcome last;
            while(viewfeed::bundleRequest(&req))
            {
                const BundleOutcome did = applyBundle(req, live);
                if(did.acted)
                {
                    last = did;
                    changed = changed || did.ok;
                }
            }
            if(last.acted)
            {
                // Only a change moves the list on: a refusal left every
                // `loaded` flag as the viewer already has it.
                if(changed)
                {
                    publishBundleList(opt, live);
                    saveBundleSet(bundlePath, live);
                    syncTags(live);
                    syncOdometry(live);
                }
                bibowire::BundleState bs;
                bs.tMonoUs = snap.monoUs;
                bs.loadedCount = static_cast<UInt32>(live.size());
                bs.anyLoaded = live.size() > 0 ? 1u : 0u;
                bs.lastKind = last.ok
                    ? bibowire::BUNDLE_EXIT_OK
                    : bibowire::BUNDLE_EXIT_REFUSED;
                bs.id = last.id;
                bs.text = last.text;
                viewfeed::publishBundleState(bs);
            }
        }
        // Who writes steer and throttle this tick. MANUAL follows section 6's
        // deadman chain:
        //   ESTOP / DEAD  STOP (neutral, disarm, release); only a new COMMAND ARM
        //                 recovers.
        //   SOFT          throttle 0, steering held at the last commanded value.
        //   LIVE          obeyed; control() has already zeroed the throttle on an
        //                 epoch or mode disagreement.
        // Something is sent every tick in every state, so the Pico's watchdog
        // fires only when this program has stopped.
        // The viewer's control, read once, so the chain and the MANUAL rules
        // below judge the same command.
        const viewfeed::Drive dm = viewfeed::drive();
        bibowire::Control cmd;
        const Bool haveCmd = viewfeed::control(&cmd);
        const Bool manualMode =
            pilotModeOf(opt) == static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_MANUAL);
        // THE CHAIN RUNS EVERY TICK, whatever drives. Its proposal, when it
        // makes one, is what the car is told; without one the mode's own
        // source drives under the chain's ceiling, so a loaded clamp binds the
        // autonomy and a hand on the keys alike (docs/bundles.md section 2).
        const bibo::Scan chainScan = carrules::toScan(rays, opt.forwardDeg, rev);
        // The detector's newest frame, for follow; absent when nothing runs.
        bibowire::Tags seenTags;
        Int32 tagsAge = 0;
        const Bool haveTags = tags::latest(&seenTags, &tagsAge);
        chain::Pass pass;
        pass.tags = haveTags ? &seenTags : nullptr;
        pass.tagsAgeMs = tagsAge;
        pass.scan = &chainScan;
        pass.dtMs = dtMs;
        pass.nowMs = static_cast<Int64>(elapsedMs(start));
        // A hand on the wheel only while the deadman is LIVE: a SOFT or DEAD
        // command is one the rules below already refuse.
        pass.haveHolder = manualMode && dm.haveHolder && haveCmd && dm.deadman == 0u;
        pass.manualSteer = static_cast<Float32>(cmd.steerMilli) / 1000.0f;
        pass.manualThrottle = static_cast<Float32>(cmd.throttleMilli) / 1000.0f;
        const chain::Outcome chosen = live.run(pass);
        if(chosen.refused > 0 && chosen.lastRefusal != chainRefusalSaid)
        {
            chainRefusalSaid = chosen.lastRefusal;
            std::printf("bundles: %s\n", chosen.lastRefusal.c_str());
            viewfeed::publishEvent(
                bibowire::Severity::SEVERITY_WARN,
                bibowire::EVENT_CODE_BUNDLE,
                chosen.lastRefusal
            );
        }
        const Int32 ceilingMilli = static_cast<Int32>(chosen.intent.ceiling * 1000.0f);
        Str steerLine;
        Str escLine;
        // SERVO ON or OFF on an arm edge, sent before the STEER: a released
        // steering pin ignores STEER.
        Str servoLine;
        // What was actually commanded, for the status line; modeWord empty means
        // describe() the autonomy's decision.
        Int32 sentSteerMilli = static_cast<Int32>(out.steer * 1000.0f);
        Int32 sentThrottleMilli = static_cast<Int32>(out.throttle * 1000.0f);
        Str modeWord;
        if(manualMode)
        {
            if(!dm.armed)
            {
                staleArm = false;
            }
            const Bool viewerArmed = dm.armed && !staleArm;
            // A deadman byte out of range reads as DEAD.
            const bibowire::deadman::State ds = dm.deadman <= 3u
                ? static_cast<bibowire::deadman::State>(dm.deadman)
                : bibowire::deadman::State::STATE_DEAD;
            modeWord = Str("manual ") + bibowire::deadman::stateName(ds);
            if(!dm.haveHolder)
            {
                // With nobody holding the slot the deadman does not apply.
                modeWord = "manual  nobody holding";
            }
            modeWord += car.armed > 0 ? " armed" : " disarmed";
            if(car.servoOn > 0)
            {
                modeWord += " steering on " + std::to_string(replies.servoUs) + "us";
            }
            else
            {
                modeWord += " steering off";
            }
            if(dm.estopLatched || dm.deadman >= 2u)
            {
                heldSteerMilli = 0;
                escLine = proto::stop();
                armSent = false;
                sentSteerMilli = 0;
                sentThrottleMilli = 0;
            }
            else if(!dm.haveHolder || !haveCmd)
            {
                // Nobody driving is not a fault: neutral, and the arm state left
                // alone. A STOP here would disarm --arm before a viewer arrives.
                // A driver who left took their ARM with them (the epoch moved), so
                // the Pico is told DISARM and SERVO OFF once.
                heldSteerMilli = 0;
                steerLine = proto::steer(0.0f);
                escLine = proto::command("ESC", armSent ? "DISARM" : "NEUTRAL");
                if(armSent)
                {
                    servoLine = proto::command("SERVO", "OFF");
                }
                armSent = false;
                sentSteerMilli = 0;
                sentThrottleMilli = 0;
            }
            else
            {
                // FORWARD THROTTLE GOES THROUGH THE CHAIN: its proposal when it
                // made one, else the viewer's own under the chain's ceiling, so
                // a loaded clamp binds hand driving too. REVERSE DOES NOT: the
                // stop clamp looks AHEAD, and backing away from what it sees is
                // exactly the move it must never prevent. So a reverse asked for
                // is the human's alone, steering included.
                const Bool backingAsked = cmd.throttleMilli < 0;
                const Int32 forwardMilli = chosen.drive
                    ? static_cast<Int32>(milliOf(chosen.throttle))
                    : std::min(static_cast<Int32>(cmd.throttleMilli), ceilingMilli);
                if(dm.deadman == 0u)
                {
                    heldSteerMilli = chosen.drive && !backingAsked
                        ? milliOf(chosen.steer)
                        : cmd.steerMilli;
                }
                steerLine = proto::steer(static_cast<Float32>(heldSteerMilli) / 1000.0f);
                // Throttle only while LIVE, while a viewer's ARM stands and the Pico
                // was sent it, and while the Pico is speaking. --arm is not consulted.
                const Bool mayPush = dm.deadman == 0u && viewerArmed && armSent && !boardSilent;
                if(viewerArmed != armSent)
                {
                    // The edge takes this tick's ESC line: ESC ARM resets the Pico's
                    // throttle target, so throttle starts next tick. The steering is
                    // engaged and released with the arm.
                    escLine = proto::command("ESC", viewerArmed ? "ARM" : "DISARM");
                    servoLine = proto::command("SERVO", viewerArmed ? "ON" : "OFF");
                    armSent = viewerArmed;
                }
                else
                {
                    const Float32 wanted = static_cast<Float32>(forwardMilli) / 1000.0f;
                    const Int32 neutralUs = static_cast<Int32>(bibowire::ESC_NEUTRAL_US);
                    // S (negative throttle) sends the reverse limit itself, not a
                    // fraction of it; the ESC chooses brake or reverse as it does for
                    // the transmitter. No limit below neutral means reverse is off.
                    const Bool reverseOn = car.escRevUs > 0 && car.escRevUs < neutralUs;
                    const Bool backing = cmd.throttleMilli < 0 && reverseOn;
                    const Int32 backUs = backing ? car.escRevUs : neutralUs;
                    // The idle test (bibowire::BUTTON_IDLE_TEST) sends the Pico's idle.
                    const Bool idleTest = (cmd.buttons & bibowire::BUTTON_IDLE_TEST) != 0u;
                    const Bool idleKnown = car.escMinUs > neutralUs;
                    if(mayPush && idleTest)
                    {
                        escLine = idleKnown
                            ? proto::escUs(car.escMinUs)
                            : proto::command("ESC", "NEUTRAL");
                    }
                    else if(mayPush && forwardMilli > 0)
                    {
                        const Int32 us = carrules::forwardPulse(wanted, car.escMinUs, car.escMaxUs);
                        escLine = proto::escUs(us);
                    }
                    else if(mayPush && backUs < neutralUs)
                    {
                        escLine = proto::escUs(backUs);
                    }
                    else
                    {
                        escLine = proto::command("ESC", "NEUTRAL");
                    }
                }
                sentSteerMilli = heldSteerMilli;
                sentThrottleMilli = !mayPush ? 0
                    : (backingAsked ? static_cast<Int32>(cmd.throttleMilli) : forwardMilli);
            }
            // THE TRIM PREVIEW: while nobody has armed, a steering trim change points
            // the wheels at what is being set, with the servo engaged, and releases
            // them TRIM_PREVIEW_MS after the last change. Never while armed or
            // stopped: that servo belongs to the driver or to the STOP.
            constexpr Float64 TRIM_PREVIEW_MS = 2500.0;
            if(!armSent && !dm.estopLatched && dm.deadman < 2u)
            {
                if(previewing && elapsedMs(previewAt) <= TRIM_PREVIEW_MS)
                {
                    if(!previewServoOn)
                    {
                        servoLine = proto::command("SERVO", "ON");
                        previewServoOn = true;
                    }
                    steerLine = proto::steer(previewSteer);
                }
                else if(previewServoOn)
                {
                    servoLine = proto::command("SERVO", "OFF");
                    steerLine = proto::steer(0.0f);
                    previewServoOn = false;
                    previewing = false;
                }
            }
            else
            {
                previewServoOn = false;
                previewing = false;
            }
        }
        else if(viewfeed::drive().estopLatched)
        {
            // Beside a STOP there is no STEER, as in MANUAL.
            escLine = proto::stop();
            estopped = true;
            sentSteerMilli = 0;
            sentThrottleMilli = 0;
            std::printf("viewer ESTOP - STOP sent, ending\n");
        }
        else
        {
            // The chain's proposal replaces the autonomy's; without one the
            // autonomy drives under the chain's ceiling.
            reactive::Outputs d = out;
            reactive::Status s = status;
            if(chosen.drive)
            {
                d.steer = chosen.steer;
                d.throttle = chosen.throttle;
                d.stop = chosen.throttle <= 0.0f;
                s = reactive::Status::STATUS_OK;
            }
            else
            {
                d.throttle = std::min(d.throttle, chosen.intent.ceiling);
            }
            steerLine = proto::steer(d.steer);
            escLine = escLineFor(s, d, boardSilent, opt.arm);
            sentSteerMilli = static_cast<Int32>(d.steer * 1000.0f);
            sentThrottleMilli = static_cast<Int32>(d.throttle * 1000.0f);
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
            // An empty line is not sent: beside a STOP there is no STEER.
            if(!servoLine.empty())
            {
                sendLine(servoLine, link);
            }
            if(!steerLine.empty())
            {
                sendLine(steerLine, link);
            }
            if(!escLine.empty())
            {
                sendLine(escLine, link);
            }
            // Only lines sent this tick are held. SERVO is an edge and never
            // re-sent, or it would re-engage a servo a STOP just released.
            if(!steerLine.empty())
            {
                hold.steer = steerLine;
            }
            if(!escLine.empty())
            {
                hold.esc = escLine;
            }
            // The operator's trim, before the replies are read, so its OK or ERR is
            // counted this tick. A dry run has no port, and viewfeed refuses tuning
            // without a Pico.
            Bool tuneDrained = false;
            for(Int32 sent = 0; sent < TUNE_PER_TICK; ++sent)
            {
                viewfeed::Tune t;
                if(!viewfeed::tune(&t))
                {
                    tuneDrained = true;
                    break;
                }
                const Str line = tuneLine(t);
                if(line.empty())
                {
                    continue;
                }
                sendLine(line, link);
                // The preview's aim: the centre for a trim; for limits, the end
                // that differs from what the Pico last reported.
                if(t.verb == bibowire::Verb::VERB_SET_SERVO_TRIM)
                {
                    previewSteer = 0.0f;
                    previewAt = monoNow();
                    previewing = true;
                }
                else if(t.verb == bibowire::Verb::VERB_SET_SERVO_LIMITS)
                {
                    if(static_cast<Int32>(t.arg1) != replies.servoMinUs)
                    {
                        previewSteer = -1.0f;
                    }
                    else if(static_cast<Int32>(t.arg2) != replies.servoMaxUs)
                    {
                        previewSteer = 1.0f;
                    }
                    previewAt = monoNow();
                    previewing = true;
                }
                // Saved when it changed something.
                if(trimfile::remember(trim, line))
                {
                    Str why;
                    if(trimfile::save(trimPath, trim, why))
                    {
                        std::printf("trim: saved \"%s\" to %s\n", line.c_str(), trimPath.c_str());
                        trimUnreported = true;
                    }
                    else
                    {
                        std::printf(
                            "trim: NOT saved \"%s\" to %s: %s\n",
                            line.c_str(),
                            trimPath.c_str(),
                            why.c_str()
                        );
                    }
                }
            }
            // The viewers hear the trim once the queue has drained, not per line,
            // so their sliders never pass through partial sets.
            if(trimUnreported && tuneDrained)
            {
                const Str report = trimfile::report(trim);
                viewfeed::publishTrim(report);
                std::printf("trim: told the viewers the board now has \"%s\"\n", report.c_str());
                trimUnreported = false;
            }
            if(!readReplies(lines, replies, link) && !link.lost)
            {
                link.lost = true;
                std::printf("pico link lost: %s\n", carlink::detail().c_str());
            }
            if(link.lost)
            {
                holdNeutral(hold);
            }
        }
        if(elapsedMs(lastStatus) >= STATUS_EVERY_MS)
        {
            const Str what = modeWord.empty() ? describe(status, out, got) : modeWord;
            // The loaded bundles, in chain order, so the log says who decided.
            Str chainWord;
            for(Size i = 0; i < live.size(); ++i)
            {
                chainWord += i == 0 ? "  [" : "+";
                chainWord += live.at(i)->name();
            }
            if(!chainWord.empty())
            {
                chainWord += "]";
            }
            if(tags::running())
            {
                const tags::Stats ts = tags::stats();
                chainWord += "  apriltag " + std::to_string(ts.frames) + " frames "
                             + std::to_string(ts.seen) + " tags, last " + std::to_string(
                                 ts.lastCount
                             )
                             + " in " + std::to_string(ts.lastDetectUs / 1000u) + " ms";
                if(ts.undecodable > 0u)
                {
                    chainWord += ", " + std::to_string(ts.undecodable) + " undecodable";
                }
            }
            std::printf(
                "%6.1f s  %s  steer %+.2f  thr %.2f  esc %d us  %5.1f rev/s  timeouts %llu  %s%s%s\n",
                elapsedS(start),
                what.c_str(),
                static_cast<Float64>(sentSteerMilli) / 1000.0,
                static_cast<Float64>(sentThrottleMilli) / 1000.0,
                static_cast<int>(replies.pico.escUs),   // the pulse the Pico reports
                snap.revPerS,
                static_cast<unsigned long long>(snap.timeouts),
                snap.pico.c_str(),
                chainWord.c_str(),
                // The Pico's watchdog fired while this program believed it was sending.
                replies.pico.stale > 0 ? "  <<< BOARD WATCHDOG STALE" : ""
            );
            windowRevs = 0;
            lastStatus = now;
            // A lost link is retried once a second, and only after this tick's BOARD
            // reported it down, so viewfeed has moved the epoch before it is back.
            if(!opt.dry && link.lost && !snap.picoOpen)
            {
                if(openPico(opt, trim, link))
                {
                    // Reopened with STOP first, which disarmed and released the
                    // steering. The keepalive holds neutral until a tick decides, and
                    // a viewer's ARM from before the loss must fall before it counts.
                    holdNeutral(hold);
                    armSent = false;
                    staleArm = true;
                    previewServoOn = false;
                    std::printf("pico %s: link back\n", opt.picoPort.c_str());
                }
            }
        }
    }
    // Shutdown: the car first, then the lidar, then the ports.
    if(!opt.dry)
    {
        sendLine(proto::stop(), link);
        sleepMs(50);   // long enough for the reply to the STOP to land
        if(!readReplies(lines, replies, link) && !link.lost)
        {
            link.lost = true;
            std::printf("pico link lost at STOP: %s\n", carlink::detail().c_str());
        }
    }
    const Float64 ran = elapsedS(start);
    std::printf(
        "%s: %llu revolutions, %llu timeouts in %.1f s (%.2f rev/s over the run)\n",
        interrupted != 0 ? "interrupted" : (estopped ? "estopped" : "done"),
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
            static_cast<unsigned long long>(replies.pico.errors),
            static_cast<unsigned long long>(replies.other)
        );
    }
    if(viewer.costTicks > 0)
    {
        std::printf(
            "viewfeed: publish cost per tick avg %.0f us, max %.0f us\n",
            viewer.costSumUs / static_cast<Float64>(viewer.costTicks),
            viewer.costMaxUs
        );
    }
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
    if(!opt.noLidar && !lidar::motorOff())
    {
        std::printf("lidar motor off: %s\n", lidar::reason().c_str());
    }
    lidar::close();
    carlink::close();
    // The detector before the feed it reads frames from.
    tags::stop();
    // The viewers last, told the board is shutting down.
    viewfeed::stop();
    // A signal is a request carried out: 0. Otherwise a run that measured nothing,
    // or lost the lidar partway, must not look like success.
    if(interrupted != 0)
    {
        return 0;
    }
    return (revolutions > 0 || opt.noLidar) && !lidarLost ? 0 : 1;
}
