// LidarSource over the network, against fake_scanfeed.py.
//
//   test_lidarnet.exe PORT LOGFILE PILOTPORT PILOTLOG
//
// build_lidarnet_test.bat starts the fake on PORT with the session list this
// file expects - junk, abrupt:5, err:3 - and hands over the path of its log,
// which section 5 reads to prove the hub said MOTOR 0, MOTOR 1 and QUIT.
// PORT+1 must have nothing listening on it; that is the refused-connect case.
//
// A second fake on PILOTPORT is the PILOT's feed (--drive --pilot, sessions
// normal, pilotquit:5): D lines after the frames, and a MOTOR 0 it refuses.
// Sections 6 to 8 are about that one, and its log proves the hub sent MOTOR 0
// exactly once.
//
// No hardware, no SDK call reached: the serial half of lidar_source.cxx is
// linked in because it is the same object, and never started.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "lidar_source.hxx"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{

  Int32 checks = 0;
  Int32 failures = 0;

  Void check(Bool ok, const Char* what)
  {
      ++checks;
      std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
      if(!ok)
      {
          ++failures;
      }
  }

  Void checkContains(const Str& text, const Char* needle, const Char* what)
  {
      ++checks;
      const Bool ok = text.find(needle) != Str::npos;
      std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
      if(!ok)
      {
          ++failures;
          std::printf("        got  \"%s\"\n        want \"%s\" in it\n", text.c_str(), needle);
      }
  }

  const Char* stateName(LidarState s)
  {
      switch(s)
      {
          case LidarState::LIDAR_STATE_IDLE:       return "Idle";
          case LidarState::LIDAR_STATE_CONNECTING: return "Connecting";
          case LidarState::LIDAR_STATE_SCANNING:   return "Scanning";
          case LidarState::LIDAR_STATE_UNPLUGGED:  return "Unplugged";
          case LidarState::LIDAR_STATE_ERROR:      return "Error";
      }
      return "?";
  }

  // Generous, because the fake ticks at 10 Hz and the source at 2 Hz: nothing
  // here should take more than a second, and five is a machine under load.
  constexpr Int32 DEADLINE_MS = 5000;

  // Polls until state() is `want`, or the deadline passes. Reports either way.
  Bool waitState(LidarSource& src, LidarState want, const Char* what)
  {
      const TimePoint began = monoNow();
      while(src.state() != want && elapsedMs(began) < DEADLINE_MS)
      {
          sleepMs(10);
      }
      const Bool ok = src.state() == want;
      Array<Char, 160> line;
      std::snprintf(
          line.data(),
          line.size(),
          "%s -> %s after %.0f ms%s%s",
          what,
          stateName(src.state()),
          elapsedMs(began),
          src.error().empty() ? "" : ": ",
          src.error().c_str()
      );
      check(ok, line.data());
      return ok;
  }

  // Polls until motorEnabled() reads `want`.
  Bool waitMotor(LidarSource& src, Bool want, const Char* what)
  {
      const TimePoint began = monoNow();
      while(src.motorEnabled() != want && elapsedMs(began) < DEADLINE_MS)
      {
          sleepMs(10);
      }
      check(src.motorEnabled() == want, what);
      return src.motorEnabled() == want;
  }

  // Frames poll() hands over in `ms`. The last one is left in `last`.
  Int32 countFrames(LidarSource& src, Int32 ms, LidarFrame& last)
  {
      Int32 n = 0;
      const TimePoint began = monoNow();
      while(elapsedMs(began) < ms)
      {
          if(src.poll(last))
          {
              ++n;
          }
          sleepMs(5);
      }
      return n;
  }

  // The fake is started by the .bat a moment before this runs, and python takes
  // a while to get to listen(). A refused connect on localhost is instant, so
  // knocking every quarter second until the deadline costs nothing and makes
  // the ordering deterministic. The first accepted connection IS session one.
  Bool connectWhenListening(LidarSource& src, const Str& target)
  {
      const TimePoint began = monoNow();
      while(elapsedMs(began) < DEADLINE_MS * 2)
      {
          src.start(target, 0);
          while(src.state() == LidarState::LIDAR_STATE_CONNECTING)
          {
              sleepMs(10);
          }
          if(src.state() != LidarState::LIDAR_STATE_ERROR)
          {
              return true;
          }
          src.stop();
          sleepMs(250);
      }
      return false;
  }

  // The tag the fake writes into every revolution and the decision that
  // follows it: the frame index mod 100, as 10.0xx Hz on the F line and as the
  // hit count on the D line. Recovered from the rate here.
  Int32 frameTag(const LidarFrame& f)
  {
      return static_cast<Int32>((f.hz - 10.0f) * 1000.0f + 0.5f);
  }

  // Polls until a decision arrives, or the deadline passes.
  Bool waitDrive(LidarSource& src, LidarDrive& out, Int32 ms)
  {
      const TimePoint began = monoNow();
      while(elapsedMs(began) < ms)
      {
          if(src.pollDrive(out))
          {
              return true;
          }
          sleepMs(5);
      }
      return false;
  }

  Int32 countOf(const Str& text, const Char* needle)
  {
      Int32 n = 0;
      const Size len = std::strlen(needle);
      for(Size at = text.find(needle); at != Str::npos; at = text.find(needle, at + len))
      {
          ++n;
      }
      return n;
  }

  Str readFile(const Str& path)
  {
      Str text;
      FILE* f = std::fopen(path.c_str(), "rb");
      if(f == nullptr)
      {
          return text;
      }
      Array<Char, 4096> buf;
      Size n = 0;
      while((n = std::fread(buf.data(), 1, buf.size(), f)) > 0)
      {
          text.append(buf.data(), n);
      }
      std::fclose(f);
      return text;
  }

}

Int32 main(Int32 argc, Char** argv)
{
    if(argc < 5)
    {
        std::printf("usage: test_lidarnet.exe PORT LOGFILE PILOTPORT PILOTLOG\n");
        return 1;
    }
    const Int32 port = std::atoi(argv[1]);
    const Str   logPath = argv[2];
    const Int32 pilotPort = std::atoi(argv[3]);
    const Str   pilotLogPath = argv[4];
    const Str   target = "127.0.0.1:" + std::to_string(port);
    const Str   nowhere = "127.0.0.1:" + std::to_string(port + 1);
    const Str   pilot = "127.0.0.1:" + std::to_string(pilotPort);

    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("=== LidarSource over the scan feed ===\n");
    std::printf(
        "feed at %s, nothing at %s, the pilot at %s\n",
        target.c_str(),
        nowhere.c_str(),
        pilot.c_str()
    );

    // ---- 1. nothing listening ----------------------------------------------
    std::printf("\n-- 1. nothing listening --\n");
    {
        LidarSource src;
        check(LidarSource::isFeedTarget(nowhere), "host:port is a feed target");
        check(!LidarSource::isFeedTarget("COM7"), "COM7 is not");

        src.start(nowhere, 0);
        check(src.state() == LidarState::LIDAR_STATE_CONNECTING, "start() -> Connecting at once");
        check(src.port() == nowhere, "port() is the target string");

        waitState(src, LidarState::LIDAR_STATE_ERROR, "refused connect");
        checkContains(
            src.error(),
            ("no scan feed at " + nowhere).c_str(),
            "message names the target"
        );
        checkContains(
            src.error(),
            "is scanfeed running on the board",
            "message says what to check"
        );
        check(!src.reachedDevice(), "never reached the device");
        src.stop();
        check(src.state() == LidarState::LIDAR_STATE_ERROR, "stop() keeps the error on screen");
    }

    // ---- 2. a room, a bad line, and the motor ------------------------------
    std::printf("\n-- 2. the room (session: junk) --\n");
    {
        LidarSource src;
        check(connectWhenListening(src, target), "connected once the fake was listening");
        check(src.port() == target, "port() is the target string");

        if(waitState(src, LidarState::LIDAR_STATE_SCANNING, "first frame"))
        {
            check(src.reachedDevice(), "reached the device");

            LidarFrame f;
            const Int32 n = countFrames(src, 1500, f);
            Array<Char, 96> line;
            std::snprintf(line.data(), line.size(), "%d frames in 1.5 s at 10 Hz (want >= 8)", n);
            check(n >= 8, line.data());

            std::snprintf(line.data(), line.size(), "%zu points (want 360)", f.points.size());
            check(f.points.size() == 360, line.data());
            std::snprintf(line.data(), line.size(), "validCount %d (want 360)", f.validCount);
            check(f.validCount == 360, line.data());
            std::snprintf(
                line.data(),
                line.size(),
                "maxDistMm %.0f (the room's corner, want 2400..2600)",
                f.maxDistMm
            );
            check(f.maxDistMm >= 2400.0f && f.maxDistMm <= 2600.0f, line.data());
            std::snprintf(line.data(), line.size(), "hz %.2f (want 9.5..10.5)", f.hz);
            check(f.hz >= 9.5f && f.hz <= 10.5f, line.data());
            check(!f.points.empty() && f.points[0].quality == 47, "quality 47 as sent");
            check(!f.points.empty() && f.points[0].angleDeg == 0.0f, "first sample at 0 deg");
            check(f.points.size() > 90 && f.points[90].angleDeg == 90.0f, "sample 90 at 90 deg");
            check(f.points.size() > 90 && f.points[90].distMm == 1500.0f, "1.5 m to the side wall");
            check(f.points[0].distMm == 2000.0f, "2 m to the end wall");

            const LidarDeviceInfo info = src.info();
            check(info.model == 65, "INFO model 65");
            check(info.fwMajor == 1 && info.fwMinor == 2, "INFO firmware 1.2");
            check(info.hwRev == 24, "INFO hardware 24");
            check(info.serial == "ABCDEF0123456789ABCDEF0123456789", "INFO serial");
            check(info.health == 0, "HEALTH 0");

            const LidarScanInfo si = src.scanInfo();
            check(
                si.mode.empty() && si.modeId < 0 && si.usPerSample == 0.0f,
                "scanInfo() left empty"
            );

            const LidarStats st = src.stats();
            std::snprintf(
                line.data(),
                line.size(),
                "one bad line counted under timeouts (%u), frames still flowing (%llu)",
                st.timeouts,
                static_cast<unsigned long long>(st.frames)
            );
            check(st.timeouts == 1 && st.frames > 3, line.data());
            check(st.points == st.frames * 360, "points counter is frames x 360");
            check(st.uptimeS > 1.0, "uptime runs from the first frame");

            // Motor off: the fake echoes MOTOR 0 and stops sending frames.
            src.setMotorEnabled(false);
            waitMotor(src, false, "MOTOR 0 echoed -> motorEnabled() false");
            check(src.state() == LidarState::LIDAR_STATE_IDLE, "state Idle while parked");
            check(src.connected(), "still connected while parked");
            countFrames(src, 300, f);   // drain what was in flight
            const Int32 quiet = countFrames(src, 600, f);
            std::snprintf(line.data(), line.size(), "%d frames while parked (want 0)", quiet);
            check(quiet == 0, line.data());

            // And back on.
            src.setMotorEnabled(true);
            waitMotor(src, true, "MOTOR 1 echoed -> motorEnabled() true");
            waitState(src, LidarState::LIDAR_STATE_SCANNING, "frames resume");
            const Int32 again = countFrames(src, 600, f);
            std::snprintf(
                line.data(),
                line.size(),
                "%d frames in 0.6 s after restart (want >= 3)",
                again
            );
            check(again >= 3, line.data());
        }

        src.stop();
        check(src.state() == LidarState::LIDAR_STATE_IDLE, "stop() -> Idle");
        check(!src.connected(), "not connected after stop()");
    }

    // ---- 3. the board goes away --------------------------------------------
    std::printf("\n-- 3. the board goes away (session: abrupt:5) --\n");
    {
        LidarSource src;
        src.start(target, 0);
        if(waitState(src, LidarState::LIDAR_STATE_SCANNING, "first frame"))
        {
            waitState(src, LidarState::LIDAR_STATE_UNPLUGGED, "connection reset");
            checkContains(src.error(), "the scan feed on 127.0.0.1 went away", "message");
            check(src.reachedDevice(), "had reached the device");
        }
        src.stop();
        check(src.state() == LidarState::LIDAR_STATE_UNPLUGGED, "stop() keeps Unplugged");
    }

    // ---- 4. the board reports a fault --------------------------------------
    std::printf("\n-- 4. the board reports a fault (session: err:3) --\n");
    {
        LidarSource src;
        src.start(target, 0);
        if(waitState(src, LidarState::LIDAR_STATE_SCANNING, "first frame"))
        {
            waitState(src, LidarState::LIDAR_STATE_ERROR, "ERR line");
            checkContains(src.error(), "internal error (fake)", "the board's words, verbatim");
            check(src.reachedDevice(), "had reached the device - not one to retry");
        }
        src.stop();
    }

    // ---- 5. what the fake saw ----------------------------------------------
    std::printf("\n-- 5. what the fake saw --\n");
    {
        // Give it a moment to write its last lines and exit.
        sleepMs(500);
        const Str log = readFile(logPath);
        check(!log.empty(), "the fake wrote a log");
        checkContains(log, "rx MOTOR 0", "hub sent MOTOR 0");
        checkContains(log, "rx MOTOR 1", "hub sent MOTOR 1");
        checkContains(log, "QUIT received", "hub sent QUIT on stop()");
        checkContains(log, "all sessions served", "fake served every session and exited");
    }

    // ---- 6. the pilot drives ------------------------------------------------
    std::printf("\n-- 6. the pilot drives (pilot session: normal, --drive --pilot) --\n");
    {
        LidarSource src;
        LidarDrive  d;
        // Not connectWhenListening(): that waits for Connecting to END, which
        // is the first frame, and the point here is what comes BEFORE it. The
        // pilot's fake has been listening since the first fake was started.
        src.start(pilot, 0);

        // Before any revolution: the pilot deciding on nothing while the lidar
        // spins up. Delivered without a frame, and says so.
        if(waitDrive(src, d, DEADLINE_MS))
        {
            check(d.mode == "blind", "first decision is blind, before any frame");
            check(d.stop, "a blind decision stops the car");
            check(d.hits == 0 && d.clearanceMm == 0, "and measures nothing");
            check(d.fresh(), "fresh() as it arrives");
            check(
                src.state() != LidarState::LIDAR_STATE_SCANNING,
                "delivered while still Connecting - no F yet"
            );
        }
        else
        {
            check(false, "a decision arrived");
        }

        if(waitState(src, LidarState::LIDAR_STATE_SCANNING, "first frame"))
        {
            // Each revolution's decision, matched to it by the fake's tag.
            LidarFrame f;
            Int32 matched = 0;
            Int32 mismatched = 0;
            Int32 missing = 0;
            Bool  sawCruise = false;
            Bool  sawSlow = false;
            Bool  sawStop = false;
            Bool  sawReverse = false;
            Bool  stopSaysStop = true;
            Bool  reverseBacks = true;
            const TimePoint began = monoNow();
            while(elapsedMs(began) < 2500)
            {
                if(!src.poll(f))
                {
                    sleepMs(5);
                    continue;
                }
                const Int32 tag = frameTag(f);
                // The D follows its F on the wire, so it is either already in
                // or a few milliseconds behind; an older one is skipped.
                Bool got = false;
                const TimePoint asked = monoNow();
                while(elapsedMs(asked) < 250)
                {
                    if(src.pollDrive(d))
                    {
                        if(d.hits == tag)
                        {
                            got = true;
                            break;
                        }
                        if(d.hits != (tag + 99) % 100)
                        {
                            ++mismatched;
                            break;
                        }
                    }
                    sleepMs(2);
                }
                if(!got)
                {
                    ++missing;
                    continue;
                }
                ++matched;
                sawCruise = sawCruise || d.mode == "cruise";
                sawSlow = sawSlow || d.mode == "slow";
                sawStop = sawStop || d.mode == "stop";
                sawReverse = sawReverse || d.mode == "reverse";
                if(d.mode == "stop" && !d.stop)
                {
                    stopSaysStop = false;
                }
                if(d.mode == "reverse" && !(d.throttle < 0.0f && !d.stop))
                {
                    reverseBacks = false;
                }
            }
            Array<Char, 128> line;
            std::snprintf(
                line.data(),
                line.size(),
                "%d decisions matched their revolution, %d wrong, %d missing (want >= 15, 0, 0)",
                matched,
                mismatched,
                missing
            );
            check(matched >= 15 && mismatched == 0 && missing == 0, line.data());
            check(sawCruise && sawSlow && sawStop && sawReverse, "all four driving modes seen");
            check(stopSaysStop, "every stop decision carries stop=1");
            check(reverseBacks, "every reverse decision is a negative throttle, not a stop");
            check(
                d.steer >= -0.3f && d.steer <= 0.3f,
                "steer is the fraction, not the thousandths"
            );
            check(d.fresh(), "the last one is fresh");

            // Stop motor: the pilot says no. One MOTOR 0 goes out, MOTOR 1
            // comes back, and the hub does not argue.
            src.setMotorEnabled(false);
            sleepMs(1200);   // two worker ticks and the answer
            check(src.motorEnabled(), "motorEnabled() stays true after MOTOR 1 came back");
            check(src.pollMotorRefused(), "pollMotorRefused() reports it once");
            check(!src.pollMotorRefused(), "and only once");
            check(src.state() == LidarState::LIDAR_STATE_SCANNING, "still Scanning");
            const Int32 still = countFrames(src, 600, f);
            std::snprintf(
                line.data(),
                line.size(),
                "%d frames in 0.6 s after the refusal (want >= 3)",
                still
            );
            check(still >= 3, line.data());
            sleepMs(1200);   // two more ticks in which a resend would happen
            check(src.motorEnabled(), "motor still on two ticks later");
        }

        src.stop();
        check(src.state() == LidarState::LIDAR_STATE_IDLE, "stop() -> Idle");
    }

    // ---- 7. the pilot stops deciding -------------------------------------------
    std::printf("\n-- 7. the pilot stops, the scan does not (pilot session: pilotquit:5) --\n");
    {
        LidarSource src;
        LidarDrive  d;
        LidarFrame  f;
        Int32 decisions = 0;
        Bool  freshOnArrival = true;

        // Counted from start(), because the blind ticks come before the first
        // frame and pollDrive() hands over only the newest: a count begun at
        // Scanning would have the spin-up overwritten before it was read.
        src.start(pilot, 0);
        {
            const TimePoint began = monoNow();
            while(elapsedMs(began) < 3000)
            {
                if(src.pollDrive(d))
                {
                    ++decisions;
                    freshOnArrival = freshOnArrival && d.fresh();
                }
                src.poll(f);
                sleepMs(5);
            }
        }

        if(waitState(src, LidarState::LIDAR_STATE_SCANNING, "scanning"))
        {
            Array<Char, 128> line;
            std::snprintf(
                line.data(),
                line.size(),
                "%d decisions, then none (want 3 blind + 5)",
                decisions
            );
            check(decisions == 8, line.data());
            check(freshOnArrival, "each was fresh as it arrived");
            std::snprintf(
                line.data(),
                line.size(),
                "the last is stale %.0f ms later (want > %d)",
                elapsedMs(d.at),
                LidarDrive::STALE_MS
            );
            check(!d.fresh(), line.data());
            check(src.state() == LidarState::LIDAR_STATE_SCANNING, "the scan carries on");
            const Int32 still = countFrames(src, 600, f);
            std::snprintf(line.data(), line.size(), "%d frames in 0.6 s (want >= 3)", still);
            check(still >= 3, line.data());
            check(!src.pollDrive(d), "pollDrive() has nothing new");
        }
        src.stop();
    }

    // ---- 8. what the pilot's fake saw ------------------------------------------
    std::printf("\n-- 8. what the pilot's fake saw --\n");
    {
        sleepMs(500);
        const Str log = readFile(pilotLogPath);
        check(!log.empty(), "the pilot's fake wrote a log");
        const Int32 motorOffs = countOf(log, "rx MOTOR 0");
        Array<Char, 96> line;
        std::snprintf(
            line.data(),
            line.size(),
            "hub sent MOTOR 0 %d time(s) (want exactly 1)",
            motorOffs
        );
        check(motorOffs == 1, line.data());
        checkContains(log, "MOTOR 0 refused", "and the fake refused it");
        check(countOf(log, "rx MOTOR 1") == 0, "hub never sent MOTOR 1 - nothing to put back");
        check(countOf(log, "QUIT received") == 2, "QUIT on both stop()s");
        checkContains(log, "all sessions served", "fake served both sessions and exited");
    }

    std::printf("\n%d check(s), %d failure(s)\n", checks, failures);
    std::printf("OVERALL: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
