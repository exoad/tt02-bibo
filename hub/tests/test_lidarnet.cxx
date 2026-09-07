// LidarSource over the network, against fake_scanfeed.py.
//
//   test_lidarnet.exe PORT LOGFILE
//
// build_lidarnet_test.bat starts the fake on PORT with the session list this
// file expects - junk, abrupt:5, err:3 - and hands over the path of its log,
// which the last section reads to prove the hub said MOTOR 0, MOTOR 1 and QUIT.
// PORT+1 must have nothing listening on it; that is the refused-connect case.
//
// No hardware, no SDK call reached: the serial half of lidar_source.cxx is
// linked in because it is the same object, and never started.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "lidar_source.hxx"

#include <cstdio>
#include <cstdlib>

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
    if(argc < 3)
    {
        std::printf("usage: test_lidarnet.exe PORT LOGFILE\n");
        return 1;
    }
    const Int32 port = std::atoi(argv[1]);
    const Str   logPath = argv[2];
    const Str   target = "127.0.0.1:" + std::to_string(port);
    const Str   nowhere = "127.0.0.1:" + std::to_string(port + 1);

    std::setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("=== LidarSource over the scan feed ===\n");
    std::printf("feed at %s, nothing at %s\n", target.c_str(), nowhere.c_str());

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

    std::printf("\n%d check(s), %d failure(s)\n", checks, failures);
    std::printf("OVERALL: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
