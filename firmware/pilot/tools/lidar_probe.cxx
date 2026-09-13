// Is the lidar there, and what does it see?
//
//   lidar_probe [port] [baud]        default /dev/ttyUSB0 460800
//
// Prints the device's info and health, then for RUN_S one line per revolution:
// points, returns, the nearest return and its bearing, and the rotation rate.
// Then stops the motor, also on Ctrl-C.
//
// The rate is timed between revolutions as they arrive, not taken from the SDK,
// whose figure comes from the scan mode's nominal sample period.
//
// Exits 0 only when at least one revolution was seen: a run that only timed out
// has measured nothing.
#include "shared.hxx"

#include "lidar.hxx"

#include <csignal>
#include <cstdio>
#include <cstdlib>

namespace
{
  constexpr Float64 RUN_S = 5.0;

  // Set by the signal handler. volatile sig_atomic_t, not Atomic<>: it is the one
  // type the standard makes safe to write from a handler.
  volatile std::sig_atomic_t interrupted = 0;

  // Int32 is int on every host this builds for, so this matches std::signal's
  // handler type.
  Void onInterrupt(Int32)
  {
      interrupted = 1;
  }

  struct Revolution
  {
      Size points = 0;
      Size valid = 0;
      Float32 minMm = 0.0f;
      Float32 minDeg = 0.0f;
  };

  [[nodiscard]] Revolution summarise(const Vec<reactive::Ray>& rays)
  {
      Revolution r;
      r.points = rays.size();
      for(const reactive::Ray& ray : rays)
      {
          // 0 mm is no return, not a hit on the bumper.
          if(ray.distMm <= 0.0f)
          {
              continue;
          }
          if(r.valid == 0 || ray.distMm < r.minMm)
          {
              r.minMm = ray.distMm;
              r.minDeg = ray.angleDeg;
          }
          ++r.valid;
      }
      return r;
  }
}

Int32 main(Int32 argc, Char** argv)
{
    const Str port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    const Int32 baud = argc > 2 ? static_cast<Int32>(std::strtol(argv[2], nullptr, 10)) : 460800;
    // Installed before the motor can start, so a Ctrl-C never leaves it spinning.
    std::signal(SIGINT, onInterrupt);
    std::signal(SIGTERM, onInterrupt);
    // No lidar::available() check: without the SDK, open() refuses and reason() says so.
    std::printf("opening %s at %d baud\n", port.c_str(), baud);
    if(!lidar::open(port, baud))
    {
        std::printf("open failed: %s\n", lidar::reason().c_str());
        return 1;
    }
    std::printf("device  %s\n", lidar::info().c_str());
    std::printf("health  %s\n", lidar::health().c_str());
    // A Ctrl-C during open(), which blocks: the motor never starts. 1: no revolution.
    if(interrupted != 0)
    {
        std::printf("interrupted before the motor started\n");
        lidar::close();
        return 1;
    }
    if(!lidar::motorOn())
    {
        std::printf("motor on failed: %s\n", lidar::reason().c_str());
        lidar::close();
        return 1;
    }

    Vec<reactive::Ray> rays;
    Int32 revolutions = 0;
    Int32 timeouts = 0;
    const TimePoint start = monoNow();
    TimePoint last = start;
    Bool haveLast = false;
    while(elapsedS(start) < RUN_S && interrupted == 0)
    {
        if(!lidar::grab(rays))
        {
            ++timeouts;
            std::printf("no revolution: %s\n", lidar::reason().c_str());
            continue;
        }
        const TimePoint now = monoNow();
        const Revolution r = summarise(rays);
        ++revolutions;
        // The first revolution has no earlier one to take a rate from.
        Array<Char, 24> rate{};
        if(haveLast)
        {
            const Float64 s = Duration<Float64>(now - last).count();
            std::snprintf(rate.data(), rate.size(), "%5.2f rev/s", s > 0.0 ? 1.0 / s : 0.0);
        }
        else
        {
            std::snprintf(rate.data(), rate.size(), "%11s", "--");
        }
        last = now;
        haveLast = true;
        if(r.valid > 0)
        {
            std::printf(
                "rev %3d  points %4zu  valid %4zu  min %6.0f mm @ %6.1f deg  %s\n",
                revolutions,
                r.points,
                r.valid,
                static_cast<Float64>(r.minMm),
                static_cast<Float64>(r.minDeg),
                rate.data()
            );
        }
        else
        {
            // In words: "min 0 mm" would read as an obstacle against the lens.
            std::printf(
                "rev %3d  points %4zu  valid    0  no returns                  %s\n",
                revolutions,
                r.points,
                rate.data()
            );
        }
    }

    const Float64 ran = elapsedS(start);
    std::printf(
        "%s: %d revolutions, %d timeouts in %.1f s (%.2f rev/s over the run)\n",
        interrupted != 0 ? "interrupted" : "done",
        revolutions,
        timeouts,
        ran,
        ran > 0.0 ? revolutions / ran : 0.0
    );
    if(!lidar::motorOff())
    {
        std::printf("motor off failed: %s\n", lidar::reason().c_str());
    }
    lidar::close();
    return revolutions > 0 ? 0 : 1;
}
