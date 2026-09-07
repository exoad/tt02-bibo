// Is the lidar there, and what does it see? The first program to run on the
// Pi after the C1 is plugged in, and the one to run again when the autonomy
// does something strange and the question is whether the sensor or the
// controller is at fault.
//
//   lidar_probe [port] [baud]        default /dev/ttyUSB0 460800
//
// Opens the port, prints what the device says about itself, then for five
// seconds prints one line per revolution: how many points came back, how many
// carried a return, the nearest return and where it was, and the rotation
// rate. Then stops the motor - also on Ctrl-C, because a lidar left spinning
// on a desk by a program that was interrupted is the specific mess this
// project has already made once with the hub.
//
// The rotation rate is MEASURED, from the wall clock between one revolution
// arriving and the next, rather than asked of the SDK. The SDK's figure is
// derived from the scan mode's nominal sample period; the wall clock is what
// the autonomy loop will actually be paced by, and the two disagreeing is
// worth seeing.
//
// Exits 0 only when at least one revolution was seen. A probe that opens a
// port, times out five times and reports success has measured nothing, and
// this project has written that bug before.

#include "shared.hxx"

#include "lidar.hxx"

#include <csignal>
#include <cstdio>
#include <cstdlib>

namespace
{

  constexpr Float64 RUN_S = 5.0;

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
          // A zero is no return, not a hit on the bumper - reactive.hxx, trap 2.
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

    // Installed before the motor can start, or the header's "also on Ctrl-C"
    // is a promise with a hole in it: a signal between motorOn() and a later
    // std::signal() takes the default action and leaves the lidar spinning.
    // The handlers only store a flag, so going in first costs nothing.
    std::signal(SIGINT, onInterrupt);
    std::signal(SIGTERM, onInterrupt);

    // No lidar::available() check first: a build without the SDK refuses at
    // open() with a reason that says so, and printing that is the whole job.
    std::printf("opening %s at %d baud\n", port.c_str(), baud);
    if(!lidar::open(port, baud))
    {
        std::printf("open failed: %s\n", lidar::reason().c_str());
        return 1;
    }
    std::printf("device  %s\n", lidar::info().c_str());
    std::printf("health  %s\n", lidar::health().c_str());

    // A Ctrl-C that landed during open(), which blocks for a while. Nothing
    // is spinning yet, and starting the motor now would be doing exactly what
    // the person asked not to happen. 1, not 0: no revolution was seen.
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

        // The first revolution has nothing to be measured against, and printing
        // a rate for it would be printing the time since motorOn().
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
            // Said in words rather than as "min 0 mm": a revolution with no
            // returns at all is the blind case, and a zero here would look like
            // an obstacle against the lens.
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
