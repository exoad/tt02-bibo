// Hardware smoke test for LidarSource against a real RPLIDAR C1.
//
//   test_lidar_source.exe [port] [baud]      defaults: COM7 460800
//
// Exits 0 on PASS, 1 on FAIL. Reads no stdin, so it is safe to run from a
// shell that hands it an already-closed stdin.

#include "shared.hxx"
#include "../src/lidar_source.hxx"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {

// Bounds taken from the verified behaviour of this device.
constexpr Int32   RUN_SECONDS   = 8;
constexpr Int32   MIN_FRAMES    = 30;
constexpr Float32 MIN_HZ        = 9.0f;
constexpr Float32 MAX_HZ        = 13.0f;
constexpr Int32   MIN_POINTS    = 350;
constexpr Int32   MAX_POINTS    = 550;

// The motor spins up over the first couple of seconds: it starts fast and
// sparse (~355 points at ~14Hz) and settles to ~510 points at ~9.8Hz. That
// transient is the device's normal behaviour, not a fault, but it sits outside
// the steady-state bounds above - so the pass/fail statistics are gathered
// after it, while the spin-up is reported separately rather than hidden.
constexpr Int32   SPIN_UP_FRAMES = 15;

const Char* stateName(LidarState s)
{
    switch(s)
    {
        case LidarState::LIDAR_STATE_IDLE:       return "Idle";
        case LidarState::LIDAR_STATE_CONNECTING: return "Connecting";
        case LidarState::LIDAR_STATE_SCANNING:   return "Scanning";
        case LidarState::LIDAR_STATE_ERROR:      return "Error";
    }
    return "?";
}

struct Check
{
    Int32 failures = 0;

    Void operator()(Bool ok, const Char* what, const Str& detail)
    {
        std::printf("  [%s] %-28s %s\n", ok ? "PASS" : "FAIL", what, detail.c_str());
        if(!ok)
        {
            ++failures;
        }
    }
};

Str fmtValue(const Char* fmt, Float64 v)
{
    Char b[64];
    std::snprintf(b, sizeof(b), fmt, v);
    return b;
}

} // namespace

int main(int argc, char** argv)
{
    const Str port = (argc > 1) ? argv[1] : "COM7";
    const Int32         baud = (argc > 2) ? std::atoi(argv[2]) : 460800;

    std::setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("=== LidarSource hardware test ===\n");
    std::printf("port=%s baud=%d\n\n", port.c_str(), baud);

    // ---- listPorts -------------------------------------------------------
    std::printf("-- listPorts --\n");
    Vec<Str> ports = LidarSource::listPorts();
    if(ports.empty())
    {
        std::printf("  (none found)\n");
    }
    else
    {
        for(const Str& p : ports)
        {
            std::printf("  %s\n", p.c_str());
        }
    }

    Bool portListed = false;
    for(const Str& p : ports)
        if(p == port)
        {
            portListed = true;
        }
    std::printf("\n");

    // ---- start ------------------------------------------------------------
    LidarSource src;
    src.start(port, baud);

    // start() must return immediately; anything slower means it blocked on the
    // driver rather than handing the work to the worker thread.
    std::printf("-- start() returned, state=%s --\n", stateName(src.state()));

    // ---- poll loop --------------------------------------------------------
    Int32    frames        = 0;
    Int32    printed       = 0;
    Bool   infoShown    = false;
    Bool   sawScanning  = false;
    Float64 hzSum        = 0.0;
    Int32    hzN          = 0;
    Int64  ptsSum       = 0;
    Int32    ptsMin       = 1 << 30;
    Int32    ptsMax       = 0;
    Float64 validPctSum  = 0.0;
    // Spin-up frames, tracked only so they can be shown.
    Int32    warmPtsMin  = 1 << 30;
    Int32    warmPtsMax  = 0;
    Float32  warmHzMax   = 0.0f;
    LidarDeviceInfo di;

    const TimePoint t0 = monoNow();
    LidarFrame frame;

    std::printf("\n-- frames --\n");
    for(;;)
    {
        const Float64 elapsed = elapsedMs(t0);
        if(elapsed >= RUN_SECONDS * 1000)
        {
            break;
        }

        LidarState st = src.state();
        if(st == LidarState::LIDAR_STATE_SCANNING)
        {
            sawScanning = true;
        }
        if(st == LidarState::LIDAR_STATE_ERROR)
        {
            std::printf("  ERROR state: %s\n", src.error().c_str());
            break;
        }

        if(!infoShown && st == LidarState::LIDAR_STATE_SCANNING)
        {
            di = src.info();
            std::printf("  device: model=%d fw=%d.%02d hw=%d health=%d\n",
                        di.model, di.fwMajor, di.fwMinor, di.hwRev, di.health);
            std::printf("  serial: %s\n\n", di.serial.c_str());
            infoShown = true;
        }

        if(src.poll(frame))
        {
            ++frames;
            Int32 n = static_cast<Int32>(frame.points.size());
            Float64 vp = n ? (100.0 * frame.validCount / n) : 0.0;

            if(frames <= SPIN_UP_FRAMES)
            {
                if(n < warmPtsMin)
                {
                    warmPtsMin = n;
                }
                if(n > warmPtsMax)
                {
                    warmPtsMax = n;
                }
                if(frame.hz > warmHzMax)
                {
                    warmHzMax = frame.hz;
                }
            }
            else
            {
                ptsSum += n;
                if(n < ptsMin)
                {
                    ptsMin = n;
                }
                if(n > ptsMax)
                {
                    ptsMax = n;
                }
                hzSum += frame.hz; ++hzN;
                validPctSum += vp;
            }

            // Every frame would be 80+ lines of scroll; a sample is enough to
            // show the stream is live and the numbers are sane.
            if(frames <= 5 || frames % 10 == 0)
            {
                std::printf("  frame %3d  pts=%4d  hz=%5.2f  valid=%5.1f%%  max=%7.1fmm\n",
                            frames, n, frame.hz, vp, frame.maxDistMm);
                ++printed;
            }
        }
        else
        {
            // Mirrors a 60fps UI: poll, find nothing new, go do something else.
            sleepMs(16);
        }
    }
    static_cast<Void>(printed);

    const Float64 secs = elapsedS(t0);

    // ---- stop -------------------------------------------------------------
    std::printf("\n-- stop() --\n");
    const TimePoint ts = monoNow();
    src.stop();
    const Float64 stopMs = elapsedMs(ts);
    std::printf("  stop() took %.0f ms, state=%s\n", stopMs, stateName(src.state()));

    // stop() on an already-stopped source must be a no-op, not a crash.
    src.stop();
    std::printf("  second stop() ok\n");

    // The worker keeps scanning until it observes the quit flag, so it may well
    // publish one last frame during shutdown that the loop above never picked
    // up. Draining it here is expected and correct. What must NOT happen is the
    // same frame being handed out twice - that is the sequence counter's job.
    LidarFrame after;
    Bool drained  = src.poll(after);
    Bool repeated = src.poll(after);
    std::printf("  poll() after stop: drained=%s repeated=%s\n",
                drained ? "true" : "false", repeated ? "true" : "false");

    // ---- verdict ----------------------------------------------------------
    std::printf("\n=== summary ===\n");
    const Int32    steady  = hzN;
    const Float64 avgHz  = steady ? hzSum / steady : 0.0;
    const Float64 avgPts = steady ? static_cast<Float64>(ptsSum) / steady : 0.0;
    const Float64 avgValidPct  = steady ? validPctSum / steady : 0.0;

    std::printf("  ran %.1fs, %d frames total (%d spin-up, %d steady)\n",
                secs, frames, frames < SPIN_UP_FRAMES ? frames : SPIN_UP_FRAMES, steady);
    if(warmPtsMax)
        std::printf("  spin-up: points %d..%d, peak hz %.2f (expected, motor accelerating)\n",
                    warmPtsMin, warmPtsMax, warmHzMax);
    std::printf("  steady : hz avg %.2f | points avg %.0f (min %d max %d) | valid avg %.1f%%\n",
                avgHz, avgPts, steady ? ptsMin : 0, ptsMax, avgValidPct);
    std::printf("\n");

    Check check;
    check(!ports.empty(), "listPorts non-empty",
          std::to_string(ports.size()) + " port(s)");
    check(portListed, "target port enumerated", port);
    check(sawScanning, "reached Scanning state", "");
    check(src.error().empty(), "no error message",
          src.error().empty() ? "" : src.error());

    check(di.model == 65, "device model == 65", std::to_string(di.model));
    check(di.fwMajor == 1 && di.fwMinor == 2, "firmware == 1.02",
          std::to_string(di.fwMajor) + "." + std::to_string(di.fwMinor));
    check(di.hwRev == 18, "hardware rev == 18", std::to_string(di.hwRev));
    check(di.serial == "A11FE18AC1EA9ED2B29C92F522BB466C", "serial matches", di.serial);
    check(di.health == 0, "health == 0 (good)", std::to_string(di.health));

    check(frames >= MIN_FRAMES, "frames >= 30", std::to_string(frames));
    check(steady > 0 && avgHz >= MIN_HZ && avgHz <= MAX_HZ,
          "steady hz in 9..13", fmtValue("%.2f", avgHz));
    check(steady > 0 && ptsMin >= MIN_POINTS && ptsMax <= MAX_POINTS,
          "steady points in 350..550",
          std::to_string(steady ? ptsMin : 0) + ".." + std::to_string(ptsMax));
    check(avgValidPct > 50.0, "valid points > 50%", fmtValue("%.1f%%", avgValidPct));

    check(!repeated, "no frame re-delivered", "");
    check(stopMs < 3000.0, "stop() under 3s", fmtValue("%.0f ms", stopMs));
    check(src.state() == LidarState::LIDAR_STATE_IDLE, "state Idle after stop",
          stateName(src.state()));

    std::printf("\n%s (%d check(s) failed)\n",
                check.failures == 0 ? "OVERALL: PASS" : "OVERALL: FAIL",
                check.failures);
    return check.failures == 0 ? 0 : 1;
}
