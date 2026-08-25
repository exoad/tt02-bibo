// Hardware smoke test for LidarSource against a real RPLIDAR C1.
//
//   test_lidar_source.exe [port] [baud]      defaults: COM7 460800
//
// Exits 0 on PASS, 1 on FAIL. Reads no stdin, so it is safe to run from a
// shell that hands it an already-closed stdin.

#include "../src/lidar_source.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

namespace {

// Bounds taken from the verified behaviour of this device.
constexpr int   kRunSeconds   = 8;
constexpr int   kMinFrames    = 30;
constexpr float kMinHz        = 9.0f;
constexpr float kMaxHz        = 13.0f;
constexpr int   kMinPoints    = 350;
constexpr int   kMaxPoints    = 550;

// The motor spins up over the first couple of seconds: it starts fast and
// sparse (~355 points at ~14Hz) and settles to ~510 points at ~9.8Hz. That
// transient is the device's normal behaviour, not a fault, but it sits outside
// the steady-state bounds above - so the pass/fail statistics are gathered
// after it, while the spin-up is reported separately rather than hidden.
constexpr int   kSpinUpFrames = 15;

const char* state_name(LidarState s)
{
    switch (s) {
        case LidarState::Idle:       return "Idle";
        case LidarState::Connecting: return "Connecting";
        case LidarState::Scanning:   return "Scanning";
        case LidarState::Error:      return "Error";
    }
    return "?";
}

struct Check
{
    int failures = 0;

    void operator()(bool ok, const char* what, const std::string& detail)
    {
        std::printf("  [%s] %-28s %s\n", ok ? "PASS" : "FAIL", what, detail.c_str());
        if (!ok) ++failures;
    }
};

std::string f2s(const char* fmt, double v)
{
    char b[64];
    std::snprintf(b, sizeof(b), fmt, v);
    return b;
}

} // namespace

int main(int argc, char** argv)
{
    const std::string port = (argc > 1) ? argv[1] : "COM7";
    const int         baud = (argc > 2) ? std::atoi(argv[2]) : 460800;

    std::setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("=== LidarSource hardware test ===\n");
    std::printf("port=%s baud=%d\n\n", port.c_str(), baud);

    // ---- list_ports -------------------------------------------------------
    std::printf("-- list_ports --\n");
    std::vector<std::string> ports = LidarSource::list_ports();
    if (ports.empty()) {
        std::printf("  (none found)\n");
    } else {
        for (const std::string& p : ports) std::printf("  %s\n", p.c_str());
    }

    bool port_listed = false;
    for (const std::string& p : ports)
        if (p == port) port_listed = true;
    std::printf("\n");

    // ---- start ------------------------------------------------------------
    LidarSource src;
    src.start(port, baud);

    // start() must return immediately; anything slower means it blocked on the
    // driver rather than handing the work to the worker thread.
    std::printf("-- start() returned, state=%s --\n", state_name(src.state()));

    // ---- poll loop --------------------------------------------------------
    int    frames        = 0;
    int    printed       = 0;
    bool   info_shown    = false;
    bool   saw_scanning  = false;
    double hz_sum        = 0.0;
    int    hz_n          = 0;
    long   pts_sum       = 0;
    int    pts_min       = 1 << 30;
    int    pts_max       = 0;
    double validpct_sum  = 0.0;
    // Spin-up frames, tracked only so they can be shown.
    int    warm_pts_min  = 1 << 30;
    int    warm_pts_max  = 0;
    float  warm_hz_max   = 0.0f;
    LidarDeviceInfo di;

    const auto t0 = std::chrono::steady_clock::now();
    LidarFrame frame;

    std::printf("\n-- frames --\n");
    for (;;) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= kRunSeconds * 1000) break;

        LidarState st = src.state();
        if (st == LidarState::Scanning) saw_scanning = true;
        if (st == LidarState::Error) {
            std::printf("  ERROR state: %s\n", src.error().c_str());
            break;
        }

        if (!info_shown && st == LidarState::Scanning) {
            di = src.info();
            std::printf("  device: model=%d fw=%d.%02d hw=%d health=%d\n",
                        di.model, di.fw_major, di.fw_minor, di.hw_rev, di.health);
            std::printf("  serial: %s\n\n", di.serial.c_str());
            info_shown = true;
        }

        if (src.poll(frame)) {
            ++frames;
            int n = (int)frame.points.size();
            double vp = n ? (100.0 * frame.valid_count / n) : 0.0;

            if (frames <= kSpinUpFrames) {
                if (n < warm_pts_min) warm_pts_min = n;
                if (n > warm_pts_max) warm_pts_max = n;
                if (frame.hz > warm_hz_max) warm_hz_max = frame.hz;
            } else {
                pts_sum += n;
                if (n < pts_min) pts_min = n;
                if (n > pts_max) pts_max = n;
                hz_sum += frame.hz; ++hz_n;
                validpct_sum += vp;
            }

            // Every frame would be 80+ lines of scroll; a sample is enough to
            // show the stream is live and the numbers are sane.
            if (frames <= 5 || frames % 10 == 0) {
                std::printf("  frame %3d  pts=%4d  hz=%5.2f  valid=%5.1f%%  max=%7.1fmm\n",
                            frames, n, frame.hz, vp, frame.max_dist_mm);
                ++printed;
            }
        } else {
            // Mirrors a 60fps UI: poll, find nothing new, go do something else.
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    }
    (void)printed;

    const double secs = (double)std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0).count() / 1000.0;

    // ---- stop -------------------------------------------------------------
    std::printf("\n-- stop() --\n");
    const auto ts = std::chrono::steady_clock::now();
    src.stop();
    const double stop_ms = (double)std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - ts).count();
    std::printf("  stop() took %.0f ms, state=%s\n", stop_ms, state_name(src.state()));

    // stop() on an already-stopped source must be a no-op, not a crash.
    src.stop();
    std::printf("  second stop() ok\n");

    // The worker keeps scanning until it observes the quit flag, so it may well
    // publish one last frame during shutdown that the loop above never picked
    // up. Draining it here is expected and correct. What must NOT happen is the
    // same frame being handed out twice - that is the sequence counter's job.
    LidarFrame after;
    bool drained  = src.poll(after);
    bool repeated = src.poll(after);
    std::printf("  poll() after stop: drained=%s repeated=%s\n",
                drained ? "true" : "false", repeated ? "true" : "false");

    // ---- verdict ----------------------------------------------------------
    std::printf("\n=== summary ===\n");
    const int    steady  = hz_n;
    const double avg_hz  = steady ? hz_sum / steady : 0.0;
    const double avg_pts = steady ? (double)pts_sum / steady : 0.0;
    const double avg_vp  = steady ? validpct_sum / steady : 0.0;

    std::printf("  ran %.1fs, %d frames total (%d spin-up, %d steady)\n",
                secs, frames, frames < kSpinUpFrames ? frames : kSpinUpFrames, steady);
    if (warm_pts_max)
        std::printf("  spin-up: points %d..%d, peak hz %.2f (expected, motor accelerating)\n",
                    warm_pts_min, warm_pts_max, warm_hz_max);
    std::printf("  steady : hz avg %.2f | points avg %.0f (min %d max %d) | valid avg %.1f%%\n",
                avg_hz, avg_pts, steady ? pts_min : 0, pts_max, avg_vp);
    std::printf("\n");

    Check check;
    check(!ports.empty(), "list_ports non-empty",
          std::to_string(ports.size()) + " port(s)");
    check(port_listed, "target port enumerated", port);
    check(saw_scanning, "reached Scanning state", "");
    check(src.error().empty(), "no error message",
          src.error().empty() ? "" : src.error());

    check(di.model == 65, "device model == 65", std::to_string(di.model));
    check(di.fw_major == 1 && di.fw_minor == 2, "firmware == 1.02",
          std::to_string(di.fw_major) + "." + std::to_string(di.fw_minor));
    check(di.hw_rev == 18, "hardware rev == 18", std::to_string(di.hw_rev));
    check(di.serial == "A11FE18AC1EA9ED2B29C92F522BB466C", "serial matches", di.serial);
    check(di.health == 0, "health == 0 (good)", std::to_string(di.health));

    check(frames >= kMinFrames, "frames >= 30", std::to_string(frames));
    check(steady > 0 && avg_hz >= kMinHz && avg_hz <= kMaxHz,
          "steady hz in 9..13", f2s("%.2f", avg_hz));
    check(steady > 0 && pts_min >= kMinPoints && pts_max <= kMaxPoints,
          "steady points in 350..550",
          std::to_string(steady ? pts_min : 0) + ".." + std::to_string(pts_max));
    check(avg_vp > 50.0, "valid points > 50%", f2s("%.1f%%", avg_vp));

    check(!repeated, "no frame re-delivered", "");
    check(stop_ms < 3000.0, "stop() under 3s", f2s("%.0f ms", stop_ms));
    check(src.state() == LidarState::Idle, "state Idle after stop",
          state_name(src.state()));

    std::printf("\n%s (%d check(s) failed)\n",
                check.failures == 0 ? "OVERALL: PASS" : "OVERALL: FAIL",
                check.failures);
    return check.failures == 0 ? 0 : 1;
}
