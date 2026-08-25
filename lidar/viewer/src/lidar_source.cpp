// Threaded wrapper around Slamtec's rplidar_sdk driver.
//
// The SDK is entirely blocking: grabScanDataHq() parks until a full revolution
// has been assembled (~100ms at 10Hz), and connect()/getDeviceInfo() can sit on
// a serial timeout for a second or more. All of that lives on a worker thread
// so the UI never stalls. The worker owns the driver object outright - it is
// created and destroyed on that thread and never touched from anywhere else -
// which keeps the lifetime rules trivial. What crosses threads is only plain
// data: state, error text, device info, and the most recent completed frame,
// each guarded below.

#include "lidar_source.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <algorithm>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <cctype>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace {

// One revolution never comes close to this, but grabScanDataHq() wants an upper
// bound and the SDK's own samples use the same figure.
constexpr size_t kMaxNodes = 8192;

// Distinguishes "the scan died" from "this one revolution timed out", which is
// routine and must not be treated as fatal.
constexpr sl_u32 kGrabTimeoutMs = 2000;

// At a 2s grab timeout this gives the device ~10s of silence before the worker
// gives up. Long enough to ride out a hiccup, short enough to notice a unplug.
constexpr int kMaxConsecutiveTimeouts = 5;

// Checks that the port can actually be opened, returning a specific complaint
// or an empty string if all is well.
//
// This exists because the SDK cannot be trusted to report an unopenable port.
// In sl_async_transceiver.cpp, openChannelAndBind() declares `u_result ans` and
// then shadows it inside the do-block with `Result<nullptr_t> ans`; the
// channel->open() failure is written to the shadowed copy, so the outer value
// stays RESULT_OK and connect() returns success for a port that was never
// opened. The symptom is that a missing COM port gets misdiagnosed further
// down as "no response from device (wrong baud?)", pointing at the wrong cause.
// Probing here restores the distinction the caller needs.
//
// The handle is closed again immediately so the SDK can take the port. That
// leaves a small window in which another process could claim it in between, in
// which case the driver's own failure path still reports a problem - just less
// precisely. Worth it for a correct message in the overwhelmingly common case.
std::string probe_port(const std::string& dev_path, const std::string& friendly)
{
    HANDLE h = CreateFileA(dev_path.c_str(),
                           GENERIC_READ | GENERIC_WRITE,
                           0,               // serial ports are exclusive
                           nullptr,
                           OPEN_EXISTING,
                           0,
                           nullptr);
    if (h != INVALID_HANDLE_VALUE) {
        CloseHandle(h);
        return std::string();
    }

    const DWORD err = GetLastError();
    switch (err) {
        case ERROR_FILE_NOT_FOUND:
        case ERROR_PATH_NOT_FOUND:
            return "cannot open " + friendly + " (no such port; is the device plugged in?)";
        case ERROR_ACCESS_DENIED:
        case ERROR_SHARING_VIOLATION:
            return "cannot open " + friendly + " (in use by another program)";
        default: {
            char buf[64];
            std::snprintf(buf, sizeof(buf), " (win32 error %lu)", (unsigned long)err);
            return "cannot open " + friendly + buf;
        }
    }
}

} // namespace

// ---------------------------------------------------------------------------

struct LidarSource::Impl
{
    std::thread       worker;
    std::atomic<bool> quit{false};
    std::atomic<bool> running{false};   // worker exists and has not been joined

    std::atomic<LidarState> state{LidarState::Idle};

    // Guards everything the worker publishes to the UI thread.
    mutable std::mutex  mtx;
    std::string         error_msg;
    LidarDeviceInfo     dev_info;
    LidarScanInfo       scan_info;
    LidarFrame          frame;
    uint64_t            frame_seq = 0;   // bumped on every published frame

    // Session counters. Written only by the worker, read by the UI thread, so
    // atomics keep them off the publish mutex - the UI reads them every frame
    // while the worker only touches them once per revolution.
    std::atomic<unsigned long long> stat_frames{0};
    std::atomic<unsigned long long> stat_points{0};
    std::atomic<unsigned int>       stat_timeouts{0};
    std::chrono::steady_clock::time_point scan_start{};
    std::atomic<bool>               scan_started{false};

    // Touched only by poll(), i.e. only by the UI thread.
    uint64_t last_seen_seq = 0;

    void set_error(const std::string& msg)
    {
        {
            std::lock_guard<std::mutex> lock(mtx);
            error_msg = msg;
        }
        state.store(LidarState::Error, std::memory_order_release);
    }

    void run(std::string port, int baud);
};

// ---------------------------------------------------------------------------

void LidarSource::Impl::run(std::string port, int baud)
{
    using namespace sl;

    state.store(LidarState::Connecting, std::memory_order_release);

    // COM10 and above are only reachable through the \\.\ device namespace, and
    // the prefix is harmless for the single-digit ports, so it always goes on.
    std::string dev_path = port;
    if (dev_path.rfind("\\\\.\\", 0) != 0)
        dev_path = "\\\\.\\" + port;

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        set_error("out of memory creating the lidar driver");
        return;
    }

    // From here on every exit path must tear the driver down, so the body is
    // wrapped and cleanup happens once at the bottom. The channel is declared
    // out here because the SDK never takes ownership of it - unbindAndClose()
    // only close()s it - so this thread has to free it on every exit path.
    bool      scanning = false;
    IChannel* channel  = nullptr;

    do {
        // Done before the driver touches the port, because the driver's own
        // report of this failure is unreliable - see probe_port().
        std::string port_problem = probe_port(dev_path, port);
        if (!port_problem.empty()) {
            set_error(port_problem);
            break;
        }

        // Assigns the outer `channel` deliberately - declaring a new one here
        // would shadow it and leak the channel plus its three event handles.
        channel = *createSerialPortChannel(dev_path.c_str(), (sl_u32)baud);
        if (!channel) {
            set_error("cannot create a serial channel for " + port);
            break;
        }

        // connect() failing means the port itself would not open: it does not
        // exist, or another process holds it. Nothing to do with baud rate.
        if (SL_IS_FAIL(drv->connect(channel))) {
            set_error("cannot open " + port + " (in use or missing)");
            break;
        }

        // The port opened but nothing answered the info request. The cable and
        // the port are fine; the framing is wrong or the device is unpowered.
        sl_lidar_response_device_info_t raw_info;
        if (SL_IS_FAIL(drv->getDeviceInfo(raw_info))) {
            set_error("no response from device on " + port +
                      " (wrong baud rate?)");
            break;
        }

        LidarDeviceInfo di;
        di.model    = (int)raw_info.model;
        di.fw_major = raw_info.firmware_version >> 8;
        di.fw_minor = raw_info.firmware_version & 0xFF;
        di.hw_rev   = (int)raw_info.hardware_version;

        char hex[33];
        for (int i = 0; i < 16; ++i)
            std::snprintf(hex + i * 2, 3, "%02X", (unsigned)raw_info.serialnum[i]);
        di.serial = hex;

        sl_lidar_response_device_health_t health;
        if (SL_IS_OK(drv->getHealth(health)))
            di.health = (int)health.status;

        {
            std::lock_guard<std::mutex> lock(mtx);
            dev_info = di;
        }

        // A hard health error means the unit will not produce usable data until
        // it is power cycled; starting the motor anyway just makes noise.
        if (di.health == SL_LIDAR_STATUS_ERROR) {
            set_error("lidar reports an internal error; power cycle the device");
            break;
        }

        drv->setMotorSpeed();

        LidarScanMode mode;
        if (SL_IS_FAIL(drv->startScan(0, 1, 0, &mode))) {
            set_error("failed to start scan on " + port);
            drv->setMotorSpeed(0);
            break;
        }
        scanning = true;

        // Record what the SDK actually negotiated. The C1 chooses the mode, and
        // without this the sample period and the mode's own range ceiling are
        // invisible to the operator.
        {
            LidarScanInfo si;
            si.mode_id        = (int)mode.id;
            si.us_per_sample  = mode.us_per_sample;
            si.max_distance_m = mode.max_distance;

            // scan_mode is a fixed 64-byte field, zero-padded but not
            // guaranteed terminated.
            char name[sizeof(mode.scan_mode) + 1];
            std::memcpy(name, mode.scan_mode, sizeof(mode.scan_mode));
            name[sizeof(mode.scan_mode)] = '\0';
            si.mode = name;

            std::lock_guard<std::mutex> lock(mtx);
            scan_info = si;
        }

        scan_start = std::chrono::steady_clock::now();
        scan_started.store(true, std::memory_order_release);

        state.store(LidarState::Scanning, std::memory_order_release);

        std::vector<sl_lidar_response_measurement_node_hq_t> nodes(kMaxNodes);

        // Reused across iterations so the steady state does no allocation once
        // the vector has grown to a revolution's worth of points.
        LidarFrame staging;
        staging.points.reserve(1024);

        int consecutive_timeouts = 0;

        while (!quit.load(std::memory_order_acquire)) {
            size_t count = nodes.size();

            if (SL_IS_FAIL(drv->grabScanDataHq(nodes.data(), count, kGrabTimeoutMs))) {
                // A dropped revolution is normal - the device occasionally
                // misses its window. A long unbroken run of them is not: the
                // cable came out, or the device stopped talking. Surface that
                // instead of sitting in Scanning forever with a frozen view.
                stat_timeouts.fetch_add(1, std::memory_order_relaxed);

                if (++consecutive_timeouts >= kMaxConsecutiveTimeouts) {
                    set_error("device stopped responding on " + port +
                              " (cable unplugged?)");
                    break;
                }
                continue;
            }
            consecutive_timeouts = 0;

            drv->ascendScanData(nodes.data(), count);

            float freq = 0.0f;
            drv->getFrequency(mode, nodes.data(), count, freq);

            staging.points.clear();
            staging.hz          = freq;
            staging.valid_count = 0;
            staging.max_dist_mm = 0.0f;

            for (size_t i = 0; i < count; ++i) {
                LidarPoint p;
                // angle_z_q14 is q14 fixed point scaled so that 1.0 == 90 deg.
                p.angle_deg = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
                p.dist_mm   = nodes[i].dist_mm_q2 / 4.0f;
                p.quality   = (uint8_t)(nodes[i].quality >>
                                        SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

                if (p.dist_mm > 0.0f) {
                    ++staging.valid_count;
                    if (p.dist_mm > staging.max_dist_mm)
                        staging.max_dist_mm = p.dist_mm;
                }
                staging.points.push_back(p);
            }

            // Publish. The lock is held only for the swap and counter bump, so
            // a UI thread polling at 60Hz never waits on the scan conversion.
            {
                std::lock_guard<std::mutex> lock(mtx);
                std::swap(frame, staging);
                ++frame_seq;
            }

            stat_frames.fetch_add(1, std::memory_order_relaxed);
            stat_points.fetch_add(count, std::memory_order_relaxed);
        }
    } while (false);

    // Order matters: stop the scan, give the device a moment to acknowledge it,
    // then cut the motor. Killing the motor first can leave the scan running.
    if (scanning) {
        drv->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        drv->setMotorSpeed(0);
    }
    delete drv;

    // The driver only close()s the channel on teardown, it never frees it, so
    // ownership comes back here. Must follow `delete drv`, which still uses it.
    delete channel;

    // A clean shutdown returns to Idle; a failure keeps its Error state and the
    // message that explains it.
    if (state.load(std::memory_order_acquire) != LidarState::Error)
        state.store(LidarState::Idle, std::memory_order_release);
}

// ---------------------------------------------------------------------------

LidarSource::LidarSource() : impl_(new Impl) {}

LidarSource::~LidarSource()
{
    stop();
    delete impl_;
}

void LidarSource::start(const std::string& port, int baud)
{
    // Starting over an existing session would leak a thread and leave the motor
    // spinning, so the previous one is retired first.
    stop();

    {
        std::lock_guard<std::mutex> lock(impl_->mtx);
        impl_->error_msg = std::string();
        impl_->dev_info  = LidarDeviceInfo();
        impl_->scan_info = LidarScanInfo();
        impl_->frame     = LidarFrame();
        impl_->frame_seq = 0;
    }
    impl_->last_seen_seq = 0;

    // Counters describe one session, so they restart with it.
    impl_->stat_frames.store(0, std::memory_order_relaxed);
    impl_->stat_points.store(0, std::memory_order_relaxed);
    impl_->stat_timeouts.store(0, std::memory_order_relaxed);
    impl_->scan_started.store(false, std::memory_order_release);

    impl_->quit.store(false, std::memory_order_release);
    impl_->state.store(LidarState::Connecting, std::memory_order_release);
    impl_->running.store(true, std::memory_order_release);

    Impl* impl = impl_;
    impl_->worker = std::thread([impl, port, baud] { impl->run(port, baud); });
}

void LidarSource::stop()
{
    // Safe when never started and safe to call twice: joinable() is false in
    // both cases and there is nothing to unwind.
    if (!impl_->worker.joinable()) {
        impl_->running.store(false, std::memory_order_release);
        return;
    }

    impl_->quit.store(true, std::memory_order_release);

    // The worker performs the scan stop, the 200ms settle and the motor stop
    // before returning, so joining is exactly the "blocks until the motor is
    // off" guarantee the header promises. Worst case is one grab timeout.
    impl_->worker.join();
    impl_->running.store(false, std::memory_order_release);
}

LidarState LidarSource::state() const
{
    return impl_->state.load(std::memory_order_acquire);
}

std::string LidarSource::error() const
{
    std::lock_guard<std::mutex> lock(impl_->mtx);
    return impl_->error_msg;
}

LidarDeviceInfo LidarSource::info() const
{
    std::lock_guard<std::mutex> lock(impl_->mtx);
    return impl_->dev_info;
}

LidarScanInfo LidarSource::scan_info() const
{
    std::lock_guard<std::mutex> lock(impl_->mtx);
    return impl_->scan_info;
}

LidarStats LidarSource::stats() const
{
    LidarStats s;
    s.frames   = impl_->stat_frames.load(std::memory_order_relaxed);
    s.points   = impl_->stat_points.load(std::memory_order_relaxed);
    s.timeouts = impl_->stat_timeouts.load(std::memory_order_relaxed);

    if (impl_->scan_started.load(std::memory_order_acquire))
    {
        const auto now = std::chrono::steady_clock::now();
        s.uptime_s = std::chrono::duration<double>(now - impl_->scan_start).count();
    }
    return s;
}

bool LidarSource::poll(LidarFrame& out)
{
    std::lock_guard<std::mutex> lock(impl_->mtx);

    // Frames land at ~10Hz while this is called at ~60Hz, so the overwhelmingly
    // common path is this comparison and an immediate return.
    if (impl_->frame_seq == impl_->last_seen_seq)
        return false;

    out = impl_->frame;
    impl_->last_seen_seq = impl_->frame_seq;
    return true;
}

// ---------------------------------------------------------------------------

std::string LidarSource::preferred_port()
{
    // HKLM\HARDWARE\DEVICEMAP\SERIALCOMM maps each driver's device name to the
    // COM port it owns. The CP210x driver registers as \Device\Silabser<N>,
    // which identifies the RPLIDAR's USB adapter without needing SetupAPI or
    // WMI - the other ports on a typical machine are Bluetooth links.
    std::string found;

    HKEY key = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM",
                      0, KEY_READ, &key) != ERROR_SUCCESS)
        return found;

    for (DWORD i = 0; ; ++i)
    {
        char  name[512];
        BYTE  data[512];
        DWORD nlen = (DWORD)sizeof(name);
        DWORD dlen = (DWORD)sizeof(data);
        DWORD type = 0;

        const LONG r = RegEnumValueA(key, i, name, &nlen, nullptr, &type, data, &dlen);
        if (r != ERROR_SUCCESS)
            break;
        if (type != REG_SZ || dlen == 0)
            continue;

        // Registry strings are not guaranteed to be terminated.
        if (dlen >= sizeof(data)) dlen = (DWORD)sizeof(data) - 1;
        data[dlen] = 0;

        std::string dev(name, nlen);
        for (char& c : dev) c = (char)std::tolower((unsigned char)c);

        if (dev.find("silabser") != std::string::npos)
        {
            found = reinterpret_cast<const char*>(data);
            break;
        }
    }

    RegCloseKey(key);
    return found;
}

std::vector<std::string> LidarSource::list_ports()
{
    std::vector<std::string> ports;

    // QueryDosDeviceA with a NULL device name returns every DOS device as a
    // packed run of NUL-terminated strings. It needs no extra import library
    // beyond kernel32, which is why it is preferred over SetupAPI here.
    try {
        DWORD cap = 8192;
        std::vector<char> buf;

        for (int attempt = 0; attempt < 6; ++attempt) {
            buf.assign(cap, '\0');
            DWORD n = QueryDosDeviceA(nullptr, buf.data(), cap);
            if (n != 0) {
                // Walk the packed list; a lone NUL terminates the whole block.
                const char* p = buf.data();
                const char* end = buf.data() + n;
                while (p < end && *p) {
                    size_t len = std::strlen(p);

                    // Accept only COM followed entirely by digits, so entries
                    // such as "COMEDIA" or "COM" alone are rejected.
                    if (len > 3 &&
                        (p[0] == 'C' || p[0] == 'c') &&
                        (p[1] == 'O' || p[1] == 'o') &&
                        (p[2] == 'M' || p[2] == 'm')) {
                        bool all_digits = true;
                        for (size_t i = 3; i < len; ++i) {
                            if (p[i] < '0' || p[i] > '9') { all_digits = false; break; }
                        }
                        if (all_digits)
                            ports.push_back(std::string(p, len));
                    }
                    p += len + 1;
                }
                break;
            }

            if (GetLastError() != ERROR_INSUFFICIENT_BUFFER)
                break;
            cap *= 2;
        }

        // Numeric order, so COM10 follows COM9 instead of preceding it.
        std::sort(ports.begin(), ports.end(),
                  [](const std::string& a, const std::string& b) {
                      long na = std::strtol(a.c_str() + 3, nullptr, 10);
                      long nb = std::strtol(b.c_str() + 3, nullptr, 10);
                      if (na != nb) return na < nb;
                      return a < b;
                  });
        ports.erase(std::unique(ports.begin(), ports.end()), ports.end());
    }
    catch (...) {
        // The header promises this never throws; a partial list beats an
        // exception escaping into the UI's frame loop.
    }

    return ports;
}
