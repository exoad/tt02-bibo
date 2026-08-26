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

#include "shared.hpp"
#include "lidar_source.hpp"

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
constexpr Size MAX_NODES = 8192;

// Distinguishes "the scan died" from "this one revolution timed out", which is
// routine and must not be treated as fatal.
constexpr sl_u32 GRAB_TIMEOUT_MS = 2000;

// At a 2s grab timeout this gives the device ~10s of silence before the worker
// gives up. Long enough to ride out a hiccup, short enough to notice a unplug.
constexpr Int32 MAX_CONSECUTIVE_TIMEOUTS = 5;

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
Str probePort(const Str& devPath, const Str& friendly)
{
    HANDLE h = CreateFileA(devPath.c_str(),
                           GENERIC_READ | GENERIC_WRITE,
                           0,               // serial ports are exclusive
                           nullptr,
                           OPEN_EXISTING,
                           0,
                           nullptr);
    if(h != INVALID_HANDLE_VALUE) {
        CloseHandle(h);
        return Str();
    }

    const DWORD err = GetLastError();
    switch(err) {
        case ERROR_FILE_NOT_FOUND:
        case ERROR_PATH_NOT_FOUND:
            return "cannot open " + friendly + " (no such port; is the device plugged in?)";
        case ERROR_ACCESS_DENIED:
        case ERROR_SHARING_VIOLATION:
            return "cannot open " + friendly + " (in use by another program)";
        default: {
            Char buf[64];
            std::snprintf(buf, sizeof(buf), " (win32 error %lu)", static_cast<unsigned long>(err));
            return "cannot open " + friendly + buf;
        }
    }
}

} // namespace

// ---------------------------------------------------------------------------

struct LidarSource::Impl
{
    std::thread       worker;
    std::atomic<Bool> quit{false};

    // Owned by the UI thread, read by the worker. Starts true so a connect
    // spins up as it always has.
    std::atomic<Bool> motorOn{ true };
    std::atomic<Bool> running{false};   // worker exists and has not been joined

    std::atomic<LidarState> state{LidarState::LIDAR_STATE_IDLE};

    // Guards everything the worker publishes to the UI thread.
    mutable std::mutex  mtx;
    Str         errorMsg;
    LidarDeviceInfo     devInfo;
    LidarScanInfo       scanInfo;
    LidarFrame          frame;
    UInt64            frameSeq = 0;   // bumped on every published frame

    // Session counters. Written only by the worker, read by the UI thread, so
    // atomics keep them off the publish mutex - the UI reads them every frame
    // while the worker only touches them once per revolution.
    std::atomic<UInt64> statFrames{0};
    std::atomic<UInt64> statPoints{0};
    std::atomic<UInt32>       statTimeouts{0};
    std::chrono::steady_clock::time_point scanStart{};
    std::atomic<Bool>               scanStarted{false};

    // Touched only by poll(), i.e. only by the UI thread.
    UInt64 lastSeenSeq = 0;

    Void setError(const Str& msg)
    {
        {
            std::lock_guard<std::mutex> lock(mtx);
            errorMsg = msg;
        }
        state.store(LidarState::LIDAR_STATE_ERROR, std::memory_order_release);
    }

    Void run(Str port, Int32 baud);
};

// ---------------------------------------------------------------------------

Void LidarSource::Impl::run(Str port, Int32 baud)
{
    using namespace sl;

    state.store(LidarState::LIDAR_STATE_CONNECTING, std::memory_order_release);

    // COM10 and above are only reachable through the \\.\ device namespace, and
    // the prefix is harmless for the single-digit ports, so it always goes on.
    Str devPath = port;
    if(devPath.rfind("\\\\.\\", 0) != 0)
        devPath = "\\\\.\\" + port;

    ILidarDriver* drv = *createLidarDriver();
    if(!drv) {
        setError("out of memory creating the lidar driver");
        return;
    }

    // From here on every exit path must tear the driver down, so the body is
    // wrapped and cleanup happens once at the bottom. The channel is declared
    // out here because the SDK never takes ownership of it - unbindAndClose()
    // only close()s it - so this thread has to free it on every exit path.
    Bool      scanning = false;
    IChannel* channel  = nullptr;

    do {
        // Done before the driver touches the port, because the driver's own
        // report of this failure is unreliable - see probePort().
        Str portProblem = probePort(devPath, port);
        if(!portProblem.empty()) {
            setError(portProblem);
            break;
        }

        // Assigns the outer `channel` deliberately - declaring a new one here
        // would shadow it and leak the channel plus its three event handles.
        channel = *createSerialPortChannel(devPath.c_str(), (sl_u32)baud);
        if(!channel) {
            setError("cannot create a serial channel for " + port);
            break;
        }

        // connect() failing means the port itself would not open: it does not
        // exist, or another process holds it. Nothing to do with baud rate.
        if(SL_IS_FAIL(drv->connect(channel))) {
            setError("cannot open " + port + " (in use or missing)");
            break;
        }

        // The port opened but nothing answered the info request. The cable and
        // the port are fine; the framing is wrong or the device is unpowered.
        sl_lidar_response_device_info_t rawInfo;
        if(SL_IS_FAIL(drv->getDeviceInfo(rawInfo))) {
            setError("no response from device on " + port +
                      " (wrong baud rate?)");
            break;
        }

        LidarDeviceInfo di;
        di.model    = static_cast<Int32>(rawInfo.model);
        di.fwMajor = rawInfo.firmware_version >> 8;
        di.fwMinor = rawInfo.firmware_version & 0xFF;
        di.hwRev   = static_cast<Int32>(rawInfo.hardware_version);

        Char hex[33];
        for(Int32 i = 0; i < 16; ++i)
            std::snprintf(hex + i * 2, 3, "%02X", static_cast<unsigned>(rawInfo.serialnum[i]));
        di.serial = hex;

        sl_lidar_response_device_health_t health;
        if(SL_IS_OK(drv->getHealth(health)))
            di.health = static_cast<Int32>(health.status);

        {
            std::lock_guard<std::mutex> lock(mtx);
            devInfo = di;
        }

        // A hard health error means the unit will not produce usable data until
        // it is power cycled; starting the motor anyway just makes noise.
        if(di.health == SL_LIDAR_STATUS_ERROR) {
            setError("lidar reports an internal error; power cycle the device");
            break;
        }

        // Stop whatever the device was already doing before starting it.
        //
        // A previous session that was TERMINATED rather than closed - Task
        // Manager, a crash, Stop-Process - leaves the C1 spinning and streaming,
        // because no user-mode code runs on TerminateProcess and nothing ever
        // sent the stop. This makes the recovery deterministic instead of
        // relying on startScan to reset a device that is already mid-scan.
        drv->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(60));

        drv->setMotorSpeed();

        LidarScanMode mode;
        if(SL_IS_FAIL(drv->startScan(0, 1, 0, &mode))) {
            setError("failed to start scan on " + port);
            drv->setMotorSpeed(0);
            break;
        }
        scanning = true;

        // Record what the SDK actually negotiated. The C1 chooses the mode, and
        // without this the sample period and the mode's own range ceiling are
        // invisible to the operator.
        {
            LidarScanInfo si;
            si.modeId        = static_cast<Int32>(mode.id);
            si.usPerSample    = mode.us_per_sample;
            si.maxDistanceM = mode.max_distance;

            // scan_mode is a fixed 64-byte field, zero-padded but not
            // guaranteed terminated.
            Char name[sizeof(mode.scan_mode) + 1];
            std::memcpy(name, mode.scan_mode, sizeof(mode.scan_mode));
            name[sizeof(mode.scan_mode)] = '\0';
            si.mode = name;

            std::lock_guard<std::mutex> lock(mtx);
            scanInfo = si;
        }

        scanStart = std::chrono::steady_clock::now();
        scanStarted.store(true, std::memory_order_release);

        state.store(LidarState::LIDAR_STATE_SCANNING, std::memory_order_release);

        std::vector<sl_lidar_response_measurement_node_hq_t> nodes(MAX_NODES);

        // Reused across iterations so the steady state does no allocation once
        // the vector has grown to a revolution's worth of points.
        LidarFrame staging;
        staging.points.reserve(1024);

        Int32 consecutiveTimeouts = 0;

        Bool motorRunning = true;

        while(!quit.load(std::memory_order_acquire)) {
            // Motor pause. Checked here rather than around the grab because the
            // scan has to be stopped BEFORE the motor - killing the motor under
            // a running scan leaves the device streaming into a stopped rotor.
            const Bool want = motorOn.load(std::memory_order_acquire);
            if(want != motorRunning) {
                if(want) {
                    drv->setMotorSpeed();
                    LidarScanMode m2;
                    if(SL_IS_FAIL(drv->startScan(0, 1, 0, &m2))) {
                        setError("failed to restart scan on " + port);
                        break;
                    }
                    state.store(LidarState::LIDAR_STATE_SCANNING,
                                std::memory_order_release);
                } else {
                    drv->stop();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    drv->setMotorSpeed(0);
                    state.store(LidarState::LIDAR_STATE_IDLE,
                                std::memory_order_release);
                }
                motorRunning = want;
                consecutiveTimeouts = 0;
            }

            if(!motorRunning) {
                // Nothing to grab and nothing to poll. Sleeping rather than
                // spinning keeps a paused lidar off the CPU entirely.
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
                continue;
            }

            Size count = nodes.size();

            if(SL_IS_FAIL(drv->grabScanDataHq(nodes.data(), count, GRAB_TIMEOUT_MS))) {
                // A dropped revolution is normal - the device occasionally
                // misses its window. A long unbroken run of them is not: the
                // cable came out, or the device stopped talking. Surface that
                // instead of sitting in Scanning forever with a frozen view.
                statTimeouts.fetch_add(1, std::memory_order_relaxed);

                if(++consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS) {
                    setError("device stopped responding on " + port +
                              " (cable unplugged?)");
                    break;
                }
                continue;
            }
            consecutiveTimeouts = 0;

            drv->ascendScanData(nodes.data(), count);

            Float32 freq = 0.0f;
            drv->getFrequency(mode, nodes.data(), count, freq);

            staging.points.clear();
            staging.hz          = freq;
            staging.validCount = 0;
            staging.maxDistMm = 0.0f;

            for(Size i = 0; i < count; ++i) {
                LidarPoint p;
                // angle_z_q14 is q14 fixed point scaled so that 1.0 == 90 deg.
                p.angleDeg = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
                p.distMm   = nodes[i].dist_mm_q2 / 4.0f;
                p.quality   = static_cast<UInt8>((nodes[i].quality >>
                                        SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT));

                if(p.distMm > 0.0f) {
                    ++staging.validCount;
                    if(p.distMm > staging.maxDistMm)
                        staging.maxDistMm = p.distMm;
                }
                staging.points.push_back(p);
            }

            // Publish. The lock is held only for the swap and counter bump, so
            // a UI thread polling at 60Hz never waits on the scan conversion.
            {
                std::lock_guard<std::mutex> lock(mtx);
                std::swap(frame, staging);
                ++frameSeq;
            }

            statFrames.fetch_add(1, std::memory_order_relaxed);
            statPoints.fetch_add(count, std::memory_order_relaxed);
        }
    } while(false);

    // Order matters: stop the scan, give the device a moment to acknowledge it,
    // then cut the motor. Killing the motor first can leave the scan running.
    // Unconditional once the device has been opened. `scanning` used to be the
    // guard, but with a pause control the motor can be off while scanning is
    // true, or the scan stopped while the rotor coasts - and the cost of telling
    // an already-stopped device to stop is nothing, while the cost of skipping
    // it is a lidar spinning on a desk with no application attached.
    if(drv != nullptr) {
        drv->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        drv->setMotorSpeed(0);
    }
    static_cast<Void>(scanning);
    delete drv;

    // The driver only close()s the channel on teardown, it never frees it, so
    // ownership comes back here. Must follow `delete drv`, which still uses it.
    delete channel;

    // A clean shutdown returns to Idle; a failure keeps its Error state and the
    // message that explains it.
    if(state.load(std::memory_order_acquire) != LidarState::LIDAR_STATE_ERROR)
        state.store(LidarState::LIDAR_STATE_IDLE, std::memory_order_release);
}

// ---------------------------------------------------------------------------

LidarSource::LidarSource() : pimpl(new Impl) {}

LidarSource::~LidarSource()
{
    stop();
    delete pimpl;
}

Void LidarSource::start(const Str& port, Int32 baud)
{
    // A connect always spins up, whatever the last session left the flag on.
    pimpl->motorOn.store(true, std::memory_order_release);

    // Starting over an existing session would leak a thread and leave the motor
    // spinning, so the previous one is retired first.
    stop();

    {
        std::lock_guard<std::mutex> lock(pimpl->mtx);
        pimpl->errorMsg = Str();
        pimpl->devInfo  = LidarDeviceInfo();
        pimpl->scanInfo = LidarScanInfo();
        pimpl->frame     = LidarFrame();
        pimpl->frameSeq = 0;
    }
    pimpl->lastSeenSeq = 0;

    // Counters describe one session, so they restart with it.
    pimpl->statFrames.store(0, std::memory_order_relaxed);
    pimpl->statPoints.store(0, std::memory_order_relaxed);
    pimpl->statTimeouts.store(0, std::memory_order_relaxed);
    pimpl->scanStarted.store(false, std::memory_order_release);

    pimpl->quit.store(false, std::memory_order_release);
    pimpl->state.store(LidarState::LIDAR_STATE_CONNECTING, std::memory_order_release);
    pimpl->running.store(true, std::memory_order_release);

    Impl* impl = pimpl;
    pimpl->worker = std::thread([impl, port, baud] { impl->run(port, baud); });
}

Bool LidarSource::connected() const noexcept
{
    return pimpl->running.load(std::memory_order_acquire);
}

Void LidarSource::setMotorEnabled(Bool on)
{
    pimpl->motorOn.store(on, std::memory_order_release);
}

Bool LidarSource::motorEnabled() const noexcept
{
    return pimpl->motorOn.load(std::memory_order_acquire);
}

Void LidarSource::stop()
{
    // Safe when never started and safe to call twice: joinable() is false in
    // both cases and there is nothing to unwind.
    if(!pimpl->worker.joinable()) {
        pimpl->running.store(false, std::memory_order_release);
        return;
    }

    pimpl->quit.store(true, std::memory_order_release);

    // The worker performs the scan stop, the 200ms settle and the motor stop
    // before returning, so joining is exactly the "blocks until the motor is
    // off" guarantee the header promises. Worst case is one grab timeout.
    pimpl->worker.join();
    pimpl->running.store(false, std::memory_order_release);
}

LidarState LidarSource::state() const
{
    return pimpl->state.load(std::memory_order_acquire);
}

Str LidarSource::error() const
{
    std::lock_guard<std::mutex> lock(pimpl->mtx);
    return pimpl->errorMsg;
}

LidarDeviceInfo LidarSource::info() const
{
    std::lock_guard<std::mutex> lock(pimpl->mtx);
    return pimpl->devInfo;
}

LidarScanInfo LidarSource::scanInfo() const
{
    std::lock_guard<std::mutex> lock(pimpl->mtx);
    return pimpl->scanInfo;
}

LidarStats LidarSource::stats() const
{
    LidarStats s;
    s.frames   = pimpl->statFrames.load(std::memory_order_relaxed);
    s.points   = pimpl->statPoints.load(std::memory_order_relaxed);
    s.timeouts = pimpl->statTimeouts.load(std::memory_order_relaxed);

    if(pimpl->scanStarted.load(std::memory_order_acquire))
    {
        const auto now = std::chrono::steady_clock::now();
        s.uptimeS = std::chrono::duration<Float64>(now - pimpl->scanStart).count();
    }
    return s;
}

Bool LidarSource::poll(LidarFrame& out)
{
    std::lock_guard<std::mutex> lock(pimpl->mtx);

    // Frames land at ~10Hz while this is called at ~60Hz, so the overwhelmingly
    // common path is this comparison and an immediate return.
    if(pimpl->frameSeq == pimpl->lastSeenSeq)
        return false;

    out = pimpl->frame;
    pimpl->lastSeenSeq = pimpl->frameSeq;
    return true;
}

// ---------------------------------------------------------------------------

Str LidarSource::preferredPort()
{
    // HKLM\HARDWARE\DEVICEMAP\SERIALCOMM maps each driver's device name to the
    // COM port it owns. The CP210x driver registers as \Device\Silabser<N>,
    // which identifies the RPLIDAR's USB adapter without needing SetupAPI or
    // WMI - the other ports on a typical machine are Bluetooth links.
    Str found;

    HKEY key = nullptr;
    if(RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM",
                      0, KEY_READ, &key) != ERROR_SUCCESS)
        return found;

    for(DWORD i = 0; ; ++i)
    {
        Char  name[512];
        BYTE  data[512];
        DWORD nlen = static_cast<DWORD>(sizeof(name));
        DWORD dlen = static_cast<DWORD>(sizeof(data));
        DWORD type = 0;

        const LONG r = RegEnumValueA(key, i, name, &nlen, nullptr, &type, data, &dlen);
        if(r != ERROR_SUCCESS)
            break;
        if(type != REG_SZ || dlen == 0)
            continue;

        // Registry strings are not guaranteed to be terminated.
        if(dlen >= sizeof(data)) dlen = static_cast<DWORD>(sizeof(data)) - 1;
        data[dlen] = 0;

        Str dev(name, nlen);
        for(Char& c : dev) c = static_cast<Char>(std::tolower(static_cast<UInt8>(c)));

        if(dev.find("silabser") != Str::npos)
        {
            found = reinterpret_cast<const Char*>(data);
            break;
        }
    }

    RegCloseKey(key);
    return found;
}

std::vector<Str> LidarSource::listPorts()
{
    std::vector<Str> ports;

    // QueryDosDeviceA with a NULL device name returns every DOS device as a
    // packed run of NUL-terminated strings. It needs no extra import library
    // beyond kernel32, which is why it is preferred over SetupAPI here.
    try {
        DWORD cap = 8192;
        std::vector<Char> buf;

        for(Int32 attempt = 0; attempt < 6; ++attempt) {
            buf.assign(cap, '\0');
            DWORD n = QueryDosDeviceA(nullptr, buf.data(), cap);
            if(n != 0) {
                // Walk the packed list; a lone NUL terminates the whole block.
                const Char* p = buf.data();
                const Char* end = buf.data() + n;
                while(p < end && *p) {
                    Size len = std::strlen(p);

                    // Accept only COM followed entirely by digits, so entries
                    // such as "COMEDIA" or "COM" alone are rejected.
                    if(len > 3 &&
                        (p[0] == 'C' || p[0] == 'c') &&
                        (p[1] == 'O' || p[1] == 'o') &&
                        (p[2] == 'M' || p[2] == 'm')) {
                        Bool allDigits = true;
                        for(Size i = 3; i < len; ++i) {
                            if(p[i] < '0' || p[i] > '9') { allDigits = false; break; }
                        }
                        if(allDigits)
                            ports.push_back(Str(p, len));
                    }
                    p += len + 1;
                }
                break;
            }

            if(GetLastError() != ERROR_INSUFFICIENT_BUFFER)
                break;
            cap *= 2;
        }

        // Numeric order, so COM10 follows COM9 instead of preceding it.
        std::sort(ports.begin(), ports.end(),
                  [](const Str& a, const Str& b) {
                      long na = std::strtol(a.c_str() + 3, nullptr, 10);
                      long nb = std::strtol(b.c_str() + 3, nullptr, 10);
                      if(na != nb) return na < nb;
                      return a < b;
                  });
        ports.erase(std::unique(ports.begin(), ports.end()), ports.end());
    }
    catch(...) {
        // The header promises this never throws; a partial list beats an
        // exception escaping into the UI's frame loop.
    }

    return ports;
}
