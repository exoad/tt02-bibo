// Threaded wrapper around Slamtec's rplidar_sdk driver.
//
// The SDK's grabScanDataHq() blocks until a full revolution is ready, so it runs
// on a worker thread and publishes completed frames for the UI thread to poll.
#pragma once

#include "shared.hxx"

#include <cstdint>

struct LidarPoint
{
    Float32   angleDeg;   // 0..360, clockwise from the device's zero mark
    Float32   distMm;     // 0 means "no return in this direction"
    UInt8 quality;     // 0..63
};

struct LidarFrame
{
    Vec<LidarPoint> points;
    Float32 hz = 0.0f;   // rotation rate reported by the SDK
    Int32   validCount = 0;      // points with distMm > 0
    Float32 maxDistMm = 0.0f;
};

struct LidarDeviceInfo
{
    Int32         model = 0;
    Int32         fwMajor = 0;
    Int32         fwMinor = 0;
    Int32         hwRev = 0;
    Str serial;          // 32 hex chars
    Int32         health = -1;   // 0=good 1=warning 2=error, -1=unknown
};

// What the SDK negotiated at start-scan. The C1 picks a mode; these fields say
// which, and what timing it implies, which is otherwise invisible.
struct LidarScanInfo
{
    Str mode;                 // mode name reported by the SDK
    Int32         modeId = -1;
    Float32     usPerSample = 0.0f; // sample period -> theoretical sample rate
    Float32       maxDistanceM = 0.0f; // mode's own range ceiling
};

// Session counters, since start().
struct LidarStats
{
    UInt64 frames = 0;   // revolutions delivered to the UI
    UInt64 points = 0;   // measurements across all revolutions
    UInt32       timeouts = 0;   // grab timeouts (a dropped revolution)
    Float64             uptimeS = 0.0; // since start() succeeded
};

enum class LidarState
{
    LIDAR_STATE_IDLE,
    LIDAR_STATE_CONNECTING,
    LIDAR_STATE_SCANNING,

    // The cable came out. Deliberately NOT an error: unplugging a USB device is
    // something a person does on purpose, and answering it with a red banner and
    // a Win32 code trains them to ignore red banners. See devlink.hxx.
    LIDAR_STATE_UNPLUGGED,

    LIDAR_STATE_ERROR,
};

class LidarSource
{
public:
    LidarSource();
    ~LidarSource();

    LidarSource(const LidarSource&) = delete;
    LidarSource& operator=(const LidarSource&) = delete;

    // Non-blocking. Spins up the worker; watch state() for the outcome.
    // `port` is a bare name such as "COM7"; the \\.\ prefix is added internally.
    Void start(const Str& port, Int32 baud);

    // Blocks until the worker has stopped the scan and the motor.
    Void stop();

    // Spins the motor down without dropping the link, and back up again.
    //
    // Separate from stop() because they answer different questions. stop() means
    // "I am done with this device"; this means "stop making noise and wearing
    // the bearing, I am still here". The C1 cannot scan with the motor off, so
    // paused is a real state, not a display filter: no frames arrive while it
    // holds, and the last one stays on screen.
    // True while the worker holds the port, whether or not the motor is
    // spinning. Distinct from state()==SCANNING on purpose: a paused lidar is
    // still connected, and a UI that keys "am I attached to a device" off the
    // scanning state loses the control that would start it again.
    [[nodiscard]] Bool connected() const noexcept;

    Void setMotorEnabled(Bool on);
    [[nodiscard]] Bool motorEnabled() const noexcept;

    LidarState      state() const;
    Str     error() const;        // last error message, empty if none

    // The port this session was opened on, whether or not it is still there.
    // Kept so the UI can watch for the device coming back without having to
    // remember what it was talking to.
    [[nodiscard]] Str port() const;
    LidarDeviceInfo info() const;
    LidarScanInfo   scanInfo() const;    // valid once scanning starts
    LidarStats      stats() const;        // session counters, always readable

    // Copies the newest frame into `out`. Returns false when nothing new has
    // arrived since the previous call, in which case `out` is untouched.
    Bool poll(LidarFrame& out);

    // Serial ports present on the system, e.g. {"COM3","COM7"}. Never throws.
    static Vec<Str> listPorts();

    // Best guess at which port the lidar is on: the Silicon Labs CP210x USB
    // bridge, which is what every RPLIDAR USB adapter presents as. Returns an
    // empty string if no such port is present. Never throws.
    static Str preferredPort();

private:
    struct Impl;
    Impl* pimpl;
};
