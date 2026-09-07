// The lidar, wherever it is, as a stream of revolutions the UI thread can poll.
//
// Two places it can be, one worker for each, and nothing above poll() knows
// which is running:
//
//   - On a local serial port, through Slamtec's rplidar_sdk. The SDK's
//     grabScanDataHq() blocks until a full revolution is ready, so it runs on a
//     worker thread and publishes completed frames.
//
//   - On the Orange Pi (bibobox), whose scanfeed service holds the device and
//     streams one revolution per text line over TCP - the format is
//     firmware/pilot/src/scanwire.hxx, compiled into both programs. The SDK can
//     only talk to a serial port and the C1 rides on the board, so this is the
//     only way the laptop sees it in the field. That worker owns the socket the
//     same way the serial one owns the driver: the UI thread never writes it,
//     it flips a flag and the worker sends MOTOR 0/1 on its own tick.
//
// start() picks the worker from the target string: a ':' makes it a network
// target, because no COM port has one and every host:port does.
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

// What the pilot decided about the revolution it just sent - the scan feed's D
// line, firmware/pilot/src/scanwire.hxx. Only the pilot sends it; a bare
// scanfeed streams the same F lines and never decides, so on most sessions this
// never arrives at all. It is its own record rather than a field of LidarFrame
// because the two do not come in step: the pilot also decides on BLIND ticks
// when no revolution came, and a frame carries a copy of the scan while this is
// a dozen bytes the UI wants beside it, not inside it.
struct LidarDrive
{
    Str     mode;                // cruise slow stop reverse blind - the pilot's own word
    Int32   clearanceMm = 0;     // how far ahead the corridor is free
    Int32   hits = 0;            // corridor returns the clearance rests on
    Float32 steer = 0.0f;        // -1..1, left negative
    Float32 throttle = 0.0f;     // -1..1, negative is reverse
    Bool    stop = true;         // the pilot sent STOP rather than the two above
    TimePoint at{};              // when this side read it

    // A decision older than this is not one. The pilot has stopped and the scan
    // may well carry on - the feed is the same either way - so the age of the
    // last D line is the only thing that says whether anyone is driving.
    static constexpr Int32 STALE_MS = 1000;

    [[nodiscard]] Bool fresh() const noexcept
    {
        return at != TimePoint{} && elapsedMs(at) < static_cast<Float64>(STALE_MS);
    }
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
    // Revolutions the UI never got: a grab timeout on the serial path, or on the
    // feed a line that did not read whole (scanwire's KIND_BAD). One counter
    // because they are one thing to the person reading it - the device turned
    // and the picture did not update - and the UI already labels it that way.
    UInt32       timeouts = 0;
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
    //
    // The feed closing is the same state for the same reason: the board
    // rebooting or the hotspot dropping is not a fault in the hub, and it comes
    // back on its own.
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
    // `port` is a bare name such as "COM7" (the \\.\ prefix is added internally),
    // or a network target "host:port" - "bibobox.local:8011" - in which case
    // `baud` is ignored and the host may be a name: the hotspot's DHCP hands the
    // board a different address every outing and the name is what stays true.
    Void start(const Str& port, Int32 baud);

    // Whether `port` names the scan feed rather than a serial port. The one
    // rule, so the UI and start() cannot disagree about what a ':' means.
    [[nodiscard]] static Bool isFeedTarget(const Str& port) noexcept;

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

    // Asks; the worker does it on its next tick. motorEnabled() answers with what
    // the DEVICE is doing as far as this side knows - the state the serial worker
    // last set, or the last MOTOR line the board sent - not with what was asked,
    // so a button keyed off it flips when the rotor does and not before.
    Void setMotorEnabled(Bool on);
    [[nodiscard]] Bool motorEnabled() const noexcept;

    // True once this session got as far as the device: a port that answered the
    // info request, or a feed that accepted the connection. A session that ended
    // in ERROR before that failed to REACH the thing, which is the case worth
    // retrying quietly - the feed appears when the hotspot does - while a fault
    // after reaching it is one to leave on screen.
    [[nodiscard]] Bool reachedDevice() const noexcept;

    LidarState      state() const;
    Str     error() const;        // last error message, empty if none

    // The port this session was opened on, whether or not it is still there -
    // "COM7", or the "host:port" string for the feed. Kept so the UI can watch
    // for the device coming back without having to remember what it was talking
    // to.
    [[nodiscard]] Str port() const;
    LidarDeviceInfo info() const;

    // Valid once scanning starts on the serial path. The feed does not send
    // mode information, so over the network this stays at its defaults and the
    // UI shows "--" for each field.
    LidarScanInfo   scanInfo() const;
    LidarStats      stats() const;        // session counters, always readable

    // Copies the newest frame into `out`. Returns false when nothing new has
    // arrived since the previous call, in which case `out` is untouched.
    Bool poll(LidarFrame& out);

    // The pilot's newest decision, the same way: false and `out` untouched when
    // none has arrived since the previous call. `out.at` is when it was read,
    // and out.fresh() is the caller's test - a caller that keeps the last one
    // must let it go stale rather than draw a decision nobody is making.
    [[nodiscard]] Bool pollDrive(LidarDrive& out);

    // True once for each MOTOR 0 the board answered with MOTOR 1: the pilot
    // keeps the lidar while it drives, and a viewer does not get to switch it
    // off under the car. The worker has already put the wish back to match
    // the board's word, so nothing is resent; this is only so the UI can say
    // so, once.
    [[nodiscard]] Bool pollMotorRefused();

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
