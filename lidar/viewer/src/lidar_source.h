// Threaded wrapper around Slamtec's rplidar_sdk driver.
//
// The SDK's grabScanDataHq() blocks until a full revolution is ready, so it runs
// on a worker thread and publishes completed frames for the UI thread to poll.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct LidarPoint
{
    float   angle_deg;   // 0..360, clockwise from the device's zero mark
    float   dist_mm;     // 0 means "no return in this direction"
    uint8_t quality;     // 0..63
};

struct LidarFrame
{
    std::vector<LidarPoint> points;
    float hz           = 0.0f;   // rotation rate reported by the SDK
    int   valid_count  = 0;      // points with dist_mm > 0
    float max_dist_mm  = 0.0f;
};

struct LidarDeviceInfo
{
    int         model    = 0;
    int         fw_major = 0;
    int         fw_minor = 0;
    int         hw_rev   = 0;
    std::string serial;          // 32 hex chars
    int         health   = -1;   // 0=good 1=warning 2=error, -1=unknown
};

enum class LidarState
{
    Idle,
    Connecting,
    Scanning,
    Error,
};

class LidarSource
{
public:
    LidarSource();
    ~LidarSource();

    LidarSource(const LidarSource&)            = delete;
    LidarSource& operator=(const LidarSource&) = delete;

    // Non-blocking. Spins up the worker; watch state() for the outcome.
    // `port` is a bare name such as "COM7"; the \\.\ prefix is added internally.
    void start(const std::string& port, int baud);

    // Blocks until the worker has stopped the scan and the motor.
    void stop();

    LidarState      state() const;
    std::string     error() const;        // last error message, empty if none
    LidarDeviceInfo info() const;

    // Copies the newest frame into `out`. Returns false when nothing new has
    // arrived since the previous call, in which case `out` is untouched.
    bool poll(LidarFrame& out);

    // Serial ports present on the system, e.g. {"COM3","COM7"}. Never throws.
    static std::vector<std::string> list_ports();

    // Best guess at which port the lidar is on: the Silicon Labs CP210x USB
    // bridge, which is what every RPLIDAR USB adapter presents as. Returns an
    // empty string if no such port is present. Never throws.
    static std::string preferred_port();

private:
    struct Impl;
    Impl* impl_;
};
