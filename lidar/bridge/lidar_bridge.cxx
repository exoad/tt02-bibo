/*
 *  lidar_bridge - streams RPLIDAR scan frames as line-delimited text on stdout.
 *
 *  Built against Slamtec's rplidar_sdk driver (the same one ultra_simple uses),
 *  so there is no reimplementation of the serial protocol here.
 *
 *  Usage:  lidar_bridge <port> <baud>
 *          lidar_bridge \\.\COM7 460800
 *
 *  stdout protocol (one record per line, '\n' terminated, flushed per frame):
 *
 *    INFO <model> <fw_major>.<fw_minor> <hw_rev> <serial_hex>
 *    HEALTH <status>                      0=good 1=warning 2=error
 *    F <count> <freq_mHz> <a>,<d>,<q> ...  one scan revolution
 *        a = angle in centi-degrees (0..35999)
 *        d = distance in mm (0 = no return)
 *        q = quality (0..63)
 *    ERR <message>                        fatal, process then exits nonzero
 *
 *  stdin: a line beginning with 'q' requests a clean shutdown (stop scan,
 *  stop motor, exit 0). EOF on stdin does the same.
 *
 *  ---------------------------------------------------------------------------
 *  STATUS. This predates hub/, which now talks to the device directly through
 *  src/lidar_source.cxx. The bridge is kept because it is the only thing that
 *  can feed a NON-Windows consumer, and because a text protocol on stdout is
 *  the easiest possible thing to point a script at.
 *
 *  Written to docs/conventions.md like the rest of the tree: shared/shared.hxx
 *  aliases, Allman braces, `if(...)` with no space, camelCase functions. The
 *  Slamtec API keeps its own spelling at the boundary, which is the point of
 *  the boundary.
 */

#include "shared.hxx"

#include <csignal>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

namespace {

// Exit codes, so a caller can tell "wrong port" from "wrong baud" without
// parsing the ERR line.
constexpr Int32 EXIT_USAGE      = 2;
constexpr Int32 EXIT_NO_MEMORY  = 3;
constexpr Int32 EXIT_NO_PORT    = 4;
constexpr Int32 EXIT_NO_ANSWER  = 5;
constexpr Int32 EXIT_UNHEALTHY  = 6;
constexpr Int32 EXIT_NO_SCAN    = 7;

// One revolution of a C1 is ~500 points; 8192 is headroom for a faster device
// without making this a heap allocation.
constexpr Size MAX_NODES = 8192;

// Big buffer, explicitly flushed once per frame: avoids a syscall per point
// while still delivering each revolution to the consumer promptly.
constexpr Size OUT_BUF = 1u << 18;

volatile Bool stopRequested = false;

Void onSigint(Int32)
{
    stopRequested = true;
}

// Watches stdin so the consumer can ask for a clean shutdown. EOF stops us too,
// which means the bridge dies with its parent instead of leaving the motor on.
Void stdinWatch()
{
    Char line[64];
    while(std::fgets(line, sizeof(line), stdin) != nullptr)
    {
        if(line[0] == 'q' || line[0] == 'Q')
        {
            break;
        }
    }
    stopRequested = true;
}

Void fail(CharSeq message)
{
    std::printf("ERR %s\n", message);
    std::fflush(stdout);
}

} // namespace

Int32 main(Int32 argc, const Char* argv[])
{
    if(argc < 3)
    {
        std::fprintf(stderr, "usage: lidar_bridge <port> <baud>\n");
        return EXIT_USAGE;
    }

    CharSeq     port = argv[1];
    const sl_u32 baud = static_cast<sl_u32>(std::strtoul(argv[2], nullptr, 10));

    static Char outBuf[OUT_BUF];
    std::setvbuf(stdout, outBuf, _IOFBF, sizeof(outBuf));

    std::signal(SIGINT, onSigint);

    ILidarDriver* drv = *createLidarDriver();
    if(drv == nullptr)
    {
        fail("insufficient memory creating driver");
        return EXIT_NO_MEMORY;
    }

    IChannel* channel = *createSerialPortChannel(port, baud);
    if(SL_IS_FAIL(drv->connect(channel)))
    {
        fail("cannot open serial port (in use, or wrong port)");
        delete drv;
        return EXIT_NO_PORT;
    }

    sl_lidar_response_device_info_t info;
    if(SL_IS_FAIL(drv->getDeviceInfo(info)))
    {
        // Almost always a baud mismatch: the port opened but nothing answered.
        fail("no response from device (wrong baud rate?)");
        delete drv;
        return EXIT_NO_ANSWER;
    }

    std::printf("INFO %d %d.%02d %d ",
                static_cast<Int32>(info.model),
                info.firmware_version >> 8,
                info.firmware_version & 0xFF,
                static_cast<Int32>(info.hardware_version));

    for(Int32 i = 0; i < 16; ++i)
    {
        std::printf("%02X", info.serialnum[i]);
    }
    std::printf("\n");

    sl_lidar_response_device_health_t health;
    if(SL_IS_OK(drv->getHealth(health)))
    {
        std::printf("HEALTH %d\n", static_cast<Int32>(health.status));
        if(health.status == SL_LIDAR_STATUS_ERROR)
        {
            fail("lidar reports internal error; power cycle required");
            std::fflush(stdout);
            delete drv;
            return EXIT_UNHEALTHY;
        }
    }
    std::fflush(stdout);

    Thread(stdinWatch).detach();

    drv->setMotorSpeed();

    LidarScanMode mode;
    if(SL_IS_FAIL(drv->startScan(0, 1, 0, &mode)))
    {
        fail("failed to start scan");
        drv->setMotorSpeed(0);
        delete drv;
        return EXIT_NO_SCAN;
    }

    static sl_lidar_response_measurement_node_hq_t nodes[MAX_NODES];

    while(!stopRequested)
    {
        Size count = MAX_NODES;

        const sl_result res = drv->grabScanDataHq(nodes, count, 2000);
        if(SL_IS_FAIL(res))
        {
            if(stopRequested)
            {
                break;
            }
            continue;   // a timeout on a single revolution is not fatal
        }

        drv->ascendScanData(nodes, count);

        Float32 freq = 0.0f;
        drv->getFrequency(mode, nodes, count, freq);

        std::printf("F %u %u",
                    static_cast<UInt32>(count),
                    static_cast<UInt32>(freq * 1000.0f));

        for(Size i = 0; i < count; ++i)
        {
            // angle_z_q14 is a q14 fixed-point value scaled so 90 deg == 1.0
            UInt32 centi = static_cast<UInt32>((nodes[i].angle_z_q14 * 9000) >> 14);
            if(centi > 35999)
            {
                centi = 35999;
            }

            std::printf(" %u,%u,%u",
                        centi,
                        static_cast<UInt32>(nodes[i].dist_mm_q2 >> 2),
                        static_cast<UInt32>(nodes[i].quality
                            >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT));
        }

        std::printf("\n");
        std::fflush(stdout);
    }

    drv->stop();
    sleepMs(200);
    drv->setMotorSpeed(0);
    delete drv;
    return 0;
}
