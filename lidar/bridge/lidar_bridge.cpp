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
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <thread>
#include <chrono>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

static volatile bool g_stop = false;

static void on_sigint(int) { g_stop = true; }

// Watches stdin so the GUI can ask for a clean shutdown; EOF also stops us,
// which means the bridge dies with its parent instead of leaving the motor on.
static void stdin_watch()
{
    char line[64];
    while (fgets(line, sizeof(line), stdin)) {
        if (line[0] == 'q' || line[0] == 'Q') break;
    }
    g_stop = true;
}

static void fail(const char* msg)
{
    printf("ERR %s\n", msg);
    fflush(stdout);
}

int main(int argc, const char* argv[])
{
    if (argc < 3) {
        fprintf(stderr, "usage: lidar_bridge <port> <baud>\n");
        return 2;
    }

    const char* port = argv[1];
    sl_u32      baud = strtoul(argv[2], NULL, 10);

    // Big buffer, explicitly flushed once per frame: avoids per-point syscalls
    // while still delivering each revolution to the GUI promptly.
    static char outbuf[1 << 18];
    setvbuf(stdout, outbuf, _IOFBF, sizeof(outbuf));

    signal(SIGINT, on_sigint);

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        fail("insufficient memory creating driver");
        return 3;
    }

    IChannel* channel = *createSerialPortChannel(port, baud);
    if (SL_IS_FAIL(drv->connect(channel))) {
        fail("cannot open serial port (in use, or wrong port)");
        delete drv;
        return 4;
    }

    sl_lidar_response_device_info_t info;
    if (SL_IS_FAIL(drv->getDeviceInfo(info))) {
        // Almost always a baud mismatch: the port opened but nothing answered.
        fail("no response from device (wrong baud rate?)");
        delete drv;
        return 5;
    }

    printf("INFO %d %d.%02d %d ",
           (int)info.model,
           info.firmware_version >> 8,
           info.firmware_version & 0xFF,
           (int)info.hardware_version);
    for (int i = 0; i < 16; ++i) printf("%02X", info.serialnum[i]);
    printf("\n");

    sl_lidar_response_device_health_t health;
    if (SL_IS_OK(drv->getHealth(health))) {
        printf("HEALTH %d\n", (int)health.status);
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            fail("lidar reports internal error; power cycle required");
            fflush(stdout);
            delete drv;
            return 6;
        }
    }
    fflush(stdout);

    std::thread(stdin_watch).detach();

    drv->setMotorSpeed();

    LidarScanMode mode;
    if (SL_IS_FAIL(drv->startScan(0, 1, 0, &mode))) {
        fail("failed to start scan");
        drv->setMotorSpeed(0);
        delete drv;
        return 7;
    }

    static sl_lidar_response_measurement_node_hq_t nodes[8192];

    while (!g_stop) {
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        sl_result res = drv->grabScanDataHq(nodes, count, 2000);
        if (SL_IS_FAIL(res)) {
            if (g_stop) break;
            continue;   // timeout on a single revolution is not fatal
        }

        drv->ascendScanData(nodes, count);

        float freq = 0.f;
        drv->getFrequency(mode, nodes, count, freq);

        printf("F %u %u", (unsigned)count, (unsigned)(freq * 1000.f));
        for (size_t i = 0; i < count; ++i) {
            // angle_z_q14 is a q14 fixed-point value scaled so 90 deg == 1.0
            unsigned centi = (unsigned)((nodes[i].angle_z_q14 * 9000) >> 14);
            if (centi > 35999) centi = 35999;
            printf(" %u,%u,%u",
                   centi,
                   (unsigned)(nodes[i].dist_mm_q2 >> 2),
                   (unsigned)(nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT));
        }
        printf("\n");
        fflush(stdout);
    }

    drv->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    drv->setMotorSpeed(0);
    delete drv;
    return 0;
}
