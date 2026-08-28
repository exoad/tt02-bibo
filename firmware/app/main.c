/*
 * pico_debug - bring-up and debug firmware for the car's Pico 2 W.
 *
 * Phase 2 of the build order (see docs/conventions.md) is "Pico replaces receiver, USB
 * serial commands drive servo + ESC, watchdog". This is the step before that:
 * prove the toolchain, the USB link and the board itself, with the smallest
 * thing that can be wrong.
 *
 * Speaks newline-terminated ASCII over USB CDC. Every command answers with
 * exactly one line starting OK / ERR / INFO / PONG, so the host GUI can tell a
 * silent board from a confused one.
 *
 *   PING              -> PONG
 *   ID                -> INFO id ...
 *   STATUS            -> INFO status ...
 *   HELP              -> several INFO lines
 *   LED ON|OFF        -> OK led ...
 *   LED BLINK <hz>    -> OK led blink <hz>   (0 stops)
 *   BOOTSEL           -> reboots into the UF2 bootloader (no reply)
 *
 * NOTE ON THE LED: on Pico 2 W the user LED is NOT an RP2350 GPIO. It hangs off
 * the CYW43439 wireless chip, so it needs cyw43_arch_init() before it will do
 * anything. On a non-W Pico the same LED is plain GPIO 25. Getting this wrong
 * produces firmware that runs perfectly and never blinks.
 *
 * STYLE: this file calls the Pico SDK directly rather than going through
 * pico2w.h. That is deliberate - this is the firmware that has to answer "is the
 * board alive", and a wrapper between it and the silicon is one more thing that
 * can be the fault. The wrapper is for sketches. Everything that is OURS here
 * follows docs/conventions.md; every snake_case identifier below is an SDK name.
 */

/* stdio for printf/snprintf. It arrives transitively through pico/stdlib.h and
 * always has, which is exactly why it was missing here - a header you rely on
 * without naming is one that disappears the day the chain above it changes. */
/* printf only. String handling, parsing and case folding all live in text.h
 * now, so this is the last of libc that the console reaches for directly. */
#include <stdio.h>

/* The whole library. An application includes this and nothing else of ours -
 * see docs/conventions.md, and the style audit that enforces it. */
#include "../lib/tt02.h"


/* The sensor drivers. pico_debug is the image the hub talks to, so it is the
 * one that has to be able to answer "what is attached and what does it say" -
 * a sketch cannot, because the hub does not know what sketch is on the board. */

/* Not LINE_MAX: POSIX reserves that name and <limits.h> defines it on some
 * newlib configurations, which would make this a redefinition rather than a
 * declaration. */
#define LINE_CAP 128

#define BLINK_MAX_HZ 50.0f
#define IDLE_BLINK_HZ 0.5f

/* Three quick flashes at power-on, before any host has opened the port. */
#define HELLO_FLASHES 3
#define HELLO_FLASH_MS 80

/* 1 ms, which keeps the blink smooth while still returning promptly. */
#define POLL_TIMEOUT_US 1000

/* -------------------------------------------------------------- commands -- */

static CharSeq cyw43Word(Void)
{
    return ledPresent() ? "up" : "FAILED";
}

static Void printId(Void)
{
    Utf8 uid[24];
    boardId(uid, sizeof(uid));

    printf("INFO id board=%s sdk=%s built=%s %s uid=%s cyw43=%s\n",
           PICO_BOARD, PICO_SDK_VERSION_STRING, __DATE__, __TIME__,
           uid, cyw43Word());
}

static Void printStatus(Void)
{
    printf("INFO status up_ms=%u led=%s blink_hz=%.2f cyw43=%s\n",
           nowMs(),
           statusIsLit() ? "on" : "off",
           (Float64) statusRate(),
           cyw43Word());
}

static Void printHelp(Void)
{
    printf("INFO help PING - answers PONG\n");
    printf("INFO help ID - board, sdk, build time, unique id\n");
    printf("INFO help STATUS - uptime and led state\n");
    printf("INFO help LED ON|OFF - solid\n");
    printf("INFO help LED BLINK <hz> - 0 stops\n");
    printf("INFO help BOOTSEL - reboot into the UF2 bootloader\n");
    printf("INFO help SENSORS - what is attached\n");
    printf("INFO help SCAN - every I2C address that answers\n");
    printf("INFO help TOF - range in mm and a status\n");
    printf("INFO help TOF MODE SHORT|LONG - 1.3 m or 4 m\n");
    printf("INFO help DRIVE - servo and esc state\n");
    printf("INFO help SERVO <us>|CENTER - steering\n");
    printf("INFO help ESC ARM|DISARM|NEUTRAL|<us> - throttle\n");
    printf("INFO help STOP - neutral both, disarm the esc\n");
    printf("INFO help STEER <-1..1> - steer as a fraction of this car's travel\n");
    printf("INFO help SERVOTRIM <us> - move where centre is\n");
    printf("INFO help SERVO OFF - stop the pulse, servo goes limp\n");
    printf("INFO help SERVO ON - drive the steering again\n");
    printf("INFO help SERVOLIMITS <min> <max> - widen to find end stops\n");
    printf("INFO help ESCLIMITS <min> <max> - widen the throttle range\n");
}

/* Uppercase in place, so commands are accepted in any case. */
/* The " (cyw43 down, no effect)" tail on an LED reply. An OK that did nothing
 * has to say so, or a dead wireless chip looks like a dead command parser. */
static CharSeq ledCaveat(Void)
{
    return ledPresent() ? "" : " (cyw43 down, no effect)";
}

static Void handleLed(Utf8* arg)
{
    if(textEq(arg, "ON"))
    {
        statusBlink(0.0f);
        statusSolid(true);
        printf("OK led on%s\n", ledCaveat());
        return;
    }

    if(textEq(arg, "OFF"))
    {
        statusBlink(0.0f);
        statusSolid(false);
        printf("OK led off%s\n", ledCaveat());
        return;
    }

    if(textStarts(arg, "BLINK"))
    {
        /* textAfter rather than arg + 5: the offset and the word it skips are
         * written once, so renaming the command cannot leave a stale count. */
        Float32 hz = 0.0f;
        if(!textFloat(textAfter(arg, "BLINK "), &hz))
        {
            printf("ERR blink wants a rate in hz\n");
            return;
        }
        if(hz < 0.0f || hz > BLINK_MAX_HZ)
        {
            printf("ERR blink rate out of range (0-%.0f hz)\n",
                   (Float64) BLINK_MAX_HZ);
            return;
        }

        statusBlink(hz);
        if(hz == 0.0f)
        {
            statusSolid(false);
        }
        printf("OK led blink %.2f\n", (Float64) hz);
        return;
    }

    printf("ERR bad LED argument: %s\n", arg);
}

/* ================================================================ sensors ==
 *
 * The hub cannot see what is plugged into the Pico. It can only ask, so this
 * image answers - which is what makes the difference between a UI that says
 * "not wired" because nothing is wired and one that says it because nobody
 * ever checked.
 *
 * Detection is deliberately a fact rather than a guess: a sensor is present if
 * it acknowledges its address AND identifies itself. Something else living at
 * 0x29 is reported as absent rather than as a broken VL53L1X.
 */

#define SENSOR_SDA 4
#define SENSOR_SCL 5
#define SENSOR_HZ  400000u

/* I2C addresses worth naming in a scan. Anything else is reported as a bare
 * number, which is still useful - it says something is there. */
#define ADDR_SCAN_FIRST 0x08
#define ADDR_SCAN_LAST  0x77

static Bool i2cUp   = false;
static Vl53 tofFront;
static Bool tofUp   = false;

static Void sensorsOpen(Void)
{
    i2cUp = i2cOpen(SENSOR_SDA, SENSOR_SCL, SENSOR_HZ);
    if(!i2cUp)
    {
        return;
    }

    tofUp = vl53Open(&tofFront, SENSOR_SDA, VL53_ADDR_DEFAULT);
    if(tofUp)
    {
        vl53StartRanging(&tofFront);
    }
}

/*
 * One line listing what is attached. The hub parses this at connect, which is
 * how its sensor rows learn whether they are real.
 *
 * Shaped as key=value pairs so a reader that does not know about a sensor added
 * later ignores it rather than failing to parse the line.
 */
static Void printSensors(Void)
{
    printf("OK sensors i2c=%d tof=%d tof_addr=0x%02X\n",
           i2cUp ? 1 : 0, tofUp ? 1 : 0, VL53_ADDR_DEFAULT);
}

/* Every address that acknowledges. The same job as the standalone scanner
 * sketch, available over the link so the hub can offer it too. */
static Void printScan(Void)
{
    if(!i2cUp)
    {
        printf("ERR scan i2c not up\n");
        return;
    }

    Int32 found = 0;
    for(Int32 a = ADDR_SCAN_FIRST; a <= ADDR_SCAN_LAST; ++a)
    {
        if(i2cPresent(SENSOR_SDA, (UInt8) a))
        {
            printf("INFO scan 0x%02X\n", a);
            ++found;
        }
    }
    printf("OK scan %d\n", found);
}

/*
 * The current range.
 *
 * Reports the STATUS as well as the number, always. A distance that came with a
 * bad status is not a shorter distance - it is not a distance - and a host that
 * only got the number would have no way to know that.
 */
static Void printTof(Void)
{
    if(!tofUp)
    {
        printf("ERR tof absent\n");
        return;
    }

    if(vl53Ready(&tofFront))
    {
        const UInt16 mm = vl53Distance(&tofFront);
        const UInt8  st = vl53Status(&tofFront);

        /* The rates are read BEFORE the interrupt is cleared - they belong
         * to THIS measurement, and clearing first would hand back whatever
         * the next one produces.
         *
         * They are what turns "83 mm" from a number into a diagnosis. A
         * strong signal at a short distance means something really is that
         * close, which includes a protective film still on the lens; a weak
         * signal with a high ambient means the sensor is being blinded by
         * infrared in the room. */
        UInt16 sig = 0;
        UInt16 amb = 0;
        (Void) vl53Rates(&tofFront, &sig, &amb);

        vl53Clear(&tofFront);
        printf("OK tof %u %u %u %u\n", mm, st, sig, amb);
        return;
    }

    /* Not ready is not an error - the sensor takes tens of milliseconds per
     * measurement and the host is entitled to ask more often than that. */
    printf("OK tof busy\n");
}

static Void handleTofMode(Utf8* arg)
{
    if(!tofUp)
    {
        printf("ERR tof absent\n");
        return;
    }

    /* Stop, reconfigure, start.
     *
     * Changing the VCSEL period and the phase windows underneath a running
     * measurement leaves the sensor half-configured for as long as that
     * measurement lasts, and what it does with the result is undefined.
     * ST's own driver brackets it this way and so does this. */
    if(textEq(arg, "SHORT"))
    {
        vl53StopRanging(&tofFront);
        vl53SetMode(&tofFront, VL53_MODE_SHORT);
        vl53ClearInterruptAndStart(&tofFront);
        printf("OK tof mode short\n");
        return;
    }
    if(textEq(arg, "LONG"))
    {
        vl53StopRanging(&tofFront);
        vl53SetMode(&tofFront, VL53_MODE_LONG);
        vl53ClearInterruptAndStart(&tofFront);
        printf("OK tof mode long\n");
        return;
    }
    printf("ERR bad mode: %s\n", arg);
}

/* ================================================================== drive ==
 *
 * Console glue over lib/chassis. Everything about what is SAFE lives in the
 * module; everything here is about what to SAY.
 *
 * That split is the point. A console, a sketch and an autonomy loop each
 * carrying their own copy of "refuse throttle until armed" is three copies of a
 * rule, and the day one of them forgets is the day it matters. The module
 * refuses; this file reports the refusal.
 */

static Void printDrive(Void)
{
    const DriveState d = driveRead();
    printf("OK drive servo=%d servo_t=%d esc=%d esc_t=%d armed=%d "
           "servo_on=%d servo_c=%d steer_m=%d "
           "servo_min=%d servo_max=%d esc_min=%d esc_max=%d\n",
           d.servoUs, d.servoTargetUs, d.escUs, d.escTargetUs,
           d.escArmed ? 1 : 0, d.servoLive ? 1 : 0, d.centerUs, d.steerMilli,
           d.servoMinUs, d.servoMaxUs, d.escMinUs, d.escMaxUs);
}

static Void handleSteer(Utf8* arg)
{
    /* Rejected rather than defaulted: "STEER" with nothing after it is far more
     * likely to be a truncated command than a request to centre, and guessing
     * that it means zero would turn a typo into a movement. */
    if(arg[0] == '\0')
    {
        printf("ERR steer wants -1.0 to 1.0\n");
        return;
    }

    Float32 n = 0.0f;
    if(!textFloat(arg, &n))
    {
        printf("ERR steer wants -1.0 to 1.0\n");
        return;
    }

    driveSteer(n);
    printDrive();
}

static Void handleTrim(Utf8* arg)
{
    Int32 us = 0;
    if(!textInt(arg, &us))
    {
        const DriveState d = driveRead();
        printf("ERR trim wants microseconds, %d-%d\n", d.servoMinUs, d.servoMaxUs);
        return;
    }

    driveTrim(us);
    printf("INFO centre is now %d us\n", driveRead().centerUs);
    printDrive();
}

static Void handleLimits(Utf8* arg)
{
    Int32 lo = 0;
    Int32 hi = 0;
    if(!textTwoInts(arg, &lo, &hi))
    {
        printf("ERR limits wants <min> <max>\n");
        return;
    }
    if(!driveSetSteerLimits(lo, hi))
    {
        printf("ERR limits min must be below max\n");
        return;
    }
    printDrive();
}

static Void handleEscLimits(Utf8* arg)
{
    Int32 lo = 0;
    Int32 hi = 0;
    if(!textTwoInts(arg, &lo, &hi))
    {
        printf("ERR esclimits wants <min> <max>\n");
        return;
    }
    if(!driveSetThrottleLimits(lo, hi))
    {
        printf("ERR esclimits min must be below max\n");
        return;
    }
    printDrive();
}

static Void handleServo(Utf8* arg)
{
    /*
     * OFF stops the pulse train outright. This is the panic button: a servo
     * leaning on a frame does not need a better number, it needs to stop being
     * told to hold a position at all.
     */
    if(textEq(arg, "OFF"))
    {
        driveEngage(false);
        printf("INFO servo released - no pulse, no holding torque\n");
        printDrive();
        return;
    }

    if(textEq(arg, "ON"))
    {
        driveEngage(true);
        printf("INFO servo engaged - holding %d us\n", driveRead().servoTargetUs);
        printDrive();
        return;
    }

    if(textEq(arg, "CENTER") || textEq(arg, "CENTRE"))
    {
        driveCenter();
        printDrive();
        return;
    }

    Int32 us = 0;
    if(!textInt(arg, &us))
    {
        const DriveState d = driveRead();
        printf("ERR servo wants microseconds, %d-%d, or ON/OFF/CENTER\n",
               d.servoMinUs, d.servoMaxUs);
        return;
    }

    /* A position asked for while released is remembered, not obeyed. Engaging
     * is a separate, deliberate act - the same shape as arming the ESC. */
    const Bool wasLive = driveRead().servoLive;
    driveSteerUs(us);
    if(!wasLive)
    {
        printf("INFO servo is released - target stored, send SERVO ON\n");
    }
    printDrive();
}

static Void handleEsc(Utf8* arg)
{
    if(textEq(arg, "ARM"))
    {
        driveArm(true);
        printf("INFO esc armed - neutral held\n");
        printDrive();
        return;
    }
    if(textEq(arg, "DISARM"))
    {
        driveArm(false);
        printf("INFO esc disarmed\n");
        printDrive();
        return;
    }
    if(textEq(arg, "NEUTRAL"))
    {
        driveThrottleNeutral();
        printDrive();
        return;
    }

    Int32 us = 0;
    if(!textInt(arg, &us))
    {
        const DriveState d = driveRead();
        printf("ERR esc wants microseconds, %d-%d\n", d.escMinUs, d.escMaxUs);
        return;
    }

    /* The module owns the arming rule; this only reports it. */
    if(!driveThrottleUs(us))
    {
        printf("ERR esc not armed - send ESC ARM first\n");
        return;
    }
    printDrive();
}


static Void handleLine(Utf8* line)
{
    /* A terminal decides for itself what to put at the end of a line. Without
     * this, "PING\r" is not "PING" and a correctly typed command is refused. */
    if(textTrimEnd(line) == 0)
    {
        return;
    }

    textUpper(line);

    if(textEq(line, "PING"))
    {
        printf("PONG\n");
        return;
    }

    if(textEq(line, "ID"))
    {
        printId();
        return;
    }

    if(textEq(line, "STATUS"))
    {
        printStatus();
        return;
    }

    if(textEq(line, "HELP") || textEq(line, "?"))
    {
        printHelp();
        return;
    }

    if(textEq(line, "BOOTSEL"))
    {
        printf("INFO rebooting into bootloader\n");
        rebootToBootsel();          /* flushes, then does not return */
        return;
    }

    if(textStarts(line, "LED "))
    {
        handleLed(line + 4);
        return;
    }

    if(textEq(line, "STOP"))
    {
        driveStop();
        printf("OK stop\n");
        return;
    }

    if(textEq(line, "DRIVE"))
    {
        printDrive();
        return;
    }

    if(textStarts(line, "STEER "))
    {
        handleSteer(line + 6);
        return;
    }

    /* Bare STEER, so the "wants an argument" message is reachable. Without this
     * it falls through to "unknown command", which is true but unhelpful: the
     * command exists, the argument does not. */
    if(textEq(line, "STEER"))
    {
        handleSteer(line + 5);
        return;
    }

    if(textStarts(line, "SERVOTRIM "))
    {
        handleTrim(line + 10);
        return;
    }

    if(textStarts(line, "SERVOLIMITS "))
    {
        handleLimits(line + 12);
        return;
    }

    if(textStarts(line, "ESCLIMITS "))
    {
        handleEscLimits(line + 10);
        return;
    }

    if(textStarts(line, "SERVO "))
    {
        handleServo(line + 6);
        return;
    }

    if(textStarts(line, "ESC "))
    {
        handleEsc(line + 4);
        return;
    }

    if(textEq(line, "SENSORS"))
    {
        printSensors();
        return;
    }

    if(textEq(line, "SCAN"))
    {
        printScan();
        return;
    }

    if(textEq(line, "TOF"))
    {
        printTof();
        return;
    }

    if(textStarts(line, "TOF MODE "))
    {
        handleTofMode(line + 9);
        return;
    }

    printf("ERR unknown command: %s\n", line);
}

/* ------------------------------------------------------------------ main -- */

Int32 main(Void)
{
    serialOpen();

    /* Sensors come up at boot so SENSORS and TOF can answer immediately. A
     * missing sensor is not a failure here - it is the answer. */
    sensorsOpen();

    /* The ESC to neutral, the steering RELEASED. See chassis.h for why those
     * are different answers. */
    driveOpen();

    /* Brings up the CYW43439, which is what the LED hangs off. Slow (hundreds
     * of ms) and able to fail, so the result is REPORTED rather than assumed -
     * a board that answers PING but says cyw43=FAILED is a very different
     * problem from a board that is silent. statusOpen() remembers it; every
     * later call is a no-op rather than a crash. */
    statusOpen();

    /* Visible proof of life the moment power is applied, before any host could
     * be listening: three quick flashes, then a slow idle heartbeat. */
    statusHello(HELLO_FLASHES, HELLO_FLASH_MS);
    statusBlink(IDLE_BLINK_HZ);

    Utf8 line[LINE_CAP];
    Size len = 0;
    Bool announced = false;

    for(;;)
    {
        statusTick();

        /* Walks the servo and ESC toward their targets, a few microseconds at a
         * time. Nothing jumps: a slider dragged end to end produces a sweep
         * rather than a step. */
        drivePump();

        /* Anything written before the host opens the port is discarded, so the
         * banner waits for a connection rather than being lost. */
        const Bool host = serialHostPresent();
        if(!announced && host)
        {
            printf("INFO ready %s sdk=%s - type HELP\n",
                   PICO_BOARD, PICO_SDK_VERSION_STRING);
            announced = true;
        }
        if(announced && !host)
        {
            announced = false;      /* re-announce on the next connection */
        }

        const Int32 c = serialReadChar(POLL_TIMEOUT_US);
        if(c == SERIAL_NONE)
        {
            continue;
        }

        if(c == '\n' || c == '\r')
        {
            line[len] = '\0';
            handleLine(line);
            len = 0;
        }
        else if(len + 1 < LINE_CAP)
        {
            line[len++] = (Utf8) c;
        }
        else
        {
            /* Overlong line: drop it rather than silently truncating into a
             * command that means something else. */
            len = 0;
            printf("ERR line too long\n");
        }
    }
}
