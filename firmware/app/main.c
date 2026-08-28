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

/* Whether the lamp came up, whatever the lamp happens to BE on this board. */
static CharSeq lampWord(Void)
{
    return ledPresent() ? "yes" : "no";
}

/*
 * The trailing " cyw43=up" field - emitted ONLY on a board that has the chip.
 *
 * Absent, not false, on the plain Pico 2. There is no wireless chip in that
 * package to be down, and the hub reads this field to light a chip indicator:
 * reporting cyw43=FAILED there would paint a hardware fault on a board that is
 * working perfectly. A field that does not apply is one you leave out, and the
 * hub already treats "not mentioned" as "nothing to say".
 */
static CharSeq cyw43Field(Void)
{
#if defined(CYW43_WL_GPIO_LED_PIN)
    return ledPresent() ? " cyw43=up" : " cyw43=FAILED";
#else
    return "";
#endif
}

static Void printId(CharSeq arg)
{
    (Void) arg;

    Utf8 uid[24];
    boardId(uid, sizeof(uid));

    serialPrintf("INFO id board=%s sdk=%s built=%s %s uid=%s lamp=%s lamp_up=%s%s\n",
           PICO_BOARD, PICO_SDK_VERSION_STRING, __DATE__, __TIME__,
           uid, ledBackend(), lampWord(), cyw43Field());
}

static Void printStatus(CharSeq arg)
{
    (Void) arg;

    /* board= is on THIS line as well as on ID, because STATUS is the line
     * anything polls. The hub asks for it every couple of seconds and asks for
     * ID only when a person clicks something - so a field that appears solely
     * on ID is a field the hub almost never has. There are two boards in this
     * project now and telling them apart is worth six characters a poll. */
    serialPrintf("INFO status up_ms=%u board=%s led=%s blink_hz=%.2f lamp=%s lamp_up=%s%s\n",
           nowMs(),
           PICO_BOARD,
           statusIsLit() ? "on" : "off",
           (Float64) statusRate(),
           ledBackend(), lampWord(), cyw43Field());
}

/* Uppercase in place, so commands are accepted in any case. */
/* The tail on an LED reply when there is nothing to light. An OK that did
 * nothing has to say so, or a dead lamp looks like a dead command parser - and
 * the two boards fail this differently, so they say it differently. */
static CharSeq ledCaveat(Void)
{
    if(ledPresent())
    {
        return "";
    }
#if defined(CYW43_WL_GPIO_LED_PIN)
    return " (cyw43 down, no effect)";
#else
    return " (no lamp on this board, no effect)";
#endif
}

static Void handleLed(CharSeq arg)
{
    if(textEq(arg, "ON"))
    {
        statusBlink(0.0f);
        statusSolid(true);
        serialPrintf("OK led on%s\n", ledCaveat());
        return;
    }

    if(textEq(arg, "OFF"))
    {
        statusBlink(0.0f);
        statusSolid(false);
        serialPrintf("OK led off%s\n", ledCaveat());
        return;
    }

    if(textStarts(arg, "BLINK"))
    {
        /* textAfter rather than arg + 5: the offset and the word it skips are
         * written once, so renaming the command cannot leave a stale count. */
        Float32 hz = 0.0f;
        if(!textFloat(textAfter(arg, "BLINK "), &hz))
        {
            serialPrintf("ERR blink wants a rate in hz\n");
            return;
        }
        if(hz < 0.0f || hz > BLINK_MAX_HZ)
        {
            serialPrintf("ERR blink rate out of range (0-%.0f hz)\n",
                   (Float64) BLINK_MAX_HZ);
            return;
        }

        statusBlink(hz);
        if(hz == 0.0f)
        {
            statusSolid(false);
        }
        serialPrintf("OK led blink %.2f\n", (Float64) hz);
        return;
    }

    serialPrintf("ERR bad LED argument: %s\n", arg);
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
static Void printSensors(CharSeq arg)
{
    (Void) arg;

    serialPrintf("OK sensors i2c=%d tof=%d tof_addr=0x%02X\n",
           i2cUp ? 1 : 0, tofUp ? 1 : 0, VL53_ADDR_DEFAULT);
}

/* Every address that acknowledges. The same job as the standalone scanner
 * sketch, available over the link so the hub can offer it too. */
static Void printScan(CharSeq arg)
{
    (Void) arg;

    if(!i2cUp)
    {
        serialPrintf("ERR scan i2c not up\n");
        return;
    }

    Int32 found = 0;
    for(Int32 a = ADDR_SCAN_FIRST; a <= ADDR_SCAN_LAST; ++a)
    {
        if(i2cPresent(SENSOR_SDA, (UInt8) a))
        {
            serialPrintf("INFO scan 0x%02X\n", a);
            ++found;
        }
    }
    serialPrintf("OK scan %d\n", found);
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
        serialPrintf("ERR tof absent\n");
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
        serialPrintf("OK tof %u %u %u %u\n", mm, st, sig, amb);
        return;
    }

    /* Not ready is not an error - the sensor takes tens of milliseconds per
     * measurement and the host is entitled to ask more often than that. */
    serialPrintf("OK tof busy\n");
}

static Void handleTofMode(CharSeq arg)
{
    if(!tofUp)
    {
        serialPrintf("ERR tof absent\n");
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
        serialPrintf("OK tof mode short\n");
        return;
    }
    if(textEq(arg, "LONG"))
    {
        vl53StopRanging(&tofFront);
        vl53SetMode(&tofFront, VL53_MODE_LONG);
        vl53ClearInterruptAndStart(&tofFront);
        serialPrintf("OK tof mode long\n");
        return;
    }
    serialPrintf("ERR bad mode: %s\n", arg);
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
    serialPrintf("OK drive servo=%d servo_t=%d esc=%d esc_t=%d armed=%d "
           "servo_on=%d servo_c=%d steer_m=%d steer_now=%d slew=%d "
           "servo_min=%d servo_max=%d esc_min=%d esc_max=%d\n",
           d.servoUs, d.servoTargetUs, d.escUs, d.escTargetUs,
           d.escArmed ? 1 : 0, d.servoLive ? 1 : 0, d.centerUs, d.steerMilli,
           d.steerNowMilli,
           d.slewStepUs,
           d.servoMinUs, d.servoMaxUs, d.escMinUs, d.escMaxUs);
}

static Void handleSteer(CharSeq arg)
{
    /* Rejected rather than defaulted: "STEER" with nothing after it is far more
     * likely to be a truncated command than a request to centre, and guessing
     * that it means zero would turn a typo into a movement. */
    if(arg[0] == '\0')
    {
        serialPrintf("ERR steer wants -1.0 to 1.0\n");
        return;
    }

    Float32 n = 0.0f;
    if(!textFloat(arg, &n))
    {
        serialPrintf("ERR steer wants -1.0 to 1.0\n");
        return;
    }

    driveSteer(n);
    printDrive();
}

static Void handleSlew(CharSeq arg)
{
    Int32 us = 0;
    if(!textInt(arg, &us) || !driveSetSlew(us))
    {
        serialPrintf("ERR slew wants microseconds per tick, %d-%d\n",
                     SLEW_MIN_STEP, SLEW_MAX_STEP);
        return;
    }

    /* Reported in a unit a person thinks in as well as the one the firmware
     * uses. "8 us per tick" is a number nobody has intuition about; "400 us/s,
     * full travel 1100 ms" is the fact that decides whether it is fast enough
     * to steer around something. */
    const DriveState d = driveRead();
    const Int32 perSec = d.slewStepUs * (1000 / SLEW_TICK_MS);
    serialPrintf("INFO slew %d us/tick = %d us/s, full travel %d ms\n",
                 d.slewStepUs, perSec,
                 (perSec > 0) ? (((d.servoMaxUs - d.servoMinUs) * 1000) / perSec)
                              : 0);
    printDrive();
}

static Void handleTrim(CharSeq arg)
{
    Int32 us = 0;
    if(!textInt(arg, &us))
    {
        const DriveState d = driveRead();
        serialPrintf("ERR trim wants microseconds, %d-%d\n", d.servoMinUs, d.servoMaxUs);
        return;
    }

    driveTrim(us);
    serialPrintf("INFO centre is now %d us\n", driveRead().centerUs);
    printDrive();
}

/*
 * SERVOLIMITS and ESCLIMITS, which are one command with two setters.
 *
 * They were written out twice, identically apart from the word in the error
 * messages and the function called - and the two copies had already drifted
 * once, because the ordering bug fixed in driveSetSteerLimits had to be
 * remembered a second time for the throttle.
 */
static Void limitsCommand(CharSeq arg, CharSeq name, Bool (*set)(Int32, Int32))
{
    Int32 lo = 0;
    Int32 hi = 0;
    if(!textTwoInts(arg, &lo, &hi))
    {
        serialPrintf("ERR %s wants <min> <max>\n", name);
        return;
    }
    if(!set(lo, hi))
    {
        serialPrintf("ERR %s min must be below max\n", name);
        return;
    }
    printDrive();
}

static Void handleLimits(CharSeq arg)
{
    limitsCommand(arg, "servolimits", driveSetSteerLimits);
}

static Void handleEscLimits(CharSeq arg)
{
    limitsCommand(arg, "esclimits", driveSetThrottleLimits);
}

static Void handleServo(CharSeq arg)
{
    /*
     * OFF stops the pulse train outright. This is the panic button: a servo
     * leaning on a frame does not need a better number, it needs to stop being
     * told to hold a position at all.
     */
    if(textEq(arg, "OFF"))
    {
        driveEngage(false);
        serialPrintf("INFO servo released - no pulse, no holding torque\n");
        printDrive();
        return;
    }

    if(textEq(arg, "ON"))
    {
        driveEngage(true);
        serialPrintf("INFO servo engaged - holding %d us\n", driveRead().servoTargetUs);
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
        serialPrintf("ERR servo wants microseconds, %d-%d, or ON/OFF/CENTER\n",
               d.servoMinUs, d.servoMaxUs);
        return;
    }

    /* A position asked for while released is remembered, not obeyed. Engaging
     * is a separate, deliberate act - the same shape as arming the ESC. */
    const Bool wasLive = driveRead().servoLive;
    driveSteerUs(us);
    if(!wasLive)
    {
        serialPrintf("INFO servo is released - target stored, send SERVO ON\n");
    }
    printDrive();
}

static Void handleEsc(CharSeq arg)
{
    if(textEq(arg, "ARM"))
    {
        driveArm(true);
        serialPrintf("INFO esc armed - neutral held\n");
        printDrive();
        return;
    }
    if(textEq(arg, "DISARM"))
    {
        driveArm(false);
        serialPrintf("INFO esc disarmed\n");
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
        serialPrintf("ERR esc wants microseconds, %d-%d\n", d.escMinUs, d.escMaxUs);
        return;
    }

    /* The module owns the arming rule; this only reports it. */
    if(!driveThrottleUs(us))
    {
        serialPrintf("ERR esc not armed - send ESC ARM first\n");
        return;
    }
    printDrive();
}


/* ---------------------------------------------------------------------------
 * LIGHTS - the indicator scaffolding. TEMPORARY, see lib/lights.h.
 *
 * Reports which way the car thinks it is turning and whether the lamp is lit
 * this instant. The hub polls it to draw the pair in the Drive view, which is
 * the only reason the "lit" half is reported at all - a blink you can see on
 * screen at the same rate as the one on the bench is how you tell the rule is
 * running rather than the LED merely being on.
 * ------------------------------------------------------------------------- */
/* The lamp names, in Lamp order, so the reply reads the way the model does. */
static CharSeq LAMP_NAME[LAMP_COUNT] =
{
    "headL", "headR", "tailL", "tailR",
    "indFL", "indFR", "indRL", "indRR",
    "revL",  "revR"
};

/*
 * Every lamp in the model is reported, bound to a pin or not.
 *
 * A lamp with no LED on it still has a correct answer, and printing it is what
 * makes wiring the next one a matter of checking a number that was already
 * there rather than trusting that a rule nobody has ever seen run is right.
 */
static Void printLights(CharSeq arg)
{
    (Void) arg;

    const LampSet   s = lightsRead();
    const LightTurn t = lightsSide();
    const Int32     f = lightsForcedLamp();

    /* ONE line, not one per lamp.
     *
     * The hub polls this every 120 ms to draw the lamps, and eight extra lines
     * a poll is seventy-five lines a second of serial traffic to say what fits
     * in one. levels[] and pins[] are in Lamp order, which is the order
     * LAMP_NAME is in and the order the model declares them. */
    serialPrintf("OK lights on=%d turn=%s forced=%s off_us=%d"
                 " levels=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u"
                 " pins=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                 lightsEnabled() ? 1 : 0,
                 (t == LIGHT_TURN_LEFT)  ? "left"
                     : (t == LIGHT_TURN_RIGHT)  ? "right"
                     : (t == LIGHT_TURN_HAZARD) ? "hazard" : "off",
                 (f == LAMP_COUNT) ? "no" : LAMP_NAME[f],
                 lightsOffThreshold(),
                 (UInt32) s.level[0], (UInt32) s.level[1],
                 (UInt32) s.level[2], (UInt32) s.level[3],
                 (UInt32) s.level[4], (UInt32) s.level[5],
                 (UInt32) s.level[6], (UInt32) s.level[7],
                 (UInt32) s.level[8], (UInt32) s.level[9],
                 lightPin[0], lightPin[1], lightPin[2], lightPin[3],
                 lightPin[4], lightPin[5], lightPin[6], lightPin[7],
                 lightPin[8], lightPin[9]);
}

/*
 * LIGHTS [ON|OFF|AUTO|<lamp>]
 *
 * A lamp NAME forces that one lamp on and everything else off, so an LED and
 * its wiring can be checked without touching the ESC or the steering. AUTO
 * hands it back to the rules.
 */
static Void handleLights(CharSeq arg)
{
    if(textEq(arg, "ON"))
    {
        lightsEnable(true);
        printLights(arg);
        return;
    }
    if(textEq(arg, "OFF"))
    {
        lightsEnable(false);
        printLights(arg);
        return;
    }
    if(textEq(arg, "AUTO"))
    {
        lightsForceLamp(LAMP_COUNT);
        printLights(arg);
        return;
    }

    /* LIGHTS OFFAT <us> - how far past idle counts as being driven. */
    if(textStarts(arg, "OFFAT"))
    {
        Int32 us = 0;
        if(!textInt(textAfter(arg, "OFFAT "), &us)
           || !lightsSetOffThreshold(us))
        {
            serialPrintf("ERR lights offat wants %d-%d us past idle\n",
                         LIGHT_OFF_US_MIN, LIGHT_OFF_US_MAX);
            return;
        }
        printLights(arg);
        return;
    }
    if(arg[0] == '\0')
    {
        printLights(arg);
        return;
    }

    /* Matched case-insensitively because handleLine has already uppercased the
     * whole line, and the table above is spelled the way the model spells it. */
    for(Int32 i = 0; i < LAMP_COUNT; ++i)
    {
        Utf8 up[12];
        Size n = 0;
        while(LAMP_NAME[i][n] != '\0' && n < sizeof(up) - 1)
        {
            up[n] = LAMP_NAME[i][n];
            ++n;
        }
        up[n] = '\0';
        textUpper(up);

        if(textEq(arg, up))
        {
            lightsForceLamp(i);
            printLights(arg);
            return;
        }
    }

    serialPrintf("ERR lights wants ON, OFF, AUTO, OFFAT <us>, a lamp name,"
                 " or nothing\n");
}

/* ---- the command table ---------------------------------------------------
 *
 * One row per command; the dispatcher and HELP both read it.
 *
 * They used to be two lists. A chain of twenty if(textStarts(...)) blocks with
 * the argument offset written out by hand - line + 10 for "SERVOTRIM ",
 * line + 12 for "SERVOLIMITS " - and a printHelp() that spelled the same
 * commands out again in prose. Two lists of the same thing drift: the offsets
 * were a silent hazard, because miscounting one makes the argument parser read
 * from the middle of the command word and the command merely stops working.
 *
 * Matching is by WHOLE WORD, so the order of the rows means nothing. The old
 * chain worked only because SERVOTRIM and SERVOLIMITS happened to be tested
 * before SERVO, which textStarts() would otherwise have matched first.
 *
 * `usage` is what may follow the name and `what` is one line about it; HELP
 * prints them and nothing else has to be kept in step.
 */
typedef Void (*CmdRun)(CharSeq arg);

typedef struct
{
    CharSeq name;
    CharSeq usage;
    CharSeq what;
    CmdRun  run;
} Command;

/* Defined below the table, which it walks. */
static Void printHelp(CharSeq arg);

/* TOF's subcommand, kept here rather than as a second row: "TOF MODE LONG" is
 * an argument to TOF, not a command called "TOF MODE", and whole-word matching
 * would hand the whole thing to TOF anyway. */
static Void cmdTof(CharSeq arg)
{
    CharSeq mode = textWord(arg, "MODE");
    if(mode != NULL)
    {
        handleTofMode(mode);
        return;
    }
    printTof();
}

static Void cmdPing(CharSeq arg)
{
    (Void) arg;
    serialPrintf("PONG\n");
}

static Void cmdDrive(CharSeq arg)
{
    (Void) arg;
    printDrive();
}

static Void cmdStop(CharSeq arg)
{
    (Void) arg;
    driveStop();
    serialPrintf("OK stop\n");
}

static Void cmdBootsel(CharSeq arg)
{
    (Void) arg;
    serialPrintf("INFO rebooting into bootloader\n");
    rebootToBootsel();          /* flushes, then does not return */
}

static const Command COMMANDS[] =
{
    { "PING",        "",                        "answers PONG",                             cmdPing },
    { "ID",          "",                        "board, sdk, build time, unique id",        printId },
    { "STATUS",      "",                        "uptime and led state",                     printStatus },
    { "HELP",        "",                        "this list",                                printHelp },
    { "BOOTSEL",     "",                        "reboot into the UF2 bootloader",           cmdBootsel },
    { "LED",         " ON|OFF|BLINK <hz>",      "solid, or blink; 0 stops",                 handleLed },

    { "SENSORS",     "",                        "what is attached",                         printSensors },
    { "SCAN",        "",                        "every I2C address that answers",           printScan },
    { "TOF",         " [MODE SHORT|LONG]",      "range in mm; the mode is 1.3 m or 4 m",    cmdTof },

    { "DRIVE",       "",                        "servo and esc state",                      cmdDrive },
    { "STOP",        "",                        "neutral both, disarm the esc",             cmdStop },
    { "STEER",       " <-1..1>",                "steer as a fraction of this car's travel", handleSteer },
    { "SLEW",        " <us>",                   "how fast outputs may move, per tick",      handleSlew },
    { "SERVO",       " <us>|ON|OFF|CENTER",     "steering; OFF stops the pulse, servo limp", handleServo },
    { "SERVOTRIM",   " <us>",                   "move where centre is",                     handleTrim },
    { "SERVOLIMITS", " <min> <max>",            "widen to find the real end stops",         handleLimits },
    { "ESC",         " ARM|DISARM|NEUTRAL|<us>", "throttle",                                handleEsc },
    { "ESCLIMITS",   " <min> <max>",            "widen the throttle range",                 handleEscLimits },

    /* TEMPORARY - the indicator scaffolding. Goes when GP15 is given back to
     * the wheel encoder. See lib/lights.h. */
    { "LIGHTS",      " [ON|OFF|AUTO|OFFAT <us>|<lamp>]", "the lamps, and what each is doing", handleLights },
};

static const Size COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

static Void printHelp(CharSeq arg)
{
    (Void) arg;
    for(Size i = 0; i < COMMAND_COUNT; ++i)
    {
        serialPrintf("INFO help %s%s - %s\n",
                     COMMANDS[i].name, COMMANDS[i].usage, COMMANDS[i].what);
    }
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

    /* "?" is HELP, and is not a row of its own: it would print as a command in
     * its own listing, which is one more thing than anybody wants to read. */
    if(textEq(line, "?"))
    {
        printHelp(line);
        return;
    }

    for(Size i = 0; i < COMMAND_COUNT; ++i)
    {
        CharSeq arg = textWord(line, COMMANDS[i].name);
        if(arg != NULL)
        {
            COMMANDS[i].run(arg);
            return;
        }
    }

    serialPrintf("ERR unknown command: %s\n", line);
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

    /* Brings up whatever this board's LED hangs off.
     *
     * On the Pico 2 W that is the CYW43439 - slow (hundreds of ms) and able to
     * fail, so the result is REPORTED rather than assumed: a board that answers
     * PING but says cyw43=FAILED is a very different problem from a board that
     * is silent. On the plain Pico 2 it is GP25 and cannot fail. Either way
     * statusOpen() remembers the outcome and every later call is a no-op
     * rather than a crash. */
    statusOpen();

    /* The indicator lamps. TEMPORARY scaffolding on borrowed pins - lib/lights.h
     * says which and why. Opened AFTER driveOpen() so that if the two ever
     * disagree about a pin, the drivetrain wins: a stray LED is a cosmetic
     * fault and a servo pin that is secretly an output is not. */
    lightsOpen();

    /* The threshold is a tuning that survives a reflash, so it lives in cal.h
     * and is handed to the module here. lights.h cannot reach for cal.h - the
     * layering forbids it, and rightly: the RULE is the same on any car and only
     * the number is this one's. */
    lightsSetOffThreshold(LIGHT_CAL_OFF_US);

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

        /* AFTER drivePump, and reading the ACTUAL servo and ESC output rather
         * than their targets. The slew limiter means the two differ for about a
         * second after every command: reading targets would light a lamp before
         * the car had done the thing the lamp is reporting. */
        {
            const DriveState d = driveRead();

            LightInput li;
            li.steerMilli = d.steerNowMilli;
            li.throttleUs = d.escUs;
            li.idleUs     = d.escMinUs;
            li.neutralUs  = DRIVE_NEUTRAL_US;
            li.armed      = d.escArmed;
            li.headOn     = false;   /* nothing the car knows implies darkness */

            lightsTick(&li);
        }

        /* Anything written before the host opens the port is discarded, so the
         * banner waits for a connection rather than being lost. */
        const Bool host = serialHostPresent();
        if(!announced && host)
        {
            serialPrintf("INFO ready %s sdk=%s - type HELP\n",
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
            serialPrintf("ERR line too long\n");
        }
    }
}
