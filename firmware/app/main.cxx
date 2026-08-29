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
#include "../lib/bibo.hxx"


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

/* ---- the deadman ----------------------------------------------------------
 *
 * How long the board will keep DRIVING with nothing heard from the host.
 *
 * There was no such limit until now, and the gap was not small: if the hub
 * crashed, hung, or was closed while the ESC was armed and moving, bibo::drive::pump()
 * went on writing the last throttle to the pin forever. On USB power the cable
 * coming out takes the board with it, which hid the problem; on the BEC it will
 * not. docs/conventions.md has described a 200 ms rule since the beginning and
 * nothing implemented it - the previous firmware, tt02_control, did have one,
 * so it was lost in the rewrite rather than never written.
 *
 * The host does not have to be a person at a keyboard, either: minimising the
 * hub stops its frame loop entirely, so anything relying on the PC to send a
 * stop stops running at exactly the moment it is needed.
 *
 * 400 ms rather than 200. The hub's own DRIVE poll runs at 250 ms and a
 * keyboard controller sends on key CHANGES rather than on a timer, so 200 would
 * trip on somebody holding W steadily and doing nothing wrong. 400 leaves one
 * missed poll of margin and is a few centimetres at the speeds this car does.
 *
 * It only applies while the car is actually being DRIVEN - armed AND commanded
 * above idle. Arming and then sitting still is not a hazard, and a timeout that
 * disarmed somebody for reading the screen is a timeout that gets switched off.
 */
#define DEADMAN_MS 400u

/* -------------------------------------------------------------- commands -- */

/* Whether the lamp came up, whatever the lamp happens to BE on this board. */
static CharSeq lampWord(Void)
{
    return bibo::led::present() ? "yes" : "no";
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
    return bibo::led::present() ? " cyw43=up" : " cyw43=FAILED";
#else
    return "";
#endif
}

static Void printId(CharSeq arg)
{
    static_cast<Void>(arg);

    Utf8 uid[24];
    bibo::board::id(uid, sizeof(uid));

    bibo::serial::printf("INFO id board=%s sdk=%s built=%s %s uid=%s lamp=%s lamp_up=%s%s\n",
           PICO_BOARD, PICO_SDK_VERSION_STRING, __DATE__, __TIME__,
           uid, bibo::led::backend(), lampWord(), cyw43Field());
}

static Void printStatus(CharSeq arg)
{
    static_cast<Void>(arg);

    /* board= is on THIS line as well as on ID, because STATUS is the line
     * anything polls. The hub asks for it every couple of seconds and asks for
     * ID only when a person clicks something - so a field that appears solely
     * on ID is a field the hub almost never has. There are two boards in this
     * project now and telling them apart is worth six characters a poll. */
    bibo::serial::printf("INFO status up_ms=%u board=%s led=%s blink_hz=%.2f lamp=%s lamp_up=%s%s\n",
           bibo::timing::nowMs(),
           PICO_BOARD,
           bibo::status::isLit() ? "on" : "off",
           static_cast<Float64>(bibo::status::rate()),
           bibo::led::backend(), lampWord(), cyw43Field());
}

/* Uppercase in place, so commands are accepted in any case. */
/* The tail on an LED reply when there is nothing to light. An OK that did
 * nothing has to say so, or a dead lamp looks like a dead command parser - and
 * the two boards fail this differently, so they say it differently. */
static CharSeq ledCaveat(Void)
{
    if(bibo::led::present())
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
    if(bibo::text::eq(arg, "ON"))
    {
        bibo::status::blink(0.0f);
        bibo::status::solid(true);
        bibo::serial::printf("OK led on%s\n", ledCaveat());
        return;
    }

    if(bibo::text::eq(arg, "OFF"))
    {
        bibo::status::blink(0.0f);
        bibo::status::solid(false);
        bibo::serial::printf("OK led off%s\n", ledCaveat());
        return;
    }

    if(bibo::text::starts(arg, "BLINK"))
    {
        /* bibo::text::after rather than arg + 5: the offset and the word it skips are
         * written once, so renaming the command cannot leave a stale count. */
        Float32 hz = 0.0f;
        if(!bibo::text::toFloat(bibo::text::after(arg, "BLINK "), &hz))
        {
            bibo::serial::printf("ERR blink wants a rate in hz\n");
            return;
        }
        if(hz < 0.0f || hz > BLINK_MAX_HZ)
        {
            bibo::serial::printf("ERR blink rate out of range (0-%.0f hz)\n",
                   static_cast<Float64>(BLINK_MAX_HZ));
            return;
        }

        bibo::status::blink(hz);
        if(hz == 0.0f)
        {
            bibo::status::solid(false);
        }
        bibo::serial::printf("OK led blink %.2f\n", static_cast<Float64>(hz));
        return;
    }

    bibo::serial::printf("ERR bad LED argument: %s\n", arg);
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

/* When the last command arrived, and whether the deadman has already fired.
 * The flag stops it stopping and reporting again every millisecond after. */
static UInt32 lastCmdMs      = 0;
static Bool   deadmanTripped = false;

/* ---- the line, as it was actually typed -----------------------------------
 *
 * handleLine() uppercases what it is given, because every command word in this
 * protocol is upper case and a person typing "ping" means PING.
 *
 * A Wi-Fi password does not work that way. Uppercasing it silently turns a
 * correct password into a wrong one, and the board then reports a failed join -
 * which reads as bad credentials, or bad range, or a bad radio, and never as
 * "the console changed what you typed".
 *
 * So the raw line is kept alongside. bibo::text::upper() rewrites in place and does not
 * change the LENGTH of anything, so an offset into the uppercased line is the
 * same offset into this one, and cmdRawArg is that pointer. Nothing but WIFI
 * needs it.
 */
static Utf8    rawLine[LINE_CAP];
static CharSeq cmdRawArg = "";

/* Which wireless state has already been announced, so a change is reported
 * once rather than every millisecond. */
static bibo::net::State netReported = bibo::net::STATE_ABSENT;

static Bool i2cUp   = false;
static bibo::tof::Vl53 tofFront;
static Bool tofUp   = false;

static Void sensorsOpen(Void)
{
    i2cUp = bibo::i2c::open(SENSOR_SDA, SENSOR_SCL, SENSOR_HZ);
    if(!i2cUp)
    {
        return;
    }

    tofUp = bibo::tof::open(&tofFront, SENSOR_SDA, VL53_ADDR_DEFAULT);
    if(tofUp)
    {
        bibo::tof::startRanging(&tofFront);
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
    static_cast<Void>(arg);

    bibo::serial::printf("OK sensors i2c=%d tof=%d tof_addr=0x%02X\n",
           i2cUp ? 1 : 0, tofUp ? 1 : 0, VL53_ADDR_DEFAULT);
}

/* Every address that acknowledges. The same job as the standalone scanner
 * sketch, available over the link so the hub can offer it too. */
static Void printScan(CharSeq arg)
{
    static_cast<Void>(arg);

    if(!i2cUp)
    {
        bibo::serial::printf("ERR scan i2c not up\n");
        return;
    }

    Int32 found = 0;
    for(Int32 a = ADDR_SCAN_FIRST; a <= ADDR_SCAN_LAST; ++a)
    {
        if(bibo::i2c::present(SENSOR_SDA, static_cast<UInt8>(a)))
        {
            bibo::serial::printf("INFO scan 0x%02X\n", a);
            ++found;
        }
    }
    bibo::serial::printf("OK scan %d\n", found);
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
        bibo::serial::printf("ERR tof absent\n");
        return;
    }

    if(bibo::tof::ready(&tofFront))
    {
        const UInt16 mm = bibo::tof::distance(&tofFront);
        const UInt8  st = bibo::tof::status(&tofFront);

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
        static_cast<Void>(bibo::tof::rates(&tofFront, &sig, &amb));

        bibo::tof::clear(&tofFront);
        bibo::serial::printf("OK tof %u %u %u %u\n", mm, st, sig, amb);
        return;
    }

    /* Not ready is not an error - the sensor takes tens of milliseconds per
     * measurement and the host is entitled to ask more often than that. */
    bibo::serial::printf("OK tof busy\n");
}

static Void handleTofMode(CharSeq arg)
{
    if(!tofUp)
    {
        bibo::serial::printf("ERR tof absent\n");
        return;
    }

    /* Stop, reconfigure, start.
     *
     * Changing the VCSEL period and the phase windows underneath a running
     * measurement leaves the sensor half-configured for as long as that
     * measurement lasts, and what it does with the result is undefined.
     * ST's own driver brackets it this way and so does this. */
    if(bibo::text::eq(arg, "SHORT"))
    {
        bibo::tof::stopRanging(&tofFront);
        bibo::tof::setMode(&tofFront, bibo::tof::MODE_SHORT);
        bibo::tof::clearInterruptAndStart(&tofFront);
        bibo::serial::printf("OK tof mode short\n");
        return;
    }
    if(bibo::text::eq(arg, "LONG"))
    {
        bibo::tof::stopRanging(&tofFront);
        bibo::tof::setMode(&tofFront, bibo::tof::MODE_LONG);
        bibo::tof::clearInterruptAndStart(&tofFront);
        bibo::serial::printf("OK tof mode long\n");
        return;
    }
    bibo::serial::printf("ERR bad mode: %s\n", arg);
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
    const bibo::drive::State d = bibo::drive::read();
    bibo::serial::printf("OK drive servo=%d servo_t=%d esc=%d esc_t=%d armed=%d "
           "servo_on=%d servo_c=%d steer_m=%d steer_now=%d "
           "slew=%d slew_esc=%d "
           "servo_min=%d servo_max=%d esc_min=%d esc_max=%d\n",
           d.servoUs, d.servoTargetUs, d.escUs, d.escTargetUs,
           d.escArmed ? 1 : 0, d.servoLive ? 1 : 0, d.centerUs, d.steerMilli,
           d.steerNowMilli,
           d.steerSlewUs, d.throttleSlewUs,
           d.servoMinUs, d.servoMaxUs, d.escMinUs, d.escMaxUs);
}

static Void handleSteer(CharSeq arg)
{
    /* Rejected rather than defaulted: "STEER" with nothing after it is far more
     * likely to be a truncated command than a request to centre, and guessing
     * that it means zero would turn a typo into a movement. */
    if(arg[0] == '\0')
    {
        bibo::serial::printf("ERR steer wants -1.0 to 1.0\n");
        return;
    }

    Float32 n = 0.0f;
    if(!bibo::text::toFloat(arg, &n))
    {
        bibo::serial::printf("ERR steer wants -1.0 to 1.0\n");
        return;
    }

    bibo::drive::steer(n);
    printDrive();
}

/*
 * SLEW [STEER|THROTTLE] <us>
 *
 * With no target, sets both - which is what "the response rate" meant when
 * there was only one, and is still the common case on a bench where you want
 * everything slow while watching something.
 */
static Void handleSlew(CharSeq arg)
{
    CharSeq rest = bibo::text::word(arg, "STEER");
    if(rest != NULL)
    {
        Int32 us = 0;
        if(!bibo::text::toInt(rest, &us) || !bibo::drive::setSteerSlew(us))
        {
            bibo::serial::printf("ERR slew steer wants %d-%d us per tick\n",
                         SLEW_MIN_STEP, SLEW_MAX_STEP);
            return;
        }
        printDrive();
        return;
    }

    rest = bibo::text::word(arg, "THROTTLE");
    if(rest != NULL)
    {
        Int32 us = 0;
        if(!bibo::text::toInt(rest, &us) || !bibo::drive::setThrottleSlew(us))
        {
            bibo::serial::printf("ERR slew throttle wants %d-%d us per tick\n",
                         SLEW_MIN_STEP, SLEW_MAX_STEP);
            return;
        }
        printDrive();
        return;
    }

    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us) || !bibo::drive::setSlew(us))
    {
        bibo::serial::printf("ERR slew wants %d-%d us per tick, or STEER/THROTTLE <us>\n",
                     SLEW_MIN_STEP, SLEW_MAX_STEP);
        return;
    }

    /*
     * Reported in a unit a person thinks in as well as the one the firmware
     * uses. "8 us per tick" is a number nobody has intuition about; "400 us/s,
     * full travel 1100 ms" is the fact that decides whether it is fast enough
     * to steer around something.
     */
    const bibo::drive::State d = bibo::drive::read();
    const Int32 perSec = d.steerSlewUs * (1000 / SLEW_TICK_MS);
    bibo::serial::printf("INFO slew steer %d us/tick = %d us/s, full travel %d ms\n",
                 d.steerSlewUs, perSec,
                 (perSec > 0) ? (((d.servoMaxUs - d.servoMinUs) * 1000) / perSec)
                              : 0);

    const Int32 escPerSec = d.throttleSlewUs * (1000 / SLEW_TICK_MS);
    bibo::serial::printf("INFO slew throttle %d us/tick = %d us/s, idle to full %d ms\n",
                 d.throttleSlewUs, escPerSec,
                 (escPerSec > 0) ? (((d.escMaxUs - d.escMinUs) * 1000) / escPerSec)
                                 : 0);
    printDrive();
}

static Void handleTrim(CharSeq arg)
{
    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us))
    {
        const bibo::drive::State d = bibo::drive::read();
        bibo::serial::printf("ERR trim wants microseconds, %d-%d\n", d.servoMinUs, d.servoMaxUs);
        return;
    }

    bibo::drive::trim(us);
    bibo::serial::printf("INFO centre is now %d us\n", bibo::drive::read().centerUs);
    printDrive();
}

/*
 * SERVOLIMITS and ESCLIMITS, which are one command with two setters.
 *
 * They were written out twice, identically apart from the word in the error
 * messages and the function called - and the two copies had already drifted
 * once, because the ordering bug fixed in bibo::drive::setSteerLimits had to be
 * remembered a second time for the throttle.
 */
static Void limitsCommand(CharSeq arg, CharSeq name, Bool (*set)(Int32, Int32))
{
    Int32 lo = 0;
    Int32 hi = 0;
    if(!bibo::text::twoInts(arg, &lo, &hi))
    {
        bibo::serial::printf("ERR %s wants <min> <max>\n", name);
        return;
    }
    if(!set(lo, hi))
    {
        bibo::serial::printf("ERR %s min must be below max\n", name);
        return;
    }
    printDrive();
}

static Void handleLimits(CharSeq arg)
{
    limitsCommand(arg, "servolimits", bibo::drive::setSteerLimits);
}

static Void handleEscLimits(CharSeq arg)
{
    limitsCommand(arg, "esclimits", bibo::drive::setThrottleLimits);
}

static Void handleServo(CharSeq arg)
{
    /*
     * OFF stops the pulse train outright. This is the panic button: a servo
     * leaning on a frame does not need a better number, it needs to stop being
     * told to hold a position at all.
     */
    if(bibo::text::eq(arg, "OFF"))
    {
        bibo::drive::engage(false);
        bibo::serial::printf("INFO servo released - no pulse, no holding torque\n");
        printDrive();
        return;
    }

    if(bibo::text::eq(arg, "ON"))
    {
        bibo::drive::engage(true);
        bibo::serial::printf("INFO servo engaged - holding %d us\n", bibo::drive::read().servoTargetUs);
        printDrive();
        return;
    }

    if(bibo::text::eq(arg, "CENTER") || bibo::text::eq(arg, "CENTRE"))
    {
        bibo::drive::center();
        printDrive();
        return;
    }

    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us))
    {
        const bibo::drive::State d = bibo::drive::read();
        bibo::serial::printf("ERR servo wants microseconds, %d-%d, or ON/OFF/CENTER\n",
               d.servoMinUs, d.servoMaxUs);
        return;
    }

    /* A position asked for while released is remembered, not obeyed. Engaging
     * is a separate, deliberate act - the same shape as arming the ESC. */
    const Bool wasLive = bibo::drive::read().servoLive;
    bibo::drive::steerUs(us);
    if(!wasLive)
    {
        bibo::serial::printf("INFO servo is released - target stored, send SERVO ON\n");
    }
    printDrive();
}

static Void handleEsc(CharSeq arg)
{
    if(bibo::text::eq(arg, "ARM"))
    {
        bibo::drive::arm(true);
        bibo::serial::printf("INFO esc armed - neutral held\n");
        printDrive();
        return;
    }
    if(bibo::text::eq(arg, "DISARM"))
    {
        bibo::drive::arm(false);
        bibo::serial::printf("INFO esc disarmed\n");
        printDrive();
        return;
    }
    if(bibo::text::eq(arg, "NEUTRAL"))
    {
        bibo::drive::throttleNeutral();
        printDrive();
        return;
    }

    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us))
    {
        const bibo::drive::State d = bibo::drive::read();
        bibo::serial::printf("ERR esc wants microseconds, %d-%d\n", d.escMinUs, d.escMaxUs);
        return;
    }

    /* The module owns the arming rule; this only reports it. */
    if(!bibo::drive::throttleUs(us))
    {
        bibo::serial::printf("ERR esc not armed - send ESC ARM first\n");
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
static CharSeq LAMP_NAME[bibo::lights::COUNT] =
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
    static_cast<Void>(arg);

    const bibo::lights::Set s = bibo::lights::read();
    const bibo::cue::Turn t = bibo::cue::side();
    const Int32   f = bibo::lights::forcedLamp();

    /* ONE line, not one per lamp.
     *
     * The hub polls this every 120 ms to draw the lamps, and eight extra lines
     * a poll is seventy-five lines a second of serial traffic to say what fits
     * in one. levels[] and pins[] are in Lamp order, which is the order
     * LAMP_NAME is in and the order the model declares them. */
    bibo::serial::printf("OK lights on=%d turn=%s forced=%s off_us=%d"
                 " levels=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u"
                 " pins=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                 bibo::lights::enabled() ? 1 : 0,
                 (t == bibo::cue::TURN_LEFT)  ? "left"
                     : (t == bibo::cue::TURN_RIGHT)  ? "right"
                     : (t == bibo::cue::TURN_HAZARD) ? "hazard" : "off",
                 (f == bibo::lights::COUNT) ? "no" : LAMP_NAME[f],
                 bibo::cue::motionUs(),
                 static_cast<UInt32>(s.level[0]), static_cast<UInt32>(s.level[1]),
                 static_cast<UInt32>(s.level[2]), static_cast<UInt32>(s.level[3]),
                 static_cast<UInt32>(s.level[4]), static_cast<UInt32>(s.level[5]),
                 static_cast<UInt32>(s.level[6]), static_cast<UInt32>(s.level[7]),
                 static_cast<UInt32>(s.level[8]), static_cast<UInt32>(s.level[9]),
                 bibo::lights::pin[0], bibo::lights::pin[1], bibo::lights::pin[2], bibo::lights::pin[3],
                 bibo::lights::pin[4], bibo::lights::pin[5], bibo::lights::pin[6], bibo::lights::pin[7],
                 bibo::lights::pin[8], bibo::lights::pin[9]);
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
    if(bibo::text::eq(arg, "ON"))
    {
        bibo::lights::enable(true);
        printLights(arg);
        return;
    }
    if(bibo::text::eq(arg, "OFF"))
    {
        bibo::lights::enable(false);
        printLights(arg);
        return;
    }
    if(bibo::text::eq(arg, "AUTO"))
    {
        bibo::lights::forceLamp(bibo::lights::COUNT);
        printLights(arg);
        return;
    }

    /* LIGHTS OFFAT <us> - how far past idle counts as being driven. */
    if(bibo::text::starts(arg, "OFFAT"))
    {
        Int32 us = 0;
        if(!bibo::text::toInt(bibo::text::after(arg, "OFFAT "), &us)
           || !bibo::cue::setMotionUs(us))
        {
            bibo::serial::printf("ERR lights offat wants %d-%d us past idle\n",
                         CUE_MOTION_US_MIN, CUE_MOTION_US_MAX);
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
    for(Int32 i = 0; i < bibo::lights::COUNT; ++i)
    {
        Utf8 up[12];
        Size n = 0;
        while(LAMP_NAME[i][n] != '\0' && n < sizeof(up) - 1)
        {
            up[n] = LAMP_NAME[i][n];
            ++n;
        }
        up[n] = '\0';
        bibo::text::upper(up);

        if(bibo::text::eq(arg, up))
        {
            bibo::lights::forceLamp(i);
            printLights(arg);
            return;
        }
    }

    bibo::serial::printf("ERR lights wants ON, OFF, AUTO, OFFAT <us>, a lamp name,"
                 " or nothing\n");
}

/* ---- the command table ---------------------------------------------------
 *
 * One row per command; the dispatcher and HELP both read it.
 *
 * They used to be two lists. A chain of twenty if(bibo::text::starts(...)) blocks with
 * the argument offset written out by hand - line + 10 for "SERVOTRIM ",
 * line + 12 for "SERVOLIMITS " - and a printHelp() that spelled the same
 * commands out again in prose. Two lists of the same thing drift: the offsets
 * were a silent hazard, because miscounting one makes the argument parser read
 * from the middle of the command word and the command merely stops working.
 *
 * Matching is by WHOLE WORD, so the order of the rows means nothing. The old
 * chain worked only because SERVOTRIM and SERVOLIMITS happened to be tested
 * before SERVO, which bibo::text::starts() would otherwise have matched first.
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
    CharSeq mode = bibo::text::word(arg, "MODE");
    if(mode != NULL)
    {
        handleTofMode(mode);
        return;
    }
    printTof();
}

static Void cmdPing(CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::serial::printf("PONG\n");
}

static Void cmdDrive(CharSeq arg)
{
    static_cast<Void>(arg);
    printDrive();
}

/*
 * STOP - the one command that has to work when nothing else is going right.
 *
 * The drivetrain first, because that is the part that can hurt somebody:
 * throttle to neutral and disarmed, steering RELEASED rather than centred.
 * Released is the stronger claim - centre is only a safe place to leave a servo
 * if 1500 us is where the linkage wants to sit, and on a car whose horn is a
 * tooth off its spline it is not. Nothing to push with is the only stop that
 * works on every car.
 *
 * Then any FORCED lamp is dropped. That is not a safety matter - a lamp cannot
 * hurt anyone - but a lamp being held on by hand is an output somebody is
 * commanding, and a stop that leaves an output commanded is not a stop. The
 * lamps go back to following the car, which with the throttle now at neutral
 * means the tails come on. That is correct: the car is not being driven.
 */
static Void cmdStop(CharSeq arg)
{
    static_cast<Void>(arg);

    bibo::drive::stop();
    bibo::lights::forceLamp(bibo::lights::COUNT);

    /* Mid-sentence is still an output being commanded, and a stop that leaves
     * an output commanded is not a stop. */
    bibo::cue::silence();

    bibo::serial::printf("OK stop\n");
}

/*
 * WIFI - the wireless command link.
 *
 *   WIFI                      where it stands
 *   WIFI JOIN <ssid> <pass>   join a network; no password means an open one
 *
 * The credentials are taken from the RAW line, not the uppercased one, and
 * they are never written to flash or to this repository - a reset loses them.
 * That is a deliberate cost: this repository is pushed, and a password in a
 * source file is a password in the history forever.
 *
 * An SSID containing a space cannot be expressed here. The password can - it is
 * everything after the first space following the SSID, verbatim.
 */
static Void cmdWifi(CharSeq arg)
{
    if(!bibo::net::present())
    {
        bibo::serial::printf("ERR wifi no radio on this board (%s)\n", PICO_BOARD);
        return;
    }

    CharSeq rest = bibo::text::word(arg, "JOIN");
    if(rest == NULL)
    {
        bibo::serial::printf("INFO wifi state=%s ip=%s port=%d peer=%s dropped=%u\n",
                     bibo::net::stateWord(bibo::net::status()),
                     bibo::net::address(),
                     static_cast<Int32>(NET_PORT),
                     bibo::net::peerKnown() ? "yes" : "no",
                     bibo::net::droppedCount());
        return;
    }

    /* The same offset into the line as it was typed. */
    CharSeq raw = cmdRawArg + (rest - arg);

    Utf8 ssid[40];
    Size n = 0;
    while(raw[n] != '\0' && raw[n] != ' ' && n + 1 < sizeof(ssid))
    {
        ssid[n] = raw[n];
        n++;
    }
    ssid[n] = '\0';

    if(n == 0)
    {
        bibo::serial::printf("ERR wifi join wants <ssid> [password]\n");
        return;
    }
    if(raw[n] != '\0' && raw[n] != ' ')
    {
        /* Ran out of buffer mid-name. Say so: a truncated SSID would fail to
         * join and look like the network was out of range. */
        bibo::serial::printf("ERR wifi ssid longer than %u characters\n",
                     static_cast<UInt32>(sizeof(ssid) - 1));
        return;
    }

    while(raw[n] == ' ')
    {
        n++;
    }

    if(!bibo::net::join(ssid, &raw[n]))
    {
        bibo::serial::printf("ERR wifi could not start joining %s\n", ssid);
        return;
    }

    /* Deliberately does not say whether it WORKED - it has not finished trying.
     * The main loop reports the state when it changes. */
    bibo::serial::printf("OK wifi joining %s\n", ssid);
}

/* ---------------------------------------------------------------------------
 * CUE - what the car is saying, and telling it to say something.
 *
 *   CUE               where it stands
 *   CUE LIST          every cue this firmware knows, and what each one means
 *   CUE <name>        raise it. A held or looping cue stays up until lowered;
 *                     a one-shot plays and ends on its own.
 *   CUE <name> OFF    lower it, and hand it back to the car's own rules
 *   CUE STOP          lower everything
 *
 * The lamps are reported by LIGHTS, not here. This reports the UTTERANCE - which
 * one, how far through it is, and what it would be sounding if there were a
 * buzzer - because "the headlights blinked" and "the car said `after you`" are
 * different facts and only one of them survives being read off a lamp level.
 * ------------------------------------------------------------------------- */
static Void printCue(Void)
{
    const bibo::cue::Kind k = bibo::cue::speaking();

    /*
     * ACTIVE IS A LIST, because the car says more than one thing at a time.
     * Headlights on, braking, and indicating left is three cues at once and is
     * an ordinary Tuesday.
     *
     * `speaking` is still here and still one name: the most important of them,
     * for anything that wants a single word. Both are reported because a status
     * line that made you parse a list to answer "what is it doing" would be a
     * worse status line, and one that only gave the winner would hide the other
     * two entirely.
     *
     * A cue a PERSON raised is marked with a *, so "the car is braking" and
     * "somebody is holding the brake lamps on" are different answers on screen
     * rather than the same one.
     */
    Utf8 list[128];
    Size at = 0;
    list[0] = '\0';

    for(Int32 i = 1; i < bibo::cue::KIND_COUNT; ++i)
    {
        const bibo::cue::Kind c = static_cast<bibo::cue::Kind>(i);
        if(!bibo::cue::on(c))
        {
            continue;
        }

        const Int32 n = snprintf(&list[at], sizeof(list) - at, "%s%s%s",
                                 (at > 0) ? "," : "",
                                 bibo::cue::name(c),
                                 bibo::cue::held(c) ? "*" : "");
        if(n <= 0 || static_cast<Size>(n) >= sizeof(list) - at)
        {
            break;   /* full: report what fits rather than a truncated name */
        }
        at += static_cast<Size>(n);
    }

    bibo::serial::printf(
        "OK cue speaking=%s active=%s step=%u loop=%u tone=%u kinds=%d off_us=%d\n",
        bibo::cue::name(k),
        (list[0] == '\0') ? "-" : list,
        static_cast<UInt32>(bibo::cue::step()),
        static_cast<UInt32>(bibo::cue::loop()),
        static_cast<UInt32>(bibo::cue::tone()),
        static_cast<Int32>(bibo::cue::KIND_COUNT - 1),
        bibo::cue::motionUs());
}

static Void cmdCue(CharSeq arg)
{
    if(arg[0] == '\0')
    {
        printCue();
        return;
    }

    if(bibo::text::eq(arg, "LIST"))
    {
        /* One line each, the way HELP does it, so nothing has to be kept in
         * step with a list written somewhere else. */
        for(Int32 k = 1; k < bibo::cue::KIND_COUNT; ++k)
        {
            /* The PLAY MODE is in the line, because it is the thing a caller
             * has to know to use the cue: `once` ends on its own, `loop` and
             * `hold` stay up until something lowers them. A board that listed
             * names alone would leave every client guessing which of its
             * buttons is a toggle. */
            bibo::serial::printf("INFO cue %s [%s] - %s\n",
                         bibo::cue::SCRIPT[k].name,
                         bibo::cue::playWord(bibo::cue::SCRIPT[k].play),
                         bibo::cue::SCRIPT[k].means);
        }
        printCue();
        return;
    }

    if(bibo::text::eq(arg, "STOP"))
    {
        bibo::cue::silence();
        printCue();
        return;
    }

    /* "CUE LEFT OFF" - the name, then the word. Split before the lookup so a
     * cue is never named "LEFT OFF". */
    Utf8    word[24];
    Size    n   = 0;
    CharSeq rest = arg;

    while(*rest != '\0' && *rest != ' ' && n + 1 < sizeof(word))
    {
        word[n++] = *rest++;
    }
    word[n] = '\0';
    while(*rest == ' ')
    {
        ++rest;
    }

    const bibo::cue::Kind want = bibo::cue::find(word);
    if(want == bibo::cue::KIND_NONE)
    {
        bibo::serial::printf(
            "ERR cue wants LIST, STOP, or a cue name - try CUE LIST\n");
        return;
    }

    const Bool off = bibo::text::eq(rest, "OFF");

    if(*rest != '\0' && !off)
    {
        bibo::serial::printf("ERR cue %s takes OFF or nothing, not %s\n",
                             bibo::cue::name(want), rest);
        return;
    }

    if(off)
    {
        bibo::cue::cancel(want);
    }
    else if(!bibo::cue::emit(want))
    {
        bibo::serial::printf("ERR cue %s cannot be raised\n",
                             bibo::cue::name(want));
        return;
    }

    printCue();
}

static Void cmdBootsel(CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::serial::printf("INFO rebooting into bootloader\n");
    bibo::board::rebootToBootsel();          /* flushes, then does not return */
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
    { "STOP",        "",                        "everything off: neutral, disarm, release", cmdStop },
    { "STEER",       " <-1..1>",                "steer as a fraction of this car's travel", handleSteer },
    { "SLEW",        " [STEER|THROTTLE] <us>",  "how fast an output may move, per tick",    handleSlew },
    { "SERVO",       " <us>|ON|OFF|CENTER",     "steering; OFF stops the pulse, servo limp", handleServo },
    { "SERVOTRIM",   " <us>",                   "move where centre is",                     handleTrim },
    { "SERVOLIMITS", " <min> <max>",            "widen to find the real end stops",         handleLimits },
    { "ESC",         " ARM|DISARM|NEUTRAL|<us>", "throttle",                                handleEsc },
    { "ESCLIMITS",   " <min> <max>",            "widen the throttle range",                 handleEscLimits },

    { "WIFI",        " [JOIN <ssid> <password>]", "the wireless command link",           cmdWifi },
    { "CUE",         " [LIST|STOP|<name> [OFF]]",     "what the car says, and saying it",      cmdCue },

    /* TEMPORARY - the indicator scaffolding. Goes when GP15 is given back to
     * the wheel encoder. See lib/lights.h. */
    { "LIGHTS",      " [ON|OFF|AUTO|OFFAT <us>|<lamp>]", "the lamps, and what each is doing", handleLights },
};

static const Size COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

static Void printHelp(CharSeq arg)
{
    static_cast<Void>(arg);
    for(Size i = 0; i < COMMAND_COUNT; ++i)
    {
        bibo::serial::printf("INFO help %s%s - %s\n",
                     COMMANDS[i].name, COMMANDS[i].usage, COMMANDS[i].what);
    }
}

static Void handleLine(Utf8* line)
{
    /* A terminal decides for itself what to put at the end of a line. Without
     * this, "PING\r" is not "PING" and a correctly typed command is refused. */
    if(bibo::text::trimEnd(line) == 0)
    {
        return;
    }

    /* BEFORE bibo::text::upper, which rewrites in place. */
    snprintf(rawLine, sizeof(rawLine), "%s", line);

    bibo::text::upper(line);

    /* ANY line from the host counts as liveness, including one that turns out
     * to be a bad command. The question this asks is "is somebody still there",
     * not "is somebody still there and getting it right". */
    lastCmdMs      = bibo::timing::nowMs();
    deadmanTripped = false;

    /* "?" is HELP, and is not a row of its own: it would print as a command in
     * its own listing, which is one more thing than anybody wants to read. */
    if(bibo::text::eq(line, "?"))
    {
        printHelp(line);
        return;
    }

    for(Size i = 0; i < COMMAND_COUNT; ++i)
    {
        CharSeq arg = bibo::text::word(line, COMMANDS[i].name);
        if(arg != NULL)
        {
            /* Same offset, other buffer - see rawLine above. */
            cmdRawArg = rawLine + (arg - (CharSeq) line);
            COMMANDS[i].run(arg);
            return;
        }
    }

    bibo::serial::printf("ERR unknown command: %s\n", line);
}

/* ------------------------------------------------------------------ main -- */

/*
 * `int`, not Int32, and this is the one place in the program where that is
 * right.
 *
 * C++ requires main to return literally `int`. Int32 is int32_t, and on this
 * toolchain int32_t is `long` - the same size, the same representation, a
 * different type as far as the language is concerned, and the compiler refuses
 * it. main's signature is the C runtime's contract, not this project's
 * vocabulary, so it is spelled the runtime's way.
 */
int main(Void)
{
    bibo::serial::open();

    /* Sensors come up at boot so SENSORS and TOF can answer immediately. A
     * missing sensor is not a failure here - it is the answer. */
    sensorsOpen();

    /* The ESC to neutral, the steering RELEASED. See chassis.h for why those
     * are different answers. */
    bibo::drive::open();

    /* Brings up whatever this board's LED hangs off.
     *
     * On the Pico 2 W that is the CYW43439 - slow (hundreds of ms) and able to
     * fail, so the result is REPORTED rather than assumed: a board that answers
     * PING but says cyw43=FAILED is a very different problem from a board that
     * is silent. On the plain Pico 2 it is GP25 and cannot fail. Either way
     * bibo::status::open() remembers the outcome and every later call is a no-op
     * rather than a crash. */
    bibo::status::open();

    /* The indicator lamps. TEMPORARY scaffolding on borrowed pins - lib/lights.h
     * says which and why. Opened AFTER bibo::drive::open() so that if the two ever
     * disagree about a pin, the drivetrain wins: a stray LED is a cosmetic
     * fault and a servo pin that is secretly an output is not. */
    bibo::lights::open();

    /* What the car SAYS with those lamps. Opened after them, because a cue with
     * nowhere to come out of is a cue that fails silently. */
    bibo::cue::open();

    /* The threshold is a tuning that survives a reflash, so it lives in cal.h
     * and is handed to the module here. cue.h cannot reach for cal.h - the
     * layering forbids it, and rightly: the RULE is the same on any car and only
     * the number is this one's. */
    bibo::cue::setMotionUs(LIGHT_CAL_OFF_US);

    /* Visible proof of life the moment power is applied, before any host could
     * be listening: three quick flashes, then a slow idle heartbeat. */
    bibo::status::hello(HELLO_FLASHES, HELLO_FLASH_MS);
    bibo::status::blink(IDLE_BLINK_HZ);

    /* ---- the wireless link ---------------------------------------------
     *
     * Wired up, not switched on. Nothing here touches the radio until somebody
     * sends WIFI JOIN, so the boards that will never use it - and the seconds
     * before anybody asks - cost nothing.
     *
     * Two connections, and they are the two halves of the same seam:
     *
     *   bibo::net::setLineHandler  a line arriving in a datagram goes to the SAME
     *                      handler a line arriving on the cable goes to. There
     *                      is one command language, not two.
     *   bibo::serial::setMirror    everything this program prints goes to the wireless
     *                      peer as well as the cable, so a host that is only
     *                      listening wirelessly sees the replies to its own
     *                      commands - and to anybody else's.
     */
    bibo::net::setLineHandler(handleLine);
    bibo::serial::setMirror(bibo::net::sendLine);

    Utf8 line[LINE_CAP];
    Size len      = 0;
    Bool overlong = false;
    Bool announced = false;

    for(;;)
    {
        bibo::status::tick();

        /* ---- the wireless link ------------------------------------------
         *
         * BEFORE the drivetrain, and before the serial read below that skips
         * the rest of the loop on an idle millisecond.
         *
         * In NO_SYS mode nothing in lwIP happens on its own: no packet is
         * received, no join completes, no timer fires except inside this call.
         * A command that arrived wirelessly is dispatched from in here, which
         * means it feeds the deadman exactly like one off the cable does.
         */
        bibo::net::poll();

        {
            const bibo::net::State ns = bibo::net::status();
            if(ns != netReported)
            {
                netReported = ns;
                bibo::serial::printf("INFO wifi state=%s ip=%s port=%d\n",
                             bibo::net::stateWord(ns), bibo::net::address(), static_cast<Int32>(NET_PORT));
            }
        }

        /* Walks the servo and ESC toward their targets, a few microseconds at a
         * time. Nothing jumps: a slider dragged end to end produces a sweep
         * rather than a step. */
        bibo::drive::pump();

        /* ---- the deadman ------------------------------------------------
         *
         * bibo::drive::stop() FIRST, then the report. bibo::serial::printf blocks while the CDC
         * TX buffer is full, for up to half a second, and a host that has
         * stopped draining the port is one of the exact situations this exists
         * for - so the car is stopped before anything is printed, not after.
         */
        {
            const bibo::drive::State dm      = bibo::drive::read();
            const Bool       driving = dm.escArmed && (dm.escTargetUs > dm.escMinUs);

            if(driving && !deadmanTripped && (bibo::timing::nowMs() - lastCmdMs) > DEADMAN_MS)
            {
                bibo::drive::stop();
                bibo::lights::forceLamp(bibo::lights::COUNT);

                /* And it SAYS so, on the car, where somebody standing next to
                 * it can see. By definition this fires when the host has
                 * stopped listening, so a line in a console nobody is reading
                 * is the one place the message must not only be. */
                bibo::cue::emit(bibo::cue::KIND_ALERT);

                deadmanTripped = true;

                bibo::serial::printf("ERR deadman - no command for %u ms, stopped\n",
                             static_cast<UInt32>(DEADMAN_MS));
            }
        }

        /* AFTER bibo::drive::pump, and reading the ACTUAL servo and ESC output rather
         * than their targets. The slew limiter means the two differ for about a
         * second after every command: reading targets would light a lamp before
         * the car had done the thing the lamp is reporting. */
        {
            const bibo::drive::State d = bibo::drive::read();

            bibo::cue::Input ci;
            ci.steerMilli = d.steerNowMilli;
            ci.throttleUs = d.escUs;
            ci.idleUs     = d.escMinUs;
            ci.neutralUs  = DRIVE_NEUTRAL_US;
            ci.armed      = d.escArmed;
            ci.headOn     = false;   /* nothing the car knows implies darkness */

            bibo::cue::tick(&ci);
        }

        /* Anything written before the host opens the port is discarded, so the
         * banner waits for a connection rather than being lost. */
        const Bool host = bibo::serial::hostPresent();
        if(!announced && host)
        {
            bibo::serial::printf("INFO ready %s sdk=%s - type HELP\n",
                   PICO_BOARD, PICO_SDK_VERSION_STRING);
            announced = true;
        }
        if(announced && !host)
        {
            announced = false;      /* re-announce on the next connection */
        }

        const Int32 c = bibo::serial::readChar(POLL_TIMEOUT_US);
        if(c == bibo::serial::NONE)
        {
            continue;
        }

        if(c == '\n' || c == '\r')
        {
            /* A line that overran is discarded at its END, not where it
             * overran - see below. */
            if(!overlong)
            {
                line[len] = '\0';
                handleLine(line);
            }
            len      = 0;
            overlong = false;
        }
        else if(overlong)
        {
            /* Swallowing the rest of an over-long line. */
        }
        else if(len + 1 < LINE_CAP)
        {
            line[len++] = (Utf8) c;
        }
        else
        {
            /*
             * This said it dropped the line and did not.
             *
             * Setting len = 0 with no "swallow" state left the TAIL of an
             * over-long line accumulating as a fresh command: a 200-byte line
             * produced this error AND whatever bytes 129 onward happened to
             * parse as - a command nobody sent, synthesised out of the middle
             * of one they did. On a link that steers a vehicle that is worth
             * more than a comment. The host side already got this right with an
             * explicit flag; this is the same fix.
             */
            len      = 0;
            overlong = true;
            bibo::serial::printf("ERR line too long\n");
        }
    }
}
