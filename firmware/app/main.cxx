/*
 * main - the Pico's program. It moves the steering servo and the ESC for the
 * pilot on the Orange Pi, which drives it over newline-terminated ASCII on USB
 * CDC. Every command answers with a line starting OK, ERR, INFO or PONG;
 * COMMANDS lists them and HELP prints it.
 *
 * Nothing here touches the Pico SDK or a GPIO number. What is safe is decided in
 * lib/chassis; the handlers only report what it refused. The motor's hall
 * sensors are counted by lib/encoder.hxx from interrupts, and every OK drive
 * line carries the count, so the pilot gets odometry from replies it already
 * reads.
 */
#include "../lib/bibo.hxx"

/* Not LINE_MAX: POSIX reserves that name and some newlib configurations define it. */
#define LINE_CAP 128

#define HELLO_FLASHES 3
#define HELLO_FLASH_MS 80
#define IDLE_BLINK_HZ 0.5f

/* Short, so the loop keeps pumping the outputs while it waits. */
#define POLL_TIMEOUT_US 1000

/*
 * The watchdog, which the pilot calls the deadman (not the RP2350's hardware
 * watchdog): with no VALID command for this long, the ESC goes to neutral, still
 * armed, steering held. It runs whether or not the car is moving, and starts
 * tripped, so nothing moves before the first valid command.
 */
static constexpr UInt32 WATCHDOG_MS = BIBO_WATCHDOG_MS;

/*
 * Refreshed only by an ACCEPTED command: an unknown command, a bad argument or
 * half a line from a sender that died mid-write must not keep the car alive.
 * DRIVE reports stale as stale=.
 */
static UInt32 lastValidCmdMs = 0;
static Bool stale = true;

static Bool cmdPing(const CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::serial::printf("PONG\n");
    return true;
}

static Bool printId(const CharSeq arg)
{
    static_cast<Void>(arg);
    Utf8 uid[24];
    bibo::board::id(uid, sizeof(uid));
    bibo::serial::printf(
        "INFO id board=%s sdk=%s built=%s %s uid=%s lamp=%s lamp_up=%s\n",
        PICO_BOARD,
        PICO_SDK_VERSION_STRING,
        __DATE__,
        __TIME__,
        uid,
        bibo::led::backend(),
        bibo::led::present() ? "yes" : "no"
    );
    return true;
}

static Bool cmdBootsel(const CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::serial::printf("INFO rebooting into bootloader\n");
    bibo::board::rebootToBootsel();
    return true;
}

/* A count on the wire is a byte: 255 means "255 or more", read as "not clean". */
static UInt32 saturated(const UInt32 n)
{
    return n > 255u ? 255u : n;
}

/*
 * Every drivetrain command answers with this line, so the pilot learns the
 * board's state from replies it already reads. tick= is the encoder's signed
 * count, tps= its speed in ticks a second, hskip= and hbad= its error counts
 * (saturated: a byte each on the wire).
 */
static Bool printDrive(Void)
{
    const bibo::drive::State d = bibo::drive::read();
    const bibo::encoder::Snapshot e = bibo::encoder::read();
    bibo::serial::printf(
        "OK drive servo=%d servo_t=%d esc=%d esc_t=%d armed=%d " "servo_on=%d servo_c=%d steer_m=%d steer_now=%d " "slew=%d slew_esc=%d " "servo_min=%d servo_max=%d esc_min=%d esc_max=%d esc_rev=%d " "stale=%d "
        "tick=%ld tps=%ld hskip=%lu hbad=%lu\n",
        d.servoUs,
        d.servoTargetUs,
        d.escUs,
        d.escTargetUs,
        d.escArmed ? 1 : 0,
        d.servoLive ? 1 : 0,
        d.centerUs,
        d.steerMilli,
        d.steerNowMilli,
        d.steerSlewUs,
        d.throttleSlewUs,
        d.servoMinUs,
        d.servoMaxUs,
        d.escMinUs,
        d.escMaxUs,
        d.escReverseUs,
        stale ? 1 : 0,
        static_cast<long>(e.ticks),
        static_cast<long>(e.ticksPerS),
        static_cast<unsigned long>(saturated(e.skips)),
        static_cast<unsigned long>(saturated(e.invalid))
    );
    return true;
}

/*
 * The encoder in full, for the bench: what OK drive carries plus the pieces of
 * the speed estimate and the counts nothing acts on. HALL ZERO restarts the
 * counts from where the wheel is.
 */
static Bool handleHall(const CharSeq arg)
{
    if(bibo::text::word(arg, "ZERO") != nullptr)
    {
        bibo::encoder::zero();
        bibo::serial::printf("OK hall zeroed\n");
        return true;
    }
    if(arg[0] != 0)
    {
        bibo::serial::printf("ERR hall wants nothing or ZERO\n");
        return false;
    }
    const bibo::encoder::Snapshot e = bibo::encoder::read();
    bibo::serial::printf(
        "OK hall tick=%ld tps=%ld src=%s win=%ld per=%ld skip=%lu bad=%lu rep=%lu miss=%lu state=%u%u%u edges=%lu\n",
        static_cast<long>(e.ticks),
        static_cast<long>(e.ticksPerS),
        e.stopped ? "stopped" : (e.usePeriod ? "period" : "window"),
        static_cast<long>(e.windowTps),
        static_cast<long>(e.periodTps),
        static_cast<unsigned long>(e.skips),
        static_cast<unsigned long>(e.invalid),
        static_cast<unsigned long>(e.repeats),
        static_cast<unsigned long>(e.misses),
        (static_cast<UInt32>(e.state) >> 2) & 1u,
        (static_cast<UInt32>(e.state) >> 1) & 1u,
        static_cast<UInt32>(e.state) & 1u,
        static_cast<unsigned long>(e.edges)
    );
    return true;
}

static Bool cmdDrive(const CharSeq arg)
{
    static_cast<Void>(arg);
    return printDrive();
}

static Bool cmdStop(const CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::drive::stop();
    bibo::serial::printf("OK stop\n");
    return true;
}

/*
 * Empty is refused rather than read as center. The pilot sends STEER every tick,
 * so this is usually what feeds the watchdog.
 */
static Bool handleSteer(const CharSeq arg)
{
    Float32 n = 0.0f;
    if(!bibo::text::toFloat(arg, &n))
    {
        bibo::serial::printf("ERR steer wants -1.0 to 1.0\n");
        return false;
    }
    bibo::drive::steer(n);
    return printDrive();
}

static Bool handleSlew(const CharSeq arg)
{
    CharSeq rest = bibo::text::word(arg, "STEER");
    if(rest != nullptr)
    {
        Int32 us = 0;
        if(!bibo::text::toInt(rest, &us) || !bibo::drive::setSteerSlew(us))
        {
            bibo::serial::printf(
                "ERR slew steer wants %d-%d us per tick\n",
                SLEW_MIN_STEP,
                SLEW_MAX_STEP
            );
            return false;
        }
        return printDrive();
    }
    rest = bibo::text::word(arg, "THROTTLE");
    if(rest != nullptr)
    {
        Int32 us = 0;
        if(!bibo::text::toInt(rest, &us) || !bibo::drive::setThrottleSlew(us))
        {
            bibo::serial::printf(
                "ERR slew throttle wants %d-%d us per tick\n",
                SLEW_MIN_STEP,
                SLEW_MAX_STEP
            );
            return false;
        }
        return printDrive();
    }
    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us) || !bibo::drive::setSlew(us))
    {
        bibo::serial::printf(
            "ERR slew wants %d-%d us per tick, or STEER/THROTTLE <us>\n",
            SLEW_MIN_STEP,
            SLEW_MAX_STEP
        );
        return false;
    }
    const bibo::drive::State d = bibo::drive::read();
    const Int32 perSec = d.steerSlewUs * (1000 / SLEW_TICK_MS);
    bibo::serial::printf(
        "INFO slew steer %d us/tick = %d us/s, full travel %d ms\n",
        d.steerSlewUs,
        perSec,
        (perSec > 0) ? (((d.servoMaxUs - d.servoMinUs) * 1000) / perSec) : 0
    );
    const Int32 escPerSec = d.throttleSlewUs * (1000 / SLEW_TICK_MS);
    bibo::serial::printf(
        "INFO slew throttle %d us/tick = %d us/s, idle to full %d ms\n",
        d.throttleSlewUs,
        escPerSec,
        (escPerSec > 0) ? (((d.escMaxUs - d.escMinUs) * 1000) / escPerSec) : 0
    );
    return printDrive();
}

/* SERVOTRIM. A center outside the servo range is clamped, not refused, so it counts as accepted. */
static Bool handleTrim(const CharSeq arg)
{
    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us))
    {
        const bibo::drive::State d = bibo::drive::read();
        bibo::serial::printf("ERR trim wants microseconds, %d-%d\n", d.servoMinUs, d.servoMaxUs);
        return false;
    }
    bibo::drive::trim(us);
    bibo::serial::printf("INFO center is now %d us\n", bibo::drive::read().centerUs);
    return printDrive();
}

static Bool limitsCommand(const CharSeq arg, const CharSeq name, Bool (*set)(Int32, Int32))
{
    Int32 lo = 0;
    Int32 hi = 0;
    if(!bibo::text::twoInts(arg, &lo, &hi))
    {
        bibo::serial::printf("ERR %s wants <min> <max>\n", name);
        return false;
    }
    if(!set(lo, hi))
    {
        bibo::serial::printf("ERR %s min must be below max\n", name);
        return false;
    }
    return printDrive();
}

static Bool handleLimits(const CharSeq arg)
{
    return limitsCommand(arg, "servolimits", bibo::drive::setSteerLimits);
}

static Bool handleEscLimits(const CharSeq arg)
{
    return limitsCommand(arg, "esclimits", bibo::drive::setThrottleLimits);
}

static Bool handleEscReverse(const CharSeq arg)
{
    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us) || !bibo::drive::setReverseLimit(us))
    {
        bibo::serial::printf(
            "ERR escreverse wants microseconds, %d-%d\n",
            ESC_HARD_MIN,
            DRIVE_NEUTRAL_US
        );
        return false;
    }
    return printDrive();
}

/* A microsecond value outside the servo range is clamped, not refused, so it counts as accepted. */
static Bool handleServo(const CharSeq arg)
{
    if(bibo::text::eq(arg, "OFF"))
    {
        bibo::drive::engage(false);
        bibo::serial::printf("INFO servo released - no pulse, no holding torque\n");
        return printDrive();
    }
    if(bibo::text::eq(arg, "ON"))
    {
        bibo::drive::engage(true);
        bibo::serial::printf(
            "INFO servo engaged - holding %d us\n",
            bibo::drive::read().servoTargetUs
        );
        return printDrive();
    }
    if(bibo::text::eq(arg, "CENTER") || bibo::text::eq(arg, "CENTRE"))
    {
        bibo::drive::center();
        return printDrive();
    }
    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us))
    {
        const bibo::drive::State d = bibo::drive::read();
        bibo::serial::printf(
            "ERR servo wants microseconds, %d-%d, or ON/OFF/CENTER\n",
            d.servoMinUs,
            d.servoMaxUs
        );
        return false;
    }
    /* Remembered, not obeyed: engaging is a separate act, like arming the ESC. */
    const Bool wasLive = bibo::drive::read().servoLive;
    bibo::drive::steerUs(us);
    if(!wasLive)
    {
        bibo::serial::printf("INFO servo is released - target stored, send SERVO ON\n");
    }
    return printDrive();
}

/*
 * A throttle is refused while disarmed rather than stored, so ARM never wakes a
 * car that moves at once, and a refusal does not feed the watchdog: a host asking
 * for what it may not have is not driving the car.
 *
 * ARM opens the gate on OUR side only. The QuicRun 10BL160 G2 ignores everything
 * until it has seen neutral since power-up, which reads exactly like a dead ESC.
 */
static Bool handleEsc(const CharSeq arg)
{
    if(bibo::text::eq(arg, "ARM"))
    {
        bibo::drive::arm(true);
        bibo::serial::printf("INFO esc armed - neutral held\n");
        return printDrive();
    }
    if(bibo::text::eq(arg, "DISARM"))
    {
        bibo::drive::arm(false);
        bibo::serial::printf("INFO esc disarmed\n");
        return printDrive();
    }
    if(bibo::text::eq(arg, "NEUTRAL"))
    {
        bibo::drive::throttleNeutral();
        return printDrive();
    }
    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us))
    {
        const bibo::drive::State d = bibo::drive::read();
        bibo::serial::printf("ERR esc wants microseconds, %d-%d\n", d.escMinUs, d.escMaxUs);
        return false;
    }
    if(!bibo::drive::throttleUs(us))
    {
        bibo::serial::printf("ERR esc not armed - send ESC ARM first\n");
        return false;
    }
    return printDrive();
}

/*
 * Gets the uppercased text after the command word. True only when the command
 * parsed completely and was in range, which is what feeds the watchdog; false
 * for anything answered with ERR.
 */
typedef Bool (*CmdRun)(CharSeq arg);

struct Command
{
    CharSeq name;
    CharSeq usage;
    CharSeq what;
    CmdRun  run;
};

static Bool printHelp(CharSeq arg);

/* Matched by WHOLE word, so row order means nothing: SERVO does not match SERVOTRIM. */
static const Command COMMANDS[] =
{
    { .name = "PING",        .usage = "",                         .what = "answers PONG",                              .run = cmdPing },
    { .name = "ID",          .usage = "",                         .what = "board, sdk, build time, unique id, lamp",   .run = printId },
    { .name = "HELP",        .usage = "",                         .what = "this list",                                 .run = printHelp },
    { .name = "BOOTSEL",     .usage = "",                         .what = "reboot into the UF2 bootloader",            .run = cmdBootsel },
    { .name = "DRIVE",       .usage = "",                         .what = "servo and esc state",                       .run = cmdDrive },
    { .name = "STOP",        .usage = "",                         .what = "everything off: neutral, disarm, release",  .run = cmdStop },
    { .name = "STEER",       .usage = " <-1..1>",                 .what = "steer as a fraction of this car's travel",  .run = handleSteer },
    { .name = "SLEW",        .usage = " [STEER|THROTTLE] <us>",   .what = "how fast an output may move, per tick",     .run = handleSlew },
    { .name = "SERVO",       .usage = " <us>|ON|OFF|CENTER",      .what = "steering; OFF stops the pulse, servo limp", .run = handleServo },
    { .name = "SERVOTRIM",   .usage = " <us>",                    .what = "move where center is",                      .run = handleTrim },
    { .name = "SERVOLIMITS", .usage = " <min> <max>",             .what = "the steering's working range",              .run = handleLimits },
    { .name = "ESC",         .usage = " ARM|DISARM|NEUTRAL|<us>", .what = "throttle",                                  .run = handleEsc },
    { .name = "ESCLIMITS",   .usage = " <min> <max>",             .what = "the throttle's working forward range",      .run = handleEscLimits },
    { .name = "ESCREVERSE",  .usage = " <us>",                    .what = "lowest brake/reverse pulse; neutral is off", .run = handleEscReverse },
    { .name = "HALL",        .usage = " [ZERO]",                  .what = "the wheel encoder in full; ZERO restarts it",  .run = handleHall },
};

static Bool printHelp(const CharSeq arg)
{
    static_cast<Void>(arg);
    for(const auto& i : COMMANDS)
    {
        bibo::serial::printf("INFO help %s%s - %s\n", i.name, i.usage, i.what);
    }
    return true;
}

static Void handleLine(Utf8* line)
{
    if(bibo::text::trimEnd(line) == 0)
    {
        return;
    }
    bibo::text::upper(line);
    Bool accepted = false;
    /* "?" is HELP, not a row of its own - it would print in its own listing. */
    if(bibo::text::eq(line, "?"))
    {
        accepted = printHelp(line);
    }
    else
    {
        Bool matched = false;
        for(const auto& i : COMMANDS)
        {
            if(const CharSeq arg = bibo::text::word(line, i.name); arg != nullptr)
            {
                accepted = i.run(arg);
                matched = true;
                break;
            }
        }
        if(!matched)
        {
            bibo::serial::printf("ERR unknown command: %s\n", line);
        }
    }
    if(accepted)
    {
        lastValidCmdMs = bibo::timing::nowMs();
        stale = false;
    }
}

/*
 * `int`, not Int32: C++ requires main to return literally `int`, and Int32 is
 * `long int` on arm-none-eabi.
 */
int main(Void)
{
    bibo::serial::open();
    /* Before anything opens: each subsystem reads its pads from the map when it opens. */
    if(!bibo::pins::begin(bibo::pins::car()))
    {
        bibo::serial::printf("ERR %s\n", bibo::pins::conflictText());
    }
    bibo::drive::open();
    /* The hall lines, counted from here on; OK drive carries the count. */
    bibo::encoder::open(
        bibo::pins::active().hallA,
        bibo::pins::active().hallB,
        bibo::pins::active().hallC
    );
    /* Can fail on the Pico 2 W, whose LED hangs off the CYW43439; ID reports that as lamp_up=no. */
    bibo::status::open();
    bibo::status::hello(HELLO_FLASHES, HELLO_FLASH_MS);
    bibo::status::blink(IDLE_BLINK_HZ);
    Utf8 line[LINE_CAP];
    Size len = 0;
    Bool overlong = false;
    Bool announced = false;
    for(;;)
    {
        bibo::status::tick();
        bibo::drive::pump();
        bibo::encoder::pump();
        /*
         * The watchdog lives in the loop, so a blocked read or a silent, killed or
         * hung host trips it. A pulled cable or a power cut turns the Pico off, and
         * no watchdog runs. The throttle is cut before the report, because
         * serial::printf can block while the host is not draining the port.
         */
        if(!stale && (bibo::timing::nowMs() - lastValidCmdMs) > WATCHDOG_MS)
        {
            const bibo::drive::State dm = bibo::drive::read();
            /* Below neutral is brake and reverse, which is driving too. */
            const Bool driving = dm.escArmed
                              && (dm.escTargetUs > dm.escMinUs || dm.escTargetUs < DRIVE_NEUTRAL_US);
            bibo::drive::throttleNeutralNow();
            stale = true;
            /*
             * Only a car that was being driven complains, or a bench board would
             * after every typed command.
             */
            if(driving)
            {
                bibo::serial::printf(
                    "ERR watchdog - no valid command for %u ms, throttle neutral\n",
                    WATCHDOG_MS
                );
            }
        }
        /* Output before the host opens the port is discarded, so the greeting waits for it. */
        const Bool host = bibo::serial::hostPresent();
        if(!announced && host)
        {
            bibo::serial::printf(
                "INFO ready %s sdk=%s - type HELP\n",
                PICO_BOARD,
                PICO_SDK_VERSION_STRING
            );
            announced = true;
        }
        if(announced && !host)
        {
            announced = false;
        }
        const Int32 c = bibo::serial::readChar(POLL_TIMEOUT_US);
        if(c == bibo::serial::NONE)
        {
            continue;
        }
        if(c == '\n' || c == '\r')
        {
            if(!overlong)
            {
                line[len] = '\0';
                handleLine(line);
            }
            len = 0;
            overlong = false;
        }
        else if(overlong)
        {
            /* Swallowing the rest of an overlong line. */
        }
        else if(len + 1 < LINE_CAP)
        {
            line[len++] = static_cast<Utf8>(c);
        }
        else
        {
            /*
             * The flag drops the TAIL too; resetting len alone would parse the rest
             * as a command nobody sent.
             */
            len = 0;
            overlong = true;
            bibo::serial::printf("ERR line too long\n");
        }
    }
}
