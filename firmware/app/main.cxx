/*
 * main - the Pico's program. It moves the steering servo and the ESC for the
 * pilot on the Orange Pi, which drives it over newline-terminated ASCII on USB
 * CDC. Every command answers with a line starting OK, ERR, INFO or PONG;
 * COMMANDS below is the list, and HELP prints it.
 *
 * Nothing here touches the Pico SDK or a GPIO number directly.
 */

/* The whole library; an application includes this and nothing else of ours. */
#include "../lib/bibo.hxx"


/*
 * Not LINE_MAX: POSIX reserves that name and <limits.h> defines it on some
 * newlib configurations, making this a redefinition.
 */
#define LINE_CAP 128

/* The heartbeat: quick flashes at power-on, before any host has the port, then a slow blink. */
#define HELLO_FLASHES 3
#define HELLO_FLASH_MS 80
#define IDLE_BLINK_HZ 0.5f

/* How long one serial read waits, so the loop keeps pumping the outputs. */
#define POLL_TIMEOUT_US 1000

/*
 * ---- the watchdog ---------------------------------------------------------
 *
 * How long the board keeps obeying the last throttle with no VALID command from
 * the host. Then the ESC goes to neutral, still armed, with the steering held.
 * Without it, a host that died with the ESC armed left pump() writing its last
 * throttle forever. It runs whether or not the car is moving, and it starts
 * tripped, so nothing moves before a first valid command.
 *
 * The pilot calls it the deadman. It is not the RP2350's hardware watchdog,
 * which this firmware does not use.
 */
static constexpr UInt32 WATCHDOG_MS = BIBO_WATCHDOG_MS;

/*
 * When the last VALID command arrived, and whether the watchdog has tripped
 * since; DRIVE reports the flag as stale=. Valid, not merely arrived: half a
 * line from a sender that died mid-write must not keep the car alive, so every
 * handler returns whether it accepted its command and handleLine() refreshes
 * these only then.
 */
static UInt32 lastValidCmdMs = 0;
static Bool stale = true;

/* -------------------------------------------------------------- commands -- */

/**
 * @brief Runs PING: answers PONG.
 *
 * @param arg unused; PING takes no argument
 * @return true - the pilot opens with PING, and it feeds the watchdog like any
 *         other accepted command
 */
static Bool cmdPing(const CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::serial::printf("PONG\n");

    return true;
}

/**
 * @brief Answers ID: board, SDK version, build time, unique id and lamp, as
 *        one INFO line.
 *
 * @param arg unused; ID takes no argument
 * @return true - ID takes no argument, so there is nothing to reject
 */
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

/**
 * @brief Runs BOOTSEL: reboots the board into the UF2 bootloader.
 *
 * @param arg unused; BOOTSEL takes no argument
 * @return true, never reached - rebootToBootsel() does not return
 */
static Bool cmdBootsel(const CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::serial::printf("INFO rebooting into bootloader\n");
    bibo::board::rebootToBootsel();

    return true;
}

/*
 * ================================================================== drive ==
 *
 * Console glue over lib/chassis. What is SAFE lives in the module - it refuses
 * throttle until armed - and this file only reports what it refused.
 */

/**
 * @brief Answers DRIVE: the servo and ESC state, as one OK line.
 *
 * Also the answer to every STEER, ESC, SERVO, SLEW and limit command, so the
 * pilot learns the board's state from replies it already reads. stale= is the
 * watchdog's flag.
 *
 * @return true - reporting the drivetrain cannot fail
 */
static Bool printDrive(Void)
{
    const bibo::drive::State d = bibo::drive::read();
    bibo::serial::printf(
        "OK drive servo=%d servo_t=%d esc=%d esc_t=%d armed=%d " "servo_on=%d servo_c=%d steer_m=%d steer_now=%d " "slew=%d slew_esc=%d " "servo_min=%d servo_max=%d esc_min=%d esc_max=%d esc_rev=%d " "stale=%d\n",
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
        stale ? 1 : 0
    );

    return true;
}

/**
 * @brief Runs DRIVE: reports the servo and ESC state.
 *
 * @param arg unused; DRIVE takes no argument
 * @return true - reporting the drivetrain cannot fail
 */
static Bool cmdDrive(const CharSeq arg)
{
    static_cast<Void>(arg);

    return printDrive();
}

/**
 * @brief Runs STOP: throttle to neutral and disarmed, steering released.
 *
 * Released rather than centered: center is only safe where the linkage wants
 * to sit, and nothing to push with is a stop on every car.
 *
 * @param arg unused; STOP takes no argument
 * @return true - STOP has no argument to get wrong and nothing to refuse
 *
 * @warning This is the emergency stop. It disarms the ESC and releases the
 *          steering servo (no holding torque).
 */
static Bool cmdStop(const CharSeq arg)
{
    static_cast<Void>(arg);

    bibo::drive::stop();
    bibo::serial::printf("OK stop\n");

    return true;
}

/**
 * @brief Runs STEER: sets the steering as a fraction of this car's travel.
 *
 * @param arg a number from -1.0 to 1.0. Empty is refused rather than read as
 *            center, and so are NAN and INF.
 * @return false for an argument that is not a finite number; true once the
 *         steering target has moved
 *
 * @note The pilot sends this every tick, so this return is usually what feeds
 *       the watchdog.
 *
 * @warning Moves the steering servo immediately if it is engaged.
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

/**
 * @brief Runs SLEW: sets how fast the steering, the throttle, or both may
 *        move, in microseconds per tick.
 *
 * @param arg "<us>" for both, or "STEER <us>" / "THROTTLE <us>" for one
 * @return false when the rate is not a number or falls outside
 *         [SLEW_MIN_STEP, SLEW_MAX_STEP]; true once it has been applied
 */
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

    /* Also in units a person can judge: us per second, and the time for full travel. */
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

/**
 * @brief Runs SERVOTRIM: moves where center is, without moving the endpoints.
 *
 * @param arg the new center, in microseconds, within the current servo range
 * @return false when the argument is not a number; true once the center has
 *         moved. A value outside the servo range is CLAMPED rather than
 *         refused, so it counts as accepted.
 */
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

/**
 * @brief Shared body for SERVOLIMITS and ESCLIMITS: parses "<min> <max>" and
 *        applies them through the given setter.
 *
 * @param arg the command's argument, expected to be "<min> <max>"
 * @param name the command word for an error message
 * @param set the setter to apply lo/hi through, once both parse
 * @return false when the pair does not parse or the setter refused it; true
 *         once the limits are in force
 */
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

/**
 * @brief Runs SERVOLIMITS: sets the steering's working microsecond range.
 *
 * @param arg "<min> <max>", in microseconds
 * @return what limitsCommand made of it
 */
static Bool handleLimits(const CharSeq arg)
{
    return limitsCommand(arg, "servolimits", bibo::drive::setSteerLimits);
}

/**
 * @brief Runs ESCLIMITS: sets the throttle's working forward range.
 *
 * @param arg "<min> <max>", in microseconds
 * @return what limitsCommand made of it
 */
static Bool handleEscLimits(const CharSeq arg)
{
    return limitsCommand(arg, "esclimits", bibo::drive::setThrottleLimits);
}

/**
 * @brief Runs ESCREVERSE: sets the lowest pulse brake and reverse may reach.
 *
 * @param arg "<us>", from ESC_HARD_MIN up to DRIVE_NEUTRAL_US. Neutral itself
 *            is reverse OFF, which is also where it starts at boot.
 * @return false when the argument is not a number or the limit was refused;
 *         true once it is in force
 *
 * @warning On this car's Forward/Reverse/Brake ESC a pulse below neutral
 *          brakes, and after a return to neutral reverses. Setting this is
 *          what makes that reachable.
 */
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

/**
 * @brief Runs SERVO: releases or engages the steering pulse, centers it, or
 *        drives it to a pulse width.
 *
 * @param arg "OFF" to release the pulse (no holding torque), "ON" to engage
 *            it, "CENTER" (or "CENTRE") to center it, or microseconds within
 *            the current servo range
 * @return false when the argument is neither ON/OFF/CENTER nor a number;
 *         true otherwise. A microsecond value outside the range is CLAMPED
 *         rather than refused, so it counts as accepted.
 *
 * @warning A microsecond value moves the servo immediately if it is engaged;
 *          while released it is only remembered until SERVO ON.
 */
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

/**
 * @brief Runs ESC: arms or disarms the throttle, sets it to neutral, or
 *        drives it to a pulse width.
 *
 * @param arg "ARM", "DISARM", "NEUTRAL", or microseconds within the current
 *            ESC range
 * @return false when the argument is neither ARM/DISARM/NEUTRAL nor a number,
 *         and when a throttle was refused because the ESC is not armed; true
 *         otherwise. A refused throttle does not feed the watchdog: a host
 *         asking for what it may not have is not driving the car.
 *
 * @warning Arming and a nonzero throttle are what let the car move. A
 *          microsecond value is refused while disarmed rather than stored, so
 *          ARM never surprises anyone with a car that immediately moves.
 *
 * @note ARM opens the gate on OUR side only. The QuicRun 10BL160 G2 must see
 *       neutral when it powers up, and ignores everything until it does - which
 *       reads exactly like a dead ESC.
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

    /* The module owns the arming rule; this only reports it. */
    if(!bibo::drive::throttleUs(us))
    {
        bibo::serial::printf("ERR esc not armed - send ESC ARM first\n");

        return false;
    }

    return printDrive();
}

/*
 * ---- the command table ---------------------------------------------------
 *
 * One row per command; the dispatcher and HELP both read it. Matching is by
 * WHOLE WORD, so row order means nothing: SERVO does not match SERVOTRIM.
 */
/**
 * @brief A command handler: receives the text after the command word.
 *
 * @param arg everything in the line after the command name and the spaces
 *            that follow it, already uppercased
 *
 * @return true when the command PARSED COMPLETELY and its values were in
 *         range - which is what feeds the watchdog. False for anything the
 *         handler answered with ERR.
 */
typedef Bool (*CmdRun)(CharSeq arg);

/**
 * @brief One row of the command table: a name to match, its usage for HELP,
 *        one line about what it does, and the handler to run.
 */
struct Command
{
    CharSeq name;
    CharSeq usage;
    CharSeq what;
    CmdRun  run;
};

/* Defined below the table, which it walks. */
static Bool printHelp(CharSeq arg);

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
};

/**
 * @brief Runs HELP (and "?"): lists every command, its usage, and one line
 *        about what it does.
 *
 * @param arg unused; HELP takes no argument
 * @return true - HELP takes no argument, so there is nothing to reject
 */
static Bool printHelp(const CharSeq arg)
{
    static_cast<Void>(arg);
    for(const auto& i : COMMANDS)
    {
        bibo::serial::printf("INFO help %s%s - %s\n", i.name, i.usage, i.what);
    }

    return true;
}

/**
 * @brief Parses one received line and dispatches it to its command handler.
 *
 * @param line the line as received, NUL-terminated; rewritten in place to
 *             uppercase before matching against COMMANDS
 *
 * @note ONLY AN ACCEPTED COMMAND FEEDS THE WATCHDOG. An unknown command, a bad
 *       argument and a truncated line all leave the clock where it was.
 */
static Void handleLine(Utf8* line)
{
    /* A terminal picks its own line ending; without this "PING\r" is not "PING". */
    if(bibo::text::trimEnd(line) == 0)
    {
        return;
    }

    bibo::text::upper(line);

    /* Whether the command parsed and was in range; only the handler knows. */
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

/* ------------------------------------------------------------------ main -- */

/*
 * `int`, not Int32: C++ requires main to return literally `int`, and Int32 is
 * `long int` on arm-none-eabi.
 */
/**
 * @brief Entry point: installs the pin map, opens the drivetrain and the LED,
 *        then loops forever pumping the outputs, running the watchdog, and
 *        dispatching whatever arrives on the serial line.
 *
 * @return never - the loop runs until the board is reset or rebooted
 */
int main(Void)
{
    bibo::serial::open();

    /*
     * The pin map before anything opens: every subsystem reads it then, and
     * until begin() succeeds every role is NONE. pins.hxx proves the car's map
     * at compile time, so this branch is for a bad edit to car().
     */
    if(!bibo::pins::begin(bibo::pins::car()))
    {
        bibo::serial::printf("ERR %s\n", bibo::pins::conflictText());
    }

    /* ESC to neutral, steering RELEASED - chassis.hxx says why those differ. */
    bibo::drive::open();

    /*
     * On the Pico 2 W the LED hangs off the CYW43439 and can fail to come up,
     * which ID reports as lamp_up=no. On the plain Pico 2 it is a GPIO.
     */
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

        /* Walks the servo and ESC toward their targets, one step per SLEW_TICK_MS. */
        bibo::drive::pump();

        /*
         * ---- the watchdog -------------------------------------------------
         *
         * IN THE LOOP, because it has to fire exactly when no command is being
         * handled: a blocked read, a killed host and a pulled cable all look the
         * same from here. The throttle first, then the report, since
         * serial::printf can block while the host is not draining the port.
         */
        if(!stale && (bibo::timing::nowMs() - lastValidCmdMs) > WATCHDOG_MS)
        {
            const bibo::drive::State dm = bibo::drive::read();

            /* Below neutral is brake and reverse, which is driving too. */
            const Bool driving = dm.escArmed
                              && (dm.escTargetUs > dm.escMinUs || dm.escTargetUs < DRIVE_NEUTRAL_US);

            /* Neutral now, still armed, steering held - see drive::throttleNeutralNow(). */
            bibo::drive::throttleNeutralNow();
            stale = true;

            /*
             * Every expiry sets stale; only a car that was being driven says so,
             * or a board on a bench would complain after every typed command.
             */
            if(driving)
            {
                bibo::serial::printf(
                    "ERR watchdog - no valid command for %u ms, throttle neutral\n",
                    WATCHDOG_MS
                );
            }
        }

        /* Output before the host opens the port is discarded, so this waits. */
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
            announced = false;      /* re-announce on the next connection */
        }

        const Int32 c = bibo::serial::readChar(POLL_TIMEOUT_US);
        if(c == bibo::serial::NONE)
        {
            continue;
        }

        if(c == '\n' || c == '\r')
        {
            /* A line that overran is discarded at its END - see below. */
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
            /* Swallowing the rest of an over-long line. */
        }
        else if(len + 1 < LINE_CAP)
        {
            line[len++] = static_cast<Utf8>(c);
        }
        else
        {
            /*
             * The overlong flag drops the TAIL too; resetting len alone would
             * parse bytes 129 onward as a fresh command nobody sent.
             */
            len = 0;
            overlong = true;
            bibo::serial::printf("ERR line too long\n");
        }
    }
}
