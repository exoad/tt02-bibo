/*
 * main - the car's program on the Pico 2 W. Declares the pin map, opens every
 * subsystem, and serves the line protocol the hub drives the car through:
 * newline-terminated ASCII over USB CDC, and over the wireless link once WIFI
 * JOIN has connected one. Every command answers with exactly one line starting
 * OK / ERR / INFO / PONG. The command list is COMMANDS, below, which is also
 * what HELP prints.
 *
 * NOTE ON THE LED: on Pico 2 W the user LED is NOT an RP2350 GPIO. It hangs off
 * the CYW43439 wireless chip, so it needs cyw43_arch_init() before it will do
 * anything. On a non-W Pico the same LED is plain GPIO 25. Getting this wrong
 * produces firmware that runs perfectly and never blinks.
 *
 * Nothing here touches the Pico SDK or a GPIO number directly.
 */

/* The whole library; an application includes this and nothing else of ours. */
#include "../lib/bibo.hxx"


/*
 * Not LINE_MAX: POSIX reserves that name and <limits.h> defines it on some
 * newlib configurations, making this a redefinition rather than a declaration.
 */
#define LINE_CAP 128

#define BLINK_MAX_HZ 50.0f
#define IDLE_BLINK_HZ 0.5f

/* Three quick flashes at power-on, before any host has opened the port. */
#define HELLO_FLASHES 3
#define HELLO_FLASH_MS 80

/* 1 ms, which keeps the blink smooth while still returning promptly. */
#define POLL_TIMEOUT_US 1000

/*
 * ---- the deadman ----------------------------------------------------------
 *
 * How long the board keeps DRIVING with nothing heard from the host. Without
 * it, a hub that crashed while the ESC was armed left bibo::drive::pump()
 * writing the last throttle to the pin forever.
 *
 * 400 ms, not the 200 in docs/conventions.md: the hub's own DRIVE poll runs at
 * 250 ms and a keyboard controller sends on key CHANGES rather than on a timer,
 * so 200 would trip on somebody holding W steadily. Applies only while armed
 * AND commanded above idle.
 */
#define DEADMAN_MS 400u

/* -------------------------------------------------------------- commands -- */

/**
 * @brief Reports whether the board's status lamp came up, whatever it
 *        physically is on this board.
 *
 * @return "yes" once bibo::led::present() is true, "no" otherwise
 */
static CharSeq lampWord(Void)
{
    return bibo::led::present() ? "yes" : "no";
}

/**
 * @brief Builds the trailing " cyw43=up" field for ID and STATUS, present
 *        only on a board that has the wireless chip.
 *
 * Absent, not false, on the plain Pico 2. There is no wireless chip in that
 * package to be down, and the hub reads this field to light a chip indicator:
 * reporting cyw43=FAILED there would paint a hardware fault on a board that is
 * working perfectly. A field that does not apply is one you leave out, and the
 * hub already treats "not mentioned" as "nothing to say".
 *
 * @return " cyw43=up" or " cyw43=FAILED" on a CYW43-equipped board, or "" on
 *         a board with no wireless chip to report on
 */
static CharSeq cyw43Field(Void)
{
#if defined(CYW43_WL_GPIO_LED_PIN)
    return bibo::led::present() ? " cyw43=up" : " cyw43=FAILED";
#else
    return "";
#endif
}

/**
 * @brief Answers ID: board, SDK version, build timestamp, unique id, and
 *        lamp state, as one INFO line.
 *
 * @param arg unused; ID takes no argument
 */
static Void printId(const CharSeq arg)
{
    static_cast<Void>(arg);

    Utf8 uid[24];
    bibo::board::id(uid, sizeof(uid));

    bibo::serial::printf("INFO id board=%s sdk=%s built=%s %s uid=%s lamp=%s lamp_up=%s%s\n",
           PICO_BOARD, PICO_SDK_VERSION_STRING, __DATE__, __TIME__,
           uid, bibo::led::backend(), lampWord(), cyw43Field());
}

/**
 * @brief Answers STATUS: uptime, board, lamp state and blink rate, as one
 *        INFO line.
 *
 * board= is on THIS line as well as on ID, because STATUS is the line
 * anything polls. The hub asks for it every couple of seconds and asks for
 * ID only when a person clicks something - so a field that appears solely
 * on ID is a field the hub almost never has. There are two boards in this
 * project now and telling them apart is worth six characters a poll.
 *
 * @param arg unused; STATUS takes no argument
 */
static Void printStatus(const CharSeq arg)
{
    static_cast<Void>(arg);

    bibo::serial::printf("INFO status up_ms=%u board=%s led=%s blink_hz=%.2f lamp=%s lamp_up=%s%s\n",
           bibo::timing::nowMs(),
           PICO_BOARD,
           bibo::status::isLit() ? "on" : "off",
           static_cast<Float64>(bibo::status::rate()),
           bibo::led::backend(), lampWord(), cyw43Field());
}

/**
 * @brief The tail appended to an LED reply when there is nothing to light.
 *
 * An OK that did nothing has to say so, or a dead lamp looks like a dead
 * command parser - and the two boards fail this differently, so they say it
 * differently.
 *
 * @return "" when a lamp is present; otherwise a parenthesized reason why
 *         the command had no effect
 */
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

/**
 * @brief Runs LED: turns the status lamp on solid, off, or blinking at a rate.
 *
 * @param arg "ON", "OFF", or "BLINK <hz>"; 0 hz stops the blink
 *
 * @note Matched uppercase because handleLine() uppercases the whole line
 *       before dispatch, so ON, OFF and BLINK are recognized whatever case
 *       they were typed in.
 */
static Void handleLed(const CharSeq arg)
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
        /* bibo::text::after rather than arg + 5: no hand-written offset to go stale. */
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

/*
 * ================================================================ sensors ==
 *
 * Detection is a fact rather than a guess: a sensor is present if it
 * acknowledges its address AND identifies itself, so something else living at
 * 0x29 is reported as absent rather than as a broken VL53L1X.
 */

#define SENSOR_SDA 4
#define SENSOR_SCL 5
#define SENSOR_HZ  400000u

/* The scan range; anything found outside the named set prints as a number. */
#define ADDR_SCAN_FIRST 0x08
#define ADDR_SCAN_LAST  0x77

/* When the last command arrived; the flag stops the deadman re-firing every ms. */
static UInt32 lastCmdMs      = 0;
static Bool   deadmanTripped = false;

/*
 * ---- the line, as it was actually typed -----------------------------------
 *
 * handleLine() uppercases what it is given; a Wi-Fi password does not survive
 * that, and the failed join reads as bad credentials rather than as the console
 * changing what was typed. bibo::text::upper() rewrites in place and does not
 * change the LENGTH of anything, so an offset into the uppercased line is the
 * same offset into this raw copy - cmdRawArg. Nothing but WIFI needs it.
 */
static Utf8    rawLine[LINE_CAP];
static CharSeq cmdRawArg = "";

/* Which wireless state has been announced, so a change is reported once. */
static bibo::net::State netReported = bibo::net::STATE_ABSENT;

static Bool i2cUp   = false;
static bibo::tof::Vl53 tofFront;
static Bool tofUp   = false;

/**
 * @brief Brings up the I2C bus and the front ToF sensor, if either is there.
 *
 * Called once at boot. Failing to find a device is not an error here - it is
 * the answer SENSORS and TOF report back.
 */
static Void sensorsOpen(Void)
{
    i2cUp = bibo::i2c::open(SENSOR_SDA, SENSOR_SCL, SENSOR_HZ);
    if(!i2cUp)
    {
        return;
    }

    tofUp = bibo::tof::open(&tofFront, VL53_ADDR_DEFAULT);
    if(tofUp)
    {
        /*
         * Checked: a sensor that opened but never STARTED reports "not ready"
         * for the rest of the session and looks identical to one that is slow.
         */
        tofUp = bibo::tof::startRanging(&tofFront);
    }
}

/**
 * @brief Answers SENSORS: which sensors this board found at boot, as one
 *        OK line.
 *
 * The hub parses this at connect, which is how its sensor rows learn whether
 * they are real. Shaped as key=value pairs so a reader that does not know
 * about a sensor added later ignores it rather than failing to parse the line.
 *
 * @param arg unused; SENSORS takes no argument
 */
static Void printSensors(const CharSeq arg)
{
    static_cast<Void>(arg);

    bibo::serial::printf("OK sensors i2c=%d tof=%d tof_addr=0x%02X\n",
           i2cUp ? 1 : 0, tofUp ? 1 : 0, VL53_ADDR_DEFAULT);
}

/**
 * @brief Runs SCAN: probes every I2C address and reports each one that
 *        acknowledges.
 *
 * The same job as the standalone scanner sketch, available over the link so
 * the hub can offer it too.
 *
 * @param arg unused; SCAN takes no argument
 */
static Void printScan(const CharSeq arg)
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

/**
 * @brief Answers TOF with no MODE argument: the front sensor's current range.
 *
 * Reports the STATUS as well as the number, always. A distance that came with
 * a bad status is not a shorter distance - it is not a distance - and a host
 * that only got the number would have no way to know that. Replies "ERR tof
 * absent" with no sensor, "OK tof busy" while a measurement is still in
 * flight, or "OK tof <mm> <status> <signal> <ambient>".
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

        /*
         * Read BEFORE the interrupt is cleared - they belong to THIS
         * measurement. They turn "83 mm" into a diagnosis: a strong signal at a
         * short distance means something really is that close (a protective
         * film still on the lens, say); a weak signal with a high ambient means
         * the sensor is being blinded by infrared in the room.
         */
        UInt16 sig = 0;
        UInt16 amb = 0;
        static_cast<Void>(bibo::tof::rates(&tofFront, &sig, &amb));

        bibo::tof::clear(&tofFront);
        bibo::serial::printf("OK tof %u %u %u %u\n", mm, st, sig, amb);
        return;
    }

    /* Not an error: a measurement takes tens of ms and the host may ask faster. */
    bibo::serial::printf("OK tof busy\n");
}

/**
 * @brief Runs TOF MODE: stops ranging, switches the sensor between short-
 *        and long-distance mode, and restarts ranging.
 *
 * Changing the VCSEL period and the phase windows underneath a running
 * measurement leaves the sensor half-configured for as long as that
 * measurement lasts, and what it does with the result is undefined. ST's own
 * driver brackets it this way and so does this.
 *
 * @param arg "SHORT" (up to about 1.3 m) or "LONG" (up to about 4 m)
 *
 * @note All three steps - stop, reconfigure, start - are attempted even when
 *       an earlier one fails, so ranging is never left stopped by a failure
 *       partway through; the reply says which of the three did not take.
 */
static Void handleTofMode(const CharSeq arg)
{
    if(!tofUp)
    {
        bibo::serial::printf("ERR tof absent\n");
        return;
    }

    /*
     * Stop, reconfigure, start - and all three results are CHECKED. Printing OK
     * after a failed setMode would tell the operator the sensor is in a mode it
     * is not in, and every reading after that is read against the wrong
     * assumption about range and ambient rejection.
     *
     * NOT chained with &&: short-circuiting would skip clearInterruptAndStart
     * whenever setMode failed, leaving ranging stopped - the sensor then answers
     * "not ready" forever and nothing says why.
     */
    if(bibo::text::eq(arg, "SHORT") || bibo::text::eq(arg, "LONG"))
    {
        const Bool wantShort = bibo::text::eq(arg, "SHORT");

        const Bool stopped = bibo::tof::stopRanging(&tofFront);
        const Bool moded   = bibo::tof::setMode(&tofFront,
                                                wantShort
                                                    ? bibo::tof::MODE_SHORT
                                                    : bibo::tof::MODE_LONG);
        const Bool started = bibo::tof::clearInterruptAndStart(&tofFront);

        if(stopped && moded && started)
        {
            bibo::serial::printf("OK tof mode %s\n",
                                 wantShort ? "short" : "long");
        }
        else
        {
            /*
             * Which of the three failed: a stop that will not take is a bus
             * problem, a mode that will not take is a refused register write,
             * and a start that will not take leaves the sensor idle.
             */
            bibo::serial::printf(
                "ERR tof mode %s failed: stop=%d set=%d start=%d\n",
                wantShort ? "short" : "long",
                static_cast<Int32>(stopped),
                static_cast<Int32>(moded),
                static_cast<Int32>(started));
        }
        return;
    }
    bibo::serial::printf("ERR bad mode: %s\n", arg);
}

/*
 * ================================================================== drive ==
 *
 * Console glue over lib/chassis. Everything about what is SAFE lives in the
 * module; everything here is about what to SAY. A console, a sketch and an
 * autonomy loop each carrying their own "refuse throttle until armed" is three
 * copies of a rule - the module refuses, this file reports the refusal.
 */

/**
 * @brief Answers DRIVE: the servo and ESC state, as one OK line.
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

/**
 * @brief Runs STEER: sets the steering as a fraction of this car's travel.
 *
 * @param arg a number from -1.0 to 1.0; empty is rejected rather than
 *            treated as center, since a truncated command is more likely
 *            than a request to center
 *
 * @warning Moves the steering servo immediately if it is engaged.
 */
static Void handleSteer(const CharSeq arg)
{
    /* Guessing that a bare STEER means zero would turn a typo into a movement. */
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

/**
 * @brief Runs SLEW: sets how fast the steering, the throttle, or both may
 *        move, in microseconds per tick.
 *
 * With no target, sets both - which is what "the response rate" meant when
 * there was only one, and is still the common case on a bench where you want
 * everything slow while watching something.
 *
 * @param arg "<us>", or "STEER <us>" / "THROTTLE <us>" to set one alone
 */
static Void handleSlew(const CharSeq arg)
{
    CharSeq rest = bibo::text::word(arg, "STEER");
    if(rest != nullptr)
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
    if(rest != nullptr)
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
     * Also reported in a unit a person thinks in: "8 us per tick" is a number
     * nobody has intuition about; "400 us/s, full travel 1100 ms" decides
     * whether it is fast enough to steer around something.
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

/**
 * @brief Runs SERVOTRIM: moves where center is, without moving the endpoints.
 *
 * @param arg the new center, in microseconds, within the current servo range
 */
static Void handleTrim(const CharSeq arg)
{
    Int32 us = 0;
    if(!bibo::text::toInt(arg, &us))
    {
        const bibo::drive::State d = bibo::drive::read();
        bibo::serial::printf("ERR trim wants microseconds, %d-%d\n", d.servoMinUs, d.servoMaxUs);
        return;
    }

    bibo::drive::trim(us);
    bibo::serial::printf("INFO center is now %d us\n", bibo::drive::read().centerUs);
    printDrive();
}

/**
 * @brief Shared body for SERVOLIMITS and ESCLIMITS: parses "<min> <max>" and
 *        applies them through the given setter.
 *
 * They were written out twice, identically apart from the word in the error
 * messages and the function called - and the two copies had already drifted
 * once, because the ordering bug fixed in bibo::drive::setSteerLimits had to
 * be remembered a second time for the throttle.
 *
 * @param arg the command's argument, expected to be "<min> <max>"
 * @param name the command word to use in an error message ("servolimits" or
 *             "esclimits")
 * @param set the setter to apply lo/hi through, once both parse
 */
static Void limitsCommand(const CharSeq arg, const CharSeq name, Bool (*set)(Int32, Int32))
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

/**
 * @brief Runs SERVOLIMITS: widens or narrows the steering's microsecond
 *        range, for finding the real end stops.
 *
 * @param arg "<min> <max>", in microseconds
 */
static Void handleLimits(const CharSeq arg)
{
    limitsCommand(arg, "servolimits", bibo::drive::setSteerLimits);
}

/**
 * @brief Runs ESCLIMITS: widens or narrows the throttle's microsecond range.
 *
 * @param arg "<min> <max>", in microseconds
 */
static Void handleEscLimits(const CharSeq arg)
{
    limitsCommand(arg, "esclimits", bibo::drive::setThrottleLimits);
}

/**
 * @brief Runs SERVO: releases or engages the steering pulse, centers it, or
 *        drives it to a specific pulse width.
 *
 * @param arg "OFF" to release the pulse (no holding torque), "ON" to engage
 *            it, "CENTER" (or "CENTRE") to center it, or a number of
 *            microseconds within the current servo range
 *
 * @warning "OFF" drops the pulse entirely - the steering goes limp. A bare
 *          microsecond value moves the servo immediately if it is already
 *          engaged; while released it is only remembered until SERVO ON.
 */
static Void handleServo(const CharSeq arg)
{
    /*
     * OFF stops the pulse train outright - the panic button. A servo leaning on
     * a frame needs to stop being told to hold a position at all.
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

    /* Remembered, not obeyed: engaging is a separate act, like arming the ESC. */
    const Bool wasLive = bibo::drive::read().servoLive;
    bibo::drive::steerUs(us);
    if(!wasLive)
    {
        bibo::serial::printf("INFO servo is released - target stored, send SERVO ON\n");
    }
    printDrive();
}

/**
 * @brief Runs ESC: arms or disarms the throttle, sets it to neutral, or
 *        drives it to a specific pulse width.
 *
 * @param arg "ARM", "DISARM", "NEUTRAL", or a number of microseconds within
 *            the current ESC range
 *
 * @warning Arming and a nonzero throttle value are what let the car move.
 *          A microsecond value is refused with an error while disarmed
 *          rather than stored for later, so ARM does not surprise anyone
 *          with a car that immediately moves.
 */
static Void handleEsc(const CharSeq arg)
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


/*
 * LIGHTS - the indicator scaffolding. TEMPORARY, see lib/lights.h.
 *
 * Reports which way the car thinks it is turning and whether the lamp is lit
 * this instant; the hub polls it to draw the pair in the Drive view. The "lit"
 * half is what shows the rule is running rather than the LED merely being on.
 */
/* The lamp names, in Lamp order, so the reply reads the way the model does. */
static CharSeq LAMP_NAME[bibo::lights::LAMP_COUNT] =
{
    "headL", "headR", "tailL", "tailR",
    "indFL", "indFR", "indRL", "indRR",
    "revL",  "revR"
};

/**
 * @brief Answers LIGHTS: every lamp's level and pin, the turn signal, and
 *        any forced lamp, as one OK line.
 *
 * Every lamp in the model is reported, bound to a pin or not. A lamp with no
 * LED on it still has a correct answer, and printing it is what makes wiring
 * the next one a matter of checking a number that was already there rather
 * than trusting that a rule nobody has ever seen run is right.
 *
 * @param arg unused; kept only so this has the same signature as a command
 *            handler and can be called after one runs
 */
static Void printLights(const CharSeq arg)
{
    static_cast<Void>(arg);

    const bibo::lights::Set s = bibo::lights::read();
    const bibo::cue::Turn t = bibo::cue::side();
    const Int32   f = bibo::lights::forcedLamp();

    /*
     * ONE line, not one per lamp: the hub polls this every 120 ms, and eight
     * extra lines a poll is seventy-five lines a second of serial traffic to
     * say what fits in one. levels[] and pins[] are in Lamp order, which is the
     * order LAMP_NAME is in.
     */
    bibo::serial::printf("OK lights on=%d turn=%s forced=%s off_us=%d"
                 " levels=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u"
                 " pins=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                 bibo::lights::enabled() ? 1 : 0,
                 (t == bibo::cue::TURN_LEFT)  ? "left"
                     : (t == bibo::cue::TURN_RIGHT)  ? "right"
                     : (t == bibo::cue::TURN_HAZARD) ? "hazard" : "off",
                 (f == bibo::lights::LAMP_COUNT) ? "no" : LAMP_NAME[f],
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

/**
 * @brief Runs LIGHTS: the master lamp switch, forcing one lamp on for
 *        wiring checks, or setting how far past idle counts as "driving".
 *
 * A lamp NAME forces that one lamp on and everything else off, so an LED and
 * its wiring can be checked without touching the ESC or the steering. AUTO
 * hands it back to the rules.
 *
 * @param arg "ON", "OFF", "AUTO", "OFFAT <us>", a lamp name, or empty to
 *            just report the current state
 */
static Void handleLights(const CharSeq arg)
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
        bibo::lights::forceLamp(bibo::lights::LAMP_COUNT);
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

    /* handleLine has already uppercased the line; LAMP_NAME is model-spelled. */
    for(Int32 i = 0; i < bibo::lights::LAMP_COUNT; ++i)
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

/*
 * ---- the command table ---------------------------------------------------
 *
 * One row per command; the dispatcher and HELP both read it. It replaced a
 * chain of if(bibo::text::starts(...)) blocks with hand-written argument
 * offsets, where miscounting one merely stops a command working. Matching is by
 * WHOLE WORD, so row order means nothing - the old chain worked only because
 * SERVOTRIM and SERVOLIMITS happened to be tested before SERVO.
 */
/**
 * @brief A command handler: receives the text after the command word.
 *
 * @param arg everything in the line after the command name and the space
 *            that follows it, already uppercased except through cmdRawArg
 */
typedef Void (*CmdRun)(CharSeq arg);

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
static Void printHelp(CharSeq arg);

/**
 * @brief Runs TOF: reports the current range, or dispatches "MODE
 *        SHORT|LONG" to handleTofMode.
 *
 * TOF's subcommand is kept here rather than as a second row: "TOF MODE LONG"
 * is an argument to TOF, not a command called "TOF MODE", and whole-word
 * matching would hand the whole thing to TOF anyway.
 *
 * @param arg empty to report the range, or "MODE SHORT" / "MODE LONG"
 */
static Void cmdTof(const CharSeq arg)
{
    const CharSeq mode = bibo::text::word(arg, "MODE");
    if(mode != nullptr)
    {
        handleTofMode(mode);
        return;
    }
    printTof();
}

/**
 * @brief Runs PING: answers PONG, proving the link is alive.
 *
 * @param arg unused; PING takes no argument
 */
static Void cmdPing(const CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::serial::printf("PONG\n");
}

/**
 * @brief Runs DRIVE: reports the servo and ESC state.
 *
 * @param arg unused; DRIVE takes no argument
 */
static Void cmdDrive(const CharSeq arg)
{
    static_cast<Void>(arg);
    printDrive();
}

/**
 * @brief Runs STOP: the one command that has to work when nothing else is
 *        going right - throttle to neutral and disarmed, steering released,
 *        any forced lamp dropped, and any cue silenced.
 *
 * The drivetrain first, because that is the part that can hurt somebody:
 * throttle to neutral and disarmed, steering RELEASED rather than centered.
 * Released is the stronger claim - center is only a safe place to leave a
 * servo if 1500 us is where the linkage wants to sit, and on a car whose horn
 * is a tooth off its spline it is not. Nothing to push with is the only stop
 * that works on every car.
 *
 * Then any FORCED lamp is dropped. That is not a safety matter - a lamp
 * cannot hurt anyone - but a lamp being held on by hand is an output somebody
 * is commanding, and a stop that leaves an output commanded is not a stop.
 * The lamps go back to following the car, which with the throttle now at
 * neutral means the tails come on. That is correct: the car is not being
 * driven.
 *
 * @param arg unused; STOP takes no argument
 *
 * @warning This is the emergency stop. It disarms the ESC and releases the
 *          steering servo (no holding torque) rather than merely centering it.
 */
static Void cmdStop(const CharSeq arg)
{
    static_cast<Void>(arg);

    bibo::drive::stop();
    bibo::lights::forceLamp(bibo::lights::LAMP_COUNT);

    /* Mid-sentence is still an output being commanded. */
    bibo::cue::silence();

    bibo::serial::printf("OK stop\n");
}

/**
 * @brief Runs WIFI: reports the wireless link's state, or joins a network.
 *
 * The credentials are taken from the RAW line, not the uppercased one, and
 * they are never written to flash or to this repository - a reset loses them.
 * That is a deliberate cost: this repository is pushed, and a password in a
 * source file is a password in the history forever.
 *
 * An SSID containing a space cannot be expressed here. The password can - it
 * is everything after the first space following the SSID, verbatim.
 *
 * @param arg empty to report link status, or "JOIN <ssid> <password>" with
 *            an empty password meaning an open network
 *
 * @note Does not report whether the join WORKED - only that it started. The
 *       main loop reports the state when it changes.
 */
static Void cmdWifi(const CharSeq arg)
{
    if(!bibo::net::present())
    {
        bibo::serial::printf("ERR wifi no radio on this board (%s)\n", PICO_BOARD);
        return;
    }

    const CharSeq rest = bibo::text::word(arg, "JOIN");
    if(rest == nullptr)
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
    const CharSeq raw = cmdRawArg + (rest - arg);

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
        /* A truncated SSID would fail to join and look like it was out of range. */
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

    /* Not whether it WORKED - the main loop reports the state when it changes. */
    bibo::serial::printf("OK wifi joining %s\n", ssid);
}

/*
 * CUE - what the car is saying, and telling it to say something.
 *
 *   CUE               where it stands
 *   CUE LIST          every cue this firmware knows, and what each one means
 *   CUE <name>        raise it. A held or looping cue stays up until lowered;
 *                     a one-shot plays and ends on its own.
 *   CUE <name> OFF    lower it, and hand it back to the car's own rules
 *   CUE STOP          lower everything
 *
 * The lamps are reported by LIGHTS; this reports the UTTERANCE - which one, how
 * far through it is, and what it would be sounding if there were a buzzer.
 */
/**
 * @brief Answers CUE with no argument (and after every CUE command): what
 *        the car is saying, as one OK line.
 *
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
static Void printCue(Void)
{
    const bibo::cue::Kind k = bibo::cue::speaking();

    Utf8 list[128];
    Size at = 0;
    list[0] = '\0';

    for(Int32 i = 1; i < bibo::cue::KIND_COUNT; ++i)
    {
        const auto c = static_cast<bibo::cue::Kind>(i);
        if(!bibo::cue::on(c))
        {
            continue;
        }

        const Int32 n = bibo::text::format(&list[at], sizeof(list) - at, "%s%s%s",
                                          at > 0 ? "," : "",
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
        list[0] == '\0' ? "-" : list,
        static_cast<UInt32>(bibo::cue::step()),
        static_cast<UInt32>(bibo::cue::loop()),
        static_cast<UInt32>(bibo::cue::tone()),
        static_cast<Int32>(bibo::cue::KIND_COUNT - 1),
        bibo::cue::motionUs());
}

/**
 * @brief Runs CUE: reports what the car is saying, lists every cue this
 *        firmware knows, raises or lowers one by name, or silences all of
 *        them.
 *
 * @param arg empty to report, "LIST" to list every cue and what it means,
 *            "STOP" to lower everything, "<name>" to raise a cue, or
 *            "<name> OFF" to lower it and hand it back to the car's own rules
 */
static Void cmdCue(const CharSeq arg)
{
    if(arg[0] == '\0')
    {
        printCue();
        return;
    }

    if(bibo::text::eq(arg, "LIST"))
    {
        /* One line each, the way HELP does it - no second list to keep in step. */
        for(Int32 k = 1; k < bibo::cue::KIND_COUNT; ++k)
        {
            /*
             * The PLAY MODE is in the line because a caller has to know it:
             * `once` ends on its own, `loop` and `hold` stay up until lowered.
             */
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

    /* "CUE LEFT OFF" - split before the lookup, so no cue is named "LEFT OFF". */
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

/**
 * @brief Runs BOOTSEL: reboots the board into the UF2 bootloader.
 *
 * @param arg unused; BOOTSEL takes no argument
 *
 * @note Does not return - bibo::board::rebootToBootsel() flushes output and
 *       then reboots the board.
 */
static Void cmdBootsel(const CharSeq arg)
{
    static_cast<Void>(arg);
    bibo::serial::printf("INFO rebooting into bootloader\n");
    bibo::board::rebootToBootsel();          /* flushes, then does not return */
}


/*
 * ===========================================================================
 * SOUND - the DFPlayer Mini. TEMPORARY bench scaffolding, not the shape sound
 * will have on the finished car: sound BELONGS in the cue system, and cue.hxx
 * already carries a `tone` field waiting for it. Nothing here reacts to
 * anything; every function is a person pressing a button. The state lives in
 * lib/sound.hxx - what is left here is console glue.
 * ========================================================================
 */

/**
 * @brief Answers SOUND with no argument (and after every SOUND command):
 *        the DFPlayer's readiness, volume, EQ, current track, and wiring,
 *        as one OK line.
 *
 * The CLIP NAME is included beside the track number, when the track has one.
 * A status line that says track 2 makes a person open sfx.hxx to find out
 * what that is.
 */
static Void printSound(Void)
{
    const CharSeq clip = bibo::sfx::nameOf(bibo::sound::track());

    bibo::serial::printf(
        "OK sound ready=%s vol=%u max=%u eq=%u track=%u clip=%s files=%u "
        "busy=%s tx=%d rx=%d busyGp=%d\n",
        bibo::sound::ready() ? "yes" : "no",
        static_cast<UInt32>(bibo::sound::volume()),
        static_cast<UInt32>(DFP_VOLUME_MAX),
        static_cast<UInt32>(bibo::sound::eq()),
        static_cast<UInt32>(bibo::sound::track()),
        (clip != nullptr) ? clip : "-",
        static_cast<UInt32>(bibo::sound::count()),
        !bibo::sound::hasVoice()
            ? "unwired"
            : (bibo::sound::speaking() ? "yes" : "no"),
        static_cast<Int32>(bibo::pins::active().soundTx),
        static_cast<Int32>(bibo::pins::active().soundRx),
        static_cast<Int32>(bibo::pins::active().soundBusy));
}

/**
 * @brief Runs SOUND: the DFPlayer console - report state, remount the card,
 *        set volume or EQ, play a clip by name or number, list known clips,
 *        or control playback directly.
 *
 * @param arg empty to report state, or one of RESET, VOL <0-30>, EQ <0-5>,
 *            PLAY <name|n>, LIST, FILES, RX, STOP, PAUSE, RESUME, NEXT, PREV
 */
static Void cmdSound(const CharSeq arg)
{
    if(arg[0] == '\0')
    {
        printSound();
        return;
    }

    if(bibo::text::eq(arg, "RESET"))
    {
        /* Two seconds of nothing, said BEFORE it happens: quiet looks hung. */
        bibo::serial::printLine("INFO sound resetting, waiting for the card");
        static_cast<Void>(bibo::sound::mount());

        printSound();
        return;
    }

    if(bibo::text::starts(arg, "VOL"))
    {
        Int32 v = 0;
        if(!bibo::text::toInt(bibo::text::after(arg, "VOL "), &v))
        {
            bibo::serial::printLine("ERR sound VOL wants a number 0-30");
            return;
        }
        if(v < 0 || v > static_cast<Int32>(DFP_VOLUME_MAX))
        {
            bibo::serial::printf("ERR sound volume out of range 0-%u\n",
                                 static_cast<UInt32>(DFP_VOLUME_MAX));
            return;
        }

        bibo::sound::setVolume(static_cast<UInt8>(v));
        printSound();
        return;
    }

    if(bibo::text::starts(arg, "PLAY"))
    {
        /*
         * A NAME OR A NUMBER, because a number is what you have before a clip
         * has earned a name. THE DECIDING IS sound::play's - everything below
         * turns a Result into a sentence, which is why a cue can make the same
         * call without reimplementing any of it.
         */
        const CharSeq want = bibo::text::after(arg, "PLAY ");

        bibo::sound::Result r = bibo::sound::RESULT_OK;

        if(want[0] == '\0')
        {
            /* Bare PLAY repeats - what a button on a test view means by it. */
            r = bibo::sound::playTrack(bibo::sound::track());
        }
        else
        {
            r = bibo::sound::play(want);

            /* Not a known name - try it as a number before giving up. */
            if(r == bibo::sound::RESULT_NO_CLIP)
            {
                Int32 n = 0;
                if(bibo::text::toInt(want, &n) && n >= 1 && n <= 3000)
                {
                    r = bibo::sound::playTrack(static_cast<UInt16>(n));
                }
            }
        }

        if(r != bibo::sound::RESULT_OK)
        {
            /* WHICH failure: all four sound identical, and need four fixes. */
            bibo::serial::printf("ERR sound %s: %s\n",
                                 bibo::sound::why(r), want);
            return;
        }

        printSound();
        return;
    }

    /*
     * SOUND EQ <0-5> - tone, NOT level. It cannot raise the ceiling: 30 is the
     * top of the protocol's range and the module clamps there.
     */
    if(bibo::text::starts(arg, "EQ"))
    {
        Int32 m = 0;
        if(!bibo::text::toInt(bibo::text::after(arg, "EQ "), &m)
           || m < 0 || m > static_cast<Int32>(DFP_EQ_MAX))
        {
            bibo::serial::printLine(
                "ERR sound EQ wants 0-5: normal pop rock jazz classic bass");
            return;
        }

        bibo::sound::setEq(static_cast<UInt8>(m));
        printSound();
        return;
    }

    /* SOUND LIST - the names, what they mean, and which track each is. */
    if(bibo::text::eq(arg, "LIST"))
    {
        for(auto [name, track, means] : bibo::sfx::CLIPS)
        {
            bibo::serial::printf("INFO sfx %s track=%u - %s\n",
                                 name,
                                 static_cast<UInt32>(track),
                                 means);
        }

        /*
         * A name pointing past the end of the card is the one failure this
         * table can have on its own.
         */
        if(bibo::sound::count() > 0
           && bibo::sfx::highest() > bibo::sound::count())
        {
            bibo::serial::printf(
                "ERR sfx names reach track %u but the card holds %u\n",
                static_cast<UInt32>(bibo::sfx::highest()),
                static_cast<UInt32>(bibo::sound::count()));
        }
        printSound();
        return;
    }

    /*
     * SOUND FILES - the only command here that WAITS FOR AN ANSWER, so it is
     * the liveness test as well as the count: a module that replies is powered,
     * listening, has a card mounted and has its TX wire on the right pad. A
     * silent module is reported as such rather than as zero files - zero is a
     * legitimate answer from an empty card.
     */
    if(bibo::text::eq(arg, "FILES"))
    {
        /*
         * Re-mounts, which is what re-counts: asking for the count alone would
         * leave the two able to disagree.
         */
        if(!bibo::sound::mount() || bibo::sound::count() == 0)
        {
            bibo::serial::printLine(
                "ERR sound the module did not answer - check power, the card, "
                "and that its TX reaches the Pico's RX");
            return;
        }

        bibo::serial::printf("INFO sound %u file(s) on the card\n",
                             static_cast<UInt32>(bibo::sound::count()));
        printSound();
        return;
    }

    /*
     * SOUND RX - whatever the module has said, as raw bytes. THE DIAGNOSTIC FOR
     * THE CASE BUSY CANNOT SETTLE: the pin is pulled up, so "idle" and "that
     * wire is not connected" both read high. The module volunteers a frame when
     * a track ENDS even with ACK off, so bytes arriving prove it is alive and
     * listening; nothing arriving points at the module, the card or the RX
     * wire, and bytes arriving while busy stays high point at the BUSY wire.
     */
    if(bibo::text::eq(arg, "RX"))
    {
        Utf8  line[96];
        Size  at = 0;
        Int32 n  = 0;

        line[0] = '\0';

        while(bibo::uart::readable(uart0) && n < 24)
        {
            const Int32 b = bibo::uart::readByte(uart0, 0);
            if(b < 0)
            {
                break;
            }
            ++n;

            const Int32 w = bibo::text::format(&line[at], sizeof(line) - at,
                                               "%02X ", b);
            if(w <= 0 || static_cast<Size>(w) >= sizeof(line) - at)
            {
                break;
            }
            at += static_cast<Size>(w);
        }

        bibo::serial::printf("OK sound rx %d bytes %s\n",
                             n, (n > 0) ? line : "-");
        return;
    }

    if(bibo::text::eq(arg, "STOP"))
    {
        bibo::sound::stop();
        printSound();
        return;
    }
    if(bibo::text::eq(arg, "PAUSE"))
    {
        bibo::sound::pause();
        printSound();
        return;
    }
    if(bibo::text::eq(arg, "RESUME"))
    {
        bibo::sound::resume();
        printSound();
        return;
    }
    if(bibo::text::eq(arg, "NEXT"))
    {
        bibo::dfplayer::send(&bibo::sound::bus, DFP_CMD_NEXT, 0);
        printSound();
        return;
    }
    if(bibo::text::eq(arg, "PREV"))
    {
        bibo::dfplayer::send(&bibo::sound::bus, DFP_CMD_PREV, 0);
        printSound();
        return;
    }

    bibo::serial::printLine(
        "ERR sound wants RESET|VOL <0-30>|EQ <0-5>|PLAY <name|n>|LIST|FILES|RX|STOP|PAUSE|RESUME|NEXT|PREV");
}

static const Command COMMANDS[] =
{
    { .name = "PING",        .usage = "",                        .what = "answers PONG",                             .run = cmdPing },
    { .name = "ID",          .usage = "",                        .what = "board, sdk, build time, unique id",        .run = printId },
    { .name = "STATUS",      .usage = "",                        .what = "uptime and led state",                     .run = printStatus },
    { .name = "HELP",        .usage = "",                        .what = "this list",                                .run = printHelp },
    { .name = "BOOTSEL",     .usage = "",                        .what = "reboot into the UF2 bootloader",           .run = cmdBootsel },
    { .name = "LED",         .usage = " ON|OFF|BLINK <hz>",      .what = "solid, or blink; 0 stops",                 .run = handleLed },

    { .name = "SENSORS",     .usage = "",                        .what = "what is attached",                         .run = printSensors },
    { .name = "SCAN",        .usage = "",                        .what = "every I2C address that answers",           .run = printScan },
    { .name = "TOF",         .usage = " [MODE SHORT|LONG]",      .what = "range in mm; the mode is 1.3 m or 4 m",    .run = cmdTof },

    { .name = "DRIVE",       .usage = "",                        .what = "servo and esc state",                      .run = cmdDrive },
    { .name = "STOP",        .usage = "",                        .what = "everything off: neutral, disarm, release", .run = cmdStop },
    { .name = "STEER",       .usage = " <-1..1>",                .what = "steer as a fraction of this car's travel", .run = handleSteer },
    { .name = "SLEW",        .usage = " [STEER|THROTTLE] <us>",  .what = "how fast an output may move, per tick",    .run = handleSlew },
    { .name = "SERVO",       .usage = " <us>|ON|OFF|CENTER",     .what = "steering; OFF stops the pulse, servo limp", .run = handleServo },
    { .name = "SERVOTRIM",   .usage = " <us>",                   .what = "move where center is",                     .run = handleTrim },
    { .name = "SERVOLIMITS", .usage = " <min> <max>",            .what = "widen to find the real end stops",         .run = handleLimits },
    { .name = "ESC",         .usage = " ARM|DISARM|NEUTRAL|<us>", .what = "throttle",                                .run = handleEsc },
    { .name = "ESCLIMITS",   .usage = " <min> <max>",            .what = "widen the throttle range",                 .run = handleEscLimits },

    { .name = "WIFI",        .usage = " [JOIN <ssid> <password>]", .what = "the wireless command link",           .run = cmdWifi },
    { .name = "CUE",         .usage = " [LIST|STOP|<name> [OFF]]",     .what = "what the car says, and saying it",      .run = cmdCue },

    { .name = "SOUND",       .usage = " [RESET|VOL|EQ|PLAY <name|n>|LIST|FILES|RX|STOP|...]",
                                                .what = "the speaker",                              .run = cmdSound },

    /* TEMPORARY - goes when GP15 is given back to the wheel encoder. */
    { .name = "LIGHTS",      .usage = " [ON|OFF|AUTO|OFFAT <us>|<lamp>]", .what = "the lamps, and what each is doing", .run = handleLights },
};

static constexpr Size COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

/**
 * @brief Runs HELP (and "?"): lists every command, its usage, and one line
 *        about what it does.
 *
 * Reads COMMANDS so nothing here has to be kept in step with a list written
 * somewhere else.
 *
 * @param arg unused; HELP takes no argument
 */
static Void printHelp(const CharSeq arg)
{
    static_cast<Void>(arg);
    for(const auto& i : COMMANDS)
    {
        bibo::serial::printf("INFO help %s%s - %s\n",
                     i.name, i.usage, i.what);
    }
}

/**
 * @brief Parses one received line and dispatches it to its command handler.
 *
 * @param line the line as received, NUL-terminated; rewritten in place to
 *             uppercase before matching against COMMANDS
 *
 * @note Any line at all counts as liveness for the deadman, including one
 *       that turns out to be a bad command - the question this asks is "is
 *       somebody still there", not "is somebody still there and getting it
 *       right".
 */
static Void handleLine(Utf8* line)
{
    /* A terminal picks its own line ending; without this "PING\r" is not "PING". */
    if(bibo::text::trimEnd(line) == 0)
    {
        return;
    }

    /* BEFORE bibo::text::upper, which rewrites in place. */
    bibo::text::format(rawLine, sizeof(rawLine), "%s", line);

    bibo::text::upper(line);

    lastCmdMs      = bibo::timing::nowMs();
    deadmanTripped = false;

    /* "?" is HELP, not a row of its own - it would print in its own listing. */
    if(bibo::text::eq(line, "?"))
    {
        printHelp(line);
        return;
    }

    for(const auto& i : COMMANDS)
    {
        if(const CharSeq arg = bibo::text::word(line, i.name); arg != nullptr)
        {
            /* Same offset, other buffer - see rawLine above. */
            cmdRawArg = rawLine + (arg - static_cast<CharSeq>(line));
            i.run(arg);
            return;
        }
    }

    bibo::serial::printf("ERR unknown command: %s\n", line);
}

/* ------------------------------------------------------------------ main -- */

/*
 * `int`, not Int32, and this is the one place in the program where that is
 * right. C++ requires main to return literally `int`; Int32 is int32_t, which
 * on arm-none-eabi is `long int` - the same size and representation, a
 * different type as far as the language is concerned, and the compiler refuses
 * it. main's signature is the C runtime's contract, not this project's
 * vocabulary, so it is spelled the runtime's way.
 */
/**
 * @brief Entry point: declares the pin map, opens every subsystem, then
 *        loops forever pumping the drivetrain, the cues, and the wireless
 *        link, and dispatching whatever arrives on the serial line.
 *
 * @return never - the loop runs until the board is reset or rebooted
 */
int main(Void)
{
    bibo::serial::open();

    /*
     * ---- WHAT IS WIRED WHERE, before anything is opened -------------------
     *
     * The first act of every program here. Nothing below pins.hxx holds a GPIO
     * number; every subsystem reads the map installed here when it opens. It
     * must come FIRST - the map starts empty, so a subsystem opened before this
     * binds NOTHING and is visibly dead, which is the right failure. A failure
     * here means two roles claim one pad; pins.hxx static_asserts the car's map
     * at compile time, so this branch is for the day somebody edits car().
     */
    if(!bibo::pins::begin(bibo::pins::car()))
    {
        bibo::serial::printf("ERR %s\n", bibo::pins::conflictText());
    }

    /* At boot, so SENSORS and TOF answer at once. A missing one is the answer. */
    sensorsOpen();

    /* ESC to neutral, steering RELEASED - chassis.h says why those differ. */
    bibo::drive::open();

    /*
     * On the Pico 2 W the LED hangs off the CYW43439 - slow (hundreds of ms)
     * and able to fail, so the result is REPORTED: a board that answers PING
     * but says cyw43=FAILED is a very different problem from a silent one. On
     * the plain Pico 2 it is GP25 and cannot fail. Either way the outcome is
     * remembered and every later call is a no-op rather than a crash.
     */
    bibo::status::open();

    /*
     * TEMPORARY scaffolding on borrowed pins - lib/lights.h says which and why.
     * AFTER bibo::drive::open() so the drivetrain wins any pin disagreement.
     */
    bibo::lights::open();

    /* After the lamps: a cue with nowhere to come out of fails silently. */
    bibo::cue::open();

    /*
     * A tuning that survives a reflash, so it lives in cal.h and is handed over
     * here; cue.h cannot reach for cal.h. The RULE is the same on any car and
     * only the number is this one's.
     */
    bibo::cue::setMotionUs(LIGHT_CAL_OFF_US);

    /*
     * Opening the UART is cheap - a baud rate and two pin functions - so it
     * happens at boot. MOUNTING THE CARD is not, and does not: SOUND RESET pays
     * the two seconds, and only a session that wants sound pays them.
     */
    bibo::sound::open();

    /* Proof of life before any host could be listening, then a slow heartbeat. */
    bibo::status::hello(HELLO_FLASHES, HELLO_FLASH_MS);
    bibo::status::blink(IDLE_BLINK_HZ);

    /*
     * ---- the wireless link ---------------------------------------------
     *
     * Wired up, not switched on: nothing touches the radio until WIFI JOIN.
     * setLineHandler sends a line arriving in a datagram to the SAME handler a
     * line off the cable goes to - one command language, not two. setMirror
     * sends everything printed to the wireless peer as well as the cable, so a
     * host listening only wirelessly sees the replies to its own commands.
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

        /*
         * BEFORE the drivetrain, and before the serial read that skips the rest
         * of the loop on an idle millisecond. In NO_SYS mode nothing in lwIP
         * happens on its own: no packet is received, no join completes, no
         * timer fires except inside this call. A wirelessly-arrived command is
         * dispatched from in here, so it feeds the deadman like a cabled one.
         */
        bibo::net::poll();

        {
            if(const bibo::net::State ns = bibo::net::status(); ns != netReported)
            {
                netReported = ns;
                bibo::serial::printf("INFO wifi state=%s ip=%s port=%d\n",
                             bibo::net::stateWord(ns), bibo::net::address(), static_cast<Int32>(NET_PORT));
            }
        }

        /*
         * Walks the servo and ESC toward their targets a few microseconds at a
         * time, so a slider dragged end to end sweeps rather than steps.
         */
        bibo::drive::pump();

        /*
         * ---- the deadman ------------------------------------------------
         *
         * stop() FIRST, then the report: bibo::serial::printf blocks for up to
         * half a second while the CDC TX buffer is full, and a host that has
         * stopped draining the port is exactly what this exists for.
         */
        {
            const bibo::drive::State dm      = bibo::drive::read();

            if(const Bool driving = dm.escArmed && (dm.escTargetUs > dm.escMinUs); driving && !deadmanTripped && (bibo::timing::nowMs() - lastCmdMs) > DEADMAN_MS)
            {
                bibo::drive::stop();
                bibo::lights::forceLamp(bibo::lights::LAMP_COUNT);

                /*
                 * And it SAYS so, on the car. This fires when the host has
                 * stopped listening, so a line in an unread console must not be
                 * the only place the message goes.
                 */
                bibo::cue::emit(bibo::cue::KIND_ALERT);

                deadmanTripped = true;

                bibo::serial::printf("ERR deadman - no command for %u ms, stopped\n",
                             static_cast<UInt32>(DEADMAN_MS));
            }
        }

        /*
         * AFTER pump, and reading the ACTUAL output rather than the targets:
         * the slew limiter means the two differ for about a second after every
         * command, and targets would light a lamp before the car had acted.
         */
        {
            const bibo::drive::State d = bibo::drive::read();

            bibo::cue::Input ci{};
            ci.steerMilli = d.steerNowMilli;
            ci.throttleUs = d.escUs;
            ci.idleUs     = d.escMinUs;
            ci.neutralUs  = DRIVE_NEUTRAL_US;
            ci.armed      = d.escArmed;
            ci.headOn     = false;   /* nothing the car knows implies darkness */

            bibo::cue::tick(&ci);
        }

        /* Output before the host opens the port is discarded, so this waits. */
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
            /* A line that overran is discarded at its END - see below. */
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
            line[len++] = static_cast<Utf8>(c);
        }
        else
        {
            /*
             * len = 0 with no "swallow" state left the TAIL of an over-long
             * line accumulating as a fresh command: a 200-byte line produced
             * this error AND whatever bytes 129 onward parsed as - a command
             * nobody sent. The overlong flag is what actually drops it.
             */
            len      = 0;
            overlong = true;
            bibo::serial::printf("ERR line too long\n");
        }
    }
}
