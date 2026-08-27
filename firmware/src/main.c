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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* Explicit relative path, so this resolves without an include path being set.
 * See the note in pico2w.h. */
#include "../../shared/shared.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"

/* The sensor drivers. pico_debug is the image the hub talks to, so it is the
 * one that has to be able to answer "what is attached and what does it say" -
 * a sketch cannot, because the hub does not know what sketch is on the board. */
#include "pico2w.h"
#include "vl53l1x.h"

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

#define BOOTSEL_FLUSH_MS 50

static Bool cyw43Ok = false;   /* did the wireless chip come up?              */
static Bool ledOn = false;
static Float32 blinkHz = 0.0f; /* 0 = not blinking                            */
static absolute_time_t nextToggle;

/* ------------------------------------------------------------------ led --- */

/* Half a period per toggle, so `hz` counts full on-off cycles per second. */
static Int64 halfPeriodUs(Float32 hz)
{
    return (Int64) (500000.0f / hz);
}

/* The debug image's own, which TRACKS the state and tolerates a wireless chip
 * that failed to start. pico2w.h's dbgLedWrite() is a bare pass-through; this one
 * is what STATUS reports and what the blink timer drives, so it keeps its
 * behaviour under a name of its own rather than shadowing the wrapper. */
static Void dbgLedWrite(Bool on)
{
    ledOn = on;
    if(cyw43Ok)
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    }
}

static Void ledSetBlink(Float32 hz)
{
    blinkHz = hz;
    if(hz > 0.0f)
    {
        nextToggle = make_timeout_time_us(halfPeriodUs(hz));
    }
}

static Void ledTick(Void)
{
    if(blinkHz <= 0.0f)
    {
        return;
    }
    if(!time_reached(nextToggle))
    {
        return;
    }

    dbgLedWrite(!ledOn);
    nextToggle = make_timeout_time_us(halfPeriodUs(blinkHz));
}

/* -------------------------------------------------------------- commands -- */

static CharSeq cyw43Word(Void)
{
    return cyw43Ok ? "up" : "FAILED";
}

static Void printId(Void)
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    Utf8 hex[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    for(Int32 i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i)
    {
        snprintf(hex + i * 2, 3, "%02X", id.id[i]);
    }

    printf("INFO id board=%s sdk=%s built=%s %s uid=%s cyw43=%s\n",
           PICO_BOARD, PICO_SDK_VERSION_STRING, __DATE__, __TIME__,
           hex, cyw43Word());
}

static Void printStatus(Void)
{
    printf("INFO status up_ms=%u led=%s blink_hz=%.2f cyw43=%s\n",
           (UInt32) to_ms_since_boot(get_absolute_time()),
           ledOn ? "on" : "off",
           (Float64) blinkHz,
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
    printf("INFO help SERVOLIMITS <min> <max> - widen to find end stops\n");
    printf("INFO help ESCLIMITS <min> <max> - widen the throttle range\n");
}

/* Uppercase in place, so commands are accepted in any case. */
static Void upperInPlace(Utf8* s)
{
    for(; *s != '\0'; ++s)
    {
        *s = (Utf8) toupper((UInt8) *s);
    }
}

/* The " (cyw43 down, no effect)" tail on an LED reply. An OK that did nothing
 * has to say so, or a dead wireless chip looks like a dead command parser. */
static CharSeq ledCaveat(Void)
{
    return cyw43Ok ? "" : " (cyw43 down, no effect)";
}

static Void handleLed(Utf8* arg)
{
    if(strcmp(arg, "ON") == 0)
    {
        ledSetBlink(0.0f);
        dbgLedWrite(true);
        printf("OK led on%s\n", ledCaveat());
        return;
    }

    if(strcmp(arg, "OFF") == 0)
    {
        ledSetBlink(0.0f);
        dbgLedWrite(false);
        printf("OK led off%s\n", ledCaveat());
        return;
    }

    if(strncmp(arg, "BLINK", 5) == 0)
    {
        const Float32 hz = (Float32) atof(arg + 5);
        if(hz < 0.0f || hz > BLINK_MAX_HZ)
        {
            printf("ERR blink rate out of range (0-%.0f hz)\n",
                   (Float64) BLINK_MAX_HZ);
            return;
        }

        ledSetBlink(hz);
        if(hz == 0.0f)
        {
            dbgLedWrite(false);
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
    if(strcmp(arg, "SHORT") == 0)
    {
        vl53StopRanging(&tofFront);
        vl53SetMode(&tofFront, VL53_MODE_SHORT);
        vl53ClearInterruptAndStart(&tofFront);
        printf("OK tof mode short\n");
        return;
    }
    if(strcmp(arg, "LONG") == 0)
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
 * The steering servo on GP0 and the ESC on GP1.
 *
 * These are the two outputs on this car that can break something. A servo
 * driven past where its linkage allows stalls and cooks its own motor; an ESC
 * handed a throttle figure spins a wheel that may be on the ground. So the
 * limits here are deliberately TIGHTER than the hardware's, and widening them
 * is a decision somebody makes on purpose rather than a default they inherit.
 *
 * ---------------------------------------------------------------------------
 * THE THREE RULES
 *
 * 1. NEUTRAL AT BOOT, ALWAYS. Both channels hold 1500 us from the moment the
 *    program starts. An ESC will not arm until it has seen neutral, and a
 *    servo that wakes at an extreme is a servo pushing against a stop.
 *
 * 2. THE ESC IS DISARMED UNTIL ASKED. Every throttle command is refused until
 *    `ESC ARM` is sent, and disarming snaps back to neutral. That is one
 *    deliberate act between a slider and a moving car.
 *
 * 3. NOTHING JUMPS. Commands set a TARGET; a timer walks the output toward it
 *    at a bounded rate. A slider dragged from one end to the other produces a
 *    sweep rather than a step, which is the difference between a servo turning
 *    and a servo being hit.
 *
 * ---------------------------------------------------------------------------
 * BEFORE THE ESC IS EVER ARMED, from docs/wiring.md:
 *
 *   - Common ground between the Pico and the ESC is REQUIRED. Signal and ground
 *     cross between two power domains; without the shared return the ESC sees
 *     noise, and that presents as erratic behaviour rather than as no behaviour.
 *   - NEVER connect the BEC 5 V to the Pico while USB is attached. Back-feeding
 *     the 5 V rail from two sources risks both.
 *   - Put the car on a stand. A wheel on the ground turns a test into a
 *     departure.
 */

#define PIN_SERVO 0
#define PIN_ESC   1

/*
 * Tighter than SERVO_MIN_US/MAX_US on purpose.
 *
 * A TT-02's steering has perhaps 30 degrees of useful travel and the linkage
 * binds before the servo's own limits. +/-200 us either side of neutral is a
 * visible sweep that reaches nothing hard. Widen it once the real end stops are
 * known - with the servo horn OFF, so a mistake costs nothing.
 */
#define SERVO_DEFAULT_MIN 1300
#define SERVO_DEFAULT_MAX 1700

/*
 * The HARD bound. Nothing widens past this, whatever is asked - it is the
 * servo's own specification, and beyond it the pulse means nothing at all.
 */
#define SERVO_HARD_MIN 1000
#define SERVO_HARD_MAX 2000

/*
 * Forward only, and barely. 1500 is neutral; 1600 is a crawl on a bench. The
 * reverse half is not offered at all - a Hobbywing QuicRun needs a
 * brake-then-reverse sequence and getting that wrong on a stand is how a
 * gearbox meets a workbench.
 */
#define ESC_DEFAULT_MIN 1500
#define ESC_DEFAULT_MAX 1600

/* Reverse stays unreachable even by widening. Finding a steering end stop is
 * careful work; discovering reverse by accident is not the same kind of
 * experiment. */
#define ESC_HARD_MIN 1500
#define ESC_HARD_MAX 1700

#define SERVO_NEUTRAL_US 1500

/* Microseconds of pulse per tick of the slew timer. At 50 Hz a full 400 us
 * sweep then takes about half a second, which looks deliberate. */
#define SLEW_STEP_US 8
#define SLEW_TICK_MS 20

static Bool  driveUp      = false;
static Bool  escArmed     = false;

/*
 * The working limits, widened only on purpose.
 *
 * They start narrow and are raised a little at a time while watching the
 * linkage, which is how an end stop is FOUND. Guessing them from a datasheet
 * gets you a number the linkage has never heard of.
 */
static Int32 servoMin = SERVO_DEFAULT_MIN;
static Int32 servoMax = SERVO_DEFAULT_MAX;
static Int32 escMin   = ESC_DEFAULT_MIN;
static Int32 escMax   = ESC_DEFAULT_MAX;

static Int32 servoTarget  = SERVO_NEUTRAL_US;
static Int32 servoNow     = SERVO_NEUTRAL_US;
static Int32 escTarget    = SERVO_NEUTRAL_US;
static Int32 escNow       = SERVO_NEUTRAL_US;

static absolute_time_t nextSlew;

static Int32 clampInt(Int32 v, Int32 lo, Int32 hi)
{
    if(v < lo)
    {
        return lo;
    }
    if(v > hi)
    {
        return hi;
    }
    return v;
}

static Void driveOpen(Void)
{
    servoOpen(PIN_SERVO);
    servoOpen(PIN_ESC);

    /* Neutral before anything else can ask for something different. */
    servoWriteUs(PIN_SERVO, SERVO_NEUTRAL_US);
    servoWriteUs(PIN_ESC, SERVO_NEUTRAL_US);

    nextSlew = make_timeout_time_ms(SLEW_TICK_MS);
    driveUp  = true;
}

/* Walks each output toward its target. Called from the main loop. */
static Void drivePump(Void)
{
    if(!driveUp || !time_reached(nextSlew))
    {
        return;
    }
    nextSlew = make_timeout_time_ms(SLEW_TICK_MS);

    if(servoNow != servoTarget)
    {
        const Int32 d = servoTarget - servoNow;
        const Int32 step = (d > SLEW_STEP_US) ? SLEW_STEP_US
                         : ((d < -SLEW_STEP_US) ? -SLEW_STEP_US : d);
        servoNow += step;
        servoWriteUs(PIN_SERVO, (UInt32) servoNow);
    }

    /* A disarmed ESC is walked back to neutral rather than snapped there: a
     * step to neutral from a moving throttle is itself a jolt. */
    const Int32 want = escArmed ? escTarget : SERVO_NEUTRAL_US;
    if(escNow != want)
    {
        const Int32 d = want - escNow;
        const Int32 step = (d > SLEW_STEP_US) ? SLEW_STEP_US
                         : ((d < -SLEW_STEP_US) ? -SLEW_STEP_US : d);
        escNow += step;
        servoWriteUs(PIN_ESC, (UInt32) escNow);
    }
}

static Void printDrive(Void)
{
    printf("OK drive servo=%d servo_t=%d esc=%d esc_t=%d armed=%d "
           "servo_min=%d servo_max=%d esc_min=%d esc_max=%d\n",
           servoNow, servoTarget, escNow, escTarget, escArmed ? 1 : 0,
           servoMin, servoMax, escMin, escMax);
}

/* Everything to neutral, and the ESC disarmed. The one command worth being able
 * to send without thinking. */
static Void driveStop(Void)
{
    escArmed    = false;
    escTarget   = SERVO_NEUTRAL_US;
    servoTarget = SERVO_NEUTRAL_US;

    /* Immediate, not slewed. A stop that eases in is not a stop. */
    escNow   = SERVO_NEUTRAL_US;
    servoNow = SERVO_NEUTRAL_US;
    if(driveUp)
    {
        servoWriteUs(PIN_ESC, SERVO_NEUTRAL_US);
        servoWriteUs(PIN_SERVO, SERVO_NEUTRAL_US);
    }
    printf("OK stop\n");
}

/*
 * Widens or narrows the working range.
 *
 * Clamped to the HARD bound, and the target is pulled back inside the new range
 * so narrowing can never leave an output sitting outside its own limits.
 *
 * This is the calibration path: with the servo horn OFF, step the limit outward
 * until the servo reaches the angle the steering actually needs, then put the
 * horn back on. Doing it with the linkage attached is how a servo discovers a
 * stop by pushing against it.
 */
static Void handleLimits(Utf8* arg)
{
    Int32 lo = 0;
    Int32 hi = 0;
    if(sscanf(arg, "%d %d", &lo, &hi) != 2)
    {
        printf("ERR limits wants <min> <max>\n");
        return;
    }

    if(lo >= hi)
    {
        printf("ERR limits min must be below max\n");
        return;
    }

    servoMin = clampInt(lo, SERVO_HARD_MIN, SERVO_HARD_MAX);
    servoMax = clampInt(hi, SERVO_HARD_MIN, SERVO_HARD_MAX);

    servoTarget = clampInt(servoTarget, servoMin, servoMax);
    printDrive();
}

static Void handleEscLimits(Utf8* arg)
{
    Int32 lo = 0;
    Int32 hi = 0;
    if(sscanf(arg, "%d %d", &lo, &hi) != 2)
    {
        printf("ERR esclimits wants <min> <max>\n");
        return;
    }
    if(lo >= hi)
    {
        printf("ERR esclimits min must be below max\n");
        return;
    }

    escMin = clampInt(lo, ESC_HARD_MIN, ESC_HARD_MAX);
    escMax = clampInt(hi, ESC_HARD_MIN, ESC_HARD_MAX);

    escTarget = clampInt(escTarget, escMin, escMax);
    printDrive();
}

static Void handleServo(Utf8* arg)
{
    if(strcmp(arg, "CENTER") == 0 || strcmp(arg, "CENTRE") == 0)
    {
        servoTarget = SERVO_NEUTRAL_US;
        printDrive();
        return;
    }

    const Int32 us = atoi(arg);
    if(us == 0)
    {
        printf("ERR servo wants microseconds, %d-%d\n",
               servoMin, servoMax);
        return;
    }

    /* Clamped rather than rejected: a slider that stops moving at the limit is
     * clearer than one that silently does nothing past it. The reply reports
     * what was actually taken. */
    servoTarget = clampInt(us, servoMin, servoMax);
    printDrive();
}

static Void handleEsc(Utf8* arg)
{
    if(strcmp(arg, "ARM") == 0)
    {
        escArmed  = true;
        escTarget = SERVO_NEUTRAL_US;
        printf("INFO esc armed - neutral held\n");
        printDrive();
        return;
    }
    if(strcmp(arg, "DISARM") == 0)
    {
        escArmed  = false;
        escTarget = SERVO_NEUTRAL_US;
        printf("INFO esc disarmed\n");
        printDrive();
        return;
    }
    if(strcmp(arg, "NEUTRAL") == 0)
    {
        escTarget = SERVO_NEUTRAL_US;
        printDrive();
        return;
    }

    if(!escArmed)
    {
        printf("ERR esc not armed - send ESC ARM first\n");
        return;
    }

    const Int32 us = atoi(arg);
    if(us == 0)
    {
        printf("ERR esc wants microseconds, %d-%d\n",
               escMin, escMax);
        return;
    }

    escTarget = clampInt(us, escMin, escMax);
    printDrive();
}

static Void handleLine(Utf8* line)
{
    /* Trim trailing CR/space that a terminal may append. */
    Size n = strlen(line);
    while(n > 0
          && (line[n - 1] == '\r' || line[n - 1] == ' ' || line[n - 1] == '\t'))
    {
        line[--n] = '\0';
    }
    if(n == 0)
    {
        return;
    }

    upperInPlace(line);

    if(strcmp(line, "PING") == 0)
    {
        printf("PONG\n");
        return;
    }

    if(strcmp(line, "ID") == 0)
    {
        printId();
        return;
    }

    if(strcmp(line, "STATUS") == 0)
    {
        printStatus();
        return;
    }

    if(strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0)
    {
        printHelp();
        return;
    }

    if(strcmp(line, "BOOTSEL") == 0)
    {
        printf("INFO rebooting into bootloader\n");
        stdio_flush();
        sleep_ms(BOOTSEL_FLUSH_MS);
        reset_usb_boot(0, 0);       /* does not return */
        return;
    }

    if(strncmp(line, "LED ", 4) == 0)
    {
        handleLed(line + 4);
        return;
    }

    if(strcmp(line, "STOP") == 0)
    {
        driveStop();
        return;
    }

    if(strcmp(line, "DRIVE") == 0)
    {
        printDrive();
        return;
    }

    if(strncmp(line, "SERVOLIMITS ", 12) == 0)
    {
        handleLimits(line + 12);
        return;
    }

    if(strncmp(line, "ESCLIMITS ", 10) == 0)
    {
        handleEscLimits(line + 10);
        return;
    }

    if(strncmp(line, "SERVO ", 6) == 0)
    {
        handleServo(line + 6);
        return;
    }

    if(strncmp(line, "ESC ", 4) == 0)
    {
        handleEsc(line + 4);
        return;
    }

    if(strcmp(line, "SENSORS") == 0)
    {
        printSensors();
        return;
    }

    if(strcmp(line, "SCAN") == 0)
    {
        printScan();
        return;
    }

    if(strcmp(line, "TOF") == 0)
    {
        printTof();
        return;
    }

    if(strncmp(line, "TOF MODE ", 9) == 0)
    {
        handleTofMode(line + 9);
        return;
    }

    printf("ERR unknown command: %s\n", line);
}

/* ------------------------------------------------------------------ main -- */

Int32 main(Void)
{
    stdio_init_all();

    /* Sensors come up at boot so SENSORS and TOF can answer immediately. A
     * missing sensor is not a failure here - it is the answer. */
    sensorsOpen();

    /* Both channels to neutral immediately. An ESC will not arm until it has
     * seen neutral, and a servo that wakes at an extreme is already pushing. */
    driveOpen();

    /* Brings up the CYW43439. Without this the LED cannot be driven at all on
     * a Pico 2 W. It is slow (hundreds of ms) and can fail, so its result is
     * reported rather than assumed - a board that answers PING but reports
     * cyw43=FAILED is a very different problem from a board that is silent. */
    cyw43Ok = (cyw43_arch_init() == 0);

    /* Visible proof of life the moment power is applied, before any host has
     * opened the port: three quick flashes, then a slow idle heartbeat. */
    if(cyw43Ok)
    {
        for(Int32 i = 0; i < HELLO_FLASHES; ++i)
        {
            dbgLedWrite(true);
            sleep_ms(HELLO_FLASH_MS);
            dbgLedWrite(false);
            sleep_ms(HELLO_FLASH_MS);
        }
    }
    ledSetBlink(IDLE_BLINK_HZ);

    Utf8 line[LINE_CAP];
    Size len = 0;
    Bool announced = false;

    for(;;)
    {
        ledTick();

        /* Walks the servo and ESC toward their targets, a few microseconds at a
         * time. Nothing jumps: a slider dragged end to end produces a sweep
         * rather than a step. */
        drivePump();

        /* Anything written before the host opens the port is discarded, so the
         * banner waits for a connection rather than being lost. */
        if(!announced && stdio_usb_connected())
        {
            printf("INFO ready %s sdk=%s - type HELP\n",
                   PICO_BOARD, PICO_SDK_VERSION_STRING);
            announced = true;
        }
        if(announced && !stdio_usb_connected())
        {
            announced = false;      /* re-announce on the next connection */
        }

        const Int32 c = getchar_timeout_us(POLL_TIMEOUT_US);
        if(c == PICO_ERROR_TIMEOUT)
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
