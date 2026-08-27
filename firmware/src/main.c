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
        vl53Clear(&tofFront);
        printf("OK tof %u %u\n", mm, st);
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

    if(strcmp(arg, "SHORT") == 0)
    {
        vl53SetMode(&tofFront, VL53_MODE_SHORT);
        printf("OK tof mode short\n");
        return;
    }
    if(strcmp(arg, "LONG") == 0)
    {
        vl53SetMode(&tofFront, VL53_MODE_LONG);
        printf("OK tof mode long\n");
        return;
    }
    printf("ERR bad mode: %s\n", arg);
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
