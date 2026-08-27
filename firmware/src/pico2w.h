/*
 * A thin wrapper over the Pico SDK, spelled the way this project spells things.
 *
 * WHY. The SDK is snake_case C with its own vocabulary (`uint`, `gpio_put`,
 * `absolute_time_t`); the rest of this project is manbox
 * (github.com/exoad/manbox) - PascalCase types, camelCase functions,
 * SCREAMING_SNAKE macros, and the shared.h aliases. Mixing the two inside one
 * function is how a style guide quietly dies. This header is the seam: below it
 * the SDK's spelling, above it ours, and exactly one place where they meet.
 *
 * EVERYTHING IS `static inline`. There is no pico2w.c and no library to link:
 * each of these compiles down to the same instructions the raw SDK call would,
 * so the wrapper costs nothing at runtime and any target that includes the
 * header gets it. That also means a sketch can use it without touching
 * CMakeLists.txt, which is the whole point of a scratch program.
 *
 * WHAT IS NOT HERE. I2C, SPI and UART. They are stateful, they have real
 * configuration, and a wrapper that hid that would teach the wrong thing. When
 * the ToF sensors and the SD card go on, they get their own headers with their
 * own state, not one more function in this one.
 *
 * PIN SAFETY. GP0/GP1 are the servo and ESC signals, GP4/GP5 are I2C, GP10-GP13
 * are the ToF XSHUT lines, GP15 is the wheel encoder and GP16-GP19 are SPI for
 * the SD card - see docs/wiring.md. GP28 is free and is what the starter sketch
 * blinks. Nothing here enforces that; the board cannot know what you soldered.
 */

#ifndef PICO2W_H
#define PICO2W_H

/*
 * Reached by an explicit relative path rather than by "shared.h" plus a
 * -I../shared on the command line.
 *
 * Both work for the COMPILER - the include directory is set for both firmware
 * targets and always has been. The difference is every other tool. A quoted
 * include is resolved relative to THIS file first, so an editor with no project
 * loaded still finds it; a bare "shared.h" needs the include path, and without
 * it the whole vocabulary - Int32, UInt16, Bool, Void, Utf8 - goes unresolved,
 * which paints a red squiggle under essentially every line of every sketch.
 *
 * That is a lot of noise to accept in exchange for a slightly prettier include.
 */
#include "../../shared/shared.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

/* ---- types --------------------------------------------------------------- */

/*
 * A GPIO number, NOT a physical pin number. GP28 is Pin 28 here and pin 34 on
 * the board; the silkscreen and the datasheet disagree and this follows the
 * datasheet, like the SDK does.
 */
typedef Int32 Pin;

typedef enum PinDir
{
    PIN_DIR_IN = 0,
    PIN_DIR_OUT = 1
} PinDir;

typedef enum PinPull
{
    PIN_PULL_NONE = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN
} PinPull;

/* ---- constants ----------------------------------------------------------- */

/*
 * Hobby servo pulse widths. The TT-02's Power HD 1501MG has a deadband of 4 us,
 * so anything finer than that is noise - see docs/wiring.md.
 */
#define SERVO_MIN_US 1000
#define SERVO_MID_US 1500
#define SERVO_MAX_US 2000

/* Both the servo and the ESC expect 50 Hz. */
#define SERVO_HZ 50
#define SERVO_PERIOD_US 20000

/* 16-bit PWM counter, so a duty cycle resolves to about 0.3 us at 50 Hz. */
#define PWM_WRAP 65535

/* ---- GPIO ---------------------------------------------------------------- */

/*
 * Claims `pin` and sets its direction. The SDK wants two calls and forgetting
 * the second is the single most common first-hour mistake: gpio_put on a pin
 * that is still an input does nothing at all, silently.
 */
static inline Void gpioOpen(Pin pin, PinDir dir)
{
    gpio_init((uint) pin);
    gpio_set_dir((uint) pin, dir == PIN_DIR_OUT);
}

static inline Void gpioWrite(Pin pin, Bool high)
{
    gpio_put((uint) pin, high);
}

static inline Bool gpioRead(Pin pin)
{
    return gpio_get((uint) pin);
}

static inline Void gpioToggle(Pin pin)
{
    gpio_xor_mask(1u << (uint) pin);
}

static inline Void gpioPull(Pin pin, PinPull pull)
{
    gpio_set_pulls((uint) pin, pull == PIN_PULL_UP, pull == PIN_PULL_DOWN);
}

/* ---- time ---------------------------------------------------------------- */

static inline Void sleepMs(UInt32 ms)
{
    sleep_ms(ms);
}

static inline Void sleepUs(UInt64 us)
{
    sleep_us(us);
}

/* Milliseconds since boot. Wraps after about 49 days. */
static inline UInt32 nowMs(Void)
{
    return (UInt32) (to_ms_since_boot(get_absolute_time()));
}

static inline UInt64 nowUs(Void)
{
    return (UInt64) (to_us_since_boot(get_absolute_time()));
}

/* ---- serial over USB ----------------------------------------------------- */

/*
 * Brings up stdio. Nothing printed before this call arrives anywhere, and on
 * USB the host also needs a moment to enumerate - if the first few lines of a
 * program never show up, that is why, not a broken printf.
 *
 * ---------------------------------------------------------------------------
 * CALL THIS IN EVERY SKETCH, INCLUDING ONES THAT PRINT NOTHING.
 *
 * USB is not automatic. This is what starts the device stack, and a program
 * that never calls it never enumerates: no COM port, no VID_2E8A, nothing for
 * the flasher to touch at 1200 baud to ask for the bootloader. The board runs
 * perfectly and is INVISIBLE to the host, and the only way back in is holding
 * BOOTSEL while plugging the cable, by hand, every single time.
 *
 * That is not hypothetical. On 2026-08-26 a sketch without it went on the
 * board, and from that moment the hub reported "no Pico found: no RPI-RP2 drive
 * and no VID_2E8A serial port" and kept reporting it until the button was held
 * down manually. It reads exactly like dead hardware.
 *
 * It costs a few KB of flash. It is what keeps the board flashable.
 */
static inline Void serialOpen(Void)
{
    stdio_init_all();
}

static inline Void serialPrint(CharSeq text)
{
    fputs(text, stdout);
}

static inline Void serialPrintLine(CharSeq text)
{
    puts(text);
}

/* Variadic, so a macro rather than a function. */
#define serialPrintf(...) printf(__VA_ARGS__)

/*
 * Blocks until the host opens the USB serial port, or until `timeoutMs`.
 *
 * This is the fix for the complaint in serialOpen() above. USB enumeration
 * takes a moment, and anything printed before the host is listening goes
 * nowhere - so the first few lines of a program appear to vanish and the
 * program looks broken when it is merely early.
 *
 * Pass 0 to wait forever, which is right on the bench and WRONG on the car:
 * a board waiting for a terminal that will never arrive is a board that never
 * starts driving.
 *
 * Returns true if the host connected.
 */
static inline Bool serialWaitForHost(UInt32 timeoutMs)
{
    const UInt32 start = to_ms_since_boot(get_absolute_time());

    while(!stdio_usb_connected())
    {
        if(timeoutMs != 0
           && (to_ms_since_boot(get_absolute_time()) - start) > timeoutMs)
        {
            return false;
        }
        sleep_ms(10);
    }
    return true;
}

/* ---- PWM ----------------------------------------------------------------- */

/*
 * Configures `pin` for PWM at `freqHz` with a 16-bit counter.
 *
 * The divider is derived from the ACTUAL system clock rather than assumed:
 * RP2040 boots at 125 MHz and RP2350 at 150 MHz, so a hard-coded divider gives
 * a servo a 20% wrong pulse width on one of the two chips - which looks like a
 * badly centred servo rather than a bug.
 */
static inline Void pwmOpen(Pin pin, UInt32 freqHz)
{
    gpio_set_function((uint) pin, GPIO_FUNC_PWM);

    const Float32 clk = (Float32) clock_get_hz(clk_sys);
    Float32 div = clk / ((Float32) freqHz * (Float32) (PWM_WRAP + 1));
    if(div < 1.0f)
    {
        div = 1.0f;
    }

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(pwm_gpio_to_slice_num((uint) pin), &cfg, true);
}

/* `duty` is 0.0 to 1.0 and is clamped. */
static inline Void pwmWrite(Pin pin, Float32 duty)
{
    if(duty < 0.0f)
    {
        duty = 0.0f;
    }
    if(duty > 1.0f)
    {
        duty = 1.0f;
    }
    pwm_set_gpio_level((uint) pin, (UInt16) (duty * (Float32) PWM_WRAP));
}

/* ---- servo and ESC ------------------------------------------------------- */

/*
 * Both the steering servo and the ESC speak the same 50 Hz pulse-width
 * protocol, so servoOpen() is what an ESC gets too.
 *
 * PUT THE CAR ON A STAND, WHEELS OFF THE GROUND, for every first run of new
 * code. docs/wiring.md says the same thing and means it.
 */
static inline Void servoOpen(Pin pin)
{
    pwmOpen(pin, SERVO_HZ);
}

/*
 * Holds `us` microseconds of pulse. Clamped to the 1000-2000 us band: a servo
 * driven past its travel stalls against its own end stop, draws locked-rotor
 * current and cooks itself, and it does it quietly.
 */
static inline Void servoWriteUs(Pin pin, UInt32 us)
{
    if(us < SERVO_MIN_US)
    {
        us = SERVO_MIN_US;
    }
    if(us > SERVO_MAX_US)
    {
        us = SERVO_MAX_US;
    }
    pwm_set_gpio_level(
        (uint) pin,
        (UInt16) (((UInt64) us * (UInt64) (PWM_WRAP + 1)) / SERVO_PERIOD_US));
}

/* Centre for a servo; neutral (no drive) for an ESC. */
static inline Void servoCenter(Pin pin)
{
    servoWriteUs(pin, SERVO_MID_US);
}

/*
 * Stops the pulse train. The servo goes limp: no holding torque, no current,
 * nothing to push against.
 *
 * This is the one thing neutral cannot do. If the horn is a tooth off its
 * spline, or the linkage is the wrong length, then 1500 us IS the binding
 * position - commanding centre presses the servo against the frame just as
 * firmly as commanding an end stop, and it will sit there and cook. The only
 * way out is to stop asking for anything.
 *
 * Level 0 leaves the pin driven LOW rather than floating. A floating signal
 * line is worse than no signal at all: noise on it reads as random pulse
 * widths, and the servo chases them into whatever it hits first.
 */
static inline Void servoRelease(Pin pin)
{
    pwm_set_gpio_level((uint) pin, 0);
}

/* ---- the onboard LED ------------------------------------------------------
 *
 * NOT a GPIO. On the Pico 2 W the user LED is wired to the CYW43439 wireless
 * chip, so it cannot be reached with gpioWrite() at any pin number - the chip
 * has to be brought up first and then driven through its own GPIO space. This
 * catches everybody once: the classic `gpio_put(25, 1)` from a Pico 1 example
 * compiles, runs, and does nothing at all here.
 *
 * ledOpen() must be called before any other led* function, and it can FAIL -
 * the chip is a real peripheral on a real bus. Check the return: if you ignore
 * it, every later call silently does nothing and you will spend the evening
 * looking at your wiring instead.
 */

static inline Bool ledOpen(Void)
{
    return cyw43_arch_init() == 0;
}

static inline Void ledWrite(Bool on)
{
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

static inline Bool ledRead(Void)
{
    return cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
}

static inline Void ledToggle(Void)
{
    ledWrite(!ledRead());
}

/* ---- ADC ----------------------------------------------------------------- */

/*
 * Only GP26-GP29 are ADC-capable, as channels 0-3. Passing anything else is a
 * programming error the hardware cannot report.
 */
static inline Void adcOpen(Pin pin)
{
    adc_init();
    adc_gpio_init((uint) pin);
}

static inline UInt16 adcRead(Pin pin)
{
    adc_select_input((uint) (pin - 26));
    return adc_read();
}

/* 12-bit reading scaled to volts against the 3.3 V reference. */
static inline Float32 adcReadVolts(Pin pin)
{
    return (Float32) adcRead(pin) * (3.3f / 4095.0f);
}

/*
 * The die temperature sensor, on ADC channel 4 rather than any pin. Accurate to
 * a couple of degrees at best and it reads the CHIP, not the room - it sits a
 * few degrees above ambient as soon as the core is busy. Useful for "is this
 * thing cooking", not for weather.
 *
 * The conversion is the one from the RP2350 datasheet.
 */
static inline Float32 tempC(Void)
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    const Float32 volts = (Float32) adc_read() * (3.3f / 4095.0f);
    return 27.0f - (volts - 0.706f) / 0.001721f;
}

/* ---- watchdog -------------------------------------------------------------
 *
 * The board resets if watchdogFeed() is not called within `ms`. On a vehicle
 * this is the difference between "the code hung" and "the code hung and the car
 * kept going at the last commanded throttle", so it is here from the start
 * rather than added after the first runaway.
 *
 * A sketch that enables it and then blocks in a long sleepMs() WILL reset. That
 * is the watchdog working.
 */
static inline Void watchdogStart(UInt32 ms)
{
    watchdog_enable(ms, true);
}

static inline Void watchdogFeed(Void)
{
    watchdog_update();
}

/* True when THIS boot was caused by the watchdog firing rather than by power or
 * the reset pin. Worth printing at startup: a board that is quietly resetting in
 * a loop looks exactly like a board that is slow to start. */
static inline Bool watchdogCausedReboot(Void)
{
    return watchdog_caused_reboot();
}

/* ---- SPI ------------------------------------------------------------------
 *
 * A synchronous bus with a clock line, so unlike I2C it has no addresses: every
 * device gets its own CHIP SELECT, and the one whose CS is held low is the one
 * listening. That is why several devices can share SCK and MOSI and still not
 * collide - and why forgetting to raise CS again is the classic way to make the
 * next device on the bus appear broken.
 *
 * The RP2350 has two controllers, and which pins each can use is fixed by the
 * silicon rather than chosen freely:
 *
 *   SPI0   SCK  GP2  GP6  GP18        MOSI GP3  GP7  GP19       MISO GP0 GP4 GP16
 *   SPI1   SCK  GP10 GP14 GP26        MOSI GP11 GP15 GP27       MISO GP8 GP12
 *
 * spiOpen() works out which controller the pins belong to, so a wrong pairing
 * fails here with a false rather than silently producing a dead bus.
 *
 * CS is deliberately NOT handled by the hardware. The SDK's hardware CS drops
 * between bytes, which several displays and cards read as the end of a
 * transaction; driving it as a plain GPIO around a whole transfer is both
 * simpler to reason about and what nearly every driver does.
 */

/* Which controller a SCK pin belongs to, or NULL if it is not a SCK pin. */
static inline spi_inst_t* spiForSck(Pin sck)
{
    switch(sck)
    {
    case 2: case 6: case 18: case 22: return spi0;
    case 10: case 14: case 26: return spi1;
    default: return NULL;
    }
}

/*
 * Which controller a MISO pin belongs to. Worth its own function because MISO
 * is the pin people get wrong: it is the only one of the four that the silicon
 * constrains and that a plain GPIO cannot stand in for.
 *
 * CS can be any pin at all - it is just an output somebody wiggles. SCK, MOSI
 * and MISO cannot: each is a specific peripheral output on a specific pad. So
 * "I moved CS to a free pin" is always fine and "I moved MISO to a free pin" is
 * always broken, and the two mistakes look identical on a wiring diagram.
 *
 *   SPI0 MISO   GP0  GP4  GP16  GP20
 *   SPI1 MISO   GP8  GP12 GP24  GP28
 */
static inline spi_inst_t* spiForMiso(Pin miso)
{
    switch(miso)
    {
    case 0: case 4: case 16: case 20: return spi0;
    case 8: case 12: case 24: case 28: return spi1;
    default: return NULL;
    }
}

/*
 * Brings up an SPI bus. `csPin` may be -1 when the caller drives chip select
 * itself, which is what you want with more than one device on the bus.
 *
 * Returns false if the pins do not belong to one controller, rather than
 * bringing up a bus that cannot work. Baud is a request: the hardware picks the
 * closest it can reach and spiBaud() reports what was actually set.
 */
static inline Bool spiOpen(Pin sck, Pin mosi, Pin csPin, UInt32 hz)
{
    spi_inst_t* const bus = spiForSck(sck);
    if(bus == NULL)
    {
        return false;
    }

    spi_init(bus, hz);
    gpio_set_function((uint) sck, GPIO_FUNC_SPI);
    gpio_set_function((uint) mosi, GPIO_FUNC_SPI);

    if(csPin >= 0)
    {
        gpioOpen(csPin, PIN_DIR_OUT);
        gpioWrite(csPin, true);        /* idle HIGH; low means "listen to me" */
    }
    return true;
}

/*
 * The same, with MISO - for a device that ANSWERS. A display never does, which
 * is why spiOpen() above leaves it out; an SD card does nothing else.
 *
 * Returns false if the four pins are not all the same controller, which is the
 * check worth having: MISO on a pad that cannot carry it fails silently, the
 * card never appears to respond, and every symptom points at the card.
 */
static inline Bool spiOpenFull(Pin sck, Pin mosi, Pin miso, Pin csPin,
                               UInt32 hz)
{
    spi_inst_t* const bus  = spiForSck(sck);
    spi_inst_t* const rxBus = spiForMiso(miso);

    if(bus == NULL || rxBus == NULL || bus != rxBus)
    {
        return false;
    }

    spi_init(bus, hz);
    gpio_set_function((uint) sck, GPIO_FUNC_SPI);
    gpio_set_function((uint) mosi, GPIO_FUNC_SPI);
    gpio_set_function((uint) miso, GPIO_FUNC_SPI);

    /* A pull-up on MISO, because an SD card leaves the line floating until it
     * is selected and a floating input reads as noise rather than as idle. */
    gpio_pull_up((uint) miso);

    if(csPin >= 0)
    {
        gpioOpen(csPin, PIN_DIR_OUT);
        gpioWrite(csPin, true);
    }
    return true;
}

/*
 * Clock polarity and phase - SPI "mode", which spi_init() does NOT set: it
 * leaves mode 0 (clock idles LOW, sample on the leading edge).
 *
 *   mode 0   cpol false, cpha false     most sensors, most SD cards
 *   mode 3   cpol true,  cpha true      ST7789 and friends
 *
 * A device on the wrong mode does not half-work. It reads every byte shifted by
 * a bit and behaves as though nothing was ever sent, which is indistinguishable
 * from a wiring fault and is why this is worth naming rather than leaving to a
 * default nobody remembers.
 */
static inline Void spiMode(Pin sck, Bool cpol, Bool cpha)
{
    spi_inst_t* const bus = spiForSck(sck);
    if(bus == NULL)
    {
        return;
    }
    spi_set_format(bus, 8,
                   cpol ? SPI_CPOL_1 : SPI_CPOL_0,
                   cpha ? SPI_CPHA_1 : SPI_CPHA_0,
                   SPI_MSB_FIRST);
}

/* What the hardware actually settled on, which is rarely exactly what was asked
 * for - the divider is an integer. Worth printing during bring-up. */
static inline UInt32 spiBaud(Pin sck, UInt32 hz)
{
    spi_inst_t* const bus = spiForSck(sck);
    return (bus == NULL) ? 0u : (UInt32) spi_set_baudrate(bus, hz);
}

/* Blocking write. Returns the number of bytes sent, or 0 for a bad SCK pin. */
static inline Size spiWrite(Pin sck, const UInt8* data, Size n)
{
    spi_inst_t* const bus = spiForSck(sck);
    if(bus == NULL || data == NULL || n == 0)
    {
        return 0;
    }
    const Int32 sent = (Int32) spi_write_blocking(bus, data, n);
    return (sent < 0) ? 0u : (Size) sent;
}

static inline Size spiWriteByte(Pin sck, UInt8 b)
{
    return spiWrite(sck, &b, 1);
}

/* Full duplex: sends `tx` and captures the same number of bytes into `rx`. */
static inline Size spiTransfer(Pin sck, const UInt8* tx, UInt8* rx, Size n)
{
    spi_inst_t* const bus = spiForSck(sck);
    if(bus == NULL || tx == NULL || rx == NULL || n == 0)
    {
        return 0;
    }
    const Int32 moved = (Int32) spi_write_read_blocking(bus, tx, rx, n);
    return (moved < 0) ? 0u : (Size) moved;
}

/* ---- I2C -------------------------------------------------------------------
 *
 * Two wires, many devices. Unlike SPI there are no chip selects: every device
 * has an ADDRESS, and the one whose address goes out at the start of a
 * transaction is the one that answers. That is why a display, four range
 * sensors and an IMU can share GP4 and GP5 - and why two devices that ship with
 * the SAME address cannot, which is what the ToF sensors' XSHUT lines are for.
 *
 * Both lines are OPEN DRAIN: a device can pull them low but never drive them
 * high, so something has to pull them up. Most breakout boards have the
 * resistors on them already; if yours does not, the bus needs about 4.7k on
 * each line to 3V3. Without them SDA and SCL float and nothing answers, which
 * looks exactly like a dead sensor.
 *
 * Which pins each controller can use is fixed by the silicon:
 *
 *   I2C0   SDA GP0 GP4 GP8 GP12 GP16 GP20     SCL GP1 GP5 GP9 GP13 GP17 GP21
 *   I2C1   SDA GP2 GP6 GP10 GP14 GP18 GP26    SCL GP3 GP7 GP11 GP15 GP19 GP27
 *
 * GP4/GP5 is I2C0, and is what docs/wiring.md reserves for this bus.
 */

/*
 * How long any single I2C transaction may take before it is abandoned.
 *
 * THIS IS NOT A TUNING PARAMETER, it is a safety net, and it exists because the
 * alternative is a board that stops.
 *
 * The SDK's i2c_read_blocking and i2c_write_blocking block FOREVER. If a device
 * holds SDA low - a sensor that has lost its way, a half-seated jumper, a
 * missing pull-up - the call never returns. The program stops there, and with
 * it the USB stack, so the board enumerates, answers nothing, and looks
 * bricked. Every symptom points at the host.
 *
 * That is not hypothetical. It happened here: the range sensor stopped
 * responding and took the whole debug link with it, and the board had to be
 * recovered through the bootloader.
 *
 * 10 ms is far longer than any transaction this project makes - a 32-byte
 * exchange at 400 kHz is under a millisecond - so a timeout means something is
 * genuinely wrong rather than merely slow.
 */
#define I2C_TIMEOUT_US 10000u

/* Which controller an SDA pin belongs to, or NULL if it is not an SDA pin. */
static inline i2c_inst_t* i2cForSda(Pin sda)
{
    switch(sda)
    {
    case 0: case 4: case 8: case 12: case 16: case 20: return i2c0;
    case 2: case 6: case 10: case 14: case 18: case 26: return i2c1;
    default: return NULL;
    }
}

/*
 * Brings up an I2C bus and enables the RP2350's internal pull-ups.
 *
 * The internal ones are weak - tens of kilohms - and are a safety net rather
 * than the real thing. They are enough for a short jumper to one board and NOT
 * enough for a long bus or several devices, where the module's own 4.7k
 * resistors do the work. Enabling them costs nothing and turns "no pull-ups at
 * all" from a silent failure into a working bus.
 *
 * Returns false if the pins do not belong to one controller, rather than
 * bringing up a bus that cannot work.
 */
static inline Bool i2cOpen(Pin sda, Pin scl, UInt32 hz)
{
    i2c_inst_t* const bus = i2cForSda(sda);
    if(bus == NULL)
    {
        return false;
    }

    i2c_init(bus, hz);
    gpio_set_function((uint) sda, GPIO_FUNC_I2C);
    gpio_set_function((uint) scl, GPIO_FUNC_I2C);
    gpio_pull_up((uint) sda);
    gpio_pull_up((uint) scl);
    return true;
}

/*
 * Is anything at `addr`?
 *
 * A zero-length read: the address goes out and the device either acknowledges
 * or it does not. Nothing is transferred, so this is safe to do to an address
 * you know nothing about - which is what makes scanning the bus possible.
 */
static inline Bool i2cPresent(Pin sda, UInt8 addr)
{
    i2c_inst_t* const bus = i2cForSda(sda);
    if(bus == NULL)
    {
        return false;
    }

    UInt8 dummy = 0;
    return i2c_read_timeout_us(bus, addr, &dummy, 1, false, I2C_TIMEOUT_US) >= 0;
}

/* Writes `n` bytes. `hold` true leaves the bus claimed for a repeated start,
 * which is how a register read is done: write the register, then read without
 * letting go. Returns bytes written, or 0 on failure. */
static inline Size i2cWrite(Pin sda, UInt8 addr, const UInt8* data, Size n,
                            Bool hold)
{
    i2c_inst_t* const bus = i2cForSda(sda);
    if(bus == NULL || data == NULL || n == 0)
    {
        return 0;
    }
    const Int32 sent =
        (Int32) i2c_write_timeout_us(bus, addr, data, n, hold, I2C_TIMEOUT_US);
    return (sent < 0) ? 0u : (Size) sent;
}

static inline Size i2cRead(Pin sda, UInt8 addr, UInt8* data, Size n, Bool hold)
{
    i2c_inst_t* const bus = i2cForSda(sda);
    if(bus == NULL || data == NULL || n == 0)
    {
        return 0;
    }
    const Int32 got =
        (Int32) i2c_read_timeout_us(bus, addr, data, n, hold, I2C_TIMEOUT_US);
    return (got < 0) ? 0u : (Size) got;
}

/*
 * Reads `n` bytes from a 16-bit register - the addressing the VL53L1X and most
 * modern sensors use. Write the register index, then read WITHOUT releasing the
 * bus: letting go between the two is what makes a sensor return the wrong
 * register, or nothing.
 */
static inline Bool i2cReadReg16(Pin sda, UInt8 addr, UInt16 reg, UInt8* data,
                                Size n)
{
    UInt8 r[2];
    r[0] = (UInt8) (reg >> 8);
    r[1] = (UInt8) (reg & 0xFF);

    if(i2cWrite(sda, addr, r, 2, true) != 2)
    {
        return false;
    }
    return i2cRead(sda, addr, data, n, false) == n;
}

static inline Bool i2cWriteReg16(Pin sda, UInt8 addr, UInt16 reg,
                                 const UInt8* data, Size n)
{
    /* Register index and payload must go out as ONE transaction, so they are
     * assembled into one buffer rather than written twice. */
    UInt8 buf[36];
    if(n + 2 > sizeof(buf))
    {
        return false;
    }
    buf[0] = (UInt8) (reg >> 8);
    buf[1] = (UInt8) (reg & 0xFF);
    for(Size i = 0; i < n; ++i)
    {
        buf[i + 2] = data[i];
    }
    return i2cWrite(sda, addr, buf, n + 2, false) == (n + 2);
}

static inline Bool i2cWriteReg16U8(Pin sda, UInt8 addr, UInt16 reg, UInt8 v)
{
    return i2cWriteReg16(sda, addr, reg, &v, 1);
}

static inline Bool i2cWriteReg16U16(Pin sda, UInt8 addr, UInt16 reg, UInt16 v)
{
    UInt8 b[2];
    b[0] = (UInt8) (v >> 8);
    b[1] = (UInt8) (v & 0xFF);
    return i2cWriteReg16(sda, addr, reg, b, 2);
}

static inline Bool i2cReadReg16U8(Pin sda, UInt8 addr, UInt16 reg, UInt8* out)
{
    return i2cReadReg16(sda, addr, reg, out, 1);
}

static inline Bool i2cReadReg16U16(Pin sda, UInt8 addr, UInt16 reg, UInt16* out)
{
    UInt8 b[2];
    if(!i2cReadReg16(sda, addr, reg, b, 2))
    {
        return false;
    }
    *out = (UInt16) (((UInt16) b[0] << 8) | b[1]);
    return true;
}

/* ---- reboot --------------------------------------------------------------- */

/*
 * Drops the board into the UF2 bootloader, so it reappears as a drive and can be
 * flashed without touching the BOOTSEL button. The hub's flash path does this
 * over USB instead (1200 baud touch), but a sketch that has painted itself into
 * a corner can offer its own way out.
 */
static inline Void rebootToBootsel(Void)
{
    reset_usb_boot(0, 0);
}

#endif
