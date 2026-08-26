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

#include "shared.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
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

#endif
