/*
 * A thin wrapper over the Pico SDK, spelled the way this project spells things.
 *
 * WHY. The SDK is snake_case C with its own vocabulary (`uint`, `gpio_put`,
 * `absolute_time_t`); the rest of this project is manbox
 * (github.com/exoad/manbox) - PascalCase types, camelCase functions,
 * SCREAMING_SNAKE macros, and the types.h aliases. Mixing the two inside one
 * function is how a style guide quietly dies. This header is the seam: below it
 * the SDK's spelling, above it ours, and exactly one place where they meet.
 *
 * EVERYTHING IS `static inline`. There is no pico2w.c and no library to link:
 * each of these compiles down to the same instructions the raw SDK call would,
 * so the wrapper costs nothing at runtime and any target that includes the
 * header gets it. That also means a sketch can use it without touching
 * CMakeLists.txt, which is the whole point of a scratch program.
 *
 * WHAT IS NOT HERE. I2C and SPI. They are stateful, they have real
 * configuration, and a wrapper that hid that would teach the wrong thing. When
 * the ToF sensors and the SD card go on, they get their own headers with their
 * own state, not one more function in this one.
 *
 * UART WAS IN THAT LIST AND IS NOT ANY MORE, and the reason is worth reading
 * before you assume the rule was just abandoned: on RP2350 the GPIO function
 * that carries UART data is not the same number on every pad, and picking the
 * obvious one on GP14/GP15 transmits onto a flow-control line in perfect
 * silence. That is not configuration a caller should be left to discover - it
 * is exactly the kind of one-place-to-be-right this header exists for. See the
 * uart namespace below.
 *
 * PIN SAFETY. lib/pins.hxx is the car's map and the place to look before
 * borrowing a pad: GP0/GP1 servo and ESC, GP4/GP5 I2C, GP10-GP13 lamps on the
 * ToF XSHUT lines, GP14/GP15 the DFPlayer's UART, GP16-GP19 SPI for the SD
 * card. GP28 is free and is what the starter sketch blinks.
 *
 * Nothing HERE enforces any of that - the board cannot know what you soldered -
 * but pins.hxx does static_assert that no two subsystems claim one pad, which
 * is the half a compiler can check.
 */

#pragma once

/*
 * The vocabulary: Int32, UInt16, Bool, Void, Utf8, CharSeq.
 *
 * A SIBLING of this file rather than a repo-level shared/ directory, and that
 * move is the point. It was in shared/ because the name promised it was shared
 * with the hub, and it never was - shared/shared.hpp is a separate C++ file that
 * mirrors it by hand, and exactly one file in the tree ever included the C one:
 * this header. A directory named for a relationship that does not exist is worse
 * than no directory, because it survives every reorganisation on the strength of
 * its name.
 *
 * Sitting next to hal.h it also resolves for any tool that has not loaded the
 * project - a quoted include is searched relative to THIS file first - which is
 * what keeps a sketch from lighting up red in an editor that is merely not
 * configured yet.
 */
#include "types.hxx"

/* vsnprintf and va_list, named because this file uses them. */
#include <stdarg.h>
#include <stdio.h>

/* pico/stdlib.h FIRST, and on its own line, because it is what drags in the
 * board header - and the board header is what decides, below, whether this
 * build has a wireless chip at all. Everything after it can then ask. */
#include "pico/stdlib.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/unique_id.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"

/* Only on a board that HAS the chip. On a plain Pico 2 this header is still on
 * the include path - the SDK ships it unconditionally - but nothing links
 * pico_cyw43_arch, so including it here would compile and then fail at the
 * link with a wall of undefined references to a peripheral the board does not
 * physically have. Ask the board, not the SDK. */
#if defined(CYW43_WL_GPIO_LED_PIN)
#include "pico/cyw43_arch.h"
#endif

namespace bibo
{

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

namespace gpio
{
static Void open(Pin pin, PinDir dir)
{
    gpio_init((uint) pin);
    gpio_set_dir((uint) pin, dir == PIN_DIR_OUT);
}

static Void write(Pin pin, Bool high)
{
    gpio_put((uint) pin, high);
}

static Bool read(Pin pin)
{
    return gpio_get((uint) pin);
}

static Void toggle(Pin pin)
{
    gpio_xor_mask(1u << (uint) pin);
}

static Void pull(Pin pin, PinPull pull)
{
    gpio_set_pulls((uint) pin, pull == PIN_PULL_UP, pull == PIN_PULL_DOWN);
}

/* ---- time ---------------------------------------------------------------- */


} // namespace gpio

/* ===========================================================================
 * uart - a serial port to another chip.
 *
 * NOT the console. serial:: below is USB CDC and talks to the laptop; this is
 * a wire to a part on the breadboard. The two have nothing to do with each
 * other and CMakeLists keeps UART stdio switched off precisely so this one is
 * free to be used for something.
 *
 * ---------------------------------------------------------------------------
 * THE FUNCSEL TRAP, which cost an evening to find and is the reason this
 * wrapper exists at all rather than every caller writing three SDK lines.
 *
 * gpio_set_function(pin, GPIO_FUNC_UART) is the call every example uses and it
 * is WRONG on some pins. GPIO_FUNC_UART is funcsel 2, and on RP2350 funcsel 2
 * means different things on different pads:
 *
 *     GP0, GP1, GP12, GP13, GP16, GP17  funcsel 2 IS UART data. Fine.
 *     GP14, GP15                        funcsel 2 is UART0 CTS and RTS -
 *                                       FLOW CONTROL, not data.
 *
 * On GP14/GP15 the data lines are funcsel 0x0b, which the SDK calls
 * GPIO_FUNC_UART_AUX. Use the wrong one and the port opens, the baud rate is
 * set, writes return normally and nothing is ever transmitted - the bytes go
 * out on a handshake line the other end is not listening to. Nothing errors.
 *
 * So open() picks the funcsel from the pin rather than trusting the caller to
 * know, and there is exactly one place in this firmware that has to be right
 * about it.
 * ======================================================================== */
namespace uart
{

/* Which funcsel carries UART DATA on this pad.
 *
 * The AUX pads are the ones whose UART function sits at 0x0b instead of 2. On
 * RP2350 that is the GP14/GP15 pair; every other UART-capable pad uses 2. A
 * pad that carries no UART at all returns the normal value and simply will not
 * work, which is what a wiring mistake should look like - the pin table in
 * pins.hxx is where that is prevented, not here. */
static gpio_function_t dataFunc(Pin pin)
{
    return (pin == 14 || pin == 15) ? GPIO_FUNC_UART_AUX : GPIO_FUNC_UART;
}

/* Brings up `port` at `baud` on the given pins.
 *
 * Either pin may be pins::NONE. A TX-only link is a real configuration and not
 * a broken one: the DFPlayer is perfectly usable without reading its replies,
 * and refusing to open would turn a working half-duplex setup into an error.
 *
 * Returns the baud rate the hardware actually achieved, which is not always
 * the one asked for - the divider is integer and the peripheral clock is
 * whatever it is. At 9600 the error is negligible; the value is returned
 * because a caller that cares should be able to see it rather than assume. */
static UInt32 open(uart_inst_t* port, UInt32 baud, Pin tx, Pin rx)
{
    const UInt32 got = uart_init(port, baud);

    if(tx != -1)
    {
        gpio_set_function(static_cast<UInt32>(tx), dataFunc(tx));
    }
    if(rx != -1)
    {
        gpio_set_function(static_cast<UInt32>(rx), dataFunc(rx));
    }

    /* 8N1 and no flow control - what every module in this drawer expects, and
     * what the DFPlayer's datasheet specifies. */
    uart_set_format(port, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(port, false, false);

    /* FIFOs on. Without them a byte that arrives while the main loop is
     * elsewhere is dropped, and this firmware's loop does a lot between
     * polls. */
    uart_set_fifo_enabled(port, true);

    return got;
}

static Void write(uart_inst_t* port, const UInt8* data, Size len)
{
    uart_write_blocking(port, data, len);
}

/* True if at least one byte is waiting. */
static Bool readable(uart_inst_t* port)
{
    return uart_is_readable(port);
}

/* One byte, or -1 if none arrived within the timeout. Int32 rather than UInt8
 * so "nothing" and "the byte 0xFF" are different answers. */
static Int32 readByte(uart_inst_t* port, UInt32 timeoutUs)
{
    if(!uart_is_readable_within_us(port, timeoutUs))
    {
        return -1;
    }
    return static_cast<Int32>(uart_getc(port));
}

/* Drops whatever is sitting in the receive FIFO.
 *
 * Worth having before a command: a module that was mid-reply when the Pico
 * reset has left bytes in there, and reading them as the answer to the NEXT
 * question is the kind of bug that looks like corruption. */
static Void drain(uart_inst_t* port)
{
    while(uart_is_readable(port))
    {
        static_cast<Void>(uart_getc(port));
    }
}

} // namespace uart

namespace timing
{
static Void ms(UInt32 ms)
{
    sleep_ms(ms);
}

static Void us(UInt64 us)
{
    sleep_us(us);
}

/* Milliseconds since boot. Wraps after about 49 days. */
static UInt32 nowMs(Void)
{
    return static_cast<UInt32>(to_ms_since_boot(get_absolute_time()));
}

static UInt64 nowUs(Void)
{
    return static_cast<UInt64>(to_us_since_boot(get_absolute_time()));
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

} // namespace timing

namespace serial
{
static Void open(Void)
{
    stdio_init_all();
}

/* ---- the second listener -------------------------------------------------
 *
 * Output goes to the USB console and, if something has registered here, to a
 * second place as well - which today means the wireless link in net.h.
 *
 * A hook rather than a call to net.h directly, because hal.h is the layer that
 * knows about the BOARD and net.h is a layer above it. hal.h calling upward
 * would invert the dependency the whole library is arranged around, and would
 * drag lwIP into the build of every program that prints anything, including
 * the sketch target that deliberately links none of it.
 *
 * The mirror is handed WHOLE LINES, already formatted and still carrying their
 * newline. That is what makes one datagram equal one command reply.
 */
typedef Void (*Mirror)(CharSeq);

static Mirror mirrorFn = NULL;

static Void setMirror(Mirror fn)
{
    mirrorFn = fn;
}

static Void emit(CharSeq text)
{
    fputs(text, stdout);

    if(mirrorFn != NULL)
    {
        mirrorFn(text);
    }
}

static Void print(CharSeq text)
{
    emit(text);
}

static Void printLine(CharSeq text)
{
    emit(text);
    emit("\n");
}

/*
 * Formatted output to the host.
 *
 * A macro rather than a function because it is variadic and a forwarding
 * wrapper would cost a va_list round trip for nothing. It compiles to exactly
 * the printf it wraps.
 *
 * The point is not overhead - there is none - it is that the SEAM is complete.
 * hal.h is where the SDK's spelling stops and this project's begins, and a
 * console calling printf() directly was the one place that reached past it,
 * sixty-two times. Everything a program needs now has a name from this library,
 * so the day the transport is not stdio - a UDP link, a log to the SD card,
 * both at once - there is one definition to change instead of sixty-two call
 * sites to find.
 */
/*
 * The one line's worth this formats into.
 *
 * Every line this program prints is a short one - the longest is a DRIVE
 * report at about a hundred characters - so this is roughly double the worst
 * real case. It is not a limit anybody should be near.
 */
static const Size LINE_CAP = 256;

/*
 * It formats into a buffer and hands the finished line to emit() rather
 * than going straight to the C library, which is what lets the same line reach
 * the wireless link as well as the cable. That is the whole reason this seam
 * was built; it is now being used.
 *
 * On truncation the tail is replaced with "...\n" instead of being left as it
 * fell. A cut line that keeps its newline is a line the host can still parse
 * and visibly complain about; one that loses it silently glues itself to the
 * next line and produces a command nobody sent. This project has been bitten by
 * exactly that shape of bug twice - a truncated cal header, and an over-long
 * serial line whose tail became a fresh command - and both times the damage
 * came from the truncation being SILENT.
 */
static Void printf(CharSeq fmt, ...)
{
    Utf8 buf[LINE_CAP];

    va_list ap;
    va_start(ap, fmt);
    const Int32 n = vsnprintf(buf, LINE_CAP, fmt, ap);
    va_end(ap);

    if(n >= static_cast<Int32>(LINE_CAP))
    {
        buf[LINE_CAP - 5] = '.';
        buf[LINE_CAP - 4] = '.';
        buf[LINE_CAP - 3] = '.';
        buf[LINE_CAP - 2] = '\n';
        buf[LINE_CAP - 1] = '\0';
    }

    emit(buf);
}

/*
 * Blocks until the host opens the USB serial port, or until `timeoutMs`.
 *
 * This is the fix for the complaint in open() above. USB enumeration
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
static Bool waitForHost(UInt32 timeoutMs)
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

/*
 * Whether a host currently has the port open.
 *
 * Not the same question as waitForHost(): this one does not block, and it
 * is how a program notices a terminal ARRIVING or LEAVING mid-run rather than
 * only at startup. A board that greeted the first listener and then went quiet
 * forever looks dead to the second one.
 */
static Bool hostPresent(Void)
{
    return stdio_usb_connected();
}

/*
 * One character, or NONE if none arrived within `timeoutUs`.
 *
 * A timeout rather than a block, because the caller almost always has something
 * else to do - a blink to keep, an output to slew - and a read that parks the
 * main loop turns every one of those into a stutter.
 */
/*
 * DERIVED from the SDK, never written out as a number.
 *
 * It was -1 here for exactly one build, because -1 is what a sentinel looks
 * like. The SDK says -2. Every timeout therefore failed this comparison and got
 * stored as (Utf8)(-2) = 0xFE, so the command buffer filled with bytes nobody
 * typed and every line came back "unknown command" preceded by a wall of
 * rubbish. A constant that AGREES with another header is a constant that will
 * disagree with it eventually.
 */
static const Int32 NONE = PICO_ERROR_TIMEOUT;

static Int32 readChar(UInt32 timeoutUs)
{
    return getchar_timeout_us(timeoutUs);
}

/* Flushes anything buffered out to the host. Worth doing before a reboot, or
 * the last thing the program said is lost with it. */
static Void flush(Void)
{
    stdio_flush();
}

/* ---- board identity ------------------------------------------------------ */

/*
 * The chip's unique id as hex, into `out`.
 *
 * Every RP2350 has one burned in, which is what makes it useful: it identifies
 * a BOARD across reflashes, so a log line can say which of two identical cars
 * produced it. `out` wants at least 17 bytes.
 */

} // namespace serial

namespace board
{
static Void id(Utf8* out, Size cap)
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    Size at = 0;
    for(Size i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i)
    {
        if(at + 2 >= cap)
        {
            break;
        }
        static const Utf8 HEX[] = "0123456789ABCDEF";
        out[at++] = HEX[(id.id[i] >> 4) & 0x0F];
        out[at++] = HEX[id.id[i] & 0x0F];
    }
    if(cap > 0)
    {
        out[(at < cap) ? at : (cap - 1)] = '\0';
    }
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

} // namespace board

namespace pwm
{
static Void open(Pin pin, UInt32 freqHz)
{
    gpio_set_function((uint) pin, GPIO_FUNC_PWM);

    const Float32 clk = static_cast<Float32>(clock_get_hz(clk_sys));
    Float32 div = clk / (static_cast<Float32>(freqHz) * static_cast<Float32>(PWM_WRAP + 1));
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
static Void write(Pin pin, Float32 duty)
{
    if(duty < 0.0f)
    {
        duty = 0.0f;
    }
    if(duty > 1.0f)
    {
        duty = 1.0f;
    }
    pwm_set_gpio_level((uint) pin, static_cast<UInt16>(duty * static_cast<Float32>(PWM_WRAP)));
}

/* ---- servo and ESC ------------------------------------------------------- */

/*
 * Both the steering servo and the ESC speak the same 50 Hz pulse-width
 * protocol, so servo::open() is what an ESC gets too.
 *
 * PUT THE CAR ON A STAND, WHEELS OFF THE GROUND, for every first run of new
 * code. docs/wiring.md says the same thing and means it.
 */

} // namespace pwm

namespace servo
{
static Void open(Pin pin)
{
    pwm::open(pin, SERVO_HZ);
}

/*
 * Holds `us` microseconds of pulse. Clamped to the 1000-2000 us band: a servo
 * driven past its travel stalls against its own end stop, draws locked-rotor
 * current and cooks itself, and it does it quietly.
 */
static Void writeUs(Pin pin, UInt32 us)
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
        static_cast<UInt16>((static_cast<UInt64>(us) * static_cast<UInt64>(PWM_WRAP + 1)) / SERVO_PERIOD_US));
}

/* Centre for a servo; neutral (no drive) for an ESC. */
static Void center(Pin pin)
{
    writeUs(pin, SERVO_MID_US);
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
static Void release(Pin pin)
{
    pwm_set_gpio_level((uint) pin, 0);
}

/* ---- the onboard LED ------------------------------------------------------
 *
 * The one part of this header where the two boards genuinely differ, so it is
 * the one part written twice.
 *
 *   Pico 2 W - the LED hangs off the CYW43439 wireless chip. It is NOT a GPIO
 *              and cannot be reached with gpio::write() at any pin number: the
 *              chip has to be brought up first and then driven through its own
 *              GPIO space. This catches everybody once, because the classic
 *              `gpio_put(25, 1)` from a Pico 1 example compiles, runs, and
 *              does nothing at all.
 *
 *   Pico 2    - the LED is exactly what you expect: GP25, a plain output.
 *              Bringing it up cannot fail.
 *
 * The API above the split does not change, which is the whole point: status::open
 * and every sketch that blinks are written once and run on either board. Only
 * the four functions below know which board they were compiled for, and they
 * are told by the SDK's own board header rather than by anything this project
 * has to remember to set.
 *
 * led::open() must be called before any other led* function, and on the W it can
 * FAIL - the chip is a real peripheral on a real bus. Check the return: if you
 * ignore it, every later call silently does nothing and you will spend the
 * evening looking at your wiring instead.
 */

/*
 * Whether the lamp came up, remembered here rather than by every caller.
 *
 * led::write() used to drive the CYW43439 whether or not it had initialised, so
 * each program carried its own `cyw43Ok` guard and its own copy of the reason.
 * A guard that every caller must remember is a guard, eventually, that one of
 * them forgets.
 */

} // namespace servo

namespace led
{
static Bool up = false;


} // namespace led
#if defined(CYW43_WL_GPIO_LED_PIN)

/*
 * The CYW43439 is brought up ONCE, here, and everything that needs it asks
 * through this.
 *
 * Two things on this board want that chip: the LED, which hangs off its GPIO,
 * and the wireless link in net.h. They arrived a year apart and each one
 * calling cyw43_arch_init() for itself is a second initialisation of a
 * half-initialised radio - which does not report an error, it just leaves the
 * chip in a state where the next thing to touch it behaves oddly.
 *
 * So: idempotent, and the ANSWER is remembered rather than the attempt being
 * repeated. Calling it twice is free; the second call returns what the first
 * one found.
 */

namespace radio
{
static Bool tried = false;

static Bool open(Void)
{
    if(!tried)
    {
        tried = true;
        led::up      = (cyw43_arch_init() == 0);
    }
    return led::up;
}

static Bool up(Void)
{
    return led::up;
}


} // namespace radio

namespace led
{
static Bool open(Void)
{
    return radio::open();
}

static Void write(Bool on)
{
    if(up)
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    }
}

static Bool read(Void)
{
    return up && cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
}

/* Where the lamp is, for a program that reports its own wiring. */
static CharSeq backend(Void)
{
    return "cyw43";
}


} // namespace led
#elif defined(PICO_DEFAULT_LED_PIN)

/* No wireless chip on this board. Named anyway so callers do not need an
 * #ifdef to ask. */

namespace radio
{
static Bool open(Void)
{
    return false;
}

static Bool up(Void)
{
    return false;
}


} // namespace radio

namespace led
{
static Bool open(Void)
{
    gpio_init((uint) PICO_DEFAULT_LED_PIN);
    gpio_set_dir((uint) PICO_DEFAULT_LED_PIN, GPIO_OUT);
    up = true;
    return up;
}

static Void write(Bool on)
{
    if(up)
    {
        gpio_put((uint) PICO_DEFAULT_LED_PIN, on);
    }
}

static Bool read(Void)
{
    return up && gpio_get((uint) PICO_DEFAULT_LED_PIN);
}

static CharSeq backend(Void)
{
    return "gpio" STRINGIFY(PICO_DEFAULT_LED_PIN);
}


} // namespace led
#else

/*
 * A board with no lamp at all. Not either of ours, but the alternative to
 * handling it is a build that fails with a macro error nobody can read, and
 * this way a program written for the car still compiles and runs on it - it
 * just cannot wave.
 */

namespace radio
{
static Bool open(Void)
{
    return false;
}

static Bool up(Void)
{
    return false;
}


} // namespace radio

namespace led
{
static Bool open(Void)
{
    up = false;
    return false;
}

static Void write(Bool on)
{
    static_cast<Void>(on);
}

static Bool read(Void)
{
    return false;
}

static CharSeq backend(Void)
{
    return "none";
}


} // namespace led
#endif

/* Whether the lamp is up. For a program that wants to REPORT the failure
 * rather than merely survive it. */

namespace led
{
static Bool present(Void)
{
    return up;
}

static Void toggle(Void)
{
    write(!read());
}

/* ---- ADC ----------------------------------------------------------------- */

/*
 * Only GP26-GP29 are ADC-capable, as channels 0-3. Passing anything else is a
 * programming error the hardware cannot report.
 */

} // namespace led

namespace adc
{
static Void open(Pin pin)
{
    adc_init();
    adc_gpio_init((uint) pin);
}

static UInt16 read(Pin pin)
{
    adc_select_input((uint) (pin - 26));
    return adc_read();
}

/* 12-bit reading scaled to volts against the 3.3 V reference. */
static Float32 readVolts(Pin pin)
{
    return static_cast<Float32>(read(pin)) * (3.3f / 4095.0f);
}

/*
 * The die temperature sensor, on ADC channel 4 rather than any pin. Accurate to
 * a couple of degrees at best and it reads the CHIP, not the room - it sits a
 * few degrees above ambient as soon as the core is busy. Useful for "is this
 * thing cooking", not for weather.
 *
 * The conversion is the one from the RP2350 datasheet.
 */
static Float32 tempC(Void)
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    const Float32 volts = static_cast<Float32>(adc_read()) * (3.3f / 4095.0f);
    return 27.0f - (volts - 0.706f) / 0.001721f;
}

/* ---- watchdog -------------------------------------------------------------
 *
 * The board resets if watchdog::feed() is not called within `ms`. On a vehicle
 * this is the difference between "the code hung" and "the code hung and the car
 * kept going at the last commanded throttle", so it is here from the start
 * rather than added after the first runaway.
 *
 * A sketch that enables it and then blocks in a long timing::ms() WILL reset. That
 * is the watchdog working.
 */

} // namespace adc

namespace watchdog
{
static Void start(UInt32 ms)
{
    watchdog_enable(ms, true);
}

static Void feed(Void)
{
    watchdog_update();
}

/* True when THIS boot was caused by the watchdog firing rather than by power or
 * the reset pin. Worth printing at startup: a board that is quietly resetting in
 * a loop looks exactly like a board that is slow to start. */
static Bool causedReboot(Void)
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
 * spi::open() works out which controller the pins belong to, so a wrong pairing
 * fails here with a false rather than silently producing a dead bus.
 *
 * CS is deliberately NOT handled by the hardware. The SDK's hardware CS drops
 * between bytes, which several displays and cards read as the end of a
 * transaction; driving it as a plain GPIO around a whole transfer is both
 * simpler to reason about and what nearly every driver does.
 */

/* Which controller a SCK pin belongs to, or NULL if it is not a SCK pin. */

} // namespace watchdog

namespace spi
{
static spi_inst_t* forSck(Pin sck)
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
static spi_inst_t* forMiso(Pin miso)
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
 * closest it can reach and baud() reports what was actually set.
 */
static Bool open(Pin sck, Pin mosi, Pin csPin, UInt32 hz)
{
    spi_inst_t* const bus = forSck(sck);
    if(bus == NULL)
    {
        return false;
    }

    spi_init(bus, hz);
    gpio_set_function((uint) sck, GPIO_FUNC_SPI);
    gpio_set_function((uint) mosi, GPIO_FUNC_SPI);

    if(csPin >= 0)
    {
        gpio::open(csPin, PIN_DIR_OUT);
        gpio::write(csPin, true);        /* idle HIGH; low means "listen to me" */
    }
    return true;
}

/*
 * The same, with MISO - for a device that ANSWERS. A display never does, which
 * is why open() above leaves it out; an SD card does nothing else.
 *
 * Returns false if the four pins are not all the same controller, which is the
 * check worth having: MISO on a pad that cannot carry it fails silently, the
 * card never appears to respond, and every symptom points at the card.
 */
static Bool openFull(Pin sck, Pin mosi, Pin miso, Pin csPin, UInt32 hz)
{
    spi_inst_t* const bus  = forSck(sck);
    spi_inst_t* const rxBus = forMiso(miso);

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
        gpio::open(csPin, PIN_DIR_OUT);
        gpio::write(csPin, true);
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
static Void mode(Pin sck, Bool cpol, Bool cpha)
{
    spi_inst_t* const bus = forSck(sck);
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
static UInt32 baud(Pin sck, UInt32 hz)
{
    spi_inst_t* const bus = forSck(sck);
    return (bus == NULL) ? 0u : static_cast<UInt32>(spi_set_baudrate(bus, hz));
}

/* Blocking write. Returns the number of bytes sent, or 0 for a bad SCK pin. */
static Size write(Pin sck, const UInt8* data, Size n)
{
    spi_inst_t* const bus = forSck(sck);
    if(bus == NULL || data == NULL || n == 0)
    {
        return 0;
    }
    const Int32 sent = static_cast<Int32>(spi_write_blocking(bus, data, n));
    return (sent < 0) ? 0u : static_cast<Size>(sent);
}

static Size writeByte(Pin sck, UInt8 b)
{
    return write(sck, &b, 1);
}

/* Full duplex: sends `tx` and captures the same number of bytes into `rx`. */
static Size transfer(Pin sck, const UInt8* tx, UInt8* rx, Size n)
{
    spi_inst_t* const bus = forSck(sck);
    if(bus == NULL || tx == NULL || rx == NULL || n == 0)
    {
        return 0;
    }
    const Int32 moved = static_cast<Int32>(spi_write_read_blocking(bus, tx, rx, n));
    return (moved < 0) ? 0u : static_cast<Size>(moved);
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

} // namespace spi
#define I2C_TIMEOUT_US 10000u

/* Which controller an SDA pin belongs to, or NULL if it is not an SDA pin. */

namespace i2c
{
static i2c_inst_t* forSda(Pin sda)
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
static Bool open(Pin sda, Pin scl, UInt32 hz)
{
    i2c_inst_t* const bus = forSda(sda);
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
static Bool present(Pin sda, UInt8 addr)
{
    i2c_inst_t* const bus = forSda(sda);
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
static Size write(Pin sda, UInt8 addr, const UInt8* data, Size n, Bool hold)
{
    i2c_inst_t* const bus = forSda(sda);
    if(bus == NULL || data == NULL || n == 0)
    {
        return 0;
    }
    const Int32 sent =
        static_cast<Int32>(i2c_write_timeout_us(bus, addr, data, n, hold, I2C_TIMEOUT_US));
    return (sent < 0) ? 0u : static_cast<Size>(sent);
}

static Size read(Pin sda, UInt8 addr, UInt8* data, Size n, Bool hold)
{
    i2c_inst_t* const bus = forSda(sda);
    if(bus == NULL || data == NULL || n == 0)
    {
        return 0;
    }
    const Int32 got =
        static_cast<Int32>(i2c_read_timeout_us(bus, addr, data, n, hold, I2C_TIMEOUT_US));
    return (got < 0) ? 0u : static_cast<Size>(got);
}

/*
 * Reads `n` bytes from a 16-bit register - the addressing the VL53L1X and most
 * modern sensors use. Write the register index, then read WITHOUT releasing the
 * bus: letting go between the two is what makes a sensor return the wrong
 * register, or nothing.
 */
static Bool readReg16(Pin sda, UInt8 addr, UInt16 reg, UInt8* data, Size n)
{
    UInt8 r[2];
    r[0] = static_cast<UInt8>(reg >> 8);
    r[1] = static_cast<UInt8>(reg & 0xFF);

    if(write(sda, addr, r, 2, true) != 2)
    {
        return false;
    }
    return read(sda, addr, data, n, false) == n;
}

static Bool writeReg16(Pin sda, UInt8 addr, UInt16 reg, const UInt8* data, Size n)
{
    /* Register index and payload must go out as ONE transaction, so they are
     * assembled into one buffer rather than written twice. */
    UInt8 buf[36];
    if(n + 2 > sizeof(buf))
    {
        return false;
    }
    buf[0] = static_cast<UInt8>(reg >> 8);
    buf[1] = static_cast<UInt8>(reg & 0xFF);
    for(Size i = 0; i < n; ++i)
    {
        buf[i + 2] = data[i];
    }
    return write(sda, addr, buf, n + 2, false) == (n + 2);
}

static Bool writeReg16U8(Pin sda, UInt8 addr, UInt16 reg, UInt8 v)
{
    return writeReg16(sda, addr, reg, &v, 1);
}

static Bool writeReg16U16(Pin sda, UInt8 addr, UInt16 reg, UInt16 v)
{
    UInt8 b[2];
    b[0] = static_cast<UInt8>(v >> 8);
    b[1] = static_cast<UInt8>(v & 0xFF);
    return writeReg16(sda, addr, reg, b, 2);
}

static Bool readReg16U8(Pin sda, UInt8 addr, UInt16 reg, UInt8* out)
{
    return readReg16(sda, addr, reg, out, 1);
}

static Bool readReg16U16(Pin sda, UInt8 addr, UInt16 reg, UInt16* out)
{
    UInt8 b[2];
    if(!readReg16(sda, addr, reg, b, 2))
    {
        return false;
    }
    *out = static_cast<UInt16>((static_cast<UInt16>(b[0]) << 8) | b[1]);
    return true;
}

/* ---- reboot --------------------------------------------------------------- */

/*
 * Drops the board into the UF2 bootloader, so it reappears as a drive and can be
 * flashed without touching the BOOTSEL button. The hub's flash path does this
 * over USB instead (1200 baud touch), but a sketch that has painted itself into
 * a corner can offer its own way out.
 */

} // namespace i2c

namespace board
{
static Void rebootToBootsel(Void)
{
    /*
     * Flush and settle before going, or the last thing the program said dies
     * with it.
     *
     * USB is not a wire: printf() lands in a buffer that a later poll walks out
     * to the host, and reset_usb_boot() does not return, so anything still in
     * that buffer is simply lost. The reboot then looks like a crash - the
     * board vanishes mid-sentence and the one line that would have explained it
     * is the line that went missing.
     *
     * The delay is for the host, not the buffer: it needs a moment to take
     * delivery before the device it is talking to stops existing.
     */
    stdio_flush();
    sleep_ms(50);
    reset_usb_boot(0, 0);
}


} // namespace board

} // namespace bibo
