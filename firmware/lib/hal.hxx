/*
 * A thin wrapper over the Pico SDK, spelled the way this project spells things.
 * The SDK is snake_case C; the rest of this project is manbox
 * (github.com/exoad/manbox). This header is the seam: below it the SDK's
 * spelling, above it ours, and exactly one place where they meet.
 *
 * EVERYTHING IS `static inline`. There is no library to link, so a sketch can
 * use it without touching CMakeLists.txt and the wrapper costs nothing.
 *
 * PIN SAFETY. lib/pins.hxx is the car's map and the place to look before
 * borrowing a pad: GP0/GP1 servo and ESC, GP4/GP5 I2C, GP10-GP13 lamps on the
 * ToF XSHUT lines, GP14/GP15 the DFPlayer's UART, GP16-GP19 SPI for the SD
 * card. GP28 is free and is what the starter sketch blinks. Nothing HERE
 * enforces that, but pins.hxx static_asserts that no two subsystems claim one
 * pad, which is the half a compiler can check.
 */

#pragma once

/*
 * The vocabulary: Int32, UInt16, Bool, Void, Utf8, CharSeq. A SIBLING of this
 * file, so a quoted include resolves for a tool that has not loaded the project.
 */
#include "types.hxx"

/* vsnprintf and va_list, named because this file uses them. */
#include <stdarg.h>
#include <stdio.h>

/*
 * BIBO_FAKE_HAL - the host-test seam. Defined ONLY by firmware/tests: none of
 * the SDK headers below compile on a laptop, and chassis.hxx holds the safety
 * property that the ESC is disarmed until asked. The substitution lives HERE
 * rather than in the build script, because `#include "../hal.hxx"` from
 * lib/chassis/ resolves next to chassis.hxx whatever the include path says.
 */
#ifdef BIBO_FAKE_HAL

#include "../tests/fakes/hal.hxx"

#else

/* FIRST: it drags in the board header, which decides if there is a radio. */
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

/*
 * Only on a board that HAS the chip: the SDK ships this header unconditionally,
 * but a plain Pico 2 does not link pico_cyw43_arch, so including it there
 * compiles and then fails at the link.
 */
#if defined(CYW43_WL_GPIO_LED_PIN)
#include "pico/cyw43_arch.h"
#endif

namespace bibo
{

  /* ---- types --------------------------------------------------------------- */

  /* A GPIO number, NOT a pin number: GP28 is Pin 28 here, pin 34 on the board. */
  typedef Int32 Pin;

  enum PinDir
  {
      PIN_DIR_IN = 0,
      PIN_DIR_OUT = 1
  };

  enum PinPull
  {
      PIN_PULL_NONE = 0,
      PIN_PULL_UP,
      PIN_PULL_DOWN
  };

  /* ---- constants ----------------------------------------------------------- */

  /* Hobby servo pulses. The TT-02's Power HD 1501MG deadbands at 4 us. */
#define SERVO_MIN_US 1000
#define SERVO_MID_US 1500
#define SERVO_MAX_US 2000

  /* Both the servo and the ESC expect 50 Hz. */
#define SERVO_HZ 50
#define SERVO_PERIOD_US 20000

  /* 16-bit PWM counter, so a duty cycle resolves to about 0.3 us at 50 Hz. */
#define PWM_WRAP 65535

  /* ---- GPIO ---------------------------------------------------------------- */

  namespace gpio
  {
    /**
     * @brief Claims a pin and sets its direction.
     *
     * The SDK wants two calls and forgetting the second is the single most
     * common first-hour mistake: gpio_put on a pin that is still an input
     * does nothing at all, silently.
     *
     * @param pin the GPIO number to claim
     * @param dir PIN_DIR_IN or PIN_DIR_OUT
     */
    inline Void open(const Pin pin, const PinDir dir)
    {
        gpio_init(static_cast<UInt32>(pin));
        gpio_set_dir(static_cast<UInt32>(pin), dir == PIN_DIR_OUT);
    }

    /**
     * @brief Drives a pin high or low.
     *
     * @param pin the GPIO number to drive
     * @param high true to drive it high, false to drive it low
     */
    inline Void write(const Pin pin, const Bool high)
    {
        gpio_put(static_cast<UInt32>(pin), high);
    }

    /**
     * @brief Reads the current level of a pin.
     *
     * @param pin the GPIO number to read
     * @return true when the pin reads high
     */
    inline Bool read(const Pin pin)
    {
        return gpio_get(static_cast<UInt32>(pin));
    }

    /**
     * @brief Flips a pin's output level.
     *
     * @param pin the GPIO number to flip
     */
    inline Void toggle(const Pin pin)
    {
        gpio_xor_mask(1u << static_cast<UInt32>(pin));
    }

    /**
     * @brief Sets or clears an internal pull resistor on a pin.
     *
     * @param pin the GPIO number to configure
     * @param pull PIN_PULL_NONE, PIN_PULL_UP, or PIN_PULL_DOWN
     */
    inline Void pull(const Pin pin, const PinPull pull)
    {
        gpio_set_pulls(static_cast<UInt32>(pin), pull == PIN_PULL_UP, pull == PIN_PULL_DOWN);
    }

  }

  /*
   * ===========================================================================
   * uart - a serial port to another chip. NOT the console: serial:: below is USB
   * CDC to the laptop, and CMakeLists keeps UART stdio off so this one is free.
   *
   * THE FUNCSEL TRAP, and the reason this wrapper exists at all.
   * gpio_set_function(pin, GPIO_FUNC_UART) is what every example uses and is
   * WRONG on some pins: it is funcsel 2, which on RP2350 means different things
   * on different pads.
   *
   *     GP0, GP1, GP12, GP13, GP16, GP17  funcsel 2 IS UART data. Fine.
   *     GP14, GP15                        funcsel 2 is UART0 CTS and RTS -
   *                                       FLOW CONTROL, not data.
   *
   * On GP14/GP15 the data lines are funcsel 0x0b, GPIO_FUNC_UART_AUX. Use the
   * wrong one and the port opens, the baud rate is set, writes return normally
   * and nothing is ever transmitted. Nothing errors. So open() picks the funcsel
   * from the pin rather than trusting the caller.
   * ========================================================================
   */
  namespace uart
  {

    /**
     * @brief Which funcsel carries UART DATA on this pad.
     *
     * The AUX pads are the ones whose UART function sits at 0x0b instead of
     * 2. On RP2350 that is the GP14/GP15 pair; every other UART-capable pad
     * uses 2. A pad that carries no UART at all returns the normal value and
     * simply will not work, which is what a wiring mistake should look
     * like - the pin table in pins.hxx is where that is prevented, not here.
     *
     * @param pin the GPIO number to check
     * @return the funcsel that selects the UART data function on that pin
     */
    inline gpio_function_t dataFunc(const Pin pin)
    {
        return pin == 14 || pin == 15 ? GPIO_FUNC_UART_AUX : GPIO_FUNC_UART;
    }

    /**
     * @brief Brings up a UART port at a given baud rate on the given pins.
     *
     * Either pin may be pins::NONE. A TX-only link is a real configuration
     * and not a broken one: the DFPlayer is perfectly usable without reading
     * its replies, and refusing to open would turn a working half-duplex
     * setup into an error.
     *
     * @param port the UART peripheral to bring up
     * @param baud the requested baud rate
     * @param tx the transmit pin, or pins::NONE to leave TX unconfigured
     * @param rx the receive pin, or pins::NONE to leave RX unconfigured
     * @return the baud rate the hardware actually achieved, which is not
     *         always the one asked for - the divider is integer and the
     *         peripheral clock is whatever it is. At 9600 the error is
     *         negligible; the value is returned because a caller that cares
     *         should be able to see it rather than assume.
     */
    inline UInt32 open(uart_inst_t* port, const UInt32 baud, const Pin tx, const Pin rx)
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

        /* 8N1, no flow control - what the DFPlayer's datasheet specifies. */
        uart_set_format(port, 8, 1, UART_PARITY_NONE);
        uart_set_hw_flow(port, false, false);

        /* FIFOs on, or a byte arriving while the loop is elsewhere is dropped. */
        uart_set_fifo_enabled(port, true);

        return got;
    }

    /**
     * @brief Writes bytes out on a UART port, blocking until sent.
     *
     * @param port the UART peripheral to write to
     * @param data the bytes to send
     * @param len how many bytes to send
     */
    inline Void write(uart_inst_t* port, const UInt8* data, const Size len)
    {
        uart_write_blocking(port, data, len);
    }

    /**
     * @brief Whether at least one byte is waiting to be read.
     *
     * @param port the UART peripheral to check
     * @return true when a byte is ready
     */
    inline Bool readable(uart_inst_t* port)
    {
        return uart_is_readable(port);
    }

    /**
     * @brief Reads one byte, waiting up to a timeout for it to arrive.
     *
     * @param port the UART peripheral to read from
     * @param timeoutUs how long to wait, in microseconds
     * @return the byte read, or -1 if none arrived within the timeout.
     *         Int32 rather than UInt8 so "nothing" and "the byte 0xFF" are
     *         different answers.
     */
    inline Int32 readByte(uart_inst_t* port, const UInt32 timeoutUs)
    {
        if(!uart_is_readable_within_us(port, timeoutUs))
        {
            return -1;
        }
        return uart_getc(port);
    }

    /**
     * @brief Discards whatever is sitting in the receive FIFO.
     *
     * Worth having before a command: a module that was mid-reply when the
     * Pico reset has left bytes in there, and reading them as the answer to
     * the NEXT question is the kind of bug that looks like corruption.
     *
     * @param port the UART peripheral to drain
     */
    inline Void drain(uart_inst_t* port)
    {
        while(uart_is_readable(port))
        {
            static_cast<Void>(uart_getc(port));
        }
    }

  }

  namespace timing
  {
    /* ---- time ---------------------------------------------------------------- */

    /**
     * @brief Blocks the program for a number of milliseconds.
     *
     * @param ms how long to sleep, in milliseconds
     */
    inline Void ms(const UInt32 ms)
    {
        sleep_ms(ms);
    }

    /**
     * @brief Blocks the program for a number of microseconds.
     *
     * @param us how long to sleep, in microseconds
     */
    inline Void us(const UInt64 us)
    {
        sleep_us(us);
    }

    /**
     * @brief Milliseconds elapsed since boot.
     *
     * @return the elapsed time; wraps after about 49 days.
     */
    inline UInt32 nowMs(Void)
    {
        return to_ms_since_boot(get_absolute_time());
    }

    /*
     * ---- deadlines: a point in the future you can ask about. timing:: could
     * say how long to WAIT and not when to ACT, so chassis.hxx reached past this
     * file for absolute_time_t; wrapping it is what let its test run on a laptop.
     */
    typedef absolute_time_t Deadline;

    /**
     * @brief Arms a deadline a number of milliseconds from now.
     *
     * @param ms how far in the future to set the deadline
     * @return the deadline, to be checked later with reached()
     */
    inline Deadline armMs(const UInt32 ms)
    {
        return make_timeout_time_ms(ms);
    }

    /**
     * @brief Whether a deadline has arrived.
     *
     * @param d the deadline to check
     * @return true once the deadline has passed
     */
    inline Bool reached(const Deadline d)
    {
        return time_reached(d);
    }

    /**
     * @brief Microseconds elapsed since boot.
     *
     * @return the elapsed time since boot, in microseconds
     */
    inline UInt64 nowUs(Void)
    {
        return to_us_since_boot(get_absolute_time());
    }

  }

  namespace serial
  {
    /**
     * @brief Brings up stdio (USB CDC) so the board can talk to a host.
     *
     * Nothing printed before this call arrives anywhere, and on USB the host
     * also needs a moment to enumerate - if the first few lines of a program
     * never show up, that is why, not a broken printf.
     *
     * @warning Call this in EVERY sketch, including ones that print nothing.
     *          USB is not automatic: this is what starts the device stack,
     *          and a program that never calls it never enumerates - no COM
     *          port, no VID_2E8A, nothing for the flasher to touch at 1200
     *          baud to ask for the bootloader. The board runs perfectly and
     *          is invisible to the host, and the only way back in is holding
     *          BOOTSEL while plugging the cable, by hand, every single time.
     *
     * @note This is not hypothetical. On 2026-08-26 a sketch without it went
     *       on the board, and from that moment the hub reported "no Pico
     *       found: no RPI-RP2 drive and no VID_2E8A serial port" and kept
     *       reporting it until the button was held down manually. It reads
     *       exactly like dead hardware. It costs a few KB of flash; that is
     *       what keeps the board flashable.
     */
    inline Void open(Void)
    {
        stdio_init_all();
    }

    /*
     * ---- the second listener: output goes to the USB console and, if something
     * has registered here, to the wireless link in net.h. A hook rather than a
     * call to net.h directly, because calling upward would drag lwIP into every
     * program that prints anything. Handed WHOLE LINES, still carrying their
     * newline, so one datagram equals one command reply.
     */
    typedef Void (*Mirror)(CharSeq);

    inline Mirror mirrorFn = nullptr;

    /**
     * @brief Registers a second destination for everything emit() prints.
     *
     * @param fn the mirror to call with each finished line, or nullptr to
     *           remove it
     */
    inline Void setMirror(const Mirror fn)
    {
        mirrorFn = fn;
    }

    /**
     * @brief Writes text to the USB console and, if one is set, the mirror.
     *
     * @param text the text to write; not necessarily a whole line
     */
    inline Void emit(const CharSeq text)
    {
        fputs(text, stdout);

        if(mirrorFn != nullptr)
        {
            mirrorFn(text);
        }
    }

    /**
     * @brief Writes text to the host with no trailing newline added.
     *
     * @param text the text to write
     */
    inline Void print(const CharSeq text)
    {
        emit(text);
    }

    /**
     * @brief Writes a line of text to the host, followed by a newline.
     *
     * @param text the line to write, without its own trailing newline
     */
    inline Void printLine(const CharSeq text)
    {
        emit(text);
        emit("\n");
    }

    /**
     * @brief Size of the buffer printf() formats into, in bytes.
     *
     * Every line this program prints is a short one - the longest is a DRIVE
     * report at about a hundred characters - so this is roughly double the
     * worst real case. It is not a limit anybody should be near.
     */
    static constexpr Size LINE_CAP = 256;

    /**
     * @brief Formatted output to the host, funneled through emit().
     *
     * Not a macro: this is a real inline function that forwards its va_list
     * to vsnprintf(), formats into a fixed buffer, and hands the finished
     * line to emit() rather than going straight to the C library - which is
     * what lets the same line reach the wireless link as well as the cable.
     *
     * The point is not overhead, it is that the SEAM is complete. hal.h is
     * where the SDK's spelling stops and this project's begins, and a
     * console calling printf() directly was the one place that reached past
     * it, sixty-two times. Everything a program needs now has a name from
     * this library, so the day the transport is not stdio - a UDP link, a
     * log to the SD card, both at once - there is one definition to change
     * instead of sixty-two call sites to find.
     *
     * @param fmt a printf-style format string, followed by its arguments
     * @note On truncation the tail is replaced with "...\n" instead of being
     *       left as it fell. A cut line that keeps its newline is a line the
     *       host can still parse and visibly complain about; one that loses
     *       it silently glues itself to the next line and produces a command
     *       nobody sent. This project has been bitten by exactly that shape
     *       of bug twice - a truncated cal header, and an over-long serial
     *       line whose tail became a fresh command - and both times the
     *       damage came from the truncation being SILENT.
     */
    inline Void printf(const CharSeq fmt, ...)
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

    /**
     * @brief Blocks until the host opens the USB serial port, or it times out.
     *
     * This is the fix for the complaint in open() above. USB enumeration
     * takes a moment, and anything printed before the host is listening goes
     * nowhere - so the first few lines of a program appear to vanish and the
     * program looks broken when it is merely early.
     *
     * @param timeoutMs how long to wait, in milliseconds; 0 waits forever
     * @return true if the host connected
     * @warning Passing 0 is right on the bench and WRONG on the car: a board
     *          waiting for a terminal that will never arrive is a board that
     *          never starts driving.
     */
    inline Bool waitForHost(const UInt32 timeoutMs)
    {
        const UInt32 start = to_ms_since_boot(get_absolute_time());

        while(!stdio_usb_connected())
        {
            if(timeoutMs != 0
               && to_ms_since_boot(get_absolute_time()) - start > timeoutMs)
            {
                return false;
            }
            sleep_ms(10);
        }
        return true;
    }

    /**
     * @brief Whether a host currently has the port open.
     *
     * Not the same question as waitForHost(): this one does not block, and
     * it is how a program notices a terminal ARRIVING or LEAVING mid-run
     * rather than only at startup. A board that greeted the first listener
     * and then went quiet forever looks dead to the second one.
     *
     * @return true while a host has the USB serial port open
     */
    inline Bool hostPresent(Void)
    {
        return stdio_usb_connected();
    }

    /**
     * @brief Sentinel returned by readChar() when nothing arrived in time.
     *
     * DERIVED from the SDK, never written out as a number.
     *
     * @warning It was -1 here for exactly one build, because -1 is what a
     *          sentinel looks like. The SDK says -2. Every timeout therefore
     *          failed this comparison and got stored as (Utf8)(-2) = 0xFE,
     *          so the command buffer filled with bytes nobody typed and
     *          every line came back "unknown command" preceded by a wall of
     *          rubbish. A constant that AGREES with another header is a
     *          constant that will disagree with it eventually.
     */
    static constexpr Int32 NONE = PICO_ERROR_TIMEOUT;

    /**
     * @brief Reads one character, waiting up to a timeout for it to arrive.
     *
     * A timeout rather than a block, because the caller almost always has
     * something else to do - a blink to keep, an output to slew - and a
     * read that parks the main loop turns every one of those into a
     * stutter.
     *
     * @param timeoutUs how long to wait, in microseconds
     * @return the character read, or NONE if none arrived in time
     */
    inline Int32 readChar(const UInt32 timeoutUs)
    {
        return getchar_timeout_us(timeoutUs);
    }

    /**
     * @brief Flushes anything buffered out to the host.
     *
     * Worth doing before a reboot, or the last thing the program said is
     * lost with it.
     */
    inline Void flush(Void)
    {
        stdio_flush();
    }

  }

  namespace board
  {
    /**
     * @brief Reads the chip's unique id, formatted as hex, into `out`.
     *
     * Every RP2350 has one burned in, which is what makes it useful: it
     * identifies a BOARD across reflashes, so a log line can say which of
     * two identical cars produced it.
     *
     * @param out buffer to receive the hex string; wants at least 17 bytes
     * @param cap size of `out` in bytes
     */
    inline Void id(Utf8* out, const Size cap)
    {
        pico_unique_board_id_t id;
        pico_get_unique_board_id(&id);

        Size at = 0;
        for(const Utf8Byte i : id.id)
        {
            if(at + 2 >= cap)
            {
                break;
            }
            static constexpr Utf8 HEX[] = "0123456789ABCDEF";
            out[at++] = HEX[(i >> 4) & 0x0F];
            out[at++] = HEX[i & 0x0F];
        }
        if(cap > 0)
        {
            out[at < cap ? at : cap - 1] = '\0';
        }
    }

  }

  namespace pwm
  {
    /**
     * @brief Configures a pin for PWM at a given frequency, 16-bit counter.
     *
     * The divider is derived from the ACTUAL system clock rather than
     * assumed: RP2040 boots at 125 MHz and RP2350 at 150 MHz, so a
     * hard-coded divider gives a servo a 20% wrong pulse width on one of the
     * two chips - which looks like a badly centered servo rather than a bug.
     *
     * @param pin the GPIO number to configure for PWM output
     * @param freqHz the PWM frequency to target
     */
    inline Void open(const Pin pin, const UInt32 freqHz)
    {
        gpio_set_function(static_cast<UInt32>(pin), GPIO_FUNC_PWM);

        const auto clk = static_cast<Float32>(clock_get_hz(clk_sys));
        Float32 div = clk / (static_cast<Float32>(freqHz) * static_cast<Float32>(PWM_WRAP + 1));
        if(div < 1.0f)
        {
            div = 1.0f;
        }

        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, div);
        pwm_config_set_wrap(&cfg, PWM_WRAP);
        pwm_init(pwm_gpio_to_slice_num(static_cast<UInt32>(pin)), &cfg, true);
    }

    /**
     * @brief Sets a PWM channel's duty cycle.
     *
     * @param pin the GPIO number to write
     * @param duty 0.0 to 1.0; out-of-range values are clamped
     */
    inline Void write(const Pin pin, Float32 duty)
    {
        if(duty < 0.0f)
        {
            duty = 0.0f;
        }
        if(duty > 1.0f)
        {
            duty = 1.0f;
        }
        pwm_set_gpio_level(static_cast<UInt32>(pin), static_cast<UInt16>(duty * static_cast<Float32>(PWM_WRAP)));
    }

  }

  namespace servo
  {
    /*
     * ---- servo and ESC: both speak the same 50 Hz pulse-width protocol, so
     * servo::open() is what an ESC gets too.
     */

    /**
     * @brief Brings up a pin for 50 Hz servo/ESC pulses.
     *
     * @param pin the GPIO number wired to the servo or ESC signal line
     * @warning PUT THE CAR ON A STAND, WHEELS OFF THE GROUND, for every
     *          first run of new code. docs/wiring.md says the same thing
     *          and means it.
     */
    inline Void open(const Pin pin)
    {
        pwm::open(pin, SERVO_HZ);
    }

    /**
     * @brief Holds a pulse width, in microseconds.
     *
     * @param pin the GPIO number to drive
     * @param us the pulse width to hold; clamped to the 1000-2000 us band -
     *           a servo driven past its travel stalls against its own end
     *           stop, draws locked-rotor current and cooks itself, and it
     *           does it quietly.
     */
    inline Void writeUs(const Pin pin, UInt32 us)
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
            static_cast<UInt32>(pin),
            static_cast<UInt16>(static_cast<UInt64>(us) * static_cast<UInt64>(PWM_WRAP + 1) / SERVO_PERIOD_US));
    }

    /**
     * @brief Centers a servo; neutral (no drive) for an ESC.
     *
     * @param pin the GPIO number to drive
     */
    inline Void center(const Pin pin)
    {
        writeUs(pin, SERVO_MID_US);
    }

    /**
     * @brief Stops the pulse train, so the servo goes limp.
     *
     * No holding torque, no current, nothing to push against.
     *
     * @param pin the GPIO number to release
     * @note This is the one thing neutral cannot do. If the horn is a tooth
     *       off its spline, or the linkage is the wrong length, then 1500 us
     *       IS the binding position - commanding center presses the servo
     *       against the frame just as firmly as commanding an end stop, and
     *       it will sit there and cook. The only way out is to stop asking
     *       for anything.
     * @note Level 0 leaves the pin driven LOW rather than floating. A
     *       floating signal line is worse than no signal at all: noise on
     *       it reads as random pulse widths, and the servo chases them into
     *       whatever it hits first.
     */
    inline Void release(const Pin pin)
    {
        pwm_set_gpio_level(static_cast<UInt32>(pin), 0);
    }

  }

  namespace led
  {
    /*
     * ---- the onboard LED, the one part of this header written twice.
     *
     *   Pico 2 W - the LED hangs off the CYW43439 and is NOT a GPIO: the chip is
     *              brought up first and then driven through its own GPIO space.
     *              `gpio_put(25, 1)` from a Pico 1 example compiles, runs, and
     *              does nothing at all.
     *   Pico 2   - GP25, a plain output. Bringing it up cannot fail.
     *
     * Which side is compiled is decided by the SDK's own board header.
     * led::open() must come before any other led* function and on the W it can
     * FAIL - check the return, or every later call silently does nothing.
     */

    /* Whether the lamp came up, remembered here rather than by every caller. */
    inline Bool up = false;


  }
#if defined(CYW43_WL_GPIO_LED_PIN)

  /*
   * The CYW43439 is brought up ONCE, here. Two things want that chip - the LED,
   * which hangs off its GPIO, and the wireless link in net.h - and a second
   * cyw43_arch_init() on a half-initialized radio reports no error, it just
   * leaves the chip in a state where the next thing to touch it behaves oddly.
   * So: idempotent, with the ANSWER remembered rather than the attempt repeated.
   */

  namespace radio
  {
    /* Whether cyw43_arch_init() has been attempted yet. */
    inline Bool tried = false;

    /**
     * @brief Brings up the CYW43439 wireless chip, once.
     *
     * Safe to call from more than one place: the second and later calls
     * return the answer the first call found rather than repeating the
     * attempt.
     *
     * @return true if the chip initialized successfully
     */
    inline Bool open(Void)
    {
        if(!tried)
        {
            tried = true;
            led::up      = cyw43_arch_init() == 0;
        }
        return led::up;
    }

    /**
     * @brief Whether the CYW43439 came up successfully.
     *
     * @return true once open() has succeeded
     */
    inline Bool up(Void)
    {
        return led::up;
    }


  }

  namespace led
  {
    /**
     * @brief Brings up the onboard LED, via the CYW43439.
     *
     * @return true if the chip initialized successfully
     * @note led::open() must be called before any other led* function, and
     *       on this board it can FAIL - the chip is a real peripheral on a
     *       real bus. Check the return: if you ignore it, every later call
     *       silently does nothing.
     */
    inline Bool open(Void)
    {
        return radio::open();
    }

    /**
     * @brief Sets the onboard LED.
     *
     * @param on true to light it, false to turn it off
     */
    inline Void write(const Bool on)
    {
        if(up)
        {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
        }
    }

    /**
     * @brief Reads the onboard LED's current state.
     *
     * @return true when it is lit
     */
    inline Bool read(Void)
    {
        return up && cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN);
    }

    /**
     * @brief Which backend is driving the onboard LED.
     *
     * For a program that reports its own wiring.
     *
     * @return "cyw43"
     */
    inline CharSeq backend(Void)
    {
        return "cyw43";
    }


  }
#elif defined(PICO_DEFAULT_LED_PIN)

  /*
   * No wireless chip on this board. Named anyway so callers do not need an
   * #ifdef to ask.
   */

  namespace radio
  {
    /**
     * @brief Always fails: there is no wireless chip on this board.
     *
     * @return false
     */
    inline Bool open(Void)
    {
        return false;
    }

    /**
     * @brief Always false: there is no wireless chip on this board.
     *
     * @return false
     */
    inline Bool up(Void)
    {
        return false;
    }


  }

  namespace led
  {
    /**
     * @brief Brings up the onboard LED, a plain GPIO output.
     *
     * @return true; bringing it up cannot fail on this board.
     */
    inline Bool open(Void)
    {
        gpio_init(static_cast<UInt32>(PICO_DEFAULT_LED_PIN));
        gpio_set_dir(static_cast<UInt32>(PICO_DEFAULT_LED_PIN), GPIO_OUT);
        up = true;
        return up;
    }

    /**
     * @brief Sets the onboard LED.
     *
     * @param on true to light it, false to turn it off
     */
    inline Void write(Bool on)
    {
        if(up)
        {
            gpio_put(static_cast<UInt32>(PICO_DEFAULT_LED_PIN), on);
        }
    }

    /**
     * @brief Reads the onboard LED's current state.
     *
     * @return true when it is lit
     */
    inline Bool read(Void)
    {
        return up && gpio_get(static_cast<UInt32>(PICO_DEFAULT_LED_PIN));
    }

    /**
     * @brief Which backend is driving the onboard LED.
     *
     * @return "gpio" followed by the pin number
     */
    inline CharSeq backend(Void)
    {
        return "gpio" STRINGIFY(PICO_DEFAULT_LED_PIN);
    }


  }
#else

  /*
   * A board with no lamp at all - not either of ours, but the alternative is a
   * build failing with an unreadable macro error. A program written for the car
   * still compiles and runs; it just cannot wave.
   */

  namespace radio
  {
    /**
     * @brief Always fails: there is no wireless chip on this board.
     *
     * @return false
     */
    inline Bool open(Void)
    {
        return false;
    }

    /**
     * @brief Always false: there is no wireless chip on this board.
     *
     * @return false
     */
    inline Bool up(Void)
    {
        return false;
    }


  }

  namespace led
  {
    /**
     * @brief Always fails: there is no lamp on this board.
     *
     * @return false
     */
    inline Bool open(Void)
    {
        up = false;
        return false;
    }

    /**
     * @brief Does nothing: there is no lamp on this board.
     *
     * @param on ignored
     */
    inline Void write(Bool on)
    {
        static_cast<Void>(on);
    }

    /**
     * @brief Always false: there is no lamp on this board.
     *
     * @return false
     */
    inline Bool read(Void)
    {
        return false;
    }

    /**
     * @brief Which backend is driving the onboard LED.
     *
     * @return "none"
     */
    inline CharSeq backend(Void)
    {
        return "none";
    }


  }
#endif

  namespace led
  {
    /*
     * Whether the lamp is up. For a program that wants to REPORT the failure
     * rather than merely survive it.
     */

    /**
     * @brief Whether the onboard LED is up and usable.
     *
     * @return true when open() has succeeded
     */
    inline Bool present(Void)
    {
        return up;
    }

    /**
     * @brief Flips the onboard LED's state.
     */
    inline Void toggle(Void)
    {
        write(!read());
    }

  }

  /*
   * ---- ADC: only GP26-GP29 are ADC-capable, as channels 0-3. Anything else is
   * a programming error the hardware cannot report.
   */

  namespace adc
  {
    /**
     * @brief Brings up the ADC and claims a pin for analog input.
     *
     * @param pin the GPIO number to claim; must be GP26-GP29
     */
    inline Void open(const Pin pin)
    {
        adc_init();
        adc_gpio_init(static_cast<UInt32>(pin));
    }

    /**
     * @brief Reads a 12-bit raw ADC sample from a pin.
     *
     * @param pin the GPIO number to read; must be GP26-GP29
     * @return the raw sample, 0-4095
     */
    inline UInt16 read(const Pin pin)
    {
        adc_select_input(static_cast<UInt32>(pin - 26));
        return adc_read();
    }

    /**
     * @brief Reads a pin and scales it to volts.
     *
     * @param pin the GPIO number to read; must be GP26-GP29
     * @return the reading in volts, against the 3.3 V reference
     */
    inline Float32 readVolts(const Pin pin)
    {
        return static_cast<Float32>(read(pin)) * (3.3f / 4095.0f);
    }

    /**
     * @brief Reads the RP2350's built-in die temperature sensor.
     *
     * On ADC channel 4 rather than any pin. Accurate to a couple of degrees
     * at best and it reads the CHIP, not the room - it sits a few degrees
     * above ambient as soon as the core is busy. Useful for "is this thing
     * cooking", not for weather.
     *
     * @return the die temperature in degrees Celsius, using the conversion
     *         from the RP2350 datasheet
     */
    inline Float32 tempC(Void)
    {
        adc_init();
        adc_set_temp_sensor_enabled(true);
        adc_select_input(4);

        const Float32 volts = static_cast<Float32>(adc_read()) * (3.3f / 4095.0f);
        return 27.0f - (volts - 0.706f) / 0.001721f;
    }

  }

  /*
   * ---- watchdog: the board resets if watchdog::feed() is not called within
   * `ms`. On a vehicle that is the difference between "the code hung" and "the
   * code hung and the car kept going at the last commanded throttle". A sketch
   * that enables it and then blocks in a long timing::ms() WILL reset.
   */

  namespace watchdog
  {
    /**
     * @brief Arms the watchdog: the board resets if feed() is not called
     *        within the given interval.
     *
     * @param ms how long the board may go without a feed() before it resets
     */
    inline Void start(const UInt32 ms)
    {
        watchdog_enable(ms, true);
    }

    /**
     * @brief Resets the watchdog countdown.
     *
     * Call this every loop iteration, or the board resets.
     */
    inline Void feed(Void)
    {
        watchdog_update();
    }

    /**
     * @brief Whether this boot was caused by the watchdog firing.
     *
     * Rather than by power or the reset pin. Worth printing at startup: a
     * board that is quietly resetting in a loop looks exactly like a board
     * that is slow to start.
     *
     * @return true if the watchdog caused this boot
     */
    inline Bool causedReboot(Void)
    {
        return watchdog_caused_reboot();
    }

  }

  /*
   * ---- SPI: no addresses, so every device gets its own CHIP SELECT and the one
   * held low is the one listening. Forgetting to raise CS again is the classic
   * way to make the next device on the bus appear broken. The RP2350's two
   * controllers have pins fixed by the silicon:
   *
   *   SPI0   SCK  GP2  GP6  GP18        MOSI GP3  GP7  GP19       MISO GP0 GP4 GP16
   *   SPI1   SCK  GP10 GP14 GP26        MOSI GP11 GP15 GP27       MISO GP8 GP12
   *
   * spi::open() works out which controller the pins belong to, so a wrong
   * pairing fails with a false rather than a silently dead bus. CS is
   * deliberately NOT hardware-driven: the SDK's hardware CS drops between bytes,
   * which several displays and cards read as the end of a transaction.
   */

  namespace spi
  {
    /**
     * @brief Which controller a SCK pin belongs to.
     *
     * @param sck the GPIO number to check
     * @return the controller `sck` belongs to, or nullptr if it is not a
     *         SCK pin
     */
    inline spi_inst_t* forSck(const Pin sck)
    {
        switch(sck)
        {
        case 2: case 6: case 18: case 22: return spi0;
        case 10: case 14: case 26: return spi1;
        default: return nullptr;
        }
    }

    /**
     * @brief Which controller a MISO pin belongs to.
     *
     * Worth its own function because MISO is the pin people get wrong: it
     * is the only one of the four that the silicon constrains and that a
     * plain GPIO cannot stand in for.
     *
     * CS can be any pin at all - it is just an output somebody wiggles. SCK,
     * MOSI and MISO cannot: each is a specific peripheral output on a
     * specific pad. So "I moved CS to a free pin" is always fine and "I
     * moved MISO to a free pin" is always broken, and the two mistakes look
     * identical on a wiring diagram.
     *
     *   SPI0 MISO   GP0  GP4  GP16  GP20
     *   SPI1 MISO   GP8  GP12 GP24  GP28
     *
     * @param miso the GPIO number to check
     * @return the controller `miso` belongs to, or nullptr if it is not a
     *         MISO pin
     */
    inline spi_inst_t* forMiso(const Pin miso)
    {
        switch(miso)
        {
        case 0: case 4: case 16: case 20: return spi0;
        case 8: case 12: case 24: case 28: return spi1;
        default: return nullptr;
        }
    }

    /**
     * @brief Brings up an SPI bus with no MISO, for a device that never
     *        answers.
     *
     * @param sck the clock pin
     * @param mosi the data-out pin
     * @param csPin the chip-select pin to drive, or -1 when the caller
     *              drives chip select itself, which is what you want with
     *              more than one device on the bus
     * @param hz the requested clock rate
     * @return false if `sck` and `mosi` do not belong to one controller,
     *         rather than bringing up a bus that cannot work.
     * @note `hz` is a request: the hardware picks the closest it can reach
     *       and baud() reports what was actually set.
     */
    inline Bool open(const Pin sck, const Pin mosi, const Pin csPin, const UInt32 hz)
    {
        spi_inst_t* const bus = forSck(sck);
        if(bus == nullptr)
        {
            return false;
        }

        spi_init(bus, hz);
        gpio_set_function(static_cast<UInt32>(sck), GPIO_FUNC_SPI);
        gpio_set_function(static_cast<UInt32>(mosi), GPIO_FUNC_SPI);

        if(csPin >= 0)
        {
            gpio::open(csPin, PIN_DIR_OUT);
            gpio::write(csPin, true);        /* idle HIGH; low means "listen to me" */
        }
        return true;
    }

    /**
     * @brief Brings up an SPI bus with MISO, for a device that ANSWERS.
     *
     * A display never does, which is why open() above leaves it out; an SD
     * card does nothing else.
     *
     * @param sck the clock pin
     * @param mosi the data-out pin
     * @param miso the data-in pin
     * @param csPin the chip-select pin to drive, or -1 when the caller
     *              drives chip select itself
     * @param hz the requested clock rate
     * @return false if the four pins are not all the same controller, which
     *         is the check worth having: MISO on a pad that cannot carry it
     *         fails silently, the card never appears to respond, and every
     *         symptom points at the card.
     */
    inline Bool openFull(const Pin sck, const Pin mosi, const Pin miso, const Pin csPin, const UInt32 hz)
    {
        spi_inst_t* const bus  = forSck(sck);
        if(const spi_inst_t* const rxBus = forMiso(miso); bus == nullptr || rxBus == nullptr || bus != rxBus)
        {
            return false;
        }

        spi_init(bus, hz);
        gpio_set_function(static_cast<UInt32>(sck), GPIO_FUNC_SPI);
        gpio_set_function(static_cast<UInt32>(mosi), GPIO_FUNC_SPI);
        gpio_set_function(static_cast<UInt32>(miso), GPIO_FUNC_SPI);

        /*
         * A pull-up on MISO, because an SD card leaves the line floating until it
         * is selected and a floating input reads as noise rather than as idle.
         */
        gpio_pull_up(static_cast<UInt32>(miso));

        if(csPin >= 0)
        {
            gpio::open(csPin, PIN_DIR_OUT);
            gpio::write(csPin, true);
        }
        return true;
    }

    /**
     * @brief Sets clock polarity and phase - SPI "mode".
     *
     * spi_init() does NOT set this: it leaves mode 0 (clock idles LOW,
     * sample on the leading edge).
     *
     *   mode 0   cpol false, cpha false     most sensors, most SD cards
     *   mode 3   cpol true,  cpha true      ST7789 and friends
     *
     * @param sck the clock pin identifying which controller to configure
     * @param cpol clock polarity for this mode
     * @param cpha clock phase for this mode
     * @note A device on the wrong mode does not half-work. It reads every
     *       byte shifted by a bit and behaves as though nothing was ever
     *       sent, which is indistinguishable from a wiring fault and is why
     *       this is worth naming rather than leaving to a default nobody
     *       remembers.
     */
    inline Void mode(const Pin sck, const Bool cpol, const Bool cpha)
    {
        spi_inst_t* const bus = forSck(sck);
        if(bus == nullptr)
        {
            return;
        }
        spi_set_format(bus, 8,
                       cpol ? SPI_CPOL_1 : SPI_CPOL_0,
                       cpha ? SPI_CPHA_1 : SPI_CPHA_0,
                       SPI_MSB_FIRST);
    }

    /**
     * @brief What baud rate the hardware actually settled on.
     *
     * Rarely exactly what was asked for - the divider is an integer. Worth
     * printing during bring-up.
     *
     * @param sck the clock pin identifying which controller to ask
     * @param hz the rate to request
     * @return the rate the hardware actually set, or 0 for a bad SCK pin
     */
    inline UInt32 baud(const Pin sck, const UInt32 hz)
    {
        spi_inst_t* const bus = forSck(sck);
        return bus == nullptr ? 0u : static_cast<UInt32>(spi_set_baudrate(bus, hz));
    }

    /**
     * @brief Blocking write of `n` bytes.
     *
     * @param sck the clock pin identifying which controller to use
     * @param data the bytes to send
     * @param n how many bytes to send
     * @return the number of bytes sent, or 0 for a bad SCK pin
     */
    inline Size write(const Pin sck, const UInt8* data, const Size n)
    {
        spi_inst_t* const bus = forSck(sck);
        if(bus == nullptr || data == nullptr || n == 0)
        {
            return 0;
        }
        const Int32 sent = spi_write_blocking(bus, data, n);
        return sent < 0 ? 0u : static_cast<Size>(sent);
    }

    /**
     * @brief Blocking write of a single byte.
     *
     * @param sck the clock pin identifying which controller to use
     * @param b the byte to send
     * @return 1 if the byte was sent, or 0 for a bad SCK pin
     */
    inline Size writeByte(const Pin sck, const UInt8 b)
    {
        return write(sck, &b, 1);
    }

    /**
     * @brief Full duplex transfer: sends `tx` and captures the same number
     *        of bytes into `rx`.
     *
     * @param sck the clock pin identifying which controller to use
     * @param tx the bytes to send
     * @param rx buffer to receive the bytes captured while sending
     * @param n how many bytes to transfer
     * @return the number of bytes moved, or 0 for a bad SCK pin
     */
    inline Size transfer(const Pin sck, const UInt8* tx, UInt8* rx, const Size n)
    {
        spi_inst_t* const bus = forSck(sck);
        if(bus == nullptr || tx == nullptr || rx == nullptr || n == 0)
        {
            return 0;
        }
        const Int32 moved = spi_write_read_blocking(bus, tx, rx, n);
        return moved < 0 ? 0u : static_cast<Size>(moved);
    }

  }

  /*
   * ---- I2C: two wires, many devices, no chip selects - every device has an
   * ADDRESS and the one addressed at the start of a transaction answers. Two
   * devices shipping with the SAME address cannot share a bus, which is what the
   * ToF sensors' XSHUT lines are for.
   *
   * Both lines are OPEN DRAIN: a device pulls them low and never drives them
   * high, so the bus needs about 4.7k on each line to 3V3 (most breakouts carry
   * it). Without pull-ups SDA and SCL float and nothing answers, which looks
   * exactly like a dead sensor. Which pins each controller can use is fixed:
   *
   *   I2C0   SDA GP0 GP4 GP8 GP12 GP16 GP20     SCL GP1 GP5 GP9 GP13 GP17 GP21
   *   I2C1   SDA GP2 GP6 GP10 GP14 GP18 GP26    SCL GP3 GP7 GP11 GP15 GP19 GP27
   *
   * GP4/GP5 is I2C0, and is what docs/wiring.md reserves for this bus.
   */

  /*
   * How long any single I2C transaction may take before it is abandoned. NOT A
   * TUNING PARAMETER, a safety net: the SDK's i2c_read_blocking and
   * i2c_write_blocking block FOREVER, so a device holding SDA low - lost sensor,
   * half-seated jumper, missing pull-up - stops the program and the USB stack
   * with it. The board then enumerates, answers nothing and looks bricked. It
   * has happened here and needed bootloader recovery.
   *
   * 10 ms is far longer than any transaction this project makes - a 32-byte
   * exchange at 400 kHz is under a millisecond - so a timeout means something is
   * genuinely wrong rather than merely slow.
   */
#define I2C_TIMEOUT_US 10000u

  namespace i2c
  {
    /**
     * @brief Which controller an SDA pin belongs to.
     *
     * @param sda the GPIO number to check
     * @return the controller `sda` belongs to, or nullptr if it is not an
     *         SDA pin
     */
    inline i2c_inst_t* forSda(const Pin sda)
    {
        switch(sda)
        {
        case 0: case 4: case 8: case 12: case 16: case 20: return i2c0;
        case 2: case 6: case 10: case 14: case 18: case 26: return i2c1;
        default: return nullptr;
        }
    }

    /**
     * @brief Brings up an I2C bus and enables the RP2350's internal
     *        pull-ups.
     *
     * The internal ones are weak - tens of kilohms - and are a safety net
     * rather than the real thing. They are enough for a short jumper to one
     * board and NOT enough for a long bus or several devices, where the
     * module's own 4.7k resistors do the work. Enabling them costs nothing
     * and turns "no pull-ups at all" from a silent failure into a working
     * bus.
     *
     * @param sda the data pin
     * @param scl the clock pin
     * @param hz the requested clock rate
     * @return false if the pins do not belong to one controller, rather
     *         than bringing up a bus that cannot work.
     */
    inline Bool open(const Pin sda, const Pin scl, const UInt32 hz)
    {
        i2c_inst_t* const bus = forSda(sda);
        if(bus == nullptr)
        {
            return false;
        }

        i2c_init(bus, hz);
        gpio_set_function(static_cast<UInt32>(sda), GPIO_FUNC_I2C);
        gpio_set_function(static_cast<UInt32>(scl), GPIO_FUNC_I2C);
        gpio_pull_up(static_cast<UInt32>(sda));
        gpio_pull_up(static_cast<UInt32>(scl));
        return true;
    }

    /**
     * @brief Whether anything is answering at `addr`.
     *
     * A zero-length read: the address goes out and the device either
     * acknowledges or it does not. Nothing is transferred, so this is safe
     * to do to an address you know nothing about - which is what makes
     * scanning the bus possible.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit address to probe
     * @return true if a device acknowledged
     */
    inline Bool present(const Pin sda, const UInt8 addr)
    {
        i2c_inst_t* const bus = forSda(sda);
        if(bus == nullptr)
        {
            return false;
        }

        UInt8 dummy = 0;
        return i2c_read_timeout_us(bus, addr, &dummy, 1, false, I2C_TIMEOUT_US) >= 0;
    }

    /**
     * @brief Writes `n` bytes to a device.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit address to write to
     * @param data the bytes to send
     * @param n how many bytes to send
     * @param hold true to leave the bus claimed for a repeated start, which
     *             is how a register read is done: write the register, then
     *             read without letting go
     * @return bytes written, or 0 on failure
     */
    inline Size write(const Pin sda, const UInt8 addr, const UInt8* data, const Size n, const Bool hold)
    {
        i2c_inst_t* const bus = forSda(sda);
        if(bus == nullptr || data == nullptr || n == 0)
        {
            return 0;
        }
        const Int32 sent =
            i2c_write_timeout_us(bus, addr, data, n, hold, I2C_TIMEOUT_US);
        return sent < 0 ? 0u : static_cast<Size>(sent);
    }

    /**
     * @brief Reads `n` bytes from a device.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit address to read from
     * @param data buffer to receive the bytes read
     * @param n how many bytes to read
     * @param hold true to leave the bus claimed for a repeated start
     * @return bytes read, or 0 on failure
     */
    inline Size read(const Pin sda, const UInt8 addr, UInt8* data, const Size n, const Bool hold)
    {
        i2c_inst_t* const bus = forSda(sda);
        if(bus == nullptr || data == nullptr || n == 0)
        {
            return 0;
        }
        const Int32 got =
            i2c_read_timeout_us(bus, addr, data, n, hold, I2C_TIMEOUT_US);
        return got < 0 ? 0u : static_cast<Size>(got);
    }

    /**
     * @brief Reads `n` bytes from a 16-bit register.
     *
     * This is the addressing the VL53L1X and most modern sensors use. Write
     * the register index, then read WITHOUT releasing the bus: letting go
     * between the two is what makes a sensor return the wrong register, or
     * nothing.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit device address
     * @param reg the 16-bit register index to read from
     * @param data buffer to receive the bytes read
     * @param n how many bytes to read
     * @return true if both the register write and the read succeeded
     */
    inline Bool readReg16(const Pin sda, const UInt8 addr, const UInt16 reg, UInt8* data, const Size n)
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

    /**
     * @brief Writes `n` bytes to a 16-bit register, in one transaction.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit device address
     * @param reg the 16-bit register index to write to
     * @param data the bytes to write
     * @param n how many bytes to write
     * @return true if the write succeeded
     */
    inline Bool writeReg16(const Pin sda, const UInt8 addr, const UInt16 reg, const UInt8* data, const Size n)
    {
        /*
         * Register index and payload must go out as ONE transaction, so they are
         * assembled into one buffer rather than written twice.
         */
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
        return write(sda, addr, buf, n + 2, false) == n + 2;
    }

    /**
     * @brief Writes a single byte to a 16-bit register.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit device address
     * @param reg the 16-bit register index to write to
     * @param v the byte to write
     * @return true if the write succeeded
     */
    inline Bool writeReg16U8(const Pin sda, const UInt8 addr, const UInt16 reg, const UInt8 v)
    {
        return writeReg16(sda, addr, reg, &v, 1);
    }

    /**
     * @brief Writes a big-endian 16-bit value to a 16-bit register.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit device address
     * @param reg the 16-bit register index to write to
     * @param v the value to write
     * @return true if the write succeeded
     */
    inline Bool writeReg16U16(const Pin sda, const UInt8 addr, const UInt16 reg, const UInt16 v)
    {
        UInt8 b[2];
        b[0] = static_cast<UInt8>(v >> 8);
        b[1] = static_cast<UInt8>(v & 0xFF);
        return writeReg16(sda, addr, reg, b, 2);
    }

    /**
     * @brief Reads a single byte from a 16-bit register.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit device address
     * @param reg the 16-bit register index to read from
     * @param out receives the byte read
     * @return true if the read succeeded
     */
    inline Bool readReg16U8(const Pin sda, const UInt8 addr, const UInt16 reg, UInt8* out)
    {
        return readReg16(sda, addr, reg, out, 1);
    }

    /**
     * @brief Reads a big-endian 16-bit value from a 16-bit register.
     *
     * @param sda the data pin identifying which bus to use
     * @param addr the 7-bit device address
     * @param reg the 16-bit register index to read from
     * @param out receives the value read
     * @return true if the read succeeded
     */
    inline Bool readReg16U16(const Pin sda, const UInt8 addr, const UInt16 reg, UInt16* out)
    {
        UInt8 b[2];
        if(!readReg16(sda, addr, reg, b, 2))
        {
            return false;
        }
        *out = static_cast<UInt16>((static_cast<UInt32>(b[0]) << 8u)
                                   | static_cast<UInt32>(b[1]));
        return true;
    }

  }

  namespace board
  {
    /**
     * @brief Drops the board into the UF2 bootloader.
     *
     * So it reappears as a drive and can be flashed without touching the
     * BOOTSEL button. The hub's flash path does this over USB instead
     * (1200 baud touch), but a sketch that has painted itself into a corner
     * can offer its own way out.
     */
    inline Void rebootToBootsel(Void)
    {
        /*
         * Flush and settle before going. printf() lands in a buffer a later poll
         * walks out to the host, and reset_usb_boot() does not return, so
         * anything still buffered is lost and the reboot looks like a crash. The
         * 50 ms is for the HOST, not the buffer: it needs a moment to take
         * delivery before the device stops existing.
         */
        stdio_flush();
        sleep_ms(50);
        reset_usb_boot(0, 0);
    }


  }

}

#endif /* BIBO_FAKE_HAL */
