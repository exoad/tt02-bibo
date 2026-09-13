/*
 * hal - the Pico SDK, spelled the way this project spells things. The SDK is
 * snake_case C; above this file everything is ours, and this is the one place
 * the two meet. Everything is inline, so there is no library to link.
 *
 * The car's pads live in pins.hxx, which fails the build when two roles claim
 * one; nothing here holds a GPIO number.
 */

#pragma once

/* The vocabulary. A sibling of this file, so a tool without the project resolves it. */
#include "shared.hxx"

/* vsnprintf and va_list, named because this file uses them. */
#include <stdarg.h>
#include <stdio.h>

/*
 * BIBO_FAKE_HAL - the host-test seam, defined only by the chassis suite: the SDK
 * headers below do not compile on a laptop. The switch lives here because
 * "../hal.hxx" from lib/chassis/ resolves next to this file whatever the include
 * path says.
 */
#ifdef BIBO_FAKE_HAL

#include "../tests/fakes/hal.hxx"

#else

/* FIRST: it drags in the board header, which decides whether there is a radio. */
#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"

/* Only on a board that HAS the chip: a plain Pico 2 does not link pico_cyw43_arch. */
#if defined(CYW43_WL_GPIO_LED_PIN)
#include "pico/cyw43_arch.h"
#endif

namespace bibo
{

  /* A GPIO number, NOT a pin number: GP28 is Pin 28 here, pin 34 on the board. */
  typedef Int32 Pin;

  /* Hobby servo pulses. Both the servo and the ESC expect 50 Hz. */
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_HZ 50
#define SERVO_PERIOD_US 20000

  /* 16-bit PWM counter, so a duty cycle resolves to about 0.3 us at 50 Hz. */
#define PWM_WRAP 65535

  namespace timing
  {
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
     * @brief Milliseconds elapsed since boot.
     *
     * @return the elapsed time; wraps after about 49 days
     */
    inline UInt32 nowMs(Void)
    {
        return to_ms_since_boot(get_absolute_time());
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

    /* A point in the future you can ask about - wrapped so chassis.hxx's suite can fake it. */
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

  }

  namespace serial
  {
    /**
     * @brief Brings up stdio over USB CDC so the board can talk to a host.
     *
     * @warning A program that never calls this never enumerates: no COM port,
     *          nothing for flash.bat's 1200-baud touch to reach, and the only
     *          way back in is holding BOOTSEL while plugging the cable in.
     */
    inline Void open(Void)
    {
        stdio_init_all();
    }

    /**
     * @brief Size of the buffer printf() formats into, in bytes - longer than
     *        any line this firmware prints.
     */
    static constexpr Size LINE_CAP = 256;

    /**
     * @brief Formatted output to the host.
     *
     * @param fmt a printf-style format string, followed by its arguments
     * @note On truncation the tail becomes "...\n". A cut line that keeps its
     *       newline is still one line to the host; one that lost it glues
     *       itself to the next and reads as a command nobody sent.
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

        fputs(buf, stdout);
    }

    /**
     * @brief Whether a host currently has the port open.
     *
     * Does not block, so the loop can notice a terminal arriving mid-run and
     * greet it.
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
     * DERIVED from the SDK, never written out: a -1 here once turned every
     * timeout into a stored 0xFE byte.
     */
    static constexpr Int32 NONE = PICO_ERROR_TIMEOUT;

    /**
     * @brief Reads one character, waiting up to a timeout for it to arrive.
     *
     * @param timeoutUs how long to wait, in microseconds
     * @return the character read, or NONE if none arrived in time
     */
    inline Int32 readChar(const UInt32 timeoutUs)
    {
        return getchar_timeout_us(timeoutUs);
    }

  }

  namespace board
  {
    /**
     * @brief Reads the chip's unique id, formatted as hex, into `out`.
     *
     * Burned into every RP2350, so it names a board across reflashes.
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

    /**
     * @brief Drops the board into the UF2 bootloader, so it reappears as a
     *        drive without anyone touching the BOOTSEL button.
     */
    inline Void rebootToBootsel(Void)
    {
        /*
         * Flush and settle first: reset_usb_boot() does not return, so anything
         * still buffered is lost and the reboot looks like a crash. The 50 ms is
         * for the HOST to take delivery before the device disappears.
         */
        stdio_flush();
        sleep_ms(50);
        reset_usb_boot(0, 0);
    }

  }

  namespace pwm
  {
    /**
     * @brief Configures a pin for PWM at a given frequency, 16-bit counter.
     *
     * The divider comes from the ACTUAL system clock: RP2040 boots at 125 MHz
     * and RP2350 at 150 MHz, and a hard-coded divider gives a servo a pulse 20%
     * wrong on one of them.
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

  }

  namespace servo
  {
    /*
     * The servo and the ESC speak the same 50 Hz pulse-width protocol, so an
     * ESC is opened and written through here too.
     */

    /**
     * @brief Brings up a pin for 50 Hz servo/ESC pulses.
     *
     * @param pin the GPIO number wired to the servo or ESC signal line
     * @warning PUT THE CAR ON A STAND, WHEELS OFF THE GROUND, for every first
     *          run of new code.
     */
    inline Void open(const Pin pin)
    {
        pwm::open(pin, SERVO_HZ);
    }

    /**
     * @brief Holds a pulse width, in microseconds.
     *
     * @param pin the GPIO number to drive
     * @param us the pulse width to hold; clamped to SERVO_MIN_US..SERVO_MAX_US,
     *           because a servo driven past its travel stalls against its own
     *           end stop and cooks itself quietly
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
            static_cast<UInt16>(static_cast<UInt64>(us) * static_cast<UInt64>(PWM_WRAP + 1) / SERVO_PERIOD_US)
        );
    }

    /**
     * @brief Stops the pulse train, so the servo goes limp.
     *
     * The one thing neutral cannot do: if the horn is a tooth off its spline,
     * 1500 us IS the binding position, and the only way out is to stop asking
     * for anything. Level 0 leaves the pin driven LOW rather than floating,
     * where noise would read as random pulses.
     *
     * @param pin the GPIO number to release
     */
    inline Void release(const Pin pin)
    {
        pwm_set_gpio_level(static_cast<UInt32>(pin), 0);
    }

  }

#if defined(CYW43_WL_GPIO_LED_PIN)

  namespace led
  {
    /*
     * Pico 2 W: the LED hangs off the CYW43439 and is NOT an RP2350 GPIO, so the
     * chip has to come up before the LED will do anything.
     */

    /* Whether the lamp came up, and whether bringing it up has been tried. */
    inline Bool up = false;
    inline Bool tried = false;

    /**
     * @brief Brings up the onboard LED, via the CYW43439.
     *
     * Tried once: a second cyw43_arch_init() on a half-up chip reports no
     * error and leaves it misbehaving.
     *
     * @return true if the chip came up; false leaves every later write a no-op
     */
    inline Bool open(Void)
    {
        if(!tried)
        {
            tried = true;
            up = cyw43_arch_init() == 0;
        }
        return up;
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
     * @brief Which backend drives the onboard LED, for ID to report.
     *
     * @return "cyw43"
     */
    inline CharSeq backend(Void)
    {
        return "cyw43";
    }

  }

#else

  namespace led
  {
    /* Pico 2: the LED is a plain GPIO, and bringing it up cannot fail. */

    inline Bool up = false;

    /**
     * @brief Brings up the onboard LED, a plain GPIO output.
     *
     * @return true
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
    inline Void write(const Bool on)
    {
        if(up)
        {
            gpio_put(static_cast<UInt32>(PICO_DEFAULT_LED_PIN), on);
        }
    }

    /**
     * @brief Which backend drives the onboard LED, for ID to report.
     *
     * @return "gpio" followed by the pin number
     */
    inline CharSeq backend(Void)
    {
        return "gpio" STRINGIFY(PICO_DEFAULT_LED_PIN);
    }

  }

#endif

  namespace led
  {
    /**
     * @brief Whether the onboard LED is up and usable.
     *
     * @return true when open() has succeeded
     */
    inline Bool present(Void)
    {
        return up;
    }

  }

}

#endif /* BIBO_FAKE_HAL */
