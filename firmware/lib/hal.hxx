/*
 * hal - the Pico SDK, spelled the way this project spells things: the one place
 * the SDK's snake_case C meets our code. Header-only. GPIO numbers live in
 * pins.hxx, never here.
 */
#pragma once

#include "shared.hxx"
#include <stdarg.h>
#include <stdio.h>

/*
 * BIBO_FAKE_HAL is the host-test seam, defined only by the chassis suite: the SDK
 * headers do not compile on a laptop. It lives here because "../hal.hxx" from
 * lib/chassis/ resolves next to this file whatever the include path says.
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

  /* Hobby servo pulses, us. The servo and the ESC both expect SERVO_HZ. */
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_HZ 50
#define SERVO_PERIOD_US 20000

  /* 16-bit PWM counter, so a duty cycle resolves to about 0.3 us at SERVO_HZ. */
#define PWM_WRAP 65535

  namespace timing
  {
    inline Void ms(const UInt32 ms)
    {
        sleep_ms(ms);
    }

    /** Wraps after about 49 days. */
    inline UInt32 nowMs(Void)
    {
        return to_ms_since_boot(get_absolute_time());
    }

    inline UInt64 nowUs(Void)
    {
        return to_us_since_boot(get_absolute_time());
    }

    /* Wrapped so the chassis suite can fake it. */
    typedef absolute_time_t Deadline;

    inline Deadline armMs(const UInt32 ms)
    {
        return make_timeout_time_ms(ms);
    }

    inline Bool reached(const Deadline d)
    {
        return time_reached(d);
    }
  }

  namespace serial
  {
    /**
     * USB CDC stdio. A program that never calls this never enumerates: no COM
     * port for flash.bat's 1200-baud touch, and the only way back in is holding
     * BOOTSEL while plugging the cable in.
     */
    inline Void open(Void)
    {
        stdio_init_all();
    }

    /** printf()'s buffer, bytes: longer than any line this firmware prints. */
    static constexpr Size LINE_CAP = 256;

    /**
     * On truncation the tail becomes "...\n": a cut line that keeps its newline is
     * still one line to the host, while one that lost it glues itself to the next
     * and reads as a command nobody sent.
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

    /** Does not block, so the loop can notice a terminal arriving mid-run. */
    inline Bool hostPresent(Void)
    {
        return stdio_usb_connected();
    }

    /**
     * readChar()'s timeout. Derived from the SDK, never written as a number: a
     * wrong sentinel stores every timeout as a byte.
     */
    static constexpr Int32 NONE = PICO_ERROR_TIMEOUT;

    inline Int32 readChar(const UInt32 timeoutUs)
    {
        return getchar_timeout_us(timeoutUs);
    }
  }

  namespace board
  {
    /**
     * The RP2350's burned-in id as hex, so it names a board across reflashes. out
     * wants at least 17 bytes.
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

    inline Void rebootToBootsel(Void)
    {
        /*
         * reset_usb_boot() does not return, so flush first, and give the HOST time
         * to take delivery before the device disappears, or the reboot looks like
         * a crash.
         */
        stdio_flush();
        sleep_ms(50);
        reset_usb_boot(0, 0);
    }
  }

  namespace pwm
  {
    /**
     * The divider comes from the ACTUAL system clock: RP2040 boots at 125 MHz and
     * RP2350 at 150 MHz, and a hard-coded divider gives a servo a pulse 20% wrong
     * on one of them.
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

  /* The servo and the ESC speak the same pulse-width protocol, so both go through servo::. */
  namespace servo
  {
    inline Void open(const Pin pin)
    {
        pwm::open(pin, SERVO_HZ);
    }

    /**
     * Clamped to SERVO_MIN_US..SERVO_MAX_US: a servo driven past its travel stalls
     * against its end stop and cooks itself quietly.
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
     * Stops the pulse train, so the servo goes limp: the one thing neutral cannot
     * do when neutral IS the binding position. Level 0 leaves the pin driven LOW
     * rather than floating, where noise would read as pulses.
     */
    inline Void release(const Pin pin)
    {
        pwm_set_gpio_level(static_cast<UInt32>(pin), 0);
    }
  }

#if defined(CYW43_WL_GPIO_LED_PIN)
  /*
   * Pico 2 W: the LED hangs off the CYW43439, not an RP2350 GPIO, so the chip
   * must come up first.
   */
  namespace led
  {
    inline Bool up = false;
    inline Bool tried = false;

    /**
     * Tried once: a second cyw43_arch_init() on a half-up chip reports no error
     * and leaves it misbehaving.
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

    inline Void write(const Bool on)
    {
        if(up)
        {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
        }
    }

    inline CharSeq backend(Void)
    {
        return "cyw43";
    }
  }
#else
  /* Pico 2: the LED is a plain GPIO, and bringing it up cannot fail. */
  namespace led
  {
    inline Bool up = false;

    inline Bool open(Void)
    {
        gpio_init(static_cast<UInt32>(PICO_DEFAULT_LED_PIN));
        gpio_set_dir(static_cast<UInt32>(PICO_DEFAULT_LED_PIN), GPIO_OUT);
        up = true;
        return up;
    }

    inline Void write(const Bool on)
    {
        if(up)
        {
            gpio_put(static_cast<UInt32>(PICO_DEFAULT_LED_PIN), on);
        }
    }

    inline CharSeq backend(Void)
    {
        return "gpio" STRINGIFY(PICO_DEFAULT_LED_PIN);
    }
  }
#endif

  namespace led
  {
    inline Bool present(Void)
    {
        return up;
    }
  }
}
#endif /* BIBO_FAKE_HAL */
