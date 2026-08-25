/*
 * pico_debug - bring-up and debug firmware for the car's Pico 2 W.
 *
 * Phase 2 of the build order (see AGENTS.md) is "Pico replaces receiver, USB
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
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/bootrom.h"
#include "pico/unique_id.h"

#define LINE_MAX 128

static bool  g_cyw43_ok   = false;   /* did the wireless chip come up?        */
static bool  g_led_on     = false;
static float g_blink_hz   = 0.0f;    /* 0 = not blinking                      */
static absolute_time_t g_next_toggle;

/* ------------------------------------------------------------------ led --- */

static void led_write(bool on)
{
    g_led_on = on;
    if (g_cyw43_ok)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

static void led_set_blink(float hz)
{
    g_blink_hz = hz;
    if (hz > 0.0f) {
        /* Half-period per toggle, so `hz` is full on-off cycles per second. */
        const int64_t half_us = (int64_t)(500000.0f / hz);
        g_next_toggle = make_timeout_time_us(half_us);
    }
}

static void led_tick(void)
{
    if (g_blink_hz <= 0.0f) return;
    if (!time_reached(g_next_toggle)) return;

    led_write(!g_led_on);
    const int64_t half_us = (int64_t)(500000.0f / g_blink_hz);
    g_next_toggle = make_timeout_time_us(half_us);
}

/* -------------------------------------------------------------- commands -- */

static void print_id(void)
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    char hex[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i)
        snprintf(hex + i * 2, 3, "%02X", id.id[i]);

    printf("INFO id board=%s sdk=%s built=%s %s uid=%s cyw43=%s\n",
           PICO_BOARD, PICO_SDK_VERSION_STRING, __DATE__, __TIME__,
           hex, g_cyw43_ok ? "up" : "FAILED");
}

static void print_status(void)
{
    printf("INFO status up_ms=%u led=%s blink_hz=%.2f cyw43=%s\n",
           (unsigned)to_ms_since_boot(get_absolute_time()),
           g_led_on ? "on" : "off",
           (double)g_blink_hz,
           g_cyw43_ok ? "up" : "FAILED");
}

static void print_help(void)
{
    printf("INFO help PING - answers PONG\n");
    printf("INFO help ID - board, sdk, build time, unique id\n");
    printf("INFO help STATUS - uptime and led state\n");
    printf("INFO help LED ON|OFF - solid\n");
    printf("INFO help LED BLINK <hz> - 0 stops\n");
    printf("INFO help BOOTSEL - reboot into the UF2 bootloader\n");
}

/* Uppercase in place, so commands are accepted in any case. */
static void upper(char *s)
{
    for (; *s; ++s) *s = (char)toupper((unsigned char)*s);
}

static void handle_line(char *line)
{
    /* Trim trailing CR/space that a terminal may append. */
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == ' ' || line[n - 1] == '\t'))
        line[--n] = '\0';
    if (n == 0) return;

    upper(line);

    if (strcmp(line, "PING") == 0)   { printf("PONG\n");   return; }
    if (strcmp(line, "ID") == 0)     { print_id();         return; }
    if (strcmp(line, "STATUS") == 0) { print_status();     return; }
    if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0) { print_help(); return; }

    if (strcmp(line, "BOOTSEL") == 0) {
        printf("INFO rebooting into bootloader\n");
        stdio_flush();
        sleep_ms(50);
        reset_usb_boot(0, 0);       /* does not return */
        return;
    }

    if (strncmp(line, "LED ", 4) == 0) {
        const char *arg = line + 4;

        if (strcmp(arg, "ON") == 0) {
            led_set_blink(0.0f);
            led_write(true);
            printf("OK led on%s\n", g_cyw43_ok ? "" : " (cyw43 down, no effect)");
            return;
        }
        if (strcmp(arg, "OFF") == 0) {
            led_set_blink(0.0f);
            led_write(false);
            printf("OK led off%s\n", g_cyw43_ok ? "" : " (cyw43 down, no effect)");
            return;
        }
        if (strncmp(arg, "BLINK", 5) == 0) {
            const float hz = (float)atof(arg + 5);
            if (hz < 0.0f || hz > 50.0f) {
                printf("ERR blink rate out of range (0-50 hz)\n");
                return;
            }
            led_set_blink(hz);
            if (hz == 0.0f) led_write(false);
            printf("OK led blink %.2f\n", (double)hz);
            return;
        }
        printf("ERR bad LED argument: %s\n", arg);
        return;
    }

    printf("ERR unknown command: %s\n", line);
}

/* ------------------------------------------------------------------ main -- */

int main(void)
{
    stdio_init_all();

    /* Brings up the CYW43439. Without this the LED cannot be driven at all on
     * a Pico 2 W. It is slow (hundreds of ms) and can fail, so its result is
     * reported rather than assumed - a board that answers PING but reports
     * cyw43=FAILED is a very different problem from a board that is silent. */
    g_cyw43_ok = (cyw43_arch_init() == 0);

    /* Visible proof of life the moment power is applied, before any host has
     * opened the port: three quick flashes, then a slow idle heartbeat. */
    if (g_cyw43_ok) {
        for (int i = 0; i < 3; ++i) {
            led_write(true);  sleep_ms(80);
            led_write(false); sleep_ms(80);
        }
    }
    led_set_blink(0.5f);

    char line[LINE_MAX];
    size_t len = 0;
    bool announced = false;

    for (;;) {
        led_tick();

        /* Anything written before the host opens the port is discarded, so the
         * banner waits for a connection rather than being lost. */
        if (!announced && stdio_usb_connected()) {
            printf("INFO ready %s sdk=%s - type HELP\n",
                   PICO_BOARD, PICO_SDK_VERSION_STRING);
            announced = true;
        }
        if (announced && !stdio_usb_connected())
            announced = false;      /* re-announce on the next connection */

        const int c = getchar_timeout_us(1000);   /* 1 ms: keeps blink smooth */
        if (c == PICO_ERROR_TIMEOUT) continue;

        if (c == '\n' || c == '\r') {
            line[len] = '\0';
            handle_line(line);
            len = 0;
        } else if (len + 1 < LINE_MAX) {
            line[len++] = (char)c;
        } else {
            /* Overlong line: drop it rather than silently truncating into a
             * command that means something else. */
            len = 0;
            printf("ERR line too long\n");
        }
    }
}
