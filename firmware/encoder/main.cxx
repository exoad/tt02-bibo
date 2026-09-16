/*
 * encoder - the second Pico: a hall-sensor encoder node and nothing else. It
 * listens to the motor's three hall lines beside the ESC, keeps a signed tick
 * count, and reports over USB CDC twenty times a second.
 *
 * DECLARE the pins, BIND the decoder to them through interrupts, RUN the report
 * loop. The decoder is lib/hall.hxx, which never sees a GPIO; this file is the
 * only place the pin numbers live.
 *
 * Every edge is an interrupt that reads all three lines at once, so the state
 * is judged whole and never from a stale pin. At 20,000 RPM a two-pole motor
 * makes 2,000 edges a second; the interrupt costs about a microsecond, so ten
 * times that still leaves the core mostly idle. As a check on that promise,
 * the loop compares the lines it can see with the state the interrupts last
 * recorded, and a difference is counted (miss=) rather than corrected quietly.
 *
 * GP0 carries a throttle pulse to the ESC, neutral from power-up, so an ESC
 * sharing the bench with this board arms quietly instead of beeping for a
 * signal. It is NOT the car's throttle: no watchdog, no arming rules, a
 * bench convenience with the wheels off the ground, and it goes back to
 * neutral the moment the USB host is gone.
 *
 * One line a report, at REPORT_HZ:
 *
 *   hall t=<ticks> tps=<speed> src=window|period|stopped win=<tps> per=<tps>
 *        err=<skips+invalid> skip=<n> bad=<n> rep=<n> miss=<n> state=<ABC> edges=<n> esc=<us>
 *
 * Lines in: `z` zeroes every counter, `?` prints the help, `n` puts the ESC
 * pulse at neutral, `p <us>` sets it (ESC_LOW_US..ESC_HIGH_US).
 */
#include "../lib/hal.hxx"
#include "../lib/hall.hxx"

#include "hardware/gpio.h"
#include "hardware/sync.h"

/* DECLARE: the three hall lines, through 10k/20k dividers, active high. */
static constexpr UInt32 PIN_HALL_A = 11;
static constexpr UInt32 PIN_HALL_B = 12;
static constexpr UInt32 PIN_HALL_C = 13;

static constexpr UInt32 REPORT_HZ = 20;

/* The ESC's signal line, and the pulse band a bench may ask for. */
static constexpr bibo::Pin PIN_ESC = 0;
static constexpr UInt32 ESC_NEUTRAL_US = 1500;
static constexpr UInt32 ESC_LOW_US = 1000;
static constexpr UInt32 ESC_HIGH_US = 2000;
static constexpr Size LINE_CAP = 32;

/* Written by the interrupt, read by the loop under a disabled-interrupt window. */
static hall::Decoder decoder;
static volatile UInt32 misses = 0;

static UInt8 linesNow(Void)
{
    const UInt32 all = gpio_get_all();
    return static_cast<UInt8>(
        (((all >> PIN_HALL_A) & 1u) << 2) | (((all >> PIN_HALL_B) & 1u) << 1) | ((all >> PIN_HALL_C) & 1u)
    );
}

static Void onEdge(uint gpio, UInt32 events)
{
    static_cast<Void>(gpio);
    static_cast<Void>(events);
    decoder.feed(linesNow(), bibo::timing::nowUs());
}

static Void printHelp(Void)
{
    bibo::serial::printf(
        "INFO hall encoder: A=GP%u B=GP%u C=GP%u, %u reports/s, ESC pulse on GP%d; "
        "z zeroes, n neutral, p <us> sets the pulse (%u..%u), ? prints this\n",
        PIN_HALL_A,
        PIN_HALL_B,
        PIN_HALL_C,
        REPORT_HZ,
        static_cast<Int32>(PIN_ESC),
        ESC_LOW_US,
        ESC_HIGH_US
    );
}

/* A whole line from the host, or false while one is still arriving. */
static Bool readLine(Utf8* line, Size& len)
{
    for(;;)
    {
        const Int32 c = bibo::serial::readChar(500);
        if(c == bibo::serial::NONE)
        {
            return false;
        }
        if(c == '\n' || c == '\r')
        {
            if(len == 0u)
            {
                continue;
            }
            line[len] = 0;
            len = 0;
            return true;
        }
        if(len + 1u < LINE_CAP)
        {
            line[len++] = static_cast<Utf8>(c);
        }
    }
}

int main(Void)
{
    bibo::serial::open();
    /* BIND: inputs pulled down, so an unplugged sensor cable reads 000 - an
     * invalid state that shows as bad= - rather than floating into counts. */
    static constexpr UInt32 PINS[3] = { PIN_HALL_A, PIN_HALL_B, PIN_HALL_C };
    for(const UInt32 pin : PINS)
    {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_down(pin);
    }
    /* The decoder starts from the lines as they are, so the first edge is a step
     * and not a priming. */
    decoder.feed(linesNow(), bibo::timing::nowUs());
    /* The ESC hears neutral from the first frame. */
    bibo::servo::open(PIN_ESC);
    bibo::servo::writeUs(PIN_ESC, ESC_NEUTRAL_US);
    UInt32 escUs = ESC_NEUTRAL_US;
    gpio_set_irq_enabled_with_callback(
        PIN_HALL_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        onEdge
    );
    gpio_set_irq_enabled(PIN_HALL_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_HALL_C, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    printHelp();
    /* RUN. */
    Int32 windowStartTicks = 0;
    Int32 windowTicks = 0;
    bibo::timing::Deadline window = bibo::timing::armMs(hall::WINDOW_MS);
    bibo::timing::Deadline report = bibo::timing::armMs(1000 / REPORT_HZ);
    Utf8 line[LINE_CAP];
    Size lineLen = 0;
    for(;;)
    {
        Utf8 c = 0;
        Bool haveLine = readLine(line, lineLen);
        if(haveLine)
        {
            c = line[0];
        }
        if(haveLine && c == 'p')
        {
            /* p <us>: digits after the letter, nothing else. */
            UInt32 us = 0;
            Bool digits = false;
            for(Size i = 1; line[i] != 0; ++i)
            {
                if(line[i] >= '0' && line[i] <= '9')
                {
                    us = us * 10u + static_cast<UInt32>(line[i] - '0');
                    digits = true;
                }
                else if(line[i] != ' ')
                {
                    digits = false;
                    break;
                }
            }
            if(!digits || us < ESC_LOW_US || us > ESC_HIGH_US)
            {
                bibo::serial::printf("ERR p wants a pulse in %u..%u us\n", ESC_LOW_US, ESC_HIGH_US);
            }
            else
            {
                escUs = us;
                bibo::servo::writeUs(PIN_ESC, escUs);
                bibo::serial::printf("OK esc %u us\n", escUs);
            }
        }
        else if(haveLine && c == 'n')
        {
            escUs = ESC_NEUTRAL_US;
            bibo::servo::writeUs(PIN_ESC, escUs);
            bibo::serial::printf("OK esc neutral\n");
        }
        else if(haveLine && c == 'z')
        {
            const UInt32 saved = save_and_disable_interrupts();
            const UInt8 state = decoder.state;
            decoder = hall::Decoder();
            decoder.feed(state, bibo::timing::nowUs());
            misses = 0;
            restore_interrupts(saved);
            windowStartTicks = 0;
            windowTicks = 0;
            bibo::serial::printf("OK zeroed\n");
        }
        else if(haveLine && c == '?')
        {
            printHelp();
        }
        else if(haveLine)
        {
            bibo::serial::printf("ERR unknown: %s\n", line);
        }
        /* No host, no throttle: a pulse set from a terminal that has gone
         * is not a pulse anyone is watching. */
        if(escUs != ESC_NEUTRAL_US && !bibo::serial::hostPresent())
        {
            escUs = ESC_NEUTRAL_US;
            bibo::servo::writeUs(PIN_ESC, escUs);
        }
        /* The lines as the loop sees them against the state the interrupts left:
         * a difference is an edge the interrupt did not see. The line is fed so
         * the decoder judges it (a skip, most likely) rather than staying wrong. */
        {
            const UInt32 saved = save_and_disable_interrupts();
            const UInt8 live = linesNow();
            if(decoder.primed && live != decoder.state)
            {
                ++misses;
                decoder.feed(live, bibo::timing::nowUs());
            }
            restore_interrupts(saved);
        }
        if(bibo::timing::reached(window))
        {
            window = bibo::timing::armMs(hall::WINDOW_MS);
            const UInt32 saved = save_and_disable_interrupts();
            const Int32 now = decoder.ticks;
            restore_interrupts(saved);
            windowTicks = now - windowStartTicks;
            windowStartTicks = now;
        }
        if(bibo::timing::reached(report))
        {
            report = bibo::timing::armMs(1000 / REPORT_HZ);
            const UInt32 saved = save_and_disable_interrupts();
            const hall::Decoder snap = decoder;
            const UInt32 missed = misses;
            restore_interrupts(saved);
            const hall::Speed s = hall::speed(windowTicks, snap, bibo::timing::nowUs());
            const CharSeq src = s.stopped ? "stopped" : (s.usePeriod ? "period" : "window");
            const Int32 tps = s.usePeriod ? s.periodTps : s.windowTps;
            bibo::serial::printf(
                "hall t=%ld tps=%ld src=%s win=%ld per=%ld err=%lu skip=%lu bad=%lu rep=%lu miss=%lu state=%u%u%u edges=%lu esc=%u\n",
                static_cast<long>(snap.ticks),
                static_cast<long>(tps),
                src,
                static_cast<long>(s.windowTps),
                static_cast<long>(s.periodTps),
                static_cast<unsigned long>(snap.errors()),
                static_cast<unsigned long>(snap.skips),
                static_cast<unsigned long>(snap.invalid),
                static_cast<unsigned long>(snap.repeats),
                static_cast<unsigned long>(missed),
                (snap.state >> 2) & 1u,
                (snap.state >> 1) & 1u,
                snap.state & 1u,
                static_cast<unsigned long>(snap.edges),
                escUs
            );
        }
    }
}
