/*
 * encoder - a second Pico as a hall-sensor encoder node and nothing else: the
 * bench image the encoder was proven on before it moved onto the car's Pico.
 * It counts the motor's three hall lines through lib/encoder.hxx, the same
 * module the car runs, and reports over USB CDC twenty times a second, so the
 * module can be watched on its own with no car firmware in the way.
 *
 * DECLARE the pins, BIND the module to them, RUN the report loop. The decoder
 * is lib/hall.hxx, the binding lib/encoder.hxx; this file is the only place
 * the pin numbers live.
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
#include "../lib/encoder.hxx"

/* DECLARE: the three hall lines, through 10k/20k dividers, active high. */
static constexpr bibo::Pin PIN_HALL_A = 11;
static constexpr bibo::Pin PIN_HALL_B = 12;
static constexpr bibo::Pin PIN_HALL_C = 13;

static constexpr UInt32 REPORT_HZ = 20;

/* The ESC's signal line, and the pulse band a bench may ask for. */
static constexpr bibo::Pin PIN_ESC = 0;
static constexpr UInt32 ESC_NEUTRAL_US = 1500;
static constexpr UInt32 ESC_LOW_US = 1000;
static constexpr UInt32 ESC_HIGH_US = 2000;
static constexpr Size LINE_CAP = 32;

static Void printHelp(Void)
{
    bibo::serial::printf(
        "INFO hall encoder: A=GP%d B=GP%d C=GP%d, %u reports/s, ESC pulse on GP%d; "
        "z zeroes, n neutral, p <us> sets the pulse (%u..%u), ? prints this\n",
        static_cast<Int32>(PIN_HALL_A),
        static_cast<Int32>(PIN_HALL_B),
        static_cast<Int32>(PIN_HALL_C),
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

static Void report(const UInt32 escUs)
{
    const bibo::encoder::Snapshot e = bibo::encoder::read();
    const CharSeq src = e.stopped ? "stopped" : (e.usePeriod ? "period" : "window");
    bibo::serial::printf(
        "hall t=%ld tps=%ld src=%s win=%ld per=%ld err=%lu skip=%lu bad=%lu rep=%lu miss=%lu state=%u%u%u edges=%lu esc=%u\n",
        static_cast<long>(e.ticks),
        static_cast<long>(e.ticksPerS),
        src,
        static_cast<long>(e.windowTps),
        static_cast<long>(e.periodTps),
        static_cast<unsigned long>(e.skips + e.invalid),
        static_cast<unsigned long>(e.skips),
        static_cast<unsigned long>(e.invalid),
        static_cast<unsigned long>(e.repeats),
        static_cast<unsigned long>(e.misses),
        (static_cast<UInt32>(e.state) >> 2) & 1u,
        (static_cast<UInt32>(e.state) >> 1) & 1u,
        static_cast<UInt32>(e.state) & 1u,
        static_cast<unsigned long>(e.edges),
        escUs
    );
}

int main(Void)
{
    bibo::serial::open();
    /* BIND. */
    bibo::encoder::open(PIN_HALL_A, PIN_HALL_B, PIN_HALL_C);
    /* The ESC hears neutral from the first frame. */
    bibo::servo::open(PIN_ESC);
    bibo::servo::writeUs(PIN_ESC, ESC_NEUTRAL_US);
    UInt32 escUs = ESC_NEUTRAL_US;
    printHelp();
    /* RUN. */
    bibo::timing::Deadline next = bibo::timing::armMs(1000 / REPORT_HZ);
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
            bibo::encoder::zero();
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
        bibo::encoder::pump();
        if(bibo::timing::reached(next))
        {
            next = bibo::timing::armMs(1000 / REPORT_HZ);
            report(escUs);
        }
    }
}
