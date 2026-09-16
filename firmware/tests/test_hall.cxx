/* hall: the six-state decoder on the host, fed by hand. */
#include "../lib/hall.hxx"

#include <cstdio>

static Int32 failures = 0;
static Int32 checks = 0;

static Void check(const Bool ok, const CharSeq what)
{
    ++checks;
    if(ok)
    {
        std::printf("  ok    %s\n", what);
    }
    else
    {
        std::printf("  FAIL  %s\n", what);
        ++failures;
    }
}

/* n steps through the sequence from `from`, forward or back, 1 ms apart. */
static UInt64 walk(hall::Decoder& d, Int32 from, Int32 steps, Bool forward, UInt64 nowUs)
{
    Int32 at = from;
    for(Int32 i = 0; i < steps; ++i)
    {
        at = forward ? (at + 1) % 6 : (at + 5) % 6;
        nowUs += 1000;
        d.feed(hall::SEQUENCE[static_cast<Size>(at)], nowUs);
    }
    return nowUs;
}

int main()
{
    std::printf("hall: the six-state decoder\n");
    std::printf("\n-- the sequence itself --\n");
    {
        Bool oneBit = true;
        for(Size i = 0; i < 6u; ++i)
        {
            const UInt8 a = hall::SEQUENCE[i];
            const UInt8 b = hall::SEQUENCE[(i + 1u) % 6u];
            const UInt8 diff = a ^ b;
            oneBit = oneBit && (diff == 1 || diff == 2 || diff == 4);
        }
        check(oneBit, "every step of the sequence changes exactly one bit");
        Bool indexed = true;
        for(Size i = 0; i < 6u; ++i)
        {
            indexed = indexed && hall::INDEX_OF[hall::SEQUENCE[i]] == static_cast<Int8>(i);
        }
        check(indexed, "and INDEX_OF is its inverse");
        check(hall::INDEX_OF[0] < 0 && hall::INDEX_OF[7] < 0, "0 and 7 are no state");
    }
    std::printf("\n-- ticks and direction --\n");
    {
        hall::Decoder d;
        check(d.ticks == 0 && !d.primed, "starts at zero, unprimed");
        d.feed(hall::SEQUENCE[0], 1000);
        check(
            d.primed && d.ticks == 0 && d.edges == 0,
            "the first valid state primes and counts nothing"
        );
        UInt64 t = walk(d, 0, 12, true, 1000);
        check(
            d.ticks == 12 && d.edges == 12 && d.errors() == 0,
            "twelve steps forward: +12, two turns, no error"
        );
        t = walk(d, 0, 5, false, t);
        check(d.ticks == 7 && d.lastDir == -1, "five back: 7, and the direction is remembered");
        check(d.periodUs == 1000, "the period is the gap between the last two steps");
        d.feed(d.state, t + 500);
        check(
            d.ticks == 7 && d.repeats == 1 && d.errors() == 0,
            "an edge that changed nothing is a repeat, not an error"
        );
    }
    std::printf("\n-- what is an error --\n");
    {
        hall::Decoder d;
        d.feed(hall::SEQUENCE[0], 1000);
        d.feed(hall::SEQUENCE[2], 2000);
        check(
            d.ticks == 0 && d.skips == 1 && d.errors() == 1,
            "two steps at once is a skip, and not a tick"
        );
        d.feed(hall::SEQUENCE[3], 3000);
        check(
            d.ticks == 1 && d.errors() == 1,
            "and the step after it counts again from the new state"
        );
        d.feed(0, 4000);
        check(
            d.invalid == 1 && d.errors() == 2 && !d.primed,
            "all three low is invalid and unprimes"
        );
        d.feed(hall::SEQUENCE[3], 5000);
        d.feed(hall::SEQUENCE[4], 6000);
        check(
            d.ticks == 2 && d.errors() == 2,
            "after an invalid state the next valid one primes, the one after counts"
        );
        d.feed(7, 7000);
        check(d.invalid == 2, "all three high is invalid too");
    }
    std::printf("\n-- speed --\n");
    {
        hall::Decoder d;
        d.feed(hall::SEQUENCE[0], 0);
        const UInt64 t = walk(d, 0, 30, true, 0);   /* 30 steps, 1 ms apart: 1000 ticks/s */
        hall::Speed s = hall::speed(30, d, t);
        check(s.windowTps == 600, "30 ticks in a 50 ms window is 600 ticks per second");
        check(s.periodTps == 1000, "a 1 ms period is 1000 ticks per second, signed forward");
        check(!s.usePeriod, "with 30 ticks in the window, the window is trusted");
        s = hall::speed(3, d, t);
        check(s.usePeriod, "with 3, the period is");
        hall::Decoder back;
        back.feed(hall::SEQUENCE[0], 0);
        const UInt64 tb = walk(back, 0, 4, false, 0);
        s = hall::speed(-4, back, tb);
        check(s.periodTps == -1000 && s.windowTps == -80, "backwards is negative both ways");
        s = hall::speed(0, d, t + hall::STOPPED_US + 1);
        check(
            s.stopped && s.periodTps == 0 && s.windowTps == 0 && !s.usePeriod,
            "a quarter second without an edge is stopped"
        );
        hall::Decoder fresh;
        s = hall::speed(0, fresh, 5000);
        check(s.stopped, "and so is a decoder that has seen nothing");
    }
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
