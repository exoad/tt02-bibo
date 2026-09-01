/*
 * pins::begin - the runtime map check.
 *
 * The car's map is proved at compile time. THIS covers a sketch's, which is
 * written fresh against whatever is on the breadboard and is the one most
 * likely to be wrong.
 */
#include "../lib/pins.hxx"
#include <stdio.h>
using namespace bibo;

static Int32 fails = 0, checks = 0;
/**
 * @brief Records a pass/fail check and prints its outcome.
 *
 * @param ok true if the property being checked held
 * @param what the description printed alongside the outcome
 */
static Void check(Bool ok, CharSeq what)
{
    ++checks;
    printf(ok ? "  ok    %s\n" : "  FAIL  %s\n", what);
    if(!ok)
    {
        ++fails;
    }
}

/**
 * @brief Runs every pins::begin() runtime map check and reports the pass/fail count.
 *
 * @return 0 if every check passed, 1 if any failed
 */
Int32 main(Void)
{
    printf("\npins::begin\n\n");

    check(!pins::ready(), "nothing installed before begin()");
    check(pins::active().servo == pins::NONE, "an empty map binds nothing");

    check(pins::begin(pins::car()), "the car's map installs");
    check(pins::ready(), "ready after a good begin()");
    check(pins::active().soundTx == 14, "soundTx is GP14");
    check(pins::active().soundRx == 15, "soundRx is GP15");
    check(pins::active().soundBusy == 9, "soundBusy is GP9");
    check(pins::active().tailL == pins::NONE, "the tails gave up their pads");

    /* A sketch's map: three roles, everything else NONE. */
    pins::Map sk;
    sk.soundTx = 14;
    sk.soundRx = 15;
    sk.soundBusy = 9;
    check(pins::begin(sk), "a sketch map installs");
    check(pins::active().servo == pins::NONE, "and does not inherit the car");

    /* The clash. */
    pins::Map bad;
    bad.soundTx = 14;
    bad.soundBusy = 14;
    check(!pins::begin(bad), "two roles on one pad is refused");
    check(pins::conflictPin() == 14, "the clashing pad is reported");
    printf("        refused: %s and %s both want GP%d\n",
           pins::conflictFirst(), pins::conflictSecond(), pins::conflictPin());

    /* A refused map must not be applied. */
    check(pins::active().soundBusy == 9, "a refused map installs NOTHING");

    /* Out of range. */
    pins::Map oor;
    oor.servo = 30;
    check(!pins::begin(oor), "GP30 does not exist on this package");
    check(pins::conflictPin() == 30, "the bad pad is reported");

    pins::Map neg;
    neg.esc = -2;
    check(!pins::begin(neg), "a negative pad that is not NONE is refused");

    printf("\n%d checks, %d failed\n\n", checks, fails);
    return fails == 0 ? 0 : 1;
}
