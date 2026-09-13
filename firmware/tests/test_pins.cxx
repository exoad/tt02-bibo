/*
 * pins::begin - the runtime map check.    tools\test.bat pins run
 *
 * The car's map is also proved at compile time; this covers what begin() does
 * with any map it is handed, including a bad one.
 */
#include "../lib/pins.hxx"
#include <stdio.h>
using namespace bibo;

static Int32 fails = 0, checks = 0;

static Void check(Bool ok, CharSeq what)
{
    ++checks;
    printf(ok ? "  ok    %s\n" : "  FAIL  %s\n", what);
    if(!ok)
    {
        ++fails;
    }
}

Int32 main(Void)
{
    printf("\npins::begin\n\n");
    check(!pins::ready(), "nothing installed before begin()");
    check(pins::active().servo == pins::NONE, "an empty map binds nothing");
    check(pins::begin(pins::car()), "the car's map installs");
    check(pins::ready(), "ready after a good begin()");
    check(pins::active().servo == 0, "servo is GP0");
    check(pins::active().esc == 1, "esc is GP1");
    pins::Map one;
    one.esc = 5;
    check(pins::begin(one), "a partial map installs");
    check(pins::active().servo == pins::NONE, "and does not inherit the car");
    pins::Map bad;
    bad.servo = 14;
    bad.esc = 14;
    check(!pins::begin(bad), "two roles on one pad is refused");
    check(pins::conflictPin() == 14, "the clashing pad is reported");
    check(
        text::eq(pins::conflictText(), "pins servo and esc both want GP14"),
        "the complaint names both roles"
    );
    check(pins::active().esc == 5, "a refused map installs NOTHING");
    pins::Map oor;
    oor.servo = 30;
    check(!pins::begin(oor), "GP30 does not exist on this package");
    check(pins::conflictPin() == 30, "the bad pad is reported");
    check(
        text::eq(pins::conflictFirst(), pins::conflictSecond()),
        "and one role is named, not two"
    );
    pins::Map neg;
    neg.esc = -2;
    check(!pins::begin(neg), "a negative pad that is not NONE is refused");
    printf("\n%d checks, %d failed\n\n", checks, fails);
    return fails == 0 ? 0 : 1;
}
