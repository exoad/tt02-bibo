// trimfile: the car's trim as the board stores and replays it.
//
// What is held to an answer: which lines are stored and which are refused,
// that SLEW with no axis sets both, the replay order, that a hand-edited file
// with junk in it loads only its valid settings, and that save and load round
// trip - including creating a missing directory and leaving no .tmp behind.
//
// What is NOT proved here: that the Pico accepts the replayed lines, or that
// the pilot replays them on a reconnect. Those happen against a real board.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"

#include <cstdio>
#include <cstdlib>

#include "trimfile.hxx"

static Int32 checks = 0;
static Int32 failures = 0;

static Void check(Bool ok, const Char* what)
{
    ++checks;
    if(!ok)
    {
        ++failures;
    }
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
}

static Void checkStr(const Str& got, const Char* want, const Char* what)
{
    const Bool ok = got == want;
    check(ok, what);
    if(!ok)
    {
        std::printf("        got \"%s\", wanted \"%s\"\n", got.c_str(), want);
    }
}

static Void testRemember()
{
    std::printf("\n-- which lines are stored --\n");

    trimfile::Store s;
    check(trimfile::remember(s, "SERVOTRIM 1480"), "a servo centre is stored");
    checkStr(s.servoTrim, "SERVOTRIM 1480", "as the line itself");
    check(!trimfile::remember(s, "SERVOTRIM 1480"), "the same value again changes nothing");
    check(trimfile::remember(s, "SERVOTRIM 1490"), "a new value replaces it");
    checkStr(s.servoTrim, "SERVOTRIM 1490", "and only one is kept");

    check(trimfile::remember(s, "SERVOLIMITS 1230 1660"), "servo limits are stored");
    check(trimfile::remember(s, "ESCLIMITS 1564 1700"), "ESC limits are stored");
    check(trimfile::remember(s, "SLEW STEER 22"), "a steering slew is stored");
    check(trimfile::remember(s, "SLEW THROTTLE 14"), "a throttle slew is stored");
    checkStr(s.steerSlew, "SLEW STEER 22", "under its own axis");
    checkStr(s.throttleSlew, "SLEW THROTTLE 14", "and the other under its own");

    check(trimfile::remember(s, "SLEW 9"), "SLEW with no axis is stored");
    checkStr(s.steerSlew, "SLEW STEER 9", "as the steering rate");
    checkStr(s.throttleSlew, "SLEW THROTTLE 9", "AND the throttle rate, as the Pico reads it");

    std::printf("\n-- which lines are refused --\n");
    trimfile::Store r;
    check(!trimfile::remember(r, "SERVOTRIM 400"), "a centre below the servo's hard range");
    check(!trimfile::remember(r, "SERVOTRIM 1480x"), "a number with a tail on it");
    check(!trimfile::remember(r, "SERVOLIMITS 1660 1230"), "servo limits in the wrong order");
    check(!trimfile::remember(r, "ESCLIMITS 1500 2100"), "an ESC limit past the hard maximum");
    check(!trimfile::remember(r, "ESCLIMITS 900 1600"), "and one below the hard minimum");
    check(!trimfile::remember(r, "SERVOLIMITS 400 1600"), "and a servo limit below its hard minimum");
    check(!trimfile::remember(r, "SLEW 0"), "a slew of zero");
    check(!trimfile::remember(r, "SLEW SIDEWAYS 10"), "a slew for no axis the Pico has");
    check(!trimfile::remember(r, "STEER 0.500"), "a motion command, which is not trim");
    check(!trimfile::remember(r, "ESC ARM"), "and arming, which above all is not");
    check(trimfile::lines(r).empty(), "so nothing at all was stored");
}

static Void testOrderAndText()
{
    std::printf("\n-- replay order and the file's text --\n");

    // Remembered in the WRONG order on purpose.
    trimfile::Store s;
    static_cast<Void>(trimfile::remember(s, "SLEW THROTTLE 14"));
    static_cast<Void>(trimfile::remember(s, "SERVOTRIM 1480"));
    static_cast<Void>(trimfile::remember(s, "SLEW STEER 22"));
    static_cast<Void>(trimfile::remember(s, "ESCLIMITS 1564 1700"));
    static_cast<Void>(trimfile::remember(s, "SERVOLIMITS 1230 1660"));

    const Vec<Str> order = trimfile::lines(s);
    check(order.size() == 5u, "all five settings replay");
    if(order.size() == 5u)
    {
        checkStr(order[0], "SERVOLIMITS 1230 1660", "the servo limits first");
        checkStr(order[1], "ESCLIMITS 1564 1700", "then the ESC limits");
        checkStr(order[2], "SERVOTRIM 1480", "then the centre, inside the limits just set");
        checkStr(order[3], "SLEW STEER 22", "then the steering rate");
        checkStr(order[4], "SLEW THROTTLE 14", "and the throttle rate last");
    }

    const trimfile::Store back = trimfile::parse(trimfile::render(s));
    check(trimfile::lines(back) == order, "rendered and parsed, the store is unchanged");

    const Str edited =
        "# a comment\n"
        "SERVOTRIM 1470\r\n"
        "   \n"
        "ESC ARM\n"
        "SERVOTRIM banana\n"
        "ESCLIMITS 1550 1690";
    const trimfile::Store hand = trimfile::parse(edited);
    checkStr(hand.servoTrim, "SERVOTRIM 1470", "a hand-edited file keeps its valid lines, CRLF and all");
    checkStr(hand.escLimits, "ESCLIMITS 1550 1690", "including a last line with no newline");
    check(trimfile::lines(hand).size() == 2u, "and nothing else - not the ARM, not the typo");
}

static Void testSaveLoad()
{
    std::printf("\n-- saved and loaded --\n");

#if defined(_WIN32)
    const Char* tmpRoot = std::getenv("TEMP");
    const Str dir = Str(tmpRoot != nullptr ? tmpRoot : ".") + "\\bibo-trimfile-test\\nested";
    const Str path = dir + "\\trim.txt";
#else
    const Str dir = "/tmp/bibo-trimfile-test/nested";
    const Str path = dir + "/trim.txt";
#endif
    static_cast<Void>(std::remove(path.c_str()));

    trimfile::Store none;
    Str why;
    check(trimfile::load(path, none, why), "a file that does not exist loads");
    check(trimfile::lines(none).empty(), "as a car nobody has tuned");

    trimfile::Store s;
    static_cast<Void>(trimfile::remember(s, "SERVOTRIM 1485"));
    static_cast<Void>(trimfile::remember(s, "ESCLIMITS 1564 1700"));
    check(trimfile::save(path, s, why), "it saves, making the missing directory");
    if(!why.empty())
    {
        std::printf("        why: %s\n", why.c_str());
    }

    trimfile::Store back;
    check(trimfile::load(path, back, why), "and loads back");
    check(trimfile::lines(back) == trimfile::lines(s), "with the same settings");

    const Str tmp = path + ".tmp";
    std::FILE* left = std::fopen(tmp.c_str(), "rb");
    check(left == nullptr, "and no .tmp is left behind by the rename");
    if(left != nullptr)
    {
        std::fclose(left);
    }

    static_cast<Void>(trimfile::remember(s, "SERVOTRIM 1490"));
    check(trimfile::save(path, s, why), "a second save replaces the first");
    static_cast<Void>(trimfile::load(path, back, why));
    checkStr(back.servoTrim, "SERVOTRIM 1490", "and the file holds the new value");

    static_cast<Void>(std::remove(path.c_str()));
}

Int32 main()
{
    std::printf("\ntrimfile - the car's trim, kept on the board\n");
    testRemember();
    testOrderAndText();
    testSaveLoad();
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
