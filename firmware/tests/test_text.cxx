/* The parsers in lib/text.h.
 *
 *   firmware\tests\build_text_test.bat run
 *
 * These exist because text.h CHANGED THE SEMANTICS of parsing, and a stricter
 * parser is only an improvement if it still accepts everything valid. The old
 * code used atoi(), which cannot fail - it returns 0 for "0", for "banana" and
 * for "", so every caller tested `if(us == 0)` and called that an error. That
 * works only as long as no command legitimately accepts zero, and the day one
 * does it inherits a parser that rejects it.
 *
 * So the interesting cases are not "does 1500 parse". They are:
 *
 *   * "0" must SUCCEED and yield 0, which is the whole reason for the change
 *   * "12abc" must FAIL, where atoi() would have said 12
 *   * "" and "banana" must fail without touching the caller's variable
 *   * bibo::text::starts must not match a prefix longer than the string
 *
 * Compiled for the HOST, not the board: text.h needs nothing from the Pico SDK,
 * only types.h. That is a property worth keeping - a parser that can only be
 * exercised by flashing a microcontroller is a parser nobody exercises.
 *
 * Exits 0 on PASS, 1 on FAIL.
 */

#include "../lib/text.hxx"

#include <stdio.h>
#include <string.h>

static Int32 failures = 0;
static Int32 checks   = 0;

static Void check(Bool ok, CharSeq what)
{
    ++checks;
    if(ok)
    {
        printf("  ok    %s\n", what);
    }
    else
    {
        ++failures;
        printf("  FAIL  %s\n", what);
    }
}

/* ---- inspecting ---------------------------------------------------------- */

static Void testInspect(Void)
{
    printf("\ninspecting\n");

    check(bibo::text::len("PING") == 4, "bibo::text::len counts");
    check(bibo::text::len("") == 0, "bibo::text::len of empty");
    check(bibo::text::len(nullptr) == 0, "bibo::text::len of nullptr does not crash");

    check(bibo::text::empty(""), "empty is empty");
    check(bibo::text::empty(nullptr), "nullptr is empty");
    check(!bibo::text::empty("x"), "one character is not empty");

    check(bibo::text::eq("STEER", "STEER"), "equal strings are equal");
    check(!bibo::text::eq("STEER", "STEEP"), "one letter apart is not equal");
    check(!bibo::text::eq("STEER", "STEE"), "a prefix is not equal");
    check(bibo::text::eq(nullptr, nullptr), "two NULLs are equal");
    check(!bibo::text::eq("x", nullptr), "a string is not nullptr");

    check(bibo::text::starts("STEER 0.5", "STEER "), "prefix matches");
    check(!bibo::text::starts("STEER", "STEER "), "prefix longer than the string");
    check(bibo::text::starts("STEER", ""), "the empty prefix always matches");
    check(!bibo::text::starts(nullptr, "X"), "nullptr starts with nothing");

    check(bibo::text::eq(bibo::text::after("STEER 0.5", "STEER "), "0.5"),
          "bibo::text::after returns the argument");
    check(bibo::text::after("STOP", "STEER ") == nullptr,
          "bibo::text::after refuses a non-match");
}

/* ---- editing ------------------------------------------------------------- */

static Void testEdit(Void)
{
    printf("\nediting in place\n");

    Utf8 a[32];
    strcpy(a, "PING\r");
    check(bibo::text::trimEnd(a) == 4 && bibo::text::eq(a, "PING"), "trims a carriage return");

    strcpy(a, "PING \t\r\n");
    check(bibo::text::trimEnd(a) == 4 && bibo::text::eq(a, "PING"), "trims a whole tail");

    strcpy(a, "PING");
    check(bibo::text::trimEnd(a) == 4, "leaves a clean line alone");

    strcpy(a, "   ");
    check(bibo::text::trimEnd(a) == 0, "all whitespace trims to nothing");

    strcpy(a, "servo on");
    bibo::text::upper(a);
    check(bibo::text::eq(a, "SERVO ON"), "upper folds a whole line");

    strcpy(a, "STEER -0.5");
    bibo::text::upper(a);
    check(bibo::text::eq(a, "STEER -0.5"), "upper leaves digits and signs alone");
}

/* ---- integers ------------------------------------------------------------ */

static Void testInt(Void)
{
    printf("\nintegers\n");

    Int32 v = -999;

    check(bibo::text::toInt("1500", &v) && v == 1500, "a plain number");
    check(bibo::text::toInt("-250", &v) && v == -250, "a negative");
    check(bibo::text::toInt("  42  ", &v) && v == 42, "surrounding space is allowed");

    /* THE case this whole file exists for. */
    v = -999;
    check(bibo::text::toInt("0", &v) && v == 0, "ZERO parses, and is not an error");

    v = -999;
    check(!bibo::text::toInt("banana", &v), "a word is refused");
    check(v == -999, "  and the caller's variable is untouched");

    check(!bibo::text::toInt("12abc", &v), "trailing rubbish is refused (atoi said 12)");
    check(!bibo::text::toInt("", &v), "empty is refused");
    check(!bibo::text::toInt(nullptr, &v), "nullptr is refused");
    check(!bibo::text::toInt("1.5", &v), "a fraction is not an integer");
    check(!bibo::text::toInt("- 5", &v), "a detached sign is refused");
}

/* ---- fractions ----------------------------------------------------------- */

static Bool near(Float32 a, Float32 b)
{
    const Float32 d = (a > b) ? (a - b) : (b - a);
    return d < 0.0001f;
}

static Void testFloat(Void)
{
    printf("\nfractions\n");

    Float32 f = -99.0f;

    check(bibo::text::toFloat("1", &f) && near(f, 1.0f), "a whole number is a fraction");
    check(bibo::text::toFloat("-1", &f) && near(f, -1.0f), "full lock one way");
    check(bibo::text::toFloat("0.5", &f) && near(f, 0.5f), "a half");
    check(bibo::text::toFloat("-0.25", &f) && near(f, -0.25f), "a negative quarter");
    check(bibo::text::toFloat(".25", &f) && near(f, 0.25f), "a leading dot");

    f = -99.0f;
    check(bibo::text::toFloat("0", &f) && near(f, 0.0f), "ZERO parses - straight ahead");

    f = -99.0f;
    check(!bibo::text::toFloat("banana", &f), "a word is refused");
    check(near(f, -99.0f), "  and the caller's variable is untouched");

    check(!bibo::text::toFloat("0.5x", &f), "trailing rubbish is refused");
    check(!bibo::text::toFloat("", &f), "empty is refused");
    check(!bibo::text::toFloat(nullptr, &f), "nullptr is refused");
}

/* ---- pairs --------------------------------------------------------------- */

static Void testTwoInts(Void)
{
    printf("\npairs\n");

    Int32 lo = 0;
    Int32 hi = 0;

    check(bibo::text::twoInts("1230 1670", &lo, &hi) && lo == 1230 && hi == 1670,
          "two numbers");
    check(bibo::text::twoInts("1230    1670", &lo, &hi) && lo == 1230 && hi == 1670,
          "any amount of space between");
    check(bibo::text::twoInts("-10 10", &lo, &hi) && lo == -10 && hi == 10,
          "negatives");

    /* sscanf("%d %d") accepts this and stops looking. */
    check(!bibo::text::twoInts("1 2 3", &lo, &hi), "a third number is refused");
    check(!bibo::text::twoInts("1 2 banana", &lo, &hi), "trailing rubbish is refused");

    check(!bibo::text::twoInts("1500", &lo, &hi), "one number is not a pair");
    check(!bibo::text::twoInts("", &lo, &hi), "empty is refused");
    check(!bibo::text::twoInts("banana boat", &lo, &hi), "two words are refused");
}


/*
 * bibo::text::word - whole-word matching, which is what the command table stands on.
 *
 * The SERVO/SERVOTRIM pair is the case that matters: bibo::text::starts() answers TRUE
 * for both, so a table matched with it would answer SERVOTRIM with the SERVO
 * handler unless the rows happened to be in the right order. These checks are
 * the reason the order of that table can be anything.
 */
static Void testWord(Void)
{
    printf("\n-- bibo::text::word --\n");

    check(bibo::text::word("PING", "PING") != nullptr, "a bare command matches");
    check(bibo::text::eq(bibo::text::word("PING", "PING"), ""), "and its argument is empty");

    check(bibo::text::eq(bibo::text::word("LED ON", "LED"), "ON"), "the argument follows");
    check(bibo::text::eq(bibo::text::word("LED   ON", "LED"), "ON"), "extra spaces are skipped");
    check(bibo::text::eq(bibo::text::word("SLEW 8", "SLEW"), "8"), "a number argument");

    /* The whole point. */
    check(bibo::text::word("SERVOTRIM 1500", "SERVO") == nullptr,
          "SERVO does not match SERVOTRIM");
    check(bibo::text::word("SERVOLIMITS 1 2", "SERVO") == nullptr,
          "SERVO does not match SERVOLIMITS");
    check(bibo::text::eq(bibo::text::word("SERVOTRIM 1500", "SERVOTRIM"), "1500"),
          "SERVOTRIM matches itself");
    check(bibo::text::eq(bibo::text::word("SERVO 1500", "SERVO"), "1500"),
          "SERVO still matches SERVO");
    check(bibo::text::word("ESCLIMITS 1 2", "ESC") == nullptr,
          "ESC does not match ESCLIMITS");

    check(bibo::text::word("PIN", "PING") == nullptr, "a truncated command does not match");
    check(bibo::text::word("", "PING") == nullptr, "empty matches nothing");
    check(bibo::text::word("PINGING", "PING") == nullptr, "a longer word does not match");

    check(bibo::text::word(nullptr, "PING") == nullptr, "nullptr input is refused");
    check(bibo::text::word("PING", nullptr) == nullptr, "nullptr word is refused");

    /* TOF's subcommand goes through the same function a second time. */
    check(bibo::text::eq(bibo::text::word(bibo::text::word("TOF MODE LONG", "TOF"), "MODE"), "LONG"),
          "a subcommand nests");
}

Int32 main(Void)
{
    printf("=== lib/text.h ===\n");

    testInspect();
    testEdit();
    testInt();
    testFloat();
    testTwoInts();
    testWord();

    printf("\n%d checks, %d failed\n", checks, failures);
    printf("%s\n", (failures == 0) ? "PASS" : "FAIL");
    return (failures == 0) ? 0 : 1;
}
