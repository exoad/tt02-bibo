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
 *   * text::starts must not match a prefix longer than the string
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

    check(text::len("PING") == 4, "text::len counts");
    check(text::len("") == 0, "text::len of empty");
    check(text::len(NULL) == 0, "text::len of NULL does not crash");

    check(text::empty(""), "empty is empty");
    check(text::empty(NULL), "NULL is empty");
    check(!text::empty("x"), "one character is not empty");

    check(text::eq("STEER", "STEER"), "equal strings are equal");
    check(!text::eq("STEER", "STEEP"), "one letter apart is not equal");
    check(!text::eq("STEER", "STEE"), "a prefix is not equal");
    check(text::eq(NULL, NULL), "two NULLs are equal");
    check(!text::eq("x", NULL), "a string is not NULL");

    check(text::starts("STEER 0.5", "STEER "), "prefix matches");
    check(!text::starts("STEER", "STEER "), "prefix longer than the string");
    check(text::starts("STEER", ""), "the empty prefix always matches");
    check(!text::starts(NULL, "X"), "NULL starts with nothing");

    check(text::eq(text::after("STEER 0.5", "STEER "), "0.5"),
          "text::after returns the argument");
    check(text::after("STOP", "STEER ") == NULL,
          "text::after refuses a non-match");
}

/* ---- editing ------------------------------------------------------------- */

static Void testEdit(Void)
{
    printf("\nediting in place\n");

    Utf8 a[32];
    strcpy(a, "PING\r");
    check(text::trimEnd(a) == 4 && text::eq(a, "PING"), "trims a carriage return");

    strcpy(a, "PING \t\r\n");
    check(text::trimEnd(a) == 4 && text::eq(a, "PING"), "trims a whole tail");

    strcpy(a, "PING");
    check(text::trimEnd(a) == 4, "leaves a clean line alone");

    strcpy(a, "   ");
    check(text::trimEnd(a) == 0, "all whitespace trims to nothing");

    strcpy(a, "servo on");
    text::upper(a);
    check(text::eq(a, "SERVO ON"), "upper folds a whole line");

    strcpy(a, "STEER -0.5");
    text::upper(a);
    check(text::eq(a, "STEER -0.5"), "upper leaves digits and signs alone");
}

/* ---- integers ------------------------------------------------------------ */

static Void testInt(Void)
{
    printf("\nintegers\n");

    Int32 v = -999;

    check(text::toInt("1500", &v) && v == 1500, "a plain number");
    check(text::toInt("-250", &v) && v == -250, "a negative");
    check(text::toInt("  42  ", &v) && v == 42, "surrounding space is allowed");

    /* THE case this whole file exists for. */
    v = -999;
    check(text::toInt("0", &v) && v == 0, "ZERO parses, and is not an error");

    v = -999;
    check(!text::toInt("banana", &v), "a word is refused");
    check(v == -999, "  and the caller's variable is untouched");

    check(!text::toInt("12abc", &v), "trailing rubbish is refused (atoi said 12)");
    check(!text::toInt("", &v), "empty is refused");
    check(!text::toInt(NULL, &v), "NULL is refused");
    check(!text::toInt("1.5", &v), "a fraction is not an integer");
    check(!text::toInt("- 5", &v), "a detached sign is refused");
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

    check(text::toFloat("1", &f) && near(f, 1.0f), "a whole number is a fraction");
    check(text::toFloat("-1", &f) && near(f, -1.0f), "full lock one way");
    check(text::toFloat("0.5", &f) && near(f, 0.5f), "a half");
    check(text::toFloat("-0.25", &f) && near(f, -0.25f), "a negative quarter");
    check(text::toFloat(".25", &f) && near(f, 0.25f), "a leading dot");

    f = -99.0f;
    check(text::toFloat("0", &f) && near(f, 0.0f), "ZERO parses - straight ahead");

    f = -99.0f;
    check(!text::toFloat("banana", &f), "a word is refused");
    check(near(f, -99.0f), "  and the caller's variable is untouched");

    check(!text::toFloat("0.5x", &f), "trailing rubbish is refused");
    check(!text::toFloat("", &f), "empty is refused");
    check(!text::toFloat(NULL, &f), "NULL is refused");
}

/* ---- pairs --------------------------------------------------------------- */

static Void testTwoInts(Void)
{
    printf("\npairs\n");

    Int32 lo = 0;
    Int32 hi = 0;

    check(text::twoInts("1230 1670", &lo, &hi) && lo == 1230 && hi == 1670,
          "two numbers");
    check(text::twoInts("1230    1670", &lo, &hi) && lo == 1230 && hi == 1670,
          "any amount of space between");
    check(text::twoInts("-10 10", &lo, &hi) && lo == -10 && hi == 10,
          "negatives");

    /* sscanf("%d %d") accepts this and stops looking. */
    check(!text::twoInts("1 2 3", &lo, &hi), "a third number is refused");
    check(!text::twoInts("1 2 banana", &lo, &hi), "trailing rubbish is refused");

    check(!text::twoInts("1500", &lo, &hi), "one number is not a pair");
    check(!text::twoInts("", &lo, &hi), "empty is refused");
    check(!text::twoInts("banana boat", &lo, &hi), "two words are refused");
}


/*
 * text::word - whole-word matching, which is what the command table stands on.
 *
 * The SERVO/SERVOTRIM pair is the case that matters: text::starts() answers TRUE
 * for both, so a table matched with it would answer SERVOTRIM with the SERVO
 * handler unless the rows happened to be in the right order. These checks are
 * the reason the order of that table can be anything.
 */
static Void testWord(Void)
{
    printf("\n-- text::word --\n");

    check(text::word("PING", "PING") != NULL, "a bare command matches");
    check(text::eq(text::word("PING", "PING"), ""), "and its argument is empty");

    check(text::eq(text::word("LED ON", "LED"), "ON"), "the argument follows");
    check(text::eq(text::word("LED   ON", "LED"), "ON"), "extra spaces are skipped");
    check(text::eq(text::word("SLEW 8", "SLEW"), "8"), "a number argument");

    /* The whole point. */
    check(text::word("SERVOTRIM 1500", "SERVO") == NULL,
          "SERVO does not match SERVOTRIM");
    check(text::word("SERVOLIMITS 1 2", "SERVO") == NULL,
          "SERVO does not match SERVOLIMITS");
    check(text::eq(text::word("SERVOTRIM 1500", "SERVOTRIM"), "1500"),
          "SERVOTRIM matches itself");
    check(text::eq(text::word("SERVO 1500", "SERVO"), "1500"),
          "SERVO still matches SERVO");
    check(text::word("ESCLIMITS 1 2", "ESC") == NULL,
          "ESC does not match ESCLIMITS");

    check(text::word("PIN", "PING") == NULL, "a truncated command does not match");
    check(text::word("", "PING") == NULL, "empty matches nothing");
    check(text::word("PINGING", "PING") == NULL, "a longer word does not match");

    check(text::word(NULL, "PING") == NULL, "NULL input is refused");
    check(text::word("PING", NULL) == NULL, "NULL word is refused");

    /* TOF's subcommand goes through the same function a second time. */
    check(text::eq(text::word(text::word("TOF MODE LONG", "TOF"), "MODE"), "LONG"),
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
