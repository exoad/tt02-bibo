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
 *   * textStarts must not match a prefix longer than the string
 *
 * Compiled for the HOST, not the board: text.h needs nothing from the Pico SDK,
 * only types.h. That is a property worth keeping - a parser that can only be
 * exercised by flashing a microcontroller is a parser nobody exercises.
 *
 * Exits 0 on PASS, 1 on FAIL.
 */

#include "text.h"

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

    check(textLen("PING") == 4, "textLen counts");
    check(textLen("") == 0, "textLen of empty");
    check(textLen(NULL) == 0, "textLen of NULL does not crash");

    check(textEmpty(""), "empty is empty");
    check(textEmpty(NULL), "NULL is empty");
    check(!textEmpty("x"), "one character is not empty");

    check(textEq("STEER", "STEER"), "equal strings are equal");
    check(!textEq("STEER", "STEEP"), "one letter apart is not equal");
    check(!textEq("STEER", "STEE"), "a prefix is not equal");
    check(textEq(NULL, NULL), "two NULLs are equal");
    check(!textEq("x", NULL), "a string is not NULL");

    check(textStarts("STEER 0.5", "STEER "), "prefix matches");
    check(!textStarts("STEER", "STEER "), "prefix longer than the string");
    check(textStarts("STEER", ""), "the empty prefix always matches");
    check(!textStarts(NULL, "X"), "NULL starts with nothing");

    check(textEq(textAfter("STEER 0.5", "STEER "), "0.5"),
          "textAfter returns the argument");
    check(textAfter("STOP", "STEER ") == NULL,
          "textAfter refuses a non-match");
}

/* ---- editing ------------------------------------------------------------- */

static Void testEdit(Void)
{
    printf("\nediting in place\n");

    Utf8 a[32];
    strcpy(a, "PING\r");
    check(textTrimEnd(a) == 4 && textEq(a, "PING"), "trims a carriage return");

    strcpy(a, "PING \t\r\n");
    check(textTrimEnd(a) == 4 && textEq(a, "PING"), "trims a whole tail");

    strcpy(a, "PING");
    check(textTrimEnd(a) == 4, "leaves a clean line alone");

    strcpy(a, "   ");
    check(textTrimEnd(a) == 0, "all whitespace trims to nothing");

    strcpy(a, "servo on");
    textUpper(a);
    check(textEq(a, "SERVO ON"), "upper folds a whole line");

    strcpy(a, "STEER -0.5");
    textUpper(a);
    check(textEq(a, "STEER -0.5"), "upper leaves digits and signs alone");
}

/* ---- integers ------------------------------------------------------------ */

static Void testInt(Void)
{
    printf("\nintegers\n");

    Int32 v = -999;

    check(textInt("1500", &v) && v == 1500, "a plain number");
    check(textInt("-250", &v) && v == -250, "a negative");
    check(textInt("  42  ", &v) && v == 42, "surrounding space is allowed");

    /* THE case this whole file exists for. */
    v = -999;
    check(textInt("0", &v) && v == 0, "ZERO parses, and is not an error");

    v = -999;
    check(!textInt("banana", &v), "a word is refused");
    check(v == -999, "  and the caller's variable is untouched");

    check(!textInt("12abc", &v), "trailing rubbish is refused (atoi said 12)");
    check(!textInt("", &v), "empty is refused");
    check(!textInt(NULL, &v), "NULL is refused");
    check(!textInt("1.5", &v), "a fraction is not an integer");
    check(!textInt("- 5", &v), "a detached sign is refused");
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

    check(textFloat("1", &f) && near(f, 1.0f), "a whole number is a fraction");
    check(textFloat("-1", &f) && near(f, -1.0f), "full lock one way");
    check(textFloat("0.5", &f) && near(f, 0.5f), "a half");
    check(textFloat("-0.25", &f) && near(f, -0.25f), "a negative quarter");
    check(textFloat(".25", &f) && near(f, 0.25f), "a leading dot");

    f = -99.0f;
    check(textFloat("0", &f) && near(f, 0.0f), "ZERO parses - straight ahead");

    f = -99.0f;
    check(!textFloat("banana", &f), "a word is refused");
    check(near(f, -99.0f), "  and the caller's variable is untouched");

    check(!textFloat("0.5x", &f), "trailing rubbish is refused");
    check(!textFloat("", &f), "empty is refused");
    check(!textFloat(NULL, &f), "NULL is refused");
}

/* ---- pairs --------------------------------------------------------------- */

static Void testTwoInts(Void)
{
    printf("\npairs\n");

    Int32 lo = 0;
    Int32 hi = 0;

    check(textTwoInts("1230 1670", &lo, &hi) && lo == 1230 && hi == 1670,
          "two numbers");
    check(textTwoInts("1230    1670", &lo, &hi) && lo == 1230 && hi == 1670,
          "any amount of space between");
    check(textTwoInts("-10 10", &lo, &hi) && lo == -10 && hi == 10,
          "negatives");

    /* sscanf("%d %d") accepts this and stops looking. */
    check(!textTwoInts("1 2 3", &lo, &hi), "a third number is refused");
    check(!textTwoInts("1 2 banana", &lo, &hi), "trailing rubbish is refused");

    check(!textTwoInts("1500", &lo, &hi), "one number is not a pair");
    check(!textTwoInts("", &lo, &hi), "empty is refused");
    check(!textTwoInts("banana boat", &lo, &hi), "two words are refused");
}

Int32 main(Void)
{
    printf("=== lib/text.h ===\n");

    testInspect();
    testEdit();
    testInt();
    testFloat();
    testTwoInts();

    printf("\n%d checks, %d failed\n", checks, failures);
    printf("%s\n", (failures == 0) ? "PASS" : "FAIL");
    return (failures == 0) ? 0 : 1;
}
