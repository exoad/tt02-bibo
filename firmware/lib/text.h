/* ---------------------------------------------------------------------------
 * text - the string handling this project actually does, spelled our way.
 *
 * Not a string library. There is no allocation here, no growth, no ownership -
 * a microcontroller parsing a command line needs about eight operations and
 * every one of them works on a caller's buffer.
 *
 * WHY THIS EXISTS
 *
 * Two reasons, and the second is the important one.
 *
 * 1. <string.h> is snake_case with C's vocabulary, and hal.h already draws the
 *    line that says the SDK's spelling stops at the library edge. strncmp(line,
 *    "STEER ", 6) sitting in a file where everything else is camelCase is the
 *    same leak, and it comes with a hand-counted 6 that is wrong the moment
 *    somebody renames the command.
 *
 * 2. atoi() CANNOT FAIL. It returns 0 for "0", for "banana", and for "". Every
 *    caller in this tree therefore tested `if(us == 0)` and called that an
 *    error - which works only because no legitimate pulse width is zero. That
 *    is a bug wearing a disguise: the first command that legitimately accepts 0
 *    inherits a parser that rejects it. textInt() returns Bool and writes
 *    through a pointer, so "0" and "not a number" are different answers.
 *
 * ---- one copy -------------------------------------------------------------
 *
 * Stateless, unlike chassis.h and status.h. Include it anywhere.
 * ------------------------------------------------------------------------- */
#pragma once

#include "types.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

/* ---- inspecting ---------------------------------------------------------- */

static Size textLen(CharSeq s)
{
    return (s == NULL) ? 0 : strlen(s);
}

static Bool textEmpty(CharSeq s)
{
    return (s == NULL) || (s[0] == '\0');
}

/* Whole-string equality. The name says what it tests, unlike `strcmp(a,b)==0`
 * where the interesting part is the `== 0` and reads as an accident. */
static inline Bool textEq(CharSeq a, CharSeq b)
{
    if(a == NULL || b == NULL)
    {
        return a == b;
    }
    return strcmp(a, b) == 0;
}

/*
 * Does `s` begin with `prefix`.
 *
 * The length comes from the prefix rather than from the caller, which is the
 * whole point: strncmp(line, "SERVOLIMITS ", 12) carries a hand-counted 12 that
 * silently stops matching the day the command is renamed, and the failure is a
 * command that quietly does nothing.
 */
static Bool textStarts(CharSeq s, CharSeq prefix)
{
    if(s == NULL || prefix == NULL)
    {
        return false;
    }
    const Size n = strlen(prefix);
    return strncmp(s, prefix, n) == 0;
}

/* What follows `prefix`, or NULL if `s` does not start with it. Pairs with
 * textStarts so the offset is never written out by hand twice. */
static CharSeq textAfter(CharSeq s, CharSeq prefix)
{
    if(!textStarts(s, prefix))
    {
        return NULL;
    }
    return s + strlen(prefix);
}

/*
 * Matches `word` as a WHOLE word at the start of `s`, and returns whatever
 * follows it with the separating spaces skipped. NULL if it does not match.
 *
 * The difference from textStarts() is the whole word, and it is the difference
 * between a command table that works and one that works by accident:
 * textStarts("SERVOTRIM 1500", "SERVO") is TRUE, so a table matched with it
 * answers SERVOTRIM with the SERVO handler unless SERVOTRIM happens to be
 * listed first. Requiring a space or the end of the string after the word means
 * the order of the rows carries no meaning at all, which is the property that
 * makes a table safe to add to.
 *
 * A command with no argument returns a pointer to the empty string at the end
 * of `s`, NOT NULL. "matched, nothing after it" and "did not match" are
 * different answers and a dispatcher has to tell them apart.
 */
static CharSeq textWord(CharSeq s, CharSeq word)
{
    if(s == NULL || word == NULL)
    {
        return NULL;
    }

    const Size n = strlen(word);
    if(strncmp(s, word, n) != 0)
    {
        return NULL;
    }
    if(s[n] != '\0' && s[n] != ' ')
    {
        return NULL;
    }

    CharSeq arg = s + n;
    while(*arg == ' ')
    {
        ++arg;
    }
    return arg;
}

/* ---- editing in place ---------------------------------------------------- */

/*
 * Strips trailing CR, space and tab. Returns the new length.
 *
 * A terminal decides on its own what to put at the end of a line, and the three
 * it might choose are exactly these. Without this, "PING\r" is not "PING" and
 * the reply is "unknown command" for a command that was typed correctly.
 */
static Size textTrimEnd(Utf8* s)
{
    if(s == NULL)
    {
        return 0;
    }
    Size n = strlen(s);
    while(n > 0 && (s[n - 1] == '\r' || s[n - 1] == '\n'
                    || s[n - 1] == ' ' || s[n - 1] == '\t'))
    {
        s[--n] = '\0';
    }
    return n;
}

static inline Void textUpper(Utf8* s)
{
    if(s == NULL)
    {
        return;
    }
    for(Size i = 0; s[i] != '\0'; ++i)
    {
        s[i] = (Utf8) toupper((Int32) (UInt8) s[i]);
    }
}

/* ---- parsing ------------------------------------------------------------- */

/*
 * A whole integer, or false.
 *
 * STRICT on purpose: leading and trailing space are allowed, anything else is a
 * refusal. atoi("12abc") is 12 and atoi("abc") is 0, and a console that accepts
 * "SERVO 12abc" as 12 is a console that will one day accept something worse.
 */
static inline Bool textInt(CharSeq s, Int32* out)
{
    if(textEmpty(s) || out == NULL)
    {
        return false;
    }

    Utf8* end = NULL;
    const Int64 v = strtol(s, &end, 10);

    if(end == s)
    {
        return false;          /* nothing numeric at all */
    }
    while(*end == ' ' || *end == '\t')
    {
        ++end;
    }
    if(*end != '\0')
    {
        return false;          /* trailing rubbish */
    }

    *out = (Int32) v;
    return true;
}

/* The same contract for a fraction. Accepts "1", "-0.5", ".25". */
static Bool textFloat(CharSeq s, Float32* out)
{
    if(textEmpty(s) || out == NULL)
    {
        return false;
    }

    Utf8* end = NULL;
    const Float64 v = strtod(s, &end);

    if(end == s)
    {
        return false;
    }
    while(*end == ' ' || *end == '\t')
    {
        ++end;
    }
    if(*end != '\0')
    {
        return false;
    }

    *out = (Float32) v;
    return true;
}

/*
 * Two integers separated by whitespace, or false.
 *
 * sscanf(arg, "%d %d", &a, &b) != 2 does this and also silently accepts
 * "1 2 3 banana", because sscanf stops looking the moment it has what it was
 * asked for. Every argument being consumed is part of the contract.
 */
static Bool textTwoInts(CharSeq s, Int32* a, Int32* b)
{
    if(textEmpty(s) || a == NULL || b == NULL)
    {
        return false;
    }

    Utf8* end = NULL;
    const Int64 first = strtol(s, &end, 10);
    if(end == s)
    {
        return false;
    }

    CharSeq rest = end;
    while(*rest == ' ' || *rest == '\t')
    {
        ++rest;
    }
    if(*rest == '\0')
    {
        return false;          /* only one number */
    }

    Int32 second = 0;
    if(!textInt(rest, &second))
    {
        return false;
    }

    *a = (Int32) first;
    *b = second;
    return true;
}
