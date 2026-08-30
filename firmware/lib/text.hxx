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
 *    inherits a parser that rejects it. text::toInt() returns Bool and writes
 *    through a pointer, so "0" and "not a number" are different answers.
 *
 * ---- one copy -------------------------------------------------------------
 *
 * Stateless, unlike chassis.h and status.h. Include it anywhere.
 * ------------------------------------------------------------------------- */
#pragma once

#include "types.hxx"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace bibo
{

  namespace text
  {

    /* ---- inspecting ---------------------------------------------------------- */

    static Size len(CharSeq s)
    {
        return (s == NULL) ? 0 : strlen(s);
    }

    static Bool empty(CharSeq s)
    {
        return (s == NULL) || (s[0] == '\0');
    }

    /* Whole-string equality. The name says what it tests, unlike `strcmp(a,b)==0`
     * where the interesting part is the `== 0` and reads as an accident. */
    static Bool eq(CharSeq a, CharSeq b)
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
    static Bool starts(CharSeq s, CharSeq prefix)
    {
        if(s == NULL || prefix == NULL)
        {
            return false;
        }
        const Size n = strlen(prefix);
        return strncmp(s, prefix, n) == 0;
    }

    /* What follows `prefix`, or NULL if `s` does not start with it. Pairs with
     * text::starts so the offset is never written out by hand twice. */
    static CharSeq after(CharSeq s, CharSeq prefix)
    {
        if(!starts(s, prefix))
        {
            return NULL;
        }
        return s + strlen(prefix);
    }

    /*
     * Matches `word` as a WHOLE word at the start of `s`, and returns whatever
     * follows it with the separating spaces skipped. NULL if it does not match.
     *
     * The difference from text::starts() is the whole word, and it is the difference
     * between a command table that works and one that works by accident:
     * text::starts("SERVOTRIM 1500", "SERVO") is TRUE, so a table matched with it
     * answers SERVOTRIM with the SERVO handler unless SERVOTRIM happens to be
     * listed first. Requiring a space or the end of the string after the word means
     * the order of the rows carries no meaning at all, which is the property that
     * makes a table safe to add to.
     *
     * A command with no argument returns a pointer to the empty string at the end
     * of `s`, NOT NULL. "matched, nothing after it" and "did not match" are
     * different answers and a dispatcher has to tell them apart.
     */
    static CharSeq word(CharSeq s, CharSeq word)
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
    static Size trimEnd(Utf8* s)
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

    static Void upper(Utf8* s)
    {
        if(s == NULL)
        {
            return;
        }
        for(Size i = 0; s[i] != '\0'; ++i)
        {
            // Through UInt8 first, deliberately: toupper takes an int whose value
            // must be representable as unsigned char, and a plain char is SIGNED on
            // this toolchain - so a byte over 0x7F would arrive negative and the
            // behaviour would be undefined.
            s[i] = static_cast<Utf8>(
                toupper(static_cast<Int32>(static_cast<UInt8>(s[i]))));
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
    static Bool toInt(CharSeq s, Int32* out)
    {
        if(empty(s) || out == NULL)
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

        *out = static_cast<Int32>(v);
        return true;
    }

    /* The same contract for a fraction. Accepts "1", "-0.5", ".25". */
    static Bool toFloat(CharSeq s, Float32* out)
    {
        if(empty(s) || out == NULL)
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

        *out = static_cast<Float32>(v);
        return true;
    }

    /*
     * Two integers separated by whitespace, or false.
     *
     * sscanf(arg, "%d %d", &a, &b) != 2 does this and also silently accepts
     * "1 2 3 banana", because sscanf stops looking the moment it has what it was
     * asked for. Every argument being consumed is part of the contract.
     */
    static Bool twoInts(CharSeq s, Int32* a, Int32* b)
    {
        if(empty(s) || a == NULL || b == NULL)
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
        if(!toInt(rest, &second))
        {
            return false;
        }

        *a = static_cast<Int32>(first);
        *b = second;
        return true;
    }


    /*
     * Bounded formatted write into the CALLER's buffer.
     *
     * The wrapper snprintf never had, and the reason app/ was still naming a libc
     * function directly: serial::printf formats and SENDS, which is the wrong
     * shape for a caller assembling a string it means to keep. There was nothing
     * here to call instead, so two call sites reached past the library - and the
     * audit could not see them, because its lookbehind is defeated by the leading
     * `s` in snprintf.
     *
     * Deliberately a PASSTHROUGH: it returns exactly what snprintf returns - the
     * length the output WANTED, which is how a caller detects truncation. A
     * wrapper that improved on that return would be a second thing to learn, and
     * the point of the seam is that it costs nothing to cross.
     */
    static Int32 format(Utf8* buf, Size cap, CharSeq fmt, ...)
    {
        va_list ap;
        va_start(ap, fmt);
        const Int32 n = vsnprintf(buf, cap, fmt, ap);
        va_end(ap);
        return n;
    }

  } // namespace text

} // namespace bibo