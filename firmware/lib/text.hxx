/*
 * ---------------------------------------------------------------------------
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
 * -------------------------------------------------------------------------
 */
#pragma once

#include "types.hxx"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace bibo::text
{

    /* ---- inspecting ---------------------------------------------------------- */

    /**
     * @brief The length of a string, in bytes.
     *
     * @param s the string to measure
     * @return the number of bytes before the terminating null, or 0
     *
     * @note Returns 0 for a null `s` rather than dereferencing it, so a
     *       caller need not check for null before asking how long something is.
     */
    inline Size len(const CharSeq s)
    {
        return s == nullptr ? 0 : strlen(s);
    }

    /**
     * @brief Whether a string has no characters.
     *
     * @param s the string to test
     * @return true when `s` is null or its first byte is the terminator
     */
    inline Bool empty(const CharSeq s)
    {
        return s == nullptr || s[0] == '\0';
    }

    /**
     * @brief Whole-string equality.
     *
     * The name says what it tests, unlike `strcmp(a,b)==0` where the
     * interesting part is the `== 0` and reads as an accident.
     *
     * @param a one string to compare
     * @param b the other string to compare
     * @return true when both are null, or both are non-null and byte-identical
     */
    inline Bool eq(const CharSeq a, const CharSeq b)
    {
        if(a == nullptr || b == nullptr)
        {
            return a == b;
        }
        return strcmp(a, b) == 0;
    }

    /**
     * @brief Whether `s` begins with `prefix`.
     *
     * The length comes from the prefix rather than from the caller, which is
     * the whole point: strncmp(line, "SERVOLIMITS ", 12) carries a
     * hand-counted 12 that silently stops matching the day the command is
     * renamed, and the failure is a command that quietly does nothing.
     *
     * @param s the string to test
     * @param prefix the prefix to look for
     * @return true when `s` starts with every byte of `prefix`
     *
     * @note Returns false, rather than crashing, when either argument is null.
     */
    inline Bool starts(const CharSeq s, const CharSeq prefix)
    {
        if(s == nullptr || prefix == nullptr)
        {
            return false;
        }
        const Size n = strlen(prefix);
        return strncmp(s, prefix, n) == 0;
    }

    /**
     * @brief What follows `prefix` in `s`.
     *
     * Pairs with text::starts so the offset is never written out by hand twice.
     *
     * @param s the string to search
     * @param prefix the prefix expected at the start of `s`
     * @return a pointer into `s` just past `prefix`, or nullptr if `s` does
     *         not start with it
     *
     * @note The returned pointer aliases `s` rather than copying it, and is
     *       only valid as long as `s` is.
     */
    inline CharSeq after(const CharSeq s, const CharSeq prefix)
    {
        if(!starts(s, prefix))
        {
            return nullptr;
        }
        return s + strlen(prefix);
    }

    /**
     * @brief Matches `word` as a WHOLE word at the start of `s`.
     *
     * The difference from text::starts() is the whole word, and it is the
     * difference between a command table that works and one that works by
     * accident: text::starts("SERVOTRIM 1500", "SERVO") is TRUE, so a table
     * matched with it answers SERVOTRIM with the SERVO handler unless
     * SERVOTRIM happens to be listed first. Requiring a space or the end of
     * the string after the word means the order of the rows carries no
     * meaning at all, which is the property that makes a table safe to add to.
     *
     * @param s the string to match against
     * @param word the whole word to look for at the start of `s`
     * @return a pointer to whatever follows `word`, with the separating
     *         spaces skipped, or nullptr if `s` does not start with `word`
     *         as a whole word
     *
     * @note A command with no argument returns a pointer to the empty string
     *       at the end of `s`, NOT nullptr. "matched, nothing after it" and
     *       "did not match" are different answers and a dispatcher has to
     *       tell them apart.
     */
    inline CharSeq word(const CharSeq s, const CharSeq word)
    {
        if(s == nullptr || word == nullptr)
        {
            return nullptr;
        }

        const Size n = strlen(word);
        if(strncmp(s, word, n) != 0)
        {
            return nullptr;
        }
        if(s[n] != '\0' && s[n] != ' ')
        {
            return nullptr;
        }

        CharSeq arg = s + n;
        while(*arg == ' ')
        {
            ++arg;
        }
        return arg;
    }

    /* ---- editing in place ---------------------------------------------------- */

    /**
     * @brief Strips trailing CR, LF, space and tab from `s`, in place.
     *
     * A terminal decides on its own what to put at the end of a line, and the
     * four it might choose are exactly these. Without this, "PING\r" is not
     * "PING" and the reply is "unknown command" for a command that was typed
     * correctly.
     *
     * @param s the buffer to trim; trailing bytes are overwritten with the
     *          terminator
     * @return the new length of `s`, or 0 if `s` is null
     *
     * @note Mutates `s` in place; nothing is reallocated or copied.
     */
    inline Size trimEnd(Utf8* s)
    {
        if(s == nullptr)
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

    /**
     * @brief Uppercases a string, in place.
     *
     * @param s the buffer to uppercase; each byte is rewritten in place
     *
     * @note Does nothing if `s` is null.
     */
    inline Void upper(Utf8* s)
    {
        if(s == nullptr)
        {
            return;
        }
        for(Size i = 0; s[i] != '\0'; ++i)
        {
            // Through UInt8 first, deliberately: toupper takes an int whose value
            // must be representable as unsigned char, and a plain char is SIGNED on
            // this toolchain - so a byte over 0x7F would arrive negative and the
            // behavior would be undefined.
            s[i] = static_cast<Utf8>(
                toupper(static_cast<UInt8>(s[i])));
        }
    }

    /* ---- parsing ------------------------------------------------------------- */

    /**
     * @brief Parses `s` as a whole integer.
     *
     * STRICT on purpose: leading and trailing space are allowed, anything
     * else is a refusal. atoi("12abc") is 12 and atoi("abc") is 0, and a
     * console that accepts "SERVO 12abc" as 12 is a console that will one
     * day accept something worse.
     *
     * @param s the text to parse
     * @param out where the parsed value is written; untouched on failure
     * @return true when all of `s`, aside from surrounding space, was
     *         consumed as one integer
     *
     * @note Returns false without writing to `out` if `s` is empty/null or
     *       `out` is null.
     */
    inline Bool toInt(const CharSeq s, Int32* out)
    {
        if(empty(s) || out == nullptr)
        {
            return false;
        }

        Utf8* end = nullptr;
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

    /**
     * @brief Parses `s` as a fraction, under the same contract as toInt().
     *
     * Accepts "1", "-0.5", ".25".
     *
     * @param s the text to parse
     * @param out where the parsed value is written; untouched on failure
     * @return true when all of `s`, aside from surrounding space, was
     *         consumed as one number
     *
     * @note Returns false without writing to `out` if `s` is empty/null or
     *       `out` is null.
     */
    inline Bool toFloat(const CharSeq s, Float32* out)
    {
        if(empty(s) || out == nullptr)
        {
            return false;
        }

        Utf8* end = nullptr;
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

    /**
     * @brief Parses `s` as two integers separated by whitespace.
     *
     * sscanf(arg, "%d %d", &a, &b) != 2 does this and also silently accepts
     * "1 2 3 banana", because sscanf stops looking the moment it has what it
     * was asked for. Every argument being consumed is part of the contract.
     *
     * @param s the text to parse
     * @param a where the first integer is written; untouched on failure
     * @param b where the second integer is written; untouched on failure
     * @return true when all of `s` was consumed as exactly two integers
     *
     * @note Returns false without writing to `a` or `b` if `s` is empty/null
     *       or either output pointer is null.
     */
    inline Bool twoInts(const CharSeq s, Int32* a, Int32* b)
    {
        if(empty(s) || a == nullptr || b == nullptr)
        {
            return false;
        }

        Utf8* end = nullptr;
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


    /**
     * @brief Bounded formatted write into the CALLER's buffer.
     *
     * The wrapper snprintf never had, and the reason app/ was still naming a
     * libc function directly: serial::printf formats and SENDS, which is the
     * wrong shape for a caller assembling a string it means to keep. There
     * was nothing here to call instead, so two call sites reached past the
     * library - and the audit could not see them, because its lookbehind is
     * defeated by the leading `s` in snprintf.
     *
     * Deliberately a PASSTHROUGH: it returns exactly what snprintf returns -
     * the length the output WANTED, which is how a caller detects
     * truncation. A wrapper that improved on that return would be a second
     * thing to learn, and the point of the seam is that it costs nothing to
     * cross.
     *
     * @param buf the caller's buffer to write into
     * @param cap the size of `buf`, in bytes, including room for the terminator
     * @param fmt a printf-style format string
     * @return the length the formatted output WANTED to be, which may exceed
     *         `cap` - the same convention as snprintf
     *
     * @note If the wanted length is >= `cap`, the output was truncated;
     *       `buf` is still null-terminated within `cap` bytes.
     */
    inline Int32 format(Utf8* buf, const Size cap, const CharSeq fmt, ...)
    {
        va_list ap;
        va_start(ap, fmt);
        const Int32 n = vsnprintf(buf, cap, fmt, ap);
        va_end(ap);
        return n;
    }

}
