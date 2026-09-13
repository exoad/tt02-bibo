/*
 * text - the string handling this project does, spelled our way, on a caller's
 * buffer: no allocation, no ownership, no state. The parsers return Bool because
 * atoi() cannot fail - it returns 0 for "0", for "banana" and for "" alike.
 */
#pragma once

#include "shared.hxx"
#include <ctype.h>
#include <float.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace bibo::text
{
    /** 0 for a null s. */
    inline Size len(const CharSeq s)
    {
        return s == nullptr ? 0 : strlen(s);
    }

    inline Bool empty(const CharSeq s)
    {
        return s == nullptr || s[0] == '\0';
    }

    inline Bool eq(const CharSeq a, const CharSeq b)
    {
        if(a == nullptr || b == nullptr)
        {
            return a == b;
        }
        return strcmp(a, b) == 0;
    }

    /**
     * The length comes from the prefix: a hand-counted strncmp length silently
     * stops matching the day a command is renamed.
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

    /** A pointer into s just past prefix, or nullptr when s does not start with it. */
    inline CharSeq after(const CharSeq s, const CharSeq prefix)
    {
        if(!starts(s, prefix))
        {
            return nullptr;
        }
        return s + strlen(prefix);
    }

    /**
     * Matches word as a WHOLE word at the start of s, and returns what follows it
     * with the spaces skipped, or nullptr. starts("SERVOTRIM 1500", "SERVO") is
     * true, so a command table matched with starts() depends on its row order;
     * with this it does not.
     *
     * A bare command returns the empty string at the end of s, NOT nullptr:
     * "matched, nothing after it" and "did not match" must stay apart.
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

    /**
     * Strips trailing CR, LF, space and tab in place, returning the new length: a
     * terminal picks its own line ending, and "PING\r" is not "PING".
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

    inline Void upper(Utf8* s)
    {
        if(s == nullptr)
        {
            return;
        }
        for(Size i = 0; s[i] != '\0'; ++i)
        {
            // Through UInt8 first: toupper's argument must be representable as
            // unsigned char, and a plain char is SIGNED on this toolchain.
            s[i] = static_cast<Utf8>(
                toupper(static_cast<UInt8>(s[i])));
        }
    }

    /**
     * STRICT: surrounding space is allowed and anything else is a refusal, where
     * atoi("12abc") is 12. out is untouched on failure.
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
        *out = static_cast<Int32>(v);
        return true;
    }

    /**
     * toInt()'s contract for a fraction such as "-0.5" or ".25". Refuses NAN, INF
     * and anything a Float32 cannot hold: strtod accepts those words, and
     * "STEER NAN" would otherwise reach an Int32 cast.
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
        /* Written so NaN fails too: every comparison with NaN is false. */
        if(!(v >= -FLT_MAX && v <= FLT_MAX))
        {
            return false;
        }
        *out = static_cast<Float32>(v);
        return true;
    }

    /**
     * Exactly two integers, a and b untouched on failure; sscanf's "%d %d" would
     * also accept "1 2 3 banana".
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
            return false;
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
     * Bounded formatted write into the caller's buffer, for a string it means to
     * keep (serial::printf formats and SENDS). A deliberate passthrough: it
     * returns the length the output WANTED, so a result >= cap means truncated.
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
