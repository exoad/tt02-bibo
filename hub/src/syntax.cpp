#include "syntax.hpp"

#include <algorithm>
#include <cstring>

namespace syn {
namespace {

Bool identStart(Char c) noexcept
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

Bool identChar(Char c) noexcept
{
    return identStart(c) || (c >= '0' && c <= '9');
}

Bool digit(Char c) noexcept
{
    return c >= '0' && c <= '9';
}

// Control flow, storage and the rest of the language's own words. Sorted only
// for readability; the lookup is linear because the list is short and a line is
// short, and a hash map here would be slower after the setup cost.
const Char* const KEYWORDS[] = {
    "alignas", "alignof", "asm", "auto", "break", "case", "catch", "class",
    "const", "consteval", "constexpr", "constinit", "const_cast", "continue",
    "decltype", "default", "delete", "do", "dynamic_cast", "else", "enum",
    "explicit", "export", "extern", "false", "final", "for", "friend", "goto",
    "if", "inline", "mutable", "namespace", "new", "noexcept", "nullptr",
    "operator", "override", "private", "protected", "public", "register",
    "reinterpret_cast", "restrict", "return", "sizeof", "static",
    "static_assert", "static_cast", "struct", "switch", "template", "this",
    "throw", "true", "try", "typedef", "typeid", "typename", "union", "using",
    "virtual", "volatile", "while",
};

// Types get their own colour because in embedded C they carry most of the
// meaning of a declaration.
//
// The FIRST block is the manbox vocabulary (github.com/exoad/manbox) - the
// aliases in firmware/src/shared.h and hub/src/shared.hpp. Those are the names
// this project actually declares things with, so they are the names that have to
// light up; a style guide that its own editor does not recognise is a style
// guide nobody follows. The rest are the underlying spellings and the Pico SDK's
// own, which still appear at every third-party boundary.
const Char* const TYPES[] = {
    // manbox aliases, C and C++ alike
    "Any", "Bool", "CFile", "Char", "CharSeq", "CharSeq16", "CharSeq32",
    "Float32", "Float64", "Fn", "ISize", "Int16", "Int32", "Int64", "Int8",
    "Opt", "SharedPtr", "Size", "Str", "StrView", "UInt16", "UInt32", "UInt64",
    "UInt8", "UPtr", "UniqPtr", "Utf16", "Utf32", "Utf8", "Variant", "Void",
    "WeakPtr",

    // what they alias to, plus the SDK's own spellings
    "absolute_time_t", "bool", "char", "double", "float", "int", "int16_t",
    "int32_t", "int64_t", "int8_t", "intptr_t", "long", "ptrdiff_t", "short",
    "signed", "size_t", "uint", "uint16_t", "uint32_t", "uint64_t", "uint8_t",
    "uintptr_t", "unsigned", "void",
};

Bool inList(const Char* const* list, Size n, const Char* s, Int32 len)
{
    for(Size i = 0; i < n; ++i)
    {
        const Char* k = list[i];
        if(std::strncmp(k, s, static_cast<Size>(len)) == 0 && k[len] == '\0')
            return true;
    }
    return false;
}

Void push(Vec<Span>& out, Int32 b, Int32 e, Role r)
{
    if(e <= b)
        return;

    // Merge with the previous span when the role matches. Without this a line of
    // plain text becomes one span per character and the renderer does a draw
    // call for each.
    if(!out.empty() && out.back().role == r && out.back().end == b)
    {
        out.back().end = e;
        return;
    }
    Span s;
    s.begin = b;
    s.end   = e;
    s.role  = r;
    out.push_back(s);
}

} // namespace

Void tokenize(const Str& line, Bool& inBlock, Vec<Span>& out)
{
    out.clear();

    const Int32 n = static_cast<Int32>(line.size());
    const Char* p = line.c_str();
    Int32       i = 0;

    // ---- continuation of a /* ... */ that opened on an earlier line ---------
    if(inBlock)
    {
        Int32 j = 0;
        while(j + 1 < n && !(p[j] == '*' && p[j + 1] == '/'))
            ++j;

        if(j + 1 < n)          // closes here
        {
            push(out, 0, j + 2, Role::ROLE_COMMENT);
            inBlock = false;
            i = j + 2;
        }
        else                   // the whole line is still inside the comment
        {
            push(out, 0, n, Role::ROLE_COMMENT);
            return;
        }
    }

    // ---- a preprocessor line is coloured whole -----------------------------
    // Checked after the block-comment carry so that a `#define` sitting inside
    // an open /* */ stays a comment, which is what the compiler thinks too.
    {
        Int32 k = i;
        while(k < n && (p[k] == ' ' || p[k] == '\t'))
            ++k;
        if(k < n && p[k] == '#')
        {
            push(out, i, k, Role::ROLE_TEXT);

            // ... except for a string or angle-bracket header, which keeps the
            // string colour. `#include "pico/stdlib.h"` reading as one flat run
            // loses the only part of the line anybody scans for.
            Int32 q = k;
            while(q < n && p[q] != '"' && p[q] != '<')
                ++q;
            push(out, k, q, Role::ROLE_PREPROC);
            if(q < n)
            {
                const Char close = (p[q] == '<') ? '>' : '"';
                Int32 e = q + 1;
                while(e < n && p[e] != close)
                    ++e;
                if(e < n)
                    ++e;
                push(out, q, e, Role::ROLE_STRING);
                push(out, e, n, Role::ROLE_PREPROC);
            }
            return;
        }
    }

    while(i < n)
    {
        const Char c = p[i];

        // ---- comments ------------------------------------------------------
        if(c == '/' && i + 1 < n && p[i + 1] == '/')
        {
            push(out, i, n, Role::ROLE_COMMENT);
            return;
        }
        if(c == '/' && i + 1 < n && p[i + 1] == '*')
        {
            Int32 j = i + 2;
            while(j + 1 < n && !(p[j] == '*' && p[j + 1] == '/'))
                ++j;
            if(j + 1 < n)
            {
                push(out, i, j + 2, Role::ROLE_COMMENT);
                i = j + 2;
            }
            else
            {
                push(out, i, n, Role::ROLE_COMMENT);
                inBlock = true;      // carries to the next line
                return;
            }
            continue;
        }

        // ---- strings and character literals ---------------------------------
        if(c == '"' || c == '\'')
        {
            Int32 j = i + 1;
            while(j < n)
            {
                // An escape consumes the next character, so \" and \\ do not
                // close the literal.
                if(p[j] == '\\')
                {
                    j += 2;
                    continue;
                }
                if(p[j] == c)
                {
                    ++j;
                    break;
                }
                ++j;
            }
            push(out, i, std::min(j, n), Role::ROLE_STRING);
            i = std::min(j, n);
            continue;
        }

        // ---- numbers --------------------------------------------------------
        // A leading digit is enough. `.5` is deliberately not handled: treating
        // '.' as a number start would miscolour every `a.b` member access, which
        // is far more common in this codebase than a bare fractional literal.
        if(digit(c))
        {
            Int32 j = i;
            while(j < n && (identChar(p[j]) || p[j] == '.'))
                ++j;   // swallows 0x1F, 1e-3, 100u, 1.5f in one bite
            push(out, i, j, Role::ROLE_NUMBER);
            i = j;
            continue;
        }

        // ---- identifiers ----------------------------------------------------
        if(identStart(c))
        {
            Int32 j = i;
            while(j < n && identChar(p[j]))
                ++j;

            const Int32 len = j - i;
            Role        r   = Role::ROLE_TEXT;

            if(inList(KEYWORDS, sizeof(KEYWORDS) / sizeof(KEYWORDS[0]), p + i, len))
                r = Role::ROLE_KEYWORD;
            else if(inList(TYPES, sizeof(TYPES) / sizeof(TYPES[0]), p + i, len))
                r = Role::ROLE_TYPE;
            else
            {
                // A call or a definition: identifier, optional spaces, '('.
                Int32 k = j;
                while(k < n && p[k] == ' ')
                    ++k;
                if(k < n && p[k] == '(')
                    r = Role::ROLE_FUNCTION;
            }

            push(out, i, j, r);
            i = j;
            continue;
        }

        // ---- punctuation and everything else ---------------------------------
        if(std::strchr("{}()[];,.<>+-*/%=!&|^~?:", c) != nullptr)
        {
            push(out, i, i + 1, Role::ROLE_PUNCT);
            ++i;
            continue;
        }

        push(out, i, i + 1, Role::ROLE_TEXT);
        ++i;
    }
}

ImU32 colorFor(Role r) noexcept
{
    switch(r)
    {
    case Role::ROLE_KEYWORD:  return gruv::RED;
    case Role::ROLE_TYPE:     return gruv::YELLOW;
    case Role::ROLE_NUMBER:   return gruv::PURPLE;
    case Role::ROLE_STRING:   return gruv::GREEN;
    case Role::ROLE_COMMENT:  return gruv::GRAY;
    case Role::ROLE_PREPROC:  return gruv::AQUA;
    case Role::ROLE_FUNCTION: return gruv::GREEN;
    case Role::ROLE_PUNCT:    return gruv::FG1;
    case Role::ROLE_TEXT:
    case Role::ROLE_COUNT:
    default:                  return gruv::FG1;
    }
}

} // namespace syn
