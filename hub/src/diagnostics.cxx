#include "diagnostics.hxx"

#include <cctype>
#include <cstdlib>
#include <cstring>

namespace diag {
namespace {

Bool digit(Char c) noexcept
{
    return c >= '0' && c <= '9';
}

// Case-insensitive match of `word` at `at`.
Bool wordAt(const Str& s, Size at, const Char* word) noexcept
{
    const Size n = std::strlen(word);
    if(at + n > s.size())
    {
        return false;
    }
    for(Size i = 0; i < n; ++i)
    {
        const Char a = static_cast<Char>(std::tolower(
            static_cast<unsigned char>(s[at + i])));
        if(a != word[i])
        {
            return false;
        }
    }
    return true;
}

Bool severityAt(const Str& s, Size at, Severity& out, Size& len) noexcept
{
    if(wordAt(s, at, "error"))
    {
        out = Severity::SEVERITY_ERR;
        len = 5;
        return true;
    }
    if(wordAt(s, at, "warning"))
    {
        out = Severity::SEVERITY_WARN;
        len = 7;
        return true;
    }
    if(wordAt(s, at, "note"))
    {
        out = Severity::SEVERITY_NOTE;
        len = 4;
        return true;
    }
    return false;
}

// The last path separator, so a full path reduces to its file name.
Str baseName(const Str& p)
{
    const Size a = p.find_last_of('/');
    const Size b = p.find_last_of('\\');

    Size at = Str::npos;
    if(a != Str::npos && b != Str::npos)
    {
        at = (a > b) ? a : b;
    }
    else if(a != Str::npos)
    {
        at = a;
    }
    else if(b != Str::npos)
    {
        at = b;
    }

    return (at == Str::npos) ? p : p.substr(at + 1);
}

Bool equalsNoCase(const Str& a, const Str& b) noexcept
{
    if(a.size() != b.size())
    {
        return false;
    }
    for(Size i = 0; i < a.size(); ++i)
    {
        if(std::tolower(static_cast<unsigned char>(a[i]))
           != std::tolower(static_cast<unsigned char>(b[i])))
        {
            return false;
        }
    }
    return true;
}

} // namespace

Bool parseLine(const Str& text, Item& out)
{
    // ---- MSVC: path(LINE,COL): error C2065: message ------------------------
    // Tried first because its shape is unambiguous, and because a path with a
    // parenthesis in it is not a thing that happens in this tree.
    {
        const Size open = text.find('(');
        if(open != Str::npos && open > 0)
        {
            Size i = open + 1;
            Int32 line = 0;
            Bool any = false;
            while(i < text.size() && digit(text[i]))
            {
                line = line * 10 + (text[i] - '0');
                ++i;
                any = true;
            }

            Int32 col = 0;
            if(any && i < text.size() && text[i] == ',')
            {
                ++i;
                while(i < text.size() && digit(text[i]))
                {
                    col = col * 10 + (text[i] - '0');
                    ++i;
                }
            }

            if(any && i + 1 < text.size() && text[i] == ')' && text[i + 1] == ':')
            {
                Size j = i + 2;
                while(j < text.size() && text[j] == ' ')
                {
                    ++j;
                }

                Severity sev = Severity::SEVERITY_ERR;
                Size     len = 0;
                if(severityAt(text, j, sev, len))
                {
                    out.file     = text.substr(0, open);
                    out.line     = line;
                    out.column   = col;
                    out.severity = sev;

                    Size m = text.find(':', j + len);
                    m = (m == Str::npos) ? (j + len) : (m + 1);
                    while(m < text.size() && text[m] == ' ')
                    {
                        ++m;
                    }
                    out.message = text.substr(m);
                    return true;
                }
            }
        }
    }

    // ---- gcc / clang: path:LINE[:COL]: severity: message -------------------
    //
    // Scanned from the LEFT for a colon followed by digits, which is what makes
    // a Windows drive letter safe: "C:/x.c:42:1: error: y" has its first colon
    // followed by '/', so the scan walks past it.
    for(Size i = 1; i + 1 < text.size(); ++i)
    {
        if(text[i] != ':' || !digit(text[i + 1]))
        {
            continue;
        }

        Size  j    = i + 1;
        Int32 line = 0;
        while(j < text.size() && digit(text[j]))
        {
            line = line * 10 + (text[j] - '0');
            ++j;
        }

        Int32 col = 0;
        if(j < text.size() && text[j] == ':' && j + 1 < text.size()
           && digit(text[j + 1]))
        {
            ++j;
            while(j < text.size() && digit(text[j]))
            {
                col = col * 10 + (text[j] - '0');
                ++j;
            }
        }

        if(j >= text.size() || text[j] != ':')
        {
            continue;   // "10:30" in a timestamp, not a location
        }

        Size k = j + 1;
        while(k < text.size() && text[k] == ' ')
        {
            ++k;
        }

        Severity sev = Severity::SEVERITY_ERR;
        Size     len = 0;
        if(!severityAt(text, k, sev, len))
        {
            continue;
        }

        Size m = k + len;
        while(m < text.size() && (text[m] == ':' || text[m] == ' '))
        {
            ++m;
        }

        out.file     = text.substr(0, i);
        out.line     = line;
        out.column   = col;
        out.severity = sev;
        out.message  = text.substr(m);
        return true;
    }

    return false;
}

Vec<Item> parseAll(const Vec<Str>& lines)
{
    Vec<Item> out;
    for(const Str& ln : lines)
    {
        Item it;
        if(parseLine(ln, it))
        {
            out.push_back(it);
        }
    }
    return out;
}

Vec<Item> forFile(const Vec<Item>& all, const Str& path)
{
    Vec<Item>  out;
    const Str  want = baseName(path);

    if(want.empty())
    {
        return out;
    }

    for(const Item& it : all)
    {
        if(equalsNoCase(baseName(it.file), want))
        {
            out.push_back(it);
        }
    }
    return out;
}

Int32 worstOnLine(const Vec<Item>& items, Int32 line)
{
    Int32 worst = -1;
    for(const Item& it : items)
    {
        if(it.line != line)
        {
            continue;
        }
        const Int32 s = static_cast<Int32>(it.severity);
        if(s > worst)
        {
            worst = s;
        }
    }
    return worst;
}

} // namespace diag
