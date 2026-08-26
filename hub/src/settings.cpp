#include "settings.hpp"

#include <windows.h>

#include <cstdio>

namespace settings {

Str dir()
{
    static Str cached;
    static Bool tried = false;

    if(tried)
        return cached;
    tried = true;

    Char buf[MAX_PATH] = {};
    if(::GetEnvironmentVariableA("LOCALAPPDATA", buf, MAX_PATH) == 0)
        return cached;                  // no profile: run without persistence

    cached = Str(buf) + "\\tt02-auto";

    // ERROR_ALREADY_EXISTS is the expected case on every run but the first.
    if(!::CreateDirectoryA(cached.c_str(), nullptr)
       && ::GetLastError() != ERROR_ALREADY_EXISTS)
        cached.clear();

    return cached;
}

Str path(const Char* name)
{
    const Str d = dir();
    return d.empty() ? Str() : (d + "\\" + name);
}

Str read(const Char* name)
{
    Str out;

    const Str p = path(name);
    if(p.empty())
        return out;

    FILE* f = std::fopen(p.c_str(), "rb");
    if(f == nullptr)
        return out;

    Char buf[1024];
    for(;;)
    {
        const Size n = std::fread(buf, 1, sizeof(buf), f);
        if(n == 0)
            break;
        out.append(buf, n);

        // A settings file is a few hundred bytes. Anything larger is a file
        // that does not belong here, and reading it all would be the bug.
        if(out.size() > 64u * 1024u)
            break;
    }
    std::fclose(f);
    return out;
}

Void write(const Char* name, const Str& text)
{
    const Str p = path(name);
    if(p.empty())
        return;

    FILE* f = std::fopen(p.c_str(), "wb");
    if(f == nullptr)
        return;

    if(!text.empty())
        static_cast<Void>(std::fwrite(text.data(), 1, text.size(), f));
    std::fclose(f);
}

} // namespace settings
