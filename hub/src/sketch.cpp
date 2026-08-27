#include "sketch.hpp"

#include "pico_flash.hpp"
#include "settings.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>

#include <algorithm>
#include <cstdio>

namespace sketch {

Str dir()
{
    const Str base = settings::dir();
    if(base.empty())
        return Str();

    const Str d = base + "\\sketches";
    if(!::CreateDirectoryA(d.c_str(), nullptr)
       && ::GetLastError() != ERROR_ALREADY_EXISTS)
        return Str();

    return d;
}

Vec<Str> list()
{
    Vec<Str> out;

    const Str d = dir();
    if(d.empty())
        return out;

    WIN32_FIND_DATAA fd = {};
    const Str        pattern = d + "\\*.c";
    HANDLE           h = ::FindFirstFileA(pattern.c_str(), &fd);
    if(h == INVALID_HANDLE_VALUE)
        return out;

    do
    {
        if((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
            out.push_back(fd.cFileName);
    }
    while(::FindNextFileA(h, &fd) != 0);
    ::FindClose(h);

    // Alphabetical, unlike recordings. These names are chosen by a person
    // rather than stamped with a time, so "newest first" would shuffle the list
    // every time one is saved and nobody would find anything twice.
    std::sort(out.begin(), out.end());
    return out;
}

Str pathOf(const Str& name)
{
    const Str d = dir();
    if(d.empty() || name.empty())
        return Str();
    return d + "\\" + name;
}

Str slotPath()
{
    const Str root = PicoFlash::repoRoot();
    if(root.empty())
        return Str();
    return root + "\\firmware\\src\\sketch.c";
}

Str firmwareDir()
{
    const Str root = PicoFlash::repoRoot();
    if(root.empty())
        return Str();
    return root + "\\firmware\\src";
}

Vec<Str> listFirmware()
{
    Vec<Str> out;

    const Str d = firmwareDir();
    if(d.empty())
        return out;

    const Char* const PATTERNS[] = { "\\*.c", "\\*.h" };
    for(const Char* pat : PATTERNS)
    {
        WIN32_FIND_DATAA fd = {};
        HANDLE           h = ::FindFirstFileA((d + pat).c_str(), &fd);
        if(h == INVALID_HANDLE_VALUE)
            continue;
        do
        {
            if((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
                out.push_back(fd.cFileName);
        }
        while(::FindNextFileA(h, &fd) != 0);
        ::FindClose(h);
    }

    std::sort(out.begin(), out.end());
    return out;
}

Str targetFor(const Str& path)
{
    if(path.empty())
        return "sketch";

    // The scratch slot itself.
    const Str slot = slotPath();
    if(!slot.empty() && _stricmp(path.c_str(), slot.c_str()) == 0)
        return "sketch";

    // Anything in the sketch library.
    const Str lib = dir();
    if(!lib.empty() && path.size() > lib.size()
       && _strnicmp(path.c_str(), lib.c_str(), lib.size()) == 0)
        return "sketch";

    // Everything else under firmware/src belongs to the debug image. shared.h
    // is compiled into both, and naming pico_debug for it is the safe answer:
    // it is the target a person editing a header is almost certainly testing.
    return "pico_debug";
}

Str load(const Str& path)
{
    if(path.empty())
        return Str();

    FILE* f = std::fopen(path.c_str(), "rb");
    if(f == nullptr)
        return Str();

    Str  out;
    Char buf[4096];
    Size n = 0;
    while((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
        out.append(buf, n);
    std::fclose(f);

    // CRLF in, LF held internally. The editor works in LF and writes LF back;
    // MSVC, gcc and every other tool in this chain read LF fine on Windows.
    Str lf;
    lf.reserve(out.size());
    for(Size i = 0; i < out.size(); ++i)
        if(out[i] != '\r')
            lf.push_back(out[i]);

    return lf;
}

Bool save(const Str& path, const Str& text, Str& err)
{
    err.clear();

    if(path.empty())
    {
        err = "no path (is %LOCALAPPDATA% set?)";
        return false;
    }

    FILE* f = std::fopen(path.c_str(), "wb");
    if(f == nullptr)
    {
        err = "cannot open " + path + " for writing";
        return false;
    }

    const Size want = text.size();
    const Size got  = want > 0 ? std::fwrite(text.data(), 1, want, f) : 0;
    const Bool ok   = (got == want);

    if(std::fclose(f) != 0 || !ok)
    {
        err = "write to " + path + " failed";
        return false;
    }
    return true;
}

UInt64 stamp(const Str& path)
{
    if(path.empty())
    {
        return 0;
    }

    WIN32_FILE_ATTRIBUTE_DATA fad = {};
    if(!::GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &fad))
    {
        return 0;
    }
    return (static_cast<UInt64>(fad.ftLastWriteTime.dwHighDateTime) << 32)
         | static_cast<UInt64>(fad.ftLastWriteTime.dwLowDateTime);
}

Bool remove(const Str& path)
{
    if(path.empty())
    {
        return false;
    }
    return ::DeleteFileA(path.c_str()) != 0;
}

Void reveal(const Str& path)
{
    if(path.empty())
    {
        return;
    }
    // /select, highlights the file itself rather than merely opening the folder,
    // which is what somebody right-clicking a file is asking for.
    const Str arg = "/select,\"" + path + "\"";
    ::ShellExecuteA(nullptr, "open", "explorer.exe", arg.c_str(), nullptr,
                    SW_SHOWNORMAL);
}

Str starter()
{
    // Kept in step with firmware/src/sketch.c by hand, and deliberately so: that
    // file has to compile from a clean clone with no hub involved, and this one
    // has to exist when the repo is not where the exe expects it.
    //
    // manbox C style throughout - shared.h aliases, Allman braces, `while(...)`
    // with no space, SCREAMING_SNAKE macros. A starter program is the strongest
    // style document a project has, because it is the one everybody copies.
    return
        "/*\n"
        " * Blink an LED on a breadboard.\n"
        " *\n"
        " * WIRING - GP28 is physical pin 34, ground is physical pin 38:\n"
        " *\n"
        " *   GP28 (34) --[ 220R-1k ]-- LED long leg\n"
        " *                             LED short leg -- GND (38)\n"
        " *\n"
        " * The resistor can go on either side of the LED. Without one the LED\n"
        " * is a short across a 3.3 V pin and both are at risk.\n"
        " *\n"
        " * GP28 is free. Do not blink GP0/GP1 (servo, ESC), GP4/GP5 (I2C),\n"
        " * GP10-13 (ToF), GP15 (encoder) or GP16-19 (SD) once the car is wired.\n"
        " *\n"
        " * pico2w.h is the SDK in this project's own naming. Everything it\n"
        " * offers is in the completion list: type gpio, servo, adc or serial.\n"
        " */\n"
        "\n"
        "#include \"pico2w.h\"\n"
        "\n"
        "#define LED_PIN 28\n"
        "#define DELAY_MS 400\n"
        "\n"
        "Int32 main(Void)\n"
        "{\n"
        "    // FIRST, and in every sketch you write, even one that prints\n"
        "    // nothing. This starts the USB stack; without it the board runs\n"
        "    // fine and never enumerates, and the only way to flash it again is\n"
        "    // holding BOOTSEL while plugging the cable in.\n"
        "    serialOpen();\n"
        "\n"
        "    gpioOpen(LED_PIN, PIN_DIR_OUT);\n"
        "\n"
        "    // The onboard LED is on the wireless chip rather than a GPIO, so it\n"
        "    // has to be brought up first - and that can fail. If it does, the\n"
        "    // breadboard LED still blinks.\n"
        "    const Bool haveOnboard = ledOpen();\n"
        "\n"
        "    while(true)\n"
        "    {\n"
        "        gpioWrite(LED_PIN, true);\n"
        "        if(haveOnboard)\n"
        "        {\n"
        "            ledWrite(false);\n"
        "        }\n"
        "        sleepMs(DELAY_MS);\n"
        "\n"
        "        gpioWrite(LED_PIN, false);\n"
        "        if(haveOnboard)\n"
        "        {\n"
        "            ledWrite(true);\n"
        "        }\n"
        "        sleepMs(DELAY_MS);\n"
        "    }\n"
        "\n"
        "    return 0;\n"
        "}\n";
}

Str makeName()
{
    const Vec<Str> have = list();

    auto taken = [&have](const Str& n)
    {
        for(const Str& h : have)
            if(_stricmp(h.c_str(), n.c_str()) == 0)
                return true;
        return false;
    };

    if(!taken("sketch.c"))
        return "sketch.c";

    for(Int32 i = 2; i < 1000; ++i)
    {
        Char buf[32];
        std::snprintf(buf, sizeof(buf), "sketch-%d.c", i);
        if(!taken(buf))
            return Str(buf);
    }
    return "sketch.c";
}

} // namespace sketch
