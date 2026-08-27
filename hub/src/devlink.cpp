// See devlink.hpp.

#include "shared.hpp"
#include "devlink.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstring>

/* The registry walk in portKind() needs advapi32. Declared here rather than in
 * the build scripts so a test that links devlink.cpp on its own gets it too. */
#pragma comment(lib, "advapi32.lib")

namespace dev {
namespace {

// Only codes that mean the device is not there. Each one is here because it has
// been seen from a real unplug, not because it sounded plausible.
constexpr UInt32 REMOVAL_CODES[] = {
    ERROR_FILE_NOT_FOUND,        //    2  the port no longer exists
    ERROR_PATH_NOT_FOUND,        //    3
    ERROR_INVALID_HANDLE,        //    6  the handle died under us
    ERROR_BAD_UNIT,              //   20
    ERROR_BAD_COMMAND,           //   22  classic mid-read USB serial removal
    ERROR_DEV_NOT_EXIST,         //   55
    ERROR_DEVICE_NOT_CONNECTED,  // 1167
    ERROR_NO_SUCH_DEVICE,        //  433
};

// Case-insensitive equality for port names, because the enumeration and the
// settings file do not always agree on capitalisation.
Bool sameName(const Str& a, const Str& b)
{
    if(a.size() != b.size())
    {
        return false;
    }
    for(Size i = 0; i < a.size(); ++i)
    {
        Char x = a[i];
        Char y = b[i];
        if(x >= 'a' && x <= 'z')
        {
            x = static_cast<Char>(x - 'a' + 'A');
        }
        if(y >= 'a' && y <= 'z')
        {
            y = static_cast<Char>(y - 'a' + 'A');
        }
        if(x != y)
        {
            return false;
        }
    }
    return true;
}

} // namespace

Bool isRemovalCode(UInt32 win32Code) noexcept
{
    for(UInt32 c : REMOVAL_CODES)
    {
        if(win32Code == c)
        {
            return true;
        }
    }
    return false;
}

Vec<Str> listPorts()
{
    Vec<Str> ports;

    // QueryDosDeviceA with a null name returns every DOS device as a packed run
    // of NUL-terminated strings. No import library beyond kernel32, which is why
    // it is preferred to SetupAPI for something called at failure time.
    DWORD     cap = 8192;
    Vec<Char> buf;

    for(Int32 attempt = 0; attempt < 6; ++attempt)
    {
        buf.assign(cap, '\0');
        const DWORD n = QueryDosDeviceA(nullptr, buf.data(), cap);
        if(n == 0)
        {
            if(::GetLastError() == ERROR_INSUFFICIENT_BUFFER)
            {
                cap *= 2;
                continue;
            }
            return ports;              // cannot tell; NOT "nothing attached"
        }

        const Char* p   = buf.data();
        const Char* end = buf.data() + n;
        while(p < end && *p != '\0')
        {
            const Size len = std::strlen(p);

            // COM followed entirely by digits, so "COMEDIA" and a bare "COM"
            // are both rejected.
            if(len > 3
               && (p[0] == 'C' || p[0] == 'c')
               && (p[1] == 'O' || p[1] == 'o')
               && (p[2] == 'M' || p[2] == 'm'))
            {
                Bool allDigits = true;
                for(Size i = 3; i < len; ++i)
                {
                    if(p[i] < '0' || p[i] > '9')
                    {
                        allDigits = false;
                        break;
                    }
                }
                if(allDigits)
                {
                    ports.push_back(Str(p, len));
                }
            }
            p += len + 1;
        }
        return ports;
    }
    return ports;
}

Bool portPresent(const Str& port)
{
    if(port.empty())
    {
        return false;
    }

    const Vec<Str> have = listPorts();

    // An empty list means the enumeration failed, not that the machine has no
    // serial ports. Reporting "gone" on that would turn a transient API failure
    // into a message telling somebody to check a cable that is fine.
    if(have.empty())
    {
        return true;
    }

    for(const Str& p : have)
    {
        if(sameName(p, port))
        {
            return true;
        }
    }
    return false;
}

PortKind portKind(const Str& port)
{
    if(port.empty())
    {
        return PortKind::PORT_KIND_UNKNOWN;
    }

    HKEY key = nullptr;
    if(::RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM",
                       0, KEY_READ, &key) != ERROR_SUCCESS)
    {
        return PortKind::PORT_KIND_UNKNOWN;
    }

    PortKind kind = PortKind::PORT_KIND_UNKNOWN;

    for(DWORD i = 0; ; ++i)
    {
        Char  name[512];
        BYTE  data[512];
        DWORD nlen = static_cast<DWORD>(sizeof(name));
        DWORD dlen = static_cast<DWORD>(sizeof(data));
        DWORD type = 0;

        const LONG r = ::RegEnumValueA(key, i, name, &nlen, nullptr, &type,
                                       data, &dlen);
        if(r != ERROR_SUCCESS)
        {
            break;
        }
        if(type != REG_SZ || dlen == 0)
        {
            continue;
        }

        // Registry strings are not guaranteed to be terminated.
        if(dlen >= sizeof(data))
        {
            dlen = static_cast<DWORD>(sizeof(data)) - 1;
        }
        data[dlen] = 0;

        if(!sameName(reinterpret_cast<const Char*>(data), port))
        {
            continue;
        }

        Str dev(name, nlen);
        for(Char& c : dev)
        {
            if(c >= 'A' && c <= 'Z')
            {
                c = static_cast<Char>(c - 'A' + 'a');
            }
        }

        if(dev.find("silabser") != Str::npos)
        {
            kind = PortKind::PORT_KIND_CP210X;
        }
        else if(dev.find("usbser") != Str::npos)
        {
            kind = PortKind::PORT_KIND_USB_CDC;
        }
        else if(dev.find("bthmodem") != Str::npos)
        {
            kind = PortKind::PORT_KIND_BLUETOOTH;
        }
        break;
    }

    ::RegCloseKey(key);
    return kind;
}

const Char* portKindName(PortKind k)
{
    switch(k)
    {
    case PortKind::PORT_KIND_CP210X:    return "CP210x - RPLIDAR adapter";
    case PortKind::PORT_KIND_USB_CDC:   return "USB serial - Pico";
    case PortKind::PORT_KIND_BLUETOOTH: return "Bluetooth";
    case PortKind::PORT_KIND_UNKNOWN:   break;
    }
    return "unknown";
}

Loss classify(const Str& port, UInt32 win32Code)
{
    // The code is the cheap signal and it is sufficient on its own when it
    // fires - these codes have no other meaning.
    if(win32Code != 0 && isRemovalCode(win32Code))
    {
        return Loss::LOSS_UNPLUGGED;
    }

    // The authoritative one. A device can vanish and leave a generic code
    // behind - ERROR_GEN_FAILURE especially - so a code that is merely unhelpful
    // must not be read as evidence the device is still there.
    if(!portPresent(port))
    {
        return Loss::LOSS_UNPLUGGED;
    }

    return Loss::LOSS_FAULT;
}

Str describe(Loss why, const Str& what, const Str& port)
{
    const Str name = what.empty() ? Str("device") : what;

    if(why == Loss::LOSS_UNPLUGGED)
    {
        // No error code, no Win32 text, no exclamation mark. The cable is out;
        // that is the whole story and the person watching already knows it.
        return port.empty() ? (name + " disconnected")
                            : (name + " disconnected - " + port + " is gone");
    }

    return port.empty() ? (name + " stopped responding")
                        : (name + " stopped responding on " + port);
}

} // namespace dev
