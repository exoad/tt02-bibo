// Win32 implementation of the Pico 2 W debug serial link.
//
// Design notes
// ------------
// * One worker thread owns the HANDLE for the whole life of a connection. It
//   is the only thread that touches the port, so no handle locking is needed.
//   ReadFile is given a short total timeout, so the thread parks inside the
//   driver instead of spinning, and a silent board simply produces a stream of
//   zero-byte reads that cost nothing and are NOT an error.
// * The UI thread only ever touches small mutex-protected queues: send() pushes
//   a string, drain() moves the accumulated lines out. Neither can block on I/O.
// * pico_link.h declares the class with no data members and no pimpl pointer,
//   and it is not ours to edit, so per-object state lives in a file-static side
//   table keyed by `this`. See IMPL_TABLE below.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "shared.hpp"
#include <windows.h>
#include <setupapi.h>

#include "pico_link.hpp"

#include "devlink.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "setupapi.lib")

namespace
{

// --- tunables ---------------------------------------------------------------

constexpr Size MAX_LOG_LINES = 4000;   // bounded backlog; oldest dropped
constexpr Size MAX_TX_QUEUE  = 1024;   // bounded send queue
constexpr Size MAX_LINE_LEN  = 2048;   // longer input lines are truncated
constexpr DWORD  READ_CHUNK   = 4096;
constexpr DWORD  READ_TIMEOUT_MS = 30;   // read wakeup period == send latency
constexpr Int32    DEFAULT_BAUD = 115200;

// Opening a Pico CDC port at 1200 baud and closing it reboots the board into
// BOOTSEL (see bootselTouch). Normal traffic must never do that by accident.
constexpr Int32 BOOTSEL_BAUD = 1200;

// --- time -------------------------------------------------------------------

Int64 nowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

Str winErrText(const Str& what, DWORD code)
{
    Char* msg = nullptr;
    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                       FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr, code, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                   reinterpret_cast<Char*>(&msg), 0, nullptr);

    Str tail;
    if(msg)
    {
        tail = msg;
        LocalFree(msg);
        while(!tail.empty() && (tail.back() == '\n' || tail.back() == '\r' || tail.back() == ' '))
            tail.pop_back();
    }

    Char buf[64];
    _snprintf_s(buf, sizeof(buf), _TRUNCATE, " (error %lu)", static_cast<unsigned long>(code));

    Str out = what;
    if(!tail.empty())
        out += ": " + tail;
    out += buf;
    return out;
}

// --- per-object state -------------------------------------------------------

struct LinkImplBody
{
    // Lifecycle (connect/disconnect/join). Never held while `mu` is held.
    Mutex              lifeMu;
    Thread             worker;
    Atomic<Bool>       stop{false};
    Atomic<Bool>       finished{false};

    // Data shared with the UI thread.
    mutable Mutex      mu;
    Deque<PicoLine>    log;
    Deque<Str> txq;
    Str             err;
    Str             portname;

    Atomic<PicoState>          state{PicoState::PICO_STATE_DISCONNECTED};
    Atomic<UInt64> tx{0};
    Atomic<UInt64> rx{0};
    Atomic<UInt64> drops{0};
    Atomic<Int64>          t0Ns{0};
    Atomic<Float64>             lastRxS{-1.0};   // <0 == nothing ever received

    Float64 elapsedS() const
    {
        return static_cast<Float64>(nowNs() - t0Ns.load(std::memory_order_relaxed)) * 1e-9;
    }

    Void setError(Str e)
    {
        LockGuard<Mutex> lk(mu);
        err = std::move(e);
    }

    Void pushLine(Bool outgoing, Str text)
    {
        PicoLine ln;
        ln.tS      = elapsedS();
        ln.outgoing = outgoing;
        ln.text     = std::move(text);

        LockGuard<Mutex> lk(mu);
        log.push_back(std::move(ln));
        while(log.size() > MAX_LOG_LINES)
            log.pop_front();   // a chatty board loses history, never memory
    }

    Void run(Str port, Int32 baud);
};

// --- port helpers -----------------------------------------------------------

// COM10 and above are only reachable through the "\\.\" device namespace; a
// bare "COM10" resolves to a legacy DOS device name and fails to open.
Str devicePath(const Str& port)
{
    if(port.size() >= 4 && port.compare(0, 4, "\\\\.\\") == 0)
        return port;
    return "\\\\.\\" + port;
}

Bool configurePort(HANDLE h, Int32 baud, Bool assertDtr, Str* err)
{
    DCB dcb;
    ZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if(!GetCommState(h, &dcb))
    {
        if(err) *err = winErrText("GetCommState failed", GetLastError());
        return false;
    }

    dcb.BaudRate         = static_cast<DWORD>(baud);
    dcb.ByteSize         = 8;
    dcb.Parity           = NOPARITY;
    dcb.StopBits         = ONESTOPBIT;
    dcb.fBinary          = TRUE;
    dcb.fParity          = FALSE;
    dcb.fOutxCtsFlow     = FALSE;
    dcb.fOutxDsrFlow     = FALSE;
    dcb.fDsrSensitivity  = FALSE;
    dcb.fTXContinueOnXoff= TRUE;
    dcb.fOutX            = FALSE;
    dcb.fInX             = FALSE;
    dcb.fErrorChar       = FALSE;
    dcb.fNull            = FALSE;
    dcb.fAbortOnError    = FALSE;
    dcb.fDtrControl      = assertDtr ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;
    dcb.fRtsControl      = assertDtr ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE;

    if(!SetCommState(h, &dcb))
    {
        if(err) *err = winErrText("SetCommState failed", GetLastError());
        return false;
    }
    return true;
}

// --- registry / SetupAPI enumeration ---------------------------------------

Int32 comNumber(const Str& s)
{
    Size i = 0;
    while(i < s.size() && (s[i] < '0' || s[i] > '9'))
        ++i;
    Int32 n = 0;
    Bool any = false;
    for(; i < s.size() && s[i] >= '0' && s[i] <= '9'; ++i)
    {
        n = n * 10 + (s[i] - '0');
        any = true;
    }
    return any ? n : 0;
}

// Every COM name Windows currently has mapped. Used to filter out stale
// registry entries for Picos that are no longer plugged in.
HashSet<Str> serialcommPorts()
{
    HashSet<Str> out;

    HKEY key = nullptr;
    if(RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0,
                      KEY_READ, &key) != ERROR_SUCCESS)
        return out;

    for(DWORD i = 0;; ++i)
    {
        Char  name[512];
        BYTE  data[512];
        DWORD nameLen = static_cast<DWORD>(sizeof(name));
        DWORD dataLen = static_cast<DWORD>(sizeof(data));
        DWORD type     = 0;

        LONG rc = RegEnumValueA(key, i, name, &nameLen, nullptr, &type, data, &dataLen);
        if(rc == ERROR_NO_MORE_ITEMS)
            break;
        if(rc != ERROR_SUCCESS)
            break;
        if(type != REG_SZ || dataLen == 0)
            continue;

        data[(std::min)(static_cast<Size>(dataLen), sizeof(data) - 1)] = 0;
        Str port(reinterpret_cast<Char*>(data));
        if(!port.empty())
            out.insert(port);
    }

    RegCloseKey(key);
    return out;
}

Bool readPortName(HKEY parent, const Char* subkey, Str* out)
{
    HKEY k = nullptr;
    if(RegOpenKeyExA(parent, subkey, 0, KEY_READ, &k) != ERROR_SUCCESS)
        return false;

    Char  buf[128];
    DWORD len  = static_cast<DWORD>(sizeof(buf));
    DWORD type = 0;
    LONG  rc   = RegQueryValueExA(k, "PortName", nullptr, &type,
                                  reinterpret_cast<BYTE*>(buf), &len);
    RegCloseKey(k);

    if(rc != ERROR_SUCCESS || type != REG_SZ || len == 0)
        return false;

    buf[(std::min)(static_cast<Size>(len), sizeof(buf) - 1)] = 0;
    *out = buf;
    return !out->empty();
}

// Preferred route: walk HKLM\SYSTEM\CurrentControlSet\Enum\USB looking for
// VID_2E8A*, then read each instance's Device Parameters\PortName. No extra
// libraries, and it copes with composite devices (VID_2E8A&PID_0009&MI_00).
Void enumViaRegistry(Vec<Str>& out)
{
    HKEY usb = nullptr;
    if(RegOpenKeyExA(HKEY_LOCAL_MACHINE, "SYSTEM\\CurrentControlSet\\Enum\\USB", 0,
                      KEY_READ, &usb) != ERROR_SUCCESS)
        return;

    for(DWORD i = 0;; ++i)
    {
        Char  dev[512];
        DWORD devLen = static_cast<DWORD>(sizeof(dev));
        LONG  rc = RegEnumKeyExA(usb, i, dev, &devLen, nullptr, nullptr, nullptr, nullptr);
        if(rc != ERROR_SUCCESS)
            break;
        if(_strnicmp(dev, "VID_2E8A", 8) != 0)
            continue;

        HKEY devk = nullptr;
        if(RegOpenKeyExA(usb, dev, 0, KEY_READ, &devk) != ERROR_SUCCESS)
            continue;

        for(DWORD j = 0;; ++j)
        {
            Char  inst[512];
            DWORD instLen = static_cast<DWORD>(sizeof(inst));
            if(RegEnumKeyExA(devk, j, inst, &instLen, nullptr, nullptr, nullptr, nullptr)
                != ERROR_SUCCESS)
                break;

            Str sub = Str(inst) + "\\Device Parameters";
            Str port;
            if(readPortName(devk, sub.c_str(), &port))
                out.push_back(port);
        }

        RegCloseKey(devk);
    }

    RegCloseKey(usb);
}

// Case-insensitive substring test, ASCII only (no locale surprises).
Bool containsCi(const Char* hay, const Char* needle)
{
    const Size nlen = strlen(needle);
    if(nlen == 0)
        return true;
    for(const Char* p = hay; *p; ++p)
        if(_strnicmp(p, needle, nlen) == 0)
            return true;
    return false;
}

// Fallback if the Enum\USB hive is unreadable: class-enumerate present COM
// ports and match VID_2E8A in the hardware ID. GUID_DEVCLASS_PORTS is spelled
// out here so we need not link uuid.lib.
Void enumViaSetupapi(Vec<Str>& out)
{
    static const GUID PORTS_CLASS = {
        0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};

    HDEVINFO set = SetupDiGetClassDevsA(&PORTS_CLASS, nullptr, nullptr, DIGCF_PRESENT);
    if(set == INVALID_HANDLE_VALUE)
        return;

    SP_DEVINFO_DATA info;
    ZeroMemory(&info, sizeof(info));
    info.cbSize = sizeof(info);

    for(DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &info); ++i)
    {
        Char  hwid[1024] = {0};
        DWORD hwidLen   = 0;
        if(!SetupDiGetDeviceRegistryPropertyA(set, &info, SPDRP_HARDWAREID, nullptr,
                                               reinterpret_cast<BYTE*>(hwid),
                                               sizeof(hwid) - 2, &hwidLen))
            continue;

        // REG_MULTI_SZ: scan every embedded string.
        Bool match = false;
        for(DWORD p = 0; p < hwidLen && hwid[p]; )
        {
            if(containsCi(hwid + p, "VID_2E8A"))
                match = true;
            p += static_cast<DWORD>(strlen(hwid + p)) + 1;
        }
        if(!match)
            continue;

        HKEY k = SetupDiOpenDevRegKey(set, &info, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if(k == INVALID_HANDLE_VALUE)
            continue;

        Char  buf[128];
        DWORD len  = static_cast<DWORD>(sizeof(buf));
        DWORD type = 0;
        if(RegQueryValueExA(k, "PortName", nullptr, &type, reinterpret_cast<BYTE*>(buf), &len)
                == ERROR_SUCCESS &&
            type == REG_SZ && len > 0)
        {
            buf[(std::min)(static_cast<Size>(len), sizeof(buf) - 1)] = 0;
            if(buf[0])
                out.push_back(buf);
        }
        RegCloseKey(k);
    }

    SetupDiDestroyDeviceInfoList(set);
}

}  // namespace

// ---------------------------------------------------------------------------
//  worker thread
// ---------------------------------------------------------------------------

namespace
{

// Keeps only text that is safe to hand to an ImGui text widget. Bytes >= 0x80
// pass through so UTF-8 from the board survives.
inline Bool printable(UInt8 c)
{
    return c >= 0x20 && c != 0x7F;
}

Void LinkImpl_run_trampoline(LinkImplBody* self, Str port, Int32 baud)
{
    self->run(std::move(port), baud);
}

Void LinkImplBody::run(Str port, Int32 baud)
{
    struct DoneFlag
    {
        LinkImplBody* p;
        ~DoneFlag()
        {
            p->finished.store(true, std::memory_order_release);
        }
    } doneFlag{this};

    const Str path = devicePath(port);

    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                           0,            // serial ports are exclusive
                           nullptr, OPEN_EXISTING,
                           0,            // synchronous; this thread may block
                           nullptr);
    if(h == INVALID_HANDLE_VALUE)
    {
        {
            const DWORD     code = ::GetLastError();
            const dev::Loss why  = dev::classify(port, code);
            if(why == dev::Loss::LOSS_UNPLUGGED)
            {
                setError(dev::describe(why, "Pico", port));
                state.store(PicoState::PICO_STATE_UNPLUGGED,
                            std::memory_order_release);
            }
            else
            {
                setError(winErrText("cannot open " + path, code));
                state.store(PicoState::PICO_STATE_ERROR,
                            std::memory_order_release);
            }
        }
        return;
    }

    Str cfgErr;
    if(!configurePort(h, baud, /*assertDtr=*/true, &cfgErr))
    {
        setError(std::move(cfgErr));
        state.store(PicoState::PICO_STATE_ERROR, std::memory_order_release);
        CloseHandle(h);
        return;
    }

    // MAXDWORD interval + MAXDWORD multiplier + a constant is the documented
    // "return as soon as anything is there, else give up after N ms" recipe.
    // A silent board therefore costs ~33 cheap wakeups a second and nothing else.
    COMMTIMEOUTS to;
    ZeroMemory(&to, sizeof(to));
    to.ReadIntervalTimeout         = MAXDWORD;
    to.ReadTotalTimeoutMultiplier  = MAXDWORD;
    to.ReadTotalTimeoutConstant    = READ_TIMEOUT_MS;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = 1000;
    if(!SetCommTimeouts(h, &to))
    {
        setError(winErrText("SetCommTimeouts failed", GetLastError()));
        state.store(PicoState::PICO_STATE_ERROR, std::memory_order_release);
        CloseHandle(h);
        return;
    }

    SetupComm(h, 8192, 8192);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
    EscapeCommFunction(h, SETDTR);   // some firmware waits for host DTR
    EscapeCommFunction(h, SETRTS);

    {
        LockGuard<Mutex> lk(mu);
        portname = port;
        err.clear();
    }
    state.store(PicoState::PICO_STATE_CONNECTED, std::memory_order_release);

    Str accum;          // partial line carried across reads
    Bool        pendingCr = false;
    Bool        overlong   = false;
    Vec<Char> buf(READ_CHUNK);

    auto emit = [&](Str text)
    {
        if(text.empty())
            return;   // blank lines are noise from \r\n\r\n padding
        rx.fetch_add(1, std::memory_order_relaxed);
        lastRxS.store(elapsedS(), std::memory_order_relaxed);
        pushLine(false, std::move(text));
    };

    while(!stop.load(std::memory_order_acquire))
    {
        // ---- transmit anything the UI queued -------------------------------
        Deque<Str> outbound;
        {
            LockGuard<Mutex> lk(mu);
            outbound.swap(txq);
        }
        Bool writeFailed = false;
        for(auto& line : outbound)
        {
            DWORD       written = 0;
            const DWORD n       = static_cast<DWORD>(line.size());
            if(!WriteFile(h, line.data(), n, &written, nullptr) || written != n)
            {
                setError(winErrText("write failed", GetLastError()));
                writeFailed = true;
                break;
            }
            tx.fetch_add(1, std::memory_order_relaxed);

            Str echo = line;
            while(!echo.empty() && (echo.back() == '\n' || echo.back() == '\r'))
                echo.pop_back();
            pushLine(true, std::move(echo));
        }
        if(writeFailed)
        {
            state.store(PicoState::PICO_STATE_ERROR, std::memory_order_release);
            break;
        }

        // ---- receive --------------------------------------------------------
        DWORD got = 0;
        if(!ReadFile(h, buf.data(), READ_CHUNK, &got, nullptr))
        {
            const DWORD code = GetLastError();
            DWORD commErr = 0;
            COMSTAT st;
            ZeroMemory(&st, sizeof(st));
            ClearCommError(h, &commErr, &st);

            if(code == ERROR_OPERATION_ABORTED)
                continue;   // recoverable: purge/abort raced with the read

            // A Pico that has been unplugged - or told to reboot into
            // BOOTSEL, which drops the CDC port on purpose - is not a fault.
            // Reporting it as one made every deliberate reflash look like a
            // failure, which is precisely how a warning stops being read.
            const dev::Loss why = dev::classify(port, code);
            if(why == dev::Loss::LOSS_UNPLUGGED)
            {
                setError(dev::describe(why, "Pico", port));
                state.store(PicoState::PICO_STATE_UNPLUGGED,
                            std::memory_order_release);
            }
            else
            {
                setError(winErrText("read failed", code));
                state.store(PicoState::PICO_STATE_ERROR,
                            std::memory_order_release);
            }
            break;
        }

        // got == 0 simply means the board said nothing this interval. That is
        // the expected steady state for a silent peer, not an error.
        for(DWORD i = 0; i < got; ++i)
        {
            const Char c = buf[i];

            if(c == '\n')
            {
                if(pendingCr)   // second half of a CRLF that we already ended
                {
                    pendingCr = false;
                    continue;
                }
                if(overlong)
                {
                    overlong = false;
                    accum.clear();
                    continue;
                }
                emit(std::move(accum));
                accum.clear();
                continue;
            }
            if(c == '\r')
            {
                pendingCr = true;
                if(overlong)
                {
                    overlong = false;
                    accum.clear();
                    continue;
                }
                emit(std::move(accum));
                accum.clear();
                continue;
            }

            pendingCr = false;
            if(overlong)
                continue;                       // swallow until the next newline
            if(!printable(static_cast<UInt8>(c)))
                continue;                       // drop NULs and stray control bytes

            accum.push_back(c);
            if(accum.size() >= MAX_LINE_LEN)
            {
                accum += " ...[truncated]";
                emit(std::move(accum));
                accum.clear();
                overlong = true;                // bound the damage, never grow
            }
        }
    }

    // A partial line that never got its newline is still worth showing.
    if(!accum.empty() && !overlong)
        emit(std::move(accum));

    EscapeCommFunction(h, CLRDTR);
    EscapeCommFunction(h, CLRRTS);
    CloseHandle(h);

    // Only a clean stop resets the state; an Error set above is left standing
    // so the UI can read it before disconnect() clears it.
    PicoState expected = PicoState::PICO_STATE_CONNECTED;
    state.compare_exchange_strong(expected, PicoState::PICO_STATE_DISCONNECTED);
}

}  // namespace

// ---------------------------------------------------------------------------
//  PicoLink
// ---------------------------------------------------------------------------

// The header forward-declares Impl; this is it. One inheritance step so the
// body above stays a plain struct and every member access below is a direct
// pointer dereference rather than a map lookup.
struct PicoLink::Impl : LinkImplBody {};

PicoLink::PicoLink()
    : pimpl(new Impl())
{
    pimpl->t0Ns.store(nowNs(), std::memory_order_relaxed);
}

PicoLink::~PicoLink()
{
    disconnect();
    delete pimpl;
    pimpl = nullptr;
}

Void PicoLink::connect(const Str& port, Int32 baud)
{
    if(!pimpl || port.empty())
        return;

    LockGuard<Mutex> life(pimpl->lifeMu);

    if(pimpl->worker.joinable())
    {
        if(!pimpl->finished.load(std::memory_order_acquire))
            return;                 // already connecting/connected: no-op
        pimpl->worker.join();        // reap a worker that died on its own
    }

    // Guard rail: 1200 baud on a Pico CDC port means "reboot into BOOTSEL".
    // Only bootselTouch() is allowed to ask for that.
    Int32 useBaud = (baud <= 0 || baud == BOOTSEL_BAUD) ? DEFAULT_BAUD : baud;

    pimpl->stop.store(false, std::memory_order_release);
    pimpl->finished.store(false, std::memory_order_release);
    pimpl->t0Ns.store(nowNs(), std::memory_order_relaxed);
    pimpl->lastRxS.store(-1.0, std::memory_order_relaxed);
    pimpl->tx.store(0, std::memory_order_relaxed);
    pimpl->rx.store(0, std::memory_order_relaxed);
    pimpl->drops.store(0, std::memory_order_relaxed);
    {
        LockGuard<Mutex> lk(pimpl->mu);
        pimpl->err.clear();
        pimpl->portname = port;
        pimpl->txq.clear();
    }
    pimpl->state.store(PicoState::PICO_STATE_CONNECTING, std::memory_order_release);

    LinkImplBody* raw = pimpl;   // Impl derives from LinkImplBody
    try
    {
        pimpl->worker = Thread(LinkImpl_run_trampoline, raw, port, useBaud);
    }
    catch(...)
    {
        pimpl->finished.store(true, std::memory_order_release);
        pimpl->setError("could not start reader thread");
        pimpl->state.store(PicoState::PICO_STATE_ERROR, std::memory_order_release);
    }
}

Void PicoLink::disconnect()
{
    if(!pimpl)
        return;

    LockGuard<Mutex> life(pimpl->lifeMu);

    pimpl->stop.store(true, std::memory_order_release);
    if(pimpl->worker.joinable())
        pimpl->worker.join();        // blocks until the reader is really gone

    pimpl->stop.store(false, std::memory_order_release);
    pimpl->finished.store(false, std::memory_order_release);
    pimpl->state.store(PicoState::PICO_STATE_DISCONNECTED, std::memory_order_release);

    LockGuard<Mutex> lk(pimpl->mu);
    pimpl->portname.clear();
    pimpl->txq.clear();
}

PicoState PicoLink::state() const
{
    return pimpl ? pimpl->state.load(std::memory_order_acquire) : PicoState::PICO_STATE_DISCONNECTED;
}

Str PicoLink::error() const
{
    if(!pimpl)
        return Str();
    LockGuard<Mutex> lk(pimpl->mu);
    return pimpl->err;
}

Str PicoLink::port() const
{
    if(!pimpl)
        return Str();
    LockGuard<Mutex> lk(pimpl->mu);
    return pimpl->portname;
}

Void PicoLink::send(const Str& line)
{
    if(!pimpl)
        return;

    if(pimpl->state.load(std::memory_order_acquire) != PicoState::PICO_STATE_CONNECTED)
    {
        pimpl->drops.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    Str payload = line;
    // Strip any CR/LF the caller supplied, then terminate with exactly one \n.
    while(!payload.empty() && (payload.back() == '\n' || payload.back() == '\r'))
        payload.pop_back();
    payload.push_back('\n');

    LockGuard<Mutex> lk(pimpl->mu);
    if(pimpl->txq.size() >= MAX_TX_QUEUE)
    {
        // Backed-up writer; counted here too so the drop is at least visible.
        pimpl->drops.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    pimpl->txq.push_back(std::move(payload));
}

Size PicoLink::drain(Vec<PicoLine>& out)
{
    if(!pimpl)
        return 0;

    Deque<PicoLine> taken;
    {
        LockGuard<Mutex> lk(pimpl->mu);   // held only for the swap
        if(pimpl->log.empty())
            return 0;
        taken.swap(pimpl->log);
    }

    const Size n = taken.size();
    out.reserve(out.size() + n);
    for(auto& l : taken)
        out.push_back(std::move(l));
    return n;
}

UInt64 PicoLink::txLines() const
{
    return pimpl ? pimpl->tx.load(std::memory_order_relaxed) : 0ull;
}

UInt64 PicoLink::rxLines() const
{
    return pimpl ? pimpl->rx.load(std::memory_order_relaxed) : 0ull;
}

UInt64 PicoLink::dropped() const
{
    return pimpl ? pimpl->drops.load(std::memory_order_relaxed) : 0ull;
}

Float64 PicoLink::lastRxAgeS() const
{
    if(!pimpl)
        return -1.0;
    const Float64 last = pimpl->lastRxS.load(std::memory_order_relaxed);
    if(last < 0.0)
        return -1.0;               // nothing has ever arrived
    const Float64 age = pimpl->elapsedS() - last;
    return age < 0.0 ? 0.0 : age;
}

Vec<Str> PicoLink::listPicoPorts()
{
    Vec<Str> found;
    try
    {
        enumViaRegistry(found);
        if(found.empty())
            enumViaSetupapi(found);

        const HashSet<Str> live = serialcommPorts();
        Vec<Str> out;
        for(const auto& p : found)
        {
            // Drop entries for Picos that are no longer plugged in. If
            // SERIALCOMM could not be read at all, do not filter.
            if(!live.empty() && live.find(p) == live.end())
                continue;
            if(std::find(out.begin(), out.end(), p) == out.end())
                out.push_back(p);
        }

        std::sort(out.begin(), out.end(), [](const Str& a, const Str& b) {
            const Int32 na = comNumber(a), nb = comNumber(b);
            return (na != nb) ? (na < nb) : (a < b);
        });
        return out;
    }
    catch(...)
    {
        return Vec<Str>();   // documented never to throw
    }
}

Bool PicoLink::bootselTouch(const Str& port)
{
    if(port.empty())
        return false;

    const Str path = devicePath(port);

    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                           OPEN_EXISTING, 0, nullptr);
    if(h == INVALID_HANDLE_VALUE)
        return false;   // the only failure the contract reports

    // The Pico SDK's reset interface watches CDC line state: a line coding of
    // 1200 baud together with DTR *deasserted* means "reboot to BOOTSEL". Set
    // the line coding first so the DTR transition is seen at 1200.
    configurePort(h, BOOTSEL_BAUD, /*assertDtr=*/false, nullptr);
    EscapeCommFunction(h, CLRDTR);
    EscapeCommFunction(h, CLRRTS);
    Sleep(64);          // let the control transfers land before the port dies

    CloseHandle(h);

    // From here the board reboots and re-enumerates as the RPI-RP2 mass storage
    // drive: THIS COM PORT DISAPPEARS. Any open PicoLink on it will fault on its
    // next read and land in PicoState::PICO_STATE_ERROR. That is expected, not a bug.
    return true;
}
