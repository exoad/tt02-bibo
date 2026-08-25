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
//   table keyed by `this`. See kImplTable below.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <setupapi.h>

#include "pico_link.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "setupapi.lib")

namespace
{

// --- tunables ---------------------------------------------------------------

constexpr size_t kMaxLogLines = 4000;   // bounded backlog; oldest dropped
constexpr size_t kMaxTxQueue  = 1024;   // bounded send queue
constexpr size_t kMaxLineLen  = 2048;   // longer input lines are truncated
constexpr DWORD  kReadChunk   = 4096;
constexpr DWORD  kReadTimeoutMs = 30;   // read wakeup period == send latency
constexpr int    kDefaultBaud = 115200;

// Opening a Pico CDC port at 1200 baud and closing it reboots the board into
// BOOTSEL (see bootsel_touch). Normal traffic must never do that by accident.
constexpr int kBootselBaud = 1200;

// --- time -------------------------------------------------------------------

long long now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::string win_err_text(const std::string& what, DWORD code)
{
    char* msg = nullptr;
    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                       FORMAT_MESSAGE_IGNORE_INSERTS,
                   nullptr, code, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                   reinterpret_cast<char*>(&msg), 0, nullptr);

    std::string tail;
    if (msg)
    {
        tail = msg;
        LocalFree(msg);
        while (!tail.empty() && (tail.back() == '\n' || tail.back() == '\r' || tail.back() == ' '))
            tail.pop_back();
    }

    char buf[64];
    _snprintf_s(buf, sizeof(buf), _TRUNCATE, " (error %lu)", static_cast<unsigned long>(code));

    std::string out = what;
    if (!tail.empty())
        out += ": " + tail;
    out += buf;
    return out;
}

// --- per-object state -------------------------------------------------------

struct LinkImpl
{
    // Lifecycle (connect/disconnect/join). Never held while `mu` is held.
    std::mutex              life_mu;
    std::thread             worker;
    std::atomic<bool>       stop{false};
    std::atomic<bool>       finished{false};

    // Data shared with the UI thread.
    mutable std::mutex      mu;
    std::deque<PicoLine>    log;
    std::deque<std::string> txq;
    std::string             err;
    std::string             portname;

    std::atomic<PicoState>          state{PicoState::Disconnected};
    std::atomic<unsigned long long> tx{0};
    std::atomic<unsigned long long> rx{0};
    std::atomic<unsigned long long> drops{0};
    std::atomic<long long>          t0_ns{0};
    std::atomic<double>             last_rx_s{-1.0};   // <0 == nothing ever received

    double elapsed_s() const
    {
        return static_cast<double>(now_ns() - t0_ns.load(std::memory_order_relaxed)) * 1e-9;
    }

    void set_error(std::string e)
    {
        std::lock_guard<std::mutex> lk(mu);
        err = std::move(e);
    }

    void push_line(bool outgoing, std::string text)
    {
        PicoLine ln;
        ln.t_s      = elapsed_s();
        ln.outgoing = outgoing;
        ln.text     = std::move(text);

        std::lock_guard<std::mutex> lk(mu);
        log.push_back(std::move(ln));
        while (log.size() > kMaxLogLines)
            log.pop_front();   // a chatty board loses history, never memory
    }

    void run(std::string port, int baud);
};

// The header gives us nowhere to put a pointer, so objects are mapped by
// address. Lookups are O(1) and happen a handful of times per UI frame.
std::mutex& impl_table_mu()
{
    static std::mutex m;
    return m;
}

std::unordered_map<const PicoLink*, std::shared_ptr<LinkImpl>>& impl_table()
{
    static std::unordered_map<const PicoLink*, std::shared_ptr<LinkImpl>> t;
    return t;
}

std::shared_ptr<LinkImpl> impl_of(const PicoLink* self)
{
    std::lock_guard<std::mutex> lk(impl_table_mu());
    auto it = impl_table().find(self);
    return (it == impl_table().end()) ? nullptr : it->second;
}

// --- port helpers -----------------------------------------------------------

// COM10 and above are only reachable through the "\\.\" device namespace; a
// bare "COM10" resolves to a legacy DOS device name and fails to open.
std::string device_path(const std::string& port)
{
    if (port.size() >= 4 && port.compare(0, 4, "\\\\.\\") == 0)
        return port;
    return "\\\\.\\" + port;
}

bool configure_port(HANDLE h, int baud, bool assert_dtr, std::string* err)
{
    DCB dcb;
    ZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb))
    {
        if (err) *err = win_err_text("GetCommState failed", GetLastError());
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
    dcb.fDtrControl      = assert_dtr ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;
    dcb.fRtsControl      = assert_dtr ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE;

    if (!SetCommState(h, &dcb))
    {
        if (err) *err = win_err_text("SetCommState failed", GetLastError());
        return false;
    }
    return true;
}

// --- registry / SetupAPI enumeration ---------------------------------------

int com_number(const std::string& s)
{
    size_t i = 0;
    while (i < s.size() && (s[i] < '0' || s[i] > '9'))
        ++i;
    int n = 0;
    bool any = false;
    for (; i < s.size() && s[i] >= '0' && s[i] <= '9'; ++i)
    {
        n = n * 10 + (s[i] - '0');
        any = true;
    }
    return any ? n : 0;
}

// Every COM name Windows currently has mapped. Used to filter out stale
// registry entries for Picos that are no longer plugged in.
std::unordered_set<std::string> serialcomm_ports()
{
    std::unordered_set<std::string> out;

    HKEY key = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0,
                      KEY_READ, &key) != ERROR_SUCCESS)
        return out;

    for (DWORD i = 0;; ++i)
    {
        char  name[512];
        BYTE  data[512];
        DWORD name_len = static_cast<DWORD>(sizeof(name));
        DWORD data_len = static_cast<DWORD>(sizeof(data));
        DWORD type     = 0;

        LONG rc = RegEnumValueA(key, i, name, &name_len, nullptr, &type, data, &data_len);
        if (rc == ERROR_NO_MORE_ITEMS)
            break;
        if (rc != ERROR_SUCCESS)
            break;
        if (type != REG_SZ || data_len == 0)
            continue;

        data[(std::min)(static_cast<size_t>(data_len), sizeof(data) - 1)] = 0;
        std::string port(reinterpret_cast<char*>(data));
        if (!port.empty())
            out.insert(port);
    }

    RegCloseKey(key);
    return out;
}

bool read_port_name(HKEY parent, const char* subkey, std::string* out)
{
    HKEY k = nullptr;
    if (RegOpenKeyExA(parent, subkey, 0, KEY_READ, &k) != ERROR_SUCCESS)
        return false;

    char  buf[128];
    DWORD len  = static_cast<DWORD>(sizeof(buf));
    DWORD type = 0;
    LONG  rc   = RegQueryValueExA(k, "PortName", nullptr, &type,
                                  reinterpret_cast<BYTE*>(buf), &len);
    RegCloseKey(k);

    if (rc != ERROR_SUCCESS || type != REG_SZ || len == 0)
        return false;

    buf[(std::min)(static_cast<size_t>(len), sizeof(buf) - 1)] = 0;
    *out = buf;
    return !out->empty();
}

// Preferred route: walk HKLM\SYSTEM\CurrentControlSet\Enum\USB looking for
// VID_2E8A*, then read each instance's Device Parameters\PortName. No extra
// libraries, and it copes with composite devices (VID_2E8A&PID_0009&MI_00).
void enum_via_registry(std::vector<std::string>& out)
{
    HKEY usb = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "SYSTEM\\CurrentControlSet\\Enum\\USB", 0,
                      KEY_READ, &usb) != ERROR_SUCCESS)
        return;

    for (DWORD i = 0;; ++i)
    {
        char  dev[512];
        DWORD dev_len = static_cast<DWORD>(sizeof(dev));
        LONG  rc = RegEnumKeyExA(usb, i, dev, &dev_len, nullptr, nullptr, nullptr, nullptr);
        if (rc != ERROR_SUCCESS)
            break;
        if (_strnicmp(dev, "VID_2E8A", 8) != 0)
            continue;

        HKEY devk = nullptr;
        if (RegOpenKeyExA(usb, dev, 0, KEY_READ, &devk) != ERROR_SUCCESS)
            continue;

        for (DWORD j = 0;; ++j)
        {
            char  inst[512];
            DWORD inst_len = static_cast<DWORD>(sizeof(inst));
            if (RegEnumKeyExA(devk, j, inst, &inst_len, nullptr, nullptr, nullptr, nullptr)
                != ERROR_SUCCESS)
                break;

            std::string sub = std::string(inst) + "\\Device Parameters";
            std::string port;
            if (read_port_name(devk, sub.c_str(), &port))
                out.push_back(port);
        }

        RegCloseKey(devk);
    }

    RegCloseKey(usb);
}

// Case-insensitive substring test, ASCII only (no locale surprises).
bool contains_ci(const char* hay, const char* needle)
{
    const size_t nlen = strlen(needle);
    if (nlen == 0)
        return true;
    for (const char* p = hay; *p; ++p)
        if (_strnicmp(p, needle, nlen) == 0)
            return true;
    return false;
}

// Fallback if the Enum\USB hive is unreadable: class-enumerate present COM
// ports and match VID_2E8A in the hardware ID. GUID_DEVCLASS_PORTS is spelled
// out here so we need not link uuid.lib.
void enum_via_setupapi(std::vector<std::string>& out)
{
    static const GUID kPortsClass = {
        0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}};

    HDEVINFO set = SetupDiGetClassDevsA(&kPortsClass, nullptr, nullptr, DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE)
        return;

    SP_DEVINFO_DATA info;
    ZeroMemory(&info, sizeof(info));
    info.cbSize = sizeof(info);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &info); ++i)
    {
        char  hwid[1024] = {0};
        DWORD hwid_len   = 0;
        if (!SetupDiGetDeviceRegistryPropertyA(set, &info, SPDRP_HARDWAREID, nullptr,
                                               reinterpret_cast<BYTE*>(hwid),
                                               sizeof(hwid) - 2, &hwid_len))
            continue;

        // REG_MULTI_SZ: scan every embedded string.
        bool match = false;
        for (DWORD p = 0; p < hwid_len && hwid[p]; )
        {
            if (contains_ci(hwid + p, "VID_2E8A"))
                match = true;
            p += static_cast<DWORD>(strlen(hwid + p)) + 1;
        }
        if (!match)
            continue;

        HKEY k = SetupDiOpenDevRegKey(set, &info, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (k == INVALID_HANDLE_VALUE)
            continue;

        char  buf[128];
        DWORD len  = static_cast<DWORD>(sizeof(buf));
        DWORD type = 0;
        if (RegQueryValueExA(k, "PortName", nullptr, &type, reinterpret_cast<BYTE*>(buf), &len)
                == ERROR_SUCCESS &&
            type == REG_SZ && len > 0)
        {
            buf[(std::min)(static_cast<size_t>(len), sizeof(buf) - 1)] = 0;
            if (buf[0])
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
inline bool printable(unsigned char c)
{
    return c >= 0x20 && c != 0x7F;
}

void LinkImpl_run_trampoline(LinkImpl* self, std::string port, int baud)
{
    self->run(std::move(port), baud);
}

void LinkImpl::run(std::string port, int baud)
{
    struct DoneFlag
    {
        LinkImpl* p;
        ~DoneFlag() { p->finished.store(true, std::memory_order_release); }
    } done_flag{this};

    const std::string path = device_path(port);

    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                           0,            // serial ports are exclusive
                           nullptr, OPEN_EXISTING,
                           0,            // synchronous; this thread may block
                           nullptr);
    if (h == INVALID_HANDLE_VALUE)
    {
        set_error(win_err_text("cannot open " + path, GetLastError()));
        state.store(PicoState::Error, std::memory_order_release);
        return;
    }

    std::string cfg_err;
    if (!configure_port(h, baud, /*assert_dtr=*/true, &cfg_err))
    {
        set_error(std::move(cfg_err));
        state.store(PicoState::Error, std::memory_order_release);
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
    to.ReadTotalTimeoutConstant    = kReadTimeoutMs;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = 1000;
    if (!SetCommTimeouts(h, &to))
    {
        set_error(win_err_text("SetCommTimeouts failed", GetLastError()));
        state.store(PicoState::Error, std::memory_order_release);
        CloseHandle(h);
        return;
    }

    SetupComm(h, 8192, 8192);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
    EscapeCommFunction(h, SETDTR);   // some firmware waits for host DTR
    EscapeCommFunction(h, SETRTS);

    {
        std::lock_guard<std::mutex> lk(mu);
        portname = port;
        err.clear();
    }
    state.store(PicoState::Connected, std::memory_order_release);

    std::string accum;          // partial line carried across reads
    bool        pending_cr = false;
    bool        overlong   = false;
    std::vector<char> buf(kReadChunk);

    auto emit = [&](std::string text)
    {
        if (text.empty())
            return;   // blank lines are noise from \r\n\r\n padding
        rx.fetch_add(1, std::memory_order_relaxed);
        last_rx_s.store(elapsed_s(), std::memory_order_relaxed);
        push_line(false, std::move(text));
    };

    while (!stop.load(std::memory_order_acquire))
    {
        // ---- transmit anything the UI queued -------------------------------
        std::deque<std::string> outbound;
        {
            std::lock_guard<std::mutex> lk(mu);
            outbound.swap(txq);
        }
        bool write_failed = false;
        for (auto& line : outbound)
        {
            DWORD       written = 0;
            const DWORD n       = static_cast<DWORD>(line.size());
            if (!WriteFile(h, line.data(), n, &written, nullptr) || written != n)
            {
                set_error(win_err_text("write failed", GetLastError()));
                write_failed = true;
                break;
            }
            tx.fetch_add(1, std::memory_order_relaxed);

            std::string echo = line;
            while (!echo.empty() && (echo.back() == '\n' || echo.back() == '\r'))
                echo.pop_back();
            push_line(true, std::move(echo));
        }
        if (write_failed)
        {
            state.store(PicoState::Error, std::memory_order_release);
            break;
        }

        // ---- receive --------------------------------------------------------
        DWORD got = 0;
        if (!ReadFile(h, buf.data(), kReadChunk, &got, nullptr))
        {
            const DWORD code = GetLastError();
            DWORD comm_err = 0;
            COMSTAT st;
            ZeroMemory(&st, sizeof(st));
            ClearCommError(h, &comm_err, &st);

            if (code == ERROR_OPERATION_ABORTED)
                continue;   // recoverable: purge/abort raced with the read

            set_error(win_err_text("read failed (device removed?)", code));
            state.store(PicoState::Error, std::memory_order_release);
            break;
        }

        // got == 0 simply means the board said nothing this interval. That is
        // the expected steady state for a silent peer, not an error.
        for (DWORD i = 0; i < got; ++i)
        {
            const char c = buf[i];

            if (c == '\n')
            {
                if (pending_cr)   // second half of a CRLF that we already ended
                {
                    pending_cr = false;
                    continue;
                }
                if (overlong) { overlong = false; accum.clear(); continue; }
                emit(std::move(accum));
                accum.clear();
                continue;
            }
            if (c == '\r')
            {
                pending_cr = true;
                if (overlong) { overlong = false; accum.clear(); continue; }
                emit(std::move(accum));
                accum.clear();
                continue;
            }

            pending_cr = false;
            if (overlong)
                continue;                       // swallow until the next newline
            if (!printable(static_cast<unsigned char>(c)))
                continue;                       // drop NULs and stray control bytes

            accum.push_back(c);
            if (accum.size() >= kMaxLineLen)
            {
                accum += " ...[truncated]";
                emit(std::move(accum));
                accum.clear();
                overlong = true;                // bound the damage, never grow
            }
        }
    }

    // A partial line that never got its newline is still worth showing.
    if (!accum.empty() && !overlong)
        emit(std::move(accum));

    EscapeCommFunction(h, CLRDTR);
    EscapeCommFunction(h, CLRRTS);
    CloseHandle(h);

    // Only a clean stop resets the state; an Error set above is left standing
    // so the UI can read it before disconnect() clears it.
    PicoState expected = PicoState::Connected;
    state.compare_exchange_strong(expected, PicoState::Disconnected);
}

}  // namespace

// ---------------------------------------------------------------------------
//  PicoLink
// ---------------------------------------------------------------------------

PicoLink::PicoLink()
{
    auto impl = std::make_shared<LinkImpl>();
    impl->t0_ns.store(now_ns(), std::memory_order_relaxed);

    std::lock_guard<std::mutex> lk(impl_table_mu());
    impl_table()[this] = std::move(impl);
}

PicoLink::~PicoLink()
{
    disconnect();
    std::lock_guard<std::mutex> lk(impl_table_mu());
    impl_table().erase(this);
}

void PicoLink::connect(const std::string& port, int baud)
{
    auto impl = impl_of(this);
    if (!impl || port.empty())
        return;

    std::lock_guard<std::mutex> life(impl->life_mu);

    if (impl->worker.joinable())
    {
        if (!impl->finished.load(std::memory_order_acquire))
            return;                 // already connecting/connected: no-op
        impl->worker.join();        // reap a worker that died on its own
    }

    // Guard rail: 1200 baud on a Pico CDC port means "reboot into BOOTSEL".
    // Only bootsel_touch() is allowed to ask for that.
    int use_baud = (baud <= 0 || baud == kBootselBaud) ? kDefaultBaud : baud;

    impl->stop.store(false, std::memory_order_release);
    impl->finished.store(false, std::memory_order_release);
    impl->t0_ns.store(now_ns(), std::memory_order_relaxed);
    impl->last_rx_s.store(-1.0, std::memory_order_relaxed);
    impl->tx.store(0, std::memory_order_relaxed);
    impl->rx.store(0, std::memory_order_relaxed);
    impl->drops.store(0, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(impl->mu);
        impl->err.clear();
        impl->portname = port;
        impl->txq.clear();
    }
    impl->state.store(PicoState::Connecting, std::memory_order_release);

    LinkImpl* raw = impl.get();
    try
    {
        impl->worker = std::thread(LinkImpl_run_trampoline, raw, port, use_baud);
    }
    catch (...)
    {
        impl->finished.store(true, std::memory_order_release);
        impl->set_error("could not start reader thread");
        impl->state.store(PicoState::Error, std::memory_order_release);
    }
}

void PicoLink::disconnect()
{
    auto impl = impl_of(this);
    if (!impl)
        return;

    std::lock_guard<std::mutex> life(impl->life_mu);

    impl->stop.store(true, std::memory_order_release);
    if (impl->worker.joinable())
        impl->worker.join();        // blocks until the reader is really gone

    impl->stop.store(false, std::memory_order_release);
    impl->finished.store(false, std::memory_order_release);
    impl->state.store(PicoState::Disconnected, std::memory_order_release);

    std::lock_guard<std::mutex> lk(impl->mu);
    impl->portname.clear();
    impl->txq.clear();
}

PicoState PicoLink::state() const
{
    auto impl = impl_of(this);
    return impl ? impl->state.load(std::memory_order_acquire) : PicoState::Disconnected;
}

std::string PicoLink::error() const
{
    auto impl = impl_of(this);
    if (!impl)
        return std::string();
    std::lock_guard<std::mutex> lk(impl->mu);
    return impl->err;
}

std::string PicoLink::port() const
{
    auto impl = impl_of(this);
    if (!impl)
        return std::string();
    std::lock_guard<std::mutex> lk(impl->mu);
    return impl->portname;
}

void PicoLink::send(const std::string& line)
{
    auto impl = impl_of(this);
    if (!impl)
        return;

    if (impl->state.load(std::memory_order_acquire) != PicoState::Connected)
    {
        impl->drops.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    std::string payload = line;
    // Strip any CR/LF the caller supplied, then terminate with exactly one \n.
    while (!payload.empty() && (payload.back() == '\n' || payload.back() == '\r'))
        payload.pop_back();
    payload.push_back('\n');

    std::lock_guard<std::mutex> lk(impl->mu);
    if (impl->txq.size() >= kMaxTxQueue)
    {
        // Backed-up writer; counted here too so the drop is at least visible.
        impl->drops.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    impl->txq.push_back(std::move(payload));
}

size_t PicoLink::drain(std::vector<PicoLine>& out)
{
    auto impl = impl_of(this);
    if (!impl)
        return 0;

    std::deque<PicoLine> taken;
    {
        std::lock_guard<std::mutex> lk(impl->mu);   // held only for the swap
        if (impl->log.empty())
            return 0;
        taken.swap(impl->log);
    }

    const size_t n = taken.size();
    out.reserve(out.size() + n);
    for (auto& l : taken)
        out.push_back(std::move(l));
    return n;
}

unsigned long long PicoLink::tx_lines() const
{
    auto impl = impl_of(this);
    return impl ? impl->tx.load(std::memory_order_relaxed) : 0ull;
}

unsigned long long PicoLink::rx_lines() const
{
    auto impl = impl_of(this);
    return impl ? impl->rx.load(std::memory_order_relaxed) : 0ull;
}

unsigned long long PicoLink::dropped() const
{
    auto impl = impl_of(this);
    return impl ? impl->drops.load(std::memory_order_relaxed) : 0ull;
}

double PicoLink::last_rx_age_s() const
{
    auto impl = impl_of(this);
    if (!impl)
        return -1.0;
    const double last = impl->last_rx_s.load(std::memory_order_relaxed);
    if (last < 0.0)
        return -1.0;               // nothing has ever arrived
    const double age = impl->elapsed_s() - last;
    return age < 0.0 ? 0.0 : age;
}

std::vector<std::string> PicoLink::list_pico_ports()
{
    std::vector<std::string> found;
    try
    {
        enum_via_registry(found);
        if (found.empty())
            enum_via_setupapi(found);

        const std::unordered_set<std::string> live = serialcomm_ports();
        std::vector<std::string> out;
        for (const auto& p : found)
        {
            // Drop entries for Picos that are no longer plugged in. If
            // SERIALCOMM could not be read at all, do not filter.
            if (!live.empty() && live.find(p) == live.end())
                continue;
            if (std::find(out.begin(), out.end(), p) == out.end())
                out.push_back(p);
        }

        std::sort(out.begin(), out.end(), [](const std::string& a, const std::string& b) {
            const int na = com_number(a), nb = com_number(b);
            return (na != nb) ? (na < nb) : (a < b);
        });
        return out;
    }
    catch (...)
    {
        return std::vector<std::string>();   // documented never to throw
    }
}

bool PicoLink::bootsel_touch(const std::string& port)
{
    if (port.empty())
        return false;

    const std::string path = device_path(port);

    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                           OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE)
        return false;   // the only failure the contract reports

    // The Pico SDK's reset interface watches CDC line state: a line coding of
    // 1200 baud together with DTR *deasserted* means "reboot to BOOTSEL". Set
    // the line coding first so the DTR transition is seen at 1200.
    configure_port(h, kBootselBaud, /*assert_dtr=*/false, nullptr);
    EscapeCommFunction(h, CLRDTR);
    EscapeCommFunction(h, CLRRTS);
    Sleep(64);          // let the control transfers land before the port dies

    CloseHandle(h);

    // From here the board reboots and re-enumerates as the RPI-RP2 mass storage
    // drive: THIS COM PORT DISAPPEARS. Any open PicoLink on it will fault on its
    // next read and land in PicoState::Error. That is expected, not a bug.
    return true;
}
