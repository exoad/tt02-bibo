// Firmware catalog and flashing - see pico_flash.h for the contract.
//
// This is a FRONT-END, not a second flashing implementation. Every operation is
// a child process running the same script you would run from a terminal:
//
//     build   ->  firmware\build.bat
//     flash   ->  firmware\flash.bat <uf2>
//     backup  ->  firmware\backup.bat <uf2>
//
// so there is exactly one mechanism and one place for the hard-won toolchain
// knowledge to live (see the comment blocks in those scripts). The only things
// done directly here are the two *queries* - what the board is, and what is on
// disk - plus reboot, which is picotool doing its own job.
//
// Everything runs on a worker thread and streams the child's stdout/stderr into
// a line log as it arrives, because a flash takes ten seconds and a build takes
// a minute: output that only appears at the end is indistinguishable from a
// hang.
#include "shared.hpp"
#include "pico_flash.hpp"

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "pico_link.hpp"   // listPicoPorts(): the VID 2E8A walk already lives there

namespace {

// The GUI owns exactly one PicoFlash and the header declares no members, so the
// state is here. The constructor enforces the "one instance" assumption rather
// than leaving it implicit.
constexpr Size PENDING_MAX = 4000;   // this app runs for hours; the log is bounded

Str trim(const Str& s)
{
    Size a = 0, b = s.size();
    while(a < b && static_cast<UInt8>(s[a]) <= ' ') ++a;
    while(b > a && static_cast<UInt8>(s[b - 1]) <= ' ') --b;
    return s.substr(a, b - a);
}

Bool dirExists(const Str& p)
{
    const DWORD a = GetFileAttributesA(p.c_str());
    return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY) != 0;
}

Bool fileExists(const Str& p)
{
    const DWORD a = GetFileAttributesA(p.c_str());
    return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY) == 0;
}

// Backslashes throughout: these strings end up on a cmd.exe command line, and
// catalog.txt writes them with forward slashes because it is a text file.
Str toBackslashes(Str s)
{
    std::replace(s.begin(), s.end(), '/', '\\');
    return s;
}

// cmd.exe's quoting rule with /s: the first and last quote of the argument are
// stripped and everything between is taken literally. That is the only reliable
// way to run "a path with spaces\x.bat" "another path with spaces".
Str quote(const Str& s)
{
    return "\"" + s + "\"";
}

Str formatMtime(const FILETIME& ft)
{
    FILETIME local{};
    SYSTEMTIME st{};
    if(!FileTimeToLocalFileTime(&ft, &local)) return Str();
    if(!FileTimeToSystemTime(&local, &st))    return Str();

    Char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d",
                  st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute);
    return buf;
}

// ---------------------------------------------------------------- board ----

// The RP2040 bootloader labels its drive RPI-RP2; the RP2350 on a Pico 2 W
// labels it RP2350. Accepting only one of the two means never finding the
// board - the scripts carry the same note for the same reason.
Bool findBootselDrive(Str& outDrive)
{
    const DWORD mask = GetLogicalDrives();
    for(Int32 i = 0; i < 26; ++i)
    {
        if(!(mask & (1u << i))) continue;

        Char root[8];
        std::snprintf(root, sizeof(root), "%c:\\", 'A' + i);
        if(GetDriveTypeA(root) != DRIVE_REMOVABLE) continue;

        Char label[MAX_PATH] = {};
        if(!GetVolumeInformationA(root, label, MAX_PATH, nullptr, nullptr, nullptr, nullptr, 0))
            continue;   // an empty card reader: no volume, not an error

        if(_stricmp(label, "RP2350") == 0 || _stricmp(label, "RPI-RP2") == 0)
        {
            outDrive = Str(1, static_cast<Char>(('A' + i))) + ":";
            return true;
        }
    }
    return false;
}

// ------------------------------------------------------------- processes ---

using LineSink = std::function<Void(const Str&)>;

// Runs `cmdline` and streams its combined stdout/stderr to `sink` a line at a
// time. Returns the process exit code, or -1 if it could not be started.
//
// stdout and stderr share one pipe on purpose: the scripts write progress to
// one and errors to the other, and interleaving them is what makes the output
// pane read like a terminal.
Int32 runCapture(const Str& cmdline, const Str& cwd, const LineSink& sink)
{
    SECURITY_ATTRIBUTES sa{};
    sa.nLength        = sizeof(sa);
    sa.bInheritHandle = TRUE;

    HANDLE rd = nullptr, wr = nullptr;
    if(!CreatePipe(&rd, &wr, &sa, 0))
    {
        sink("[error] CreatePipe failed");
        return -1;
    }
    // Only the write end is inherited; leaving the read end inheritable means
    // the final ReadFile never sees EOF because the child still holds it.
    SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);

    // A real handle for stdin, not NULL: a batch file that reads (or that pipes
    // into something that reads) would otherwise block forever.
    HANDLE nul = CreateFileA("NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                             &sa, OPEN_EXISTING, 0, nullptr);

    STARTUPINFOA si{};
    si.cb         = sizeof(si);
    si.dwFlags    = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
    si.wShowWindow = SW_HIDE;
    si.hStdInput  = (nul == INVALID_HANDLE_VALUE) ? nullptr : nul;
    si.hStdOutput = wr;
    si.hStdError  = wr;

    PROCESS_INFORMATION pi{};

    // CreateProcessA writes to its command line argument, so it cannot be a
    // string literal or a c_str().
    std::vector<Char> mutableCmd(cmdline.begin(), cmdline.end());
    mutableCmd.push_back('\0');

    const BOOL ok = CreateProcessA(nullptr, mutableCmd.data(), nullptr, nullptr,
                                   TRUE, CREATE_NO_WINDOW, nullptr,
                                   cwd.empty() ? nullptr : cwd.c_str(), &si, &pi);

    CloseHandle(wr);   // the parent's copy, or the read below never ends
    if(nul != INVALID_HANDLE_VALUE) CloseHandle(nul);

    if(!ok)
    {
        Char buf[160];
        std::snprintf(buf, sizeof(buf), "[error] could not start the process (%lu)",
                      static_cast<unsigned long>(GetLastError()));
        sink(buf);
        CloseHandle(rd);
        return -1;
    }

    Str partial;
    Char        chunk[4096];
    DWORD       got = 0;

    while(ReadFile(rd, chunk, sizeof(chunk), &got, nullptr) && got > 0)
    {
        partial.append(chunk, got);

        Size nl;
        while((nl = partial.find('\n')) != Str::npos)
        {
            Str line = partial.substr(0, nl);
            partial.erase(0, nl + 1);
            if(!line.empty() && line.back() == '\r') line.pop_back();
            sink(line);
        }

        // A script that ends without a newline, or one that prints a prompt and
        // waits, would otherwise be invisible. Flush anything that has sat in
        // the buffer once it gets long.
        if(partial.size() > 512)
        {
            sink(partial);
            partial.clear();
        }
    }
    if(!partial.empty()) sink(partial);

    WaitForSingleObject(pi.hProcess, INFINITE);

    DWORD code = 0;
    GetExitCodeProcess(pi.hProcess, &code);

    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    CloseHandle(rd);
    return static_cast<Int32>(code);
}

// ------------------------------------------------------------------ state --

struct Impl
{
    mutable std::mutex mu;              // guards catalog, op, brd

    std::vector<FirmwareEntry> catalog;
    Str                op;
    BoardStatus                brd;

    std::mutex              logMu;
    std::deque<Str> pending;

    std::atomic<FlashState> state{FlashState::FLASH_STATE_IDLE};
    std::atomic<Bool>       boardQuerying{false};

    std::thread worker;

    Void log(const Str& line)
    {
        std::lock_guard<std::mutex> lk(logMu);
        pending.push_back(line);
        while(pending.size() > PENDING_MAX) pending.pop_front();
    }

    Void logf(const Char* fmt, ...)
    {
        Char buf[1024];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        log(buf);
    }

    // Starts `body` on the worker. Rejects (and says so) while one is running,
    // which is the documented behaviour and also the only safe one: two things
    // touching the board's USB at once is how you end up with a brick.
    Void start(const Str& desc, std::function<Int32()> body)
    {
        if(state.load() == FlashState::FLASH_STATE_WORKING)
        {
            std::lock_guard<std::mutex> lk(mu);
            logf("[busy ] %s is still running - %s was not started", op.c_str(), desc.c_str());
            return;
        }

        if(worker.joinable()) worker.join();   // finished, just not reaped yet

        {
            std::lock_guard<std::mutex> lk(mu);
            op = desc;
        }
        state.store(FlashState::FLASH_STATE_WORKING);
        logf("[start] %s", desc.c_str());

        worker = std::thread([this, body, desc]()
        {
            Int32 code = -1;
            try { code = body(); } catch(...) { code = -1; }

            if(code == 0)
            {
                logf("[ok   ] %s", desc.c_str());
                state.store(FlashState::FLASH_STATE_SUCCESS);
            }
            else
            {
                logf("[fail ] %s (exit %d)", desc.c_str(), code);
                state.store(FlashState::FLASH_STATE_FAILED);
            }
        });
    }
};

Impl& pimpl()
{
    static Impl s;
    return s;
}

const FirmwareEntry* findEntry(const std::vector<FirmwareEntry>& v, const Str& id)
{
    for(const FirmwareEntry& e : v)
        if(e.id == id) return &e;
    return nullptr;
}

} // namespace

// ------------------------------------------------------------------ public --

PicoFlash::PicoFlash()
{
    static_cast<Void>(pimpl());   // force the singleton up now, not from the first worker
}

PicoFlash::~PicoFlash()
{
    // Never detach: the worker writes into pimpl(), and pimpl() is a function
    // static that outlives this object but not the process teardown.
    if(pimpl().worker.joinable()) pimpl().worker.join();
}

// repoRoot(): the exe lives in hub\build\, so walk up until a
// directory holds both firmware\ and lidar\. Derived rather than hard-coded so
// the tree can be moved or renamed.
Str PicoFlash::repoRoot()
{
    static Str cached;
    static Bool        done = false;
    if(done) return cached;
    done = true;

    Char exe[MAX_PATH] = {};
    if(GetModuleFileNameA(nullptr, exe, MAX_PATH) == 0)
        return cached;

    Str dir(exe);
    Size slash = dir.find_last_of("\\/");
    if(slash == Str::npos) return cached;
    dir.resize(slash);

    for(Int32 up = 0; up < 12; ++up)
    {
        // Markers are `firmware\` (what we shell out to) and `hub\` (our own
        // home). Deliberately not `lidar\`: that is one sensor's directory and
        // the app no longer lives inside it, so keying the repo root off it
        // would break the moment the tree is reorganised again.
        if(dirExists(dir + "\\firmware") && dirExists(dir + "\\hub"))
        {
            cached = dir;
            return cached;
        }
        slash = dir.find_last_of("\\/");
        if(slash == Str::npos || slash < 3) break;   // past "C:\"
        dir.resize(slash);
    }
    return cached;   // empty: the UI shows that plainly rather than guessing
}

Void PicoFlash::refreshCatalog()
{
    const Str root = repoRoot();
    std::vector<FirmwareEntry> out;

    if(root.empty())
    {
        pimpl().log("[error] repo root not found above the executable - "
                   "expected a directory containing both firmware\\ and lidar\\");
        std::lock_guard<std::mutex> lk(pimpl().mu);
        pimpl().catalog.swap(out);
        return;
    }

    const Str path = root + "\\firmware\\catalog.txt";
    std::ifstream in(path.c_str());
    if(!in)
    {
        pimpl().logf("[error] cannot read %s", path.c_str());
        std::lock_guard<std::mutex> lk(pimpl().mu);
        pimpl().catalog.swap(out);
        return;
    }

    Str line;
    while(std::getline(in, line))
    {
        Str s = trim(line);
        if(s.empty() || s[0] == '#') continue;

        // id | name | uf2 path | buildable | description
        Str field[5];
        Int32         n     = 0;
        Size      start = 0;
        for(Size i = 0; i <= s.size() && n < 5; ++i)
        {
            if(i == s.size() || s[i] == '|')
            {
                // The description is last and may itself contain a pipe, so the
                // final field takes the whole remainder.
                if(n == 4) { field[n++] = trim(s.substr(start)); break; }
                field[n++] = trim(s.substr(start, i - start));
                start = i + 1;
            }
        }
        if(n < 4 || field[0].empty() || field[2].empty()) continue;

        FirmwareEntry e;
        e.id          = field[0];
        e.name        = field[1].empty() ? field[0] : field[1];
        e.uf2Path    = root + "\\" + toBackslashes(field[2]);
        e.buildable   = (_stricmp(field[3].c_str(), "yes") == 0 ||
                         _stricmp(field[3].c_str(), "true") == 0);
        e.description = field[4];

        WIN32_FILE_ATTRIBUTE_DATA fa{};
        if(GetFileAttributesExA(e.uf2Path.c_str(), GetFileExInfoStandard, &fa) &&
            !(fa.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
        {
            e.present    = true;
            e.sizeBytes = (static_cast<Int64>(fa.nFileSizeHigh) << 32) | static_cast<Int64>(fa.nFileSizeLow);
            e.builtAt   = formatMtime(fa.ftLastWriteTime);
        }

        out.push_back(std::move(e));
    }

    std::lock_guard<std::mutex> lk(pimpl().mu);
    pimpl().catalog.swap(out);
}

const std::vector<FirmwareEntry>& PicoFlash::catalog() const
{
    // Only refreshCatalog() writes this, and only from the UI thread, so
    // handing back a reference is safe. The lock is for the swap.
    std::lock_guard<std::mutex> lk(pimpl().mu);
    return pimpl().catalog;
}

FlashState PicoFlash::state() const { return pimpl().state.load(); }
Bool       PicoFlash::busy()  const { return pimpl().state.load() == FlashState::FLASH_STATE_WORKING; }

Str PicoFlash::currentOp() const
{
    std::lock_guard<std::mutex> lk(pimpl().mu);
    return pimpl().op;
}

Size PicoFlash::drainLog(std::vector<Str>& out)
{
    std::lock_guard<std::mutex> lk(pimpl().logMu);
    const Size n = pimpl().pending.size();
    for(const Str& s : pimpl().pending) out.push_back(s);
    pimpl().pending.clear();
    return n;
}

BoardStatus PicoFlash::board() const
{
    std::lock_guard<std::mutex> lk(pimpl().mu);
    return pimpl().brd;
}

// ---------------------------------------------------------------- board -----

Void PicoFlash::refreshBoard()
{
    // Deliberately its own thread and its own guard: this is a query, not an
    // operation, so it must not make busy() true and block the buttons - but it
    // spawns picotool, so it must not run on the UI thread either.
    if(pimpl().boardQuerying.exchange(true)) return;

    std::thread([]()
    {
        BoardStatus b;

        b.bootsel = findBootselDrive(b.drive);

        if(!b.bootsel)
        {
            const std::vector<Str> ports = PicoLink::listPicoPorts();
            if(!ports.empty()) b.port = ports.front();
        }

        // picotool WITHOUT -f. With -f it would reboot a running board into the
        // bootloader just to answer a question, which is a rude thing for a
        // status refresh to do. The trade: the program name is only readable
        // when the board is already in BOOTSEL.
        const Str root = PicoFlash::repoRoot();
        if(!root.empty())
        {
            const Str picotool =
                root + "\\vendor\\picotool-2.3.0\\picotool\\picotool.exe";

            if(fileExists(picotool))
            {
                std::vector<Str> lines;
                runCapture(quote(picotool) + " info", root,
                            [&lines](const Str& l) { lines.push_back(l); });

                for(const Str& l : lines)
                {
                    const Str t = trim(l);

                    // In BOOTSEL:  " name:          pico_debug"
                    //              " target chip:   RP2350"
                    if(t.rfind("name:", 0) == 0)
                        b.program = trim(t.substr(5));
                    else if(t.rfind("target chip:", 0) == 0)
                        b.chip = trim(t.substr(12));

                    // Running: "RP2350 device at bus 1, address 13 appears to
                    // have a USB serial connection". That line is picotool
                    // declining to do anything, but it still identifies the chip.
                    else if(b.chip.empty() && t.find(" device at bus ") != Str::npos)
                    {
                        const Size sp = t.find(' ');
                        if(sp != Str::npos && sp > 0) b.chip = t.substr(0, sp);
                    }
                }
            }
        }

        b.present = b.bootsel || !b.port.empty() || !b.chip.empty();

        {
            std::lock_guard<std::mutex> lk(pimpl().mu);
            pimpl().brd = b;
        }

        if(!b.present)
            pimpl().log("[board] nothing found: no RP2350/RPI-RP2 drive and no VID 2E8A port");
        else if(b.bootsel)
            pimpl().logf("[board] BOOTSEL on %s%s%s", b.drive.c_str(),
                        b.program.empty() ? "" : "  running: ",
                        b.program.c_str());
        else
            pimpl().logf("[board] %s running on %s",
                        b.chip.empty() ? "board" : b.chip.c_str(),
                        b.port.empty() ? "(no serial port)" : b.port.c_str());

        pimpl().boardQuerying.store(false);
    }).detach();
}

// ------------------------------------------------------------ operations ----

Void PicoFlash::build(const Str& id)
{
    const Str root = repoRoot();
    if(root.empty()) { pimpl().log("[error] repo root not found"); return; }

    Str name = id;
    Bool        can  = false;
    {
        std::lock_guard<std::mutex> lk(pimpl().mu);
        if(const FirmwareEntry* e = findEntry(pimpl().catalog, id))
        {
            name = e->name;
            can  = e->buildable;
        }
    }
    if(!can)
    {
        pimpl().logf("[error] %s is not buildable from this repo - "
                    "there is no source for it here", id.c_str());
        return;
    }

    const Str bat = root + "\\firmware\\build.bat";
    if(!fileExists(bat)) { pimpl().logf("[error] missing %s", bat.c_str()); return; }

    const Str desc = "building " + name;
    pimpl().start(desc, [root, bat]()
    {
        // build.bat takes no target: it builds what firmware/CMakeLists.txt
        // defines. Any future second target belongs in the script, not here.
        const Str cmd = "cmd.exe /s /c \"" + quote(bat) + "\"";
        return runCapture(cmd, root, [](const Str& l) { pimpl().log(l); });
    });
}

Void PicoFlash::flash(const Str& id)
{
    const Str root = repoRoot();
    if(root.empty()) { pimpl().log("[error] repo root not found"); return; }

    Str uf2, name = id;
    Bool        present = false;
    {
        std::lock_guard<std::mutex> lk(pimpl().mu);
        if(const FirmwareEntry* e = findEntry(pimpl().catalog, id))
        {
            uf2     = e->uf2Path;
            name    = e->name;
            present = e->present;
        }
    }
    if(uf2.empty()) { pimpl().logf("[error] no catalog entry \"%s\"", id.c_str()); return; }

    // Re-stat rather than trusting the cached flag: the catalog may have been
    // scanned minutes ago and this is the destructive one.
    if(!present || !fileExists(uf2))
    {
        pimpl().logf("[error] %s does not exist - build it first", uf2.c_str());
        return;
    }

    const Str bat = root + "\\firmware\\flash.bat";
    if(!fileExists(bat)) { pimpl().logf("[error] missing %s", bat.c_str()); return; }

    const Str desc = "flashing " + name;
    pimpl().start(desc, [root, bat, uf2]()
    {
        const Str cmd = "cmd.exe /s /c \"" + quote(bat) + " " + quote(uf2) + "\"";
        return runCapture(cmd, root, [](const Str& l) { pimpl().log(l); });
    });
}

Void PicoFlash::backup(const Str& outPath)
{
    const Str root = repoRoot();
    if(root.empty()) { pimpl().log("[error] repo root not found"); return; }
    if(outPath.empty()) { pimpl().log("[error] no backup filename given"); return; }

    const Str out = toBackslashes(outPath);

    // A backup is only worth taking if it cannot destroy the previous one. The
    // .uf2 read off this board before it was first reflashed is the ONLY copy of
    // that firmware on this machine - its source is on another computer - so
    // overwriting is refused outright rather than confirmed.
    if(fileExists(out))
    {
        pimpl().logf("[error] %s already exists - refusing to overwrite a backup. "
                    "Pick another name.", out.c_str());
        return;
    }

    const Str bat = root + "\\firmware\\backup.bat";
    if(!fileExists(bat)) { pimpl().logf("[error] missing %s", bat.c_str()); return; }

    const Str desc = "backing up flash to " + out;
    pimpl().start(desc, [root, bat, out]()
    {
        const Str cmd = "cmd.exe /s /c \"" + quote(bat) + " " + quote(out) + "\"";
        return runCapture(cmd, root, [](const Str& l) { pimpl().log(l); });
    });
}

// ---------------------------------------------------------------- reboot ----
// picotool's own job, and the one place it is used for something other than a
// query. `-f` forces a board that is running firmware to accept the request;
// without it picotool declines and prints the "consider -f" note.

Void PicoFlash::rebootBootsel()
{
    const Str root = repoRoot();
    if(root.empty()) { pimpl().log("[error] repo root not found"); return; }

    const Str picotool = root + "\\vendor\\picotool-2.3.0\\picotool\\picotool.exe";
    if(!fileExists(picotool)) { pimpl().logf("[error] missing %s", picotool.c_str()); return; }

    pimpl().start("rebooting into BOOTSEL", [root, picotool]()
    {
        Str already;
        if(findBootselDrive(already))
        {
            pimpl().logf("[skip ] already in BOOTSEL, mounted as %s", already.c_str());
            return 0;
        }

        // -f: the board is running firmware, so picotool has to ask it over the
        // SDK's reset interface rather than talking to a bootloader.
        const Int32 rc = runCapture(quote(picotool) + " reboot -f -u", root,
                                   [](const Str& l) { pimpl().log(l); });
        if(rc == 0)
        {
            // The drive takes a moment to mount; without this the status the UI
            // refreshes to straight afterwards is the pre-reboot one.
            for(Int32 i = 0; i < 40; ++i)
            {
                Str d;
                if(findBootselDrive(d))
                {
                    pimpl().logf("[ok   ] bootloader mounted as %s", d.c_str());
                    break;
                }
                Sleep(250);
            }
        }
        return rc;
    });
}

Void PicoFlash::rebootNormal()
{
    const Str root = repoRoot();
    if(root.empty()) { pimpl().log("[error] repo root not found"); return; }

    const Str picotool = root + "\\vendor\\picotool-2.3.0\\picotool\\picotool.exe";
    if(!fileExists(picotool)) { pimpl().logf("[error] missing %s", picotool.c_str()); return; }

    pimpl().start("rebooting into the application", [root, picotool]()
    {
        // A board in the bootloader is already talking to picotool directly, so
        // -f (which means "force a running board into the bootloader first") is
        // both unnecessary and rejected there.
        Str drive;
        const Str args = findBootselDrive(drive) ? " reboot" : " reboot -f";

        const Int32 rc = runCapture(quote(picotool) + args, root,
                                   [](const Str& l) { pimpl().log(l); });
        if(rc == 0) Sleep(1500);   // let USB re-enumerate before the status refresh
        return rc;
    });
}
