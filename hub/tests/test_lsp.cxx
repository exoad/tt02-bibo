// The clangd client in hub/src/lsp.cxx, against the REAL clangd.
//
//   tests\build_lsp_test.bat run
//
// NOT A MOCK, on purpose. Everything this file could get wrong is a property of
// the actual server: whether the URI spelling matches the one clangd keys its
// replies to, whether the framing survives a 300 KB reply, whether `label`
// carries a decoration character that would end up in the buffer, whether a
// full-text didChange is enough for it to complete against an edit that has not
// been saved. A stub server would agree with whatever this client already
// believes, which is the one answer that proves nothing.
//
// So it needs clangd installed and firmware\build\compile_commands.json
// present. If either is missing this SKIPS rather than fails - a missing
// toolchain is a fact about the machine, not a defect in the code, and a test
// that cannot tell those apart teaches people to ignore it.
//
// Exits 0 on PASS or SKIP, 1 on FAIL.

#include "shared.hxx"
#include "lsp.hxx"
#include "pico_flash.hxx"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdio>

// The one thing lsp.cxx needs from the app. Defined here rather than linking
// pico_flash.cxx, which drags in the whole flashing machine for one string.
//
// DERIVED, NOT WRITTEN DOWN. The real one in pico_flash.cxx walks up from the
// executable looking for a directory that holds both firmware\ and hub\, and
// this does the same thing for the same two reasons: the tree can be moved or
// renamed without editing a test, and nobody's home directory ends up in a
// repository that is going public.
Str PicoFlash::repoRoot()
{
    Array<Char, MAX_PATH> exe = {};
    if(GetModuleFileNameA(nullptr, exe.data(), MAX_PATH) == 0)
    {
        return Str();
    }

    Str  dir(exe.data());
    Size slash = dir.find_last_of("\\/");
    if(slash == Str::npos)
    {
        return Str();
    }
    dir.resize(slash);

    const auto isDir = [](const Str& p)
    {
        const DWORD a = GetFileAttributesA(p.c_str());
        return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY) != 0;
    };

    for(Int32 up = 0; up < 12; ++up)
    {
        if(isDir(dir + "\\firmware") && isDir(dir + "\\hub"))
        {
            return dir;
        }
        slash = dir.find_last_of("\\/");
        if(slash == Str::npos || slash < 3)     // past "C:\"
        {
            break;
        }
        dir.resize(slash);
    }

    // Empty rather than a guess. main() below turns that into a SKIP with the
    // reason said out loud, which is the honest outcome for a test that cannot
    // find the tree it is supposed to be testing.
    return Str();
}

static Int32 failures = 0;
static Int32 checks   = 0;

static Void check(Bool ok, const Char* what)
{
    ++checks;
    if(ok)
    {
        std::printf("  ok    %s\n", what);
    }
    else
    {
        std::printf("  FAIL  %s\n", what);
        ++failures;
    }
}

static Str readFile(const Str& path)
{
    InFile f(path, std::ios::binary);
    if(!f)
    {
        return Str();
    }
    return Str((std::istreambuf_iterator<Char>(f)),
                std::istreambuf_iterator<Char>());
}

// One question, bundled so the helper below has a signature that fits a line.
struct Query
{
    Str    path;
    Str    text;
    UInt64 version = 0;
    Int32  line    = 0;
    Int32  col     = 0;
};

// Asks, then waits for the answer that matches the question.
//
// Waits on the SERIAL rather than on "any answer": lsp::take() hands back
// whatever landed last, and a reply to an earlier question would otherwise pass
// for this one's.
static Bool askAndWait(const Query& q, lsp::Answer& got, Int32 timeoutMs)
{
    const UInt64 before = got.serial;

    // ask() refuses until clangd has built an AST for the file, so this keeps
    // asking. The first call is what sends the document and starts that build.
    Bool sent = false;

    for(Int32 waited = 0; waited < timeoutMs; waited += 50)
    {
        if(!sent)
        {
            sent = lsp::ask(q.path, q.text, q.version, q.line, q.col);
        }
        if(sent && lsp::take(got) && got.serial != before)
        {
            return true;
        }
        sleepMs(50);
    }
    return false;
}

static Bool plainIdentifier(const Str& s)
{
    if(s.empty())
    {
        return false;
    }
    for(const Char c : s)
    {
        const Bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
                     || (c >= '0' && c <= '9') || c == '_' || c == '~';
        if(!ok)
        {
            return false;
        }
    }
    return true;
}

static Bool has(const lsp::Answer& a, const Char* name)
{
    for(const lsp::Item& it : a.items)
    {
        if(it.name == name)
        {
            return true;
        }
    }
    return false;
}

int main()
{
    std::printf("\nlsp - against the clangd on this machine\n\n");

    const Str root = PicoFlash::repoRoot();
    if(root.empty())
    {
        std::printf("  SKIP  could not find the repository root above this"
                    " executable\n\n");
        return 0;
    }
    std::printf("  root  %s\n\n", root.c_str());

    const Str file = root + "\\firmware\\sketches\\speaker.cxx";
    const Str ccj  = root + "\\firmware\\build\\compile_commands.json";

    if(readFile(ccj).empty())
    {
        std::printf("  SKIP  no firmware\\build\\compile_commands.json"
                    " - run firmware\\build.bat once\n\n");
        return 0;
    }

    const Str text = readFile(file);
    if(text.empty())
    {
        std::printf("  SKIP  could not read %s\n\n", file.c_str());
        return 0;
    }

    // ---- start ------------------------------------------------------------
    if(!lsp::start())
    {
        // Not a failure if clangd simply is not here. It IS a failure if it is
        // here and would not start, and status() is what tells them apart.
        const Str why = lsp::status();
        if(why.find("not installed") != Str::npos)
        {
            std::printf("  SKIP  %s\n\n", why.c_str());
            return 0;
        }
        std::printf("  FAIL  start(): %s\n\n", why.c_str());
        return 1;
    }

    // The handshake is a round trip to a process that has just been spawned.
    for(Int32 waited = 0; waited < 20000; waited += 50)
    {
        if(lsp::state() != lsp::State::STATE_STARTING)
        {
            break;
        }
        sleepMs(50);
    }

    check(lsp::state() == lsp::State::STATE_READY,
          "the handshake completes and clangd offers completion");
    if(lsp::state() != lsp::State::STATE_READY)
    {
        std::printf("        status: %s\n", lsp::status().c_str());
        lsp::stop();
        std::printf("\n%d checks, %d failed\n\n", checks, failures);
        return 1;
    }

    // ---- where to ask -----------------------------------------------------
    // Found by searching rather than hard-coded to a line number: this sketch
    // is edited, and a test that silently starts completing inside a comment is
    // a test that starts passing for the wrong reason.
    Vec<Str> lines;
    {
        Str cur;
        for(const Char c : text)
        {
            if(c == '\n')
            {
                if(!cur.empty() && cur.back() == '\r')
                {
                    cur.pop_back();
                }
                lines.push_back(cur);
                cur.clear();
            }
            else
            {
                cur.push_back(c);
            }
        }
        lines.push_back(cur);
    }

    Int32 nsLine = -1;
    Int32 nsCol  = -1;
    for(Size i = 0; i < lines.size(); ++i)
    {
        const Size at = lines[i].find("dfplayer::reset");
        if(at != Str::npos)
        {
            nsLine = static_cast<Int32>(i);
            nsCol  = static_cast<Int32>(at) + 10;   // just past the ::
            break;
        }
    }
    check(nsLine >= 0, "found a `dfplayer::` call site to complete at");
    if(nsLine < 0)
    {
        lsp::stop();
        std::printf("\n%d checks, %d failed\n\n", checks, failures);
        return 1;
    }

    // ---- a namespace member -----------------------------------------------
    Query first;
    first.path    = file;
    first.text    = text;
    first.version = 1;
    first.line    = nsLine;
    first.col     = nsCol;

    lsp::Answer got;
    const Bool  came = askAndWait(first, got, 40000);

    check(came, "a completion request gets an answer");
    if(!came)
    {
        std::printf("        status: %s\n", lsp::status().c_str());
        lsp::stop();
        std::printf("\n%d checks, %d failed\n\n", checks, failures);
        return 1;
    }

    check(!got.items.empty(), "and the answer has items in it");
    check(got.line == nsLine && got.col == nsCol,
          "tagged with the position it was asked about");

    std::printf("        %d item(s) at dfplayer::\n",
                static_cast<Int32>(got.items.size()));

    // The decoration character is the trap. clangd's `label` starts with a
    // non-ASCII bullet, and inserting a label verbatim puts that bullet in the
    // source - a bug that looks like a font problem for about an hour.
    Bool allPlain = true;
    for(const lsp::Item& it : got.items)
    {
        if(!plainIdentifier(it.name))
        {
            allPlain = false;
            std::printf("        not an identifier: \"%s\"\n", it.name.c_str());
            break;
        }
    }
    check(allPlain, "every insertion is a plain identifier, no decoration");

    // The real members of that namespace, which is the only proof that clangd
    // read the firmware's own headers rather than guessing from the text.
    check(has(got, "reset") && has(got, "volume") && has(got, "playMp3"),
          "the items are dfplayer's actual members");

    Bool anyDetail = false;
    for(const lsp::Item& it : got.items)
    {
        if(!it.detail.empty())
        {
            anyDetail = true;
            break;
        }
    }
    check(anyDetail, "signatures come through in `detail`");

    for(Size i = 0; i < got.items.size() && i < 6; ++i)
    {
        std::printf("          %-22s %-34s\n",
                    got.items[i].name.substr(0, 22).c_str(),
                    got.items[i].detail.substr(0, 34).c_str());
    }

    // ---- a designated initialiser, against an UNSAVED edit ----------------
    //
    // The case the whole feature was asked for. `pins::Map` has 24 fields and
    // this is where you least remember their names.
    //
    // Also the proof that full-text didChange works: the text below exists only
    // in this process's memory and has never been written to the file, so an
    // answer that names those fields can only have come from the buffer we
    // sent.
    Int32 anchor = -1;
    for(Size i = 0; i < lines.size(); ++i)
    {
        if(lines[i].find("pins::Map wiring;") != Str::npos)
        {
            anchor = static_cast<Int32>(i);
            break;
        }
    }
    check(anchor >= 0, "found the pins::Map declaration to edit near");

    if(anchor >= 0)
    {
        Vec<Str> edited = lines;
        const Str probe = "    pins::Map probe = { .so";
        edited.insert(edited.begin() + anchor + 1, probe);

        Str joined;
        for(Size i = 0; i < edited.size(); ++i)
        {
            joined += edited[i];
            if(i + 1 < edited.size())
            {
                joined.push_back('\n');
            }
        }

        lsp::Answer fields;
        fields.serial = got.serial;   // wait for something NEWER than the last

        Query second;
        second.path    = file;
        second.text    = joined;
        second.version = 2;
        second.line    = anchor + 1;
        second.col     = static_cast<Int32>(probe.size());

        const Bool got2 = askAndWait(second, fields, 40000);
        check(got2, "a second request on a changed buffer gets an answer");

        if(got2)
        {
            std::printf("        %d item(s) at { .so\n",
                        static_cast<Int32>(fields.items.size()));

            check(has(fields, "soundTx") && has(fields, "soundRx")
                  && has(fields, "soundBusy"),
                  "designated initialiser fields, from an unsaved edit");

            Bool anyField = false;
            for(const lsp::Item& it : fields.items)
            {
                if(it.kind == cmpl::Kind::KIND_FIELD)
                {
                    anyField = true;
                    break;
                }
            }
            check(anyField, "and they arrive tagged as fields");

            for(Size i = 0; i < fields.items.size() && i < 6; ++i)
            {
                std::printf("          %-22s %-34s\n",
                            fields.items[i].name.substr(0, 22).c_str(),
                            fields.items[i].detail.substr(0, 34).c_str());
            }
        }
    }

    // ---- shutting down actually ends the process --------------------------
    lsp::stop();
    check(lsp::state() == lsp::State::STATE_OFF, "stop() leaves it off");

    // And a restart works, because the Code view can be closed and reopened.
    const Bool again = lsp::start();
    check(again, "it can be started again after a stop");
    lsp::stop();

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
