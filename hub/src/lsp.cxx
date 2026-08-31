// See lsp.hxx.

#include "lsp.hxx"

#include "json.hxx"
#include "pico_flash.hxx"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace lsp
{
  namespace
  {

    // Where clangd usually is on this machine, in the order worth trying.
    //
    // The PATH lookup is second, not first: a PATH clangd could be any version
    // and any bitness, whereas the LLVM installer's path is the one that came
    // with the toolchain the firmware is actually built with.
    constexpr const Char* const CLANGD_PATHS[] = {
        "C:\\Program Files\\LLVM\\bin\\clangd.exe",
        "C:\\Program Files (x86)\\LLVM\\bin\\clangd.exe",
    };

    // More than the popup shows, on purpose. The popup displays ten and scrolls;
    // holding a few hundred means the local filter below has something to filter
    // as the user keeps typing, instead of an empty list and a round trip.
    constexpr Size MAX_ITEMS = 400;

    // How long after the file's first diagnostics before the first question is
    // worth asking. See Impl::parsedAtMs - 500 ms was where the answer went
    // from 5 items to 20 on this machine, and this is that with margin.
    constexpr UInt64 SETTLE_MS = 750;

    // ------------------------------------------------------------- plumbing ---

    struct Pipe
    {
        HANDLE rd = nullptr;
        HANDLE wr = nullptr;
    };

    Void closeHandle(HANDLE& h)
    {
        if(h != nullptr && h != INVALID_HANDLE_VALUE)
        {
            CloseHandle(h);
        }
        h = nullptr;
    }

    // file:///C%3A/Users/... - the spelling clangd's own logs use.
    //
    // The colon after the drive letter MUST be escaped. clangd tolerates the
    // unescaped form for didOpen and then hands back completion items keyed to
    // the escaped one, and the mismatch shows up as a document that is open and
    // has no completions - which reads exactly like a parse failure.
    Str toUri(const Str& path)
    {
        Str out = "file:///";
        for(const Char c : path)
        {
            if(c == '\\')
            {
                out.push_back('/');
            }
            else if(c == ':')
            {
                out += "%3A";
            }
            else if(c == ' ')
            {
                out += "%20";
            }
            else
            {
                out.push_back(c);
            }
        }
        return out;
    }

    // Whether two URIs name the same file.
    //
    // NOT strcmp, and this is not theoretical. We send
    // `file:///C%3A/Users/...` because that is the form clangd's own logs use,
    // and clangd sends `file:///C:/Users/...` back in its notifications. Both
    // are the same file and the same server; only the escaping of one colon
    // differs. Comparing them literally means the "the AST is ready" signal
    // never matches the document it is about, and the completion popup stays
    // permanently on its fallback - working, plausible, and never once asking
    // the language server.
    Bool sameUri(const Str& a, const Str& b)
    {
        Size i = 0;
        Size j = 0;
        while(i < a.size() && j < b.size())
        {
            Char x = a[i];
            Char y = b[j];

            if(x == '%' && i + 2 < a.size()
               && a[i + 1] == '3' && (a[i + 2] == 'A' || a[i + 2] == 'a'))
            {
                x = ':';
                i += 2;
            }
            if(y == '%' && j + 2 < b.size()
               && b[j + 1] == '3' && (b[j + 2] == 'A' || b[j + 2] == 'a'))
            {
                y = ':';
                j += 2;
            }

            // Case-insensitively, because Windows paths are and clangd may hand
            // back a different capitalisation of the drive than we sent.
            if(std::tolower(static_cast<UInt8>(x)) != std::tolower(static_cast<UInt8>(y)))
            {
                return false;
            }
            ++i;
            ++j;
        }
        return i == a.size() && j == b.size();
    }

    // LSP CompletionItemKind -> the popup's four-and-two.
    //
    // The mapping is lossy and deliberately so: the protocol has 25 kinds and a
    // popup that shows 25 different two-letter tags is a popup nobody reads.
    // What matters to somebody typing is "is this a call, a name, or a field".
    cmpl::Kind kindOf(Int32 lspKind)
    {
        switch(lspKind)
        {
        case 2:    // Method
        case 3:    // Function
        case 4:    // Constructor
        case 24:   // Operator
            return cmpl::Kind::KIND_FUNCTION;

        case 7:    // Class
        case 8:    // Interface
        case 13:   // Enum
        case 22:   // Struct
        case 25:   // TypeParameter
            return cmpl::Kind::KIND_TYPE;

        case 20:   // EnumMember
        case 21:   // Constant
            return cmpl::Kind::KIND_MACRO;

        case 14:   // Keyword
        case 15:   // Snippet
            return cmpl::Kind::KIND_KEYWORD;

        case 5:    // Field
        case 10:   // Property
            return cmpl::Kind::KIND_FIELD;

        case 6:    // Variable
        case 9:    // Module
        case 12:   // Value
        default:
            return cmpl::Kind::KIND_VARIABLE;
        }
    }

    // What to actually type when an item is accepted.
    //
    // clangd's `label` is for HUMANS: it carries a leading decoration character
    // (a bullet, not ASCII) and often a trailing signature, and inserting it
    // verbatim puts a bullet in the source. `filterText` is the plain
    // identifier and is what belongs in the buffer.
    //
    // insertText is ignored on purpose. With snippet support declared off,
    // clangd sends it as plain text, but it still includes the parenthesis for
    // a function - and a completion that types the `(` for you is a completion
    // that fights you every time you wanted the function's address or a
    // designated initializer's name.
    Str insertionFor(const js::Value& item)
    {
        const Str filter = item.at("filterText").string();
        if(!filter.empty())
        {
            return filter;
        }

        // No filterText: strip the decoration off the label by hand. Everything
        // before the first identifier character is ornament.
        const Str label = item.at("label").string();
        Size      begin = 0;
        while(begin < label.size())
        {
            const Char c = label[begin];
            const Bool isIdent = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
                              || (c >= '0' && c <= '9') || c == '_' || c == '~';
            if(isIdent)
            {
                break;
            }
            ++begin;
        }

        Size end = begin;
        while(end < label.size())
        {
            const Char c = label[end];
            const Bool isIdent = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
                              || (c >= '0' && c <= '9') || c == '_';
            if(!isIdent)
            {
                break;
            }
            ++end;
        }
        return label.substr(begin, end - begin);
    }

    // The first line of the documentation, and only the first.
    //
    // A doc comment on a firmware header can be forty lines - pins.hxx's are -
    // and the popup has one row for it. The first line of a well-written
    // comment is the summary, which is the only part that fits anyway.
    Str firstLine(const js::Value& doc)
    {
        Str text = doc.string();
        if(text.empty() && doc.isObject())
        {
            text = doc.at("value").string();   // MarkupContent
        }

        if(const Size nl = text.find('\n'); nl != Str::npos)
        {
            text.resize(nl);
        }
        while(!text.empty() && (text.back() == ' ' || text.back() == '\r'))
        {
            text.pop_back();
        }
        return text;
    }

    // ------------------------------------------------------------------ state --

    struct Impl
    {
        // ---- the child ----
        HANDLE proc    = nullptr;
        HANDLE toChild = nullptr;    // our end of its stdin
        HANDLE frChild = nullptr;    // our end of its stdout

        Thread reader;
        Thread writer;

        Atomic<Bool>  running{false};
        Atomic<State> st{State::STATE_OFF};

        mutable Mutex noteMu;
        Str           note = "clangd: not started";

        // ---- the outbound queue ----
        Mutex     outMu;
        CondVar   outCv;
        Deque<Str> outbox;

        // ---- what we have asked and heard ----
        Atomic<Int64>  inFlight{-1};    // request id, or -1
        Atomic<UInt64> nextId{2};       // 1 is initialize

        // Whether clangd has finished building an AST for the open document.
        //
        // THIS IS NOT PEDANTRY, it is the difference between the feature working
        // and the feature being noise. Asked before the preamble is built,
        // clangd answers anyway - from an identifier index, not the AST - and
        // `dfplayer::` comes back with `printf`, `define` and `serial` in it.
        // Fifty-seven plausible wrong answers rather than twenty right ones, and
        // nothing in the reply says which kind it is.
        //
        // The signal is publishDiagnostics for our URI, which clangd sends when
        // the AST is done and sends even when there is nothing to report.
        Atomic<Bool> parsed{false};

        // When that happened, because being parsed is not quite being ready.
        //
        // MEASURED, not guessed. Asking at the instant the diagnostics arrive
        // gives 5 items for `dfplayer::`; a quarter of a second later, still 5;
        // half a second later, all 20. The AST is built by the time diagnostics
        // are published but the preamble index is not, and clangd answers from
        // what it has rather than waiting or saying so.
        //
        // So the first question waits out the gap. Nothing is lost by it: the
        // hand-written table answers during those few hundred milliseconds, and
        // they are spent on the keystrokes right after a file is opened.
        Atomic<UInt64> parsedAtMs{0};

        mutable Mutex uriMu;    // openUri is written by ask(), read by the reader
        Str           openUri;

        Mutex  answerMu;
        Answer answer;                  // serial 0 until the first reply
        Bool   answerFresh = false;

        // clangd's diagnostics for the open file. Replaced wholesale on every
        // publish, because clangd sends the complete set each time and an empty
        // array means "clean now" rather than "nothing to say".
        Mutex           diagMu;
        Vec<diag::Item> diags;
        Bool            diagsFresh = false;

        UInt64 sentVersion = 0;         // the document version clangd has
        Str    openPath;                // and which file it is
        Int32  docVersion = 1;          // LSP's own counter, must increase

        Void say(const Str& s)
        {
            LockGuard<Mutex> lk(noteMu);
            note = s;
        }

        Void sayf(const Char* fmt, ...)
        {
            Array<Char, 512> buf;
            va_list ap;
            va_start(ap, fmt);
            std::vsnprintf(buf.data(), buf.size(), fmt, ap);
            va_end(ap);
            say(buf.data());
        }

        // Queues one JSON-RPC message. The framing is added here so no caller
        // has to remember it.
        Void send(const Str& body)
        {
            Array<Char, 64> head;
            std::snprintf(head.data(), head.size(),
                          "Content-Length: %llu\r\n\r\n",
                          static_cast<unsigned long long>(body.size()));

            {
                LockGuard<Mutex> lk(outMu);
                outbox.push_back(Str(head.data()) + body);
            }
            outCv.notify_one();
        }
    };

    Impl& impl()
    {
        static Impl s;
        return s;
    }

    // ------------------------------------------------------------- the threads --

    Void writerLoop()
    {
        Impl& s = impl();

        while(true)
        {
            Str msg;
            {
                UniqueLock<Mutex> lk(s.outMu);
                s.outCv.wait(lk, [&s]()
                {
                    return !s.outbox.empty() || !s.running.load();
                });

                if(!s.running.load() && s.outbox.empty())
                {
                    return;
                }
                msg = s.outbox.front();
                s.outbox.pop_front();
            }

            // A partial write is possible on a pipe and is not an error; loop
            // until the whole frame is out or the pipe dies. Half a frame is
            // worse than none: clangd would then read the next message's bytes
            // as this one's body and never resynchronise.
            Size written = 0;
            while(written < msg.size())
            {
                DWORD      got = 0;
                const BOOL ok  = WriteFile(s.toChild, msg.data() + written,
                                           static_cast<DWORD>(msg.size() - written),
                                           &got, nullptr);
                if(!ok || got == 0)
                {
                    return;   // the child is gone; readerLoop reports it
                }
                written += got;
            }
        }
    }

    // Reads exactly `n` bytes, or returns false because the pipe closed.
    Bool readExact(HANDLE h, Char* into, Size n)
    {
        Size got = 0;
        while(got < n)
        {
            DWORD chunk = 0;
            if(!ReadFile(h, into + got, static_cast<DWORD>(n - got), &chunk, nullptr)
               || chunk == 0)
            {
                return false;
            }
            got += chunk;
        }
        return true;
    }

    // One header line, up to and including the \r\n, which is discarded.
    //
    // Byte at a time, which looks wasteful and is not: header lines are short,
    // and the alternative is buffering ahead of the body and then having to
    // hand the leftovers to readExact. That bookkeeping is where framing bugs
    // live.
    Bool readLine(HANDLE h, Str& out)
    {
        out.clear();
        for(;;)
        {
            Char  c     = 0;
            DWORD chunk = 0;
            if(!ReadFile(h, &c, 1, &chunk, nullptr) || chunk == 0)
            {
                return false;
            }
            if(c == '\n')
            {
                if(!out.empty() && out.back() == '\r')
                {
                    out.pop_back();
                }
                return true;
            }
            out.push_back(c);
        }
    }

    Void handle(const js::Value& msg);

    Void readerLoop()
    {
        Impl& s = impl();

        while(s.running.load())
        {
            // ---- framing ----
            Size length = 0;
            Bool sawLen = false;

            for(;;)
            {
                Str line;
                if(!readLine(s.frChild, line))
                {
                    s.running.store(false);
                    if(s.st.load() != State::STATE_OFF)
                    {
                        s.st.store(State::STATE_FAILED);
                        s.say("clangd: the process closed its output");
                    }
                    s.outCv.notify_one();   // let the writer go
                    return;
                }

                if(line.empty())
                {
                    break;             // end of headers
                }

                // Case-insensitive: the spec says the header name is fixed, but
                // reading it strictly is free and misreading it is silent.
                if(line.size() > 15 && _strnicmp(line.c_str(), "Content-Length:", 15) == 0)
                {
                    length = static_cast<Size>(std::strtoull(line.c_str() + 15,
                                                             nullptr, 10));
                    sawLen = true;
                }
            }

            if(!sawLen || length == 0 || length > (64u * 1024u * 1024u))
            {
                // No length, or an absurd one. Cannot resynchronise a stream
                // whose framing is wrong, so stop rather than read garbage.
                s.running.store(false);
                s.st.store(State::STATE_FAILED);
                s.sayf("clangd: bad frame header (length %llu)",
                       static_cast<unsigned long long>(length));
                s.outCv.notify_one();
                return;
            }

            Str body;
            body.resize(length);
            if(!readExact(s.frChild, body.data(), length))
            {
                s.running.store(false);
                s.st.store(State::STATE_FAILED);
                s.say("clangd: the process closed mid-message");
                s.outCv.notify_one();
                return;
            }

            // Every frame clangd sends, to a file, when BIBO_LSP_DUMP is set.
            //
            // Kept because it is what settled the one bug this file had that
            // could not be reasoned about: the popup was showing 5 completions
            // where a Python probe of the same server got 20, and the only way
            // to know whether the client was dropping items or the server was
            // sending five was to look at the bytes. It was sending five.
            //
            // Off unless asked for, and writes only into the working directory.
            if(std::getenv("BIBO_LSP_DUMP") != nullptr)
            {
                static Int32 nth = 0;
                Array<Char, 64> name;
                std::snprintf(name.data(), name.size(), "lspdump_%03d.json", nth++);
                std::FILE* f = std::fopen(name.data(), "wb");
                if(f != nullptr)
                {
                    std::fwrite(body.data(), 1, body.size(), f);
                    std::fclose(f);
                }
            }

            // ---- the message ----
            Bool      ok      = false;
            Size      stopped = 0;
            js::Value msg     = js::parse(body, ok, stopped);
            if(!ok)
            {
                // Report it and carry on: one unreadable message is not a
                // reason to lose the session, and the framing is still sound
                // because the length told us exactly where it ended.
                s.sayf("clangd: unreadable reply at byte %llu",
                       static_cast<unsigned long long>(stopped));
                continue;
            }

            handle(msg);
        }
    }

    Void handle(const js::Value& msg)
    {
        Impl& s = impl();

        // A request FROM clangd - it has both an id and a method. Answer it,
        // even with null: clangd blocks waiting for a reply to one it sent, and
        // a server waiting on us looks exactly like a server that is slow.
        if(!msg.at("method").isNull() && !msg.at("id").isNull())
        {
            Array<Char, 96> buf;
            std::snprintf(buf.data(), buf.size(),
                          "{\"jsonrpc\":\"2.0\",\"id\":%d,\"result\":null}",
                          msg.at("id").integer());
            s.send(buf.data());
            return;
        }

        // A notification: diagnostics, logs, progress.
        if(msg.at("id").isNull())
        {
            // The one that matters. Not for the diagnostics themselves - the
            // build's are better and diagnostics.cxx already has them - but
            // because its arrival is clangd saying the AST is ready, which is
            // the earliest moment a completion is worth asking for.
            if(msg.at("method").string() == "textDocument/publishDiagnostics")
            {
                Str mine;
                Str minePath;
                {
                    LockGuard<Mutex> lk(s.uriMu);
                    mine     = s.openUri;
                    minePath = s.openPath;
                }
                if(!mine.empty() && sameUri(msg.at("params").at("uri").string(), mine))
                {
                    if(!s.parsed.load())
                    {
                        s.parsedAtMs.store(GetTickCount64());
                        s.parsed.store(true);
                    }

                    // The payload, which used to be dropped on the floor. Every
                    // parse error clangd found, and every clang-tidy check a
                    // .clangd turned on, arrives here and nowhere else.
                    const js::Value& arr = msg.at("params").at("diagnostics");
                    Vec<diag::Item>  got;

                    for(Size i = 0; i < arr.size(); ++i)
                    {
                        const js::Value& d = arr[i];

                        diag::Item it;
                        it.file = minePath;

                        // LSP counts lines and characters from ZERO; every
                        // compiler this project talks to counts from one, and
                        // diag::Item is documented as holding what a compiler
                        // said. Off by one here puts the underline on the line
                        // above the mistake, which is worse than no underline.
                        it.line   = d.at("range").at("start").at("line").integer(0) + 1;
                        it.column = d.at("range").at("start").at("character").integer(0) + 1;

                        // LSP severity: 1 error, 2 warning, 3 information,
                        // 4 hint. Anything softer than a warning is a note.
                        const Int32 sev = d.at("severity").integer(2);
                        it.severity = (sev == 1) ? diag::Severity::SEVERITY_ERR
                                    : (sev == 2) ? diag::Severity::SEVERITY_WARN
                                                 : diag::Severity::SEVERITY_NOTE;

                        it.message = d.at("message").string();

                        // Which check fired, appended the way clang-tidy itself
                        // prints it. Without this a tidy finding is
                        // indistinguishable from a compiler warning, and the
                        // two are fixed differently - one is a mistake, the
                        // other is a house rule with a name you can look up.
                        const Str code = d.at("code").string();
                        if(!code.empty())
                        {
                            it.message += " [" + code + "]";
                        }

                        got.push_back(it);
                    }

                    LockGuard<Mutex> lk(s.diagMu);
                    s.diags      = got;
                    s.diagsFresh = true;
                }
            }
            return;
        }

        const Int32 id = msg.at("id").integer(-1);

        // ---- the handshake ----
        if(id == 1)
        {
            const js::Value& caps = msg.at("result").at("capabilities");
            if(caps.at("completionProvider").isNull())
            {
                s.st.store(State::STATE_FAILED);
                s.say("clangd: started but offers no completion");
                return;
            }

            s.send("{\"jsonrpc\":\"2.0\",\"method\":\"initialized\",\"params\":{}}");
            s.st.store(State::STATE_READY);
            s.say("clangd: ready");
            return;
        }

        // ---- a completion reply ----
        if(static_cast<Int64>(id) != s.inFlight.load())
        {
            return;   // an answer to a question the caret has already left
        }
        s.inFlight.store(-1);

        // The result is either a CompletionList (an object with `items`) or a
        // bare array. clangd sends the first; the protocol allows both.
        const js::Value& result = msg.at("result");
        const js::Value& items  = result.isArray() ? result : result.at("items");

        Answer built;
        built.items.reserve(items.size() < MAX_ITEMS ? items.size() : MAX_ITEMS);

        for(Size i = 0; i < items.size() && built.items.size() < MAX_ITEMS; ++i)
        {
            const js::Value& it = items[i];

            Item out;
            out.name = insertionFor(it);
            if(out.name.empty())
            {
                continue;
            }

            out.detail = it.at("detail").string();
            out.doc    = firstLine(it.at("documentation"));
            out.kind   = kindOf(it.at("kind").integer(6));

            built.items.push_back(std::move(out));
        }

        {
            LockGuard<Mutex> lk(s.answerMu);
            built.serial = s.answer.serial + 1u;
            built.path   = s.answer.path;    // set by ask(), see below
            built.line   = s.answer.line;
            built.col    = s.answer.col;
            s.answer      = std::move(built);
            s.answerFresh = true;
        }
    }

    // ------------------------------------------------------------- starting ---

    // The compiler the build actually invokes, read out of the first entry of
    // compile_commands.json.
    //
    // WHAT THIS IS FOR. clangd sees `arm-none-eabi-g++` in the compile command
    // and targets arm-none-eabi, but it does NOT ask that compiler where its
    // headers are - running an executable named by a file in the workspace is a
    // thing it refuses to do unless told which ones are allowed. Without being
    // told, `#include <cstddef>` does not resolve, types.hxx fails, and every
    // Int32 in the firmware is an unknown type name.
    //
    // The visible symptom is not an error, which is what makes it worth this
    // much code: the file still "parses", clangd still answers, and the answers
    // come from an identifier index instead of the AST. `dfplayer::` returns
    // `printf`, `define` and `serial` - twenty right answers replaced by
    // fifty-seven plausible wrong ones, with nothing in the reply to say so.
    //
    // The EXACT path, not a glob over the directory. --query-driver makes
    // clangd execute what it matches, so the only thing worth allowing is the
    // one compiler this project's own build already runs.
    Str driverFromDatabase(const Str& ccPath)
    {
        std::FILE* f = std::fopen(ccPath.c_str(), "rb");
        if(f == nullptr)
        {
            return Str();
        }

        // The first entry is at the top; a few tens of kilobytes is plenty and
        // the whole file can be tens of megabytes.
        Str head;
        head.resize(64u * 1024u);
        const Size got = std::fread(head.data(), 1, head.size(), f);
        std::fclose(f);
        head.resize(got);

        // Both spellings: CMake writes "command", some generators write
        // "arguments" as an array whose first element is the driver.
        Size at = head.find("\"command\"");
        if(at != Str::npos)
        {
            at = head.find('"', at + 9);        // the value's opening quote
            if(at != Str::npos)
            {
                ++at;
            }
        }
        else
        {
            at = head.find("\"arguments\"");
            if(at != Str::npos)
            {
                at = head.find('"', head.find('[', at) + 1);
                if(at != Str::npos)
                {
                    ++at;
                }
            }
        }

        if(at == Str::npos)
        {
            return Str();
        }

        // The first token, unescaping as we go. The driver may itself be quoted
        // inside the JSON string when its path has spaces, which is why the
        // \" case both unescapes and toggles.
        Str  out;
        Bool quoted = false;
        while(at < head.size())
        {
            const Char c = head[at];

            if(c == '\\' && at + 1 < head.size())
            {
                const Char e = head[at + 1];
                if(e == '\\')
                {
                    out.push_back('\\');
                    at += 2;
                    continue;
                }
                if(e == '"')
                {
                    if(out.empty())
                    {
                        quoted = true;      // an opening quote around the path
                    }
                    else if(quoted)
                    {
                        break;              // and its closing one
                    }
                    at += 2;
                    continue;
                }
            }

            if(c == '"')
            {
                break;                      // end of the JSON string entirely
            }
            if(c == ' ' && !quoted)
            {
                break;                      // end of the first token
            }

            out.push_back(c);
            ++at;
        }

        // Only if it is really there. A stale database naming a compiler that
        // has been uninstalled would otherwise hand clangd a path to run.
        if(out.empty() || GetFileAttributesA(out.c_str()) == INVALID_FILE_ATTRIBUTES)
        {
            return Str();
        }
        return out;
    }

    // The clangd to run, or empty with `whyNot` explaining.
    Str findClangd(Str& whyNot)
    {
        for(const Char* p : CLANGD_PATHS)
        {
            if(GetFileAttributesA(p) != INVALID_FILE_ATTRIBUTES)
            {
                return Str(p);
            }
        }

        Array<Char, MAX_PATH> found = {};
        if(SearchPathA(nullptr, "clangd.exe", nullptr,
                       static_cast<DWORD>(found.size()), found.data(), nullptr) != 0)
        {
            return Str(found.data());
        }

        whyNot = "clangd: not installed - looked in Program Files\\LLVM\\bin and on PATH";
        return Str();
    }

  }

  // ------------------------------------------------------------------ public --

  Bool start()
  {
      Impl& s = impl();
      if(s.running.load())
      {
          return s.st.load() != State::STATE_FAILED;
      }

      Str whyNot;
      const Str exe = findClangd(whyNot);
      if(exe.empty())
      {
          s.st.store(State::STATE_FAILED);
          s.say(whyNot);
          return false;
      }

      const Str root   = PicoFlash::repoRoot();
      const Str ccDir  = root + "\\firmware\\build";
      const Str ccPath = ccDir + "\\compile_commands.json";

      const Str driver = driverFromDatabase(ccPath);

      if(GetFileAttributesA(ccPath.c_str()) == INVALID_FILE_ATTRIBUTES)
      {
          // Not fatal. clangd still completes from the file's own includes; it
          // just cannot see the SDK's flags, so half the firmware headers will
          // not resolve. Worth saying out loud rather than looking broken.
          s.say("clangd: no compile_commands.json - run firmware\\build.bat once");
      }

      SECURITY_ATTRIBUTES sa{};
      sa.nLength        = sizeof(sa);
      sa.bInheritHandle = TRUE;

      Pipe in;    // parent writes in.wr  -> child reads in.rd  as stdin
      Pipe out;   // child writes out.wr  -> parent reads out.rd as stdout

      if(!CreatePipe(&in.rd, &in.wr, &sa, 0) || !CreatePipe(&out.rd, &out.wr, &sa, 0))
      {
          closeHandle(in.rd);
          closeHandle(in.wr);
          closeHandle(out.rd);
          closeHandle(out.wr);
          s.st.store(State::STATE_FAILED);
          s.sayf("clangd: CreatePipe failed (%lu)",
                 static_cast<unsigned long>(GetLastError()));
          return false;
      }

      // The parent's ends must NOT be inherited, or the child holds a copy and
      // the reader never sees EOF when clangd exits.
      SetHandleInformation(in.wr,  HANDLE_FLAG_INHERIT, 0);
      SetHandleInformation(out.rd, HANDLE_FLAG_INHERIT, 0);

      STARTUPINFOA si{};
      si.cb          = sizeof(si);
      si.dwFlags     = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
      si.wShowWindow = SW_HIDE;
      si.hStdInput   = in.rd;
      si.hStdOutput  = out.wr;
      si.hStdError   = nullptr;   // clangd's log; we do not read it, so drop it

      // --log=error, not the default: clangd's info log is a few hundred lines
      // per parse and it all goes to a handle nobody drains, which eventually
      // blocks the process that is writing it.
      Str cmd = "\"" + exe + "\""
              + " --compile-commands-dir=\"" + ccDir + "\""
              + " --background-index=false"
              + " --header-insertion=never"
              + " --limit-results=" + std::to_string(MAX_ITEMS)
              + " --log=error";

      if(!driver.empty())
      {
          cmd += " \"--query-driver=" + driver + "\"";
      }

      Vec<Char> mutableCmd(cmd.begin(), cmd.end());
      mutableCmd.push_back('\0');

      PROCESS_INFORMATION pi{};
      const BOOL ok = CreateProcessA(nullptr, mutableCmd.data(), nullptr, nullptr,
                                     TRUE, CREATE_NO_WINDOW, nullptr,
                                     root.c_str(), &si, &pi);

      closeHandle(in.rd);    // the child's ends; the parent must not hold them
      closeHandle(out.wr);

      if(!ok)
      {
          const DWORD err = GetLastError();
          closeHandle(in.wr);
          closeHandle(out.rd);
          s.st.store(State::STATE_FAILED);
          s.sayf("clangd: could not start %s (%lu)", exe.c_str(),
                 static_cast<unsigned long>(err));
          return false;
      }

      CloseHandle(pi.hThread);
      s.proc    = pi.hProcess;
      s.toChild = in.wr;
      s.frChild = out.rd;

      // A restart must not inherit the previous session's unsent tail, or the
      // first thing the new clangd reads is the middle of a conversation it
      // was not part of.
      {
          LockGuard<Mutex> lk(s.outMu);
          s.outbox.clear();
      }
      {
          LockGuard<Mutex> lk(s.answerMu);
          s.answer      = Answer();
          s.answerFresh = false;
      }
      s.inFlight.store(-1);
      s.openPath.clear();
      s.sentVersion = 0;
      s.docVersion  = 1;

      s.running.store(true);
      s.st.store(State::STATE_STARTING);
      s.say("clangd: starting");

      s.reader = Thread(readerLoop);
      s.writer = Thread(writerLoop);

      const Str rootUri = toUri(root);
      s.send("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{"
             "\"processId\":null,"
             "\"rootUri\":" + js::quote(rootUri) + ","
             "\"capabilities\":{\"textDocument\":{\"completion\":{"
               "\"completionItem\":{"
                 // Snippets OFF. clangd would otherwise send `${1:x}` placeholder
                 // syntax, which this editor has no machinery to expand and would
                 // insert literally.
                 "\"snippetSupport\":false,"
                 "\"documentationFormat\":[\"plaintext\"]}}}}}}");

      return true;
  }

  Void stop()
  {
      Impl& s = impl();
      if(!s.running.load() && !s.reader.joinable() && !s.writer.joinable())
      {
          return;
      }

      s.running.store(false);
      s.outCv.notify_one();

      // Close our write end first: clangd sees EOF on stdin and exits cleanly,
      // which closes its stdout and unblocks the reader's ReadFile. Terminating
      // it outright would leave the reader parked on a pipe with a live handle
      // at the other end.
      closeHandle(s.toChild);

      if(s.proc != nullptr && WaitForSingleObject(s.proc, 2000) == WAIT_TIMEOUT)
      {
          TerminateProcess(s.proc, 1);
      }

      if(s.writer.joinable())
      {
          s.writer.join();
      }
      if(s.reader.joinable())
      {
          s.reader.join();
      }

      closeHandle(s.frChild);
      closeHandle(s.proc);

      s.st.store(State::STATE_OFF);
      s.say("clangd: stopped");
  }

  State state()
  {
      return impl().st.load();
  }

  Str status()
  {
      Impl&            s = impl();
      LockGuard<Mutex> lk(s.noteMu);
      return s.note;
  }

  Bool busy()
  {
      return impl().inFlight.load() >= 0;
  }

  Bool diagnostics(Vec<diag::Item>& out)
  {
      Impl& s = impl();

      LockGuard<Mutex> lk(s.diagMu);
      if(!s.diagsFresh)
      {
          return false;
      }

      // Handed over even when EMPTY, and that is the point: clangd publishes a
      // complete set every time, so an empty one means the file just became
      // clean. Returning false there would leave the last errors underlined
      // after they were fixed.
      out          = s.diags;
      s.diagsFresh = false;
      return true;
  }

  Bool ask(const Str& path, const Str& text, UInt64 version, Int32 line, Int32 col)
  {
      Impl& s = impl();
      if(s.st.load() != State::STATE_READY || path.empty())
      {
          return false;
      }

      const Str uri = toUri(path);

      // ---- keep clangd's copy of the file current ----
      //
      // Full-text sync rather than incremental ranges. Incremental is what a
      // serious editor does and it is also where the subtle bugs are: a single
      // dropped or misordered change and the server's copy silently diverges
      // from ours, after which every completion is about a file that does not
      // exist. Sending the whole buffer is a few tens of kilobytes on a
      // keystroke that only happens when the text actually changed.
      if(path != s.openPath)
      {
          if(!s.openPath.empty())
          {
              s.send("{\"jsonrpc\":\"2.0\",\"method\":\"textDocument/didClose\","
                     "\"params\":{\"textDocument\":{\"uri\":"
                     + js::quote(toUri(s.openPath)) + "}}}");
          }

          s.send("{\"jsonrpc\":\"2.0\",\"method\":\"textDocument/didOpen\","
                 "\"params\":{\"textDocument\":{"
                 "\"uri\":" + js::quote(uri) + ","
                 "\"languageId\":\"cpp\","
                 "\"version\":1,"
                 "\"text\":" + js::quote(text) + "}}}");

          {
              // openPath joins openUri under this lock because the reader
              // thread needs it now: a diagnostic has to carry the PATH, and
              // reading it unguarded from there would be a race on a Str.
              LockGuard<Mutex> lk(s.uriMu);
              s.openUri  = uri;
              s.openPath = path;
          }
          s.parsed.store(false);     // a new file: nothing is built for it yet

          s.docVersion  = 1;
          s.sentVersion = version;
      }
      else if(version != s.sentVersion)
      {
          ++s.docVersion;
          s.send("{\"jsonrpc\":\"2.0\",\"method\":\"textDocument/didChange\","
                 "\"params\":{\"textDocument\":{"
                 "\"uri\":" + js::quote(uri) + ","
                 "\"version\":" + std::to_string(s.docVersion) + "},"
                 "\"contentChanges\":[{\"text\":" + js::quote(text) + "}]}}");
          s.sentVersion = version;
      }

      // ---- the question ----
      //
      // Not before the AST exists. The didOpen/didChange above still went out -
      // that is what gets the AST built - but the question waits, and the caller
      // is told it was not taken so it asks again rather than treating the
      // silence as an answer.
      if(!s.parsed.load()
         || GetTickCount64() - s.parsedAtMs.load() < SETTLE_MS)
      {
          return false;
      }

      const Int64 id = static_cast<Int64>(s.nextId.fetch_add(1));

      // Record where we asked BEFORE sending, so the reply cannot land on a
      // slot that still describes the previous question.
      {
          LockGuard<Mutex> lk(s.answerMu);
          s.answer.path = path;
          s.answer.line = line;
          s.answer.col  = col;
      }
      s.inFlight.store(id);

      Array<Char, 256> head;
      std::snprintf(head.data(), head.size(),
                    "{\"jsonrpc\":\"2.0\",\"id\":%lld,"
                    "\"method\":\"textDocument/completion\",\"params\":{"
                    "\"position\":{\"line\":%d,\"character\":%d},"
                    "\"textDocument\":{\"uri\":",
                    static_cast<long long>(id), line, col);

      s.send(Str(head.data()) + js::quote(uri) + "}}}");
      return true;
  }

  Bool take(Answer& out)
  {
      Impl&            s = impl();
      LockGuard<Mutex> lk(s.answerMu);

      if(!s.answerFresh)
      {
          return false;
      }
      out           = s.answer;
      s.answerFresh = false;
      return true;
  }

}
