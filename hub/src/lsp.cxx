// See lsp.hxx.

#include "lsp.hxx"

#include "json.hxx"
#include "pico_flash.hxx"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace lsp
{
  namespace
  {

    // Tried before the PATH lookup: a PATH clangd could be any version or
    // bitness, where the LLVM installer's came with the firmware's toolchain.
    constexpr const Char* const CLANGD_PATHS[] = {
        "C:\\Program Files\\LLVM\\bin\\clangd.exe",
        "C:\\Program Files (x86)\\LLVM\\bin\\clangd.exe",
    };

    // More than the popup's ten, so the local filter has something to filter as
    // the user keeps typing rather than needing a round trip.
    constexpr Size MAX_ITEMS = 400;

    // How long after the file's first diagnostics a question is worth asking. See
    // Impl::parsedAtMs - 500 ms was where the answer went from 5 items to 20 on
    // this machine; this is that with margin.
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

    // file:///C%3A/Users/... - the spelling clangd's own logs use. The drive
    // letter's colon MUST be escaped: clangd accepts the unescaped form for
    // didOpen, then keys its items to the escaped one.
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

    // Whether two URIs name the same file. NOT strcmp: we send
    // `file:///C%3A/Users/...` and clangd sends `file:///C:/Users/...` back.
    // Comparing literally means the "AST is ready" signal never matches its
    // document, and the popup stays permanently on its fallback.
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

            // Case-insensitive: clangd may hand back a different capitalisation of
            // the drive than we sent.
            if(std::tolower(static_cast<UInt8>(x)) != std::tolower(static_cast<UInt8>(y)))
            {
                return false;
            }
            ++i;
            ++j;
        }
        return i == a.size() && j == b.size();
    }

    // LSP CompletionItemKind -> the popup's six. Deliberately lossy: the protocol
    // has 25 kinds, and 25 two-letter tags is a popup nobody reads.
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

    // What to actually type when an item is accepted. clangd's `label` is for
    // HUMANS - a leading non-ASCII decoration character and often a signature - so
    // `filterText`, the plain identifier, is what belongs in the buffer.
    // insertText is ignored: even with snippets off it includes a function's `(`.
    Str insertionFor(const js::Value& item)
    {
        const Str filter = item.at("filterText").string();
        if(!filter.empty())
        {
            return filter;
        }

        // No filterText: everything before the first identifier character is
        // ornament.
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

    // The first line of the documentation, and only the first: the popup has one
    // row for it, and a firmware header's doc comment can be forty lines.
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

    // A MarkupContent, or a bare string. json.hxx has no isString(), so this
    // follows firstLine()'s idiom: take the string, and if there is none, look
    // for the object's `value`.
    Str markupText(const js::Value& v)
    {
        Str text = v.string();
        if(text.empty() && v.isObject())
        {
            text = v.at("value").string();
        }
        return text;
    }

    // clangd's hover markdown, split into the declaration and the prose.
    //
    // WHAT IT ACTUALLY SENDS, which is why this is not a one-liner:
    //
    //     ### function `trimEnd`
    //     ---
    //     -> `Size`
    //     Parameters:
    //     - `Utf8 * s`
    //     ---
    //     Strips trailing CR, LF, space and tab from `s`, in place.
    //     ---
    //     ```cpp
    //     Size trimEnd(Utf8 *s)
    //     ```
    //
    // The fenced block is the declaration and is the one part always present.
    // Everything outside it is prose, EXCEPT the generated preamble - the
    // `###` title, the `->` return line and the `Parameters:` list all restate
    // what the signature already shows, and repeating them under it is how a
    // tooltip becomes taller than the code it covers.
    Void splitHover(const Str& raw, Str& sig, Str& doc)
    {
        sig.clear();
        doc.clear();

        Vec<Str> body;
        Bool     inFence = false;

        Size i = 0;
        while(i <= raw.size())
        {
            const Size nl = raw.find('\n', i);
            const Size stop = (nl == Str::npos) ? raw.size() : nl;
            Str        line = raw.substr(i, stop - i);
            while(!line.empty() && (line.back() == '\r' || line.back() == ' '))
            {
                line.pop_back();
            }

            if(line.rfind("```", 0) == 0)
            {
                inFence = !inFence;
            }
            else if(inFence)
            {
                if(!sig.empty())
                {
                    sig += "\n";
                }
                sig += line;
            }
            else
            {
                // The preamble, dropped. `---` is a rule, `###` the generated
                // title, and the arrow and parameter list are the signature
                // said twice.
                const Bool noise = line.empty()
                                   || line.rfind("---", 0) == 0
                                   || line.rfind("###", 0) == 0
                                   || line.rfind("→", 0) == 0
                                   || line.rfind("->", 0) == 0
                                   || line.rfind("Parameters:", 0) == 0
                                   || line.rfind("- `", 0) == 0;
                if(!noise)
                {
                    // Backticks are markdown's, not the reader's. `provided by
                    // `"hal.hxx"`` is clangd quoting a string it has already
                    // quoted, and in a tooltip it is just clutter.
                    Str clean;
                    clean.reserve(line.size());
                    for(const Char c : line)
                    {
                        if(c != '`')
                        {
                            clean.push_back(c);
                        }
                    }
                    body.push_back(clean);
                }
            }

            if(nl == Str::npos)
            {
                break;
            }
            i = nl + 1;
        }

        // NO FENCE AT ALL. For a namespace clangd sends plain text -
        //
        //     namespace gpio
        //
        //     // In namespace bibo
        //     namespace gpio {}
        //
        // - and nothing in it is fenced, so the walk above filed every line
        // as prose and the signature stayed empty. An empty signature is what
        // the Code view reads as "clangd has nothing to say", which is how
        // hovering a namespace showed nothing at all. In that shape the first
        // line IS the declaration: it becomes the signature, the rest stays
        // prose. Measured against clangd 18's actual reply, not guessed.
        if(sig.empty() && !body.empty())
        {
            sig = body.front();
            body.erase(body.begin());
        }

        // FOUR LINES OF PROSE, no more. A hover answers "what is this" - the
        // declaration and the sentence under it. A forty-line doc comment
        // belongs in the header, where the reader chose to go, not in a box
        // over the code they were reading.
        constexpr Size HOVER_PROSE_MAX = 4;
        if(body.size() > HOVER_PROSE_MAX)
        {
            body.resize(HOVER_PROSE_MAX);
            body.back() += " ...";
        }

        for(const Str& l : body)
        {
            if(!doc.empty())
            {
                doc += "\n";
            }
            doc += l;
        }
    }

    // ------------------------------------------------------------------ state --

    struct Impl
    {
        // ---- the child ----
        HANDLE proc = nullptr;
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

        // Whether clangd has finished building an AST. Asked before the preamble
        // exists it answers ANYWAY, from an identifier index, and nothing in the
        // reply says so. The signal is publishDiagnostics for our URI, which clangd
        // sends when the AST is done even with nothing to report.
        Atomic<Bool> parsed{false};

        // When that happened - parsed is not quite ready. MEASURED: asking as the
        // diagnostics arrive gives 5 items for `dfplayer::`, half a second later
        // all 20. The AST is built by publish time; the preamble index is not.
        Atomic<UInt64> parsedAtMs{0};

        mutable Mutex uriMu;    // openUri is written by ask(), read by the reader
        Str           openUri;

        Mutex  answerMu;
        Answer answer;                  // serial 0 until the first reply
        Bool   answerFresh = false;

        // Hover's own slot and its own in-flight id. Sharing either with
        // completion loses replies: the two are asked at different moments and
        // whichever answered second would be dropped as "a question the caret
        // has already left".
        Atomic<Int64> infoInFlight{-1};
        Mutex         infoMu;
        Info          info;
        Bool          infoFresh = false;

        // Signature help and the outline, each with the same shape as hover and
        // for the same reason: a shared slot drops whichever reply lands second.
        Atomic<Int64> sigInFlight{-1};
        Mutex         sigMu;
        Sig           sig;
        Bool          sigFresh = false;

        Atomic<Int64> outlineInFlight{-1};
        Mutex         outlineMu;
        Vec<Symbol>   outline;
        Bool          outlineFresh = false;

        // Replaced wholesale on every publish: clangd sends the complete set each
        // time, and an empty array means "clean now", not "nothing to say".
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

        // Queues one JSON-RPC message; the framing is added here.
        Void send(const Str& body)
        {
            Array<Char, 64> head;
            std::snprintf(
                head.data(),
                head.size(),
                "Content-Length: %llu\r\n\r\n",
                static_cast<unsigned long long>(body.size())
            );

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

            // A partial write on a pipe is not an error, and half a frame leaves
            // clangd permanently desynchronised - so loop until it is all out.
            Size written = 0;
            while(written < msg.size())
            {
                DWORD      got = 0;
                const BOOL ok = WriteFile(
                    s.toChild,
                    msg.data() + written,
                    static_cast<DWORD>(msg.size() - written),
                    &got,
                    nullptr
                );
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

    // One header line, up to and including the \r\n, which is discarded. Byte at a
    // time on purpose: the alternative buffers ahead of the body and has to hand
    // the leftovers to readExact, which is where framing bugs live.
    Bool readLine(HANDLE h, Str& out)
    {
        out.clear();
        for(;;)
        {
            Char  c = 0;
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

                // Case-insensitive: misreading the header name fails silently.
                if(line.size() > 15 && _strnicmp(line.c_str(), "Content-Length:", 15) == 0)
                {
                    length = static_cast<Size>(std::strtoull(line.c_str() + 15, nullptr, 10));
                    sawLen = true;
                }
            }

            if(!sawLen || length == 0 || length > (64u * 1024u * 1024u))
            {
                // A stream whose framing is wrong cannot be resynchronised.
                s.running.store(false);
                s.st.store(State::STATE_FAILED);
                s.sayf(
                    "clangd: bad frame header (length %llu)",
                    static_cast<unsigned long long>(length)
                );
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

            // Every frame clangd sends, dumped to lspdump_NNN.json when
            // BIBO_LSP_DUMP is set - the only way to tell a client dropping items
            // from a server sending fewer.
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
            Bool      ok = false;
            Size      stopped = 0;
            js::Value msg = js::parse(body, ok, stopped);
            if(!ok)
            {
                // Carry on: the framing is still sound, because the length told us
                // exactly where the message ended.
                s.sayf(
                    "clangd: unreadable reply at byte %llu",
                    static_cast<unsigned long long>(stopped)
                );
                continue;
            }

            handle(msg);
        }
    }

    Void handle(const js::Value& msg)
    {
        Impl& s = impl();

        // A request FROM clangd - both an id and a method. Answer it, even with
        // null: clangd BLOCKS waiting for the reply.
        if(!msg.at("method").isNull() && !msg.at("id").isNull())
        {
            Array<Char, 96> buf;
            std::snprintf(
                buf.data(),
                buf.size(),
                "{\"jsonrpc\":\"2.0\",\"id\":%d,\"result\":null}",
                msg.at("id").integer()
            );
            s.send(buf.data());
            return;
        }

        // A notification: diagnostics, logs, progress.
        if(msg.at("id").isNull())
        {
            // The one that matters - its ARRIVAL is clangd saying the AST is ready,
            // the earliest moment a completion is worth asking for.
            if(msg.at("method").string() == "textDocument/publishDiagnostics")
            {
                Str mine;
                Str minePath;
                {
                    LockGuard<Mutex> lk(s.uriMu);
                    mine = s.openUri;
                    minePath = s.openPath;
                }
                if(!mine.empty() && sameUri(msg.at("params").at("uri").string(), mine))
                {
                    if(!s.parsed.load())
                    {
                        s.parsedAtMs.store(GetTickCount64());
                        s.parsed.store(true);
                    }

                    // Every parse error clangd found, and every clang-tidy check a
                    // .clangd turned on, arrives here and nowhere else.
                    const js::Value& arr = msg.at("params").at("diagnostics");
                    Vec<diag::Item>  got;

                    for(Size i = 0; i < arr.size(); ++i)
                    {
                        const js::Value& d = arr[i];

                        diag::Item it;
                        it.file = minePath;

                        // LSP counts lines and characters from ZERO; diag::Item
                        // holds what a compiler said, and those count from one.
                        it.line = d.at("range").at("start").at("line").integer(0) + 1;
                        it.column = d.at("range").at("start").at("character").integer(0) + 1;

                        // LSP severity: 1 error, 2 warning, 3 information,
                        // 4 hint. Anything softer than a warning is a note.
                        const Int32 sev = d.at("severity").integer(2);
                        it.severity = (sev == 1) ? diag::Severity::SEVERITY_ERR
                                    : (sev == 2) ? diag::Severity::SEVERITY_WARN
                                                 : diag::Severity::SEVERITY_NOTE;

                        it.message = d.at("message").string();

                        // Which check fired, the way clang-tidy prints it - without
                        // it a tidy finding looks like a compiler warning.
                        const Str code = d.at("code").string();
                        if(!code.empty())
                        {
                            it.message += " [" + code + "]";
                        }

                        got.push_back(it);
                    }

                    LockGuard<Mutex> lk(s.diagMu);
                    s.diags = got;
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

        // ---- a signature help reply ----
        if(static_cast<Int64>(id) == s.sigInFlight.load())
        {
            s.sigInFlight.store(-1);

            const js::Value& res = msg.at("result");
            const js::Value& sigs = res.at("signatures");

            Sig built;
            if(sigs.isArray() && sigs.size() > 0)
            {
                // Which overload. Absent means the first, per the protocol.
                Int32 which = res.at("activeSignature").integer(0);
                if(which < 0 || static_cast<Size>(which) >= sigs.size())
                {
                    which = 0;
                }
                const js::Value& sg = sigs[static_cast<Size>(which)];
                built.label = sg.at("label").string();

                // The active parameter may be on the signature or on the result;
                // clangd puts it on the result.
                Int32 ap = sg.at("activeParameter").integer(-1);
                if(ap < 0)
                {
                    ap = res.at("activeParameter").integer(-1);
                }

                const js::Value& params = sg.at("parameters");
                if(ap >= 0 && params.isArray() && static_cast<Size>(ap) < params.size())
                {
                    // A parameter's label is EITHER a substring of the signature
                    // OR a [start, end] pair into it. clangd sends the string;
                    // the pair costs three lines and means the range is exact
                    // when a name appears twice.
                    const js::Value& pl = params[static_cast<Size>(ap)].at("label");
                    if(pl.isArray() && pl.size() == 2)
                    {
                        built.activeBegin = pl[0].integer(-1);
                        built.activeEnd = pl[1].integer(-1);
                    }
                    else
                    {
                        const Str  name = pl.string();
                        const Size at = name.empty() ? Str::npos : built.label.find(name);
                        if(at != Str::npos)
                        {
                            built.activeBegin = static_cast<Int32>(at);
                            built.activeEnd = static_cast<Int32>(at + name.size());
                        }
                    }
                }
            }

            {
                LockGuard<Mutex> lk(s.sigMu);
                built.serial = s.sig.serial + 1u;
                built.path = s.sig.path;
                built.line = s.sig.line;
                built.col = s.sig.col;
                s.sig = std::move(built);
                s.sigFresh = true;
            }
            return;
        }

        // ---- an outline reply ----
        if(static_cast<Int64>(id) == s.outlineInFlight.load())
        {
            s.outlineInFlight.store(-1);

            Vec<Symbol> got;
            const js::Value& arr = msg.at("result");

            // LSP SymbolKind, the ones a C++ file actually contains.
            const auto tag = [](Int32 k) -> const Char*
            {
                switch(k)
                {
                case 3:            return "ns";
                case 5:            return "cls";
                case 23:           return "struct";
                case 10: case 22:  return "enum";     // Enum, EnumMember
                case 6:  case 12:  return "fn";       // Method, Function
                case 9:            return "ctor";
                case 7:  case 8:   return "fld";      // Field, Property
                case 13:           return "var";
                case 14:           return "const";
                case 26:           return "type";
                default:           return "";
                }
            };

            // Two shapes, both legal. SymbolInformation is flat and carries a
            // `location`; DocumentSymbol is a tree with `range` and `children`.
            // We advertise no tree support so clangd sends the flat one, but the
            // tree is handled too, because a different server would not.
            const auto add = [&](auto&& self, const js::Value& it, Int32 depth) -> Void
            {
                Symbol sym;
                sym.name = it.at("name").string();
                sym.kind = tag(it.at("kind").integer(0));
                sym.depth = depth;

                if(!it.at("location").isNull())
                {
                    sym.line = it.at("location").at("range").at("start").at("line").integer(0);
                    if(!it.at("containerName").string().empty())
                    {
                        sym.depth = 1;
                    }
                }
                else
                {
                    const js::Value& r = it.at("selectionRange").isNull() ? it.at("range")
                                                                          : it.at("selectionRange");
                    sym.line = r.at("start").at("line").integer(0);
                }

                if(!sym.name.empty())
                {
                    got.push_back(sym);
                }

                const js::Value& kids = it.at("children");
                if(kids.isArray())
                {
                    for(Size i = 0; i < kids.size(); ++i)
                    {
                        self(self, kids[i], depth + 1);
                    }
                }
            };

            if(arr.isArray())
            {
                for(Size i = 0; i < arr.size(); ++i)
                {
                    add(add, arr[i], 0);
                }
            }

            // Source order, whatever order clangd chose. A list you jump around
            // in has to read the way the file does.
            std::sort(got.begin(), got.end(), [](const Symbol& a, const Symbol& b)
            {
                return a.line < b.line;
            });

            {
                LockGuard<Mutex> lk(s.outlineMu);
                s.outline = std::move(got);
                s.outlineFresh = true;
            }
            return;
        }

        // ---- a hover reply ----
        if(static_cast<Int64>(id) == s.infoInFlight.load())
        {
            s.infoInFlight.store(-1);

            // `contents` is one of three shapes and the protocol allows all of
            // them: a MarkupContent object, a bare string, or an array of
            // either. clangd sends the first; handling the rest costs six lines
            // and means a different server does not come back blank.
            const js::Value& c = msg.at("result").at("contents");
            Str              raw;
            if(c.isArray())
            {
                for(Size i = 0; i < c.size(); ++i)
                {
                    const Str piece = markupText(c[i]);
                    if(!piece.empty())
                    {
                        if(!raw.empty())
                        {
                            raw += "\n";
                        }
                        raw += piece;
                    }
                }
            }
            else
            {
                raw = markupText(c);
            }

            Info built;
            splitHover(raw, built.sig, built.doc);

            {
                LockGuard<Mutex> lk(s.infoMu);
                built.serial = s.info.serial + 1u;
                built.path = s.info.path;    // set by askInfo()
                built.line = s.info.line;
                built.col = s.info.col;
                s.info = std::move(built);
                s.infoFresh = true;
            }
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
        const js::Value& items = result.isArray() ? result : result.at("items");

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
            out.doc = firstLine(it.at("documentation"));
            out.kind = kindOf(it.at("kind").integer(6));

            built.items.push_back(std::move(out));
        }

        {
            LockGuard<Mutex> lk(s.answerMu);
            built.serial = s.answer.serial + 1u;
            built.path = s.answer.path;    // set by ask(), see below
            built.line = s.answer.line;
            built.col = s.answer.col;
            s.answer = std::move(built);
            s.answerFresh = true;
        }
    }

    // ------------------------------------------------------------- starting ---

    // The compiler the build invokes, from the first entry of
    // compile_commands.json, to feed --query-driver.
    //
    // clangd will NOT ask that compiler where its headers are unless
    // --query-driver allows it, and without them `#include <cstddef>` does not
    // resolve. The symptom is NOT an error: the file still "parses" and clangd
    // still answers, from an identifier index instead of the AST. The EXACT path,
    // not a glob - --query-driver makes clangd EXECUTE what it matches.
    Str driverFromDatabase(const Str& ccPath)
    {
        std::FILE* f = std::fopen(ccPath.c_str(), "rb");
        if(f == nullptr)
        {
            return Str();
        }

        // The first entry is at the top, and the whole file can be tens of MB.
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
        // inside the JSON string, so the \" case both unescapes and toggles.
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

        // Only if it is really there: a stale database naming an uninstalled
        // compiler would otherwise hand clangd a path to run.
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
        if(SearchPathA(
            nullptr,
            "clangd.exe",
            nullptr,
            static_cast<DWORD>(found.size()),
            found.data(),
            nullptr
        ) != 0)
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

      const Str root = PicoFlash::repoRoot();
      const Str ccDir = root + "\\firmware\\build";
      const Str ccPath = ccDir + "\\compile_commands.json";

      const Str driver = driverFromDatabase(ccPath);

      if(GetFileAttributesA(ccPath.c_str()) == INVALID_FILE_ATTRIBUTES)
      {
          // Not fatal: clangd still completes from the file's own includes, but
          // without the SDK's flags half the firmware headers will not resolve.
          s.say("clangd: no compile_commands.json - run firmware\\build.bat once");
      }

      SECURITY_ATTRIBUTES sa{};
      sa.nLength = sizeof(sa);
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
          s.sayf("clangd: CreatePipe failed (%lu)", static_cast<unsigned long>(GetLastError()));
          return false;
      }

      // The parent's ends must NOT be inherited, or the child holds a copy and
      // the reader never sees EOF when clangd exits.
      SetHandleInformation(in.wr,  HANDLE_FLAG_INHERIT, 0);
      SetHandleInformation(out.rd, HANDLE_FLAG_INHERIT, 0);

      STARTUPINFOA si{};
      si.cb = sizeof(si);
      si.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
      si.wShowWindow = SW_HIDE;
      si.hStdInput = in.rd;
      si.hStdOutput = out.wr;
      si.hStdError = nullptr;   // clangd's log; we do not read it, so drop it

      // --log=error, not the default: the info log is a few hundred lines per
      // parse into a handle nobody drains, which eventually blocks clangd.
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
      const BOOL ok = CreateProcessA(
          nullptr,
          mutableCmd.data(),
          nullptr,
          nullptr,
          TRUE,
          CREATE_NO_WINDOW,
          nullptr,
          root.c_str(),
          &si,
          &pi
      );

      closeHandle(in.rd);    // the child's ends; the parent must not hold them
      closeHandle(out.wr);

      if(!ok)
      {
          const DWORD err = GetLastError();
          closeHandle(in.wr);
          closeHandle(out.rd);
          s.st.store(State::STATE_FAILED);
          s.sayf("clangd: could not start %s (%lu)", exe.c_str(), static_cast<unsigned long>(err));
          return false;
      }

      CloseHandle(pi.hThread);
      s.proc = pi.hProcess;
      s.toChild = in.wr;
      s.frChild = out.rd;

      // A restart must not inherit the previous session's unsent tail, or the new
      // clangd starts reading the middle of a conversation it was not part of.
      {
          LockGuard<Mutex> lk(s.outMu);
          s.outbox.clear();
      }
      {
          LockGuard<Mutex> lk(s.answerMu);
          s.answer = Answer();
          s.answerFresh = false;
      }
      s.inFlight.store(-1);
      s.openPath.clear();
      s.sentVersion = 0;
      s.docVersion = 1;

      s.running.store(true);
      s.st.store(State::STATE_STARTING);
      s.say("clangd: starting");

      s.reader = Thread(readerLoop);
      s.writer = Thread(writerLoop);

      const Str rootUri = toUri(root);
      s.send("{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{"
             "\"processId\":null,"
             "\"rootUri\":" + js::quote(rootUri) + ","
             "\"capabilities\":{\"textDocument\":{"
               "\"completion\":{"
                 "\"completionItem\":{"
                   // Snippets OFF: clangd would otherwise send `${1:x}` placeholder
                   // syntax, which this editor would insert literally.
                   "\"snippetSupport\":false,"
                   "\"documentationFormat\":[\"plaintext\"]}},"
               // MARKDOWN HOVERS, asked for by name. With no contentFormat
               // declared clangd falls back to plain text, and plain text has
               // no fenced block - so splitHover(), written for the markdown
               // shape, found no declaration in ANY reply and every hover came
               // back empty. Namespaces were merely the first thing anyone
               // hovered. Measured against clangd 18 both ways.
               "\"hover\":{\"contentFormat\":[\"markdown\",\"plaintext\"]}"
             "}}}}");

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

      // Close our write end FIRST: clangd sees EOF on stdin, exits, and closes its
      // stdout, which is what unblocks the reader's ReadFile.
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

      // Handed over even when EMPTY: clangd publishes a complete set every time,
      // so an empty one means the file just became clean.
      out = s.diags;
      s.diagsFresh = false;
      return true;
  }

  // Brings clangd's copy of the file up to date and returns its URI, or an
  // empty string if there is nothing to talk to.
  //
  // SHARED BY ask() AND askInfo(), which is the point: two callers each pushing
  // their own didOpen/didChange would double the version counter and send the
  // buffer twice per keystroke. Whichever asks first pays for the sync; the
  // other finds the document already current and sends nothing.
  Str syncDoc(const Str& path, const Str& text, UInt64 version)
  {
      Impl& s = impl();
      if(s.st.load() != State::STATE_READY || path.empty())
      {
          return Str();
      }

      const Str uri = toUri(path);

      // ---- keep clangd's copy of the file current ----
      // FULL-TEXT sync, not incremental ranges: one dropped or misordered change
      // and the server's copy silently diverges, after which every completion is
      // about a file that does not exist.
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
              // openPath joins openUri under this lock: the reader thread needs
              // the PATH for a diagnostic, and reading it unguarded races on a Str.
              LockGuard<Mutex> lk(s.uriMu);
              s.openUri = uri;
              s.openPath = path;
          }
          s.parsed.store(false);     // a new file: nothing is built for it yet

          s.docVersion = 1;
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

      return uri;
  }

  // Whether clangd has an AST ready for the open file. The didOpen/didChange
  // still went out before this is consulted - only the QUESTION waits, and a
  // caller told false asks again rather than reading silence as an answer.
  Bool astReady()
  {
      Impl& s = impl();
      return s.parsed.load()
             && GetTickCount64() - s.parsedAtMs.load() >= SETTLE_MS;
  }

  Bool ask(const Str& path, const Str& text, UInt64 version, Int32 line, Int32 col)
  {
      Impl&     s = impl();
      const Str uri = syncDoc(path, text, version);
      if(uri.empty() || !astReady())
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
          s.answer.col = col;
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
      out = s.answer;
      s.answerFresh = false;
      return true;
  }

  Bool askInfo(const Str& path, const Str& text, UInt64 version, Int32 line, Int32 col)
  {
      Impl&     s = impl();
      const Str uri = syncDoc(path, text, version);
      if(uri.empty() || !astReady())
      {
          return false;
      }

      const Int64 id = static_cast<Int64>(s.nextId.fetch_add(1));

      // Recorded BEFORE sending, so a fast reply cannot land on a slot that
      // still describes the previous question.
      {
          LockGuard<Mutex> lk(s.infoMu);
          s.info.path = path;
          s.info.line = line;
          s.info.col = col;
      }
      s.infoInFlight.store(id);

      Array<Char, 256> head;
      std::snprintf(head.data(), head.size(),
                    "{\"jsonrpc\":\"2.0\",\"id\":%lld,"
                    "\"method\":\"textDocument/hover\",\"params\":{"
                    "\"position\":{\"line\":%d,\"character\":%d},"
                    "\"textDocument\":{\"uri\":",
                    static_cast<long long>(id), line, col);

      s.send(Str(head.data()) + js::quote(uri) + "}}}");
      return true;
  }

  Bool takeInfo(Info& out)
  {
      Impl&            s = impl();
      LockGuard<Mutex> lk(s.infoMu);

      if(!s.infoFresh)
      {
          return false;
      }
      out = s.info;
      s.infoFresh = false;
      return true;
  }

  Bool sync(const Str& path, const Str& text, UInt64 version)
  {
      return !syncDoc(path, text, version).empty();
  }

  Bool askSig(const Str& path, const Str& text, UInt64 version, Int32 line, Int32 col)
  {
      Impl&     s = impl();
      const Str uri = syncDoc(path, text, version);
      if(uri.empty() || !astReady())
      {
          return false;
      }

      const Int64 id = static_cast<Int64>(s.nextId.fetch_add(1));
      {
          LockGuard<Mutex> lk(s.sigMu);
          s.sig.path = path;
          s.sig.line = line;
          s.sig.col = col;
      }
      s.sigInFlight.store(id);

      Array<Char, 256> head;
      std::snprintf(head.data(), head.size(),
                    "{\"jsonrpc\":\"2.0\",\"id\":%lld,"
                    "\"method\":\"textDocument/signatureHelp\",\"params\":{"
                    "\"position\":{\"line\":%d,\"character\":%d},"
                    "\"textDocument\":{\"uri\":",
                    static_cast<long long>(id), line, col);

      s.send(Str(head.data()) + js::quote(uri) + "}}}");
      return true;
  }

  Bool takeSig(Sig& out)
  {
      Impl&            s = impl();
      LockGuard<Mutex> lk(s.sigMu);
      if(!s.sigFresh)
      {
          return false;
      }
      out = s.sig;
      s.sigFresh = false;
      return true;
  }

  Bool askOutline(const Str& path, const Str& text, UInt64 version)
  {
      Impl&     s = impl();
      const Str uri = syncDoc(path, text, version);
      if(uri.empty() || !astReady())
      {
          return false;
      }

      const Int64 id = static_cast<Int64>(s.nextId.fetch_add(1));
      s.outlineInFlight.store(id);

      Array<Char, 160> head;
      std::snprintf(head.data(), head.size(),
                    "{\"jsonrpc\":\"2.0\",\"id\":%lld,"
                    "\"method\":\"textDocument/documentSymbol\",\"params\":{"
                    "\"textDocument\":{\"uri\":",
                    static_cast<long long>(id));

      s.send(Str(head.data()) + js::quote(uri) + "}}}");
      return true;
  }

  Bool takeOutline(Vec<Symbol>& out)
  {
      Impl&            s = impl();
      LockGuard<Mutex> lk(s.outlineMu);
      if(!s.outlineFresh)
      {
          return false;
      }
      out = s.outline;
      s.outlineFresh = false;
      return true;
  }

}
