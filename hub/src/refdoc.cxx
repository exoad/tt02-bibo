// The .bdoc parser and renderer. See refdoc.hxx for the format and the rules.

#include "shared.hxx"

#include "refdoc.hxx"
#include "syntax.hxx"
#include "theme.hxx"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>

namespace refdoc
{

  namespace
  {

    // ---------------------------------------------------------------------------
    //  parsing
    // ---------------------------------------------------------------------------

    // Where includes resolve from, and how deep we are.
    //
    // A depth limit rather than a set of visited paths: a document that includes
    // itself is the obvious cycle, but so is A includes B includes A, and a limit
    // catches every shape of it with one integer. Eight is far past anything a
    // real document needs and far short of a stack overflow.
    struct Ctx
    {
        Str   baseDir;
        Int32 depth = 0;
    };

    constexpr Int32 MAX_INCLUDE_DEPTH = 8;

    struct Cursor
    {
        const Str* src  = nullptr;
        Size       at   = 0;
        Int32      line = 1;

        [[nodiscard]] Bool done() const
        {
            return at >= src->size();
        }

        [[nodiscard]] Char peek(Size ahead = 0) const
        {
            const Size i = at + ahead;
            return (i < src->size()) ? (*src)[i] : '\0';
        }

        Char take()
        {
            const Char c = peek();
            ++at;
            if(c == '\n')
            {
                ++line;
            }
            return c;
        }

        [[nodiscard]] Bool starts(const Char* s) const
        {
            return src->compare(at, std::strlen(s), s) == 0;
        }
    };

    Bool nameChar(Char c)
    {
        return std::isalnum(static_cast<UInt8>(c)) != 0 || c == '_' || c == '-';
    }

    Void skipSpace(Cursor& cur)
    {
        while(!cur.done() && std::isspace(static_cast<UInt8>(cur.peek())) != 0)
        {
            cur.take();
        }
    }

    // The four that have to exist, and no more.
    //
    // A format with a full entity table needs a table, and then it needs the table
    // to be right. These four are the ones you cannot write any other way, which is
    // the only reason an entity is ever needed.
    Bool entity(Cursor& cur, Str& out)
    {
        static const struct
        {
            const Char* text;
            Char        ch;
        } ENT[] = {
            { "&lt;",   '<'  },
            { "&gt;",   '>'  },
            { "&amp;",  '&'  },
            { "&quot;", '"'  },
        };

        for(const auto& e : ENT)
        {
            if(cur.starts(e.text))
            {
                for(Size i = 0; i < std::strlen(e.text); ++i)
                {
                    cur.take();
                }
                out.push_back(e.ch);
                return true;
            }
        }
        return false;
    }

    // Text between tags, with runs of whitespace collapsed to one space.
    //
    // COLLAPSED, so a paragraph can be hand-wrapped in the file at whatever width
    // reads well and still flow to the panel it is drawn in. A format that
    // preserved the source line breaks would make every document's layout a
    // property of the editor it was typed in.
    Str readText(Cursor& cur)
    {
        Str out;
        Bool space = false;

        while(!cur.done() && cur.peek() != '<')
        {
            if(cur.peek() == '&' && entity(cur, out))
            {
                space = false;
                continue;
            }

            const Char c = cur.take();
            if(std::isspace(static_cast<UInt8>(c)) != 0)
            {
                space = true;
                continue;
            }

            // The LEADING space is kept too, and that is the whole point.
            //
            // "For <Bold>bibo</Bold> this is" is three runs, and the third one
            // starts with the space between "</Bold>" and "this". Dropping it
            // because the run was empty is what produced "Forbibothis" - the gap
            // either side of every inline element silently closing up.
            //
            // A leading space costs nothing when it is genuinely at the start of a
            // paragraph: drawRuns ignores a gap when the line is empty.
            if(space)
            {
                out.push_back(' ');
            }
            space = false;
            out.push_back(c);
        }

        // A trailing space matters: "<Bold>x</Bold> and" must not become "xand".
        if(space && !out.empty())
        {
            out.push_back(' ');
        }
        return out;
    }

    // Elements whose content is NOT collapsed.
    //
    // Everywhere else, runs of whitespace become one space so a paragraph can be
    // hand-wrapped in the file at whatever width reads well and still flow to the
    // panel. These two are the opposite case: the newlines and the columns ARE the
    // content. A <Code> block showing the shape of something, flattened to one
    // line, is showing nothing; a <Draw> program is a line per command.
    Bool rawElement(const Str& name)
    {
        return name == "Code" || name == "Draw";
    }

    // Everything up to the matching close tag, verbatim.
    //
    // No entity expansion and no nested elements: inside a raw element a '<' is
    // just a '<', which is what lets a Code block quote this format's own tags
    // without every one of them needing to be written as &lt;.
    Str readRaw(Cursor& cur, const Str& name)
    {
        const Str close = "</" + name;

        Str out;
        while(!cur.done() && !cur.starts(close.c_str()))
        {
            out.push_back(cur.take());
        }

        // A leading newline after the open tag is where the author pressed return,
        // not content. The same for trailing whitespace before the close tag.
        Size b = 0;
        while(b < out.size() && (out[b] == '\n' || out[b] == '\r'))
        {
            ++b;
        }
        Size e = out.size();
        while(e > b && (out[e - 1] == '\n' || out[e - 1] == '\r'
                        || out[e - 1] == ' ' || out[e - 1] == '\t'))
        {
            --e;
        }
        return out.substr(b, e - b);
    }

    Bool parseNode(Cursor& cur, Node& out, Str& err, Int32& errLine, const Ctx& ctx);

    // ---- <Include file="..."/> ------------------------------------------------
    //
    // C's #include, and for C's reason: a pinout is worth keeping in its own file
    // and worth reading inside the document that explains the part.
    //
    // TEXTUAL, and at PARSE time. The included file's nodes are spliced in where
    // the tag was, so by the time anything is drawn there is one tree and no
    // renderer has to know an include ever happened. Resolving it later would mean
    // every pass - drawing, style checking, error reporting - carrying its own idea
    // of what the document contains.
    //
    // A pinout file IS a document. There is no second file type: what makes it a
    // pinout is what is in it, not what it is called, so one can be opened and read
    // on its own or pulled into three others.
    Str readFile(const Str& path)
    {
        Str out;
        FILE* f = nullptr;
        if(::fopen_s(&f, path.c_str(), "rb") != 0 || f == nullptr)
        {
            return out;
        }
        Array<Char, 4096> buf;
        Size n = 0;
        while((n = std::fread(buf.data(), 1, buf.size(), f)) > 0)
        {
            out.append(buf.data(), n);
        }
        std::fclose(f);

        // CRLF out, like the editor does, so a line number in an error message
        // counts the same lines a person sees.
        Str lf;
        lf.reserve(out.size());
        for(Char c : out)
        {
            if(c != '\r')
            {
                lf.push_back(c);
            }
        }
        return lf;
    }

    Str joinPath(const Str& baseDir, const Str& rel)
    {
        if(baseDir.empty())
        {
            return rel;
        }
        Str out = baseDir;
        if(out.back() != '\\' && out.back() != '/')
        {
            out.push_back('\\');
        }
        for(Char c : rel)
        {
            out.push_back((c == '/') ? '\\' : c);
        }
        return out;
    }

    // A node that says, visibly, that an include did not work.
    //
    // Rendered rather than silent, and rendered where the included content would
    // have been. An include that quietly produces nothing is a document with a
    // hole in it that nobody notices until the part is in their hands.
    Node includeError(const Str& what, Int32 line)
    {
        Node warn;
        warn.name = "Warn";
        warn.line = line;

        Node text;
        text.text = what;
        text.line = line;
        warn.kids.push_back(std::move(text));
        return warn;
    }

    Bool parseDoc(const Str& text, const Ctx& ctx, Node& out, Str& err, Int32& errLine);

    // Everything up to the matching close tag.
    Bool parseChildren(Cursor& cur, Node& out, Str& err, Int32& errLine, const Ctx& ctx)
    {
        for(;;)
        {
            if(cur.done())
            {
                err     = "reached the end of the file inside <" + out.name + ">";
                errLine = out.line;
                return false;
            }

            if(cur.starts("</"))
            {
                return true;
            }

            if(cur.starts("<!--"))
            {
                while(!cur.done() && !cur.starts("-->"))
                {
                    cur.take();
                }
                for(Int32 i = 0; i < 3 && !cur.done(); ++i)
                {
                    cur.take();
                }
                continue;
            }

            if(cur.peek() == '<')
            {
                Node kid;
                if(!parseNode(cur, kid, err, errLine, ctx))
                {
                    return false;
                }

                // The splice. Everything the included document holds is dropped in
                // where the tag stood, so nothing downstream knows it happened.
                if(kid.name == "Include")
                {
                    const Str rel = kid.attr("file", "");

                    if(rel.empty())
                    {
                        out.kids.push_back(
                            includeError("<Include> with no file attribute",
                                         kid.line));
                    }
                    else if(ctx.depth >= MAX_INCLUDE_DEPTH)
                    {
                        out.kids.push_back(includeError(
                            "includes nested more than "
                                + std::to_string(MAX_INCLUDE_DEPTH)
                                + " deep - is a document including itself?",
                            kid.line));
                    }
                    else if(ctx.baseDir.empty())
                    {
                        out.kids.push_back(includeError(
                            "cannot resolve " + rel + " - this document was parsed"
                            " without a folder to resolve against", kid.line));
                    }
                    else
                    {
                        const Str path = joinPath(ctx.baseDir, rel);
                        const Str body = readFile(path);

                        if(body.empty())
                        {
                            out.kids.push_back(includeError(
                                "cannot read " + rel, kid.line));
                        }
                        else
                        {
                            Ctx sub2;
                            sub2.baseDir = ctx.baseDir;
                            sub2.depth   = ctx.depth + 1;

                            Node  inner;
                            Str   ierr;
                            Int32 iline = 0;
                            if(!parseDoc(body, sub2, inner, ierr, iline))
                            {
                                out.kids.push_back(includeError(
                                    rel + " line " + std::to_string(iline) + ": "
                                        + ierr, kid.line));
                            }
                            else
                            {
                                for(Node& k : inner.kids)
                                {
                                    out.kids.push_back(std::move(k));
                                }
                            }
                        }
                    }
                    continue;
                }

                out.kids.push_back(std::move(kid));
                continue;
            }

            const Int32 ln = cur.line;
            Str         t  = readText(cur);
            if(!t.empty())
            {
                Node run;
                run.text = std::move(t);
                run.line = ln;
                out.kids.push_back(std::move(run));
            }
        }
    }

    Bool parseNode(Cursor& cur, Node& out, Str& err, Int32& errLine, const Ctx& ctx)
    {
        out.line = cur.line;
        cur.take();   // '<'

        if(cur.peek() == '/')
        {
            err     = "a closing tag with nothing open";
            errLine = out.line;
            return false;
        }

        while(!cur.done() && nameChar(cur.peek()))
        {
            out.name.push_back(cur.take());
        }
        if(out.name.empty())
        {
            err     = "a '<' with no element name after it";
            errLine = out.line;
            return false;
        }

        // ---- attributes ------------------------------------------------------
        for(;;)
        {
            skipSpace(cur);

            if(cur.starts("/>"))
            {
                cur.take();
                cur.take();
                return true;   // self-closing: no children
            }
            if(cur.peek() == '>')
            {
                cur.take();
                break;
            }
            if(cur.done())
            {
                err     = "the file ends inside the tag <" + out.name + ">";
                errLine = out.line;
                return false;
            }

            Attr a;
            while(!cur.done() && nameChar(cur.peek()))
            {
                a.name.push_back(cur.take());
            }
            if(a.name.empty())
            {
                err = Str("unexpected '") + cur.peek() + "' in the tag <"
                    + out.name + ">";
                errLine = cur.line;
                return false;
            }

            skipSpace(cur);
            if(cur.peek() != '=')
            {
                err     = "the attribute " + a.name + " has no value";
                errLine = cur.line;
                return false;
            }
            cur.take();
            skipSpace(cur);

            // Double quotes, always. Allowing bare or single-quoted values is a
            // small kindness that costs a parser branch and a class of confusing
            // failures, and this format has one author.
            if(cur.peek() != '"')
            {
                err     = a.name + " must be in double quotes";
                errLine = cur.line;
                return false;
            }
            cur.take();

            while(!cur.done() && cur.peek() != '"')
            {
                if(cur.peek() == '&' && entity(cur, a.value))
                {
                    continue;
                }
                a.value.push_back(cur.take());
            }
            if(cur.done())
            {
                err     = a.name + " has no closing quote";
                errLine = cur.line;
                return false;
            }
            cur.take();

            out.attrs.push_back(std::move(a));
        }

        // ---- children and the close tag --------------------------------------
        if(rawElement(out.name))
        {
            Node run;
            run.text = readRaw(cur, out.name);
            run.line = cur.line;
            out.kids.push_back(std::move(run));

            if(cur.done())
            {
                err     = "the file ends inside <" + out.name + ">";
                errLine = out.line;
                return false;
            }
        }
        else if(!parseChildren(cur, out, err, errLine, ctx))
        {
            return false;
        }

        cur.take();   // '<'
        cur.take();   // '/'

        Str closing;
        while(!cur.done() && nameChar(cur.peek()))
        {
            closing.push_back(cur.take());
        }
        skipSpace(cur);
        if(cur.peek() != '>')
        {
            err     = "the closing tag for <" + out.name + "> is malformed";
            errLine = cur.line;
            return false;
        }
        cur.take();

        if(closing != out.name)
        {
            err = "<" + out.name + "> is closed by </" + closing + ">";
            errLine = cur.line;
            return false;
        }
        return true;
    }

    // ---------------------------------------------------------------------------
    //  drawing
    // ---------------------------------------------------------------------------

    // ---- headings, at three sizes ---------------------------------------------
    //
    // 21 / 17 / 15 / 13, which is theme.hxx's STAT / TITLE / BODY / SMALL. A
    // document with one text size is a wall, and the reason to have a document
    // rather than a comment block is that you can skim it - which is a thing you do
    // by size before you do it by reading.
    //
    // PushFont and PopFont are paired inside this function and never straddle a
    // child window. That is not fussiness: an unbalanced font stack across
    // EndChild renders the whole window blank white, with no error, and it has
    // cost this project an afternoon before.
    Void headingText(ImFont* f, ImU32 col, const Char* text)
    {
        if(f != nullptr)
        {
            ImGui::PushFont(f, 0.0f);
        }
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(col));
        ImGui::TextUnformatted(text);
        ImGui::PopStyleColor();
        if(f != nullptr)
        {
            ImGui::PopFont();
        }
    }

    ImU32 classColor(const Char* cls)
    {
        struct Map
        {
            const Char* name;
            ImU32       col;
        };

        // The same sixteen the rest of the app draws in, so a pinout here and a
        // plot on the next screen mean the same thing by the same color.
        static constexpr Map MAP[] = {
            { "power",   ui::ansi::RED      },
            { "ground",  ui::ansi::GRAY     },
            { "serial",  ui::ansi::BRCYAN   },
            { "digital", ui::ansi::GREEN    },
            { "analog",  ui::ansi::MAGENTA  },
            // BRIGHT blue. Plain ANSI blue is 0x0000EE, which on a black page is
            // very nearly invisible - the USB pins were legible only if you
            // already knew they were there.
            { "usb",     ui::ansi::BRBLUE   },
            { "audio",   ui::ansi::BRYELLOW },
            { "unused",  ui::pin::FREE      },
        };

        for(const auto& m : MAP)
        {
            if(std::strcmp(m.name, cls) == 0)
            {
                return m.col;
            }
        }
        return ui::ansi::WHITE;
    }

    struct Run
    {
        Str   text;
        ImU32 col  = ui::ansi::WHITE;
        Bool  mono = false;
    };

    // Flattens a paragraph's children into styled runs.
    Void collect(const Node& n, Vec<Run>& out, ImU32 col, Bool mono)
    {
        for(const Node& k : n.kids)
        {
            if(k.isText())
            {
                Run r;
                r.text = k.text;
                r.col  = col;
                r.mono = mono;
                out.push_back(std::move(r));
                continue;
            }

            if(k.name == "Bold")
            {
                collect(k, out, ui::ansi::BRWHITE, mono);
            }
            else if(k.name == "Dim")
            {
                collect(k, out, ui::ansi::GRAY, mono);
            }
            else if(k.name == "Code")
            {
                collect(k, out, ui::ansi::BRCYAN, true);
            }
            else
            {
                collect(k, out, col, mono);
            }
        }
    }

    // Lays runs out word by word, wrapping at `width`.
    //
    // ImGui has no rich text, so a paragraph that mixes colors cannot be one
    // TextWrapped call. Emitting a word at a time with SameLine(0,0) is the usual
    // way round it, and the only subtlety is that the trailing space of a run has
    // to survive - "<Bold>x</Bold> and" must not come out as "xand".
    Void drawRuns(const Vec<Run>& runs, Float32 width, Float32 indent)
    {
        const Float32 spaceW = ImGui::CalcTextSize(" ").x;

        Float32 x     = 0.0f;
        Bool    empty = true;   // nothing on the current line yet

        // Whether a space is owed to the NEXT word, and it has to outlive the run
        // it was found in.
        //
        // "For <Bold>bibo</Bold> this" is three runs, and the space that separates
        // "For" from "bibo" is the trailing character of the FIRST one. Kept inside
        // the per-run loop it was consumed and then discarded, because the run
        // ended before another word needed it - which is what left "Forbibo".
        Bool gap = false;

        if(indent > 0.0f)
        {
            ImGui::Indent(indent);
            width -= indent;
        }

        for(const Run& r : runs)
        {
            Size i = 0;
            while(i < r.text.size())
            {
                // Whitespace before this word is the only thing that decides
                // whether a gap is drawn, and drawing it as the SameLine SPACING
                // rather than as a character is what keeps a wrapped line from
                // starting with one.
                while(i < r.text.size() && r.text[i] == ' ')
                {
                    gap = true;
                    ++i;
                }
                if(i >= r.text.size())
                {
                    break;
                }

                const Size start = i;
                while(i < r.text.size() && r.text[i] != ' ')
                {
                    ++i;
                }
                const Str word = r.text.substr(start, i - start);

                if(r.mono && ui::fonts.mono != nullptr)
                {
                    ImGui::PushFont(ui::fonts.mono, 0.0f);
                }
                const Float32 w = ImGui::CalcTextSize(word.c_str()).x;

                Float32 adv = (gap && !empty) ? spaceW : 0.0f;
                if(!empty && (x + adv + w) > width)
                {
                    x     = 0.0f;
                    empty = true;
                    adv   = 0.0f;
                }
                if(!empty)
                {
                    ImGui::SameLine(0.0f, adv);
                }

                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(r.col), "%s",
                                   word.c_str());

                if(r.mono && ui::fonts.mono != nullptr)
                {
                    ImGui::PopFont();
                }

                x    += adv + w;
                empty = false;
                gap   = false;
            }
        }

        if(indent > 0.0f)
        {
            ImGui::Unindent(indent);
        }
    }

    Void paragraph(const Node& n, Float32 width, ImU32 col, Float32 indent)
    {
        Vec<Run> runs;
        collect(n, runs, col, false);
        if(runs.empty())
        {
            return;
        }
        drawRuns(runs, width, indent);
        ImGui::Spacing();
    }

    // A bar down the left, and the text beside it. Used by Note and Warn, which
    // differ only in color and in how much they are asking for your attention.
    Void banner(const Node& n, Float32 width, ImU32 col)
    {
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const Float32 pad = 8.0f;

        ImGui::Indent(pad + 4.0f);
        Vec<Run> runs;
        collect(n, runs, col, false);
        drawRuns(runs, width - pad - 8.0f, 0.0f);
        ImGui::Unindent(pad + 4.0f);

        const ImVec2 p1 = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddRectFilled(
            ImVec2(p0.x, p0.y), ImVec2(p0.x + 2.0f, p1.y - 2.0f), col);

        ImGui::Spacing();
    }

    // ---- <Code> as a BLOCK ------------------------------------------------
    //
    // Gruvbox, line numbers, and the same tokenizer the editor uses.
    //
    // NOT A SECOND HIGHLIGHTER. syn::tokenize and syn::gruv already exist for
    // the Code view, so a block here colors a snippet exactly the way the
    // editor colors the file it came from. Writing a small independent one for
    // documents would have been quicker and would have drifted the first time
    // somebody taught the editor a new keyword.
    //
    // IT DOES NOT WRAP. Code is a shape, and a wrapped line is a different
    // shape - an 80-column table of pins folded at 62 stops being a table. The
    // page pans in both directions without limit now, so a long line is
    // reachable rather than lost, which is what makes not-wrapping affordable.
    //
    // Inline <Code> is untouched and still a cyan run inside a sentence; only
    // the block form is a panel.
    Void codeBlock(const Node& n, Float32 width)
    {
        Str body;
        for(const Node& k : n.kids)
        {
            if(k.isText())
            {
                body += k.text;
            }
        }

        // Split first: the number of lines decides the gutter width, and the
        // gutter width decides where everything else starts.
        Vec<Str> lines;
        {
            Str cur;
            for(const Char c : body)
            {
                if(c == '\n')
                {
                    lines.push_back(cur);
                    cur.clear();
                }
                else if(c != '\r')
                {
                    cur.push_back(c);
                }
            }
            lines.push_back(cur);
        }
        while(lines.size() > 1 && lines.back().empty())
        {
            lines.pop_back();
        }

        if(ui::fonts.mono != nullptr)
        {
            ImGui::PushFont(ui::fonts.mono, 0.0f);
        }

        const Float32 lineH = ImGui::GetTextLineHeight();
        const Float32 charW = ImGui::CalcTextSize("0").x;
        const Float32 pad   = 6.0f;

        // Wide enough for the largest number it will actually print, so a
        // 9-line block does not carry the gutter of a 900-line one.
        Int32 digits = 1;
        for(Size t = lines.size(); t >= 10; t /= 10)
        {
            ++digits;
        }
        const Float32 gutter = (static_cast<Float32>(digits) * charW) + pad * 2.0f;

        const ImVec2   p0 = ImGui::GetCursorScreenPos();
        const Float32  h  = (static_cast<Float32>(lines.size()) * lineH)
                          + pad * 2.0f;
        ImDrawList*    dl = ImGui::GetWindowDrawList();

        // Painted before the text, in one pass, because the height is known
        // from the line count rather than discovered by drawing.
        dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), syn::gruv::BG0_H);
        dl->AddRectFilled(p0, ImVec2(p0.x + gutter, p0.y + h), syn::gruv::BG1);
        dl->AddRect(p0, ImVec2(p0.x + width, p0.y + h), syn::gruv::BG3,
                    0.0f, 0, 1.0f);

        // `inBlock` carries a /* that opened on an earlier line, which is why
        // the tokenizer takes it by reference and why it lives out here rather
        // than inside the loop.
        Bool       inBlock = false;
        Vec<syn::Span> spans;

        for(Size i = 0; i < lines.size(); ++i)
        {
            const Str&    ln = lines[i];
            const Float32 y  = p0.y + pad + (static_cast<Float32>(i) * lineH);

            Array<Char, 16> num;
            std::snprintf(num.data(), num.size(), "%*d", digits,
                          static_cast<Int32>(i + 1));
            dl->AddText(ImVec2(p0.x + pad, y), syn::gruv::FG4, num.data());

            syn::tokenize(ln, inBlock, spans);

            Float32 x = p0.x + gutter + pad;
            for(const syn::Span& sp : spans)
            {
                if(sp.end <= sp.begin)
                {
                    continue;
                }
                const Char* b = ln.c_str() + sp.begin;
                const Char* e = ln.c_str() + sp.end;

                dl->AddText(ImVec2(x, y), syn::colorFor(sp.role), b, e);
                x += ImGui::CalcTextSize(b, e).x;
            }
        }

        if(ui::fonts.mono != nullptr)
        {
            ImGui::PopFont();
        }

        // Claim the space. The block was drawn straight onto the draw list, so
        // without this the layout has no idea it happened and the next
        // paragraph lands on top of it.
        ImGui::Dummy(ImVec2(width, h));
        ImGui::Spacing();
    }

    // One side of a package.
    Void pinColumn(const Node& col)
    {
        for(const Node& p : col.kids)
        {
            if(p.isText() || p.name != "Pin")
            {
                continue;
            }

            const Char* num   = p.attr("n", "?");
            const Char* name  = p.attr("name", "?");
            const Char* cls   = p.attr("class", "");
            const Bool  check = p.hasAttr("check");

            const ImU32 col32 = classColor(cls);

            // The pad number is always dim and always first: it is the thing you
            // count along the package with, and it is never the interesting part.
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GRAY),
                               "%3s", num);
            ImGui::SameLine(0.0f, 8.0f);

            if(ui::fonts.mono != nullptr)
            {
                ImGui::PushFont(ui::fonts.mono, 0.0f);
            }
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col32), "%-7s", name);
            if(ui::fonts.mono != nullptr)
            {
                ImGui::PopFont();
            }

            if(check)
            {
                ImGui::SameLine(0.0f, 6.0f);
                ImGui::TextColored(
                    ImGui::ColorConvertU32ToFloat4(ui::ansi::BRYELLOW), "check");
            }

            Vec<Run> runs;
            collect(p, runs, ui::ansi::WHITE, false);
            if(!runs.empty())
            {
                ImGui::SameLine(0.0f, 10.0f);
                drawRuns(runs, std::max(80.0f, ImGui::GetContentRegionAvail().x),
                         0.0f);
            }
        }
    }

    // ---------------------------------------------------------------------------
    //  <Draw> - a canvas, one command per line
    // ---------------------------------------------------------------------------
    //
    // A pinout is a picture as much as it is a table, and some things - a
    // connector's keying, which way a DIP is numbered, where a wire actually goes -
    // only ever read as a drawing. This is that, without leaving the document.
    //
    // An imperative DSL rather than more elements, and deliberately:
    //
    //   <Draw width="..." height="...">
    //     setColor cyan
    //     drawRect 10 10 200 120
    //     drawText 16 30 "VCC"
    //   </Draw>
    //
    // Elements are good at STRUCTURE and bad at sequence. A drawing is a sequence -
    // this on top of that, in this order - and expressing it as nested tags means
    // one tag per primitive, attributes for coordinates, and a document three times
    // the size saying the same thing. Every 2D library anybody has used, Flutter's
    // Canvas included, is a list of calls against a state, so this is a list of
    // calls against a state.
    //
    // COORDINATES ARE THE DOCUMENT'S OWN. <Draw> declares its natural size and the
    // renderer scales to whatever width the panel gives it, so a drawing composed
    // against 640x320 stays composed at any zoom or DPI. Nothing here deals in
    // pixels.
    //
    // Unknown commands are REPORTED, not ignored. A silently skipped line in a
    // drawing is a picture that is quietly missing a wire.

    struct Pen
    {
        ImU32   col   = ui::ansi::BRWHITE;
        Float32 width = 1.0f;
        Float32 text  = 12.0f;

        // -1 left, 0 centered, +1 right. The x in drawText is the ANCHOR, and this
        // says which part of the string lands on it.
        //
        // Added because the first real drawing needed it: a pinout has a column of
        // names ending at the pads, and without alignment the only way to right-
        // align them is to guess a per-character width and subtract. That guess is
        // wrong on any other font, and it goes wrong quietly - a column that
        // slowly frays instead of failing.
        Int32   align = -1;
    };

    // A command's arguments, already split.
    struct Words
    {
        Vec<Str> w;

        [[nodiscard]] Size count() const
        {
            return w.size();
        }

        [[nodiscard]] Float32 num(Size i) const
        {
            return (i < w.size()) ? static_cast<Float32>(std::atof(w[i].c_str()))
                                  : 0.0f;
        }

        [[nodiscard]] Str str(Size i) const
        {
            return (i < w.size()) ? w[i] : Str();
        }
    };

    // Splits a line into words, keeping "quoted strings" whole.
    Words split(const Str& line)
    {
        Words out;
        Size  i = 0;

        while(i < line.size())
        {
            while(i < line.size() && std::isspace(static_cast<UInt8>(line[i])) != 0)
            {
                ++i;
            }
            if(i >= line.size())
            {
                break;
            }

            Str tok;
            if(line[i] == '"')
            {
                ++i;
                while(i < line.size() && line[i] != '"')
                {
                    tok.push_back(line[i]);
                    ++i;
                }
                ++i;   // the closing quote
            }
            else
            {
                while(i < line.size()
                      && std::isspace(static_cast<UInt8>(line[i])) == 0)
                {
                    tok.push_back(line[i]);
                    ++i;
                }
            }
            out.w.push_back(std::move(tok));
        }
        return out;
    }

    // A color by name, by pin class, or as #RRGGBB.
    //
    // The names are the sixteen the whole app draws in, so a drawing cannot invent
    // a color that means nothing anywhere else. #RRGGBB is allowed because a
    // drawing sometimes depicts a real-world thing - a resistor band, a connector's
    // keying - whose color is a fact rather than a choice.
    Bool colorByName(const Str& name, ImU32& out)
    {
        struct Map
        {
            const Char* name;
            ImU32       col;
        };
        static const Map MAP[] = {
            { "black",   ui::ansi::BLACK     },
            { "red",     ui::ansi::RED       },
            { "green",   ui::ansi::GREEN     },
            { "yellow",  ui::ansi::YELLOW    },
            { "blue",    ui::ansi::BLUE      },
            { "magenta", ui::ansi::MAGENTA   },
            { "cyan",    ui::ansi::CYAN      },
            { "white",   ui::ansi::WHITE     },
            { "grey",    ui::ansi::GRAY      },
            { "grey",    ui::ansi::GRAY      },
            { "brRed",   ui::ansi::BRRED     },
            { "brGreen", ui::ansi::BRGREEN   },
            { "brYellow",ui::ansi::BRYELLOW  },
            { "brBlue",  ui::ansi::BRBLUE    },
            { "brCyan",  ui::ansi::BRCYAN    },
            { "brWhite", ui::ansi::BRWHITE   },
            { "grid",    ui::ansi::GRID      },
        };

        for(const auto& m : MAP)
        {
            if(name == m.name)
            {
                out = m.col;
                return true;
            }
        }

        // The pin classes, so a drawing and the pin table beside it agree.
        if(name == "power" || name == "ground" || name == "serial"
           || name == "digital" || name == "analog" || name == "usb"
           || name == "audio" || name == "unused")
        {
            out = classColor(name.c_str());
            return true;
        }

        if(name.size() == 7 && name[0] == '#')
        {
            UInt32 v = 0;
            for(Size i = 1; i < 7; ++i)
            {
                const Char c = name[i];
                UInt32     d = 0;
                if(c >= '0' && c <= '9')
                {
                    d = static_cast<UInt32>(c - '0');
                }
                else if(c >= 'a' && c <= 'f')
                {
                    d = static_cast<UInt32>(c - 'a' + 10);
                }
                else if(c >= 'A' && c <= 'F')
                {
                    d = static_cast<UInt32>(c - 'A' + 10);
                }
                else
                {
                    return false;
                }
                v = (v << 4) | d;
            }
            out = IM_COL32((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF, 0xFF);
            return true;
        }
        return false;
    }

    Void runDraw(const Node& n, Float32 measure)
    {
        Str src;
        for(const Node& k : n.kids)
        {
            if(k.isText())
            {
                src += k.text;
            }
        }

        const Float32 natW = std::max(1.0f, static_cast<Float32>(
                                          std::atof(n.attr("width", "640"))));
        const Float32 natH = std::max(1.0f, static_cast<Float32>(
                                          std::atof(n.attr("height", "320"))));

        // Scaled to the measure it is given, and the measure is purely a function
        // of the page's zoom - so a drawing grows by exactly the factor the text
        // around it grows by.
        //
        // NOT capped. It was clamped to 2x, which meant that past a certain zoom
        // the type kept growing and the diagrams stopped, which is the same
        // disorienting mismatch from the other end.
        const Float32 scale = measure / natW;

        const ImVec2 origin = ImGui::GetCursorScreenPos();
        ImDrawList*  dl     = ImGui::GetWindowDrawList();

        const auto at = [&](Float32 x, Float32 y)
        {
            return ImVec2(origin.x + (x * scale), origin.y + (y * scale));
        };
        const auto len = [&](Float32 v)
        {
            return v * scale;
        };

        Pen      pen;
        Vec<Str> errs;
        Int32    lineNo = 0;

        Size at0 = 0;
        while(at0 <= src.size())
        {
            const Size nl   = src.find('\n', at0);
            const Str  line = src.substr(at0, (nl == Str::npos) ? Str::npos
                                                               : (nl - at0));
            at0 = (nl == Str::npos) ? (src.size() + 1) : (nl + 1);
            ++lineNo;

            const Words a  = split(line);
            if(a.count() == 0)
            {
                continue;
            }
            const Str cmd = a.str(0);
            if(cmd.empty() || cmd[0] == '#')
            {
                continue;   // a comment
            }

            // Line thickness never falls below one pixel: a hairline that vanishes
            // at small scale is a drawing that loses parts of itself when the panel
            // is narrow.
            const Float32 th = std::max(1.0f, len(pen.width));

            if(cmd == "setColor" && a.count() >= 2)
            {
                if(!colorByName(a.str(1), pen.col))
                {
                    errs.push_back("line " + std::to_string(lineNo)
                                   + ": no color called " + a.str(1));
                }
            }
            else if(cmd == "setWidth" && a.count() >= 2)
            {
                pen.width = std::max(0.5f, a.num(1));
            }
            else if(cmd == "setTextSize" && a.count() >= 2)
            {
                pen.text = std::max(6.0f, a.num(1));
            }
            else if(cmd == "setTextAlign" && a.count() >= 2)
            {
                const Str w = a.str(1);
                if(w == "left")
                {
                    pen.align = -1;
                }
                else if(w == "centre" || w == "centre")
                {
                    pen.align = 0;
                }
                else if(w == "right")
                {
                    pen.align = 1;
                }
                else
                {
                    errs.push_back("line " + std::to_string(lineNo)
                                   + ": setTextAlign wants left, center or right");
                }
            }
            else if(cmd == "drawLine" && a.count() >= 5)
            {
                dl->AddLine(at(a.num(1), a.num(2)), at(a.num(3), a.num(4)),
                            pen.col, th);
            }
            else if(cmd == "drawRect" && a.count() >= 5)
            {
                dl->AddRect(at(a.num(1), a.num(2)), at(a.num(3), a.num(4)),
                            pen.col, 0.0f, 0, th);
            }
            else if(cmd == "fillRect" && a.count() >= 5)
            {
                dl->AddRectFilled(at(a.num(1), a.num(2)), at(a.num(3), a.num(4)),
                                  pen.col, 0.0f);
            }
            else if(cmd == "drawCircle" && a.count() >= 4)
            {
                dl->AddCircle(at(a.num(1), a.num(2)), len(a.num(3)), pen.col, 0, th);
            }
            else if(cmd == "fillCircle" && a.count() >= 4)
            {
                dl->AddCircleFilled(at(a.num(1), a.num(2)), len(a.num(3)), pen.col);
            }
            else if(cmd == "drawTriangle" && a.count() >= 7)
            {
                dl->AddTriangle(at(a.num(1), a.num(2)), at(a.num(3), a.num(4)),
                                at(a.num(5), a.num(6)), pen.col, th);
            }
            else if(cmd == "fillTriangle" && a.count() >= 7)
            {
                dl->AddTriangleFilled(at(a.num(1), a.num(2)), at(a.num(3), a.num(4)),
                                      at(a.num(5), a.num(6)), pen.col);
            }
            else if(cmd == "drawText" && a.count() >= 4)
            {
                ImFont* f = (ui::fonts.small != nullptr) ? ui::fonts.small
                                                         : ImGui::GetFont();
                const Float32 px = std::max(8.0f, len(pen.text));
                const Str     t  = a.str(3);

                // Measured, not estimated. The renderer knows the font; a document
                // does not and should not have to.
                Float32 dx = 0.0f;
                if(pen.align != -1)
                {
                    const Float32 w =
                        f->CalcTextSizeA(px, FLT_MAX, 0.0f, t.c_str()).x;
                    dx = (pen.align == 0) ? (-w * 0.5f) : -w;
                }

                ImVec2 p = at(a.num(1), a.num(2));
                p.x += dx;
                dl->AddText(f, px, p, pen.col, t.c_str());
            }
            else
            {
                // Named, not swallowed. A drawing quietly missing a wire is worse
                // than one that says which line it could not read.
                errs.push_back("line " + std::to_string(lineNo) + ": "
                               + (a.count() < 2 ? cmd : (cmd + " - wrong arguments")));
            }
        }

        ImGui::Dummy(ImVec2(natW * scale, natH * scale));

        for(const Str& e : errs)
        {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::BRRED),
                               "Draw: %s", e.c_str());
        }
        ImGui::Spacing();
    }

    // ---- <Table> --------------------------------------------------------------
    //
    // Added because the first three documents converted from the old hardcoded
    // pages all needed one and all had to fake it with a list. A wiring table is a
    // grid - a name, a pin, and what it is for, lining up down the page - and a
    // list of sentences is a worse way to read the same thing.
    //
    //   <Table>
    //     <Row head="yes"><Cell>module</Cell><Cell>Pico</Cell><Cell>why</Cell></Row>
    //     <Row><Cell class="serial">SCL</Cell><Cell>GP18</Cell><Cell>clock</Cell></Row>
    //   </Table>
    //
    // A cell takes inline markup like anything else, and a `class` colors it with
    // the same pin palette the pinouts use - so a row about a serial pin is the
    // same color as that pin in the diagram above it.
    Void table(const Node& n)
    {
        Int32 cols = 0;
        for(const Node& r : n.kids)
        {
            if(r.isText() || r.name != "Row")
            {
                continue;
            }
            Int32 c = 0;
            for(const Node& cell : r.kids)
            {
                if(!cell.isText() && cell.name == "Cell")
                {
                    ++c;
                }
            }
            cols = std::max(cols, c);
        }
        if(cols <= 0)
        {
            return;
        }

        ImGui::Spacing();

        // Borders, because the whole point of a table over a list is that the eye
        // can run down a column, and a column needs an edge to run along.
        if(ImGui::BeginTable("##bdoctable", cols,
                             ImGuiTableFlags_Borders
                             | ImGuiTableFlags_RowBg
                             | ImGuiTableFlags_SizingStretchProp))
        {
            for(const Node& r : n.kids)
            {
                if(r.isText() || r.name != "Row")
                {
                    continue;
                }

                const Bool head = r.hasAttr("head");
                ImGui::TableNextRow();

                for(const Node& cell : r.kids)
                {
                    if(cell.isText() || cell.name != "Cell")
                    {
                        continue;
                    }
                    ImGui::TableNextColumn();

                    const Char* cls = cell.attr("class", "");
                    const ImU32 col = head ? ui::ansi::BRWHITE
                                           : (cls[0] != '\0' ? classColor(cls)
                                                             : ui::ansi::WHITE);

                    Vec<Run> runs;
                    collect(cell, runs, col, false);
                    drawRuns(runs, std::max(60.0f,
                                            ImGui::GetContentRegionAvail().x),
                             0.0f);
                }
            }
            ImGui::EndTable();
        }

        ImGui::Spacing();
    }

    Void pinout(const Node& n, Float32 width)
    {
        const Char* title = n.attr("title", "Pinout");
        const Char* pkg   = n.attr("package", "");
        const Char* pitch = n.attr("pitch", "");

        ImGui::Spacing();
        headingText(ui::fonts.title, ui::ansi::BRCYAN, title);

        const ImVec2 ph = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddLine(
            ImVec2(ph.x, ph.y), ImVec2(ph.x + width, ph.y), ui::ansi::GRID, 1.0f);
        ImGui::Spacing();

        if(pkg[0] != '\0' || pitch[0] != '\0')
        {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GRAY),
                               "%s%s%s", pkg,
                               (pkg[0] != '\0' && pitch[0] != '\0') ? "   " : "",
                               pitch);
            ImGui::Spacing();
        }

        for(const Node& k : n.kids)
        {
            if(!k.isText() && k.name == "Note")
            {
                banner(k, width, ui::ansi::YELLOW);
            }
        }

        // Two columns side by side when there is room, stacked when there is not.
        // A pinout squeezed into two 90-pixel columns is less readable than the
        // same pinout in one, and the panel is resizable.
        Vec<const Node*> cols;
        for(const Node& k : n.kids)
        {
            if(!k.isText() && k.name == "Column")
            {
                cols.push_back(&k);
            }
        }
        if(cols.empty())
        {
            return;
        }

        const Bool side = (width > 640.0f) && (cols.size() == 2);

        if(side && ImGui::BeginTable("##pins", 2, ImGuiTableFlags_SizingStretchSame))
        {
            ImGui::TableNextRow();
            for(const Node* c : cols)
            {
                ImGui::TableNextColumn();
                const Char* s = c->attr("side", "");
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GRAY),
                                   "%s  %s - %s", s, c->attr("from", ""),
                                   c->attr("to", ""));
                ImGui::Spacing();
                pinColumn(*c);
            }
            ImGui::EndTable();
        }
        else
        {
            for(const Node* c : cols)
            {
                const Char* s = c->attr("side", "");
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GRAY),
                                   "%s  %s - %s", s, c->attr("from", ""),
                                   c->attr("to", ""));
                ImGui::Spacing();
                pinColumn(*c);
                ImGui::Spacing();
            }
        }

        ImGui::Spacing();
    }

    Void body(const Node& n, Float32 width);

    Void section(const Node& n, Float32 width)
    {
        const Char* title = n.attr("title", "");
        if(title[0] != '\0')
        {
            ImGui::Spacing();
            headingText(ui::fonts.title, ui::ansi::BRWHITE, title);

            // A rule under the heading rather than through it. SeparatorText draws
            // the label at the body size and cannot be told otherwise, which is
            // exactly the thing being fixed.
            const ImVec2 p = ImGui::GetCursorScreenPos();
            ImGui::GetWindowDrawList()->AddLine(
                ImVec2(p.x, p.y), ImVec2(p.x + width, p.y), ui::ansi::GRID, 1.0f);
            ImGui::Spacing();
        }
        body(n, width);
    }

    Void body(const Node& n, Float32 width)
    {
        for(const Node& k : n.kids)
        {
            if(k.isText())
            {
                continue;   // stray text between blocks is whitespace
            }

            if(k.name == "Section")
            {
                section(k, width);
            }
            else if(k.name == "Pinout")
            {
                pinout(k, width);
            }
            else if(k.name == "Text")
            {
                paragraph(k, width, ui::ansi::WHITE, 0.0f);
            }
            else if(k.name == "Note")
            {
                banner(k, width, ui::ansi::YELLOW);
            }
            else if(k.name == "Warn")
            {
                banner(k, width, ui::ansi::BRYELLOW);
            }
            else if(k.name == "Code")
            {
                codeBlock(k, width);
            }
            else if(k.name == "Draw")
            {
                runDraw(k, width);
            }
            else if(k.name == "Table")
            {
                table(k);
            }
            else if(k.name == "List")
            {
                for(const Node& it : k.kids)
                {
                    if(it.isText() || it.name != "Item")
                    {
                        continue;
                    }
                    ImGui::TextColored(
                        ImGui::ColorConvertU32ToFloat4(ui::ansi::BRCYAN), "-");
                    ImGui::SameLine(0.0f, 8.0f);
                    Vec<Run> runs;
                    collect(it, runs, ui::ansi::WHITE, false);
                    drawRuns(runs, std::max(120.0f,
                                            ImGui::GetContentRegionAvail().x),
                             0.0f);
                    ImGui::Spacing();
                }
                ImGui::Spacing();
            }
            else
            {
                // An element the renderer does not know. Drawn as its text rather
                // than dropped, so a typo in a tag name loses the styling and not
                // the sentence.
                paragraph(k, width, ui::ansi::GRAY, 0.0f);
            }
        }
    }

  }

  // ---------------------------------------------------------------------------

  const Char* Node::attr(const Char* want, const Char* fallback) const
  {
      for(const Attr& a : attrs)
      {
          if(a.name == want)
          {
              return a.value.c_str();
          }
      }
      return fallback;
  }

  Bool Node::hasAttr(const Char* want) const
  {
      for(const Attr& a : attrs)
      {
          if(a.name == want)
          {
              return true;
          }
      }
      return false;
  }

  Bool isDocPath(const Str& path)
  {
      const Size n = std::strlen(EXT);
      if(path.size() <= n)
      {
          return false;
      }
      return _stricmp(path.c_str() + (path.size() - n), EXT) == 0;
  }

  namespace
  {

    // The whole of one file: skip to the root element, parse it, insist it is a
    // <Doc>. Shared by the public entry point and by every include, which is what
    // makes an included file exactly the same kind of thing as a top-level one.
    Bool parseDoc(const Str& text, const Ctx& ctx, Node& out, Str& err, Int32& errLine)
    {
        Cursor cur;
        cur.src = &text;

        for(;;)
        {
            skipSpace(cur);
            if(cur.done())
            {
                err     = "the file has no <Doc> in it";
                errLine = 1;
                return false;
            }
            if(cur.starts("<!--"))
            {
                while(!cur.done() && !cur.starts("-->"))
                {
                    cur.take();
                }
                for(Int32 i = 0; i < 3 && !cur.done(); ++i)
                {
                    cur.take();
                }
                continue;
            }
            break;
        }

        if(!parseNode(cur, out, err, errLine, ctx))
        {
            return false;
        }

        if(out.name != "Doc")
        {
            err     = "the root element is <" + out.name + ">, not <Doc>";
            errLine = out.line;
            return false;
        }
        return true;
    }

  }

  Doc parse(const Str& text, const Str& baseDir)
  {
      Ctx ctx;
      ctx.baseDir = baseDir;
      ctx.depth   = 0;

      Doc d;
      if(!parseDoc(text, ctx, d.root, d.error, d.errorLine))
      {
          return d;
      }
      return d;
  }

  namespace
  {

    Void checkNode(const Node& n, Vec<Str>& out)
    {
        if(!n.isText())
        {
            if(std::isupper(static_cast<UInt8>(n.name[0])) == 0)
            {
                out.push_back("element <" + n.name + "> is not PascalCase (line "
                              + std::to_string(n.line) + ")");
            }
            for(const Attr& a : n.attrs)
            {
                if(std::isupper(static_cast<UInt8>(a.name[0])) != 0)
                {
                    out.push_back("attribute " + a.name + " is not camelCase (line "
                                  + std::to_string(n.line) + ")");
                }
            }
            if(n.name == "Pin" && !n.hasAttr("name"))
            {
                out.push_back("a <Pin> with no name attribute (line "
                              + std::to_string(n.line) + ")");
            }
        }
        for(const Node& k : n.kids)
        {
            checkNode(k, out);
        }
    }

  }

  Vec<Str> check(const Doc& d)
  {
      Vec<Str> out;
      if(d.ok())
      {
          checkNode(d.root, out);
      }
      return out;
  }

  Void drawPage(const Doc& d, const ImVec2& size, View& view, Float32 dpiScale)
  {
      // ---------------------------------------------------------------------
      // NO SCROLLING. The pan is an offset applied to the content, and it is
      // deliberately unbounded.
      //
      // This used to drag with SetScrollX/SetScrollY, and ImGui CLAMPS those to
      // [0, ScrollMax] by definition - so the page hit a wall at the top-left
      // and could never be pushed past its own edges. That is right for a
      // scrolled column of prose and wrong for these: a .bdoc is mostly a
      // diagram, and a diagram is a canvas. Wanting to drag the corner of a
      // pinout into the middle of the panel to look at it is an ordinary thing
      // to want, and the scroll range forbade it.
      //
      // So the child does not scroll at all - the content is drawn from a
      // cursor offset by the pan, which can be any value in any direction. The
      // scrollbars go with it, which is the right trade: a scrollbar on an
      // infinite canvas is measuring nothing.
      //
      // THE COST is that you can pan the page entirely out of sight, and an
      // empty panel looks like a broken renderer. Double-click puts it back -
      // see below - and that is the whole reason this is not just an unclamped
      // number.
      // ---------------------------------------------------------------------
      ImGui::BeginChild("##bdocpage", size, ImGuiChildFlags_None,
                        ImGuiWindowFlags_NoScrollbar
                        | ImGuiWindowFlags_NoScrollWithMouse);

      const Bool over = ImGui::IsWindowHovered(
          ImGuiHoveredFlags_ChildWindows
          | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);

      const ImVec2 origin = ImGui::GetCursorScreenPos();

      // THE WHEEL ZOOMS, and dragging pans. No modifier.
      //
      // It was Ctrl+wheel with the wheel scrolling, which is right for a wall of
      // prose and wrong for this: most of what is in these documents is a
      // diagram, and the thing you want to do to a diagram is get closer to it.
      // Dragging covers what the wheel used to.
      if(over)
      {
          const Float32 wheel = ImGui::GetIO().MouseWheel;
          if(wheel != 0.0f)
          {
              // ZOOM ABOUT THE CURSOR, not about the top-left corner.
              //
              // With an unbounded pan this stops being a nicety. Zooming about
              // the corner moves whatever you were looking at away from the
              // pointer, and on a canvas you have already dragged somewhere
              // that means chasing the diagram across the panel with a wheel.
              // Holding the point under the mouse still is what makes zoom and
              // pan feel like one gesture rather than two that fight.
              //
              // The page is not a pure scale - text reflows against `measure` -
              // so this is exact for drawings and an approximation for
              // paragraphs. The approximation is invisible; the alternative was
              // not.
              const Float32 was = view.zoom;
              const Float32 now = std::min(4.0f,
                                           std::max(0.4f,
                                                    was * std::pow(1.15f, wheel)));

              if(now != was)
              {
                  const ImVec2 m = ImGui::GetIO().MousePos;

                  // Where the cursor is over the CONTENT, in unzoomed units.
                  const Float32 cx = (m.x - origin.x - view.panX) / was;
                  const Float32 cy = (m.y - origin.y - view.panY) / was;

                  // Put that same content point back under the cursor.
                  view.panX = m.x - origin.x - (cx * now);
                  view.panY = m.y - origin.y - (cy * now);
                  view.zoom = now;
              }
          }

          // A drag STARTS here, but it does not end here - see below.
          if(ImGui::IsMouseClicked(ImGuiMouseButton_Left))
          {
              view.panning = true;
          }

          // THE WAY HOME. An unbounded pan can put the page somewhere with no
          // edge to find it by, and an empty panel is indistinguishable from a
          // renderer that failed. Double-click is the gesture every canvas uses
          // for this and it costs nothing to offer.
          if(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
          {
              view = View();
          }
      }

      // ---------------------------------------------------------------------
      // THE DRAG CONTINUES OUTSIDE THE PANEL, and this is deliberately not
      // inside the `over` test above.
      //
      // It was, and the pan stopped dead the moment the cursor crossed the
      // panel edge mid-drag - which is precisely when you are dragging a
      // diagram in from off-screen and have the furthest to go. Measured: a
      // 720-pixel drag to the upper left moved the page 262 and then stopped,
      // because the cursor had left the pane and `over` went false.
      //
      // Latched on the press and held until the release, so where the mouse
      // wanders in between does not matter. That is what every map, canvas and
      // image viewer does, and the reason it is the convention is that a hand
      // dragging paper does not let go when it passes the edge of the desk.
      //
      // `+=`, not `-=`: the content follows the hand rather than running away
      // from it. A scrollbar is the one that moves the other way.
      // ---------------------------------------------------------------------
      if(!ImGui::IsMouseDown(ImGuiMouseButton_Left))
      {
          view.panning = false;
      }
      else if(view.panning)
      {
          view.panX += ImGui::GetIO().MouseDelta.x;
          view.panY += ImGui::GetIO().MouseDelta.y;
      }

      // PushFont, not SetWindowFontScale.
      //
      // SetWindowFontScale is obsolete in ImGui 1.92 - imgui.cpp calls it "not
      // useful anymore" and imgui.h says to prefer PushFont(NULL, size). It
      // still COMPILES, which is why this looked right and did nothing: the
      // page never zoomed and the only evidence was that nothing happened.
      //
      // NULL keeps the current face and changes only the size.
      const Float32 baseSize = ImGui::GetFontSize();
      ImGui::PushFont(nullptr, baseSize * view.zoom);

      // ONE SCALE FOR EVERYTHING, and this line is the whole fix.
      //
      // The measure used to be clamped to the panel width. Text was scaled by the
      // font push and drawings were scaled by measure/naturalWidth - so the
      // moment the panel was the smaller of the two, zooming grew the type and
      // left every diagram exactly where it was. Text and pictures pulling apart
      // as you zoom is disorienting in a way that is hard to even name while it
      // is happening.
      //
      // Unclamped, the measure is purely a function of zoom, so the drawing scale
      // and the font scale are the same number. The page is allowed to be wider
      // than its frame; that is what the horizontal scrollbar and dragging are
      // for.
      // The BASE measure fits the panel; zoom multiplies it. Both halves matter.
      //
      // Fitting first means a page at 1x wraps inside its frame and needs no
      // horizontal scrolling to read - which is the ordinary case and was broken
      // when the measure ignored the panel entirely. Multiplying by zoom after
      // means the drawing scale, measure/naturalWidth, carries exactly the same
      // factor as the font, so text and diagrams grow together.
      const Float32 avail   = ImGui::GetContentRegionAvail().x;
      const Float32 base    = std::min(std::max(avail - 24.0f, 200.0f),
                                       780.0f * dpiScale);
      const Float32 measure = base * view.zoom;
      const Float32 pad     = std::max(0.0f, (avail - measure) * 0.5f);

      // THE PAN IS APPLIED HERE, and the two axes need DIFFERENT mechanisms.
      //
      // Y is a cursor move: vertical position accumulates down the window, so
      // placing the first item lower carries everything after it.
      //
      // X IS NOT. After every item ImGui resets CursorPos.x to the window's
      // indent, so setting the cursor's X moves only the FIRST thing drawn and
      // every line after it snaps back. That is exactly what happened on the
      // first attempt here: vertical panning worked, horizontal did nothing at
      // all, and the page looked like it was obeying one axis out of stubborn-
      // ness. Measured across four frames - the title's top edge moved 122 ->
      // 322 while its left edge sat at 623 in every one of them.
      //
      // So X goes through the INDENT, which is the thing those per-item resets
      // read from. It folds in with the centering pad because they are the same
      // quantity: how far right the content starts.
      //
      // Both are unbounded and negative values are the point - ImGui clips
      // items to the window rather than refusing to place them, so a page
      // dragged up and left simply has its corner off-panel. That is what the
      // old SetScroll pair could not express.
      const Float32 shift = view.panX + ((pad > 1.0f) ? pad : 0.0f);

      ImGui::SetCursorPosY(ImGui::GetCursorPosY() + view.panY);

      if(shift != 0.0f)
      {
          ImGui::Indent(shift);
      }
      draw(d, measure);
      if(shift != 0.0f)
      {
          ImGui::Unindent(shift);
      }

      // Popped before the child closes. A font left pushed across EndChild is
      // the unbalanced-stack bug that renders a window blank white.
      ImGui::PopFont();
      ImGui::EndChild();
  }

  Void draw(const Doc& d, Float32 width)
  {
      if(!d.ok())
      {
          // The error where the page would have been, with the line. A blank
          // panel cannot be told from an empty document.
          ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::BRRED),
                             "this document does not parse");
          ImGui::Spacing();
          ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::WHITE),
                             "line %d: %s", d.errorLine, d.error.c_str());
          ImGui::Spacing();
          ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GRAY),
                             "Switch to Source above to see it.");
          return;
      }

      const Char* title = d.root.attr("title", "");
      const Char* sub   = d.root.attr("subtitle", "");

      if(title[0] != '\0')
      {
          headingText(ui::fonts.stat, ui::ansi::BRCYAN, title);
      }
      if(sub[0] != '\0' || d.root.hasAttr("category"))
      {
          if(ui::fonts.small != nullptr)
          {
              ImGui::PushFont(ui::fonts.small, 0.0f);
          }

          // The category, if the document declares one. It had been parsed and
          // then ignored - an attribute that documents write and nothing reads
          // is an attribute that quietly becomes wrong.
          const Char* cat = d.root.attr("category", "");
          if(cat[0] != '\0')
          {
              ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::BRCYAN),
                                 "%s", cat);
              if(sub[0] != '\0')
              {
                  ImGui::SameLine(0.0f, 8.0f);
                  ImGui::TextColored(
                      ImGui::ColorConvertU32ToFloat4(ui::ansi::GRID), "|");
                  ImGui::SameLine(0.0f, 8.0f);
              }
          }
          if(sub[0] != '\0')
          {
              ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GRAY),
                                 "%s", sub);
          }

          if(ui::fonts.small != nullptr)
          {
              ImGui::PopFont();
          }
      }
      ImGui::Spacing();
      ImGui::Separator();

      body(d.root, width);
  }

}
