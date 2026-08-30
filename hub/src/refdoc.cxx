// The .bdoc parser and renderer. See refdoc.hxx for the format and the rules.

#include "shared.hxx"

#include "refdoc.hxx"
#include "theme.hxx"

#include <algorithm>
#include <cctype>
#include <cstring>

namespace refdoc {

namespace {

// ---------------------------------------------------------------------------
//  parsing
// ---------------------------------------------------------------------------

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

Bool parseNode(Cursor& cur, Node& out, Str& err, Int32& errLine);

// Everything up to the matching close tag.
Bool parseChildren(Cursor& cur, Node& out, Str& err, Int32& errLine)
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
            if(!parseNode(cur, kid, err, errLine))
            {
                return false;
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

Bool parseNode(Cursor& cur, Node& out, Str& err, Int32& errLine)
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
    if(!parseChildren(cur, out, err, errLine))
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
        ImGui::PushFont(f);
    }
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(col));
    ImGui::TextUnformatted(text);
    ImGui::PopStyleColor();
    if(f != nullptr)
    {
        ImGui::PopFont();
    }
}

ImU32 classColour(const Char* cls)
{
    struct Map
    {
        const Char* name;
        ImU32       col;
    };

    // The same sixteen the rest of the app draws in, so a pinout here and a
    // plot on the next screen mean the same thing by the same colour.
    static const Map MAP[] = {
        { "power",   ui::ansi::RED      },
        { "ground",  ui::ansi::GREY     },
        { "serial",  ui::ansi::BRCYAN   },
        { "digital", ui::ansi::GREEN    },
        { "analog",  ui::ansi::MAGENTA  },
        { "usb",     ui::ansi::BLUE     },
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
            collect(k, out, ui::ansi::GREY, mono);
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
// ImGui has no rich text, so a paragraph that mixes colours cannot be one
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
                ImGui::PushFont(ui::fonts.mono);
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
// differ only in colour and in how much they are asking for your attention.
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

    // A block keeps its newlines, unlike every other text in this format. It is
    // showing the shape of something, and the shape is the point.
    const ImVec2 p0 = ImGui::GetCursorScreenPos();

    if(ui::fonts.mono != nullptr)
    {
        ImGui::PushFont(ui::fonts.mono);
    }
    ImGui::Indent(10.0f);
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::ColorConvertU32ToFloat4(ui::ansi::BRCYAN));
    ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + width - 20.0f);
    ImGui::TextUnformatted(body.c_str());
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
    ImGui::Unindent(10.0f);
    if(ui::fonts.mono != nullptr)
    {
        ImGui::PopFont();
    }

    const ImVec2 p1 = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddRect(
        ImVec2(p0.x, p0.y - 2.0f), ImVec2(p0.x + width, p1.y),
        ui::ansi::GRID, 0.0f, 0, 1.0f);

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

        const ImU32 col32 = classColour(cls);

        // The pad number is always dim and always first: it is the thing you
        // count along the package with, and it is never the interesting part.
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GREY),
                           "%3s", num);
        ImGui::SameLine(0.0f, 8.0f);

        if(ui::fonts.mono != nullptr)
        {
            ImGui::PushFont(ui::fonts.mono);
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
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GREY),
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
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GREY),
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
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GREY),
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
            paragraph(k, width, ui::ansi::GREY, 0.0f);
        }
    }
}

} // namespace

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

Doc parse(const Str& text)
{
    Doc    d;
    Cursor cur;
    cur.src = &text;

    for(;;)
    {
        skipSpace(cur);
        if(cur.done())
        {
            d.error     = "the file has no <Doc> in it";
            d.errorLine = 1;
            return d;
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

    if(!parseNode(cur, d.root, d.error, d.errorLine))
    {
        return d;
    }

    if(d.root.name != "Doc")
    {
        d.error     = "the root element is <" + d.root.name + ">, not <Doc>";
        d.errorLine = d.root.line;
    }
    return d;
}

namespace {

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

} // namespace

Vec<Str> check(const Doc& d)
{
    Vec<Str> out;
    if(d.ok())
    {
        checkNode(d.root, out);
    }
    return out;
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
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GREY),
                           "Switch to Source above to see it.");
        return;
    }

    const Char* title = d.root.attr("title", "");
    const Char* sub   = d.root.attr("subtitle", "");

    if(title[0] != '\0')
    {
        headingText(ui::fonts.stat, ui::ansi::BRCYAN, title);
    }
    if(sub[0] != '\0')
    {
        if(ui::fonts.small != nullptr)
        {
            ImGui::PushFont(ui::fonts.small);
        }
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::ansi::GREY),
                           "%s", sub);
        if(ui::fonts.small != nullptr)
        {
            ImGui::PopFont();
        }
    }
    ImGui::Spacing();
    ImGui::Separator();

    body(d.root, width);
}

} // namespace refdoc
