#include "editor.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace ed {
namespace {

constexpr Size MAX_UNDO = 256;

Bool wordChar(Char c) noexcept
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
        || (c >= '0' && c <= '9') || c == '_';
}

Bool space(Char c) noexcept
{
    return c == ' ' || c == '\t';
}

// The closer for an opener, or 0. Quotes are their own closer, which is why the
// caller has to check "am I already on one" before pairing them - see
// insertChar().
Char closerFor(Char c) noexcept
{
    switch(c)
    {
    case '(':  return ')';
    case '[':  return ']';
    case '{':  return '}';
    case '"':  return '"';
    case '\'': return '\'';
    default:   return 0;
    }
}

Bool isCloser(Char c) noexcept
{
    return c == ')' || c == ']' || c == '}' || c == '"' || c == '\'';
}

Bool before(const Cursor& a, const Cursor& b) noexcept
{
    return (a.line != b.line) ? (a.line < b.line) : (a.col < b.col);
}

} // namespace

// ---------------------------------------------------------------------------

Editor::Editor()
{
    lines.push_back(Str());
}

Void Editor::setText(const Str& t)
{
    lines.clear();

    Str acc;
    for(Size i = 0; i < t.size(); ++i)
    {
        const Char c = t[i];
        if(c == '\r')
            continue;                    // CRLF in, LF held internally
        if(c == '\n') { lines.push_back(acc); acc.clear(); continue; }
        acc.push_back(c);
    }
    lines.push_back(acc);

    cur = Cursor();
    md  = Mode::MODE_NORMAL;
    undoStack.clear();
    redoStack.clear();
    pendCount = 0;
    pendOp    = 0;
    cmdLine.clear();
    dirtyFlag = false;
}

Str Editor::text() const
{
    Str out;
    for(Size i = 0; i < lines.size(); ++i)
    {
        out += lines[i];
        if(i + 1 < lines.size())
            out.push_back('\n');
    }
    return out;
}

Int32 Editor::lineCount() const
{
    return static_cast<Int32>(lines.size());
}

const Str& Editor::line(Int32 i) const
{
    static const Str empty;
    if(i < 0 || i >= lineCount())
        return empty;
    return lines[static_cast<Size>(i)];
}

Void Editor::setCursor(Int32 l, Int32 c)
{
    cur.line = l;
    cur.col  = c;
    clampCursor();
}

Void Editor::setMode(Mode m)
{
    md = m;
    if(m != Mode::MODE_COMMAND)
        cmdLine.clear();
    if(m == Mode::MODE_NORMAL)
    {
        pendCount = 0;
        pendOp    = 0;
    }
    clampCursor();
}

Str Editor::takeSubmittedCommand()
{
    Str s;
    s.swap(submitted);
    return s;
}

Str Editor::takeMessage()
{
    Str s;
    s.swap(message);
    return s;
}

Bool Editor::selection(Cursor& from, Cursor& to) const
{
    if(md != Mode::MODE_VISUAL && md != Mode::MODE_VISUAL_LINE)
        return false;

    Cursor a = visAnchor;
    Cursor b = cur;
    if(before(b, a))
        std::swap(a, b);

    if(md == Mode::MODE_VISUAL_LINE)
    {
        a.col = 0;
        b.col = static_cast<Int32>(line(b.line).size());
    }
    else
    {
        // Visual mode is inclusive of the character under the caret; the
        // range this returns is half-open, so the end moves on by one.
        b.col = std::min(b.col + 1, static_cast<Int32>(line(b.line).size()));
    }

    from = a;
    to   = b;
    return true;
}

Void Editor::clampCursor()
{
    if(lines.empty())
        lines.push_back(Str());

    cur.line = std::max(0, std::min(cur.line, lineCount() - 1));

    const Int32 len = static_cast<Int32>(lines[static_cast<Size>(cur.line)].size());

    // Normal and visual mode sit ON a character, so the last valid column is
    // len-1. Insert mode sits BETWEEN them, so len is valid. Getting this wrong
    // is what makes an editor feel subtly broken at the end of a line.
    const Int32 maxCol = (md == Mode::MODE_INSERT) ? len : std::max(0, len - 1);
    cur.col = std::max(0, std::min(cur.col, maxCol));
}

Int32 Editor::indentOf(Int32 lineIdx) const
{
    const Str& s = line(lineIdx);
    Int32 i = 0;
    while(i < static_cast<Int32>(s.size()) && space(s[static_cast<Size>(i)]))
        ++i;
    return i;
}

Void Editor::pushUndo()
{
    Snapshot s;
    s.lines = lines;
    s.cur   = cur;
    undoStack.push_back(std::move(s));
    if(undoStack.size() > MAX_UNDO)
        undoStack.erase(undoStack.begin());

    redoStack.clear();   // a new edit invalidates the redo branch
    dirtyFlag = true;
}

Void Editor::undo()
{
    if(undoStack.empty())
    {
        message = "already at oldest change";
        return;
    }
    Snapshot now;
    now.lines = lines;
    now.cur   = cur;
    redoStack.push_back(std::move(now));

    lines = undoStack.back().lines;
    cur   = undoStack.back().cur;
    undoStack.pop_back();
    dirtyFlag = true;
    clampCursor();
}

Void Editor::redo()
{
    if(redoStack.empty())
    {
        message = "already at newest change";
        return;
    }
    Snapshot now;
    now.lines = lines;
    now.cur   = cur;
    undoStack.push_back(std::move(now));

    lines = redoStack.back().lines;
    cur   = redoStack.back().cur;
    redoStack.pop_back();
    dirtyFlag = true;
    clampCursor();
}

// ---------------------------------------------------------------- insert mode

Void Editor::insertChar(Char c)
{
    Str&        s    = lines[static_cast<Size>(cur.line)];
    const Char  next = (cur.col < static_cast<Int32>(s.size()))
                     ? s[static_cast<Size>(cur.col)] : '\0';

    // Typing the closer that is already sitting there moves over it instead of
    // producing `))`. This is the half of auto-closing that makes it bearable -
    // without it every pair costs a keystroke to escape.
    if(isCloser(c) && next == c)
    {
        ++cur.col;
        return;
    }

    const Char close = closerFor(c);

    // A quote only opens a pair when it is not being used to close a word, so
    // typing an apostrophe in a comment does not spray '' everywhere.
    Bool pair = (close != 0);
    if(c == '"' || c == '\'')
    {
        const Char prev = (cur.col > 0) ? s[static_cast<Size>(cur.col - 1)] : '\0';
        if(wordChar(prev) || prev == '\\')
            pair = false;
    }
    else if(close != 0)
    {
        // Only pair a bracket when what follows is whitespace, a closer, or the
        // end of the line. Typing `(` before existing text usually means
        // wrapping it, and an injected `)` would land in the wrong place.
        if(next != '\0' && !space(next) && !isCloser(next) && next != ',')
            pair = false;
    }

    s.insert(static_cast<Size>(cur.col), 1, c);
    ++cur.col;

    if(pair)
        s.insert(static_cast<Size>(cur.col), 1, close);

    // A closing brace typed on a line that is otherwise blank re-indents to
    // match its opener's level. This is the one piece of "smart" indentation
    // here, and it is the one that people actually notice missing.
    if(c == '}')
    {
        Int32 firstNonSpace = 0;
        while(firstNonSpace < static_cast<Int32>(s.size())
              && space(s[static_cast<Size>(firstNonSpace)]))
            ++firstNonSpace;

        if(firstNonSpace == cur.col - 1)
        {
            // Walk back for the matching opener and copy its indent.
            Int32 depth = 0;
            for(Int32 l = cur.line; l >= 0; --l)
            {
                const Str& t   = lines[static_cast<Size>(l)];
                const Int32 from = (l == cur.line) ? cur.col - 2
                                                   : static_cast<Int32>(t.size()) - 1;
                for(Int32 i = from; i >= 0; --i)
                {
                    if(t[static_cast<Size>(i)] == '}') ++depth;
                    else if(t[static_cast<Size>(i)] == '{')
                    {
                        if(depth == 0)
                        {
                            const Int32 want = indentOf(l);
                            s.erase(0, static_cast<Size>(firstNonSpace));
                            s.insert(0, static_cast<Size>(want), ' ');
                            cur.col += want - firstNonSpace;
                            return;
                        }
                        --depth;
                    }
                }
            }
        }
    }
}

Void Editor::newlineWithIndent()
{
    Str&        s    = lines[static_cast<Size>(cur.line)];
    const Int32 ind  = indentOf(cur.line);
    const Char  prev = (cur.col > 0) ? s[static_cast<Size>(cur.col - 1)] : '\0';
    const Char  next = (cur.col < static_cast<Int32>(s.size()))
                     ? s[static_cast<Size>(cur.col)] : '\0';

    Str tail = s.substr(static_cast<Size>(cur.col));
    s.erase(static_cast<Size>(cur.col));

    const Int32 newInd = (prev == '{') ? ind + INDENT : ind;

    // Enter pressed between { and } opens the block out into three lines with
    // the caret on the middle one, which is what every editor does and what the
    // hands expect.
    if(prev == '{' && next == '}')
    {
        lines.insert(lines.begin() + cur.line + 1, Str(static_cast<Size>(newInd), ' '));
        lines.insert(lines.begin() + cur.line + 2,
                     Str(static_cast<Size>(ind), ' ') + tail);
        cur.line += 1;
        cur.col   = newInd;
        return;
    }

    // Leading whitespace on the tail is dropped; the new indent replaces it.
    Size cut = 0;
    while(cut < tail.size() && space(tail[cut]))
        ++cut;
    tail.erase(0, cut);

    lines.insert(lines.begin() + cur.line + 1,
                 Str(static_cast<Size>(newInd), ' ') + tail);
    cur.line += 1;
    cur.col   = newInd;
}

Void Editor::backspace()
{
    if(cur.col > 0)
    {
        Str&       s    = lines[static_cast<Size>(cur.line)];
        const Char left = s[static_cast<Size>(cur.col - 1)];
        const Char right = (cur.col < static_cast<Int32>(s.size()))
                         ? s[static_cast<Size>(cur.col)] : '\0';

        // Deleting the opener of an empty pair takes the closer with it,
        // otherwise auto-closing leaves litter behind every correction.
        if(closerFor(left) != 0 && closerFor(left) == right)
        {
            s.erase(static_cast<Size>(cur.col - 1), 2);
            --cur.col;
            return;
        }

        // Inside leading whitespace, backspace removes a whole indent level.
        Int32 firstNonSpace = 0;
        while(firstNonSpace < static_cast<Int32>(s.size())
              && space(s[static_cast<Size>(firstNonSpace)]))
            ++firstNonSpace;

        if(cur.col <= firstNonSpace && cur.col >= INDENT)
        {
            const Int32 back = ((cur.col % INDENT) == 0) ? INDENT : (cur.col % INDENT);
            s.erase(static_cast<Size>(cur.col - back), static_cast<Size>(back));
            cur.col -= back;
            return;
        }

        s.erase(static_cast<Size>(cur.col - 1), 1);
        --cur.col;
        return;
    }

    if(cur.line == 0)
        return;

    // Join with the line above.
    const Int32 prevLen = static_cast<Int32>(lines[static_cast<Size>(cur.line - 1)].size());
    lines[static_cast<Size>(cur.line - 1)] += lines[static_cast<Size>(cur.line)];
    lines.erase(lines.begin() + cur.line);
    cur.line -= 1;
    cur.col   = prevLen;
}

Void Editor::insertText(const Str& s)
{
    if(s.empty())
        return;

    pushUndo();
    const Mode saved = md;
    md = Mode::MODE_INSERT;      // so clampCursor allows the past-the-end column

    for(Size i = 0; i < s.size(); ++i)
    {
        const Char c = s[i];
        if(c == '\r')
            continue;
        if(c == '\n')
        {
            // A pasted newline must NOT re-indent: the text already carries its
            // own indentation and doing both doubles it.
            Str& ln  = lines[static_cast<Size>(cur.line)];
            Str  tail = ln.substr(static_cast<Size>(cur.col));
            ln.erase(static_cast<Size>(cur.col));
            lines.insert(lines.begin() + cur.line + 1, tail);
            cur.line += 1;
            cur.col   = 0;
            continue;
        }
        lines[static_cast<Size>(cur.line)].insert(static_cast<Size>(cur.col), 1, c);
        ++cur.col;
    }

    md = saved;
    clampCursor();
}

Str Editor::wordBeforeCursor() const
{
    const Str&  s = line(cur.line);
    const Int32 n = std::min(cur.col, static_cast<Int32>(s.size()));

    Int32 i = n;
    while(i > 0 && wordChar(s[static_cast<Size>(i - 1)]))
        --i;

    // A run starting with a digit is a number being typed, not a name.
    if(i < n && s[static_cast<Size>(i)] >= '0' && s[static_cast<Size>(i)] <= '9')
        return Str();

    return s.substr(static_cast<Size>(i), static_cast<Size>(n - i));
}

Void Editor::replaceWordBeforeCursor(const Str& with)
{
    const Str word = wordBeforeCursor();
    if(word.empty())
        return;

    pushUndo();

    Str&        s     = lines[static_cast<Size>(cur.line)];
    const Int32 start = cur.col - static_cast<Int32>(word.size());
    s.erase(static_cast<Size>(start), word.size());
    s.insert(static_cast<Size>(start), with);
    cur.col = start + static_cast<Int32>(with.size());
    clampCursor();
}

// -------------------------------------------------------------------- motions

Bool Editor::motion(Char c, Int32 count, Cursor& out, Bool& linewise)
{
    Cursor p = cur;
    linewise = false;
    const Int32 n = std::max(1, count);

    switch(c)
    {
    case 'h':
        p.col = std::max(0, p.col - n);
        break;

    case 'l':
        p.col = p.col + n;
        break;

    case 'k':
        p.line   = std::max(0, p.line - n);
        linewise = true;
        break;

    case 'j':
        p.line   = std::min(lineCount() - 1, p.line + n);
        linewise = true;
        break;

    case '0':
        p.col = 0;
        break;

    case '$':
        p.col = std::max(0, static_cast<Int32>(line(p.line).size()) - 1);
        break;

    case '^':
    {
        p.col = indentOf(p.line);
        break;
    }

    case 'G':
        // With a count, G is "go to line N"; without, "go to the last line".
        p.line   = (count > 0) ? std::min(lineCount() - 1, count - 1)
                               : lineCount() - 1;
        p.col    = indentOf(p.line);
        linewise = true;
        break;

    case 'w':
    {
        for(Int32 r = 0; r < n; ++r)
        {
            const Str& s   = line(p.line);
            const Int32 len = static_cast<Int32>(s.size());

            if(p.col >= len)
            {
                if(p.line + 1 >= lineCount()) break;
                ++p.line; p.col = 0;
                continue;
            }
            const Bool startWord = wordChar(s[static_cast<Size>(p.col)]);
            while(p.col < len && wordChar(s[static_cast<Size>(p.col)]) == startWord
                  && !space(s[static_cast<Size>(p.col)]))
                ++p.col;
            while(p.col < len && space(s[static_cast<Size>(p.col)]))
                ++p.col;
            if(p.col >= len && p.line + 1 < lineCount())
            {
                ++p.line; p.col = 0;
            }
        }
        break;
    }

    case 'b':
    {
        for(Int32 r = 0; r < n; ++r)
        {
            if(p.col == 0)
            {
                if(p.line == 0) break;
                --p.line;
                p.col = static_cast<Int32>(line(p.line).size());
            }
            const Str& s = line(p.line);
            while(p.col > 0 && space(s[static_cast<Size>(p.col - 1)]))
                --p.col;
            if(p.col > 0)
            {
                const Bool w = wordChar(s[static_cast<Size>(p.col - 1)]);
                while(p.col > 0 && wordChar(s[static_cast<Size>(p.col - 1)]) == w
                      && !space(s[static_cast<Size>(p.col - 1)]))
                    --p.col;
            }
        }
        break;
    }

    case 'e':
    {
        for(Int32 r = 0; r < n; ++r)
        {
            const Str& s   = line(p.line);
            const Int32 len = static_cast<Int32>(s.size());
            ++p.col;
            while(p.col < len && space(s[static_cast<Size>(p.col)]))
                ++p.col;
            if(p.col < len)
            {
                const Bool w = wordChar(s[static_cast<Size>(p.col)]);
                while(p.col + 1 < len
                      && wordChar(s[static_cast<Size>(p.col + 1)]) == w
                      && !space(s[static_cast<Size>(p.col + 1)]))
                    ++p.col;
            }
            p.col = std::min(p.col, std::max(0, len - 1));
        }
        break;
    }

    default:
        return false;
    }

    // Clamp the column against the line the motion landed on.
    p.line = std::max(0, std::min(p.line, lineCount() - 1));
    const Int32 lim = static_cast<Int32>(line(p.line).size());
    p.col = std::max(0, std::min(p.col, lim));

    out = p;
    return true;
}

// ------------------------------------------------------------------- operators

Void Editor::yankRange(Cursor a, Cursor b, Bool linewise)
{
    if(before(b, a))
        std::swap(a, b);

    yankLinewise = linewise;
    yankBuf.clear();

    if(linewise)
    {
        for(Int32 l = a.line; l <= b.line && l < lineCount(); ++l)
        {
            yankBuf += line(l);
            yankBuf.push_back('\n');
        }
        const Int32 n = b.line - a.line + 1;
        if(n > 1)
        {
            Char buf[64];
            std::snprintf(buf, sizeof(buf), "%d lines yanked", n);
            message = buf;
        }
        return;
    }

    if(a.line == b.line)
    {
        const Str& s = line(a.line);
        const Int32 from = std::min(a.col, static_cast<Int32>(s.size()));
        const Int32 to   = std::min(b.col, static_cast<Int32>(s.size()));
        yankBuf = s.substr(static_cast<Size>(from), static_cast<Size>(to - from));
        return;
    }

    yankBuf = line(a.line).substr(static_cast<Size>(std::min(a.col,
                  static_cast<Int32>(line(a.line).size()))));
    yankBuf.push_back('\n');
    for(Int32 l = a.line + 1; l < b.line; ++l)
    {
        yankBuf += line(l);
        yankBuf.push_back('\n');
    }
    yankBuf += line(b.line).substr(0, static_cast<Size>(std::min(b.col,
                   static_cast<Int32>(line(b.line).size()))));
}

Void Editor::deleteRange(Cursor a, Cursor b, Bool linewise, Bool yank)
{
    if(before(b, a))
        std::swap(a, b);

    if(yank)
        yankRange(a, b, linewise);

    pushUndo();

    if(linewise)
    {
        const Int32 from = std::max(0, a.line);
        const Int32 to   = std::min(lineCount() - 1, b.line);
        lines.erase(lines.begin() + from, lines.begin() + to + 1);
        if(lines.empty())
            lines.push_back(Str());
        cur.line = std::min(from, lineCount() - 1);
        cur.col  = indentOf(cur.line);
        clampCursor();
        return;
    }

    if(a.line == b.line)
    {
        Str&        s  = lines[static_cast<Size>(a.line)];
        const Int32 f  = std::min(a.col, static_cast<Int32>(s.size()));
        const Int32 t  = std::min(b.col, static_cast<Int32>(s.size()));
        s.erase(static_cast<Size>(f), static_cast<Size>(t - f));
        cur = Cursor{ a.line, f };
        clampCursor();
        return;
    }

    Str head = lines[static_cast<Size>(a.line)].substr(0,
                   static_cast<Size>(std::min(a.col,
                       static_cast<Int32>(lines[static_cast<Size>(a.line)].size()))));
    Str tail = lines[static_cast<Size>(b.line)].substr(
                   static_cast<Size>(std::min(b.col,
                       static_cast<Int32>(lines[static_cast<Size>(b.line)].size()))));

    lines.erase(lines.begin() + a.line, lines.begin() + b.line + 1);
    lines.insert(lines.begin() + a.line, head + tail);
    cur = Cursor{ a.line, static_cast<Int32>(head.size()) };
    clampCursor();
}

Void Editor::put(Bool beforeCursor)
{
    if(yankBuf.empty())
        return;

    pushUndo();

    if(yankLinewise)
    {
        // Split the register on newlines and splice whole lines in.
        std::vector<Str> add;
        Str acc;
        for(Size i = 0; i < yankBuf.size(); ++i)
        {
            if(yankBuf[i] == '\n') { add.push_back(acc); acc.clear(); continue; }
            acc.push_back(yankBuf[i]);
        }
        if(!acc.empty())
            add.push_back(acc);

        const Int32 at = beforeCursor ? cur.line : cur.line + 1;
        lines.insert(lines.begin() + at, add.begin(), add.end());
        cur.line = at;
        cur.col  = indentOf(cur.line);
        clampCursor();
        return;
    }

    Str&        s  = lines[static_cast<Size>(cur.line)];
    const Int32 at = beforeCursor ? cur.col
                                  : std::min(cur.col + 1, static_cast<Int32>(s.size()));
    s.insert(static_cast<Size>(at), yankBuf);
    cur.col = at + static_cast<Int32>(yankBuf.size()) - 1;
    clampCursor();
}

// ------------------------------------------------------------------ key routing

Bool Editor::key(const Key& k)
{
    const Cursor           beforeCur = cur;
    const Size             beforeN   = lines.size();
    const Str              beforeL   = lines[static_cast<Size>(
                                          std::min<Int32>(cur.line, lineCount() - 1))];

    switch(md)
    {
    case Mode::MODE_INSERT:       applyInsertKey(k);  break;
    case Mode::MODE_VISUAL:
    case Mode::MODE_VISUAL_LINE:  applyVisualKey(k);  break;
    case Mode::MODE_COMMAND:      applyCommandKey(k); break;
    case Mode::MODE_NORMAL:
    default:                      applyNormalKey(k);  break;
    }

    clampCursor();

    return cur.line != beforeCur.line || cur.col != beforeCur.col
        || lines.size() != beforeN
        || lines[static_cast<Size>(std::min<Int32>(cur.line, lineCount() - 1))] != beforeL;
}

Void Editor::applyInsertKey(const Key& k)
{
    if(k.sp == Special::SPECIAL_ESC)
    {
        // vim leaves the caret one to the left when you leave insert mode.
        //
        // The decrement has to happen BEFORE the mode change: setMode clamps to
        // the normal-mode limit (len-1), so switching first turns a caret at
        // len into len-1 and the decrement below then takes a second column
        // that the user never asked for.
        cur.col = std::max(0, cur.col - 1);
        setMode(Mode::MODE_NORMAL);
        return;
    }
    if(k.sp == Special::SPECIAL_ENTER)  { pushUndo(); newlineWithIndent(); return; }
    if(k.sp == Special::SPECIAL_BACKSPACE) { pushUndo(); backspace();      return; }
    if(k.sp == Special::SPECIAL_TAB)
    {
        pushUndo();
        Str& s = lines[static_cast<Size>(cur.line)];
        const Int32 add = INDENT - (cur.col % INDENT);
        s.insert(static_cast<Size>(cur.col), static_cast<Size>(add), ' ');
        cur.col += add;
        return;
    }
    if(k.sp == Special::SPECIAL_DELETE)
    {
        Str& s = lines[static_cast<Size>(cur.line)];
        if(cur.col < static_cast<Int32>(s.size()))
        {
            pushUndo();
            s.erase(static_cast<Size>(cur.col), 1);
        }
        return;
    }
    if(k.sp == Special::SPECIAL_LEFT)  { --cur.col; return; }
    if(k.sp == Special::SPECIAL_RIGHT) { ++cur.col; return; }
    if(k.sp == Special::SPECIAL_UP)    { --cur.line; return; }
    if(k.sp == Special::SPECIAL_DOWN)  { ++cur.line; return; }
    if(k.sp == Special::SPECIAL_HOME)  { cur.col = 0; return; }
    if(k.sp == Special::SPECIAL_END)
    {
        cur.col = static_cast<Int32>(line(cur.line).size());
        return;
    }

    if(k.ch >= 32 && k.ch < 127 && !k.ctrl)
    {
        pushUndo();
        insertChar(k.ch);
    }
}

Void Editor::applyNormalKey(const Key& k)
{
    if(k.sp == Special::SPECIAL_ESC)
    {
        pendCount = 0;
        pendOp    = 0;
        return;
    }

    // Arrows work in every mode. Modal editing is a preference, not a hostage
    // situation, and somebody reaching for an arrow key should get a cursor
    // move rather than nothing.
    if(k.sp == Special::SPECIAL_LEFT)  { --cur.col;  return; }
    if(k.sp == Special::SPECIAL_RIGHT) { ++cur.col;  return; }
    if(k.sp == Special::SPECIAL_UP)    { --cur.line; return; }
    if(k.sp == Special::SPECIAL_DOWN)  { ++cur.line; return; }
    if(k.sp == Special::SPECIAL_HOME)  { cur.col = 0; return; }
    if(k.sp == Special::SPECIAL_END)
    {
        cur.col = std::max(0, static_cast<Int32>(line(cur.line).size()) - 1);
        return;
    }

    const Char c = k.ch;
    if(c == 0)
        return;

    if(k.ctrl && (c == 'r' || c == 'R')) { redo(); return; }

    // ---- pending r<char>: replace the character under the caret ------------
    if(pendOp == 'r')
    {
        pendOp = 0;
        if(c >= 32 && c < 127)
        {
            Str& s = lines[static_cast<Size>(cur.line)];
            if(cur.col < static_cast<Int32>(s.size()))
            {
                pushUndo();
                s[static_cast<Size>(cur.col)] = c;
            }
        }
        return;
    }

    // ---- pending g: only gg is supported -----------------------------------
    if(pendOp == 'g')
    {
        pendOp = 0;
        if(c == 'g')
        {
            cur.line = (pendCount > 0) ? std::min(lineCount() - 1, pendCount - 1) : 0;
            cur.col  = indentOf(cur.line);
        }
        pendCount = 0;
        return;
    }

    if(pendOp == 'Z')
    {
        pendOp = 0;
        if(c == 'Z')
            submitted = "wq";
        return;
    }

    // ---- counts -------------------------------------------------------------
    // '0' is a motion when no count is being typed and a digit when one is.
    if(c >= '1' && c <= '9')
    {
        pendCount = pendCount * 10 + (c - '0');
        return;
    }
    if(c == '0' && pendCount > 0)
    {
        pendCount *= 10;
        return;
    }

    // ---- an operator is pending: this key must be its motion ----------------
    if(pendOp == 'd' || pendOp == 'c' || pendOp == 'y')
    {
        const Char op = pendOp;
        pendOp = 0;
        const Int32 count = pendCount;
        pendCount = 0;

        // Doubling the operator (dd, cc, yy) acts on whole lines.
        if(c == op)
        {
            const Int32 n    = std::max(1, count);
            Cursor      a    = Cursor{ cur.line, 0 };
            Cursor      b    = Cursor{ std::min(lineCount() - 1, cur.line + n - 1), 0 };

            if(op == 'y')      { yankRange(a, b, true); }
            else if(op == 'd') { deleteRange(a, b, true, true); }
            else               // cc: clear the lines but keep one, and insert
            {
                yankRange(a, b, true);
                pushUndo();
                const Int32 ind = indentOf(a.line);
                lines.erase(lines.begin() + a.line, lines.begin() + b.line + 1);
                lines.insert(lines.begin() + a.line, Str(static_cast<Size>(ind), ' '));
                cur = Cursor{ a.line, ind };
                setMode(Mode::MODE_INSERT);
            }
            return;
        }

        Cursor to;
        Bool   linewise = false;
        if(!motion(c, count, to, linewise))
            return;

        // cw behaves like ce - it does not eat the whitespace after the word.
        // This is vim's own special case and its absence is immediately felt.
        if(op == 'c' && c == 'w')
        {
            const Str& s = line(cur.line);
            Int32      e = cur.col;
            const Int32 len = static_cast<Int32>(s.size());
            while(e < len && !space(s[static_cast<Size>(e)]))
                ++e;
            to = Cursor{ cur.line, e };
            linewise = false;
        }

        // A forward motion used as an operator target is exclusive; k/j make it
        // a line operation.
        if(op == 'y')      yankRange(cur, to, linewise);
        else if(op == 'd') deleteRange(cur, to, linewise, true);
        else
        {
            deleteRange(cur, to, linewise, true);
            setMode(Mode::MODE_INSERT);
        }
        return;
    }

    const Int32 count = pendCount;

    // ---- motions ------------------------------------------------------------
    {
        Cursor to;
        Bool   linewise = false;
        if(motion(c, count, to, linewise))
        {
            cur       = to;
            pendCount = 0;
            return;
        }
    }

    // ---- everything else -----------------------------------------------------
    switch(c)
    {
    case 'i': setMode(Mode::MODE_INSERT); break;
    case 'a': setMode(Mode::MODE_INSERT); ++cur.col; break;
    case 'I': setMode(Mode::MODE_INSERT); cur.col = indentOf(cur.line); break;
    case 'A': setMode(Mode::MODE_INSERT);
              cur.col = static_cast<Int32>(line(cur.line).size()); break;

    case 'o':
    {
        pushUndo();
        const Int32 ind = indentOf(cur.line);
        const Str&  s   = line(cur.line);
        const Int32 extra = (!s.empty() && s[s.size() - 1] == '{') ? INDENT : 0;
        lines.insert(lines.begin() + cur.line + 1,
                     Str(static_cast<Size>(ind + extra), ' '));
        ++cur.line;
        cur.col = ind + extra;
        setMode(Mode::MODE_INSERT);
        break;
    }

    case 'O':
    {
        pushUndo();
        const Int32 ind = indentOf(cur.line);
        lines.insert(lines.begin() + cur.line, Str(static_cast<Size>(ind), ' '));
        cur.col = ind;
        setMode(Mode::MODE_INSERT);
        break;
    }

    case 'x':
    {
        Str& s = lines[static_cast<Size>(cur.line)];
        if(!s.empty() && cur.col < static_cast<Int32>(s.size()))
        {
            const Int32 n = std::min(std::max(1, count),
                                     static_cast<Int32>(s.size()) - cur.col);
            pushUndo();
            yankBuf = s.substr(static_cast<Size>(cur.col), static_cast<Size>(n));
            yankLinewise = false;
            s.erase(static_cast<Size>(cur.col), static_cast<Size>(n));
        }
        break;
    }

    case 'X':
        if(cur.col > 0)
        {
            pushUndo();
            lines[static_cast<Size>(cur.line)].erase(static_cast<Size>(cur.col - 1), 1);
            --cur.col;
        }
        break;

    case 'D':
    {
        Str& s = lines[static_cast<Size>(cur.line)];
        if(cur.col < static_cast<Int32>(s.size()))
        {
            pushUndo();
            yankBuf = s.substr(static_cast<Size>(cur.col));
            yankLinewise = false;
            s.erase(static_cast<Size>(cur.col));
        }
        break;
    }

    case 'C':
    {
        Str& s = lines[static_cast<Size>(cur.line)];
        pushUndo();
        if(cur.col < static_cast<Int32>(s.size()))
            s.erase(static_cast<Size>(cur.col));
        setMode(Mode::MODE_INSERT);
        break;
    }

    case 's':
    {
        Str& s = lines[static_cast<Size>(cur.line)];
        if(cur.col < static_cast<Int32>(s.size()))
        {
            pushUndo();
            s.erase(static_cast<Size>(cur.col), 1);
        }
        setMode(Mode::MODE_INSERT);
        break;
    }

    case 'J':
        if(cur.line + 1 < lineCount())
        {
            pushUndo();
            Str& s    = lines[static_cast<Size>(cur.line)];
            Str  next = lines[static_cast<Size>(cur.line + 1)];

            Size cut = 0;
            while(cut < next.size() && space(next[cut]))
                ++cut;
            next.erase(0, cut);

            cur.col = static_cast<Int32>(s.size());
            if(!s.empty() && !next.empty())
                s.push_back(' ');
            s += next;
            lines.erase(lines.begin() + cur.line + 1);
        }
        break;

    case 'p': put(false); break;
    case 'P': put(true);  break;
    case 'u': undo();     break;

    case 'v': visAnchor = cur; setMode(Mode::MODE_VISUAL);      break;
    case 'V': visAnchor = cur; setMode(Mode::MODE_VISUAL_LINE); break;

    case 'd': case 'c': case 'y': case 'r': case 'g': case 'Z':
        pendOp = c;
        return;   // keep the count for the motion

    case ':':
        setMode(Mode::MODE_COMMAND);
        cmdLine.clear();
        break;

    default:
        break;
    }

    pendCount = 0;
}

Void Editor::applyVisualKey(const Key& k)
{
    if(k.sp == Special::SPECIAL_ESC)
    {
        setMode(Mode::MODE_NORMAL);
        return;
    }

    const Char c = k.ch;
    if(c == 0)
    {
        if(k.sp == Special::SPECIAL_LEFT)  --cur.col;
        if(k.sp == Special::SPECIAL_RIGHT) ++cur.col;
        if(k.sp == Special::SPECIAL_UP)    --cur.line;
        if(k.sp == Special::SPECIAL_DOWN)  ++cur.line;
        return;
    }

    if(c >= '1' && c <= '9') { pendCount = pendCount * 10 + (c - '0'); return; }

    {
        Cursor to;
        Bool   linewise = false;
        if(motion(c, pendCount, to, linewise))
        {
            cur       = to;
            pendCount = 0;
            return;
        }
    }

    const Bool lineMode = (md == Mode::MODE_VISUAL_LINE);
    Cursor a, b;

    switch(c)
    {
    case 'd':
    case 'x':
        if(selection(a, b)) { setMode(Mode::MODE_NORMAL); deleteRange(a, b, lineMode, true); }
        break;

    case 'y':
        if(selection(a, b))
        {
            yankRange(a, b, lineMode);
            setMode(Mode::MODE_NORMAL);
            cur = a;
        }
        break;

    case 'c':
        if(selection(a, b))
        {
            setMode(Mode::MODE_NORMAL);
            deleteRange(a, b, lineMode, true);
            if(lineMode)
            {
                pushUndo();
                lines.insert(lines.begin() + cur.line, Str());
                cur.col = 0;
            }
            setMode(Mode::MODE_INSERT);
        }
        break;

    case 'v':
        setMode(md == Mode::MODE_VISUAL ? Mode::MODE_NORMAL : Mode::MODE_VISUAL);
        break;

    case 'V':
        setMode(md == Mode::MODE_VISUAL_LINE ? Mode::MODE_NORMAL : Mode::MODE_VISUAL_LINE);
        break;

    case ':':
        setMode(Mode::MODE_COMMAND);
        break;

    default:
        break;
    }

    pendCount = 0;
}

Void Editor::applyCommandKey(const Key& k)
{
    if(k.sp == Special::SPECIAL_ESC)
    {
        setMode(Mode::MODE_NORMAL);
        return;
    }
    if(k.sp == Special::SPECIAL_ENTER)
    {
        submitted = cmdLine;

        // :N jumps to a line. Handled here rather than by the caller because it
        // is the editor's own business - the caller only deals with :w and :q.
        if(!cmdLine.empty() && cmdLine.find_first_not_of("0123456789") == Str::npos)
        {
            const Int32 n = std::atoi(cmdLine.c_str());
            cur.line  = std::max(0, std::min(lineCount() - 1, n - 1));
            cur.col   = indentOf(cur.line);
            submitted.clear();
        }
        setMode(Mode::MODE_NORMAL);
        return;
    }
    if(k.sp == Special::SPECIAL_BACKSPACE)
    {
        if(cmdLine.empty())
            setMode(Mode::MODE_NORMAL);   // backspacing off the colon leaves
        else
            cmdLine.pop_back();
        return;
    }
    if(k.ch >= 32 && k.ch < 127)
        cmdLine.push_back(k.ch);
}

} // namespace ed
