#include "editor.hxx"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace ed
{
  namespace
  {

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

  }

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
          {
              continue;                    // CRLF in, LF held internally
          }
          if(c == '\n')
          {
              lines.push_back(acc); acc.clear(); continue;;
          }
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
          {
              out.push_back('\n');
          }
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
      {
          return empty;
      }
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
      {
          cmdLine.clear();
      }
      if(m == Mode::MODE_NORMAL)
      {
          pendCount   = 0;
          pendOp      = 0;
          pendFind    = 0;
          pendObjKind = 0;
          pendMark    = 0;
          pendOpCount = 0;
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
      {
          return false;
      }

      Cursor a = visAnchor;
      Cursor b = cur;
      if(before(b, a))
      {
          std::swap(a, b);
      }

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
      {
          lines.push_back(Str());
      }

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
      {
          ++i;
      }
      return i;
  }

  Void Editor::pushUndo()
  {
      ++changeSeq;

      Snapshot s;
      s.lines = lines;
      s.cur   = cur;
      undoStack.push_back(std::move(s));
      if(undoStack.size() > MAX_UNDO)
      {
          undoStack.erase(undoStack.begin());
      }

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
          {
              pair = false;
          }
      }
      else if(close != 0)
      {
          // Only pair a bracket when what follows is whitespace, a closer, or the
          // end of the line. Typing `(` before existing text usually means
          // wrapping it, and an injected `)` would land in the wrong place.
          if(next != '\0' && !space(next) && !isCloser(next) && next != ',')
          {
              pair = false;
          }
      }

      s.insert(static_cast<Size>(cur.col), 1, c);
      ++cur.col;

      if(pair)
      {
          s.insert(static_cast<Size>(cur.col), 1, close);
      }

      // A closing brace typed on a line that is otherwise blank re-indents to
      // match its opener's level. This is the one piece of "smart" indentation
      // here, and it is the one that people actually notice missing.
      if(c == '}')
      {
          Int32 firstNonSpace = 0;
          while(firstNonSpace < static_cast<Int32>(s.size())
                && space(s[static_cast<Size>(firstNonSpace)]))
          {
              ++firstNonSpace;
          }

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
                      if(t[static_cast<Size>(i)] == '}')
                      {
                          ++depth;
                      }
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
      {
          ++cut;
      }
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
          {
              ++firstNonSpace;
          }

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
      {
          return;
      }

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
      {
          return;
      }

      pushUndo();
      const Mode saved = md;
      md = Mode::MODE_INSERT;      // so clampCursor allows the past-the-end column

      for(Size i = 0; i < s.size(); ++i)
      {
          const Char c = s[i];
          if(c == '\r')
          {
              continue;
          }
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
      {
          --i;
      }

      // A run starting with a digit is a number being typed, not a name.
      if(i < n && s[static_cast<Size>(i)] >= '0' && s[static_cast<Size>(i)] <= '9')
      {
          return Str();
      }

      return s.substr(static_cast<Size>(i), static_cast<Size>(n - i));
  }

  Void Editor::replaceWordBeforeCursor(const Str& with)
  {
      const Str word = wordBeforeCursor();
      if(word.empty())
      {
          return;
      }

      pushUndo();

      Str&        s     = lines[static_cast<Size>(cur.line)];
      const Int32 start = cur.col - static_cast<Int32>(word.size());
      s.erase(static_cast<Size>(start), word.size());
      s.insert(static_cast<Size>(start), with);
      cur.col = start + static_cast<Int32>(with.size());
      clampCursor();
  }

  Void Editor::insertCompletion(const Str& with)
  {
      const Str word = wordBeforeCursor();
      if(!word.empty())
      {
          replaceWordBeforeCursor(with);
          return;
      }

      pushUndo();

      Str&        s   = lines[static_cast<Size>(cur.line)];
      const Int32 at  = std::min(cur.col, static_cast<Int32>(s.size()));
      s.insert(static_cast<Size>(at), with);
      cur.col = at + static_cast<Int32>(with.size());
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

      // WORD motions: whitespace-delimited, so they step over punctuation that w
      // and b stop at. In C this is the difference between crossing `foo->bar`
      // in one press and in five.
      case 'W':
      {
          const Str& sl  = line(p.line);
          const Int32 len = static_cast<Int32>(sl.size());
          for(Int32 i = 0; i < n; ++i)
          {
              while(p.col < len && !space(sl[static_cast<Size>(p.col)]))
              {
                  ++p.col;
              }
              while(p.col < len && space(sl[static_cast<Size>(p.col)]))
              {
                  ++p.col;
              }
          }
          break;
      }

      case 'B':
      {
          const Str& sl = line(p.line);
          for(Int32 i = 0; i < n; ++i)
          {
              while(p.col > 0 && space(sl[static_cast<Size>(p.col - 1)]))
              {
                  --p.col;
              }
              while(p.col > 0 && !space(sl[static_cast<Size>(p.col - 1)]))
              {
                  --p.col;
              }
          }
          break;
      }

      case 'E':
      {
          const Str& sl  = line(p.line);
          const Int32 len = static_cast<Int32>(sl.size());
          for(Int32 i = 0; i < n; ++i)
          {
              ++p.col;
              while(p.col < len && space(sl[static_cast<Size>(p.col)]))
              {
                  ++p.col;
              }
              while(p.col + 1 < len && !space(sl[static_cast<Size>(p.col + 1)]))
              {
                  ++p.col;
              }
          }
          break;
      }

      // Screen-relative, which is why the view has to be pushed in. Without a
      // viewport these would silently mean something else.
      case 'H':
          p.line   = std::max(0, std::min(lineCount() - 1, viewFirst + (n - 1)));
          p.col    = indentOf(p.line);
          linewise = true;
          break;

      case 'M':
          p.line   = std::max(0, std::min(lineCount() - 1,
                                          viewFirst + (viewSpan / 2)));
          p.col    = indentOf(p.line);
          linewise = true;
          break;

      case 'L':
          p.line   = std::max(0, std::min(lineCount() - 1,
                                          viewFirst + viewSpan - n));
          p.col    = indentOf(p.line);
          linewise = true;
          break;

      case '%':
      {
          Cursor m;
          if(!matchBracket(m))
          {
              return false;
          }
          p = m;
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
                  if(p.line + 1 >= lineCount())
                  {
                      break;
                  }
                  ++p.line; p.col = 0;
                  continue;
              }
              const Bool startWord = wordChar(s[static_cast<Size>(p.col)]);
              while(p.col < len && wordChar(s[static_cast<Size>(p.col)]) == startWord
                    && !space(s[static_cast<Size>(p.col)]))
              {
                  ++p.col;
              }
              while(p.col < len && space(s[static_cast<Size>(p.col)]))
              {
                  ++p.col;
              }
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
                  if(p.line == 0)
                  {
                      break;
                  }
                  --p.line;
                  p.col = static_cast<Int32>(line(p.line).size());
              }
              const Str& s = line(p.line);
              while(p.col > 0 && space(s[static_cast<Size>(p.col - 1)]))
              {
                  --p.col;
              }
              if(p.col > 0)
              {
                  const Bool w = wordChar(s[static_cast<Size>(p.col - 1)]);
                  while(p.col > 0 && wordChar(s[static_cast<Size>(p.col - 1)]) == w
                        && !space(s[static_cast<Size>(p.col - 1)]))
                  {
                      --p.col;
                  }
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
              {
                  ++p.col;
              }
              if(p.col < len)
              {
                  const Bool w = wordChar(s[static_cast<Size>(p.col)]);
                  while(p.col + 1 < len
                        && wordChar(s[static_cast<Size>(p.col + 1)]) == w
                        && !space(s[static_cast<Size>(p.col + 1)]))
                  {
                      ++p.col;
                  }
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

  Void Editor::setYank(const Str& text, Bool linewise)
  {
      yankBuf      = text;
      yankLinewise = linewise;
  }

  Void Editor::yankRange(Cursor a, Cursor b, Bool linewise)
  {
      if(before(b, a))
      {
          std::swap(a, b);
      }

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
              Array<Char, 64> buf;
              std::snprintf(buf.data(), buf.size(), "%d lines yanked", n);
              message = buf.data();
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
      {
          std::swap(a, b);
      }

      if(yank)
      {
          yankRange(a, b, linewise);
      }

      pushUndo();

      if(linewise)
      {
          const Int32 from = std::max(0, a.line);
          const Int32 to   = std::min(lineCount() - 1, b.line);
          lines.erase(lines.begin() + from, lines.begin() + to + 1);
          if(lines.empty())
          {
              lines.push_back(Str());
          }
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
      {
          return;
      }

      pushUndo();

      if(yankLinewise)
      {
          // Split the register on newlines and splice whole lines in.
          Vec<Str> add;
          Str acc;
          for(Size i = 0; i < yankBuf.size(); ++i)
          {
              if(yankBuf[i] == '\n')
              {
                  add.push_back(acc);
                  acc.clear(); continue;
              }
              acc.push_back(yankBuf[i]);
          }
          if(!acc.empty())
          {
              add.push_back(acc);
          }

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

      // ---- recording for `.` -------------------------------------------------
      //
      // A change is not one key. `ciw` plus its replacement text plus Escape is
      // five or fifty, and `.` has to repeat all of it - which is why the KEYS are
      // recorded rather than a parsed command. Replaying the keys reproduces the
      // insert for free; a parsed representation would have to model it.
      //
      // A fresh recording starts on any key pressed in normal mode with nothing
      // pending. Anything else - the motion after an operator, every character of
      // an insert - appends to the recording already running.
      const UInt64 seqBefore = changeSeq;
      Bool         dotKey    = false;
      if(!replaying)
      {
          const Bool idle = (md == Mode::MODE_NORMAL) && (pendOp == 0)
                         && (pendFind == 0) && (pendObjKind == 0)
                         && (pendMark == 0) && (pendCount == 0);

          // `.` is neither recorded nor committed. Recording it would nest; and
          // committing after it would overwrite lastChange with whatever keys
          // happened to be in the buffer - the `0` of a `j0` - so the SECOND `.`
          // would replay that instead and silently do nothing.
          dotKey = idle && (k.ch == '.');
          if(idle && !dotKey)
          {
              recBuf.clear();
              changeOpen = false;
          }
          if(!dotKey)
          {
              recBuf.push_back(k);
          }
      }

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

      if(!replaying && !dotKey)
      {
          if(changeSeq != seqBefore)
          {
              changeOpen = true;
          }

          // Committed only once the command is FINISHED - back in normal mode with
          // nothing pending. Committing per key would make `.` after `ciwfoo<Esc>`
          // repeat only the final keystroke.
          const Bool settled = (md == Mode::MODE_NORMAL) && (pendOp == 0)
                            && (pendFind == 0) && (pendObjKind == 0)
                            && (pendMark == 0);
          if(changeOpen && settled)
          {
              lastChange = recBuf;
              changeOpen = false;
          }
      }

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
      if(k.sp == Special::SPECIAL_ENTER)
      {
          pushUndo();
          newlineWithIndent();
          return;
      }
      if(k.sp == Special::SPECIAL_BACKSPACE)
      {
          pushUndo();
          backspace();
          return;
      }
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
      if(k.sp == Special::SPECIAL_LEFT)
      {
          --cur.col;
          return;
      }
      if(k.sp == Special::SPECIAL_RIGHT)
      {
          ++cur.col;
          return;
      }
      if(k.sp == Special::SPECIAL_UP)
      {
          --cur.line;
          return;
      }
      if(k.sp == Special::SPECIAL_DOWN)
      {
          ++cur.line;
          return;
      }
      if(k.sp == Special::SPECIAL_HOME)
      {
          cur.col = 0;
          return;
      }
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


  Void Editor::setViewport(Int32 firstLine, Int32 lineSpan)
  {
      viewFirst = std::max(0, firstLine);
      viewSpan  = std::max(1, lineSpan);
  }

  // ------------------------------------------------------------------- search

  // LITERAL, not a regular expression, and deliberately so. These are hundred-line
  // C sketches; `/gpio` is what gets typed, and a regex engine would mean std::regex
  // - which would silently reinterpret every . ( ) [ ] * + in the C under the caret
  // as syntax. Searching for `gpioWrite(LED` would then find nothing and give no
  // hint why. Literal is the behaviour that matches what people actually type here.
  //
  // Smartcase, as vim does it: an all-lowercase pattern ignores case, a pattern
  // with any capital in it is exact.
  namespace
  {

    Bool patternHasUpper(const Str& p)
    {
        for(Char c : p)
        {
            if(c >= 'A' && c <= 'Z')
            {
                return true;
            }
        }
        return false;
    }

    Char lowerOf(Char c)
    {
        return (c >= 'A' && c <= 'Z') ? static_cast<Char>(c - 'A' + 'a') : c;
    }

    // Index of `pat` in `hay` at or after `from`, or -1.
    Int32 findIn(const Str& hay, const Str& pat, Int32 from, Bool caseless)
    {
        const Int32 hn = static_cast<Int32>(hay.size());
        const Int32 pn = static_cast<Int32>(pat.size());
        if(pn == 0 || pn > hn)
        {
            return -1;
        }
        for(Int32 i = std::max(0, from); i + pn <= hn; ++i)
        {
            Int32 j = 0;
            while(j < pn)
            {
                const Char a = hay[static_cast<Size>(i + j)];
                const Char b = pat[static_cast<Size>(j)];
                if(caseless ? (lowerOf(a) != lowerOf(b)) : (a != b))
                {
                    break;
                }
                ++j;
            }
            if(j == pn)
            {
                return i;
            }
        }
        return -1;
    }

    // Last index of `pat` in `hay` strictly before `before`, or -1.
    Int32 rfindIn(const Str& hay, const Str& pat, Int32 before, Bool caseless)
    {
        Int32 best = -1;
        Int32 at   = 0;
        while(true)
        {
            const Int32 hit = findIn(hay, pat, at, caseless);
            if(hit < 0 || hit >= before)
            {
                break;
            }
            best = hit;
            at   = hit + 1;
        }
        return best;
    }

  }

  Bool Editor::searchFrom(const Str& pat, Bool forward, Cursor from, Cursor& out) const
  {
      if(pat.empty() || lineCount() == 0)
      {
          return false;
      }

      const Bool caseless = !patternHasUpper(pat);
      const Int32 n = lineCount();

      if(forward)
      {
          // Start just past the caret so `n` advances rather than finding the
          // match it is already sitting on.
          Int32 col = from.col + 1;
          for(Int32 step = 0; step <= n; ++step)
          {
              const Int32 l = (from.line + step) % n;
              const Int32 startAt = (step == 0) ? col : 0;
              const Int32 hit = findIn(line(l), pat, startAt, caseless);
              if(hit >= 0)
              {
                  out = Cursor{ l, hit };
                  return true;
              }
          }
          return false;
      }

      for(Int32 step = 0; step <= n; ++step)
      {
          const Int32 l = ((from.line - step) % n + n) % n;
          const Int32 limit = (step == 0) ? from.col
                                          : static_cast<Int32>(line(l).size());
          const Int32 hit = rfindIn(line(l), pat, limit, caseless);
          if(hit >= 0)
          {
              out = Cursor{ l, hit };
              return true;
          }
      }
      return false;
  }

  Void Editor::runSearch(const Str& pat, Bool forward, Int32 count)
  {
      if(pat.empty())
      {
          return;
      }

      Cursor at = cur;
      const Int32 n = std::max(1, count);
      for(Int32 i = 0; i < n; ++i)
      {
          Cursor hit;
          if(!searchFrom(pat, forward, at, hit))
          {
              message = "pattern not found: " + pat;
              return;
          }
          at = hit;
      }
      cur = at;
  }

  Str Editor::wordUnder(Cursor at) const
  {
      const Str& sl = line(at.line);
      const Int32 len = static_cast<Int32>(sl.size());
      if(at.col < 0 || at.col >= len)
      {
          return Str();
      }

      Int32 b = at.col;
      Int32 e = at.col;

      // Standing on punctuation, step forward to the next word rather than
      // returning nothing - which is what vim's * does and what feels right.
      if(!wordChar(sl[static_cast<Size>(b)]))
      {
          while(b < len && !wordChar(sl[static_cast<Size>(b)]))
          {
              ++b;
          }
          if(b >= len)
          {
              return Str();
          }
          e = b;
      }

      while(b > 0 && wordChar(sl[static_cast<Size>(b - 1)]))
      {
          --b;
      }
      while(e + 1 < len && wordChar(sl[static_cast<Size>(e + 1)]))
      {
          ++e;
      }
      return sl.substr(static_cast<Size>(b), static_cast<Size>(e - b + 1));
  }

  // -------------------------------------------------------------- f F t T

  Bool Editor::findInLine(Char cmd, Char target, Int32 count, Cursor& out) const
  {
      const Str&  sl  = line(cur.line);
      const Int32 len = static_cast<Int32>(sl.size());
      const Bool  fwd = (cmd == 'f' || cmd == 't');

      Int32 at = cur.col;
      const Int32 n = std::max(1, count);

      for(Int32 i = 0; i < n; ++i)
      {
          // `t` stops one short, so repeating it has to start one further along
          // or it would find the character it is already parked next to.
          Int32 probe = fwd ? (at + 1) : (at - 1);
          if((cmd == 't' && i > 0))
          {
              ++probe;
          }
          else if(cmd == 'T' && i > 0)
          {
              --probe;
          }

          Bool got = false;
          while(fwd ? (probe < len) : (probe >= 0))
          {
              if(sl[static_cast<Size>(probe)] == target)
              {
                  got = true;
                  break;
              }
              probe += fwd ? 1 : -1;
          }
          if(!got)
          {
              return false;
          }
          at = probe;
      }

      if(cmd == 't')
      {
          --at;
      }
      else if(cmd == 'T')
      {
          ++at;
      }

      out = Cursor{ cur.line, std::max(0, at) };
      return true;
  }

  // ------------------------------------------------------------------- %

  Bool Editor::matchBracket(Cursor& out) const
  {
      const Str&  sl  = line(cur.line);
      const Int32 len = static_cast<Int32>(sl.size());

      // vim scans FORWARD along the line for the first bracket, so % works from
      // anywhere on `    if(x)` rather than only from the parenthesis itself.
      Int32 at = cur.col;
      while(at < len)
      {
          const Char c = sl[static_cast<Size>(at)];
          if(c == '(' || c == ')' || c == '[' || c == ']' || c == '{' || c == '}')
          {
              break;
          }
          ++at;
      }
      if(at >= len)
      {
          return false;
      }

      const Char open = sl[static_cast<Size>(at)];
      Char       want = 0;
      Bool       fwd  = true;
      switch(open)
      {
      case '(': want = ')'; fwd = true;  break;
      case '[': want = ']'; fwd = true;  break;
      case '{': want = '}'; fwd = true;  break;
      case ')': want = '('; fwd = false; break;
      case ']': want = '['; fwd = false; break;
      case '}': want = '{'; fwd = false; break;
      default: return false;
      }

      Int32 depth = 0;
      Int32 l = cur.line;
      Int32 c = at;

      while(l >= 0 && l < lineCount())
      {
          const Str&  ls  = line(l);
          const Int32 ln  = static_cast<Int32>(ls.size());

          while(c >= 0 && c < ln)
          {
              const Char ch = ls[static_cast<Size>(c)];
              if(ch == open)
              {
                  ++depth;
              }
              else if(ch == want)
              {
                  --depth;
                  if(depth == 0)
                  {
                      out = Cursor{ l, c };
                      return true;
                  }
              }
              c += fwd ? 1 : -1;
          }

          l += fwd ? 1 : -1;
          if(l < 0 || l >= lineCount())
          {
              break;
          }
          c = fwd ? 0 : std::max(0, static_cast<Int32>(line(l).size()) - 1);
      }
      return false;
  }

  // -------------------------------------------------------- text objects

  Bool Editor::textObject(Char kind, Char obj, Cursor& a, Cursor& b, Bool& linewise) const
  {
      linewise = false;
      const Str&  sl  = line(cur.line);
      const Int32 len = static_cast<Int32>(sl.size());

      if(obj == 'w' || obj == 'W')
      {
          if(len == 0)
          {
              return false;
          }
          const Int32 at = std::min(cur.col, len - 1);

          // Word or run-of-whitespace, whichever the caret is standing in - so
          // diw in the gap between two words deletes the gap.
          const Bool onWord = (obj == 'w') ? wordChar(sl[static_cast<Size>(at)])
                                           : !space(sl[static_cast<Size>(at)]);
          const auto same = [&](Char c)
          {
              if(obj == 'W')
              {
                  return space(c) != onWord ? false : true;
              }
              return onWord ? wordChar(c) : !wordChar(c);
          };

          Int32 s0 = at;
          Int32 e0 = at;
          while(s0 > 0 && same(sl[static_cast<Size>(s0 - 1)]))
          {
              --s0;
          }
          while(e0 + 1 < len && same(sl[static_cast<Size>(e0 + 1)]))
          {
              ++e0;
          }

          // `aw` takes the trailing whitespace too, and only falls back to the
          // leading run when there is none after - which is vim's rule and the
          // reason daw at the end of a line does not leave a dangling space.
          if(kind == 'a')
          {
              Int32 e1 = e0;
              while(e1 + 1 < len && space(sl[static_cast<Size>(e1 + 1)]))
              {
                  ++e1;
              }
              if(e1 != e0)
              {
                  e0 = e1;
              }
              else
              {
                  while(s0 > 0 && space(sl[static_cast<Size>(s0 - 1)]))
                  {
                      --s0;
                  }
              }
          }

          a = Cursor{ cur.line, s0 };
          b = Cursor{ cur.line, e0 + 1 };
          return true;
      }

      // Brackets. Scans outward from the caret across lines, so ci{ works from
      // anywhere inside a function body.
      Char openCh  = 0;
      Char closeCh = 0;
      switch(obj)
      {
      case '(': case ')': case 'b': openCh = '('; closeCh = ')'; break;
      case '[': case ']':           openCh = '['; closeCh = ']'; break;
      case '{': case '}': case 'B': openCh = '{'; closeCh = '}'; break;
      case '<': case '>':           openCh = '<'; closeCh = '>'; break;
      default: break;
      }

      if(openCh != 0)
      {
          // Backwards for the unmatched opener.
          Int32 depth = 0;
          Int32 l = cur.line;
          Int32 c = cur.col;
          Cursor op{ -1, -1 };

          while(l >= 0)
          {
              const Str& ls = line(l);
              if(c < 0)
              {
                  c = static_cast<Int32>(ls.size()) - 1;
              }
              while(c >= 0)
              {
                  const Char ch = ls[static_cast<Size>(c)];
                  if(ch == closeCh && !(l == cur.line && c == cur.col))
                  {
                      ++depth;
                  }
                  else if(ch == openCh)
                  {
                      if(depth == 0)
                      {
                          op = Cursor{ l, c };
                          break;
                      }
                      --depth;
                  }
                  --c;
              }
              if(op.line >= 0)
              {
                  break;
              }
              --l;
              c = -1;
          }
          if(op.line < 0)
          {
              return false;
          }

          // Forwards for its partner.
          depth = 0;
          l = op.line;
          c = op.col;
          Cursor cl{ -1, -1 };
          while(l < lineCount())
          {
              const Str&  ls = line(l);
              const Int32 ln = static_cast<Int32>(ls.size());
              while(c < ln)
              {
                  const Char ch = ls[static_cast<Size>(c)];
                  if(ch == openCh)
                  {
                      ++depth;
                  }
                  else if(ch == closeCh)
                  {
                      --depth;
                      if(depth == 0)
                      {
                          cl = Cursor{ l, c };
                          break;
                      }
                  }
                  ++c;
              }
              if(cl.line >= 0)
              {
                  break;
              }
              ++l;
              c = 0;
          }
          if(cl.line < 0)
          {
              return false;
          }

          if(kind == 'a')
          {
              a = op;
              b = Cursor{ cl.line, cl.col + 1 };
          }
          else
          {
              a = Cursor{ op.line, op.col + 1 };
              b = cl;
          }
          return true;
      }

      // Quotes, which are their own closer - so they are paired by counting from
      // the start of the line rather than by nesting.
      if(obj == '"' || obj == '\'' || obj == '`')
      {
          Int32 openAt  = -1;
          Int32 closeAt = -1;
          Bool  inside  = false;
          Int32 last    = -1;

          for(Int32 i = 0; i < len; ++i)
          {
              if(sl[static_cast<Size>(i)] != obj)
              {
                  continue;
              }
              if(i > 0 && sl[static_cast<Size>(i - 1)] == '\\')
              {
                  continue;                      // an escaped quote is not one
              }
              if(!inside)
              {
                  inside = true;
                  last   = i;
              }
              else
              {
                  inside = false;
                  if(cur.col >= last && cur.col <= i)
                  {
                      openAt  = last;
                      closeAt = i;
                      break;
                  }
              }
          }
          if(openAt < 0)
          {
              return false;
          }

          if(kind == 'a')
          {
              a = Cursor{ cur.line, openAt };
              b = Cursor{ cur.line, closeAt + 1 };
          }
          else
          {
              a = Cursor{ cur.line, openAt + 1 };
              b = Cursor{ cur.line, closeAt };
          }
          return true;
      }

      return false;
  }

  // ------------------------------------------------------------- operators

  Void Editor::applyOperator(Char op, Cursor a, Cursor b, Bool linewise)
  {
      if(before(b, a))
      {
          std::swap(a, b);
      }

      switch(op)
      {
      case 'y':
          yankRange(a, b, linewise);
          cur = a;                    // vim leaves the caret at the start of a yank
          break;

      case 'd':
          deleteRange(a, b, linewise, true);
          break;

      case 'c':
          if(linewise)
          {
              // Same as cc: the lines go, one blank line at the original indent
              // stays, and insert starts on it. Deleting the line outright would
              // put the caret on the following line, which is not what c means.
              yankRange(a, b, true);
              pushUndo();
              const Int32 ind = indentOf(a.line);
              lines.erase(lines.begin() + a.line, lines.begin() + b.line + 1);
              lines.insert(lines.begin() + a.line, Str(static_cast<Size>(ind), ' '));
              cur = Cursor{ a.line, ind };
              setMode(Mode::MODE_INSERT);
          }
          else
          {
              deleteRange(a, b, linewise, true);
              setMode(Mode::MODE_INSERT);
          }
          break;

      case '>':
          indentLines(a.line, b.line, true);
          break;

      case '<':
          indentLines(a.line, b.line, false);
          break;

      default:
          break;
      }
  }

  Void Editor::indentLines(Int32 first, Int32 last, Bool rightwards)
  {
      first = std::max(0, first);
      last  = std::min(lineCount() - 1, last);
      if(first > last)
      {
          return;
      }

      pushUndo();
      for(Int32 l = first; l <= last; ++l)
      {
          Str& sl = lines[static_cast<Size>(l)];
          if(rightwards)
          {
              // A blank line stays blank. Indenting whitespace into nothing leaves
              // trailing spaces that no one asked for and every linter reports.
              if(!sl.empty())
              {
                  sl.insert(0, static_cast<Size>(INDENT), ' ');
              }
          }
          else
          {
              Int32 take = 0;
              while(take < INDENT && take < static_cast<Int32>(sl.size())
                    && sl[static_cast<Size>(take)] == ' ')
              {
                  ++take;
              }
              if(take > 0)
              {
                  sl.erase(0, static_cast<Size>(take));
              }
          }
      }
      cur.line = std::max(first, std::min(cur.line, last));
      cur.col  = indentOf(cur.line);
  }

  // ------------------------------------------------------------ :s/foo/bar/

  Void Editor::substitute(const Str& spec)
  {
      // spec is what came after the colon: "s/a/b/g" or "%s/a/b/gi". Any single
      // character may be the delimiter, as in vim, because a path is much easier
      // to substitute with :s#/usr#/opt# than with a line of backslashes.
      Size i = 0;
      Bool wholeFile = false;
      if(i < spec.size() && spec[i] == '%')
      {
          wholeFile = true;
          ++i;
      }
      if(i >= spec.size() || spec[i] != 's')
      {
          return;
      }
      ++i;
      if(i >= spec.size())
      {
          return;
      }

      const Char delim = spec[i];
      ++i;

      Array<Str, 3> parts;
      Int32 which = 0;
      for(; i < spec.size() && which < 3; ++i)
      {
          if(spec[i] == '\\' && i + 1 < spec.size() && spec[i + 1] == delim)
          {
              parts[which].push_back(delim);
              ++i;
              continue;
          }
          if(spec[i] == delim)
          {
              ++which;
              continue;
          }
          parts[which].push_back(spec[i]);
      }

      const Str& pat  = parts[0];
      const Str& repl = parts[1];
      const Str& flag = parts[2];
      if(pat.empty())
      {
          return;
      }

      const Bool all      = flag.find('g') != Str::npos;
      const Bool caseless = (flag.find('i') != Str::npos) || !patternHasUpper(pat);

      const Int32 from = wholeFile ? 0 : cur.line;
      const Int32 to   = wholeFile ? (lineCount() - 1) : cur.line;

      Int32 hits  = 0;
      Int32 touchedLines = 0;
      Bool  first = true;

      for(Int32 l = from; l <= to; ++l)
      {
          Str&  sl = lines[static_cast<Size>(l)];
          Int32 at = 0;
          Bool  touched = false;

          while(true)
          {
              const Int32 hit = findIn(sl, pat, at, caseless);
              if(hit < 0)
              {
                  break;
              }
              if(first)
              {
                  pushUndo();
                  first = false;
              }
              sl.erase(static_cast<Size>(hit), pat.size());
              sl.insert(static_cast<Size>(hit), repl);
              at = hit + static_cast<Int32>(repl.size());
              ++hits;
              touched = true;
              if(!all)
              {
                  break;
              }
          }
          if(touched)
          {
              ++touchedLines;
          }
      }

      Array<Char, 96> buf;
      if(hits == 0)
      {
          std::snprintf(buf.data(), buf.size(), "pattern not found: %s", pat.c_str());
      }
      else
      {
          std::snprintf(buf.data(), buf.size(), "%d substitution%s on %d line%s",
                        hits, hits == 1 ? "" : "s",
                        touchedLines, touchedLines == 1 ? "" : "s");
      }
      message = buf.data();
      clampCursor();
  }

  Void Editor::applyNormalKey(const Key& k)
  {
      if(k.sp == Special::SPECIAL_ESC)
      {
          pendCount   = 0;
          pendOp      = 0;
          pendFind    = 0;
          pendObjKind = 0;
          pendMark    = 0;
          pendOpCount = 0;
          return;
      }

      // Arrows work in every mode. Modal editing is a preference, not a hostage
      // situation, and somebody reaching for an arrow key should get a cursor
      // move rather than nothing.
      if(k.sp == Special::SPECIAL_LEFT)
      {
          --cur.col;
          return;
      }
      if(k.sp == Special::SPECIAL_RIGHT)
      {
          ++cur.col;
          return;
      }
      if(k.sp == Special::SPECIAL_UP)
      {
          --cur.line;
          return;
      }
      if(k.sp == Special::SPECIAL_DOWN)
      {
          ++cur.line;
          return;
      }
      if(k.sp == Special::SPECIAL_HOME)
      {
          cur.col = 0;
          return;
      }
      if(k.sp == Special::SPECIAL_END)
      {
          cur.col = std::max(0, static_cast<Int32>(line(cur.line).size()) - 1);
          return;
      }

      const Char c = k.ch;
      if(c == 0)
      {
          return;
      }

      if(k.ctrl && (c == 'r' || c == 'R'))
      {
          redo();
          return;
      }

      // Half a screen, which is why the viewport has to be pushed in. The caret
      // moves and the VIEW follows it, rather than the other way round - the code
      // that draws already scrolls to keep the caret visible.
      if(k.ctrl && (c == 'd' || c == 'D'))
      {
          cur.line = std::min(lineCount() - 1, cur.line + std::max(1, viewSpan / 2));
          cur.col  = indentOf(cur.line);
          return;
      }
      if(k.ctrl && (c == 'u' || c == 'U'))
      {
          cur.line = std::max(0, cur.line - std::max(1, viewSpan / 2));
          cur.col  = indentOf(cur.line);
          return;
      }

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

      // ---- pending g ---------------------------------------------------------
      if(pendOp == 'g')
      {
          pendOp = 0;
          if(c == 'g')
          {
              cur.line = (pendCount > 0) ? std::min(lineCount() - 1, pendCount - 1) : 0;
              cur.col  = indentOf(cur.line);
          }
          else if(c == 'e')
          {
              // Back to the end of the previous word - the motion you want when
              // the caret has overshot and `b` would land at the wrong end.
              const Str& sl = line(cur.line);
              Int32 i = cur.col;
              for(Int32 r = 0; r < std::max(1, pendCount); ++r)
              {
                  --i;
                  while(i > 0 && !wordChar(sl[static_cast<Size>(i)]))
                  {
                      --i;
                  }
                  while(i > 0 && wordChar(sl[static_cast<Size>(i - 1)])
                        && !wordChar(sl[static_cast<Size>(i)]))
                  {
                      --i;
                  }
              }
              cur.col = std::max(0, i);
          }
          pendCount = 0;
          return;
      }

      // ---- pending f/F/t/T: this key is the target character ------------------
      if(pendFind != 0)
      {
          const Char cmd = pendFind;
          pendFind = 0;

          const Char op    = pendOp;
          const Int32 total = std::max(1, pendCount) * std::max(1, pendOpCount);
          pendOp      = 0;
          pendCount   = 0;
          pendOpCount = 0;

          if(c < 32 || c >= 127)
          {
              return;
          }

          lastFindCmd  = cmd;
          lastFindChar = c;

          Cursor to;
          if(!findInLine(cmd, c, total, to))
          {
              return;                          // vim beeps; we simply do nothing
          }

          if(op != 0)
          {
              // f as an operator target is INCLUSIVE of the character it lands on,
              // unlike w. dfx has to take the x or it is not what anyone means.
              applyOperator(op, cur, Cursor{ to.line, to.col + 1 }, false);
          }
          else
          {
              cur = to;
          }
          return;
      }

      // ---- pending i/a: this key names the text object ------------------------
      if(pendObjKind != 0)
      {
          const Char kind = pendObjKind;
          pendObjKind = 0;

          const Char op = pendOp;
          pendOp      = 0;
          pendCount   = 0;
          pendOpCount = 0;

          Cursor a;
          Cursor b;
          Bool   lw = false;
          if(op != 0 && textObject(kind, c, a, b, lw))
          {
              applyOperator(op, a, b, lw);
          }
          return;
      }

      // ---- pending m / ' / ` : this key names the mark ------------------------
      if(pendMark != 0)
      {
          const Char what = pendMark;
          pendMark = 0;

          const Char op = pendOp;
          pendOp    = 0;
          pendCount = 0;

          if(c < 32 || c >= 127)
          {
              return;
          }

          if(what == 'm')
          {
              marks[c] = cur;
              return;
          }

          const auto it = marks.find(c);
          if(it == marks.end())
          {
              message = "mark not set";
              return;
          }

          // ' goes to the first non-blank of the line and is linewise; ` goes to
          // the exact column. Vim distinguishes them and so does this.
          Cursor to = it->second;
          to.line = std::max(0, std::min(lineCount() - 1, to.line));
          if(what == '\'')
          {
              to.col = indentOf(to.line);
          }

          if(op != 0)
          {
              applyOperator(op, cur, to, what == '\'');
          }
          else
          {
              cur = to;
          }
          return;
      }

      if(pendOp == 'Z')
      {
          pendOp = 0;
          if(c == 'Z')
          {
              submitted = "wq";
          }
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
      if(pendOp == 'd' || pendOp == 'c' || pendOp == 'y'
         || pendOp == '>' || pendOp == '<')
      {
          // A count typed BETWEEN the operator and the motion - the 3 in d3w.
          // Multiplied by any count before the operator, so 2d3w takes six.
          if(c >= '1' && c <= '9')
          {
              pendOpCount = pendOpCount * 10 + (c - '0');
              return;
          }
          if(c == '0' && pendOpCount > 0)
          {
              pendOpCount *= 10;
              return;
          }

          // These need one more key each, so the operator stays pending.
          if(c == 'i' || c == 'a')
          {
              pendObjKind = c;
              return;
          }
          if(c == 'f' || c == 'F' || c == 't' || c == 'T')
          {
              pendFind = c;
              return;
          }
          if(c == '\'' || c == '`')
          {
              pendMark = c;
              return;
          }

          const Char op = pendOp;
          pendOp = 0;

          // Zero has to survive. "No count" is not "a count of one": G with no
          // count is the LAST line and G with a count of one is line one, so
          // collapsing them turns yG into yy.
          const Int32 count = (pendCount == 0 && pendOpCount == 0)
                            ? 0
                            : (std::max(1, pendCount) * std::max(1, pendOpCount));
          pendCount   = 0;
          pendOpCount = 0;

          // Doubling the operator (dd, cc, yy, >>, <<) acts on whole lines.
          if(c == op)
          {
              const Int32 n    = std::max(1, count);
              Cursor      a    = Cursor{ cur.line, 0 };
              Cursor      b    = Cursor{ std::min(lineCount() - 1, cur.line + n - 1), 0 };

              if(op == '>' || op == '<')
              {
                  indentLines(a.line, b.line, op == '>');
                  return;
              }

              if(op == 'y')
              {
                  yankRange(a, b, true);
              }
              else if(op == 'd')
              {
                  deleteRange(a, b, true, true);
              }
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
          {
              return;
          }

          // cw behaves like ce - it does not eat the whitespace after the word.
          // This is vim's own special case and its absence is immediately felt.
          if(op == 'c' && c == 'w')
          {
              const Str& s = line(cur.line);
              Int32      e = cur.col;
              const Int32 len = static_cast<Int32>(s.size());
              while(e < len && !space(s[static_cast<Size>(e)]))
              {
                  ++e;
              }
              to = Cursor{ cur.line, e };
              linewise = false;
          }

          // A forward motion used as an operator target is exclusive; k/j make it
          // a line operation.
          applyOperator(op, cur, to, linewise);
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
          {
              s.erase(static_cast<Size>(cur.col));
          }
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
              {
                  ++cut;
              }
              next.erase(0, cut);

              cur.col = static_cast<Int32>(s.size());
              if(!s.empty() && !next.empty())
              {
                  s.push_back(' ');
              }
              s += next;
              lines.erase(lines.begin() + cur.line + 1);
          }
          break;

      case 'p': put(false); break;
      case 'P': put(true);  break;
      case 'u': undo();     break;

      case 'v': visAnchor = cur; setMode(Mode::MODE_VISUAL);      break;
      case 'V': visAnchor = cur; setMode(Mode::MODE_VISUAL_LINE); break;

      // ---- search ----------------------------------------------------------
      case '/':
      case '?':
          setMode(Mode::MODE_COMMAND);
          cmdPrefix = c;
          cmdLine.clear();
          break;

      case 'n':
          runSearch(lastSearch, searchForward, count);
          break;

      case 'N':
          runSearch(lastSearch, !searchForward, count);
          break;

      case '*':
      case '#':
      {
          // Search for the word the caret is on. The single most useful key in
          // vim for reading code you did not write, and the one whose absence is
          // felt within about a minute.
          const Str w = wordUnder(cur);
          if(w.empty())
          {
              break;
          }
          lastSearch    = w;
          searchForward = (c == '*');
          runSearch(lastSearch, searchForward, count);
          break;
      }

      // ---- repeat -----------------------------------------------------------
      case '.':
      {
          if(lastChange.empty() || replaying)
          {
              break;
          }

          // The recording is replayed through the front door, so every command
          // behaves exactly as it did the first time. `replaying` stops the
          // replay from recording itself over the thing being replayed.
          const Vec<Key> take = lastChange;
          replaying = true;
          for(const Key& rk : take)
          {
              key(rk);
          }
          replaying = false;
          break;
      }

      // ---- repeat the last f/F/t/T -------------------------------------------
      case ';':
      case ',':
      {
          if(lastFindCmd == 0)
          {
              break;
          }

          // `,` is the same search the other way round.
          Char cmd = lastFindCmd;
          if(c == ',')
          {
              switch(cmd)
              {
              case 'f': cmd = 'F'; break;
              case 'F': cmd = 'f'; break;
              case 't': cmd = 'T'; break;
              case 'T': cmd = 't'; break;
              default: break;
              }
          }

          Cursor to;
          if(findInLine(cmd, lastFindChar, count, to))
          {
              cur = to;
          }
          break;
      }

      // ---- case ---------------------------------------------------------------
      case '~':
      {
          Str& sl = lines[static_cast<Size>(cur.line)];
          const Int32 n = std::max(1, count);
          Bool touched = false;
          for(Int32 i = 0; i < n && cur.col < static_cast<Int32>(sl.size()); ++i)
          {
              Char& ch = sl[static_cast<Size>(cur.col)];
              if(ch >= 'a' && ch <= 'z')
              {
                  if(!touched)
                  {
                      pushUndo();
                      touched = true;
                  }
                  ch = static_cast<Char>(ch - 'a' + 'A');
              }
              else if(ch >= 'A' && ch <= 'Z')
              {
                  if(!touched)
                  {
                      pushUndo();
                      touched = true;
                  }
                  ch = static_cast<Char>(ch - 'A' + 'a');
              }
              ++cur.col;
          }
          break;
      }

      case 'm':
      case '\'':
      case '`':
          pendMark = c;
          return;

      case 'f': case 'F': case 't': case 'T':
          pendFind = c;
          return;   // keep the count for the target

      case 'd': case 'c': case 'y': case 'r': case 'g': case 'Z':
      case '>': case '<':
          pendOp = c;
          return;   // keep the count for the motion

      case ':':
          setMode(Mode::MODE_COMMAND);
          cmdPrefix = ':';
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
          if(k.sp == Special::SPECIAL_LEFT)
          {
              --cur.col;
          }
          if(k.sp == Special::SPECIAL_RIGHT)
          {
              ++cur.col;
          }
          if(k.sp == Special::SPECIAL_UP)
          {
              --cur.line;
          }
          if(k.sp == Special::SPECIAL_DOWN)
          {
              ++cur.line;
          }
          return;
      }

      if(c >= '1' && c <= '9')
      {
          pendCount = pendCount * 10 + (c - '0');
          return;
      }

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
          if(selection(a, b))
          {
              setMode(Mode::MODE_NORMAL);
              deleteRange(a, b, lineMode, true);
          }
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
          cmdPrefix = ':';
          return;
      }
      if(k.sp == Special::SPECIAL_ENTER)
      {
          // A search is the editor's own business and never reaches the caller -
          // only :w and :q do, and handing them a "/gpio" to puzzle over would be
          // a bug waiting for a careless strcmp.
          if(cmdPrefix == '/' || cmdPrefix == '?')
          {
              const Bool fwd = (cmdPrefix == '/');
              if(!cmdLine.empty())
              {
                  lastSearch    = cmdLine;
                  searchForward = fwd;
              }
              setMode(Mode::MODE_NORMAL);
              cmdPrefix = ':';
              runSearch(lastSearch, fwd, 1);
              return;
          }

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
          // :s and :%s likewise - substitution is buffer work, not file work.
          else if(!cmdLine.empty()
                  && (cmdLine[0] == 's' || (cmdLine[0] == '%' && cmdLine.size() > 1
                                            && cmdLine[1] == 's')))
          {
              substitute(cmdLine);
              submitted.clear();
          }
          setMode(Mode::MODE_NORMAL);
          return;
      }
      if(k.sp == Special::SPECIAL_BACKSPACE)
      {
          if(cmdLine.empty())
          {
              setMode(Mode::MODE_NORMAL);   // backspacing off the colon leaves
              cmdPrefix = ':';
          }
          else
          {
              cmdLine.pop_back();
          }
          return;
      }
      if(k.ch >= 32 && k.ch < 127)
      {
          cmdLine.push_back(k.ch);
      }
  }

}
