#include "code_view.hxx"

#include "complete.hxx"
#include "syntax.hxx"
#include "theme.hxx"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace ui
{
  namespace
  {

    // Scratch reused across lines and frames. The tokenizer clears it, so this only
    // exists to stop a per-line vector allocation sixty times a second.
    Vec<syn::Span> spans;

    // Whether a /* was open at the START of each line. Rebuilt whenever the buffer
    // changes, because a block comment opened at line 3 changes the colour of line
    // 400 and the renderer only ever sees the forty lines that are on screen.
    Vec<Bool> blockAt;

    Void rebuildBlockFlags(const ed::Editor& e)
    {
        blockAt.assign(static_cast<Size>(e.lineCount()) + 1, false);

        Bool                   open = false;
        Vec<syn::Span> tmp;
        for(Int32 i = 0; i < e.lineCount(); ++i)
        {
            blockAt[static_cast<Size>(i)] = open;
            syn::tokenize(e.line(i), open, tmp);
        }
        blockAt[static_cast<Size>(e.lineCount())] = open;
    }

    const Char* modeName(ed::Mode m) noexcept
    {
        switch(m)
        {
        case ed::Mode::MODE_INSERT:      return "INSERT";
        case ed::Mode::MODE_VISUAL:      return "VISUAL";
        case ed::Mode::MODE_VISUAL_LINE: return "V-LINE";
        case ed::Mode::MODE_COMMAND:     return "COMMAND";
        case ed::Mode::MODE_NORMAL:
        default:                         return "NORMAL";
        }
    }

    ImU32 modeColor(ed::Mode m) noexcept
    {
        switch(m)
        {
        case ed::Mode::MODE_INSERT:      return syn::gruv::GREEN;
        case ed::Mode::MODE_VISUAL:
        case ed::Mode::MODE_VISUAL_LINE: return syn::gruv::PURPLE;
        case ed::Mode::MODE_COMMAND:     return syn::gruv::YELLOW;
        case ed::Mode::MODE_NORMAL:
        default:                         return syn::gruv::BLUE;
        }
    }

    // ImGui key -> ed::Special, for the presses that carry no character.
    struct SpecialKey
    {
        ImGuiKey    key;
        ed::Special sp;
    };

    const SpecialKey SPECIALS[] = {
        { ImGuiKey_Escape,     ed::Special::SPECIAL_ESC       },
        { ImGuiKey_Enter,      ed::Special::SPECIAL_ENTER     },
        { ImGuiKey_KeypadEnter,ed::Special::SPECIAL_ENTER     },
        { ImGuiKey_Backspace,  ed::Special::SPECIAL_BACKSPACE },
        { ImGuiKey_Tab,        ed::Special::SPECIAL_TAB       },
        { ImGuiKey_Delete,     ed::Special::SPECIAL_DELETE    },
        { ImGuiKey_LeftArrow,  ed::Special::SPECIAL_LEFT      },
        { ImGuiKey_RightArrow, ed::Special::SPECIAL_RIGHT     },
        { ImGuiKey_UpArrow,    ed::Special::SPECIAL_UP        },
        { ImGuiKey_DownArrow,  ed::Special::SPECIAL_DOWN      },
        { ImGuiKey_Home,       ed::Special::SPECIAL_HOME      },
        { ImGuiKey_End,        ed::Special::SPECIAL_END       },
        { ImGuiKey_PageUp,     ed::Special::SPECIAL_PAGE_UP   },
        { ImGuiKey_PageDown,   ed::Special::SPECIAL_PAGE_DOWN },
    };

    // Below this many characters the popup would offer half the table and get in
    // the way. Two is enough to cut forty entries to a handful.
    constexpr Size MIN_PREFIX = 2;

    // How many rows are on screen at once. Past this the list scrolls rather
    // than growing: a popup taller than this covers the code the suggestion is
    // about, and clangd routinely offers two hundred names in a namespace.
    constexpr Int32 MAX_VISIBLE = 10;

    // How many are kept behind the scroll. Enough that scrolling is worth having
    // and bounded so a completion at file scope - where the answer is "every
    // name in the SDK" - does not turn into a thousand-row list nobody reads
    // past the tenth entry of anyway.
    constexpr Size MAX_KEPT = 200;

    // ---- diagnostics -----------------------------------------------------
    //
    // Three places drew this colour from the same ternary and a fourth was
    // about to. One function, so a note can never come out yellow in the
    // gutter and blue under the text.
    ImU32 severityColor(diag::Severity s) noexcept
    {
        switch(s)
        {
        case diag::Severity::SEVERITY_ERR:  return syn::gruv::RED;
        case diag::Severity::SEVERITY_WARN: return syn::gruv::YELLOW;
        case diag::Severity::SEVERITY_NOTE:
        default:                            return syn::gruv::BLUE;
        }
    }

    const Char* severityName(diag::Severity s) noexcept
    {
        switch(s)
        {
        case diag::Severity::SEVERITY_ERR:  return "error";
        case diag::Severity::SEVERITY_WARN: return "warning";
        case diag::Severity::SEVERITY_NOTE:
        default:                            return "note";
        }
    }

    // How wide the hover panel gets before a message wraps.
    //
    // Compiler messages are not short - a template error runs to hundreds of
    // characters and gcc's "In included file:" chains carry their own newlines.
    // A panel that grows to fit one of those covers the code it is about.
    constexpr Size HOVER_COLS = 76;

    // And how many overlapping diagnostics are shown before it just says how
    // many more there are. One position routinely collects an error plus the
    // three notes explaining it.
    constexpr Size HOVER_MAX = 4;

    // Breaks `text` at spaces so no line exceeds `cols` cells.
    //
    // Newlines already in the message are kept: gcc puts the file chain of an
    // include error on its own lines and running them together loses the one
    // piece of structure the message had.
    Void wrapTo(const Str& text, Size cols, Vec<Str>& out)
    {
        Str line;

        const auto flush = [&]()
        {
            out.push_back(line);
            line.clear();
        };

        for(Size i = 0; i <= text.size(); ++i)
        {
            const Bool end = (i == text.size());
            const Char c   = end ? '\n' : text[i];

            if(c == '\r')
            {
                continue;
            }
            if(c == '\n')
            {
                flush();
                continue;
            }

            line.push_back(c);
            if(line.size() < cols)
            {
                continue;
            }

            // Back up to the last space, so a word is not cut in half. A token
            // with no space in it - a mangled name, a long path - is hard-broken
            // instead, because the alternative is a panel wider than the pane.
            const Size sp = line.find_last_of(' ');
            if(sp == Str::npos || sp < cols / 3u)
            {
                flush();
                continue;
            }

            Str carry = line.substr(sp + 1);
            line.resize(sp);
            flush();
            line = carry;
        }

        while(!out.empty() && out.back().empty())
        {
            out.pop_back();
        }
    }

    ImU32 kindColor(cmpl::Kind k) noexcept
    {
        switch(k)
        {
        case cmpl::Kind::KIND_FUNCTION: return syn::gruv::GREEN;
        case cmpl::Kind::KIND_TYPE:     return syn::gruv::YELLOW;
        case cmpl::Kind::KIND_MACRO:    return syn::gruv::PURPLE;
        case cmpl::Kind::KIND_FIELD:    return syn::gruv::AQUA;
        case cmpl::Kind::KIND_VARIABLE: return syn::gruv::BLUE;
        case cmpl::Kind::KIND_KEYWORD:
        default:                        return syn::gruv::RED;
        }
    }

    const Char* kindTag(cmpl::Kind k) noexcept
    {
        switch(k)
        {
        case cmpl::Kind::KIND_FUNCTION: return "fn";
        case cmpl::Kind::KIND_TYPE:     return "ty";
        case cmpl::Kind::KIND_MACRO:    return "##";
        case cmpl::Kind::KIND_FIELD:    return ".f";
        case cmpl::Kind::KIND_VARIABLE: return "va";
        case cmpl::Kind::KIND_KEYWORD:
        default:                        return "kw";
        }
    }

    // ---- one popup, two sources ------------------------------------------
    //
    // Rows point at strings owned elsewhere - the table's are static, clangd's
    // live in v.lspAnswer - and both outlive the frame. Copying them into the
    // row would be tidier and would also mean allocating a few hundred strings
    // sixty times a second for a list that is thrown away each frame.
    struct Row
    {
        const Char* name    = nullptr;
        const Char* detail  = nullptr;
        const Char* doc     = nullptr;
        cmpl::Kind  kind    = cmpl::Kind::KIND_FUNCTION;
        Bool        fromLsp = false;
    };

    Bool startsWithFold(const Str& hay, const Str& needle) noexcept
    {
        if(needle.size() > hay.size())
        {
            return false;
        }
        for(Size i = 0; i < needle.size(); ++i)
        {
            const Char a = static_cast<Char>(std::tolower(static_cast<UInt8>(hay[i])));
            const Char b = static_cast<Char>(std::tolower(static_cast<UInt8>(needle[i])));
            if(a != b)
            {
                return false;
            }
        }
        return true;
    }

    // Whether the reply in hand is about the word being typed right now.
    //
    // clangd answers a question asked several frames ago. If the caret has since
    // left the line, or moved before the start of the word the answer was for,
    // the items are about a different place and showing them would be confident
    // and wrong - the failure mode this whole project keeps removing.
    Bool answerFits(const CodeView& v, const ed::Editor& e, Int32 startCol) noexcept
    {
        return v.lspAnswer.serial != 0
            && v.lspAnswer.line == e.cursor().line
            && v.lspAnswer.col  >= startCol
            && v.lspAnswer.col  <= e.cursor().col;
    }

    // Builds the list the popup shows: clangd's answer first, then anything the
    // hand-written table has that clangd did not mention.
    //
    // clangd goes first because it is answering about THIS translation unit -
    // it knows `pins::Map` has a field called `soundBusy` and the table never
    // will. The table goes second because it is the only source that survives
    // clangd being absent, still parsing, or dead, and because its one-line docs
    // are better than the ones a header actually carries.
    Void gather(const CodeView& v, const ed::Editor& e, Int32 startCol, Vec<Row>& out)
    {
        out.clear();

        // An empty prefix is only ever a member trigger. Everywhere else it
        // would mean "offer the entire table on a keystroke", which is why
        // MIN_PREFIX exists.
        if(v.popupPrefix.empty() && !v.popupTrigger)
        {
            return;
        }

        if(answerFits(v, e, startCol))
        {
            for(const lsp::Item& it : v.lspAnswer.items)
            {
                if(out.size() >= MAX_KEPT)
                {
                    break;
                }
                // clangd filtered for the prefix as it stood when we asked; the
                // user has usually typed more since. Filtering again here is
                // what keeps the list correct in the gap before the next reply.
                if(!startsWithFold(it.name, v.popupPrefix))
                {
                    continue;
                }

                Row r;
                r.name    = it.name.c_str();
                r.detail  = it.detail.c_str();
                r.doc     = it.doc.c_str();
                r.kind    = it.kind;
                r.fromLsp = true;
                out.push_back(r);
            }
        }

        // The table has nothing useful to say after a member operator: it does
        // not know what `gfx::` contains, and offering its forty names on the
        // strength of an empty prefix would bury whatever clangd found.
        if(v.popupPrefix.empty())
        {
            return;
        }

        Vec<const cmpl::Item*> table;
        cmpl::suggest(v.popupPrefix, table, MAX_KEPT);

        for(const cmpl::Item* it : table)
        {
            if(out.size() >= MAX_KEPT)
            {
                break;
            }

            Bool dup = false;
            for(Row& r : out)
            {
                if(std::strcmp(r.name, it->name) != 0)
                {
                    continue;
                }
                dup = true;

                // The same name from both sources. Keep clangd's row - its
                // detail is the real signature - but take the table's doc if
                // clangd had none, which is most of the time: a one-line summary
                // written for a reader beats an empty column.
                if((r.doc == nullptr || r.doc[0] == 0)
                   && it->doc != nullptr && it->doc[0] != 0)
                {
                    r.doc = it->doc;
                }
                break;
            }
            if(dup)
            {
                continue;
            }

            Row r;
            r.name   = it->name;
            r.detail = it->detail;
            r.doc    = it->doc;
            r.kind   = it->kind;
            out.push_back(r);
        }
    }

    // Whether the caret sits just after `::`, `.` or `->`.
    //
    // WHY THIS EXISTS. Without it the popup needs two characters of a name
    // before it says anything, and `dfplayer::` followed by a blank is exactly
    // the moment you do not know what to type. Two characters is the right rule
    // for a general prefix - a popup on every single letter is a popup in the
    // way - but after a member operator there is nothing to guess at: the set of
    // legal next words is small, known, and the whole reason a language server
    // is here.
    //
    // NOT INSIDE A COMMENT OR STRING, which is what the tokenizer is for. Prose
    // is full of full stops, and a completion list appearing after every
    // sentence in a doc comment would make this feature something to switch off.
    Bool afterMember(const ed::Editor& e, Bool blockOpen)
    {
        const Str&  line = e.line(e.cursor().line);
        const Int32 col  = std::min(e.cursor().col, static_cast<Int32>(line.size()));

        if(col < 1)
        {
            return false;
        }

        const Bool dot   = (line[static_cast<Size>(col - 1)] == '.');
        const Bool colon = (col >= 2 && line[static_cast<Size>(col - 1)] == ':'
                                     && line[static_cast<Size>(col - 2)] == ':');
        const Bool arrow = (col >= 2 && line[static_cast<Size>(col - 1)] == '>'
                                     && line[static_cast<Size>(col - 2)] == '-');
        if(!dot && !colon && !arrow)
        {
            return false;
        }

        // A number being typed - `1.` is not a member access.
        if(dot && col >= 2)
        {
            const Char before = line[static_cast<Size>(col - 2)];
            if(before >= '0' && before <= '9')
            {
                return false;
            }
        }

        Bool           open = blockOpen;
        Vec<syn::Span> tmp;
        syn::tokenize(line, open, tmp);

        for(const syn::Span& sp : tmp)
        {
            if(col - 1 < sp.begin || col - 1 >= sp.end)
            {
                continue;
            }
            return sp.role != syn::Role::ROLE_COMMENT
                && sp.role != syn::Role::ROLE_STRING
                && sp.role != syn::Role::ROLE_PREPROC;
        }
        return true;
    }

    // FNV-1a over the buffer, so the document is only re-sent when it changed.
    UInt64 hashOf(const Str& text) noexcept
    {
        UInt64 h = 1469598103934665603ull;
        for(const Char c : text)
        {
            h ^= static_cast<UInt64>(static_cast<UInt8>(c));
            h *= 1099511628211ull;
        }
        return h;
    }

  } // namespace

  Void setNote(CodeView& v, const Str& text, Float64 nowS)
  {
      v.note    = text;
      v.noteAtS = nowS;
  }

  Bool drawCode(CodeView& v, ed::Editor& e, const ImVec2& size, Float64 nowS)
  {
      ImGuiIO& io = ImGui::GetIO();

      // Collect clangd's reply once, at the top, so every use of it below sees
      // the same list. Taking it mid-frame would swap the strings out from under
      // the Row pointers built from them.
      static_cast<Void>(lsp::take(v.lspAnswer));

      ImFont* const font = (fonts.mono != nullptr) ? fonts.mono : fonts.body;

      // Multiplied by fontScale() so the editor takes part in the floating
      // workspace's optical zoom. Without it the panel grows and the text does
      // not, which reads as the editor reflowing rather than zooming - more
      // columns at the same size, when what was wanted was the same columns
      // bigger.
      const Float32 fontSz = ((font != nullptr && font->LegacySize > 0.0f)
                              ? font->LegacySize : ImGui::GetFontSize())
                           * fontScale();

      ImGui::PushFont(font, fontSz);
      ImGui::PushStyleColor(ImGuiCol_ChildBg, syn::gruv::BG0_H);
      ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

      ImGui::BeginChild("##code", size, ImGuiChildFlags_None,
                        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
                        | ImGuiWindowFlags_NoNavInputs);

      const ImVec2  origin   = ImGui::GetCursorScreenPos();
      const ImVec2  region   = ImGui::GetContentRegionAvail();
      ImDrawList*   dl       = ImGui::GetWindowDrawList();

      const Float32 lineH    = ImGui::GetTextLineHeight();
      // The character cell, rounded to a WHOLE pixel.
      //
      // Everything on screen is placed at `column * charW`, so this one number has
      // to be exact or nothing lines up. Left fractional it accumulates: at 12.43
      // px per cell, column 60 is three quarters of a character away from where
      // the grid says it is, and the caret ends up sitting between two glyphs.
      // Rounded to the nearest whole pixel the grid is closed, and a monospace
      // face at a whole-pixel advance is what a terminal has always done.
      Float32 charW = ImGui::CalcTextSize("0").x;
      charW = (charW > 1.0f) ? static_cast<Float32>(static_cast<Int32>(charW + 0.5f))
                             : 1.0f;
      const Float32 statusH  = lineH + 6.0f * dpiScale();

      // Gutter wide enough for the largest line number this buffer will ever show,
      // so it does not jump a pixel when the file crosses 100 lines.
      Array<Char, 16> numBuf;
      std::snprintf(numBuf.data(), numBuf.size(), "%d", std::max(1, e.lineCount()));
      // +3 rather than +2: one column for the diagnostic mark on the left, and
      // one of margin on each side of the number itself.
      const Float32 gutterW = (static_cast<Float32>(std::strlen(numBuf.data())) + 3.0f) * charW;

      const Float32 textX   = origin.x + gutterW;
      const Float32 viewH   = std::max(lineH, region.y - statusH);

      // ---- input ------------------------------------------------------------
      // An invisible button over the text area gives us a click target and a
      // hovered test without drawing anything. Focus is sticky: click in to take
      // the keyboard, click anywhere else to give it back.
      ImGui::SetCursorScreenPos(origin);
      ImGui::InvisibleButton("##codehit", ImVec2(std::max(1.0f, region.x),
                                                 std::max(1.0f, viewH)));
      const Bool hovered = ImGui::IsItemHovered();

      if(ImGui::IsItemClicked(ImGuiMouseButton_Left))
      {
          v.focused = true;

          // Put the caret where the click landed.
          const ImVec2 m = io.MousePos;
          const Int32  l = std::max(0, std::min(e.lineCount() - 1,
                               static_cast<Int32>((m.y - origin.y + v.scrollY) / lineH)));
          const Int32  c = std::max(0, static_cast<Int32>((m.x - textX) / charW + 0.5f));
          e.setCursor(l, c);
          v.followCaret = true;
          v.lastKeyS    = nowS;
      }
      else if(io.MouseClicked[0] && !hovered)
      {
          v.focused = false;
      }

      Bool changed = false;

      // Anything the editor wants to say - "3 lines yanked", "already at oldest
      // change". It produced these all along and nothing ever showed them.
      {
          const Str msg = e.takeMessage();
          if(!msg.empty())
          {
              setNote(v, msg, nowS);
          }
      }

      // ---- every yank leaves the app ---------------------------------------
      //
      // Done after the keys of the PREVIOUS frame have been applied, so it
      // catches a yank however it happened: yy, yG, y in visual mode, x, D, or
      // the delete half of any operator. The register is the single thing they
      // all write, so watching it costs one string compare and cannot miss one.
      //
      // Empty is not pushed. Clearing somebody's clipboard is never what they
      // wanted, and the register is briefly empty at the start of every yank.
      if(e.yankText() != v.lastYank)
      {
          v.lastYank = e.yankText();
          if(!v.lastYank.empty())
          {
              ImGui::SetClipboardText(v.lastYank.c_str());
          }
      }

      if(v.focused)
      {
          // Tell ImGui we own the keyboard this frame, so nothing else in the app
          // acts on the same presses. Without this, typing `f` in the editor could
          // also trip a global shortcut somewhere else.
          ImGui::SetNextFrameWantCaptureKeyboard(true);

          const Bool ctrl = io.KeyCtrl;

          // ---- the completion popup eats its keys FIRST ---------------------
          // Up/Down/Tab/Enter/Escape mean something different while a list is
          // open, and the editor must not also act on them. Anything not claimed
          // here falls through untouched.
          Bool eatEnter = false;
          Bool eatTab   = false;
          Bool eatEsc   = false;
          Bool eatPage  = false;

          if(v.popupOpen)
          {
              const Int32 startCol = e.cursor().col
                                   - static_cast<Int32>(v.popupPrefix.size());
              Vec<Row> hits;
              gather(v, e, startCol, hits);

              if(!hits.empty())
              {
                  const Int32 n = static_cast<Int32>(hits.size());

                  if(ImGui::IsKeyPressed(ImGuiKey_DownArrow, true)
                     || (ctrl && ImGui::IsKeyPressed(ImGuiKey_N, true)))
                      v.popupSel = (v.popupSel + 1) % n;

                  if(ImGui::IsKeyPressed(ImGuiKey_UpArrow, true)
                     || (ctrl && ImGui::IsKeyPressed(ImGuiKey_P, true)))
                      v.popupSel = (v.popupSel + n - 1) % n;

                  // A page at a time, for a list that is now long enough to have
                  // pages.
                  if(ImGui::IsKeyPressed(ImGuiKey_PageDown, true))
                  {
                      v.popupSel = std::min(n - 1, v.popupSel + MAX_VISIBLE);
                      eatPage    = true;
                  }
                  if(ImGui::IsKeyPressed(ImGuiKey_PageUp, true))
                  {
                      v.popupSel = std::max(0, v.popupSel - MAX_VISIBLE);
                      eatPage    = true;
                  }

                  v.popupSel = std::max(0, std::min(v.popupSel, n - 1));

                  const Bool accept = ImGui::IsKeyPressed(ImGuiKey_Tab, false)
                                   || ImGui::IsKeyPressed(ImGuiKey_Enter, false);
                  if(accept)
                  {
                      // insertCompletion, not replaceWordBeforeCursor: after a
                      // member operator there is no partial word to replace and
                      // the accepted name has to go in as it stands.
                      e.insertCompletion(hits[static_cast<Size>(v.popupSel)].name);
                      v.popupOpen = false;
                      v.dismissed = true;      // do not reopen on this word
                      changed     = true;
                      eatEnter    = true;
                      eatTab      = true;
                      v.lastKeyS  = nowS;
                  }

                  if(ImGui::IsKeyPressed(ImGuiKey_Escape, false))
                  {
                      v.popupOpen = false;
                      v.dismissed = true;
                      eatEsc      = true;
                  }
              }
          }

          // Specials first: they carry no character, and Tab/Enter would otherwise
          // also arrive as characters on some layouts.
          for(const SpecialKey& s : SPECIALS)
          {
              if(eatEnter && s.sp == ed::Special::SPECIAL_ENTER)
              {
                  continue;
              }
              if(eatTab   && s.sp == ed::Special::SPECIAL_TAB)
              {
                  continue;
              }
              if(eatEsc   && s.sp == ed::Special::SPECIAL_ESC)
              {
                  continue;
              }
              if(eatPage && (s.sp == ed::Special::SPECIAL_PAGE_UP
                             || s.sp == ed::Special::SPECIAL_PAGE_DOWN))
              {
                  continue;
              }

              if(ImGui::IsKeyPressed(s.key, true))
              {
                  ed::Key k;
                  k.sp    = s.sp;
                  k.ctrl  = ctrl;
                  k.shift = io.KeyShift;
                  changed |= e.key(k);
                  v.followCaret = true;
                  v.lastKeyS    = nowS;
              }
          }

          // Ctrl combinations, which produce no usable character.
          if(ctrl && ImGui::IsKeyPressed(ImGuiKey_R, true))
          {
              ed::Key k;
              k.ch   = 'r';
              k.ctrl = true;
              changed |= e.key(k);
              v.followCaret = true;
              v.lastKeyS    = nowS;
          }

          // Then printable characters, straight off the platform's queue so the
          // keyboard layout, shift and dead keys are all already resolved.
          for(Int32 i = 0; i < io.InputQueueCharacters.Size; ++i)
          {
              const ImWchar wc = io.InputQueueCharacters[i];
              if(wc < 32 || wc > 126)
                  continue;                       // control bytes and non-ASCII
              if(ctrl)
                  continue;                       // handled above

              // ---- p and P pull from the system clipboard ------------------
              //
              // Checked here, one keystroke before the put, rather than every
              // frame: opening the clipboard takes a global lock, and a program
              // that grabs it sixty times a second is a program that makes
              // copying flaky everywhere else on the desktop.
              //
              // Only in normal and visual mode. In insert mode `p` is the letter
              // p, and clobbering the register every time somebody types the word
              // "open" would be a genuinely baffling bug.
              //
              // Adopted only when the clipboard differs from what we last saw. A
              // yank inside the editor already pushed its text out, so the two
              // match and the register is left exactly as vim left it - including
              // whether it was linewise, which the newline guess below cannot
              // always recover.
              const Char ch = static_cast<Char>(wc);
              if((ch == 'p' || ch == 'P')
                 && (e.mode() == ed::Mode::MODE_NORMAL
                     || e.mode() == ed::Mode::MODE_VISUAL
                     || e.mode() == ed::Mode::MODE_VISUAL_LINE))
              {
                  const Char* clip = ImGui::GetClipboardText();
                  if(clip != nullptr && *clip != '\0' && v.lastYank != clip)
                  {
                      Str text = clip;

                      // CRLF in, LF held internally - the same normalisation
                      // sketch::load() does, and for the same reason: every line
                      // ending in this editor is one byte.
                      Str lf;
                      lf.reserve(text.size());
                      for(Size j = 0; j < text.size(); ++j)
                      {
                          if(text[j] != '\r')
                          {
                              lf.push_back(text[j]);
                          }
                      }

                      const Bool linewise = !lf.empty() && lf.back() == '\n';
                      e.setYank(lf, linewise);
                      v.lastYank = lf;
                  }
              }

              ed::Key k;
              k.ch = static_cast<Char>(wc);
              changed |= e.key(k);
              v.followCaret = true;
              v.lastKeyS    = nowS;
          }
          io.InputQueueCharacters.resize(0);

          // ---- reopen / close decision --------------------------------------
          // Recomputed from the buffer every frame rather than tracked as a state
          // machine: the word under the caret is the truth, and anything that
          // moves the caret - a click, a motion, an undo - then does the right
          // thing for free.
          const Str word = (e.mode() == ed::Mode::MODE_INSERT) ? e.wordBeforeCursor()
                                                               : Str();

          // blockAt is rebuilt further down, so on the first frame it is empty
          // and on a frame after an edit it is one behind. Neither matters here:
          // the flag only decides whether the caret is inside a block comment,
          // and being wrong for one frame costs a popup that appears or does not
          // and is corrected on the next.
          const Int32 cl        = e.cursor().line;
          const Bool  blockOpen = (cl >= 0 && cl < static_cast<Int32>(blockAt.size()))
                                ? blockAt[static_cast<Size>(cl)] : false;

          const Bool trigger = word.empty()
                            && e.mode() == ed::Mode::MODE_INSERT
                            && afterMember(e, blockOpen);

          if(word != v.popupPrefix || trigger != v.popupTrigger)
          {
              v.popupPrefix  = word;
              v.popupTrigger = trigger;
              v.popupSel     = 0;
              v.popupTop     = 0;
              v.dismissed    = false;   // a different word is a new question
          }

          const Int32 startCol = e.cursor().col - static_cast<Int32>(word.size());

          // What makes the popup worth opening at all: a long enough prefix, or
          // a member operator the caret is sitting right after.
          const Bool wanted = (word.size() >= MIN_PREFIX || trigger) && !v.dismissed;

          // ---- ask clangd ---------------------------------------------------
          //
          // Only when the caret has moved since the last question, and never
          // while one is outstanding. That pair is the whole rate limit: the
          // next question goes out when the previous answer lands, so the
          // request rate is the round-trip rate and cannot run ahead of it
          // however fast the user types.
          if(wanted
             && !v.lspPath.empty()
             && lsp::state() == lsp::State::STATE_READY
             && !lsp::busy()
             && (e.cursor().line != v.lspAskLine || e.cursor().col != v.lspAskCol))
          {
              const Str    text = e.text();
              const UInt64 h    = hashOf(text);

              // Only remember WHERE we asked if the question was taken. Refused
              // while clangd is still parsing, and recording the position anyway
              // would mean never asking again until the caret moved - so the
              // popup would sit on the hand-written table for the rest of the
              // session and look exactly like a working feature.
              if(lsp::ask(v.lspPath, text, h, e.cursor().line, e.cursor().col))
              {
                  v.lspAskLine = e.cursor().line;
                  v.lspAskCol  = e.cursor().col;
              }
          }

          if(wanted)
          {
              Vec<Row> hits;
              gather(v, e, startCol, hits);
              v.popupOpen = !hits.empty();
          }
          else
          {
              v.popupOpen = false;
          }
      }
      else
      {
          v.popupOpen    = false;
          v.popupTrigger = false;
      }

      if(hovered && io.MouseWheel != 0.0f)
      {
          v.scrollY -= io.MouseWheel * lineH * 3.0f;
          v.followCaret = false;
      }

      // ---- scrolling ---------------------------------------------------------
      const Float32 contentH = static_cast<Float32>(e.lineCount()) * lineH;
      const Float32 maxScroll = std::max(0.0f, contentH - viewH + lineH);

      if(v.followCaret)
      {
          const Float32 caretY = static_cast<Float32>(e.cursor().line) * lineH;
          if(caretY < v.scrollY)
              v.scrollY = caretY;
          else if(caretY + lineH > v.scrollY + viewH)
              v.scrollY = caretY + lineH - viewH;
          v.followCaret = false;
      }
      v.scrollY = std::max(0.0f, std::min(v.scrollY, maxScroll));

      // ---- background --------------------------------------------------------
      dl->AddRectFilled(origin, ImVec2(origin.x + region.x, origin.y + viewH),
                        syn::gruv::BG0_H);
      dl->AddRectFilled(origin, ImVec2(origin.x + gutterW, origin.y + viewH),
                        syn::gruv::BG0);

      dl->PushClipRect(origin, ImVec2(origin.x + region.x, origin.y + viewH), true);

      // The block-comment carry has to be right for the FIRST visible line, and
      // that depends on every line above it - so it is recomputed whenever the
      // line count or the buffer changes. Cheap: a sketch is tens of lines.
      if(blockAt.size() != static_cast<Size>(e.lineCount()) + 1 || changed)
          rebuildBlockFlags(e);

      const Int32 first = std::max(0, static_cast<Int32>(v.scrollY / lineH));
      const Int32 last  = std::min(e.lineCount() - 1,
                                   first + static_cast<Int32>(viewH / lineH) + 1);

      // H, M, L and Ctrl-D/Ctrl-U mean "the screen", not "the buffer", so the
      // editor is told where the screen is - from the same two numbers the drawing
      // below uses, which is what keeps them from ever disagreeing.
      e.setViewport(first, std::max(1, static_cast<Int32>(viewH / lineH)));

      ed::Cursor selA, selB;
      const Bool hasSel = e.selection(selA, selB);

      // ---- what the pointer is over ------------------------------------------
      //
      // Collected while the rows are drawn rather than recomputed afterwards,
      // because the row's y and the span's x are already in hand here and
      // deriving them twice is how the two end up disagreeing by a pixel.
      //
      // Only while the editor is hovered and nothing is being dragged: a
      // tooltip that appears mid-selection is a tooltip covering the thing you
      // are selecting.
      Vec<const diag::Item*> underPointer;
      ImVec2                 underAt(0.0f, 0.0f);

      const Bool wantHover = hovered
                          && !ImGui::IsMouseDown(ImGuiMouseButton_Left)
                          && !v.diags.empty();

      for(Int32 l = first; l <= last; ++l)
      {
          const Float32 y = origin.y + static_cast<Float32>(l) * lineH - v.scrollY;

          // Current-line band, behind everything else on the row.
          if(l == e.cursor().line && v.focused)
          {
              dl->AddRectFilled(ImVec2(origin.x + gutterW, y),
                                ImVec2(origin.x + region.x, y + lineH),
                                syn::gruv::BG1);
          }

          // ---- line number -------------------------------------------------
          //
          // RELATIVE in every mode except insert, and absolute on the caret's own
          // line. That is vim's `number` + `relativenumber`, and it is not a
          // stylistic choice: relative numbers exist so `12j` and `4dd` can be
          // read straight off the screen without counting, which is the entire
          // point of operator-plus-count editing.
          //
          // Insert mode switches to absolute because there are no counted motions
          // to serve there, and because a compiler error says "line 42", not
          // "line 42 relative to wherever your caret happens to be".
          const Bool onCaret  = (l == e.cursor().line);
          const Bool absolute = onCaret || (e.mode() == ed::Mode::MODE_INSERT);

          const Int32 shown = absolute ? (l + 1)
                                       : std::abs(l - e.cursor().line);
          std::snprintf(numBuf.data(), numBuf.size(), "%d", shown);
          const Float32 numW = static_cast<Float32>(std::strlen(numBuf.data())) * charW;
          // The caret's own number is left-aligned and bright; the relative ones
          // are right-aligned against it, so the column of distances reads as a
          // ruler rather than as a list of numbers.
          const Float32 numX = onCaret
              ? (origin.x + charW)
              : (origin.x + gutterW - charW - numW);

          dl->AddText(ImVec2(numX, y),
                      onCaret ? syn::gruv::YELLOW : syn::gruv::FG4,
                      numBuf.data());

          // ---- diagnostic mark in the gutter ---------------------------------
          if(!v.diags.empty())
          {
              const Int32 worst = diag::worstOnLine(v.diags, l + 1);
              if(worst >= 0)
              {
                  const ImU32 mc =
                      severityColor(static_cast<diag::Severity>(worst));

                  const Float32 r = std::max(2.0f, lineH * 0.16f);
                  dl->AddCircleFilled(ImVec2(origin.x + r * 1.6f, y + lineH * 0.5f),
                                      r, mc, 10);

                  // The whole gutter row is the target, not the four-pixel dot.
                  // The dot is a MARK - something to notice from across the
                  // screen - and asking anyone to land on it with a pointer
                  // would make it a worse one.
                  if(wantHover && underPointer.empty()
                     && io.MousePos.x >= origin.x && io.MousePos.x < textX
                     && io.MousePos.y >= y && io.MousePos.y < y + lineH)
                  {
                      for(const diag::Item& d : v.diags)
                      {
                          if(d.line == l + 1)
                          {
                              underPointer.push_back(&d);
                          }
                      }
                      underAt = ImVec2(origin.x, y);
                  }
              }
          }

          const Str&  src = e.line(l);
          const Int32 len = static_cast<Int32>(src.size());

          // Selection band for this row.
          if(hasSel && l >= selA.line && l <= selB.line)
          {
              const Int32 from = (l == selA.line) ? selA.col : 0;
              const Int32 to   = (l == selB.line) ? selB.col : len;
              if(to > from)
              {
                  dl->AddRectFilled(
                      ImVec2(textX + static_cast<Float32>(from) * charW, y),
                      ImVec2(textX + static_cast<Float32>(to) * charW, y + lineH),
                      syn::gruv::BG2);
              }
          }

          // The text itself, one draw per coloured run.
          Bool open = (l < static_cast<Int32>(blockAt.size())) ? blockAt[static_cast<Size>(l)]
                                                              : false;
          syn::tokenize(src, open, spans);

          // ---- diagnostic underline ------------------------------------------
          // A squiggle would be prettier and is not worth the vertex count at
          // 60 fps; a flat rule under the span says the same thing.
          if(!v.diags.empty())
          {
              for(const diag::Item& d : v.diags)
              {
                  if(d.line != l + 1)
                  {
                      continue;
                  }

                  // Column 0 means the compiler did not say where, so the whole
                  // line is marked rather than a span being invented for it.
                  const Int32 from = (d.column > 0) ? (d.column - 1) : 0;
                  const Int32 to   = (d.column > 0) ? std::min(len, from + 9) : len;
                  if(to <= from)
                  {
                      continue;
                  }

                  const ImU32   uc = severityColor(d.severity);
                  const Float32 x0 = textX + static_cast<Float32>(from) * charW;
                  const Float32 x1 = textX + static_cast<Float32>(to) * charW;

                  const Float32 uy = y + lineH - 1.5f * dpiScale();
                  dl->AddLine(ImVec2(x0, uy), ImVec2(x1, uy),
                              uc, std::max(1.0f, 1.5f * dpiScale()));

                  // The hit box is the TEXT, the full row height, not the rule
                  // under it. Nobody hovers a one-pixel line on purpose; they
                  // put the pointer on the word that is wrong.
                  if(wantHover && underPointer.empty()
                     && io.MousePos.x >= x0 && io.MousePos.x < x1
                     && io.MousePos.y >= y && io.MousePos.y < y + lineH)
                  {
                      // Everything overlapping that cell, not just this one: an
                      // error and the notes explaining it land on the same
                      // column, and showing one of them is showing the wrong
                      // half of the answer.
                      for(const diag::Item& o : v.diags)
                      {
                          if(o.line != l + 1)
                          {
                              continue;
                          }
                          const Int32 of = (o.column > 0) ? (o.column - 1) : 0;
                          const Int32 ot = (o.column > 0) ? std::min(len, of + 9) : len;

                          const Float32 ox0 = textX + static_cast<Float32>(of) * charW;
                          const Float32 ox1 = textX + static_cast<Float32>(ot) * charW;
                          if(io.MousePos.x >= ox0 && io.MousePos.x < ox1)
                          {
                              underPointer.push_back(&o);
                          }
                      }
                      underAt = ImVec2(x0, y);
                  }
              }
          }

          // ONE CELL AT A TIME, and this is the whole reason the caret used to
          // drift.
          //
          // Drawing a span with a single AddText lets ImGui advance by the FONT's
          // own glyph advance between characters, while the caret advances by
          // `charW`. The two differ by a fraction of a pixel, which is invisible
          // for three characters and half a cell by column forty - so the caret
          // slowly slid off the text and ended up between two glyphs. A long
          // comment is one span, which is exactly where it showed worst.
          //
          // Per-cell placement makes the grid the single source of truth: the
          // glyph and the caret are computed by the same expression, so they
          // cannot disagree. Whitespace is skipped, which in indented code is a
          // large fraction of the cells.
          for(const syn::Span& s : spans)
          {
              const ImU32 col = syn::colorFor(s.role);

              for(Int32 k = s.begin; k < s.end && k < len; ++k)
              {
                  const Char ch = src[static_cast<Size>(k)];
                  if(ch == ' ' || ch == '\t')
                  {
                      continue;
                  }

                  const Array<Char, 2> one= { ch, '\0' };
                  dl->AddText(ImVec2(textX + static_cast<Float32>(k) * charW, y),
                              col, one.data(), one.data() + 1);
              }
          }
      }

      // ---- caret -------------------------------------------------------------
      // Solid for 530 ms then off for 530 ms, the terminal convention, and forced
      // solid for a moment after every keystroke.
      if(v.focused)
      {
          const Float64 since = nowS - v.lastKeyS;
          const Bool    on    = (since < 0.25)
                             || (static_cast<Int32>((since - 0.25) / 0.53) % 2) == 0;
          if(on)
          {
              const Float32 cx = textX + static_cast<Float32>(e.cursor().col) * charW;
              const Float32 cy = origin.y + static_cast<Float32>(e.cursor().line) * lineH
                               - v.scrollY;

              // Block caret in normal mode, bar in insert - the same signal vim
              // gives, and the fastest way to see which mode you are in without
              // looking away from what you are typing.
              if(e.mode() == ed::Mode::MODE_INSERT)
              {
                  dl->AddRectFilled(ImVec2(cx, cy),
                                    ImVec2(cx + std::max(1.0f, 2.0f * dpiScale()), cy + lineH),
                                    syn::gruv::FG1);
              }
              else
              {
                  dl->AddRectFilled(ImVec2(cx, cy), ImVec2(cx + charW, cy + lineH),
                                    IM_COL32(0xEB, 0xDB, 0xB2, 0x80));
              }
          }
      }

      dl->PopClipRect();

      // ---- what the pointer is resting on -------------------------------------
      //
      // The compiler already said what is wrong with that line. Until now it
      // said it in the build pane, which means reading a message in one place
      // and finding the column it belongs to in another. The underline was
      // already there; this is the half that tells you what it means.
      //
      // Suppressed while the completion list is up. Two panels a few pixels
      // apart, one of which you are typing into, is worse than either alone -
      // and the one you are typing into wins.
      if(!underPointer.empty() && !v.popupOpen)
      {
          const Float32 pad  = 8.0f * dpiScale();
          const Float32 rowH = lineH;

          // Lay the whole thing out first, so the box is sized to what it will
          // actually hold rather than to a guess that has to be clamped after.
          struct HoverRow
          {
              Str   text;
              ImU32 col  = 0;
              Bool  head = false;
          };

          Vec<HoverRow> rows;
          const Size    show = std::min(underPointer.size(), HOVER_MAX);

          for(Size i = 0; i < show; ++i)
          {
              const diag::Item& d = *underPointer[i];

              if(i > 0)
              {
                  rows.push_back(HoverRow{ Str(), syn::gruv::BG3, false });
              }

              Array<Char, 128> head;
              if(d.column > 0)
              {
                  std::snprintf(head.data(), head.size(), "%s  %d:%d",
                                severityName(d.severity), d.line, d.column);
              }
              else
              {
                  std::snprintf(head.data(), head.size(), "%s  line %d",
                                severityName(d.severity), d.line);
              }
              rows.push_back(HoverRow{ Str(head.data()),
                                       severityColor(d.severity), true });

              Vec<Str> wrapped;
              wrapTo(d.message, HOVER_COLS, wrapped);
              for(const Str& w : wrapped)
              {
                  rows.push_back(HoverRow{ w, syn::gruv::FG1, false });
              }
          }

          if(underPointer.size() > show)
          {
              Array<Char, 64> more;
              std::snprintf(more.data(), more.size(), "+%d more on this line",
                            static_cast<Int32>(underPointer.size() - show));
              rows.push_back(HoverRow{ Str(more.data()), syn::gruv::GRAY, false });
          }

          Float32 wide = 0.0f;
          for(const HoverRow& r : rows)
          {
              if(!r.text.empty())
              {
                  wide = std::max(wide, ImGui::CalcTextSize(r.text.c_str()).x);
              }
          }

          const Float32 boxW = wide + pad * 2.0f;
          const Float32 boxH = rowH * static_cast<Float32>(rows.size()) + pad;

          // Under the underlined text, and above it when there is no room -
          // the same rule the completion list follows, so the two never appear
          // to come from different programs.
          Float32 hx = underAt.x;
          Float32 hy = underAt.y + lineH + 3.0f * dpiScale();
          if(hy + boxH > origin.y + viewH)
          {
              hy = underAt.y - boxH - 3.0f * dpiScale();
          }
          hx = std::max(origin.x, std::min(hx, origin.x + region.x - boxW));

          ImDrawList* fg = ImGui::GetForegroundDrawList();
          fg->AddRectFilled(ImVec2(hx, hy), ImVec2(hx + boxW, hy + boxH),
                            syn::gruv::BG0_H, 3.0f * dpiScale());
          fg->AddRect(ImVec2(hx, hy), ImVec2(hx + boxW, hy + boxH),
                      severityColor(underPointer[0]->severity), 3.0f * dpiScale());

          for(Size i = 0; i < rows.size(); ++i)
          {
              const HoverRow& r  = rows[i];
              const Float32   ry = hy + pad * 0.5f + static_cast<Float32>(i) * rowH;

              if(r.text.empty())
              {
                  // The blank row between two diagnostics is drawn as a rule.
                  // Whitespace alone reads as a gap in the panel rather than as
                  // a boundary between two separate things.
                  fg->AddLine(ImVec2(hx + pad, ry + rowH * 0.5f),
                              ImVec2(hx + boxW - pad, ry + rowH * 0.5f),
                              syn::gruv::BG3);
                  continue;
              }

              fg->AddText(ImVec2(hx + pad, ry), r.col, r.text.c_str());
          }
      }

      // ---- completion popup --------------------------------------------------
      // Drawn on the FOREGROUND draw list so it is not clipped by the editor
      // child, which is what lets it hang below the last visible line.
      if(v.popupOpen)
      {
          const Int32 startCol = e.cursor().col
                               - static_cast<Int32>(v.popupPrefix.size());
          Vec<Row> hits;
          gather(v, e, startCol, hits);

          if(hits.empty())
          {
              v.popupOpen = false;
          }
          else
          {
              const Int32 n   = static_cast<Int32>(hits.size());
              const Int32 sel = std::max(0, std::min(v.popupSel, n - 1));

              // ---- the visible window ----
              // Ten rows, and the selection drags it. Clamped after, so a list
              // that just filtered down from two hundred to three does not leave
              // the window parked past the end showing nothing.
              const Int32 shown = std::min(n, MAX_VISIBLE);

              if(sel < v.popupTop)
              {
                  v.popupTop = sel;
              }
              if(sel >= v.popupTop + shown)
              {
                  v.popupTop = sel - shown + 1;
              }
              v.popupTop = std::max(0, std::min(v.popupTop, n - shown));

              // Widest name and widest detail ACROSS THE WHOLE LIST, not just
              // the visible rows: sizing to the window would make the box change
              // width as it scrolls, which reads as the list jumping rather than
              // moving.
              Float32 nameW = 0.0f;
              Float32 detW  = 0.0f;
              for(const Row& it : hits)
              {
                  nameW = std::max(nameW, ImGui::CalcTextSize(it.name).x);
                  if(it.detail != nullptr && it.detail[0] != 0)
                      detW = std::max(detW, ImGui::CalcTextSize(it.detail).x);
              }

              // clangd's `detail` is a full C++ type and can be a hundred
              // characters. Cap the column so one std::vector<...> signature
              // cannot push the box off the side of the pane.
              detW = std::min(detW, charW * 44.0f);

              const Float32 pad   = 8.0f * dpiScale();
              const Float32 tagW  = ImGui::CalcTextSize("##").x + pad;
              const Float32 rowH  = lineH + 2.0f * dpiScale();
              const Float32 barW  = (n > shown) ? 4.0f * dpiScale() : 0.0f;
              const Float32 boxW  = tagW + nameW + pad * 2.0f + detW + pad * 2.0f + barW;
              const Float32 boxH  = rowH * static_cast<Float32>(shown) + pad;

              // Anchored to the START of the word, so the list lines up with what
              // it is completing rather than with the caret.
              Float32 px = textX + static_cast<Float32>(startCol) * charW;
              Float32 py = origin.y
                         + static_cast<Float32>(e.cursor().line + 1) * lineH - v.scrollY;

              // Flip above the caret rather than fall off the bottom.
              if(py + boxH > origin.y + viewH)
                  py = origin.y + static_cast<Float32>(e.cursor().line) * lineH
                     - v.scrollY - boxH;

              px = std::max(origin.x, std::min(px, origin.x + region.x - boxW));

              // NO WHEEL SCROLLING HERE, deliberately. The buffer's own wheel
              // handler runs three hundred lines earlier, before this box has a
              // position, so a wheel over the popup would move the list AND the
              // code underneath it. The list is driven by the keys that were
              // already driving it - arrows, ctrl-N/P, and now page up and down.
              ImDrawList* fg = ImGui::GetForegroundDrawList();
              fg->AddRectFilled(ImVec2(px, py), ImVec2(px + boxW, py + boxH),
                                syn::gruv::BG1, 3.0f * dpiScale());
              fg->AddRect(ImVec2(px, py), ImVec2(px + boxW, py + boxH),
                          syn::gruv::BG3, 3.0f * dpiScale());

              for(Int32 r = 0; r < shown; ++r)
              {
                  const Int32   idx = v.popupTop + r;
                  const Row&    it  = hits[static_cast<Size>(idx)];
                  const Float32 ry  = py + pad * 0.5f
                                    + static_cast<Float32>(r) * rowH;

                  if(idx == sel)
                  {
                      // Stops short of the scroll track, so the highlight does
                      // not paint over the one thing that says where in the list
                      // you are.
                      fg->AddRectFilled(ImVec2(px + 2.0f, ry),
                                        ImVec2(px + boxW - barW - 2.0f, ry + rowH),
                                        syn::gruv::BG2);
                  }

                  fg->AddText(ImVec2(px + pad * 0.5f, ry), syn::gruv::GRAY,
                              kindTag(it.kind));
                  fg->AddText(ImVec2(px + tagW, ry), kindColor(it.kind), it.name);

                  if(it.detail != nullptr && it.detail[0] != 0)
                  {
                      // Clipped, not wrapped: a row that wraps stops being a row.
                      const Float32 dx = px + tagW + nameW + pad * 2.0f;
                      fg->PushClipRect(ImVec2(dx, ry),
                                       ImVec2(dx + detW, ry + rowH), true);
                      fg->AddText(ImVec2(dx, ry), syn::gruv::FG4, it.detail);
                      fg->PopClipRect();
                  }
              }

              // ---- the scroll indicator ----
              // Position only, and no dragging. What it is for is answering "is
              // there more below" - a question the list cannot answer on its own
              // once it stops showing all of itself.
              if(n > shown)
              {
                  const Float32 trackX = px + boxW - barW - 2.0f * dpiScale();
                  const Float32 frac   = static_cast<Float32>(shown)
                                       / static_cast<Float32>(n);
                  const Float32 thumbH = std::max(rowH * 0.6f, boxH * frac);
                  const Float32 travel = boxH - thumbH;
                  const Float32 at     = (n == shown) ? 0.0f
                                       : static_cast<Float32>(v.popupTop)
                                         / static_cast<Float32>(n - shown);

                  fg->AddRectFilled(ImVec2(trackX, py),
                                    ImVec2(trackX + barW, py + boxH),
                                    syn::gruv::BG2);
                  fg->AddRectFilled(ImVec2(trackX, py + travel * at),
                                    ImVec2(trackX + barW, py + travel * at + thumbH),
                                    syn::gruv::GRAY);
              }

              // The selected entry's one-line doc, under the box. Only one, so
              // the popup stays a list rather than becoming a manual.
              //
              // The count rides along on the right, because "10 of 137" is the
              // difference between a list that is short and a list that is
              // showing you its first ten.
              const Row& cur = hits[static_cast<Size>(sel)];

              Array<Char, 96> tally;
              tally[0] = '\0';
              if(n > shown)
              {
                  std::snprintf(tally.data(), tally.size(), "%d of %d", sel + 1, n);
              }

              const Bool haveDoc = (cur.doc != nullptr && cur.doc[0] != 0);
              if(haveDoc || tally[0] != '\0')
              {
                  const Float32 dy   = py + boxH + 2.0f * dpiScale();
                  const Float32 docW = haveDoc ? ImGui::CalcTextSize(cur.doc).x : 0.0f;
                  const Float32 tw   = (tally[0] != '\0')
                                     ? ImGui::CalcTextSize(tally.data()).x + pad
                                     : 0.0f;
                  const Float32 dw   = std::min(docW + tw + pad * 2.0f, boxW * 1.5f);

                  fg->AddRectFilled(ImVec2(px, dy), ImVec2(px + dw, dy + rowH),
                                    syn::gruv::BG1, 3.0f * dpiScale());

                  if(haveDoc)
                  {
                      fg->PushClipRect(ImVec2(px + pad, dy),
                                       ImVec2(px + dw - tw - pad, dy + rowH), true);
                      fg->AddText(ImVec2(px + pad, dy + 1.0f * dpiScale()),
                                  syn::gruv::FG1, cur.doc);
                      fg->PopClipRect();
                  }
                  if(tally[0] != '\0')
                  {
                      fg->AddText(ImVec2(px + dw - tw, dy + 1.0f * dpiScale()),
                                  syn::gruv::GRAY, tally.data());
                  }
              }
          }
      }

      // ---- status line -------------------------------------------------------
      const Float32 sy = origin.y + viewH;
      dl->AddRectFilled(ImVec2(origin.x, sy), ImVec2(origin.x + region.x, sy + statusH),
                        syn::gruv::BG1);

      const Float32 pad = 6.0f * dpiScale();
      const Float32 ty  = sy + 3.0f * dpiScale();

      // The mode badge, in the mode's own colour. Reversed like vim's, so it is
      // legible at a glance rather than being one more word on a grey bar.
      {
          const Char*   mn = modeName(e.mode());
          const Float32 mw = ImGui::CalcTextSize(mn).x;
          dl->AddRectFilled(ImVec2(origin.x, sy),
                            ImVec2(origin.x + mw + pad * 2.0f, sy + statusH),
                            modeColor(e.mode()));
          dl->AddText(ImVec2(origin.x + pad, ty), syn::gruv::BG0_H, mn);

          Array<Char, 64> pos;
          std::snprintf(pos.data(), pos.size(), "%d:%d",
                        e.cursor().line + 1, e.cursor().col + 1);
          const Float32 pw = ImGui::CalcTextSize(pos.data()).x;
          dl->AddText(ImVec2(origin.x + region.x - pw - pad, ty), syn::gruv::FG4, pos.data());

          // ---- clangd, always ------------------------------------------------
          //
          // Present in every state rather than only when something is wrong. An
          // empty completion list is the same picture whether the server has
          // nothing to suggest, has not finished parsing, or is not running at
          // all, and those are three different situations with three different
          // things to do about them. A word on the status bar is the cheapest
          // possible way to never have to guess which one you are looking at.
          //
          // It also says when a question is IN FLIGHT, which is the state that
          // otherwise looks exactly like "no suggestions".
          Float32 rightOf = origin.x + region.x - pw - pad * 3.0f;
          {
              const Char* tag = "clangd";
              ImU32       col = syn::gruv::GRAY;

              switch(lsp::state())
              {
              case lsp::State::STATE_READY:
                  tag = lsp::busy() ? "clangd ..." : "clangd";
                  col = lsp::busy() ? syn::gruv::AQUA : syn::gruv::GREEN;
                  break;
              case lsp::State::STATE_STARTING:
                  tag = "clangd starting";
                  col = syn::gruv::YELLOW;
                  break;
              case lsp::State::STATE_FAILED:
                  tag = "clangd failed";
                  col = syn::gruv::RED;
                  break;
              case lsp::State::STATE_OFF:
              default:
                  tag = "clangd off";
                  col = syn::gruv::GRAY;
                  break;
              }

              const Float32 tw = ImGui::CalcTextSize(tag).x;
              dl->AddText(ImVec2(rightOf - tw, ty), col, tag);
              rightOf -= tw + pad * 2.0f;
          }

          // The middle slot, in priority order: what you are typing, then what
          // just happened, then a hint. A transient note outranks the hint
          // because it is news; the command line outranks everything because you
          // are looking straight at it.
          Str   mid;
          ImU32 midCol = syn::gruv::GRAY;

          if(e.mode() == ed::Mode::MODE_COMMAND)
          {
              mid    = Str(1, e.commandPrefix()) + e.commandLine();
              midCol = (e.commandPrefix() == ':') ? syn::gruv::FG1
                                                  : syn::gruv::AQUA;
          }
          else if(!v.note.empty())
          {
              const Float64 age = nowS - v.noteAtS;
              if(age < NOTE_HOLD_S + NOTE_FADE_S)
              {
                  mid = v.note;

                  // Fades out rather than vanishing. A message that disappears
                  // between glances is one you are never sure you saw.
                  Float32 a = 1.0f;
                  if(age > NOTE_HOLD_S)
                  {
                      a = 1.0f - static_cast<Float32>((age - NOTE_HOLD_S) / NOTE_FADE_S);
                  }
                  a = std::max(0.0f, std::min(1.0f, a));

                  const ImU32 base = syn::gruv::GREEN;
                  midCol = (base & 0x00FFFFFFu)
                         | (static_cast<ImU32>(a * 255.0f) << 24);
              }
              else
              {
                  v.note.clear();
              }
          }
          else if(lsp::state() == lsp::State::STATE_FAILED)
          {
              // OUTRANKS "click to edit" on purpose. The tag on the right says
              // THAT it failed; this says WHY, in the failure's own words -
              // "not installed" and "exited during startup" want different
              // things done about them and a red word cannot tell them apart.
              mid    = lsp::status();
              midCol = syn::gruv::RED;
          }
          else if(!v.focused)
          {
              mid = "click to edit";
          }

          if(!mid.empty())
          {
              dl->AddText(ImVec2(origin.x + mw + pad * 3.0f, ty), midCol, mid.c_str());
          }

          // ---- a diagnostic count, right of the position ---------------------
          if(!v.diags.empty())
          {
              Int32 errs = 0;
              Int32 warns = 0;
              for(const diag::Item& d : v.diags)
              {
                  if(d.severity == diag::Severity::SEVERITY_ERR)
                  {
                      ++errs;
                  }
                  else if(d.severity == diag::Severity::SEVERITY_WARN)
                  {
                      ++warns;
                  }
              }

              if(errs > 0 || warns > 0)
              {
                  Array<Char, 64> db;
                  std::snprintf(db.data(), db.size(), "%d e  %d w", errs, warns);
                  const Float32 dw = ImGui::CalcTextSize(db.data()).x;

                  // Left of the clangd tag, which claimed its own slot above.
                  dl->AddText(ImVec2(rightOf - dw, ty),
                              (errs > 0) ? syn::gruv::RED : syn::gruv::YELLOW, db.data());
              }
          }
      }

      ImGui::EndChild();
      ImGui::PopStyleVar(2);
      ImGui::PopStyleColor();
      ImGui::PopFont();

      return changed;
  }

} // namespace ui
