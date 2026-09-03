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

    // Scratch reused across lines and frames, to avoid a per-line allocation sixty
    // times a second. The tokenizer clears it.
    Vec<syn::Span> spans;

    // Whether a block comment was open at the START of each line. Rebuilt whenever
    // the buffer changes: one opened at line 3 colors line 400.
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

    constexpr SpecialKey SPECIALS[] = {
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
    // the way.
    constexpr Size MIN_PREFIX = 2;

    // Rows on screen at once; past this the list scrolls rather than growing over
    // the code. clangd routinely offers two hundred names in a namespace.
    constexpr Int32 MAX_VISIBLE = 10;

    // How many are kept behind the scroll, bounded so a completion at file scope
    // does not become a thousand-row list.
    constexpr Size MAX_KEPT = 200;

    // ---- diagnostics -----------------------------------------------------
    // One function, so a note can never come out yellow in the gutter and blue
    // under the text.
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

    // How wide the hover panel gets before a message wraps. A template error runs
    // to hundreds of characters, and a panel fitting one covers the code.
    constexpr Size HOVER_COLS = 76;

    // How many overlapping diagnostics are shown before it just says how many more
    // there are. One position routinely collects an error plus its notes.
    constexpr Size HOVER_MAX = 4;

    // Frames the pointer must rest on an identifier before clangd is asked.
    // At 60 fps this is about a fifth of a second - long enough that sweeping
    // the pointer across a line asks nothing, short enough that stopping to
    // read something does not feel like waiting for it.
    constexpr Int32 HOVER_REST_FRAMES = 12;

    // Breaks `text` at spaces so no line exceeds `cols` cells. Newlines already in
    // the message are KEPT: gcc puts an include error's file chain on its own lines.
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
            const Char c = end ? '\n' : text[i];

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

            // Back up to the last space so a word is not cut in half; a token with
            // no space in it is hard-broken instead.
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
    // Rows point at strings owned elsewhere - the table's are static, clangd's live
    // in v.lspAnswer - and both outlive the frame. Copying them would be a few
    // hundred allocations a frame for a list thrown away each frame.
    struct Row
    {
        const Char* name = nullptr;
        const Char* detail = nullptr;
        const Char* doc = nullptr;
        cmpl::Kind  kind = cmpl::Kind::KIND_FUNCTION;
        Bool        fromLsp = false;
    };

    // An identifier character, for the underline and the hover word alike.
    [[nodiscard]] Bool identChar(Char ch) noexcept
    {
        return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')
               || (ch >= '0' && ch <= '9') || ch == '_';
    }

    // Where the token starting at `from` ends, for a diagnostic underline. The
    // rule was a fixed nine characters from the reported column, which is too
    // long for `x` and too short for `driveThrottleUs` - and either way the
    // underline said nothing about what it was under. Now it covers the
    // identifier, and one cell for punctuation, which is what a compiler's
    // column actually points at.
    [[nodiscard]] Int32 tokenEnd(const Str& src, Int32 from, Int32 len) noexcept
    {
        Int32 t = from;
        while(t < len && identChar(src[static_cast<Size>(t)]))
        {
            ++t;
        }
        return std::max(t, std::min(len, from + 1));
    }

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

    // Whether the reply in hand is about the word being typed RIGHT NOW. clangd
    // answers a question asked several frames ago; if the caret has since left the
    // line or the word's start, showing the items is confidently wrong.
    Bool answerFits(const CodeView& v, const ed::Editor& e, Int32 startCol) noexcept
    {
        return v.lspAnswer.serial != 0
            && v.lspAnswer.line == e.cursor().line
            && v.lspAnswer.col  >= startCol
            && v.lspAnswer.col  <= e.cursor().col;
    }

    // Builds the list the popup shows: clangd's answer first, since it answers
    // about THIS translation unit; then whatever the hand-written table adds, since
    // the table is the only source that survives clangd being absent or dead.
    Void gather(const CodeView& v, const ed::Editor& e, Int32 startCol, Vec<Row>& out)
    {
        out.clear();

        // An empty prefix is only ever a member trigger; everywhere else it would
        // mean offering the entire table on one keystroke.
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
                // clangd filtered for the prefix as it stood when we asked; the user
                // has usually typed more since. Refiltering covers that gap.
                if(!startsWithFold(it.name, v.popupPrefix))
                {
                    continue;
                }

                Row r;
                r.name = it.name.c_str();
                r.detail = it.detail.c_str();
                r.doc = it.doc.c_str();
                r.kind = it.kind;
                r.fromLsp = true;
                out.push_back(r);
            }
        }

        // The table has nothing useful to say after a member operator, and its
        // forty names on an empty prefix would bury whatever clangd found.
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

                // The same name from both sources: keep clangd's row - its detail is
                // the real signature - but take the table's doc if clangd had none.
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
            r.name = it->name;
            r.detail = it->detail;
            r.doc = it->doc;
            r.kind = it->kind;
            out.push_back(r);
        }
    }

    // Whether the caret sits just after `::`, `.` or `->` - where MIN_PREFIX is the
    // wrong rule, because the set of legal next words is small and known and
    // `dfplayer::` followed by a blank is exactly when you do not know what to
    // type. NOT INSIDE A COMMENT OR STRING, which is what the tokenizer is for:
    // prose is full of full stops.
    Bool afterMember(const ed::Editor& e, Bool blockOpen)
    {
        const Str&  line = e.line(e.cursor().line);
        const Int32 col = std::min(e.cursor().col, static_cast<Int32>(line.size()));

        if(col < 1)
        {
            return false;
        }

        const Bool dot = (line[static_cast<Size>(col - 1)] == '.');
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

  }

  Void setNote(CodeView& v, const Str& text, Float64 nowS)
  {
      v.note = text;
      v.noteAtS = nowS;
  }

  Bool drawCode(CodeView& v, ed::Editor& e, const ImVec2& size, Float64 nowS)
  {
      ImGuiIO& io = ImGui::GetIO();

      // Collect clangd's reply ONCE, at the top: taking it mid-frame would swap the
      // strings out from under the Row pointers built from them.
      static_cast<Void>(lsp::take(v.lspAnswer));

      ImFont* const font = (fonts.mono != nullptr) ? fonts.mono : fonts.body;

      // Multiplied by fontScale() so the editor takes part in the workspace's
      // optical zoom - without it the panel grows and the text does not.
      const Float32 fontSz = ((font != nullptr && font->LegacySize > 0.0f)
                              ? font->LegacySize : ImGui::GetFontSize())
                           * fontScale();

      // The status line's insets, read BEFORE the two pushes below zero them:
      // the ends sit at WindowPadding.x, the slots are ItemSpacing.x apart.
      const Float32 statusInset = ImGui::GetStyle().WindowPadding.x;
      const Float32 statusGap = ImGui::GetStyle().ItemSpacing.x;

      ImGui::PushFont(font, fontSz);
      ImGui::PushStyleColor(ImGuiCol_ChildBg, syn::gruv::BG0_H);
      ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

      ImGui::BeginChild("##code", size, ImGuiChildFlags_None,
                        ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
                        | ImGuiWindowFlags_NoNavInputs);

      const ImVec2  origin = ImGui::GetCursorScreenPos();
      const ImVec2  region = ImGui::GetContentRegionAvail();
      ImDrawList*   dl = ImGui::GetWindowDrawList();

      const Float32 lineH = ImGui::GetTextLineHeight();
      // The character cell, rounded to a WHOLE pixel. Everything is placed at
      // `column * charW`, so a fractional value accumulates: at 12.43 px per cell,
      // column 60 is three quarters of a character off.
      Float32 charW = ImGui::CalcTextSize("0").x;
      charW = (charW > 1.0f) ? static_cast<Float32>(static_cast<Int32>(charW + 0.5f))
                             : 1.0f;
      // One frame tall - the mono face is pushed, so this is lineH plus the
      // style's FramePadding twice, not a 6 px of its own.
      const Float32 statusH = ImGui::GetFrameHeight();

      // Gutter wide enough for the largest line number this buffer will ever show,
      // so it does not jump a pixel when the file crosses 100 lines.
      Array<Char, 16> numBuf;
      std::snprintf(numBuf.data(), numBuf.size(), "%d", std::max(1, e.lineCount()));
      // +3: one column for the diagnostic mark, one of margin either side.
      const Float32 gutterW = (static_cast<Float32>(std::strlen(numBuf.data())) + 3.0f) * charW;

      const Float32 textX = origin.x + gutterW;
      const Float32 viewH = std::max(lineH, region.y - statusH);

      // ---- input ------------------------------------------------------------
      // An invisible button gives a click target and a hovered test. Focus is
      // sticky: click in to take the keyboard, elsewhere to give it back.
      ImGui::SetCursorScreenPos(origin);
      ImGui::InvisibleButton("##codehit", ImVec2(std::max(1.0f, region.x), std::max(1.0f, viewH)));
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
          v.lastKeyS = nowS;
      }
      else if(io.MouseClicked[0] && !hovered)
      {
          v.focused = false;
      }

      Bool changed = false;

      // Anything the editor wants to say - "3 lines yanked", "already at oldest
      // change".
      {
          const Str msg = e.takeMessage();
          if(!msg.empty())
          {
              setNote(v, msg, nowS);
          }
      }

      // ---- every yank leaves the app ---------------------------------------
      // After the PREVIOUS frame's keys are applied, so it catches a yank however
      // it happened - yy, yG, visual y, x, D, or any operator's delete half; the
      // register is the one thing they all write. Empty is NOT pushed: it is
      // briefly empty at the start of every yank.
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
          // We own the keyboard this frame, so typing `f` here cannot also trip a
          // global shortcut elsewhere.
          ImGui::SetNextFrameWantCaptureKeyboard(true);

          const Bool ctrl = io.KeyCtrl;

          // ---- the completion popup eats its keys FIRST ---------------------
          // Up/Down/Tab/Enter/Escape mean something different while a list is open
          // and the editor must not also act on them. Anything unclaimed falls
          // through untouched.
          Bool eatEnter = false;
          Bool eatTab = false;
          Bool eatEsc = false;
          Bool eatPage = false;

          // The outline is modal in the small sense: while it is up, every key
          // belongs to it and none reach the buffer - j and k move the list,
          // not the caret.
          Bool eatAll = false;

          if(v.outlineOpen)
          {
              eatAll = true;
              const Int32 n = static_cast<Int32>(v.outline.size());

              Bool down = ImGui::IsKeyPressed(ImGuiKey_DownArrow, true)
                       || (ctrl && ImGui::IsKeyPressed(ImGuiKey_N, true));
              Bool up = ImGui::IsKeyPressed(ImGuiKey_UpArrow, true)
                     || (ctrl && ImGui::IsKeyPressed(ImGuiKey_P, true));

              for(Int32 i = 0; i < io.InputQueueCharacters.Size; ++i)
              {
                  const ImWchar wc = io.InputQueueCharacters[i];
                  down |= (wc == 'j');
                  up |= (wc == 'k');
              }
              io.InputQueueCharacters.resize(0);

              if(n > 0)
              {
                  if(down)
                  {
                      v.outlineSel = (v.outlineSel + 1) % n;
                  }
                  if(up)
                  {
                      v.outlineSel = (v.outlineSel + n - 1) % n;
                  }
                  if(ImGui::IsKeyPressed(ImGuiKey_PageDown, true))
                  {
                      v.outlineSel = std::min(n - 1, v.outlineSel + MAX_VISIBLE);
                  }
                  if(ImGui::IsKeyPressed(ImGuiKey_PageUp, true))
                  {
                      v.outlineSel = std::max(0, v.outlineSel - MAX_VISIBLE);
                  }
                  v.outlineSel = std::max(0, std::min(v.outlineSel, n - 1));

                  if(ImGui::IsKeyPressed(ImGuiKey_Enter, false))
                  {
                      const lsp::Symbol& s = v.outline[static_cast<Size>(v.outlineSel)];
                      e.setCursor(s.line, 0);
                      v.followCaret = true;
                      v.outlineOpen = false;
                      v.lastKeyS = nowS;
                  }
              }

              if(ImGui::IsKeyPressed(ImGuiKey_Escape, false))
              {
                  v.outlineOpen = false;
              }
          }

          if(v.popupOpen && !eatAll)
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
                  {
                      v.popupSel = (v.popupSel + 1) % n;
                  }

                  if(ImGui::IsKeyPressed(ImGuiKey_UpArrow, true)
                     || (ctrl && ImGui::IsKeyPressed(ImGuiKey_P, true)))
                  {
                      v.popupSel = (v.popupSel + n - 1) % n;
                  }

                  // A page at a time, for a list long enough to have pages.
                  if(ImGui::IsKeyPressed(ImGuiKey_PageDown, true))
                  {
                      v.popupSel = std::min(n - 1, v.popupSel + MAX_VISIBLE);
                      eatPage = true;
                  }
                  if(ImGui::IsKeyPressed(ImGuiKey_PageUp, true))
                  {
                      v.popupSel = std::max(0, v.popupSel - MAX_VISIBLE);
                      eatPage = true;
                  }

                  v.popupSel = std::max(0, std::min(v.popupSel, n - 1));

                  const Bool accept = ImGui::IsKeyPressed(ImGuiKey_Tab, false)
                                   || ImGui::IsKeyPressed(ImGuiKey_Enter, false);
                  if(accept)
                  {
                      // insertCompletion, not replaceWordBeforeCursor: after a
                      // member operator there is no partial word to replace.
                      e.insertCompletion(hits[static_cast<Size>(v.popupSel)].name);
                      v.popupOpen = false;
                      v.dismissed = true;      // do not reopen on this word
                      changed = true;
                      eatEnter = true;
                      eatTab = true;
                      v.lastKeyS = nowS;
                  }

                  if(ImGui::IsKeyPressed(ImGuiKey_Escape, false))
                  {
                      v.popupOpen = false;
                      v.dismissed = true;
                      eatEsc = true;
                  }
              }
          }

          // Specials first: they carry no character, and Tab/Enter would otherwise
          // also arrive as characters on some layouts.
          for(const SpecialKey& s : SPECIALS)
          {
              if(eatAll)
              {
                  break;
              }
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
                  k.sp = s.sp;
                  k.ctrl = ctrl;
                  k.shift = io.KeyShift;
                  changed |= e.key(k);
                  v.followCaret = true;
                  v.lastKeyS = nowS;
              }
          }

          // Ctrl combinations, which produce no usable character.
          if(!eatAll && ctrl && ImGui::IsKeyPressed(ImGuiKey_R, true))
          {
              ed::Key k;
              k.ch = 'r';
              k.ctrl = true;
              changed |= e.key(k);
              v.followCaret = true;
              v.lastKeyS = nowS;
          }

          // Then printable characters, straight off the platform's queue, so layout,
          // shift and dead keys are already resolved.
          // Whether this frame's typing asks for signature help: `(` opens a
          // call and `,` moves to its next parameter; `)` closes it. Decided
          // here, where the character is known, and acted on after the loop.
          Bool sigAsk = false;
          Bool sigDrop = false;

          for(Int32 i = 0; i < io.InputQueueCharacters.Size; ++i)
          {
              if(eatAll)
              {
                  break;
              }
              const ImWchar wc = io.InputQueueCharacters[i];
              if(wc < 32 || wc > 126)
              {
                  continue;                       // control bytes and non-ASCII
              }
              if(ctrl)
              {
                  continue;                       // handled above
              }

              if(e.mode() == ed::Mode::MODE_INSERT)
              {
                  if(wc == '(' || wc == ',')
                  {
                      sigAsk = true;
                  }
                  else if(wc == ')')
                  {
                      sigDrop = true;
                  }
              }

              // ---- p and P pull from the system clipboard ------------------
              // Checked one keystroke before the put rather than every frame:
              // opening the clipboard takes a global lock, and grabbing it sixty
              // times a second makes copying flaky across the whole desktop.
              //
              // NORMAL AND VISUAL ONLY - in insert mode `p` is the letter p.
              // Adopted only when the clipboard DIFFERS from what we last saw, so a
              // yank made in the editor leaves the register in vim's own state,
              // including linewise, which the newline guess below cannot recover.
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

                      // CRLF in, LF held internally - the same normalization
                      // sketch::load() does: every line ending here is one byte.
                      Str lf;
                      lf.reserve(text.size());
                      for(const Char c : text)
                      {
                          if(c != '\r')
                          {
                              lf.push_back(c);
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
              v.lastKeyS = nowS;
          }
          io.InputQueueCharacters.resize(0);

          if(sigDrop)
          {
              // sigLine is invalidated too, not just the answer: takeSig()
              // accepts any reply for this line, and clangd answers late. A
              // `)` typed while the ask was still in flight would otherwise
              // let the reply land after the call was already closed.
              v.sigAnswer = lsp::Sig();
              v.sigIn = 0;
              v.sigLine = -1;
          }
          else if(sigAsk)
          {
              // Asked from where the caret now is - after the `(` or `,` went
              // in - which is the position clangd needs to know which
              // parameter is active.
              v.sigLine = e.cursor().line;
              v.sigCol = e.cursor().col;
              v.sigIn = 1;
              v.sigRetries = 0;
          }

          // ---- reopen / close decision --------------------------------------
          // Recomputed from the buffer every frame, not tracked as a state machine:
          // the word under the caret is the truth, so anything that moves the caret
          // does the right thing for free.
          const Str word = (e.mode() == ed::Mode::MODE_INSERT) ? e.wordBeforeCursor()
                                                               : Str();

          // blockAt is rebuilt further down, so it is empty on the first frame and
          // one behind after an edit. Harmless: it only decides whether the caret
          // is inside a block comment, and one wrong frame self-corrects.
          const Int32 cl = e.cursor().line;
          const Bool  blockOpen = (cl >= 0 && cl < static_cast<Int32>(blockAt.size()))
                                ? blockAt[static_cast<Size>(cl)] : false;

          const Bool trigger = word.empty()
                            && e.mode() == ed::Mode::MODE_INSERT
                            && afterMember(e, blockOpen);

          if(word != v.popupPrefix || trigger != v.popupTrigger)
          {
              v.popupPrefix = word;
              v.popupTrigger = trigger;
              v.popupSel = 0;
              v.popupTop = 0;
              v.dismissed = false;   // a different word is a new question
          }

          const Int32 startCol = e.cursor().col - static_cast<Int32>(word.size());

          // A long enough prefix, or a member operator the caret sits right after.
          const Bool wanted = (word.size() >= MIN_PREFIX || trigger) && !v.dismissed;

          // ---- ask clangd ---------------------------------------------------
          // Only when the caret has moved since the last question, and never while
          // one is outstanding. That pair is the whole rate limit: the request rate
          // becomes the round-trip rate, however fast the user types.
          if(wanted
             && !v.lspPath.empty()
             && lsp::state() == lsp::State::STATE_READY
             && !lsp::busy()
             && (e.cursor().line != v.lspAskLine || e.cursor().col != v.lspAskCol))
          {
              const Str    text = e.text();
              const UInt64 h = hashOf(text);

              // Only remember WHERE we asked if the question was TAKEN. It is
              // refused while clangd parses, and recording the position anyway
              // means never asking again until the caret moves - after which the
              // popup sits on the table and looks like a working feature.
              if(lsp::ask(v.lspPath, text, h, e.cursor().line, e.cursor().col))
              {
                  v.lspAskLine = e.cursor().line;
                  v.lspAskCol = e.cursor().col;
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
          v.popupOpen = false;
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
          {
              v.scrollY = caretY;
          }
          else if(caretY + lineH > v.scrollY + viewH)
          {
              v.scrollY = caretY + lineH - viewH;
          }
          v.followCaret = false;
      }
      v.scrollY = std::max(0.0f, std::min(v.scrollY, maxScroll));

      // ---- background --------------------------------------------------------
      dl->AddRectFilled(origin, ImVec2(origin.x + region.x, origin.y + viewH), syn::gruv::BG0_H);
      dl->AddRectFilled(origin, ImVec2(origin.x + gutterW, origin.y + viewH), syn::gruv::BG0);

      dl->PushClipRect(origin, ImVec2(origin.x + region.x, origin.y + viewH), true);

      // The block-comment carry must be right for the FIRST visible line, which
      // depends on every line above it - so it is rebuilt whenever the buffer does.
      if(blockAt.size() != static_cast<Size>(e.lineCount()) + 1 || changed)
      {
          rebuildBlockFlags(e);
      }

      const Int32 first = std::max(0, static_cast<Int32>(v.scrollY / lineH));
      const Int32 last = std::min(e.lineCount() - 1,
                                   first + static_cast<Int32>(viewH / lineH) + 1);

      // H, M, L and Ctrl-D/Ctrl-U mean "the screen", not "the buffer", so the editor
      // is told where it is - from the same two numbers the drawing below uses.
      e.setViewport(first, std::max(1, static_cast<Int32>(viewH / lineH)));

      ed::Cursor selA, selB;
      const Bool hasSel = e.selection(selA, selB);

      // ---- what the pointer is over ------------------------------------------
      // Collected while the rows are drawn, not recomputed after: deriving the
      // row's y and span's x twice is how the two end up a pixel apart. Only while
      // hovered and undragged, so a tooltip cannot cover a live selection.
      Vec<const diag::Item*> underPointer;
      ImVec2                 underAt(0.0f, 0.0f);

      const Bool wantHover = hovered
                          && !ImGui::IsMouseDown(ImGuiMouseButton_Left)
                          && !v.diags.empty();

      // ---- the symbol under the pointer ------------------------------------
      //
      // Asked of clangd only when the pointer RESTS on an identifier it has not
      // already asked about. Diagnostics win where they overlap: a red underline
      // is a thing to fix and a declaration is a thing to read, and covering the
      // first with the second would be answering a question nobody asked.
      // ---- keep clangd current, on a debounce -------------------------------
      // The buffer used to reach clangd only inside a completion or a hover
      // request, so the underlines refreshed when you asked for something and
      // at no other time. Now every change starts a short clock, and the file
      // goes out when the clock runs down without another keystroke. Once
      // clangd has it, diagnostics arrive on their own.
      if(!v.lspPath.empty())
      {
          const UInt64 rev = e.revision();
          if(rev != v.syncedRev || v.lspPath != v.syncedPath)
          {
              if(!v.syncPending)
              {
                  v.syncPending = true;
                  v.syncDueS = nowS + 0.30;
              }
              if(nowS >= v.syncDueS)
              {
                  const Str text = e.text();
                  if(lsp::sync(v.lspPath, text, hashOf(text)))
                  {
                      v.syncedRev = rev;
                      v.syncedPath = v.lspPath;
                  }
                  v.syncPending = false;
              }
          }
      }

      // ---- signature help: ask, take, drop ----------------------------------
      if(v.sigIn > 0 && --v.sigIn == 0)
      {
          const Str text = e.text();
          if(!lsp::askSig(v.lspPath, text, hashOf(text), v.sigLine, v.sigCol))
          {
              // Retried while clangd parses, bounded so a server that never
              // comes up is not asked sixty times a second forever.
              v.sigIn = (++v.sigRetries < 120) ? 1 : 0;
          }
      }
      {
          lsp::Sig got;
          if(lsp::takeSig(got) && got.line == v.sigLine)
          {
              v.sigAnswer = got;
          }
      }
      if(v.sigLine >= 0
         && (e.mode() != ed::Mode::MODE_INSERT || e.cursor().line != v.sigLine))
      {
          // Left the call: leaving insert mode, or moving to another line. The
          // line goes to -1 so a reply still in flight is refused on arrival.
          v.sigAnswer = lsp::Sig();
          v.sigIn = 0;
          v.sigLine = -1;
      }

      // ---- outline: open on request, ask, take --------------------------------
      if(v.outlineRequest)
      {
          v.outlineRequest = false;
          if(!v.lspPath.empty())
          {
              v.outlineOpen = true;
              v.outline.clear();
              v.outlineSel = 0;
              v.outlineTop = 0;
              v.outlineIn = 1;
              v.outlineRetries = 0;
          }
      }
      if(v.outlineOpen && v.outlineIn > 0 && --v.outlineIn == 0)
      {
          const Str text = e.text();
          if(!lsp::askOutline(v.lspPath, text, hashOf(text)))
          {
              v.outlineIn = (++v.outlineRetries < 180) ? 1 : 0;
          }
      }
      if(v.outlineOpen)
      {
          Vec<lsp::Symbol> syms;
          if(lsp::takeOutline(syms))
          {
              v.outline = syms;
          }
      }

      Str   hoverWord;
      Int32 hoverLine = -1;
      Int32 hoverCol = -1;

      if(hovered && !ImGui::IsMouseDown(ImGuiMouseButton_Left)
         && !v.lspPath.empty())
      {
          const Int32 hl = static_cast<Int32>((io.MousePos.y - origin.y + v.scrollY) / lineH);
          const Int32 hc = static_cast<Int32>((io.MousePos.x - textX) / charW);

          if(hl >= 0 && hl < e.lineCount() && hc >= 0 && io.MousePos.x >= textX)
          {
              const Str&  ln = e.line(hl);
              const Int32 len = static_cast<Int32>(ln.size());

              const auto wordChar = [](Char ch)
              {
                  return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')
                         || (ch >= '0' && ch <= '9') || ch == '_';
              };

              if(hc < len && wordChar(ln[static_cast<Size>(hc)]))
              {
                  Int32 a = hc;
                  Int32 b = hc;
                  while(a > 0 && wordChar(ln[static_cast<Size>(a - 1)]))
                  {
                      --a;
                  }
                  while(b + 1 < len && wordChar(ln[static_cast<Size>(b + 1)]))
                  {
                      ++b;
                  }

                  // A leading digit means a number, not a name.
                  if(!(ln[static_cast<Size>(a)] >= '0' && ln[static_cast<Size>(a)] <= '9'))
                  {
                      hoverWord = ln.substr(static_cast<Size>(a), static_cast<Size>(b - a + 1));
                      hoverLine = hl;
                      hoverCol = a;
                  }
              }
          }
      }

      if(hoverWord.empty() || hoverLine != v.infoLine || hoverWord != v.infoWord)
      {
          // Moved off, or onto something else. Drop the old answer rather than
          // letting it hang under a different symbol.
          if(hoverWord != v.infoWord || hoverLine != v.infoLine)
          {
              v.infoAnswer = lsp::Info();
          }
          v.infoWord = hoverWord;
          v.infoLine = hoverLine;
          v.infoCol = hoverCol;
          v.infoIn = hoverWord.empty() ? 0 : HOVER_REST_FRAMES;
      }
      else if(v.infoIn > 0 && --v.infoIn == 0)
      {
          const Str text = e.text();

          // A REFUSED question is asked again next frame, for as long as the
          // pointer rests. It is refused while clangd is still parsing, and
          // the old code asked exactly once - so resting on a word during the
          // first second after opening a file produced nothing, and the word
          // looked like one clangd had no opinion about. Namespaces were the
          // usual victim only because they are the first thing you hover.
          if(!lsp::askInfo(v.lspPath, text, hashOf(text), v.infoLine, v.infoCol))
          {
              v.infoIn = 1;
          }
      }

      {
          lsp::Info got;
          if(lsp::takeInfo(got))
          {
              // Only if it still describes what the pointer is on.
              if(got.line == v.infoLine && got.col == v.infoCol)
              {
                  v.infoAnswer = std::move(got);
              }
          }
      }

      for(Int32 l = first; l <= last; ++l)
      {
          const Float32 y = origin.y + static_cast<Float32>(l) * lineH - v.scrollY;

          // Current-line band, behind everything else on the row.
          if(l == e.cursor().line && v.focused)
          {
              dl->AddRectFilled(
                  ImVec2(origin.x + gutterW, y),
                  ImVec2(origin.x + region.x, y + lineH),
                  syn::gruv::BG1
              );
          }

          // ---- line number -------------------------------------------------
          // RELATIVE except in insert, absolute on the caret's own line - vim's
          // `number` + `relativenumber`, so `12j` and `4dd` can be read off the
          // screen. Insert goes absolute: no counted motions there, and a compiler
          // error says "line 42".
          const Bool onCaret = (l == e.cursor().line);
          const Bool absolute = onCaret || (e.mode() == ed::Mode::MODE_INSERT);

          const Int32 shown = absolute ? (l + 1)
                                       : std::abs(l - e.cursor().line);
          std::snprintf(numBuf.data(), numBuf.size(), "%d", shown);
          const Float32 numW = static_cast<Float32>(std::strlen(numBuf.data())) * charW;
          // The caret's own number is left-aligned and bright, the relative ones
          // right-aligned against it, so the column reads as a ruler.
          const Float32 numX = onCaret
              ? (origin.x + charW)
              : (origin.x + gutterW - charW - numW);

          dl->AddText(ImVec2(numX, y), onCaret ? syn::gruv::YELLOW : syn::gruv::FG4, numBuf.data());

          // ---- diagnostic mark in the gutter ---------------------------------
          if(!v.diags.empty())
          {
              const Int32 worst = diag::worstOnLine(v.diags, l + 1);
              if(worst >= 0)
              {
                  const ImU32 mc =
                      severityColor(static_cast<diag::Severity>(worst));

                  const Float32 r = std::max(2.0f, lineH * 0.16f);
                  dl->AddCircleFilled(ImVec2(origin.x + r * 1.6f, y + lineH * 0.5f), r, mc, 10);

                  // The whole gutter ROW is the target, not the four-pixel dot,
                  // which is a mark to notice rather than to land a pointer on.
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
              const Int32 to = (l == selB.line) ? selB.col : len;
              if(to > from)
              {
                  dl->AddRectFilled(
                      ImVec2(textX + static_cast<Float32>(from) * charW, y),
                      ImVec2(textX + static_cast<Float32>(to) * charW, y + lineH),
                      syn::gruv::BG2
                  );
              }
          }

          // The text itself, one draw per colored run.
          Bool open = (l < static_cast<Int32>(blockAt.size())) ? blockAt[static_cast<Size>(l)]
                                                              : false;
          syn::tokenize(src, open, spans);

          // ---- diagnostic underline ------------------------------------------
          // A flat rule, not a squiggle - not worth the vertex count at 60 fps.
          if(!v.diags.empty())
          {
              for(const diag::Item& d : v.diags)
              {
                  if(d.line != l + 1)
                  {
                      continue;
                  }

                  // Column 0 means the compiler did not say where, so the whole line
                  // is marked rather than a span invented for it.
                  const Int32 from = (d.column > 0) ? (d.column - 1) : 0;
                  const Int32 to = (d.column > 0) ? tokenEnd(src, from, len) : len;
                  if(to <= from)
                  {
                      continue;
                  }

                  const ImU32   uc = severityColor(d.severity);
                  const Float32 x0 = textX + static_cast<Float32>(from) * charW;
                  const Float32 x1 = textX + static_cast<Float32>(to) * charW;

                  const Float32 uy = y + lineH - 1.5f * dpiScale();
                  dl->AddLine(
                      ImVec2(x0, uy),
                      ImVec2(x1, uy),
                      uc,
                      std::max(1.0f, 1.5f * dpiScale())
                  );

                  // The hit box is the TEXT at full row height, not the one-pixel
                  // rule under it.
                  if(wantHover && underPointer.empty()
                     && io.MousePos.x >= x0 && io.MousePos.x < x1
                     && io.MousePos.y >= y && io.MousePos.y < y + lineH)
                  {
                      // Everything overlapping that cell, not just this one: an
                      // error and its explaining notes land on the same column.
                      for(const diag::Item& o : v.diags)
                      {
                          if(o.line != l + 1)
                          {
                              continue;
                          }
                          const Int32 of = (o.column > 0) ? (o.column - 1) : 0;
                          const Int32 ot = (o.column > 0) ? tokenEnd(src, of, len) : len;

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

          // ONE CELL AT A TIME, which is why the caret no longer drifts: a span
          // drawn with a single AddText advances by the FONT's glyph advance while
          // the caret advances by `charW`, and the fractional difference is half a
          // cell by column forty. Per-cell placement makes the grid the one source
          // of truth. Whitespace is skipped.
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
                  dl->AddText(
                      ImVec2(textX + static_cast<Float32>(k) * charW, y),
                      col,
                      one.data(),
                      one.data() + 1
                  );
              }
          }
      }

      // ---- caret -------------------------------------------------------------
      // Solid for 530 ms then off for 530 ms, the terminal convention, and forced
      // solid for a moment after every keystroke.
      if(v.focused)
      {
          const Float64 since = nowS - v.lastKeyS;
          const Bool    on = (since < 0.25)
                             || (static_cast<Int32>((since - 0.25) / 0.53) % 2) == 0;
          if(on)
          {
              const Float32 cx = textX + static_cast<Float32>(e.cursor().col) * charW;
              const Float32 cy = origin.y + static_cast<Float32>(e.cursor().line) * lineH
                               - v.scrollY;

              // Block caret in normal mode, bar in insert - the same signal vim
              // gives.
              if(e.mode() == ed::Mode::MODE_INSERT)
              {
                  dl->AddRectFilled(
                      ImVec2(cx, cy),
                      ImVec2(cx + std::max(1.0f, 2.0f * dpiScale()), cy + lineH),
                      syn::gruv::FG1
                  );
              }
              else
              {
                  dl->AddRectFilled(
                      ImVec2(cx, cy),
                      ImVec2(cx + charW, cy + lineH),
                      IM_COL32(0xEB, 0xDB, 0xB2, 0x80)
                  );
              }
          }
      }

      dl->PopClipRect();

      // ---- what the pointer is resting on -------------------------------------
      // The message beside its underline, rather than in the build pane. Suppressed
      // while the completion list is up: two panels a few pixels apart, one of
      // which you are typing into, is worse than either alone.
      if(!underPointer.empty() && !v.popupOpen)
      {
          const Float32 pad = 8.0f * dpiScale();
          const Float32 rowH = lineH;

          // Laid out first, so the box is sized to what it holds rather than to a
          // guess that has to be clamped after.
          struct HoverRow
          {
              Str   text;
              ImU32 col = 0;
              Bool  head = false;
          };

          Vec<HoverRow> rows;
          const Size    show = std::min(underPointer.size(), HOVER_MAX);

          // THE MESSAGE AND NOTHING ELSE. There was a heading per diagnostic -
          // `error  41:12` - and a rule between them, and the box was three
          // rows tall for one line of text. The severity is already the colour
          // of the underline you are pointing at, and the position is where the
          // pointer is; restating both above every message was the cramped
          // part.
          for(Size i = 0; i < show; ++i)
          {
              const diag::Item& d = *underPointer[i];

              if(i > 0)
              {
                  rows.push_back(HoverRow{ Str(), 0u, false });   // a blank line
              }

              Vec<Str> wrapped;
              // Wrapped to what the PANE can hold, not to a fixed width: at 76
              // columns a box anchored mid-pane ran past the editor's edge and
              // over the panel beside it. Never below 24, so a very narrow pane
              // still wraps into words rather than into letters.
              const Size fitCols = std::min(
                  HOVER_COLS,
                  static_cast<Size>(std::max(24.0f, (region.x - pad * 4.0f) / charW))
              );
              wrapTo(d.message, fitCols, wrapped);
              for(const Str& w : wrapped)
              {
                  rows.push_back(HoverRow{ w, syn::gruv::FG1, false });
              }
          }

          if(underPointer.size() > show)
          {
              Array<Char, 64> more;
              std::snprintf(
                  more.data(),
                  more.size(),
                  "+%d more on this line",
                  static_cast<Int32>(underPointer.size() - show)
              );
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

          // Under the underlined text, above it when there is no room - the same
          // rule the completion list follows.
          Float32 hx = underAt.x;
          Float32 hy = underAt.y + lineH + 3.0f * dpiScale();
          if(hy + boxH > origin.y + viewH)
          {
              hy = underAt.y - boxH - 3.0f * dpiScale();
          }
          hx = std::max(origin.x, std::min(hx, origin.x + region.x - boxW));

          // Square. A rounded box reads as a floating card; this is a margin
          // note pinned to a specific span of text, and the corners saying so
          // is the whole difference.
          ImDrawList* fg = ImGui::GetForegroundDrawList();
          fg->AddRectFilled(ImVec2(hx, hy), ImVec2(hx + boxW, hy + boxH), syn::gruv::BG0_H, 0.0f);

          // A neutral hairline. The severity is on the underline, not the box.
          fg->AddRect(ImVec2(hx, hy), ImVec2(hx + boxW, hy + boxH), syn::gruv::BG3, 0.0f);

          for(Size i = 0; i < rows.size(); ++i)
          {
              const HoverRow& r = rows[i];
              const Float32   ry = hy + pad * 0.5f + static_cast<Float32>(i) * rowH;

              if(r.text.empty())
              {
                  // A rule, not whitespace: a gap reads as padding rather than as a
                  // boundary between two separate diagnostics.
                  fg->AddLine(
                      ImVec2(hx + pad, ry + rowH * 0.5f),
                      ImVec2(hx + boxW - pad, ry + rowH * 0.5f),
                      syn::gruv::BG3
                  );
                  continue;
              }

              fg->AddText(ImVec2(hx + pad, ry), r.col, r.text.c_str());
          }
      }

      // ---- a macro, resolved through its renames -----------------------------
      //
      // THE VALUE FIRST, the chain under it. `SERVO_DEFAULT_MIN` is
      // `STEER_CAL_LEFT` is `1230`, and the number is what you came to find out;
      // the path is how to check it, not the answer.
      //
      // The walk stops at the first body that is NOT a bare name, because that
      // is the point where the macro stops being an alias and starts being a
      // value. `CUE_BLINK_PERIOD_MS` is `(CUE_BLINK_ON_MS + CUE_BLINK_OFF_MS)`
      // and expanding that further would produce arithmetic nobody wrote.
      else if(!v.infoWord.empty() && v.infoLine >= 0 && !v.popupOpen
              && v.macros.count(v.infoWord) != 0u)
      {
          struct Step
          {
              Str   name;
              Str   body;
              Str   file;
              Int32 line = 0;
          };

          Vec<Step> chain;
          Str       cur = v.infoWord;

          // Bounded, and it has to be: `#define A B` / `#define B A` is legal
          // and the preprocessor itself only stops because it refuses to expand
          // a macro inside its own expansion.
          for(Int32 hop = 0; hop < 16; ++hop)
          {
              const auto it = v.macros.find(cur);
              if(it == v.macros.end())
              {
                  break;
              }
              chain.push_back(Step{ cur, it->second.body,
                                    it->second.file, it->second.line });

              // A pure rename is one identifier and nothing else.
              const Str& b = it->second.body;
              Bool       bare = !b.empty();
              for(Size i = 0; i < b.size() && bare; ++i)
              {
                  bare = (std::isalnum(static_cast<unsigned char>(b[i])) != 0)
                         || b[i] == '_';
              }
              if(!bare || v.macros.count(b) == 0u)
              {
                  break;
              }

              // Already seen means a cycle, and the value is the name itself.
              Bool loop = false;
              for(const Step& s : chain)
              {
                  loop = loop || (s.name == b);
              }
              if(loop)
              {
                  break;
              }
              cur = b;
          }

          if(!chain.empty())
          {
              const Float32 pad = 8.0f * dpiScale();
              const Float32 rowH = lineH;

              struct MRow
              {
                  Str   text;
                  ImU32 col = 0;
              };
              Vec<MRow> rows;

              // The answer.
              const Str value = chain.back().body;
              rows.push_back(MRow{ v.infoWord + "  =  "
                                   + (value.empty() ? Str("(defined, no value)") : value),
                                   syn::gruv::FG1 });

              // The path to it, only when there was one - a macro that resolves
              // in one step is its own tree and drawing it says nothing.
              if(chain.size() > 1)
              {
                  rows.push_back(MRow{ Str(), syn::gruv::BG3 });
                  for(Size i = 0; i < chain.size(); ++i)
                  {
                      Str indent;
                      for(Size k = 0; k < i; ++k)
                      {
                          indent += "  ";
                      }
                      const Str lead = (i == 0) ? Str() : indent + "\xE2\x94\x94 ";
                      rows.push_back(MRow{ lead + chain[i].name + "  " + chain[i].body,
                                           (i + 1 == chain.size()) ? syn::gruv::FG1
                                                                   : syn::gruv::GRAY });
                  }
              }

              // Where the value actually lives, which is the next thing you want
              // once you know what it is.
              {
                  // `at`, not `last`: `last` is the bottom visible row of the
                  // editor, declared far above this.
                  const Step& at = chain.back();
                  Str         base = at.file;
                  if(const Size sl = base.find_last_of("/\\"); sl != Str::npos)
                  {
                      base = base.substr(sl + 1);
                  }
                  rows.push_back(MRow{ Str(), syn::gruv::BG3 });
                  rows.push_back(MRow{ base + ":" + std::to_string(at.line),
                                       syn::gruv::GRAY });
              }

              Float32 wide = 0.0f;
              for(const MRow& r : rows)
              {
                  if(!r.text.empty())
                  {
                      wide = std::max(wide, ImGui::CalcTextSize(r.text.c_str()).x);
                  }
              }

              const Float32 boxW = wide + pad * 2.0f;
              const Float32 boxH = rowH * static_cast<Float32>(rows.size()) + pad;

              Float32 mx = textX + static_cast<Float32>(v.infoCol) * charW;
              Float32 my = origin.y + static_cast<Float32>(v.infoLine + 1) * lineH
                           - v.scrollY + 3.0f * dpiScale();
              if(my + boxH > origin.y + viewH)
              {
                  my = origin.y + static_cast<Float32>(v.infoLine) * lineH
                       - v.scrollY - boxH - 3.0f * dpiScale();
              }
              mx = std::max(origin.x, std::min(mx, origin.x + region.x - boxW));

              // Purple, so a macro is not mistaken for a declaration at a
              // glance - the two answer different questions.
              ImDrawList* fg = ImGui::GetForegroundDrawList();
              fg->AddRectFilled(
                  ImVec2(mx, my),
                  ImVec2(mx + boxW, my + boxH),
                  syn::gruv::BG0_H,
                  0.0f
              );
              fg->AddRect(ImVec2(mx, my), ImVec2(mx + boxW, my + boxH), syn::gruv::BG3, 0.0f);   // neutral: no coloured edges

              for(Size i = 0; i < rows.size(); ++i)
              {
                  const MRow&   r = rows[i];
                  const Float32 ry = my + pad * 0.5f + static_cast<Float32>(i) * rowH;

                  if(r.text.empty())
                  {
                      fg->AddLine(
                          ImVec2(mx + pad, ry + rowH * 0.5f),
                          ImVec2(mx + boxW - pad, ry + rowH * 0.5f),
                          syn::gruv::BG3
                      );
                      continue;
                  }
                  fg->AddText(ImVec2(mx + pad, ry), r.col, r.text.c_str());
              }
          }
      }

      // ---- what clangd knows about the symbol under the pointer --------------
      //
      // The SAME box as the diagnostic tooltip above - square, dark plate, one
      // coloured rule at the top - because they answer the same kind of
      // question about the same span of text. Only the accent differs: a
      // diagnostic is coloured by severity, a declaration is blue, so which one
      // you are looking at is clear before you have read a word of it.
      //
      // Suppressed while a diagnostic is showing. Two boxes over one span is
      // worse than either.
      else if(!v.infoAnswer.sig.empty() && !v.popupOpen
              && v.infoLine >= 0 && !v.infoWord.empty())
      {
          const Float32 pad = 8.0f * dpiScale();
          const Float32 rowH = lineH;

          struct InfoRow
          {
              Str   text;
              ImU32 col = 0;
          };
          Vec<InfoRow> rows;

          // The declaration, one row per line - clangd wraps a long parameter
          // list itself, and re-wrapping it loses that.
          Size at = 0;
          while(at <= v.infoAnswer.sig.size())
          {
              const Size nl = v.infoAnswer.sig.find('\n', at);
              const Size stop = (nl == Str::npos) ? v.infoAnswer.sig.size() : nl;
              rows.push_back(InfoRow{ v.infoAnswer.sig.substr(at, stop - at),
                                      syn::gruv::FG1 });
              if(nl == Str::npos)
              {
                  break;
              }
              at = nl + 1;
          }

          if(!v.infoAnswer.doc.empty())
          {
              rows.push_back(InfoRow{ Str(), syn::gruv::BG3 });   // a rule

              Vec<Str> wrapped;
              // Wrapped to what the PANE can hold, not to a fixed width: at 76
              // columns a box anchored mid-pane ran past the editor's edge and
              // over the panel beside it. Never below 24, so a very narrow pane
              // still wraps into words rather than into letters.
              const Size fitCols = std::min(
                  HOVER_COLS,
                  static_cast<Size>(std::max(24.0f, (region.x - pad * 4.0f) / charW))
              );
              wrapTo(v.infoAnswer.doc, fitCols, wrapped);

              // Bounded. A doc comment in this project runs to paragraphs, and a
              // tooltip taller than the window hides the code it describes.
              const Size cap = 12;
              for(Size i = 0; i < wrapped.size() && i < cap; ++i)
              {
                  rows.push_back(InfoRow{ wrapped[i], syn::gruv::GRAY });
              }
              if(wrapped.size() > cap)
              {
                  rows.push_back(InfoRow{ Str("..."), syn::gruv::BG3 });
              }
          }

          Float32 wide = 0.0f;
          for(const InfoRow& r : rows)
          {
              if(!r.text.empty())
              {
                  wide = std::max(wide, ImGui::CalcTextSize(r.text.c_str()).x);
              }
          }

          const Float32 boxW = wide + pad * 2.0f;
          const Float32 boxH = rowH * static_cast<Float32>(rows.size()) + pad;

          // Pinned to the identifier, not to the pointer: the box stays put
          // while the pointer moves within the word.
          Float32 ix = textX + static_cast<Float32>(v.infoCol) * charW;
          Float32 iy = origin.y + static_cast<Float32>(v.infoLine + 1) * lineH
                       - v.scrollY + 3.0f * dpiScale();
          if(iy + boxH > origin.y + viewH)
          {
              iy = origin.y + static_cast<Float32>(v.infoLine) * lineH
                   - v.scrollY - boxH - 3.0f * dpiScale();
          }
          ix = std::max(origin.x, std::min(ix, origin.x + region.x - boxW));

          ImDrawList* fg = ImGui::GetForegroundDrawList();
          fg->AddRectFilled(ImVec2(ix, iy), ImVec2(ix + boxW, iy + boxH), syn::gruv::BG0_H, 0.0f);
          fg->AddRect(ImVec2(ix, iy), ImVec2(ix + boxW, iy + boxH), syn::gruv::BG3, 0.0f);   // neutral: no coloured edges

          for(Size i = 0; i < rows.size(); ++i)
          {
              const InfoRow& r = rows[i];
              const Float32  ry = iy + pad * 0.5f + static_cast<Float32>(i) * rowH;

              if(r.text.empty())
              {
                  fg->AddLine(
                      ImVec2(ix + pad, ry + rowH * 0.5f),
                      ImVec2(ix + boxW - pad, ry + rowH * 0.5f),
                      syn::gruv::BG3
                  );
                  continue;
              }
              fg->AddText(ImVec2(ix + pad, ry), r.col, r.text.c_str());
          }
      }

      // ---- signature help ---------------------------------------------------
      // One row, directly above the line being typed: the signature with the
      // active parameter in full colour and the rest dimmed. Nothing else -
      // no documentation, no overload count. It is there to answer "which
      // argument am I on", and a box that answers more than that gets in the
      // way of the line it is about.
      if(!v.sigAnswer.label.empty() && e.mode() == ed::Mode::MODE_INSERT
         && !v.popupOpen && !v.outlineOpen && e.cursor().line == v.sigLine)
      {
          const Str&    label = v.sigAnswer.label;
          const Float32 pad = 6.0f * dpiScale();
          const Float32 boxW = ImGui::CalcTextSize(label.c_str()).x + pad * 2.0f;
          const Float32 boxH = lineH + pad;

          Float32 sx = textX + static_cast<Float32>(std::max(0, v.sigCol - 1)) * charW;
          Float32 sy = origin.y + static_cast<Float32>(v.sigLine) * lineH - v.scrollY - boxH
                     - 2.0f * dpiScale();
          if(sy < origin.y)
          {
              // No room above: below the line instead, like the popups do.
              sy = origin.y + static_cast<Float32>(v.sigLine + 1) * lineH - v.scrollY
                 + 2.0f * dpiScale();
          }
          sx = std::max(origin.x, std::min(sx, origin.x + region.x - boxW));

          ImDrawList* fg = ImGui::GetForegroundDrawList();
          fg->AddRectFilled(ImVec2(sx, sy), ImVec2(sx + boxW, sy + boxH), syn::gruv::BG1, 0.0f);
          fg->AddRect(ImVec2(sx, sy), ImVec2(sx + boxW, sy + boxH), syn::gruv::BG3, 0.0f);

          // Three pieces: before, the active parameter, after. Drawn as
          // separate runs at measured offsets rather than as one string with a
          // colour change, which ImGui cannot do.
          const Int32 ab = v.sigAnswer.activeBegin;
          const Int32 ae = v.sigAnswer.activeEnd;
          const Bool  marked = ab >= 0 && ae > ab && static_cast<Size>(ae) <= label.size();

          const Float32 tx = sx + pad;
          const Float32 ty = sy + pad * 0.5f;
          if(!marked)
          {
              fg->AddText(ImVec2(tx, ty), syn::gruv::FG4, label.c_str());
          }
          else
          {
              const Str before = label.substr(0, static_cast<Size>(ab));
              const Str active = label.substr(static_cast<Size>(ab), static_cast<Size>(ae - ab));
              const Str after = label.substr(static_cast<Size>(ae));

              Float32 x = tx;
              fg->AddText(ImVec2(x, ty), syn::gruv::FG4, before.c_str());
              x += ImGui::CalcTextSize(before.c_str()).x;
              fg->AddText(ImVec2(x, ty), syn::gruv::FG1, active.c_str());
              x += ImGui::CalcTextSize(active.c_str()).x;
              fg->AddText(ImVec2(x, ty), syn::gruv::FG4, after.c_str());
          }
      }

      // ---- outline ----------------------------------------------------------
      // A list in the top-left of the text, the width of its longest name.
      // Members sit one indent under their owner; the tag on the left says what
      // each thing is. j/k move, Enter goes, Escape closes.
      if(v.outlineOpen)
      {
          const Int32 n = static_cast<Int32>(v.outline.size());
          const Int32 shown = std::max(1, std::min(n, MAX_VISIBLE));

          if(v.outlineSel < v.outlineTop)
          {
              v.outlineTop = v.outlineSel;
          }
          if(v.outlineSel >= v.outlineTop + shown)
          {
              v.outlineTop = v.outlineSel - shown + 1;
          }
          v.outlineTop = std::max(0, std::min(v.outlineTop, std::max(0, n - shown)));

          const Float32 pad = 8.0f * dpiScale();
          const Float32 tagW = ImGui::CalcTextSize("struct").x + pad;
          Float32       nameW = ImGui::CalcTextSize("...").x;
          for(const lsp::Symbol& s : v.outline)
          {
              nameW = std::max(
                  nameW,
                  ImGui::CalcTextSize(s.name.c_str()).x
                      + static_cast<Float32>(s.depth) * charW * 2.0f
              );
          }

          const Float32 rowH = lineH + 2.0f * dpiScale();
          const Float32 barW = (n > shown) ? 4.0f * dpiScale() : 0.0f;
          const Float32 boxW = tagW + nameW + pad * 2.0f + barW;
          const Float32 boxH = rowH * static_cast<Float32>(shown) + pad;
          const Float32 px = textX;
          const Float32 py = origin.y + 2.0f * dpiScale();

          ImDrawList* fg = ImGui::GetForegroundDrawList();
          fg->AddRectFilled(ImVec2(px, py), ImVec2(px + boxW, py + boxH), syn::gruv::BG1, 0.0f);
          fg->AddRect(ImVec2(px, py), ImVec2(px + boxW, py + boxH), syn::gruv::BG3, 0.0f);

          if(n == 0)
          {
              // Waiting on clangd, or a file with nothing in it. Either way the
              // honest thing to draw is that there is nothing yet.
              fg->AddText(ImVec2(px + pad * 0.5f, py + pad * 0.5f), syn::gruv::GRAY, "...");
          }

          for(Int32 r = 0; r < shown && r < n; ++r)
          {
              const Int32        idx = v.outlineTop + r;
              const lsp::Symbol& s = v.outline[static_cast<Size>(idx)];
              const Float32      ry = py + pad * 0.5f + static_cast<Float32>(r) * rowH;

              if(idx == v.outlineSel)
              {
                  fg->AddRectFilled(
                      ImVec2(px + 2.0f, ry),
                      ImVec2(px + boxW - barW - 2.0f, ry + rowH),
                      syn::gruv::BG2
                  );
              }

              fg->AddText(ImVec2(px + pad * 0.5f, ry), syn::gruv::GRAY, s.kind.c_str());
              fg->AddText(
                  ImVec2(px + tagW + static_cast<Float32>(s.depth) * charW * 2.0f, ry),
                  syn::gruv::FG1,
                  s.name.c_str()
              );
          }

          if(n > shown)
          {
              const Float32 trackX = px + boxW - barW - 2.0f * dpiScale();
              const Float32 frac = static_cast<Float32>(shown) / static_cast<Float32>(n);
              const Float32 thumbH = std::max(rowH * 0.6f, boxH * frac);
              const Float32 travel = boxH - thumbH;
              const Float32 at = static_cast<Float32>(v.outlineTop)
                                 / static_cast<Float32>(n - shown);

              fg->AddRectFilled(
                  ImVec2(trackX, py),
                  ImVec2(trackX + barW, py + boxH),
                  syn::gruv::BG2
              );
              fg->AddRectFilled(
                  ImVec2(trackX, py + travel * at),
                  ImVec2(trackX + barW, py + travel * at + thumbH),
                  syn::gruv::GRAY
              );
          }
      }

      // ---- completion popup --------------------------------------------------
      // Drawn on the FOREGROUND draw list so it is not clipped by the editor
      // child, which is what lets it hang below the last visible line.
      if(v.popupOpen && !v.outlineOpen)
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
              const Int32 n = static_cast<Int32>(hits.size());
              const Int32 sel = std::max(0, std::min(v.popupSel, n - 1));

              // ---- the visible window ----
              // The selection drags it, clamped after - so a list that just
              // filtered from two hundred to three is not parked past the end.
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

              // Widest name and detail ACROSS THE WHOLE LIST, not just the visible
              // rows: sizing to the window makes the box change width as it scrolls.
              Float32 nameW = 0.0f;
              Float32 detW = 0.0f;
              for(const Row& it : hits)
              {
                  nameW = std::max(nameW, ImGui::CalcTextSize(it.name).x);
                  if(it.detail != nullptr && it.detail[0] != 0)
                  {
                      detW = std::max(detW, ImGui::CalcTextSize(it.detail).x);
                  }
              }

              // clangd's `detail` is a full C++ type and can be a hundred
              // characters; capped so one cannot push the box off the pane.
              detW = std::min(detW, charW * 44.0f);

              const Float32 pad = 8.0f * dpiScale();
              const Float32 tagW = ImGui::CalcTextSize("##").x + pad;
              const Float32 rowH = lineH + 2.0f * dpiScale();
              const Float32 barW = (n > shown) ? 4.0f * dpiScale() : 0.0f;
              const Float32 boxW = tagW + nameW + pad * 2.0f + detW + pad * 2.0f + barW;
              const Float32 boxH = rowH * static_cast<Float32>(shown) + pad;

              // Anchored to the START of the word, not the caret, so the list lines
              // up with what it is completing.
              Float32 px = textX + static_cast<Float32>(startCol) * charW;
              Float32 py = origin.y
                         + static_cast<Float32>(e.cursor().line + 1) * lineH - v.scrollY;

              // Flip above the caret rather than fall off the bottom.
              if(py + boxH > origin.y + viewH)
              {
                  py = origin.y + static_cast<Float32>(e.cursor().line) * lineH
                     - v.scrollY - boxH;
              }

              px = std::max(origin.x, std::min(px, origin.x + region.x - boxW));

              // NO WHEEL SCROLLING HERE: the buffer's own wheel handler runs long
              // before this box has a position, so a wheel over the popup would
              // move the list AND the code under it. Keys only.
              // Square, like every other corner in the program.
              ImDrawList* fg = ImGui::GetForegroundDrawList();
              fg->AddRectFilled(ImVec2(px, py), ImVec2(px + boxW, py + boxH), syn::gruv::BG1, 0.0f);
              fg->AddRect(ImVec2(px, py), ImVec2(px + boxW, py + boxH), syn::gruv::BG3, 0.0f);

              for(Int32 r = 0; r < shown; ++r)
              {
                  const Int32   idx = v.popupTop + r;
                  const Row&    it = hits[static_cast<Size>(idx)];
                  const Float32 ry = py + pad * 0.5f
                                    + static_cast<Float32>(r) * rowH;

                  if(idx == sel)
                  {
                      // Stops short of the scroll track, so the highlight does not
                      // paint over the one thing saying where in the list you are.
                      fg->AddRectFilled(
                          ImVec2(px + 2.0f, ry),
                          ImVec2(px + boxW - barW - 2.0f, ry + rowH),
                          syn::gruv::BG2
                      );
                  }

                  fg->AddText(ImVec2(px + pad * 0.5f, ry), syn::gruv::GRAY, kindTag(it.kind));
                  fg->AddText(ImVec2(px + tagW, ry), kindColor(it.kind), it.name);

                  if(it.detail != nullptr && it.detail[0] != 0)
                  {
                      // Clipped, not wrapped: a row that wraps stops being a row.
                      const Float32 dx = px + tagW + nameW + pad * 2.0f;
                      fg->PushClipRect(ImVec2(dx, ry), ImVec2(dx + detW, ry + rowH), true);
                      fg->AddText(ImVec2(dx, ry), syn::gruv::FG4, it.detail);
                      fg->PopClipRect();
                  }
              }

              // ---- the scroll indicator ----
              // Position only, no dragging. It answers "is there more below" - a
              // question the list cannot answer once it stops showing all of itself.
              if(n > shown)
              {
                  const Float32 trackX = px + boxW - barW - 2.0f * dpiScale();
                  const Float32 frac = static_cast<Float32>(shown)
                                       / static_cast<Float32>(n);
                  const Float32 thumbH = std::max(rowH * 0.6f, boxH * frac);
                  const Float32 travel = boxH - thumbH;
                  const Float32 at = (n == shown) ? 0.0f
                                       : static_cast<Float32>(v.popupTop)
                                         / static_cast<Float32>(n - shown);

                  fg->AddRectFilled(
                      ImVec2(trackX, py),
                      ImVec2(trackX + barW, py + boxH),
                      syn::gruv::BG2
                  );
                  fg->AddRectFilled(
                      ImVec2(trackX, py + travel * at),
                      ImVec2(trackX + barW, py + travel * at + thumbH),
                      syn::gruv::GRAY
                  );
              }

              // The selected entry's one-line doc, under the box - only one, so the
              // popup stays a list. "10 of 137" on the right separates a short list
              // from a truncated one.
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
                  const Float32 dy = py + boxH + 2.0f * dpiScale();
                  const Float32 docW = haveDoc ? ImGui::CalcTextSize(cur.doc).x : 0.0f;
                  const Float32 tw = (tally[0] != '\0')
                                     ? ImGui::CalcTextSize(tally.data()).x + pad
                                     : 0.0f;
                  const Float32 dw = std::min(docW + tw + pad * 2.0f, boxW * 1.5f);

                  // Square, like the list above it and every other box.
                  fg->AddRectFilled(
                      ImVec2(px, dy),
                      ImVec2(px + dw, dy + rowH),
                      syn::gruv::BG1,
                      0.0f
                  );

                  if(haveDoc)
                  {
                      fg->PushClipRect(
                          ImVec2(px + pad, dy),
                          ImVec2(px + dw - tw - pad, dy + rowH),
                          true
                      );
                      fg->AddText(
                          ImVec2(px + pad, dy + 1.0f * dpiScale()),
                          syn::gruv::FG1,
                          cur.doc
                      );
                      fg->PopClipRect();
                  }
                  if(tally[0] != '\0')
                  {
                      fg->AddText(
                          ImVec2(px + dw - tw, dy + 1.0f * dpiScale()),
                          syn::gruv::GRAY,
                          tally.data()
                      );
                  }
              }
          }
      }

      // ---- status line -------------------------------------------------------
      // The casing tone, not BG1: a strip lighter than both the well above and
      // the panel below was a third surface. One hairline marks the well's edge.
      const Float32 sy = origin.y + viewH;
      dl->AddRectFilled(
          ImVec2(origin.x, sy),
          ImVec2(origin.x + region.x, sy + statusH),
          ImGui::GetColorU32(ImGuiCol_WindowBg)
      );
      dl->AddLine(
          ImVec2(origin.x, sy + 0.5f),
          ImVec2(origin.x + region.x, sy + 0.5f),
          ImGui::GetColorU32(ImGuiCol_Border)
      );

      // From the style, captured at the top of drawCode.
      const Float32 inset = statusInset;
      const Float32 gap = statusGap;
      const Float32 ty = sy + ImGui::GetStyle().FramePadding.y;

      // The mode, in its own colour but as plain text: a filled chip was the one
      // coloured block in an otherwise flat line.
      {
          const Char*   mn = modeName(e.mode());
          const Float32 mw = ImGui::CalcTextSize(mn).x;
          dl->AddText(ImVec2(origin.x + inset, ty), modeColor(e.mode()), mn);

          Array<Char, 64> pos;
          std::snprintf(pos.data(), pos.size(), "%d:%d", e.cursor().line + 1, e.cursor().col + 1);
          const Float32 pw = ImGui::CalcTextSize(pos.data()).x;
          dl->AddText(ImVec2(origin.x + region.x - pw - inset, ty), syn::gruv::FG4, pos.data());

          // ---- clangd, always ------------------------------------------------
          // Present in every state, not only when wrong: an empty completion list
          // looks identical whether the server has nothing to suggest, is still
          // parsing, or is not running - and it also says when a question is IN
          // FLIGHT, which otherwise looks like "no suggestions".
          Float32 rightOf = origin.x + region.x - pw - inset - gap;
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
              rightOf -= tw + gap;
          }

          // The middle slot, in priority order: the command line (you are looking
          // straight at it), then a transient note, then a hint.
          Str   mid;
          ImU32 midCol = syn::gruv::GRAY;

          if(e.mode() == ed::Mode::MODE_COMMAND)
          {
              mid = Str(1, e.commandPrefix()) + e.commandLine();
              midCol = (e.commandPrefix() == ':') ? syn::gruv::FG1
                                                  : syn::gruv::AQUA;
          }
          else if(!v.note.empty())
          {
              const Float64 age = nowS - v.noteAtS;
              if(age < NOTE_HOLD_S + NOTE_FADE_S)
              {
                  mid = v.note;

                  // Fades rather than vanishing: a message that disappears between
                  // glances is one you are never sure you saw.
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
              // OUTRANKS "click to edit" on purpose: the tag on the right says
              // THAT it failed, this says WHY. "not installed" and "exited during
              // startup" want different things done about them.
              mid = lsp::status();
              midCol = syn::gruv::RED;
          }
          else if(!v.focused)
          {
              mid = "click to edit";
          }

          if(!mid.empty())
          {
              dl->AddText(ImVec2(origin.x + inset + mw + gap, ty), midCol, mid.c_str());
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
                  dl->AddText(
                      ImVec2(rightOf - dw, ty),
                      (errs > 0) ? syn::gruv::RED : syn::gruv::YELLOW,
                      db.data()
                  );
              }
          }
      }

      ImGui::EndChild();
      ImGui::PopStyleVar(2);
      ImGui::PopStyleColor();
      ImGui::PopFont();

      return changed;
  }

}
