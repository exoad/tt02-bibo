#include "code_view.hpp"

#include "complete.hpp"
#include "syntax.hpp"
#include "theme.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace ui {
namespace {

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
constexpr Size MIN_PREFIX  = 2;
constexpr Size MAX_ENTRIES = 8;

ImU32 kindColor(cmpl::Kind k) noexcept
{
    switch(k)
    {
    case cmpl::Kind::KIND_FUNCTION: return syn::gruv::GREEN;
    case cmpl::Kind::KIND_TYPE:     return syn::gruv::YELLOW;
    case cmpl::Kind::KIND_MACRO:    return syn::gruv::PURPLE;
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
    case cmpl::Kind::KIND_KEYWORD:
    default:                        return "kw";
    }
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
    Char  numBuf[16];
    std::snprintf(numBuf, sizeof(numBuf), "%d", std::max(1, e.lineCount()));
    // +3 rather than +2: one column for the diagnostic mark on the left, and
    // one of margin on each side of the number itself.
    const Float32 gutterW = (static_cast<Float32>(std::strlen(numBuf)) + 3.0f) * charW;

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

        if(v.popupOpen)
        {
            Vec<const cmpl::Item*> hits;
            cmpl::suggest(v.popupPrefix, hits, MAX_ENTRIES);

            if(!hits.empty())
            {
                const Int32 n = static_cast<Int32>(hits.size());

                if(ImGui::IsKeyPressed(ImGuiKey_DownArrow, true)
                   || (ctrl && ImGui::IsKeyPressed(ImGuiKey_N, true)))
                    v.popupSel = (v.popupSel + 1) % n;

                if(ImGui::IsKeyPressed(ImGuiKey_UpArrow, true)
                   || (ctrl && ImGui::IsKeyPressed(ImGuiKey_P, true)))
                    v.popupSel = (v.popupSel + n - 1) % n;

                v.popupSel = std::max(0, std::min(v.popupSel, n - 1));

                const Bool accept = ImGui::IsKeyPressed(ImGuiKey_Tab, false)
                                 || ImGui::IsKeyPressed(ImGuiKey_Enter, false);
                if(accept)
                {
                    e.replaceWordBeforeCursor(hits[static_cast<Size>(v.popupSel)]->name);
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
            if(eatEnter && s.sp == ed::Special::SPECIAL_ENTER) continue;
            if(eatTab   && s.sp == ed::Special::SPECIAL_TAB)   continue;
            if(eatEsc   && s.sp == ed::Special::SPECIAL_ESC)   continue;

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

        if(word != v.popupPrefix)
        {
            v.popupPrefix = word;
            v.popupSel    = 0;
            v.dismissed   = false;   // a different word is a new question
        }

        if(word.size() >= MIN_PREFIX && !v.dismissed)
        {
            Vec<const cmpl::Item*> hits;
            v.popupOpen = (cmpl::suggest(word, hits, MAX_ENTRIES) > 0);
        }
        else
        {
            v.popupOpen = false;
        }
    }
    else
    {
        v.popupOpen = false;
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

    ed::Cursor selA, selB;
    const Bool hasSel = e.selection(selA, selB);

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
        std::snprintf(numBuf, sizeof(numBuf), "%d", shown);
        const Float32 numW = static_cast<Float32>(std::strlen(numBuf)) * charW;
        // The caret's own number is left-aligned and bright; the relative ones
        // are right-aligned against it, so the column of distances reads as a
        // ruler rather than as a list of numbers.
        const Float32 numX = onCaret
            ? (origin.x + charW)
            : (origin.x + gutterW - charW - numW);

        dl->AddText(ImVec2(numX, y),
                    onCaret ? syn::gruv::YELLOW : syn::gruv::FG4,
                    numBuf);

        // ---- diagnostic mark in the gutter ---------------------------------
        if(!v.diags.empty())
        {
            const Int32 worst = diag::worstOnLine(v.diags, l + 1);
            if(worst >= 0)
            {
                const ImU32 mc =
                    (worst == static_cast<Int32>(diag::Severity::SEVERITY_ERR))
                        ? syn::gruv::RED
                        : (worst == static_cast<Int32>(diag::Severity::SEVERITY_WARN))
                              ? syn::gruv::YELLOW
                              : syn::gruv::BLUE;

                const Float32 r = std::max(2.0f, lineH * 0.16f);
                dl->AddCircleFilled(ImVec2(origin.x + r * 1.6f, y + lineH * 0.5f),
                                    r, mc, 10);
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

                const ImU32 uc =
                    (d.severity == diag::Severity::SEVERITY_ERR)
                        ? syn::gruv::RED
                        : (d.severity == diag::Severity::SEVERITY_WARN)
                              ? syn::gruv::YELLOW
                              : syn::gruv::BLUE;

                const Float32 uy = y + lineH - 1.5f * dpiScale();
                dl->AddLine(ImVec2(textX + static_cast<Float32>(from) * charW, uy),
                            ImVec2(textX + static_cast<Float32>(to) * charW, uy),
                            uc, std::max(1.0f, 1.5f * dpiScale()));
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

                const Char one[2] = { ch, '\0' };
                dl->AddText(ImVec2(textX + static_cast<Float32>(k) * charW, y),
                            col, one, one + 1);
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

    // ---- completion popup --------------------------------------------------
    // Drawn on the FOREGROUND draw list so it is not clipped by the editor
    // child, which is what lets it hang below the last visible line.
    if(v.popupOpen)
    {
        Vec<const cmpl::Item*> hits;
        cmpl::suggest(v.popupPrefix, hits, MAX_ENTRIES);

        if(hits.empty())
        {
            v.popupOpen = false;
        }
        else
        {
            const Int32 sel = std::max(0, std::min(v.popupSel,
                                                   static_cast<Int32>(hits.size()) - 1));

            // Widest name and widest detail, so the two columns line up rather
            // than jittering as the list filters down.
            Float32 nameW = 0.0f;
            Float32 detW  = 0.0f;
            for(const cmpl::Item* it : hits)
            {
                nameW = std::max(nameW, ImGui::CalcTextSize(it->name).x);
                if(it->detail != nullptr && it->detail[0] != 0)
                    detW = std::max(detW, ImGui::CalcTextSize(it->detail).x);
            }

            const Float32 pad  = 8.0f * dpiScale();
            const Float32 tagW = ImGui::CalcTextSize("##").x + pad;
            const Float32 rowH = lineH + 2.0f * dpiScale();
            const Float32 boxW = tagW + nameW + pad * 2.0f + detW + pad * 2.0f;
            const Float32 boxH = rowH * static_cast<Float32>(hits.size()) + pad;

            // Anchored to the START of the word, so the list lines up with what
            // it is completing rather than with the caret.
            Float32 px = textX
                       + static_cast<Float32>(e.cursor().col
                                              - static_cast<Int32>(v.popupPrefix.size()))
                         * charW;
            Float32 py = origin.y
                       + static_cast<Float32>(e.cursor().line + 1) * lineH - v.scrollY;

            // Flip above the caret rather than fall off the bottom.
            if(py + boxH > origin.y + viewH)
                py = origin.y + static_cast<Float32>(e.cursor().line) * lineH
                   - v.scrollY - boxH;

            px = std::max(origin.x, std::min(px, origin.x + region.x - boxW));

            ImDrawList* fg = ImGui::GetForegroundDrawList();
            fg->AddRectFilled(ImVec2(px, py), ImVec2(px + boxW, py + boxH),
                              syn::gruv::BG1, 3.0f * dpiScale());
            fg->AddRect(ImVec2(px, py), ImVec2(px + boxW, py + boxH),
                        syn::gruv::BG3, 3.0f * dpiScale());

            for(Size i = 0; i < hits.size(); ++i)
            {
                const cmpl::Item* it = hits[i];
                const Float32     ry = py + pad * 0.5f
                                     + static_cast<Float32>(i) * rowH;

                if(static_cast<Int32>(i) == sel)
                {
                    fg->AddRectFilled(ImVec2(px + 2.0f, ry),
                                      ImVec2(px + boxW - 2.0f, ry + rowH),
                                      syn::gruv::BG2);
                }

                fg->AddText(ImVec2(px + pad * 0.5f, ry), syn::gruv::GRAY,
                            kindTag(it->kind));
                fg->AddText(ImVec2(px + tagW, ry), kindColor(it->kind), it->name);

                if(it->detail != nullptr && it->detail[0] != 0)
                {
                    fg->AddText(ImVec2(px + tagW + nameW + pad * 2.0f, ry),
                                syn::gruv::FG4, it->detail);
                }
            }

            // The selected entry's one-line doc, under the box. Only one, so
            // the popup stays a list rather than becoming a manual.
            const cmpl::Item* cur = hits[static_cast<Size>(sel)];
            if(cur->doc != nullptr && cur->doc[0] != 0)
            {
                const Float32 dy = py + boxH + 2.0f * dpiScale();
                const Float32 dw = ImGui::CalcTextSize(cur->doc).x + pad * 2.0f;
                fg->AddRectFilled(ImVec2(px, dy), ImVec2(px + dw, dy + rowH),
                                  syn::gruv::BG1, 3.0f * dpiScale());
                fg->AddText(ImVec2(px + pad, dy + 1.0f * dpiScale()),
                            syn::gruv::FG1, cur->doc);
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

        Char pos[64];
        std::snprintf(pos, sizeof(pos), "%d:%d",
                      e.cursor().line + 1, e.cursor().col + 1);
        const Float32 pw = ImGui::CalcTextSize(pos).x;
        dl->AddText(ImVec2(origin.x + region.x - pw - pad, ty), syn::gruv::FG4, pos);

        // The middle slot, in priority order: what you are typing, then what
        // just happened, then a hint. A transient note outranks the hint
        // because it is news; the command line outranks everything because you
        // are looking straight at it.
        Str   mid;
        ImU32 midCol = syn::gruv::GRAY;

        if(e.mode() == ed::Mode::MODE_COMMAND)
        {
            mid    = ":" + e.commandLine();
            midCol = syn::gruv::FG1;
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
                Char db[64];
                std::snprintf(db, sizeof(db), "%d e  %d w", errs, warns);
                const Float32 dw = ImGui::CalcTextSize(db).x;
                dl->AddText(ImVec2(origin.x + region.x - pw - dw - pad * 3.0f, ty),
                            (errs > 0) ? syn::gruv::RED : syn::gruv::YELLOW, db);
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
