// Draws an ed::Editor: Gruvbox dark, a line-number gutter, and a vim status
// line. Also the only place that turns ImGui key state into ed::Key.
//
// Kept out of app_ui.cpp because it is the one panel in this app that owns the
// keyboard. Everything else here reacts to clicks; this reads the raw input
// queue and must be able to say "I have the keys now" without the rest of the UI
// having to know why.
#pragma once

#include "diagnostics.hpp"
#include "editor.hpp"
#include "shared.hpp"

#include "imgui.h"

namespace ui {

// One editor's view state. Scroll position and focus live here rather than in
// ed::Editor, because they are properties of a WINDOW showing a buffer, not of
// the buffer - the same file in two panes would want two of these and one
// Editor.
struct CodeView
{
    Float32 scrollY      = 0.0f;
    Bool    focused      = false;

    // Set while the caret has moved and the view has not caught up yet. Drawing
    // clears it. Kept as state rather than computed because a caret move and the
    // frame that must react to it are not the same frame.
    Bool    followCaret  = true;

    // Blink phase for the caret, in seconds. Reset on every keystroke so the
    // caret is always solid while you are typing - a caret that blinks out
    // mid-keystroke reads as dropped input.
    Float64 lastKeyS     = 0.0;

    // ---- completion popup ------------------------------------------------
    // Open only in insert mode, and only while the caret sits at the end of an
    // identifier of at least MIN_PREFIX characters. `dismissed` remembers that
    // the user pressed escape on THIS word, so it does not pop straight back up
    // on the next keystroke - which is the difference between a suggestion and
    // an interruption.
    Bool  popupOpen   = false;
    Bool  dismissed   = false;
    Int32 popupSel    = 0;
    Str   popupPrefix;

    // ---- diagnostics -----------------------------------------------------
    // Set by the caller after a build. Empty means "no build has run", which
    // is drawn as nothing rather than as "no problems" - the editor has no
    // opinion about code it has not compiled.
    Vec<diag::Item> diags;

    // ---- transient status message ----------------------------------------
    // "3 lines yanked", "saved sketch.c". Shown on the status line and faded
    // out, because a message that stays forever stops being noticed and a
    // message that vanishes instantly is never read.
    Str     note;
    Float64 noteAtS = 0.0;
};

// How long a status note stays fully visible, and how long it takes to fade.
inline constexpr Float64 NOTE_HOLD_S = 2.5;
inline constexpr Float64 NOTE_FADE_S = 0.8;

// Draws `e` into the current window at `size`, handling input when focused.
//
// `nowS` is any monotonic clock, used only for the caret blink. Returns true if
// the buffer or caret changed this frame, which the caller uses to decide
// whether the file is worth saving.
Bool drawCode(CodeView& v, ed::Editor& e, const ImVec2& size, Float64 nowS);

// Shows `text` on the editor's status line for a few seconds. Used for :w, for
// yanks, and for anything else the editor wants to say in passing.
Void setNote(CodeView& v, const Str& text, Float64 nowS);

} // namespace ui
