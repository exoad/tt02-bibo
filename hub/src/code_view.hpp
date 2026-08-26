// Draws an ed::Editor: Gruvbox dark, a line-number gutter, and a vim status
// line. Also the only place that turns ImGui key state into ed::Key.
//
// Kept out of app_ui.cpp because it is the one panel in this app that owns the
// keyboard. Everything else here reacts to clicks; this reads the raw input
// queue and must be able to say "I have the keys now" without the rest of the UI
// having to know why.
#pragma once

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
};

// Draws `e` into the current window at `size`, handling input when focused.
//
// `nowS` is any monotonic clock, used only for the caret blink. Returns true if
// the buffer or caret changed this frame, which the caller uses to decide
// whether the file is worth saving.
Bool drawCode(CodeView& v, ed::Editor& e, const ImVec2& size, Float64 nowS);

} // namespace ui
