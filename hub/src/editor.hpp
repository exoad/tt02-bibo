// A small modal text editor: buffer, cursor, vim keybindings, undo.
//
// WHY THIS EXISTS AT ALL. ImGui's InputTextMultiline owns its own key handling
// (it drives stb_textedit internally), so modal editing cannot be layered on top
// of it - `h` would insert an h before any interceptor saw it. Owning the buffer
// and the keys is what makes vim mode, auto-closing braces and per-token colour
// all possible at once, and each of them separately is easier this way than
// fighting the widget for it.
//
// ImGui-free ON PURPOSE, like map_geometry and lights: every rule below is a
// pure function of (buffer, cursor, key) and is tested as one in
// tests/test_editor.cpp. The drawing and the key capture live in the UI layer.
//
// ---------------------------------------------------------------------------
// THE VIM SUBSET, stated honestly.
//
// This is not vim. It is the set of bindings that carry most of the day-to-day
// use, and nothing beyond it. What IS here:
//
//   modes     normal, insert, visual, visual-line, command (`:`)
//   motions   h j k l, w b e, 0 ^ $, gg G, {count} prefix on all of them
//   enter     i a I A o O
//   delete    x X, dd dw d$ D, {count}dd
//   change    cc cw C, s
//   yank/put  yy yw, p P
//   misc      r<char>, J, u, Ctrl-R, ZZ
//   visual    v V then d y c x
//   command   :w :q :wq :q! :{number}
//
// What is NOT here, and would be missed: registers beyond the unnamed one,
// marks, macros, `.` repeat, search (/ ? n N), text objects (ciw, di"), and
// counts on operators in the {op}{count}{motion} order (3dw). If one of those
// starts costing real time, add it here with a test rather than approximating
// it in the UI layer.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hpp"

namespace ed {

enum class Mode
{
    MODE_NORMAL = 0,
    MODE_INSERT,
    MODE_VISUAL,
    MODE_VISUAL_LINE,
    MODE_COMMAND,
};

// A key press, already decoded by the UI layer. `ch` is the printable character
// (0 if the press has none); `sp` names the non-printable ones. Both may be set
// for keys like Tab where either reading is useful.
enum class Special
{
    SPECIAL_NONE = 0,
    SPECIAL_ESC,
    SPECIAL_ENTER,
    SPECIAL_BACKSPACE,
    SPECIAL_TAB,
    SPECIAL_DELETE,
    SPECIAL_LEFT,
    SPECIAL_RIGHT,
    SPECIAL_UP,
    SPECIAL_DOWN,
    SPECIAL_HOME,
    SPECIAL_END,
    SPECIAL_PAGE_UP,
    SPECIAL_PAGE_DOWN,
};

struct Key
{
    Char    ch   = 0;
    Special sp   = Special::SPECIAL_NONE;
    Bool    ctrl = false;
    Bool    shift = false;
};

// Where the caret is. `col` is a byte offset into the line and may equal the
// line's length (one past the last character), which is where insert mode puts
// it at the end of a line.
struct Cursor
{
    Int32 line = 0;
    Int32 col  = 0;
};

// How many spaces one indent level is. Four, matching the C++ in this repo and
// the firmware beside it. Tab inserts spaces rather than a tab character for the
// same reason the rest of the tree has none: a file that mixes them reflows
// differently in every other tool that opens it.
inline constexpr Int32 INDENT = 4;

class Editor
{
public:
    Editor();

    // ---- content ---------------------------------------------------------
    Void       setText(const Str& text);   // resets cursor, undo and dirty flag
    [[nodiscard]] Str text() const;

    [[nodiscard]] Int32      lineCount() const;
    [[nodiscard]] const Str& line(Int32 i) const;

    [[nodiscard]] Cursor cursor() const
    {
        return cur;
    }
    Void                 setCursor(Int32 line, Int32 col);

    // True when the buffer has changed since the last setText() or clearDirty().
    [[nodiscard]] Bool dirty() const
    {
        return dirtyFlag;
    }
    Void               clearDirty()
    {
        dirtyFlag = false;
    }

    // ---- editing ---------------------------------------------------------
    // One key press. Returns true if anything about the buffer or the cursor
    // changed, which the UI uses to decide whether to re-tokenize and scroll.
    Bool key(const Key& k);

    // Text typed as a unit - a paste, or a character stream from the platform's
    // input queue. Goes through the same insert path as a key, so auto-indent
    // and brace matching behave identically.
    Void insertText(const Str& s);

    // ---- completion support ----------------------------------------------
    // The editor does not own the popup - that is the view's business - but it
    // does own the buffer, so the two operations that touch text live here and
    // are tested here.

    // The identifier the caret is part-way through typing. Empty if the caret
    // is not at the end of one.
    [[nodiscard]] Str wordBeforeCursor() const;

    // Replaces that identifier with `s`. A no-op when there is none, so an
    // accepted completion with nothing to replace cannot corrupt the line.
    Void replaceWordBeforeCursor(const Str& s);

    // ---- mode ------------------------------------------------------------
    [[nodiscard]] Mode mode() const
    {
        return md;
    }
    Void               setMode(Mode m);

    // The `:` line being typed, without the colon. Empty outside command mode.
    [[nodiscard]] const Str& commandLine() const
    {
        return cmdLine;
    }

    // A command mode line that has been submitted with Enter, consumed by the
    // caller. Empty when there is nothing pending. This is how :w reaches the
    // build/save machinery without the editor knowing what a file is.
    [[nodiscard]] Str takeSubmittedCommand();

    // Anything the editor wants shown on the status line - "3 lines yanked",
    // "already at oldest change". Cleared when read.
    [[nodiscard]] Str takeMessage();

    // ---- the unnamed register --------------------------------------------
    // Exposed so the host can bridge it to the system clipboard. This class
    // includes no OS header at all and is not going to start; that is what
    // makes it testable without a window.
    //
    // vim keeps its registers to itself and wants "+y to reach the desktop.
    // That is the wrong default here: this is one small editor inside a larger
    // app, and a yank you cannot paste into a browser is a yank that did not
    // work as far as anybody using it is concerned.
    [[nodiscard]] const Str& yankText() const
    {
        return yankBuf;
    }

    [[nodiscard]] Bool yankIsLinewise() const
    {
        return yankLinewise;
    }

    // Load the register from outside, so p puts down what was copied in another
    // program. Text ending in a newline came off whole lines and is treated as
    // linewise, which is the same guess vim makes.
    Void setYank(const Str& text, Bool linewise);

    // ---- selection -------------------------------------------------------
    // Ordered [start, end] inclusive of start, exclusive of end, in visual mode.
    // Returns false when nothing is selected.
    [[nodiscard]] Bool selection(Cursor& from, Cursor& to) const;

    // ---- undo ------------------------------------------------------------
    Void undo();
    Void redo();

private:
    // A whole-buffer snapshot. Sketches are a few dozen lines, so storing the
    // text outright costs nothing measurable and removes every class of bug
    // that a diff-based undo has. If this ever edits something large, THIS is
    // the thing to replace, and the tests above it will not change.
    struct Snapshot
    {
        Vec<Str> lines;
        Cursor           cur;
    };

    Void pushUndo();
    Void applyNormalKey(const Key& k);
    Void applyInsertKey(const Key& k);
    Void applyVisualKey(const Key& k);
    Void applyCommandKey(const Key& k);

    // Motion resolution shared by normal and visual mode. Returns false if `c`
    // is not a motion, leaving `out` untouched.
    [[nodiscard]] Bool motion(Char c, Int32 count, Cursor& out, Bool& linewise);

    Void deleteRange(Cursor a, Cursor b, Bool linewise, Bool yank);
    Void yankRange(Cursor a, Cursor b, Bool linewise);
    Void put(Bool before);

    Void insertChar(Char c);
    Void newlineWithIndent();
    Void backspace();

    Void clampCursor();
    [[nodiscard]] Int32 indentOf(Int32 lineIdx) const;

    Vec<Str>      lines;
    Cursor                cur;
    Mode                  md = Mode::MODE_NORMAL;

    // Normal-mode parse state: a pending count and a pending operator, both
    // cleared whenever a command completes or is abandoned.
    Int32 pendCount = 0;
    Char  pendOp    = 0;

    Cursor  visAnchor;
    Str     cmdLine;
    Str     submitted;
    Str     message;

    // The unnamed register. `yankLinewise` decides whether p opens a new line
    // or splices into the current one, which is the whole difference between
    // yy/p and yw/p.
    Str  yankBuf;
    Bool yankLinewise = false;

    Vec<Snapshot> undoStack;
    Vec<Snapshot> redoStack;
    Bool                  dirtyFlag = false;
};

} // namespace ed
