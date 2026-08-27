// The code editor's buffer, vim bindings, auto-indent and highlighter.
//
//   tests\build_editor_test.bat run
//
// These exist because a modal editor is almost entirely edge cases, and every
// one of them is the kind that feels "slightly off" rather than broken:
//
//   * The caret's legal range DIFFERS BY MODE. Normal mode sits on a character
//     (last valid column is len-1); insert mode sits between them (len is
//     valid). Use one rule for both and the end of every line misbehaves.
//   * cw is not dw. vim's own special case: cw stops at the end of the word
//     instead of eating the space after it. Nobody can say why it bothers them,
//     but everybody notices.
//   * Auto-closing must SKIP an existing closer rather than add a second one,
//     or every pair costs a keystroke to escape and the feature is a net loss.
//   * A pasted newline must not re-indent. The text carries its own
//     indentation; doing both doubles it.
//
// No hardware, no window. Exits 0 on PASS, 1 on FAIL.

#include "shared.hpp"
#include "../src/complete.hpp"
#include "../src/editor.hpp"
#include "../src/syntax.hpp"

#include <cstdio>
#include <cstring>

namespace {

Int32 failures = 0;
Int32 checks   = 0;

Void check(Bool ok, const Char* what)
{
    ++checks;
    if(!ok)
    {
        ++failures;
        std::printf("  FAIL  %s\n", what);;
    }
    else
    {
        std::printf("  ok    %s\n", what);;
    }
}

Void checkStr(const Str& got, const Str& want, const Char* what)
{
    ++checks;
    if(got != want)
    {
        ++failures;
        std::printf("  FAIL  %s\n         got  \"%s\"\n         want \"%s\"\n",
                    what, got.c_str(), want.c_str());
    }
    else
    {
        std::printf("  ok    %s\n", what);
    }
}

// Feeds a string of printable keys, one at a time, exactly as typing would.
Void type(ed::Editor& e, const Char* s)
{
    for(const Char* p = s; *p; ++p)
    {
        ed::Key k;
        k.ch = *p;
        e.key(k);
    }
}

Void special(ed::Editor& e, ed::Special sp)
{
    ed::Key k;
    k.sp = sp;
    e.key(k);
}

// ---------------------------------------------------------------------------

Void testModesAndMotions()
{
    std::printf("\n-- modes and motions --\n");

    ed::Editor e;
    e.setText("hello world\nsecond line\nthird");

    check(e.mode() == ed::Mode::MODE_NORMAL, "starts in normal mode");
    check(e.lineCount() == 3, "three lines parsed");

    type(e, "l");
    check(e.cursor().col == 1, "l moves right");
    type(e, "j");
    check(e.cursor().line == 1, "j moves down");
    type(e, "0");
    check(e.cursor().col == 0, "0 goes to column zero");
    type(e, "$");
    check(e.cursor().col == 10, "$ goes to the last character, not past it");

    type(e, "gg");
    check(e.cursor().line == 0, "gg goes to the first line");
    type(e, "G");
    check(e.cursor().line == 2, "G goes to the last line");

    e.setCursor(0, 0);
    type(e, "w");
    check(e.cursor().col == 6, "w skips to the next word");
    type(e, "b");
    check(e.cursor().col == 0, "b goes back a word");

    // Counts.
    e.setCursor(0, 0);
    type(e, "3l");
    check(e.cursor().col == 3, "3l moves three right");

    e.setCursor(0, 0);
    type(e, "2j");
    check(e.cursor().line == 2, "2j moves two down");

    // The mode-dependent column limit - the bug this test exists for.
    e.setText("ab");
    type(e, "$");
    check(e.cursor().col == 1, "normal mode caret stops at len-1");
    type(e, "a");
    check(e.mode() == ed::Mode::MODE_INSERT, "a enters insert mode");
    check(e.cursor().col == 2, "insert mode allows the column past the end");
    special(e, ed::Special::SPECIAL_ESC);
    check(e.cursor().col == 1, "escape pulls the caret back one, as vim does");
}

Void testEditing()
{
    std::printf("\n-- editing --\n");

    ed::Editor e;
    e.setText("one\ntwo\nthree");

    type(e, "dd");
    checkStr(e.text(), "two\nthree", "dd deletes the line");

    type(e, "p");
    checkStr(e.text(), "two\none\nthree", "p puts the yanked line back after");

    e.setText("alpha beta");
    e.setCursor(0, 0);
    type(e, "x");
    checkStr(e.text(), "lpha beta", "x deletes a character");

    e.setText("alpha beta");
    e.setCursor(0, 0);
    type(e, "3x");
    checkStr(e.text(), "ha beta", "3x deletes three characters");

    e.setText("alpha beta");
    e.setCursor(0, 0);
    type(e, "dw");
    checkStr(e.text(), "beta", "dw deletes the word and the space after it");

    // cw stops at the end of the word - vim's special case.
    e.setText("alpha beta");
    e.setCursor(0, 0);
    type(e, "cw");
    check(e.mode() == ed::Mode::MODE_INSERT, "cw enters insert mode");
    checkStr(e.text(), " beta", "cw keeps the space that dw eats");

    e.setText("hello");
    e.setCursor(0, 0);
    type(e, "D");
    checkStr(e.text(), "", "D deletes to the end of the line");

    e.setText("abc");
    e.setCursor(0, 1);
    type(e, "rZ");
    checkStr(e.text(), "aZc", "r replaces one character");

    e.setText("one\ntwo");
    e.setCursor(0, 0);
    type(e, "J");
    checkStr(e.text(), "one two", "J joins the next line with a space");

    // Undo and redo.
    e.setText("keep");
    type(e, "x");
    checkStr(e.text(), "eep", "x applied");
    type(e, "u");
    checkStr(e.text(), "keep", "u undoes it");
    {
        ed::Key k;
        k.ch   = 'r';
        k.ctrl = true;
        e.key(k);
    }
    checkStr(e.text(), "eep", "ctrl-r redoes it");

    // o and O.
    e.setText("first");
    e.setCursor(0, 0);
    type(e, "o");
    check(e.mode() == ed::Mode::MODE_INSERT, "o enters insert mode");
    type(e, "x");
    checkStr(e.text(), "first\nx", "o opens a line below");

    e.setText("first");
    e.setCursor(0, 0);
    type(e, "O");
    type(e, "x");
    checkStr(e.text(), "x\nfirst", "O opens a line above");
}

Void testVisual()
{
    std::printf("\n-- visual mode --\n");

    ed::Editor e;
    e.setText("abcdef");
    e.setCursor(0, 0);
    type(e, "vll");
    check(e.mode() == ed::Mode::MODE_VISUAL, "v enters visual mode");

    ed::Cursor a, b;
    check(e.selection(a, b), "there is a selection");
    check(a.col == 0 && b.col == 3, "visual selection is inclusive of the caret");

    type(e, "d");
    checkStr(e.text(), "def", "visual d deletes the selection");
    check(e.mode() == ed::Mode::MODE_NORMAL, "and returns to normal mode");

    e.setText("one\ntwo\nthree");
    e.setCursor(0, 0);
    type(e, "Vj");
    check(e.mode() == ed::Mode::MODE_VISUAL_LINE, "V enters visual-line mode");
    type(e, "d");
    checkStr(e.text(), "three", "visual-line d deletes whole lines");
}

Void testAutoClose()
{
    std::printf("\n-- auto-closing and indent --\n");

    ed::Editor e;
    e.setText("");
    type(e, "i");
    type(e, "(");
    checkStr(e.text(), "()", "( inserts a matched pair");
    check(e.cursor().col == 1, "and leaves the caret between them");

    // Typing the closer must skip over, not double it.
    type(e, ")");
    checkStr(e.text(), "()", ") skips over the existing closer");
    check(e.cursor().col == 2, "caret is past the pair");

    e.setText("");
    type(e, "i");
    type(e, "{");
    checkStr(e.text(), "{}", "{ inserts a matched pair");
    special(e, ed::Special::SPECIAL_BACKSPACE);
    checkStr(e.text(), "", "backspace removes both halves of an empty pair");

    // A bracket before existing text does NOT pair - it is a wrap.
    e.setText("abc");
    e.setCursor(0, 0);
    type(e, "i");
    type(e, "(");
    checkStr(e.text(), "(abc", "( before a word does not inject a closer");

    // A quote after a word character is an apostrophe, not a new pair.
    e.setText("");
    type(e, "i");
    type(e, "it");
    type(e, "'");
    checkStr(e.text(), "it'", "a quote after a letter does not pair");

    // Enter after { indents; between { and } opens the block out.
    e.setText("");
    type(e, "i");
    type(e, "if (x) {");
    special(e, ed::Special::SPECIAL_ENTER);
    checkStr(e.text(), "if (x) {\n    \n}", "enter between braces opens the block");
    check(e.cursor().line == 1 && e.cursor().col == 4, "caret lands on the indented line");

    // Tab inserts spaces to the next stop, never a tab character.
    e.setText("");
    type(e, "i");
    special(e, ed::Special::SPECIAL_TAB);
    checkStr(e.text(), "    ", "tab inserts four spaces");
    check(e.text().find('\t') == Str::npos, "and no tab character");

    // Backspace in leading whitespace removes a whole level.
    special(e, ed::Special::SPECIAL_BACKSPACE);
    checkStr(e.text(), "", "backspace removes a whole indent level");

    // Auto-indent carries the previous line's indentation.
    e.setText("");
    type(e, "i");
    type(e, "    body");
    special(e, ed::Special::SPECIAL_ENTER);
    type(e, "x");
    checkStr(e.text(), "    body\n    x", "enter carries the indent");

    // A pasted newline must not re-indent.
    e.setText("");
    e.insertText("    a\n    b");
    checkStr(e.text(), "    a\n    b", "pasted text keeps its own indentation");
}

Void testCommandLine()
{
    std::printf("\n-- command line --\n");

    ed::Editor e;
    e.setText("a\nb\nc");
    type(e, ":");
    check(e.mode() == ed::Mode::MODE_COMMAND, ": enters command mode");
    type(e, "w");
    checkStr(e.commandLine(), "w", "the command line accumulates");
    special(e, ed::Special::SPECIAL_ENTER);
    checkStr(e.takeSubmittedCommand(), "w", ":w is handed to the caller");
    check(e.mode() == ed::Mode::MODE_NORMAL, "and we are back in normal mode");
    checkStr(e.takeSubmittedCommand(), "", "a submitted command is consumed once");

    // :N is the editor's own business and must not reach the caller.
    type(e, ":3");
    special(e, ed::Special::SPECIAL_ENTER);
    check(e.cursor().line == 2, ":3 jumps to line 3");
    checkStr(e.takeSubmittedCommand(), "", ":N is handled internally");

    // Escape abandons.
    type(e, ":q");
    special(e, ed::Special::SPECIAL_ESC);
    checkStr(e.takeSubmittedCommand(), "", "escape abandons the command");
}

Void testDirty()
{
    std::printf("\n-- dirty tracking --\n");

    ed::Editor e;
    e.setText("clean");
    check(!e.dirty(), "setText leaves the buffer clean");
    type(e, "x");
    check(e.dirty(), "an edit marks it dirty");
    e.clearDirty();
    check(!e.dirty(), "clearDirty clears it");
    type(e, "l");
    check(!e.dirty(), "a bare motion does not dirty the buffer");
}

Void testSyntax()
{
    std::printf("\n-- highlighter --\n");

    Vec<syn::Span> spans;
    Bool inBlock = false;

    // A role lookup helper: what colour does byte `at` get?
    auto roleAt = [&](Int32 at) -> syn::Role
    {
        for(const syn::Span& s : spans)
            if(at >= s.begin && at < s.end)
                return s.role;
        return syn::Role::ROLE_COUNT;
    };

    syn::tokenize("int x = 42;", inBlock, spans);
    check(roleAt(0) == syn::Role::ROLE_TYPE,    "int is a type");
    check(roleAt(8) == syn::Role::ROLE_NUMBER,  "42 is a number");
    check(roleAt(10) == syn::Role::ROLE_PUNCT,  "; is punctuation");

    syn::tokenize("if (a) return;", inBlock, spans);
    check(roleAt(0) == syn::Role::ROLE_KEYWORD, "if is a keyword");
    check(roleAt(7) == syn::Role::ROLE_KEYWORD, "return is a keyword");

    syn::tokenize("gpio_put(LED, 1);", inBlock, spans);
    check(roleAt(0) == syn::Role::ROLE_FUNCTION, "an identifier before ( is a call");

    syn::tokenize("x = \"hi\";", inBlock, spans);
    check(roleAt(4) == syn::Role::ROLE_STRING, "a string literal is a string");

    syn::tokenize("a; // trailing", inBlock, spans);
    check(roleAt(3) == syn::Role::ROLE_COMMENT, "// starts a comment");
    check(!inBlock, "a line comment does not open a block");

    // The include line keeps its header in string colour.
    syn::tokenize("#include \"pico/stdlib.h\"", inBlock, spans);
    check(roleAt(1) == syn::Role::ROLE_PREPROC, "#include is preprocessor");
    check(roleAt(10) == syn::Role::ROLE_STRING, "the header path is a string");

    // A block comment carries across lines - the one bit of state there is.
    inBlock = false;
    syn::tokenize("/* opens here", inBlock, spans);
    check(inBlock, "an unterminated /* sets the carry");
    syn::tokenize("still inside", inBlock, spans);
    check(roleAt(0) == syn::Role::ROLE_COMMENT, "the next line is all comment");
    check(inBlock, "and the carry survives");
    syn::tokenize("done */ int", inBlock, spans);
    check(!inBlock, "*/ clears the carry");
    check(roleAt(8) == syn::Role::ROLE_TYPE, "code after */ is highlighted again");

    // A # inside a block comment is not a preprocessor line.
    inBlock = true;
    syn::tokenize("#define X 1", inBlock, spans);
    check(roleAt(0) == syn::Role::ROLE_COMMENT, "# inside /* */ stays a comment");
    inBlock = false;

    // Spans must tile the line with no gaps, which the renderer relies on.
    syn::tokenize("int f(void) { return 0; }", inBlock, spans);
    Bool contiguous = true;
    Int32 expect = 0;
    for(const syn::Span& s : spans)
    {
        if(s.begin != expect)
        {
            contiguous = false;
            break;
        }
        expect = s.end;
    }
    check(contiguous && expect == 25, "spans tile the whole line with no gaps");
}

Void testCompletion()
{
    std::printf("\n-- completion --\n");

    // The word under the caret is what gets completed, and a number is not one.
    ed::Editor e;
    e.setText("");
    type(e, "i");
    type(e, "gpio");
    checkStr(e.wordBeforeCursor(), "gpio", "the partial identifier is found");

    e.setText("");
    type(e, "i");
    type(e, "x = 42");
    checkStr(e.wordBeforeCursor(), "", "a number is not an identifier");

    e.setText("");
    type(e, "i");
    type(e, "servoWriteUs");
    type(e, "(");
    checkStr(e.wordBeforeCursor(), "", "after a bracket there is no partial word");

    // Accepting a completion replaces only the partial word.
    e.setText("");
    type(e, "i");
    type(e, "    gpioW");
    e.replaceWordBeforeCursor("gpioWrite");
    checkStr(e.text(), "    gpioWrite", "the partial word is replaced in place");
    check(e.cursor().col == 13, "and the caret lands after it");

    // With nothing to replace it must do nothing rather than corrupt the line.
    e.setText("");
    type(e, "i");
    type(e, "x = ");
    e.replaceWordBeforeCursor("gpioWrite");
    checkStr(e.text(), "x = ", "replacing with no partial word is a no-op");

    // ---- the table and the ranking ----------------------------------------
    Vec<const cmpl::Item*> hits;

    hits.clear();
    cmpl::suggest("gpio", hits, 8);
    check(!hits.empty(), "gpio matches something");
    checkStr(hits[0]->name, "gpioOpen",
             "shorter names rank first: gpioOpen before gpioToggle");

    hits.clear();
    cmpl::suggest("servo", hits, 8);
    Bool sawWriteUs = false;
    for(const cmpl::Item* it : hits)
        if(std::strcmp(it->name, "servoWriteUs") == 0) sawWriteUs = true;
    check(sawWriteUs, "servo offers servoWriteUs");

    // Matching folds case, but ranking does NOT: `Pin` and `PIN` are different
    // things in this codebase (a type and a set of enum constants) and each must
    // offer its own first.
    hits.clear();
    cmpl::suggest("Pin", hits, 8);
    checkStr(hits[0]->name, "Pin", "Pin offers the type first");

    hits.clear();
    cmpl::suggest("PIN", hits, 8);
    checkStr(hits[0]->name, "PIN_DIR_IN", "PIN offers the constants first");

    hits.clear();
    cmpl::suggest("int", hits, 8);
    check(!hits.empty(), "matching folds case: int still finds Int32");

    // A prefix that matches nothing yields nothing, and the popup closes on it.
    hits.clear();
    check(cmpl::suggest("zzzz", hits, 8) == 0, "an unknown prefix matches nothing");

    // An empty prefix must NOT return the whole table - that would open the
    // popup on every keystroke.
    hits.clear();
    check(cmpl::suggest("", hits, 8) == 0, "an empty prefix matches nothing");

    // max is honoured.
    hits.clear();
    check(cmpl::suggest("s", hits, 3) <= 3, "max limits the result count");

    // Every entry must be well formed - this is the table's only guard against
    // a typo introduced while keeping it in step with pico2w.h.
    Bool wellFormed = true;
    for(const cmpl::Item& it : cmpl::all())
    {
        if(it.name == nullptr || it.name[0] == 0)
        {
            wellFormed = false;
            break;
        }
        if(it.detail == nullptr)
        {
            wellFormed = false;
            break;
        }
        if(it.doc == nullptr)
        {
            wellFormed = false;
            break;
        }
    }
    check(wellFormed, "every table entry has a name, a detail and a doc");

    // No duplicate names: two entries with one name means one is unreachable.
    Bool unique = true;
    const Vec<cmpl::Item>& items = cmpl::all();
    for(Size i = 0; i < items.size() && unique; ++i)
        for(Size j = i + 1; j < items.size(); ++j)
            if(std::strcmp(items[i].name, items[j].name) == 0)
            {
                unique = false;
                break;
            }
    check(unique, "no duplicate entries in the table");

    // wordAtEnd agrees with the editor's own idea of a partial word.
    checkStr(cmpl::wordAtEnd("    gpioW"), "gpioW", "wordAtEnd finds the trailing word");
    checkStr(cmpl::wordAtEnd("x = 42"),    "",      "wordAtEnd rejects a number");
    checkStr(cmpl::wordAtEnd("foo("),      "",      "wordAtEnd stops at punctuation");
}

} // namespace

// The register is what the host copies to the system clipboard, so what lands
// in it matters beyond this file now. gg+yG in particular: it reported "N lines
// yanked" and nothing reached the clipboard, because nothing was reading the
// register at all.
Void testYankRegister()
{
    std::printf("\n-- yank register --\n");

    {
        ed::Editor e;
        e.setText("alpha\nbeta\ngamma");
        e.setCursor(2, 3);

        // The whole file, from wherever the caret happens to be.
        type(e, "gg");
        type(e, "yG");

        checkStr(e.yankText(), "alpha\nbeta\ngamma\n",
                 "gg yG yanks every line into the register");
        check(e.yankIsLinewise(), "gg yG is linewise");
        check(e.text() == "alpha\nbeta\ngamma", "yanking changes nothing");
    }

    {
        ed::Editor e;
        e.setText("alpha\nbeta");
        e.setCursor(0, 0);
        type(e, "yy");
        checkStr(e.yankText(), "alpha\n", "yy yanks the line with its newline");
        check(e.yankIsLinewise(), "yy is linewise");
    }

    {
        ed::Editor e;
        e.setText("hello world");
        e.setCursor(0, 0);
        type(e, "yw");
        check(!e.yankIsLinewise(), "yw is characterwise");
        check(!e.yankText().empty(), "yw fills the register");
    }

    // x and D write the register too, which is why the host watches the buffer
    // rather than counting calls to yankRange().
    {
        ed::Editor e;
        e.setText("abcdef");
        e.setCursor(0, 0);
        type(e, "3x");
        checkStr(e.yankText(), "abc", "x fills the register");
        check(!e.yankIsLinewise(), "x is characterwise");
    }

    {
        ed::Editor e;
        e.setText("abcdef");
        e.setCursor(0, 2);
        type(e, "D");
        checkStr(e.yankText(), "cdef", "D fills the register");
    }

    // dd, so the delete half of an operator is covered as well.
    {
        ed::Editor e;
        e.setText("alpha\nbeta");
        e.setCursor(0, 0);
        type(e, "dd");
        checkStr(e.yankText(), "alpha\n", "dd fills the register");
        checkStr(e.text(), "beta", "dd removes the line");
    }

    // The other direction: text arriving from the system clipboard.
    {
        ed::Editor e;
        e.setText("alpha");
        e.setCursor(0, 0);

        e.setYank("from elsewhere\n", true);
        checkStr(e.yankText(), "from elsewhere\n", "setYank loads the register");
        check(e.yankIsLinewise(), "setYank keeps linewise");

        type(e, "p");
        checkStr(e.text(), "alpha\nfrom elsewhere",
                 "p puts linewise clipboard text on its own line");
    }

    {
        ed::Editor e;
        e.setText("ac");
        e.setCursor(0, 0);
        e.setYank("b", false);
        type(e, "p");
        checkStr(e.text(), "abc", "p splices characterwise clipboard text");
    }
}

int main()
{
    std::printf("editor + syntax tests\n");

    testModesAndMotions();
    testEditing();
    testVisual();
    testYankRegister();
    testAutoClose();
    testCommandLine();
    testDirty();
    testCompletion();
    testSyntax();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
