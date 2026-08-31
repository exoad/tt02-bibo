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

#include "shared.hxx"
#include "../src/complete.hxx"
#include "../src/editor.hxx"
#include "../src/syntax.hxx"

#include <cstdio>
#include <cstring>

namespace
{

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
          if(std::strcmp(it->name, "servoWriteUs") == 0)
          {
              sawWriteUs = true;
          }
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

// Feeds a special key, for the sequences that need Escape or Enter.
Void esc(ed::Editor& e)
{
    ed::Key k;
    k.sp = ed::Special::SPECIAL_ESC;
    e.key(k);
}

Void enter(ed::Editor& e)
{
    ed::Key k;
    k.sp = ed::Special::SPECIAL_ENTER;
    e.key(k);
}

Void testSearch()
{
    std::printf("\n-- search --\n");

    ed::Editor e;
    e.setText("alpha beta\ngamma beta\ndelta");
    e.setCursor(0, 0);

    type(e, "/beta");
    check(e.mode() == ed::Mode::MODE_COMMAND, "/ opens the command line");
    check(e.commandPrefix() == '/', "and it knows it is a search, not a colon");
    checkStr(e.commandLine(), "beta", "the pattern is what was typed");

    enter(e);
    check(e.mode() == ed::Mode::MODE_NORMAL, "enter leaves command mode");
    check(e.cursor().line == 0 && e.cursor().col == 6, "/ lands on the first match");

    type(e, "n");
    check(e.cursor().line == 1 && e.cursor().col == 6, "n goes to the next");

    // Wraps, as vim does.
    type(e, "n");
    check(e.cursor().line == 0 && e.cursor().col == 6, "n wraps to the top");

    type(e, "N");
    check(e.cursor().line == 1 && e.cursor().col == 6, "N goes back");

    // A search that finds nothing must not move the caret.
    {
        ed::Editor f;
        f.setText("one\ntwo");
        f.setCursor(1, 1);
        type(f, "/zzz");
        enter(f);
        check(f.cursor().line == 1 && f.cursor().col == 1,
              "a search with no match leaves the caret alone");
        checkStr(f.takeMessage(), "pattern not found: zzz",
                 "and says so on the status line");
    }

    // Smartcase: lowercase is insensitive, any capital makes it exact.
    {
        ed::Editor f;
        f.setText("xx Gpio");
        f.setCursor(0, 0);
        type(f, "/gpio");
        enter(f);
        check(f.cursor().col == 3, "an all-lowercase pattern ignores case");

        // The capitalised pattern must SKIP the lowercase one at column 3.
        ed::Editor g;
        g.setText("xx gpio yy Gpio");
        g.setCursor(0, 0);
        type(g, "/Gpio");
        enter(g);
        check(g.cursor().col == 11, "a pattern with a capital is exact");
    }

    // Backwards.
    {
        ed::Editor f;
        f.setText("hit\nmiss\nhit");
        f.setCursor(1, 0);
        type(f, "?hit");
        enter(f);
        check(f.cursor().line == 0, "? searches backwards");
    }

    // * takes the word under the caret.
    {
        ed::Editor f;
        f.setText("gpioWrite(1);\nsleepMs(2);\ngpioWrite(0);");
        f.setCursor(0, 2);
        type(f, "*");
        check(f.cursor().line == 2, "* finds the next use of the word under the caret");
        checkStr(f.searchPattern(), "gpioWrite", "and remembers what it searched for");
    }

    // A search is the editor's business and must never reach the caller.
    {
        ed::Editor f;
        f.setText("abc");
        type(f, "/ab");
        enter(f);
        checkStr(f.takeSubmittedCommand(), "",
                 "a search does not submit a command to the host");
    }
}

Void testFindInLine()
{
    std::printf("\n-- f F t T ; , --\n");

    ed::Editor e;
    e.setText("alpha,beta,gamma");
    e.setCursor(0, 0);

    type(e, "f,");
    check(e.cursor().col == 5, "f lands ON the character");

    type(e, ";");
    check(e.cursor().col == 10, "; repeats it");

    type(e, ",");
    check(e.cursor().col == 5, ", repeats it backwards");

    e.setCursor(0, 0);
    type(e, "t,");
    check(e.cursor().col == 4, "t stops one short");

    e.setCursor(0, 15);
    type(e, "F,");
    check(e.cursor().col == 10, "F searches backwards");

    e.setCursor(0, 0);
    type(e, "2f,");
    check(e.cursor().col == 10, "a count takes the second match");

    // As an operator target, f is inclusive.
    {
        ed::Editor f;
        f.setText("alpha,beta");
        f.setCursor(0, 0);
        type(f, "df,");
        checkStr(f.text(), "beta", "df deletes through the character");
    }

    // A character that is not there changes nothing.
    {
        ed::Editor f;
        f.setText("abc");
        f.setCursor(0, 0);
        type(f, "fz");
        check(f.cursor().col == 0, "f with no match does not move");
    }
}

Void testTextObjects()
{
    std::printf("\n-- text objects --\n");

    {
        ed::Editor e;
        e.setText("one two three");
        e.setCursor(0, 5);            // on "two"
        type(e, "diw");
        checkStr(e.text(), "one  three", "diw takes the word and leaves the spaces");
    }

    {
        ed::Editor e;
        e.setText("one two three");
        e.setCursor(0, 5);
        type(e, "daw");
        checkStr(e.text(), "one three", "daw takes the trailing space too");
    }

    {
        ed::Editor e;
        e.setText("gpioWrite(LED_PIN, true);");
        e.setCursor(0, 14);           // inside the parentheses
        type(e, "di(");
        checkStr(e.text(), "gpioWrite();", "di( empties the parentheses");
    }

    {
        ed::Editor e;
        e.setText("gpioWrite(LED_PIN);");
        e.setCursor(0, 12);
        type(e, "da(");
        checkStr(e.text(), "gpioWrite;", "da( takes the brackets as well");
    }

    {
        ed::Editor e;
        e.setText("serialPrint(\"hello world\");");
        e.setCursor(0, 15);
        type(e, "ci\"");
        checkStr(e.text(), "serialPrint(\"\");", "ci\" empties the string");
        check(e.mode() == ed::Mode::MODE_INSERT, "and leaves you in insert mode");
    }

    // Across lines, which is the case that makes ci{ worth having.
    {
        ed::Editor e;
        e.setText("void f()\n{\n    body();\n}\ntail");
        e.setCursor(2, 4);
        type(e, "di{");
        checkStr(e.text(), "void f()\n{}\ntail", "di{ works across lines");
    }

    // yi( must not change the buffer.
    {
        ed::Editor e;
        e.setText("f(abc)");
        e.setCursor(0, 3);
        type(e, "yi(");
        checkStr(e.text(), "f(abc)", "yi( changes nothing");
        checkStr(e.yankText(), "abc", "and fills the register");
    }

    // An object with no enclosing pair is a no-op, not a crash or a wipe.
    {
        ed::Editor e;
        e.setText("no brackets here");
        e.setCursor(0, 4);
        type(e, "di(");
        checkStr(e.text(), "no brackets here", "di( with no pair does nothing");
    }
}

Void testDotRepeat()
{
    std::printf("\n-- . repeat --\n");

    {
        ed::Editor e;
        e.setText("aaa\naaa\naaa");
        e.setCursor(0, 0);
        type(e, "x");
        checkStr(e.text(), "aa\naaa\naaa", "x deletes one");

        type(e, "j0");
        type(e, ".");
        checkStr(e.text(), "aa\naa\naaa", ". repeats it");

        type(e, "j0");
        type(e, ".");
        checkStr(e.text(), "aa\naa\naa", "and again");
    }

    // The whole insert is repeated, not just the last key.
    {
        ed::Editor e;
        e.setText("one two\nthree four");
        e.setCursor(0, 0);
        type(e, "ciwX");
        esc(e);
        checkStr(e.text(), "X two\nthree four", "ciw then text");

        type(e, "j0");
        type(e, ".");
        checkStr(e.text(), "X two\nX four", ". repeats the change AND its text");
    }

    // dd repeats.
    {
        ed::Editor e;
        e.setText("a\nb\nc\nd");
        e.setCursor(0, 0);
        type(e, "dd");
        type(e, ".");
        checkStr(e.text(), "c\nd", ". repeats dd");
    }
}

Void testIndentAndCase()
{
    std::printf("\n-- indent, case, marks --\n");

    {
        ed::Editor e;
        e.setText("a\nb\nc");
        e.setCursor(0, 0);
        type(e, ">>");
        checkStr(e.text(), "    a\nb\nc", ">> indents one line");

        type(e, "<<");
        checkStr(e.text(), "a\nb\nc", "<< takes it back");
    }

    {
        ed::Editor e;
        e.setText("a\nb\nc");
        e.setCursor(0, 0);
        type(e, "3>>");
        checkStr(e.text(), "    a\n    b\n    c", "a count indents that many lines");
    }

    {
        ed::Editor e;
        e.setText("x\n\ny");
        e.setCursor(0, 0);
        type(e, "3>>");
        checkStr(e.text(), "    x\n\n    y",
                 "a blank line is left blank rather than filled with spaces");
    }

    {
        ed::Editor e;
        e.setText("abc");
        e.setCursor(0, 0);
        type(e, "~");
        checkStr(e.text(), "Abc", "~ flips one character and advances");
        type(e, "~");
        checkStr(e.text(), "ABc", "and again");
    }

    // Marks.
    {
        ed::Editor e;
        e.setText("one\ntwo\nthree\nfour");
        e.setCursor(1, 1);
        type(e, "ma");
        type(e, "G");
        check(e.cursor().line == 3, "G moved away");
        type(e, "'a");
        check(e.cursor().line == 1, "'a comes back");
    }

    {
        ed::Editor e;
        e.setText("hello");
        e.setCursor(0, 0);
        type(e, "'z");
        checkStr(e.takeMessage(), "mark not set", "an unset mark says so");
    }
}

Void testBracketMatch()
{
    std::printf("\n-- %% and WORD motions --\n");

    {
        ed::Editor e;
        e.setText("if(a && (b))");
        e.setCursor(0, 2);
        type(e, "%");
        check(e.cursor().col == 11, "%% finds the matching close across nesting");

        type(e, "%");
        check(e.cursor().col == 2, "and back again");
    }

    {
        ed::Editor e;
        e.setText("void f()\n{\n    x;\n}");
        e.setCursor(1, 0);
        type(e, "%");
        check(e.cursor().line == 3, "%% spans lines");
    }

    // W steps over punctuation that w stops at.
    {
        ed::Editor e;
        e.setText("a->b c");
        e.setCursor(0, 0);
        type(e, "W");
        check(e.cursor().col == 5, "W jumps the whole blob");

        e.setCursor(0, 0);
        type(e, "w");
        check(e.cursor().col < 5, "w stops inside it");
    }
}

Void testSubstitute()
{
    std::printf("\n-- :s --\n");

    {
        ed::Editor e;
        e.setText("foo bar foo");
        e.setCursor(0, 0);
        type(e, ":s/foo/baz/");
        enter(e);
        checkStr(e.text(), "baz bar foo", ":s replaces the first on the line");
    }

    {
        ed::Editor e;
        e.setText("foo bar foo");
        e.setCursor(0, 0);
        type(e, ":s/foo/baz/g");
        enter(e);
        checkStr(e.text(), "baz bar baz", "g replaces every one");
    }

    {
        ed::Editor e;
        e.setText("aa\nbb\naa");
        e.setCursor(0, 0);
        type(e, ":%s/aa/zz/g");
        enter(e);
        checkStr(e.text(), "zz\nbb\nzz", "%%s does the whole file");
    }

    // :s must not reach the host either.
    {
        ed::Editor e;
        e.setText("abc");
        type(e, ":s/a/b/");
        enter(e);
        checkStr(e.takeSubmittedCommand(), "", ":s is not submitted to the host");
    }

    // An alternative delimiter, so paths do not need escaping.
    {
        ed::Editor e;
        e.setText("/usr/lib");
        e.setCursor(0, 0);
        type(e, ":s#/usr#/opt#");
        enter(e);
        checkStr(e.text(), "/opt/lib", "any character can be the delimiter");
    }
}

Int32 main()
{
    std::printf("editor + syntax tests\n");

    testModesAndMotions();
    testEditing();
    testVisual();
    testYankRegister();
    testSearch();
    testFindInLine();
    testTextObjects();
    testDotRepeat();
    testIndentAndCase();
    testBracketMatch();
    testSubstitute();
    testAutoClose();
    testCommandLine();
    testDirty();
    testCompletion();
    testSyntax();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
