// Live style checking in the editor.
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS ALONGSIDE diagnostics.hxx
//
// diagnostics.hxx parses COMPILER output. It is exact, it is authoritative, and
// it only exists after a build - so the editor was blank until you pressed Run,
// and a naming mistake survived until the style audit caught it at commit time.
//
// This runs on the buffer as you type. It cannot tell you about a type error;
// it can tell you that `int count` should be `Int32 count` at the moment you
// write it, which is when that is cheapest to fix.
//
// ---------------------------------------------------------------------------
// THE RULES ARE tools/style_audit.py's RULES
//
// Deliberately the same set, deliberately the same wording. A linter that
// disagrees with the gate is worse than no linter: it either passes things the
// commit will reject, or flags things the project has decided are fine, and
// either way people learn to ignore it.
//
// Where the audit's regex needs a real parser to be exact, this errs toward
// SILENCE. A false positive in an editor is an underline sitting under correct
// code all day, and that is how a squiggle stops being read.
//
// ---------------------------------------------------------------------------
// C AND C++ ARE NOT THE SAME HERE
//
// The firmware is C. It cannot use named casts, `Str`, or the shared.hxx
// aliases - it has shared.h and its own vocabulary - so those rules are skipped
// for a .c or .h file. Applying C++ rules to C is how a linter ends up
// underlining an entire file that is exactly as it should be.
#pragma once

#include "shared.hxx"

#include "diagnostics.hxx"

namespace lint
{

  // Which language the rules should be read as. Chosen from the file extension by
  // the caller, because the buffer alone cannot tell.
  enum class Lang
  {
      LANG_CPP = 0,
      LANG_C
  };

  [[nodiscard]] Lang langOf(const Str& path);

  // Every violation in `text`, as diagnostics the gutter already knows how to
  // draw. Line and column are 1-based, matching the compiler's convention and
  // diagnostics.hxx's.
  //
  // Pure: no file access, no globals, no ImGui. Tested in tests/test_lint.cxx.
  [[nodiscard]] Vec<diag::Item> check(const Str& text, Lang lang);

}
