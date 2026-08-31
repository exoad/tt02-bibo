// Parsing compiler output into editor marks.
//
//   tests\build_diagnostics_test.bat run
//
// This exists because the obvious implementation - split the line on ':' - is
// wrong on Windows and wrong in a way that looks right in testing:
//
//     C:/dev/sketch.c:42:15: error: 'foo' undeclared
//
// has four colons and the FIRST one is a drive letter, not a separator. Get it
// wrong and every diagnostic points at a file called "C".
//
// The second trap is in this very tree: the compiler reports
// firmware/src/sketch.c while the editor is showing the same bytes opened from
// the sketch library. Matching on the full path would mark nothing in the one
// workflow the feature exists for.
//
// No hardware, no window. Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "diagnostics.hxx"

#include <cstdio>

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
          std::printf("  FAIL  %s\n", what);
      }
      else
      {
          std::printf("  ok    %s\n", what);
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

  Void testGcc()
  {
      std::printf("\n-- gcc / clang --\n");

      diag::Item it;

      // The Windows drive-letter case. This is the whole reason for the parser.
      check(diag::parseLine(
                "C:/dev/bibo/firmware/src/sketch.c:42:15: "
                "error: 'foo' undeclared (first use in this function)", it),
            "a path with a drive letter parses");
      checkStr(it.file, "C:/dev/bibo/firmware/src/sketch.c",
               "the drive letter stays part of the path");
      check(it.line == 42, "line is 42");
      check(it.column == 15, "column is 15");
      check(it.severity == diag::Severity::SEVERITY_ERR, "severity is error");
      checkStr(it.message, "'foo' undeclared (first use in this function)",
               "message is everything after the severity");

      check(diag::parseLine("src/sketch.c:10:5: warning: unused variable 'x' "
                            "[-Wunused-variable]", it),
            "a warning parses");
      check(it.severity == diag::Severity::SEVERITY_WARN, "severity is warning");
      check(it.line == 10 && it.column == 5, "line and column");

      check(diag::parseLine("src/sketch.c:7: error: no column here", it),
            "a diagnostic with no column parses");
      check(it.line == 7 && it.column == 0, "column is 0 when absent");

      check(diag::parseLine("src/sketch.c:99:1: note: declared here", it),
            "a note parses");
      check(it.severity == diag::Severity::SEVERITY_NOTE, "severity is note");
  }

  Void testMsvc()
  {
      std::printf("\n-- msvc --\n");

      diag::Item it;

      check(diag::parseLine(
                "C:\\hub\\src\\app_ui.cxx(294,12): error C2065: 'x': undeclared identifier",
                it),
            "msvc with line and column parses");
      check(it.line == 294 && it.column == 12, "line and column");
      check(it.severity == diag::Severity::SEVERITY_ERR, "severity is error");

      check(diag::parseLine("C:\\hub\\src\\a.cpp(17): warning C4189: unused", it),
            "msvc with line only parses");
      check(it.line == 17 && it.column == 0, "column is 0");
      check(it.severity == diag::Severity::SEVERITY_WARN, "severity is warning");
  }

  Void testRejects()
  {
      std::printf("\n-- lines that are NOT diagnostics --\n");

      diag::Item it;

      check(!diag::parseLine("[1/2] Building C object sketch.c.obj", it),
            "a ninja progress line is not a diagnostic");
      check(!diag::parseLine("[ok] firmware/build/sketch.uf2", it),
            "an ok line is not a diagnostic");
      check(!diag::parseLine("", it), "an empty line is not a diagnostic");
      check(!diag::parseLine("just some prose", it), "prose is not a diagnostic");

      // A timestamp has digits after a colon and must not be mistaken for one.
      check(!diag::parseLine("started at 10:30:15 local", it),
            "a timestamp is not a diagnostic");

      // A path and a line number but no severity word.
      check(!diag::parseLine("src/sketch.c:42:15: something happened", it),
            "no severity word means no diagnostic");
  }

  Void testFileMatching()
  {
      std::printf("\n-- matching a diagnostic to the open file --\n");

      Vec<Str> lines;
      lines.push_back("C:/repo/firmware/src/sketch.c:12:3: error: first");
      lines.push_back("[1/2] Building C object");
      lines.push_back("C:/repo/firmware/src/sketch.c:20:1: warning: second");
      lines.push_back("C:/repo/firmware/src/main.c:5:1: error: other file");

      const Vec<diag::Item> all = diag::parseAll(lines);
      check(all.size() == 3, "three diagnostics found among four lines");

      // The compiler says firmware/src/sketch.c; the editor has the same bytes
      // open from the sketch library. It must still mark them.
      const Vec<diag::Item> mine = diag::forFile(
          all, "D:\\tt02-auto\\sketches\\sketch.c");
      check(mine.size() == 2, "matched by file NAME, not by full path");

      const Vec<diag::Item> other = diag::forFile(all, "somewhere/main.c");
      check(other.size() == 1, "the other file gets its own");

      const Vec<diag::Item> none = diag::forFile(all, "nothing.c");
      check(none.empty(), "an unrelated file gets none");

      // Backslashes and forward slashes must compare equal.
      const Vec<diag::Item> mixed = diag::forFile(all, "C:/repo/firmware/src/SKETCH.C");
      check(mixed.size() == 2, "matching ignores case and slash direction");
  }

  Void testWorstOnLine()
  {
      std::printf("\n-- worst severity per line --\n");

      Vec<Str> lines;
      lines.push_back("a.c:5:1: warning: w");
      lines.push_back("a.c:5:9: error: e");
      lines.push_back("a.c:9:1: note: n");

      const Vec<diag::Item> all = diag::parseAll(lines);

      check(diag::worstOnLine(all, 5) == static_cast<Int32>(diag::Severity::SEVERITY_ERR),
            "error beats warning on the same line");
      check(diag::worstOnLine(all, 9) == static_cast<Int32>(diag::Severity::SEVERITY_NOTE),
            "a note alone reports as a note");
      check(diag::worstOnLine(all, 7) == -1, "a clean line reports -1");
  }

} // namespace

int main()
{
    std::printf("diagnostics parser tests\n");

    testGcc();
    testMsvc();
    testRejects();
    testFileMatching();
    testWorstOnLine();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
