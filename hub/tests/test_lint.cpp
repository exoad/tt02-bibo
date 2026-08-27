// Tests for the live style linter. See src/lint.hpp.
//
// The two things being tested are not equally important. That a violation is
// FOUND matters; that correct code is left ALONE matters more. An underline
// sitting under good code all day is how a squiggle stops being read, and after
// that the linter may as well not exist.

#include "shared.hpp"
#include "lint.hpp"

#include <cstdio>

namespace {

Int32 checks   = 0;
Int32 failures = 0;

Void check(Bool ok, const Char* what)
{
    ++checks;
    if(ok)
    {
        std::printf("  ok    %s\n", what);
    }
    else
    {
        ++failures;
        std::printf("  FAIL  %s\n", what);
    }
}

// True when some diagnostic on the given line mentions `needle`.
Bool flags(const Str& src, const Char* needle, lint::Lang lang = lint::Lang::LANG_CPP)
{
    const Vec<diag::Item> v = lint::check(src, lang);
    for(const diag::Item& d : v)
    {
        if(d.message.find(needle) != Str::npos)
        {
            return true;
        }
    }
    return false;
}

Int32 count(const Str& src, lint::Lang lang = lint::Lang::LANG_CPP)
{
    return static_cast<Int32>(lint::check(src, lang).size());
}

Void testBuiltins()
{
    std::printf("\n-- bare builtin types --\n");

    check(flags("int x = 0;", "bare builtin"), "int is flagged");
    check(flags("float y = 1.0f;", "bare builtin"), "float is flagged");
    check(flags("bool ok = true;", "bare builtin"), "bool is flagged");
    check(flags("unsigned n = 0;", "bare builtin"), "unsigned is flagged");

    // The ones that must NOT fire.
    check(count("Int32 x = 0;") == 0, "Int32 is fine");
    check(count("Float32 y = 1.0f;") == 0, "Float32 is fine");
    check(count("sprint(x);") == 0, "int inside sprint is not the word int");
    check(count("Int32 printed = 0;") == 0, "int inside printed is not int");
    check(count("point.x = 1;") == 0, "point does not contain a bare int");
}

Void testLiteralsAndComments()
{
    std::printf("\n-- text is not code --\n");

    check(count("serialPrint(\"int is wrong here\");") == 0,
          "a builtin named in a STRING is not a violation");
    check(count("// int x, in a comment") == 0,
          "a line comment is not code");
    check(count("/* int x */ Int32 y = 0;") == 0,
          "a block comment is not code");

    // A block comment spanning lines, which is most of this project's headers.
    const Str block =
        "/*\n"
        " * int x = 0;   this is prose about a mistake\n"
        " */\n"
        "Int32 y = 0;\n";
    check(count(block) == 0, "a multi-line comment is not code");

    check(count("#include <stdint.h>") == 0, "preprocessor lines are exempt");
    check(count("#define COUNT 4") == 0, "a #define is not a violation");
}

Void testNames()
{
    std::printf("\n-- names --\n");

    check(flags("Void DoThing(Int32 x)", "camelCase"),
          "a PascalCase function is flagged");
    check(flags("static Void do_thing(Int32 x)", "camelCase"),
          "a snake_case function is flagged");
    check(count("Void doThing(Int32 x)") == 0, "camelCase function is fine");
    check(count("static Bool isReady(Void)") == 0, "static camelCase is fine");

    check(flags("struct thing", "PascalCase"), "a lowercase struct is flagged");
    check(flags("enum class my_mode", "PascalCase"), "a snake_case enum is flagged");
    check(count("struct Thing") == 0, "PascalCase struct is fine");
    check(count("enum class Mode") == 0, "PascalCase enum class is fine");

    check(flags("Int32 m_count = 0;", "m_"), "m_ prefix is flagged");
    check(flags("Int32 g_total = 0;", "g_"), "g_ prefix is flagged");
    check(flags("Int32 kMaxCount = 4;", "SCREAMING"), "k-prefix is flagged");
    check(flags("Int32 lines_ = 0;", "trailing"), "a trailing underscore is flagged");

    // A CALL to something ending in _ is not a declaration, and SCREAMING_ is
    // not lowerCamel_.
    check(count("Int32 n = MAX_COUNT;") == 0, "SCREAMING_SNAKE is fine");
}

Void testSpacingAndBraces()
{
    std::printf("\n-- spacing and braces --\n");

    check(flags("    if (x > 0)", "if("), "`if (` is flagged");
    check(flags("    while (running)", "while("), "`while (` is flagged");
    check(count("    if(x > 0)") == 0, "`if(` is fine");
    check(count("    for(Int32 i = 0; i < n; ++i)") == 0, "`for(` is fine");

    check(flags("    if(x) { return; }", "one-lined"), "a one-lined body is flagged");
    check(flags("    else { y = 1; }", "one-lined"), "a one-lined else is flagged");

    // The case that would ruin every table in the tree if it fired.
    check(count("    { Icon::ICON_RADAR, \"radar\" },") == 0,
          "an aggregate ROW is data, not a body");
    check(count("    { 1, 2, 3 },") == 0, "a plain initialiser row is fine");
    check(count("    Int32 empty[] = {};") == 0, "an empty brace pair is fine");
}

Void testCppOnly()
{
    std::printf("\n-- C is not C++ --\n");

    check(flags("Vec<std::string> names;", "shared.hpp alias"),
          "an unaliased std type is flagged in C++");

    // The firmware is C. It has shared.h, not shared.hpp, and no std at all -
    // so a C file must not be judged by C++ rules.
    check(count("struct thing", lint::Lang::LANG_C) > 0,
          "naming rules still apply to C");

    check(lint::langOf("sketch.c") == lint::Lang::LANG_C, ".c is C");
    check(lint::langOf("pico2w.h") == lint::Lang::LANG_C, ".h is C in this tree");
    check(lint::langOf("app_ui.cpp") == lint::Lang::LANG_CPP, ".cpp is C++");
    check(lint::langOf("editor.hpp") == lint::Lang::LANG_CPP, ".hpp is C++");
}

Void testPositions()
{
    std::printf("\n-- positions --\n");

    const Vec<diag::Item> v = lint::check("Int32 a = 0;\nint b = 0;\n",
                                          lint::Lang::LANG_CPP);
    check(v.size() == 1, "one violation on the second line");
    if(!v.empty())
    {
        check(v[0].line == 2, "reported on line 2, 1-based");
        check(v[0].column == 1, "column is 1-based");
        check(v[0].severity == diag::Severity::SEVERITY_WARN,
              "style issues are warnings, not errors");
    }
}

// The real files in this repo are the strongest possible test: every one of
// them passes tools/style_audit.py, so the linter agreeing with that is what
// "does not cry wolf" actually means.
Void testRealSources()
{
    std::printf("\n-- against this project's own code --\n");

    const Str clean =
        "// A comment mentioning int and float harmlessly.\n"
        "#include \"shared.hpp\"\n"
        "\n"
        "namespace ui {\n"
        "\n"
        "struct Thing\n"
        "{\n"
        "    Int32   count = 0;\n"
        "    Float32 scale = 1.0f;\n"
        "};\n"
        "\n"
        "static const Char* const NAMES[] = {\n"
        "    { \"one\" },\n"
        "};\n"
        "\n"
        "Void doThing(Int32 n)\n"
        "{\n"
        "    for(Int32 i = 0; i < n; ++i)\n"
        "    {\n"
        "        if(i > 2)\n"
        "        {\n"
        "            break;\n"
        "        }\n"
        "    }\n"
        "}\n"
        "\n"
        "} // namespace ui\n";

    const Vec<diag::Item> v = lint::check(clean, lint::Lang::LANG_CPP);
    if(!v.empty())
    {
        for(const diag::Item& d : v)
        {
            std::printf("        line %d col %d: %s\n",
                        d.line, d.column, d.message.c_str());
        }
    }
    check(v.empty(), "a conforming file produces NO diagnostics");
}

} // namespace

// With file arguments, lints those files and prints what it found instead of
// running the suite. The repo's own sources all pass tools/style_audit.py, so
// running this across them is the real false-positive test - synthetic snippets
// prove very little by comparison.
Int32 lintFiles(Int32 argc, Utf8** argv)
{
    Int32 total = 0;
    for(Int32 a = 1; a < argc; ++a)
    {
        const Str  path = argv[a];
        std::FILE* f    = std::fopen(path.c_str(), "rb");
        if(f == nullptr)
        {
            continue;
        }

        Str  text;
        Char buf[4096];
        Size n = 0;
        while((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
        {
            text.append(buf, n);
        }
        std::fclose(f);

        const Vec<diag::Item> v = lint::check(text, lint::langOf(path));
        for(const diag::Item& d : v)
        {
            std::printf("%s:%d:%d: %s\n", path.c_str(), d.line, d.column,
                        d.message.c_str());
        }
        total += static_cast<Int32>(v.size());
    }
    std::printf("TOTAL %d\n", total);
    return 0;
}

int main(int argc, char** argv)
{
    if(argc > 1)
    {
        return lintFiles(static_cast<Int32>(argc), argv);
    }

    std::printf("lint tests\n");

    testBuiltins();
    testLiteralsAndComments();
    testNames();
    testSpacingAndBraces();
    testCppOnly();
    testPositions();
    testRealSources();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
