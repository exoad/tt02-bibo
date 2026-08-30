// See lint.hxx.
//
// Hand-written scanning rather than std::regex. The audit's patterns are
// convenient in Python and would be run here on every keystroke, on a buffer
// that is being edited - std::regex is slow enough that this would be felt, and
// it throws on patterns it does not like, which is not a thing an editor should
// do while somebody is typing.

#include "shared.hxx"
#include "lint.hxx"

#include <cstring>

namespace lint
{
  namespace
  {

    Bool wordChar(Char c) noexcept
    {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
            || (c >= '0' && c <= '9') || c == '_';
    }

    Bool space(Char c) noexcept
    {
        return c == ' ' || c == '\t';
    }

    // A word at `at` is only a word if what surrounds it is not. Without this,
    // "int" matches inside "print", "sprint" and "Int32".
    Bool isWordAt(const Str& s, Size at, const Char* word)
    {
        const Size n = std::strlen(word);
        if(at + n > s.size() || s.compare(at, n, word) != 0)
        {
            return false;
        }
        if(at > 0 && wordChar(s[at - 1]))
        {
            return false;
        }
        if(at + n < s.size() && wordChar(s[at + n]))
        {
            return false;
        }
        return true;
    }

    // ---------------------------------------------------------------------------
    // Lines the rules do not apply to.
    //
    // Mirrors the audit's EXEMPT list. Preprocessor lines are the big one: an
    // #include path is full of words that look like bare types, and #define is
    // where SCREAMING_SNAKE lives rather than a violation of it.
    Bool exemptLine(const Str& line)
    {
        Size i = 0;
        while(i < line.size() && space(line[i]))
        {
            ++i;
        }
        if(i < line.size() && line[i] == '#')
        {
            return true;                      // preprocessor
        }

        const Char* const SKIP[] = {
            "sizeof",            // sizeof(Float32) is not a cast
            "static_cast<int>",  // a named cast TO an ABI type is correct
            "static_cast<float>",
            "static_cast<unsigned",
            "WINAPI", "WinMain", "APIENTRY",
            "IMGUI_IMPL_API",
            "extern \"C\"",
        };
        for(const Char* k : SKIP)
        {
            if(line.find(k) != Str::npos)
            {
                return true;
            }
        }

        // `using Float32 = float;` is the alias DEFINITION - shared.hxx has to name
        // the builtin, that being the entire point of the file.
        if(line.find("using ") != Str::npos && line.find(" = ") != Str::npos)
        {
            return true;
        }

        // And `typedef char Utf8;`, which is the same thing in shared.h. Both files
        // must name the builtins; that is what makes the aliases exist.
        {
            Size t = 0;
            while(t < line.size() && space(line[t]))
            {
                ++t;
            }
            if(line.compare(t, 8, "typedef ") == 0)
            {
                return true;
            }
        }

        // A NAMED cast to an ABI type is the correct way to reach one, so the
        // builtin inside it is not a violation - it is the fix. Any cast counts:
        // static_cast<long long>, reinterpret_cast<unsigned char*> and the rest.
        if(line.find("_cast<") != Str::npos)
        {
            return true;
        }

        // A lambda body genuinely reads better on one line when it is a single
        // expression - `[](const Str& a, const Str& b) { return a > b; }` as a sort
        // predicate. The Allman rule is about function and control-flow bodies, and
        // firing here would underline every comparator in the tree.
        //
        // Detected as `[` ... `]` followed by `(` ... `)` and then a brace, which
        // is the shape of a lambda and not of an if.
        {
            const Size br = line.find('[');
            if(br != Str::npos)
            {
                const Size brEnd = line.find(']', br);
                if(brEnd != Str::npos)
                {
                    Size p = brEnd + 1;
                    while(p < line.size() && space(line[p]))
                    {
                        ++p;
                    }
                    if(p < line.size() && line[p] == '(')
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    // Strips string and character literals and line comments, replacing them with
    // spaces so columns still line up. Without this, a message like "int is wrong"
    // trips the bare-builtin rule, and a URL in a comment trips half of them.
    Str blankOutText(const Str& line, Bool& inBlockComment)
    {
        Str out = line;
        Bool inStr  = false;
        Bool inChr  = false;

        for(Size i = 0; i < out.size(); ++i)
        {
            if(inBlockComment)
            {
                if(i + 1 < out.size() && out[i] == '*' && out[i + 1] == '/')
                {
                    inBlockComment = false;
                    out[i] = ' ';
                    out[i + 1] = ' ';
                    ++i;
                    continue;
                }
                out[i] = ' ';
                continue;
            }

            if(inStr || inChr)
            {
                const Char close = inStr ? '"' : '\'';
                if(out[i] == '\\' && i + 1 < out.size())
                {
                    out[i] = ' ';
                    out[i + 1] = ' ';
                    ++i;
                    continue;
                }
                if(out[i] == close)
                {
                    inStr = false;
                    inChr = false;
                }
                out[i] = ' ';
                continue;
            }

            if(out[i] == '"')
            {
                inStr = true;
                out[i] = ' ';
                continue;
            }
            if(out[i] == '\'')
            {
                inChr = true;
                out[i] = ' ';
                continue;
            }
            if(i + 1 < out.size() && out[i] == '/' && out[i + 1] == '/')
            {
                for(Size j = i; j < out.size(); ++j)
                {
                    out[j] = ' ';
                }
                break;
            }
            if(i + 1 < out.size() && out[i] == '/' && out[i + 1] == '*')
            {
                inBlockComment = true;
                out[i] = ' ';
                out[i + 1] = ' ';
                ++i;
                continue;
            }
        }
        return out;
    }

    Void add(Vec<diag::Item>& out, Int32 line, Size col, const Char* msg)
    {
        diag::Item it;
        it.line     = line;
        it.column   = static_cast<Int32>(col) + 1;   // 1-based, as compilers count
        it.severity = diag::Severity::SEVERITY_WARN;
        it.message  = msg;
        out.push_back(it);
    }

    // ---------------------------------------------------------------------------
    // The types the project spells its own way. `int` here is a naming choice, not
    // a portability one - the alias says how wide it is, which `int` never did.
    // EXACTLY the set tools/style_audit.py checks, and no more.
    //
    // Standalone `long` and `short` are NOT here, because the audit does not flag
    // them - and a linter that is stricter than the gate flags code the commit will
    // happily accept, which teaches people to ignore the linter. If those should be
    // caught, the place to add them is the audit, and then here.
    const Char* const BUILTINS[] = {
        "int", "float", "double", "bool", "char", "unsigned", "size_t",
    };

    // The return types a function definition starts with. Used to find the NAME
    // that follows, which is what the case rules are about.
    const Char* const RETURN_TYPES[] = {
        "Void", "Bool", "Int8", "Int16", "Int32", "Int64",
        "UInt8", "UInt16", "UInt32", "UInt64", "Float32", "Float64",
        "Size", "Str", "Char", "Utf8",
    };

    const Char* const STD_TYPES[] = {
        "std::vector", "std::deque", "std::array", "std::map", "std::set",
        "std::unordered_map", "std::unordered_set", "std::pair", "std::tuple",
        "std::string_view", "std::string", "std::optional", "std::variant",
        "std::function", "std::unique_ptr", "std::shared_ptr", "std::weak_ptr",
        "std::recursive_mutex", "std::mutex", "std::lock_guard", "std::unique_lock",
        "std::thread", "std::atomic",
    };

    Void checkLine(const Str& raw, const Str& code, Int32 lineNo, Lang lang, Vec<diag::Item>& out)
    {
        if(exemptLine(raw))
        {
            return;
        }

        // ---- bare builtin types ----------------------------------------------
        for(Size i = 0; i < code.size(); ++i)
        {
            for(const Char* b : BUILTINS)
            {
                if(!isWordAt(code, i, b))
                {
                    continue;
                }

                // In C the aliases come from shared.h and are the same names, so
                // this rule holds for both languages - but `char` in C is used for
                // string literals far more often, and Utf8 is the alias, so the
                // wording differs.
                add(out, lineNo, i,
                    (std::strcmp(b, "char") == 0 && lang == Lang::LANG_C)
                        ? "bare `char` - use Utf8 from shared.h"
                        : "bare builtin type - use the shared alias "
                          "(Int32, Float32, Bool, ...)");
                i += std::strlen(b) - 1;
                break;
            }
        }

        // ---- keyword spacing --------------------------------------------------
        const Char* const KEYWORDS[] = { "if", "for", "while", "switch" };
        for(const Char* k : KEYWORDS)
        {
            const Size n = std::strlen(k);
            for(Size i = 0; i + n < code.size(); ++i)
            {
                if(!isWordAt(code, i, k))
                {
                    continue;
                }
                Size j = i + n;
                Size spaces = 0;
                while(j < code.size() && space(code[j]))
                {
                    ++j;
                    ++spaces;
                }
                if(spaces > 0 && j < code.size() && code[j] == '(')
                {
                    Str msg = Str("`") + k + " (` - this project writes `" + k + "(`";
                    add(out, lineNo, i, msg.c_str());
                }
            }
        }

        // ---- prefixes and suffixes the project does not use -------------------
        for(Size i = 0; i + 2 < code.size(); ++i)
        {
            const Bool boundary = (i == 0) || !wordChar(code[i - 1]);
            if(!boundary)
            {
                continue;
            }
            if(code[i] == 'm' && code[i + 1] == '_' && wordChar(code[i + 2]))
            {
                add(out, lineNo, i, "`m_` prefix - members are plain camelCase");
            }
            else if(code[i] == 'g' && code[i + 1] == '_' && wordChar(code[i + 2]))
            {
                add(out, lineNo, i, "`g_` prefix - globals are plain camelCase");
            }
            else if(code[i] == 'k' && code[i + 1] >= 'A' && code[i + 1] <= 'Z')
            {
                // kSomething is the Google constant style; here constants are
                // SCREAMING_SNAKE.
                Size j = i + 1;
                while(j < code.size() && wordChar(code[j]))
                {
                    ++j;
                }
                Bool hasLower = false;
                for(Size q = i + 1; q < j; ++q)
                {
                    if(code[q] >= 'a' && code[q] <= 'z')
                    {
                        hasLower = true;
                    }
                }
                if(hasLower)
                {
                    add(out, lineNo, i,
                        "`k` prefix - constants are SCREAMING_SNAKE_CASE");
                }
            }
        }

        // ---- trailing underscore ----------------------------------------------
        for(Size i = 0; i < code.size(); ++i)
        {
            if(code[i] != '_')
            {
                continue;
            }
            const Bool endsWord = (i + 1 >= code.size()) || !wordChar(code[i + 1]);
            if(!endsWord || i == 0 || !wordChar(code[i - 1]))
            {
                continue;
            }

            // Walk back to the start; only flag lowerCamel_, not SOME_MACRO_.
            Size b = i;
            while(b > 0 && wordChar(code[b - 1]))
            {
                --b;
            }
            if(!(code[b] >= 'a' && code[b] <= 'z'))
            {
                continue;
            }
            Bool allLowerOrDigit = true;
            for(Size q = b; q < i; ++q)
            {
                if(code[q] == '_')
                {
                    allLowerOrDigit = false;   // snake_case_, a different rule
                }
            }
            if(!allLowerOrDigit)
            {
                continue;
            }

            // `foo_(` is a call to something ending in _, which the audit skips.
            Size k = i + 1;
            while(k < code.size() && space(code[k]))
            {
                ++k;
            }
            if(k < code.size() && code[k] == '(')
            {
                continue;
            }
            add(out, lineNo, b, "trailing underscore - camelCase, no trailing _");
        }

        // ---- function names ---------------------------------------------------
        //
        // Only a line that STARTS with a return type, which is what a definition
        // looks like. A call in the middle of an expression is not one, and
        // treating it as one is how a linter starts underlining correct code.
        {
            Size i = 0;
            while(i < code.size() && space(code[i]))
            {
                ++i;
            }
            const Char* const LEAD[] = { "static ", "const ", "inline ", "extern " };
            Bool moved = true;
            while(moved)
            {
                moved = false;
                for(const Char* l : LEAD)
                {
                    const Size n = std::strlen(l);
                    if(code.compare(i, n, l) == 0)
                    {
                        i += n;
                        moved = true;
                    }
                }
            }

            for(const Char* t : RETURN_TYPES)
            {
                if(!isWordAt(code, i, t))
                {
                    continue;
                }
                Size j = i + std::strlen(t);
                while(j < code.size() && (space(code[j]) || code[j] == '*'
                                          || code[j] == '&'))
                {
                    ++j;
                }
                if(j >= code.size() || !wordChar(code[j]))
                {
                    break;
                }

                const Size nameAt = j;
                Size e = j;
                while(e < code.size() && wordChar(code[e]))
                {
                    ++e;
                }
                Size p = e;
                while(p < code.size() && space(code[p]))
                {
                    ++p;
                }
                if(p >= code.size() || code[p] != '(')
                {
                    break;                      // a variable, not a function
                }

                const Str name = code.substr(nameAt, e - nameAt);

                // Capital-first AND no underscore, which is what the audit's
                // PascalCase pattern matches. A name like LinkImpl_run_trampoline
                // is neither PascalCase nor snake_case; the audit passes it and so
                // does this, because the two must not disagree.
                const Bool hasUnderscore = (name.find('_') != Str::npos);
                if(name[0] >= 'A' && name[0] <= 'Z' && !hasUnderscore)
                {
                    add(out, lineNo, nameAt,
                        "function starts with a capital - functions are camelCase");
                }
                else if(hasUnderscore && name[0] >= 'a' && name[0] <= 'z')
                {
                    add(out, lineNo, nameAt,
                        "snake_case function - functions are camelCase");
                }
                break;
            }
        }

        // ---- type names -------------------------------------------------------
        //
        // struct / enum / typedef introduce a TYPE, and types are PascalCase.
        {
            const Char* const INTRO[] = { "struct ", "enum class ", "enum ",
                                          "class ", "union " };
            for(const Char* kw : INTRO)
            {
                const Size pos = code.find(kw);
                if(pos == Str::npos)
                {
                    continue;
                }
                if(pos > 0 && wordChar(code[pos - 1]))
                {
                    continue;
                }
                Size j = pos + std::strlen(kw);
                while(j < code.size() && space(code[j]))
                {
                    ++j;
                }
                if(j >= code.size() || !wordChar(code[j]))
                {
                    break;
                }
                Size e = j;
                while(e < code.size() && wordChar(code[e]))
                {
                    ++e;
                }
                const Str name = code.substr(j, e - j);
                if(name[0] >= 'a' && name[0] <= 'z')
                {
                    add(out, lineNo, j,
                        "type name is lowercase - types are PascalCase");
                }
                else if(name.find('_') != Str::npos
                        && !(name[0] >= 'A' && name[0] <= 'Z'
                             && name.find_first_of("abcdefghijklmnopqrstuvwxyz")
                                == Str::npos))
                {
                    add(out, lineNo, j,
                        "snake_case type name - types are PascalCase");
                }
                break;
            }
        }

        // ---- C++ only ---------------------------------------------------------
        if(lang == Lang::LANG_CPP)
        {
            for(const Char* t : STD_TYPES)
            {
                const Size pos = code.find(t);
                if(pos != Str::npos)
                {
                    add(out, lineNo, pos,
                        "unaliased std type - use the shared.hxx alias "
                        "(Vec, Str, Map, Mutex, ...)");
                    break;
                }
            }
        }

        // ---- one-lined body ---------------------------------------------------
        //
        // A `{` with content and a `}` on the same line, where what precedes the
        // brace is a `)` or else/do/try. That last part is what separates a BODY
        // from a table ROW - `{ Icon::ICON_RADAR, "radar" },` is data, and
        // expanding it would quadruple every table in the tree for nothing.
        {
            const Size open = code.find('{');
            if(open != Str::npos)
            {
                const Size close = code.find('}', open + 1);
                if(close != Str::npos && code.find('{', open + 1) == Str::npos)
                {
                    Bool hasContent = false;
                    for(Size q = open + 1; q < close; ++q)
                    {
                        if(!space(code[q]))
                        {
                            hasContent = true;
                        }
                    }

                    Size b = open;
                    while(b > 0 && space(code[b - 1]))
                    {
                        --b;
                    }
                    Bool isBody = (b > 0 && code[b - 1] == ')');
                    for(const Char* kw : { "else", "do", "try" })
                    {
                        const Size n = std::strlen(kw);
                        if(b >= n && code.compare(b - n, n, kw) == 0
                           && (b == n || !wordChar(code[b - n - 1])))
                        {
                            isBody = true;
                        }
                    }

                    if(hasContent && isBody)
                    {
                        add(out, lineNo, open,
                            "one-lined body - Allman braces, on their own lines");
                    }
                }
            }
        }
    }

  } // namespace

  Lang langOf(const Str& path)
  {
      // .h is ambiguous everywhere else and is not here: this project's headers
      // are .hpp for C++ and .h for C, which is exactly why it picked that split.
      if(path.size() >= 2 && path.compare(path.size() - 2, 2, ".c") == 0)
      {
          return Lang::LANG_C;
      }
      if(path.size() >= 2 && path.compare(path.size() - 2, 2, ".h") == 0)
      {
          return Lang::LANG_C;
      }
      return Lang::LANG_CPP;
  }

  Vec<diag::Item> check(const Str& text, Lang lang)
  {
      Vec<diag::Item> out;

      Str   line;
      Int32 lineNo = 1;
      Bool  inBlockComment = false;

      for(Size i = 0; i <= text.size(); ++i)
      {
          if(i == text.size() || text[i] == '\n')
          {
              const Str code = blankOutText(line, inBlockComment);
              checkLine(line, code, lineNo, lang, out);
              line.clear();
              ++lineNo;
              if(i == text.size())
              {
                  break;
              }
              continue;
          }
          if(text[i] != '\r')
          {
              line.push_back(text[i]);
          }
      }
      return out;
  }

} // namespace lint
