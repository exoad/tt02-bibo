// See json.hxx. A recursive-descent reader, and nothing more.

#include "shared.hxx"
#include "json.hxx"

#include <cstdio>
#include <cstdlib>

namespace js
{

  namespace
  {

    // The one returned for every miss. Static so `at()` can hand back a
    // reference rather than a pointer, which is what lets a caller chain
    // `.at("result").at("items")[0]` without a guard at every step.
    const Value& none()
    {
        static const Value nil;
        return nil;
    }

    struct Reader
    {
        const Str* src = nullptr;
        Size       at  = 0;
        Bool       bad = false;

        [[nodiscard]] Bool done() const
        {
            return at >= src->size();
        }

        [[nodiscard]] Char peek() const
        {
            return done() ? '\0' : (*src)[at];
        }

        // Never reads past the end: at EOF this gives '\0', which every caller
        // below already treats as "not the character I wanted".
        Char take()
        {
            return done() ? '\0' : (*src)[at++];
        }

        Void skipSpace()
        {
            while(!done())
            {
                const Char c = (*src)[at];
                if(c == ' ' || c == '\t' || c == '\n' || c == '\r')
                {
                    ++at;
                }
                else
                {
                    break;
                }
            }
        }

        Bool want(Char c)
        {
            skipSpace();
            if(peek() != c)
            {
                bad = true;
                return false;
            }
            ++at;
            return true;
        }
    };

    Void parseValue(Reader& r, Value& out);

    // \uXXXX to UTF-8.
    //
    // clangd sends these: a doc comment with a non-ASCII character arrives
    // escaped, and so does the decoration character it puts on a completion
    // label. Dropping them would corrupt exactly the strings this reader
    // exists to deliver intact.
    //
    // Surrogate pairs are joined. A lone surrogate becomes U+FFFD rather than
    // being emitted raw, because a half-character in the middle of an
    // identifier is worse than a visible replacement.
    Void appendUtf8(Str& out, UInt32 cp)
    {
        if(cp < 0x80u)
        {
            out.push_back(static_cast<Char>(cp));
        }
        else if(cp < 0x800u)
        {
            out.push_back(static_cast<Char>(0xC0u | (cp >> 6)));
            out.push_back(static_cast<Char>(0x80u | (cp & 0x3Fu)));
        }
        else if(cp < 0x10000u)
        {
            out.push_back(static_cast<Char>(0xE0u | (cp >> 12)));
            out.push_back(static_cast<Char>(0x80u | ((cp >> 6) & 0x3Fu)));
            out.push_back(static_cast<Char>(0x80u | (cp & 0x3Fu)));
        }
        else
        {
            out.push_back(static_cast<Char>(0xF0u | (cp >> 18)));
            out.push_back(static_cast<Char>(0x80u | ((cp >> 12) & 0x3Fu)));
            out.push_back(static_cast<Char>(0x80u | ((cp >> 6) & 0x3Fu)));
            out.push_back(static_cast<Char>(0x80u | (cp & 0x3Fu)));
        }
    }

    [[nodiscard]] Bool hex4(Reader& r, UInt32& out)
    {
        out = 0u;
        for(Int32 i = 0; i < 4; ++i)
        {
            const Char c = r.take();
            UInt32     d = 0u;

            if(c >= '0' && c <= '9')
            {
                d = static_cast<UInt32>(c - '0');
            }
            else if(c >= 'a' && c <= 'f')
            {
                d = static_cast<UInt32>(c - 'a' + 10);
            }
            else if(c >= 'A' && c <= 'F')
            {
                d = static_cast<UInt32>(c - 'A' + 10);
            }
            else
            {
                return false;
            }

            out = (out << 4) | d;
        }
        return true;
    }

    Void parseString(Reader& r, Str& out)
    {
        if(!r.want('"'))
        {
            return;
        }

        while(!r.done())
        {
            const Char c = r.take();

            if(c == '"')
            {
                return;
            }
            if(c != '\\')
            {
                out.push_back(c);
                continue;
            }

            const Char e = r.take();
            switch(e)
            {
            case '"':  out.push_back('"');  break;
            case '\\': out.push_back('\\'); break;
            case '/':  out.push_back('/');  break;
            case 'b':  out.push_back('\b'); break;
            case 'f':  out.push_back('\f'); break;
            case 'n':  out.push_back('\n'); break;
            case 'r':  out.push_back('\r'); break;
            case 't':  out.push_back('\t'); break;
            case 'u':
            {
                UInt32 cp = 0u;
                if(!hex4(r, cp))
                {
                    r.bad = true;
                    return;
                }

                // A high surrogate must be followed by \uDC00-\uDFFF.
                if(cp >= 0xD800u && cp <= 0xDBFFu)
                {
                    if(r.peek() == '\\')
                    {
                        const Size save = r.at;
                        r.take();
                        if(r.take() == 'u')
                        {
                            UInt32 lo = 0u;
                            if(hex4(r, lo) && lo >= 0xDC00u && lo <= 0xDFFFu)
                            {
                                cp = 0x10000u
                                   + ((cp - 0xD800u) << 10)
                                   + (lo - 0xDC00u);
                            }
                            else
                            {
                                r.at = save;
                                cp   = 0xFFFDu;
                            }
                        }
                        else
                        {
                            r.at = save;
                            cp   = 0xFFFDu;
                        }
                    }
                    else
                    {
                        cp = 0xFFFDu;
                    }
                }
                else if(cp >= 0xDC00u && cp <= 0xDFFFu)
                {
                    cp = 0xFFFDu;   // a low surrogate on its own
                }

                appendUtf8(out, cp);
                break;
            }
            default:
                r.bad = true;
                return;
            }
        }

        r.bad = true;   // ran out before the closing quote
    }

    Void parseArray(Reader& r, Value& out)
    {
        out.type = Type::TYPE_ARRAY;
        if(!r.want('['))
        {
            return;
        }

        r.skipSpace();
        if(r.peek() == ']')
        {
            ++r.at;
            return;
        }

        while(!r.done() && !r.bad)
        {
            Value kid;
            parseValue(r, kid);
            if(r.bad)
            {
                return;
            }
            out.kids.push_back(std::move(kid));

            r.skipSpace();
            const Char c = r.take();
            if(c == ']')
            {
                return;
            }
            if(c != ',')
            {
                r.bad = true;
                return;
            }
        }
        r.bad = true;
    }

    Void parseObject(Reader& r, Value& out)
    {
        out.type = Type::TYPE_OBJECT;
        if(!r.want('{'))
        {
            return;
        }

        r.skipSpace();
        if(r.peek() == '}')
        {
            ++r.at;
            return;
        }

        while(!r.done() && !r.bad)
        {
            r.skipSpace();

            Str key;
            parseString(r, key);
            if(r.bad || !r.want(':'))
            {
                r.bad = true;
                return;
            }

            Value kid;
            parseValue(r, kid);
            if(r.bad)
            {
                return;
            }

            out.keys.push_back(std::move(key));
            out.kids.push_back(std::move(kid));

            r.skipSpace();
            const Char c = r.take();
            if(c == '}')
            {
                return;
            }
            if(c != ',')
            {
                r.bad = true;
                return;
            }
        }
        r.bad = true;
    }

    Void parseValue(Reader& r, Value& out)
    {
        r.skipSpace();
        const Char c = r.peek();

        if(c == '{')
        {
            parseObject(r, out);
        }
        else if(c == '[')
        {
            parseArray(r, out);
        }
        else if(c == '"')
        {
            out.type = Type::TYPE_STRING;
            parseString(r, out.str);
        }
        else if(r.src->compare(r.at, 4, "true") == 0)
        {
            out.type = Type::TYPE_BOOL;
            out.b    = true;
            r.at += 4;
        }
        else if(r.src->compare(r.at, 5, "false") == 0)
        {
            out.type = Type::TYPE_BOOL;
            out.b    = false;
            r.at += 5;
        }
        else if(r.src->compare(r.at, 4, "null") == 0)
        {
            out.type = Type::TYPE_NULL;
            r.at += 4;
        }
        else if(c == '-' || (c >= '0' && c <= '9'))
        {
            const Size begin = r.at;
            if(r.peek() == '-')
            {
                ++r.at;
            }
            while(!r.done())
            {
                const Char d = r.peek();
                if((d >= '0' && d <= '9') || d == '.' || d == 'e' || d == 'E'
                   || d == '+' || d == '-')
                {
                    ++r.at;
                }
                else
                {
                    break;
                }
            }
            out.type = Type::TYPE_NUMBER;
            out.num  = std::strtod(r.src->substr(begin, r.at - begin).c_str(),
                                   nullptr);
        }
        else
        {
            r.bad = true;
        }
    }

  }

  const Value& Value::at(const Char* key) const
  {
      if(type != Type::TYPE_OBJECT || key == nullptr)
      {
          return none();
      }
      for(Size i = 0; i < keys.size() && i < kids.size(); ++i)
      {
          if(keys[i] == key)
          {
              return kids[i];
          }
      }
      return none();
  }

  const Value& Value::operator[](Size i) const
  {
      return (i < kids.size()) ? kids[i] : none();
  }

  Str Value::string(const Char* fallback) const
  {
      return (type == Type::TYPE_STRING) ? str : Str(fallback);
  }

  Int32 Value::integer(Int32 fallback) const
  {
      return (type == Type::TYPE_NUMBER) ? static_cast<Int32>(num) : fallback;
  }

  Bool Value::boolean(Bool fallback) const
  {
      return (type == Type::TYPE_BOOL) ? b : fallback;
  }

  Value parse(const Str& text, Bool& ok, Size& stoppedAt)
  {
      Reader r;
      r.src = &text;

      Value out;
      parseValue(r, out);

      ok        = !r.bad;
      stoppedAt = r.at;

      if(!ok)
      {
          out = Value();
      }
      return out;
  }

  Str quote(const Str& raw)
  {
      Str out;
      out.reserve(raw.size() + 2u);
      out.push_back('"');

      for(const Char c : raw)
      {
          switch(c)
          {
          case '"':  out += "\\\""; break;
          case '\\': out += "\\\\"; break;
          case '\b': out += "\\b";  break;
          case '\f': out += "\\f";  break;
          case '\n': out += "\\n";  break;
          case '\r': out += "\\r";  break;
          case '\t': out += "\\t";  break;
          default:
              // Control characters MUST be escaped or the receiver rejects the
              // whole message. Everything else, including UTF-8 continuation
              // bytes, passes through untouched.
              if(static_cast<UInt8>(c) < 0x20u)
              {
                  Array<Char, 8> buf;
                  std::snprintf(buf.data(), buf.size(), "\\u%04X",
                                static_cast<UInt32>(static_cast<UInt8>(c)));
                  out += buf.data();
              }
              else
              {
                  out.push_back(c);
              }
              break;
          }
      }

      out.push_back('"');
      return out;
  }

}
