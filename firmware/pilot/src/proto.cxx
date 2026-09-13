#include "proto.hxx"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace proto
{
  namespace
  {
    Bool isSpace(Char c) noexcept
    {
        return c == ' ' || c == '\t';
    }

    // [begin, end) of the first word at or after `at`, skipping leading spaces.
    Void wordAt(const Str& s, Size at, Size& begin, Size& end)
    {
        while(at < s.size() && isSpace(s[at]))
        {
            ++at;
        }
        begin = at;
        while(at < s.size() && !isSpace(s[at]))
        {
            ++at;
        }
        end = at;
    }

    // Everything from `at`, with leading spaces removed.
    Str tailFrom(const Str& s, Size at)
    {
        while(at < s.size() && isSpace(s[at]))
        {
            ++at;
        }
        return (at < s.size()) ? s.substr(at) : Str();
    }

    Str trimEnd(const Str& s)
    {
        Size n = s.size();
        while(n > 0 && (s[n - 1] == '\r' || s[n - 1] == '\n'
                        || isSpace(s[n - 1])))
        {
            --n;
        }
        return s.substr(0, n);
    }
  }

  Reply read(const Str& line)
  {
      Reply out;
      out.line = trimEnd(line);
      Size b = 0;
      Size e = 0;
      wordAt(out.line, 0, b, e);
      if(b >= e)
      {
          out.kind = Kind::KIND_EMPTY;
          return out;
      }
      const Str head = out.line.substr(b, e - b);
      if(head == "ERR")
      {
          // No topic: an ERR's remainder is a sentence, whose first word is not
          // a topic.
          out.kind = Kind::KIND_ERR;
          out.rest = tailFrom(out.line, e);
          return out;
      }
      if(head == "OK" || head == "INFO")
      {
          out.kind = (head == "OK") ? Kind::KIND_OK : Kind::KIND_INFO;
          Size tb = 0;
          Size te = 0;
          wordAt(out.line, e, tb, te);
          if(tb < te)
          {
              out.topic = out.line.substr(tb, te - tb);
              out.rest = tailFrom(out.line, te);
          }
          return out;
      }
      out.kind = Kind::KIND_OTHER;
      out.rest = out.line;
      return out;
  }

  Bool field(const Str& text, const Char* key, Str& out)
  {
      if(key == nullptr || *key == '\0')
      {
          return false;
      }
      const Size klen = std::strlen(key);
      for(Size i = 0; i + klen <= text.size(); ++i)
      {
          if(i != 0 && !isSpace(text[i - 1]))
          {
              continue;
          }
          if(text.compare(i, klen, key) != 0)
          {
              continue;
          }
          Size at = i + klen;
          Size to = at;
          while(to < text.size() && !isSpace(text[to]))
          {
              ++to;
          }
          out = text.substr(at, to - at);
          return true;
      }
      return false;
  }

  Bool fieldInt(const Str& text, const Char* key, Int32& out)
  {
      Str raw;
      if(!field(text, key, raw) || raw.empty())
      {
          return false;
      }
      Char*       stop = nullptr;
      const Int64 v = std::strtol(raw.c_str(), &stop, 10);
      // The WHOLE value must be a number: `esc=off` returned as 0 would read as
      // neutral throttle.
      if(stop == nullptr || *stop != '\0')
      {
          return false;
      }
      out = static_cast<Int32>(v);
      return true;
  }

  Bool fieldFloat(const Str& text, const Char* key, Float32& out)
  {
      Str raw;
      if(!field(text, key, raw) || raw.empty())
      {
          return false;
      }
      Char*         stop = nullptr;
      const Float64 v = std::strtod(raw.c_str(), &stop);
      if(stop == nullptr || *stop != '\0')
      {
          return false;
      }
      out = static_cast<Float32>(v);
      return true;
  }

  namespace
  {
    // Three decimals built from integers, NOT snprintf("%.3f"): that honours
    // the C locale, and with a comma decimal separator emits `0,250`, which the
    // board reads as 0, so the car goes straight when told to turn. A
    // thousandth of the steering range is well under a microsecond of pulse.
    Str fixed3(Float32 v)
    {
        Bool neg = (v < 0.0f);
        if(neg)
        {
            v = -v;
        }
        // Rounds rather than truncating.
        const Int32 scaled = static_cast<Int32>(v * 1000.0f + 0.5f);
        const Int32 whole = scaled / 1000;
        const Int32 frac = scaled % 1000;
        Array<Char, 32> buf;
        std::snprintf(buf.data(), buf.size(), "%s%d.%03d", neg ? "-" : "", whole, frac);
        return Str(buf.data());
    }
  }

  Str steer(Float32 fraction)
  {
      if(fraction < -1.0f)
      {
          fraction = -1.0f;
      }
      if(fraction > 1.0f)
      {
          fraction = 1.0f;
      }
      return Str("STEER ") + fixed3(fraction);
  }

  Str escUs(Int32 us)
  {
      Array<Char, 32> buf;
      std::snprintf(buf.data(), buf.size(), "ESC %d", us);
      return Str(buf.data());
  }

  Str stop()
  {
      return Str("STOP");
  }

  Str command(const Char* verb, const Char* args)
  {
      if(verb == nullptr || *verb == '\0')
      {
          return Str();
      }
      Str out(verb);
      if(args != nullptr && *args != '\0')
      {
          out += ' ';
          out += args;
      }
      return out;
  }
}
