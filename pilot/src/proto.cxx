// See proto.hxx.

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

  } // namespace

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
          // No topic: the remainder of an ERR is a sentence, and splitting a
          // word off the front of it would produce a "topic" that is really
          // just the first word of the reason.
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
              out.rest  = tailFrom(out.line, te);
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
          // The boundary test. Without it `esc=` matches inside `desc=` and
          // this returns a number from a field nobody asked for.
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
      const Int64 v    = std::strtol(raw.c_str(), &stop, 10);

      // The WHOLE value has to be a number. strtol stopping early means the
      // field holds something else - `esc=off` is not 0, it is a different kind
      // of answer, and returning 0 for it would read as neutral throttle.
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
      const Float64 v    = std::strtod(raw.c_str(), &stop);
      if(stop == nullptr || *stop != '\0')
      {
          return false;
      }
      out = static_cast<Float32>(v);
      return true;
  }

  // ---- outbound ----------------------------------------------------------

  namespace
  {

    // A fixed-point float, written without printf.
    //
    // NOT snprintf("%.3f"). That honours the C locale, and on a machine set to
    // a comma decimal separator it emits `0,250` - which the board's parser
    // reads as 0, so the car goes straight when it was told to turn. The bug
    // appears only on somebody else's laptop and only in the field.
    //
    // Three decimals: the steering resolution is 430 us over the full range, so
    // a thousandth of the range is well under one microsecond of pulse.
    Str fixed3(Float32 v)
    {
        Bool neg = (v < 0.0f);
        if(neg)
        {
            v = -v;
        }

        // +0.0005 so the truncation below rounds instead of always going down.
        const Int32 scaled = static_cast<Int32>(v * 1000.0f + 0.5f);
        const Int32 whole  = scaled / 1000;
        const Int32 frac   = scaled % 1000;

        Array<Char, 32> buf;
        std::snprintf(buf.data(), buf.size(), "%s%d.%03d",
                      neg ? "-" : "", whole, frac);
        return Str(buf.data());
    }

  } // namespace

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

} // namespace proto
