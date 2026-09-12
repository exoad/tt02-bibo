#include "trimfile.hxx"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include "bibowire.hxx"

namespace trimfile
{

  namespace
  {

    [[nodiscard]] Vec<Str> words(const Str& line)
    {
        Vec<Str> out;
        Size at = 0;
        while(at < line.size())
        {
            while(at < line.size() && (line[at] == ' ' || line[at] == '\t' || line[at] == '\r'))
            {
                ++at;
            }
            Size to = at;
            while(to < line.size() && line[to] != ' ' && line[to] != '\t' && line[to] != '\r')
            {
                ++to;
            }
            if(to > at)
            {
                out.push_back(line.substr(at, to - at));
            }
            at = to;
        }
        return out;
    }

    // The WHOLE word must be a number inside [lo, hi]. strtol stopping early
    // means the word is something else, and "1480x" is not 1480.
    [[nodiscard]] Bool number(const Str& word, Int32 lo, Int32 hi, Int32& out)
    {
        if(word.empty())
        {
            return false;
        }
        Char* stop = nullptr;
        const long v = std::strtol(word.c_str(), &stop, 10);
        if(stop == nullptr || *stop != '\0' || v < lo || v > hi)
        {
            return false;
        }
        out = static_cast<Int32>(v);
        return true;
    }

    // Integers only, through %d - no locale can put a comma in these.
    [[nodiscard]] Str one(const Char* verb, Int32 a)
    {
        Array<Char, 48> t{};
        std::snprintf(t.data(), t.size(), "%s %d", verb, static_cast<int>(a));
        return Str(t.data());
    }

    [[nodiscard]] Str two(const Char* verb, Int32 a, Int32 b)
    {
        Array<Char, 48> t{};
        std::snprintf(t.data(), t.size(), "%s %d %d", verb, static_cast<int>(a), static_cast<int>(b));
        return Str(t.data());
    }

    [[nodiscard]] Bool set(Str& slot, const Str& value)
    {
        if(slot == value)
        {
            return false;
        }
        slot = value;
        return true;
    }

    // Every directory above `path`, created in turn. EEXIST is the common case
    // and not a failure; anything else is left for the open that follows to
    // report, with a reason that names the file rather than a component.
    Void makeParents(const Str& path)
    {
        for(Size i = 1; i < path.size(); ++i)
        {
            if(path[i] != '/' && path[i] != '\\')
            {
                continue;
            }
            const Str dir = path.substr(0, i);
#if defined(_WIN32)
            static_cast<Void>(::_mkdir(dir.c_str()));
#else
            static_cast<Void>(::mkdir(dir.c_str(), 0755));
#endif
        }
    }

  }

  Bool remember(Store& s, const Str& line)
  {
      const Vec<Str> w = words(line);
      if(w.empty())
      {
          return false;
      }

      const Int32 servoLo = static_cast<Int32>(bibowire::SERVO_US_HARD_MIN);
      const Int32 servoHi = static_cast<Int32>(bibowire::SERVO_US_HARD_MAX);
      const Int32 escLo = static_cast<Int32>(bibowire::ESC_US_HARD_MIN);
      const Int32 escHi = static_cast<Int32>(bibowire::ESC_US_HARD_MAX);
      const Int32 slewLo = static_cast<Int32>(bibowire::SLEW_US_MIN);
      const Int32 slewHi = static_cast<Int32>(bibowire::SLEW_US_MAX);

      Int32 a = 0;
      Int32 b = 0;
      if(w[0] == "SERVOLIMITS" && w.size() == 3u)
      {
          if(number(w[1], servoLo, servoHi, a) && number(w[2], servoLo, servoHi, b) && a < b)
          {
              return set(s.servoLimits, two("SERVOLIMITS", a, b));
          }
          return false;
      }
      if(w[0] == "SERVOTRIM" && w.size() == 2u)
      {
          if(number(w[1], servoLo, servoHi, a))
          {
              return set(s.servoTrim, one("SERVOTRIM", a));
          }
          return false;
      }
      if(w[0] == "ESCLIMITS" && w.size() == 3u)
      {
          if(number(w[1], escLo, escHi, a) && number(w[2], escLo, escHi, b) && a < b)
          {
              return set(s.escLimits, two("ESCLIMITS", a, b));
          }
          return false;
      }
      if(w[0] == "SLEW")
      {
          // The axis-less form sets BOTH, because that is what the Pico does
          // with it - storing it as its own key would replay a rate that the
          // per-axis lines stored after it silently contradict.
          if(w.size() == 2u && number(w[1], slewLo, slewHi, a))
          {
              const Bool steer = set(s.steerSlew, one("SLEW STEER", a));
              const Bool throttle = set(s.throttleSlew, one("SLEW THROTTLE", a));
              return steer || throttle;
          }
          if(w.size() == 3u && number(w[2], slewLo, slewHi, a))
          {
              if(w[1] == "STEER")
              {
                  return set(s.steerSlew, one("SLEW STEER", a));
              }
              if(w[1] == "THROTTLE")
              {
                  return set(s.throttleSlew, one("SLEW THROTTLE", a));
              }
          }
          return false;
      }
      return false;
  }

  Vec<Str> lines(const Store& s)
  {
      Vec<Str> out;
      for(const Str* slot : { &s.servoLimits, &s.escLimits, &s.servoTrim, &s.steerSlew, &s.throttleSlew })
      {
          if(!slot->empty())
          {
              out.push_back(*slot);
          }
      }
      return out;
  }

  Str render(const Store& s)
  {
      Str out;
      out += "# bibo trim - the Pico's own commands, replayed by the pilot whenever it opens\n";
      out += "# the Pico. Written by the board when the viewer's Trim pane changes a value.\n";
      out += "# Safe to edit by hand; a line that is not a valid setting is ignored.\n";
      for(const Str& line : lines(s))
      {
          out += line;
          out += '\n';
      }
      return out;
  }

  Store parse(const Str& text)
  {
      Store s;
      Size at = 0;
      while(at <= text.size())
      {
          Size nl = text.find('\n', at);
          if(nl == Str::npos)
          {
              nl = text.size();
          }
          const Str line = text.substr(at, nl - at);
          if(!line.empty() && line[0] != '#')
          {
              static_cast<Void>(remember(s, line));
          }
          at = nl + 1u;
      }
      return s;
  }

  Str defaultPath()
  {
      const Char* explicitPath = std::getenv("BIBO_TRIM_FILE");
      if(explicitPath != nullptr && explicitPath[0] != '\0')
      {
          return Str(explicitPath);
      }
      const Char* home = std::getenv("HOME");
      if(home != nullptr && home[0] != '\0')
      {
          return Str(home) + "/.config/bibo/trim.txt";
      }
      return Str("bibo-trim.txt");
  }

  Bool load(const Str& path, Store& out, Str& why)
  {
      out = Store();
      std::FILE* f = std::fopen(path.c_str(), "rb");
      if(f == nullptr)
      {
          if(errno == ENOENT)
          {
              return true;
          }
          why = std::strerror(errno);
          return false;
      }
      Str text;
      Array<Char, 512> chunk{};
      for(;;)
      {
          const Size n = std::fread(chunk.data(), 1u, chunk.size(), f);
          if(n == 0u)
          {
              break;
          }
          text.append(chunk.data(), n);
      }
      std::fclose(f);
      out = parse(text);
      return true;
  }

  Bool save(const Str& path, const Store& s, Str& why)
  {
      makeParents(path);
      const Str tmp = path + ".tmp";
      std::FILE* f = std::fopen(tmp.c_str(), "wb");
      if(f == nullptr)
      {
          why = std::strerror(errno);
          return false;
      }
      const Str text = render(s);
      const Bool wrote = std::fwrite(text.data(), 1u, text.size(), f) == text.size();
      const Bool flushed = std::fflush(f) == 0;
      std::fclose(f);
      if(!wrote || !flushed)
      {
          why = "the write did not complete";
          static_cast<Void>(std::remove(tmp.c_str()));
          return false;
      }
#if defined(_WIN32)
      // rename() will not replace an existing file on Windows.
      static_cast<Void>(std::remove(path.c_str()));
#endif
      if(std::rename(tmp.c_str(), path.c_str()) != 0)
      {
          why = std::strerror(errno);
          static_cast<Void>(std::remove(tmp.c_str()));
          return false;
      }
      return true;
  }

}
