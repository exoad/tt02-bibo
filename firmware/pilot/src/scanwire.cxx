#include "scanwire.hxx"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace scanwire
{

  namespace
  {

    constexpr Int32 CENTI_PER_TURN = 36000;
    constexpr Int32 QUALITY_MAX = 63;

    // The whole token has to be digits (with an optional sign): strtol stopping
    // early is how "12abc" reads as 12, and a wire format is where that kind
    // of forgiveness becomes a wrong picture rather than a wrong number.
    [[nodiscard]] Bool wholeInt(StrView tok, Int64* out)
    {
        if(tok.empty() || tok.size() > 20)
        {
            return false;
        }
        Array<Char, 24> buf{};
        std::memcpy(buf.data(), tok.data(), tok.size());
        Char* stop = nullptr;
        const Int64 v = std::strtoll(buf.data(), &stop, 10);
        if(stop == buf.data() || *stop != '\0')
        {
            return false;
        }
        *out = v;
        return true;
    }

    // Successive whitespace-separated tokens of a line.
    struct Words
    {
        StrView rest;

        [[nodiscard]] Bool next(StrView* tok)
        {
            Size i = 0;
            while(i < rest.size() && (rest[i] == ' ' || rest[i] == '\t'))
            {
                ++i;
            }
            if(i >= rest.size())
            {
                rest = StrView();
                return false;
            }
            Size j = i;
            while(j < rest.size() && rest[j] != ' ' && rest[j] != '\t')
            {
                ++j;
            }
            *tok = rest.substr(i, j - i);
            rest = rest.substr(j);
            return true;
        }
    };

    [[nodiscard]] Int32 roundToInt(Float32 v)
    {
        return static_cast<Int32>(v + (v >= 0.0f ? 0.5f : -0.5f));
    }

    // "a,d,q" -> Sample. Each part whole and in range, or the frame is bad.
    [[nodiscard]] Bool readSample(StrView tok, Sample* s)
    {
        const Size c1 = tok.find(',');
        if(c1 == StrView::npos)
        {
            return false;
        }
        const Size c2 = tok.find(',', c1 + 1);
        if(c2 == StrView::npos)
        {
            return false;
        }
        Int64 a = 0;
        Int64 d = 0;
        Int64 q = 0;
        if(!wholeInt(tok.substr(0, c1), &a) || !wholeInt(tok.substr(c1 + 1, c2 - c1 - 1), &d)
           || !wholeInt(tok.substr(c2 + 1), &q))
        {
            return false;
        }
        if(a < 0 || a >= CENTI_PER_TURN || d < 0 || q < 0 || q > QUALITY_MAX)
        {
            return false;
        }
        s->angleDeg = static_cast<Float32>(a) / 100.0f;
        s->distMm = static_cast<Float32>(d);
        s->quality = static_cast<UInt8>(q);
        return true;
    }

    [[nodiscard]] Kind bad(Line* out)
    {
        out->frame.samples.clear();
        out->kind = Kind::KIND_BAD;
        return out->kind;
    }

  }

  // ---- writing ---------------------------------------------------------------

  Str formatFrame(const Frame& f)
  {
      // ~500 samples of "35999,12000,63 " is ~8 KB; reserve once rather than
      // grow through it ten times a second.
      Str line;
      line.reserve(24 + f.samples.size() * 16);
      Array<Char, 48> head{};
      std::snprintf(
          head.data(),
          head.size(),
          "F %u %d",
          static_cast<unsigned>(f.samples.size()),
          roundToInt(f.hz * 1000.0f)
      );
      line += head.data();
      Array<Char, 40> one{};
      for(const Sample& s : f.samples)
      {
          // The device's own convention: 360 wraps to 0, never to 36000.
          Int32 a = roundToInt(s.angleDeg * 100.0f) % CENTI_PER_TURN;
          if(a < 0)
          {
              a += CENTI_PER_TURN;
          }
          const Int32 d = s.distMm > 0.0f ? roundToInt(s.distMm) : 0;
          const Int32 q = s.quality > QUALITY_MAX ? QUALITY_MAX : static_cast<Int32>(s.quality);
          std::snprintf(one.data(), one.size(), " %d,%d,%d", a, d, q);
          line += one.data();
      }
      line += '\n';
      return line;
  }

  Str formatDrive(const Drive& d)
  {
      // The mode is one word or the line is not a D line: a space in it would
      // shift every field after it for the reader.
      Str mode;
      for(const Char c : d.mode)
      {
          mode += (c == ' ' || c == '\t') ? '_' : c;
      }
      Array<Char, 96> buf{};
      std::snprintf(
          buf.data(),
          buf.size(),
          "D %s %d %d %d %d %d\n",
          mode.empty() ? "-" : mode.c_str(),
          d.clearanceMm,
          d.hits,
          roundToInt(d.steer * 1000.0f),
          roundToInt(d.throttle * 1000.0f),
          d.stop ? 1 : 0
      );
      return Str(buf.data());
  }

  Str formatInfo(const Info& i)
  {
      Array<Char, 128> buf{};
      std::snprintf(
          buf.data(),
          buf.size(),
          "INFO %d %d.%d %d %s\n",
          i.model,
          i.fwMajor,
          i.fwMinor,
          i.hwRev,
          i.serial.empty() ? "-" : i.serial.c_str()
      );
      return Str(buf.data());
  }

  Str formatHealth(const Int32 health)
  {
      Array<Char, 24> buf{};
      std::snprintf(buf.data(), buf.size(), "HEALTH %d\n", health);
      return Str(buf.data());
  }

  Str formatMotor(const Bool on)
  {
      return on ? Str("MOTOR 1\n") : Str("MOTOR 0\n");
  }

  Str formatErr(const Str& message)
  {
      // One line, whatever the message contained.
      Str line = "ERR ";
      for(const Char c : message)
      {
          line += (c == '\n' || c == '\r') ? ' ' : c;
      }
      line += '\n';
      return line;
  }

  Str formatQuit()
  {
      return Str("QUIT\n");
  }

  // ---- reading ---------------------------------------------------------------

  Kind parse(StrView line, Line* out)
  {
      if(!line.empty() && line.back() == '\r')
      {
          line.remove_suffix(1);
      }
      out->text.clear();
      Words w{line};
      StrView word;
      if(!w.next(&word))
      {
          out->frame.samples.clear();
          out->kind = Kind::KIND_EMPTY;
          return out->kind;
      }

      if(word == "F")
      {
          StrView countTok;
          StrView freqTok;
          Int64   count = 0;
          Int64   freq = 0;
          if(!w.next(&countTok) || !wholeInt(countTok, &count) || !w.next(&freqTok)
             || !wholeInt(freqTok, &freq) || count < 0 || count > 65535 || freq < 0)
          {
              return bad(out);
          }
          out->frame.samples.clear();
          out->frame.samples.reserve(static_cast<Size>(count));
          StrView tok;
          while(w.next(&tok))
          {
              Sample s;
              if(!readSample(tok, &s))
              {
                  return bad(out);
              }
              out->frame.samples.push_back(s);
          }
          // The count is a promise about the line; a line that broke it was
          // cut somewhere, and its remaining samples are not the revolution.
          if(static_cast<Int64>(out->frame.samples.size()) != count)
          {
              return bad(out);
          }
          out->frame.hz = static_cast<Float32>(freq) / 1000.0f;
          out->kind = Kind::KIND_FRAME;
          return out->kind;
      }

      out->frame.samples.clear();

      if(word == "D")
      {
          StrView mode;
          Array<StrView, 5> tok{};
          Array<Int64, 5>   v{};
          if(!w.next(&mode))
          {
              return bad(out);
          }
          for(Size i = 0; i < tok.size(); ++i)
          {
              if(!w.next(&tok[i]) || !wholeInt(tok[i], &v[i]))
              {
                  return bad(out);
              }
          }
          if(v[0] < 0 || v[1] < 0 || v[2] < -1000 || v[2] > 1000 || v[3] < -1000 || v[3] > 1000
             || (v[4] != 0 && v[4] != 1))
          {
              return bad(out);
          }
          out->drive.mode = Str(mode);
          out->drive.clearanceMm = static_cast<Int32>(v[0]);
          out->drive.hits = static_cast<Int32>(v[1]);
          out->drive.steer = static_cast<Float32>(v[2]) / 1000.0f;
          out->drive.throttle = static_cast<Float32>(v[3]) / 1000.0f;
          out->drive.stop = v[4] == 1;
          out->kind = Kind::KIND_DRIVE;
          return out->kind;
      }

      if(word == "INFO")
      {
          StrView model;
          StrView fw;
          StrView hw;
          StrView serial;
          Int64   m = 0;
          Int64   h = 0;
          Int64   major = 0;
          Int64   minor = 0;
          if(!w.next(&model) || !w.next(&fw) || !w.next(&hw) || !w.next(&serial))
          {
              return bad(out);
          }
          const Size dot = fw.find('.');
          if(dot == StrView::npos || !wholeInt(model, &m) || !wholeInt(hw, &h))
          {
              return bad(out);
          }
          if(!wholeInt(fw.substr(0, dot), &major) || !wholeInt(fw.substr(dot + 1), &minor))
          {
              return bad(out);
          }
          out->info.model = static_cast<Int32>(m);
          out->info.fwMajor = static_cast<Int32>(major);
          out->info.fwMinor = static_cast<Int32>(minor);
          out->info.hwRev = static_cast<Int32>(h);
          out->info.serial = serial == "-" ? Str() : Str(serial);
          out->kind = Kind::KIND_INFO;
          return out->kind;
      }

      if(word == "HEALTH")
      {
          StrView tok;
          Int64   v = 0;
          if(!w.next(&tok) || !wholeInt(tok, &v) || v < 0 || v > 2)
          {
              return bad(out);
          }
          out->health = static_cast<Int32>(v);
          out->kind = Kind::KIND_HEALTH;
          return out->kind;
      }

      if(word == "MOTOR")
      {
          StrView tok;
          Int64   v = 0;
          if(!w.next(&tok) || !wholeInt(tok, &v) || (v != 0 && v != 1))
          {
              return bad(out);
          }
          out->motor = v == 1;
          out->kind = Kind::KIND_MOTOR;
          return out->kind;
      }

      if(word == "ERR")
      {
          Size i = 0;
          while(i < w.rest.size() && (w.rest[i] == ' ' || w.rest[i] == '\t'))
          {
              ++i;
          }
          out->text = Str(w.rest.substr(i));
          out->kind = Kind::KIND_ERR;
          return out->kind;
      }

      if(word == "QUIT")
      {
          out->kind = Kind::KIND_QUIT;
          return out->kind;
      }

      out->text = Str(word);
      out->kind = Kind::KIND_UNKNOWN;
      return out->kind;
  }

}
