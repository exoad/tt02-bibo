#include "vlog.hxx"

// <windows.h> for what the standard library cannot say here: where this exe
// lives, making a directory by wide path, a precise UTC clock and the process id.
// NOTHING OF OURS IS INCLUDED AFTER IT, which is why this file carries none of
// main.cxx's `small`/`near`/`far`/SEVERITY_ERROR surgery - the day a link or
// bibowire header is included below this line, that block has to come with it.
#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace vlog
{

  namespace
  {

    // 50 MiB. A healthy session writes a summary line a second and a handful of
    // keepalive lines beside it - well under a megabyte an hour - so reaching
    // this means something is logging in a loop, and the cap is what stops that
    // filling the disk of a laptop left running in a field.
    constexpr Size CAP_BYTES = Size{50} * 1024 * 1024;

    // One line after its prefix. An EVENT or CMDACK sentence is a few hundred
    // bytes at most, and the once-a-second summary is the longest line written.
    constexpr Size LINE_BYTES = 2048;

    constexpr Size PATH_CHARS = 1024;

    // A wide path, because the directory a person unzipped the viewer into is
    // not promised to be ASCII, and a log that silently failed to open over an
    // accented folder name is the absence this module exists to end.
    using WidePath = Array<wchar_t, PATH_CHARS>;

    // The same run started twice inside one second gets -2, -3... rather than
    // the second viewer truncating the first one's evidence.
    constexpr UInt32 NAME_TRIES = 9;

    Mutex lock;
    std::FILE* file = nullptr;
    Str where;
    TimePoint base = monoNow();
    Thread::id opener;
    Size written = 0;
    Bool capped = false;

    // The fast path's test, read without the lock: a viewer that could not open
    // a file must not take a mutex for every line it would have written.
    Atomic<Bool> live = false;

    thread_local CharSeq threadTag = nullptr;

    [[nodiscard]] CharSeq tagOfThisThread()
    {
        if(threadTag != nullptr)
        {
            return threadTag;
        }
        return std::this_thread::get_id() == opener ? "ui" : "thread";
    }

    // Formats into `into` and makes the result safe to write as ONE line. A
    // sentence from the board can carry a newline, and a log whose lines can be
    // split by the data inside them is a log whose timestamps that data can
    // forge - so every control character becomes a space.
    Void formatInto(Array<Char, LINE_BYTES>& into, CharSeq fmt, std::va_list args)
    {
        const Int32 need = std::vsnprintf(into.data(), into.size(), fmt, args);
        if(need < 0)
        {
            std::snprintf(into.data(), into.size(), "[unformattable line] %s", fmt);
        }
        for(Char& c : into)
        {
            if(c == '\0')
            {
                break;
            }
            if(static_cast<UInt8>(c) < 0x20u)
            {
                c = ' ';
            }
        }
        // SAID, not silently cut: a line that ends mid-sentence with nothing to
        // show for it reads as the whole sentence.
        if(need >= static_cast<Int32>(into.size()))
        {
            const Size at = into.size() - sizeof(" [truncated]");
            std::snprintf(into.data() + at, into.size() - at, "%s", " [truncated]");
        }
    }

    // `lock` must be held.
    Void writeLocked(const Array<Char, LINE_BYTES>& body)
    {
        if(file == nullptr || capped)
        {
            return;
        }

        // PRECISE rather than GetSystemTime, whose answer moves in timer ticks
        // of up to 15.6 ms - a granularity that could put two lines from
        // different ends of a round trip in the wrong order.
        FILETIME ft = {};
        ::GetSystemTimePreciseAsFileTime(&ft);
        SYSTEMTIME utc = {};
        ::FileTimeToSystemTime(&ft, &utc);
        const Int64 sinceMs = std::chrono::duration_cast<Millis>(Clock::now() - base).count();

        Array<Char, 96> head = {};
        const Int32 headLen = std::snprintf(
            head.data(),
            head.size(),
            "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ +%lldms [%s] ",
            static_cast<UInt32>(utc.wYear),
            static_cast<UInt32>(utc.wMonth),
            static_cast<UInt32>(utc.wDay),
            static_cast<UInt32>(utc.wHour),
            static_cast<UInt32>(utc.wMinute),
            static_cast<UInt32>(utc.wSecond),
            static_cast<UInt32>(utc.wMilliseconds),
            sinceMs,
            tagOfThisThread()
        );
        if(headLen <= 0)
        {
            return;
        }
        const Size headBytes = static_cast<Size>(headLen);
        const Size bodyBytes = std::strlen(body.data());
        const Size total = headBytes + bodyBytes + 1;

        if(written + total > CAP_BYTES)
        {
            capped = true;
            std::fwrite(head.data(), 1, headBytes, file);
            std::fputs("log reached 50 MB - NOTHING AFTER THIS LINE WAS WRITTEN\n", file);
            std::fflush(file);
            return;
        }

        std::fwrite(head.data(), 1, headBytes, file);
        std::fwrite(body.data(), 1, bodyBytes, file);
        std::fputc('\n', file);
        std::fflush(file);
        written += total;
    }

    [[nodiscard]] Str utf8Of(const WidePath& text)
    {
        Array<Char, PATH_CHARS * 3> out = {};
        const Int32 n = ::WideCharToMultiByte(
            CP_UTF8,
            0,
            text.data(),
            -1,
            out.data(),
            static_cast<Int32>(out.size()),
            nullptr,
            nullptr
        );
        return n > 0 ? Str(out.data()) : Str("(path not representable)");
    }

    // The directory bibo.exe was started from, without a trailing separator.
    [[nodiscard]] Bool exeDirectory(WidePath& dir)
    {
        const DWORD cap = static_cast<DWORD>(dir.size());
        const DWORD n = ::GetModuleFileNameW(nullptr, dir.data(), cap);
        // n == cap is TRUNCATION, not success - the path is longer than the
        // buffer, and cutting it at a separator would name some other directory.
        if(n == 0 || n >= cap)
        {
            return false;
        }
        for(Size i = n; i > 0; --i)
        {
            if(dir[i - 1] == L'\\' || dir[i - 1] == L'/')
            {
                dir[i - 1] = L'\0';
                return true;
            }
        }
        return false;
    }

    // A fresh file in `dir`, named for the UTC moment, and the name it got.
    [[nodiscard]] std::FILE* openIn(const WidePath& dir, const SYSTEMTIME& utc, WidePath& name)
    {
        if(::CreateDirectoryW(dir.data(), nullptr) == 0 && ::GetLastError() != ERROR_ALREADY_EXISTS)
        {
            return nullptr;
        }
        for(UInt32 attempt = 1; attempt <= NAME_TRIES; ++attempt)
        {
            Array<wchar_t, 8> suffix = {};
            if(attempt > 1)
            {
                std::swprintf(suffix.data(), suffix.size(), L"-%u", attempt);
            }
            std::swprintf(
                name.data(),
                name.size(),
                L"%ls\\viewer-%04u%02u%02u-%02u%02u%02u%ls.log",
                dir.data(),
                static_cast<UInt32>(utc.wYear),
                static_cast<UInt32>(utc.wMonth),
                static_cast<UInt32>(utc.wDay),
                static_cast<UInt32>(utc.wHour),
                static_cast<UInt32>(utc.wMinute),
                static_cast<UInt32>(utc.wSecond),
                suffix.data()
            );
            if(::GetFileAttributesW(name.data()) != INVALID_FILE_ATTRIBUTES)
            {
                continue;
            }
            // A name that does not exist and still will not open is a directory
            // that cannot be written, and trying eight more names in it would
            // only fail eight more times.
            return ::_wfopen(name.data(), L"wb");
        }
        return nullptr;
    }

    // latest.txt: the newest log's full path and nothing else, so finding it is
    // one read rather than a directory listing sorted by somebody's guess.
    Void pointLatestAt(const WidePath& dir, const Str& logPath)
    {
        WidePath latest = {};
        std::swprintf(latest.data(), latest.size(), L"%ls\\latest.txt", dir.data());
        std::FILE* f = ::_wfopen(latest.data(), L"wb");
        if(f == nullptr)
        {
            return;
        }
        std::fputs(logPath.c_str(), f);
        std::fputs("\n", f);
        std::fclose(f);
    }

  }

  Bool open()
  {
      if(live.load())
      {
          return true;
      }

      FILETIME ft = {};
      ::GetSystemTimePreciseAsFileTime(&ft);
      SYSTEMTIME utc = {};
      ::FileTimeToSystemTime(&ft, &utc);

      // Beside the exe first - viewer\build\ is gitignored, so a log there can
      // never be committed - and %TEMP% only when that directory will not take
      // a file, which is a viewer copied somewhere read-only.
      WidePath exeDir = {};
      WidePath logsDir = {};
      WidePath chosen = {};
      std::FILE* f = nullptr;
      Bool inTemp = false;

      if(exeDirectory(exeDir))
      {
          std::swprintf(logsDir.data(), logsDir.size(), L"%ls\\logs", exeDir.data());
          f = openIn(logsDir, utc, chosen);
      }
      if(f == nullptr)
      {
          WidePath temp = {};
          const DWORD n = ::GetTempPathW(static_cast<DWORD>(temp.size()), temp.data());
          if(n > 0 && n < temp.size())
          {
              std::swprintf(logsDir.data(), logsDir.size(), L"%lsbibo-viewer-logs", temp.data());
              f = openIn(logsDir, utc, chosen);
              inTemp = f != nullptr;
          }
      }
      if(f == nullptr)
      {
          return false;
      }

      const Str logPath = utf8Of(chosen);
      pointLatestAt(logsDir, logPath);

      {
          LockGuard<Mutex> held(lock);
          file = f;
          where = logPath;
          base = monoNow();
          opener = std::this_thread::get_id();
          written = 0;
          capped = false;
      }
      live.store(true);

      line("log opened: %s", logPath.c_str());
      if(inTemp)
      {
          line("the exe's own directory would not take a file, so this log is under %%TEMP%%");
      }

      // The LOCAL time once, so a person who remembers "about ten past one"
      // can find the moment - every prefix after this is UTC.
      SYSTEMTIME local = {};
      if(::SystemTimeToTzSpecificLocalTime(nullptr, &utc, &local) != 0)
      {
          line(
              "local wall clock at open %04u-%02u-%02u %02u:%02u:%02u - every prefix is UTC",
              static_cast<UInt32>(local.wYear),
              static_cast<UInt32>(local.wMonth),
              static_cast<UInt32>(local.wDay),
              static_cast<UInt32>(local.wHour),
              static_cast<UInt32>(local.wMinute),
              static_cast<UInt32>(local.wSecond)
          );
      }
      line("pid %u", static_cast<UInt32>(::GetCurrentProcessId()));
      return true;
  }

  Void close()
  {
      if(!live.load())
      {
          return;
      }
      line("log closed");
      LockGuard<Mutex> held(lock);
      if(file != nullptr)
      {
          std::fclose(file);
          file = nullptr;
      }
      live.store(false);
  }

  Void nameThread(CharSeq tag)
  {
      threadTag = tag;
  }

  Void line(CharSeq fmt, ...)
  {
      if(!live.load())
      {
          return;
      }
      // Formatted OUTSIDE the lock, so one thread's long line never holds the
      // other thread's short one - the worker must not wait on the UI's text.
      Array<Char, LINE_BYTES> body = {};
      std::va_list args;
      va_start(args, fmt);
      formatInto(body, fmt, args);
      va_end(args);

      LockGuard<Mutex> held(lock);
      writeLocked(body);
  }

  Void append(Str& out, CharSeq fmt, ...)
  {
      Array<Char, LINE_BYTES> part = {};
      std::va_list args;
      va_start(args, fmt);
      formatInto(part, fmt, args);
      va_end(args);
      out += part.data();
  }

  Bool isOpen()
  {
      return live.load();
  }

  Str path()
  {
      LockGuard<Mutex> held(lock);
      return where;
  }

}
