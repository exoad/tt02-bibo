#include "applog.hxx"

#include "pico_flash.hxx"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace applog
{
  namespace
  {

    // How many session logs to keep. Twenty is a few days of ordinary use and about
    // a megabyte; the alternative is a directory that grows without limit for the
    // rest of the project's life.
    constexpr Size MAX_SESSIONS = 20;

    Mutex mu;
    FILE* file = nullptr;
    Bool  started = false;
    Str   filePath;

    const Char* levelName(Level l) noexcept
    {
        switch(l)
        {
        case Level::LEVEL_DEBUG: return "DEBUG";
        case Level::LEVEL_INFO:  return "INFO ";
        case Level::LEVEL_WARN:  return "WARN ";
        case Level::LEVEL_ERROR: return "ERROR";
        default:                 return "?????";
        }
    }

    // Milliseconds since init(), so a reader can see how long things took without
    // subtracting wall-clock timestamps by hand.
    Int64 startTicks = 0;

    Int64 nowMs()
    {
        LARGE_INTEGER f;
        LARGE_INTEGER c;
        if(!::QueryPerformanceFrequency(&f) || !::QueryPerformanceCounter(&c) || f.QuadPart == 0)
        {
            return 0;
        }
        return (c.QuadPart * 1000) / f.QuadPart;
    }

    // Deletes the oldest sessions past MAX_SESSIONS. Names are timestamped and
    // fixed-width, so sorting them as text sorts them by time.
    Void prune(const Str& d)
    {
        Vec<Str> names;

        WIN32_FIND_DATAA fd = {};
        HANDLE h = ::FindFirstFileA((d + "\\session-*.log").c_str(), &fd);
        if(h == INVALID_HANDLE_VALUE)
        {
            return;
        }
        do
        {
            if((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
            {
                names.push_back(fd.cFileName);
            }
        }
        while(::FindNextFileA(h, &fd) != 0);
        ::FindClose(h);

        if(names.size() <= MAX_SESSIONS)
        {
            return;
        }

        std::sort(names.begin(), names.end());
        const Size drop = names.size() - MAX_SESSIONS;
        for(Size i = 0; i < drop; ++i)
        {
            ::DeleteFileA((d + "\\" + names[i]).c_str());
        }
    }

  }

  Str dir()
  {
      // Beside the source, not in %LOCALAPPDATA%, and deliberately: a log you have
      // to go hunting for is a log nobody attaches to a bug report. It is
      // gitignored, so it never leaves the machine either way.
      const Str root = PicoFlash::repoRoot();
      if(root.empty())
      {
          return Str();
      }

      const Str d = root + "\\logs";
      if(!::CreateDirectoryA(d.c_str(), nullptr)
         && ::GetLastError() != ERROR_ALREADY_EXISTS)
      {
          return Str();
      }
      return d;
  }

  Void init()
  {
      LockGuard<Mutex> lk(mu);
      if(started)
      {
          return;
      }
      started = true;
      startTicks = nowMs();

      const Str d = dir();
      if(d.empty())
      {
          return;   // no repo root: run without a log rather than not at all
      }

      prune(d);

      SYSTEMTIME t = {};
      ::GetLocalTime(&t);

      Array<Char, 64> name;
      std::snprintf(
          name.data(),
          name.size(),
          "session-%04d%02d%02d-%02d%02d%02d.log",
          t.wYear,
          t.wMonth,
          t.wDay,
          t.wHour,
          t.wMinute,
          t.wSecond
      );

      filePath = d + "\\" + name.data();
      file = std::fopen(filePath.c_str(), "wb");
      if(file == nullptr)
      {
          filePath.clear();
          return;
      }

      std::fprintf(file, "bibo session log\n");
      std::fprintf(
          file,
          "started %04d-%02d-%02d %02d:%02d:%02d local\n",
          t.wYear,
          t.wMonth,
          t.wDay,
          t.wHour,
          t.wMinute,
          t.wSecond
      );
      std::fprintf(
          file,
          "command line: %s\n",
          (::GetCommandLineA() != nullptr) ? ::GetCommandLineA() : "?"
      );
      std::fprintf(file, "%s\n", Str(70, '-').c_str());
      std::fflush(file);
  }

  Void shutdown()
  {
      LockGuard<Mutex> lk(mu);
      if(file != nullptr)
      {
          std::fprintf(file, "%s\n", Str(70, '-').c_str());
          std::fprintf(
              file,
              "[%8.3f] INFO  app    session ended\n",
              static_cast<Float64>(nowMs() - startTicks) / 1000.0
          );
          std::fclose(file);
          file = nullptr;
      }
  }

  Void writef(Level level, const Char* tag, const Char* fmt, ...)
  {
      LockGuard<Mutex> lk(mu);
      if(file == nullptr)
      {
          return;
      }

      Array<Char, 1024> body;
      va_list ap;
      va_start(ap, fmt);
      std::vsnprintf(body.data(), body.size(), fmt, ap);
      va_end(ap);

      // Seconds since start, then level, then a fixed-width tag. Fixed width so
      // the message column lines up and the file is scannable rather than merely
      // readable.
      std::fprintf(
          file,
          "[%8.3f] %s %-6s %s\n",
          static_cast<Float64>(nowMs() - startTicks) / 1000.0,
          levelName(level),
          (tag != nullptr) ? tag : "-",
          body.data()
      );

      // Anything that went wrong is flushed at once. A crash after a warning must
      // still leave the warning on disk - that is most of the value here.
      if(level >= Level::LEVEL_WARN)
      {
          std::fflush(file);
      }
  }

  Str path()
  {
      LockGuard<Mutex> lk(mu);
      return filePath;
  }

}
