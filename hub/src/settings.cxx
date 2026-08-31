#include "settings.hxx"

#include <windows.h>

#include <cstdio>

namespace settings
{

  Str dir()
  {
      static Str cached;
      static Bool tried = false;

      if(tried)
      {
          return cached;
      }
      tried = true;

      Array<Char, MAX_PATH> buf= {};
      if(::GetEnvironmentVariableA("LOCALAPPDATA", buf.data(), MAX_PATH) == 0)
      {
          return cached;                  // no profile: run without persistence
      }

      cached = Str(buf.data()) + "\\bibo";

      // ---- the car got a name, and the folder follows it -------------------
      //
      // MOVED, not abandoned. This folder holds the window layout, the car's
      // saved address, the steering calibration and every recording ever made,
      // and simply pointing at a new empty directory would look exactly like the
      // program having lost all of it.
      //
      // Once, and only into a name that is not already taken: if \bibo exists
      // this does nothing, so a second machine, a restored backup, or a person
      // who moved it by hand is never overwritten.
      {
          const Str older = Str(buf.data()) + "\\tt02-auto";
          if(::GetFileAttributesA(cached.c_str()) == INVALID_FILE_ATTRIBUTES
             && ::GetFileAttributesA(older.c_str()) != INVALID_FILE_ATTRIBUTES)
          {
              ::MoveFileA(older.c_str(), cached.c_str());
          }
      }

      // ERROR_ALREADY_EXISTS is the expected case on every run but the first.
      if(!::CreateDirectoryA(cached.c_str(), nullptr)
         && ::GetLastError() != ERROR_ALREADY_EXISTS)
      {
          cached.clear();
      }

      return cached;
  }

  Str path(const Char* name)
  {
      const Str d = dir();
      return d.empty() ? Str() : (d + "\\" + name);
  }

  Str read(const Char* name)
  {
      Str out;

      const Str p = path(name);
      if(p.empty())
      {
          return out;
      }

      FILE* f = std::fopen(p.c_str(), "rb");
      if(f == nullptr)
      {
          return out;
      }

      Array<Char, 1024> buf;
      for(;;)
      {
          const Size n = std::fread(buf.data(), 1, buf.size(), f);
          if(n == 0)
          {
              break;
          }
          out.append(buf.data(), n);

          // A settings file is a few hundred bytes. Anything larger is a file
          // that does not belong here, and reading it all would be the bug.
          if(out.size() > 64u * 1024u)
          {
              break;
          }
      }
      std::fclose(f);
      return out;
  }

  Void write(const Char* name, const Str& text)
  {
      const Str p = path(name);
      if(p.empty())
      {
          return;
      }

      FILE* f = std::fopen(p.c_str(), "wb");
      if(f == nullptr)
      {
          return;
      }

      if(!text.empty())
      {
          static_cast<Void>(std::fwrite(text.data(), 1, text.size(), f));
      }
      std::fclose(f);
  }

} // namespace settings
