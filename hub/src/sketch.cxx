#include "sketch.hxx"

#include "pico_flash.hxx"
#include "settings.hxx"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>

#include <algorithm>
#include <cstdio>

namespace sketch
{

  Str dir()
  {
      // IN THE REPO, not in %LOCALAPPDATA%. It was the latter, and that is what
      // made a sketch feel disposable: it was not in git, not in a clone, not in
      // a backup and not in the file tree next to the code it was written to
      // test. A finding you cannot find later is not a finding.
      const Str root = PicoFlash::repoRoot();
      if(root.empty())
      {
          return Str();
      }

      const Str d = root + "\\firmware\\sketches";
      if(!::CreateDirectoryA(d.c_str(), nullptr)
         && ::GetLastError() != ERROR_ALREADY_EXISTS)
      {
          return Str();
      }

      return d;
  }

  Vec<Str> list()
  {
      Vec<Str> out;

      const Str d = dir();
      if(d.empty())
      {
          return out;
      }

      WIN32_FIND_DATAA fd = {};
      // *.cxx AND *.c. The library is C++ now, but sketches written before that
      // are still on disk and still the user's - listing only the new extension
      // would make them vanish from a view they were saved in.
      //
      // Only .cxx gets a CMake target, though: the glob in firmware/CMakeLists.txt
      // is *.cxx. A stray .c here is listed and editable and will not build, which
      // is the honest answer - inventing a target for it would produce an image
      // nobody asked for.
      const Str        pattern = d + "\\*.c*";
      HANDLE           h = ::FindFirstFileA(pattern.c_str(), &fd);
      if(h == INVALID_HANDLE_VALUE)
      {
          return out;
      }

      do
      {
          if((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
          {
              out.push_back(fd.cFileName);
          }
      }
      while(::FindNextFileA(h, &fd) != 0);
      ::FindClose(h);

      // Alphabetical, unlike recordings. These names are chosen by a person
      // rather than stamped with a time, so "newest first" would shuffle the list
      // every time one is saved and nobody would find anything twice.
      std::sort(out.begin(), out.end());
      return out;
  }

  Str pathOf(const Str& name)
  {
      const Str d = dir();
      if(d.empty() || name.empty())
      {
          return Str();
      }
      return d + "\\" + name;
  }

  Str firmwareDir()
  {
      const Str root = PicoFlash::repoRoot();
      if(root.empty())
      {
          return Str();
      }
      return root + "\\firmware";
  }

  // ---------------------------------------------------------------------------
  // A WALK, NOT A LIST.
  //
  // This was a fixed array of folder names, on the argument that the layout is
  // the architecture and a new folder should be a decision somebody writes down
  // rather than something a scan quietly absorbs. That reasoning was sound right
  // up until the Code view grew a right-click menu that can CREATE folders: a
  // fixed list means the folder you just made does not appear in the tree you
  // made it from, which reads as the command having silently failed.
  //
  // So the tree now shows what is on disk. What keeps it honest is the skip list
  // below rather than an allow list - the difference being that a new source
  // folder shows up on its own and a new build directory still does not.
  // ---------------------------------------------------------------------------

  namespace
  {

    // Directories never descended into. Build output, tool state and vendored
    // trees - none of them are things anybody edits here, and pico-sdk alone is
    // tens of thousands of files that would swamp the tree and the rescan.
    [[nodiscard]] Bool skipDir(const Char* name)
    {
        // Anything dotted: .git, .vs, .idea, .cate. One rule rather than six.
        if(name[0] == '.')
        {
            return true;
        }

        constexpr const Char* const SKIP[] = {
            "build", "vendor", "pico-sdk", "node_modules", "__pycache__",
        };
        for(const Char* s : SKIP)
        {
            if(_stricmp(name, s) == 0)
            {
                return true;
            }
        }

        // build-pico2, build-whatever: one per PICO_BOARD, all of them output.
        return _strnicmp(name, "build-", 6) == 0;
    }

  }

  // The extensions the Code view shows and creates. AN ALLOW LIST, and a short
  // one: this is a C/C++ editor, and the first version of the walk used a deny
  // list on the theory that anything you create should appear - which let 29
  // .bat, .ps1, .md, .json and .cmake files into a tree meant for source. The
  // menu now refuses to create anything outside this list instead, so the two
  // agree and nothing you make can vanish.
  //
  // .bdoc is here because firmware/docs holds the reference documents and they
  // are edited here like anything else - see hub/src/refdoc.hxx.
  constexpr const Char* const SHOWN_EXT[] = { ".cxx", ".hxx", ".c", ".h", ".bdoc" };

  [[nodiscard]] Bool shownFile(const Str& name)
  {
      const Size dot = name.rfind('.');
      if(dot == Str::npos)
      {
          return false;
      }
      const Str ext = name.substr(dot);
      for(const Char* s : SHOWN_EXT)
      {
          if(_stricmp(ext.c_str(), s) == 0)
          {
              return true;
          }
      }
      return false;
  }

  namespace
  {

    // Guards against a directory symlink loop turning the rescan into a hang.
    // Nothing in this tree is anywhere near this deep.
    constexpr Int32 WALK_MAX_DEPTH = 12;

    Void walk(const Str& root, const Str& rel, Int32 depth, Vec<Str>& out)
    {
        if(depth > WALK_MAX_DEPTH)
        {
            return;
        }

        const Str dir = rel.empty() ? root : (root + "\\" + rel);

        WIN32_FIND_DATAA fd = {};
        HANDLE           h = ::FindFirstFileA((dir + "\\*").c_str(), &fd);
        if(h == INVALID_HANDLE_VALUE)
        {
            return;
        }

        do
        {
            const Str name = fd.cFileName;
            if(name == "." || name == "..")
            {
                continue;
            }

            if((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
            {
                if(!skipDir(fd.cFileName))
                {
                    walk(root, rel.empty() ? name : (rel + "\\" + name), depth + 1, out);
                }
                continue;
            }

            if(shownFile(name))
            {
                // Relative to the firmware root, so the caller's firmwareDir() +
                // "\\" + name still resolves - and so the folder a file lives in
                // is visible in the picker rather than being something you have
                // to already know.
                out.push_back(rel.empty() ? name : (rel + "\\" + name));
            }
        }
        while(::FindNextFileA(h, &fd) != 0);

        ::FindClose(h);
    }

  }

  Vec<Str> listFirmware()
  {
      Vec<Str> out;

      const Str root = firmwareDir();
      if(root.empty())
      {
          return out;
      }

      walk(root, Str(), 0, out);
      return out;
  }

  namespace
  {

    // The bare filename with its extension removed - "range-view" out of any of
    // C:\...\firmware\sketches\range-view.cxx, sketches\range-view.cxx or
    // range-view.cxx. This is the CMake target name, because
    // get_filename_component(... NAME_WE) computes exactly the same thing from the
    // same file. Two implementations of one rule, which is a risk; the alternative
    // was a hand-kept table mapping files to targets, which is a worse one.
    [[nodiscard]] Str stemOf(const Str& path)
    {
        Size begin = 0;
        for(Size i = 0; i < path.size(); ++i)
        {
            if(path[i] == '\\' || path[i] == '/')
            {
                begin = i + 1;
            }
        }

        Size end = path.size();
        for(Size i = path.size(); i > begin; --i)
        {
            if(path[i - 1] == '.')
            {
                end = i - 1;
                break;
            }
        }

        return path.substr(begin, end - begin);
    }

  }

  Str targetFor(const Str& path)
  {
      if(path.empty())
      {
          return "pico_debug";
      }

      // A sketch owns a target named after its own file. This used to answer
      // "sketch" for every one of them, because there was one target and the
      // Code view copied whichever file you had open into it first - so the
      // answer was right by making the question meaningless.
      const Str sk = dir();
      if(!sk.empty() && path.size() > sk.size()
         && _strnicmp(path.c_str(), sk.c_str(), sk.size()) == 0
         && (path[sk.size()] == '\\' || path[sk.size()] == '/'))
      {
          return stemOf(path);
      }

      // Everything else under firmware/ belongs to the debug image. The library
      // headers are compiled into BOTH images, and naming pico_debug for them is
      // the safe answer: it is the target a person editing the chassis or the HAL
      // is almost certainly testing.
      return "pico_debug";
  }

  Str load(const Str& path)
  {
      if(path.empty())
      {
          return Str();
      }

      FILE* f = std::fopen(path.c_str(), "rb");
      if(f == nullptr)
      {
          return Str();
      }

      Str  out;
      Array<Char, 4096> buf;
      Size n = 0;
      while((n = std::fread(buf.data(), 1, buf.size(), f)) > 0)
      {
          out.append(buf.data(), n);
      }
      std::fclose(f);

      // CRLF in, LF held internally. The editor works in LF and writes LF back;
      // MSVC, gcc and every other tool in this chain read LF fine on Windows.
      Str lf;
      lf.reserve(out.size());
      for(const Char c : out)
      {
          if(c != '\r')
          {
              lf.push_back(c);
          }
      }

      return lf;
  }

  Bool save(const Str& path, const Str& text, Str& err)
  {
      err.clear();

      if(path.empty())
      {
          err = "no path (is %LOCALAPPDATA% set?)";
          return false;
      }

      FILE* f = std::fopen(path.c_str(), "wb");
      if(f == nullptr)
      {
          err = "cannot open " + path + " for writing";
          return false;
      }

      const Size want = text.size();
      const Size got = want > 0 ? std::fwrite(text.data(), 1, want, f) : 0;
      const Bool ok = (got == want);

      if(std::fclose(f) != 0 || !ok)
      {
          err = "write to " + path + " failed";
          return false;
      }
      return true;
  }

  UInt64 stamp(const Str& path)
  {
      if(path.empty())
      {
          return 0;
      }

      WIN32_FILE_ATTRIBUTE_DATA fad = {};
      if(!::GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &fad))
      {
          return 0;
      }
      return (static_cast<UInt64>(fad.ftLastWriteTime.dwHighDateTime) << 32)
           | static_cast<UInt64>(fad.ftLastWriteTime.dwLowDateTime);
  }

  Int64 modifiedAtUnix(const Str& path)
  {
      const UInt64 ft = stamp(path);
      if(ft == 0)
      {
          return 0;
      }

      // FILETIME counts 100 ns ticks from 1601-01-01; Unix time counts seconds
      // from 1970-01-01. 11644473600 is the gap between the two epochs.
      return static_cast<Int64>(ft / 10000000ULL) - 11644473600LL;
  }

  Bool remove(const Str& path)
  {
      if(path.empty())
      {
          return false;
      }
      return ::DeleteFileA(path.c_str()) != 0;
  }

  // ---------------------------------------------------------------------------
  // What the Code view's right-click menu needs.
  //
  // Every one of these takes a name the user typed, which is the reason
  // validName() exists and is called first in all of them. A tree that hands an
  // unchecked string to CreateDirectory lets "..\..\hub\src" be a folder name,
  // and the resulting file lands somewhere the person who typed it will never
  // look for it.
  // ---------------------------------------------------------------------------

  Bool validName(const Str& name, Str& err)
  {
      if(name.empty())
      {
          err = "a name is required";
          return false;
      }
      if(name.size() > 200)
      {
          err = "that name is too long";
          return false;
      }

      // Separators and traversal, which is the whole point of this function. A
      // name is a NAME - a path is the caller's business, assembled from a
      // folder it already knows.
      if(name.find('\\') != Str::npos || name.find('/') != Str::npos)
      {
          err = "a name cannot contain a path separator";
          return false;
      }
      if(name == "." || name == "..")
      {
          err = "that is not a name";
          return false;
      }

      // The characters Windows refuses outright, named here so the message says
      // which one rather than letting CreateFile fail with a number.
      for(const Char c : name)
      {
          if(c == ':' || c == '*' || c == '?' || c == '"' || c == '<' || c == '>'
             || c == '|')
          {
              err = Str("a name cannot contain ") + c;
              return false;
          }
          if(static_cast<UInt8>(c) < 32u)
          {
              err = "a name cannot contain control characters";
              return false;
          }
      }

      // Trailing dots and spaces are accepted by the API and then unopenable,
      // which is a worse outcome than refusing them here.
      if(name.back() == '.' || name.back() == ' ')
      {
          err = "a name cannot end with a dot or a space";
          return false;
      }

      return true;
  }

  Bool createFile(const Str& path, Str& err)
  {
      if(path.empty())
      {
          err = "no path";
          return false;
      }

      // CREATE_NEW, so an existing file is an ERROR rather than being truncated.
      // The menu's job is to make a new file; silently emptying one somebody
      // already had would be the worst possible reading of "New File".
      const HANDLE h = ::CreateFileA(
          path.c_str(),
          GENERIC_WRITE,
          0,
          nullptr,
          CREATE_NEW,
          FILE_ATTRIBUTE_NORMAL,
          nullptr
      );
      if(h == INVALID_HANDLE_VALUE)
      {
          err = (::GetLastError() == ERROR_FILE_EXISTS) ? "that file already exists"
                                                        : "could not create that file";
          return false;
      }
      ::CloseHandle(h);
      return true;
  }

  Bool createDir(const Str& path, Str& err)
  {
      if(path.empty())
      {
          err = "no path";
          return false;
      }
      if(::CreateDirectoryA(path.c_str(), nullptr) == 0)
      {
          err = (::GetLastError() == ERROR_ALREADY_EXISTS) ? "that folder already exists"
                                                           : "could not create that folder";
          return false;
      }
      return true;
  }

  Bool rename(const Str& from, const Str& to, Str& err)
  {
      if(from.empty() || to.empty())
      {
          err = "no path";
          return false;
      }

      // NO REPLACE FLAG. MoveFileEx with MOVEFILE_REPLACE_EXISTING would let a
      // rename silently destroy the file it landed on, and the one thing a
      // rename must never do is lose the other file.
      if(::MoveFileA(from.c_str(), to.c_str()) == 0)
      {
          const DWORD e = ::GetLastError();
          err = (e == ERROR_ALREADY_EXISTS || e == ERROR_FILE_EXISTS)
                  ? "something with that name is already there"
                  : "could not rename it";
          return false;
      }
      return true;
  }

  Bool removeDir(const Str& path, Str& err)
  {
      if(path.empty())
      {
          err = "no path";
          return false;
      }

      // EMPTY ONLY. Recursively deleting a directory tree from a right-click is
      // a great deal of destruction behind one small menu entry, and the undo is
      // "restore it from git, if it was ever in git". Refusing a non-empty
      // folder means the person has to look at what is inside it first.
      if(::RemoveDirectoryA(path.c_str()) == 0)
      {
          const DWORD e = ::GetLastError();
          err = (e == ERROR_DIR_NOT_EMPTY) ? "that folder is not empty"
                                           : "could not remove that folder";
          return false;
      }
      return true;
  }

  Bool isDir(const Str& path)
  {
      if(path.empty())
      {
          return false;
      }
      const DWORD a = ::GetFileAttributesA(path.c_str());
      return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY) != 0;
  }

  Bool formatFile(const Str& path, Str& err)
  {
      if(path.empty())
      {
          err = "no file";
          return false;
      }

      const Str root = PicoFlash::repoRoot();
      if(root.empty())
      {
          err = "repo root not found";
          return false;
      }

      // CreateProcess writes into its command line, so it cannot be a literal.
      Str cmd = "python \"" + root + "\\tools\\format.py\" --apply \"" + path + "\"";
      Vec<Char> line(cmd.begin(), cmd.end());
      line.push_back('\0');

      STARTUPINFOA        si = {};
      PROCESS_INFORMATION pi = {};
      si.cb = sizeof(si);

      // No console window: this runs from a GUI on every :format, and a black
      // box flashing over the editor for a quarter of a second reads as a crash.
      if(::CreateProcessA(
          nullptr,
          line.data(),
          nullptr,
          nullptr,
          FALSE,
          CREATE_NO_WINDOW,
          nullptr,
          root.c_str(),
          &si,
          &pi
      ) == 0)
      {
          err = "could not start python - is it on PATH?";
          return false;
      }

      const DWORD waited = ::WaitForSingleObject(pi.hProcess, 15000);
      DWORD       code = 1;
      if(waited == WAIT_TIMEOUT)
      {
          // KILLED, not abandoned. A formatter left running past the timeout
          // would write the file AFTER the editor had reloaded it, and the
          // buffer and the disk would quietly disagree from then on.
          ::TerminateProcess(pi.hProcess, 1);
      }
      else
      {
          ::GetExitCodeProcess(pi.hProcess, &code);
      }
      ::CloseHandle(pi.hThread);
      ::CloseHandle(pi.hProcess);

      if(waited == WAIT_TIMEOUT)
      {
          err = "formatter did not finish and was stopped";
          return false;
      }
      if(code != 0)
      {
          // The script refuses to write when its token check fails, and says so
          // on stdout; that text is not captured here. The one fact that matters
          // reaches the user: the file was NOT touched.
          err = "formatter refused the file - run tools\\format.py to see why";
          return false;
      }
      return true;
  }

  Void reveal(const Str& path)
  {
      if(path.empty())
      {
          return;
      }
      // /select, highlights the file itself rather than merely opening the folder,
      // which is what somebody right-clicking a file is asking for.
      const Str arg = "/select,\"" + path + "\"";
      ::ShellExecuteA(nullptr, "open", "explorer.exe", arg.c_str(), nullptr, SW_SHOWNORMAL);
  }

  Void openDocs()
  {
      const Str root = PicoFlash::repoRoot();
      if(root.empty())
      {
          return;
      }

      const Str bat = root + "\\website\\docs.bat";
      const Str dir = root + "\\website";

      // SW_SHOWMINNOACTIVE, because the console is a means and not the thing
      // asked for. It is instant once the site has been built, and the browser
      // it launches is what should take the foreground.
      //
      // The working directory is passed explicitly rather than relying on the
      // batch file's own `cd /d %~dp0`. Both are correct; having both means a
      // relative path inside it cannot resolve against hub\build\ if that line
      // is ever edited out.
      ::ShellExecuteA(nullptr, "open", bat.c_str(), nullptr, dir.c_str(), SW_SHOWMINNOACTIVE);
  }

  Str starter()
  {
      // A STARTER PROGRAM IS THE STRONGEST STYLE DOCUMENT A PROJECT HAS, because
      // it is the one everybody copies - and that cuts both ways. This one has
      // been wrong twice: it included "pico2w.h" after the library stopped using
      // that name, so a new sketch did not build at all, and it emitted
      // #include "bibo.hxx" while the layering rule for firmware/sketches wants
      // "../lib/bibo.hxx", so every new sketch failed the style audit on line one.
      //
      // IT SHOWS THE THREE PHASES every program in this firmware is written in,
      // because a template that skipped them would teach the opposite:
      //
      //   1 DECLARE  what is wired where - pins::begin(), before anything opens
      //   2 BIND     hand low-level parts to high-level ones - a tft::Screen to
      //              a gfx::Canvas, an i2c bus to the device sitting on it
      //   3 RUN      the loop
      //
      // A blink has nothing to bind, and the template says so rather than
      // inventing a binding to demonstrate. Pretending otherwise would be worse
      // than leaving the phase out.
      //
      // manbox style throughout: the shared.hxx aliases, Allman braces, `while(...)`
      // with no space, SCREAMING_SNAKE macros, named casts, and the namespaces the
      // library now has.
      return
          "/*\n"
          " * Blink an LED on a breadboard.\n"
          " *\n"
          " * WIRING - GP28 is physical pin 34, ground is physical pin 38:\n"
          " *\n"
          " *   GP28 (34) --[ 220R-1k ]-- LED long leg\n"
          " *                             LED short leg -- GND (38)\n"
          " *\n"
          " * The resistor can go on either side of the LED. Without one the LED\n"
          " * is a short across a 3.3 V pin and both are at risk.\n"
          " *\n"
          " * GP28 is free. lib/pins.hxx is the car's map and the place to look\n"
          " * before borrowing a pad: GP0/GP1 servo and ESC, GP4/GP5 I2C, GP9 the\n"
          " * DFPlayer's BUSY, GP10-13 lamps on the ToF XSHUT lines, GP14/GP15 the\n"
          " * DFPlayer's UART, GP16-21 the SPI display, GP22 the SD card.\n"
          " *\n"
          " * bibo.hxx is the whole library. Every module is a namespace - type\n"
          " * gpio::, serial::, led::, pins::, gfx:: and the completion list will\n"
          " * tell you the rest.\n"
          " */\n"
          "\n"
          "// \"../lib/bibo.hxx\", not \"bibo.hxx\". Both compile - the include path\n"
          "// carries lib/ - but the layering check in hub/tools/style_audit.py\n"
          "// wants the explicit one, and a sketch that fails the audit on its\n"
          "// first line is a bad way to start.\n"
          "#include \"../lib/bibo.hxx\"\n"
          "\n"
          "// The library lives in namespace bibo. This opens it, so the calls\n"
          "// below are gpio::write rather than bibo::gpio::write. A sketch is\n"
          "// one file and links nothing else, so it can afford that.\n"
          "using namespace bibo;\n"        "\n"
          "#define LED_PIN 28\n"
          "#define DELAY_MS 400\n"
          "\n"
          "// `int`, not Int32: C++ requires main to return literally int, and on\n"
          "// this toolchain Int32 is a different type with the same shape.\n"
          "int main(Void)\n"
          "{\n"
          "    // FIRST, and in every sketch you write, even one that prints\n"
          "    // nothing. This starts the USB stack; without it the board runs\n"
          "    // fine and never enumerates, and the only way to flash it again is\n"
          "    // holding BOOTSEL while plugging the cable in.\n"
          "    serial::open();\n"
          "\n"
          "    // ================================================== 1 DECLARE\n"
          "    //\n"
          "    // What is wired where, before anything is opened. Every driver in\n"
          "    // this library reads the installed map rather than holding pin\n"
          "    // numbers, and the map starts EMPTY - so a program that skips this\n"
          "    // opens a display, a servo or a UART on nothing and looks broken\n"
          "    // for no visible reason.\n"
          "    //\n"
          "    // A blink needs no role from the map: a breadboard LED is not part\n"
          "    // of the car, so its pad is a plain constant above. The map is\n"
          "    // installed anyway, because the moment you use a DRIVER - tft, the\n"
          "    // DFPlayer, the chassis - it takes its pads from here and nowhere\n"
          "    // else. Start from pins::car() instead if you want the car's\n"
          "    // wiring, and override the fields you are moving.\n"
          "    pins::Map wiring;\n"
          "\n"
          "    if(!pins::begin(wiring))\n"
          "    {\n"
          "        // Two roles on one pad. It names both rather than saying\n"
          "        // \"pin conflict\", because the pad alone sends you back to\n"
          "        // the file to work out which two things wanted it.\n"
          "        serial::printf(\"ERR pins %s and %s both want GP%d\\n\",\n"
          "                       pins::conflictFirst(),\n"
          "                       pins::conflictSecond(),\n"
          "                       pins::conflictPin());\n"
          "        return 1;\n"
          "    }\n"
          "\n"
          "    // ===================================================== 2 BIND\n"
          "    //\n"
          "    // Where a low-level part is handed to the high-level one that uses\n"
          "    // it - a tft::Screen to a gfx::Canvas, an i2c bus to the sensor on\n"
          "    // it. A blink has nothing to bind, so this phase is empty here.\n"
          "    // See firmware/sketches/range-view.cxx for one that is not.\n"
          "\n"
          "    // ====================================================== 3 RUN\n"
          "    gpio::open(LED_PIN, PIN_DIR_OUT);\n"
          "\n"
          "    // The onboard LED is on the wireless chip rather than a GPIO, so it\n"
          "    // has to be brought up first - and that can fail. If it does, the\n"
          "    // breadboard LED still blinks.\n"
          "    const Bool haveOnboard = led::open();\n"
          "\n"
          "    while(true)\n"
          "    {\n"
          "        gpio::write(LED_PIN, true);\n"
          "        if(haveOnboard)\n"
          "        {\n"
          "            led::write(false);\n"
          "        }\n"
          "        timing::ms(DELAY_MS);\n"
          "\n"
          "        gpio::write(LED_PIN, false);\n"
          "        if(haveOnboard)\n"
          "        {\n"
          "            led::write(true);\n"
          "        }\n"
          "        timing::ms(DELAY_MS);\n"
          "    }\n"
          "\n"
          "    return 0;\n"
          "}\n";
  }

  Str makeName()
  {
      const Vec<Str> have = list();

      auto taken = [&have](const Str& n)
      {
          for(const Str& h : have)
          {
              if(_stricmp(h.c_str(), n.c_str()) == 0)
              {
                  return true;
              }
          }
          return false;
      };

      if(!taken("sketch.cxx"))
      {
          return "sketch.cxx";
      }

      for(Int32 i = 2; i < 1000; ++i)
      {
          Array<Char, 32> buf;
          std::snprintf(buf.data(), buf.size(), "sketch-%d.cxx", i);
          if(!taken(buf.data()))
          {
              return Str(buf.data());
          }
      }
      return "sketch.cxx";
  }

}
