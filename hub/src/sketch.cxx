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

  // The library's folders, in dependency order rather than alphabetical: the
  // order a person should read them in, which is also the order they may include
  // each other in.
  //
  // A fixed list rather than a directory walk. The layout is the architecture -
  // if a folder appears that is not here, that is a decision somebody made and it
  // should be a decision somebody writes down, not something a scan quietly
  // absorbs.
  constexpr const Char* const FW_DIRS[] = {
      "lib",
      "lib\\drivers",
      "lib\\chassis",
      "app",
      "sketches",
      "docs",
      "docs\\pinouts",
  };

  Vec<Str> listFirmware()
  {
      Vec<Str> out;

      const Str root = firmwareDir();
      if(root.empty())
      {
          return out;
      }

      for(const Char* sub : FW_DIRS)
      {
          const Str d = root + "\\" + sub;

          // .bdoc joins the list because firmware/docs holds the reference
          // documents and they are edited here like anything else. See
          // hub/src/refdoc.hxx - the Code view renders them, and the toggle above
          // the editor swaps between the page and the source that made it.
          constexpr const Char* const PATTERNS[] = { "\\*.cxx", "\\*.hxx",
                                           "\\*.c", "\\*.h",
                                           "\\*.bdoc" };
          for(const Char* pat : PATTERNS)
          {
              WIN32_FIND_DATAA fd = {};
              HANDLE           h = ::FindFirstFileA((d + pat).c_str(), &fd);
              if(h == INVALID_HANDLE_VALUE)
              {
                  continue;
              }
              do
              {
                  if((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
                  {
                      // Relative to the firmware root, so the caller's
                      // firmwareDir() + "\\" + name still resolves - and so the
                      // folder a file lives in is visible in the picker rather
                      // than being something you have to already know.
                      out.push_back(Str(sub) + "\\" + fd.cFileName);
                  }
              }
              while(::FindNextFileA(h, &fd) != 0);
              ::FindClose(h);
          }
      }

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
      for(Size i = 0; i < out.size(); ++i)
      {
          if(out[i] != '\r')
          {
              lf.push_back(out[i]);
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
      const Size got  = want > 0 ? std::fwrite(text.data(), 1, want, f) : 0;
      const Bool ok   = (got == want);

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

  Void reveal(const Str& path)
  {
      if(path.empty())
      {
          return;
      }
      // /select, highlights the file itself rather than merely opening the folder,
      // which is what somebody right-clicking a file is asking for.
      const Str arg = "/select,\"" + path + "\"";
      ::ShellExecuteA(nullptr, "open", "explorer.exe", arg.c_str(), nullptr,
                      SW_SHOWNORMAL);
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
      ::ShellExecuteA(nullptr, "open", bat.c_str(), nullptr, dir.c_str(),
                      SW_SHOWMINNOACTIVE);
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
      // manbox style throughout: the types.hxx aliases, Allman braces, `while(...)`
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
