#include "settings.hxx"
#include "vlog.hxx"

// Included after every header of ours: settings.hxx pulls in bibowire, whose
// Severity::SEVERITY_ERROR <winnt.h> would turn into a number. A header of ours
// below this line would need main.cxx's #undef block with it.
#include <windows.h>

#include <cstdio>
#include <cwchar>

namespace settings
{
  namespace
  {
    constexpr Size PATH_CHARS = 1024;

    // Wide, because the folder the viewer was unzipped into may not be ASCII.
    using WidePath = Array<wchar_t, PATH_CHARS>;

    constexpr const wchar_t* FILE_NAME = L"bibo-viewer-settings.ini";

    // A real file is a few hundred bytes; a larger one is not ours and is not read.
    constexpr Size MAX_FILE_BYTES = Size{64} * 1024;

    // Notepad can prepend one, and the first key would then silently not match.
    constexpr StrView UTF8_BOM = "\xEF\xBB\xBF";

    // The one list of keys, used by both toText and fromText, so a field cannot
    // be written and read under different spellings.
    struct Field
    {
        CharSeq key;
        Int32 Values::* member;
    };

    constexpr Array<Field, VALUE_COUNT> FIELDS = { {
        { "trim.steerMinUs", &Values::steerMinUs },
        { "trim.steerMaxUs", &Values::steerMaxUs },
        { "trim.steerTrimUs", &Values::steerTrimUs },
        { "trim.escMinUs", &Values::escMinUs },
        { "trim.escMaxUs", &Values::escMaxUs },
        { "trim.escReverseUs", &Values::escReverseUs },
        { "trim.steerSlewUs", &Values::steerSlewUs },
        { "trim.throttleSlewUs", &Values::throttleSlewUs },
        { "drive.throttleCapMilli", &Values::throttleCapMilli },
        { "drive.steerRateMilliPerS", &Values::steerRateMilliPerS },
        { "drive.assumedMode", &Values::assumedMode },
    } };

    [[nodiscard]] Bool isBlank(Char c)
    {
        return c == ' ' || c == '\t' || c == '\r';
    }

    [[nodiscard]] StrView trimmed(StrView s)
    {
        while(!s.empty() && isBlank(s.front()))
        {
            s.remove_prefix(1);
        }
        while(!s.empty() && isBlank(s.back()))
        {
            s.remove_suffix(1);
        }
        return s;
    }

    // An optional sign and one to nine digits, nothing else. Ten digits can
    // overflow Int32 and no field is near that, so a longer run is garbage.
    [[nodiscard]] Opt<Int32> integerOf(StrView s)
    {
        Bool negative = false;
        if(!s.empty() && (s.front() == '-' || s.front() == '+'))
        {
            negative = s.front() == '-';
            s.remove_prefix(1);
        }
        if(s.empty() || s.size() > 9)
        {
            return {};
        }
        Int32 value = 0;
        for(const Char c : s)
        {
            if(c < '0' || c > '9')
            {
                return {};
            }
            value = (value * 10) + static_cast<Int32>(c - '0');
        }
        return negative ? -value : value;
    }

    Void put(const Values& from, trimview::View& trim, driveview::View& drive)
    {
        trim.steerMinUs = from.steerMinUs;
        trim.steerMaxUs = from.steerMaxUs;
        trim.steerTrimUs = from.steerTrimUs;
        trim.escMinUs = from.escMinUs;
        trim.escMaxUs = from.escMaxUs;
        trim.escReverseUs = from.escReverseUs;
        trim.steerSlewUs = from.steerSlewUs;
        trim.throttleSlewUs = from.throttleSlewUs;
        drive.throttleCapMilli = from.throttleCapMilli;
        drive.steerRateMilliPerS = from.steerRateMilliPerS;
        drive.assumedMode = from.assumedMode;
    }

    [[nodiscard]] Values take(const trimview::View& trim, const driveview::View& drive)
    {
        Values v;
        v.steerMinUs = trim.steerMinUs;
        v.steerMaxUs = trim.steerMaxUs;
        v.steerTrimUs = trim.steerTrimUs;
        v.escMinUs = trim.escMinUs;
        v.escMaxUs = trim.escMaxUs;
        v.escReverseUs = trim.escReverseUs;
        v.steerSlewUs = trim.steerSlewUs;
        v.throttleSlewUs = trim.throttleSlewUs;
        v.throttleCapMilli = drive.throttleCapMilli;
        v.steerRateMilliPerS = drive.steerRateMilliPerS;
        v.assumedMode = drive.assumedMode;
        return v;
    }

    [[nodiscard]] Bool wideOf(const Str& utf8, WidePath& out)
    {
        const Int32 n = ::MultiByteToWideChar(
            CP_UTF8,
            MB_ERR_INVALID_CHARS,
            utf8.c_str(),
            -1,
            out.data(),
            static_cast<Int32>(out.size())
        );
        return n > 0;
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
        return n > 0 ? Str(out.data()) : Str();
    }

    Void saveFailed(const Str& path, CharSeq step, DWORD error)
    {
        vlog::line(
            "settings: NOT saved to %s - %s failed (error %u)",
            path.c_str(),
            step,
            static_cast<UInt32>(error)
        );
    }
  }

  Values capture(const trimview::View& trim, const driveview::View& drive)
  {
      trimview::View t = trim;
      driveview::View d = drive;
      trimview::settleAll(t);
      driveview::settle(d);
      return take(t, d);
  }

  Void apply(const Values& from, trimview::View& trim, driveview::View& drive)
  {
      put(from, trim, drive);
      trimview::settleAll(trim);
      driveview::settle(drive);
  }

  Values settle(const Values& v)
  {
      trimview::View trim;
      driveview::View drive;
      apply(v, trim, drive);
      return take(trim, drive);
  }

  Str toText(const Values& v)
  {
      Str out = "# bibo viewer settings - integers only, one key=value per line\n";
      for(const Field& f : FIELDS)
      {
          Array<Char, 96> line = {};
          std::snprintf(line.data(), line.size(), "%s=%d\n", f.key, v.*(f.member));
          out += line.data();
      }
      return out;
  }

  Size fromText(StrView text, Values& into)
  {
      if(text.substr(0, UTF8_BOM.size()) == UTF8_BOM)
      {
          text.remove_prefix(UTF8_BOM.size());
      }
      Array<Bool, VALUE_COUNT> seen = {};
      while(!text.empty())
      {
          const Size eol = text.find('\n');
          StrView line = trimmed(text.substr(0, eol));
          text = eol == StrView::npos ? StrView() : text.substr(eol + 1);
          if(line.empty() || line.front() == '#' || line.front() == ';')
          {
              continue;
          }
          const Size eq = line.find('=');
          if(eq == StrView::npos)
          {
              continue;
          }
          const StrView key = trimmed(line.substr(0, eq));
          const Opt<Int32> value = integerOf(trimmed(line.substr(eq + 1)));
          if(!value.has_value())
          {
              continue;
          }
          for(Size i = 0; i < FIELDS.size(); ++i)
          {
              if(key == FIELDS[i].key)
              {
                  into.*(FIELDS[i].member) = *value;
                  seen[i] = true;
              }
          }
      }
      Size taken = 0;
      for(const Bool got : seen)
      {
          if(got)
          {
              ++taken;
          }
      }
      return taken;
  }

  Str defaultPath()
  {
      constexpr const wchar_t* DIR_NAME = L"bibo";
      WidePath appData = {};
      const DWORD cap = static_cast<DWORD>(appData.size());
      const DWORD n = ::GetEnvironmentVariableW(L"APPDATA", appData.data(), cap);
      // n >= cap: the buffer is too small and n is the size needed.
      if(n == 0 || n >= cap)
      {
          vlog::line("settings: APPDATA is not set - the settings stay beside bibo.exe");
          return legacyPath();
      }
      WidePath full = {};
      if(std::swprintf(
          full.data(),
          full.size(),
          L"%ls\\%ls\\%ls",
          appData.data(),
          DIR_NAME,
          FILE_NAME
      ) < 0)
      {
          return legacyPath();
      }
      return utf8Of(full);
  }

  Str legacyPath()
  {
      WidePath exe = {};
      const DWORD cap = static_cast<DWORD>(exe.size());
      const DWORD n = ::GetModuleFileNameW(nullptr, exe.data(), cap);
      // n == cap is truncation, not success.
      if(n == 0 || n >= cap)
      {
          return "";
      }
      for(Size i = n; i > 0; --i)
      {
          if(exe[i - 1] == L'\\' || exe[i - 1] == L'/')
          {
              exe[i - 1] = L'\0';
              WidePath full = {};
              if(std::swprintf(full.data(), full.size(), L"%ls\\%ls", exe.data(), FILE_NAME) < 0)
              {
                  return "";
              }
              return utf8Of(full);
          }
      }
      return "";
  }

  Opt<Size> load(const Str& path, Values& into)
  {
      WidePath wide = {};
      if(path.empty() || !wideOf(path, wide))
      {
          vlog::line("settings: no usable path for the settings file - using the defaults");
          return {};
      }
      const HANDLE h = ::CreateFileW(
          wide.data(),
          GENERIC_READ,
          FILE_SHARE_READ,
          nullptr,
          OPEN_EXISTING,
          FILE_ATTRIBUTE_NORMAL,
          nullptr
      );
      if(h == INVALID_HANDLE_VALUE)
      {
          const DWORD error = ::GetLastError();
          // No file is a first run, logged apart from a real open failure so
          // the failure line is not trained into noise.
          if(error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND)
          {
              vlog::line("settings: no file yet at %s - using the defaults", path.c_str());
          }
          else
          {
              vlog::line(
                  "settings: could not open %s (error %u) - using the defaults",
                  path.c_str(),
                  static_cast<UInt32>(error)
              );
          }
          return {};
      }
      LARGE_INTEGER size = {};
      Str text;
      Bool read = ::GetFileSizeEx(h, &size) != 0
                  && size.QuadPart >= 0
                  && size.QuadPart <= static_cast<Int64>(MAX_FILE_BYTES);
      if(read && size.QuadPart > 0)
      {
          text.resize(static_cast<Size>(size.QuadPart));
          DWORD got = 0;
          const DWORD want = static_cast<DWORD>(text.size());
          read = ::ReadFile(h, text.data(), want, &got, nullptr) != 0 && got == want;
      }
      const DWORD error = read ? 0 : ::GetLastError();
      ::CloseHandle(h);
      if(!read)
      {
          vlog::line(
              "settings: could not read %s (error %u, or larger than %u bytes) - using the defaults",
              path.c_str(),
              static_cast<UInt32>(error),
              static_cast<UInt32>(MAX_FILE_BYTES)
          );
          return {};
      }
      Values raw = into;
      const Size taken = fromText(text, raw);
      into = settle(raw);
      vlog::line("settings: loaded %u values from %s", static_cast<UInt32>(taken), path.c_str());
      return taken;
  }

  Bool save(const Str& path, const Values& v)
  {
      WidePath wide = {};
      WidePath temp = {};
      if(path.empty() || !wideOf(path, wide))
      {
          vlog::line("settings: NOT saved - no usable path for the settings file");
          return false;
      }
      if(std::swprintf(temp.data(), temp.size(), L"%ls.tmp", wide.data()) < 0)
      {
          vlog::line("settings: NOT saved - the path is too long for its .tmp beside it");
          return false;
      }
      // CreateFileW will not make the folder, which does not exist before the
      // first save. One level is all defaultPath() adds; a real failure is
      // reported by CreateFileW below.
      WidePath folder = wide;
      for(Size i = std::wcslen(folder.data()); i > 0; --i)
      {
          if(folder[i - 1] == L'\\' || folder[i - 1] == L'/')
          {
              folder[i - 1] = L'\0';
              static_cast<Void>(::CreateDirectoryW(folder.data(), nullptr));
              break;
          }
      }
      const Str text = toText(settle(v));
      const HANDLE h = ::CreateFileW(
          temp.data(),
          GENERIC_WRITE,
          0,
          nullptr,
          CREATE_ALWAYS,
          FILE_ATTRIBUTE_NORMAL,
          nullptr
      );
      if(h == INVALID_HANDLE_VALUE)
      {
          saveFailed(path, "creating the .tmp", ::GetLastError());
          return false;
      }
      DWORD wrote = 0;
      const DWORD want = static_cast<DWORD>(text.size());
      const Bool written = ::WriteFile(h, text.data(), want, &wrote, nullptr) != 0 && wrote == want;
      DWORD error = written ? 0 : ::GetLastError();
      // Flushed before the rename, or a power cut can leave the new name over
      // empty contents.
      const Bool flushed = written && ::FlushFileBuffers(h) != 0;
      if(written && !flushed)
      {
          error = ::GetLastError();
      }
      ::CloseHandle(h);
      if(!flushed)
      {
          static_cast<Void>(::DeleteFileW(temp.data()));
          saveFailed(path, written ? "flushing the .tmp" : "writing the .tmp", error);
          return false;
      }
      const DWORD how = MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH;
      if(::MoveFileExW(temp.data(), wide.data(), how) == 0)
      {
          error = ::GetLastError();
          static_cast<Void>(::DeleteFileW(temp.data()));
          saveFailed(path, "replacing the file", error);
          return false;
      }
      vlog::line("settings: saved %u values to %s", static_cast<UInt32>(VALUE_COUNT), path.c_str());
      return true;
  }
}
