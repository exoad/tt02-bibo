// ---------------------------------------------------------------------------
// Where per-user state lives.
//
// %LOCALAPPDATA%\tt02-auto\ - deliberately NOT next to the exe. The build
// directory is deleted by `build.bat clean`, and a preference that resets every
// time the app is rebuilt is not a preference.
//
// Everything here degrades to "no persistence" rather than failing: an empty
// path means the caller skips the read or the write and the app runs with its
// defaults. Losing a window position is not worth a startup error.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

namespace settings
{

  // The directory, created on first use. Empty if there is no user profile.
  [[nodiscard]] Str dir();

  // A file inside it. Empty if dir() is.
  [[nodiscard]] Str path(const Char* name);

  // Whole-file text read/write, for the handful of tiny files this app keeps.
  // read() returns an empty string for a missing file, which is the same thing as
  // an empty one as far as every caller is concerned.
  [[nodiscard]] Str read(const Char* name);
  Void write(const Char* name, const Str& text);

} // namespace settings
