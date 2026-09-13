// The Trim pane's limits and rates and the Drive pane's cap, steering rate and
// asserted mode, kept between runs in
//
//   %APPDATA%\bibo\bibo-viewer-settings.ini
//
// Outside the repo, so no `git add` can commit it, and outside viewer\build,
// so build.bat clean cannot delete it. A file at legacyPath() is copied here
// once (main.cxx), and legacyPath() is used outright when APPDATA is unset.
//
//   # bibo viewer settings - integers only, one key=value per line
//   trim.steerMinUs=1230
//
// Values are integers, parsed digit by digit. Anything but an optional sign and
// digits is ignored, not half-read: "1541.5" is not 1541. An unknown key is
// skipped, a missing key keeps the pane's value, a missing file is a first run,
// and out-of-range numbers are clamped by trimview::settleAll and
// driveview::settle, the panes' own ranges.
//
// Saving writes a flushed .tmp and moves it over the file with MoveFileExW, so
// a kill mid-save leaves the old file or the new one.
//
// Not saved, on purpose: the held steering, the enable, and window open flags.
// Restoring those could start the viewer up turning the wheels. Loading sends
// nothing to the car.
//
// No <windows.h> in this header - settings.cxx includes it, after ours.
#pragma once

#include "shared.hxx"

#include "trim.hxx"
#include "drive.hxx"

namespace settings
{
  constexpr Size VALUE_COUNT = 11;

  // A flat copy, not pointers into the panes, so "changed since the last save"
  // is one comparison.
  struct Values
  {
      Int32 steerMinUs = trimview::STEER_MIN_DEFAULT;
      Int32 steerMaxUs = trimview::STEER_MAX_DEFAULT;
      Int32 steerTrimUs = trimview::STEER_CENTRE_DEFAULT;
      Int32 escMinUs = trimview::ESC_MIN_DEFAULT;
      Int32 escMaxUs = trimview::ESC_MAX_DEFAULT;
      Int32 escReverseUs = trimview::ESC_REVERSE_DEFAULT;
      Int32 steerSlewUs = trimview::STEER_SLEW_DEFAULT;
      Int32 throttleSlewUs = trimview::THROTTLE_SLEW_DEFAULT;
      Int32 throttleCapMilli = driveview::THROTTLE_CAP_DEFAULT;
      Int32 steerRateMilliPerS = driveview::STEER_RATE_DEFAULT;
      Int32 assumedMode = 0;

      [[nodiscard]] Bool operator==(const Values& other) const = default;
  };

  // What the panes hold now, settled, so a value mid-typing is never out of range.
  [[nodiscard]] Values capture(const trimview::View& trim, const driveview::View& drive);

  // Into the panes, settled. Touches only the saved fields.
  Void apply(const Values& from, trimview::View& trim, driveview::View& drive);

  // `v` clamped to its widgets' ranges.
  [[nodiscard]] Values settle(const Values& v);

  // A comment line, then one key=value line per field.
  [[nodiscard]] Str toText(const Values& v);

  // Reads `text` into `into` WITHOUT settling; returns how many distinct known
  // keys carried a valid integer.
  [[nodiscard]] Size fromText(StrView text, Values& into);

  // The APPDATA path as UTF-8, or legacyPath() when APPDATA is unset. The
  // folder need not exist yet - save() makes it.
  [[nodiscard]] Str defaultPath();

  // <directory of bibo.exe>\bibo-viewer-settings.ini, or empty when the exe's
  // own path could not be read.
  [[nodiscard]] Str legacyPath();

  // The file at `path` read into `into`, settled. The number of values it
  // carried, or nothing when there is no file or it could not be read; the log
  // says which.
  [[nodiscard]] Opt<Size> load(const Str& path, Values& into);

  // Writes `v`, settled, to `path` atomically. Not [[nodiscard]]: it logs its
  // own failure with the reason.
  Bool save(const Str& path, const Values& v);
}
