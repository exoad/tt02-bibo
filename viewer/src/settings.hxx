// The viewer's own memory of how this operator set it up: the Trim pane's limits
// and rates, and the Drive pane's cap, steering rate and asserted mode, kept in a
// small text file between runs.
//
// ---------------------------------------------------------------------------
// WHERE IT GOES
//
//   <directory of bibo.exe>\bibo-viewer-settings.ini
//
// vlog.hxx's reason for beside the exe: viewer\build\ is gitignored, so this
// file can never be committed by a careless `git add`, and one laptop's tuning
// never lands in a diff that somebody else's viewer then loads.
//
// ---------------------------------------------------------------------------
// THE FILE
//
//   # bibo viewer settings - integers only, one key=value per line
//   trim.steerMinUs=1230
//   ...
//
// INTEGERS ONLY, READ BY HAND. The value is parsed digit by digit, never through
// a locale-aware call and never through a float: a machine set to a comma
// decimal is the bug this project has met three times, and a microsecond has no
// fraction to lose anyway. A value with anything but an optional sign and digits
// in it is IGNORED, not half-read - "1541.5" is not 1541.
//
// FORGIVING ON THE WAY IN. An unknown key is skipped (a newer viewer's file read
// by an older one), a key the file does not name keeps the pane's value, a
// missing file is a first run rather than an error, and an out-of-range number
// is clamped to the range its widget uses - by trimview::settleAll and
// driveview::settle, the functions the panes themselves use, so there is one
// copy of every range.
//
// ATOMIC ON THE WAY OUT. The text goes to a .tmp beside the file, is flushed to
// disk, and is moved over the old file with MoveFileExW. A viewer killed
// mid-save leaves the old file or the new one, never half of each.
//
// ---------------------------------------------------------------------------
// WHAT IS NOT SAVED, ON PURPOSE
//
// The held steering, the enable, and whether a window is open. Those describe a
// MOMENT, and restoring a moment is how a viewer starts up turning the wheels.
//
// NOTHING HERE TALKS TO THE CAR. Loading fills the panes and sends nothing; the
// Trim pane's "send all to the car" is that act, taken on purpose.
//
// No <windows.h> in this header - settings.cxx includes it, after ours.
#pragma once

#include "shared.hxx"

#include "trim.hxx"
#include "drive.hxx"

namespace settings
{

  constexpr Size VALUE_COUNT = 10;

  // A FLAT COPY rather than pointers into the panes, so "has anything changed
  // since the last save" is one comparison of two values.
  struct Values
  {
      Int32 steerMinUs = trimview::STEER_MIN_DEFAULT;
      Int32 steerMaxUs = trimview::STEER_MAX_DEFAULT;
      Int32 steerTrimUs = trimview::STEER_CENTRE_DEFAULT;
      Int32 escMinUs = trimview::ESC_MIN_DEFAULT;
      Int32 escMaxUs = trimview::ESC_MAX_DEFAULT;
      Int32 steerSlewUs = trimview::STEER_SLEW_DEFAULT;
      Int32 throttleSlewUs = trimview::THROTTLE_SLEW_DEFAULT;
      Int32 throttleCapMilli = driveview::THROTTLE_CAP_DEFAULT;
      Int32 steerRateMilliPerS = driveview::STEER_RATE_DEFAULT;
      Int32 assumedMode = 0;

      [[nodiscard]] Bool operator==(const Values& other) const = default;
  };

  // What the panes hold now, settled - so a slider mid-typing can never be
  // written out of range.
  [[nodiscard]] Values capture(const trimview::View& trim, const driveview::View& drive);

  // Into the panes, settled. Touches the ten fields and nothing else: the
  // windows' open flags, the enable, the held steering and the counters stay.
  Void apply(const Values& from, trimview::View& trim, driveview::View& drive);

  // `v` clamped to its widgets' ranges.
  [[nodiscard]] Values settle(const Values& v);

  // The file's text: a comment line, then one key=value line per field.
  [[nodiscard]] Str toText(const Values& v);

  // Reads `text` into `into` key by key, WITHOUT settling, and returns how many
  // distinct known keys carried a valid integer.
  [[nodiscard]] Size fromText(StrView text, Values& into);

  // <directory of bibo.exe>\bibo-viewer-settings.ini as UTF-8, or empty when the
  // exe's own path could not be read.
  [[nodiscard]] Str defaultPath();

  // The file at `path` read into `into`, settled. The number of values it
  // carried, or nothing when there is no file - a first run - or it could not
  // be read; the log says which.
  [[nodiscard]] Opt<Size> load(const Str& path, Values& into);

  // Writes `v`, settled, to `path` atomically. NOT [[nodiscard]]: it logs its own
  // outcome with the reason, and a caller that has nothing better to do with a
  // failure than that is the usual one.
  Bool save(const Str& path, const Values& v);

}
