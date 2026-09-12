// The car's trim, kept on the board and handed back to the Pico whenever the
// pilot opens it.
//
// ---------------------------------------------------------------------------
// WHY THE BOARD KEEPS IT
//
// The Pico holds its servo limits, servo centre, ESC limits and slew rates in
// RAM. A power cycle, a replug or a reflash puts it back on cal.hxx's compiled
// numbers, so an operator who spent an hour in the viewer's Trim pane found the
// car had forgotten all of it the next time it was switched on - and nothing
// anywhere said so. The board is up whenever the Pico is, so the board keeps
// the numbers the operator chose and replays them on every open, including the
// quiet reconnect after a replug.
//
// ---------------------------------------------------------------------------
// THE FILE IS THE PICO'S OWN COMMANDS
//
// One per line - "SERVOTRIM 1480" - so what is replayed is exactly what is
// stored and a person can read it, fix it or delete it with a text editor.
// Anything that does not parse as one of the five known settings, inside
// bibowire's hard limits, is ignored on load rather than sent: the file is
// hand-editable, and a hand-edited typo must not reach the steering.
#pragma once

#include "shared.hxx"

namespace trimfile
{

  // One stored setting per key. Empty means "never set", which leaves the
  // Pico's compiled value standing.
  struct Store
  {
      Str servoLimits;    // "SERVOLIMITS <min> <max>"
      Str servoTrim;      // "SERVOTRIM <us>"
      Str escLimits;      // "ESCLIMITS <min> <max>"
      Str steerSlew;      // "SLEW STEER <us>"
      Str throttleSlew;   // "SLEW THROTTLE <us>"
  };

  // Folds one line that was sent to the Pico into the store. "SLEW <us>" with
  // no axis sets both, exactly as the Pico reads it. True when the store
  // changed; a line that is not a valid trim setting changes nothing.
  [[nodiscard]] Bool remember(Store& s, const Str& line);

  // Every stored line in replay order - the limits before the centre and the
  // rates, so nothing is clamped against a range about to be replaced.
  [[nodiscard]] Vec<Str> lines(const Store& s);

  [[nodiscard]] Str render(const Store& s);
  [[nodiscard]] Store parse(const Str& text);

  // BIBO_TRIM_FILE when set, else $HOME/.config/bibo/trim.txt, else
  // bibo-trim.txt in the working directory.
  [[nodiscard]] Str defaultPath();

  // A file that does not exist is an empty store and TRUE: that is a car
  // nobody has tuned yet, not a failure.
  [[nodiscard]] Bool load(const Str& path, Store& out, Str& why);

  // Written to <path>.tmp and renamed over <path>, so a crash mid-write leaves
  // the previous file whole. The directory is created when it is missing.
  [[nodiscard]] Bool save(const Str& path, const Store& s, Str& why);

}
