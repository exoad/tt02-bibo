// The car's trim, kept on the board and replayed to the Pico whenever the pilot
// opens it. The Pico holds its servo limits, servo centre, ESC limits and slew
// rates in RAM, so a power cycle, replug or reflash puts it back on cal.hxx's
// compiled numbers. The board is up whenever the Pico is, so it replays the
// operator's numbers on every open, including the reconnect after a replug.
//
// The file is the Pico's own commands, one per line ("SERVOTRIM 1480"), so what
// is replayed is exactly what is stored, and a person can read, fix or delete
// it with a text editor. A line that is not one of the six known settings,
// inside bibowire's hard limits, is ignored on load rather than sent: a
// hand-edited typo must not reach the steering.
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
      Str escReverse;     // "ESCREVERSE <us>", neutral being reverse off
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

  // Every stored line in replay order on ONE line, joined by "; " - what the
  // board tells a viewer under bibowire::EVENT_CODE_TRIM. Empty for a car
  // nobody has tuned.
  [[nodiscard]] Str report(const Store& s);

  // BIBO_TRIM_FILE when set, else $HOME/.config/bibo/trim.txt, else
  // bibo-trim.txt in the working directory.
  [[nodiscard]] Str defaultPath();

  // Every directory above `path`, created in turn; EEXIST is the common case
  // and not a failure. Shared with the bundle set, which lives beside the trim,
  // so there is one mkdir idiom rather than two.
  Void makeParents(const Str& path);

  // A file that does not exist is an empty store and TRUE: that is a car
  // nobody has tuned yet, not a failure.
  [[nodiscard]] Bool load(const Str& path, Store& out, Str& why);

  // Written to <path>.tmp and renamed over <path>, so a crash mid-write leaves
  // the previous file whole. The directory is created when it is missing.
  [[nodiscard]] Bool save(const Str& path, const Store& s, Str& why);
}
