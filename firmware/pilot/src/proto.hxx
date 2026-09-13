// The car's line protocol, read and written from the companion board: the
// Pico's newline-terminated ASCII. Text rather than binary because the firmware
// already parses it and a person can drive it by hand; a second format would
// need a second parser in the Pico's scarce flash. Parsing text is where the
// bugs live, and this file is where that parsing is held to account.
//
//     OK <verb> [key=value ...]      it did the thing
//     ERR <reason>                   it did not, and why
//     INFO <topic> [key=value ...]   telemetry, solicited or not
//
// Fields are read BY NAME, so a field the firmware adds later is ignored by an
// older reader instead of shifting the rest. Pure: no sockets, serial or clock.
#pragma once

#include "shared.hxx"

namespace proto
{
  enum class Kind
  {
      KIND_OK = 0,
      KIND_ERR,
      KIND_INFO,

      // None of the above, and NOT an error: the board prints banner text at
      // boot, and a person may be typing into the same port.
      KIND_OTHER,

      // Nothing but whitespace.
      KIND_EMPTY,
  };

  // One line from the board, split but not interpreted.
  struct Reply
  {
      Kind kind = Kind::KIND_EMPTY;

      // The word after OK or INFO - "drive", "stop", "esc". Empty for ERR,
      // whose remainder is prose rather than a topic, and for OTHER.
      Str topic;

      // Everything after the topic: the key=value pairs, or ERR's whole reason.
      Str rest;

      // The line as received, minus the line ending, for logs.
      Str line;
  };

  // Classifies one line. Strips CR, so a caller may hand it either ending.
  [[nodiscard]] Reply read(const Str& line);

  // The value of `key` in `text`, or false if it is not there.
  //
  // MATCHES ON TOKEN BOUNDARIES: the key must start the text or follow a space.
  // A plain substring search finds "esc=" inside "desc=" and reports success
  // with a number from the wrong field.
  //
  // The value runs to the next space. No quoting: nothing the firmware emits
  // has a space in a value.
  [[nodiscard]] Bool field(const Str& text, const Char* key, Str& out);

  // The same, converted. False when the key is absent OR the value is not a
  // number: an unparseable value is a firmware change, not a default to guess.
  [[nodiscard]] Bool fieldInt(const Str& text, const Char* key, Int32& out);
  [[nodiscard]] Bool fieldFloat(const Str& text, const Char* key, Float32& out);

  // A steering command, as a fraction of this car's travel, clamped to -1..1.
  // Never formatted with printf("%f"), which is locale-sensitive: see fixed3 in
  // proto.cxx.
  [[nodiscard]] Str steer(Float32 fraction);

  // A throttle pulse in microseconds.
  [[nodiscard]] Str escUs(Int32 us);

  // Everything off: neutral, disarm, release.
  [[nodiscard]] Str stop();

  // Any other command, assembled without a format string.
  [[nodiscard]] Str command(const Char* verb, const Char* args = "");
}
