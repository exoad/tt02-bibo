// The car's line protocol, read and written from the companion board.
//
// The Pico speaks newline-terminated ASCII in both directions and has done
// since long before there was anything but a person typing at it. The hub is
// one client of that protocol; the Orange Pi is about to be a second. Nothing
// here invents a wire format - it reads the one that already exists.
//
// ---------------------------------------------------------------------------
// WHY THE COMPANION SPEAKS THE HUMAN PROTOCOL
//
// A binary protocol between two computers is the obvious choice and it is the
// wrong one here. The text protocol is already implemented on the board, is
// already carried over both USB CDC and UDP (see hub/src/pico_link.hxx, which
// swaps transports under one send()/drain() precisely because the payload is
// text), and can be driven by hand when something is wrong. A second format
// would mean a second parser in the firmware, on a board whose flash is the
// scarce resource, to save bytes on a link that carries a few hundred a second.
//
// The cost is real and worth naming: parsing text is where the bugs live. That
// is what this file is for.
//
// ---------------------------------------------------------------------------
// WHAT THE BOARD SAYS
//
//     OK <verb> [key=value ...]      it did the thing
//     ERR <reason>                   it did not, and why
//     INFO <topic> [key=value ...]   telemetry, solicited or not
//
// Fields are read BY NAME rather than by position, so a field added to the
// firmware later is ignored by an older reader instead of shifting everything
// after it. That convention already exists in the hub; what does not exist
// there is a reader that respects token boundaries - see field().
//
// ---------------------------------------------------------------------------
// Pure: no sockets, no serial, no clock. That is what makes it testable on a
// laptop today, months before the board it is for is plugged in.
#pragma once

#include "shared.hxx"

namespace proto
{

  enum class Kind
  {
      KIND_OK = 0,
      KIND_ERR,
      KIND_INFO,

      // A line that is none of the above. NOT an error: the board prints banner
      // text at boot and a person may be typing into the same port. A companion
      // that treats every unrecognized line as a fault is a companion that stops
      // when somebody opens a terminal.
      KIND_OTHER,

      // Nothing but whitespace.
      KIND_EMPTY,
  };

  // One line from the board, split but not interpreted.
  struct Reply
  {
      Kind kind = Kind::KIND_EMPTY;

      // The word after OK or INFO - "drive", "status", "sound". Empty for ERR,
      // whose remainder is prose rather than a topic, and for OTHER.
      Str topic;

      // Everything after the topic, which is where the key=value pairs live.
      // For ERR this is the whole reason.
      Str rest;

      // The line as received, minus the line ending. Kept because a log of what
      // the board actually said is worth more than a log of what we made of it.
      Str line;
  };

  // Classifies one line. Strips CR, so a caller may hand it either ending.
  [[nodiscard]] Reply read(const Str& line);

  // The value of `key` in `text`, or false if it is not there.
  //
  // MATCHES ON TOKEN BOUNDARIES, and that is the whole reason this function
  // exists rather than a strstr at each call site. `strstr(line, "esc=")` finds
  // the "esc=" inside "desc=" and returns a number from the wrong field, having
  // reported success. The key here must start the text or follow a space.
  //
  // The value runs to the next space. No quoting: nothing the firmware emits
  // has a space in a value, and inventing an escape convention for a case that
  // does not arise would be a second format to keep in step.
  [[nodiscard]] Bool field(const Str& text, const Char* key, Str& out);

  // The same, converted. False when the key is absent OR the value is not a
  // number - a field that is present and unparseable is a change in the
  // firmware, not a default worth guessing at.
  [[nodiscard]] Bool fieldInt(const Str& text, const Char* key, Int32& out);
  [[nodiscard]] Bool fieldFloat(const Str& text, const Char* key, Float32& out);

  // ---- what the companion sends -----------------------------------------
  //
  // Formatted here rather than at the call sites so there is one place where
  // the number formatting is decided. printf("%f") is locale-sensitive and a
  // machine set to a comma decimal separator would send `STEER 0,25`, which the
  // board's parser reads as 0 - a silent hard-left at the first corner on a
  // developer's laptop in the wrong locale.

  // A steering command, as a fraction of this car's travel. Clamped to -1..1,
  // because the board clamps anyway and a companion that sends 4.0 has a bug
  // worth seeing at the point it happens.
  [[nodiscard]] Str steer(Float32 fraction);

  // A throttle pulse in microseconds.
  [[nodiscard]] Str escUs(Int32 us);

  // Everything off: neutral, disarm, release.
  [[nodiscard]] Str stop();

  // Any other command, assembled without a format string.
  [[nodiscard]] Str command(const Char* verb, const Char* args = "");

}
