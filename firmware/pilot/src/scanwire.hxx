// The scan feed's wire format: one lidar revolution per line of text.
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS
//
// The C1 is on the Orange Pi, and the person looking at its picture is at the
// hub on a laptop across the phone's hotspot. Slamtec's SDK can only talk to a
// serial port, so something on the board has to read the device and something
// on the laptop has to read that. This file is the agreement between the two:
// the board's scanfeed FORMATS lines with it and the hub's lidar source PARSES
// them with it, and the same object file is compiled into both programs (as
// reactive.cxx already is) so they cannot drift apart.
//
// ---------------------------------------------------------------------------
// WHY TEXT, AND WHY ONLY INTEGERS
//
// Text for the same reason proto.hxx gives: `nc bibobox.local 8011` shows a
// person the scan with no tool at all, and a bug in the reader is one a person
// can see. It is the format lidar/bridge/lidar_bridge.cxx has printed on stdout
// since before the hub existed, extended by one request line.
//
// Integers because the only float formatting in the C library honours the
// locale, and a hub on a machine set to a comma decimal would then read every
// distance as zero (proto.cxx met exactly that). Angles are whole
// centi-degrees, distances whole millimetres, rotation rate in milli-hertz -
// finer than the device measures, and nothing to parse but digits.
//
// ---------------------------------------------------------------------------
// THE LINES
//
//   board -> hub
//     INFO <model> <fwMajor>.<fwMinor> <hwRev> <serialHex>   once, when the device opens
//     HEALTH <0|1|2>                                          0 good, 1 warning, 2 error
//     MOTOR <0|1>                                             the motor's state, on every change
//     F <count> <freq_mHz> <a>,<d>,<q> ...                    one revolution: a centi-degrees
//                                                             0..35999, d mm (0 = no return),
//                                                             q quality 0..63
//     ERR <message>                                           something failed; the feed closes
//
//   hub -> board
//     MOTOR <0|1>                                             spin the motor down or up
//     QUIT                                                    done; the feed parks the device
//
// A line the reader does not know is KIND_UNKNOWN and is ignored, so a field or
// a line added later does not break an older reader. A line it knows but cannot
// read whole is KIND_BAD, and the reader says so rather than guessing at the
// half it could read - a frame with the wrong count is not a shorter frame.
//
// Pure: no sockets, no clock, no device. Tested on a laptop like proto.
#pragma once

#include "shared.hxx"

namespace scanwire
{

  // Where the board's feed listens. One port, fixed, so the hub can offer
  // "the Orange Pi" as a choice rather than a form.
  constexpr UInt16 PORT = 8011;

  // One measurement as the hub draws it: hub/src/lidar_source.hxx's LidarPoint,
  // spelled here so the pilot tree does not include the hub's.
  struct Sample
  {
      Float32 angleDeg = 0.0f;   // 0..360, clockwise from the device's zero mark
      Float32 distMm = 0.0f;   // 0 means no return in this direction
      UInt8   quality = 0;      // 0..63
  };

  struct Frame
  {
      Vec<Sample> samples;
      Float32     hz = 0.0f;   // rotation rate, as the device reports it
  };

  struct Info
  {
      Int32 model = 0;
      Int32 fwMajor = 0;
      Int32 fwMinor = 0;
      Int32 hwRev = 0;
      Str   serial;   // hex, as printed
  };

  enum class Kind
  {
      KIND_FRAME = 0,
      KIND_INFO,
      KIND_HEALTH,
      KIND_MOTOR,
      KIND_ERR,
      KIND_QUIT,
      KIND_UNKNOWN,   // a line this reader has no name for; ignore it
      KIND_BAD,       // a line it knows, that did not read whole; do not use it
      KIND_EMPTY,     // nothing but whitespace
  };

  // What one parsed line said. Only the member for `kind` is meaningful.
  struct Line
  {
      Kind  kind = Kind::KIND_EMPTY;
      Frame frame;
      Info  info;
      Int32 health = -1;
      Bool  motor = false;
      Str   text;   // ERR's message, or the first word of an unknown line
  };

  // ---- writing ---------------------------------------------------------------
  // Every formatter returns a complete line including its '\n'.
  [[nodiscard]] Str formatFrame(const Frame& f);
  [[nodiscard]] Str formatInfo(const Info& i);
  [[nodiscard]] Str formatHealth(Int32 health);
  [[nodiscard]] Str formatMotor(Bool on);
  [[nodiscard]] Str formatErr(const Str& message);
  [[nodiscard]] Str formatQuit();

  // ---- reading ---------------------------------------------------------------
  // `line` is one line WITHOUT its terminator; a trailing '\r' is tolerated.
  // Always writes `out`; returns out.kind for the caller's convenience.
  Kind parse(StrView line, Line* out);

  // A Frame is REPLACED by parse() on KIND_FRAME, so a reader that keeps one
  // Line across calls does not pay for the vector each revolution. It is
  // emptied on every other kind, and on KIND_BAD, for the reason above.

}
