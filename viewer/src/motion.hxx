// motion - is the car moving, which way, how fast, and how far has it gone:
// from ODOM alone, which the board sends whenever the Pico is talking, so
// the default view shows movement with no bundle loaded. PURE: frames in,
// a state out; the scene draws it and the View window prints it.
//
// The Pico's own speed estimate is used as it comes (hall.hxx: by edge period
// when slow, so a single tick shows within a quarter second, by window when
// fast), and its sign is the direction. Distance is the ticks between frames
// in the tick's length (odom.hxx), signed, so reversing takes it back.
#pragma once

#include "shared.hxx"

#include "bibowire.hxx"

namespace motion
{
  struct State
  {
      Bool have = false;       // an ODOM frame is fresh
      Bool moving = false;     // the Pico reports a speed
      Int32 dir = 0;           // +1 forward, -1 reverse, 0 stopped or unknown
      Float32 speedMps = 0.0f; // unsigned
      Float64 travelM = 0.0;   // signed, since the first frame
      Int32 lastTicks = 0;
      UInt8 lastSeq = 0;
      Bool primed = false;
  };

  // One fresh ODOM frame. The same frame again (same seq) changes nothing.
  Void feed(State& s, const bibowire::Odom& m);

  // No fresh frame: nothing is known about movement, the distance is kept.
  Void lost(State& s);

  // A speed as a person reads it: "forward 0.42 m/s", "reverse 0.10 m/s",
  // "stopped", or "--" when unknown. Whole centimetres a second, so no
  // locale can put a comma in it.
  [[nodiscard]] Str describe(const State& s);
}
