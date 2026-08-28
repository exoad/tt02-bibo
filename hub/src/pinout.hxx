// The Pico's pin table: the forty pads, what each one is, and what this project
// has wired to it.
//
// ONE table, for both boards. Every pin this project uses is identical on the
// Pico 2 and the Pico 2 W - see docs/wiring.md - and the four that are not
// (GP23, GP24, GP25, GP29) are ones nothing here is allowed to touch. That is
// exactly what makes the two boards drop-in for each other, and it is why a
// single table can describe both without lying about either.
//
// There used to be a drawing of the physical board alongside this. It was
// removed once there were two boards: a picture of one of them, shown whichever
// happened to be plugged in, is a diagram that can be wrong, and a wrong pinout
// is worse than none. The reference library's flat chart is the one rendering
// now.
#pragma once

#include "shared.hxx"

namespace board {

// What a pad is FOR, which is what decides how the reference chart colours it.
enum class PinRole : UInt8
{
    PIN_ROLE_GPIO = 0,
    PIN_ROLE_GROUND,
    PIN_ROLE_POWER,
    PIN_ROLE_SYS
};

struct PinRef
{
    Int32       phys = 0;          // physical pad, 1..40
    const Char* name = nullptr;    // silkscreen name
    const Char* use  = nullptr;    // this project's wiring, or nullptr if free
    PinRole     role = PinRole::PIN_ROLE_GPIO;
};

[[nodiscard]] Int32  pinCount();
[[nodiscard]] PinRef pinAt(Int32 i);

// True for pads 1..20, which run down the LEFT edge. 21..40 run UP the right,
// so 40 is top-right - the ordering that trips up everyone who assumes the two
// columns both count downward.
[[nodiscard]] Bool pinOnLeft(Int32 phys);

} // namespace board
