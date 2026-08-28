// ---------------------------------------------------------------------------
// Vehicle lighting: the state machine, not the drawing.
//
// This is the behaviour written down in docs/conventions.md under "Lighting behavior",
// implemented once. The 3D view drives it today so the rules can be watched and
// argued with before any LED exists; the firmware will need exactly this logic,
// and it is here rather than in the renderer so that porting it is a copy rather
// than a re-derivation from a picture.
//
// Pure: inputs and a clock go in, per-lamp brightness comes out. No statics, no
// device, nothing to initialise - which is what makes it testable, and it is
// tested (tests/test_lights.cxx), because three of these rules are the kind
// that look right and are not:
//
//   * The indicator OVERRIDES the brake on its own side.
//   * Hazards are both sides in phase, not alternating.
//   * Front and rear on one side share ONE clock, or they drift apart.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

namespace lights {

// 1.5 Hz, which is the legal standard and what reads as correct. 400 on, 267
// off - deliberately not 50/50: a slightly longer on than off is what real
// flasher cans do and what the eye expects.
inline constexpr Float64 BLINK_ON_S     = 0.400;
inline constexpr Float64 BLINK_OFF_S    = 0.267;
inline constexpr Float64 BLINK_PERIOD_S = BLINK_ON_S + BLINK_OFF_S;

// Tail and brake are the SAME red lamp. Brake is not a separate light on most
// cars, so this is a duty cycle rather than a second bulb.
inline constexpr Float32 TAIL_LEVEL  = 0.30f;
inline constexpr Float32 BRAKE_LEVEL = 1.00f;

// Daytime running lights are the headlight at reduced output, again one lamp.
inline constexpr Float32 DRL_LEVEL  = 0.45f;
inline constexpr Float32 HEAD_LEVEL = 1.00f;

enum class Head
{
    HEAD_OFF = 0,
    HEAD_DRL,
    HEAD_ON,
};

enum class Turn
{
    TURN_OFF = 0,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_HAZARD,
};

struct Input
{
    Head head    = Head::HEAD_OFF;
    Turn turn    = Turn::TURN_OFF;
    Bool brake   = false;
    Bool reverse = false;
};

// Brightness per lamp, 0..1. Left and right are the CAR's left and right, seen
// from behind it looking forward - the same convention a driver uses.
struct Lamps
{
    Float32 headL = 0.0f, headR = 0.0f;   // white, front
    Float32 tailL = 0.0f, tailR = 0.0f;   // red, rear - tail AND brake
    Float32 indFL = 0.0f, indFR = 0.0f;   // amber, front
    Float32 indRL = 0.0f, indRR = 0.0f;   // amber, rear
    Float32 revL  = 0.0f, revR  = 0.0f;   // white, rear
};

// True while the shared flasher is in its ON phase. Exposed because every
// indicator on the vehicle must read the SAME clock: per-lamp timers drift, and
// two indicators on one side blinking out of step is instantly wrong.
[[nodiscard]] Bool blinkPhase(Float64 seconds) noexcept;

// `seconds` is any monotonic clock. See blinkPhase for why there is one.
[[nodiscard]] Lamps solve(const Input& in, Float64 seconds) noexcept;

} // namespace lights
