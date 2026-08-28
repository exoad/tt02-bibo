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
//   * The indicator does NOT override the brake - separate lamps on this car.
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

// ---- working out the Input for yourself -----------------------------------
//
// Where an Input comes from when nobody is pressing buttons.
//
// This car has exactly two signals - where the steering is and what the ESC is
// being given - and both indicators and brake lights can be guessed from them.
// Guessed is the right word and it is worth being honest about: a real car
// knows it is braking because a pedal went down and knows it is turning because
// a stalk was pushed. Neither of those exists here, so both are inferred from
// the outputs, and the inference is wrong in the cases you would expect. A long
// sweeping bend indicates the whole way round; lifting off to coast lights the
// brakes. Both are what a driver would signal ANYWAY, which is why the trade is
// worth making.
//
// The brake reading in particular is COMMANDED deceleration, not measured -
// there is no wheel encoder yet. When there is one, the throttle input here
// becomes a speed and nothing else about this changes.
//
// Kept apart from solve() and pure in the same way, with its memory passed in
// explicitly. The rules need memory - hysteresis, a minimum flash, a brake that
// outlives the instant that triggered it - and memory hidden in a static is
// memory that cannot be tested.

struct Drive
{
    Float32 steer      = 0.0f;   // -1 hard left .. +1 hard right, as the board reports it
    Int32   throttleUs = 0;      // what the ESC is actually being given, not the slider
    Bool    armed      = false;
};

struct AutoConfig
{
    // Two thresholds, not one. A single threshold chatters: steering parked
    // exactly on it makes the indicator stutter at the servo's own noise, which
    // reads as a fault in the car rather than in the rule. Indicating starts at
    // `turnOnAbs` and does not stop until it falls back through `turnOffAbs`.
    Float32 turnOnAbs  = 0.45f;
    Float32 turnOffAbs = 0.28f;

    // At least one COMPLETE flash once triggered.
    //
    // The flasher clock is free-running and shared (see blinkPhase), so a turn
    // beginning halfway through a cycle would otherwise show a fragment of an
    // on-phase and vanish. One full period guarantees a whole flash is seen
    // whenever in the cycle it started.
    Float64 turnMinS = BLINK_PERIOD_S;

    // A drop of this many microseconds between samples counts as lifting off.
    // Small, because the slew limiter means the throttle comes down in steps of
    // SLEW_CAL_STEP and a threshold above that would never trigger at all.
    Int32 brakeDropUs = 3;

    // ...and the lamp stays lit this long afterwards. A brake light that is on
    // for one frame is a brake light nobody sees.
    Float64 brakeHoldS = 0.35;

    // Tail lights whenever the car is armed, so the brake has something to be
    // brighter THAN. Without this the red lamp is dark and the brake reads as
    // "a light came on" rather than "that light got brighter", which is the
    // whole visual grammar of a brake light.
    Bool drlWhenArmed = true;
};

struct AutoState
{
    Turn    turn         = Turn::TURN_OFF;
    Float64 turnUntil    = 0.0;   // earliest time the turn may stop
    Int32   lastThrottle = 0;
    Bool    haveThrottle = false; // the first sample is a baseline, not a drop
    Float64 brakeUntil   = 0.0;
};

[[nodiscard]] Input detect(AutoState& st, const Drive& d, Float64 seconds,
                           const AutoConfig& cfg = AutoConfig{}) noexcept;

} // namespace lights
