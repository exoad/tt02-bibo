#include "lights.hpp"

#include <cmath>

namespace lights {

Bool blinkPhase(Float64 seconds) noexcept
{
    if(seconds < 0.0)
        seconds = 0.0;

    // std::fmod, not an accumulating counter: an accumulator drifts with the
    // frame rate and this has to stay locked to wall-clock time.
    const Float64 phase = std::fmod(seconds, BLINK_PERIOD_S);
    return phase < BLINK_ON_S;
}

Lamps solve(const Input& in, Float64 seconds) noexcept
{
    Lamps out;

    const Bool on = blinkPhase(seconds);

    // Hazards are BOTH sides, in phase. Not alternating - alternating is what a
    // film prop does, and it is the single most common way to get this wrong.
    // Both sides read the same `on`, so being in phase is structural rather
    // than something that has to be maintained.
    const Bool leftTurn  = (in.turn == Turn::TURN_LEFT)  || (in.turn == Turn::TURN_HAZARD);
    const Bool rightTurn = (in.turn == Turn::TURN_RIGHT) || (in.turn == Turn::TURN_HAZARD);

    const Float32 lit = (on ? 1.0f : 0.0f);
    out.indFL = leftTurn  ? lit : 0.0f;
    out.indRL = leftTurn  ? lit : 0.0f;
    out.indFR = rightTurn ? lit : 0.0f;
    out.indRR = rightTurn ? lit : 0.0f;

    // Headlights: one lamp at two levels.
    const Float32 head = (in.head == Head::HEAD_ON)  ? HEAD_LEVEL
                       : (in.head == Head::HEAD_DRL) ? DRL_LEVEL
                                                     : 0.0f;
    out.headL = head;
    out.headR = head;

    // The red lamp: lit at TAIL_LEVEL whenever the lights are on, at
    // BRAKE_LEVEL when braking. Braking wins - it is the more urgent claim.
    Float32 red = 0.0f;
    if(in.brake)
        red = BRAKE_LEVEL;
    else if(in.head != Head::HEAD_OFF)
        red = TAIL_LEVEL;

    out.tailL = red;
    out.tailR = red;

    // THE OVERRIDE. On a side that is indicating, the red lamp is interrupted
    // while the indicator is lit, so that side alternates bright/off while the
    // other stays solid. That asymmetry is what makes it look like a car and
    // not like a light show - and it is why the indicator has to be resolved
    // before the brake rather than drawn beside it.
    if(leftTurn && on)  out.tailL = 0.0f;
    if(rightTurn && on) out.tailR = 0.0f;

    // Reverse. Small, white, and not interrupted by anything: it says which way
    // the gearbox is, which no other signal contradicts.
    const Float32 rev = in.reverse ? 1.0f : 0.0f;
    out.revL = rev;
    out.revR = rev;

    return out;
}

} // namespace lights
