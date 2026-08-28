#include "lights.hxx"

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

    // NO OVERRIDE. The red lamp is not interrupted by anything.
    //
    // It was, and the reason was a real convention applied to the wrong car. On
    // many cars the rear indicator and the brake light are ONE bulb, so the
    // indicator must interrupt the brake to be seen - and that interruption is
    // what makes such a car read as a car.
    //
    // This car does not have that bulb. Tails and indicators are separate LEDs
    // on separate pins, with a second indicator pair going on the rear, so they
    // never compete for one lamp. Applied anyway it made the brake light blink
    // in antiphase to the signal beside it, which is the one thing a brake
    // light must not do.
    //
    // A shared-bulb cluster, if one is ever fitted, is a BINDING problem - two
    // lamps on one pin - not a rule.

    // Reverse. Small, white, and not interrupted by anything: it says which way
    // the gearbox is, which no other signal contradicts.
    const Float32 rev = in.reverse ? 1.0f : 0.0f;
    out.revL = rev;
    out.revR = rev;

    return out;
}

// ---------------------------------------------------------------------------
// detect: steering and throttle in, an Input out.
//
// The order matters. The indicator decision reads `steer` only and the brake
// decision reads `throttleUs` only, so neither can affect the other - which is
// what keeps the two rules independently testable and independently wrong.
// ---------------------------------------------------------------------------
Input detect(AutoState& st, const Drive& d, Float64 seconds, const AutoConfig& cfg) noexcept
{
    Input in;

    // ---- indicators ------------------------------------------------------
    const Float32 mag  = (d.steer < 0.0f) ? -d.steer : d.steer;
    const Turn    want = (d.steer <= -cfg.turnOnAbs) ? Turn::TURN_LEFT
                       : (d.steer >=  cfg.turnOnAbs) ? Turn::TURN_RIGHT
                                                     : Turn::TURN_OFF;

    if(want != Turn::TURN_OFF && want != st.turn)
    {
        // A genuine change of side takes effect at once, hold or no hold.
        // Waiting out the old side's minimum flash would leave the car
        // indicating LEFT while the wheels are already turning right, which is
        // the one thing an indicator must never do.
        st.turn      = want;
        st.turnUntil = seconds + cfg.turnMinS;
    }
    else if(st.turn != Turn::TURN_OFF
            && mag < cfg.turnOffAbs
            && seconds >= st.turnUntil)
    {
        st.turn = Turn::TURN_OFF;
    }
    in.turn = st.turn;

    // ---- brake -----------------------------------------------------------
    //
    // The FIRST sample only establishes a baseline. Without that, connecting to
    // a board already holding throttle reads as a drop from zero and flashes
    // the brakes on connect.
    if(!st.haveThrottle)
    {
        st.lastThrottle = d.throttleUs;
        st.haveThrottle = true;
    }

    // Not gated on `armed`. Disarming walks the ESC back to neutral, which IS
    // the car slowing down, and the brake lamp should say so.
    const Int32 drop = st.lastThrottle - d.throttleUs;
    if(drop >= cfg.brakeDropUs)
    {
        st.brakeUntil = seconds + cfg.brakeHoldS;
    }
    st.lastThrottle = d.throttleUs;

    in.brake = (seconds < st.brakeUntil);

    // ---- the rest --------------------------------------------------------
    in.head = (cfg.drlWhenArmed && d.armed) ? Head::HEAD_DRL : Head::HEAD_OFF;

    // Never. chassis.h is forward-only - the throttle is clamped to
    // [escMin, escMax] and the board refuses anything below 1500 - so there is
    // no reverse for this to report. It stays in Input because the lamp exists
    // on the car and the day the ESC gains a reverse sequence this is where the
    // answer goes.
    in.reverse = false;

    return in;
}

} // namespace lights
