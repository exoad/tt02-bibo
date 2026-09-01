/* ---------------------------------------------------------------------------
 * control - PID and feedforward, as arithmetic.
 *
 * NO HARDWARE, NO SDK, NO CLOCK. Everything here takes the numbers it needs as
 * arguments, including the timestep. That is what lets it be tested on a host
 * against invented inputs instead of by driving a car into a wall, and it is
 * the same split that makes dfplayer_proto and sfx testable.
 *
 * The caller owns the loop and the clock. This owns the response.
 *
 * ---------------------------------------------------------------------------
 * WHY FEEDFORWARD FIRST AND PID SECOND.
 *
 * A PID asked to do the whole job has to build its output out of accumulated
 * error, which means it can only correct a speed it is ALREADY getting wrong.
 * On a car whose motor does not move at all below a certain pulse, that is
 * most of the useful range: the integral winds up during the dead zone,
 * then dumps when the wheels finally break loose.
 *
 * Feedforward is the model - "this much pulse produces about this much speed" -
 * and the PID only corrects what the model got wrong. That is why gainS
 * exists, and why it matters most on this drivetrain.
 * ------------------------------------------------------------------------- */
#pragma once

#include "types.hxx"

namespace bibo::control
{

    /* ---- feedforward ------------------------------------------------------
     *
     * output = gainS*sign(v) + gainV*v + gainA*a
     *
     * Named gainS/gainV/gainA rather than the literature's kS/kV/kA, which the
     * style rule reserves for SCREAMING_SNAKE constants. Same three terms.
     *
     *   gainS  what it takes to move AT ALL - static friction, cogging, the
     *          gearbox. On this car it is most of the answer:
     *          THROTTLE_CAL_MIN is 1541 us, so 41 us of pulse produces no
     *          motion at all, and a controller that does not know that spends
     *          its first 41 us achieving nothing.
     *
     *   gainV  output per metre per second, once moving. The linear part.
     *
     *   gainA  output per metre per second squared. Small on a light car and
     *          safe at zero until the rest is tuned - it matters when a target
     *          CHANGES quickly rather than when it is held.
     *
     * All three are in the caller's output units. For this car that is
     * MICROSECONDS ABOVE IDLE, not raw pulse - see chassis.hxx, where idle is
     * a measured property of this ESC and motor rather than 1500. */
    struct Feedforward
    {
        Float32 gainS = 0.0f;
        Float32 gainV = 0.0f;
        Float32 gainA = 0.0f;
    };

    /* THE SIGN OF gainS IS THE TARGET'S, AND ZERO MEANS ZERO.
     *
     * The static term at a target of zero is a car that creeps - asking for the
     * pulse that just barely moves it while being told to stand still.
     * That is not a tuning mistake anybody would spot in a log; it is a car
     * that will not hold still on a bench and looks like a calibration fault.
     *
     * So the static term is gated on the target being non-zero, not on the
     * MEASUREMENT being non-zero - gating on measurement would mean a stopped
     * car could never start. */
    inline Float32 predict(const Feedforward* f, const Float32 vTarget, const Float32 aTarget)
    {
        if(f == nullptr)
        {
            return 0.0f;
        }

        Float32 out = f->gainV * vTarget + f->gainA * aTarget;

        if(vTarget > 0.0f)
        {
            out += f->gainS;
        }
        else if(vTarget < 0.0f)
        {
            out -= f->gainS;
        }

        return out;
    }

    /* ---- PID --------------------------------------------------------------
     *
     * Correcting what the model got wrong, which is a much smaller job than
     * producing the output. */
    struct Pid
    {
        Float32 kp = 0.0f;
        Float32 ki = 0.0f;
        Float32 kd = 0.0f;

        /* The integral is clamped in OUTPUT units, so the limit means the same
         * thing whatever ki is. A limit expressed in error-seconds changes
         * meaning every time the gain is retuned, which is how an anti-windup
         * clamp quietly stops clamping. */
        Float32 iMax = 0.0f;

        Float32 outMin = 0.0f;
        Float32 outMax = 0.0f;

        /* ---- state ---- */
        Float32 integral  = 0.0f;
        Float32 lastMeas  = 0.0f;
        Bool    primed    = false;
    };

    inline Void reset(Pid* p)
    {
        if(p == nullptr)
        {
            return;
        }
        p->integral = 0.0f;
        p->lastMeas = 0.0f;
        p->primed   = false;
    }

    /* One step. `dtS` is seconds since the last call.
     *
     * THREE THINGS THIS DOES THAT A TEXTBOOK PID DOES NOT, each because the
     * textbook version misbehaves on a real vehicle:
     *
     * DERIVATIVE ON MEASUREMENT, not on error. A step change in the target -
     * which is every time a person moves a slider - makes the error jump, and
     * d(error)/dt of a jump is a spike. Differentiating the measurement instead
     * gives the same damping with no kick, because the measurement cannot jump.
     * The sign flips to compensate.
     *
     * NO DERIVATIVE ON THE FIRST STEP. There is no previous measurement, so the
     * first one would differentiate against zero and produce a large output
     * from nothing. `primed` exists for that one step.
     *
     * CONDITIONAL INTEGRATION. The integral only accumulates when the output is
     * not already saturated, or when the error would drive it back INTO range.
     * Without that, a target the car cannot reach - a wheel against a kerb -
     * winds the integral up for as long as it is held, and the car leaps when
     * it comes free. Clamping alone does not fix that; it only bounds how long
     * the leap lasts. */
    inline Float32 step(Pid* p, const Float32 setpoint, const Float32 measured, const Float32 dtS)
    {
        if(p == nullptr)
        {
            return 0.0f;
        }

        /* A zero or negative timestep is a caller bug or a clock that wrapped.
         * Returning the proportional term alone is the safe answer: it uses no
         * history, so it cannot be corrupted by a bad dt, and it does not
         * silently freeze the loop. */
        const Float32 error = setpoint - measured;

        if(dtS <= 0.0f)
        {
            return p->kp * error;
        }

        const Float32 pTerm = p->kp * error;

        /* Derivative of the MEASUREMENT, negated. */
        Float32 dTerm = 0.0f;
        if(p->primed)
        {
            dTerm = -p->kd * ((measured - p->lastMeas) / dtS);
        }
        p->lastMeas = measured;
        p->primed   = true;

        /* Provisional, to find out whether integrating would saturate. */
        const Float32 without = pTerm + dTerm + p->integral;

        const Float32 add = p->ki * error * dtS;

        const Bool high = p->outMax > p->outMin && without >= p->outMax;
        const Bool low  = p->outMax > p->outMin && without <= p->outMin;

        /* Integrate unless that would push further into a limit we are already
         * against. Error pointing back toward the middle always integrates. */
        if(!((high && add > 0.0f) || (low && add < 0.0f)))
        {
            p->integral += add;

            if(p->iMax > 0.0f)
            {
                if(p->integral > p->iMax)
                {
                    p->integral = p->iMax;
                }
                else if(p->integral < -p->iMax)
                {
                    p->integral = -p->iMax;
                }
            }
        }

        Float32 out = pTerm + dTerm + p->integral;

        if(p->outMax > p->outMin)
        {
            if(out > p->outMax)
            {
                out = p->outMax;
            }
            else if(out < p->outMin)
            {
                out = p->outMin;
            }
        }
        return out;
    }

    /* What the caller WANTS, as one value. Speed and acceleration are a single
     * demand - they are always set together and always mean the same instant -
     * and passing them separately made a six-argument signature out of four
     * things. */
    struct Demand
    {
        Float32 v = 0.0f;   /* metres per second        */
        Float32 a = 0.0f;   /* metres per second squared */
    };

    /* Feedforward plus correction, which is the arrangement the whole file is
     * arguing for. Kept as one call so the ORDER cannot be got wrong at a call
     * site - the model first, the correction on top of it. */
    inline Float32 command(const Feedforward* f, Pid* p, const Demand d, const Float32 v, const Float32 dt)
    {
        return predict(f, d.v, d.a) + step(p, d.v, v, dt);
    }

}
