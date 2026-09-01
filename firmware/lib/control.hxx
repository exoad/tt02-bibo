/*
 * ---------------------------------------------------------------------------
 * control - PID and feedforward, as arithmetic.
 *
 * NO HARDWARE, NO SDK, NO CLOCK. Every number arrives as an argument, the
 * timestep included, so this tests on a host. The caller owns the loop and the
 * clock; this owns the response.
 *
 * FEEDFORWARD FIRST, PID SECOND. A PID can only correct a speed it is already
 * getting wrong, and this car's motor does not move at all below a certain
 * pulse. The model predicts; the PID corrects what the model got wrong.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "shared.hxx"

namespace bibo::control
{

    /* ---- feedforward -------------------------------------------------------- */

    /**
     * @brief The three feedforward gains: output = gainS*sign(v) + gainV*v +
     *        gainA*a.
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
     *   gainV  output per meter per second, once moving. The linear part.
     *
     *   gainA  output per meter per second squared. Small on a light car and
     *          safe at zero until the rest is tuned - it matters when a target
     *          CHANGES quickly rather than when it is held.
     *
     * @note All three are in the caller's output units. For this car that is
     *       MICROSECONDS ABOVE IDLE, not raw pulse - see chassis.hxx, where
     *       idle is a measured property of this ESC and motor rather than
     *       1500.
     */
    struct Feedforward
    {
        Float32 gainS = 0.0f;
        Float32 gainV = 0.0f;
        Float32 gainA = 0.0f;
    };

    /**
     * @brief Predicts the output needed to hold a target speed and
     *        acceleration, before any correction is added.
     *
     * @param f the gains to predict with; does nothing (returns 0.0) when
     *          null
     * @param vTarget the target speed, meters per second; its SIGN, not its
     *                measured value, decides which way gainS is applied
     * @param aTarget the target acceleration, meters per second squared
     * @return the predicted output, in the caller's units (see Feedforward)
     *
     * @note THE SIGN OF gainS IS THE TARGET'S, AND ZERO MEANS ZERO. The
     *       static term at a target of zero is a car that creeps - asking
     *       for the pulse that just barely moves it while being told to
     *       stand still. That is not a tuning mistake anybody would spot in
     *       a log; it is a car that will not hold still on a bench and
     *       looks like a calibration fault. So the static term is gated on
     *       the target being non-zero, not on the MEASUREMENT being
     *       non-zero - gating on measurement would mean a stopped car could
     *       never start.
     */
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

    /* ---- PID ------------------------------------------------------------- */

    /**
     * @brief One PID's gains, output limits, and running state.
     *
     * Correcting what the model got wrong, which is a much smaller job than
     * producing the output.
     */
    struct Pid
    {
        Float32 kp = 0.0f;
        Float32 ki = 0.0f;
        Float32 kd = 0.0f;

        /* Clamped in OUTPUT units, so the limit means the same thing whatever ki is. */
        Float32 iMax = 0.0f;   /* output units; 0 disables the clamp */

        Float32 outMin = 0.0f; /* output units */
        Float32 outMax = 0.0f; /* output units; outMax <= outMin disables clamping */

        /* ---- state ---- */
        Float32 integral  = 0.0f;
        Float32 lastMeas  = 0.0f;
        Bool    primed    = false;
    };

    /**
     * @brief Clears a Pid's running state, without touching its gains or
     *        limits.
     *
     * @param p the controller to reset; does nothing when null
     */
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

    /**
     * @brief One PID step.
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
     * Without that, a target the car cannot reach - a wheel against a curb -
     * winds the integral up for as long as it is held, and the car leaps when
     * it comes free. Clamping alone does not fix that; it only bounds how long
     * the leap lasts.
     *
     * @param p the controller to step; returns 0.0 when null
     * @param setpoint the target value, in the measurement's units
     * @param measured the current measured value, same units as setpoint
     * @param dtS seconds since the last call to step() for this controller;
     *            must be positive
     * @return the controller's output, in the caller's output units,
     *         clamped to [outMin, outMax] when outMax > outMin
     *
     * @note A zero or negative dtS is a caller bug or a clock that wrapped.
     *       The proportional term alone is returned in that case: it uses no
     *       history, so it cannot be corrupted by a bad dt, and it does not
     *       silently freeze the loop.
     */
    inline Float32 step(Pid* p, const Float32 setpoint, const Float32 measured, const Float32 dtS)
    {
        if(p == nullptr)
        {
            return 0.0f;
        }

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

        /* Integrate unless that pushes further into a limit already reached. */
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

    /**
     * @brief What the caller WANTS, as one value.
     *
     * Speed and acceleration are a single demand - they are always set
     * together and always mean the same instant - and passing them
     * separately made a six-argument signature out of four things.
     */
    struct Demand
    {
        Float32 v = 0.0f;   /* meters per second        */
        Float32 a = 0.0f;   /* meters per second squared */
    };

    /**
     * @brief Feedforward plus correction: the full command for one step.
     *
     * Kept as one call so the ORDER cannot be got wrong at a call site - the
     * model first, the correction on top of it.
     *
     * @param f the feedforward gains; see predict()
     * @param p the PID state to correct with and update; see step()
     * @param d the demand: target speed (m/s) and acceleration (m/s^2)
     * @param v the current measured speed, meters per second
     * @param dt seconds since the last call to command() for this `p`
     * @return the output to send to the actuator, in the caller's output
     *         units - predict()'s result plus step()'s correction
     */
    inline Float32 command(const Feedforward* f, Pid* p, const Demand d, const Float32 v, const Float32 dt)
    {
        return predict(f, d.v, d.a) + step(p, d.v, v, dt);
    }

}
