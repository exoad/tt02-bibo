/* ---------------------------------------------------------------------------
 * odom - how far the car has gone, and how fast.
 *
 * Ticks in, metres out. NO HARDWARE HERE: the pin, the interrupt and the clock
 * belong to whoever is counting, and this turns their number into a distance.
 * That is what makes it testable on a host, and it is why every function takes
 * the timestamp rather than reading one.
 *
 * ---------------------------------------------------------------------------
 * NOTHING IS WIRED YET, AND THE NUMBERS BELOW ARE NOT MEASURED.
 *
 * pins::car().encoder is NONE - GP15 was earmarked for the Hall sensor and the
 * DFPlayer took it, so the encoder has no pad. Two things must be decided
 * before any of this reports a real distance, and they are decisions about
 * HARDWARE that no amount of code can settle:
 *
 *   WHERE THE MAGNET GOES. On the AXLE, one tick is one wheel revolution and
 *   the gearing is irrelevant - simple, robust, and coarse: at 2 m/s a 64 mm
 *   wheel turns about ten times a second, so ten ticks a second is all the
 *   resolution there is. On the SPUR or the motor, every tick is divided by
 *   10.7 and the resolution is superb, but the number now depends on the
 *   pinion - and this car changed from 19T to 17T last week, which would have
 *   silently rescaled every distance it had ever measured.
 *
 *   That is the argument for the axle, and it is why GEAR_RATIO below is
 *   1.0 with a note rather than 10.7: a constant that is only right for one
 *   mounting is a constant that will be wrong after the next rebuild.
 *
 *   HOW MANY MAGNETS. One tick per revolution or six; it multiplies directly
 *   into TICKS_PER_REV.
 *
 * Until both are settled and WHEEL_MM is measured with calipers rather than
 * taken from a catalogue, every distance this produces is proportional to the
 * truth and not equal to it. calibrated() says which, so a caller can refuse to
 * navigate on a guess instead of driving confidently into a wall.
 * ------------------------------------------------------------------------- */
#pragma once

#include "../types.hxx"

namespace bibo
{

  namespace odom
  {

    /* ---- the three numbers that turn ticks into metres --------------------
     *
     * MEASURED, NOT LOOKED UP, when the time comes. A TT-02's rolling diameter
     * depends on the tyre, the foam insert and how much the car weighs, and the
     * difference between a catalogue number and the real one is a map that
     * drifts a few percent per lap - which looks exactly like bad odometry
     * rather than a wrong constant. */

    /* Rolling diameter in millimetres. 64 is the stock TT-02 touring tyre as a
     * PLACEHOLDER; roll the car a measured distance and divide. */
    #define ODOM_WHEEL_MM 64.0f

    /* Ticks per revolution OF THE THING THE MAGNET IS ON. */
    #define ODOM_TICKS_PER_REV 1.0f

    /* Revolutions of that thing per revolution of the WHEEL. 1.0 for an
     * axle-mounted magnet, which is the arrangement this file recommends; the
     * full 10.7 only if the magnet ends up on the motor or spur, and then it
     * changes with the pinion. */
    #define ODOM_GEAR_RATIO 1.0f

    /* Set once the three above are real. Until then every reading is
     * proportional to the truth and calibrated() says so. */
    #define ODOM_CALIBRATED 0

    static Bool calibrated(Void)
    {
        return ODOM_CALIBRATED != 0;
    }

    /* Metres of travel per tick. One multiply at the call site instead of a
     * division, and one place that knows the geometry. */
    static Float32 metresPerTick(Void)
    {
        const Float32 circumference = 3.14159265f * (ODOM_WHEEL_MM / 1000.0f);
        return circumference / (ODOM_TICKS_PER_REV * ODOM_GEAR_RATIO);
    }

    /* ---- state ------------------------------------------------------------
     *
     * `ticks` is written by whoever is counting - an interrupt, most likely -
     * and read here. It is the one field that changes underneath this code,
     * which is why it is the only one marked volatile and why nothing in this
     * file does arithmetic on it twice in a row without copying it first. */
    typedef struct Wheel
    {
        volatile UInt32 ticks = 0u;

        UInt32  lastTicks = 0u;
        UInt64  lastUs    = 0u;
        Float32 speed     = 0.0f;   /* metres per second, filtered */
        Bool    primed    = false;
    } Wheel;

    /* Called by the counter. Deliberately the smallest thing in this file: an
     * interrupt handler that does more than increment is an interrupt handler
     * that will one day be blamed for a servo glitch. */
    static Void tick(Wheel* w)
    {
        if(w != NULL)
        {
            w->ticks = w->ticks + 1u;
        }
    }

    static Void reset(Wheel* w, UInt64 nowUs)
    {
        if(w == NULL)
        {
            return;
        }
        w->ticks     = 0u;
        w->lastTicks = 0u;
        w->lastUs    = nowUs;
        w->speed     = 0.0f;
        w->primed    = false;
    }

    /* Total distance since the last reset. */
    static Float32 distance(const Wheel* w)
    {
        if(w == NULL)
        {
            return 0.0f;
        }
        return static_cast<Float32>(w->ticks) * metresPerTick();
    }

    /* ---- speed ------------------------------------------------------------
     *
     * Ticks since the last call, over the time since the last call.
     *
     * SMOOTHED, because the raw number is unusable at low speed for a reason no
     * amount of care in the caller can fix: with a one-tick-per-revolution
     * magnet, a car doing walking pace produces about three ticks a second, so
     * any window short enough to be responsive contains zero, one or two ticks
     * and the speed reads as a square wave. The filter is a first-order lag -
     * one multiply, no history buffer, and its constant is in SECONDS rather
     * than in samples so the behaviour does not change when the loop rate does.
     *
     * A caller that wants the raw value can take distance() twice and divide;
     * this is the one for a control loop. */
    static Float32 update(Wheel* w, UInt64 nowUs, Float32 tauS)
    {
        if(w == NULL)
        {
            return 0.0f;
        }

        /* Copied ONCE. Reading the volatile twice could see two different
         * values if the interrupt lands between them, and the difference would
         * be silently wrong rather than obviously so. */
        const UInt32 now = w->ticks;

        if(!w->primed)
        {
            w->lastTicks = now;
            w->lastUs    = nowUs;
            w->primed    = true;
            return 0.0f;
        }

        if(nowUs <= w->lastUs)
        {
            return w->speed;   /* no time has passed, or the clock went back */
        }

        const Float32 dtS = static_cast<Float32>(nowUs - w->lastUs) / 1000000.0f;

        /* Unsigned subtraction, so a wrap of the tick counter is still the
         * right delta. This car is forward-only, so ticks never count down. */
        const UInt32  dTicks = now - w->lastTicks;
        const Float32 raw    = (static_cast<Float32>(dTicks) * metresPerTick())
                             / dtS;

        w->lastTicks = now;
        w->lastUs    = nowUs;

        if(tauS <= 0.0f)
        {
            w->speed = raw;
        }
        else
        {
            /* alpha = dt / (tau + dt). Bounded to 1 by construction, so a long
             * gap between calls settles to the raw value rather than
             * overshooting it. */
            const Float32 alpha = dtS / (tauS + dtS);
            w->speed += alpha * (raw - w->speed);
        }
        return w->speed;
    }

    static Float32 speed(const Wheel* w)
    {
        return (w != NULL) ? w->speed : 0.0f;
    }

  } /* namespace odom */

} /* namespace bibo */
