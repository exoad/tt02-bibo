/*
 * ---------------------------------------------------------------------------
 * odom - how far the car has gone, and how fast.
 *
 * Ticks in, meters out. NO HARDWARE HERE: the pin, the interrupt and the clock
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
 * taken from a catalog, every distance this produces is proportional to the
 * truth and not equal to it. calibrated() says which, so a caller can refuse to
 * navigate on a guess instead of driving confidently into a wall.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "../shared.hxx"

namespace bibo::odom
{

    /*
     * ---- the three numbers that turn ticks into meters --------------------
     *
     * A VALUE, NOT A #define, and that is the point of this block.
     *
     * They were defines, which meant tuning them was a rebuild and a reflash.
     * These are numbers you find by rolling the car across a floor with a tape
     * measure - the loop is measure, adjust, measure again - and a loop with a
     * two-minute build in it is a loop nobody runs twice. Now a sketch or the
     * console can set them and read the answer back immediately.
     *
     * The defines remain as the DEFAULTS, so a program that configures nothing
     * still behaves exactly as before.
     *
     * MEASURED, NOT LOOKED UP. A TT-02's rolling diameter depends on the tire,
     * the foam insert and how much the car weighs, and the difference between a
     * catalog number and the real one is a map that drifts a few percent per
     * lap - which looks like bad odometry rather than a wrong constant.
     */

    /**
     * @brief Default rolling wheel diameter, in millimeters.
     *
     * 64 is the stock TT-02 touring tire as a PLACEHOLDER; roll the car a
     * measured distance and divide.
     *
     * @note Overridden by Config::wheelMm once configure() is called with a
     *       measured value; see calibrated().
     */
#define ODOM_WHEEL_MM 64.0f

    /**
     * @brief Default sensor ticks per revolution of the thing the magnet is on.
     */
#define ODOM_TICKS_PER_REV 1.0f

    /**
     * @brief Default reduction between the magnet's shaft and the wheel.
     *
     * Revolutions of that thing per revolution of the WHEEL. 1.0 for an
     * axle-mounted magnet, which is the arrangement this file recommends; the
     * full 10.7 only if the magnet ends up on the motor or spur, and then it
     * changes with the pinion.
     */
#define ODOM_GEAR_RATIO 1.0f

    /**
     * @brief The three numbers that turn ticks into meters, plus a claim
     *        about whether they were measured.
     */
    struct Config
    {
        Float32 wheelMm     = ODOM_WHEEL_MM;      ///< Rolling wheel diameter, millimeters.
        Float32 ticksPerRev = ODOM_TICKS_PER_REV; ///< Sensor ticks per revolution of the magnet's shaft.
        Float32 gearRatio   = ODOM_GEAR_RATIO;    ///< Revolutions of that shaft per wheel revolution.

        /**
         * Set this yourself once the three above are MEASURED rather than
         * guessed. Nothing can work it out for you - a number is not more true
         * for having been typed in - so it is a claim the person who measured
         * makes, and calibrated() only ever repeats it.
         */
        Bool    measured    = false;
    };

    inline Config tuning;

    /**
     * @brief Installs a new tuning configuration, if it is usable.
     *
     * Rejects anything that would make metersPerTick meaningless. A zero wheel
     * or zero ticks-per-rev divides by zero and every distance afterwards is
     * inf, which propagates into the pose and is then very hard to trace back
     * to a setter that quietly accepted a bad value.
     *
     * @param c the configuration to install; wheelMm, ticksPerRev and
     *          gearRatio must each be strictly positive
     * @return false when any of the three is zero or negative; tuning is
     *         left unchanged
     */
    inline Bool configure(const Config& c)
    {
        if(c.wheelMm <= 0.0f || c.ticksPerRev <= 0.0f || c.gearRatio <= 0.0f)
        {
            return false;
        }
        tuning = c;
        return true;
    }

    /**
     * @brief The active tuning configuration.
     *
     * @return a reference to the live Config
     */
    static const Config& config(Void)
    {
        return tuning;
    }

    /**
     * @brief Whether the active configuration was measured rather than
     *        guessed.
     *
     * @return the value of Config::measured, as set by whoever last called
     *         configure()
     */
    inline Bool calibrated(Void)
    {
        return tuning.measured;
    }

    /**
     * @brief Meters of travel represented by one sensor tick.
     *
     * One multiply at the call site instead of a division, and one place
     * that knows the geometry.
     *
     * @return meters per tick, using the active Config
     */
    inline Float32 metersPerTick(Void)
    {
        const Float32 circumference = 3.14159265f * (tuning.wheelMm / 1000.0f);
        return circumference / (tuning.ticksPerRev * tuning.gearRatio);
    }

    /* ---- state ------------------------------------------------------------ */

    /**
     * @brief Per-wheel tick count and derived speed.
     *
     * `ticks` is written by whoever is counting - an interrupt, most likely -
     * and read here. It is the one field that changes underneath this code,
     * which is why it is the only one marked volatile and why nothing in this
     * file does arithmetic on it twice in a row without copying it first.
     */
    struct Wheel
    {
        volatile UInt32 ticks = 0u; ///< Raw tick count since the last reset().

        UInt32  lastTicks = 0u;    ///< ticks as of the previous update() call.
        UInt64  lastUs    = 0u;    ///< Timestamp of the previous update() call, microseconds.
        Float32 speed     = 0.0f;   /* meters per second, filtered */
        Bool    primed    = false; ///< False until the first update() call has run.
    };

    /**
     * @brief Registers one sensor tick.
     *
     * Called by the counter. Deliberately the smallest thing in this file: an
     * interrupt handler that does more than increment is an interrupt handler
     * that will one day be blamed for a servo glitch.
     *
     * @param w the wheel to increment; does nothing if null
     * @note Safe to call from an interrupt context - it only increments the
     *       volatile tick counter.
     */
    inline Void tick(Wheel* w)
    {
        if(w != nullptr)
        {
            w->ticks = w->ticks + 1u;
        }
    }

    /**
     * @brief Zeroes a wheel's ticks and speed, and re-primes it.
     *
     * @param w     the wheel to reset; does nothing if null
     * @param nowUs the current time, in microseconds, used as the baseline
     *              for the next update() call
     */
    inline Void reset(Wheel* w, const UInt64 nowUs)
    {
        if(w == nullptr)
        {
            return;
        }
        w->ticks     = 0u;
        w->lastTicks = 0u;
        w->lastUs    = nowUs;
        w->speed     = 0.0f;
        w->primed    = false;
    }

    /**
     * @brief Total distance travelled since the last reset().
     *
     * @param w the wheel to read; a null wheel reads as zero
     * @return distance, in meters
     */
    inline Float32 distance(const Wheel* w)
    {
        if(w == nullptr)
        {
            return 0.0f;
        }
        return static_cast<Float32>(w->ticks) * metersPerTick();
    }

    /* ---- speed -------------------------------------------------------------- */

    /**
     * @brief Filtered wheel speed, updated once per call.
     *
     * Ticks since the last call, over the time since the last call.
     *
     * SMOOTHED, because the raw number is unusable at low speed for a reason no
     * amount of care in the caller can fix: with a one-tick-per-revolution
     * magnet, a car doing walking pace produces about three ticks a second, so
     * any window short enough to be responsive contains zero, one or two ticks
     * and the speed reads as a square wave. The filter is a first-order lag -
     * one multiply, no history buffer, and its constant is in SECONDS rather
     * than in samples so the behavior does not change when the loop rate does.
     *
     * A caller that wants the raw value can take distance() twice and divide;
     * this is the one for a control loop.
     *
     * @param w     the wheel to update; a null wheel reads and returns 0.0
     * @param nowUs the current time, in microseconds
     * @param tauS  the filter's time constant, in seconds; zero or negative
     *              disables filtering and returns the raw instantaneous speed
     * @return the filtered speed, in meters per second
     * @note The first call after reset() only primes the filter and returns
     *       0.0; a real value needs a second call once time has passed.
     */
    inline Float32 update(Wheel* w, const UInt64 nowUs, const Float32 tauS)
    {
        if(w == nullptr)
        {
            return 0.0f;
        }

        /*
         * Copied ONCE. Reading the volatile twice could see two different
         * values if the interrupt lands between them, and the difference would
         * be silently wrong rather than obviously so.
         */
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

        /*
         * Unsigned subtraction, so a wrap of the tick counter is still the
         * right delta. This car is forward-only, so ticks never count down.
         */
        const UInt32  dTicks = now - w->lastTicks;
        const Float32 raw    = static_cast<Float32>(dTicks) * metersPerTick()
                               / dtS;

        w->lastTicks = now;
        w->lastUs    = nowUs;

        if(tauS <= 0.0f)
        {
            w->speed = raw;
        }
        else
        {
            /*
             * alpha = dt / (tau + dt). Bounded to 1 by construction, so a long
             * gap between calls settles to the raw value rather than
             * overshooting it.
             */
            const Float32 alpha = dtS / (tauS + dtS);
            w->speed += alpha * (raw - w->speed);
        }
        return w->speed;
    }

    /**
     * @brief The last speed computed by update().
     *
     * @param w the wheel to read; a null wheel reads as zero
     * @return speed, in meters per second
     */
    inline Float32 speed(const Wheel* w)
    {
        return w != nullptr ? w->speed : 0.0f;
    }

}
