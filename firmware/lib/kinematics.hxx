/*
 * ---------------------------------------------------------------------------
 * kinematics - the bicycle model, which is what a TT-02 is.
 *
 * A car steers by pointing its front wheels: it cannot turn on the spot and it
 * cannot move sideways, which is why a skid-steer controller dropped onto this
 * car asks for demands it can never satisfy. TWO WHEELS, not four - each axle
 * collapses to one wheel on the centerline, and the Ackermann error that
 * ignores is smaller than the whole-tooth steps the servo horn's spline allows.
 *
 * Pure arithmetic. Compiles for the Pico, the Orange Pi and the host test.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "geom.hxx"

namespace bibo::kin
{

    /**
     * @brief Front axle to rear axle, in meters.
     *
     * 257 mm is the TT-02 catalog figure. This number sets the whole
     * relationship between steering angle and curvature, so an error here is
     * a constant proportional error in every turn the car ever makes - which
     * reads as a controller that is badly tuned rather than a number that is
     * wrong.
     *
     * @warning UNMEASURED PLACEHOLDER. This is not a measurement of this
     *          car - measure it between the axle centers on the actual
     *          chassis. A controller tuned against a guessed wheelbase is a
     *          controller that will need retuning once this is measured.
     */
#define KIN_WHEELBASE_M 0.257f

    /**
     * @brief The largest steering angle the linkage actually reaches, in
     *        radians.
     *
     * Find it by measuring the tightest circle the car can drive and
     * inverting steerFor() - a circle of radius R is a curvature of 1/R.
     *
     * NOT SYMMETRIC ON THIS CAR. cal.hxx has 1230 left, 1480 center, 1660
     * right: 250 us one way and 180 the other. So this is the SMALLER of the
     * two, because a controller that assumes it has the larger will ask for a
     * turn it cannot make in one direction and quietly understeer.
     * chassis.hxx scales each side separately when it converts a fraction to
     * microseconds; this constant is the honest limit for a controller
     * reasoning in angles.
     *
     * @warning UNMEASURED PLACEHOLDER, and the harder of the two to guess: it
     *          depends on the horn position, the tie rods, and where the
     *          servo's travel was calibrated to. It is not a measurement of
     *          this car, and a controller tuned against a guessed steering
     *          limit is a controller that will need retuning once this is
     *          measured.
     */
#define KIN_MAX_STEER_RAD 0.42f

    /* ---- the car's shape, as a value --------------------------------------- */

    /**
     * @brief The car's physical shape: wheelbase and steering limit, as a
     *        value that can be measured and set at runtime.
     *
     * The defines above are the DEFAULTS. These are the numbers you get by
     * driving the car and measuring what it did - the tightest circle it can
     * turn, the distance between its axle centers - and that is a loop of
     * measure, adjust, measure again. A rebuild in the middle of that loop is
     * why it does not get run a second time.
     *
     * A sketch or the console can set these, and everything below that reads
     * them takes effect on the next call.
     */
    struct Config
    {
        Float32 wheelbase = KIN_WHEELBASE_M;   /* meters, front axle to rear */
        Float32 maxSteer  = KIN_MAX_STEER_RAD; /* radians, the smaller lock  */

        /* As with odometry: a claim the person who measured makes. */
        Bool    measured  = false;
    };

    inline Config tuning;

    /**
     * @brief Installs a new car shape, rejecting one a car cannot have.
     *
     * @param c the shape to install; wheelbase in meters, maxSteer in
     *          radians
     * @return false, leaving the previous shape in place, when wheelbase or
     *         maxSteer is zero or negative
     *
     * @note A zero or negative wheelbase would divide by zero in
     *       curvatureFor(); a zero maxSteer would make steerFraction()
     *       meaningless and report full lock as zero for every input.
     */
    inline Bool configure(const Config& c)
    {
        if(c.wheelbase <= 0.0f || c.maxSteer <= 0.0f)
        {
            return false;
        }
        tuning = c;
        return true;
    }

    /**
     * @brief The car shape currently installed.
     *
     * @return a reference to the live Config
     */
    static const Config& config(Void)
    {
        return tuning;
    }

    /**
     * @brief Whether the installed shape was actually measured on this car.
     *
     * @return true only when the caller of configure() set Config::measured;
     *         false for the header's placeholder defaults
     */
    inline Bool calibrated(Void)
    {
        return tuning.measured;
    }

    /* ---- steering angle and curvature are the same fact ------------------- */

    /**
     * @brief Converts a steering angle to the curvature it drives.
     *
     * curvature = tan(steer) / wheelbase
     *
     * Curvature is 1/radius: zero is straight, 2.0 is a half-meter circle. A
     * controller thinks in curvature because that is what a path has; the
     * servo wants an angle. This function and steerFor() are the only place
     * the conversion happens.
     *
     * @param steerRad the steering angle, radians, positive turning left
     * @param wheelbase front axle to rear axle, meters, must be positive
     * @return the curvature, 1/meters; 0.0 when wheelbase is not positive
     */
    inline Float32 curvatureFor(const Float32 steerRad, const Float32 wheelbase)
    {
        if(wheelbase <= 0.0f)
        {
            return 0.0f;
        }
        return tanf(steerRad) / wheelbase;
    }

    /**
     * @brief Converts a curvature to the steering angle that drives it.
     *
     * The inverse of curvatureFor().
     *
     * @param curvature the wanted curvature, 1/meters, positive turning left
     * @param wheelbase front axle to rear axle, meters
     * @return the steering angle, radians; not clamped to the linkage's
     *         travel - pass the result through clampSteer() before using it
     */
    inline Float32 steerFor(const Float32 curvature, const Float32 wheelbase)
    {
        return atanf(curvature * wheelbase);
    }

    /**
     * @brief Clamps a steering angle to what the linkage can reach.
     *
     * A controller handed back an angle it cannot achieve would keep asking
     * for a tighter turn and never notice it was not getting one.
     *
     * @param steerRad the wanted steering angle, radians
     * @param maxRad the linkage's limit, radians, applied symmetrically
     * @return steerRad clamped to [-maxRad, maxRad]
     *
     * @note The CURVATURE that comes from a clamped angle is clamped with it,
     *       since curvature and angle are the same fact - see curvatureFor().
     */
    inline Float32 clampSteer(const Float32 steerRad, const Float32 maxRad)
    {
        if(steerRad > maxRad)
        {
            return maxRad;
        }
        if(steerRad < -maxRad)
        {
            return -maxRad;
        }
        return steerRad;
    }

    /**
     * @brief A steering angle as a fraction of full lock.
     *
     * This is what drive::steer wants. The chassis maps that fraction onto
     * this car's asymmetric microseconds.
     *
     * @param steerRad the steering angle, radians
     * @param maxRad the linkage's limit, radians; must be positive
     * @return the clamped angle divided by maxRad, in [-1, 1]; 0.0 when
     *         maxRad is not positive
     */
    inline Float32 steerFraction(const Float32 steerRad, const Float32 maxRad)
    {
        if(maxRad <= 0.0f)
        {
            return 0.0f;
        }
        const Float32 f = clampSteer(steerRad, maxRad) / maxRad;
        return f;
    }

    /* ---- dead reckoning ----------------------------------------------------- */

    /**
     * @brief Where one step of driving puts the car.
     *
     * THE EXACT ARC, not a straight line. Over one 20 ms tick at 2 m/s the
     * car moves 40 mm, and on a tight turn the difference between the chord
     * and the arc is small - but it is a BIAS, not noise: it always cuts the
     * corner, so it accumulates in one direction and a lap of a room closes
     * short by a predictable amount. Integrating the arc costs a sin and a
     * cos that this code was going to spend anyway.
     *
     * The straight-line case is kept for near-zero curvature, where the arc
     * form divides by a heading change that is approaching zero.
     *
     * @param p the pose before this step: x and y in meters, heading in
     *          radians
     * @param v speed, meters per second, along the car's heading
     * @param steerRad the steering angle for this step, radians
     * @param wheelbase front axle to rear axle, meters
     * @param dtS the step duration, seconds; must be positive
     * @return the pose after driving the arc for dtS seconds; unchanged from
     *         `p` when dtS or wheelbase is not positive
     */
    inline geom::Pose integrate(const geom::Pose& p, const Float32 v, const Float32 steerRad,
                                const Float32 wheelbase, const Float32 dtS)
    {
        if(dtS <= 0.0f || wheelbase <= 0.0f)
        {
            return p;
        }

        const Float32 ds     = v * dtS;                                /* meters */
        const Float32 dTheta = ds * tanf(steerRad) / wheelbase;      /* radians */

        geom::Pose out = p;

        /* 1e-6 rad in one step is a 40 km radius - straight, and the divide blows up. */
        if(dTheta < 1e-6f && dTheta > -1e-6f)
        {
            out.x += ds * cosf(p.heading);
            out.y += ds * sinf(p.heading);
        }
        else
        {
            const Float32 radius = ds / dTheta;
            const Float32 next   = p.heading + dTheta;

            out.x += radius * (sinf(next) - sinf(p.heading));
            out.y -= radius * (cosf(next) - cosf(p.heading));
        }

        out.heading = geom::wrapPi(p.heading + dTheta);
        return out;
    }

    /*
     * ---- the same four, on the INSTALLED shape ------------------------------
     * The explicit-wheelbase forms above stay for tests and hypotheticals; these
     * are what ordinary code calls, so no call site threads the numbers itself.
     */

    /**
     * @brief curvatureFor(), using the installed Config::wheelbase.
     *
     * @param steerRad the steering angle, radians, positive turning left
     * @return the curvature, 1/meters
     */
    inline Float32 curvatureFor(const Float32 steerRad)
    {
        return curvatureFor(steerRad, tuning.wheelbase);
    }

    /**
     * @brief steerFor(), using the installed Config::wheelbase.
     *
     * @param curvature the wanted curvature, 1/meters, positive turning left
     * @return the steering angle, radians, not clamped to the linkage
     */
    inline Float32 steerFor(const Float32 curvature)
    {
        return steerFor(curvature, tuning.wheelbase);
    }

    /**
     * @brief clampSteer(), using the installed Config::maxSteer.
     *
     * @param steerRad the wanted steering angle, radians
     * @return steerRad clamped to [-maxSteer, maxSteer]
     */
    inline Float32 clampSteer(const Float32 steerRad)
    {
        return clampSteer(steerRad, tuning.maxSteer);
    }

    /**
     * @brief steerFraction(), using the installed Config::maxSteer.
     *
     * @param steerRad the steering angle, radians
     * @return the fraction of full lock, in [-1, 1]
     */
    inline Float32 steerFraction(const Float32 steerRad)
    {
        return steerFraction(steerRad, tuning.maxSteer);
    }

    /**
     * @brief integrate(), using the installed Config::wheelbase.
     *
     * @param p the pose before this step: x and y in meters, heading in
     *          radians
     * @param v speed, meters per second
     * @param steerRad the steering angle for this step, radians
     * @param dtS the step duration, seconds; must be positive
     * @return the pose after driving the arc for dtS seconds
     */
    inline geom::Pose integrate(const geom::Pose& p, const Float32 v, const Float32 steerRad, const Float32 dtS)
    {
        return integrate(p, v, steerRad, tuning.wheelbase, dtS);
    }

    /**
     * @brief The tightest circle this car can drive, at full lock.
     *
     * Useful for refusing a path before following it into a wall rather than
     * after.
     *
     * @param maxRad the steering limit, radians
     * @param wheelbase front axle to rear axle, meters
     * @return the turn radius, meters; 1e6 (effectively straight) when the
     *         curvature at full lock is within 1e-6 of zero
     */
    inline Float32 minTurnRadius(const Float32 maxRad, const Float32 wheelbase)
    {
        const Float32 k = curvatureFor(maxRad, wheelbase);
        if(k < 1e-6f && k > -1e-6f)
        {
            return 1e6f;   /* effectively straight */
        }
        return k < 0.0f ? -1.0f / k : 1.0f / k;
    }

    /**
     * @brief minTurnRadius(), using the installed Config.
     *
     * @return the turn radius at full lock, meters
     */
    inline Float32 minTurnRadius(Void)
    {
        return minTurnRadius(tuning.maxSteer, tuning.wheelbase);
    }

}
