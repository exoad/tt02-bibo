/* ---------------------------------------------------------------------------
 * kinematics - the bicycle model, which is what a TT-02 is.
 *
 * A car steers by pointing its front wheels; it cannot turn on the spot and it
 * cannot move sideways. That single constraint is the difference between this
 * and the differential-drive maths most robot code is written around, and it is
 * why a skid-steer controller dropped onto this car produces demands it can
 * physically never satisfy.
 *
 * TWO WHEELS, not four. Real Ackermann steering points the inner and outer
 * wheels at slightly different angles so both roll without scrubbing; the
 * bicycle model collapses each axle to one wheel on the centreline. The error
 * is a fraction of a degree at this wheelbase and track, and carrying the full
 * geometry would buy precision the steering linkage cannot deliver - the horn
 * only fits its spline at whole-tooth intervals, which is a far larger error
 * than the one being ignored.
 *
 * Pure arithmetic. Compiles for the Pico, the Orange Pi and the host test.
 * ------------------------------------------------------------------------- */
#pragma once

#include "geom.hxx"

namespace bibo
{

  namespace kin
  {

    /* Front axle to rear axle, in metres.
     *
     * 257 mm is the TT-02 catalogue figure and is a PLACEHOLDER. Measure it
     * between the axle centres on the actual car: this number sets the whole
     * relationship between steering angle and curvature, so an error here is a
     * constant proportional error in every turn the car ever makes - which
     * reads as a controller that is badly tuned rather than a number that is
     * wrong. */
    #define KIN_WHEELBASE_M 0.257f

    /* The largest steering angle the linkage actually reaches, in radians.
     *
     * ALSO A PLACEHOLDER, and the harder of the two to guess: it depends on the
     * horn position, the tie rods and where the servo's travel was calibrated
     * to. Find it by measuring the tightest circle the car can drive and
     * inverting steerFor - a circle of radius R is a curvature of 1/R.
     *
     * AND IT IS NOT SYMMETRIC ON THIS CAR. cal.hxx has 1230 left, 1480 centre,
     * 1660 right: 250 us one way and 180 the other. So this is the SMALLER of
     * the two, because a controller that assumes it has the larger will ask for
     * a turn it cannot make in one direction and quietly understeer. chassis.hxx
     * scales each side separately when it converts a fraction to microseconds;
     * this constant is the honest limit for a controller reasoning in angles. */
    #define KIN_MAX_STEER_RAD 0.42f

    /* ---- the car's shape, as a value -------------------------------------
     *
     * The defines above are the DEFAULTS. These are the numbers you get by
     * driving the car and measuring what it did - the tightest circle it can
     * turn, the distance between its axle centres - and that is a loop of
     * measure, adjust, measure again. A rebuild in the middle of that loop is
     * why it does not get run a second time.
     *
     * So a sketch or the console can set them, and everything above that reads
     * them takes effect on the next call. */
    struct Config
    {
        Float32 wheelbase = KIN_WHEELBASE_M;
        Float32 maxSteer  = KIN_MAX_STEER_RAD;

        /* As with odometry: a claim the person who measured makes, not
         * something this file can work out. */
        Bool    measured  = false;
    };

    inline Config tuning;

    /* Refuses a shape a car cannot have. A zero or negative wheelbase divides
     * by zero in curvatureFor; a zero maxSteer makes steerFraction meaningless
     * and would report full lock as zero for every input. */
    inline Bool configure(const Config& c)
    {
        if(c.wheelbase <= 0.0f || c.maxSteer <= 0.0f)
        {
            return false;
        }
        tuning = c;
        return true;
    }

    static const Config& config(Void)
    {
        return tuning;
    }

    inline Bool calibrated(Void)
    {
        return tuning.measured;
    }

    /* ---- steering angle and curvature are the same fact ------------------
     *
     * curvature = tan(steer) / wheelbase
     *
     * Curvature is 1/radius: zero is straight, 2.0 is a half-metre circle.
     * A controller thinks in curvature because that is what a path has; the
     * servo wants an angle. These two functions are the only place the
     * conversion happens. */
    inline Float32 curvatureFor(Float32 steerRad, Float32 wheelbase)
    {
        if(wheelbase <= 0.0f)
        {
            return 0.0f;
        }
        return tanf(steerRad) / wheelbase;
    }

    inline Float32 steerFor(Float32 curvature, Float32 wheelbase)
    {
        return atanf(curvature * wheelbase);
    }

    /* Clamped to what the linkage can reach, and the CURVATURE clamped with it.
     * A controller handed back an angle it cannot achieve would keep asking for
     * a tighter turn and never notice it was not getting one. */
    inline Float32 clampSteer(Float32 steerRad, Float32 maxRad)
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

    /* As a fraction of full lock, which is what drive::steer wants. The chassis
     * maps that onto this car's asymmetric microseconds. */
    inline Float32 steerFraction(Float32 steerRad, Float32 maxRad)
    {
        if(maxRad <= 0.0f)
        {
            return 0.0f;
        }
        const Float32 f = clampSteer(steerRad, maxRad) / maxRad;
        return f;
    }

    /* ---- dead reckoning ---------------------------------------------------
     *
     * Where one step of driving puts the car.
     *
     * THE EXACT ARC, not a straight line. Over one 20 ms tick at 2 m/s the car
     * moves 40 mm, and on a tight turn the difference between the chord and the
     * arc is small - but it is a BIAS, not noise: it always cuts the corner, so
     * it accumulates in one direction and a lap of a room closes short by a
     * predictable amount. Integrating the arc costs a sin and a cos that this
     * code was going to spend anyway.
     *
     * The straight-line case is kept for near-zero curvature, where the arc
     * form divides by a heading change that is approaching zero. */
    inline geom::Pose integrate(geom::Pose p, Float32 v, Float32 steerRad,
                                Float32 wheelbase, Float32 dtS)
    {
        if(dtS <= 0.0f || wheelbase <= 0.0f)
        {
            return p;
        }

        const Float32 ds     = v * dtS;                                /* metres */
        const Float32 dTheta = (ds * tanf(steerRad)) / wheelbase;      /* radians */

        geom::Pose out = p;

        /* 1e-6 rad over one step is a radius of about 40 km - straight, by any
         * measure this car can take. Below it the arc form's divide is the only
         * thing that would be interesting, and it would be interesting in the
         * wrong way. */
        if((dTheta < 1e-6f) && (dTheta > -1e-6f))
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

    /* The tightest circle this car can drive, in metres. Useful for refusing a
     * path before following it into a wall rather than after. */
    /* ---- the same four, on the INSTALLED shape ---------------------------
     *
     * The explicit-wheelbase forms above stay, because a test wants to ask
     * about a car that is not this one and a planner may reason about a
     * hypothetical. These are what ordinary code calls: threading the same two
     * numbers through every call site is how one of them ends up stale. */
    inline Float32 curvatureFor(Float32 steerRad)
    {
        return curvatureFor(steerRad, tuning.wheelbase);
    }

    inline Float32 steerFor(Float32 curvature)
    {
        return steerFor(curvature, tuning.wheelbase);
    }

    inline Float32 clampSteer(Float32 steerRad)
    {
        return clampSteer(steerRad, tuning.maxSteer);
    }

    inline Float32 steerFraction(Float32 steerRad)
    {
        return steerFraction(steerRad, tuning.maxSteer);
    }

    inline geom::Pose integrate(geom::Pose p, Float32 v, Float32 steerRad, Float32 dtS)
    {
        return integrate(p, v, steerRad, tuning.wheelbase, dtS);
    }

    inline Float32 minTurnRadius(Float32 maxRad, Float32 wheelbase)
    {
        const Float32 k = curvatureFor(maxRad, wheelbase);
        if((k < 1e-6f) && (k > -1e-6f))
        {
            return 1e6f;   /* effectively straight */
        }
        return (k < 0.0f) ? (-1.0f / k) : (1.0f / k);
    }

    inline Float32 minTurnRadius(Void)
    {
        return minTurnRadius(tuning.maxSteer, tuning.wheelbase);
    }

  } /* namespace kin */

} /* namespace bibo */
