/* ---------------------------------------------------------------------------
 * plan - the three things between "follow a path" and "drive a route".
 *
 * pursuit answers WHERE TO STEER. These answer HOW FAST, WHETHER TO GO AT ALL,
 * and WHERE THE PATH CAME FROM. Nothing here is implemented yet.
 *
 * ---------------------------------------------------------------------------
 * THE STUBS RETURN NOT_IMPLEMENTED, NOT A PLAUSIBLE NUMBER.
 *
 * That is the whole design of this file and it is deliberate. A stub that
 * returns 0.0f for a speed limit reads as "stop", a stub that returns vMax
 * reads as "go", and both look exactly like a working implementation having an
 * opinion. Six times today a thing reported success while doing nothing, and
 * every one of them cost more to find than it would have cost to refuse.
 *
 * So every entry point here answers with a Status, and the caller cannot get a
 * number out without looking at it. When one is implemented, its Status stops
 * being NOT_IMPLEMENTED and nothing at the call site has to change.
 *
 * ---------------------------------------------------------------------------
 * THE TUNINGS ARE REAL NOW, THOUGH.
 *
 * Each of the three carries its Limits/Guard/Recorder struct with defaults, and
 * those are usable and settable today - from a sketch, from app/, from a
 * console command. They are the durable half of an interface: the numbers
 * outlive whichever algorithm ends up consuming them, and having them settled
 * means the day one of these is written it has nothing left to argue about.
 *
 * Pure arithmetic when it arrives - no SDK, no clock. Same as the rest of the
 * autonomy stack, so it runs on the Pico or the Orange Pi from one copy.
 * ------------------------------------------------------------------------- */
#pragma once

#include "geom.hxx"
#include "pursuit.hxx"

namespace bibo::plan
{

    enum Status
    {
        STATUS_OK = 0,
        STATUS_NOT_IMPLEMENTED,   /* the stub. Never mistake this for a result */
        STATUS_NO_PATH,
        STATUS_BAD_TUNING,
        STATUS_FULL
    };

    /**
     * @brief A human-readable reason for a Status.
     *
     * @param s the status to describe
     * @return a short, static, null-terminated string; "?" for an unknown
     *         value
     */
    inline CharSeq why(const Status s)
    {
        switch(s)
        {
            case STATUS_OK:              return "ok";
            case STATUS_NOT_IMPLEMENTED: return "not implemented yet";
            case STATUS_NO_PATH:         return "no path";
            case STATUS_BAD_TUNING:      return "the tuning is not usable";
            case STATUS_FULL:            return "no room left";
            default:                     return "?";
        }
    }

    /* =========================================================================
     * HOW FAST - a speed limit for the point on the path the car is at.
     *
     * The shape it will take: slow for curvature, slow for the approach to the
     * end, and never accelerate or brake harder than the tires will take. On a
     * rear-wheel-drive car with no differential lock the cornering limit is the
     * interesting one - too fast into a bend and the front washes out, and the
     * follower's steering demand is then met by a car that is not turning.
     *
     * vCorner is the term that does that work: a cap of roughly
     * sqrt(latAccel / curvature), which falls out of the same circle pursuit
     * already computes. It is why this file includes pursuit rather than
     * duplicating the geometry.
     * ========================================================================= */

    /**
     * @brief The speed and acceleration limits for one route.
     */
    struct Limits
    {
        Float32 vMax     = 1.5f;    /* m/s, the fastest this route allows   */
        Float32 aMax     = 1.0f;    /* m/s^2 accelerating                   */
        Float32 aBrake   = 2.0f;    /* m/s^2 slowing - larger, brakes beat  */
        /* the motor on this drivetrain         */
        Float32 latAccel = 2.5f;    /* m/s^2 the tires hold in a corner     */
        Float32 vMin     = 0.15f;   /* m/s. below this the car does not     */
        /* move at all, so asking for less is   */
        /* asking for a stall - see             */
        /* THROTTLE_CAL_MIN                     */
    };

    inline Limits limits;

    /**
     * @brief Installs a new set of speed and acceleration limits.
     *
     * @param l the limits to install; vMax, aMax, aBrake, latAccel in their
     *          documented units, vMin in meters per second
     * @return false, leaving the previous limits in place, when vMax, aMax,
     *         aBrake, or latAccel is not positive, or vMin is negative or
     *         exceeds vMax
     */
    inline Bool configure(const Limits& l)
    {
        if(l.vMax <= 0.0f || l.aMax <= 0.0f || l.aBrake <= 0.0f
           || l.latAccel <= 0.0f || l.vMin < 0.0f || l.vMin > l.vMax)
        {
            return false;
        }
        limits = l;
        return true;
    }

    /**
     * @brief The speed limits currently installed.
     *
     * @return a reference to the live Limits
     */
    static const Limits& tuning(Void)
    {
        return limits;
    }

    /**
     * @brief STUB. Will return the speed to hold at `pose` on `path`.
     *
     * `out` is untouched on anything but STATUS_OK, so a caller that ignores
     * the Status gets whatever it initialized - which is its own value, not
     * a number this file invented.
     *
     * @param path the path the car is following, in world meters
     * @param pose where the car is and which way it faces
     * @param out set to the speed to hold, meters per second, only when the
     *            return is STATUS_OK
     * @return STATUS_NOT_IMPLEMENTED always, for now
     */
    inline Status speedFor(const pursuit::Path* path, geom::Pose pose,
                           Float32* out)
    {
        static_cast<Void>(path);
        static_cast<Void>(pose);
        static_cast<Void>(out);
        return STATUS_NOT_IMPLEMENTED;
    }

    /* =========================================================================
     * WHETHER TO GO - the obstacle gate.
     *
     * A cap on speed from whatever the car can see. The lidar is the eventual
     * source; the ToF bumpers are the near-field one and are not fitted, their
     * XSHUT pins currently being lamps.
     *
     * TWO DISTANCES, NOT ONE. A single stop threshold makes a car that drives
     * at full speed until it slams to a halt, which is both alarming and worse
     * at avoiding anything - by the time it reacts it has no room. slowM starts
     * the taper, stopM ends it.
     * ========================================================================= */

    /**
     * @brief The stop/slow distances and the width the obstacle gate watches.
     */
    struct Guard
    {
        Float32 stopM = 0.30f;   /* meters, nearer than this: stop          */
        Float32 slowM = 1.20f;   /* meters, between the two: taper to stop  */

        /* How far to either side counts as in the way. Narrower than the car
         * is optimistic; wider makes it flinch at doorframes. Half the track
         * plus a margin is the honest starting point. */
        Float32 widthM = 0.22f;  /* meters */
    };

    inline Guard guard;

    /**
     * @brief Installs a new obstacle gate.
     *
     * @param g the gate to install; stopM, slowM, widthM all in meters
     * @return false, leaving the previous gate in place, when stopM is
     *         negative, slowM does not exceed stopM, or widthM is not
     *         positive
     */
    inline Bool configure(const Guard& g)
    {
        if(g.stopM < 0.0f || g.slowM <= g.stopM || g.widthM <= 0.0f)
        {
            return false;
        }
        guard = g;
        return true;
    }

    /**
     * @brief The obstacle gate currently installed.
     *
     * @return a reference to the live Guard
     */
    static const Guard& guardTuning(Void)
    {
        return guard;
    }

    /**
     * @brief STUB. Will return a speed cap given the nearest obstacle ahead.
     *
     * @param nearestM distance to the nearest obstacle ahead, meters
     * @param out set to the speed cap, meters per second, only when the
     *            return is STATUS_OK
     * @return STATUS_NOT_IMPLEMENTED always, for now
     */
    inline Status capFor(const Float32 nearestM, Float32* out)
    {
        static_cast<Void>(nearestM);
        static_cast<Void>(out);
        return STATUS_NOT_IMPLEMENTED;
    }

    /* =========================================================================
     * WHERE THE PATH CAME FROM - teach.
     *
     * Drive the route by hand once and keep the poses. The whole project is
     * teach-and-repeat, and this is the teach half.
     *
     * SAMPLED BY DISTANCE, NOT BY TIME. A time-sampled recording of a car that
     * stopped for ten seconds is a hundred identical points, which is a path
     * with a hundred chances to decide it has arrived. Spacing by distance
     * makes the density of the path a property of the ROUTE rather than of how
     * long the driver dithered.
     * ========================================================================= */

    /**
     * @brief The spacing rules teach uses to decide which poses to keep.
     */
    struct Recorder
    {
        /* Meters between kept points. Too fine wastes memory and adds nothing;
         * too coarse cuts corners on the replay because the follower is
         * interpolating between things that were never adjacent. */
        Float32 spacingM = 0.10f;   /* meters */

        /* And a heading change worth keeping even when the car has barely
         * moved - a tight corner taken slowly would otherwise be recorded as
         * two points and replayed as a straight line through the wall. */
        Float32 headingRad = 0.15f; /* radians */
    };

    inline Recorder recorder;

    /**
     * @brief Installs new recording spacing rules.
     *
     * @param r the rules to install; spacingM in meters, headingRad in
     *          radians
     * @return false, leaving the previous rules in place, when spacingM or
     *         headingRad is not positive
     */
    inline Bool configure(const Recorder& r)
    {
        if(r.spacingM <= 0.0f || r.headingRad <= 0.0f)
        {
            return false;
        }
        recorder = r;
        return true;
    }

    /**
     * @brief The recording spacing rules currently installed.
     *
     * @return a reference to the live Recorder
     */
    static const Recorder& recorderTuning(Void)
    {
        return recorder;
    }

    /**
     * @brief STUB. Will append `pose` to `into` if it is far enough from the
     *        last point kept.
     *
     * @param pose the pose to consider keeping
     * @param into the buffer of kept points, world meters
     * @param cap the capacity of `into`, in points
     * @param count the number of points kept so far; incremented on a keep
     * @return STATUS_NOT_IMPLEMENTED always, for now; once implemented,
     *         STATUS_FULL rather than an overwrite when `into` is exhausted
     */
    inline Status keep(geom::Pose pose, geom::Vec2* into, const Size cap, Size* count)
    {
        static_cast<Void>(pose);
        static_cast<Void>(into);
        static_cast<Void>(cap);
        static_cast<Void>(count);
        return STATUS_NOT_IMPLEMENTED;
    }

}
