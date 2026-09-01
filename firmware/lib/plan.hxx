/*
 * ---------------------------------------------------------------------------
 * plan - the three things between "follow a path" and "drive a route".
 *
 * pursuit answers WHERE TO STEER. These answer HOW FAST, WHETHER TO GO AT ALL,
 * and WHERE THE PATH CAME FROM. Nothing here is implemented yet.
 *
 * THE STUBS RETURN NOT_IMPLEMENTED, NOT A PLAUSIBLE NUMBER. 0.0f for a speed
 * limit reads as "stop" and vMax reads as "go", both looking exactly like a
 * working implementation with an opinion; a Status instead means no number
 * comes out without the caller looking at it. The tunings, unlike the code, are
 * real and settable today, and the arithmetic when it arrives uses no SDK and
 * no clock, so it runs on the Pico or the Orange Pi from one copy.
 * -------------------------------------------------------------------------
 */
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

    /*
     * HOW FAST - a speed limit for the point on the path the car is at. Slow for
     * curvature, slow for the approach to the end, never harder than the tires
     * will take. vCorner does that work: a cap of roughly sqrt(latAccel /
     * curvature), out of the same circle pursuit already computes.
     */

    /**
     * @brief The speed and acceleration limits for one route.
     */
    struct Limits
    {
        Float32 vMax = 1.5f;    /* m/s, the fastest this route allows           */
        Float32 aMax = 1.0f;    /* m/s^2 accelerating                           */
        Float32 aBrake = 2.0f;    /* m/s^2 slowing - larger, brakes beat the motor */
        Float32 latAccel = 2.5f;    /* m/s^2 the tires hold in a corner             */
        Float32 vMin = 0.15f;   /* m/s, under this it stalls - THROTTLE_CAL_MIN */
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
    inline Status speedFor(const pursuit::Path* path, geom::Pose pose, Float32* out)
    {
        static_cast<Void>(path);
        static_cast<Void>(pose);
        static_cast<Void>(out);
        return STATUS_NOT_IMPLEMENTED;
    }

    /*
     * WHETHER TO GO - the obstacle gate, a cap on speed from what the car can see:
     * the lidar eventually, the ToF bumpers once fitted. TWO DISTANCES, NOT ONE -
     * a single stop threshold drives at full speed until it slams to a halt, with
     * no room left by the time it reacts. slowM starts the taper, stopM ends it.
     */

    /**
     * @brief The stop/slow distances and the width the obstacle gate watches.
     */
    struct Guard
    {
        Float32 stopM = 0.30f;   /* meters, nearer than this: stop          */
        Float32 slowM = 1.20f;   /* meters, between the two: taper to stop  */

        /*
         * How far to either side counts as in the way: half the track plus a
         * margin, narrower being optimistic and wider flinching at doorframes.
         */
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

    /*
     * WHERE THE PATH CAME FROM - teach. Drive the route by hand once and keep the
     * poses. SAMPLED BY DISTANCE, NOT BY TIME: a car that stopped for ten seconds
     * is a hundred identical points, a hundred chances to decide it has arrived.
     * Distance makes density a property of the ROUTE, not of the driver.
     */

    /**
     * @brief The spacing rules teach uses to decide which poses to keep.
     */
    struct Recorder
    {
        /*
         * Meters between kept points. Too fine wastes memory; too coarse cuts
         * corners on the replay, interpolating between points never adjacent.
         */
        Float32 spacingM = 0.10f;   /* meters */

        /*
         * A heading change worth keeping even when the car has barely moved - a
         * slow tight corner would otherwise replay as a line through the wall.
         */
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
