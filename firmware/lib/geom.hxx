/*
 * ---------------------------------------------------------------------------
 * geom - where the car is, in meters and radians.
 *
 * WORLD FRAME, not pixels: gfx.hxx has a Point and it is a screen coordinate.
 * Pure arithmetic - no SDK, no clock, no hardware - so it compiles for the
 * Pico, the Orange Pi and the host test from one copy.
 *
 * ANGLES ARE RADIANS AND ARE WRAPPED. 179 degrees and -179 degrees are 2
 * degrees apart the short way and 358 the long way, and a controller that
 * takes the long way turns the car all the way round. wrapPi exists for that.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "types.hxx"

/* The float forms deliberately: the double ones pull in software double-precision on an M33. */
#include <math.h>

namespace bibo::geom
{

#define GEOM_PI  3.14159265358979f
#define GEOM_TAU 6.28318530717959f

    struct Vec2
    {
        Float32 x = 0.0f;   /* meters */
        Float32 y = 0.0f;
    };

    /**
     * @brief Where the car is and which way it faces.
     *
     * Flat rather than a Vec2 plus an angle: a pose is one thing to the code
     * that integrates it, and splitting it invites a position updated without
     * its heading.
     *
     * @note Heading is RADIANS, zero along +x and increasing toward +y - the
     *       ordinary mathematical convention, not a compass bearing. That is
     *       what lets cos/sin be used on it without a sign flip.
     */
    struct Pose
    {
        Float32 x       = 0.0f;
        Float32 y       = 0.0f;
        Float32 heading = 0.0f;
    };

    /* ---- angles ----------------------------------------------------------- */

    /**
     * @brief Folds an angle into (-pi, pi].
     *
     * A loop rather than fmod, deliberately: fmod on a negative gives a
     * negative remainder, so the naive one-liner is wrong for exactly half its
     * inputs. The loop runs at most a couple of times for any angle a vehicle
     * actually produces.
     *
     * @param a an angle in radians, of any magnitude
     * @return the same angle folded into (-pi, pi]
     */
    inline Float32 wrapPi(Float32 a)
    {
        while(a > GEOM_PI)
        {
            a -= GEOM_TAU;
        }
        while(a <= -GEOM_PI)
        {
            a += GEOM_TAU;
        }
        return a;
    }

    /**
     * @brief The SHORT way round from one heading to another.
     *
     * What a heading controller wants. Plain subtraction is the one that sends
     * the car the long way round when the two straddle +/-pi.
     *
     * @param from the current heading, radians
     * @param to the wanted heading, radians
     * @return the signed difference in (-pi, pi]; positive turns toward +y
     */
    inline Float32 angleDelta(const Float32 from, const Float32 to)
    {
        return wrapPi(to - from);
    }

    /* ---- distances -------------------------------------------------------- */

    /**
     * @brief The squared distance between two points.
     *
     * Offered separately because most uses COMPARE distances, and a comparison
     * does not need the square root - which on an M33 with no hardware divide
     * is worth avoiding inside a loop over a path.
     *
     * @param a one point, meters
     * @param b the other, meters
     * @return the squared distance, meters squared
     */
    inline Float32 distanceSq(const Vec2 a, const Vec2 b)
    {
        const Float32 dx = b.x - a.x;
        const Float32 dy = b.y - a.y;
        return dx * dx + dy * dy;
    }

    /**
     * @brief The distance between two points.
     *
     * @param a one point, meters
     * @param b the other, meters
     * @return the distance, meters
     *
     * @note Prefer distanceSq() where the value is only being compared - this
     *       one pays for a square root that the comparison does not need.
     */
    inline Float32 distance(const Vec2 a, const Vec2 b)
    {
        return sqrtf(distanceSq(a, b));
    }

    /* ---- frames ----------------------------------------------------------- */

    /**
     * @brief A world point expressed in the car's own frame.
     *
     * +x is straight ahead, +y is to the left. This is the transform pure
     * pursuit is built on.
     *
     * @param p where the car is and which way it faces
     * @param world the point, in world meters
     * @return the same point relative to the car, in meters
     *
     * @warning THE SIGN OF THE RESULT IS THE WHOLE ANSWER - a goal with
     *          positive y is to the left, and the car steers left. Getting the
     *          rotation backwards produces a controller that steers away from
     *          the path and reads as an unstable gain rather than a wrong
     *          transform.
     */
    inline Vec2 toLocal(const Pose& p, const Vec2 world)
    {
        const Float32 dx = world.x - p.x;
        const Float32 dy = world.y - p.y;

        const Float32 c = cosf(p.heading);
        const Float32 s = sinf(p.heading);

        Vec2 out;
        out.x = dx * c + dy * s;
        out.y = -dx * s + dy * c;
        return out;
    }

    /**
     * @brief The inverse of toLocal(): a car-frame point back on the map.
     *
     * For turning something the car worked out about itself into a place in the
     * world.
     *
     * @param p where the car is and which way it faces
     * @param local the point in the car's frame, meters
     * @return the same point in world meters
     */
    inline Vec2 toWorld(const Pose& p, const Vec2 local)
    {
        const Float32 c = cosf(p.heading);
        const Float32 s = sinf(p.heading);

        Vec2 out;
        out.x = p.x + local.x * c - local.y * s;
        out.y = p.y + local.x * s + local.y * c;
        return out;
    }

}
