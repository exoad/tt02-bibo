/* ---------------------------------------------------------------------------
 * geom - where the car is, in metres and radians.
 *
 * WORLD FRAME, not pixels. gfx.hxx has a Point and it is a screen coordinate;
 * this is the other kind, and keeping them in separate files with separate
 * names is what stops one being passed where the other is meant.
 *
 * Pure arithmetic - no SDK, no clock, no hardware. Which means it compiles for
 * the Pico, for the Orange Pi, and for the host test, from one copy. That is
 * deliberate and it is the reason none of this reaches for a peripheral: the
 * moment a controller needs a timestamp of its own, it stops being portable and
 * starts being firmware.
 *
 * ---------------------------------------------------------------------------
 * ANGLES ARE RADIANS AND ARE WRAPPED.
 *
 * A heading that accumulates without wrapping reaches 200 rad after a few
 * minutes of driving in circles, and every comparison against it starts
 * behaving oddly for reasons that look like sensor drift. Worse, the difference
 * between 179 degrees and -179 degrees is 2 degrees the short way and 358 the
 * long way, and a controller that takes the long way turns the car all the way
 * round to reach a heading it was almost at. wrapPi exists for exactly that.
 * ------------------------------------------------------------------------- */
#pragma once

#include "types.hxx"

/* sqrtf, sinf, cosf, atanf. The float forms deliberately: the double ones pull
 * in software double-precision on an M33, which is both slower and larger for
 * arithmetic that is never more accurate than the encoder feeding it. */
#include <math.h>

namespace bibo::geom
{

#define GEOM_PI  3.14159265358979f
#define GEOM_TAU 6.28318530717959f

    struct Vec2
    {
        Float32 x = 0.0f;   /* metres */
        Float32 y = 0.0f;
    };

    /* Where the car is and which way it faces. Heading is radians, zero along
     * +x, increasing toward +y - the ordinary mathematical convention rather
     * than a compass bearing, so that cos/sin work without a sign flip. */
    struct Pose
    {
        Float32 x       = 0.0f;
        Float32 y       = 0.0f;
        Float32 heading = 0.0f;
    };

    /* ---- angles -----------------------------------------------------------
     *
     * Into (-pi, pi]. A loop rather than fmod: fmod on a negative gives a
     * negative remainder, so the naive one-liner is wrong for exactly half the
     * inputs, and the loop runs at most a couple of times for any angle a
     * vehicle actually produces. */
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

    /* The SHORT way from `from` to `to`. This is the one a heading controller
     * wants; plain subtraction is the one that drives the long way round. */
    inline Float32 angleDelta(const Float32 from, const Float32 to)
    {
        return wrapPi(to - from);
    }

    /* ---- distances --------------------------------------------------------
     *
     * The squared form is offered because most uses COMPARE distances, and a
     * comparison does not need the square root - which on an M33 without a
     * hardware divide is worth avoiding inside a loop over a path. */
    inline Float32 distanceSq(const Vec2 a, const Vec2 b)
    {
        const Float32 dx = b.x - a.x;
        const Float32 dy = b.y - a.y;
        return dx * dx + dy * dy;
    }

    inline Float32 distance(const Vec2 a, const Vec2 b)
    {
        return sqrtf(distanceSq(a, b));
    }

    /* ---- frames -----------------------------------------------------------
     *
     * A world point in the CAR's frame: +x straight ahead, +y to the left.
     *
     * This is the transform pure pursuit is built on, and the sign of the
     * result is the whole answer - a goal with positive y is to the left and
     * the car steers left. Getting the rotation backwards produces a controller
     * that steers away from the path and looks like an unstable gain. */
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

    /* The inverse, for turning something the car worked out about itself back
     * into a place on the map. */
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
