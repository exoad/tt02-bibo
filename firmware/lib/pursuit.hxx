/* ---------------------------------------------------------------------------
 * pursuit - pure pursuit, the path follower.
 *
 * Look a fixed distance ahead along the path, aim at that point, drive the arc
 * that reaches it. That is the whole algorithm, and its virtue is that it has
 * one tuning knob with a physical meaning instead of three gains with none.
 *
 * curvature = 2 * y / L^2
 *
 * where y is how far LEFT the goal is in the car's own frame and L is the
 * lookahead. It falls straight out of the geometry: the unique circle through
 * the car's rear axle, tangent to its heading, passing through the goal.
 *
 * Pure arithmetic. Compiles for the Pico, the Orange Pi and the host test.
 *
 * ---------------------------------------------------------------------------
 * THE LOOKAHEAD IS THE WHOLE TUNING, AND IT SCALES WITH SPEED.
 *
 *   TOO SHORT and the car weaves. It corrects hard for an error it is already
 *   almost past, overshoots, corrects back. At a standstill a fixed short
 *   lookahead is unstable in the literal sense.
 *
 *   TOO LONG and it cuts corners. The goal is past the bend before the car has
 *   started turning, so it drives the chord and clips the inside.
 *
 * Scaling with speed is what resolves that: the car needs roughly a constant
 * TIME to react, so the distance it should look ahead is proportional to how
 * fast it is closing on it. The floor matters as much as the gain - at zero
 * speed the product is zero, and a zero lookahead divides by zero in the
 * curvature.
 *
 * ---------------------------------------------------------------------------
 * THE SEARCH ONLY EVER MOVES FORWARD.
 *
 * A path that crosses itself - a figure of eight, or any lap - has two points
 * at the lookahead distance, and the nearest-point search will happily pick the
 * one from the wrong lap. Holding the index and searching only forward from it
 * makes the follower commit to the path in order.
 *
 * The cost is that a car knocked far off course cannot recover by itself: the
 * index has already passed the part of the path it is now nearest to. That is
 * the right trade for a follower - rejoining a path is a planner's job, and a
 * follower that silently teleports to a different lap is worse than one that
 * reports it is lost.
 * ------------------------------------------------------------------------- */
#pragma once

#include "geom.hxx"
#include "kinematics.hxx"

namespace bibo
{

  namespace pursuit
  {

    /* A path is somebody else's array. Not owned, not copied - this runs on a
     * microcontroller and on a single-board computer, and neither wants a
     * follower that allocates. */
    struct Path
    {
        const geom::Vec2* pts = nullptr;
        Size              n   = 0u;
    };

    struct Follower
    {
        /* Lookahead = clamp(perMs * speed, minM, maxM). */
        Float32 minM  = 0.35f;
        Float32 maxM  = 1.50f;
        Float32 perMs = 0.7f;    /* metres of lookahead per m/s of speed */

        /* Within this of the last point, the path is finished. */
        Float32 arriveM = 0.25f;

        Float32 wheelbase = KIN_WHEELBASE_M;
        Float32 maxSteer  = KIN_MAX_STEER_RAD;

        /* How far along the path the follower has committed. Only increases. */
        Size    at = 0u;
    };

    struct Aim
    {
        Bool       valid     = false;  /* false: nothing to steer toward */
        Bool       arrived   = false;  /* the end is within arriveM      */
        geom::Vec2 goal;               /* world frame                    */
        Float32    lookahead = 0.0f;
        Float32    curvature = 0.0f;   /* 1/metres, + is left            */
        Float32    steer     = 0.0f;   /* radians, clamped to maxSteer   */
        Float32    fraction  = 0.0f;   /* -1..1, what drive::steer wants */
        Float32    crossTrack = 0.0f;  /* metres, + is left of the path  */
    };

    inline Void reset(Follower* f)
    {
        if(f != nullptr)
        {
            f->at = 0u;
        }
    }

    /* ---------------------------------------------------------------------
     * One step. Returns what to steer, or valid=false if there is no path.
     * ------------------------------------------------------------------- */
    inline Aim follow(Follower* f, const Path* path, geom::Pose pose, Float32 speed)
    {
        Aim aim;

        if(f == nullptr || path == nullptr || path->pts == nullptr || path->n == 0u)
        {
            return aim;
        }

        /* Speed-scaled, floored, capped. The floor is load-bearing: at rest the
         * product is zero and the curvature would divide by it. */
        Float32 ld = f->perMs * ((speed < 0.0f) ? -speed : speed);
        if(ld < f->minM)
        {
            ld = f->minM;
        }
        if(ld > f->maxM)
        {
            ld = f->maxM;
        }
        aim.lookahead = ld;

        const geom::Vec2 last = path->pts[path->n - 1u];

        /* ARRIVED is checked against the END, not the index. A follower that
         * decided it had arrived because it ran out of path would also decide
         * that the moment it lost track of where it was. */
        if(geom::distance(geom::Vec2{ pose.x, pose.y }, last) <= f->arriveM)
        {
            aim.valid   = true;
            aim.arrived = true;
            aim.goal    = last;
            return aim;
        }

        /* ---- advance the committed index ---------------------------------
         *
         * Forward only. Walks past everything already within the lookahead, so
         * the goal is the first point still further away than ld. */
        const Float32 ldSq = ld * ld;
        const geom::Vec2 here{ pose.x, pose.y };

        Size i = f->at;
        while((i + 1u) < path->n
              && geom::distanceSq(here, path->pts[i]) <= ldSq)
        {
            ++i;
        }
        f->at = i;

        /* ---- pick the goal ------------------------------------------------
         *
         * The first point at or beyond the lookahead, searching forward. If the
         * path ends inside the lookahead - which it does on every approach to
         * the finish - the last point IS the goal, so the car drives to the end
         * instead of stopping short of it.
         *
         * A point BEHIND the car is skipped. Aiming at one produces a curvature
         * that turns the car around, which is both wrong and alarming. */
        Bool       got = false;
        geom::Vec2 goal = last;

        for(Size k = i; k < path->n; ++k)
        {
            const geom::Vec2 local = geom::toLocal(pose, path->pts[k]);
            if(local.x <= 0.0f)
            {
                continue;   /* behind the rear axle */
            }
            if(geom::distanceSq(here, path->pts[k]) >= ldSq)
            {
                goal = path->pts[k];
                got  = true;
                break;
            }
        }

        if(!got)
        {
            /* Nothing far enough ahead: aim at the end. Only valid if the end
             * is actually in front, or the car would reverse toward it. */
            const geom::Vec2 localEnd = geom::toLocal(pose, last);
            if(localEnd.x <= 0.0f)
            {
                aim.valid = false;
                return aim;
            }
            goal = last;
        }

        /* ---- the geometry -------------------------------------------------
         *
         * curvature = 2y / L^2, with y the goal's offset in the car's frame.
         * L is the ACTUAL distance to the goal rather than the nominal
         * lookahead - when the goal is the last point it can be nearer than ld,
         * and using ld there would understate the curvature and cut the corner
         * at exactly the moment precision matters most. */
        const geom::Vec2 local = geom::toLocal(pose, goal);
        const Float32    dist  = geom::distance(here, goal);
        const Float32    use   = (dist > 0.01f) ? dist : 0.01f;

        aim.goal       = goal;
        aim.crossTrack = local.y;
        aim.curvature  = (2.0f * local.y) / (use * use);
        aim.steer      = kin::clampSteer(
                             kin::steerFor(aim.curvature, f->wheelbase),
                             f->maxSteer);
        aim.fraction   = kin::steerFraction(aim.steer, f->maxSteer);
        aim.valid      = true;
        return aim;
    }

  } /* namespace pursuit */

} /* namespace bibo */
