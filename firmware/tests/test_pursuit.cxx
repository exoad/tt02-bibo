/* geom, the bicycle model and pure pursuit.
 *
 *   firmware\tests\build_pursuit_test.bat run
 *
 * EVERY FAILURE IN THIS FILE IS SILENT ON A CAR. A rotation with the wrong sign
 * steers away from the path and reads as an unstable gain. A lookahead that
 * collapses to zero divides by zero once and then does something arbitrary
 * forever. An index that can move backwards makes a figure-of-eight follow the
 * wrong lap. None of them throws; all of them look like tuning.
 *
 * Which is why the cases below are mostly about SIGN and about the degenerate
 * inputs a path produces at its ends.
 *
 * Compiled for the HOST. None of the three headers touches the SDK - the same
 * code runs on the Pico and will run on the Orange Pi.
 *
 * Exits 0 on PASS, 1 on FAIL.
 */

#include "../lib/pursuit.hxx"

#include <stdio.h>
#include <math.h>

using namespace bibo;

static Int32 failures = 0;
static Int32 checks   = 0;

static Void check(Bool ok, CharSeq what)
{
    ++checks;
    if(ok)
    {
        printf("  ok    %s\n", what);
    }
    else
    {
        printf("  FAIL  %s\n", what);
        ++failures;
    }
}

static Bool near(Float32 a, Float32 b, Float32 tol)
{
    const Float32 d = a - b;
    return ((d < 0.0f) ? -d : d) <= tol;
}

int main(Void)
{
    printf("\ngeom\n\n");

    check(near(geom::wrapPi(0.0f), 0.0f, 1e-5f), "zero wraps to zero");
    check(near(geom::wrapPi(GEOM_PI + 0.1f), -GEOM_PI + 0.1f, 1e-4f),
          "just past pi wraps to just past -pi");
    check(near(geom::wrapPi(-GEOM_PI - 0.1f), GEOM_PI - 0.1f, 1e-4f),
          "and the other way");
    check(near(geom::wrapPi(20.0f * GEOM_TAU + 0.3f), 0.3f, 1e-3f),
          "twenty turns of accumulated heading still wraps");

    /* THE SHORT WAY. Plain subtraction gives 358 degrees here and turns the car
     * all the way round to reach a heading it was two degrees from. */
    const Float32 d = geom::angleDelta(GEOM_PI - 0.05f, -GEOM_PI + 0.05f);
    check(near(d, 0.1f, 1e-4f), "the delta across the wrap is the short way");

    /* +y IS LEFT. Getting this backwards is the sign error that makes pursuit
     * steer away from the path. */
    geom::Pose at0;
    geom::Vec2 leftOfCar{ 0.0f, 1.0f };
    check(geom::toLocal(at0, leftOfCar).y > 0.0f,
          "a point to the left is +y in the car frame");

    geom::Pose facingY;
    facingY.heading = GEOM_PI / 2.0f;
    const geom::Vec2 ahead = geom::toLocal(facingY, geom::Vec2{ 0.0f, 1.0f });
    check(near(ahead.x, 1.0f, 1e-4f) && near(ahead.y, 0.0f, 1e-4f),
          "facing +y, a point at +y is straight ahead");

    /* Round trip. */
    geom::Pose odd;
    odd.x = 1.5f; odd.y = -2.0f; odd.heading = 0.9f;
    const geom::Vec2 there{ 3.0f, 4.0f };
    const geom::Vec2 back = geom::toWorld(odd, geom::toLocal(odd, there));
    check(near(back.x, there.x, 1e-3f) && near(back.y, there.y, 1e-3f),
          "toLocal and toWorld are inverses");

    printf("\nkinematics\n\n");

    check(near(kin::curvatureFor(0.0f, 0.257f), 0.0f, 1e-6f),
          "straight wheels is zero curvature");
    check(kin::curvatureFor(0.3f, 0.257f) > 0.0f,
          "a left steer is a positive curvature");

    const Float32 k = kin::curvatureFor(0.3f, 0.257f);
    check(near(kin::steerFor(k, 0.257f), 0.3f, 1e-4f),
          "steerFor and curvatureFor are inverses");

    check(near(kin::curvatureFor(0.3f, 0.0f), 0.0f, 1e-6f),
          "a zero wheelbase is 0, not a division by zero");

    check(near(kin::steerFraction(KIN_MAX_STEER_RAD, KIN_MAX_STEER_RAD), 1.0f, 1e-4f),
          "full lock is a fraction of 1");
    check(near(kin::steerFraction(2.0f, KIN_MAX_STEER_RAD), 1.0f, 1e-4f),
          "past full lock is clamped, not extrapolated");

    /* DEAD RECKONING. Straight first, because the arc form must not disturb it. */
    geom::Pose p;
    p = kin::integrate(p, 1.0f, 0.0f, 0.257f, 1.0f);
    check(near(p.x, 1.0f, 1e-4f) && near(p.y, 0.0f, 1e-4f),
          "one second at 1 m/s straight is one metre along +x");
    check(near(p.heading, 0.0f, 1e-5f), "and the heading has not moved");

    /* A LEFT TURN GOES LEFT. Sign again - the one that matters. */
    geom::Pose t;
    t = kin::integrate(t, 1.0f, 0.3f, 0.257f, 0.1f);
    check(t.heading > 0.0f, "a positive steer turns toward +heading");
    check(t.y > 0.0f, "and moves the car to the left");

    /* A FULL CIRCLE RETURNS. This is the test that catches an arc integrator
     * that is subtly a chord integrator: the chord version closes short, and
     * the error accumulates in one direction rather than averaging out. */
    geom::Pose c;
    const Float32 steer = 0.3f;
    const Float32 curv  = kin::curvatureFor(steer, 0.257f);
    const Float32 circ  = GEOM_TAU / curv;          /* metres round the circle */
    const Int32   steps = 2000;
    const Float32 dt    = (circ / 1.0f) / static_cast<Float32>(steps);

    for(Int32 i = 0; i < steps; ++i)
    {
        c = kin::integrate(c, 1.0f, steer, 0.257f, dt);
    }
    check(near(c.x, 0.0f, 0.02f) && near(c.y, 0.0f, 0.02f),
          "a full circle comes back to where it started");

    check(kin::minTurnRadius(KIN_MAX_STEER_RAD, 0.257f) > 0.0f,
          "the tightest circle has a positive radius");

    printf("\npursuit\n\n");

    /* A straight path along +x. */
    geom::Vec2 line[11];
    for(Int32 i = 0; i < 11; ++i)
    {
        line[i].x = static_cast<Float32>(i) * 0.5f;
        line[i].y = 0.0f;
    }
    pursuit::Path path;
    path.pts = line;
    path.n   = 11u;

    pursuit::Follower f;
    pursuit::reset(&f);

    /* ON the line, facing along it: straight ahead. */
    geom::Pose on;
    pursuit::Aim a = pursuit::follow(&f, &path, on, 1.0f);
    check(a.valid, "a car on the path has something to steer toward");
    check(near(a.curvature, 0.0f, 1e-3f), "and steers straight");

    /* OFF to the RIGHT of the line must steer LEFT. This single assertion is
     * the one that catches a sign error anywhere in the chain - the frame
     * rotation, the curvature formula, or the steering conversion. */
    pursuit::reset(&f);
    geom::Pose right;
    right.y = -0.3f;
    pursuit::Aim l = pursuit::follow(&f, &path, right, 1.0f);
    check(l.valid && l.curvature > 0.0f,
          "a car right of the path steers LEFT to rejoin it");
    check(l.crossTrack > 0.0f, "and reports the path as being to its left");

    pursuit::reset(&f);
    geom::Pose left;
    left.y = 0.3f;
    pursuit::Aim r = pursuit::follow(&f, &path, left, 1.0f);
    check(r.valid && r.curvature < 0.0f, "and left of the path steers right");

    /* THE LOOKAHEAD FLOOR. At a standstill the speed-scaled product is zero,
     * and a zero lookahead divides by zero in the curvature. */
    pursuit::reset(&f);
    pursuit::Aim stopped = pursuit::follow(&f, &path, on, 0.0f);
    check(stopped.lookahead >= f.minM,
          "at rest the lookahead is the floor, not zero");
    check(!isnan(stopped.curvature) && !isinf(stopped.curvature),
          "so the curvature is a real number");

    /* And it grows with speed, up to the cap. */
    pursuit::reset(&f);
    const Float32 slow = pursuit::follow(&f, &path, on, 0.6f).lookahead;
    pursuit::reset(&f);
    const Float32 fast = pursuit::follow(&f, &path, on, 1.8f).lookahead;
    check(fast > slow, "a faster car looks further ahead");
    pursuit::reset(&f);
    check(pursuit::follow(&f, &path, on, 50.0f).lookahead <= f.maxM + 1e-4f,
          "and never further than the cap");

    /* THE INDEX ONLY MOVES FORWARD. Drive along, then ask again from behind:
     * the follower must not rewind to an earlier part of the path. */
    pursuit::reset(&f);
    geom::Pose along;
    for(Int32 i = 0; i < 8; ++i)
    {
        along.x = static_cast<Float32>(i) * 0.5f;
        static_cast<Void>(pursuit::follow(&f, &path, along, 1.0f));
    }
    const Size advanced = f.at;
    check(advanced > 0u, "the committed index advanced along the path");

    geom::Pose backAtStart;
    static_cast<Void>(pursuit::follow(&f, &path, backAtStart, 1.0f));
    check(f.at >= advanced, "and never goes backwards");

    /* ARRIVAL is measured against the END, not the index. */
    pursuit::reset(&f);
    geom::Pose atEnd;
    atEnd.x = 5.0f;
    pursuit::Aim done = pursuit::follow(&f, &path, atEnd, 0.5f);
    check(done.valid && done.arrived, "reaching the last point is arrival");

    /* DEGENERATE INPUTS. */
    pursuit::Path empty;
    check(!pursuit::follow(&f, &empty, on, 1.0f).valid,
          "an empty path is not valid, and does not crash");
    check(!pursuit::follow(&f, nullptr, on, 1.0f).valid, "nor is a null path");
    check(!pursuit::follow(nullptr, &path, on, 1.0f).valid, "nor a null follower");

    /* A path entirely BEHIND the car. Aiming at it would spin the car round. */
    pursuit::reset(&f);
    geom::Pose past;
    past.x = 20.0f;
    pursuit::Aim behind = pursuit::follow(&f, &path, past, 1.0f);
    check(!behind.valid || behind.arrived,
          "a path wholly behind the car is refused, not turned toward");

    printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
