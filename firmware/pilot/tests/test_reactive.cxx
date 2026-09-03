// Reactive driving from a lidar scan.
//
//   tests\build_reactive_test.bat run
//
// Pure arithmetic over a plain array, so the whole behaviour is exercised here
// with synthetic scans - no lidar, no car, no clock, and no wall to drive into
// while finding out whether the sign of the steering was right.
//
// THE CASES THAT MATTER ARE NOT THE HAPPY ONES. They are: a scan with nothing
// in it, a scan of zeroes, one spurious near return, and the sign of the
// steering while reversing. Each of those returns something plausible under the
// obvious implementation, and each of them drives the car into something.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"

#include "reactive.hxx"

#include <cstdio>

static Int32 failures = 0;
static Int32 checks = 0;

static Void check(Bool ok, const Char* what)
{
    ++checks;
    if(ok)
    {
        std::printf("  ok    %s\n", what);
    }
    else
    {
        std::printf("  FAIL  %s\n", what);
        ++failures;
    }
}

// ---- scan building --------------------------------------------------------

// A full revolution at one distance: the inside of a round room.
static Vec<reactive::Ray> ring(const Float32 distMm)
{
    Vec<reactive::Ray> out;
    for(Int32 a = 0; a < 360; ++a)
    {
        reactive::Ray r;
        r.angleDeg = static_cast<Float32>(a);
        r.distMm = distMm;
        out.push_back(r);
    }
    return out;
}

// Adds returns over a bearing range, in the same POSITIVE-IS-RIGHT sense the
// module uses, so a test reads the way the geometry does.
static Void arc(Vec<reactive::Ray>& s, Float32 fromDeg, Float32 toDeg, Float32 distMm)
{
    for(Float32 b = fromDeg; b <= toDeg; b += 1.0f)
    {
        reactive::Ray r;
        r.angleDeg = (b < 0.0f) ? (b + 360.0f) : b;
        r.distMm = distMm;
        s.push_back(r);
    }
}

static reactive::Status drive(
    const Vec<reactive::Ray>& s,
    Int32 dtMs,
    reactive::State* st,
    reactive::Outputs* out
)
{
    return reactive::step(s.data(), s.size(), dtMs, st, out);
}

Int32 main()
{
    std::printf("\nreactive - lidar in, throttle and steering out\n\n");

    const reactive::Config base;
    check(reactive::configure(base), "the default tuning is accepted");

    // ---- tunings that cannot be driven are REFUSED -------------------------
    {
        reactive::Config c = base;
        c.stopMm = c.slowMm + 1.0f;
        check(!reactive::configure(c), "stop further out than slow is refused");

        c = base;
        c.frontArcDeg = 120.0f;
        check(!reactive::configure(c), "a front arc past 90 degrees is refused");

        c = base;
        c.crawl = c.cruise + 0.1f;
        check(!reactive::configure(c), "a crawl faster than the cruise is refused");

        c = base;
        c.minHits = 1000;
        check(!reactive::configure(c), "more hits than the buffer holds is refused");

        c = base;
        c.reverseMaxMs = c.reverseMs - 1;
        check(!reactive::configure(c), "a reverse cap below its minimum is refused");

        check(
            reactive::tuning().stopMm == base.stopMm,
            "and a refused tuning leaves the old one installed"
        );
    }

    // ---- TRAP 1: an empty scan is not an empty room ------------------------
    {
        reactive::State   st;
        reactive::Outputs out;
        out.throttle = 0.9f;   // whatever the caller had before
        out.stop = false;

        const Vec<reactive::Ray> none;
        const reactive::Status   s = drive(none, 20, &st, &out);

        check(s == reactive::Status::STATUS_BLIND, "an empty scan reports BLIND");
        check(out.mode == reactive::Mode::MODE_BLIND, "and the mode says so");
        check(out.stop, "and the car is stopped");
        check(out.throttle == 0.0f, "and the caller's throttle is overwritten, not left");
    }

    // ---- TRAP 2: distMm == 0 is "no return", not "touching the bumper" -----
    {
        reactive::State          st;
        reactive::Outputs        out;
        const Vec<reactive::Ray> zeros = ring(0.0f);

        const reactive::Status s = drive(zeros, 20, &st, &out);
        check(s == reactive::Status::STATUS_BLIND, "a scan of zeroes is BLIND, not an obstacle");
        check(out.stop, "and stops the car rather than reporting a clear road");
    }

    // ---- the ordinary cases ------------------------------------------------
    {
        reactive::State   st;
        reactive::Outputs out;

        check(
            drive(ring(3000.0f), 20, &st, &out) == reactive::Status::STATUS_OK,
            "an open room drives"
        );
        check(out.mode == reactive::Mode::MODE_CRUISE, "and cruises");
        check(out.throttle == base.cruise, "at the cruise throttle");
        check(!out.stop, "and does not ask for STOP");
    }
    {
        reactive::State   st;
        reactive::Outputs out;
        static_cast<Void>(drive(ring(1000.0f), 20, &st, &out));
        check(out.mode == reactive::Mode::MODE_SLOW, "a wall at 1 m slows");
        check(
            out.throttle > base.crawl && out.throttle < base.cruise,
            "to between the crawl and the cruise"
        );
    }
    {
        reactive::State   st;
        reactive::Outputs out;
        static_cast<Void>(drive(ring(300.0f), 20, &st, &out));
        check(out.mode == reactive::Mode::MODE_STOP, "a wall at 300 mm stops");
        check(out.throttle == 0.0f, "with no throttle");
        check(out.stop, "and asks for STOP");
    }
    {
        reactive::State   st;
        reactive::Outputs out;
        static_cast<Void>(drive(ring(200.0f), 20, &st, &out));
        check(out.mode == reactive::Mode::MODE_REVERSE, "a wall at 200 mm reverses");
        check(out.throttle < 0.0f, "with a negative throttle");
        check(!out.stop, "which is motion, so not a STOP");
    }

    // ---- TRAP 3: one bad point is not an obstacle --------------------------
    {
        reactive::State   st;
        reactive::Outputs out;

        Vec<reactive::Ray> s = ring(3000.0f);
        reactive::Ray      speck;
        speck.angleDeg = 0.0f;
        speck.distMm = 150.0f;   // well inside the reverse threshold
        s.push_back(speck);

        static_cast<Void>(drive(s, 20, &st, &out));
        check(
            out.mode == reactive::Mode::MODE_CRUISE,
            "a single spurious near return does not brake the car"
        );
    }
    {
        // ...but a real object, seen by enough rays, does.
        reactive::State   st;
        reactive::Outputs out;

        Vec<reactive::Ray> s = ring(3000.0f);
        arc(s, -2.0f, 2.0f, 200.0f);   // five rays, above the default minHits

        static_cast<Void>(drive(s, 20, &st, &out));
        check(
            out.mode == reactive::Mode::MODE_REVERSE,
            "an object seen by several rays is believed"
        );
    }

    // ---- the corridor, not a cone ------------------------------------------
    {
        reactive::State   st;
        reactive::Outputs out;

        // 800 mm away at 60 degrees is 693 mm to the side - wide of a 160 mm
        // half width, so the car should drive straight past it rather than
        // treating it as something in the way.
        Vec<reactive::Ray> s = ring(3000.0f);
        arc(s, 55.0f, 65.0f, 800.0f);

        static_cast<Void>(drive(s, 20, &st, &out));
        check(
            out.mode == reactive::Mode::MODE_CRUISE,
            "an object beside the car is not an object in front of it"
        );
    }

    // ---- steering goes toward the room -------------------------------------
    {
        reactive::State   st;
        reactive::Outputs out;

        Vec<reactive::Ray> s = ring(3000.0f);
        arc(s, 20.0f, 70.0f, 500.0f);   // close on the RIGHT

        static_cast<Void>(drive(s, 20, &st, &out));
        check(out.steer < 0.0f, "a wall on the right steers left");
    }
    {
        reactive::State   st;
        reactive::Outputs out;

        Vec<reactive::Ray> s = ring(3000.0f);
        arc(s, -70.0f, -20.0f, 500.0f);   // close on the LEFT

        static_cast<Void>(drive(s, 20, &st, &out));
        check(out.steer > 0.0f, "a wall on the left steers right");
    }
    {
        reactive::State   st;
        reactive::Outputs out;
        static_cast<Void>(drive(ring(3000.0f), 20, &st, &out));
        check(
            out.steer > -0.05f && out.steer < 0.05f,
            "equal room either side steers straight"
        );
    }

    // ---- THE SIGN THAT IS EASY TO GET WRONG --------------------------------
    //
    // Backing up, the nose swings OPPOSITE the wheels. To end up facing the
    // side with room, the wheels must point the other way. Getting this
    // backwards reverses the car deeper into the corner it is escaping.
    {
        reactive::State   st;
        reactive::Outputs out;

        // Built by hand rather than from ring(): a ring puts near returns on
        // BOTH sides, so "room on the left" has to be the absence of them, not
        // a far arc laid over the top of them. The side room is a minimum, and
        // a minimum does not care what else you added.
        Vec<reactive::Ray> s;
        arc(s, -80.0f, -12.0f, 2500.0f);   // left, and the whole side band: far
        arc(s, -11.0f, 80.0f, 200.0f);     // straight ahead and right: close
        arc(s, 100.0f, 260.0f, 3000.0f);   // behind, so the scan is trusted

        static_cast<Void>(drive(s, 20, &st, &out));
        check(out.mode == reactive::Mode::MODE_REVERSE, "boxed in, the car reverses");
        check(
            out.steer > 0.0f,
            "and with room on the LEFT it steers RIGHT, so reversing swings the nose left"
        );
    }

    // ---- a reverse is committed to, not reconsidered every tick ------------
    {
        reactive::State   st;
        reactive::Outputs out;

        static_cast<Void>(drive(ring(200.0f), 20, &st, &out));
        check(out.mode == reactive::Mode::MODE_REVERSE, "reversing");

        // The road is suddenly clear - but not for long enough yet.
        static_cast<Void>(drive(ring(3000.0f), 100, &st, &out));
        check(
            out.mode == reactive::Mode::MODE_REVERSE,
            "a clear road one tick later does not cancel the reverse"
        );

        // Past the commitment window, it lets go.
        for(Int32 i = 0; i < 10; ++i)
        {
            static_cast<Void>(drive(ring(3000.0f), 100, &st, &out));
        }
        check(out.mode != reactive::Mode::MODE_REVERSE, "but it does let go once committed time has passed");
    }
    {
        // Wedged: still blocked when the cap runs out, so it gives up rather
        // than grinding backwards into whatever is behind it.
        reactive::State   st;
        reactive::Outputs out;

        for(Int32 i = 0; i < 40; ++i)
        {
            static_cast<Void>(drive(ring(200.0f), 100, &st, &out));
        }
        check(
            out.mode == reactive::Mode::MODE_STOP,
            "a reverse that never finds room gives up and stops"
        );
    }

    // ---- hysteresis: no stuttering at the threshold ------------------------
    {
        reactive::State   st;
        reactive::Outputs out;

        static_cast<Void>(drive(ring(300.0f), 20, &st, &out));
        check(out.mode == reactive::Mode::MODE_STOP, "stopped at 300 mm");

        // Just over the stop line, but inside the hysteresis band.
        static_cast<Void>(drive(ring(base.stopMm + 40.0f), 20, &st, &out));
        check(
            out.mode == reactive::Mode::MODE_STOP,
            "a hair past the stop line does not set off again"
        );

        // Clear of the band, it goes.
        static_cast<Void>(drive(ring(base.stopMm + base.hysteresisMm + 200.0f), 20, &st, &out));
        check(out.mode != reactive::Mode::MODE_STOP, "well past it, the car moves");
    }

    // ---- mounting is a tuning ----------------------------------------------
    {
        // The same room, with the lidar bolted on backwards: an obstacle at raw
        // 180 is dead ahead once forwardDeg says so.
        reactive::Config c = base;
        c.forwardDeg = 180.0f;
        check(reactive::configure(c), "a rotated mounting configures");

        reactive::State   st;
        reactive::Outputs out;
        Vec<reactive::Ray> s = ring(3000.0f);
        arc(s, 178.0f, 182.0f, 200.0f);

        static_cast<Void>(drive(s, 20, &st, &out));
        check(
            out.mode == reactive::Mode::MODE_REVERSE,
            "and an obstacle at raw 180 is then straight ahead"
        );

        check(reactive::configure(base), "and the base tuning restores");
    }

    // ---- null arguments are refused, not dereferenced ----------------------
    {
        reactive::Outputs out;
        reactive::State   st;
        check(
            reactive::step(nullptr, 0, 20, &st, &out) == reactive::Status::STATUS_BLIND,
            "a null ray array is BLIND rather than a crash"
        );
        check(
            reactive::step(nullptr, 0, 20, nullptr, &out) == reactive::Status::STATUS_BLIND,
            "so is a null state"
        );
    }

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
