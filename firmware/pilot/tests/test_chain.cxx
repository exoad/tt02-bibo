// chain: the loaded behaviours and the one steer and throttle they resolve to,
// held to chain.hxx with synthetic scans. No board, no viewer, no clock.
//
//     tools\test.bat chain run
//
// The three cases docs/bundles.md section 9 names as the whole safety argument
// are the point of this file: a clamp beats a driver, order does not change the
// answer for clamps, and no chain can raise authority. The test doubles below
// are deliberately badly behaved - a driver that always asks for everything, a
// behaviour that proposes throttle it is not allowed - because a rule enforced
// only by well-written behaviours is not enforced.
#include "shared.hxx"

#include "chain.hxx"

#include <cmath>
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

static Bool approx(Float32 a, Float32 b)
{
    const Float32 d = a - b;
    return (d < 0.0005f) && (d > -0.0005f);
}

// A full revolution at one distance: the inside of a round room. 360 points, so
// well past SCAN_MIN_POINTS, and the strip Scan::ahead looks along holds far
// more than SCAN_MIN_HITS of them.
static bibo::Scan ring(Float32 distM)
{
    bibo::Scan s;
    s.revolution = 1;
    for(Int32 a = -180; a < 180; ++a)
    {
        bibo::Point p;
        p.bearingDeg = static_cast<Float32>(a);
        p.distanceM = distM;
        s.points.push_back(p);
    }
    return s;
}

// A room with different room to each side and the way AHEAD left clear.
// Negative bearings are LEFT, as car.hxx spells it.
//
// The bands start at 25 degrees, not 15, and that is not cosmetic. Scan::ahead
// sweeps a corridor CAR_HALF_WIDTH_M (0.16 m) wide, and a return at 0.5 m and
// 15 degrees sits 0.13 m off the axis - inside it. Bands from 15 degrees would
// put an obstacle 0.48 m in front of a scan this comment calls clear, and the
// test would be asserting against a room it had not actually built. At 25
// degrees 0.5 m is 0.21 m to the side, outside the corridor and still well
// within the band nearest() compares.
static bibo::Scan sided(Float32 leftM, Float32 rightM)
{
    bibo::Scan s;
    s.revolution = 1;
    for(Int32 a = -180; a < 180; ++a)
    {
        bibo::Point p;
        p.bearingDeg = static_cast<Float32>(a);
        p.distanceM = 3.0f;
        if(a >= -80 && a <= -25)
        {
            p.distanceM = leftM;
        }
        else if(a >= 25 && a <= 80)
        {
            p.distanceM = rightM;
        }
        s.points.push_back(p);
    }
    return s;
}

// No revolution at all: Scan::blind, and Scan::ahead answers 0.
static bibo::Scan blindScan()
{
    return bibo::Scan();
}

static chain::Pass over(const bibo::Scan& s)
{
    chain::Pass p;
    p.scan = &s;
    p.dtMs = 100;
    p.nowMs = 1000;
    return p;
}

// ---- test doubles ----------------------------------------------------------
// Asks for everything, every pass. What a clamp has to be able to stop.
class Greedy final : public chain::Behaviour
{
public:
    CharSeq id() const override
    {
        return "net.exoad.test.greedy";
    }

    CharSeq name() const override
    {
        return "greedy";
    }

    Bool mayDrive() const override
    {
        return true;
    }

    chain::Reply step(const chain::Pass&) override
    {
        return chain::propose(1.0f, 0.25f);
    }
};

// Proposes throttle while mayDrive() is false: docs/bundles.md section 3's
// `drive no`, which the host clamps rather than trusting.
class Liar final : public chain::Behaviour
{
public:
    CharSeq id() const override
    {
        return "net.exoad.test.liar";
    }

    CharSeq name() const override
    {
        return "liar";
    }

    chain::Reply step(const chain::Pass&) override
    {
        return chain::propose(1.0f, -0.5f);
    }
};

// A clamp with a ceiling chosen at construction, and an id to match, so several
// can be loaded at once and reordered.
class Ceiling final : public chain::Behaviour
{
public:
    Ceiling(CharSeq who, Float32 at)
        : who(who)
        , at(at)
    {
    }

    CharSeq id() const override
    {
        return who;
    }

    CharSeq name() const override
    {
        return who;
    }

    chain::Reply step(const chain::Pass&) override
    {
        return chain::clampTo(at);
    }

private:
    CharSeq who;
    Float32 at;
};

// Says nothing, ever: the trim bundle's shape.
class Silent final : public chain::Behaviour
{
public:
    CharSeq id() const override
    {
        return "net.exoad.test.silent";
    }

    CharSeq name() const override
    {
        return "silent";
    }

    chain::Reply step(const chain::Pass&) override
    {
        return chain::nothing();
    }
};

// Every float it can reach is NaN.
class Nan final : public chain::Behaviour
{
public:
    CharSeq id() const override
    {
        return "net.exoad.test.nan";
    }

    CharSeq name() const override
    {
        return "nan";
    }

    Bool mayDrive() const override
    {
        return true;
    }

    chain::Reply step(const chain::Pass&) override
    {
        const Float32 bad = std::nanf("");
        return chain::propose(bad, bad);
    }
};

static Bool add(chain::Chain& c, UniqPtr<chain::Behaviour> b)
{
    Str why;
    return c.load(std::move(b), why);
}

static UniqPtr<chain::Behaviour> ceilingAt(CharSeq who, Float32 at)
{
    return makeUniq<Ceiling>(who, at);
}

Int32 main()
{
    std::printf("\nchain - loaded behaviours in, one steer and throttle out\n\n");
    std::printf("-- an empty chain --\n");
    {
        chain::Chain c;
        const bibo::Scan s = ring(3.0f);
        const chain::Outcome o = c.run(over(s));
        check(c.size() == 0, "nothing is loaded");
        check(!o.drive, "an empty chain proposes NOTHING, so the host calls nothing");
        check(o.throttle == 0.0f, "and the throttle it resolves to is 0");
        check(o.intent.ceiling == 1.0f, "with authority never having been reduced");
    }
    std::printf("\n-- a clamp beats a driver --\n");
    {
        chain::Chain c;
        check(add(c, makeUniq<Greedy>()), "a driver that always asks for everything loads");
        const bibo::Scan clear = ring(3.0f);
        const chain::Outcome alone = c.run(over(clear));
        check(alone.drive && approx(alone.throttle, 1.0f), "alone, it gets everything it asked");
        check(add(c, chain::makeStop()), "and the stop clamp loads beside it");
        const chain::Outcome stillClear = c.run(over(clear));
        check(approx(stillClear.throttle, 1.0f), "with the way clear the clamp takes nothing");
        const bibo::Scan wall = ring(0.30f);
        const chain::Outcome blocked = c.run(over(wall));
        check(blocked.throttle == 0.0f, "with a wall at 300 mm the driver gets ZERO");
        check(blocked.drive, "but something did propose, so this is an active zero");
        check(
            approx(blocked.steer, 0.25f),
            "and the driver keeps the steering: a clamp never steers"
        );
        check(blocked.clamped == 1, "and the pass reports that a proposal was cut down");
    }
    std::printf("\n-- a clamp binds a driver loaded AFTER it --\n");
    {
        // The ordering trap: were ceilings applied as they were met rather than
        // at the end, a driver loaded later would simply overwrite them.
        chain::Chain c;
        check(add(c, chain::makeStop()), "the clamp loads first");
        check(add(c, makeUniq<Greedy>()), "and the driver after it");
        const bibo::Scan wall = ring(0.30f);
        const chain::Outcome o = c.run(over(wall));
        check(o.throttle == 0.0f, "load order does not get a driver out from under a clamp");
    }
    std::printf("\n-- order does not change the answer for clamps --\n");
    {
        const bibo::Scan s = ring(3.0f);
        chain::Chain a;
        static_cast<Void>(add(a, makeUniq<Greedy>()));
        static_cast<Void>(add(a, ceilingAt("net.exoad.test.half", 0.5f)));
        static_cast<Void>(add(a, ceilingAt("net.exoad.test.quarter", 0.25f)));
        chain::Chain b;
        static_cast<Void>(add(b, makeUniq<Greedy>()));
        static_cast<Void>(add(b, ceilingAt("net.exoad.test.quarter", 0.25f)));
        static_cast<Void>(add(b, ceilingAt("net.exoad.test.half", 0.5f)));
        chain::Chain d;
        static_cast<Void>(add(d, ceilingAt("net.exoad.test.quarter", 0.25f)));
        static_cast<Void>(add(d, makeUniq<Greedy>()));
        static_cast<Void>(add(d, ceilingAt("net.exoad.test.half", 0.5f)));
        const chain::Outcome oa = a.run(over(s));
        const chain::Outcome ob = b.run(over(s));
        const chain::Outcome od = d.run(over(s));
        check(approx(oa.throttle, 0.25f), "two clamps and a driver resolve to the lower ceiling");
        check(approx(oa.throttle, ob.throttle), "swapping the two clamps changes nothing");
        check(approx(oa.throttle, od.throttle), "nor does moving the driver between them");
        check(
            approx(oa.steer, ob.steer) && approx(oa.steer, od.steer),
            "and the steering agrees too"
        );
    }
    std::printf("\n-- no chain can raise authority --\n");
    {
        const bibo::Scan s = ring(3.0f);
        chain::Chain c;
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        Float32 last = c.run(over(s)).throttle;
        check(approx(last, 1.0f), "one driver, full throttle");
        const Array<Float32, 5> steps = { 0.8f, 0.9f, 0.3f, 0.95f, 0.1f };
        // A behaviour BORROWS its id, so the strings have to outlive the loop
        // that loads them rather than living on its stack.
        Array<Str, 5> ids;
        Bool everRose = false;
        for(Size i = 0; i < steps.size(); ++i)
        {
            ids[i] = Str("net.exoad.test.step") + Str(1, static_cast<Char>('0' + i));
            static_cast<Void>(add(c, ceilingAt(ids[i].c_str(), steps[i])));
            const Float32 now = c.run(over(s)).throttle;
            if(now > last + 0.0005f)
            {
                everRose = true;
            }
            last = now;
        }
        check(!everRose, "adding a behaviour NEVER raised the throttle, over five loads");
        check(approx(last, 0.1f), "and the answer is the lowest ceiling any of them set");
    }
    {
        // A clamp cannot even express a raise: clampTo above 1 is a no-op, not
        // a way back up to full authority.
        chain::Chain c;
        static_cast<Void>(add(c, ceilingAt("net.exoad.test.low", 0.2f)));
        static_cast<Void>(add(c, ceilingAt("net.exoad.test.high", 5.0f)));
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        const bibo::Scan s = ring(3.0f);
        check(
            approx(c.run(over(s)).throttle, 0.2f),
            "a clamp asking for MORE than 1 raises nothing"
        );
    }
    std::printf("\n-- a behaviour that may not drive --\n");
    {
        chain::Chain c;
        static_cast<Void>(add(c, makeUniq<Liar>()));
        const bibo::Scan s = ring(3.0f);
        const chain::Outcome o = c.run(over(s));
        check(
            o.throttle == 0.0f,
            "a proposal from a behaviour that may not drive gets no throttle"
        );
        check(o.refused == 1, "and the refusal is counted, not silently obeyed");
        check(o.lastRefusal.find("liar") != Str::npos, "and it says which behaviour");
        check(approx(o.steer, -0.5f), "its steering still stands: `drive no` is about throttle");
    }
    std::printf("\n-- clamps alone, and behaviours with nothing to say --\n");
    {
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeStop()));
        const bibo::Scan wall = ring(0.30f);
        const chain::Outcome o = c.run(over(wall));
        check(!o.drive, "a chain of only clamps proposes nothing, so the host calls nothing");
        check(o.intent.ceiling == 0.0f, "though the ceiling it set is there to see");
    }
    {
        chain::Chain c;
        static_cast<Void>(add(c, makeUniq<Silent>()));
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        const bibo::Scan s = ring(3.0f);
        check(
            approx(c.run(over(s)).throttle, 1.0f),
            "a behaviour with nothing to say changes nothing"
        );
    }
    std::printf("\n-- wasd, the viewer's driving --\n");
    {
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeWasd()));
        const bibo::Scan s = ring(3.0f);
        chain::Pass p = over(s);
        p.haveHolder = false;
        p.manualThrottle = 0.9f;
        p.manualSteer = 0.5f;
        const chain::Outcome idle = c.run(p);
        check(!idle.drive, "with nobody holding the wheel wasd says NOTHING, not stop");
        p.haveHolder = true;
        const chain::Outcome held = c.run(p);
        check(held.drive, "with a holder it proposes");
        check(
            approx(held.throttle, 0.9f) && approx(held.steer, 0.5f),
            "exactly what the host read"
        );
    }
    {
        // What the car is actually for: drive it by hand, and it refuses to run
        // into things. Two bundles, cooperating, neither knowing the other.
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeWasd()));
        static_cast<Void>(add(c, chain::makeStop()));
        const bibo::Scan clear = ring(3.0f);
        const bibo::Scan wall = ring(0.30f);
        chain::Pass p = over(clear);
        p.haveHolder = true;
        p.manualThrottle = 0.8f;
        p.manualSteer = -0.3f;
        check(approx(c.run(p).throttle, 0.8f), "wasd + stop, clear ahead: the driver has the car");
        chain::Pass q = over(wall);
        q.haveHolder = true;
        q.manualThrottle = 0.8f;
        q.manualSteer = -0.3f;
        const chain::Outcome o = c.run(q);
        check(o.throttle == 0.0f, "and with something in front the throttle is taken away");
        check(approx(o.steer, -0.3f), "while the wheels stay where the driver put them");
    }
    std::printf("\n-- forward, and a blind scan --\n");
    {
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeForward()));
        const bibo::Scan clear = ring(3.0f);
        const bibo::Scan wall = ring(0.30f);
        const bibo::Scan edge = ring(0.70f);
        check(approx(c.run(over(wall)).throttle, 0.0f), "forward stops for a wall on its own");
        check(
            approx(c.run(over(edge)).throttle, 0.0f),
            "between the stop and go marks it stays stopped"
        );
        check(
            approx(c.run(over(clear)).throttle, chain::CREEP_THROTTLE),
            "past the go mark it creeps"
        );
        check(
            approx(c.run(over(edge)).throttle, chain::CREEP_THROTTLE),
            "and coming back it keeps going"
        );
    }
    {
        chain::Chain c;
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        static_cast<Void>(add(c, chain::makeStop()));
        const bibo::Scan nothingSeen = blindScan();
        check(
            c.run(over(nothingSeen)).throttle == 0.0f,
            "a BLIND scan clamps: it is not an empty room"
        );
    }
    {
        chain::Chain c;
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        static_cast<Void>(add(c, chain::makeStop()));
        chain::Pass p;
        p.dtMs = 100;
        check(c.run(p).throttle == 0.0f, "and so does having no scan at all");
    }
    std::printf("\n-- follow: the first driver that looks through the camera --\n");
    {
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeFollow()));
        const bibo::Scan room = ring(3.0f);
        chain::Pass none = over(room);
        chain::Outcome o = c.run(none);
        check(
            o.drive && o.throttle == 0.0f && o.steer == 0.0f,
            "no detection: an ACTIVE stop, wheels straight"
        );
        bibowire::Tags seen;
        seen.flags = bibowire::TAGS_FLAG_CALIBRATED;
        bibowire::Tag far;
        far.id = 5;
        far.rangeMm = 2000;
        far.bearingCdeg = 1500;
        seen.tags.push_back(far);
        chain::Pass p = over(room);
        p.tags = &seen;
        p.tagsAgeMs = 100;
        o = c.run(p);
        check(approx(o.throttle, chain::FOLLOW_CRUISE), "a fresh tag 2 m away: creep");
        check(approx(o.steer, 0.5f), "15 degrees right of the axis is half lock RIGHT");
        seen.tags[0].bearingCdeg = -6000;
        o = c.run(p);
        check(approx(o.steer, -1.0f), "60 degrees left is full lock left, clamped");
        seen.tags[0].rangeMm = 500;
        o = c.run(p);
        check(
            o.throttle == 0.0f && approx(o.steer, -1.0f),
            "inside the stop mark it stops and keeps aiming"
        );
        seen.tags[0].rangeMm = 700;
        check(c.run(p).throttle == 0.0f, "between the marks it stays stopped");
        seen.tags[0].rangeMm = 900;
        check(approx(c.run(p).throttle, chain::FOLLOW_CRUISE), "past the go mark it creeps again");
        seen.tags[0].rangeMm = 700;
        check(approx(c.run(p).throttle, chain::FOLLOW_CRUISE), "and coming back it keeps going");
        p.tagsAgeMs = chain::TAGS_FRESH_MS + 1;
        o = c.run(p);
        check(o.throttle == 0.0f && o.steer == 0.0f, "a stale detection is no detection");
        p.tagsAgeMs = 100;
        check(c.run(p).throttle == 0.0f, "and after a loss it starts blocked again");
        seen.tags[0].rangeMm = 2000;
        check(approx(c.run(p).throttle, chain::FOLLOW_CRUISE), "until the tag is past the go mark");
        // Uncalibrated: no range, so aim and hold still.
        seen.flags = 0;
        seen.tags[0].rangeMm = 0;
        seen.tags[0].bearingCdeg = 900;
        o = c.run(p);
        check(
            o.throttle == 0.0f && approx(o.steer, 0.3f),
            "no range: it aims at the tag and does not move"
        );
        // Two tags: the nearest is followed.
        seen.flags = bibowire::TAGS_FLAG_CALIBRATED;
        seen.tags[0].rangeMm = 3000;
        seen.tags[0].bearingCdeg = -3000;
        bibowire::Tag near;
        near.id = 7;
        near.rangeMm = 1500;
        near.bearingCdeg = 1500;
        seen.tags.push_back(near);
        o = c.run(p);
        check(approx(o.steer, 0.5f), "of two tags the nearer is followed");
    }
    {
        // Under the lidar's stop clamp, the camera driver is held like any other.
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeFollow()));
        static_cast<Void>(add(c, chain::makeStop()));
        bibowire::Tags seen;
        bibowire::Tag far;
        far.rangeMm = 2000;
        seen.tags.push_back(far);
        const bibo::Scan wall = ring(0.30f);
        chain::Pass p = over(wall);
        p.tags = &seen;
        check(
            c.run(p).throttle == 0.0f,
            "a wall in front clamps follow to zero whatever the tag says"
        );
    }
    std::printf("\n-- NaN, and values out of range --\n");
    {
        chain::Chain c;
        static_cast<Void>(add(c, makeUniq<Nan>()));
        const bibo::Scan s = ring(3.0f);
        const chain::Outcome o = c.run(over(s));
        check(o.throttle == 0.0f, "a NaN throttle counts as 0, as bibo::Car::drive does");
        check(o.steer == 0.0f, "and so does a NaN steering");
    }
    {
        chain::Chain c;
        static_cast<Void>(add(c, ceilingAt("net.exoad.test.nanclamp", std::nanf(""))));
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        const bibo::Scan s = ring(3.0f);
        check(
            c.run(over(s)).throttle == 0.0f,
            "a NaN ceiling is the STRONGEST clamp, not the weakest"
        );
    }
    {
        chain::Chain c;
        static_cast<Void>(add(c, ceilingAt("net.exoad.test.negative", -3.0f)));
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        const bibo::Scan s = ring(3.0f);
        check(c.run(over(s)).throttle == 0.0f, "and so is a negative one");
    }
    std::printf("\n-- two drivers: the last one wins --\n");
    {
        // docs/bundles.md section 10 leaves this open; this is the choice made,
        // and the test is here so changing it is a deliberate act.
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeForward()));
        static_cast<Void>(add(c, chain::makeWasd()));
        const bibo::Scan s = ring(3.0f);
        chain::Pass p = over(s);
        p.haveHolder = true;
        p.manualThrottle = 0.7f;
        p.manualSteer = 0.4f;
        check(approx(c.run(p).throttle, 0.7f), "wasd loaded after forward takes the car");
        check(approx(c.run(p).steer, 0.4f), "and the steering with it");
    }
    std::printf("\n-- loading and unloading --\n");
    {
        chain::Chain c;
        Str why;
        check(c.load(chain::makeWasd(), why), "wasd loads");
        check(c.has(chain::ID_WASD), "and the chain has it");
        check(!c.load(chain::makeWasd(), why), "loading the same id twice is refused");
        check(why.find(chain::ID_WASD) != Str::npos, "and the refusal names the id");
        check(c.size() == 1, "and nothing was added");
        check(c.load(chain::makeStop(), why), "a second, different bundle loads");
        check(
            c.at(0) != nullptr && Str(c.at(0)->id()) == chain::ID_WASD,
            "chain order is load order"
        );
        check(
            c.at(1) != nullptr && Str(c.at(1)->id()) == chain::ID_STOP,
            "with the second after it"
        );
        check(c.at(2) == nullptr, "and past the end is null rather than a crash");
        check(!c.unload("net.exoad.test.never"), "unloading what was never loaded is false");
        check(c.unload(chain::ID_WASD), "unloading a loaded bundle succeeds");
        check(!c.has(chain::ID_WASD), "and it is gone");
        check(c.size() == 1, "leaving the other one");
        check(c.at(0) != nullptr && Str(c.at(0)->id()) == chain::ID_STOP, "which has moved up");
        check(!c.load(nullptr, why), "loading nothing at all is refused rather than stored");
    }
    {
        // Unloading everything is legal, and means the car coasts: an empty
        // chain proposes nothing, which is not the same as proposing zero.
        chain::Chain c;
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        check(c.unload("net.exoad.test.greedy"), "the last bundle unloads");
        const bibo::Scan s = ring(3.0f);
        check(!c.run(over(s)).drive, "and the chain goes back to proposing nothing");
    }
    std::printf("\n-- the catalog --\n");
    {
        const Vec<chain::Entry>& all = chain::catalog();
        check(!all.empty(), "the catalog lists something");
        Bool sorted = true;
        Bool uniqueIds = true;
        for(Size i = 1; i < all.size(); ++i)
        {
            const Str prev = all[i - 1].id;
            const Str here = all[i].id;
            if(!(prev < here))
            {
                sorted = false;
            }
            if(prev == here)
            {
                uniqueIds = false;
            }
        }
        check(sorted, "sorted by id, so an index means the same bundle after a restart");
        check(uniqueIds, "and no id appears twice");
        // EVERY ROW IS CHECKED AGAINST THE BEHAVIOUR IT NAMES, so the table a
        // viewer is shown cannot drift from what actually loads.
        Bool madeAll = true;
        Bool idsAgree = true;
        Bool namesAgree = true;
        Bool drivesAgree = true;
        for(const chain::Entry& e : all)
        {
            UniqPtr<chain::Behaviour> b = chain::make(e.id);
            if(!b)
            {
                madeAll = false;
                continue;
            }
            if(Str(b->id()) != Str(e.id))
            {
                idsAgree = false;
            }
            if(Str(b->name()) != Str(e.name))
            {
                namesAgree = false;
            }
            if(b->mayDrive() != e.drives)
            {
                drivesAgree = false;
            }
        }
        check(madeAll, "every catalog id makes a behaviour");
        check(idsAgree, "which reports the id it was asked for");
        check(namesAgree, "and the name the catalog advertises");
        check(drivesAgree, "and whose mayDrive agrees with the catalog's `drives`");
        check(chain::make("net.exoad.test.nonesuch") == nullptr, "an unknown id makes nothing");
        check(chain::make(nullptr) == nullptr, "and so does no id at all");
    }
    {
        // The whole catalog loads into one chain: the ids are unique, so nothing
        // refuses. This is what "several bundles cooperate" rests on.
        chain::Chain c;
        Bool allLoaded = true;
        for(const chain::Entry& e : chain::catalog())
        {
            Str why;
            if(!c.load(chain::make(e.id), why))
            {
                allLoaded = false;
            }
        }
        check(allLoaded, "every bundle in the catalog loads at once");
        check(c.size() == chain::catalog().size(), "and the chain holds them all");
    }
    {
        // A viewer greys a row on these bits, so a wrong one is a bundle refused
        // on a car that could run it perfectly well.
        UInt8 stopNeeds = 0xFFu;
        UInt8 wasdNeeds = 0xFFu;
        for(const chain::Entry& e : chain::catalog())
        {
            if(Str(e.id) == Str(chain::ID_STOP))
            {
                stopNeeds = e.needs;
            }
            if(Str(e.id) == Str(chain::ID_WASD))
            {
                wasdNeeds = e.needs;
            }
        }
        const Bool stopOk = (stopNeeds & bibowire::BUNDLE_NEEDS_LIDAR) != 0
                         && (stopNeeds & bibowire::BUNDLE_NEEDS_PICO) == 0;
        const Bool wasdOk = (wasdNeeds & bibowire::BUNDLE_NEEDS_PICO) != 0
                         && (wasdNeeds & bibowire::BUNDLE_NEEDS_LIDAR) == 0;
        check(stopOk, "the stop clamp needs the lidar and NOT the Pico: it only takes away");
        check(wasdOk, "and wasd needs the Pico and not the lidar: it never reads a scan");
    }
    std::printf("\n-- weave, the second driver --\n");
    {
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeWeave()));
        const chain::Outcome roomRight = c.run(over(sided(0.5f, 3.0f)));
        check(roomRight.steer > 0.0f, "with room on the RIGHT, weave steers right");
        const chain::Outcome roomLeft = c.run(over(sided(3.0f, 0.5f)));
        check(roomLeft.steer < 0.0f, "and with room on the left it steers left");
        const chain::Outcome even = c.run(over(ring(3.0f)));
        check(approx(even.steer, 0.0f), "equal room either side steers straight");
        check(approx(even.throttle, chain::WEAVE_CRUISE), "and a clear room cruises");
        check(c.run(over(ring(0.30f))).throttle == 0.0f, "a wall in front takes the throttle");
    }
    {
        // Two drivers that disagree about the wheel, which is the case
        // docs/bundles.md section 10 leaves to a rule rather than to luck.
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeForward()));
        static_cast<Void>(add(c, chain::makeWeave()));
        const chain::Outcome o = c.run(over(sided(0.5f, 3.0f)));
        check(o.steer > 0.0f, "weave loaded last takes the steering from forward");
        check(approx(o.throttle, chain::WEAVE_CRUISE), "and the throttle with it");
    }

    std::printf("\n-- trim, ready, and the loaded set as a file --\n");
    {
        UniqPtr<chain::Behaviour> t = chain::make(chain::ID_TRIM);
        check(t != nullptr, "trim is in the catalog");
        check(t != nullptr && !t->mayDrive(), "and may not drive");
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeTrim()));
        static_cast<Void>(add(c, makeUniq<Greedy>()));
        check(approx(c.run(over(ring(3.0f))).throttle, 1.0f), "and it changes nothing in a pass");
    }
    {
        chain::Present have;
        have.lidar = true;
        check(chain::readyFor(bibowire::BUNDLE_NEEDS_LIDAR, have), "a met need is ready");
        check(!chain::readyFor(bibowire::BUNDLE_NEEDS_PICO, have), "an unmet one is not");
        check(chain::readyFor(bibowire::BUNDLE_MAY_DRIVE, have), "MAY_DRIVE is not a need");
        check(chain::readyFor(0u, have), "and needing nothing is ready on anything");
    }
    {
        chain::Chain c;
        static_cast<Void>(add(c, chain::makeStop()));
        static_cast<Void>(add(c, chain::makeWasd()));
        Vec<Str> ids;
        Str unknown;
        check(chain::parseSet(chain::formatSet(c), ids, unknown), "the set round-trips");
        const Bool ordered = ids.size() == 2 && ids[0] == chain::ID_STOP && ids[1] == chain::ID_WASD;
        check(ordered, "in chain order");
        const Str empty = chain::formatSet(chain::Chain());
        check(empty.find("net.exoad") == Str::npos, "an empty chain writes no ids");
    }
    {
        Vec<Str> ids;
        Str unknown;
        Str text = "# a comment\n  ";
        text += Str(chain::ID_WASD) + "   # trailing\nnet.exoad.gone.away\n\n";
        text += Str(chain::ID_WASD) + "\n" + chain::ID_STOP + "\r\n";
        check(!chain::parseSet(text, ids, unknown), "an unknown id makes the parse say so");
        check(unknown == "net.exoad.gone.away", "and names it");
        const Bool kept = ids.size() == 2 && ids[0] == chain::ID_WASD && ids[1] == chain::ID_STOP;
        check(kept, "while the known ones load, once each, comments and a CR dropped");
        check(chain::parseSet("", ids, unknown) && ids.empty(), "an empty file is an empty set");
    }

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
