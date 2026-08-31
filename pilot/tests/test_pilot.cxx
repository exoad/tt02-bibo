// The companion board's link and autonomy stubs.
//
//   tests\build_pilot_test.bat run
//
// TWO THINGS ARE BEING CHECKED, and neither is "does the code work" - it does
// not do anything yet, deliberately.
//
//   1. The stubs REFUSE rather than fabricate. A step() that returned a
//      plausible steering angle would be indistinguishable from a working
//      controller until the car was moving, and open() returning OK would mean
//      a STOP command silently going nowhere.
//
//   2. firmware/lib's pure headers really do compile away from the Pico. Four
//      of them say so in their own comments; this file is the first thing that
//      holds them to it, by including them from a program that is not firmware
//      and has no SDK.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"

#include "autonomy.hxx"
#include "link.hxx"

#include <cstdio>
#include <cstring>

static Int32 failures = 0;
static Int32 checks   = 0;

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

Int32 main()
{
    std::printf("\npilot - the companion board's link and loop\n\n");

    // ---- the link refuses, and says which absence it is --------------------
    {
        link::Config cfg;
        cfg.where = "/dev/ttyACM0";

        const link::Result r = link::open(cfg);
        check(r == link::Result::RESULT_NO_PLATFORM,
              "open() refuses, and not by pretending the port is missing");
        std::printf("        \"%s\"\n", link::why(r));

        check(!link::isOpen(), "and the link is not open afterwards");
        check(link::send("PING") == link::Result::RESULT_NOT_OPEN,
              "send() on no link is NOT_OPEN, not a silent success");

        // The one that matters most. A drain() that reported OK with nothing in
        // it is indistinguishable from a quiet car.
        Vec<Str> lines;
        lines.push_back("a line from somewhere else");
        check(link::drain(lines) == link::Result::RESULT_NOT_OPEN,
              "drain() on no link is NOT_OPEN");
        check(lines.size() == 1,
              "and it did not clear a caller's vector on the way out");

        check(link::silentForMs() == -1, "silence is unknown, not zero");
    }

    // Every Result has a sentence, including whatever gets added later.
    {
        const link::Result all[] = {
            link::Result::RESULT_OK,           link::Result::RESULT_NO_PLATFORM,
            link::Result::RESULT_NO_PORT,      link::Result::RESULT_DENIED,
            link::Result::RESULT_NOT_OPEN,     link::Result::RESULT_WRITE_FAILED,
            link::Result::RESULT_CLOSED,
        };
        Bool named = true;
        for(const link::Result r : all)
        {
            const CharSeq s = link::why(r);
            if(s == nullptr || std::strcmp(s, "?") == 0 || *s == '\0')
            {
                named = false;
            }
        }
        check(named, "every link Result says something other than \"?\"");
    }

    // ---- the loop refuses too, and touches nothing -------------------------
    {
        autonomy::Outputs out;
        out.steer = 0.375f;          // a value only the caller could have set
        out.escUs = 1234;

        autonomy::Inputs in;
        const autonomy::Status s = autonomy::step(in, nullptr, &out);

        check(s == autonomy::Status::STATUS_NOT_IMPLEMENTED,
              "step() reports NOT_IMPLEMENTED");
        check(out.steer > 0.374f && out.steer < 0.376f && out.escUs == 1234,
              "and left the outputs exactly as the caller set them");
        std::printf("        \"%s\"\n", autonomy::why(s));
    }

    // ---- the tunings are real, and refuse what cannot be run ---------------
    {
        autonomy::Config c;
        check(autonomy::configure(c), "the defaults are accepted");
        check(autonomy::tuning().tickHz > 49.0f
              && autonomy::tuning().tickHz < 51.0f,
              "and read back");

        c.tickHz = 0.0f;
        check(!autonomy::configure(c), "a tick rate of zero is refused");

        c.tickHz    = 50.0f;
        c.silenceMs = 0;
        check(!autonomy::configure(c), "a silence window of zero is refused");

        check(autonomy::tuning().silenceMs == 500,
              "and a refused config did not partially apply");
    }

    // ---- the firmware's maths, running off the Pico -------------------------
    //
    // Not a test of the arithmetic - firmware/tests already covers that. A test
    // that these headers COMPILE AND LINK into a non-firmware program, which is
    // the claim four of them make in their comments and which nothing checked
    // until now.
    {
        // Pose is flat - x, y, heading - not a Vec2 and an angle.
        const bibo::geom::Pose at{ 0.0f, 0.0f, 0.0f };
        const bibo::geom::Vec2 pts[] = { { 1.0f, 0.0f }, { 2.0f, 0.0f } };

        bibo::pursuit::Path path;
        path.pts = pts;
        path.n   = 2;

        bibo::pursuit::Follower  f;
        const bibo::pursuit::Aim aim = bibo::pursuit::follow(&f, &path, at, 1.0f);
        check(aim.valid, "pursuit runs in a program that is not firmware");

        // bibo::kin, not bibo::kinematics - the header is named for the subject
        // and the namespace for the reader.
        const Float32 steer = bibo::kin::steerFor(0.5f);
        check(steer > 0.0f, "so does kinematics");

        bibo::control::Pid pid;
        pid.kp = 1.0f;
        const Float32 u = bibo::control::step(&pid, 1.0f, 0.0f, 0.02f);
        check(u > 0.9f && u < 1.1f, "and control");
    }

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
