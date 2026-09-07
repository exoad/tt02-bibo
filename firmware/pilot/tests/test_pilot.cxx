// The companion board's link and autonomy stubs.
//
//   tests\build_pilot_test.bat run          MSVC, the refusing link
//   ctest --test-dir build-pilot            g++ on Linux, the termios link
//
// THREE THINGS ARE BEING CHECKED.
//
//   1. What refuses, refuses rather than fabricates. autonomy::step() returning
//      a plausible steering angle would be indistinguishable from a working
//      controller until the car was moving; carlink::open() returning OK on a
//      platform with no transport would mean a STOP command silently going
//      nowhere. On Linux, where there IS a transport, the same rule reads: a
//      missing device is NO_PORT, a device that is not a serial line is
//      OPEN_FAILED, and a send with no link is a counted drop. The lidar is
//      held to the same rule: built without its SDK, open() refuses naming
//      the SDK rather than a cable, and grab() empties the caller's vector
//      rather than leaving a stale revolution for reactive::step to drive on.
//
//   2. The Linux link moves lines. Not against a Pico - the board was not on
//      the Pi when this was written - but against a pseudo-terminal, which is a
//      real tty as far as termios and the reader thread are concerned. Lines
//      go out with their newline, come back split on '\n' with '\r' stripped, a
//      half line waits for its other half, and the far end closing is reported
//      as CLOSED rather than as a quiet car.
//
//   3. firmware/lib's pure headers really do compile away from the Pico. Four
//      of them say so in their own comments; this file is the first thing that
//      holds them to it, by including them from a program that is not firmware
//      and has no SDK.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"

#include "autonomy.hxx"
#include "lidar.hxx"
#include "link.hxx"

#include <cstdio>
#include <cstring>

#if defined(__linux__)
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#endif

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

#if defined(__linux__)

// Reads from the pty master until `want` bytes have arrived or `ms` have
// passed. The master is the "board" in these tests.
static Str readMaster(const Int32 master, const Size want, const Int32 ms)
{
    Str got;
    const TimePoint start = monoNow();
    while(got.size() < want && elapsedMs(start) < ms)
    {
        pollfd p{};
        p.fd = master;
        p.events = POLLIN;
        if(::poll(&p, 1, 20) <= 0)
        {
            continue;
        }
        Array<Char, 256> buf;
        const ISize n = ::read(master, buf.data(), buf.size());
        if(n > 0)
        {
            got.append(buf.data(), static_cast<Size>(n));
        }
    }
    return got;
}

// drain()s until `lines` holds `want` entries or `ms` have passed. Returns the
// last Result drain() gave, which is what a caller polling the link would see.
static carlink::Result drainUntil(Vec<Str>& lines, const Size want, const Int32 ms)
{
    carlink::Result r = carlink::Result::RESULT_OK;
    const TimePoint start = monoNow();
    while(lines.size() < want && elapsedMs(start) < ms)
    {
        r = carlink::drain(lines);
        if(lines.size() < want)
        {
            sleepMs(5);
        }
    }
    return r;
}

#endif

Int32 main()
{
    std::printf("\npilot - the companion board's link and loop\n\n");

    // Every send() in this file is tallied here, so the last check can hold
    // the link to `sends == tx + dropped` across everything that happened.
    UInt64 sends = 0;

    // ---- with no link, on every platform -----------------------------------
    {
        check(!carlink::isOpen(), "the link starts closed");

        const UInt64 droppedBefore = carlink::dropped();
        ++sends;
        check(
            carlink::send("PING") == carlink::Result::RESULT_NOT_OPEN,
            "send() on no link is NOT_OPEN, not a silent success"
        );
        check(
            carlink::dropped() == droppedBefore + 1,
            "and it was counted as dropped - sends == tx + dropped, as the hub's link keeps it"
        );
        check(carlink::txLines() == 0, "and not as transmitted");

        // The one that matters most. A drain() that reported OK with nothing in
        // it is indistinguishable from a quiet car.
        Vec<Str> lines;
        lines.push_back("a line from somewhere else");
        check(
            carlink::drain(lines) == carlink::Result::RESULT_NOT_OPEN,
            "drain() on no link is NOT_OPEN"
        );
        check(lines.size() == 1, "and it did not clear a caller's vector on the way out");

        check(carlink::silentForMs() == -1, "silence is unknown, not zero");

        carlink::close();
        carlink::close();
        check(!carlink::isOpen(), "close() with no link, twice, is nothing");
    }

#if !defined(__linux__)
    // ---- the link refuses, and says which absence it is --------------------
    {
        carlink::Config cfg;
        cfg.where = "/dev/ttyACM0";

        const carlink::Result r = carlink::open(cfg);
        check(
            r == carlink::Result::RESULT_NO_PLATFORM,
            "open() refuses, and not by pretending the port is missing"
        );
        std::printf("        \"%s\"\n", carlink::why(r));

        check(!carlink::isOpen(), "and the link is not open afterwards");
    }
#else
    // ---- the link opens what is there, and names what is not ---------------
    {
        carlink::Config cfg;
        cfg.where = "/dev/ttyBIBO-nowhere";

        const carlink::Result r = carlink::open(cfg);
        check(
            r == carlink::Result::RESULT_NO_PORT,
            "open() on a device that is not there is NO_PORT"
        );
        check(!carlink::isOpen(), "and the link is not open afterwards");
        const CharSeq reason = carlink::why(r);
        check(
            reason != nullptr && *reason != '\0' && std::strcmp(reason, "?") != 0,
            "with a reason a person can read"
        );
        check(!carlink::detail().empty(), "and the system's own words for it");
        std::printf("        \"%s\"\n", reason);
        std::printf("        %s\n", carlink::detail().c_str());

        // There, opens, and is not a serial line. NOT "no such port" - that
        // would send somebody to check a cable that is fine.
        cfg.where = "/dev/null";
        const carlink::Result nul = carlink::open(cfg);
        check(
            nul == carlink::Result::RESULT_OPEN_FAILED,
            "open() on /dev/null is OPEN_FAILED, not NO_PORT"
        );
        check(!carlink::isOpen(), "and the link is not open afterwards either");
        std::printf("        %s\n", carlink::detail().c_str());

        cfg.where = "/dev/ttyBIBO-nowhere";
        cfg.baud = 123456;
        check(
            carlink::open(cfg) == carlink::Result::RESULT_OPEN_FAILED,
            "a baud termios has no name for is refused before the device is touched"
        );
    }

    // ---- the link moves lines, against a pseudo-terminal -------------------
    //
    // A pty slave is a tty: termios configures it, the reader thread polls it,
    // and whatever the test writes into the master comes out of the slave as
    // if a board had sent it. Not a Pico. The first run against the real board
    // is still owed, and this is what makes that run a check rather than a
    // debugging session.
    {
        const Int32 master = ::posix_openpt(O_RDWR | O_NOCTTY);
        check(master >= 0, "a pseudo-terminal can be made to stand in for the board");
        if(master >= 0)
        {
            ::grantpt(master);
            ::unlockpt(master);
            const Char* name = ::ptsname(master);
            check(name != nullptr, "and it has a device path");

            carlink::Config cfg;
            cfg.where = name != nullptr ? name : "";
            std::printf("        %s\n", cfg.where.c_str());

            const carlink::Result r = carlink::open(cfg);
            check(r == carlink::Result::RESULT_OK, "open() on a real tty is OK");
            if(r != carlink::Result::RESULT_OK)
            {
                std::printf("        %s\n", carlink::detail().c_str());
            }
            check(carlink::isOpen(), "and the link is open");
            check(
                carlink::open(cfg) == carlink::Result::RESULT_OK,
                "a second open() while open is OK, not a second link"
            );

            const Int32 quiet = carlink::silentForMs();
            check(
                quiet >= 0 && quiet < 1000,
                "silence counts from open(), not from -1, before the board speaks"
            );

            // Out: the newline is the link's job.
            const UInt64 txBefore = carlink::txLines();
            ++sends;
            check(
                carlink::send("PING") == carlink::Result::RESULT_OK,
                "send() on an open link is OK"
            );
            check(carlink::txLines() == txBefore + 1, "and counted as transmitted");
            const Str wire = readMaster(master, 5, 1000);
            check(wire == "PING\n", "the board receives the line with exactly one newline");

            ++sends;
            check(
                carlink::send("STOP\n") == carlink::Result::RESULT_OK,
                "a caller's own newline is accepted"
            );
            check(readMaster(master, 5, 1000) == "STOP\n", "and not doubled");

            // In: two lines with mixed endings and the start of a third.
            const Str burst = "PONG\r\nSTATUS esc=1500\nhal";
            check(
                ::write(master, burst.data(), burst.size()) == static_cast<ISize>(burst.size()),
                "the board speaks"
            );
            Vec<Str> lines;
            lines.push_back("from somewhere else");
            const carlink::Result d = drainUntil(lines, 3, 1000);
            check(d == carlink::Result::RESULT_OK, "drain() on a live link is OK");
            check(lines.size() == 3, "two whole lines arrive, appended after the caller's own");
            check(lines.size() >= 2 && lines[1] == "PONG", "the first with its \\r stripped");
            check(lines.size() >= 3 && lines[2] == "STATUS esc=1500", "the second intact");
            sleepMs(50);
            check(
                carlink::drain(lines) == carlink::Result::RESULT_OK && lines.size() == 3,
                "and the half line is held, not delivered short"
            );

            const Str rest = "f\n";
            check(::write(master, rest.data(), rest.size()) == 2, "the other half turns up");
            check(
                drainUntil(lines, 4, 1000) == carlink::Result::RESULT_OK && lines.size() == 4 && lines[3] == "half",
                "and the line is delivered whole"
            );
            check(carlink::rxLines() == 3, "three lines received, counted");
            check(carlink::silentForMs() < 500, "and the silence clock was reset by them");

            // The board stops reading. On a pty that is a master nobody reads:
            // the kernel holds about 64 KiB on its behalf and then the slave's
            // writer gets EAGAIN, so a line several times that size gets its
            // head onto the wire and then stalls out the deadline. The point
            // is not the stall - it is what the NEXT line looks like to the
            // board, which is still collecting the head of this one.
            {
                const Str huge(256 * 1024, 'x');
                const UInt64 droppedBefore = carlink::dropped();
                const UInt64 txBeforeStall = carlink::txLines();
                ++sends;
                check(
                    carlink::send(huge) == carlink::Result::RESULT_WRITE_FAILED,
                    "a send() the device will not take within the deadline is WRITE_FAILED"
                );
                check(carlink::dropped() == droppedBefore + 1, "and counted as dropped");
                check(carlink::txLines() == txBeforeStall, "not as transmitted");
                std::printf("        %s\n", carlink::detail().c_str());

                // The board reads again. What it finds is the head of the line
                // the link gave up on, with no newline anywhere in it.
                const Str fragment = readMaster(master, huge.size() + 1, 500);
                check(
                    !fragment.empty() && fragment.size() < huge.size(),
                    "part of that line reached the board before the stall"
                );
                check(
                    fragment.find('\n') == Str::npos,
                    "and none of it is a newline - the board is still collecting it"
                );

                ++sends;
                check(
                    carlink::send("STOP") == carlink::Result::RESULT_OK,
                    "the next send() goes through"
                );
                check(
                    readMaster(master, 6, 1000) == "\nSTOP\n",
                    "and ends the fragment before its own line, so the board rejects the fragment alone"
                );

                ++sends;
                check(
                    carlink::send("PING") == carlink::Result::RESULT_OK,
                    "and the one after that"
                );
                check(
                    readMaster(master, 5, 1000) == "PING\n",
                    "arrives with no extra newline - the wire was clean again"
                );
            }

            // The far end goes away. On a USB CDC port this is the cable
            // coming out; on a pty it is the master closing. Either way the
            // link must say so rather than keep reporting a quiet car.
            ::close(master);
            const TimePoint gone = monoNow();
            while(carlink::isOpen() && elapsedMs(gone) < 1000)
            {
                sleepMs(5);
            }
            check(!carlink::isOpen(), "the board going away is noticed without anyone sending");
            std::printf("        %s\n", carlink::detail().c_str());

            const UInt64 droppedBefore = carlink::dropped();
            ++sends;
            check(
                carlink::send("STOP") == carlink::Result::RESULT_CLOSED,
                "send() after it is CLOSED, not OK"
            );
            check(carlink::dropped() == droppedBefore + 1, "and counted as dropped");
            check(
                carlink::drain(lines) == carlink::Result::RESULT_CLOSED,
                "drain() after it is CLOSED too"
            );

            carlink::close();
            check(!carlink::isOpen(), "close() releases it");
            check(carlink::silentForMs() == -1, "and silence is unknown again");
            carlink::close();
            check(!carlink::isOpen(), "close() twice is still nothing");
        }
    }
#endif

    check(
        sends == carlink::txLines() + carlink::dropped(),
        "every send() in this file was counted exactly once, as sent or as dropped"
    );

    // Every Result has a sentence, including whatever gets added later.
    {
        const carlink::Result all[] = {
            carlink::Result::RESULT_OK,           carlink::Result::RESULT_NO_PLATFORM,
            carlink::Result::RESULT_NO_PORT,      carlink::Result::RESULT_DENIED,
            carlink::Result::RESULT_BUSY,         carlink::Result::RESULT_OPEN_FAILED,
            carlink::Result::RESULT_NOT_OPEN,     carlink::Result::RESULT_WRITE_FAILED,
            carlink::Result::RESULT_CLOSED,
        };
        Bool named = true;
        for(const carlink::Result r : all)
        {
            const CharSeq s = carlink::why(r);
            if(s == nullptr || std::strcmp(s, "?") == 0 || *s == '\0')
            {
                named = false;
            }
        }
        check(named, "every link Result says something other than \"?\"");
    }

    // ---- the lidar refuses what it cannot do, and says which absence -------
    //
    // On MSVC, and on a CMake build without -DPILOT_RPLIDAR_SDK, lidar.cxx is
    // its refusing half, and this is what holds that half to lidar.hxx's
    // promises. With the SDK compiled in the same calls reach the real half,
    // which owes the same answers with nothing open - so the checks that hold
    // in both builds run in both, and the SDK build never names a port a lidar
    // could actually be on.
    {
        Vec<reactive::Ray> rays;
        rays.push_back(reactive::Ray{});
        check(!lidar::grab(rays), "grab() with nothing open is false");
        check(rays.empty(), "and EMPTIED the vector - a stale revolution is not handed on");
        check(!lidar::reason().empty(), "with a reason");

        check(
            !lidar::open("/dev/ttyBIBO-nowhere"),
            "open() on a port that is not there is refused"
        );
        check(!lidar::isOpen(), "and nothing is open afterwards");
        check(!lidar::reason().empty(), "with a reason a person can read");
        std::printf("        \"%s\"\n", lidar::reason().c_str());

        check(!lidar::motorOff(), "motorOff() with nothing open is false, and nothing worse");
        lidar::close();
        lidar::close();
        check(!lidar::isOpen(), "close() with nothing open, twice, is nothing");

#if !defined(PILOT_HAVE_RPLIDAR)
        check(!lidar::available(), "no SDK is built into this program, and it says so");
        check(!lidar::open("/dev/ttyUSB0"), "so open() on the port the C1 lives on refuses");
        check(
            lidar::reason().find("SDK") != Str::npos,
            "naming the missing SDK, not a cable - nobody is sent to check the car"
        );
        std::printf("        \"%s\"\n", lidar::reason().c_str());
#else
        check(lidar::available(), "an SDK is built into this program, and it says so");
#endif
    }

    // ---- the loop refuses too, and touches nothing -------------------------
    {
        autonomy::Outputs out;
        out.steer = 0.375f;          // a value only the caller could have set
        out.escUs = 1234;

        autonomy::Inputs in;
        const autonomy::Status s = autonomy::step(in, nullptr, &out);

        check(s == autonomy::Status::STATUS_NOT_IMPLEMENTED, "step() reports NOT_IMPLEMENTED");
        check(
            out.steer > 0.374f && out.steer < 0.376f && out.escUs == 1234,
            "and left the outputs exactly as the caller set them"
        );
        std::printf("        \"%s\"\n", autonomy::why(s));
    }

    // ---- the tunings are real, and refuse what cannot be run ---------------
    {
        autonomy::Config c;
        check(autonomy::configure(c), "the defaults are accepted");
        check(
            autonomy::tuning().tickHz > 49.0f && autonomy::tuning().tickHz < 51.0f,
            "and read back"
        );

        c.tickHz = 0.0f;
        check(!autonomy::configure(c), "a tick rate of zero is refused");

        c.tickHz = 50.0f;
        c.silenceMs = 0;
        check(!autonomy::configure(c), "a silence window of zero is refused");

        check(autonomy::tuning().silenceMs == 500, "and a refused config did not partially apply");
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
        path.n = 2;

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
