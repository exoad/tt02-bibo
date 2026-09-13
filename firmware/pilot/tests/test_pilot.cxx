// pilot: the companion board's link and lidar.
//
//   tools\test.bat pilot run                MSVC, the refusing link
//   ctest --test-dir build-pilot            g++ on Linux, the termios link
//
// What refuses must refuse rather than fabricate: a link that said OK with no
// transport would send STOP nowhere. On Linux the link is also driven against a
// pseudo-terminal, which is a real tty to termios and the reader thread.
#include "shared.hxx"

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

// Reads the pty master, which plays the board, until `want` bytes arrive or `ms` pass.
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

// drain()s until `lines` holds `want` entries or `ms` pass; returns drain()'s last Result.
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
    std::printf("\npilot - the companion board's link and lidar\n\n");
    // Every send() is tallied, for the last check: sends == tx + dropped.
    UInt64 sends = 0;
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
            "and it was counted as dropped - sends == tx + dropped"
        );
        check(carlink::txLines() == 0, "and not as transmitted");
        // A drain() that said OK with no link would look like a quiet car.
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
        // Not NO_PORT, which would send somebody to check a cable that is fine.
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
            // Two lines with mixed endings and the start of a third.
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
            // The board stops reading: the pty buffers about 64 KiB, so a line several
            // times that size stalls past the write deadline with its head on the wire.
            // What matters is how the next line looks to the board.
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
            // The master closing stands in for the USB cable coming out.
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
    // Without the SDK (MSVC, or CMake without -DPILOT_RPLIDAR_SDK) this holds
    // lidar.cxx's refusing half to lidar.hxx; with it, the real half owes the same
    // answers with nothing open. Only the build without the SDK names a real port.
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
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
