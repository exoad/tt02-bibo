// car: a Car that cannot open, held to car.hxx with no lidar and no Pico.
//
// What is held to an answer: bad flags, --help and an unmeasured --drive end the
// run before anything opens; a lidar that will not open ends it before the Pico
// is touched, so not one line is sent; arm(), scan() and drive() on such a run;
// finish()'s exit codes, and the same code twice; one Car at a time; and, on
// Linux, carlink::stopFromSignal against a pseudo-terminal, a signal stack with
// SIGSEGV's handler on it, and a child that overflows its stack still writing STOP.
//
// What is NOT proved here: the threads, arming and the signal hooks against a
// real lidar and Pico. That is a run on the car, on a stand.
//
// The ports named below do not exist, so a build with the lidar SDK on the board
// still never touches the real lidar or the Pico.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"

#include <cstdio>
#include <initializer_list>

#include "car.hxx"
#include "link.hxx"

#if defined(__linux__)
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

static Int32 checks = 0;
static Int32 failures = 0;

constexpr CharSeq NO_LIDAR = "/dev/bibo-no-such-lidar";
constexpr CharSeq NO_PICO = "/dev/bibo-no-such-pico";

static Void check(Bool ok, const Char* what)
{
    ++checks;
    if(!ok)
    {
        ++failures;
    }
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
}

static Void checkInt(Int64 got, Int64 want, const Char* what)
{
    const Bool ok = got == want;
    check(ok, what);
    if(!ok)
    {
        std::printf(
            "        got %lld, wanted %lld\n",
            static_cast<long long>(got),
            static_cast<long long>(want)
        );
    }
}

// A program's argv: its name, then the flags.
struct Argv
{
    Vec<Str> words;
    Vec<Char*> ptrs;

    Argv(std::initializer_list<CharSeq> flags)
    {
        words.emplace_back("test_car");
        words.insert(words.end(), flags.begin(), flags.end());
        for(Str& w : words)
        {
            ptrs.push_back(w.data());
        }
        ptrs.push_back(nullptr);
    }

    Argv(const Argv&) = delete;
    Argv& operator=(const Argv&) = delete;

    [[nodiscard]] Int32 count() const
    {
        return static_cast<Int32>(words.size());
    }
};

#if defined(__linux__)

static Void checkStr(const Str& got, const Char* want, const Char* what)
{
    const Bool ok = got == want;
    check(ok, what);
    if(!ok)
    {
        std::printf("        got %zu byte(s)\n", got.size());
    }
}

// Reads the pty master until `want` bytes have arrived or `ms` have passed.
static Str readMaster(Int32 master, Size want, Int32 ms)
{
    Str got;
    const TimePoint start = monoNow();
    while(got.size() < want && elapsedMs(start) < static_cast<Float64>(ms))
    {
        pollfd p{};
        p.fd = master;
        p.events = POLLIN;
        if(::poll(&p, 1, 20) <= 0)
        {
            continue;
        }
        Array<Char, 64> buf{};
        const ISize n = ::read(master, buf.data(), buf.size());
        if(n <= 0)
        {
            // The far end has closed: nothing more is coming.
            break;
        }
        got.append(buf.data(), static_cast<Size>(n));
    }
    return got;
}

// Recurses until the stack runs out. Each frame writes a page it keeps, and the
// sum after the call stops a compiler making the recursion a loop.
static Int32 overflow(Int32 depth)
{
    Array<volatile Char, 4096> frame{};
    frame[0] = static_cast<Char>(depth);
    if(depth >= (1 << 30))
    {
        return 0;
    }
    return overflow(depth + 1) + frame[0];
}

#endif

Int32 main()
{
    std::printf("\ncar - a Car that cannot open, with no lidar and no Pico\n\n");

    const UInt64 txBefore = carlink::txLines();
    const UInt64 droppedBefore = carlink::dropped();

    // ---- flags that end the run before anything opens ----
    {
        Argv a{ "--dry" };
        bibo::Car car(a.count(), a.ptrs.data());
        check(!car.ok(), "an unknown flag: ok() is false at once");
        check(!car.arm(), "and arm() refuses");
        checkInt(car.finish(), 2, "and finish() is 2");
        checkInt(car.finish(), 2, "and a second finish() gives the same code");
    }
    {
        Argv a{ "--seconds", "12x" };
        bibo::Car car(a.count(), a.ptrs.data());
        checkInt(car.finish(), 2, "a malformed value is bad flags too");
    }
    {
        Argv a{ "--help" };
        bibo::Car car(a.count(), a.ptrs.data());
        check(!car.ok(), "--help: ok() is false");
        checkInt(car.finish(), 0, "and finish() is 0");
    }
    {
        Argv a{ "--drive", "--lidar", NO_LIDAR, "--pico", NO_PICO, "--no-viewer" };
        bibo::Car car(a.count(), a.ptrs.data());
        const Int64 want = bibo::LIDAR_FORWARD_MEASURED ? 1 : 2;
        checkInt(
            car.finish(),
            want,
            "--drive with the forward angle unmeasured and no --forward is refused"
        );
    }

    // ---- a dry run whose lidar will not open ----
    {
        Argv a{ "--lidar", NO_LIDAR, "--no-viewer" };
        bibo::Car car(a.count(), a.ptrs.data());
        check(!car.ok(), "a dry run whose lidar will not open: ok() is false");

        TimePoint at = monoNow();
        check(!car.arm(), "arm() refuses");
        check(elapsedMs(at) < 1000.0, "at once, not after FIRST_REVOLUTION_MS");

        at = monoNow();
        const bibo::Scan s = car.scan();
        check(s.blind() && s.revolution == 0u, "scan() is blind");
        const Float64 limitMs = static_cast<Float64>(bibo::SCAN_WAIT_MS) + 100.0;
        check(elapsedMs(at) < limitMs, "within SCAN_WAIT_MS");

        car.drive(0.5f, 0.0f);
        checkInt(car.finish(), 1, "finish() is 1: nothing was measured");
    }

    // ---- --drive whose lidar will not open: the Pico is never touched ----
    {
        Argv a{
            "--drive", "--forward", "0", "--lidar", NO_LIDAR, "--pico", NO_PICO, "--no-viewer"
        };
        bibo::Car car(a.count(), a.ptrs.data());
        check(!car.ok(), "--drive whose lidar will not open: ok() is false");
        check(!car.arm(), "arm() refuses");
        car.drive(0.5f, 0.0f);
        check(!carlink::isOpen(), "the Pico's port was never opened");
        checkInt(car.finish(), 1, "finish() is 1");
    }
    check(
        carlink::txLines() == txBefore && carlink::dropped() == droppedBefore,
        "not one line was sent to the Pico, or dropped on the way"
    );

    // ---- one Car at a time ----
    {
        Argv a{ "--lidar", NO_LIDAR, "--no-viewer" };
        bibo::Car first(a.count(), a.ptrs.data());
        {
            Argv b{ "--bogus" };
            bibo::Car second(b.count(), b.ptrs.data());
            check(!second.ok(), "a second Car while the first exists: ok() is false");
            checkInt(second.finish(), 1, "and it refuses before reading its flags: 1, not 2");
        }
        checkInt(first.finish(), 1, "the first Car is unaffected");
    }
    {
        Argv a{ "--lidar", NO_LIDAR, "--no-viewer" };
        bibo::Car unfinished(a.count(), a.ptrs.data());
        check(!unfinished.ok(), "a Car left for its destructor to finish");
    }
    {
        Argv b{ "--bogus" };
        bibo::Car again(b.count(), b.ptrs.data());
        checkInt(again.finish(), 2, "once the last Car is gone, a new one reads its flags");
    }

#if defined(__linux__)
    // ---- the crash hooks run on a signal stack ----
    {
        Argv a{ "--lidar", NO_LIDAR, "--no-viewer" };
        bibo::Car car(a.count(), a.ptrs.data());
        stack_t ss{};
        const Bool stack = ::sigaltstack(nullptr, &ss) == 0 && (ss.ss_flags & SS_DISABLE) == 0;
        check(stack, "the thread that made the Car has a signal stack");
        struct sigaction sa{};
        const Bool asked = ::sigaction(SIGSEGV, nullptr, &sa) == 0;
        check(asked && (sa.sa_flags & SA_ONSTACK) != 0, "and SIGSEGV's handler runs on it");
        checkInt(car.finish(), 1, "finish() is 1");
    }

    // ---- stopFromSignal, against a pseudo-terminal standing in for the Pico ----
    {
        const Int32 master = ::posix_openpt(O_RDWR | O_NOCTTY);
        check(master >= 0, "a pseudo-terminal stands in for the Pico");
        if(master >= 0)
        {
            ::grantpt(master);
            ::unlockpt(master);
            const Char* name = ::ptsname(master);
            carlink::Config cfg;
            cfg.where = name != nullptr ? name : "";
            check(carlink::open(cfg) == carlink::Result::RESULT_OK, "the link opens on it");

            const UInt64 txOpen = carlink::txLines();
            carlink::stopFromSignal();
            const Str got = readMaster(master, 6, 1000);
            checkStr(got, "\nSTOP\n", "stopFromSignal() writes a newline and STOP");
            check(carlink::txLines() == txOpen, "and counts no line as sent");

            // The hooks the Cars above installed, inherited by a child that
            // overflows its stack: the crash handler still writes STOP, then the
            // child dies of the SIGSEGV as it would have.
            const Int32 child = ::fork();
            if(child == 0)
            {
                static_cast<Void>(overflow(0));
                std::_Exit(0);
            }
            const Str crashed = readMaster(master, 6, 2000);
            Int32 status = 0;
            const Bool reaped = child > 0 && ::waitpid(child, &status, 0) == child;
            checkStr(crashed, "\nSTOP\n", "a stack overflow still writes STOP");
            check(
                reaped && WIFSIGNALED(status) && WTERMSIG(status) == SIGSEGV,
                "and the program still dies of the SIGSEGV"
            );

            carlink::close();
            carlink::stopFromSignal();
            check(readMaster(master, 1, 200).empty(), "with the link closed it writes nothing");
            ::close(master);
        }
    }
#endif

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
