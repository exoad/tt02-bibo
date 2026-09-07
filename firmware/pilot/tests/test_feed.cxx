// The scan feed's network half, held to feed.hxx over real sockets.
//
//   ctest --test-dir build-pilot -R feed     g++ on Linux, loopback TCP
//
// NOT IN firmware\verify.bat, AND NOT IN A .bat AT ALL: feed.cxx's real half
// is Linux-only - accept4, pipe2, poll, MSG_NOSIGNAL - and on MSVC every
// function refuses. A laptop run could prove nothing but that start() returns
// false, which tests/test_pilot.cxx already proves for the lidar and the link
// in the same shape. This is a CMake test, run on the board.
//
// WHAT IS CHECKED, in order:
//
//   1. A client is greeted with exactly the Policy's greeting, and lines
//      handed to publish() arrive in the order they were handed over.
//   2. MOTOR under MOTOR_REFUSE is answered with Policy::motorOn - not with
//      what was asked - and the callback never runs.
//   3. A client that stops reading is dropped within about a second, while
//      another client keeps receiving. This is the property the pilot's tick
//      depends on: a stalled phone must cost the car nothing.
//   4. stop() closes every client and frees the port for a second start().
//   5. A port that is taken refuses start() - and is answered with
//      Policy::fallbackPort when one is given, port() saying which.
//   6. relay(): a client hears a fake upstream's greeting and lines and
//      nothing of the owner's, its own lines reach the upstream, a second
//      client is relayed on a second ask, and an upstream closing closes its
//      client and no other.
//   7. An upstream that refuses the connect: the client is told the owner's
//      reason in an ERR line and closed, and Policy::onRelay hears why.
//
// Every socket read here has a deadline, so a feed that sends nothing fails
// the check rather than hanging the test.
//
// Exits 0 on PASS, 1 on FAIL. On any platform but Linux it checks only that
// start() refuses, and says so.

#include "shared.hxx"

#include "feed.hxx"

#include <cstdio>

#if defined(__linux__)
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
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

static Void checkStr(const Str& got, const Char* want, const Char* what)
{
    const Bool ok = (got == want);
    check(ok, what);
    if(!ok)
    {
        std::printf("        got  \"%s\"\n        want \"%s\"\n", got.c_str(), want);
    }
}

#if defined(__linux__)

// A blocking client on loopback, with every read under a deadline.
struct Peer
{
    Int32 fd = -1;
    Str   buf;

    [[nodiscard]] Bool connect(UInt16 port)
    {
        fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if(fd < 0)
        {
            return false;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);
        if(::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            ::close(fd);
            fd = -1;
            return false;
        }
        return true;
    }

    // One line without its '\n', or empty after `ms` with nothing whole.
    // Empty is also what the feed closing the socket reads as; closed()
    // tells the two apart.
    [[nodiscard]] Str line(Int32 ms)
    {
        const TimePoint start = monoNow();
        for(;;)
        {
            const Size nl = buf.find('\n');
            if(nl != Str::npos)
            {
                const Str one = buf.substr(0, nl);
                buf.erase(0, nl + 1);
                return one;
            }
            const Float64 left = static_cast<Float64>(ms) - elapsedMs(start);
            if(left <= 0.0 || fd < 0)
            {
                return Str();
            }
            pollfd p{};
            p.fd = fd;
            p.events = POLLIN;
            if(::poll(&p, 1, static_cast<Int32>(left)) <= 0)
            {
                return Str();
            }
            Array<Char, 4096> chunk{};
            const ISize n = ::recv(fd, chunk.data(), chunk.size(), 0);
            if(n <= 0)
            {
                ::close(fd);
                fd = -1;
                return Str();
            }
            buf.append(chunk.data(), static_cast<Size>(n));
        }
    }

    // Whether the far end has closed, within `ms`. Reads and discards
    // whatever arrives first.
    [[nodiscard]] Bool closed(Int32 ms)
    {
        const TimePoint start = monoNow();
        while(fd >= 0 && elapsedMs(start) < static_cast<Float64>(ms))
        {
            pollfd p{};
            p.fd = fd;
            p.events = POLLIN;
            if(::poll(&p, 1, 50) <= 0)
            {
                continue;
            }
            Array<Char, 4096> chunk{};
            const ISize n = ::recv(fd, chunk.data(), chunk.size(), 0);
            if(n <= 0)
            {
                ::close(fd);
                fd = -1;
            }
        }
        return fd < 0;
    }

    Void send(const Str& text)
    {
        if(fd >= 0)
        {
            static_cast<Void>(::send(fd, text.data(), text.size(), MSG_NOSIGNAL));
        }
    }

    Void close()
    {
        if(fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
    }
};

// A listening socket on an ephemeral loopback port: the thing that holds a
// port in check 5, and the fake upstream of checks 6 and 7.
struct Listener
{
    Int32  fd = -1;
    UInt16 port = 0;

    [[nodiscard]] Bool open()
    {
        fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if(fd < 0)
        {
            return false;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = 0;
        socklen_t len = sizeof(addr);
        if(::bind(fd, reinterpret_cast<const sockaddr*>(&addr), len) < 0 || ::listen(fd, 4) < 0
           || ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) < 0)
        {
            close();
            return false;
        }
        port = ntohs(addr.sin_port);
        return true;
    }

    // One connection, as a Peer, within `ms`.
    [[nodiscard]] Bool accept(Peer& into, Int32 ms)
    {
        pollfd p{};
        p.fd = fd;
        p.events = POLLIN;
        if(::poll(&p, 1, ms) <= 0)
        {
            return false;
        }
        into.fd = ::accept4(fd, nullptr, nullptr, SOCK_CLOEXEC);
        return into.fd >= 0;
    }

    Void close()
    {
        if(fd >= 0)
        {
            ::close(fd);
            fd = -1;
        }
    }
};

// Whether nothing is bound on `port` right now - so that check 5 does not
// fail on the day the port next to the blocker happens to be somebody's.
static Bool portFree(UInt16 port)
{
    const Int32 fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if(fd < 0)
    {
        return false;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    const Bool ok = ::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0;
    ::close(fd);
    return ok;
}

// Polls feed::clients() until it reads `want` or `ms` pass.
static Bool clientsReach(Size want, Int32 ms)
{
    const TimePoint start = monoNow();
    while(feed::clients() != want && elapsedMs(start) < static_cast<Float64>(ms))
    {
        sleepMs(5);
    }
    return feed::clients() == want;
}

static Bool motorCalled = false;

static Void onMotor(Bool on)
{
    static_cast<Void>(on);
    motorCalled = true;
}

// What onRelay last said, under a lock because it is said on the feed thread.
static Mutex relayM;
static Int32 relayUps = 0;
static Int32 relayFails = 0;
static Str   relayDetail;

static Void onRelay(Bool up, const Str& detail)
{
    LockGuard<Mutex> lock(relayM);
    if(up)
    {
        ++relayUps;
    }
    else
    {
        ++relayFails;
    }
    relayDetail = detail;
}

static Bool relaysReach(Int32 ups, Int32 fails, Int32 ms)
{
    const TimePoint start = monoNow();
    for(;;)
    {
        {
            LockGuard<Mutex> lock(relayM);
            if(relayUps == ups && relayFails == fails)
            {
                return true;
            }
        }
        if(elapsedMs(start) >= static_cast<Float64>(ms))
        {
            return false;
        }
        sleepMs(5);
    }
}

static Str relaySaid()
{
    LockGuard<Mutex> lock(relayM);
    return relayDetail;
}

Int32 main()
{
    std::printf("\nfeed - the scan feed's network half, over loopback\n\n");

    // ---- 1. greeting, then lines in order ----------------------------------------
    feed::Policy policy;
    policy.greeting = "INFO 65 1.2 18 -\nHEALTH 0\nMOTOR 1\n";
    policy.motor = feed::Motor::MOTOR_REFUSE;
    policy.motorOn = true;
    policy.refusal = "the test keeps it";
    policy.onMotor = onMotor;

    check(feed::start(0, policy), "the feed starts on an ephemeral port");
    check(feed::port() != 0, "and says which");
    check(!feed::start(0, policy), "a second start() without a stop() is refused");
    check(feed::clients() == 0, "no clients yet");

    Peer a;
    check(a.connect(feed::port()), "a client connects");
    checkStr(a.line(1000), "INFO 65 1.2 18 -", "and is greeted: INFO");
    checkStr(a.line(1000), "HEALTH 0", "HEALTH");
    checkStr(a.line(1000), "MOTOR 1", "MOTOR");
    check(clientsReach(1, 1000), "clients() counts it");

    for(Int32 i = 0; i < 20; ++i)
    {
        Array<Char, 32> line{};
        std::snprintf(line.data(), line.size(), "F 0 %d\n", i);
        feed::publish(Str(line.data()));
    }
    Bool inOrder = true;
    for(Int32 i = 0; i < 20 && inOrder; ++i)
    {
        Array<Char, 32> want{};
        std::snprintf(want.data(), want.size(), "F 0 %d", i);
        const Str got = a.line(1000);
        inOrder = got == want.data();
        if(!inOrder)
        {
            std::printf("        line %d: got \"%s\"\n", i, got.c_str());
        }
    }
    check(inOrder, "twenty published lines arrive, whole and in order");

    // ---- 2. MOTOR under MOTOR_REFUSE -----------------------------------------------
    a.send("MOTOR 0\n");
    checkStr(a.line(1000), "MOTOR 1", "MOTOR 0 is answered with the announced state, MOTOR 1");
    check(!motorCalled, "and the callback never ran");
    a.send("MOTOR 0\n");
    checkStr(a.line(1000), "MOTOR 1", "every time");
    feed::publish("F 0 99\n");
    checkStr(a.line(1000), "F 0 99", "and lines keep coming after it");
    a.send("NONSENSE line\n");
    feed::publish("F 0 100\n");
    checkStr(a.line(1000), "F 0 100", "a verb the feed has no name for is ignored, not fatal");

    // ---- 3. a client that stops reading is dropped, the other is not -----------------
    Peer b;
    check(b.connect(feed::port()), "a second client connects");
    checkStr(b.line(1000), "INFO 65 1.2 18 -", "and gets the greeting too");
    check(clientsReach(2, 1000), "clients() is 2");
    static_cast<Void>(b.line(1000));   // HEALTH
    static_cast<Void>(b.line(1000));   // MOTOR

    // A stalls: 8 KB lines, never read, until the kernel's buffers on both
    // ends of its socket are full (loopback autotunes them to megabytes, so
    // this goes in batches) and the feed's own pending bytes age past its
    // BEHIND_MS. B reads every line of every batch.
    const Str big = "F 0 0" + Str(8000, ' ') + "\n";
    const Str bigLine = big.substr(0, big.size() - 1);
    constexpr Int32 BATCH = 32;
    const TimePoint stallStart = monoNow();
    Bool bKept = true;
    Bool aDropped = false;
    while(elapsedMs(stallStart) < 8000.0 && !aDropped && bKept)
    {
        for(Int32 i = 0; i < BATCH; ++i)
        {
            feed::publish(big);
        }
        for(Int32 i = 0; i < BATCH && bKept; ++i)
        {
            bKept = b.line(1000) == bigLine;
        }
        aDropped = feed::clients() == 1;
        sleepMs(10);
    }
    check(aDropped, "the client that stopped reading is dropped within a few seconds");
    check(bKept, "while the reading client received every line");
    check(a.closed(1000), "and its socket is closed");
    std::printf("        (dropped after %.0f ms of not reading)\n", elapsedMs(stallStart));

    // ---- 4. stop() closes clients and frees the port -----------------------------------
    const UInt16 port = feed::port();
    feed::stop();
    check(b.closed(1000), "stop() closes the remaining client");
    check(feed::clients() == 0 && feed::port() == 0, "and reports nothing connected and no port");
    feed::publish("F 0 0\n");   // must not crash with nothing running
    check(feed::start(port, policy), "the same port can be taken again straight after");
    feed::stop();
    a.close();
    b.close();

    // ---- 5. a taken port refuses, or falls back when told where ----------------------
    // A blocker listens on an ephemeral port whose neighbour is free; a few
    // tries, because the neighbour is whoever's it happens to be.
    Listener blocker;
    Bool     haveBlocker = false;
    for(Int32 attempt = 0; attempt < 10 && !haveBlocker; ++attempt)
    {
        if(!blocker.open())
        {
            break;
        }
        haveBlocker = blocker.port < 65535 && portFree(static_cast<UInt16>(blocker.port + 1));
        if(!haveBlocker)
        {
            blocker.close();
        }
    }
    check(haveBlocker, "a blocker holds a port with a free neighbour");
    const UInt16 taken = blocker.port;
    const UInt16 next = static_cast<UInt16>(taken + 1);
    check(!feed::start(taken, policy), "start() on the taken port refuses with no fallback");
    check(feed::port() == 0, "and reports no port");
    feed::Policy fallback = policy;
    fallback.fallbackPort = next;
    check(feed::start(taken, fallback), "with a fallback it starts");
    check(feed::port() == next, "on the fallback port, and port() says so");
    Peer viaNext;
    check(viaNext.connect(next), "a client reaches it there");
    checkStr(viaNext.line(1000), "INFO 65 1.2 18 -", "and is greeted");
    viaNext.close();
    feed::stop();
    blocker.close();

    // ---- 6. relay: the upstream's lines to the client, the client's lines up --------
    // The feed is started the way an idle scanfeed is: no greeting, because
    // its own device is not open - the upstream's greeting is the greeting.
    Listener upstream;
    check(upstream.open(), "a fake upstream listens on an ephemeral port");
    feed::Policy relayPolicy;
    relayPolicy.motor = feed::Motor::MOTOR_OBEY;
    relayPolicy.onMotor = onMotor;
    relayPolicy.onRelay = onRelay;
    check(feed::start(0, relayPolicy), "a feed with no greeting starts");

    Peer c;
    check(c.connect(feed::port()), "a client connects");
    check(clientsReach(1, 1000), "and is counted");
    feed::relay("127.0.0.1", upstream.port, "the test refused");
    Peer up;
    check(upstream.accept(up, 1000), "relay() connects the feed to the upstream");
    check(relaysReach(1, 0, 1000), "and onRelay hears that it is up");
    {
        Array<Char, 32> want{};
        std::snprintf(
            want.data(),
            want.size(),
            "127.0.0.1:%u",
            static_cast<unsigned>(upstream.port)
        );
        checkStr(relaySaid(), want.data(), "naming the upstream");
    }
    up.send("INFO 99 1.0 1 -\nHEALTH 0\nMOTOR 1\n");
    checkStr(c.line(1000), "INFO 99 1.0 1 -", "the upstream's greeting reaches the client: INFO");
    checkStr(c.line(1000), "HEALTH 0", "HEALTH");
    checkStr(c.line(1000), "MOTOR 1", "MOTOR");
    up.send("F 0 7\nD cruise 1200 3 0 250 0\n");
    checkStr(c.line(1000), "F 0 7", "and so does an F line");
    checkStr(c.line(1000), "D cruise 1200 3 0 250 0", "and a D line the relay has no name for");
    motorCalled = false;
    c.send("MOTOR 0\n");
    checkStr(up.line(1000), "MOTOR 0", "the client's MOTOR goes upstream, whole");
    check(!motorCalled, "and not to the owner's onMotor");
    feed::publish("F 0 8\n");
    checkStr(c.line(300), "", "the owner's own lines do not reach a relayed client");

    Peer d;
    check(d.connect(feed::port()), "a second client connects");
    check(clientsReach(2, 1000), "and is counted");
    feed::relay("127.0.0.1", upstream.port, "the test refused");
    Peer up2;
    check(upstream.accept(up2, 1000), "a second relay() connects it, and only it");
    check(relaysReach(2, 0, 1000), "onRelay hears the second");
    up2.send("INFO 99 1.0 1 -\n");
    checkStr(d.line(1000), "INFO 99 1.0 1 -", "the second client hears its own upstream");
    up.send("F 0 9\n");
    checkStr(c.line(1000), "F 0 9", "the first still hears the first");
    checkStr(d.line(300), "", "and not the second");

    up.close();
    check(c.closed(1000), "the first upstream closing closes the first client");
    check(clientsReach(1, 1000), "and the count says one is left");
    up2.send("F 0 10\n");
    checkStr(d.line(1000), "F 0 10", "which is still served");
    d.send("QUIT\n");
    checkStr(up2.line(1000), "QUIT", "a relayed QUIT goes upstream rather than closing here");
    up2.close();
    check(d.closed(1000), "and the upstream closing on it closes it");
    check(clientsReach(0, 1000), "nobody left");

    // ---- 7. an upstream that refuses: ERR with the owner's reason ---------------------
    upstream.close();
    Peer e;
    check(e.connect(feed::port()), "a client connects with the upstream gone");
    check(clientsReach(1, 1000), "and is counted");
    feed::relay("127.0.0.1", upstream.port, "the test refused");
    checkStr(e.line(1000), "ERR the test refused", "it hears the owner's reason, not the socket's");
    check(e.closed(1000), "and closed");
    check(relaysReach(2, 1, 1000), "onRelay hears the failure");
    check(!relaySaid().empty(), "with a reason of its own");
    std::printf("        (\"%s\")\n", relaySaid().c_str());
    check(clientsReach(0, 1000), "and the count is back to nothing");
    feed::stop();
    c.close();
    d.close();
    e.close();
    up.close();
    up2.close();

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

#else

Int32 main()
{
    std::printf("\nfeed - the scan feed's network half\n\n");
    feed::Policy policy;
    check(!feed::start(0, policy), "start() refuses on a platform with no feed");
    check(feed::clients() == 0 && feed::port() == 0, "and there is nobody connected");
    feed::publish("F 0 0\n");
    feed::stop();
    check(true, "publish() and stop() are harmless with nothing running");
    checkStr(Str(), "", "(the socket tests run on Linux only - see the header)");
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}

#endif
