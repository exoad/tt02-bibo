// The viewer's bibowire client, held to what it promises - WITHOUT A BOARD.
//
//   viewer\tests\build_link_test.bat run
//
// WHY THIS FILE EXISTS AND WHAT IT CANNOT DO.
//
// The pilot that serves port 8020 is being written in parallel and has never
// run, so there is nothing to point this viewer at and nothing on the far end
// of a socket to prove anything against. What CAN be proved offline is the half
// that decides what gets drawn: bytes in, decoded state out, and the question
// "is this still true" answered the same way every time. So link.cxx is split
// with that seam in it - `Session` is pure, takes the caller's clock, and never
// names a socket - and this file drives it with hand-built frames.
//
// WHAT IS NOT COVERED HERE, said plainly rather than implied:
//   - connect, resolve, the 3000 ms deadline, TCP_NODELAY / SO_RCVBUF /
//     keepalive, the UDP bind and its peer filter, the select loop, and the
//     reconnect loop's use of the backoff numbers below. Those need a socket
//     and a peer, and one end of the pair does not exist yet.
//   - anything about CONTROL. This viewer does not send it; the Pico is not
//     connected to the board, so that path could not be exercised even with a
//     pilot running.
//
// The framing, the CRC, the resync and every message body belong to
// firmware/pilot/src/bibowire.cxx and its 312 checks; this file uses that codec
// to BUILD its inputs rather than restating what it already proves.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"

#include "bibowire.hxx"
#include "link.hxx"

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

static Void checkStr(const Str& got, const Char* want, const Char* what)
{
    const Bool ok = (got == want);
    check(ok, what);
    if(!ok)
    {
        std::printf("        got  \"%s\"\n        want \"%s\"\n", got.c_str(), want);
    }
}

[[nodiscard]] static Bool near(Float32 got, Float32 want)
{
    return std::fabs(got - want) < 0.002f;
}

// ---- building the bytes a board would send ---------------------------------

static Size framed(Vec<UInt8>& out, bibowire::Type t, const UInt8* body, Size len)
{
    bibowire::Head h;
    h.type = t;
    h.ver = 1;
    h.seq = static_cast<UInt16>(out.size() & 0xFFFFu);

    Vec<UInt8> frame(24000, 0);
    const bibowire::Body payload = { body, len };
    const Size n = bibowire::put(h, payload, frame.data(), frame.size());
    out.insert(out.end(), frame.begin(), frame.begin() + static_cast<ISize>(n));
    return n;
}

[[nodiscard]] static bibowire::Scan scanOf(UInt32 rev, UInt64 tUs)
{
    bibowire::Scan s;
    s.tMonoUs = tUs;
    s.revIndex = rev;
    s.freqMilliHz = 10000;
    s.health = 0;
    s.motor = 1;
    s.droppedSinceLast = 0;
    s.scanDivisor = 1;
    // One point per quadrant, so the frame conversion is checked at every
    // cardinal bearing rather than at one angle where a swapped sine and cosine
    // would still agree. The fifth is a NO RETURN and must not become a point at
    // the origin.
    s.points.push_back(bibowire::ScanPoint{ 0, 1000 });
    s.points.push_back(bibowire::ScanPoint{ 9000, 2000 });
    s.points.push_back(bibowire::ScanPoint{ 18000, 3000 });
    s.points.push_back(bibowire::ScanPoint{ 27000, 4000 });
    s.points.push_back(bibowire::ScanPoint{ 4500, 0 });
    s.quality.assign(s.points.size(), 47);
    return s;
}

static Size pushScan(Vec<UInt8>& out, const bibowire::Scan& s)
{
    Vec<UInt8> body(24000, 0);
    const Size n = bibowire::writeScan(s, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_SCAN, body.data(), n);
}

static Size pushWelcome(Vec<UInt8>& out, UInt32 boot, UInt16 major)
{
    bibowire::Welcome m;
    m.protoMajor = major;
    m.protoMinor = 0;
    m.sessionId = 0x51E55101u;
    m.bootId = boot;
    m.boardMonoUs = 500000;
    m.accepted = 2;
    m.armEpoch = 3;
    m.capabilities = 0x0Fu;
    m.boardName = "bibobox";
    m.text = "observer accepted";
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeWelcome(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_WELCOME, body.data(), n);
}

static Size pushDecide(Vec<UInt8>& out, UInt32 rev, UInt8 mode)
{
    bibowire::Decide m;
    m.revIndex = rev;
    m.clearanceMm = 3410;
    m.hits = 12;
    m.steerMilli = -457;
    m.throttleMilli = 350;
    m.mode = mode;
    m.stop = 0;
    m.source = 1;
    m.modeMs = 900;
    Array<UInt8, 64> body = {};
    const Size n = bibowire::writeDecide(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_DECIDE, body.data(), n);
}

static Size pushBoard(Vec<UInt8>& out, UInt64 tUs)
{
    bibowire::BoardState m;
    m.tMonoUs = tUs;
    m.upS = 812;
    m.cpuCentiC = 5420;
    m.battMilliV = bibowire::BATT_ABSENT;
    m.picoLink = 1;
    m.picoArmed = 2;
    m.pilotMode = 2;
    m.deadman = 0;
    m.lidarHealth = 0;
    m.lidarSpinning = 1;
    m.armEpoch = 3;
    m.picoSilentMs = bibowire::PICO_SILENT_ABSENT;
    m.revolutions = 4412;
    m.wifiName = "WhoopWhoop";
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeBoard(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_BOARD, body.data(), n);
}

static Size pushEvent(Vec<UInt8>& out, const Str& text, UInt16 dropped)
{
    bibowire::Event m;
    m.tMonoUs = 900000;
    m.severity = bibowire::Severity::SEVERITY_WARN;
    m.code = 7;
    m.droppedSince = dropped;
    m.text = text;
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeEvent(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_EVENT, body.data(), n);
}

static Size pushPing(Vec<UInt8>& out, UInt64 token)
{
    bibowire::Ping m;
    m.token = token;
    m.senderMonoUs = 1234567;
    Array<UInt8, 32> body = {};
    const Size n = bibowire::writePing(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_PING, body.data(), n);
}

// The board's answer to a PING THIS VIEWER sent: the token comes back verbatim
// and senderMonoUs is the board's own clock when it replied.
static Size pushPong(Vec<UInt8>& out, UInt64 token, UInt64 boardUs)
{
    bibowire::Ping m;
    m.token = token;
    m.senderMonoUs = boardUs;
    Array<UInt8, 32> body = {};
    const Size n = bibowire::writePing(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_PONG, body.data(), n);
}

static Size pushCtlState(Vec<UInt8>& out, UInt64 tUs, UInt8 armed)
{
    bibowire::CtlState m;
    m.tMonoUs = tUs;
    m.ackSeq = 8814;
    m.controlAgeMs = 41;
    m.steerNowMilli = -120;
    m.throttleMilli = 0;
    m.escUs = 1602;
    m.armed = armed;
    m.armEpoch = 3;
    m.deadman = 1;
    m.refuse = bibowire::Refuse::REFUSE_DEADMAN_SOFT;
    m.holder = 2;
    m.pilotMode = 2;
    m.scanAgeMs = 60;
    Array<UInt8, 64> body = {};
    const Size n = bibowire::writeCtlState(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_CTLSTATE, body.data(), n);
}

static Size pushBye(Vec<UInt8>& out, bibowire::Reason why, const Str& text)
{
    bibowire::Bye m;
    m.reason = why;
    m.text = text;
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeBye(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_BYE, body.data(), n);
}

// A type this build has no name for. 0x7E is in nobody's tag space and the
// length prefix is the whole extensibility story, so it must be stepped over by
// exactly its len and counted.
static Size pushUnknown(Vec<UInt8>& out)
{
    Array<UInt8, 8> body = { 1, 2, 3, 4, 5, 6, 7, 8 };
    bibowire::Head h;
    h.type = static_cast<bibowire::Type>(0x7E);
    h.ver = 1;
    h.seq = 9;
    Vec<UInt8> frame(64, 0);
    const bibowire::Body payload = { body.data(), body.size() };
    const Size n = bibowire::put(h, payload, frame.data(), frame.size());
    out.insert(out.end(), frame.begin(), frame.begin() + static_cast<ISize>(n));
    return n;
}

static Size feed(link::Session& s, const Vec<UInt8>& bytes, Int64 nowMs)
{
    return link::ingestBytes(s, bytes.data(), bytes.size(), nowMs);
}

// ---- the cases -------------------------------------------------------------

static Void testGeometry()
{
    std::printf("\n-- a SCAN becomes points in the scene's frame --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushScan(wire, scanOf(41, 1000000)));

    link::Session s;
    const Size used = feed(s, wire, 1000);
    check(used == wire.size(), "the whole scan frame is consumed");
    check(s.haveScan, "a scan arrived");
    check(s.revIndex == 41u, "revIndex is the frame's");
    check(s.freqMilliHz == 10000u, "freqMilliHz is the frame's");

    // Four, not five: distMm == 0 is NO RETURN and must not become a point at
    // the sensor's own position - a ring of those would read as an obstacle
    // wrapped around the car.
    check(s.cloud.size() == 4, "a no-return point is dropped, not placed at zero");

    if(s.cloud.size() == 4)
    {
        check(near(s.cloud[0].x, 0.0f) && near(s.cloud[0].y, 1.0f), "0 deg is +Y at 1 m");
        check(near(s.cloud[1].x, 2.0f) && near(s.cloud[1].y, 0.0f), "90 deg is +X at 2 m");
        check(near(s.cloud[2].x, 0.0f) && near(s.cloud[2].y, -3.0f), "180 deg is -Y at 3 m");
        check(near(s.cloud[3].x, -4.0f) && near(s.cloud[3].y, 0.0f), "270 deg is -X at 4 m");
        check(near(s.cloud[0].z, 0.16f), "z is the lidar's height, in metres");
    }
}

static Void testByteBoundaries()
{
    std::printf("\n-- the same stream, one byte at a time --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushWelcome(wire, 1, bibowire::PROTO_MAJOR));
    static_cast<Void>(pushScan(wire, scanOf(41, 1000000)));
    static_cast<Void>(pushDecide(wire, 41, 0));
    static_cast<Void>(pushBoard(wire, 1000000));

    link::Session whole;
    static_cast<Void>(feed(whole, wire, 1000));

    // A ring that is handed one byte per call must produce the identical
    // answer: NEED_MORE consumes nothing, so a partial frame is never a short
    // message.
    link::Session drip;
    Vec<UInt8> ring;
    for(Size i = 0; i < wire.size(); ++i)
    {
        ring.push_back(wire[i]);
        const Size used = link::ingestBytes(drip, ring.data(), ring.size(), 1000);
        ring.erase(ring.begin(), ring.begin() + static_cast<ISize>(used));
    }

    check(ring.empty(), "nothing is left over when the last byte lands");
    check(drip.frames == whole.frames, "the same number of frames either way");
    check(drip.cloud.size() == whole.cloud.size(), "the same cloud either way");
    check(drip.revIndex == whole.revIndex, "the same revolution either way");
    check(drip.haveWelcome && drip.haveDecide && drip.haveBoard, "every message arrives");
}

static Void testJunkAndUnknown()
{
    std::printf("\n-- junk, and a type this build has no name for --\n");

    Vec<UInt8> wire;
    // Seven bytes of rubbish, including something that spells the magic, so the
    // resync has a false lock to reject before it finds the real frame.
    const Array<UInt8, 7> junk = { 0x11, 0x42, 0x57, 0x00, 0x99, 0xAB, 0xCD };
    wire.insert(wire.end(), junk.begin(), junk.end());
    static_cast<Void>(pushScan(wire, scanOf(7, 500000)));

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));
    check(s.haveScan, "the frame behind the junk is found");
    check(s.resyncBytes == 7u, "the skipped bytes are counted exactly");

    Vec<UInt8> mixed;
    static_cast<Void>(pushUnknown(mixed));
    static_cast<Void>(pushScan(mixed, scanOf(8, 600000)));

    link::Session u;
    const Size used = feed(u, mixed, 1000);
    check(used == mixed.size(), "an unknown type is stepped over by exactly its len");
    check(u.unknownFrames == 1u, "and counted");
    check(u.haveScan && u.revIndex == 8u, "the frame after it parses");
    check(u.resyncBytes == 0u, "an unknown type is not junk");
}

static Void testCorruption()
{
    std::printf("\n-- a corrupted frame is not a shorter frame --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushScan(wire, scanOf(41, 1000000)));
    wire[20] = static_cast<UInt8>(wire[20] ^ 0xFFu);

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));
    check(!s.haveScan, "a CRC failure yields no scan at all");
    check(s.resyncBytes > 0u, "and the bytes are counted rather than dropped silently");
}

static Void testTruncation()
{
    std::printf("\n-- a truncated frame consumes nothing --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushScan(wire, scanOf(41, 1000000)));
    wire.resize(wire.size() - 3);

    link::Session s;
    const Size used = feed(s, wire, 1000);
    check(used == 0, "no bytes are retired");
    check(!s.haveScan, "and no half-read revolution is drawn");
    check(s.frames == 0u, "a partial frame is not a frame");
}

static Void testBootId()
{
    std::printf("\n-- a different bootId clears everything --\n");

    Vec<UInt8> first;
    static_cast<Void>(pushWelcome(first, 1000, bibowire::PROTO_MAJOR));
    static_cast<Void>(pushScan(first, scanOf(41, 1000000)));
    static_cast<Void>(pushDecide(first, 41, 0));

    link::Session s;
    static_cast<Void>(feed(s, first, 1000));
    check(s.haveScan && s.haveDecide, "the first session has a picture");
    check(s.bootId == 1000u, "and remembers the boot it belongs to");

    // A reconnect: the connection state goes, the bootId does NOT - it is the
    // only thing worth carrying across, and carrying it is what makes the
    // comparison below possible at all.
    link::clearSession(s);
    check(!s.haveScan && !s.haveDecide, "a reconnect resumes nothing");
    check(s.haveBootId && s.bootId == 1000u, "but the bootId survives the reconnect");

    Vec<UInt8> same;
    static_cast<Void>(pushWelcome(same, 1000, bibowire::PROTO_MAJOR));
    static_cast<Void>(pushScan(same, scanOf(42, 1100000)));
    static_cast<Void>(feed(s, same, 2000));
    check(s.notes.empty(), "the same bootId is not a restart and says nothing");
    check(s.haveScan, "and the picture rebuilds normally");

    // Now the pilot restarts. A healthy new socket to a restarted car, still
    // showing the previous run, is the most convincing stale picture there is.
    Vec<UInt8> other;
    static_cast<Void>(pushWelcome(other, 2001, bibowire::PROTO_MAJOR));
    static_cast<Void>(feed(s, other, 3000));
    check(!s.haveScan, "a different bootId clears the scan");
    check(!s.haveDecide, "and the decision");
    check(s.revIndex == 0u, "and the revIndex baseline");
    check(s.bootId == 2001u, "and adopts the new boot");
    check(s.haveWelcome, "while keeping the WELCOME that told it so");
    check(!s.notes.empty(), "and says so in words");
}

static Void testDecideTie()
{
    std::printf("\n-- a DECIDE belongs to one revolution --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushScan(wire, scanOf(41, 1000000)));
    static_cast<Void>(pushDecide(wire, 41, 0));

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));
    check(s.haveDecide, "a DECIDE naming the scan we have is kept");
    check(s.decide.mode == 0u, "with its mode");
    check(s.decide.clearanceMm == 3410u, "and its clearance");

    Vec<UInt8> orphan;
    static_cast<Void>(pushDecide(orphan, 99, 2));
    static_cast<Void>(feed(s, orphan, 1010));
    check(s.orphanDecides == 1u, "a DECIDE naming a scan we never saw is counted");
    check(s.decide.mode == 0u, "and does not replace the one that belongs here");

    // revIndex 0 is the BLIND tick: no revolution behind it BY DEFINITION, which
    // is not the same thing as naming one we missed.
    Vec<UInt8> blind;
    static_cast<Void>(pushDecide(blind, 0, 4));
    static_cast<Void>(feed(s, blind, 1020));
    check(s.decide.mode == 4u, "a BLIND tick with revIndex 0 is kept");
    check(s.orphanDecides == 1u, "and is not counted as an orphan");
}

static Void testStaleness()
{
    std::printf("\n-- what may be drawn, and when it may not --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushScan(wire, scanOf(41, 0)));
    static_cast<Void>(pushDecide(wire, 41, 0));
    static_cast<Void>(pushBoard(wire, 0));

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));

    check(s.revolution(1000).has_value(), "a fresh revolution is there to draw");
    check(!s.revolution(1000)->stale, "and is not marked stale");
    check(s.revolution(1399).has_value(), "at 399 ms it is still fresh");
    check(!s.revolution(1399)->stale, "and still not marked");
    check(s.revolution(1401).has_value(), "at 401 ms it is still drawable");
    check(s.revolution(1401)->stale, "but is marked stale");
    check(s.revolution(1401)->ageMs == 401, "with its age");

    // Beyond 1500 ms there is NO VALUE AT ALL. A greyed-out picture is still a
    // picture and people read pictures as current whatever colour they are.
    check(!s.revolution(2501).has_value(), "past 1500 ms there is nothing to draw");
    check(!s.decision(2501).has_value(), "and no decision either");
    check(!s.boardState(2501).has_value(), "and no board state");

    // A negative age is a bug upstream and the safe reading of a bug is "old".
    check(!s.revolution(900).has_value(), "a clock that went backwards reads as gone");
}

static Void testArrivalFloor()
{
    std::printf("\n-- an age is never smaller than the arrival floor --\n");

    // The first scan sets the offset: it left the board at 100 ms and arrived at
    // 1000, so the board's clock is 900 ms behind this one.
    Vec<UInt8> first;
    static_cast<Void>(pushScan(first, scanOf(1, 100000)));
    link::Session s;
    static_cast<Void>(feed(s, first, 1000));
    check(s.haveOffset && s.offsetMs == 900, "the first sample sets the offset");

    // The second one left at 200 ms and did not arrive until 1600 - it spent
    // 500 ms in a stall. Local arrival alone would call it 100 ms old at 1700;
    // the board's own clock says 600, and the LARGER wins.
    Vec<UInt8> late;
    static_cast<Void>(pushScan(late, scanOf(2, 200000)));
    static_cast<Void>(feed(s, late, 1600));
    check(s.offsetMs == 900, "a slower sample does not move the offset");

    const Opt<link::Revolution> rev = s.revolution(1700);
    check(rev.has_value(), "the revolution is still drawable");
    check(rev->ageMs == 600, "and its age is the board's, not the socket's");
    check(rev->stale, "which is what makes it read as stale rather than fresh");
}

static Void testPingAndProse()
{
    std::printf("\n-- PING, and the sentences --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushPing(wire, 0xABCDEF0123456789ull));
    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));
    check(s.pongsDue.size() == 1, "a PING is queued for an answer");
    check(s.pongsDue[0].token == 0xABCDEF0123456789ull, "with its token echoed verbatim");

    Vec<UInt8> events;
    static_cast<Void>(pushEvent(events, "lidar timeout - no revolution in 200 ms", 0));
    static_cast<Void>(feed(s, events, 1010));
    check(s.notes.size() == 1, "an EVENT becomes a note");
    checkStr(s.notes[0].text, "lidar timeout - no revolution in 200 ms", "carried verbatim");
    check(s.notes[0].severity == bibowire::Severity::SEVERITY_WARN, "with its severity");

    Vec<UInt8> dropped;
    static_cast<Void>(pushEvent(dropped, "port busy", 4));
    static_cast<Void>(feed(s, dropped, 1020));
    checkStr(s.notes[1].text, "port busy (+4 suppressed)", "and says how many it did not see");
}

static Void testBoardAndControl()
{
    std::printf("\n-- absence is representable --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushBoard(wire, 1000000));
    static_cast<Void>(pushCtlState(wire, 1000000, 1));

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));
    check(s.haveBoard, "BOARD arrives");
    check(s.board.battMilliV == bibowire::BATT_ABSENT, "an unmeasured battery stays absent");
    check(s.board.picoSilentMs == bibowire::PICO_SILENT_ABSENT, "so does a missing Pico link");
    check(s.board.picoArmed == 2u, "and an unknown arm state is unknown, not no");
    check(s.board.cpuCentiC == 5420, "a measured value is itself");

    check(s.haveControl, "CTLSTATE arrives, as it would on UDP");
    check(s.control.armed == 1u, "with what the board IS doing");
    check(s.control.refuse == bibowire::Refuse::REFUSE_DEADMAN_SOFT, "and why it refuses");
}

static Void testGapsAndBye()
{
    std::printf("\n-- gaps counted, and a BYE with a reason --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushScan(wire, scanOf(41, 1000000)));
    static_cast<Void>(pushScan(wire, scanOf(45, 1400000)));

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));
    check(s.missedRevs == 3u, "three missing revolutions are counted");
    checkStr(s.gapText, "revolutions 42-44 missing", "and named exactly");

    Vec<UInt8> bye;
    static_cast<Void>(pushBye(bye, bibowire::Reason::REASON_SHUTDOWN, "pilot stopping"));
    static_cast<Void>(feed(s, bye, 1100));
    check(s.haveBye, "a BYE is noticed");
    check(s.byeReason == bibowire::Reason::REASON_SHUTDOWN, "with its reason");
    checkStr(s.byeText, "pilot stopping", "and its sentence, which is the part a person acts on");
}

static Void testVersionRule()
{
    std::printf("\n-- protoMajor must be EQUAL --\n");

    Vec<UInt8> wire;
    static_cast<Void>(pushWelcome(wire, 5, 2));
    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));

    check(s.haveBye, "a major mismatch in WELCOME ends the session");
    check(s.byeReason == bibowire::Reason::REASON_VERSION, "for the version reason");
    // The sentence must EXIST. An explanation that says only "incompatible"
    // sends a person to read source in a field, which is the failure the whole
    // rule exists to prevent.
    check(!s.byeText.empty(), "with a non-empty sentence naming both versions");
    check(s.byeText.find("rebuild") != Str::npos, "and an action to take");
}

static Void testBackoff()
{
    std::printf("\n-- the reconnect schedule --\n");

    check(link::backoffBaseMs(1) == 250, "first retry at 250 ms");
    check(link::backoffBaseMs(2) == 500, "then 500");
    check(link::backoffBaseMs(3) == 1000, "then 1 s");
    check(link::backoffBaseMs(4) == 2000, "then 2 s");
    check(link::backoffBaseMs(5) == 4000, "then 4 s");
    check(link::backoffBaseMs(6) == 4000, "and 4 s forever after");
    check(link::backoffBaseMs(400) == 4000, "never giving up, and never longer");

    // +-20 %, and the jitter must actually move: a schedule that always returned
    // the base would pass a bounds check and still put two clients in lockstep.
    Bool inBand = true;
    Bool moved = false;
    UInt32 seed = 12345;
    for(Int32 i = 0; i < 4000; ++i)
    {
        seed = link::stir(seed);
        const Int32 got = link::jittered(4000, seed);
        if(got < 3200 || got > 4800)
        {
            inBand = false;
        }
        if(got != 4000)
        {
            moved = true;
        }
    }
    check(inBand, "4 s jitters inside +-20 %");
    check(moved, "and is not the base every time");

    Bool smallInBand = true;
    seed = 999;
    for(Int32 i = 0; i < 4000; ++i)
    {
        seed = link::stir(seed);
        const Int32 got = link::jittered(250, seed);
        if(got < 200 || got > 300)
        {
            smallInBand = false;
        }
    }
    check(smallInBand, "250 ms jitters inside +-20 %");
    check(link::stir(0) != 0u, "the generator never sticks at zero");
}

static Void testRoundTrip()
{
    std::printf("\n-- the round trip, measured rather than assumed --\n");

    link::Session s;
    check(!s.rttMs().has_value(), "before any PONG there is no latency to show");
    check(!s.bestRttMs().has_value(), "and no minimum either");

    // Sent at 1000, answered at 1100: 100 ms on the wire, measured from HERE.
    link::notePingSent(s, 7001u, 1000);
    Vec<UInt8> pong;
    static_cast<Void>(pushPong(pong, 7001u, 1000000));
    static_cast<Void>(feed(s, pong, 1100));

    const Opt<Int64> rtt = s.rttMs();
    const Opt<Int64> best = s.bestRttMs();
    const Opt<Int64> oneWay = s.oneWayMs();
    check(rtt.has_value() && *rtt == 100, "a matched PONG is a measured round trip");
    check(best.has_value() && *best == 100, "one sample is its own minimum");
    check(oneWay.has_value() && *oneWay == 50, "half the minimum is the one-way delay");

    // A PONG for a PING nobody sent must not be able to invent a round trip.
    Vec<UInt8> stray;
    static_cast<Void>(pushPong(stray, 999999u, 1000000));
    static_cast<Void>(feed(s, stray, 1200));
    const Opt<Int64> after = s.rttMs();
    check(after.has_value() && *after == 100, "an unmatched token is ignored");
    check(s.rtts.size() == 1, "and adds no sample");

    // Twenty round trips, nineteen of them stalled and one fast. THE MINIMUM is
    // the path; the mean would be the worst moment of the last sixteen seconds
    // wearing the path's name.
    link::Session many;
    for(Int32 i = 0; i < 20; ++i)
    {
        const UInt64 token = static_cast<UInt64>(i) + 1u;
        const Int64 sent = 1000 + (i * 1000);
        link::notePingSent(many, token, sent);
        const Int64 trip = (i == 18) ? 12 : 900;
        Vec<UInt8> reply;
        static_cast<Void>(pushPong(reply, token, static_cast<UInt64>(sent) * 1000u));
        static_cast<Void>(feed(many, reply, sent + trip));
    }
    const Opt<Int64> lowest = many.bestRttMs();
    const Opt<Int64> latest = many.rttMs();
    check(many.rtts.size() == 16, "only the last 16 round trips are kept");
    check(lowest.has_value() && *lowest == 12, "the minimum wins, not the mean");
    check(latest.has_value() && *latest == 900, "while the current one stays current");

    // And the offset a round trip implies can only make the picture OLDER.
    link::Session aged;
    link::notePingSent(aged, 42u, 1000);
    Vec<UInt8> wire;
    static_cast<Void>(pushPong(wire, 42u, 1000000));
    static_cast<Void>(feed(aged, wire, 1100));
    check(aged.haveOffset && aged.offsetMs == 50, "the offset comes off the best round trip");

    Vec<UInt8> scan;
    static_cast<Void>(pushScan(scan, scanOf(1, 1000000)));
    static_cast<Void>(feed(aged, scan, 1100));
    check(aged.offsetMs == 50, "an arrival sample cannot raise it back");

    // Local arrival alone would call this 100 ms old; through the board's clock
    // it is 150, and the LARGER wins.
    const Opt<link::Revolution> rev = aged.revolution(1200);
    check(rev.has_value() && rev->ageMs == 150, "so the age is the larger of the two");
}

static Void testSilenceInput()
{
    std::printf("\n-- every frame feeds the silence watchdog --\n");

    // Including one whose body this build has no name for. A reader that only
    // stamped the messages it understood would redial a perfectly live board the
    // day it meets a new type.
    Vec<UInt8> wire;
    static_cast<Void>(pushUnknown(wire));

    link::Session s;
    s.lastFrameMs = 0;
    static_cast<Void>(feed(s, wire, 7777));
    check(s.lastFrameMs == 7777, "an unknown frame still counts as contact");
    check(s.frames == 1u, "and is counted as a frame");
}

int main()
{
    std::printf("\nviewer link (bibowire client), no board attached\n");

    testGeometry();
    testByteBoundaries();
    testJunkAndUnknown();
    testCorruption();
    testTruncation();
    testBootId();
    testDecideTie();
    testStaleness();
    testArrivalFloor();
    testRoundTrip();
    testPingAndProse();
    testBoardAndControl();
    testGapsAndBye();
    testVersionRule();
    testBackoff();
    testSilenceInput();

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
