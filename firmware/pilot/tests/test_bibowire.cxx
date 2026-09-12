// bibowire's wire format and its deadman, held to their header.
//
//   tests\build_bibowire_test.bat run
//
// Pure byte work and pure arithmetic, like proto and scanwire: the same object
// file goes into the board's pilot and the Windows viewer, so what is proved
// here is proved for both ends at once - and the deadman is a pure function, so
// the safety property is exercised in microseconds on a laptop rather than by
// sleeping next to a car. That is the only way a deadman gets tested more than
// once.
//
// The cases that carry the weight, and why each is here:
//
//   - The TRUNCATION SWEEP. Every frame cut at every length must say
//     NEED_MORE and consume nothing. There is no path that yields a short
//     message, because a short message is a frame with the wrong count wearing
//     a valid frame's clothes.
//   - The CORRUPTION SWEEP. Every byte of a 500-point SCAN flipped in turn,
//     2540 of them, every one caught. None may come back as a valid-looking
//     scan with a different count - a wrong-but-plausible picture is the most
//     expensive failure available to a car that steers by what it sees.
//   - The FUZZ LOOP. 200,000 pseudo-random sequences through take(), from a
//     generator written here rather than rand(), because a fuzz failure nobody
//     can reproduce is a fuzz failure nobody will fix.
//   - The DEADMAN TABLE. The exact millisecond boundaries, and a negative age
//     reading as DEAD rather than as freshness.
//   - The LOCALE TEST. Nothing on this wire is a float, and this is the line
//     that catches the day somebody adds one.
//
// Section 11 numbers forty case groups; 35 to 40 belong to test_viewfeed.cxx,
// the socket suite, which is ctest-on-Linux only and out of this file's scope.
// Said out loud rather than quietly skipped.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "bibowire.hxx"

#include <clocale>
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

using namespace bibowire;

// A scratch pair of buffers plus the frame that came back out of them. One
// object rather than five locals at every call site, and the reason the helper
// below takes two parameters instead of eight.
struct Wire
{
    Vec<UInt8> body = Vec<UInt8>(24000, 0);
    Vec<UInt8> buf = Vec<UInt8>(24000, 0);
    Size bodyLen = 0;
    Size frameLen = 0;
    Frame frame;
};

[[nodiscard]] static Bool wrap(Wire* w, Type t, UInt16 seq)
{
    Head h;
    h.type = t;
    h.seq = seq;
    w->frameLen = put(h, Body{ w->body.data(), w->bodyLen }, w->buf.data(), w->buf.size());
    if(w->frameLen == 0)
    {
        return false;
    }
    Size used = 0;
    const Take got = take(w->buf.data(), w->frameLen, &w->frame, &used);
    return got == Take::TAKE_FRAME && used == w->frameLen;
}

static Void push(Vec<Str>* out, Wire* w, Type t, UInt16 seq)
{
    out->push_back(wrap(w, t, seq) ? describe(w->frame) : Str("ENCODE FAILED"));
}

[[nodiscard]] static Scan makeScan(Size count)
{
    Scan s;
    s.tMonoUs = 7000;
    s.revIndex = 41;
    s.freqMilliHz = 10123;
    s.health = 0;
    s.motor = 1;
    s.droppedSinceLast = 2;
    s.scanDivisor = 1;
    for(Size i = 0; i < count; ++i)
    {
        ScanPoint p;
        p.angleCentiDeg = static_cast<UInt16>((i * 72u) % 36000u);
        p.distMm = static_cast<UInt16>((i * 37u) % 12000u);
        s.points.push_back(p);
        s.quality.push_back(static_cast<UInt8>(i % 64u));
    }
    return s;
}

// A seeded xorshift, written here on purpose. rand() differs between libraries
// and srand(time(0)) differs between runs, and a fuzz case that cannot be
// reproduced from the source alone is a fuzz case nobody can fix.
struct Rng
{
    UInt32 state = 0x2545F491u;

    [[nodiscard]] UInt32 next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }
};

// One frame of every type, rendered. Built in one place so the describe cases
// and the locale case are looking at exactly the same bytes.
[[nodiscard]] static Vec<Str> buildAll()
{
    Vec<Str> out;
    Wire w;

    {
        Hello m;
        m.featureMask = 0x0000000Fu;
        m.viewerBuild = 0x03F9A1C2u;
        m.viewerUdpPort = 41234;
        m.wantControl = 1;
        m.controlHz = 20;
        m.name = "viewer";
        w.bodyLen = writeHello(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_HELLO, 1);
    }
    {
        Welcome m;
        m.sessionId = 0x0BADC0DEu;
        m.bootId = 0x00C0FFEEu;
        m.featureMask = 0x0000001Fu;
        m.accepted = 1;
        m.refusal = 0;
        m.boardMonoUs = 1234567;
        m.armEpoch = 3;
        m.capabilities = 0x0F;
        m.boardName = "bibobox";
        m.text = "control granted";
        w.bodyLen = writeWelcome(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_WELCOME, 2);
    }
    {
        Bye m;
        m.reason = Reason::REASON_VERSION;
        m.text = "rebuild the viewer";
        w.bodyLen = writeBye(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_BYE, 3);
    }
    {
        Ping m;
        m.token = 0x0011223344556677u;
        m.senderMonoUs = 42;
        w.bodyLen = writePing(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_PING, 4);
        push(&out, &w, Type::TYPE_PONG, 5);
    }
    {
        Leave m;
        m.sessionId = 0x0BADC0DEu;
        w.bodyLen = writeLeave(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_LEAVE, 6);
    }
    {
        const Scan m = makeScan(3);
        w.bodyLen = writeScan(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_SCAN, 7);
    }
    {
        Decide m;
        m.revIndex = 41;
        m.clearanceMm = 3410;
        m.hits = 12;
        m.steerMilli = -120;
        m.throttleMilli = 350;
        m.mode = 0;
        m.stop = 0;
        m.source = 2;
        m.modeMs = 800;
        w.bodyLen = writeDecide(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_DECIDE, 8);
    }
    {
        BoardState m;
        m.tMonoUs = 9;
        m.upS = 812;
        m.cpuCentiC = 5420;
        m.picoLink = 1;
        m.picoArmed = 1;
        m.pilotMode = 2;
        m.deadman = 0;
        m.lidarHealth = 0;
        m.lidarSpinning = 1;
        m.armEpoch = 3;
        m.controlHolder = 1;
        m.loopWorstUs = 1800;
        m.picoSilentMs = 41;
        m.revolutions = 8123;
        m.timeouts = 2;
        m.rxControl = 16240;
        m.rxControlStale = 3;
        m.ipv4 = 0xC0A82B07u;
        m.encodeAvgNs = 3100;
        m.encodeMaxNs = 9400;
        m.clients = 1;
        m.wifiName = "WhoopWhoop";
        w.bodyLen = writeBoard(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_BOARD, 9);
    }
    {
        LidarInfo m;
        m.model = 65;
        m.fwMajor = 1;
        m.fwMinor = 2;
        m.hwRev = 18;
        for(Size i = 0; i < m.serial.size(); ++i)
        {
            m.serial[i] = static_cast<UInt8>(i);
        }
        m.baud = 460800;
        w.bodyLen = writeLidarInfo(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_LIDAR_INFO, 10);
    }
    {
        Event m;
        m.tMonoUs = 5;
        m.severity = Severity::SEVERITY_WARN;
        m.code = 7;
        m.droppedSince = 0;
        m.text = "another program has /dev/ttyUSB0";
        w.bodyLen = writeEvent(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_EVENT, 11);
    }
    {
        CtlState m;
        m.tMonoUs = 11;
        m.ackSeq = 8814;
        m.controlAgeMs = 41;
        m.steerNowMilli = -120;
        m.throttleMilli = 0;
        m.escUs = 1602;
        m.neutralInMs = 109;
        m.disarmInMs = 259;
        m.armed = 1;
        m.armEpoch = 3;
        m.deadman = 0;
        m.refuse = Refuse::REFUSE_NONE;
        m.holder = 1;
        m.pilotMode = 2;
        m.scanAgeMs = 64;
        m.picoSilentMs = 41;
        m.lastCmdId = 12;
        w.bodyLen = writeCtlState(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_CTLSTATE, 12);
    }
    {
        CmdAck m;
        m.cmdId = 12;
        m.verb = Verb::VERB_ARM;
        m.result = 0;
        m.armEpoch = 4;
        m.text = "armed";
        w.bodyLen = writeCmdAck(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_CMDACK, 13);
    }
    {
        Camera m;
        m.tMonoUs = 1;
        m.frameIndex = 9;
        m.width = 640;
        m.height = 480;
        m.codec = 1;
        m.flags = 0;
        m.data = Vec<UInt8>(8, 0x5A);
        w.bodyLen = writeCamera(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_CAMERA, 14);
    }
    {
        Pose m;
        m.tMonoUs = 1;
        m.xMm = 100;
        m.yMm = -200;
        m.headingMilliRad = 1571;
        m.sigmaXyMm = 50;
        m.sigmaHeadingMilliRad = 20;
        m.source = 1;
        m.valid = 1;
        w.bodyLen = writePose(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_POSE, 15);
    }
    {
        Path m;
        m.tMonoUs = 1;
        m.seq = 4;
        m.points.push_back(PathPoint{ 100, 200 });
        m.points.push_back(PathPoint{ -300, 400 });
        m.points.push_back(PathPoint{ 0, 0 });
        w.bodyLen = writePath(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_PATH, 16);
    }
    {
        Waypoint m;
        m.tMonoUs = 1;
        m.seq = 4;
        m.index = 7;
        m.total = 12;
        m.xMm = 100;
        m.yMm = -200;
        m.flags = 0;
        w.bodyLen = writeWaypoint(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_WAYPOINT, 17);
    }
    {
        Control m;
        m.sessionId = 0x0BADC0DEu;
        m.seq = 8814;
        m.tMonoUs = 99;
        m.steerMilli = -120;
        m.throttleMilli = 350;
        m.buttons = BUTTON_ENABLE;
        m.armEpoch = 3;
        m.assumedMode = 2;
        w.bodyLen = writeControl(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_CONTROL, 18);
    }
    {
        Command m;
        m.sessionId = 0x0BADC0DEu;
        m.cmdId = 12;
        m.verb = Verb::VERB_SET_MODE;
        m.arg0 = 2;
        m.armEpoch = 3;
        w.bodyLen = writeCommand(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_COMMAND, 19);
    }
    {
        Subscribe m;
        m.sessionId = 0x0BADC0DEu;
        m.typeMask = 0x0000003Fu;
        m.scanDivisor = 3;
        w.bodyLen = writeSubscribe(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_SUBSCRIBE, 20);
    }
    {
        Describe m;
        m.type = 0x10;
        w.bodyLen = writeDescribe(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_DESCRIBE, 21);
    }
    {
        Schema m;
        m.text = "0x10 SCAN v1 24+4n+n";
        w.bodyLen = writeSchema(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_SCHEMA, 22);
    }
    {
        // Every optional field ABSENT. This is the row that proves a sentinel
        // renders as absence rather than as a measurement.
        BoardState m;
        w.bodyLen = writeBoard(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_BOARD, 40);
    }
    {
        // A negative centi-Celsius, so the hand-built decimal is exercised on
        // the side of zero where a sign is easy to lose.
        BoardState m;
        m.cpuCentiC = -55;
        w.bodyLen = writeBoard(m, w.body.data(), w.body.size());
        push(&out, &w, Type::TYPE_BOARD, 41);
    }
    return out;
}

// Feeds `stream` through take() `chunk` bytes at a time and logs what came out,
// so the same three-frame stream can be compared across every chunk size.
[[nodiscard]] static Str drain(const UInt8* stream, Size total, Size chunk)
{
    Vec<UInt8> ring;
    Str log;
    Size fed = 0;
    while(fed < total)
    {
        const Size n = chunk < total - fed ? chunk : total - fed;
        ring.insert(ring.end(), stream + fed, stream + fed + n);
        fed += n;
        for(;;)
        {
            Frame f;
            Size used = 0;
            const Take got = take(ring.data(), ring.size(), &f, &used);
            if(got == Take::TAKE_FRAME)
            {
                log += describe(f);
                log += '\n';
            }
            if(used == 0)
            {
                break;
            }
            ring.erase(ring.begin(), ring.begin() + static_cast<ISize>(used));
        }
    }
    return log;
}

// The body every type produces with nothing but its defaults - the constant
// term of its size expression, which is what the catalog claims.
[[nodiscard]] static Size defaultBody(Type t, UInt8* out, Size cap)
{
    switch(t)
    {
        case Type::TYPE_HELLO:
            return writeHello(Hello{}, out, cap);
        case Type::TYPE_WELCOME:
        {
            Welcome m;
            m.sessionId = 1;   // never 0 on the wire; see writeWelcome
            return writeWelcome(m, out, cap);
        }
        case Type::TYPE_BYE:
            return writeBye(Bye{}, out, cap);
        case Type::TYPE_PING:
        case Type::TYPE_PONG:
            return writePing(Ping{}, out, cap);
        case Type::TYPE_LEAVE:
            return writeLeave(Leave{}, out, cap);
        case Type::TYPE_SCAN:
            return writeScan(Scan{}, out, cap);
        case Type::TYPE_DECIDE:
            return writeDecide(Decide{}, out, cap);
        case Type::TYPE_BOARD:
            return writeBoard(BoardState{}, out, cap);
        case Type::TYPE_LIDAR_INFO:
            return writeLidarInfo(LidarInfo{}, out, cap);
        case Type::TYPE_EVENT:
            return writeEvent(Event{}, out, cap);
        case Type::TYPE_CTLSTATE:
            return writeCtlState(CtlState{}, out, cap);
        case Type::TYPE_CMDACK:
            return writeCmdAck(CmdAck{}, out, cap);
        case Type::TYPE_CAMERA:
            return writeCamera(Camera{}, out, cap);
        case Type::TYPE_POSE:
            return writePose(Pose{}, out, cap);
        case Type::TYPE_PATH:
            return writePath(Path{}, out, cap);
        case Type::TYPE_WAYPOINT:
            return writeWaypoint(Waypoint{}, out, cap);
        case Type::TYPE_CONTROL:
            return writeControl(Control{}, out, cap);
        case Type::TYPE_COMMAND:
            return writeCommand(Command{}, out, cap);
        case Type::TYPE_SUBSCRIBE:
            return writeSubscribe(Subscribe{}, out, cap);
        case Type::TYPE_DESCRIBE:
            return writeDescribe(Describe{}, out, cap);
        case Type::TYPE_SCHEMA:
            return writeSchema(Schema{}, out, cap);
        default:
            return 0;
    }
}

[[nodiscard]] static deadman::Inputs held(Int64 age)
{
    deadman::Inputs in;
    in.nowMs = 1000;
    in.lastControlMs = 1000 - age;
    in.haveHolder = true;
    in.enable = true;
    in.epochMatches = true;
    in.modeAgrees = true;
    in.estopLatched = false;
    return in;
}

[[nodiscard]] static control::Gate freshGate()
{
    control::Gate g;
    g.sessionId = 0x0BADC0DEu;
    g.highestSeq = 0;
    g.haveHolder = true;
    g.fromHolder = true;
    g.armEpoch = 3;
    g.pilotMode = 2;
    return g;
}

[[nodiscard]] static Control freshControl(UInt32 seq)
{
    Control c;
    c.sessionId = 0x0BADC0DEu;
    c.seq = seq;
    c.steerMilli = -120;
    c.throttleMilli = 350;
    c.buttons = BUTTON_ENABLE;
    c.armEpoch = 3;
    c.assumedMode = 2;
    return c;
}

Int32 main()
{
    std::printf("\nbibowire - the viewer wire, its framing and its deadman\n\n");

    // ---- 1. round trips, every type, at its extremes --------------------------
    {
        Wire w;
        Scan s = makeScan(0);
        s.points.push_back(ScanPoint{ 0, 0 });          // angle 0, NO RETURN
        s.quality.push_back(0);
        s.points.push_back(ScanPoint{ 35999, 12000 });  // the far edge of both
        s.quality.push_back(63);
        s.health = HEALTH_ABSENT;
        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(w.bodyLen == 24 + 8 + 4, "a 2-point SCAN body is 24 + 4n + pad(n)");
        check(wrap(&w, Type::TYPE_SCAN, 1), "it frames and comes back as a frame");

        Scan back;
        check(readScan(w.frame.body, 1, &back), "and reads back");
        check(back.points.size() == 2, "with both points");
        check(back.points[0].angleCentiDeg == 0, "angle 0 survives");
        check(back.points[0].distMm == 0, "distance 0 - no return - survives as 0");
        check(back.points[1].angleCentiDeg == 35999, "angle 35999 survives");
        check(back.points[1].distMm == 12000, "distance 12000 survives");
        check(back.quality[0] == 0, "quality 0 survives");
        check(back.quality[1] == 63, "quality 63 survives");
        check(
            back.health == HEALTH_ABSENT,
            "health 255 reads back as absent, not as 255 degrees of good"
        );
        check(back.tMonoUs == 7000 && back.revIndex == 41, "the timestamp and revIndex survive");
        check(back.freqMilliHz == 10123, "and the rate, in milli-hertz");
    }
    {
        Wire w;
        const Scan s = makeScan(1024);
        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(w.bodyLen == 24 + 4096 + 1024, "a 1024-point SCAN is MAX_SCAN_POINTS and encodes");
        check(wrap(&w, Type::TYPE_SCAN, 2), "and frames");
        Scan back;
        check(readScan(w.frame.body, 1, &back), "and reads back");
        check(back.points.size() == 1024, "with all 1024 points");

        Scan tooMany = makeScan(1025);
        check(
            writeScan(tooMany, w.body.data(), w.body.size()) == 0,
            "1025 points is refused - the bound is a promise"
        );
    }
    {
        Wire w;
        const Scan s = makeScan(0);
        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(w.bodyLen == 24, "a SCAN of zero points is still a SCAN, 24 bytes");
        check(wrap(&w, Type::TYPE_SCAN, 3), "and it frames");
        Scan back;
        check(readScan(w.frame.body, 1, &back) && back.points.empty(), "and reads back empty");
    }
    {
        Wire w;
        Decide m;
        m.steerMilli = -1000;
        m.throttleMilli = 1000;
        m.revIndex = 0;   // a BLIND tick with no revolution behind it
        w.bodyLen = writeDecide(m, w.body.data(), w.body.size());
        check(w.bodyLen == 24, "a DECIDE body is 24 bytes");
        check(wrap(&w, Type::TYPE_DECIDE, 4) && w.frameLen == 40, "and its frame is 40");
        Decide back;
        check(readDecide(w.frame.body, 1, &back), "it reads back");
        check(
            back.steerMilli == -1000 && back.throttleMilli == 1000,
            "steer -1000 and throttle +1000 survive"
        );
        check(back.revIndex == 0, "revIndex 0 survives as 0 - it means BLIND, not missing");

        m.steerMilli = -1001;
        check(
            writeDecide(m, w.body.data(), w.body.size()) == 0,
            "a steer of -1001 is refused by the encoder"
        );
        m.steerMilli = 0;
        m.throttleMilli = 1001;
        check(writeDecide(m, w.body.data(), w.body.size()) == 0, "and a throttle of 1001");
    }
    {
        Wire w;
        BoardState m;   // every optional field at its absent sentinel
        w.bodyLen = writeBoard(m, w.body.data(), w.body.size());
        check(w.bodyLen == 72, "a BOARD with no wifi name is 72 bytes");
        check(wrap(&w, Type::TYPE_BOARD, 5), "and frames");
        BoardState back;
        check(readBoard(w.frame.body, 1, &back), "it reads back");
        check(
            back.battMilliV == BATT_ABSENT,
            "battMilliV comes back 0xFFFF - NOT MEASURED, and 0 is a real dead pack"
        );
        check(back.cpuCentiC == CPU_ABSENT, "cpuCentiC comes back -32768");
        check(back.picoSilentMs == PICO_SILENT_ABSENT, "picoSilentMs comes back 0xFFFFFFFF");
        check(back.lidarHealth == HEALTH_ABSENT, "lidarHealth comes back 255");

        m.wifiName = "WhoopWhoop";
        m.battMilliV = 7412;
        m.cpuCentiC = -55;
        w.bodyLen = writeBoard(m, w.body.data(), w.body.size());
        check(w.bodyLen == 72 + 12, "with a 10-character wifi name it is 72 + pad(10)");
        check(wrap(&w, Type::TYPE_BOARD, 6), "and frames");
        check(readBoard(w.frame.body, 1, &back), "and reads back");
        checkStr(back.wifiName, "WhoopWhoop", "the connection name survives");
        check(back.battMilliV == 7412, "a real battery reading survives");
        check(back.cpuCentiC == -55, "and a negative temperature survives its sign");

        BoardState tooLong;
        tooLong.wifiName = Str(33, 'x');
        check(
            writeBoard(tooLong, w.body.data(), w.body.size()) == 0,
            "a 33-character wifi name is refused"
        );
    }
    {
        Wire w;
        Hello m;
        m.name = Str(31, 'n');
        w.bodyLen = writeHello(m, w.body.data(), w.body.size());
        check(w.bodyLen == 20 + 32, "a HELLO with a 31-character name is 20 + pad(31)");
        check(wrap(&w, Type::TYPE_HELLO, 7), "and frames");
        Hello back;
        check(
            readHello(w.frame.body, 1, &back) && back.name.size() == 31,
            "and the whole name comes back"
        );
        m.name = Str(32, 'n');
        check(
            writeHello(m, w.body.data(), w.body.size()) == 0,
            "a 32-character name is refused - MAX_NAME is 31"
        );
    }
    {
        Wire w;
        Welcome m;
        m.sessionId = 0;
        check(
            writeWelcome(m, w.body.data(), w.body.size()) == 0,
            "a WELCOME with sessionId 0 is refused - 0 means no session"
        );
        m.sessionId = 7;
        m.boardName = "bibobox";
        m.text = "ok";
        w.bodyLen = writeWelcome(m, w.body.data(), w.body.size());
        check(w.bodyLen == 40 + 12, "a WELCOME is 40 + pad(name + text)");
        check(wrap(&w, Type::TYPE_WELCOME, 8), "and frames");
        Welcome back;
        check(readWelcome(w.frame.body, 1, &back), "it reads back");
        checkStr(back.boardName, "bibobox", "the board name survives");
        checkStr(back.text, "ok", "and the sentence beside it");
        check(
            back.staleMs == 150 && back.deadMs == 300,
            "and the deadman numbers the viewer must use"
        );
    }
    {
        Wire w;
        Event m;
        m.text = Str(200, 'e');
        w.bodyLen = writeEvent(m, w.body.data(), w.body.size());
        check(w.bodyLen == 16 + 200, "an EVENT at MAX_EVENT_TEXT is 16 + 200");
        check(wrap(&w, Type::TYPE_EVENT, 9), "and frames");
        Event back;
        check(
            readEvent(w.frame.body, 1, &back) && back.text.size() == 200,
            "and 200 characters come back whole"
        );
        m.text = Str(201, 'e');
        check(
            writeEvent(m, w.body.data(), w.body.size()) == 0,
            "201 is refused - the bound is on the wire"
        );
    }
    {
        Wire w;
        Control m;
        m.sessionId = 0x0BADC0DEu;
        m.seq = 4294967295u;
        m.steerMilli = 1000;
        m.throttleMilli = -1000;
        m.buttons = static_cast<UInt16>(BUTTON_ESTOP | BUTTON_ENABLE | BUTTON_MOTOR_WANTED);
        m.armEpoch = 255;
        m.assumedMode = 2;
        w.bodyLen = writeControl(m, w.body.data(), w.body.size());
        check(w.bodyLen == 24, "a CONTROL body is 24 bytes");
        check(wrap(&w, Type::TYPE_CONTROL, 10) && w.frameLen == 40, "and its frame is 40");
        Control back;
        check(readControl(w.frame.body, 1, &back), "it reads back");
        check(back.seq == 4294967295u, "a seq at the top of the u32 survives");
        check(
            back.steerMilli == 1000 && back.throttleMilli == -1000,
            "full lock and full reverse survive"
        );
        check(back.buttons == 0x0007u, "all three button bits survive");
        check(back.armEpoch == 255, "and an epoch of 255");
    }
    {
        Wire w;
        CtlState m;
        m.escUs = ESC_ABSENT;
        m.controlAgeMs = CONTROL_AGE_NEVER;
        m.picoSilentMs = PICO_SILENT_ABSENT;
        m.refuse = Refuse::REFUSE_NO_UDP;
        w.bodyLen = writeCtlState(m, w.body.data(), w.body.size());
        check(w.bodyLen == 44, "a CTLSTATE body is 44 bytes");
        check(wrap(&w, Type::TYPE_CTLSTATE, 11) && w.frameLen == 60, "and its frame is 60");
        CtlState back;
        check(readCtlState(w.frame.body, 1, &back), "it reads back");
        check(back.escUs == ESC_ABSENT, "an unknown ESC pulse stays unknown");
        check(
            back.controlAgeMs == CONTROL_AGE_NEVER,
            "and a control that has never arrived stays never"
        );
        check(back.refuse == Refuse::REFUSE_NO_UDP, "and the refusal reason survives by name");
    }
    {
        Wire w;
        LidarInfo m;
        m.model = 65;
        m.serial[0] = 0xDE;
        m.serial[15] = 0xEF;
        m.baud = 460800;
        w.bodyLen = writeLidarInfo(m, w.body.data(), w.body.size());
        check(w.bodyLen == 32, "a LIDAR_INFO body is 32 bytes");
        check(wrap(&w, Type::TYPE_LIDAR_INFO, 12), "and frames");
        LidarInfo back;
        check(readLidarInfo(w.frame.body, 1, &back), "it reads back");
        check(
            back.serial[0] == 0xDE && back.serial[15] == 0xEF,
            "all sixteen serial bytes survive"
        );
        check(back.baud == 460800, "and the baud rate");
    }
    {
        Wire w;
        Pose m;
        m.xMm = -2147483647 - 1;
        m.yMm = 2147483647;
        m.headingMilliRad = -1;
        w.bodyLen = writePose(m, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_POSE, 13) && w.bodyLen == 32, "a POSE body is 32 bytes");
        Pose back;
        check(readPose(w.frame.body, 1, &back), "it reads back");
        check(back.xMm == -2147483647 - 1, "the most negative i32 survives");
        check(back.yMm == 2147483647, "and the most positive");
        check(back.headingMilliRad == -1, "and a heading of -1 is -1, not 4294967295");
    }
    {
        Wire w;
        Path m;
        m.points.push_back(PathPoint{ -1, 1 });
        w.bodyLen = writePath(m, w.body.data(), w.body.size());
        check(w.bodyLen == 16 + 8, "a 1-point PATH body is 16 + 8n");
        check(wrap(&w, Type::TYPE_PATH, 14), "and frames");
        Path back;
        check(readPath(w.frame.body, 1, &back), "it reads back");
        check(back.points.size() == 1 && back.points[0].xMm == -1, "with its signed coordinates");
    }
    {
        Wire w;
        Camera m;
        m.data = Vec<UInt8>(6, 0x11);
        w.bodyLen = writeCamera(m, w.body.data(), w.body.size());
        check(w.bodyLen == 24 + 8, "a 6-byte CAMERA payload pads to 24 + 8");
        check(wrap(&w, Type::TYPE_CAMERA, 15), "and frames");
        Camera back;
        check(readCamera(w.frame.body, 1, &back), "it reads back");
        check(back.data.size() == 6, "with exactly six bytes, not the padding");
    }
    {
        Wire w;
        Subscribe m;
        m.scanDivisor = 0;
        check(
            writeSubscribe(m, w.body.data(), w.body.size()) == 0,
            "a scanDivisor of 0 is refused - every zeroth revolution is not a rate"
        );
        m.scanDivisor = 3;
        m.sessionId = 1;
        w.bodyLen = writeSubscribe(m, w.body.data(), w.body.size());
        check(
            wrap(&w, Type::TYPE_SUBSCRIBE, 16) && w.bodyLen == 12,
            "a SUBSCRIBE body is 12 bytes"
        );
        Subscribe back;
        check(
            readSubscribe(w.frame.body, 1, &back) && back.scanDivisor == 3,
            "and the divisor survives"
        );
        check(back.camFps == 0, "a viewer that named no camera rate asks for none");

        // THE CAMERA RATE RIDES IN WHAT SECTION 5 CALLED reserved0. The body is
        // still 12 bytes and the version is unchanged, which is the whole point
        // of spending a reserved field rather than growing the frame: an older
        // board ignores those two bytes exactly as it always did, and a newer
        // board reading an older viewer sees 0 and keeps its own default.
        m.camFps = 12;
        w.bodyLen = writeSubscribe(m, w.body.data(), w.body.size());
        check(
            wrap(&w, Type::TYPE_SUBSCRIBE, 17) && w.bodyLen == 12,
            "a SUBSCRIBE carrying a camera rate is STILL 12 bytes"
        );
        Subscribe rate;
        check(
            readSubscribe(w.frame.body, 1, &rate) && rate.camFps == 12,
            "and the rate survives the round trip"
        );
        check(rate.scanDivisor == 3, "beside the divisor it shares the frame with");
    }

    // ---- 2 and 3. the edges the device itself defines -------------------------
    {
        Wire w;
        Scan s;
        s.points.push_back(ScanPoint{ 36000, 1000 });
        s.quality.push_back(10);
        check(
            writeScan(s, w.body.data(), w.body.size()) == 0,
            "an angle of exactly 36000 is refused - the device says 0"
        );

        s.points[0].angleCentiDeg = 35999;
        check(writeScan(s, w.body.data(), w.body.size()) != 0, "and 35999 is fine");

        s.quality[0] = 64;
        check(
            writeScan(s, w.body.data(), w.body.size()) == 0,
            "a quality of 64 is refused - the wire says 0..63"
        );
        s.quality[0] = 63;
        check(writeScan(s, w.body.data(), w.body.size()) != 0, "and 63 is fine");

        // The same two rules on the way IN, where a stranger writes the bytes.
        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_SCAN, 17), "a good one-point SCAN frames");
        w.buf[12 + 24] = 0xA0;
        w.buf[12 + 25] = 0x8C;   // 36000
        Scan back;
        check(
            !readScan(Body{ w.buf.data() + 12, w.bodyLen }, 1, &back),
            "an angle of 36000 on the wire is refused by the reader"
        );

        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_SCAN, 18), "and again");
        w.buf[12 + 28] = 64;
        check(
            !readScan(Body{ w.buf.data() + 12, w.bodyLen }, 1, &back),
            "a quality of 64 on the wire is refused by the reader"
        );
    }

    // ---- 4. describe(), one frame of every type ------------------------------
    const Vec<Str> rendered = buildAll();
    check(rendered.size() == 24, "one rendered frame per type, plus two BOARDs of sentinels");
    {
        Size i = 0;
        checkStr(
            rendered[i++],
            "HELLO v1 seq=1 len=28 : proto=1.0 features=0x0000000f build=0x03f9a1c2 udp=41234 want=1 hz=20 name=\"viewer\"",
            "describe: HELLO"
        );
        checkStr(
            rendered[i++],
            "WELCOME v1 seq=2 len=64 : proto=1.0 session=0x0badc0de boot=0x00c0ffee features=0x0000001f udp=8020 period=50 stale=150 dead=300 accepted=1 refusal=0 mono=1234567 epoch=3 caps=0x0f board=\"bibobox\" text=\"control granted\"",
            "describe: WELCOME"
        );
        checkStr(
            rendered[i++],
            "BYE v1 seq=3 len=24 : reason=version(1) text=\"rebuild the viewer\"",
            "describe: BYE"
        );
        checkStr(
            rendered[i++],
            "PING v1 seq=4 len=16 : token=0x0011223344556677 mono=42",
            "describe: PING"
        );
        checkStr(
            rendered[i++],
            "PONG v1 seq=5 len=16 : token=0x0011223344556677 mono=42",
            "describe: PONG"
        );
        checkStr(rendered[i++], "LEAVE v1 seq=6 len=8 : session=0x0badc0de", "describe: LEAVE");
        checkStr(
            rendered[i++],
            "SCAN v1 seq=7 len=40 : mono=7000 rev=41 hz=10123 n=3 health=good motor=1 dropped=2 div=1",
            "describe: SCAN"
        );
        checkStr(
            rendered[i++],
            "DECIDE v1 seq=8 len=24 : rev=41 clear=3410 hits=12 steer=-120 throttle=350 mode=cruise stop=0 source=drive modeMs=800",
            "describe: DECIDE"
        );
        checkStr(
            rendered[i++],
            "BOARD v1 seq=9 len=84 : mono=9 up=812 cpu=54.20 batt=n/a pico=up armed=1 mode=drive deadman=live health=good spin=1 epoch=3 holder=you loopWorst=1800 loopLate=0 picoSilent=41 revs=8123 timeouts=2 txDropped=0 rxCtl=16240 rxStale=3 ip=0xc0a82b07 encAvg=3100 encMax=9400 clients=1 wifi=\"WhoopWhoop\"",
            "describe: BOARD"
        );
        checkStr(
            rendered[i++],
            "LIDAR_INFO v1 seq=10 len=32 : model=65 fw=1.2 hw=18 serial=000102030405060708090A0B0C0D0E0F baud=460800",
            "describe: LIDAR_INFO"
        );
        checkStr(
            rendered[i++],
            "EVENT v1 seq=11 len=48 : mono=5 sev=warn code=7 dropped=0 text=\"another program has /dev/ttyUSB0\"",
            "describe: EVENT"
        );
        checkStr(
            rendered[i++],
            "CTLSTATE v1 seq=12 len=44 : mono=11 ack=8814 age=41 steerNow=-120 throttle=0 esc=1602 neutralIn=109 disarmIn=259 armed=1 epoch=3 deadman=live refuse=none holder=you mode=drive scanAge=64 picoSilent=41 lastCmd=12",
            "describe: CTLSTATE"
        );
        checkStr(
            rendered[i++],
            "CMDACK v1 seq=13 len=20 : cmd=12 verb=arm result=0 epoch=4 text=\"armed\"",
            "describe: CMDACK"
        );
        checkStr(
            rendered[i++],
            "CAMERA v1 seq=14 len=32 : mono=1 frame=9 size=640x480 codec=1 flags=0 bytes=8",
            "describe: CAMERA"
        );
        checkStr(
            rendered[i++],
            "POSE v1 seq=15 len=32 : mono=1 x=100 y=-200 heading=1571 sigmaXy=50 sigmaHeading=20 source=1 valid=1",
            "describe: POSE"
        );
        checkStr(rendered[i++], "PATH v1 seq=16 len=40 : mono=1 seq=4 n=3", "describe: PATH");
        checkStr(
            rendered[i++],
            "WAYPOINT v1 seq=17 len=32 : mono=1 seq=4 index=7 total=12 x=100 y=-200 flags=0x0000",
            "describe: WAYPOINT"
        );
        checkStr(
            rendered[i++],
            "CONTROL v1 seq=18 len=24 : session=0x0badc0de seq=8814 mono=99 steer=-120 throttle=350 buttons=0x0002 epoch=3 mode=drive",
            "describe: CONTROL"
        );
        checkStr(
            rendered[i++],
            "COMMAND v1 seq=19 len=16 : session=0x0badc0de cmd=12 verb=set_mode arg0=2 arg1=0 arg2=0 epoch=3",
            "describe: COMMAND"
        );
        checkStr(
            rendered[i++],
            "SUBSCRIBE v1 seq=20 len=12 : session=0x0badc0de mask=0x0000003f div=3",
            "describe: SUBSCRIBE"
        );
        checkStr(rendered[i++], "DESCRIBE v1 seq=21 len=4 : type=0x10", "describe: DESCRIBE");
        checkStr(
            rendered[i++],
            "SCHEMA v1 seq=22 len=24 : text=\"0x10 SCAN v1 24+4n+n\"",
            "describe: SCHEMA"
        );
        checkStr(
            rendered[i++],
            "BOARD v1 seq=40 len=72 : mono=0 up=0 cpu=n/a batt=n/a pico=down armed=2 mode=manual deadman=live health=n/a spin=0 epoch=0 holder=nobody loopWorst=0 loopLate=0 picoSilent=n/a revs=0 timeouts=0 txDropped=0 rxCtl=0 rxStale=0 ip=0x00000000 encAvg=0 encMax=0 clients=0 wifi=\"\"",
            "describe: BOARD with every sentinel renders n/a, never a number"
        );
        checkStr(
            rendered[i++],
            "BOARD v1 seq=41 len=72 : mono=0 up=0 cpu=-0.55 batt=n/a pico=down armed=2 mode=manual deadman=live health=n/a spin=0 epoch=0 holder=nobody loopWorst=0 loopLate=0 picoSilent=n/a revs=0 timeouts=0 txDropped=0 rxCtl=0 rxStale=0 ip=0x00000000 encAvg=0 encMax=0 clients=0 wifi=\"\"",
            "describe: a negative centi-Celsius keeps its sign through the hand-built decimal"
        );
    }

    // ---- 5. the truncation sweep ---------------------------------------------
    //
    // Every frame cut at every length from 0 to len-1 must say NEED_MORE and
    // consume NOTHING. There is no path that yields a short message.
    {
        Wire w;
        Int32 cuts = 0;
        Int32 wrong = 0;
        Int32 consumedSomething = 0;

        const Array<Size, 4> counts = { 0, 1, 3, 500 };
        for(const Size n : counts)
        {
            const Scan s = makeScan(n);
            w.bodyLen = writeScan(s, w.body.data(), w.body.size());
            if(!wrap(&w, Type::TYPE_SCAN, 20))
            {
                ++wrong;
                continue;
            }
            for(Size cut = 0; cut < w.frameLen; ++cut)
            {
                Frame f;
                Size used = 0;
                const Take got = take(w.buf.data(), cut, &f, &used);
                ++cuts;
                if(got != Take::TAKE_NEED_MORE)
                {
                    ++wrong;
                }
                if(used != 0)
                {
                    ++consumedSomething;
                }
            }
        }

        // The same sweep on a frame that is not a SCAN, so the rule is about
        // framing rather than about one body.
        {
            Ping p;
            p.token = 0x0102030405060708u;
            w.bodyLen = writePing(p, w.body.data(), w.body.size());
            if(wrap(&w, Type::TYPE_PING, 21))
            {
                for(Size cut = 0; cut < w.frameLen; ++cut)
                {
                    Frame f;
                    Size used = 0;
                    const Take got = take(w.buf.data(), cut, &f, &used);
                    ++cuts;
                    if(got != Take::TAKE_NEED_MORE)
                    {
                        ++wrong;
                    }
                    if(used != 0)
                    {
                        ++consumedSomething;
                    }
                }
            }
        }

        // 40 + 48 + 56 + 2540 for the four SCANs, 32 for the PING: every cut of
        // every one of them. Spelled out so a sweep that quietly stopped
        // sweeping cannot report success.
        check(
            cuts == 40 + 48 + 56 + 2540 + 32,
            "the truncation sweep covered every cut of five frames"
        );
        check(wrong == 0, "every truncated frame is NEED_MORE, never a short message");
        check(consumedSomething == 0, "and every one of them consumed nothing at all");
    }

    // ---- 6. the corruption sweep ---------------------------------------------
    {
        Wire w;
        const Scan s = makeScan(500);
        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(w.bodyLen == 2524, "a 500-point SCAN body is 2524 bytes");
        check(wrap(&w, Type::TYPE_SCAN, 22), "it frames and decodes");
        check(w.frameLen == 2540, "and a 500-point SCAN frame is 2540 bytes, as section 5 states");

        const Vec<UInt8> good(w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));
        Int32 caught = 0;
        Int32 slipped = 0;
        Int32 wrongCount = 0;
        for(Size i = 0; i < w.frameLen; ++i)
        {
            Vec<UInt8> bad = good;
            bad[i] = static_cast<UInt8>(bad[i] ^ 0xFFu);
            Frame f;
            Size used = 0;
            const Take got = take(bad.data(), bad.size(), &f, &used);
            if(got != Take::TAKE_FRAME)
            {
                ++caught;
                continue;
            }
            ++slipped;
            Scan back;
            if(readScan(f.body, f.head.ver, &back) && back.points.size() != 500)
            {
                ++wrongCount;
            }
        }
        check(caught == 2540, "all 2540 single-byte corruptions of a 500-point SCAN are caught");
        check(slipped == 0, "not one of them decodes as a frame");
        check(wrongCount == 0, "and none yields a valid-looking SCAN with a different count");
    }

    // ---- 7. resync, and the junk counted exactly -----------------------------
    {
        Wire w;
        Ping p;
        p.token = 0x0102030405060708u;
        w.bodyLen = writePing(p, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_PING, 23), "a PING frames");
        const Vec<UInt8> frame(w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));

        Int32 wrongSkip = 0;
        Int32 notFound = 0;
        for(Size junk = 0; junk <= 64; ++junk)
        {
            Vec<UInt8> stream;
            for(Size i = 0; i < junk; ++i)
            {
                // Deliberately includes the pair 0x42 0x57, so the scan has to
                // reject a candidate rather than lock onto the first magic.
                stream.push_back(
                    static_cast<UInt8>((i % 3u) == 0u ? 0x42u : ((i % 3u) == 1u ? 0x57u : 0x99u))
                );
            }
            stream.insert(stream.end(), frame.begin(), frame.end());

            Frame f;
            Size used = 0;
            Take got = take(stream.data(), stream.size(), &f, &used);
            Size skipped = 0;
            while(got == Take::TAKE_RESYNC)
            {
                skipped += used;
                got = take(stream.data() + skipped, stream.size() - skipped, &f, &used);
            }
            if(got != Take::TAKE_FRAME)
            {
                ++notFound;
            }
            if(skipped != junk)
            {
                ++wrongSkip;
            }
        }
        check(notFound == 0, "a frame behind 0 to 64 bytes of garbage is always found");
        check(
            wrongSkip == 0,
            "and the skipped bytes are counted EXACTLY, garbage spelling the magic included"
        );
    }
    {
        // Junk with no frame behind it at all: everything is consumed except a
        // trailing lone 0x42, which might still become a magic.
        const Array<UInt8, 5> junk = { 0x00, 0x11, 0x22, 0x33, 0x42 };
        Frame f;
        Size used = 0;
        check(take(junk.data(), junk.size(), &f, &used) == Take::TAKE_RESYNC, "pure junk resyncs");
        check(used == 4, "consuming everything but the trailing 0x42, which may yet be a magic");

        const Array<UInt8, 1> lone = { 0x42 };
        check(
            take(lone.data(), lone.size(), &f, &used) == Take::TAKE_NEED_MORE,
            "a lone 0x42 is NEED_MORE"
        );
        check(used == 0, "and consumes nothing");
        check(
            take(lone.data(), 0, &f, &used) == Take::TAKE_NEED_MORE,
            "an empty buffer is NEED_MORE"
        );
        check(used == 0, "and consumes nothing");
    }

    // ---- 8. headers that are refused rather than believed --------------------
    {
        Wire w;
        Ping p;
        w.bodyLen = writePing(p, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_PING, 24), "a PING frames");

        // A multiple of 4, beyond the bound, under a tag the reader knows: the
        // case the whole no-allocation rule exists for.
        Vec<UInt8> huge(w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));
        wr32(huge.data() + 8, 0xFFFFFFFCu);
        Frame f;
        Size used = 0;
        check(
            take(huge.data(), huge.size(), &f, &used) == Take::TAKE_TOO_BIG,
            "a len of 0xFFFFFFFC under a known tag is TAKE_TOO_BIG"
        );
        check(used == 0, "and nothing is consumed, so nothing was ever sized from the claim");

        // 0xFFFFFFFF is NOT a multiple of 4, so it fails the shape test before
        // the size bound is ever consulted. Rejected either way, and still
        // without allocating - but by a different rule, which is worth pinning
        // separately rather than assuming one covers the other.
        Vec<UInt8> odder(w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));
        wr32(odder.data() + 8, 0xFFFFFFFFu);
        check(
            take(odder.data(), odder.size(), &f, &used) != Take::TAKE_FRAME,
            "a len of 0xFFFFFFFF is refused as well - it is not a multiple of 4"
        );

        Vec<UInt8> odd(w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));
        wr32(odd.data() + 8, 17);
        check(
            take(odd.data(), odd.size(), &f, &used) != Take::TAKE_FRAME,
            "a len that is not a multiple of 4 is not a frame"
        );

        Vec<UInt8> zeroVer(w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));
        zeroVer[3] = 0;
        check(
            take(zeroVer.data(), zeroVer.size(), &f, &used) != Take::TAKE_FRAME,
            "ver 0 is not a frame - versions start at 1"
        );

        check(
            put(Head{}, Body{ w.body.data(), 3 }, w.buf.data(), w.buf.size()) == 0,
            "put refuses a body that is not a multiple of 4"
        );
        Head tiny;
        tiny.type = Type::TYPE_PING;
        check(
            put(tiny, Body{ w.body.data(), 16 }, w.buf.data(), 8) == 0,
            "and refuses a buffer it would overrun"
        );
    }

    // ---- 9. the same three frames, fed at every chunk size -------------------
    {
        Wire w;
        Vec<UInt8> stream;
        Ping p;
        p.token = 0x1122334455667788u;
        w.bodyLen = writePing(p, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_PING, 30), "frame one");
        stream.insert(stream.end(), w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));

        const Scan s = makeScan(7);
        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_SCAN, 31), "frame two");
        stream.insert(stream.end(), w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));

        Decide d;
        d.revIndex = 41;
        w.bodyLen = writeDecide(d, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_DECIDE, 32), "frame three");
        stream.insert(stream.end(), w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));

        const Str whole = drain(stream.data(), stream.size(), stream.size());
        Int32 differed = 0;
        for(Size chunk = 1; chunk <= stream.size(); ++chunk)
        {
            if(drain(stream.data(), stream.size(), chunk) != whole)
            {
                ++differed;
            }
        }
        check(!whole.empty(), "the three-frame stream renders");
        check(
            differed == 0,
            "and every chunk size from 1 byte to the whole stream gives identical output"
        );
    }

    // ---- 10. a SCAN whose count disagrees with its len -----------------------
    {
        Wire w;
        const Scan s = makeScan(8);
        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_SCAN, 33), "an 8-point SCAN frames");

        Scan back;
        back.revIndex = 999;
        wr16(w.buf.data() + 12 + 14, 7);
        check(
            !readScan(Body{ w.buf.data() + 12, w.bodyLen }, 1, &back),
            "a count of 7 in a body sized for 8 is refused"
        );
        check(
            back.revIndex == 999,
            "and the caller's Scan is not touched - readX never partially fills"
        );
        wr16(w.buf.data() + 12 + 14, 9);
        check(
            !readScan(Body{ w.buf.data() + 12, w.bodyLen }, 1, &back),
            "and so is a count of 9 - it is not a shorter revolution"
        );
    }

    // ---- 11. an unknown type is skipped by exactly len -----------------------
    {
        Wire w;
        Head h;
        h.type = static_cast<Type>(0x7E);
        h.seq = 34;
        for(Size i = 0; i < 8; ++i)
        {
            w.body[i] = static_cast<UInt8>(0xC0 + i);
        }
        const Size one = put(h, Body{ w.body.data(), 8 }, w.buf.data(), w.buf.size());
        check(one == 24, "a frame of an unknown type 0x7E is 16 + 8");

        Ping p;
        p.token = 7;
        Wire w2;
        w2.bodyLen = writePing(p, w2.body.data(), w2.body.size());
        check(wrap(&w2, Type::TYPE_PING, 35), "and a PING follows it");

        Vec<UInt8> stream(w.buf.begin(), w.buf.begin() + static_cast<ISize>(one));
        stream.insert(
            stream.end(),
            w2.buf.begin(),
            w2.buf.begin() + static_cast<ISize>(w2.frameLen)
        );

        Frame f;
        Size used = 0;
        check(
            take(stream.data(), stream.size(), &f, &used) == Take::TAKE_FRAME,
            "the unknown type still decodes as a frame"
        );
        check(
            used == 24,
            "and is skipped by EXACTLY its len - that is what the length prefix is for"
        );
        checkStr(
            describe(f),
            "UNKNOWN(0x7e) v1 seq=34 len=8 : skipped, 8 bytes",
            "and the renderer says it skipped it rather than pretending it understood"
        );
        check(
            take(stream.data() + used, stream.size() - used, &f, &used) == Take::TAKE_FRAME,
            "the frame after it parses"
        );
        check(f.head.type == Type::TYPE_PING, "and it is the PING");
    }

    // ---- 12. flags: one is ignored, one is refused ---------------------------
    {
        Wire w;
        Ping p;
        p.token = 3;
        w.bodyLen = writePing(p, w.body.data(), w.body.size());

        Head h;
        h.type = Type::TYPE_PING;
        h.seq = 36;
        h.flags = 0x0008;   // an annotating bit this build has no name for
        const Size n = put(h, Body{ w.body.data(), w.bodyLen }, w.buf.data(), w.buf.size());
        check(n == 32, "a frame carrying an unknown annotating flag is written");
        Frame f;
        Size used = 0;
        check(
            take(w.buf.data(), n, &f, &used) == Take::TAKE_FRAME,
            "and it is accepted - unknown annotating bits are IGNORED"
        );
        checkStr(
            describe(f),
            "PING v1 seq=36 len=16 [0x0008] : token=0x0000000000000003 mono=0",
            "the renderer shows the bit it ignored rather than dropping it silently"
        );

        h.flags = FLAG_ESTOP | FLAG_DEADMAN;
        const Size n2 = put(h, Body{ w.body.data(), w.bodyLen }, w.buf.data(), w.buf.size());
        check(
            take(w.buf.data(), n2, &f, &used) == Take::TAKE_FRAME,
            "estop and deadman ride every board frame"
        );
        checkStr(
            describe(f),
            "PING v1 seq=36 len=16 [estop,deadman] : token=0x0000000000000003 mono=0",
            "and both are named, so a stopped car is readable off any frame at all"
        );

        // FLAG_MORE changes FRAMING, so it may not be ignored - and put()
        // refuses to build one, which is why the byte is patched by hand here.
        h.flags = FLAG_MORE;
        check(
            put(h, Body{ w.body.data(), w.bodyLen }, w.buf.data(), w.buf.size()) == 0,
            "put refuses to build a FLAG_MORE frame in v1"
        );
        h.flags = 0;
        const Size n3 = put(h, Body{ w.body.data(), w.bodyLen }, w.buf.data(), w.buf.size());
        wr16(w.buf.data() + 4, FLAG_MORE);
        wr32(w.buf.data() + 12 + w.bodyLen, crc32c(w.buf.data(), 12 + w.bodyLen));
        check(n3 == 32, "a good frame is built, then FLAG_MORE is patched in and the CRC redone");
        check(
            take(w.buf.data(), n3, &f, &used) == Take::TAKE_BAD_FLAG,
            "and take refuses it - a framing flag may not be ignored"
        );
    }

    // ---- 13. a ver HIGHER than this build knows ------------------------------
    {
        Wire w;
        Decide d;
        d.revIndex = 41;
        d.clearanceMm = 3410;
        d.steerMilli = -120;
        w.bodyLen = writeDecide(d, w.body.data(), w.body.size());

        // A v2 sender appends four bytes this build has no name for.
        w.body[24] = 0xDE;
        w.body[25] = 0xAD;
        w.body[26] = 0xBE;
        w.body[27] = 0xEF;
        Head h;
        h.type = Type::TYPE_DECIDE;
        h.ver = 2;
        h.seq = 37;
        const Size n = put(h, Body{ w.body.data(), 28 }, w.buf.data(), w.buf.size());
        Frame f;
        Size used = 0;
        check(
            take(w.buf.data(), n, &f, &used) == Take::TAKE_FRAME,
            "a v2 DECIDE with a longer body still frames"
        );
        check(f.head.ver == 2, "and carries its version");

        Decide back;
        check(readDecide(f.body, f.head.ver, &back), "a v1 reader reads the prefix it understands");
        check(back.revIndex == 41 && back.clearanceMm == 3410, "with every known field exact");
        check(back.steerMilli == -120, "including the signed ones");

        // The same bytes claiming v1 are a wrong length and are refused.
        Decide untouched;
        untouched.revIndex = 777;
        check(
            !readDecide(f.body, 1, &untouched),
            "the same body claiming v1 is the wrong length and is refused"
        );
        check(untouched.revIndex == 777, "and the caller's Decide is untouched");
    }

    // ---- 14. absence is read, never fabricated -------------------------------
    {
        Wire w;
        BoardState m;
        m.upS = 5;
        w.bodyLen = writeBoard(m, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_BOARD, 38), "a BOARD with nothing measured frames");

        BoardState back;
        back.battMilliV = 1234;
        back.cpuCentiC = 99;
        check(readBoard(w.frame.body, 1, &back), "and reads back");
        check(
            back.battMilliV == BATT_ABSENT,
            "battMilliV is ABSENT, not the 1234 that was in the output"
        );
        check(back.cpuCentiC == CPU_ABSENT, "cpuCentiC is ABSENT, not the 99 that was there");
        check(back.picoSilentMs == PICO_SILENT_ABSENT, "picoSilentMs is ABSENT");
        check(back.lidarHealth == HEALTH_ABSENT, "lidarHealth is ABSENT");
        check(back.upS == 5, "and the one field that WAS measured came through");

        // A body one byte short is a wrong length, and a wrong length never
        // half-fills the answer.
        BoardState guard;
        guard.upS = 4242;
        check(
            !readBoard(Body{ w.frame.body.bytes, w.frame.body.len - 4 }, 1, &guard),
            "a BOARD body four bytes short is refused"
        );
        check(guard.upS == 4242, "and leaves the caller's BoardState exactly as it was");
    }

    // ---- 15. a major version mismatch, with a sentence -----------------------
    {
        Hello h;
        h.protoMajor = 2;
        h.protoMinor = 0;
        h.name = "viewer";
        check(!versionOk(h.protoMajor), "protoMajor 2 into a v1 board is refused - EQUAL, not >=");
        check(versionOk(PROTO_MAJOR), "and the board's own major is accepted");

        const Bye b = versionRefusal(h, "3f9a1c2");
        check(b.reason == Reason::REASON_VERSION, "the answer is BYE(VERSION)");
        check(
            !b.text.empty(),
            "with a NON-EMPTY sentence - an empty explanation is the failure the whole rule exists to prevent"
        );
        check(b.text.find("2.0") != Str::npos, "the sentence names what the viewer sent");
        check(b.text.find("1.0") != Str::npos, "and what the board speaks");
        check(
            b.text.find("3f9a1c2") != Str::npos,
            "and the board build, which is what turns it into an action"
        );

        Wire w;
        w.bodyLen = writeBye(b, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_BYE, 39), "and the refusal itself frames and decodes");
        Bye back;
        check(readBye(w.frame.body, 1, &back), "it reads back");
        checkStr(back.text, b.text.c_str(), "with the sentence intact");
    }

    // ---- 16. the catalog cannot drift from the codec -------------------------
    {
        check(catalogCount() == TYPE_COUNT, "the catalog has one row per Type");

        Vec<UInt8> body(24000, 0);
        Int32 lenMismatch = 0;
        Int32 nameless = 0;
        Int32 unwritable = 0;
        for(Size i = 0; i < catalogCount(); ++i)
        {
            const Desc& d = catalog()[i];
            if(d.name == nullptr || Str(d.name).empty())
            {
                ++nameless;
            }
            const Size wrote = defaultBody(d.type, body.data(), body.size());
            if(wrote == 0)
            {
                ++unwritable;
                continue;
            }
            if(wrote != d.fixedLen)
            {
                ++lenMismatch;
                std::printf(
                    "        %s: catalog says %u, writeX produced %u\n",
                    d.name,
                    static_cast<UInt32>(d.fixedLen),
                    static_cast<UInt32>(wrote)
                );
            }
        }
        check(nameless == 0, "every catalog row is named");
        check(unwritable == 0, "every Type in the catalog can be encoded from its defaults");
        check(lenMismatch == 0, "and every row's declared body length agrees with its own writeX");

        Int32 disagreed = 0;
        for(Size tag = 0; tag < 256u; ++tag)
        {
            const Bool known = knownType(static_cast<UInt8>(tag));
            const Bool listed = descOf(static_cast<Type>(tag)) != nullptr;
            if(known != listed)
            {
                ++disagreed;
            }
        }
        check(disagreed == 0, "knownType() and the catalog agree about all 256 tags");
        check(
            classOf(Type::TYPE_SCAN) == Class::CLASS_LIVE,
            "SCAN is LIVE - the newest revolution wins"
        );
        check(
            classOf(Type::TYPE_CAMERA) == Class::CLASS_BULK,
            "CAMERA is BULK - discarded before the car's picture"
        );
        check(classOf(Type::TYPE_BOARD) == Class::CLASS_VITAL, "BOARD is VITAL - never dropped");
    }

    // ---- 17. the SCHEMA line for SCAN ----------------------------------------
    {
        checkStr(
            schemaLine(Type::TYPE_SCAN),
            "0x10 SCAN v1 24+4n+n : u64 tMonoUs us ; u32 revIndex ; u16 freqMilliHz mHz ; u16 count ; u8 health ; u8 motor ; u16 droppedSinceLast ; u16 scanDivisor ; u16 reserved0 ; { u16 angleCentiDeg cdeg ; u16 distMm mm }[n] ; u8 quality[n]",
            "the SCHEMA line for SCAN, generated from the catalog rather than written twice"
        );
        check(
            schemaAll().find("0x40 CONTROL v1 24 :") != Str::npos,
            "and the whole schema names CONTROL too"
        );
        check(
            schemaLine(static_cast<Type>(0x7E)).empty(),
            "an unknown tag has no schema line to give"
        );
    }

    // ---- 18 to 23. applying a CONTROL ----------------------------------------
    {
        const control::Gate g = freshGate();
        const control::Outcome first = control::apply(g, freshControl(10));
        check(first.verdict == control::Verdict::VERDICT_APPLIED, "the first CONTROL is applied");
        check(first.highestSeq == 10, "and moves the high-water mark to its seq");
        check(
            first.steerMilli == -120 && first.throttleMilli == 350,
            "with both stick values reaching the car"
        );
        check(first.feedsDeadman, "and it feeds the deadman");

        control::Gate g2 = g;
        g2.highestSeq = 10;
        const control::Outcome same = control::apply(g2, freshControl(10));
        check(
            same.verdict == control::Verdict::VERDICT_STALE_SEQ,
            "a repeat of the applied seq is STALE"
        );
        check(
            same.steerMilli == 0 && same.throttleMilli == 0,
            "it moves neither steer nor throttle"
        );
        check(same.highestSeq == 10, "the high-water mark does not move");
        check(!same.feedsDeadman, "and a stale datagram does NOT feed the deadman");

        const control::Outcome older = control::apply(g2, freshControl(5));
        check(
            older.verdict == control::Verdict::VERDICT_STALE_SEQ,
            "and an older seq is stale too - newest wins"
        );
    }
    {
        check(control::newer(2, 1), "2 is newer than 1");
        check(!control::newer(1, 2), "and 1 is not newer than 2");
        check(control::newer(1, 4294967295u), "1 is newer than 0xFFFFFFFF - the comparison wraps");
        check(!control::newer(4294967295u, 1), "and 0xFFFFFFFF is not newer than 1");
        check(!control::newer(7, 7), "and a seq is never newer than itself");

        control::Gate stuck = freshGate();
        stuck.highestSeq = 5000;
        const control::Outcome refused = control::apply(stuck, freshControl(1));
        check(
            refused.verdict == control::Verdict::VERDICT_STALE_SEQ,
            "a viewer restarting at seq 1 is refused while the old mark stands"
        );

        control::Gate reset = freshGate();   // the handshake resets it, per session
        const control::Outcome accepted = control::apply(reset, freshControl(1));
        check(
            accepted.verdict == control::Verdict::VERDICT_APPLIED,
            "and accepted once the handshake has reset the mark"
        );
    }
    {
        const control::Gate g = freshGate();
        Control c = freshControl(11);
        c.armEpoch = 2;   // the board is at 3
        const control::Outcome o = control::apply(g, c);
        check(
            o.verdict == control::Verdict::VERDICT_APPLIED,
            "a stale armEpoch still counts as a datagram"
        );
        check(o.throttleMilli == 0, "but its THROTTLE is dropped to neutral");
        check(o.steerMilli == -120, "while its STEERING is still applied");
        check(
            o.refuse == Refuse::REFUSE_EPOCH,
            "and the reason is named REFUSE_EPOCH, not left to look like an ESC fault"
        );
    }
    {
        const control::Gate g = freshGate();
        Control c = freshControl(12);
        c.sessionId = 0xDEADBEEFu;
        const control::Outcome o = control::apply(g, c);
        check(
            o.verdict == control::Verdict::VERDICT_BAD_SESSION,
            "a foreign sessionId is discarded"
        );
        check(!o.feedsDeadman, "and does not feed the deadman");
        check(o.throttleMilli == 0 && o.steerMilli == 0, "and drives nothing");
    }
    {
        control::Gate g = freshGate();
        g.fromHolder = false;
        const control::Outcome o = control::apply(g, freshControl(13));
        check(
            o.verdict == control::Verdict::VERDICT_NOT_HOLDER,
            "a second viewer's CONTROL is discarded"
        );
        check(!o.feedsDeadman, "and DOES NOT FEED THE DEADMAN - the fatal flaw, pinned here");
        check(o.refuse == Refuse::REFUSE_NOT_HOLDER, "with a named reason");

        control::Gate none = freshGate();
        none.haveHolder = false;
        none.fromHolder = false;
        const control::Outcome o2 = control::apply(none, freshControl(14));
        check(
            o2.verdict == control::Verdict::VERDICT_NOT_HOLDER,
            "and with no holder at all it is still not applied"
        );
        check(!o2.feedsDeadman, "and still does not feed the timer");
    }
    {
        const control::Gate g = freshGate();   // the board is in drive (2)
        Control c = freshControl(15);
        c.assumedMode = 0;                     // the viewer believes manual
        const control::Outcome o = control::apply(g, c);
        check(o.refuse == Refuse::REFUSE_MODE, "a mode disagreement is REFUSE_MODE");
        check(o.throttleMilli == 0, "throttle is dropped");
        check(o.steerMilli == -120, "and steering is still applied");
    }

    // ---- 24 to 32. the deadman -----------------------------------------------
    {
        check(
            deadman::step(held(0)).state == deadman::State::STATE_LIVE,
            "silence of 0 ms is LIVE"
        );
        check(
            deadman::step(held(149)).state == deadman::State::STATE_LIVE,
            "149 ms is still LIVE - one lost datagram must not cut the throttle"
        );
        check(deadman::step(held(150)).state == deadman::State::STATE_SOFT, "150 ms is SOFT");
        check(deadman::step(held(299)).state == deadman::State::STATE_SOFT, "299 ms is still SOFT");
        check(deadman::step(held(300)).state == deadman::State::STATE_DEAD, "300 ms is DEAD");
        check(deadman::step(held(5000)).state == deadman::State::STATE_DEAD, "and it stays DEAD");
        check(
            deadman::step(held(150)).refuse == Refuse::REFUSE_DEADMAN_SOFT,
            "SOFT names itself REFUSE_DEADMAN_SOFT"
        );
        check(
            deadman::step(held(300)).refuse == Refuse::REFUSE_DEADMAN_DEAD,
            "and DEAD names itself REFUSE_DEADMAN_DEAD"
        );
    }
    {
        // A CONTROL applied at 149 ms resets the age to zero.
        deadman::Inputs in = held(149);
        check(deadman::step(in).state == deadman::State::STATE_LIVE, "at 149 ms it is LIVE");
        in.lastControlMs = in.nowMs;
        const deadman::Output o = deadman::step(in);
        check(o.state == deadman::State::STATE_LIVE, "a fresh CONTROL keeps it LIVE");
        check(o.neutralInMs == 150 && o.disarmInMs == 300, "and both countdowns are back to full");
    }
    {
        deadman::Inputs in = held(0);
        in.nowMs = 100;
        in.lastControlMs = 200;   // the future
        const deadman::Output o = deadman::step(in);
        check(
            o.state == deadman::State::STATE_DEAD,
            "a NEGATIVE age is DEAD, not LIVE - the safe reading of a bug is stopped"
        );
        check(o.refuse == Refuse::REFUSE_DEADMAN_DEAD, "with the dead reason");
        check(o.neutralInMs == 0 && o.disarmInMs == 0, "and no countdown left to run");
    }
    {
        deadman::Inputs in = held(5000);
        in.haveHolder = false;
        const deadman::Output o = deadman::step(in);
        check(
            o.state == deadman::State::STATE_LIVE,
            "with NO holder this timer imposes nothing, however long the silence"
        );
        check(o.refuse == Refuse::REFUSE_NONE, "and refuses nothing");
        check(o.neutralInMs == CONTROL_STALE_MS, "the countdowns sit at their full budgets");
        check(o.disarmInMs == CONTROL_DEAD_MS, "because nothing is counting down");
    }
    {
        deadman::Inputs in = held(0);
        in.enable = false;
        const deadman::Output o = deadman::step(in);
        check(
            o.state == deadman::State::STATE_SOFT,
            "enable clear is SOFT even on a perfectly fresh link"
        );
        check(o.refuse == Refuse::REFUSE_NOT_ARMED, "named REFUSE_NOT_ARMED");
        check(o.neutralInMs == 150, "and the link's own countdown keeps running underneath it");

        deadman::Inputs late = held(200);
        late.enable = false;
        check(
            deadman::step(late).state == deadman::State::STATE_SOFT,
            "at 200 ms with enable clear it is still SOFT"
        );
        deadman::Inputs gone = held(400);
        gone.enable = false;
        check(
            deadman::step(gone).state == deadman::State::STATE_DEAD,
            "but a DEAD link outranks a released enable - the stronger stop wins"
        );
    }
    {
        deadman::Inputs in = held(0);
        in.estopLatched = true;
        check(
            deadman::step(in).state == deadman::State::STATE_ESTOP,
            "estop outranks a healthy link"
        );
        check(deadman::step(in).refuse == Refuse::REFUSE_ESTOP, "and names itself");
        deadman::Inputs soft = held(200);
        soft.estopLatched = true;
        check(deadman::step(soft).state == deadman::State::STATE_ESTOP, "estop outranks SOFT");
        deadman::Inputs dead = held(400);
        dead.estopLatched = true;
        check(deadman::step(dead).state == deadman::State::STATE_ESTOP, "estop outranks DEAD");
        deadman::Inputs nobody = held(0);
        nobody.estopLatched = true;
        nobody.haveHolder = false;
        check(
            deadman::step(nobody).state == deadman::State::STATE_ESTOP,
            "and estop outranks having no holder at all"
        );
    }
    {
        check(deadman::step(held(0)).neutralInMs == 150, "at 0 ms, neutral in 150");
        check(deadman::step(held(0)).disarmInMs == 300, "at 0 ms, disarm in 300");
        check(deadman::step(held(74)).neutralInMs == 76, "at 74 ms, neutral in 76");
        check(deadman::step(held(74)).disarmInMs == 226, "at 74 ms, disarm in 226");
        check(deadman::step(held(149)).neutralInMs == 1, "at 149 ms, neutral in 1");
        check(deadman::step(held(149)).disarmInMs == 151, "at 149 ms, disarm in 151");
        check(deadman::step(held(150)).neutralInMs == 0, "at 150 ms neutral has already fired");
        check(deadman::step(held(150)).disarmInMs == 150, "and disarm is 150 away");
        check(deadman::step(held(299)).neutralInMs == 0, "at 299 ms neutral is still 0");
        check(deadman::step(held(299)).disarmInMs == 1, "and disarm is 1 away");
        check(deadman::step(held(300)).neutralInMs == 0, "at 300 ms both have fired");
        check(deadman::step(held(300)).disarmInMs == 0, "and neither is negative");
        check(
            deadman::step(held(100000)).disarmInMs == 0,
            "and long past the end they are still 0, never wrapped"
        );
        check(deadman::step(held(100000)).neutralInMs >= 0, "never negative");
    }
    {
        // A fresh stream alone does not undo a DEAD: the epoch moved when the
        // board disarmed, and only an explicit re-arm brings it back.
        deadman::Inputs in = held(0);
        in.epochMatches = false;
        const deadman::Output o = deadman::step(in);
        check(
            o.state != deadman::State::STATE_LIVE,
            "a fresh stream under a dead epoch is NOT LIVE"
        );
        check(o.state == deadman::State::STATE_SOFT, "it is SOFT");
        check(o.refuse == Refuse::REFUSE_EPOCH, "and says the epoch is why");
        in.epochMatches = true;
        check(
            deadman::step(in).state == deadman::State::STATE_LIVE,
            "and only the explicit re-arm path brings LIVE back"
        );

        deadman::Inputs mode = held(0);
        mode.modeAgrees = false;
        check(
            deadman::step(mode).refuse == Refuse::REFUSE_MODE,
            "a mode disagreement on a fresh link is REFUSE_MODE"
        );
        check(deadman::step(mode).state == deadman::State::STATE_SOFT, "and is SOFT");
    }
    {
        // The constants themselves, so raising one is a visible diff rather
        // than a quiet change of a safety margin.
        check(CONTROL_PERIOD_MS == 50, "CONTROL_PERIOD_MS is 50");
        check(CONTROL_STALE_MS == 150, "CONTROL_STALE_MS is 150");
        check(CONTROL_DEAD_MS == 300, "CONTROL_DEAD_MS is 300");
        check(REARM_STREAM_MS == 500, "REARM_STREAM_MS is 500");
        check(TICK_MS == 20, "TICK_MS is 20");
        check(PICO_HOP_BUDGET_MS == 100, "PICO_HOP_BUDGET_MS is 100");
        check(
            PICO_DEADMAN_MS == 400,
            "PICO_DEADMAN_MS is 400 - firmware/app/main.cxx:49, not ours to change"
        );
        static_assert(
            CONTROL_DEAD_MS + PICO_HOP_BUDGET_MS <= PICO_DEADMAN_MS,
            "the Pi's stop must beat the Pico's, or the blunt layer fires first"
        );
        check(
            CONTROL_DEAD_MS + PICO_HOP_BUDGET_MS <= PICO_DEADMAN_MS,
            "and the Pi's stop plus one Pico hop still beats the Pico's own 400"
        );
        checkStr(
            Str(deadman::stateName(deadman::State::STATE_LIVE)),
            "live",
            "the states have names"
        );
        checkStr(Str(refuseName(Refuse::REFUSE_NO_UDP)), "no_udp", "and so do the refusals");
    }

    // ---- 33. the locale trap -------------------------------------------------
    //
    // Nothing on this wire is a float and describe() builds its digits by hand,
    // so a comma-decimal machine must produce byte-identical output. This is the
    // line that catches the day somebody adds a float.
    {
        const Array<CharSeq, 6> names = {
            "German_Germany.1252", "de_DE.UTF-8", "de-DE",
            "French_France.1252", "fr_FR.UTF-8", "de_DE"
        };
        CharSeq got = nullptr;
        for(const CharSeq name : names)
        {
            if(got == nullptr)
            {
                got = std::setlocale(LC_ALL, name);
            }
        }

        // Whether the locale TOOK is measured, not assumed. A test that quietly
        // runs under "C" would compare two identical things and prove nothing,
        // which is this repo's named recurring bug.
        Array<Char, 32> probe{};
        std::snprintf(probe.data(), probe.size(), "%.1f", 1.5);
        const Bool comma = Str(probe.data()).find(',') != Str::npos;
        check(got != nullptr, "a comma-decimal locale could be installed");
        check(comma, "and it really is comma-decimal, so this test measures something");

        const Vec<Str> after = buildAll();
        check(after.size() == rendered.size(), "the whole message set still renders");
        check(after == rendered, "and every byte of it is identical under a comma decimal");
        check(after[22].find("cpu=n/a") != Str::npos, "an absent temperature is still n/a");
        check(
            after[23].find("cpu=-0.55") != Str::npos,
            "and a real one still has a DOT, not a comma"
        );

        got = std::setlocale(LC_ALL, "C");
        check(got != nullptr, "and the locale is put back for whatever runs next");
    }

    // ---- 34. the fuzz loop ---------------------------------------------------
    {
        Wire w;
        const Scan s = makeScan(6);
        w.bodyLen = writeScan(s, w.body.data(), w.body.size());
        check(wrap(&w, Type::TYPE_SCAN, 44), "a seed frame for the fuzz to splice in");
        const Vec<UInt8> seed(w.buf.begin(), w.buf.begin() + static_cast<ISize>(w.frameLen));

        Rng rng;
        const Size room = 96;
        const Size guard = 16;
        Int32 overran = 0;
        Int32 stalled = 0;
        Int32 badConsumed = 0;
        Int32 frames = 0;
        Int32 resyncs = 0;

        for(Int32 iter = 0; iter < 200000; ++iter)
        {
            Array<UInt8, 128> arena{};
            for(Size i = 0; i < arena.size(); ++i)
            {
                arena[i] = 0xA5;
            }
            UInt8* buf = arena.data() + guard;
            const Size n = static_cast<Size>(rng.next() % (room + 1u));

            // Every fourth case starts from real frame bytes, so the fuzz
            // spends time near valid headers instead of only in noise.
            Size copied = 0;
            if((rng.next() & 3u) == 0u)
            {
                copied = static_cast<Size>(rng.next() % (seed.size() + 1u));
                if(copied > n)
                {
                    copied = n;
                }
                for(Size i = 0; i < copied; ++i)
                {
                    buf[i] = seed[i];
                }
            }
            for(Size i = copied; i < n; ++i)
            {
                buf[i] = static_cast<UInt8>(rng.next());
            }

            Size at = 0;
            Int32 steps = 0;
            for(;;)
            {
                Frame f;
                Size used = 0;
                const Take got = take(buf + at, n - at, &f, &used);
                ++steps;
                if(used > n - at)
                {
                    ++badConsumed;
                    break;
                }
                if(got == Take::TAKE_FRAME)
                {
                    ++frames;
                    // Render it: a body that take() accepted must be safe to
                    // walk, and describe() walks all of it.
                    const Str rendering = describe(f);
                    if(rendering.empty())
                    {
                        ++badConsumed;
                    }
                }
                if(got == Take::TAKE_RESYNC)
                {
                    ++resyncs;
                }
                if(used == 0)
                {
                    break;
                }
                at += used;
                if(steps > static_cast<Int32>(n) + 2)
                {
                    ++stalled;
                    break;
                }
            }

            for(Size i = 0; i < guard; ++i)
            {
                if(arena[i] != 0xA5 || arena[arena.size() - 1u - i] != 0xA5)
                {
                    ++overran;
                }
            }
        }

        check(
            badConsumed == 0,
            "200,000 fuzz cases: take() never claims more bytes than it was given"
        );
        check(stalled == 0, "and always makes progress - no input drives it in circles");
        check(overran == 0, "and never writes outside the buffer it was handed");
        check(
            frames + resyncs > 0,
            "and the loop really did reach the decoder, not just the empty case"
        );
    }

    // ---- the tuning verbs, argument by argument -------------------------------
    //
    // Verbs 9, 10 and 11 carry their payload in arg0/arg1/arg2 rather than in
    // any new field, so COMMAND's 16 bytes did not change - which is exactly the
    // arrangement in which a dropped argument costs nothing at encode time and
    // shows up as a car trimmed to zero. Every check below asserts the VALUE
    // that came back, never that the decode merely succeeded: a readCommand
    // that forgot arg2 would still return true.
    {
        Wire w;
        {
            Command m;
            m.sessionId = 0x0BADC0DEu;
            m.cmdId = 77;
            m.verb = Verb::VERB_SET_SERVO_LIMITS;
            m.arg1 = SERVO_US_HARD_MIN;
            m.arg2 = SERVO_US_HARD_MAX;
            m.armEpoch = 5;
            w.bodyLen = writeCommand(m, w.body.data(), w.body.size());
            check(wrap(&w, Type::TYPE_COMMAND, 50), "SET_SERVO_LIMITS frames and comes back");

            Command back;
            check(readCommand(w.frame.body, 1, &back), "and reads back");
            check(back.verb == Verb::VERB_SET_SERVO_LIMITS, "verb 9 survives");
            check(back.arg1 == SERVO_US_HARD_MIN, "the servo's min us survives in arg1");
            check(back.arg2 == SERVO_US_HARD_MAX, "and its max us in arg2, not folded into one");
            check(back.cmdId == 77, "and the cmdId the CMDACK has to echo");
            check(back.armEpoch == 5, "and the epoch it was sent under");
        }
        {
            Command m;
            m.sessionId = 0x0BADC0DEu;
            m.cmdId = 78;
            m.verb = Verb::VERB_SET_ESC_LIMITS;
            m.arg1 = ESC_US_HARD_MIN;
            m.arg2 = ESC_US_HARD_MAX;
            w.bodyLen = writeCommand(m, w.body.data(), w.body.size());
            check(wrap(&w, Type::TYPE_COMMAND, 51), "SET_ESC_LIMITS frames and comes back");

            Command back;
            check(readCommand(w.frame.body, 1, &back), "and reads back");
            check(back.arg1 == ESC_US_HARD_MIN, "the throttle's min us survives in arg1");
            check(back.arg2 == ESC_US_HARD_MAX, "and its max us in arg2");
            check(
                ESC_US_HARD_MIN == 1500,
                "and the forward-only floor is still 1500 - reverse is not reached from a slider"
            );
        }
        {
            // A centre that is NOT the midpoint, and not any default. A trim of
            // 1500 would pass a test that had dropped the field entirely on a
            // build whose sentinel happened to be the midpoint; 1487 cannot.
            Command m;
            m.sessionId = 0x0BADC0DEu;
            m.cmdId = 79;
            m.verb = Verb::VERB_SET_SERVO_TRIM;
            m.arg1 = 1487;
            w.bodyLen = writeCommand(m, w.body.data(), w.body.size());
            check(wrap(&w, Type::TYPE_COMMAND, 52), "SET_SERVO_TRIM frames and comes back");

            Command back;
            check(readCommand(w.frame.body, 1, &back), "and reads back");
            check(back.verb == Verb::VERB_SET_SERVO_TRIM, "verb 10 survives");
            check(back.arg1 == 1487, "an off-centre centre survives exactly, not rounded to 1500");
            check(back.arg0 == 0, "arg0 is unused by trim and stays 0");
            check(back.arg2 == 0, "so does arg2");
        }
        {
            // Every field of SET_SLEW different from every other, so a decoder
            // that read arg0 out of arg1's offset fails rather than passing by
            // coincidence.
            Command m;
            m.sessionId = 0x0BADC0DEu;
            m.cmdId = 80;
            m.verb = Verb::VERB_SET_SLEW;
            m.arg0 = SLEW_AXIS_THROTTLE;
            m.arg1 = 37;
            m.armEpoch = 6;
            w.bodyLen = writeCommand(m, w.body.data(), w.body.size());
            check(wrap(&w, Type::TYPE_COMMAND, 53), "SET_SLEW frames and comes back");

            Command back;
            check(readCommand(w.frame.body, 1, &back), "and reads back");
            check(back.verb == Verb::VERB_SET_SLEW, "verb 11 survives");
            check(back.arg0 == SLEW_AXIS_THROTTLE, "the axis survives in arg0 - 2, not 0 for both");
            check(back.arg1 == 37, "and the us-per-tick in arg1");
            check(back.armEpoch == 6, "and the epoch, which sits past both args");
            check(
                37u * SLEW_TICKS_PER_S == 1850u,
                "and us-per-tick becomes us-per-second at 50 ticks a second"
            );
        }

        // The names, through the only public path that renders them: verbName
        // has internal linkage in bibowire.cxx, and describe() of a CMDACK is
        // where a person actually reads a verb's name back. A new verb missing
        // from that switch renders as "?" and fails here.
        {
            CmdAck a;
            a.cmdId = 77;
            a.verb = Verb::VERB_SET_SERVO_LIMITS;
            a.armEpoch = 5;
            a.text = "ok";
            w.bodyLen = writeCmdAck(a, w.body.data(), w.body.size());
            check(wrap(&w, Type::TYPE_CMDACK, 54), "a CMDACK for verb 9 frames");
            checkStr(
                describe(w.frame),
                "CMDACK v1 seq=54 len=16 : cmd=77 verb=set_servo_limits result=0 epoch=5 text=\"ok\"",
                "verbName: set_servo_limits"
            );
        }
        {
            CmdAck a;
            a.cmdId = 79;
            a.verb = Verb::VERB_SET_SERVO_TRIM;
            a.armEpoch = 5;
            a.text = "ok";
            w.bodyLen = writeCmdAck(a, w.body.data(), w.body.size());
            check(wrap(&w, Type::TYPE_CMDACK, 55), "a CMDACK for verb 10 frames");
            checkStr(
                describe(w.frame),
                "CMDACK v1 seq=55 len=16 : cmd=79 verb=set_servo_trim result=0 epoch=5 text=\"ok\"",
                "verbName: set_servo_trim"
            );
        }
        {
            // result = 3 as well as the name: refused-while-armed is the state
            // this verb spends most of its life in, and the number a viewer
            // branches on to say so.
            CmdAck a;
            a.cmdId = 80;
            a.verb = Verb::VERB_SET_SLEW;
            a.result = 3;
            a.armEpoch = 6;
            a.text = "ok";
            w.bodyLen = writeCmdAck(a, w.body.data(), w.body.size());
            check(wrap(&w, Type::TYPE_CMDACK, 56), "a CMDACK for verb 11 frames");
            checkStr(
                describe(w.frame),
                "CMDACK v1 seq=56 len=16 : cmd=80 verb=set_slew result=3 epoch=6 text=\"ok\"",
                "verbName: set_slew, and result 3 rides beside it"
            );
        }
    }

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
