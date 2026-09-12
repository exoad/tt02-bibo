// The .bibo archive container, held to its header.
//
//   tests\build_archive_test.bat run
//
// Pure, like proto, scanwire and bibowire: no socket, no device, and no clock
// this file does not control. The whole suite runs against a span of memory
// through the same Bytes and Sink the board uses against its SD card, so what
// is proved on the laptop is proved for the Pi - including the one case a real
// card cannot be asked for on demand, which is a run whose power was pulled
// before the footer was written.
//
// The cases that carry the weight, and why each is here:
//
//   - EVERY TYPE, BYTE-IDENTICAL. A recorded frame is the exact bytes put()
//     emitted. Byte-identity is the stronger claim and it is checked for all
//     twenty-two types; the per-type field decodes beside it are there to prove
//     the DECODE path too, rather than only that a memcpy is a memcpy.
//   - A CAMERA WHOSE JPEG IS HOSTILE. Runs of 0x00, runs of 0xFF, and the
//     bytes 0x42 0x57 - the frame magic itself - sitting inside the payload.
//     A container that re-synchronised on its own contents would cut a frame in
//     half here, and MJPEG is full of 0xFF.
//   - THE TRUNCATION SWEEP. The file cut at EVERY length. At each one the
//     reader must recover exactly the frames that are wholly present, invent
//     none, account for every byte it did not return, and never read past the
//     end - checked by a source that records the furthest offset it was asked
//     for.
//   - THE FOOTER IS NEVER TRUSTED UNVERIFIED. Absent, truncated, bad CRC, bad
//     version, a CRC-valid footer whose numbers cannot describe the file, and
//     an index that does not tile the frames it claims to - each falls back to
//     the linear scan, recovers the same frames, and SAYS SO.
//   - THE ACCOUNTING INVARIANT. On the scan path, the returned frame bytes plus
//     the junk plus the incomplete tail must equal the file exactly. That one
//     line is what makes "recovered every complete frame" a measurement rather
//     than a hope, and it is checked on every cut and on every fuzz case.
//   - THE FUZZ LOOP. Pseudo-random bytes from a generator written here rather
//     than rand(), because a fuzz failure nobody can reproduce is one nobody
//     will fix.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "archive.hxx"

#include <cstdio>
#include <cstdlib>
#include <cstring>

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
using namespace archive;

// A wall clock this file owns, so nothing here reads the machine's. 2026-09-10
// 00:00:00 UTC in unix microseconds - a real instant, chosen once, and the same
// on every run and every machine.
static const UInt64 EPOCH_US = 1788998400000000ull;

// ---- building frames --------------------------------------------------------

// One frame, encoded exactly as the wire would carry it. The archive stores
// `bytes` verbatim, so this is also the thing every round trip is compared with.
struct Made
{
    Vec<UInt8> bytes;
    Type type = Type::TYPE_PING;
    UInt16 seq = 0;
};

[[nodiscard]] static Made makeFrame(Type t, UInt16 seq, const UInt8* body, Size len)
{
    Made m;
    m.type = t;
    m.seq = seq;
    m.bytes.assign(FRAME_OVERHEAD + len, 0);
    Head h;
    h.type = t;
    h.seq = seq;
    const Size n = put(h, Body{ body, len }, m.bytes.data(), m.bytes.size());
    m.bytes.resize(n);
    return m;
}

// One frame of every Type, each carrying values a reader can tell apart from
// its neighbours - so a container that swapped two records would be caught by
// the fields and not only by the byte compare.
[[nodiscard]] static Vec<Made> buildOnePerType()
{
    Vec<Made> out;
    Vec<UInt8> b(300000, 0);
    Size n = 0;

    {
        Hello m;
        m.featureMask = 0x0000000Fu;
        m.viewerBuild = 0x03F9A1C2u;
        m.viewerUdpPort = 40100;
        m.wantControl = 1;
        m.name = "laptop";
        n = writeHello(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_HELLO, 1, b.data(), n));
    }
    {
        Welcome m;
        m.sessionId = 0x0BADC0DEu;
        m.bootId = 0x00C0FFEEu;
        m.boardMonoUs = 123456789ull;
        m.armEpoch = 3;
        m.capabilities = 0x0F;
        m.boardName = "bibobox";
        m.text = "accepted as control";
        n = writeWelcome(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_WELCOME, 2, b.data(), n));
    }
    {
        Bye m;
        m.reason = Reason::REASON_SHUTDOWN;
        m.text = "pilot stopping";
        n = writeBye(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_BYE, 3, b.data(), n));
    }
    {
        Ping m;
        m.token = 0xFEEDFACECAFEB0BAull;
        m.senderMonoUs = 200000ull;
        n = writePing(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_PING, 4, b.data(), n));
    }
    {
        Ping m;
        m.token = 0xFEEDFACECAFEB0BAull;
        m.senderMonoUs = 200500ull;
        n = writePing(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_PONG, 5, b.data(), n));
    }
    {
        Leave m;
        m.sessionId = 0x0BADC0DEu;
        n = writeLeave(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_LEAVE, 6, b.data(), n));
    }
    {
        Scan m;
        m.tMonoUs = 300000ull;
        m.revIndex = 41;
        m.freqMilliHz = 10123;
        m.health = 0;
        m.motor = 1;
        m.droppedSinceLast = 2;
        m.scanDivisor = 1;
        for(Size i = 0; i < 500u; ++i)
        {
            ScanPoint p;
            p.angleCentiDeg = static_cast<UInt16>((i * 72u) % 36000u);
            p.distMm = static_cast<UInt16>((i * 37u) % 12000u);
            m.points.push_back(p);
            m.quality.push_back(static_cast<UInt8>(i % 64u));
        }
        n = writeScan(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_SCAN, 7, b.data(), n));
    }
    {
        Decide m;
        m.revIndex = 41;
        m.clearanceMm = 3410;
        m.hits = 12;
        m.steerMilli = -120;
        m.throttleMilli = 350;
        m.mode = 0;
        m.source = 2;
        m.modeMs = 900;
        n = writeDecide(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_DECIDE, 8, b.data(), n));
    }
    {
        BoardState m;
        m.tMonoUs = 400000ull;
        m.upS = 812;
        m.cpuCentiC = 5420;
        m.picoLink = 1;
        m.pilotMode = 2;
        m.revolutions = 4412;
        m.wifiName = "FieldPhone";
        n = writeBoard(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_BOARD, 9, b.data(), n));
    }
    {
        LidarInfo m;
        m.model = 97;
        m.fwMajor = 1;
        m.fwMinor = 2;
        m.hwRev = 7;
        m.baud = 460800;
        for(Size i = 0; i < m.serial.size(); ++i)
        {
            m.serial[i] = static_cast<UInt8>(0xA0u + i);
        }
        n = writeLidarInfo(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_LIDAR_INFO, 10, b.data(), n));
    }
    {
        Event m;
        m.tMonoUs = 500000ull;
        m.severity = Severity::SEVERITY_WARN;
        m.code = 12;
        m.droppedSince = 3;
        m.text = "no control datagrams on UDP 8020";
        n = writeEvent(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_EVENT, 11, b.data(), n));
    }
    {
        CtlState m;
        m.tMonoUs = 600000ull;
        m.ackSeq = 8814;
        m.controlAgeMs = 41;
        m.steerNowMilli = -120;
        m.throttleMilli = 350;
        m.escUs = 1602;
        m.neutralInMs = 109;
        m.armed = 1;
        m.armEpoch = 3;
        m.refuse = Refuse::REFUSE_EPOCH;
        m.holder = 1;
        m.lastCmdId = 77;
        n = writeCtlState(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_CTLSTATE, 12, b.data(), n));
    }
    {
        CmdAck m;
        m.cmdId = 55;
        m.verb = Verb::VERB_ARM;
        m.result = 0;
        m.armEpoch = 4;
        m.text = "armed under epoch 4";
        n = writeCmdAck(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_CMDACK, 13, b.data(), n));
    }
    {
        Camera m;
        m.tMonoUs = 700000ull;
        m.frameIndex = 9;
        m.width = 640;
        m.height = 480;
        m.codec = 1;
        for(Size i = 0; i < 1000u; ++i)
        {
            m.data.push_back(static_cast<UInt8>(i * 7u));
        }
        n = writeCamera(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_CAMERA, 14, b.data(), n));
    }
    {
        Pose m;
        m.tMonoUs = 800000ull;
        m.xMm = -1234;
        m.yMm = 5678;
        m.headingMilliRad = -900;
        m.sigmaXyMm = 50;
        m.valid = 1;
        n = writePose(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_POSE, 15, b.data(), n));
    }
    {
        Path m;
        m.tMonoUs = 900000ull;
        m.seq = 4;
        for(Size i = 0; i < 12u; ++i)
        {
            PathPoint p;
            p.xMm = static_cast<Int32>(i) * 100 - 500;
            p.yMm = static_cast<Int32>(i) * -70;
            m.points.push_back(p);
        }
        n = writePath(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_PATH, 16, b.data(), n));
    }
    {
        Waypoint m;
        m.tMonoUs = 1000000ull;
        m.seq = 4;
        m.index = 7;
        m.total = 9;
        m.xMm = -4321;
        m.yMm = 8765;
        m.flags = 2;
        n = writeWaypoint(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_WAYPOINT, 17, b.data(), n));
    }
    {
        Control m;
        m.sessionId = 0x0BADC0DEu;
        m.seq = 9001;
        m.tMonoUs = 1100000ull;
        m.steerMilli = -1000;
        m.throttleMilli = 1000;
        m.buttons = BUTTON_ENABLE;
        m.armEpoch = 3;
        m.assumedMode = 2;
        n = writeControl(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_CONTROL, 18, b.data(), n));
    }
    {
        Command m;
        m.sessionId = 0x0BADC0DEu;
        m.cmdId = 55;
        m.verb = Verb::VERB_SET_ESC_LIMITS;
        m.arg1 = 1000;
        m.arg2 = 2000;
        m.armEpoch = 3;
        n = writeCommand(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_COMMAND, 19, b.data(), n));
    }
    {
        Subscribe m;
        m.sessionId = 0x0BADC0DEu;
        m.typeMask = 0x00FF00FFu;
        m.scanDivisor = 3;
        n = writeSubscribe(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_SUBSCRIBE, 20, b.data(), n));
    }
    {
        Describe m;
        m.type = 0x10;
        n = writeDescribe(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_DESCRIBE, 21, b.data(), n));
    }
    {
        Schema m;
        m.text = schemaLine(Type::TYPE_SCAN);
        n = writeSchema(m, b.data(), b.size());
        out.push_back(makeFrame(Type::TYPE_SCHEMA, 22, b.data(), n));
    }
    return out;
}

// ---- the sources this suite reads through -----------------------------------
//
// The furthest byte offset any read reached. "Never reads past the end" is a
// claim that has to be MEASURED, so the source records it and the tests compare
// it with the length they actually handed over.
static UInt64 probeEnd = 0;

static Size probeRead(const Void* ctx, UInt64 at, UInt8* out, Size want)
{
    const UInt64 end = at + static_cast<UInt64>(want);
    if(end > probeEnd)
    {
        probeEnd = end;
    }
    const UInt8* base = static_cast<const UInt8*>(ctx);
    std::memcpy(out, base + at, want);
    return want;
}

[[nodiscard]] static Bytes probeOver(const UInt8* data, Size len)
{
    Bytes b;
    b.ctx = data;
    b.total = static_cast<UInt64>(len);
    b.read = &probeRead;
    return b;
}

// ---- writing and reading whole archives -------------------------------------

struct Got
{
    Vec<Record> recs;
    Source beganAs = Source::SOURCE_NONE;
    Fallback beganWhy = Fallback::FALLBACK_NONE;
    Source endedAs = Source::SOURCE_NONE;
    Fallback endedWhy = Fallback::FALLBACK_NONE;
    UInt64 junk = 0;
    UInt64 tail = 0;
    Bool opened = false;
};

static Void collect(Reader* r, const Bytes& b, Got* g)
{
    *g = Got();
    g->opened = r->begin(b);
    if(!g->opened)
    {
        return;
    }
    g->beganAs = r->source;
    g->beganWhy = r->fallback;
    Record rec;
    while(r->next(&rec))
    {
        g->recs.push_back(rec);
    }
    g->endedAs = r->source;
    g->endedWhy = r->fallback;
    g->junk = r->junkBytes;
    g->tail = r->tailBytes;
}

[[nodiscard]] static Got readSpan(const UInt8* data, Size len)
{
    Reader r;
    Got g;
    collect(&r, probeOver(data, len), &g);
    return g;
}

[[nodiscard]] static Bool writeAll(Vec<UInt8>* out, const Vec<Made>& frames, Bool closeIt)
{
    Writer w;
    const Sink s = intoMemory(out);
    if(!w.begin(s, EPOCH_US))
    {
        return false;
    }
    for(const Made& m : frames)
    {
        if(!w.put(m.bytes.data(), m.bytes.size()))
        {
            return false;
        }
    }
    if(closeIt)
    {
        return w.finish();
    }
    w.abandon();
    return true;
}

[[nodiscard]] static const Record* findType(const Vec<Record>& v, Type t)
{
    for(const Record& r : v)
    {
        if(r.head.type == t)
        {
            return &r;
        }
    }
    return nullptr;
}

[[nodiscard]] static Bool sameBytes(const Record& r, const Made& m)
{
    if(r.frameLen != m.bytes.size())
    {
        return false;
    }
    if(r.body.size() + FRAME_OVERHEAD != m.bytes.size())
    {
        return false;
    }
    return std::memcmp(r.body.data(), m.bytes.data() + HEAD_BYTES, r.body.size()) == 0;
}

// Every byte of the file is either inside a frame that came back, or junk that
// was counted, or the incomplete tail. On the scan path those three must add up
// to the file exactly - which is what turns "recovered every complete frame"
// into a measurement instead of a hope.
[[nodiscard]] static Bool accountsForEveryByte(const Got& g, Size len)
{
    UInt64 used = 0;
    for(const Record& r : g.recs)
    {
        used += static_cast<UInt64>(r.frameLen);
    }
    return used + g.junk + g.tail == static_cast<UInt64>(len);
}

[[nodiscard]] static Bool footerAt(const Vec<UInt8>& bytes, Footer* out)
{
    if(bytes.size() < FOOTER_BYTES)
    {
        return false;
    }
    return unpackFooter(bytes.data() + (bytes.size() - FOOTER_BYTES), out);
}

static Void restamp(Vec<UInt8>* bytes, const Footer& f)
{
    packFooter(f, bytes->data() + (bytes->size() - FOOTER_BYTES));
}

// A seeded xorshift, written here on purpose. rand() differs between libraries
// and srand(time(0)) differs between runs, and a fuzz case that cannot be
// reproduced from the source alone is a fuzz case nobody can fix.
struct Rng
{
    UInt32 state = 0x9E3779B9u;

    [[nodiscard]] UInt32 next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }
};

[[nodiscard]] static Str tempPath()
{
    const Char* t = std::getenv("TEMP");
    if(t == nullptr)
    {
        t = std::getenv("TMPDIR");
    }
    if(t == nullptr)
    {
        return Str("/tmp/bibo-archive-test.bibo");
    }
    return Str(t) + "/bibo-archive-test.bibo";
}

Int32 main()
{
    std::printf("\narchive - the .bibo container, its footer and its fallback\n\n");

    const Vec<Made> all = buildOnePerType();

    // ---- 1. every message type, write then read, fields exact -----------------
    {
        check(all.size() == TYPE_COUNT, "one frame built for every bibowire Type");
        Bool encoded = true;
        for(const Made& m : all)
        {
            if(m.bytes.size() < FRAME_OVERHEAD)
            {
                encoded = false;
            }
        }
        check(encoded, "every one of them encoded onto the wire");

        Vec<UInt8> file;
        check(writeAll(&file, all, true), "a closed archive of all twenty-two writes");

        probeEnd = 0;
        const Got g = readSpan(file.data(), file.size());
        check(g.opened, "and opens");
        check(g.beganAs == Source::SOURCE_INDEX, "a properly closed archive reads by INDEX");
        check(g.beganWhy == Fallback::FALLBACK_NONE, "and reports no fallback");
        check(g.recs.size() == all.size(), "every frame comes back, and no extra one");
        check(probeEnd <= file.size(), "and nothing read past the end of the file");

        Bool order = true;
        Bool bytesOk = true;
        Bool heads = true;
        for(Size i = 0; i < g.recs.size() && i < all.size(); ++i)
        {
            if(g.recs[i].head.type != all[i].type)
            {
                order = false;
            }
            if(g.recs[i].head.seq != all[i].seq)
            {
                heads = false;
            }
            if(!sameBytes(g.recs[i], all[i]))
            {
                bytesOk = false;
            }
        }
        check(order, "in the order they were recorded");
        check(heads, "each keeping its own header seq");
        check(bytesOk, "and each byte-identical to what put() emitted");

        // Byte-identity is the stronger claim; these decode the recovered bodies
        // so the DECODE path is exercised too, per type, field by field.
        const Record* r = nullptr;

        r = findType(g.recs, Type::TYPE_HELLO);
        Hello hello;
        check(
            r != nullptr && readHello(Body{ r->body.data(), r->body.size() }, 1, &hello),
            "HELLO decodes"
        );
        checkStr(hello.name, "laptop", "HELLO name exact");
        check(hello.viewerBuild == 0x03F9A1C2u, "HELLO viewerBuild exact");

        r = findType(g.recs, Type::TYPE_WELCOME);
        Welcome wel;
        check(
            r != nullptr && readWelcome(Body{ r->body.data(), r->body.size() }, 1, &wel),
            "WELCOME decodes"
        );
        check(wel.sessionId == 0x0BADC0DEu, "WELCOME sessionId exact");
        check(
            wel.boardMonoUs == 123456789ull,
            "WELCOME boardMonoUs exact at its unaligned offset 28"
        );
        checkStr(wel.text, "accepted as control", "WELCOME sentence survives");

        r = findType(g.recs, Type::TYPE_BYE);
        Bye bye;
        check(
            r != nullptr && readBye(Body{ r->body.data(), r->body.size() }, 1, &bye),
            "BYE decodes"
        );
        check(bye.reason == Reason::REASON_SHUTDOWN, "BYE reason exact");
        checkStr(bye.text, "pilot stopping", "BYE sentence survives");

        r = findType(g.recs, Type::TYPE_PING);
        Ping ping;
        check(
            r != nullptr && readPing(Body{ r->body.data(), r->body.size() }, 1, &ping),
            "PING decodes"
        );
        check(ping.token == 0xFEEDFACECAFEB0BAull, "PING token exact across all 64 bits");

        r = findType(g.recs, Type::TYPE_PONG);
        Ping pong;
        check(
            r != nullptr && readPing(Body{ r->body.data(), r->body.size() }, 1, &pong),
            "PONG decodes"
        );
        check(pong.senderMonoUs == 200500ull, "PONG senderMonoUs exact");

        r = findType(g.recs, Type::TYPE_LEAVE);
        Leave leave;
        check(
            r != nullptr && readLeave(Body{ r->body.data(), r->body.size() }, 1, &leave),
            "LEAVE decodes"
        );
        check(leave.sessionId == 0x0BADC0DEu, "LEAVE sessionId exact");

        r = findType(g.recs, Type::TYPE_SCAN);
        Scan scan;
        check(
            r != nullptr && readScan(Body{ r->body.data(), r->body.size() }, 1, &scan),
            "SCAN decodes"
        );
        check(scan.points.size() == 500u, "SCAN keeps all 500 points");
        check(scan.quality.size() == 500u, "and all 500 qualities");
        check(scan.revIndex == 41u, "SCAN revIndex exact");
        check(
            scan.points[499].angleCentiDeg == static_cast<UInt16>((499u * 72u) % 36000u),
            "SCAN last angle exact"
        );
        check(scan.quality[63] == 63u, "SCAN a quality at the 63 ceiling survives");

        r = findType(g.recs, Type::TYPE_DECIDE);
        Decide dec;
        check(
            r != nullptr && readDecide(Body{ r->body.data(), r->body.size() }, 1, &dec),
            "DECIDE decodes"
        );
        check(dec.steerMilli == -120, "DECIDE a negative steer stays negative");
        check(dec.clearanceMm == 3410u, "DECIDE clearanceMm exact");

        r = findType(g.recs, Type::TYPE_BOARD);
        BoardState bs;
        check(
            r != nullptr && readBoard(Body{ r->body.data(), r->body.size() }, 1, &bs),
            "BOARD decodes"
        );
        check(
            bs.battMilliV == BATT_ABSENT,
            "BOARD an ABSENT battery is still absent after a round trip"
        );
        check(bs.cpuCentiC == 5420, "BOARD cpuCentiC exact");
        checkStr(
            bs.wifiName,
            "FieldPhone",
            "BOARD wifi NAME survives - and no passphrase exists to"
        );

        r = findType(g.recs, Type::TYPE_LIDAR_INFO);
        LidarInfo li;
        check(
            r != nullptr && readLidarInfo(Body{ r->body.data(), r->body.size() }, 1, &li),
            "LIDAR_INFO decodes"
        );
        check(li.baud == 460800u, "LIDAR_INFO baud exact");
        check(li.serial[15] == 0xAFu, "LIDAR_INFO the last serial byte exact");

        r = findType(g.recs, Type::TYPE_EVENT);
        Event ev;
        check(
            r != nullptr && readEvent(Body{ r->body.data(), r->body.size() }, 1, &ev),
            "EVENT decodes"
        );
        check(ev.severity == Severity::SEVERITY_WARN, "EVENT severity exact");
        checkStr(ev.text, "no control datagrams on UDP 8020", "EVENT prose survives verbatim");

        r = findType(g.recs, Type::TYPE_CTLSTATE);
        CtlState cs;
        check(
            r != nullptr && readCtlState(Body{ r->body.data(), r->body.size() }, 1, &cs),
            "CTLSTATE decodes"
        );
        check(
            cs.refuse == Refuse::REFUSE_EPOCH,
            "CTLSTATE the refusal REASON survives, not just the refusal"
        );
        check(cs.escUs == 1602u, "CTLSTATE escUs exact");

        r = findType(g.recs, Type::TYPE_CMDACK);
        CmdAck ack;
        check(
            r != nullptr && readCmdAck(Body{ r->body.data(), r->body.size() }, 1, &ack),
            "CMDACK decodes"
        );
        check(ack.verb == Verb::VERB_ARM, "CMDACK verb exact");
        checkStr(ack.text, "armed under epoch 4", "CMDACK sentence survives");

        r = findType(g.recs, Type::TYPE_CAMERA);
        Camera cam;
        check(
            r != nullptr && readCamera(Body{ r->body.data(), r->body.size() }, 1, &cam),
            "CAMERA decodes"
        );
        check(
            cam.codec == 1u,
            "CAMERA codec 1 (JPEG) echoed on the frame, so a capture is self-describing"
        );
        check(cam.data.size() == 1000u, "CAMERA payload length exact");
        check(cam.width == 640u && cam.height == 480u, "CAMERA dimensions exact");

        r = findType(g.recs, Type::TYPE_POSE);
        Pose pose;
        check(
            r != nullptr && readPose(Body{ r->body.data(), r->body.size() }, 1, &pose),
            "POSE decodes"
        );
        check(pose.xMm == -1234, "POSE a negative x stays negative");
        check(pose.headingMilliRad == -900, "POSE a negative heading stays negative");

        r = findType(g.recs, Type::TYPE_PATH);
        Path path;
        check(
            r != nullptr && readPath(Body{ r->body.data(), r->body.size() }, 1, &path),
            "PATH decodes"
        );
        check(path.points.size() == 12u, "PATH keeps all 12 points");
        check(path.points[11].yMm == -770, "PATH a negative y at the tail exact");

        r = findType(g.recs, Type::TYPE_WAYPOINT);
        Waypoint wp;
        check(
            r != nullptr && readWaypoint(Body{ r->body.data(), r->body.size() }, 1, &wp),
            "WAYPOINT decodes"
        );
        check(wp.index == 7u && wp.total == 9u, "WAYPOINT index and total exact");

        r = findType(g.recs, Type::TYPE_CONTROL);
        Control ctl;
        check(
            r != nullptr && readControl(Body{ r->body.data(), r->body.size() }, 1, &ctl),
            "CONTROL decodes"
        );
        check(ctl.seq == 9001u, "CONTROL seq exact");
        check(
            ctl.steerMilli == -1000 && ctl.throttleMilli == 1000,
            "CONTROL both milli extremes exact"
        );

        r = findType(g.recs, Type::TYPE_COMMAND);
        Command cmd;
        check(
            r != nullptr && readCommand(Body{ r->body.data(), r->body.size() }, 1, &cmd),
            "COMMAND decodes"
        );
        check(cmd.verb == Verb::VERB_SET_ESC_LIMITS, "COMMAND verb exact");
        check(cmd.arg1 == 1000u && cmd.arg2 == 2000u, "COMMAND both args exact");

        r = findType(g.recs, Type::TYPE_SUBSCRIBE);
        Subscribe sub;
        check(
            r != nullptr && readSubscribe(Body{ r->body.data(), r->body.size() }, 1, &sub),
            "SUBSCRIBE decodes"
        );
        check(sub.scanDivisor == 3u, "SUBSCRIBE scanDivisor exact");

        r = findType(g.recs, Type::TYPE_DESCRIBE);
        Describe desc;
        check(
            r != nullptr && readDescribe(Body{ r->body.data(), r->body.size() }, 1, &desc),
            "DESCRIBE decodes"
        );
        check(desc.type == 0x10u, "DESCRIBE type exact");

        r = findType(g.recs, Type::TYPE_SCHEMA);
        Schema sch;
        check(
            r != nullptr && readSchema(Body{ r->body.data(), r->body.size() }, 1, &sch),
            "SCHEMA decodes"
        );
        check(
            sch.text == schemaLine(Type::TYPE_SCAN),
            "SCHEMA text survives the container unchanged"
        );
    }

    // ---- 2. the footer's own numbers -----------------------------------------
    {
        Vec<UInt8> file;
        check(writeAll(&file, all, true), "a closed archive for the footer's own numbers");
        Footer f;
        check(footerAt(file, &f), "the footer is at EOF, its magic and CRC32C both good");
        check(f.version == VERSION, "footer version is this build's");
        check(f.frameCount == all.size(), "frameCount counts every frame");
        check(f.indexCount == f.frameCount, "and the index has one row per frame");
        check(f.fileBytes == file.size(), "fileBytes measures the whole file");
        check(f.wallEpochUs == EPOCH_US, "the wall-clock epoch is the one handed in");
        check(
            f.firstMonoUs == 123456789ull,
            "firstMonoUs is the FIRST frame that carried a clock (WELCOME)"
        );

        const Size scanSlot = typeSlot(static_cast<UInt8>(Type::TYPE_SCAN));
        const Size camSlot = typeSlot(static_cast<UInt8>(Type::TYPE_CAMERA));
        check(scanSlot < TYPE_COUNT && camSlot < TYPE_COUNT, "known tags have a per-type column");
        check(f.typeCounts[scanSlot] == 1u, "the per-type count for SCAN is 1");
        check(f.typeCounts[camSlot] == 1u, "the per-type count for CAMERA is 1");
        check(
            typeSlot(0x7Eu) == TYPE_COUNT,
            "a tag this build has no name for has no column, and says so"
        );

        UInt8 raw[FOOTER_BYTES];
        packFooter(f, raw);
        Footer back;
        check(unpackFooter(raw, &back), "a footer packs and unpacks");
        check(back.frameCount == f.frameCount, "and survives the round trip field for field");
        check(back.wallEpochUs == f.wallEpochUs, "including the full 64-bit wall clock");
    }

    // ---- 3. a CAMERA whose JPEG is hostile ------------------------------------
    //
    // MJPEG is full of 0xFF, and a container that resynchronised on its own
    // contents would cut a frame in half here. The payload deliberately contains
    // the frame magic 0x42 0x57 as well.
    {
        Camera m;
        m.tMonoUs = 4242424242ull;
        m.frameIndex = 3;
        m.width = 1280;
        m.height = 720;
        m.codec = 1;
        m.flags = 0;
        m.data.push_back(0xFFu);
        m.data.push_back(0xD8u);
        for(Size i = 0; i < 64u; ++i)
        {
            m.data.push_back(0x00u);
        }
        for(Size i = 0; i < 64u; ++i)
        {
            m.data.push_back(0xFFu);
        }
        m.data.push_back(MAGIC_LO);
        m.data.push_back(MAGIC_HI);
        m.data.push_back(MAGIC_LO);
        m.data.push_back(MAGIC_HI);
        for(Size i = 0; i < 4096u; ++i)
        {
            m.data.push_back(static_cast<UInt8>((i * 131u) ^ 0xA5u));
        }
        m.data.push_back(0xFFu);
        m.data.push_back(0xD9u);

        const Vec<UInt8> want = m.data;
        Vec<UInt8> body(300000, 0);
        const Size n = writeCamera(m, body.data(), body.size());
        check(n > 0u, "a hostile MJPEG payload encodes");

        Vec<Made> one;
        one.push_back(makeFrame(Type::TYPE_CAMERA, 77, body.data(), n));
        Vec<UInt8> file;
        check(writeAll(&file, one, true), "and records");

        const Got g = readSpan(file.data(), file.size());
        check(g.recs.size() == 1u, "and comes back as exactly one frame");
        Camera back;
        const Bool ok = g.recs.size() == 1u
            && readCamera(Body{ g.recs[0].body.data(), g.recs[0].body.size() }, 1, &back);
        check(ok, "and decodes");
        check(ok && back.data.size() == want.size(), "with its payload length unchanged");
        const Bool identical = ok && back.data.size() == want.size()
            && std::memcmp(back.data.data(), want.data(), want.size()) == 0;
        check(
            identical,
            "and its bytes IDENTICAL - 0x00 runs, 0xFF runs and the frame magic inside it"
        );
        check(
            ok && back.codec == 1u,
            "and codec 1 - there is no re-encode anywhere in this design"
        );

        // The scan path must survive it too: the fallback is the one that walks
        // the bytes, and it is the one the embedded magic is aimed at.
        Vec<UInt8> open;
        check(writeAll(&open, one, false), "the same frame with no footer");
        const Got s = readSpan(open.data(), open.size());
        check(s.beganAs == Source::SOURCE_SCAN, "reads by SCAN");
        check(s.recs.size() == 1u, "and the embedded 0x42 0x57 does not split it");
        const Bool same2 = s.recs.size() == 1u && sameBytes(s.recs[0], one[0]);
        check(same2, "and the scan recovers it byte-identically as well");
    }

    // ---- 4. missing footer: the pulled-power case ----------------------------
    {
        Vec<UInt8> closed;
        Vec<UInt8> open;
        check(writeAll(&closed, all, true), "the same run, closed");
        check(writeAll(&open, all, false), "and abandoned mid-run as a pulled battery leaves it");
        check(open.size() < closed.size(), "an abandoned run is shorter - no index, no footer");

        const Got a = readSpan(closed.data(), closed.size());
        const Got b = readSpan(open.data(), open.size());

        check(b.opened, "the abandoned run still OPENS - that is not a failure to report");
        check(b.beganAs == Source::SOURCE_SCAN, "and falls back to the linear scan");
        check(b.beganWhy == Fallback::FALLBACK_NO_FOOTER, "and SAYS the footer was not there");
        check(b.recs.size() == all.size(), "and recovers every single frame anyway");
        check(a.recs.size() == b.recs.size(), "the indexed and scanned paths agree on the count");

        Bool identical = a.recs.size() == b.recs.size();
        for(Size i = 0; i < a.recs.size() && i < b.recs.size(); ++i)
        {
            if(!sameBytes(b.recs[i], all[i]) || a.recs[i].head.type != b.recs[i].head.type)
            {
                identical = false;
            }
        }
        check(identical, "and frame for frame they are the same bytes in the same order");
        check(accountsForEveryByte(b, open.size()), "the scan accounts for every byte of the file");
        check(b.junk == 0u, "a cleanly abandoned run has no junk between its frames");
        check(b.tail == 0u, "and no incomplete tail, because it was abandoned between frames");
        check(
            std::strcmp(sourceName(b.beganAs), "scan") == 0,
            "the path has a name a person can print"
        );
        check(std::strcmp(fallbackName(b.beganWhy), "no_footer") == 0, "and so does the reason");
    }

    // ---- 5. a footer that must not be trusted --------------------------------
    {
        Vec<UInt8> good;
        check(writeAll(&good, all, true), "a closed archive to corrupt");

        // (a) one flipped byte in the middle of the footer.
        {
            Vec<UInt8> f = good;
            f[f.size() - 40u] ^= 0x40u;
            const Got g = readSpan(f.data(), f.size());
            check(g.beganAs == Source::SOURCE_SCAN, "a flipped footer byte falls back to the scan");
            check(g.beganWhy == Fallback::FALLBACK_BAD_CRC, "and names the CRC as the reason");
            check(g.recs.size() == all.size(), "and every frame is recovered regardless");
        }

        // (b) the magic destroyed.
        {
            Vec<UInt8> f = good;
            f[f.size() - FOOTER_BYTES] ^= 0xFFu;
            const Got g = readSpan(f.data(), f.size());
            check(
                g.beganWhy == Fallback::FALLBACK_NO_FOOTER,
                "a destroyed magic reads as NO FOOTER, not as a bad one"
            );
            check(g.recs.size() == all.size(), "and every frame is still recovered");
        }

        // (c) a version this build does not know, CRC recomputed so only the
        //     version is wrong.
        {
            Vec<UInt8> f = good;
            Footer ft;
            check(footerAt(f, &ft), "the good footer parses before being rewritten");
            ft.version = 99;
            restamp(&f, ft);
            const Got g = readSpan(f.data(), f.size());
            check(
                g.beganWhy == Fallback::FALLBACK_BAD_VERSION,
                "an unknown footer version falls back and says so"
            );
            check(g.recs.size() == all.size(), "and every frame is still recovered");
        }

        // (d) absurd counts, with a PERFECTLY VALID CRC. This is the case a
        //     CRC-only check would wave through: the bytes are not corrupt, the
        //     numbers are a lie.
        {
            Vec<UInt8> f = good;
            Footer ft;
            check(footerAt(f, &ft), "the good footer parses before the counts are made absurd");
            ft.frameCount = 0xFFFFFFFFFFFFFFF0ull;
            ft.indexCount = 0xFFFFFFFFFFFFFFF0ull;
            restamp(&f, ft);
            const Got g = readSpan(f.data(), f.size());
            check(
                g.beganWhy == Fallback::FALLBACK_INSANE,
                "a CRC-VALID footer with absurd counts is still refused"
            );
            check(g.recs.size() == all.size(), "and every frame is recovered by the scan");
        }

        // (e) a frame region that runs past EOF, CRC valid.
        {
            Vec<UInt8> f = good;
            Footer ft;
            check(
                footerAt(f, &ft),
                "the good footer parses before its offsets are pushed past EOF"
            );
            ft.frameBytes = ft.fileBytes + 4096ull;
            restamp(&f, ft);
            const Got g = readSpan(f.data(), f.size());
            check(g.beganWhy == Fallback::FALLBACK_INSANE, "an index offset past EOF is refused");
            check(g.recs.size() == all.size(), "and every frame is recovered by the scan");
        }

        // (f) a fileBytes that does not match the file in front of the reader.
        {
            Vec<UInt8> f = good;
            Footer ft;
            check(footerAt(f, &ft), "the good footer parses before fileBytes is made to disagree");
            ft.fileBytes = ft.fileBytes - 4ull;
            restamp(&f, ft);
            const Got g = readSpan(f.data(), f.size());
            check(
                g.beganWhy == Fallback::FALLBACK_INSANE,
                "a footer describing a DIFFERENT file is refused"
            );
            check(g.recs.size() == all.size(), "and every frame is recovered by the scan");
        }

        // (g) frameCount and indexCount disagreeing, CRC valid.
        {
            Vec<UInt8> f = good;
            Footer ft;
            check(footerAt(f, &ft), "the good footer parses before its two counts are split");
            ft.frameCount = ft.frameCount + 1ull;
            restamp(&f, ft);
            const Got g = readSpan(f.data(), f.size());
            check(
                g.beganWhy == Fallback::FALLBACK_INSANE,
                "a footer whose own two counts disagree is refused"
            );
        }

        // (h) an index that does not tile the frames. The footer's CRC does not
        //     cover the index, so this is a valid footer over a broken index.
        {
            Vec<UInt8> f = good;
            Footer ft;
            check(footerAt(f, &ft), "the good footer parses before the index is broken");
            const Size firstEntry = static_cast<Size>(ft.frameBytes);
            f[firstEntry] = 0x08u;   // entry 0 now claims to start at offset 8
            const Got g = readSpan(f.data(), f.size());
            check(
                g.beganWhy == Fallback::FALLBACK_BAD_INDEX,
                "an index that does not tile the frames is refused"
            );
            check(g.recs.size() == all.size(), "and every frame is recovered by the scan");
        }

        // (i) a valid footer and a valid tiling, over a frame that has been
        //     corrupted. The index cannot be caught before the payload is read,
        //     so this is the MID-RUN fallback, and it must not re-serve what it
        //     has already handed out.
        {
            Vec<UInt8> f = good;
            const Size hit = all[0].bytes.size() + 8u;   // inside frame 1, not frame 0
            f[hit] ^= 0xFFu;
            const Got g = readSpan(f.data(), f.size());
            check(
                g.beganAs == Source::SOURCE_INDEX,
                "it begins on the index, which looked perfectly good"
            );
            check(
                g.endedAs == Source::SOURCE_SCAN,
                "and ends on the scan, having stopped believing it"
            );
            check(g.endedWhy == Fallback::FALLBACK_BAD_INDEX, "reporting the index as the reason");
            check(g.recs.size() == all.size() - 1u, "the corrupted frame is lost and NOT invented");
            Bool dupes = false;
            for(Size i = 1; i < g.recs.size(); ++i)
            {
                if(g.recs[i].offset <= g.recs[i - 1u].offset)
                {
                    dupes = true;
                }
            }
            check(!dupes, "and no frame is served twice by the changeover");
        }
    }

    // ---- 6. the index really points at frame starts ---------------------------
    {
        Vec<UInt8> file;
        check(writeAll(&file, all, true), "a closed archive to seek around in");
        Reader r;
        check(r.begin(probeOver(file.data(), file.size())), "opens");
        check(r.source == Source::SOURCE_INDEX, "on the index");

        Bool everyOne = true;
        Bool offsetsMatch = true;
        Bool typesMatch = true;
        UInt64 want = 0;
        for(UInt64 i = 0; i < r.foot.indexCount; ++i)
        {
            Entry e;
            Record rec;
            if(!r.entry(i, &e))
            {
                everyOne = false;
                break;
            }
            if(e.offset != want)
            {
                offsetsMatch = false;
            }
            // The whole claim: seek straight there and parse.
            if(!r.at(i, &rec))
            {
                everyOne = false;
                break;
            }
            if(rec.head.type != all[static_cast<Size>(i)].type || e.type != static_cast<UInt8>(rec.head.type))
            {
                typesMatch = false;
            }
            if(!sameBytes(rec, all[static_cast<Size>(i)]))
            {
                everyOne = false;
            }
            want += e.frameLen;
        }
        check(everyOne, "every index offset seeks to a frame that parses, byte-identical");
        check(offsetsMatch, "and the offsets tile the frame region with no gap and no overlap");
        check(typesMatch, "and each entry's cached type is the frame's own");
        check(want == r.foot.frameBytes, "and the last one ends exactly where the index begins");
        Record none;
        check(!r.at(r.foot.indexCount, &none), "seeking one past the last entry is refused");
        check(!r.at(0xFFFFFFFFull, &none), "and so is a wild index");
    }

    // ---- 7. empty, exactly one, and multi-megabyte ---------------------------
    {
        Vec<UInt8> empty;
        const Vec<Made> nothing;
        check(writeAll(&empty, nothing, true), "an archive of no frames at all closes");
        check(empty.size() == FOOTER_BYTES, "and is exactly a footer");
        const Got ge = readSpan(empty.data(), empty.size());
        check(ge.opened, "an empty archive opens");
        check(ge.beganAs == Source::SOURCE_INDEX, "on its index");
        check(ge.recs.size() == 0u, "and yields no frames - which is the truth, not a failure");
        Footer fe;
        check(footerAt(empty, &fe) && fe.frameCount == 0u, "its footer says zero frames");

        Vec<UInt8> nofoot;
        const Got gn = readSpan(nofoot.data(), 0u);
        check(gn.opened, "a zero-byte file opens rather than erroring");
        check(gn.recs.size() == 0u, "and yields nothing");
        check(gn.beganWhy == Fallback::FALLBACK_NO_FOOTER, "and says there was no footer to read");

        Vec<Made> one;
        one.push_back(all[6]);   // the 500-point SCAN
        Vec<UInt8> single;
        check(writeAll(&single, one, true), "an archive of exactly one frame closes");
        const Got g1 = readSpan(single.data(), single.size());
        check(g1.recs.size() == 1u, "and yields exactly one");
        check(g1.recs.size() == 1u && sameBytes(g1.recs[0], one[0]), "byte-identical");
        Vec<UInt8> single2;
        check(writeAll(&single2, one, false), "the same single frame, abandoned");
        const Got g2 = readSpan(single2.data(), single2.size());
        check(g2.recs.size() == 1u, "still yields exactly one by scan");
        check(g2.recs.size() == 1u && sameBytes(g2.recs[0], one[0]), "and still byte-identical");

        // Multi-megabyte: ten CAMERA frames near the wire's largest payload.
        Vec<Made> big;
        Vec<UInt8> body(300000, 0);
        for(Size k = 0; k < 10u; ++k)
        {
            Camera m;
            m.tMonoUs = 2000000ull + k * 33000ull;
            m.frameIndex = static_cast<UInt32>(k);
            m.width = 640;
            m.height = 480;
            m.codec = 1;
            m.data.assign(250000u, 0);
            for(Size i = 0; i < m.data.size(); ++i)
            {
                m.data[i] = static_cast<UInt8>((i * 31u + k * 7u) & 0xFFu);
            }
            const Size n = writeCamera(m, body.data(), body.size());
            big.push_back(makeFrame(Type::TYPE_CAMERA, static_cast<UInt16>(k), body.data(), n));
        }
        Vec<UInt8> huge;
        check(
            writeAll(&huge, big, true),
            "a multi-megabyte archive of ten near-maximum CAMERA frames"
        );
        check(huge.size() > 2u * 1024u * 1024u, "and it really is over two megabytes");
        probeEnd = 0;
        const Got gh = readSpan(huge.data(), huge.size());
        check(gh.recs.size() == 10u, "all ten come back");
        check(probeEnd <= huge.size(), "and nothing read past the end");
        Bool bigOk = gh.recs.size() == 10u;
        for(Size i = 0; i < gh.recs.size() && i < big.size(); ++i)
        {
            if(!sameBytes(gh.recs[i], big[i]))
            {
                bigOk = false;
            }
        }
        check(bigOk, "each byte-identical across a quarter-megabyte payload");

        // And by scan, where the window has to assemble each one across refills.
        Vec<UInt8> hugeOpen;
        check(writeAll(&hugeOpen, big, false), "the same run abandoned");
        const Got gs = readSpan(hugeOpen.data(), hugeOpen.size());
        check(gs.beganAs == Source::SOURCE_SCAN, "reads by scan");
        check(gs.recs.size() == 10u, "and still finds all ten across the window refills");
        check(
            accountsForEveryByte(gs, hugeOpen.size()),
            "accounting for every byte of two megabytes"
        );
    }

    // ---- 8. the truncation sweep ---------------------------------------------
    //
    // The file cut at EVERY length. This is the case a car actually produces.
    {
        Vec<Made> six;
        six.push_back(all[3]);    // PING
        six.push_back(all[7]);    // DECIDE
        six.push_back(all[10]);   // EVENT
        six.push_back(all[11]);   // CTLSTATE
        six.push_back(all[13]);   // CAMERA
        six.push_back(all[17]);   // CONTROL
        Vec<UInt8> file;
        check(writeAll(&file, six, true), "a six-frame archive to cut apart");

        // Where each frame ends, so "how many are wholly present" is arithmetic
        // rather than a guess.
        Vec<Size> endsAt;
        Size run = 0;
        for(const Made& m : six)
        {
            run += m.bytes.size();
            endsAt.push_back(run);
        }

        Size wrongCount = 0;
        Size wrongBytes = 0;
        Size overran = 0;
        Size unaccounted = 0;
        Size neverOpened = 0;
        Reader r;
        Got g;
        for(Size cut = 0; cut <= file.size(); ++cut)
        {
            Size expect = 0;
            for(Size i = 0; i < endsAt.size(); ++i)
            {
                if(endsAt[i] <= cut)
                {
                    ++expect;
                }
            }
            probeEnd = 0;
            collect(&r, probeOver(file.data(), cut), &g);
            if(!g.opened)
            {
                ++neverOpened;
                continue;
            }
            if(probeEnd > static_cast<UInt64>(cut))
            {
                ++overran;
            }
            if(g.recs.size() != expect)
            {
                ++wrongCount;
            }
            for(Size i = 0; i < g.recs.size() && i < six.size(); ++i)
            {
                if(!sameBytes(g.recs[i], six[i]))
                {
                    ++wrongBytes;
                }
            }
            // Only the scan path claims the whole file; the index path stops at
            // the end of the frame region by design.
            if(g.beganAs == Source::SOURCE_SCAN && g.endedAs == Source::SOURCE_SCAN)
            {
                if(!accountsForEveryByte(g, cut))
                {
                    ++unaccounted;
                }
            }
        }
        check(
            neverOpened == 0u,
            "every cut of the file still OPENS - none reports a failure to read"
        );
        check(
            wrongCount == 0u,
            "every cut recovers exactly the frames wholly present, and invents none"
        );
        check(wrongBytes == 0u, "and every frame it does recover is byte-identical");
        check(overran == 0u, "and no cut is ever read past its own end");
        check(
            unaccounted == 0u,
            "and on every cut the frames, the junk and the tail add up to the file"
        );

        // The two ends of the sweep, called out by name.
        const Got whole = readSpan(file.data(), file.size());
        check(
            whole.beganAs == Source::SOURCE_INDEX,
            "at full length the footer is intact and the index is used"
        );
        check(whole.recs.size() == 6u, "and all six come back");
        const Got minusOne = readSpan(file.data(), file.size() - 1u);
        check(
            minusOne.beganAs == Source::SOURCE_SCAN,
            "one byte short, the footer is gone and the scan takes over"
        );
        check(minusOne.recs.size() == 6u, "and all six STILL come back");
        const Got midFrame = readSpan(file.data(), endsAt[2] + 4u);
        check(midFrame.recs.size() == 3u, "cut four bytes into frame four, exactly three survive");
        check(
            midFrame.tail == 4u,
            "and the four bytes of the incomplete one are reported as a tail"
        );
    }

    // ---- 9. monotonic to wall clock ------------------------------------------
    {
        // A run whose first clock-carrying frame is a known tMonoUs, so the
        // mapping is arithmetic this test can predict exactly.
        Vec<Made> run;
        Vec<UInt8> body(4096, 0);
        const UInt64 base = 5000000ull;
        for(Size k = 0; k < 5u; ++k)
        {
            CtlState m;
            m.tMonoUs = base + k * 50000ull;   // 50 ms apart, the control period
            m.ackSeq = static_cast<UInt32>(k);
            const Size n = writeCtlState(m, body.data(), body.size());
            run.push_back(makeFrame(Type::TYPE_CTLSTATE, static_cast<UInt16>(k), body.data(), n));
        }
        Vec<UInt8> file;
        check(writeAll(&file, run, true), "a five-tick run recorded 50 ms apart");
        Footer f;
        check(footerAt(file, &f), "its footer parses");
        check(f.firstMonoUs == base, "firstMonoUs is the first frame's own tMonoUs");
        check(f.wallEpochUs == EPOCH_US, "paired with the wall clock the recorder was given");

        check(wallUsOf(f, base) == EPOCH_US, "the pairing instant maps to the epoch exactly");
        check(
            wallUsOf(f, base + 50000ull) == EPOCH_US + 50000ull,
            "one tick later is one tick later"
        );
        check(
            wallUsOf(f, base + 200000ull) == EPOCH_US + 200000ull,
            "and the last tick lands 200 ms on"
        );

        // Every recorded frame's own clock, through the mapping.
        const Got g = readSpan(file.data(), file.size());
        check(g.recs.size() == 5u, "all five ticks come back");
        Bool timesOk = g.recs.size() == 5u;
        for(Size i = 0; i < g.recs.size(); ++i)
        {
            UInt64 mono = 0;
            if(!monoOf(
                g.recs[i].head.type,
                Body{ g.recs[i].body.data(), g.recs[i].body.size() },
                &mono
            ))
            {
                timesOk = false;
                continue;
            }
            if(wallUsOf(f, mono) != EPOCH_US + static_cast<UInt64>(i) * 50000ull)
            {
                timesOk = false;
            }
        }
        check(timesOk, "and every one recovers its real time of day from the file alone");

        // Before the pairing instant, and the clamp at the bottom.
        check(
            wallUsOf(f, base - 1000ull) == EPOCH_US - 1000ull,
            "a frame stamped BEFORE the pairing maps backwards"
        );
        Footer small;
        small.wallEpochUs = 100;
        small.firstMonoUs = 5000;
        check(
            wallUsOf(small, 0) == 0u,
            "and a wild backwards stamp clamps at zero rather than wrapping"
        );

        // Which types carry a clock at all, and where.
        UInt64 mono = 0;
        const Record* wel = findType(readSpan(file.data(), file.size()).recs, Type::TYPE_CTLSTATE);
        check(wel != nullptr, "CTLSTATE is in the run");
        Vec<UInt8> hb(4096, 0);
        Control c;
        c.sessionId = 1;
        c.seq = 1;
        c.tMonoUs = 987654321ull;
        const Size cn = writeControl(c, hb.data(), hb.size());
        check(monoOf(Type::TYPE_CONTROL, Body{ hb.data(), cn }, &mono), "CONTROL carries a clock");
        check(mono == 987654321ull, "at offset 8, the viewer's own, not its session id");
        Hello h;
        h.name = "x";
        const Size hn = writeHello(h, hb.data(), hb.size());
        check(
            !monoOf(Type::TYPE_HELLO, Body{ hb.data(), hn }, &mono),
            "HELLO carries none, and no zero is invented for it"
        );
        Describe d;
        const Size dn = writeDescribe(d, hb.data(), hb.size());
        check(!monoOf(Type::TYPE_DESCRIBE, Body{ hb.data(), dn }, &mono), "nor does DESCRIBE");
    }

    // ---- 10. what the writer refuses -----------------------------------------
    {
        Vec<UInt8> out;
        Writer w;
        Sink broken;
        check(!w.begin(broken, EPOCH_US), "a sink with no write and no flush is refused up front");
        check(w.begin(intoMemory(&out), EPOCH_US), "a real sink is accepted");

        const Made good = all[3];
        check(!w.put(nullptr, 40u), "a null frame is refused");
        check(!w.put(good.bytes.data(), 4u), "something shorter than a frame header is refused");
        check(
            !w.put(good.bytes.data(), good.bytes.size() - 1u),
            "a frame short by one byte is refused"
        );

        Vec<UInt8> bent = good.bytes;
        bent[6] ^= 0xFFu;   // break the CRC by changing the header's seq
        check(
            !w.put(bent.data(), bent.size()),
            "a frame whose CRC does not verify is refused, not appended"
        );

        Vec<UInt8> two = good.bytes;
        two.insert(two.end(), good.bytes.begin(), good.bytes.end());
        check(
            !w.put(two.data(), two.size()),
            "TWO frames in one call are refused - put() records exactly one"
        );

        check(w.put(good.bytes.data(), good.bytes.size()), "and a whole valid frame is accepted");
        check(out.size() == good.bytes.size(), "and appended verbatim, nothing added");
        check(std::memcmp(out.data(), good.bytes.data(), out.size()) == 0, "byte for byte");
        check(w.finish(), "and the run closes");
        check(!w.finish(), "a second finish is refused rather than writing a second footer");
    }

    // ---- 11. junk between frames --------------------------------------------
    //
    // Not a shape the writer makes, but a shape a half-written card produces.
    // The frames on either side must still be found, and the junk COUNTED.
    {
        Vec<UInt8> f;
        f.insert(f.end(), all[3].bytes.begin(), all[3].bytes.end());
        for(Size i = 0; i < 37u; ++i)
        {
            f.push_back(static_cast<UInt8>(0x5Au + i));
        }
        f.insert(f.end(), all[7].bytes.begin(), all[7].bytes.end());
        f.push_back(MAGIC_LO);
        f.push_back(MAGIC_HI);
        f.push_back(0x00u);
        f.insert(f.end(), all[11].bytes.begin(), all[11].bytes.end());

        const Got g = readSpan(f.data(), f.size());
        check(g.beganAs == Source::SOURCE_SCAN, "a file with no footer scans");
        check(g.recs.size() == 3u, "and finds all three frames around the junk");
        check(g.junk == 40u, "and counts the junk exactly - 37 bytes plus a false magic of 3");
        check(accountsForEveryByte(g, f.size()), "and every byte of the file is accounted for");
        check(
            g.recs.size() == 3u && g.recs[0].head.type == Type::TYPE_PING,
            "the first frame is the PING"
        );
        check(
            g.recs.size() == 3u && g.recs[2].head.type == Type::TYPE_CTLSTATE,
            "and the last is the CTLSTATE"
        );
    }

    // ---- 12. the fuzz loop ---------------------------------------------------
    {
        Rng rng;
        Reader r;
        Got g;
        Size overran = 0;
        Size unaccounted = 0;
        Size bogus = 0;
        Size cases = 0;
        UInt64 found = 0;
        Vec<UInt8> buf;
        for(Size iter = 0; iter < 4000u; ++iter)
        {
            const Size len = static_cast<Size>(rng.next() % 3000u);
            buf.assign(len, 0);
            for(Size i = 0; i < len; ++i)
            {
                buf[i] = static_cast<UInt8>(rng.next() & 0xFFu);
            }
            probeEnd = 0;
            collect(&r, probeOver(buf.data(), len), &g);
            ++cases;
            if(probeEnd > static_cast<UInt64>(len))
            {
                ++overran;
            }
            if(!accountsForEveryByte(g, len))
            {
                ++unaccounted;
            }
            for(const Record& rec : g.recs)
            {
                found += 1u;
                if(rec.offset + rec.frameLen > static_cast<UInt64>(len))
                {
                    ++bogus;
                }
                if(rec.body.size() + FRAME_OVERHEAD != rec.frameLen)
                {
                    ++bogus;
                }
            }
        }
        check(cases == 4000u, "4,000 pseudo-random files went through the reader");
        check(overran == 0u, "and not one read a byte past the end of its buffer");
        check(unaccounted == 0u, "and every one accounted for every byte it was given");
        check(bogus == 0u, "and any frame it claimed to find lay wholly inside the file");
        std::printf(
            "        (the fuzz found %llu accidental frame(s) in noise)\n",
            static_cast<unsigned long long>(found)
        );

        // Noise wrapped around a real archive: the frames must still come out.
        Vec<UInt8> real;
        Vec<Made> two;
        two.push_back(all[3]);
        two.push_back(all[7]);
        check(writeAll(&real, two, false), "two frames, abandoned");
        Vec<UInt8> messy;
        for(Size i = 0; i < 200u; ++i)
        {
            messy.push_back(static_cast<UInt8>(rng.next() & 0xFFu));
        }
        messy.insert(messy.end(), real.begin(), real.end());
        for(Size i = 0; i < 200u; ++i)
        {
            messy.push_back(static_cast<UInt8>(rng.next() & 0xFFu));
        }
        const Got gm = readSpan(messy.data(), messy.size());
        check(
            gm.recs.size() >= 2u,
            "buried in 400 bytes of noise, both real frames are still found"
        );
        check(accountsForEveryByte(gm, messy.size()), "and the noise around them is all counted");
    }

    // ---- 13. the same thing, through a real file ------------------------------
    //
    // Everything above runs against memory so the suite stays pure. This one
    // section walks the file-backed Sink and Bytes the board will actually use,
    // because a path that is never executed is a path that is not tested - and
    // it uses the OS temp directory, never the repository.
    {
        const Str path = tempPath();
        Vec<Made> few;
        few.push_back(all[6]);    // the 500-point SCAN
        few.push_back(all[13]);   // the CAMERA
        few.push_back(all[11]);   // a CTLSTATE

        File fw;
        Bool wrote = false;
        if(openWrite(path, &fw))
        {
            Writer w;
            wrote = w.begin(sinkOf(&fw), EPOCH_US);
            for(const Made& m : few)
            {
                if(wrote && !w.put(m.bytes.data(), m.bytes.size()))
                {
                    wrote = false;
                }
            }
            if(wrote)
            {
                wrote = w.finish();
            }
            closeFile(&fw);
        }
        check(wrote, "a three-frame archive writes to a real file through the file sink");

        File fr;
        Bool opened = false;
        Vec<Record> back;
        Source how = Source::SOURCE_NONE;
        if(wrote && openRead(path, &fr))
        {
            opened = true;
            Reader r;
            if(r.begin(bytesOf(&fr)))
            {
                how = r.source;
                Record rec;
                while(r.next(&rec))
                {
                    back.push_back(rec);
                }
            }
            closeFile(&fr);
        }
        check(opened, "and reads back through the file source");
        check(how == Source::SOURCE_INDEX, "by its index");
        check(back.size() == few.size(), "with every frame present");
        Bool fileOk = back.size() == few.size();
        for(Size i = 0; i < back.size() && i < few.size(); ++i)
        {
            if(!sameBytes(back[i], few[i]))
            {
                fileOk = false;
            }
        }
        check(fileOk, "and byte-identical to what went in, off a real filesystem");
        std::remove(path.c_str());
    }

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
