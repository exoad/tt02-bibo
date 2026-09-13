// carrules: what the Car decides, with no lidar, Pico or clock.
//
// Not proved here: car.cxx's threads, ports and signal handlers, and that the
// Pico's firmware still prints the keys fold() reads.
#include "shared.hxx"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

#include "carrules.hxx"

using carrules::Arm;
using carrules::End;
using carrules::exitCode;
using carrules::forwardPulse;
using carrules::steerLine;

static Int32 checks = 0;
static Int32 failures = 0;

constexpr Float32 RAD_TO_DEG = 180.0f / 3.14159265358979323846f;
constexpr Float32 NAN_F = std::numeric_limits<Float32>::quiet_NaN();
constexpr Float32 INF_F = std::numeric_limits<Float32>::infinity();
constexpr Int32 NEUTRAL_US = bibowire::ESC_NEUTRAL_US;
constexpr Float32 FAR_M = bibo::SCAN_FAR_M;

static Void check(Bool ok, const Char* what)
{
    ++checks;
    if(!ok)
    {
        ++failures;
    }
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what);
}

static Void checkStr(const Str& got, const Char* want, const Char* what)
{
    const Bool ok = got == want;
    check(ok, what);
    if(!ok)
    {
        std::printf("        got \"%s\", wanted \"%s\"\n", got.c_str(), want);
    }
}

static Void checkNear(Float32 got, Float32 want, const Char* what)
{
    const Bool ok = std::fabs(got - want) < 0.001f;
    check(ok, what);
    if(!ok)
    {
        std::printf("        got %.4f, wanted %.4f\n", got, want);
    }
}

static Void checkInt(Int64 got, Int64 want, const Char* what)
{
    const Bool ok = got == want;
    check(ok, what);
    if(!ok)
    {
        const Str g = std::to_string(got);
        const Str w = std::to_string(want);
        std::printf("        got %s, wanted %s\n", g.c_str(), w.c_str());
    }
}

static Str joined(const Vec<Str>& lines)
{
    Str out;
    for(const Str& l : lines)
    {
        out += out.empty() ? "" : " | ";
        out += l;
    }
    return out;
}

static bibo::Point at(Float32 bearingDeg, Float32 distanceM)
{
    bibo::Point p;
    p.bearingDeg = bearingDeg;
    p.distanceM = distanceM;
    return p;
}

// A return forwardM ahead of the lidar's axis and sideM to its right.
static bibo::Point xy(Float32 forwardM, Float32 sideM)
{
    return at(std::atan2(sideM, forwardM) * RAD_TO_DEG, std::hypot(forwardM, sideM));
}

static Vec<bibo::Point> repeat(bibo::Point p, Int32 n)
{
    return Vec<bibo::Point>(static_cast<Size>(n), p);
}

// Ten returns across 0.18 m, centred sideM right of the axis, all forwardM ahead.
static Vec<bibo::Point> wall(Float32 forwardM, Float32 sideM)
{
    Vec<bibo::Point> out;
    for(Int32 i = 0; i < 10; ++i)
    {
        out.push_back(xy(forwardM, sideM - 0.09f + 0.02f * static_cast<Float32>(i)));
    }
    return out;
}

static Vec<bibo::Point> plus(Vec<bibo::Point> a, const Vec<bibo::Point>& b)
{
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

// Revolution 1, topped up to SCAN_MIN_POINTS with returns straight behind the car,
// where neither ahead() nor any nearest() range used below looks.
static bibo::Scan scanOf(Vec<bibo::Point> points)
{
    while(points.size() < static_cast<Size>(bibo::SCAN_MIN_POINTS))
    {
        points.push_back(at(180.0f, 5.0f));
    }
    bibo::Scan s;
    s.revolution = 1;
    s.points = points;
    return s;
}

static Void testAhead()
{
    std::printf("\n-- Scan::ahead --\n");
    const Int32 hits = bibo::SCAN_MIN_HITS;
    const Float32 arc = bibo::SCAN_FRONT_ARC_DEG;
    checkNear(scanOf(wall(0.5f, 0.0f)).ahead(), 0.5f, "a wall 0.50 m dead ahead reads 0.50");
    checkNear(scanOf(wall(0.5f, 0.3f)).ahead(), FAR_M, "the same wall 0.30 m right is clear");
    checkNear(scanOf(wall(0.5f, -0.3f)).ahead(), FAR_M, "and 0.30 m left");
    checkNear(scanOf(wall(0.5f, 0.3f)).ahead(0.4f), 0.5f, "a wider strip sees it");
    const bibo::Scan right15 = scanOf(repeat(xy(0.8f, 0.15f), hits));
    const bibo::Scan left15 = scanOf(repeat(xy(0.8f, -0.15f), hits));
    const bibo::Scan right17 = scanOf(repeat(xy(0.8f, 0.17f), hits));
    checkNear(right15.ahead(), 0.8f, "0.15 m right is in the CAR_HALF_WIDTH_M strip");
    checkNear(left15.ahead(), 0.8f, "and 0.15 m left");
    checkNear(right17.ahead(), FAR_M, "0.17 m right is not");
    const Vec<bibo::Point> specks = repeat(xy(0.2f, 0.0f), hits - 1);
    const bibo::Scan speckWall = scanOf(plus(specks, wall(1.0f, 0.0f)));
    checkNear(speckWall.ahead(), 1.0f, "specks short of SCAN_MIN_HITS before a wall read the wall");
    checkNear(scanOf(specks).ahead(), FAR_M, "and alone read clear");
    checkNear(scanOf(repeat(xy(0.2f, 0.0f), hits)).ahead(), 0.2f, "but SCAN_MIN_HITS are one");
    const bibo::Scan beside = scanOf(repeat(at(arc + 5.0f, 0.1f), hits));
    const bibo::Scan inside = scanOf(repeat(at(arc - 5.0f, 0.1f), hits));
    const Float32 along = 0.1f * std::cos((arc - 5.0f) / RAD_TO_DEG);
    checkNear(beside.ahead(), FAR_M, "beside the lidar, past SCAN_FRONT_ARC_DEG, is ignored");
    checkNear(inside.ahead(), along, "inside the arc it counts, along the axis");
    checkNear(scanOf(repeat(at(180.0f, 0.3f), hits)).ahead(), FAR_M, "behind the car is ignored");
    checkNear(scanOf(repeat(at(-170.0f, 0.3f), hits)).ahead(), FAR_M, "and beside behind");
    checkNear(scanOf(repeat(xy(20.0f, 0.0f), hits)).ahead(), FAR_M, "never beyond SCAN_FAR_M");
    const bibo::Scan wall50 = scanOf(wall(0.5f, 0.0f));
    checkNear(wall50.ahead(-0.1f), 0.0f, "a negative half width reads 0, not clear");
    checkNear(wall50.ahead(NAN_F), 0.0f, "and so does a NaN one");
}

static Void testNearestAndBlind()
{
    std::printf("\n-- Scan::nearest and blind scans --\n");
    const Int32 hits = bibo::SCAN_MIN_HITS;
    const Vec<bibo::Point> left = repeat(at(-45.0f, 0.7f), hits);
    const bibo::Scan sides = scanOf(plus(left, repeat(at(45.0f, 0.3f), hits)));
    checkNear(sides.nearest(-80.0f, -15.0f), 0.7f, "nearest(-80, -15) sees only the left");
    checkNear(sides.nearest(15.0f, 80.0f), 0.3f, "nearest(15, 80) only the right");
    checkNear(sides.nearest(-10.0f, 10.0f), FAR_M, "an empty range reads SCAN_FAR_M");
    checkNear(sides.nearest(-15.0f, -80.0f), 0.0f, "fromDeg > toDeg reads 0");
    checkNear(sides.nearest(NAN_F, 10.0f), 0.0f, "and so does a NaN bound");
    const bibo::Scan edge = scanOf(repeat(at(-15.0f, 0.4f), hits));
    checkNear(edge.nearest(-80.0f, -15.0f), 0.4f, "both bounds are included");
    const Vec<bibo::Point> specks = repeat(at(30.0f, 0.2f), hits - 1);
    const bibo::Scan speckSide = scanOf(plus(specks, repeat(at(30.0f, 0.9f), hits)));
    checkNear(speckSide.nearest(15.0f, 80.0f), 0.9f, "nearest takes the SCAN_MIN_HITS-th too");
    bibo::Scan few;
    few.revolution = 1;
    few.points = repeat(at(0.0f, 0.5f), bibo::SCAN_MIN_POINTS - 1);
    check(few.blind(), "one point short of SCAN_MIN_POINTS is blind");
    checkNear(few.ahead(), 0.0f, "a blind scan reads 0 ahead");
    checkNear(few.nearest(-180.0f, 180.0f), 0.0f, "and 0 nearest");
    bibo::Scan enough = few;
    enough.points.push_back(at(0.0f, 0.5f));
    check(!enough.blind(), "SCAN_MIN_POINTS points are not blind");
    enough.revolution = 0;
    check(enough.blind(), "revolution 0 is blind whatever it holds");
    const bibo::Scan empty;
    check(empty.blind() && empty.ahead() == 0.0f, "an empty Scan is blind and reads 0");
}

// The bearing toScan gives one ray at rawDeg, or NaN when it dropped the ray.
static Float32 bearing(Float32 rawDeg, Float32 forwardDeg)
{
    const bibo::Scan s = carrules::toScan({reactive::Ray{rawDeg, 1000.0f}}, forwardDeg, 1);
    return s.points.empty() ? NAN_F : s.points[0].bearingDeg;
}

static Void testToScan()
{
    std::printf("\n-- toScan --\n");
    const Vec<reactive::Ray> rays = {
        reactive::Ray{0.0f, 500.0f},
        reactive::Ray{90.0f, 0.0f},
        reactive::Ray{90.0f, -5.0f},
        reactive::Ray{270.0f, 1000.0f},
        reactive::Ray{NAN_F, 800.0f},
        reactive::Ray{10.0f, INF_F},
    };
    const bibo::Scan s = carrules::toScan(rays, 0.0f, 7);
    checkInt(static_cast<Int64>(s.points.size()), 2, "no return or not finite: dropped");
    checkInt(s.revolution, 7, "the revolution number is kept");
    if(s.points.size() == 2)
    {
        checkNear(s.points[0].distanceM, 0.5f, "500 mm is 0.50 m");
        checkNear(s.points[1].bearingDeg, -90.0f, "raw 270 is 90 degrees left");
    }
    checkNear(bearing(90.0f, 0.0f), 90.0f, "raw 90, clockwise from above, is right: positive");
    checkNear(bearing(350.0f, 10.0f), -20.0f, "forward 10: raw 350 is 20 left");
    checkNear(bearing(20.0f, 10.0f), 10.0f, "forward 10: raw 20 is 10 right");
    checkNear(bearing(10.0f, 350.0f), 20.0f, "forward 350: raw 10 wraps to 20 right");
    checkNear(bearing(340.0f, 350.0f), -10.0f, "forward 350: raw 340 is 10 left");
    checkNear(bearing(190.0f, 0.0f), -170.0f, "raw 190 folds to -170");
    checkNear(std::fabs(bearing(180.0f, 0.0f)), 180.0f, "raw 180 is straight behind");
    checkNear(std::fabs(bearing(0.0f, 180.0f)), 180.0f, "and so is raw 0 with forward 180");
    checkNear(bearing(270.0f, -90.0f), 0.0f, "forward -90 is forward 270");
    checkNear(bearing(360.0f, 0.0f), 0.0f, "raw 360 is raw 0");
    check(carrules::toScan(rays, NAN_F, 1).points.empty(), "a NaN forward angle keeps nothing");
}

struct Parsed
{
    Bool ok = false;
    carrules::Options opt;
    Str why;
};

static Parsed parse(Bool measured, std::initializer_list<const Char*> args)
{
    Vec<Str> words = {"forward"};
    words.insert(words.end(), args.begin(), args.end());
    Vec<Char*> argv;
    for(Str& w : words)
    {
        argv.push_back(w.data());
    }
    Parsed p;
    const Int32 argc = static_cast<Int32>(argv.size());
    p.ok = carrules::parseArgs(argc, argv.data(), measured, p.opt, p.why);
    return p;
}

static Void testFlags()
{
    std::printf("\n-- parseArgs --\n");
    Parsed p = parse(false, {});
    check(p.ok && !p.opt.drive && !p.opt.help, "no flags is a dry run");
    check(p.opt.viewer && p.opt.seconds == 0, "with the viewer and no time limit");
    check(p.opt.forwardDeg == bibo::LIDAR_FORWARD_DEG, "at the committed forward angle");
    check(p.opt.lidarPort == bibo::LIDAR_PORT, "on the default lidar port");
    check(p.opt.picoPort == bibo::PICO_PORT, "and the default Pico port");
    p = parse(false, {"--drive"});
    check(!p.ok, "--drive, unmeasured, with no --forward is refused");
    check(p.why.find("LIDAR_FORWARD_MEASURED") != Str::npos, "and says what to set");
    p = parse(true, {"--drive"});
    check(p.ok && p.opt.drive, "--drive with the angle measured drives");
    p = parse(false, {"--drive", "--forward", "12.5"});
    check(p.ok && p.opt.drive, "--drive with --forward drives unmeasured");
    checkNear(p.opt.forwardDeg, 12.5f, "at the angle given");
    p = parse(false, {"--forward", "-5"});
    check(p.ok, "a negative --forward is a number, not a flag");
    checkNear(p.opt.forwardDeg, -5.0f, "and is kept");
    p = parse(false, {"--seconds", "5", "--lidar", "/dev/ttyUSB1", "--pico", "/dev/ttyACM1"});
    check(p.ok && p.opt.seconds == 5, "--seconds parses");
    check(p.opt.lidarPort == "/dev/ttyUSB1", "--lidar sets the lidar port");
    check(p.opt.picoPort == "/dev/ttyACM1", "--pico sets the Pico port");
    p = parse(false, {"--no-viewer"});
    check(p.ok && !p.opt.viewer, "--no-viewer turns the viewer off");
    p = parse(false, {"--help", "--drive"});
    check(p.ok && p.opt.help, "--help is accepted, even beside an unmeasured --drive");
    check(!parse(false, {"--forward", "12x"}).ok, "--forward 12x is refused");
    check(!parse(false, {"--forward", "nan"}).ok, "--forward nan is refused");
    check(!parse(false, {"--forward", "400"}).ok, "--forward past a full turn is refused");
    check(!parse(false, {"--forward"}).ok, "--forward with no value is refused");
    check(!parse(false, {"--seconds", "0"}).ok, "--seconds 0 is refused");
    check(!parse(false, {"--seconds", "-1"}).ok, "--seconds -1 is refused");
    check(!parse(false, {"--seconds", "2.5"}).ok, "--seconds 2.5 is refused");
    check(!parse(false, {"--seconds", "1000000"}).ok, "--seconds past a day is refused");
    check(!parse(false, {"--seconds"}).ok, "--seconds with no value is refused");
    check(!parse(false, {"--lidar"}).ok, "--lidar with no value is refused");
    check(!parse(false, {"--pico", "--drive"}).ok, "--pico followed by a flag is refused");
    check(!parse(true, {"--dry"}).ok, "--dry is refused: no flag at all is the dry run");
    check(!parse(true, {"--amr"}).ok, "a typo is refused");
    check(!parse(true, {"drive"}).ok, "a bare word is refused");
    check(parse(true, {"--amr"}).why.find("--amr") != Str::npos, "and the reason names it");
    const Array<const Char*, 7> flags = {
        "--drive", "--seconds", "--forward", "--lidar", "--pico", "--no-viewer", "--help",
    };
    const Str u = carrules::usage();
    Bool all = true;
    for(const Char* flag : flags)
    {
        all = all && u.find(flag) != Str::npos;
    }
    check(all, "usage() lists every flag");
}

// An OK drive line as firmware/app/main.cxx printDrive prints it.
static Str driveLine(Int32 armed, Int32 servoOn, Int32 stale, Int32 escMaxUs = 1700)
{
    Array<Char, 320> buf = {};
    std::snprintf(
        buf.data(),
        buf.size(),
        "OK drive servo=1500 servo_t=1500 esc=1510 esc_t=1520 armed=%d servo_on=%d "
        "servo_c=1500 steer_m=0 steer_now=0 slew=8 slew_esc=8 servo_min=1000 "
        "servo_max=2000 esc_min=1564 esc_max=%d esc_rev=1400 stale=%d",
        armed,
        servoOn,
        escMaxUs,
        stale
    );
    return Str(buf.data());
}

static Void testFold()
{
    std::printf("\n-- fold --\n");
    carrules::Board b;
    check(carrules::fold(b, driveLine(1, 1, 0)), "an OK drive line is a report");
    check(b.armed == 1 && b.servoOn == 1 && b.stale == 0, "armed=, servo_on=, stale= read");
    checkInt(b.escUs, 1510, "esc= is read, not esc_t=");
    checkInt(b.escMinUs, 1564, "esc_min=");
    checkInt(b.escMaxUs, 1700, "esc_max=");
    checkInt(b.escRevUs, 1400, "esc_rev=");
    check(!b.stopAnswered, "a drive line is not an OK stop");
    check(carrules::fold(b, "OK drive armed=1 servo_on=1"), "a short drive line is a report too");
    checkInt(b.escMaxUs, -1, "and a key it lacks reads -1, not the last value");
    carrules::Board c;
    const Str sparse = "OK drive esc_min=1564 esc_max=1700 servo=1600 servo_t=1600";
    check(carrules::fold(c, sparse), "a sparse drive line is a report");
    checkInt(c.escUs, -1, "esc= is not read out of esc_min= or esc_max=");
    checkInt(c.servoOn, -1, "servo_on= is not read out of servo= or servo_t=");
    check(carrules::fold(c, "OK drive armed=yes"), "a drive line with a bad value is a report");
    checkInt(c.armed, -1, "whose bad value reads -1");
    carrules::Board d;
    check(!carrules::fold(d, "OK stop"), "OK stop is not a drive report");
    check(d.stopAnswered, "but it is remembered");
    check(!carrules::fold(d, "ERR esc not armed - send ESC ARM first"), "ERR is not a report");
    checkInt(d.errors, 1, "but it is counted");
    check(!carrules::fold(d, "PONG"), "PONG is not a report");
    check(!carrules::fold(d, "INFO esc armed - neutral held"), "nor is INFO");
    check(!carrules::fold(d, "") && !carrules::fold(d, "bibo"), "nor a blank or banner line");
    check(d.armed == -1 && d.errors == 1, "and none of those changed anything else");
}

static Void testPulseAndSteer()
{
    std::printf("\n-- forwardPulse and steerLine --\n");
    checkInt(forwardPulse(0.5f, 1564, 1700), 1632, "(0.5, 1564, 1700) is 1632");
    checkInt(forwardPulse(1.0f, 1564, 1700), 1700, "1 is the top of the band");
    checkInt(forwardPulse(0.001f, 1564, 1700), 1564, "a whisper is the bottom of the band");
    checkInt(forwardPulse(0.0f, 1564, 1700), NEUTRAL_US, "0 is neutral");
    checkInt(forwardPulse(-0.25f, 1564, 1700), NEUTRAL_US, "below 0 is neutral: no reverse");
    checkInt(forwardPulse(NAN_F, 1564, 1700), NEUTRAL_US, "NaN is neutral");
    checkInt(forwardPulse(2.0f, 1564, 1700), 1700, "above 1 clamps to the top");
    checkInt(forwardPulse(INF_F, 1564, 1700), 1700, "and so does infinity");
    checkInt(forwardPulse(0.5f, 1450, 1700), 1600, "an esc_min below neutral is lifted to it");
    checkInt(forwardPulse(0.5f, -1, -1), NEUTRAL_US, "unknown limits are neutral");
    checkInt(forwardPulse(0.5f, 1564, 1564), NEUTRAL_US, "an empty band is neutral");
    checkInt(forwardPulse(0.5f, 1450, 1500), NEUTRAL_US, "a band with no forward part: neutral");
    checkStr(steerLine(0.25f), "STEER 0.250", "steer 0.25");
    checkStr(steerLine(-0.25f), "STEER -0.250", "negative is left");
    checkStr(steerLine(1.5f), "STEER 1.000", "clamped to 1");
    checkStr(steerLine(-7.0f), "STEER -1.000", "clamped to -1");
    checkStr(steerLine(NAN_F), "STEER 0.000", "NaN is centre");
}

static Void testEnds()
{
    std::printf("\n-- exitCode and endName --\n");
    checkInt(exitCode(End::END_SIGNAL, 5), 0, "Ctrl-C after revolutions is 0");
    checkInt(exitCode(End::END_ESTOP, 5), 0, "a viewer's ESTOP is 0");
    checkInt(exitCode(End::END_SECONDS, 5), 0, "--seconds is 0");
    checkInt(exitCode(End::END_FINISHED, 5), 0, "the program's own finish() is 0");
    checkInt(exitCode(End::END_SIGNAL, 0), 1, "but 1 when nothing was measured");
    checkInt(exitCode(End::END_FINISHED, 0), 1, "for every requested end");
    checkInt(exitCode(End::END_HELP, 0), 0, "--help is 0");
    checkInt(exitCode(End::END_BAD_FLAGS, 0), 2, "bad flags are 2");
    checkInt(exitCode(End::END_LIDAR_LOST, 5), 1, "a lost lidar is 1");
    checkInt(exitCode(End::END_SEND_GAP, 5), 1, "a send gap is 1");
    checkInt(exitCode(End::END_VIEWER_STUCK, 5), 1, "a stuck viewer feed is 1");
    checkInt(exitCode(End::END_ARM_TIMEOUT, 5), 1, "an arm timeout is 1");
    checkInt(exitCode(End::END_NONE, 5), 1, "no reason at all is 1");
    Set<Str> names;
    const Int32 last = static_cast<Int32>(End::END_VIEWER_STUCK);
    for(Int32 e = 0; e <= last; ++e)
    {
        names.insert(carrules::endName(static_cast<End>(e)));
    }
    check(names.size() == static_cast<Size>(last + 1), "every End has its own name");
    check(names.count("") == 0, "and none is empty");
}

// A minder passing every MINDER_MS over a healthy car unless a test says otherwise:
// a revolution and a drive() arrive at each pass, and the port accepts every line.
struct Bench
{
    carrules::Governor gov;
    carrules::Inputs in;
    Bool freshScan = true;
    Bool freshDrive = true;

    // The clock starts well past 0, so a time a test backdates is never negative:
    // the Governor reads a negative time as "never".
    explicit Bench(Bool dry = false, Int32 seconds = 0)
        : gov(dry, seconds)
    {
        in.nowMs = 60000;
    }

    Void open()
    {
        in.sentMs = gov.open({}).empty() ? in.sentMs : in.nowMs;
    }

    Vec<Str> tick(const Vec<Str>& replies = {}, Int64 stepMs = bibo::MINDER_MS)
    {
        in.nowMs += stepMs;
        in.scanMs = freshScan ? in.nowMs : in.scanMs;
        in.driveMs = freshDrive ? in.nowMs : in.driveMs;
        in.replies = replies;
        const Vec<Str> out = gov.pass(in);
        in.sentMs = out.empty() ? in.sentMs : in.nowMs;
        return out;
    }
};

static const Str IDLE = driveLine(0, 0, 0);
static const Str NO_STEER = driveLine(1, 0, 0);
static const Str ARMED = driveLine(1, 1, 0);
static const Char* PULSE = "STEER 0.000 | ESC 1632";
static const Char* HELD = "STEER 0.000 | ESC NEUTRAL";

// Opens b and arms it with throttle 0.5 and steer 0; returns the confirming pass.
static Vec<Str> armUp(Bench& b)
{
    b.open();
    b.tick({"OK stop", "PONG"});
    b.gov.requestArm();
    b.tick({IDLE});
    b.in.throttle = 0.5f;
    return b.tick({NO_STEER, ARMED});
}

static Bench armedBench(Int32 seconds = 0)
{
    Bench b(false, seconds);
    armUp(b);
    return b;
}

// Passes while now - since stays within limitMs after the pass; true when every
// one of them left the run going.
static Bool runsThrough(Bench& b, Int64 since, Int64 limitMs, const Vec<Str>& replies, Str& last)
{
    Bool running = true;
    while(b.in.nowMs + bibo::MINDER_MS - since <= limitMs)
    {
        last = joined(b.tick(replies));
        running = running && b.gov.ended() == End::END_NONE;
    }
    return running;
}

// lines, the pass that ended the run, must be exactly STOP for want. So must every
// later pass, with the cause gone, and a signal after it must not change the reason.
static Void checkLatched(Bench& b, const Vec<Str>& lines, End want, const Char* what)
{
    const Str got = joined(lines);
    const End first = b.gov.ended();
    b.in.estop = false;
    b.in.viewerStuck = false;
    b.in.signal = false;
    b.in.linkLost = false;
    b.in.gapMs = 0;
    Bool stays = true;
    for(Int32 i = 0; i < 3; ++i)
    {
        stays = stays && joined(b.tick({ARMED})) == "STOP";
    }
    b.in.signal = true;
    stays = stays && joined(b.tick({ARMED})) == "STOP" && b.gov.ended() == want;
    check(got == "STOP" && first == want && stays, what);
    if(got != "STOP" || first != want)
    {
        std::printf("        got \"%s\" for %s\n", got.c_str(), carrules::endName(first));
    }
}

static Void testGovernorArming()
{
    std::printf("\n-- Governor: opening and arming --\n");
    carrules::Governor g(false, 0);
    checkStr(
        joined(g.open({"ESCLIMITS 1564 1700", "SERVOTRIM 1480"})),
        "STOP | PING | ESCLIMITS 1564 1700 | SERVOTRIM 1480",
        "open is STOP, PING, then the trim in order"
    );
    Bench b;
    b.open();
    b.in.throttle = 0.5f;
    const Vec<Str> first = b.tick({"OK stop", "PONG", driveLine(0, 0, 1)});
    checkStr(joined(first), "ESC NEUTRAL", "before arm: ESC NEUTRAL only");
    check(b.gov.ended() == End::END_NONE, "stale=1 before arming ends nothing");
    b.gov.requestArm();
    checkStr(joined(b.tick({IDLE})), "ESC ARM | SERVO ON", "the arm edge: no pulse on that pass");
    check(b.gov.armState() == Arm::ARM_PENDING, "then it waits for the Pico");
    checkStr(joined(b.tick({NO_STEER})), "ESC NEUTRAL", "armed, steering off: not confirmed");
    checkStr(joined(b.tick({ARMED})), PULSE, "armed, steering on: STEER, then ESC 1632");
    check(b.gov.armState() == Arm::ARM_CONFIRMED, "confirmed");
    Bench quiet;
    quiet.open();
    quiet.in.gapMs = bibo::SEND_GAP_STOP_MS + 1;
    const Vec<Str> q = quiet.tick({"OK stop"});
    checkStr(joined(q), "ESC NEUTRAL", "a send gap before arming ends nothing");
    check(quiet.gov.ended() == End::END_NONE, "and the run goes on");
    Bench early;
    early.open();
    early.tick();
    early.gov.requestArm();
    early.tick();
    early.tick({ARMED, "OK stop"});
    check(early.gov.armState() == Arm::ARM_PENDING, "armed=1 before OK stop does not confirm");
    early.tick({ARMED});
    check(early.gov.armState() == Arm::ARM_CONFIRMED, "the same report after it does");
    Bench narrow;
    narrow.open();
    narrow.tick({"OK stop"});
    narrow.gov.requestArm();
    narrow.tick();
    narrow.tick({driveLine(1, 1, 0, NEUTRAL_US)});
    check(narrow.gov.armState() == Arm::ARM_PENDING, "esc_max not above neutral: not confirmed");
    Bench slow;
    slow.open();
    slow.tick({"OK stop"});
    slow.gov.requestArm();
    slow.tick();
    Str last;
    const Bool waited = runsThrough(slow, slow.in.nowMs, bibo::ARM_CONFIRM_MS, {NO_STEER}, last);
    check(waited, "an unconfirmed arm waits ARM_CONFIRM_MS");
    checkStr(last, "ESC NEUTRAL", "at neutral");
    checkLatched(slow, slow.tick({NO_STEER}), End::END_ARM_TIMEOUT, "then STOP: END_ARM_TIMEOUT");
}

static Void testGovernorFreshness()
{
    std::printf("\n-- Governor: neutral until fresh --\n");
    Bench b;
    checkStr(joined(armUp(b)), PULSE, "armed and fresh: a pulse");
    b.in.steer = 0.25f;
    b.freshDrive = false;
    b.in.driveMs = b.in.nowMs + bibo::MINDER_MS - bibo::DRIVE_FRESH_MS;
    checkStr(joined(b.tick({ARMED})), "STEER 0.250 | ESC 1632", "DRIVE_FRESH_MS old: drives");
    b.in.driveMs = b.in.nowMs + bibo::MINDER_MS - bibo::DRIVE_FRESH_MS - 1;
    checkStr(joined(b.tick({ARMED})), "STEER 0.250 | ESC NEUTRAL", "older: neutral, steer held");
    b.freshDrive = true;
    b.in.steer = 0.0f;
    checkStr(joined(b.tick({ARMED})), PULSE, "a fresh drive() drives again");
    b.freshScan = false;
    b.in.scanMs = b.in.nowMs + bibo::MINDER_MS - bibo::SCAN_FRESH_MS;
    checkStr(joined(b.tick({ARMED})), PULSE, "a revolution SCAN_FRESH_MS old: drives");
    b.in.scanMs = b.in.nowMs + bibo::MINDER_MS - bibo::SCAN_FRESH_MS - 1;
    checkStr(joined(b.tick({ARMED})), HELD, "older: neutral");
    b.freshScan = true;
    checkStr(joined(b.tick({ARMED})), PULSE, "a fresh revolution drives again");
    Str last;
    check(runsThrough(b, b.in.nowMs, bibo::PICO_QUIET_MS, {}, last), "a quiet Pico ends nothing");
    checkStr(last, PULSE, "and still drives at PICO_QUIET_MS");
    checkStr(joined(b.tick()), HELD, "past it: neutral");
    checkStr(joined(b.tick({ARMED})), PULSE, "it speaks again: a pulse");
    b.in.throttle = 0.0f;
    checkStr(joined(b.tick({ARMED})), HELD, "throttle 0 is neutral");
    b.in.throttle = NAN_F;
    checkStr(joined(b.tick({ARMED})), HELD, "NaN throttle is neutral");
    b.in.throttle = 0.5f;
    const Int64 gap = bibo::SEND_GAP_STOP_MS;
    checkStr(joined(b.tick({ARMED}, 2 * bibo::MINDER_MS)), PULSE, "one missed pass still drives");
    checkStr(joined(b.tick({ARMED}, gap)), PULSE, "a gap of SEND_GAP_STOP_MS still drives");
    b.in.gapMs = gap;
    checkStr(joined(b.tick({ARMED})), PULSE, "and so does one between two sends");
    b.in.gapMs = 0;
    check(b.gov.ended() == End::END_NONE, "and nothing so far ended the run");
}

static Void testGovernorLatches()
{
    std::printf("\n-- Governor: what ends the run, then exactly STOP every pass --\n");
    Bench disarmed = armedBench();
    const Vec<Str> d = disarmed.tick({driveLine(0, 1, 0)});
    checkLatched(disarmed, d, End::END_PICO_DISARMED, "armed=0 after arming: END_PICO_DISARMED");
    Bench watchdog = armedBench();
    const Vec<Str> w = watchdog.tick({driveLine(1, 1, 1)});
    checkLatched(watchdog, w, End::END_PICO_WATCHDOG, "stale=1 after arming: END_PICO_WATCHDOG");
    Bench late;
    late.open();
    late.tick({"OK stop"});
    late.gov.requestArm();
    late.tick();
    const Vec<Str> ld = late.tick({ARMED, driveLine(0, 1, 0)});
    checkLatched(late, ld, End::END_PICO_DISARMED, "armed=0 right behind the confirming line");
    Bench gap = armedBench();
    const Vec<Str> g = gap.tick({ARMED}, bibo::SEND_GAP_STOP_MS + 1);
    checkLatched(gap, g, End::END_SEND_GAP, "a send gap past SEND_GAP_STOP_MS: END_SEND_GAP");
    Bench between = armedBench();
    between.in.gapMs = bibo::SEND_GAP_STOP_MS + 1;
    const Vec<Str> bg = between.tick({ARMED});
    checkLatched(between, bg, End::END_SEND_GAP, "the same gap between two sends: END_SEND_GAP");
    Bench estop = armedBench();
    estop.in.estop = true;
    const Vec<Str> e = estop.tick({ARMED});
    checkLatched(estop, e, End::END_ESTOP, "a viewer's ESTOP: END_ESTOP");
    Bench stuck = armedBench();
    stuck.in.viewerStuck = true;
    const Vec<Str> vs = stuck.tick({ARMED});
    checkLatched(stuck, vs, End::END_VIEWER_STUCK, "the viewer's feed stuck: END_VIEWER_STUCK");
    Bench signal = armedBench();
    signal.in.signal = true;
    const Vec<Str> s = signal.tick({ARMED});
    checkLatched(signal, s, End::END_SIGNAL, "a signal: END_SIGNAL");
    Bench answered = armedBench();
    answered.in.signal = true;
    const Vec<Str> a = answered.tick({"OK stop", IDLE});
    checkLatched(answered, a, End::END_SIGNAL, "a signal whose STOP the Pico answered: END_SIGNAL");
    Bench link = armedBench();
    link.in.linkLost = true;
    const Vec<Str> l = link.tick({ARMED});
    checkLatched(link, l, End::END_LINK_LOST, "the port closing: END_LINK_LOST");
    Bench finished = armedBench();
    finished.gov.end(End::END_FINISHED);
    finished.gov.end(End::END_ESTOP);
    const Vec<Str> f = finished.tick({ARMED});
    checkLatched(finished, f, End::END_FINISHED, "finish(), then another reason: END_FINISHED");
    Bench lost = armedBench();
    lost.freshScan = false;
    Str last;
    const Bool held = runsThrough(lost, lost.in.scanMs, bibo::LIDAR_LOST_MS, {ARMED}, last);
    check(held, "no revolution: the run goes on for LIDAR_LOST_MS");
    checkStr(last, HELD, "at neutral");
    checkLatched(lost, lost.tick({ARMED}), End::END_LIDAR_LOST, "then STOP: END_LIDAR_LOST");
    Bench timed = armedBench(1);
    const Bool drove = runsThrough(timed, timed.in.nowMs, 1000, {ARMED}, last);
    check(drove, "--seconds 1 drives for a second");
    checkStr(last, PULSE, "with pulses");
    checkLatched(timed, timed.tick({ARMED}), End::END_SECONDS, "then STOP: END_SECONDS");
    Bench before;
    before.open();
    before.in.estop = true;
    checkStr(joined(before.tick()), "STOP", "ESTOP before arming sends STOP too");
    before.gov.requestArm();
    checkStr(joined(before.tick()), "STOP", "and a later arm does nothing");
    Bench early;
    early.open();
    early.in.viewerStuck = true;
    checkStr(joined(early.tick()), "STOP", "a stuck viewer feed before arming sends STOP too");
}

static Void testGovernorDry()
{
    std::printf("\n-- Governor: a dry run sends nothing and latches the same --\n");
    Bench d(true, 0);
    check(d.gov.open({"ESCLIMITS 1564 1700"}).empty(), "a dry run opens nothing");
    check(d.tick().empty(), "sends nothing before arm");
    d.gov.requestArm();
    check(d.tick().empty(), "nor on the arm pass");
    check(d.gov.armState() == Arm::ARM_CONFIRMED, "and counts as armed");
    d.in.throttle = 0.5f;
    d.in.linkLost = true;
    Bool quiet = true;
    for(Int32 i = 0; i < 10; ++i)
    {
        quiet = quiet && d.tick().empty();
    }
    check(quiet && d.gov.ended() == End::END_NONE, "no Pico: no link, send gap or arm latch");
    d.in.signal = true;
    check(d.tick().empty() && d.gov.ended() == End::END_SIGNAL, "a signal ends it, silently");
    Bench lost(true, 0);
    lost.gov.requestArm();
    lost.tick();
    lost.freshScan = false;
    Str last;
    const Bool ran = runsThrough(lost, lost.in.scanMs, bibo::LIDAR_LOST_MS, {}, last);
    check(ran, "no revolution: runs LIDAR_LOST_MS");
    check(lost.tick().empty() && lost.gov.ended() == End::END_LIDAR_LOST, "then ends, silently");
    Bench timed(true, 1);
    timed.gov.requestArm();
    timed.tick();
    const Bool timedRan = runsThrough(timed, timed.in.nowMs, 1000, {}, last);
    check(timedRan, "--seconds 1 runs a second");
    check(timed.tick().empty() && timed.gov.ended() == End::END_SECONDS, "then ends, silently");
}

// One write: how long it takes, and what the port answers.
struct Step
{
    Int64 ms = 1;
    carlink::Result result = carlink::Result::RESULT_OK;
};

// A port whose writes take scripted time. A write moves the clock by its step's
// ms, cut off at its wait as carlink::send's is, and answers the step's result;
// past the script a write takes 1 ms and is accepted.
struct FakePort
{
    Int64 nowMs = 60000;
    Vec<Step> script;
    Size next = 0;
    Vec<Str> lines;       // every line handed to the port, accepted or not
    Vec<Int32> waits;     // and the wait each was given

    carrules::Sender sender{
        [this]() { return nowMs; },
        [this](const Str& line, Int32 waitMs) { return write(line, waitMs); }
    };

    carlink::Result write(const Str& line, Int32 waitMs)
    {
        const Step s = next < script.size() ? script[next] : Step();
        ++next;
        lines.push_back(line);
        waits.push_back(waitMs);
        nowMs += std::min<Int64>(s.ms, waitMs);
        return s.ms > waitMs ? carlink::Result::RESULT_WRITE_FAILED : s.result;
    }

    // STOP, accepted, as the Car opens with; the script then starts at its first step.
    Void open(const Vec<Step>& steps)
    {
        static_cast<Void>(sender.send("STOP"));
        script = steps;
        next = 0;
        lines.clear();
        waits.clear();
    }
};

static Void testSender()
{
    std::printf("\n-- Sender: a throttle line is judged as it goes out --\n");
    const Int64 gap = bibo::SEND_GAP_STOP_MS;
    const Int64 wait = std::min<Int64>(gap, carlink::WRITE_WAIT_MS);
    const carlink::Result ok = carlink::Result::RESULT_OK;
    checkInt(carrules::pulseWaitMs(1000, -1), 0, "nothing accepted yet: no time for a pulse");
    checkInt(carrules::pulseWaitMs(1000, 1000), gap, "right after a line: SEND_GAP_STOP_MS");
    checkInt(carrules::pulseWaitMs(1000 + gap, 1000), 0, "SEND_GAP_STOP_MS after it: none");
    checkInt(carrules::pulseIn("ESC 1632"), 1632, "pulseIn reads ESC <us>");
    check(carrules::pulseIn("ESC NEUTRAL") == -1, "and not ESC NEUTRAL");
    check(carrules::pulseIn("ESC ") == -1, "nor a bare ESC");
    check(carrules::pulseIn("ESC 16x") == -1, "nor one with a tail");
    check(carrules::pulseIn("STEER 0.500") == -1, "nor another command");
    const Vec<Str> drive = {"STEER 0.000", "ESC 1632"};
    const Vec<Str> pulse = {"ESC 1632"};
    {
        FakePort p;
        End ended = End::END_NONE;
        const Str sent = joined(p.sender.pass(pulse, ended));
        checkStr(sent, "STOP", "a pulse with nothing accepted before it: STOP");
        check(ended == End::END_SEND_GAP, "and END_SEND_GAP");
        checkStr(joined(p.lines), "STOP", "the pulse never reached the port");
    }
    {
        FakePort p;
        p.open({});
        checkInt(p.sender.sentMs(), 60001, "a line counts as sent when its write returns");
        p.nowMs += bibo::MINDER_MS;
        End ended = End::END_NONE;
        const Str sent = joined(p.sender.pass(drive, ended));
        checkStr(sent, "STEER 0.000 | ESC 1632", "a pass on time sends its pulse");
        check(ended == End::END_NONE, "and ends nothing");
        checkInt(p.waits.back(), wait, "the pulse may take SEND_GAP_STOP_MS from the STEER");
        checkInt(p.sender.takeGapMs(), bibo::MINDER_MS + 1, "takeGapMs gives the longest gap");
        checkInt(p.sender.takeGapMs(), 0, "once");
    }
    {
        FakePort p;
        p.open({Step{gap, ok}});
        p.nowMs += bibo::MINDER_MS;
        End ended = End::END_NONE;
        const Str sent = joined(p.sender.pass(drive, ended));
        checkStr(sent, "STEER 0.000 | STOP", "a STEER accepted late: STOP");
        check(ended == End::END_SEND_GAP, "and END_SEND_GAP");
        checkStr(joined(p.lines), "STEER 0.000 | STOP", "the pulse never reached the port");
    }
    {
        FakePort p;
        p.open({Step{wait, carlink::Result::RESULT_WRITE_FAILED}});
        p.nowMs += bibo::MINDER_MS;
        End ended = End::END_NONE;
        const Str sent = joined(p.sender.pass(drive, ended));
        checkStr(sent, "STEER 0.000 | STOP", "a STEER that stalled and failed: STOP");
        check(ended == End::END_SEND_GAP, "and END_SEND_GAP: no time was left for the pulse");
    }
    {
        FakePort p;
        const Int64 half = gap / 2;
        p.open({});
        p.nowMs += half;
        End ended = End::END_NONE;
        const Str sent = joined(p.sender.pass(pulse, ended));
        checkStr(sent, "ESC 1632", "a pulse half the gap after a line goes");
        checkInt(p.waits.back(), std::min<Int64>(gap - half, wait), "given only the time left");
    }
    {
        FakePort p;
        const Int64 half = gap / 2;
        p.open({Step{gap, ok}});
        p.nowMs += half;
        End ended = End::END_NONE;
        const Str sent = joined(p.sender.pass(pulse, ended));
        checkStr(sent, "STOP", "a pulse whose write outlasts that: STOP");
        check(ended == End::END_SEND_GAP, "and END_SEND_GAP");
        checkStr(joined(p.lines), "ESC 1632 | STOP", "its write was cut off, then STOP went");
    }
    {
        FakePort p;
        p.open({Step(), Step{1, carlink::Result::RESULT_CLOSED}});
        p.nowMs += bibo::MINDER_MS;
        End ended = End::END_NONE;
        const Str sent = joined(p.sender.pass(drive, ended));
        checkStr(sent, "STEER 0.000 | STOP", "a pulse the closed port refused: STOP");
        check(ended == End::END_LINK_LOST, "and END_LINK_LOST, not a send gap");
    }
    {
        FakePort p;
        p.open({});
        p.nowMs += gap + 50;
        End ended = End::END_NONE;
        const Vec<Str> held = {"STEER 0.000", "ESC NEUTRAL"};
        const Str sent = joined(p.sender.pass(held, ended));
        checkStr(sent, "STEER 0.000 | ESC NEUTRAL", "a late pass without a pulse sends it all");
        check(ended == End::END_NONE, "and ends nothing itself");
        checkInt(p.sender.takeGapMs(), gap + 51, "its gap goes to the Governor");
        checkInt(p.sender.worstGapMs(), gap + 51, "and into worstGapMs");
    }
}

int main()
{
    std::printf("test_carrules\n");
    testAhead();
    testNearestAndBlind();
    testToScan();
    testFlags();
    testFold();
    testPulseAndSteer();
    testEnds();
    testGovernorArming();
    testGovernorFreshness();
    testGovernorLatches();
    testGovernorDry();
    testSender();
    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("OVERALL: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
