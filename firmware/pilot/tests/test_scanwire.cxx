// The scan feed's wire format, held to its header.
//
//   tests\build_scanwire_test.bat run
//
// Pure string work, like proto: the same object file goes into the board's
// scanfeed and the hub's lidar source, so what is proved here is proved for
// both ends at once.
//
// The cases that matter: a frame whose count and samples disagree (a line cut
// in transit is not a shorter revolution), a sample with a letter in it, an
// angle of exactly 360 (the device says 0), a quality over 63, and the Frame
// being EMPTIED on every kind that is not a frame - a reader that keeps one
// Line across calls must never draw last revolution's points under this
// line's MOTOR 0.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "scanwire.hxx"

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

// What the wire does to an angle: whole centi-degrees, wrapped. Written the
// way the formatter does it so the round trip below is exact rather than
// approximately equal - a tolerance would hide an off-by-one centi-degree.
static Float32 wireAngle(Float32 deg)
{
    Int32 centi = static_cast<Int32>(deg * 100.0f + (deg >= 0.0f ? 0.5f : -0.5f)) % 36000;
    if(centi < 0)
    {
        centi += 36000;
    }
    return static_cast<Float32>(centi) / 100.0f;
}

static Float32 wireMm(Float32 mm)
{
    return mm > 0.0f ? static_cast<Float32>(static_cast<Int32>(mm + 0.5f)) : 0.0f;
}

Int32 main()
{
    std::printf("\nscanwire - the scan feed's wire format\n\n");

    using scanwire::Frame;
    using scanwire::Kind;
    using scanwire::Line;
    using scanwire::Sample;

    // ---- a revolution, out and back ------------------------------------------
    {
        Frame f;
        f.hz = 10.123f;
        for(Int32 i = 0; i < 500; ++i)
        {
            Sample s;
            s.angleDeg = static_cast<Float32>(i) * 0.7197f;   // not a multiple of 0.01: rounding is real
            s.distMm = static_cast<Float32>((i * 37) % 12000) + 0.4f;
            s.quality = static_cast<UInt8>(i % 64);
            f.samples.push_back(s);
        }
        const Str line = scanwire::formatFrame(f);
        check(!line.empty() && line.back() == '\n', "a frame ends in one newline");
        check(line.find('\n') == line.size() - 1, "and contains no other");
        check(line.rfind("F 500 10123 ", 0) == 0, "its head is F <count> <milli-hertz>");

        Line out;
        check(
            scanwire::parse(StrView(line).substr(0, line.size() - 1), &out) == Kind::KIND_FRAME,
            "parses as a frame"
        );
        check(out.frame.samples.size() == 500, "with all 500 samples");
        check(out.frame.hz == 10.123f, "hz survives the milli-hertz round trip exactly");

        Bool angles = true;
        Bool dists = true;
        Bool quals = true;
        for(Size i = 0; i < out.frame.samples.size() && i < f.samples.size(); ++i)
        {
            angles = angles && out.frame.samples[i].angleDeg == wireAngle(f.samples[i].angleDeg);
            dists = dists && out.frame.samples[i].distMm == wireMm(f.samples[i].distMm);
            quals = quals && out.frame.samples[i].quality == f.samples[i].quality;
        }
        check(angles, "every angle is exact to the centi-degree");
        check(dists, "every distance is exact to the millimetre");
        check(quals, "every quality is what was sent");

        // The point is that the ROUNDING happened, not just that it was
        // consistent: 3 * 0.7197 is 2.1591 degrees and the wire says 216.
        check(
            out.frame.samples[3].angleDeg == 2.16f,
            "2.1591 degrees went out as 216 centi-degrees"
        );
        check(out.frame.samples[1].distMm == 37.0f, "37.4 mm went out as 37");
    }

    // ---- the edges of a sample -------------------------------------------------
    {
        Frame f;
        Sample s;
        s.angleDeg = 360.0f;
        s.distMm = 1000.0f;
        s.quality = 10;
        f.samples.push_back(s);

        s.angleDeg = 359.996f;   // rounds to 36000, which is 0
        f.samples.push_back(s);

        s.angleDeg = 90.0f;
        s.quality = 200;   // the byte holds it; the wire does not
        f.samples.push_back(s);

        s.quality = 5;
        s.distMm = -3.0f;   // a negative distance is no return
        f.samples.push_back(s);

        s.distMm = 0.0f;    // and so is an absent one
        f.samples.push_back(s);

        const Str line = scanwire::formatFrame(f);
        checkStr(
            line,
            "F 5 0 0,1000,10 0,1000,10 9000,1000,63 9000,0,5 9000,0,5\n",
            "360 wraps to 0, quality clamps at 63, no return is 0"
        );

        Line out;
        check(
            scanwire::parse(StrView(line).substr(0, line.size() - 1), &out) == Kind::KIND_FRAME,
            "and that line reads back"
        );
        check(
            out.frame.samples.size() == 5 && out.frame.samples[0].angleDeg == 0.0f,
            "with the wrapped angle at 0"
        );
        check(out.frame.hz == 0.0f, "an hz of 0 is 0");
    }

    // ---- the other lines, out and back ----------------------------------------
    {
        scanwire::Info info;
        info.model = 65;
        info.fwMajor = 1;
        info.fwMinor = 2;
        info.hwRev = 18;
        info.serial = "DEADBEEF0123456789ABCDEF00112233";
        checkStr(
            scanwire::formatInfo(info),
            "INFO 65 1.2 18 DEADBEEF0123456789ABCDEF00112233\n",
            "an INFO line"
        );

        Line out;
        check(
            scanwire::parse("INFO 65 1.2 18 DEADBEEF0123456789ABCDEF00112233", &out) == Kind::KIND_INFO,
            "parses as INFO"
        );
        check(
            out.info.model == 65 && out.info.fwMajor == 1 && out.info.fwMinor == 2 && out.info.hwRev == 18,
            "with every number in its place"
        );
        checkStr(out.info.serial, "DEADBEEF0123456789ABCDEF00112233", "and the serial as text");

        info.serial.clear();
        checkStr(
            scanwire::formatInfo(info),
            "INFO 65 1.2 18 -\n",
            "an empty serial is written as -"
        );
        check(
            scanwire::parse("INFO 65 1.2 18 -", &out) == Kind::KIND_INFO && out.info.serial.empty(),
            "and read back as empty"
        );
    }
    {
        checkStr(scanwire::formatHealth(2), "HEALTH 2\n", "a HEALTH line");
        checkStr(scanwire::formatMotor(true), "MOTOR 1\n", "MOTOR 1");
        checkStr(scanwire::formatMotor(false), "MOTOR 0\n", "MOTOR 0");
        checkStr(
            scanwire::formatErr("another program has /dev/ttyUSB0"),
            "ERR another program has /dev/ttyUSB0\n",
            "an ERR line"
        );
        checkStr(
            scanwire::formatErr("two\nlines\r\n"),
            "ERR two lines  \n",
            "an ERR with newlines in it is still one line"
        );
        checkStr(scanwire::formatQuit(), "QUIT\n", "QUIT");

        Line out;
        check(
            scanwire::parse("HEALTH 1", &out) == Kind::KIND_HEALTH && out.health == 1,
            "HEALTH 1 reads as health 1"
        );
        check(
            scanwire::parse("MOTOR 1", &out) == Kind::KIND_MOTOR && out.motor,
            "MOTOR 1 reads as on"
        );
        check(
            scanwire::parse("MOTOR 0", &out) == Kind::KIND_MOTOR && !out.motor,
            "MOTOR 0 reads as off"
        );
        check(
            scanwire::parse("ERR   the port is held", &out) == Kind::KIND_ERR,
            "ERR reads as ERR"
        );
        checkStr(out.text, "the port is held", "with its message whole and unpadded");
        check(scanwire::parse("QUIT", &out) == Kind::KIND_QUIT, "QUIT reads as QUIT");
        check(scanwire::parse("", &out) == Kind::KIND_EMPTY, "empty is empty");
        check(scanwire::parse("  \t ", &out) == Kind::KIND_EMPTY, "and so is whitespace");
        check(
            scanwire::parse("HELLO there", &out) == Kind::KIND_UNKNOWN,
            "a line with a verb this reader has no name for is UNKNOWN"
        );
        checkStr(out.text, "HELLO", "and says which verb");
        check(
            scanwire::parse("MOTOR 1\r", &out) == Kind::KIND_MOTOR && out.motor,
            "a trailing carriage return is tolerated"
        );
        check(
            scanwire::parse("F 1 0 100,200,3\r", &out) == Kind::KIND_FRAME && out.frame.samples.size() == 1,
            "on a frame too"
        );
    }

    // ---- lines that are known and wrong are BAD, not shorter ------------------
    {
        Line out;
        check(
            scanwire::parse("F 3 0 100,200,3 100,200,3", &out) == Kind::KIND_BAD,
            "a count that disagrees with its samples is BAD"
        );
        check(out.frame.samples.empty(), "and the samples it did read are not handed on");
        check(
            scanwire::parse("F 1 0 100,200,3 100,200,3", &out) == Kind::KIND_BAD,
            "in either direction"
        );
        check(
            scanwire::parse("F 1 0 1x0,200,3", &out) == Kind::KIND_BAD,
            "a sample with a letter in it is BAD"
        );
        check(
            scanwire::parse("F 1 0 100,200", &out) == Kind::KIND_BAD,
            "a sample with two parts is BAD"
        );
        check(
            scanwire::parse("F 1 0 36000,200,3", &out) == Kind::KIND_BAD,
            "an angle of 36000 is BAD - the wire says 0..35999"
        );
        check(
            scanwire::parse("F 1 0 100,200,64", &out) == Kind::KIND_BAD,
            "a quality of 64 is BAD"
        );
        check(
            scanwire::parse("F 1 0 100,-1,3", &out) == Kind::KIND_BAD,
            "a negative distance is BAD"
        );
        check(scanwire::parse("F 1", &out) == Kind::KIND_BAD, "a frame with no rate is BAD");
        check(
            scanwire::parse("F x 0", &out) == Kind::KIND_BAD,
            "a frame whose count is not a number is BAD"
        );
        check(
            scanwire::parse("F 0 5000", &out) == Kind::KIND_FRAME && out.frame.samples.empty() && out.frame.hz == 5.0f,
            "but a frame of zero samples is a frame"
        );
        check(
            scanwire::parse("HEALTH 3", &out) == Kind::KIND_BAD,
            "HEALTH 3 is BAD - the wire says 0, 1 or 2"
        );
        check(scanwire::parse("HEALTH", &out) == Kind::KIND_BAD, "HEALTH with no value is BAD");
        check(scanwire::parse("MOTOR 2", &out) == Kind::KIND_BAD, "MOTOR 2 is BAD");
        check(
            scanwire::parse("MOTOR on", &out) == Kind::KIND_BAD,
            "MOTOR on is BAD - the wire says 0 or 1"
        );
        check(
            scanwire::parse("INFO 65 1 18 -", &out) == Kind::KIND_BAD,
            "an INFO whose firmware has no dot is BAD"
        );
        check(scanwire::parse("INFO 65 1.2", &out) == Kind::KIND_BAD, "an INFO cut short is BAD");
    }

    // ---- the Frame is emptied on every kind that is not a frame ---------------
    {
        Line out;
        check(
            scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME && out.frame.samples.size() == 2,
            "a frame fills the Frame"
        );
        check(
            scanwire::parse("MOTOR 0", &out) == Kind::KIND_MOTOR && out.frame.samples.empty(),
            "MOTOR empties it"
        );

        check(scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME, "refilled");
        check(
            scanwire::parse("F 2 0 100,200,3 200,300", &out) == Kind::KIND_BAD && out.frame.samples.empty(),
            "BAD empties it"
        );

        check(scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME, "refilled");
        check(
            scanwire::parse("HEALTH 0", &out) == Kind::KIND_HEALTH && out.frame.samples.empty(),
            "HEALTH empties it"
        );

        check(scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME, "refilled");
        check(
            scanwire::parse("INFO 65 1.2 18 -", &out) == Kind::KIND_INFO && out.frame.samples.empty(),
            "INFO empties it"
        );

        check(scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME, "refilled");
        check(
            scanwire::parse("ERR x", &out) == Kind::KIND_ERR && out.frame.samples.empty(),
            "ERR empties it"
        );

        check(scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME, "refilled");
        check(
            scanwire::parse("QUIT", &out) == Kind::KIND_QUIT && out.frame.samples.empty(),
            "QUIT empties it"
        );

        check(scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME, "refilled");
        check(
            scanwire::parse("WHATEVER", &out) == Kind::KIND_UNKNOWN && out.frame.samples.empty(),
            "UNKNOWN empties it"
        );

        check(scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME, "refilled");
        check(
            scanwire::parse("", &out) == Kind::KIND_EMPTY && out.frame.samples.empty(),
            "EMPTY empties it"
        );

        // And a frame REPLACES, rather than appends to, the last one.
        check(scanwire::parse("F 2 0 100,200,3 200,300,4", &out) == Kind::KIND_FRAME, "refilled");
        check(
            scanwire::parse("F 1 0 100,200,3", &out) == Kind::KIND_FRAME && out.frame.samples.size() == 1,
            "a frame replaces the previous frame"
        );
    }

    // ---- the locale trap, again -------------------------------------------------
    //
    // Nothing on the wire is a float, so a comma can only appear as the
    // separator inside a sample. Checked because the day somebody "improves"
    // the rate to %.3f, this is the line that catches it.
    {
        Frame f;
        f.hz = 10.5f;
        const Str line = scanwire::formatFrame(f);
        checkStr(line, "F 0 10500\n", "the rate is milli-hertz, an integer, on every locale");
    }

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
