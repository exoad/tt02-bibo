// proto: the car's line protocol, as the companion board reads and writes it.
//
//   tools\test.bat proto run
//
// The cases that matter each give a plausible wrong answer under the obvious
// implementation: a key inside another key, a value that is not a number, a reply
// that is neither OK nor ERR, and a comma-decimal locale.
#include "shared.hxx"
#include "proto.hxx"

#include <cstdio>
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

Int32 main()
{
    std::printf("\nproto - the car's line protocol\n\n");
    {
        const proto::Reply r = proto::read("OK drive servo=1500 esc=1541");
        check(r.kind == proto::Kind::KIND_OK, "OK is recognized");
        checkStr(r.topic, "drive", "and its topic split off");
        checkStr(r.rest, "servo=1500 esc=1541", "and the fields kept");
    }
    {
        const proto::Reply r = proto::read("INFO status up_ms=4210 led=on");
        check(r.kind == proto::Kind::KIND_INFO, "INFO is recognized");
        checkStr(r.topic, "status", "and its topic split off");
    }
    {
        const proto::Reply r = proto::read("ERR blink wants a rate in hz");
        check(r.kind == proto::Kind::KIND_ERR, "ERR is recognized");
        check(r.topic.empty(), "and has no topic invented for it");
        checkStr(r.rest, "blink wants a rate in hz", "the reason is kept whole");
    }
    {
        const proto::Reply r = proto::read("bibo firmware, RP2350");
        check(r.kind == proto::Kind::KIND_OTHER, "an unrecognized line is OTHER, not a fault");
    }
    check(proto::read("").kind == proto::Kind::KIND_EMPTY, "empty is empty");
    check(proto::read("   \t ").kind == proto::Kind::KIND_EMPTY, "and so is whitespace");
    checkStr(proto::read("OK led on\r\n").line, "OK led on", "CRLF is stripped from the kept line");
    {
        const Str line = "servo=1500 servo_t=1480 esc=1541 esc_t=1600";
        Int32 v = 0;
        check(proto::fieldInt(line, "servo=", v) && v == 1500, "servo=");
        check(proto::fieldInt(line, "servo_t=", v) && v == 1480, "servo_t=");
        check(proto::fieldInt(line, "esc=", v) && v == 1541, "esc=");
        check(proto::fieldInt(line, "esc_t=", v) && v == 1600, "esc_t=");
    }
    {
        // strstr(line, "esc=") would find the "esc=" inside "desc=".
        const Str line = "desc=99 esc=1541";
        Int32 v = 0;
        check(proto::fieldInt(line, "esc=", v) && v == 1541, "esc= is not matched inside desc=");
        std::printf(
            "        (strstr would have returned %d here)\n",
            std::atoi(std::strstr(line.c_str(), "esc=") + 4)
        );
        check(proto::fieldInt(line, "desc=", v) && v == 99, "and desc= still reads correctly");
    }
    {
        const Str line = "a=1 b=2";
        Int32     v = 0;
        check(!proto::fieldInt(line, "c=", v), "a missing key is false");
        check(!proto::fieldInt(line, "", v), "an empty key is false");
        check(!proto::fieldInt(line, nullptr, v), "a null key is false");
    }
    {
        // esc=off is not a 0, which would read as neutral throttle.
        const Str line = "esc=off servo=1500";
        Int32 v = -1;
        check(
            !proto::fieldInt(line, "esc=", v),
            "a non-numeric value is refused, not defaulted to 0"
        );
        check(v == -1, "and the out parameter is left alone");
        Str raw;
        check(
            proto::field(line, "esc=", raw) && raw == "off",
            "though it can still be read as text"
        );
    }
    {
        const Str line = "esc=1541abc";
        Int32     v = 0;
        check(!proto::fieldInt(line, "esc=", v), "a trailing tail is refused");
    }
    {
        // The shape of firmware/app/main.cxx printDrive(). Every value differs, and
        // several keys are prefixes of others on the line (esc= of esc_t=, esc_min=
        // and esc_max=; slew= of slew_esc=), so matching characters rather than
        // names reads a wrong number.
        const Str line =
            "OK drive servo=1600 servo_t=1610 esc=1560 esc_t=1570 armed=1 "
            "servo_on=1 servo_c=1480 steer_m=-300 steer_now=-250 slew=8 "
            "slew_esc=12 servo_min=1230 servo_max=1660 esc_min=1541 esc_max=1600";
        const proto::Reply r = proto::read(line);
        check(r.kind == proto::Kind::KIND_OK, "the drive reply is an OK");
        checkStr(r.topic, "drive", "with topic drive");
        Int32 v = 0;
        check(proto::fieldInt(r.rest, "armed=", v) && v == 1, "armed= reads the arm state");
        check(
            proto::fieldInt(r.rest, "esc=", v) && v == 1560,
            "esc= is not esc_t=, esc_min= or esc_max="
        );
        check(
            proto::fieldInt(r.rest, "steer_now=", v) && v == -250,
            "steer_now= is negative and is not steer_m="
        );
        check(proto::fieldInt(r.rest, "slew=", v) && v == 8, "slew= is not slew_esc=");
        // The '=' belongs to the key: without it strtol reads "=1" and fails,
        // leaving the out parameter as it was.
        v = 0;
        check(
            !proto::fieldInt(r.rest, "armed", v),
            "a key missing its = is refused, not silently empty"
        );
        check(v == 0, "and it leaves the out parameter alone");
    }
    {
        const Str line = "hz=2.50 gain=-0.125";
        Float32   f = 0.0f;
        check(proto::fieldFloat(line, "hz=", f) && f > 2.49f && f < 2.51f, "a float field");
        check(
            proto::fieldFloat(line, "gain=", f) && f < -0.124f && f > -0.126f,
            "a negative float field"
        );
    }
    {
        const Str line = "up_ms=4210 led=on";
        Int32     v = 0;
        check(
            proto::fieldInt(line, "up_ms=", v) && v == 4210,
            "the first field on the line is found"
        );
    }
    checkStr(proto::steer(0.25f), "STEER 0.250", "a steering command");
    checkStr(proto::steer(-0.5f), "STEER -0.500", "a negative one");
    checkStr(proto::steer(0.0f), "STEER 0.000", "center");
    checkStr(proto::steer(4.0f), "STEER 1.000", "beyond full lock is clamped");
    checkStr(proto::steer(-9.0f), "STEER -1.000", "and the other way");
    checkStr(proto::steer(0.2499f), "STEER 0.250", "the third decimal rounds");
    checkStr(proto::escUs(1541), "ESC 1541", "a throttle pulse");
    checkStr(proto::stop(), "STOP", "the stop command");
    checkStr(proto::command("LED", "BLINK 2"), "LED BLINK 2", "a built command");
    checkStr(proto::command("PING"), "PING", "one with no arguments");
    checkStr(proto::command(nullptr), "", "a null verb makes nothing");
    // The locale trap: steer() avoids snprintf("%.3f"), which prints a comma under a
    // comma-decimal locale, and the board would read STEER 0.
    {
        const Str s = proto::steer(0.25f);
        check(s.find(',') == Str::npos, "no comma survives into a command");
        check(s.find('.') != Str::npos, "the separator is a full stop");
    }
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
