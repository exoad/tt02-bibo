// The car's line protocol, as read and written by the companion board.
//
//   tests\build_proto_test.bat run
//
// Pure string work, so all of it is testable on a laptop months before the
// Orange Pi is plugged in - which is the point of writing this layer first.
//
// The cases that matter are not the happy ones. They are: a key that is a
// substring of another key, a field whose value is not a number, a reply that
// is neither OK nor ERR, and a float formatted on a machine whose locale uses a
// comma. Each of those returns a plausible wrong answer under the obvious
// implementation.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "proto.hxx"

#include <cstdio>
#include <cstring>

static Int32 failures = 0;
static Int32 checks   = 0;

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
        std::printf("        got  \"%s\"\n        want \"%s\"\n",
                    got.c_str(), want);
    }
}

Int32 main()
{
    std::printf("\nproto - the car's line protocol\n\n");

    // ---- classifying a line -----------------------------------------------
    {
        const proto::Reply r = proto::read("OK drive servo=1500 esc=1541");
        check(r.kind == proto::Kind::KIND_OK, "OK is recognised");
        checkStr(r.topic, "drive", "and its topic split off");
        checkStr(r.rest, "servo=1500 esc=1541", "and the fields kept");
    }
    {
        const proto::Reply r = proto::read("INFO status up_ms=4210 led=on");
        check(r.kind == proto::Kind::KIND_INFO, "INFO is recognised");
        checkStr(r.topic, "status", "and its topic split off");
    }
    {
        // ERR keeps its whole reason. Splitting a topic off the front would
        // produce "blink" as a topic when the sentence is "blink wants a rate".
        const proto::Reply r = proto::read("ERR blink wants a rate in hz");
        check(r.kind == proto::Kind::KIND_ERR, "ERR is recognised");
        check(r.topic.empty(), "and has no topic invented for it");
        checkStr(r.rest, "blink wants a rate in hz", "the reason is kept whole");
    }
    {
        // Boot banners and a person typing in the same port are NOT errors.
        const proto::Reply r = proto::read("bibo firmware, RP2350");
        check(r.kind == proto::Kind::KIND_OTHER,
              "an unrecognised line is OTHER, not a fault");
    }
    check(proto::read("").kind == proto::Kind::KIND_EMPTY, "empty is empty");
    check(proto::read("   \t ").kind == proto::Kind::KIND_EMPTY,
          "and so is whitespace");

    // Either line ending, since the transport may hand over either.
    checkStr(proto::read("OK led on\r\n").line, "OK led on",
             "CRLF is stripped from the kept line");

    // ---- fields, and the trap ---------------------------------------------
    {
        const Str line = "servo=1500 servo_t=1480 esc=1541 esc_t=1600";

        Int32 v = 0;
        check(proto::fieldInt(line, "servo=", v) && v == 1500, "servo=");
        check(proto::fieldInt(line, "servo_t=", v) && v == 1480, "servo_t=");
        check(proto::fieldInt(line, "esc=", v) && v == 1541, "esc=");
        check(proto::fieldInt(line, "esc_t=", v) && v == 1600, "esc_t=");
    }
    {
        // THE CASE THE OBVIOUS IMPLEMENTATION GETS WRONG.
        //
        // strstr(line, "esc=") finds the "esc=" inside "desc=" and reports
        // success with a value from a field nobody asked for. Reading by name
        // has to mean reading a NAME, not a run of characters.
        const Str line = "desc=99 esc=1541";

        Int32 v = 0;
        check(proto::fieldInt(line, "esc=", v) && v == 1541,
              "esc= is not matched inside desc=");
        std::printf("        (strstr would have returned %d here)\n",
                    std::atoi(std::strstr(line.c_str(), "esc=") + 4));

        check(proto::fieldInt(line, "desc=", v) && v == 99,
              "and desc= still reads correctly");
    }
    {
        const Str line = "a=1 b=2";
        Int32     v    = 0;
        check(!proto::fieldInt(line, "c=", v), "a missing key is false");
        check(!proto::fieldInt(line, "", v), "an empty key is false");
        check(!proto::fieldInt(line, nullptr, v), "a null key is false");
    }
    {
        // A field that is present and not a number is NOT a zero. `esc=off` is
        // a different answer, and returning 0 would read as neutral throttle.
        const Str line = "esc=off servo=1500";

        Int32 v = -1;
        check(!proto::fieldInt(line, "esc=", v),
              "a non-numeric value is refused, not defaulted to 0");
        check(v == -1, "and the out parameter is left alone");

        Str raw;
        check(proto::field(line, "esc=", raw) && raw == "off",
              "though it can still be read as text");
    }
    {
        // Partial numbers are the same trap one layer down: "1541abc" must not
        // quietly become 1541.
        const Str line = "esc=1541abc";
        Int32     v    = 0;
        check(!proto::fieldInt(line, "esc=", v), "a trailing tail is refused");
    }
    {
        const Str line = "hz=2.50 gain=-0.125";
        Float32   f    = 0.0f;
        check(proto::fieldFloat(line, "hz=", f) && f > 2.49f && f < 2.51f,
              "a float field");
        check(proto::fieldFloat(line, "gain=", f) && f < -0.124f && f > -0.126f,
              "a negative float field");
    }
    {
        // The key at the very start of the text, which the boundary rule has to
        // allow rather than require a leading space.
        const Str line = "up_ms=4210 led=on";
        Int32     v    = 0;
        check(proto::fieldInt(line, "up_ms=", v) && v == 4210,
              "the first field on the line is found");
    }

    // ---- what we send ------------------------------------------------------
    checkStr(proto::steer(0.25f), "STEER 0.250", "a steering command");
    checkStr(proto::steer(-0.5f), "STEER -0.500", "a negative one");
    checkStr(proto::steer(0.0f), "STEER 0.000", "centre");

    // Clamped, because the board clamps anyway and a companion sending 4.0 has
    // a bug worth seeing where it happens.
    checkStr(proto::steer(4.0f), "STEER 1.000", "beyond full lock is clamped");
    checkStr(proto::steer(-9.0f), "STEER -1.000", "and the other way");

    // Rounding rather than truncation: 0.2499 must not become 0.249.
    checkStr(proto::steer(0.2499f), "STEER 0.250", "the third decimal rounds");

    checkStr(proto::escUs(1541), "ESC 1541", "a throttle pulse");
    checkStr(proto::stop(), "STOP", "the stop command");
    checkStr(proto::command("LED", "BLINK 2"), "LED BLINK 2", "a built command");
    checkStr(proto::command("PING"), "PING", "one with no arguments");
    checkStr(proto::command(nullptr), "", "a null verb makes nothing");

    // ---- the locale trap ---------------------------------------------------
    //
    // The reason steer() does not use snprintf("%.3f"). If this ever prints a
    // comma the board reads STEER 0 and the car goes straight through the first
    // corner it was told to turn. Checked rather than trusted, because the
    // failure only appears on a machine configured differently from this one.
    {
        const Str s = proto::steer(0.25f);
        check(s.find(',') == Str::npos, "no comma survives into a command");
        check(s.find('.') != Str::npos, "the separator is a full stop");
    }

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
