// The JSON reader in hub/src/json.cxx.
//
//   tests\build_json_test.bat run
//
// THIS EXISTS BECAUSE THE ALTERNATIVE WAS strstr. A clangd completion reply is
// a few hundred kilobytes of nested objects, and the interesting fields sit
// inside an array inside an object. Scanning for `"label"` works until a doc
// comment contains that text, or a C++ signature contains a brace, or a string
// contains an escaped quote - and each of those produces a plausible WRONG
// answer rather than a failure.
//
// So the cases below are mostly the ones a scanner gets wrong: braces and
// key-looking text inside strings, escapes, and the empty containers that a
// hand-rolled parser forgets.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"
#include "json.hxx"

#include <cstdio>

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

static js::Value read(const Char* text, Bool& ok)
{
    Size at = 0;
    return js::parse(Str(text), ok, at);
}

int main()
{
    std::printf("\njson\n\n");

    Bool ok = false;

    // ---- the shapes -------------------------------------------------------
    js::Value v = read("{\"a\":1,\"b\":\"two\",\"c\":true,\"d\":null}", ok);
    check(ok, "a flat object parses");
    check(v.isObject(), "and is an object");
    check(v.at("a").integer(-1) == 1, "a number reads back");
    check(v.at("b").string() == "two", "a string reads back");
    check(v.at("c").boolean(false), "a bool reads back");
    check(v.at("d").isNull(), "null is null");

    // A MISSING KEY IS NOT A CRASH, and neither is a wrong type. Every caller
    // here is walking a reply it did not write.
    check(v.at("nope").isNull(), "a missing key is a null value");
    check(v.at("nope").at("deeper").isNull(), "and can be walked further");
    check(v.at("a").string("fallback") == "fallback",
          "reading a number as a string gives the fallback");
    check(v.at("b").integer(-7) == -7,
          "reading a string as a number gives the fallback");
    check(v.at(nullptr).isNull(), "a null key is a null value");

    // ---- arrays -----------------------------------------------------------
    v = read("{\"items\":[{\"label\":\"one\"},{\"label\":\"two\"}]}", ok);
    check(ok && v.at("items").size() == 2u, "an array of objects parses");
    check(v.at("items")[0].at("label").string() == "one", "element 0");
    check(v.at("items")[1].at("label").string() == "two", "element 1");
    check(v.at("items")[9].isNull(), "an index past the end is null, not a crash");

    // ---- the cases a scanner gets wrong -----------------------------------
    //
    // A BRACE INSIDE A STRING. This is not hypothetical: clangd returns C++
    // signatures, and `Void open(Bus* b)` sits next to snippets that contain
    // braces outright.
    v = read("{\"detail\":\"struct { int x; }\",\"kind\":3}", ok);
    check(ok && v.at("kind").integer() == 3,
          "a brace inside a string does not end the object");
    check(v.at("detail").string() == "struct { int x; }",
          "and the string survives intact");

    // KEY-LOOKING TEXT INSIDE A STRING. A scanner for `"label"` finds this one.
    v = read("{\"doc\":\"pass \\\"label\\\": to it\",\"label\":\"real\"}", ok);
    check(ok && v.at("label").string() == "real",
          "an escaped quoted key inside a string is not mistaken for the key");
    check(v.at("doc").string() == "pass \"label\": to it",
          "and the escapes come back out");

    // ESCAPES.
    v = read("{\"s\":\"a\\nb\\tc\\\\d\\/e\"}", ok);
    check(ok && v.at("s").string() == "a\nb\tc\\d/e", "the escape set decodes");

    // \u, WHICH CLANGD SENDS. Its completion labels carry a non-ASCII
    // decoration character, so getting this wrong corrupts every identifier.
    v = read("{\"s\":\"\\u0041\\u00e9\"}", ok);
    check(ok && v.at("s").string() == "A\xC3\xA9", "\\u decodes to UTF-8");

    // A surrogate pair is one character, not two broken ones.
    v = read("{\"s\":\"\\ud83d\\ude00\"}", ok);
    check(ok && v.at("s").size() == 0u, "a surrogate pair is an object-less string");
    check(v.at("s").string() == "\xF0\x9F\x98\x80",
          "and joins into one 4-byte character");

    // A LONE SURROGATE becomes a visible replacement rather than half a
    // character in the middle of an identifier.
    v = read("{\"s\":\"\\ud83dZ\"}", ok);
    check(ok && v.at("s").string() == "\xEF\xBF\xBDZ",
          "a lone surrogate becomes U+FFFD and parsing continues");

    // ---- empty containers, which hand-rolled parsers forget ---------------
    v = read("{\"a\":[],\"b\":{}}", ok);
    check(ok, "empty containers parse");
    check(v.at("a").isArray() && v.at("a").size() == 0u, "an empty array");
    check(v.at("b").isObject() && v.at("b").size() == 0u, "an empty object");

    // ---- whitespace and nesting -------------------------------------------
    v = read("  {\n  \"a\" : [ 1 , 2 ,\n 3 ] }\n", ok);
    check(ok && v.at("a").size() == 3u, "whitespace anywhere is fine");

    v = read("{\"a\":{\"b\":{\"c\":{\"d\":42}}}}", ok);
    check(ok && v.at("a").at("b").at("c").at("d").integer() == 42,
          "deep nesting");

    // ---- malformed input is REFUSED, not half-accepted --------------------
    static_cast<Void>(read("{\"a\":1", ok));
    check(!ok, "a truncated object is refused");
    static_cast<Void>(read("{\"a\":}", ok));
    check(!ok, "a missing value is refused");
    static_cast<Void>(read("{a:1}", ok));
    check(!ok, "an unquoted key is refused");
    static_cast<Void>(read("[1,2,", ok));
    check(!ok, "a truncated array is refused");
    static_cast<Void>(read("\"unterminated", ok));
    check(!ok, "an unterminated string is refused");
    static_cast<Void>(read("", ok));
    check(!ok, "empty input is refused");

    // ---- numbers ----------------------------------------------------------
    v = read("{\"a\":-12,\"b\":3.5,\"c\":1e3}", ok);
    check(ok && v.at("a").integer() == -12, "a negative integer");
    check(v.at("b").integer() == 3, "a float truncates toward zero");
    check(v.at("c").integer() == 1000, "exponent notation");

    // ---- quote() round-trips ----------------------------------------------
    //
    // The one part of a request that can contain anything is a file's text, so
    // this is the function that stops a source file breaking the protocol.
    const Str awkward = Str("say \"hi\"\n\tand \\ back")
                      + Str(1, static_cast<Char>(0x01));
    const Str wrapped = js::quote(awkward);

    v = read((Str("{\"t\":") + wrapped + "}").c_str(), ok);
    check(ok, "quote() produces something parseable");
    check(v.at("t").string() == awkward, "and it round-trips exactly");

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
