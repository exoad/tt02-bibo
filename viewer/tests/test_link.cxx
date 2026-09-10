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
//   - THE CAMERA'S SOCKET HALF. The decode path below is driven with
//     hand-built CAMERA frames and a real JPEG, but no SUBSCRIBE has ever been
//     put on a wire: the board-side producer is being written in parallel and
//     no board has ever sent a CAMERA frame. What is proved here is that the
//     bytes survive the codec and become pixels; what is NOT proved is that
//     the board answers a subscription, that it stops sending when one is
//     withdrawn, or that a real camera's JPEG looks like this one.
//   - the texture upload and the window. Those need a D3D11 device, which is
//     why the decoder is its own module (jpeg.cxx) and the window is not.
//
// The framing, the CRC, the resync and every message body belong to
// firmware/pilot/src/bibowire.cxx and its 312 checks; this file uses that codec
// to BUILD its inputs rather than restating what it already proves.
//
// Exits 0 on PASS, 1 on FAIL.

#include "shared.hxx"

#include "bibowire.hxx"
#include "link.hxx"
#include "jpeg.hxx"

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

// A REAL, DECODABLE JPEG - 16x12, quality 92 - and not a plausible-looking
// blob, because a blob would prove the framing and quietly prove nothing about
// the decoder this viewer actually ships.
//
// It carries a COM segment holding two kinds of hostile bytes:
//
//   - `ff d8 ff` runs, which is what an embedded EXIF thumbnail looks like. A
//     valid JPEG cannot carry those in its entropy-coded data - 0xFF is always
//     byte-stuffed - so a comment segment is how they really turn up, and this
//     one puts them at offsets 6, 12, 14 and 21 as well as at 0.
//   - `42 57`, which is BIBOWIRE'S OWN FRAME MAGIC, four times over. A reader
//     that hunted for the magic inside a payload instead of trusting byteLen
//     would resync in the middle of a picture; this is the byte pattern that
//     catches it, and testCameraByteAtATime is where it would show.
static const Array<UInt8, 741> TINY_JPEG = {
    0xFF, 0xD8, 0xFF, 0xFE, 0x00, 0x18, 0xFF, 0xD8, 0xFF, 0xE0, 0x42, 0x57, 0xFF, 0xD8,
    0xFF, 0xD8, 0xFF, 0x42, 0x57, 0x42, 0x57, 0xFF, 0xD8, 0xFF, 0x42, 0x57, 0x00, 0x10,
    0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01,
    0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43, 0x00, 0x03, 0x02, 0x02, 0x02, 0x02,
    0x02, 0x03, 0x02, 0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x04, 0x06, 0x04, 0x04, 0x04,
    0x04, 0x04, 0x08, 0x06, 0x06, 0x05, 0x06, 0x09, 0x08, 0x0A, 0x0A, 0x09, 0x08, 0x09,
    0x09, 0x0A, 0x0C, 0x0F, 0x0C, 0x0A, 0x0B, 0x0E, 0x0B, 0x09, 0x09, 0x0D, 0x11, 0x0D,
    0x0E, 0x0F, 0x10, 0x10, 0x11, 0x10, 0x0A, 0x0C, 0x12, 0x13, 0x12, 0x10, 0x13, 0x0F,
    0x10, 0x10, 0x10, 0xFF, 0xDB, 0x00, 0x43, 0x01, 0x03, 0x03, 0x03, 0x04, 0x03, 0x04,
    0x08, 0x04, 0x04, 0x08, 0x10, 0x0B, 0x09, 0x0B, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
    0x10, 0x10, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0x0C, 0x00, 0x10, 0x03, 0x01, 0x22,
    0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xC4, 0x00, 0x1F, 0x00, 0x00, 0x01,
    0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xC4,
    0x00, 0xB5, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04,
    0x04, 0x00, 0x00, 0x01, 0x7D, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21,
    0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1,
    0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82,
    0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x34,
    0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A,
    0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86,
    0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2,
    0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
    0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3,
    0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
    0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF,
    0xC4, 0x00, 0x1F, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00, 0xB5, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04,
    0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02,
    0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13,
    0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52,
    0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18,
    0x19, 0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43,
    0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77,
    0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8,
    0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4,
    0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9,
    0xDA, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5,
    0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xDA, 0x00, 0x0C, 0x03, 0x01, 0x00, 0x02, 0x11,
    0x03, 0x11, 0x00, 0x3F, 0x00, 0xF0, 0x0F, 0xF8, 0x49, 0xB4, 0x3F, 0xF9, 0xFE, 0xFF,
    0x00, 0xC8, 0x4F, 0xFE, 0x15, 0xF6, 0x7F, 0xFC, 0x28, 0x4F, 0x8B, 0x3F, 0xF4, 0x2A,
    0x7F, 0xE4, 0xF5, 0xB7, 0xFF, 0x00, 0x1C, 0xAF, 0xCF, 0x4A, 0xFD, 0xCB, 0xAF, 0x6F,
    0xC4, 0x8F, 0x09, 0xB2, 0x5C, 0x93, 0xEA, 0xBF, 0x57, 0xAB, 0x55, 0xF3, 0xF3, 0xDF,
    0x9A, 0x50, 0x7B, 0x72, 0x6D, 0x68, 0x2E, 0xE6, 0x7C, 0x73, 0x8F, 0xA9, 0xE2, 0x8F,
    0xB0, 0xFE, 0xD8, 0x4A, 0x1F, 0x57, 0xE7, 0xE5, 0xF6, 0x5A, 0x5F, 0xDA, 0x72, 0xDF,
    0x9B, 0x99, 0xCE, 0xF6, 0xE4, 0x56, 0xB5, 0xBA, 0xDE, 0xFD, 0x3F, 0xFF, 0xD9,
};

static Size pushCamera(Vec<UInt8>& out, UInt32 index, UInt64 tUs, const Vec<UInt8>& pic)
{
    bibowire::Camera m;
    m.tMonoUs = tUs;
    m.frameIndex = index;
    m.width = 16;
    m.height = 12;
    m.codec = 1;
    m.flags = 0;
    m.data = pic;
    Vec<UInt8> body(4096, 0);
    const Size n = bibowire::writeCamera(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_CAMERA, body.data(), n);
}

[[nodiscard]] static UInt8 channelAt(const jpeg::Picture& pic, Int32 x, Int32 y, Int32 ch)
{
    const Size row = static_cast<Size>(y) * static_cast<Size>(pic.width);
    const Size at = ((row + static_cast<Size>(x)) * 4u) + static_cast<Size>(ch);
    return at < pic.rgba.size() ? pic.rgba[at] : 0u;
}

// JPEG is lossy, so the tolerance is real. The numbers it is compared against
// are what PIL decoded the SAME bytes to, which makes this a comparison
// between two independent decoders rather than against what was painted.
[[nodiscard]] static Bool nearByte(UInt8 got, Int32 want)
{
    const Int32 delta = static_cast<Int32>(got) - want;
    return delta > -24 && delta < 24;
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

static Void testCameraFrames()
{
    std::printf("\n-- a CAMERA frame, carried verbatim --\n");

    // The array is the whole file, not a truncated paste: a short one would
    // zero-fill silently and every check below would still be testing
    // something, just not a JPEG.
    check(TINY_JPEG[TINY_JPEG.size() - 1] == 0xD9u, "the sample ends with a JPEG EOI");

    const Vec<UInt8> jpegBytes(TINY_JPEG.begin(), TINY_JPEG.end());

    Vec<UInt8> wire;
    static_cast<Void>(pushCamera(wire, 7, 1000000, jpegBytes));

    link::Session s;
    const Size used = feed(s, wire, 1000);
    check(used == wire.size(), "the whole camera frame is consumed");
    check(s.haveCamera, "a camera frame arrived");
    check(s.cameraFrames == 1u, "and is counted");
    check(s.camera.frameIndex == 7u, "with its frame index");
    check(s.camera.width == 16u && s.camera.height == 12u, "and its dimensions");
    check(s.camera.codec == 1u, "and its codec tag, echoed rather than assumed");

    // THE BYTES, EXACTLY. The payload contains bibowire's own frame magic and
    // five ff d8 ff runs; a reader that scanned for either instead of trusting
    // byteLen would have truncated the picture here.
    check(s.camera.data.size() == jpegBytes.size(), "the JPEG is the length it was sent at");
    check(s.camera.data == jpegBytes, "and is byte-for-byte what the board sent");

    // A reconnect subscribes to nothing and carries no picture: the board
    // keeps no subscription across a session, so neither may this.
    s.cameraSubscribed = true;
    link::clearSession(s);
    check(!s.cameraSubscribed, "a reconnect subscribes to nothing");
    check(!s.haveCamera, "and carries no picture across");
}

static Void testCameraByteAtATime()
{
    std::printf("\n-- the same camera frame, one byte at a time --\n");

    const Vec<UInt8> jpegBytes(TINY_JPEG.begin(), TINY_JPEG.end());
    Vec<UInt8> wire;
    static_cast<Void>(pushScan(wire, scanOf(3, 900000)));
    static_cast<Void>(pushCamera(wire, 1, 1000000, jpegBytes));
    static_cast<Void>(pushScan(wire, scanOf(4, 1100000)));

    link::Session drip;
    Vec<UInt8> ring;
    for(Size i = 0; i < wire.size(); ++i)
    {
        ring.push_back(wire[i]);
        const Size used = link::ingestBytes(drip, ring.data(), ring.size(), 1000);
        ring.erase(ring.begin(), ring.begin() + static_cast<ISize>(used));
    }

    check(ring.empty(), "nothing is left over when the last byte lands");
    check(drip.haveCamera, "the camera frame is reassembled");
    check(drip.camera.data == jpegBytes, "byte for byte, across every split");
    check(drip.revIndex == 4u, "and the scan after it still parses");

    // THE ONE THAT MATTERS. The JPEG contains `42 57` four times, which is the
    // frame magic. A single resynced byte here would mean the reader had gone
    // looking for structure inside a payload it was already told the length of.
    check(drip.resyncBytes == 0u, "with no resync inside the picture");
}

static Void testCameraDecodes()
{
    std::printf("\n-- and it is a picture stb_image can actually read --\n");

    // The thing a hand-built frame cannot prove on its own: that what came off
    // the wire is a real JPEG, and that the decoder this viewer ships turns it
    // into the pixels the camera saw.
    const Vec<UInt8> jpegBytes(TINY_JPEG.begin(), TINY_JPEG.end());
    Vec<UInt8> wire;
    static_cast<Void>(pushCamera(wire, 11, 1000000, jpegBytes));

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));

    const Opt<link::CameraShot> shot = s.cameraShot(1000);
    check(shot.has_value(), "a fresh camera frame is there to draw");
    if(!shot.has_value())
    {
        return;
    }

    jpeg::Picture pic;
    Str why;
    const Bool ok = jpeg::decode(shot->bytes.data(), shot->bytes.size(), &pic, &why);
    check(ok, "the JPEG decodes");
    if(!ok)
    {
        std::printf("        %s\n", why.c_str());
        return;
    }

    check(pic.width == 16 && pic.height == 12, "to the size the frame claimed");
    check(pic.rgba.size() == 16u * 12u * 4u, "with four bytes a pixel, RGBA");

    // Four quadrants, sampled well inside each, so a swapped row or column
    // order is a failure rather than a rounding difference.
    check(nearByte(channelAt(pic, 4, 3, 0), 203), "top-left is red");
    check(nearByte(channelAt(pic, 12, 3, 1), 199), "top-right is green");
    check(nearByte(channelAt(pic, 4, 9, 2), 205), "bottom-left is blue");
    check(nearByte(channelAt(pic, 12, 9, 0), 231), "bottom-right is near white");

    // A JPEG has no alpha to read, so reqComp 4 must synthesise an opaque one.
    // A 0 here would upload a fully transparent texture - a window that is
    // empty for a reason nobody would think to look for.
    check(channelAt(pic, 8, 6, 3) == 255u, "and every pixel is opaque");

    // Refusals are sentences, not silence.
    jpeg::Picture bad;
    Str badWhy;
    const Array<UInt8, 4> notAPicture = { 0x00, 0x01, 0x02, 0x03 };
    const Bool refused =
        jpeg::decode(notAPicture.data(), notAPicture.size(), &bad, &badWhy);
    check(!refused, "bytes that are not a picture are refused");
    check(!badWhy.empty(), "with a sentence saying so");
    check(bad.rgba.empty(), "and nothing half-decoded to draw");
}

static Void testCameraStaleness()
{
    std::printf("\n-- a camera frame too old to draw is not drawn --\n");

    const Vec<UInt8> jpegBytes(TINY_JPEG.begin(), TINY_JPEG.end());
    Vec<UInt8> wire;
    static_cast<Void>(pushCamera(wire, 1, 0, jpegBytes));

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));

    check(s.cameraShot(1000).has_value(), "a fresh frame is there to draw");
    check(!s.cameraShot(1000)->stale, "and is not marked stale");
    check(s.cameraShot(1401).has_value(), "at 401 ms it is still drawable");
    check(s.cameraShot(1401)->stale, "but is marked stale");
    check(s.cameraShot(1401)->ageMs == 401, "with its age");

    // Past 1500 ms there is NO PICTURE AT ALL. A photograph of a corridor is
    // equally convincing whether it was taken now or forty seconds ago - there
    // is nothing in the image for a person to read the age off - which is why
    // the band above stale is absence rather than a dimmer picture.
    check(!s.cameraShot(2501).has_value(), "past 1500 ms there is nothing to draw");

    // While the session still knows one arrived, so the window can say "no
    // camera frame for 3.2 s" instead of "not subscribed".
    check(s.haveCamera, "while the session still knows one arrived");

    check(!s.cameraShot(900).has_value(), "a clock that went backwards reads as gone");
}

static Void testCameraGaps()
{
    std::printf("\n-- camera gaps are counted, never smoothed --\n");

    const Vec<UInt8> jpegBytes(TINY_JPEG.begin(), TINY_JPEG.end());
    Vec<UInt8> wire;
    static_cast<Void>(pushCamera(wire, 90, 1000000, jpegBytes));
    static_cast<Void>(pushCamera(wire, 95, 1400000, jpegBytes));

    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));
    check(s.missedCameraFrames == 4u, "four missing frames are counted");
    checkStr(s.cameraGapText, "frames 91-94 missing", "and named exactly");
    check(s.cameraFrames == 2u, "while the two that arrived are counted too");
    check(s.camera.frameIndex == 95u, "and the newest is the one kept");
}

static Void testCameraRefusal()
{
    std::printf("\n-- why there is no picture, in the board's own words --\n");

    link::Session s;
    check(!s.haveCameraNote, "nothing is claimed before the board says anything");

    Vec<UInt8> wire;
    static_cast<Void>(
        pushEvent(wire, "camera busy - the phone dashboard holds /dev/video0", 0)
    );
    static_cast<Void>(feed(s, wire, 1000));

    check(s.haveCameraNote, "an EVENT about the camera is kept where the window can show it");
    checkStr(
        s.cameraNoteText,
        "camera busy - the phone dashboard holds /dev/video0",
        "verbatim, because the sentence is the part a person can act on"
    );
    check(s.notes.size() == 1, "and it is still an ordinary note as well");

    // An EVENT about something else must not be dressed up as a camera
    // refusal: an empty window blaming the wrong subsystem is worse than an
    // empty window.
    Vec<UInt8> other;
    static_cast<Void>(pushEvent(other, "lidar timeout - no revolution in 200 ms", 0));
    static_cast<Void>(feed(s, other, 1010));
    checkStr(
        s.cameraNoteText,
        "camera busy - the phone dashboard holds /dev/video0",
        "and an unrelated EVENT does not replace it"
    );
}

static Void testSubscriptionMask()
{
    std::printf("\n-- the mask, which is why any of this arrives at all --\n");

    // bit = tag - 0x10, settled in firmware/pilot/src/viewfeed.cxx. CAMERA is
    // 0x20, so it is bit 16 - NOT bit 0, which is what the naive
    // `1u << (tag & 0x1F)` gives it, and which is the same bug that puts
    // DECIDE and SCHEMA on one bit.
    check(link::typeBit(bibowire::Type::TYPE_SCAN) == 1u, "SCAN is bit 0");
    check(link::typeBit(bibowire::Type::TYPE_DECIDE) == 2u, "DECIDE is bit 1");
    check(link::typeBit(bibowire::Type::TYPE_CAMERA) == 65536u, "CAMERA is bit 16");
    check(link::typeBit(bibowire::Type::TYPE_SCHEMA) == 0u, "SCHEMA has no bit");
    check(
        link::typeBit(bibowire::Type::TYPE_DECIDE) != link::typeBit(bibowire::Type::TYPE_SCHEMA),
        "so DECIDE and SCHEMA cannot collide"
    );

    const UInt32 without = link::subscriptionMask(false);
    const UInt32 with = link::subscriptionMask(true);

    // A ZERO MASK MEANS EVERYTHING to the board, so the mask that switches the
    // camera off must never be 0 - it would ask for MORE than it started with,
    // and the only symptom would be a bandwidth figure nobody is watching.
    check(without != 0u, "the no-camera mask is never zero");
    check((without & 65536u) == 0u, "and does not claim the camera");
    check((with & 65536u) != 0u, "while the camera mask does");
    check(with == (without | 65536u), "and differs in exactly that one bit");

    // Everything this viewer draws survives turning the camera off.
    check((without & link::typeBit(bibowire::Type::TYPE_SCAN)) != 0u, "the scan survives");
    check((without & link::typeBit(bibowire::Type::TYPE_EVENT)) != 0u, "so do the sentences");
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
    testCameraFrames();
    testCameraByteAtATime();
    testCameraDecodes();
    testCameraStaleness();
    testCameraGaps();
    testCameraRefusal();
    testSubscriptionMask();

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
