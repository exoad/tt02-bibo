// The viewer's bibowire client, tested without a board.
//
//   tools\test.bat link run
//
// link.cxx's Session is pure, so this file drives it with hand-built frames,
// built with firmware/pilot/src/bibowire.cxx rather than restating what that
// codec's own suite proves. Not covered here:
//   - anything needing a socket and a peer: connect, resolve, CONNECT_MS, the
//     socket options, the UDP bind and filter, the select and reconnect loops;
//   - CONTROL's socket half: sendControl, sendto, the TCP fallback and its latch,
//     the reverse-path probe. The deadman, arming and REFUSE_MODE are held only
//     as shapes matched to docs/bibowire.md section 6;
//   - the heading arrow and bending guides as drawn (their signs are tested);
//   - the camera's socket half: the subscription handshake, and why a real
//     camera drops out (decode, pixels and gap classification are tested);
//   - the texture upload and the window, which need a D3D11 device;
//   - the trim pane as drawn and the board's half of COMMAND, so "refused while
//     armed" is checked only in shape;
//   - the alignment overlays, drawn in window space by camera.cxx;
//   - the settings file on disk: the atomic save never runs, so the suite leaves
//     no files behind.
#include "shared.hxx"

#include "bibowire.hxx"
#include "link.hxx"
#include "jpeg.hxx"
#include "carmesh.hxx"
#include "trail.hxx"
#include "orient.hxx"
#include "trim.hxx"
#include "drive.hxx"
#include "settings.hxx"

// For bendAt and pctToUnit. camera.hxx names no ImGui or D3D type, so this links
// nothing graphical.
#include "camera.hxx"
#include "scene.hxx"

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
    // One point per quadrant, so a swapped sine and cosine fail. The fifth is a
    // NO RETURN and must not become a point.
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
    m.wifiName = "FieldPhone";
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeBoard(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_BOARD, body.data(), n);
}

static Size pushEvent(Vec<UInt8>& out, const Str& text, UInt16 dropped, UInt8 code = 7)
{
    bibowire::Event m;
    m.tMonoUs = 900000;
    m.severity = bibowire::Severity::SEVERITY_WARN;
    m.code = code;
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

// A WELCOME with every field chosen by the caller; pushWelcome fixes `accepted`,
// the field the control tests are about.
static Size pushWelcomeAs(Vec<UInt8>& out, const bibowire::Welcome& m)
{
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeWelcome(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_WELCOME, body.data(), n);
}

static Size pushCmdAck(Vec<UInt8>& out, const bibowire::CmdAck& m)
{
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeCmdAck(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_CMDACK, body.data(), n);
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

// A type this build has no name for; 0x7E is in no tag space.
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

// A real, decodable 16x12 JPEG, so the shipped decoder is tested and not just
// the framing. Its COM segment holds hostile bytes: `ff d8 ff` runs (what an
// embedded EXIF thumbnail looks like) and `42 57`, bibowire's frame magic, four
// times. A reader that searched a payload for the magic instead of trusting
// byteLen would resync mid-picture (testCameraByteAtATime).
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

static Size pushTags(Vec<UInt8>& out, UInt32 index, UInt16 id, Int16 x0Deci, Int16 y0Deci)
{
    bibowire::Tags m;
    m.tMonoUs = static_cast<UInt64>(index) * 1000u;
    m.frameIndex = index;
    m.width = 640;
    m.height = 480;
    m.detectUs = 5300;
    bibowire::Tag one;
    one.id = id;
    one.marginMilli = 150000;
    one.corners[0] = bibowire::TagCorner{ x0Deci, y0Deci };
    one.corners[1] = bibowire::TagCorner{ static_cast<Int16>(x0Deci + 400), y0Deci };
    const Int16 x1Deci = static_cast<Int16>(x0Deci + 400);
    const Int16 y1Deci = static_cast<Int16>(y0Deci + 400);
    one.corners[2] = bibowire::TagCorner{ x1Deci, y1Deci };
    one.corners[3] = bibowire::TagCorner{ x0Deci, static_cast<Int16>(y0Deci + 400) };
    one.rangeMm = 1500;
    one.bearingCdeg = 750;
    m.flags = bibowire::TAGS_FLAG_CALIBRATED;
    m.tags.push_back(one);
    Vec<UInt8> body(256, 0);
    const Size n = bibowire::writeTags(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_TAGS, body.data(), n);
}

[[nodiscard]] static UInt8 channelAt(const jpeg::Picture& pic, Int32 x, Int32 y, Int32 ch)
{
    const Size row = static_cast<Size>(y) * static_cast<Size>(pic.width);
    const Size at = ((row + static_cast<Size>(x)) * 4u) + static_cast<Size>(ch);
    return at < pic.rgba.size() ? pic.rgba[at] : 0u;
}

// JPEG is lossy, so the tolerance is real. The expected values are what PIL
// decoded the same bytes to: two independent decoders compared.
[[nodiscard]] static Bool nearByte(UInt8 got, Int32 want)
{
    const Int32 delta = static_cast<Int32>(got) - want;
    return delta > -24 && delta < 24;
}

static Size feed(link::Session& s, const Vec<UInt8>& bytes, Int64 nowMs)
{
    return link::ingestBytes(s, bytes.data(), bytes.size(), nowMs);
}

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
    // Four, not five: distMm == 0 is NO RETURN, and points at the sensor would
    // read as an obstacle around the car.
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
    // One byte per call must give the identical answer: NEED_MORE consumes
    // nothing, so a partial frame is never a short message.
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
    // Seven bytes of junk that spell the magic, so the resync must reject a
    // false lock.
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
    // A reconnect drops the connection state and keeps the bootId, which the
    // comparison below needs.
    link::clearSession(s);
    check(!s.haveScan && !s.haveDecide, "a reconnect resumes nothing");
    check(s.haveBootId && s.bootId == 1000u, "but the bootId survives the reconnect");
    // The same boot reconnects.
    Vec<UInt8> same;
    static_cast<Void>(pushWelcome(same, 1000, bibowire::PROTO_MAJOR));
    static_cast<Void>(pushScan(same, scanOf(42, 1100000)));
    static_cast<Void>(feed(s, same, 2000));
    check(s.notes.empty(), "the same bootId is not a restart and says nothing");
    check(s.haveScan, "and the picture rebuilds normally");
    // The pilot restarts.
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
    // revIndex 0 is the BLIND tick: no revolution behind it by definition, which
    // differs from naming one we missed.
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
    check(!s.revolution(2501).has_value(), "past 1500 ms there is nothing to draw");
    check(!s.decision(2501).has_value(), "and no decision either");
    check(!s.boardState(2501).has_value(), "and no board state");
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
    // Whether 600 ms reads stale is the measured band's call: the feed's one
    // interval was 600 ms, which earns a 900 ms band.
    check(!rev->stale, "600 ms is not stale for a feed whose measured gap IS 600");
    check(rev->staleAtMs == 900, "the band that one 600 ms interval earned is 900 ms");
    const Opt<link::Revolution> later = s.revolution(2500);
    check(later.has_value(), "the same revolution is still drawable at 1400 ms");
    check(later.has_value() && later->stale, "and past the band it does read stale");
}

static Void testCadenceBand()
{
    std::printf("\n-- the staleness band is measured, not assumed --\n");
    link::Cadence fresh;
    check(link::worstGapMs(fresh) == 0, "an unmeasured feed reports no gap");
    check(link::staleBandMs(fresh) == link::FRESH_MS, "and is held to FRESH_MS");
    link::noteArrival(fresh, 1000);
    check(link::worstGapMs(fresh) == 0, "the first frame is not a gap");
    check(link::staleBandMs(fresh) == link::FRESH_MS, "so the band has not moved");
    // A fast feed must not tighten the band below FRESH_MS.
    link::Cadence quick;
    for(Int32 i = 0; i < 20; ++i)
    {
        link::noteArrival(quick, 1000 + (i * 20));
    }
    check(link::worstGapMs(quick) == 20, "a 50 Hz feed measures a 20 ms gap");
    check(link::staleBandMs(quick) == link::FRESH_MS, "and is STILL held to FRESH_MS, never less");
    // Two frames a second, the board's default camera rate.
    link::Cadence slow;
    for(Int32 i = 0; i < 20; ++i)
    {
        link::noteArrival(slow, 1000 + (i * 500));
    }
    check(link::worstGapMs(slow) == 500, "a 2 fps feed measures a 500 ms gap");
    check(link::staleBandMs(slow) == 750, "and earns a 750 ms band");
    check(link::staleBandMs(slow) > 500, "which is wider than the interval it delivers at");
    // However bad the feed, stale stays below GONE_MS, so nothing goes from live
    // to absent without a stale band between.
    link::Cadence awful;
    for(Int32 i = 0; i < 8; ++i)
    {
        link::noteArrival(awful, 1000 + (i * 5000));
    }
    check(
        link::staleBandMs(awful) == link::STALE_CEIL_MS,
        "a dreadful feed is capped at the ceiling"
    );
    check(
        link::staleBandMs(awful) < link::GONE_MS,
        "so stale is always passed THROUGH on the way to gone"
    );
    // The window slides, so a past stall stops widening the band.
    link::Cadence passing;
    link::noteArrival(passing, 0);
    link::noteArrival(passing, 900);
    check(link::worstGapMs(passing) == 900, "a stall is measured while it is recent");
    Int64 at = 900;
    for(Size i = 0; i < link::CADENCE_SAMPLES; ++i)
    {
        at += 100;
        link::noteArrival(passing, at);
    }
    check(link::worstGapMs(passing) == 100, "and leaves the window once it has scrolled out");
    check(link::staleBandMs(passing) == link::FRESH_MS, "so the band narrows again");
}

static Void testCameraKeepsItsPromise()
{
    std::printf("\n-- a camera delivering what it promised is never stale --\n");
    const Vec<UInt8> jpegBytes(TINY_JPEG.begin(), TINY_JPEG.end());
    link::Session s;
    // Twelve frames at the board's 2 fps default, each on time. tMonoUs is 0, so
    // ages come from local arrival and only the band is tested.
    Int64 at = 1000;
    for(UInt32 i = 0; i < 12u; ++i)
    {
        Vec<UInt8> wire;
        static_cast<Void>(pushCamera(wire, i + 1u, 0, jpegBytes));
        static_cast<Void>(feed(s, wire, at));
        at += 500;
    }
    const Int64 last = at - 500;
    const Opt<link::CameraShot> shot = s.cameraShot(last + 499);
    check(shot.has_value(), "the newest picture is there to draw");
    check(shot.has_value() && !shot->stale, "and 499 ms after it arrived it is NOT stale");
    check(shot.has_value() && shot->staleAtMs == 750, "because the band it earned is 750 ms");
    check(shot.has_value() && shot->worstGapMs == 500, "from its measured 500 ms cadence");
    // A camera that has stopped still goes stale, then gone.
    const Opt<link::CameraShot> aging = s.cameraShot(last + 800);
    check(aging.has_value(), "a stopped camera is still drawable at 800 ms");
    check(aging.has_value() && aging->stale, "but it IS stale past the band it earned");
    check(!s.cameraShot(last + 1600).has_value(), "and past GONE_MS there is no picture at all");
}

static Void testCameraRate()
{
    std::printf("\n-- the rate this viewer asks the board for --\n");
    link::Client c;
    check(link::cameraFpsWanted(c) == 0, "a fresh client asks for no particular rate");
    link::wantCameraFps(c, 10);
    check(link::cameraFpsWanted(c) == 10, "and carries what it was given");
    // The board owns the ceiling; the viewer must not be able to ask past it.
    link::wantCameraFps(c, 900);
    check(
        link::cameraFpsWanted(c) == static_cast<Int32>(bibowire::CAM_FPS_MAX),
        "a request above the board's ceiling is clamped to it"
    );
    link::wantCameraFps(c, -5);
    check(link::cameraFpsWanted(c) == 0, "and a negative rate is no request at all");
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
    // +-JITTER_PERCENT, and the jitter must actually move: always returning the
    // base would pass the bounds and keep clients in lockstep.
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
    Vec<UInt8> stray;
    static_cast<Void>(pushPong(stray, 999999u, 1000000));
    static_cast<Void>(feed(s, stray, 1200));
    const Opt<Int64> after = s.rttMs();
    check(after.has_value() && *after == 100, "an unmatched token is ignored");
    check(s.rtts.size() == 1, "and adds no sample");
    // Twenty round trips, one of them fast: the minimum is the path.
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
    // The offset a round trip implies can only make the picture older.
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
    // it is 150, and the larger wins.
    const Opt<link::Revolution> rev = aged.revolution(1200);
    check(rev.has_value() && rev->ageMs == 150, "so the age is the larger of the two");
}

static Void testSilenceInput()
{
    std::printf("\n-- every frame feeds the silence watchdog --\n");
    // Including an unknown type, or a live board would be redialled the day it
    // sends a new one.
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
    // A short array would zero-fill silently, so the sample must end in EOI.
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
    // The payload holds frame magic and ff d8 ff runs, so a reader that scanned
    // for either instead of trusting byteLen would truncate it.
    check(s.camera.data.size() == jpegBytes.size(), "the JPEG is the length it was sent at");
    check(s.camera.data == jpegBytes, "and is byte-for-byte what the board sent");
    // The board keeps no subscription across a session, so neither may this.
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
    // The JPEG contains the frame magic, so one resynced byte would mean the
    // reader searched a payload whose length it already had.
    check(drip.resyncBytes == 0u, "with no resync inside the picture");
}

static Void testCameraDecodes()
{
    std::printf("\n-- and it is a picture stb_image can actually read --\n");
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
    // Sampled well inside each quadrant, so a swapped row or column order fails.
    check(nearByte(channelAt(pic, 4, 3, 0), 203), "top-left is red");
    check(nearByte(channelAt(pic, 12, 3, 1), 199), "top-right is green");
    check(nearByte(channelAt(pic, 4, 9, 2), 205), "bottom-left is blue");
    check(nearByte(channelAt(pic, 12, 9, 0), 231), "bottom-right is near white");
    // A JPEG has no alpha, so reqComp 4 must synthesise an opaque one; 0 would
    // upload an invisible texture.
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
    check(!s.cameraShot(2501).has_value(), "past 1500 ms there is nothing to draw");
    // So the window can say how long since the last frame, not "not subscribed".
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
        pushEvent(wire, "camera busy - another pilot holds /dev/video0", 0)
    );
    static_cast<Void>(feed(s, wire, 1000));
    check(s.haveCameraNote, "an EVENT about the camera is kept where the window can show it");
    checkStr(
        s.cameraNoteText,
        "camera busy - another pilot holds /dev/video0",
        "verbatim, because the sentence is the part a person can act on"
    );
    check(s.notes.size() == 1, "and it is still an ordinary note as well");
    // An unrelated EVENT must not be shown as the camera's reason.
    Vec<UInt8> other;
    static_cast<Void>(pushEvent(other, "lidar timeout - no revolution in 200 ms", 0));
    static_cast<Void>(feed(s, other, 1010));
    checkStr(
        s.cameraNoteText,
        "camera busy - another pilot holds /dev/video0",
        "and an unrelated EVENT does not replace it"
    );
}

static Void testIdleTestIntent()
{
    std::printf("\n-- the idle test rides the stream only with the enable --\n");
    driveview::View v;
    v.throttleCapMilli = 300;
    driveview::Keys w;
    w.forward = true;
    v.idleTest = true;
    const link::Intent off = driveview::intentFrom(w, v);
    check(
        (off.buttons & bibowire::BUTTON_IDLE_TEST) == 0u,
        "without the enable there is no idle test on the wire"
    );
    v.enabled = true;
    const link::Intent on = driveview::intentFrom(w, v);
    check((on.buttons & bibowire::BUTTON_IDLE_TEST) != 0u, "with it, the bit is set");
    check((on.buttons & bibowire::BUTTON_ENABLE) != 0u, "beside ENABLE");
    check(on.throttleMilli == 0, "and W counts for nothing while the motor is held at idle");
    link::ControlStamp at;
    at.sessionId = 0x1D1E7E57u;
    at.seq = 1u;
    at.armEpoch = 1u;
    const bibowire::Control m = link::buildControl(on, at);
    check((m.buttons & bibowire::BUTTON_IDLE_TEST) != 0u, "and it survives onto the datagram");
    v.idleTest = false;
    check(driveview::intentFrom(w, v).throttleMilli == 300, "untick it and W is W again");
}

static Void testBoardTrim()
{
    std::printf("\n-- the trim the board has saved, taken into the pane --\n");
    const Char* FULL = "SERVOLIMITS 1200 1700; ESCLIMITS 1564 1700; SERVOTRIM 1470; SLEW STEER 22; SLEW THROTTLE 14";
    link::Session s;
    check(!s.haveBoardTrim, "nothing is claimed before the board says anything");
    Vec<UInt8> wire;
    static_cast<Void>(pushEvent(wire, FULL, 0, bibowire::EVENT_CODE_TRIM));
    static_cast<Void>(feed(s, wire, 1000));
    check(s.haveBoardTrim, "an EVENT under EVENT_CODE_TRIM is kept as the board's saved trim");
    checkStr(s.boardTrimText, FULL, "verbatim");
    check(
        s.boardTrimCount == 1u && s.boardTrimAtMs == 1000,
        "counted and stamped, so the pane takes it once"
    );
    check(
        s.notes.size() == 1 && s.notes[0].text.rfind("trim saved on the board: ", 0) == 0,
        "and listed in words as a note"
    );
    // The code decides, not the text.
    Vec<UInt8> other;
    static_cast<Void>(pushEvent(other, "SERVOTRIM 1500", 0));
    static_cast<Void>(feed(s, other, 1010));
    checkStr(
        s.boardTrimText,
        FULL,
        "an EVENT under another code is not taken as trim, whatever its text"
    );
    check(s.boardTrimCount == 1u, "and is not counted as a report");
    Vec<UInt8> none;
    static_cast<Void>(pushEvent(none, "", 0, bibowire::EVENT_CODE_TRIM));
    static_cast<Void>(feed(s, none, 1020));
    check(
        s.haveBoardTrim && s.boardTrimText.empty(),
        "an empty report says the board has nothing saved"
    );
    check(s.boardTrimCount == 2u, "and is a report of its own");
    trimview::View v;
    check(trimview::adoptReport(v, FULL) == 5, "all five settings are taken");
    check(
        v.steerMinUs == 1200 && v.steerMaxUs == 1700 && v.steerTrimUs == 1470,
        "the steering limits and centre"
    );
    check(v.escMinUs == 1564 && v.escMaxUs == 1700, "the throttle limits");
    check(v.steerSlewUs == 22 && v.throttleSlewUs == 14, "and both rates, each on its own axis");
    trimview::View rev;
    check(
        trimview::adoptReport(rev, "ESCLIMITS 1541 1700; ESCREVERSE 1350") == 2,
        "a reverse limit is taken beside the throttle limits"
    );
    check(rev.escReverseUs == 1350, "at the pulse the board saved");
    trimview::View revHigh;
    check(
        trimview::adoptReport(revHigh, "ESCREVERSE 1600") == 1,
        "a reverse limit above neutral is still read"
    );
    check(
        revHigh.escReverseUs == trimview::ESC_REVERSE_DEFAULT,
        "and settled back to off, never a forward pulse"
    );
    trimview::View part;
    check(
        trimview::adoptReport(part, "SERVOTRIM 1490") == 1,
        "a board that saved only a centre gives one setting"
    );
    check(
        part.steerTrimUs == 1490
            && part.steerMinUs == trimview::STEER_MIN_DEFAULT
            && part.escMaxUs == trimview::ESC_MAX_DEFAULT
            && part.throttleSlewUs == trimview::THROTTLE_SLEW_DEFAULT,
        "which moves the centre and leaves every other slider where it was"
    );
    trimview::View junk;
    const Int32 junkTaken = trimview::adoptReport(
        junk,
        "SERVOTRIM 1490x; ESCLIMITS 1564; SLEW SIDEWAYS 9; ESC ARM; SERVOLIMITS 1200 1700 1800; SERVOTRIM -5"
    );
    check(junkTaken == 0, "a malformed line is skipped whole, never half-read");
    check(
        junk.steerTrimUs == trimview::STEER_CENTRE_DEFAULT
            && junk.escMinUs == trimview::ESC_MIN_DEFAULT
            && junk.steerSlewUs == trimview::STEER_SLEW_DEFAULT,
        "so nothing moved"
    );
    trimview::View wild;
    check(
        trimview::adoptReport(wild, "ESCLIMITS 900 3000") == 1,
        "limits past the hard range are taken"
    );
    check(
        wild.escMinUs == static_cast<Int32>(bibowire::ESC_US_HARD_MIN)
            && wild.escMaxUs == static_cast<Int32>(bibowire::ESC_US_HARD_MAX),
        "and settled to the sliders' own range"
    );
    trimview::View blank;
    check(trimview::adoptReport(blank, "") == 0, "an empty report takes nothing");
    check(
        blank.steerTrimUs == trimview::STEER_CENTRE_DEFAULT,
        "and leaves the laptop's copy standing"
    );
}

static Void testSubscriptionMask()
{
    std::printf("\n-- the mask, which is why any of this arrives at all --\n");
    const UInt32 without = link::subscriptionMask(false);
    const UInt32 with = link::subscriptionMask(true);
    check(without != 0u, "the no-camera mask is never zero");
    check((without & 65536u) == 0u, "and does not claim the camera");
    check((with & 65536u) != 0u, "while the camera mask does");
    check(with == (without | 65536u), "and differs in exactly that one bit");
    check((without & bibowire::typeBit(bibowire::Type::TYPE_SCAN)) != 0u, "the scan survives");
    check((without & bibowire::typeBit(bibowire::Type::TYPE_EVENT)) != 0u, "so do the sentences");
}

static Void checkUv(const orient::Uv& got, Float32 u, Float32 v, const Char* what)
{
    check(near(got.u, u) && near(got.v, v), what);
}

[[nodiscard]] static Bool sameCorners(const Array<orient::Uv, 4>& a, const Array<orient::Uv, 4>& b)
{
    for(Size i = 0; i < 4u; ++i)
    {
        if(!near(a[i].u, b[i].u) || !near(a[i].v, b[i].v))
        {
            return false;
        }
    }
    return true;
}

static Void testOrientation()
{
    std::printf("\n-- rotating and flipping the camera picture --\n");
    const Array<orient::Uv, 4> flat = orient::cornerUvs(0, false, false);
    checkUv(flat[0], 0.0f, 0.0f, "unturned, the top-left is the source's top-left");
    checkUv(flat[2], 1.0f, 1.0f, "and the bottom-right is the source's bottom-right");
    check(!orient::sideways(0), "and the picture is not on its side");
    // A quarter turn clockwise, which ImGui::Image's two opposite uv corners
    // cannot express.
    const Array<orient::Uv, 4> cw = orient::cornerUvs(1, false, false);
    checkUv(cw[0], 0.0f, 1.0f, "turned 90, the top-left samples the source's BOTTOM-left");
    checkUv(cw[1], 0.0f, 0.0f, "the top-right samples the source's top-left");
    checkUv(cw[2], 1.0f, 0.0f, "the bottom-right samples the source's top-right");
    checkUv(cw[3], 1.0f, 1.0f, "and the bottom-left samples the source's bottom-right");
    check(orient::sideways(1), "and 90 degrees IS on its side, so the fit swaps");
    // Half a turn is both axes mirrored.
    const Array<orient::Uv, 4> half = orient::cornerUvs(2, false, false);
    checkUv(half[0], 1.0f, 1.0f, "turned 180, the top-left samples the far corner");
    check(!orient::sideways(2), "and 180 is not on its side");
    check(orient::sideways(3), "while 270 is");
    // The flips are in source space, so they mean the same at any angle.
    const Array<orient::Uv, 4> mirrored = orient::cornerUvs(0, true, false);
    checkUv(mirrored[0], 1.0f, 0.0f, "flipped horizontally, the top-left samples the top-right");
    const Array<orient::Uv, 4> upended = orient::cornerUvs(0, false, true);
    checkUv(upended[0], 0.0f, 1.0f, "flipped vertically, the top-left samples the bottom-left");
    check(
        sameCorners(orient::cornerUvs(0, true, true), half),
        "and flipping BOTH axes is the same picture as turning it 180"
    );
    // place() is cornerUvs run forward: the source corner a destination corner
    // samples lands on that destination corner.
    const orient::Place cwBl = orient::place(1, false, false, 0.0f, 1.0f);
    check(
        cwBl.x < 0.001f && cwBl.y < 0.001f,
        "turned 90, the source's bottom-left lands at the top-left"
    );
    const orient::Place cwTl = orient::place(1, false, false, 0.0f, 0.0f);
    check(cwTl.x > 0.999f && cwTl.y < 0.001f, "and the source's top-left at the top-right");
    const orient::Place halfTl = orient::place(2, false, false, 0.0f, 0.0f);
    check(
        halfTl.x > 0.999f && halfTl.y > 0.999f,
        "turned 180, the top-left lands at the bottom-right"
    );
    const orient::Place ccwTl = orient::place(3, false, false, 0.0f, 0.0f);
    check(ccwTl.x < 0.001f && ccwTl.y > 0.999f, "turned 270, at the bottom-left");
    const orient::Place mirroredTl = orient::place(0, true, false, 0.0f, 0.0f);
    check(mirroredTl.x > 0.999f && mirroredTl.y < 0.001f, "flipped horizontally, at the top-right");
    const orient::Place mid = orient::place(-3, true, true, 0.25f, 0.5f);
    check(
        mid.x > 0.499f && mid.x < 0.501f && mid.y > 0.749f && mid.y < 0.751f,
        "and an interior point turns with the picture, negatives folded"
    );
    // Turns fold rather than being refused: a rotate-left button passes negatives.
    check(
        sameCorners(orient::cornerUvs(-1, false, false), orient::cornerUvs(3, false, false)),
        "a negative turn folds to the same corners as 3"
    );
    check(sameCorners(orient::cornerUvs(5, false, false), cw), "and 5 quarter turns is one");
    check(orient::sideways(-1), "a negative turn still knows it is on its side");
}

static Void testCommandIds()
{
    std::printf("\n-- a cmdId is never 0, and never repeats --\n");
    link::Client c;
    check(c.pending.empty(), "a fresh client has nothing queued");
    check(link::commandsDropped(c) == 0u, "and has dropped nothing");
    link::sendCommand(c, bibowire::Verb::VERB_SET_SERVO_TRIM, 0, 1480, 0);
    link::sendCommand(c, bibowire::Verb::VERB_SET_SLEW, bibowire::SLEW_AXIS_STEER, 8, 0);
    link::sendCommand(c, bibowire::Verb::VERB_SET_ESC_LIMITS, 0, 1541, 1600);
    check(c.pending.size() == 3, "three deliberate acts are three queued commands");
    if(c.pending.size() != 3)
    {
        return;
    }
    check(c.pending[0].cmdId != 0u, "the first cmdId is not 0");
    check(c.pending[1].cmdId != 0u, "nor the second");
    check(c.pending[2].cmdId != 0u, "nor the third");
    check(c.pending[0].cmdId == 1u, "the counter starts at 1");
    check(c.pending[1].cmdId == 2u, "and the second is 2");
    check(c.pending[2].cmdId == 3u, "and the third is 3");
    // Also asserted as an ordering, not only as constants.
    check(c.pending[1].cmdId > c.pending[0].cmdId, "strictly increasing");
    check(c.pending[2].cmdId > c.pending[1].cmdId, "at every step");
    check(c.pending[0].verb == bibowire::Verb::VERB_SET_SERVO_TRIM, "the first verb survives");
    check(c.pending[2].verb == bibowire::Verb::VERB_SET_ESC_LIMITS, "and so does the third");
    check(c.pending[0].sessionId == 0u, "sessionId is left for the worker to stamp");
    check(c.pending[0].armEpoch == 0u, "and so is armEpoch");
}

static Void testCommandEncoding()
{
    std::printf("\n-- a queued command becomes a COMMAND frame, exactly --\n");
    link::Client c;
    link::sendCommand(c, bibowire::Verb::VERB_SET_SERVO_LIMITS, 0, 1230, 1660);
    check(c.pending.size() == 1, "one command is queued");
    if(c.pending.size() != 1)
    {
        return;
    }
    // Stamped as the worker does before it writes.
    bibowire::Command cmd = c.pending[0];
    cmd.sessionId = 0x51E55101u;
    cmd.armEpoch = 3;
    Array<UInt8, 64> body = {};
    const Size n = bibowire::writeCommand(cmd, body.data(), body.size());
    check(n != 0, "it encodes");
    if(n == 0)
    {
        return;
    }
    Vec<UInt8> wire;
    static_cast<Void>(framed(wire, bibowire::Type::TYPE_COMMAND, body.data(), n));
    bibowire::Frame f;
    Size used = 0;
    const bibowire::Take got = bibowire::take(wire.data(), wire.size(), &f, &used);
    check(got == bibowire::Take::TAKE_FRAME, "and comes back off the wire as a frame");
    check(f.head.type == bibowire::Type::TYPE_COMMAND, "tagged COMMAND, 0x41");
    if(got != bibowire::Take::TAKE_FRAME)
    {
        return;
    }
    bibowire::Command back;
    check(bibowire::readCommand(f.body, f.head.ver, &back), "and decodes");
    // Distinct values, so swapping arg1 (min) and arg2 (max) fails.
    check(back.verb == bibowire::Verb::VERB_SET_SERVO_LIMITS, "verb 9 survives");
    check(back.arg1 == 1230u, "min lands in arg1");
    check(back.arg2 == 1660u, "max lands in arg2");
    check(back.arg0 == 0u, "arg0 is unused by this verb and is zero");
    check(back.cmdId == 1u, "the cmdId survives");
    check(back.sessionId == 0x51E55101u, "and the session the worker stamped");
    check(back.armEpoch == 3u, "and the epoch");
    // SET_SLEW puts the axis in arg0 and the rate in arg1; a swap would apply a
    // throttle rate to the steering.
    link::Client s;
    link::sendCommand(s, bibowire::Verb::VERB_SET_SLEW, bibowire::SLEW_AXIS_THROTTLE, 12, 0);
    check(s.pending.size() == 1, "a slew command is queued");
    if(s.pending.size() != 1)
    {
        return;
    }
    Array<UInt8, 64> slewBody = {};
    const Size sn = bibowire::writeCommand(s.pending[0], slewBody.data(), slewBody.size());
    check(sn != 0, "it encodes");
    if(sn == 0)
    {
        return;
    }
    Vec<UInt8> slewWire;
    static_cast<Void>(framed(slewWire, bibowire::Type::TYPE_COMMAND, slewBody.data(), sn));
    bibowire::Frame sf;
    Size sUsed = 0;
    static_cast<Void>(bibowire::take(slewWire.data(), slewWire.size(), &sf, &sUsed));
    bibowire::Command slewBack;
    check(bibowire::readCommand(sf.body, sf.head.ver, &slewBack), "and decodes");
    check(slewBack.verb == bibowire::Verb::VERB_SET_SLEW, "verb 11 survives");
    check(slewBack.arg0 == bibowire::SLEW_AXIS_THROTTLE, "the AXIS is arg0");
    check(slewBack.arg0 == 2u, "which is 2 for the throttle");
    check(slewBack.arg1 == 12u, "and the rate is arg1");
    check(slewBack.arg0 != slewBack.arg1, "so axis and rate cannot have been swapped");
}

static Void testSlewArithmetic()
{
    std::printf("\n-- us per tick, into us per second, into a TIME --\n");
    // The Pico's tick is 20 ms, so 50 ticks a second.
    check(trimview::slewUsPerSec(8) == 400, "8 us a tick is 400 us a second");
    check(trimview::slewUsPerSec(1) == 50, "1 is 50");
    check(trimview::slewUsPerSec(200) == 10000, "and 200 is 10000");
    // 430 us is the steering travel, 1230 to 1660.
    check(trimview::crossCentis(430, 8) == 107, "430 us of travel at 8 is 1.07 s lock to lock");
    // The throttle's 59 us band - 1541 to 1600 - at the same rate.
    check(trimview::crossCentis(59, 8) == 14, "the 59 us throttle band at 8 is 0.14 s");
    // The slowest and fastest the protocol allows, across the steering's travel.
    check(trimview::crossCentis(430, 1) == 860, "at 1 us a tick the same travel takes 8.60 s");
    check(trimview::crossCentis(430, 200) == 4, "and at 200 it takes 0.04 s");
    // Span and rate are both small integers, so a swap compiles; these differ.
    check(
        trimview::crossCentis(430, 8) != trimview::crossCentis(8, 430),
        "span and rate are not interchangeable"
    );
    // No answer is -1, shown as a dash: a zero span would read as instant, and
    // a zero rate divides by zero.
    check(trimview::crossCentis(0, 8) == -1, "no travel to cross has no time");
    check(trimview::crossCentis(-5, 8) == -1, "nor does a crossed pair of limits");
    check(trimview::crossCentis(430, 0) == -1, "and a rate of zero never arrives");
    // The pane's defaults must match the firmware's committed values.
    check(trimview::STEER_MIN_DEFAULT == 1230, "the steering minimum mirrors cal.hxx");
    check(trimview::STEER_CENTRE_DEFAULT == 1480, "and the centre, which is not 1500");
    check(trimview::STEER_MAX_DEFAULT == 1660, "and the maximum");
    check(trimview::ESC_MIN_DEFAULT == 1541, "and the throttle's idle");
    check(trimview::ESC_MAX_DEFAULT == 1600, "and its full");
    // Inside the protocol's bounds, or the pane opens on a value the board refuses.
    check(
        trimview::STEER_MIN_DEFAULT >= static_cast<Int32>(bibowire::SERVO_US_HARD_MIN),
        "inside the servo floor"
    );
    check(
        trimview::STEER_MAX_DEFAULT <= static_cast<Int32>(bibowire::SERVO_US_HARD_MAX),
        "and the servo ceiling"
    );
    check(
        trimview::ESC_MIN_DEFAULT >= static_cast<Int32>(bibowire::ESC_US_HARD_MIN),
        "inside the ESC floor"
    );
    check(
        trimview::ESC_MAX_DEFAULT <= static_cast<Int32>(bibowire::ESC_US_HARD_MAX),
        "and the ESC ceiling"
    );
}

static Void testCmdAck()
{
    std::printf("\n-- what the board said, which is the whole point of a refusal --\n");
    link::Session s;
    check(!s.newestAck().has_value(), "before any answer there is nothing to show");
    // result 3, "not in this state", is what a tuning verb gets while armed.
    bibowire::CmdAck refused;
    refused.cmdId = 7;
    refused.verb = bibowire::Verb::VERB_SET_SERVO_LIMITS;
    refused.result = 3;
    refused.armEpoch = 3;
    refused.text = "refused - disarm before changing the servo limits";
    Vec<UInt8> wire;
    static_cast<Void>(pushCmdAck(wire, refused));
    static_cast<Void>(feed(s, wire, 1000));
    const Opt<link::Ack> got = s.newestAck();
    check(got.has_value(), "a CMDACK is kept");
    if(!got.has_value())
    {
        return;
    }
    check(got->ack.cmdId == 7u, "with the cmdId it answers");
    check(got->ack.result == 3u, "and its result");
    check(got->ack.verb == bibowire::Verb::VERB_SET_SERVO_LIMITS, "and the verb it answers");
    check(got->atMs == 1000, "and when it landed");
    checkStr(
        got->ack.text,
        "refused - disarm before changing the servo limits",
        "and its sentence, exactly as the board wrote it"
    );
    checkStr(Str(link::ackResultName(3)), "not in this state", "result 3 has a name");
    checkStr(Str(link::ackResultName(0)), "ok", "and so does 0");
    checkStr(
        Str(link::ackResultName(2)),
        "unknown verb",
        "and 2, for a board too old for these verbs"
    );
    bibowire::CmdAck ok;
    ok.cmdId = 8;
    ok.verb = bibowire::Verb::VERB_SET_SLEW;
    ok.result = 0;
    ok.text = "slew steer 8";
    Vec<UInt8> second;
    static_cast<Void>(pushCmdAck(second, ok));
    static_cast<Void>(feed(s, second, 1100));
    check(s.newestAck().has_value() && s.newestAck()->ack.cmdId == 8u, "the newest answer wins");
    check(s.acks.size() == 2, "while the one before it is still kept");
    for(UInt32 i = 0; i < 20u; ++i)
    {
        bibowire::CmdAck more;
        more.cmdId = 100u + i;
        more.result = 0;
        Vec<UInt8> bytes;
        static_cast<Void>(pushCmdAck(bytes, more));
        static_cast<Void>(feed(s, bytes, 1200));
    }
    check(s.acks.size() == link::MAX_ACKS, "the ack list is bounded");
    check(s.newestAck()->ack.cmdId == 119u, "and keeps the newest, not the first");
    link::clearSession(s);
    check(s.acks.empty(), "a reconnect carries no acks across");
    check(!s.newestAck().has_value(), "and has nothing to show");
}

static Void testControlSeq()
{
    std::printf("\n-- a CONTROL seq starts at 1, never repeats and is never 0 --\n");
    check(link::nextControlSeq(0u) == 1u, "the first seq of a session is 1, not 0");
    check(link::nextControlSeq(1u) == 2u, "then 2");
    check(link::nextControlSeq(41u) == 42u, "and it counts by one");
    // Also asserted as an ordering over a run, not only as constants.
    UInt32 seq = 0;
    Bool rising = true;
    Bool everZero = false;
    for(Int32 i = 0; i < 4000; ++i)
    {
        const UInt32 next = link::nextControlSeq(seq);
        if(next <= seq)
        {
            rising = false;
        }
        if(next == 0u)
        {
            everZero = true;
        }
        seq = next;
    }
    check(rising, "strictly increasing across four thousand datagrams");
    check(!everZero, "and never 0 - CTLSTATE's ackSeq uses 0 for none applied");
    check(seq == 4000u, "four thousand sends is seq 4000, so none were skipped");
    check(link::nextControlSeq(0xFFFFFFFFu) == 1u, "the wrap skips 0 and begins again at 1");
}

static Void testControlRoundTrip()
{
    std::printf("\n-- a CONTROL this viewer built, field by field --\n");
    link::Intent in;
    in.driving = true;
    in.steerMilli = -437;
    in.throttleMilli = 268;
    in.buttons = static_cast<UInt16>(bibowire::BUTTON_ENABLE | bibowire::BUTTON_MOTOR_WANTED);
    in.assumedMode = static_cast<UInt8>(bibowire::PilotMode::PILOT_MODE_LOOK);
    link::ControlStamp at;
    at.sessionId = 0x51E55101u;
    at.seq = 4242u;
    at.armEpoch = 7u;
    at.tMonoUs = 1234567890ull;
    const bibowire::Control m = link::buildControl(in, at);
    Array<UInt8, 64> body = {};
    const Size n = bibowire::writeControl(m, body.data(), body.size());
    check(n != 0, "it encodes");
    if(n == 0)
    {
        return;
    }
    Vec<UInt8> wire;
    static_cast<Void>(framed(wire, bibowire::Type::TYPE_CONTROL, body.data(), n));
    bibowire::Frame f;
    Size used = 0;
    const bibowire::Take got = bibowire::take(wire.data(), wire.size(), &f, &used);
    check(got == bibowire::Take::TAKE_FRAME, "and comes back off the wire as a frame");
    check(f.head.type == bibowire::Type::TYPE_CONTROL, "tagged CONTROL, 0x40");
    if(got != bibowire::Take::TAKE_FRAME)
    {
        return;
    }
    bibowire::Control back;
    check(bibowire::readControl(f.body, f.head.ver, &back), "and decodes");
    // Every field a different value, so a swapped pair fails. The two Int16s and
    // the two trailing UInt8s are also asserted against each other.
    check(back.sessionId == 0x51E55101u, "the session survives");
    check(back.seq == 4242u, "and the seq");
    check(back.tMonoUs == 1234567890ull, "and the viewer's own clock");
    check(back.steerMilli == -437, "steer lands in steerMilli, negative for left");
    check(back.throttleMilli == 268, "and throttle in throttleMilli");
    check(back.steerMilli != back.throttleMilli, "so the two i16 fields cannot have been swapped");
    check(back.buttons == 6u, "ENABLE and MOTOR_WANTED are bits 1 and 2");
    check(back.armEpoch == 7u, "the epoch the viewer believes");
    check(back.assumedMode == 1u, "and the mode it asserts");
    check(back.armEpoch != back.assumedMode, "so the two trailing u8s cannot have been swapped");
    check(back.sessionId != back.seq, "nor the two u32s");
}

static Void testDriveKeys()
{
    std::printf("\n-- what a key means, which is the whole of the driving --\n");
    const driveview::Keys none;
    driveview::Keys a;
    a.left = true;
    driveview::Keys d;
    d.right = true;
    driveview::Keys both;
    both.left = true;
    both.right = true;
    // The direction steerHeldStep moves the held steering (testSteerHeld).
    check(driveview::steerFrom(none) == 0, "no key is no push");
    check(driveview::steerFrom(a) == -1000, "A pushes toward full left");
    check(driveview::steerFrom(d) == 1000, "D pushes toward full right");
    check(driveview::steerFrom(a) != driveview::steerFrom(d), "and left is not right");
    // Both cancel, rather than the last one winning: a hand resting on A while
    // reaching for D must not turn the wheel.
    check(driveview::steerFrom(both) == 0, "A and D together cancel to no push");
    check(driveview::steerFrom(both) != driveview::steerFrom(a), "not the left one");
    check(driveview::steerFrom(both) != driveview::steerFrom(d), "and not the right one");
    driveview::Keys w;
    w.forward = true;
    driveview::Keys s;
    s.brake = true;
    driveview::Keys ws;
    ws.forward = true;
    ws.brake = true;
    check(driveview::throttleFrom(none, 300) == 0, "no key is no throttle");
    check(driveview::throttleFrom(w, 100) == 100, "W is the cap the operator set");
    check(driveview::throttleFrom(w, 300) == 300, "whatever that cap is");
    check(driveview::throttleFrom(w, 5000) == 1000, "and it is clamped to full scale");
    check(driveview::throttleFrom(w, 0) == 0, "a cap of zero is a key that does nothing");
    check(driveview::throttleFrom(w, -5) == 0, "and a negative cap is not reverse");
    // S is minus the cap, which this ESC takes as a brake and then, after a
    // return to neutral, as reverse.
    check(
        driveview::throttleFrom(s, 300) == -300,
        "S alone is minus the cap - brake, then reverse"
    );
    check(driveview::throttleFrom(s, 5000) == -1000, "clamped to full scale like W");
    check(driveview::throttleFrom(s, 0) == 0, "and with a cap of zero S is a plain stop");
    // S beats W: W is usually still held when someone reaches for the brake.
    check(driveview::throttleFrom(ws, 300) == -300, "and S BEATS W when both are down");
    check(
        driveview::throttleFrom(ws, 300) != driveview::throttleFrom(w, 300),
        "which is a different answer from W alone, so the precedence is real"
    );
}

// steerHeldStep at the default rate, named short so each case reads as one line.
[[nodiscard]] static Int16 heldAfter(Int16 held, const driveview::Keys& k, Int32 dtMs)
{
    return driveview::steerHeldStep(held, k, driveview::STEER_RATE_DEFAULT, dtMs);
}

static Void testSteerHeld()
{
    std::printf("\n-- the steering ramps out and springs back to centre --\n");
    const driveview::Keys none;
    driveview::Keys a;
    a.left = true;
    driveview::Keys d;
    d.right = true;
    driveview::Keys both;
    both.left = true;
    both.right = true;
    driveview::Keys c;
    c.centre = true;
    driveview::Keys ac;
    ac.left = true;
    ac.centre = true;
    check(driveview::STEER_RATE_DEFAULT == 1500, "the default rate is 1500 a second");
    check(driveview::steerLockMs(1500) == 666, "which is centre to full lock in 666 ms");
    check(driveview::steerLockMs(0) == -1, "and a rate of zero has no time to show");
    // 1500 a second over 100 ms is 150; a 16 ms frame is 24.
    check(heldAfter(0, a, 100) == -150, "A moves the held steering toward full left at the rate");
    check(heldAfter(0, d, 100) == 150, "D moves it toward full right at the same rate");
    check(heldAfter(0, d, 16) == 24, "and a 16 ms frame is 24 milli");
    check(heldAfter(-300, d, 100) == -150, "D from a left angle travels back toward centre");
    Int16 held = 0;
    Int32 frames = 0;
    Bool overshot = false;
    while(held < driveview::STEER_FULL && frames < 1000)
    {
        held = heldAfter(held, d, 16);
        ++frames;
        if(held > driveview::STEER_FULL)
        {
            overshot = true;
        }
    }
    check(held == 1000, "holding D arrives at exactly full right");
    check(!overshot, "without passing it on the way");
    check(frames == 42, "in 42 frames of 16 ms - the rate's 666 ms plus one partial frame");
    check(heldAfter(1000, d, 100) == 1000, "D held at full right stays there");
    check(heldAfter(-990, a, 100) == -1000, "A near full left clamps to it rather than past it");
    check(
        heldAfter(3000, none, 16) == 976,
        "a value out of range is clamped first, then springs back"
    );
    // Released, the steering springs back to centre at the same rate.
    check(
        heldAfter(-420, none, 100) == -270,
        "releasing both keys moves the wheel back toward centre at the rate"
    );
    check(heldAfter(-420, none, 16) == -396, "a 16 ms frame at a time, the way a game's does");
    check(
        heldAfter(420, both, 100) == 270,
        "A and D together return toward centre too, rather than picking one"
    );
    check(heldAfter(100, none, 100) == 0, "and it stops AT centre rather than swinging past it");
    check(heldAfter(-5, none, 100) == 0, "from the other side as well");
    check(heldAfter(0, none, 100) == 0, "and centre with no key stays centre");
    check(heldAfter(-420, none, 0) == -420, "no time passing moves nothing on the way back either");
    check(heldAfter(-420, c, 16) == 0, "C puts the held steering back to centre");
    check(heldAfter(1000, c, 0) == 0, "instantly - it needs no time to pass");
    check(heldAfter(-420, ac, 100) == 0, "and C beats A held down with it");
    // A long frame moves at most STEER_FRAME_MS_MAX's worth, so one stall is not
    // full lock.
    check(heldAfter(0, d, 5000) == 150, "a 5000 ms frame moves only as far as a 100 ms one");
    check(heldAfter(0, d, driveview::STEER_FRAME_MS_MAX) == 150, "which is the cap");
    check(heldAfter(200, d, -40) == 200, "a negative frame length moves nothing");
    check(heldAfter(200, d, 0) == 200, "and neither does a zero-length one");
    // The rate is held to the slider's ends, and a tiny step still moves, or a
    // held key would look broken.
    check(driveview::steerHeldStep(0, d, 100000, 100) == 500, "a rate past the slider is its top");
    check(driveview::steerHeldStep(0, d, 0, 100) == 25, "a rate of zero is its bottom, not dead");
    check(
        driveview::steerHeldStep(0, d, driveview::STEER_RATE_MIN, 1) == 1,
        "and a step that rounds to zero milli is one"
    );
    // The intent carries the held value, not the keys.
    driveview::View v;
    check(v.steerRateMilliPerS == driveview::STEER_RATE_DEFAULT, "a fresh pane has the default");
    check(v.steerHeldMilli == 0, "and starts straight");
    v.enabled = true;
    v.steerHeldMilli = -420;
    check(driveview::intentFrom(none, v).steerMilli == -420, "no key down sends the held angle");
    check(driveview::intentFrom(d, v).steerMilli == -420, "and a key does not go around the ramp");
    v.enabled = false;
    check(driveview::intentFrom(none, v).steerMilli == 0, "not driving, no held angle is sent");
}

static Void testSettingsText()
{
    std::printf("\n-- the settings file: integers only, forgiving in, exact out --\n");
    const settings::Values defaults;
    const Str text = settings::toText(defaults);
    check(text.find("trim.steerMinUs=1230\n") != Str::npos, "a value is written as key=value");
    check(text.find("drive.steerRateMilliPerS=1500\n") != Str::npos, "the steering rate is saved");
    check(text.find("drive.assumedMode=0\n") != Str::npos, "and the asserted mode");
    check(text.find("Held") == Str::npos, "the held steering is NOT - a moment is not a setting");
    settings::Values back;
    back.steerMinUs = 1;
    back.assumedMode = 2;
    check(settings::fromText(text, back) == settings::VALUE_COUNT, "every one is read back");
    check(back == defaults, "as the values that were written");
    settings::Values tuned;
    tuned.steerMinUs = 1250;
    tuned.steerMaxUs = 1700;
    tuned.steerTrimUs = 1470;
    tuned.escMinUs = 1520;
    tuned.escMaxUs = 1650;
    tuned.steerSlewUs = 12;
    tuned.throttleSlewUs = 3;
    tuned.throttleCapMilli = 180;
    tuned.steerRateMilliPerS = 900;
    tuned.assumedMode = 1;
    settings::Values read;
    check(
        settings::fromText(settings::toText(tuned), read) == settings::VALUE_COUNT,
        "a tuned set round-trips"
    );
    check(read == tuned, "unchanged");
    check(settings::settle(tuned) == tuned, "and a set inside its ranges is left alone by settle");
    // Forgiving in: a BOM, CRLF, blank lines, comments, spaces around the '=',
    // a leading plus, no final newline, and an unknown key, skipped.
    settings::Values loose;
    const Size looseTaken = settings::fromText(
        "\xEF\xBB\xBF# a comment\r\n\r\n  trim.escMaxUs = 1620 \r\n; another\r\n"
        "future.key=7\r\ndrive.throttleCapMilli=+250",
        loose
    );
    check(looseTaken == 2u, "two known keys are read through all of it, the unknown one skipped");
    check(loose.escMaxUs == 1620, "the spaced value");
    check(loose.throttleCapMilli == 250, "and the signed one with no newline after it");
    check(loose.steerMinUs == trimview::STEER_MIN_DEFAULT, "a key the file lacks keeps its value");
    // Integers only: a value that is not one is ignored, never half-read.
    settings::Values strict;
    const Size strictTaken = settings::fromText(
        "trim.escMinUs=1541.5\ntrim.escMaxUs=1,600\ntrim.steerSlewUs=fast\n"
        "trim.throttleSlewUs=\ndrive.assumedMode=1e1\ntrim.steerMaxUs=12345678901\n",
        strict
    );
    check(strictTaken == 0u, "a decimal, a comma, a word, nothing, 1e1 and 11 digits: none");
    check(strict == settings::Values(), "and not one of them changed a value");
    // Out of range is clamped by the panes' own settle functions.
    settings::Values wild;
    const Size wildTaken = settings::fromText(
        "trim.steerMinUs=99999\ntrim.steerTrimUs=-5\ntrim.steerSlewUs=0\n"
        "drive.throttleCapMilli=-40\ndrive.steerRateMilliPerS=1\ndrive.assumedMode=9\n",
        wild
    );
    check(wildTaken == 6u, "out-of-range integers are still integers, and are read");
    const settings::Values tame = settings::settle(wild);
    const Int32 servoHi = static_cast<Int32>(bibowire::SERVO_US_HARD_MAX);
    check(tame.steerMinUs == servoHi - 1, "a minimum past the servo is one below its ceiling");
    check(tame.steerMaxUs == servoHi, "which pushes the maximum up to the ceiling");
    check(tame.steerTrimUs == tame.steerMinUs, "and a centre below both is brought inside them");
    const Int32 slewLo = static_cast<Int32>(bibowire::SLEW_US_MIN);
    check(tame.steerSlewUs == slewLo, "a slew of 0 is the slowest the board accepts");
    check(tame.throttleCapMilli == 0, "a negative cap is zero, never reverse");
    check(tame.steerRateMilliPerS == driveview::STEER_RATE_MIN, "a slow rate clamps to the slider");
    check(tame.assumedMode == 0, "and an impossible mode folds to MANUAL, not up to DRIVE");
    trimview::View trim;
    driveview::View drive;
    drive.open = true;
    drive.enabled = true;
    drive.steerHeldMilli = 300;
    settings::apply(tuned, trim, drive);
    check(settings::capture(trim, drive) == tuned, "applied then captured is the same set");
    check(
        drive.open && drive.enabled && drive.steerHeldMilli == 300,
        "and the window, the enable and the held steering are left alone"
    );
    // A missing file is a first run, not an error.
    settings::Values untouched = tuned;
    const Opt<Size> none = settings::load(
        "Z:\\no\\such\\directory\\bibo-viewer-settings.ini",
        untouched
    );
    check(!none.has_value(), "a missing file loads nothing");
    check(untouched == tuned, "and changes nothing");
    // Not beside bibo.exe in viewer/build, which build.bat clean deletes. Only
    // the paths are checked; nothing is written.
    const Str home = settings::defaultPath();
    const Str old = settings::legacyPath();
    const StrView tail = "\\bibo\\bibo-viewer-settings.ini";
    check(
        home.size() > tail.size() && StrView(home).substr(home.size() - tail.size()) == tail,
        "the settings live in a bibo folder of their own"
    );
    check(!old.empty() && home != old, "which is not the old place beside bibo.exe");
}

static Void testEnableOnEveryDatagram()
{
    std::printf("\n-- ENABLE on every datagram, or the keys look dead --\n");
    driveview::View v;
    v.enabled = true;
    v.throttleCapMilli = 200;
    link::ControlStamp at;
    at.sessionId = 0x51E55101u;
    at.armEpoch = 3u;
    // Every combination of the five keys, driving and then not. deadman::step
    // reaches STATE_LIVE only with `enable` set, so a combination that dropped
    // the bit would leave the car at REFUSE_NOT_ARMED.
    Bool alwaysEnabled = true;
    Bool seqRising = true;
    UInt32 seq = 0;
    for(Int32 mask = 0; mask < 32; ++mask)
    {
        driveview::Keys k;
        k.left = (mask & 1) != 0;
        k.right = (mask & 2) != 0;
        k.forward = (mask & 4) != 0;
        k.brake = (mask & 8) != 0;
        k.estop = (mask & 16) != 0;
        const UInt32 next = link::nextControlSeq(seq);
        if(next <= seq)
        {
            seqRising = false;
        }
        seq = next;
        at.seq = next;
        const bibowire::Control m = link::buildControl(driveview::intentFrom(k, v), at);
        if((m.buttons & bibowire::BUTTON_ENABLE) == 0u)
        {
            alwaysEnabled = false;
        }
    }
    check(alwaysEnabled, "ENABLE is set on all 32 key combinations while driving");
    check(seqRising, "and the seq rose on every one of them");
    // Not driving, no key may set ENABLE: the enable is consent.
    v.enabled = false;
    // A held steering, so "neither axis moves" tests the gate rather than a zero.
    v.steerHeldMilli = 600;
    Bool everEnabled = false;
    Bool everMoved = false;
    Bool estopSurvived = true;
    for(Int32 mask = 0; mask < 32; ++mask)
    {
        driveview::Keys k;
        k.left = (mask & 1) != 0;
        k.right = (mask & 2) != 0;
        k.forward = (mask & 4) != 0;
        k.brake = (mask & 8) != 0;
        k.estop = (mask & 16) != 0;
        const bibowire::Control m = link::buildControl(driveview::intentFrom(k, v), at);
        if((m.buttons & bibowire::BUTTON_ENABLE) != 0u)
        {
            everEnabled = true;
        }
        if(m.steerMilli != 0 || m.throttleMilli != 0)
        {
            everMoved = true;
        }
        const Bool wantStop = k.estop;
        const Bool sentStop = (m.buttons & bibowire::BUTTON_ESTOP) != 0u;
        if(wantStop != sentStop)
        {
            estopSurvived = false;
        }
    }
    check(!everEnabled, "ENABLE is never set while the operator is not driving");
    check(!everMoved, "and neither axis moves - not even a steering held at 600");
    check(estopSurvived, "while ESTOP still rides every datagram, which is the point of it");
}

static Void testAssumedModeIsTheOperatorsOwn()
{
    std::printf("\n-- assumedMode is the OPERATOR's belief, never the board's answer --\n");
    // A board reporting DRIVE.
    Vec<UInt8> wire;
    static_cast<Void>(pushCtlState(wire, 1000000, 1));
    link::Session s;
    static_cast<Void>(feed(s, wire, 1000));
    check(s.control.pilotMode == 2u, "the board reports it is driving");
    driveview::View v;
    v.enabled = true;
    v.assumedMode = static_cast<Int32>(bibowire::PilotMode::PILOT_MODE_MANUAL);
    const driveview::Keys none;
    const link::Intent manual = driveview::intentFrom(none, v);
    // If this followed CTLSTATE, the board's comparison of assumedMode with its
    // real mode would always pass and REFUSE_MODE could never fire.
    check(manual.assumedMode == 0u, "the datagram asserts what the UI selected");
    check(
        manual.assumedMode != static_cast<UInt8>(s.control.pilotMode),
        "and it DISAGREES with the board, which is how REFUSE_MODE can ever fire"
    );
    v.assumedMode = static_cast<Int32>(bibowire::PilotMode::PILOT_MODE_LOOK);
    check(driveview::intentFrom(none, v).assumedMode == 1u, "LOOK selected is LOOK sent");
    v.assumedMode = static_cast<Int32>(bibowire::PilotMode::PILOT_MODE_DRIVE);
    check(driveview::intentFrom(none, v).assumedMode == 2u, "DRIVE selected is DRIVE sent");
    // An impossible selection folds to MANUAL, the mode whose stick values the
    // board reads.
    v.assumedMode = 9;
    check(driveview::intentFrom(none, v).assumedMode == 0u, "an out-of-range mode folds to manual");
    v.assumedMode = -3;
    check(driveview::intentFrom(none, v).assumedMode == 0u, "and so does a negative one");
    link::ControlStamp at;
    at.sessionId = 1u;
    at.seq = 1u;
    v.assumedMode = static_cast<Int32>(bibowire::PilotMode::PILOT_MODE_MANUAL);
    const bibowire::Control m = link::buildControl(driveview::intentFrom(none, v), at);
    check(m.assumedMode == 0u, "the built CONTROL carries the operator's mode");
    check(m.assumedMode != s.control.pilotMode, "not the one CTLSTATE reported");
}

static Void testControlSlotAndCadence()
{
    std::printf("\n-- who holds the slot, and whose clock the stream keeps --\n");
    link::Session fresh;
    check(!link::holdsSlot(fresh), "before WELCOME this viewer holds nothing");
    check(
        link::controlPeriodMs(fresh) == static_cast<Int64>(bibowire::CONTROL_PERIOD_MS),
        "and would send at the protocol's own period"
    );
    // Through the codec, because `accepted` decides the slot.
    bibowire::Welcome driver;
    driver.sessionId = 0x51E55101u;
    driver.bootId = 77u;
    driver.accepted = 1;
    driver.armEpoch = 4;
    driver.capabilities = 0x0Fu;
    driver.controlPeriodMs = 80;
    driver.boardName = "bibobox";
    driver.text = "control is yours";
    Vec<UInt8> wire;
    static_cast<Void>(pushWelcomeAs(wire, driver));
    link::Session held;
    static_cast<Void>(feed(held, wire, 1000));
    check(held.haveWelcome, "the WELCOME arrives");
    check(link::holdsSlot(held), "accepted = 1 is the control slot");
    check(link::controlPeriodMs(held) == 80, "and the cadence is the board's 80 ms, not 50");
    bibowire::Welcome observer = driver;
    observer.accepted = 2;
    observer.refusal = 2;
    Vec<UInt8> watching;
    static_cast<Void>(pushWelcomeAs(watching, observer));
    link::Session obs;
    static_cast<Void>(feed(obs, watching, 1000));
    check(obs.haveWelcome, "an observer is welcomed too");
    check(!link::holdsSlot(obs), "but accepted = 2 holds no slot, so it sends no CONTROL");
    bibowire::Welcome refused = driver;
    refused.accepted = 0;
    Vec<UInt8> denied;
    static_cast<Void>(pushWelcomeAs(denied, refused));
    link::Session no;
    static_cast<Void>(feed(no, denied, 1000));
    check(!link::holdsSlot(no), "and a refused connection holds nothing at all");
    // Built directly rather than through the wire, so this tests the viewer's
    // rule, not the encoder's.
    link::Session zero;
    zero.haveWelcome = true;
    zero.welcome.controlPeriodMs = 0;
    check(
        link::controlPeriodMs(zero) == static_cast<Int64>(bibowire::CONTROL_PERIOD_MS),
        "a controlPeriodMs of 0 falls back to the protocol's 50 ms"
    );
}

static Void testCameraCaptureGaps()
{
    std::printf("\n-- which clock a camera dropout happened on --\n");
    const Vec<UInt8> jpegBytes(TINY_JPEG.begin(), TINY_JPEG.end());
    // The first frame only starts the capture clock, or the board's uptime would
    // read as a stall.
    {
        link::Session first;
        Vec<UInt8> one;
        static_cast<Void>(pushCamera(one, 1, 1000000, jpegBytes));
        static_cast<Void>(feed(first, one, 5000));
        check(
            first.cameraWorstCaptureMs == 0,
            "the first frame starts the capture clock and is not itself a gap"
        );
    }
    // Both frames arrive at one local instant, 400 ms apart on the board's
    // clock: the two measurements must stay independent.
    link::Session s;
    Vec<UInt8> wire;
    static_cast<Void>(pushCamera(wire, 1, 1000000, jpegBytes));
    static_cast<Void>(pushCamera(wire, 2, 1400000, jpegBytes));
    static_cast<Void>(feed(s, wire, 5000));
    check(
        s.cameraWorstCaptureMs == 400,
        "400 ms between captures is measured on the BOARD's clock"
    );
    check(
        link::worstGapMs(s.cameraRate) == 0,
        "while the arrival cadence, fed at one instant, saw no gap at all"
    );
    Vec<UInt8> narrow;
    static_cast<Void>(pushCamera(narrow, 3, 1500000, jpegBytes));
    static_cast<Void>(feed(s, narrow, 5100));
    check(s.cameraWorstCaptureMs == 400, "a narrower gap afterwards does not erase the worst one");
    // A restarted board sends a smaller timestamp; unsigned subtraction would
    // wrap to an enormous stall.
    Vec<UInt8> back;
    static_cast<Void>(pushCamera(back, 4, 900000, jpegBytes));
    static_cast<Void>(feed(s, back, 5200));
    check(
        s.cameraWorstCaptureMs == 400,
        "and a board whose clock went backwards is skipped, never counted"
    );
}

// Positive steer is right (chassis.hxx's steerToUs sends it to STEER_CAL_RIGHT)
// and +X is right (scene.hxx's frame). Invert either and the picture still looks
// reasonable, so the signs are held to an answer here.
static Void testSteerSigns()
{
    std::printf("\n  the signs of the heading arrow and the bending guides\n");
    constexpr Float32 EPS = 0.0005f;
    // The camera's guides, in image space.
    check(camview::bendAt(0.0f, 45, 1.0f) == 0.0f, "no steering is no bend");
    check(
        camview::bendAt(1.0f, 45, 0.0f) == 0.0f,
        "and no bend at the bumper, whatever the wheels do"
    );
    check(
        camview::bendAt(1.0f, 45, 1.0f) > 0.0f,
        "a RIGHT turn sweeps the guides toward larger image u"
    );
    check(camview::bendAt(-1.0f, 45, 1.0f) < 0.0f, "and a left turn sweeps them the other way");
    check(
        std::fabs(camview::bendAt(1.0f, 45, 1.0f) + camview::bendAt(-1.0f, 45, 1.0f)) < EPS,
        "the two are exact opposites, so the guides are not biased to one side"
    );
    check(camview::bendAt(1.0f, 0, 1.0f) == 0.0f, "a bend slider at zero turns the sweep off");
    // t squared, not t: a real path barely moves at the bumper.
    const Float32 half = camview::bendAt(1.0f, 45, 0.5f);
    const Float32 full = camview::bendAt(1.0f, 45, 1.0f);
    check(std::fabs(full - (4.0f * half)) < EPS, "the swing grows with t squared, not with t");
    check(
        camview::pctToUnit(150, 0, 100) == camview::pctToUnit(100, 0, 100),
        "a percent past its range clamps rather than drawing off the picture"
    );
    // The 3D arrow, in the world frame.
    const scene::Vec3 straight = scene::headingDir(0.0f);
    check(std::fabs(straight.x) < EPS, "straight wheels point along no sideways axis at all");
    check(
        std::fabs(straight.y - 1.0f) < EPS,
        "and straight ahead is +Y, which is where the car faces"
    );
    const scene::Vec3 right = scene::headingDir(1.0f);
    const scene::Vec3 left = scene::headingDir(-1.0f);
    check(right.x > 0.0f, "a RIGHT turn points the arrow toward +X, which the frame calls right");
    check(left.x < 0.0f, "and a left turn toward -X");
    check(std::fabs(right.x + left.x) < EPS, "the two are mirrored, not offset");
    check(
        right.y > 0.0f && left.y > 0.0f,
        "and both still point forwards - this is a heading, not a turn in place"
    );
    const Float32 len = (right.x * right.x) + (right.y * right.y);
    check(
        std::fabs(len - 1.0f) < EPS,
        "the direction is a unit vector, so ARROW_LEN alone sets its length"
    );
    // The pair must agree: bendAt (camera.hxx) and headingDir (scene.hxx) each
    // look right alone whichever sign they carry, so only together is a flip
    // caught.
    check(
        (camview::bendAt(1.0f, 45, 1.0f) > 0.0f) == (scene::headingDir(1.0f).x > 0.0f),
        "the guides and the arrow agree about which way is right"
    );
    check(
        (camview::bendAt(-1.0f, 45, 1.0f) < 0.0f) == (scene::headingDir(-1.0f).x < 0.0f),
        "and about which way is left"
    );
}

static Void testControlDefaults()
{
    std::printf("\n-- a fresh viewer asks for the slot, and drives nothing --\n");
    link::Client c;
    // The default is the operator's choice (Client::wantSlot), held here so it
    // cannot drift unread.
    check(
        link::controlSlotWanted(c),
        "a fresh client asks for the control slot, by the operator's choice"
    );
    const link::Intent idle = link::controlIntent(c);
    check(!idle.driving, "and is not driving");
    check(idle.steerMilli == 0 && idle.throttleMilli == 0, "with both axes neutral");
    check(idle.buttons == 0u, "and no buttons - no ENABLE, no ESTOP");
    const driveview::View pane;
    check(!pane.open, "the drive window starts closed");
    check(!pane.enabled, "with the enable off");
    check(pane.throttleCapMilli == driveview::THROTTLE_CAP_DEFAULT, "and a conservative cap");
    check(pane.throttleCapMilli < driveview::THROTTLE_CAP_MAX, "which is far below full throttle");
    link::Intent want;
    want.driving = true;
    want.steerMilli = -1000;
    want.throttleMilli = 250;
    want.buttons = bibowire::BUTTON_ENABLE;
    want.assumedMode = 2;
    link::setControl(c, want);
    const link::Intent got = link::controlIntent(c);
    check(got.driving, "what the pane published is what the worker reads");
    check(got.steerMilli == -1000, "steer included");
    check(got.throttleMilli == 250, "throttle included");
    check(got.buttons == bibowire::BUTTON_ENABLE, "buttons included");
    check(got.assumedMode == 2u, "and the asserted mode");
    link::wantControlSlot(c, true);
    check(link::controlSlotWanted(c), "asking for the slot is remembered for the next HELLO");
    link::wantControlSlot(c, false);
    check(!link::controlSlotWanted(c), "and withdrawing it is too");
}

static Size pushBundle(Vec<UInt8>& out, const bibowire::Bundle& m)
{
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeBundle(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_BUNDLE, body.data(), n);
}

static bibowire::Bundle bundleAt(UInt32 gen, UInt16 index, UInt16 count, CharSeq id, Bool loaded)
{
    bibowire::Bundle b;
    b.generation = gen;
    b.index = index;
    b.count = count;
    b.id = id;
    b.name = "x";
    b.about = "about";
    b.needs = bibowire::BUNDLE_NEEDS_LIDAR;
    b.ready = 1;
    b.loaded = loaded ? 1u : 0u;
    return b;
}

// A list is shown whole or not at all: frames land by index, a newer generation
// replaces the old only once it is complete, and count 0 is the empty list.
static Void testTags()
{
    std::printf("\n-- tags: the board's detections, paired with a picture by frame index --\n");
    link::Session s;
    check(!s.tagsSeen(100).has_value(), "nothing seen before any TAGS frame");
    Vec<UInt8> bytes;
    check(pushTags(bytes, 41, 7, 800, 500) > 0, "a TAGS frame with one tag frames");
    static_cast<Void>(feed(s, bytes, 100));
    check(s.haveTags && s.tagFrames == 1, "and is held, counted");
    const Opt<link::TagsSeen> seen = s.tagsSeen(150);
    check(seen.has_value(), "seen while fresh");
    if(seen.has_value())
    {
        check(seen->tags.frameIndex == 41, "under the camera frame's own index");
        check(seen->tags.width == 640 && seen->tags.height == 480, "and the frame's size");
        check(seen->tags.tags.size() == 1 && seen->tags.tags[0].id == 7, "with the tag");
        check(seen->tags.tags[0].corners[0].xDeci == 800, "and its corners in tenths of a pixel");
        check(
            seen->tags.tags[0].rangeMm == 1500 && seen->tags.tags[0].bearingCdeg == 750,
            "its range and bearing"
        );
        check(
            (seen->tags.flags & bibowire::TAGS_FLAG_CALIBRATED) != 0u,
            "and that they are measured"
        );
        check(seen->ageMs == 50, "aged from its arrival");
        check(!seen->stale, "not stale at 50 ms");
    }
    check(
        !s.tagsSeen(100 + link::GONE_MS + 1).has_value(),
        "and gone past GONE_MS, like the picture"
    );
    // A newer frame replaces the older one whole.
    bytes.clear();
    static_cast<Void>(pushTags(bytes, 42, 9, 100, 100));
    static_cast<Void>(feed(s, bytes, 200));
    const Opt<link::TagsSeen> next = s.tagsSeen(210);
    check(
        next.has_value() && next->tags.frameIndex == 42 && next->tags.tags[0].id == 9,
        "the next frame replaces it"
    );
    check(s.tagFrames == 2, "counted again");
    // A body claiming more tags than it carries is refused, and the held one stays.
    bytes.clear();
    static_cast<Void>(pushTags(bytes, 43, 1, 0, 0));
    bytes[bibowire::HEAD_BYTES + 16] = 5;
    // The CRC no longer matches, so the frame is refused at take(); either way nothing changes.
    static_cast<Void>(feed(s, bytes, 300));
    check(s.tagFrames == 2 && s.tags.frameIndex == 42, "a damaged TAGS frame changes nothing");
    check(
        (link::subscriptionMask(false) & bibowire::typeBit(bibowire::Type::TYPE_TAGS)) != 0u,
        "TAGS is always subscribed: it costs 24 bytes when nothing is seen"
    );
}

static Size pushOdom(Vec<UInt8>& out, UInt64 monoUs, Int32 ticks, Int16 tps, UInt8 seq)
{
    bibowire::Odom m;
    m.tMonoUs = monoUs;
    m.ticks = ticks;
    m.ticksPerS = tps;
    m.skips = 0;
    m.invalid = 2;
    m.seq = seq;
    Vec<UInt8> body(32, 0);
    const Size n = bibowire::writeOdom(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_ODOM, body.data(), n);
}

static Void testOdom()
{
    std::printf("\n-- odom: the Pico's wheel count, a LIVE feed of its own --\n");
    link::Session s;
    check(!s.odometry(100).has_value(), "nothing before any ODOM frame");
    Vec<UInt8> bytes;
    check(pushOdom(bytes, 1000, -2859, -420, 7) > 0, "an ODOM frame frames");
    static_cast<Void>(feed(s, bytes, 100));
    check(s.haveOdom && s.odomFrames == 1, "and is held, counted");
    const Opt<link::Odometry> o = s.odometry(150);
    check(o.has_value(), "read while fresh");
    if(o.has_value())
    {
        check(
            o->odom.ticks == -2859 && o->odom.ticksPerS == -420,
            "with the signed count and speed"
        );
        check(o->odom.invalid == 2 && o->odom.seq == 7, "the error count and the sequence");
        check(o->ageMs == 50 && !o->stale, "aged from its arrival, fresh at 50 ms");
    }
    check(
        !s.odometry(100 + link::GONE_MS + 1).has_value(),
        "and gone past GONE_MS: a silent Pico is not a stopped wheel"
    );
    bytes.clear();
    static_cast<Void>(pushOdom(bytes, 2000, -2853, 120, 8));
    static_cast<Void>(feed(s, bytes, 200));
    const Opt<link::Odometry> next = s.odometry(210);
    check(
        next.has_value() && next->odom.ticks == -2853 && next->odom.seq == 8,
        "the next frame replaces it"
    );
    // A short body is refused and the held one stays.
    bytes.clear();
    Vec<UInt8> shortBody(12, 0);
    static_cast<Void>(framed(bytes, bibowire::Type::TYPE_ODOM, shortBody.data(), shortBody.size()));
    static_cast<Void>(feed(s, bytes, 300));
    const Opt<link::Odometry> held = s.odometry(310);
    check(held.has_value() && held->odom.seq == 8, "a short ODOM body leaves the held count");
}

static Size pushPose(Vec<UInt8>& out, UInt64 monoUs, Int32 xMm, Int32 yMm, Int32 headingMilliRad)
{
    bibowire::Pose m;
    m.tMonoUs = monoUs;
    m.xMm = xMm;
    m.yMm = yMm;
    m.headingMilliRad = headingMilliRad;
    m.sigmaXyMm = 30;
    m.sigmaHeadingMilliRad = 50;
    m.source = 1;
    m.valid = 1;
    Vec<UInt8> body(32, 0);
    const Size n = bibowire::writePose(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_POSE, body.data(), n);
}

static Void testPose()
{
    std::printf("\n-- pose: where the board reckons the car is --\n");
    link::Session s;
    check(!s.poseSeen(100).has_value(), "nothing before any POSE frame");
    Vec<UInt8> bytes;
    check(pushPose(bytes, 1000, 250, -1200, 1571) > 0, "a POSE frame frames");
    static_cast<Void>(feed(s, bytes, 100));
    check(s.havePose && s.poseFrames == 1, "and is held, counted");
    const Opt<link::PoseSeen> p = s.poseSeen(150);
    check(p.has_value(), "read while fresh");
    if(p.has_value())
    {
        check(
            p->pose.xMm == 250 && p->pose.yMm == -1200 && p->pose.headingMilliRad == 1571,
            "with the position and heading"
        );
        check(p->pose.valid == 1u && p->pose.sigmaXyMm == 30, "valid, with its uncertainty");
        check(p->ageMs == 50 && !p->stale, "aged from its arrival, fresh at 50 ms");
    }
    check(!s.poseSeen(100 + link::GONE_MS + 1).has_value(), "and gone past GONE_MS");
}

static Void testTrail()
{
    std::printf("\n-- trail: the world's points seen from the car --\n");
    bibowire::Pose now;
    now.xMm = 1000;
    now.yMm = 2000;
    now.headingMilliRad = 0;
    bibowire::Pose behind;
    behind.xMm = 1000;
    behind.yMm = 1000;
    trail::Point p = trail::inCarFrame(now, behind);
    check(
        std::fabs(p.x) < 1.0e-6f && std::fabs(p.y + 1.0f) < 1.0e-6f,
        "a metre behind a car facing +Y is at car (0, -1)"
    );
    // The car has turned to face world -X (heading +pi/2). A point a metre
    // further along -X is straight ahead of it.
    now.headingMilliRad = 1571;
    bibowire::Pose ahead;
    ahead.xMm = 0;
    ahead.yMm = 2000;
    p = trail::inCarFrame(now, ahead);
    check(
        std::fabs(p.x) < 1.0e-3f && std::fabs(p.y - 1.0f) < 1.0e-3f,
        "after a left turn to face -X, a point along -X is ahead, car +Y"
    );
    // And a point at world +Y of the car is now on its RIGHT.
    bibowire::Pose side;
    side.xMm = 1000;
    side.yMm = 3000;
    p = trail::inCarFrame(now, side);
    check(
        std::fabs(p.x - 1.0f) < 1.0e-3f && std::fabs(p.y) < 1.0e-3f,
        "and a point at world +Y is on the car's right, +X"
    );
    trail::Trail t;
    bibowire::Pose a;
    check(t.add(a), "the first pose is kept");
    a.xMm = 10;
    check(!t.add(a), "one that moved under MIN_STEP_MM is not");
    a.xMm = 40;
    check(t.add(a) && t.points.size() == 2, "one that moved enough is");
    check(
        std::fabs(t.distanceMm - 40.0) < 1.0e-9,
        "and the distance adds the step between kept points"
    );
    t.mark(a);
    check(t.marks.size() == 1, "a mark is kept");
    for(Int32 i = 0; i < static_cast<Int32>(trail::MAX_POINTS) + 10; ++i)
    {
        a.xMm += 100;
        static_cast<Void>(t.add(a));
    }
    check(t.points.size() == trail::MAX_POINTS, "a full trail drops its oldest");
    check(t.points.front().xMm > 40, "which was the start");
    t.clear();
    check(t.points.empty() && t.marks.empty() && t.distanceMm == 0.0, "clear empties everything");
}

static Void testCarMesh()
{
    std::printf("\n-- the car model: OBJ text into the world frame --\n");
    // A box 200 wide (x), 100 tall (y, up), 400 long (z), floor at y = -10, with
    // one quad face carrying texture coordinates and one triangle without.
    const Str obj = "o shell\n"
                    "v -100 -10 -200\n"
                    "v 100 -10 -200\n"
                    "v 100 -10 200\n"
                    "v -100 -10 200\n"
                    "v 0 90 0\n"
                    "vt 0 0\n"
                    "vt 1 0\n"
                    "vt 1 1\n"
                    "vt 0 1\n"
                    "f 1/1 2/2 3/3 4/4\n"
                    "f 1 2 5\n";
    const Str mtl = "newmtl skin\nmap_Kd body.jpg\n";
    carmesh::Mesh m;
    Str why;
    check(carmesh::parse(obj, mtl, &m, why), "a small model parses");
    check(m.name == "shell", "and keeps its name");
    check(m.triangles.size() == 3, "a quad fans into two triangles, plus the one");
    check(m.textureFile == "body.jpg", "the MTL names the skin");
    if(m.triangles.size() == 3)
    {
        // Fitted: the 400-unit length becomes the car's length along Y, the
        // floor sits at CAR_FLOOR_M, the width scales alike, and the roof is up.
        Float32 minY = 1.0e9f;
        Float32 maxY = -1.0e9f;
        Float32 minX = 1.0e9f;
        Float32 maxX = -1.0e9f;
        Float32 minZ = 1.0e9f;
        Float32 maxZ = -1.0e9f;
        for(const carmesh::Triangle& t : m.triangles)
        {
            for(const carmesh::Vertex& v : t.at)
            {
                minY = std::min(minY, v.y);
                maxY = std::max(maxY, v.y);
                minX = std::min(minX, v.x);
                maxX = std::max(maxX, v.x);
                minZ = std::min(minZ, v.z);
                maxZ = std::max(maxZ, v.z);
            }
        }
        check(
            std::fabs((maxY - minY) - carmesh::CAR_LENGTH_M) < 1.0e-4f,
            "the long axis is the car's length, along Y"
        );
        check(
            std::fabs((maxX - minX) - carmesh::CAR_LENGTH_M * 0.5f) < 1.0e-4f,
            "the width scales with it"
        );
        check(
            std::fabs(minZ - carmesh::CAR_FLOOR_M) < 1.0e-4f,
            "the floor sits at the car's floor height"
        );
        check(maxZ > minZ + 0.1f, "and the model's Y became up");
        check(
            std::fabs(m.triangles[0].at[0].v - 1.0f) < 1.0e-6f,
            "texture V is flipped to the GPU's top-left origin"
        );
        check(
            m.triangles[2].at[2].u == 0.0f && m.triangles[2].at[2].v == 0.0f,
            "a corner without coordinates reads (0, 0)"
        );
        check(
            std::fabs(m.roofZ - maxZ) < 1.0e-6f,
            "the roof over the origin is the apex, where the hat goes"
        );
    }
    // Refusals, each in words.
    carmesh::Mesh none;
    check(
        !carmesh::parse("v 0 0 0\n", "", &none, why) && why == "no faces in the model",
        "a model with no faces is refused"
    );
    check(
        !carmesh::parse("v 0 0 0\nf 1 2 9\n", "", &none, why),
        "a face naming a missing vertex is refused"
    );
    check(
        !carmesh::load("Z:\\nowhere\\car.obj", &none, why) && why.find("cannot read") == 0,
        "a missing file says so"
    );
    check(
        carmesh::directoryOf("a\\b\\c.obj") == "a\\b\\" && carmesh::directoryOf("c.obj").empty(),
        "the directory of a path, with its separator"
    );
}

static Void testBundles()
{
    std::printf("\n-- bundles: a list arrives whole or not at all --\n");
    link::Session s;
    Vec<UInt8> bytes;
    static_cast<Void>(pushBundle(bytes, bundleAt(1, 1, 2, "net.exoad.test.b", false)));
    static_cast<Void>(feed(s, bytes, 100));
    check(!s.haveBundles, "one frame of two is not a list yet");
    check(s.unknownFrames == 0u, "and BUNDLE is a known type, not counted as unknown");
    bytes.clear();
    static_cast<Void>(pushBundle(bytes, bundleAt(1, 0, 2, "net.exoad.test.a", true)));
    static_cast<Void>(feed(s, bytes, 110));
    check(s.haveBundles && s.bundles.size() == 2, "the second frame completes it, out of order");
    const Bool byIndex = s.bundles.size() == 2 && s.bundles[0].id == "net.exoad.test.a"
                      && s.bundles[1].id == "net.exoad.test.b";
    check(byIndex, "placed by index, not by arrival");
    check(
        s.bundles[0].loaded == 1u && s.bundleGeneration == 1u,
        "with loaded and the generation kept"
    );
    bytes.clear();
    static_cast<Void>(pushBundle(bytes, bundleAt(2, 0, 1, "net.exoad.test.c", false)));
    static_cast<Void>(feed(s, bytes, 120));
    check(
        s.bundles.size() == 1 && s.bundleGeneration == 2u,
        "a newer generation replaces the list whole"
    );
    bytes.clear();
    static_cast<Void>(pushBundle(bytes, bundleAt(1, 0, 2, "net.exoad.test.a", false)));
    static_cast<Void>(feed(s, bytes, 130));
    check(
        s.bundles.size() == 1 && s.bundleGeneration == 2u,
        "a late frame from an old generation shows nothing"
    );
    bytes.clear();
    static_cast<Void>(pushBundle(bytes, bundleAt(3, 0, 0, "net.exoad.test.none", false)));
    static_cast<Void>(feed(s, bytes, 140));
    check(
        s.haveBundles && s.bundles.empty() && s.bundleGeneration == 3u,
        "count 0 is the empty list, known and empty"
    );
    bytes.clear();
    static_cast<Void>(pushBundle(bytes, bundleAt(4, 3, 2, "net.exoad.test.z", false)));
    const UInt32 refusedBefore = s.refusedFrames;
    static_cast<Void>(feed(s, bytes, 150));
    check(s.refusedFrames == refusedBefore + 1u, "an index past count is a refused body");
    bibowire::BundleState st;
    st.loadedCount = 2;
    st.anyLoaded = 1;
    st.id = "net.exoad.tt02bibo.stop";
    st.text = "stop loaded";
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeBundleState(st, body.data(), body.size());
    bytes.clear();
    static_cast<Void>(framed(bytes, bibowire::Type::TYPE_BUNDLE_STATE, body.data(), n));
    static_cast<Void>(feed(s, bytes, 160));
    check(s.haveBundleState && s.bundleState.loadedCount == 2u, "BUNDLE_STATE is kept");
    bytes.clear();
    static_cast<Void>(pushEvent(bytes, "weave unloaded", 0, bibowire::EVENT_CODE_BUNDLE));
    static_cast<Void>(feed(s, bytes, 170));
    check(
        s.haveBundleNote && s.bundleNote.text == "weave unloaded",
        "an EVENT about a bundle is kept for the window"
    );
    check(
        !s.notes.empty() && s.notes.back().text == "weave unloaded",
        "and still listed as a note"
    );
    const UInt32 mask = link::subscriptionMask(false);
    const Bool asks = (mask & bibowire::typeBit(bibowire::Type::TYPE_BUNDLE)) != 0u
                   && (mask & bibowire::typeBit(bibowire::Type::TYPE_BUNDLE_STATE)) != 0u;
    check(asks, "and the viewer asks the board for both types");
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
    testCadenceBand();
    testCameraKeepsItsPromise();
    testCameraRate();
    testOrientation();
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
    testBoardTrim();
    testIdleTestIntent();
    testSubscriptionMask();
    testCommandIds();
    testCommandEncoding();
    testSlewArithmetic();
    testCmdAck();
    testControlSeq();
    testControlRoundTrip();
    testDriveKeys();
    testSteerHeld();
    testEnableOnEveryDatagram();
    testAssumedModeIsTheOperatorsOwn();
    testControlSlotAndCadence();
    testControlDefaults();
    testCameraCaptureGaps();
    testSteerSigns();
    testSettingsText();
    testBundles();
    testTags();
    testOdom();
    testPose();
    testTrail();
    testCarMesh();
    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
