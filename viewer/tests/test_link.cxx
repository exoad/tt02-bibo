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
//   - CONTROL'S SOCKET HALF. This viewer DOES send CONTROL now - the seq rule,
//     the encoding, the key mapping and the enable bit are all held to an
//     answer below - but nothing here puts a datagram on a wire. sendControl,
//     the UDP sendto, the TCP fallback and its latch, and the reverse-path
//     probe are compiled and reasoned about only. NO CONTROL DATAGRAM HAS EVER
//     REACHED A BOARD, no wheel has moved, and the deadman, the arm sequence
//     and REFUSE_MODE are shapes matched to docs/bibowire.md section 6 rather
//     than behaviours anyone has observed.
//   - THE HEADING ARROW AND THE BENDING GUIDES AS DRAWN. Their SIGNS are held
//     to an answer below, which is the part that cannot be eyeballed: positive
//     steer is right (chassis.hxx's steerToUs toward servoMax, which cal.hxx
//     names STEER_CAL_RIGHT) and +X is right (scene.hxx's frame note), so an
//     inversion of either draws a confident arrow the wrong way and compiles
//     perfectly. What is NOT proved is any pixel of either: the drawing lives
//     behind an ImDrawList and NEITHER HAS EVER BEEN SEEN ON SCREEN.
//   - THE CAMERA'S SOCKET HALF. The decode path below is driven with
//     hand-built CAMERA frames and a real JPEG. The board DOES send CAMERA now
//     and a real one has been watched for hours - that sentence used to say no
//     board had ever sent one, and it went stale. What is proved here is that
//     the bytes survive the codec, become pixels, and that a dropout is
//     classified onto the right clock; what is NOT proved is the subscription
//     handshake itself, that the board stops sending when one is withdrawn, or
//     anything about WHY a real camera drops out - which is what the arrival
//     and capture gaps below exist to let an operator answer in the field
//     rather than by sending somebody a log.
//   - the texture upload and the window. Those need a D3D11 device, which is
//     why the decoder is its own module (jpeg.cxx) and the window is not.
//   - THE TRIM PANE AS DRAWN, and the board's half of COMMAND. The queue, the
//     cmdId rule, the encoding and the slew arithmetic are all held to an answer
//     below, and trim.hxx keeps that arithmetic inline in the header precisely
//     so this suite can reach it without linking a file that names ImGui. What
//     is NOT proved is any of it against a car: NO COMMAND HAS EVER BEEN PUT ON
//     A WIRE. The board-side handler for verbs 8-11 is being written in
//     parallel, the Pico is not connected, and so "refused while armed" is a
//     sentence this suite can only check the SHAPE of, never the behaviour.
//   - THE ALIGNMENT OVERLAYS. They are placed in the window's frame - a scale
//     and an offset onto the drawn picture, ignoring rotate and flip by the
//     user's decision - so there is no mapping left here to hold to an answer.
//     The drawing itself lives in camera.cxx behind an ImDrawList; it was
//     checked by driving camview::drawWindow headlessly, not by this suite.
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
#include "orient.hxx"
#include "trim.hxx"
#include "drive.hxx"

// For bendAt and pctToUnit, which live at namespace scope in the header rather
// than in camera.cxx's anonymous namespace precisely so this file can reach
// them. camera.hxx names no ImGui type and no D3D type - only forward
// declarations - so including it here links nothing graphical.
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
    m.wifiName = "FieldPhone";
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

// A WELCOME whose every field the caller chose. pushWelcome above fixes
// `accepted` at 2, which is exactly the field the control tests are about, so
// this one takes the struct - pushCmdAck's shape, for pushCmdAck's reason.
static Size pushWelcomeAs(Vec<UInt8>& out, const bibowire::Welcome& m)
{
    Array<UInt8, 256> body = {};
    const Size n = bibowire::writeWelcome(m, body.data(), body.size());
    return n == 0 ? 0 : framed(out, bibowire::Type::TYPE_WELCOME, body.data(), n);
}

// The board's answer to one COMMAND. Takes the whole struct rather than a
// parameter per field, the way pushScan does: the interesting cases differ in
// three or four fields at once and a list of them would wrap.
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

    // THE AGE IS WHAT THIS TEST IS ABOUT AND IT HAS NOT CHANGED. Whether 600 ms
    // READS as stale is now the measured band's business, and it deliberately
    // answers differently: this feed has delivered exactly one interval and that
    // interval was 600 ms, so 600 ms is the cadence it is keeping rather than
    // evidence it has stopped. It goes stale past the band that interval earned,
    // which is the check below - and the old assertion here, that 600 ms is
    // stale because 600 > 400, is the very arithmetic that made the lidar dots
    // and the camera flicker.
    check(!rev->stale, "600 ms is not stale for a feed whose measured gap IS 600");
    check(rev->staleAtMs == 900, "the band that one 600 ms interval earned is 900 ms");

    const Opt<link::Revolution> later = s.revolution(2500);
    check(later.has_value(), "the same revolution is still drawable at 1400 ms");
    check(later.has_value() && later->stale, "and past the band it does read stale");
}

static Void testCadenceBand()
{
    std::printf("\n-- the staleness band is measured, not assumed --\n");

    // Nothing measured yet: section 7's number stands until a feed has earned
    // a different one.
    link::Cadence fresh;
    check(link::worstGapMs(fresh) == 0, "an unmeasured feed reports no gap");
    check(link::staleBandMs(fresh) == link::FRESH_MS, "and is held to FRESH_MS");

    // The first arrival starts the clock; it is not itself an interval.
    link::noteArrival(fresh, 1000);
    check(link::worstGapMs(fresh) == 0, "the first frame is not a gap");
    check(link::staleBandMs(fresh) == link::FRESH_MS, "so the band has not moved");

    // A FAST feed must not be able to tighten the band. Measuring may only ever
    // widen it for a slow feed, never shorten section 7's number for a quick one.
    link::Cadence quick;
    for(Int32 i = 0; i < 20; ++i)
    {
        link::noteArrival(quick, 1000 + (i * 20));
    }
    check(link::worstGapMs(quick) == 20, "a 50 Hz feed measures a 20 ms gap");
    check(link::staleBandMs(quick) == link::FRESH_MS, "and is STILL held to FRESH_MS, never less");

    // Two frames a second - the board's own default, and the case that was
    // measured reading STALE on 57 frames out of 57.
    link::Cadence slow;
    for(Int32 i = 0; i < 20; ++i)
    {
        link::noteArrival(slow, 1000 + (i * 500));
    }
    check(link::worstGapMs(slow) == 500, "a 2 fps feed measures a 500 ms gap");
    check(link::staleBandMs(slow) == 750, "and earns a 750 ms band");
    check(link::staleBandMs(slow) > 500, "which is wider than the interval it delivers at");

    // THE CEILING, and why it exists: however dreadful the feed, stale has to
    // stay strictly below GONE_MS or a picture would go from live to absent
    // with no band in between to warn anybody it was aging.
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

    // The window SLIDES. A stall that has stopped happening must stop widening
    // the band, or a feed could die quietly inside room its worst moment bought
    // it an hour ago.
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

    // Twelve frames at the board's 2 fps default, each arriving exactly when
    // the one before it implied. tMonoUs is 0 throughout so the age is the one
    // measured from local arrival and this test is about the band alone.
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

    // A camera that has genuinely STOPPED still goes stale, and then still goes.
    // The band moved; what it means did not.
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

    // An EVENT about something else must not be dressed up as a camera
    // refusal: an empty window blaming the wrong subsystem is worse than an
    // empty window.
    Vec<UInt8> other;
    static_cast<Void>(pushEvent(other, "lidar timeout - no revolution in 200 ms", 0));
    static_cast<Void>(feed(s, other, 1010));
    checkStr(
        s.cameraNoteText,
        "camera busy - another pilot holds /dev/video0",
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

    // Unturned, every corner maps to itself.
    const Array<orient::Uv, 4> flat = orient::cornerUvs(0, false, false);
    checkUv(flat[0], 0.0f, 0.0f, "unturned, the top-left is the source's top-left");
    checkUv(flat[2], 1.0f, 1.0f, "and the bottom-right is the source's bottom-right");
    check(!orient::sideways(0), "and the picture is not on its side");

    // A QUARTER TURN CLOCKWISE, which is the case ImGui::Image cannot express:
    // what was at the source's bottom-left belongs at the destination's
    // top-left, and no pair of opposite uv corners can say that.
    const Array<orient::Uv, 4> cw = orient::cornerUvs(1, false, false);
    checkUv(cw[0], 0.0f, 1.0f, "turned 90, the top-left samples the source's BOTTOM-left");
    checkUv(cw[1], 0.0f, 0.0f, "the top-right samples the source's top-left");
    checkUv(cw[2], 1.0f, 0.0f, "the bottom-right samples the source's top-right");
    checkUv(cw[3], 1.0f, 1.0f, "and the bottom-left samples the source's bottom-right");
    check(orient::sideways(1), "and 90 degrees IS on its side, so the fit swaps");

    // Half a turn is both axes mirrored - the one rotation a plain Image could
    // also have drawn, so the two had better agree about it.
    const Array<orient::Uv, 4> half = orient::cornerUvs(2, false, false);
    checkUv(half[0], 1.0f, 1.0f, "turned 180, the top-left samples the far corner");
    check(!orient::sideways(2), "and 180 is not on its side");
    check(orient::sideways(3), "while 270 is");

    // The flips are in SOURCE space, so they mean the same thing at any angle.
    const Array<orient::Uv, 4> mirrored = orient::cornerUvs(0, true, false);
    checkUv(mirrored[0], 1.0f, 0.0f, "flipped horizontally, the top-left samples the top-right");
    const Array<orient::Uv, 4> upended = orient::cornerUvs(0, false, true);
    checkUv(upended[0], 0.0f, 1.0f, "flipped vertically, the top-left samples the bottom-left");
    check(
        sameCorners(orient::cornerUvs(0, true, true), half),
        "and flipping BOTH axes is the same picture as turning it 180"
    );

    // FOLDED, NOT REFUSED. A rotate-left button hands this a negative, and a
    // mapping that only works for 0..3 breaks the first time one is wired up.
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

    // ZERO IS RESERVED. CTLSTATE's lastCmdId uses 0 to mean "none applied", so a
    // command numbered 0 is one the board could never report having run.
    check(c.pending[0].cmdId != 0u, "the first cmdId is not 0");
    check(c.pending[1].cmdId != 0u, "nor the second");
    check(c.pending[2].cmdId != 0u, "nor the third");

    check(c.pending[0].cmdId == 1u, "the counter starts at 1");
    check(c.pending[1].cmdId == 2u, "and the second is 2");
    check(c.pending[2].cmdId == 3u, "and the third is 3");

    // STRICTLY increasing, asserted as an ordering and not only as three
    // constants: the constants above would still pass if the counter were reset
    // between calls in some way that happened to produce 1, 2, 3.
    check(c.pending[1].cmdId > c.pending[0].cmdId, "strictly increasing");
    check(c.pending[2].cmdId > c.pending[1].cmdId, "at every step");

    // They do not COLLAPSE. Two camera-on requests are one fact; two tuning
    // commands are two acts, each of which gets its own CMDACK.
    check(c.pending[0].verb == bibowire::Verb::VERB_SET_SERVO_TRIM, "the first verb survives");
    check(c.pending[2].verb == bibowire::Verb::VERB_SET_ESC_LIMITS, "and so does the third");

    // sessionId and armEpoch are the WORKER's to stamp, at the moment of
    // sending - a snapshot taken here would be the session the operator typed
    // into rather than the one the frame goes out on.
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

    // What the worker does before it writes: stamp the connection's facts onto
    // the act the UI thread queued.
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

    // VALUES, NOT SUCCESS. 1230 and 1660 are different numbers on purpose, so a
    // swap of arg1 and arg2 fails here rather than round-tripping happily - and
    // arg1/arg2 are min/max, which is the pair a reader of the verb table is
    // most likely to reverse.
    check(back.verb == bibowire::Verb::VERB_SET_SERVO_LIMITS, "verb 9 survives");
    check(back.arg1 == 1230u, "min lands in arg1");
    check(back.arg2 == 1660u, "max lands in arg2");
    check(back.arg0 == 0u, "arg0 is unused by this verb and is zero");
    check(back.cmdId == 1u, "the cmdId survives");
    check(back.sessionId == 0x51E55101u, "and the session the worker stamped");
    check(back.armEpoch == 3u, "and the epoch");

    // SET_SLEW puts the AXIS in arg0 and the rate in arg1, which is the other
    // place a field can be put in the wrong slot - and the failure would be a
    // throttle rate silently applied to the steering.
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

    // 50 ticks a second, because the Pico's tick is 20 ms. This is the whole of
    // the first conversion and it is the one an operator never has to do again.
    check(trimview::slewUsPerSec(8) == 400, "8 us a tick is 400 us a second");
    check(trimview::slewUsPerSec(1) == 50, "1 is 50");
    check(trimview::slewUsPerSec(200) == 10000, "and 200 is 10000");

    // THE NUMBER cal.hxx ITSELF CLAIMS. Its comment says 8 is 400 us/s, "which
    // walks this car's 430 us of steering travel in about a second" - 1230 to
    // 1660 is 430, and the arithmetic here says 1.07 s. Agreeing with the
    // firmware's own prose is the point: two files describing one car.
    check(trimview::crossCentis(430, 8) == 107, "430 us of travel at 8 is 1.07 s lock to lock");

    // The throttle's 59 us band - 1541 to 1600 - at the same rate.
    check(trimview::crossCentis(59, 8) == 14, "the 59 us throttle band at 8 is 0.14 s");

    // The slowest and fastest the protocol allows, across the steering's travel.
    check(trimview::crossCentis(430, 1) == 860, "at 1 us a tick the same travel takes 8.60 s");
    check(trimview::crossCentis(430, 200) == 4, "and at 200 it takes 0.04 s");

    // ARGUMENT ORDER. A span and a rate are both small integers, so a swap
    // compiles and produces a plausible-looking number; these are different
    // answers, which is what makes the check worth writing.
    check(
        trimview::crossCentis(430, 8) != trimview::crossCentis(8, 430),
        "span and rate are not interchangeable"
    );

    // NOT A QUESTION WITH AN ANSWER. A zero span would read as "instant" and a
    // zero rate is a divide by zero; both are -1, which the pane renders as a
    // dash rather than as 0.00 s.
    check(trimview::crossCentis(0, 8) == -1, "no travel to cross has no time");
    check(trimview::crossCentis(-5, 8) == -1, "nor does a crossed pair of limits");
    check(trimview::crossCentis(430, 0) == -1, "and a rate of zero never arrives");

    // The defaults this pane starts from are the committed ones, so a drift in
    // either file is a failure here rather than a surprise on the car.
    check(trimview::STEER_MIN_DEFAULT == 1230, "the steering minimum mirrors cal.hxx");
    check(trimview::STEER_CENTRE_DEFAULT == 1480, "and the centre, which is not 1500");
    check(trimview::STEER_MAX_DEFAULT == 1660, "and the maximum");
    check(trimview::ESC_MIN_DEFAULT == 1541, "and the throttle's idle");
    check(trimview::ESC_MAX_DEFAULT == 1600, "and its full");

    // Every default must sit inside the bounds the protocol will accept, or the
    // pane opens on a value the board would refuse.
    check(trimview::STEER_MIN_DEFAULT >= static_cast<Int32>(bibowire::SERVO_US_HARD_MIN), "inside the servo floor");
    check(trimview::STEER_MAX_DEFAULT <= static_cast<Int32>(bibowire::SERVO_US_HARD_MAX), "and the servo ceiling");
    check(trimview::ESC_MIN_DEFAULT >= static_cast<Int32>(bibowire::ESC_US_HARD_MIN), "inside the ESC floor");
    check(trimview::ESC_MAX_DEFAULT <= static_cast<Int32>(bibowire::ESC_US_HARD_MAX), "and the ESC ceiling");
}

static Void testCmdAck()
{
    std::printf("\n-- what the board said, which is the whole point of a refusal --\n");

    link::Session s;
    check(!s.newestAck().has_value(), "before any answer there is nothing to show");

    // A REFUSAL. result 3 is "not in this state", which is what a tuning verb
    // gets while the car is armed.
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

    // VERBATIM. The sentence is the part a person can act on, and a viewer that
    // kept only the result byte would leave an operator with a number.
    checkStr(
        got->ack.text,
        "refused - disarm before changing the servo limits",
        "and its sentence, exactly as the board wrote it"
    );
    checkStr(Str(link::ackResultName(3)), "not in this state", "result 3 has a name");
    checkStr(Str(link::ackResultName(0)), "ok", "and so does 0");
    checkStr(Str(link::ackResultName(2)), "unknown verb", "and 2, for a board too old for these verbs");

    // The NEWEST is the one shown. An older ack sitting where the latest belongs
    // would report the wrong command's result at the moment somebody is watching.
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

    // BOUNDED. An unbounded list is a leak with a good excuse.
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

    // A cmdId belongs to one connection, so the answers go with the session.
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

    // STRICTLY INCREASING, asserted as an ordering over a run rather than as
    // three constants: the three above would still pass if the counter reset in
    // some way that happened to produce 1, 2, 42.
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

    // 6.8 years away at 20 Hz, and written anyway: a rule held by an arithmetic
    // coincidence is a rule nobody can point at.
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

    // VALUES, NOT SUCCESS, and EVERY FIELD A DIFFERENT VALUE - so a swap of any
    // pair fails here instead of round-tripping happily. The two Int16s and the
    // two trailing UInt8s are the pairs a reader of the byte table is most
    // likely to reverse, and they are asserted against each other as well.
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

    // BANG-BANG. The Pico's own SLEW does the smoothing and the trim pane tunes
    // it; a second ramp here would be two filters in series that nobody could
    // tell apart when the steering felt wrong.
    check(driveview::steerFrom(none) == 0, "no key is straight ahead");
    check(driveview::steerFrom(a) == -1000, "A is full left");
    check(driveview::steerFrom(d) == 1000, "D is full right");
    check(driveview::steerFrom(a) != driveview::steerFrom(d), "and left is not right");

    // BOTH CANCELS, and it is not "the last one wins": a hand resting on A while
    // reaching for D is the case, and a car that picked one would turn while its
    // operator believed it was straight.
    check(driveview::steerFrom(both) == 0, "A and D together cancel to straight");
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

    // S BEATS W. A brake W can override is not a brake - and W is already held
    // when somebody reaches for S, so "both down" is precisely the moment the
    // rule exists for. Inverted, the assertion below would read 300.
    check(driveview::throttleFrom(s, 300) == 0, "S alone is zero throttle");
    check(driveview::throttleFrom(ws, 300) == 0, "and S BEATS W when both are down");
    check(
        driveview::throttleFrom(ws, 300) != driveview::throttleFrom(w, 300),
        "which is a different answer from W alone, so the precedence is real"
    );

    // Zero and not negative: the band this project commands is forward-only, so
    // there is no reverse to ask for.
    check(driveview::throttleFrom(ws, 300) >= 0, "a brake is never negative throttle");
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

    // EVERY COMBINATION OF THE FIVE KEYS, twice over - once driving and once
    // not. deadman::step reaches STATE_LIVE only while `enable` is set, so a
    // combination that dropped the bit would be a car sitting at
    // REFUSE_NOT_ARMED with somebody leaning on W.
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

    // AND IT CANNOT BE SET WHILE THE OPERATOR IS NOT DRIVING, whatever the keys
    // say - the enable is consent, and consent is not something a key press
    // supplies on the operator's behalf.
    v.enabled = false;
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
    check(!everMoved, "and neither axis moves - steering is applied even when throttle is not");
    check(estopSurvived, "while ESTOP still rides every datagram, which is the point of it");
}

static Void testAssumedModeIsTheOperatorsOwn()
{
    std::printf("\n-- assumedMode is the OPERATOR's belief, never the board's answer --\n");

    // A board that says it is in DRIVE, twenty times a second.
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

    // THE WHOLE CHECK. If this ever followed CTLSTATE, the board's own
    // comparison - assumedMode against its real mode - would be true by
    // construction and REFUSE_MODE could never fire, which is the bug that was
    // just found and fixed on the BOARD side of the same comparison.
    check(manual.assumedMode == 0u, "the datagram asserts what the UI selected");
    check(
        manual.assumedMode != static_cast<UInt8>(s.control.pilotMode),
        "and it DISAGREES with the board, which is how REFUSE_MODE can ever fire"
    );

    v.assumedMode = static_cast<Int32>(bibowire::PilotMode::PILOT_MODE_LOOK);
    check(driveview::intentFrom(none, v).assumedMode == 1u, "LOOK selected is LOOK sent");
    v.assumedMode = static_cast<Int32>(bibowire::PilotMode::PILOT_MODE_DRIVE);
    check(driveview::intentFrom(none, v).assumedMode == 2u, "DRIVE selected is DRIVE sent");

    // An impossible selection folds to MANUAL - the mode whose stick values the
    // board actually reads - rather than becoming a belief about a mode nobody
    // is in.
    v.assumedMode = 9;
    check(driveview::intentFrom(none, v).assumedMode == 0u, "an out-of-range mode folds to manual");
    v.assumedMode = -3;
    check(driveview::intentFrom(none, v).assumedMode == 0u, "and so does a negative one");

    // And it survives the trip to the wire, where the board reads it.
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

    // accepted = 1 is "control is yours". Through the codec, because this is the
    // field the whole feature turns on.
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

    // THE BOARD'S NUMBER, not a constant compiled into this viewer months
    // earlier. Section 4 puts controlPeriodMs in WELCOME for exactly this.
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

    // A board that sends 0 does not get to make this viewer spin: a period of
    // zero is not a faster stream, it is a busy loop on the link the stream is
    // trying to survive on. Built directly rather than through the wire, so the
    // case is the viewer's rule and not the encoder's opinion of it.
    link::Session zero;
    zero.haveWelcome = true;
    zero.welcome.controlPeriodMs = 0;
    check(
        link::controlPeriodMs(zero) == static_cast<Int64>(bibowire::CONTROL_PERIOD_MS),
        "a controlPeriodMs of 0 falls back to the protocol's 50 ms"
    );
}

// THE TWO SIGNS NOBODY CAN EYEBALL.
//
// Both rest on facts written down elsewhere rather than inferred: positive
// steer is RIGHT, because chassis.hxx's steerToUs sends a positive fraction
// toward servoMax and cal.hxx names that STEER_CAL_RIGHT; and +X is the car's
// RIGHT, because scene.hxx states the frame and link.cxx repeats it where it
// turns a bearing into a cloud point. Invert either and the code compiles, the
// picture looks entirely reasonable, and the error is found while driving.
//
// This is orient.cxx's argument - a quarter turn is a transpose and nobody can
// eyeball a transpose - applied to a sign.
// WHICH CLOCK A DROPOUT HAPPENED ON.
//
// Two numbers answer it and neither can alone: the widest wait between
// ARRIVALS, on this viewer's clock, and the widest gap between CAPTURES, on the
// board's. Frames missing with captures steady means they were made and lost on
// the way; no frames missing with a capture gap means the board stopped making
// them. Before these existed a dropout needed a log tailed on somebody else's
// machine to explain, which is no use in a field.
static Void testCameraCaptureGaps()
{
    std::printf("\n-- which clock a camera dropout happened on --\n");

    const Vec<UInt8> jpegBytes(TINY_JPEG.begin(), TINY_JPEG.end());

    // THE FIRST FRAME ONLY STARTS THE CLOCK. Without that rule the board's
    // whole uptime - a thousand seconds here - is reported as a stall on the
    // very first picture, which is the classic version of this bug.
    {
        link::Session first;
        Vec<UInt8> one;
        static_cast<Void>(pushCamera(one, 1, 1000000, jpegBytes));
        static_cast<Void>(feed(first, one, 5000));
        check(first.cameraWorstCaptureMs == 0, "the first frame starts the capture clock and is not itself a gap");
    }

    // TWO CLOCKS, HELD APART. Both frames are fed at the SAME local instant, so
    // the arrival cadence sees no gap whatsoever - while the capture gap is
    // 400 ms, because that is what the board's own timestamps say. If these are
    // ever fed from one source this check fails, and it should: the entire
    // diagnosis rests on them being independent measurements.
    link::Session s;
    Vec<UInt8> wire;
    static_cast<Void>(pushCamera(wire, 1, 1000000, jpegBytes));
    static_cast<Void>(pushCamera(wire, 2, 1400000, jpegBytes));
    static_cast<Void>(feed(s, wire, 5000));
    check(s.cameraWorstCaptureMs == 400, "400 ms between captures is measured on the BOARD's clock");
    check(link::worstGapMs(s.cameraRate) == 0, "while the arrival cadence, fed at one instant, saw no gap at all");

    Vec<UInt8> narrow;
    static_cast<Void>(pushCamera(narrow, 3, 1500000, jpegBytes));
    static_cast<Void>(feed(s, narrow, 5100));
    check(s.cameraWorstCaptureMs == 400, "a narrower gap afterwards does not erase the worst one");

    // A board that restarted sends a SMALLER timestamp. Skipped rather than
    // recorded: the safe reading of a clock that moved the wrong way is
    // "measure again", and the unsigned subtraction would otherwise wrap to
    // something enormous and read as a catastrophic stall.
    Vec<UInt8> back;
    static_cast<Void>(pushCamera(back, 4, 900000, jpegBytes));
    static_cast<Void>(feed(s, back, 5200));
    check(s.cameraWorstCaptureMs == 400, "and a board whose clock went backwards is skipped, never counted");
}

static Void testSteerSigns()
{
    std::printf("\n  the signs of the heading arrow and the bending guides\n");

    constexpr Float32 EPS = 0.0005f;

    // ---- the camera's guides, in image space -------------------------------
    check(camview::bendAt(0.0f, 45, 1.0f) == 0.0f, "no steering is no bend");
    check(camview::bendAt(1.0f, 45, 0.0f) == 0.0f, "and no bend at the bumper, whatever the wheels do");
    check(camview::bendAt(1.0f, 45, 1.0f) > 0.0f, "a RIGHT turn sweeps the guides toward larger image u");
    check(camview::bendAt(-1.0f, 45, 1.0f) < 0.0f, "and a left turn sweeps them the other way");
    check(
        std::fabs(camview::bendAt(1.0f, 45, 1.0f) + camview::bendAt(-1.0f, 45, 1.0f)) < EPS,
        "the two are exact opposites, so the guides are not biased to one side"
    );
    check(camview::bendAt(1.0f, 0, 1.0f) == 0.0f, "a bend slider at zero turns the sweep off");

    // t*t and not t - four times the swing at twice the distance. A linear
    // sweep would move the guides at the bumper, where a real one barely does.
    const Float32 half = camview::bendAt(1.0f, 45, 0.5f);
    const Float32 full = camview::bendAt(1.0f, 45, 1.0f);
    check(std::fabs(full - (4.0f * half)) < EPS, "the swing grows with t squared, not with t");

    check(
        camview::pctToUnit(150, 0, 100) == camview::pctToUnit(100, 0, 100),
        "a percent past its range clamps rather than drawing off the picture"
    );

    // ---- the 3D arrow, in the world frame ----------------------------------
    const scene::Vec3 straight = scene::headingDir(0.0f);
    check(std::fabs(straight.x) < EPS, "straight wheels point along no sideways axis at all");
    check(std::fabs(straight.y - 1.0f) < EPS, "and straight ahead is +Y, which is where the car faces");

    const scene::Vec3 right = scene::headingDir(1.0f);
    const scene::Vec3 left = scene::headingDir(-1.0f);
    check(right.x > 0.0f, "a RIGHT turn points the arrow toward +X, which the frame calls right");
    check(left.x < 0.0f, "and a left turn toward -X");
    check(std::fabs(right.x + left.x) < EPS, "the two are mirrored, not offset");
    check(right.y > 0.0f && left.y > 0.0f, "and both still point forwards - this is a heading, not a turn in place");

    const Float32 len = (right.x * right.x) + (right.y * right.y);
    check(std::fabs(len - 1.0f) < EPS, "the direction is a unit vector, so ARROW_LEN alone sets its length");

    // ---- AND THE PAIR AGREES ----------------------------------------------
    //
    // The check that matters most, and the one neither file can make alone.
    // bendAt is in camera.hxx and headingDir is in scene.hxx; each is correct
    // on its own terms whichever sign it carries. If one is ever flipped, the
    // car draws guides sweeping right while the arrow points left, both look
    // reasonable in isolation, and only the two together are wrong - which is
    // this repo's named failure with a steering wheel attached.
    check(
        (camview::bendAt(1.0f, 45, 1.0f) > 0.0f) == (scene::headingDir(1.0f).x > 0.0f),
        "the guides and the arrow agree about which way is right"
    );
    check(
        (camview::bendAt(-1.0f, 45, 1.0f) < 0.0f) == (scene::headingDir(-1.0f).x < 0.0f),
        "and about which way is left"
    );
}

static Void testControlIsOptIn()
{
    std::printf("\n-- control is opt-in, and a fresh viewer asks for nothing --\n");

    link::Client c;

    // THE SAFETY PROPERTY, pinned. Section 6: the moment a viewer takes the
    // slot its cadence becomes the consent the deadman watches, and losing it
    // stops the car - so merely opening this program must not arm a deadman
    // over somebody else's autonomous run.
    check(!link::controlSlotWanted(c), "a fresh client does NOT ask for the control slot");

    const link::Intent idle = link::controlIntent(c);
    check(!idle.driving, "and is not driving");
    check(idle.steerMilli == 0 && idle.throttleMilli == 0, "with both axes neutral");
    check(idle.buttons == 0u, "and no buttons - no ENABLE, no ESTOP");

    // A default View is the same answer from the other end of the pane.
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
    testSubscriptionMask();
    testCommandIds();
    testCommandEncoding();
    testSlewArithmetic();
    testCmdAck();
    testControlSeq();
    testControlRoundTrip();
    testDriveKeys();
    testEnableOnEveryDatagram();
    testAssumedModeIsTheOperatorsOwn();
    testControlSlotAndCadence();
    testControlIsOptIn();
    testCameraCaptureGaps();
    testSteerSigns();

    std::printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
