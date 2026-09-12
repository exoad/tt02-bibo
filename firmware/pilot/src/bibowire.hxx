// bibowire v1: the wire between the Windows viewer and the pilot on the Pi.
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS, AND WHY IT IS ONE FILE COMPILED TWICE
//
// The viewer draws a picture of a car it cannot see, over a phone hotspot that
// stalls for seconds and drops when the phone moves. This file is the whole
// agreement about what crosses that link: one 16-byte frame shape, one body
// layout per message, one deadman, one renderer. The board's program and the
// Windows viewer compile the SAME OBJECT FILE - the rule scanwire.cxx and
// reactive.cxx already follow - so the encoder and the decoder cannot drift
// into disagreeing about a field's offset while both still compile.
//
// Pure, in the proto.hxx sense: no sockets, no clock, no device, no globals.
// Everything here is a function of its arguments. That is what lets the frame
// codec and the SAFETY TIMER be exercised on a laptop, in microseconds, with
// no car anywhere near a wall - and a deadman that is only testable with
// hardware is a deadman that gets tested once.
//
// ---------------------------------------------------------------------------
// THE ASYMMETRY THE SHAPE FOLLOWS FROM
//
// Telemetry is a picture and control is a command, and they fail in OPPOSITE
// directions. A late picture is worthless but harmless - drop the old one. A
// late command is dangerous: a three-second stall on an ordered stream delivers
// sixty queued steering commands when it clears, and the car applies a
// three-second-old throttle before it can reach the fresh one, at the moment
// the operator has started walking toward it. So telemetry rides TCP with a
// drop-oldest queue and control rides UDP where the board drains to empty and
// keeps only the newest seq. This file carries the ONE frame shape both use.
//
// ---------------------------------------------------------------------------
// NO FLOATING POINT ANYWHERE ON THE WIRE
//
// Every quantity is a fixed-point integer with its unit in its name. That is
// not compactness - it is the locale bug proto.cxx and scanwire.cxx both
// already met (printf("%.3f") honours the locale, a machine set to a comma
// decimal emits `0,250`, the far end reads `0`, and that is a silent hard-left
// at the first corner on somebody else's laptop) deleted at the root. There is
// no formatting step to be locale-sensitive, no NaN, no denormal, and every
// round-trip test is byte-exact rather than approximately equal.
//
// The same rule binds describe() below: it builds its digits by hand rather
// than through the C library, so the one decimal point it ever prints comes
// from integer arithmetic and not from a locale's idea of a radix character.
//
// ---------------------------------------------------------------------------
// FIELD ACCESS IS BY memcpy HELPER, NEVER BY CASTING A STRUCT AT THE PAYLOAD
//
// rd16/rd32/rd64 and wr16/wr32/wr64 below. On little-endian both compile to a
// single load or store, and the helper version cannot be silently broken by a
// compiler's padding decision or by an aarch64 alignment fault on a field
// somebody adds next year. Both ends are LE forever (aarch64 Linux, x86-64
// Windows) and that is static_assert-ed in the .cxx.
//
// The payload starts at frame offset 12, so a u64 at body offset 0 is NOT
// 8-aligned within the frame. Reading it through a struct pointer would be
// undefined on the day this runs on something that cares. Through memcpy it is
// simply correct, everywhere, forever.
//
// ---------------------------------------------------------------------------
// THE ABSENT SENTINELS, AND WHY EVERY OPTIONAL FIELD HAS ONE
//
// battMilliV 0xFFFF, health 255, cpuCentiC -32768, picoSilentMs 0xFFFFFFFF,
// escUs 0xFFFF, controlAgeMs 0xFFFFFFFF. Absence is REPRESENTABLE everywhere,
// so a field that is not measured never has to be faked by a zero - and 0 is a
// real reading of a dead battery pack, which is exactly why the sentinel is
// not 0. A reader given a `ver` it does not know must mark the fields that
// version did not carry with these rather than inventing them.
#pragma once

#include "shared.hxx"

#include <cstring>

namespace bibowire
{

  // ---- the wire's fixed numbers ---------------------------------------------

  // 8020, TCP and UDP, the same number for both. Deliberately a clear gap from
  // scanwire's 8011/8012, so `801x` means "the text feed a person can read" and
  // 8020 means bibowire, with no arithmetic. There is no fallback port and no
  // relay: a second protocol quietly moving next door is how a board ends up
  // with two feeds nobody can tell apart.
  constexpr UInt16 PORT = 8020;

  constexpr UInt16 PROTO_MAJOR = 1;
  constexpr UInt16 PROTO_MINOR = 0;

  // Frame <= 256 KiB, sized for a JPEG that does not exist yet.
  constexpr Size MAX_PAYLOAD = 262128;

  // What the BOARD accepts from a viewer. A header claiming more than this is
  // not a big frame, it is a hostile or broken peer: the socket half answers
  // BYE(TOO_BIG) and closes WITHOUT EVER ALLOCATING FOR THE CLAIM. This is the
  // single most important bound in the design, and it is a constant here so
  // that both halves read it from the same line.
  constexpr Size MAX_INBOUND_PAYLOAD = 256;

  // 1400 and not 1472: the board is also reachable over Tailscale, and
  // WireGuard's ~60 bytes of overhead would fragment a 1472-byte datagram
  // INSIDE the tunnel. No v1 UDP message exceeds 60 bytes; the bound exists so
  // the receive buffer is a fixed array and nothing sizes an allocation from a
  // claim a stranger on a hotspot wrote.
  constexpr Size MAX_DATAGRAM = 1400;

  constexpr Size MAX_SCAN_POINTS = 1024;
  constexpr Size MAX_EVENT_TEXT = 200;
  constexpr Size MAX_NAME = 31;
  constexpr Size MAX_BOARD_TEXT = 96;
  constexpr Size MAX_WIFI_NAME = 32;
  constexpr Size MAX_CLIENTS = 4;

  // 12 header + 4 trailer. Written as three names because every length
  // arithmetic below reads better for saying which part it means.
  constexpr Size HEAD_BYTES = 12;
  constexpr Size TRAILER_BYTES = 4;
  constexpr Size FRAME_OVERHEAD = HEAD_BYTES + TRAILER_BYTES;

  // 0x5742 little-endian puts the bytes 0x42 0x57 - 'B','W' - on the wire in
  // that order. The two spellings are the same number; MAGIC_LO and MAGIC_HI
  // exist because the resync scan looks at BYTES and should not have to
  // reconstruct a u16 to know where to start.
  constexpr UInt16 MAGIC = 0x5742;
  constexpr UInt8 MAGIC_LO = 0x42;
  constexpr UInt8 MAGIC_HI = 0x57;

  // Flags. Bits 0 and 1 are set on EVERY board->viewer frame, so "the car is
  // stopped" is derivable from any frame that arrives - even one whose body
  // this reader has no name for.
  //
  // The asymmetry is the rule that makes minor versions safe: a flag that
  // changes FRAMING must be refused by a reader that cannot honour it, and a
  // flag that only ANNOTATES may be ignored. Bits 3..15 are annotating and
  // reserved, so v1.1 may define one.
  constexpr UInt16 FLAG_ESTOP = 0x0001;
  constexpr UInt16 FLAG_DEADMAN = 0x0002;
  constexpr UInt16 FLAG_MORE = 0x0004;

  // ---- the absent sentinels --------------------------------------------------

  constexpr UInt16 BATT_ABSENT = 0xFFFF;
  constexpr UInt8 HEALTH_ABSENT = 255;
  constexpr Int16 CPU_ABSENT = -32768;
  constexpr UInt32 PICO_SILENT_ABSENT = 0xFFFFFFFFu;
  constexpr UInt16 ESC_ABSENT = 0xFFFF;
  constexpr UInt32 CONTROL_AGE_NEVER = 0xFFFFFFFFu;

  // ---- the deadman chain, timings and owners --------------------------------
  //
  // L0 the viewer (50), L1 the Pi to neutral (150), L2 the Pi to a full stop
  // (300), L3 the pilot's own blind/silence rules, L4 the Pico (400). The
  // static_assert that orders L2 against L4 lives in the .cxx beside the
  // function that trips.
  //
  // 150 because ONE lost datagram must not cut the throttle: Wi-Fi drops single
  // frames routinely and a car that stutters on every lost packet is a car
  // nobody will drive. Three consecutive losses at 20 Hz is no longer a lossy
  // link, it is a link that has stopped.
  constexpr Int32 CONTROL_PERIOD_MS = 50;
  constexpr Int32 CONTROL_STALE_MS = 150;
  constexpr Int32 CONTROL_DEAD_MS = 300;
  constexpr Int32 REARM_STREAM_MS = 500;
  constexpr Int32 TICK_MS = 20;

  // Pi -> USB CDC -> Pico, measured worst case.
  constexpr Int32 PICO_HOP_BUDGET_MS = 100;

  // firmware/app/main.cxx:49. NOT OURS TO CHANGE, and do not tighten it: it is
  // the only layer that covers the Orange Pi itself hanging, because nothing
  // running on the Pi can save you from the Pi.
  constexpr Int32 PICO_DEADMAN_MS = 400;

  // The control slot is released by 1000 ms of silence, not by 300: the car has
  // ALREADY been stopped by the deadman at 300, and handing the wheel to
  // somebody else 300 ms into a stall - while the first operator is still
  // holding the throttle and about to come back - would be worse than the
  // stall.
  constexpr Int32 CONTROL_SLOT_MS = 1000;

  // Within this long of WELCOME the board must have seen at least
  // REVERSE_PROBE_MIN datagrams from a viewer that asked for control, or it
  // says so in words. Measuring the reverse path rather than assuming it
  // because the other direction is fine is this repo's recurring bug class
  // applied to the link itself.
  constexpr Int32 REVERSE_PROBE_MS = 1000;
  constexpr Int32 REVERSE_PROBE_MIN = 5;

  // ---- types -----------------------------------------------------------------

  // Grouped so a reader can classify by range: 0x00-0x0F session, 0x10-0x3F
  // board->viewer, 0x40-0x4F viewer->board, 0xF0+ introspection. A tag is never
  // reused and a field never moves; that is the whole of the compatibility
  // promise, and the length prefix is what enforces it.
  enum class Type : UInt8
  {
      TYPE_HELLO = 0x01,
      TYPE_WELCOME = 0x02,
      TYPE_BYE = 0x03,
      TYPE_PING = 0x04,
      TYPE_PONG = 0x05,
      TYPE_LEAVE = 0x06,
      TYPE_SCAN = 0x10,
      TYPE_DECIDE = 0x11,
      TYPE_BOARD = 0x12,
      TYPE_LIDAR_INFO = 0x13,
      TYPE_EVENT = 0x14,
      TYPE_CTLSTATE = 0x15,
      TYPE_CMDACK = 0x16,
      TYPE_CAMERA = 0x20,
      TYPE_POSE = 0x21,
      TYPE_PATH = 0x22,
      TYPE_WAYPOINT = 0x23,
      TYPE_CONTROL = 0x40,
      TYPE_COMMAND = 0x41,
      TYPE_SUBSCRIBE = 0x42,
      TYPE_DESCRIBE = 0xF0,
      TYPE_SCHEMA = 0xF1,
  };

  constexpr Size TYPE_COUNT = 22;

  // The drop priority the socket half enforces against its bounded ring. This
  // is the reason a camera is safe to add to a 220 kbit/s link at all: BULK is
  // discarded before any LIVE or VITAL frame, so the day someone points a JPEG
  // stream down this socket the thing that degrades is the camera and not the
  // car's picture of the world.
  enum class Class
  {
      CLASS_VITAL = 0,
      CLASS_LIVE,
      CLASS_BULK,
  };

  // What take() found. TAKE_NEED_MORE is the ONLY non-error partial answer and
  // it consumes nothing; TAKE_RESYNC reports the junk it skipped so a count of
  // it can be kept, because junk on a checksummed stream that is never counted
  // is a fault nobody discovers.
  enum class Take
  {
      TAKE_FRAME = 0,
      TAKE_NEED_MORE,
      TAKE_RESYNC,
      TAKE_TOO_BIG,
      TAKE_BAD_FLAG,
  };

  // WHY throttle is not being applied, on the wire twenty times a second.
  //
  // This is the fix for the "one wrong byte bricks a healthy car" problem: an
  // epoch or mode disagreement correctly refuses throttle, but a refusal whose
  // only symptom is "throttle dead, steering fine" looks like an ESC or Pico
  // fault - and the ESC and the Pico are where a person goes looking first. So
  // the reason is a named field rather than a deduction.
  enum class Refuse : UInt8
  {
      REFUSE_NONE = 0,
      REFUSE_NOT_ARMED,
      REFUSE_DEADMAN_SOFT,
      REFUSE_DEADMAN_DEAD,
      REFUSE_ESTOP,
      REFUSE_EPOCH,
      REFUSE_MODE,
      REFUSE_NOT_HOLDER,
      REFUSE_PICO_DOWN,
      REFUSE_NO_UDP,
  };

  // The BYE reasons of section 4. The document spells these BYE_VERSION,
  // BYE_TOO_BIG and so on; that would collide with the `Bye` message struct
  // below, and one struct per Type named after its Type is worth more than the
  // enum's spelling. The WIRE VALUES are the document's, which is the part that
  // has to match.
  enum class Reason : UInt16
  {
      REASON_NONE = 0,
      REASON_VERSION = 1,
      REASON_TOO_BIG = 2,
      REASON_BAD_CRC = 3,
      REASON_BAD_SESSION = 4,
      REASON_TIMEOUT = 5,
      REASON_SHUTDOWN = 6,
      REASON_SUPERSEDED = 7,
      REASON_REFUSED = 8,
      REASON_BAD_FLAG = 9,
  };

  enum class Verb : UInt8
  {
      VERB_NONE = 0,
      VERB_ARM = 1,
      VERB_DISARM = 2,
      VERB_ESTOP = 3,
      VERB_CLEAR_ESTOP = 4,
      VERB_MOTOR_ON = 5,
      VERB_MOTOR_OFF = 6,
      VERB_SET_MODE = 7,
      VERB_SET_ESC_LIMITS = 8,

      // ---- tuning ---------------------------------------------------------
      //
      // These carry the old hub's Drive-view trim onto this wire. They fit
      // arg0/arg1/arg2 exactly as they already are, so COMMAND's 16 bytes do
      // not change and a board built before these verbs existed answers them
      // with result = 2 (unknown verb) rather than misreading a field.
      //
      // REFUSED WHILE ARMED, all three, with result = 3. Re-tuning the limits
      // a throttle is being clamped to, while that throttle is live, is the
      // one way this pane could hurt somebody - and the car is disarmed by
      // default, so the rule costs an operator nothing.
      VERB_SET_SERVO_LIMITS = 9,   // arg1 = min us, arg2 = max us
      VERB_SET_SERVO_TRIM = 10,    // arg1 = centre us
      VERB_SET_SLEW = 11,          // arg0 = axis, arg1 = us per 20 ms tick
  };

  // Which output VERB_SET_SLEW is talking about. "Both" is the bare `SLEW <us>`
  // the Pico has always accepted and is what the single shared rate used to
  // mean, kept because it is the common case on a bench.
  constexpr UInt8 SLEW_AXIS_BOTH = 0;
  constexpr UInt8 SLEW_AXIS_STEER = 1;
  constexpr UInt8 SLEW_AXIS_THROTTLE = 2;

  // ---------------------------------------------------------------------------
  // THE TUNING BOUNDS ARE MIRRORED HERE, AND THEY MUST AGREE WITH THE PICO
  //
  // The authority is firmware/lib/chassis/chassis.hxx - SLEW_MIN_STEP,
  // SLEW_MAX_STEP, and the hard servo and ESC clamps - and the Pico re-clamps
  // everything it is sent regardless of what arrives here. These copies exist
  // so a viewer can refuse an impossible number at the slider instead of
  // watching a command travel the length of the link to be rejected by a board
  // that then has to explain itself.
  //
  // That makes them a cross-boundary constant, which is this repo's named way
  // of shipping two halves that are each correct and broken as a pair. If the
  // chassis numbers ever move, THESE MOVE WITH THEM: the Pico is the one that
  // decides, and a viewer whose slider stops short of what the car can do is a
  // viewer lying about the car.
  constexpr UInt16 SLEW_US_MIN = 1;
  constexpr UInt16 SLEW_US_MAX = 200;

  // Ticks a second, so a viewer can turn us-per-tick into us-per-second and
  // into a lock-to-lock TIME, which is the unit an operator actually thinks in.
  // SLEW_TICK_MS is 20 in chassis.hxx; this is the same fact divided into 1000.
  constexpr UInt16 SLEW_TICKS_PER_S = 50;

  // The servo's own range. What a TT-02's steering can REACH is narrower and
  // off-centre (cal.hxx), which is what SET_SERVO_LIMITS is for finding.
  constexpr UInt16 SERVO_US_HARD_MIN = 1000;
  constexpr UInt16 SERVO_US_HARD_MAX = 2000;

  // Forward only, and not by omission. The board refuses below 1500 whatever is
  // asked for: reverse needs a brake-then-reverse sequence and is not something
  // to reach by widening a limit from a slider.
  constexpr UInt16 ESC_US_HARD_MIN = 1500;
  constexpr UInt16 ESC_US_HARD_MAX = 1700;

  enum class Severity : UInt8
  {
      SEVERITY_INFO = 0,
      SEVERITY_WARN = 1,
      SEVERITY_ERROR = 2,
  };

  struct Head
  {
      Type type = Type::TYPE_PING;
      UInt8 ver = 1;
      UInt16 flags = 0;
      UInt16 seq = 0;
  };

  // Borrowed, not owned: `bytes` points INTO the caller's buffer and is valid
  // only until that buffer is refilled. There is no allocation anywhere in
  // take(), which is what lets the board decode on a fixed 1024-byte ring with
  // no heap in the receive path at all.
  struct Body
  {
      const UInt8* bytes = nullptr;
      Size len = 0;
  };

  struct Frame
  {
      Head head;
      Body body;
  };

  // ---- byte access -----------------------------------------------------------
  //
  // Signed fields go through the unsigned reader and a named cast: in C++20 the
  // conversion is defined for every value, and one pair of helpers per width is
  // fewer things to get wrong than two.

  [[nodiscard]] inline UInt8 rd8(const UInt8* p)
  {
      return *p;
  }

  [[nodiscard]] inline UInt16 rd16(const UInt8* p)
  {
      UInt16 v = 0;
      std::memcpy(&v, p, sizeof(v));
      return v;
  }

  [[nodiscard]] inline UInt32 rd32(const UInt8* p)
  {
      UInt32 v = 0;
      std::memcpy(&v, p, sizeof(v));
      return v;
  }

  [[nodiscard]] inline UInt64 rd64(const UInt8* p)
  {
      UInt64 v = 0;
      std::memcpy(&v, p, sizeof(v));
      return v;
  }

  inline Void wr8(UInt8* p, UInt8 v)
  {
      *p = v;
  }

  inline Void wr16(UInt8* p, UInt16 v)
  {
      std::memcpy(p, &v, sizeof(v));
  }

  inline Void wr32(UInt8* p, UInt32 v)
  {
      std::memcpy(p, &v, sizeof(v));
  }

  inline Void wr64(UInt8* p, UInt64 v)
  {
      std::memcpy(p, &v, sizeof(v));
  }

  // Every payload is a multiple of 4, so every variable tail is padded up.
  [[nodiscard]] constexpr Size padTo4(Size n)
  {
      return (n + 3u) & ~static_cast<Size>(3u);
  }

  // ---- framing ---------------------------------------------------------------

  // Castagnoli, reflected, init and final xor 0xFFFFFFFF - the CRC32C the
  // RK3588 has an instruction for. No payload is ever exempted from it: 40 KB
  // at 30 fps is 1.2 MB/s, which at ~8 B/cycle is 0.006 % of one core, so the
  // special case is not worth its own branch.
  [[nodiscard]] UInt32 crc32c(const UInt8* data, Size len);

  // Takes ONE frame from the front of `buf`.
  //
  // `*consumed` is always written and is the number of bytes the caller should
  // retire from its ring: the whole frame on TAKE_FRAME, the junk skipped on
  // TAKE_RESYNC, and ZERO on every other answer.
  //
  // RESYNC IS DETERMINISTIC. Scan forward for the pair 0x42 0x57; reject the
  // candidate unless the type is known, `ver` is nonzero, `len` is a multiple
  // of 4 and within MAX_PAYLOAD, and the CRC over the whole candidate verifies.
  // Otherwise advance EXACTLY ONE BYTE and try again. Magic plus CRC together
  // put a false frame lock near 2^-48 per candidate rather than the 2^-16 a
  // magic alone would give.
  //
  // On TAKE_RESYNC the frame that was found is NOT returned - the junk is
  // reported first and the caller's next call returns the frame. One answer per
  // call keeps `consumed` unambiguous, which is the whole point of it.
  [[nodiscard]] Take take(const UInt8* buf, Size len, Frame* out, Size* consumed);

  // Writes head + body + CRC. Returns the frame length, or 0 when it would not
  // fit or the body is not a legal payload.
  //
  // `b.bytes` may point at `out + HEAD_BYTES`, in which case the body copy is
  // skipped - so an encoder can build its body straight into the frame buffer
  // and pay nothing for the framing.
  [[nodiscard]] Size put(const Head& h, const Body& b, UInt8* out, Size cap);

  [[nodiscard]] Class classOf(Type t);
  [[nodiscard]] Bool knownType(UInt8 tag);
  [[nodiscard]] CharSeq typeName(Type t);

  // ---- one struct and one typed pair per Type --------------------------------
  //
  // Each writeX returns the BODY bytes written, or 0 when it would not fit or
  // the message could not be represented on the wire (a count over the bound, a
  // quality over 63, an angle at 36000 which the device spells 0). Refusing to
  // encode is deliberate: a board that emits a frame its own reader would
  // refuse has moved a bug from the encoder into somebody else's decoder.
  //
  // Each readX returns false on a wrong length or an out-of-range field and
  // NEVER PARTIALLY FILLS `out`. A frame with the wrong count is not a shorter
  // frame - test_scanwire.cxx's own words about the text format, and the same
  // rule here.
  //
  // `ver` HIGHER than this module knows: the known prefix is read and the tail
  // ignored, which is why every readX accepts a body LONGER than it needs.
  // `ver` LOWER: the fields that version did not carry are stamped with their
  // absent sentinels and never fabricated. Every type in v1.0 is at version 1
  // and every field exists at version 1, so the lower-ver half of that rule has
  // no instance on today's wire; it is written and reachable so that the day a
  // type reaches v2 the mechanism is already the tested one.

  struct Hello
  {
      UInt16 protoMajor = PROTO_MAJOR;
      UInt16 protoMinor = PROTO_MINOR;
      UInt32 featureMask = 0;
      UInt32 viewerBuild = 0;
      UInt16 viewerUdpPort = 0;
      UInt16 wantControl = 0;
      UInt16 controlHz = 20;
      Str name;
  };

  struct Welcome
  {
      UInt16 protoMajor = PROTO_MAJOR;
      UInt16 protoMinor = PROTO_MINOR;
      UInt32 sessionId = 0;
      UInt32 bootId = 0;
      UInt32 featureMask = 0;
      UInt16 controlUdpPort = PORT;
      UInt16 controlPeriodMs = static_cast<UInt16>(CONTROL_PERIOD_MS);
      UInt16 staleMs = static_cast<UInt16>(CONTROL_STALE_MS);
      UInt16 deadMs = static_cast<UInt16>(CONTROL_DEAD_MS);
      UInt16 accepted = 0;
      UInt16 refusal = 0;
      UInt64 boardMonoUs = 0;
      UInt8 armEpoch = 0;
      UInt8 capabilities = 0;
      Str boardName;
      Str text;
  };

  struct Bye
  {
      Reason reason = Reason::REASON_NONE;
      Str text;
  };

  struct Ping
  {
      UInt64 token = 0;
      UInt64 senderMonoUs = 0;
  };

  struct Leave
  {
      UInt32 sessionId = 0;
      UInt32 reserved0 = 0;
  };

  struct ScanPoint
  {
      UInt16 angleCentiDeg = 0;   // 0..35999; 360 degrees wraps to 0, never 36000
      UInt16 distMm = 0;          // 0 means NO RETURN, not zero range
  };

  struct Scan
  {
      UInt64 tMonoUs = 0;
      UInt32 revIndex = 0;
      UInt16 freqMilliHz = 0;
      UInt8 health = HEALTH_ABSENT;
      UInt8 motor = 0;
      UInt16 droppedSinceLast = 0;
      UInt16 scanDivisor = 1;

      // Two parallel arrays rather than one 6-byte interleaved record, because
      // the quality array is then a straight copy from the Vec<UInt8> that
      // lidar::grab already fills. `quality` must be the same length as
      // `points` or writeScan refuses: a count is a promise about the frame.
      Vec<ScanPoint> points;
      Vec<UInt8> quality;
  };

  struct Decide
  {
      UInt32 revIndex = 0;   // 0 = a BLIND tick with no revolution behind it
      UInt32 clearanceMm = 0;
      UInt16 hits = 0;
      Int16 steerMilli = 0;
      Int16 throttleMilli = 0;
      UInt8 mode = 0;     // reactive::Mode: 0 cruise 1 slow 2 stop 3 reverse 4 blind
      UInt8 stop = 0;
      UInt8 source = 0;   // 0 manual, 1 look, 2 drive - WHO produced these numbers
      UInt32 modeMs = 0;
  };

  struct BoardState
  {
      UInt64 tMonoUs = 0;
      UInt32 upS = 0;
      Int16 cpuCentiC = CPU_ABSENT;
      UInt16 battMilliV = BATT_ABSENT;
      UInt8 picoLink = 0;   // 0 down, 1 up, 2 up-but-silent
      UInt8 picoArmed = 2;  // 0/1, 2 unknown
      UInt8 pilotMode = 0;
      UInt8 deadman = 0;    // 0 live, 1 soft, 2 dead, 3 estop latched
      UInt8 lidarHealth = HEALTH_ABSENT;
      UInt8 lidarSpinning = 0;
      UInt8 armEpoch = 0;
      UInt8 controlHolder = 0;   // 0 nobody, 1 you, 2 another viewer
      UInt32 loopWorstUs = 0;
      UInt32 loopLateCount = 0;
      UInt32 picoSilentMs = PICO_SILENT_ABSENT;
      UInt32 revolutions = 0;
      UInt32 timeouts = 0;
      UInt32 txDroppedFrames = 0;
      UInt32 rxControl = 0;
      UInt32 rxControlStale = 0;
      UInt32 ipv4 = 0;
      UInt32 encodeAvgNs = 0;
      UInt32 encodeMaxNs = 0;
      UInt8 clients = 0;

      // NetworkManager's connection name. The PASSPHRASE IS NOT A FIELD AND
      // NEVER WILL BE.
      Str wifiName;
  };

  struct LidarInfo
  {
      UInt16 model = 0;
      UInt8 fwMajor = 0;
      UInt8 fwMinor = 0;
      UInt16 hwRev = 0;
      Array<UInt8, 16> serial{};
      UInt32 baud = 0;
  };

  struct Event
  {
      UInt64 tMonoUs = 0;
      Severity severity = Severity::SEVERITY_INFO;
      UInt8 code = 0;

      // How many events the rate limiter suppressed since the last one, so the
      // viewer knows events were dropped rather than believing it saw them all.
      UInt16 droppedSince = 0;

      // Every lidar::reason(), every carlink::detail(), every refusal sentence
      // the board already writes for a person travels here VERBATIM. A binary
      // protocol that keeps only the codes is how a project loses the one thing
      // that makes a fault diagnosable.
      Str text;
  };

  struct CtlState
  {
      UInt64 tMonoUs = 0;
      UInt32 ackSeq = 0;                        // the CONTROL seq most recently APPLIED
      UInt32 controlAgeMs = CONTROL_AGE_NEVER;
      Int16 steerNowMilli = 0;                  // where the wheels ACTUALLY are
      Int16 throttleMilli = 0;                  // what was SENT to the Pico this tick
      UInt16 escUs = ESC_ABSENT;
      UInt16 neutralInMs = 0;
      UInt16 disarmInMs = 0;
      UInt8 armed = 0;
      UInt8 armEpoch = 0;
      UInt8 deadman = 0;
      Refuse refuse = Refuse::REFUSE_NONE;
      UInt8 holder = 0;
      UInt8 pilotMode = 0;
      UInt32 scanAgeMs = 0;
      UInt32 picoSilentMs = PICO_SILENT_ABSENT;
      UInt32 lastCmdId = 0;
  };

  struct CmdAck
  {
      UInt32 cmdId = 0;
      Verb verb = Verb::VERB_NONE;
      UInt8 result = 0;     // 0 ok, 1 refused, 2 unknown verb, 3 not in this state, 4 no Pico
      UInt8 armEpoch = 0;   // the epoch in force AFTER this command
      Str text;
  };

  struct Camera
  {
      UInt64 tMonoUs = 0;
      UInt32 frameIndex = 0;
      UInt16 width = 0;
      UInt16 height = 0;
      UInt8 codec = 0;   // 1 = JPEG, echoed on every frame so a capture is self-describing
      UInt8 flags = 0;
      Vec<UInt8> data;
  };

  struct Pose
  {
      UInt64 tMonoUs = 0;
      Int32 xMm = 0;
      Int32 yMm = 0;
      Int32 headingMilliRad = 0;
      UInt32 sigmaXyMm = 0;
      UInt32 sigmaHeadingMilliRad = 0;
      UInt8 source = 0;
      UInt8 valid = 0;
  };

  struct PathPoint
  {
      Int32 xMm = 0;
      Int32 yMm = 0;
  };

  struct Path
  {
      UInt64 tMonoUs = 0;
      UInt32 seq = 0;
      Vec<PathPoint> points;
  };

  // One waypoint, INDEXED - not PATH with a kind byte. The operation a person
  // actually performs on waypoints is nudging one, and folding them into PATH
  // makes editing waypoint 7 a resend of the whole set.
  struct Waypoint
  {
      UInt64 tMonoUs = 0;
      UInt32 seq = 0;
      UInt16 index = 0;
      UInt16 total = 0;
      Int32 xMm = 0;
      Int32 yMm = 0;
      UInt16 flags = 0;
  };

  struct Control
  {
      UInt32 sessionId = 0;
      UInt32 seq = 0;      // strictly increasing from 1; at 20 Hz a u32 lasts 6.8 years
      UInt64 tMonoUs = 0;  // the VIEWER's clock, only ever compared with itself
      Int16 steerMilli = 0;
      Int16 throttleMilli = 0;
      UInt16 buttons = 0;  // b0 ESTOP, b1 ENABLE, b2 MOTOR_WANTED
      UInt8 armEpoch = 0;  // the epoch the viewer BELIEVES it is armed under
      UInt8 assumedMode = 0;
  };

  constexpr UInt16 BUTTON_ESTOP = 0x0001;
  constexpr UInt16 BUTTON_ENABLE = 0x0002;
  constexpr UInt16 BUTTON_MOTOR_WANTED = 0x0004;

  struct Command
  {
      UInt32 sessionId = 0;
      UInt32 cmdId = 0;   // viewer-monotonic, never 0
      Verb verb = Verb::VERB_NONE;
      UInt8 arg0 = 0;
      UInt16 arg1 = 0;
      UInt16 arg2 = 0;
      UInt8 armEpoch = 0;
  };

  // The ceiling the board will honour on a camera request. 640x480 MJPEG is
  // about 45 KB a frame measured, so 15 fps is ~675 KB/s - which a LAN absorbs
  // and a phone hotspot does not. The viewer asks, the BOARD decides, and this
  // is where "decides" is written down: a request above this is clamped to it
  // rather than refused, because a viewer asking for too much should get the
  // most this board will give rather than nothing at all.
  //
  // It is safe to OFFER a rate this high only because CAMERA is CLASS_BULK:
  // pictures are discarded ahead of any scan or state frame, so a link that
  // cannot carry the rate loses camera frames and never the car's view.
  constexpr UInt16 CAM_FPS_MAX = 15;

  struct Subscribe
  {
      UInt32 sessionId = 0;
      UInt32 typeMask = 0;
      UInt16 scanDivisor = 1;   // 1 = every revolution, 3 = every third

      // Frames per second the VIEWER is asking the camera for. 0 is "did not
      // ask" and is what every viewer written before this field sends, so the
      // board's own default stands and nothing changes for them.
      //
      // This occupies what section 5 calls `reserved0`, which is what a
      // reserved field is for: the length is still 12, an older board ignores
      // it exactly as it always ignored those two bytes, and a newer board
      // reading an older viewer sees 0 and keeps its default. No version bump,
      // and no frame changes size.
      UInt16 camFps = 0;
  };

  struct Describe
  {
      UInt8 type = 0;   // 0 = all
  };

  struct Schema
  {
      Str text;
  };

  [[nodiscard]] Size writeHello(const Hello& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readHello(const Body& b, UInt8 ver, Hello* out);
  [[nodiscard]] Size writeWelcome(const Welcome& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readWelcome(const Body& b, UInt8 ver, Welcome* out);
  [[nodiscard]] Size writeBye(const Bye& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readBye(const Body& b, UInt8 ver, Bye* out);
  [[nodiscard]] Size writePing(const Ping& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readPing(const Body& b, UInt8 ver, Ping* out);
  [[nodiscard]] Size writeLeave(const Leave& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readLeave(const Body& b, UInt8 ver, Leave* out);
  [[nodiscard]] Size writeScan(const Scan& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readScan(const Body& b, UInt8 ver, Scan* out);
  [[nodiscard]] Size writeDecide(const Decide& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readDecide(const Body& b, UInt8 ver, Decide* out);
  [[nodiscard]] Size writeBoard(const BoardState& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readBoard(const Body& b, UInt8 ver, BoardState* out);
  [[nodiscard]] Size writeLidarInfo(const LidarInfo& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readLidarInfo(const Body& b, UInt8 ver, LidarInfo* out);
  [[nodiscard]] Size writeEvent(const Event& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readEvent(const Body& b, UInt8 ver, Event* out);
  [[nodiscard]] Size writeCtlState(const CtlState& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readCtlState(const Body& b, UInt8 ver, CtlState* out);
  [[nodiscard]] Size writeCmdAck(const CmdAck& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readCmdAck(const Body& b, UInt8 ver, CmdAck* out);
  [[nodiscard]] Size writeCamera(const Camera& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readCamera(const Body& b, UInt8 ver, Camera* out);
  [[nodiscard]] Size writePose(const Pose& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readPose(const Body& b, UInt8 ver, Pose* out);
  [[nodiscard]] Size writePath(const Path& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readPath(const Body& b, UInt8 ver, Path* out);
  [[nodiscard]] Size writeWaypoint(const Waypoint& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readWaypoint(const Body& b, UInt8 ver, Waypoint* out);
  [[nodiscard]] Size writeControl(const Control& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readControl(const Body& b, UInt8 ver, Control* out);
  [[nodiscard]] Size writeCommand(const Command& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readCommand(const Body& b, UInt8 ver, Command* out);
  [[nodiscard]] Size writeSubscribe(const Subscribe& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readSubscribe(const Body& b, UInt8 ver, Subscribe* out);
  [[nodiscard]] Size writeDescribe(const Describe& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readDescribe(const Body& b, UInt8 ver, Describe* out);
  [[nodiscard]] Size writeSchema(const Schema& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readSchema(const Body& b, UInt8 ver, Schema* out);

  // ---- the catalog -----------------------------------------------------------
  //
  // One row per Type: the tag, the name, the version, the drop class, the body
  // length with every variable tail EMPTY, the size expression a person reads,
  // and the field list. Both the SCHEMA answer and the tests' completeness
  // check are generated from it, so a message whose layout changed without its
  // catalog row changing is a test failure rather than a document that quietly
  // stopped describing the build.
  //
  // The .cxx static_asserts that the table covers every enumerator in ALL_TYPES
  // and that knownType() agrees with it for all 256 tags. What that CANNOT
  // catch, and nothing in C++20 can without reflection, is a Type added to the
  // enum and to neither list - said out loud rather than left to be discovered.
  struct Desc
  {
      Type type = Type::TYPE_PING;
      CharSeq name = "";
      UInt8 ver = 1;
      Class cls = Class::CLASS_VITAL;
      Size fixedLen = 0;
      Bool variable = false;
      CharSeq lenExpr = "";
      CharSeq fields = "";
  };

  [[nodiscard]] const Desc* catalog();
  [[nodiscard]] Size catalogCount();
  [[nodiscard]] const Desc* descOf(Type t);

  // "0x10 SCAN v1 24+4n+n : u64 tMonoUs us ; u32 revIndex ; ..." - the line
  // SCHEMA carries, so a board can be asked in ASCII what it speaks.
  [[nodiscard]] Str schemaLine(Type t);
  [[nodiscard]] Str schemaAll();

  // ---- the one shared renderer -----------------------------------------------
  //
  // Used by biboctl, the viewer's log pane, the pilot's console and the tests'
  // failure output, so a frame is printed the same way everywhere and the
  // printer cannot drift from the codec it sits beside.
  //
  // Sentinels render as `n/a`, never as 65535 or -32768. A number that means
  // "not measured" printed as a measurement is the same bug as a stale picture
  // drawn as live.
  [[nodiscard]] Str describe(const Frame& f);

  // ---- the version rule ------------------------------------------------------

  // protoMajor must be EQUAL. Not >=, not "compatible".
  [[nodiscard]] Bool versionOk(UInt16 protoMajor);

  // The refusal a person can act on. An explanation that says only
  // "incompatible" sends somebody to read source in a field; the two build
  // stamps are what turn it into an action, which is why the sentence is part
  // of the protocol and not part of the log.
  [[nodiscard]] Bye versionRefusal(const Hello& h, CharSeq boardBuild);

  // ---- applying a CONTROL ----------------------------------------------------

  namespace control
  {

    enum class Verdict
    {
        VERDICT_APPLIED = 0,
        VERDICT_STALE_SEQ,
        VERDICT_BAD_SESSION,
        VERDICT_NOT_HOLDER,
    };

    struct Gate
    {
        UInt32 sessionId = 0;      // the session WELCOME issued
        UInt32 highestSeq = 0;     // highest CONTROL seq APPLIED this session
        Bool haveHolder = false;
        Bool fromHolder = false;   // this datagram came from the current holder
        UInt8 armEpoch = 0;        // the epoch the BOARD owns
        UInt8 pilotMode = 0;       // the board's ACTUAL mode
    };

    struct Outcome
    {
        Verdict verdict = Verdict::VERDICT_BAD_SESSION;

        // "Parsed", not "received". Bytes half-read in an accumulator are not a
        // command, and counting them as liveness is this repo's recurring bug
        // class applied to the safety timer.
        Bool feedsDeadman = false;

        Int16 steerMilli = 0;
        Int16 throttleMilli = 0;
        Refuse refuse = Refuse::REFUSE_NONE;
        UInt32 highestSeq = 0;   // what the caller's high-water mark becomes
    };

    // static_cast<Int32>(a - b) > 0 on the UInt32s, so a viewer that restarted
    // and began again at seq 1 compares correctly rather than freezing the
    // board on a huge stale seq. THE HIGH-WATER MARK IS RESET BY THE HANDSHAKE,
    // per session - the obvious implementation is the bug: a board that kept it
    // across a reconnect would silently ignore every command from a restarted
    // viewer while the socket looked perfect.
    [[nodiscard]] Bool newer(UInt32 a, UInt32 b);

    // Pure. Decides what this datagram is allowed to do, and whether it feeds
    // the deadman - it does so only when it carries the current session AND
    // came from the current holder AND was actually applied. Without that last
    // rule a second viewer's stream keeps the timer alive while the first
    // operator, whose laptop just slept, is protected by nothing.
    [[nodiscard]] Outcome apply(const Gate& g, const Control& c);

  }

  // ---- the deadman -----------------------------------------------------------

  namespace deadman
  {

    enum class State
    {
        STATE_LIVE = 0,
        STATE_SOFT,
        STATE_DEAD,
        STATE_ESTOP,
    };

    struct Inputs
    {
        Int64 nowMs = 0;
        Int64 lastControlMs = 0;   // when the HOLDER's newest CONTROL was APPLIED
        Bool haveHolder = false;
        Bool enable = false;
        Bool epochMatches = false;
        Bool modeAgrees = false;
        Bool estopLatched = false;
    };

    struct Output
    {
        State state = State::STATE_DEAD;
        Refuse refuse = Refuse::REFUSE_DEADMAN_DEAD;
        Int32 neutralInMs = 0;
        Int32 disarmInMs = 0;
    };

    // No clock inside it - the shape reactive::step already uses. Two
    // consequences: the safety property is exercised on a laptop in
    // microseconds rather than by sleeping, and THE VIEWER RUNS THE IDENTICAL
    // FUNCTION on its own copy of the numbers, so an operator watching
    // "neutral in 90 ms" during a hiccup is watching the same arithmetic that
    // will do the tripping.
    [[nodiscard]] Output step(const Inputs& in);

    [[nodiscard]] CharSeq stateName(State s);

  }

  [[nodiscard]] CharSeq refuseName(Refuse r);

}
