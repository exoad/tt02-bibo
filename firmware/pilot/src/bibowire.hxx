// bibowire v1: the wire between the Windows viewer and the pilot on the Pi.
// docs/bibowire.md is the specification; "section N" means its sections.
//
// The board and the viewer compile this same codec, so the encoder and the
// decoder cannot disagree about a field's offset. Pure: no sockets, no clock,
// no device, no globals, so the frame codec and the deadman are tested on a
// laptop.
//
// Telemetry rides TCP with drop-oldest queues; control rides UDP, where the
// board keeps only the newest seq, because a late command is dangerous where a
// late picture is only worthless. Both use the one frame shape here.
//
// No floating point on the wire: every quantity is a fixed-point integer with
// its unit in its name, so no formatting step can meet the locale trap (a comma
// decimal sends `0,250` and the far end reads `0`). describe() builds its
// digits by hand for the same reason.
//
// Fields go through the rd/wr memcpy helpers, never a struct cast over the
// payload: the payload starts at frame offset 12, so a u64 in it is not
// 8-aligned. Both ends are little-endian, asserted in the .cxx.
//
// Every optional field has an ABSENT sentinel, because 0 is a real reading. A
// reader given an older `ver` stamps the fields that version lacked with them
// rather than inventing values.
#pragma once

#include "shared.hxx"

#include "../../lib/chassis/cal.hxx"

#include <cstring>

namespace bibowire
{
  // TCP and UDP both. No fallback port: a feed that moves is one no viewer can
  // find.
  constexpr UInt16 PORT = 8020;

  constexpr UInt16 PROTO_MAJOR = 1;
  constexpr UInt16 PROTO_MINOR = 0;

  // 256 KiB less FRAME_OVERHEAD, sized for a camera JPEG.
  constexpr Size MAX_PAYLOAD = 262128;

  // What the board accepts from a viewer. A header claiming more is answered
  // BYE(TOO_BIG) and closed without ever allocating for the claim.
  constexpr Size MAX_INBOUND_PAYLOAD = 256;

  // 1400, not 1472: over Tailscale, WireGuard's overhead would fragment a
  // 1472-byte datagram inside the tunnel. A fixed bound, so the receive buffer
  // is a fixed array and nothing sizes an allocation from a stranger's claim.
  constexpr Size MAX_DATAGRAM = 1400;

  constexpr Size MAX_SCAN_POINTS = 1024;
  constexpr Size MAX_EVENT_TEXT = 200;
  constexpr Size MAX_NAME = 31;
  constexpr Size MAX_BOARD_TEXT = 96;
  constexpr Size MAX_WIFI_NAME = 32;
  constexpr Size MAX_CLIENTS = 4;

  // A bundle's id is reverse-DNS and stable - net.exoad.tt02bibo.forward. It is
  // what a viewer keys its window on, so it must survive a rename of the
  // human-facing name beside it.
  constexpr Size MAX_BUNDLE_ID = 63;
  constexpr Size MAX_BUNDLE_NAME = 31;
  constexpr Size MAX_BUNDLE_ABOUT = 96;

  // Detections in one camera frame. Far more than a frame holds at the
  // sizes this camera offers; the bound keeps a TAGS body under 2 KB.
  constexpr Size MAX_TAGS = 64;

  constexpr Size HEAD_BYTES = 12;
  constexpr Size TRAILER_BYTES = 4;
  constexpr Size FRAME_OVERHEAD = HEAD_BYTES + TRAILER_BYTES;

  // 0x5742 little-endian puts 'B','W' (0x42 0x57) on the wire. The resync scan
  // matches MAGIC_LO and MAGIC_HI as bytes.
  constexpr UInt16 MAGIC = 0x5742;
  constexpr UInt8 MAGIC_LO = 0x42;
  constexpr UInt8 MAGIC_HI = 0x57;

  // FLAG_ESTOP and FLAG_DEADMAN are set on every board->viewer frame, so a
  // stopped car shows on any frame, even one of an unknown type. A reader must
  // refuse a flag that changes framing and may ignore one that only annotates;
  // bits 3..15 are reserved annotating flags.
  constexpr UInt16 FLAG_ESTOP = 0x0001;
  constexpr UInt16 FLAG_DEADMAN = 0x0002;
  constexpr UInt16 FLAG_MORE = 0x0004;

  constexpr UInt16 BATT_ABSENT = 0xFFFF;
  constexpr UInt8 HEALTH_ABSENT = 255;
  constexpr Int16 CPU_ABSENT = -32768;
  constexpr UInt32 PICO_SILENT_ABSENT = 0xFFFFFFFFu;
  constexpr UInt16 ESC_ABSENT = 0xFFFF;
  constexpr UInt32 CONTROL_AGE_NEVER = 0xFFFFFFFFu;

  // The deadman chain: L0 the viewer sends every CONTROL_PERIOD_MS, L1 the Pi
  // goes to neutral after CONTROL_STALE_MS, L2 to a full stop after
  // CONTROL_DEAD_MS, L3 the pilot's own blind and silence rules, L4 the Pico's
  // PICO_DEADMAN_MS.
  //
  // CONTROL_STALE_MS is three periods because one lost datagram must not cut
  // the throttle: Wi-Fi drops single frames routinely.
  constexpr Int32 CONTROL_PERIOD_MS = 50;
  constexpr Int32 CONTROL_STALE_MS = 150;
  constexpr Int32 CONTROL_DEAD_MS = 300;
  constexpr Int32 REARM_STREAM_MS = 500;
  constexpr Int32 TICK_MS = 20;

  // Pi -> USB CDC -> Pico, measured worst case.
  constexpr Int32 PICO_HOP_BUDGET_MS = 100;

  // The Pico's watchdog, owned by BIBO_WATCHDOG_MS in firmware/lib/chassis/
  // cal.hxx: the only layer that covers the Orange Pi itself hanging. The pilot
  // sends every TICK_MS in every state and re-sends the held command every
  // PICO_KEEPALIVE_MS during a lidar wait (firmware/pilot/app/main.cxx), so the
  // gap between two lines reaching the Pico is bounded by that keepalive.
  constexpr Int32 PICO_DEADMAN_MS = BIBO_WATCHDOG_MS;

  // Longer than CONTROL_DEAD_MS on purpose: the deadman has already stopped
  // the car, and handing the wheel to somebody else mid-stall, while the first
  // operator is still holding the throttle, would be worse than the stall.
  constexpr Int32 CONTROL_SLOT_MS = 1000;

  // Who writes steer and throttle (section 6). NOT Decide::mode, the autonomy's
  // cruise..blind: rendered through the other's names, MANUAL prints as
  // "cruise". pilotModeName() spells this; driveModeName() spells Decide::mode.
  enum class PilotMode : UInt8
  {
      PILOT_MODE_MANUAL = 0,   // CONTROL's stick values go to the Pico
      PILOT_MODE_LOOK = 1,     // the autonomy runs, throttle forced to 0 - this is --dry
      PILOT_MODE_DRIVE = 2,    // the autonomy drives; CONTROL is consent, not input
  };

  // Within REVERSE_PROBE_MS of WELCOME the board must have seen
  // REVERSE_PROBE_MIN datagrams from a viewer that asked for control, or it
  // says so in words: the reverse path is measured, not assumed.
  constexpr Int32 REVERSE_PROBE_MS = 1000;
  constexpr Int32 REVERSE_PROBE_MIN = 5;

  // 0x00-0x0F session, 0x10-0x3F board->viewer, 0x40-0x4F viewer->board,
  // 0xF0+ introspection. A tag is never reused and a field never moves.
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
      TYPE_BUNDLE = 0x17,
      TYPE_BUNDLE_STATE = 0x18,
      TYPE_CAMERA = 0x20,
      TYPE_POSE = 0x21,
      TYPE_PATH = 0x22,
      TYPE_WAYPOINT = 0x23,
      TYPE_TAGS = 0x24,
      TYPE_ODOM = 0x25,
      TYPE_CONTROL = 0x40,
      TYPE_COMMAND = 0x41,
      TYPE_SUBSCRIBE = 0x42,
      TYPE_DESCRIBE = 0xF0,
      TYPE_SCHEMA = 0xF1,
  };

  constexpr Size TYPE_COUNT = 26;

  // The drop priority of the socket half's bounded ring: BULK is discarded
  // first, then LIVE, and VITAL never, so the camera degrades before the car's
  // picture of the world.
  enum class Class
  {
      CLASS_VITAL = 0,
      CLASS_LIVE,
      CLASS_BULK,
  };

  // What take() found. TAKE_NEED_MORE is the only non-error partial answer and
  // consumes nothing; TAKE_RESYNC reports the junk it skipped so it is counted.
  enum class Take
  {
      TAKE_FRAME = 0,
      TAKE_NEED_MORE,
      TAKE_RESYNC,
      TAKE_TOO_BIG,
      TAKE_BAD_FLAG,
  };

  // Why throttle is not being applied, carried in CTLSTATE. Without a named
  // reason, an epoch or mode refusal looks like an ESC or Pico fault.
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

  // BYE reasons (section 4), named REASON_ so they do not collide with struct
  // Bye. The wire values are the specification's.
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

      // Tuning, from the viewer's Trim pane. They fit arg0/arg1/arg2 as they
      // are, so a board without them answers result 2 (unknown verb) rather
      // than misreading a field. Every tuning verb is refused while armed, with
      // result 3, except SET_ESC_LIMITS during BUTTON_IDLE_TEST: re-tuning the
      // limits a live throttle is clamped to could hurt somebody.
      VERB_SET_SERVO_LIMITS = 9,   // arg1 = min us, arg2 = max us
      VERB_SET_SERVO_TRIM = 10,    // arg1 = centre us
      VERB_SET_SLEW = 11,          // arg0 = axis, arg1 = us per 20 ms tick

      // The lowest pulse brake and reverse may reach. ESC_NEUTRAL_US is valid
      // and turns reverse off; above it is refused.
      VERB_SET_ESC_REVERSE = 12,   // arg1 = us

      // Which bundle runs. COMMAND has no string field, so a bundle is named
      // here by its INDEX in the BUNDLE frames the board published, and arg1
      // carries the generation those frames were stamped with. A load against a
      // list the board has since replaced is refused rather than starting
      // whatever now sits at that index - adding a bundle while a viewer has
      // the panel open must not be able to launch the wrong program.
      VERB_LOAD_BUNDLE = 13,   // arg0 = index, arg1 = generation
      // Unload takes the same pair: several behaviours are loaded at once, so
      // it has to name WHICH, and by index for the same reason LOAD does.
      VERB_STOP_BUNDLE = 14,   // arg0 = index, arg1 = generation
  };

  // Which output VERB_SET_SLEW sets. BOTH is the Pico's bare `SLEW <us>`.
  constexpr UInt8 SLEW_AXIS_BOTH = 0;
  constexpr UInt8 SLEW_AXIS_STEER = 1;
  constexpr UInt8 SLEW_AXIS_THROTTLE = 2;

  // An EVENT under this code is state, not news: the trim the board has saved
  // and replays to the Pico, as the Pico's lines joined by "; " -
  // "SERVOLIMITS 1230 1660; SERVOTRIM 1480" - or an empty text when nothing is
  // saved. Sent after WELCOME and after each save, never through the event rate
  // limiter.
  constexpr UInt8 EVENT_CODE_TRIM = 84;   // 'T'

  // A bundle loaded, stopped, was refused, or a manifest would not parse. The
  // last of those is why this exists: a typo in a .bundle must reach a person
  // rather than leave a row quietly missing from the master window.
  constexpr UInt8 EVENT_CODE_BUNDLE = 66;   // 'B'

  // The tuning bounds, mirrored from firmware/lib/chassis/chassis.hxx
  // (SLEW_MIN_STEP, SLEW_MAX_STEP and the hard servo and ESC clamps) so a
  // viewer can refuse an impossible number at the slider. The Pico re-clamps
  // regardless. A cross-boundary constant: if the chassis numbers move, these
  // move with them.
  constexpr UInt16 SLEW_US_MIN = 1;
  constexpr UInt16 SLEW_US_MAX = 200;

  // 1000 / SLEW_TICK_MS, so a viewer can turn us per tick into us per second
  // and a lock-to-lock time.
  constexpr UInt16 SLEW_TICKS_PER_S = 50;

  // The widest pulse a hobby servo is driven with. What a TT-02's steering can
  // reach is far narrower and off-centre (cal.hxx); the working limits found
  // with SET_SERVO_LIMITS are what protect the linkage.
  constexpr UInt16 SERVO_US_HARD_MIN = 500;
  constexpr UInt16 SERVO_US_HARD_MAX = 2500;

  // The whole RC pulse range. It does not enforce forward-only: the pilot never
  // maps W below neutral, whatever the working minimum (carrules::forwardPulse).
  constexpr UInt16 ESC_US_HARD_MIN = 1000;
  constexpr UInt16 ESC_US_HARD_MAX = 2000;

  // Neutral on every RC ESC. Below it is brake and then reverse on this car's
  // Forward/Reverse/Brake ESC, and VERB_SET_ESC_REVERSE bounds how far.
  constexpr UInt16 ESC_NEUTRAL_US = 1500;

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

  // Borrowed: `bytes` points into the caller's buffer and is valid only until
  // that buffer is refilled. take() never allocates.
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

  // Signed fields go through the unsigned reader and a cast, which C++20
  // defines for every value.
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

  // CRC32C: Castagnoli, reflected, init and final xor 0xFFFFFFFF. No payload is
  // exempt; even the camera's cost is a negligible fraction of a core.
  [[nodiscard]] UInt32 crc32c(const UInt8* data, Size len);

  // Takes ONE frame from the front of `buf`. `*consumed` is always written: the
  // whole frame on TAKE_FRAME, the junk skipped on TAKE_RESYNC, 0 otherwise.
  //
  // Resync scans for 0x42 0x57 and rejects a candidate unless `ver` is nonzero,
  // `len` is a multiple of 4 within MAX_PAYLOAD, the type is known (anywhere
  // but offset 0), and the CRC verifies; otherwise it advances ONE byte. Magic
  // plus CRC put a false frame lock near 2^-48 per candidate.
  //
  // On TAKE_RESYNC the frame found is not returned; the next call returns it,
  // so `consumed` is never ambiguous.
  [[nodiscard]] Take take(const UInt8* buf, Size len, Frame* out, Size* consumed);

  // Writes head + body + CRC. Returns the frame length, or 0 when it would not
  // fit or the body is not a legal payload. `b.bytes` may point at
  // `out + HEAD_BYTES`, and then the body is not copied.
  [[nodiscard]] Size put(const Head& h, const Body& b, UInt8* out, Size cap);

  [[nodiscard]] Class classOf(Type t);
  [[nodiscard]] Bool knownType(UInt8 tag);
  [[nodiscard]] CharSeq typeName(Type t);

  // A type's bit in HELLO.featureMask, WELCOME.featureMask and
  // SUBSCRIBE.typeMask: tag - 0x10, for the board->viewer tags 0x10..0x2F. Any
  // other type has no bit (0) and is always sent, so a mask cannot switch off
  // the frames that negotiate it. Not `1u << (tag & 0x1F)`: DECIDE (0x11) and
  // SCHEMA (0xF1) would share a bit.
  [[nodiscard]] constexpr UInt32 typeBit(Type type)
  {
      const UInt8 tag = static_cast<UInt8>(type);
      return tag >= 0x10u && tag <= 0x2Fu ? (1u << (tag - 0x10u)) : 0u;
  }

  // One struct and one write/read pair per Type.
  //
  // writeX returns the BODY bytes written, or 0 when it would not fit or cannot
  // be represented (a count over its bound, a quality over 63, an angle of
  // 36000): a board must not emit a frame its own reader would refuse.
  //
  // readX returns false on a wrong length or an out-of-range field and never
  // partially fills `out`. A HIGHER `ver` is read for its known prefix and the
  // tail ignored; a LOWER one gets the absent sentinels for the fields it
  // lacks. Every v1.0 type is at version 1, so that has no case yet.
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

      // Parallel arrays, so `quality` is a straight copy of what lidar::grab
      // fills. It must be as long as `points` or writeScan refuses.
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

      // NetworkManager's connection name. The passphrase is never a field.
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

      // Events the rate limiter suppressed since the last one.
      UInt16 droppedSince = 0;

      // Every lidar::reason(), carlink::detail() and refusal sentence the board
      // writes for a person, VERBATIM: the codes alone do not make a fault
      // diagnosable.
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

  // One waypoint, indexed rather than a PATH with a kind byte, so nudging one
  // waypoint does not resend the whole set.
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

  // The tag families a TAGS frame can name. Only 36h11 is detected today.
  constexpr UInt8 TAG_FAMILY_36H11 = 0;

  // One corner of a detected tag, in TENTHS of a pixel of the frame the
  // detector ran on (Tags::width by Tags::height), so a viewer scales into
  // whatever it decoded rather than trusting a size it never measured.
  // Tenths in an i16 reach 3276 pixels, past any size this camera offers,
  // and a corner refined to just outside the frame is legal and negative.
  struct TagCorner
  {
      Int16 xDeci = 0;
      Int16 yDeci = 0;
  };

  struct Tag
  {
      UInt16 id = 0;
      UInt8 hamming = 0;       // bit errors corrected; 0 is a clean read
      Int32 marginMilli = 0;   // the detector's decision margin, times 1000

      // In the detector's order: counter-clockwise around the tag as printed.
      Array<TagCorner, 4> corners = {};

      // Where the tag is, from the corners, the camera's intrinsics and the
      // printed tag's size: straight-line distance to its centre, and its
      // bearing from the camera's axis, positive to the RIGHT as steering is.
      // Both 0 while the camera is uncalibrated (Tags::flags), never a guess.
      Int32 rangeMm = 0;
      Int16 bearingCdeg = 0;
  };

  // Tags::flags: the ranges and bearings are measured, not zero for want of
  // a calibration.
  constexpr UInt8 TAGS_FLAG_CALIBRATED = 0x01;

  // What the detector found in ONE camera frame: every tag, or none, so a
  // frame with nothing in it is still a statement and a viewer can tell
  // "nothing seen" from "the detector stopped". frameIndex, width and height
  // are the CAMERA frame's own, so a viewer pairs a detection with the
  // picture it was made on. detectUs is the detector's own time on that
  // frame, which is the number an NPU port has to beat.
  struct Tags
  {
      UInt64 tMonoUs = 0;
      UInt32 frameIndex = 0;
      UInt16 width = 0;
      UInt16 height = 0;
      UInt8 family = TAG_FAMILY_36H11;
      UInt8 flags = 0;   // TAGS_FLAG_*
      UInt32 detectUs = 0;
      Vec<Tag> tags;
  };

  // The wheel encoder, as the car's Pico counts it: six hall steps per motor
  // turn (docs/hardware.md). `ticks` is monotonic and signed, forward
  // positive; `ticksPerS` is the Pico's own speed estimate. The two error
  // counts SATURATE at 255 rather than wrap, so a glance says whether the
  // stream has been clean since the Pico came up. `seq` steps per frame so
  // a stalled encoder is told from a stopped wheel: a stall repeats seq.
  struct Odom
  {
      UInt64 tMonoUs = 0;
      Int32 ticks = 0;
      Int16 ticksPerS = 0;
      UInt8 skips = 0;
      UInt8 invalid = 0;
      UInt8 seq = 0;
  };

  struct Control
  {
      UInt32 sessionId = 0;
      UInt32 seq = 0;      // strictly increasing from 1
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

  // The Trim pane's idle test. With ENABLE, while armed and live, the pilot
  // holds the ESC at exactly the Pico's idle pulse and ignores the throttle
  // field, and the board accepts SET_ESC_LIMITS despite the arm: the only
  // throttle it can change is that idle.
  constexpr UInt16 BUTTON_IDLE_TEST = 0x0008;

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

  // The highest camera rate the board honours. A request above it is clamped,
  // not refused, so asking for too much still gets a picture. 640x480 MJPEG at
  // this rate is more than a hotspot carries; it is safe to offer only because
  // CAMERA is CLASS_BULK.
  constexpr UInt16 CAM_FPS_MAX = 15;

  struct Subscribe
  {
      UInt32 sessionId = 0;
      UInt32 typeMask = 0;
      UInt16 scanDivisor = 1;   // 1 = every revolution, 3 = every third

      // The camera rate the viewer asks for; 0 is "did not ask" and keeps the
      // board's default. These two bytes were reserved, so the length is still
      // 12 and an older peer reads or sends 0.
      UInt16 camFps = 0;
  };

  // What a bundle needs before it may be loaded. The supervisor refuses a load
  // whose needs are not met, and a viewer greys the row with the reason rather
  // than hiding it: "no lidar" is the answer to why you cannot drive.
  constexpr UInt8 BUNDLE_NEEDS_LIDAR = 0x01;
  constexpr UInt8 BUNDLE_NEEDS_PICO = 0x02;
  constexpr UInt8 BUNDLE_NEEDS_CAMERA = 0x04;
  constexpr UInt8 BUNDLE_MAY_DRIVE = 0x08;

  // How the last bundle ended, for BundleState::exitKind.
  constexpr UInt8 BUNDLE_EXIT_NONE = 0;
  constexpr UInt8 BUNDLE_EXIT_OK = 1;
  constexpr UInt8 BUNDLE_EXIT_FAILED = 2;
  constexpr UInt8 BUNDLE_EXIT_SIGNALLED = 3;
  constexpr UInt8 BUNDLE_EXIT_REFUSED = 4;

  // ONE BUNDLE PER FRAME, the way SCAN is one revolution per frame. A viewer
  // collects `count` of them carrying the same `generation` to have the list;
  // a frame from an older generation is discarded rather than merged, so a list
  // is never half of one set and half of another.
  //
  // `count` 0 is the empty list, announced once with `index` 0, so a viewer can
  // tell "this board has no bundles" from "this board has not said yet".
  struct Bundle
  {
      UInt32 generation = 0;
      UInt16 index = 0;
      UInt16 count = 0;
      UInt8 needs = 0;    // BUNDLE_NEEDS_*
      UInt8 ready = 0;    // 0 when something in `needs` is missing
      UInt8 loaded = 0;   // 1 while this behaviour is in the host's chain
      Str id;             // reverse-DNS, stable; what a viewer keys its window on
      Str name;           // for a person
      Str about;          // one line
  };

  // The AGGREGATE: how many behaviours are loaded, and the last thing that
  // happened to one. Published on change and on connect, so a viewer joining
  // mid-run has both without waiting for an event it already missed.
  //
  // It used to describe ONE running bundle - upS, running, exitKind. Several
  // load at once now, so per-bundle state moved into Bundle::loaded and this
  // became the summary. Nothing moved on the wire: the offsets and widths are
  // unchanged and `ver` stays 1, because only the meaning of two fields
  // changed, on a type no reader has ever consumed.
  struct BundleState
  {
      UInt64 tMonoUs = 0;
      UInt32 loadedCount = 0;   // was upS
      UInt32 lastCode = 0;
      UInt8 anyLoaded = 0;      // was running; 1 while the chain is not empty
      UInt8 lastKind = BUNDLE_EXIT_NONE;
      Str id;                   // which bundle the last event concerned, or empty
      Str text;                 // what happened, for a person
  };

  struct Describe
  {
      UInt8 type = 0;   // 0 = all
  };

  struct Schema
  {
      Str text;
  };

  [[nodiscard]] Size writeBundle(const Bundle& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readBundle(const Body& b, UInt8 ver, Bundle* out);
  [[nodiscard]] Size writeBundleState(const BundleState& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readBundleState(const Body& b, UInt8 ver, BundleState* out);
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
  [[nodiscard]] Size writeTags(const Tags& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readTags(const Body& b, UInt8 ver, Tags* out);
  [[nodiscard]] Size writeOdom(const Odom& m, UInt8* out, Size cap);
  [[nodiscard]] Bool readOdom(const Body& b, UInt8 ver, Odom* out);
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

  // The catalog: one row per Type with its tag, name, version, drop class, body
  // length with every variable tail empty, size expression and field list.
  // SCHEMA and the tests' completeness check are generated from it. The .cxx
  // static_asserts that it covers ALL_TYPES and agrees with knownType() for all
  // 256 tags; a Type added to the enum and to neither list is the one case no
  // C++20 check can catch.
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

  // The one renderer, used by the viewer's log pane, the pilot's console and
  // the tests, so a frame prints the same everywhere. Sentinels render as
  // `n/a`, never as 65535 or -32768.
  [[nodiscard]] Str describe(const Frame& f);

  // protoMajor must be EQUAL. Not >=, not "compatible".
  [[nodiscard]] Bool versionOk(UInt16 protoMajor);

  // The refusal names both versions and the board's build stamp, so a person
  // in a field can act on it instead of reading source.
  [[nodiscard]] Bye versionRefusal(const Hello& h, CharSeq boardBuild);

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

        // "Parsed", not "received": bytes half-read in an accumulator are not a
        // command and must not count as liveness.
        Bool feedsDeadman = false;

        Int16 steerMilli = 0;
        Int16 throttleMilli = 0;
        Refuse refuse = Refuse::REFUSE_NONE;
        UInt32 highestSeq = 0;   // what the caller's high-water mark becomes
    };

    // static_cast<Int32>(a - b) > 0 on the UInt32s, so a viewer that restarted
    // at seq 1 is not frozen out by a huge old seq. The high-water mark is reset
    // by the handshake, per session: kept across a reconnect, it would ignore
    // every command from a restarted viewer while the socket looked perfect.
    [[nodiscard]] Bool newer(UInt32 a, UInt32 b);

    // Pure. Decides what this datagram may do, and whether it feeds the
    // deadman: only when it carries the current session AND came from the
    // current holder AND was applied. Otherwise a second viewer's stream would
    // keep the timer alive for a holder whose laptop just slept.
    [[nodiscard]] Outcome apply(const Gate& g, const Control& c);
  }

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

    // No clock inside it, so the safety property is tested without sleeping,
    // and the viewer runs this identical function on its own copy of the
    // numbers: the "neutral in 90 ms" an operator watches is the arithmetic
    // that trips.
    [[nodiscard]] Output step(const Inputs& in);

    [[nodiscard]] CharSeq stateName(State s);
  }

  // Names for a person, the same on the board and in the viewer.
  // driveModeName: Decide::mode (0 cruise .. 4 blind). pilotModeName: PilotMode,
  // as BoardState::pilotMode and Decide::source carry it. "?" when unknown.
  [[nodiscard]] CharSeq driveModeName(UInt8 mode);
  [[nodiscard]] CharSeq pilotModeName(UInt8 mode);

  [[nodiscard]] CharSeq refuseName(Refuse r);
}
