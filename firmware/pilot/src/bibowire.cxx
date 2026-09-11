#include "bibowire.hxx"

#include <bit>

namespace bibowire
{

  // Both ends are little-endian forever - aarch64 Linux on the board, x86-64
  // Windows in the viewer - and byte-swapping every field to buy a portability
  // nobody will use costs the Pi cycles it has better uses for. Asserted rather
  // than assumed, so the day somebody builds this for a big-endian target they
  // get a compile error instead of a scan drawn inside out.
  static_assert(
      std::endian::native == std::endian::little,
      "bibowire is little-endian on the wire; this target is not"
  );

  // The single cheapest safety artifact in this protocol. Every design that led
  // here confessed its deadman will make the car stutter outdoors and that
  // someone will eventually raise the constant because they were right that it
  // felt bad. This is the only thing standing in the way of that edit, and what
  // it protects is not the stop - it is the DIAGNOSABILITY of the stop.
  static_assert(
      CONTROL_DEAD_MS + PICO_HOP_BUDGET_MS <= PICO_DEADMAN_MS,
      "the Pi's stop must beat the Pico's, or the blunt layer fires first and prints ERR deadman onto a cable nobody is watching"
  );

  namespace
  {

    // ---- body lengths ------------------------------------------------------
    //
    // The constant term of each type's size expression: the body with every
    // variable tail empty. The catalog carries the same numbers and the test
    // suite checks them against what writeX actually produces, so a layout that
    // moves without its row moving is a failure rather than a stale document.

    constexpr Size HELLO_FIXED = 20;
    constexpr Size WELCOME_FIXED = 40;
    constexpr Size BYE_FIXED = 4;
    constexpr Size PING_LEN = 16;
    constexpr Size LEAVE_LEN = 8;
    constexpr Size SCAN_FIXED = 24;
    constexpr Size DECIDE_LEN = 24;
    constexpr Size BOARD_FIXED = 72;
    constexpr Size LIDAR_INFO_LEN = 32;
    constexpr Size EVENT_FIXED = 16;
    constexpr Size CTLSTATE_LEN = 44;
    constexpr Size CMDACK_FIXED = 12;
    constexpr Size CAMERA_FIXED = 24;
    constexpr Size POSE_LEN = 32;
    constexpr Size PATH_FIXED = 16;
    constexpr Size WAYPOINT_LEN = 32;
    constexpr Size CONTROL_LEN = 24;
    constexpr Size COMMAND_LEN = 16;
    constexpr Size SUBSCRIBE_LEN = 12;
    constexpr Size DESCRIBE_LEN = 4;
    constexpr Size SCHEMA_FIXED = 4;

    constexpr UInt16 CENTI_PER_TURN = 36000;
    constexpr UInt8 QUALITY_MAX = 63;
    constexpr Int16 MILLI_MAX = 1000;

    [[nodiscard]] Size scanBodyLen(Size count)
    {
        return SCAN_FIXED + 4u * count + padTo4(count);
    }

    [[nodiscard]] Size pathBodyLen(Size count)
    {
        return PATH_FIXED + 8u * count;
    }

    // A body is acceptable when it is EXACTLY what this version promises. A
    // HIGHER ver may carry a tail this build has no name for, and the tail is
    // ignored - which works precisely because `len` is authoritative and every
    // body is fixed-width up to the point a new field would begin.
    [[nodiscard]] Bool lenOk(const Body& b, UInt8 ver, Size need)
    {
        if(ver > 1)
        {
            return b.len >= need;
        }
        return b.len == need;
    }

    [[nodiscard]] Bool bodyUsable(const Body& b, UInt8 ver, Size fixed)
    {
        return b.bytes != nullptr && ver != 0 && b.len >= fixed;
    }

    // ---- CRC32C ------------------------------------------------------------
    //
    // Castagnoli reflected, 0x82F63B78. Built at compile time so the module has
    // no mutable state to initialize and nothing to race on: `constexpr` data
    // is not a global in the sense that matters, it is a constant in the image.

    [[nodiscard]] constexpr Array<UInt32, 256> buildCrcTable()
    {
        Array<UInt32, 256> t{};
        for(UInt32 i = 0; i < 256u; ++i)
        {
            UInt32 c = i;
            for(Int32 k = 0; k < 8; ++k)
            {
                const Bool low = (c & 1u) != 0u;
                c = low ? (0x82F63B78u ^ (c >> 1)) : (c >> 1);
            }
            t[i] = c;
        }
        return t;
    }

    constexpr Array<UInt32, 256> CRC_TABLE = buildCrcTable();

    // ---- text, built from integers by hand ---------------------------------
    //
    // NOT through the C library. snprintf's integer conversions happen to be
    // locale-independent today, but the whole point of a wire with no floats on
    // it is that there is no formatting step a locale can reach - and a
    // renderer that reaches for %f the day somebody adds a float would put the
    // bug straight back. Digits assembled here cannot.

    [[nodiscard]] Str decU(UInt64 v)
    {
        if(v == 0u)
        {
            return Str("0");
        }
        Array<Char, 24> buf{};
        Size n = buf.size();
        while(v > 0u)
        {
            --n;
            buf[n] = static_cast<Char>('0' + static_cast<Int32>(v % 10u));
            v /= 10u;
        }
        return Str(buf.data() + n, buf.size() - n);
    }

    [[nodiscard]] Str decI(Int64 v)
    {
        if(v < 0)
        {
            // Through UInt64 rather than -v: negating the most negative Int64
            // overflows, and a renderer is not where that should be discovered.
            const UInt64 mag = static_cast<UInt64>(-(v + 1)) + 1u;
            return Str("-") + decU(mag);
        }
        return decU(static_cast<UInt64>(v));
    }

    [[nodiscard]] Char hexLower(UInt32 v)
    {
        if(v < 10u)
        {
            return static_cast<Char>('0' + static_cast<Int32>(v));
        }
        return static_cast<Char>('a' + static_cast<Int32>(v - 10u));
    }

    [[nodiscard]] Char hexUpper(UInt32 v)
    {
        if(v < 10u)
        {
            return static_cast<Char>('0' + static_cast<Int32>(v));
        }
        return static_cast<Char>('A' + static_cast<Int32>(v - 10u));
    }

    [[nodiscard]] Str hexN(UInt64 v, Size digits)
    {
        Str s = "0x";
        for(Size i = digits; i > 0u; --i)
        {
            s += hexLower(static_cast<UInt32>((v >> ((i - 1u) * 4u)) & 0xFu));
        }
        return s;
    }

    // Centi-units as a decimal a person reads, assembled from integers. The
    // point of this file is that this dot is arithmetic and not a radix
    // character somebody's locale chose.
    [[nodiscard]] Str centi(Int32 v)
    {
        const Bool neg = v < 0;
        const Int32 mag = neg ? -v : v;
        Str s = neg ? Str("-") : Str();
        s += decU(static_cast<UInt64>(mag / 100));
        s += '.';
        s += static_cast<Char>('0' + (mag % 100) / 10);
        s += static_cast<Char>('0' + (mag % 100) % 10);
        return s;
    }

    // A sentence on the wire is bytes a stranger wrote. It is rendered between
    // quotes with its control characters flattened, so a log line stays one
    // line and a terminal escape in an EVENT cannot repaint somebody's screen.
    [[nodiscard]] Str quoteText(const Str& text)
    {
        Str s = "\"";
        for(const Char c : text)
        {
            const Bool printable = c >= 0x20 && c < 0x7F;
            if(c == '"' || c == '\\')
            {
                s += '\\';
                s += c;
            }
            else if(printable)
            {
                s += c;
            }
            else
            {
                s += '.';
            }
        }
        s += '"';
        return s;
    }

    [[nodiscard]] Str readText(const UInt8* p, Size n)
    {
        return Str(reinterpret_cast<const Char*>(p), n);
    }

    Void writeText(UInt8* p, const Str& text, Size padded)
    {
        std::memset(p, 0, padded);
        if(!text.empty())
        {
            std::memcpy(p, text.data(), text.size());
        }
    }

    // ---- names -------------------------------------------------------------

    [[nodiscard]] CharSeq healthName(UInt8 v)
    {
        if(v == 0u)
        {
            return "good";
        }
        if(v == 1u)
        {
            return "warn";
        }
        if(v == 2u)
        {
            return "error";
        }
        if(v == HEALTH_ABSENT)
        {
            return "n/a";
        }
        return "?";
    }

    [[nodiscard]] CharSeq driveModeName(UInt8 v)
    {
        switch(v)
        {
            case 0u:
                return "cruise";
            case 1u:
                return "slow";
            case 2u:
                return "stop";
            case 3u:
                return "reverse";
            case 4u:
                return "blind";
            default:
                return "?";
        }
    }

    [[nodiscard]] CharSeq pilotModeName(UInt8 v)
    {
        switch(v)
        {
            case 0u:
                return "manual";
            case 1u:
                return "look";
            case 2u:
                return "drive";
            default:
                return "?";
        }
    }

    [[nodiscard]] CharSeq deadmanName(UInt8 v)
    {
        switch(v)
        {
            case 0u:
                return "live";
            case 1u:
                return "soft";
            case 2u:
                return "dead";
            case 3u:
                return "estop";
            default:
                return "?";
        }
    }

    [[nodiscard]] CharSeq picoLinkName(UInt8 v)
    {
        switch(v)
        {
            case 0u:
                return "down";
            case 1u:
                return "up";
            case 2u:
                return "silent";
            default:
                return "?";
        }
    }

    [[nodiscard]] CharSeq holderName(UInt8 v)
    {
        switch(v)
        {
            case 0u:
                return "nobody";
            case 1u:
                return "you";
            case 2u:
                return "other";
            default:
                return "?";
        }
    }

    [[nodiscard]] CharSeq severityName(Severity s)
    {
        switch(s)
        {
            case Severity::SEVERITY_INFO:
                return "info";
            case Severity::SEVERITY_WARN:
                return "warn";
            case Severity::SEVERITY_ERROR:
                return "error";
            default:
                return "?";
        }
    }

    [[nodiscard]] CharSeq verbName(Verb v)
    {
        switch(v)
        {
            case Verb::VERB_NONE:
                return "none";
            case Verb::VERB_ARM:
                return "arm";
            case Verb::VERB_DISARM:
                return "disarm";
            case Verb::VERB_ESTOP:
                return "estop";
            case Verb::VERB_CLEAR_ESTOP:
                return "clear_estop";
            case Verb::VERB_MOTOR_ON:
                return "motor_on";
            case Verb::VERB_MOTOR_OFF:
                return "motor_off";
            case Verb::VERB_SET_MODE:
                return "set_mode";
            case Verb::VERB_SET_ESC_LIMITS:
                return "set_esc_limits";
            default:
                return "?";
        }
    }

    [[nodiscard]] CharSeq reasonName(Reason r)
    {
        switch(r)
        {
            case Reason::REASON_NONE:
                return "none";
            case Reason::REASON_VERSION:
                return "version";
            case Reason::REASON_TOO_BIG:
                return "too_big";
            case Reason::REASON_BAD_CRC:
                return "bad_crc";
            case Reason::REASON_BAD_SESSION:
                return "bad_session";
            case Reason::REASON_TIMEOUT:
                return "timeout";
            case Reason::REASON_SHUTDOWN:
                return "shutdown";
            case Reason::REASON_SUPERSEDED:
                return "superseded";
            case Reason::REASON_REFUSED:
                return "refused";
            case Reason::REASON_BAD_FLAG:
                return "bad_flag";
            default:
                return "?";
        }
    }

    // ---- the catalog -------------------------------------------------------

    constexpr Array<Type, TYPE_COUNT> ALL_TYPES = {
        Type::TYPE_HELLO, Type::TYPE_WELCOME, Type::TYPE_BYE, Type::TYPE_PING,
        Type::TYPE_PONG, Type::TYPE_LEAVE, Type::TYPE_SCAN, Type::TYPE_DECIDE,
        Type::TYPE_BOARD, Type::TYPE_LIDAR_INFO, Type::TYPE_EVENT, Type::TYPE_CTLSTATE,
        Type::TYPE_CMDACK, Type::TYPE_CAMERA, Type::TYPE_POSE, Type::TYPE_PATH,
        Type::TYPE_WAYPOINT, Type::TYPE_CONTROL, Type::TYPE_COMMAND, Type::TYPE_SUBSCRIBE,
        Type::TYPE_DESCRIBE, Type::TYPE_SCHEMA,
    };

    constexpr Array<Desc, TYPE_COUNT> CATALOG = {
        Desc{ Type::TYPE_HELLO, "HELLO", 1, Class::CLASS_VITAL, HELLO_FIXED, true, "20+name",
              "u16 protoMajor ; u16 protoMinor ; u32 featureMask ; u32 viewerBuild ; u16 viewerUdpPort ; u16 wantControl ; u16 controlHz ; u8 nameLen ; u8 reserved0 ; u8 name[nameLen]" },
        Desc{ Type::TYPE_WELCOME, "WELCOME", 1, Class::CLASS_VITAL, WELCOME_FIXED, true, "40+name+text",
              "u16 protoMajor ; u16 protoMinor ; u32 sessionId ; u32 bootId ; u32 featureMask ; u16 controlUdpPort ; u16 controlPeriodMs ms ; u16 staleMs ms ; u16 deadMs ms ; u16 accepted ; u16 refusal ; u64 boardMonoUs us ; u8 armEpoch ; u8 capabilities ; u8 nameLen ; u8 textLen ; u8 boardName[nameLen] ; u8 text[textLen]" },
        Desc{ Type::TYPE_BYE, "BYE", 1, Class::CLASS_VITAL, BYE_FIXED, true, "4+text",
              "u16 reason ; u16 textLen ; u8 text[textLen]" },
        Desc{ Type::TYPE_PING, "PING", 1, Class::CLASS_VITAL, PING_LEN, false, "16",
              "u64 token ; u64 senderMonoUs us" },
        Desc{ Type::TYPE_PONG, "PONG", 1, Class::CLASS_VITAL, PING_LEN, false, "16",
              "u64 token ; u64 senderMonoUs us" },
        Desc{ Type::TYPE_LEAVE, "LEAVE", 1, Class::CLASS_VITAL, LEAVE_LEN, false, "8",
              "u32 sessionId ; u32 reserved0" },
        Desc{ Type::TYPE_SCAN, "SCAN", 1, Class::CLASS_LIVE, SCAN_FIXED, true, "24+4n+n",
              "u64 tMonoUs us ; u32 revIndex ; u16 freqMilliHz mHz ; u16 count ; u8 health ; u8 motor ; u16 droppedSinceLast ; u16 scanDivisor ; u16 reserved0 ; { u16 angleCentiDeg cdeg ; u16 distMm mm }[n] ; u8 quality[n]" },
        Desc{ Type::TYPE_DECIDE, "DECIDE", 1, Class::CLASS_LIVE, DECIDE_LEN, false, "24",
              "u32 revIndex ; u32 clearanceMm mm ; u16 hits ; i16 steerMilli ; i16 throttleMilli ; u8 mode ; u8 stop ; u8 source ; u8 reserved0 ; u16 reserved1 ; u32 modeMs ms" },
        Desc{ Type::TYPE_BOARD, "BOARD", 1, Class::CLASS_VITAL, BOARD_FIXED, true, "72+wifi",
              "u64 tMonoUs us ; u32 upS s ; i16 cpuCentiC cC ; u16 battMilliV mV ; u8 picoLink ; u8 picoArmed ; u8 pilotMode ; u8 deadman ; u8 lidarHealth ; u8 lidarSpinning ; u8 armEpoch ; u8 controlHolder ; u32 loopWorstUs us ; u32 loopLateCount ; u32 picoSilentMs ms ; u32 revolutions ; u32 timeouts ; u32 txDroppedFrames ; u32 rxControl ; u32 rxControlStale ; u32 ipv4 ; u32 encodeAvgNs ns ; u32 encodeMaxNs ns ; u8 wifiNameLen ; u8 clients ; u16 reserved0 ; u8 wifiName[wifiNameLen]" },
        Desc{ Type::TYPE_LIDAR_INFO, "LIDAR_INFO", 1, Class::CLASS_VITAL, LIDAR_INFO_LEN, false, "32",
              "u16 model ; u8 fwMajor ; u8 fwMinor ; u16 hwRev ; u16 reserved0 ; u8 serial[16] ; u32 baud ; u32 reserved1" },
        Desc{ Type::TYPE_EVENT, "EVENT", 1, Class::CLASS_VITAL, EVENT_FIXED, true, "16+text",
              "u64 tMonoUs us ; u8 severity ; u8 code ; u16 textLen ; u16 droppedSince ; u16 reserved0 ; u8 text[textLen]" },
        Desc{ Type::TYPE_CTLSTATE, "CTLSTATE", 1, Class::CLASS_VITAL, CTLSTATE_LEN, false, "44",
              "u64 tMonoUs us ; u32 ackSeq ; u32 controlAgeMs ms ; i16 steerNowMilli ; i16 throttleMilli ; u16 escUs us ; u16 neutralInMs ms ; u16 disarmInMs ms ; u8 armed ; u8 armEpoch ; u8 deadman ; u8 refuse ; u8 holder ; u8 pilotMode ; u32 scanAgeMs ms ; u32 picoSilentMs ms ; u32 lastCmdId" },
        Desc{ Type::TYPE_CMDACK, "CMDACK", 1, Class::CLASS_VITAL, CMDACK_FIXED, true, "12+text",
              "u32 cmdId ; u8 verb ; u8 result ; u16 textLen ; u8 armEpoch ; u8 reserved0 ; u16 reserved1 ; u8 text[textLen]" },
        Desc{ Type::TYPE_CAMERA, "CAMERA", 1, Class::CLASS_BULK, CAMERA_FIXED, true, "24+bytes",
              "u64 tMonoUs us ; u32 frameIndex ; u16 width ; u16 height ; u8 codec ; u8 flags ; u16 reserved0 ; u32 byteLen ; u8 data[byteLen]" },
        Desc{ Type::TYPE_POSE, "POSE", 1, Class::CLASS_LIVE, POSE_LEN, false, "32",
              "u64 tMonoUs us ; i32 xMm mm ; i32 yMm mm ; i32 headingMilliRad mrad ; u32 sigmaXyMm mm ; u32 sigmaHeadingMilliRad mrad ; u8 source ; u8 valid ; u16 reserved0" },
        Desc{ Type::TYPE_PATH, "PATH", 1, Class::CLASS_VITAL, PATH_FIXED, true, "16+8n",
              "u64 tMonoUs us ; u32 seq ; u16 count ; u16 reserved0 ; { i32 xMm mm ; i32 yMm mm }[n]" },
        Desc{ Type::TYPE_WAYPOINT, "WAYPOINT", 1, Class::CLASS_VITAL, WAYPOINT_LEN, false, "32",
              "u64 tMonoUs us ; u32 seq ; u16 index ; u16 total ; i32 xMm mm ; i32 yMm mm ; u16 flags ; u16 reserved0 ; u32 reserved1" },
        Desc{ Type::TYPE_CONTROL, "CONTROL", 1, Class::CLASS_VITAL, CONTROL_LEN, false, "24",
              "u32 sessionId ; u32 seq ; u64 tMonoUs us ; i16 steerMilli ; i16 throttleMilli ; u16 buttons ; u8 armEpoch ; u8 assumedMode" },
        Desc{ Type::TYPE_COMMAND, "COMMAND", 1, Class::CLASS_VITAL, COMMAND_LEN, false, "16",
              "u32 sessionId ; u32 cmdId ; u8 verb ; u8 arg0 ; u16 arg1 ; u16 arg2 ; u8 armEpoch ; u8 reserved0" },
        Desc{ Type::TYPE_SUBSCRIBE, "SUBSCRIBE", 1, Class::CLASS_VITAL, SUBSCRIBE_LEN, false, "12",
              "u32 sessionId ; u32 typeMask ; u16 scanDivisor ; u16 camFps fps" },
        Desc{ Type::TYPE_DESCRIBE, "DESCRIBE", 1, Class::CLASS_VITAL, DESCRIBE_LEN, false, "4",
              "u8 type ; u8 reserved0 ; u16 reserved1" },
        Desc{ Type::TYPE_SCHEMA, "SCHEMA", 1, Class::CLASS_VITAL, SCHEMA_FIXED, true, "4+text",
              "u16 textLen ; u16 reserved0 ; u8 text[textLen]" },
    };

    [[nodiscard]] constexpr Bool catalogHas(UInt8 tag)
    {
        for(const Desc& d : CATALOG)
        {
            if(static_cast<UInt8>(d.type) == tag)
            {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] constexpr Bool catalogCoversEveryType()
    {
        for(const Type t : ALL_TYPES)
        {
            if(!catalogHas(static_cast<UInt8>(t)))
            {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] constexpr Bool catalogTagsUnique()
    {
        for(Size i = 0; i < CATALOG.size(); ++i)
        {
            for(Size j = i + 1u; j < CATALOG.size(); ++j)
            {
                if(CATALOG[i].type == CATALOG[j].type)
                {
                    return false;
                }
            }
        }
        return true;
    }

  }

  // knownType is a HAND-WRITTEN switch and not a lookup in the table, on
  // purpose: it is the second opinion. The static_asserts below require the two
  // to agree for all 256 tags, so a type added to one and forgotten in the
  // other does not compile. What no C++20 check can catch is an enumerator
  // added to Type and to NEITHER - written down rather than left to be found.
  Bool knownType(UInt8 tag)
  {
      switch(tag)
      {
          case 0x01:
          case 0x02:
          case 0x03:
          case 0x04:
          case 0x05:
          case 0x06:
          case 0x10:
          case 0x11:
          case 0x12:
          case 0x13:
          case 0x14:
          case 0x15:
          case 0x16:
          case 0x20:
          case 0x21:
          case 0x22:
          case 0x23:
          case 0x40:
          case 0x41:
          case 0x42:
          case 0xF0:
          case 0xF1:
              return true;
          default:
              return false;
      }
  }

  namespace
  {

    [[nodiscard]] constexpr Bool knownTypeAgreesWithCatalog()
    {
        for(Size i = 0; i < 256u; ++i)
        {
            const UInt8 tag = static_cast<UInt8>(i);
            Bool inTable = false;
            for(const Desc& d : CATALOG)
            {
                if(static_cast<UInt8>(d.type) == tag)
                {
                    inTable = true;
                }
            }
            Bool known = false;
            switch(tag)
            {
                case 0x01:
                case 0x02:
                case 0x03:
                case 0x04:
                case 0x05:
                case 0x06:
                case 0x10:
                case 0x11:
                case 0x12:
                case 0x13:
                case 0x14:
                case 0x15:
                case 0x16:
                case 0x20:
                case 0x21:
                case 0x22:
                case 0x23:
                case 0x40:
                case 0x41:
                case 0x42:
                case 0xF0:
                case 0xF1:
                    known = true;
                    break;
                default:
                    known = false;
                    break;
            }
            if(known != inTable)
            {
                return false;
            }
        }
        return true;
    }

  }

  static_assert(
      CATALOG.size() == TYPE_COUNT,
      "the catalog has one row per Type and TYPE_COUNT says how many"
  );
  static_assert(catalogTagsUnique(), "two catalog rows claim the same tag - a tag is never reused");
  static_assert(
      catalogCoversEveryType(),
      "a Type has no catalog row: add it, or DESCRIBE will not name it"
  );
  static_assert(
      knownTypeAgreesWithCatalog(),
      "knownType() and the catalog disagree about some tag"
  );

  const Desc* catalog()
  {
      return CATALOG.data();
  }

  Size catalogCount()
  {
      return CATALOG.size();
  }

  const Desc* descOf(Type t)
  {
      for(const Desc& d : CATALOG)
      {
          if(d.type == t)
          {
              return &d;
          }
      }
      return nullptr;
  }

  CharSeq typeName(Type t)
  {
      const Desc* d = descOf(t);
      return d != nullptr ? d->name : "UNKNOWN";
  }

  Class classOf(Type t)
  {
      const Desc* d = descOf(t);
      // An unknown tag is skipped by `len` and counted, never queued - so the
      // class it would have had never comes up. VITAL is the answer that cannot
      // silently drop something, which is the right way to be wrong here.
      return d != nullptr ? d->cls : Class::CLASS_VITAL;
  }

  // ---- framing ---------------------------------------------------------------

  UInt32 crc32c(const UInt8* data, Size len)
  {
      UInt32 c = 0xFFFFFFFFu;
      if(data == nullptr)
      {
          return c ^ 0xFFFFFFFFu;
      }
      for(Size i = 0; i < len; ++i)
      {
          c = CRC_TABLE[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
      }
      return c ^ 0xFFFFFFFFu;
  }

  Take take(const UInt8* buf, Size len, Frame* out, Size* consumed)
  {
      if(consumed == nullptr)
      {
          return Take::TAKE_NEED_MORE;
      }
      *consumed = 0;
      if(buf == nullptr || out == nullptr)
      {
          return Take::TAKE_NEED_MORE;
      }

      Size at = 0;
      for(;;)
      {
          Size p = at;
          while(p + 2u <= len && !(buf[p] == MAGIC_LO && buf[p + 1u] == MAGIC_HI))
          {
              ++p;
          }
          if(p + 2u > len)
          {
              // Nothing that could still become a frame. A lone trailing 0x42
              // is KEPT - it may be the first half of a magic whose second byte
              // has not arrived - and everything before it is junk that is
              // reported rather than dropped in silence.
              Size keep = 0;
              if(len > 0u && buf[len - 1u] == MAGIC_LO)
              {
                  keep = 1;
              }
              Size junk = len - keep;
              if(junk < at)
              {
                  junk = at;
              }
              if(junk > 0u)
              {
                  *consumed = junk;
                  return Take::TAKE_RESYNC;
              }
              return Take::TAKE_NEED_MORE;
          }

          at = p;
          const Size avail = len - at;
          if(avail < HEAD_BYTES)
          {
              if(at > 0u)
              {
                  *consumed = at;
                  return Take::TAKE_RESYNC;
              }
              return Take::TAKE_NEED_MORE;
          }

          const UInt8 tag = rd8(buf + at + 2u);
          const UInt8 ver = rd8(buf + at + 3u);
          const UInt16 flags = rd16(buf + at + 4u);
          const UInt16 seq = rd16(buf + at + 6u);
          const UInt32 plen = rd32(buf + at + 8u);

          // THE KNOWN-TYPE TEST IS A RESYNC FILTER, NOT AN ACCEPTANCE TEST, and
          // the document is in two minds about it: section 3 lists "type is a
          // known tag" among the things a resync candidate must satisfy, while
          // sections 5 and 7 promise that an unknown type is "skipped by exactly
          // len and counted, never fatal" - which is what lets an older viewer
          // keep driving a newer board. Both hold only if the tag is judged
          // where it matters and not where it does not.
          //
          // At the stream position (at == 0) we are already on a boundary the
          // previous frame proved, so the CRC alone is enough and a tag this
          // build has no name for is handed up with its length honoured. Deeper
          // in, we are guessing where a frame starts, and requiring a known tag
          // is most of what keeps a false lock near 2^-48.
          const Bool atBoundary = at == 0u;
          const Bool shaped = ver != 0u && (plen % 4u) == 0u && (atBoundary || knownType(tag));

          if(shaped && plen > MAX_PAYLOAD)
          {
              // A peer whose TAG WE KNOW, at the stream position, telling us how
              // much memory to find: that is the hostile-or-broken case, and it
              // is answered rather than skipped past. Nothing is allocated for
              // the claim either way.
              //
              // The known-tag half is load-bearing and was learned the hard way:
              // without it, four bytes of junk that happen to spell the magic
              // and a nonzero ver close a healthy connection instead of being
              // resynced past. An unknown tag here is only junk wearing a
              // header, so it is rejected like any other bad candidate.
              if(atBoundary && knownType(tag))
              {
                  return Take::TAKE_TOO_BIG;
              }
              ++at;
              continue;
          }
          if(!shaped)
          {
              ++at;
              continue;
          }
          if((flags & FLAG_MORE) != 0u)
          {
              // A flag that changes FRAMING may not be ignored by a reader that
              // cannot honour it. FLAG_MORE is reserved for fragmenting a
              // future camera frame and must be 0 in v1.
              //
              // Guarded by the known tag for the same reason TAKE_TOO_BIG is:
              // refusing a connection because junk set a bit is a worse failure
              // than the one the refusal exists to catch.
              if(atBoundary && knownType(tag))
              {
                  return Take::TAKE_BAD_FLAG;
              }
              ++at;
              continue;
          }

          const Size total = FRAME_OVERHEAD + static_cast<Size>(plen);
          if(avail < total)
          {
              if(at > 0u)
              {
                  *consumed = at;
                  return Take::TAKE_RESYNC;
              }
              return Take::TAKE_NEED_MORE;
          }

          const UInt32 want = rd32(buf + at + HEAD_BYTES + plen);
          if(crc32c(buf + at, HEAD_BYTES + plen) != want)
          {
              ++at;
              continue;
          }

          // Junk first, frame next call. One answer per call is what keeps
          // `consumed` unambiguous, and an ambiguous consumed count is how a
          // ring buffer loses a frame nobody can account for.
          if(at > 0u)
          {
              *consumed = at;
              return Take::TAKE_RESYNC;
          }

          out->head.type = static_cast<Type>(tag);
          out->head.ver = ver;
          out->head.flags = flags;
          out->head.seq = seq;
          out->body.bytes = buf + at + HEAD_BYTES;
          out->body.len = plen;
          *consumed = total;
          return Take::TAKE_FRAME;
      }
  }

  Size put(const Head& h, const Body& b, UInt8* out, Size cap)
  {
      if(out == nullptr || h.ver == 0u)
      {
          return 0;
      }
      if(b.len > MAX_PAYLOAD || (b.len % 4u) != 0u)
      {
          return 0;
      }
      if(b.len > 0u && b.bytes == nullptr)
      {
          return 0;
      }
      // A v1 sender never sets FLAG_MORE, and building a frame every v1 reader
      // is required to refuse is a bug worth failing at the encoder. The tests
      // reach past this by patching the byte, which is also how they prove
      // take()'s check is real rather than a mirror of this one.
      if((h.flags & FLAG_MORE) != 0u)
      {
          return 0;
      }
      const Size total = FRAME_OVERHEAD + b.len;
      if(cap < total)
      {
          return 0;
      }

      wr16(out, MAGIC);
      wr8(out + 2u, static_cast<UInt8>(h.type));
      wr8(out + 3u, h.ver);
      wr16(out + 4u, h.flags);
      wr16(out + 6u, h.seq);
      wr32(out + 8u, static_cast<UInt32>(b.len));
      // An encoder may build its body straight into the frame buffer and hand
      // that pointer back, in which case there is nothing to copy.
      if(b.len > 0u && b.bytes != out + HEAD_BYTES)
      {
          std::memcpy(out + HEAD_BYTES, b.bytes, b.len);
      }
      wr32(out + HEAD_BYTES + b.len, crc32c(out, HEAD_BYTES + b.len));
      return total;
  }

  // ---- HELLO -----------------------------------------------------------------

  Size writeHello(const Hello& m, UInt8* out, Size cap)
  {
      if(out == nullptr || m.name.size() > MAX_NAME)
      {
          return 0;
      }
      const Size need = HELLO_FIXED + padTo4(m.name.size());
      if(cap < need)
      {
          return 0;
      }
      wr16(out, m.protoMajor);
      wr16(out + 2u, m.protoMinor);
      wr32(out + 4u, m.featureMask);
      wr32(out + 8u, m.viewerBuild);
      wr16(out + 12u, m.viewerUdpPort);
      wr16(out + 14u, m.wantControl);
      wr16(out + 16u, m.controlHz);
      wr8(out + 18u, static_cast<UInt8>(m.name.size()));
      wr8(out + 19u, 0);
      writeText(out + HELLO_FIXED, m.name, padTo4(m.name.size()));
      return need;
  }

  Bool readHello(const Body& b, UInt8 ver, Hello* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, HELLO_FIXED))
      {
          return false;
      }
      const Size nameLen = rd8(b.bytes + 18u);
      if(nameLen > MAX_NAME)
      {
          return false;
      }
      if(!lenOk(b, ver, HELLO_FIXED + padTo4(nameLen)))
      {
          return false;
      }
      Hello m;
      m.protoMajor = rd16(b.bytes);
      m.protoMinor = rd16(b.bytes + 2u);
      m.featureMask = rd32(b.bytes + 4u);
      m.viewerBuild = rd32(b.bytes + 8u);
      m.viewerUdpPort = rd16(b.bytes + 12u);
      m.wantControl = rd16(b.bytes + 14u);
      m.controlHz = rd16(b.bytes + 16u);
      m.name = readText(b.bytes + HELLO_FIXED, nameLen);
      *out = m;
      return true;
  }

  // ---- WELCOME ---------------------------------------------------------------

  Size writeWelcome(const Welcome& m, UInt8* out, Size cap)
  {
      if(out == nullptr || m.boardName.size() > MAX_NAME || m.text.size() > MAX_BOARD_TEXT)
      {
          return 0;
      }
      // sessionId is NEVER 0: every CONTROL datagram carries it and the board
      // drops any whose id is not the current one, so 0 has to stay available
      // as "no session" on the receiving side.
      if(m.sessionId == 0u)
      {
          return 0;
      }
      const Size tail = m.boardName.size() + m.text.size();
      const Size need = WELCOME_FIXED + padTo4(tail);
      if(cap < need)
      {
          return 0;
      }
      wr16(out, m.protoMajor);
      wr16(out + 2u, m.protoMinor);
      wr32(out + 4u, m.sessionId);
      wr32(out + 8u, m.bootId);
      wr32(out + 12u, m.featureMask);
      wr16(out + 16u, m.controlUdpPort);
      wr16(out + 18u, m.controlPeriodMs);
      wr16(out + 20u, m.staleMs);
      wr16(out + 22u, m.deadMs);
      wr16(out + 24u, m.accepted);
      wr16(out + 26u, m.refusal);
      // Offset 28 for a u64 is the document's own layout and is NOT 8-aligned.
      // It is correct here only because every field goes through memcpy; a
      // struct laid over this payload would be undefined on a machine that
      // faults. See the note at the top of the header.
      wr64(out + 28u, m.boardMonoUs);
      wr8(out + 36u, m.armEpoch);
      wr8(out + 37u, m.capabilities);
      wr8(out + 38u, static_cast<UInt8>(m.boardName.size()));
      wr8(out + 39u, static_cast<UInt8>(m.text.size()));
      std::memset(out + WELCOME_FIXED, 0, padTo4(tail));
      if(!m.boardName.empty())
      {
          std::memcpy(out + WELCOME_FIXED, m.boardName.data(), m.boardName.size());
      }
      if(!m.text.empty())
      {
          std::memcpy(out + WELCOME_FIXED + m.boardName.size(), m.text.data(), m.text.size());
      }
      return need;
  }

  Bool readWelcome(const Body& b, UInt8 ver, Welcome* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, WELCOME_FIXED))
      {
          return false;
      }
      const Size nameLen = rd8(b.bytes + 38u);
      const Size textLen = rd8(b.bytes + 39u);
      if(nameLen > MAX_NAME || textLen > MAX_BOARD_TEXT)
      {
          return false;
      }
      if(!lenOk(b, ver, WELCOME_FIXED + padTo4(nameLen + textLen)))
      {
          return false;
      }
      const UInt32 sessionId = rd32(b.bytes + 4u);
      if(sessionId == 0u)
      {
          return false;
      }
      Welcome m;
      m.protoMajor = rd16(b.bytes);
      m.protoMinor = rd16(b.bytes + 2u);
      m.sessionId = sessionId;
      m.bootId = rd32(b.bytes + 8u);
      m.featureMask = rd32(b.bytes + 12u);
      m.controlUdpPort = rd16(b.bytes + 16u);
      m.controlPeriodMs = rd16(b.bytes + 18u);
      m.staleMs = rd16(b.bytes + 20u);
      m.deadMs = rd16(b.bytes + 22u);
      m.accepted = rd16(b.bytes + 24u);
      m.refusal = rd16(b.bytes + 26u);
      m.boardMonoUs = rd64(b.bytes + 28u);
      m.armEpoch = rd8(b.bytes + 36u);
      m.capabilities = rd8(b.bytes + 37u);
      m.boardName = readText(b.bytes + WELCOME_FIXED, nameLen);
      m.text = readText(b.bytes + WELCOME_FIXED + nameLen, textLen);
      *out = m;
      return true;
  }

  // ---- BYE -------------------------------------------------------------------

  Size writeBye(const Bye& m, UInt8* out, Size cap)
  {
      if(out == nullptr || m.text.size() > 0xFFFFu)
      {
          return 0;
      }
      const Size need = BYE_FIXED + padTo4(m.text.size());
      if(cap < need || need > MAX_PAYLOAD)
      {
          return 0;
      }
      wr16(out, static_cast<UInt16>(m.reason));
      wr16(out + 2u, static_cast<UInt16>(m.text.size()));
      writeText(out + BYE_FIXED, m.text, padTo4(m.text.size()));
      return need;
  }

  Bool readBye(const Body& b, UInt8 ver, Bye* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, BYE_FIXED))
      {
          return false;
      }
      const Size textLen = rd16(b.bytes + 2u);
      if(!lenOk(b, ver, BYE_FIXED + padTo4(textLen)))
      {
          return false;
      }
      Bye m;
      m.reason = static_cast<Reason>(rd16(b.bytes));
      m.text = readText(b.bytes + BYE_FIXED, textLen);
      *out = m;
      return true;
  }

  // ---- PING / PONG -----------------------------------------------------------

  Size writePing(const Ping& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < PING_LEN)
      {
          return 0;
      }
      wr64(out, m.token);
      wr64(out + 8u, m.senderMonoUs);
      return PING_LEN;
  }

  Bool readPing(const Body& b, UInt8 ver, Ping* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, PING_LEN) || !lenOk(b, ver, PING_LEN))
      {
          return false;
      }
      Ping m;
      m.token = rd64(b.bytes);
      m.senderMonoUs = rd64(b.bytes + 8u);
      *out = m;
      return true;
  }

  // ---- LEAVE -----------------------------------------------------------------

  Size writeLeave(const Leave& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < LEAVE_LEN)
      {
          return 0;
      }
      wr32(out, m.sessionId);
      wr32(out + 4u, 0);
      return LEAVE_LEN;
  }

  Bool readLeave(const Body& b, UInt8 ver, Leave* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, LEAVE_LEN) || !lenOk(b, ver, LEAVE_LEN))
      {
          return false;
      }
      Leave m;
      m.sessionId = rd32(b.bytes);
      m.reserved0 = rd32(b.bytes + 4u);
      *out = m;
      return true;
  }

  // ---- SCAN ------------------------------------------------------------------

  Size writeScan(const Scan& m, UInt8* out, Size cap)
  {
      const Size count = m.points.size();
      // A count is a promise about the frame, and the two arrays are that
      // promise made twice. A quality array of a different length is a bug in
      // the caller, not a shorter revolution.
      if(out == nullptr || count > MAX_SCAN_POINTS || m.quality.size() != count)
      {
          return 0;
      }
      const Size need = scanBodyLen(count);
      if(cap < need)
      {
          return 0;
      }
      // Validated whole before a byte is written, so a refused encode leaves
      // the caller's buffer exactly as it found it.
      for(Size i = 0; i < count; ++i)
      {
          if(m.points[i].angleCentiDeg >= CENTI_PER_TURN || m.quality[i] > QUALITY_MAX)
          {
              return 0;
          }
      }
      wr64(out, m.tMonoUs);
      wr32(out + 8u, m.revIndex);
      wr16(out + 12u, m.freqMilliHz);
      wr16(out + 14u, static_cast<UInt16>(count));
      wr8(out + 16u, m.health);
      wr8(out + 17u, m.motor);
      wr16(out + 18u, m.droppedSinceLast);
      wr16(out + 20u, m.scanDivisor);
      wr16(out + 22u, 0);
      for(Size i = 0; i < count; ++i)
      {
          wr16(out + SCAN_FIXED + i * 4u, m.points[i].angleCentiDeg);
          wr16(out + SCAN_FIXED + i * 4u + 2u, m.points[i].distMm);
      }
      UInt8* qual = out + SCAN_FIXED + count * 4u;
      std::memset(qual, 0, padTo4(count));
      if(count > 0u)
      {
          std::memcpy(qual, m.quality.data(), count);
      }
      return need;
  }

  Bool readScan(const Body& b, UInt8 ver, Scan* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, SCAN_FIXED))
      {
          return false;
      }
      const Size count = rd16(b.bytes + 14u);
      if(count > MAX_SCAN_POINTS)
      {
          return false;
      }
      // A SCAN whose count disagrees with its len is refused OUTRIGHT. A frame
      // with the wrong count is not a shorter frame.
      if(!lenOk(b, ver, scanBodyLen(count)))
      {
          return false;
      }
      const UInt8* qual = b.bytes + SCAN_FIXED + count * 4u;
      for(Size i = 0; i < count; ++i)
      {
          if(rd16(b.bytes + SCAN_FIXED + i * 4u) >= CENTI_PER_TURN || qual[i] > QUALITY_MAX)
          {
              return false;
          }
      }
      Scan m;
      m.tMonoUs = rd64(b.bytes);
      m.revIndex = rd32(b.bytes + 8u);
      m.freqMilliHz = rd16(b.bytes + 12u);
      m.health = rd8(b.bytes + 16u);
      m.motor = rd8(b.bytes + 17u);
      m.droppedSinceLast = rd16(b.bytes + 18u);
      m.scanDivisor = rd16(b.bytes + 20u);
      m.points.resize(count);
      m.quality.resize(count);
      for(Size i = 0; i < count; ++i)
      {
          m.points[i].angleCentiDeg = rd16(b.bytes + SCAN_FIXED + i * 4u);
          m.points[i].distMm = rd16(b.bytes + SCAN_FIXED + i * 4u + 2u);
          m.quality[i] = qual[i];
      }
      *out = m;
      return true;
  }

  // ---- DECIDE ----------------------------------------------------------------

  Size writeDecide(const Decide& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < DECIDE_LEN)
      {
          return 0;
      }
      if(m.steerMilli < -MILLI_MAX || m.steerMilli > MILLI_MAX)
      {
          return 0;
      }
      if(m.throttleMilli < -MILLI_MAX || m.throttleMilli > MILLI_MAX)
      {
          return 0;
      }
      wr32(out, m.revIndex);
      wr32(out + 4u, m.clearanceMm);
      wr16(out + 8u, m.hits);
      wr16(out + 10u, static_cast<UInt16>(m.steerMilli));
      wr16(out + 12u, static_cast<UInt16>(m.throttleMilli));
      wr8(out + 14u, m.mode);
      wr8(out + 15u, m.stop);
      wr8(out + 16u, m.source);
      wr8(out + 17u, 0);
      wr16(out + 18u, 0);
      wr32(out + 20u, m.modeMs);
      return DECIDE_LEN;
  }

  Bool readDecide(const Body& b, UInt8 ver, Decide* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, DECIDE_LEN) || !lenOk(b, ver, DECIDE_LEN))
      {
          return false;
      }
      const Int16 steer = static_cast<Int16>(rd16(b.bytes + 10u));
      const Int16 throttle = static_cast<Int16>(rd16(b.bytes + 12u));
      if(steer < -MILLI_MAX || steer > MILLI_MAX || throttle < -MILLI_MAX || throttle > MILLI_MAX)
      {
          return false;
      }
      Decide m;
      m.revIndex = rd32(b.bytes);
      m.clearanceMm = rd32(b.bytes + 4u);
      m.hits = rd16(b.bytes + 8u);
      m.steerMilli = steer;
      m.throttleMilli = throttle;
      m.mode = rd8(b.bytes + 14u);
      m.stop = rd8(b.bytes + 15u);
      m.source = rd8(b.bytes + 16u);
      m.modeMs = rd32(b.bytes + 20u);
      *out = m;
      return true;
  }

  // ---- BOARD -----------------------------------------------------------------

  Size writeBoard(const BoardState& m, UInt8* out, Size cap)
  {
      if(out == nullptr || m.wifiName.size() > MAX_WIFI_NAME)
      {
          return 0;
      }
      const Size need = BOARD_FIXED + padTo4(m.wifiName.size());
      if(cap < need)
      {
          return 0;
      }
      wr64(out, m.tMonoUs);
      wr32(out + 8u, m.upS);
      wr16(out + 12u, static_cast<UInt16>(m.cpuCentiC));
      wr16(out + 14u, m.battMilliV);
      wr8(out + 16u, m.picoLink);
      wr8(out + 17u, m.picoArmed);
      wr8(out + 18u, m.pilotMode);
      wr8(out + 19u, m.deadman);
      wr8(out + 20u, m.lidarHealth);
      wr8(out + 21u, m.lidarSpinning);
      wr8(out + 22u, m.armEpoch);
      wr8(out + 23u, m.controlHolder);
      wr32(out + 24u, m.loopWorstUs);
      wr32(out + 28u, m.loopLateCount);
      wr32(out + 32u, m.picoSilentMs);
      wr32(out + 36u, m.revolutions);
      wr32(out + 40u, m.timeouts);
      wr32(out + 44u, m.txDroppedFrames);
      wr32(out + 48u, m.rxControl);
      wr32(out + 52u, m.rxControlStale);
      wr32(out + 56u, m.ipv4);
      wr32(out + 60u, m.encodeAvgNs);
      wr32(out + 64u, m.encodeMaxNs);
      wr8(out + 68u, static_cast<UInt8>(m.wifiName.size()));
      wr8(out + 69u, m.clients);
      wr16(out + 70u, 0);
      writeText(out + BOARD_FIXED, m.wifiName, padTo4(m.wifiName.size()));
      return need;
  }

  Bool readBoard(const Body& b, UInt8 ver, BoardState* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, BOARD_FIXED))
      {
          return false;
      }
      const Size wifiLen = rd8(b.bytes + 68u);
      if(wifiLen > MAX_WIFI_NAME)
      {
          return false;
      }
      if(!lenOk(b, ver, BOARD_FIXED + padTo4(wifiLen)))
      {
          return false;
      }
      BoardState m;
      m.tMonoUs = rd64(b.bytes);
      m.upS = rd32(b.bytes + 8u);
      m.cpuCentiC = static_cast<Int16>(rd16(b.bytes + 12u));
      m.battMilliV = rd16(b.bytes + 14u);
      m.picoLink = rd8(b.bytes + 16u);
      m.picoArmed = rd8(b.bytes + 17u);
      m.pilotMode = rd8(b.bytes + 18u);
      m.deadman = rd8(b.bytes + 19u);
      m.lidarHealth = rd8(b.bytes + 20u);
      m.lidarSpinning = rd8(b.bytes + 21u);
      m.armEpoch = rd8(b.bytes + 22u);
      m.controlHolder = rd8(b.bytes + 23u);
      m.loopWorstUs = rd32(b.bytes + 24u);
      m.loopLateCount = rd32(b.bytes + 28u);
      m.picoSilentMs = rd32(b.bytes + 32u);
      m.revolutions = rd32(b.bytes + 36u);
      m.timeouts = rd32(b.bytes + 40u);
      m.txDroppedFrames = rd32(b.bytes + 44u);
      m.rxControl = rd32(b.bytes + 48u);
      m.rxControlStale = rd32(b.bytes + 52u);
      m.ipv4 = rd32(b.bytes + 56u);
      m.encodeAvgNs = rd32(b.bytes + 60u);
      m.encodeMaxNs = rd32(b.bytes + 64u);
      m.clients = rd8(b.bytes + 69u);
      m.wifiName = readText(b.bytes + BOARD_FIXED, wifiLen);
      *out = m;
      return true;
  }

  // ---- LIDAR_INFO ------------------------------------------------------------

  Size writeLidarInfo(const LidarInfo& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < LIDAR_INFO_LEN)
      {
          return 0;
      }
      wr16(out, m.model);
      wr8(out + 2u, m.fwMajor);
      wr8(out + 3u, m.fwMinor);
      wr16(out + 4u, m.hwRev);
      wr16(out + 6u, 0);
      std::memcpy(out + 8u, m.serial.data(), m.serial.size());
      wr32(out + 24u, m.baud);
      wr32(out + 28u, 0);
      return LIDAR_INFO_LEN;
  }

  Bool readLidarInfo(const Body& b, UInt8 ver, LidarInfo* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, LIDAR_INFO_LEN) || !lenOk(b, ver, LIDAR_INFO_LEN))
      {
          return false;
      }
      LidarInfo m;
      m.model = rd16(b.bytes);
      m.fwMajor = rd8(b.bytes + 2u);
      m.fwMinor = rd8(b.bytes + 3u);
      m.hwRev = rd16(b.bytes + 4u);
      std::memcpy(m.serial.data(), b.bytes + 8u, m.serial.size());
      m.baud = rd32(b.bytes + 24u);
      *out = m;
      return true;
  }

  // ---- EVENT -----------------------------------------------------------------

  Size writeEvent(const Event& m, UInt8* out, Size cap)
  {
      if(out == nullptr || m.text.size() > MAX_EVENT_TEXT)
      {
          return 0;
      }
      const Size need = EVENT_FIXED + padTo4(m.text.size());
      if(cap < need)
      {
          return 0;
      }
      wr64(out, m.tMonoUs);
      wr8(out + 8u, static_cast<UInt8>(m.severity));
      wr8(out + 9u, m.code);
      wr16(out + 10u, static_cast<UInt16>(m.text.size()));
      wr16(out + 12u, m.droppedSince);
      wr16(out + 14u, 0);
      writeText(out + EVENT_FIXED, m.text, padTo4(m.text.size()));
      return need;
  }

  Bool readEvent(const Body& b, UInt8 ver, Event* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, EVENT_FIXED))
      {
          return false;
      }
      const Size textLen = rd16(b.bytes + 10u);
      if(textLen > MAX_EVENT_TEXT)
      {
          return false;
      }
      if(!lenOk(b, ver, EVENT_FIXED + padTo4(textLen)))
      {
          return false;
      }
      const UInt8 sev = rd8(b.bytes + 8u);
      if(sev > static_cast<UInt8>(Severity::SEVERITY_ERROR))
      {
          return false;
      }
      Event m;
      m.tMonoUs = rd64(b.bytes);
      m.severity = static_cast<Severity>(sev);
      m.code = rd8(b.bytes + 9u);
      m.droppedSince = rd16(b.bytes + 12u);
      m.text = readText(b.bytes + EVENT_FIXED, textLen);
      *out = m;
      return true;
  }

  // ---- CTLSTATE --------------------------------------------------------------

  Size writeCtlState(const CtlState& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < CTLSTATE_LEN)
      {
          return 0;
      }
      wr64(out, m.tMonoUs);
      wr32(out + 8u, m.ackSeq);
      wr32(out + 12u, m.controlAgeMs);
      wr16(out + 16u, static_cast<UInt16>(m.steerNowMilli));
      wr16(out + 18u, static_cast<UInt16>(m.throttleMilli));
      wr16(out + 20u, m.escUs);
      wr16(out + 22u, m.neutralInMs);
      wr16(out + 24u, m.disarmInMs);
      wr8(out + 26u, m.armed);
      wr8(out + 27u, m.armEpoch);
      wr8(out + 28u, m.deadman);
      wr8(out + 29u, static_cast<UInt8>(m.refuse));
      wr8(out + 30u, m.holder);
      wr8(out + 31u, m.pilotMode);
      wr32(out + 32u, m.scanAgeMs);
      wr32(out + 36u, m.picoSilentMs);
      wr32(out + 40u, m.lastCmdId);
      return CTLSTATE_LEN;
  }

  Bool readCtlState(const Body& b, UInt8 ver, CtlState* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, CTLSTATE_LEN) || !lenOk(b, ver, CTLSTATE_LEN))
      {
          return false;
      }
      const UInt8 refuse = rd8(b.bytes + 29u);
      if(refuse > static_cast<UInt8>(Refuse::REFUSE_NO_UDP))
      {
          return false;
      }
      CtlState m;
      m.tMonoUs = rd64(b.bytes);
      m.ackSeq = rd32(b.bytes + 8u);
      m.controlAgeMs = rd32(b.bytes + 12u);
      m.steerNowMilli = static_cast<Int16>(rd16(b.bytes + 16u));
      m.throttleMilli = static_cast<Int16>(rd16(b.bytes + 18u));
      m.escUs = rd16(b.bytes + 20u);
      m.neutralInMs = rd16(b.bytes + 22u);
      m.disarmInMs = rd16(b.bytes + 24u);
      m.armed = rd8(b.bytes + 26u);
      m.armEpoch = rd8(b.bytes + 27u);
      m.deadman = rd8(b.bytes + 28u);
      m.refuse = static_cast<Refuse>(refuse);
      m.holder = rd8(b.bytes + 30u);
      m.pilotMode = rd8(b.bytes + 31u);
      m.scanAgeMs = rd32(b.bytes + 32u);
      m.picoSilentMs = rd32(b.bytes + 36u);
      m.lastCmdId = rd32(b.bytes + 40u);
      *out = m;
      return true;
  }

  // ---- CMDACK ----------------------------------------------------------------

  Size writeCmdAck(const CmdAck& m, UInt8* out, Size cap)
  {
      if(out == nullptr || m.text.size() > 0xFFFFu)
      {
          return 0;
      }
      const Size need = CMDACK_FIXED + padTo4(m.text.size());
      if(cap < need || need > MAX_PAYLOAD)
      {
          return 0;
      }
      wr32(out, m.cmdId);
      wr8(out + 4u, static_cast<UInt8>(m.verb));
      wr8(out + 5u, m.result);
      wr16(out + 6u, static_cast<UInt16>(m.text.size()));
      wr8(out + 8u, m.armEpoch);
      wr8(out + 9u, 0);
      wr16(out + 10u, 0);
      writeText(out + CMDACK_FIXED, m.text, padTo4(m.text.size()));
      return need;
  }

  Bool readCmdAck(const Body& b, UInt8 ver, CmdAck* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, CMDACK_FIXED))
      {
          return false;
      }
      const Size textLen = rd16(b.bytes + 6u);
      if(!lenOk(b, ver, CMDACK_FIXED + padTo4(textLen)))
      {
          return false;
      }
      CmdAck m;
      m.cmdId = rd32(b.bytes);
      m.verb = static_cast<Verb>(rd8(b.bytes + 4u));
      m.result = rd8(b.bytes + 5u);
      m.armEpoch = rd8(b.bytes + 8u);
      m.text = readText(b.bytes + CMDACK_FIXED, textLen);
      *out = m;
      return true;
  }

  // ---- CAMERA ----------------------------------------------------------------

  Size writeCamera(const Camera& m, UInt8* out, Size cap)
  {
      if(out == nullptr)
      {
          return 0;
      }
      const Size need = CAMERA_FIXED + padTo4(m.data.size());
      if(need > MAX_PAYLOAD || cap < need)
      {
          return 0;
      }
      wr64(out, m.tMonoUs);
      wr32(out + 8u, m.frameIndex);
      wr16(out + 12u, m.width);
      wr16(out + 14u, m.height);
      wr8(out + 16u, m.codec);
      wr8(out + 17u, m.flags);
      wr16(out + 18u, 0);
      wr32(out + 20u, static_cast<UInt32>(m.data.size()));
      std::memset(out + CAMERA_FIXED, 0, padTo4(m.data.size()));
      if(!m.data.empty())
      {
          std::memcpy(out + CAMERA_FIXED, m.data.data(), m.data.size());
      }
      return need;
  }

  Bool readCamera(const Body& b, UInt8 ver, Camera* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, CAMERA_FIXED))
      {
          return false;
      }
      const Size byteLen = rd32(b.bytes + 20u);
      if(byteLen > MAX_PAYLOAD)
      {
          return false;
      }
      if(!lenOk(b, ver, CAMERA_FIXED + padTo4(byteLen)))
      {
          return false;
      }
      Camera m;
      m.tMonoUs = rd64(b.bytes);
      m.frameIndex = rd32(b.bytes + 8u);
      m.width = rd16(b.bytes + 12u);
      m.height = rd16(b.bytes + 14u);
      m.codec = rd8(b.bytes + 16u);
      m.flags = rd8(b.bytes + 17u);
      m.data.assign(b.bytes + CAMERA_FIXED, b.bytes + CAMERA_FIXED + byteLen);
      *out = m;
      return true;
  }

  // ---- POSE ------------------------------------------------------------------

  Size writePose(const Pose& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < POSE_LEN)
      {
          return 0;
      }
      wr64(out, m.tMonoUs);
      wr32(out + 8u, static_cast<UInt32>(m.xMm));
      wr32(out + 12u, static_cast<UInt32>(m.yMm));
      wr32(out + 16u, static_cast<UInt32>(m.headingMilliRad));
      wr32(out + 20u, m.sigmaXyMm);
      wr32(out + 24u, m.sigmaHeadingMilliRad);
      wr8(out + 28u, m.source);
      wr8(out + 29u, m.valid);
      wr16(out + 30u, 0);
      return POSE_LEN;
  }

  Bool readPose(const Body& b, UInt8 ver, Pose* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, POSE_LEN) || !lenOk(b, ver, POSE_LEN))
      {
          return false;
      }
      Pose m;
      m.tMonoUs = rd64(b.bytes);
      m.xMm = static_cast<Int32>(rd32(b.bytes + 8u));
      m.yMm = static_cast<Int32>(rd32(b.bytes + 12u));
      m.headingMilliRad = static_cast<Int32>(rd32(b.bytes + 16u));
      m.sigmaXyMm = rd32(b.bytes + 20u);
      m.sigmaHeadingMilliRad = rd32(b.bytes + 24u);
      m.source = rd8(b.bytes + 28u);
      m.valid = rd8(b.bytes + 29u);
      *out = m;
      return true;
  }

  // ---- PATH ------------------------------------------------------------------

  Size writePath(const Path& m, UInt8* out, Size cap)
  {
      const Size count = m.points.size();
      if(out == nullptr || count > 0xFFFFu)
      {
          return 0;
      }
      const Size need = pathBodyLen(count);
      if(need > MAX_PAYLOAD || cap < need)
      {
          return 0;
      }
      wr64(out, m.tMonoUs);
      wr32(out + 8u, m.seq);
      wr16(out + 12u, static_cast<UInt16>(count));
      wr16(out + 14u, 0);
      for(Size i = 0; i < count; ++i)
      {
          wr32(out + PATH_FIXED + i * 8u, static_cast<UInt32>(m.points[i].xMm));
          wr32(out + PATH_FIXED + i * 8u + 4u, static_cast<UInt32>(m.points[i].yMm));
      }
      return need;
  }

  Bool readPath(const Body& b, UInt8 ver, Path* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, PATH_FIXED))
      {
          return false;
      }
      const Size count = rd16(b.bytes + 12u);
      if(!lenOk(b, ver, pathBodyLen(count)))
      {
          return false;
      }
      Path m;
      m.tMonoUs = rd64(b.bytes);
      m.seq = rd32(b.bytes + 8u);
      m.points.resize(count);
      for(Size i = 0; i < count; ++i)
      {
          m.points[i].xMm = static_cast<Int32>(rd32(b.bytes + PATH_FIXED + i * 8u));
          m.points[i].yMm = static_cast<Int32>(rd32(b.bytes + PATH_FIXED + i * 8u + 4u));
      }
      *out = m;
      return true;
  }

  // ---- WAYPOINT --------------------------------------------------------------

  Size writeWaypoint(const Waypoint& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < WAYPOINT_LEN)
      {
          return 0;
      }
      wr64(out, m.tMonoUs);
      wr32(out + 8u, m.seq);
      wr16(out + 12u, m.index);
      wr16(out + 14u, m.total);
      wr32(out + 16u, static_cast<UInt32>(m.xMm));
      wr32(out + 20u, static_cast<UInt32>(m.yMm));
      wr16(out + 24u, m.flags);
      wr16(out + 26u, 0);
      wr32(out + 28u, 0);
      return WAYPOINT_LEN;
  }

  Bool readWaypoint(const Body& b, UInt8 ver, Waypoint* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, WAYPOINT_LEN) || !lenOk(b, ver, WAYPOINT_LEN))
      {
          return false;
      }
      Waypoint m;
      m.tMonoUs = rd64(b.bytes);
      m.seq = rd32(b.bytes + 8u);
      m.index = rd16(b.bytes + 12u);
      m.total = rd16(b.bytes + 14u);
      m.xMm = static_cast<Int32>(rd32(b.bytes + 16u));
      m.yMm = static_cast<Int32>(rd32(b.bytes + 20u));
      m.flags = rd16(b.bytes + 24u);
      *out = m;
      return true;
  }

  // ---- CONTROL ---------------------------------------------------------------

  Size writeControl(const Control& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < CONTROL_LEN)
      {
          return 0;
      }
      if(m.steerMilli < -MILLI_MAX || m.steerMilli > MILLI_MAX)
      {
          return 0;
      }
      if(m.throttleMilli < -MILLI_MAX || m.throttleMilli > MILLI_MAX)
      {
          return 0;
      }
      wr32(out, m.sessionId);
      wr32(out + 4u, m.seq);
      wr64(out + 8u, m.tMonoUs);
      wr16(out + 16u, static_cast<UInt16>(m.steerMilli));
      wr16(out + 18u, static_cast<UInt16>(m.throttleMilli));
      wr16(out + 20u, m.buttons);
      wr8(out + 22u, m.armEpoch);
      wr8(out + 23u, m.assumedMode);
      return CONTROL_LEN;
  }

  Bool readControl(const Body& b, UInt8 ver, Control* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, CONTROL_LEN) || !lenOk(b, ver, CONTROL_LEN))
      {
          return false;
      }
      const Int16 steer = static_cast<Int16>(rd16(b.bytes + 16u));
      const Int16 throttle = static_cast<Int16>(rd16(b.bytes + 18u));
      if(steer < -MILLI_MAX || steer > MILLI_MAX || throttle < -MILLI_MAX || throttle > MILLI_MAX)
      {
          return false;
      }
      Control m;
      m.sessionId = rd32(b.bytes);
      m.seq = rd32(b.bytes + 4u);
      m.tMonoUs = rd64(b.bytes + 8u);
      m.steerMilli = steer;
      m.throttleMilli = throttle;
      m.buttons = rd16(b.bytes + 20u);
      m.armEpoch = rd8(b.bytes + 22u);
      m.assumedMode = rd8(b.bytes + 23u);
      *out = m;
      return true;
  }

  // ---- COMMAND ---------------------------------------------------------------

  Size writeCommand(const Command& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < COMMAND_LEN)
      {
          return 0;
      }
      wr32(out, m.sessionId);
      wr32(out + 4u, m.cmdId);
      wr8(out + 8u, static_cast<UInt8>(m.verb));
      wr8(out + 9u, m.arg0);
      wr16(out + 10u, m.arg1);
      wr16(out + 12u, m.arg2);
      wr8(out + 14u, m.armEpoch);
      wr8(out + 15u, 0);
      return COMMAND_LEN;
  }

  Bool readCommand(const Body& b, UInt8 ver, Command* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, COMMAND_LEN) || !lenOk(b, ver, COMMAND_LEN))
      {
          return false;
      }
      Command m;
      m.sessionId = rd32(b.bytes);
      m.cmdId = rd32(b.bytes + 4u);
      m.verb = static_cast<Verb>(rd8(b.bytes + 8u));
      m.arg0 = rd8(b.bytes + 9u);
      m.arg1 = rd16(b.bytes + 10u);
      m.arg2 = rd16(b.bytes + 12u);
      m.armEpoch = rd8(b.bytes + 14u);
      *out = m;
      return true;
  }

  // ---- SUBSCRIBE -------------------------------------------------------------

  Size writeSubscribe(const Subscribe& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < SUBSCRIBE_LEN || m.scanDivisor == 0u)
      {
          return 0;
      }
      wr32(out, m.sessionId);
      wr32(out + 4u, m.typeMask);
      wr16(out + 8u, m.scanDivisor);
      // What section 5 reserved, now spent on the camera rate. 0 keeps its old
      // meaning exactly - "nothing asked" - so this byte pair reads the same to
      // a board that has never heard of the field.
      wr16(out + 10u, m.camFps);
      return SUBSCRIBE_LEN;
  }

  Bool readSubscribe(const Body& b, UInt8 ver, Subscribe* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, SUBSCRIBE_LEN) || !lenOk(b, ver, SUBSCRIBE_LEN))
      {
          return false;
      }
      const UInt16 divisor = rd16(b.bytes + 8u);
      // A divisor of 0 would mean "every zeroth revolution", which is a
      // division by zero wearing a subscription's clothes.
      if(divisor == 0u)
      {
          return false;
      }
      Subscribe m;
      m.sessionId = rd32(b.bytes);
      m.typeMask = rd32(b.bytes + 4u);
      m.scanDivisor = divisor;
      // NOT clamped here. The codec's job is to carry what was said, and a
      // reader that quietly rewrote the number would make the board's own
      // ceiling invisible to anything reading this frame - the log, the schema
      // dump, a capture. viewfeed clamps it where the decision belongs.
      m.camFps = rd16(b.bytes + 10u);
      *out = m;
      return true;
  }

  // ---- DESCRIBE / SCHEMA -----------------------------------------------------

  Size writeDescribe(const Describe& m, UInt8* out, Size cap)
  {
      if(out == nullptr || cap < DESCRIBE_LEN)
      {
          return 0;
      }
      wr8(out, m.type);
      wr8(out + 1u, 0);
      wr16(out + 2u, 0);
      return DESCRIBE_LEN;
  }

  Bool readDescribe(const Body& b, UInt8 ver, Describe* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, DESCRIBE_LEN) || !lenOk(b, ver, DESCRIBE_LEN))
      {
          return false;
      }
      Describe m;
      m.type = rd8(b.bytes);
      *out = m;
      return true;
  }

  Size writeSchema(const Schema& m, UInt8* out, Size cap)
  {
      if(out == nullptr || m.text.size() > 0xFFFFu)
      {
          return 0;
      }
      const Size need = SCHEMA_FIXED + padTo4(m.text.size());
      if(need > MAX_PAYLOAD || cap < need)
      {
          return 0;
      }
      wr16(out, static_cast<UInt16>(m.text.size()));
      wr16(out + 2u, 0);
      writeText(out + SCHEMA_FIXED, m.text, padTo4(m.text.size()));
      return need;
  }

  Bool readSchema(const Body& b, UInt8 ver, Schema* out)
  {
      if(out == nullptr || !bodyUsable(b, ver, SCHEMA_FIXED))
      {
          return false;
      }
      const Size textLen = rd16(b.bytes);
      if(!lenOk(b, ver, SCHEMA_FIXED + padTo4(textLen)))
      {
          return false;
      }
      Schema m;
      m.text = readText(b.bytes + SCHEMA_FIXED, textLen);
      *out = m;
      return true;
  }

  // ---- the schema text -------------------------------------------------------

  Str schemaLine(Type t)
  {
      const Desc* d = descOf(t);
      if(d == nullptr)
      {
          return Str();
      }
      Str s = hexN(static_cast<UInt8>(d->type), 2);
      s += ' ';
      s += d->name;
      s += " v";
      s += decU(d->ver);
      s += ' ';
      s += d->lenExpr;
      s += " : ";
      s += d->fields;
      return s;
  }

  Str schemaAll()
  {
      Str s;
      for(const Desc& d : CATALOG)
      {
          s += schemaLine(d.type);
          s += '\n';
      }
      return s;
  }

  // ---- describe --------------------------------------------------------------

  namespace
  {

    [[nodiscard]] Str flagText(UInt16 flags)
    {
        if(flags == 0u)
        {
            return Str();
        }
        Str inner;
        UInt16 rest = flags;
        if((flags & FLAG_ESTOP) != 0u)
        {
            inner += "estop";
            // XOR rather than AND-NOT: the bit is known set on this branch, so
            // this clears exactly it, and there is no `~` to promote a UInt16
            // to a signed int on the way.
            rest = static_cast<UInt16>(rest ^ FLAG_ESTOP);
        }
        if((flags & FLAG_DEADMAN) != 0u)
        {
            if(!inner.empty())
            {
                inner += ',';
            }
            inner += "deadman";
            rest = static_cast<UInt16>(rest ^ FLAG_DEADMAN);
        }
        if((flags & FLAG_MORE) != 0u)
        {
            if(!inner.empty())
            {
                inner += ',';
            }
            inner += "more";
            rest = static_cast<UInt16>(rest ^ FLAG_MORE);
        }
        if(rest != 0u)
        {
            if(!inner.empty())
            {
                inner += ',';
            }
            inner += hexN(rest, 4);
        }
        return Str(" [") + inner + "]";
    }

    // Sentinels are rendered as `n/a`, never as the number they are made of. A
    // 65535 mV battery printed as a reading is the same species of lie as a
    // stale picture drawn as live.
    [[nodiscard]] Str battText(UInt16 v)
    {
        return v == BATT_ABSENT ? Str("n/a") : decU(v) + "mV";
    }

    [[nodiscard]] Str cpuText(Int16 v)
    {
        return v == CPU_ABSENT ? Str("n/a") : centi(v);
    }

    [[nodiscard]] Str silentText(UInt32 v)
    {
        return v == PICO_SILENT_ABSENT ? Str("n/a") : decU(v);
    }

    [[nodiscard]] Str escText(UInt16 v)
    {
        return v == ESC_ABSENT ? Str("n/a") : decU(v);
    }

    [[nodiscard]] Str ageText(UInt32 v)
    {
        return v == CONTROL_AGE_NEVER ? Str("never") : decU(v);
    }

    [[nodiscard]] Str serialHex(const Array<UInt8, 16>& serial)
    {
        Str s;
        for(const UInt8 octet : serial)
        {
            s += hexUpper(static_cast<UInt32>(octet >> 4));
            s += hexUpper(static_cast<UInt32>(octet & 0x0Fu));
        }
        return s;
    }

    [[nodiscard]] Str describeBody(const Frame& f)
    {
        const UInt8 ver = f.head.ver;
        switch(f.head.type)
        {
            case Type::TYPE_HELLO:
            {
                Hello m;
                if(!readHello(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "proto=" + decU(m.protoMajor) + "." + decU(m.protoMinor);
                s += " features=" + hexN(m.featureMask, 8);
                s += " build=" + hexN(m.viewerBuild, 8);
                s += " udp=" + decU(m.viewerUdpPort);
                s += " want=" + decU(m.wantControl);
                s += " hz=" + decU(m.controlHz);
                s += " name=" + quoteText(m.name);
                return s;
            }
            case Type::TYPE_WELCOME:
            {
                Welcome m;
                if(!readWelcome(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "proto=" + decU(m.protoMajor) + "." + decU(m.protoMinor);
                s += " session=" + hexN(m.sessionId, 8);
                s += " boot=" + hexN(m.bootId, 8);
                s += " features=" + hexN(m.featureMask, 8);
                s += " udp=" + decU(m.controlUdpPort);
                s += " period=" + decU(m.controlPeriodMs);
                s += " stale=" + decU(m.staleMs);
                s += " dead=" + decU(m.deadMs);
                s += " accepted=" + decU(m.accepted);
                s += " refusal=" + decU(m.refusal);
                s += " mono=" + decU(m.boardMonoUs);
                s += " epoch=" + decU(m.armEpoch);
                s += " caps=" + hexN(m.capabilities, 2);
                s += " board=" + quoteText(m.boardName);
                s += " text=" + quoteText(m.text);
                return s;
            }
            case Type::TYPE_BYE:
            {
                Bye m;
                if(!readBye(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "reason=";
                s += reasonName(m.reason);
                s += "(" + decU(static_cast<UInt16>(m.reason)) + ")";
                s += " text=" + quoteText(m.text);
                return s;
            }
            case Type::TYPE_PING:
            case Type::TYPE_PONG:
            {
                Ping m;
                if(!readPing(f.body, ver, &m))
                {
                    return Str();
                }
                return "token=" + hexN(m.token, 16) + " mono=" + decU(m.senderMonoUs);
            }
            case Type::TYPE_LEAVE:
            {
                Leave m;
                if(!readLeave(f.body, ver, &m))
                {
                    return Str();
                }
                return "session=" + hexN(m.sessionId, 8);
            }
            case Type::TYPE_SCAN:
            {
                Scan m;
                if(!readScan(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "mono=" + decU(m.tMonoUs);
                s += " rev=" + decU(m.revIndex);
                s += " hz=" + decU(m.freqMilliHz);
                s += " n=" + decU(m.points.size());
                s += " health=";
                s += healthName(m.health);
                s += " motor=" + decU(m.motor);
                s += " dropped=" + decU(m.droppedSinceLast);
                s += " div=" + decU(m.scanDivisor);
                return s;
            }
            case Type::TYPE_DECIDE:
            {
                Decide m;
                if(!readDecide(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "rev=" + decU(m.revIndex);
                s += " clear=" + decU(m.clearanceMm);
                s += " hits=" + decU(m.hits);
                s += " steer=" + decI(m.steerMilli);
                s += " throttle=" + decI(m.throttleMilli);
                s += " mode=";
                s += driveModeName(m.mode);
                s += " stop=" + decU(m.stop);
                s += " source=";
                s += pilotModeName(m.source);
                s += " modeMs=" + decU(m.modeMs);
                return s;
            }
            case Type::TYPE_BOARD:
            {
                BoardState m;
                if(!readBoard(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "mono=" + decU(m.tMonoUs);
                s += " up=" + decU(m.upS);
                s += " cpu=" + cpuText(m.cpuCentiC);
                s += " batt=" + battText(m.battMilliV);
                s += " pico=";
                s += picoLinkName(m.picoLink);
                s += " armed=" + decU(m.picoArmed);
                s += " mode=";
                s += pilotModeName(m.pilotMode);
                s += " deadman=";
                s += deadmanName(m.deadman);
                s += " health=";
                s += healthName(m.lidarHealth);
                s += " spin=" + decU(m.lidarSpinning);
                s += " epoch=" + decU(m.armEpoch);
                s += " holder=";
                s += holderName(m.controlHolder);
                s += " loopWorst=" + decU(m.loopWorstUs);
                s += " loopLate=" + decU(m.loopLateCount);
                s += " picoSilent=" + silentText(m.picoSilentMs);
                s += " revs=" + decU(m.revolutions);
                s += " timeouts=" + decU(m.timeouts);
                s += " txDropped=" + decU(m.txDroppedFrames);
                s += " rxCtl=" + decU(m.rxControl);
                s += " rxStale=" + decU(m.rxControlStale);
                s += " ip=" + hexN(m.ipv4, 8);
                s += " encAvg=" + decU(m.encodeAvgNs);
                s += " encMax=" + decU(m.encodeMaxNs);
                s += " clients=" + decU(m.clients);
                s += " wifi=" + quoteText(m.wifiName);
                return s;
            }
            case Type::TYPE_LIDAR_INFO:
            {
                LidarInfo m;
                if(!readLidarInfo(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "model=" + decU(m.model);
                s += " fw=" + decU(m.fwMajor) + "." + decU(m.fwMinor);
                s += " hw=" + decU(m.hwRev);
                s += " serial=" + serialHex(m.serial);
                s += " baud=" + decU(m.baud);
                return s;
            }
            case Type::TYPE_EVENT:
            {
                Event m;
                if(!readEvent(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "mono=" + decU(m.tMonoUs);
                s += " sev=";
                s += severityName(m.severity);
                s += " code=" + decU(m.code);
                s += " dropped=" + decU(m.droppedSince);
                s += " text=" + quoteText(m.text);
                return s;
            }
            case Type::TYPE_CTLSTATE:
            {
                CtlState m;
                if(!readCtlState(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "mono=" + decU(m.tMonoUs);
                s += " ack=" + decU(m.ackSeq);
                s += " age=" + ageText(m.controlAgeMs);
                s += " steerNow=" + decI(m.steerNowMilli);
                s += " throttle=" + decI(m.throttleMilli);
                s += " esc=" + escText(m.escUs);
                s += " neutralIn=" + decU(m.neutralInMs);
                s += " disarmIn=" + decU(m.disarmInMs);
                s += " armed=" + decU(m.armed);
                s += " epoch=" + decU(m.armEpoch);
                s += " deadman=";
                s += deadmanName(m.deadman);
                s += " refuse=";
                s += refuseName(m.refuse);
                s += " holder=";
                s += holderName(m.holder);
                s += " mode=";
                s += pilotModeName(m.pilotMode);
                s += " scanAge=" + decU(m.scanAgeMs);
                s += " picoSilent=" + silentText(m.picoSilentMs);
                s += " lastCmd=" + decU(m.lastCmdId);
                return s;
            }
            case Type::TYPE_CMDACK:
            {
                CmdAck m;
                if(!readCmdAck(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "cmd=" + decU(m.cmdId);
                s += " verb=";
                s += verbName(m.verb);
                s += " result=" + decU(m.result);
                s += " epoch=" + decU(m.armEpoch);
                s += " text=" + quoteText(m.text);
                return s;
            }
            case Type::TYPE_CAMERA:
            {
                Camera m;
                if(!readCamera(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "mono=" + decU(m.tMonoUs);
                s += " frame=" + decU(m.frameIndex);
                s += " size=" + decU(m.width) + "x" + decU(m.height);
                s += " codec=" + decU(m.codec);
                s += " flags=" + decU(m.flags);
                s += " bytes=" + decU(m.data.size());
                return s;
            }
            case Type::TYPE_POSE:
            {
                Pose m;
                if(!readPose(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "mono=" + decU(m.tMonoUs);
                s += " x=" + decI(m.xMm);
                s += " y=" + decI(m.yMm);
                s += " heading=" + decI(m.headingMilliRad);
                s += " sigmaXy=" + decU(m.sigmaXyMm);
                s += " sigmaHeading=" + decU(m.sigmaHeadingMilliRad);
                s += " source=" + decU(m.source);
                s += " valid=" + decU(m.valid);
                return s;
            }
            case Type::TYPE_PATH:
            {
                Path m;
                if(!readPath(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "mono=" + decU(m.tMonoUs);
                s += " seq=" + decU(m.seq);
                s += " n=" + decU(m.points.size());
                return s;
            }
            case Type::TYPE_WAYPOINT:
            {
                Waypoint m;
                if(!readWaypoint(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "mono=" + decU(m.tMonoUs);
                s += " seq=" + decU(m.seq);
                s += " index=" + decU(m.index);
                s += " total=" + decU(m.total);
                s += " x=" + decI(m.xMm);
                s += " y=" + decI(m.yMm);
                s += " flags=" + hexN(m.flags, 4);
                return s;
            }
            case Type::TYPE_CONTROL:
            {
                Control m;
                if(!readControl(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "session=" + hexN(m.sessionId, 8);
                s += " seq=" + decU(m.seq);
                s += " mono=" + decU(m.tMonoUs);
                s += " steer=" + decI(m.steerMilli);
                s += " throttle=" + decI(m.throttleMilli);
                s += " buttons=" + hexN(m.buttons, 4);
                s += " epoch=" + decU(m.armEpoch);
                s += " mode=";
                s += pilotModeName(m.assumedMode);
                return s;
            }
            case Type::TYPE_COMMAND:
            {
                Command m;
                if(!readCommand(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "session=" + hexN(m.sessionId, 8);
                s += " cmd=" + decU(m.cmdId);
                s += " verb=";
                s += verbName(m.verb);
                s += " arg0=" + decU(m.arg0);
                s += " arg1=" + decU(m.arg1);
                s += " arg2=" + decU(m.arg2);
                s += " epoch=" + decU(m.armEpoch);
                return s;
            }
            case Type::TYPE_SUBSCRIBE:
            {
                Subscribe m;
                if(!readSubscribe(f.body, ver, &m))
                {
                    return Str();
                }
                Str s = "session=" + hexN(m.sessionId, 8);
                s += " mask=" + hexN(m.typeMask, 8);
                s += " div=" + decU(m.scanDivisor);
                // ONLY WHEN ASKED. A viewer that did not request a rate has
                // nothing to say about one, and printing "fps=0" would put a
                // number on a decision nobody made.
                if(m.camFps != 0u)
                {
                    s += " fps=" + decU(m.camFps);
                }
                return s;
            }
            case Type::TYPE_DESCRIBE:
            {
                Describe m;
                if(!readDescribe(f.body, ver, &m))
                {
                    return Str();
                }
                return "type=" + hexN(m.type, 2);
            }
            case Type::TYPE_SCHEMA:
            {
                Schema m;
                if(!readSchema(f.body, ver, &m))
                {
                    return Str();
                }
                return "text=" + quoteText(m.text);
            }
            default:
                return Str();
        }
    }

  }

  Str describe(const Frame& f)
  {
      const UInt8 tag = static_cast<UInt8>(f.head.type);
      const Desc* d = descOf(f.head.type);
      Str s;
      if(d != nullptr)
      {
          s += d->name;
      }
      else
      {
          s += "UNKNOWN(" + hexN(tag, 2) + ")";
      }
      s += " v" + decU(f.head.ver);
      s += " seq=" + decU(f.head.seq);
      s += " len=" + decU(f.body.len);
      s += flagText(f.head.flags);
      s += " : ";
      if(d == nullptr)
      {
          // Skipped by exactly `len` and counted, never fatal - the whole
          // extensibility story in one line, and the renderer says so rather
          // than pretending it understood.
          s += "skipped, " + decU(f.body.len) + " bytes";
          return s;
      }
      const Str body = describeBody(f);
      if(body.empty())
      {
          s += "bad body";
          return s;
      }
      s += body;
      return s;
  }

  // ---- the version rule ------------------------------------------------------

  Bool versionOk(UInt16 protoMajor)
  {
      return protoMajor == PROTO_MAJOR;
  }

  Bye versionRefusal(const Hello& h, CharSeq boardBuild)
  {
      Bye m;
      m.reason = Reason::REASON_VERSION;
      // Not decoration. A refusal that says only "incompatible" sends a person
      // to read source in a field; the two version numbers and the build stamp
      // are what turn it into an action they can take standing up.
      Str text = "board speaks bibowire ";
      text += decU(PROTO_MAJOR);
      text += ".";
      text += decU(PROTO_MINOR);
      text += ", viewer sent ";
      text += decU(h.protoMajor);
      text += ".";
      text += decU(h.protoMinor);
      text += " - rebuild the viewer from the same commit as the board (board build ";
      text += boardBuild != nullptr ? Str(boardBuild) : Str("unknown");
      text += ")";
      m.text = text;
      return m;
  }

  CharSeq refuseName(Refuse r)
  {
      switch(r)
      {
          case Refuse::REFUSE_NONE:
              return "none";
          case Refuse::REFUSE_NOT_ARMED:
              return "not_armed";
          case Refuse::REFUSE_DEADMAN_SOFT:
              return "deadman_soft";
          case Refuse::REFUSE_DEADMAN_DEAD:
              return "deadman_dead";
          case Refuse::REFUSE_ESTOP:
              return "estop";
          case Refuse::REFUSE_EPOCH:
              return "epoch";
          case Refuse::REFUSE_MODE:
              return "mode";
          case Refuse::REFUSE_NOT_HOLDER:
              return "not_holder";
          case Refuse::REFUSE_PICO_DOWN:
              return "pico_down";
          case Refuse::REFUSE_NO_UDP:
              return "no_udp";
          default:
              return "?";
      }
  }

  // ---- applying a CONTROL ----------------------------------------------------

  namespace control
  {

    Bool newer(UInt32 a, UInt32 b)
    {
        return static_cast<Int32>(a - b) > 0;
    }

    Outcome apply(const Gate& g, const Control& c)
    {
        Outcome o;
        o.highestSeq = g.highestSeq;

        // After a hotspot blip the viewer reconnects and datagrams from the
        // previous session may still be in flight in a buffer somewhere.
        // Without this test they would drive the car with a second-old stick
        // position, and the socket would look perfect while they did it.
        if(g.sessionId == 0u || c.sessionId != g.sessionId)
        {
            o.verdict = Verdict::VERDICT_BAD_SESSION;
            return o;
        }

        // An observer's datagrams and a second viewer's datagrams are counted
        // and DISCARDED, never blended - and above all they do not feed the
        // timer. Without that, viewer B's stream keeps the deadman alive while
        // viewer A, whose laptop just slept, is protected by nothing.
        if(!g.haveHolder || !g.fromHolder)
        {
            o.verdict = Verdict::VERDICT_NOT_HOLDER;
            o.refuse = Refuse::REFUSE_NOT_HOLDER;
            return o;
        }

        // Newest wins. Retransmitting a stale command is worse than dropping
        // it, which is the conclusion docs/conventions.md already reached for
        // the Pico link.
        if(!newer(c.seq, g.highestSeq))
        {
            o.verdict = Verdict::VERDICT_STALE_SEQ;
            return o;
        }

        o.verdict = Verdict::VERDICT_APPLIED;
        o.feedsDeadman = true;
        o.highestSeq = c.seq;
        o.steerMilli = c.steerMilli;

        // The epoch is the one byte that closes the case seq cannot: a
        // four-second stall, the board disarms at 300 ms, the link returns, and
        // the viewer is still sending throttle 400 twenty times a second
        // because the operator's finger never left the key. Those datagrams
        // carry HIGHER seqs and look perfectly valid.
        //
        // Steering is still applied in every one of these cases. A refusal is
        // about who may add energy, not about where the wheels point.
        if(c.armEpoch != g.armEpoch)
        {
            o.throttleMilli = 0;
            o.refuse = Refuse::REFUSE_EPOCH;
            return o;
        }
        if(c.assumedMode != g.pilotMode)
        {
            o.throttleMilli = 0;
            o.refuse = Refuse::REFUSE_MODE;
            return o;
        }
        o.throttleMilli = c.throttleMilli;
        o.refuse = Refuse::REFUSE_NONE;
        return o;
    }

  }

  // ---- the deadman -----------------------------------------------------------

  namespace deadman
  {

    namespace
    {

      [[nodiscard]] Int32 countdown(Int64 age, Int32 budget)
      {
          const Int64 left = static_cast<Int64>(budget) - age;
          if(left <= 0)
          {
              return 0;
          }
          return static_cast<Int32>(left);
      }

    }

    Output step(const Inputs& in)
    {
        Output o;

        // The latch outranks everything, including a link that is perfectly
        // healthy. It clears only through COMMAND CLEAR_ESTOP while disarmed
        // and then a deliberate ARM: three steps, because a stop that can be
        // undone by releasing a key is a stop that will be undone by accident.
        if(in.estopLatched)
        {
            o.state = State::STATE_ESTOP;
            o.refuse = Refuse::REFUSE_ESTOP;
            o.neutralInMs = 0;
            o.disarmInMs = 0;
            return o;
        }

        // With NO viewer holding the slot this timer does not apply at all -
        // the pilot runs under its own blind and silence rules exactly as it
        // does today with nobody watching. STATE_LIVE here means "this timer is
        // not the thing stopping the car", not "somebody is driving".
        if(!in.haveHolder)
        {
            o.state = State::STATE_LIVE;
            o.refuse = Refuse::REFUSE_NONE;
            o.neutralInMs = CONTROL_STALE_MS;
            o.disarmInMs = CONTROL_DEAD_MS;
            return o;
        }

        // A negative age is a bug somewhere upstream, and the safe reading of a
        // bug is "stopped". Treating it as freshness is how a sign error
        // becomes a car that will not stop.
        const Int64 age = in.nowMs - in.lastControlMs;
        if(age < 0 || age >= CONTROL_DEAD_MS)
        {
            o.state = State::STATE_DEAD;
            o.refuse = Refuse::REFUSE_DEADMAN_DEAD;
            o.neutralInMs = 0;
            o.disarmInMs = 0;
            return o;
        }

        o.neutralInMs = countdown(age, CONTROL_STALE_MS);
        o.disarmInMs = countdown(age, CONTROL_DEAD_MS);

        if(age >= CONTROL_STALE_MS)
        {
            o.state = State::STATE_SOFT;
            o.refuse = Refuse::REFUSE_DEADMAN_SOFT;
            return o;
        }

        // The link is fresh, so anything refusing throttle now is about consent
        // rather than about silence. Reported most fundamental first: whether
        // the operator is asking at all, then the arm generation they claim,
        // then the mode they believe is running.
        o.state = State::STATE_SOFT;
        if(!in.enable)
        {
            o.refuse = Refuse::REFUSE_NOT_ARMED;
            return o;
        }
        if(!in.epochMatches)
        {
            o.refuse = Refuse::REFUSE_EPOCH;
            return o;
        }
        if(!in.modeAgrees)
        {
            o.refuse = Refuse::REFUSE_MODE;
            return o;
        }
        o.state = State::STATE_LIVE;
        o.refuse = Refuse::REFUSE_NONE;
        return o;
    }

    CharSeq stateName(State s)
    {
        switch(s)
        {
            case State::STATE_LIVE:
                return "live";
            case State::STATE_SOFT:
                return "soft";
            case State::STATE_DEAD:
                return "dead";
            case State::STATE_ESTOP:
                return "estop";
            default:
                return "?";
        }
    }

  }

}
