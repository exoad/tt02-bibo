#include "carrules.hxx"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

#include "proto.hxx"

namespace
{
  constexpr Float32 DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
  constexpr Int32 NEUTRAL_US = bibowire::ESC_NEUTRAL_US;

  // A --seconds past a day is a typo, and this keeps seconds * 1000 far from overflow.
  constexpr Int64 SECONDS_MAX = 86400;

  static_assert(bibo::SCAN_MIN_HITS > 0, "the SCAN_MIN_HITS-th nearest needs at least one hit");

  // The SCAN_MIN_HITS-th smallest distance it is shown, capped at SCAN_FAR_M, or
  // SCAN_FAR_M when it was shown fewer.
  struct Nth
  {
      Array<Float32, static_cast<Size>(bibo::SCAN_MIN_HITS)> v = {};
      Size n = 0;

      Void add(Float32 x)
      {
          if(!(x >= 0.0f) || (n == v.size() && x >= v[n - 1]))
          {
              return;
          }
          Size i = (n < v.size()) ? n++ : n - 1;
          while(i > 0 && v[i - 1] > x)
          {
              v[i] = v[i - 1];
              --i;
          }
          v[i] = x;
      }

      [[nodiscard]] Float32 result() const
      {
          return n < v.size() ? bibo::SCAN_FAR_M : std::min(v[n - 1], bibo::SCAN_FAR_M);
      }
  };

  // Both angles count clockwise seen from above, so the difference is positive
  // to the right. NaN when either is not finite.
  [[nodiscard]] Float32 bearingOf(Float32 rawDeg, Float32 forwardDeg)
  {
      Float32 b = std::fmod(rawDeg - forwardDeg, 360.0f);
      if(b > 180.0f)
      {
          b -= 360.0f;
      }
      else if(b < -180.0f)
      {
          b += 360.0f;
      }
      return b;
  }

  // The whole of text as a number; a tail such as "12x" is refused.
  [[nodiscard]] Bool wholeInt(const Str& text, Int64& out)
  {
      Char* stop = nullptr;
      out = std::strtoll(text.c_str(), &stop, 10);
      return !text.empty() && stop != nullptr && *stop == '\0';
  }

  [[nodiscard]] Bool wholeFloat(const Str& text, Float32& out)
  {
      Char* stop = nullptr;
      const Float64 v = std::strtod(text.c_str(), &stop);
      out = static_cast<Float32>(v);
      return !text.empty() && stop != nullptr && *stop == '\0' && std::isfinite(v);
  }

  // key's value in an OK drive line's fields, -1 when absent or not a number.
  [[nodiscard]] Int32 keyOr(const Str& fields, const Char* key)
  {
      Int32 v = -1;
      return proto::fieldInt(fields, key, v) ? v : -1;
  }

  [[nodiscard]] Str neutralLine()
  {
      return proto::command("ESC", "NEUTRAL");
  }

  // Armed, steering on and a forward band, in a report that came after OK stop:
  // a report from before our opening STOP can never confirm an arm.
  [[nodiscard]] Bool confirms(const carrules::Board& b)
  {
      return b.stopAnswered && b.armed == 1 && b.servoOn == 1
             && b.escMaxUs > std::max(b.escMinUs, NEUTRAL_US);
  }
}

namespace bibo
{
  Bool Scan::blind() const
  {
      return revolution == 0 || points.size() < static_cast<Size>(SCAN_MIN_POINTS);
  }

  Float32 Scan::ahead(Float32 halfWidthM) const
  {
      if(blind() || !(halfWidthM >= 0.0f))
      {
          return 0.0f;
      }
      Nth nth;
      for(const Point& p : points)
      {
          if(!(std::abs(p.bearingDeg) <= SCAN_FRONT_ARC_DEG))
          {
              continue;
          }
          const Float32 rad = p.bearingDeg * DEG_TO_RAD;
          if(std::abs(p.distanceM * std::sin(rad)) <= halfWidthM)
          {
              nth.add(p.distanceM * std::cos(rad));
          }
      }
      return nth.result();
  }

  Float32 Scan::nearest(Float32 fromDeg, Float32 toDeg) const
  {
      if(blind() || !(fromDeg <= toDeg))
      {
          return 0.0f;
      }
      Nth nth;
      for(const Point& p : points)
      {
          if(p.bearingDeg >= fromDeg && p.bearingDeg <= toDeg)
          {
              nth.add(p.distanceM);
          }
      }
      return nth.result();
  }
}

namespace carrules
{
  Str usage()
  {
      return Str("flags:\n")
             + "  (none)          dry run: the Pico is never opened, the car cannot move\n"
             + "  --drive         really drive\n"
             + "  --seconds N     stop the run N seconds after arm()\n"
             + "  --forward DEG   the raw lidar angle that points straight ahead, for this run\n"
             + "  --lidar PORT    default " + bibo::LIDAR_PORT + "\n"
             + "  --pico PORT     default " + bibo::PICO_PORT + "\n"
             + "  --no-viewer     do not serve the Windows viewer\n"
             + "  --help          print this list\n";
  }

  Bool parseArgs(Int32 argc, Char** argv, Bool measured, Options& out, Str& why)
  {
      out = Options();
      why.clear();
      Bool forwardGiven = false;
      for(Int32 i = 1; i < argc; ++i)
      {
          const Str flag = argv[i] != nullptr ? argv[i] : "";
          if(flag == "--drive")
          {
              out.drive = true;
              continue;
          }
          if(flag == "--no-viewer")
          {
              out.viewer = false;
              continue;
          }
          if(flag == "--help")
          {
              out.help = true;
              continue;
          }
          const Bool takesValue = flag == "--seconds" || flag == "--forward"
                                  || flag == "--lidar" || flag == "--pico";
          if(!takesValue)
          {
              why = "unknown flag '" + flag + "' - --help lists the flags";
              return false;
          }
          if(i + 1 >= argc || argv[i + 1] == nullptr)
          {
              why = flag + " needs a value";
              return false;
          }
          const Str value = argv[++i];
          if(flag == "--seconds")
          {
              Int64 n = 0;
              if(!wholeInt(value, n) || n < 1 || n > SECONDS_MAX)
              {
                  why = "--seconds wants whole seconds, 1 to " + std::to_string(SECONDS_MAX)
                        + ", not '" + value + "'";
                  return false;
              }
              out.seconds = static_cast<Int32>(n);
          }
          else if(flag == "--forward")
          {
              Float32 deg = 0.0f;
              if(!wholeFloat(value, deg) || deg < -360.0f || deg > 360.0f)
              {
                  why = "--forward wants degrees, -360 to 360, not '" + value + "'";
                  return false;
              }
              out.forwardDeg = deg;
              forwardGiven = true;
          }
          else if(value.empty() || value[0] == '-')
          {
              why = flag + " wants a port, not '" + value + "'";
              return false;
          }
          else if(flag == "--lidar")
          {
              out.lidarPort = value;
          }
          else
          {
              out.picoPort = value;
          }
      }
      if(out.drive && !out.help && !measured && !forwardGiven)
      {
          why = "--drive needs the lidar's forward angle: measure it with a dry run and set"
                " LIDAR_FORWARD_DEG and LIDAR_FORWARD_MEASURED in car.hxx, or pass --forward DEG";
          return false;
      }
      return true;
  }

  bibo::Scan toScan(const Vec<reactive::Ray>& rays, Float32 forwardDeg, UInt32 rev)
  {
      bibo::Scan s;
      s.revolution = rev;
      s.points.reserve(rays.size());
      for(const reactive::Ray& r : rays)
      {
          const Float32 b = bearingOf(r.angleDeg, forwardDeg);
          if(!(r.distMm > 0.0f && std::isfinite(r.distMm) && std::isfinite(b)))
          {
              continue;
          }
          s.points.push_back(bibo::Point{b, r.distMm / 1000.0f});
      }
      return s;
  }

  Bool fold(Board& b, const Str& line)
  {
      const proto::Reply r = proto::read(line);
      if(r.kind == proto::Kind::KIND_ERR)
      {
          ++b.errors;
          return false;
      }
      if(r.kind != proto::Kind::KIND_OK)
      {
          return false;
      }
      if(r.topic == "stop")
      {
          b.stopAnswered = true;
          return false;
      }
      if(r.topic != "drive")
      {
          return false;
      }
      // The '=' is part of each key, and proto::field matches whole keys, so
      // esc= is never read out of esc_min=.
      b.armed = keyOr(r.rest, "armed=");
      b.servoOn = keyOr(r.rest, "servo_on=");
      b.escUs = keyOr(r.rest, "esc=");
      b.escMinUs = keyOr(r.rest, "esc_min=");
      b.escMaxUs = keyOr(r.rest, "esc_max=");
      b.escRevUs = keyOr(r.rest, "esc_rev=");
      b.stale = keyOr(r.rest, "stale=");
      return true;
  }

  Int32 forwardPulse(Float32 t, Int32 escMinUs, Int32 escMaxUs)
  {
      const Int32 lo = std::max(escMinUs, NEUTRAL_US);
      if(!(t > 0.0f) || escMinUs <= 0 || escMaxUs <= lo)
      {
          return NEUTRAL_US;
      }
      const Float32 span = static_cast<Float32>(escMaxUs - lo);
      return lo + static_cast<Int32>(std::min(t, 1.0f) * span + 0.5f);
  }

  Str steerLine(Float32 steer)
  {
      return proto::steer(std::isnan(steer) ? 0.0f : std::clamp(steer, -1.0f, 1.0f));
  }

  Int32 pulseIn(const Str& line)
  {
      const StrView head = "ESC ";
      if(!line.starts_with(head) || line.size() == head.size())
      {
          return -1;
      }
      Char* end = nullptr;
      const Int64 us = std::strtol(line.c_str() + head.size(), &end, 10);
      return end != nullptr && *end == '\0' ? static_cast<Int32>(us) : -1;
  }

  CharSeq endName(End e)
  {
      switch(e)
      {
      case End::END_NONE:
          return "still running";
      case End::END_FINISHED:
          return "the program finished";
      case End::END_SIGNAL:
          return "a signal (Ctrl-C, SIGTERM or SIGHUP)";
      case End::END_ESTOP:
          return "ESTOP from a viewer";
      case End::END_SECONDS:
          return "--seconds ran out";
      case End::END_HELP:
          return "--help";
      case End::END_BAD_FLAGS:
          return "bad flags";
      case End::END_OPEN_FAILED:
          return "a device would not open";
      case End::END_NO_REVOLUTION:
          return "the lidar delivered no good revolution";
      case End::END_ARM_TIMEOUT:
          return "the Pico did not confirm the arm";
      case End::END_LIDAR_LOST:
          return "the lidar stopped delivering good revolutions";
      case End::END_LINK_LOST:
          return "the Pico's port closed";
      case End::END_PICO_DISARMED:
          return "the Pico reported it was disarmed";
      case End::END_PICO_WATCHDOG:
          return "the Pico reported its watchdog fired";
      case End::END_SEND_GAP:
          return "too long between lines sent to the Pico";
      case End::END_VIEWER_STUCK:
          return "the viewer's feed was stuck, so no ESTOP could arrive";
      }
      return "unknown";
  }

  Int32 exitCode(End e, UInt64 goodRevolutions)
  {
      switch(e)
      {
      case End::END_HELP:
          return 0;
      case End::END_BAD_FLAGS:
          return 2;
      case End::END_FINISHED:
      case End::END_SIGNAL:
      case End::END_ESTOP:
      case End::END_SECONDS:
          return goodRevolutions > 0 ? 0 : 1;
      default:
          return 1;
      }
  }

  Governor::Governor(Bool dryRun, Int32 seconds)
  {
      dry = dryRun;
      secondsMs = static_cast<Int64>(seconds) * 1000;
  }

  Vec<Str> Governor::open(const Vec<Str>& trim)
  {
      if(dry)
      {
          return {};
      }
      Vec<Str> out = {proto::stop(), proto::command("PING")};
      out.insert(out.end(), trim.begin(), trim.end());
      return out;
  }

  Void Governor::requestArm()
  {
      if(arm == Arm::ARM_IDLE)
      {
          arm = Arm::ARM_REQUESTED;
      }
  }

  Void Governor::end(End why)
  {
      if(reason == End::END_NONE)
      {
          reason = why;
      }
  }

  Vec<Str> Governor::pass(const Inputs& in)
  {
      const Int64 now = in.nowMs;
      // First, so that the Pico disarming in answer to a signal handler's own
      // STOP reads as the signal.
      if(in.signal)
      {
          end(End::END_SIGNAL);
      }
      if(in.estop)
      {
          end(End::END_ESTOP);
      }
      if(in.viewerStuck)
      {
          end(End::END_VIEWER_STUCK);
      }
      if(!in.replies.empty())
      {
          heardMs = now;
      }
      for(const Str& line : in.replies)
      {
          if(!fold(pico, line))
          {
              continue;
          }
          if(arm == Arm::ARM_CONFIRMED)
          {
              if(pico.armed == 0)
              {
                  end(End::END_PICO_DISARMED);
              }
              if(pico.stale == 1)
              {
                  end(End::END_PICO_WATCHDOG);
              }
          }
          else if(arm == Arm::ARM_PENDING && reason == End::END_NONE && confirms(pico))
          {
              arm = Arm::ARM_CONFIRMED;
              armedMs = now;
          }
      }
      const Bool arming = arm == Arm::ARM_PENDING || arm == Arm::ARM_CONFIRMED;
      if(!dry && in.linkLost)
      {
          end(End::END_LINK_LOST);
      }
      if(arm == Arm::ARM_CONFIRMED && secondsMs > 0 && now - armedMs > secondsMs)
      {
          end(End::END_SECONDS);
      }
      if(arming && now - std::max(in.scanMs, armSentMs) > bibo::LIDAR_LOST_MS)
      {
          end(End::END_LIDAR_LOST);
      }
      if(!dry && arm == Arm::ARM_PENDING && now - armSentMs > bibo::ARM_CONFIRM_MS)
      {
          end(End::END_ARM_TIMEOUT);
      }
      const Bool gap = in.sentMs < 0 || now - in.sentMs > bibo::SEND_GAP_STOP_MS
                       || in.gapMs > bibo::SEND_GAP_STOP_MS;
      if(!dry && arming && gap)
      {
          end(End::END_SEND_GAP);
      }
      if(reason != End::END_NONE)
      {
          return dry ? Vec<Str>() : Vec<Str>{proto::stop()};
      }
      if(arm == Arm::ARM_REQUESTED)
      {
          armSentMs = now;
          if(dry)
          {
              arm = Arm::ARM_CONFIRMED;
              armedMs = now;
              return {};
          }
          arm = Arm::ARM_PENDING;
          return {proto::command("ESC", "ARM"), proto::command("SERVO", "ON")};
      }
      if(dry)
      {
          return {};
      }
      if(arm != Arm::ARM_CONFIRMED)
      {
          return {neutralLine()};
      }
      // The steering is sent every pass, so it holds while the throttle is neutral.
      const Bool fresh = in.driveMs >= 0 && now - in.driveMs <= bibo::DRIVE_FRESH_MS
                         && in.scanMs >= 0 && now - in.scanMs <= bibo::SCAN_FRESH_MS
                         && heardMs >= 0 && now - heardMs <= bibo::PICO_QUIET_MS
                         && pico.armed == 1;
      const Int32 us = forwardPulse(in.throttle, pico.escMinUs, pico.escMaxUs);
      return {steerLine(in.steer), fresh && us > NEUTRAL_US ? proto::escUs(us) : neutralLine()};
  }

  Arm Governor::armState() const
  {
      return arm;
  }

  End Governor::ended() const
  {
      return reason;
  }

  const Board& Governor::board() const
  {
      return pico;
  }

  Int64 pulseWaitMs(Int64 nowMs, Int64 sentMs)
  {
      return sentMs < 0 ? 0 : sentMs + bibo::SEND_GAP_STOP_MS - nowMs;
  }

  Sender::Sender(ClockFn clockMs, SendFn send) : clock(std::move(clockMs)), write(std::move(send))
  {
  }

  carlink::Result Sender::send(const Str& line, Int32 waitMs)
  {
      const carlink::Result r = write(line, waitMs);
      if(r != carlink::Result::RESULT_OK)
      {
          return r;
      }
      const Int64 now = clock();
      const Int64 gap = lastMs >= 0 ? now - lastMs : 0;
      lastMs = now;
      worstMs = std::max(worstMs, gap);
      recentMs = std::max(recentMs, gap);
      passMs = std::max(passMs, gap);
      return r;
  }

  Vec<Str> Sender::pass(const Vec<Str>& lines, End& ended)
  {
      passMs = 0;
      Vec<Str> sent;
      for(const Str& line : lines)
      {
          if(pulseIn(line) <= NEUTRAL_US)
          {
              static_cast<Void>(send(line));
              sent.push_back(line);
              continue;
          }
          // Judged now, after whatever this pass has already waited on.
          const Int64 leftMs = pulseWaitMs(clock(), lastMs);
          const Int64 waitMs = std::min<Int64>(leftMs, carlink::WRITE_WAIT_MS);
          const Bool inTime = passMs <= bibo::SEND_GAP_STOP_MS && waitMs > 0;
          const carlink::Result r = inTime
              ? send(line, static_cast<Int32>(waitMs))
              : carlink::Result::RESULT_WRITE_FAILED;
          if(r == carlink::Result::RESULT_OK)
          {
              sent.push_back(line);
              continue;
          }
          const Bool gone = r == carlink::Result::RESULT_CLOSED
                            || r == carlink::Result::RESULT_NOT_OPEN;
          ended = gone ? End::END_LINK_LOST : End::END_SEND_GAP;
          sent.push_back(proto::stop());
          static_cast<Void>(send(sent.back()));
          break;
      }
      return sent;
  }

  Int64 Sender::sentMs() const
  {
      return lastMs;
  }

  Int64 Sender::worstGapMs() const
  {
      return worstMs;
  }

  Int64 Sender::takeGapMs()
  {
      const Int64 gap = recentMs;
      recentMs = 0;
      return gap;
  }
}
