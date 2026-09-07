// See lidar.hxx. Two implementations in one file: the real one, behind the SDK
// and Linux, and the refusing one everywhere else. One file rather than two so
// the two cannot drift apart in what they promise - the same header, the same
// reason() string carrying the same kind of answer, and a build that picks a
// half rather than a file.

#include "lidar.hxx"

#include <cstdio>

namespace lidar
{
  namespace
  {

    // Why the last refusing call refused. Shared by both halves below so that
    // reason() has one definition, and so a caller on the laptop reads the
    // same shape of answer it will read on the board.
    Str why;

  }

  const Str& reason()
  {
      return why;
  }

}

#if defined(__linux__) && defined(PILOT_HAVE_RPLIDAR)

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace lidar
{
  namespace
  {

    // One revolution never comes close to this, but grabScanDataHq() wants an
    // upper bound and the SDK's own samples use the same figure. Allocated at
    // open() rather than declared as an Array: 8192 nodes is 64 KiB, which is
    // nothing on the Pi but is not something a program that never opens a lidar
    // should carry in .bss for the whole of its life.
    constexpr Size MAX_NODES = 8192;

    // How long stop() is given to be acted on before the motor is cut. Cutting
    // the motor first can leave the scan running, and the next startScan then
    // meets a device already mid-stream. The figure is the SDK sample's.
    constexpr Int64 STOP_SETTLE_MS = 200;

    sl::ILidarDriver* drv = nullptr;
    sl::IChannel* channel = nullptr;
    Bool spinning = false;

    Vec<sl_lidar_response_measurement_node_hq_t> nodes;

    Str infoStr;
    Str healthStr;
    Int32 healthStatus = -1;

    // The identity as numbers, filled at open() and cleared at close(). Its
    // `health` member is NOT kept here - device() copies healthStatus into it
    // on the way out, so there is one place the status lives and the record
    // cannot lag a reading that readHealth() just took.
    Device dev;

    // A second descriptor on the port, held for as long as the SDK's is, with
    // the tty marked EXCLUSIVE (TIOCEXCL) - so that every later open() of the
    // device by any other unprivileged process fails with EBUSY, which is the
    // answer probePort() below already knew how to read.
    //
    // Without it that branch could never fire. Linux lets any number of
    // processes open one tty, and the SDK takes no lock, so on 2026-09-07
    // `pilot --dry` opened /dev/ttyUSB0 WHILE the scan feed was streaming
    // from it: the pilot reported "nothing answered at 460800 baud" (the
    // feed had the bytes), and the feed lost five revolutions in a row to
    // the pilot's probe commands. Two programs each convinced the other did
    // not exist. The flag is cleared by the kernel when the last descriptor
    // on the tty closes, so a crash releases it as surely as close() does.
    Int32 guardFd = -1;

    Void dropGuard()
    {
        if(guardFd >= 0)
        {
            ::close(guardFd);
            guardFd = -1;
        }
    }

    [[nodiscard]] Str hex(const sl_result r)
    {
        Array<Char, 16> buf{};
        std::snprintf(buf.data(), buf.size(), "0x%X", static_cast<unsigned>(r));
        return Str(buf.data());
    }

    // Opens the port, to find out whether it CAN be opened before the SDK
    // touches it, and keeps the descriptor as guardFd for open() to mark
    // exclusive once the SDK has its own.
    //
    // This exists because the SDK cannot be trusted to report an unopenable
    // port: hub/src/lidar_source.cxx documents the shadowed `ans` in the SDK's
    // openChannelAndBind() that turns a failed channel open into a successful
    // connect(). Without this probe a missing /dev/ttyUSB0 is reported two
    // seconds later as "nothing answered at 460800 baud", pointing whoever
    // reads it at the baud rate when the cable is the problem.
    //
    // errno is turned into words HERE, because the three causes want three
    // different actions from the person reading them: plug it in, add yourself
    // to dialout, or find the other program.
    [[nodiscard]] Bool probePort(const Str& port)
    {
        // ::open, not open - the unqualified name inside this namespace is
        // lidar::open, which is the function calling this.
        const Int32 fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(fd >= 0)
        {
            guardFd = fd;
            return true;
        }

        switch(errno)
        {
        case ENOENT:
            why = "no such port " + port;
            break;
        case EACCES:
            why = "permission denied opening " + port + " - is this user in the dialout group?";
            break;
        case EBUSY:
            why = "another program has " + port;
            break;
        default:
            why = "cannot open " + port + ": " + std::strerror(errno);
            break;
        }
        return false;
    }

    // Reads the device's self-report into healthStr, and returns whether the
    // READ worked - not whether the device is healthy. A device that answers
    // "error" has answered; a device that does not answer is a different fact.
    [[nodiscard]] Bool readHealth()
    {
        sl_lidar_response_device_health_t h{};
        const sl_result r = drv->getHealth(h);
        if(SL_IS_FAIL(r))
        {
            healthStr = "unreadable (SDK code " + hex(r) + ")";
            healthStatus = -1;
            why = "the lidar did not answer a health query (SDK code " + hex(r) + ")";
            return false;
        }

        healthStatus = h.status;
        switch(h.status)
        {
        case SL_LIDAR_STATUS_OK:
            healthStr = "ok";
            break;
        case SL_LIDAR_STATUS_WARNING:
            healthStr = "warning (code " + hex(h.error_code) + ")";
            break;
        case SL_LIDAR_STATUS_ERROR:
            healthStr = "error (code " + hex(h.error_code) + ") - power cycle the device";
            break;
        default:
            healthStr = "status " + hex(h.status) + " (code " + hex(h.error_code) + ")";
            break;
        }
        return true;
    }

    // The SDK's packed reply as this header's record. firmware_version is one
    // 16-bit word, major in the high byte - the split is the SDK sample's.
    [[nodiscard]] Device record(const sl_lidar_response_device_info_t& di)
    {
        Device d;
        d.model = static_cast<Int32>(di.model);
        d.fwMajor = static_cast<Int32>(di.firmware_version >> 8);
        d.fwMinor = static_cast<Int32>(di.firmware_version & 0xFF);
        d.hwRev = static_cast<Int32>(di.hardware_version);
        for(const sl_u8 b : di.serialnum)
        {
            Array<Char, 4> two{};
            std::snprintf(two.data(), two.size(), "%02X", static_cast<unsigned>(b));
            d.serial += two.data();
        }
        return d;
    }

    // The sentence info() returns, built from the record so the two cannot
    // disagree about a field.
    [[nodiscard]] Str describe(const Device& d)
    {
        Array<Char, 96> buf{};
        std::snprintf(
            buf.data(),
            buf.size(),
            "model 0x%02X  firmware %d.%02d  hardware %d  serial ",
            static_cast<unsigned>(d.model),
            d.fwMajor,
            d.fwMinor,
            d.hwRev
        );
        return Str(buf.data()) + d.serial;
    }

    // stop, settle, motor off - in that order, and BOTH halves run whatever the
    // first one said. A failed stop with the motor still spinning is a lidar
    // running on a desk with no program attached, which is the outcome this
    // sequence exists to prevent; see "when a hardware sequence may
    // short-circuit" in docs/conventions.md.
    [[nodiscard]] Bool stopAndPark()
    {
        const sl_result stopped = drv->stop();
        sleepMs(STOP_SETTLE_MS);
        const sl_result parked = drv->setMotorSpeed(0);

        // Whatever the device did, grab() must not believe there is a scan.
        spinning = false;

        if(SL_IS_FAIL(stopped))
        {
            why = "the lidar did not acknowledge stop (SDK code " + hex(stopped) + ")";
            return false;
        }
        if(SL_IS_FAIL(parked))
        {
            why = "the lidar did not accept motor off (SDK code " + hex(parked) + ")";
            return false;
        }
        return true;
    }

  }

  Bool available()
  {
      return true;
  }

  Bool open(const Str& port, const Int32 baud)
  {
      why.clear();
      if(drv != nullptr)
      {
          return true;
      }

      if(!probePort(port))
      {
          return false;
      }

      sl::Result<sl::ILidarDriver*> d = sl::createLidarDriver();
      if(!d || *d == nullptr)
      {
          dropGuard();
          why = "the SDK could not create a driver (SDK code " + hex(d.err) + ")";
          return false;
      }

      sl::Result<sl::IChannel*> ch = sl::createSerialPortChannel(port, baud);
      if(!ch || *ch == nullptr)
      {
          delete *d;
          dropGuard();
          why = "the SDK could not create a serial channel for " + port
              + " (SDK code " + hex(ch.err) + ")";
          return false;
      }

      sl_result r = (*d)->connect(*ch);
      if(SL_IS_FAIL(r))
      {
          delete *d;
          delete *ch;
          dropGuard();
          why = "cannot connect to " + port + " (SDK code " + hex(r) + ")";
          return false;
      }

      // Only NOW, with the SDK's own descriptor open: TIOCEXCL refuses every
      // open() that comes after it, and the SDK's would have been one of
      // them. Best effort - a tty that will not take the flag is still a
      // working lidar, just an unguarded one.
      static_cast<Void>(::ioctl(guardFd, TIOCEXCL));

      // The first real exchange. The probe above proved the port opens, so a
      // silence here is the device end: the wrong baud, or a serial adapter
      // with nothing behind it.
      sl_lidar_response_device_info_t di{};
      r = (*d)->getDeviceInfo(di);
      if(SL_IS_FAIL(r))
      {
          (*d)->disconnect();
          delete *d;
          delete *ch;
          dropGuard();
          why = port + " opened but nothing answered at " + std::to_string(baud) + " baud"
              + " (SDK code " + hex(r) + ") - wrong baud, or not a lidar";
          return false;
      }

      drv = *d;
      channel = *ch;
      nodes.assign(MAX_NODES, sl_lidar_response_measurement_node_hq_t{});
      dev = record(di);
      infoStr = describe(dev);

      // A previous session that was TERMINATED rather than closed - a crash,
      // a kill, a lost SSH session - leaves the C1 spinning and streaming,
      // because nothing ever sent the stop. Park it now so that `spinning ==
      // false` is a fact about the device and not a guess about history. The
      // result is not a failure of open(): the device has already answered,
      // and motorOn() will report the next thing it refuses.
      static_cast<Void>(stopAndPark());

      // Recorded, not judged: a device answering "error" has still opened, and
      // the caller can print info() and health() side by side before deciding.
      // motorOn() is where an error refuses.
      static_cast<Void>(readHealth());
      why.clear();
      return true;
  }

  Void close()
  {
      if(drv == nullptr)
      {
          return;
      }

      // Unconditional, not guarded by `spinning`: the cost of telling a stopped
      // device to stop is nothing, and the cost of skipping it is the lidar the
      // note in open() describes.
      static_cast<Void>(stopAndPark());

      drv->disconnect();
      delete drv;
      drv = nullptr;

      // The driver only closes the channel on teardown, it never frees it, so
      // ownership comes back here. Must follow `delete drv`, which still uses it.
      delete channel;
      channel = nullptr;

      // Last, so the port is never unguarded while the SDK still has it.
      dropGuard();

      nodes.clear();
      nodes.shrink_to_fit();
      infoStr.clear();
      healthStr.clear();
      healthStatus = -1;
      dev = Device{};
  }

  Bool isOpen()
  {
      return drv != nullptr;
  }

  Bool motorOn()
  {
      if(drv == nullptr)
      {
          why = "the lidar is not open";
          return false;
      }
      why.clear();
      if(spinning)
      {
          return true;
      }

      // A hard health error means the unit will not produce usable data until
      // it is power cycled; starting the motor anyway just makes noise. Read
      // fresh rather than trusting open()'s copy - the whole point of the
      // status is that it can change.
      if(!readHealth())
      {
          return false;
      }
      if(healthStatus == SL_LIDAR_STATUS_ERROR)
      {
          why = "the lidar reports " + healthStr;
          return false;
      }

      // DEFAULT_MOTOR_SPEED asks the device for the speed IT wants, which on a
      // C1 is an RPM figure the firmware knows and this file does not.
      sl_result r = drv->setMotorSpeed();
      if(SL_IS_FAIL(r))
      {
          why = "the lidar did not accept motor on (SDK code " + hex(r) + ")";
          return false;
      }

      // force = false: no scan while the health says not to. useTypicalScan =
      // true: the mode the device recommends for itself, which for the C1 is
      // the only one worth having, rather than a mode id copied from a
      // datasheet for a different unit.
      sl::LidarScanMode mode{};
      r = drv->startScan(false, true, 0, &mode);
      if(SL_IS_FAIL(r))
      {
          // The motor was told to spin and the scan did not start, so the two
          // disagree; put the motor back where a failed motorOn() leaves things.
          static_cast<Void>(drv->setMotorSpeed(0));
          why = "the lidar did not start scanning (SDK code " + hex(r) + ")";
          return false;
      }

      spinning = true;
      return true;
  }

  Bool motorOff()
  {
      if(drv == nullptr)
      {
          why = "the lidar is not open";
          return false;
      }
      why.clear();
      if(!spinning)
      {
          return true;
      }
      return stopAndPark();
  }

  Bool isSpinning()
  {
      return spinning;
  }

  Bool grab(Vec<reactive::Ray>& out, const Int32 timeoutMs, Vec<UInt8>* quality)
  {
      // Emptied FIRST, so that every path out of here that is not a fresh
      // revolution leaves nothing stale behind - see the header for why an
      // empty scan is the safe thing to hand to reactive::step. The quality
      // vector follows the same rule, so it is never longer than `out`.
      out.clear();
      if(quality != nullptr)
      {
          quality->clear();
      }

      if(drv == nullptr)
      {
          why = "the lidar is not open";
          return false;
      }
      if(!spinning)
      {
          why = "the motor is off - call motorOn() first";
          return false;
      }

      Size count = nodes.size();
      const sl_u32 timeout = timeoutMs < 0 ? 0u : static_cast<sl_u32>(timeoutMs);
      const sl_result r = drv->grabScanDataHq(nodes.data(), count, timeout);
      if(SL_IS_FAIL(r))
      {
          if(r == SL_RESULT_OPERATION_TIMEOUT)
          {
              why = "no complete revolution within " + std::to_string(timeoutMs) + " ms";
          }
          else
          {
              why = "grab failed (SDK code " + hex(r) + ")";
          }
          return false;
      }

      // Sorted by angle so a consumer walking the vector sees the room in
      // order. Its one failure is "every node invalid", and that is not hidden
      // by ignoring it: the loop below then produces a revolution of zero
      // distances, which reactive::step reads as blind and stops the car.
      static_cast<Void>(drv->ascendScanData(nodes.data(), count));

      out.reserve(count);
      if(quality != nullptr)
      {
          quality->reserve(count);
      }
      for(Size i = 0; i < count; ++i)
      {
          const sl_lidar_response_measurement_node_hq_t& n = nodes[i];
          reactive::Ray ray;
          // angle_z_q14 is q14 fixed point scaled so that 1.0 == 90 degrees;
          // dist_mm_q2 is q2 millimetres. Both conversions are the SDK
          // sample's, and hub/src/lidar_source.cxx's, to the constant.
          ray.angleDeg = static_cast<Float32>(n.angle_z_q14) * 90.0f / 16384.0f;
          ray.distMm = static_cast<Float32>(n.dist_mm_q2) / 4.0f;
          out.push_back(ray);
          if(quality != nullptr)
          {
              // The low two bits of the byte are flags; the strength is the
              // six above them, which is where the wire's 0..63 comes from.
              quality->push_back(static_cast<UInt8>(n.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT));
          }
      }

      why.clear();
      return true;
  }

  Str info()
  {
      return drv == nullptr ? Str() : infoStr;
  }

  Device device()
  {
      if(drv == nullptr)
      {
          return Device{};
      }
      Device d = dev;
      d.health = healthStatus;
      return d;
  }

  Str health()
  {
      if(drv == nullptr)
      {
          return Str();
      }
      if(!spinning)
      {
          // The result is deliberately dropped: a failed read writes its own
          // account into healthStr, and that account IS the answer.
          static_cast<Void>(readHealth());
      }
      return healthStr;
  }

}

#else

// The refusing half. Every entry point says that there is no SDK here, in
// those words, rather than "no such port" - which would send somebody to check
// a cable when the truth is that this program was built without the code to
// talk to one.

namespace lidar
{
  namespace
  {

    constexpr CharSeq NO_SDK =
        "no lidar SDK is built into this program"
        " - configure with -DPILOT_RPLIDAR_SDK=<path> on Linux";

  }

  Bool available()
  {
      return false;
  }

  Bool open(const Str& port, const Int32 baud)
  {
      static_cast<Void>(port);
      static_cast<Void>(baud);
      why = NO_SDK;
      return false;
  }

  Void close()
  {
  }

  Bool isOpen()
  {
      return false;
  }

  Bool motorOn()
  {
      why = NO_SDK;
      return false;
  }

  Bool motorOff()
  {
      why = NO_SDK;
      return false;
  }

  Bool isSpinning()
  {
      return false;
  }

  Bool grab(Vec<reactive::Ray>& out, const Int32 timeoutMs, Vec<UInt8>* quality)
  {
      static_cast<Void>(timeoutMs);
      // Emptied, as the header promises, so a caller that ignores the Bool
      // hands reactive::step a blind scan and not whatever was there before.
      out.clear();
      if(quality != nullptr)
      {
          quality->clear();
      }
      why = NO_SDK;
      return false;
  }

  Str info()
  {
      return Str();
  }

  Device device()
  {
      return Device{};
  }

  Str health()
  {
      return Str();
  }

}

#endif
