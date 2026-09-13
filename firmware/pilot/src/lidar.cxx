// The SDK half, on Linux with PILOT_HAVE_RPLIDAR, and the refusing half
// everywhere else, in one file so both answer through the same reason() and
// refusal().
#include "lidar.hxx"

#include <cstdio>

namespace lidar
{
  namespace
  {
    // Written by both halves.
    Str why;
    Refusal refused = Refusal::REFUSAL_NONE;
  }

  const Str& reason()
  {
      return why;
  }

  Refusal refusal()
  {
      return refused;
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
    // grabScanDataHq() wants an upper bound; the SDK samples use this one.
    // Allocated at open(), not an Array, so a program that never opens a lidar
    // does not carry it in .bss.
    constexpr Size MAX_NODES = 8192;

    // The SDK sample's wait between stop() and cutting the motor (see motorOff()).
    constexpr Int64 STOP_SETTLE_MS = 200;

    sl::ILidarDriver* drv = nullptr;
    sl::IChannel* channel = nullptr;
    Bool spinning = false;

    Vec<sl_lidar_response_measurement_node_hq_t> nodes;

    Str infoStr;
    Str healthStr;
    Int32 healthStatus = -1;

    // Filled at open(), cleared at close(). Its health is NOT kept here:
    // device() copies healthStatus in, so the record never lags a reading.
    Device dev;

    // A second descriptor on the port, held as long as the SDK's, with the tty
    // marked EXCLUSIVE (TIOCEXCL), so any later unprivileged open() fails with
    // EBUSY and probePort() reports REFUSAL_HELD. Linux lets any number of
    // processes open a tty and the SDK takes no lock: without the guard a second
    // program reads the first one's bytes and both fail. The kernel clears the
    // flag when the last descriptor closes, so a crash releases it too.
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

    // Opens the port before the SDK touches it, keeping the descriptor as
    // guardFd. The SDK cannot report an unopenable port: a shadowed `ans` in its
    // openChannelAndBind() turns a failed open into a successful connect()
    // ("The lidar" in docs/hardware.md), so a missing port would read seconds
    // later as "nothing answered", blaming the baud. errno becomes words here
    // because each cause wants a different fix: plug it in, join dialout, or
    // stop the other program.
    [[nodiscard]] Bool probePort(const Str& port)
    {
        // ::open, because the bare name here is lidar::open.
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
            refused = Refusal::REFUSAL_NO_PORT;
            break;
        case EACCES:
            why = "permission denied opening " + port + " - is this user in the dialout group?";
            refused = Refusal::REFUSAL_NO_PERMISSION;
            break;
        case EBUSY:
            why = "another program has " + port;
            refused = Refusal::REFUSAL_HELD;
            break;
        default:
            why = "cannot open " + port + ": " + std::strerror(errno);
            refused = Refusal::REFUSAL_CANNOT_OPEN;
            break;
        }
        return false;
    }

    // Reads the device's self-report into healthStr. Returns whether the READ
    // worked, not whether the device is healthy.
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

    // firmware_version is one 16-bit word, major in the high byte, as the SDK
    // sample splits it.
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

    // info()'s sentence, built from the record so the two cannot disagree.
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

    // stop, settle, motor off, and the motor is cut whatever stop returned: a
    // failed stop must not leave the lidar spinning with no program attached
    // (the hardware-sequence rule in docs/conventions.md).
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
      refused = Refusal::REFUSAL_NONE;
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
          refused = Refusal::REFUSAL_SDK;
          return false;
      }
      sl::Result<sl::IChannel*> ch = sl::createSerialPortChannel(port, baud);
      if(!ch || *ch == nullptr)
      {
          delete *d;
          dropGuard();
          why = "the SDK could not create a serial channel for " + port
              + " (SDK code " + hex(ch.err) + ")";
          refused = Refusal::REFUSAL_SDK;
          return false;
      }
      sl_result r = (*d)->connect(*ch);
      if(SL_IS_FAIL(r))
      {
          delete *d;
          delete *ch;
          dropGuard();
          why = "cannot connect to " + port + " (SDK code " + hex(r) + ")";
          refused = Refusal::REFUSAL_SDK;
          return false;
      }
      // Only now, after the SDK's own open, which TIOCEXCL would have refused.
      // Best effort: an unguarded lidar still works.
      static_cast<Void>(::ioctl(guardFd, TIOCEXCL));
      // The port opened, so silence here is the device end: the wrong baud, or
      // a serial adapter with nothing behind it.
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
          refused = Refusal::REFUSAL_NOT_A_LIDAR;
          return false;
      }
      drv = *d;
      channel = *ch;
      nodes.assign(MAX_NODES, sl_lidar_response_measurement_node_hq_t{});
      dev = record(di);
      infoStr = describe(dev);
      // A session that was killed rather than closed left the C1 spinning and
      // streaming. Parked, so spinning == false is true of the device. Not a
      // failure of open(): motorOn() reports what it refuses.
      static_cast<Void>(stopAndPark());
      // Recorded, not judged: a health error refuses in motorOn(), not here.
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
      // Unconditional, not guarded by spinning: stopping a stopped device costs
      // nothing, and a skipped stop can leave it spinning.
      static_cast<Void>(stopAndPark());
      drv->disconnect();
      delete drv;
      drv = nullptr;
      // The driver closes the channel on teardown but never frees it. Must
      // follow `delete drv`, which still uses it.
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
      // Read fresh, not open()'s copy: a health ERROR means no usable data
      // until a power cycle, so the motor stays off.
      if(!readHealth())
      {
          return false;
      }
      if(healthStatus == SL_LIDAR_STATUS_ERROR)
      {
          why = "the lidar reports " + healthStr;
          return false;
      }
      // The default argument asks the device for its own speed.
      sl_result r = drv->setMotorSpeed();
      if(SL_IS_FAIL(r))
      {
          why = "the lidar did not accept motor on (SDK code " + hex(r) + ")";
          return false;
      }
      // force false: no scan against the health. useTypicalScan true: the mode
      // the device recommends for itself.
      sl::LidarScanMode mode{};
      r = drv->startScan(false, true, 0, &mode);
      if(SL_IS_FAIL(r))
      {
          // The scan did not start, so the motor is stopped again.
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
      // Emptied first, so no path out leaves a stale revolution behind.
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
      // Sorted by angle. Its only failure, every node invalid, becomes a
      // revolution of zero distances below, which reads as blind.
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
          // angle_z_q14: q14 fixed point where 1.0 is 90 degrees. dist_mm_q2:
          // q2 millimetres. Both conversions are the SDK sample's.
          ray.angleDeg = static_cast<Float32>(n.angle_z_q14) * 90.0f / 16384.0f;
          ray.distMm = static_cast<Float32>(n.dist_mm_q2) / 4.0f;
          out.push_back(ray);
          if(quality != nullptr)
          {
              // The low two bits are flags; the six above are the 0..63 strength.
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
          // A failed read writes its own account into healthStr, which is the
          // answer either way.
          static_cast<Void>(readHealth());
      }
      return healthStr;
  }
}

#else

// The refusing half says there is no SDK, never "no such port", which would send
// someone to check a cable.
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
      refused = Refusal::REFUSAL_NO_SDK;
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
