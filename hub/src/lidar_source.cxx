// Two workers behind one LidarSource: the serial one over Slamtec's SDK, the
// network one over the board's scan feed. See lidar_source.hxx for why both.
//
// The SDK is entirely blocking: grabScanDataHq() parks until a full revolution
// has been assembled (~100ms at 10Hz), and connect()/getDeviceInfo() can sit on
// a serial timeout for a second or more. All of that lives on a worker thread
// so the UI never stalls. The worker owns the driver object outright - it is
// created and destroyed on that thread and never touched from anywhere else -
// which keeps the lifetime rules trivial. What crosses threads is only plain
// data: state, error text, device info, and the most recent completed frame,
// each guarded below.
//
// The feed worker (runFeed, at the bottom) owns its socket the same way, and
// publishes through the same members with the same counters, so poll() and
// everything above it cannot tell the two apart.

#include "shared.hxx"
#include "lidar_source.hxx"

#include "devlink.hxx"
#include "scanwire.hxx"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <algorithm>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

// BEFORE windows.h, which otherwise pulls in the original winsock.h that
// winsock2.h then redefines half of - see pico_link.cxx, which met the same
// hundred errors first.
#include <winsock2.h>
#include <ws2tcpip.h>

#include <windows.h>

#pragma comment(lib, "ws2_32.lib")

#include <cctype>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace
{

  // One revolution never comes close to this, but grabScanDataHq() wants an upper
  // bound and the SDK's own samples use the same figure.
  constexpr Size MAX_NODES = 8192;

  // Distinguishes "the scan died" from "this one revolution timed out", which is
  // routine and must not be treated as fatal.
  constexpr sl_u32 GRAB_TIMEOUT_MS = 2000;

  // At a 2s grab timeout this gives the device ~10s of silence before the worker
  // gives up. Long enough to ride out a hiccup, short enough to notice a unplug.
  constexpr Int32 MAX_CONSECUTIVE_TIMEOUTS = 5;

  // ---- the feed's tunables ---------------------------------------------------

  // How long the feed worker waits on the socket before going round to look at
  // the quit flag and the motor flag. A silent board costs two wakeups a second;
  // a Stop or a motor toggle is honoured within one tick.
  constexpr Int32 FEED_TICK_MS = 500;

  // Past this the board is not there. On a LAN a refused connect comes back in
  // a millisecond; the wait is for a name that resolved to an address nothing
  // answers at, which is what an unplugged board on the hotspot looks like.
  constexpr Int32 FEED_CONNECT_TIMEOUT_MS = 3000;

  // A revolution is ~8 KB of text. Bytes that pass this without a newline are
  // not a long line, they are a stream that lost its framing, and keeping them
  // is how a worker grows by one byte per read forever.
  constexpr Size FEED_MAX_LINE = 256 * 1024;

  // "host:port" -> its two halves. The LAST colon splits, so a host that is
  // itself written with one - unlikely, but a v6 literal would be - still
  // finds its port. Returns false when there is no port, or not a number.
  [[nodiscard]] Bool splitTarget(const Str& target, Str* host, UInt16* port)
  {
      const Size colon = target.rfind(':');
      if(colon == Str::npos || colon == 0 || colon + 1 >= target.size())
      {
          return false;
      }
      Char* stop = nullptr;
      const long v = std::strtol(target.c_str() + colon + 1, &stop, 10);
      if(*stop != '\0' || v <= 0 || v > 65535)
      {
          return false;
      }
      *host = target.substr(0, colon);
      *port = static_cast<UInt16>(v);
      return true;
  }

  // One select() on one socket, for `ms`. True when it is ready in the asked
  // direction; false on a timeout OR an error, which the caller then finds out
  // about from the recv/send that follows.
  [[nodiscard]] Bool socketReady(SOCKET s, Bool forWrite, Int32 ms)
  {
      fd_set set;
      FD_ZERO(&set);
      FD_SET(s, &set);
      timeval tv;
      tv.tv_sec = ms / 1000;
      tv.tv_usec = (ms % 1000) * 1000;
      const Int32 n = select(0, forWrite ? nullptr : &set, forWrite ? &set : nullptr, nullptr, &tv);
      return n > 0 && FD_ISSET(s, &set);
  }

  // Writes the whole of `text` to a non-blocking socket, waiting for room
  // when the stack has none. The lines this side sends are a dozen bytes, so
  // the wait is theoretical - but a send that returned short and was not
  // finished would hand the board half a command, which is worse than a slow
  // one. False when the link is gone.
  [[nodiscard]] Bool sendAll(SOCKET s, const Str& text)
  {
      Size done = 0;
      while(done < text.size())
      {
          const Int32 n = send(s, text.data() + done, static_cast<Int32>(text.size() - done), 0);
          if(n > 0)
          {
              done += static_cast<Size>(n);
              continue;
          }
          if(n == SOCKET_ERROR && WSAGetLastError() == WSAEWOULDBLOCK)
          {
              if(!socketReady(s, true, 1000))
              {
                  return false;
              }
              continue;
          }
          return false;
      }
      return true;
  }

  // Checks that the port can actually be opened, returning a specific complaint
  // or an empty string if all is well.
  //
  // This exists because the SDK cannot be trusted to report an unopenable port.
  // In sl_async_transceiver.cpp, openChannelAndBind() declares `u_result ans` and
  // then shadows it inside the do-block with `Result<nullptr_t> ans`; the
  // channel->open() failure is written to the shadowed copy, so the outer value
  // stays RESULT_OK and connect() returns success for a port that was never
  // opened. The symptom is that a missing COM port gets misdiagnosed further
  // down as "no response from device (wrong baud?)", pointing at the wrong cause.
  // Probing here restores the distinction the caller needs.
  //
  // The handle is closed again immediately so the SDK can take the port. That
  // leaves a small window in which another process could claim it in between, in
  // which case the driver's own failure path still reports a problem - just less
  // precisely. Worth it for a correct message in the overwhelmingly common case.
  // Opens and immediately closes the port, to find out whether it CAN be opened
  // before the driver touches it - the driver's own report of this failure is
  // unreliable, which is the whole reason this exists.
  //
  // Returns the Win32 code, or 0 on success. It deliberately does NOT decide what
  // the code MEANS: it used to, with its own copy of the removal list, and that
  // copy drifted from the one the mid-scan path used. An unplugged lidar was a
  // quiet disconnection when the cable came out mid-scan and a red error when you
  // pressed Connect afterwards, which is the same event described two ways.
  // One rule, in devlink.hxx, and this reports into it.
  DWORD probePort(const Str& devPath)
  {
      HANDLE h = CreateFileA(devPath.c_str(),
                             GENERIC_READ | GENERIC_WRITE,
                             0,               // serial ports are exclusive
                             nullptr,
                             OPEN_EXISTING,
                             0,
                             nullptr);
      if(h != INVALID_HANDLE_VALUE)
      {
          CloseHandle(h);
          return 0;
      }
      return GetLastError();
  }

  // The fault-side wording, for codes that are NOT a removal. Only reached when
  // the port is genuinely there and still would not open.
  Str openFailText(const Str& friendly, DWORD err)
  {
      switch(err)
      {
          case ERROR_ACCESS_DENIED:
          case ERROR_SHARING_VIOLATION:
              return "cannot open " + friendly + " (in use by another program)";
          default: {
              Array<Char, 64> buf;
              std::snprintf(
                  buf.data(),
                  buf.size(),
                  " (win32 error %lu)",
                  static_cast<unsigned long>(err)
              );
              return "cannot open " + friendly + buf.data();
          }
      }
  }

}

// ---------------------------------------------------------------------------

struct LidarSource::Impl
{
    Thread       worker;
    Atomic<Bool> quit{false};

    // Owned by the UI thread, read by the worker. Starts true so a connect
    // spins up as it always has.
    Atomic<Bool> motorOn{ true };

    // What the device is doing, as last confirmed: written by the worker when
    // it has switched the motor (serial) or when the board says so (feed).
    // Separate from motorOn because the two differ for the moment between the
    // ask and the act, and motorEnabled() promises the act.
    Atomic<Bool> motorActual{ true };

    // Set once the session got as far as the device itself. See the header.
    Atomic<Bool> reached{ false };

    Atomic<Bool> running{false};   // worker exists and has not been joined

    Atomic<LidarState> state{LidarState::LIDAR_STATE_IDLE};

    // Guards everything the worker publishes to the UI thread.
    mutable Mutex  mtx;
    Str         errorMsg;
    LidarDeviceInfo     devInfo;
    LidarScanInfo       scanInfo;
    LidarFrame          frame;
    UInt64            frameSeq = 0;   // bumped on every published frame

    // Session counters. Written only by the worker, read by the UI thread, so
    // atomics keep them off the publish mutex - the UI reads them every frame
    // while the worker only touches them once per revolution.
    Atomic<UInt64> statFrames{0};
    Atomic<UInt64> statPoints{0};
    Atomic<UInt32>       statTimeouts{0};
    TimePoint scanStart{};
    Atomic<Bool>               scanStarted{false};

    // Touched only by poll(), i.e. only by the UI thread.
    UInt64 lastSeenSeq = 0;

    // The port this session was opened on - "COM7" or "host:port". Written by
    // start() and held under mtx so the UI can read it after the worker has
    // gone, which is exactly when it is wanted: to watch for the device coming
    // back.
    Str openedPort;

    Void setError(const Str& msg)
    {
        {
            LockGuard<Mutex> lock(mtx);
            errorMsg = msg;
        }
        state.store(LidarState::LIDAR_STATE_ERROR, std::memory_order_release);
    }

    // A link that stopped, classified before it is reported. Everything that
    // can fail once the device is open goes through here rather than straight
    // to setError, so "the cable came out" cannot be dressed up as a fault
    // in one code path and not another.
    Void setLost(const Str& port, UInt32 code, const Str& faultMsg)
    {
        const dev::Loss why = dev::classify(port, code);
        {
            LockGuard<Mutex> lock(mtx);
            errorMsg = (why == dev::Loss::LOSS_UNPLUGGED)
                     ? dev::describe(why, "RPLIDAR C1", port)
                     : faultMsg;
        }
        state.store(why == dev::Loss::LOSS_UNPLUGGED
                        ? LidarState::LIDAR_STATE_UNPLUGGED
                        : LidarState::LIDAR_STATE_ERROR,
                    std::memory_order_release);
    }

    // The feed's counterpart to setLost, and NOT a call to it: setLost decides
    // "gone or broken" from a Win32 code and whether the COM port is still
    // enumerated, and a socket has neither - classify() would call every loss
    // an unplug for a reason that does not apply. Here the classification is
    // already made by the caller: the peer closing, or the read failing under
    // us, is the board or the hotspot going away, and that is UNPLUGGED for the
    // reason the header gives.
    Void setGone(const Str& msg)
    {
        {
            LockGuard<Mutex> lock(mtx);
            errorMsg = msg;
        }
        state.store(LidarState::LIDAR_STATE_UNPLUGGED, std::memory_order_release);
    }

    // Publishes one revolution, from either worker, so the counters agree.
    Void publish(LidarFrame& staging)
    {
        const Size count = staging.points.size();

        // The lock is held only for the swap and counter bump, so a UI thread
        // polling at 60Hz never waits on the scan conversion.
        {
            LockGuard<Mutex> lock(mtx);
            std::swap(frame, staging);
            ++frameSeq;
        }

        statFrames.fetch_add(1, std::memory_order_relaxed);
        statPoints.fetch_add(count, std::memory_order_relaxed);
    }

    Void run(Str port, Int32 baud);
    Void runFeed(Str target);
};

// ---------------------------------------------------------------------------

Void LidarSource::Impl::run(Str port, Int32 baud)
{
    using namespace sl;

    state.store(LidarState::LIDAR_STATE_CONNECTING, std::memory_order_release);

    // COM10 and above are only reachable through the \\.\ device namespace, and
    // the prefix is harmless for the single-digit ports, so it always goes on.
    Str devPath = port;
    if(devPath.rfind("\\\\.\\", 0) != 0)
    {
        devPath = "\\\\.\\" + port;
    }

    ILidarDriver* drv = *createLidarDriver();
    if(!drv)
    {
        setError("out of memory creating the lidar driver");
        return;
    }

    // From here on every exit path must tear the driver down, so the body is
    // wrapped and cleanup happens once at the bottom. The channel is declared
    // out here because the SDK never takes ownership of it - unbindAndClose()
    // only close()s it - so this thread has to free it on every exit path.
    Bool      scanning = false;
    IChannel* channel = nullptr;

    do
    {
        // Done before the driver touches the port, because the driver's own
        // report of this failure is unreliable - see probePort().
        const DWORD probe = probePort(devPath);
        if(probe != 0)
        {
            setLost(port, probe, openFailText(port, probe));
            break;
        }

        // Assigns the outer `channel` deliberately - declaring a new one here
        // would shadow it and leak the channel plus its three event handles.
        channel = *createSerialPortChannel(devPath.c_str(), static_cast<sl_u32>(baud));
        if(!channel)
        {
            setError("cannot create a serial channel for " + port);
            break;
        }

        // connect() failing means the port itself would not open: it does not
        // exist, or another process holds it. Nothing to do with baud rate.
        if(SL_IS_FAIL(drv->connect(channel)))
        {
            // Distinguishes "not plugged in" from "another program has it",
            // which are the two causes and want completely different actions
            // from the person reading it.
            setLost(
                port,
                ::GetLastError(),
                "cannot open " + port + " - another program may have it"
            );
            break;
        }

        // The port opened but nothing answered the info request. The cable and
        // the port are fine; the framing is wrong or the device is unpowered.
        sl_lidar_response_device_info_t rawInfo;
        if(SL_IS_FAIL(drv->getDeviceInfo(rawInfo)))
        {
            setLost(port, 0, "no response from device on " + port +
                      " (wrong baud rate?)");
            break;
        }

        LidarDeviceInfo di;
        di.model = static_cast<Int32>(rawInfo.model);
        di.fwMajor = rawInfo.firmware_version >> 8;
        di.fwMinor = rawInfo.firmware_version & 0xFF;
        di.hwRev = static_cast<Int32>(rawInfo.hardware_version);

        Array<Char, 33> hex;
        for(Int32 i = 0; i < 16; ++i)
        {
            std::snprintf(
                hex.data() + i * 2,
                3,
                "%02X",
                static_cast<unsigned>(rawInfo.serialnum[i])
            );
        }
        di.serial = hex.data();

        sl_lidar_response_device_health_t health;
        if(SL_IS_OK(drv->getHealth(health)))
        {
            di.health = static_cast<Int32>(health.status);
        }

        {
            LockGuard<Mutex> lock(mtx);
            devInfo = di;
        }
        reached.store(true, std::memory_order_release);

        // A hard health error means the unit will not produce usable data until
        // it is power cycled; starting the motor anyway just makes noise.
        if(di.health == SL_LIDAR_STATUS_ERROR)
        {
            setError("lidar reports an internal error; power cycle the device");
            break;
        }

        // Stop whatever the device was already doing before starting it.
        //
        // A previous session that was TERMINATED rather than closed - Task
        // Manager, a crash, Stop-Process - leaves the C1 spinning and streaming,
        // because no user-mode code runs on TerminateProcess and nothing ever
        // sent the stop. This makes the recovery deterministic instead of
        // relying on startScan to reset a device that is already mid-scan.
        drv->stop();
        sleepMs(60);

        drv->setMotorSpeed();

        LidarScanMode mode;
        if(SL_IS_FAIL(drv->startScan(0, 1, 0, &mode)))
        {
            setError("failed to start scan on " + port);
            drv->setMotorSpeed(0);
            break;
        }
        scanning = true;

        // Record what the SDK actually negotiated. The C1 chooses the mode, and
        // without this the sample period and the mode's own range ceiling are
        // invisible to the operator.
        {
            LidarScanInfo si;
            si.modeId = static_cast<Int32>(mode.id);
            si.usPerSample = mode.us_per_sample;
            si.maxDistanceM = mode.max_distance;

            // scan_mode is a fixed 64-byte field, zero-padded but not
            // guaranteed terminated.
            Array<Char, sizeof(mode.scan_mode) + 1> name;
            std::memcpy(name.data(), mode.scan_mode, sizeof(mode.scan_mode));
            name[sizeof(mode.scan_mode)] = '\0';
            si.mode = name.data();

            LockGuard<Mutex> lock(mtx);
            scanInfo = si;
        }

        scanStart = monoNow();
        scanStarted.store(true, std::memory_order_release);

        state.store(LidarState::LIDAR_STATE_SCANNING, std::memory_order_release);

        Vec<sl_lidar_response_measurement_node_hq_t> nodes(MAX_NODES);

        // Reused across iterations so the steady state does no allocation once
        // the vector has grown to a revolution's worth of points.
        LidarFrame staging;
        staging.points.reserve(1024);

        Int32 consecutiveTimeouts = 0;

        Bool motorRunning = true;

        while(!quit.load(std::memory_order_acquire))
        {
            // Motor pause. Checked here rather than around the grab because the
            // scan has to be stopped BEFORE the motor - killing the motor under
            // a running scan leaves the device streaming into a stopped rotor.
            const Bool want = motorOn.load(std::memory_order_acquire);
            if(want != motorRunning)
            {
                if(want)
                {
                    drv->setMotorSpeed();
                    LidarScanMode m2;
                    if(SL_IS_FAIL(drv->startScan(0, 1, 0, &m2)))
                    {
                        setError("failed to restart scan on " + port);
                        break;
                    }
                    state.store(LidarState::LIDAR_STATE_SCANNING, std::memory_order_release);
                }
                else
                {
                    drv->stop();
                    sleepMs(200);
                    drv->setMotorSpeed(0);
                    state.store(LidarState::LIDAR_STATE_IDLE, std::memory_order_release);
                }
                motorRunning = want;
                motorActual.store(want, std::memory_order_release);
                consecutiveTimeouts = 0;
            }

            if(!motorRunning)
            {
                // Nothing to grab and nothing to poll. Sleeping rather than
                // spinning keeps a paused lidar off the CPU entirely.
                sleepMs(40);
                continue;
            }

            Size count = nodes.size();

            if(SL_IS_FAIL(drv->grabScanDataHq(nodes.data(), count, GRAB_TIMEOUT_MS)))
            {
                // A dropped revolution is normal - the device occasionally
                // misses its window. A long unbroken run of them is not: the
                // cable came out, or the device stopped talking. Surface that
                // instead of sitting in Scanning forever with a frozen view.
                statTimeouts.fetch_add(1, std::memory_order_relaxed);

                if(++consecutiveTimeouts >= MAX_CONSECUTIVE_TIMEOUTS)
                {
                    // No code to offer - the SDK swallowed it - so the port
                    // enumeration decides on its own, which is the signal that
                    // was authoritative anyway.
                    setLost(port, 0, "device stopped responding on " + port);
                    break;
                }
                continue;
            }
            consecutiveTimeouts = 0;

            drv->ascendScanData(nodes.data(), count);

            Float32 freq = 0.0f;
            drv->getFrequency(mode, nodes.data(), count, freq);

            staging.points.clear();
            staging.hz = freq;
            staging.validCount = 0;
            staging.maxDistMm = 0.0f;

            for(Size i = 0; i < count; ++i)
            {
                LidarPoint p;
                // angle_z_q14 is q14 fixed point scaled so that 1.0 == 90 deg.
                p.angleDeg = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
                p.distMm = nodes[i].dist_mm_q2 / 4.0f;
                p.quality = static_cast<UInt8>((nodes[i].quality >>
                                        SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT));

                if(p.distMm > 0.0f)
                {
                    ++staging.validCount;
                    if(p.distMm > staging.maxDistMm)
                    {
                        staging.maxDistMm = p.distMm;
                    }
                }
                staging.points.push_back(p);
            }

            publish(staging);
        }
    } while(false);

    // Order matters: stop the scan, give the device a moment to acknowledge it,
    // then cut the motor. Killing the motor first can leave the scan running.
    // Unconditional once the device has been opened. `scanning` used to be the
    // guard, but with a pause control the motor can be off while scanning is
    // true, or the scan stopped while the rotor coasts - and the cost of telling
    // an already-stopped device to stop is nothing, while the cost of skipping
    // it is a lidar spinning on a desk with no application attached.
    if(drv != nullptr)
    {
        drv->stop();
        sleepMs(200);
        drv->setMotorSpeed(0);
    }
    static_cast<Void>(scanning);
    delete drv;

    // The driver only close()s the channel on teardown, it never frees it, so
    // ownership comes back here. Must follow `delete drv`, which still uses it.
    delete channel;

    // A clean shutdown returns to Idle; a failure keeps its Error state and the
    // message that explains it.
    const LidarState endState = state.load(std::memory_order_acquire);
    if(endState != LidarState::LIDAR_STATE_ERROR
       && endState != LidarState::LIDAR_STATE_UNPLUGGED)
    {
        state.store(LidarState::LIDAR_STATE_IDLE, std::memory_order_release);
    }
}

// ---------------------------------------------------------------------------
//  the feed worker
// ---------------------------------------------------------------------------
//
// Shape: resolve, connect with a deadline, then one loop that each tick sends
// a MOTOR line if the flag moved, waits up to FEED_TICK_MS for bytes, and
// hands every complete line to scanwire::parse. Everything it learns goes into
// the same members the serial worker fills, through the same publish().
//
// The states mean what they mean on the serial path. CONNECTING until the
// first F line - a TCP accept proves the SERVICE is there, not the lidar, and
// the board only starts streaming once it has the device spinning. SCANNING
// while frames arrive. IDLE when the board says MOTOR 0: the link is open and
// parked, which is what "Motor off" in the UI means. UNPLUGGED when the peer
// goes, ERROR when the board says ERR or the feed could not be reached.

Void LidarSource::Impl::runFeed(Str target)
{
    state.store(LidarState::LIDAR_STATE_CONNECTING, std::memory_order_release);

    Str    host;
    UInt16 port = 0;
    if(!splitTarget(target, &host, &port))
    {
        setError("not a scan feed address: \"" + target + "\" (want host:port)");
        return;
    }

    // The one message for "nothing there", whether refused, timed out or the
    // name did not resolve to anything that answers. It names both things a
    // person can do about it, because those ARE the two causes.
    const Str noFeed = "no scan feed at " + target
                     + " - is scanfeed running on the board, and are you on the same network?";

    // Winsock is reference counted, so start/stop here is correct even if
    // something else in the process has it open too.
    WSADATA wsa;
    ZeroMemory(&wsa, sizeof(wsa));
    if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        setError("WSAStartup failed");
        return;
    }
    struct WsaGuard
    {
        ~WsaGuard()
        {
            WSACleanup();
        }
    } wsaGuard;

    sockaddr_in peer;
    ZeroMemory(&peer, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_port = htons(port);
    if(inet_pton(AF_INET, host.c_str(), &peer.sin_addr) != 1)
    {
        // Not a dotted quad, so a NAME - bibobox.local over the hotspot's
        // mDNS, which Windows resolves natively. The same reasoning as the
        // Pico's UDP link: the address changes every outing, the name does not.
        addrinfo hints;
        ZeroMemory(&hints, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* found = nullptr;
        if(getaddrinfo(host.c_str(), nullptr, &hints, &found) != 0 || found == nullptr)
        {
            setError("cannot resolve " + host + " - is the board on the same network?");
            return;
        }
        peer.sin_addr = reinterpret_cast<sockaddr_in*>(found->ai_addr)->sin_addr;
        freeaddrinfo(found);
    }

    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(s == INVALID_SOCKET)
    {
        setError("socket failed (winsock error " + std::to_string(WSAGetLastError()) + ")");
        return;
    }
    struct SockGuard
    {
        SOCKET h;
        ~SockGuard()
        {
            closesocket(h);
        }
    } sockGuard{s};

    // Non-blocking, so the connect has a deadline this side chooses and a Stop
    // pressed during it is honoured within a tick rather than after the stack's
    // own twenty-second retry schedule.
    u_long nonBlocking = 1;
    ioctlsocket(s, FIONBIO, &nonBlocking);

    if(connect(s, reinterpret_cast<const sockaddr*>(&peer), sizeof(peer)) == SOCKET_ERROR
       && WSAGetLastError() != WSAEWOULDBLOCK)
    {
        setError(noFeed);
        return;
    }

    {
        const TimePoint began = monoNow();
        Bool connected = false;
        while(!connected)
        {
            if(quit.load(std::memory_order_acquire))
            {
                return;
            }
            if(elapsedMs(began) > FEED_CONNECT_TIMEOUT_MS)
            {
                setError(noFeed);
                return;
            }

            // Writable means the handshake finished - one way or the other.
            // SO_ERROR says which; a refusal lands here as WSAECONNREFUSED.
            fd_set write;
            fd_set except;
            FD_ZERO(&write);
            FD_ZERO(&except);
            FD_SET(s, &write);
            FD_SET(s, &except);
            timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = FEED_TICK_MS * 1000;
            if(select(0, nullptr, &write, &except, &tv) <= 0)
            {
                continue;
            }
            Int32 soErr = 0;
            Int32 len = static_cast<Int32>(sizeof(soErr));
            getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<Char*>(&soErr), &len);
            if(FD_ISSET(s, &except) || soErr != 0)
            {
                setError(noFeed);
                return;
            }
            connected = true;
        }
    }

    // The service is there. Whether the lidar behind it is, the first frame
    // will say - but this is the line past which a failure is the board's to
    // explain rather than the network's.
    reached.store(true, std::memory_order_release);

    // The board spins the device up on connect, so the last thing "told" to it
    // is on; the first MOTOR line this side sends is the first change.
    Bool motorSent = true;
    Bool gotFrame = false;
    Bool keepGoing = true;

    Str            accum;
    scanwire::Line line;
    LidarFrame     staging;
    staging.points.reserve(1024);
    Vec<Char> buf(65536);

    const Str gone = "the scan feed on " + host + " went away";

    auto handleLine = [&](StrView text)
    {
        switch(scanwire::parse(text, &line))
        {
            case scanwire::Kind::KIND_FRAME:
            {
                staging.points.clear();
                staging.hz = line.frame.hz;
                staging.validCount = 0;
                staging.maxDistMm = 0.0f;
                for(const scanwire::Sample& smp : line.frame.samples)
                {
                    LidarPoint p;
                    p.angleDeg = smp.angleDeg;
                    p.distMm = smp.distMm;
                    p.quality = smp.quality;
                    if(p.distMm > 0.0f)
                    {
                        ++staging.validCount;
                        if(p.distMm > staging.maxDistMm)
                        {
                            staging.maxDistMm = p.distMm;
                        }
                    }
                    staging.points.push_back(p);
                }
                publish(staging);

                if(!gotFrame)
                {
                    gotFrame = true;
                    scanStart = monoNow();
                    scanStarted.store(true, std::memory_order_release);
                }
                state.store(LidarState::LIDAR_STATE_SCANNING, std::memory_order_release);
                break;
            }
            case scanwire::Kind::KIND_INFO:
            {
                LockGuard<Mutex> lock(mtx);
                devInfo.model = line.info.model;
                devInfo.fwMajor = line.info.fwMajor;
                devInfo.fwMinor = line.info.fwMinor;
                devInfo.hwRev = line.info.hwRev;
                devInfo.serial = line.info.serial;
                break;
            }
            case scanwire::Kind::KIND_HEALTH:
            {
                LockGuard<Mutex> lock(mtx);
                devInfo.health = line.health;
                break;
            }
            case scanwire::Kind::KIND_MOTOR:
                // The board's word, not this side's wish - it is what
                // motorEnabled() promises. Off parks the link in IDLE the way
                // the serial worker does; on is SCANNING only once a frame has
                // proven it, so a spin-up reads as Connecting, not as a picture
                // that has not arrived.
                motorActual.store(line.motor, std::memory_order_release);
                if(!line.motor)
                {
                    state.store(LidarState::LIDAR_STATE_IDLE, std::memory_order_release);
                }
                else if(gotFrame)
                {
                    state.store(LidarState::LIDAR_STATE_SCANNING, std::memory_order_release);
                }
                break;
            case scanwire::Kind::KIND_ERR:
                // The board's words are already for a person. The feed closes
                // after ERR, so stop here rather than let the close that follows
                // rewrite this as "went away".
                setError(line.text.empty() ? Str("the board reported an error") : line.text);
                keepGoing = false;
                break;
            case scanwire::Kind::KIND_BAD:
                // A revolution the board sent and this side could not read
                // whole. Counted where a dropped revolution is counted, never
                // drawn: half a frame is a wrong picture, not a smaller one.
                statTimeouts.fetch_add(1, std::memory_order_relaxed);
                break;
            case scanwire::Kind::KIND_UNKNOWN:
            case scanwire::Kind::KIND_QUIT:
            case scanwire::Kind::KIND_EMPTY:
                // A line from a newer board, or nothing. Not ours to mind.
                break;
        }
    };

    while(keepGoing && !quit.load(std::memory_order_acquire))
    {
        // The motor flag, on this thread's tick: the UI never writes the socket.
        const Bool want = motorOn.load(std::memory_order_acquire);
        if(want != motorSent)
        {
            if(!sendAll(s, scanwire::formatMotor(want)))
            {
                setGone(gone);
                break;
            }
            motorSent = want;
        }

        if(!socketReady(s, false, FEED_TICK_MS))
        {
            continue;
        }

        const Int32 n = recv(s, buf.data(), static_cast<Int32>(buf.size()), 0);
        if(n == 0)
        {
            // An orderly close: the board parked the device and hung up, or
            // scanfeed was stopped. Either way it is not here any more.
            setGone(gone);
            break;
        }
        if(n < 0)
        {
            if(WSAGetLastError() == WSAEWOULDBLOCK)
            {
                continue;
            }
            // A reset mid-stream is what a rebooting board or a dropped hotspot
            // looks like from here. Same thing, same state.
            setGone(gone);
            break;
        }

        accum.append(buf.data(), static_cast<Size>(n));

        Size from = 0;
        for(;;)
        {
            const Size nl = accum.find('\n', from);
            if(nl == Str::npos)
            {
                break;
            }
            handleLine(StrView(accum).substr(from, nl - from));
            from = nl + 1;
            if(!keepGoing)
            {
                break;
            }
        }
        accum.erase(0, from);

        if(accum.size() > FEED_MAX_LINE)
        {
            // Whatever this was, it was not a line. Drop it as one bad
            // revolution and pick the stream up at the next newline.
            accum.clear();
            statTimeouts.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // Tell the board we are leaving so it parks the device when the last client
    // goes. (Not a cast to Void: MSVC's C4834 only recognises the literal
    // `void`, and this project spells it with the alias.)
    if(!sendAll(s, scanwire::formatQuit()))
    {
        // Already gone, then. The board has nobody to park the device for, and
        // it knows that without being told.
    }

    // A clean shutdown returns to Idle; a failure keeps its state and message.
    const LidarState endState = state.load(std::memory_order_acquire);
    if(endState != LidarState::LIDAR_STATE_ERROR
       && endState != LidarState::LIDAR_STATE_UNPLUGGED)
    {
        state.store(LidarState::LIDAR_STATE_IDLE, std::memory_order_release);
    }
}

// ---------------------------------------------------------------------------

LidarSource::LidarSource() : pimpl(new Impl) {}

LidarSource::~LidarSource()
{
    stop();
    delete pimpl;
}

Void LidarSource::start(const Str& port, Int32 baud)
{
    // A connect always spins up, whatever the last session left the flag on.
    pimpl->motorOn.store(true, std::memory_order_release);
    pimpl->motorActual.store(true, std::memory_order_release);
    pimpl->reached.store(false, std::memory_order_release);

    // Starting over an existing session would leak a thread and leave the motor
    // spinning, so the previous one is retired first.
    stop();

    {
        LockGuard<Mutex> lock(pimpl->mtx);
        pimpl->errorMsg = Str();
        pimpl->devInfo = LidarDeviceInfo();
        pimpl->scanInfo = LidarScanInfo();
        pimpl->frame = LidarFrame();
        pimpl->frameSeq = 0;

        // Recorded HERE, not by the worker: port() has to answer the moment
        // start() returns, and a worker that has not been scheduled yet has
        // not written anything.
        pimpl->openedPort = port;
    }
    pimpl->lastSeenSeq = 0;

    // Counters describe one session, so they restart with it.
    pimpl->statFrames.store(0, std::memory_order_relaxed);
    pimpl->statPoints.store(0, std::memory_order_relaxed);
    pimpl->statTimeouts.store(0, std::memory_order_relaxed);
    pimpl->scanStarted.store(false, std::memory_order_release);

    pimpl->quit.store(false, std::memory_order_release);
    pimpl->state.store(LidarState::LIDAR_STATE_CONNECTING, std::memory_order_release);
    pimpl->running.store(true, std::memory_order_release);

    Impl* impl = pimpl;
    if(isFeedTarget(port))
    {
        pimpl->worker = Thread([impl, port] { impl->runFeed(port); });
    }
    else
    {
        pimpl->worker = Thread([impl, port, baud] { impl->run(port, baud); });
    }
}

Bool LidarSource::isFeedTarget(const Str& port) noexcept
{
    return port.find(':') != Str::npos;
}

Bool LidarSource::connected() const noexcept
{
    return pimpl->running.load(std::memory_order_acquire);
}

Void LidarSource::setMotorEnabled(Bool on)
{
    pimpl->motorOn.store(on, std::memory_order_release);
}

Bool LidarSource::motorEnabled() const noexcept
{
    return pimpl->motorActual.load(std::memory_order_acquire);
}

Bool LidarSource::reachedDevice() const noexcept
{
    return pimpl->reached.load(std::memory_order_acquire);
}

Void LidarSource::stop()
{
    // Safe when never started and safe to call twice: joinable() is false in
    // both cases and there is nothing to unwind.
    if(!pimpl->worker.joinable())
    {
        pimpl->running.store(false, std::memory_order_release);
        return;
    }

    pimpl->quit.store(true, std::memory_order_release);

    // The worker performs the scan stop, the 200ms settle and the motor stop
    // before returning, so joining is exactly the "blocks until the motor is
    // off" guarantee the header promises. Worst case is one grab timeout on
    // the serial path, or one tick plus the QUIT on the feed.
    pimpl->worker.join();
    pimpl->running.store(false, std::memory_order_release);
}

LidarState LidarSource::state() const
{
    return pimpl->state.load(std::memory_order_acquire);
}

Str LidarSource::error() const
{
    LockGuard<Mutex> lock(pimpl->mtx);
    return pimpl->errorMsg;
}

Str LidarSource::port() const
{
    LockGuard<Mutex> lock(pimpl->mtx);
    return pimpl->openedPort;
}

LidarDeviceInfo LidarSource::info() const
{
    LockGuard<Mutex> lock(pimpl->mtx);
    return pimpl->devInfo;
}

LidarScanInfo LidarSource::scanInfo() const
{
    LockGuard<Mutex> lock(pimpl->mtx);
    return pimpl->scanInfo;
}

LidarStats LidarSource::stats() const
{
    LidarStats s;
    s.frames = pimpl->statFrames.load(std::memory_order_relaxed);
    s.points = pimpl->statPoints.load(std::memory_order_relaxed);
    s.timeouts = pimpl->statTimeouts.load(std::memory_order_relaxed);

    if(pimpl->scanStarted.load(std::memory_order_acquire))
    {
        s.uptimeS = elapsedS(pimpl->scanStart);
    }
    return s;
}

Bool LidarSource::poll(LidarFrame& out)
{
    LockGuard<Mutex> lock(pimpl->mtx);

    // Frames land at ~10Hz while this is called at ~60Hz, so the overwhelmingly
    // common path is this comparison and an immediate return.
    if(pimpl->frameSeq == pimpl->lastSeenSeq)
    {
        return false;
    }

    out = pimpl->frame;
    pimpl->lastSeenSeq = pimpl->frameSeq;
    return true;
}

// ---------------------------------------------------------------------------

Str LidarSource::preferredPort()
{
    // HKLM\HARDWARE\DEVICEMAP\SERIALCOMM maps each driver's device name to the
    // COM port it owns. The CP210x driver registers as \Device\Silabser<N>,
    // which identifies the RPLIDAR's USB adapter without needing SetupAPI or
    // WMI - the other ports on a typical machine are Bluetooth links.
    Str found;

    HKEY key = nullptr;
    if(RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) != ERROR_SUCCESS)
    {
        return found;
    }

    for(DWORD i = 0; ; ++i)
    {
        Array<Char, 512> name;
        BYTE  data[512];
        DWORD nlen = static_cast<DWORD>(name.size());
        DWORD dlen = static_cast<DWORD>(sizeof(data));
        DWORD type = 0;

        const LONG r = RegEnumValueA(
            key,
            i,
            name.data(),
            &nlen,
            nullptr,
            &type,
            data,
            &dlen
        );
        if(r != ERROR_SUCCESS)
        {
            break;
        }
        if(type != REG_SZ || dlen == 0)
        {
            continue;
        }

        // Registry strings are not guaranteed to be terminated.
        if(dlen >= sizeof(data))
        {
            dlen = static_cast<DWORD>(sizeof(data)) - 1;
        }
        data[dlen] = 0;

        Str dev(name.data(), nlen);
        for(Char& c : dev)
        {
            c = static_cast<Char>(std::tolower(static_cast<UInt8>(c)));
        }

        if(dev.find("silabser") != Str::npos)
        {
            found = reinterpret_cast<const Char*>(data);
            break;
        }
    }

    RegCloseKey(key);
    return found;
}

Vec<Str> LidarSource::listPorts()
{
    Vec<Str> ports;

    // QueryDosDeviceA with a NULL device name returns every DOS device as a
    // packed run of NUL-terminated strings. It needs no extra import library
    // beyond kernel32, which is why it is preferred over SetupAPI here.
    try
    {
        DWORD cap = 8192;
        Vec<Char> buf;

        for(Int32 attempt = 0; attempt < 6; ++attempt)
        {
            buf.assign(cap, '\0');
            DWORD n = QueryDosDeviceA(nullptr, buf.data(), cap);
            if(n != 0)
            {
                // Walk the packed list; a lone NUL terminates the whole block.
                const Char* p = buf.data();
                const Char* end = buf.data() + n;
                while(p < end && *p)
                {
                    Size len = std::strlen(p);

                    // Accept only COM followed entirely by digits, so entries
                    // such as "COMEDIA" or "COM" alone are rejected.
                    if(len > 3 &&
                        (p[0] == 'C' || p[0] == 'c') &&
                        (p[1] == 'O' || p[1] == 'o') &&
                        (p[2] == 'M' || p[2] == 'm'))
                    {
                        Bool allDigits = true;
                        for(Size i = 3; i < len; ++i)
                        {
                            if(p[i] < '0' || p[i] > '9')
                            {
                                allDigits = false;
                                break;
                            }
                        }
                        if(allDigits)
                        {
                            ports.push_back(Str(p, len));
                        }
                    }
                    p += len + 1;
                }
                break;
            }

            if(GetLastError() != ERROR_INSUFFICIENT_BUFFER)
            {
                break;
            }
            cap *= 2;
        }

        // Numeric order, so COM10 follows COM9 instead of preceding it.
        std::sort(ports.begin(), ports.end(),
                  [](const Str& a, const Str& b) {
                      long na = std::strtol(a.c_str() + 3, nullptr, 10);
                      long nb = std::strtol(b.c_str() + 3, nullptr, 10);
                      if(na != nb)
                      {
                          return na < nb;
                      }
                      return a < b;
                  });
        ports.erase(std::unique(ports.begin(), ports.end()), ports.end());
    }
    catch(...)
    {
        // The header promises this never throws; a partial list beats an
        // exception escaping into the UI's frame loop.
    }

    return ports;
}
