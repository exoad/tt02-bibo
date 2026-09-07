// scanfeed - the C1's revolutions, streamed to whoever connects on TCP 8011.
//
//   scanfeed [port]        default /dev/ttyUSB0; listens on 0.0.0.0:8011
//
// The board's half of scanwire.hxx. The hub on the laptop connects across the
// hotspot, reads INFO, HEALTH and MOTOR 1, then one F line per revolution, and
// may write MOTOR 0 / MOTOR 1 / QUIT back. `nc bibobox.local 8011` is the
// whole client a person needs to see that the lidar is alive.
//
// ---------------------------------------------------------------------------
// IDLE MEANS THE PORT IS FREE
//
// With no client connected the lidar is CLOSED, not merely stopped. The pilot
// holds the same serial port exclusively when it runs, and the SDK cannot share
// one, so a feed that kept the device open between viewers would be a feed
// that stopped the car from driving. The first client opens the device and the
// last client's departure parks and closes it. This program runs under systemd
// all day precisely because being connected to nobody costs the car nothing.
//
// ---------------------------------------------------------------------------
// WHILE THE PILOT DRIVES, THIS PROGRAM RELAYS
//
// The pilot cannot take port 8011 from a service that is idling on it, and
// nobody on the board has root to stop the service, so the pilot serves the
// same wire on scanwire::PILOT_PORT and THIS program stays the address a
// viewer dials. A client arrives; open() refuses because another program has
// the port - lidar::Refusal::REFUSAL_HELD, a value and not a sentence - and
// the client is handed to the pilot's feed by src/feed.hxx's relay: the
// pilot's greeting, F lines and D lines reach it as they are, its MOTOR lines
// go to the pilot, which refuses them itself. When the pilot exits, its feed
// closes, the relay closes the client, the hub's retry reconnects a second
// later, and open() then succeeds: the device is this program's again with
// nobody having typed anything. A port held by something that is NOT serving
// on PILOT_PORT (lidar_probe, say) fails the relay's connect, and that client
// gets the ERR line it always got. Any other refusal is an ERR line as before.
//
// Every arrival asks for the device, not only the first: a second viewer
// while the pilot drives must be relayed too, and asking for a device that is
// already open costs nothing.
//
// ---------------------------------------------------------------------------
// TWO THREADS, ONE OWNER EACH
//
// This thread - main - is the ONLY thread that calls lidar::*. Those functions
// share file-scope state with no lock (one device, one caller - lidar.hxx),
// and grab() blocks for up to two seconds, which a socket loop cannot afford.
// The sockets are src/feed.hxx's thread, and it never touches the device: it
// hands over two wishes - wantOpen, from the client count; wantMotor, from a
// MOTOR line - under a mutex, and this thread makes the device match them
// between grabs, publishing what happened (opened, failed, motor state, a
// frame) back through the feed. The self-pipe inside feed.cxx means a frame
// reaches the sockets the moment it is formatted rather than at the next
// poll timeout.
//
// The cost is latency on a wish: while the motor is spinning up, this thread
// sits in a 2 s grab and a MOTOR 0 waits for it. That is the correct trade -
// the alternative is two threads inside the SDK at once.
//
// What a slow client costs, and why it is dropped rather than waited for, is
// feed.hxx's business now and is written down there.
//
// ---------------------------------------------------------------------------
// SIGNALS PARK THE DEVICE
//
// SIGINT and SIGTERM (systemctl stop) set a flag, the loop notices, the feed
// is stopped, and the device is parked - motor off, port closed - on the way
// out. The handlers are installed before the socket is bound and long before
// anything can open the lidar, so there is no window in which a stop leaves
// the C1 spinning on the bench with nobody attached - the mess lidar_probe.cxx
// describes and this project has made once already.
//
// Exits 0 on a signal. Exits 1 only when it could not listen at all.

#include "shared.hxx"

#include "feed.hxx"
#include "lidar.hxx"
#include "scanwire.hxx"

#include <csignal>
#include <cstdio>

#if defined(__linux__)

namespace
{

  // lidar.hxx: the first grab after motorOn() times out at this figure, every
  // time, while the motor comes up to speed. Known, and not an error.
  constexpr Int32 GRAB_TIMEOUT_MS = 2000;

  // The longest the loop sleeps with nothing to do. Bounds how long a signal
  // waits to be noticed, since a signal does not wake a condition variable.
  constexpr Int32 IDLE_WAIT_MS = 250;

  // Written from the signal handler, read from the loop. volatile
  // sig_atomic_t is the one type the standard promises is safe in a handler.
  volatile std::sig_atomic_t interrupted = 0;

  Void onInterrupt(Int32)
  {
      interrupted = 1;
  }

  // The wishes, set by the feed thread's callbacks and read by the lidar
  // loop. Every member is touched under `m`.
  struct Wishes
  {
      Mutex   m;
      CondVar cv;
      Bool    wantOpen = false;
      Bool    wantMotor = false;
      Size    clients = 0;   // the count the feed last reported
  };

  Wishes wishes;

  // ---- the feed's callbacks, on the feed thread ------------------------------

  // The wishes follow the client count: every arrival asks for the device
  // (granted already, or relayed - see the header), the first one also asks
  // for the motor, the last one leaving withdraws both. An arrival while the
  // device is open and the motor off changes nothing about the motor -
  // somebody may have turned it off on purpose.
  Void onClients(Size n)
  {
      {
          LockGuard<Mutex> lock(wishes.m);
          if(n > wishes.clients)
          {
              wishes.wantOpen = true;
          }
          if(wishes.clients == 0 && n > 0)
          {
              wishes.wantMotor = true;
          }
          else if(wishes.clients > 0 && n == 0)
          {
              wishes.wantOpen = false;
              wishes.wantMotor = false;
          }
          wishes.clients = n;
      }
      wishes.cv.notify_one();
  }

  Void onMotor(Bool on)
  {
      {
          LockGuard<Mutex> lock(wishes.m);
          wishes.wantMotor = on;
      }
      wishes.cv.notify_one();
  }

  // ---- the lidar loop, on this thread ----------------------------------------

  [[nodiscard]] Str frameLine(const Vec<reactive::Ray>& rays, const Vec<UInt8>& quality, Float32 hz)
  {
      scanwire::Frame f;
      f.hz = hz;
      f.samples.reserve(rays.size());
      for(Size i = 0; i < rays.size(); ++i)
      {
          scanwire::Sample s;
          s.angleDeg = rays[i].angleDeg;
          s.distMm = rays[i].distMm;
          s.quality = i < quality.size() ? quality[i] : static_cast<UInt8>(0);
          f.samples.push_back(s);
      }
      return scanwire::formatFrame(f);
  }

  // INFO and HEALTH while open, then the latest MOTOR line: what a late
  // joiner is told so it does not have to infer the device from the frames.
  struct Greeting
  {
      Str device;
      Str motor;

      Void apply() const
      {
          feed::setGreeting(device + motor);
      }
  };

  // Motor off, port closed. Safe with nothing open; says nothing then.
  Void park(Greeting& greeting)
  {
      if(!lidar::isOpen())
      {
          return;
      }
      if(!lidar::motorOff())
      {
          std::printf("motor off failed: %s\n", lidar::reason().c_str());
      }
      lidar::close();
      std::printf("parked: motor off, port closed\n");
      greeting.device.clear();
      greeting.motor.clear();
      greeting.apply();
  }

  // The wish is withdrawn when it cannot be granted, so the wait below does
  // not spin on it. The clients' departure - closed after an ERR, or when
  // the pilot's feed closes under a relay - withdraws it again.
  Void withdrawOpen()
  {
      LockGuard<Mutex> lock(wishes.m);
      wishes.wantOpen = false;
  }

  // A failure the clients have to hear about.
  Void fail(Greeting& greeting, const Str& why)
  {
      withdrawOpen();
      std::printf("refused: %s\n", why.c_str());
      greeting.device.clear();
      greeting.motor.clear();
      greeting.apply();
      feed::fail(why);
  }

  // The device is the pilot's: every client not yet served is handed to the
  // pilot's feed instead - see the header. The lidar's reason travels with
  // the request, so a relay the pilot's end refuses answers as fail() would.
  Void relayToPilot(const Str& why)
  {
      withdrawOpen();
      std::printf(
          "%s - relaying to the pilot on %u\n",
          why.c_str(),
          static_cast<unsigned>(scanwire::PILOT_PORT)
      );
      feed::relay("127.0.0.1", scanwire::PILOT_PORT, why);
  }

  Void lidarLoop(const Str& port)
  {
      Vec<reactive::Ray> rays;
      Vec<UInt8>         quality;
      TimePoint          last;
      Bool               haveLast = false;
      Greeting           greeting;

      while(interrupted == 0)
      {
          Bool wantOpen = false;
          Bool wantMotor = false;
          {
              UniqueLock<Mutex> lock(wishes.m);
              // Sleep only while the device already matches every wish and is
              // not spinning; a spinning device has revolutions to collect.
              // A timed wait, because the signal handler cannot notify.
              wishes.cv.wait_for(
                  lock,
                  Millis(IDLE_WAIT_MS),
                  [&]
                  {
                      return interrupted != 0 || lidar::isSpinning() || wishes.wantOpen != lidar::isOpen()
                          || (lidar::isOpen() && wishes.wantMotor != lidar::isSpinning());
                  }
              );
              if(interrupted != 0)
              {
                  std::printf("stopping: parking the lidar\n");
                  break;
              }
              wantOpen = wishes.wantOpen;
              wantMotor = wishes.wantMotor;
          }

          if(!wantOpen)
          {
              park(greeting);
              haveLast = false;
              continue;
          }

          if(!lidar::isOpen())
          {
              if(!lidar::open(port))
              {
                  if(lidar::refusal() == lidar::Refusal::REFUSAL_HELD)
                  {
                      relayToPilot(lidar::reason());
                  }
                  else
                  {
                      fail(greeting, lidar::reason());
                  }
                  continue;
              }
              const lidar::Device d = lidar::device();
              std::printf("device opened: %s\n", lidar::info().c_str());

              scanwire::Info info;
              info.model = d.model;
              info.fwMajor = d.fwMajor;
              info.fwMinor = d.fwMinor;
              info.hwRev = d.hwRev;
              info.serial = d.serial;
              Str hello = scanwire::formatInfo(info);
              // HEALTH carries only the three values the wire defines. A -1
              // (the device did not answer) is left out rather than sent as a
              // line every reader would have to reject.
              if(d.health >= 0 && d.health <= 2)
              {
                  hello += scanwire::formatHealth(d.health);
              }
              else
              {
                  std::printf("health unknown: %s\n", lidar::health().c_str());
              }
              greeting.device = hello;
              greeting.apply();
              feed::publish(hello);
          }

          if(wantMotor != lidar::isSpinning())
          {
              if(wantMotor)
              {
                  if(!lidar::motorOn())
                  {
                      fail(greeting, lidar::reason());
                      continue;
                  }
                  haveLast = false;
              }
              else if(!lidar::motorOff())
              {
                  std::printf("motor off failed: %s\n", lidar::reason().c_str());
              }
              // The state the device is actually in, not the one requested.
              const Bool on = lidar::isSpinning();
              std::printf("motor %s\n", on ? "on" : "off");
              greeting.motor = scanwire::formatMotor(on);
              greeting.apply();
              feed::publish(greeting.motor);
          }

          if(lidar::isSpinning())
          {
              if(!lidar::grab(rays, GRAB_TIMEOUT_MS, &quality))
              {
                  std::printf("no revolution: %s\n", lidar::reason().c_str());
                  continue;
              }
              // Measured from the wall clock between revolutions, as
              // lidar_probe does, because that is the rate a reader will
              // actually see frames arrive at. 0 until there are two.
              const TimePoint now = monoNow();
              Float32 hz = 0.0f;
              if(haveLast)
              {
                  const Float64 s = Duration<Float64>(now - last).count();
                  hz = s > 0.0 ? static_cast<Float32>(1.0 / s) : 0.0f;
              }
              last = now;
              haveLast = true;
              feed::publish(frameLine(rays, quality, hz));
          }
      }

      park(greeting);
  }

  [[nodiscard]] Int32 run(const Str& port)
  {
      // Under systemd stdout is a pipe to the journal, and fully buffered by
      // default - "device opened" would appear when the buffer filled, hours
      // later. One line, one write.
      std::setvbuf(stdout, nullptr, _IOLBF, 0);

      // Before the socket, before the thread, before anything can open the
      // lidar - see the header.
      std::signal(SIGINT, onInterrupt);
      std::signal(SIGTERM, onInterrupt);
      std::signal(SIGPIPE, SIG_IGN);

      feed::Policy policy;
      policy.motor = feed::Motor::MOTOR_OBEY;
      policy.onMotor = onMotor;
      policy.onClients = onClients;
      // No fallbackPort, deliberately: this is the address viewers dial. A
      // pilot already on 8011 (started with no service running) means exit 1
      // and systemd retrying every two seconds until the pilot is done - a
      // feed that moved to 8012 would be relaying to itself.
      if(!feed::start(scanwire::PORT, policy))
      {
          return 1;
      }
      std::printf("serving %s\n", port.c_str());

      lidarLoop(port);
      feed::stop();
      return 0;
  }

}

Int32 main(Int32 argc, Char** argv)
{
    const Str port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    return run(port);
}

#else

// Not a stub that listens and reports an empty room: a program that says it
// cannot do the job here. The feed needs Linux sockets and lidar.cxx's real
// half, and this build has neither.
Int32 main(Int32 argc, Char** argv)
{
    static_cast<Void>(argc);
    static_cast<Void>(argv);
    std::printf(
        "scanfeed runs on the board: it needs Linux sockets and the lidar SDK, and this build has neither\n"
    );
    return 1;
}

#endif
