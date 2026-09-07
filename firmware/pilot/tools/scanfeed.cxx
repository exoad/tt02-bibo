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
// last client's departure parks and closes it; if open() fails because the
// pilot has the port, that client is told why in an ERR line and closed, and
// the next client simply tries again. This program runs under systemd all day
// precisely because being connected to nobody costs the car nothing.
//
// ---------------------------------------------------------------------------
// TWO THREADS, ONE OWNER EACH
//
// The lidar thread is the ONLY thread that calls lidar::*. Those functions
// share file-scope state with no lock (one device, one caller - lidar.hxx),
// and grab() blocks for up to two seconds, which a socket loop cannot afford.
// So the network thread never touches the device: it sets two wishes under a
// mutex - wantOpen, wantMotor - and the lidar thread makes the device match
// them between grabs, posting back what happened as messages (opened, failed,
// motor state, a frame, parked). A self-pipe wakes poll() when a message
// lands, so a frame reaches the sockets the moment it is formatted rather
// than at the next poll timeout.
//
// The cost is latency on a wish: while the motor is spinning up, the lidar
// thread sits in a 2 s grab and a MOTOR 0 waits for it. That is the correct
// trade - the alternative is two threads inside the SDK at once.
//
// ---------------------------------------------------------------------------
// A SLOW CLIENT IS DROPPED, NEVER WAITED FOR
//
// Every socket is non-blocking. A frame is appended to each client's pending
// buffer and pushed with send(); whatever the socket will not take stays
// pending and goes out on POLLOUT. A client whose oldest pending byte is more
// than BEHIND_MS old is closed. A phone on the far side of a hotspot can stall
// for seconds, and a blocking write to it would stall the revolution for every
// other viewer - and, worse, the loop that answers MOTOR 0.
//
// Frames are not queued per client beyond that half second: the picture is
// live or it is nothing, which is the same rule the hub's own lidar worker
// applies to a stale revolution.
//
// ---------------------------------------------------------------------------
// SIGNALS PARK THE DEVICE
//
// SIGINT and SIGTERM (systemctl stop) set a flag, the loop notices, the lidar
// thread is asked to quit and joined, and it stops the motor and closes the
// port on its way out. The handlers are installed before the socket is bound
// and long before anything can open the lidar, so there is no window in which
// a stop leaves the C1 spinning on the bench with nobody attached - the mess
// lidar_probe.cxx describes and this project has made once already.
//
// Exits 0 on a signal. Exits 1 only when it could not listen at all.

#include "shared.hxx"

#include "lidar.hxx"
#include "scanwire.hxx"

#include <csignal>
#include <cstdio>

#if defined(__linux__)

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{

  // lidar.hxx: the first grab after motorOn() times out at this figure, every
  // time, while the motor comes up to speed. Known, and not an error.
  constexpr Int32 GRAB_TIMEOUT_MS = 2000;

  // How far behind a client may fall before it is dropped: five revolutions.
  constexpr Int64 BEHIND_MS = 500;

  // The longest the loop sleeps with nothing to do. Bounds how long a signal
  // waits to be noticed if poll() happened not to be interrupted by it.
  constexpr Int32 POLL_MS = 250;

  // A request line longer than this is not a request; the client is dropped.
  constexpr Size REQUEST_MAX = 1024;

  // Written from the signal handler, read from the loop. volatile
  // sig_atomic_t is the one type the standard promises is safe in a handler.
  volatile std::sig_atomic_t interrupted = 0;

  Void onInterrupt(Int32)
  {
      interrupted = 1;
  }

  // What the lidar thread tells the network thread.
  enum class Note
  {
      NOTE_OPENED,   // text: the INFO and HEALTH lines, ready to send
      NOTE_FAILED,   // text: why; the feed tells every client and closes them
      NOTE_MOTOR,    // on: the motor's state after the request
      NOTE_FRAME,    // text: one F line
      NOTE_PARKED,   // motor off, port closed and free
  };

  struct Message
  {
      Note kind = Note::NOTE_PARKED;
      Str  text;
      Bool on = false;
  };

  // Everything the two threads share. Every member is touched under `m`
  // except wakeFd, which is set once before the thread starts.
  struct Shared
  {
      Mutex          m;
      CondVar        cv;
      Bool           wantOpen = false;
      Bool           wantMotor = false;
      Bool           quit = false;
      Deque<Message> notes;
      Int32          wakeFd = -1;
  };

  // ---- the lidar thread -----------------------------------------------------

  // A constructor in all but name. Not a designated initializer at each call
  // site: gcc 11's -Wextra reports every member a designated list leaves to
  // its default, which is the whole point of having defaults.
  [[nodiscard]] Message note(Note kind, Str text = Str(), Bool on = false)
  {
      Message m;
      m.kind = kind;
      m.text = std::move(text);
      m.on = on;
      return m;
  }

  Void post(Shared& sh, Message msg)
  {
      {
          LockGuard<Mutex> lock(sh.m);
          sh.notes.push_back(std::move(msg));
      }
      // One byte to wake poll().
      const Char one = 1;
      if(::write(sh.wakeFd, &one, 1) < 0)
      {
          // Deliberately nothing: a full pipe means thousands of unread
          // wakeups, so the loop is awake already. glibc marks write()
          // warn_unused_result, which a cast to Void does not satisfy.
      }
  }

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

  // Motor off, port closed. Safe with nothing open; says nothing then.
  Void park(Shared& sh)
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
      post(sh, note(Note::NOTE_PARKED));
  }

  // A failure the clients have to hear about. wantOpen is withdrawn HERE, on
  // this thread, so the wait below does not spin on a wish that cannot be
  // granted; the network thread withdraws it again when it closes the clients.
  Void fail(Shared& sh, const Str& why)
  {
      {
          LockGuard<Mutex> lock(sh.m);
          sh.wantOpen = false;
      }
      std::printf("refused: %s\n", why.c_str());
      post(sh, note(Note::NOTE_FAILED, why));
  }

  Void lidarThread(Shared& sh, const Str& port)
  {
      Vec<reactive::Ray> rays;
      Vec<UInt8>         quality;
      TimePoint          last;
      Bool               haveLast = false;

      for(;;)
      {
          Bool wantOpen = false;
          Bool wantMotor = false;
          {
              UniqueLock<Mutex> lock(sh.m);
              // Sleep only while the device already matches every wish and is
              // not spinning; a spinning device has revolutions to collect.
              sh.cv.wait(
                  lock,
                  [&]
                  {
                      return sh.quit || lidar::isSpinning() || sh.wantOpen != lidar::isOpen()
                          || (lidar::isOpen() && sh.wantMotor != lidar::isSpinning());
                  }
              );
              if(sh.quit)
              {
                  break;
              }
              wantOpen = sh.wantOpen;
              wantMotor = sh.wantMotor;
          }

          if(!wantOpen)
          {
              park(sh);
              haveLast = false;
              continue;
          }

          if(!lidar::isOpen())
          {
              if(!lidar::open(port))
              {
                  fail(sh, lidar::reason());
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
              post(sh, note(Note::NOTE_OPENED, std::move(hello)));
          }

          if(wantMotor != lidar::isSpinning())
          {
              if(wantMotor)
              {
                  if(!lidar::motorOn())
                  {
                      fail(sh, lidar::reason());
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
              post(sh, note(Note::NOTE_MOTOR, Str(), on));
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
              post(sh, note(Note::NOTE_FRAME, frameLine(rays, quality, hz)));
          }
      }

      park(sh);
  }

  // ---- the network thread ----------------------------------------------------

  struct Client
  {
      Int32     fd = -1;
      Str       peer;
      Str       inbuf;          // bytes read and not yet a whole line
      Str       pending;        // bytes owed and not yet taken by the socket
      TimePoint pendingSince;   // when `pending` last went from empty to not
      Str       dropWhy;        // non-empty: to be closed by reap()
  };

  [[nodiscard]] Int32 listenOn(UInt16 port)
  {
      const Int32 fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if(fd < 0)
      {
          std::printf("cannot create a socket: %s\n", std::strerror(errno));
          return -1;
      }
      // Without it a restart within a minute of a stop fails with "address in
      // use" while the old connections sit in TIME_WAIT - and systemd's
      // Restart=on-failure would then restart it into the same failure.
      const Int32 yes = 1;
      static_cast<Void>(::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)));

      sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
      addr.sin_port = htons(port);
      if(::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0)
      {
          std::printf(
              "cannot bind 0.0.0.0:%u: %s\n",
              static_cast<unsigned>(port),
              std::strerror(errno)
          );
          ::close(fd);
          return -1;
      }
      if(::listen(fd, 8) < 0)
      {
          std::printf("cannot listen: %s\n", std::strerror(errno));
          ::close(fd);
          return -1;
      }
      return fd;
  }

  // Pushes what the socket will take. false when the client is gone or has
  // fallen more than BEHIND_MS behind; dropWhy says which.
  [[nodiscard]] Bool flush(Client& c)
  {
      while(!c.pending.empty())
      {
          // MSG_NOSIGNAL: a peer that vanished must be an error here, not a
          // SIGPIPE that kills the feed for everyone else.
          const ISize n = ::send(
              c.fd,
              c.pending.data(),
              c.pending.size(),
              MSG_NOSIGNAL | MSG_DONTWAIT
          );
          if(n > 0)
          {
              c.pending.erase(0, static_cast<Size>(n));
              continue;
          }
          if(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
          {
              break;
          }
          c.dropWhy = n == 0 ? Str("closed") : Str(std::strerror(errno));
          return false;
      }
      if(!c.pending.empty() && elapsedMs(c.pendingSince) > static_cast<Float64>(BEHIND_MS))
      {
          Array<Char, 48> why{};
          std::snprintf(why.data(), why.size(), "%.0f ms behind", elapsedMs(c.pendingSince));
          c.dropWhy = why.data();
          return false;
      }
      return true;
  }

  Void queue(Client& c, const Str& bytes)
  {
      if(c.pending.empty())
      {
          c.pendingSince = monoNow();
      }
      c.pending += bytes;
      static_cast<Void>(flush(c));
  }

  Void broadcast(Vec<Client>& clients, const Str& bytes)
  {
      for(Client& c : clients)
      {
          if(c.dropWhy.empty())
          {
              queue(c, bytes);
          }
      }
  }

  Void dropAll(Vec<Client>& clients, const Str& why)
  {
      for(Client& c : clients)
      {
          if(c.dropWhy.empty())
          {
              c.dropWhy = why;
          }
      }
  }

  // Closes and forgets every client marked for dropping, saying so.
  Void reap(Vec<Client>& clients)
  {
      Size kept = 0;
      for(Size i = 0; i < clients.size(); ++i)
      {
          Client& c = clients[i];
          if(c.dropWhy.empty())
          {
              if(kept != i)
              {
                  clients[kept] = std::move(c);
              }
              ++kept;
              continue;
          }
          std::printf("client %s dropped: %s\n", c.peer.c_str(), c.dropWhy.c_str());
          ::close(c.fd);
      }
      clients.resize(kept);
  }

  // Tells the lidar thread what the clients currently want.
  Void wish(Shared& sh, Bool open, Bool motor)
  {
      {
          LockGuard<Mutex> lock(sh.m);
          sh.wantOpen = open;
          sh.wantMotor = motor;
      }
      sh.cv.notify_one();
  }

  // One request line from a client. Anything but MOTOR and QUIT is ignored,
  // so a hub that learns a new verb before the board does costs nothing here.
  Void request(Shared& sh, Client& c, StrView line)
  {
      scanwire::Line parsed;
      switch(scanwire::parse(line, &parsed))
      {
      case scanwire::Kind::KIND_MOTOR:
      {
          std::printf("client %s asks MOTOR %d\n", c.peer.c_str(), parsed.motor ? 1 : 0);
          LockGuard<Mutex> lock(sh.m);
          sh.wantMotor = parsed.motor;
          sh.cv.notify_one();
          break;
      }
      case scanwire::Kind::KIND_QUIT:
          c.dropWhy = "QUIT";
          break;
      default:
          break;
      }
  }

  Void readFrom(Shared& sh, Client& c)
  {
      Array<Char, 512> chunk{};
      const ISize n = ::recv(c.fd, chunk.data(), chunk.size(), MSG_DONTWAIT);
      if(n == 0)
      {
          c.dropWhy = "left";
          return;
      }
      if(n < 0)
      {
          if(errno != EAGAIN && errno != EWOULDBLOCK)
          {
              c.dropWhy = std::strerror(errno);
          }
          return;
      }
      c.inbuf.append(chunk.data(), static_cast<Size>(n));
      Size nl = c.inbuf.find('\n');
      while(nl != Str::npos && c.dropWhy.empty())
      {
          request(sh, c, StrView(c.inbuf).substr(0, nl));
          c.inbuf.erase(0, nl + 1);
          nl = c.inbuf.find('\n');
      }
      if(c.inbuf.size() > REQUEST_MAX)
      {
          c.dropWhy = "not speaking the protocol";
      }
  }

  // Takes every connection waiting on the listening socket.
  Void acceptAll(Int32 listenFd, Vec<Client>& clients, const Str& greeting)
  {
      for(;;)
      {
          sockaddr_in peer{};
          socklen_t   len = sizeof(peer);
          const Int32 fd = ::accept4(listenFd, reinterpret_cast<sockaddr*>(&peer), &len, SOCK_NONBLOCK | SOCK_CLOEXEC);
          if(fd < 0)
          {
              return;
          }
          // Each F line is one write and should be one segment train, now,
          // not held back by Nagle waiting for the next one 100 ms later.
          const Int32 yes = 1;
          static_cast<Void>(::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)));

          Array<Char, INET_ADDRSTRLEN> ip{};
          static_cast<Void>(::inet_ntop(AF_INET, &peer.sin_addr, ip.data(), ip.size()));
          Array<Char, 64> name{};
          std::snprintf(
              name.data(),
              name.size(),
              "%s:%u",
              ip.data(),
              static_cast<unsigned>(ntohs(peer.sin_port))
          );

          Client c;
          c.fd = fd;
          c.peer = name.data();
          std::printf("client from %s\n", c.peer.c_str());
          // A late joiner is told what the first one was told, so it does not
          // have to infer the device from the frames.
          if(!greeting.empty())
          {
              queue(c, greeting);
          }
          clients.push_back(std::move(c));
      }
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

      Array<Int32, 2> wake{ -1, -1 };
      if(::pipe2(wake.data(), O_NONBLOCK | O_CLOEXEC) < 0)
      {
          std::printf("cannot create the wake pipe: %s\n", std::strerror(errno));
          return 1;
      }
      const Int32 listenFd = listenOn(scanwire::PORT);
      if(listenFd < 0)
      {
          return 1;
      }
      std::printf(
          "listening on 0.0.0.0:%u for %s\n",
          static_cast<unsigned>(scanwire::PORT),
          port.c_str()
      );

      Shared sh;
      sh.wakeFd = wake[1];
      Thread worker(lidarThread, std::ref(sh), port);

      Vec<Client> clients;
      Str greeting;    // INFO and HEALTH while open, then the latest MOTOR line
      Str motorLine;

      while(interrupted == 0)
      {
          Vec<pollfd> fds;
          fds.push_back(pollfd{ listenFd, POLLIN, 0 });
          fds.push_back(pollfd{ wake[0], POLLIN, 0 });
          for(const Client& c : clients)
          {
              // pollfd's events is a short; Int16 is that type on this ABI.
              fds.push_back(
                  pollfd{ c.fd, static_cast<Int16>(POLLIN | (c.pending.empty() ? 0 : POLLOUT)), 0 }
              );
          }

          if(::poll(fds.data(), fds.size(), POLL_MS) < 0 && errno != EINTR)
          {
              std::printf("poll failed: %s\n", std::strerror(errno));
              break;
          }

          const Size before = clients.size();
          if((fds[0].revents & POLLIN) != 0)
          {
              acceptAll(listenFd, clients, greeting + motorLine);
          }
          if((fds[1].revents & POLLIN) != 0)
          {
              Array<Char, 64> sink{};
              while(::read(wake[0], sink.data(), sink.size()) > 0)
              {
              }
          }

          // The lidar thread's news, taken all at once so the lock is held
          // for a swap and not for a send.
          Deque<Message> notes;
          {
              LockGuard<Mutex> lock(sh.m);
              notes.swap(sh.notes);
          }
          for(const Message& msg : notes)
          {
              switch(msg.kind)
              {
              case Note::NOTE_OPENED:
                  greeting = msg.text;
                  broadcast(clients, msg.text);
                  break;
              case Note::NOTE_FAILED:
                  // ERR then close, for everyone waiting on this open; the
                  // next client to arrive tries again from idle.
                  broadcast(clients, scanwire::formatErr(msg.text));
                  dropAll(clients, "ERR sent");
                  greeting.clear();
                  motorLine.clear();
                  break;
              case Note::NOTE_MOTOR:
                  motorLine = scanwire::formatMotor(msg.on);
                  broadcast(clients, motorLine);
                  break;
              case Note::NOTE_FRAME:
                  broadcast(clients, msg.text);
                  break;
              case Note::NOTE_PARKED:
                  greeting.clear();
                  motorLine.clear();
                  break;
              }
          }

          // Client sockets, in the order the pollfds were built. Only the
          // clients that existed before accept() have an entry.
          for(Size i = 0; i < before && i < clients.size(); ++i)
          {
              Client& c = clients[i];
              const Int16 ev = fds[2 + i].revents;
              if(!c.dropWhy.empty())
              {
                  continue;
              }
              if((ev & (POLLERR | POLLHUP | POLLNVAL)) != 0 && (ev & POLLIN) == 0)
              {
                  c.dropWhy = "hung up";
                  continue;
              }
              if((ev & POLLIN) != 0)
              {
                  readFrom(sh, c);
              }
              if(c.dropWhy.empty() && !c.pending.empty())
              {
                  static_cast<Void>(flush(c));
              }
          }

          reap(clients);

          // The wishes follow the client count: the first client asks for the
          // device and the motor, the last one leaving withdraws both. A
          // client arriving while open changes nothing - somebody may have
          // turned the motor off on purpose.
          if(before == 0 && !clients.empty())
          {
              wish(sh, true, true);
          }
          else if(before > 0 && clients.empty())
          {
              wish(sh, false, false);
          }
      }

      std::printf("stopping: parking the lidar\n");
      {
          LockGuard<Mutex> lock(sh.m);
          sh.quit = true;
      }
      sh.cv.notify_one();
      worker.join();

      for(Client& c : clients)
      {
          ::close(c.fd);
      }
      ::close(listenFd);
      ::close(wake[0]);
      ::close(wake[1]);
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
