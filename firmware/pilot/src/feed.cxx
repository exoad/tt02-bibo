#include "feed.hxx"

#include "scanwire.hxx"

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

namespace feed
{

  namespace
  {

    // How far behind a client may fall before it is dropped: five revolutions.
    constexpr Int64 BEHIND_MS = 500;

    // The longest the loop sleeps with nothing to do. Bounds how long stop()
    // waits if the wake byte happened to be lost, and nothing else.
    constexpr Int32 POLL_MS = 250;

    // A request line longer than this is not a request; the client is dropped.
    constexpr Size REQUEST_MAX = 1024;

    // How long an upstream has to accept a relay's connect before the client
    // is told the owner's reason instead. Loopback answers in microseconds
    // either way; the bound exists for the day the host is not local.
    constexpr Int64 RELAY_CONNECT_MS = 2000;

    struct Client
    {
        Int32     fd = -1;
        Str       peer;
        Str       inbuf;          // bytes read and not yet a whole line
        Str       pending;        // bytes owed and not yet taken by the socket
        TimePoint pendingSince;   // when `pending` last went from empty to not
        Str       dropWhy;        // non-empty: to be closed by reap()
        Bool      askedMotor = false;   // MOTOR_REFUSE: the refusal was logged once

        // The relay, when this client has one. Bytes from the upstream go
        // straight into `pending`; bytes from the client wait in upPending
        // for the upstream to take them. While upConnecting the socket is
        // not yet a connection, and upSince says how long it has been trying.
        Int32     upFd = -1;
        Bool      upConnecting = false;
        TimePoint upSince;
        Str       upPending;
        Str       upName;     // host:port, for the log
        Str       upOrElse;   // the ERR the client hears if the upstream refuses

        // Where this client's sockets sit in the pollfd vector, for one pass
        // of the loop; upAt is 0 when it has no upstream entry.
        Size      at = 0;
        Size      upAt = 0;
    };

    // What the owner hands the thread.
    enum class What
    {
        WHAT_LINE,    // a line for every client the owner serves
        WHAT_FAIL,    // ERR for those, then the door
        WHAT_RELAY,   // every client not yet relayed gets an upstream
    };

    struct Item
    {
        What   what = What::WHAT_LINE;
        Str    line;       // LINE and FAIL: the line; RELAY: the ERR if refused
        Str    host;       // RELAY
        UInt16 port = 0;   // RELAY
    };

    // Everything the two threads share. Every member is touched under `m`
    // except wakeFd, which is set once before the thread starts, and `count`,
    // which is atomic so publish() can look at it without the lock.
    struct Shared
    {
        Mutex        m;
        Str          greeting;
        Deque<Item>  items;
        Bool         quit = false;
        Int32        wakeFd = -1;
        Atomic<Size> count{ 0 };
    };

    // One feed per program, the way there is one lidar: free functions and
    // file-local state, not a class. A second listener would be a second
    // design decision.
    Shared          sh;
    Policy          policy;   // set at start(); greeting lives in sh instead
    Int32           listenFd = -1;
    Array<Int32, 2> wake{ -1, -1 };
    UInt16          boundPort = 0;
    Bool            running = false;
    Thread          worker;

    Void wakeLoop()
    {
        // One byte to wake poll().
        const Char one = 1;
        if(::write(sh.wakeFd, &one, 1) < 0)
        {
            // Deliberately nothing: a full pipe means thousands of unread
            // wakeups, so the loop is awake already. glibc marks write()
            // warn_unused_result, which a cast to Void does not satisfy.
        }
    }

    Void post(Item item)
    {
        {
            LockGuard<Mutex> lock(sh.m);
            sh.items.push_back(std::move(item));
        }
        wakeLoop();
    }

    // Listens on 0.0.0.0:port, or returns -1 having printed why. `taken` is
    // set when the reason was another listener on the port - the one failure
    // start() may answer with Policy::fallbackPort. A socket that cannot be
    // made at all would fail the same way next door, so that is not retried.
    [[nodiscard]] Int32 listenOn(UInt16 port, Bool& taken)
    {
        taken = false;
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
            taken = errno == EADDRINUSE;
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

    // The owner's line, to every client the owner serves. A relayed client
    // hears its upstream and nothing else, so the two feeds cannot interleave
    // on one socket.
    Void broadcast(Vec<Client>& clients, const Str& bytes)
    {
        for(Client& c : clients)
        {
            if(c.dropWhy.empty() && c.upFd < 0)
            {
                queue(c, bytes);
            }
        }
    }

    Void dropAll(Vec<Client>& clients, const Str& why)
    {
        for(Client& c : clients)
        {
            if(c.dropWhy.empty() && c.upFd < 0)
            {
                c.dropWhy = why;
            }
        }
    }

    // ---- the relay ---------------------------------------------------------------

    Void closeUpstream(Client& c)
    {
        if(c.upFd >= 0)
        {
            ::close(c.upFd);
            c.upFd = -1;
        }
        c.upConnecting = false;
        c.upPending.clear();
    }

    // The upstream would not have this client: it hears what the owner would
    // have said without the relay, and goes. The one log line for a relay
    // that never started.
    Void relayFailed(Client& c, const Str& why)
    {
        std::printf(
            "relay to %s for client %s failed: %s\n",
            c.upName.c_str(),
            c.peer.c_str(),
            why.c_str()
        );
        closeUpstream(c);
        queue(c, scanwire::formatErr(c.upOrElse));
        c.dropWhy = "relay refused";
        if(policy.onRelay)
        {
            policy.onRelay(false, why);
        }
    }

    Void relayUp(Client& c)
    {
        c.upConnecting = false;
        std::printf("relay to %s for client %s started\n", c.upName.c_str(), c.peer.c_str());
        if(policy.onRelay)
        {
            policy.onRelay(true, c.upName);
        }
    }

    // Begins a non-blocking connect; serveUpstream() finishes it on POLLOUT.
    // Non-blocking so that an upstream that neither accepts nor refuses -
    // a host that is not there - costs the other clients nothing.
    Void startRelay(Client& c, const Item& ask)
    {
        c.upName = ask.host + ":" + std::to_string(static_cast<unsigned>(ask.port));
        c.upOrElse = ask.line;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(ask.port);
        if(::inet_pton(AF_INET, ask.host.c_str(), &addr.sin_addr) != 1)
        {
            relayFailed(c, "not an IPv4 address: " + ask.host);
            return;
        }
        c.upFd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if(c.upFd < 0)
        {
            relayFailed(c, std::strerror(errno));
            return;
        }
        // The client's MOTOR line should reach the upstream now, for the same
        // reason an F line should reach the client now.
        const Int32 yes = 1;
        static_cast<Void>(::setsockopt(c.upFd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)));

        c.upSince = monoNow();
        if(::connect(c.upFd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0)
        {
            relayUp(c);
            return;
        }
        if(errno == EINPROGRESS)
        {
            c.upConnecting = true;
            return;
        }
        relayFailed(c, std::strerror(errno));
    }

    // The upstream's bytes go to the client as they are. The relay does not
    // read the wire, so a line kind the upstream learns tomorrow passes
    // through today.
    Void readUpstream(Client& c)
    {
        Array<Char, 4096> chunk{};
        const ISize n = ::recv(c.upFd, chunk.data(), chunk.size(), MSG_DONTWAIT);
        if(n == 0)
        {
            c.dropWhy = "upstream closed";
            return;
        }
        if(n < 0)
        {
            if(errno != EAGAIN && errno != EWOULDBLOCK)
            {
                c.dropWhy = Str("upstream: ") + std::strerror(errno);
            }
            return;
        }
        queue(c, Str(chunk.data(), static_cast<Size>(n)));
    }

    // The client's bytes go up, whatever the socket will take now.
    Void flushUpstream(Client& c)
    {
        while(!c.upPending.empty())
        {
            const ISize n = ::send(
                c.upFd,
                c.upPending.data(),
                c.upPending.size(),
                MSG_NOSIGNAL | MSG_DONTWAIT
            );
            if(n > 0)
            {
                c.upPending.erase(0, static_cast<Size>(n));
                continue;
            }
            if(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                return;
            }
            c.dropWhy = n == 0 ? Str("upstream closed") : Str("upstream: ") + std::strerror(errno);
            return;
        }
    }

    // One pass of the loop for a client's upstream: finishes the connect, or
    // moves bytes both ways. `ev` is what poll() said about the upstream fd.
    Void serveUpstream(Client& c, Int16 ev)
    {
        if(c.upConnecting)
        {
            if((ev & (POLLOUT | POLLERR | POLLHUP)) != 0)
            {
                // A non-blocking connect reports its verdict as SO_ERROR;
                // POLLOUT alone does not mean it succeeded.
                Int32     err = 0;
                socklen_t len = sizeof(err);
                if(::getsockopt(c.upFd, SOL_SOCKET, SO_ERROR, &err, &len) < 0)
                {
                    err = errno;
                }
                if(err != 0)
                {
                    relayFailed(c, std::strerror(err));
                    return;
                }
                relayUp(c);
                flushUpstream(c);
            }
            else if(elapsedMs(c.upSince) > static_cast<Float64>(RELAY_CONNECT_MS))
            {
                Array<Char, 48> why{};
                std::snprintf(
                    why.data(),
                    why.size(),
                    "no answer within %lld ms",
                    static_cast<long long>(RELAY_CONNECT_MS)
                );
                relayFailed(c, why.data());
            }
            return;
        }
        if((ev & POLLIN) != 0)
        {
            readUpstream(c);
        }
        else if((ev & (POLLERR | POLLHUP | POLLNVAL)) != 0)
        {
            c.dropWhy = "upstream closed";
        }
        if(c.dropWhy.empty() && (ev & POLLOUT) != 0 && !c.upPending.empty())
        {
            flushUpstream(c);
        }
    }

    // ---- the clients -------------------------------------------------------------

    // Closes and forgets every client marked for dropping, saying so - and
    // saying so for its relay first, so a relay's start and stop are a pair.
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
            if(c.upFd >= 0)
            {
                std::printf(
                    "relay to %s for client %s stopped: %s\n",
                    c.upName.c_str(),
                    c.peer.c_str(),
                    c.dropWhy.c_str()
                );
                closeUpstream(c);
            }
            std::printf("client %s dropped: %s\n", c.peer.c_str(), c.dropWhy.c_str());
            ::close(c.fd);
        }
        clients.resize(kept);
    }

    // One request line from a client. Anything but MOTOR and QUIT is ignored,
    // so a hub that learns a new verb before the board does costs nothing here.
    Void request(Client& c, StrView line)
    {
        scanwire::Line parsed;
        switch(scanwire::parse(line, &parsed))
        {
        case scanwire::Kind::KIND_MOTOR:
            if(policy.motor == Motor::MOTOR_OBEY)
            {
                std::printf("client %s asks MOTOR %d\n", c.peer.c_str(), parsed.motor ? 1 : 0);
                if(policy.onMotor)
                {
                    policy.onMotor(parsed.motor);
                }
                break;
            }
            // Refused: the asker alone hears the state it may not change.
            // The log line is per client, because a viewer that retries
            // every second would otherwise write the pilot's log for it.
            if(!c.askedMotor)
            {
                c.askedMotor = true;
                std::printf(
                    "client %s asks MOTOR %d: %s\n",
                    c.peer.c_str(),
                    parsed.motor ? 1 : 0,
                    policy.refusal.empty() ? "refused" : policy.refusal.c_str()
                );
            }
            queue(c, scanwire::formatMotor(policy.motorOn));
            break;
        case scanwire::Kind::KIND_QUIT:
            c.dropWhy = "QUIT";
            break;
        default:
            break;
        }
    }

    Void readFrom(Client& c)
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

        // Relayed: the upstream reads the wire, this one only carries it -
        // the upstream is the one refusing or obeying MOTOR, and a QUIT is
        // answered by the upstream closing, which closes this client too.
        if(c.upFd >= 0)
        {
            c.upPending += c.inbuf;
            c.inbuf.clear();
            if(!c.upConnecting)
            {
                flushUpstream(c);
            }
            return;
        }

        Size nl = c.inbuf.find('\n');
        while(nl != Str::npos && c.dropWhy.empty())
        {
            request(c, StrView(c.inbuf).substr(0, nl));
            c.inbuf.erase(0, nl + 1);
            nl = c.inbuf.find('\n');
        }
        if(c.inbuf.size() > REQUEST_MAX)
        {
            c.dropWhy = "not speaking the protocol";
        }
    }

    // Takes every connection waiting on the listening socket.
    Void acceptAll(Vec<Client>& clients)
    {
        Str greeting;
        {
            LockGuard<Mutex> lock(sh.m);
            greeting = sh.greeting;
        }
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

    // ---- the thread ------------------------------------------------------------

    Void loop()
    {
        Vec<Client> clients;
        Size        announced = 0;   // the count onClients last heard

        for(;;)
        {
            Vec<pollfd> fds;
            fds.push_back(pollfd{ listenFd, POLLIN, 0 });
            fds.push_back(pollfd{ wake[0], POLLIN, 0 });
            for(Client& c : clients)
            {
                // pollfd's events is a short; Int16 is that type on this ABI.
                c.at = fds.size();
                fds.push_back(
                    pollfd{ c.fd, static_cast<Int16>(POLLIN | (c.pending.empty() ? 0 : POLLOUT)), 0 }
                );
                c.upAt = 0;
                if(c.upFd >= 0)
                {
                    // A connect in progress reports as writable when it is
                    // decided; a connection wants POLLOUT only when it owes.
                    const Bool wantOut = c.upConnecting || !c.upPending.empty();
                    c.upAt = fds.size();
                    fds.push_back(
                        pollfd{ c.upFd, static_cast<Int16>(POLLIN | (wantOut ? POLLOUT : 0)), 0 }
                    );
                }
            }

            if(::poll(fds.data(), fds.size(), POLL_MS) < 0 && errno != EINTR)
            {
                std::printf("poll failed: %s\n", std::strerror(errno));
                break;
            }

            const Size before = clients.size();
            if((fds[0].revents & POLLIN) != 0)
            {
                acceptAll(clients);
            }
            if((fds[1].revents & POLLIN) != 0)
            {
                Array<Char, 64> sink{};
                while(::read(wake[0], sink.data(), sink.size()) > 0)
                {
                }
            }

            // The owner's lines, taken all at once so the lock is held for a
            // swap and not for a send.
            Deque<Item> items;
            Bool        quit = false;
            {
                LockGuard<Mutex> lock(sh.m);
                items.swap(sh.items);
                quit = sh.quit;
            }
            if(quit)
            {
                break;
            }
            for(const Item& item : items)
            {
                switch(item.what)
                {
                case What::WHAT_LINE:
                    broadcast(clients, item.line);
                    break;
                case What::WHAT_FAIL:
                    // ERR then close, for everyone the owner serves: the next
                    // client to arrive starts again from the greeting.
                    broadcast(clients, item.line);
                    dropAll(clients, "ERR sent");
                    break;
                case What::WHAT_RELAY:
                    // Including the clients accepted a moment ago, above: the
                    // owner asked because of a client it was told about, and
                    // that client is in this vector by now.
                    for(Client& c : clients)
                    {
                        if(c.dropWhy.empty() && c.upFd < 0)
                        {
                            startRelay(c, item);
                        }
                    }
                    break;
                }
            }

            // Client sockets, in the order the pollfds were built. Only the
            // clients that existed before accept() have an entry.
            for(Size i = 0; i < before && i < clients.size(); ++i)
            {
                Client& c = clients[i];
                if(!c.dropWhy.empty())
                {
                    continue;
                }
                const Int16 ev = fds[c.at].revents;
                if((ev & (POLLERR | POLLHUP | POLLNVAL)) != 0 && (ev & POLLIN) == 0)
                {
                    c.dropWhy = "hung up";
                    continue;
                }
                if((ev & POLLIN) != 0)
                {
                    readFrom(c);
                }
                if(c.dropWhy.empty() && !c.pending.empty())
                {
                    static_cast<Void>(flush(c));
                }
                if(c.dropWhy.empty() && c.upAt != 0)
                {
                    serveUpstream(c, fds[c.upAt].revents);
                }
            }

            reap(clients);

            // The count is published before the owner hears of it, so an
            // onClients that turns round and publish()es reaches the client
            // that caused it.
            sh.count.store(clients.size());
            if(clients.size() != announced)
            {
                announced = clients.size();
                if(policy.onClients)
                {
                    policy.onClients(announced);
                }
            }
        }

        for(Client& c : clients)
        {
            closeUpstream(c);
            ::close(c.fd);
        }
        sh.count.store(0);
    }

  }

  Bool start(UInt16 port, const Policy& p)
  {
      if(running)
      {
          std::printf("feed already running on port %u\n", static_cast<unsigned>(boundPort));
          return false;
      }
      if(::pipe2(wake.data(), O_NONBLOCK | O_CLOEXEC) < 0)
      {
          std::printf("cannot create the wake pipe: %s\n", std::strerror(errno));
          return false;
      }
      Bool taken = false;
      listenFd = listenOn(port, taken);
      if(listenFd < 0 && taken && p.fallbackPort != 0)
      {
          listenFd = listenOn(p.fallbackPort, taken);
      }
      if(listenFd < 0)
      {
          ::close(wake[0]);
          ::close(wake[1]);
          wake = { -1, -1 };
          return false;
      }
      // Asked for 0, got something: the test needs to know what.
      sockaddr_in bound{};
      socklen_t   len = sizeof(bound);
      if(::getsockname(listenFd, reinterpret_cast<sockaddr*>(&bound), &len) == 0)
      {
          boundPort = ntohs(bound.sin_port);
      }
      else
      {
          boundPort = port;
      }

      policy = p;
      {
          LockGuard<Mutex> lock(sh.m);
          sh.greeting = p.greeting;
          sh.items.clear();
          sh.quit = false;
          sh.wakeFd = wake[1];
      }
      sh.count.store(0);
      running = true;
      worker = Thread(loop);
      if(port != 0 && boundPort != port)
      {
          std::printf(
              "listening on 0.0.0.0:%u - %u was taken\n",
              static_cast<unsigned>(boundPort),
              static_cast<unsigned>(port)
          );
      }
      else
      {
          std::printf("listening on 0.0.0.0:%u\n", static_cast<unsigned>(boundPort));
      }
      return true;
  }

  UInt16 port()
  {
      return running ? boundPort : static_cast<UInt16>(0);
  }

  Void relay(const Str& host, const UInt16 port, const Str& orElse)
  {
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_RELAY;
      item.host = host;
      item.port = port;
      item.line = orElse;
      post(std::move(item));
  }

  Void setGreeting(const Str& lines)
  {
      LockGuard<Mutex> lock(sh.m);
      sh.greeting = lines;
  }

  Void publish(Str line)
  {
      // Nobody to be live for: not even the lock. This is what keeps an
      // unwatched pilot's tick the price it was before the feed existed.
      if(!running || sh.count.load() == 0)
      {
          return;
      }
      Item item;
      item.line = std::move(line);
      post(std::move(item));
  }

  Void fail(const Str& why)
  {
      if(!running)
      {
          return;
      }
      Item item;
      item.what = What::WHAT_FAIL;
      item.line = scanwire::formatErr(why);
      post(std::move(item));
  }

  Size clients()
  {
      return sh.count.load();
  }

  Void stop()
  {
      if(!running)
      {
          return;
      }
      {
          LockGuard<Mutex> lock(sh.m);
          sh.quit = true;
      }
      wakeLoop();
      worker.join();
      running = false;
      ::close(listenFd);
      ::close(wake[0]);
      ::close(wake[1]);
      listenFd = -1;
      wake = { -1, -1 };
      boundPort = 0;
      policy = Policy();
  }

}

#else

// Not a stub that listens and reports an empty room: a module that says it
// cannot do the job here. The feed needs Linux sockets, and every call
// behaves as it would with no client connected - which is to say, does
// nothing, and says so from start().
namespace feed
{

  Bool start(UInt16 port, const Policy& p)
  {
      static_cast<Void>(p);
      std::printf(
          "feed: no TCP feed on this platform (port %u asked for)\n",
          static_cast<unsigned>(port)
      );
      return false;
  }

  UInt16 port()
  {
      return 0;
  }

  Void relay(const Str& host, UInt16 port, const Str& orElse)
  {
      static_cast<Void>(host);
      static_cast<Void>(port);
      static_cast<Void>(orElse);
  }

  Void setGreeting(const Str& lines)
  {
      static_cast<Void>(lines);
  }

  Void publish(Str line)
  {
      static_cast<Void>(line);
  }

  Void fail(const Str& why)
  {
      static_cast<Void>(why);
  }

  Size clients()
  {
      return 0;
  }

  Void stop()
  {
  }

}

#endif
