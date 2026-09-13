// POSIX termios on Linux and the refusing path everywhere else, chosen by the
// preprocessor. The counters and why() are shared by both. One file because the
// CMake build and tools\test.bat pilot both name link.cxx.
#include "link.hxx"

#if defined(__linux__)
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace carlink
{
  namespace
  {
    Atomic<UInt64> txCount{ 0 };
    Atomic<UInt64> rxCount{ 0 };
    Atomic<UInt64> dropCount{ 0 };

    Mutex detailMu;
    Str lastDetail;

    Void note(const Str& what)
    {
        LockGuard<Mutex> g(detailMu);
        lastDetail = what;
    }
  }

  CharSeq why(Result r)
  {
      switch(r)
      {
      case Result::RESULT_OK:           return "ok";
      case Result::RESULT_NO_PLATFORM:  return "no serial transport is built for this platform yet";
      case Result::RESULT_NO_PORT:      return "no such port - is the board plugged in?";
      case Result::RESULT_DENIED:       return "permission denied opening the port - is this user in dialout?";
      case Result::RESULT_BUSY:         return "another program holds the port";
      case Result::RESULT_OPEN_FAILED:  return "the port opened but could not be set up as a serial line";
      case Result::RESULT_NOT_OPEN:     return "the link is not open";
      case Result::RESULT_WRITE_FAILED: return "the write failed";
      case Result::RESULT_CLOSED:       return "the link closed - the board went away";
      default:                          return "?";
      }
  }

  Str detail()
  {
      LockGuard<Mutex> g(detailMu);
      return lastDetail;
  }

  UInt64 txLines()
  {
      return txCount.load();
  }

  UInt64 rxLines()
  {
      return rxCount.load();
  }

  UInt64 dropped()
  {
      return dropCount.load();
  }

#if defined(__linux__)

  // One descriptor, one reader thread, three mutexes:
  //   ctlMu   open() and close() one at a time, held for the whole call.
  //   txMu    one writer at a time; close() takes it before closing anything,
  //           so the descriptor never closes under a write.
  //   rxMu    the line queue, the partial line and the last-heard time, shared
  //           by the reader thread and drain()/silentForMs().
  // The reader never takes ctlMu, and nothing takes ctlMu while holding either
  // of the others, so there is no lock order to get wrong.
  namespace
  {
    // How long the reader's poll() waits between checks for a stop. Bounds
    // close()'s latency only: arriving bytes wake it at once.
    constexpr Int32 POLL_MS = 50;

    // The longest partial line held. The protocol's lines are tens of bytes;
    // a run this long without a newline (binary, or a wrong baud) is discarded
    // up to the next newline.
    constexpr Size LINE_CAP = 4096;

    // Lines held for drain(), the oldest dropped when full: a caller that stopped
    // draining wants the car's recent state, not its history.
    constexpr Size QUEUE_CAP = 4096;

    struct Rate
    {
        Int32 baud;
        speed_t code;
    };

    // termios wants its own name for a rate. Only the rates this project uses.
    constexpr Array<Rate, 8> RATES = { {
        { 9600, B9600 },
        { 19200, B19200 },
        { 38400, B38400 },
        { 57600, B57600 },
        { 115200, B115200 },
        { 230400, B230400 },
        { 460800, B460800 },
        { 921600, B921600 },
    } };

    Mutex ctlMu;
    Mutex txMu;
    Mutex rxMu;

    // Atomic so isOpen() and send() can read them without a lock. Written only
    // under ctlMu (fd, running) or by the reader (lost).
    Atomic<Int32> fd{ -1 };
    Atomic<Bool> running{ false };
    Atomic<Bool> lost{ false };
    Thread reader;

    // stopFromSignal() reads fd inside a signal handler, which only a lock-free
    // load may do.
    static_assert(Atomic<Int32>::is_always_lock_free, "stopFromSignal needs a lock-free fd");

    constexpr StrView SIGNAL_STOP = "\nSTOP\n";

    // Under rxMu.
    Vec<Str> queue;
    Str partial;
    Bool discarding = false;
    TimePoint lastRx;

    // Under txMu. True while the head of a line a send() gave up on may sit in
    // the board's buffer without its newline; the next send() ends it first.
    // NOT cleared by teardown(): closing the port does not un-send the fragment,
    // and a wrong guess costs one empty line, which the board ignores.
    Bool dirty = false;

    [[nodiscard]] Str errnoText(const Str& op, const Int32 err)
    {
        return op + ": " + std::strerror(err);
    }

    [[nodiscard]] Bool rateFor(const Int32 baud, speed_t* code)
    {
        for(const Rate& r : RATES)
        {
            if(r.baud == baud)
            {
                *code = r.code;
                return true;
            }
        }
        return false;
    }

    // The descriptor is left for close(): closing it from the reader while a
    // writer may hold it is the race txMu prevents.
    Void markLost(const Str& what)
    {
        note(what);
        lost.store(true);
    }

    // '\r' is dropped wherever it appears: the board ends lines "\r\n" on some
    // paths and "\n" on others, and a trailing '\r' fails every token comparison
    // while looking identical in a log.
    Void feed(const Char* bytes, const Size n)
    {
        LockGuard<Mutex> g(rxMu);
        for(Size i = 0; i < n; ++i)
        {
            const Char c = bytes[i];
            if(c == '\r')
            {
                continue;
            }
            if(c == '\n')
            {
                // Any complete line, even an empty or discarded one, is the
                // board speaking, for silentForMs().
                lastRx = monoNow();
                if(!discarding && !partial.empty())
                {
                    if(queue.size() >= QUEUE_CAP)
                    {
                        queue.erase(queue.begin());
                    }
                    queue.push_back(std::move(partial));
                    ++rxCount;
                }
                partial.clear();
                discarding = false;
                continue;
            }
            if(discarding)
            {
                continue;
            }
            if(partial.size() >= LINE_CAP)
            {
                partial.clear();
                discarding = true;
                continue;
            }
            partial.push_back(c);
        }
    }

    Void readLoop(const Int32 f)
    {
        Array<Char, 512> buf;
        while(running.load())
        {
            pollfd p{};
            p.fd = f;
            p.events = POLLIN;
            const Int32 rc = ::poll(&p, 1, POLL_MS);
            if(rc < 0)
            {
                if(errno == EINTR)
                {
                    continue;
                }
                markLost(errnoText("poll", errno));
                return;
            }
            if(rc == 0)
            {
                continue;
            }
            if(p.revents & POLLIN)
            {
                const ISize n = ::read(f, buf.data(), buf.size());
                if(n > 0)
                {
                    feed(buf.data(), static_cast<Size>(n));
                    continue;
                }
                if(n == 0)
                {
                    // End of file on a tty: the cable came out, or the board
                    // rebooted into BOOTSEL.
                    markLost("read: end of file - the device went away");
                    return;
                }
                if(errno == EAGAIN || errno == EINTR)
                {
                    continue;
                }
                markLost(errnoText("read", errno));
                return;
            }
            if(p.revents & (POLLHUP | POLLERR | POLLNVAL))
            {
                markLost("poll: hangup - the device went away");
                return;
            }
        }
    }

    // Caller holds ctlMu. The reader is joined before the descriptor closes,
    // because it uses the descriptor without a lock.
    Void teardown()
    {
        running.store(false);
        if(reader.joinable())
        {
            reader.join();
        }
        LockGuard<Mutex> tx(txMu);
        const Int32 f = fd.exchange(-1);
        if(f >= 0)
        {
            ::close(f);
        }
        lost.store(false);
        LockGuard<Mutex> rx(rxMu);
        queue.clear();
        partial.clear();
        discarding = false;
    }
  }

  Result open(const Config& cfg)
  {
      LockGuard<Mutex> ctl(ctlMu);
      if(fd.load() >= 0)
      {
          if(!lost.load())
          {
              return Result::RESULT_OK;
          }
          // The board went away under the last link: release it and reopen.
          teardown();
      }
      speed_t code = 0;
      if(!rateFor(cfg.baud, &code))
      {
          note("baud " + std::to_string(cfg.baud) + " is not a rate termios has a name for");
          return Result::RESULT_OPEN_FAILED;
      }
      // ::open, because the bare name here is carlink::open. O_NONBLOCK so a
      // write to a full device returns EAGAIN and send() keeps its deadline;
      // O_NOCTTY so the port cannot become this process's controlling terminal
      // and send it SIGHUP.
      const Int32 f = ::open(cfg.where.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if(f < 0)
      {
          const Int32 err = errno;
          note(errnoText("open(" + cfg.where + ")", err));
          switch(err)
          {
          // The node is missing, or has nothing behind it: plug the cable in.
          case ENOENT:
          case ENOTDIR:
          case ENXIO:
          case ENODEV:
              return Result::RESULT_NO_PORT;
          case EACCES:
          case EPERM:
              return Result::RESULT_DENIED;
          // Another process opened it with TIOCEXCL, as this one does below.
          case EBUSY:
              return Result::RESULT_BUSY;
          default:
              return Result::RESULT_OPEN_FAILED;
          }
      }
      termios tio{};
      if(::tcgetattr(f, &tio) != 0)
      {
          // ENOTTY: it opened and is not a terminal, like /dev/null or a
          // regular file.
          note(errnoText("tcgetattr(" + cfg.where + ")", errno));
          ::close(f);
          return Result::RESULT_OPEN_FAILED;
      }
      // Raw: no line editing, echo, signal characters or CR/LF translation,
      // which would eat the protocol's bytes. 8N1 with no flow control: nothing
      // on the other end asserts RTS.
      ::cfmakeraw(&tio);
      tio.c_cflag |= (CLOCAL | CREAD);
      tio.c_cflag &= ~(CSTOPB | CRTSCTS);
      tio.c_iflag &= ~(IXON | IXOFF | IXANY);
      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;
      ::cfsetispeed(&tio, code);
      ::cfsetospeed(&tio, code);
      if(::tcsetattr(f, TCSANOW, &tio) != 0)
      {
          note(errnoText("tcsetattr(" + cfg.where + ")", errno));
          ::close(f);
          return Result::RESULT_OPEN_FAILED;
      }
      // What the board said before anyone listened is dropped, so drain()'s
      // first line is a whole one.
      ::tcflush(f, TCIFLUSH);
      // A second opener gets EBUSY. Best effort: the link works without it.
      static_cast<Void>(::ioctl(f, TIOCEXCL));
      {
          LockGuard<Mutex> rx(rxMu);
          queue.clear();
          partial.clear();
          discarding = false;
          // Silence counts from here.
          lastRx = monoNow();
      }
      lost.store(false);
      fd.store(f);
      running.store(true);
      reader = Thread(readLoop, f);
      return Result::RESULT_OK;
  }

  Void close()
  {
      LockGuard<Mutex> ctl(ctlMu);
      teardown();
  }

  Bool isOpen()
  {
      return fd.load() >= 0 && !lost.load();
  }

  Void stopFromSignal()
  {
      // Not under txMu: the thread holding it may be the one this signal
      // interrupted. One attempt; a port too full for SIGNAL_STOP is left to
      // the Pico's watchdog.
      const Int32 f = fd.load();
      if(f < 0)
      {
          return;
      }
      const Int32 saved = errno;
      const ISize n = ::write(f, SIGNAL_STOP.data(), SIGNAL_STOP.size());
      static_cast<Void>(n);
      errno = saved;
  }

  Result send(const Str& line, Int32 waitMs)
  {
      LockGuard<Mutex> tx(txMu);
      const Int32 f = fd.load();
      if(f < 0)
      {
          ++dropCount;
          return Result::RESULT_NOT_OPEN;
      }
      if(lost.load())
      {
          ++dropCount;
          return Result::RESULT_CLOSED;
      }
      // Only if absent, so a caller's own newline makes no blank line.
      Str msg = line;
      if(msg.empty() || msg.back() != '\n')
      {
          msg += '\n';
      }
      // Without this newline the line joins the last fragment ("ESC 15STOP")
      // and the board rejects both. The board ignores an empty line silently.
      if(dirty)
      {
          msg.insert(msg.begin(), '\n');
      }
      // A partial write that then fails is a drop, and leaves the line dirty.
      // Every failure exits through verdict, so both are settled once, below.
      const TimePoint start = monoNow();
      Result verdict = Result::RESULT_OK;
      Size done = 0;
      while(done < msg.size())
      {
          const Int32 left = waitMs - static_cast<Int32>(elapsedMs(start));
          if(left <= 0)
          {
              note("write: stalled - nothing took the bytes within the deadline");
              verdict = Result::RESULT_WRITE_FAILED;
              break;
          }
          pollfd p{};
          p.fd = f;
          p.events = POLLOUT;
          const Int32 rc = ::poll(&p, 1, left);
          if(rc < 0)
          {
              if(errno == EINTR)
              {
                  continue;
              }
              note(errnoText("poll", errno));
              verdict = Result::RESULT_WRITE_FAILED;
              break;
          }
          if(rc == 0)
          {
              continue;
          }
          if(p.revents & (POLLERR | POLLHUP | POLLNVAL))
          {
              markLost("poll: hangup - the device went away");
              verdict = Result::RESULT_CLOSED;
              break;
          }
          const ISize n = ::write(f, msg.data() + done, msg.size() - done);
          if(n < 0)
          {
              const Int32 err = errno;
              if(err == EAGAIN || err == EINTR)
              {
                  continue;
              }
              if(err == EIO || err == ENXIO || err == ENODEV)
              {
                  markLost(errnoText("write", err));
                  verdict = Result::RESULT_CLOSED;
                  break;
              }
              note(errnoText("write", err));
              verdict = Result::RESULT_WRITE_FAILED;
              break;
          }
          done += static_cast<Size>(n);
      }
      if(verdict != Result::RESULT_OK)
      {
          ++dropCount;
          // A drop that wrote nothing leaves dirty as it was.
          if(done > 0)
          {
              dirty = true;
          }
          return verdict;
      }
      dirty = false;
      ++txCount;
      return Result::RESULT_OK;
  }

  Result drain(Vec<Str>& out)
  {
      if(fd.load() < 0)
      {
          return Result::RESULT_NOT_OPEN;
      }
      {
          LockGuard<Mutex> rx(rxMu);
          for(Str& s : queue)
          {
              out.push_back(std::move(s));
          }
          queue.clear();
      }
      // Lines first, so what the board said on its way out arrives with
      // RESULT_CLOSED.
      return lost.load() ? Result::RESULT_CLOSED : Result::RESULT_OK;
  }

  Int32 silentForMs()
  {
      if(fd.load() < 0)
      {
          return -1;
      }
      LockGuard<Mutex> rx(rxMu);
      const Float64 ms = elapsedMs(lastRx);
      const Float64 cap = static_cast<Float64>(std::numeric_limits<Int32>::max());
      return ms >= cap ? std::numeric_limits<Int32>::max() : static_cast<Int32>(ms);
  }

#else

  namespace
  {
    // Nothing sets it true here. A variable rather than constants, so the
    // tests see the same open, fail, still-closed sequence as on Linux.
    Bool opened = false;
  }

  Result open(const Config& cfg)
  {
      static_cast<Void>(cfg);
      // NOT RESULT_NO_PORT, which would send someone to check the cable when
      // this build has no transport at all.
      return Result::RESULT_NO_PLATFORM;
  }

  Void close()
  {
      opened = false;
  }

  Bool isOpen()
  {
      return opened;
  }

  Void stopFromSignal()
  {
      // No port can be open here, so there is nothing to stop.
  }

  Result send(const Str& line, Int32 waitMs)
  {
      static_cast<Void>(line);
      static_cast<Void>(waitMs);
      // Counted as dropped here too.
      ++dropCount;
      return opened ? Result::RESULT_NO_PLATFORM : Result::RESULT_NOT_OPEN;
  }

  Result drain(Vec<Str>& out)
  {
      static_cast<Void>(out);
      return opened ? Result::RESULT_NO_PLATFORM : Result::RESULT_NOT_OPEN;
  }

  Int32 silentForMs()
  {
      return -1;
  }

#endif
}
