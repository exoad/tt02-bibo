// See link.hxx. Two implementations in one file, chosen by the preprocessor.
//
// Linux gets POSIX termios - the Orange Pi is Linux, and so is any bench box a
// Pico might be plugged into on the way there. Everything else gets the
// refusing path: every entry point answers honestly that there is no transport
// here, and the one function that would otherwise have to lie, open(), says
// RESULT_NO_PLATFORM. The counters and the sentences are shared, so the
// invariant `sends == tx + dropped` is the same fact on both.
//
// One file rather than link_posix.cxx beside a link_stub.cxx because both
// build scripts - the CMake for g++ and tests/build_pilot_test.bat for MSVC -
// name link.cxx, and a second file is a second place for them to disagree.

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

    // ---- shared by both implementations ------------------------------------

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

  // ---------------------------------------------------------------------------
  // THE TERMIOS IMPLEMENTATION
  //
  // One descriptor, one reader thread, three mutexes with three jobs:
  //
  //   ctlMu   open() and close() run one at a time. Held for the whole call.
  //   txMu    one writer at a time, and - the part that matters - the
  //           descriptor cannot be closed under a write in progress, because
  //           close() takes this before it closes anything.
  //   rxMu    the line queue, the partial line and the last-heard time. Shared
  //           between the reader thread and drain()/silentForMs().
  //
  // The reader never takes ctlMu, and nothing takes ctlMu while holding either
  // of the others, so there is no order to get wrong.

  namespace
  {

    // How long the reader sleeps in poll() before checking whether it has been
    // asked to stop. Bounds close()'s latency, and nothing else - a line that
    // arrives wakes it immediately.
    constexpr Int32 POLL_MS = 50;

    // How long send() will wait for the device to take the bytes. A USB CDC
    // port whose board has stopped reading fills its buffer and then blocks
    // writers forever; the hub's serial thread stalled that way once and the
    // stall was counted as neither sent nor dropped. Here it is a drop.
    constexpr Int32 WRITE_WAIT_MS = 100;

    // The longest line the reader will hold while waiting for its newline. The
    // protocol's lines are tens of bytes; something that runs to this many
    // without a newline is a board speaking binary or a wrong baud, and it is
    // discarded up to the next newline rather than delivered as a 4 KB line.
    constexpr Size LINE_CAP = 4096;

    // Lines held for drain(). Oldest dropped when full: a caller that stopped
    // draining wants the recent state of the car, not the history.
    constexpr Size QUEUE_CAP = 4096;

    struct Rate
    {
        Int32 baud;
        speed_t code;
    };

    // termios wants its own name for a rate, not the number. Only rates the
    // project has a use for; the C1 is 460800 and the Pico is 115200.
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

    // Under rxMu.
    Vec<Str> queue;
    Str partial;
    Bool discarding = false;
    TimePoint lastRx;

    // Under txMu. True while the head of a line this side gave up on may be
    // sitting in the device's buffer without its newline. A send() that wrote
    // part of its message and then stalled cannot take those bytes back, and
    // the board's console goes on collecting until a newline arrives - so the
    // next send() ends that line before it starts its own. See send().
    //
    // NOT cleared by teardown(): the fragment is in the board's buffer, not
    // the descriptor's, and closing the port does not un-send it. The cost of
    // being wrong the other way is one empty line, which the board ignores.
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

    // The board is gone, or as good as. Recorded; the descriptor is left for
    // close() to release, because closing it from the reader while a writer
    // might hold it is the race txMu exists to prevent.
    Void markLost(const Str& what)
    {
        note(what);
        lost.store(true);
    }

    // Bytes off the wire become lines. '\r' is dropped wherever it appears -
    // the board ends lines "\r\n" on some paths and "\n" on others, and a line
    // with a stray '\r' on its tail fails every token comparison downstream
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
                // board speaking - which is what silentForMs() is measuring.
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
                    // End of file on a tty: the far end hung up. On a USB CDC
                    // port that is the cable coming out or the board rebooting
                    // into BOOTSEL - not a fault, the commonest way a link ends.
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

    // Caller holds ctlMu. Stops the reader, then closes, in that order: the
    // reader uses the descriptor without a lock, on the promise that it is
    // joined before the descriptor goes.
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
          // The board went away under the last link. Release it and try
          // again, so a caller's recovery is "call open()" and nothing else.
          teardown();
      }

      speed_t code = 0;
      if(!rateFor(cfg.baud, &code))
      {
          note("baud " + std::to_string(cfg.baud) + " is not a rate termios has a name for");
          return Result::RESULT_OPEN_FAILED;
      }

      // ::open, because inside this namespace the bare name is carlink::open.
      // O_NONBLOCK so a write to a full device returns EAGAIN and send() can
      // keep its deadline; O_NOCTTY so a modem-control line on the port cannot
      // make it this process's controlling terminal and send it SIGHUP.
      const Int32 f = ::open(cfg.where.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if(f < 0)
      {
          const Int32 err = errno;
          note(errnoText("open(" + cfg.where + ")", err));
          switch(err)
          {
          // The node is missing, or is there with nothing behind it. Both
          // mean "plug the cable in", so both are NO_PORT.
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
          // ENOTTY: it opened, and it is not a terminal. /dev/null does this,
          // and so does a path that names a regular file by mistake.
          note(errnoText("tcgetattr(" + cfg.where + ")", errno));
          ::close(f);
          return Result::RESULT_OPEN_FAILED;
      }

      // Raw: no line editing, no echo, no signal characters, no CR/LF
      // translation in either direction - the line discipline would otherwise
      // eat the very bytes the protocol is made of. 8N1, no flow control:
      // there is nothing on the other end to assert RTS.
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

      // Whatever the board said before anyone was listening is thrown away,
      // so the first line drain() delivers is a whole one and not the tail
      // of something that started before the link did.
      ::tcflush(f, TCIFLUSH);

      // Exclusive. A second opener gets EBUSY instead of half the bytes. Not
      // fatal if refused - the link works without it, it is just not alone.
      static_cast<Void>(::ioctl(f, TIOCEXCL));

      {
          LockGuard<Mutex> rx(rxMu);
          queue.clear();
          partial.clear();
          discarding = false;
          // Silence is counted from here. See silentForMs() in the header.
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

  Result send(const Str& line)
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

      // Added if absent rather than unconditionally: a caller that already
      // terminated its line should not produce a blank one behind it.
      Str msg = line;
      if(msg.empty() || msg.back() != '\n')
      {
          msg += '\n';
      }

      // The last send() left the head of its line on the wire. Written
      // straight after it, this line would be appended to that head and the
      // board would reject the pair as one unknown command - "ESC 15STOP",
      // with the STOP inside it. One newline first ends the fragment, so the
      // board rejects the fragment alone and this line arrives whole. The
      // board drops an empty line without a reply, so nothing is heard of it.
      if(dirty)
      {
          msg.insert(msg.begin(), '\n');
      }

      // The whole line or nothing counted. A partial write followed by a
      // failure is a drop, not a send - but the bytes that did go cannot be
      // recalled, so the line is left marked dirty and the next send() ends
      // it before its own. Every failed exit comes out through `verdict` so
      // that mark and the count are settled in one place below.
      const TimePoint start = monoNow();
      Result verdict = Result::RESULT_OK;
      Size done = 0;
      while(done < msg.size())
      {
          const Int32 left = WRITE_WAIT_MS - static_cast<Int32>(elapsedMs(start));
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
          // Only if something went: a drop that wrote nothing left the wire
          // as it found it, and `dirty` keeps whatever it already said.
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
      // `out` is left ALONE rather than cleared. A caller that gathers from
      // several sources into one vector should not have its earlier lines
      // deleted by a transport that had nothing to add.
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
      // Lines first, then the verdict: what the board said on its way out is
      // delivered with the news that it went.
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

  // ---------------------------------------------------------------------------
  // THE REFUSING IMPLEMENTATION
  //
  // Not a stub that pretends. Every call says which absence it is.

  namespace
  {

    // Kept even though nothing can set it true here, because the shape of the
    // state is part of what this file is settling. isOpen() reading a real
    // variable rather than `return false` is what lets the test assert the
    // sequence - open, fail, still closed - instead of a constant.
    Bool opened = false;

  }

  Result open(const Config& cfg)
  {
      static_cast<Void>(cfg);

      // NOT RESULT_NO_PORT. That would read as "plug the cable in" and send
      // somebody looking at the car, when the truth is that the code to talk to
      // it has not been written for this platform. Naming the right absence is
      // the whole job of an error value.
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

  Result send(const Str& line)
  {
      static_cast<Void>(line);
      // Counted, like every other line send() turns away. A stop that went
      // nowhere has to show up in a number somewhere, on every platform.
      ++dropCount;
      return opened ? Result::RESULT_NO_PLATFORM : Result::RESULT_NOT_OPEN;
  }

  Result drain(Vec<Str>& out)
  {
      // `out` is left ALONE rather than cleared. A caller that gathers from
      // several sources into one vector should not have its earlier lines
      // deleted by a transport that had nothing to add.
      static_cast<Void>(out);
      return opened ? Result::RESULT_NO_PLATFORM : Result::RESULT_NOT_OPEN;
  }

  Int32 silentForMs()
  {
      return -1;
  }

#endif

}
