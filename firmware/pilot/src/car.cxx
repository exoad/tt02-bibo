// See car.hxx. The Car's threads, ports, hooks and printing; what it decides is
// carrules. Four threads touch the car: the program's, the lidar's (the only
// caller of lidar:: once open() is done), the minder (the only sender to the
// Pico until finish() has joined it) and the printer, which writes the Car's
// console lines so the minder never waits on the console.
//
// The signal hooks are Linux-only. Everywhere else carlink cannot open a port
// and lidar refuses, so a Car there refuses at lidar::open and nothing can move.

#include "car.hxx"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <initializer_list>

#include "carrules.hxx"
#include "lidar.hxx"
#include "link.hxx"
#include "proto.hxx"
#include "trimfile.hxx"
#include "viewfeed.hxx"

#if defined(__linux__)
#include <signal.h>
#endif

namespace
{

  using carrules::Arm;
  using carrules::End;

  constexpr Int32 NEUTRAL_US = bibowire::ESC_NEUTRAL_US;

  // The C1's baud, which LIDAR_INFO also reports.
  constexpr Int32 LIDAR_BAUD = 460800;

  // How long one grab waits. A revolution takes about 100 ms, so running out
  // means a late or missing one. It also bounds finish()'s wait for the thread.
  constexpr Int32 LIDAR_WAIT_MS = 200;

  // How long finish() waits after its own STOP for the Pico's answer.
  constexpr Int32 STOP_REPLY_MS = 50;

  constexpr Int32 STATUS_MS = 1000;

  // Console lines waiting for the printer. Past this they are counted and
  // dropped: a console that has stopped taking output must not grow memory.
  constexpr Size PRINT_QUEUE_MAX = 64;

  constexpr CharSeq PORT_HELD = "is bibo-pilot running? sudo systemctl stop bibo-pilot";
  constexpr CharSeq BUILD = __DATE__ " " __TIME__;

  // One Car at a time in a process.
  Atomic<Bool> carAlive{ false };

  // Set by the SIGINT, SIGTERM and SIGHUP handler; read by the minder.
  volatile std::sig_atomic_t signalled = 0;

#if defined(__linux__)

  using Handler = Void (*)(Int32);

  // Room for the handlers, which write one buffer and raise.
  constexpr Size ALT_STACK_BYTES = 65536;

  Bool hooksInstalled = false;
  std::terminate_handler previousTerminate = nullptr;

  // Int32 is int on Linux, so these are sigaction handlers.
  Void onStopSignal(Int32 sig)
  {
      static_cast<Void>(sig);
      carlink::stopFromSignal();
      signalled = 1;
  }

  // SA_RESETHAND has already put the default action back, so raising the same
  // signal ends the process the way the crash would have, after the STOP.
  Void onCrashSignal(Int32 sig)
  {
      carlink::stopFromSignal();
      static_cast<Void>(std::raise(sig));
  }

  [[noreturn]] Void onTerminate()
  {
      carlink::stopFromSignal();
      if(previousTerminate != nullptr)
      {
          previousTerminate();
      }
      std::abort();
  }

  Void onExit()
  {
      carlink::stopFromSignal();
  }

  Void hook(Int32 sig, Handler handler, Int32 flags)
  {
      struct sigaction sa{};
      sa.sa_handler = handler;
      sa.sa_flags = flags;
      sigemptyset(&sa.sa_mask);
      static_cast<Void>(::sigaction(sig, &sa, nullptr));
  }

  // A stack for this thread's signal handlers, so a stack overflow still reaches
  // onCrashSignal and sends STOP. A thread that already has one keeps it.
  class AltStack
  {
  public:
      AltStack()
      {
          stack_t now{};
          if(::sigaltstack(nullptr, &now) != 0 || (now.ss_flags & SS_DISABLE) == 0)
          {
              return;
          }
          bytes.resize(ALT_STACK_BYTES);
          stack_t ss{};
          ss.ss_sp = bytes.data();
          ss.ss_size = bytes.size();
          mine = ::sigaltstack(&ss, nullptr) == 0;
      }

      ~AltStack()
      {
          if(mine)
          {
              stack_t off{};
              off.ss_flags = SS_DISABLE;
              static_cast<Void>(::sigaltstack(&off, nullptr));
          }
      }

      AltStack(const AltStack&) = delete;
      AltStack& operator=(const AltStack&) = delete;

  private:
      Vec<Char> bytes;
      Bool mine = false;
  };

  // Gives the calling thread an AltStack, which lasts until the thread ends.
  Void useAltStack()
  {
      thread_local const AltStack stack;
      static_cast<Void>(stack);
  }

  // Once per process, before a device is opened.
  Void installHooks()
  {
      if(hooksInstalled)
      {
          return;
      }
      hooksInstalled = true;

      // A status line a second, even into a pipe.
      std::setvbuf(stdout, nullptr, _IOLBF, BUFSIZ);

      // One-shot, so a second Ctrl-C meets the default action and kills a
      // program that is stuck.
      for(const Int32 sig : { SIGINT, SIGTERM, SIGHUP })
      {
          hook(sig, onStopSignal, static_cast<Int32>(SA_RESETHAND | SA_RESTART));
      }
      for(const Int32 sig : { SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT })
      {
          hook(sig, onCrashSignal, static_cast<Int32>(SA_RESETHAND | SA_NODEFER | SA_ONSTACK));
      }
      // A dropped ssh session must not kill the program at its next printf,
      // before SIGHUP's handler has sent STOP.
      hook(SIGPIPE, SIG_IGN, 0);
      previousTerminate = std::set_terminate(onTerminate);
      std::atexit(onExit);
  }

#else

  // No port can open here, so no hook would have anything to stop.
  Void installHooks()
  {
  }

  Void useAltStack()
  {
  }

#endif

  [[nodiscard]] Str baseName(const Char* path)
  {
      const Str p = path != nullptr ? path : "";
      const Size slash = p.find_last_of("/\\");
      return slash == Str::npos ? p : p.substr(slash + 1);
  }

  // Thousandths of -1..1, as the wire carries them; NaN counts as 0.
  [[nodiscard]] Int16 milli(Float32 fraction)
  {
      const Float32 f = std::isnan(fraction) ? 0.0f : std::clamp(fraction, -1.0f, 1.0f);
      return static_cast<Int16>(std::lround(f * 1000.0f));
  }

  [[nodiscard]] UInt8 healthByte(Int32 health)
  {
      return health >= 0 && health <= 2 ? static_cast<UInt8>(health) : bibowire::HEALTH_ABSENT;
  }

  // One revolution as SCAN carries it: whole centi-degrees 0..35999, whole
  // millimetres with 0 for no return, and quality capped at the wire's 63.
  [[nodiscard]] bibowire::Scan wireScan(const Vec<reactive::Ray>& rays, const Vec<UInt8>& quality)
  {
      bibowire::Scan s;
      const Size n = std::min(rays.size(), bibowire::MAX_SCAN_POINTS);
      s.points.reserve(n);
      s.quality.reserve(n);
      for(Size i = 0; i < n; ++i)
      {
          const Float32 deg = std::fmod(std::fmod(rays[i].angleDeg, 360.0f) + 360.0f, 360.0f);
          const Float32 mm = rays[i].distMm;
          bibowire::ScanPoint p;
          p.angleCentiDeg = static_cast<UInt16>(static_cast<UInt32>(deg * 100.0f + 0.5f) % 36000u);
          p.distMm = static_cast<UInt16>(mm > 0.0f ? std::min(mm, 65535.0f) + 0.5f : 0.0f);
          s.points.push_back(p);
          const UInt8 q = i < quality.size() ? quality[i] : static_cast<UInt8>(0);
          s.quality.push_back(std::min(q, static_cast<UInt8>(63)));
      }
      return s;
  }

  // The device's identity as LIDAR_INFO carries it. A serial that is not 32
  // characters goes as zeros.
  [[nodiscard]] bibowire::LidarInfo wireLidar(const lidar::Device& d)
  {
      bibowire::LidarInfo i;
      i.model = static_cast<UInt16>(d.model);
      i.fwMajor = static_cast<UInt8>(d.fwMajor);
      i.fwMinor = static_cast<UInt8>(d.fwMinor);
      i.hwRev = static_cast<UInt16>(d.hwRev);
      i.baud = static_cast<UInt32>(LIDAR_BAUD);
      if(d.serial.size() == 2 * i.serial.size())
      {
          for(Size b = 0; b < i.serial.size(); ++b)
          {
              const Str hex = d.serial.substr(2 * b, 2);
              i.serial[b] = static_cast<UInt8>(std::strtoul(hex.c_str(), nullptr, 16));
          }
      }
      return i;
  }

  // One minder pass, as the viewer and the status line are told it.
  struct Pass
  {
      Int64 nowMs = 0;
      carrules::Board board;
      Arm arm = Arm::ARM_IDLE;
      End ended = End::END_NONE;
      Bool linkLost = false;
      Float32 throttle = 0.0f;
      Float32 steer = 0.0f;
      Float32 aheadM = 0.0f;
      Float32 nearestM = 0.0f;
      Float32 nearestRawDeg = 0.0f;
  };

}

namespace bibo
{

  struct Car::Inner
  {
      carrules::Options opt;
      carrules::Governor gov{ true, 0 };
      trimfile::Store trim;
      Str program = "car";
      TimePoint startedAt = monoNow();

      // Settled by open() before any thread starts, and by finish() after the
      // threads have joined.
      Bool owner = false;        // holds the one-Car slot
      Bool lidarOpen = false;
      Bool spinning = false;
      Bool picoOpen = false;
      Bool viewer = false;

      Thread minder;
      Thread lidarThread;
      Thread printer;

      // Under mu. wake is notified on every revolution, every pass and at the end.
      Mutex mu;
      CondVar wake;
      Bool quit = false;
      Bool minderStarted = false;
      Bool armed = false;        // arm() succeeded
      Bool saidEarlyDrive = false;
      Scan latest;
      UInt32 lastReturned = 0;
      UInt64 goodRevolutions = 0;
      Int64 scanMs = -1;
      Float32 throttle = 0.0f;
      Float32 steer = 0.0f;
      Int64 driveMs = -1;
      Float32 aheadM = 0.0f;
      Float32 nearestM = 0.0f;
      Float32 nearestRawDeg = 0.0f;

      // Written under mu, read anywhere. stopped is set only once STOP has gone
      // to the port, so ok() never reads false ahead of it.
      Atomic<Bool> stopped{ false };
      Atomic<Bool> pulsing{ false };     // the last pass sent a forward pulse
      Atomic<UInt64> revolutions{ 0 };
      Atomic<UInt64> timeouts{ 0 };
      Atomic<Int32> lidarHealth{ -1 };

      // The Car's console lines, waiting for the printer. Under printMu.
      Mutex printMu;
      CondVar printWake;
      Deque<Str> printQueue;
      UInt64 printDropped = 0;
      Bool printing = false;     // lines are queued rather than written here
      Bool printQuit = false;

      // The minder's; open()'s before the minder starts and finish()'s once it
      // has joined.
      carrules::Sender sender{
          [this]() { return nowMs(); },
          [this](const Str& line, Int32 waitMs) { return sendRaw(line, waitMs); }
      };
      Int64 heardMs = -1;
      Bool saidSendFail = false;
      Bool saidEnd = false;
      Str lastOrder;
      Int64 statusMs = 0;
      UInt64 statusRevolutions = 0;

      Bool finished = false;
      Int32 code = 1;

      [[nodiscard]] Int64 nowMs() const;
      [[nodiscard]] Bool heard(Int64 now) const;
      [[nodiscard]] UInt8 pilotMode() const;
      [[nodiscard]] bibowire::BoardState boardState(const Pass& p) const;

      Void open(Int32 argc, Char** argv);
      Void endRun(End why);
      Void say(Str line);
      Void sayErrors(const Vec<Str>& replies);
      Void printerLoop();
      Void stopPrinter();
      [[nodiscard]] carlink::Result sendRaw(const Str& line, Int32 waitMs);
      Void minderLoop();
      Void minderPass();
      Void printStatus(const Pass& p);
      Void lidarLoop();
      [[nodiscard]] Bool arm();
      [[nodiscard]] Scan nextScan();
      Void order(Float32 t, Float32 s);
      Int32 finish();
  };

  Int64 Car::Inner::nowMs() const
  {
      return static_cast<Int64>(elapsedMs(startedAt));
  }

  Bool Car::Inner::heard(Int64 now) const
  {
      return heardMs >= 0 && now - heardMs <= PICO_QUIET_MS;
  }

  UInt8 Car::Inner::pilotMode() const
  {
      const bibowire::PilotMode m = opt.drive
          ? bibowire::PilotMode::PILOT_MODE_DRIVE
          : bibowire::PilotMode::PILOT_MODE_LOOK;
      return static_cast<UInt8>(m);
  }

  bibowire::BoardState Car::Inner::boardState(const Pass& p) const
  {
      const Int32 silent = picoOpen ? carlink::silentForMs() : -1;
      bibowire::BoardState b;
      b.tMonoUs = static_cast<UInt64>(p.nowMs) * 1000u;
      b.upS = static_cast<UInt32>(p.nowMs / 1000);
      b.picoLink = static_cast<UInt8>(!picoOpen || p.linkLost ? 0 : (heard(p.nowMs) ? 1 : 2));
      b.picoArmed = static_cast<UInt8>(p.board.armed < 0 ? 2 : (p.board.armed != 0 ? 1 : 0));
      b.pilotMode = pilotMode();
      b.lidarHealth = healthByte(lidarHealth.load());
      b.lidarSpinning = static_cast<UInt8>(spinning ? 1 : 0);
      b.picoSilentMs = silent < 0 ? bibowire::PICO_SILENT_ABSENT : static_cast<UInt32>(silent);
      b.revolutions = static_cast<UInt32>(revolutions.load());
      b.timeouts = static_cast<UInt32>(timeouts.load());
      return b;
  }

  Void Car::Inner::open(Int32 argc, Char** argv)
  {
      if(carAlive.exchange(true))
      {
          std::printf("car: a program has one bibo::Car, and this is a second\n");
          endRun(End::END_OPEN_FAILED);
          return;
      }
      owner = true;

      Str why;
      if(!carrules::parseArgs(argc, argv, LIDAR_FORWARD_MEASURED, opt, why))
      {
          std::printf("car: %s\n", why.c_str());
          endRun(End::END_BAD_FLAGS);
          return;
      }
      if(opt.help)
      {
          std::printf("%s", carrules::usage().c_str());
          endRun(End::END_HELP);
          return;
      }
      gov = carrules::Governor(!opt.drive, opt.seconds);
      if(argc > 0)
      {
          program = baseName(argv[0]);
      }
      installHooks();
      // The thread that makes the Car runs the program's loop, so a stack
      // overflow there must still reach the crash handler.
      useAltStack();

      // The lidar before the Pico: a car that cannot see is never opened.
      if(!lidar::open(opt.lidarPort, LIDAR_BAUD))
      {
          std::printf("lidar %s: %s\n", opt.lidarPort.c_str(), lidar::reason().c_str());
          if(lidar::refusal() == lidar::Refusal::REFUSAL_HELD)
          {
              std::printf("lidar %s: %s\n", opt.lidarPort.c_str(), PORT_HELD);
          }
          endRun(End::END_OPEN_FAILED);
          return;
      }
      lidarOpen = true;
      if(!lidar::motorOn())
      {
          std::printf("lidar motor: %s\n", lidar::reason().c_str());
          endRun(End::END_OPEN_FAILED);
          return;
      }
      spinning = true;

      const Str trimPath = trimfile::defaultPath();
      if(!trimfile::load(trimPath, trim, why))
      {
          std::printf(
              "trim: cannot read %s: %s - the Pico keeps its compiled values\n",
              trimPath.c_str(),
              why.c_str()
          );
      }

      if(opt.viewer)
      {
          viewfeed::Policy policy;
          policy.boardName = "bibobox: " + program;
          policy.boardBuild = BUILD;
          policy.bootId = static_cast<UInt32>(WallClock::now().time_since_epoch().count());
          // b0 can drive, b1 has a lidar, b2 has a Pico.
          policy.capabilities = static_cast<UInt8>(opt.drive ? 0x07u : 0x02u);
          viewer = viewfeed::start(bibowire::PORT, policy);
          if(!viewer)
          {
              std::printf("viewer: not serving - no remote ESTOP this run\n");
          }
          else
          {
              // The mode first, so the viewer's trim and ARM are refused from its
              // first COMMAND.
              viewfeed::publishBoard(boardState(Pass()));
              viewfeed::publishLidarInfo(wireLidar(lidar::device()));
              viewfeed::publishTrim(trimfile::report(trim));
          }
      }

      if(!opt.drive)
      {
          std::printf("car: dry run - the Pico is never opened, nothing will move\n");
      }
      else
      {
          carlink::Config cfg;
          cfg.where = opt.picoPort;
          const carlink::Result r = carlink::open(cfg);
          if(r != carlink::Result::RESULT_OK)
          {
              std::printf(
                  "pico %s: %s - %s\n",
                  opt.picoPort.c_str(),
                  carlink::why(r),
                  carlink::detail().c_str()
              );
              if(r == carlink::Result::RESULT_BUSY)
              {
                  std::printf("pico %s: %s\n", opt.picoPort.c_str(), PORT_HELD);
              }
              endRun(End::END_OPEN_FAILED);
              return;
          }
          picoOpen = true;
          const Vec<Str> trimLines = trimfile::lines(trim);
          for(const Str& line : gov.open(trimLines))
          {
              static_cast<Void>(sender.send(line));
          }
          std::printf(
              "pico %s: open - STOP, PING and %u trim setting(s) sent\n",
              opt.picoPort.c_str(),
              static_cast<unsigned>(trimLines.size())
          );
      }

      {
          LockGuard<Mutex> lock(printMu);
          printing = true;
      }
      printer = Thread(&Inner::printerLoop, this);
      minder = Thread(&Inner::minderLoop, this);
      {
          LockGuard<Mutex> lock(mu);
          minderStarted = true;
      }
      lidarThread = Thread(&Inner::lidarLoop, this);
  }

  // From any thread but the minder's. Returns once STOP has gone: the minder's
  // next pass sends it, or, before the minder has started, this thread does.
  Void Car::Inner::endRun(End why)
  {
      UniqueLock<Mutex> lock(mu);
      gov.end(why);
      if(!minderStarted)
      {
          lock.unlock();
          if(picoOpen)
          {
              static_cast<Void>(sender.send(proto::stop()));
          }
          lock.lock();
          stopped = true;
      }
      wake.wait(lock, [this]() { return stopped.load(); });
      lock.unlock();
      wake.notify_all();
  }

  // Never waits on the console while the printer runs: a full queue drops the line.
  Void Car::Inner::say(Str line)
  {
      UniqueLock<Mutex> lock(printMu);
      if(!printing)
      {
          lock.unlock();
          std::fputs(line.c_str(), stdout);
          return;
      }
      if(printQueue.size() < PRINT_QUEUE_MAX)
      {
          printQueue.push_back(std::move(line));
      }
      else
      {
          ++printDropped;
      }
      lock.unlock();
      printWake.notify_one();
  }

  Void Car::Inner::sayErrors(const Vec<Str>& replies)
  {
      for(const Str& line : replies)
      {
          if(proto::read(line).kind == proto::Kind::KIND_ERR)
          {
              say("pico: " + line + "\n");
          }
      }
  }

  Void Car::Inner::printerLoop()
  {
      useAltStack();
      UniqueLock<Mutex> lock(printMu);
      for(;;)
      {
          printWake.wait(lock, [this]() { return printQuit || !printQueue.empty(); });
          if(printQueue.empty())
          {
              return;
          }
          Deque<Str> batch;
          batch.swap(printQueue);
          const UInt64 dropped = printDropped;
          printDropped = 0;
          lock.unlock();
          for(const Str& line : batch)
          {
              std::fputs(line.c_str(), stdout);
          }
          if(dropped > 0u)
          {
              std::printf(
                  "car: %llu console line(s) dropped - the console fell behind\n",
                  static_cast<unsigned long long>(dropped)
              );
          }
          lock.lock();
      }
  }

  // After the minder has joined. What the printer did not take is written here.
  Void Car::Inner::stopPrinter()
  {
      {
          LockGuard<Mutex> lock(printMu);
          printQuit = true;
      }
      printWake.notify_all();
      if(printer.joinable())
      {
          printer.join();
      }
      Deque<Str> left;
      {
          LockGuard<Mutex> lock(printMu);
          printing = false;
          left.swap(printQueue);
      }
      for(const Str& line : left)
      {
          std::fputs(line.c_str(), stdout);
      }
  }

  carlink::Result Car::Inner::sendRaw(const Str& line, Int32 waitMs)
  {
      const carlink::Result r = carlink::send(line, waitMs);
      if(r != carlink::Result::RESULT_OK && !saidSendFail)
      {
          saidSendFail = true;
          say(Str("pico: ") + carlink::why(r) + " - " + carlink::detail() + "\n");
      }
      return r;
  }

  Void Car::Inner::minderLoop()
  {
      useAltStack();
      Int64 due = nowMs();
      Bool quitting = false;
      while(!quitting)
      {
          minderPass();
          // A late pass moves the next one, rather than two going out together.
          due = std::max(due + MINDER_MS, nowMs());
          UniqueLock<Mutex> lock(mu);
          quitting = wake.wait_until(lock, startedAt + Millis(due), [this]() { return quit; });
      }
      // finish() ended the run before asking to quit, so this last pass sends STOP.
      minderPass();
  }

  Void Car::Inner::minderPass()
  {
      carrules::Inputs in;
      Pass p;
      if(picoOpen)
      {
          const carlink::Result r = carlink::drain(in.replies);
          p.linkLost = r == carlink::Result::RESULT_CLOSED || r == carlink::Result::RESULT_NOT_OPEN;
      }
      if(!in.replies.empty())
      {
          heardMs = nowMs();
      }
      in.linkLost = p.linkLost;
      in.signal = signalled != 0;
      const viewfeed::Drive vd = viewfeed::drive();
      in.estop = vd.estopLatched;
      in.viewerStuck = viewer && vd.seenAgeMs > VIEWER_STUCK_MS;
      in.sentMs = sender.sentMs();
      in.gapMs = sender.takeGapMs();

      Vec<Str> lines;
      {
          LockGuard<Mutex> lock(mu);
          // The pass is timed here, after everything it has waited on, so the
          // send gap is judged at the moment the lines are chosen.
          p.nowMs = nowMs();
          in.nowMs = p.nowMs;
          in.scanMs = scanMs;
          in.throttle = throttle;
          in.steer = steer;
          in.driveMs = driveMs;
          lines = gov.pass(in);
          p.board = gov.board();
          p.arm = gov.armState();
          p.ended = gov.ended();
          p.throttle = throttle;
          p.steer = steer;
          p.aheadM = aheadM;
          p.nearestM = nearestM;
          p.nearestRawDeg = nearestRawDeg;
      }
      wake.notify_all();

      // The throttle's line is always the last of a pass. One that cannot reach
      // the port in time is replaced by STOP, which ends the run here.
      End late = End::END_NONE;
      const Vec<Str> sent = sender.pass(lines, late);
      if(late != End::END_NONE)
      {
          LockGuard<Mutex> lock(mu);
          gov.end(late);
          p.ended = gov.ended();
      }
      if(!sent.empty())
      {
          lastOrder = sent.back();
      }
      pulsing = !sent.empty() && carrules::pulseIn(sent.back()) > NEUTRAL_US;

      sayErrors(in.replies);
      if(p.ended != End::END_NONE)
      {
          if(!saidEnd)
          {
              saidEnd = true;
              const CharSeq what = picoOpen ? "STOP" : "run ended";
              say(Str("car: ") + what + " - " + carrules::endName(p.ended) + "\n");
          }
          {
              LockGuard<Mutex> lock(mu);
              stopped = true;
          }
          wake.notify_all();
      }

      if(viewer)
      {
          const bibowire::BoardState b = boardState(p);
          viewfeed::publishBoard(b);
          viewfeed::Applied ap;
          ap.throttleMilli = static_cast<Int16>(pulsing.load() ? milli(p.throttle) : 0);
          ap.escUs = p.board.escUs < 0 ? bibowire::ESC_ABSENT : static_cast<UInt16>(p.board.escUs);
          ap.armed = static_cast<UInt8>(p.board.armed > 0 ? 1 : 0);
          ap.pilotMode = b.pilotMode;
          ap.picoSilentMs = b.picoSilentMs;
          viewfeed::applied(ap);
      }

      if(p.nowMs - statusMs >= STATUS_MS)
      {
          printStatus(p);
      }
  }

  Void Car::Inner::printStatus(const Pass& p)
  {
      const UInt64 revs = revolutions.load();
      const Float64 windowS = static_cast<Float64>(p.nowMs - statusMs) / 1000.0;
      const Float64 rate = static_cast<Float64>(revs - statusRevolutions) / windowS;
      statusRevolutions = revs;
      statusMs = p.nowMs;

      Array<Char, 128> seen{};
      std::snprintf(
          seen.data(),
          seen.size(),
          "ahead %.2f m  nearest %.2f m @ raw %.0f deg  lidar %.1f/s  viewers %u",
          static_cast<Float64>(p.aheadM),
          static_cast<Float64>(p.nearestM),
          static_cast<Float64>(p.nearestRawDeg),
          rate,
          static_cast<unsigned>(viewfeed::clients())
      );
      const Float64 shownThrottle = milli(p.throttle) / 1000.0;
      const Float64 shownSteer = milli(p.steer) / 1000.0;

      Array<Char, 320> line{};
      if(!picoOpen)
      {
          std::snprintf(
              line.data(),
              line.size(),
              "car DRY  asked thr %.2f steer %+.2f  %s  (nothing sent)\n",
              shownThrottle,
              shownSteer,
              seen.data()
          );
          say(line.data());
          return;
      }

      CharSeq armWord = "not armed";
      if(p.ended != End::END_NONE)
      {
          armWord = "STOPPED";
      }
      else if(p.arm == Arm::ARM_CONFIRMED)
      {
          armWord = "armed";
      }
      else if(p.arm != Arm::ARM_IDLE)
      {
          armWord = "arming";
      }
      CharSeq picoWord = heard(p.nowMs) ? "pico ok" : "pico silent";
      if(p.linkLost)
      {
          picoWord = "pico lost";
      }
      std::snprintf(
          line.data(),
          line.size(),
          "car DRIVE %s  thr %.2f -> %s (pico esc %d)  steer %+.2f  %s  %s\n",
          armWord,
          shownThrottle,
          lastOrder.c_str(),
          p.board.escUs,
          shownSteer,
          seen.data(),
          picoWord
      );
      say(line.data());
  }

  Void Car::Inner::lidarLoop()
  {
      useAltStack();
      Vec<reactive::Ray> rays;
      Vec<UInt8> quality;
      UInt32 rev = 0;
      Int64 lastRevMs = -1;
      for(;;)
      {
          {
              LockGuard<Mutex> lock(mu);
              if(quit)
              {
                  return;
              }
          }
          const TimePoint began = monoNow();
          if(!lidar::grab(rays, LIDAR_WAIT_MS, &quality))
          {
              ++timeouts;
              // A grab that fails at once rather than by waiting must not spin.
              if(elapsedMs(began) < static_cast<Float64>(LIDAR_WAIT_MS) / 2.0)
              {
                  sleepMs(LIDAR_WAIT_MS);
              }
              continue;
          }
          const Int64 now = nowMs();
          revolutions = ++rev;
          lidarHealth = lidar::device().health;

          Scan s = carrules::toScan(rays, opt.forwardDeg, rev);
          const Bool good = !s.blind();
          const Float32 ahead = s.ahead();

          // The single nearest return and its raw angle: a box held straight
          // ahead reads LIDAR_FORWARD_DEG off the status line.
          Float32 closestMm = 0.0f;
          Float32 closestDeg = 0.0f;
          for(const reactive::Ray& r : rays)
          {
              if(r.distMm > 0.0f && (closestMm == 0.0f || r.distMm < closestMm))
              {
                  closestMm = r.distMm;
                  closestDeg = r.angleDeg;
              }
          }

          Float32 askedThrottle = 0.0f;
          Float32 askedSteer = 0.0f;
          {
              LockGuard<Mutex> lock(mu);
              if(good)
              {
                  scanMs = now;
                  ++goodRevolutions;
              }
              aheadM = ahead;
              nearestM = closestMm / 1000.0f;
              nearestRawDeg = closestDeg;
              latest = std::move(s);
              askedThrottle = throttle;
              askedSteer = steer;
          }
          wake.notify_all();

          if(viewer)
          {
              bibowire::Scan w = wireScan(rays, quality);
              const Int64 gap = lastRevMs >= 0 ? now - lastRevMs : 0;
              w.tMonoUs = static_cast<UInt64>(now) * 1000u;
              w.revIndex = rev;
              const Int64 milliHz = gap > 0 ? std::min<Int64>(1000000 / gap, 65535) : 0;
              w.freqMilliHz = static_cast<UInt16>(milliHz);
              w.health = healthByte(lidarHealth.load());
              w.motor = 1;
              viewfeed::publishScan(std::move(w));

              // What the program asked for, and what the Car made of it.
              const reactive::Mode mode = !good
                  ? reactive::Mode::MODE_BLIND
                  : (pulsing.load() ? reactive::Mode::MODE_CRUISE : reactive::Mode::MODE_STOP);
              bibowire::Decide d;
              d.revIndex = rev;
              d.clearanceMm = static_cast<UInt32>(ahead * 1000.0f + 0.5f);
              d.steerMilli = milli(askedSteer);
              d.throttleMilli = milli(askedThrottle);
              d.mode = static_cast<UInt8>(mode);
              d.stop = static_cast<UInt8>(mode == reactive::Mode::MODE_CRUISE ? 0 : 1);
              d.source = pilotMode();
              viewfeed::publishDecide(d);
          }
          lastRevMs = now;
      }
  }

  Bool Car::Inner::arm()
  {
      UniqueLock<Mutex> lock(mu);
      if(armed)
      {
          return !stopped.load();
      }
      // A run that had already ended has said why.
      const Bool wasRunning = gov.ended() == End::END_NONE;
      if(wasRunning)
      {
          const Bool seen = wake.wait_for(lock, Millis(FIRST_REVOLUTION_MS), [this]() {
              return gov.ended() != End::END_NONE || goodRevolutions > 0;
          });
          if(!seen)
          {
              gov.end(End::END_NO_REVOLUTION);
          }
      }
      if(gov.ended() == End::END_NONE)
      {
          gov.requestArm();
          // The Governor ends the run itself past ARM_CONFIRM_MS; two more passes
          // are its time to say so.
          const Int32 waitMs = ARM_CONFIRM_MS + 2 * MINDER_MS;
          const Bool confirmed = wake.wait_for(lock, Millis(waitMs), [this]() {
              return gov.ended() != End::END_NONE || gov.armState() == Arm::ARM_CONFIRMED;
          });
          if(!confirmed)
          {
              gov.end(End::END_ARM_TIMEOUT);
          }
      }
      const End why = gov.ended();
      const carrules::Board board = gov.board();
      armed = why == End::END_NONE;
      if(!armed)
      {
          // Not before the minder has sent STOP, which car.hxx promises.
          wake.wait(lock, [this]() { return stopped.load(); });
      }
      lock.unlock();
      wake.notify_all();

      if(!armed)
      {
          if(wasRunning)
          {
              say(Str("arm: ") + carrules::endName(why) + "\n");
          }
          return false;
      }
      if(!opt.drive)
      {
          say("arm: dry run - the Pico is not open, nothing will move\n");
          return true;
      }
      const CharSeq from = trim.escLimits.empty()
          ? "the Pico's compiled band - save ESC limits in the viewer's Trim pane"
          : "trim.txt";
      Array<Char, 160> line{};
      std::snprintf(
          line.data(),
          line.size(),
          "arm: armed, esc %d..%d us (%s)\n",
          std::max(board.escMinUs, NEUTRAL_US),
          board.escMaxUs,
          from
      );
      say(line.data());
      return true;
  }

  Scan Car::Inner::nextScan()
  {
      UniqueLock<Mutex> lock(mu);
      const Bool fresh = wake.wait_for(lock, Millis(SCAN_WAIT_MS), [this]() {
          return stopped.load() || latest.revolution > lastReturned;
      });
      if(!fresh || stopped.load())
      {
          return Scan();
      }
      lastReturned = latest.revolution;
      return latest;
  }

  Void Car::Inner::order(Float32 t, Float32 s)
  {
      Bool early = false;
      {
          LockGuard<Mutex> lock(mu);
          if(!armed)
          {
              early = !saidEarlyDrive;
              saidEarlyDrive = true;
          }
          else
          {
              throttle = t;
              steer = s;
              driveMs = nowMs();
          }
      }
      if(early)
      {
          say("car: drive() before arm() moves nothing\n");
      }
  }

  Int32 Car::Inner::finish()
  {
      if(finished)
      {
          return code;
      }
      finished = true;
      {
          LockGuard<Mutex> lock(mu);
          gov.end(End::END_FINISHED);
          quit = true;
      }
      wake.notify_all();
      if(minder.joinable())
      {
          minder.join();
      }

      // The minder's last pass sent STOP. This one is finish()'s own, and the
      // answer is read so a last ERR is printed.
      if(picoOpen)
      {
          static_cast<Void>(sender.send(proto::stop()));
          sleepMs(STOP_REPLY_MS);
          Vec<Str> last;
          if(carlink::drain(last) == carlink::Result::RESULT_CLOSED)
          {
              say("pico: the port was gone at the final STOP\n");
          }
          sayErrors(last);
      }
      {
          LockGuard<Mutex> lock(mu);
          stopped = true;
      }
      wake.notify_all();
      if(lidarThread.joinable())
      {
          lidarThread.join();
      }
      stopPrinter();

      if(spinning && !lidar::motorOff())
      {
          std::printf("lidar motor off: %s\n", lidar::reason().c_str());
      }
      spinning = false;
      if(lidarOpen)
      {
          lidar::close();
      }
      if(picoOpen)
      {
          carlink::close();
      }
      if(viewer)
      {
          viewfeed::stop();
      }

      End why = End::END_NONE;
      UInt64 good = 0;
      UInt32 errors = 0;
      {
          LockGuard<Mutex> lock(mu);
          why = gov.ended();
          good = goodRevolutions;
          errors = gov.board().errors;
      }
      code = carrules::exitCode(why, good);
      if(why == End::END_HELP || why == End::END_BAD_FLAGS)
      {
          return code;
      }
      Array<Char, 80> pico{};
      if(picoOpen)
      {
          std::snprintf(
              pico.data(),
              pico.size(),
              "  worst send gap %lld ms  pico errors %u",
              static_cast<long long>(sender.worstGapMs()),
              static_cast<unsigned>(errors)
          );
      }
      std::printf(
          "ended: %s  revolutions %llu, %llu good%s  exit %d\n",
          carrules::endName(why),
          static_cast<unsigned long long>(revolutions.load()),
          static_cast<unsigned long long>(good),
          pico.data(),
          code
      );
      return code;
  }

  Car::Car(Int32 argc, Char** argv) : inner(makeUniq<Inner>())
  {
      try
      {
          inner->open(argc, argv);
      }
      catch(const std::exception& e)
      {
          std::printf("car: could not open: %s\n", e.what());
          inner->endRun(End::END_OPEN_FAILED);
      }
  }

  Car::~Car()
  {
      inner->finish();
      if(inner->owner)
      {
          carAlive = false;
      }
  }

  Bool Car::arm()
  {
      return inner->arm();
  }

  Bool Car::ok() const
  {
      return !inner->stopped.load();
  }

  Scan Car::scan()
  {
      return inner->nextScan();
  }

  Void Car::drive(Float32 throttle, Float32 steer)
  {
      inner->order(throttle, steer);
  }

  Int32 Car::finish()
  {
      return inner->finish();
  }

}
