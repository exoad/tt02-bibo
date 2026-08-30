// Tests for the gone-versus-broken rule. See src/devlink.hxx.
//
// This one DOES touch the OS - portPresent asks Windows what serial ports it
// has - which is the point. The rule is only worth anything if it agrees with
// the machine, and a test that mocked the enumeration would only prove the mock
// matched itself.

#include "shared.hxx"
#include "devlink.hxx"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdio>

namespace
{

  Int32 checks   = 0;
  Int32 failures = 0;

  Void check(Bool ok, const Char* what)
  {
      ++checks;
      if(ok)
      {
          std::printf("  ok    %s\n", what);
      }
      else
      {
          ++failures;
          std::printf("  FAIL  %s\n", what);
      }
  }

  Void checkStr(const Str& got, const Str& want, const Char* what)
  {
      ++checks;
      if(got == want)
      {
          std::printf("  ok    %s\n", what);
      }
      else
      {
          ++failures;
          std::printf("  FAIL  %s\n         got  \"%s\"\n         want \"%s\"\n",
                      what, got.c_str(), want.c_str());
      }
  }

  Void testRemovalCodes()
  {
      std::printf("\n-- removal codes --\n");

      check(dev::isRemovalCode(ERROR_FILE_NOT_FOUND),
            "a port that does not exist is a removal");
      check(dev::isRemovalCode(ERROR_BAD_COMMAND),
            "ERROR_BAD_COMMAND is the classic mid-read USB serial removal");
      check(dev::isRemovalCode(ERROR_DEVICE_NOT_CONNECTED),
            "ERROR_DEVICE_NOT_CONNECTED is a removal");
      check(dev::isRemovalCode(ERROR_INVALID_HANDLE),
            "a handle that died under us is a removal");

      // The one that must NOT be a removal. A port held by another program is a
      // real problem with a real fix, and calling it "unplugged" would send
      // somebody to check a cable that is perfectly fine.
      check(!dev::isRemovalCode(ERROR_ACCESS_DENIED),
            "ACCESS_DENIED is NOT a removal - something else holds the port");

      check(!dev::isRemovalCode(ERROR_GEN_FAILURE),
            "GEN_FAILURE is too vague to be evidence on its own");
      check(!dev::isRemovalCode(0), "success is not a removal");
  }

  Void testPortPresence()
  {
      std::printf("\n-- port presence --\n");

      // COM255 is legal to name and vanishingly unlikely to exist.
      check(!dev::portPresent("COM255"), "a port that is not there is not present");
      check(!dev::portPresent(""), "an empty port name is not present");

      const Vec<Str> ports = dev::listPorts();
      std::printf("  ..    %d serial port(s) on this machine\n",
                  static_cast<Int32>(ports.size()));

      // Whatever IS enumerated must be reported as present, including in the
      // other case - the settings file and the enumeration disagree about
      // capitalisation often enough to matter.
      if(!ports.empty())
      {
          check(dev::portPresent(ports[0]),
                "an enumerated port is reported present");

          Str lowered = ports[0];
          for(Char& c : lowered)
          {
              if(c >= 'A' && c <= 'Z')
              {
                  c = static_cast<Char>(c - 'A' + 'a');
              }
          }
          check(dev::portPresent(lowered), "and the comparison ignores case");
      }
      else
      {
          std::printf("  ..    (no ports; skipping the present-port checks)\n");
      }
  }

  Void testClassify()
  {
      std::printf("\n-- classify --\n");

      // A removal code decides on its own, without asking the OS.
      check(dev::classify("COM7", ERROR_BAD_COMMAND) == dev::Loss::LOSS_UNPLUGGED,
            "a removal code is enough by itself");

      // No code at all: the enumeration decides. COM255 is not there.
      check(dev::classify("COM255", 0) == dev::Loss::LOSS_UNPLUGGED,
            "a port that is gone is a disconnect even with no error code");

      // A vague code on a port that IS present must stay a fault. This is the
      // case that keeps the red banner meaningful.
      const Vec<Str> ports = dev::listPorts();
      if(!ports.empty())
      {
          check(dev::classify(ports[0], ERROR_GEN_FAILURE) == dev::Loss::LOSS_FAULT,
                "a vague code on a port that is still there stays a fault");
          check(dev::classify(ports[0], ERROR_ACCESS_DENIED) == dev::Loss::LOSS_FAULT,
                "a port held by another program is a fault, not a disconnect");
      }
  }

  // The bug this was written for: a port that is a Pico must never be mistaken
  // for a lidar adapter, because serial ports are exclusive and a wrong guess
  // takes the port away from the subsystem that was right.
  Void testPortKind()
  {
      std::printf("\n-- what is on a port --\n");

      const Vec<Str> ports = dev::listPorts();
      Int32 cp210x = 0;
      Int32 usbcdc = 0;
      Int32 bt     = 0;

      for(const Str& p : ports)
      {
          const dev::PortKind k = dev::portKind(p);
          std::printf("  ..    %-6s %s\n", p.c_str(), dev::portKindName(k));
          if(k == dev::PortKind::PORT_KIND_CP210X)
          {
              ++cp210x;
          }
          if(k == dev::PortKind::PORT_KIND_USB_CDC)
          {
              ++usbcdc;
          }
          if(k == dev::PortKind::PORT_KIND_BLUETOOTH)
          {
              ++bt;
          }
      }

      check(dev::portKind("COM255") == dev::PortKind::PORT_KIND_UNKNOWN,
            "a port that does not exist has no kind");
      check(dev::portKind("") == dev::PortKind::PORT_KIND_UNKNOWN,
            "an empty name has no kind");

      // Every port on the machine is classified as something, or the registry
      // walk is not finding entries it should.
      Int32 known = cp210x + usbcdc + bt;
      check(known == static_cast<Int32>(ports.size()),
            "every enumerated port is identified");

      // Whatever is attached, nothing the machine reports may be BOTH a Pico and
      // a lidar adapter. Stated over the enumeration rather than over one port,
      // so it says something when there is no Pico plugged in.
      Int32 both = 0;
      for(const Str& p : ports)
      {
          const dev::PortKind k = dev::portKind(p);
          if(k == dev::PortKind::PORT_KIND_USB_CDC
             && dev::couldBeLidar(k))
          {
              ++both;
          }
      }
      check(both == 0, "no enumerated port is both a Pico and a lidar candidate");
  }

  // The rule that stops the lidar grabbing the Pico's port.
  //
  // This replaces a check that read as if it guarded the COM10 collision and
  // could not: it asked whether a port already known to be USB_CDC was also
  // CP210X, inside the branch that had just established it was not. A PortKind is
  // one value, so that could never fail - and it only ran at all when a Pico
  // happened to be plugged in, so the count of checks changed depending on what
  // was on the desk.
  //
  // Over the KINDS, so it needs no hardware and covers the cases the machine does
  // not currently have.
  Void testLidarRule()
  {
      std::printf("\n-- what may be the lidar --\n");

      check(dev::couldBeLidar(dev::PortKind::PORT_KIND_CP210X),
            "a CP210x is the adapter every RPLIDAR uses");

      check(!dev::couldBeLidar(dev::PortKind::PORT_KIND_USB_CDC),
            "a USB CDC port is a Pico and is NEVER the lidar");

      check(!dev::couldBeLidar(dev::PortKind::PORT_KIND_BLUETOOTH),
            "a Bluetooth port is never the lidar");

      // Deliberately permissive: an adapter nobody has seen before should be
      // offerable. Refusing it would be a worse failure than offering it, because
      // a port missing from the list reads as a driver problem.
      check(dev::couldBeLidar(dev::PortKind::PORT_KIND_UNKNOWN),
            "an unrecognised port is still offered");
  }

  Void testDescribe()
  {
      std::printf("\n-- messages --\n");

      checkStr(dev::describe(dev::Loss::LOSS_UNPLUGGED, "RPLIDAR C1", "COM7"),
               "RPLIDAR C1 disconnected - COM7 is gone",
               "an unplug reads as an unplug");

      checkStr(dev::describe(dev::Loss::LOSS_FAULT, "RPLIDAR C1", "COM7"),
               "RPLIDAR C1 stopped responding on COM7",
               "a fault reads as a fault");

      checkStr(dev::describe(dev::Loss::LOSS_UNPLUGGED, "Pico", ""),
               "Pico disconnected",
               "no port name still produces a sentence");

      // No error codes, no hex, no exclamation marks in the calm case.
      const Str m = dev::describe(dev::Loss::LOSS_UNPLUGGED, "Pico", "COM10");
      check(m.find("0x") == Str::npos && m.find('!') == Str::npos,
            "an unplug message carries no error code and does not shout");
  }

} // namespace

int main()
{
    std::printf("devlink tests\n");

    testRemovalCodes();
    testPortPresence();
    testClassify();
    testPortKind();
    testLidarRule();
    testDescribe();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
