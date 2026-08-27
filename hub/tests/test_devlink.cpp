// Tests for the gone-versus-broken rule. See src/devlink.hpp.
//
// This one DOES touch the OS - portPresent asks Windows what serial ports it
// has - which is the point. The rule is only worth anything if it agrees with
// the machine, and a test that mocked the enumeration would only prove the mock
// matched itself.

#include "shared.hpp"
#include "devlink.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdio>

namespace {

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
    testDescribe();

    std::printf("\n%d checks, %d failed\n", checks, failures);
    std::printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
