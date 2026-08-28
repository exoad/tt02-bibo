// Telling "the device is GONE" apart from "the device is BROKEN".
//
// ---------------------------------------------------------------------------
// WHY THIS IS ITS OWN FILE
//
// Every link in this app - the lidar, the Pico, anything on a COM port later -
// hits the same moment: a read fails, and the question is what to say about it.
// Both of them used to answer the same way, with a red Error state and the
// Win32 text, which is wrong for by far the commonest cause. Pulling a USB
// cable is not a fault. It is a thing a person does deliberately, twenty times
// an evening, and an app that responds to it with a red banner and a diagnostic
// is an app that cries wolf until the banner stops meaning anything.
//
// So the rule lives in one place, and every link asks it the same question.
//
// ---------------------------------------------------------------------------
// HOW IT DECIDES
//
// Two signals, cheapest and weakest first:
//
//   1. The Win32 error code. A handful of them mean "removed" and nothing else.
//      This is fast but not sufficient: a device can vanish and leave a generic
//      code behind, and ERROR_GEN_FAILURE in particular is what a serial port
//      returns for half a dozen unrelated reasons.
//
//   2. Whether the port is still ENUMERATED. This is the authoritative one. If
//      Windows no longer lists COM7, the adapter is not plugged in, and no error
//      code is going to change that. It costs a QueryDosDevice call, so it is
//      asked once at the moment of failure rather than in any loop.
//
// A failure is a DISCONNECT if either says so. Everything else is a real fault
// and keeps its red banner, because those are the ones worth reading.
#pragma once

#include "shared.hpp"

namespace dev {

// Why a link stopped.
enum class Loss
{
    // The device is not there any more. Not an error: say so quietly, re-enable
    // the connect control, and watch for it coming back.
    LOSS_UNPLUGGED = 0,

    // Something else went wrong and the device is, as far as we can tell, still
    // attached. Worth a person's attention.
    LOSS_FAULT
};

// True for the Win32 codes that only ever mean "that device is not there".
//
// Deliberately does NOT include ERROR_ACCESS_DENIED: a port that is present but
// held by another program is a real problem with a real fix, and telling
// somebody it is unplugged would send them to check a cable that is fine.
[[nodiscard]] Bool isRemovalCode(UInt32 win32Code) noexcept;

// Serial ports Windows currently lists, e.g. {"COM3","COM7"}. Never throws;
// returns empty on any failure, which callers must read as "cannot tell"
// rather than as "nothing is attached".
[[nodiscard]] Vec<Str> listPorts();

// Whether `port` ("COM7", case-insensitive) is enumerated right now.
//
// Returns true when the enumeration itself failed - "I could not check" must
// not be reported as "it is gone", or a transient API failure would present as
// an unplugged cable.
[[nodiscard]] Bool portPresent(const Str& port);

// ---------------------------------------------------------------------------
// WHAT IS ON A PORT
//
// "COM10" says nothing about what is plugged into it, and guessing is worse
// than useless: serial ports are EXCLUSIVE, so a subsystem that guesses wrong
// does not merely fail - it takes the port away from the subsystem that was
// right, and both then report an error about hardware that is working.
//
// That is not hypothetical. With the lidar unplugged, its "a USB bridge
// normally enumerates above the built-in ports" fallback picked the highest
// number on the machine, which was the Pico, and the two fought over COM10
// until neither worked.
//
// HKLM\HARDWARE\DEVICEMAP\SERIALCOMM maps each driver's device name to the
// port it owns, which answers the question outright and needs neither SetupAPI
// nor WMI:
//
//   \Device\Silabser0  -> COM7    the CP210x bridge every RPLIDAR adapter uses
//   \Device\USBSER000  -> COM10   a USB CDC device, which is what a Pico is
//   \Device\BthModem0  -> COM3    a Bluetooth link, and never anything we want
enum class PortKind
{
    PORT_KIND_UNKNOWN = 0,
    PORT_KIND_CP210X,      // Silicon Labs bridge - the RPLIDAR adapter
    PORT_KIND_USB_CDC,     // a USB serial device - the Pico
    PORT_KIND_BLUETOOTH    // an outgoing Bluetooth port, always a dead end
};

[[nodiscard]] PortKind portKind(const Str& port);

// Human-readable, for a dropdown that would otherwise be six identical-looking
// COM numbers with no way to tell which is which.
[[nodiscard]] const Char* portKindName(PortKind k);

// Could a port of this kind be the lidar.
//
// Takes the KIND rather than the port name on purpose: the rule is about
// classification, not about this machine, so it can be exercised without a
// CP210x plugged in. It lived in app_ui.cpp behind a port name, where the only
// test that could reach it was one that needed the right hardware attached -
// and the test that existed was a tautology that could not fail.
//
// This is the rule that stops the lidar grabbing COM10. A USB CDC device is a
// Pico; a Bluetooth port is a dead end that answers with a timeout looking
// exactly like a broken lidar. UNKNOWN is allowed through because an
// unrecognised adapter is a real possibility and refusing it would be worse
// than offering it.
[[nodiscard]] Bool couldBeLidar(PortKind k);

// The verdict. `win32Code` may be 0 when the caller has no code to offer, in
// which case the enumeration decides on its own.
[[nodiscard]] Loss classify(const Str& port, UInt32 win32Code);

// One sentence for a person, given the verdict. `what` names the device the way
// the UI does ("RPLIDAR C1", "Pico"), so the message reads as being about their
// hardware rather than about a handle.
[[nodiscard]] Str describe(Loss why, const Str& what, const Str& port);

} // namespace dev
