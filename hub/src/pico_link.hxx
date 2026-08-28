// Serial link to the Pico 2 W on the car.
//
// Deliberately dumb: it moves newline-terminated ASCII lines in both directions
// and keeps a bounded log of them. It knows nothing about what the lines mean —
// the command vocabulary lives in firmware/ and in the UI that renders it.
//
// This is the debug/bring-up channel over USB CDC. The real control link is UDP
// over the Pico's own AP (see docs/conventions.md); this one exists so a human can talk to
// the board directly, and so phase 2 can be brought up before any of that works.
#pragma once

#include "shared.hxx"

#include <cstddef>

enum class PicoState
{
    PICO_STATE_DISCONNECTED,
    PICO_STATE_CONNECTING,
    PICO_STATE_CONNECTED,

    // The board went away - unplugged, or rebooted into BOOTSEL, which drops
    // the CDC port by design and is the single commonest way this link ends.
    // Not an error, for the reasons in devlink.hxx.
    PICO_STATE_UNPLUGGED,

    PICO_STATE_ERROR,
};

// One logged line, in either direction.
struct PicoLine
{
    Float64      tS      = 0.0;    // seconds since the link was opened
    Bool        outgoing = false;  // true = host -> Pico
    Str text;
};

class PicoLink
{
public:
    PicoLink();
    ~PicoLink();

    PicoLink(const PicoLink&)            = delete;
    PicoLink& operator=(const PicoLink&) = delete;

    // Non-blocking. Watch state() for the outcome.
    // The Pico's CDC ignores the baud rate; 115200 is convention, not protocol.
    Void connect(const Str& port, Int32 baud = 115200);
    Void disconnect();

    PicoState   state() const;
    Str error() const;      // last error, empty if none
    Str port() const;       // port currently open, empty if none

    // Queues a line for transmission. A trailing newline is added if absent.
    // Safe to call when disconnected (the line is dropped and counted).
    Void send(const Str& line);

    // Moves newly logged lines into `out` (appending). Returns how many were
    // appended. Call once per UI frame.
    Size drain(Vec<PicoLine>& out);

    UInt64 txLines() const;
    UInt64 rxLines() const;
    UInt64 dropped() const;   // sends attempted while disconnected

    // Seconds since the last received line; large means the board is silent.
    // Returns a negative value when nothing has ever been received.
    Float64 lastRxAgeS() const;

    // Serial ports whose USB VID is 2E8A (Raspberry Pi). Never throws.
    static Vec<Str> listPicoPorts();

    // Opens `port` at 1200 baud and closes it, which the Pico SDK's USB reset
    // interface interprets as "reboot into BOOTSEL". That is what makes flashing
    // possible without picotool: the board then mounts as the RPI-RP2 drive and
    // a .uf2 can simply be copied onto it.
    //
    // Returns false if the port could not be opened at all. A successful touch
    // makes the port disappear, so callers should expect to lose the link.
    static Bool bootselTouch(const Str& port);

private:
    // Opaque implementation. Declared here rather than left out: without a
    // member to hang state on, the .cpp had to keep a file-static
    // unorderedMap<const PicoLink*, ...> behind a global mutex and do a hashed
    // lookup on every accessor, including drain() at 60 fps.
    struct Impl;
    Impl* pimpl = nullptr;
};
