// Serial link to the Pico 2 W on the car.
//
// Deliberately dumb: it moves newline-terminated ASCII lines in both directions
// and keeps a bounded log of them. It knows nothing about what the lines mean —
// the command vocabulary lives in firmware/ and in the UI that renders it.
//
// TWO TRANSPORTS, ONE CLASS. connect() opens a USB CDC serial port;
// connectUdp() sends the same lines to the same firmware over Wi-Fi. Everything
// above this - the console, the drive polls, the keyboard controller, the
// emergency stop - is written against send() and drain() and does not know or
// care which one is open. That is the whole reason the wireless link speaks the
// existing text protocol instead of a binary one of its own.
//
// What is NOT the same: flashing. Rebooting into BOOTSEL is a 1200-baud touch
// on a serial port, and a board reached only over Wi-Fi cannot be reflashed at
// all. wireless() exists so the UI can say that rather than fail obscurely.
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
    Float64      tS = 0.0;    // seconds since the link was opened
    Bool        outgoing = false;  // true = host -> Pico
    Str text;

    // Traffic the HUB generated on its own schedule, rather than anything a
    // person did: the STATUS, LIGHTS and sensor polls, and the replies to them.
    //
    // Worth a bit of its own because a console that shows it is unreadable. The
    // indicator poll alone runs at 120 ms, so eight sends and eight replies a
    // second bury everything you actually typed. Hidden by default and one
    // checkbox away, rather than dropped - when the question is "is the link
    // alive at all", this chatter is the answer.
    Bool poll = false;
};

class PicoLink
{
public:
    PicoLink();
    ~PicoLink();

    PicoLink(const PicoLink&) = delete;
    PicoLink& operator=(const PicoLink&) = delete;

    // Non-blocking. Watch state() for the outcome.
    // The Pico's CDC ignores the baud rate; 115200 is convention, not protocol.
    Void connect(const Str& port, Int32 baud = 115200);

    // The same link, over Wi-Fi. `host` is the address the board printed when
    // it joined - WIFI on the USB console says it - and `port` is NET_PORT from
    // firmware/lib/net.h.
    //
    // UDP has no connection to make, so "connected" here cannot mean what it
    // means on a cable. It is EARNED: this sends a PING and stays CONNECTING
    // until the board answers, because a link that reported itself up the
    // instant somebody typed an address would report a switched-off car as
    // present.
    Void connectUdp(const Str& host, UInt16 port = 4242);

    Void disconnect();

    // True while the open link is the wireless one. Flashing needs a cable.
    Bool wireless() const;

    PicoState   state() const;
    Str error() const;      // last error, empty if none
    Str port() const;       // port currently open, empty if none

    // Queues a line for transmission. A trailing newline is added if absent.
    // Safe to call when disconnected (the line is dropped and counted).
    // `poll` marks a line the hub sent itself on a timer. See PicoLine::poll.
    Void send(const Str& line, Bool poll = false);

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
