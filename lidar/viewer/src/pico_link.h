// Serial link to the Pico 2 W on the car.
//
// Deliberately dumb: it moves newline-terminated ASCII lines in both directions
// and keeps a bounded log of them. It knows nothing about what the lines mean —
// the command vocabulary lives in firmware/ and in the UI that renders it.
//
// This is the debug/bring-up channel over USB CDC. The real control link is UDP
// over the Pico's own AP (see AGENTS.md); this one exists so a human can talk to
// the board directly, and so phase 2 can be brought up before any of that works.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

enum class PicoState
{
    Disconnected,
    Connecting,
    Connected,
    Error,
};

// One logged line, in either direction.
struct PicoLine
{
    double      t_s      = 0.0;    // seconds since the link was opened
    bool        outgoing = false;  // true = host -> Pico
    std::string text;
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
    void connect(const std::string& port, int baud = 115200);
    void disconnect();

    PicoState   state() const;
    std::string error() const;      // last error, empty if none
    std::string port() const;       // port currently open, empty if none

    // Queues a line for transmission. A trailing newline is added if absent.
    // Safe to call when disconnected (the line is dropped and counted).
    void send(const std::string& line);

    // Moves newly logged lines into `out` (appending). Returns how many were
    // appended. Call once per UI frame.
    size_t drain(std::vector<PicoLine>& out);

    unsigned long long tx_lines() const;
    unsigned long long rx_lines() const;
    unsigned long long dropped() const;   // sends attempted while disconnected

    // Seconds since the last received line; large means the board is silent.
    // Returns a negative value when nothing has ever been received.
    double last_rx_age_s() const;

    // Serial ports whose USB VID is 2E8A (Raspberry Pi). Never throws.
    static std::vector<std::string> list_pico_ports();

    // Opens `port` at 1200 baud and closes it, which the Pico SDK's USB reset
    // interface interprets as "reboot into BOOTSEL". That is what makes flashing
    // possible without picotool: the board then mounts as the RPI-RP2 drive and
    // a .uf2 can simply be copied onto it.
    //
    // Returns false if the port could not be opened at all. A successful touch
    // makes the port disappear, so callers should expect to lose the link.
    static bool bootsel_touch(const std::string& port);
};
