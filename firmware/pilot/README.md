# pilot

The companion board's program. It sees a room and decides; it has not yet been
handed the car.

## What runs where

    Pico 2 W      the things that must not stop - servo and ESC pulses, slew
                  limits, the deadman, the lamps. Already written.
    Orange Pi     the thinking - odometry, the speed controller, pure pursuit,
                  and eventually the lidar. This directory.
    hub           the operator console, on a laptop. Does NOT run on the Pi.

The split is not about compute; an RP2350 could do pursuit arithmetic all day.
It is about what happens when something takes too long. On the Pi a late tick is
a late command and the car holds its last one under a deadman. On the Pico a
late tick is a servo that stops being told anything.

## What is here

    src/proto.hxx      the car's line protocol, read and written. Finished and
                       tested - 41 checks.
    src/link.hxx       the transport to the car, namespace `carlink`. POSIX
                       termios behind `#if defined(__linux__)`, a reader thread
                       splitting bytes into lines, the port held exclusively,
                       a 100 ms cap on a stalled write, and every send()
                       counted once as transmitted or dropped. Tested on the
                       Pi against a pseudo-terminal; NOT yet against the real
                       Pico. Everywhere else it refuses (RESULT_NO_PLATFORM).
    src/lidar.hxx      the C1 over Slamtec's SDK, one revolution at a time as
                       a `Vec<reactive::Ray>`. Real only when CMake is given
                       -DPILOT_RPLIDAR_SDK on Linux; otherwise every call
                       refuses with a reason, and grab() EMPTIES its vector on
                       every failure so an ignored Bool reads as blind.
    src/reactive.hxx   the dumb driver: one scan in, steer and throttle out.
                       Pure; the three traps that crash cars are its tests.
    src/autonomy.hxx   one tick of pursuit along a path. Tunings are real;
                       step() returns STATUS_NOT_IMPLEMENTED and touches no
                       output. Waits on an encoder.
    src/scanwire.hxx   the wire format between the board's scan feed and the
                       hub: one revolution per text line, integers only, the
                       same object file compiled into both ends. Pure; its
                       header comment is the spec and tests/test_scanwire.cxx
                       holds it to it. The pilot adds a D line - its decision
                       about the revolution just sent.
    src/feed.hxx       the scan feed's network half, namespace `feed`: a TCP
                       server on its own thread that hands complete lines to
                       every client, drops one that falls half a second
                       behind, and takes MOTOR and QUIT back as wishes for
                       the owner. It never touches the lidar - the program
                       that owns the device does, from its own thread - so
                       scanfeed and the pilot serve one wire from one file.
                       Linux behind `#if defined(__linux__)`; refuses
                       elsewhere. tests/test_feed.cxx, on the board only.
    app/main.cxx       the program, `pilot`. Grabs a revolution, runs
                       reactive::step with the measured dt, sends STEER and
                       ESC (or NEUTRAL) to the car; sends anyway after 200 ms
                       without a revolution so the board's 400 ms deadman is
                       never what stops the car. Serves the scan feed while it
                       drives, and the same lines to /tmp/bibo-scan.txt,
                       which nothing reads now. `--dry` decides without a Pico;
                       `--no-feed` drives without viewers. Built only with
                       the SDK.
    tools/lidar_probe  is the lidar there and what does it see. Run it first.
    tools/scanfeed     the lidar's revolutions on TCP 8011 for the hub, under
                       systemd: a lidar-owning loop on top of src/feed.hxx.
                       Idle with the port free until a client connects. See
                       "Seeing the lidar from the hub".

`proto` was finished first because it is the part that could be finished: pure
string work, provable on a laptop, and where the bugs in a text protocol live.
`reactive` came next for the same reason. The two files that touch hardware
each carry a refusing half, so that a build without the device says "no
device" rather than pretending to have one.

## The maths is the firmware's

`geom`, `kinematics`, `pursuit`, `control` and `plan` are pure headers in
`firmware/lib` that each claim, in their own comments, to compile for the Pico,
the Orange Pi and the host test from one copy. `tests/test_pilot.cxx` is the
first thing that holds them to it, by including them in a program that is not
firmware and has no SDK.

Two copies of pure pursuit, one per board, would agree right up until somebody
fixed a sign in one of them.

## Why the companion speaks the human protocol

A binary protocol between two computers is the obvious choice and the wrong one.
The text protocol already exists on the board, is already carried over both USB
CDC and UDP - `hub/src/pico_link.hxx` swaps transports under one `send()` and
`drain()` precisely because the payload is text - and can be driven by hand when
something is wrong. A second format means a second parser in the firmware, on
the board whose flash is scarce, to save bytes on a link carrying a few hundred
a second.

The cost is that parsing text is where bugs live. `proto::field()` matches on
token boundaries for that reason: `strstr(line, "esc=")` finds the `esc=` inside
`desc=` and returns a number from the wrong field, having reported success. The
test demonstrates it rather than asserting it.

## The board

Ordered, not yet arrived. Recorded here because the numbers decide what this
program may assume.

    SoC        Allwinner A733 - NOT the RK3588S the older Orange Pi 4 used
    CPU        2x Cortex-A76 @ 2.0 GHz + 6x Cortex-A55
    NPU        3 TOPS INT8
    RAM        up to 16 GB LPDDR5
    Storage    M.2 NVMe, microSD
    USB        1x USB3, 3x USB2 - the Pico takes one, the lidar another
    Network    Gigabit Ethernet, WiFi 6
    GPIO       40-pin header
    Power      5 V 3 A over USB-C
    Size       89 x 56 mm, 58 g
    OS         Debian / Ubuntu
    Price      about $35

The two big cores are what the autonomy gets; the six little ones are what keeps
the rest of the system out of its way. The NPU is not used by anything here and
should not be planned around until something needs it.

## Building

On the laptop, MSVC:

    tests\build_proto_test.bat run
    tests\build_pilot_test.bat run
    tests\build_reactive_test.bat run
    tests\build_scanwire_test.bat run

On the board (or any Linux box), g++ and CMake - the same sources, no Pico SDK:

    cmake -S firmware/pilot -B build-pilot && cmake --build build-pilot -j
    ctest --test-dir build-pilot --output-on-failure

First built on the Orange Pi on 2026-09-07: Ubuntu Jammy, gcc 11.4, aarch64,
41 + 17 + 45 checks passing. The first thing the Pi build found was that
`namespace link` collides with POSIX `link()` from `<unistd.h>` - a name MSVC
never objected to - which is why the link's namespace is `carlink`.

That builds the library and the three tests, with the lidar half refusing. To
build the program and the probe, point CMake at a built checkout of Slamtec's
SDK (`make` in `rplidar_sdk`; the library lands in `output/Linux/Release`):

    cmake -S firmware/pilot -B build-pilot -DPILOT_RPLIDAR_SDK=$HOME/rplidar_sdk
    cmake --build build-pilot -j

## Running

    build-pilot/lidar_probe                  # is the C1 there, what does it see
    build-pilot/pilot --dry --seconds 12     # decide for 12 s, touch no car
    build-pilot/pilot --arm                  # drive, until Ctrl-C

    pilot [--lidar PORT] [--pico PORT] [--dry] [--arm] [--forward DEG] [--seconds N] [--no-feed]

The lidar is `/dev/ttyUSB0` and the Pico `/dev/ttyACM0` unless told otherwise.
While it runs the pilot also serves the scan feed on TCP 8011 and writes
`/tmp/bibo-scan.txt` (which nothing reads now) - see "Seeing the lidar from the
hub"; `--no-feed` turns both off. When 8011 is already taken (scanfeed idling
under systemd) the feed moves to 8012 and the log says so; scanfeed relays
viewers there, so nothing on the laptop changes. A feed that can bind neither
is said once and the pilot drives without viewers rather than refusing to
start.
`--forward` is the raw lidar angle that points along the car - a mounting
fact, not a tuning, and 0 is an assumption until it is measured. `--arm` is
what lets the car move: without it the board refuses every throttle pulse and
the program prints each refusal, which is the correct behaviour for a car that
was not meant to go anywhere. Ctrl-C sends STOP, stops the motor, and exits 0.
A timed run that saw no revolution at all exits 1.

### Reaching the board

Outdoors the board joins the phone's hotspot (hidden; the SSID is not recorded
in this public repository) by itself
at boot, and switches to it within 20 s whenever it appears while the board is
on some other network (the `bibo-prefer-hotspot` timer; NetworkManager would
otherwise stay put); join the laptop to the same hotspot and the two are on one
LAN with nothing in between. The hotspot hands out addresses, so use the name: `ssh
jack@bibobox.local` once mDNS is enabled on the board, or `jack@bibobox` over the
tailnet whenever the phone has data. The literal address is not recorded here:
this repository is public, and the name is what should be dialled regardless.
docs/conventions.md "Link" has the whole picture.

### Installing the board services

    sudo sh ~/tt02-bibo/firmware/pilot/tools/status/install.sh

installs the `bibo-scanfeed` unit and the `bibo-prefer-hotspot` timer, both
enabled at boot, and turns mDNS on - both systemd-resolved's global switch and
the hotspot profile's, since on this Ubuntu the second cannot exceed the first.
The profile half needs the hotspot joined first, or `BIBO_HOTSPOT=<profile>`.
Run it again after a pull; it is idempotent. A board that still carries the
old status page's units (`bibo-status.*`) has them disabled and deleted.

### Seeing the lidar from the hub

`tools/scanfeed` is the board's end of `src/scanwire.hxx`: a TCP server on
port 8011 that opens the C1, spins it up, and writes one text line per
revolution - `F <count> <milli-hertz> <centi-deg>,<mm>,<quality> ...` - to
every client connected, after an `INFO`, a `HEALTH` and a `MOTOR 1`. A client
may write `MOTOR 0` and `MOTOR 1` to stop and restart the motor, or `QUIT`.
The hub's lidar source is the intended client; the no-tool check is

    nc bibobox.local 8011

which prints the device and then a revolution ten times a second until Ctrl-C.

**The pilot serves the same feed while it drives.** Same port, same lines,
from `src/feed.hxx` - the network half scanfeed is built on - plus one the
standalone feed never sends: after each `F` a `D <mode> <clearance-mm> <hits>
<steer> <throttle> <stop>` saying what the pilot decided about that revolution,
steer and throttle in thousandths. A blind tick sends only the `D`, so a
viewer's mode reads `blind` when the pilot's does rather than freezing on the
last picture. What differs is who the lidar is spinning for: **a viewer cannot
stop the motor while the pilot runs.** `MOTOR 0` is answered with the true
state, `MOTOR 1`, and the log says "viewer asked for the motor; the pilot keeps
it while driving" once per client. Measured on the Pi on 2026-09-07, serving
two clients cost the tick under a millisecond and left its ~100 ms dt where it
was; a viewer that stalls is dropped by the feed's thread, not waited for by
the car. The same `F` and `D` text goes to `/tmp/bibo-scan.txt` each tick,
rewritten whole through a rename, though nothing reads it now; `--no-feed`
turns the feed and the file off together, and the file is removed at exit.

**One address, always answered: 8011 is scanfeed's, and it hands over.**
Slamtec's SDK holds the serial port, so only one program can have the lidar.
scanfeed is built for that: with no client it keeps the lidar CLOSED and the
port free, opens it for the first client and parks it (motor off, port
closed) when the last one leaves. The rule while the car drives, in three
lines:

- The service always answers on 8011. An idle scanfeed still holds the port,
  so a pilot started beside the systemd unit serves its feed on **8012**
  (`scanwire::PILOT_PORT`) and logs `feed: port 8011 is taken ... serving on
  8012`. Nobody needs root to make room for it.
- While the pilot drives, the service RELAYS to it. A client connecting to
  8011 makes scanfeed try the lidar; the open is refused because another
  program holds the port (`lidar::Refusal::REFUSAL_HELD` - a value, so the
  wording of the reason is nobody's contract), and scanfeed connects that
  client through to `127.0.0.1:8012`. The pilot's greeting, `F` lines and `D`
  lines arrive as they are; `MOTOR 0` goes to the pilot, which answers `MOTOR
  1` as it always did. Each client gets its own connection to the pilot, so
  the pilot's feed is the one dropping a slow viewer, as before. scanfeed's
  log says `relay to 127.0.0.1:8012 for client ... started`.
- When the pilot stops, viewers reconnect and the service opens the lidar
  itself. The pilot's feed closes, the relay closes each client with it, the
  hub's retry reconnects within a second, and this time the open succeeds:
  `F` lines without `D` lines, `MOTOR 0` obeyed, the device parked when the
  last viewer leaves. No unit is stopped or started by anyone.

Measured on the Pi on 2026-09-07 with the real C1 and `pilot --dry`: two
clients through the relay saw the pilot's `D` lines at ~10/s, both were
closed the moment the pilot exited, and a reconnect 1 s later was served by
scanfeed's own device. The other direction is unchanged: running `pilot`
while somebody is watching scanfeed's own device fails with `another program
has /dev/ttyUSB0` and exits 1 - the viewer got there first. A port held by
something that is NOT a pilot (lidar_probe, say) fails the relay's connect
and the client gets the `ERR another program has /dev/ttyUSB0` line it always
got. A pilot started with no scanfeed running (a laptop, or the service not
up) serves 8011 directly. A client too slow to take a frame (half a second
behind) is dropped by either program rather than allowed to stall the others.

Built only with the SDK, next to `pilot` and `lidar_probe`. On the board it
runs as the systemd unit `bibo-scanfeed`, installed by
`tools/status/install.sh` (see "Installing the board services") and up
whenever the board is powered, on any network. The unit points at
`~jack/build-pilot-app/scanfeed` and the installer says so if that has not
been built yet. By hand: `build-pilot/scanfeed [/dev/ttyUSB0]`, one log line
per event on stdout; SIGINT or SIGTERM parks the device and exits 0.

The first run of `pilot --dry --seconds 12` against the real C1 was on the
Orange Pi on 2026-09-06, with the Pico still on the laptop: the board saw its
room and decided, ten times a second. Driving the car is the next milestone
and it waits on one cable moving.
