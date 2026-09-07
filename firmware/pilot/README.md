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
    app/main.cxx       the program, `pilot`. Grabs a revolution, runs
                       reactive::step with the measured dt, sends STEER and
                       ESC (or NEUTRAL) to the car; sends anyway after 200 ms
                       without a revolution so the board's 400 ms deadman is
                       never what stops the car. `--dry` decides without a
                       Pico. Built only with the SDK.
    tools/lidar_probe  is the lidar there and what does it see. Run it first.

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

    pilot [--lidar PORT] [--pico PORT] [--dry] [--arm] [--forward DEG] [--seconds N]

The lidar is `/dev/ttyUSB0` and the Pico `/dev/ttyACM0` unless told otherwise.
`--forward` is the raw lidar angle that points along the car - a mounting
fact, not a tuning, and 0 is an assumption until it is measured. `--arm` is
what lets the car move: without it the board refuses every throttle pulse and
the program prints each refusal, which is the correct behaviour for a car that
was not meant to go anywhere. Ctrl-C sends STOP, stops the motor, and exits 0.
A timed run that saw no revolution at all exits 1.

### Reaching the board

Outdoors the board joins the phone's hotspot (`WhoopWhoop`, hidden) by itself
at boot; join the laptop to the same hotspot and the two are on one LAN with
nothing in between. The hotspot hands out addresses, so use the name: `ssh
jack@bibobox.local` once mDNS is enabled on the board, or the tailnet address
`jack@bibobox` (100.125.100.51) whenever the phone has data. docs/conventions.md
"Link" has the whole picture.

### The status page

`tools/status/` is the car's one URL: `http://bibobox.local/` on the phone,
walking behind the car. Plain text, refreshed every two seconds: CPU
temperature, the pilot's last second (lidar rev/s, mode, clearance; whether the
Pico has been heard), and honest absences for what nothing measures yet
(battery, localization). The pilot writes `/tmp/bibo-pilot.json` once a second
and the page reads it, because the pilot holds the lidar's port and nothing
else may open it; a file older than three seconds reads as "pilot not running".

    sudo sh ~/tt02-bibo/firmware/pilot/tools/status/install.sh

installs a systemd unit and a NetworkManager dispatcher hook, so the page
starts when the board joins `WhoopWhoop` and stops when it leaves, and turns
mDNS on - both systemd-resolved's global switch and the profile's, since on
this Ubuntu the second cannot exceed the first. Run it again after a pull; it
is idempotent. `BIBO_STATUS_PORT=8080 python3 status_server.py` runs the page
by hand, on any network, without root.

The first run of `pilot --dry --seconds 12` against the real C1 was on the
Orange Pi on 2026-09-06, with the Pico still on the laptop: the board saw its
room and decided, ten times a second. Driving the car is the next milestone
and it waits on one cable moving.
