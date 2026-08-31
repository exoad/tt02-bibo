# pilot

The companion board's program. Stubs, ahead of hardware that is not here yet.

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
    src/link.hxx       the transport to the car. Declared, and REFUSES: there is
                       no implementation for any platform this compiles on.
    src/autonomy.hxx   one tick of the driving. Tunings are real; step() returns
                       STATUS_NOT_IMPLEMENTED and touches no output.

`proto` is finished first because it is the part that can be finished. It is
pure string work, so it is provable on a laptop months before the board arrives,
and it is where the bugs in a text protocol actually live.

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

    tests\build_proto_test.bat run
    tests\build_pilot_test.bat run

Both compile with MSVC on a laptop. There is no Pi build yet because there is no
Pi, and no serial implementation to build for it - see `src/link.hxx`, which says
so in its own words rather than returning success.
