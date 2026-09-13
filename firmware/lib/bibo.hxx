/*
 * bibo - the firmware library, in one include.
 *
 * app/main.cxx includes this and nothing else from lib/; tools/style_audit.py
 * fails the build when it reaches past it. Everything is header-only, so a call
 * such as bibo::drive::pump() compiles down to the stores it makes.
 *
 *   shared.hxx           the vocabulary: Int32, Bool, Void, CharSeq. Global,
 *                        not in namespace bibo.
 *   hal.hxx              the board: timing, serial, board, pwm, servo, led.
 *   text.hxx             text:: - parsing and formatting on a caller's buffer.
 *   pins.hxx             pins:: - the car's pin map, checked at compile time.
 *   status.hxx           status:: - the onboard LED as a heartbeat.
 *   chassis/cal.hxx      the car's compiled-in defaults, as macros.
 *   chassis/chassis.hxx  drive:: - steering and throttle, and their safety rules.
 *
 * Includes point downward only: hal knows nothing above it, chassis knows hal,
 * the pin map and cal, and the app knows only this file. Every include in lib/
 * is relative to the file that writes it ("../hal.hxx" from chassis/), so an
 * editor without the CMake project loaded still resolves it.
 */
#pragma once

#include "hal.hxx"
#include "text.hxx"
#include "pins.hxx"
#include "status.hxx"

#include "chassis/cal.hxx"
#include "chassis/chassis.hxx"
#include "shared.hxx"
