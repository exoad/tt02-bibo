/*
 * bibo - the firmware library, in one include. app/ includes this and nothing
 * else from lib/; tools/style_audit.py fails the build when it reaches past it.
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
#include "hall.hxx"
#include "encoder.hxx"
#include "chassis/cal.hxx"
#include "chassis/chassis.hxx"
#include "shared.hxx"
