/* ---------------------------------------------------------------------------
 * bibo - the whole library, in one include.
 *
 * Application code includes THIS and nothing else from the library. Reaching
 * past it to a specific header still compiles, and it is still wrong: it makes
 * every file's dependencies a thing you have to read the top of the file to
 * know, and it means a header that moves breaks callers that had no business
 * naming it. tools/style_audit.py fails the build on it.
 *
 * Everything here is header-only. That is deliberate on a microcontroller: the
 * compiler sees through a call to bibo::gpio::write() and emits the single
 * store it actually is, where a real function call would be a branch, a
 * register spill and a return for the sake of tidiness nobody can measure. The
 * cost is that including two library headers in one translation unit costs
 * compile time rather than link time, which at this size is free.
 *
 * ---- the layers ------------------------------------------------------------
 *
 *   types.hxx            the vocabulary: Int32, Bool, Void, Utf8, CharSeq.
 *                        NOT in namespace bibo - it is the spelling the whole
 *                        project uses, the hub included.
 *   hal.hxx              the board: gpio, pwm, i2c, spi, serial, led, radio,
 *                        adc, watchdog, timing, board. Nothing above knows
 *                        which pins exist.
 *   text.hxx             text:: - the string handling this project does.
 *                        Stateless.
 *   status.hxx           status:: - the onboard LED as something readable
 *                        across a room.
 *   lights.hxx           lights:: - the lamps, and which pin each is on.
 *                        Output only.
 *   cue.hxx              cue:: - what the car SAYS: indicating, braking, a
 *                        headlight flash. Decides; lights emits.
 *   net.hxx              net:: - the same command link, over Wi-Fi.
 *
 *   drivers/display.hxx  tft:: - an ST7789 / ST7735 panel over SPI. THE PANEL:
 *                        its size, its pads, and the things that are true of
 *                        glass - inversion, sleep, backlight brightness. Owns
 *                        a tft::Screen and nothing about drawing.
 *   gfx.hxx              gfx:: - the 2D layer. gfx::open() is handed a panel
 *                        and returns a gfx::Canvas, which owns the back
 *                        buffer, the clip and the text state, and which every
 *                        shape is drawn onto. Knows shapes, not panels.
 *
 *                        Use gfx to draw a frame; reach for tft when you want
 *                        the hardware itself.
 *   drivers/range.hxx    tof:: - a VL53L1X time-of-flight sensor over I2C.
 *   drivers/storage.hxx  sd:: - an SD card over SPI.
 *
 *   chassis/cal.hxx      this car's measured numbers. GENERATED - written by
 *                        the hub's Drive view, not by hand. Macros, so not in
 *                        a namespace: the preprocessor has finished before C++
 *                        has heard of one.
 *   chassis/chassis.hxx  drive:: - steering and throttle, in fractions rather
 *                        than microseconds. The only thing that reads cal.
 *
 * The dependency direction is strictly downward: hal knows nothing, drivers
 * know hal, chassis knows hal and the calibration, and applications know only
 * this file. A driver that needed another driver would be two things wearing
 * one name.
 *
 * ---- naming ----------------------------------------------------------------
 *
 * Everything is in namespace bibo, and inside it every module is a namespace of
 * its own - gpio, pwm, i2c, spi, serial, led, radio, adc, watchdog, timing,
 * board, text, status, lights, cue, net, tft, gfx, tof, sd, drive. So a call
 * site says which layer it reaches into without anyone having to look it up:
 *
 *     bibo::gpio::write(28, true);
 *     bibo::drive::stop();
 *
 * This was a PREFIX until the library became C++ - gpioWrite, driveStop - which
 * is the C answer to the same problem and reads as a prefix somebody remembered
 * rather than a boundary the compiler knows about. style_audit.py checks that
 * each module still declares its namespace, because a header that quietly stops
 * doing so still compiles: its symbols simply move to the global namespace, one
 * file at a time, which is exactly how the prefixes decayed before anything
 * checked them.
 *
 * A SKETCH may open it - `using namespace bibo;` - and firmware/scratch does.
 * One file, linking nothing else, whose whole purpose is to be the easy thing.
 * app/main.cxx does not.
 * ------------------------------------------------------------------------- */
/*
 * EVERY PROJECT INCLUDE IN THIS LIBRARY IS RELATIVE TO THE FILE THAT WRITES IT.
 *
 * "../hal.h" from lib/drivers/, not "hal.h". Uglier, and it resolves for a tool
 * that has loaded nothing: a quoted include is searched next to the including
 * file first, so a driver naming a header one directory up must say so. The
 * bare spelling needs -Ifirmware/lib, which comes from the CMake project - and
 * an editor that has not attached the project then underlines every include in
 * the library at once, which reads as broken code rather than as unconfigured
 * tooling.
 *
 * The include path is still set for both targets, so either spelling compiles.
 * This one also parses.
 */
#pragma once

#include "hal.hxx"
#include "text.hxx"

/* The car's pin map. Included before anything that binds a pin, so a subsystem
 * can name pins::HEAD_L rather than 11 - and so the conflict static_asserts in
 * it fire on every build rather than only when somebody happens to include it. */
#include "pins.hxx"

/* What the sounds on the card mean. A leaf like pins - names and numbers,
 * no SDK - so it can be read and tested without a board. */
#include "sfx.hxx"

/* The car's voice, asked for by name. Above the driver and above sfx, the way
 * cue sits above lights. */
#include "sound.hxx"

/* Drivetrain maths - PID, feedforward, and ticks to metres. Pure arithmetic,
 * no SDK, so both are tested on the host. */
#include "geom.hxx"
#include "kinematics.hxx"
#include "pursuit.hxx"
#include "control.hxx"
#include "chassis/odom.hxx"
#include "boot.hxx"

#include "drivers/dfplayer.hxx"
#include "drivers/display.hxx"
#include "gfx.hxx"
#include "status.hxx"
#include "lights.hxx"
#include "cue.hxx"
#include "net.hxx"
#include "drivers/range.hxx"
#include "drivers/storage.hxx"

#include "chassis/cal.hxx"
#include "chassis/chassis.hxx"
