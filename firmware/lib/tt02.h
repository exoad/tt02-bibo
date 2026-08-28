/* ---------------------------------------------------------------------------
 * tt02 - the whole library, in one include.
 *
 * Application code includes THIS and nothing else from the library. Reaching
 * past it to a specific header still compiles, and it is still wrong: it makes
 * every file's dependencies a thing you have to read the top of the file to
 * know, and it means a header that moves breaks callers that had no business
 * naming it. tools/style_audit.py fails the build on it.
 *
 * Everything here is header-only and `static inline`. That is deliberate on a
 * microcontroller: the compiler sees through a call to gpioWrite() and emits
 * the single store it actually is, where a real function call would be a
 * branch, a register spill and a return for the sake of tidiness nobody can
 * measure. The cost is that including two library headers in one translation
 * unit costs compile time rather than link time, which at this size is free.
 *
 * ---- the layers ------------------------------------------------------------
 *
 *   types.h          the vocabulary: Int32, Bool, Void, Utf8, CharSeq.
 *   hal.h            the board. GPIO, PWM, I2C, SPI, serial, the onboard LED,
 *                    sleeping. Nothing above knows which pins exist.
 *   text.h           the string handling this project does. Stateless.
 *   status.h         the onboard LED as something readable across a room.
 *
 *   tt02_display.h   an ST7789 / ST7735 panel over SPI. Owns a Screen.
 *   tt02_gfx.h       drawing INTO a Screen. Knows shapes, not panels.
 *   tt02_range.h     a VL53L1X time-of-flight sensor over I2C.
 *   tt02_storage.h   an SD card over SPI.
 *
 *   tt02_cal.h       this car's measured numbers. GENERATED - written by the
 *                    hub's Drive view, not by hand.
 *   tt02_chassis.h   steering and throttle, in fractions rather than
 *                    microseconds. The only thing that reads tt02_cal.h.
 *
 * The dependency direction is strictly downward: hal knows nothing, drivers
 * know hal, chassis knows hal and the calibration, and applications know only
 * this file. A driver that needed another driver would be two things wearing
 * one name.
 *
 * ---- naming ----------------------------------------------------------------
 *
 * Every public symbol carries its module's prefix - gpio, pwm, i2c, spi,
 * serial, led, tft, gfx, vl53, sd, drive, steer - so a call site says which
 * layer it is reaching into without anyone having to look it up. This is
 * enforced, not encouraged.
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.h"
#include "text.h"

#include "drivers/display.h"
#include "gfx.h"
#include "status.h"
#include "drivers/range.h"
#include "drivers/storage.h"

#include "chassis/cal.h"
#include "chassis/chassis.h"
