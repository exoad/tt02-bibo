/*
 * cal - the car's compiled-in defaults, maintained by hand. The Pico boots on
 * these; each time the pilot opens the port it replays the operator's saved trim
 * (trim.txt, set from the viewer's Trim pane) over them.
 */
#pragma once

/*
 * Steering pulses measured on this TT-02: full lock one way, straight ahead, full
 * lock the other.
 */
#define STEER_CAL_LEFT 1230
#define STEER_CAL_CENTER 1480
#define STEER_CAL_RIGHT 1660
#define STEER_CAL_STAMP "measured 2026-08-29"

/*
 * The forward throttle band; MIN is idle, the last pulse before the motor turns.
 * Not re-measured for the QuicRun 10BL160 G2 and 21.5T brushless, which map
 * neutral to full almost linearly, so MIN may creep and MAX is no longer a crawl.
 * The band stays because it is too narrow to launch the car; re-measure on a
 * stand before widening it.
 */
#define THROTTLE_CAL_MIN 1541
#define THROTTLE_CAL_MAX 1600
#define THROTTLE_CAL_STAMP "1060 brushed, 2026-08 - superseded, re-measure"

/*
 * us per SLEW_TICK_MS tick. A choice, not a measurement: the steering takes about
 * a second lock to lock, which suits a bench and is too slow to steer around
 * anything.
 */
#define SLEW_CAL_STEER    8
#define SLEW_CAL_THROTTLE 8

/*
 * main.cxx's watchdog: how long the Pico obeys the last throttle with no valid
 * command. The pilot's bibowire::PICO_DEADMAN_MS is this number.
 */
#define BIBO_WATCHDOG_MS 200
