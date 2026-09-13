/*
 * ---------------------------------------------------------------------------
 * cal - the car's compiled-in defaults, maintained by hand.
 *
 * The Pico boots on these. Every time the pilot opens the port it replays the
 * operator's saved trim (trim.txt, set from the viewer's Trim pane) over them,
 * so on the car they hold only until the pilot connects.
 *
 * The steering numbers are measurements of this TT-02, not a datasheet: the
 * servo horn fits its spline only at whole teeth, so the wheels point straight
 * wherever they point straight, which is not 1500 us.
 * -------------------------------------------------------------------------
 */
#pragma once

/* Steering pulse at full lock one way, straight ahead, and full lock the other. */
#define STEER_CAL_LEFT 1230
#define STEER_CAL_CENTER 1480
#define STEER_CAL_RIGHT 1660
#define STEER_CAL_STAMP "measured 2026-08-29"

/*
 * The forward throttle band. MIN is idle: the last pulse before the motor turns.
 *
 * Measured on the brushed 1060 and 540, both gone. The QuicRun 10BL160 G2 with
 * the 21.5T brushless maps 1500..2000 almost linearly, so 1541 probably creeps
 * and 1600 is no longer a crawl. The numbers stay because the band is too narrow
 * to launch the car; re-measure on a stand before widening it.
 */
#define THROTTLE_CAL_MIN 1541
#define THROTTLE_CAL_MAX 1600
#define THROTTLE_CAL_STAMP "1060 brushed, 2026-08 - superseded, re-measure"

/*
 * How fast each output may move, in microseconds of pulse per SLEW_TICK_MS tick
 * (chassis.hxx). A choice rather than a measurement: at this rate the steering
 * takes about a second lock to lock, which suits a bench and is too slow to
 * steer around anything.
 */
#define SLEW_CAL_STEER    8
#define SLEW_CAL_THROTTLE 8

/*
 * The Pico's watchdog: how long it keeps obeying the last throttle with no
 * valid command before it puts the ESC at neutral. The pilot's
 * bibowire::PICO_DEADMAN_MS is this number.
 */
#define BIBO_WATCHDOG_MS 200
