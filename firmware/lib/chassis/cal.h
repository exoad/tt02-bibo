/* ---------------------------------------------------------------------------
 * Steering calibration - GENERATED.
 *
 * Written by the hub's Drive view. Edit it THERE, not here: the next "Write to
 * firmware" overwrites this file completely, and a number typed in by hand is
 * gone the first time anyone touches the calibration UI.
 *
 * These are measurements of one particular car, not a datasheet. A servo's own
 * range is 1000-2000 us; what a TT-02's steering can actually reach is narrower
 * and off-centre, because the horn only fits the spline at whole-tooth
 * intervals and the linkage is whatever length it is. There is no way to know
 * these numbers except by moving the servo and watching.
 *
 * CENTER is the interesting one. 1500 us is the middle of the servo's range and
 * has nothing to say about where a car's wheels point straight - assuming it
 * does is how a servo ends up leaning on a frame at "neutral".
 * ------------------------------------------------------------------------- */
#pragma once

/* Full lock one way. */
#define STEER_CAL_LEFT 1230

/* Wheels straight ahead. Not necessarily 1500, and usually not. */
#define STEER_CAL_CENTER 1484

/* Full lock the other way. */
#define STEER_CAL_RIGHT 1670

/* ---- throttle ------------------------------------------------------------
 *
 * The working range for the ESC, and the reason this section exists: the
 * steering has been persisted here since it was measured, and the throttle was
 * not. Anything set with ESCLIMITS lived in RAM and was silently back to
 * 1500-1600 after the next reboot or reflash - which is not a calibration, it
 * is a setting you have to remember to make again.
 *
 * Still forward-only. The board refuses anything below 1500 whatever is written
 * here; reverse needs a brake-then-reverse sequence and is not something to
 * reach by editing a number.
 */
#define THROTTLE_CAL_MIN 1500
#define THROTTLE_CAL_MAX 1600

/* When these were measured, and by whom, so a stale calibration can be spotted
 * rather than trusted. "defaults" means nobody has calibrated this car yet. */
#define STEER_CAL_STAMP "measured 2026-08-27"
