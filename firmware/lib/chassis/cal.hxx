/*
 * ---------------------------------------------------------------------------
 * Steering calibration - GENERATED.
 *
 * Written by the hub's Drive view. Edit it THERE, not here: the next "Write to
 * firmware" overwrites this file completely, and a number typed in by hand is
 * gone the first time anyone touches the calibration UI.
 *
 * These are measurements of one particular car, not a datasheet. A servo's own
 * range is 1000-2000 us; what a TT-02's steering can actually reach is narrower
 * and off-center, because the horn only fits the spline at whole-tooth
 * intervals and the linkage is whatever length it is. There is no way to know
 * these numbers except by moving the servo and watching.
 *
 * CENTER is the interesting one. 1500 us is the middle of the servo's range and
 * has nothing to say about where a car's wheels point straight - assuming it
 * does is how a servo ends up leaning on a frame at "neutral".
 * -------------------------------------------------------------------------
 */
#pragma once

/* Full lock one way. */
#define STEER_CAL_LEFT 1230

/* Wheels straight ahead. Not necessarily 1500, and usually not. */
#define STEER_CAL_CENTER 1480

/* Full lock the other way. */
#define STEER_CAL_RIGHT 1660

/*
 * ---- throttle ------------------------------------------------------------
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
 *
 * MIN is IDLE. Not the ESC's neutral and not a safety floor, but the pulse at
 * which this motor sits still and the next microsecond starts it turning. That
 * is a fact about this car's ESC and motor, found by winding it up until the
 * wheels moved - which is why it is not the round number anybody would guess.
 *
 * It matters that this is the floor the sliders are built from: a range
 * starting below idle spends its first stretch doing nothing at all, so the
 * control feels dead at one end for no reason a driver could work out.
 *
 * A GEAR CHANGE INVALIDATES THIS. Idle is where the motor overcomes the
 * drivetrain, so more reduction breaks static friction at a lower pulse and
 * this number goes down. The 17T pinion went on after this was measured at 19T,
 * and it has not been re-measured since - so 1541 is an upper bound on idle
 * rather than idle.
 *
 * That is not cosmetic. driveThrottleUs clamps UPWARD to escMin, and the
 * deadman calls the car driven when escTargetUs > escMinUs - so a car that
 * creeps at what this file calls idle is a car the deadman does not think is
 * moving.
 */
#define THROTTLE_CAL_MIN 1541
#define THROTTLE_CAL_MAX 1600

/*
 * ---- tuning, not measurement ---------------------------------------------
 *
 * Everything above is a fact about this car that was found by moving it. This
 * is not: it is a choice about how fast the outputs are allowed to move, and a
 * different answer is right for a bench than for driving.
 *
 * It lives here anyway for one reason - this is the file that survives a
 * reflash. The throttle range was runtime-only until 2026-08-27 and was
 * silently lost every time the board was rewritten, which is not a setting, it
 * is a setting you have to remember to make again.
 *
 * Microseconds of pulse per 20 ms tick. 8 is 400 us/s, which walks this car's
 * 430 us of steering travel in about a second - deliberate on a bench and far
 * too slow to steer around anything.
 */
#define SLEW_CAL_STEER    8

/*
 * The throttle's own rate. Separate from the steering because the right answer
 * is different: a servo should arrive promptly, an ESC should be led there.
 * Starts equal to the steering, which is what the single shared rate used to
 * give - so nothing changes until it is tuned.
 */
#define SLEW_CAL_THROTTLE 8

/*
 * ---- when the tail lamps go out ------------------------------------------
 *
 * Microseconds ABOVE idle at which the car counts as being driven, and the
 * tails extinguish. Below it the motor is turning but barely, and a car
 * crawling is a car that has not really pulled away - the lamp should still be
 * on.
 *
 * Also used the other way for reverse: more than this BELOW neutral counts as
 * being driven backwards.
 *
 * Tuning, not measurement, like the slew step above - it is a judgment about
 * when "moving" starts, and the honest answer is whatever looks right on the
 * car. It lives here for the same reason: this is the file that survives a
 * reflash.
 */
#define LIGHT_CAL_OFF_US 10

/*
 * When this car was last calibrated, so a stale set of numbers can be spotted
 * rather than trusted. "defaults" means nobody has calibrated this car yet.
 */
#define STEER_CAL_STAMP "measured 2026-08-29"
