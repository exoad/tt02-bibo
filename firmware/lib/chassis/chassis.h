/* ---------------------------------------------------------------------------
 * chassis - the two outputs that move this car.
 *
 * The steering servo on GP0 and the ESC on GP1. These are the only things on
 * the vehicle that can break something, so the safety lives HERE rather than
 * in whatever is calling: a console, a sketch and an autonomy loop must not
 * each carry their own copy of "refuse throttle until armed", because the day
 * one of them forgets is the day it matters.
 *
 * ---- what a caller gets ----------------------------------------------------
 *
 * Fractions, not microseconds. driveSteer(-1..+1) maps through the measured
 * calibration in cal.h, with each side scaled separately, so nothing above this
 * file needs to know that this car throws 254 us one way from centre and 186
 * the other. The raw microsecond entry points exist for calibrating and are
 * named so it is obvious you are below the abstraction.
 *
 * Every entry point that can refuse returns Bool. It never prints: what to say
 * about a refusal is the caller's business, and a library that printf'd into
 * somebody's protocol would be unusable from the one place that matters.
 *
 * ---- the three rules -------------------------------------------------------
 *
 * 1. THE STEERING IS RELEASED AT BOOT. Not neutral - released, no pulse at all.
 *    Driving neutral the instant USB power arrives assumes 1500 us is a safe
 *    place for the linkage to be, and on a car whose horn is a tooth off its
 *    spline it is not. The ESC still gets neutral immediately, because it is
 *    listening for exactly that to come up disarmed and an ESC fed no pulse
 *    sits there beeping about a lost signal.
 *
 * 2. THE ESC IS DISARMED UNTIL ASKED. Every throttle command is refused until
 *    driveArm(true), and disarming walks back to neutral. One deliberate act
 *    between a slider and a moving car.
 *
 * 3. NOTHING JUMPS. Calls set a TARGET; drivePump() walks the output toward it
 *    at a bounded rate. A slider dragged end to end produces a sweep rather
 *    than a step, which is the difference between a servo turning and a servo
 *    being hit. driveStop() is the exception, on purpose.
 *
 * ---- before the ESC is ever armed, from docs/wiring.md ---------------------
 *
 *   - Common ground between the Pico and the ESC is REQUIRED. Signal and
 *     ground cross between two power domains; without the shared return the
 *     ESC and the servo see noise, and that presents as erratic behaviour
 *     rather than as no behaviour. Check the breadboard rails are bridged -
 *     they are split in the middle and it does not look like it.
 *   - NEVER connect the BEC 5 V to the Pico while USB is attached.
 *   - Put the car on a stand. A wheel on the ground turns a test into a
 *     departure.
 *
 * ---- one copy ---------------------------------------------------------------
 *
 * The state below is file-scope, so including this in two translation units
 * would give you two chassis and one car. Each firmware image is a single
 * translation unit, which is why that is fine and why it is written down.
 * ------------------------------------------------------------------------- */
#pragma once

#include "../hal.h"
#include "cal.h"

/* ---- pins ---------------------------------------------------------------- */

#define PIN_SERVO 0
#define PIN_ESC   1

/* ---- bounds -------------------------------------------------------------- */

/*
 * Defaults come from the calibration, which is a measurement of THIS car.
 * If nobody has calibrated yet, cal.h carries a guess and says so in its stamp.
 */
#define SERVO_DEFAULT_MIN STEER_CAL_LEFT
#define SERVO_DEFAULT_MAX STEER_CAL_RIGHT

/*
 * The HARD bound. Nothing widens past this, whatever is asked - it is the
 * servo's own specification, and beyond it the pulse means nothing at all.
 */
#define SERVO_HARD_MIN 1000
#define SERVO_HARD_MAX 2000

/*
 * Forward only, and barely. 1500 is neutral; 1600 is a crawl on a bench. The
 * reverse half is not offered - a Hobbywing QuicRun needs a brake-then-reverse
 * sequence and getting that wrong on a stand is how a gearbox meets a
 * workbench. Reverse stays unreachable even by widening: finding a steering end
 * stop is careful work, discovering reverse by accident is not the same kind of
 * experiment.
 */
#define ESC_DEFAULT_MIN THROTTLE_CAL_MIN
#define ESC_DEFAULT_MAX THROTTLE_CAL_MAX
#define ESC_HARD_MIN    1500
#define ESC_HARD_MAX    1700

#define DRIVE_NEUTRAL_US 1500

/*
 * How fast an output is allowed to move: microseconds of pulse per tick.
 *
 * 8 us every 20 ms is 400 us per second. This car's steering travel is 440 us
 * (1230 to 1670), so lock to lock takes 1.1 SECONDS.
 *
 * The comment here used to say "about half a second", which was wrong by a
 * factor of two and had been wrong since it was written - 400 us of travel at
 * 400 us per second is one second, not half of one.
 *
 * That is roughly a TENTH of what the servo can do. A Power HD 1501MG is
 * 0.14 s per 60 degrees, and 440 us is around 44 degrees of shaft, so the
 * hardware would place it in about 0.1 s.
 *
 * The limit is deliberate and it is right for a bench: a slider dragged end to
 * end should sweep rather than fling a servo into a stop, and rule 3 above says
 * so. It is WRONG FOR DRIVING. A car that needs 1.1 s to go lock to lock cannot
 * make an avoidance correction, and the reactive layer will need this raised -
 * probably to a value set at runtime rather than baked in here, since bench
 * work and driving want different answers and neither is a bug.
 */
#define SLEW_STEP_US SLEW_CAL_STEP
#define SLEW_TICK_MS 20

/*
 * The bounds on that rate.
 *
 * 1 us/tick is 50 us/s - a full traverse in nine seconds, which is slower than
 * anyone wants but is a legitimate thing to ask for while watching a linkage.
 *
 * 200 us/tick is 10000 us/s: this car's whole travel in 44 ms, which is faster
 * than the servo can physically follow. That is the right ceiling - the limit
 * should stop being the software's before it stops being the hardware's, so
 * that "as fast as it goes" means the servo and not this file.
 */
#define SLEW_MIN_STEP 1
#define SLEW_MAX_STEP 200

/* ---- what the caller can see -------------------------------------------- */

/*
 * A snapshot, taken all at once.
 *
 * Returned by value rather than exposed as globals so a caller cannot read
 * `servoNow` from one moment and `servoTarget` from the next and report a car
 * that never existed.
 */
typedef struct
{
    Int32 servoUs;       /* what is being OUTPUT, which lags the target       */
    Int32 servoTargetUs; /* what it is heading toward                         */
    Int32 escUs;
    Int32 escTargetUs;
    Bool  escArmed;
    Bool  servoLive;     /* is the steering pin being driven at all           */
    Int32 centerUs;      /* where the wheels point straight, measured         */
    Int32 steerMilli;    /* the target as -1000..1000 of this car's travel    */

    /* Where the wheels ACTUALLY are, on the same scale.
     *
     * Not the same thing as steerMilli and the difference is the slew limiter:
     * a full-lock command arrives at once and the servo takes about a second to
     * walk there, so for a whole second these two disagree. Anything watching
     * the car - an indicator lamp, a controller - wants this one; anything
     * reporting what was asked for wants the other. */
    Int32 steerNowMilli;
    Int32 servoMinUs;
    Int32 servoMaxUs;
    Int32 escMinUs;
    Int32 escMaxUs;
    Int32 slewStepUs;    /* us of pulse per 20 ms tick */
} DriveState;

/* ---- state --------------------------------------------------------------- */

static Bool  driveUp    = false;
static Bool  escArmed   = false;
static Bool  servoLive  = false;

/*
 * The working limits, widened only on purpose. They start at the calibration
 * and are raised a little at a time while watching the linkage, which is how an
 * end stop is FOUND. Guessing them from a datasheet gets you a number the
 * linkage has never heard of.
 */
static Int32 servoMin = SERVO_DEFAULT_MIN;
static Int32 servoMax = SERVO_DEFAULT_MAX;
static Int32 escMin   = ESC_DEFAULT_MIN;
static Int32 escMax   = ESC_DEFAULT_MAX;

/*
 * Where the wheels actually point straight.
 *
 * DRIVE_NEUTRAL_US is the middle of the SERVO's range and has nothing to say
 * about the CAR's. The horn only fits its spline at whole-tooth intervals, so
 * straight-ahead lands wherever it lands - and treating 1500 as centre is how a
 * servo comes to lean on a frame at what everyone is calling neutral.
 */
static Int32 servoCenterUs = STEER_CAL_CENTER;

static Int32 servoTarget = STEER_CAL_CENTER;
static Int32 servoNow    = STEER_CAL_CENTER;
static Int32 escTarget   = DRIVE_NEUTRAL_US;
static Int32 escNow      = DRIVE_NEUTRAL_US;

/*
 * How fast an output may move, in microseconds per tick.
 *
 * Runtime, because the right answer changes with what you are doing: slow while
 * finding an end stop with the horn off, fast while driving. Baking it in meant
 * the steering crawled to wherever a slider was dragged, which reads as lag in
 * the UI and is a real limit on the car.
 */
static Int32 slewStepUs = SLEW_STEP_US;

static absolute_time_t nextSlew;

/* ---- helpers ------------------------------------------------------------- */

static inline Int32 driveClamp(Int32 v, Int32 lo, Int32 hi)
{
    if(v < lo)
    {
        return lo;
    }
    if(v > hi)
    {
        return hi;
    }
    return v;
}

/*
 * Steering as a fraction of THIS car's travel: -1 is full lock one way, +1 is
 * full lock the other, 0 is wheels straight.
 *
 * The two sides are scaled separately, and that is the entire point. The
 * linkage is not symmetric and no linkage ever is. Code that adds microseconds
 * to a midpoint therefore steers further one way than the other, and a car that
 * pulls left every time it is asked for "half" is a bug that hides for a long
 * time because every individual command looks reasonable.
 */
static inline Int32 driveSteerToUs(Float32 n)
{
    if(n < -1.0f)
    {
        n = -1.0f;
    }
    if(n > 1.0f)
    {
        n = 1.0f;
    }

    /* A centre sitting on top of an end is not a range to interpolate across.
     * It happens while limits are being narrowed, and it must not divide. */
    const Int32 lo = servoCenterUs - servoMin;
    const Int32 hi = servoMax - servoCenterUs;

    if(n < 0.0f)
    {
        return servoCenterUs + (Int32) (n * (Float32) ((lo > 0) ? lo : 0));
    }
    return servoCenterUs + (Int32) (n * (Float32) ((hi > 0) ? hi : 0));
}

/* The inverse, in THOUSANDTHS so it can be reported without a float formatter
 * having to survive on a microcontroller. -1000 to +1000. */
static inline Int32 driveSteerFromUs(Int32 us)
{
    const Int32 d = us - servoCenterUs;
    if(d == 0)
    {
        return 0;
    }
    if(d < 0)
    {
        const Int32 lo = servoCenterUs - servoMin;
        return (lo > 0) ? ((d * 1000) / lo) : 0;
    }
    const Int32 hi = servoMax - servoCenterUs;
    return (hi > 0) ? ((d * 1000) / hi) : 0;
}

/* ---- lifecycle ----------------------------------------------------------- */

static inline Void driveOpen(Void)
{
    servoOpen(PIN_SERVO);
    servoOpen(PIN_ESC);

    /* Rule 1: released, not neutral. */
    servoRelease(PIN_SERVO);
    servoLive = false;
    servoWriteUs(PIN_ESC, DRIVE_NEUTRAL_US);

    nextSlew = make_timeout_time_ms(SLEW_TICK_MS);
    driveUp  = true;
}

/* Walks each output toward its target. Call from the main loop, often. */
static inline Void drivePump(Void)
{
    if(!driveUp || !time_reached(nextSlew))
    {
        return;
    }
    nextSlew = make_timeout_time_ms(SLEW_TICK_MS);

    if(servoLive && servoNow != servoTarget)
    {
        const Int32 d    = servoTarget - servoNow;
        const Int32 step = (d > slewStepUs) ? slewStepUs
                         : ((d < -slewStepUs) ? -slewStepUs : d);
        servoNow += step;
        servoWriteUs(PIN_SERVO, (UInt32) servoNow);
    }

    /* A disarmed ESC is walked back to neutral rather than snapped there: a
     * step to neutral from a moving throttle is itself a jolt. */
    const Int32 want = escArmed ? escTarget : DRIVE_NEUTRAL_US;
    if(escNow != want)
    {
        const Int32 d    = want - escNow;
        const Int32 step = (d > slewStepUs) ? slewStepUs
                         : ((d < -slewStepUs) ? -slewStepUs : d);
        escNow += step;
        servoWriteUs(PIN_ESC, (UInt32) escNow);
    }
}

/*
 * The ESC disarmed and neutral, and the steering RELEASED.
 *
 * Released rather than centred, and that distinction is the whole point.
 * Centre is only a safe place to put a servo if it happens to be where the
 * linkage wants to sit; if the horn is a tooth off its spline it is not, and
 * "stop" would then mean "keep pushing, just somewhere else". Nothing to push
 * with is the only stop that is a stop on every car.
 *
 * Immediate, not slewed. A stop that eases in is not a stop.
 */
static inline Void driveStop(Void)
{
    escArmed    = false;
    escTarget   = DRIVE_NEUTRAL_US;
    escNow      = DRIVE_NEUTRAL_US;
    servoTarget = servoCenterUs;
    servoNow    = servoCenterUs;
    servoLive   = false;

    if(driveUp)
    {
        servoWriteUs(PIN_ESC, DRIVE_NEUTRAL_US);
        servoRelease(PIN_SERVO);
    }
}

static inline DriveState driveRead(Void)
{
    DriveState s;
    s.servoUs       = servoNow;
    s.servoTargetUs = servoTarget;
    s.escUs         = escNow;
    s.escTargetUs   = escTarget;
    s.escArmed      = escArmed;
    s.servoLive     = servoLive;
    s.centerUs      = servoCenterUs;
    s.steerMilli    = driveSteerFromUs(servoTarget);
    s.steerNowMilli = driveSteerFromUs(servoNow);
    s.servoMinUs    = servoMin;
    s.servoMaxUs    = servoMax;
    s.escMinUs      = escMin;
    s.escMaxUs      = escMax;
    s.slewStepUs    = slewStepUs;
    return s;
}

/*
 * How fast the outputs may move, in microseconds of pulse per 20 ms tick.
 *
 * Clamped rather than refused, so a caller asking for "as fast as possible" by
 * passing a large number gets the ceiling instead of an error. Returns false
 * only for a value that is not a rate at all.
 */
static inline Bool driveSetSlew(Int32 usPerTick)
{
    if(usPerTick <= 0)
    {
        return false;
    }
    slewStepUs = driveClamp(usPerTick, SLEW_MIN_STEP, SLEW_MAX_STEP);
    return true;
}

/* ---- steering ------------------------------------------------------------ */

/*
 * Starts or stops driving the steering pin.
 *
 * Engaging picks up from the CAR's centre and slews to wherever the target
 * already is, rather than jumping: the servo has been limp and its actual
 * position is unknown, so the first command after engaging is the one most
 * likely to be a surprise.
 */
static inline Void driveEngage(Bool on)
{
    if(on && !servoLive)
    {
        servoNow  = servoCenterUs;
        servoLive = true;
        servoWriteUs(PIN_SERVO, (UInt32) servoNow);
        return;
    }
    if(!on && servoLive)
    {
        servoLive = false;
        servoRelease(PIN_SERVO);
    }
}

/* Steer as a fraction of this car's travel. THE entry point for driving. */
static inline Void driveSteer(Float32 n)
{
    servoTarget = driveClamp(driveSteerToUs(n), servoMin, servoMax);
}

/* Wheels straight, wherever that measures out to be. */
static inline Void driveCenter(Void)
{
    servoTarget = driveClamp(servoCenterUs, servoMin, servoMax);
}

/*
 * Raw microseconds. For CALIBRATING - finding where the ends and the centre
 * actually are - not for driving. Clamped rather than refused: a slider that
 * stops moving at the limit is clearer than one that silently does nothing.
 */
static inline Void driveSteerUs(Int32 us)
{
    servoTarget = driveClamp(us, servoMin, servoMax);
}

/*
 * Moves where "centre" is.
 *
 * Clamped into the working range, because a centre outside the limits is one
 * the servo can never be commanded to - driveCenter() would silently mean
 * something else, which is worse than refusing.
 */
static inline Void driveTrim(Int32 us)
{
    servoCenterUs = driveClamp(us, servoMin, servoMax);
}

/*
 * Widens or narrows the working range. False if the two are the wrong way
 * round; the caller decides what to say about that.
 *
 * Clamped to the hard bound, and both the target and the centre are pulled back
 * inside so narrowing can never leave an output sitting outside its own limits.
 */
static inline Bool driveSetSteerLimits(Int32 lo, Int32 hi)
{
    if(lo >= hi)
    {
        return false;
    }

    /*
     * Checked AFTER clamping, not only before.
     *
     * `SERVOLIMITS 1 2` passes the ordering test - 1 is genuinely below 2 - and
     * then both clamp to SERVO_HARD_MIN and the range is 1000 to 1000. The
     * steering can no longer be commanded anywhere, driveSteerToUs divides a
     * span of zero, and the reply reports a car that looks configured. Found on
     * the board rather than by reading this, which is the only reason it is
     * here: it needs two plausible numbers to reach.
     */
    const Int32 lo2 = driveClamp(lo, SERVO_HARD_MIN, SERVO_HARD_MAX);
    const Int32 hi2 = driveClamp(hi, SERVO_HARD_MIN, SERVO_HARD_MAX);
    if(lo2 >= hi2)
    {
        return false;
    }

    servoMin      = lo2;
    servoMax      = hi2;
    servoTarget   = driveClamp(servoTarget, servoMin, servoMax);
    servoCenterUs = driveClamp(servoCenterUs, servoMin, servoMax);
    return true;
}

/* ---- throttle ------------------------------------------------------------ */

static inline Void driveArm(Bool on)
{
    escArmed  = on;
    escTarget = DRIVE_NEUTRAL_US;
}

/* False when the ESC is not armed. Rule 2, and it lives here so no caller can
 * forget it. */
static inline Bool driveThrottleUs(Int32 us)
{
    if(!escArmed)
    {
        return false;
    }
    escTarget = driveClamp(us, escMin, escMax);
    return true;
}

static inline Void driveThrottleNeutral(Void)
{
    escTarget = DRIVE_NEUTRAL_US;
}

static inline Bool driveSetThrottleLimits(Int32 lo, Int32 hi)
{
    if(lo >= hi)
    {
        return false;
    }

    /* The same collapse as the steering, and worse here: the throttle's hard
     * band is only 200 us wide, so any pair below 1500 lands on 1500/1500. */
    const Int32 lo2 = driveClamp(lo, ESC_HARD_MIN, ESC_HARD_MAX);
    const Int32 hi2 = driveClamp(hi, ESC_HARD_MIN, ESC_HARD_MAX);
    if(lo2 >= hi2)
    {
        return false;
    }

    escMin    = lo2;
    escMax    = hi2;
    escTarget = driveClamp(escTarget, escMin, escMax);
    return true;
}
