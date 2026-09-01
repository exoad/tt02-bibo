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
 * Fractions, not microseconds. drive::steer(-1..+1) maps through the measured
 * calibration in cal.h, with each side scaled separately, so nothing above this
 * file needs to know that this car throws 250 us one way from centre and 180
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
 *    drive::arm(true), and disarming walks back to neutral. One deliberate act
 *    between a slider and a moving car.
 *
 * 3. NOTHING JUMPS. Calls set a TARGET; drive::pump() walks the output toward it
 *    at a bounded rate. A slider dragged end to end produces a sweep rather
 *    than a step, which is the difference between a servo turning and a servo
 *    being hit. drive::stop() is the exception, on purpose.
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

#include "../hal.hxx"
#include "../pins.hxx"
#include "cal.hxx"

namespace bibo::drive
{

    /* ---- pins ---------------------------------------------------------------- */

    /**
     * @brief GPIO pin driving the steering servo, and the ESC, from the
     *        active pin map.
     *
     * Read from the map THIS PROGRAM installed, not spelled here. These two
     * were the last literal GPIO numbers in the chassis, and they had to move
     * because "which pins are taken" was a question you answered by grepping -
     * which is how the DFPlayer nearly landed on the tail lamps.
     *
     * A read rather than a constant, so a sketch driving a servo on a different
     * pad is pins::begin() and nothing else. Both resolve to NONE until begin()
     * has run, and drive::open() then binds nothing rather than binding pad 0
     * because that is what an uninitialised map used to say.
     */
#define PIN_SERVO (pins::active().servo)
#define PIN_ESC   (pins::active().esc)

    /* ---- bounds -------------------------------------------------------------- */

    /**
     * @brief Startup steering limits, in microseconds of servo pulse.
     *
     * Defaults come from the calibration, which is a measurement of THIS car.
     * If nobody has calibrated yet, cal.h carries a guess and says so in its stamp.
     */
#define SERVO_DEFAULT_MIN STEER_CAL_LEFT
#define SERVO_DEFAULT_MAX STEER_CAL_RIGHT

    /**
     * @brief The servo's absolute pulse-width limits, in microseconds.
     *
     * The HARD bound. Nothing widens past this, whatever is asked - it is the
     * servo's own specification, and beyond it the pulse means nothing at all.
     *
     * @warning setSteerLimits() clamps into this range; nothing above this
     *          file can command the steering pulse outside it.
     */
#define SERVO_HARD_MIN 1000
#define SERVO_HARD_MAX 2000

    /**
     * @brief Startup and absolute throttle limits, in microseconds of ESC
     *        pulse.
     *
     * Forward only, and barely. 1500 is neutral; 1600 is a crawl on a bench. The
     * reverse half is not offered - a Hobbywing QuicRun needs a brake-then-reverse
     * sequence and getting that wrong on a stand is how a gearbox meets a
     * workbench. Reverse stays unreachable even by widening: finding a steering end
     * stop is careful work, discovering reverse by accident is not the same kind of
     * experiment.
     *
     * @warning ESC_HARD_MIN/ESC_HARD_MAX are the absolute ceiling; nothing
     *          above this file can command the ESC outside them, and that is
     *          what keeps a stray value from ever reaching reverse.
     */
#define ESC_DEFAULT_MIN THROTTLE_CAL_MIN
#define ESC_DEFAULT_MAX THROTTLE_CAL_MAX
#define ESC_HARD_MIN    1500
#define ESC_HARD_MAX    1700

    /**
     * @brief The neutral ESC pulse, in microseconds.
     *
     * @warning Sent immediately at open() and by stop(), whether or not the
     *          ESC is armed - an ESC left with no pulse at all sits there
     *          beeping about a lost signal.
     */
#define DRIVE_NEUTRAL_US 1500

    /**
     * @brief How often the slew limiter takes a step, in milliseconds.
     *
     * How fast an output is allowed to move: microseconds of pulse per tick.
     *
     * 8 us every 20 ms is 400 us per second. This car's steering travel is 430 us
     * (1230 to 1660), so lock to lock takes 1.1 SECONDS.
     *
     * The comment here used to say "about half a second", which was wrong by a
     * factor of two and had been wrong since it was written - 400 us of travel at
     * 400 us per second is one second, not half of one.
     *
     * That is roughly a TENTH of what the servo can do. A Power HD 1501MG is
     * 0.14 s per 60 degrees, and 430 us is around 43 degrees of shaft, so the
     * hardware would place it in about 0.1 s.
     *
     * The limit is deliberate and it is right for a bench: a slider dragged end to
     * end should sweep rather than fling a servo into a stop, and rule 3 above says
     * so. It is WRONG FOR DRIVING. A car that needs 1.1 s to go lock to lock cannot
     * make an avoidance correction, and the reactive layer will need this raised -
     * probably to a value set at runtime rather than baked in here, since bench
     * work and driving want different answers and neither is a bug.
     *
     * @warning Governs how fast the wheels and the throttle can physically
     *          move; raising it makes the car respond faster to every
     *          command.
     */
#define SLEW_TICK_MS 20

    /**
     * @brief Default slew rates, in microseconds of pulse per SLEW_TICK_MS
     *        tick.
     *
     * TWO rates, not one. The steering and the throttle want different answers and
     * always did.
     *
     * A servo should get where it is told promptly - a steering correction that
     * arrives late is a correction applied to a car that has already moved past the
     * thing it was avoiding. An ESC should not: throttle slammed on spins the
     * wheels, throttle slammed off pitches the car onto its nose, and a brushed
     * motor asked for a step change draws a current spike that the BEC feels.
     *
     * They shared one number until now, so tuning the steering to be quick made the
     * throttle violent and gentling the throttle made the steering vague. Neither
     * of those is a setting anybody would choose; it was just the only shape
     * available.
     */
#define STEER_SLEW_US    SLEW_CAL_STEER
#define THROTTLE_SLEW_US SLEW_CAL_THROTTLE

    /**
     * @brief Valid range for a slew rate, in microseconds of pulse per tick.
     *
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

    /**
     * @brief A snapshot of the chassis, taken all at once.
     *
     * Returned by value rather than exposed as globals so a caller cannot read
     * `servoNow` from one moment and `servoTarget` from the next and report a car
     * that never existed.
     */
    struct State
    {
        Int32 servoUs;       ///< Steering pulse being OUTPUT now, microseconds; lags the target.
        Int32 servoTargetUs; ///< Steering pulse the output is heading toward, microseconds.
        Int32 escUs;         ///< ESC pulse being output now, microseconds.
        Int32 escTargetUs;   ///< ESC pulse the output is heading toward, microseconds.
        Bool  escArmed;      ///< True when throttleUs() is allowed through.
        Bool  servoLive;     ///< True while the steering pin is being driven at all.
        Int32 centerUs;      ///< Where the wheels point straight, measured, microseconds.
        Int32 steerMilli;    ///< Target steering as -1000..1000 of this car's travel.

        /**
         * Where the wheels ACTUALLY are, on the same scale.
         *
         * Not the same thing as steerMilli and the difference is the slew limiter:
         * a full-lock command arrives at once and the servo takes about a second to
         * walk there, so for a whole second these two disagree. Anything watching
         * the car - an indicator lamp, a controller - wants this one; anything
         * reporting what was asked for wants the other.
         */
        Int32 steerNowMilli;
        Int32 servoMinUs;      ///< Working lower steering bound, microseconds.
        Int32 servoMaxUs;      ///< Working upper steering bound, microseconds.
        Int32 escMinUs;        ///< Working lower throttle bound, microseconds.
        Int32 escMaxUs;        ///< Working upper throttle bound, microseconds.
        Int32 steerSlewUs;    ///< us of pulse per 20 ms tick, steering.
        Int32 throttleSlewUs; ///< ...and throttle. They are separate settings.
    };

    /* ---- state --------------------------------------------------------------- */

    inline Bool  up    = false;
    inline Bool  escArmed   = false;
    inline Bool  servoLive  = false;

    /*
     * The working limits, widened only on purpose. They start at the calibration
     * and are raised a little at a time while watching the linkage, which is how an
     * end stop is FOUND. Guessing them from a datasheet gets you a number the
     * linkage has never heard of.
     */
    inline Int32 servoMin = SERVO_DEFAULT_MIN;
    inline Int32 servoMax = SERVO_DEFAULT_MAX;
    inline Int32 escMin   = ESC_DEFAULT_MIN;
    inline Int32 escMax   = ESC_DEFAULT_MAX;

    /*
     * Where the wheels actually point straight.
     *
     * DRIVE_NEUTRAL_US is the middle of the SERVO's range and has nothing to say
     * about the CAR's. The horn only fits its spline at whole-tooth intervals, so
     * straight-ahead lands wherever it lands - and treating 1500 as centre is how a
     * servo comes to lean on a frame at what everyone is calling neutral.
     */
    inline Int32 servoCenterUs = STEER_CAL_CENTER;

    inline Int32 servoTarget = STEER_CAL_CENTER;
    inline Int32 servoNow    = STEER_CAL_CENTER;
    inline Int32 escTarget   = DRIVE_NEUTRAL_US;
    inline Int32 escNow      = DRIVE_NEUTRAL_US;

    /*
     * How fast an output may move, in microseconds per tick.
     *
     * Runtime, because the right answer changes with what you are doing: slow while
     * finding an end stop with the horn off, fast while driving. Baking it in meant
     * the steering crawled to wherever a slider was dragged, which reads as lag in
     * the UI and is a real limit on the car.
     */
    inline Int32 steerSlewUs    = STEER_SLEW_US;
    inline Int32 throttleSlewUs = THROTTLE_SLEW_US;

    /* When the slew limiter may next take a step. The only symbol in this module
     * that carried no module prefix, which is exactly the kind of thing the
     * prefix rule in tools/style_audit.py exists to stop drifting in. */
    inline timing::Deadline slewNextAt;

    /* ---- helpers ------------------------------------------------------------- */

    /**
     * @brief Clamps a value to an inclusive range.
     *
     * @param v  the value to clamp
     * @param lo the inclusive lower bound
     * @param hi the inclusive upper bound
     * @return v itself if already in range, otherwise the nearer bound
     */
    inline Int32 clamp(const Int32 v, const Int32 lo, const Int32 hi)
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

    /**
     * @brief Converts a steering fraction to a servo pulse width.
     *
     * Steering as a fraction of THIS car's travel: -1 is full lock one way, +1 is
     * full lock the other, 0 is wheels straight.
     *
     * The two sides are scaled separately, and that is the entire point. The
     * linkage is not symmetric and no linkage ever is. Code that adds microseconds
     * to a midpoint therefore steers further one way than the other, and a car that
     * pulls left every time it is asked for "half" is a bug that hides for a long
     * time because every individual command looks reasonable.
     *
     * @param n steering fraction, clamped to -1.0 (full lock one way) through
     *          +1.0 (full lock the other); 0.0 is straight ahead
     * @return the servo pulse width, in microseconds, before the working
     *         range is applied
     */
    inline Int32 steerToUs(Float32 n)
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
            return servoCenterUs + static_cast<Int32>(n * static_cast<Float32>(lo > 0 ? lo : 0));
        }
        return servoCenterUs + static_cast<Int32>(n * static_cast<Float32>(hi > 0 ? hi : 0));
    }

    /**
     * @brief Converts a servo pulse width back to a steering fraction.
     *
     * The inverse, in THOUSANDTHS so it can be reported without a float formatter
     * having to survive on a microcontroller.
     *
     * @param us a servo pulse width, in microseconds
     * @return steering as -1000 (full lock one way) to +1000 (full lock the
     *         other), 0 for straight ahead
     */
    inline Int32 steerFromUs(const Int32 us)
    {
        const Int32 d = us - servoCenterUs;
        if(d == 0)
        {
            return 0;
        }
        if(d < 0)
        {
            const Int32 lo = servoCenterUs - servoMin;
            return lo > 0 ? d * 1000 / lo : 0;
        }
        const Int32 hi = servoMax - servoCenterUs;
        return hi > 0 ? d * 1000 / hi : 0;
    }

    /* ---- lifecycle ----------------------------------------------------------- */

    /**
     * @brief Opens the servo and ESC pins and brings the chassis up disarmed.
     *
     * Rule 1: the steering is released, not driven to neutral, at boot. The
     * ESC is driven to DRIVE_NEUTRAL_US immediately, disarmed, because it is
     * listening for exactly that to come up rather than beeping about a lost
     * signal.
     *
     * @note pins::begin() must have run first, or PIN_SERVO/PIN_ESC resolve
     *       to no pin at all and nothing is opened.
     * @warning Writes the ESC pulse immediately. Put the car on a stand
     *          before calling this, per docs/wiring.md.
     */
    inline Void open(Void)
    {
        servo::open(PIN_SERVO);
        servo::open(PIN_ESC);

        /* Rule 1: released, not neutral. */
        servo::release(PIN_SERVO);
        servoLive = false;
        servo::writeUs(PIN_ESC, DRIVE_NEUTRAL_US);

        /* Rule 2, and open() has to say it too. escArmed was only ever cleared
         * by its initialiser and by stop(), so a SECOND open() - a re-init, a
         * mode change - parked the ESC at neutral while leaving it armed, and
         * the next throttleUs() was accepted by something that reads like a
         * fresh bring-up. Found by the first test this module ever had. */
        escArmed  = false;
        escTarget = DRIVE_NEUTRAL_US;
        escNow    = DRIVE_NEUTRAL_US;

        slewNextAt = timing::armMs(SLEW_TICK_MS);
        up  = true;
    }

    /**
     * @brief Walks each output one step closer to its target.
     *
     * Call from the main loop, often - the slew limiter only advances once
     * per SLEW_TICK_MS, so calling less often than that makes the car slower
     * to respond, not smoother.
     *
     * @note A disarmed ESC is walked back to neutral rather than snapped
     *       there: a step to neutral from a moving throttle is itself a jolt.
     * @warning This is what actually moves the servo and the ESC. Nothing
     *          the caller commands takes effect at the pins until pump()
     *          runs.
     */
    inline Void pump(Void)
    {
        if(!up || !timing::reached(slewNextAt))
        {
            return;
        }
        slewNextAt = timing::armMs(SLEW_TICK_MS);

        if(servoLive && servoNow != servoTarget)
        {
            const Int32 d    = servoTarget - servoNow;
            const Int32 step = d > steerSlewUs ? steerSlewUs
                                   : d < -steerSlewUs ? -steerSlewUs : d;
            servoNow += step;
            servo::writeUs(PIN_SERVO, static_cast<UInt32>(servoNow));
        }

        /* A disarmed ESC is walked back to neutral rather than snapped there: a
         * step to neutral from a moving throttle is itself a jolt. */
        if(const Int32 want = escArmed ? escTarget : DRIVE_NEUTRAL_US; escNow != want)
        {
            const Int32 d    = want - escNow;
            const Int32 step = d > throttleSlewUs ? throttleSlewUs
                                   : d < -throttleSlewUs ? -throttleSlewUs : d;
            escNow += step;
            servo::writeUs(PIN_ESC, static_cast<UInt32>(escNow));
        }
    }

    /**
     * @brief Disarms the ESC to neutral and releases the steering, at once.
     *
     * The ESC disarmed and neutral, and the steering RELEASED.
     *
     * Released rather than centred, and that distinction is the whole point.
     * Centre is only a safe place to put a servo if it happens to be where the
     * linkage wants to sit; if the horn is a tooth off its spline it is not, and
     * "stop" would then mean "keep pushing, just somewhere else". Nothing to push
     * with is the only stop that is a stop on every car.
     *
     * Immediate, not slewed. A stop that eases in is not a stop.
     *
     * @warning This is the emergency stop. It writes the ESC pulse and
     *          releases the servo directly, bypassing the slew limiter, and
     *          disarms the ESC so a later throttleUs() is refused until
     *          arm(true) is called again.
     */
    inline Void stop(Void)
    {
        escArmed    = false;
        escTarget   = DRIVE_NEUTRAL_US;
        escNow      = DRIVE_NEUTRAL_US;
        servoTarget = servoCenterUs;
        servoNow    = servoCenterUs;
        servoLive   = false;

        if(up)
        {
            servo::writeUs(PIN_ESC, DRIVE_NEUTRAL_US);
            servo::release(PIN_SERVO);
        }
    }

    /**
     * @brief Reads a snapshot of the chassis state.
     *
     * @return a State with every output, target and bound in microseconds,
     *         and steerMilli/steerNowMilli in thousandths of full travel
     */
    inline State read(Void)
    {
        State s{};
        s.servoUs       = servoNow;
        s.servoTargetUs = servoTarget;
        s.escUs         = escNow;
        s.escTargetUs   = escTarget;
        s.escArmed      = escArmed;
        s.servoLive     = servoLive;
        s.centerUs      = servoCenterUs;
        s.steerMilli    = steerFromUs(servoTarget);
        s.steerNowMilli = steerFromUs(servoNow);
        s.servoMinUs    = servoMin;
        s.servoMaxUs    = servoMax;
        s.escMinUs      = escMin;
        s.escMaxUs      = escMax;
        s.steerSlewUs    = steerSlewUs;
        s.throttleSlewUs = throttleSlewUs;
        return s;
    }

    /**
     * @brief Sets how fast the steering may move.
     *
     * How fast the STEERING may move, in microseconds of pulse per 20 ms tick.
     *
     * Clamped rather than refused, so a caller asking for "as fast as possible" by
     * passing a large number gets the ceiling instead of an error. Returns false
     * only for a value that is not a rate at all.
     *
     * @param usPerTick microseconds of pulse per SLEW_TICK_MS tick; clamped
     *                   into [SLEW_MIN_STEP, SLEW_MAX_STEP]
     * @return false only when usPerTick is zero or negative
     * @warning Raising this makes the wheels turn faster in response to the
     *          same steer() command.
     */
    [[nodiscard]] static Bool setSteerSlew(const Int32 usPerTick)
    {
        if(usPerTick <= 0)
        {
            return false;
        }
        steerSlewUs = clamp(usPerTick, SLEW_MIN_STEP, SLEW_MAX_STEP);
        return true;
    }

    /**
     * @brief Sets how fast the throttle may move.
     *
     * How fast the THROTTLE may move. Same units, same bounds, different setting.
     *
     * This is the one that decides whether the car pulls away or lurches. It is
     * separate from the steering because the right answer is different: a servo
     * wants to arrive promptly and an ESC wants to be led there.
     *
     * @param usPerTick microseconds of pulse per SLEW_TICK_MS tick; clamped
     *                   into [SLEW_MIN_STEP, SLEW_MAX_STEP]
     * @return false only when usPerTick is zero or negative
     * @warning Raising this makes the car accelerate and brake harder for
     *          the same throttleUs() command.
     */
    [[nodiscard]] static Bool setThrottleSlew(const Int32 usPerTick)
    {
        if(usPerTick <= 0)
        {
            return false;
        }
        throttleSlewUs = clamp(usPerTick, SLEW_MIN_STEP, SLEW_MAX_STEP);
        return true;
    }

    /**
     * @brief Sets the steering and throttle slew rates to the same value.
     *
     * Both at once, which is what "the response rate" meant when there was only
     * one. Kept because it is genuinely the common case on a bench - you are
     * usually asking for everything to be slow while you watch something - and
     * because a caller that does not care should not have to make two calls.
     *
     * @param usPerTick microseconds of pulse per SLEW_TICK_MS tick, applied
     *                   to both outputs; clamped into [SLEW_MIN_STEP,
     *                   SLEW_MAX_STEP]
     * @return false only when usPerTick is zero or negative
     */
    [[nodiscard]] static Bool setSlew(const Int32 usPerTick)
    {
        return setSteerSlew(usPerTick) && setThrottleSlew(usPerTick);
    }

    /* ---- steering ------------------------------------------------------------ */

    /**
     * @brief Starts or stops driving the steering pin.
     *
     * Engaging picks up from the CAR's centre and slews to wherever the target
     * already is, rather than jumping: the servo has been limp and its actual
     * position is unknown, so the first command after engaging is the one most
     * likely to be a surprise.
     *
     * @param on true to drive the steering pin, false to release it
     * @warning Engaging writes the pulse for the car's measured centre
     *          immediately, without waiting for pump().
     */
    inline Void engage(const Bool on)
    {
        if(on && !servoLive)
        {
            servoNow  = servoCenterUs;
            servoLive = true;
            servo::writeUs(PIN_SERVO, static_cast<UInt32>(servoNow));
            return;
        }
        if(!on && servoLive)
        {
            servoLive = false;
            servo::release(PIN_SERVO);
        }
    }

    /**
     * @brief Sets the steering target as a fraction of this car's travel.
     *
     * THE entry point for driving.
     *
     * @param n steering fraction, -1.0 (full lock one way) through +1.0
     *          (full lock the other); 0.0 is straight ahead
     * @note The output moves toward this target only as pump() is called,
     *       at the rate set by setSteerSlew().
     * @warning Steers the car. Has no effect on a released steering pin;
     *          call engage(true) first.
     */
    inline Void steer(const Float32 n)
    {
        servoTarget = clamp(steerToUs(n), servoMin, servoMax);
    }

    /**
     * @brief Sets the steering target to straight ahead.
     *
     * Wheels straight, wherever that measures out to be.
     *
     * @note Uses the measured centre (servoCenterUs / trim()), not the pulse
     *       midpoint.
     */
    inline Void center(Void)
    {
        servoTarget = clamp(servoCenterUs, servoMin, servoMax);
    }

    /**
     * @brief Sets the steering target as a raw servo pulse width.
     *
     * Raw microseconds. For CALIBRATING - finding where the ends and the centre
     * actually are - not for driving. Clamped rather than refused: a slider that
     * stops moving at the limit is clearer than one that silently does nothing.
     *
     * @param us the target pulse width, in microseconds; clamped into
     *           [servoMin, servoMax]
     * @warning Moves the steering servo. Use steer() for driving; this is
     *          for finding the end stops with the car on a stand.
     */
    inline Void steerUs(const Int32 us)
    {
        servoTarget = clamp(us, servoMin, servoMax);
    }

    /**
     * @brief Moves where "centre" is.
     *
     * Clamped into the working range, because a centre outside the limits is one
     * the servo can never be commanded to - drive::center() would silently mean
     * something else, which is worse than refusing.
     *
     * @param us the new centre pulse width, in microseconds; clamped into
     *           [servoMin, servoMax]
     * @note Does not move the servo by itself; center() and steer() read
     *       this value on their next call.
     */
    inline Void trim(const Int32 us)
    {
        servoCenterUs = clamp(us, servoMin, servoMax);
    }

    /**
     * @brief Widens or narrows the working steering range.
     *
     * Widens or narrows the working range. False if the two are the wrong way
     * round; the caller decides what to say about that.
     *
     * Clamped to the hard bound, and both the target and the centre are pulled back
     * inside so narrowing can never leave an output sitting outside its own limits.
     *
     * @param lo the new lower bound, in microseconds; must be less than hi
     * @param hi the new upper bound, in microseconds; must be greater than lo
     * @return false when lo is not strictly less than hi, either before or
     *         after clamping to [SERVO_HARD_MIN, SERVO_HARD_MAX]
     * @note Checked after clamping too - see the comment inside - or two
     *       in-order but out-of-hardware-range values collapse to a
     *       zero-width steering range.
     * @warning Also re-clamps the current target and centre, which can move
     *          the steering the next time pump() runs.
     */
    [[nodiscard]] static Bool setSteerLimits(const Int32 lo, const Int32 hi)
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
         * steering can no longer be commanded anywhere, drive::steerToUs divides a
         * span of zero, and the reply reports a car that looks configured. Found on
         * the board rather than by reading this, which is the only reason it is
         * here: it needs two plausible numbers to reach.
         */
        const Int32 lo2 = clamp(lo, SERVO_HARD_MIN, SERVO_HARD_MAX);
        const Int32 hi2 = clamp(hi, SERVO_HARD_MIN, SERVO_HARD_MAX);
        if(lo2 >= hi2)
        {
            return false;
        }

        servoMin      = lo2;
        servoMax      = hi2;
        servoTarget   = clamp(servoTarget, servoMin, servoMax);
        servoCenterUs = clamp(servoCenterUs, servoMin, servoMax);
        return true;
    }

    /* ---- throttle ------------------------------------------------------------ */

    /**
     * @brief Arms or disarms the ESC.
     *
     * Rule 2: every throttle command is refused until this is called with
     * true, and disarming walks the target back to neutral.
     *
     * @param on true to arm the ESC and allow throttleUs() through, false
     *           to disarm it
     * @warning One deliberate act between a slider and a moving car. This
     *          does not itself move the throttle; any prior throttleUs()
     *          target is replaced with neutral.
     */
    inline Void arm(const Bool on)
    {
        escArmed  = on;
        escTarget = DRIVE_NEUTRAL_US;
    }

    /**
     * @brief Sets the throttle target, in raw ESC pulse width.
     *
     * False when the ESC is not armed. Rule 2, and it lives here so no caller can
     * forget it.
     *
     * @param us the target pulse width, in microseconds; clamped into
     *           [escMin, escMax]
     * @return false when the ESC is not armed; the target is left unchanged
     * @warning Sets the throttle target the car will accelerate toward as
     *          pump() runs. Requires arm(true) first.
     */
    [[nodiscard]] static Bool throttleUs(const Int32 us)
    {
        if(!escArmed)
        {
            return false;
        }
        escTarget = clamp(us, escMin, escMax);
        return true;
    }

    /**
     * @brief Sets the throttle target back to neutral.
     *
     * @note Does not disarm the ESC; use stop() or arm(false) for that.
     */
    inline Void throttleNeutral(Void)
    {
        escTarget = DRIVE_NEUTRAL_US;
    }

    /**
     * @brief Widens or narrows the working throttle range.
     *
     * @param lo the new lower bound, in microseconds; must be less than hi
     * @param hi the new upper bound, in microseconds; must be greater than lo
     * @return false when lo is not strictly less than hi, either before or
     *         after clamping to [ESC_HARD_MIN, ESC_HARD_MAX]
     * @note The throttle's hard band is only 200 us wide (ESC_HARD_MIN to
     *       ESC_HARD_MAX), so this collapses to a zero-width range more
     *       easily than setSteerLimits() does - see the comment inside.
     * @warning Also re-clamps the current throttle target, which can change
     *          the ESC pulse the next time pump() runs.
     */
    [[nodiscard]] static Bool setThrottleLimits(const Int32 lo, const Int32 hi)
    {
        if(lo >= hi)
        {
            return false;
        }

        /* The same collapse as the steering, and worse here: the throttle's hard
         * band is only 200 us wide, so any pair below 1500 lands on 1500/1500. */
        const Int32 lo2 = clamp(lo, ESC_HARD_MIN, ESC_HARD_MAX);
        const Int32 hi2 = clamp(hi, ESC_HARD_MIN, ESC_HARD_MAX);
        if(lo2 >= hi2)
        {
            return false;
        }

        escMin    = lo2;
        escMax    = hi2;
        escTarget = clamp(escTarget, escMin, escMax);
        return true;
    }
}
