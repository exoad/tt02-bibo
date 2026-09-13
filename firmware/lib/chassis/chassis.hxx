/*
 * chassis - the steering servo and the ESC, the two outputs on this car that can
 * break something, so the safety lives HERE and not in the caller. Anything that
 * can refuse returns Bool and never prints. State is file-scope: each image is a
 * single translation unit, so there is one chassis.
 *
 * 1. THE STEERING IS RELEASED AT BOOT - no pulse at all, because 1500 us is not
 *    safe for a linkage whose horn is a tooth off its spline. The ESC does get
 *    neutral at once: fed no pulse it beeps about a lost signal.
 * 2. THE ESC IS DISARMED UNTIL ASKED. Throttle is refused until arm(true).
 * 3. NOTHING JUMPS. Calls set a TARGET and pump() walks toward it at a bounded
 *    rate. stop() and throttleNeutralNow() write the pin at once, on purpose.
 *
 * Before the ESC is ever armed:
 *   - The Pico and the ESC need a common ground; without it both outputs see
 *     noise, which looks like erratic behavior rather than none. The breadboard
 *     rails are split in the middle, and it does not look like it.
 *   - NEVER connect the BEC to the Pico: 6 V on the 10BL160 G2 is over the
 *     Pico's VSYS limit, USB attached or not.
 *   - Put the car on a stand.
 */
#pragma once

#include "../hal.hxx"
#include "../pins.hxx"
#include "cal.hxx"

namespace bibo::drive
{
    /*
     * From the installed pin map: NONE until pins::begin() has run, and then
     * open() binds nothing.
     */
#define PIN_SERVO (pins::active().servo)
#define PIN_ESC   (pins::active().esc)

#define SERVO_DEFAULT_MIN STEER_CAL_LEFT
#define SERVO_DEFAULT_MAX STEER_CAL_RIGHT

    /*
     * The widest steering pulse, us. setSteerLimits() clamps into it, so nothing
     * above this file can command the steering outside it; the working limits
     * are what keep the linkage off its end stops.
     */
#define SERVO_HARD_MIN 500
#define SERVO_HARD_MAX 2500

    /*
     * Throttle pulses, us. ESC_HARD_MIN/ESC_HARD_MAX are the absolute ceiling.
     * BELOW NEUTRAL IS ITS OWN RANGE: the ESC runs Forward/Reverse/Brake, so the
     * first pulse below neutral brakes and, after a return to neutral, the next
     * reverses. Its limit is escReverse, which starts AT neutral (reverse off), so
     * nothing below neutral is reachable until ESCREVERSE sets a limit on purpose.
     */
#define ESC_DEFAULT_MIN THROTTLE_CAL_MIN
#define ESC_DEFAULT_MAX THROTTLE_CAL_MAX
#define ESC_HARD_MIN    1000
#define ESC_HARD_MAX    2000
#define ESC_REVERSE_DEFAULT 1500
#define DRIVE_NEUTRAL_US 1500

    /* Rule 3's clock, ms: each tick moves an output by at most its slew rate. */
#define SLEW_TICK_MS 20

    /*
     * Two rates, us per tick, because the outputs want different answers: a late
     * steering correction reaches a car already past what it was avoiding, while
     * a throttle step spins the wheels and draws a current spike the BEC feels.
     */
#define STEER_SLEW_US    SLEW_CAL_STEER
#define THROTTLE_SLEW_US SLEW_CAL_THROTTLE

    /*
     * Slew bounds, us per tick. SLEW_MAX_STEP is faster than the servo can
     * follow, so "as fast as it goes" is the servo's limit and not this file's.
     */
#define SLEW_MIN_STEP 1
#define SLEW_MAX_STEP 200

    /*
     * A snapshot, by value, so a caller cannot read servoNow from one moment and
     * servoTarget from the next and report a car that never existed.
     */
    struct State
    {
        Int32 servoUs;       ///< Output now; lags servoTargetUs by the slew.
        Int32 servoTargetUs;
        Int32 escUs;
        Int32 escTargetUs;
        Bool  escArmed;
        Bool  servoLive;     ///< True while the steering pin is driven at all.
        Int32 centerUs;      ///< Where the wheels point straight, measured.
        Int32 steerMilli;    ///< Target steering, -1000..1000 of this car's travel.
        /**
         * Where the wheels are now, same scale; lags steerMilli by the slew, so a
         * watcher wants this one.
         */
        Int32 steerNowMilli;
        Int32 servoMinUs;
        Int32 servoMaxUs;
        Int32 escMinUs;
        Int32 escMaxUs;
        Int32 escReverseUs;    ///< Lowest brake/reverse pulse; neutral is reverse off.
        Int32 steerSlewUs;    ///< us per SLEW_TICK_MS tick.
        Int32 throttleSlewUs;
    };

    inline Bool  up = false;
    inline Bool  escArmed = false;
    inline Bool  servoLive = false;

    /*
     * Working limits: they start at the calibration and are widened only while
     * watching the linkage.
     */
    inline Int32 servoMin = SERVO_DEFAULT_MIN;
    inline Int32 servoMax = SERVO_DEFAULT_MAX;
    inline Int32 escMin = ESC_DEFAULT_MIN;
    inline Int32 escMax = ESC_DEFAULT_MAX;
    inline Int32 escReverse = ESC_REVERSE_DEFAULT;

    /*
     * Where the wheels actually point straight. DRIVE_NEUTRAL_US is the middle of
     * the SERVO's range, not the CAR's: the horn fits its spline only at whole
     * teeth, and a servo centered at neutral leans on the frame.
     */
    inline Int32 servoCenterUs = STEER_CAL_CENTER;
    inline Int32 servoTarget = STEER_CAL_CENTER;
    inline Int32 servoNow = STEER_CAL_CENTER;
    inline Int32 escTarget = DRIVE_NEUTRAL_US;
    inline Int32 escNow = DRIVE_NEUTRAL_US;

    /* Runtime, because the job decides: slow while finding an end stop, fast while driving. */
    inline Int32 steerSlewUs = STEER_SLEW_US;
    inline Int32 throttleSlewUs = THROTTLE_SLEW_US;
    inline timing::Deadline slewNextAt;

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
     * Steering fraction to pulse, us, before the working range is applied: -1 is
     * full lock one way, +1 the other, 0 straight; n is clamped to that. The two
     * sides are scaled SEPARATELY because no linkage is symmetric: microseconds
     * added to a midpoint steer further one way than the other.
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
        /* A center sitting on an end is no range to interpolate, and must not divide. */
        const Int32 lo = servoCenterUs - servoMin;
        const Int32 hi = servoMax - servoCenterUs;
        if(n < 0.0f)
        {
            return servoCenterUs + static_cast<Int32>(n * static_cast<Float32>(lo > 0 ? lo : 0));
        }
        return servoCenterUs + static_cast<Int32>(n * static_cast<Float32>(hi > 0 ? hi : 0));
    }

    /** The inverse, -1000..1000: thousandths, so no float formatter is needed on the Pico. */
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

    /** Writes the ESC pulse at once: put the car on a stand first. */
    inline Void open(Void)
    {
        servo::open(PIN_SERVO);
        servo::open(PIN_ESC);
        /* Rule 1: released, not neutral. */
        servo::release(PIN_SERVO);
        servoLive = false;
        servo::writeUs(PIN_ESC, DRIVE_NEUTRAL_US);
        /*
         * Rule 2 on every open(): a second open() must not leave the ESC armed for
         * the next throttleUs().
         */
        escArmed = false;
        escTarget = DRIVE_NEUTRAL_US;
        escNow = DRIVE_NEUTRAL_US;
        slewNextAt = timing::armMs(SLEW_TICK_MS);
        up = true;
    }

    /**
     * The only thing that moves the pins toward their targets. Call it often from
     * the main loop; it steps once per SLEW_TICK_MS, so calling it less often makes
     * the car slower to respond, not smoother.
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
            const Int32 d = servoTarget - servoNow;
            const Int32 step = d > steerSlewUs ? steerSlewUs
                                   : d < -steerSlewUs ? -steerSlewUs : d;
            servoNow += step;
            servo::writeUs(PIN_SERVO, static_cast<UInt32>(servoNow));
        }
        /* A disarmed ESC is walked back to neutral: a step there is itself a jolt. */
        if(const Int32 want = escArmed ? escTarget : DRIVE_NEUTRAL_US; escNow != want)
        {
            const Int32 d = want - escNow;
            const Int32 step = d > throttleSlewUs ? throttleSlewUs
                                   : d < -throttleSlewUs ? -throttleSlewUs : d;
            escNow += step;
            servo::writeUs(PIN_ESC, static_cast<UInt32>(escNow));
        }
    }

    /**
     * THE EMERGENCY STOP: the ESC disarmed at neutral and the steering RELEASED,
     * written at once, bypassing the slew. Released rather than centered: if the
     * horn is a tooth off its spline, center still pushes, and nothing to push
     * with is a stop on every car. throttleUs() is refused until arm(true).
     */
    inline Void stop(Void)
    {
        escArmed = false;
        escTarget = DRIVE_NEUTRAL_US;
        escNow = DRIVE_NEUTRAL_US;
        servoTarget = servoCenterUs;
        servoNow = servoCenterUs;
        servoLive = false;
        if(up)
        {
            servo::writeUs(PIN_ESC, DRIVE_NEUTRAL_US);
            servo::release(PIN_SERVO);
        }
    }

    inline State read(Void)
    {
        State s{};
        s.servoUs = servoNow;
        s.servoTargetUs = servoTarget;
        s.escUs = escNow;
        s.escTargetUs = escTarget;
        s.escArmed = escArmed;
        s.servoLive = servoLive;
        s.centerUs = servoCenterUs;
        s.steerMilli = steerFromUs(servoTarget);
        s.steerNowMilli = steerFromUs(servoNow);
        s.servoMinUs = servoMin;
        s.servoMaxUs = servoMax;
        s.escMinUs = escMin;
        s.escMaxUs = escMax;
        s.escReverseUs = escReverse;
        s.steerSlewUs = steerSlewUs;
        s.throttleSlewUs = throttleSlewUs;
        return s;
    }

    /**
     * us per SLEW_TICK_MS tick, clamped into [SLEW_MIN_STEP, SLEW_MAX_STEP] rather
     * than refused, so a large number means as fast as allowed. False only for a
     * rate of zero or less. The throttle and both-at-once setters work the same.
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

    [[nodiscard]] static Bool setThrottleSlew(const Int32 usPerTick)
    {
        if(usPerTick <= 0)
        {
            return false;
        }
        throttleSlewUs = clamp(usPerTick, SLEW_MIN_STEP, SLEW_MAX_STEP);
        return true;
    }

    [[nodiscard]] static Bool setSlew(const Int32 usPerTick)
    {
        return setSteerSlew(usPerTick) && setThrottleSlew(usPerTick);
    }

    /**
     * Engaging writes the car's measured center at once and slews to the target
     * from there: the limp servo's position is unknown, so the first command after
     * engaging is the one most likely to surprise.
     */
    inline Void engage(const Bool on)
    {
        if(on && !servoLive)
        {
            servoNow = servoCenterUs;
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
     * THE entry point for driving; n as in steerToUs(). A released pin ignores it
     * until engage(true).
     */
    inline Void steer(const Float32 n)
    {
        servoTarget = clamp(steerToUs(n), servoMin, servoMax);
    }

    inline Void center(Void)
    {
        servoTarget = clamp(servoCenterUs, servoMin, servoMax);
    }

    /**
     * Raw microseconds, for finding the end stops on a stand, not for driving.
     * Clamped rather than refused: a slider that stops at the limit is clearer than
     * one that silently does nothing.
     */
    inline Void steerUs(const Int32 us)
    {
        servoTarget = clamp(us, servoMin, servoMax);
    }

    /**
     * Moves where center is, without moving the servo. Clamped into the working
     * range: a center the servo can never be commanded to would make center()
     * silently mean something else.
     */
    inline Void trim(const Int32 us)
    {
        servoCenterUs = clamp(us, servoMin, servoMax);
    }

    /**
     * False when lo is not below hi. The target and center are pulled back inside,
     * so narrowing never leaves an output outside its limits; that can move the
     * steering on the next pump().
     */
    [[nodiscard]] static Bool setSteerLimits(const Int32 lo, const Int32 hi)
    {
        if(lo >= hi)
        {
            return false;
        }
        /*
         * Checked again after clamping: SERVOLIMITS 1 2 is in order, but both clamp
         * to SERVO_HARD_MIN, a span steerToUs cannot use.
         */
        const Int32 lo2 = clamp(lo, SERVO_HARD_MIN, SERVO_HARD_MAX);
        const Int32 hi2 = clamp(hi, SERVO_HARD_MIN, SERVO_HARD_MAX);
        if(lo2 >= hi2)
        {
            return false;
        }
        servoMin = lo2;
        servoMax = hi2;
        servoTarget = clamp(servoTarget, servoMin, servoMax);
        servoCenterUs = clamp(servoCenterUs, servoMin, servoMax);
        return true;
    }

    /** Rule 2. Either way the target becomes neutral, so arming never moves the car. */
    inline Void arm(const Bool on)
    {
        escArmed = on;
        escTarget = DRIVE_NEUTRAL_US;
    }

    /**
     * Rule 2 lives here so no caller can forget it: false, target unchanged, while
     * disarmed. At or above neutral the target is clamped into [escMin, escMax];
     * below it, into [escReverse, DRIVE_NEUTRAL_US], so with reverse off it becomes
     * neutral, never the forward idle.
     */
    [[nodiscard]] static Bool throttleUs(const Int32 us)
    {
        if(!escArmed)
        {
            return false;
        }
        if(us < DRIVE_NEUTRAL_US)
        {
            escTarget = clamp(us, escReverse, DRIVE_NEUTRAL_US);
        }
        else
        {
            escTarget = clamp(us, escMin, escMax);
        }
        return true;
    }

    /** Moves the TARGET to neutral; pump() slews there. The ESC stays armed. */
    inline Void throttleNeutral(Void)
    {
        escTarget = DRIVE_NEUTRAL_US;
    }

    /**
     * Writes the ESC pin AT neutral now; the ESC stays ARMED and the steering keeps
     * its angle and pulse. The watchdog's call.
     *
     * Not throttleNeutral(): its slew outlasts the BIBO_WATCHDOG_MS that called for
     * the stop, even from THROTTLE_CAL_MAX at the default rate. Not stop(): wheels
     * going limp as the link dies change the car's line at the worst moment, and
     * re-arming needs a person who is not there. escTarget moves too, so pump()
     * holds neutral.
     */
    inline Void throttleNeutralNow(Void)
    {
        escTarget = DRIVE_NEUTRAL_US;
        escNow = DRIVE_NEUTRAL_US;
        if(up)
        {
            servo::writeUs(PIN_ESC, DRIVE_NEUTRAL_US);
        }
    }

    /** As setSteerLimits(), including the check after clamping. */
    [[nodiscard]] static Bool setThrottleLimits(const Int32 lo, const Int32 hi)
    {
        if(lo >= hi)
        {
            return false;
        }
        const Int32 lo2 = clamp(lo, ESC_HARD_MIN, ESC_HARD_MAX);
        const Int32 hi2 = clamp(hi, ESC_HARD_MIN, ESC_HARD_MAX);
        if(lo2 >= hi2)
        {
            return false;
        }
        escMin = lo2;
        escMax = hi2;
        /*
         * Only a FORWARD target is re-clamped: neutral or a brake pulled up into
         * [escMin, escMax] is a creep nobody asked for.
         */
        if(escTarget > DRIVE_NEUTRAL_US)
        {
            escTarget = clamp(escTarget, escMin, escMax);
        }
        return true;
    }

    [[nodiscard]] static Bool setReverseLimit(const Int32 us)
    {
        if(us < ESC_HARD_MIN || us > DRIVE_NEUTRAL_US)
        {
            return false;
        }
        escReverse = us;
        if(escTarget < DRIVE_NEUTRAL_US)
        {
            escTarget = clamp(escTarget, escReverse, DRIVE_NEUTRAL_US);
        }
        return true;
    }
}
