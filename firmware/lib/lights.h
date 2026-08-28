/* ---------------------------------------------------------------------------
 * lights - the car's indicators, worked out from where the wheels are pointing.
 *
 * ===========================================================================
 * TEMPORARY. THIS WHOLE FILE IS SCAFFOLDING.
 *
 * The pins below are BORROWED and both are already spoken for:
 *
 *     GP15 is the WHEEL ENCODER. docs/wiring.md, hal.h and the hub's System
 *          panel all say so, and the Hall sensor for it is on its way. The
 *          moment that arrives this file gives GP15 back - a pin cannot be an
 *          interrupt-driven input and an LED at the same time.
 *
 *     GP13 is ToF #4 XSHUT. Nothing is on the I2C bus today (SCAN answers 0),
 *          so today this costs nothing. It stops being free the moment a
 *          fourth ToF is fitted.
 *
 * This exists to watch the rule work on a real car before committing to it.
 * The permanent home is the five-pin map in docs/wiring.md - GP2/GP3 for the
 * indicators, GP6/GP7 for the tails, GP8 for both heads - and moving there is
 * changing the two numbers below.
 * ===========================================================================
 *
 * ---- what it does ---------------------------------------------------------
 *
 * Steer far enough one way and that side blinks. That is the whole rule, and it
 * is a GUESS: a real car indicates because somebody pushed a stalk, and this
 * car has no stalk. It is wrong in the cases you would expect - a long sweeping
 * bend indicates the whole way round - and right in the case that matters,
 * which is that a driver would have signalled that turn anyway.
 *
 * The same rule is implemented in the hub (hub/src/lights.hxx) so it can be
 * watched on screen, and the constants are deliberately identical. If one
 * changes, change both; they describe one car.
 *
 * ---- the three things that look right and are not -------------------------
 *
 * 1. TWO THRESHOLDS, not one. Steering parked exactly on a single threshold
 *    makes the lamp stutter at the servo's own jitter, which reads as a fault
 *    in the car rather than in the rule.
 *
 * 2. ONE COMPLETE FLASH, minimum. The blink clock free-runs, so a turn starting
 *    halfway through a cycle would otherwise show a sliver of an on-phase and
 *    vanish.
 *
 * 3. ...EXCEPT when the side changes, which is immediate. Indicating left while
 *    the wheels are already going right is the one thing an indicator must
 *    never do, so a genuine change of side ignores rule 2.
 *
 * ---- calling it -----------------------------------------------------------
 *
 *     lightsOpen();                          once, at startup
 *     lightsTick(driveRead().steerNowMilli); often, from the main loop
 *
 * The steering is passed IN rather than read from chassis.h, so this file needs
 * nothing but hal.h - see the layering rule in docs/conventions.md. It takes
 * the ACTUAL wheel position, not the target: the lamp follows the wheels, and
 * the slew limiter means those are different for about a second after every
 * command.
 *
 * On/off, not PWM. An indicator is lit or it is not - the hub's model gives it
 * 0.0 or 1.0 and nothing between. The tails and headlights will need PWM when
 * they are fitted, because a tail lamp at 30% and a brake at 100% are the same
 * bulb, and that is the point at which this file grows a duty cycle.
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.h"

/* ---- the borrowed pins, see the banner ---------------------------------- */
#define LIGHT_LEFT_PIN  15
#define LIGHT_RIGHT_PIN 13

/* ---- the rule ------------------------------------------------------------
 *
 * Thresholds in the same milli-units as DriveState.steerNowMilli: 1000 is full
 * lock. 450 and 280 are the hub's 0.45 and 0.28, and they are the same numbers
 * on purpose.
 */
#define LIGHT_ON_MILLI  450
#define LIGHT_OFF_MILLI 280

/* 1.5 Hz. 400 on, 267 off - deliberately not 50/50, because a slightly longer
 * on than off is what a real flasher can does and what the eye expects. */
#define LIGHT_ON_MS     400u
#define LIGHT_OFF_MS    267u
#define LIGHT_PERIOD_MS (LIGHT_ON_MS + LIGHT_OFF_MS)

typedef enum
{
    LIGHT_TURN_OFF = 0,
    LIGHT_TURN_LEFT,
    LIGHT_TURN_RIGHT
} LightTurn;

/* ---- state, one copy - the same deal chassis.h makes -------------------- */
static Bool      lightsUp      = false;
static Bool      lightsOn      = true;   /* the master switch, for testing   */
static LightTurn lightsTurn    = LIGHT_TURN_OFF;
static UInt64    lightsHoldUs  = 0;      /* earliest the turn may stop       */
static Bool      lightsLit     = false;  /* which half of the blink we are in */
static UInt64    lightsNextUs  = 0;

static inline Void lightsWrite(Bool left, Bool right)
{
    gpioWrite(LIGHT_LEFT_PIN,  left);
    gpioWrite(LIGHT_RIGHT_PIN, right);
}

static inline Void lightsOpen(Void)
{
    gpioOpen(LIGHT_LEFT_PIN,  PIN_DIR_OUT);
    gpioOpen(LIGHT_RIGHT_PIN, PIN_DIR_OUT);
    lightsWrite(false, false);

    lightsUp     = true;
    lightsTurn   = LIGHT_TURN_OFF;
    lightsHoldUs = 0;
    lightsLit    = false;
    lightsNextUs = 0;
}

/* The master switch. Off parks both lamps dark rather than leaving whichever
 * one happened to be lit when it was turned off. */
static inline Void lightsEnable(Bool on)
{
    lightsOn = on;
    if(!on)
    {
        lightsTurn = LIGHT_TURN_OFF;
        lightsLit  = false;
        lightsWrite(false, false);
    }
}

static inline Bool lightsEnabled(Void)
{
    return lightsOn;
}

static inline LightTurn lightsSide(Void)
{
    return lightsTurn;
}

/* Whether a lamp is lit THIS INSTANT, for a program that reports itself. */
static inline Bool lightsIsLit(Void)
{
    return lightsLit;
}

/*
 * Call often. `steerMilli` is -1000..1000, negative left.
 *
 * Cheap when there is nothing to do, and it does not run on an interrupt on
 * purpose: this drives GPIO from the same thread as everything else, so there
 * is no half-written state for another context to find.
 */
static inline Void lightsTick(Int32 steerMilli)
{
    if(!lightsUp || !lightsOn)
    {
        return;
    }

    const Int32  mag = (steerMilli < 0) ? -steerMilli : steerMilli;
    const UInt64 now = nowUs();

    const LightTurn want = (steerMilli <= -LIGHT_ON_MILLI) ? LIGHT_TURN_LEFT
                         : (steerMilli >=  LIGHT_ON_MILLI) ? LIGHT_TURN_RIGHT
                                                           : LIGHT_TURN_OFF;

    if(want != LIGHT_TURN_OFF && want != lightsTurn)
    {
        /* Rule 3: a genuine change of side takes effect at once. Starting the
         * blink clock here as well means the new side begins with a LIT phase
         * rather than inheriting whatever half of the cycle was in progress. */
        lightsTurn   = want;
        lightsHoldUs = now + (UInt64) LIGHT_PERIOD_MS * 1000u;
        lightsLit    = true;
        lightsNextUs = now + (UInt64) LIGHT_ON_MS * 1000u;
    }
    else if(lightsTurn != LIGHT_TURN_OFF
            && mag < LIGHT_OFF_MILLI
            && now >= lightsHoldUs)
    {
        /* Rules 1 and 2 together: below the LOWER threshold, and only once the
         * minimum flash has been served. */
        lightsTurn = LIGHT_TURN_OFF;
        lightsLit  = false;
    }

    if(lightsTurn == LIGHT_TURN_OFF)
    {
        lightsWrite(false, false);
        return;
    }

    /* The blink itself. Half-periods of different lengths, so this cannot be
     * the usual toggle-on-a-fixed-interval. */
    if(now >= lightsNextUs)
    {
        lightsLit    = !lightsLit;
        lightsNextUs = now + (UInt64) (lightsLit ? LIGHT_ON_MS : LIGHT_OFF_MS) * 1000u;
    }

    lightsWrite(lightsTurn == LIGHT_TURN_LEFT  && lightsLit,
                lightsTurn == LIGHT_TURN_RIGHT && lightsLit);
}
