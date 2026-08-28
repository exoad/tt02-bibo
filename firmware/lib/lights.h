/* ---------------------------------------------------------------------------
 * lights - what the car's lamps are doing, and which pins currently show it.
 *
 * Two separate things live here, and the split is the whole point:
 *
 *   THE RULES     which lamp should be lit, given what the car is doing. This
 *                 is a permanent part of the library. It is a fact about how a
 *                 vehicle signals, it does not change when a wire moves, and it
 *                 is written once so a sketch, the debug image and the eventual
 *                 autonomous program all signal identically.
 *
 *   THE BINDING   which GPIO each lamp is soldered to today. This is temporary
 *                 and is one table, below.
 *
 * The car has eight lamps in the model whether or not eight LEDs exist. A lamp
 * with no pin bound simply is not written - lightsSolve() still computes it,
 * and LIGHTS still reports it, so wiring an LED later is filling in a table
 * entry rather than writing a rule.
 *
 * ===========================================================================
 * THE BINDING BELOW IS TEMPORARY AND ON BORROWED PINS.
 *
 *     GP15 is the WHEEL ENCODER. docs/wiring.md, hal.h and the hub's System
 *          panel all say so, and the Hall sensor for it is here. The moment it
 *          goes on, this table gives GP15 back - a pin cannot be an
 *          interrupt-driven input and an LED at the same time.
 *
 *     GP13 is ToF #4 XSHUT, free only because the I2C bus is empty (SCAN
 *          answers 0).
 *
 * The permanent map is in docs/wiring.md: GP2/GP3 indicators, GP6/GP7 tails,
 * GP8 both heads. Moving there is editing the table and nothing else.
 * ===========================================================================
 *
 * ---- the rules, and how honest each one is --------------------------------
 *
 * TAILS / BRAKE   Lit at full whenever no throttle is being asked for, dim
 *                 whenever the headlights are on, dark otherwise.
 *
 *                 This is NOT braking, and the difference matters. A brake
 *                 light on a real car means a pedal went down; this car has no
 *                 pedal, no brake, and no way to measure whether it is actually
 *                 slowing - the wheel encoder that would answer that is the
 *                 very pin this is borrowing. So the lamp reports the one
 *                 throttle fact available: nothing is being asked of the motor.
 *
 *                 The consequence, stated rather than left to be found: a car
 *                 standing still with the ESC disarmed has its tails ON,
 *                 because it is not being driven forward. Right by this rule,
 *                 wrong for a real car. Telling "stopped" from "slowing" needs
 *                 a speed, and a speed needs the encoder.
 *
 * INDICATORS      Steer far enough one way and that side blinks. Also a guess -
 *                 a real car indicates because somebody pushed a stalk - and
 *                 wrong in the cases you would expect: a long sweeping bend
 *                 indicates the whole way round. Right in the case that
 *                 matters, which is that a driver would have signalled it.
 *
 *                 Three things here look right written down and are wrong on a
 *                 car:
 *                   1. TWO thresholds. Steering parked on a single one makes
 *                      the lamp stutter at the servo's own jitter.
 *                   2. ONE COMPLETE FLASH minimum. The blink clock free-runs,
 *                      so a turn starting mid-cycle would otherwise show a
 *                      sliver of an on-phase and vanish.
 *                   3. ...EXCEPT a change of side, which is immediate.
 *                      Indicating left while the wheels go right is the one
 *                      thing an indicator must never do.
 *
 *                 THE OVERRIDE: an indicating side interrupts its own tail
 *                 lamp. That asymmetry - one side alternating, the other solid
 *                 - is what makes a car read as a car rather than a light show.
 *
 * HEAD            Manual. Nothing the car knows implies "it is dark".
 *
 * REVERSE         Never. chassis.h is forward-only.
 *
 * The same rules are implemented in hub/src/lights.hxx so they can be watched
 * on screen, and the constants are deliberately identical. If one changes,
 * change both; they describe one car.
 *
 * ---- calling it -----------------------------------------------------------
 *
 *     lightsOpen();                      once, at startup
 *     lightsTick(&in);                   often, from the main loop
 *
 * Everything the rules read is passed IN rather than reached for, so this file
 * needs nothing but hal.h - see the layering rule in docs/conventions.md. Feed
 * it the ACTUAL servo and ESC output, not the targets: the slew limiter means
 * the two differ for about a second after every command, and a lamp should
 * follow the car rather than the request.
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.h"

/* ---- the lamps a car has ------------------------------------------------ */
typedef enum
{
    LAMP_HEAD_L = 0,
    LAMP_HEAD_R,
    LAMP_TAIL_L,
    LAMP_TAIL_R,
    LAMP_IND_L,
    LAMP_IND_R,
    LAMP_REV_L,
    LAMP_REV_R,
    LAMP_COUNT
} Lamp;

/*
 * Brightness, 0 dark to 255 full.
 *
 * A level rather than a Bool because the tails are ONE lamp at two brightnesses
 * - dim for running, full for braking - which is how a real car does it and
 * cannot be said with an on/off flag. Nothing drives PWM yet, so a bound pin is
 * written as (level > 0); the day the tails move to a PWM-capable pin, that one
 * line changes and no rule does.
 */
#define LAMP_OFF   0u
#define LAMP_DIM   76u    /* 30%: tail */
#define LAMP_DRL   115u   /* 45%: daytime running */
#define LAMP_FULL  255u

typedef struct
{
    UInt8 level[LAMP_COUNT];
} LampSet;

/* What the rules read. */
typedef struct
{
    Int32 steerMilli;   /* -1000..1000 of this car's travel, ACTUAL           */
    Int32 throttleUs;   /* what the ESC is actually being given               */
    Int32 idleUs;       /* the pulse at which this motor sits still (cal.h)   */
    Int32 neutralUs;    /* the pulse that is neither forward nor back         */
    Bool  armed;
    Bool  headOn;       /* nothing the car knows implies this; a human sets it */
} LightInput;

/* ---- the rule's constants ----------------------------------------------- */

/*
 * How far past idle the throttle has to go before the car counts as being
 * DRIVEN and the tails go out - and, mirrored, how far below neutral before it
 * counts as reversing.
 *
 * Settable rather than fixed because it is a judgement about when "moving"
 * starts, not a measurement: a car crawling has not really pulled away and its
 * lamp should still be on, and where exactly that line falls is something you
 * find by watching the car. Seeded from cal.h by the application - this file
 * cannot reach for cal.h and should not want to, since the rule is the same on
 * any car and only the number is this one's.
 */
static Int32 lightsOffUs = 10;

/* Wide enough to be useful, narrow enough that a typo cannot switch the lamps
 * off for the whole usable throttle range. */
#define LIGHT_OFF_US_MIN 0
#define LIGHT_OFF_US_MAX 60

static inline Bool lightsSetOffThreshold(Int32 us)
{
    if(us < LIGHT_OFF_US_MIN || us > LIGHT_OFF_US_MAX)
    {
        return false;
    }
    lightsOffUs = us;
    return true;
}

static inline Int32 lightsOffThreshold(Void)
{
    return lightsOffUs;
}

/* Milli-units of travel. 450/280 are the hub's 0.45/0.28, on purpose. */
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
    LIGHT_TURN_RIGHT,
    LIGHT_TURN_HAZARD
} LightTurn;

/* ===========================================================================
 * THE BINDING. Temporary - see the banner.
 * ======================================================================== */
#define LIGHT_PIN_NONE (-1)

static Int32 lightPin[LAMP_COUNT] =
{
    LIGHT_PIN_NONE,   /* LAMP_HEAD_L */
    LIGHT_PIN_NONE,   /* LAMP_HEAD_R */
    15,               /* LAMP_TAIL_L - borrowed from the wheel encoder */
    13,               /* LAMP_TAIL_R - borrowed from ToF #4 XSHUT      */
    LIGHT_PIN_NONE,   /* LAMP_IND_L  */
    LIGHT_PIN_NONE,   /* LAMP_IND_R  */
    LIGHT_PIN_NONE,   /* LAMP_REV_L  */
    LIGHT_PIN_NONE    /* LAMP_REV_R  */
};

/* ---- state, one copy - the same deal chassis.h makes -------------------- */
static Bool      lightsUp     = false;
static Bool      lightsOn     = true;   /* the master switch, for testing    */
static LampSet   lightsNow;             /* what was last written             */

static LightTurn lightsTurn   = LIGHT_TURN_OFF;
static UInt64    lightsHoldUs = 0;      /* earliest the turn may stop        */
static Bool      lightsFlash  = false;  /* which half of the blink we are in */
static UInt64    lightsNextUs = 0;

/*
 * A lamp held on by hand, ignoring every rule. LAMP_COUNT means "no override",
 * which is the normal state.
 *
 * This exists because "the lamp does not work" has three unrelated causes - the
 * rule never fired, the pin never moved, or the LED is wired backwards - and
 * without a way to light one on demand there is no telling them apart.
 */
static Int32 lightsForced = LAMP_COUNT;

static inline Void lightsWriteSet(const LampSet* s)
{
    for(Int32 i = 0; i < LAMP_COUNT; ++i)
    {
        if(lightPin[i] != LIGHT_PIN_NONE)
        {
            gpioWrite((Pin) lightPin[i], s->level[i] > LAMP_OFF);
        }
    }
    lightsNow = *s;
}

static inline Void lightsClear(LampSet* s)
{
    for(Int32 i = 0; i < LAMP_COUNT; ++i)
    {
        s->level[i] = LAMP_OFF;
    }
}

static inline Void lightsOpen(Void)
{
    for(Int32 i = 0; i < LAMP_COUNT; ++i)
    {
        if(lightPin[i] != LIGHT_PIN_NONE)
        {
            gpioOpen((Pin) lightPin[i], PIN_DIR_OUT);
        }
    }

    lightsClear(&lightsNow);
    lightsWriteSet(&lightsNow);

    lightsUp     = true;
    lightsTurn   = LIGHT_TURN_OFF;
    lightsHoldUs = 0;
    lightsFlash  = false;
    lightsNextUs = 0;
    lightsForced = LAMP_COUNT;
}

/* The master switch. Off parks every lamp dark rather than leaving whichever
 * happened to be lit when it was turned off. */
static inline Void lightsEnable(Bool on)
{
    lightsOn = on;
    if(!on)
    {
        LampSet dark;
        lightsClear(&dark);
        lightsWriteSet(&dark);
    }
}

static inline Bool lightsEnabled(Void)
{
    return lightsOn;
}

/* What each lamp is doing this instant. */
static inline LampSet lightsRead(Void)
{
    return lightsNow;
}

static inline Bool lightsLit(Lamp l)
{
    return lightsNow.level[l] > LAMP_OFF;
}

/* Which way the car reckons it is turning. */
static inline LightTurn lightsSide(Void)
{
    return lightsTurn;
}

/* Hold ONE lamp lit, or LAMP_COUNT to hand it back to the rules. */
static inline Void lightsForceLamp(Int32 lamp)
{
    lightsForced = lamp;
}

static inline Int32 lightsForcedLamp(Void)
{
    return lightsForced;
}

/*
 * THE RULES. Pure: same input and same clock, same answer, no hardware.
 *
 * Split out from lightsTick so it can be reasoned about - and eventually
 * tested on the host, the way lib/text.h is - without a Pico in the loop. The
 * blink phase is passed in for the same reason.
 */
static inline Void lightsSolve(const LightInput* in, LightTurn turn, Bool flash, LampSet* out)
{
    lightsClear(out);

    const Bool left  = (turn == LIGHT_TURN_LEFT)  || (turn == LIGHT_TURN_HAZARD);
    const Bool right = (turn == LIGHT_TURN_RIGHT) || (turn == LIGHT_TURN_HAZARD);

    /* Hazards are BOTH sides IN PHASE, not alternating. Alternating is what a
     * film prop does and is the single most common way to get this wrong; both
     * sides reading one `flash` makes being in phase structural. */
    out->level[LAMP_IND_L] = (left  && flash) ? LAMP_FULL : LAMP_OFF;
    out->level[LAMP_IND_R] = (right && flash) ? LAMP_FULL : LAMP_OFF;

    const UInt8 head = in->headOn ? LAMP_FULL : LAMP_OFF;
    out->level[LAMP_HEAD_L] = head;
    out->level[LAMP_HEAD_R] = head;

    /*
     * Is the car being DRIVEN - either way?
     *
     * Forward needs to clear idle by the threshold, because idle is the pulse
     * at which nothing turns and the microsecond after it is a car that has not
     * really pulled away. Reverse mirrors it about neutral. Between the two the
     * motor is doing nothing worth calling motion, and the tails stay on.
     *
     * Symmetric on purpose. A rule that extinguished the lamps the instant the
     * throttle left neutral in one direction and not the other would be a rule
     * with a side, and there is nothing about this car that has one.
     */
    const Bool fwd = (in->throttleUs > (in->idleUs + lightsOffUs));
    const Bool rev = (in->throttleUs < (in->neutralUs - lightsOffUs));
    const Bool driven = fwd || rev;

    /* One red lamp at two brightnesses. Braking wins over running: it is the
     * more urgent claim. */
    const UInt8 red = driven ? (in->headOn ? LAMP_DIM : LAMP_OFF)
                             : LAMP_FULL;

    out->level[LAMP_TAIL_L] = red;
    out->level[LAMP_TAIL_R] = red;

    /* THE OVERRIDE. An indicating side interrupts its own tail while the
     * indicator is lit, so that side alternates and the other stays solid. It
     * has to be resolved AFTER the tail rather than beside it. */
    if(left  && flash) out->level[LAMP_TAIL_L] = LAMP_OFF;
    if(right && flash) out->level[LAMP_TAIL_R] = LAMP_OFF;

    /*
     * Reverse lamps: lit while the car is being driven BACKWARDS.
     *
     * chassis.h is forward-only today - it clamps the throttle to [idle, max]
     * and the board refuses anything below neutral - so `rev` is currently
     * always false and these lamps never light. The rule is here anyway, and
     * that is deliberate: reverse arrives as a brake-then-reverse sequence in
     * the ESC, and when it does the lighting should already be right rather
     * than being the thing somebody remembers afterwards.
     *
     * White, and NOT interrupted by an indicator: they report which way the
     * gearbox is, which no other signal contradicts.
     */
    const UInt8 white = rev ? LAMP_FULL : LAMP_OFF;
    out->level[LAMP_REV_L] = white;
    out->level[LAMP_REV_R] = white;
}

/* Call often. Cheap when there is nothing to do. */
static inline Void lightsTick(const LightInput* in)
{
    if(!lightsUp || !lightsOn || in == NULL)
    {
        return;
    }

    const UInt64 now = nowUs();

    /* Forced: one lamp, nothing else, no rules. */
    if(lightsForced != LAMP_COUNT)
    {
        LampSet s;
        lightsClear(&s);
        s.level[lightsForced] = LAMP_FULL;
        lightsWriteSet(&s);
        return;
    }

    /* ---- which way, with hysteresis and a minimum flash ------------------ */
    const Int32 mag = (in->steerMilli < 0) ? -in->steerMilli : in->steerMilli;

    const LightTurn want = (in->steerMilli <= -LIGHT_ON_MILLI) ? LIGHT_TURN_LEFT
                         : (in->steerMilli >=  LIGHT_ON_MILLI) ? LIGHT_TURN_RIGHT
                                                               : LIGHT_TURN_OFF;

    if(want != LIGHT_TURN_OFF && want != lightsTurn)
    {
        /* Rule 3: a genuine change of side takes effect at once. Restarting the
         * blink here means the new side begins with a LIT phase rather than
         * inheriting whatever half of the cycle was in progress. */
        lightsTurn   = want;
        lightsHoldUs = now + (UInt64) LIGHT_PERIOD_MS * 1000u;
        lightsFlash  = true;
        lightsNextUs = now + (UInt64) LIGHT_ON_MS * 1000u;
    }
    else if(lightsTurn != LIGHT_TURN_OFF
            && mag < LIGHT_OFF_MILLI
            && now >= lightsHoldUs)
    {
        /* Rules 1 and 2 together: below the LOWER threshold, and only once the
         * minimum flash has been served. */
        lightsTurn  = LIGHT_TURN_OFF;
        lightsFlash = false;
    }

    /* Half-periods of different lengths, so this cannot be the usual
     * toggle-on-a-fixed-interval. */
    if(lightsTurn != LIGHT_TURN_OFF && now >= lightsNextUs)
    {
        lightsFlash  = !lightsFlash;
        lightsNextUs = now + (UInt64) (lightsFlash ? LIGHT_ON_MS : LIGHT_OFF_MS) * 1000u;
    }

    LampSet s;
    lightsSolve(in, lightsTurn, lightsFlash, &s);
    lightsWriteSet(&s);
}
