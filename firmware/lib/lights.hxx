/* ---------------------------------------------------------------------------
 * lights - the lamps the car has, and which pins they are on today.
 *
 * THE OUTPUT LAYER, and nothing else. It knows what a lamp is, which GPIO each
 * one is soldered to, and how to write a set of brightnesses out. It does not
 * know why any lamp is lit.
 *
 * WHY a lamp is lit lives in cue.h - indicating, not being driven, reversing,
 * or a deliberate utterance like a headlight flash. Those rules used to be in
 * this file, next to the pin table, and that stopped working the moment the car
 * needed more than one way to express something: either this file would grow a
 * sound API, or sound would grow its own copy of "is the car turning". The
 * split is by job now. This one drives lamps.
 *
 * The car has ten lamps in the model whether or not ten LEDs exist. A lamp with
 * no pin bound simply is not written - cue::solve() still computes it and LIGHTS
 * still reports it, so wiring an LED later is filling in a table entry rather
 * than writing a rule.
 *
 * ===========================================================================
 * THE BINDING BELOW IS TEMPORARY AND ON BORROWED PINS.
 *
 *     GP15 is the WHEEL ENCODER. docs/wiring.md, hal.h and the hub's System
 *          panel all say so, and the Hall sensor for it is here. The moment it
 *          goes on, this table gives GP15 back - a pin cannot be an
 *          interrupt-driven input and an LED at the same time.
 *
 *          That is the one borrowing here with a deadline on it.
 *
 *     GP10, GP11, GP12 and GP13 are ToF #1-#4 XSHUT. ALL FOUR are now lamps:
 *          GP13/GP12 the front indicators, and GP11/GP10 the headlights added
 *          on 2026-08-28. They are free only because the I2C bus is empty -
 *          SCAN answers 0 - and they stop being free the moment a single ToF is
 *          fitted, not just the third and fourth.
 *
 *          Worth saying plainly because the hub's System panel still lists
 *          "ToF bumpers (GP10-13)" as a subsystem: that row is describing where
 *          those sensors are GOING, and every one of those pins currently has
 *          an LED on it.
 *
 *     GP14 is the only one of the six that is genuinely spare. It is the
 *          neighbour set aside for the encoder's B channel if quadrature were
 *          ever wanted, and this car is forward-only, so it is not.
 *
 * The permanent map is in docs/wiring.md: GP2/GP3 indicators, GP6/GP7 tails,
 * GP8 both heads. Moving there is editing the table and nothing else.
 *
 * TWO PAIRS OF INDICATORS are coming - front and rear, four amber lamps. The
 * model already has all four and cue.h computes them; only the rear pair has no
 * pin. Wiring them is two numbers in the table below, which is the whole reason
 * the rules and the binding are separate things.
 * ===========================================================================
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.hxx"

namespace lights
{

/* ---- the lamps a car has ------------------------------------------------ */
typedef enum
{
    HEAD_L = 0,
    HEAD_R,
    TAIL_L,
    TAIL_R,
    IND_FL,
    IND_FR,
    IND_RL,
    IND_RR,
    REV_L,
    REV_R,
    COUNT
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
    UInt8 level[COUNT];
} Set;

/* ===========================================================================
 * THE BINDING. Temporary - see the banner.
 * ======================================================================== */
#define LIGHT_PIN_NONE (-1)

static Int32 pin[COUNT] =
{
    11,               /* HEAD_L - borrowed from ToF #2 XSHUT         */
    10,               /* HEAD_R - borrowed from ToF #1 XSHUT         */
    15,               /* TAIL_L - borrowed from the WHEEL ENCODER    */
    14,               /* TAIL_R - genuinely spare today              */
    13,               /* IND_FL - borrowed from ToF #4 XSHUT        */
    12,               /* IND_FR - borrowed from ToF #3 XSHUT        */
    LIGHT_PIN_NONE,   /* IND_RL - the second pair, not wired yet    */
    LIGHT_PIN_NONE,   /* IND_RR                                     */
    LIGHT_PIN_NONE,   /* REV_L                                      */
    LIGHT_PIN_NONE    /* REV_R                                      */
};

/* ---- state, one copy - the same deal chassis.h makes -------------------- */
static Bool    up = false;
static Bool    on = true;    /* the master switch, for testing         */
static Set now;          /* what was last written                  */

/*
 * A lamp held on by hand, ignoring every rule and every cue. lights::COUNT means
 * "no override", which is the normal state.
 *
 * This exists because "the lamp does not work" has three unrelated causes - the
 * rule never fired, the pin never moved, or the LED is wired backwards - and
 * without a way to light one on demand there is no telling them apart.
 *
 * It lives HERE rather than in cue.h on purpose. It is a claim about the
 * hardware, not about what the car means, and putting it at the output means a
 * forced lamp survives whatever the cue layer is doing - including a one-shot
 * cue that would otherwise take the channel back off you mid-test.
 */
static Int32 forced = COUNT;

static Void clear(Set* s)
{
    for(Int32 i = 0; i < COUNT; ++i)
    {
        s->level[i] = LAMP_OFF;
    }
}

/* Straight at the pins, no gates. Everything below goes through this so there
 * is exactly one place that touches a GPIO. */
static Void push(const Set* s)
{
    for(Int32 i = 0; i < COUNT; ++i)
    {
        if(pin[i] != LIGHT_PIN_NONE)
        {
            gpio::write((Pin) pin[i], s->level[i] > LAMP_OFF);
        }
    }
    now = *s;
}

static Void open(Void)
{
    for(Int32 i = 0; i < COUNT; ++i)
    {
        if(pin[i] != LIGHT_PIN_NONE)
        {
            gpio::open((Pin) pin[i], PIN_DIR_OUT);
        }
    }

    up     = true;
    forced = COUNT;

    clear(&now);
    push(&now);
}

/*
 * Show this set of lamps.
 *
 * The two overrides live here rather than in the caller: a master switch that
 * only worked when the cue layer remembered to ask is a master switch, one day,
 * that does not.
 */
static Void write(const Set* s)
{
    if(!up || !on || s == NULL)
    {
        return;
    }

    if(forced != COUNT)
    {
        Set one;
        clear(&one);
        one.level[forced] = LAMP_FULL;
        push(&one);
        return;
    }

    push(s);
}

/* The master switch. Off parks every lamp dark rather than leaving whichever
 * happened to be lit when it was turned off. */
static Void enable(Bool on)
{
    on = on;
    if(!on)
    {
        Set dark;
        clear(&dark);
        push(&dark);
    }
}

static Bool enabled(Void)
{
    return on;
}

/* What each lamp is doing this instant. */
static Set read(Void)
{
    return now;
}

static Bool lit(Lamp l)
{
    return now.level[l] > LAMP_OFF;
}

/* Hold ONE lamp lit, or lights::COUNT to hand it back to the cue layer. */
static Void forceLamp(Int32 lamp)
{
    forced = lamp;
}

static Int32 forcedLamp(Void)
{
    return forced;
}


} // namespace lights