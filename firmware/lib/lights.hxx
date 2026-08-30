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
 * THE PINS ARE NOT CHOSEN HERE. lib/pins.hxx holds the car's whole map and
 * this file reads it. What follows is why the lamps sit where they do.
 *
 *     GP10-GP13 are ToF #1-#4 XSHUT, and all four are lamps: GP13/GP12 the
 *          front indicators, GP11/GP10 the headlights. They are free only
 *          because the I2C bus is empty - SCAN answers 0 - and they stop being
 *          free the moment a single ToF is fitted, not just the third and
 *          fourth.
 *
 *          Worth saying plainly because the hub's System panel still lists
 *          "ToF bumpers (GP10-13)" as a subsystem: that row describes where
 *          those sensors are GOING, and every one of those pins has an LED on
 *          it today.
 *
 *     GP14 AND GP15 WERE THE TAIL LAMPS AND ARE NOW THE DFPLAYER. On RP2350
 *          they are the only pads carrying UART0 that do not cost GP0/GP1 -
 *          the servo and the ESC - or GP12/GP13, the front indicators. Sound
 *          needed a UART and nothing else could give it one, so the tails are
 *          pins::NONE and simply are not written.
 *
 *          GP15 had also been earmarked for the WHEEL ENCODER. It cannot be
 *          all three; the encoder is now unassigned in pins.hxx and needs a
 *          pad chosen before it goes on.
 *
 * The permanent map is in docs/wiring.md: GP2/GP3 indicators, GP6/GP7 tails,
 * GP8 both heads. Moving there is editing pins.hxx and nothing else - which is
 * the entire reason the numbers left this file.
 *
 * FOUR LAMPS HAVE NO PIN AT ALL - both rear indicators and both reversing
 * lamps - and now the two tails as well. cue::solve() computes all ten
 * regardless and push() skips any lamp whose pin is NONE, so wiring one later
 * is a line in pins.hxx rather than a rule anywhere.
 * ===========================================================================
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.hxx"
#include "pins.hxx"

namespace bibo
{

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
/* One spelling of "no LED on this lamp", shared with the pin map so the two
 * cannot disagree. */
#define LIGHT_PIN_NONE pins::NONE

/* THE NUMBERS ARE NOT HERE ANY MORE. This is the order of the Lamp enum
 * mapped onto the car's pin map, and pins.hxx is where a GPIO is chosen.
 *
 * That move is what caught the collision it was written for: TAIL_L and TAIL_R
 * were GP15 and GP14, and those are the only two pads on RP2350 that carry
 * UART0 without taking GP0/GP1 from the servo and the ESC. The DFPlayer needed
 * them. With the numbers in two files nothing would have said so until a lamp
 * and a speaker fought on a breadboard; with one table it is a static_assert.
 *
 * Both tails read pins::NONE now and are simply not written - the same state
 * the rear indicators have been in since they were added. */
static Int32 pin[COUNT] =
{
    pins::HEAD_L,
    pins::HEAD_R,
    pins::TAIL_L,
    pins::TAIL_R,
    pins::IND_FL,
    pins::IND_FR,
    pins::IND_RL,
    pins::IND_RR,
    pins::REV_L,
    pins::REV_R
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
            gpio::write(static_cast<Pin>(pin[i]), s->level[i] > LAMP_OFF);
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
            gpio::open(static_cast<Pin>(pin[i]), PIN_DIR_OUT);
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
static Void enable(Bool state)
{
    /* Named `state`, not `on`: a parameter called `on` shadows the file-scope
     * switch, and `on = on` then assigns the parameter to itself. The switch
     * never latched, so an off parked the lamps once and the next cue lit them
     * again - while enabled() went on reporting true. */
    on = state;
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

} // namespace bibo