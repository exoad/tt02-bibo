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

namespace bibo::lights
{

    /* ---- the lamps a car has ------------------------------------------------ */
    enum Lamp
    {
        LAMP_HEAD_L = 0,
        LAMP_HEAD_R,
        LAMP_TAIL_L,
        LAMP_TAIL_R,
        LAMP_IND_FL,
        LAMP_IND_FR,
        LAMP_IND_RL,
        LAMP_IND_RR,
        LAMP_REV_L,
        LAMP_REV_R,
        LAMP_COUNT
    };

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

    struct Set
    {
        UInt8 level[LAMP_COUNT];
    };

    /* ===========================================================================
     * THE BINDING. Temporary - see the banner.
     * ======================================================================== */
    /* One spelling of "no LED on this lamp", shared with the pin map so the two
     * cannot disagree. */
#define LIGHT_PIN_NONE pins::NONE

    /* FILLED AT open(), FROM THE INSTALLED MAP - not initialized here.
     *
     * It held literal GPIO numbers, then it held pins:: constants, and both had
     * the same flaw: the binding was fixed when the firmware was COMPILED. A
     * sketch that wanted the lamps somewhere else had to edit the car.
     *
     * Now the program says what is wired where - pins::begin() - and this table
     * is a copy taken when the lamps are opened. Same ten entries, same order
     * as the Lamp enum, and still NONE for anything with no LED on it. */
    inline Int32 pin[LAMP_COUNT] =
    {
        pins::NONE, pins::NONE, pins::NONE, pins::NONE, pins::NONE,
        pins::NONE, pins::NONE, pins::NONE, pins::NONE, pins::NONE
    };

    /* ---- state, one copy - the same deal chassis.h makes -------------------- */
    inline Bool    up = false;
    inline Bool    on = true;    /* the master switch, for testing         */
    inline Set now;          /* what was last written                  */

    /*
     * A lamp held on by hand, ignoring every rule and every cue. lights::LAMP_COUNT means
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
    inline Int32 forced = LAMP_COUNT;

    /**
     * @brief Darkens every lamp in a set.
     *
     * Operates on the caller's set rather than on the live one, so it is also
     * how a temporary set is initialized before a single lamp is raised in it.
     *
     * @param s the set to darken; every level becomes LAMP_OFF
     */
    inline Void clear(Set* s)
    {
        for(Utf8Byte& i : s->level)
        {
            i = LAMP_OFF;
        }
    }

    /**
     * @brief Writes a set straight at the pins, with no gating.
     *
     * Everything below goes through this, so there is exactly one place in the
     * module that touches a GPIO. A lamp bound to LIGHT_PIN_NONE is skipped -
     * which is how a lamp with no LED behind it stays in the model, and is
     * computed and reported, without anything being written for it.
     *
     * @param s the levels to write; also becomes what read() reports
     */
    inline Void push(const Set* s)
    {
        for(Int32 i = 0; i < LAMP_COUNT; ++i)
        {
            if(pin[i] != LIGHT_PIN_NONE)
            {
                gpio::write(pin[i], s->level[i] > LAMP_OFF);
            }
        }
        now = *s;
    }

    /**
     * @brief Takes the installed pin map and opens every bound lamp for output.
     *
     * The map is COPIED rather than read through on every write: push() runs
     * every tick, and a lamp table that could change underneath it is a race
     * nobody needs.
     *
     * @note pins::begin() must have run first. Called before it, every lamp
     *       binds to LIGHT_PIN_NONE and nothing is ever written.
     */
    inline Void open(Void)
    {
        const pins::Map& m = pins::active();

        pin[LAMP_HEAD_L] = m.headL;
        pin[LAMP_HEAD_R] = m.headR;
        pin[LAMP_TAIL_L] = m.tailL;
        pin[LAMP_TAIL_R] = m.tailR;
        pin[LAMP_IND_FL] = m.indFL;
        pin[LAMP_IND_FR] = m.indFR;
        pin[LAMP_IND_RL] = m.indRL;
        pin[LAMP_IND_RR] = m.indRR;
        pin[LAMP_REV_L]  = m.revL;
        pin[LAMP_REV_R]  = m.revR;

        for(const Int64 i : pin)
        {
            if(i != LIGHT_PIN_NONE)
            {
                gpio::open(i, PIN_DIR_OUT);
            }
        }

        up     = true;
        forced = LAMP_COUNT;

        clear(&now);
        push(&now);
    }

    /*
     * @brief Shows a set of lamps, honoring the master switch and any override.
     *
     * The two overrides are applied HERE rather than left to the caller: a
     * master switch that only worked when the cue layer remembered to ask is a
     * master switch that one day does not.
     *
     * Does nothing before open(), while disabled, or on a null set. When a lamp
     * is forced, that lamp alone is lit and `s` is ignored entirely.
     *
     * @param s the lamps to show; ignored while an override is held
     */
    inline Void write(const Set* s)
    {
        if(!up || !on || s == nullptr)
        {
            return;
        }

        if(forced != LAMP_COUNT)
        {
            Set one{};
            clear(&one);
            one.level[forced] = LAMP_FULL;
            push(&one);
            return;
        }

        push(s);
    }

    /**
     * @brief The master switch. Off parks every lamp dark.
     *
     * Turning it off writes darkness immediately rather than leaving whichever
     * lamps happened to be lit at that moment.
     *
     * @param state true to let write() through, false to park the lamps
     *
     * @note The parameter is `state` and not `on` on purpose. Named `on` it
     *       shadows the file-scope switch, `on = on` assigns the parameter to
     *       itself, and the switch never latches - so an off parked the lamps
     *       once, the next cue lit them again, and enabled() went on reporting
     *       true throughout.
     */
    inline Void enable(const Bool state)
    {
        on = state;
        if(!on)
        {
            Set dark{};
            clear(&dark);
            push(&dark);
        }
    }

    /**
     * @brief Whether the master switch is on.
     *
     * @return true when write() is allowed through to the pins
     */
    inline Bool enabled(Void)
    {
        return on;
    }

    /**
     * @brief What every lamp is doing this instant.
     *
     * Reports what was last WRITTEN, including lamps bound to
     * LIGHT_PIN_NONE - those are computed and reported but never driven, so a
     * lamp can read as lit with no LED on the car.
     *
     * @return a copy of the live set
     */
    inline Set read(Void)
    {
        return now;
    }

    /**
     * @brief Whether one lamp is lit.
     *
     * @param l the lamp to ask about
     * @return true when its level is above LAMP_OFF
     */
    inline Bool lit(const Lamp l)
    {
        return now.level[l] > LAMP_OFF;
    }

    /**
     * @brief Holds ONE lamp lit, ignoring every rule and cue.
     *
     * Exists because "the lamp does not work" has three unrelated causes - the
     * rule never fired, the pin never moved, or the LED is wired backwards -
     * and without a way to light one on demand there is no telling them apart.
     *
     * @param lamp the lamp to hold, or lights::LAMP_COUNT to release it back
     *             to the cue layer
     */
    inline Void forceLamp(const Int32 lamp)
    {
        forced = lamp;
    }

    /**
     * @brief Which lamp is being held, if any.
     *
     * @return the forced lamp, or lights::LAMP_COUNT when nothing is held
     */
    inline Int32 forcedLamp(Void)
    {
        return forced;
    }


}
