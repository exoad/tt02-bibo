/*
 * ---------------------------------------------------------------------------
 * lights - the lamps the car has, and which pins they are on today.
 *
 * THE OUTPUT LAYER, and nothing else: what a lamp is, which GPIO it is soldered
 * to, and how to write a set of brightnesses out. WHY a lamp is lit is cue.h.
 *
 * The car has ten lamps in the model whether or not ten LEDs exist. A lamp with
 * no pin bound is simply not written - cue::solve() still computes it and LIGHTS
 * still reports it - so wiring one later is a line in pins.hxx rather than a rule
 * anywhere. Six have no pin today: both rear indicators, both reversing lamps,
 * both tails.
 *
 * THE PINS ARE NOT CHOSEN HERE. lib/pins.hxx holds the car's whole map and
 * this file reads it. Two entries in it are worth knowing about:
 *
 *     GP10-GP13 are ToF #1-#4 XSHUT and all four are lamps today: GP13/GP12 the
 *          front indicators, GP11/GP10 the headlights. They are free only while
 *          the I2C bus is empty, and stop being free the moment a ToF is fitted.
 *
 *     GP14 and GP15 were the tail lamps and are now the DFPlayer - the only pads
 *          carrying UART0 on RP2350 that cost neither GP0/GP1 nor the front
 *          indicators - so the tails are pins::NONE. GP15 was earmarked for the
 *          wheel encoder too, which is now unassigned in pins.hxx.
 *
 * The permanent map is in docs/wiring.md: GP2/GP3 indicators, GP6/GP7 tails,
 * GP8 both heads. Moving there is editing pins.hxx and nothing else.
 * -------------------------------------------------------------------------
 */
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
     * Brightness, 0 dark to 255 full: a level and not a Bool because the tails are ONE
     * lamp at two brightnesses. Nothing drives PWM yet, so a bound pin is (level > 0).
     */
#define LAMP_OFF   0u
#define LAMP_DIM   76u    /* 30%: tail */
#define LAMP_DRL   115u   /* 45%: daytime running */
#define LAMP_FULL  255u

    struct Set
    {
        UInt8 level[LAMP_COUNT];
    };

    /* ---- the binding, temporary - see the banner. NONE is "no LED here" ----- */
#define LIGHT_PIN_NONE pins::NONE

    /*
     * FILLED AT open(), FROM THE INSTALLED MAP - not initialized here. pins::begin()
     * says what is wired where and this is a copy taken when the lamps are opened:
     * same ten entries, same order as the Lamp enum, NONE for anything unwired.
     */
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
     * A lamp held on by hand, ignoring every rule and cue; LAMP_COUNT means "no
     * override". It lives at the output so a forced lamp survives the cue layer.
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
     * The two overrides are applied HERE, not left to the caller: a master switch
     * the cue layer has to remember to ask about is one that one day does not work.
     * Nothing happens before open(), while disabled, or on a null set; a forced
     * lamp is lit alone and `s` is ignored.
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
