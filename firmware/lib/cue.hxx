/* ---------------------------------------------------------------------------
 * cue - what the car is SAYING.
 *
 * Everything the car emits at a person: which way it is about to turn, that it
 * is not being driven, that it has stopped itself, that it has seen you. Lamps
 * today, a buzzer later, and the point of the module is that the day the buzzer
 * arrives nothing above here changes.
 *
 * ---- why this is not just "lights" ----------------------------------------
 *
 * Indicators and brake lights were rules living inside lights.h, next to the
 * pin table. That reads fine until a second way of expressing something turns
 * up - a horn, a chirp, a headlight flash - and then either the lighting file
 * grows a sound API or the sound gets its own copy of "is the car turning".
 * Two files, both deciding what the car means, is one file too many.
 *
 * So the split is by JOB, not by hardware:
 *
 *   cue.h      decides what the car is expressing. Knows nothing about pins.
 *   lights.h   drives the lamps it is handed. Knows nothing about why.
 *
 * A cue reaches a person through one or more CHANNELS. A channel is a group of
 * lamps - the headlights, the tails, one side's indicators - or, eventually, a
 * tone. Cues are written against channels, never against individual lamps,
 * because "flash the headlights" is the intent and "set lamp 0 and lamp 1" is
 * an implementation of it that stops being true the moment a lamp moves.
 *
 * ---- two kinds of cue -----------------------------------------------------
 *
 * CONTINUOUS   what the car is doing right now, recomputed every tick from the
 *              drivetrain: indicating, not being driven, reversing, headlights.
 *              These are the rules that used to live in lights.h and they are
 *              unchanged - see cue::solve().
 *
 * ONE-SHOT     an utterance with a beginning and an end. A short script of
 *              steps, each naming which channels are lit and for how long. It
 *              plays OVER the continuous state and hands the channels it
 *              borrowed straight back.
 *
 * A one-shot cue OWNS every channel any of its steps mentions, for its whole
 * duration. That is what makes a headlight flash visible when the headlights
 * are already on: the cue takes the head channel, and its dark steps really are
 * dark. Channels it does not mention are left entirely alone, which is what
 * lets the indicators go on blinking through a flash.
 *
 * ---- calling it -----------------------------------------------------------
 *
 *     lights::open();                      the lamps, once
 *     cue::open();                         once
 *     cue::tick(&in);                      often, from the main loop
 *     cue::emit(cue::KIND_FLASH);           whenever the car has something to say
 *
 * Everything the rules read is passed IN rather than reached for, so this file
 * needs hal.h and lights.h and nothing else. Feed it the ACTUAL servo and ESC
 * output, not the targets: the slew limiter means the two differ for about a
 * second after every command, and a cue should follow the car rather than the
 * request.
 *
 * The CONTINUOUS rules are mirrored in hub/src/lights.hxx so they can be
 * watched on screen, and the constants are deliberately identical. One-shot
 * cues are not mirrored - the hub sees them the way a person does, in the lamp
 * levels the board reports.
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.hxx"
#include "lights.hxx"

namespace bibo
{

  namespace cue
  {

    /* ---- channels ------------------------------------------------------------
     *
     * What a cue can reach a person through. A bitmask, because an utterance is
     * routinely more than one thing at once - the alert below is both indicator
     * pairs AND the tails.
     *
     * Deliberately coarser than the lamp list. No cue wants the front left
     * indicator on its own; wanting one side is what CUE_CH_IND_L means, and which
     * lamps that turns out to be is lights.h's business.
     */
#define CUE_CH_HEAD  0x01u
#define CUE_CH_TAIL  0x02u
#define CUE_CH_IND_L 0x04u
#define CUE_CH_IND_R 0x08u
#define CUE_CH_REV   0x10u

#define CUE_CH_IND_BOTH (CUE_CH_IND_L | CUE_CH_IND_R)

    /* ---- tones ---------------------------------------------------------------
     *
     * NOTHING DRIVES THESE YET. There is no buzzer on the car and no pin set aside
     * for one, so cue::soundWrite() below records the tone and returns.
     *
     * They are declared now, and every cue script already carries a tone per step,
     * because the alternative is discovering later that cue::Step has no room for
     * sound and revising every script that exists by then. A field that is always
     * cue::TONE_NONE costs one byte per step and keeps the shape honest.
     */
    enum Tone
    {
        TONE_NONE = 0,
        TONE_LOW,
        TONE_MID,
        TONE_HIGH,
        TONE_COUNT
    };

    /* ---- the continuous rules' constants ------------------------------------ */

    /* Milli-units of travel. 450/280 are the hub's 0.45/0.28, on purpose. */
#define CUE_TURN_ON_MILLI  450
#define CUE_TURN_OFF_MILLI 280

    /* ---- the flash rate, and the standard it has to sit inside ---------------
     *
     * FMVSS 108 does not state a flash rate itself. It incorporates SAE J590 (turn
     * signal flashers) and SAE J945 (vehicular hazard warning flashers) BY
     * REFERENCE, and those carry the numbers:
     *
     *   60-120 flashes per minute   for a NORMALLY OPEN (variable load) flasher
     *   90-120 flashes per minute   for a NORMALLY CLOSED (fixed load) flasher
     *
     * plus a percent-current-on-time envelope - J945 Figure 1 - rather than a
     * single duty figure. The practical form of that envelope is that ON must not
     * be shorter than OFF.
     *
     * A 2002 NHTSA interpretation extends this to non-uniform flashers: any three
     * consecutive flashes have to fall inside the band, and the on-time rule always
     * applies. Nothing here is non-uniform, but it is why the band is about the
     * RATE ACHIEVED and not about the number written in a header.
     *
     * ---- what this was, and why it moved -----------------------------------
     *
     * 400 on / 267 off. That is 667 ms, which is 89.96 flashes per minute - inside
     * the normally-open band and 0.04 fpm UNDER the floor of the normally-closed
     * one. The comment beside it, here and in the hub, called 1.5 Hz "the legal
     * standard", and 1.5 Hz is 90 fpm exactly: a boundary quoted as a target, then
     * missed by rounding.
     *
     * 360 / 240 is 600 ms, 100.0 flashes per minute, 60% on. Inside BOTH bands with
     * margin at either end, on longer than off, and round numbers that cannot round
     * their way back out.
     *
     * This car is not road-legal and is never going on a road. The standard is used
     * here because it is a good one - it is what makes an indicator read as an
     * indicator to anybody who has ever driven - and not because anything requires
     * it of a 1/10 scale model.
     */
#define CUE_BLINK_ON_MS     360u
#define CUE_BLINK_OFF_MS    240u
#define CUE_BLINK_PERIOD_MS (CUE_BLINK_ON_MS + CUE_BLINK_OFF_MS)

    /*
     * THE STANDARD, ENFORCED BY THE BUILD rather than described in a comment above
     * two numbers somebody can edit.
     *
     * Stated as periods rather than rates so it is exact integer arithmetic with no
     * division and no rounding to argue about:
     *
     *     rate >=  90 fpm   <=>   60000 >=  90 * period   <=>   period <= 666 ms
     *     rate <= 120 fpm   <=>   60000 <= 120 * period   <=>   period >= 500 ms
     *
     * The tighter of the two bands, so satisfying this satisfies both.
     */
    static_assert(CUE_BLINK_PERIOD_MS >= 500u,
                  "flash rate over 120/min - faster than SAE J590/J945 allow");
    static_assert(CUE_BLINK_PERIOD_MS <= 666u,
                  "flash rate under 90/min - below the normally-closed band");
    static_assert(CUE_BLINK_ON_MS >= CUE_BLINK_OFF_MS,
                  "on-time shorter than off-time - outside the J945 Figure 1 envelope");

    enum Turn
    {
        TURN_OFF = 0,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_HAZARD
    };

    /* What the continuous rules read. */
    struct Input
    {
        Int32 steerMilli;   /* -1000..1000 of this car's travel, ACTUAL            */
        Int32 throttleUs;   /* what the ESC is actually being given                */
        Int32 idleUs;       /* the pulse at which this motor sits still (cal.h)    */
        Int32 neutralUs;    /* the pulse that is neither forward nor back          */
        Bool  armed;
        Bool  headOn;       /* nothing the car knows implies this; a human sets it */
    };

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
    inline Int32 motionUsNow = 10;

    /* Wide enough to be useful, narrow enough that a typo cannot switch the lamps
     * off for the whole usable throttle range. */
#define CUE_MOTION_US_MIN 0
#define CUE_MOTION_US_MAX 60

    inline Bool setMotionUs(Int32 us)
    {
        if(us < CUE_MOTION_US_MIN || us > CUE_MOTION_US_MAX)
        {
            return false;
        }
        motionUsNow = us;
        return true;
    }

    inline Int32 motionUs(Void)
    {
        return motionUsNow;
    }

    /* ---- what a cue IS -------------------------------------------------------
     *
     * One mechanism, three ways of playing, and that is the whole API:
     *
     *   PLAY_ONCE   run the script through `repeats` times and stop. An utterance
     *               with a beginning and an end - flash, alert.
     *   PLAY_LOOP   run it round until something cancels it. A blink whose rhythm
     *               is the script's own steps - the indicators.
     *   PLAY_HOLD   one state, held until cancelled. Not really a script at all,
     *               but expressed as one so there is nothing else to learn - the
     *               headlights, the brake lamps, reverse.
     *
     * This started as one-shots only, with the continuous behaviour - indicators,
     * brake, reverse - computed by hand in a solve() beside it. That split looked
     * reasonable and was not: it meant the car had two lighting systems, one you
     * could name and trigger and one you could not, and the second one grew every
     * time the car learned to say something. Everything is a cue now.
     */
    enum Play
    {
        PLAY_ONCE = 0,
        PLAY_LOOP,
        PLAY_HOLD
    };

    /*
     * One step of an utterance: hold these channels, at this brightness, for this
     * long, with this tone.
     *
     * A table rather than code, so adding a cue is adding data. The first thing
     * anyone will want to do with this module is invent a new noise for a new
     * situation, and that should not mean writing another state machine.
     *
     * `level` is what makes a running lamp and a brake lamp the SAME mechanism on
     * the same bulb: one is LAMP_DIM and one is LAMP_FULL, and which of them you
     * see is decided by priority rather than by an if inside a rule.
     */
    struct Step
    {
        UInt16 ms;
        UInt8  lamps;   /* CUE_CH_* bitmask lit during this step */
        UInt8  level;   /* LAMP_OFF..LAMP_FULL for the lamps above */
        UInt8  tone;    /* a Tone. Not driven yet - see above */
    };

    struct Script
    {
        CharSeq     name;    /* what to type at the console                     */
        CharSeq     means;   /* what the car is SAYING, in words                */
        const Step* step;
        UInt8       steps;
        UInt8       repeats; /* PLAY_ONCE only: how many times it runs          */
        UInt8       play;    /* a Play                                          */
    };

    /* ---- the scripts ---------------------------------------------------------
     *
     * HOLD scripts are one step with no duration. The ms is ignored - nothing
     * advances a held cue - and writing 0 says so.
     */

    /*
     * FLASH - the headlights, twice, quickly.
     *
     * TWO flashes and not one, deliberately. A single 90 ms blip is what a loose
     * connection looks like, and the whole value of a cue is that it reads as
     * something the car MEANT. Two says it on purpose. It is also what a driver
     * does to mean "I have seen you, go ahead", which is the meaning this one
     * carries.
     */
    static const Step STEPS_FLASH[] =
    {
        {  90u, CUE_CH_HEAD, LAMP_FULL, TONE_NONE },
        { 110u, 0u,          LAMP_OFF,  TONE_NONE }
    };

    /*
     * ALERT - both indicator pairs and the tails, three times.
     *
     * What the car says when it has stopped itself and nobody asked it to: the
     * deadman firing, which by definition happens when the host is not listening.
     * A message that only appears in a console is a message nobody standing next to
     * the car can read.
     *
     * Slower and heavier than FLASH on purpose. The two must not be confusable at
     * a glance - one means "after you" and the other means "something is wrong".
     */
    static const Step STEPS_ALERT[] =
    {
        { 160u, CUE_CH_IND_BOTH | CUE_CH_TAIL, LAMP_FULL, TONE_LOW  },
        { 160u, 0u,                            LAMP_OFF,  TONE_NONE }
    };

    /*
     * The indicators. 400 on, 267 off - not 50/50, because a slightly longer on
     * than off is what a real flasher can does and what the eye expects.
     *
     * HAZARD IS ITS OWN CUE rather than left and right together, and that is not
     * tidiness. Two cues have two step clocks, and two clocks started a
     * millisecond apart drift - so "both sides" assembled out of the two single
     * cues would come apart into an alternating flash, which is what a film prop
     * does and is the single most common way to get this wrong. One cue driving
     * both sides makes being in phase structural.
     *
     * Front and rear on a side share one flash for the same reason. The rear pair
     * has no LED on it yet; it is computed and reported anyway, which is what makes
     * wiring it later a change to the pin table and nothing else.
     */
    static const Step STEPS_LEFT[] =
    {
        { CUE_BLINK_ON_MS,  CUE_CH_IND_L, LAMP_FULL, TONE_NONE },
        { CUE_BLINK_OFF_MS, 0u,           LAMP_OFF,  TONE_NONE }
    };

    static const Step STEPS_RIGHT[] =
    {
        { CUE_BLINK_ON_MS,  CUE_CH_IND_R, LAMP_FULL, TONE_NONE },
        { CUE_BLINK_OFF_MS, 0u,           LAMP_OFF,  TONE_NONE }
    };

    static const Step STEPS_HAZARD[] =
    {
        { CUE_BLINK_ON_MS,  CUE_CH_IND_BOTH, LAMP_FULL, TONE_NONE },
        { CUE_BLINK_OFF_MS, 0u,              LAMP_OFF,  TONE_NONE }
    };

    /* Held states. */
    static const Step STEPS_HEAD[]    = { { 0u, CUE_CH_HEAD, LAMP_FULL, TONE_NONE } };
    static const Step STEPS_RUNNING[] = { { 0u, CUE_CH_TAIL, LAMP_DIM,  TONE_NONE } };
    static const Step STEPS_BRAKE[]   = { { 0u, CUE_CH_TAIL, LAMP_FULL, TONE_NONE } };
    static const Step STEPS_REVERSE[] = { { 0u, CUE_CH_REV,  LAMP_FULL, TONE_NONE } };

    /*
     * ORDER IS PRIORITY. A later cue wins a channel an earlier one also wants.
     *
     * Read down the list and it is a sentence about what matters more than what:
     * being lit at all, then which way the gearbox is, then which way the car is
     * turning, then the two things the car says on purpose. ALERT is last because
     * "something is wrong" outranks everything, including an indicator.
     *
     * RUNNING before BRAKE is the whole reason `level` exists: both want the tails,
     * BRAKE is later, so braking shows FULL over the DIM the running lamps were
     * holding. No rule anywhere has to know about the other.
     */
    enum Kind
    {
        KIND_NONE = 0,
        KIND_HEAD,
        KIND_RUNNING,
        KIND_BRAKE,
        KIND_REVERSE,
        KIND_LEFT,
        KIND_RIGHT,
        KIND_HAZARD,
        KIND_FLASH,
        KIND_ALERT,
        KIND_COUNT
    };

    /* IN cue::Kind ORDER. The enum indexes this table directly, so a row added in
     * the wrong place silently renames two cues at once. */
    static const Script SCRIPT[KIND_COUNT] =
    {
        { "none",    "nothing",                      nullptr,          0u, 0u, PLAY_HOLD },
        { "head",    "my headlights are on",         STEPS_HEAD,    1u, 0u, PLAY_HOLD },
        { "running", "I am lit but not braking",     STEPS_RUNNING, 1u, 0u, PLAY_HOLD },
        { "brake",   "I am not being driven",        STEPS_BRAKE,   1u, 0u, PLAY_HOLD },
        { "reverse", "I am backing up",              STEPS_REVERSE, 1u, 0u, PLAY_HOLD },
        { "left",    "I am turning left",            STEPS_LEFT,    2u, 0u, PLAY_LOOP },
        { "right",   "I am turning right",           STEPS_RIGHT,   2u, 0u, PLAY_LOOP },
        { "hazard",  "I am a hazard",                STEPS_HAZARD,  2u, 0u, PLAY_LOOP },
        { "flash",   "I have seen you - after you",  STEPS_FLASH,   2u, 2u, PLAY_ONCE },
        { "alert",   "I have stopped myself",        STEPS_ALERT,   2u, 3u, PLAY_ONCE }
    };

    /* ---- state, one copy - the same deal chassis.hxx makes ------------------
     *
     * PER KIND, because the car says more than one thing at a time. Headlights on,
     * braking, and indicating left is three cues at once and is an ordinary
     * Tuesday; the old single `kindNow` could hold exactly one of them.
     */
    inline Bool   up = false;

    inline Bool   active[KIND_COUNT];
    inline Bool   latched[KIND_COUNT];   /* raised by a person, not by the car */
    inline UInt8  stepIx[KIND_COUNT];
    inline UInt8  loopIx[KIND_COUNT];
    inline UInt64 stepAtUs[KIND_COUNT];

    inline UInt8  toneNow = TONE_NONE;

    /* The turn hysteresis, which decides what the CAR wants, not what is lit. */
    inline Turn   turnWant = TURN_OFF;
    inline UInt64 turnHoldUs = 0;

    inline Void open(Void)
    {
        lights::open();

        for(Int32 k = 0; k < KIND_COUNT; ++k)
        {
            active[k]   = false;
            latched[k]  = false;
            stepIx[k]   = 0;
            loopIx[k]   = 0;
            stepAtUs[k] = 0;
        }
        turnWant   = TURN_OFF;
        turnHoldUs = 0;
        toneNow    = TONE_NONE;
        up         = true;
    }

    inline Bool valid(Int32 k)
    {
        return k > KIND_NONE && k < KIND_COUNT;
    }

    inline Bool on(Kind k)
    {
        return valid(k) && active[k];
    }

    inline Bool held(Kind k)
    {
        return valid(k) && latched[k];
    }

    inline CharSeq name(Kind k)
    {
        return (k >= 0 && k < KIND_COUNT) ? SCRIPT[k].name : "?";
    }

    inline CharSeq means(Kind k)
    {
        return (k >= 0 && k < KIND_COUNT) ? SCRIPT[k].means : "?";
    }

    inline CharSeq playWord(UInt8 p)
    {
        switch(p)
        {
            case PLAY_ONCE: return "once";
            case PLAY_LOOP: return "loop";
            case PLAY_HOLD: return "hold";
            default:        return "?";
        }
    }

    /* The kind with this name, or cue::KIND_NONE. Case-insensitive, because every
     * other command word on this link is upper case by the time it arrives. */
    inline Kind find(CharSeq want)
    {
        if(want == nullptr)
        {
            return KIND_NONE;
        }

        for(Int32 k = 1; k < KIND_COUNT; ++k)
        {
            CharSeq a = SCRIPT[k].name;
            CharSeq b = want;
            Bool    same = true;

            while(*a != '\0' && *b != '\0')
            {
                Utf8 ca = *a;
                Utf8 cb = *b;
                if(ca >= 'A' && ca <= 'Z')
                {
                    ca = static_cast<Utf8>(ca + 32);
                }
                if(cb >= 'A' && cb <= 'Z')
                {
                    cb = static_cast<Utf8>(cb + 32);
                }
                if(ca != cb)
                {
                    same = false;
                    break;
                }
                ++a;
                ++b;
            }
            if(same && *a == '\0' && *b == '\0')
            {
                return static_cast<Kind>(k);
            }
        }
        return KIND_NONE;
    }

    /* ---- raising and lowering ------------------------------------------------ */

    inline Void start(Kind k, UInt64 now)
    {
        active[k]   = true;
        stepIx[k]   = 0;
        loopIx[k]   = 0;
        stepAtUs[k] = now + static_cast<UInt64>(SCRIPT[k].step[0].ms) * 1000u;
    }

    /*
     * Raised BY A PERSON, so it latches: the car's own rules will not lower it
     * again. That is what makes "headlights on" a switch rather than a suggestion
     * the next tick overrules.
     *
     * Left and right cancel each other. A car cannot indicate both ways - that is
     * what hazard is - and two blinkers with independent step clocks would drift
     * apart into an alternating flash within a few seconds.
     */
    inline Bool emit(Kind k)
    {
        if(!up || !valid(k) || SCRIPT[k].step == nullptr)
        {
            return false;
        }

        const UInt64 now = timing::nowUs();

        if(k == KIND_LEFT || k == KIND_RIGHT || k == KIND_HAZARD)
        {
            const Kind others[3] = { KIND_LEFT, KIND_RIGHT, KIND_HAZARD };
            for(Int32 i = 0; i < 3; ++i)
            {
                if(others[i] != k)
                {
                    active[others[i]]  = false;
                    latched[others[i]] = false;
                }
            }
        }

        start(k, now);
        latched[k] = true;
        return true;
    }

    /* Lowered by a person: stops it AND hands it back to the car's own rules. */
    inline Bool cancel(Kind k)
    {
        if(!up || !valid(k))
        {
            return false;
        }
        active[k]  = false;
        latched[k] = false;
        return true;
    }

    /* Stop mid-sentence and hand every borrowed channel back. */
    inline Void silence(Void)
    {
        for(Int32 k = 1; k < KIND_COUNT; ++k)
        {
            active[k]  = false;
            latched[k] = false;
        }
        toneNow = TONE_NONE;
    }

    /*
     * What the CAR wants, which a latched cue overrules.
     *
     * The asymmetry is the point: a person switching the headlights on means it
     * until they say otherwise, and the car noticing it is no longer braking must
     * not put them out.
     */
    inline Void wants(Kind k, Bool want, UInt64 now)
    {
        if(latched[k])
        {
            return;
        }
        if(want && !active[k])
        {
            start(k, now);
        }
        else if(!want && active[k])
        {
            active[k] = false;
        }
    }

    /* ---- reporting ----------------------------------------------------------- */

    /*
     * The most important thing being said, for a one-line status.
     *
     * The LAST active kind, because the enum is in priority order - so this answers
     * with what a person looking at the car would notice first, not with whichever
     * happened to be raised earliest.
     */
    inline Kind speaking(Void)
    {
        for(Int32 k = KIND_COUNT - 1; k > KIND_NONE; --k)
        {
            if(active[k])
            {
                return static_cast<Kind>(k);
            }
        }
        return KIND_NONE;
    }

    inline Bool busy(Void)
    {
        return speaking() != KIND_NONE;
    }

    inline UInt8 step(Void)
    {
        const Kind k = speaking();
        return (k == KIND_NONE) ? 0u : stepIx[k];
    }

    inline UInt8 loop(Void)
    {
        const Kind k = speaking();
        return (k == KIND_NONE) ? 0u : loopIx[k];
    }

    /* Which way the car is indicating. Derived from the cues rather than kept
     * beside them, so there is one answer and not two that can disagree. */
    inline Turn side(Void)
    {
        if(active[KIND_HAZARD])
        {
            return TURN_HAZARD;
        }
        if(active[KIND_LEFT])
        {
            return TURN_LEFT;
        }
        if(active[KIND_RIGHT])
        {
            return TURN_RIGHT;
        }
        return TURN_OFF;
    }

    /*
     * Which lamps a channel is, in one place. A cue names the channel; this is the
     * only thing that knows which bulbs that turns out to be.
     *
     * ---- WHY ALL FOUR INDICATORS CANNOT DRIFT -------------------------------
     *
     * A side is ONE channel and both its lamps are written from ONE step, in one
     * assignment pair, so front-left and rear-left are the same value by
     * construction rather than by two timers agreeing. There is no clock per lamp
     * to drift, and there is no arrangement of the code in which they differ.
     *
     * The same argument covers hazards: CUE_CH_IND_BOTH is one channel mask read
     * from one step of one cue, so all four are written together and are in phase
     * structurally. That is why hazard is its own cue rather than left and right
     * raised at once - two cues have two step clocks, and two clocks started a
     * millisecond apart come apart into an alternating flash, which is what a film
     * prop does.
     *
     * The rear pair has no pin bound yet. It is computed and REPORTED anyway, which
     * is what let this be measured before the LEDs exist: LIGHTS shows all ten
     * levels, and front and rear matched on every sample of both cues. Wiring them
     * is a change to the pin table in lights.hxx and nothing else.
     */
    inline Void channelLamps(UInt8 ch, UInt8 level, lights::Set* out)
    {
        if(ch & CUE_CH_HEAD)
        {
            out->level[lights::LAMP_HEAD_L] = level;
            out->level[lights::LAMP_HEAD_R] = level;
        }
        if(ch & CUE_CH_TAIL)
        {
            out->level[lights::LAMP_TAIL_L] = level;
            out->level[lights::LAMP_TAIL_R] = level;
        }
        if(ch & CUE_CH_IND_L)
        {
            out->level[lights::LAMP_IND_FL] = level;
            out->level[lights::LAMP_IND_RL] = level;
        }
        if(ch & CUE_CH_IND_R)
        {
            out->level[lights::LAMP_IND_FR] = level;
            out->level[lights::LAMP_IND_RR] = level;
        }
        if(ch & CUE_CH_REV)
        {
            out->level[lights::LAMP_REV_L] = level;
            out->level[lights::LAMP_REV_R] = level;
        }
    }

    /*
     * Sound. A seam, not an implementation.
     *
     * There is no buzzer on this car and no pin set aside for one. This records
     * what the running cue WOULD be sounding so CUE can report it, and drives
     * nothing. When a buzzer goes on, this function grows a pwm tone call and no
     * script changes - which is the entire reason the tone is in cue::Step now
     * rather than being added to it later.
     */
    inline Void soundWrite(UInt8 tone)
    {
        toneNow = tone;
    }

    /*
     * Advance one cue's script, and stop it if it has finished.
     *
     * A while, not an if, and the deadline ACCUMULATES rather than being restarted
     * from `now`.
     *
     * Both halves of that matter and they are the same point. Restarting from `now`
     * would add a whole pass of the main loop to every step, so a two-step cue
     * repeated three times would run six loop-periods long - and it would make the
     * while unreachable, because the new deadline would always be in the future.
     * Accumulating keeps the cue the length the script says, and lets the loop
     * catch up honestly if something upstream blocked: serial::printf can sit for
     * half a second when the host stops draining the port, and a cue should have
     * PLAYED during that, not be waiting to.
     */
    inline Void advance(Int32 k, UInt64 now)
    {
        const Script* sc = &SCRIPT[k];

        if(sc->play == PLAY_HOLD)
        {
            return;   /* nothing to advance; it is one state until cancelled */
        }

        while(active[k] && now >= stepAtUs[k])
        {
            ++stepIx[k];
            if(stepIx[k] >= sc->steps)
            {
                stepIx[k] = 0;
                ++loopIx[k];

                if(sc->play == PLAY_ONCE && loopIx[k] >= sc->repeats)
                {
                    /* A one-shot that has finished is finished - it does not stay
                     * latched waiting for somebody to lower it. */
                    active[k]  = false;
                    latched[k] = false;
                    return;
                }
            }
            stepAtUs[k] += static_cast<UInt64>(sc->step[stepIx[k]].ms) * 1000u;
        }
    }

    /*
     * Everything active, composited in priority order.
     *
     * A cue OWNS every channel any of its steps mentions, for its whole duration,
     * including the steps where that channel is dark. Without that a flash would be
     * invisible whenever the headlights were already on - the cue's on-steps would
     * agree with whatever was underneath and its off-steps would be overwritten by
     * it.
     *
     * That ownership is also what makes priority mean something. A lower cue writes
     * its channels; a higher one writes over them, lit or dark, and the result is
     * the higher cue's opinion in full rather than a blend of two.
     */
    inline Void compose(UInt64 now, lights::Set* out)
    {
        lights::clear(out);
        UInt8 tone = TONE_NONE;

        for(Int32 k = 1; k < KIND_COUNT; ++k)
        {
            if(!active[k])
            {
                continue;
            }

            advance(k, now);
            if(!active[k])
            {
                continue;   /* it ended on this tick */
            }

            const Script* sc = &SCRIPT[k];

            UInt8 owned = 0u;
            for(UInt8 i = 0; i < sc->steps; ++i)
            {
                owned = static_cast<UInt8>(owned | sc->step[i].lamps);
            }

            const Step* st = &sc->step[stepIx[k]];

            channelLamps(static_cast<UInt8>(owned & ~st->lamps), LAMP_OFF, out);
            channelLamps(st->lamps, st->level, out);

            if(st->tone != TONE_NONE)
            {
                tone = st->tone;
            }
        }

        soundWrite(tone);
    }

    /* Call often. Cheap when there is nothing to do. */
    inline Void tick(const Input* in)
    {
        if(!up || in == nullptr)
        {
            return;
        }

        const UInt64 now = timing::nowUs();

        /* ---- which way, with hysteresis and a minimum flash ------------------
         *
         * Three things here look right written down and are wrong on a car:
         *   1. TWO thresholds. Steering parked on a single one makes the lamp
         *      stutter at the servo's own jitter.
         *   2. ONE COMPLETE FLASH minimum, or a turn starting and ending inside one
         *      blink period shows a sliver of an on-phase and vanishes.
         *   3. ...EXCEPT a change of side, which is immediate. Indicating left
         *      while the wheels go right is the one thing an indicator must never
         *      do.
         */
        const Int32 mag = (in->steerMilli < 0) ? -in->steerMilli : in->steerMilli;

        const Turn want = (in->steerMilli <= -CUE_TURN_ON_MILLI) ? TURN_LEFT
                           : (in->steerMilli >=  CUE_TURN_ON_MILLI) ? TURN_RIGHT
                                                                    : TURN_OFF;

        if(want != TURN_OFF && want != turnWant)
        {
            turnWant   = want;
            turnHoldUs = now + static_cast<UInt64>(CUE_BLINK_PERIOD_MS) * 1000u;
        }
        else if(turnWant != TURN_OFF
                && mag < CUE_TURN_OFF_MILLI
                && now >= turnHoldUs)
        {
            turnWant = TURN_OFF;
        }

        /* ---- what the car wants to say --------------------------------------
         *
         * Every one of these is a REQUEST, overruled by anything a person has
         * latched. The car's opinion and the operator's are the same mechanism with
         * different priority, which is what stops "headlights on" being a special
         * case bolted to the side of the lighting rules.
         */
        wants(KIND_LEFT,  turnWant == TURN_LEFT  && !active[KIND_HAZARD], now);
        wants(KIND_RIGHT, turnWant == TURN_RIGHT && !active[KIND_HAZARD], now);

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
        const Bool fwd    = (in->throttleUs > (in->idleUs + motionUsNow));
        const Bool rev    = (in->throttleUs < (in->neutralUs - motionUsNow));
        const Bool driven = fwd || rev;

        /*
         * The tails, as two cues on one pair of lamps.
         *
         * RUNNING is the dim glow that goes with the headlights. BRAKE is full, and
         * wins because it is later in the enum. Neither knows about the other, and
         * the thing that used to be a nested conditional inside one rule is now the
         * ORDER of two.
         *
         * NO OVERRIDE from the indicators. They were interrupted until 2026-08-28,
         * and the reason was a real convention applied to the wrong car: on many
         * cars the rear indicator and the brake light are one bulb, so the indicator
         * has to interrupt the brake to be seen at all. This car has separate LEDs
         * on separate pins and a second indicator pair going on the rear, so nothing
         * is ever competing for one bulb. Applied anyway, it made the brake light
         * blink in antiphase to the signal beside it, which is exactly what a brake
         * light must not do. If a shared-bulb cluster is ever fitted, that belongs
         * in the BINDING - two lamps on one pin - and not back in here.
         */
        wants(KIND_BRAKE, !driven, now);

        /*
         * Reverse lamps: lit while the car is being driven BACKWARDS.
         *
         * chassis.hxx is forward-only today - it clamps the throttle to [idle, max]
         * and the board refuses anything below neutral - so `rev` is currently
         * always false and these lamps never light. The rule is here anyway, and
         * that is deliberate: reverse arrives as a brake-then-reverse sequence in
         * the ESC, and when it does the lighting should already be right rather than
         * being the thing somebody remembers afterwards.
         */
        wants(KIND_REVERSE, rev, now);

        /* The headlights are the one thing the car cannot know for itself - nothing
         * it measures implies darkness - so `headOn` is a person's answer arriving
         * through the Input, and latching it with CUE HEAD is the other way to say
         * the same thing.
         *
         * RESOLVED BEFORE RUNNING, and that order is load-bearing: the running
         * lamps follow the HEADLIGHTS, not the Input. Reading in->headOn for both
         * left CUE HEAD lighting the heads and not the tails - the operator's
         * switch worked on one of the two things a headlight switch does, which is
         * the sort of half-working that gets blamed on wiring. */
        wants(KIND_HEAD, in->headOn, now);

        /* Dim tails whenever the car is lit at all. See the note above the brake:
         * this is the lower half of one pair of lamps and BRAKE is the upper. */
        wants(KIND_RUNNING, active[KIND_HEAD], now);

        /*
         * Composed EVERY tick, even with the lamps switched off.
         *
         * The old tick returned early when the master switch was off, which froze
         * the turn state machine with it - so LIGHTS went on reporting whichever way
         * the car had been turning when somebody killed the lamps. Only the WRITE is
         * gated now, in lights.hxx. What the car means is true whether or not
         * anything is lit to say it.
         */
        lights::Set s;
        compose(now, &s);
        lights::write(&s);
    }

    /* What the running cue is sounding, for anything that reports it. Always
     * cue::TONE_NONE until a buzzer exists - see cue::soundWrite(). */
    inline UInt8 tone(Void)
    {
        return toneNow;
    }

  } // namespace cue

} // namespace bibo