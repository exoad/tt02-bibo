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
typedef enum
{
    TONE_NONE = 0,
    TONE_LOW,
    TONE_MID,
    TONE_HIGH,
    TONE_COUNT
} Tone;

/* ---- the continuous rules' constants ------------------------------------ */

/* Milli-units of travel. 450/280 are the hub's 0.45/0.28, on purpose. */
#define CUE_TURN_ON_MILLI  450
#define CUE_TURN_OFF_MILLI 280

/* 1.5 Hz. 400 on, 267 off - deliberately not 50/50, because a slightly longer
 * on than off is what a real flasher can does and what the eye expects. */
#define CUE_BLINK_ON_MS     400u
#define CUE_BLINK_OFF_MS    267u
#define CUE_BLINK_PERIOD_MS (CUE_BLINK_ON_MS + CUE_BLINK_OFF_MS)

typedef enum
{
    TURN_OFF = 0,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_HAZARD
} Turn;

/* What the continuous rules read. */
typedef struct
{
    Int32 steerMilli;   /* -1000..1000 of this car's travel, ACTUAL            */
    Int32 throttleUs;   /* what the ESC is actually being given                */
    Int32 idleUs;       /* the pulse at which this motor sits still (cal.h)    */
    Int32 neutralUs;    /* the pulse that is neither forward nor back          */
    Bool  armed;
    Bool  headOn;       /* nothing the car knows implies this; a human sets it */
} Input;

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
static Int32 motionUsNow = 10;

/* Wide enough to be useful, narrow enough that a typo cannot switch the lamps
 * off for the whole usable throttle range. */
#define CUE_MOTION_US_MIN 0
#define CUE_MOTION_US_MAX 60

static Bool setMotionUs(Int32 us)
{
    if(us < CUE_MOTION_US_MIN || us > CUE_MOTION_US_MAX)
    {
        return false;
    }
    motionUsNow = us;
    return true;
}

static Int32 motionUs(Void)
{
    return motionUsNow;
}

/* ---- one-shot cues ------------------------------------------------------- */

/*
 * One step of an utterance: hold these channels for this long, with this tone.
 *
 * A table rather than code, so adding a cue is adding data. The first thing
 * anyone will want to do with this module is invent a new noise for a new
 * situation, and that should not mean writing another state machine.
 */
typedef struct
{
    UInt16 ms;
    UInt8  lamps;   /* CUE_CH_* bitmask lit during this step */
    UInt8  tone;    /* a Tone. Not driven yet - see above */
} Step;

typedef struct
{
    CharSeq        name;    /* what to type at the console                     */
    CharSeq        means;   /* what the car is SAYING, in words                */
    const Step* step;
    UInt8          steps;
    UInt8          repeats; /* how many times the whole script runs            */
} Script;

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
    {  90u, CUE_CH_HEAD, TONE_NONE },
    { 110u, 0u,          TONE_NONE }
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
    { 160u, CUE_CH_IND_BOTH | CUE_CH_TAIL, TONE_LOW  },
    { 160u, 0u,                            TONE_NONE }
};

typedef enum
{
    KIND_NONE = 0,
    KIND_FLASH,
    KIND_ALERT,
    KIND_COUNT
} Kind;

/* IN cue::Kind ORDER. The enum indexes this table directly, so a row added in the
 * wrong place silently renames two cues at once. */
static const Script SCRIPT[KIND_COUNT] =
{
    { "none",  "nothing",                     NULL,            0u, 0u },
    { "flash", "I have seen you - after you", STEPS_FLASH, 2u, 2u },
    { "alert", "I have stopped myself",       STEPS_ALERT, 2u, 3u }
};

/* ---- state, one copy - the same deal chassis.h makes -------------------- */
static Bool    up        = false;

static Turn turnNow   = TURN_OFF;
static UInt64  holdUs    = 0;      /* earliest the turn may stop           */
static Bool    blinkOn   = false;  /* which half of the blink we are in    */
static UInt64  blinkAtUs = 0;

static Kind kindNow   = KIND_NONE;
static UInt8   stepIx    = 0;
static UInt8   loopIx    = 0;
static UInt64  stepAtUs  = 0;      /* when the current step ends           */
static UInt8   toneNow   = TONE_NONE;

static Void open(Void)
{
    up        = true;
    turnNow   = TURN_OFF;
    holdUs    = 0;
    blinkOn   = false;
    blinkAtUs = 0;
    kindNow   = KIND_NONE;
    stepIx    = 0;
    loopIx    = 0;
    stepAtUs  = 0;
    toneNow   = TONE_NONE;
}

/* Which way the car reckons it is turning. */
static Turn side(Void)
{
    return turnNow;
}

static Kind speaking(Void)
{
    return kindNow;
}

static Bool busy(Void)
{
    return kindNow != KIND_NONE;
}

/* How far through it is. For anything that REPORTS a cue - without these the
 * console would be reaching straight into this module's state, which is the
 * habit the whole layering here exists to stop. */
static UInt8 step(Void)
{
    return stepIx;
}

static UInt8 loop(Void)
{
    return loopIx;
}

static CharSeq name(Kind k)
{
    return (k >= 0 && k < KIND_COUNT) ? SCRIPT[k].name : "?";
}

static CharSeq means(Kind k)
{
    return (k >= 0 && k < KIND_COUNT) ? SCRIPT[k].means : "?";
}

/* The kind with this name, or cue::KIND_NONE. Case-insensitive, because every
 * line reaching the console has already been uppercased. */
static Kind find(CharSeq name)
{
    for(Int32 k = 1; k < KIND_COUNT; ++k)
    {
        CharSeq a = SCRIPT[k].name;
        Int32   i = 0;

        while(a[i] != '\0' && name[i] != '\0')
        {
            const Utf8 x = (Utf8) ((a[i] >= 'a' && a[i] <= 'z') ? (a[i] - 32) : a[i]);
            const Utf8 y = (Utf8) ((name[i] >= 'a' && name[i] <= 'z') ? (name[i] - 32) : name[i]);
            if(x != y)
            {
                break;
            }
            ++i;
        }

        if(a[i] == '\0' && name[i] == '\0')
        {
            return (Kind) k;
        }
    }
    return KIND_NONE;
}

/*
 * Say something. Starts at once, from the beginning, even if a cue is already
 * running.
 *
 * The newest utterance wins rather than being queued behind the old one. A cue
 * is a statement about the car RIGHT NOW - "I have stopped myself" is not worth
 * hearing four hundred milliseconds late, and a queue would mean the car
 * finishing a pleasantry before mentioning a fault.
 */
static Bool emit(Kind k)
{
    if(!up || k <= KIND_NONE || k >= KIND_COUNT)
    {
        return false;
    }
    if(SCRIPT[k].steps == 0u || SCRIPT[k].step == NULL)
    {
        return false;
    }

    kindNow  = k;
    stepIx   = 0;
    loopIx   = 0;
    stepAtUs = timing::nowUs() + static_cast<UInt64>(SCRIPT[k].step[0].ms) * 1000u;
    return true;
}

/* Stop mid-sentence and hand every borrowed channel back. */
static Void silence(Void)
{
    kindNow = KIND_NONE;
    stepIx  = 0;
    loopIx  = 0;
    toneNow = TONE_NONE;
}

/*
 * THE CONTINUOUS RULES. Pure: same input and same clock, same answer, no
 * hardware.
 *
 * Split out so it can be reasoned about - and eventually tested on the host,
 * the way lib/text.h is - without a Pico in the loop. The blink phase is passed
 * in for the same reason.
 */
static Void solve(const Input* in, Turn turn, Bool blink, lights::Set* out)
{
    lights::clear(out);

    const Bool left  = (turn == TURN_LEFT)  || (turn == TURN_HAZARD);
    const Bool right = (turn == TURN_RIGHT) || (turn == TURN_HAZARD);

    /*
     * Hazards are BOTH sides IN PHASE, not alternating. Alternating is what a
     * film prop does and is the single most common way to get this wrong; both
     * sides reading one `blink` makes being in phase structural.
     *
     * FRONT and REAR on a side share ONE flash for the same reason. Two
     * indicators on the same corner of the same car blinking a frame apart is
     * instantly, obviously wrong, and per-lamp timers drift.
     *
     * The rear pair has no LED on it yet. It is computed and reported anyway,
     * which is what makes wiring it later a change to the pin table and nothing
     * else.
     */
    const UInt8 amberL = (left  && blink) ? LAMP_FULL : LAMP_OFF;
    const UInt8 amberR = (right && blink) ? LAMP_FULL : LAMP_OFF;

    out->level[lights::IND_FL] = amberL;
    out->level[lights::IND_RL] = amberL;
    out->level[lights::IND_FR] = amberR;
    out->level[lights::IND_RR] = amberR;

    const UInt8 head = in->headOn ? LAMP_FULL : LAMP_OFF;
    out->level[lights::HEAD_L] = head;
    out->level[lights::HEAD_R] = head;

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

    /* One red lamp at two brightnesses. Braking wins over running: it is the
     * more urgent claim. */
    const UInt8 red = driven ? (in->headOn ? LAMP_DIM : LAMP_OFF) : LAMP_FULL;

    out->level[lights::TAIL_L] = red;
    out->level[lights::TAIL_R] = red;

    /*
     * NO OVERRIDE. The tails are not interrupted by anything.
     *
     * They were, until 2026-08-28, and the reason was a real convention applied
     * to the wrong car. On many cars the REAR indicator and the brake light are
     * one bulb, so the indicator has to interrupt the brake to be seen at all -
     * and that interruption is what makes such a car read as a car.
     *
     * This car does not have that bulb. The tails and the indicators are
     * separate LEDs on separate pins, and a second pair of indicators is going
     * on the rear, so nothing here is ever competing for the same lamp. Applied
     * anyway, it made the brake light blink in antiphase to the signal beside
     * it, which is exactly what a brake light must not do.
     *
     * If a shared-bulb cluster is ever fitted, the override belongs in the
     * BINDING - two lamps mapped to one pin - and not back in here.
     */

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
    out->level[lights::REV_L] = white;
    out->level[lights::REV_R] = white;
}

/* Which lamps a channel is, in one place. A cue names the channel; this is the
 * only thing that knows which bulbs that turns out to be. */
static Void channelLamps(UInt8 ch, Bool lit, lights::Set* out)
{
    const UInt8 on = lit ? LAMP_FULL : LAMP_OFF;

    if(ch & CUE_CH_HEAD)
    {
        out->level[lights::HEAD_L] = on;
        out->level[lights::HEAD_R] = on;
    }
    if(ch & CUE_CH_TAIL)
    {
        out->level[lights::TAIL_L] = on;
        out->level[lights::TAIL_R] = on;
    }
    if(ch & CUE_CH_IND_L)
    {
        out->level[lights::IND_FL] = on;
        out->level[lights::IND_RL] = on;
    }
    if(ch & CUE_CH_IND_R)
    {
        out->level[lights::IND_FR] = on;
        out->level[lights::IND_RR] = on;
    }
    if(ch & CUE_CH_REV)
    {
        out->level[lights::REV_L] = on;
        out->level[lights::REV_R] = on;
    }
}

/*
 * Sound. A seam, not an implementation.
 *
 * There is no buzzer on this car and no pin set aside for one. This records
 * what the running cue WOULD be sounding so CUE can report it, and drives
 * nothing. When a buzzer goes on, this function grows a pwmTone() call and no
 * script changes - which is the entire reason the tone is in cue::Step now rather
 * than being added to it later.
 */
static Void soundWrite(UInt8 tone)
{
    toneNow = tone;
}

/*
 * Plays the running one-shot cue over `out`, advancing it.
 *
 * The cue OWNS every channel any of its steps mentions, for its whole duration,
 * including the steps where that channel is dark. Without that a flash would be
 * invisible whenever the headlights were already on - the cue's on-steps would
 * agree with the continuous state and its off-steps would be overwritten by it.
 */
static Void overlay(UInt64 now, lights::Set* out)
{
    if(kindNow == KIND_NONE)
    {
        return;
    }

    const Script* sc = &SCRIPT[kindNow];

    /*
     * Advance. A while, not an if, and the deadline ACCUMULATES rather than
     * being restarted from `now`.
     *
     * Both halves of that matter and they are the same point. Restarting from
     * `now` would add a whole pass of the main loop to every step, so a
     * two-step cue repeated three times would run six loop-periods long - and
     * it would make the while unreachable, because the new deadline would
     * always be in the future. Accumulating keeps the cue the length the script
     * says, and lets the loop catch up honestly if something upstream blocked:
     * serial::printf can sit for half a second when the host stops draining the
     * port, and a cue should have PLAYED during that, not be waiting to.
     */
    while(kindNow != KIND_NONE && now >= stepAtUs)
    {
        ++stepIx;
        if(stepIx >= sc->steps)
        {
            stepIx = 0;
            ++loopIx;
            if(loopIx >= sc->repeats)
            {
                silence();
                return;   /* channels handed back untouched this tick */
            }
        }
        stepAtUs += static_cast<UInt64>(sc->step[stepIx].ms) * 1000u;
    }

    /* Everything this script ever touches, dark steps included. */
    UInt8 owned = 0u;
    for(UInt8 i = 0; i < sc->steps; ++i)
    {
        owned = static_cast<UInt8>(owned | sc->step[i].lamps);
    }

    const UInt8 lit = sc->step[stepIx].lamps;

    channelLamps(static_cast<UInt8>(owned & ~lit), false, out);
    channelLamps(lit, true, out);

    soundWrite(sc->step[stepIx].tone);
}

/* Call often. Cheap when there is nothing to do. */
static Void tick(const Input* in)
{
    if(!up || in == NULL)
    {
        return;
    }

    const UInt64 now = timing::nowUs();

    /* ---- which way, with hysteresis and a minimum flash ------------------
     *
     * Three things here look right written down and are wrong on a car:
     *   1. TWO thresholds. Steering parked on a single one makes the lamp
     *      stutter at the servo's own jitter.
     *   2. ONE COMPLETE FLASH minimum. The blink clock free-runs, so a turn
     *      starting mid-cycle would otherwise show a sliver of an on-phase and
     *      vanish.
     *   3. ...EXCEPT a change of side, which is immediate. Indicating left
     *      while the wheels go right is the one thing an indicator must never
     *      do.
     */
    const Int32 mag = (in->steerMilli < 0) ? -in->steerMilli : in->steerMilli;

    const Turn want = (in->steerMilli <= -CUE_TURN_ON_MILLI) ? TURN_LEFT
                       : (in->steerMilli >=  CUE_TURN_ON_MILLI) ? TURN_RIGHT
                                                                : TURN_OFF;

    if(want != TURN_OFF && want != turnNow)
    {
        /* Rule 3. Restarting the blink here means the new side begins with a
         * LIT phase rather than inheriting whatever half of the cycle was in
         * progress. */
        turnNow   = want;
        holdUs    = now + static_cast<UInt64>(CUE_BLINK_PERIOD_MS) * 1000u;
        blinkOn   = true;
        blinkAtUs = now + static_cast<UInt64>(CUE_BLINK_ON_MS) * 1000u;
    }
    else if(turnNow != TURN_OFF
            && mag < CUE_TURN_OFF_MILLI
            && now >= holdUs)
    {
        /* Rules 1 and 2 together: below the LOWER threshold, and only once the
         * minimum flash has been served. */
        turnNow = TURN_OFF;
        blinkOn = false;
    }

    /* Half-periods of different lengths, so this cannot be the usual
     * toggle-on-a-fixed-interval. */
    if(turnNow != TURN_OFF && now >= blinkAtUs)
    {
        blinkOn   = !blinkOn;
        blinkAtUs = now
                     + static_cast<UInt64>(blinkOn ? CUE_BLINK_ON_MS : CUE_BLINK_OFF_MS) * 1000u;
    }

    /*
     * Solved and overlaid EVERY tick, even with the lamps switched off.
     *
     * The old lightsTick() returned early when the master switch was off, which
     * froze the turn state machine with it - so LIGHTS went on reporting
     * whichever way the car had been turning when somebody killed the lamps.
     * Only the WRITE is gated now, in lights.h. What the car means is true
     * whether or not anything is lit to say it.
     */
    lights::Set s;
    solve(in, turnNow, blinkOn, &s);
    overlay(now, &s);
    lights::write(&s);
}

/* What the running cue is sounding, for anything that reports it. Always
 * cue::TONE_NONE until a buzzer exists - see cue::soundWrite(). */
static UInt8 tone(Void)
{
    return toneNow;
}


} // namespace cue

} // namespace bibo