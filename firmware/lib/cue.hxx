/*
 * ---------------------------------------------------------------------------
 * cue - what the car is SAYING: which way it is about to turn, that it is not
 * being driven, that it has stopped itself, that it has seen you. Lamps today,
 * a buzzer later, and nothing above here changes when the buzzer arrives.
 *
 * The split is by JOB, not by hardware:
 *
 *   cue.h      decides what the car is expressing. Knows nothing about pins.
 *   lights.h   drives the lamps it is handed. Knows nothing about why.
 *
 * A cue reaches a person through CHANNELS - a group of lamps, or eventually a
 * tone - never through individual lamps, since "set lamp 0 and lamp 1" stops
 * being true the moment a lamp moves.
 *
 * CONTINUOUS cues are what the car is doing right now, recomputed every tick
 * from the drivetrain. ONE-SHOTs are a short script of steps played OVER the
 * continuous state, and a one-shot OWNS every channel any of its steps
 * mentions for its whole duration - which is what makes a headlight flash
 * visible when the headlights are already on. Channels it does not mention are
 * left alone, so the indicators blink through a flash.
 *
 *     lights::open();  cue::open();  cue::tick(&in);  cue::emit(KIND_FLASH);
 *
 * Everything the rules read is passed IN rather than reached for. Feed it the
 * ACTUAL servo and ESC output, not the targets: the slew limiter means the two
 * differ for about a second after every command. The CONTINUOUS rules are
 * mirrored in hub/src/lights.hxx with deliberately identical constants.
 * -------------------------------------------------------------------------
 */
#pragma once

#include "hal.hxx"
#include "lights.hxx"

namespace bibo::cue
{

    /*
     * ---- channels: a bitmask, because an utterance is routinely more than one
     * thing at once. Coarser than the lamp list on purpose - no cue wants the
     * front left indicator alone, and which lamps a side is, is lights.h's.
     */
#define CUE_CH_HEAD  0x01u
#define CUE_CH_TAIL  0x02u
#define CUE_CH_IND_L 0x04u
#define CUE_CH_IND_R 0x08u
#define CUE_CH_REV   0x10u

#define CUE_CH_IND_BOTH (CUE_CH_IND_L | CUE_CH_IND_R)

    /*
     * ---- tones: NOTHING DRIVES THESE YET - no buzzer, no pin set aside for
     * one, so cue::soundWrite() below records the tone and returns. Declared
     * now, and carried by every script step, so sound does not mean revising
     * every script that exists by then.
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

    /*
     * ---- the flash rate, and the standard it has to sit inside ---------------
     *
     * FMVSS 108 incorporates SAE J590 (turn signals) and J945 (hazard flashers)
     * by reference, and those carry the numbers:
     *
     *   60-120 flashes per minute   for a NORMALLY OPEN (variable load) flasher
     *   90-120 flashes per minute   for a NORMALLY CLOSED (fixed load) flasher
     *
     * plus a percent-current-on-time envelope (J945 Figure 1) whose practical
     * form is that ON must not be shorter than OFF.
     *
     * 360 / 240 is 600 ms, 100.0 fpm, 60% on - inside BOTH bands with margin.
     * The previous 400/267 was 667 ms, 89.96 fpm: 0.04 fpm under the
     * normally-closed floor, from calling 1.5 Hz "the legal standard" and then
     * missing 90 fpm exactly by rounding.
     */
#define CUE_BLINK_ON_MS     360u
#define CUE_BLINK_OFF_MS    240u
#define CUE_BLINK_PERIOD_MS (CUE_BLINK_ON_MS + CUE_BLINK_OFF_MS)

    /*
     * THE STANDARD, ENFORCED BY THE BUILD, as periods rather than rates so it
     * is exact integer arithmetic - and the tighter of the two bands, so
     * satisfying this satisfies both:
     *
     *     rate >=  90 fpm   <=>   60000 >=  90 * period   <=>   period <= 666 ms
     *     rate <= 120 fpm   <=>   60000 <= 120 * period   <=>   period >= 500 ms
     */
    static_assert(
        CUE_BLINK_PERIOD_MS >= 500u,
        "flash rate over 120/min - faster than SAE J590/J945 allow"
    );
    static_assert(
        CUE_BLINK_PERIOD_MS <= 666u,
        "flash rate under 90/min - below the normally-closed band"
    );
    static_assert(
        CUE_BLINK_ON_MS >= CUE_BLINK_OFF_MS,
        "on-time shorter than off-time - outside the J945 Figure 1 envelope"
    );

    /* Which way the car is indicating, or wants to. */
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
     * How far past idle the throttle goes before the car counts as DRIVEN and
     * the tails go out - mirrored below neutral for reversing. Settable because
     * it is a judgment found by watching the car, not a measurement. Seeded
     * from cal.h by the application; this file cannot reach for cal.h.
     */
    inline Int32 motionUsNow = 10;

    /* Narrow enough that a typo cannot kill the lamps over the whole range. */
#define CUE_MOTION_US_MIN 0
#define CUE_MOTION_US_MAX 60

    /**
     * @brief Sets the runtime threshold for counting the car as moving.
     *
     * @param us the new threshold, in microseconds past idle (forward) or
     *           neutral (reverse)
     * @return true when accepted; false when outside
     *         [CUE_MOTION_US_MIN, CUE_MOTION_US_MAX], leaving the
     *         threshold unchanged
     */
    inline Bool setMotionUs(const Int32 us)
    {
        if(us < CUE_MOTION_US_MIN || us > CUE_MOTION_US_MAX)
        {
            return false;
        }
        motionUsNow = us;
        return true;
    }

    /**
     * @brief The threshold currently used to decide the car is moving.
     *
     * @return the value last accepted by setMotionUs(), in microseconds
     */
    inline Int32 motionUs(Void)
    {
        return motionUsNow;
    }

    /*
     * ---- what a cue IS -------------------------------------------------------
     *
     * One mechanism, three ways of playing, and that is the whole API:
     *
     *   PLAY_ONCE   run the script through `repeats` times and stop - flash, alert.
     *   PLAY_LOOP   run it round until something cancels it - the indicators.
     *   PLAY_HOLD   one state, held until canceled - headlights, brake, reverse.
     *
     * The continuous behavior used to be a solve() beside the one-shots, which
     * meant two lighting systems. Everything is a cue now.
     */
    enum Play
    {
        PLAY_ONCE = 0,
        PLAY_LOOP,
        PLAY_HOLD
    };

    /*
     * One step: hold these channels, at this brightness, for this long, with
     * this tone. A table rather than code, so adding a cue is adding data.
     * `level` is what makes a running lamp and a brake lamp the SAME mechanism
     * on one bulb - LAMP_DIM against LAMP_FULL, resolved by priority.
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

    /* ---- the scripts: a HOLD is one step whose ms is ignored, so write 0. --- */

    /*
     * FLASH - the headlights, TWICE and not once: a single 90 ms blip is what a
     * loose connection looks like. It is what a driver does to mean "after you".
     */
    static constexpr Step STEPS_FLASH[] =
    {
        {  .ms = 90u, .lamps = CUE_CH_HEAD, .level = LAMP_FULL, .tone = TONE_NONE },
        { .ms = 110u, .lamps = 0u, .level = LAMP_OFF,  .tone = TONE_NONE }
    };

    /*
     * ALERT - both indicator pairs and the tails, three times. What the car says
     * when it has stopped itself: the deadman firing, which by definition
     * happens when the host is not listening. Slower and heavier than FLASH on
     * purpose, since the two must not be confusable at a glance.
     */
    static constexpr Step STEPS_ALERT[] =
    {
        { .ms = 160u, .lamps = CUE_CH_IND_BOTH | CUE_CH_TAIL, .level = LAMP_FULL, .tone = TONE_LOW  },
        { .ms = 160u, .lamps = 0u, .level = LAMP_OFF,  .tone = TONE_NONE }
    };

    /*
     * The indicators. 360 on, 240 off - not 50/50, because a slightly longer on
     * than off is what a real flasher does and what the eye expects.
     *
     * HAZARD IS ITS OWN CUE rather than left and right together: two cues have
     * two step clocks, and two clocks started a millisecond apart drift into an
     * alternating flash. Front and rear on a side share one flash for the same
     * reason; the rear pair has no LED yet but is computed and reported anyway.
     */
    static constexpr Step STEPS_LEFT[] =
    {
        {.ms = CUE_BLINK_ON_MS, .lamps = CUE_CH_IND_L, .level = LAMP_FULL, .tone = TONE_NONE },
        {.ms = CUE_BLINK_OFF_MS, .lamps = 0u, .level = LAMP_OFF,  .tone = TONE_NONE }
    };

    static constexpr Step STEPS_RIGHT[] =
    {
        {.ms = CUE_BLINK_ON_MS, .lamps = CUE_CH_IND_R, .level = LAMP_FULL, .tone = TONE_NONE },
        {.ms = CUE_BLINK_OFF_MS, .lamps = 0u, .level = LAMP_OFF,  .tone = TONE_NONE }
    };

    static constexpr Step STEPS_HAZARD[] =
    {
        {.ms = CUE_BLINK_ON_MS, .lamps = CUE_CH_IND_BOTH, .level = LAMP_FULL, .tone = TONE_NONE },
        {.ms = CUE_BLINK_OFF_MS, .lamps = 0u, .level = LAMP_OFF,  .tone = TONE_NONE }
    };

    /* Held states. */
    static constexpr Step STEPS_HEAD[] = { { .ms = 0u, .lamps = CUE_CH_HEAD, .level = LAMP_FULL, .tone = TONE_NONE } };
    static constexpr Step STEPS_RUNNING[] = { { .ms = 0u, .lamps = CUE_CH_TAIL, .level = LAMP_DIM,  .tone = TONE_NONE } };
    static constexpr Step STEPS_BRAKE[] = { { .ms = 0u, .lamps = CUE_CH_TAIL, .level = LAMP_FULL, .tone = TONE_NONE } };
    static constexpr Step STEPS_REVERSE[] = { { .ms = 0u, .lamps = CUE_CH_REV, .level = LAMP_FULL, .tone = TONE_NONE } };

    /*
     * ORDER IS PRIORITY. A later cue wins a channel an earlier one also wants,
     * and ALERT is last because "something is wrong" outranks everything.
     * RUNNING before BRAKE is the whole reason `level` exists - both want the
     * tails, BRAKE is later, so braking shows FULL over the running lamps' DIM.
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

    /* IN cue::Kind ORDER - a row in the wrong place renames two cues at once. */
    static const Script SCRIPT[KIND_COUNT] =
    {
        { .name = "none",    .means = "nothing",                      .step = nullptr,          .steps = 0u, .repeats = 0u, .play = PLAY_HOLD },
        { .name = "head",    .means = "my headlights are on",         .step = STEPS_HEAD,    .steps = 1u, .repeats = 0u, .play = PLAY_HOLD },
        { .name = "running", .means = "I am lit but not braking",     .step = STEPS_RUNNING, .steps = 1u, .repeats = 0u, .play = PLAY_HOLD },
        { .name = "brake",   .means = "I am not being driven",        .step = STEPS_BRAKE,   .steps = 1u, .repeats = 0u, .play = PLAY_HOLD },
        { .name = "reverse", .means = "I am backing up",              .step = STEPS_REVERSE, .steps = 1u, .repeats = 0u, .play = PLAY_HOLD },
        { .name = "left",    .means = "I am turning left",            .step = STEPS_LEFT,    .steps = 2u, .repeats = 0u, .play = PLAY_LOOP },
        { .name = "right",   .means = "I am turning right",           .step = STEPS_RIGHT,   .steps = 2u, .repeats = 0u, .play = PLAY_LOOP },
        { .name = "hazard",  .means = "I am a hazard",                .step = STEPS_HAZARD,  .steps = 2u, .repeats = 0u, .play = PLAY_LOOP },
        { .name = "flash",   .means = "I have seen you - after you",  .step = STEPS_FLASH,   .steps = 2u, .repeats = 2u, .play = PLAY_ONCE },
        { .name = "alert",   .means = "I have stopped myself",        .step = STEPS_ALERT,   .steps = 2u, .repeats = 3u, .play = PLAY_ONCE }
    };

    /*
     * State, one copy - the same deal chassis.hxx makes. PER KIND, because the
     * car says more than one thing at a time: headlights on, braking and
     * indicating left is three cues at once.
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

    /**
     * @brief Opens the lamps and resets every cue's state to idle.
     */
    inline Void open(Void)
    {
        lights::open();

        for(Int32 k = 0; k < KIND_COUNT; ++k)
        {
            active[k] = false;
            latched[k] = false;
            stepIx[k] = 0;
            loopIx[k] = 0;
            stepAtUs[k] = 0;
        }
        turnWant = TURN_OFF;
        turnHoldUs = 0;
        toneNow = TONE_NONE;
        up = true;
    }

    /**
     * @brief Whether a value names a real cue, not KIND_NONE and in range.
     *
     * @param k the value to check, typically a cue::Kind
     * @return true when it names an actual cue
     */
    inline Bool valid(const Int32 k)
    {
        return k > KIND_NONE && k < KIND_COUNT;
    }

    /**
     * @brief Whether a cue is currently playing.
     *
     * @param k the cue to check
     * @return true when it is active this tick
     */
    inline Bool on(const Kind k)
    {
        return valid(k) && active[k];
    }

    /**
     * @brief Whether a cue is latched by a person rather than by the car.
     *
     * @param k the cue to check
     * @return true when a person raised it and has not lowered it
     */
    inline Bool held(const Kind k)
    {
        return valid(k) && latched[k];
    }

    /**
     * @brief The console word for a cue.
     *
     * @param k the cue to name
     * @return the name a person types to raise it, or "?" out of range
     */
    inline CharSeq name(const Kind k)
    {
        return k >= 0 && k < KIND_COUNT ? SCRIPT[k].name : "?";
    }

    /**
     * @brief What a cue is saying, in words.
     *
     * @param k the cue to describe
     * @return the meaning shown beside its name, or "?" out of range
     */
    inline CharSeq means(const Kind k)
    {
        return k >= 0 && k < KIND_COUNT ? SCRIPT[k].means : "?";
    }

    /**
     * @brief The console word for a play mode.
     *
     * @param p a cue::Play value
     * @return "once", "loop", "hold", or "?" for anything else
     */
    inline CharSeq playWord(const UInt8 p)
    {
        switch(p)
        {
            case PLAY_ONCE: return "once";
            case PLAY_LOOP: return "loop";
            case PLAY_HOLD: return "hold";
            default:        return "?";
        }
    }

    /**
     * @brief Looks up a cue by its console name.
     *
     * Case-insensitive, because every other command word on this link is
     * upper case by the time it arrives.
     *
     * @param want the name to match, or nullptr
     * @return the matching cue, or cue::KIND_NONE when nothing matches
     */
    inline Kind find(const CharSeq want)
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

    /**
     * @brief Begins playing a cue's script from its first step.
     *
     * @param k the cue to start
     * @param now the current time, used to schedule the first step's end
     */
    inline Void start(const Kind k, const UInt64 now)
    {
        active[k] = true;
        stepIx[k] = 0;
        loopIx[k] = 0;
        stepAtUs[k] = now + static_cast<UInt64>(SCRIPT[k].step[0].ms) * 1000u;
    }

    /**
     * @brief Raises a cue on a person's command, and latches it.
     *
     * Raised BY A PERSON, so it latches: the car's own rules will not lower
     * it again. That is what makes "headlights on" a switch rather than a
     * suggestion the next tick overrules.
     *
     * Left and right cancel each other. A car cannot indicate both ways -
     * that is what hazard is - and two blinkers with independent step
     * clocks would drift apart into an alternating flash within a few
     * seconds.
     *
     * @param k the cue to raise
     * @return true when raised; false for an invalid kind, KIND_NONE, or a
     *         cue with no steps
     */
    inline Bool emit(const Kind k)
    {
        if(!up || !valid(k) || SCRIPT[k].step == nullptr)
        {
            return false;
        }

        const UInt64 now = timing::nowUs();

        if(k == KIND_LEFT || k == KIND_RIGHT || k == KIND_HAZARD)
        {
            constexpr Kind others[3] = { KIND_LEFT, KIND_RIGHT, KIND_HAZARD };
            for(const auto other : others)
            {
                if(other != k)
                {
                    active[other] = false;
                    latched[other] = false;
                }
            }
        }

        start(k, now);
        latched[k] = true;
        return true;
    }

    /**
     * @brief Lowers a cue on a person's command.
     *
     * Stops it AND hands it back to the car's own rules.
     *
     * @param k the cue to lower
     * @return true when lowered; false for an invalid kind
     */
    inline Bool cancel(const Kind k)
    {
        if(!up || !valid(k))
        {
            return false;
        }
        active[k] = false;
        latched[k] = false;
        return true;
    }

    /**
     * @brief Stops every cue mid-sentence and hands every channel back.
     */
    inline Void silence(Void)
    {
        for(Int32 k = 1; k < KIND_COUNT; ++k)
        {
            active[k] = false;
            latched[k] = false;
        }
        toneNow = TONE_NONE;
    }

    /**
     * @brief States what the CAR wants a cue to be doing, this tick.
     *
     * A latched cue overrules this. The asymmetry is the point: a person
     * switching the headlights on means it until they say otherwise, and
     * the car noticing it is no longer braking must not put them out.
     *
     * @param k the cue the car has an opinion about
     * @param want true to start the cue if it is not already active, false
     *             to stop it, unless a person has latched it
     * @param now the current time, used to schedule the cue if it starts
     */
    inline Void wants(const Kind k, const Bool want, const UInt64 now)
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

    /**
     * @brief The most important thing the car is saying, for a one-line
     *        status.
     *
     * Returns the LAST active kind, because the enum is in priority order
     * - so this answers with what a person looking at the car would
     * notice first, not with whichever happened to be raised earliest.
     *
     * @return the highest-priority active cue, or cue::KIND_NONE
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

    /**
     * @brief Whether the car is saying anything at all.
     *
     * @return true when some cue is active
     */
    inline Bool busy(Void)
    {
        return speaking() != KIND_NONE;
    }

    /**
     * @brief Which step of its script the loudest cue is on.
     *
     * @return the step index of speaking()'s cue, or 0 when nothing is
     *         active
     */
    inline UInt8 step(Void)
    {
        const Kind k = speaking();
        return k == KIND_NONE ? 0u : stepIx[k];
    }

    /**
     * @brief How many times the loudest cue has repeated its script.
     *
     * @return the loop count of speaking()'s cue, or 0 when nothing is
     *         active
     */
    inline UInt8 loop(Void)
    {
        const Kind k = speaking();
        return k == KIND_NONE ? 0u : loopIx[k];
    }

    /**
     * @brief Which way the car is indicating.
     *
     * Derived from the cues rather than kept beside them, so there is one
     * answer and not two that can disagree.
     *
     * @return TURN_HAZARD, TURN_LEFT, TURN_RIGHT, or TURN_OFF
     */
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

    /**
     * @brief Turns a channel bitmask and a level into lamp levels.
     *
     * Which lamps a channel is, in one place. A cue names the channel;
     * this is the only thing that knows which bulbs that turns out to be.
     *
     * ---- WHY ALL FOUR INDICATORS CANNOT DRIFT --------------------------
     *
     * A side is ONE channel and both its lamps are written from ONE step,
     * in one assignment pair, so front-left and rear-left are the same
     * value by construction rather than by two timers agreeing. There is
     * no clock per lamp to drift, and there is no arrangement of the code
     * in which they differ.
     *
     * The same argument covers hazards: CUE_CH_IND_BOTH is one channel
     * mask read from one step of one cue, so all four are written
     * together and are in phase structurally. That is why hazard is its
     * own cue rather than left and right raised at once - two cues have
     * two step clocks, and two clocks started a millisecond apart come
     * apart into an alternating flash, which is what a film prop does.
     *
     * The rear pair has no pin bound yet. It is computed and REPORTED
     * anyway, which is what let this be measured before the LEDs exist:
     * LIGHTS shows all ten levels, and front and rear matched on every
     * sample of both cues. Wiring them is a change to the pin table in
     * lights.hxx and nothing else.
     *
     * @param ch the CUE_CH_* channels to set
     * @param level the level to give every lamp in those channels
     * @param out the lamp set to write into; lamps outside `ch` are left
     *            untouched
     */
    inline Void channelLamps(const UInt8 ch, const UInt8 level, lights::Set* out)
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

    /**
     * @brief Records the tone the running cue would be sounding.
     *
     * Sound. A seam, not an implementation.
     *
     * There is no buzzer on this car and no pin set aside for one. This
     * records what the running cue WOULD be sounding so CUE can report
     * it, and drives nothing. When a buzzer goes on, this function grows
     * a pwm tone call and no script changes - which is the entire reason
     * the tone is in cue::Step now rather than being added to it later.
     *
     * @param tone the tone the current step calls for
     */
    inline Void soundWrite(const UInt8 tone)
    {
        toneNow = tone;
    }

    /**
     * @brief Advances one cue's script, and stops it if it has finished.
     *
     * A while, not an if, and the deadline ACCUMULATES rather than being
     * restarted from `now`.
     *
     * Both halves of that matter and they are the same point. Restarting
     * from `now` would add a whole pass of the main loop to every step,
     * so a two-step cue repeated three times would run six loop-periods
     * long - and it would make the while unreachable, because the new
     * deadline would always be in the future. Accumulating keeps the cue
     * the length the script says, and lets the loop catch up honestly if
     * something upstream blocked: serial::printf can sit for half a
     * second when the host stops draining the port, and a cue should
     * have PLAYED during that, not be waiting to.
     *
     * @param k the cue to advance, as a raw index into cue::SCRIPT
     * @param now the current time, compared against the cue's step
     *            deadline
     */
    inline Void advance(const Int32 k, const UInt64 now)
    {
        const Script* sc = &SCRIPT[k];

        if(sc->play == PLAY_HOLD)
        {
            return;   /* nothing to advance; it is one state until canceled */
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
                    /* A finished one-shot does not stay latched. */
                    active[k] = false;
                    latched[k] = false;
                    return;
                }
            }
            stepAtUs[k] += static_cast<UInt64>(sc->step[stepIx[k]].ms) * 1000u;
        }
    }

    /**
     * @brief Composites every active cue into one lamp set, in priority
     *        order.
     *
     * A cue OWNS every channel any of its steps mentions, for its whole
     * duration, including the steps where that channel is dark. Without
     * that a flash would be invisible whenever the headlights were
     * already on - the cue's on-steps would agree with whatever was
     * underneath and its off-steps would be overwritten by it.
     *
     * That ownership is also what makes priority mean something. A lower
     * cue writes its channels; a higher one writes over them, lit or
     * dark, and the result is the higher cue's opinion in full rather
     * than a blend of two.
     *
     * @param now the current time, passed on to advance()
     * @param out the lamp set to fill; cleared first
     */
    inline Void compose(const UInt64 now, lights::Set* out)
    {
        lights::clear(out);

        /*
         * `want`, not `tone`: a local named `tone` would shadow the accessor
         * below, and a later edit meaning to CALL it would read the half-built
         * local instead.
         */
        UInt8 want = TONE_NONE;

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

            channelLamps(
                static_cast<UInt8>(static_cast<UInt32>(owned) & ~static_cast<UInt32>(st->lamps)),
                LAMP_OFF,
                out
            );
            channelLamps(st->lamps, st->level, out);

            if(st->tone != TONE_NONE)
            {
                want = st->tone;
            }
        }

        soundWrite(want);
    }

    /**
     * @brief Runs one tick of every continuous rule, and shows the result.
     *
     * Call often. Cheap when there is nothing to do.
     *
     * @param in what the continuous rules read this tick; must be the
     *           ACTUAL servo and ESC output, not the targets - see the
     *           file banner for why
     *
     * @note Composes every tick, even with the master lamp switch off, so
     *       the turn-signal state machine keeps running while the car is
     *       dark. Only the write to the pins is gated, in lights.hxx.
     */
    inline Void tick(const Input* in)
    {
        if(!up || in == nullptr)
        {
            return;
        }

        const UInt64 now = timing::nowUs();

        /*
         * ---- which way, with hysteresis and a minimum flash ---------------
         *   1. TWO thresholds. Steering parked on a single one makes the lamp
         *      stutter at the servo's own jitter.
         *   2. ONE COMPLETE FLASH minimum, or a turn starting and ending inside
         *      one blink period shows a sliver of an on-phase and vanishes.
         *   3. ...EXCEPT a change of side, which is immediate. Indicating left
         *      while the wheels go right is what an indicator must never do.
         */
        const Int32 mag = in->steerMilli < 0 ? -in->steerMilli : in->steerMilli;

        const Turn want = in->steerMilli <= -CUE_TURN_ON_MILLI ? TURN_LEFT
                              : in->steerMilli >=  CUE_TURN_ON_MILLI ? TURN_RIGHT
                                    : TURN_OFF;

        if(want != TURN_OFF && want != turnWant)
        {
            turnWant = want;
            turnHoldUs = now + static_cast<UInt64>(CUE_BLINK_PERIOD_MS) * 1000u;
        }
        else if(turnWant != TURN_OFF
                && mag < CUE_TURN_OFF_MILLI
                && now >= turnHoldUs)
        {
            turnWant = TURN_OFF;
        }

        /*
         * ---- what the car wants to say. Every one is a REQUEST, overruled by
         * anything a person has latched: the car's opinion and the operator's
         * are the same mechanism at different priority.
         */
        wants(KIND_LEFT,  turnWant == TURN_LEFT  && !active[KIND_HAZARD], now);
        wants(KIND_RIGHT, turnWant == TURN_RIGHT && !active[KIND_HAZARD], now);

        /*
         * Is the car being DRIVEN - either way? Forward must clear idle by the
         * threshold, since idle is the pulse at which nothing turns; reverse
         * mirrors it about neutral, symmetric on purpose. Between the two the
         * motor is doing nothing worth calling motion and the tails stay on.
         */
        const Bool fwd = in->throttleUs > in->idleUs + motionUsNow;
        const Bool rev = in->throttleUs < in->neutralUs - motionUsNow;
        const Bool driven = fwd || rev;

        /*
         * The tails, as two cues on one pair of lamps: RUNNING is the dim glow,
         * BRAKE is full and wins by being later in the enum. NO OVERRIDE from
         * the indicators - interrupting the brake is a real convention for cars
         * where the rear indicator and the brake light are ONE BULB, and on
         * this car's separate LEDs it only made the brake light blink in
         * antiphase to the signal beside it.
         */
        wants(KIND_BRAKE, !driven, now);

        /*
         * Reverse lamps. chassis.hxx is forward-only today - it clamps the
         * throttle to [idle, max] - so `rev` is always false and these never
         * light. The rule is here so the lighting is already right when reverse
         * arrives as a brake-then-reverse sequence in the ESC.
         */
        wants(KIND_REVERSE, rev, now);

        /*
         * Nothing the car measures implies darkness, so `headOn` is a person's
         * answer arriving through the Input. RESOLVED BEFORE RUNNING, and that
         * order is load-bearing: the running lamps follow the HEADLIGHTS, not
         * the Input, or CUE HEAD lights the heads and not the tails.
         */
        wants(KIND_HEAD, in->headOn, now);

        /* Dim tails whenever the car is lit at all - BRAKE is the upper half. */
        wants(KIND_RUNNING, active[KIND_HEAD], now);

        /*
         * Composed EVERY tick, even with the lamps switched off. Returning early
         * on the master switch froze the turn state machine with it, so LIGHTS
         * went on reporting whichever way the car had been turning when somebody
         * killed the lamps. Only the WRITE is gated, in lights.hxx.
         */
        lights::Set s;
        compose(now, &s);
        lights::write(&s);
    }

    /**
     * @brief What the running cue is sounding, for anything that reports
     *        it.
     *
     * Always cue::TONE_NONE until a buzzer exists - see
     * cue::soundWrite().
     *
     * @return the tone from the current step of the loudest cue
     */
    inline UInt8 tone(Void)
    {
        return toneNow;
    }

}
