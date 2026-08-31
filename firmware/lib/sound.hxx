/* ---------------------------------------------------------------------------
 * sound - the car's voice, asked for by name.
 *
 *     sound::play("hit1");
 *
 * That is the whole point of this file. A caller says what it wants to be
 * heard; which track that is on the card, whether the card is even mounted, and
 * what the module needs told to make it happen are settled here.
 *
 * ---------------------------------------------------------------------------
 * WHY IT IS IN THE LIBRARY AND NOT IN main.cxx.
 *
 * The DFPlayer's Bus, the volume, the equaliser and the file count all lived in
 * app/main.cxx as console state. That works exactly as long as a PERSON is the
 * only thing making noise. The moment the CAR wants to - and cue.hxx already
 * carries a `tone` field waiting for it - the cue system would have to reach up
 * into the application to find the speaker, which is backwards: an app is glue
 * over a library, not a thing the library reads from.
 *
 * So the state is here, beside lights and chassis, and main.cxx is console glue
 * over it. Same arrangement drive:: and lights:: already have, and the reason
 * is the same: the thing that OWNS the hardware is not the thing that decides
 * when to use it.
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS IS NOT.
 *
 * It does not decide when the car should speak. That is cue.hxx's job, and
 * mixing them would give the car two places that know what it means - which is
 * the mistake lights.hxx's banner describes from the last time it happened.
 *
 * play() by NAME is the entry point worth having; play() by number is kept
 * because a number is what you have before a clip has earned a name, and making
 * that awkward would only push people back to remembering track numbers.
 * ------------------------------------------------------------------------- */
#pragma once

#include "hal.hxx"
#include "pins.hxx"
#include "sfx.hxx"
#include "drivers/dfplayer.hxx"

namespace bibo
{

  namespace sound
  {

    /* ---- state, one copy - the same deal chassis.hxx and lights.hxx make -- */
    static dfplayer::Bus bus;

    /* Remembered rather than asked for. The module has no "what volume are
     * you" query worth a round trip, and a value the hub cannot read back is a
     * slider that jumps to a default every time the view is opened. */
    static UInt8  level   = 8;
    static UInt8  tone    = DFP_EQ_NORMAL;
    static UInt16 last    = 1;

    /* How many files the card holds, 0 for "not asked, or it did not answer".
     * Asked at mount, which is when it can change - a card is not swapped while
     * the board is running - rather than on every status line, because the
     * query WAITS and the hub polls twice a second. */
    static UInt16 files   = 0;

    /* Whether the card has been mounted since power-on. It takes 1.5-3 s and a
     * play sent before that is LOST - no error, no sound - so boot does not pay
     * for it and the first thing that needs the card asks. */
    static Bool   mounted = false;

    static Bool   up      = false;

    /* ---- bring-up ---------------------------------------------------------
     *
     * Opens the UART and the BUSY pin from the installed map. CHEAP - a baud
     * rate and two pin functions - so a program can call it at boot without
     * paying for the card.
     *
     * pins::begin() must have run: the map starts empty, and a sound opened
     * before it would bind nothing at all. */
    inline Void open(Void)
    {
        const pins::Map& m = pins::active();

        dfplayer::open(&bus, uart0, m.soundTx, m.soundRx, m.soundBusy);
        up = true;
    }

    /* Resets the module and waits for the card, then puts back the settings a
     * reset clears.
     *
     * THE VOLUME AND TONE ARE RE-APPLIED, and that is not tidiness: a reset
     * returns the module to its defaults, so without this the console would go
     * on reporting a volume the module is no longer at. A status line that
     * disagrees with the hardware is worse than no status line. */
    inline Bool mount(Void)
    {
        if(!up)
        {
            return false;
        }

        dfplayer::reset(&bus);
        dfplayer::useCard(&bus);
        dfplayer::volume(&bus, level);
        dfplayer::eq(&bus, tone);

        mounted = true;

        UInt16 n = 0;
        files = dfplayer::fileCount(&bus, &n) ? n : 0;

        return true;
    }

    /* ---- what came back ---------------------------------------------------
     *
     * ready() is the card, hasVoice() is the BUSY wire, speaking() is a track
     * actually sounding. Three questions rather than one, because they fail
     * independently and a caller that got a single Bool could not tell a
     * silent module from an unwired pin. */
    inline Bool ready(Void)
    {
        return up && mounted;
    }

    inline Bool hasVoice(Void)
    {
        return dfplayer::hasBusy(&bus);
    }

    inline Bool speaking(Void)
    {
        return dfplayer::playing(&bus);
    }

    inline UInt16 count(Void)
    {
        return files;
    }

    inline UInt8 volume(Void)
    {
        return level;
    }

    inline UInt8 eq(Void)
    {
        return tone;
    }

    inline UInt16 track(Void)
    {
        return last;
    }

    /* ---- settings ---------------------------------------------------------
     *
     * Clamped by the driver, remembered here so a reset can put them back. */
    inline Void setVolume(UInt8 v)
    {
        level = (v > DFP_VOLUME_MAX) ? DFP_VOLUME_MAX : v;
        dfplayer::volume(&bus, level);
    }

    inline Void setEq(UInt8 e)
    {
        tone = (e > DFP_EQ_MAX) ? DFP_EQ_MAX : e;
        dfplayer::eq(&bus, tone);
    }

    /* ---- why this file exists --------------------------------------------
     *
     *     sound::play("hit1")
     *
     * WHY IT RETURNS A REASON rather than a Bool. Four different things stop a
     * sound happening and they need four different fixes: nobody opened the
     * speaker, the card was never mounted, the name is not in sfx::CLIPS, or
     * the name is there and points past the end of the card. A single false
     * would send somebody to a bench to work out which - and every one of them
     * SOUNDS the same, which is to say like nothing at all. */
    enum Result
    {
        RESULT_OK = 0,
        RESULT_CLOSED,     /* open() has not run                        */
        RESULT_NO_CARD,    /* mount() has not run, or the card refused  */
        RESULT_NO_CLIP,    /* no clip by that name in sfx::CLIPS        */
        RESULT_PAST_END,   /* the clip names a track the card lacks     */
        RESULT_RESERVED    /* track 0 - 0000.mp3 is not a playable file */
    };

    inline Result playTrack(UInt16 t)
    {
        if(!up)
        {
            return RESULT_CLOSED;
        }
        if(!mounted)
        {
            return RESULT_NO_CARD;
        }

        /* ZERO IS RESERVED AND IS NOT A FILE.
         *
         * The DFPlayer numbers files from 1, so there is no mp3/0000.mp3 to
         * play. It is also the value sfx::NONE uses for "no such clip", which
         * is what makes this worth a guard rather than a comment: a lookup that
         * misses returns 0, and a caller that forgets to check it would hand
         * that 0 straight to here. Refusing names the mistake instead of
         * sending a play for a file that cannot exist.
         *
         * The console's numeric path rejected 0 already. The LIBRARY did not,
         * and the library is what a cue will call. */
        if(t == 0u)
        {
            return RESULT_RESERVED;
        }

        /* Only checked when the count is known. 0 means nobody has asked, and
         * refusing on an unknown would make the speaker useless on a module
         * that will not answer a query but plays perfectly well. */
        if(files > 0 && t > files)
        {
            return RESULT_PAST_END;
        }

        last = t;
        dfplayer::playMp3(&bus, t);
        return RESULT_OK;
    }

    /* By NAME. The lookup is sfx's - case-insensitive and whole-string - so a
     * caller never sees a track number unless it wants one. */
    inline Result play(CharSeq clip)
    {
        const UInt16 t = sfx::track(clip);
        if(t == sfx::NONE)
        {
            return RESULT_NO_CLIP;
        }
        return playTrack(t);
    }

    /* One line each, so a caller can report a refusal without a switch of its
     * own and every place that refuses says the same words. */
    inline CharSeq why(Result r)
    {
        switch(r)
        {
        case RESULT_OK:       return "ok";
        case RESULT_CLOSED:   return "the speaker was never opened";
        case RESULT_NO_CARD:  return "the card is not mounted";
        case RESULT_NO_CLIP:  return "no clip by that name";
        case RESULT_PAST_END: return "that clip is past the end of the card";
        case RESULT_RESERVED: return "track 0 is reserved - files start at 1";
        default:             return "?";
        }
    }

    inline Void stop(Void)
    {
        dfplayer::stop(&bus);
    }

    inline Void pause(Void)
    {
        dfplayer::pause(&bus);
    }

    inline Void resume(Void)
    {
        dfplayer::play(&bus);
    }

  } /* namespace sound */

} /* namespace bibo */
