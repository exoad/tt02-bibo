/**
 * @file sound.hxx
 * @brief The car's voice, asked for by name: the third link in the sound
 * chain, the one a program actually calls.
 *
 * dfplayer_proto.hxx and dfplayer.hxx (below this file) encode and drive the
 * DFPlayer Mini's wire format; sfx.hxx names which numbered file on the card
 * is which clip. Land here to make the car say something.
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
 */
#pragma once

#include "hal.hxx"
#include "pins.hxx"
#include "sfx.hxx"
#include "drivers/dfplayer.hxx"

namespace bibo::sound
{

    /* ---- state, one copy - the same deal chassis.hxx and lights.hxx make -- */
    inline dfplayer::Bus bus;

    /* Remembered, not asked for: no query on the module is worth a round trip. */
    inline UInt8  level   = 8;
    inline UInt8  tone    = DFP_EQ_NORMAL;
    inline UInt16 last    = 1;

    /*
     * How many files the card holds, 0 for "not asked, or it did not answer".
     * Asked at mount rather than per status line: the query WAITS and the hub
     * polls twice a second.
     */
    inline UInt16 files   = 0;

    /*
     * Whether the card has been mounted since power-on. It takes 1.5-3 s and a
     * play sent before that is LOST, so boot does not pay for it.
     */
    inline Bool   mounted = false;

    inline Bool   up      = false;

    /* ---- bring-up ----------------------------------------------------- */
    /**
     * @brief Opens the UART and the BUSY pin from the installed map.
     *
     * CHEAP - a baud rate and two pin functions - so a program can call it
     * at boot without paying for the card.
     *
     * @note pins::begin() must have run: the map starts empty, and a sound
     * opened before it would bind nothing at all.
     * @note On this project's pin map, the UART lives on GP14/GP15 - the
     * pads that carried the tail lamps before sound needed them.
     */
    inline Void open(Void)
    {
        const pins::Map& m = pins::active();

        dfplayer::open(&bus, uart0, m.soundTx, m.soundRx, m.soundBusy);
        up = true;
    }

    /**
     * @brief Resets the module, waits for the card, and puts back the
     * settings a reset clears.
     *
     * THE VOLUME AND TONE ARE RE-APPLIED, and that is not tidiness: a
     * reset returns the module to its defaults, so without this the
     * console would go on reporting a volume the module is no longer at.
     * A status line that disagrees with the hardware is worse than no
     * status line.
     *
     * @return true once the reset/remount sequence has run; false when
     *         open() has not been called yet
     */
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

    /*
     * ---- what came back -------------------------------------------------
     * ready() is the card, hasVoice() is the BUSY wire, speaking() is a track
     * sounding. Three questions, because they fail independently.
     */
    /**
     * @brief Whether the speaker is open and the card mounted.
     *
     * @return true once open() and mount() have both succeeded
     */
    inline Bool ready(Void)
    {
        return up && mounted;
    }

    /**
     * @brief Whether this bus has a BUSY wire to report speaking().
     *
     * @return true when a BUSY pin was given to open()
     */
    inline Bool hasVoice(Void)
    {
        return dfplayer::hasBusy(&bus);
    }

    /**
     * @brief Whether a track is audibly playing right now.
     *
     * @return true while BUSY reports the module is playing
     */
    inline Bool speaking(Void)
    {
        return dfplayer::playing(&bus);
    }

    /**
     * @brief How many files the card held as of the last mount().
     *
     * @return the file count, or 0 when nobody has asked or the module
     *         did not answer
     */
    inline UInt16 count(Void)
    {
        return files;
    }

    /**
     * @brief The volume last set.
     *
     * @return the current volume, 0-30
     */
    inline UInt8 volume(Void)
    {
        return level;
    }

    /**
     * @brief The equaliser preset last set.
     *
     * @return the current preset, 0-5
     */
    inline UInt8 eq(Void)
    {
        return tone;
    }

    /**
     * @brief The track number last sent to play.
     *
     * @return the last track played, or 1 if nothing has played yet
     */
    inline UInt16 track(Void)
    {
        return last;
    }

    /* ---- settings - remembered here so a reset can put them back --------- */
    /**
     * @brief Sets the volume and remembers it for the next mount()/reset.
     *
     * @param v the new volume, 0-30; values above the maximum are clamped
     */
    inline Void setVolume(const UInt8 v)
    {
        level = v > DFP_VOLUME_MAX ? DFP_VOLUME_MAX : v;
        dfplayer::volume(&bus, level);
    }

    /**
     * @brief Sets the equaliser preset and remembers it for the next
     * mount()/reset.
     *
     * @param e the new preset, 0-5; values above the maximum are clamped
     */
    inline Void setEq(const UInt8 e)
    {
        tone = e > DFP_EQ_MAX ? DFP_EQ_MAX : e;
        dfplayer::eq(&bus, tone);
    }

    /* ---- why this file exists: sound::play("hit1") ------------------------ */
    /**
     * @brief Why a sound did or did not play.
     *
     * WHY IT RETURNS A REASON rather than a Bool. Four different things
     * stop a sound happening and they need four different fixes: nobody
     * opened the speaker, the card was never mounted, the name is not in
     * sfx::CLIPS, or the name is there and points past the end of the
     * card. A single false would send somebody to a bench to work out
     * which - and every one of them SOUNDS the same, which is to say like
     * nothing at all.
     */
    enum Result
    {
        RESULT_OK = 0,
        RESULT_CLOSED,     /* open() has not run                        */
        RESULT_NO_CARD,    /* mount() has not run, or the card refused  */
        RESULT_NO_CLIP,    /* no clip by that name in sfx::CLIPS        */
        RESULT_PAST_END,   /* the clip names a track the card lacks     */
        RESULT_RESERVED    /* track 0 - 0000.mp3 is not a playable file */
    };

    /**
     * @brief Plays a track by number, refusing anything that cannot
     * exist.
     *
     * @param t the file number to play, one-based, matching the filename;
     *        0 is refused as RESULT_RESERVED
     * @return RESULT_OK on success, or the reason playback was refused
     */
    inline Result playTrack(const UInt16 t)
    {
        if(!up)
        {
            return RESULT_CLOSED;
        }
        if(!mounted)
        {
            return RESULT_NO_CARD;
        }

        /*
         * ZERO IS RESERVED AND IS NOT A FILE. The DFPlayer numbers files from 1,
         * and 0 is also what sfx::NONE returns for "no such clip" - so a lookup
         * that missed would arrive here as a play for a file that cannot exist.
         */
        if(t == 0u)
        {
            return RESULT_RESERVED;
        }

        /*
         * Only checked when the count is known; 0 means nobody has asked.
         */
        if(files > 0 && t > files)
        {
            return RESULT_PAST_END;
        }

        last = t;
        dfplayer::playMp3(&bus, t);
        return RESULT_OK;
    }

    /**
     * @brief Plays a clip by name.
     *
     * The lookup is sfx's - case-insensitive and whole-string - so a
     * caller never sees a track number unless it wants one.
     *
     * @param clip the clip name to look up in sfx::CLIPS
     * @return RESULT_OK on success, RESULT_NO_CLIP when the name is not
     *         in the table, or another Result as playTrack() would
     *         return
     */
    inline Result play(const CharSeq clip)
    {
        const UInt16 t = sfx::track(clip);
        if(t == sfx::NONE)
        {
            return RESULT_NO_CLIP;
        }
        return playTrack(t);
    }

    /**
     * @brief One human-readable line per Result.
     *
     * One line each, so a caller can report a refusal without a switch of
     * its own and every place that refuses says the same words.
     *
     * @param r the result to explain
     * @return a short, constant string describing r
     */
    inline CharSeq why(const Result r)
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

    /** @brief Stops playback. */
    inline Void stop(Void)
    {
        dfplayer::stop(&bus);
    }

    /** @brief Pauses playback. */
    inline Void pause(Void)
    {
        dfplayer::pause(&bus);
    }

    /** @brief Resumes playback of a paused track. */
    inline Void resume(Void)
    {
        dfplayer::play(&bus);
    }

}
