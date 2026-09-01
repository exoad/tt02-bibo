/**
 * @file sfx.hxx
 * @brief The clip catalog: names which numbered file on the card is which
 * sound - the fourth piece of the sound chain, alongside sound.hxx.
 *
 * sound.hxx is the layer a program calls to make noise, and it calls this
 * table to turn a name into a track. Beneath both, dfplayer.hxx drives the
 * DFPlayer Mini over UART and dfplayer_proto.hxx encodes what goes out on
 * the wire. Land here to rename a clip or find out what track a name
 * points at.
 *
 * The DFPlayer plays a number. mp3/0002.mp3 is track 2 and the module has no
 * idea what is in it, which means every caller either remembers or guesses -
 * and `playMp3(&sound, 2)` at a call site is a fact about a filesystem written
 * into a place that is reasoning about a car.
 *
 * This is the table that gives them names. A cue asks for `horn`; what that
 * costs in track numbers is settled here and nowhere else.
 *
 * Same shape and same reason as cue::SCRIPT, which names what the car SAYS -
 * `{ name, means }`, one row per thing, with the meaning in plain English
 * beside it. A caller that had to know both the name and the number would be
 * carrying the mapping this file exists to hold.
 *
 * ---------------------------------------------------------------------------
 * THE CARD IS STILL THE SOURCE OF TRUTH about what EXISTS.
 *
 * This table says what each track MEANS; it cannot say whether the file is
 * there. Those are different questions and conflating them is how a name comes
 * to point at silence. SOUND FILES asks the module how many files it actually
 * holds, and a name whose track is past that count is reported rather than
 * played - see cmdSound.
 *
 * So a row here is a CLAIM about the card, and the claim is checkable.
 *
 * ---------------------------------------------------------------------------
 * THE NAMES BELOW ARE PLACEHOLDERS AND SHOULD BE CHANGED.
 *
 * Only the person who put the mp3s on the card knows what is in them, and
 * inventing "horn" for a file nobody has heard would be worse than a number -
 * a wrong name is believed, where a number at least admits it means nothing.
 *
 * Rename them for what they SOUND like, not for where they are used: a clip
 * called `startup` can be reused as a greeting, and one called `cueBrake` is
 * stuck the day the brake cue wants a different noise.
 */
#pragma once

#include "types.hxx"

namespace bibo::sfx
{

    /**
     * @brief Sentinel meaning "no such clip".
     *
     * Not a track. The DFPlayer numbers files from 1, so 0 is free to
     * mean "no such clip" and every caller can test it without a second
     * flag.
     */
    constexpr UInt16 NONE = 0;

    /**
     * @brief One named clip: what a caller asks for, which file it is,
     * and what it sounds like.
     */
    struct Clip
    {
        CharSeq name;    /* what a caller asks for       */
        UInt16  track;   /* mp3/000N.mp3 on the card     */
        CharSeq means;   /* what it is, in plain English */
    };

    /**
     * @brief The clip table: one row per named sound.
     *
     * @warning The names below are PLACEHOLDERS. Rename each for what it
     * actually sounds like once someone has heard the file on the card.
     */
    static constexpr Clip CLIPS[] =
    {
        { .name = "clip1", .track = 1u, .means = "PLACEHOLDER - rename for what it sounds like" },
        { .name = "clip2", .track = 2u, .means = "PLACEHOLDER - rename for what it sounds like" },
        { .name = "clip3", .track = 3u, .means = "PLACEHOLDER - rename for what it sounds like" }
    };

    /** @brief How many rows CLIPS holds. */
    constexpr Size COUNT = sizeof(CLIPS) / sizeof(CLIPS[0]);

    /*
     * ---- lookup -------------------------------------------------------
     *
     * CASE-INSENSITIVE, and that is required, not politeness: the console
     * upper-cases a whole command line before dispatching, so a clip named
     * `horn` arrives as HORN and a case-sensitive compare misses every time.
     * Folding here rather than upper-casing the table keeps the names
     * readable. Whole-string, not a prefix: `horn` must not match `hornLong`.
     */
    /**
     * @brief Compares two names, folding ASCII case and requiring a full
     * match.
     *
     * @param a the first name; nullptr never matches
     * @param b the second name; nullptr never matches
     * @return true when a and b are the same name, ignoring letter case
     */
    inline Bool sameName(const CharSeq a, const CharSeq b)
    {
        if(a == nullptr || b == nullptr)
        {
            return false;
        }

        Size i = 0;
        for(; a[i] != '\0' && b[i] != '\0'; ++i)
        {
            const Utf8 ca = a[i] >= 'A' && a[i] <= 'Z'
                                ? static_cast<Utf8>(a[i] - 'A' + 'a') : a[i];
            const Utf8 cb = b[i] >= 'A' && b[i] <= 'Z'
                                ? static_cast<Utf8>(b[i] - 'A' + 'a') : b[i];
            if(ca != cb)
            {
                return false;
            }
        }
        return a[i] == '\0' && b[i] == '\0';
    }

    /**
     * @brief Looks up a clip's track number by name.
     *
     * The caller gets one answer and one test rather than an index it
     * then has to remember to bounds-check.
     *
     * @param name the clip name to look up; nullptr returns NONE
     * @return the matching track number, or NONE when there is no such
     *         clip
     */
    inline UInt16 track(const CharSeq name)
    {
        if(name == nullptr)
        {
            return NONE;
        }
        for(const auto i : CLIPS)
        {
            if(sameName(name, i.name))
            {
                return i.track;
            }
        }
        return NONE;
    }

    /**
     * @brief Looks up a clip's name by track number, for REPORTING.
     *
     * The other direction from track(). A status line that says track 2
     * makes a person open this file; one that says `clip2` does not.
     *
     * @param t the track number to look up
     * @return the matching clip name, or nullptr when no clip names this
     *         track - returned rather than "?" so a caller can decide
     *         how to render an unnamed track
     */
    inline CharSeq nameOf(const UInt16 t)
    {
        for(const auto i : CLIPS)
        {
            if(i.track == t)
            {
                return i.name;
            }
        }
        return nullptr;
    }

    /**
     * @brief Looks up what a clip sounds like, in plain English.
     *
     * @param name the clip name to look up
     * @return the matching Clip::means text, or nullptr when no clip has
     *         this name
     */
    inline CharSeq means(const CharSeq name)
    {
        for(const auto i : CLIPS)
        {
            if(sameName(name, i.name))
            {
                return i.means;
            }
        }
        return nullptr;
    }

    /**
     * @brief The highest track number any clip in CLIPS names.
     *
     * What a caller checks the card's file count against: if the card
     * holds fewer files than this, at least one name in the table points
     * at nothing.
     *
     * @return the highest track number in CLIPS, or 0 if CLIPS is empty
     */
    inline UInt16 highest(Void)
    {
        UInt16 top = 0;
        for(const auto i : CLIPS)
        {
            if(i.track > top)
            {
                top = i.track;
            }
        }
        return top;
    }

}
