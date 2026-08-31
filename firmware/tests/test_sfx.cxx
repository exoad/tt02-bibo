/* The clip table in lib/sfx.hxx.
 *
 *   firmware\tests\build_sfx_test.bat run
 *
 * A table of names and numbers cannot be wrong in an interesting way - which is
 * exactly why it is worth a test. The failures it CAN have are all quiet ones:
 * two clips sharing a name, so one is unreachable and nothing says which; a
 * track of 0, which collides with the "no such clip" sentinel; a lookup that
 * matches a prefix, so `horn` and `hornLong` return the same sound.
 *
 * None of those produce an error at the call site. They produce the wrong noise,
 * or silence, which on a bench is indistinguishable from every other sound
 * fault - a wire, a card, a supply. That is the whole reason this is checked
 * here instead of by listening.
 *
 * Compiled for the HOST: sfx.hxx needs types.hxx and text.hxx and nothing else.
 *
 * Exits 0 on PASS, 1 on FAIL.
 */

#include "../lib/sfx.hxx"

#include <stdio.h>
#include <string.h>

using namespace bibo;

static Int32 failures = 0;
static Int32 checks   = 0;

static Void check(Bool ok, CharSeq what)
{
    ++checks;
    if(ok)
    {
        printf("  ok    %s\n", what);
    }
    else
    {
        printf("  FAIL  %s\n", what);
        ++failures;
    }
}

int main(Void)
{
    printf("\nsfx clip table\n\n");

    check(sfx::COUNT > 0, "the table is not empty");

    /* ---- every clip is usable ------------------------------------------- */
    Int32 badTrack = 0;
    for(Size i = 0; i < sfx::COUNT; ++i)
    {
        if(sfx::CLIPS[i].track == sfx::NONE)
        {
            ++badTrack;
        }
        if(sfx::CLIPS[i].name == nullptr || sfx::CLIPS[i].name[0] == '\0')
        {
            ++badTrack;
        }
        if(sfx::CLIPS[i].means == nullptr || sfx::CLIPS[i].means[0] == '\0')
        {
            ++badTrack;
        }
    }
    check(badTrack == 0, "every clip has a name, a meaning, and a track >= 1");

    /* ---- no two clips share a name --------------------------------------
     *
     * The duplicate is REACHABLE - track() returns the first match - so the
     * second one is simply dead, silently, and the only symptom is the wrong
     * sound. */
    Int32 dupName = 0;
    for(Size a = 0; a < sfx::COUNT; ++a)
    {
        for(Size b = a + 1; b < sfx::COUNT; ++b)
        {
            if(strcmp(sfx::CLIPS[a].name, sfx::CLIPS[b].name) == 0)
            {
                printf("        %s is defined twice\n", sfx::CLIPS[a].name);
                ++dupName;
            }
        }
    }
    check(dupName == 0, "no name is defined twice");

    /* Two names for ONE track is legal and deliberate - an alias. Reported
     * rather than failed, because it is a thing somebody might mean. */
    Int32 shared = 0;
    for(Size a = 0; a < sfx::COUNT; ++a)
    {
        for(Size b = a + 1; b < sfx::COUNT; ++b)
        {
            if(sfx::CLIPS[a].track == sfx::CLIPS[b].track)
            {
                printf("        note: %s and %s are both track %u\n",
                       sfx::CLIPS[a].name, sfx::CLIPS[b].name,
                       static_cast<UInt32>(sfx::CLIPS[a].track));
                ++shared;
            }
        }
    }
    printf("        %d alias(es)\n", shared);

    /* ---- lookup round-trips --------------------------------------------- */
    Int32 lost = 0;
    for(Size i = 0; i < sfx::COUNT; ++i)
    {
        if(sfx::track(sfx::CLIPS[i].name) != sfx::CLIPS[i].track)
        {
            ++lost;
        }
    }
    check(lost == 0, "every name looks up to its own track");

    check(sfx::track("definitelyNotAClip") == sfx::NONE,
          "an unknown name is NONE, not a track");
    check(sfx::track("") == sfx::NONE, "the empty name is NONE");
    check(sfx::track(nullptr) == sfx::NONE, "a null name is NONE, not a crash");

    /* PREFIXES MUST NOT MATCH. text::eq is whole-string, and this is the check
     * that says so - a prefix match would make `clip1` and `clip12` the same
     * sound, which is the kind of thing found by ear months later. */
    if(sfx::COUNT > 0)
    {
        Utf8 longer[64];
        snprintf(longer, sizeof(longer), "%sXX", sfx::CLIPS[0].name);
        check(sfx::track(longer) == sfx::NONE,
              "a longer name that starts with a real one does not match");
    }

    /* ---- the reverse direction ------------------------------------------ */
    check(sfx::nameOf(sfx::CLIPS[0].track) != nullptr,
          "a known track has a name");
    check(sfx::nameOf(60000u) == nullptr,
          "an unknown track has no name, and says so with nullptr");

    /* ---- zero is the sentinel and must never BE a clip -------------------
     *
     * sfx::NONE is 0, and the DFPlayer numbers files from 1, so a row claiming
     * track 0 would be both unplayable and indistinguishable from "not found".
     * The table check above already rejects it; this states why. */
    Int32 zero = 0;
    for(Size i = 0; i < sfx::COUNT; ++i)
    {
        if(sfx::CLIPS[i].track == 0u)
        {
            ++zero;
        }
    }
    check(zero == 0, "no clip claims track 0, which is the NONE sentinel");
    check(sfx::nameOf(0u) == nullptr, "track 0 has no name");

    /* ---- highest() is what the card is checked against ------------------- */
    UInt16 top = 0;
    for(Size i = 0; i < sfx::COUNT; ++i)
    {
        if(sfx::CLIPS[i].track > top)
        {
            top = sfx::CLIPS[i].track;
        }
    }
    check(sfx::highest() == top, "highest() is the largest track in the table");

    printf("\n%d checks, %d failed\n\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
