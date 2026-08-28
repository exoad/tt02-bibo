// Identifier completion for the code editor.
//
// A fixed, hand-written table rather than a parse of the source. That sounds
// like a shortcut and is actually the honest design for this: what a sketch
// wants to complete is the WRAPPER API - pico2w.h and shared.h - and that is a
// known, small, slow-moving set. A real index would spend a translation unit's
// worth of work to rediscover forty names that are written down two files away,
// and would still not carry the one-line doc that makes a suggestion useful.
//
// The cost is that this table and firmware/src/pico2w.h have to be kept in step
// by hand. tests/test_editor.cxx checks the shape of every entry, and the header
// says the same thing at the top, which is the most that can be done without
// building a C parser to serve a popup.
//
// ImGui-free, like editor.cxx and map_geometry.cxx: matching and ranking are
// pure functions and are tested as such.
#pragma once

#include "shared.hxx"

namespace cmpl {

enum class Kind
{
    KIND_FUNCTION = 0,
    KIND_TYPE,
    KIND_MACRO,       // and enum constants: both are SCREAMING_SNAKE to a reader
    KIND_KEYWORD,
};

struct Item
{
    const Char* name      = nullptr;
    const Char* detail    = nullptr;   // signature, or the underlying type
    const Char* doc       = nullptr;   // one line, may be empty
    Kind        kind      = Kind::KIND_FUNCTION;
};

// Everything the table holds, for tests and for a "show all" listing.
[[nodiscard]] const Vec<Item>& all();

// Appends up to `max` matches for `prefix` to `out`, best first.
//
// Matching is case-insensitive on a PREFIX only - not a fuzzy subsequence.
// Fuzzy matching is worth it when the candidate set is thousands of names and
// you are guessing; here it is forty names you already half-know, and it would
// mostly serve to put `servoWriteUs` under `sw`.
//
// Ranking, in order: exact prefix with matching case, then case-insensitive
// prefix, then shorter names before longer, then alphabetical. The last two
// matter more than they look - `gpioOpen` must beat `gpioToggle` for "gpio" or
// the list reads as unsorted.
//
// Returns the number appended. `out` is NOT cleared, so a caller can gather
// from several prefixes.
Size suggest(const Str& prefix, Vec<const Item*>& out, Size max);

// The identifier ending at the end of `line` - what the user is part-way
// through typing. Empty when the line ends in anything else, which is the
// signal to close the popup rather than show all forty entries.
[[nodiscard]] Str wordAtEnd(const Str& line);

} // namespace cmpl
