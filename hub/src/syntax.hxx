// A C/C++ tokenizer for the code editor, and the Gruvbox palette it paints with.
//
// ImGui-free and line-at-a-time on purpose: the editor draws only the lines that
// are actually on screen, so tokenizing must be cheap per line and must not need
// the whole file. Block comments are the one construct that crosses a line
// boundary, so that single bit of carry is threaded through explicitly rather
// than hidden in a member - see tokenize().
//
// This is a HIGHLIGHTER, not a parser. It does not know types from identifiers
// in any real sense; it knows a fixed keyword list and a few lexical shapes.
// That is the correct amount of machinery for colouring text a human is reading.
#pragma once

// For IM_COL32 and ImU32. The tokenizer itself needs nothing from ImGui - only
// the palette below does - but a colour table that cannot be handed straight to
// a draw list would just move the conversion to every call site.
#include "imgui.h"

#include "shared.hxx"

namespace syn
{

  // What a run of characters is, as far as colour is concerned.
  enum class Role
  {
      ROLE_TEXT = 0,   // identifiers, whitespace, anything unclaimed
      ROLE_KEYWORD,    // if, while, return, static ...
      ROLE_TYPE,       // int, uint32_t, bool, void ...
      ROLE_NUMBER,
      ROLE_STRING,     // "..." and '...' alike
      ROLE_COMMENT,
      ROLE_PREPROC,    // a whole #... line
      ROLE_FUNCTION,   // an identifier immediately followed by '('
      ROLE_PUNCT,      // braces, operators, semicolons

      ROLE_COUNT
  };

  // One coloured run within a single line. [begin, end) are byte offsets.
  struct Span
  {
      Int32 begin = 0;
      Int32 end   = 0;
      Role  role  = Role::ROLE_TEXT;
  };

  // Tokenizes ONE line.
  //
  // `inBlock` is true if a /* opened on an earlier line and has not closed; it is
  // updated on return for the next line. A caller that starts mid-file must have
  // carried this from line 0, which is why the editor keeps a per-line flag.
  //
  // `out` is cleared first. Spans are emitted in order and cover the line with no
  // gaps, so a renderer can walk them and never has to ask what lies between two.
  Void tokenize(const Str& line, Bool& inBlock, Vec<Span>& out);

  // ---------------------------------------------------------------------------
  // Gruvbox dark (Pavel Pertsev's original "morhetz" palette), packed ABGR for
  // ImGui. Only the values actually used are here - a palette is a set of
  // decisions, and carrying the twenty unused ones invites arbitrary picks later.
  //
  // The role assignment follows what gruvbox.vim itself does for C, so a file
  // looks the way it would in vim rather than merely using the same colours:
  // keywords red, types yellow, strings green, preprocessor aqua, numbers purple,
  // comments grey, functions bright green.
  // ---------------------------------------------------------------------------
  namespace gruv
  {

    // Backgrounds and foregrounds.
    inline constexpr ImU32 BG0_H   = IM_COL32(0x1D, 0x20, 0x21, 0xFF);  // hard bg, the editor
    inline constexpr ImU32 BG0     = IM_COL32(0x28, 0x28, 0x28, 0xFF);
    inline constexpr ImU32 BG1     = IM_COL32(0x3C, 0x38, 0x36, 0xFF);  // current-line, gutter
    inline constexpr ImU32 BG2     = IM_COL32(0x50, 0x49, 0x45, 0xFF);  // selection
    inline constexpr ImU32 BG3     = IM_COL32(0x66, 0x5C, 0x54, 0xFF);
    inline constexpr ImU32 FG1     = IM_COL32(0xEB, 0xDB, 0xB2, 0xFF);  // main text
    inline constexpr ImU32 FG4     = IM_COL32(0xA8, 0x99, 0x84, 0xFF);  // line numbers
    inline constexpr ImU32 GRAY    = IM_COL32(0x92, 0x83, 0x74, 0xFF);  // comments

    // The bright accents. Gruvbox's dark variants are for light backgrounds.
    inline constexpr ImU32 RED     = IM_COL32(0xFB, 0x49, 0x34, 0xFF);
    inline constexpr ImU32 GREEN   = IM_COL32(0xB8, 0xBB, 0x26, 0xFF);
    inline constexpr ImU32 YELLOW  = IM_COL32(0xFA, 0xBD, 0x2F, 0xFF);
    inline constexpr ImU32 BLUE    = IM_COL32(0x83, 0xA5, 0x98, 0xFF);
    inline constexpr ImU32 PURPLE  = IM_COL32(0xD3, 0x86, 0x9B, 0xFF);
    inline constexpr ImU32 AQUA    = IM_COL32(0x8E, 0xC0, 0x7C, 0xFF);
    inline constexpr ImU32 ORANGE  = IM_COL32(0xFE, 0x80, 0x19, 0xFF);

  } // namespace gruv

  // The colour for a role. Free function rather than a table the caller indexes,
  // so adding a Role is a compile error here instead of a silent black span.
  [[nodiscard]] ImU32 colorFor(Role r) noexcept;

} // namespace syn
