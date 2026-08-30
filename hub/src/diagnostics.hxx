// Compiler diagnostics for the code editor: gutter marks, underlines, tooltips.
//
// ---------------------------------------------------------------------------
// WHY THIS IS NOT AN LSP
//
// A language server would mean shipping clangd, keeping a compile database in
// step with two build systems, and running a second parser that disagrees with
// the one that actually builds the firmware. For a scratch-sketch editor that is
// a great deal of machinery to arrive at a worse answer.
//
// The compiler is already right there and already knows. arm-none-eabi-gcc emits
//
//     C:/path/sketch.cxx:42:15: error: 'foo' undeclared (first use in this function)
//
// on every build, and that is the ground truth - the same diagnosis the build
// failed on, not an approximation of it. This file turns those lines into marks
// in the gutter.
//
// The trade, stated plainly: diagnostics appear when you BUILD, not as you type.
// You get the real compiler's opinion a second after asking for it, rather than
// an imitation of it continuously. For a file you flash to a microcontroller,
// that is the right side of the trade.
//
// ImGui-free and tested in tests/test_diagnostics.cxx.
#pragma once

#include "shared.hxx"

namespace diag
{

  // SEVERITY_ERR rather than the obvious SEVERITY_ERROR, and SEVERITY_WARN rather
  // than SEVERITY_WARNING: <winerror.h> defines both of those as macros for
  // building HRESULTs, so any translation unit that reaches windows.h before this
  // header would see the enumerators textually replaced and fail with a syntax
  // error pointing here rather than at the collision. Renaming is rude to nobody;
  // #undef-ing another library's macros from a public header is.
  enum class Severity
  {
      SEVERITY_NOTE = 0,
      SEVERITY_WARN,
      SEVERITY_ERR,
  };

  struct Item
  {
      Str      file;              // exactly as the compiler spelled it
      Int32    line = 0;          // 1-based, as the compiler counts
      Int32    column = 0;        // 1-based; 0 when the compiler gave none
      Severity severity = Severity::SEVERITY_ERR;
      Str      message;
  };

  // Parses ONE line of build output. Returns false when the line is not a
  // diagnostic, which is most of them.
  //
  // Handles the shapes gcc, clang and MSVC actually emit:
  //
  //     path:LINE:COL: error: message          gcc / clang
  //     path:LINE: error: message              gcc, no column
  //     path(LINE,COL): error C2065: message    MSVC
  //     path(LINE): error C2065: message        MSVC
  //
  // A Windows drive letter is the reason this cannot be a naive split on ':' -
  // "C:/x/y.c:42:1: error: z" has four colons and the first one is not a
  // separator.
  [[nodiscard]] Bool parseLine(const Str& text, Item& out);

  // Every diagnostic in a block of build output, in the order emitted.
  [[nodiscard]] Vec<Item> parseAll(const Vec<Str>& lines);

  // Those belonging to `path`, matched on the FILE NAME rather than the whole
  // path. The compiler sees firmware/scratch/sketch.cxx while the editor may be showing
  // the same bytes from the sketch library, and refusing to mark the file the user
  // is actually looking at would make the feature useless in the one workflow it
  // exists for.
  [[nodiscard]] Vec<Item> forFile(const Vec<Item>& all, const Str& path);

  // The worst severity on `line`, or -1 if the line is clean. Used per row while
  // drawing, so it is a plain scan rather than a map: a sketch has tens of
  // diagnostics at the very worst, and building an index would cost more than it
  // saves.
  [[nodiscard]] Int32 worstOnLine(const Vec<Item>& items, Int32 line);

} // namespace diag
