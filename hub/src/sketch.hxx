// Firmware programs ("sketches") written in the hub's Code view.
//
// A sketch is one .cxx file compiled against pico_stdlib and the hardware
// units - no lwIP. The point is a short round trip: type, press Build & Flash,
// watch the LED. It is where you find out what a GPIO does.
//
// ---------------------------------------------------------------------------
// ONE LOCATION NOW: firmware/sketches/*.cxx, in the repo, in git.
//
// There used to be two, and the pair was the problem:
//
//   the LIBRARY   %LOCALAPPDATA%\tt02-auto\sketches\*.c - saved programs,
//                 outside the repo and outside git.
//   the SLOT      firmware/scratch/sketch.cxx - what CMake compiled, COPIED
//                 OVER from the library before every single build.
//
// So the file being compiled was never the file you had open, the slot's
// previous contents were destroyed by pressing Build, and the programs
// themselves lived somewhere no clone, backup or `git log` would ever find. A
// working VL53L1X range view sat in that slot for weeks, one keystroke from
// gone; it is firmware/sketches/range-view.cxx now.
//
// Today firmware/sketches/*.cxx each get their OWN CMake target named after the
// file, so the Code view builds the file on screen by name. Nothing is copied
// anywhere and nothing is overwritten.
//
//   app/        the car. Multi-file, one image, the finished thing.
//   sketches/   findings and side quests. One file, one image, one question.
//
// There is exactly one compiler path in this project and this is a front-end to
// it, not a second one.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

namespace sketch
{

  // firmware/sketches, created on demand. Empty if the repo root cannot be found,
  // in which case the Code view still works but cannot save.
  [[nodiscard]] Str dir();

  // The sketches, alphabetical, bare filenames including the extension.
  [[nodiscard]] Vec<Str> list();

  // Absolute path of a sketch by bare filename.
  [[nodiscard]] Str pathOf(const Str& name);

  // firmware/src - the real firmware sources, absolute. Empty if the repo root
  // cannot be found (a copied exe with no tree around it).
  [[nodiscard]] Str firmwareDir();

  // The .c and .h files in firmware/src, bare filenames, alphabetical. These are
  // editable from the Code view too: it is the same toolchain and the same style
  // guide, and having to leave the app to fix a typo in main.c would be silly.
  [[nodiscard]] Vec<Str> listFirmware();

  // Which catalog target owns `path`, so Build & Flash writes the image that
  // actually contains the file on screen. A sketch owns a target named after the
  // file - firmware/sketches/range-view.cxx builds range-view - and everything
  // else under firmware/ belongs to "pico_debug".
  //
  // Getting this wrong is the worst failure this view has: it would build
  // successfully, flash successfully, and run code the user did not edit. That is
  // no longer a naming convention this function has to guess at - CMake derives
  // the target from the same filename, so the two cannot drift apart.
  [[nodiscard]] Str targetFor(const Str& path);

  // Whole-file text. load() returns empty for a missing file; save() reports
  // failure through `err` rather than throwing, because the caller is a UI that
  // has to say something useful instead of dying.
  [[nodiscard]] Str  load(const Str& path);
  [[nodiscard]] Bool save(const Str& path, const Str& text, Str& err);

  // ---------------------------------------------------------------------------
  // Filesystem operations, kept HERE rather than in the UI.
  //
  // This file already owns every Windows call the sketch library makes. Pulling
  // <windows.h> into app_ui.cxx to add three more brought min/max and
  // SEVERITY_ERROR with it and broke two unrelated headers - the macros in that
  // header are a genuine hazard and the fewer translation units that see them the
  // better.
  // ---------------------------------------------------------------------------

  // Last-write time as an opaque, comparable number. 0 when the file cannot be
  // stat'd, which callers must treat as "unknown", not as "very old".
  [[nodiscard]] UInt64 stamp(const Str& path);

  // Deletes a file. False if it could not be removed.
  [[nodiscard]] Bool remove(const Str& path);

  // Opens Explorer with `path` selected. Best-effort and silent on failure -
  // nothing about the app depends on it.
  Void reveal(const Str& path);

  // The program a new sketch starts from: a blink on GP28 with the wiring in a
  // comment above it. A blank buffer is a worse starting point than a working
  // program you can change one number in.
  [[nodiscard]] Str starter();

  // A name that does not collide with an existing sketch: sketch.cxx, sketch-2.c...
  [[nodiscard]] Str makeName();

} // namespace sketch
