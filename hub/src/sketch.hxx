// Scratch firmware programs ("sketches") written in the hub's Code view.
//
// A sketch is one .c file compiled against pico_stdlib and nothing else. The
// point is a short round trip: type, press Build & Flash, watch the LED. It is
// where you find out what a GPIO does, and it is deliberately NOT where the
// car's firmware lives - anything worth keeping graduates to its own .c file and
// its own target in firmware/CMakeLists.txt.
//
// ---------------------------------------------------------------------------
// TWO LOCATIONS, and the difference matters.
//
//   the LIBRARY   %LOCALAPPDATA%\tt02-auto\sketches\*.c
//                 Your saved programs. Survives `build.bat clean`, survives a
//                 fresh clone, is not in git.
//
//   the SLOT      firmware/scratch/sketch.cxx
//                 What CMake actually compiles. Overwritten on every build.
//
// Build & Flash copies library -> slot and then runs the existing firmware
// build/flash scripts. There is exactly one compiler path in this project and
// this is a front-end to it, not a second one.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

namespace sketch {

// %LOCALAPPDATA%\tt02-auto\sketches, created on demand. Empty if there is no
// user profile, in which case the Code view still works but cannot save.
[[nodiscard]] Str dir();

// Saved sketches, newest first, bare filenames including the .c.
[[nodiscard]] Vec<Str> list();

// Absolute path of a library sketch by bare filename.
[[nodiscard]] Str pathOf(const Str& name);

// firmware/scratch/sketch.cxx - the file CMake compiles. Absolute.
[[nodiscard]] Str slotPath();

// firmware/src - the real firmware sources, absolute. Empty if the repo root
// cannot be found (a copied exe with no tree around it).
[[nodiscard]] Str firmwareDir();

// The .c and .h files in firmware/src, bare filenames, alphabetical. These are
// editable from the Code view too: it is the same toolchain and the same style
// guide, and having to leave the app to fix a typo in main.c would be silly.
[[nodiscard]] Vec<Str> listFirmware();

// Which catalog target owns `path`, so Build & Flash writes the image that
// actually contains the file on screen. "sketch" for the scratch slot and for
// anything in the library, "pico_debug" for the rest of firmware/src.
//
// Getting this wrong is the worst failure this view has: it would build
// successfully, flash successfully, and run code the user did not edit.
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
