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
//   the LIBRARY   %LOCALAPPDATA%\bibo\sketches\*.c - saved programs,
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

  // Last-write time in seconds since 1970, for the callers that need to PRINT a
  // date rather than compare two. 0 when the file cannot be stat'd.
  //
  // Separate from stamp() because that one is documented as opaque, and a caller
  // that converts an opaque value is relying on something it was promised
  // nothing about. Seconds rather than std::time_t so this header still needs no
  // <ctime> - the point of this module is how few includes it forces on anyone.
  [[nodiscard]] Int64 modifiedAtUnix(const Str& path);

  // Deletes a file. False if it could not be removed.
  [[nodiscard]] Bool remove(const Str& path);

  // ---------------------------------------------------------------------------
  // What the Code view's right-click menu acts through.
  //
  // All of these take a name a person typed into a popup, which is why
  // validName() exists and why every one of them calls it. Handing an unchecked
  // string to CreateDirectory lets "..\..\hub\src" be a folder name.
  //
  // They report through `err` rather than throwing, for the same reason save()
  // does: the caller is a UI that has to say something useful.
  // ---------------------------------------------------------------------------

  // Whether `name` is a single, creatable filename - no separators, no
  // traversal, none of the characters Windows refuses, no trailing dot or
  // space. `err` says which rule it broke.
  [[nodiscard]] Bool validName(const Str& name, Str& err);

  // Whether a file with this name would appear in the tree: .cxx, .hxx, .c, .h
  // or .bdoc. The tree is an allow list, so New File checks this before
  // creating anything - a file the tree will not show is a file that looks like
  // it was never made.
  [[nodiscard]] Bool shownFile(const Str& name);

  // Creates an empty file. FAILS if one is already there rather than truncating
  // it: "New File" must never be a way to empty a file somebody already had.
  [[nodiscard]] Bool createFile(const Str& path, Str& err);

  // Creates one directory. Fails if it exists.
  [[nodiscard]] Bool createDir(const Str& path, Str& err);

  // Renames a file or a folder. Refuses to overwrite whatever is already at
  // `to` - a rename that destroys the file it lands on is the one outcome that
  // cannot be recovered from.
  [[nodiscard]] Bool rename(const Str& from, const Str& to, Str& err);

  // Removes an EMPTY directory. Deliberately not recursive: deleting a tree
  // from a right-click is a lot of destruction behind a small menu entry.
  [[nodiscard]] Bool removeDir(const Str& path, Str& err);

  // Whether the path is a directory. False for a missing path, so callers get
  // "not a directory" rather than having to tell the two apart.
  [[nodiscard]] Bool isDir(const Str& path);

  // Runs the repository's own formatter on one file, in place.
  //
  // tools/format.py, NOT clangd's textDocument/formatting. There is no
  // .clang-format in this tree, so clangd would apply LLVM defaults - and the
  // repo already has a formatter with rules of its own (call-site wrapping,
  // equals padding) that verify.bat enforces. Two formatters that disagree
  // about one file is a file that is never clean. Blocks for as long as the
  // script takes, which for one file is well under a second.
  [[nodiscard]] Bool formatFile(const Str& path, Str& err);

  // Opens Explorer with `path` selected. Best-effort and silent on failure -
  // nothing about the app depends on it.
  Void reveal(const Str& path);

  // Opens the generated firmware documentation in the default browser.
  //
  // It sits beside reveal() because this file already owns "ask Windows to
  // open something", not because documentation is a sketch concern.
  //
  // THE SITE HAS TO BE SERVED, not opened from disk. Nuxt writes absolute
  // asset URLs - `/_nuxt/entry.css` - which under file:// resolve to the root
  // of C: and load nothing, so a double-clicked index.html is unstyled text
  // with a dead sidebar. Checked against the built output, not assumed.
  //
  // website\docs.bat is the whole entry point: it builds the site if it has
  // never been built, starts a local server if one is not already listening,
  // and opens the browser. Keeping those three steps there rather than here
  // means the button and the shell do the same thing.
  //
  // Best-effort and silent on failure, like reveal().
  Void openDocs();

  // The program a new sketch starts from: a blink on GP28 with the wiring in a
  // comment above it. A blank buffer is a worse starting point than a working
  // program you can change one number in.
  [[nodiscard]] Str starter();

  // A name that does not collide with an existing sketch: sketch.cxx, sketch-2.c...
  [[nodiscard]] Str makeName();

}
