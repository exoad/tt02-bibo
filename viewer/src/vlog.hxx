// The viewer's round-trip log: one plain-text file per run, to read beside the
// board's journal. bibo.exe has no console, so this is its only record.
//
//   <directory of bibo.exe>\logs\viewer-YYYYMMDD-HHMMSS.log     UTC in the name
//   <directory of bibo.exe>\logs\latest.txt                     the newest path
//
// Beside the exe because viewer\build\ is gitignored; %TEMP% when that
// directory cannot be written, and the file's first line says which.
//
//   2026-09-12T20:28:30.143Z +12345ms [net] board PING token=41 ...
//
// UTC wall clock first, to match the board's UTC journal. Monotonic ms since
// open second, because the wall clock can be stepped mid-run: read gaps from
// the second column and moments from the first. Then the thread: [ui] is the
// frame loop, [net] the link worker.
//
// INTEGERS ONLY, here and in every viewer readout: printf's %f and ImGui's float
// widgets follow the locale and write "3,2" on a comma-decimal machine.
// FLUSHED EVERY LINE, so a killed process keeps its last lines. That is a
// system call per line, so no per-SCAN or per-CAMERA path may call line().
// NEVER FAILS: with no file open every call is a silent no-op.
// CAPPED at CAP_BYTES, and the file says so when the cap is hit.
#pragma once

#include "shared.hxx"

namespace vlog
{
  // Opens this run's file and records the calling thread as the UI thread.
  // Returns whether a file is open; a second call while open returns true.
  Bool open();

  // Writes a closing line and closes the file. Every line() after it is a no-op.
  Void close();

  // Names the calling thread in its lines, e.g. "net". Unnamed threads are "ui"
  // if they opened the log and "thread" otherwise.
  Void nameThread(CharSeq tag);

  // One line, printf-style, prefixed and flushed. Thread-safe.
  Void line(CharSeq fmt, ...);

  // printf-style, appended to `out`, for building a long line in parts.
  Void append(Str& out, CharSeq fmt, ...);

  [[nodiscard]] Bool isOpen();

  // The file this run is writing, or empty when none could be opened.
  [[nodiscard]] Str path();
}
