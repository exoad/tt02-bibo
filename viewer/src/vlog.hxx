// The viewer's round-trip log: one plain-text file per run, written line by line
// as things happen, so this end's account of a connection can be laid beside the
// board's own journal and read as one story.
//
// ---------------------------------------------------------------------------
// WHY THIS EXISTS
//
// bibo.exe is a /SUBSYSTEM:WINDOWS program. It has no console, and until this
// module it wrote nothing anywhere at all. When the board's frame ring showed a
// viewer that kept SENDING its own PINGs and COMMANDs while it stopped ANSWERING
// the board's, there was no way to ask the viewer what it had experienced - the
// half of the round trip that decides the diagnosis was the half nobody could
// read. That is this repo's recurring bug class again: not a wrong number but a
// missing one.
//
// ---------------------------------------------------------------------------
// WHERE IT GOES
//
//   <directory of bibo.exe>\logs\viewer-YYYYMMDD-HHMMSS.log     UTC in the name
//   <directory of bibo.exe>\logs\latest.txt                     the newest path
//
// Beside the exe rather than in a profile directory, because the exe lives in
// viewer\build\, which is gitignored: a log there can never be committed by a
// careless `git add`. If that directory cannot be written, %TEMP% is tried, and
// the first line of the file says which one won.
//
// ---------------------------------------------------------------------------
// THE LINE
//
//   2026-09-12T20:28:30.143Z +12345ms [net] board PING token=41 ...
//
// UTC WALL CLOCK FIRST, because the whole point is correlation with a board
// whose journal is stamped in UTC, and a local time would make every comparison
// a timezone sum somebody gets wrong at midnight. MONOTONIC MS SINCE OPEN
// second, because the wall clock can be stepped by time sync mid-run and an
// interval measured across a step is a lie - so gaps are read off the second
// column and moments off the first. THE THREAD third: [ui] is the frame loop,
// [net] is the link worker, and a question like "did the button press reach the
// socket" is answered by reading the two interleaved.
//
// ---------------------------------------------------------------------------
// RULES FOR WHAT IS WRITTEN
//
// INTEGERS ONLY. printf's %f honours the locale, a machine set to a comma
// decimal writes "3,2", and this repo has been bitten by exactly that three
// times. Milliseconds, bytes and milli-units are integers already; anything
// that must show a fraction does it with integer arithmetic, as link.cxx does.
//
// FLUSHED EVERY LINE, so a process that is killed from Task Manager mid-outage
// still leaves the lines that led up to it. That costs a system call per line,
// which is why nothing on a per-SCAN or per-CAMERA path may call line().
//
// NEVER A REASON TO FAIL. If no file could be opened every call is a silent
// no-op; a diagnostic that can take the program down is worse than none.
//
// CAPPED at 50 MB, and the cap says so once, in the file, rather than the file
// simply stopping - a log that ends without saying why reads as a program that
// stopped doing things.
#pragma once

#include "shared.hxx"

namespace vlog
{

  // Opens this run's file and records the calling thread as the UI thread.
  // Returns whether a file is open. A second call while one is open does nothing
  // and returns true.
  Bool open();

  // Writes a closing line and closes the file. Every line() after it is a no-op.
  Void close();

  // Names the calling thread in every line it writes: "net" for the link worker.
  // A thread that never names itself is "ui" if it opened the log and "thread"
  // otherwise - never a guess that could put a line on the wrong side of a seam.
  Void nameThread(CharSeq tag);

  // One line, printf-style, prefixed and flushed. Thread-safe.
  Void line(CharSeq fmt, ...);

  // printf-style, appended to `out`. For building a long line in parts - the
  // once-a-second summary - without a format string nobody can read.
  Void append(Str& out, CharSeq fmt, ...);

  [[nodiscard]] Bool isOpen();

  // The file this run is writing, or empty when none could be opened.
  [[nodiscard]] Str path();

}
