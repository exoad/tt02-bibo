// Session logging: one file per launch, under logs/ at the repo root.
//
// ---------------------------------------------------------------------------
// WHY
//
// Everything this app talks to is a physical thing that can be unplugged, held
// open by something else, or running firmware that does not answer. When one of
// those goes wrong the UI shows a single red line - "flash failed", "no Pico
// found" - and the sequence that led there is gone the moment the frame is
// drawn.
//
// The evening of 2026-08-26 is the argument for this file. A flash failed with
// "RPI-RP2 never appeared" while the status bar said the Pico was connected, and
// answering "why" took reading the flash script, the SDK's USB reset source, and
// three separate hardware probes. Every fact needed was known to the app at the
// time and none of it was written down: which port was open, who opened it, what
// the script printed, what the board did next.
//
// ---------------------------------------------------------------------------
// WHAT IT IS NOT
//
// Not telemetry, and nothing leaves the machine. Not a debug channel for
// tracing draw calls either - a log nobody reads because it is 40 MB of frame
// noise has the same value as no log at all. What belongs here is what a person
// would want to know an hour later: connections, operations, and anything that
// failed.
//
// Writes are mutex-guarded because the lidar and the Pico link both run worker
// threads, and WARN/ERROR flush immediately so a crash still leaves the reason
// on disk.
// ---------------------------------------------------------------------------
#pragma once

#include "shared.hxx"

namespace applog
{

  enum class Level
  {
      LEVEL_DEBUG = 0,   // detail; kept out of the way but written
      LEVEL_INFO,        // something happened that a person would care about
      LEVEL_WARN,        // degraded, recoverable, worth noticing
      LEVEL_ERROR,       // an operation failed
  };

  // Opens logs/session-YYYYMMDD-HHMMSS.log and writes the header. Safe to call
  // twice; the second is a no-op. If the file cannot be opened the app runs
  // exactly as before and every write below becomes a no-op - losing a log must
  // never cost a session.
  Void init();

  // Flushes and closes. Called from app::shutdown().
  Void shutdown();

  // printf-style. Prefer the macros below so the call site carries its own tag.
  Void writef(Level level, const Char* tag, const Char* fmt, ...);

  // The file being written, for showing in the UI. Empty when logging is off.
  [[nodiscard]] Str path();

  // The directory, created on demand.
  [[nodiscard]] Str dir();

} // namespace applog

// The tag is a short subsystem name - "lidar", "pico", "flash", "code". It is
// what makes a log greppable, which is the only property that matters when you
// are looking at one at two in the morning.
#define LOG_DEBUG(tag, ...) ::applog::writef(::applog::Level::LEVEL_DEBUG, tag, __VA_ARGS__)
#define LOG_INFO(tag, ...)  ::applog::writef(::applog::Level::LEVEL_INFO,  tag, __VA_ARGS__)
#define LOG_WARN(tag, ...)  ::applog::writef(::applog::Level::LEVEL_WARN,  tag, __VA_ARGS__)
#define LOG_ERROR(tag, ...) ::applog::writef(::applog::Level::LEVEL_ERROR, tag, __VA_ARGS__)
