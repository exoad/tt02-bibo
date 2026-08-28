// Firmware catalog and flashing, driven from the GUI.
//
// The point is to be able to load a different program onto the Pico on demand -
// the debug/blink build, the car control firmware, a one-off experiment - and to
// get back to a known-good image afterwards. Everything here shells out to the
// same scripts in firmware/ that work from a terminal, so there is exactly one
// flashing mechanism and the GUI is a front-end to it, not a second path.
#pragma once

#include "shared.hxx"

#include <cstddef>

// One flashable image, read from firmware/catalog.txt plus what is on disk.
struct FirmwareEntry
{
    Str id;            // stable key, e.g. "pico_debug"
    Str name;          // display name
    Str description;
    Str uf2Path;      // absolute, resolved at scan time
    Bool        buildable = false;   // has a build script we can invoke

    // Which board this image is FOR - "pico2_w", "pico2", or "" when nothing
    // here builds it. Passed straight to build.bat, and shown next to the
    // entry, because a UF2 carries no hint of it: the RP2350 takes either image
    // without complaint and a wrong one fails silently at run time.
    Str board;

    // Filled in by refreshCatalog() from the filesystem.
    Bool        present    = false;
    Int64   sizeBytes = 0;
    Str builtAt;      // local time of the .uf2's mtime, "" if absent
};

// What the board is doing right now.
struct BoardStatus
{
    Bool        present = false;   // any RP-series device visible at all
    Bool        bootsel = false;   // sitting in the UF2 bootloader
    Str port;              // CDC port when running firmware, else ""
    Str drive;             // e.g. "D:" when in BOOTSEL, else ""
    Str program;           // program name from picotool info, if readable
    Str chip;              // e.g. "RP2350"
};

enum class FlashState
{
    FLASH_STATE_IDLE,
    FLASH_STATE_WORKING,
    FLASH_STATE_SUCCESS,
    FLASH_STATE_FAILED,
};

class PicoFlash
{
public:
    PicoFlash();
    ~PicoFlash();

    PicoFlash(const PicoFlash&)            = delete;
    PicoFlash& operator=(const PicoFlash&) = delete;

    // ---- catalog --------------------------------------------------------
    // Re-reads firmware/catalog.txt and stats each .uf2. Cheap; safe to call
    // when the user asks, not every frame.
    Void refreshCatalog();
    const Vec<FirmwareEntry>& catalog() const;

    // ---- operations -----------------------------------------------------
    // All non-blocking: they start a worker and return immediately. Only one
    // runs at a time; a request while Working is rejected and logged.
    // Watch state(), and drainLog() for output.
    Void build(const Str& id);
    Void flash(const Str& id);
    Void backup(const Str& outPath);   // reads the board's flash back
    Void rebootBootsel();
    Void rebootNormal();

    FlashState  state() const;
    Str currentOp() const;   // human-readable, e.g. "flashing pico_debug"
    Bool        busy() const;

    // Moves newly produced output lines into `out` (appending). Call once per
    // UI frame. These are the raw lines from the underlying script.
    Size drainLog(Vec<Str>& out);

    // ---- board ----------------------------------------------------------
    // Cached; refreshBoard() re-queries. Querying spawns picotool, so it is
    // deliberately not automatic.
    BoardStatus board() const;
    Void        refreshBoard();

    // Repo root, derived from the executable location. Exposed because the UI
    // shows paths and the default backup location lives under it.
    static Str repoRoot();
};
