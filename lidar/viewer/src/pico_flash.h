// Firmware catalog and flashing, driven from the GUI.
//
// The point is to be able to load a different program onto the Pico on demand -
// the debug/blink build, the car control firmware, a one-off experiment - and to
// get back to a known-good image afterwards. Everything here shells out to the
// same scripts in firmware/ that work from a terminal, so there is exactly one
// flashing mechanism and the GUI is a front-end to it, not a second path.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

// One flashable image, read from firmware/catalog.txt plus what is on disk.
struct FirmwareEntry
{
    std::string id;            // stable key, e.g. "pico_debug"
    std::string name;          // display name
    std::string description;
    std::string uf2_path;      // absolute, resolved at scan time
    bool        buildable = false;   // has a build script we can invoke

    // Filled in by refresh_catalog() from the filesystem.
    bool        present    = false;
    long long   size_bytes = 0;
    std::string built_at;      // local time of the .uf2's mtime, "" if absent
};

// What the board is doing right now.
struct BoardStatus
{
    bool        present = false;   // any RP-series device visible at all
    bool        bootsel = false;   // sitting in the UF2 bootloader
    std::string port;              // CDC port when running firmware, else ""
    std::string drive;             // e.g. "D:" when in BOOTSEL, else ""
    std::string program;           // program name from picotool info, if readable
    std::string chip;              // e.g. "RP2350"
};

enum class FlashState
{
    Idle,
    Working,
    Success,
    Failed,
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
    void refresh_catalog();
    const std::vector<FirmwareEntry>& catalog() const;

    // ---- operations -----------------------------------------------------
    // All non-blocking: they start a worker and return immediately. Only one
    // runs at a time; a request while Working is rejected and logged.
    // Watch state(), and drain_log() for output.
    void build(const std::string& id);
    void flash(const std::string& id);
    void backup(const std::string& out_path);   // reads the board's flash back
    void reboot_bootsel();
    void reboot_normal();

    FlashState  state() const;
    std::string current_op() const;   // human-readable, e.g. "flashing pico_debug"
    bool        busy() const;

    // Moves newly produced output lines into `out` (appending). Call once per
    // UI frame. These are the raw lines from the underlying script.
    size_t drain_log(std::vector<std::string>& out);

    // ---- board ----------------------------------------------------------
    // Cached; refresh_board() re-queries. Querying spawns picotool, so it is
    // deliberately not automatic.
    BoardStatus board() const;
    void        refresh_board();

    // Repo root, derived from the executable location. Exposed because the UI
    // shows paths and the default backup location lives under it.
    static std::string repo_root();
};
