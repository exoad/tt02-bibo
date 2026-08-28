// Application layout: a dashboard, not a set of pages.
//
//   status strip   full width, one line, always visible
//   the map        the whole left region, permanently. Never replaced, never
//                  hidden - it is the thing this app is for.
//   control bar    under the map: range, overlays, reset view
//   right sidebar  one scrollable column holding everything else, as collapsing
//                  sections: System, Sensors, Vehicle, Firmware, Console
//
// There are no workspaces. The old left nav rail swapped the centre pane between
// five screens, which meant the map - the only continuously useful surface -
// disappeared four times out of five. Now the map is fixed and the panels queue
// up beside it.
//
// The sidebar scrolls; that is what a sidebar is. The two logs get their own
// fixed-height inner scroll regions, because a scrolling log inside a scrolling
// column cannot be used.
//
// The screen shows live state and offers actions. It does not explain itself in
// prose: a readout is a short label and a value, absence is "--" or a muted
// label, and a warning belongs in the confirm modal at the moment of risk, not
// parked permanently on screen. Project background lives in docs/conventions.md and
// docs/log.md, which is where it can actually be read.
#include "shared.hxx"
#include "app_ui.hxx"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cstring>
#include <ctime>

#include "imgui.h"
#include "lidar_source.hxx"
#include "pico_flash.hxx"
#include "pico_link.hxx"
#include "pinout.hxx"
#include "radar.hxx"
#include "icons.hxx"
#include "lights.hxx"
#include "applog.hxx"
#include "diagnostics.hxx"
#include "code_view.hxx"
#include "editor.hxx"
#include "recording.hxx"
#include "sketch.hxx"
#include "reference.hxx"
#include "devlink.hxx"
#include "lint.hxx"
#include "settings.hxx"
#include "theme.hxx"

namespace {

LidarSource lidarSource;
RadarView   radarView;
PicoLink    picoLink;
PicoFlash   picoFlash;

Vec<Str> lidarPorts;
Vec<const Char*> portItems;
Int32   portIndex  = -1;
Int32   baudIndex  = 0;     // 460800, the only rate a C1 has
Int32   rangeIndex = 0;     // Fit
Float32 uiDpiScale         = 1.0f;

LidarFrame latestFrame;
Bool       haveFrame = false;

// The C1 is specified over 0.05 - 12 m. Returns outside that window are the
// housing (below) or unreliable long-range noise (above). The same window
// governs what the map draws, so nothing is ever shown that is not also
// counted.
constexpr Float32 MIN_VALID_MM = 50.0f;
constexpr Float32 MAX_VALID_MM = 12000.0f;

// ---------------------------------------------------------------- derived ---
// Recomputed once per revolution, not per UI frame. LidarFrame's own
// validCount / maxDistMm are deliberately raw (every return the device
// sent), so everything spec-bounded is derived here.

Float32 meanMm   = 0.0f;
Float32 maxRangeMm = 0.0f;
Float32 pointsPs = 0.0f;

// Return classification. These four sum to the revolution's sample count.
Int32 nInspec   = 0;
Int32 nNoreturn = 0;   // dist == 0, the device saw nothing that way
Int32 nToonear  = 0;   // 0 < dist < 50 mm, housing reflection
Int32 nToofar   = 0;   // dist > 12 m, beyond the rated range

// Signal quality, over in-spec returns only.
Float32 qMean = 0.0f;
Int32   qMin  = 0;
Int32   qMax  = 0;

constexpr Int32 QUALITY_BUCKETS = 16;         // 0..63 folded into 16 bins
Float32 qHist[QUALITY_BUCKETS] = {};
Float32 qHistMax = 1.0f;

constexpr Int32 DIST_BUCKETS = 24;            // 0..12 m in 0.5 m bins
Float32 distHist[DIST_BUCKETS] = {};
Float32 distHistMax = 1.0f;

// Angular coverage: fraction of 1-degree bins with at least one in-spec return.
Float32 coverageDeg = 0.0f;

// Clearance: distance to the nearest return in each 30 degree sector, in metres.
constexpr Int32 SECTORS = 12;
Float32 sectorM[SECTORS] = {};
constexpr Float32 CLEARANCE_CAP_M = 2.5f;   // beyond this a direction is just "clear"

// ------------------------------------------------------------- pico link ---
// The debug/bring-up channel to the Pico 2 W over USB CDC. Ports, the drained
// line log, and the console's own view state.
//
// If a connected board ever appears to swallow writes: TinyUSB CDC refuses OUT
// data until the host asserts DTR, and a write without it blocks until the
// driver gives up with "the semaphore timeout period has expired". That reads
// exactly like dead hardware and is not. pico_link.cxx asserts DTR; do not
// remove it.

Vec<Str> picoPorts;
Vec<const Char*> picoItems;
Int32  picoIndex = -1;

// The console is a debug aid, not a record: this app runs for hours, so the log
// is bounded and the oldest lines fall off the front.
constexpr Size LOG_MAX = 4000;
Vec<PicoLine> picoLog;

// Console view state. The selection is by INDEX into picoLog, so it survives
// the filter being retyped - selecting five lines, filtering, and clearing the
// filter should give you back the same five.
Set<Int32> logSel;
Int32      logSelAnchor = -1;   // for shift-click runs. -1 = nothing anchored
Bool       logShowPoll  = false;

// Whether the most recent line SENT was one of the hub's own polls, so the
// replies that follow it can be marked the same way. See pumpData().
Bool lastSendWasPoll = false;
Vec<Int32>      logShown;    // indices passing the filter, rebuilt per frame

Char cmdBuf[192]   = {};
Char filterBuf[64] = {};
Bool logAutoscroll = true;

// Result of the last BOOTSEL touch, so a failure is not silent.
Bool bootselDone = false;
Bool bootselOk   = false;

// Opened from more than one workspace now, so the modal lives at the root of
// the frame and this is the request to raise it. A popup's identity comes from
// the ID stack, so OpenPopup and BeginPopupModal must sit at the same level.
Bool openBootsel = false;

// ------------------------------------------------------- controller state ---
// What the board says about the car, as opposed to what the serial port says
// about the board.
//
// tt02_control answers `?` with "S <uptimeMs> <a> <b> <servoUs> <escUs> <t>"
// then "OK"; fields 4 and 5 are the servo and ESC pulse widths in microseconds
// (1500 = neutral). pico_debug, which is what is flashed right now, answers
// PING/ID/STATUS/HELP/LED and returns "ERR bad command" to `?`.
//
// Both are handled by observing what actually arrives. Nothing here guesses a
// pulse width; when the firmware does not report one, the readout says so.

struct VehicleStatus
{
    Bool               have      = false;
    Float64             seenAt   = 0.0;   // ImGui::GetTime() when the line landed
    UInt64 uptimeMs = 0;
    long               a         = 0;
    long               b         = 0;
    Int32                servoUs  = 0;
    Int32                escUs    = 0;
    UInt64 lastMs   = 0;     // field 6, tracks uptime - last command
};

VehicleStatus vehicleStatus;
Bool          vehUnsupported = false;   // `?` was answered, but not with an S line
Bool          vehAwait       = false;   // `?` sent, first reply not yet seen
Str   lastCmd;                  // last line we sent, trimmed

// ---- what pico_debug reports about itself ---------------------------------
//
// Parsed out of `INFO status` / `INFO id` / `OK led ...`, which is the
// vocabulary in firmware/app/main.c. Kept separate from VehicleStatus above
// because the two come from DIFFERENT firmware: pico_debug answers STATUS,
// tt02_control answers `?`, and neither understands the other's command. A
// board runs one of them, so at most one of these two structs is ever live.
struct DebugStatus
{
    // Which board is on the other end - "pico2_w" or "pico2" - straight from
    // INFO id's board= field.
    //
    // This matters more than it looks. There are two RP2350 boards in this
    // project now, the mule and the car's, and over USB they are indis-
    // tinguishable: same VID, same chip, same bootloader, adjacent COM numbers
    // that swap around on replug. The firmware is the only thing that knows,
    // because it was COMPILED for one of them, so this is the one authoritative
    // answer available and it belongs on screen rather than in a log line.
    Str boardName;
};

DebugStatus debugStatus;

// STATUS polling. tt02_control answers `ERR bad command`, so a board running it
// must not be asked again every two seconds forever - one refusal is enough.
// ---- what the board says is attached ------------------------------------
//
// The hub cannot see the Pico's pins. It can only ask, which is why these are
// answers rather than assumptions - the difference between a row that reads
// "not wired" because nothing is wired and one that reads it because nobody
// ever checked.
//
// `tofAsked` separates "no" from "not yet". Before the first reply the honest
// state is unknown, and drawing that as absent would be a guess.
Bool   sensorsAsked  = false;
Bool   sensorI2c     = false;
Bool   sensorTof     = false;

// The newest range, and whether it is worth believing. A distance that arrived
// with a bad status is not a shorter distance - it is not a distance - so the
// two are kept together and never separated.
Int32   tofMm        = 0;
Int32   tofStatus    = 255;

// The rates that came with the newest reading, in the sensor's own 16.16 fixed
// point, or -1 when the firmware did not send them.
//
// These are what make a wrong-looking number diagnosable rather than merely
// wrong: a STRONG signal at a short distance means something really is that
// close - a protective film on the lens is the classic - while a weak signal
// with a high ambient means the sensor is being blinded by room infrared.
Int32   tofSignal    = -1;
Int32   tofAmbient   = -1;

// Which distance mode the board is in. The firmware boots in LONG, so that is
// what this starts as - it is a mirror of the board's state, not a request.
Bool    tofModeShort = false;

// ---- the drive channels -------------------------------------------------
//
// Mirrors of the BOARD's state, not requests. The board owns the limits and
// the arming, because it is the thing holding the wires - a hub that decided
// those would be a hub whose safety evaporates the moment it disconnects.
Bool  driveKnown   = false;
Int32 driveServo   = 1500;   // what the board is outputting
Int32 driveServoT  = 1500;   // what it is heading toward
Int32 driveEsc     = 1500;
Int32 driveEscT    = 1500;
Bool  driveArmed   = false;

// Whether the board is driving the steering pin at all. Not the same question
// as "what position is it holding" - a released servo holds nothing, which is
// the only state that is safe on a car whose centre is not its centre.
Bool  driveServoOn = false;

// Where the board thinks centre is. Not 1500 in general - see the calibration
// block below for why that matters more than it sounds.
Int32 driveServoC  = 1500;

// How fast the board lets an output move, in microseconds per 20 ms tick.
// 8 is 400 us/s, which walks this car's 440 us of travel in 1.1 seconds.
Int32 driveSlew = 8;

// What the slider shows, and whether it is under the thumb. Same split as the
// steering: a reply arriving mid-drag must not yank the handle out from under
// the person moving it.
Int32 driveSlewWant = 8;
Bool  driveSlewHeld = false;

// How far past idle the throttle must go before the tail lamps go out, in
// microseconds. The board owns it; this follows unless the slider is being
// dragged, the same deal every other control here makes.
Int32 lightsOffWant = 10;
Bool  lightsOffHeld = false;
Int32 boardLightsOff = 10;

// Steering as a fraction of this car's travel, -1 to +1. What the board
// reports, and what the slider shows.
//
// The board sends it in thousandths so no float has to survive a printf on a
// microcontroller; it becomes a fraction here, where floats are free.
Float32 driveSteer     = 0.0f;

// Where the wheels ACTUALLY are, as against driveSteer which is where they were
// told to go. The board reports both; the slew limiter is the difference.
//
// The drawing uses THIS one. It used to use the target, and that made the Drive
// view tell two stories at once: with the servo released, dragging the slider
// turned the wheels on screen while the indicator stayed dark - correctly, since
// drivePump only moves servoNow while the servo is live - and the obvious
// reading of that is "the light is broken" rather than "the wheels have not
// moved". One number for both, and the picture cannot disagree with the lamp.
Float32 driveSteerNow  = 0.0f;
Float32 driveSteerWant = 0.0f;
Bool    driveSteerHeld = false;

// ---- the calibration ----------------------------------------------------
//
// Three measurements of ONE car: the two ends its steering can actually reach,
// and where its wheels point straight. None of them are derivable. A servo's
// range is 1000-2000 us and says nothing about linkage length, and the horn
// only meets its spline at whole-tooth intervals, so straight-ahead lands
// wherever it lands.
//
// Held here as the working copy, saved to settings so a session does not lose
// them, and written out to firmware/lib/chassis/cal.h when the user commits -
// three places on purpose. Settings is what survives a restart; the header is
// what survives a reflash and what other code can actually read.
Int32 calLeft   = 1300;
Int32 calCenter = 1500;
Int32 calRight  = 1700;
Bool  calLoaded = false;
Bool  calDirty  = false;

// What the header on disk says, so the view can show whether the working copy
// has drifted from what the firmware would actually be built with.
Str calWritten;

// The limits the BOARD reports. Sliders are built from these rather than from
// constants here, so tightening them in firmware tightens the UI too and the
// two can never disagree about what is safe.
// ---- the indicator scaffolding. TEMPORARY - see firmware/lib/lights.h ----
//
// What the BOARD says its lamps are doing, not what this hub would have
// decided. The rule runs in the firmware, because the car has to indicate when
// no laptop is attached; this only draws the answer. lights::detect() in the
// hub is the same rule for the 3D view and the two are deliberately separate -
// one is the car, the other is a picture of it.
Int32   boardTurn     = 0;       // -1 left, 0 off, +1 right
Bool    boardLightsOn = true;
Float64 lightsLastPoll = 0.0;

// Every lamp in the firmware's model, in its Lamp order:
//   0 headL  1 headR  2 tailL  3 tailR  4 indL  5 indR  6 revL  7 revR
//
// Levels 0..255, straight from the board. A lamp with no LED soldered to it
// still has a correct level - the rule computes all eight whether or not the
// wiring shows them - so the drawing can display the whole car's lighting while
// only two of them exist in copper. That is the point of the split: wiring the
// next LED changes a table in the firmware and nothing here.
constexpr Int32 LAMP_N = 8;
Int32 boardLamp[LAMP_N] = {};
Int32 boardLampPin[LAMP_N] = { -1, -1, -1, -1, -1, -1, -1, -1 };

Int32 driveServoMin = 1300;
Int32 driveServoMax = 1700;
Int32 driveEscMin   = 1500;
Int32 driveEscMax   = 1600;

// What the sliders are showing. Separate from the board's value so dragging is
// smooth - snapping the handle to a reply that arrives every 200 ms would make
// the control feel broken.
Int32 driveServoWant = 1500;
Int32 driveEscWant   = 1500;

// ---- the sweep ----------------------------------------------------------
//
// Driven from the hub rather than the board: it is a testing convenience, and
// putting it in firmware would mean a car that can start moving on its own
// because of something left running in a UI.
Bool    driveSweep     = false;
Float64 driveSweepNext = 0.0;
Int32   driveSweepDir  = 1;

// The limits the user is editing, separate from what the board has accepted.
// Widening is a two-step act - type it, then apply it - because a limit that
// moved as you dragged would be no limit at all.
Int32 driveLimitLo = 1300;
Int32 driveLimitHi = 1700;
Bool  driveLimitsDirty = false;

Int32 driveEscLimitLo = 1500;
Int32 driveEscLimitHi = 1600;
Bool  driveEscLimitsDirty = false;

// When the board was last asked what its drive state is. Throttled, because the
// asking happens from a draw and a draw happens sixty times a second.
Float64 driveAskedAt = -1.0;

// Whether a slider handle is under the user's thumb RIGHT NOW. The board's
// replies must not move a handle being dragged, but they must move everything
// else - so this is per-slider rather than "is anything in the app active".
Bool driveServoHeld = false;
Bool driveEscHeld   = false;
Float64 tofLastReply = 0.0;
UInt64  tofReplies   = 0;

// A rolling history for the strip chart. Fixed size, oldest overwritten - a
// chart that grows without bound is a leak with a picture on it.
constexpr Int32 TOF_HISTORY = 240;
Float32 tofHistory[TOF_HISTORY] = {};
Int32   tofHistoryAt = 0;
Bool    tofHistoryWrapped = false;

// Extremes since connect. Sweeping the sensor and reading these off is the
// honest answer to "how far does it reach" for THIS sensor in THIS light,
// which no datasheet figure can give.
Int32 tofSeenMin = 0;
Int32 tofSeenMax = 0;

Bool   dbgUnsupported = false;
Bool   dbgAwait       = false;
Float64 dbgLastPoll   = 0.0;

Void resetBoardStatus()
{
    debugStatus             = DebugStatus();
    dbgUnsupported = false;
    sensorsAsked   = false;
    sensorI2c      = false;
    sensorTof      = false;
    tofMm          = 0;
    tofStatus      = 255;
    tofReplies     = 0;
    tofHistoryAt   = 0;
    tofHistoryWrapped = false;
    tofSeenMin     = 0;
    tofSeenMax     = 0;
    tofSignal      = -1;
    tofAmbient     = -1;
    tofModeShort   = false;
    driveKnown     = false;
    driveArmed     = false;
    driveServoOn   = false;
    driveServoC    = 1500;
    driveSlew      = 8;
    driveSlewWant  = 8;
    driveSlewHeld  = false;
    driveSteer     = 0.0f;
    driveSteerWant = 0.0f;
    driveSteerHeld = false;
    driveServo     = 1500;
    driveServoT    = 1500;
    driveEsc       = 1500;
    driveEscT      = 1500;
    driveServoWant = 1500;
    driveEscWant   = 1500;
    dbgAwait       = false;
    dbgLastPoll   = 0.0;
}

// ----------------------------------------------------------------- flash ---
// The firmware suite: catalog, board state, and the output of whichever script
// is running. PicoFlash does the work on a worker thread; everything here is
// display plus the one confirmation that stands between a click and an
// irreversible overwrite.

constexpr Size FLASH_LOG_MAX = 3000;
Vec<Str> flashLog;

Char backupBuf[320] = {};
Bool flashAutoscroll = true;

// Set when the confirm modal is opened, so the modal can name what it is about
// to destroy rather than saying "the firmware".
Str confirmId;
Str confirmName;
Str confirmPath;

FlashState flashPrev = FlashState::FLASH_STATE_IDLE;

// --------------------------------------------------------------- sidebar ---
// The right column's sections. Order is the order they are drawn in; System and
// Sensors are open by default because they are the two you read, the rest are
// things you go and do.

// Unscoped on purpose. SECTION_COUNT is an array bound and the rest are array
// indices at two dozen sites; `enum class` would add a static_cast to every one
// of them and change nothing about what the code means.
enum Section
{
    SECTION_SYSTEM = 0,
    SECTION_SENSORS,
    SECTION_VEHICLE,
    SECTION_FIRMWARE,
    SECTION_COUNT,
};

// ---------------------------------------------------------------------------
// Panel layout: what order the sections are in, which ones have been torn off
// into their own windows, and how wide the column is.
//
// All three are the user's, not the app's, so all three persist. The floating
// windows' positions and sizes are ImGui's own business - it writes them to
// layout.ini - and what is kept here is only the fact that they ARE floating,
// which ImGui has no way of knowing.
// ---------------------------------------------------------------------------
// The console is NOT here. It was, until it moved to its own column on the
// left - see drawConsoleColumn(). A settings file written before that has five
// records; the loader bounds-checks each id against SECTION_COUNT and only
// accepts a complete permutation, so the stale fifth is dropped and the other
// four load normally.
Int32 sectionOrder[SECTION_COUNT] = { SECTION_SYSTEM, SECTION_SENSORS, SECTION_VEHICLE,
                                  SECTION_FIRMWARE };
Bool  sectionFloating[SECTION_COUNT] = {};

// Logical (96-dpi) pixels, so the column keeps its apparent width across a DPI
// change or a zoom rather than growing in one and not the other.
Float32 sidebarLogicalW = 400.0f;

// ---- the console column, on the LEFT ------------------------------------
//
// Its own column rather than a section in the right-hand sidebar, because it is
// the one panel you read WHILE doing something else. Sharing the sidebar meant
// it competed for height with System and Sensors and got about a fifth of a
// screen, which is four lines of a log that produces hundreds.
//
// On the left because the sidebar is on the right and a console between the two
// would put the thing you glance at in the middle of the thing you work in.
Float32 consoleLogicalW = 380.0f;
Bool    consoleOpen     = false;

constexpr Float32 CONSOLE_MIN_W = 260.0f;
constexpr Float32 CONSOLE_DEF_W = 380.0f;

constexpr Float32 SIDEBAR_MIN_W = 260.0f;   // narrower than this and rows wrap

// The Code view's file tree. LOGICAL pixels, like the sidebar, so a drag feels
// the same at 100% and at 200% and the stored value survives a DPI change.
constexpr Float32 CODE_TREE_MIN_W = 130.0f;   // below this the file names clip
constexpr Float32 CODE_TREE_MAX_W = 640.0f;
constexpr Float32 CODE_TREE_DEF_W = 240.0f;

Float32 codeTreeLogicalW  = CODE_TREE_DEF_W;
Bool    codeTreeCollapsed = false;

// The reference library's browsing state - which page, the drawer width, the
// zoom. Held here rather than inside the module so the panel stays re-entrant.
ref::State  refView;

// Which view the bottom control bar belongs to. The same thing as the open tab
// now that tabs are the only layout, and kept as its own name because the bar
// asks "which view am I configuring" rather than "which tab is open".
Int32 wsFocused = 0;

Bool  panelLayoutDirty = false;             // written out at the end of a frame

Void loadPanelLayout()
{
    const Str txt = settings::read("panels.txt");
    if(txt.empty())
        return;

    // "w <px>" and "s <id> <floating>", one per line. Hand-written so a broken
    // file costs a default layout rather than a parser.
    Size i = 0;
    Int32 seen = 0;
    Int32 order[SECTION_COUNT] = {};

    while(i < txt.size())
    {
        const Size e = txt.find('\n', i);
        const Str line = txt.substr(i, (e == Str::npos) ? Str::npos : e - i);
        i = (e == Str::npos) ? txt.size() : e + 1;

        if(line.size() > 2 && line[0] == 'w')
        {
            const Float64 v = std::atof(line.c_str() + 1);
            if(v >= SIDEBAR_MIN_W && v <= 1600.0)
                sidebarLogicalW = static_cast<Float32>(v);
        }
        else if(line.size() > 2 && line[0] == 'k')
        {
            Float64 v = 0.0;
            Int32   o = 0;
            if(std::sscanf(line.c_str() + 1, "%lf %d", &v, &o) == 2)
            {
                if(v >= CONSOLE_MIN_W && v <= 1600.0)
                    consoleLogicalW = static_cast<Float32>(v);
                consoleOpen = (o != 0);
            }
        }
        else if(line.size() > 2 && line[0] == 't')
        {
            Float64 v = 0.0;
            Int32   c = 0;
            if(std::sscanf(line.c_str() + 1, "%lf %d", &v, &c) == 2)
            {
                if(v >= CODE_TREE_MIN_W && v <= CODE_TREE_MAX_W)
                    codeTreeLogicalW = static_cast<Float32>(v);
                codeTreeCollapsed = (c != 0);
            }
        }
        else if(line.size() > 2 && line[0] == 's' && seen < SECTION_COUNT)
        {
            Int32 id = -1, fl = 0;
            if(std::sscanf(line.c_str() + 1, "%d %d", &id, &fl) == 2
               && id >= 0 && id < SECTION_COUNT)
            {
                order[seen++] = id;
                sectionFloating[id] = (fl != 0);
            }
        }
    }

    // Only accept an order that is a genuine permutation. A partial or repeated
    // one would silently drop a section off the screen with no way back.
    if(seen == SECTION_COUNT)
    {
        Bool used[SECTION_COUNT] = {};
        Bool ok = true;
        for(Int32 k = 0; k < SECTION_COUNT; ++k)
        {
            if(used[order[k]])
            {
                ok = false;
                break;
            }
            used[order[k]] = true;
        }
        if(ok)
            for(Int32 k = 0; k < SECTION_COUNT; ++k)
                sectionOrder[k] = order[k];
    }
}

Void savePanelLayout()
{
    // Wide enough for the longest line here, which is a panel record.
    Char buf[160];
    Str out;
    std::snprintf(buf, sizeof(buf), "w %.0f\n", static_cast<Float64>(sidebarLogicalW));
    out += buf;
    std::snprintf(buf, sizeof(buf), "t %.0f %d\n",
                  static_cast<Float64>(codeTreeLogicalW),
                  codeTreeCollapsed ? 1 : 0);
    out += buf;
    std::snprintf(buf, sizeof(buf), "k %.0f %d\n",
                  static_cast<Float64>(consoleLogicalW),
                  consoleOpen ? 1 : 0);
    out += buf;

    for(Int32 k = 0; k < SECTION_COUNT; ++k)
    {
        std::snprintf(buf, sizeof(buf), "s %d %d\n",
                      sectionOrder[k], sectionFloating[sectionOrder[k]] ? 1 : 0);
        out += buf;
    }
    settings::write("panels.txt", out);
}

// --tab <name> opens one section at startup, for screenshots and for launching
// straight into the thing you care about. The lidar sub-tab names still work and
// open Sensors with that readout showing.
Int32 forceSection    = -1;
Int32 forceSub        = -1;
Int32 forceTabFrames = 0;   // a tab bar only honours SetSelected once laid out

// Map layers. Only the RPLIDAR exists today; the rest are declared so that
// wiring one later is filling in a row rather than redesigning the screen.
Bool layerLidar     = true;
Bool layerLidarPrev = true;
Int32  selSensor      = 0;    // which sensor the telemetry sub-tabs describe

// ---------------------------------------------------------------------------
// The recorder.
//
// A second RadarView, so the recorder's trail, mode and accumulated map are its
// own - scrubbing a recording must not disturb the live map you were watching,
// and the live map must not scribble over a recording you are studying.
// ---------------------------------------------------------------------------
RadarView recView;
rec::Recording recording;

// ---- the Code view ---------------------------------------------------------
// A sketch is edited here, written to firmware/scratch/sketch.c, and built and
// flashed by the SAME scripts the Firmware panel uses. There is one toolchain
// path in this project and this is a front-end to it.
ed::Editor   codeEditor;
ui::CodeView codeView;
Str          codePath;        // absolute path of the open file, or empty
Str          codeName;        // its display name
Str          codeMessage;     // last save/build note, shown on the toolbar
Bool         codeLoaded = false;

// ---- IDE state -------------------------------------------------------------
// Diagnostics from the last build, for the file on screen. Rebuilt when a build
// finishes; NOT cleared when you type, because a stale mark that says where the
// error was is more use than no mark at all until you ask again.
Vec<diag::Item> codeDiags;

// Last-write time of the open file, for noticing an edit made outside the app.
// Zero means "not watching" - a file we have never successfully stat'd.
UInt64 codeFileStamp = 0;
Int32  codeWatchIn   = 0;      // frames until the next stat

// Autosave. Counted in frames from the last edit rather than on a wall clock,
// so it fires a moment after you STOP typing rather than in the middle of a
// word.
Bool  codeAutosave     = true;
Int32 codeAutosaveIn   = 0;

// A file the tree wants to act on, resolved after the menu closes - deleting an
// entry while iterating the list that drew it is how a tree crashes.
Str codePendingDelete;

// Build & Flash is TWO operations, and which one is in flight decides what a
// failure means. One boolean could say "a flash is queued" but not "the flash
// itself just failed", which is why a failed flash used to leave the Code view
// still saying "flashing" while the status bar said OP FAILED and neither said
// why.
enum class CodeOp
{
    CODE_OP_NONE = 0,
    CODE_OP_BUILDING,
    CODE_OP_FLASHING,
};

// Set when an operation made us drop the Pico link, so it can be restored when
// the board comes back. Counts down in frames because the board re-enumerates a
// second or two AFTER the flash script exits.
Bool  picoRelinkWanted = false;
Int32 picoRelinkIn     = 0;
Str   picoRelinkPort;

CodeOp codeOp         = CodeOp::CODE_OP_NONE;
Str    codeFlashTarget = "sketch";

Bool    recArmed   = false;   // capturing
Bool    recPlaying = false;
Float64 recStartS  = 0.0;     // clock at the moment recording began
Float64 recPlayS   = 0.0;     // playback position, seconds into the recording
Size    recIndex   = 0;       // revolution currently shown
Str     recStatus;            // last save/load result, shown to the user
Bool    recStatusBad = false;

// Set when the scrub moves, so the pump re-renders that frame once even though
// playback is paused. Without it, dragging the scrub while paused changes the
// index and nothing on screen moves.
Bool    recPendingSeek = false;

// Files on disk, refreshed on demand rather than every frame - a directory
// listing per frame is a syscall per frame for a list that changes when the
// user asks it to.
Vec<Str> recFiles;
Int32            recFileIndex = 0;

Void refreshRecordings()
{
    recFiles = rec::list();
    if(recFileIndex >= static_cast<Int32>(recFiles.size()))
        recFileIndex = 0;
}

// Which central view is on screen. 0 is the flat map, 1 is the 3D scene,
// 2 is the recorder, and 3 + boardIndex is a board.
//
// The dimension is a TAB, not a control inside the map, because 2D and 3D are
// two viewers of the same data with different overlays - the same relationship
// the board view already has to the map. It was a segmented switch on the
// overlay strip first, which put the thing that decides which overlays exist
// inside the strip of overlays it decides.
//
// Persisted across frames because the bottom bar belongs to the VIEW, and the
// layout has to reserve that bar's height before the tab bar has had a chance to
// tell us which tab is selected. Switching tabs therefore sizes the bar from the
// outgoing view for exactly one frame, which at these frame rates is invisible.
Int32 centralView = 0;

// ---------------------------------------------------------------------------
// Vehicle lighting, driven by hand.
//
// NOTHING IS WIRED. No LED exists, the board has no lighting firmware, and this
// switch reaches exactly as far as the 3D view. It is here so the rules in
// docs/conventions.md can be watched running - particularly the indicator-overrides-brake
// asymmetry, which is the one that has to be seen to be believed - before there
// is hardware to get it wrong on.
lights::Input& lightInput = radarView.lighting;

// Automatic lighting: the lamps worked out from what the car is doing rather
// than from the buttons below them.
//
// Off by default, because the bench panel exists to exercise one lamp at a time
// and an automatic mode that overwrites your choice every frame would make that
// impossible. On, the buttons go read-only and show what was decided.
Bool              autoLights = false;
lights::AutoState autoLightState;

// Rolling rotation-rate history for the sparkline.
constexpr Int32 HISTORY = 240;
Float32 hzHist[HISTORY] = {};
Int32   hzCount = 0;

// The C1M1 datasheet rev 1.1 lists exactly ONE rate: Figure 2-1 gives
// "Communication Speed 460800" and Figure 2-8 gives 460800 with no minimum and
// no maximum. There is no 115200 mode and no 256000 mode to fall back to.
//
// Those two were offered here for years, defaulted past, and could only ever
// have produced a connection that opens and then returns nothing - which is
// among the worst failures to hand somebody, because the port is open and the
// device is silent and neither of those looks like a wrong setting.
//
// The others are kept, disabled, rather than deleted: somebody who has read
// about an A1 at 115200 will come looking for it, and a greyed entry that says
// why is a better answer than an empty list that looks like a missing feature.
struct BaudOpt
{
    Int32       rate;
    const Char* label;
    Bool        supported;
};

const BaudOpt BAUDS[] = {
    { 460800, "460800", true  },
    { 115200, "115200", false },
    { 256000, "256000", false },
};
constexpr Int32 BAUD_COUNT = static_cast<Int32>(sizeof(BAUDS) / sizeof(BAUDS[0]));

struct RangeOpt { const Char* label; Float32 mm; };   // mm <= 0 means auto-fit
const RangeOpt RANGES[] = {
    { "Fit", 0.0f }, { "0.5 m", 500.0f }, { "1 m", 1000.0f }, { "2 m", 2000.0f },
    { "4 m", 4000.0f }, { "8 m", 8000.0f }, { "12 m", 12000.0f },
};
constexpr Int32 RANGE_COUNT = static_cast<Int32>((sizeof(RANGES) / sizeof(RANGES[0])));
const Char* RANGE_ITEMS[RANGE_COUNT] = {};

// PushFont in 1.92 takes a pre-scale base size; LegacySize already has the DPI
// baked in by LoadFonts, so it is never multiplied by uiDpiScale again.
struct ScopedFont
{
    explicit ScopedFont(ImFont* f)
    {
        ImGui::PushFont(f, f ? f->LegacySize : 0.0f);
    }
    ~ScopedFont()
    {
        ImGui::PopFont();
    }
};

Void refreshPorts()
{
    // Captured BEFORE the list is replaced, and restored by NAME rather than by
    // index. The enumeration reorders and shrinks as things are plugged and
    // unplugged, so an index that meant COM7 a second ago can mean COM3 now -
    // and silently retargeting somebody's Connect button at a different device
    // is the worst outcome available here.
    const Str wasSelected =
        (portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
            ? lidarPorts[static_cast<Size>(portIndex)]
            : Str();

    lidarPorts = LidarSource::listPorts();

    // A port that vanished while it was selected STAYS in the list. After an
    // unplug the combo should still read COM7: that is the thing you are going
    // to reconnect to when the cable goes back in, and dropping it makes the
    // app look like it has forgotten what it was talking to.
    if(!wasSelected.empty())
    {
        Bool present = false;
        for(const Str& p : lidarPorts)
        {
            if(_stricmp(p.c_str(), wasSelected.c_str()) == 0)
            {
                present = true;
                break;
            }
        }
        if(!present)
        {
            lidarPorts.push_back(wasSelected);
        }
    }

    portItems.clear();
    for(const auto& s : lidarPorts) portItems.push_back(s.c_str());

    if(lidarPorts.empty())
    {
        portIndex = -1;
        return;
    }

    // Whatever was chosen stays chosen, present or not.
    if(!wasSelected.empty())
    {
        for(Int32 i = 0; i < static_cast<Int32>(lidarPorts.size()); ++i)
        {
            if(_stricmp(lidarPorts[static_cast<Size>(i)].c_str(),
                        wasSelected.c_str()) == 0)
            {
                portIndex = i;
                return;
            }
        }
    }

    // Identify the CP210x bridge outright where we can - the remaining ports on
    // a typical machine are Bluetooth links, and connecting to one of those just
    // produces a confusing timeout.
    const Str preferred = LidarSource::preferredPort();
    if(!preferred.empty())
    {
        for(Int32 i = 0; i < static_cast<Int32>(lidarPorts.size()); ++i)
        {
            if(_stricmp(lidarPorts[i].c_str(), preferred.c_str()) == 0)
            {
                portIndex = i;
                return;
            }
        }
    }

    // ---- and if it cannot be identified, SELECT NOTHING --------------------
    //
    // There used to be a fallback here: "a USB bridge normally enumerates above
    // the built-in ports", so pick the highest number. It is a reasonable guess
    // and it caused a genuinely confusing failure.
    //
    // Serial ports are EXCLUSIVE. With the lidar unplugged there was no CP210x
    // to find, so the guess picked the highest port on the machine - which was
    // COM10, the Pico. The lidar then opened the Pico's port, the Pico could
    // not, and the app reported an error about a board that was sitting there
    // working perfectly. Two subsystems, one port, and the one that was RIGHT
    // lost it to the one that was guessing.
    //
    // So: no guess. If nothing here is a CP210x, nothing is selected, Connect
    // is disabled and the panel says why. "I do not know which port your lidar
    // is on" is a true and useful thing to say; picking one at random and
    // failing on it is neither.
    //
    // A Bluetooth port would have been the other bad outcome - the machine has
    // six - and connecting to one produces a timeout that looks like a broken
    // lidar rather than like a wrong port.
    portIndex = -1;
}

// True when a port could belong to the lidar. Used to grey out the ones that
// certainly cannot, rather than hiding them - a port that is missing from the
// list looks like a driver problem, and a port that is visible and disabled
// explains itself.
Bool portCouldBeLidar(const Str& port)
{
    return dev::couldBeLidar(dev::portKind(port));
}

Bool isBusy()
{
    const LidarState s = lidarSource.state();
    return s == LidarState::LIDAR_STATE_SCANNING || s == LidarState::LIDAR_STATE_CONNECTING;
}

// ---------------------------------------------------------------- pico ----

// Set by the window procedure when Windows says the device tree changed, and
// consumed by pumpDeviceScan() below.
Atomic<Bool> deviceChangePending{false};

// Frames until the rescan runs. The notification arrives BEFORE the COM port
// exists - it is about the device NODE, and the serial driver registers its
// port a moment later - so rescanning immediately finds nothing and the app
// concludes the board is still absent. Which is the bug, arriving by a
// slightly different road.
Int32 deviceScanIn = 0;

// Backstop, in frames. A notification that never arrives - or arrives while the
// window is not pumping messages - would otherwise leave the app permanently
// convinced nothing is attached, and that is precisely the failure this exists
// to fix. It should not be reachable by a different route.
Int32 deviceScanIdle = 0;

// Set when the user presses Disconnect, cleared when they press Connect. A
// device reappearing should reconnect, EXCEPT when they deliberately let go of
// it: an app that grabs the port straight back makes Disconnect useless.
Bool picoUserDisconnected  = false;
Bool lidarUserDisconnected = false;

Void refreshPicoPorts()
{
    picoPorts = PicoLink::listPicoPorts();

    picoItems.clear();
    for(const auto& s : picoPorts) picoItems.push_back(s.c_str());

    if(picoPorts.empty())
    {
        picoIndex = -1;
        return;
    }
    if(picoIndex < 0 || picoIndex >= static_cast<Int32>(picoPorts.size())) picoIndex = 0;
}

Void connectPico()
{
    picoUserDisconnected = false;

    LOG_INFO("pico", "connect requested: port=%s",
             (picoIndex >= 0 && picoIndex < static_cast<Int32>(picoPorts.size()))
                 ? picoPorts[picoIndex].c_str() : "(none)");
    if(picoIndex < 0 || picoIndex >= static_cast<Int32>(picoPorts.size())) return;
    resetBoardStatus();
    picoLink.connect(picoPorts[picoIndex]);
}

Void sendPico(const Char* line)
{
    if(!line || !line[0]) return;
    picoLink.send(line);            // the link logs it; drain() gives it back to us
}

// The same thing, for traffic the hub generates on a timer rather than because
// somebody asked. Marked so the console can hide it - see PicoLine::poll.
//
// A separate function rather than a defaulted argument, so that every polling
// call site READS as polling at the point of call. The distinction is easy to
// forget and the cost of forgetting is a console nobody can use.
Void pollPico(const Char* line)
{
    if(!line || !line[0]) return;
    picoLink.send(line, true);
}

Str trimLine(const Str& s)
{
    Size a = 0, b = s.size();
    while(a < b && static_cast<UInt8>(s[a]) <= ' ') ++a;
    while(b > a && static_cast<UInt8>(s[b - 1]) <= ' ') --b;
    return s.substr(a, b - a);
}

// Reads meaning out of a line without assuming which firmware produced it.
//
// The test is the SHAPE of the reply to `?`, not a particular error string. On
// the bench, pico_debug answers `?` with its HELP listing rather than the
// documented "ERR bad command" - so keying off "ERR" would have silently
// reported nothing at all. Anything that is not an S line means this firmware
// does not report servo and ESC state, whatever it chose to say instead.
Void observeLine(const PicoLine& ln)
{
    const Str t = trimLine(ln.text);
    if(t.empty()) return;

    if(ln.outgoing)
    {
        lastCmd = t;
        if(t == "?")      vehAwait = true;
        if(t == "STATUS") dbgAwait = true;
        return;
    }

    // ---- pico_debug ------------------------------------------------------
    // "INFO status up_ms=... led=on blink_hz=2.00 lamp=gpio25 lamp_up=yes"
    // "INFO id board=pico2 sdk=... uid=... lamp=gpio25 lamp_up=yes"
    //
    // cyw43= is present ONLY on a board that has the chip, so its absence is
    // not a failure to report - it is a plain Pico 2 correctly declining to
    // mention a peripheral it does not have.
    // Scanned for the fields rather than parsed positionally: the two lines
    // carry an overlapping subset and the order is the firmware's business.
    if(t.compare(0, 5, "INFO ") == 0)
    {
        const Char* s = t.c_str();

        // A bare word terminated by space or end of line.
        const auto word = [](const Char* q) -> Str
        {
            Size n = 0;
            while(q[n] != '\0' && q[n] != ' ' && q[n] != '\r' && q[n] != '\n')
            {
                ++n;
            }
            return Str(q, n);
        };

        if(const Char* q = std::strstr(s, "board="))
        {
            debugStatus.boardName = word(q + 6);
        }

        dbgUnsupported = false;
        dbgAwait       = false;
        return;
    }

    // "OK drive servo=1500 servo_t=1500 esc=1500 esc_t=1500 armed=0 ..."
    //
    // Read by NAME, like the sensor line, so a field added later is ignored
    // rather than shifting everything after it.
    if(t.compare(0, 9, "OK drive ") == 0)
    {
        const Char* p = t.c_str();
        const auto  field = [p](const Char* key, Int32& out)
        {
            if(const Char* q = std::strstr(p, key))
            {
                out = std::atoi(q + std::strlen(key));
            }
        };

        field("servo=",     driveServo);
        field("servo_t=",   driveServoT);
        field("esc=",       driveEsc);
        field("esc_t=",     driveEscT);
        field("servo_min=", driveServoMin);
        field("servo_max=", driveServoMax);
        field("esc_min=",   driveEscMin);
        field("esc_max=",   driveEscMax);

        Int32 armed = 0;
        field("armed=", armed);
        driveArmed = (armed != 0);

        Int32 on = 0;
        field("servo_on=", on);
        driveServoOn = (on != 0);

        field("servo_c=", driveServoC);
        field("slew=", driveSlew);
        if(!driveSlewHeld)
        {
            driveSlewWant = driveSlew;
        }

        Int32 milli = 0;
        field("steer_m=", milli);
        driveSteer = static_cast<Float32>(milli) / 1000.0f;

        // Absent from a board running firmware older than this field; the
        // default of 0 then means the drawing shows straight-ahead, which is
        // the safe thing to show when nobody has said otherwise.
        Int32 milliNow = 0;
        field("steer_now=", milliNow);
        driveSteerNow = static_cast<Float32>(milliNow) / 1000.0f;
        if(!driveSteerHeld)
        {
            driveSteerWant = driveSteer;
        }

        // Each slider follows the board unless that slider is being dragged.
        if(!driveServoHeld)
        {
            driveServoWant = driveServoT;
        }
        if(!driveEscHeld)
        {
            driveEscWant = driveEscT;
        }

        driveKnown = true;
        return;
    }

    // "OK lights on=1 turn=off forced=no levels=0,0,255,255,0,0,0,0
    //             pins=-1,-1,15,13,-1,-1,-1,-1"
    if(t.compare(0, 10, "OK lights ") == 0)
    {
        const Char* p = t.c_str();
        if(const Char* q = std::strstr(p, "turn="))
        {
            boardTurn = (std::strncmp(q + 5, "left", 4) == 0)  ? -1
                      : (std::strncmp(q + 5, "right", 5) == 0) ?  1
                                                               :  0;
        }
        if(const Char* q = std::strstr(p, "on="))
        {
            boardLightsOn = (std::atoi(q + 3) != 0);
        }

        // Two comma lists, read the same way. Short lists leave the tail of the
        // array alone rather than zeroing it: a board running older firmware
        // says less, and saying less should not read as "every lamp is dark".
        const auto commas = [](const Char* q, Int32* out, Int32 n)
        {
            if(q == nullptr) return;
            for(Int32 i = 0; i < n && *q != '\0' && *q != ' '; ++i)
            {
                out[i] = std::atoi(q);
                const Char* c = std::strchr(q, ',');
                if(c == nullptr || *(c + 1) == '\0') break;
                q = c + 1;
            }
        };

        if(const Char* q = std::strstr(p, "off_us="))
        {
            boardLightsOff = std::atoi(q + 7);
            if(!lightsOffHeld)
            {
                lightsOffWant = boardLightsOff;
            }
        }

        if(const Char* q = std::strstr(p, "levels="))
        {
            commas(q + 7, boardLamp, LAMP_N);
        }
        if(const Char* q = std::strstr(p, "pins="))
        {
            commas(q + 5, boardLampPin, LAMP_N);
        }
        return;
    }

    if(t.compare(0, 7, "OK stop") == 0)
    {
        driveArmed     = false;
        driveServoWant = 1500;
        driveEscWant   = 1500;
        LOG_INFO("pico", "stop acknowledged");
        return;
    }

    // "OK sensors i2c=1 tof=1 tof_addr=0x29" - what the board found at boot.
    //
    // Read as key=value pairs rather than by position, so a sensor added to the
    // firmware later is ignored by an older hub instead of breaking the parse.
    if(t.compare(0, 11, "OK sensors ") == 0)
    {
        const Char* p = t.c_str();
        if(const Char* q = std::strstr(p, "i2c="))
        {
            sensorI2c = (std::atoi(q + 4) != 0);
        }
        if(const Char* q = std::strstr(p, "tof="))
        {
            sensorTof = (std::atoi(q + 4) != 0);
        }
        sensorsAsked = true;

        LOG_INFO("pico", "sensors: i2c=%d tof=%d",
                 sensorI2c ? 1 : 0, sensorTof ? 1 : 0);
        return;
    }

    // "OK tof <mm> <status>", or "OK tof busy" when the measurement is not
    // finished. Busy is NOT an error: the sensor takes tens of milliseconds and
    // the hub is entitled to ask more often than that.
    if(t.compare(0, 7, "OK tof ") == 0)
    {
        const Char* a = t.c_str() + 7;
        if(std::strncmp(a, "busy", 4) == 0)
        {
            return;
        }

        // Signal and ambient are optional: an older firmware sends two fields
        // and a newer one sends four, and reading however many arrived means
        // the hub works with both rather than refusing the older one.
        Int32 mm  = 0;
        Int32 st  = 0;
        Int32 sig = -1;
        Int32 amb = -1;
        const Int32 got = std::sscanf(a, "%d %d %d %d", &mm, &st, &sig, &amb);
        if(got >= 2)
        {
            tofSignal  = (got >= 3) ? sig : -1;
            tofAmbient = (got >= 4) ? amb : -1;
            tofMm        = mm;
            tofStatus    = st;
            tofLastReply = ImGui::GetTime();
            ++tofReplies;

            // Only a GOOD reading enters the history and the extremes. A bad
            // one plotted as a number would draw a cliff that never happened.
            if(st == 0)
            {
                tofHistory[tofHistoryAt] = static_cast<Float32>(mm);
                tofHistoryAt = (tofHistoryAt + 1) % TOF_HISTORY;
                if(tofHistoryAt == 0)
                {
                    tofHistoryWrapped = true;
                }

                if(tofSeenMax == 0 || mm < tofSeenMin)
                {
                    tofSeenMin = mm;
                }
                if(mm > tofSeenMax)
                {
                    tofSeenMax = mm;
                }
            }
        }
        return;
    }

    if(t.compare(0, 12, "ERR tof abse") == 0)
    {
        sensorTof    = false;
        sensorsAsked = true;
        return;
    }

    // "OK led on" / "OK led off" / "OK led blink 2.00".
    //
    // Nothing displays the lamp's state any more - the board drawing that did
    // is gone - so this stores nothing. It still has to RECOGNISE the line and
    // return, though: dbgAwait is set while a STATUS is outstanding, and
    // anything unrecognised arriving in that window is read as "this firmware
    // has no STATUS command" and stops the polling permanently. An LED command
    // sent by hand while a poll was in flight would do exactly that.
    if(t.compare(0, 7, "OK led ") == 0)
    {
        return;
    }

    if(dbgAwait)
    {
        // Anything else in reply to STATUS means this firmware has no such
        // command. Stop asking.
        dbgUnsupported = true;
        dbgAwait       = false;
    }

    UInt64 up = 0, last = 0;
    long a = 0, b = 0;
    Int32  servo = 0, esc = 0;

    // sscanf_s rather than sscanf: no %s or %c here, so it needs no extra size
    // arguments, and it keeps this file warning-clean at /W4 without relying on
    // _CRT_SECURE_NO_WARNINGS being defined by the build.
    if(sscanf_s(t.c_str(), "S %llu %ld %ld %d %d %llu",
                 &up, &a, &b, &servo, &esc, &last) == 6)
    {
        vehicleStatus.have      = true;
        vehicleStatus.seenAt   = ImGui::GetTime();
        vehicleStatus.uptimeMs = up;
        vehicleStatus.a         = a;
        vehicleStatus.b         = b;
        vehicleStatus.servoUs  = servo;
        vehicleStatus.escUs    = esc;
        vehicleStatus.lastMs   = last;
        vehUnsupported = false;
        vehAwait       = false;
        return;
    }

    if(vehAwait)
    {
        vehUnsupported = true;
        vehAwait       = false;
    }
}

// Drains once per frame, which is what PicoLink asks for, and keeps the log
// bounded.
Void pumpPico()
{
    const Size before = picoLog.size();
    picoLink.drain(picoLog);

    // A reply inherits the flag from the request it answers.
    //
    // The link cannot know this - it sees bytes, not a protocol - and it is the
    // whole difference between hiding the chatter and hiding half of it. The
    // board answers in order, so the last thing SENT is what any arriving line
    // is a reply to. That also means a STATUS somebody types by hand shows its
    // answer normally, which a content match on "INFO status" could never do.
    for(Size i = before; i < picoLog.size(); ++i)
    {
        if(picoLog[i].outgoing)
        {
            lastSendWasPoll = picoLog[i].poll;
        }
        else
        {
            picoLog[i].poll = lastSendWasPoll;
        }
    }

    for(Size i = before; i < picoLog.size(); ++i) observeLine(picoLog[i]);

    if(picoLog.size() > LOG_MAX)
        picoLog.erase(picoLog.begin(), picoLog.begin() + (picoLog.size() - LOG_MAX));
}

// Asks the board what it is doing, at most every couple of seconds.
//
// This used to run only while the board view was on screen, on the grounds that
// polling a link nobody is looking at is traffic nobody asked for. That view is
// gone, and the argument went with it: the System panel in the sidebar is
// ALWAYS visible and shows what the reply carries - the board name, whether the
// firmware is running - so there is now always somebody looking.
Void pollBoardStatus()
{
    if(dbgUnsupported) return;
    if(picoLink.state() != PicoState::PICO_STATE_CONNECTED) return;

    const Float64 now = ImGui::GetTime();
    if(dbgLastPoll > 0.0 && (now - dbgLastPoll) < 2.0) return;

    dbgLastPoll = now;
    pollPico("STATUS");
}

// Asks the board what its indicator lamps are doing.
//
// Fast, because the point is to WATCH it blink: at 1.5 Hz the lamp changes
// every 267 ms, so a two-second poll would show a still frame of a flashing
// light. 120 ms is comfortably inside the shorter half-cycle.
//
// TEMPORARY, with the rest of the indicator scaffolding.
Void pollLights()
{
    if(dbgUnsupported) return;
    if(picoLink.state() != PicoState::PICO_STATE_CONNECTED) return;

    const Float64 now = ImGui::GetTime();
    if(lightsLastPoll > 0.0 && (now - lightsLastPoll) < 0.12) return;

    lightsLastPoll = now;
    pollPico("LIGHTS");
}

// Asks the board what is attached, once per connection.
//
// Separate from the range poll because the answer does not change while the
// board is running - a sensor cannot be plugged into a Pico that is already
// powered without it being reset - so asking repeatedly would be noise on a
// link that is also carrying the readings.
Void pollSensorList()
{
    if(dbgUnsupported || sensorsAsked)
    {
        return;
    }
    if(picoLink.state() != PicoState::PICO_STATE_CONNECTED)
    {
        return;
    }
    pollPico("SENSORS");
}

// Asks for a range reading, at a rate the sensor can actually sustain.
//
// Only while the Range view is on screen. Polling a sensor nobody is looking at
// fills the console log and the link with traffic to no purpose, and this link
// is also how firmware gets flashed.
Void pollTof(Bool wanted)
{
    if(!wanted || !sensorTof || dbgUnsupported)
    {
        return;
    }
    if(picoLink.state() != PicoState::PICO_STATE_CONNECTED)
    {
        return;
    }

    // ~10 Hz. The sensor's own measurement is slower than this, so asking
    // faster would only be told "busy" more often.
    static Float64 lastAsk = 0.0;
    const Float64  now     = ImGui::GetTime();
    if(now - lastAsk < 0.1)
    {
        return;
    }
    lastAsk = now;
    pollPico("TOF");
}

const Char* picoStateText(PicoState s)
{
    switch(s)
    {
    case PicoState::PICO_STATE_CONNECTING: return "Connecting";
    case PicoState::PICO_STATE_CONNECTED:  return "Connected";
    case PicoState::PICO_STATE_ERROR:      return "Error";

    // A Pico that rebooted into BOOTSEL drops its CDC port BY DESIGN, and that
    // happens on every single flash. Calling it an error made the normal path
    // through this app look like a failure.
    case PicoState::PICO_STATE_UNPLUGGED:  return "Not connected";
    default:                    return "Not connected";
    }
}

ImU32 picoStateColor(PicoState s)
{
    switch(s)
    {
    case PicoState::PICO_STATE_CONNECTING: return ui::sem::WARN;
    case PicoState::PICO_STATE_CONNECTED:  return ui::sem::GOOD;
    case PicoState::PICO_STATE_ERROR:      return ui::sem::BAD;
    case PicoState::PICO_STATE_UNPLUGGED:  return ui::sem::MUTED;
    default:                    return ui::sem::MUTED;
    }
}

const Char* lidarStateText()
{
    switch(lidarSource.state())
    {
    case LidarState::LIDAR_STATE_CONNECTING: return "Connecting";
    case LidarState::LIDAR_STATE_SCANNING:   return "Scanning";
    case LidarState::LIDAR_STATE_ERROR:      return "Error";

    // Spelled out rather than left to the default, because it is a DECISION:
    // a device somebody unplugged reads as not connected, which is what it is.
    // "Error" is reserved for the cases where something is actually wrong.
    case LidarState::LIDAR_STATE_UNPLUGGED:  return "Not connected";

    default:
        // "Not connected" would be a lie while the port is open and the motor
        // is simply parked. Those are different situations and the operator
        // needs to be able to tell them apart at a glance.
        return lidarSource.connected() ? "Motor off" : "Not connected";
    }
}

// The same state in two colours, because it is printed on two grounds: the
// status strip is light chrome, the map HUD is the dark viewport. See the note
// on the two palettes in theme.hxx.
ImU32 lidarStateColor()
{
    switch(lidarSource.state())
    {
    case LidarState::LIDAR_STATE_CONNECTING: return ui::sem::WARN;
    case LidarState::LIDAR_STATE_SCANNING:   return ui::sem::GOOD;
    case LidarState::LIDAR_STATE_ERROR:      return ui::sem::BAD;
    case LidarState::LIDAR_STATE_UNPLUGGED:  return ui::sem::MUTED;
    default:
        // Connected but not spinning is a deliberate state, not an absence.
        return lidarSource.connected() ? ui::sem::WARN : ui::sem::MUTED;
    }
}

ImU32 lidarStateColorOnViewport()
{
    switch(lidarSource.state())
    {
    case LidarState::LIDAR_STATE_CONNECTING: return ui::plot::WARN;
    case LidarState::LIDAR_STATE_SCANNING:   return ui::plot::OK;
    case LidarState::LIDAR_STATE_ERROR:      return ui::plot::BAD;
    case LidarState::LIDAR_STATE_UNPLUGGED:  return ui::plot::IDLE;
    default:
        return lidarSource.connected() ? ui::plot::WARN : ui::plot::IDLE;
    }
}

// A silent board is the expected state right now - the flashed firmware only
// speaks when spoken to - so this says so in words rather than showing an empty
// readout.
Void picoAgeText(Char* buf, Size n, Float64 ageS)
{
    if(ageS < 0.0)        std::snprintf(buf, n, "--");
    else if(ageS < 600.0) std::snprintf(buf, n, "%.1f s ago", ageS);
    else                    std::snprintf(buf, n, "%.0f min ago", ageS / 60.0);
}

Bool logMatches(const PicoLine& ln)
{
    if(filterBuf[0] == '\0') return true;

    const Char* hay = ln.text.c_str();
    for(; *hay; ++hay)
    {
        const Char* h = hay;
        const Char* n = filterBuf;
        while(*n && *h &&
               std::tolower(static_cast<UInt8>(*h)) == std::tolower(static_cast<UInt8>(*n)))
               {
                   ++h;
                   ++n;
               }
        if(*n == '\0') return true;
    }
    return false;
}

Void recomputeDerived()
{
    pointsPs = latestFrame.hz * static_cast<Float32>(latestFrame.points.size());

    Float32 sectorMm[SECTORS] = {};
    for(Int32 i = 0; i < QUALITY_BUCKETS; ++i) qHist[i] = 0.0f;
    for(Int32 i = 0; i < DIST_BUCKETS; ++i)    distHist[i] = 0.0f;

    static Bool binSeen[360];
    std::memset(binSeen, 0, sizeof(binSeen));

    Float64 sum   = 0.0;
    Float64 qSum = 0.0;
    Int32    n     = 0;
    Float32  maxMm = 0.0f;
    Int32    qLo = 255, qHi = 0;

    nNoreturn = nToonear = nToofar = 0;

    for(const LidarPoint& p : latestFrame.points)
    {
        if(p.distMm <= 0.0f)
        {
            ++nNoreturn;
            continue;
        }
        if(p.distMm < MIN_VALID_MM)
        {
            ++nToonear;
            continue;
        }
        if(p.distMm > MAX_VALID_MM)
        {
            ++nToofar;
            continue;
        }

        sum += p.distMm;
        ++n;
        if(p.distMm > maxMm) maxMm = p.distMm;

        qSum += p.quality;
        if(p.quality < qLo) qLo = p.quality;
        if(p.quality > qHi) qHi = p.quality;

        Int32 qb = static_cast<Int32>(p.quality) * QUALITY_BUCKETS / 64;
        qb = std::min(std::max(qb, 0), QUALITY_BUCKETS - 1);
        qHist[qb] += 1.0f;

        Int32 db = static_cast<Int32>((p.distMm / (MAX_VALID_MM / DIST_BUCKETS)));
        db = std::min(std::max(db, 0), DIST_BUCKETS - 1);
        distHist[db] += 1.0f;

        Int32 ab = static_cast<Int32>(p.angleDeg);
        if(ab >= 0 && ab < 360) binSeen[ab] = true;

        Int32 s = static_cast<Int32>((p.angleDeg / (360.0f / SECTORS)));
        s = std::min(std::max(s, 0), SECTORS - 1);

        // Nearest return wins the sector - that is the obstacle that matters.
        if(sectorMm[s] == 0.0f || p.distMm < sectorMm[s])
            sectorMm[s] = p.distMm;
    }

    nInspec = n;
    meanMm  = n ? static_cast<Float32>((sum / n)) : 0.0f;
    maxRangeMm = maxMm;
    qMean   = n ? static_cast<Float32>((qSum / n)) : 0.0f;
    qMin    = n ? qLo : 0;
    qMax    = n ? qHi : 0;

    Int32 covered = 0;
    for(Int32 i = 0; i < 360; ++i) if(binSeen[i]) ++covered;
    coverageDeg = covered / 360.0f;

    qHistMax = 1.0f;
    for(Int32 i = 0; i < QUALITY_BUCKETS; ++i) qHistMax = std::max(qHistMax, qHist[i]);
    distHistMax = 1.0f;
    for(Int32 i = 0; i < DIST_BUCKETS; ++i) distHistMax = std::max(distHistMax, distHist[i]);

    // Capped, not scaled to the maximum: one open doorway at 8 m would
    // otherwise crush every near-field bar to invisibility.
    for(Int32 i = 0; i < SECTORS; ++i)
        sectorM[i] = std::min(sectorMm[i] / 1000.0f, CLEARANCE_CAP_M);

    if(hzCount < HISTORY)
    {
        hzHist[hzCount++] = latestFrame.hz;
    }
    else
    {
        std::memmove(hzHist, hzHist + 1, sizeof(Float32) * (HISTORY - 1));
        hzHist[HISTORY - 1] = latestFrame.hz;
    }
}

// ----------------------------------------------------------------- flash ---

// vendor/ is where the existing backup lives, so restores are all in one place.
// The date is in the name because the only thing you ever want to know about a
// backup is which one is newer.
Void defaultBackupName()
{
    const Str root = PicoFlash::repoRoot();

    std::time_t t = std::time(nullptr);
    std::tm     lt{};
    localtime_s(&lt, &t);

    Char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M", &lt);

    std::snprintf(backupBuf, sizeof(backupBuf), "%s\\vendor\\pico-flash-%s.uf2",
                  root.empty() ? "." : root.c_str(), stamp);
}

// The most useful line of the flash/build log: the last one the script marked
// as an error, or failing that the last line it printed at all.
//
// The scripts are disciplined about prefixing real failures with "[error]", so
// this nearly always finds the sentence a person actually needs - "no Pico
// found: ... Either plug it in, or hold BOOTSEL while connecting USB."
Str lastFlashError()
{
    for(Size i = flashLog.size(); i > 0; --i)
    {
        const Str& ln = flashLog[i - 1];
        if(ln.find("[error]") != Str::npos || ln.find("error:") != Str::npos)
        {
            // Drop the marker; the Code view's line is short and the word
            // "failed" is already in front of it.
            const Size at = ln.find("[error]");
            Str out = (at != Str::npos) ? ln.substr(at + 7) : ln;
            while(!out.empty() && (out.front() == ' ' || out.front() == '\t'))
            {
                out.erase(out.begin());
            }
            return out.empty() ? ln : out;
        }
    }

    if(!flashLog.empty())
    {
        return flashLog.back();
    }
    return "see the Firmware panel";
}

// Drops the Pico serial link before an operation that needs the port ITSELF.
//
// flash.ps1 reboots a running board by opening its port at 1200 baud - that is
// the whole mechanism that saves you reaching for the BOOTSEL button. Windows
// gives serial ports exclusively, so it cannot do that while the hub has the
// same port open: the touch fails with "Access to the port 'COM10' is denied",
// no bootloader ever appears, and the script reports
//
//     [error] RPI-RP2 never appeared.
//
// while the status bar cheerfully says PICO Connected. Those two facts together
// read as a broken board rather than as a busy port, which is what makes this
// worth an explicit release rather than a note in a README.
//
// The link is restored once the board has re-enumerated - see pumpPicoRelink().
Void releasePicoPortForBoardOp()
{
    if(picoLink.state() == PicoState::PICO_STATE_DISCONNECTED)
    {
        return;
    }

    picoRelinkPort   = picoLink.port();
    picoRelinkWanted = true;
    picoRelinkIn     = 0;      // armed by the operation finishing, not yet

    LOG_INFO("flash", "releasing %s so the 1200-baud touch can open it",
             picoRelinkPort.c_str());
    picoLink.disconnect();
}

// Reconnects after a board operation, once the port is back.
//
// Frame-counted rather than immediate: flashing reboots the board, so the port
// vanishes and returns a second or two after the script exits. Reconnecting the
// instant the operation ends just fails.
// Defined further down; the per-frame pumps below need them and sit above them.
Bool   saveSketch();

// Notices a file edited outside the app, and reloads it.
//
// Reloads SILENTLY when the buffer is clean, and refuses when it is dirty -
// throwing away edits you have not saved to take edits from elsewhere is the
// one outcome nobody wants. A dirty buffer gets told instead, and the choice is
// left where it belongs.
//
// Polled rather than watched with ReadDirectoryChangesW: one GetFileAttributesEx
// twice a second costs nothing measurable, and a directory watch would need a
// thread, a handle to close, and a story about what happens when the sketch
// library moves.
Void pumpCodeWatch()
{
    if(codePath.empty() || centralView != 3)
    {
        return;
    }

    if(--codeWatchIn > 0)
    {
        return;
    }
    codeWatchIn = 30;       // twice a second at 60 fps

    const UInt64 now = sketch::stamp(codePath);
    if(now == 0 || codeFileStamp == 0 || now == codeFileStamp)
    {
        codeFileStamp = (codeFileStamp == 0) ? now : codeFileStamp;
        return;
    }

    codeFileStamp = now;

    if(codeEditor.dirty())
    {
        codeMessage = "changed on disk - save or reload to resolve";
        ui::setNote(codeView, "changed on disk (buffer is modified)",
                    ImGui::GetTime());
        LOG_WARN("code", "%s changed on disk while the buffer was dirty",
                 codePath.c_str());
        return;
    }

    const ed::Cursor keep = codeEditor.cursor();
    codeEditor.setText(sketch::load(codePath));
    codeEditor.setCursor(keep.line, keep.col);   // stay where you were reading

    ui::setNote(codeView, "reloaded from disk", ImGui::GetTime());
    LOG_INFO("code", "reloaded %s after an external change", codePath.c_str());
}

// The last build's diagnostics, kept apart from the linter's so a rebuild
// replaces one without discarding the other. They are merged for display.
Vec<diag::Item> codeLintDiags;

// Frames until the buffer is re-linted. Counted from the last EDIT, like
// autosave, so it never fires mid-word - and re-linting on every keystroke
// would re-scan the file dozens of times while a single line is typed.
Int32 codeLintIn = 0;

// Merges the compiler's opinion with the linter's, compiler first.
//
// Order matters where both land on one line: the gutter shows the WORST
// severity, and a build error must not be hidden behind a style warning about
// the same line.
Void refreshCodeDiags()
{
    codeView.diags = codeDiags;
    codeView.diags.insert(codeView.diags.end(),
                          codeLintDiags.begin(), codeLintDiags.end());
}

// Re-checks the buffer against docs/conventions.md a moment after typing stops.
//
// WHY THIS IS SEPARATE FROM THE COMPILER. diagnostics.hxx reports what the
// build said, which is exact and only exists after a build. This reports what
// the style audit will say at commit time, which is worth knowing while the
// line is still under the cursor rather than an hour later.
//
// The rules are tools/style_audit.py's, deliberately the same set - a linter
// that disagrees with the gate either passes what the commit rejects or flags
// what the project has decided is fine, and both teach people to ignore it.
Void pumpCodeLint()
{
    if(codePath.empty())
    {
        codeLintDiags.clear();
        return;
    }

    if(--codeLintIn > 0)
    {
        return;
    }
    codeLintIn = 30;                 // ~0.5 s at 60 fps

    const Size before = codeLintDiags.size();
    codeLintDiags = lint::check(codeEditor.text(), lint::langOf(codePath));

    if(codeLintDiags.size() != before)
    {
        refreshCodeDiags();
    }
}

// Saves a few seconds after you stop typing.
//
// Counted from the last EDIT, not on a wall clock, so it never fires mid-word.
// Only ever writes a file that is already named - autosave must not invent a
// path, because a file you did not choose the name of is a file you will not
// find again.
Void pumpCodeAutosave()
{
    if(!codeAutosave || codePath.empty() || !codeEditor.dirty())
    {
        codeAutosaveIn = 180;      // ~3 s
        return;
    }

    if(--codeAutosaveIn > 0)
    {
        return;
    }
    codeAutosaveIn = 180;

    if(saveSketch())
    {
        ui::setNote(codeView, "autosaved", ImGui::GetTime());
    }
}

// Defined further down, beside the rest of the lidar controls; declared here
// because the rescan below may want to reconnect a device that just appeared.
Void connect();

// Rescans the ports after something was plugged in or unplugged.
//
// WHY THIS DID NOT EXIST, and why the absence was invisible: both port lists
// were built once at startup and then only ever refreshed by an explicit
// button, a flash, or a BOOTSEL reboot. Launch the app with nothing attached,
// plug a board in, and it was never noticed - Connect stayed greyed out
// forever with nothing on screen to suggest what to do about it.
//
// Event-driven rather than polled: the answer is almost always "nothing
// changed", and asking Windows for the port list every frame to learn that
// would be sixty registry walks a second for nothing.
Void pumpDeviceScan()
{
    // ~2 s at 60 fps. Only fires when a notification did not.
    constexpr Int32 IDLE_FRAMES  = 120;

    // ~0.4 s. Long enough for the serial driver to have registered the port,
    // short enough that plugging a board in feels immediate.
    constexpr Int32 SETTLE_FRAMES = 24;

    if(deviceChangePending.exchange(false, std::memory_order_acq_rel))
    {
        deviceScanIn = SETTLE_FRAMES;
    }

    Bool due = false;
    if(deviceScanIn > 0)
    {
        --deviceScanIn;
        due = (deviceScanIn == 0);
    }
    if(++deviceScanIdle >= IDLE_FRAMES)
    {
        deviceScanIdle = 0;
        due            = true;
    }
    if(!due)
    {
        return;
    }

    const Bool hadPico  = (picoIndex >= 0);
    const Bool hadLidar = (portIndex >= 0);

    refreshPicoPorts();
    refreshPorts();

    // ---- a board that has just appeared -----------------------------------
    //
    // Reconnecting matches what the app does at startup, so plugging a board in
    // behaves the same as having it plugged in already - which is the whole
    // point. It does NOT override an explicit Disconnect.
    if(!hadPico && picoIndex >= 0)
    {
        LOG_INFO("pico", "board appeared on %s",
                 picoPorts[static_cast<Size>(picoIndex)].c_str());

        const PicoState ps = picoLink.state();
        if(!picoUserDisconnected
           && ps != PicoState::PICO_STATE_CONNECTED
           && ps != PicoState::PICO_STATE_CONNECTING)
        {
            connectPico();
        }
    }

    if(!hadLidar && portIndex >= 0)
    {
        LOG_INFO("lidar", "RPLIDAR adapter appeared on %s",
                 lidarPorts[static_cast<Size>(portIndex)].c_str());

        if(!lidarUserDisconnected && !lidarSource.connected()
           && lidarSource.state() != LidarState::LIDAR_STATE_CONNECTING)
        {
            connect();
        }
    }
}

Void pumpPicoRelink()
{
    if(!picoRelinkWanted || picoRelinkIn <= 0)
    {
        return;
    }

    --picoRelinkIn;
    if(picoRelinkIn > 0)
    {
        return;
    }

    picoRelinkWanted = false;
    refreshPicoPorts();

    for(Int32 i = 0; i < static_cast<Int32>(picoPorts.size()); ++i)
    {
        if(_stricmp(picoPorts[i].c_str(), picoRelinkPort.c_str()) == 0)
        {
            picoIndex = i;
            LOG_INFO("pico", "board is back on %s; reconnecting",
                     picoRelinkPort.c_str());
            connectPico();
            return;
        }
    }
    // The port did not come back under the same name. Leaving it disconnected
    // is right: the Link panel shows what happened and a person can pick.
    LOG_WARN("pico", "%s did not come back after the operation",
             picoRelinkPort.c_str());
}

Void pumpFlash()
{
    // Mirror the scripts' output into the session log. This is the single most
    // useful thing in the file: it is the toolchain's own account of what it
    // tried, and it is otherwise lost when the panel is cleared.
    const Size before = flashLog.size();
    picoFlash.drainLog(flashLog);
    for(Size i = before; i < flashLog.size(); ++i)
    {
        const Str& ln = flashLog[i];

        // picotool draws a progress bar with carriage returns, which arrives
        // here as enormous lines of "Saving file: [====] 47%". One backup put
        // twelve of them in the log and buried everything else. The bar is for
        // a person watching; the log wants the outcome.
        if(ln.find("Saving file:") != Str::npos
           || ln.find("Loading into") != Str::npos)
        {
            continue;
        }

        const Bool bad = ln.find("[error]") != Str::npos;
        ::applog::writef(bad ? ::applog::Level::LEVEL_ERROR
                             : ::applog::Level::LEVEL_DEBUG,
                         "script", "%s", ln.c_str());
    }
    if(flashLog.size() > FLASH_LOG_MAX)
        flashLog.erase(flashLog.begin(),
                          flashLog.begin() + (flashLog.size() - FLASH_LOG_MAX));

    // An operation ending changes the world: a build makes a .uf2 appear, a
    // flash changes what the board is running and takes its COM port away and
    // gives it back. Re-scan once on the transition rather than polling.
    const FlashState s = picoFlash.state();
    if(s != flashPrev)
    {
        if(flashPrev == FlashState::FLASH_STATE_WORKING)
        {
            picoFlash.refreshCatalog();
            picoFlash.refreshBoard();
            refreshPicoPorts();

            // ~2 s at 60 fps, which is about how long a freshly flashed board
            // takes to come back as a serial device.
            if(picoRelinkWanted)
            {
                picoRelinkIn = 120;
            }

            // The compiler's own opinion of the file on screen. Parsed from the
            // build output rather than from a second parser of our own, so what
            // the editor marks and what the build failed on cannot disagree.
            {
                const Vec<diag::Item> all = diag::parseAll(flashLog);
                codeDiags       = diag::forFile(all, codePath);
                refreshCodeDiags();

                if(!codeDiags.empty())
                {
                    LOG_INFO("code", "%d diagnostic(s) for %s",
                             static_cast<Int32>(codeDiags.size()), codeName.c_str());
                }
            }

            // The second half of the Code view's Build & Flash. Chained on the
            // transition rather than started alongside the build, because the
            // two cannot overlap - PicoFlash runs one operation at a time and
            // would simply reject the flash.
            // A failure has to say WHY where the person is looking. "OP FAILED"
            // in the status bar with the reason buried in another panel's log
            // is a failure report that costs more time than it saves - the
            // commonest cause by far is simply that the board is not plugged
            // in, and the script says exactly that.
            if(codeOp == CodeOp::CODE_OP_BUILDING)
            {
                if(s == FlashState::FLASH_STATE_SUCCESS)
                {
                    codeOp      = CodeOp::CODE_OP_FLASHING;
                    codeMessage = "built; flashing " + codeFlashTarget;
                    LOG_INFO("code", "build ok; flashing %s", codeFlashTarget.c_str());
                    releasePicoPortForBoardOp();
                    picoFlash.flash(codeFlashTarget);
                }
                else
                {
                    codeOp      = CodeOp::CODE_OP_NONE;
                    codeMessage = "build failed: " + lastFlashError();
                    LOG_ERROR("code", "build failed: %s", lastFlashError().c_str());
                }
            }
            else if(codeOp == CodeOp::CODE_OP_FLASHING)
            {
                codeOp = CodeOp::CODE_OP_NONE;
                if(s == FlashState::FLASH_STATE_SUCCESS)
                {
                    codeMessage = "flashed " + codeFlashTarget;
                    LOG_INFO("code", "flashed %s", codeFlashTarget.c_str());
                }
                else
                {
                    codeMessage = "flash failed: " + lastFlashError();
                    LOG_ERROR("code", "flash failed: %s", lastFlashError().c_str());
                }
            }
        }
        flashPrev = s;
    }
}

Void pumpData()
{
    if(lidarSource.poll(latestFrame))
    {
        // Telemetry keeps updating whether or not the layer is drawn: hiding a
        // layer is a map decision, not a "stop measuring" decision.
        if(layerLidar) radarView.push(latestFrame);
        haveFrame = true;
        recomputeDerived();

        // The recorder. Captured from the SAME frame the live map got, so a
        // recording is exactly what was on screen and not a second sampling of
        // the device with its own timing.
        if(recArmed)
            recording.append(latestFrame, ImGui::GetTime() - recStartS);

        // Its view follows the live feed unless a recording is being played or
        // scrubbed - otherwise the tab would sit black until you pressed
        // something, and you could not frame a shot before capturing it.
        if(recArmed || (!recPlaying && recording.empty()))
            recView.push(latestFrame);
    }

    // Turning the layer off empties the map once rather than every frame, so the
    // the trail does not linger and the fit history does not spring back on return.
    if(layerLidar != layerLidarPrev)
    {
        if(!layerLidar) radarView.clear();
        layerLidarPrev = layerLidar;
    }

    pumpPico();
    pumpFlash();
    pumpDeviceScan();
    pumpCodeLint();
    pumpPicoRelink();
    pumpCodeWatch();
    pumpCodeAutosave();
}

Void applyRange()
{
    const Float32 mm = RANGES[rangeIndex].mm;
    if(mm <= 0.0f) radarView.fit();
    else            radarView.setRangeMm(mm);
}

Void connect()
{
    lidarUserDisconnected = false;

    LOG_INFO("lidar", "connect requested: port=%s baud=%d",
             (portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
                 ? lidarPorts[portIndex].c_str() : "(none)",
             BAUDS[baudIndex].rate);
    if(portIndex < 0 || portIndex >= static_cast<Int32>(lidarPorts.size())) return;

    radarView.clear();
    haveFrame = false;
    hzCount   = 0;
    lidarSource.start(lidarPorts[portIndex], BAUDS[baudIndex].rate);
}

Void startBackup()
{
    // The board is about to be rebooted into BOOTSEL by backup.ps1, which takes
    // its COM port away; an open link would just fault.
    picoLink.disconnect();
    releasePicoPortForBoardOp();
    picoFlash.backup(backupBuf);
}

// ------------------------------------------------------------- HUD on map

Void drawMapHud(const ImVec2& p0, const ImVec2& size)
{
    // Minimal has no HUD. It is the one mode whose subject is the picture, and
    // a status line over it is the difference between a display and a readout.
    if(!radarView.is3D && radarView.mode == MapMode::MAP_MODE_MINIMAL)
        return;

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImFont* f = ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
    const Float32 px  = f->LegacySize;
    const Float32 pad = 14.0f * uiDpiScale;

    const Char* stateText = lidarStateText();
    const ImU32 accent     = lidarStateColorOnViewport();

    // ---- top left: state + connection -----------------------------------
    Float32 x = p0.x + pad;
    const Float32 y = p0.y + pad;

    ui::led(dl, ImVec2(x + px * 0.28f, y + px * 0.55f), px * 0.24f, accent,
            lidarSource.state() != LidarState::LIDAR_STATE_IDLE);
    x += px * 0.85f;
    dl->AddText(f, px, ImVec2(x, y), accent, stateText);
    x += f->CalcTextSizeA(px, FLT_MAX, 0.0f, stateText).x + 12.0f * uiDpiScale;

    if(portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
    {
        Char conn[64];
        std::snprintf(conn, sizeof(conn), "%s  -  %d baud",
                      lidarPorts[portIndex].c_str(), BAUDS[baudIndex].rate);
        dl->AddText(f, px, ImVec2(x, y), ui::plot::LABEL, conn);
    }

    // ---- top right: throughput ------------------------------------------
    Char thru[96];
    std::snprintf(thru, sizeof(thru), "%.0f pts/s   %.0f fps",
                  pointsPs, ImGui::GetIO().Framerate);
    const Float32 tw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, thru).x;
    dl->AddText(f, px, ImVec2(p0.x + size.x - pad - tw, y), ui::plot::LABEL, thru);

    // ---- bottom left: cursor / measurement -------------------------------
    Char read[128];
    read[0] = '\0';

    if(radarView.measuring)
        std::snprintf(read, sizeof(read), "measure   %.2f m", radarView.measureMm / 1000.0f);
    else if(radarView.cursorValid)
        std::snprintf(read, sizeof(read), "%.1f deg   %.2f m",
                      radarView.cursorBearingDeg, radarView.cursorRangeMm / 1000.0f);

    if(read[0])
    {
        const Float32 rw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, read).x;
        const ImVec2 bp(p0.x + pad, p0.y + size.y - pad - px - 10.0f * uiDpiScale);
        const ImVec2 be(bp.x + rw + 18.0f * uiDpiScale, bp.y + px + 12.0f * uiDpiScale);
        // A raised plate, the same treatment the buttons get: a readout sitting
        // on the display still belongs to the machine around it.
        ui::plate(bp, be, IM_COL32(0x33, 0x36, 0x3B, 0xF2), ImGui::GetStyle().FrameRounding);
        dl->AddText(f, px, ImVec2(bp.x + 9.0f * uiDpiScale, bp.y + 6.0f * uiDpiScale),
                    IM_COL32(235, 238, 242, 255), read);
    }

    // ---- bottom right: zoom state ----------------------------------------
    Char zoom[96];
    std::snprintf(zoom, sizeof(zoom), "%s   %.1f m across",
                  radarView.isAutoFit() ? "fit" : "manual",
                  radarView.visibleRangeMm() * 2.0f / 1000.0f);
    const Float32 zw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, zoom).x;
    dl->AddText(f, px, ImVec2(p0.x + size.x - pad - zw, p0.y + size.y - pad - px),
                ui::plot::LABEL, zoom);

    // ---- second line, top left: the active mode and its reading ----------
    //
    // A mode is a picture until it produces a number. This is where the number
    // goes - the widest gap, the tightest sector, how much of the revolution
    // came back unusable - so the view and its measurement are read together
    // rather than the measurement living in a panel on the other side.
    {
        const Float32 my = y + px + 6.0f * uiDpiScale;
        Float32 mx = p0.x + pad;

        // Whichever family is actually on screen. Printing the flat map's mode
        // name over a 3D scene was the first thing wrong with the 3D view.
        const Char* mn = radarView.is3D
                       ? scene3d::sceneModeName(radarView.scene)
                       : mapModeName(radarView.mode);
        dl->AddText(f, px, ImVec2(mx, my), ui::plot::ACCENT, mn);
        mx += f->CalcTextSizeA(px, FLT_MAX, 0.0f, mn).x + 12.0f * uiDpiScale;

        if(radarView.diag[0] != 0)
            dl->AddText(f, px, ImVec2(mx, my), ui::plot::LABEL, radarView.diag);
    }
}

// ------------------------------------------------------------ small parts

// A metric and its label. Deliberately NOT colour-coded: the six Live values
// used to be green / blue / green / orange / grey / grey, which looked like it
// meant something and did not. The caption says which number it is; colour is
// reserved for values that actually carry a state (see ui::sem).
Void statCell(const Char* value, const Char* caption)
{
    {
        ScopedFont sf(ui::fonts.stat);
        ImGui::TextUnformatted(value);
    }
    ScopedFont sf(ui::fonts.small);
    ImGui::TextDisabled("%s", caption);
}

Void keyValue(const Char* k, const Char* fmt, ...)
{
    Char buf[128];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    ImGui::TableNextRow();
    ImGui::TableNextColumn(); ImGui::TextDisabled("%s", k);
    ImGui::TableNextColumn(); ImGui::TextUnformatted(buf);
}

Void colored(ImU32 col, const Char* fmt, ...)
{
    Char buf[192];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    ImGui::PushStyleColor(ImGuiCol_Text, col);
    ImGui::TextUnformatted(buf);
    ImGui::PopStyleColor();
}

// ====================================================================== strip
// Always visible, whatever workspace is up, because "is it connected" is the
// question you ask constantly and the answer must never be one click away.

// One field of the status strip: an icon, a dim name, the state, and whatever
// detail belongs with it.
//
// The lamp that used to lead each field is gone. Two reasons, and the second is
// the one that actually mattered:
//
//   - It was redundant. A lamp AND a colour-coded word said the same thing
//     twice, which is what made the row feel heavy.
//   - Its halo did not fit. The strip is exactly one text line tall, and a lit
//     lamp's glow extends about 2.6x its radius - so the top and bottom of every
//     halo was being clipped by the child, which is what read as "not vertically
//     centred". It was centred; it was cropped.
//
// The icon replaces it and carries something the colour does not: WHICH
// subsystem this is. Identity from the icon, state from the colour, one each.
// ---------------------------------------------------------------------------
// The status bar's fields.
//
// DRAWN, not flowed. The old strip put each piece in with ImGui::Image and
// TextUnformatted on a SameLine, which aligns items by their TOP edge - so a
// 16 px icon sat high against 20 px text and every field was a pixel or two off
// from its neighbour. There is no way to fix that by nudging padding, because
// the icon size and the type size move independently with DPI and with the zoom
// control sitting in the same bar.
//
// So the bar owns a centreline and every element is centred on it. `x` is
// advanced by each field explicitly. This is more code than a row of SameLine
// calls and it is the only version that is actually aligned.
// ---------------------------------------------------------------------------

// Vertical centre of the bar, and the pen position along it.
struct BarPen
{
    ImDrawList* dl = nullptr;
    Float32     x  = 0.0f;   // advances left to right
    Float32     cy = 0.0f;   // the centreline, in screen space
};

Void barText(BarPen& p, const Char* text, ImU32 col)
{
    if(text == nullptr || text[0] == 0)
        return;

    const ImVec2 sz = ImGui::CalcTextSize(text);
    p.dl->AddText(ImVec2(p.x, p.cy - sz.y * 0.5f), col, text);
    p.x += sz.x;
}

Void barGap(BarPen& p, Float32 w)
{
    p.x += w;
}

Void stripField(BarPen& p, ui::Icon ic, const Char* label, ImU32 col, const Char* value, const Char* extra)
{
    const Float32 inner = ImGui::GetStyle().ItemInnerSpacing.x;

    if(ui::iconsReady())
    {
        const Float32 isz = ui::iconSize();
        ui::iconAt(p.dl, ic, ImVec2(p.x, p.cy - isz * 0.5f));
        p.x += isz + inner;
    }

    barText(p, label, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    barGap(p, inner);
    barText(p, value, col);

    if(extra != nullptr && extra[0] != 0)
    {
        barGap(p, inner);
        barText(p, extra, ImGui::GetColorU32(ImGuiCol_TextDisabled));
    }
}

Void stripSep(BarPen& p)
{
    const Float32 g = ImGui::GetFontSize() * 0.75f;
    barGap(p, g);
    barText(p, "|", ImGui::GetColorU32(ImGuiCol_TextDisabled));
    barGap(p, g);
}

// Builds a label with enough leading spaces to clear an icon drawn in the frame
// padding.
//
// COMPUTED, not guessed. The first attempt hard-coded three spaces and the icons
// sat on top of their labels the moment the type scale changed - the icon size
// and the space width move independently, so the only stable answer is to
// measure both.
const Char* iconTabLabel(Char* buf, Size cap, const Char* name)
{
    const Float32 spaceW = ImGui::CalcTextSize(" ").x;
    Int32 n = 3;
    if(spaceW > 0.0f)
        n = static_cast<Int32>(std::ceil((ui::iconSize()
                                          + ImGui::GetStyle().ItemInnerSpacing.x) / spaceW));
    if(n < 1)  n = 1;
    if(n > 24) n = 24;
    std::snprintf(buf, cap, "%*s%s", n, "", name);
    return buf;
}

Void tabIcon(ui::Icon ic)
{
    if(!ui::iconsReady())
        return;
    const ImVec2  a  = ImGui::GetItemRectMin();
    const ImVec2  b  = ImGui::GetItemRectMax();
    const Float32 sz = ui::iconSize();
    ui::iconAt(ImGui::GetWindowDrawList(), ic,
               ImVec2(a.x + ImGui::GetStyle().FramePadding.x,
                      a.y + ((b.y - a.y) - sz) * 0.5f));
}

// A- / A+ and the current percentage. Text rather than icons: the Fugue subset
// vendored in assets/icons does not carry a magnifier, and two letters read as
// "text size" more directly than a magnifying glass does anyway.
// The UI zoom, right-aligned in the bar and centred on its line.
//
// Visible rather than shortcut-only. "The UI is too small" is a complaint about
// the app, and an app whose answer is a key combination nobody is told about has
// not answered it. The keys work too - Ctrl +/-/0.
//
// Returns the x it started at, so the caller knows where the fields must stop.
Float32 drawZoomControl(Float32 cy, Float32 rightEdge)
{
    const ImGuiStyle& sty = ImGui::GetStyle();

    Char pct[16];
    std::snprintf(pct, sizeof(pct), "%d%%",
                  static_cast<Int32>(ui::userScale() * 100.0f + 0.5f));

    const Float32 btnW = ImGui::CalcTextSize("A+").x + sty.FramePadding.x * 2.0f;
    const Float32 pctW = ImGui::CalcTextSize("000%").x;
    const Float32 gap  = sty.ItemInnerSpacing.x;
    const Float32 need = btnW * 2.0f + pctW + gap * 2.0f;

    const Float32 x0 = rightEdge - need;

    // A SmallButton is NOT GetFrameHeight() tall.
    //
    // ImGui draws it with FramePadding.y forced to zero, so its height is the
    // text line and nothing more. Centring it on GetFrameHeight() - which is
    // the line PLUS two paddings - lifted both buttons above the centreline by
    // one padding, while the percentage beside them was centred correctly and
    // sat on it. Two things centred by two different rules, a few pixels apart,
    // which is exactly the kind of misalignment that reads as sloppy without
    // being obvious enough to chase.
    const Float32 bh = ImGui::GetTextLineHeight();

    const Bool atMin = ui::userScale() <= ui::USER_SCALE_MIN + 0.001f;
    const Bool atMax = ui::userScale() >= ui::USER_SCALE_MAX - 0.001f;

    // SmallButton is a real widget, so it is POSITIONED on the centreline
    // rather than drawn on it: place the cursor at (centre - height/2).
    ImGui::SetCursorScreenPos(ImVec2(x0, cy - bh * 0.5f));

    ImGui::BeginDisabled(atMin);
    if(ImGui::SmallButton("A-"))
        ui::setUserScale(ui::userScale() - ui::USER_SCALE_STEP);
    ImGui::EndDisabled();
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("Smaller  (Ctrl -)");

    // The percentage is text, so it centres on the line directly.
    //
    // Centred inside its OWN slot as well, not left-aligned in it. The slot is
    // sized for "000%" so the buttons either side never move as the number
    // changes, and left-aligning inside it meant 90% and 110% sat at different
    // distances from the button on their right.
    const ImVec2   psz  = ImGui::CalcTextSize(pct);
    const Float32  slot = x0 + btnW + gap;
    const Float32  px   = slot + ((pctW - psz.x) * 0.5f);
    ImGui::GetWindowDrawList()->AddText(ImVec2(px, cy - psz.y * 0.5f),
                                        ImGui::GetColorU32(ImGuiCol_TextDisabled),
                                        pct);

    // An invisible hit box over it, so the click-to-reset and the tooltip still
    // work now that the text is drawn rather than submitted.
    // Over the whole SLOT rather than the glyphs, so the click target does not
    // shrink when the number does.
    ImGui::SetCursorScreenPos(ImVec2(slot, cy - psz.y * 0.5f));
    ImGui::InvisibleButton("##zoompct", ImVec2(pctW, psz.y));
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("UI scale. Ctrl 0 resets it to 100%%.");
    if(ImGui::IsItemClicked())
        ui::setUserScale(1.0f);

    ImGui::SetCursorScreenPos(ImVec2(x0 + btnW + pctW + gap * 2.0f, cy - bh * 0.5f));
    ImGui::BeginDisabled(atMax);
    if(ImGui::SmallButton("A+"))
        ui::setUserScale(ui::userScale() + ui::USER_SCALE_STEP);
    ImGui::EndDisabled();
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("Bigger  (Ctrl +)");

    return x0;
}

// The bottom status bar: what every subsystem is doing, in one line.
//
// At the BOTTOM because that is where a status bar belongs - it is ambient
// information you glance at, not a header you read first. Along the top it
// competed with the tab bar for the eye and pushed the actual work down.
Void drawStatusBar()
{
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 av = ImGui::GetContentRegionAvail();

    BarPen pen;
    pen.dl = ImGui::GetWindowDrawList();
    pen.x  = p0.x;
    pen.cy = p0.y + av.y * 0.5f;   // the one centreline everything shares

    // The zoom control first, so the fields know where they have to stop.
    const Float32 stopAt = drawZoomControl(pen.cy, p0.x + av.x);

    // ---- lidar ----------------------------------------------------------
    Char lidarExtra[64] = {};
    if(lidarSource.state() == LidarState::LIDAR_STATE_SCANNING)
        std::snprintf(lidarExtra, sizeof(lidarExtra), "%s  %.1f Hz",
                      (portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
                          ? lidarPorts[portIndex].c_str() : "",
                      haveFrame ? latestFrame.hz : 0.0f);
    else if(portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
        std::snprintf(lidarExtra, sizeof(lidarExtra), "%s", lidarPorts[portIndex].c_str());

    stripField(pen, ui::Icon::ICON_RADAR, "LIDAR", lidarStateColor(),
               lidarStateText(), lidarExtra);

    // ---- pico link -------------------------------------------------------
    stripSep(pen);
    const PicoState ps = picoLink.state();
    const Str pport = picoLink.port().empty()
        ? (picoIndex >= 0 && picoIndex < static_cast<Int32>(picoPorts.size())
               ? picoPorts[picoIndex] : Str())
        : picoLink.port();
    stripField(pen, ui::Icon::ICON_PROCESSOR, "PICO", picoStateColor(ps),
               picoStateText(ps), pport.c_str());

    // ---- board -----------------------------------------------------------
    stripSep(pen);
    const BoardStatus brd = picoFlash.board();
    if(brd.bootsel)
        stripField(pen, ui::Icon::ICON_FIRMWARE, "BOARD", ui::sem::WARN, "BOOTSEL",
                   brd.drive.c_str());
    else if(brd.present)
        stripField(pen, ui::Icon::ICON_FIRMWARE, "BOARD", ui::sem::GOOD, "Running",
                   brd.program.c_str());
    else
        stripField(pen, ui::Icon::ICON_FIRMWARE, "BOARD", ui::sem::MUTED, "absent", "");

    // ---- long-running operation ------------------------------------------
    // Dropped rather than overlapped when the window is too narrow to hold it
    // clear of the zoom control. A status bar that runs into its own controls
    // is worse than one that shows three fields instead of four.
    const FlashState fs = picoFlash.state();
    if(pen.x + ImGui::GetFontSize() * 8.0f < stopAt)
    {
        stripSep(pen);
        if(fs == FlashState::FLASH_STATE_WORKING)
            stripField(pen, ui::Icon::ICON_BUILD, "OP", ui::sem::WARN, "running",
                       picoFlash.currentOp().c_str());
        else if(fs == FlashState::FLASH_STATE_SUCCESS)
            stripField(pen, ui::Icon::ICON_BUILD, "OP", ui::sem::GOOD, "done",
                       picoFlash.currentOp().c_str());
        else if(fs == FlashState::FLASH_STATE_FAILED)
            stripField(pen, ui::Icon::ICON_BUILD, "OP", ui::sem::BAD, "FAILED",
                       picoFlash.currentOp().c_str());
        else
            stripField(pen, ui::Icon::ICON_BUILD, "OP", ui::sem::MUTED, "idle", "");
    }
}

// =================================================================== overview
// Every subsystem's state, and the actions reached for most often. Live rows are
// read from the hardware; the rest are labelled with what they are, which is
// nothing yet. No status is invented for something that has never been wired.

// name | state | live value. The third column stays empty for anything that has
// no live value to report - an empty cell is the honest reading.
Void subsystemRow(ui::Icon ic, const Char* name, ImU32 col, const Char* state, const Char* value, Bool lit = true)
{
    ImGui::TableNextRow();

    ImGui::TableNextColumn();
    ui::iconLabel(ic);
    ImGui::TextUnformatted(name);

    ImGui::TableNextColumn();
    // A lamp beside every state, so the column scans as a row of indicators
    // before any of it is read as words.
    {
        const Float32 r  = ImGui::GetFontSize() * 0.20f;
        const ImVec2  cp = ImGui::GetCursorScreenPos();
        ui::led(ImGui::GetWindowDrawList(),
                ImVec2(cp.x + r * 1.8f, cp.y + ImGui::GetTextLineHeight() * 0.5f),
                r, col, lit);
        ImGui::Dummy(ImVec2(r * 3.8f, ImGui::GetTextLineHeight()));
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    }
    colored(col, "%s", state);

    ImGui::TableNextColumn();
    if(value && value[0]) ImGui::TextUnformatted(value);
}

Void drawSubsystems()
{

    if(!ImGui::BeginTable("subsys", 3,
                           ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
        return;

    // The RPLIDAR deliberately has no row here. The Sensors section sits a few
    // lines below in this same column and is open by default, so a row saying
    // "RPLIDAR C1  Scanning  COM7 9.8 Hz" would be the same fact twice on one
    // screen. The strip at the top carries it too.
    const PicoState ps = picoLink.state();
    subsystemRow(ui::Icon::ICON_LINK, "Pico link", picoStateColor(ps), picoStateText(ps),
                 picoLink.port().c_str(), ps != PicoState::PICO_STATE_DISCONNECTED);

    const BoardStatus brd = picoFlash.board();
    if(brd.bootsel)
        subsystemRow(ui::Icon::ICON_FIRMWARE, "Board firmware", ui::sem::WARN, "BOOTSEL",
                     brd.drive.c_str());
    else if(brd.present)
        // The board NAME in the right column once the firmware has answered
        // ID, falling back to the chip until it does.
        //
        // "RP2350" is true of both boards in this project and so distinguishes
        // neither. With the mule and the car's board both plugged in - which is
        // the normal state now - the only thing that tells them apart is what
        // the firmware was compiled for, and that is precisely what board= is.
        subsystemRow(ui::Icon::ICON_FIRMWARE, "Board firmware", ui::sem::GOOD,
                     brd.program.empty() ? "running" : brd.program.c_str(),
                     debugStatus.boardName.empty()
                         ? brd.chip.c_str()
                         : debugStatus.boardName.c_str());
    else
        subsystemRow(ui::Icon::ICON_FIRMWARE, "Board firmware", ui::sem::MUTED, "absent", "", false);

    // Nothing below is connected, so nothing below reports a value.
    subsystemRow(ui::Icon::ICON_SERVO,   "Servo (GP0)",           ui::sem::MUTED, "not driven", "", false);
    subsystemRow(ui::Icon::ICON_SERVO,   "ESC (GP1)",             ui::sem::MUTED, "not driven", "", false);
    subsystemRow(ui::Icon::ICON_TOF,     "ToF bumpers (GP10-13)", ui::sem::MUTED, "not wired",  "", false);
    subsystemRow(ui::Icon::ICON_ENCODER, "Wheel encoder (GP15)",  ui::sem::MUTED, "not wired",  "", false);
    subsystemRow(ui::Icon::ICON_IMU,     "IMU (I2C)",             ui::sem::MUTED, "not wired",  "", false);
    subsystemRow(ui::Icon::ICON_STORAGE, "MicroSD (SPI)",         ui::sem::MUTED, "no header",  "", false);
    subsystemRow(ui::Icon::ICON_NETWORK, "UDP link",              ui::sem::MUTED, "not built",  "", false);

    ImGui::EndTable();
}

Void drawQuickActions()
{
    const Float32 bh   = ImGui::GetFrameHeight() * 1.2f;
    const Bool  busy = picoFlash.busy();

    ImGui::SeparatorText("Quick actions");

    if(ImGui::BeginTable("quick", 2, ImGuiTableFlags_SizingStretchSame))
    {
        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        // Keyed on the LINK, not on scanning. A paused lidar is still attached,
        // and keying this off isBusy() meant pausing the motor replaced the very
        // button that would start it again with "Connect lidar".
        if(lidarSource.connected())
        {
            // Spin control, not link control. Stopping the motor is the thing
            // you actually want most of the time - the noise and the bearing
            // wear come from the rotor, not from the serial port - and it keeps
            // the device open so it comes straight back.
            const Bool spinning = lidarSource.motorEnabled();
            // Amber to stop a spinning rotor, green to start one: the tint is
            // the claim about what pressing it does, and it flips with the verb.
            if(ui::iconButton(spinning ? ui::Icon::ICON_MOTOR_STOP
                                       : ui::Icon::ICON_MOTOR_RUN,
                              spinning ? "Stop motor" : "Start motor",
                              ImVec2(-FLT_MIN, bh),
                              spinning ? ui::Tint::TINT_WARN : ui::Tint::TINT_GOOD))
                lidarSource.setMotorEnabled(!spinning);
        }
        else
        {
            ImGui::BeginDisabled(lidarPorts.empty());
            if(ui::iconButton(ui::Icon::ICON_PLUG_CONNECT, "Connect lidar",
                              ImVec2(-FLT_MIN, bh), ui::Tint::TINT_GOOD))
                connect();
            ImGui::EndDisabled();
        }

        ImGui::TableNextColumn();
        const PicoState ps = picoLink.state();
        if(ps == PicoState::PICO_STATE_CONNECTED || ps == PicoState::PICO_STATE_CONNECTING)
        {
            if(ui::iconButton(ui::Icon::ICON_PLUG_DISCONNECT, "Disconnect Pico",
                              ImVec2(-FLT_MIN, bh), ui::Tint::TINT_WARN))
            {
                // Deliberate, so a rescan must not grab the port straight back.
                // Only the button sets this - the disconnects around a flash or
                // a BOOTSEL touch are ours and transient, and treating those as
                // intent would stop a board ever reconnecting after a reflash.
                //
                // Logged because it was NOT, and that made a report of "it
                // crashes when I press disconnect" impossible to confirm from
                // the log afterwards: the press left no trace at all.
                LOG_INFO("pico", "disconnect requested by user (state=%s)",
                         picoStateText(ps));
                picoUserDisconnected = true;
                picoLink.disconnect();
                LOG_INFO("pico", "disconnect returned");
            }
        }
        else
        {
            // Greyed out when no board is present - and a greyed-out control
            // with no explanation reads as a broken one. It gets a reason.
            const Bool noPico = (picoIndex < 0);
            ImGui::BeginDisabled(noPico);
            if(ui::iconButton(ui::Icon::ICON_PLUG_CONNECT, "Connect Pico",
                              ImVec2(-FLT_MIN, bh), ui::Tint::TINT_GOOD))
            {
                connectPico();
            }
            ImGui::EndDisabled();

            if(noPico
               && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            {
                ImGui::SetTooltip(
                    "No Pico found.\n\n"
                    "Plug the board in - it is detected automatically, and this "
                    "button enables itself\nwithin about a second. Nothing "
                    "needs pressing first.\n\n"
                    "If it stays greyed: the board is running a sketch that "
                    "never called serialOpen(),\nso it never enumerated over "
                    "USB. Hold BOOTSEL while plugging the cable in\nand flash "
                    "it again.");
            }
        }

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        {
            const Bool havePort = !picoLink.port().empty() || picoIndex >= 0;
            ImGui::BeginDisabled(!havePort);
            if(ui::iconButton(ui::Icon::ICON_REBOOT, "Reboot to BOOTSEL...",
                          ImVec2(-FLT_MIN, bh), ui::Tint::TINT_WARN))
                openBootsel = true;
            ImGui::EndDisabled();
        }

        ImGui::TableNextColumn();
        ImGui::BeginDisabled(busy || backupBuf[0] == '\0');
        if(ui::iconButton(ui::Icon::ICON_BACKUP, "Back up board flash", ImVec2(-FLT_MIN, bh))) startBackup();
        ImGui::EndDisabled();

        ImGui::EndTable();
    }

    // The result of the last BOOTSEL touch, next to the button that asks for
    // one, so a failure is not silent. Only once one has been attempted: the
    // old unconditional "--" existed to stop a fixed-height panel jumping a
    // line, and in a column that scrolls it is just an unlabelled dash.
    if(bootselDone)
        colored(bootselOk ? ui::sem::GOOD : ui::sem::BAD,
                bootselOk ? "BOOTSEL touch sent" : "BOOTSEL touch failed");
}

Void sectionSystem()
{
    // The board's own report drives the rows below - the board name in
    // particular - so the ask lives with the display. This section is always on
    // screen, which is what makes polling from here reasonable rather than
    // chatter; pollBoardStatus() rate-limits itself and gives up entirely on
    // firmware that has no STATUS command.
    pollBoardStatus();

    drawSubsystems();
    ImGui::Spacing();
    drawQuickActions();
}

// ==================================================================== sensors
// The fused world view. One rotating scanner today; a ToF ring, an encoder and
// an IMU are named here so that wiring one later fills in a row instead of
// forcing a redesign.

Void drawConnection()
{
    const Bool busy = isBusy();

    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if(portItems.empty())
        ImGui::TextDisabled("No serial ports found");
    else
        ui::combo("##port", &portIndex, portItems.data(), static_cast<Int32>(portItems.size()));

    // Drawn by hand rather than with ui::combo, because the point is that two
    // of the three rows are NOT selectable and a combo cannot say that.
    ImGui::SetNextItemWidth(-FLT_MIN);
    if(ImGui::BeginCombo("##baud", BAUDS[baudIndex].label))
    {
        for(Int32 i = 0; i < BAUD_COUNT; ++i)
        {
            ImGui::BeginDisabled(!BAUDS[i].supported);
            if(ImGui::Selectable(BAUDS[i].label, i == baudIndex))
            {
                baudIndex = i;
            }
            ImGui::EndDisabled();

            if(!BAUDS[i].supported
               && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            {
                ImGui::SetTooltip(
                    "Not a rate the C1 has.\n"
                    "\n"
                    "The C1M1 datasheet lists 460800 and nothing else - no\n"
                    "minimum, no maximum, no alternative. Other RPLIDAR models\n"
                    "do run at 115200, which is where the expectation comes\n"
                    "from, and it is why this is greyed rather than absent.\n"
                    "\n"
                    "Selecting it would open the port and then receive nothing,\n"
                    "which looks like a dead sensor rather than a wrong number.");
            }
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();

    const Float32 bh = ImGui::GetFrameHeight() * 1.2f;
    if(busy)
    {
        if(ui::iconButton(ui::Icon::ICON_PLUG_DISCONNECT, "Disconnect",
                          ImVec2(-FLT_MIN, bh), ui::Tint::TINT_WARN))
        {
            lidarUserDisconnected = true;
            lidarSource.stop();
        }
    }
    else
    {
        ImGui::BeginDisabled(lidarPorts.empty());
        if(ui::iconButton(ui::Icon::ICON_PLUG_CONNECT, "Connect",
                          ImVec2(-FLT_MIN, bh), ui::Tint::TINT_GOOD))
            connect();
        ImGui::EndDisabled();
    }

    const Str        err = lidarSource.error();
    const LidarState ls  = lidarSource.state();

    if(!err.empty() && ls == LidarState::LIDAR_STATE_ERROR)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::sem::BAD);
        ImGui::TextWrapped("%s", err.c_str());
        ImGui::PopStyleColor();
    }
    else if(!err.empty() && ls == LidarState::LIDAR_STATE_UNPLUGGED)
    {
        // Muted, not red. A pulled cable is a thing somebody did on purpose and
        // already knows about; the line is here to confirm the app noticed, not
        // to raise an alarm about it.
        ImGui::PushStyleColor(ImGuiCol_Text, ui::sem::MUTED);
        ImGui::TextWrapped("%s", err.c_str());
        ImGui::PopStyleColor();

        // And the useful half: say when it is back, so the answer to "did it
        // come back" is on screen instead of being something to go and check.
        const Str was = lidarSource.port();
        if(!was.empty() && dev::portPresent(was))
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ui::sem::GOOD);
            ImGui::TextWrapped("%s is back - connect when ready", was.c_str());
            ImGui::PopStyleColor();
        }
    }
}

// One row of the layer list: a visibility box, the sensor's name, and what it
// is actually doing. Unwired sensors are disabled rather than hidden - the
// point of the list is that the shape of the finished thing is visible now.
Void sensorRow(Int32 index, Bool wired, Bool* vis, const Char* name, ImU32 col, const Char* state)
{
    static Bool never = false;

    ImGui::PushID(index);
    ImGui::BeginDisabled(!wired);

    ui::checkbox("##vis", wired ? vis : &never);
    ImGui::SameLine();

    // A list row, not a control: only the selected one is drawn, marked down its
    // left edge. Outlining every row would make the list read as a column of
    // text fields, which is exactly what it is not.
    const Bool selected = (selSensor == index);
    const ImVec2 rowSz(ImGui::GetContentRegionAvail().x * 0.52f, 0.0f);

    ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.0f, 0.5f));
    const Bool hit = ui::segmentedButton(name, selected, rowSz, ui::Mark::MARK_LEFT_BAR);
    ImGui::PopStyleVar();
    if(hit && wired)
        selSensor = index;

    ImGui::SameLine();
    colored(col, "%s", state);

    ImGui::EndDisabled();
    ImGui::PopID();
}

Void drawSensorList()
{

    Char st[48];
    if(lidarSource.state() == LidarState::LIDAR_STATE_SCANNING)
        std::snprintf(st, sizeof(st), "%.1f Hz", haveFrame ? latestFrame.hz : 0.0f);
    else
        std::snprintf(st, sizeof(st), "%s", lidarStateText());

    sensorRow(0, true, &layerLidar, "RPLIDAR C1", lidarStateColor(), st);

    static Bool off = false;
    sensorRow(1, false, &off, "ToF front level (GP10)",   ui::sem::MUTED, "not wired");
    sensorRow(2, false, &off, "ToF front down ~20 (GP11)", ui::sem::MUTED, "not wired");
    sensorRow(3, false, &off, "ToF (GP12)",                ui::sem::MUTED, "not wired");
    sensorRow(4, false, &off, "ToF (GP13)",                ui::sem::MUTED, "not wired");
    sensorRow(5, false, &off, "Wheel encoder (GP15)",      ui::sem::MUTED, "not wired");
    sensorRow(6, false, &off, "IMU (I2C)",                 ui::sem::MUTED, "not wired");
}

Void tabLive()
{
    Char hz[24] = "--", pts[24] = "--", valid[24] = "--";
    Char nearS[24] = "--", meanS[24] = "--", maxS[24] = "--";

    if(haveFrame)
    {
        std::snprintf(hz,  sizeof(hz),  "%.1f", latestFrame.hz);
        std::snprintf(pts, sizeof(pts), "%d",   static_cast<Int32>(latestFrame.points.size()));

        const Float64 frac = latestFrame.points.empty()
                          ? 0.0 : static_cast<Float64>(nInspec) / static_cast<Float64>(latestFrame.points.size());
        std::snprintf(valid, sizeof(valid), "%d%%", static_cast<Int32>((frac * 100.0 + 0.5)));

        if(radarView.hasNearest)
            std::snprintf(nearS, sizeof(nearS), "%.2f", radarView.nearestMm / 1000.0f);
        std::snprintf(meanS, sizeof(meanS), "%.2f", meanMm / 1000.0f);
        std::snprintf(maxS,  sizeof(maxS),  "%.2f", maxRangeMm / 1000.0f);
    }

    if(ImGui::BeginTable("stats", 3, ImGuiTableFlags_SizingStretchSame))
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); statCell(hz,     "Hz");
        ImGui::TableNextColumn(); statCell(pts,    "pts/rev");
        ImGui::TableNextColumn(); statCell(valid,  "in-spec");
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "Returns inside 0.05-12 m, as a share of the revolution.\n"
                "\n"
                "12 m is the WHITE figure. The C1M1 datasheet gives two ranges:\n"
                "0.05-12 m against a 70%% reflective target and only 0.05-6 m\n"
                "against a 10%% one. A dark surface at 8 m is outside what the\n"
                "sensor is specified to see, and it is counted here as in-spec\n"
                "regardless - nothing in the stream says how reflective the\n"
                "thing it hit was.\n"
                "\n"
                "So on a dark scene this reads high. It is a coverage figure,\n"
                "not a guarantee.");
        }

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); statCell(nearS, "near (m)");
        ImGui::TableNextColumn(); statCell(meanS, "mean (m)");
        ImGui::TableNextColumn(); statCell(maxS,  "max (m)");
        ImGui::EndTable();
    }

    Char overlay[48];
    std::snprintf(overlay, sizeof(overlay), "rotation  %.1f Hz", haveFrame ? latestFrame.hz : 0.0f);
    ImGui::PlotLines("##hz", hzHist, hzCount, 0, overlay,
                     0.0f, 15.0f, ImVec2(-FLT_MIN, 46.0f * uiDpiScale));

    ImGui::TextDisabled("Clearance by sector (m, capped %.1f)", CLEARANCE_CAP_M);
    ImGui::PlotHistogram("##sectors", sectorM, SECTORS, 0, nullptr,
                         0.0f, CLEARANCE_CAP_M, ImVec2(-FLT_MIN, 58.0f * uiDpiScale));
}

Void tabSignal()
{
    // Return classification. These four sum to the revolution's sample count,
    // which is what makes the in-spec percentage interpretable rather than
    // just low.
    const Int32 total = haveFrame ? static_cast<Int32>(latestFrame.points.size()) : 0;

    ImGui::TextDisabled("Returns this revolution (%d samples)", total);

    if(ImGui::BeginTable("returns", 3, ImGuiTableFlags_SizingStretchSame |
                                        ImGuiTableFlags_RowBg))
    {
        auto cell = [&](const Char* label, Int32 n, ImU32 col)
        {
            ImGui::TableNextColumn();
            ImGui::PushStyleColor(ImGuiCol_Text, col);
            ImGui::Text("%d", n);
            ImGui::PopStyleColor();
            ScopedFont sf(ui::fonts.small);
            ImGui::TextDisabled("%s", label);
            if(total > 0) ImGui::TextDisabled("%.0f%%", 100.0 * n / total);
        };

        ImGui::TableNextRow();
        cell("in spec",   nInspec,   ui::sem::GOOD);
        cell("no return", nNoreturn, ui::sem::MUTED);
        cell("< 50 mm",   nToonear,  ui::sem::WARN);

        ImGui::EndTable();
    }

    if(nToofar > 0)
        ImGui::TextDisabled("beyond 12 m: %d", nToofar);
    else
        ImGui::TextDisabled("beyond 12 m: none");

    ImGui::Spacing();

    // Signal quality is reported per measurement by the device and is otherwise
    // completely invisible - it is the main clue when returns start dropping.
    ImGui::TextDisabled("Signal quality  (mean %.1f, range %d-%d of 63)",
                        qMean, qMin, qMax);
    ImGui::PlotHistogram("##qhist", qHist, QUALITY_BUCKETS, 0, nullptr,
                         0.0f, qHistMax, ImVec2(-FLT_MIN, 62.0f * uiDpiScale));
}

Void tabScan()
{
    const LidarScanInfo si = lidarSource.scanInfo();

    const Float32 angRes = (haveFrame && !latestFrame.points.empty())
                        ? 360.0f / static_cast<Float32>(latestFrame.points.size()) : 0.0f;

    if(ImGui::BeginTable("scan", 2, ImGuiTableFlags_SizingStretchProp))
    {
        keyValue("Mode", "%s", si.mode.empty() ? "--" : si.mode.c_str());
        keyValue("Mode id", "%d", si.modeId);
        keyValue("Sample period", si.usPerSample > 0 ? "%.2f us" : "--",
                 si.usPerSample);
        keyValue("Sample rate", si.usPerSample > 0 ? "%.2f kHz" : "--",
                 si.usPerSample > 0 ? 1000.0f / si.usPerSample : 0.0f);
        keyValue("Mode max range", si.maxDistanceM > 0 ? "%.1f m" : "--",
                 si.maxDistanceM);
        keyValue("Angular res", angRes > 0 ? "%.2f deg" : "--", angRes);
        keyValue("Coverage", "%.0f%% of 360 deg", coverageDeg * 100.0f);
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Range distribution (0 - 12 m, 0.5 m bins)");
    ImGui::PlotHistogram("##dhist", distHist, DIST_BUCKETS, 0, nullptr,
                         0.0f, distHistMax, ImVec2(-FLT_MIN, 70.0f * uiDpiScale));
}

Void tabDevice()
{
    const LidarDeviceInfo info = lidarSource.info();
    const LidarStats      st   = lidarSource.stats();
    const Bool known = !info.serial.empty();

    if(ImGui::BeginTable("dev", 2, ImGuiTableFlags_SizingStretchProp))
    {
        keyValue("Model",    known ? "0x%02X" : "--", info.model);
        keyValue("Firmware", known ? "%d.%02d" : "--", info.fwMajor, info.fwMinor);
        keyValue("Hardware", known ? "rev %d" : "--", info.hwRev);
        keyValue("Health",   "%s", known ? (info.health == 0 ? "OK" : "check") : "--");
        ImGui::EndTable();
    }

    if(known)
    {
        ScopedFont sf(ui::fonts.small);
        ImGui::TextDisabled("%s", info.serial.c_str());
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Session");

    if(ImGui::BeginTable("sess", 2, ImGuiTableFlags_SizingStretchProp))
    {
        const Int32 mins = static_cast<Int32>((st.uptimeS / 60.0));
        const Int32 secs = static_cast<Int32>(st.uptimeS) % 60;

        keyValue("Uptime", "%dm %02ds", mins, secs);
        keyValue("Revolutions", "%llu", st.frames);
        keyValue("Measurements", "%llu", st.points);
        keyValue("Dropped revs", "%u", st.timeouts);
        keyValue("Avg rate", st.uptimeS > 1.0 ? "%.2f Hz" : "--",
                 st.uptimeS > 1.0 ? static_cast<Float64>(st.frames) / st.uptimeS : 0.0);

        // The time-of-flight core drifts with die temperature, so ranges are
         // not trustworthy for the first two minutes. Measured on this unit -
         // the datasheet states TOF and a fusion algorithm and goes no finer.
         // That is a value, not a lecture.
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::TextDisabled("Pre-heat");
        ImGui::TableNextColumn();
        if(st.uptimeS <= 0.0)       ImGui::TextUnformatted("--");
        else if(st.uptimeS < 120.0) colored(ui::sem::WARN, "%.0f / 120 s", st.uptimeS);
        else                          colored(ui::sem::GOOD, "done");

        ImGui::EndTable();
    }
}

// Car or bare sensor, at the origin. Shared by both dimensions, because it is a
// claim about the machine rather than about a projection.
//
// Worth a control rather than a constant: right now SENSOR is the honest
// picture. The C1 is on a desk and there is no car, so a 430 mm shell drawn
// round a 56 mm puck is a statement about the future, and there should be a way
// to ask what is actually there.
Void drawEgoSwitch()
{
    // A SCOPE, because ImGui derives a widget's identity from its LABEL, and
    // this bar has two buttons called "Car" - one here, one in the camera Lock
    // pair beside it. Without a scope they are literally the same widget: ImGui
    // reports the conflict, and clicking one can drive the other.
    //
    // Named rather than an index, so the two scopes cannot collide with each
    // other the way the labels did.
    ImGui::PushID("ego-switch");

    const ImGuiStyle& sty = ImGui::GetStyle();
    const Float32 w = ImGui::CalcTextSize("Sensor").x + sty.FramePadding.x * 2.0f;

    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("Show");
    ImGui::SameLine();

    const Bool isCar = (radarView.ego == scene3d::EgoView::EGO_VIEW_CAR);

    if(ui::segmentedIconButton(ui::Icon::ICON_SCENE_FIT, "Car", isCar,
                               ImVec2(w, 0.0f)))
        radarView.ego = scene3d::EgoView::EGO_VIEW_CAR;
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("The TT-02, to scale: 442 x 186 mm.\n"
                          "What will be there once it is built.");

    ImGui::SameLine();
    if(ui::segmentedIconButton(ui::Icon::ICON_RADAR, "Sensor", !isCar,
                               ImVec2(w, 0.0f)))
        radarView.ego = scene3d::EgoView::EGO_VIEW_SENSOR;
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("The RPLIDAR C1 alone, to scale: 55.6 x 55.6 x 41.3 mm.\n"
                          "What is actually on the desk.");

    ImGui::PopID();
}

Void drawControlBar()
{
    // The row belongs to the dimension. Range, Trail, Labels and Nearest are all
    // properties of a top-down projection - there is no "range" when you are
    // orbiting, and a trail of past revolutions in 3D is a pile, not a history.
    // Showing them greyed out would only advertise controls that will never do
    // anything here.
    if(centralView == 1)
    {
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Drag to orbit  |  right-drag to pan  |  wheel to zoom");

        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();

        if(ui::iconButton(ui::Icon::ICON_RESET_VIEW, "Reset camera"))
            radarView.cam = scene3d::Camera{};

        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("Lock");
        ImGui::SameLine();

        // See drawEgoSwitch: this pair also has a button called "Car".
        ImGui::PushID("camera-lock");

        const Float32 lockW = ImGui::CalcTextSize("World").x
                            + ImGui::GetStyle().FramePadding.x * 2.0f;

        if(ui::segmentedIconButton(ui::Icon::ICON_SCENE_FIT, "Car",
                                   radarView.cam.lockToCar, ImVec2(lockW, 0.0f)))
            radarView.cam.lockToCar = true;
        if(ImGui::IsItemHovered())
            ImGui::SetTooltip("The car stays centred. Orbit and zoom still work; "
                              "panning does not, because that is what locked means.");

        ImGui::SameLine();
        if(ui::segmentedIconButton(ui::Icon::ICON_DIM_3D, "World",
                                   !radarView.cam.lockToCar, ImVec2(lockW, 0.0f)))
            radarView.cam.lockToCar = false;
        if(ImGui::IsItemHovered())
            ImGui::SetTooltip("Free camera. Right-drag pans anywhere and the car "
                              "can leave the frame.");

        ImGui::PopID();

        ImGui::SameLine();
        ImGui::TextUnformatted("|");
        ImGui::SameLine();
        drawEgoSwitch();

        ImGui::SameLine();
        if(ui::iconButton(ui::Icon::ICON_MODE_POINTS, "Top down"))
        {
            // Straight down, which is the flat map's viewpoint - so the two
            // dimensions can be compared without guessing at the orientation.
            radarView.cam.pitch = 1.52f;
            radarView.cam.yaw   = 0.0f;
        }
        return;
    }

    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Range");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.0f);
    if(ui::combo("##range", &rangeIndex, RANGE_ITEMS, RANGE_COUNT))
        applyRange();

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    ui::checkbox("Grid", &radarView.showGrid);          ImGui::SameLine();
    ui::checkbox("Trail", &radarView.showTrail);        ImGui::SameLine();
    ui::checkbox("Labels", &radarView.showLabels);      ImGui::SameLine();
    ui::checkbox("Nearest", &radarView.showNearest);

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    if(ui::iconButton(ui::Icon::ICON_RESET_VIEW, "Reset view"))
    {
        rangeIndex = 0;
        radarView.fit();
    }

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();
    drawEgoSwitch();
}

// The map itself is drawn at the root of the frame, not here - it is permanent
// and belongs to no section. This is everything that describes it.
Void sectionSensors()
{
    drawSensorList();

    ImGui::SeparatorText("RPLIDAR C1 link");
    drawConnection();

    // The readouts below describe the SELECTED sensor, not the app. Today
    // that is always the RPLIDAR - saying so keeps the four tab names from
    // reading as global.
    ImGui::SeparatorText("Telemetry - RPLIDAR C1");

    if(ImGui::BeginTabBar("##lidartabs"))
    {
        auto sub = [](Int32 which)
        {
            return (forceSub == which && forceTabFrames > 0)
                 ? ImGuiTabItemFlags_SetSelected : 0;
        };

        {
            Char lb[40];
            const Bool t = ImGui::BeginTabItem(iconTabLabel(lb, sizeof(lb), "Live"),
                                               nullptr, sub(0));
            tabIcon(ui::Icon::ICON_LIVE);
            if(t)
            {
                tabLive();
                ImGui::EndTabItem();
            }
        }
        {
            Char lb[40];
            const Bool t = ImGui::BeginTabItem(iconTabLabel(lb, sizeof(lb), "Signal"),
                                               nullptr, sub(1));
            tabIcon(ui::Icon::ICON_SIGNAL);
            if(t)
            {
                tabSignal();
                ImGui::EndTabItem();
            }
        }
        {
            Char lb[40];
            const Bool t = ImGui::BeginTabItem(iconTabLabel(lb, sizeof(lb), "Scan"),
                                               nullptr, sub(2));
            tabIcon(ui::Icon::ICON_SCAN);
            if(t)
            {
                tabScan();
                ImGui::EndTabItem();
            }
        }
        {
            Char lb[40];
            const Bool t = ImGui::BeginTabItem(iconTabLabel(lb, sizeof(lb), "Device"),
                                               nullptr, sub(3));
            tabIcon(ui::Icon::ICON_DEVICE);
            if(t)
            {
                tabDevice();
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
}

// The permanent left region: the map, with the control bar under it. Both are
// sized by the caller, which owns the split between map and sidebar.
// --view <map|3d|record|code|pico|reference> preselects a central tab at startup. Held for a few frames
// because a tab bar only honours SetSelected once it has laid its items out,

// which is not on frame one.
Int32 forceView        = -1;   // -1 none, 0 = 2D, 1 = 3D, 2+ board index + 2
Int32 forceViewFrames = 0;

Int32 modeToggleRows();

// True when the mode strip has to scroll sideways at this content width. The
// layout and the strip itself both ask, so they cannot disagree about whether a
// scrollbar is going to appear and how tall the bar therefore is.
Bool  modeToggleScrolls(Float32 contentW);

// Rows of controls the given view puts under itself. Zero is a legitimate
// answer and means the view gets no bottom bar at all - not an empty one.
[[nodiscard]] Int32 centralControlRows(Int32 view) noexcept
{
    // Transport, then playback. The recorder does not get the render-mode strip:
    // it is a Points view on purpose - see drawRecorderControls.
    if(view == 2)
        return 2;

    // Code: files on one row, build/flash on the next.
    if(view == 3)
        return 2;

    if(view == 0 || view == 1)
        return modeToggleRows() + 1;   // render modes, then the map controls

    // The board views have nothing to configure yet. When one of them grows a
    // control - a pin filter, a package outline toggle - it declares its rows
    // here and draws them in drawCentralControls(), and no other code changes.
    return 0;
}

// Height of that bar, or 0 when the view has no controls. `contentW` is the
// width the bar will be laid out in, which the map view needs because its mode
// strip grows a scrollbar once the cells would be too narrow to read.
[[nodiscard]] Float32 centralControlHeight(Int32 view, Float32 contentW)
{
    const Int32 rows = centralControlRows(view);
    if(rows <= 0)
        return 0.0f;

    const ImGuiStyle& sty = ImGui::GetStyle();
    const Float32 n = static_cast<Float32>(rows);
    Float32 h = ImGui::GetFrameHeight() * n
              + sty.ItemSpacing.y * (n - 1.0f)
              + sty.WindowPadding.y * 2.0f;

    if((view == 0 || view == 1)
       && modeToggleScrolls(contentW - sty.WindowPadding.x * 2.0f))
        h += sty.ScrollbarSize;

    return h;
}

// Beyond this many render modes the toggle wraps to a second row rather than
// squeezing the cells until the labels clip.
constexpr Int32 MODE_TOGGLE_MAX_PER_ROW = 5;

// The render-mode toggle. A segmented row rather than a combo: the modes are
// few, switching between them is the point, and a combo hides four of the five
// behind a click.
// How many rows the mode toggle occupies. Callers size the control bar from
// this, so the two cannot disagree about how tall it is.
// How many overlay buttons the strip is carrying, which depends on which
// dimension is selected: the flat map has nine, the scene has four.
Int32 activeModeCount()
{
    // Keyed on centralView rather than radarView.is3D on purpose: the bar's
    // HEIGHT is computed before the tab bar has run and its CONTENTS after, and
    // centralView is the value that is stable across both.
    return (centralView == 1)
         ? static_cast<Int32>(scene3d::SceneMode::SCENE_MODE_COUNT)
         : static_cast<Int32>(MapMode::MAP_MODE_COUNT);
}

Int32 modeToggleRows()
{
    return (activeModeCount() > MODE_TOGGLE_MAX_PER_ROW) ? 2 : 1;
}

// The tooltip that makes a mode legible. Without this the toggle is twelve
// one-word labels, several of which ("Density", "Sweep", "Validity") do not say
// what they mean to anyone who has not read the source.
//
// Plain IsItemHovered, no ImGuiHoveredFlags_DelayNormal. A delay would be nicer
// - it stops a tooltip storm while the cursor crosses the strip - but that flag
// also implies Stationary, and it never fired for a cursor placed by
// SetCursorPos, which is how this is verified. An unverifiable nicety loses to a
// tooltip that provably appears.
// One tooltip body for both mode families - they carry the same three fields
// and there is no reason for the flat map and the scene to explain themselves
// differently.
Void modeTooltipBody(const Char* name, const Char* what, const Char* read)
{
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);

    ImGui::TextUnformatted(name);
    if(ui::fonts.small != nullptr)
        ImGui::PushFont(ui::fonts.small, ui::fonts.small->LegacySize);

    ImGui::Spacing();
    ImGui::TextUnformatted(what);
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED));
    ImGui::TextUnformatted(read);
    ImGui::PopStyleColor();

    if(ui::fonts.small != nullptr)
        ImGui::PopFont();
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

Void sceneTooltip(scene3d::SceneMode m)
{
    if(!ImGui::IsItemHovered())
        return;
    const scene3d::SceneModeInfo& i = scene3d::sceneModeInfo(m);
    modeTooltipBody(i.name, i.what, i.read);
}

Void modeTooltip(MapMode m)
{
    if(!ImGui::IsItemHovered())
        return;

    const MapModeInfo& info = mapModeInfo(m);

    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);

    ImGui::TextUnformatted(info.name);
    if(ui::fonts.small != nullptr)
        ImGui::PushFont(ui::fonts.small, ui::fonts.small->LegacySize);

    ImGui::Spacing();
    ImGui::TextUnformatted(info.what);
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED));
    ImGui::TextUnformatted(info.read);
    ImGui::PopStyleColor();

    if(ui::fonts.small != nullptr)
        ImGui::PopFont();
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

Bool modeToggleScrolls(Float32 contentW)
{
    const Int32   n    = activeModeCount();
    const Int32   topN = (n + modeToggleRows() - 1) / modeToggleRows();
    const Float32 gap  = ImGui::GetStyle().ItemSpacing.x;
    const Float32 w    = (contentW - gap * static_cast<Float32>(topN - 1))
                       / static_cast<Float32>(topN);
    return w < (118.0f * uiDpiScale);
}

Void drawModeToggle()
{
    const ImGuiStyle& sty = ImGui::GetStyle();
    const Int32   n    = activeModeCount();
    const Float32 gap  = sty.ItemSpacing.x;
    const Int32   rows = modeToggleRows();

    // Rows follow the mode count now: the flat map's nine wrap to two, the
    // scene's four sit on one. What gives instead of shrinking is the WIDTH -
    // below a legible minimum the strip scrolls sideways, because a cell narrow
    // enough to clip "Clearance" has stopped being a label.
    const Float32 avail   = ImGui::GetContentRegionAvail().x;
    const Int32   topN    = (n + rows - 1) / rows;
    const Bool    scrolls = modeToggleScrolls(avail);
    const Float32 w        = scrolls
                           ? 118.0f * uiDpiScale
                           : (avail - gap * static_cast<Float32>(topN - 1))
                                 / static_cast<Float32>(topN);

    const Float32 stripH = ImGui::GetFrameHeight() * static_cast<Float32>(rows)
                         + sty.ItemSpacing.y * static_cast<Float32>(rows - 1)
                         + (scrolls ? sty.ScrollbarSize : 0.0f);

    ImGui::BeginChild("##modes", ImVec2(0.0f, stripH), ImGuiChildFlags_None,
                      scrolls ? ImGuiWindowFlags_HorizontalScrollbar
                              : ImGuiWindowFlags_NoScrollbar);

    const Float32 modeX = ImGui::GetCursorPosX();

    for(Int32 i = 0; i < n; ++i)
    {
        const Bool  second = (i >= topN);
        const Int32 rowN   = second ? (n - topN) : topN;
        const Int32 col    = second ? (i - topN) : i;

        // Cells keep a uniform width WITHIN a row so each row reads as one
        // strip; the rows do not have to match each other.
        const Float32 cellW = scrolls
                            ? w
                            : (avail - gap * static_cast<Float32>(rowN - 1))
                                  / static_cast<Float32>(rowN);

        if(col)
            ImGui::SameLine(0.0f, gap);
        else if(second)
            ImGui::SetCursorPosX(modeX);

        ImGui::PushID(i);

        if(centralView == 1)
        {
            const scene3d::SceneMode m =
                static_cast<scene3d::SceneMode>(i);
            const ui::Icon ic = static_cast<ui::Icon>(
                static_cast<Int32>(ui::Icon::ICON_SCENE_CLOUD) + i);

            if(ui::segmentedIconButton(ic, scene3d::sceneModeName(m),
                                       radarView.scene == m, ImVec2(cellW, 0.0f)))
                radarView.scene = m;
            sceneTooltip(m);
        }
        else
        {
            const MapMode m = static_cast<MapMode>(i);

            // The mode icons live in a contiguous block that mirrors MapMode, so
            // the mapping is arithmetic rather than a second table to keep in
            // step.
            const ui::Icon ic = static_cast<ui::Icon>(
                static_cast<Int32>(ui::Icon::ICON_MODE_POINTS) + i);

            if(ui::segmentedIconButton(ic, mapModeName(m),
                                       radarView.mode == m, ImVec2(cellW, 0.0f)))
                radarView.mode = m;
            modeTooltip(m);
        }

        ImGui::PopID();
    }

    ImGui::EndChild();
}

// The bottom bar for whichever view is on screen. Must agree with
// centralControlRows() about how many rows it draws, or the bar clips.
// Transport and playback for the recorder tab.
//
// The view itself is deliberately Points and nothing else. Every other mode is
// an interpretation, and what a recording has to preserve - and show you it has
// preserved - is the returns. If you want Density over a recording, that is a
// question for the mapper this file exists to make possible, not for the
// recorder.
// One line over the recorder's map, saying which frame you are looking at and
// where it came from. Without it a paused playback and a live feed are
// indistinguishable, which is the single most confusing thing a recorder can do.
Void drawRecorderHud(const ImVec2& p0, const ImVec2& size)
{
    ImDrawList* dl = ImGui::GetWindowDrawList();
    if(dl == nullptr)
        return;

    const Float32 pad = 8.0f * uiDpiScale;
    ImFont* f = ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
    const Float32 fs = f->LegacySize > 0.0f ? f->LegacySize : ImGui::GetFontSize();

    Char line[192];
    ImU32 col = ui::sem::MUTED;

    if(recArmed)
    {
        std::snprintf(line, sizeof(line), "RECORDING  -  %zu revolutions",
                      recording.count());
        col = ui::sem::BAD;
    }
    else if(!recording.empty())
    {
        std::snprintf(line, sizeof(line),
                      "%s  -  revolution %zu of %zu  -  %.2f s",
                      recPlaying ? "PLAYING" : "PAUSED",
                      recIndex + 1u, recording.count(), recPlayS);
        col = recPlaying ? ui::sem::GOOD : ui::sem::WARN;
    }
    else
    {
        std::snprintf(line, sizeof(line), "%s",
                      (lidarSource.state() == LidarState::LIDAR_STATE_SCANNING)
                          ? "live  -  press Record to capture"
                          : "no lidar; connect one to record");
    }

    dl->AddText(f, fs, ImVec2(p0.x + pad, p0.y + pad), col, line);
    static_cast<Void>(size);
}

Void drawRecorderControls()
{
    const Bool live = (lidarSource.state() == LidarState::LIDAR_STATE_SCANNING);

    // ---- row 1: capture and files ---------------------------------------
    ImGui::BeginDisabled(!live && !recArmed);
    if(recArmed)
    {
        if(ui::iconButton(ui::Icon::ICON_PAUSE, "Stop"))
            recArmed = false;
    }
    else
    {
        if(ui::iconButton(ui::Icon::ICON_RECORD, "Record", ImVec2(0, 0),
                          ui::Tint::TINT_BAD))
        {
            // A new take replaces the old one. Anything worth keeping should
            // have been saved, and silently appending two runs into one file
            // would be worse than losing the first.
            recording.clear();
            recIndex   = 0;
            recPlayS   = 0.0;
            recPlaying = false;
            recArmed   = true;
            recStartS  = ImGui::GetTime();
            recStatus.clear();
        }
    }
    ImGui::EndDisabled();
    if(!live && !recArmed && ImGui::IsItemHovered())
        ImGui::SetTooltip("The lidar is not scanning. Connect it first.");

    ImGui::SameLine();
    ImGui::BeginDisabled(recording.empty() || recArmed);
    if(ui::iconButton(ui::Icon::ICON_CLEAR, "Clear"))
    {
        recording.clear();
        recIndex = 0; recPlayS = 0.0; recPlaying = false;
        recStatus.clear();
    }
    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_SAVE, "Save"))
    {
        const Str d = rec::dir();
        if(d.empty())
        {
            recStatus = "no writable recordings directory";
            recStatusBad = true;
        }
        else
        {
            const Str name = rec::makeName();
            Str err;
            if(recording.save(d + "\\" + name, err))
            {
                recStatus = "saved " + name;
                recStatusBad = false;
                refreshRecordings();
            }
            else
            {
                recStatus = err;
                recStatusBad = true;
            }
        }
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 14.0f);
    if(recFiles.empty())
    {
        ImGui::BeginDisabled(true);
        Int32 dummy = 0;
        const Char* none = "(no recordings)";
        ui::combo("##recfile", &dummy, &none, 1);
        ImGui::EndDisabled();
    }
    else
    {
        static Vec<const Char*> items;
        items.clear();
        for(const Str& f : recFiles)
            items.push_back(f.c_str());
        ui::combo("##recfile", &recFileIndex, items.data(),
                  static_cast<Int32>(items.size()));
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(recFiles.empty() || recArmed);
    if(ui::iconButton(ui::Icon::ICON_OPEN, "Load"))
    {
        Str err;
        const Str path = rec::dir() + "\\" + recFiles[static_cast<Size>(recFileIndex)];
        if(recording.load(path, err))
        {
            recIndex = 0; recPlayS = 0.0; recPlaying = false;
            recPendingSeek = true;
            recView.clear();
            // A partial load still reports its error - "kept what was readable"
            // is a warning, not a success.
            recStatus = err.empty()
                ? ("loaded " + recFiles[static_cast<Size>(recFileIndex)])
                : err;
            recStatusBad = !err.empty();
        }
        else
        {
            recStatus = err;
            recStatusBad = true;
        }
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_REFRESH, "Rescan"))
        refreshRecordings();

    // ---- row 2: playback and readout ------------------------------------
    ImGui::BeginDisabled(recording.empty() || recArmed);

    if(recPlaying)
    {
        if(ui::iconButton(ui::Icon::ICON_PAUSE, "Pause"))
            recPlaying = false;
    }
    else
    {
        if(ui::iconButton(ui::Icon::ICON_PLAY, "Play", ImVec2(0, 0),
                              ui::Tint::TINT_GOOD))
        {
            // Replaying from the end restarts, rather than sitting there doing
            // nothing and looking broken.
            if(recPlayS >= recording.durationS() - 1e-3)
                recPlayS = 0.0;
            recPlaying = true;
        }
    }

    ImGui::SameLine();
    const Size n = recording.count();
    Int32 idx = static_cast<Int32>(recIndex);

    ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 22.0f);
    if(ImGui::SliderInt("##scrub", &idx, 0,
                        (n > 0u) ? static_cast<Int32>(n - 1u) : 0, "rev %d"))
    {
        recIndex       = static_cast<Size>(idx);
        recPlayS       = recording.at(recIndex).tS;
        recPlaying     = false;      // dragging the scrub means you want to look
        recPendingSeek = true;
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if(recArmed)
    {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::BAD),
                           "REC  %zu rev  %.1f s  %.1f MB",
                           recording.count(),
                           ImGui::GetTime() - recStartS,
                           static_cast<Float64>(recording.pointCount() * 8u)
                               / (1024.0 * 1024.0));
    }
    else if(!recording.empty())
    {
        ImGui::Text("%zu rev  %.1f s  %.1f MB", recording.count(),
                    recording.durationS(),
                    static_cast<Float64>(recording.pointCount() * 8u)
                        / (1024.0 * 1024.0));
    }
    else
    {
        ImGui::TextDisabled("nothing recorded");
    }

    if(!recStatus.empty())
    {
        ImGui::SameLine();
        ImGui::TextColored(
            ImGui::ColorConvertU32ToFloat4(recStatusBad ? ui::sem::BAD : ui::sem::GOOD),
            "%s", recStatus.c_str());
    }
}

// ================================================================= code view

// Last-write time of `path`, or 0 if it cannot be read.
// Writes the buffer to the sketch library AND to firmware/scratch/sketch.c.
//
// Both, always. The library copy is the one that survives; the slot is what
// CMake compiles. Saving only the library would build stale code, and saving
// only the slot would lose the file on the next Build & Flash - and both
// failures look like "my change did nothing", which is the worst bug a beginner
// can be handed.
Bool saveSketch()
{
    if(codeName.empty())
        codeName = sketch::makeName();
    if(codePath.empty())
        codePath = sketch::pathOf(codeName);

    const Str text = codeEditor.text();
    Str       err;

    if(!sketch::save(codePath, text, err))
    {
        codeMessage = err;
        return false;
    }

    // A LIBRARY sketch is also mirrored into the slot, because the slot is what
    // CMake compiles. A firmware source is not: it already IS where the build
    // reads it from, and copying main.c over sketch.c would silently replace one
    // program with another.
    if(sketch::targetFor(codePath) == "sketch"
       && _stricmp(codePath.c_str(), sketch::slotPath().c_str()) != 0)
    {
        if(!sketch::save(sketch::slotPath(), text, err))
        {
            codeMessage = err;
            return false;
        }
    }

    codeEditor.clearDirty();
    codeMessage = "saved " + codeName;

    // Our OWN write must not look like somebody else's. Without this the
    // watcher below would see the file change a frame later and offer to
    // reload the buffer we just wrote.
    codeFileStamp = sketch::stamp(codePath);

    ui::setNote(codeView, "saved " + codeName, ImGui::GetTime());
    LOG_INFO("code", "saved %s", codePath.c_str());
    return true;
}

// Opens `path` under the display name `name`, saving whatever is open first.
// Switching files must never be the thing that loses work.
Void openCodeFile(const Str& path, const Str& name)
{
    if(codeEditor.dirty() && !codePath.empty())
        saveSketch();

    // Marks the view as loaded even though it may never have been drawn. Its
    // first draw otherwise runs a lazy init that picks the first sketch in the
    // library, which would replace whatever was just opened - the view would
    // switch, announce "opened cal.h", and show a different file.
    codeLoaded = true;

    codePath = path;
    codeName = name;
    codeEditor.setText(sketch::load(path));
    codeView.scrollY = 0.0f;
    codeView.diags.clear();      // a new file has not been compiled yet
    codeDiags.clear();

    // Linted immediately rather than half a second later: opening a file and
    // seeing nothing, then seeing marks appear, reads as a glitch.
    codeLintDiags = lint::check(codeEditor.text(), lint::langOf(codePath));
    codeLintIn    = 30;
    refreshCodeDiags();
    codeFileStamp = sketch::stamp(path);
    codeMessage   = "opened " + name;
    ui::setNote(codeView, "opened " + name, ImGui::GetTime());
}

// Defined down with sidebarSplitter(), so the two drag handles sit together and
// stay the same as each other. Declared here because the Code tab is laid out
// long before that point in the file.
Void codeTreeSplitter(const ImVec2& at, Float32 h, Float32 thickness);

// The file tree down the left of the Code view.
//
// Two roots, because there are genuinely two kinds of file here and conflating
// them would hide the one distinction that matters: a sketch is scratch space
// that Build & Flash overwrites, and firmware/src is the real thing.
Void drawCodeTree(Float32 w, Float32 h)
{
    ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0x28, 0x28, 0x28, 0xFF));
    ImGui::BeginChild("##codetree", ImVec2(w, h), ImGuiChildFlags_None);

    // ---- collapsed: a strip with the way back, and nothing else -----------
    // A collapsed panel that leaves no handle is a panel the user has lost. The
    // arrow is the whole width of the strip so it is hard to miss.
    if(codeTreeCollapsed)
    {
        if(ImGui::ArrowButton("##treeopen", ImGuiDir_Right))
        {
            codeTreeCollapsed = false;
            panelLayoutDirty  = true;
        }
        if(ImGui::IsItemHovered())
            ImGui::SetTooltip("Show the file tree");

        ImGui::EndChild();
        ImGui::PopStyleColor();
        return;
    }

    if(ImGui::ArrowButton("##treeclose", ImGuiDir_Left))
    {
        codeTreeCollapsed = true;
        panelLayoutDirty  = true;
    }
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("Hide the file tree");

    ImGui::SameLine();
    ImGui::TextDisabled("Files");
    ImGui::Separator();

    // Re-scanned on a timer, not every frame: it is two directory enumerations,
    // and a file can appear behind our back - a sketch saved by the editor, or
    // one dropped into the folder from Explorer.
    static Vec<Str> libFiles;
    static Vec<Str> fwFiles;
    static Int32    rescanIn = 0;
    if(rescanIn <= 0)
    {
        libFiles = sketch::list();
        fwFiles  = sketch::listFirmware();
        rescanIn = 120;          // ~2 s at 60 fps
    }
    --rescanIn;

    // One row, plus the right-click menu that belongs to it.
    //
    // The menu is opened with BeginPopupContextItem, which scopes it to THIS
    // row's ID - so it acts on the file you right-clicked rather than on
    // whichever one happens to be selected. Those are different files often
    // enough to matter.
    //
    // Destructive entries do not act here. They record what was asked and the
    // caller resolves it after the tree has finished drawing: deleting a file
    // while iterating the list that drew it is how a tree crashes.
    const auto row = [](const Str& name, const Str& path, Bool sel, ui::Icon ic,
                        Bool deletable)
    {
        ImGui::PushID(path.c_str());
        ui::icon(ic);
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        const Bool hit = ImGui::Selectable(name.c_str(), sel);

        if(ImGui::BeginPopupContextItem("##rowmenu"))
        {
            ImGui::TextDisabled("%s", name.c_str());
            ImGui::Separator();

            if(ui::iconMenuItem(ui::Icon::ICON_CODE, "Open"))
            {
                openCodeFile(path, name);
            }

            if(ui::iconMenuItem(ui::Icon::ICON_SAVE, "Duplicate"))
            {
                const Str copyName = sketch::makeName();
                Str       err;
                if(sketch::save(sketch::pathOf(copyName), sketch::load(path), err))
                {
                    openCodeFile(sketch::pathOf(copyName), copyName);
                }
                else
                {
                    codeMessage = err;
                }
            }

            if(ui::iconMenuItem(ui::Icon::ICON_OPEN, "Reveal in Explorer"))
            {
                sketch::reveal(path);
            }

            ImGui::Separator();

            // firmware/src files are NOT deletable from here. They are the real
            // firmware and are in git; losing one to a stray right-click would
            // be a genuinely bad afternoon.
            ImGui::BeginDisabled(!deletable);
            ui::pushTint(ui::Tint::TINT_BAD);
            if(ui::iconMenuItem(ui::Icon::ICON_CLEAR, "Delete"))
            {
                codePendingDelete = path;
            }
            ui::popTint(ui::Tint::TINT_BAD);
            ImGui::EndDisabled();

            if(!deletable && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            {
                ImGui::SetTooltip("firmware/ is tracked in git - delete it there");
            }

            ImGui::EndPopup();
        }

        ImGui::PopID();
        return hit;
    };

    // A folder glyph, DRAWN rather than sprited.
    //
    // The icon set is Fugue and this project ships 69 of its files; a folder is
    // not among them. Pressing one of the others into service - a card, a table,
    // a server - would put a symbol on screen that means something else, and the
    // one thing a file tree must not do is lie about what a row is.
    //
    // Two shapes: a tab, and a body. An open folder leans its lid back, which is
    // the only difference a person needs to see at 16 pixels.
    const auto folderGlyph = [](Bool open)
    {
        const Float32 sz = ui::iconSize();
        const ImVec2  at = ImGui::GetCursorScreenPos();
        ImDrawList*   dl = ImGui::GetWindowDrawList();

        // Vertically centred on the text line, like ui::icon does.
        const Float32 y0 = at.y + ((ImGui::GetTextLineHeight() - sz) * 0.5f);
        const Float32 x0 = at.x;

        const ImU32 body = open ? IM_COL32(0xD8, 0x9E, 0x3C, 0xFF)
                                : IM_COL32(0xB0, 0x82, 0x33, 0xFF);
        const ImU32 tab  = IM_COL32(0x8A, 0x66, 0x28, 0xFF);

        // The tab, along the top-left.
        dl->AddRectFilled(ImVec2(x0 + sz * 0.06f, y0 + sz * 0.18f),
                          ImVec2(x0 + sz * 0.46f, y0 + sz * 0.34f),
                          tab, sz * 0.06f);

        // The body. An open folder is drawn a touch shallower so the lid reads
        // as tipped rather than as a different rectangle.
        dl->AddRectFilled(ImVec2(x0 + sz * 0.06f, y0 + sz * 0.30f),
                          ImVec2(x0 + sz * 0.94f,
                                 y0 + sz * (open ? 0.80f : 0.86f)),
                          body, sz * 0.08f);

        ImGui::Dummy(ImVec2(sz, ImGui::GetTextLineHeight()));
    };

    // One node of the firmware tree: a directory and what is directly in it.
    //
    // Built from the relative paths listFirmware() returns ("lib\\drivers\\
    // display.h"), rather than walking the disk again - the list is already the
    // architecture, in dependency order, and re-deriving it here would be a
    // second opinion about the same thing.
    struct FwNode
    {
        Str          name;                 // the folder's own name
        Vec<Str>     files;                // leaf names, in list order
        Vec<FwNode>  dirs;
    };

    const auto fwInsert = [](auto&& self, FwNode& node, const Str& rel) -> Void
    {
        const Size cut = rel.find('\\');
        if(cut == Str::npos)
        {
            node.files.push_back(rel);
            return;
        }

        const Str head = rel.substr(0, cut);
        const Str tail = rel.substr(cut + 1);

        for(FwNode& d : node.dirs)
        {
            if(d.name == head)
            {
                self(self, d, tail);
                return;
            }
        }
        node.dirs.push_back(FwNode{ head, {}, {} });
        self(self, node.dirs.back(), tail);
    };

    if(ImGui::TreeNodeEx("Sketches", ImGuiTreeNodeFlags_DefaultOpen))
    {
        for(const Str& n : libFiles)
        {
            const Str p = sketch::pathOf(n);
            if(row(n, p, _stricmp(p.c_str(), codePath.c_str()) == 0,
                   ui::Icon::ICON_CODE, true))
            {
                openCodeFile(p, n);
            }
        }
        if(libFiles.empty())
            ImGui::TextDisabled("  none saved yet");
        ImGui::TreePop();
    }

    // ---- firmware, as the folders it actually is -------------------------
    //
    // It used to be a flat list of paths - "lib\\drivers\\display.h" printed as
    // text - which is a tree written down rather than a tree. The layering IS
    // the architecture here, so the view that shows the files should show it.
    {
        FwNode root{ "firmware", {}, {} };
        for(const Str& n : fwFiles)
        {
            fwInsert(fwInsert, root, n);
        }

        // Folders first, then files, each group alphabetical, at every level.
        //
        // The insertion order was listFirmware()'s, which is the library's
        // DEPENDENCY order - hal before drivers before chassis. That is the
        // right order to read the library in and the wrong order to find a file
        // in, and a tree is for finding. Anyone wanting the dependency order has
        // docs/conventions.md, which states it as a rule rather than implying it
        // through a listing.
        //
        // Case-insensitive, because "Makefile" sorting above "app" on ASCII is
        // an artefact of the encoding and not something anybody means.
        const auto fwSort = [](auto&& self, FwNode& node) -> Void
        {
            const auto byName = [](const Str& a, const Str& b)
            {
                return _stricmp(a.c_str(), b.c_str()) < 0;
            };

            std::sort(node.dirs.begin(), node.dirs.end(),
                      [&byName](const FwNode& a, const FwNode& b)
                      {
                          return byName(a.name, b.name);
                      });
            std::sort(node.files.begin(), node.files.end(), byName);

            for(FwNode& d : node.dirs)
            {
                self(self, d);
            }
        };
        fwSort(fwSort, root);

        const Str dir = sketch::firmwareDir();

        // Recursive, so a folder added under lib/ appears without anyone
        // teaching this function about it.
        const auto drawNode = [&](auto&& self, const FwNode& node,
                                  const Str& prefix) -> Void
        {
            for(const FwNode& d : node.dirs)
            {
                ImGui::PushID(d.name.c_str());

                // The glyph sits before the label, so the arrow, the folder and
                // the name read left to right the way every file tree does.
                const Bool openNode = ImGui::TreeNodeEx(
                    "##dir", ImGuiTreeNodeFlags_DefaultOpen
                             | ImGuiTreeNodeFlags_SpanAvailWidth);

                ImGui::SameLine(0.0f, 0.0f);
                folderGlyph(openNode);
                ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
                ImGui::TextUnformatted(d.name.c_str());

                if(openNode)
                {
                    self(self, d, prefix + d.name + "\\");
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }

            for(const Str& f : node.files)
            {
                const Str rel = prefix + f;
                const Str p   = dir + "\\" + rel;
                const Bool hdr = (f.size() > 2
                                  && f.compare(f.size() - 2, 2, ".h") == 0);

                // The LEAF name in the tree, the RELATIVE path everywhere else -
                // two files called main.c in different folders must not look
                // like one row, and the editor's title should still say which.
                if(row(f, p, _stricmp(p.c_str(), codePath.c_str()) == 0,
                       hdr ? ui::Icon::ICON_FIRMWARE : ui::Icon::ICON_CODE,
                       false))
                {
                    openCodeFile(p, rel);
                }
            }
        };

        ImGui::PushID("fwroot");
        const Bool openRoot = ImGui::TreeNodeEx(
            "##fw", ImGuiTreeNodeFlags_DefaultOpen
                    | ImGuiTreeNodeFlags_SpanAvailWidth);
        ImGui::SameLine(0.0f, 0.0f);
        folderGlyph(openRoot);
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::TextUnformatted("firmware");

        if(openRoot)
        {
            drawNode(drawNode, root, Str());
            if(fwFiles.empty())
            {
                ImGui::TextDisabled("  repo not found");
            }
            ImGui::TreePop();
        }
        ImGui::PopID();
    }

    ImGui::EndChild();
    ImGui::PopStyleColor();

    // ---- destructive actions, resolved AFTER the tree has drawn ----------
    // Never during it. Deleting a file while iterating the list that drew it is
    // how a tree crashes, and the popup that asked for it is a child of the row
    // that would disappear.
    if(!codePendingDelete.empty())
    {
        const Str victim = codePendingDelete;
        codePendingDelete.clear();

        if(sketch::remove(victim))
        {
            LOG_INFO("code", "deleted %s", victim.c_str());
            rescanIn = 0;                   // the tree must forget it now

            // If it was the open one, fall back to something rather than
            // leaving the editor pointed at a file that no longer exists.
            if(_stricmp(victim.c_str(), codePath.c_str()) == 0)
            {
                const Vec<Str> left = sketch::list();
                if(!left.empty())
                {
                    openCodeFile(sketch::pathOf(left.front()), left.front());
                }
                else
                {
                    codeName = sketch::makeName();
                    codePath = sketch::pathOf(codeName);
                    codeEditor.setText(sketch::starter());
                    codeFileStamp = 0;
                }
            }
            ui::setNote(codeView, "deleted", ImGui::GetTime());
        }
        else
        {
            codeMessage = "could not delete " + victim;
            LOG_ERROR("code", "delete failed: %s", victim.c_str());
        }
    }
}

// The `:` commands the editor hands up. Everything the editor cannot know about
// on its own - what a file is, what a board is - is resolved here.
Void handleCodeCommand()
{
    const Str cmd = codeEditor.takeSubmittedCommand();
    if(cmd.empty())
        return;

    if(cmd == "w" || cmd == "wq" || cmd == "x")
    {
        // saveSketch() sets the note itself, so :w and the Save button say the
        // same thing in the same place.
        saveSketch();
        return;
    }
    if(cmd == "q" || cmd == "q!")
    {
        // There is no window to close. Saying so is better than doing nothing
        // and letting the user wonder whether the key registered.
        codeMessage = "no window to quit - use the tab bar";
        return;
    }
    if(cmd == "make" || cmd == "!make")
    {
        if(saveSketch())
        {
            codeFlashTarget = sketch::targetFor(codePath);
            codeOp          = CodeOp::CODE_OP_BUILDING;
            picoFlash.build(codeFlashTarget);
        }
        return;
    }

    codeMessage = "not a command: :" + cmd;
}

Void drawCodeControls()
{
    const Bool busy = picoFlash.busy();

    // ---- row 1: which sketch, and the file operations --------------------
    if(ui::iconButton(ui::Icon::ICON_SAVE, "Save"))
        saveSketch();

    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_CODE, "New"))
    {
        codeName = sketch::makeName();
        codePath = sketch::pathOf(codeName);
        codeEditor.setText(sketch::starter());
        codeMessage = "new sketch: " + codeName;
    }

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    // The path, not just the name. Two files called sketch.c can exist - one in
    // the library and one in the slot - and knowing which is open is the whole
    // difference between editing your program and editing its copy.
    ImGui::TextDisabled("%s", codePath.empty() ? "(unsaved)" : codePath.c_str());

    if(codeEditor.dirty())
    {
        ImGui::SameLine();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN), "modified");
    }

    // ---- row 2: the round trip -------------------------------------------
    const Float32 bh = ImGui::GetFrameHeight();

    // A HEADER is not a translation unit, and Run on one is meaningless.
    //
    // The source picker already knew this and greyed headers out - but the file
    // TREE happily opens one, and Run then offered "Build & Flash: pico_debug,
    // compiles gfx.h", which is three kinds of wrong at once. It would build
    // some unrelated target, flash it, and report success against a file it
    // never compiled.
    const Bool onHeader = (codePath.size() > 2
                           && codePath.compare(codePath.size() - 2, 2, ".h") == 0);

    ImGui::BeginDisabled(busy || onHeader);

    // Amber rather than green: this writes to the board. It is the same claim
    // the Firmware panel's Flash button makes, and it has to be the same colour
    // or the colour stops meaning anything.
    const Str target = sketch::targetFor(codePath);

    // "Run", because that is the verb every IDE uses and the one a person
    // reaches for. What it ACTUALLY does - compile, then overwrite the board's
    // flash - is in the tooltip, because a button that writes to hardware
    // should not hide that behind a friendly word.
    //
    // Still amber. It is the same claim the Firmware panel's Flash button makes
    // and it has to be the same colour or the colour stops meaning anything.
    if(ui::iconButton(ui::Icon::ICON_PLAY, "Run",
                      ImVec2(120.0f * uiDpiScale, bh), ui::Tint::TINT_WARN))
    {
        if(saveSketch())
        {
            codeFlashTarget = target;
            codeOp          = CodeOp::CODE_OP_BUILDING;
            picoFlash.build(target);
        }
    }

    if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        if(onHeader)
        {
            ImGui::SetTooltip("%s is a header.\n\n"
                              "Headers are included by other files rather than "
                              "compiled on their own,\nso there is nothing here "
                              "to build. Open a .c file - the arrow beside this "
                              "button\nlists the ones that can be flashed.",
                              codeName.c_str());
        }
        else
        {
            ImGui::SetTooltip("Build & Flash: %s\n\n"
                              "Compiles %s\nand writes it to the board, "
                              "replacing what is on it.",
                              target.c_str(),
                              codePath.empty() ? "(unsaved)" : codePath.c_str());
        }
    }

    // ---- the split-button arrow ------------------------------------------
    //
    // Run acts on the file that is OPEN, which is right nearly always and
    // impossible to be sure of at a glance. This is the "nearly": one click to
    // see exactly which source is about to be compiled, and to pick another.
    //
    // A built-in picker rather than the OS dialog. What is worth choosing here
    // is a small known set - the sketch library and firmware/src - and a native
    // dialog would happily let you pick a .png from the desktop and then fail
    // somewhere much less obvious.
    ImGui::SameLine(0.0f, 1.0f);
    if(ui::button("v", ImVec2(bh, bh), ui::Tint::TINT_WARN))
    {
        ImGui::OpenPopup("##srcpick");
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Choose which source file Run compiles");
    }

    if(ImGui::BeginPopup("##srcpick"))
    {
        // What is about to happen, spelled out. The whole reason this popup
        // exists is that it was not obvious.
        ImGui::TextDisabled("Run compiles this file:");
        ImGui::Separator();
        ImGui::TextUnformatted(codeName.empty() ? "(unsaved)" : codeName.c_str());
        ImGui::TextDisabled("%s", codePath.empty() ? "" : codePath.c_str());
        ImGui::TextDisabled("target %s  ->  firmware/build/%s.uf2",
                            target.c_str(), target.c_str());

        ImGui::Separator();
        ImGui::TextDisabled("Sketches");

        // Two lists of filenames in one popup, and a menu entry takes its ID
        // from its label - so a sketch named sketch.c and firmware/scratch/sketch.c
        // are literally the same widget to ImGui, which says so. The path is
        // unique by construction, so it is the ID.
        //
        // The tick marks whichever file is open. That matters most in exactly
        // the case that caused the clash: when both rows read the same, the
        // group heading says which is which and the tick says which is live.
        auto entry = [](ui::Icon ic, const Str& name, const Str& path,
                        const Char* note, Bool enabled)
        {
            ImGui::PushID(path.c_str());
            const Bool open = !codePath.empty()
                           && _stricmp(path.c_str(), codePath.c_str()) == 0;
            const Bool hit  = ui::iconMenuItem(ic, name.c_str(), note, enabled, open);
            ImGui::PopID();
            return hit;
        };

        const Vec<Str> lib = sketch::list();
        for(const Str& n : lib)
        {
            const Str p = sketch::pathOf(n);
            if(entry(ui::Icon::ICON_CODE, n, p, nullptr, true))
            {
                openCodeFile(p, n);
            }
        }
        if(lib.empty())
        {
            ImGui::TextDisabled("  none saved");
        }

        ImGui::Separator();
        ImGui::TextDisabled("firmware");

        const Str      fwd  = sketch::firmwareDir();
        const Str      slot = sketch::slotPath();
        const Vec<Str> fws  = sketch::listFirmware();
        for(const Str& n : fws)
        {
            const Bool hdr = (n.size() > 2 && n.compare(n.size() - 2, 2, ".h") == 0);
            const Str  p   = fwd + "\\" + n;

            // firmware/scratch/sketch.c is the scratch slot every library sketch is
            // copied into before a build, which is why it can share a name with
            // one - and why saying so here is worth a word.
            const Bool isSlot = !slot.empty()
                             && _stricmp(p.c_str(), slot.c_str()) == 0;

            // A header is not a translation unit. Listed so this matches the
            // tree, disabled so it cannot be chosen and then quietly do nothing.
            if(entry(hdr ? ui::Icon::ICON_FIRMWARE : ui::Icon::ICON_CODE, n, p,
                     hdr ? "header" : (isSlot ? "slot" : nullptr), !hdr))
            {
                openCodeFile(p, n);
            }
        }

        ImGui::EndPopup();
    }

    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_BUILD, "Build",
                      ImVec2(130.0f * uiDpiScale, bh)))
    {
        if(saveSketch())
        {
            // Build only: no flash chained onto the end of it.
            codeOp = CodeOp::CODE_OP_NONE;
            picoFlash.build(target);
        }
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Compile only. Does not touch the board.");
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    if(ImGui::Checkbox("auto", &codeAutosave))
    {
        codeAutosaveIn = 180;
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Autosave a few seconds after you stop typing");
    }

    ImGui::SameLine();
    ImGui::TextUnformatted("|");
    ImGui::SameLine();

    if(busy)
    {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                           "%s...", picoFlash.currentOp().c_str());
    }
    else if(!codeMessage.empty())
    {
        // A message naming a failure is red; everything else is a note.
        const Bool bad = codeMessage.find("failed") != Str::npos
                      || codeMessage.find("cannot") != Str::npos
                      || codeMessage.find("no Pico") != Str::npos
                      || codeMessage.find("not a command") != Str::npos;
        ImGui::TextColored(
            ImGui::ColorConvertU32ToFloat4(bad ? ui::sem::BAD : ui::sem::GOOD),
            "%s", codeMessage.c_str());
    }
    else
    {
        ImGui::TextDisabled("i insert - esc normal - :w save - / find - "
                            "ciw change word - . repeat");
    }
}

Void drawCentralControls(Int32 view)
{
    if(view == 2)
    {
        drawRecorderControls();
        return;
    }

    if(view == 3)
    {
        drawCodeControls();
        return;
    }

    if(view == 0 || view == 1)
    {
        drawModeToggle();
        drawControlBar();
    }
    // Board views: nothing. See centralControlRows().
}

// The central region. The map is one view among several rather than the only
// one, but it is still the default and still where the app lands.
// ---------------------------------------------------------------------------
// ONE view's content, at the current cursor, filling w x h.
//
// Shared by both layouts. A tab and a floating panel showing "2D" must be the
// same picture, and the only way to guarantee that is for there to be one
// function that draws it.
// ---------------------------------------------------------------------------
// Views 0-3 are the fixed ones, then the rest in tab order.
//
// There was a per-board view between Code and Reference until 2026-08-28,
// which is why these are named constants rather than literals - the indices
// have moved before and will again. A saved tab index from an older build can
// therefore select the neighbouring view once, which is a preference and not
// data; drawTabbedViews clamps anything out of range.
constexpr Int32 REF_VIEW   = 4;
constexpr Int32 RANGE_VIEW = REF_VIEW + 1;
constexpr Int32 DRIVE_VIEW = RANGE_VIEW + 1;
constexpr Int32 VIEW_COUNT = DRIVE_VIEW + 1;

// ============================================== the range view ==
//
// What the ToF sensor on the car's nose is seeing, live.
//
// The hub cannot see the Pico's pins, so everything here is an ANSWER from the
// board rather than an assumption about it. That distinction is the whole point
// of the top strip: "not detected" and "never asked" look the same on screen if
// you let them, and they are completely different problems.
Void drawRangeBody(Float32 w, Float32 h)
{
    ImGui::BeginChild("##range", ImVec2(w, h), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar
                      | ImGuiWindowFlags_NoScrollWithMouse);

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImDrawList*  dl = ImGui::GetWindowDrawList();

    dl->AddRectFilled(p0, ImVec2(p0.x + w, p0.y + h),
                      ui::ansi::BLACK);

    const Float32 pad  = 16.0f * uiDpiScale;
    const Bool    live = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

    // ---- the top strip: is there a sensor at all ------------------------
    {
        const Char* what = nullptr;
        ImU32       col  = ui::sem::MUTED;

        if(!live)
        {
            what = "Pico not connected";
        }
        else if(!sensorsAsked)
        {
            what = "asking the board what is attached...";
            col  = ui::sem::WARN;
        }
        else if(!sensorI2c)
        {
            what = "no I2C bus on the board";
            col  = ui::sem::BAD;
        }
        else if(!sensorTof)
        {
            // MUTED, not BAD. Nothing is broken - there is no ToF wired to this
            // car yet, and the sidebar says exactly that in grey two panels
            // away. Red here made an accurate report of an empty I2C bus look
            // like a failure, and a colour that cries wolf about the ordinary
            // case is a colour nobody reads on the day it matters.
            //
            // "no I2C bus" above stays BAD: that one IS a fault. The bus is on
            // the board whether or not anything hangs off it.
            what = "no VL53L1X wired - nothing at 0x29";
            col  = ui::sem::MUTED;
        }
        else
        {
            what = "VL53L1X on I2C0, GP4 / GP5";
            col  = ui::sem::GOOD;
        }

        ui::iconAt(dl, sensorTof ? ui::Icon::ICON_STATUS_OK
                                 : ui::Icon::ICON_STATUS_IDLE,
                   ImVec2(p0.x + pad, p0.y + pad));
        dl->AddText(ImVec2(p0.x + pad + ui::iconSize() + 8.0f * uiDpiScale,
                           p0.y + pad),
                    col, what);
    }

    // ---- the mode switch ------------------------------------------------
    //
    // SHORT reaches about 1.3 m and rejects ambient infrared well; LONG reaches
    // about 4 m and is easily blinded. Which one is right depends on the room,
    // so it belongs on screen rather than in a #define.
    if(live && sensorTof)
    {
        ImGui::SetCursorScreenPos(ImVec2(p0.x + w - 190.0f * uiDpiScale,
                                         p0.y + pad - 4.0f * uiDpiScale));
        if(ui::segmentedButton("Short", tofModeShort,
                               ImVec2(88.0f * uiDpiScale, 0.0f)))
        {
            tofModeShort = true;
            sendPico("TOF MODE SHORT");
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "Short range - reaches about 1.3 m\n"
                "\n"
                "The laser pulses at a HIGHER frequency, which narrows the\n"
                "window the sensor accepts a return in. A narrower window lets\n"
                "in less of everything else, so stray infrared is rejected -\n"
                "which is what actually limits this sensor outdoors.\n"
                "\n"
                "Use it when: there is sunlight, a halogen or an incandescent\n"
                "lamp about, or the target is dark or angled. All of those\n"
                "weaken the return relative to the background.\n"
                "\n"
                "The trade is only reach. Accuracy is no worse - if anything\n"
                "it is steadier, because there is less to confuse it.\n"
                "\n"
                "This is the mode a bumper wants. Nothing useful for stopping\n"
                "a car is more than a metre away, and daylight is exactly the\n"
                "condition it has to work in.");
        }

        ImGui::SameLine(0.0f, 2.0f);
        if(ui::segmentedButton("Long", !tofModeShort,
                               ImVec2(88.0f * uiDpiScale, 0.0f)))
        {
            tofModeShort = false;
            sendPico("TOF MODE LONG");
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "Long range - reaches about 4 m indoors\n"
                "\n"
                "A LOWER pulse frequency and a wider acceptance window, so a\n"
                "faint return from something far away still counts. That same\n"
                "width is what lets ambient infrared in, and daylight has a\n"
                "great deal of it.\n"
                "\n"
                "Use it when: indoors, away from a window, and you need to see\n"
                "past a metre or so.\n"
                "\n"
                "The 4 m on the box assumes a white target, a dark room and a\n"
                "long timing budget. A dark or angled surface returns far less\n"
                "light and will fall well short of it.\n"
                "\n"
                "Watch the signal and ambient figures below: if ambient starts\n"
                "to approach signal, this mode is being blinded and Short will\n"
                "do better even though it reaches less far.");
        }
    }

    const Float32 top = p0.y + pad + 28.0f * uiDpiScale;

    if(!live || !sensorTof)
    {
        // Nothing to plot, and a chart of nothing is worse than a sentence.
        const Char* hint =
            !live ? "Connect the Pico from the sidebar."
                  : "Flash pico_debug - it is the image that reports sensors.\n"
                    "Then check the wiring: VIN to 3V3, SDA to GP4, SCL to GP5.";
        // Aligned with the LABEL above it rather than with the status lamp,
        // so the block has one left edge. It was set to the panel padding,
        // which put it a lamp's width left of the line it explains.
        const Float32 textX = p0.x + pad + ui::iconSize() + 8.0f * uiDpiScale;
        dl->AddText(ImVec2(textX, top + 8.0f * uiDpiScale),
                    ui::sem::MUTED, hint);

        // The bezel, which this path used to skip - so the view lost its frame
        // in precisely the states somebody spends the most time looking at.
        ui::screenInset(p0, ImVec2(p0.x + w, p0.y + h));
        ImGui::EndChild();
        return;
    }

    const Bool good = (tofStatus == 0);

    // ---- the number ------------------------------------------------------
    {
        Char buf[32];
        std::snprintf(buf, sizeof(buf), good ? "%d" : "----", tofMm);

        ImFont* const f  = ui::fonts.big ? ui::fonts.big : ImGui::GetFont();
        const Float32 fs = (f != nullptr && f->LegacySize > 0.0f)
                         ? f->LegacySize * 2.0f
                         : ImGui::GetFontSize() * 3.0f;

        dl->AddText(f, fs, ImVec2(p0.x + pad, top),
                    good ? ui::ansi::BRWHITE : ui::sem::MUTED, buf);

        const Float32 numW = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, buf).x;
        dl->AddText(ImVec2(p0.x + pad + numW + 10.0f * uiDpiScale,
                           top + fs * 0.55f),
                    ui::sem::MUTED, "mm");

        if(good)
        {
            std::snprintf(buf, sizeof(buf), "%d.%02d m",
                          tofMm / 1000, (tofMm % 1000) / 10);
            dl->AddText(ImVec2(p0.x + pad + numW + 10.0f * uiDpiScale,
                               top + fs * 0.05f),
                        ui::plot::OK, buf);
        }

        // The status, spelled out. A bad reading is not a short reading.
        if(!good)
        {
            const Char* why =
                (tofStatus == 1) ? "too noisy"
              : (tofStatus == 2) ? "no signal - nothing came back"
              : (tofStatus == 3) ? "out of range"
              : (tofStatus == 4) ? "hardware fault"
              : (tofStatus == 5) ? "wrapped target - an echo from further away"
              : "no reading";
            dl->AddText(ImVec2(p0.x + pad, top + fs + 4.0f * uiDpiScale),
                        ui::sem::WARN, why);
        }
    }

    // ---- the strip chart -------------------------------------------------
    //
    // A number alone is hard to WATCH. Moving a hand and seeing the trace
    // follow says the sensor is tracking; a digit flickering between 812 and
    // 809 says almost nothing.
    const Float32 chartTop = top + 92.0f * uiDpiScale;
    const Float32 chartH   = std::max(60.0f, (p0.y + h) - chartTop - 62.0f * uiDpiScale);
    const Float32 chartW   = w - (2.0f * pad);

    const ImVec2 c0(p0.x + pad, chartTop);
    const ImVec2 c1(c0.x + chartW, chartTop + chartH);

    dl->AddRectFilled(c0, c1, IM_COL32(0x14, 0x16, 0x1A, 0xFF));
    dl->AddRect(c0, c1, IM_COL32(0x30, 0x32, 0x38, 0xFF));

    // A fixed 2 m scale rather than one fitted to the data. An autoscaling
    // chart looks identical whether the sensor is sweeping a room or jittering
    // by three millimetres, which is the opposite of what it is for.
    constexpr Float32 FULL_MM = 2000.0f;

    for(Int32 g = 1; g < 4; ++g)
    {
        const Float32 gy = c1.y - (chartH * (static_cast<Float32>(g) / 4.0f));
        dl->AddLine(ImVec2(c0.x, gy), ImVec2(c1.x, gy),
                    IM_COL32(0x26, 0x28, 0x2E, 0xFF));

        Char lab[16];
        std::snprintf(lab, sizeof(lab), "%.1fm",
                      static_cast<Float64>(FULL_MM * (static_cast<Float32>(g) / 4.0f)
                                           / 1000.0f));
        dl->AddText(ImVec2(c0.x + 4.0f, gy - 14.0f * uiDpiScale),
                    IM_COL32(0x50, 0x52, 0x58, 0xFF), lab);
    }

    {
        const Int32 count = tofHistoryWrapped ? TOF_HISTORY : tofHistoryAt;
        if(count > 1)
        {
            const Float32 step = chartW / static_cast<Float32>(TOF_HISTORY - 1);

            ImVec2 prev(0.0f, 0.0f);
            Bool   havePrev = false;

            for(Int32 i = 0; i < count; ++i)
            {
                // Oldest first, so the trace runs left to right in time.
                const Int32 idx = tofHistoryWrapped
                                ? ((tofHistoryAt + i) % TOF_HISTORY)
                                : i;

                Float32 v = tofHistory[idx] / FULL_MM;
                v = (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);

                const ImVec2 pt(c0.x + (static_cast<Float32>(i) * step),
                                c1.y - (v * chartH));
                if(havePrev)
                {
                    dl->AddLine(prev, pt, ui::plot::OK, 1.6f);
                }
                prev     = pt;
                havePrev = true;
            }
        }
    }

    // ---- the footer ------------------------------------------------------
    {
        Char buf[96];
        if(tofSeenMax > 0)
        {
            // Raw, in the sensor's own fixed point. Scaling them into
            // "mega-counts per second" would be a unit nobody can check and
            // would suggest a precision that is not there - the RATIO is the
            // whole diagnostic and it is scale-free.
            if(tofSignal >= 0)
            {
                std::snprintf(buf, sizeof(buf),
                              "seen %d - %d mm   %llu readings   "
                              "signal %d   ambient %d",
                              tofSeenMin, tofSeenMax,
                              static_cast<unsigned long long>(tofReplies),
                              tofSignal, tofAmbient);
            }
            else
            {
                std::snprintf(buf, sizeof(buf),
                              "seen %d - %d mm     %llu readings",
                              tofSeenMin, tofSeenMax,
                              static_cast<unsigned long long>(tofReplies));
            }
        }
        else
        {
            std::snprintf(buf, sizeof(buf), "no good reading yet");
        }
        dl->AddText(ImVec2(c0.x, c1.y + 8.0f * uiDpiScale), ui::sem::MUTED, buf);

        // How stale the number is. A link that has gone quiet leaves the last
        // reading on screen looking perfectly current, which is the one way a
        // display like this can actively mislead.
        // ---- why the reading is what it is -----------------------------
        //
        // The DISTANCE pattern alone cannot tell you. A first version of this
        // guessed "protective film" from short-and-steady readings and was
        // wrong on the real sensor: the rates said signal 5, ambient 511, which
        // is not a close object at all.
        //
        // The RATIO is the diagnosis:
        //
        //   ambient >> signal    the sensor is blinded by infrared in the room.
        //                        Sunlight, halogen and incandescent lamps all
        //                        pour out the wavelength it listens on. Short
        //                        mode exists for exactly this.
        //
        //   signal very high     something really is that close - which
        //   at a short range      includes the protective film every one of
        //                         these ships with, nearly invisible and stuck
        //                         over the lens.
        if(tofSignal >= 0 && tofReplies > 30)
        {
            const Char* why = nullptr;

            // Blinded: the room's infrared drowns the return. The threshold
            // is deliberately high - signal and ambient are COMPARABLE in
            // normal use, so "ambient exceeds signal" on its own means nothing.
            if(tofAmbient > (tofSignal * 6) && tofSignal < 300)
            {
                why = "Ambient light is swamping the signal - try Short mode, "
                      "or move away from a window or lamp.";
            }
            // A STRONG return from a FIXED short distance, whatever is in
            // front. That is not a room; it is something on the lens. Every one
            // of these sensors ships with a protective film that is nearly
            // invisible and reflects the laser straight back.
            else if(tofSeenMax > 0 && tofSeenMax < 250 && tofSignal > 200
                    && (tofSeenMax - tofSeenMin) < (tofSeenMax / 2))
            {
                why = "A strong return from a fixed short distance - is the "
                      "protective film still on the lens?";
            }

            if(why != nullptr)
            {
                dl->AddText(ImVec2(c0.x, c1.y + 26.0f * uiDpiScale),
                            ui::sem::WARN, why);
            }
        }

        const Float64 age = ImGui::GetTime() - tofLastReply;
        if(tofReplies > 0 && age > 1.0)
        {
            std::snprintf(buf, sizeof(buf), "last reply %.0f s ago", age);
            dl->AddText(ImVec2(c0.x, c1.y + 26.0f * uiDpiScale),
                        ui::sem::WARN, buf);
        }
    }

    ui::screenInset(p0, ImVec2(p0.x + w, p0.y + h));
    ImGui::EndChild();
}

// One step button: what it says, and what it does.
struct Step
{
    const Char* label;
    Int32       by;
};

// Steering steps. 50 us is about a degree and a half of wheel on a TT-02 - big
// enough to see, small enough that overshooting costs nothing.
const Step SERVO_STEPS[] =
{
    { "-50", -50 }, { "-10", -10 }, { "-1", -1 },
    { "+1",    1 }, { "+10",  10 }, { "+50", 50 },
};

// Throttle steps are smaller. A motor that jumps 50 us has already spun up by
// the time you decide it was too much. The "##esc" suffixes keep these distinct
// from the steering row - ImGui derives a widget's identity from its label, so
// two buttons called "-10" would be one button.
const Step ESC_STEPS[] =
{
    { "-10##esc", -10 }, { "-5##esc", -5 }, { "-1##esc", -1 },
    { "+1##esc",    1 }, { "+5##esc",  5 }, { "+10##esc", 10 },
};

template <typename T, Size N>
constexpr Size countOf(const T (&)[N])
{
    return N;
}

Int32 clampInt(Int32 v, Int32 lo, Int32 hi)
{
    if(v < lo)
    {
        return lo;
    }
    if(v > hi)
    {
        return hi;
    }
    return v;
}

// Offers the image this view actually needs. Shared by both of the ways a
// board can fail to answer - the port that will not open, and the port that
// opens and then says nothing - because they have the same cause and the same
// fix, and two copies of a button drift.
Void drawDriveFlashButton()
{
    if(picoFlash.busy())
    {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                           "%s...", picoFlash.currentOp().c_str());
        return;
    }

    if(ui::iconButton(ui::Icon::ICON_FLASH, "Flash Debug / Blink",
                      ImVec2(280.0f * uiDpiScale, 0.0f), ui::Tint::TINT_WARN))
    {
        // flash.ps1 does the 1200-baud touch itself, and it cannot open the
        // port while this app is holding it.
        picoLink.disconnect();
        releasePicoPortForBoardOp();
        picoFlash.flash("pico_debug");
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Builds firmware/app/main.c and writes it to the "
                          "board.\n\nThis OVERWRITES whatever sketch is on "
                          "there. The\nsource is untouched - rebuild and "
                          "reflash it from the\nCode view whenever you want "
                          "it back.");
    }
}

// ---- the calibration, on disk ------------------------------------------

Str steeringCalPath()
{
    const Str d = sketch::firmwareDir();
    return d.empty() ? Str() : (d + "\\lib\\chassis\\cal.h");
}

Bool readThrottleNumbers(const Str& text, Int32& lo, Int32& hi);

// The STEERING is loaded from settings, not from the header. The header is
// generated output - parsing back what we printed would make the two silently
// diverge the moment somebody hand-edits it, and the file itself says not to.
Void loadCalibration()
{
    calLoaded = true;
    calWritten = sketch::load(steeringCalPath());

    // The THROTTLE is different, and does come from the header.
    //
    // Not inconsistency: the steering has a working copy in settings and the
    // throttle has none. driveEscMin/Max are otherwise set only by the board's
    // DRIVE reply, so a hub that has not connected yet sits on the compile-time
    // defaults - and "Write to firmware" from that state would put 1500 back
    // over a measured idle of 1541 without anyone having touched a slider. The
    // header is what the firmware would actually be built with, which makes it
    // the right answer in the gap before a board has spoken.
    Int32 tlo = 0;
    Int32 thi = 0;
    if(!calWritten.empty()
       && readThrottleNumbers(calWritten, tlo, thi)
       && tlo > 0 && thi > tlo)
    {
        driveEscMin = tlo;
        driveEscMax = thi;
    }

    const Str txt = settings::read("steering.txt");
    if(txt.empty())
    {
        return;
    }

    Int32 l = 0;
    Int32 c = 0;
    Int32 r = 0;
    if(std::sscanf(txt.c_str(), "%d %d %d", &l, &c, &r) == 3
       && l > 0 && c > 0 && r > 0 && l < c && c < r)
    {
        calLeft   = l;
        calCenter = c;
        calRight  = r;
    }
}

Void saveCalibration()
{
    Char buf[64];
    std::snprintf(buf, sizeof(buf), "%d %d %d\n", calLeft, calCenter, calRight);
    settings::write("steering.txt", Str(buf));
}

// The generated header, built as text so the view can SHOW it before anything
// is written. A file that appears on disk with no preview is a file nobody
// reads until it is wrong.
Str steeringCalText()
{
    Char when[64] = "unknown date";
    const std::time_t now = std::time(nullptr);
    std::tm           tm{};
    if(localtime_s(&tm, &now) == 0)
    {
        std::strftime(when, sizeof(when), "%Y-%m-%d", &tm);
    }

    Char buf[2048];
    std::snprintf(buf, sizeof(buf),
        "/* ---------------------------------------------------------------------------\n"
        " * Steering calibration - GENERATED.\n"
        " *\n"
        " * Written by the hub's Drive view. Edit it THERE, not here: the next \"Write to\n"
        " * firmware\" overwrites this file completely, and a number typed in by hand is\n"
        " * gone the first time anyone touches the calibration UI.\n"
        " *\n"
        " * These are measurements of one particular car, not a datasheet. A servo's own\n"
        " * range is 1000-2000 us; what a TT-02's steering can actually reach is narrower\n"
        " * and off-centre, because the horn only fits the spline at whole-tooth\n"
        " * intervals and the linkage is whatever length it is. There is no way to know\n"
        " * these numbers except by moving the servo and watching.\n"
        " *\n"
        " * CENTER is the interesting one. 1500 us is the middle of the servo's range and\n"
        " * has nothing to say about where a car's wheels point straight - assuming it\n"
        " * does is how a servo ends up leaning on a frame at \"neutral\".\n"
        " * ------------------------------------------------------------------------- */\n"
        "#pragma once\n"
        "\n"
        "/* Full lock one way. */\n"
        "#define STEER_CAL_LEFT %d\n"
        "\n"
        "/* Wheels straight ahead. Not necessarily 1500, and usually not. */\n"
        "#define STEER_CAL_CENTER %d\n"
        "\n"
        "/* Full lock the other way. */\n"
        "#define STEER_CAL_RIGHT %d\n"
        "\n"
        "/* ---- throttle ------------------------------------------------------------\n"
        " *\n"
        " * The working range for the ESC, and the reason this section exists: the\n"
        " * steering has been persisted here since it was measured, and the throttle was\n"
        " * not. Anything set with ESCLIMITS lived in RAM and was silently back to\n"
        " * 1500-1600 after the next reboot or reflash - which is not a calibration, it\n"
        " * is a setting you have to remember to make again.\n"
        " *\n"
        " * Still forward-only. The board refuses anything below 1500 whatever is written\n"
        " * here; reverse needs a brake-then-reverse sequence and is not something to\n"
        " * reach by editing a number.\n"
        " * MIN is IDLE. Not the ESC's neutral and not a safety floor, but the pulse at\n"
        " * which this motor sits still and the next microsecond starts it turning. That\n"
        " * is a fact about this car's ESC and motor, found by winding it up until the\n"
        " * wheels moved - which is why it is not the round number anybody would guess.\n"
        " *\n"
        " * It matters that this is the floor the sliders are built from: a range\n"
        " * starting below idle spends its first stretch doing nothing at all, so the\n"
        " * control feels dead at one end for no reason a driver could work out.\n"
        " */\n"
        "#define THROTTLE_CAL_MIN %d\n"
        "#define THROTTLE_CAL_MAX %d\n"
        "\n"
        "/* ---- tuning, not measurement ---------------------------------------------\n"
        " *\n"
        " * Everything above is a fact about this car that was found by moving it.\n"
        " * These are not: they are choices about how fast an output may move and\n"
        " * about where \"moving\" starts, and a different answer is right for a bench\n"
        " * than for driving.\n"
        " *\n"
        " * They live here anyway for one reason - this is the file that survives a\n"
        " * reflash - and they are PRINTED here because this generator rewrites the\n"
        " * whole header. Anything it does not print is deleted, and chassis.h needs\n"
        " * SLEW_CAL_STEP to compile.\n"
        " */\n"
        "#define SLEW_CAL_STEP %d\n"
        "\n"
        "/* Microseconds past idle at which the car counts as being driven and the\n"
        " * tail lamps go out. Mirrored below neutral for reverse. */\n"
        "#define LIGHT_CAL_OFF_US %d\n"
        "\n"
        "/* When this car was last calibrated, so a stale set of numbers can be spotted\n"
        " * rather than trusted. \"defaults\" means nobody has calibrated this car yet. */\n"
        "#define STEER_CAL_STAMP \"measured %s\"\n",
        calLeft, calCenter, calRight, driveEscMin, driveEscMax,
        driveSlew, boardLightsOff, when);
    return Str(buf);
}

// Pulls the three numbers back out of a generated header.
//
// Not a parser for the file - a parser for what we ourselves printed, used only
// to answer "is what is on disk the same three numbers". Anything unrecognised
// reads as "no", which is the safe answer: it prompts a write rather than
// claiming agreement that was never established.
Bool readCalNumbers(const Str& text, Int32& l, Int32& c, Int32& r)
{
    const auto one = [&text](const Char* key, Int32& out)
    {
        const Size at = text.find(key);
        if(at == Str::npos)
        {
            return false;
        }
        out = std::atoi(text.c_str() + at + std::strlen(key));
        return out > 0;
    };

    return one("#define STEER_CAL_LEFT ", l)
        && one("#define STEER_CAL_CENTER ", c)
        && one("#define STEER_CAL_RIGHT ", r);
}

// The throttle pair, separately: a header written before the throttle was
// persisted has the three steering numbers and not these, and that file is
// still readable - it just needs writing again.
Bool readThrottleNumbers(const Str& text, Int32& lo, Int32& hi)
{
    const Char* p = text.c_str();
    const auto  one = [p](const Char* key, Int32& out)
    {
        const Char* q = std::strstr(p, key);
        if(q == nullptr)
        {
            return false;
        }
        out = std::atoi(q + std::strlen(key));
        return out > 0;
    };

    return one("#define THROTTLE_CAL_MIN ", lo)
        && one("#define THROTTLE_CAL_MAX ", hi);
}

// One row of the calibration: what it is, where it is, and the two things you
// ever want to do with it.
//
// "Set to here" captures the TARGET rather than the output, deliberately. The
// output is mid-slew half the time, and capturing a number the servo is merely
// passing through would record a position nobody ever looked at.
Void calRow(const Char* label, const Char* help, Int32* value)
{
    ImGui::PushID(label);

    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                       "%-10s", label);
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("%s", help);
    }

    ImGui::SameLine(110.0f * uiDpiScale);
    ImGui::SetNextItemWidth(110.0f * uiDpiScale);
    if(ImGui::InputInt("##us", value, 1, 10))
    {
        *value  = clampInt(*value, 1000, 2000);
        calDirty = true;
        saveCalibration();
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(!driveKnown);
    if(ui::button("Set to here", ImVec2(120.0f * uiDpiScale, 0.0f)))
    {
        *value   = driveServoT;
        calDirty = true;
        saveCalibration();
    }
    ImGui::EndDisabled();
    if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        ImGui::SetTooltip("Records where the servo is being told to hold, right\n"
                          "now, as this point.\n"
                          "\n"
                          "The TARGET, not the output - the output is mid-ramp\n"
                          "half the time, and a number the servo was only\n"
                          "passing through is not a position anyone looked at.");
    }

    ImGui::SameLine();
    ImGui::BeginDisabled(!driveServoOn);
    if(ui::button(driveServoOn ? "Go" : "Go (off)",
                  ImVec2(60.0f * uiDpiScale, 0.0f)))
    {
        driveSweep     = false;
        driveServoWant = *value;
        Char cmd[32];
        std::snprintf(cmd, sizeof(cmd), "SERVO %d", *value);
        sendPico(cmd);
    }
    ImGui::EndDisabled();
    if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        ImGui::SetTooltip("Drives the servo to this point, so you can check it\n"
                          "is still where you thought it was.\n"
                          "\n"
                          "Needs the servo engaged.");
    }

    ImGui::PopID();
}

// A state lamp with its label, the way every other indicator in this app is
// drawn. ui::led throws a halo when lit, which is what makes it read as
// emitting rather than as a coloured full stop - and it is the difference
// between this view looking like the console it lives in and looking like a
// form.
Void driveLamp(Bool lit, ImU32 colour, const Char* label)
{
    const Float32 r  = 4.0f * uiDpiScale;
    const ImVec2  at = ImGui::GetCursorScreenPos();
    const Float32 mid = at.y + (ImGui::GetTextLineHeight() * 0.5f);

    ui::led(ImGui::GetWindowDrawList(), ImVec2(at.x + r, mid), r, colour, lit);

    ImGui::Dummy(ImVec2(r * 2.0f + (6.0f * uiDpiScale), ImGui::GetTextLineHeight()));
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(colour), "%s", label);
}

// ---- the car, seen from above ---------------------------------------------
//
// A wireframe rather than a picture. The point is not to look like a TT-02 -
// it is to answer, at a glance and from across the bench, the two questions
// somebody standing next to a powered car actually has: WHICH WAY ARE THE
// FRONT WHEELS POINTING, and IS THE MOTOR LIVE.
//
// Both are things the numbers already say and nobody reads in time. "steering
// +0.42" is a fact you have to convert; a wheel turned forty-two percent of the
// way to full lock is a thing you see. That conversion is the whole job.
//
// Drawn in the console's own language: a bevelled plate for the chassis, lamps
// for state, and a recessed well behind it all so it reads as an instrument set
// into the panel rather than a diagram printed on it.

// How far the front wheels swing on screen at full lock.
//
// A TT-02's actual steering is around 28 degrees and drawing it true looks
// timid at this size - the wheel barely moves and the display fails at its one
// job. Exaggerated to 34, which is legible and still plainly a steering angle
// rather than a caster wheel spinning.
constexpr Float32 CHASSIS_LOCK_DEG = 34.0f;

// One stroke weight for the whole drivetrain.
//
// A wireframe is a drawing made entirely of lines, so the ONE thing that must
// not vary is how heavy a line is - vary it and the eye reads the heavier parts
// as nearer, which is a depth cue this drawing has no business making. Colour
// carries the meaning here; weight carries none.
constexpr Float32 WIRE_W = 1.0f;

// The unfilled interior. Not quite the panel black, so a wheel crossing a beam
// still occludes it.
constexpr ImU32 WIRE_VOID = IM_COL32(0x08, 0x08, 0x08, 0xFF);

// One wheel, rotated about its own centre.
Void drawWheel(ImDrawList* dl, const ImVec2& c, Float32 hw, Float32 hh, Float32 deg, ImU32 fill, ImU32 edge)
{
    const Float32 r  = deg * 3.14159265f / 180.0f;
    const Float32 cs = std::cos(r);
    const Float32 sn = std::sin(r);

    const ImVec2 corner[4] = {
        ImVec2(-hw, -hh), ImVec2(hw, -hh), ImVec2(hw, hh), ImVec2(-hw, hh)
    };

    ImVec2 p[4];
    for(Int32 i = 0; i < 4; ++i)
    {
        p[i] = ImVec2(c.x + (corner[i].x * cs) - (corner[i].y * sn),
                      c.y + (corner[i].x * sn) + (corner[i].y * cs));
    }

    // Outline over a near-black fill, not a solid.
    //
    // The fill is there only to stop the beam behind a wheel showing through it
    // - a wireframe still needs to say which part is in front - and it is dark
    // enough to read as unfilled. Everything that carries information is in the
    // stroke.
    dl->AddQuadFilled(p[0], p[1], p[2], p[3], fill);
    dl->AddQuad(p[0], p[1], p[2], p[3], edge, WIRE_W);

    // A tread line down the middle, so a rotated wheel reads as rotated rather
    // than as a slightly different rectangle.
    const ImVec2 t0(c.x - (hh * 0.62f * -sn), c.y - (hh * 0.62f * cs));
    const ImVec2 t1(c.x + (hh * 0.62f * -sn), c.y + (hh * 0.62f * cs));
    dl->AddLine(t0, t1, edge, WIRE_W);
}

// ---------------------------------------------------------------------------
// The Drive view's layout grid.
//
// ONE right-hand column, for the whole view. Every slider stops at the same x
// and whatever sits beside it - a button, a reading - starts at the same x, so
// the controls form a column instead of a staircase.
//
// They did not before: the two tuning sliders reserved 260 px for their
// readings and the steering and throttle ones reserved 120 for their buttons,
// so three different right edges ran down a panel that is one column wide. That
// is most of what made this look like a pile rather than a page.
// ---------------------------------------------------------------------------
constexpr Float32 DRIVE_TAIL_W = 120.0f;

// A section head: a rule, the name in the title face, and what it is for.
//
// Sections were MUTED body text, which is exactly what the captions under them
// are - so a heading and a footnote were typographically the same thing and the
// view read as one undifferentiated stack. The rule and the weight are the
// whole fix; no colour changes.
Void driveSection(const Char* name, const Char* what)
{
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    {
        ScopedFont sf(ui::fonts.title);
        ImGui::TextUnformatted(name);
    }

    if(what != nullptr && what[0] != '\0')
    {
        ImGui::SameLine();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED), " -  %s", what);
    }
}

// A reading pushed to the right-hand edge of the current line.
//
// Right-aligned rather than trailing the label, so a number that changes width
// - "8 us/tick" against "200 us/tick" - does not shuffle everything after it.
Void driveReading(ImU32 col, const Char* text)
{
    // One spacing in from the edge. Flush against it, the last glyph is clipped
    // by the panel's own clip rect - "lock to lock 1.10 s" lost its s - and a
    // reading that touches the frame reads as overflowing whether it is or not.
    const Float32 pad = ImGui::GetStyle().ItemSpacing.x;
    const Float32 w   = ImGui::CalcTextSize(text).x;
    const Float32 x   = ImGui::GetCursorPosX()
                      + ImGui::GetContentRegionAvail().x - w - pad;

    ImGui::SameLine();
    ImGui::SetCursorPosX(x);
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%s", text);
}

// steerN is -1..+1 of THIS car's travel; powerN is 0..1 of its throttle range.
//
// A DRIVETRAIN, not a car. Two axle beams and the shaft between them - the "I"
// - with a wheel on each end and nothing else. There is no bodywork here
// because there is nothing about the body worth reporting: the shell does not
// steer, does not drive, and drawing it would be decoration competing with the
// two things that are actually live.
Void drawChassis(ImDrawList* dl, const ImVec2& p0, Float32 w, Float32 h, Float32 steerN, Float32 powerN, Bool servoLive, Bool armed, const Int32* lamp, const Int32* lampPin)
{
    const ImVec2 p1(p0.x + w, p0.y + h);

    // The well the drivetrain sits in. Black and square-cornered: this is a
    // screen the drawing is on, not a moulded recess it sits in.
    dl->AddRectFilled(p0, p1, ui::ansi::BLACK, 0.0f);

    const ImVec2 mid((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);

    // Everything is derived from the wheelbase, so the drawing scales with the
    // panel rather than being a pile of tuned pixels.
    const Float32 byH  = h * 0.62f;
    const Float32 byW  = w * 0.46f;
    const Float32 base = (byH < byW) ? byH : byW;   // front axle to rear axle

    const Float32 axleF = mid.y - (base * 0.5f);
    const Float32 axleR = mid.y + (base * 0.5f);
    const Float32 track = base * 0.42f;             // centre to wheel centre

    const Float32 wheelH = base * 0.20f;
    const Float32 wheelW = base * 0.075f;

    const Float32 beamT  = base * 0.045f;           // half-thickness of a beam
    const Float32 shaftT = base * 0.030f;

    // Structure is one colour and one colour only. Anything that changes
    // colour in this drawing is reporting something; the frame reports nothing,
    // so it stays put.
    const ImU32 frame = ui::ansi::CYAN;

    // ---- the shaft, drawn first so the axles sit on top ------------------
    //
    // Warms with throttle along its whole length: this is the one part that
    // carries power from the motor to the rear axle, and it is the honest place
    // to show that something is being asked of it.
    // ---- the lamps --------------------------------------------------------
    //
    // All four corners, from the board's own answer. Indicators outboard of the
    // FRONT wheels and tails outboard of the REAR ones, where they are on a car.
    //
    // Drawn before the drivetrain so a beam crossing one reads as the lamp being
    // behind the axle rather than painted over it.
    //
    // A lamp with no LED soldered to it is drawn as an EMPTY ring rather than a
    // dark one. The firmware computes all eight either way, so "this lamp is off"
    // and "this lamp does not exist yet" are different facts and the drawing
    // should not merge them - the whole reason the pin list is reported at all.
    {
        const Float32 lampR = base * 0.055f;
        const Float32 lampX = track + wheelW * 2.6f;
        const Float32 lampFY = axleF - wheelH * 0.35f;
        const Float32 lampRY = axleR + wheelH * 0.35f;

        // BRYELLOW rather than a true amber, and BRRED for the tails. A real
        // indicator is amber and this is a shade off it, which is the price of
        // having exactly sixteen colours; next to the rest of the drawing the
        // consistency is worth more than the accuracy.
        const ImU32 unwired = ui::ansi::GRID;

        const auto oneLamp = [&](Float32 x, Float32 y, Int32 idx, ImU32 col)
        {
            const Bool wired = (lampPin[idx] >= 0);
            const Bool lit   = wired && (lamp[idx] > 0);

            if(wired)
            {
                ui::led(dl, ImVec2(x, y), lampR, col, lit);
            }
            dl->AddCircle(ImVec2(x, y), lampR,
                          wired ? ((col & 0x00FFFFFFu) | (0x60u << IM_COL32_A_SHIFT))
                                : unwired,
                          0, WIRE_W);
        };

        oneLamp(mid.x - lampX, lampFY, 4, ui::ansi::BRYELLOW);   // indL
        oneLamp(mid.x + lampX, lampFY, 5, ui::ansi::BRYELLOW);   // indR
        oneLamp(mid.x - lampX, lampRY, 2, ui::ansi::BRRED);      // tailL
        oneLamp(mid.x + lampX, lampRY, 3, ui::ansi::BRRED);      // tailR
    }

    const Float32 g = armed ? powerN : 0.0f;

    // The shaft: an outline, and a fill that grows from the bottom as a BAR.
    //
    // The old version tinted the whole shaft brighter with throttle, which
    // reads as "warmer" and not as a quantity - you cannot tell 40% from 60% by
    // hue. A column that fills is a number you can actually read off, and it
    // fills from the rear axle forward because that is the end the motor drives.
    dl->AddRectFilled(ImVec2(mid.x - shaftT, axleF),
                      ImVec2(mid.x + shaftT, axleR), WIRE_VOID, 0.0f);

    if(g > 0.0f)
    {
        const Float32 fillTop = axleR - ((axleR - axleF) * g);
        dl->AddRectFilled(ImVec2(mid.x - shaftT, fillTop),
                          ImVec2(mid.x + shaftT, axleR), ui::ansi::BRGREEN, 0.0f);
    }

    dl->AddRect(ImVec2(mid.x - shaftT, axleF),
                ImVec2(mid.x + shaftT, axleR),
                armed ? ui::ansi::BRGREEN : frame, 0.0f, 0, WIRE_W);

    // ---- the two axle beams ---------------------------------------------
    const auto beam = [&](Float32 y)
    {
        dl->AddRectFilled(ImVec2(mid.x - track, y - beamT),
                          ImVec2(mid.x + track, y + beamT), WIRE_VOID, 0.0f);
        dl->AddRect(ImVec2(mid.x - track, y - beamT),
                    ImVec2(mid.x + track, y + beamT), frame, 0.0f, 0, WIRE_W);
    };
    beam(axleF);
    beam(axleR);

    // A hub at each end, so a wheel reads as mounted on the beam rather than
    // floating beside it.
    // Eight segments, not twelve: at this size the facets are visible, and a
    // visibly faceted circle is what a wireframe from this era looked like.
    const auto hub = [&](Float32 x, Float32 y)
    {
        dl->AddCircleFilled(ImVec2(x, y), beamT * 1.15f, WIRE_VOID, 8);
        dl->AddCircle(ImVec2(x, y), beamT * 1.15f, frame, 8, WIRE_W);
    };

    // ---- wheels -----------------------------------------------------------
    const Float32 deg = steerN * CHASSIS_LOCK_DEG;

    // Front: dark when released, because a released servo holds nothing and the
    // wheels are wherever the ground last left them. Drawing them straight would
    // be the display inventing a fact.
    // Bright while the servo is holding them, grey while it is released - the
    // steering is either being commanded or it is not, and that is a two-state
    // fact rather than a gradient.
    const ImU32 frontFill = WIRE_VOID;
    const ImU32 frontEdge = servoLive ? ui::ansi::BRCYAN : ui::ansi::IDLE;

    drawWheel(dl, ImVec2(mid.x - track, axleF), wheelW, wheelH, deg, frontFill, frontEdge);
    drawWheel(dl, ImVec2(mid.x + track, axleF), wheelW, wheelH, deg, frontFill, frontEdge);
    hub(mid.x - track, axleF);
    hub(mid.x + track, axleF);

    // Rear: the driven pair. They warm rather than spin - nothing on this car
    // measures speed yet, so brightness is a COMMAND and not a reading, and
    // animating it would imply a measurement that does not exist.
    const ImU32 rearFill = WIRE_VOID;
    const ImU32 rearEdge = armed ? ui::ansi::BRGREEN : ui::ansi::IDLE;

    drawWheel(dl, ImVec2(mid.x - track, axleR), wheelW, wheelH, 0.0f, rearFill, rearEdge);
    drawWheel(dl, ImVec2(mid.x + track, axleR), wheelW, wheelH, 0.0f, rearFill, rearEdge);
    hub(mid.x - track, axleR);
    hub(mid.x + track, axleR);

    // ---- labels -----------------------------------------------------------
    const ImU32 faint = ui::ansi::IDLE;

    const Char* const FRONT = "FRONT";
    const Char* const REAR  = "REAR";
    dl->AddText(ImVec2(mid.x - (ImGui::CalcTextSize(FRONT).x * 0.5f),
                       p0.y + (6.0f * uiDpiScale)), faint, FRONT);
    dl->AddText(ImVec2(mid.x - (ImGui::CalcTextSize(REAR).x * 0.5f),
                       p1.y - (20.0f * uiDpiScale)), faint, REAR);

    // The angle beside the front axle: a number and a picture of the same
    // thing, because one is checkable and the other is fast.
    Char deglabel[24];
    std::snprintf(deglabel, sizeof(deglabel), "%+.0f deg", static_cast<Float64>(deg));
    dl->AddText(ImVec2(mid.x + track + (wheelW * 2.6f), axleF - (7.0f * uiDpiScale)),
                servoLive ? ui::ansi::BRYELLOW : faint, deglabel);

    // A hard border rather than the bevelled inset the rest of the panel uses.
    // The inset is a moulding, and this is a screen.
    dl->AddRect(p0, p1, ui::ansi::GRID, 0.0f, 0, WIRE_W);
}

// ============================================== the drive view ==
//
// Steering on GP0 and the ESC on GP1, with sliders instead of typed numbers.
//
// EVERY LIMIT HERE COMES FROM THE BOARD. The sliders are built from the range
// the firmware reports, not from constants in this file, so tightening the
// firmware tightens the UI and the two can never disagree about what is safe.
// A hub that decided the limits would be a hub whose safety disappears the
// moment it disconnects - and the board keeps holding the wires either way.
Void drawDriveBody(Float32 w, Float32 h)
{
    // Scrolls with the wheel, like anything else that is a list of controls.
    //
    // This carried ImGuiWindowFlags_NoScrollWithMouse, copied from the map
    // views where it is correct because the wheel ZOOMS there and scrolling
    // would fight it. Nothing here zooms, so the flag only meant the scrollbar
    // had to be dragged - a panel that shows a scrollbar and then ignores the
    // wheel reads as broken, and is.
    ImGui::BeginChild("##drive", ImVec2(w, h), ImGuiChildFlags_None,
                      ImGuiWindowFlags_None);

    // The cut-out this view sits in. Range, Reference and the board view are
    // all recessed into the casing and this one was not - it was widgets lying
    // on the panel, which is why it read as a different application. The bezel
    // is drawn last, over the content, so it shadows the top edge the way a
    // milled cut-out does.
    const ImVec2 drivePanel0 = ImGui::GetCursorScreenPos();

    const Bool live = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

    if(!live)
    {
        driveServoHeld = false;
        driveEscHeld   = false;
        driveKnown     = false;

        const PicoState st = picoLink.state();

        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                           "Pico not connected.");
        ui::screenInset(drivePanel0,
                        ImVec2(drivePanel0.x + w, drivePanel0.y + h));

        // A board that is PRESENT but will not talk is a different problem from
        // a board that is not there, and the fix for it is not "try again".
        if(st == PicoState::PICO_STATE_ERROR)
        {
            ImGui::Spacing();
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                               "The port is there and will not open.");
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                               "Most often this is the firmware, not the cable.\n"
                               "\n"
                               "A sketch that never opens a serial console still\n"
                               "enumerates a COM port - Windows keeps offering it,\n"
                               "and every attempt to open it hangs. From out here\n"
                               "that is indistinguishable from a dead board.\n"
                               "\n"
                               "This view needs the Debug / Blink firmware\n"
                               "(firmware/app/main.c). Flashing a sketch from the\n"
                               "Code view REPLACES it - they are separate programs\n"
                               "and only one can be on the board at a time.");

            const Str err = picoLink.error();
            if(!err.empty())
            {
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                                   "  %s", err.c_str());
            }

            ImGui::Spacing();
            drawDriveFlashButton();
        }

        ImGui::EndChild();
        return;
    }

    if(!driveKnown)
    {
        // ---- the firmware trap ------------------------------------------
        //
        // There are TWO images. `sketch` is whatever is open in the Code view;
        // `pico_debug` is the one that answers these commands. Pressing Build &
        // Flash on a sketch replaces pico_debug, and every view that talks to
        // the board goes quiet - which reads as "the app broke" rather than as
        // "a different program is running".
        //
        // So this says which image it needs, and offers to flash it.
        //
        // Asked on a timer rather than every frame: a draw happens sixty times a
        // second and the board has better things to do.
        const Float64 nowAsk = ImGui::GetTime();
        if(nowAsk - driveAskedAt > 1.0)
        {
            driveAskedAt = nowAsk;
            pollPico("DRIVE");
        }

        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                           "The board is not answering drive commands.");
        ImGui::Spacing();
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                           "This view needs the Debug / Blink firmware "
                           "(firmware/app/main.c).\n"
                           "\n"
                           "If you have flashed a sketch from the Code view, "
                           "that REPLACED it -\n"
                           "the two are separate programs and only one can be "
                           "on the board at a time.");
        ImGui::Spacing();

        drawDriveFlashButton();

        ui::screenInset(drivePanel0,
                        ImVec2(drivePanel0.x + w, drivePanel0.y + h));
        ImGui::EndChild();
        return;
    }

    // Poll while the view is open. Not a subscription on the board: this way
    // the traffic stops dead when the view is not on screen, and a firmware
    // that is mid-reflash is never mid-broadcast.
    const Float64 nowPoll = ImGui::GetTime();
    if(nowPoll - driveAskedAt > 0.25)
    {
        driveAskedAt = nowPoll;
        pollPico("DRIVE");
    }

    // The sweep runs here so it stops the moment the view is not drawn - a
    // servo cycling behind a tab nobody is looking at is exactly the thing that
    // should not be possible.
    if(driveSweep && !driveServoOn)
    {
        driveSweep = false;
    }
    if(driveSweep)
    {
        const Float64 now = ImGui::GetTime();
        if(now - driveSweepNext > 0.4)
        {
            driveSweepNext = now;
            driveServoWant += driveSweepDir * 50;
            if(driveServoWant >= driveServoMax)
            {
                driveServoWant = driveServoMax;
                driveSweepDir  = -1;
            }
            else if(driveServoWant <= driveServoMin)
            {
                driveServoWant = driveServoMin;
                driveSweepDir  = 1;
            }
            Char cmd[32];
            std::snprintf(cmd, sizeof(cmd), "SERVO %d", driveServoWant);
            sendPico(cmd);
        }
    }

    // ---- the stop, first and biggest ------------------------------------
    //
    // Above the controls rather than below them, and full width. The one thing
    // somebody reaches for without looking should not be somewhere they have to
    // find.
    ui::pushTint(ui::Tint::TINT_BAD);
    if(ui::iconButton(ui::Icon::ICON_MOTOR_STOP, "STOP",
                      ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 2.0f)))
    {
        driveSweep     = false;
        driveServoWant = 1500;
        driveEscWant   = 1500;
        sendPico("STOP");
    }
    ui::popTint(ui::Tint::TINT_BAD);
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(
            "The ESC to neutral and disarmed, and the steering RELEASED.\n"
            "\n"
            "Released, not centred. Centre is only a safe place to leave a\n"
            "servo if 1500 us is where the linkage wants to sit - if the horn\n"
            "is a tooth off its spline it is not, and centring would just be\n"
            "pushing somewhere else. Nothing to push with is the only stop\n"
            "that works on every car.\n"
            "\n"
            "Not slewed. A stop that eases in is not a stop.");
    }

    ImGui::Spacing();

    // ---- the car ---------------------------------------------------------
    //
    // What the numbers below already say, in the form somebody standing next to
    // a powered car can read without converting anything.
    {
        // The well is NARROWER than the panel and centred. Full width left the
        // car adrift in a metre of empty black, which reads as a rendering
        // fault rather than as a diagram - an instrument is the size of the
        // thing it shows, not the size of the space it was given.
        const Float32 full = ImGui::GetContentRegionAvail().x;
        const Float32 want = 460.0f * uiDpiScale;
        const Float32 caw  = (full < want) ? full : want;
        const Float32 cah  = 250.0f * uiDpiScale;

        const ImVec2 here = ImGui::GetCursorScreenPos();
        const ImVec2 cp0(here.x + ((full - caw) * 0.5f), here.y);

        // The throttle as a fraction of ITS range, which is what the rear
        // wheels warm with. Guarded: the range collapses to nothing while
        // limits are being edited, and dividing by it would light the wheels
        // on a car that is doing nothing.
        const Int32   span  = driveEscMax - 1500;
        const Float32 power = (span > 0)
            ? (static_cast<Float32>(driveEscT - 1500) / static_cast<Float32>(span))
            : 0.0f;

        drawChassis(ImGui::GetWindowDrawList(), cp0, caw, cah,
                    driveSteerNow, power, driveServoOn, driveArmed,
                    boardLamp, boardLampPin);

        ImGui::Dummy(ImVec2(full, cah));
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "The car from above, drawn from what the BOARD reports.\n"
                "\n"
                "Front wheels turn with the steering. They go dark when the\n"
                "servo is released, because a released servo holds nothing and\n"
                "the wheels are wherever the ground last left them - drawing\n"
                "them straight would be the display inventing a fact.\n"
                "\n"
                "Rear wheels warm with throttle. They do not spin: nothing on\n"
                "this car measures speed yet, so brightness is a COMMAND and\n"
                "not a reading, and that difference is worth being able to see.");
        }
    }

    ImGui::Spacing();

    // ---- how fast anything is allowed to move ---------------------------
    //
    // A slider rather than presets. Four named buttons tell you four points and
    // hide the rest of the range; the useful rate for a given job is somewhere
    // between them and the only way to find it is to move it and watch.
    //
    // LOGARITHMIC, because the interesting part is the bottom. 1 to 20 is where
    // the difference between "creeps" and "moves" lives, and on a linear scale
    // that entire question is the first tenth of the track.
    {
        driveSection("Response", "how fast an output may move");

        // The reading goes on the HEAD's line rather than beside the slider,
        // which is what lets the slider run to the shared right edge.
        {
            const Int32 perSec = driveSlew * 50;
            const Int32 travel = driveServoMax - driveServoMin;

            Char r[64];
            std::snprintf(r, sizeof(r), "%d us/s   lock to lock %.2f s",
                          perSec,
                          (perSec > 0) ? (static_cast<Float64>(travel) / perSec)
                                       : 0.0);
            driveReading(ui::sem::MUTED, r);
        }

        ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
        if(ImGui::SliderInt("##slew", &driveSlewWant, 1, 200, "%d us/tick",
                            ImGuiSliderFlags_Logarithmic))
        {
            Char cmd[32];
            std::snprintf(cmd, sizeof(cmd), "SLEW %d", driveSlewWant);
            sendPico(cmd);
        }
        driveSlewHeld = ImGui::IsItemActive();

        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "Microseconds of pulse the board may move an output, per\n"
                "20 ms tick. It governs the steering AND the throttle.\n"
                "\n"
                "The scale is logarithmic: the interesting range is the\n"
                "bottom, where the difference between creeping and moving\n"
                "lives, and a linear track would bury it in the first tenth.\n"
                "\n"
                "  ~2    creeps - slow enough to stop the moment a linkage\n"
                "        binds, which is what finding an end stop wants\n"
                "  8     the default. A slider dragged end to end sweeps\n"
                "        rather than flinging the servo at a stop\n"
                "  ~40   lock to lock in about a fifth of a second, quick\n"
                "        enough to correct a line\n"
                "  200   faster than the servo can physically follow, so the\n"
                "        limit stops being this software and starts being\n"
                "        the hardware\n"
                "\n"
                "Not saved by itself - Write to firmware, under Throttle\n"
                "range, is what keeps it across a reflash.");
        }

    }


    // ---- when the tail lamps go out ------------------------------------
    //
    // Beside Response because it is the same KIND of thing: a judgement about
    // where a boundary sits, found by watching the car rather than measured off
    // it. Idle is the pulse at which the motor sits still; this is how far past
    // that counts as actually going somewhere.
    {
        driveSection("Tail lamps", "how much throttle counts as moving");

        {
            const Bool lit = (boardLamp[2] > 0) || (boardLamp[3] > 0);
            driveReading(lit ? ui::sem::BAD : ui::sem::MUTED,
                         lit ? "lit" : "dark");
        }

        ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
        if(ImGui::SliderInt("##lightsoff", &lightsOffWant, 0, 60, "%d us past idle"))
        {
            Char cmd[40];
            std::snprintf(cmd, sizeof(cmd), "LIGHTS OFFAT %d", lightsOffWant);
            sendPico(cmd);
        }
        lightsOffHeld = ImGui::IsItemActive();

        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "The tail lamps are lit whenever the car is not being driven,\n"
                "and go out once the throttle clears idle by this much.\n"
                "\n"
                "Zero means the lamps go out the instant the throttle leaves\n"
                "idle, which lights them off for a car that has not really\n"
                "pulled away. Wind it up until they stay on at a crawl and go\n"
                "out when the car actually goes somewhere.\n"
                "\n"
                "Mirrored for reverse: this far BELOW neutral counts as being\n"
                "driven backwards, and lights the reverse lamps instead.\n"
                "\n"
                "Idle here is %d us and full throttle is %d, so the whole\n"
                "usable range is %d us wide.\n"
                "\n"
                "Not saved to the board's calibration by itself - Write to\n"
                "firmware, under Throttle range, is what makes it stick.",
                driveEscMin, driveEscMax, driveEscMax - driveEscMin);
        }

    }

    ImGui::Spacing();
    // ---- steering --------------------------------------------------------
    driveSection("Steering", "GP0");

    // Engaging is a deliberate act, the same shape as arming the ESC. The board
    // comes up released and stays that way until asked, because driving neutral
    // at power-on assumes 1500 us is somewhere safe - and on a car whose horn is
    // a tooth out, the servo picks up the frame before anyone types anything.
    if(driveServoOn)
    {
        ui::pushTint(ui::Tint::TINT_WARN);
        if(ui::iconButton(ui::Icon::ICON_MOTOR_STOP, "Release the servo",
                          ImVec2(240.0f * uiDpiScale, 0.0f)))
        {
            driveSweep = false;
            sendPico("SERVO OFF");
        }
        ui::popTint(ui::Tint::TINT_WARN);
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "Stops the pulse. The servo goes limp - no holding torque, no\n"
                "current, nothing to push against.\n"
                "\n"
                "This is the thing to reach for when it is leaning on the\n"
                "frame. A better number will not help; not being asked to hold\n"
                "a position at all is what helps.");
        }

        ImGui::SameLine();
        driveLamp(true, ui::sem::WARN, "driving");
    }

    else
    {
        if(ui::iconButton(ui::Icon::ICON_MOTOR_RUN, "Engage the servo",
                          ImVec2(240.0f * uiDpiScale, 0.0f)))
        {
            sendPico("SERVO ON");
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "Starts driving GP0, from neutral, slewed.\n"
                "\n"
                "Everything below is remembered while released and takes\n"
                "effect when you engage - so you can set a value first and\n"
                "then commit to it, rather than the other way round.");
        }

        ImGui::SameLine();
        driveLamp(false, ui::sem::MUTED, "released  -  the servo is limp");

        // Drawn as a band across the controls it applies to, because the thing
        // being explained is that ALL of them are inert. A note beside the
        // button would explain the button.
        // ONE line, and a rule down the left rather than a box round it.
        //
        // It was a two-line placard with a full border, and it is on screen
        // whenever the servo is released - which is most of the time. A notice
        // that is always up and shouts stops being read; the same words at one
        // line with a coloured edge still catch the eye and stop being the
        // loudest thing in the panel. The detail moved to the tooltip on the
        // control it is actually about.
        const ImVec2  a  = ImGui::GetCursorScreenPos();
        const Float32 bh = ImGui::GetTextLineHeight() + 6.0f * uiDpiScale;
        ImDrawList*   dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(a, ImVec2(a.x + 3.0f * uiDpiScale, a.y + bh),
                          ui::sem::WARN, 0.0f);

        ImGui::Dummy(ImVec2(0.0f, 3.0f * uiDpiScale));
        ImGui::Indent(10.0f);
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                           "Released - the controls below set a target only.");
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip(
                "The slider and the steps write a target that the board\n"
                "remembers. Nothing reaches the servo until you engage it,\n"
                "at which point it walks to that target at the Response\n"
                "rate rather than jumping.");
        }
        ImGui::Unindent(10.0f);
        ImGui::Dummy(ImVec2(0.0f, 3.0f * uiDpiScale));
    }

    ImGui::Spacing();

    // ---- steering, as a fraction of THIS car's travel -------------------
    ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
    if(ImGui::SliderFloat("##steer", &driveSteerWant, -1.0f, 1.0f, "%+.2f"))
    {
        driveSweep = false;
        Char cmd[32];
        std::snprintf(cmd, sizeof(cmd), "STEER %.3f",
                      static_cast<Float64>(driveSteerWant));
        sendPico(cmd);
    }
    driveSteerHeld = ImGui::IsItemActive();
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(
            "-1 is full lock one way, +1 the other, 0 is wheels straight.\n"
            "\n"
            "The two sides are scaled SEPARATELY, from the calibration. This\n"
            "car throws %d us one way from centre and %d the other, so half\n"
            "left and half right are not the same number of microseconds -\n"
            "and anything that added a fixed amount to a midpoint would pull\n"
            "to one side every time it was asked for half.\n"
            "\n"
            "This is the command the autonomy layer should use. Nothing above\n"
            "the calibration needs to know what a microsecond is.",
            driveServoC - driveServoMin, driveServoMax - driveServoC);
    }

    ImGui::SameLine();
    if(ui::button("Straight", ImVec2(-FLT_MIN, 0.0f)))
    {
        driveSweep     = false;
        driveSteerWant = 0.0f;
        sendPico("STEER 0");
    }

    ImGui::Spacing();
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                       "Raw microseconds  -  for calibrating, not for driving");

    ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
    if(ImGui::SliderInt("##servo", &driveServoWant,
                        driveServoMin, driveServoMax, "%d us"))
    {
        Char cmd[32];
        std::snprintf(cmd, sizeof(cmd), "SERVO %d", driveServoWant);
        sendPico(cmd);
    }
    driveServoHeld = ImGui::IsItemActive();
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(
            "1500 us is centre. The range is %d-%d, which the BOARD sets.\n"
            "\n"
            "Deliberately narrower than the servo's own 1000-2000: a TT-02's\n"
            "steering binds against its linkage well before the servo's limits,\n"
            "and a servo pushing a stop stalls and cooks itself.\n"
            "\n"
            "Widen it in firmware once the real end stops are known - with the\n"
            "servo horn OFF, so being wrong costs nothing.",
            driveServoMin, driveServoMax);
    }

    ImGui::SameLine();
    if(ui::button("Centre", ImVec2(-FLT_MIN, 0.0f)))
    {
        driveSweep     = false;
        driveServoWant = 1500;
        sendPico("SERVO CENTER");
    }

    // ---- exact values, and steps ----------------------------------------
    //
    // A slider is for feeling out where things are; a number is for saying
    // exactly where. Calibration needs both, and reading an end stop off a
    // slider handle is not reading it.
    const auto nudge = [](Int32 by)
    {
        driveSweep      = false;
        driveServoWant += by;
        driveServoWant  = clampInt(driveServoWant, driveServoMin, driveServoMax);
        Char cmd[32];
        std::snprintf(cmd, sizeof(cmd), "SERVO %d", driveServoWant);
        sendPico(cmd);
    };

    const Float32 bw = 46.0f * uiDpiScale;
    for(Size i = 0; i < countOf(SERVO_STEPS); ++i)
    {
        if(i > 0)
        {
            ImGui::SameLine(0.0f, 3.0f);
        }
        if(ui::button(SERVO_STEPS[i].label, ImVec2(bw, 0.0f)))
        {
            nudge(SERVO_STEPS[i].by);
        }
        if(!driveServoOn && ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Sets the target. The servo is released, so\n"
                              "nothing moves until you engage it.");
        }
    }

    ImGui::SameLine(0.0f, 12.0f);
    ImGui::BeginDisabled(!driveServoOn);
    if(ui::segmentedIconButton(driveSweep ? ui::Icon::ICON_PAUSE
                                          : ui::Icon::ICON_PLAY,
                               "Sweep", driveSweep,
                               ImVec2(110.0f * uiDpiScale, 0.0f)))
    {
        driveSweep     = !driveSweep;
        driveSweepNext = ImGui::GetTime();
    }
    ImGui::EndDisabled();
    if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        ImGui::SetTooltip(
            "Walks the servo between its limits, back and forth.\n"
            "\n"
            "For watching the linkage move through its whole travel and seeing\n"
            "whether anything binds or fouls before you trust a number.\n"
            "\n"
            "Driven from the hub, not the board, and it stops the moment this\n"
            "view is not on screen - a servo cycling behind a tab nobody is\n"
            "looking at is exactly what should not be possible.");
    }

    // What the board is actually OUTPUTTING, which lags the slider while the
    // slew runs. Showing both is what makes the ramp visible rather than
    // looking like lag. Directly under the buttons that move it: the limits
    // block below expands, and this must not be pushed away by that.
    // ONE status line for the whole steering section.
    //
    // There were three, and between them 1484 appeared four times in five
    // lines: the fraction, the raw slider, and this. A number repeated is a
    // number you stop reading.
    {
        Char st[96];
        if(driveServoOn)
        {
            std::snprintf(st, sizeof(st), "%+.2f   %d us   target %d us",
                          static_cast<Float64>(driveSteer), driveServo, driveServoT);
        }
        else
        {
            std::snprintf(st, sizeof(st), "%+.2f   target %d us   not driven",
                          static_cast<Float64>(driveSteer), driveServoT);
        }
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED), "%s", st);
    }

    // ---- the calibration -------------------------------------------------
    //
    // Three named points instead of a min/max pair, because a car has three
    // interesting positions and only two of them are ends. Centre used to be
    // assumed to be 1500 and that assumption is what put a servo against a
    // frame - it is a measurement now, like the other two.
    if(ImGui::TreeNode("Calibration  -  the three numbers for THIS car"))
    {
        if(!calLoaded)
        {
            loadCalibration();
        }

        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                           "Servo horn OFF while you find these.");
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                           "Engage, step until the output shaft reaches each end,\n"
                           "and press Set to here. With the horn off, being wrong\n"
                           "costs nothing at all.");
        ImGui::Spacing();

        calRow("Max left", "Full lock one way. Where the STEERING stops, which is\n"
                           "not where the servo stops - the linkage binds first,\n"
                           "and the servo will happily keep pushing past it.",
               &calLeft);
        calRow("Centre",   "Wheels straight ahead.\n"
                           "\n"
                           "The one that matters most and the one nobody measures.\n"
                           "1500 us is the middle of the SERVO's range and says\n"
                           "nothing about the CAR's - the horn only meets its\n"
                           "spline at whole-tooth intervals, so straight-ahead\n"
                           "lands wherever it lands.",
               &calCenter);
        calRow("Max right", "Full lock the other way.", &calRight);

        ImGui::Spacing();

        const Bool ordered = (calLeft < calCenter && calCenter < calRight);
        if(!ordered)
        {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::BAD),
                               "Left must be below centre, and centre below right.");
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                               "If your steering runs the other way, swap which end\n"
                               "you call left - the firmware only needs the order.");
        }

        ImGui::BeginDisabled(!ordered || !driveKnown);
        if(ui::iconButton(ui::Icon::ICON_SEND, "Send to the board",
                          ImVec2(200.0f * uiDpiScale, 0.0f), ui::Tint::TINT_WARN))
        {
            Char cmd[48];
            std::snprintf(cmd, sizeof(cmd), "SERVOLIMITS %d %d", calLeft, calRight);
            sendPico(cmd);
            std::snprintf(cmd, sizeof(cmd), "SERVOTRIM %d", calCenter);
            sendPico(cmd);
        }
        ImGui::EndDisabled();
        if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        {
            ImGui::SetTooltip("Applies these to the RUNNING board so you can try\n"
                              "them immediately.\n"
                              "\n"
                              "Lost on the next reboot - Write to firmware is what\n"
                              "makes them stick.");
        }

        ImGui::SameLine();
        ImGui::BeginDisabled(!ordered);
        if(ui::iconButton(ui::Icon::ICON_SAVE, "Write to firmware",
                          ImVec2(210.0f * uiDpiScale, 0.0f)))
        {
            const Str path = steeringCalPath();
            const Str text = steeringCalText();
            Str       err;
            if(path.empty())
            {
                LOG_WARN("drive", "no firmware directory - nothing written");
            }
            else if(sketch::save(path, text, err))
            {
                calWritten = text;
                calDirty   = false;
                LOG_INFO("drive", "wrote %s", path.c_str());
            }
            else
            {
                LOG_WARN("drive", "could not write cal.h: %s",
                         err.c_str());
            }
        }
        ImGui::EndDisabled();
        if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        {
            ImGui::SetTooltip("Writes firmware/lib/chassis/cal.h - the three\n"
                              "steering numbers AND the throttle range the\n"
                              "board is currently using.\n"
                              "\n"
                              "main.c includes it, so these become the limits and\n"
                              "the centre the board comes up with. Reflash after\n"
                              "writing or the board keeps running the old ones.");
        }

        // Whether what is on screen matches what the firmware would build with.
        // Silence here would mean "written" and "typed but not written" look
        // identical, which is the whole problem generated files have.
        {
            Int32 fl = 0;
            Int32 fc = 0;
            Int32 fr = 0;
            if(calWritten.empty()
               || !readCalNumbers(calWritten, fl, fc, fr))
            {
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                                   "cal.h could not be read.");
            }
            else if(fl != calLeft || fc != calCenter || fr != calRight)
            {
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                                   "cal.h still says %d / %d / %d.",
                                   fl, fc, fr);
            }
            else if(Int32 tl = 0, th = 0;
                    !readThrottleNumbers(calWritten, tl, th)
                    || tl != driveEscMin || th != driveEscMax)
            {
                // The steering matches and the throttle does not, which is what
                // a header written before the throttle was persisted looks
                // like. Worth its own sentence rather than a silent green tick.
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                                   "Steering matches; the throttle range in "
                                   "cal.h does not.");
            }
            else
            {
                ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::GOOD),
                                   "Matches cal.h - the firmware would "
                                   "build with these.");
            }
        }

        if(ui::iconButton(ui::Icon::ICON_CODE, "Open cal.h",
                          ImVec2(210.0f * uiDpiScale, 0.0f)))
        {
            const Str path = steeringCalPath();
            if(!path.empty())
            {
                openCodeFile(path, "cal.h");
                forceView       = 3;
                forceViewFrames = 4;
            }
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Opens the generated header in the Code view, so\n"
                              "the numbers are readable as code rather than only\n"
                              "as boxes in a panel.");
        }

        if(driveKnown)
        {
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                               "The board is currently using  %d / %d / %d.",
                               driveServoMin, driveServoC, driveServoMax);
        }

        ImGui::TreePop();
    }

    // ---- the raw limits, still reachable ---------------------------------
    //
    // Kept because finding an end stop means pushing PAST where you think it
    // is, and the calibration above will not let you: it clamps to what the
    // board already accepts. This is the way out of that circle.
    if(ImGui::TreeNode("Limits  -  widen to find the real end stops"))
    {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                           "Take the servo horn OFF first.");
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                           "The linkage binds before the servo's own limits do,\n"
                           "and a servo pushing a stop stalls and cooks itself.\n"
                           "With the horn off, being wrong costs nothing.");
        ImGui::Spacing();

        if(!driveLimitsDirty)
        {
            driveLimitLo = driveServoMin;
            driveLimitHi = driveServoMax;
        }

        ImGui::SetNextItemWidth(120.0f * uiDpiScale);
        if(ImGui::InputInt("min us", &driveLimitLo, 10, 50))
        {
            driveLimitsDirty = true;
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f * uiDpiScale);
        if(ImGui::InputInt("max us", &driveLimitHi, 10, 50))
        {
            driveLimitsDirty = true;
        }

        ImGui::BeginDisabled(!driveLimitsDirty);
        if(ui::iconButton(ui::Icon::ICON_SAVE, "Apply to the board",
                          ImVec2(220.0f * uiDpiScale, 0.0f), ui::Tint::TINT_WARN))
        {
            Char cmd[48];
            std::snprintf(cmd, sizeof(cmd), "SERVOLIMITS %d %d",
                          driveLimitLo, driveLimitHi);
            sendPico(cmd);
            driveLimitsDirty = false;
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        if(ui::button("Reset", ImVec2(100.0f * uiDpiScale, 0.0f)))
        {
            sendPico("SERVOLIMITS 1300 1700");
            driveLimitsDirty = false;
        }

        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                           "The board clamps to 1000-2000 whatever is asked, and\n"
                           "pulls the current target and centre back inside a\n"
                           "narrowed range.\n"
                           "\n"
                           "Widen here to go looking; record what you find in the\n"
                           "Calibration block above, which is what gets written\n"
                           "into the firmware.");
        ImGui::TreePop();
    }

    // ---- throttle --------------------------------------------------------
    driveSection("Throttle", "GP1  (ESC)");

    // The percentage belongs up here with the other readings, not stranded
    // under the power bar where it was the only thing on its line.
    {
        const Int32   span = driveEscMax - driveEscMin;
        const Float32 frac = (span > 0)
            ? (static_cast<Float32>(driveEsc - driveEscMin)
               / static_cast<Float32>(span))
            : 0.0f;

        Char r[32];
        std::snprintf(r, sizeof(r), "%.0f%%",
                      static_cast<Float64>((driveArmed ? frac : 0.0f) * 100.0f));
        driveReading(driveArmed ? ui::sem::WARN : ui::sem::MUTED, r);
    }

    // Arming is a separate, deliberate act. The slider does nothing until it
    // happens, and the board refuses throttle commands regardless of what this
    // checkbox says - this is the reminder, not the enforcement.
    Bool arm = driveArmed;
    if(ui::checkbox("Arm the ESC", &arm))
    {
        sendPico(arm ? "ESC ARM" : "ESC DISARM");
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(
            "The ESC ignores every throttle command until this is on.\n"
            "\n"
            "BEFORE ARMING:\n"
            "  - the car on a stand, wheels off the ground\n"
            "  - common ground between the Pico and the ESC (mandatory)\n"
            "  - the BEC 5 V NOT connected while USB is\n"
            "\n"
            "Enforced on the BOARD, not here. This checkbox is the reminder.");
    }

    ImGui::BeginDisabled(!driveArmed);
    ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
    if(ImGui::SliderInt("##esc", &driveEscWant,
                        driveEscMin, driveEscMax, "%d us"))
    {
        Char cmd[32];
        std::snprintf(cmd, sizeof(cmd), "ESC %d", driveEscWant);
        sendPico(cmd);
    }
    driveEscHeld = ImGui::IsItemActive();
    ImGui::EndDisabled();

    if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    {
        ImGui::SetTooltip(
            "1500 us is neutral. The range is %d-%d - forward only, and barely.\n"
            "\n"
            "1600 is a crawl on a bench. Reverse is not offered at all: a\n"
            "QuicRun needs a brake-then-reverse sequence, and getting that\n"
            "wrong on a stand is how a gearbox meets a workbench.\n"
            "\n"
            "The board ramps toward whatever you set rather than jumping to it.",
            driveEscMin, driveEscMax);
    }

    ImGui::SameLine();
    if(ui::button("Neutral", ImVec2(-FLT_MIN, 0.0f)))
    {
        driveEscWant = 1500;
        sendPico("ESC NEUTRAL");
    }

    // Throttle steps are smaller than the servo's. A motor that jumps 50 us is
    // a motor that has already spun up by the time you decide it was too much.
    const auto nudgeEsc = [](Int32 by)
    {
        driveEscWant += by;
        driveEscWant  = clampInt(driveEscWant, driveEscMin, driveEscMax);
        Char cmd[32];
        std::snprintf(cmd, sizeof(cmd), "ESC %d", driveEscWant);
        sendPico(cmd);
    };

    // A power bar, because a microsecond figure is not a sense of how much.
    //
    // Segmented rather than smooth: a continuous bar invites reading a
    // precision that is not there. Ten blocks of the throttle's own range say
    // "about a third" without pretending to say more.
    {
        const Float32 barW = ImGui::GetContentRegionAvail().x - (120.0f * uiDpiScale);
        const Float32 barH = 10.0f * uiDpiScale;
        const ImVec2  b0   = ImGui::GetCursorScreenPos();
        ImDrawList*   dl   = ImGui::GetWindowDrawList();

        const Int32   span = driveEscMax - 1500;
        const Float32 frac = (span > 0)
            ? (static_cast<Float32>(driveEsc - 1500) / static_cast<Float32>(span))
            : 0.0f;

        dl->AddRectFilled(b0, ImVec2(b0.x + barW, b0.y + barH),
                          IM_COL32(0x14, 0x15, 0x18, 0xFF), 2.0f);

        constexpr Int32 SEGS = 10;
        const Float32 segW = (barW - (2.0f * uiDpiScale) * (SEGS - 1)) / SEGS;
        for(Int32 i = 0; i < SEGS; ++i)
        {
            const Float32 at = static_cast<Float32>(i + 1) / SEGS;
            const Bool    on = driveArmed && (frac >= at - (0.5f / SEGS));
            const Float32 x  = b0.x + (i * (segW + 2.0f * uiDpiScale));

            // The top of the range is amber: the last blocks are the ones worth
            // noticing before pressing anything else.
            const ImU32 lit = (i >= SEGS - 3) ? ui::sem::WARN : ui::sem::GOOD;
            dl->AddRectFilled(ImVec2(x, b0.y), ImVec2(x + segW, b0.y + barH),
                              on ? lit : IM_COL32(0x26, 0x28, 0x2C, 0xFF), 1.0f);
        }
        ui::screenInset(b0, ImVec2(b0.x + barW, b0.y + barH), 0.7f);

        ImGui::Dummy(ImVec2(barW, barH));
    }

    ImGui::BeginDisabled(!driveArmed);
    for(Size i = 0; i < countOf(ESC_STEPS); ++i)
    {
        if(i > 0)
        {
            ImGui::SameLine(0.0f, 3.0f);
        }
        // Suffixed: several of these labels repeat the servo row's, and ImGui
        // derives a widget's identity from its label.
        if(ui::button(ESC_STEPS[i].label, ImVec2(bw, 0.0f)))
        {
            nudgeEsc(ESC_STEPS[i].by);
        }
    }
    ImGui::EndDisabled();

    {
        Char line[64];
        std::snprintf(line, sizeof(line), "output %d us   target %d us   %s",
                      driveEsc, driveEscT, driveArmed ? "ARMED" : "disarmed");
        driveLamp(driveArmed, driveArmed ? ui::sem::BAD : ui::sem::MUTED, line);
    }

    if(ImGui::TreeNode("Throttle range  -  widen once the car is on a stand"))
    {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                           "Wheels off the ground before touching this.");
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                           "The board will not go above 1700 whatever is asked,\n"
                           "and reverse stays unreachable - a QuicRun needs a\n"
                           "brake-then-reverse sequence, and getting that wrong on\n"
                           "a stand is how a gearbox meets a workbench.");
        ImGui::Spacing();

        if(!driveEscLimitsDirty)
        {
            driveEscLimitLo = driveEscMin;
            driveEscLimitHi = driveEscMax;
        }

        ImGui::SetNextItemWidth(120.0f * uiDpiScale);
        if(ImGui::InputInt("min us##esc", &driveEscLimitLo, 10, 50))
        {
            driveEscLimitsDirty = true;
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f * uiDpiScale);
        if(ImGui::InputInt("max us##esc", &driveEscLimitHi, 10, 50))
        {
            driveEscLimitsDirty = true;
        }

        ImGui::BeginDisabled(!driveEscLimitsDirty);
        if(ui::iconButton(ui::Icon::ICON_SAVE, "Apply to the board##esc",
                          ImVec2(220.0f * uiDpiScale, 0.0f), ui::Tint::TINT_WARN))
        {
            Char cmd[48];
            std::snprintf(cmd, sizeof(cmd), "ESCLIMITS %d %d",
                          driveEscLimitLo, driveEscLimitHi);
            sendPico(cmd);
            driveEscLimitsDirty = false;
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        if(ui::button("Reset##esc", ImVec2(100.0f * uiDpiScale, 0.0f)))
        {
            sendPico("ESCLIMITS 1500 1600");
            driveEscLimitsDirty = false;
        }
        ImGui::TreePop();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                       "Nothing here jumps. The board walks each output toward\n"
                       "its target a few microseconds at a time, so a slider\n"
                       "dragged end to end produces a sweep rather than a step.");

    // ---- the wiring this view assumes ------------------------------------
    //
    // Written down because a signal wire in the wrong hole looks exactly like
    // firmware that does not work, and the two are debugged very differently.
    if(ImGui::TreeNode("Wiring"))
    {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                           "  servo signal  ->  GP0\n"
                           "  ESC signal    ->  GP1\n"
                           "  ESC ground    ->  a Pico GND  (mandatory)\n"
                           "\n"
                           "The ESC's BEC 5 V goes NOWHERE while USB is plugged\n"
                           "in - two supplies fighting over one rail is how a\n"
                           "Pico stops being a Pico.\n"
                           "\n"
                           "Nothing else is assumed connected. This view does not\n"
                           "need the display or the ToF sensor.");
        ImGui::TreePop();
    }

    ui::screenInset(drivePanel0, ImVec2(drivePanel0.x + w, drivePanel0.y + h));
    ImGui::EndChild();
}

Void drawViewBody(Int32 view, Float32 w, Float32 h)
{
    const ImVec2 p0 = ImGui::GetCursorScreenPos();

    // Explicitly black, matching every other surface. An explicit push rather
    // than inherited, because the map is the one panel whose background must
    // never pick up a tint or an alpha - it is the surface the point cloud is
    // read against.
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::ansi::BLACK);
    ImGui::PushID(view);

    if(view == 0 || view == 1)
    {
        // is3D is set immediately before the draw, so both maps can be on
        // screen at once in the floating layout: each draw sees the flag it
        // needs, and the 2D and 3D state they read live in separate members.
        radarView.is3D = (view == 1);

        ImGui::BeginChild("##map", ImVec2(w, h), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        radarView.draw(ImGui::GetContentRegionAvail());
        drawMapHud(p0, ImVec2(w, h));
        ImGui::GetWindowDrawList()->AddRect(
            p0, ImVec2(p0.x + w, p0.y + h),
            IM_COL32(0x3A, 0x3A, 0x3A, 0xFF), 0.0f, 0, 1.0f);
        ImGui::EndChild();
    }
    else if(view == 2)
    {
        ImGui::BeginChild("##recmap", ImVec2(w, h), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        recView.draw(ImGui::GetContentRegionAvail());
        drawRecorderHud(p0, ImVec2(w, h));
        ImGui::GetWindowDrawList()->AddRect(
            p0, ImVec2(p0.x + w, p0.y + h),
            IM_COL32(0x3A, 0x3A, 0x3A, 0xFF), 0.0f, 0, 1.0f);
        ImGui::EndChild();
    }
    else if(view == 3)
    {
        // Loaded on first sight rather than at startup: reading the sketch
        // library costs a directory scan, and most sessions never open it.
        if(!codeLoaded)
        {
            codeLoaded = true;
            const Vec<Str> have = sketch::list();
            if(!have.empty())
            {
                codeName = have.front();
                codePath = sketch::pathOf(codeName);
                codeEditor.setText(sketch::load(codePath));
            }
            else
            {
                codeName = sketch::makeName();
                codePath = sketch::pathOf(codeName);
                codeEditor.setText(sketch::starter());
            }
        }

        // Tree | splitter | editor. The tree's width is the user's, clamped so
        // a drag can neither squeeze the editor away nor strand the tree at a
        // width with no grabbable edge.
        const Float32 splitW = std::max(ImGui::GetStyle().ItemSpacing.x,
                                        8.0f * uiDpiScale);

        const Float32 treeW = codeTreeCollapsed
            ? ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.x * 2.0f
            : std::max(CODE_TREE_MIN_W * uiDpiScale,
                       std::min(codeTreeLogicalW * uiDpiScale,
                                std::min(CODE_TREE_MAX_W * uiDpiScale, w * 0.6f)));

        drawCodeTree(treeW, h);
        ImGui::SameLine(0.0f, 0.0f);

        if(codeTreeCollapsed)
        {
            // No handle when there is nothing to resize, but the same gap so
            // the editor does not shift by eight pixels on collapse.
            ImGui::SetCursorScreenPos(ImVec2(p0.x + treeW, p0.y));
            ImGui::Dummy(ImVec2(splitW, h));
        }
        else
        {
            codeTreeSplitter(ImVec2(p0.x + treeW, p0.y), h, splitW);
        }
        ImGui::SameLine(0.0f, 0.0f);

        ui::drawCode(codeView, codeEditor,
                     ImVec2(std::max(120.0f, w - treeW - splitW), h),
                     ImGui::GetTime());
        handleCodeCommand();
    }
    else if(view == DRIVE_VIEW)
    {
        // Polled inside the body, so the fast LIGHTS poll only runs while
        // somebody is actually watching the lamps. TEMPORARY.
        pollLights();
        drawDriveBody(w, h);
    }
    else if(view == RANGE_VIEW)
    {
        // Polled only while this view is drawn - which is to say, only while
        // somebody is looking at it. See pollTof().
        pollSensorList();
        pollTof(true);
        drawRangeBody(w, h);
    }
    else
    {
        // Reference is the terminal branch, so an index from a stale settings
        // file lands on a real view rather than on nothing.
        ImGui::BeginChild("##ref", ImVec2(w, h), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar
                          | ImGuiWindowFlags_NoScrollWithMouse);
        const ImVec2 rp0 = ImGui::GetCursorScreenPos();
        ref::draw(refView, ImGui::GetContentRegionAvail());
        ui::screenInset(rp0, ImVec2(rp0.x + w, rp0.y + h));
        ImGui::EndChild();
    }

    ImGui::PopID();
    ImGui::PopStyleColor();
}

// The view's name and icon, for both the tab bar and the panel title bars.
const Char* viewName(Int32 view)
{
    switch(view)
    {
    case 0:  return "2D";
    case 1:  return "3D";
    case 2:  return "Record";
    case 3:  return "Code";
    default: break;
    }
    if(view == REF_VIEW)
    {
        return "Reference";
    }
    if(view == RANGE_VIEW)
    {
        return "Range";
    }
    if(view == DRIVE_VIEW)
    {
        return "Drive";
    }
    return "Reference";
}

ui::Icon viewIcon(Int32 view)
{
    switch(view)
    {
    case 0:  return ui::Icon::ICON_DIM_2D;
    case 1:  return ui::Icon::ICON_DIM_3D;
    case 2:  return ui::Icon::ICON_RECORD;
    case 3:  return ui::Icon::ICON_CODE;
    default: break;
    }
    if(view == RANGE_VIEW)
    {
        return ui::Icon::ICON_TOF;
    }
    if(view == DRIVE_VIEW)
    {
        return ui::Icon::ICON_SERVO;
    }
    return ui::Icon::ICON_REFERENCE;
}

// ===================================================== the tabbed layout

Void drawTabbedViews(Float32 mapW, Float32 viewH)
{
    if(!ImGui::BeginTabBar("##central", ImGuiTabBarFlags_None))
    {
        return;
    }

    const auto viewSel = [](Int32 which)
    {
        return (forceView == which && forceViewFrames > 0)
             ? ImGuiTabItemFlags_SetSelected : 0;
    };
    if(forceViewFrames > 0)
    {
        --forceViewFrames;
    }

    const Int32 total = VIEW_COUNT;
    for(Int32 v = 0; v < total; ++v)
    {
        Char label[48];
        iconTabLabel(label, sizeof(label), viewName(v));

        const Bool open = ImGui::BeginTabItem(label, nullptr, viewSel(v));
        tabIcon(viewIcon(v));
        if(open)
        {
            centralView = v;
            wsFocused   = v;
            drawViewBody(v, mapW, viewH);
            ImGui::EndTabItem();
        }
    }

    // Right-aligned, so opening the console is where the tabs END rather than
    // in a second row of chrome above them. The same place the old layout
    // switch used to live, for the same reason.
    if(ImGui::TabItemButton(consoleOpen ? "  Console  <  " : "  >  Console  ",
                            ImGuiTabItemFlags_Trailing
                            | ImGuiTabItemFlags_NoTooltip))
    {
        consoleOpen      = !consoleOpen;
        panelLayoutDirty = true;
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("The serial and build logs, in a column on the left.\n"
                          "Ctrl+` toggles it.");
    }

    ImGui::EndTabBar();
}

// The central region. The map is one view among several rather than the only
// one, but it is still the default and still where the app lands.
Void drawMapRegion(Float32 mapW, Float32 mapH, Float32 ctrlH)
{
    const Float32 headH = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.y;
    const Float32 viewH = mapH - headH;

    drawTabbedViews(mapW, viewH);

    // The bottom bar belongs to the VIEW above it, not to the central region.
    // Points/Rays/Density and Grid/Trail/Labels configure the map and nothing
    // else, so on a board tab they are not merely disabled - they are absent,
    // and the board gets the height back.
    //
    // In the floating layout it follows the FOCUSED panel, so clicking a map
    // brings up the map's controls.
    if(ctrlH > 0.0f)
    {
        ImGui::BeginChild("##controls", ImVec2(mapW, ctrlH), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        drawCentralControls(centralView);
        ImGui::EndChild();
    }
}

// ==================================================================== vehicle
// The Pico as the car's controller rather than as a serial port. Left: the link
// and its command vocabulary. Right: what the board says about servo and ESC,
// and - when it says nothing - why.

Void drawPicoLinkBlock()
{
    const ImGuiStyle& sty  = ImGui::GetStyle();
    const PicoState   st   = picoLink.state();
    const Bool        live = (st == PicoState::PICO_STATE_CONNECTED);
    const Bool        busy = live || st == PicoState::PICO_STATE_CONNECTING;
    const Float32       bh   = ImGui::GetFrameHeight() * 1.2f;

    ImGui::SeparatorText("Link");

    const Float32 refreshW = ImGui::CalcTextSize("Refresh").x + sty.FramePadding.x * 2.0f;

    ImGui::BeginDisabled(busy);
    if(picoItems.empty())
    {
        ImGui::AlignTextToFramePadding();
        colored(ui::sem::WARN, "No Pico found");
    }
    else
    {
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - refreshW - sty.ItemSpacing.x);
        ui::combo("##picoport", &picoIndex, picoItems.data(), static_cast<Int32>(picoItems.size()));
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_REFRESH, "Refresh")) refreshPicoPorts();

    if(busy)
    {
        if(ui::iconButton(ui::Icon::ICON_PLUG_DISCONNECT, "Disconnect",
                          ImVec2(-FLT_MIN, bh), ui::Tint::TINT_WARN))
            picoLink.disconnect();
    }
    else
    {
        ImGui::BeginDisabled(picoIndex < 0);
        if(ui::iconButton(ui::Icon::ICON_PLUG_CONNECT, "Connect",
                          ImVec2(-FLT_MIN, bh), ui::Tint::TINT_GOOD))
            connectPico();
        ImGui::EndDisabled();
    }

    const Str err = picoLink.error();
    if(!err.empty() && st == PicoState::PICO_STATE_UNPLUGGED)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::sem::MUTED);
        ImGui::TextWrapped("%s", err.c_str());
        ImGui::PopStyleColor();
    }
    else if(!err.empty() && st == PicoState::PICO_STATE_ERROR)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::sem::BAD);
        ImGui::TextWrapped("%s", err.c_str());
        ImGui::PopStyleColor();
    }

    if(ImGui::BeginTable("picostat", 2, ImGuiTableFlags_SizingStretchProp))
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::TextDisabled("State");
        ImGui::TableNextColumn(); colored(picoStateColor(st), "%s", picoStateText(st));

        const Str p = picoLink.port();
        keyValue("Port",    "%s",   p.empty() ? "--" : p.c_str());
        keyValue("Sent",    "%llu", picoLink.txLines());
        keyValue("Received","%llu", picoLink.rxLines());
        keyValue("Dropped", "%llu", picoLink.dropped());

        // A board that never answers has to read as deliberately silent rather
        // than broken, and the value alone carries that.
        const Float64 age = picoLink.lastRxAgeS();
        Char ageS[48];
        picoAgeText(ageS, sizeof(ageS), age);

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::TextDisabled("Last line");
        ImGui::TableNextColumn();
        colored((age >= 0.0 && age < 2.0) ? ui::sem::GOOD
                : (live ? ui::sem::WARN : ui::sem::MUTED), "%s", ageS);

        ImGui::EndTable();
    }
}

Void drawPicoCommands()
{
    const ImGuiStyle& sty  = ImGui::GetStyle();
    const Bool        live = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

    ImGui::SeparatorText("Commands");

    ImGui::BeginDisabled(!live);
    if(ImGui::BeginTable("picocmd", 3, ImGuiTableFlags_SizingStretchSame))
    {
        auto cmd = [](ui::Icon ic, const Char* label, const Char* line)
        {
            ImGui::TableNextColumn();
            if(ui::iconButton(ic, label, ImVec2(-FLT_MIN, 0.0f))) sendPico(line);
        };

        // Two vocabularies, both real. pico_debug (flashed now) answers
        // PING/ID/STATUS/HELP/LED and returns "ERR bad command" to `?`;
        // tt02_control answers only `?`. Whichever is on the board, the other
        // half of this grid comes back ERR, which is the board being correct
        // rather than anything here being broken.
        ImGui::TableNextRow();
        cmd(ui::Icon::ICON_HELP,       "?  status", "?");
        cmd(ui::Icon::ICON_LINK,       "PING",      "PING");
        cmd(ui::Icon::ICON_DEVICE,     "ID",        "ID");

        ImGui::TableNextRow();
        cmd(ui::Icon::ICON_STATUS_OK,  "STATUS",  "STATUS");
        cmd(ui::Icon::ICON_HELP,       "HELP",    "HELP");
        cmd(ui::Icon::ICON_LAMP,       "LED ON",  "LED ON");

        ImGui::TableNextRow();
        cmd(ui::Icon::ICON_STATUS_IDLE, "LED OFF",   "LED OFF");
        cmd(ui::Icon::ICON_LAMP_DIM,    "Blink 2",   "LED BLINK 2");
        cmd(ui::Icon::ICON_STATUS_IDLE, "Blink off", "LED BLINK 0");
        ImGui::EndTable();
    }

    // Free-text line. Enter sends and keeps the cursor here, which is what you
    // want when you are poking at a fresh command vocabulary.
    // Matches what iconButton auto-sizes to, so the input field ends exactly
    // where the button begins.
    const Float32 sendW = ImGui::CalcTextSize("Send").x + sty.FramePadding.x * 2.0f
                        + (ui::iconsReady()
                           ? ui::iconSize() + sty.ItemInnerSpacing.x : 0.0f);
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - sendW - sty.ItemSpacing.x);

    Bool fire = ImGui::InputTextWithHint("##picocmdline", "type a command",
                                         cmdBuf, sizeof(cmdBuf),
                                         ImGuiInputTextFlags_EnterReturnsTrue);
    if(fire) ImGui::SetKeyboardFocusHere(-1);

    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_SEND, "Send")) fire = true;

    if(fire)
    {
        sendPico(cmdBuf);
        cmdBuf[0] = '\0';
    }
    ImGui::EndDisabled();
}

// Servo and ESC pulse widths, when the firmware reports them. Nothing is
// synthesised here: if no S line has arrived the readouts say "--" and the note
// underneath says which firmware would have answered.
Void drawControllerState()
{
    ImGui::SeparatorText("Controller state");

    Char servo[24] = "--", esc[24] = "--";
    if(vehicleStatus.have)
    {
        std::snprintf(servo, sizeof(servo), "%d", vehicleStatus.servoUs);
        std::snprintf(esc,   sizeof(esc),   "%d", vehicleStatus.escUs);
    }

    // Bounded rather than stretched: at full workspace width a two-column table
    // throws its second value against the far edge and the pair stops reading
    // as a pair.
    const Float32 tableW = std::min(520.0f * uiDpiScale, ImGui::GetContentRegionAvail().x);

    if(ImGui::BeginTable("pwm", 2, ImGuiTableFlags_SizingStretchSame,
                          ImVec2(tableW, 0.0f)))
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn(); statCell(servo, "servo GP0  us");
        ImGui::TableNextColumn(); statCell(esc,   "ESC GP1  us");
        ImGui::EndTable();
    }

    ImGui::Spacing();

    // Everything the board reports about itself, and "--" for everything it
    // does not. The reply row is the only thing that says which of the two
    // firmwares is answering, and it says it in three words.
    if(ImGui::BeginTable("vehstat", 2, ImGuiTableFlags_SizingStretchProp,
                          ImVec2(tableW, 0.0f)))
    {
        const BoardStatus brd = picoFlash.board();
        keyValue("Program", "%s", brd.program.empty() ? "--" : brd.program.c_str());

        ImGui::TableNextRow();
        ImGui::TableNextColumn(); ImGui::TextDisabled("? reply");
        ImGui::TableNextColumn();
        if(vehicleStatus.have)              colored(ui::sem::GOOD, "S line");
        else if(vehUnsupported)  colored(ui::sem::WARN, "no S line");
        else                         ImGui::TextUnformatted("--");

        if(vehicleStatus.have)
        {
            keyValue("Board uptime", "%llu ms", vehicleStatus.uptimeMs);
            keyValue("Field 2",      "%ld",     vehicleStatus.a);
            keyValue("Field 3",      "%ld",     vehicleStatus.b);
            keyValue("Field 6",      "%llu ms", vehicleStatus.lastMs);
            keyValue("Reading age",  "%.1f s",  ImGui::GetTime() - vehicleStatus.seenAt);
        }
        else
        {
            keyValue("Board uptime", "--");
            keyValue("Field 2",      "--");
            keyValue("Field 3",      "--");
            keyValue("Field 6",      "--");
            keyValue("Reading age",  "--");
        }

        keyValue("Watchdog", "--");
        ImGui::EndTable();
    }
}

// The lighting bench. See lightInput for what this is and is not connected to.
Void drawLightingBench()
{
    ImGui::SeparatorText("Lighting (bench test)");

    ImGui::TextDisabled("Drives the 3D view only. Nothing is wired to the board.");
    ImGui::Spacing();

    // ---- automatic --------------------------------------------------------
    if(ui::checkbox("Automatic", &autoLights))
    {
        // Start from a clean slate rather than from whatever the lamps happened
        // to be showing: a stale brake hold or a half-finished flash carried
        // into automatic mode looks like the detector's first decision.
        autoLightState = lights::AutoState{};
        if(!autoLights)
        {
            lightInput = lights::Input{};
        }
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(
            "Work the lamps out from what the car is doing.\n\n"
            "Steering past %.0f%% of lock indicates that way and holds until it\n"
            "unwinds past %.0f%%. Any drop in throttle lights the brakes for\n"
            "%.0f ms. Armed turns the tail lights on.\n\n"
            "Both are guesses from the outputs - there is no brake pedal and no\n"
            "indicator stalk on this car, and no encoder yet, so \"braking\" means\n"
            "the throttle was reduced rather than the car actually slowed.",
            static_cast<Float64>(lights::AutoConfig{}.turnOnAbs * 100.0f),
            static_cast<Float64>(lights::AutoConfig{}.turnOffAbs * 100.0f),
            lights::AutoConfig{}.brakeHoldS * 1000.0);
    }

    if(autoLights)
    {
        ImGui::SameLine();
        const Char* say = (lightInput.turn == lights::Turn::TURN_LEFT)  ? "indicating left"
                        : (lightInput.turn == lights::Turn::TURN_RIGHT) ? "indicating right"
                        : lightInput.brake                              ? "braking"
                        : driveArmed                                    ? "running"
                                                                        : "dark";
        colored(lightInput.brake ? ui::sem::WARN : ui::sem::MUTED, "%s", say);
    }

    ImGui::Spacing();

    // The controls below SHOW the decision while automatic is on. Left live,
    // they would fight the detector - a press would last exactly one frame,
    // which reads as a broken button rather than as a busy one.
    ImGui::BeginDisabled(autoLights);

    const Float32 w = ImGui::GetContentRegionAvail().x;
    const Float32 third = (w - ImGui::GetStyle().ItemSpacing.x * 2.0f) / 3.0f;
    const Float32 quarter = (w - ImGui::GetStyle().ItemSpacing.x * 3.0f) / 4.0f;

    ImGui::TextDisabled("Headlights");
    struct HeadOpt { const Char* label; lights::Head v; };
    static const HeadOpt HEADS[3] = {
        { "Off", lights::Head::HEAD_OFF },
        { "DRL", lights::Head::HEAD_DRL },
        { "On",  lights::Head::HEAD_ON  },
    };
    // Off / dim / full, as three states of one lamp - which is what they are.
    static const ui::Icon HEAD_ICONS[3] = {
        ui::Icon::ICON_STATUS_IDLE, ui::Icon::ICON_LAMP_DIM, ui::Icon::ICON_LAMP,
    };
    for(Int32 i = 0; i < 3; ++i)
    {
        if(i) ImGui::SameLine();
        if(ui::segmentedIconButton(HEAD_ICONS[i], HEADS[i].label,
                                   lightInput.head == HEADS[i].v,
                                   ImVec2(third, 0.0f)))
            lightInput.head = HEADS[i].v;
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Turn");
    struct TurnOpt { const Char* label; lights::Turn v; };
    static const TurnOpt TURNS[4] = {
        { "Off",    lights::Turn::TURN_OFF    },
        { "Left",   lights::Turn::TURN_LEFT   },
        { "Right",  lights::Turn::TURN_RIGHT  },
        { "Hazard", lights::Turn::TURN_HAZARD },
    };
    for(Int32 i = 0; i < 4; ++i)
    {
        if(i) ImGui::SameLine();

        // Hazards get the warning glyph and the amber the lamp itself uses;
        // Left and Right get neither, because the pack has no left/right arrow
        // and a rotate glyph pointing the wrong way is worse than nothing.
        const Bool sel = (lightInput.turn == TURNS[i].v);
        const ui::Tint t = (TURNS[i].v == lights::Turn::TURN_HAZARD)
                         ? ui::Tint::TINT_WARN : ui::Tint::TINT_NONE;

        Bool hit;
        if(TURNS[i].v == lights::Turn::TURN_HAZARD)
        {
            ui::pushTint(t);
            hit = ui::segmentedIconButton(ui::Icon::ICON_HAZARD, TURNS[i].label,
                                          sel, ImVec2(quarter, 0.0f));
            ui::popTint(t);
        }
        else if(TURNS[i].v == lights::Turn::TURN_OFF)
        {
            hit = ui::segmentedIconButton(ui::Icon::ICON_STATUS_IDLE, TURNS[i].label,
                                          sel, ImVec2(quarter, 0.0f));
        }
        else
        {
            hit = ui::segmentedButton(TURNS[i].label, sel, ImVec2(quarter, 0.0f));
        }

        if(hit)
            lightInput.turn = TURNS[i].v;
    }

    ImGui::Spacing();
    ui::checkbox("Brake", &lightInput.brake);
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("Tail and brake are the same red lamp - 30%% and 100%%.\n"
                          "With an indicator running, that side alternates and the\n"
                          "other stays solid. Try Brake with Right.");
    ImGui::SameLine();
    ui::checkbox("Reverse", &lightInput.reverse);

    ImGui::Spacing();
    if(ui::iconButton(ui::Icon::ICON_CLEAR, "All off"))
        lightInput = lights::Input{};

    ImGui::EndDisabled();
}

Void sectionVehicle()
{
    drawLightingBench();
    ImGui::Spacing();
    drawPicoLinkBlock();
    ImGui::Spacing();
    drawPicoCommands();
    ImGui::Spacing();
    drawControllerState();
}

// =================================================================== firmware
// Load a different program onto the Pico on demand. Everything here is a thin
// face over firmware\build.bat / flash.bat / backup.bat - the same scripts that
// work from a terminal - so there is one flashing mechanism, not two.

Void sizeText(Char* buf, Size n, Int64 bytes)
{
    if(bytes >= 1024 * 1024) std::snprintf(buf, n, "%.1f MB", bytes / (1024.0 * 1024.0));
    else                      std::snprintf(buf, n, "%lld KB", (bytes + 512) / 1024);
}

// The catalog's descriptions are paragraphs written for a human reading the
// file, not labels. They live in a hover tooltip so a row stays one predictable
// height and no prose sits permanently on screen.
Void descriptionTooltip(const Str& text)
{
    if(text.empty() || !ImGui::IsItemHovered()) return;

    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(420.0f * uiDpiScale);
    ImGui::TextUnformatted(text.c_str());
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

// The script output pane. A build prints a hundred lines and you want to watch
// them arrive, so this scrolls - it is a log.
Void drawFlashOutput(const Char* id, const ImVec2& size)
{
    if(ui::iconButton(ui::Icon::ICON_CLEAR, "Clear")) flashLog.clear();
    ImGui::SameLine();
    ui::checkbox("Auto-scroll", &flashAutoscroll);
    ImGui::SameLine();
    {
        ScopedFont sf(ui::fonts.small);
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled("%d lines", static_cast<Int32>(flashLog.size()));
    }

    ImGui::BeginChild(id, size, ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);
    {
        ScopedFont sf(ui::fonts.small);

        if(!flashLog.empty())
        {
            ImGuiListClipper clipper;
            clipper.Begin(static_cast<Int32>(flashLog.size()));
            while(clipper.Step())
            {
                for(Int32 r = clipper.DisplayStart; r < clipper.DisplayEnd; ++r)
                {
                    const Str& s = flashLog[r];

                    // The scripts prefix their lines: [error]/[fail ] red,
                    // [ok   ] green, the rest plain.
                    ImU32 col = ui::plot::LABEL;
                    if(s.rfind("[error", 0) == 0 || s.rfind("[fail", 0) == 0)
                        col = ui::sem::BAD;
                    else if(s.rfind("[ok", 0) == 0)
                        col = ui::sem::GOOD;
                    else if(s.rfind("[start", 0) == 0 || s.rfind("[busy", 0) == 0 ||
                             s.rfind("[skip", 0) == 0)
                        col = ui::sem::WARN;

                    ImGui::PushStyleColor(ImGuiCol_Text, col);
                    ImGui::TextUnformatted(s.c_str());
                    ImGui::PopStyleColor();
                }
            }
        }

        if(flashAutoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
}

Void drawFlashControls()
{
    const ImGuiStyle& sty  = ImGui::GetStyle();
    const BoardStatus brd  = picoFlash.board();
    const Bool        busy = picoFlash.busy();
    const Float32       bh   = ImGui::GetFrameHeight() * 1.2f;

    // ---- board -----------------------------------------------------------
    ImGui::SeparatorText("Board");

    const Float32 refreshW = ImGui::CalcTextSize("Refresh").x + sty.FramePadding.x * 2.0f;

    {
        // BOOTSEL is a MODE, and the single most common way to be confused by
        // this board is to forget you are in it: the COM port is gone, nothing
        // answers, and it looks broken. So it gets the loud treatment.
        ScopedFont sf(ui::fonts.title);
        ImGui::AlignTextToFramePadding();

        if(brd.bootsel)
            colored(ui::sem::WARN, "BOOTSEL  -  %s", brd.drive.c_str());
        else if(brd.present)
            colored(ui::sem::GOOD, "Running  -  %s",
                    brd.port.empty() ? "no serial port" : brd.port.c_str());
        else
            colored(ui::sem::MUTED, "No board found");
    }

    ImGui::SameLine(ImGui::GetContentRegionAvail().x - refreshW);
    if(ui::iconButton(ui::Icon::ICON_REFRESH, "Refresh"))
    {
        picoFlash.refreshBoard();
        picoFlash.refreshCatalog();
    }

    {
        // Always drawn, even when there is nothing to say. Rendering this row
        // conditionally made every control below it jump a line as the board
        // entered and left BOOTSEL - which is exactly when you are looking at
        // this panel and clicking things.
        ScopedFont sf(ui::fonts.small);

        if(brd.present && !brd.bootsel)
            ImGui::TextDisabled("%s%s%s", brd.chip.c_str(),
                                brd.program.empty() ? "" : "   ", brd.program.c_str());
        else if(brd.bootsel)
            ImGui::TextDisabled("%s%sbootloader",
                                brd.chip.c_str(), brd.chip.empty() ? "" : "   ");
        else
            ImGui::TextDisabled("--");
    }

    // ---- current operation ------------------------------------------------
    const FlashState st = picoFlash.state();
    if(st != FlashState::FLASH_STATE_IDLE)
    {
        const ImU32 col = (st == FlashState::FLASH_STATE_WORKING) ? ui::sem::WARN
                        : (st == FlashState::FLASH_STATE_SUCCESS) ? ui::sem::GOOD : ui::sem::BAD;
        const Char* verb = (st == FlashState::FLASH_STATE_WORKING) ? "Running"
                         : (st == FlashState::FLASH_STATE_SUCCESS) ? "Done" : "FAILED";
        colored(col, "%s: %s", verb, picoFlash.currentOp().c_str());
    }
    else
    {
        ImGui::TextDisabled("Idle");
    }

    // ---- catalog ----------------------------------------------------------
    ImGui::Spacing();
    ImGui::SeparatorText("Firmware");

    const Vec<FirmwareEntry>& cat = picoFlash.catalog();

    // OpenPopup is deferred out of the row's PushID scope: a popup's identity
    // comes from the ID stack, so opening it inside the row and beginning it
    // outside would never match.
    Bool openConfirm = false;

    if(cat.empty())
        colored(ui::sem::WARN, "catalog.txt: no entries");

    for(const FirmwareEntry& e : cat)
    {
        ImGui::PushID(e.id.c_str());

        {
            ScopedFont sf(ui::fonts.title);
            ImGui::TextUnformatted(e.name.c_str());
        }
        descriptionTooltip(e.description);

        {
            ScopedFont sf(ui::fonts.small);
            if(e.present)
            {
                Char sz[32];
                sizeText(sz, sizeof(sz), e.sizeBytes);
                colored(ui::sem::GOOD, "%s   %s", sz, e.builtAt.c_str());
            }
            else
            {
                colored(ui::sem::WARN, "%s",
                        e.buildable ? "not built yet" : "missing on disk");
            }

            // Which board the image is FOR.
            //
            // Stated here because it is stated nowhere else: a .uf2 carries no
            // board, the RP2350 accepts either one without complaint, and a
            // Pico 2 W image on the car's plain Pico 2 boots, runs, answers
            // every command, and simply has a dead LED - because it spent its
            // startup bringing up a wireless chip that is not in the package.
            // That is a long evening to save with one word in a list.
            if(!e.board.empty())
            {
                ImGui::SameLine();
                colored(ui::sem::MUTED, "   %s", e.board.c_str());
            }
        }

        const Float32 half = (ImGui::GetContentRegionAvail().x - sty.ItemSpacing.x) * 0.5f;

        ImGui::BeginDisabled(busy || !e.buildable);
        if(ui::iconButton(ui::Icon::ICON_BUILD, "Build", ImVec2(half, bh)))
            picoFlash.build(e.id);
        ImGui::EndDisabled();
        if(!e.buildable && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("No source for this in the repo - the .uf2 is all there is.");

        ImGui::SameLine();

        ImGui::BeginDisabled(busy || !e.present);
        if(ui::iconButton(ui::Icon::ICON_FLASH, "Flash...", ImVec2(half, bh),
                          ui::Tint::TINT_WARN))
        {
            confirmId   = e.id;
            confirmName = e.name;
            confirmPath = e.uf2Path;
            openConfirm   = true;
        }
        ImGui::EndDisabled();

        ImGui::PopID();
        ImGui::Separator();
    }

    if(openConfirm) ImGui::OpenPopup("Flash this firmware?");

    // ---- backup -----------------------------------------------------------
    ImGui::SeparatorText("Backup");

    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputTextWithHint("##backupout", "output .uf2 path",
                             backupBuf, sizeof(backupBuf));

    ImGui::BeginDisabled(busy || backupBuf[0] == '\0');
    if(ui::iconButton(ui::Icon::ICON_BACKUP, "Back up board flash", ImVec2(-FLT_MIN, bh))) startBackup();
    ImGui::EndDisabled();

    // ---- reboot -----------------------------------------------------------
    ImGui::Spacing();
    ImGui::SeparatorText("Reboot");

    const Float32 half = (ImGui::GetContentRegionAvail().x - sty.ItemSpacing.x) * 0.5f;

    ImGui::BeginDisabled(busy);
    if(ui::iconButton(ui::Icon::ICON_REBOOT, "To BOOTSEL", ImVec2(half, bh),
                      ui::Tint::TINT_WARN))
    {
        picoLink.disconnect();
        releasePicoPortForBoardOp();
        picoFlash.rebootBootsel();
    }
    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_REBOOT, "Normally", ImVec2(half, bh)))
        releasePicoPortForBoardOp();
        picoFlash.rebootNormal();
    ImGui::EndDisabled();

    // ---- confirmation -----------------------------------------------------
    // Flashing is destructive and, for anything not in the catalog, permanent.
    // It gets a modal that names the image and says so.
    const ImVec2 centre = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if(ImGui::BeginPopupModal("Flash this firmware?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::PushTextWrapPos(460.0f * uiDpiScale);
        {
            ScopedFont sf(ui::fonts.title);
            ImGui::TextUnformatted(confirmName.c_str());
        }
        {
            ScopedFont sf(ui::fonts.small);
            ImGui::TextDisabled("%s", confirmPath.c_str());
        }

        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ui::sem::BAD);
        ImGui::TextWrapped("This overwrites whatever the board is running now.");
        ImGui::PopStyleColor();

        ImGui::Spacing();
        ImGui::BulletText("The board reboots into the bootloader and the COM port drops.");
        ImGui::BulletText("The existing firmware is replaced, not saved.");
        ImGui::BulletText("There is no undo: unless that firmware is in the catalog\n"
                          "above, the only way back is a backup you took first.");
        ImGui::Spacing();
        ImGui::TextWrapped("If you are unsure what is on the board, cancel and press "
                           "\"Back up board flash\" first.");
        ImGui::PopTextWrapPos();

        ImGui::Separator();

        if(ui::button("Cancel", ImVec2(150.0f * uiDpiScale, bh)))
            ImGui::CloseCurrentPopup();

        ImGui::SameLine();
        // Red: this is the point of no return - it overwrites the board.
        if(ui::iconButton(ui::Icon::ICON_FLASH, "Flash it",
                          ImVec2(260.0f * uiDpiScale, bh), ui::Tint::TINT_BAD))
        {
            // flash.ps1 does the 1200-baud touch itself, and it cannot open the
            // port while this app has it.
            picoLink.disconnect();
            releasePicoPortForBoardOp();
            picoFlash.flash(confirmId);
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// Board state, the catalog, backup and reboot. What the scripts PRINT while
// doing any of it is in the Console section - one log, one place.
Void sectionFirmware()
{
    drawFlashControls();
}

// ==================================================================== console
// Everything that streams, in one place: the serial line log and whatever the
// flash/build scripts are printing.
//
// Both live in the sidebar, which scrolls, so each gets its OWN fixed-height
// scroll region rather than filling the column - a log that grows the page it
// sits on cannot be read, and one that autoscrolls while the page also scrolls
// cannot be used at all.
constexpr Float32 LOG_PANE_H = 260.0f;   // logical px; multiplied by uiDpiScale at use

// ---- the console's palette ------------------------------------------------
//
// A terminal, not a panel. Pure black rather than the skeuomorphic slate the
// rest of the hub uses, because this is the one surface that is not a moulded
// object - it is a screen, and a screen in 2010 was black with phosphor on it.
// The bevels and plates elsewhere are pretending to be plastic; this is
// pretending to be a CRT, and mixing the two is what made it look like a text
// box rather than a terminal.
//
// The colours are the ANSI sixteen, which is the palette every serial monitor
// has used for forty years, so a line reads the way it would in any of them.
namespace ansi {

constexpr ImU32 BLACK   = IM_COL32(0x0A, 0x0A, 0x0A, 0xFF);   // the ground
constexpr ImU32 RED     = IM_COL32(0xCD, 0x32, 0x32, 0xFF);
constexpr ImU32 GREEN   = IM_COL32(0x3F, 0xC0, 0x50, 0xFF);
constexpr ImU32 YELLOW  = IM_COL32(0xCD, 0xAA, 0x2E, 0xFF);
constexpr ImU32 BLUE    = IM_COL32(0x40, 0x80, 0xD0, 0xFF);
constexpr ImU32 MAGENTA = IM_COL32(0xB0, 0x5C, 0xC0, 0xFF);
constexpr ImU32 CYAN    = IM_COL32(0x35, 0xB5, 0xB5, 0xFF);
constexpr ImU32 WHITE   = IM_COL32(0xC8, 0xC8, 0xC8, 0xFF);
constexpr ImU32 GREY    = IM_COL32(0x66, 0x66, 0x66, 0xFF);
constexpr ImU32 BRIGHT  = IM_COL32(0xEE, 0xEE, 0xEE, 0xFF);

} // namespace ansi

// What colour a line is, from what the board actually says.
//
// The firmware's vocabulary is four words wide - OK, ERR, INFO, and a bare
// reply - and it is worth colouring because scanning a console for the one ERR
// in three hundred lines is exactly what colour is for.
ImU32 consoleColour(const PicoLine& ln)
{
    if(ln.poll)
    {
        // Chatter, when it is shown at all, is dim. It is context, not content.
        return ansi::GREY;
    }
    if(ln.outgoing)
    {
        return ansi::CYAN;      // what WE said
    }

    const Char* t = ln.text.c_str();
    if(std::strncmp(t, "ERR", 3) == 0)  return ansi::RED;
    if(std::strncmp(t, "OK", 2) == 0)   return ansi::GREEN;
    if(std::strncmp(t, "INFO", 4) == 0) return ansi::BLUE;
    if(std::strncmp(t, "PONG", 4) == 0) return ansi::GREEN;
    return ansi::WHITE;
}

Void drawSerialConsole(const ImVec2& size)
{
    if(ui::iconButton(ui::Icon::ICON_CLEAR, "Clear")) picoLog.clear();
    ImGui::SameLine();
    ui::checkbox("Auto-scroll", &logAutoscroll);
    ImGui::SameLine();

    // Off by default. The hub polls DRIVE, LIGHTS, STATUS and TOF on their own
    // timers - the indicator poll alone is eight sends and eight replies a
    // second - and a console showing all of it buries everything you typed.
    ui::checkbox("Polling", &logShowPoll);
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Show the traffic the hub generates on its own:\n"
                          "DRIVE, LIGHTS, STATUS and TOF polls, and their\n"
                          "replies. Anything you send by hand always shows.");
    }

    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_SAVE, "Copy"))
    {
        Str all;
        for(Int32 i : logShown)
        {
            const PicoLine& ln = picoLog[i];
            all += ln.outgoing ? "> " : "< ";
            all += ln.text;
            all += "\n";
        }
        ImGui::SetClipboardText(all.c_str());
        LOG_INFO("console", "copied %d lines", static_cast<Int32>(logShown.size()));
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Copy every line currently shown - what the filter\n"
                          "left, in the order it is on screen.");
    }

    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputTextWithHint("##logfilter", "filter lines", filterBuf, sizeof(filterBuf));

    logShown.clear();
    for(Int32 i = 0; i < static_cast<Int32>(picoLog.size()); ++i)
    {
        if(!logShowPoll && picoLog[i].poll) continue;
        if(logMatches(picoLog[i])) logShown.push_back(i);
    }

    {
        ScopedFont sf(ui::fonts.small);
        const Int32 hidden = static_cast<Int32>(picoLog.size())
                           - static_cast<Int32>(logShown.size());
        if(hidden > 0)
            ImGui::TextDisabled("%d of %d lines   -   %d hidden   -   %llu sent / %llu received",
                                static_cast<Int32>(logShown.size()),
                                static_cast<Int32>(picoLog.size()), hidden,
                                picoLink.txLines(), picoLink.rxLines());
        else
            ImGui::TextDisabled("%d lines   -   %llu sent / %llu received",
                                static_cast<Int32>(picoLog.size()),
                                picoLink.txLines(), picoLink.rxLines());
    }

    // ---- the screen -------------------------------------------------------
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ansi::BLACK);
    ImGui::PushStyleColor(ImGuiCol_Header,        IM_COL32(0x2A, 0x44, 0x60, 0xFF));
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, IM_COL32(0x22, 0x36, 0x4E, 0xFF));
    ImGui::PushStyleColor(ImGuiCol_HeaderActive,  IM_COL32(0x2A, 0x44, 0x60, 0xFF));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));

    ImGui::BeginChild("##console", size, ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);

    // The font is pushed and popped INSIDE the child.
    //
    // A ScopedFont declared at function scope would pop after EndChild, and
    // ImGui compares the style and font stack sizes at Begin against End - a
    // push that straddles the boundary leaves the frame unbalanced, which
    // renders as a blank window rather than as an error anybody can read.
    {
    ScopedFont mono(ui::fonts.mono);

    if(!logShown.empty())
    {
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<Int32>(logShown.size()));
        while(clipper.Step())
        {
            for(Int32 r = clipper.DisplayStart; r < clipper.DisplayEnd; ++r)
            {
                const Int32     idx = logShown[r];
                const PicoLine& ln  = picoLog[idx];

                Char buf[512];
                std::snprintf(buf, sizeof(buf), "%8.2f  %c  %s",
                              ln.tS, ln.outgoing ? '>' : '<', ln.text.c_str());

                // A Selectable rather than plain text, so lines can be
                // highlighted and copied. Click one, shift-click another for a
                // run, ctrl-click to add or remove one - the same three
                // gestures every list in every program uses.
                ImGui::PushStyleColor(ImGuiCol_Text, consoleColour(ln));
                ImGui::PushID(idx);
                const Bool wasSel = (logSel.count(idx) != 0);
                if(ImGui::Selectable(buf, wasSel,
                                     ImGuiSelectableFlags_AllowDoubleClick))
                {
                    const ImGuiIO& io = ImGui::GetIO();
                    if(io.KeyShift && logSelAnchor >= 0)
                    {
                        // A run, in SCREEN order rather than log order: the
                        // filter can hide lines between the two clicks and
                        // selecting what is not visible would be a surprise.
                        Int32 a = -1;
                        Int32 b = -1;
                        for(Int32 k = 0; k < static_cast<Int32>(logShown.size()); ++k)
                        {
                            if(logShown[k] == logSelAnchor) a = k;
                            if(logShown[k] == idx)          b = k;
                        }
                        if(a >= 0 && b >= 0)
                        {
                            if(a > b)
                            {
                                const Int32 t = a;
                                a = b;
                                b = t;
                            }
                            logSel.clear();
                            for(Int32 k = a; k <= b; ++k) logSel.insert(logShown[k]);
                        }
                    }
                    else if(io.KeyCtrl)
                    {
                        if(wasSel) logSel.erase(idx);
                        else       logSel.insert(idx);
                        logSelAnchor = idx;
                    }
                    else
                    {
                        logSel.clear();
                        logSel.insert(idx);
                        logSelAnchor = idx;
                    }
                }
                ImGui::PopID();
                ImGui::PopStyleColor();
            }
        }
    }

    // Ctrl+C copies the selection, which is what the highlight is FOR. Scoped
    // to this child being hovered or focused so it does not steal the shortcut
    // from the editor.
    if(!logSel.empty()
       && ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)
       && ImGui::IsKeyPressed(ImGuiKey_C)
       && ImGui::GetIO().KeyCtrl)
    {
        Str out;
        for(Int32 i : logShown)
        {
            if(logSel.count(i) == 0) continue;
            out += picoLog[i].outgoing ? "> " : "< ";
            out += picoLog[i].text;
            out += "\n";
        }
        ImGui::SetClipboardText(out.c_str());
        LOG_INFO("console", "copied %d selected line(s)",
                 static_cast<Int32>(logSel.size()));
    }

    // Sticks to the bottom only while the view already is at the bottom, so
    // scrolling up to read something does not yank you back.
    if(logAutoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();

    ImGui::PopStyleVar();
    ImGui::PopStyleColor(4);
}

// The console column: the two logs, filling their own column on the left.
//
// The same two tabs the sidebar section had, but given a whole column's height
// instead of a fifth of one. That is the entire reason it moved: a log that
// produces hundreds of lines and shows four is a log you scroll rather than
// read.
Void drawConsoleColumn(Float32 w, Float32 h)
{
    ImGui::BeginChild("##consolecol", ImVec2(w, h), ImGuiChildFlags_Borders);

    if(ImGui::BeginTabBar("##concoltabs"))
    {
        Char lbA[40];
        const Bool tA = ImGui::BeginTabItem(
            iconTabLabel(lbA, sizeof(lbA), "Pico serial"));
        tabIcon(ui::Icon::ICON_CONSOLE);
        if(tA)
        {
            // Zero height means "the rest of this child", so the log grows with
            // the window instead of being pinned to a constant.
            drawSerialConsole(ImVec2(0.0f, 0.0f));
            ImGui::EndTabItem();
        }

        Char lbB[40];
        const Bool tB = ImGui::BeginTabItem(
            iconTabLabel(lbB, sizeof(lbB), "Build / flash"));
        tabIcon(ui::Icon::ICON_BUILD);
        if(tB)
        {
            drawFlashOutput("##flashout_col", ImVec2(0.0f, 0.0f));
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::EndChild();
}

// ===================================================================== modals
// Raised from more than one workspace, so opened and begun at the root of the
// frame where the ID stack is stable.

Void drawGlobalModals()
{
    const Float32 bh = ImGui::GetFrameHeight() * 1.2f;

    const Str bport = picoLink.port().empty()
        ? (picoIndex >= 0 && picoIndex < static_cast<Int32>(picoPorts.size())
               ? picoPorts[picoIndex] : Str())
        : picoLink.port();

    if(openBootsel)
    {
        ImGui::OpenPopup("Reboot to BOOTSEL?");
        openBootsel = false;
    }

    const ImVec2 centre = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if(ImGui::BeginPopupModal("Reboot to BOOTSEL?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::PushTextWrapPos(420.0f * uiDpiScale);
        ImGui::TextWrapped("This reboots %s into the RP2350 USB bootloader.",
                           bport.empty() ? "the board" : bport.c_str());
        ImGui::Spacing();
        ImGui::BulletText("Whatever the board is running stops immediately.");
        ImGui::BulletText("The serial link drops and the port disappears.");
        ImGui::BulletText("It remounts as the RP2350 mass-storage drive.");
        ImGui::BulletText("It does not come back until a .uf2 is copied onto it,\n"
                          "or the board is power-cycled.");
        ImGui::PopTextWrapPos();

        ImGui::Separator();

        if(ui::button("Cancel", ImVec2(150.0f * uiDpiScale, bh)))
            ImGui::CloseCurrentPopup();

        ImGui::SameLine();
        if(ui::iconButton(ui::Icon::ICON_REBOOT, "Reboot to BOOTSEL",
                          ImVec2(260.0f * uiDpiScale, bh), ui::Tint::TINT_WARN))
        {
            picoLink.disconnect();
            bootselOk   = PicoLink::bootselTouch(bport);
            bootselDone = true;
            refreshPicoPorts();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

// ==================================================================== sidebar
// One scrollable column, five collapsing sections, drawn in a fixed order. This
// is the only place in the app that scrolls as a page.

// ---------------------------------------------------------------------------
// The sections, and the three things you can do to them: reorder, tear off,
// put back.
// ---------------------------------------------------------------------------
struct SectionEntry
{
    const Char* label;          // leading spaces clear the icon; see iconTabLabel
    const Char* title;          // no padding, for the floating window's title bar
    Int32       id;
    Bool        openByDefault;
    ui::Icon    icon;
    Void      (*body)();
};

const SectionEntry SECTIONS[SECTION_COUNT] = {
    { "    System",   "System",   SECTION_SYSTEM,   true,  ui::Icon::ICON_SYSTEM,   &sectionSystem   },
    { "    Sensors",  "Sensors",  SECTION_SENSORS,  true,  ui::Icon::ICON_SENSORS,  &sectionSensors  },
    { "    Vehicle",  "Vehicle",  SECTION_VEHICLE,  false, ui::Icon::ICON_VEHICLE,  &sectionVehicle  },
    { "    Firmware", "Firmware", SECTION_FIRMWARE, false, ui::Icon::ICON_FIRMWARE, &sectionFirmware },
};

const SectionEntry& sectionById(Int32 id)
{
    for(const SectionEntry& e : SECTIONS)
        if(e.id == id)
            return e;
    return SECTIONS[0];
}

// Moves the section at slot `from` to slot `to`, shifting the rest along.
// A rotate, not a swap: dragging Console to the top should leave the others in
// their relative order, and a swap would drop System to the bottom instead.
Void moveSection(Int32 from, Int32 to)
{
    if(from == to || from < 0 || to < 0 || from >= SECTION_COUNT || to >= SECTION_COUNT)
        return;

    const Int32 moved = sectionOrder[from];
    if(from < to)
        for(Int32 k = from; k < to; ++k) sectionOrder[k] = sectionOrder[k + 1];
    else
        for(Int32 k = from; k > to; --k) sectionOrder[k] = sectionOrder[k - 1];

    sectionOrder[to]   = moved;
    panelLayoutDirty   = true;
}

// The tear-off button, sitting at the right end of the row the header owns.
// Returns true if it was pressed. Call it immediately after the header, which
// must have had SetNextItemAllowOverlap() so this wins the hit test.
//
// SameLine, NOT SetCursorScreenPos.
//
// The first version placed the button by hand and then restored the cursor, and
// it deadlocked the app on the very first frame: moving the cursor past the last
// submitted item and then ending the window without submitting another is an
// ImGui assertion, and asserts are live in this build. The last section in the
// column is the one that trips it - every other one has a following header to
// grow the bounds back. SameLine keeps the whole thing inside ImGui's own layout
// and cannot desynchronise from it.
Bool tearOffButton(Int32 id, Bool floating)
{
    const Char*   lbl = floating ? "dock" : "float";
    const Float32 w   = ImGui::CalcTextSize(lbl).x
                      + ImGui::GetStyle().FramePadding.x * 2.0f;

    const Float32 x = ImGui::GetContentRegionMax().x - w
                    - ImGui::GetStyle().FramePadding.x;

    ImGui::SameLine(x, 0.0f);
    ImGui::PushID(id);
    const Bool hit = ImGui::Button(lbl, ImVec2(w, 0.0f));
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip(floating ? "Put this panel back in the column"
                                   : "Tear this panel off into its own window");
    ImGui::PopID();
    return hit;
}

Void drawSidebar(Float32 width, Float32 height)
{
    ImGui::BeginChild("##sidebar", ImVec2(width, height), ImGuiChildFlags_None,
                      ImGuiWindowFlags_None);

    Int32 dragFrom = -1, dragTo = -1;

    for(Int32 slot = 0; slot < SECTION_COUNT; ++slot)
    {
        const SectionEntry& e = sectionById(sectionOrder[slot]);

        // Torn off: the row stays, as a placeholder, so the panel has somewhere
        // to come back to and the column does not silently lose an entry.
        if(sectionFloating[e.id])
        {
            ImGui::PushStyleColor(ImGuiCol_Header,        ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_HeaderActive,  ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
            ImGui::SetNextItemAllowOverlap();
            ImGui::Selectable(e.label);
            ImGui::PopStyleColor(4);

            const ImVec2 a2 = ImGui::GetItemRectMin();
            const ImVec2 b2 = ImGui::GetItemRectMax();
            if(ui::iconsReady())
                ui::iconAt(ImGui::GetWindowDrawList(), e.icon,
                           ImVec2(a2.x + ImGui::GetStyle().FramePadding.x
                                       + ImGui::GetFontSize() * 1.35f,
                                  a2.y + ((b2.y - a2.y) - ui::iconSize()) * 0.5f));

            if(tearOffButton(e.id, true))
            {
                sectionFloating[e.id] = false;
                panelLayoutDirty      = true;
            }
            continue;
        }

        const Bool forced = (forceSection == e.id && forceTabFrames > 0);
        if(forced) ImGui::SetNextItemOpen(true, ImGuiCond_Always);

        // So the tear-off button can sit on top of the header's own hit box.
        ImGui::SetNextItemAllowOverlap();
        const Bool open = ImGui::CollapsingHeader(
            e.label, e.openByDefault ? ImGuiTreeNodeFlags_DefaultOpen : 0);

        const ImVec2 hp = ImGui::GetItemRectMin();
        const ImVec2 hq = ImGui::GetItemRectMax();

        // Drag a header to reorder the column. The payload is the SLOT, not the
        // section id: the drop target needs to know where the thing came from.
        if(ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceNoDisableHover))
        {
            ImGui::SetDragDropPayload("TT02_SECTION", &slot, sizeof(Int32));
            ImGui::TextUnformatted(e.title);
            ImGui::EndDragDropSource();
        }
        if(ImGui::BeginDragDropTarget())
        {
            if(const ImGuiPayload* pl = ImGui::AcceptDragDropPayload("TT02_SECTION"))
            {
                dragFrom = *static_cast<const Int32*>(pl->Data);
                dragTo   = slot;
            }
            ImGui::EndDragDropTarget();
        }

        // Drawn over the header after the fact rather than as part of its label:
        // CollapsingHeader takes a string, and putting an image inside it would
        // mean hand-rolling the whole widget to get one 16px picture in.
        if(ui::iconsReady())
            ui::iconAt(ImGui::GetWindowDrawList(), e.icon,
                       ImVec2(hp.x + ImGui::GetStyle().FramePadding.x
                                   + ImGui::GetFontSize() * 1.35f,
                              hp.y + ((hq.y - hp.y) - ui::iconSize()) * 0.5f));

        if(tearOffButton(e.id, false))
        {
            sectionFloating[e.id] = true;
            panelLayoutDirty      = true;
        }

        // A named section is no use if it opened below the fold. Scrolling to
        // the header itself, not into its body, keeps the label on screen.
        if(forced) ImGui::SetScrollHereY(0.0f);

        if(open)
        {
            ImGui::PushID(e.id);
            e.body();
            ImGui::PopID();
            ImGui::Spacing();
        }
    }

    // Applied after the loop: reordering mid-iteration would draw a section
    // twice or not at all in the frame it moved.
    if(dragFrom >= 0)
        moveSection(dragFrom, dragTo);
    ImGui::EndChild();
}

// The torn-off panels. Drawn OUTSIDE the root window, after it ends, so they
// float above the whole app rather than being clipped to a child region.
Void drawFloatingPanels()
{
    for(Int32 slot = 0; slot < SECTION_COUNT; ++slot)
    {
        const SectionEntry& e = sectionById(sectionOrder[slot]);
        if(!sectionFloating[e.id])
            continue;

        // First appearance only. After that ImGui's own ini has the position the
        // user dragged it to, and forcing one would undo their arrangement on
        // every launch.
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f
                                           + static_cast<Float32>(slot) * 24.0f * uiDpiScale,
                                       vp->WorkPos.y + 80.0f * uiDpiScale
                                           + static_cast<Float32>(slot) * 24.0f * uiDpiScale),
                                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(420.0f * uiDpiScale, 380.0f * uiDpiScale),
                                 ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints(ImVec2(240.0f * uiDpiScale, 120.0f * uiDpiScale),
                                            ImVec2(FLT_MAX, FLT_MAX));

        Bool open = true;
        if(ImGui::Begin(e.title, &open))
        {
            ImGui::PushID(e.id);
            e.body();
            ImGui::PopID();
        }
        ImGui::End();

        // Closing a torn-off panel docks it. It does not hide the section - a
        // window with an X that makes a feature unreachable is a trap.
        if(!open)
        {
            sectionFloating[e.id] = false;
            panelLayoutDirty      = true;
        }
    }
}

// The column's width for this frame, clamped to something usable. Pure: the
// layout needs it before the handle can be drawn, and a widget cannot be asked
// for its answer twice in one frame without colliding with its own ID.
Float32 consoleWidth(Float32 availW)
{
    const Float32 lo = CONSOLE_MIN_W * uiDpiScale;
    const Float32 hi = std::max(lo, availW * 0.50f);

    Float32 w = consoleLogicalW * uiDpiScale;
    if(w < lo) w = lo;
    if(w > hi) w = hi;
    return w;
}

// The handle on the console column's RIGHT edge.
//
// The sign is the opposite of the sidebar's for the same reason the code tree's
// is: this panel is on the left, so dragging right widens it.
Void consoleSplitter(const ImVec2& at, Float32 h, Float32 thickness)
{
    ImGui::SetCursorScreenPos(at);
    ImGui::InvisibleButton("##console-split", ImVec2(thickness, h));

    const Bool active = ImGui::IsItemActive();
    if(active || ImGui::IsItemHovered())
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);

    if(active)
    {
        consoleLogicalW += ImGui::GetIO().MouseDelta.x / uiDpiScale;
        consoleLogicalW  = std::max(CONSOLE_MIN_W, std::min(1600.0f, consoleLogicalW));
        panelLayoutDirty = true;
    }

    if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
    {
        consoleLogicalW  = CONSOLE_DEF_W;
        panelLayoutDirty = true;
    }

    const ImU32 col = active   ? ui::accent::CYAN
                    : ImGui::IsItemHovered() ? ui::accent::CYAN_HI
                    : IM_COL32(0x50, 0x58, 0x60, 0xFF);
    ImDrawList*   dl = ImGui::GetWindowDrawList();
    const Float32 cx = at.x + thickness * 0.5f;
    const Float32 cy = at.y + h * 0.5f;
    const Float32 r  = 1.5f * uiDpiScale;
    for(Int32 k = -1; k <= 1; ++k)
        dl->AddCircleFilled(ImVec2(cx, cy + static_cast<Float32>(k) * 6.0f * uiDpiScale),
                            r, col, 8);
}

Float32 sidebarWidth(Float32 availW)
{
    const Float32 lo = SIDEBAR_MIN_W * uiDpiScale;
    const Float32 hi = std::max(lo, availW * 0.62f);

    Float32 w = sidebarLogicalW * uiDpiScale;
    if(w < lo) w = lo;
    if(w > hi) w = hi;
    return w;
}

// The drag handle between the map and the column.
//
// The value it adjusts is in LOGICAL pixels: dividing the mouse delta by the
// scale is what keeps a drag feeling the same at 100% and at 200%.
Void sidebarSplitter(const ImVec2& at, Float32 h, Float32 thickness)
{
    ImGui::SetCursorScreenPos(at);
    ImGui::InvisibleButton("##sidebar-split", ImVec2(thickness, h));

    const Bool active = ImGui::IsItemActive();
    if(active || ImGui::IsItemHovered())
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);

    if(active)
    {
        sidebarLogicalW -= ImGui::GetIO().MouseDelta.x / uiDpiScale;
        panelLayoutDirty = true;
    }

    // Double-click restores the default, which is the only way back from a
    // column dragged to a width you cannot grab the edge of.
    if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
    {
        sidebarLogicalW  = 400.0f;
        panelLayoutDirty = true;
    }

    // The grip: three dots, only once it is worth noticing. A permanently drawn
    // handle down the full height of the window would be a bigger mark on the
    // screen than the thing it resizes.
    const ImU32 col = active   ? ui::accent::CYAN
                    : ImGui::IsItemHovered() ? ui::accent::CYAN_HI
                    : IM_COL32(0x50, 0x58, 0x60, 0xFF);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const Float32 cx = at.x + thickness * 0.5f;
    const Float32 cy = at.y + h * 0.5f;
    const Float32 r  = 1.5f * uiDpiScale;
    for(Int32 k = -1; k <= 1; ++k)
        dl->AddCircleFilled(ImVec2(cx, cy + static_cast<Float32>(k) * 6.0f * uiDpiScale),
                            r, col, 8);
}

// The drag handle between the Code view's file tree and the editor.
//
// Deliberately the same shape as sidebarSplitter() above - same three-dot grip,
// same hover colours, same double-click-to-restore. Two resize handles in one
// app that behave differently is worse than either behaviour on its own.
//
// The sign differs because the panel is on the LEFT of this one: dragging right
// widens the tree, where dragging right narrows the sidebar.
Void codeTreeSplitter(const ImVec2& at, Float32 h, Float32 thickness)
{
    ImGui::SetCursorScreenPos(at);
    ImGui::InvisibleButton("##codetree-split", ImVec2(thickness, h));

    const Bool active = ImGui::IsItemActive();
    if(active || ImGui::IsItemHovered())
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);

    if(active)
    {
        codeTreeLogicalW += ImGui::GetIO().MouseDelta.x / uiDpiScale;
        codeTreeLogicalW = std::max(CODE_TREE_MIN_W,
                                    std::min(CODE_TREE_MAX_W, codeTreeLogicalW));
        panelLayoutDirty = true;
    }

    if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
    {
        codeTreeLogicalW = CODE_TREE_DEF_W;
        panelLayoutDirty = true;
    }

    const ImU32 col = active   ? ui::accent::CYAN
                    : ImGui::IsItemHovered() ? ui::accent::CYAN_HI
                    : IM_COL32(0x50, 0x58, 0x60, 0xFF);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const Float32 cx = at.x + thickness * 0.5f;
    const Float32 cy = at.y + h * 0.5f;
    const Float32 r  = 1.5f * uiDpiScale;
    for(Int32 k = -1; k <= 1; ++k)
        dl->AddCircleFilled(ImVec2(cx, cy + static_cast<Float32>(k) * 6.0f * uiDpiScale),
                            r, col, 8);
}

} // namespace

// ------------------------------------------------------------------- public

Void app::init(Float32 dpiScale)
{
    // First thing, so everything after it is on the record - including the
    // startup that fails.
    applog::init();
    LOG_INFO("app", "init: dpi=%.3f", static_cast<Float64>(dpiScale));
    uiDpiScale = dpiScale > 0.0f ? dpiScale : 1.0f;

    loadPanelLayout();

    // The recordings directory is listed once here and on demand after, not
    // every frame: it is a syscall for a list that changes when you ask.
    refreshRecordings();

    for(Int32 i = 0; i < RANGE_COUNT; ++i) RANGE_ITEMS[i] = RANGES[i].label;

    refreshPorts();
    refreshPicoPorts();
    applyRange();

    // The catalog is a couple of file stats; the board query spawns picotool and
    // is asynchronous, so neither delays the first frame.
    picoFlash.refreshCatalog();
    picoFlash.refreshBoard();
    defaultBackupName();

    // --connect [port] [baud] pins a specific port; --no-connect suppresses the
    // automatic attempt. With neither, we just connect: the lidar is the one
    // thing that works today and the common case is "plug it in and look at it".
    Bool suppress = false;

    for(Int32 i = 1; i < __argc; ++i)
    {
        if(std::strcmp(__argv[i], "--no-connect") == 0)
        {
            suppress = true;
            continue;
        }

        // --tab names a sidebar section and opens it. The old workspace and
        // lidar sub-tab names still work - they map onto the section that now
        // holds that content - so existing scripts and habits do not break.
        if(std::strcmp(__argv[i], "--map") == 0 && i + 1 < __argc)
        {
            for(Int32 m = 0; m < static_cast<Int32>(MapMode::MAP_MODE_COUNT); ++m)
                if(_stricmp(__argv[i + 1], mapModeName(static_cast<MapMode>(m))) == 0)
                {
                    radarView.mode  = static_cast<MapMode>(m);
                    radarView.is3D  = false;
                    forceView       = 0;
                    forceViewFrames = 4;
                    centralView     = 0;
                }
            continue;
        }

        // --scene <name> selects a 3D overlay AND switches to 3D, since asking
        // for one without the other is never what was meant.
        if(std::strcmp(__argv[i], "--scene") == 0 && i + 1 < __argc)
        {
            for(Int32 m = 0;
                m < static_cast<Int32>(scene3d::SceneMode::SCENE_MODE_COUNT); ++m)
                if(_stricmp(__argv[i + 1],
                            scene3d::sceneModeName(static_cast<scene3d::SceneMode>(m))) == 0)
                {
                    radarView.scene = static_cast<scene3d::SceneMode>(m);
                    radarView.is3D  = true;
                    forceView       = 1;
                    forceViewFrames = 4;
                    centralView     = 1;
                }
            continue;
        }

        if(std::strcmp(__argv[i], "--view") == 0 && i + 1 < __argc)
        {
            const Char* v = __argv[i + 1];
            if(_stricmp(v, "map") == 0 || _stricmp(v, "2d") == 0)
                { forceView = 0; forceViewFrames = 4; }
            else if(_stricmp(v, "3d") == 0 || _stricmp(v, "scene") == 0)
                { forceView = 1; forceViewFrames = 4; }
            else if(_stricmp(v, "record") == 0 || _stricmp(v, "recorder") == 0)
                { forceView = 2; forceViewFrames = 4; }
            else if(_stricmp(v, "code") == 0 || _stricmp(v, "editor") == 0)
                { forceView = 3; forceViewFrames = 4; }
            else if(_stricmp(v, "range") == 0 ||
                     _stricmp(v, "tof") == 0)
                     {
                         forceView = RANGE_VIEW;
                         forceViewFrames = 4;
                     }
            else if(_stricmp(v, "drive") == 0 ||
                     _stricmp(v, "servo") == 0 ||
                     _stricmp(v, "esc") == 0)
                     {
                         forceView = DRIVE_VIEW;
                         forceViewFrames = 4;
                     }
            else if(_stricmp(v, "reference") == 0 ||
                     _stricmp(v, "ref") == 0 ||
                     _stricmp(v, "docs") == 0)
                     {
                         forceView = REF_VIEW;
                         forceViewFrames = 4;
                     }

            // Seed the live selection too, so the first frame reserves the
            // right bottom-bar height instead of the map's.
            if(forceView >= 0) centralView = forceView;
            continue;
        }

        if(std::strcmp(__argv[i], "--tab") == 0 && i + 1 < __argc)
        {
            struct TabName { const Char* name; Int32 sec; Int32 sub; };
            static const TabName TAB_NAMES[] = {
                { "system",   SECTION_SYSTEM,   -1 }, { "overview", SECTION_SYSTEM,   -1 },
                { "sensors",  SECTION_SENSORS,  -1 }, { "world",    SECTION_SENSORS,  -1 },
                { "lidar",    SECTION_SENSORS,  -1 },
                { "live",     SECTION_SENSORS,   0 }, { "signal",   SECTION_SENSORS,   1 },
                { "scan",     SECTION_SENSORS,   2 }, { "device",   SECTION_SENSORS,   3 },
                // "map" and "pico" are deliberately absent: they now name central
                // views (--view), and one word must not select two different things.
                { "vehicle",  SECTION_VEHICLE,  -1 },
                { "firmware", SECTION_FIRMWARE, -1 }, { "flash",    SECTION_FIRMWARE, -1 },
            };
            // "console" and "debug" no longer name a SECTION - the logs have
            // their own column now - so they open that instead of selecting
            // something in the sidebar.
            if(_stricmp(__argv[i + 1], "console") == 0
               || _stricmp(__argv[i + 1], "debug") == 0)
            {
                consoleOpen = true;
                continue;
            }

            for(const TabName& t : TAB_NAMES)
                if(_stricmp(__argv[i + 1], t.name) == 0)
                {
                    forceSection    = t.sec;
                    forceSub        = t.sub;
                    forceTabFrames = 4;
                }
            continue;
        }

        // --range <metres> pins the view instead of auto-fitting.
        if(std::strcmp(__argv[i], "--range") == 0 && i + 1 < __argc)
        {
            const Float32 m = static_cast<Float32>(std::atof(__argv[i + 1]));
            for(Int32 k = 0; k < RANGE_COUNT; ++k)
                if(RANGES[k].mm > 0.0f && std::fabs(RANGES[k].mm - m * 1000.0f) < 1.0f)
                    rangeIndex = k;
            applyRange();
            continue;
        }

        if(std::strcmp(__argv[i], "--connect") != 0) continue;

        if(i + 1 < __argc && __argv[i + 1][0] != '-')
        {
            const Char* want  = __argv[i + 1];
            Bool        found = false;
            for(Int32 p = 0; p < static_cast<Int32>(lidarPorts.size()); ++p)
            {
                if(_stricmp(lidarPorts[p].c_str(), want) == 0)
                {
                    portIndex = p;
                    found     = true;
                }
            }

            // A named port that is NOT enumerated is offered anyway, and the
            // connect is allowed to fail on it. Silently falling back to some
            // other port is worse in every way: --connect COM7 would talk to
            // whatever happened to be selected, and the script that asked for
            // COM7 would report success against the wrong device.
            if(!found)
            {
                lidarPorts.push_back(Str(want));
                portIndex = static_cast<Int32>(lidarPorts.size()) - 1;
            }
        }
        if(i + 2 < __argc && __argv[i + 2][0] != '-')
        {
            const Int32 b = std::atoi(__argv[i + 2]);
            for(Int32 k = 0; k < BAUD_COUNT; ++k)
            {
                if(BAUDS[k].rate == b)
                {
                    baudIndex = k;
                }
            }
        }
        break;
    }

    // Only when a port was actually identified. refreshPorts() leaves
    // portIndex at -1 when it cannot tell which port the lidar is on, and
    // auto-connecting anyway would open somebody else's device - which is
    // exactly how the lidar ended up holding the Pico's COM10.
    if(!suppress && portIndex >= 0)
    {
        connect();
    }
    else if(!suppress)
    {
        LOG_INFO("lidar", "no RPLIDAR adapter found; not auto-connecting");
    }

    // The Pico is the other half of "launched with no arguments, both devices
    // connected", which is what this app is documented to do - but only the
    // lidar was ever wired up here. The board view made the omission obvious:
    // it has nothing live to show until the link is open, and every launch was
    // opening it closed. --no-connect suppresses both, as it always has.
    if(!suppress) connectPico();
}

Void app::notifyDeviceChange()
{
    // Called from the window procedure, so it touches nothing but an atomic and
    // returns immediately. The rescan itself happens on the UI thread in
    // pumpDeviceScan(), where it is safe to touch the port lists.
    deviceChangePending.store(true, std::memory_order_release);
}

Void app::setDpiScale(Float32 dpiScale)
{
    uiDpiScale = dpiScale > 0.0f ? dpiScale : 1.0f;
}

// Works the lamps out from the drive state, once a frame.
//
// The inputs are what the BOARD reports it is doing - driveSteer and driveEsc -
// not what the sliders are asking for. The difference is the slew limiter: a
// slider dragged to full lock arrives instantly, the servo takes a second to
// get there, and it is the servo the indicator should follow. Reading the
// targets would flash the indicator before the wheels had moved.
Void updateAutoLights()
{
    if(!autoLights)
    {
        return;
    }

    lights::Drive d;
    d.steer      = driveSteer;
    d.throttleUs = driveEsc;
    d.armed      = driveArmed;

    lightInput = lights::detect(autoLightState, d, ImGui::GetTime());
}

Void app::frame()
{
    pumpData();
    updateAutoLights();

    // Ctrl+` for the console, which is the shortcut every editor and terminal
    // already uses for exactly this. Checked before anything draws so the
    // layout below measures the state it is about to render.
    if(ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_GraveAccent))
    {
        consoleOpen      = !consoleOpen;
        panelLayoutDirty = true;
    }

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);

    ImGui::Begin("##root", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    const ImGuiStyle& sty = ImGui::GetStyle();

    // ---- the status bar's height, reserved now and DRAWN LAST ------------
    //
    // Sized to whichever is taller, the type or the icons, plus its padding.
    // Sizing to the text alone is what cropped the old lamps.
    //
    // It lives at the BOTTOM. Along the top it competed with the tab bar for
    // the eye and pushed the actual work down; a status bar is ambient
    // information you glance at, not a header you read first.
    const Float32 stripPad = 6.0f * uiDpiScale;
    const Float32 stripH   = std::max(ImGui::GetTextLineHeight(), ui::iconSize())
                           + stripPad * 2.0f;

    // ---- map + sidebar ---------------------------------------------------
    // Everything above the bar lays out against the height that is left once
    // the bar has taken its own.
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y -= stripH + sty.ItemSpacing.y;

    // The column's width is the user's now - see sidebarSplitter(). It is still
    // a LOGICAL width, so the sidebar's contents lay out against a number that
    // does not move when the DPI or the zoom does.
    const Float32 gap   = std::max(sty.ItemSpacing.x, 8.0f * uiDpiScale);
    const Float32 sideW = sidebarWidth(avail.x);

    // The console column, when it is open, comes off the LEFT before anything
    // else is measured - so the central view narrows and the sidebar does not
    // move. A panel that shoved the sidebar around every time it opened would
    // make the whole window feel unstable.
    const Float32 consW = consoleOpen ? consoleWidth(avail.x) : 0.0f;
    const Float32 consGap = consoleOpen ? gap : 0.0f;

    // Sized for the view that is on screen, which is why centralView is kept
    // across frames. A view with no controls costs no height at all - the
    // spacing goes too, or a board tab would sit above a blank strip.
    const Float32 mapW  = avail.x - sideW - gap - consW - consGap;
    const Float32 ctrlH = centralControlHeight(centralView, mapW);
    const Float32 mapH  = avail.y - ctrlH - (ctrlH > 0.0f ? sty.ItemSpacing.y : 0.0f);

    const ImVec2 p0 = ImGui::GetCursorScreenPos();

    if(consoleOpen)
    {
        drawConsoleColumn(consW, avail.y);
        consoleSplitter(ImVec2(p0.x + consW, p0.y), avail.y, consGap);
        ImGui::SetCursorScreenPos(ImVec2(p0.x + consW + consGap, p0.y));
    }

    const Float32 midX = p0.x + consW + consGap;

    if(mapW > 80.0f * uiDpiScale && mapH > 80.0f * uiDpiScale)
    {
        // The central region gets its own child window, and it has to.
        //
        // Everything inside it begins with a tab bar, and ImGui puts the cursor
        // back at the WINDOW's left content edge after one - not at wherever the
        // caller had moved it. With the console column open that edge is 400 px
        // to the left of where the view should start, so the tab bar landed
        // correctly and the body underneath it did not. A child makes "the
        // window's left edge" mean this column's left edge, which is the only
        // way to say it that a tab bar cannot undo.
        //
        // Zero padding on the child so mapW is exactly the width the view gets;
        // popped immediately, so the widgets inside still get the normal one.
        ImGui::SetCursorScreenPos(ImVec2(midX, p0.y));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::BeginChild("##centralcol", ImVec2(mapW, avail.y), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::PopStyleVar();

        drawMapRegion(mapW, mapH, ctrlH);

        ImGui::EndChild();

        sidebarSplitter(ImVec2(midX + mapW, p0.y), avail.y, gap);
        ImGui::SetCursorScreenPos(ImVec2(midX + mapW + gap, p0.y));
    }

    drawSidebar(sideW, avail.y);

    // ---- the status bar, last --------------------------------------------
    //
    // Pinned to the window's bottom edge rather than flowed after the sidebar,
    // because the two columns above it do not necessarily end at the same y and
    // a bar that followed whichever was taller would move as panels opened.
    {
        const ImVec2 wp = ImGui::GetWindowPos();
        const ImVec2 ws = ImGui::GetWindowSize();

        ImGui::SetCursorScreenPos(ImVec2(wp.x + sty.WindowPadding.x,
                                         wp.y + ws.y - sty.WindowPadding.y - stripH));

        // A hairline above it, so the bar reads as chrome rather than as more
        // content that happens to be at the bottom.
        ImGui::GetWindowDrawList()->AddLine(
            ImVec2(wp.x, wp.y + ws.y - sty.WindowPadding.y - stripH - 1.0f),
            ImVec2(wp.x + ws.x, wp.y + ws.y - sty.WindowPadding.y - stripH - 1.0f),
            ImGui::GetColorU32(ImGuiCol_Separator));

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,
                            ImVec2(12.0f * uiDpiScale, stripPad));
        ImGui::BeginChild("##statusbar",
                          ImVec2(ws.x - sty.WindowPadding.x * 2.0f, stripH),
                          ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar
                          | ImGuiWindowFlags_NoScrollWithMouse);
        drawStatusBar();
        ImGui::EndChild();
        ImGui::PopStyleVar();
    }

    // ---- the recorder's frame source -------------------------------------
    //
    // Live while idle or capturing, recorded while playing or scrubbed. Done
    // here rather than in the tab body so a recording keeps advancing while you
    // are looking at another tab - stopping playback because you glanced at the
    // board view would be surprising.
    if(!recording.empty() && (recPlaying || recPendingSeek))
    {
        if(recPlaying)
        {
            recPlayS += static_cast<Float64>(ImGui::GetIO().DeltaTime);
            if(recPlayS >= recording.durationS())
            {
                recPlayS   = recording.durationS();
                recPlaying = false;    // holds on the last frame
            }
        }
        recIndex = recording.indexAt(recPlayS);
        recPendingSeek = false;

        const rec::Rev& r = recording.at(recIndex);
        LidarFrame lf;
        lf.points = r.points;
        lf.hz     = r.hz;
        for(const LidarPoint& p : lf.points)
            if(p.distMm > 0.0f) ++lf.validCount;
        recView.push(lf);
    }

    // The preselect has to persist a few frames: a tab bar only honours
    // SetSelected once it has laid its items out, which is not on frame one.
    if(forceTabFrames > 0) --forceTabFrames;

    drawGlobalModals();

    ImGui::End();

    // After the root window ends, so a torn-off panel floats above the whole app
    // instead of being clipped into the column it came from.
    drawFloatingPanels();

    if(panelLayoutDirty)
    {
        savePanelLayout();
        panelLayoutDirty = false;
    }
}

Void app::shutdown()
{
    LOG_INFO("app", "shutdown requested");
    lidarSource.stop();
    picoLink.disconnect();

    // Last, so anything the two lines above logged on their way out is in the
    // file before it closes.
    applog::shutdown();
}
