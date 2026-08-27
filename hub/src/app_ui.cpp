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
#include "shared.hpp"
#include "app_ui.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cstring>
#include <ctime>

#include "imgui.h"
#include "lidar_source.hpp"
#include "pico_flash.hpp"
#include "pico_link.hpp"
#include "board_view.hpp"
#include "radar.hpp"
#include "icons.hpp"
#include "lights.hpp"
#include "applog.hpp"
#include "diagnostics.hpp"
#include "code_view.hpp"
#include "editor.hpp"
#include "recording.hpp"
#include "sketch.hpp"
#include "reference.hpp"
#include "devlink.hpp"
#include "lint.hpp"
#include "workspace.hpp"
#include "settings.hpp"
#include "theme.hpp"

namespace {

LidarSource lidarSource;
RadarView   radarView;
PicoLink    picoLink;
PicoFlash   picoFlash;

Vec<Str> lidarPorts;
Vec<const Char*> portItems;
Int32   portIndex  = -1;
Int32   baudIndex  = 2;     // 460800
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
// exactly like dead hardware and is not. pico_link.cpp asserts DTR; do not
// remove it.

Vec<Str> picoPorts;
Vec<const Char*> picoItems;
Int32  picoIndex = -1;

// The console is a debug aid, not a record: this app runs for hours, so the log
// is bounded and the oldest lines fall off the front.
constexpr Size LOG_MAX = 4000;
Vec<PicoLine> picoLog;
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
// vocabulary in firmware/src/main.c. Kept separate from VehicleStatus above
// because the two come from DIFFERENT firmware: pico_debug answers STATUS,
// tt02_control answers `?`, and neither understands the other's command. A
// board runs one of them, so at most one of these two structs is ever live.
struct DebugStatus
{
    board::Live::Tri  cyw43  = board::Live::Tri::TRI_UNKNOWN;
    board::Live::Led  led    = board::Live::Led::LED_UNKNOWN;
    Float32             ledHz = 0.0f;
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
    SECTION_CONSOLE,
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
Int32 sectionOrder[SECTION_COUNT] = { SECTION_SYSTEM, SECTION_SENSORS, SECTION_VEHICLE,
                                  SECTION_FIRMWARE, SECTION_CONSOLE };
Bool  sectionFloating[SECTION_COUNT] = {};

// Logical (96-dpi) pixels, so the column keeps its apparent width across a DPI
// change or a zoom rather than growing in one and not the other.
Float32 sidebarLogicalW = 400.0f;

constexpr Float32 SIDEBAR_MIN_W = 260.0f;   // narrower than this and rows wrap

// The Code view's file tree. LOGICAL pixels, like the sidebar, so a drag feels
// the same at 100% and at 200% and the stored value survives a DPI change.
constexpr Float32 CODE_TREE_MIN_W = 130.0f;   // below this the file names clip
constexpr Float32 CODE_TREE_MAX_W = 640.0f;
constexpr Float32 CODE_TREE_DEF_W = 240.0f;

Float32 codeTreeLogicalW  = CODE_TREE_DEF_W;
Bool    codeTreeCollapsed = false;

// ---- the central region's layout ------------------------------------------
// 0 = tabbed (one view, full width), 1 = floating (a board of panels you
// arrange). Both render the SAME view bodies - see drawViewBody() - so a panel
// and a tab are never two implementations of one picture.
Int32       layoutFloating = 0;
ws::Panel   wsPanels[ws::PANEL_COUNT];

// The reference library's browsing state - which page, the drawer width, the
// zoom. Held here rather than inside the module so the panel stays re-entrant.
ref::State  refView;
ws::Canvas  wsCanvas;
Bool        wsInit = false;

// Which panel the bottom control bar belongs to: the last one clicked. In the
// tabbed layout that is just the open tab, and centralView carries it.
Int32 wsFocused = 0;

// Live drag state. -1 when nothing is being dragged.
Int32   wsDragPanel = -1;

// 0 = move. Otherwise a bitmask: 1 = width, 2 = height, 3 = both.
Int32   wsDragEdge = 0;
ImVec2  wsDragOrigin(0.0f, 0.0f);
ws::Rect wsDragRect0;
Bool    wsPanning = false;

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
        else if(line.size() > 2 && line[0] == 'L')
        {
            layoutFloating = (std::atoi(line.c_str() + 1) != 0) ? 1 : 0;
        }
        else if(line.size() > 2 && line[0] == 'C')
        {
            Float64 z = 1.0;
            Float64 px = 0.0;
            Float64 py = 0.0;
            if(std::sscanf(line.c_str() + 1, "%lf %lf %lf", &z, &px, &py) == 3)
            {
                wsCanvas.zoom = ws::clampZoom(static_cast<Float32>(z));
                wsCanvas.panX = static_cast<Float32>(px);
                wsCanvas.panY = static_cast<Float32>(py);
            }
        }
        else if(line.size() > 2 && line[0] == 'P')
        {
            Int32   idx = -1;
            Float64 x = 0.0, y = 0.0, w = 0.0, h = 0.0;
            Int32   op = 1, cl = 0, zz = 0;
            if(std::sscanf(line.c_str() + 1, "%d %lf %lf %lf %lf %d %d %d",
                           &idx, &x, &y, &w, &h, &op, &cl, &zz) == 8
               && idx >= 0 && idx < ws::PANEL_COUNT)
            {
                ws::Panel& p = wsPanels[idx];
                p.rect.x    = static_cast<Float32>(x);
                p.rect.y    = static_cast<Float32>(y);
                p.rect.w    = std::max(ws::PANEL_MIN_W, static_cast<Float32>(w));
                p.rect.h    = std::max(ws::PANEL_MIN_H, static_cast<Float32>(h));
                p.open      = (op != 0);
                p.collapsed = (cl != 0);
                p.z         = zz;
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

    // The panel z order has to be a dense permutation - hitTest resolves
    // overlap with it. A hand-edited or truncated file could give duplicates, so
    // it is normalised here rather than trusted.
    {
        Int32 idx[ws::PANEL_COUNT];
        for(Int32 k = 0; k < ws::PANEL_COUNT; ++k)
        {
            idx[k] = k;
        }
        std::sort(idx, idx + ws::PANEL_COUNT,
                  [](Int32 a, Int32 b) { return wsPanels[a].z < wsPanels[b].z; });
        for(Int32 k = 0; k < ws::PANEL_COUNT; ++k)
        {
            wsPanels[idx[k]].z = k;
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

    std::snprintf(buf, sizeof(buf), "L %d\n", layoutFloating);
    out += buf;
    std::snprintf(buf, sizeof(buf), "C %.4f %.1f %.1f\n",
                  static_cast<Float64>(wsCanvas.zoom),
                  static_cast<Float64>(wsCanvas.panX),
                  static_cast<Float64>(wsCanvas.panY));
    out += buf;
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        const ws::Panel& p = wsPanels[i];
        std::snprintf(buf, sizeof(buf), "P %d %.1f %.1f %.1f %.1f %d %d %d\n",
                      i,
                      static_cast<Float64>(p.rect.x),
                      static_cast<Float64>(p.rect.y),
                      static_cast<Float64>(p.rect.w),
                      static_cast<Float64>(p.rect.h),
                      p.open ? 1 : 0, p.collapsed ? 1 : 0, p.z);
        out += buf;
    }
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
// A sketch is edited here, written to firmware/src/sketch.c, and built and
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

// Rolling rotation-rate history for the sparkline.
constexpr Int32 HISTORY = 240;
Float32 hzHist[HISTORY] = {};
Int32   hzCount = 0;

const Int32   BAUDS[]      = { 115200, 256000, 460800 };
const Char* BAUD_ITEMS[]  = { "115200", "256000", "460800" };

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
    const dev::PortKind k = dev::portKind(port);
    return k == dev::PortKind::PORT_KIND_CP210X
        || k == dev::PortKind::PORT_KIND_UNKNOWN;
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
    // "INFO status up_ms=... led=on blink_hz=2.00 cyw43=up"
    // "INFO id board=... cyw43=up"
    // Scanned for the fields rather than parsed positionally: the two lines
    // carry an overlapping subset and the order is the firmware's business.
    if(t.compare(0, 5, "INFO ") == 0)
    {
        const Char* s = t.c_str();

        if(const Char* q = std::strstr(s, "cyw43="))
        {
            debugStatus.cyw43 = (std::strncmp(q + 6, "up", 2) == 0)
                        ? board::Live::Tri::TRI_YES : board::Live::Tri::TRI_NO;
        }

        if(const Char* q = std::strstr(s, "led="))
        {
            debugStatus.led = (std::strncmp(q + 4, "on", 2) == 0)
                      ? board::Live::Led::LED_ON : board::Live::Led::LED_OFF;
        }

        // blink_hz is authoritative over led= : the firmware reports the LED's
        // instantaneous level AND its blink rate, so a non-zero rate means it is
        // blinking whichever half of the cycle the sample happened to catch.
        if(const Char* q = std::strstr(s, "blink_hz="))
        {
            const Float32 hz = static_cast<Float32>(std::atof(q + 9));
            if(hz > 0.0f)
            {
                debugStatus.led    = board::Live::Led::LED_BLINK;
                debugStatus.ledHz = hz;
            }
            else
            {
                debugStatus.ledHz = 0.0f;
            }
        }

        dbgUnsupported = false;
        dbgAwait       = false;
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

    // "OK led on" / "OK led off" / "OK led blink 2.00" - the acknowledgement,
    // so a command the user typed takes effect on the drawing immediately
    // rather than at the next poll.
    if(t.compare(0, 7, "OK led ") == 0)
    {
        const Char* a = t.c_str() + 7;
        if(std::strncmp(a, "blink", 5) == 0)
        {
            const Float32 hz = static_cast<Float32>(std::atof(a + 5));
            debugStatus.led    = (hz > 0.0f) ? board::Live::Led::LED_BLINK : board::Live::Led::LED_OFF;
            debugStatus.ledHz = (hz > 0.0f) ? hz : 0.0f;
        }
        else if(std::strncmp(a, "on", 2) == 0)
        {
            debugStatus.led = board::Live::Led::LED_ON;  debugStatus.ledHz = 0.0f;
        }
        else if(std::strncmp(a, "off", 3) == 0)
        {
            debugStatus.led = board::Live::Led::LED_OFF; debugStatus.ledHz = 0.0f;
        }
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
    for(Size i = before; i < picoLog.size(); ++i) observeLine(picoLog[i]);

    if(picoLog.size() > LOG_MAX)
        picoLog.erase(picoLog.begin(), picoLog.begin() + (picoLog.size() - LOG_MAX));
}

// What the board view draws. Assembled here rather than in board_view.cpp so
// that file stays a drawing and knows nothing about serial links or flash tools.
board::Live boardLive()
{
    board::Live lv;

    const BoardStatus brd = picoFlash.board();
    lv.link       = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);
    lv.bootsel    = brd.bootsel;
    lv.fwPresent = brd.present;

    lv.cyw43  = debugStatus.cyw43;
    lv.led    = debugStatus.led;
    lv.ledHz = debugStatus.ledHz;

    // Only claim the pins are being driven while the reports are still arriving.
    // A stale S line from a board that has since been unplugged would otherwise
    // leave two pads ringed green forever.
    if(vehicleStatus.have && (ImGui::GetTime() - vehicleStatus.seenAt) < 3.0)
    {
        lv.havePwm = true;
        lv.servoUs = vehicleStatus.servoUs;
        lv.escUs   = vehicleStatus.escUs;
    }

    return lv;
}

// Asks the board what it is doing, at most every couple of seconds, and only
// while the board view is actually on screen. Polling a debug link that nobody
// is looking at would fill the console with traffic the user did not ask for.
Void pollBoardStatus()
{
    if(dbgUnsupported) return;
    if(picoLink.state() != PicoState::PICO_STATE_CONNECTED) return;

    const Float64 now = ImGui::GetTime();
    if(dbgLastPoll > 0.0 && (now - dbgLastPoll) < 2.0) return;

    dbgLastPoll = now;
    sendPico("STATUS");
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
    sendPico("SENSORS");
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
    sendPico("TOF");
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
// on the two palettes in theme.hpp.
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
// WHY THIS IS SEPARATE FROM THE COMPILER. diagnostics.hpp reports what the
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
             BAUDS[baudIndex]);
    if(portIndex < 0 || portIndex >= static_cast<Int32>(lidarPorts.size())) return;

    radarView.clear();
    haveFrame = false;
    hzCount   = 0;
    lidarSource.start(lidarPorts[portIndex], BAUDS[baudIndex]);
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
                      lidarPorts[portIndex].c_str(), BAUDS[baudIndex]);
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

Void stripField(BarPen& p, ui::Icon ic, const Char* label, ImU32 col,
                const Char* value, const Char* extra)
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
    const Float32 bh = ImGui::GetFrameHeight();

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
    const ImVec2 psz = ImGui::CalcTextSize(pct);
    const Float32 px = x0 + btnW + gap;
    ImGui::GetWindowDrawList()->AddText(ImVec2(px, cy - psz.y * 0.5f),
                                        ImGui::GetColorU32(ImGuiCol_TextDisabled),
                                        pct);

    // An invisible hit box over it, so the click-to-reset and the tooltip still
    // work now that the text is drawn rather than submitted.
    ImGui::SetCursorScreenPos(ImVec2(px, cy - psz.y * 0.5f));
    ImGui::InvisibleButton("##zoompct", ImVec2(std::max(pctW, psz.x), psz.y));
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
Void subsystemRow(ui::Icon ic, const Char* name, ImU32 col, const Char* state,
                  const Char* value, Bool lit = true)
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
        subsystemRow(ui::Icon::ICON_FIRMWARE, "Board firmware", ui::sem::GOOD,
                     brd.program.empty() ? "running" : brd.program.c_str(),
                     brd.chip.c_str());
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

    ImGui::SetNextItemWidth(-FLT_MIN);
    ui::combo("##baud", &baudIndex, BAUD_ITEMS, 3);
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
Void sensorRow(Int32 index, Bool wired, Bool* vis, const Char* name, ImU32 col,
               const Char* state)
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

        // The dToF core drifts with die temperature, so ranges are not
        // trustworthy for the first two minutes. That is a value, not a lecture.
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
// Writes the buffer to the sketch library AND to firmware/src/sketch.c.
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
                ImGui::SetTooltip("firmware/src is tracked in git - delete it there");
            }

            ImGui::EndPopup();
        }

        ImGui::PopID();
        return hit;
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

    if(ImGui::TreeNodeEx("firmware/src", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const Str d = sketch::firmwareDir();
        for(const Str& n : fwFiles)
        {
            const Str p = d + "\\" + n;
            const Bool hdr = (n.size() > 2 && n.compare(n.size() - 2, 2, ".h") == 0);
            if(row(n, p, _stricmp(p.c_str(), codePath.c_str()) == 0,
                   hdr ? ui::Icon::ICON_FIRMWARE : ui::Icon::ICON_CODE, false))
            {
                openCodeFile(p, n);
            }
        }
        if(fwFiles.empty())
            ImGui::TextDisabled("  repo not found");
        ImGui::TreePop();
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
        // from its label - so a sketch named sketch.c and firmware/src/sketch.c
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
        ImGui::TextDisabled("firmware/src");

        const Str      fwd  = sketch::firmwareDir();
        const Str      slot = sketch::slotPath();
        const Vec<Str> fws  = sketch::listFirmware();
        for(const Str& n : fws)
        {
            const Bool hdr = (n.size() > 2 && n.compare(n.size() - 2, 2, ".h") == 0);
            const Str  p   = fwd + "\\" + n;

            // firmware/src/sketch.c is the scratch slot every library sketch is
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
// Views 0-3 are the fixed ones, then one per board, then the reference
// library. Appended at the end so every index already in a saved layout keeps
// meaning what it meant.
constexpr Int32 BOARD_VIEW_0 = 4;
constexpr Int32 REF_VIEW =
    BOARD_VIEW_0 + static_cast<Int32>(board::Which::WHICH_COUNT);
constexpr Int32 RANGE_VIEW = REF_VIEW + 1;
constexpr Int32 VIEW_COUNT = RANGE_VIEW + 1;

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
                      IM_COL32(0x0E, 0x0F, 0x12, 0xFF));

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
            what = "no VL53L1X found at 0x29";
            col  = ui::sem::BAD;
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
        ImGui::SameLine(0.0f, 2.0f);
        if(ui::segmentedButton("Long", !tofModeShort,
                               ImVec2(88.0f * uiDpiScale, 0.0f)))
        {
            tofModeShort = false;
            sendPico("TOF MODE LONG");
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
        dl->AddText(ImVec2(p0.x + pad, top + 8.0f * uiDpiScale),
                    ui::sem::MUTED, hint);
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
                    good ? IM_COL32(0xE8, 0xE4, 0xDA, 0xFF) : ui::sem::MUTED, buf);

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
            // The rates are in the sensor's 16.16 fixed point; the top 16 bits
            // are whole mega-counts per second, which is all the resolution
            // worth showing.
            if(tofSignal >= 0)
            {
                std::snprintf(buf, sizeof(buf),
                              "seen %d - %d mm   %llu readings   "
                              "signal %d   ambient %d",
                              tofSeenMin, tofSeenMax,
                              static_cast<unsigned long long>(tofReplies),
                              tofSignal >> 7, tofAmbient >> 7);
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

            if(tofAmbient > (tofSignal * 8) && tofAmbient > 32)
            {
                why = "Ambient light is swamping the signal - try Short mode, "
                      "or move away from a window or lamp.";
            }
            else if(tofSeenMax > 0 && tofSeenMax < 200
                    && (tofSeenMax - tofSeenMin) < 60
                    && tofSignal > 64)
            {
                why = "Short, steady and a strong return - is the protective "
                      "film still on the lens?";
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

Void drawViewBody(Int32 view, Float32 w, Float32 h)
{
    const ImVec2 p0 = ImGui::GetCursorScreenPos();

    // Explicitly black, matching every other surface. An explicit push rather
    // than inherited, because the map is the one panel whose background must
    // never pick up a tint or an alpha - it is the surface the point cloud is
    // read against.
    ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0x0E, 0x0F, 0x12, 0xFF));
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
    else if(view == RANGE_VIEW)
    {
        // Polled only while this view is drawn - which is to say, only while
        // somebody is looking at it. See pollTof().
        pollSensorList();
        pollTof(true);
        drawRangeBody(w, h);
    }
    else if(view == REF_VIEW)
    {
        ImGui::BeginChild("##ref", ImVec2(w, h), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar
                          | ImGuiWindowFlags_NoScrollWithMouse);
        const ImVec2 rp0 = ImGui::GetCursorScreenPos();
        ref::draw(refView, ImGui::GetContentRegionAvail());
        ui::screenInset(rp0, ImVec2(rp0.x + w, rp0.y + h));
        ImGui::EndChild();
    }
    else
    {
        const board::Which which =
            static_cast<board::Which>(std::max(0, view - BOARD_VIEW_0));

        ImGui::BeginChild("##board", ImVec2(w, h), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        // Polled inside the body so it only runs while the view is on screen.
        pollBoardStatus();
        const ImVec2 bp0 = ImGui::GetCursorScreenPos();
        board::draw(which, ImGui::GetContentRegionAvail(), boardLive());
        ui::screenInset(bp0, ImVec2(bp0.x + w, bp0.y + h));
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
    return board::name(static_cast<board::Which>(std::max(0, view - BOARD_VIEW_0)));
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
    return (view == REF_VIEW) ? ui::Icon::ICON_REFERENCE : ui::Icon::ICON_PROCESSOR;
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

    // Right-aligned, so switching layout is where the tabs end rather than in a
    // second row of chrome above them.
    if(ImGui::TabItemButton("  Float  ", ImGuiTabItemFlags_Trailing
                                       | ImGuiTabItemFlags_NoTooltip))
    {
        layoutFloating   = 1;
        panelLayoutDirty = true;
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Arrange the views as floating panels instead");
    }

    ImGui::EndTabBar();
}

// =================================================== the floating layout

// Title-bar height in SCREEN space. Scales with zoom so a zoomed-out board
// looks right, but never below a size you can actually aim at - a 4 px title
// bar is a panel you cannot move.
Float32 wsTitleHeight()
{
    const Float32 want = 26.0f * uiDpiScale * wsCanvas.zoom;
    return std::max(16.0f * uiDpiScale, want);
}

Void drawFloatingWorkspace(Float32 mapW, Float32 viewH)
{
    ImGuiIO& io = ImGui::GetIO();

    if(!wsInit)
    {
        wsInit = true;
        // Only lay out afresh when nothing was restored, so a saved arrangement
        // survives. defaultLayout leaves z dense, which hitTest relies on.
        Bool any = false;
        for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
        {
            if(wsPanels[i].rect.w > 1.0f)
            {
                any = true;
            }
        }
        if(!any)
        {
            ws::defaultLayout(wsPanels, ws::PANEL_COUNT);
            ws::fitAll(wsPanels, ws::PANEL_COUNT, wsCanvas, mapW, viewH);
        }
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();

    ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0x16, 0x18, 0x1B, 0xFF));
    ImGui::BeginChild("##canvas", ImVec2(mapW, viewH), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImDrawList* dl = ImGui::GetWindowDrawList();

    // ---- the board itself, so panning has something to move against -------
    {
        const Float32 step = 48.0f * wsCanvas.zoom;
        if(step > 6.0f)
        {
            const ImU32 dot = IM_COL32(0x2A, 0x2D, 0x31, 0xFF);
            const Float32 ox = std::fmod(wsCanvas.panX, step);
            const Float32 oy = std::fmod(wsCanvas.panY, step);
            for(Float32 x = ox; x < mapW; x += step)
            {
                for(Float32 y = oy; y < viewH; y += step)
                {
                    if(x < 0.0f || y < 0.0f)
                    {
                        continue;
                    }
                    dl->AddRectFilled(ImVec2(origin.x + x, origin.y + y),
                                      ImVec2(origin.x + x + 1.0f, origin.y + y + 1.0f),
                                      dot);
                }
            }
        }
    }

    // ---- canvas-level input -----------------------------------------------
    //
    // NO invisible button over the canvas, deliberately. One was tried and it
    // broke panel dragging outright: a full-size button takes ImGui's ActiveId
    // on press, and an item that owns ActiveId makes every later overlapping
    // item non-hoverable - so the title bars submitted after it never saw their
    // own click and every panel drag panned the board instead.
    //
    // IsWindowHovered() WITHOUT ChildWindows is the right test anyway: it is
    // true over empty canvas and over title bars, and false over a panel's body.
    // That is exactly the split we want, because a wheel over a map belongs to
    // the map's own zoom, not to the board's.
    // What the mouse is over, resolved OURSELVES rather than asked of ImGui.
    //
    // Now that a whole panel is one child window, IsWindowHovered() on the
    // canvas is false over a panel's title bar as well as its body, so it can no
    // longer tell "empty board" from "a panel's chrome". ws::hitTest can, and it
    // is the same front-to-back resolution the panels are drawn with, so the two
    // cannot disagree about what is under the cursor.
    const Float32 titleHPre = wsTitleHeight();
    Float32 hitH[ws::PANEL_COUNT];
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        const ws::Rect sr = ws::toScreen(wsPanels[i].rect, wsCanvas, origin.x, origin.y);
        hitH[i] = wsPanels[i].collapsed
                ? titleHPre
                : titleHPre + std::max(0.0f, sr.h - titleHPre);
    }
    const Int32 under = ws::hitTest(wsPanels, ws::PANEL_COUNT, wsCanvas,
                                    origin.x, origin.y, hitH,
                                    io.MousePos.x, io.MousePos.y);

    // Anywhere inside the canvas region, panels included.
    const Bool inCanvas = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);

    // Over a panel's title bar rather than its body.
    Bool onChrome = false;
    if(under >= 0)
    {
        const ws::Rect sr = ws::toScreen(wsPanels[under].rect, wsCanvas,
                                         origin.x, origin.y);
        onChrome = (io.MousePos.y < sr.y + titleHPre);
    }

    // The wheel belongs to the BOARD over empty canvas and over a title bar, and
    // to the PANEL over its body - a wheel on a map is the map's own zoom, and
    // taking it away to move the board would be worse than useless.
    if(inCanvas && (under < 0 || onChrome) && io.MouseWheel != 0.0f)
    {
        const Float32 f = (io.MouseWheel > 0.0f) ? ws::ZOOM_STEP : 1.0f / ws::ZOOM_STEP;
        ws::zoomAt(wsCanvas, f, io.MousePos.x, io.MousePos.y, origin.x, origin.y);
        panelLayoutDirty = true;
    }

    // Panning starts only when the press did NOT land on a panel.
    //
    // An invisible background button cannot answer that: it would take ImGui's
    // ActiveId on press, and an item holding ActiveId makes every later
    // overlapping item non-hoverable - so the panels submitted after it would
    // never see their own clicks. That is exactly why dragging a panel used to
    // pan the whole board.
    if(inCanvas && under < 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        wsPanning = true;
    }
    // Middle-drag pans from anywhere, panel or not - the usual escape hatch for
    // a board where the empty space has run out.
    if(inCanvas && ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
    {
        wsPanning = true;
    }
    if(!ImGui::IsMouseDown(ImGuiMouseButton_Left)
       && !ImGui::IsMouseDown(ImGuiMouseButton_Middle))
    {
        wsPanning = false;
    }
    if(wsPanning)
    {
        wsCanvas.panX += io.MouseDelta.x;
        wsCanvas.panY += io.MouseDelta.y;
        panelLayoutDirty = true;
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    }

    // ---- the panels, back to front ----------------------------------------
    // Submitting in z order means the front-most panel's widgets are submitted
    // LAST, and ImGui gives the last submitted item at a position the hover -
    // so overlap resolves the way it looks.
    Int32 order[ws::PANEL_COUNT];
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        order[i] = i;
    }
    std::sort(order, order + ws::PANEL_COUNT,
              [](Int32 a, Int32 b) { return wsPanels[a].z < wsPanels[b].z; });

    const Float32 titleH = wsTitleHeight();
    Int32 raise = -1;

    for(Int32 oi = 0; oi < ws::PANEL_COUNT; ++oi)
    {
        const Int32 i = order[oi];
        ws::Panel&  p = wsPanels[i];
        if(!p.open)
        {
            continue;
        }

        const ws::Rect sr = ws::toScreen(p.rect, wsCanvas, origin.x, origin.y);
        const Float32  bodyH = p.collapsed ? 0.0f : std::max(0.0f, sr.h - titleH);
        const Float32  fullH = titleH + bodyH;

        // Off screen entirely: skip the whole panel, content included. This is
        // what keeps a large board cheap - an unseen map is not drawn.
        if(sr.x + sr.w < origin.x - 4.0f || sr.x > origin.x + mapW + 4.0f
           || sr.y + fullH < origin.y - 4.0f || sr.y > origin.y + viewH + 4.0f)
        {
            continue;
        }

        ImGui::PushID(i);

        const Bool focused = (wsFocused == i);

        // ---- ONE CHILD WINDOW PER PANEL, frame included --------------------
        //
        // This is what makes the stacking work, and the reason is not obvious.
        //
        // ImGui renders a parent window's whole draw list FIRST and every child
        // window afterwards. With the frames drawn into the canvas list and only
        // the bodies in children, a panel could never truly be "on top": its
        // frame and title bar sat under every other panel's content, however
        // recently it had been clicked. The order was right and the layering was
        // not.
        //
        // Wrapping each panel - frame, title, handles and content - in its own
        // child makes it a single unit. Children are sorted by
        // BeginOrderWithinParent (see ChildWindowComparer in imgui.cpp), which
        // IS the order they are submitted in, and they are submitted here in z
        // order. So the panel stacks whole.
        ImGui::SetCursorScreenPos(ImVec2(sr.x, sr.y));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::BeginChild("##panel",
                          ImVec2(std::max(1.0f, sr.w), std::max(1.0f, fullH)),
                          ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar
                          | ImGuiWindowFlags_NoScrollWithMouse
                          | ImGuiWindowFlags_NoBackground);

        // The panel's OWN draw list. Using the canvas list here would put the
        // frame back under every other panel's body, which is the bug above.
        ImDrawList* pdl = ImGui::GetWindowDrawList();

        // Frame and title bar. The border is inset by a pixel so the child's
        // clip rect does not shave it in half.
        pdl->AddRectFilled(ImVec2(sr.x, sr.y), ImVec2(sr.x + sr.w, sr.y + fullH),
                           IM_COL32(0x1E, 0x21, 0x25, 0xFF), 4.0f * uiDpiScale);
        pdl->AddRectFilled(ImVec2(sr.x, sr.y), ImVec2(sr.x + sr.w, sr.y + titleH),
                           focused ? IM_COL32(0x3A, 0x44, 0x50, 0xFF)
                                   : IM_COL32(0x2A, 0x2E, 0x34, 0xFF),
                           4.0f * uiDpiScale, ImDrawFlags_RoundCornersTop);
        pdl->AddRect(ImVec2(sr.x + 1.0f, sr.y + 1.0f),
                     ImVec2(sr.x + sr.w - 1.0f, sr.y + fullH - 1.0f),
                     focused ? ui::accent::CYAN : IM_COL32(0x3A, 0x3F, 0x45, 0xFF),
                     4.0f * uiDpiScale, 0, focused ? 2.0f : 1.0f);

        // Clicking anywhere in the panel raises it - the title bar is not the
        // only part of a window you expect to be able to bring forward.
        if(ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows)
           && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            raise = i;
        }

        // ---- title bar: drag to move, double-click to collapse ------------
        //
        // The drag area STOPS SHORT of the fold and close buttons. It used to
        // span the whole bar, and that silently disabled both of them: an item
        // that owns ImGui's ActiveId makes every later overlapping item
        // non-hoverable, so the buttons submitted after it never saw a click.
        // Two widgets cannot share a rectangle; the drag area gives way.
        const Float32 btn = std::min(titleH - 4.0f * uiDpiScale, 16.0f * uiDpiScale);
        const Float32 btnZone = (btn > 6.0f)
                              ? (btn * 2.0f + 14.0f * uiDpiScale) : 0.0f;
        const Float32 bx = sr.x + sr.w - btn * 2.0f - 10.0f * uiDpiScale;

        ImGui::SetCursorScreenPos(ImVec2(sr.x, sr.y));
        ImGui::InvisibleButton("##title",
                               ImVec2(std::max(1.0f, sr.w - btnZone),
                                      std::max(1.0f, titleH)));
        const Bool titleHovered = ImGui::IsItemHovered();

        if(ImGui::IsItemClicked(ImGuiMouseButton_Left))
        {
            raise        = i;
            wsDragPanel  = i;
            wsDragEdge   = 0;
            wsDragOrigin = io.MousePos;
            wsDragRect0  = p.rect;
        }
        if(titleHovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
        {
            p.collapsed      = !p.collapsed;
            panelLayoutDirty = true;
        }

        // Title text and icon, centred on the bar's own centreline - the same
        // rule the status bar follows, and for the same reason.
        {
            const Float32 cy  = sr.y + titleH * 0.5f;
            Float32       tx  = sr.x + 6.0f * uiDpiScale;

            if(ui::iconsReady() && titleH >= ui::iconSize())
            {
                ui::iconAt(pdl, viewIcon(i), ImVec2(tx, cy - ui::iconSize() * 0.5f));
                tx += ui::iconSize() + 4.0f * uiDpiScale;
            }

            const Char*  nm = viewName(i);
            const ImVec2 ts = ImGui::CalcTextSize(nm);
            if(ts.y <= titleH)
            {
                pdl->AddText(ImVec2(tx, cy - ts.y * 0.5f),
                             focused ? IM_COL32_WHITE : IM_COL32(0xC0, 0xC6, 0xCC, 0xFF),
                             nm);
            }

            // Collapse chevron and close, right-aligned on the same centreline.
            if(btn > 6.0f)
            {
                ImGui::SetCursorScreenPos(ImVec2(bx, cy - btn * 0.5f));
                if(ImGui::InvisibleButton("##fold", ImVec2(btn, btn)))
                {
                    p.collapsed      = !p.collapsed;
                    panelLayoutDirty = true;
                }
                const ImU32 fc = ImGui::IsItemHovered() ? IM_COL32_WHITE
                                                        : IM_COL32(0x9A, 0xA2, 0xAA, 0xFF);
                pdl->AddText(ImVec2(bx, cy - ImGui::GetFontSize() * 0.5f), fc,
                             p.collapsed ? "+" : "-");

                ImGui::SetCursorScreenPos(ImVec2(bx + btn + 4.0f * uiDpiScale,
                                                 cy - btn * 0.5f));
                if(ImGui::InvisibleButton("##close", ImVec2(btn, btn)))
                {
                    p.open           = false;
                    panelLayoutDirty = true;
                }
                const ImU32 cc = ImGui::IsItemHovered() ? ui::sem::BAD
                                                        : IM_COL32(0x9A, 0xA2, 0xAA, 0xFF);
                pdl->AddText(ImVec2(bx + btn + 4.0f * uiDpiScale,
                                    cy - ImGui::GetFontSize() * 0.5f), cc, "x");
            }
        }

        // ---- body ----------------------------------------------------------
        if(!p.collapsed && bodyH > 8.0f && sr.w > 8.0f)
        {
            // ---- resize handles, in the panel's BORDER RING -----------------
            //
            // OUTSIDE the body child, and that is the whole point. A child
            // window is a separate ImGui window drawn on top, so anything
            // submitted underneath it is not hoverable - which is exactly why
            // the old bottom-right grip, drawn over the child, never once
            // fired.
            //
            // The ring is a fixed number of SCREEN pixels and is NOT scaled by
            // zoom: a grab target that shrinks with the board becomes unusable
            // at precisely the zoom where you most want to resize something.
            const Float32 edge = 6.0f * uiDpiScale;
            const Float32 innerW = std::max(1.0f, sr.w - edge * 2.0f);
            const Float32 innerH = std::max(1.0f, bodyH - edge);

            struct Handle
            {
                const Char*       id;
                ImVec2            pos;
                ImVec2            size;
                Int32             mask;
                ImGuiMouseCursor  cursor;
            };
            const Handle handles[3] = {
                { "##edgeR", ImVec2(sr.x + sr.w - edge, sr.y + titleH),
                  ImVec2(edge, std::max(1.0f, bodyH - edge)), 1,
                  ImGuiMouseCursor_ResizeEW },
                { "##edgeB", ImVec2(sr.x, sr.y + fullH - edge),
                  ImVec2(std::max(1.0f, sr.w - edge), edge), 2,
                  ImGuiMouseCursor_ResizeNS },
                { "##corner", ImVec2(sr.x + sr.w - edge, sr.y + fullH - edge),
                  ImVec2(edge, edge), 3, ImGuiMouseCursor_ResizeNWSE },
            };

            for(const Handle& hd : handles)
            {
                ImGui::SetCursorScreenPos(hd.pos);
                ImGui::InvisibleButton(hd.id, hd.size);
                if(ImGui::IsItemHovered()
                   || (wsDragPanel == i && wsDragEdge == hd.mask))
                {
                    ImGui::SetMouseCursor(hd.cursor);
                }
                if(ImGui::IsItemClicked(ImGuiMouseButton_Left))
                {
                    raise        = i;
                    wsDragPanel  = i;
                    wsDragEdge   = hd.mask;
                    wsDragOrigin = io.MousePos;
                    wsDragRect0  = p.rect;
                }
            }

            // The corner's three diagonal ticks, so the handle is visible.
            for(Int32 k = 1; k <= 3; ++k)
            {
                const Float32 o = static_cast<Float32>(k) * 3.0f * uiDpiScale;
                pdl->AddLine(ImVec2(sr.x + sr.w - o, sr.y + fullH - 1.0f),
                             ImVec2(sr.x + sr.w - 1.0f, sr.y + fullH - o),
                             IM_COL32(0x70, 0x78, 0x80, 0xFF), 1.0f);
            }

            // ---- the content, OPTICALLY zoomed -----------------------------
            //
            // The panel is already zoom * canvas pixels wide. Drawing the
            // content into it unchanged gives it MORE PIXELS, so it re-lays-out
            // - more columns in the editor, a wider fit on the map. That is
            // reflow, not zoom: zooming in should make the same thing bigger,
            // not show more of it at the same size.
            //
            // So the geometry scale is raised by the same factor for the
            // duration. Everything that derives from dpiScale() - padding,
            // radii, line thicknesses, the editor's cell - grows with it, and
            // fontScale() carries the text along.
            //
            // Saved and restored around the call rather than set globally,
            // because the sidebar and the status bar are outside the canvas and
            // must not move when the board zooms.
            const Float32    dpiWas   = uiDpiScale;
            const ImGuiStyle styleWas = ImGui::GetStyle();

            uiDpiScale = dpiWas * wsCanvas.zoom;
            ui::setDpiScale(uiDpiScale);
            ImGui::GetStyle().ScaleAllSizes(wsCanvas.zoom);

            // Un-pushed text scales too. PushFont with a null face keeps the
            // current one and changes only its size, which is what the dynamic
            // atlas in 1.92 is for.
            ImGui::PushFont(nullptr, ImGui::GetFontSize() * wsCanvas.zoom);

            ImGui::SetCursorScreenPos(ImVec2(sr.x + edge, sr.y + titleH));

            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
            ImGui::BeginChild("##body", ImVec2(innerW, innerH),
                              ImGuiChildFlags_None,
                              ImGuiWindowFlags_NoScrollbar
                              | ImGuiWindowFlags_NoScrollWithMouse);

            drawViewBody(i, innerW, innerH);

            ImGui::EndChild();
            ImGui::PopStyleVar();

            ImGui::PopFont();
            ImGui::GetStyle() = styleWas;
            uiDpiScale        = dpiWas;
            ui::setDpiScale(dpiWas);
        }

        ImGui::EndChild();
        ImGui::PopStyleVar();   // WindowPadding pushed for the panel child
        ImGui::PopID();
    }

    // ---- apply the live drag ----------------------------------------------
    // Applied once, after every panel is submitted, so a drag cannot be
    // interrupted by another panel's widget stealing the mouse mid-frame.
    if(wsDragPanel >= 0)
    {
        if(!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            wsDragPanel = -1;
        }
        else
        {
            ws::Panel& p = wsPanels[wsDragPanel];
            const Float32 z  = (wsCanvas.zoom > 0.0001f) ? wsCanvas.zoom : 0.0001f;
            const Float32 dx = (io.MousePos.x - wsDragOrigin.x) / z;
            const Float32 dy = (io.MousePos.y - wsDragOrigin.y) / z;

            if(wsDragEdge != 0)
            {
                if((wsDragEdge & 1) != 0)
                {
                    p.rect.w = std::max(ws::PANEL_MIN_W, wsDragRect0.w + dx);
                }
                if((wsDragEdge & 2) != 0)
                {
                    p.rect.h = std::max(ws::PANEL_MIN_H, wsDragRect0.h + dy);
                }
            }
            else
            {
                p.rect.x = wsDragRect0.x + dx;
                p.rect.y = wsDragRect0.y + dy;
            }
            panelLayoutDirty = true;
        }
    }

    if(raise >= 0)
    {
        ws::bringToFront(wsPanels, ws::PANEL_COUNT, raise);
        wsFocused        = raise;
        centralView      = raise;
        panelLayoutDirty = true;
    }

    ImGui::EndChild();
    ImGui::PopStyleColor();
}

// The floating layout's own header: back to tabs, the zoom, and which panels
// are on the board.
Void drawFloatingHeader(Float32 mapW)
{
    const ImGuiStyle& sty = ImGui::GetStyle();

    if(ui::iconButton(ui::Icon::ICON_DIM_2D, "Tabs"))
    {
        layoutFloating   = 0;
        panelLayoutDirty = true;
    }
    if(ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Back to one view at a time");
    }

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    // Panel chips: each toggles one panel onto or off the board.
    for(Int32 i = 0; i < ws::PANEL_COUNT; ++i)
    {
        if(i)
        {
            ImGui::SameLine(0.0f, sty.ItemInnerSpacing.x);
        }
        if(ui::segmentedIconButton(viewIcon(i), viewName(i), wsPanels[i].open))
        {
            wsPanels[i].open = !wsPanels[i].open;
            if(wsPanels[i].open)
            {
                ws::bringToFront(wsPanels, ws::PANEL_COUNT, i);
                wsFocused = i;
            }
            panelLayoutDirty = true;
        }
    }

    // Zoom readout and fit, right-aligned.
    Char z[32];
    std::snprintf(z, sizeof(z), "%d%%",
                  static_cast<Int32>(wsCanvas.zoom * 100.0f + 0.5f));

    const Float32 fitW = ImGui::CalcTextSize("Fit").x + sty.FramePadding.x * 2.0f
                       + (ui::iconsReady() ? ui::iconSize() + sty.ItemInnerSpacing.x
                                           : 0.0f);
    const Float32 zW   = ImGui::CalcTextSize("000%").x;
    const Float32 need = fitW + zW + sty.ItemInnerSpacing.x * 2.0f;

    const Float32 x = mapW - need;
    if(x > ImGui::GetCursorPosX())
    {
        ImGui::SameLine(x, 0.0f);
        ImGui::TextDisabled("%s", z);
        ImGui::SameLine(0.0f, sty.ItemInnerSpacing.x);
        if(ui::iconButton(ui::Icon::ICON_RESET_VIEW, "Fit"))
        {
            ws::fitAll(wsPanels, ws::PANEL_COUNT, wsCanvas,
                       mapW, ImGui::GetContentRegionAvail().y);
            panelLayoutDirty = true;
        }
        if(ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Frame every open panel  (wheel zooms, drag the "
                              "background to pan)");
        }
    }
}

// The central region. The map is one view among several rather than the only
// one, but it is still the default and still where the app lands.
Void drawMapRegion(Float32 mapW, Float32 mapH, Float32 ctrlH)
{
    const Float32 headH = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.y;
    const Float32 viewH = mapH - headH;

    if(layoutFloating != 0)
    {
        drawFloatingHeader(mapW);
        drawFloatingWorkspace(mapW, viewH);
    }
    else
    {
        drawTabbedViews(mapW, viewH);
    }

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

Void drawSerialConsole(const ImVec2& size)
{
    if(ui::iconButton(ui::Icon::ICON_CLEAR, "Clear")) picoLog.clear();
    ImGui::SameLine();
    ui::checkbox("Auto-scroll", &logAutoscroll);

    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputTextWithHint("##logfilter", "filter lines", filterBuf, sizeof(filterBuf));

    logShown.clear();
    for(Int32 i = 0; i < static_cast<Int32>(picoLog.size()); ++i)
        if(logMatches(picoLog[i])) logShown.push_back(i);

    {
        ScopedFont sf(ui::fonts.small);
        if(filterBuf[0])
            ImGui::TextDisabled("%d of %d lines   -   %llu sent / %llu received",
                                static_cast<Int32>(logShown.size()), static_cast<Int32>(picoLog.size()),
                                picoLink.txLines(), picoLink.rxLines());
        else
            ImGui::TextDisabled("%d lines   -   %llu sent / %llu received",
                                static_cast<Int32>(picoLog.size()), picoLink.txLines(), picoLink.rxLines());
    }

    ImGui::BeginChild("##console", size, ImGuiChildFlags_None,
                      ImGuiWindowFlags_HorizontalScrollbar);

    if(!logShown.empty())
    {
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<Int32>(logShown.size()));
        while(clipper.Step())
        {
            for(Int32 r = clipper.DisplayStart; r < clipper.DisplayEnd; ++r)
            {
                const PicoLine& ln = picoLog[logShown[r]];

                Char buf[512];
                std::snprintf(buf, sizeof(buf), "%8.2f  %c  %s",
                              ln.tS, ln.outgoing ? '>' : '<', ln.text.c_str());

                ImGui::PushStyleColor(ImGuiCol_Text,
                                      ln.outgoing ? ui::plot::RAMP_NEAR : ui::plot::RAMP_FAR);
                ImGui::TextUnformatted(buf);
                ImGui::PopStyleColor();
            }
        }
    }

    // Sticks to the bottom only while the view already is at the bottom, so
    // scrolling up to read something does not yank you back.
    if(logAutoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);

    ImGui::EndChild();
}

// Two logs, one section. Side by side they would each get 190 px of a 400 px
// column; stacked they would cost 600 px of scrolling to reach the second. Tabs
// give whichever one you are watching the full width and a usable height.
Void sectionConsole()
{
    const ImVec2 pane(0.0f, LOG_PANE_H * uiDpiScale);

    if(ImGui::BeginTabBar("##consoletabs"))
    {
        Char lb_ICON_CONSOLE[40];
        const Bool t_ICON_CONSOLE = ImGui::BeginTabItem(
            iconTabLabel(lb_ICON_CONSOLE, sizeof(lb_ICON_CONSOLE), "Pico serial"));
        tabIcon(ui::Icon::ICON_CONSOLE);
        if(t_ICON_CONSOLE)
        {
            drawSerialConsole(pane);
            ImGui::EndTabItem();
        }
        Char lb_ICON_BUILD[40];
        const Bool t_ICON_BUILD = ImGui::BeginTabItem(
            iconTabLabel(lb_ICON_BUILD, sizeof(lb_ICON_BUILD), "Build / flash"));
        tabIcon(ui::Icon::ICON_BUILD);
        if(t_ICON_BUILD)
        {
            drawFlashOutput("##flashout_con", pane);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
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
    { "    Console",  "Console",  SECTION_CONSOLE,  false, ui::Icon::ICON_CONSOLE,  &sectionConsole  },
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
            else if(_stricmp(v, "pico") == 0 ||
                     _stricmp(v, "pico2w") == 0 ||
                     _stricmp(v, "board") == 0)
                     {
                         forceView = BOARD_VIEW_0;
                         forceViewFrames = 4;
                     }
            else if(_stricmp(v, "range") == 0 ||
                     _stricmp(v, "tof") == 0)
                     {
                         forceView = RANGE_VIEW;
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

        if(std::strcmp(__argv[i], "--layout") == 0 && i + 1 < __argc)
        {
            const Char* v = __argv[i + 1];
            if(_stricmp(v, "floating") == 0 || _stricmp(v, "float") == 0)
                layoutFloating = 1;
            else if(_stricmp(v, "tabbed") == 0 || _stricmp(v, "tabs") == 0)
                layoutFloating = 0;
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
                { "console",  SECTION_CONSOLE,  -1 }, { "debug",    SECTION_CONSOLE,  -1 },
            };
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
            for(Int32 k = 0; k < 3; ++k)
                if(BAUDS[k] == b) baudIndex = k;
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

Void app::frame()
{
    pumpData();

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

    // Sized for the view that is on screen, which is why centralView is kept
    // across frames. A view with no controls costs no height at all - the
    // spacing goes too, or a board tab would sit above a blank strip.
    const Float32 mapW  = avail.x - sideW - gap;
    const Float32 ctrlH = centralControlHeight(centralView, mapW);
    const Float32 mapH  = avail.y - ctrlH - (ctrlH > 0.0f ? sty.ItemSpacing.y : 0.0f);

    const ImVec2 p0 = ImGui::GetCursorScreenPos();

    if(mapW > 80.0f * uiDpiScale && mapH > 80.0f * uiDpiScale)
    {
        drawMapRegion(mapW, mapH, ctrlH);
        sidebarSplitter(ImVec2(p0.x + mapW, p0.y), avail.y, gap);
        ImGui::SetCursorScreenPos(ImVec2(p0.x + mapW + gap, p0.y));
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
