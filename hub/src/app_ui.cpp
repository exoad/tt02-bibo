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
#include <string>
#include <vector>

#include "imgui.h"
#include "lidar_source.hpp"
#include "pico_flash.hpp"
#include "pico_link.hpp"
#include "board_view.hpp"
#include "radar.hpp"
#include "icons.hpp"
#include "lights.hpp"
#include "code_view.hpp"
#include "editor.hpp"
#include "recording.hpp"
#include "sketch.hpp"
#include "settings.hpp"
#include "theme.hpp"

namespace {

LidarSource lidarSource;
RadarView   radarView;
PicoLink    picoLink;
PicoFlash   picoFlash;

std::vector<Str> lidarPorts;
std::vector<const Char*> portItems;
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

std::vector<Str> picoPorts;
std::vector<const Char*> picoItems;
Int32  picoIndex = -1;

// The console is a debug aid, not a record: this app runs for hours, so the log
// is bounded and the oldest lines fall off the front.
constexpr Size LOG_MAX = 4000;
std::vector<PicoLine> picoLog;
std::vector<Int32>      logShown;    // indices passing the filter, rebuilt per frame

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
Bool   dbgUnsupported = false;
Bool   dbgAwait       = false;
Float64 dbgLastPoll   = 0.0;

Void resetBoardStatus()
{
    debugStatus             = DebugStatus();
    dbgUnsupported = false;
    dbgAwait       = false;
    dbgLastPoll   = 0.0;
}

// ----------------------------------------------------------------- flash ---
// The firmware suite: catalog, board state, and the output of whichever script
// is running. PicoFlash does the work on a worker thread; everything here is
// display plus the one confirmation that stands between a click and an
// irreversible overwrite.

constexpr Size FLASH_LOG_MAX = 3000;
std::vector<Str> flashLog;

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

enum Section
{
    SEC_System = 0,
    SEC_Sensors,
    SEC_Vehicle,
    SEC_Firmware,
    SEC_Console,
    SEC_Count,
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
Int32 sectionOrder[SEC_Count] = { SEC_System, SEC_Sensors, SEC_Vehicle,
                                  SEC_Firmware, SEC_Console };
Bool  sectionFloating[SEC_Count] = {};

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
    Int32 order[SEC_Count] = {};

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
        else if(line.size() > 2 && line[0] == 's' && seen < SEC_Count)
        {
            Int32 id = -1, fl = 0;
            if(std::sscanf(line.c_str() + 1, "%d %d", &id, &fl) == 2
               && id >= 0 && id < SEC_Count)
            {
                order[seen++] = id;
                sectionFloating[id] = (fl != 0);
            }
        }
    }

    // Only accept an order that is a genuine permutation. A partial or repeated
    // one would silently drop a section off the screen with no way back.
    if(seen == SEC_Count)
    {
        Bool used[SEC_Count] = {};
        Bool ok = true;
        for(Int32 k = 0; k < SEC_Count; ++k)
        {
            if(used[order[k]]) { ok = false; break; }
            used[order[k]] = true;
        }
        if(ok)
            for(Int32 k = 0; k < SEC_Count; ++k)
                sectionOrder[k] = order[k];
    }
}

Void savePanelLayout()
{
    Char buf[64];
    Str out;
    std::snprintf(buf, sizeof(buf), "w %.0f\n", static_cast<Float64>(sidebarLogicalW));
    out += buf;
    std::snprintf(buf, sizeof(buf), "t %.0f %d\n",
                  static_cast<Float64>(codeTreeLogicalW),
                  codeTreeCollapsed ? 1 : 0);
    out += buf;
    for(Int32 k = 0; k < SEC_Count; ++k)
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

// Set while a Build & Flash is mid-flight so the second half (the flash) fires
// when the build finishes rather than racing it.
Bool codeFlashPending = false;
Str  codeFlashTarget  = "sketch";

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
std::vector<Str> recFiles;
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
    explicit ScopedFont(ImFont* f) { ImGui::PushFont(f, f ? f->LegacySize : 0.0f); }
    ~ScopedFont() { ImGui::PopFont(); }
};

Void refreshPorts()
{
    lidarPorts = LidarSource::listPorts();

    portItems.clear();
    for(const auto& s : lidarPorts) portItems.push_back(s.c_str());

    if(lidarPorts.empty()) { portIndex = -1; return; }

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

    // Fallback: a USB bridge normally enumerates above the built-in ports.
    if(portIndex < 0 || portIndex >= static_cast<Int32>(lidarPorts.size()))
        portIndex = static_cast<Int32>(lidarPorts.size()) - 1;
}

Bool isBusy()
{
    const LidarState s = lidarSource.state();
    return s == LidarState::LIDAR_STATE_SCANNING || s == LidarState::LIDAR_STATE_CONNECTING;
}

// ---------------------------------------------------------------- pico ----

Void refreshPicoPorts()
{
    picoPorts = PicoLink::listPicoPorts();

    picoItems.clear();
    for(const auto& s : picoPorts) picoItems.push_back(s.c_str());

    if(picoPorts.empty()) { picoIndex = -1; return; }
    if(picoIndex < 0 || picoIndex >= static_cast<Int32>(picoPorts.size())) picoIndex = 0;
}

Void connectPico()
{
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

const Char* picoStateText(PicoState s)
{
    switch(s)
    {
    case PicoState::PICO_STATE_CONNECTING: return "Connecting";
    case PicoState::PICO_STATE_CONNECTED:  return "Connected";
    case PicoState::PICO_STATE_ERROR:      return "Error";
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
               std::tolower(static_cast<UInt8>(*h)) == std::tolower(static_cast<UInt8>(*n))) { ++h; ++n; }
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
        if(p.distMm <= 0.0f)          { ++nNoreturn; continue; }
        if(p.distMm < MIN_VALID_MM)    { ++nToonear;  continue; }
        if(p.distMm > MAX_VALID_MM)    { ++nToofar;   continue; }

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

Void pumpFlash()
{
    picoFlash.drainLog(flashLog);
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

            // The second half of the Code view's Build & Flash. Chained on the
            // transition rather than started alongside the build, because the
            // two cannot overlap - PicoFlash runs one operation at a time and
            // would simply reject the flash.
            if(codeFlashPending)
            {
                codeFlashPending = false;
                if(s == FlashState::FLASH_STATE_SUCCESS)
                {
                    codeMessage = "built; flashing " + codeFlashTarget;
                    picoFlash.flash(codeFlashTarget);
                }
                else
                {
                    codeMessage = "build failed - see the Firmware panel";
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
}

Void applyRange()
{
    const Float32 mm = RANGES[rangeIndex].mm;
    if(mm <= 0.0f) radarView.fit();
    else            radarView.setRangeMm(mm);
}

Void connect()
{
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
Void stripField(ui::Icon ic, const Char* label, ImU32 col, const Char* value,
                const Char* extra)
{
    ui::iconLabel(ic);

    ImGui::TextDisabled("%s", label);
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);

    ImGui::PushStyleColor(ImGuiCol_Text, col);
    ImGui::TextUnformatted(value);
    ImGui::PopStyleColor();

    if(extra != nullptr && extra[0] != 0)
    {
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::TextDisabled("%s", extra);
    }
}

Void stripSep()
{
    const Float32 g = ImGui::GetFontSize() * 0.75f;
    ImGui::SameLine(0.0f, g);
    ImGui::TextDisabled("|");
    ImGui::SameLine(0.0f, g);
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
Void drawZoomControl()
{
    const ImGuiStyle& sty = ImGui::GetStyle();

    Char pct[16];
    std::snprintf(pct, sizeof(pct), "%d%%",
                  static_cast<Int32>(ui::userScale() * 100.0f + 0.5f));

    const Float32 btnW = ImGui::CalcTextSize("A+").x + sty.FramePadding.x * 2.0f;
    const Float32 pctW = ImGui::CalcTextSize("000%").x;
    const Float32 gap  = sty.ItemInnerSpacing.x;
    const Float32 need = btnW * 2.0f + pctW + gap * 2.0f;

    // Right-aligned, and silently dropped if the strip is too narrow to hold it
    // without colliding with the status fields.
    const Float32 x = ImGui::GetWindowContentRegionMax().x - need;
    if(x <= ImGui::GetCursorPosX() + gap)
        return;

    ImGui::SameLine(x, 0.0f);

    const Bool atMin = ui::userScale() <= ui::USER_SCALE_MIN + 0.001f;
    const Bool atMax = ui::userScale() >= ui::USER_SCALE_MAX - 0.001f;

    ImGui::BeginDisabled(atMin);
    if(ImGui::SmallButton("A-"))
        ui::setUserScale(ui::userScale() - ui::USER_SCALE_STEP);
    ImGui::EndDisabled();
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("Smaller  (Ctrl -)");

    ImGui::SameLine(0.0f, gap);
    ImGui::TextDisabled("%s", pct);
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("UI scale. Ctrl 0 resets it to 100%%.");
    if(ImGui::IsItemClicked())
        ui::setUserScale(1.0f);

    ImGui::SameLine(0.0f, gap);
    ImGui::BeginDisabled(atMax);
    if(ImGui::SmallButton("A+"))
        ui::setUserScale(ui::userScale() + ui::USER_SCALE_STEP);
    ImGui::EndDisabled();
    if(ImGui::IsItemHovered())
        ImGui::SetTooltip("Bigger  (Ctrl +)");
}

Void drawStatusStrip()
{
    // ---- lidar ----------------------------------------------------------
    Char lidarExtra[64] = {};
    if(lidarSource.state() == LidarState::LIDAR_STATE_SCANNING)
        std::snprintf(lidarExtra, sizeof(lidarExtra), "%s  %.1f Hz",
                      (portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
                          ? lidarPorts[portIndex].c_str() : "",
                      haveFrame ? latestFrame.hz : 0.0f);
    else if(portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
        std::snprintf(lidarExtra, sizeof(lidarExtra), "%s", lidarPorts[portIndex].c_str());

    stripField(ui::Icon::ICON_RADAR, "LIDAR", lidarStateColor(), lidarStateText(),
               lidarExtra);

    // ---- pico link -------------------------------------------------------
    stripSep();
    const PicoState ps = picoLink.state();
    const Str pport = picoLink.port().empty()
        ? (picoIndex >= 0 && picoIndex < static_cast<Int32>(picoPorts.size())
               ? picoPorts[picoIndex] : Str())
        : picoLink.port();
    stripField(ui::Icon::ICON_PROCESSOR, "PICO", picoStateColor(ps), picoStateText(ps),
               pport.c_str());

    // ---- board -----------------------------------------------------------
    stripSep();
    const BoardStatus brd = picoFlash.board();
    if(brd.bootsel)
        stripField(ui::Icon::ICON_FIRMWARE, "BOARD", ui::sem::WARN, "BOOTSEL",
                   brd.drive.c_str());
    else if(brd.present)
        stripField(ui::Icon::ICON_FIRMWARE, "BOARD", ui::sem::GOOD, "Running",
                   brd.program.c_str());
    else
        stripField(ui::Icon::ICON_FIRMWARE, "BOARD", ui::sem::MUTED, "absent", "");

    // ---- long-running operation ------------------------------------------
    stripSep();
    const FlashState fs = picoFlash.state();
    if(fs == FlashState::FLASH_STATE_WORKING)
        stripField(ui::Icon::ICON_BUILD, "OP", ui::sem::WARN, "running",
                   picoFlash.currentOp().c_str());
    else if(fs == FlashState::FLASH_STATE_SUCCESS)
        stripField(ui::Icon::ICON_BUILD, "OP", ui::sem::GOOD, "done",
                   picoFlash.currentOp().c_str());
    else if(fs == FlashState::FLASH_STATE_FAILED)
        stripField(ui::Icon::ICON_BUILD, "OP", ui::sem::BAD, "FAILED",
                   picoFlash.currentOp().c_str());
    else
        stripField(ui::Icon::ICON_BUILD, "OP", ui::sem::MUTED, "idle", "");

    // ---- UI zoom, right-aligned ------------------------------------------
    //
    // Visible rather than shortcut-only. "The UI is too small" is a complaint
    // about the app, and an app whose answer to it is a key combination nobody
    // is told about has not answered it. The keys work too - Ctrl +/-/0.
    drawZoomControl();
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
                picoLink.disconnect();
        }
        else
        {
            ImGui::BeginDisabled(picoIndex < 0);
            if(ui::iconButton(ui::Icon::ICON_PLUG_CONNECT, "Connect Pico",
                              ImVec2(-FLT_MIN, bh), ui::Tint::TINT_GOOD))
                connectPico();
            ImGui::EndDisabled();
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
            lidarSource.stop();
    }
    else
    {
        ImGui::BeginDisabled(lidarPorts.empty());
        if(ui::iconButton(ui::Icon::ICON_PLUG_CONNECT, "Connect",
                          ImVec2(-FLT_MIN, bh), ui::Tint::TINT_GOOD))
            connect();
        ImGui::EndDisabled();
    }

    const Str err = lidarSource.error();
    if(!err.empty() && lidarSource.state() == LidarState::LIDAR_STATE_ERROR)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ui::sem::BAD);
        ImGui::TextWrapped("%s", err.c_str());
        ImGui::PopStyleColor();
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
            if(t) { tabLive();   ImGui::EndTabItem(); }
        }
        {
            Char lb[40];
            const Bool t = ImGui::BeginTabItem(iconTabLabel(lb, sizeof(lb), "Signal"),
                                               nullptr, sub(1));
            tabIcon(ui::Icon::ICON_SIGNAL);
            if(t) { tabSignal(); ImGui::EndTabItem(); }
        }
        {
            Char lb[40];
            const Bool t = ImGui::BeginTabItem(iconTabLabel(lb, sizeof(lb), "Scan"),
                                               nullptr, sub(2));
            tabIcon(ui::Icon::ICON_SCAN);
            if(t) { tabScan();   ImGui::EndTabItem(); }
        }
        {
            Char lb[40];
            const Bool t = ImGui::BeginTabItem(iconTabLabel(lb, sizeof(lb), "Device"),
                                               nullptr, sub(3));
            tabIcon(ui::Icon::ICON_DEVICE);
            if(t) { tabDevice(); ImGui::EndTabItem(); }
        }
        ImGui::EndTabBar();
    }
}

// The permanent left region: the map, with the control bar under it. Both are
// sized by the caller, which owns the split between map and sidebar.
// --view <map|pico> preselects a central tab at startup. Held for a few frames
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
        static std::vector<const Char*> items;
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
    codeMessage = "opened " + name;
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
    static std::vector<Str> libFiles;
    static std::vector<Str> fwFiles;
    static Int32            rescanIn = 0;
    if(rescanIn <= 0)
    {
        libFiles = sketch::list();
        fwFiles  = sketch::listFirmware();
        rescanIn = 120;          // ~2 s at 60 fps
    }
    --rescanIn;

    const auto row = [](const Str& name, const Str& path, Bool sel, ui::Icon ic)
    {
        ImGui::PushID(path.c_str());
        ui::icon(ic);
        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        const Bool hit = ImGui::Selectable(name.c_str(), sel);
        ImGui::PopID();
        return hit;
    };

    if(ImGui::TreeNodeEx("Sketches", ImGuiTreeNodeFlags_DefaultOpen))
    {
        for(const Str& n : libFiles)
        {
            const Str p = sketch::pathOf(n);
            if(row(n, p, _stricmp(p.c_str(), codePath.c_str()) == 0,
                   ui::Icon::ICON_CODE))
                openCodeFile(p, n);
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
                   hdr ? ui::Icon::ICON_FIRMWARE : ui::Icon::ICON_CODE))
                openCodeFile(p, n);
        }
        if(fwFiles.empty())
            ImGui::TextDisabled("  repo not found");
        ImGui::TreePop();
    }

    ImGui::EndChild();
    ImGui::PopStyleColor();
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
            codeFlashTarget  = sketch::targetFor(codePath);
            codeFlashPending = true;
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

    ImGui::BeginDisabled(busy);

    // Amber rather than green: this writes to the board. It is the same claim
    // the Firmware panel's Flash button makes, and it has to be the same colour
    // or the colour stops meaning anything.
    const Str target = sketch::targetFor(codePath);

    Char flashLabel[64];
    std::snprintf(flashLabel, sizeof(flashLabel), "Build & Flash %s", target.c_str());

    if(ui::iconButton(ui::Icon::ICON_FLASH, flashLabel,
                      ImVec2(280.0f * uiDpiScale, bh), ui::Tint::TINT_WARN))
    {
        if(saveSketch())
        {
            codeFlashTarget  = target;
            codeFlashPending = true;
            picoFlash.build(target);
        }
    }

    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_BUILD, "Build only",
                      ImVec2(150.0f * uiDpiScale, bh)))
    {
        if(saveSketch())
        {
            codeFlashPending = false;
            picoFlash.build(target);
        }
    }
    ImGui::EndDisabled();

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
                      || codeMessage.find("not a command") != Str::npos;
        ImGui::TextColored(
            ImGui::ColorConvertU32ToFloat4(bad ? ui::sem::BAD : ui::sem::GOOD),
            "%s", codeMessage.c_str());
    }
    else
    {
        ImGui::TextDisabled("i insert - esc normal - :w save - hjkl move");
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
Void drawMapRegion(Float32 mapW, Float32 mapH, Float32 ctrlH)
{
    const Float32 tabH = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.y;
    const Float32 viewH = mapH - tabH;

    if(ImGui::BeginTabBar("##central", ImGuiTabBarFlags_None))
    {
        const auto viewSel = [](Int32 which) {
            return (forceView == which && forceViewFrames > 0)
                 ? ImGuiTabItemFlags_SetSelected : 0;
        };
        if(forceViewFrames > 0) --forceViewFrames;

        // Two map tabs. Same viewer, same revolution, different dimension - and
        // the tab bar is where this app already says "a different way of looking
        // at the same machine".
        for(Int32 d = 0; d < 2; ++d)
        {
        Char mapLb[32];
        const Bool mapTab = ImGui::BeginTabItem(
            iconTabLabel(mapLb, sizeof(mapLb), d == 0 ? "2D" : "3D"),
            nullptr, viewSel(d));
        tabIcon(d == 0 ? ui::Icon::ICON_DIM_2D : ui::Icon::ICON_DIM_3D);
        if(mapTab)
        {
            centralView    = d;
            radarView.is3D = (d == 1);
            const ImVec2 p0 = ImGui::GetCursorScreenPos();

            // Explicitly black, matching every other surface. Kept as an
            // explicit push rather than inherited, because the map is the one
            // panel whose background must never pick up a tint or an alpha - it
            // is the surface the point cloud is read against.
            ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0x0E, 0x0F, 0x12, 0xFF));
            ImGui::PushID(d);
            ImGui::BeginChild("##map", ImVec2(mapW, viewH), ImGuiChildFlags_None,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            radarView.draw(ImGui::GetContentRegionAvail());
            drawMapHud(p0, ImVec2(mapW, viewH));
            // The display is set INTO the console, so it carries a bezel rather
            // than an outline. Drawn last so it sits over the point cloud.
            // No bezel on the map. The board view still gets one, because
            // that panel IS a depiction of a physical thing; this one is a
            // terminal, and a terminal's edge is a line.
            ImGui::GetWindowDrawList()->AddRect(
                p0, ImVec2(p0.x + mapW, p0.y + viewH),
                IM_COL32(0x3A, 0x3A, 0x3A, 0xFF), 0.0f, 0, 1.0f);
            ImGui::EndChild();
            ImGui::PopID();
            ImGui::PopStyleColor();

            ImGui::EndTabItem();
        }
        }

        // ---- the recorder ------------------------------------------------
        {
        Char recLb[32];
        const Bool recTab = ImGui::BeginTabItem(
            iconTabLabel(recLb, sizeof(recLb), "Record"), nullptr, viewSel(2));
        tabIcon(ui::Icon::ICON_RECORD);
        if(recTab)
        {
            centralView = 2;
            const ImVec2 rp0 = ImGui::GetCursorScreenPos();

            ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0x0E, 0x0F, 0x12, 0xFF));
            ImGui::PushID("recorder");
            ImGui::BeginChild("##recmap", ImVec2(mapW, viewH), ImGuiChildFlags_None,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            recView.draw(ImGui::GetContentRegionAvail());
            drawRecorderHud(rp0, ImVec2(mapW, viewH));
            ImGui::GetWindowDrawList()->AddRect(
                rp0, ImVec2(rp0.x + mapW, rp0.y + viewH),
                IM_COL32(0x3A, 0x3A, 0x3A, 0xFF), 0.0f, 0, 1.0f);
            ImGui::EndChild();
            ImGui::PopID();
            ImGui::PopStyleColor();

            ImGui::EndTabItem();
        }
        }

        // ---- the code editor ---------------------------------------------
        {
        Char codeLb[32];
        const Bool codeTab = ImGui::BeginTabItem(
            iconTabLabel(codeLb, sizeof(codeLb), "Code"), nullptr, viewSel(3));
        tabIcon(ui::Icon::ICON_CODE);
        if(codeTab)
        {
            centralView = 3;

            // Loaded on first sight rather than at startup: reading the sketch
            // library costs a directory scan, and most sessions never open this
            // tab at all.
            if(!codeLoaded)
            {
                codeLoaded = true;
                const std::vector<Str> have = sketch::list();
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

            // Tree | splitter | editor. The tree's width is the user's, clamped
            // so a drag can neither squeeze the editor away nor strand the tree
            // at a width with no grabbable edge.
            const ImVec2  cp0    = ImGui::GetCursorScreenPos();
            const Float32 splitW = std::max(ImGui::GetStyle().ItemSpacing.x,
                                            8.0f * uiDpiScale);

            const Float32 treeW = codeTreeCollapsed
                ? ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.x * 2.0f
                : std::max(CODE_TREE_MIN_W * uiDpiScale,
                           std::min(codeTreeLogicalW * uiDpiScale,
                                    std::min(CODE_TREE_MAX_W * uiDpiScale,
                                             mapW * 0.6f)));

            drawCodeTree(treeW, viewH);
            ImGui::SameLine(0.0f, 0.0f);

            if(codeTreeCollapsed)
            {
                // No handle when there is nothing to resize, but the same gap so
                // the editor does not shift by eight pixels on collapse.
                ImGui::SetCursorScreenPos(ImVec2(cp0.x + treeW, cp0.y));
                ImGui::Dummy(ImVec2(splitW, viewH));
            }
            else
            {
                codeTreeSplitter(ImVec2(cp0.x + treeW, cp0.y), viewH, splitW);
            }
            ImGui::SameLine(0.0f, 0.0f);

            ui::drawCode(codeView, codeEditor,
                         ImVec2(std::max(120.0f, mapW - treeW - splitW), viewH),
                         ImGui::GetTime());
            handleCodeCommand();

            ImGui::EndTabItem();
        }
        }

        for(Int32 b = 0; b < static_cast<Int32>(board::Which::WHICH_COUNT); ++b)
        {
            const board::Which which = static_cast<board::Which>(b);
            Char tabLabel[48];
            iconTabLabel(tabLabel, sizeof(tabLabel), board::name(which));
            const Bool boardTab = ImGui::BeginTabItem(tabLabel, nullptr, viewSel(b + 4));
            tabIcon(ui::Icon::ICON_PROCESSOR);
            if(boardTab)
            {
                centralView = b + 4;
                ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0x0E, 0x0F, 0x12, 0xFF));
                ImGui::BeginChild("##board", ImVec2(mapW, viewH), ImGuiChildFlags_None,
                                  ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                // Polled here, inside the tab body, so it only runs while this
                // view is the one on screen.
                pollBoardStatus();
                const ImVec2 bp0 = ImGui::GetCursorScreenPos();
                board::draw(which, ImGui::GetContentRegionAvail(), boardLive());
                ui::screenInset(bp0, ImVec2(bp0.x + mapW, bp0.y + viewH));
                ImGui::EndChild();
                ImGui::PopStyleColor();

                ImGui::EndTabItem();
            }
        }

        ImGui::EndTabBar();
    }

    // The bottom bar belongs to the VIEW above it, not to the central region.
    // Points/Rays/Density and Grid/Trail/Labels configure the map and nothing
    // else, so on a board tab they are not merely disabled - they are absent,
    // and the board gets the height back.
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
    if(!err.empty() && st == PicoState::PICO_STATE_ERROR)
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

    const std::vector<FirmwareEntry>& cat = picoFlash.catalog();

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
        picoFlash.rebootBootsel();
    }
    ImGui::SameLine();
    if(ui::iconButton(ui::Icon::ICON_REBOOT, "Normally", ImVec2(half, bh)))
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

const SectionEntry SECTIONS[SEC_Count] = {
    { "    System",   "System",   SEC_System,   true,  ui::Icon::ICON_SYSTEM,   &sectionSystem   },
    { "    Sensors",  "Sensors",  SEC_Sensors,  true,  ui::Icon::ICON_SENSORS,  &sectionSensors  },
    { "    Vehicle",  "Vehicle",  SEC_Vehicle,  false, ui::Icon::ICON_VEHICLE,  &sectionVehicle  },
    { "    Firmware", "Firmware", SEC_Firmware, false, ui::Icon::ICON_FIRMWARE, &sectionFirmware },
    { "    Console",  "Console",  SEC_Console,  false, ui::Icon::ICON_CONSOLE,  &sectionConsole  },
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
    if(from == to || from < 0 || to < 0 || from >= SEC_Count || to >= SEC_Count)
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

    for(Int32 slot = 0; slot < SEC_Count; ++slot)
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
    for(Int32 slot = 0; slot < SEC_Count; ++slot)
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
                     _stricmp(v, "board") == 0) { forceView = 4; forceViewFrames = 4; }

            // Seed the live selection too, so the first frame reserves the
            // right bottom-bar height instead of the map's.
            if(forceView >= 0) centralView = forceView;
            continue;
        }

        if(std::strcmp(__argv[i], "--tab") == 0 && i + 1 < __argc)
        {
            struct TabName { const Char* name; Int32 sec; Int32 sub; };
            static const TabName TAB_NAMES[] = {
                { "system",   SEC_System,   -1 }, { "overview", SEC_System,   -1 },
                { "sensors",  SEC_Sensors,  -1 }, { "world",    SEC_Sensors,  -1 },
                { "lidar",    SEC_Sensors,  -1 },
                { "live",     SEC_Sensors,   0 }, { "signal",   SEC_Sensors,   1 },
                { "scan",     SEC_Sensors,   2 }, { "device",   SEC_Sensors,   3 },
                // "map" and "pico" are deliberately absent: they now name central
                // views (--view), and one word must not select two different things.
                { "vehicle",  SEC_Vehicle,  -1 },
                { "firmware", SEC_Firmware, -1 }, { "flash",    SEC_Firmware, -1 },
                { "console",  SEC_Console,  -1 }, { "debug",    SEC_Console,  -1 },
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
            for(Int32 p = 0; p < static_cast<Int32>(lidarPorts.size()); ++p)
                if(_stricmp(lidarPorts[p].c_str(), __argv[i + 1]) == 0) portIndex = p;
        }
        if(i + 2 < __argc && __argv[i + 2][0] != '-')
        {
            const Int32 b = std::atoi(__argv[i + 2]);
            for(Int32 k = 0; k < 3; ++k)
                if(BAUDS[k] == b) baudIndex = k;
        }
        break;
    }

    if(!suppress) connect();

    // The Pico is the other half of "launched with no arguments, both devices
    // connected", which is what this app is documented to do - but only the
    // lidar was ever wired up here. The board view made the omission obvious:
    // it has nothing live to show until the link is open, and every launch was
    // opening it closed. --no-connect suppresses both, as it always has.
    if(!suppress) connectPico();
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

    // ---- status strip, always -------------------------------------------
    // Sized to the text it holds, not to a frame. It was
    // GetFrameHeight() + WindowPadding*2, which reserves room for a widget's
    // padding plus the window's own - about 55px of bar around 22px of text,
    // with AlignTextToFramePadding pushing that text off centre inside it.
    // Sized to whichever is taller, the type or the icons. Sizing to the text
    // alone is what cropped the old lamps.
    const Float32 stripPad = 6.0f * uiDpiScale;
    const Float32 stripH   = std::max(ImGui::GetTextLineHeight(), ui::iconSize())
                           + stripPad * 2.0f;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f * uiDpiScale, stripPad));
    ImGui::BeginChild("##strip", ImVec2(0.0f, stripH), ImGuiChildFlags_None,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    drawStatusStrip();
    ImGui::EndChild();
    ImGui::PopStyleVar();   // WindowPadding pushed for the strip

    // ---- map + sidebar ---------------------------------------------------
    const ImVec2 avail = ImGui::GetContentRegionAvail();

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
    lidarSource.stop();
    picoLink.disconnect();
}
