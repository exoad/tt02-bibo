// Application layout: a dashboard, not a set of pages.
//
//   status strip   full width, one line, always visible
//   the map        the whole left region, permanently
//   control bar    under the map: range, overlays, reset view
//   right sidebar  one scrollable column of collapsing sections
//
// The two logs need their own fixed-height inner scroll regions: a scrolling log
// inside a scrolling column cannot be used. Project background lives in
// docs/conventions.md and docs/log.md.
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

// For SetKeyOwner only. Space has to be taken AWAY from ImGui's own keyboard
// navigation, which activates whatever the nav cursor is on - updateEmergencyKey().
#include "imgui_internal.h"

#include "lidar_source.hxx"
#include "pico_flash.hxx"
#include "pico_link.hxx"
#include "radar.hxx"
#include "icons.hxx"
#include "lights.hxx"
#include "applog.hxx"
#include "diagnostics.hxx"
#include "code_view.hxx"
#include "editor.hxx"
#include "recording.hxx"
#include "sketch.hxx"
#include "refdoc.hxx"
#include "devlink.hxx"
#include "lint.hxx"
#include "settings.hxx"
#include "theme.hxx"

namespace
{

  LidarSource lidarSource;
  RadarView   radarView;
  PicoLink    picoLink;
  PicoFlash   picoFlash;

  Vec<Str> lidarPorts;
  Vec<const Char*> portItems;
  Int32   portIndex = -1;
  Int32   baudIndex = 0;     // 460800, the only rate a C1 has
  Int32   rangeIndex = 0;     // Fit
  Float32 uiDpiScale = 1.0f;

  LidarFrame latestFrame;
  Bool       haveFrame = false;

  // The C1 is specified over 0.05 - 12 m: below is the housing, above is noise.
  // The same window governs what the map draws, so nothing is shown uncounted.
  constexpr Float32 MIN_VALID_MM = 50.0f;
  constexpr Float32 MAX_VALID_MM = 12000.0f;

  // ---------------------------------------------------------------- derived ---
  // Recomputed once per revolution, not per UI frame. LidarFrame's own counts are
  // raw, so everything spec-bounded is derived here.

  Float32 meanMm = 0.0f;
  Float32 maxRangeMm = 0.0f;
  Float32 pointsPs = 0.0f;

  // Return classification. These four sum to the revolution's sample count.
  Int32 nInspec = 0;
  Int32 nNoreturn = 0;   // dist == 0, the device saw nothing that way
  Int32 nToonear = 0;   // 0 < dist < 50 mm, housing reflection
  Int32 nToofar = 0;   // dist > 12 m, beyond the rated range

  // Signal quality, over in-spec returns only.
  Float32 qMean = 0.0f;
  Int32   qMin = 0;
  Int32   qMax = 0;

  constexpr Int32 QUALITY_BUCKETS = 16;         // 0..63 folded into 16 bins
  Array<Float32, QUALITY_BUCKETS> qHist= {};
  Float32 qHistMax = 1.0f;

  constexpr Int32 DIST_BUCKETS = 24;            // 0..12 m in 0.5 m bins
  Array<Float32, DIST_BUCKETS> distHist= {};
  Float32 distHistMax = 1.0f;

  // Angular coverage: fraction of 1-degree bins with at least one in-spec return.
  Float32 coverageDeg = 0.0f;

  // Clearance: distance to the nearest return in each 30 degree sector, in meters.
  constexpr Int32 SECTORS = 12;
  Array<Float32, SECTORS> sectorM= {};
  constexpr Float32 CLEARANCE_CAP_M = 2.5f;   // beyond this a direction is just "clear"

  // ------------------------------------------------------------- pico link ---
  // The debug/bring-up channel to the Pico 2 W over USB CDC.
  // TRAP: TinyUSB CDC refuses OUT data until the host asserts DTR, and a write
  // without it blocks until "the semaphore timeout period has expired" - which
  // reads exactly like dead hardware. pico_link.cxx asserts DTR; do not remove.

  Vec<Str> picoPorts;
  Vec<const Char*> picoItems;
  Int32  picoIndex = -1;

  // The console is a debug aid, not a record: this app runs for hours, so the log
  // is bounded and the oldest lines fall off the front.
  constexpr Size LOG_MAX = 4000;
  Vec<PicoLine> picoLog;

  // Console view state. The selection is by INDEX into picoLog, so it survives
  // the filter being retyped.
  Set<Int32> logSel;
  Int32      logSelAnchor = -1;   // for shift-click runs. -1 = nothing anchored
  Bool       logShowPoll = false;

  // Whether the most recent line SENT was one of the hub's own polls, so the
  // replies that follow it can be marked the same way. See pumpData().
  Bool lastSendWasPoll = false;
  Vec<Int32>      logShown;    // indices passing the filter, rebuilt per frame

  Array<Char, 192> cmdBuf= {};
  Array<Char, 64> filterBuf= {};
  Bool logAutoscroll = true;

  // Result of the last BOOTSEL touch, so a failure is not silent.
  Bool bootselDone = false;
  Bool bootselOk = false;

  // The modal lives at the root of the frame and this raises it: a popup's
  // identity is its ID stack, so OpenPopup and BeginPopupModal sit at one level.
  Bool openBootsel = false;

  // ------------------------------------------------------- controller state ---
  // What the board says about the car, not what the port says about the board.
  // tt02_control answers `?` with "S <uptimeMs> <a> <b> <servoUs> <escUs> <t>"
  // then "OK"; fields 4 and 5 are servo and ESC pulse widths in us (1500 =
  // neutral). pico_debug answers PING/ID/STATUS/HELP/LED and "ERR bad command".

  struct VehicleStatus
  {
      Bool               have = false;
      Float64             seenAt = 0.0;   // ImGui::GetTime() when the line landed
      UInt64 uptimeMs = 0;
      long               a = 0;
      long               b = 0;
      Int32                servoUs = 0;
      Int32                escUs = 0;
      UInt64 lastMs = 0;     // field 6, tracks uptime - last command
  };

  VehicleStatus vehicleStatus;
  Bool          vehUnsupported = false;   // `?` was answered, but not with an S line
  Bool          vehAwait = false;   // `?` sent, first reply not yet seen
  Str   lastCmd;                  // last line we sent, trimmed

  // ---- what pico_debug reports about itself ---------------------------------
  // Parsed out of `INFO status` / `INFO id` / `OK led ...` (firmware/app/main.c).
  // Separate from VehicleStatus because the two come from DIFFERENT firmware; at
  // most one of the two structs is ever live.
  struct DebugStatus
  {
      // Which board is on the other end - "pico2_w" or "pico2" - from INFO id.
      // The two RP2350 boards are indistinguishable over USB; only the firmware
      // knows, because it was COMPILED for one of them.
      Str boardName;
  };

  DebugStatus debugStatus;

  // STATUS polling. tt02_control answers `ERR bad command`, so one refusal stops
  // the polling - do not ask a board running it every two seconds forever.
  // ---- what the board says is attached ------------------------------------
  // The hub cannot see the Pico's pins; these are answers, and `tofAsked`
  // separates "no" from "not yet".
  Bool   sensorsAsked = false;
  Bool   sensorI2c = false;
  Bool   sensorTof = false;

  // The newest range and its status. A distance that arrived with a bad status
  // is not a shorter distance - it is not a distance - so the two never separate.
  Int32   tofMm = 0;
  Int32   tofStatus = 255;

  // The rates that came with the newest reading, in the sensor's own 16.16 fixed
  // point, or -1 when the firmware did not send them. Strong signal at short
  // range means something really is that close (lens film is the classic); weak
  // signal with a high ambient means it is blinded by room infrared.
  Int32   tofSignal = -1;
  Int32   tofAmbient = -1;

  // Which distance mode the board is in. The firmware boots in LONG, so that is
  // what this starts as - it is a mirror of the board's state, not a request.
  Bool    tofModeShort = false;

  // ---- the drive channels -------------------------------------------------
  // Mirrors of the BOARD's state, not requests: it owns the limits and the arming,
  // so safety survives the hub disconnecting.
  Bool  driveKnown = false;
  Int32 driveServo = 1500;   // what the board is outputting
  Int32 driveServoT = 1500;   // what it is heading toward
  Int32 driveEsc = 1500;
  Int32 driveEscT = 1500;
  Bool  driveArmed = false;

  // Whether the board is driving the steering pin at all - not the same as what
  // position it holds. A released servo holds nothing, which is the safe state.
  Bool  driveServoOn = false;

  // Where the board thinks center is. Not 1500 in general - see the calibration
  // block below for why that matters more than it sounds.
  Int32 driveServoC = 1500;

  // How fast the board lets an output move, in microseconds per 20 ms tick.
  // 8 is 400 us/s, which walks this car's 430 us of travel in about a second.
  Int32 driveSlew = 8;

  // What the slider shows, and whether it is under the thumb: a reply arriving
  // mid-drag must not yank the handle out from under the person moving it.
  Int32 driveSlewWant = 8;
  Bool  driveSlewHeld = false;

  // The throttle's own response rate, separate from the steering's.
  // ---- keyboard drive -------------------------------------------------------
  // Hold-to-drive on WASD. Off by default: this turns a text-editing application
  // into something that moves a vehicle.
  Bool  wasdOn = false;

  // The forward cap, in microseconds. Six us above the measured idle, about a
  // tenth of this car's 59 us band - deliberately a crawl.
  Int32 wasdCapUs = 1547;

  // The hard ceiling on that cap, in microseconds - the slider itself stops here.
  // Nine us over the measured idle of 1541, about a sixth of this car's 59 us
  // band: W is digital, so the number it slams to is all that keeps it sane. The
  // ceiling is wasdCapCeil(), below - it needs the calibration and clampInt.
#define WASD_CAP_HARD 1550

  // What was last SENT, so a command only goes out when the answer changes: at
  // 60 fps the naive version is sixty commands a second per axis.
  Int32   wasdSentSteer = 0;      // -1, 0, +1
  Int32   wasdSentEsc = 0;      // 0 = neutral, else the cap
  Float64 wasdFedAt = 0.0;    // last keepalive, for the board's deadman

  Int32 driveEscSlew = 8;
  Int32 driveEscSlewWant = 8;
  Bool  driveEscSlewHeld = false;

  // How far past idle the throttle must go before the tail lamps go out, in
  // microseconds. The board owns it; this follows unless the slider is dragged.
  Int32 lightsOffWant = 10;
  Bool  lightsOffHeld = false;
  Int32 boardLightsOff = 10;

  // Steering as a fraction of this car's travel, -1 to +1. The board sends it in
  // thousandths so no float has to survive a printf on a microcontroller.
  Float32 driveSteer = 0.0f;

  // Where the wheels ACTUALLY are, as against driveSteer which is where they were
  // told to go; the slew limiter is the difference. The drawing must use THIS
  // one - drivePump only moves servoNow while the servo is live, so the target
  // would turn the on-screen wheels while the indicator stayed dark.
  Float32 driveSteerNow = 0.0f;
  Float32 driveSteerWant = 0.0f;
  Bool    driveSteerHeld = false;

  // ---- the calibration ----------------------------------------------------
  // Three measurements of ONE car: the two ends its steering can reach, and where
  // its wheels point straight. None are derivable - a servo's 1000-2000 us range
  // says nothing about linkage length, and the horn meets its spline only at
  // whole-tooth intervals. Three places on purpose: the working copy here,
  // settings (survives a restart) and firmware/lib/chassis/cal.hxx on commit
  // (survives a reflash, and is what other code can read).
  Int32 calLeft = 1300;
  Int32 calCenter = 1500;
  Int32 calRight = 1700;
  Bool  calLoaded = false;
  Bool  calDirty = false;

  // What the header on disk says, so the view can show whether the working copy
  // has drifted from what the firmware would actually be built with.
  Str calWritten;

  // The limits the BOARD reports. Sliders are built from these, not constants
  // here, so tightening firmware tightens the UI.
  // ---- the indicator scaffolding. TEMPORARY - see firmware/lib/lights.h ----
  // What the BOARD says its lamps are doing: the rule runs in the firmware (the
  // car must indicate with no laptop attached) and this only draws the answer.
  Int32   boardTurn = 0;       // -1 left, 0 off, +1 right
  Bool    boardLightsOn = true;
  Float64 lightsLastPoll = 0.0;

  // Every lamp in the firmware's model, in its Lamp order:
  //   0 headL  1 headR  2 tailL  3 tailR
  //   4 indFL  5 indFR  6 indRL  7 indRR
  //   8 revL   9 revR
  //
  // Ten, not eight: the car gets front AND rear indicators, and the rear pair has
  // no LED yet. Levels 0..255 straight from the board, so a lamp with no LED
  // still has a correct level - wiring one changes a firmware table, not this.
  constexpr Int32 LAMP_N = 10;
  Array<Int32, LAMP_N> boardLamp= {};
  Array<Int32, LAMP_N> boardLampPin= { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };

  Int32 driveServoMin = 1300;
  Int32 driveServoMax = 1700;
  Int32 driveEscMin = 1500;
  Int32 driveEscMax = 1600;

  // What the sliders are showing. Separate from the board's value so dragging is
  // smooth - the replies arrive every 200 ms and would snap the handle.
  Int32 driveServoWant = 1500;
  Int32 driveEscWant = 1500;

  // ---- the sweep ----------------------------------------------------------
  // Driven from the hub, not the board: in firmware it would be a car that can
  // start moving on its own because of something left running in a UI.
  Bool    driveSweep = false;
  Float64 driveSweepNext = 0.0;
  Int32   driveSweepDir = 1;

  // The limits the user is editing, separate from what the board has accepted.
  // Widening is two steps - type it, then apply it - or it is no limit at all.
  Int32 driveLimitLo = 1300;
  Int32 driveLimitHi = 1700;
  Bool  driveLimitsDirty = false;

  Int32 driveEscLimitLo = 1500;
  Int32 driveEscLimitHi = 1600;
  Bool  driveEscLimitsDirty = false;

  // When the board was last asked what its drive state is. Throttled, because the
  // asking happens from a draw and a draw happens sixty times a second.
  Float64 driveAskedAt = -1.0;

  // Whether a slider handle is under the user's thumb RIGHT NOW. Per-slider, not
  // "is anything active": replies must move everything except the handle held.
  Bool driveServoHeld = false;
  Bool driveEscHeld = false;
  Float64 tofLastReply = 0.0;
  UInt64  tofReplies = 0;

  // A rolling history for the strip chart. Fixed size, oldest overwritten - a
  // chart that grows without bound is a leak with a picture on it.
  constexpr Int32 TOF_HISTORY = 240;
  Array<Float32, TOF_HISTORY> tofHistory= {};
  Int32   tofHistoryAt = 0;
  Bool    tofHistoryWrapped = false;

  // Extremes since connect. Sweeping the sensor and reading these off answers
  // "how far does it reach" for THIS sensor in THIS light.
  Int32 tofSeenMin = 0;
  Int32 tofSeenMax = 0;

  // ---- the cue board ------------------------------------------------------
  // THE BOARD IS THE SOURCE OF TRUTH: the list comes from CUE LIST once per
  // connection, so a cue added to firmware/lib/cue.hxx appears here unchanged.
  struct CueEntry
  {
      Str  name;      // what CUE <name> takes
      Str  means;     // the sentence the firmware gives it
      Str  play;      // "once", "loop" or "hold" - see below
      Bool on = false;      // the board says it is up
      Bool latched = false; // ...and a person put it there, not the car
  };

  // The play mode decides button or switch: a "once" cue plays and ends, while a
  // "hold" or "loop" cue stays up until lowered, so its button must be a TOGGLE.

  Vec<CueEntry> cueList;
  Bool          cueListAsked = false;

  // What the board says it is doing RIGHT NOW - a cue is a few hundred ms long.
  // ---- the Sound view. TEMPORARY -----------------------------------------
  // Scaffolding for bringing the DFPlayer up on a breadboard. Sound belongs in
  // the cue system - cue.hxx already carries a `tone` field waiting for it.
  Bool    soundReady = false;
  Int32   soundVol = 8;
  Int32   soundVolMax = 30;
  Int32   soundTrack = 1;
  Str     soundBusy = "unwired";
  Int32   soundTx = -1;
  Int32   soundRx = -1;
  Int32   soundBusyGp = -1;

  // How many tracks the CARD holds, 0 for "not asked yet or did not answer". The
  // hub keeps no list, so adding a track to the card is the whole of adding one.
  Int32   soundFiles = 0;
  Int32   soundEq = 0;
  Float64 soundLastPoll = 0.0;

  Str     cueSpeaking = "none";
  Int32   cueStepNow = 0;
  Int32   cueLoopNow = 0;
  Int32   cueKinds = 0;
  Float64 cueLastPoll = 0.0;

  Bool   dbgUnsupported = false;
  Bool   dbgAwait = false;
  Float64 dbgLastPoll = 0.0;

  Void resetBoardStatus()
  {
      debugStatus = DebugStatus();
      dbgUnsupported = false;
      sensorsAsked = false;

      // Asked again on the next connection: the list belongs to the running
      // firmware, and a reflash between connections is the ordinary case here.
      cueList.clear();
      cueListAsked = false;
      cueSpeaking = "none";
      sensorI2c = false;
      sensorTof = false;
      tofMm = 0;
      tofStatus = 255;
      tofReplies = 0;
      tofHistoryAt = 0;
      tofHistoryWrapped = false;
      tofSeenMin = 0;
      tofSeenMax = 0;
      tofSignal = -1;
      tofAmbient = -1;
      tofModeShort = false;
      driveKnown = false;
      driveArmed = false;
      driveServoOn = false;
      driveServoC = 1500;
      driveSlew = 8;
      driveSlewWant = 8;
      driveSlewHeld = false;
      driveSteer = 0.0f;
      driveSteerWant = 0.0f;
      driveSteerHeld = false;
      driveServo = 1500;
      driveServoT = 1500;
      driveEsc = 1500;
      driveEscT = 1500;
      driveServoWant = 1500;
      driveEscWant = 1500;
      dbgAwait = false;
      dbgLastPoll = 0.0;
  }

  // ----------------------------------------------------------------- flash ---
  // Catalog, board state, and the running script's output. PicoFlash works on a
  // worker thread; everything here is display plus the one confirm.

  constexpr Size FLASH_LOG_MAX = 3000;
  Vec<Str> flashLog;

  Array<Char, 320> backupBuf= {};
  Bool flashAutoscroll = true;

  // Set when the confirm modal is opened, so the modal can name what it is about
  // to destroy rather than saying "the firmware".
  Str confirmId;
  Str confirmName;
  Str confirmPath;

  FlashState flashPrev = FlashState::FLASH_STATE_IDLE;

  // --------------------------------------------------------------- sidebar ---
  // The right column's sections, in draw order. System and Sensors open by
  // default: they are the two you read.

  // Unscoped on purpose. SECTION_COUNT is an array bound and the rest are array
  // indices at two dozen sites; `enum class` would add a static_cast to each.
  enum Section
  {
      SECTION_SYSTEM = 0,
      SECTION_SENSORS,
      SECTION_VEHICLE,
      SECTION_FIRMWARE,
      SECTION_COUNT,
  };

  // Panel layout: section order, which sections are torn off into their own
  // windows, and the column width. All three persist. Floating windows' geometry
  // is ImGui's own business (layout.ini); only the fact that they ARE floating is
  // kept here, which ImGui has no way of knowing. The console is NOT here - see
  // drawConsoleColumn(). A settings file written before that has five records;
  // the loader accepts only a complete permutation, so the stale fifth is dropped.
  Array<Int32, SECTION_COUNT> sectionOrder = { SECTION_SYSTEM, SECTION_SENSORS,
                                               SECTION_VEHICLE, SECTION_FIRMWARE };
  Array<Bool, SECTION_COUNT> sectionFloating= {};

  // Logical (96-dpi) pixels, so the column keeps its apparent width across a DPI
  // change or a zoom rather than growing in one and not the other.
  Float32 sidebarLogicalW = 400.0f;

  // ---- the console column, on the LEFT ------------------------------------
  // Its own column, not a sidebar section: it is the one panel you read WHILE
  // doing something else, and the sidebar left it four lines tall.
  Float32 consoleLogicalW = 380.0f;
  Bool    consoleOpen = false;

  constexpr Float32 CONSOLE_MIN_W = 260.0f;
  constexpr Float32 CONSOLE_DEF_W = 380.0f;

  constexpr Float32 SIDEBAR_MIN_W = 260.0f;   // narrower than this and rows wrap

  // The Code view's file tree. LOGICAL pixels, like the sidebar, so a drag feels
  // the same at 100% and at 200% and the stored value survives a DPI change.
  constexpr Float32 CODE_TREE_MIN_W = 130.0f;   // below this the file names clip
  constexpr Float32 CODE_TREE_MAX_W = 640.0f;
  constexpr Float32 CODE_TREE_DEF_W = 240.0f;

  Float32 codeTreeLogicalW = CODE_TREE_DEF_W;
  Bool    codeTreeCollapsed = false;

  // ---- the Reference view is gone ------------------------------------------
  // Its seven hardcoded C++ pages are now .bdoc files in firmware/docs, opened
  // from the Code tree like any other file. A page was a rebuild; a document is
  // a file, and pinouts get corrected with the wires in your hands.

  // Which view the bottom control bar belongs to. The same thing as the open tab,
  // named separately because the bar asks "which view am I configuring".
  Int32 wsFocused = 0;

  Bool  panelLayoutDirty = false;             // written out at the end of a frame

  Void loadPanelLayout()
  {
      const Str txt = settings::read("panels.txt");
      if(txt.empty())
      {
          return;
      }

      // "w <px>" and "s <id> <floating>", one per line. Hand-written so a broken
      // file costs a default layout rather than a parser.
      Size i = 0;
      Int32 seen = 0;
      Array<Int32, SECTION_COUNT> order= {};

      while(i < txt.size())
      {
          const Size e = txt.find('\n', i);
          const Str line = txt.substr(i, (e == Str::npos) ? Str::npos : e - i);
          i = (e == Str::npos) ? txt.size() : e + 1;

          if(line.size() > 2 && line[0] == 'w')
          {
              const Float64 v = std::atof(line.c_str() + 1);
              if(v >= SIDEBAR_MIN_W && v <= 1600.0)
              {
                  sidebarLogicalW = static_cast<Float32>(v);
              }
          }
          else if(line.size() > 2 && line[0] == 'k')
          {
              Float64 v = 0.0;
              Int32   o = 0;
              if(std::sscanf(line.c_str() + 1, "%lf %d", &v, &o) == 2)
              {
                  if(v >= CONSOLE_MIN_W && v <= 1600.0)
                  {
                      consoleLogicalW = static_cast<Float32>(v);
                  }
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
                  {
                      codeTreeLogicalW = static_cast<Float32>(v);
                  }
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
          Array<Bool, SECTION_COUNT> used= {};
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
          {
              for(Int32 k = 0; k < SECTION_COUNT; ++k)
              {
                  sectionOrder[k] = order[k];
              }
          }
      }
  }

  Void savePanelLayout()
  {
      // Wide enough for the longest line here, which is a panel record.
      Array<Char, 160> buf;
      Str out;
      std::snprintf(buf.data(), buf.size(), "w %.0f\n", static_cast<Float64>(sidebarLogicalW));
      out += buf.data();
      std::snprintf(
          buf.data(),
          buf.size(),
          "t %.0f %d\n",
          static_cast<Float64>(codeTreeLogicalW),
          codeTreeCollapsed ? 1 : 0
      );
      out += buf.data();
      std::snprintf(
          buf.data(),
          buf.size(),
          "k %.0f %d\n",
          static_cast<Float64>(consoleLogicalW),
          consoleOpen ? 1 : 0
      );
      out += buf.data();

      for(Int32 k = 0; k < SECTION_COUNT; ++k)
      {
          std::snprintf(
              buf.data(),
              buf.size(),
              "s %d %d\n",
              sectionOrder[k],
              sectionFloating[sectionOrder[k]] ? 1 : 0
          );
          out += buf.data();
      }
      settings::write("panels.txt", out);
  }

  // --tab <name> opens one section at startup, for screenshots. The lidar sub-tab
  // names still work and open Sensors with that readout showing.
  Int32 forceSection = -1;
  Int32 forceSub = -1;
  Int32 forceTabFrames = 0;   // a tab bar only honors SetSelected once laid out

  // Map layers. Only the RPLIDAR exists today; the rest are declared so that
  // wiring one later is filling in a row rather than redesigning the screen.
  Bool layerLidar = true;
  Bool layerLidarPrev = true;
  Int32  selSensor = 0;    // which sensor the telemetry sub-tabs describe

  // The recorder. A second RadarView, so its trail, mode and accumulated map are
  // its own - scrubbing must not disturb the live map, nor the reverse.
  RadarView recView;
  rec::Recording recording;

  // ---- the Code view ---------------------------------------------------------
  // A file is edited here, saved where it already lives, and built and flashed by
  // the SAME scripts the Firmware panel uses. Each firmware/sketches/*.cxx has its
  // own CMake target named after the file, so Build & Flash writes the image
  // containing the file on screen.
  // ---- .bdoc: the page, or the source that made it -------------------------
  // One flag and the file it belongs to; switching documents starts on the PAGE.
  Str  docModeFor;        // the path the flag below is about
  Bool docModeSource = false;

  // Where the reader is on the page - zoom and pan, one value for the VIEW. Reset
  // when the file changes: a carried pan can open the next one entirely off-panel.
  refdoc::View docView;

  // The parsed document, cached against the text it came from. Reparsed only when
  // the text changes; a parse is a few hundred microseconds at this size.
  refdoc::Doc docParsed;
  Str         docParsedFrom;

  ed::Editor   codeEditor;
  ui::CodeView codeView;
  Str          codePath;        // absolute path of the open file, or empty
  Str          codeName;        // its display name
  Str          codeMessage;     // last save/build note, shown on the toolbar
  Bool         codeLoaded = false;

  // ---- IDE state -----------------------------------------------------------
  // Diagnostics from the last build, for the file on screen. NOT cleared when you
  // type - a stale mark beats no mark at all.
  Vec<diag::Item> codeDiags;

  // Last-write time of the open file, for noticing an edit made outside the app.
  // Zero means "not watching" - a file we have never successfully stat'd.
  UInt64 codeFileStamp = 0;
  Int32  codeWatchIn = 0;      // frames until the next stat

  // ---- where you were in each file -----------------------------------------
  //
  // THIS SESSION ONLY, deliberately. Nothing is written to disk: a scroll
  // position is a fact about what you are doing right now, not about the file,
  // and one restored from a week ago points at a line that has since moved.
  //
  // The caret goes with the scroll because restoring one without the other is
  // worse than restoring neither - the view lands where you left it and the
  // first arrow key jumps it back to line 1.
  struct CodeSpot
  {
      Float32 scrollY = 0.0f;
      Int32   line = 0;
      Int32   col = 0;
  };
  HashMap<Str, CodeSpot> codeSpots;

  // The directory part of a path, without the separator. Empty when there is
  // none. Both separators, because a path here can come from git, from the
  // Windows APIs, or from a #include line, and those disagree.
  static Str dirOf(const Str& path)
  {
      const Size a = path.find_last_of("/\\");
      return (a == Str::npos) ? Str() : path.substr(0, a);
  }

  // `dir` + `rel`, with `rel` taken as relative to it. Leading `../` segments
  // are folded rather than left in, so two spellings of one file do not look
  // like two files to the visited set.
  static Str joinPath(const Str& dir, const Str& rel)
  {
      Str out = dir.empty() ? rel : dir + "\\" + rel;
      for(Char& c : out)
      {
          if(c == '/')
          {
              c = '\\';
          }
      }

      Vec<Str> parts;
      Size     at = 0;
      while(at <= out.size())
      {
          const Size sep = out.find('\\', at);
          const Size stop = (sep == Str::npos) ? out.size() : sep;
          const Str  seg = out.substr(at, stop - at);

          if(seg == ".." && !parts.empty() && parts.back() != "..")
          {
              parts.pop_back();
          }
          else if(seg != "." && !seg.empty())
          {
              parts.push_back(seg);
          }

          if(sep == Str::npos)
          {
              break;
          }
          at = sep + 1;
      }

      Str joined;
      for(Size i = 0; i < parts.size(); ++i)
      {
          if(i > 0)
          {
              joined += "\\";
          }
          joined += parts[i];
      }
      return joined;
  }

  // ---- the macro index -----------------------------------------------------
  //
  // Every object-like #define the open file can see, collected by walking its
  // quoted includes. Built on open and after a save, not per keystroke: it
  // costs a few file reads and a macro's VALUE changes far less often than the
  // buffer does.
  //
  // Quoted includes only. <pico/stdlib.h> and friends resolve against the SDK,
  // which is thousands of files for a handful of macros a sketch ever hovers.
  Void rebuildMacroIndex()
  {
      codeView.macros.clear();
      if(codePath.empty())
      {
          return;
      }

      Vec<Str>     queue{ codePath };
      HashMap<Str, Bool> seen;
      Int32        budget = 64;      // files, not depth: a cycle cannot outrun it

      while(!queue.empty() && budget-- > 0)
      {
          const Str path = queue.back();
          queue.pop_back();
          if(seen.count(path) != 0u)
          {
              continue;
          }
          seen[path] = true;

          // The open buffer may be dirtier than the file on disk, and its own
          // macros are the ones most likely to be under the pointer.
          const Str text = (path == codePath) ? codeEditor.text() : sketch::load(path);
          if(text.empty())
          {
              continue;
          }

          const Str dir = dirOf(path);

          Size at = 0;
          Int32 lineNo = 0;
          while(at <= text.size())
          {
              const Size nl = text.find('\n', at);
              const Size stop = (nl == Str::npos) ? text.size() : nl;
              Str        line = text.substr(at, stop - at);
              ++lineNo;

              while(!line.empty() && (line.back() == '\r' || line.back() == ' '))
              {
                  line.pop_back();
              }

              Size i = 0;
              while(i < line.size() && (line[i] == ' ' || line[i] == '\t'))
              {
                  ++i;
              }

              if(i < line.size() && line[i] == '#')
              {
                  const Str rest = line.substr(i + 1);
                  if(rest.rfind("define", 0) == 0 && rest.size() > 6
                     && (rest[6] == ' ' || rest[6] == '\t'))
                  {
                      Size n = 6;
                      while(n < rest.size() && (rest[n] == ' ' || rest[n] == '\t'))
                      {
                          ++n;
                      }
                      const Size nameAt = n;
                      while(n < rest.size()
                            && (std::isalnum(static_cast<unsigned char>(rest[n])) != 0
                                || rest[n] == '_'))
                      {
                          ++n;
                      }

                      // FUNCTION-LIKE MACROS ARE SKIPPED. `#define F(x) ...` has
                      // no value without arguments, and showing its body under a
                      // pointer that is over a call site would be a lie about
                      // what that call becomes.
                      if(n > nameAt && !(n < rest.size() && rest[n] == '('))
                      {
                          const Str name = rest.substr(nameAt, n - nameAt);
                          while(n < rest.size() && (rest[n] == ' ' || rest[n] == '\t'))
                          {
                              ++n;
                          }
                          Str body = (n < rest.size()) ? rest.substr(n) : Str();

                          // A trailing comment is not part of the value.
                          if(const Size c = body.find("/*"); c != Str::npos)
                          {
                              body.resize(c);
                          }
                          if(const Size c = body.find("//"); c != Str::npos)
                          {
                              body.resize(c);
                          }
                          while(!body.empty() && (body.back() == ' ' || body.back() == '\t'))
                          {
                              body.pop_back();
                          }

                          if(codeView.macros.count(name) == 0u)
                          {
                              codeView.macros[name] =
                                  ui::CodeView::Macro{ body, path, lineNo };
                          }
                      }
                  }
                  else if(rest.rfind("include", 0) == 0)
                  {
                      const Size q = rest.find('"');
                      const Size r = (q == Str::npos) ? Str::npos : rest.find('"', q + 1);
                      if(r != Str::npos)
                      {
                          queue.push_back(joinPath(dir, rest.substr(q + 1, r - q - 1)));
                      }
                  }
              }

              if(nl == Str::npos)
              {
                  break;
              }
              at = nl + 1;
          }
      }
  }

  // Records where the open file is being left. Called before codePath changes,
  // and on close - anywhere the buffer is about to stop being the one on screen.
  Void rememberCodeSpot()
  {
      if(codePath.empty())
      {
          return;
      }
      const ed::Cursor c = codeEditor.cursor();
      codeSpots[codePath] = CodeSpot{ codeView.scrollY, c.line, c.col };
  }

  // Autosave. Counted in frames from the last edit rather than on a wall clock,
  // so it fires a moment after you STOP typing rather than mid-word.
  Bool  codeAutosave = true;
  Int32 codeAutosaveIn = 0;

  // A file the tree wants to act on, resolved after the menu closes - deleting an
  // entry while iterating the list that drew it is how a tree crashes.
  Str codePendingDelete;

  // Build & Flash is TWO operations, and which is in flight decides what a failure
  // means: one boolean cannot tell "queued" from "the flash itself just failed".
  enum class CodeOp
  {
      CODE_OP_NONE = 0,
      CODE_OP_BUILDING,
      CODE_OP_FLASHING,
  };

  // Set when an operation made us drop the Pico link, so it can be restored.
  // Frame-counted: the board re-enumerates a second or two AFTER the script exits.
  Bool  picoRelinkWanted = false;
  Int32 picoRelinkIn = 0;
  Str   picoRelinkPort;

  CodeOp codeOp = CodeOp::CODE_OP_NONE;
  // Which image the last Build was for, so the Flash that follows writes the same
  // one. "pico_debug" is the only target guaranteed to exist.
  Str    codeFlashTarget = "pico_debug";

  Bool    recArmed = false;   // capturing
  Bool    recPlaying = false;
  Float64 recStartS = 0.0;     // clock at the moment recording began
  Float64 recPlayS = 0.0;     // playback position, seconds into the recording
  Size    recIndex = 0;       // revolution currently shown
  Str     recStatus;            // last save/load result, shown to the user
  Bool    recStatusBad = false;

  // Set when the scrub moves, so the pump re-renders that frame once even though
  // playback is paused - without it, scrubbing while paused moves nothing.
  Bool    recPendingSeek = false;

  // Files on disk, refreshed on demand rather than every frame - a directory
  // listing per frame is a syscall per frame for a list that rarely changes.
  Vec<Str> recFiles;
  Int32            recFileIndex = 0;

  Void refreshRecordings()
  {
      recFiles = rec::list();
      if(recFileIndex >= static_cast<Int32>(recFiles.size()))
      {
          recFileIndex = 0;
      }
  }

  // Which central view is on screen. 0 is the flat map, 1 is the 3D scene,
  // 2 is the recorder, and 3 + boardIndex is a board. Persisted across frames
  // because the bottom bar belongs to the VIEW and the layout must reserve its
  // height before the tab bar has said which tab is selected.
  Int32 centralView = 0;

  // Vehicle lighting, driven by hand. NOTHING IS WIRED: this reaches exactly as
  // far as the 3D view, so docs/conventions.md's rules can be watched running.
  lights::Input& lightInput = radarView.lighting;

  // Automatic lighting: the lamps worked out from what the car is doing. Off by
  // default, or it would overwrite the bench panel's choices every frame.
  Bool              autoLights = false;
  lights::AutoState autoLightState;

  // Rolling rotation-rate history for the sparkline.
  constexpr Int32 HISTORY = 240;
  Array<Float32, HISTORY> hzHist= {};
  Int32   hzCount = 0;

  // The C1M1 datasheet rev 1.1 lists exactly ONE rate: Figures 2-1 and 2-8 both
  // give 460800, with no minimum and no maximum. There is no 115200 or 256000
  // mode to fall back to - picking one opens the port and returns nothing, which
  // does not look like a wrong setting. Kept disabled rather than deleted.
  struct BaudOpt
  {
      Int32       rate;
      const Char* label;
      Bool        supported;
  };

  constexpr BaudOpt BAUDS[] = {
      { 460800, "460800", true  },
      { 115200, "115200", false },
      { 256000, "256000", false },
  };
  constexpr Int32 BAUD_COUNT = static_cast<Int32>(sizeof(BAUDS) / sizeof(BAUDS[0]));

  struct RangeOpt { const Char* label; Float32 mm; };   // mm <= 0 means auto-fit
  constexpr RangeOpt RANGES[] = {
    { "Fit", 0.0f }, { "0.5 m", 500.0f }, { "1 m", 1000.0f }, { "2 m", 2000.0f },
    { "4 m", 4000.0f }, { "8 m", 8000.0f }, { "12 m", 12000.0f },
  };
  constexpr Int32 RANGE_COUNT = static_cast<Int32>((sizeof(RANGES) / sizeof(RANGES[0])));
  Array<const Char*, RANGE_COUNT> RANGE_ITEMS = {};

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
      // Captured BEFORE the list is replaced, and restored by NAME, not by index:
      // the enumeration reorders, so an index that meant COM7 can mean COM3 now.
      const Str wasSelected =
          (portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
              ? lidarPorts[static_cast<Size>(portIndex)]
              : Str();

      lidarPorts = LidarSource::listPorts();

      // A port that vanished while it was selected STAYS in the list: after an
      // unplug the combo should still read COM7, which is what you reconnect to.
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
      for(const auto& s : lidarPorts)
      {
          portItems.push_back(s.c_str());
      }

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
              if(_stricmp(lidarPorts[static_cast<Size>(i)].c_str(), wasSelected.c_str()) == 0)
              {
                  portIndex = i;
                  return;
              }
          }
      }

      // Identify the CP210x bridge outright where we can - the other ports on a
      // typical machine are Bluetooth links, which just time out confusingly.
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
      // Do NOT reinstate a "pick the highest COM number" fallback. Serial ports
      // are EXCLUSIVE: with the lidar unplugged that guess picked COM10, the Pico,
      // so the lidar stole the port from a board that was working. A Bluetooth
      // port - this machine has six - times out like a broken lidar.
      portIndex = -1;
  }

  // True when a port could belong to the lidar. Grays out the ones that cannot
  // rather than hiding them - a missing port looks like a driver problem.
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
  // exists - it is about the device NODE - so rescanning at once finds nothing.
  Int32 deviceScanIn = 0;

  // Backstop, in frames. A notification that never arrives - or arrives while the
  // window is not pumping messages - would leave the app permanently deaf.
  Int32 deviceScanIdle = 0;

  // Set when the user presses Disconnect, cleared when they press Connect. A
  // device reappearing reconnects EXCEPT after a deliberate Disconnect.
  Bool picoUserDisconnected = false;

  // ---- the car's address on the network -------------------------------------
  // Typed once and remembered. The ADDRESS only: the network password never
  // reaches the hub, it goes person -> board over the USB console and lives in
  // the board's RAM until reset. See firmware/lib/net.h.
  Array<Char, 64> wifiHost{};
  Bool wifiHostLoaded = false;

  const Char* const WIFI_HOST_FILE = "car-address.txt";

  Void loadWifiHost()
  {
      if(wifiHostLoaded)
      {
          return;
      }
      wifiHostLoaded = true;

      const Str saved = settings::read(WIFI_HOST_FILE);
      Size n = 0;
      while(n < saved.size() && n + 1 < wifiHost.size()
            && saved[n] != '\n' && saved[n] != '\r')
      {
          wifiHost[n] = saved[n];
          ++n;
      }
      wifiHost[n] = '\0';
  }
  Bool lidarUserDisconnected = false;

  Void refreshPicoPorts()
  {
      picoPorts = PicoLink::listPicoPorts();

      picoItems.clear();
      for(const auto& s : picoPorts)
      {
          picoItems.push_back(s.c_str());
      }

      if(picoPorts.empty())
      {
          picoIndex = -1;
          return;
      }
      if(picoIndex < 0 || picoIndex >= static_cast<Int32>(picoPorts.size()))
      {
          picoIndex = 0;
      }
  }

  Void connectPico()
  {
      picoUserDisconnected = false;

      LOG_INFO(
          "pico",
          "connect requested: port=%s",
          (picoIndex >= 0 && picoIndex < static_cast<Int32>(picoPorts.size())) ? picoPorts[picoIndex].c_str() : "(none)"
      );
      if(picoIndex < 0 || picoIndex >= static_cast<Int32>(picoPorts.size()))
      {
          return;
      }
      resetBoardStatus();
      picoLink.connect(picoPorts[picoIndex]);
  }

  Void sendPico(const Char* line)
  {
      if(!line || !line[0])
      {
          return;
      }
      picoLink.send(line);            // the link logs it; drain() gives it back to us
  }

  // The same thing for timer-driven traffic, marked so the console can hide it.
  // A separate function, not a default argument, so a call site reads as polling.
  Void pollPico(const Char* line)
  {
      if(!line || !line[0])
      {
          return;
      }
      picoLink.send(line, true);
  }

  Str trimLine(const Str& s)
  {
      Size a = 0, b = s.size();
      while(a < b && static_cast<UInt8>(s[a]) <= ' ')
      {
          ++a;
      }
      while(b > a && static_cast<UInt8>(s[b - 1]) <= ' ')
      {
          --b;
      }
      return s.substr(a, b - a);
  }

  // Reads meaning out of a line without assuming which firmware produced it. The
  // test is the SHAPE of the reply to `?`, not an error string: pico_debug answers
  // `?` with its HELP listing, so keying off "ERR" reports nothing at all. Not an
  // S line means no servo/ESC state is reported.
  Void observeLine(const PicoLine& ln)
  {
      const Str t = trimLine(ln.text);
      if(t.empty())
      {
          return;
      }

      if(ln.outgoing)
      {
          lastCmd = t;
          if(t == "?")
          {
              vehAwait = true;
          }
          if(t == "STATUS")
          {
              dbgAwait = true;
          }
          return;
      }

      // ---- pico_debug ------------------------------------------------------
      // "INFO status up_ms=... led=on blink_hz=2.00 lamp=gpio25 lamp_up=yes"
      // "INFO id board=pico2 sdk=... uid=... lamp=gpio25 lamp_up=yes"
      // cyw43= appears ONLY on a board that has the chip; its absence is a plain
      // Pico 2. Scanned for fields rather than parsed positionally.
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

          // "INFO cue flash [once] - I have seen you - after you". Split at the
          // FIRST " - " only: the meaning may contain the same separator.
          if(t.compare(0, 9, "INFO cue ") == 0)
          {
              const Str rest = t.substr(9);
              if(const Size cut = rest.find(" - "); cut != Str::npos)
              {
                  CueEntry e;
                  e.name = rest.substr(0, cut);
                  e.means = rest.substr(cut + 3);

                  // "name [mode]" -> the two of them. Firmware predating the
                  // modes sends no bracket, and an empty play draws as a button.
                  if(const Size br = e.name.find(" [");
                     br != Str::npos && e.name.back() == ']')
                  {
                      e.play = e.name.substr(br + 2, e.name.size() - br - 3);
                      e.name = e.name.substr(0, br);
                  }

                  Bool have = false;
                  for(const CueEntry& c : cueList)
                  {
                      if(c.name == e.name)
                      {
                          have = true;
                          break;
                      }
                  }
                  if(!have)
                  {
                      cueList.push_back(e);
                  }
              }
              return;
          }

          dbgUnsupported = false;
          dbgAwait = false;
          return;
      }

      // "OK sound ready=yes vol=8 max=30 track=1 busy=no tx=14 rx=15"
      if(t.compare(0, 9, "OK sound ") == 0)
      {
          const Char* p = t.c_str();

          // Read into a local first and only then commit: a slider being dragged
          // must not be yanked back by a reply that was already in flight.
          const auto num = [p](const Char* key, Int32 fallback)
          {
              const Char* q = std::strstr(p, key);
              if(q == nullptr)
              {
                  return fallback;
              }
              return std::atoi(q + std::strlen(key));
          };
          const auto word = [p](const Char* key, const Char* fallback)
          {
              const Char* q = std::strstr(p, key);
              if(q == nullptr)
              {
                  return Str(fallback);
              }
              q += std::strlen(key);
              Size n = 0;
              while(q[n] != '\0' && q[n] != ' ')
              {
                  ++n;
              }
              return Str(q, n);
          };

          soundReady = (word("ready=", "no") == "yes");
          soundVolMax = num("max=", 30);
          soundTrack = num("track=", soundTrack);
          soundBusy = word("busy=", "unwired");
          soundTx = num("tx=", -1);
          soundRx = num("rx=", -1);
          soundBusyGp = num("busyGp=", -1);
          soundFiles = num("files=", soundFiles);
          soundEq = num("eq=", soundEq);

          // The volume follows the board EXCEPT while the slider is held. See
          // above: the reply is older than the drag.
          if(!ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f))
          {
              soundVol = num("vol=", soundVol);
          }
          return;
      }

      // "OK cue speaking=flash step=0 loop=0 tone=0 kinds=2 off_us=10"
      if(t.compare(0, 7, "OK cue ") == 0)
      {
          const Char* p = t.c_str();

          if(const Char* q = std::strstr(p, "speaking="))
          {
              q += 9;
              Size n = 0;
              while(q[n] != '\0' && q[n] != ' ')
              {
                  ++n;
              }
              cueSpeaking = Str(q, n);
          }

          // "active=head*,brake,left*" - everything up, with a * on the ones a
          // PERSON raised. Both matter, and they look identical on the lamps.
          for(CueEntry& c : cueList)
          {
              c.on = false;
              c.latched = false;
          }

          if(const Char* q = std::strstr(p, "active="))
          {
              q += 7;

              Str tok;
              const auto flush = [&tok]()
              {
                  if(tok.empty() || tok == "-")
                  {
                      tok.clear();
                      return;
                  }
                  Bool lat = false;
                  if(tok.back() == '*')
                  {
                      lat = true;
                      tok.pop_back();
                  }
                  for(CueEntry& c : cueList)
                  {
                      if(c.name == tok)
                      {
                          c.on = true;
                          c.latched = lat;
                      }
                  }
                  tok.clear();
              };

              while(*q != '\0' && *q != ' ')
              {
                  if(*q == ',')
                  {
                      flush();
                  }
                  else
                  {
                      tok.push_back(*q);
                  }
                  ++q;
              }
              flush();
          }

          const auto num = [p](const Char* key, Int32& out)
          {
              if(const Char* q = std::strstr(p, key))
              {
                  out = std::atoi(q + std::strlen(key));
              }
          };
          num("step=",  cueStepNow);
          num("loop=",  cueLoopNow);
          num("kinds=", cueKinds);
          return;
      }

      // "OK drive servo=1500 servo_t=1500 esc=1500 esc_t=1500 armed=0 ...", read
      // by NAME so a field added later is ignored rather than shifting the rest.
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

          field("slew_esc=", driveEscSlew);
          if(!driveEscSlewHeld)
          {
              driveEscSlewWant = driveEscSlew;
          }

          Int32 milli = 0;
          field("steer_m=", milli);
          driveSteer = static_cast<Float32>(milli) / 1000.0f;

          // Absent on firmware older than this field; the default of 0 then
          // draws straight-ahead, which is the safe thing to show.
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
          // array alone: older firmware says less, not "every lamp is dark".
          const auto commas = [](const Char* q, Int32* out, Int32 n)
          {
              if(q == nullptr)
              {
                  return;
              }
              for(Int32 i = 0; i < n && *q != '\0' && *q != ' '; ++i)
              {
                  out[i] = std::atoi(q);
                  const Char* c = std::strchr(q, ',');
                  if(c == nullptr || *(c + 1) == '\0')
                  {
                      break;
                  }
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
              commas(q + 7, boardLamp.data(), LAMP_N);
          }
          if(const Char* q = std::strstr(p, "pins="))
          {
              commas(q + 5, boardLampPin.data(), LAMP_N);
          }
          return;
      }

      if(t.compare(0, 7, "OK stop") == 0)
      {
          driveArmed = false;
          driveServoWant = 1500;
          driveEscWant = 1500;
          LOG_INFO("pico", "stop acknowledged");
          return;
      }

      // "OK sensors i2c=1 tof=1 tof_addr=0x29" - what the board found at boot.
      // Read as key=value pairs, so a sensor added later does not break the parse.
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

          LOG_INFO("pico", "sensors: i2c=%d tof=%d", sensorI2c ? 1 : 0, sensorTof ? 1 : 0);
          return;
      }

      // "OK tof <mm> <status>", or "OK tof busy" when the measurement is not
      // finished. Busy is NOT an error: the sensor takes tens of milliseconds.
      if(t.compare(0, 7, "OK tof ") == 0)
      {
          const Char* a = t.c_str() + 7;
          if(std::strncmp(a, "busy", 4) == 0)
          {
              return;
          }

          // Signal and ambient are optional: older firmware sends two fields and
          // newer sends four, so read however many arrived.
          Int32 mm = 0;
          Int32 st = 0;
          Int32 sig = -1;
          Int32 amb = -1;
          const Int32 got = std::sscanf(a, "%d %d %d %d", &mm, &st, &sig, &amb);
          if(got >= 2)
          {
              tofSignal = (got >= 3) ? sig : -1;
              tofAmbient = (got >= 4) ? amb : -1;
              tofMm = mm;
              tofStatus = st;
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
          sensorTof = false;
          sensorsAsked = true;
          return;
      }

      // "OK led on" / "OK led off" / "OK led blink 2.00". Stores nothing, but MUST
      // recognize the line: anything unrecognized while dbgAwait is set stops the
      // STATUS polling permanently.
      if(t.compare(0, 7, "OK led ") == 0)
      {
          return;
      }

      if(dbgAwait)
      {
          // Anything else in reply to STATUS means this firmware has no such
          // command. Stop asking.
          dbgUnsupported = true;
          dbgAwait = false;
      }

      UInt64 up = 0, last = 0;
      long a = 0, b = 0;
      Int32  servo = 0, esc = 0;

      // sscanf_s rather than sscanf: no %s or %c here, so it needs no extra size
      // arguments, and it keeps /W4 clean without _CRT_SECURE_NO_WARNINGS.
      if(sscanf_s(
          t.c_str(),
          "S %llu %ld %ld %d %d %llu",
          &up,
          &a,
          &b,
          &servo,
          &esc,
          &last
      ) == 6)
      {
          vehicleStatus.have = true;
          vehicleStatus.seenAt = ImGui::GetTime();
          vehicleStatus.uptimeMs = up;
          vehicleStatus.a = a;
          vehicleStatus.b = b;
          vehicleStatus.servoUs = servo;
          vehicleStatus.escUs = esc;
          vehicleStatus.lastMs = last;
          vehUnsupported = false;
          vehAwait = false;
          return;
      }

      if(vehAwait)
      {
          vehUnsupported = true;
          vehAwait = false;
      }
  }

  // Drains once per frame, which is what PicoLink asks for, and keeps the log
  // bounded.
  Void pumpPico()
  {
      const Size before = picoLog.size();
      picoLink.drain(picoLog);

      // A reply inherits the flag from the request it answers - the link sees bytes,
      // not a protocol. The board answers in order, so the last SENT line is it.
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

      for(Size i = before; i < picoLog.size(); ++i)
      {
          observeLine(picoLog[i]);
      }

      if(picoLog.size() > LOG_MAX)
      {
          picoLog.erase(picoLog.begin(), picoLog.begin() + (picoLog.size() - LOG_MAX));
      }
  }

  // Asks the board what it is doing, at most every couple of seconds. Runs
  // unconditionally: the System panel is always visible and shows the reply.
  Void pollBoardStatus()
  {
      if(dbgUnsupported)
      {
          return;
      }
      if(picoLink.state() != PicoState::PICO_STATE_CONNECTED)
      {
          return;
      }

      const Float64 now = ImGui::GetTime();
      if(dbgLastPoll > 0.0 && (now - dbgLastPoll) < 2.0)
      {
          return;
      }

      dbgLastPoll = now;
      pollPico("STATUS");
  }

  // Asks the board what its indicator lamps are doing. Fast, to WATCH it blink: at
  // 1.5 Hz the lamp changes every 267 ms, so 120 ms fits. TEMPORARY scaffolding.
  Void pollLights()
  {
      if(dbgUnsupported)
      {
          return;
      }
      if(picoLink.state() != PicoState::PICO_STATE_CONNECTED)
      {
          return;
      }

      const Float64 now = ImGui::GetTime();
      if(lightsLastPoll > 0.0 && (now - lightsLastPoll) < 0.12)
      {
          return;
      }

      lightsLastPoll = now;
      pollPico("LIGHTS");
  }

  // Asks the board WHICH CUES IT HAS, once per connection: the table is compiled
  // in. A reflash gives a new connection and resetBoardStatus() clears the flag.
  Void pollCueList()
  {
      if(dbgUnsupported || cueListAsked)
      {
          return;
      }
      if(picoLink.state() != PicoState::PICO_STATE_CONNECTED)
      {
          return;
      }
      cueListAsked = true;
      pollPico("CUE LIST");
  }

  // And what it is saying right now. Fast for the same reason the lamp poll is:
  // a cue is a few hundred milliseconds end to end.
  Void pollCueState()
  {
      if(dbgUnsupported)
      {
          return;
      }
      if(picoLink.state() != PicoState::PICO_STATE_CONNECTED)
      {
          return;
      }

      const Float64 now = ImGui::GetTime();
      if(cueLastPoll > 0.0 && (now - cueLastPoll) < 0.12)
      {
          return;
      }

      cueLastPoll = now;
      pollPico("CUE");
  }

  // Slower than the cue poll on purpose: nothing here changes on its own except
  // BUSY, and the board answers a full status line to every command anyway.
  Void pollSoundState()
  {
      if(dbgUnsupported)
      {
          return;
      }
      if(picoLink.state() != PicoState::PICO_STATE_CONNECTED)
      {
          return;
      }

      const Float64 now = ImGui::GetTime();
      if(soundLastPoll > 0.0 && (now - soundLastPoll) < 0.5)
      {
          return;
      }

      soundLastPoll = now;
      pollPico("SOUND");
  }

  // Asks the board what is attached, once per connection: the answer cannot
  // change while the board runs, and this link also carries the readings.
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

  // Asks for a range reading, at a rate the sensor can sustain. Only while the
  // Range view is on screen - this link is also how firmware is flashed.
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
      const Float64  now = ImGui::GetTime();
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

      // A Pico that rebooted into BOOTSEL drops its CDC port BY DESIGN, on every
      // single flash. Calling that an error makes the normal path look broken.
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

      // Spelled out rather than left to the default, because it is a DECISION: an
      // unplugged device is not connected, and "Error" means something is wrong.
      case LidarState::LIDAR_STATE_UNPLUGGED:  return "Not connected";

      default:
          // "Not connected" would be a lie while the port is open and the motor is
          // simply parked, and the operator has to tell those apart at a glance.
          return lidarSource.connected() ? "Motor off" : "Not connected";
      }
  }

  // The same state in two colors, because it is printed on two grounds: the strip
  // is light chrome, the map HUD the dark viewport. See theme.hxx.
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

  // A silent board is the expected state - the flashed firmware only speaks when
  // spoken to - so this says so in words rather than showing an empty readout.
  Void picoAgeText(Char* buf, Size n, Float64 ageS)
  {
      if(ageS < 0.0)
      {
          std::snprintf(buf, n, "--");
      }
      else if(ageS < 600.0)
      {
          std::snprintf(buf, n, "%.1f s ago", ageS);
      }
      else
      {
          std::snprintf(buf, n, "%.0f min ago", ageS / 60.0);
      }
  }

  Bool logMatches(const PicoLine& ln)
  {
      if(filterBuf[0] == '\0')
      {
          return true;
      }

      const Char* hay = ln.text.c_str();
      for(; *hay; ++hay)
      {
          const Char* h = hay;
          const Char* n = filterBuf.data();
          while(*n && *h &&
                 std::tolower(static_cast<UInt8>(*h)) == std::tolower(static_cast<UInt8>(*n)))
                 {
                     ++h;
                     ++n;
                 }
          if(*n == '\0')
          {
              return true;
          }
      }
      return false;
  }

  Void recomputeDerived()
  {
      pointsPs = latestFrame.hz * static_cast<Float32>(latestFrame.points.size());

      Array<Float32, SECTORS> sectorMm= {};
      for(Int32 i = 0; i < QUALITY_BUCKETS; ++i)
      {
          qHist[i] = 0.0f;
      }
      for(Int32 i = 0; i < DIST_BUCKETS; ++i)
      {
          distHist[i] = 0.0f;
      }

      static Array<Bool, 360> binSeen;
      std::memset(binSeen.data(), 0, (binSeen.size() * sizeof(Bool)));

      Float64 sum = 0.0;
      Float64 qSum = 0.0;
      Int32    n = 0;
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
          if(p.distMm > maxMm)
          {
              maxMm = p.distMm;
          }

          qSum += p.quality;
          if(p.quality < qLo)
          {
              qLo = p.quality;
          }
          if(p.quality > qHi)
          {
              qHi = p.quality;
          }

          Int32 qb = static_cast<Int32>(p.quality) * QUALITY_BUCKETS / 64;
          qb = std::min(std::max(qb, 0), QUALITY_BUCKETS - 1);
          qHist[qb] += 1.0f;

          Int32 db = static_cast<Int32>((p.distMm / (MAX_VALID_MM / DIST_BUCKETS)));
          db = std::min(std::max(db, 0), DIST_BUCKETS - 1);
          distHist[db] += 1.0f;

          Int32 ab = static_cast<Int32>(p.angleDeg);
          if(ab >= 0 && ab < 360)
          {
              binSeen[ab] = true;
          }

          Int32 s = static_cast<Int32>((p.angleDeg / (360.0f / SECTORS)));
          s = std::min(std::max(s, 0), SECTORS - 1);

          // Nearest return wins the sector - that is the obstacle that matters.
          if(sectorMm[s] == 0.0f || p.distMm < sectorMm[s])
          {
              sectorMm[s] = p.distMm;
          }
      }

      nInspec = n;
      meanMm = n ? static_cast<Float32>((sum / n)) : 0.0f;
      maxRangeMm = maxMm;
      qMean = n ? static_cast<Float32>((qSum / n)) : 0.0f;
      qMin = n ? qLo : 0;
      qMax = n ? qHi : 0;

      Int32 covered = 0;
      for(Int32 i = 0; i < 360; ++i)
      {
          if(binSeen[i])
          {
              ++covered;
          }
      }
      coverageDeg = covered / 360.0f;

      qHistMax = 1.0f;
      for(Int32 i = 0; i < QUALITY_BUCKETS; ++i)
      {
          qHistMax = std::max(qHistMax, qHist[i]);
      }
      distHistMax = 1.0f;
      for(Int32 i = 0; i < DIST_BUCKETS; ++i)
      {
          distHistMax = std::max(distHistMax, distHist[i]);
      }

      // Capped, not scaled to the maximum: one open doorway at 8 m would
      // otherwise crush every near-field bar to invisibility.
      for(Int32 i = 0; i < SECTORS; ++i)
      {
          sectorM[i] = std::min(sectorMm[i] / 1000.0f, CLEARANCE_CAP_M);
      }

      if(hzCount < HISTORY)
      {
          hzHist[hzCount++] = latestFrame.hz;
      }
      else
      {
          std::memmove(hzHist.data(), hzHist.data() + 1, sizeof(Float32) * (HISTORY - 1));
          hzHist[HISTORY - 1] = latestFrame.hz;
      }
  }

  // ----------------------------------------------------------------- flash ---

  // vendor/ is where the existing backup lives, so restores are all in one place.
  // The date is in the name: the only thing you want to know is which is newer.
  Void defaultBackupName()
  {
      const Str root = PicoFlash::repoRoot();

      std::time_t t = std::time(nullptr);
      std::tm     lt{};
      localtime_s(&lt, &t);

      Array<Char, 32> stamp;
      std::strftime(stamp.data(), stamp.size(), "%Y%m%d-%H%M", &lt);

      std::snprintf(
          backupBuf.data(),
          backupBuf.size(),
          "%s\\vendor\\pico-flash-%s.uf2",
          root.empty() ? "." : root.c_str(),
          stamp.data()
      );
  }

  // The most useful line of the flash/build log: the last one the script marked as
  // an error, else the last line printed. Real failures are prefixed "[error]".
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
  // flash.ps1 reboots a running board by opening its port at 1200 baud, and
  // Windows gives serial ports exclusively - so with the hub holding the same
  // port the touch fails with "Access to the port 'COM10' is denied" and the
  // script reports "[error] RPI-RP2 never appeared." while the status bar says
  // PICO Connected. Restored once the board re-enumerates - see pumpPicoRelink().
  Void releasePicoPortForBoardOp()
  {
      if(picoLink.state() == PicoState::PICO_STATE_DISCONNECTED)
      {
          return;
      }

      picoRelinkPort = picoLink.port();
      picoRelinkWanted = true;
      picoRelinkIn = 0;      // armed by the operation finishing, not yet

      LOG_INFO("flash", "releasing %s so the 1200-baud touch can open it", picoRelinkPort.c_str());
      picoLink.disconnect();
  }

  // Reconnects after a board operation, once the port is back. Frame-counted
  // rather than immediate: flashing reboots the board, so the port vanishes and
  // returns a second or two after the script exits.
  // Defined further down; the per-frame pumps below need them and sit above them.
  Bool   saveSketch();

  // Notices a file edited outside the app and reloads it, SILENTLY when the buffer
  // is clean and never when it is dirty. Polled rather than watched with
  // ReadDirectoryChangesW: one GetFileAttributesEx twice a second costs nothing,
  // and a watch would need a thread and a handle to close.
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
          ui::setNote(codeView, "changed on disk (buffer is modified)", ImGui::GetTime());
          LOG_WARN("code", "%s changed on disk while the buffer was dirty", codePath.c_str());
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

  // clangd's, kept apart from both for the same reason: it republishes the whole
  // set on every keystroke, and merging would let one source clear the other's
  // marks. Also where clang-tidy shows up - clangd runs it in-process and
  // publishes on the same channel, told apart by the check name.
  Vec<diag::Item> codeLspDiags;

  // Frames until the buffer is re-linted. Counted from the last EDIT, like
  // autosave, so it never fires mid-word and never re-scans on every keystroke.
  Int32 codeLintIn = 0;

  // Keeps clangd running and pointed at the open file. STARTED LAZILY, not at
  // boot: clangd parses a Pico translation unit on its first question and holds
  // the result, worth several seconds and a few hundred megabytes. start() is
  // idempotent and cheap after the first call, so this is a plain per-frame call.
  Void refreshCodeDiags();      // defined just below; pumpCodeIntel calls it

  Void pumpCodeIntel()
  {
      if(centralView != 3)
      {
          return;
      }

      static_cast<Void>(lsp::start());

      // The view asks about whatever this says. Empty for an unsaved buffer, which
      // is correct: with no compile_commands.json entry clangd can only guess.
      codeView.lspPath = codePath;

      // clangd republishes the whole set whenever it reparses, a few hundred ms
      // after a keystroke, so this is polled - diagnostics() is false in between.
      if(lsp::diagnostics(codeLspDiags))
      {
          refreshCodeDiags();
      }
  }

  // Merges the compiler's opinion with the linter's, compiler first: the gutter
  // shows the WORST severity, and a build error must not hide behind a warning.
  Void refreshCodeDiags()
  {
      codeView.diags.clear();
      if(codePath.empty())
      {
          return;
      }

      // FILTERED BY FILE, every time. Clearing on open is not enough on its own:
      // clangd publishes asynchronously, so a set for the file just closed can
      // land after the switch and paint its line numbers onto the new buffer.
      // code_view.cxx draws every item it is handed without ever reading
      // Item::file, so this is the only place the question gets asked.
      for(const diag::Item& d : diag::forFile(codeDiags, codePath))
      {
          codeView.diags.push_back(d);
      }

      // NOT filtered, and it must not be: lint::check() never fills Item::file,
      // so forFile() would compare against an empty name and drop every one of
      // them. It does not need filtering either - it is recomputed from the open
      // buffer on open and after each pause in typing, so it cannot be stale.
      codeView.diags.insert(codeView.diags.end(), codeLintDiags.begin(), codeLintDiags.end());

      for(const diag::Item& d : diag::forFile(codeLspDiags, codePath))
      {
          codeView.diags.push_back(d);
      }
  }

  // Re-checks the buffer against docs/conventions.md a moment after typing stops.
  // SEPARATE FROM THE COMPILER: diagnostics.hxx reports what the build said, this
  // reports what the style audit will say at commit time. The rules are
  // tools/style_audit.py's, deliberately the same set.
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

  // Saves a few seconds after you stop typing, counted from the last EDIT so it
  // never fires mid-word. Only writes a file already named; it invents no path.
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

  // Rescans the ports after something was plugged in or unplugged. Without this,
  // both lists were built once at startup and a board plugged in later was never
  // noticed. Event-driven rather than polled: the answer is almost always
  // "nothing changed", and asking Windows every frame is 60 registry walks a sec.
  Void pumpDeviceScan()
  {
      // ~2 s at 60 fps. Only fires when a notification did not.
      constexpr Int32 IDLE_FRAMES = 120;

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
          due = true;
      }
      if(!due)
      {
          return;
      }

      const Bool hadPico = (picoIndex >= 0);
      const Bool hadLidar = (portIndex >= 0);

      refreshPicoPorts();
      refreshPorts();

      // ---- a board that has just appeared -----------------------------------
      // Reconnecting matches what the app does at startup, so plugging a board in
      // behaves the same as having it plugged in already. It does NOT override an
      // explicit Disconnect.
      if(!hadPico && picoIndex >= 0)
      {
          LOG_INFO("pico", "board appeared on %s", picoPorts[static_cast<Size>(picoIndex)].c_str());

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
          LOG_INFO(
              "lidar",
              "RPLIDAR adapter appeared on %s",
              lidarPorts[static_cast<Size>(portIndex)].c_str()
          );

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
              LOG_INFO("pico", "board is back on %s; reconnecting", picoRelinkPort.c_str());
              connectPico();
              return;
          }
      }
      // The port did not come back under the same name. Leaving it disconnected
      // is right: the Link panel shows what happened and a person can pick.
      LOG_WARN("pico", "%s did not come back after the operation", picoRelinkPort.c_str());
  }

  Void pumpFlash()
  {
      // Mirror the scripts' output into the session log: it is the toolchain's own
      // account of what it tried, and is otherwise lost when the panel is cleared.
      const Size before = flashLog.size();
      picoFlash.drainLog(flashLog);
      for(Size i = before; i < flashLog.size(); ++i)
      {
          const Str& ln = flashLog[i];

          // picotool draws a progress bar with carriage returns, arriving here as
          // enormous "Saving file: [====] 47%" lines. The log wants the outcome.
          if(ln.find("Saving file:") != Str::npos
             || ln.find("Loading into") != Str::npos)
          {
              continue;
          }

          const Bool bad = ln.find("[error]") != Str::npos;
          ::applog::writef(
              bad ? ::applog::Level::LEVEL_ERROR : ::applog::Level::LEVEL_DEBUG,
              "script",
              "%s",
              ln.c_str()
          );
      }
      if(flashLog.size() > FLASH_LOG_MAX)
      {
          flashLog.erase(flashLog.begin(), flashLog.begin() + (flashLog.size() - FLASH_LOG_MAX));
      }

      // An operation ending changes the world - a build makes a .uf2 appear, a
      // flash takes the COM port away. Re-scan once on the transition, not polled.
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

              // The compiler's own opinion of the file on screen, parsed from the
              // build output so the marks and the failure cannot disagree.
              {
                  const Vec<diag::Item> all = diag::parseAll(flashLog);
                  codeDiags = diag::forFile(all, codePath);
                  refreshCodeDiags();

                  if(!codeDiags.empty())
                  {
                      LOG_INFO(
                          "code",
                          "%d diagnostic(s) for %s",
                          static_cast<Int32>(codeDiags.size()),
                          codeName.c_str()
                      );
                  }
              }

              // The second half of the Code view's Build & Flash, chained on the
              // transition rather than started alongside it: PicoFlash runs one
              // operation at a time. A failure has to say WHY where the person is
              // looking - usually the board is simply not plugged in.
              if(codeOp == CodeOp::CODE_OP_BUILDING)
              {
                  if(s == FlashState::FLASH_STATE_SUCCESS)
                  {
                      codeOp = CodeOp::CODE_OP_FLASHING;
                      codeMessage = "built; flashing " + codeFlashTarget;
                      LOG_INFO("code", "build ok; flashing %s", codeFlashTarget.c_str());
                      releasePicoPortForBoardOp();
                      picoFlash.flash(codeFlashTarget);
                  }
                  else
                  {
                      codeOp = CodeOp::CODE_OP_NONE;
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
          if(layerLidar)
          {
              radarView.push(latestFrame);
          }
          haveFrame = true;
          recomputeDerived();

          // The recorder. Captured from the SAME frame the live map got, so a
          // recording is what was on screen, not a second sampling of the device.
          if(recArmed)
          {
              recording.append(latestFrame, ImGui::GetTime() - recStartS);
          }

          // Its view follows the live feed unless a recording is being played or
          // scrubbed - otherwise the tab sits black until you press something.
          if(recArmed || (!recPlaying && recording.empty()))
          {
              recView.push(latestFrame);
          }
      }

      // Turning the layer off empties the map once rather than every frame, so the
      // the trail does not linger and the fit history does not spring back on return.
      if(layerLidar != layerLidarPrev)
      {
          if(!layerLidar)
          {
              radarView.clear();
          }
          layerLidarPrev = layerLidar;
      }

      pumpPico();
      pumpFlash();
      pumpDeviceScan();
      pumpCodeLint();
      pumpCodeIntel();
      pumpPicoRelink();
      pumpCodeWatch();
      pumpCodeAutosave();
  }

  Void applyRange()
  {
      const Float32 mm = RANGES[rangeIndex].mm;
      if(mm <= 0.0f)
      {
          radarView.fit();
      }
      else
      {
          radarView.setRangeMm(mm);
      }
  }

  Void connect()
  {
      lidarUserDisconnected = false;

      LOG_INFO(
          "lidar",
          "connect requested: port=%s baud=%d",
          (portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size())) ? lidarPorts[portIndex].c_str() : "(none)",
          BAUDS[baudIndex].rate
      );
      if(portIndex < 0 || portIndex >= static_cast<Int32>(lidarPorts.size()))
      {
          return;
      }

      radarView.clear();
      haveFrame = false;
      hzCount = 0;
      lidarSource.start(lidarPorts[portIndex], BAUDS[baudIndex].rate);
  }

  Void startBackup()
  {
      // The board is about to be rebooted into BOOTSEL by backup.ps1, which takes
      // its COM port away; an open link would just fault.
      picoLink.disconnect();
      releasePicoPortForBoardOp();
      picoFlash.backup(backupBuf.data());
  }

  // ------------------------------------------------------------- HUD on map

  Void drawMapHud(const ImVec2& p0, const ImVec2& size)
  {
      // Minimal has no HUD. It is the one mode whose subject is the picture, and
      // a status line over it is the difference between a display and a readout.
      if(!radarView.is3D && radarView.mode == MapMode::MAP_MODE_MINIMAL)
      {
          return;
      }

      ImDrawList* dl = ImGui::GetWindowDrawList();
      ImFont* f = ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
      const Float32 px = f->LegacySize;
      const Float32 pad = 14.0f * uiDpiScale;

      const Char* stateText = lidarStateText();
      const ImU32 accent = lidarStateColorOnViewport();

      // ---- top left: state + connection -----------------------------------
      Float32 x = p0.x + pad;
      const Float32 y = p0.y + pad;

      ui::led(
          dl,
          ImVec2(x + px * 0.28f, y + px * 0.55f),
          px * 0.24f,
          accent,
          lidarSource.state() != LidarState::LIDAR_STATE_IDLE
      );
      x += px * 0.85f;
      dl->AddText(f, px, ImVec2(x, y), accent, stateText);
      x += f->CalcTextSizeA(px, FLT_MAX, 0.0f, stateText).x + 12.0f * uiDpiScale;

      if(portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
      {
          Array<Char, 64> conn;
          std::snprintf(
              conn.data(),
              conn.size(),
              "%s - %d baud",
              lidarPorts[portIndex].c_str(),
              BAUDS[baudIndex].rate
          );
          dl->AddText(f, px, ImVec2(x, y), ui::plot::LABEL, conn.data());
      }

      // ---- top right: throughput ------------------------------------------
      Array<Char, 96> thru;
      std::snprintf(
          thru.data(),
          thru.size(),
          "%.0f pts/s %.0f fps",
          pointsPs,
          ImGui::GetIO().Framerate
      );
      const Float32 tw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, thru.data()).x;
      dl->AddText(f, px, ImVec2(p0.x + size.x - pad - tw, y), ui::plot::LABEL, thru.data());

      // ---- bottom left: cursor / measurement -------------------------------
      Array<Char, 128> read;
      read[0] = '\0';

      if(radarView.measuring)
      {
          std::snprintf(read.data(), read.size(), "measure   %.2f m", radarView.measureMm / 1000.0f);
      }
      else if(radarView.cursorValid)
      {
          std::snprintf(
              read.data(),
              read.size(),
              "%.1f deg %.2f m",
              radarView.cursorBearingDeg,
              radarView.cursorRangeMm / 1000.0f
          );
      }

      if(read[0])
      {
          const Float32 rw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, read.data()).x;
          const ImVec2 bp(p0.x + pad, p0.y + size.y - pad - px - 10.0f * uiDpiScale);
          const ImVec2 be(bp.x + rw + 18.0f * uiDpiScale, bp.y + px + 12.0f * uiDpiScale);
          // A raised plate, the same treatment the buttons get: a readout sitting
          // on the display still belongs to the machine around it.
          ui::plate(bp, be, IM_COL32(0x33, 0x36, 0x3B, 0xF2), ImGui::GetStyle().FrameRounding);
          dl->AddText(
              f,
              px,
              ImVec2(bp.x + 9.0f * uiDpiScale, bp.y + 6.0f * uiDpiScale),
              IM_COL32(235, 238, 242, 255),
              read.data()
          );
      }

      // ---- bottom right: zoom state ----------------------------------------
      Array<Char, 96> zoom;
      std::snprintf(
          zoom.data(),
          zoom.size(),
          "%s %.1f m across",
          radarView.isAutoFit() ? "fit" : "manual",
          radarView.visibleRangeMm() * 2.0f / 1000.0f
      );
      const Float32 zw = f->CalcTextSizeA(px, FLT_MAX, 0.0f, zoom.data()).x;
      dl->AddText(
          f,
          px,
          ImVec2(p0.x + size.x - pad - zw, p0.y + size.y - pad - px),
          ui::plot::LABEL,
          zoom.data()
      );

      // ---- second line, top left: the active mode and its reading ----------
      // A mode is a picture until it produces a number. This is where the number
      // goes - the widest gap, the tightest sector, how much came back unusable -
      // so the view and its measurement are read together.
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
          {
              dl->AddText(f, px, ImVec2(mx, my), ui::plot::LABEL, radarView.diag.data());
          }
      }
  }

  // ------------------------------------------------------------ small parts

  // A metric and its label. Deliberately NOT color-coded: the caption says which
  // number it is, and color is reserved for values that carry a state (ui::sem).
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
      Array<Char, 128> buf;
      va_list ap;
      va_start(ap, fmt);
      std::vsnprintf(buf.data(), buf.size(), fmt, ap);
      va_end(ap);

      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextDisabled("%s", k);
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(buf.data());
  }

  Void colored(ImU32 col, const Char* fmt, ...)
  {
      Array<Char, 192> buf;
      va_list ap;
      va_start(ap, fmt);
      std::vsnprintf(buf.data(), buf.size(), fmt, ap);
      va_end(ap);

      ImGui::PushStyleColor(ImGuiCol_Text, col);
      ImGui::TextUnformatted(buf.data());
      ImGui::PopStyleColor();
  }

  // ====================================================================== strip
  // Always visible: "is it connected" must never be one click away.

  // One field of the status strip: an icon, a dim name, the state, and whatever
  // detail belongs with it. No lamp: the strip is exactly one text line tall and
  // a lit lamp's glow extends about 2.6x its radius, so every halo was clipped by
  // the child. Identity comes from the icon, state from the color.
  //
  // The status bar's fields are DRAWN rather than flowed. SameLine aligns items by
  // their TOP edge, so a 16 px icon sits high against 20 px text, and padding
  // cannot fix it because the two sizes move independently with DPI. So the bar
  // owns a centerline and `x` advances explicitly.

  // Vertical center of the bar, and the pen position along it.
  struct BarPen
  {
      ImDrawList* dl = nullptr;
      Float32     x = 0.0f;   // advances left to right
      Float32     cy = 0.0f;   // the centerline, in screen space
  };

  Void barText(BarPen& p, const Char* text, ImU32 col)
  {
      if(text == nullptr || text[0] == 0)
      {
          return;
      }

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
  // padding. COMPUTED: a hard-coded count breaks the moment the type scale does.
  const Char* iconTabLabel(Char* buf, Size cap, const Char* name)
  {
      const Float32 spaceW = ImGui::CalcTextSize(" ").x;
      Int32 n = 3;
      if(spaceW > 0.0f)
      {
          n = static_cast<Int32>(std::ceil(
              (ui::iconSize() + ImGui::GetStyle().ItemInnerSpacing.x) / spaceW
          ));
      }
      if(n < 1)
      {
          n = 1;
      }
      if(n > 24)
      {
          n = 24;
      }
      std::snprintf(buf, cap, "%*s%s", n, "", name);
      return buf;
  }

  Void tabIcon(ui::Icon ic)
  {
      if(!ui::iconsReady())
      {
          return;
      }
      const ImVec2  a = ImGui::GetItemRectMin();
      const ImVec2  b = ImGui::GetItemRectMax();
      const Float32 sz = ui::iconSize();
      ui::iconAt(
          ImGui::GetWindowDrawList(),
          ic,
          ImVec2(a.x + ImGui::GetStyle().FramePadding.x, a.y + ((b.y - a.y) - sz) * 0.5f)
      );
  }

  // The UI zoom: A- / A+ and the current percentage, right-aligned in the bar and
  // centered on its line. Text rather than icons - the Fugue subset in
  // assets/icons carries no magnifier. Ctrl +/-/0 work too. Returns the x it
  // started at, so the caller knows where the fields must stop.
  Float32 drawZoomControl(Float32 cy, Float32 rightEdge)
  {
      const ImGuiStyle& sty = ImGui::GetStyle();

      Array<Char, 16> pct;
      std::snprintf(
          pct.data(),
          pct.size(),
          "%d%%",
          static_cast<Int32>(ui::userScale() * 100.0f + 0.5f)
      );

      const Float32 btnW = ImGui::CalcTextSize("A+").x + sty.FramePadding.x * 2.0f;
      const Float32 pctW = ImGui::CalcTextSize("000%").x;
      const Float32 gap = sty.ItemInnerSpacing.x;
      const Float32 need = btnW * 2.0f + pctW + gap * 2.0f;

      const Float32 x0 = rightEdge - need;

      // A SmallButton is NOT GetFrameHeight() tall: ImGui forces FramePadding.y to
      // zero, so centering on GetFrameHeight() lifts it by one padding.
      const Float32 bh = ImGui::GetTextLineHeight();

      const Bool atMin = ui::userScale() <= ui::USER_SCALE_MIN + 0.001f;
      const Bool atMax = ui::userScale() >= ui::USER_SCALE_MAX - 0.001f;

      // SmallButton is a real widget, so it is POSITIONED on the centerline
      // rather than drawn on it: place the cursor at (center - height/2).
      ImGui::SetCursorScreenPos(ImVec2(x0, cy - bh * 0.5f));

      ImGui::BeginDisabled(atMin);
      if(ImGui::SmallButton("A-"))
      {
          ui::setUserScale(ui::userScale() - ui::USER_SCALE_STEP);
      }
      ImGui::EndDisabled();
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("Smaller  (Ctrl -)");
      }

      // The percentage is text, so it centers on the line directly - and inside
      // its OWN slot, sized for "000%" so the buttons never move.
      const ImVec2   psz = ImGui::CalcTextSize(pct.data());
      const Float32  slot = x0 + btnW + gap;
      const Float32  px = slot + ((pctW - psz.x) * 0.5f);
      ImGui::GetWindowDrawList()->AddText(
          ImVec2(px, cy - psz.y * 0.5f),
          ImGui::GetColorU32(ImGuiCol_TextDisabled),
          pct.data()
      );

      // An invisible hit box, so click-to-reset and the tooltip still work now the
      // text is drawn. Over the whole SLOT, so the target does not shrink.
      ImGui::SetCursorScreenPos(ImVec2(slot, cy - psz.y * 0.5f));
      ImGui::InvisibleButton("##zoompct", ImVec2(pctW, psz.y));
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("UI scale. Ctrl 0 resets it to 100%%.");
      }
      if(ImGui::IsItemClicked())
      {
          ui::setUserScale(1.0f);
      }

      ImGui::SetCursorScreenPos(ImVec2(x0 + btnW + pctW + gap * 2.0f, cy - bh * 0.5f));
      ImGui::BeginDisabled(atMax);
      if(ImGui::SmallButton("A+"))
      {
          ui::setUserScale(ui::userScale() + ui::USER_SCALE_STEP);
      }
      ImGui::EndDisabled();
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("Bigger  (Ctrl +)");
      }

      return x0;
  }

  // The bottom status bar: what every subsystem is doing, in one line. At the
  // BOTTOM because it is ambient information you glance at, not a header.
  Void drawStatusBar()
  {
      const ImVec2 p0 = ImGui::GetCursorScreenPos();
      const ImVec2 av = ImGui::GetContentRegionAvail();

      BarPen pen;
      pen.dl = ImGui::GetWindowDrawList();
      pen.x = p0.x;
      pen.cy = p0.y + av.y * 0.5f;   // the one centerline everything shares

      // The zoom control first, so the fields know where they have to stop.
      const Float32 stopAt = drawZoomControl(pen.cy, p0.x + av.x);

      // ---- lidar ----------------------------------------------------------
      Array<Char, 64> lidarExtra= {};
      if(lidarSource.state() == LidarState::LIDAR_STATE_SCANNING)
      {
          std::snprintf(
              lidarExtra.data(),
              lidarExtra.size(),
              "%s %.1f Hz",
              (portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size())) ? lidarPorts[portIndex].c_str() : "",
              haveFrame ? latestFrame.hz : 0.0f
          );
      }
      else if(portIndex >= 0 && portIndex < static_cast<Int32>(lidarPorts.size()))
      {
          std::snprintf(lidarExtra.data(), lidarExtra.size(), "%s", lidarPorts[portIndex].c_str());
      }

      stripField(
          pen,
          ui::Icon::ICON_RADAR,
          "LIDAR",
          lidarStateColor(),
          lidarStateText(),
          lidarExtra.data()
      );

      // ---- pico link -------------------------------------------------------
      stripSep(pen);
      const PicoState ps = picoLink.state();
      const Str pport = picoLink.port().empty()
          ? (picoIndex >= 0 && picoIndex < static_cast<Int32>(picoPorts.size())
                 ? picoPorts[picoIndex] : Str())
          : picoLink.port();
      stripField(
          pen,
          ui::Icon::ICON_PROCESSOR,
          "PICO",
          picoStateColor(ps),
          picoStateText(ps),
          pport.c_str()
      );

      // ---- board -----------------------------------------------------------
      stripSep(pen);
      const BoardStatus brd = picoFlash.board();
      if(brd.bootsel)
      {
          stripField(
              pen,
              ui::Icon::ICON_FIRMWARE,
              "BOARD",
              ui::sem::WARN,
              "BOOTSEL",
              brd.drive.c_str()
          );
      }
      else if(brd.present)
      {
          stripField(
              pen,
              ui::Icon::ICON_FIRMWARE,
              "BOARD",
              ui::sem::GOOD,
              "Running",
              brd.program.c_str()
          );
      }
      else
      {
          stripField(pen, ui::Icon::ICON_FIRMWARE, "BOARD", ui::sem::MUTED, "absent", "");
      }

      // ---- long-running operation ------------------------------------------
      // Dropped, not overlapped, when the window cannot clear the zoom control.
      const FlashState fs = picoFlash.state();
      if(pen.x + ImGui::GetFontSize() * 8.0f < stopAt)
      {
          stripSep(pen);
          if(fs == FlashState::FLASH_STATE_WORKING)
          {
              stripField(
                  pen,
                  ui::Icon::ICON_BUILD,
                  "OP",
                  ui::sem::WARN,
                  "running",
                  picoFlash.currentOp().c_str()
              );
          }
          else if(fs == FlashState::FLASH_STATE_SUCCESS)
          {
              stripField(
                  pen,
                  ui::Icon::ICON_BUILD,
                  "OP",
                  ui::sem::GOOD,
                  "done",
                  picoFlash.currentOp().c_str()
              );
          }
          else if(fs == FlashState::FLASH_STATE_FAILED)
          {
              stripField(
                  pen,
                  ui::Icon::ICON_BUILD,
                  "OP",
                  ui::sem::BAD,
                  "FAILED",
                  picoFlash.currentOp().c_str()
              );
          }
          else
          {
              stripField(pen, ui::Icon::ICON_BUILD, "OP", ui::sem::MUTED, "idle", "");
          }
      }
  }

  // =================================================================== overview
  // Every subsystem's state and the actions reached for most often. No status is
  // invented for something that has never been wired.

  // name | state | live value. The third column stays empty when there is no live
  // value - an empty cell is the honest reading.
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
          const Float32 r = ImGui::GetFontSize() * 0.20f;
          const ImVec2  cp = ImGui::GetCursorScreenPos();
          ui::led(
              ImGui::GetWindowDrawList(),
              ImVec2(cp.x + r * 1.8f, cp.y + ImGui::GetTextLineHeight() * 0.5f),
              r,
              col,
              lit
          );
          ImGui::Dummy(ImVec2(r * 3.8f, ImGui::GetTextLineHeight()));
          ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
      }
      colored(col, "%s", state);

      ImGui::TableNextColumn();
      if(value && value[0])
      {
          ImGui::TextUnformatted(value);
      }
  }

  Void drawSubsystems()
  {

      if(!ImGui::BeginTable("subsys", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg))
      {
          return;
      }

      // The RPLIDAR deliberately has no row here: the Sensors section below in
      // this same column, and the strip at the top, already carry it.
      const PicoState ps = picoLink.state();
      subsystemRow(
          ui::Icon::ICON_LINK,
          "Pico link",
          picoStateColor(ps),
          picoStateText(ps),
          picoLink.port().c_str(),
          ps != PicoState::PICO_STATE_DISCONNECTED
      );

      const BoardStatus brd = picoFlash.board();
      if(brd.bootsel)
      {
          subsystemRow(
              ui::Icon::ICON_FIRMWARE,
              "Board firmware",
              ui::sem::WARN,
              "BOOTSEL",
              brd.drive.c_str()
          );
      }
      else if(brd.present)
      {
          // The board NAME once the firmware has answered ID, else the chip.
          // "RP2350" is true of both boards here and distinguishes neither.
          subsystemRow(
              ui::Icon::ICON_FIRMWARE,
              "Board firmware",
              ui::sem::GOOD,
              brd.program.empty() ? "running" : brd.program.c_str(),
              debugStatus.boardName.empty() ? brd.chip.c_str() : debugStatus.boardName.c_str()
          );
      }
      else
      {
          subsystemRow(
              ui::Icon::ICON_FIRMWARE,
              "Board firmware",
              ui::sem::MUTED,
              "absent",
              "",
              false
          );
      }

      // Nothing below is connected, so nothing below reports a value.
      subsystemRow(ui::Icon::ICON_SERVO,   "Servo (GP0)",           ui::sem::MUTED, "not driven", "", false);
      subsystemRow(ui::Icon::ICON_SERVO,   "ESC (GP1)",             ui::sem::MUTED, "not driven", "", false);
      // All four XSHUT pins have an LED on them now - GP11/GP10 the headlights,
      // GP13/GP12 the front indicators. The row says where the SENSORS are going.
      subsystemRow(
          ui::Icon::ICON_TOF,
          "ToF bumpers (GP10-13)",
          ui::sem::WARN,
          "lamps on those pins",
          "",
          false
      );
      subsystemRow(
          ui::Icon::ICON_ENCODER,
          "Wheel encoder (GP15)",
          ui::sem::MUTED,
          "not wired",
          "",
          false
      );
      subsystemRow(ui::Icon::ICON_IMU,     "IMU (I2C)",             ui::sem::MUTED, "not wired",  "", false);
      subsystemRow(ui::Icon::ICON_STORAGE, "MicroSD (SPI)",         ui::sem::MUTED, "no header",  "", false);
      // Was "not built" for the whole of phase 2, and is now a real answer.
      if(picoLink.wireless())
      {
          subsystemRow(
              ui::Icon::ICON_NETWORK,
              "UDP link",
              picoStateColor(ps),
              picoStateText(ps),
              picoLink.port().c_str(),
              true
          );
      }
      else
      {
          subsystemRow(
              ui::Icon::ICON_NETWORK,
              "UDP link",
              ui::sem::MUTED,
              "not in use",
              wifiHost[0] != '\0' ? wifiHost.data() : "",
              false
          );
      }

      ImGui::EndTable();
  }

  // Linking to the car over Wi-Fi, in two deliberately separate steps:
  //   1. On the USB console, once:  WIFI JOIN <ssid> <password>
  //      The board answers with its address. The password is never stored.
  //   2. Here: that address, and a button.
  // After that the wireless link IS the link - everything is written against
  // PicoLink::send(), which does not care whether it holds a port or a socket.
  Void drawWifiLink(Float32 bh)
  {
      loadWifiHost();

      const PicoState ps = picoLink.state();
      const Bool      live = picoLink.wireless()
                          && (ps == PicoState::PICO_STATE_CONNECTED
                              || ps == PicoState::PICO_STATE_CONNECTING);

      ImGui::SeparatorText("Wireless link");

      ImGui::BeginDisabled(live);
      ImGui::SetNextItemWidth(-FLT_MIN);
      if(ImGui::InputTextWithHint(
          "##carip",
          "the car's address, e.g. 192.168.1.42",
          wifiHost.data(),
          wifiHost.size()
      ))
      {
          settings::write(WIFI_HOST_FILE, Str(wifiHost.data()));
      }
      ImGui::EndDisabled();

      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Where the car is on the network.\n" "\n" "The board says it. Plug it in, and on the console:\n" "\n" " WIFI JOIN <your network> <your password>\n" "\n" "It answers with its address once the router has given it one.\n" "The password stays between you and the board - it is never sent\n" "here, never written to disk, and a reset forgets it."
          );
      }

      if(live)
      {
          if(ui::iconButton(
              ui::Icon::ICON_PLUG_DISCONNECT,
              "Drop the Wi-Fi link",
              ImVec2(-FLT_MIN, bh),
              ui::Tint::TINT_WARN
          ))
          {
              LOG_INFO("pico", "wireless link dropped by user");
              picoUserDisconnected = true;
              picoLink.disconnect();
          }
      }
      else
      {
          // Disabled with a REASON on the tooltip, never bare: a grayed control
          // with no explanation reads as a broken one.
          const Bool wired = (ps == PicoState::PICO_STATE_CONNECTED
                              || ps == PicoState::PICO_STATE_CONNECTING);
          const Bool ready = (wifiHost[0] != '\0') && !wired;

          ImGui::BeginDisabled(!ready);
          if(ui::iconButton(
              ui::Icon::ICON_NETWORK,
              "Link over Wi-Fi",
              ImVec2(-FLT_MIN, bh),
              ui::Tint::TINT_GOOD
          ))
          {
              LOG_INFO("pico", "linking to %s over UDP", wifiHost.data());
              picoUserDisconnected = false;
              picoLink.connectUdp(Str(wifiHost.data()));
          }
          ImGui::EndDisabled();

          if(ImGui::IsItemHovered())
          {
              if(wired)
              {
                  ImGui::SetTooltip(
                      "The cable link is open. Disconnect it first - one link at\n" "a time, or two consoles would be talking over each other."
                  );
              }
              else if(wifiHost[0] == '\0')
              {
                  ImGui::SetTooltip("Needs the car's address, above.");
              }
              else
              {
                  ImGui::SetTooltip(
                      "Sends the same commands to the same firmware, over UDP\n" "port 4242 instead of the cable.\n" "\n" "It stays amber until the board actually answers. UDP has\n" "no connection to make, so anything else would report a\n" "car that is switched off as present.\n" "\n" "Flashing still needs the cable."
                  );
              }
          }
      }
  }

  Void drawQuickActions()
  {
      const Float32 bh = ImGui::GetFrameHeight() * 1.2f;
      const Bool  busy = picoFlash.busy();

      ImGui::SeparatorText("Quick actions");

      if(ImGui::BeginTable("quick", 2, ImGuiTableFlags_SizingStretchSame))
      {
          ImGui::TableNextRow();

          ImGui::TableNextColumn();
          // Keyed on the LINK, not on scanning: off isBusy(), pausing the motor
          // replaced the very button that would start it again.
          if(lidarSource.connected())
          {
              // Spin control, not link control: the noise and the bearing wear
              // come from the rotor, and this keeps the device open.
              const Bool spinning = lidarSource.motorEnabled();
              // Amber to stop a spinning rotor, green to start one: the tint is
              // the claim about what pressing it does, and it flips with the verb.
              if(ui::iconButton(
                  spinning ? ui::Icon::ICON_MOTOR_STOP : ui::Icon::ICON_MOTOR_RUN,
                  spinning ? "Stop motor" : "Start motor",
                  ImVec2(-FLT_MIN, bh),
                  spinning ? ui::Tint::TINT_WARN : ui::Tint::TINT_GOOD
              ))
              {
                  lidarSource.setMotorEnabled(!spinning);
              }
          }
          else
          {
              ImGui::BeginDisabled(lidarPorts.empty());
              if(ui::iconButton(
                  ui::Icon::ICON_PLUG_CONNECT,
                  "Connect lidar",
                  ImVec2(-FLT_MIN, bh),
                  ui::Tint::TINT_GOOD
              ))
              {
                  connect();
              }
              ImGui::EndDisabled();
          }

          ImGui::TableNextColumn();
          const PicoState ps = picoLink.state();
          if(ps == PicoState::PICO_STATE_CONNECTED || ps == PicoState::PICO_STATE_CONNECTING)
          {
              if(ui::iconButton(
                  ui::Icon::ICON_PLUG_DISCONNECT,
                  "Disconnect Pico",
                  ImVec2(-FLT_MIN, bh),
                  ui::Tint::TINT_WARN
              ))
              {
                  // Deliberate, so a rescan must not grab the port straight back.
                  // ONLY the button sets this: a flash or BOOTSEL touch is not
                  // intent and must not stop the board reconnecting.
                  LOG_INFO("pico", "disconnect requested by user (state=%s)", picoStateText(ps));
                  picoUserDisconnected = true;
                  picoLink.disconnect();
                  LOG_INFO("pico", "disconnect returned");
              }
          }
          else
          {
              // Grayed out when no board is present - and a grayed-out control
              // with no explanation reads as a broken one. It gets a reason.
              const Bool noPico = (picoIndex < 0);
              ImGui::BeginDisabled(noPico);
              if(ui::iconButton(
                  ui::Icon::ICON_PLUG_CONNECT,
                  "Connect Pico",
                  ImVec2(-FLT_MIN, bh),
                  ui::Tint::TINT_GOOD
              ))
              {
                  connectPico();
              }
              ImGui::EndDisabled();

              if(noPico
                 && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
              {
                  ImGui::SetTooltip(
                      "No Pico found.\n\n" "Plug the board in - it is detected automatically, and this " "button enables itself\nwithin about a second. Nothing " "needs pressing first.\n\n" "If it stays grayed: the board is running a sketch that " "never called serialOpen(),\nso it never enumerated over " "USB. Hold BOOTSEL while plugging the cable in\nand flash " "it again."
                  );
              }
          }

          ImGui::TableNextRow();

          ImGui::TableNextColumn();
          {
              const Bool havePort = !picoLink.port().empty() || picoIndex >= 0;
              ImGui::BeginDisabled(!havePort);
              if(ui::iconButton(
                  ui::Icon::ICON_REBOOT,
                  "Reboot to BOOTSEL...",
                  ImVec2(-FLT_MIN, bh),
                  ui::Tint::TINT_WARN
              ))
              {
                  openBootsel = true;
              }
              ImGui::EndDisabled();
          }

          ImGui::TableNextColumn();
          ImGui::BeginDisabled(busy || backupBuf[0] == '\0');
          if(ui::iconButton(ui::Icon::ICON_BACKUP, "Back up board flash", ImVec2(-FLT_MIN, bh)))
          {
              startBackup();
          }
          ImGui::EndDisabled();

          ImGui::EndTable();
      }

      // The result of the last BOOTSEL touch, next to the button that asks for
      // one, so a failure is not silent. Only once one has been attempted.
      if(bootselDone)
      {
          colored(
              bootselOk ? ui::sem::GOOD : ui::sem::BAD,
              bootselOk ? "BOOTSEL touch sent" : "BOOTSEL touch failed"
          );
      }
  }

  Void sectionSystem()
  {
      // The board's own report drives the rows below, so the ask lives with the
      // display. pollBoardStatus() rate-limits itself and gives up on old firmware.
      pollBoardStatus();

      drawSubsystems();
      ImGui::Spacing();
      drawQuickActions();

      // Under the quick actions rather than among them: this is a two-step thing
      // with a text field in it, not a button you hit without looking.
      drawWifiLink(ImGui::GetFrameHeight() * 1.2f);
  }

  // ==================================================================== sensors
  // The fused world view. One rotating scanner today; the ToF ring, encoder and
  // IMU are named so wiring one later fills in a row.

  Void drawConnection()
  {
      const Bool busy = isBusy();

      ImGui::BeginDisabled(busy);
      ImGui::SetNextItemWidth(-FLT_MIN);
      if(portItems.empty())
      {
          ImGui::TextDisabled("No serial ports found");
      }
      else
      {
          ui::combo("##port", &portIndex, portItems.data(), static_cast<Int32>(portItems.size()));
      }

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
                      "Not a rate the C1 has.\n" "\n" "The C1M1 datasheet lists 460800 and nothing else - no\n" "minimum, no maximum, no alternative. Other RPLIDAR models\n" "do run at 115200, which is where the expectation comes\n" "from, and it is why this is grayed rather than absent.\n" "\n" "Selecting it would open the port and then receive nothing,\n" "which looks like a dead sensor rather than a wrong number."
                  );
              }
          }
          ImGui::EndCombo();
      }
      ImGui::EndDisabled();

      const Float32 bh = ImGui::GetFrameHeight() * 1.2f;
      if(busy)
      {
          if(ui::iconButton(
              ui::Icon::ICON_PLUG_DISCONNECT,
              "Disconnect",
              ImVec2(-FLT_MIN, bh),
              ui::Tint::TINT_WARN
          ))
          {
              lidarUserDisconnected = true;
              lidarSource.stop();
          }
      }
      else
      {
          ImGui::BeginDisabled(lidarPorts.empty());
          if(ui::iconButton(
              ui::Icon::ICON_PLUG_CONNECT,
              "Connect",
              ImVec2(-FLT_MIN, bh),
              ui::Tint::TINT_GOOD
          ))
          {
              connect();
          }
          ImGui::EndDisabled();
      }

      const Str        err = lidarSource.error();
      const LidarState ls = lidarSource.state();

      if(!err.empty() && ls == LidarState::LIDAR_STATE_ERROR)
      {
          ImGui::PushStyleColor(ImGuiCol_Text, ui::sem::BAD);
          ImGui::TextWrapped("%s", err.c_str());
          ImGui::PopStyleColor();
      }
      else if(!err.empty() && ls == LidarState::LIDAR_STATE_UNPLUGGED)
      {
          // Muted, not red: a pulled cable is deliberate and already known. The
          // line confirms the app noticed, it does not raise an alarm.
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
  // is doing. Unwired sensors are disabled rather than hidden.
  Void sensorRow(Int32 index, Bool wired, Bool* vis, const Char* name, ImU32 col, const Char* state)
  {
      static Bool never = false;

      ImGui::PushID(index);
      ImGui::BeginDisabled(!wired);

      ui::checkbox("##vis", wired ? vis : &never);
      ImGui::SameLine();

      // A list row, not a control: only the selected one is drawn, marked down its
      // left edge. Outlining every row would read as a column of text fields.
      const Bool selected = (selSensor == index);
      const ImVec2 rowSz(ImGui::GetContentRegionAvail().x * 0.52f, 0.0f);

      ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.0f, 0.5f));
      const Bool hit = ui::segmentedButton(name, selected, rowSz, ui::Mark::MARK_LEFT_BAR);
      ImGui::PopStyleVar();
      if(hit && wired)
      {
          selSensor = index;
      }

      ImGui::SameLine();
      colored(col, "%s", state);

      ImGui::EndDisabled();
      ImGui::PopID();
  }

  Void drawSensorList()
  {

      Array<Char, 48> st;
      if(lidarSource.state() == LidarState::LIDAR_STATE_SCANNING)
      {
          std::snprintf(st.data(), st.size(), "%.1f Hz", haveFrame ? latestFrame.hz : 0.0f);
      }
      else
      {
          std::snprintf(st.data(), st.size(), "%s", lidarStateText());
      }

      sensorRow(0, true, &layerLidar, "RPLIDAR C1", lidarStateColor(), st.data());

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
      Array<Char, 24> hz = {'-', '-'};
      Array<Char, 24> pts = {'-', '-'};
      Array<Char, 24> valid = {'-', '-'};
      Array<Char, 24> nearS = {'-', '-'};
      Array<Char, 24> meanS = {'-', '-'};
      Array<Char, 24> maxS = {'-', '-'};

      if(haveFrame)
      {
          std::snprintf(hz.data(),  hz.size(),  "%.1f", latestFrame.hz);
          std::snprintf(
              pts.data(),
              sizeof(pts.data()),
              "%d",
              static_cast<Int32>(latestFrame.points.size())
          );

          const Float64 frac = latestFrame.points.empty()
                            ? 0.0 : static_cast<Float64>(nInspec) / static_cast<Float64>(latestFrame.points.size());
          std::snprintf(
              valid.data(),
              sizeof(valid.data()),
              "%d%%",
              static_cast<Int32>((frac * 100.0 + 0.5))
          );

          if(radarView.hasNearest)
          {
              std::snprintf(nearS.data(), nearS.size(), "%.2f", radarView.nearestMm / 1000.0f);
          }
          std::snprintf(meanS.data(), sizeof(meanS.data()), "%.2f", meanMm / 1000.0f);
          std::snprintf(maxS.data(),  sizeof(maxS.data()),  "%.2f", maxRangeMm / 1000.0f);
      }

      if(ImGui::BeginTable("stats", 3, ImGuiTableFlags_SizingStretchSame))
      {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          statCell(hz.data(),     "Hz");
          ImGui::TableNextColumn();
          statCell(pts.data(),    "pts/rev");
          ImGui::TableNextColumn();
          statCell(valid.data(),  "in-spec");
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "Returns inside 0.05-12 m, as a share of the revolution.\n" "\n" "12 m is the WHITE figure. The C1M1 datasheet gives two ranges:\n" "0.05-12 m against a 70%% reflective target and only 0.05-6 m\n" "against a 10%% one. A dark surface at 8 m is outside what the\n" "sensor is specified to see, and it is counted here as in-spec\n" "regardless - nothing in the stream says how reflective the\n" "thing it hit was.\n" "\n" "So on a dark scene this reads high. It is a coverage figure,\n" "not a guarantee."
              );
          }

          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          statCell(nearS.data(), "near (m)");
          ImGui::TableNextColumn();
          statCell(meanS.data(), "mean (m)");
          ImGui::TableNextColumn();
          statCell(maxS.data(),  "max (m)");
          ImGui::EndTable();
      }

      Array<Char, 48> overlay;
      std::snprintf(
          overlay.data(),
          overlay.size(),
          "rotation %.1f Hz",
          haveFrame ? latestFrame.hz : 0.0f
      );
      ImGui::PlotLines(
          "##hz",
          hzHist.data(),
          hzCount,
          0,
          overlay.data(),
          0.0f,
          15.0f,
          ImVec2(-FLT_MIN, 46.0f * uiDpiScale)
      );

      ImGui::TextDisabled("Clearance by sector (m, capped %.1f)", CLEARANCE_CAP_M);
      ImGui::PlotHistogram(
          "##sectors",
          sectorM.data(),
          SECTORS,
          0,
          nullptr,
          0.0f,
          CLEARANCE_CAP_M,
          ImVec2(-FLT_MIN, 58.0f * uiDpiScale)
      );
  }

  Void tabSignal()
  {
      // Return classification. These four sum to the revolution's sample count,
      // which is what makes the in-spec percentage interpretable.
      const Int32 total = haveFrame ? static_cast<Int32>(latestFrame.points.size()) : 0;

      ImGui::TextDisabled("Returns this revolution (%d samples)", total);

      if(ImGui::BeginTable("returns", 3, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_RowBg))
      {
          auto cell = [&](const Char* label, Int32 n, ImU32 col)
          {
              ImGui::TableNextColumn();
              ImGui::PushStyleColor(ImGuiCol_Text, col);
              ImGui::Text("%d", n);
              ImGui::PopStyleColor();
              ScopedFont sf(ui::fonts.small);
              ImGui::TextDisabled("%s", label);
              if(total > 0)
              {
                  ImGui::TextDisabled("%.0f%%", 100.0 * n / total);
              }
          };

          ImGui::TableNextRow();
          cell("in spec",   nInspec,   ui::sem::GOOD);
          cell("no return", nNoreturn, ui::sem::MUTED);
          cell("< 50 mm",   nToonear,  ui::sem::WARN);

          ImGui::EndTable();
      }

      if(nToofar > 0)
      {
          ImGui::TextDisabled("beyond 12 m: %d", nToofar);
      }
      else
      {
          ImGui::TextDisabled("beyond 12 m: none");
      }

      ImGui::Spacing();

      // Signal quality is reported per measurement by the device and is otherwise
      // completely invisible - it is the main clue when returns start dropping.
      ImGui::TextDisabled("Signal quality (mean %.1f, range %d-%d of 63)", qMean, qMin, qMax);
      ImGui::PlotHistogram(
          "##qhist",
          qHist.data(),
          QUALITY_BUCKETS,
          0,
          nullptr,
          0.0f,
          qHistMax,
          ImVec2(-FLT_MIN, 62.0f * uiDpiScale)
      );
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
          keyValue("Sample period", si.usPerSample > 0 ? "%.2f us" : "--", si.usPerSample);
          keyValue(
              "Sample rate",
              si.usPerSample > 0 ? "%.2f kHz" : "--",
              si.usPerSample > 0 ? 1000.0f / si.usPerSample : 0.0f
          );
          keyValue("Mode max range", si.maxDistanceM > 0 ? "%.1f m" : "--", si.maxDistanceM);
          keyValue("Angular res", angRes > 0 ? "%.2f deg" : "--", angRes);
          keyValue("Coverage", "%.0f%% of 360 deg", coverageDeg * 100.0f);
          ImGui::EndTable();
      }

      ImGui::Spacing();
      ImGui::TextDisabled("Range distribution (0 - 12 m, 0.5 m bins)");
      ImGui::PlotHistogram(
          "##dhist",
          distHist.data(),
          DIST_BUCKETS,
          0,
          nullptr,
          0.0f,
          distHistMax,
          ImVec2(-FLT_MIN, 70.0f * uiDpiScale)
      );
  }

  Void tabDevice()
  {
      const LidarDeviceInfo info = lidarSource.info();
      const LidarStats      st = lidarSource.stats();
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
          keyValue(
              "Avg rate",
              st.uptimeS > 1.0 ? "%.2f Hz" : "--",
              st.uptimeS > 1.0 ? static_cast<Float64>(st.frames) / st.uptimeS : 0.0
          );

          // The time-of-flight core drifts with die temperature, so ranges are
           // not trustworthy for the first two minutes. Measured on this unit.
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextDisabled("Pre-heat");
          ImGui::TableNextColumn();
          if(st.uptimeS <= 0.0)
          {
              ImGui::TextUnformatted("--");
          }
          else if(st.uptimeS < 120.0)
          {
              colored(ui::sem::WARN, "%.0f / 120 s", st.uptimeS);
          }
          else
          {
              colored(ui::sem::GOOD, "done");
          }

          ImGui::EndTable();
      }
  }

  // Car or bare sensor, at the origin. Shared by both dimensions: a claim about
  // the machine, not a projection. A control because SENSOR is honest today - a
  // 430 mm shell round a 56 mm puck is a statement about the future.
  Void drawEgoSwitch()
  {
      // A SCOPE, because ImGui identifies a widget by its LABEL and this bar has
      // two buttons called "Car" - without one they are literally the same widget.
      ImGui::PushID("ego-switch");

      const ImGuiStyle& sty = ImGui::GetStyle();
      const Float32 w = ImGui::CalcTextSize("Sensor").x + sty.FramePadding.x * 2.0f;

      ImGui::AlignTextToFramePadding();
      ImGui::TextDisabled("Show");
      ImGui::SameLine();

      const Bool isCar = (radarView.ego == scene3d::EgoView::EGO_VIEW_CAR);

      if(ui::segmentedIconButton(ui::Icon::ICON_SCENE_FIT, "Car", isCar, ImVec2(w, 0.0f)))
      {
          radarView.ego = scene3d::EgoView::EGO_VIEW_CAR;
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "The TT-02, to scale: 442 x 186 mm.\n" "What will be there once it is built."
          );
      }

      ImGui::SameLine();
      if(ui::segmentedIconButton(ui::Icon::ICON_RADAR, "Sensor", !isCar, ImVec2(w, 0.0f)))
      {
          radarView.ego = scene3d::EgoView::EGO_VIEW_SENSOR;
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "The RPLIDAR C1 alone, to scale: 55.6 x 55.6 x 41.3 mm.\n" "What is actually on the desk."
          );
      }

      ImGui::PopID();
  }

  Void drawControlBar()
  {
      // The row belongs to the dimension: Range, Trail, Labels and Nearest are all
      // properties of a top-down projection and mean nothing while orbiting.
      if(centralView == 1)
      {
          ImGui::AlignTextToFramePadding();
          ImGui::TextDisabled("Drag to orbit  |  right-drag to pan  |  wheel to zoom");

          ImGui::SameLine();
          ImGui::TextUnformatted("|");
          ImGui::SameLine();

          if(ui::iconButton(ui::Icon::ICON_RESET_VIEW, "Reset camera"))
          {
              radarView.cam = scene3d::Camera{};
          }

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

          if(ui::segmentedIconButton(
              ui::Icon::ICON_SCENE_FIT,
              "Car",
              radarView.cam.lockToCar,
              ImVec2(lockW, 0.0f)
          ))
          {
              radarView.cam.lockToCar = true;
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "The car stays centered. Orbit and zoom still work; " "panning does not, because that is what locked means."
              );
          }

          ImGui::SameLine();
          if(ui::segmentedIconButton(
              ui::Icon::ICON_DIM_3D,
              "World",
              !radarView.cam.lockToCar,
              ImVec2(lockW, 0.0f)
          ))
          {
              radarView.cam.lockToCar = false;
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "Free camera. Right-drag pans anywhere and the car " "can leave the frame."
              );
          }

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
              radarView.cam.yaw = 0.0f;
          }
          return;
      }

      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted("Range");
      ImGui::SameLine();

      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.0f);
      if(ui::combo("##range", &rangeIndex, RANGE_ITEMS.data(), RANGE_COUNT))
      {
          applyRange();
      }

      ImGui::SameLine();
      ImGui::TextUnformatted("|");
      ImGui::SameLine();

      ui::checkbox("Grid", &radarView.showGrid);
      ImGui::SameLine();
      ui::checkbox("Trail", &radarView.showTrail);
      ImGui::SameLine();
      ui::checkbox("Labels", &radarView.showLabels);
      ImGui::SameLine();
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

      // The readouts below describe the SELECTED sensor, not the app - saying so
      // keeps the four tab names from reading as global.
      ImGui::SeparatorText("Telemetry - RPLIDAR C1");

      if(ImGui::BeginTabBar("##lidartabs"))
      {
          auto sub = [](Int32 which)
          {
              return (forceSub == which && forceTabFrames > 0)
                   ? ImGuiTabItemFlags_SetSelected : 0;
          };

          {
              Array<Char, 40> lb;
              const Bool t = ImGui::BeginTabItem(iconTabLabel(lb.data(), lb.size(), "Live"),
                                                 nullptr, sub(0));
              tabIcon(ui::Icon::ICON_LIVE);
              if(t)
              {
                  tabLive();
                  ImGui::EndTabItem();
              }
          }
          {
              Array<Char, 40> lb;
              const Bool t = ImGui::BeginTabItem(iconTabLabel(lb.data(), lb.size(), "Signal"),
                                                 nullptr, sub(1));
              tabIcon(ui::Icon::ICON_SIGNAL);
              if(t)
              {
                  tabSignal();
                  ImGui::EndTabItem();
              }
          }
          {
              Array<Char, 40> lb;
              const Bool t = ImGui::BeginTabItem(iconTabLabel(lb.data(), lb.size(), "Scan"),
                                                 nullptr, sub(2));
              tabIcon(ui::Icon::ICON_SCAN);
              if(t)
              {
                  tabScan();
                  ImGui::EndTabItem();
              }
          }
          {
              Array<Char, 40> lb;
              const Bool t = ImGui::BeginTabItem(iconTabLabel(lb.data(), lb.size(), "Device"),
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

  // The permanent left region: the map, with the control bar under it, both sized
  // by the caller. --view <map|3d|record|code|range|drive|sound|flash|reference>
  // preselects a central tab at startup, held a few frames because a tab bar only
  // honors SetSelected once it has laid its items out.
  Int32 forceView = -1;   // -1 none, 0 = 2D, 1 = 3D, 2+ board index + 2
  Int32 forceViewFrames = 0;

  Int32 modeToggleRows();

  // True when the mode strip has to scroll sideways at this content width. The
  // layout and the strip both ask, so they cannot disagree about the bar height.
  Bool  modeToggleScrolls(Float32 contentW);

  // Rows of controls the given view puts under itself. Zero is a legitimate
  // answer and means the view gets no bottom bar at all - not an empty one.
  [[nodiscard]] Int32 centralControlRows(Int32 view) noexcept
  {
      // Transport, then playback. The recorder does not get the render-mode strip:
      // it is a Points view on purpose - see drawRecorderControls.
      if(view == 2)
      {
          return 2;
      }

      // Code: files on one row, build/flash on the next.
      if(view == 3)
      {
          return 2;
      }

      if(view == 0 || view == 1)
      {
          return modeToggleRows() + 1;   // render modes, then the map controls
      }

      // The board views have nothing to configure yet. One that grows a control
      // declares its rows here and draws them in drawCentralControls().
      return 0;
  }

  // Height of that bar, or 0 when the view has no controls. `contentW` is needed
  // because the map's mode strip grows a scrollbar once the cells get too narrow.
  [[nodiscard]] Float32 centralControlHeight(Int32 view, Float32 contentW)
  {
      const Int32 rows = centralControlRows(view);
      if(rows <= 0)
      {
          return 0.0f;
      }

      const ImGuiStyle& sty = ImGui::GetStyle();
      const Float32 n = static_cast<Float32>(rows);
      Float32 h = ImGui::GetFrameHeight() * n
                + sty.ItemSpacing.y * (n - 1.0f)
                + sty.WindowPadding.y * 2.0f;

      if((view == 0 || view == 1)
         && modeToggleScrolls(contentW - sty.WindowPadding.x * 2.0f))
      {
          h += sty.ScrollbarSize;
      }

      return h;
  }

  // Beyond this many render modes the toggle wraps to a second row rather than
  // squeezing the cells until the labels clip.
  constexpr Int32 MODE_TOGGLE_MAX_PER_ROW = 5;

  // The render-mode toggle: a segmented row rather than a combo, because switching
  // is the point. Nine buttons flat, four in the scene; see modeToggleRows().
  Int32 activeModeCount()
  {
      // Keyed on centralView, not radarView.is3D: the bar's HEIGHT is computed
      // before the tab bar runs and its CONTENTS after; only this is stable.
      return (centralView == 1)
           ? static_cast<Int32>(scene3d::SceneMode::SCENE_MODE_COUNT)
           : static_cast<Int32>(MapMode::MAP_MODE_COUNT);
  }

  Int32 modeToggleRows()
  {
      return (activeModeCount() > MODE_TOGGLE_MAX_PER_ROW) ? 2 : 1;
  }

  // The tooltip that makes a mode legible - "Density", "Sweep" and "Validity" do
  // not say what they mean to anyone who has not read the source. One body for
  // both mode families. Plain IsItemHovered, NOT ImGuiHoveredFlags_DelayNormal:
  // that flag implies Stationary and never fires for a SetCursorPos cursor.
  Void modeTooltipBody(const Char* name, const Char* what, const Char* read)
  {
      ImGui::BeginTooltip();
      ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);

      ImGui::TextUnformatted(name);
      if(ui::fonts.small != nullptr)
      {
          ImGui::PushFont(ui::fonts.small, ui::fonts.small->LegacySize);
      }

      ImGui::Spacing();
      ImGui::TextUnformatted(what);
      ImGui::Spacing();
      ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED));
      ImGui::TextUnformatted(read);
      ImGui::PopStyleColor();

      if(ui::fonts.small != nullptr)
      {
          ImGui::PopFont();
      }
      ImGui::PopTextWrapPos();
      ImGui::EndTooltip();
  }

  Void sceneTooltip(scene3d::SceneMode m)
  {
      if(!ImGui::IsItemHovered())
      {
          return;
      }
      const scene3d::SceneModeInfo& i = scene3d::sceneModeInfo(m);
      modeTooltipBody(i.name, i.what, i.read);
  }

  Void modeTooltip(MapMode m)
  {
      if(!ImGui::IsItemHovered())
      {
          return;
      }

      const MapModeInfo& info = mapModeInfo(m);

      ImGui::BeginTooltip();
      ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);

      ImGui::TextUnformatted(info.name);
      if(ui::fonts.small != nullptr)
      {
          ImGui::PushFont(ui::fonts.small, ui::fonts.small->LegacySize);
      }

      ImGui::Spacing();
      ImGui::TextUnformatted(info.what);
      ImGui::Spacing();
      ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED));
      ImGui::TextUnformatted(info.read);
      ImGui::PopStyleColor();

      if(ui::fonts.small != nullptr)
      {
          ImGui::PopFont();
      }
      ImGui::PopTextWrapPos();
      ImGui::EndTooltip();
  }

  Bool modeToggleScrolls(Float32 contentW)
  {
      const Int32   n = activeModeCount();
      const Int32   topN = (n + modeToggleRows() - 1) / modeToggleRows();
      const Float32 gap = ImGui::GetStyle().ItemSpacing.x;
      const Float32 w = (contentW - gap * static_cast<Float32>(topN - 1))
                         / static_cast<Float32>(topN);
      return w < (118.0f * uiDpiScale);
  }

  Void drawModeToggle()
  {
      const ImGuiStyle& sty = ImGui::GetStyle();
      const Int32   n = activeModeCount();
      const Float32 gap = sty.ItemSpacing.x;
      const Int32   rows = modeToggleRows();

      // Rows follow the mode count: the flat map's nine wrap to two, the scene's
      // four sit on one. Below a legible width the strip scrolls sideways.
      const Float32 avail = ImGui::GetContentRegionAvail().x;
      const Int32   topN = (n + rows - 1) / rows;
      const Bool    scrolls = modeToggleScrolls(avail);
      const Float32 w = scrolls
                             ? 118.0f * uiDpiScale
                             : (avail - gap * static_cast<Float32>(topN - 1))
                                   / static_cast<Float32>(topN);

      const Float32 stripH = ImGui::GetFrameHeight() * static_cast<Float32>(rows)
                           + sty.ItemSpacing.y * static_cast<Float32>(rows - 1)
                           + (scrolls ? sty.ScrollbarSize : 0.0f);

      ImGui::BeginChild(
          "##modes",
          ImVec2(0.0f, stripH),
          ImGuiChildFlags_None,
          scrolls ? ImGuiWindowFlags_HorizontalScrollbar : ImGuiWindowFlags_NoScrollbar
      );

      const Float32 modeX = ImGui::GetCursorPosX();

      for(Int32 i = 0; i < n; ++i)
      {
          const Bool  second = (i >= topN);
          const Int32 rowN = second ? (n - topN) : topN;
          const Int32 col = second ? (i - topN) : i;

          // Cells keep a uniform width WITHIN a row so each row reads as one
          // strip; the rows do not have to match each other.
          const Float32 cellW = scrolls
                              ? w
                              : (avail - gap * static_cast<Float32>(rowN - 1))
                                    / static_cast<Float32>(rowN);

          if(col)
          {
              ImGui::SameLine(0.0f, gap);
          }
          else if(second)
          {
              ImGui::SetCursorPosX(modeX);
          }

          ImGui::PushID(i);

          if(centralView == 1)
          {
              const scene3d::SceneMode m =
                  static_cast<scene3d::SceneMode>(i);
              const ui::Icon ic = static_cast<ui::Icon>(
                  static_cast<Int32>(ui::Icon::ICON_SCENE_CLOUD) + i);

              if(ui::segmentedIconButton(
                  ic,
                  scene3d::sceneModeName(m),
                  radarView.scene == m,
                  ImVec2(cellW, 0.0f)
              ))
              {
                  radarView.scene = m;
              }
              sceneTooltip(m);
          }
          else
          {
              const MapMode m = static_cast<MapMode>(i);

              // The mode icons live in a contiguous block that mirrors MapMode, so
              // the mapping is arithmetic rather than a second table to maintain.
              const ui::Icon ic = static_cast<ui::Icon>(
                  static_cast<Int32>(ui::Icon::ICON_MODE_POINTS) + i);

              if(ui::segmentedIconButton(
                  ic,
                  mapModeName(m),
                  radarView.mode == m,
                  ImVec2(cellW, 0.0f)
              ))
              {
                  radarView.mode = m;
              }
              modeTooltip(m);
          }

          ImGui::PopID();
      }

      ImGui::EndChild();
  }

  // The bottom bar for whichever view is on screen. Must agree with
  // centralControlRows() about how many rows it draws, or the bar clips. The
  // recorder view is deliberately Points and nothing else, and its one HUD line
  // says which frame you are looking at.
  Void drawRecorderHud(const ImVec2& p0, const ImVec2& size)
  {
      ImDrawList* dl = ImGui::GetWindowDrawList();
      if(dl == nullptr)
      {
          return;
      }

      const Float32 pad = 8.0f * uiDpiScale;
      ImFont* f = ui::fonts.small ? ui::fonts.small : ImGui::GetFont();
      const Float32 fs = f->LegacySize > 0.0f ? f->LegacySize : ImGui::GetFontSize();

      Array<Char, 192> line;
      ImU32 col = ui::sem::MUTED;

      if(recArmed)
      {
          std::snprintf(line.data(), line.size(), "RECORDING - %zu revolutions", recording.count());
          col = ui::sem::BAD;
      }
      else if(!recording.empty())
      {
          std::snprintf(
              line.data(),
              line.size(),
              "%s - revolution %zu of %zu - %.2f s",
              recPlaying ? "PLAYING" : "PAUSED",
              recIndex + 1u,
              recording.count(),
              recPlayS
          );
          col = recPlaying ? ui::sem::GOOD : ui::sem::WARN;
      }
      else
      {
          std::snprintf(
              line.data(),
              line.size(),
              "%s",
              (lidarSource.state() == LidarState::LIDAR_STATE_SCANNING) ? "live - press Record to capture" : "no lidar; connect one to record"
          );
      }

      dl->AddText(f, fs, ImVec2(p0.x + pad, p0.y + pad), col, line.data());
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
          {
              recArmed = false;
          }
      }
      else
      {
          if(ui::iconButton(ui::Icon::ICON_RECORD, "Record", ImVec2(0, 0), ui::Tint::TINT_BAD))
          {
              // A new take replaces the old one: silently appending two runs into
              // one file would be worse than losing the first.
              recording.clear();
              recIndex = 0;
              recPlayS = 0.0;
              recPlaying = false;
              recArmed = true;
              recStartS = ImGui::GetTime();
              recStatus.clear();
          }
      }
      ImGui::EndDisabled();
      if(!live && !recArmed && ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("The lidar is not scanning. Connect it first.");
      }

      ImGui::SameLine();
      ImGui::BeginDisabled(recording.empty() || recArmed);
      if(ui::iconButton(ui::Icon::ICON_CLEAR, "Clear"))
      {
          recording.clear();
          recIndex = 0;
          recPlayS = 0.0;
          recPlaying = false;
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
          {
              items.push_back(f.c_str());
          }
          ui::combo("##recfile", &recFileIndex, items.data(), static_cast<Int32>(items.size()));
      }

      ImGui::SameLine();
      ImGui::BeginDisabled(recFiles.empty() || recArmed);
      if(ui::iconButton(ui::Icon::ICON_OPEN, "Load"))
      {
          Str err;
          const Str path = rec::dir() + "\\" + recFiles[static_cast<Size>(recFileIndex)];
          if(recording.load(path, err))
          {
              recIndex = 0;
              recPlayS = 0.0;
              recPlaying = false;
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
      {
          refreshRecordings();
      }

      // ---- row 2: playback and readout ------------------------------------
      ImGui::BeginDisabled(recording.empty() || recArmed);

      if(recPlaying)
      {
          if(ui::iconButton(ui::Icon::ICON_PAUSE, "Pause"))
          {
              recPlaying = false;
          }
      }
      else
      {
          if(ui::iconButton(ui::Icon::ICON_PLAY, "Play", ImVec2(0, 0), ui::Tint::TINT_GOOD))
          {
              // Replaying from the end restarts, rather than sitting there doing
              // nothing and looking broken.
              if(recPlayS >= recording.durationS() - 1e-3)
              {
                  recPlayS = 0.0;
              }
              recPlaying = true;
          }
      }

      ImGui::SameLine();
      const Size n = recording.count();
      Int32 idx = static_cast<Int32>(recIndex);

      ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 22.0f);
      if(ImGui::SliderInt("##scrub", &idx, 0, (n > 0u) ? static_cast<Int32>(n - 1u) : 0, "rev %d"))
      {
          recIndex = static_cast<Size>(idx);
          recPlayS = recording.at(recIndex).tS;
          recPlaying = false;      // dragging the scrub means you want to look
          recPendingSeek = true;
      }
      ImGui::EndDisabled();

      ImGui::SameLine();
      if(recArmed)
      {
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::BAD),
              "REC %zu rev %.1f s %.1f MB",
              recording.count(),
              ImGui::GetTime() - recStartS,
              static_cast<Float64>(recording.pointCount() * 8u) / (1024.0 * 1024.0)
          );
      }
      else if(!recording.empty())
      {
          ImGui::Text(
              "%zu rev %.1f s %.1f MB",
              recording.count(),
              recording.durationS(),
              static_cast<Float64>(recording.pointCount() * 8u) / (1024.0 * 1024.0)
          );
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
              "%s",
              recStatus.c_str()
          );
      }
  }

  // ================================================================= code view

  // Writes the buffer to the file it came from. One write, one file: sketches
  // each have their own target, so the file on screen is the file that compiles.
  Bool saveSketch()
  {
      if(codeName.empty())
      {
          codeName = sketch::makeName();
      }
      if(codePath.empty())
      {
          codePath = sketch::pathOf(codeName);
      }

      const Str text = codeEditor.text();
      Str       err;

      if(!sketch::save(codePath, text, err))
      {
          codeMessage = err;
          return false;
      }

      codeEditor.clearDirty();
      codeMessage = "saved " + codeName;

      // Our OWN write must not look like somebody else's, or the watcher below
      // would offer to reload the buffer we just wrote.
      codeFileStamp = sketch::stamp(codePath);

      // A save is the moment a #define you just wrote becomes real.
      rebuildMacroIndex();

      ui::setNote(codeView, "saved " + codeName, ImGui::GetTime());
      LOG_INFO("code", "saved %s", codePath.c_str());
      return true;
  }

  // Opens `path` under the display name `name`, saving whatever is open first.
  // Switching files must never be the thing that loses work.
  Void openCodeFile(const Str& path, const Str& name)
  {
      if(codeEditor.dirty() && !codePath.empty())
      {
          saveSketch();
      }

      // Marks the view as loaded even though it may never have been drawn: its
      // first draw otherwise lazy-inits and replaces whatever was just opened.
      codeLoaded = true;

      // BEFORE codePath moves, or the spot is filed under the file being opened.
      rememberCodeSpot();

      codePath = path;
      codeName = name;
      codeEditor.setText(sketch::load(path));
      codeView.diags.clear();      // a new file has not been compiled yet

      // Back where you left it, if you have been here this session. setText()
      // above resets the caret to 0,0, so this has to come after it.
      const auto spot = codeSpots.find(path);
      if(spot != codeSpots.end())
      {
          codeEditor.setCursor(spot->second.line, spot->second.col);
          codeView.scrollY = spot->second.scrollY;

          // The view already agrees with the caret, so do NOT let the next draw
          // scroll to it - followCaret would snap a restored mid-file position
          // to wherever the caret happens to sit in the window.
          codeView.followCaret = false;
      }
      else
      {
          codeView.scrollY = 0.0f;
      }

      // The PAGE's view too, not just the editor's: a pan carried from the last
      // document can open this one entirely off-panel.
      docView = refdoc::View();
      codeDiags.clear();

      // AND the LSP's. This was the one set that survived an open: codeDiags is
      // cleared above and codeLintDiags is recomputed below, but clangd's were
      // carried straight back in by refreshCodeDiags() a few lines down - so the
      // previous file's underlines reappeared at the previous file's line
      // numbers, on top of a buffer that had never been parsed.
      codeLspDiags.clear();

      // Linted immediately rather than half a second later: opening a file and
      // seeing nothing, then seeing marks appear, reads as a glitch.
      codeLintDiags = lint::check(codeEditor.text(), lint::langOf(codePath));
      codeLintIn = 30;
      refreshCodeDiags();
      rebuildMacroIndex();
      codeFileStamp = sketch::stamp(path);
      codeMessage = "opened " + name;
      ui::setNote(codeView, "opened " + name, ImGui::GetTime());
  }

  // Defined down with sidebarSplitter() so the two drag handles stay the same;
  // declared here because the Code tab is laid out long before that point.
  Void codeTreeSplitter(const ImVec2& at, Float32 h, Float32 thickness);

  // The file tree down the left of the Code view. Two roots: a sketch is scratch
  // space that Build & Flash overwrites, firmware/src is the real thing.
  Void drawCodeTree(Float32 w, Float32 h)
  {
      ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(0x28, 0x28, 0x28, 0xFF));
      ImGui::BeginChild("##codetree", ImVec2(w, h), ImGuiChildFlags_None);

      // ---- collapsed: a strip with the way back, and nothing else -----------
      // A collapsed panel that leaves no handle is a panel the user has lost.
      if(codeTreeCollapsed)
      {
          if(ImGui::ArrowButton("##treeopen", ImGuiDir_Right))
          {
              codeTreeCollapsed = false;
              panelLayoutDirty = true;
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip("Show the file tree");
          }

          ImGui::EndChild();
          ImGui::PopStyleColor();
          return;
      }

      if(ImGui::ArrowButton("##treeclose", ImGuiDir_Left))
      {
          codeTreeCollapsed = true;
          panelLayoutDirty = true;
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("Hide the file tree");
      }

      ImGui::SameLine();
      ImGui::TextDisabled("Files");
      ImGui::Separator();

      // Re-scanned on a timer, not every frame: it is two directory enumerations,
      // and a file can appear behind our back.
      static Vec<Str> libFiles;
      static Vec<Str> fwFiles;
      static Int32    rescanIn = 0;
      if(rescanIn <= 0)
      {
          libFiles = sketch::list();
          fwFiles = sketch::listFirmware();
          rescanIn = 120;          // ~2 s at 60 fps
      }
      --rescanIn;

      // One row, plus the right-click menu that belongs to it.
      // BeginPopupContextItem scopes the menu to THIS row's ID, so it acts on the
      // file you right-clicked, not the selected one. Destructive entries do not
      // act here: deleting a file while iterating the list that drew it is how a
      // tree crashes. `label` tints the NAME as well as the glyph.
      const auto row = [](const Str& name, const Str& path, Bool sel, ui::Icon ic,
                          Bool deletable, ImU32 label = 0)
      {
          ImGui::PushID(path.c_str());

          // The icon, centered on the LABEL, not hung from the top: SameLine aligns
          // by top edges. Guarded, or an icon TALLER than the text is pushed out.
          const Float32 iconH = ui::iconSize();
          const Float32 textH = ImGui::GetTextLineHeight();
          if(textH > iconH)
          {
              ImGui::SetCursorPosY(ImGui::GetCursorPosY() + ((textH - iconH) * 0.5f));
          }

          ui::icon(ic);
          ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
          if(label != 0)
          {
              ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(label));
          }
          const Bool hit = ImGui::Selectable(name.c_str(), sel);
          if(label != 0)
          {
              ImGui::PopStyleColor();
          }

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

              // firmware/src files are NOT deletable from here: they are the real
              // firmware and are in git.
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

      // A folder glyph, DRAWN rather than sprited: the vendored Fugue subset has no
      // folder. Two shapes, a tab and a body; an open folder leans its lid back.
      const auto folderGlyph = [](Bool open)
      {
          const Float32 sz = ui::iconSize();
          const ImVec2  at = ImGui::GetCursorScreenPos();
          ImDrawList*   dl = ImGui::GetWindowDrawList();

          // Vertically centered on the text line, like ui::icon does.
          const Float32 y0 = at.y + ((ImGui::GetTextLineHeight() - sz) * 0.5f);
          const Float32 x0 = at.x;

          const ImU32 body = open ? IM_COL32(0xD8, 0x9E, 0x3C, 0xFF)
                                  : IM_COL32(0xB0, 0x82, 0x33, 0xFF);
          const ImU32 tab = IM_COL32(0x8A, 0x66, 0x28, 0xFF);

          // The tab, along the top-left.
          dl->AddRectFilled(
              ImVec2(x0 + sz * 0.06f, y0 + sz * 0.18f),
              ImVec2(x0 + sz * 0.46f, y0 + sz * 0.34f),
              tab,
              sz * 0.06f
          );

          // The body. An open folder is drawn a touch shallower so the lid reads
          // as tipped rather than as a different rectangle.
          dl->AddRectFilled(
              ImVec2(x0 + sz * 0.06f, y0 + sz * 0.30f),
              ImVec2(x0 + sz * 0.94f, y0 + sz * (open ? 0.80f : 0.86f)),
              body,
              sz * 0.08f
          );

          ImGui::Dummy(ImVec2(sz, ImGui::GetTextLineHeight()));
      };

      // One node of the firmware tree: a directory and what is directly in it,
      // built from listFirmware()'s relative paths rather than walking the disk.
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

      // ---- there is no sketch library any more ----------------------------
      // Sketches are firmware/sketches/*.cxx - in the tree, in git, each with its
      // own build target. The tree shows the REPOSITORY.

      // ---- firmware, as the folders it actually is -------------------------
      // The layering IS the architecture, so this shows folders, not flat paths.
      {
        FwNode root{ "firmware", {}, {} };
          for(const Str& n : fwFiles)
          {
              fwInsert(fwInsert, root, n);
          }

          // Folders first, then files, each group alphabetical, at every level.
          // listFirmware()'s own order is DEPENDENCY order, right for reading the
          // library and wrong for finding a file. Case-insensitive, because
          // "Makefile" above "app" is an artifact of ASCII.
          const auto fwSort = [](auto&& self, FwNode& node) -> Void
          {
              const auto byName = [](const Str& a, const Str& b)
              {
                  return _stricmp(a.c_str(), b.c_str()) < 0;
              };

              std::sort(
                  node.dirs.begin(),
                  node.dirs.end(),
                  [&byName](const FwNode& a, const FwNode& b) { return byName(a.name, b.name); }
              );
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
                  const Str p = dir + "\\" + rel;

                  // A document is not code and should not look like it. The
                  // header test has to know about .hxx as well as .h.
                  const auto ends = [](const Str& x, const Char* suf)
                  {
                      const Size n = std::strlen(suf);
                      return x.size() > n
                          && _stricmp(x.c_str() + (x.size() - n), suf) == 0;
                  };
                  const Bool doc = refdoc::isDocPath(f);
                  const Bool hdr = ends(f, ".h") || ends(f, ".hxx");

                  // The LEAF name in the tree, the RELATIVE path everywhere else:
                  // two main.c in different folders must not look like one row.
                  if(row(
                      f,
                      p,
                      _stricmp(p.c_str(), codePath.c_str()) == 0,
                      doc ? ui::Icon::ICON_DOC : (
                          hdr ? ui::Icon::ICON_FIRMWARE : ui::Icon::ICON_CODE
                      ),
                      false,
                      doc ? ui::ansi::BRCYAN : 0u
                  ))
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
      // Deleting a file while iterating the list that drew it is how a tree crashes.
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
      {
          return;
      }

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
              codeOp = CodeOp::CODE_OP_BUILDING;
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
      {
          saveSketch();
      }

      // ---- there is no "New" any more --------------------------------------
      // "Create a file in the repository" has a different right answer per folder
      // - a driver is not a document is not a scratch program - and one button
      // cannot have all of them.

      ImGui::SameLine();
      ImGui::TextUnformatted("|");
      ImGui::SameLine();

      // The path, not just the name: the firmware tree is full of repeated leaf
      // names, and which one is open is the whole question.
      ImGui::TextDisabled("%s", codePath.empty() ? "(unsaved)" : codePath.c_str());

      if(codeEditor.dirty())
      {
          ImGui::SameLine();
          ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::WARN), "modified");
      }

      // ---- the firmware reference ----------------------------------------
      // ON THIS ROW rather than the tab strip: the reference documents the code,
      // and row 1 with the file operations, not row 2 with Run and Build.
      {
          const Float32 dw = 92.0f * uiDpiScale;
          ImGui::SameLine();
          ImGui::SetCursorPosX(std::max(
              ImGui::GetCursorPosX(),
              ImGui::GetWindowWidth() - dw - ImGui::GetStyle().WindowPadding.x
          ));

          if(ui::iconButton(ui::Icon::ICON_REFERENCE, "Docs", ImVec2(dw, ImGui::GetFrameHeight())))
          {
              sketch::openDocs();
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "The firmware API reference, in your browser.\n\n" "Generated from firmware/lib, so a signature there " "is the one\nthat compiles. Builds on first use - " "a few seconds - instant after."
              );
          }
      }

      // ---- row 2: the round trip -------------------------------------------
      const Float32 bh = ImGui::GetFrameHeight();

      // A HEADER is not a translation unit, and Run on one is meaningless: it
      // would build some unrelated target, flash it, and report success against a
      // file it never compiled. Test ".hxx" AND ".h" - bibo.hxx does not end in
      // ".h". A .bdoc is not buildable either: it is not code.
      const auto endsWith = [](const Str& p, const Char* suf)
      {
          const Size n = std::strlen(suf);
          return p.size() > n && _stricmp(p.c_str() + (p.size() - n), suf) == 0;
      };
      const Bool onHeader = endsWith(codePath, ".h")
                         || endsWith(codePath, ".hxx")
                         || refdoc::isDocPath(codePath);

      ImGui::BeginDisabled(busy || onHeader);

      const Str target = sketch::targetFor(codePath);

      // "Run", the verb every IDE uses; what it ACTUALLY does - compile, then
      // overwrite the board's flash - is in the tooltip. Amber, not green.
      if(ui::iconButton(
          ui::Icon::ICON_PLAY,
          "Run",
          ImVec2(120.0f * uiDpiScale, bh),
          ui::Tint::TINT_WARN
      ))
      {
          if(saveSketch())
          {
              codeFlashTarget = target;
              codeOp = CodeOp::CODE_OP_BUILDING;
              picoFlash.build(target);
          }
      }

      if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      {
          if(onHeader)
          {
              ImGui::SetTooltip(
                  "%s is a header.\n\n" "Headers are included by other files rather than " "compiled on their own,\nso there is nothing here " "to build. Open a .c file - the arrow beside this " "button\nlists the ones that can be flashed.",
                  codeName.c_str()
              );
          }
          else
          {
              ImGui::SetTooltip(
                  "Build & Flash: %s\n\n" "Compiles %s\nand writes it to the board, " "replacing what is on it.",
                  target.c_str(),
                  codePath.empty() ? "(unsaved)" : codePath.c_str()
              );
          }
      }

      // ---- the split-button arrow ------------------------------------------
      // Run acts on the file that is OPEN; this is one click to see which source
      // is about to be compiled, and to pick another. A built-in picker rather
      // than the OS dialog: the choosable set is small and known.
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
          ImGui::TextDisabled("target %s -> firmware/build/%s.uf2", target.c_str(), target.c_str());

          ImGui::Separator();
          ImGui::TextDisabled("firmware");

          // ONE LIST. Sketches arrive through listFirmware() like every other
          // source. The PATH is the ID: two folders can hold the same filename.
          auto entry = [](ui::Icon ic, const Str& name, const Str& path,
                          const Char* note, Bool enabled)
          {
              ImGui::PushID(path.c_str());
              const Bool open = !codePath.empty()
                             && _stricmp(path.c_str(), codePath.c_str()) == 0;
              const Bool hit = ui::iconMenuItem(ic, name.c_str(), note, enabled, open);
              ImGui::PopID();
              return hit;
          };

          const Str      fwd = sketch::firmwareDir();
          const Vec<Str> fws = sketch::listFirmware();
          for(const Str& n : fws)
          {
              const Bool hdr = endsWith(n, ".h") || endsWith(n, ".hxx");
              const Bool doc = refdoc::isDocPath(n);
              const Str  p = fwd + "\\" + n;

              // Which image this file ends up in, said out loud: it is what Build
              // & Flash acts on and it is not guessable from a filename.
              const Str tgt = (hdr || doc) ? Str() : sketch::targetFor(p);

              // A header is not a translation unit. Listed so this matches the
              // tree, disabled so it cannot be chosen and then quietly do nothing.
              if(entry(
                  hdr ? ui::Icon::ICON_FIRMWARE : (doc ? ui::Icon::ICON_DOC : ui::Icon::ICON_CODE),
                  n,
                  p,
                  hdr ? "header" : (tgt.empty() ? nullptr : tgt.c_str()),
                  !hdr
              ))
              {
                  openCodeFile(p, n);
              }
          }

          ImGui::EndPopup();
      }

      ImGui::SameLine();
      if(ui::iconButton(ui::Icon::ICON_BUILD, "Build", ImVec2(130.0f * uiDpiScale, bh)))
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
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
              "%s...",
              picoFlash.currentOp().c_str()
          );
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
              "%s",
              codeMessage.c_str()
          );
      }
      else
      {
          ImGui::TextDisabled(
              "i insert - esc normal - :w save - / find - " "ciw change word - . repeat"
          );
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

  // The central region. The map is one view among several, and still the default.
  // ONE view's content, at the current cursor, filling w x h. Shared by both
  // layouts, so a tab and a floating panel showing "2D" are the same picture.
  // Views 0-3 are the fixed ones, then the rest in tab order. Named constants
  // because the indices have moved before; drawTabbedViews clamps out of range.
  constexpr Int32 RANGE_VIEW = 4;
  constexpr Int32 DRIVE_VIEW = RANGE_VIEW + 1;
  constexpr Int32 CUE_VIEW = DRIVE_VIEW + 1;

  // TEMPORARY. Goes when sound moves into the cue system, and it is LAST in the
  // order so removing it cannot renumber anything else.
  constexpr Int32 SOUND_VIEW = CUE_VIEW + 1;

  // The firmware catalog. APPENDED rather than slotted in beside the other board
  // views: a settings file holding a view index would open on a different tab.
  constexpr Int32 FLASH_VIEW = SOUND_VIEW + 1;
  constexpr Int32 VIEW_COUNT = FLASH_VIEW + 1;

  // ================================================ the cue board ==
  // A button per cue, and the grid comes FROM THE BOARD: CUE LIST is asked once
  // per connection and every row it answers becomes a button, so a cue added to
  // firmware/lib/cue.hxx and flashed appears here with no hub change at all.
  // =========================================================== the Sound view
  // TEMPORARY, and the tab says so. Nothing here reacts to the car; when sound
  // moves into the cue system this view goes and the Cues board grows a column.
  Void drawSoundBody(Float32 w, Float32 h)
  {
      static_cast<Void>(w);
      static_cast<Void>(h);

      const Bool linkUp =
          (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

      ImGui::BeginDisabled(!linkUp);

      // ---- what is wired, said out loud -------------------------------------
      // The pins are read back FROM THE BOARD, not printed from a constant: the
      // failure this view debugs is a wire on the wrong pad.
      ImGui::SeparatorText("Link");
      if(soundTx >= 0)
      {
          colored(ui::ansi::GRAY, "TX GP%d", soundTx);
          ImGui::SameLine(0.0f, 16.0f);
          colored(ui::ansi::GRAY, "-> module RX (through 1k)");
          ImGui::SameLine(0.0f, 24.0f);
          colored(ui::ansi::GRAY, "RX GP%d", soundRx);
          ImGui::SameLine(0.0f, 16.0f);
          colored(ui::ansi::GRAY, "<- module TX");

          // BUSY's pad, read back like the other two: this row is the BOARD's
          // account of its own wiring.
          if(soundBusyGp >= 0)
          {
              colored(ui::ansi::GRAY, "BUSY GP%d", soundBusyGp);
              ImGui::SameLine(0.0f, 16.0f);
              colored(ui::ansi::GRAY, "<- module BUSY, low while playing");
          }
          else
          {
              colored(ui::ansi::GRAY, "BUSY not wired");
          }
      }
      else
      {
          ImGui::TextDisabled("the board has not answered SOUND yet");
      }

      // ---- the card -------------------------------------------------------
      // FIRST and prominent, because its absence is silent: the SD card takes
      // 1.5-3 s to mount and a play sent before that is discarded with no sound.
      ImGui::SeparatorText("Card");

      if(soundReady)
      {
          colored(ui::sem::GOOD, "mounted");
      }
      else
      {
          colored(ui::sem::WARN, "not mounted - nothing will play");
      }
      ImGui::SameLine(0.0f, 16.0f);

      if(ui::iconButton(
          ui::Icon::ICON_REBOOT,
          "Reset / mount card",
          ImVec2(220.0f * uiDpiScale, 0.0f)
      ))
      {
          sendPico("SOUND RESET");
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Resets the module and waits ~2 s for the card.\n" "Needed once after the module is powered up.\n" "The console goes quiet while it waits."
          );
      }

      // ---- volume -----------------------------------------------------------
      ImGui::SeparatorText("Volume");

      // Sent on RELEASE, not every frame: the link is 9600 baud and the module
      // needs 20-40 ms between commands, so a drag would drop most of them.
      ImGui::SetNextItemWidth(320.0f * uiDpiScale);
      ImGui::SliderInt("##soundvol", &soundVol, 0, soundVolMax, "%d");
      if(ImGui::IsItemDeactivatedAfterEdit())
      {
          Array<Char, 48> cmd;
          std::snprintf(cmd.data(), cmd.size(), "SOUND VOL %d", soundVol);
          sendPico(cmd.data());
      }

      ImGui::SameLine(0.0f, 12.0f);
      colored(
          soundVol > (soundVolMax / 2) ? ui::sem::WARN : ui::ansi::GRAY,
          "%d / %d",
          soundVol,
          soundVolMax
      );

      // Named steps rather than a bare number: "8" means nothing until you have
      // been deafened once, and these modules are painful well below half.
      const auto vol = [](const Char* label, Int32 level, const Char* why)
      {
          if(ui::iconButton(ui::Icon::ICON_SIGNAL, label, ImVec2(96.0f * uiDpiScale, 0.0f)))
          {
              Array<Char, 48> cmd;
              std::snprintf(cmd.data(), cmd.size(), "SOUND VOL %d", level);
              sendPico(cmd.data());
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip("%s", why);
          }
      };

      vol("Quiet", 4,  "4 of 30. Start here with the speaker near your head.");
      ImGui::SameLine();
      vol("Low", 8,    "8 of 30. What the speaker sketch uses.");
      ImGui::SameLine();
      vol("Half", 15,  "15 of 30. Loud in a room.");
      ImGui::SameLine();
      vol(
          "Max",
          30,
          "30 of 30, and there is nothing above it - the\n" "protocol range is 0-30, and the module clamps."
      );

      // ---- tone -------------------------------------------------------------
      // NOT a second volume, and labeled so - it is what people reach for at 30.
      ImGui::SeparatorText("Tone");

      static constexpr const Char* const EQ_NAME[6] =
      {
          "Normal", "Pop", "Rock", "Jazz", "Classic", "Bass"
      };

      for(Int32 e = 0; e < 6; ++e)
      {
          if(e > 0)
          {
              ImGui::SameLine();
          }
          ImGui::PushID(100 + e);

          const Bool on = (soundEq == e);
          if(on)
          {
              ImGui::PushStyleColor(ImGuiCol_Button, ImGui::ColorConvertU32ToFloat4(ui::sem::GOOD));
          }
          if(ImGui::Button(EQ_NAME[e], ImVec2(74.0f * uiDpiScale, 0.0f)))
          {
              Array<Char, 32> cmd;
              std::snprintf(cmd.data(), cmd.size(), "SOUND EQ %d", e);
              sendPico(cmd.data());
          }
          if(on)
          {
              ImGui::PopStyleColor();
          }
          ImGui::PopID();
      }

      ImGui::TextDisabled("Tone, not level. Bass can read as louder on a small");
      ImGui::TextDisabled("speaker - or quieter, if it clips a cone with no box.");

      // ---- the track --------------------------------------------------------
      ImGui::SeparatorText("Track");

      // WHAT IS ON THE CARD, asked of the module rather than assumed. Neither the
      // hub nor the firmware keeps a list: adding an mp3 is the whole of adding.
      if(soundFiles > 0)
      {
          colored(
              ui::ansi::GRAY,
              "%d track%s on the card",
              soundFiles,
              (soundFiles == 1) ? "" : "s"
          );
      }
      else
      {
          ImGui::TextDisabled("card not counted yet");
      }
      ImGui::SameLine(0.0f, 12.0f);
      if(ui::iconButton(ui::Icon::ICON_REFRESH, "Count", ImVec2(110.0f * uiDpiScale, 0.0f)))
      {
          sendPico("SOUND FILES");
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Asks the module how many files the card holds.\n" "The only command here that waits for an answer,\n" "so a reply also proves the module is powered,\n" "has a card, and its TX reaches the Pico."
          );
      }

      ImGui::SetNextItemWidth(120.0f * uiDpiScale);
      ImGui::InputInt("##soundtrack", &soundTrack);

      // Clamped to what is ACTUALLY THERE once the card has been counted: track
      // 7 on a card holding three is silence, which has too many causes already.
      const Int32 highest = (soundFiles > 0) ? soundFiles : 3000;
      if(soundTrack < 1)
      {
          soundTrack = 1;
      }
      if(soundTrack > highest)
      {
          soundTrack = highest;
      }
      ImGui::SameLine(0.0f, 12.0f);
      colored(ui::ansi::GRAY, "mp3/%04d.mp3", soundTrack);

      // One button per track, once the count is known and small enough to fit.
      // Above a dozen the row stops being quicker than the box.
      if(soundFiles > 0 && soundFiles <= 12)
      {
          for(Int32 t = 1; t <= soundFiles; ++t)
          {
              if(t > 1)
              {
                  ImGui::SameLine();
              }
              ImGui::PushID(t);
              Array<Char, 8> lbl;
              std::snprintf(lbl.data(), lbl.size(), "%d", t);
              if(ImGui::Button(lbl.data(), ImVec2(44.0f * uiDpiScale, 0.0f)))
              {
                  soundTrack = t;
                  Array<Char, 48> cmd;
                  std::snprintf(cmd.data(), cmd.size(), "SOUND PLAY %d", t);
                  sendPico(cmd.data());
              }
              ImGui::PopID();
          }
      }

      ImGui::Spacing();

      // PLAY is green and wide: it is the thing this view is for.
      if(ui::iconButton(ui::Icon::ICON_PLAY, "Play", ImVec2(160.0f * uiDpiScale, 0.0f)))
      {
          Array<Char, 48> cmd;
          std::snprintf(cmd.data(), cmd.size(), "SOUND PLAY %d", soundTrack);
          sendPico(cmd.data());
      }

      ImGui::SameLine();
      if(ui::iconButton(ui::Icon::ICON_PAUSE, "Pause", ImVec2(120.0f * uiDpiScale, 0.0f)))
      {
          sendPico("SOUND PAUSE");
      }
      ImGui::SameLine();
      if(ui::iconButton(ui::Icon::ICON_PLAY, "Resume", ImVec2(120.0f * uiDpiScale, 0.0f)))
      {
          sendPico("SOUND RESUME");
      }
      ImGui::SameLine();
      if(ui::iconButton(ui::Icon::ICON_MOTOR_STOP, "Stop", ImVec2(120.0f * uiDpiScale, 0.0f)))
      {
          sendPico("SOUND STOP");
      }

      ImGui::Spacing();
      if(ui::iconButton(ui::Icon::ICON_REFRESH, "Prev", ImVec2(110.0f * uiDpiScale, 0.0f)))
      {
          sendPico("SOUND PREV");
      }
      ImGui::SameLine();
      if(ui::iconButton(ui::Icon::ICON_REFRESH, "Next", ImVec2(110.0f * uiDpiScale, 0.0f)))
      {
          sendPico("SOUND NEXT");
      }

      // ---- playing ----------------------------------------------------------
      ImGui::SeparatorText("Playing");

      if(soundBusy == "unwired")
      {
          // Said plainly rather than shown as "not playing", which this car cannot
          // claim: BUSY is the module's own output, while a serial reply only says
          // a command was ACCEPTED. The symbol named below must stay in step with
          // the firmware - it was pins::SOUND_BUSY until the pin map became a value.
          ImGui::TextDisabled("BUSY is not wired, so the board cannot tell.");
          ImGui::TextDisabled("Wire it and set");
          ImGui::SameLine(0.0f, 4.0f);
          colored(ui::ansi::BRCYAN, "soundBusy");
          ImGui::SameLine(0.0f, 4.0f);
          ImGui::TextDisabled("in");
          ImGui::SameLine(0.0f, 4.0f);
          colored(ui::ansi::BRCYAN, "pins::car()");
      }
      else if(soundBusy == "yes")
      {
          // A FILLED BAR, not just a word: this is the one live thing on the
          // screen, readable from across the bench.
          colored(ui::sem::GOOD, "playing");
          ImGui::SameLine(0.0f, 12.0f);

          const Float32 t = static_cast<Float32>(ImGui::GetTime());
          ImGui::PushStyleColor(
              ImGuiCol_PlotHistogram,
              ImGui::ColorConvertU32ToFloat4(ui::sem::GOOD)
          );
          ImGui::ProgressBar(
              0.5f + 0.5f * std::sin(t * 6.0f),
              ImVec2(200.0f * uiDpiScale, 0.0f),
              ""
          );
          ImGui::PopStyleColor();
      }
      else
      {
          colored(ui::ansi::GRAY, "idle");
          ImGui::SameLine(0.0f, 12.0f);
          ImGui::TextDisabled("(BUSY on GP%d reads high)", soundBusyGp);
      }

      ImGui::EndDisabled();

      if(!linkUp)
      {
          ImGui::Spacing();
          colored(ui::sem::WARN, "no board connected");
      }
  }

  Void drawCueBody(Float32 w, Float32 h)
  {
      ImGui::BeginChild("##cueboard", ImVec2(w, h), ImGuiChildFlags_None, ImGuiWindowFlags_None);

      const ImVec2 p0 = ImGui::GetCursorScreenPos();
      const Bool   linkUp = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

      // ---- what the car is saying right now --------------------------------
      // Always present, because a cue is only a few hundred milliseconds long and
      // the only other evidence is a line in a console.
      const Bool speaking = linkUp && !cueSpeaking.empty()
                         && cueSpeaking != "none" && cueSpeaking != "-";

      ImGui::AlignTextToFramePadding();
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED), "Saying");
      ImGui::SameLine();

      if(!linkUp)
      {
          colored(ui::sem::MUTED, "nothing - the board is not connected");
      }
      else if(speaking)
      {
          colored(ui::sem::GOOD, "%s", cueSpeaking.c_str());
          ImGui::SameLine();
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              "step %d, pass %d",
              cueStepNow,
              cueLoopNow + 1
          );
      }
      else
      {
          colored(ui::sem::MUTED, "nothing");
      }

      ImGui::Separator();
      ImGui::Spacing();

      // ---- the grid ---------------------------------------------------------
      if(cueList.empty())
      {
          // Three different silences, and they are three different problems.
          ImGui::Spacing();
          if(!linkUp)
          {
              colored(
                  ui::sem::MUTED,
                  "The cues come from the board, so this fills in when it is\n" "connected. Nothing about them is stored on this side."
              );
          }
          else if(!cueListAsked)
          {
              colored(ui::sem::MUTED, "Asking the board...");
          }
          else
          {
              colored(
                  ui::sem::WARN,
                  "The board answered CUE LIST with nothing.\n" "\n" "Either the firmware on it predates the cue system, or it\n" "was built with no cues in the table. Reflash and it will\n" "fill in - the list is asked again on every connection."
              );
          }

          ui::screenInset(p0, ImVec2(p0.x + w, p0.y + h));
          ImGui::EndChild();
          return;
      }

      // Columns from the width, not a fixed count. A name and a sentence need
      // room; two cramped columns are worse than one honest one.
      const Float32 avail = ImGui::GetContentRegionAvail().x;
      const Float32 cellMin = 260.0f * uiDpiScale;
      const Int32   cols = std::max(1, static_cast<Int32>(avail / cellMin));

      const Float32 btnH = ImGui::GetFrameHeight() * 1.9f;

      ImGui::BeginDisabled(!linkUp);
      if(ImGui::BeginTable("##cuegrid", cols, ImGuiTableFlags_SizingStretchSame))
      {
          for(Size i = 0; i < cueList.size(); ++i)
          {
              if((i % static_cast<Size>(cols)) == 0)
              {
                  ImGui::TableNextRow();
              }
              ImGui::TableNextColumn();

              const CueEntry& c = cueList[i];
              ImGui::PushID(static_cast<Int32>(i));

              // A cue that stays up is a TOGGLE; a one-shot is a button. The
              // firmware says which, so this does not have to guess.
              const Bool sticky = (c.play == "hold" || c.play == "loop");

              // Tinted while it is up. GREEN for a cue a person is holding, AMBER
              // for one the car raised by itself.
              const ui::Tint tint = !c.on          ? ui::Tint::TINT_NONE
                                   : c.latched     ? ui::Tint::TINT_GOOD
                                                   : ui::Tint::TINT_WARN;

              Array<Char, 64> label;
              std::snprintf(label.data(), label.size(), "%s", c.name.c_str());
              for(Char* q = label.data(); *q != '\0'; ++q)
              {
                  *q = static_cast<Char>(std::toupper(static_cast<UInt8>(*q)));
              }

              if(ui::iconButton(ui::Icon::ICON_LAMP, label.data(), ImVec2(-FLT_MIN, btnH), tint))
              {
                  Array<Char, 80> cmd;

                  // Pressing a sticky cue a PERSON is holding lowers it; pressing
                  // one the CAR raised takes it over rather than fighting it.
                  if(sticky && c.on && c.latched)
                  {
                      std::snprintf(cmd.data(), cmd.size(), "CUE %s OFF", c.name.c_str());
                  }
                  else
                  {
                      std::snprintf(cmd.data(), cmd.size(), "CUE %s", c.name.c_str());
                  }

                  sendPico(cmd.data());
                  LOG_INFO("cue", "%s", cmd.data() + 4);
              }

              if(ImGui::IsItemHovered())
              {
                  const Char* what =
                      (c.play == "hold") ? "Held until you press it again."
                    : (c.play == "loop") ? "Repeats until you press it again."
                    : (c.play == "once") ? "Plays once and ends on its own."
                                         : "";

                  const Char* who =
                      !c.on        ? ""
                    : c.latched    ? "\n\nUP, because you raised it."
                                   : "\n\nUP, because the car raised it. Pressing"
                                     " takes it over.";

                  ImGui::SetTooltip(
                      "CUE %s\n\n%s\n\n%s%s",
                      c.name.c_str(),
                      c.means.c_str(),
                      what,
                      who
                  );
              }

              // The sentence the FIRMWARE gives it, not one written here. If it
              // reads badly that is a prompt to fix cue.hxx, where it belongs.
              ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED));
              ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x);
              ImGui::TextUnformatted(c.means.c_str());
              ImGui::PopTextWrapPos();
              ImGui::PopStyleColor();

              if(!c.play.empty())
              {
                  ImGui::TextColored(
                      ImGui::ColorConvertU32ToFloat4(
                          c.on ? (c.latched ? ui::sem::GOOD : ui::sem::WARN) : ui::sem::MUTED
                      ),
                      "%s%s", c.play.c_str(),
                      !c.on ? "" : (c.latched ? "  -  you" : "  -  the car"));
              }

              ImGui::Spacing();
              ImGui::PopID();
          }
          ImGui::EndTable();
      }
      ImGui::EndDisabled();

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::Spacing();

      // ---- stop -------------------------------------------------------------
      // NOT disabled when the link is down: a stop you cannot press is worse than
      // one that presses and does nothing.
      ui::pushTint(ui::Tint::TINT_WARN);
      if(ui::iconButton(
          ui::Icon::ICON_MOTOR_STOP,
          "Stop the cue",
          ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 1.4f)
      ))
      {
          sendPico("CUE STOP");
          LOG_INFO("cue", "stopped");
      }
      ui::popTint(ui::Tint::TINT_WARN);

      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Ends whatever cue is running and hands its channels back.\n" "\n" "A cue OWNS every channel its steps mention for its whole\n" "duration, dark steps included - which is what makes a flash\n" "visible when the headlights are already on. Stopping it gives\n" "those channels back to the car, so the tails and indicators go\n" "back to following what the car is doing.\n" "\n" "Not the emergency stop. This stops the car TALKING, not the car."
          );
      }

      ImGui::Spacing();
      ImGui::TextColored(
          ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
          "%d cue%s, read from the board on connect." " Green is yours, amber is the car's.",
          static_cast<Int32>(cueList.size()),
          cueList.size() == 1 ? "" : "s"
      );

      ui::screenInset(p0, ImVec2(p0.x + w, p0.y + h));
      ImGui::EndChild();
  }

  // ============================================== the range view ==
  // What the ToF sensor on the car's nose is seeing, live. Everything here is an
  // ANSWER from the board, not an assumption: "not detected" and "never asked"
  // are different problems, and the top strip keeps them apart.
  Void drawRangeBody(Float32 w, Float32 h)
  {
      ImGui::BeginChild(
          "##range",
          ImVec2(w, h),
          ImGuiChildFlags_None,
          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
      );

      const ImVec2 p0 = ImGui::GetCursorScreenPos();
      ImDrawList*  dl = ImGui::GetWindowDrawList();

      dl->AddRectFilled(p0, ImVec2(p0.x + w, p0.y + h), ui::ansi::BLACK);

      const Float32 pad = 16.0f * uiDpiScale;
      const Bool    live = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

      // ---- the top strip: is there a sensor at all ------------------------
      {
          const Char* what = nullptr;
          ImU32       col = ui::sem::MUTED;

          if(!live)
          {
              what = "Pico not connected";
          }
          else if(!sensorsAsked)
          {
              what = "asking the board what is attached...";
              col = ui::sem::WARN;
          }
          else if(!sensorI2c)
          {
              what = "no I2C bus on the board";
              col = ui::sem::BAD;
          }
          else if(!sensorTof)
          {
              // MUTED, not BAD: nothing is broken, there is simply no ToF wired to
              // this car yet. "no I2C bus" above stays BAD - that one IS a fault.
              what = "no VL53L1X wired - nothing at 0x29";
              col = ui::sem::MUTED;
          }
          else
          {
              what = "VL53L1X on I2C0, GP4 / GP5";
              col = ui::sem::GOOD;
          }

          ui::iconAt(
              dl,
              sensorTof ? ui::Icon::ICON_STATUS_OK : ui::Icon::ICON_STATUS_IDLE,
              ImVec2(p0.x + pad, p0.y + pad)
          );
          dl->AddText(
              ImVec2(p0.x + pad + ui::iconSize() + 8.0f * uiDpiScale, p0.y + pad),
              col,
              what
          );
      }

      // ---- the mode switch ------------------------------------------------
      // SHORT reaches about 1.3 m and rejects ambient infrared well; LONG reaches
      // about 4 m and is easily blinded. Which is right depends on the room.
      if(live && sensorTof)
      {
          ImGui::SetCursorScreenPos(ImVec2(
              p0.x + w - 190.0f * uiDpiScale,
              p0.y + pad - 4.0f * uiDpiScale
          ));
          if(ui::segmentedButton("Short", tofModeShort, ImVec2(88.0f * uiDpiScale, 0.0f)))
          {
              tofModeShort = true;
              sendPico("TOF MODE SHORT");
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "Short range - reaches about 1.3 m\n" "\n" "The laser pulses at a HIGHER frequency, which narrows the\n" "window the sensor accepts a return in. A narrower window lets\n" "in less of everything else, so stray infrared is rejected -\n" "which is what actually limits this sensor outdoors.\n" "\n" "Use it when: there is sunlight, a halogen or an incandescent\n" "lamp about, or the target is dark or angled. All of those\n" "weaken the return relative to the background.\n" "\n" "The trade is only reach. Accuracy is no worse - if anything\n" "it is steadier, because there is less to confuse it.\n" "\n" "This is the mode a bumper wants. Nothing useful for stopping\n" "a car is more than a meter away, and daylight is exactly the\n" "condition it has to work in."
              );
          }

          ImGui::SameLine(0.0f, 2.0f);
          if(ui::segmentedButton("Long", !tofModeShort, ImVec2(88.0f * uiDpiScale, 0.0f)))
          {
              tofModeShort = false;
              sendPico("TOF MODE LONG");
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "Long range - reaches about 4 m indoors\n" "\n" "A LOWER pulse frequency and a wider acceptance window, so a\n" "faint return from something far away still counts. That same\n" "width is what lets ambient infrared in, and daylight has a\n" "great deal of it.\n" "\n" "Use it when: indoors, away from a window, and you need to see\n" "past a meter or so.\n" "\n" "The 4 m on the box assumes a white target, a dark room and a\n" "long timing budget. A dark or angled surface returns far less\n" "light and will fall well short of it.\n" "\n" "Watch the signal and ambient figures below: if ambient starts\n" "to approach signal, this mode is being blinded and Short will\n" "do better even though it reaches less far."
              );
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
          // Aligned with the LABEL above it rather than with the status lamp, so
          // the block has one left edge.
          const Float32 textX = p0.x + pad + ui::iconSize() + 8.0f * uiDpiScale;
          dl->AddText(ImVec2(textX, top + 8.0f * uiDpiScale), ui::sem::MUTED, hint);

          // The bezel, which this path used to skip - so the view lost its frame
          // in precisely the states somebody spends the most time looking at.
          ui::screenInset(p0, ImVec2(p0.x + w, p0.y + h));
          ImGui::EndChild();
          return;
      }

      const Bool good = (tofStatus == 0);

      // ---- the number ------------------------------------------------------
      {
          Array<Char, 32> buf;
          std::snprintf(buf.data(), buf.size(), good ? "%d" : "----", tofMm);

          ImFont* const f = ui::fonts.big ? ui::fonts.big : ImGui::GetFont();
          const Float32 fs = (f != nullptr && f->LegacySize > 0.0f)
                           ? f->LegacySize * 2.0f
                           : ImGui::GetFontSize() * 3.0f;

          dl->AddText(
              f,
              fs,
              ImVec2(p0.x + pad, top),
              good ? ui::ansi::BRWHITE : ui::sem::MUTED,
              buf.data()
          );

          const Float32 numW = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, buf.data()).x;
          dl->AddText(
              ImVec2(p0.x + pad + numW + 10.0f * uiDpiScale, top + fs * 0.55f),
              ui::sem::MUTED,
              "mm"
          );

          if(good)
          {
              std::snprintf(buf.data(), buf.size(), "%d.%02d m", tofMm / 1000, (tofMm % 1000) / 10);
              dl->AddText(
                  ImVec2(p0.x + pad + numW + 10.0f * uiDpiScale, top + fs * 0.05f),
                  ui::plot::OK,
                  buf.data()
              );
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
              dl->AddText(ImVec2(p0.x + pad, top + fs + 4.0f * uiDpiScale), ui::sem::WARN, why);
          }
      }

      // ---- the strip chart -------------------------------------------------
      // A number alone is hard to WATCH: a hand moved and a trace following it
      // says the sensor is tracking.
      const Float32 chartTop = top + 92.0f * uiDpiScale;
      const Float32 chartH = std::max(60.0f, (p0.y + h) - chartTop - 62.0f * uiDpiScale);
      const Float32 chartW = w - (2.0f * pad);

      const ImVec2 c0(p0.x + pad, chartTop);
      const ImVec2 c1(c0.x + chartW, chartTop + chartH);

      dl->AddRectFilled(c0, c1, IM_COL32(0x14, 0x16, 0x1A, 0xFF));
      dl->AddRect(c0, c1, IM_COL32(0x30, 0x32, 0x38, 0xFF));

      // A fixed 2 m scale rather than one fitted to the data: an autoscaling
      // chart looks identical sweeping a room and jittering by three millimeters.
      constexpr Float32 FULL_MM = 2000.0f;

      for(Int32 g = 1; g < 4; ++g)
      {
          const Float32 gy = c1.y - (chartH * (static_cast<Float32>(g) / 4.0f));
          dl->AddLine(ImVec2(c0.x, gy), ImVec2(c1.x, gy), IM_COL32(0x26, 0x28, 0x2E, 0xFF));

          Array<Char, 16> lab;
          std::snprintf(
              lab.data(),
              lab.size(),
              "%.1fm",
              static_cast<Float64>(FULL_MM * (static_cast<Float32>(g) / 4.0f) / 1000.0f)
          );
          dl->AddText(
              ImVec2(c0.x + 4.0f, gy - 14.0f * uiDpiScale),
              IM_COL32(0x50, 0x52, 0x58, 0xFF),
              lab.data()
          );
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

                  const ImVec2 pt(c0.x + (static_cast<Float32>(i) * step), c1.y - (v * chartH));
                  if(havePrev)
                  {
                      dl->AddLine(prev, pt, ui::plot::OK, 1.6f);
                  }
                  prev = pt;
                  havePrev = true;
              }
          }
      }

      // ---- the footer ------------------------------------------------------
      {
          Array<Char, 96> buf;
          if(tofSeenMax > 0)
          {
              // Raw, in the sensor's own fixed point: the RATIO is the whole
              // diagnostic and it is scale-free.
              if(tofSignal >= 0)
              {
                  std::snprintf(
                      buf.data(),
                      buf.size(),
                      "seen %d - %d mm %llu readings " "signal %d ambient %d",
                      tofSeenMin,
                      tofSeenMax,
                      static_cast<unsigned long long>(tofReplies),
                      tofSignal,
                      tofAmbient
                  );
              }
              else
              {
                  std::snprintf(
                      buf.data(),
                      buf.size(),
                      "seen %d - %d mm %llu readings",
                      tofSeenMin,
                      tofSeenMax,
                      static_cast<unsigned long long>(tofReplies)
                  );
              }
          }
          else
          {
              std::snprintf(buf.data(), buf.size(), "no good reading yet");
          }
          dl->AddText(ImVec2(c0.x, c1.y + 8.0f * uiDpiScale), ui::sem::MUTED, buf.data());

          // How stale the number is: a quiet link leaves the last reading looking
          // perfectly current.
          // ---- why the reading is what it is -----------------------------
          // The DISTANCE pattern alone cannot tell you - short-and-steady reads as
          // "protective film" while the rates say signal 5, ambient 511. The RATIO
          // is the diagnosis: ambient >> signal means blinded by room infrared,
          // what Short mode is for; high signal at short range really is that close.
          if(tofSignal >= 0 && tofReplies > 30)
          {
              const Char* why = nullptr;

              // Blinded: the room's infrared drowns the return. The threshold is
              // deliberately high - the two are COMPARABLE in normal use.
              if(tofAmbient > (tofSignal * 6) && tofSignal < 300)
              {
                  why = "Ambient light is swamping the signal - try Short mode, "
                        "or move away from a window or lamp.";
              }
              // A STRONG return from a FIXED short distance whatever is in front
              // is not a room; it is the protective film on the lens.
              else if(tofSeenMax > 0 && tofSeenMax < 250 && tofSignal > 200
                      && (tofSeenMax - tofSeenMin) < (tofSeenMax / 2))
              {
                  why = "A strong return from a fixed short distance - is the "
                        "protective film still on the lens?";
              }

              if(why != nullptr)
              {
                  dl->AddText(ImVec2(c0.x, c1.y + 26.0f * uiDpiScale), ui::sem::WARN, why);
              }
          }

          const Float64 age = ImGui::GetTime() - tofLastReply;
          if(tofReplies > 0 && age > 1.0)
          {
              std::snprintf(buf.data(), buf.size(), "last reply %.0f s ago", age);
              dl->AddText(ImVec2(c0.x, c1.y + 26.0f * uiDpiScale), ui::sem::WARN, buf.data());
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
  constexpr Step SERVO_STEPS[] =
  {
    { "-50", -50 }, { "-10", -10 }, { "-1", -1 },
    { "+1",    1 }, { "+10",  10 }, { "+50", 50 },
  };

  // Throttle steps are smaller: a motor that jumps 50 us has already spun up. The
  // "##esc" suffixes are required - ImGui identifies a widget by its LABEL.
  constexpr Step ESC_STEPS[] =
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

  // The real ceiling on the WASD forward cap: the hard limit, but never outside
  // the calibration, and never inverted if a recalibration put idle above it.
  Int32 wasdCapCeil()
  {
      return std::max(driveEscMin, std::min(driveEscMax, WASD_CAP_HARD));
  }

  // What W will actually ask for, wherever the slider happens to be sitting.
  Int32 wasdCapNow()
  {
      return clampInt(wasdCapUs, driveEscMin, wasdCapCeil());
  }

  // Offers the image this view actually needs. Shared by both ways a board can
  // fail to answer, because they have the same cause and the same fix.
  Void drawDriveFlashButton()
  {
      if(picoFlash.busy())
      {
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
              "%s...",
              picoFlash.currentOp().c_str()
          );
          return;
      }

      if(ui::iconButton(
          ui::Icon::ICON_FLASH,
          "Flash Debug / Blink",
          ImVec2(280.0f * uiDpiScale, 0.0f),
          ui::Tint::TINT_WARN
      ))
      {
          // flash.ps1 does the 1200-baud touch itself, and it cannot open the
          // port while this app is holding it.
          picoLink.disconnect();
          releasePicoPortForBoardOp();
          picoFlash.flash("pico_debug");
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Builds firmware/app/main.c and writes it to the " "board.\n\nThis OVERWRITES whatever sketch is on " "there. The\nsource is untouched - rebuild and " "reflash it from the\nCode view whenever you want " "it back."
          );
      }
  }

  // ---- the calibration, on disk ------------------------------------------

  Str steeringCalPath()
  {
      const Str d = sketch::firmwareDir();
      // .hxx, NOT ".h": chassis.hxx includes "cal.hxx", so writing cal.h puts the
      // measured numbers into a file nothing compiles, with no error anywhere.
      return d.empty() ? Str() : (d + "\\lib\\chassis\\cal.hxx");
  }

  Bool readThrottleNumbers(const Str& text, Int32& lo, Int32& hi);

  // The STEERING is loaded from settings, not the header: the header is generated
  // output, and parsing back what we printed would let the two silently diverge.
  Void loadCalibration()
  {
      calLoaded = true;
      calWritten = sketch::load(steeringCalPath());

      // The THROTTLE does come from the header: with no working copy in settings,
      // "Write to firmware" would put 1500 back over a measured idle of 1541.
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
          calLeft = l;
          calCenter = c;
          calRight = r;
      }
  }

  Void saveCalibration()
  {
      Array<Char, 64> buf;
      std::snprintf(buf.data(), buf.size(), "%d %d %d\n", calLeft, calCenter, calRight);
      settings::write("steering.txt", Str(buf.data()));
  }

  // When the three numbers were MEASURED, which is NOT when the header is
  // written: time(nullptr) at write time makes the field that exists so stale
  // numbers can be spotted the one thing guaranteed to look fresh. steering.txt's
  // mtime is honest - it is written by saveCalibration() and nothing else.
  std::time_t calMeasuredAt()
  {
      const Int64 secs = sketch::modifiedAtUnix(settings::path("steering.txt"));
      return (secs > 0) ? static_cast<std::time_t>(secs) : std::time(nullptr);
  }

  // The generated header, built as text so the view can SHOW it before anything
  // is written - a file with no preview is one nobody reads until it is wrong.
  Str steeringCalText()
  {
      // The fallback when localtime_s below fails; normally overwritten.
      Array<Char, 64> when{};
      std::snprintf(when.data(), when.size(), "unknown date");
      const std::time_t now = calMeasuredAt();
      std::tm           tm{};
      if(localtime_s(&tm, &now) == 0)
      {
          std::strftime(when.data(), when.size(), "%Y-%m-%d", &tm);
      }

      /*
       * Big enough, and CHECKED. snprintf truncates silently and returns what it
       * WOULD have written: at 2048 the header outgrew the buffer and the Drive
       * view wrote a cal.hxx that stopped mid-comment. The guard below is the
       * part that matters - if it does not fit, this returns nothing and the
       * caller refuses to write, because no file beats half of one.
       */
      Array<Char, 8192> buf;
      const Int32 need = std::snprintf(buf.data(), buf.size(),
          "/* ---------------------------------------------------------------------------\n"
          " * Steering calibration - GENERATED.\n"
          " *\n"
          " * Written by the hub's Drive view. Edit it THERE, not here: the next \"Write to\n"
          " * firmware\" overwrites this file completely, and a number typed in by hand is\n"
          " * gone the first time anyone touches the calibration UI.\n"
          " *\n"
          " * These are measurements of one particular car, not a datasheet. A servo's own\n"
          " * range is 1000-2000 us; what a TT-02's steering can actually reach is narrower\n"
          " * and off-center, because the horn only fits the spline at whole-tooth\n"
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
          " *\n"
          " * MIN is IDLE. Not the ESC's neutral and not a safety floor, but the pulse at\n"
          " * which this motor sits still and the next microsecond starts it turning. That\n"
          " * is a fact about this car's ESC and motor, found by winding it up until the\n"
          " * wheels moved - which is why it is not the round number anybody would guess.\n"
          " *\n"
          " * It matters that this is the floor the sliders are built from: a range\n"
          " * starting below idle spends its first stretch doing nothing at all, so the\n"
          " * control feels dead at one end for no reason a driver could work out.\n"
          " *\n"
          " * A GEAR CHANGE INVALIDATES THIS. Idle is where the motor overcomes the\n"
          " * drivetrain, so more reduction breaks static friction at a lower pulse and\n"
          " * this number goes down. The 17T pinion went on after this was measured at 19T,\n"
          " * and it has not been re-measured since - so 1541 is an upper bound on idle\n"
          " * rather than idle.\n"
          " *\n"
          " * That is not cosmetic. driveThrottleUs clamps UPWARD to escMin, and the\n"
          " * deadman calls the car driven when escTargetUs > escMinUs - so a car that\n"
          " * creeps at what this file calls idle is a car the deadman does not think is\n"
          " * moving.\n"
          " */\n"
          "#define THROTTLE_CAL_MIN %d\n"
          "#define THROTTLE_CAL_MAX %d\n"
          "\n"
          "/* ---- tuning, not measurement ---------------------------------------------\n"
          " *\n"
          " * Everything above is a fact about this car that was found by moving it. This\n"
          " * is not: it is a choice about how fast the outputs are allowed to move, and a\n"
          " * different answer is right for a bench than for driving.\n"
          " *\n"
          " * It lives here anyway for one reason - this is the file that survives a\n"
          " * reflash. The throttle range was runtime-only until 2026-08-27 and was\n"
          " * silently lost every time the board was rewritten, which is not a setting, it\n"
          " * is a setting you have to remember to make again.\n"
          " *\n"
          " * Microseconds of pulse per 20 ms tick. 8 is 400 us/s, which walks this car's\n"
          " * 430 us of steering travel in about a second - deliberate on a bench and far\n"
          " * too slow to steer around anything.\n"
          " */\n"
          "#define SLEW_CAL_STEER    %d\n"
          "\n"
          "/* The throttle's own rate. Separate from the steering because the right answer\n"
          " * is different: a servo should arrive promptly, an ESC should be led there.\n"
          " * Starts equal to the steering, which is what the single shared rate used to\n"
          " * give - so nothing changes until it is tuned. */\n"
          "#define SLEW_CAL_THROTTLE %d\n"
          "\n"
          "/* ---- when the tail lamps go out ------------------------------------------\n"
          " *\n"
          " * Microseconds ABOVE idle at which the car counts as being driven, and the\n"
          " * tails extinguish. Below it the motor is turning but barely, and a car\n"
          " * crawling is a car that has not really pulled away - the lamp should still be\n"
          " * on.\n"
          " *\n"
          " * Also used the other way for reverse: more than this BELOW neutral counts as\n"
          " * being driven backwards.\n"
          " *\n"
          " * Tuning, not measurement, like the slew step above - it is a judgment about\n"
          " * when \"moving\" starts, and the honest answer is whatever looks right on the\n"
          " * car. It lives here for the same reason: this is the file that survives a\n"
          " * reflash.\n"
          " */\n"
          "#define LIGHT_CAL_OFF_US %d\n"
          "\n"
          "/* When this car was last calibrated, so a stale set of numbers can be spotted\n"
          " * rather than trusted. \"defaults\" means nobody has calibrated this car yet. */\n"
          "#define STEER_CAL_STAMP \"measured %s\"\n",
          calLeft, calCenter, calRight, driveEscMin, driveEscMax,
          driveSlew, driveEscSlew, boardLightsOff, when.data());

      if(need < 0 || static_cast<Size>(need) >= buf.size())
      {
          LOG_WARN(
              "drive",
              "cal.hxx would be %d bytes, buffer is %d - not written",
              need,
              static_cast<Int32>(buf.size())
          );
          return Str();
      }

      return Str(buf.data());
  }

  // Pulls the three numbers back out of a generated header - a parser for what we
  // printed. Anything unrecognized reads as "no", which prompts a write.
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
  // persisted has the steering numbers and not these, and is still readable.
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

  // One row of the calibration. "Set to here" captures the TARGET, not the
  // output: the output is mid-slew half the time and nobody looked at it.
  Void calRow(const Char* label, const Char* help, Int32* value)
  {
      ImGui::PushID(label);

      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED), "%-10s", label);
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip("%s", help);
      }

      ImGui::SameLine(110.0f * uiDpiScale);
      ImGui::SetNextItemWidth(110.0f * uiDpiScale);
      if(ImGui::InputInt("##us", value, 1, 10))
      {
          *value = clampInt(*value, 1000, 2000);
          calDirty = true;
          saveCalibration();
      }

      ImGui::SameLine();
      ImGui::BeginDisabled(!driveKnown);
      if(ui::button("Set to here", ImVec2(120.0f * uiDpiScale, 0.0f)))
      {
          *value = driveServoT;
          calDirty = true;
          saveCalibration();
      }
      ImGui::EndDisabled();
      if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      {
          ImGui::SetTooltip(
              "Records where the servo is being told to hold, right\n" "now, as this point.\n" "\n" "The TARGET, not the output - the output is mid-ramp\n" "half the time, and a number the servo was only\n" "passing through is not a position anyone looked at."
          );
      }

      ImGui::SameLine();
      ImGui::BeginDisabled(!driveServoOn);
      if(ui::button(driveServoOn ? "Go" : "Go (off)", ImVec2(60.0f * uiDpiScale, 0.0f)))
      {
          driveSweep = false;
          driveServoWant = *value;
          Array<Char, 32> cmd;
          std::snprintf(cmd.data(), cmd.size(), "SERVO %d", *value);
          sendPico(cmd.data());
      }
      ImGui::EndDisabled();
      if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      {
          ImGui::SetTooltip(
              "Drives the servo to this point, so you can check it\n" "is still where you thought it was.\n" "\n" "Needs the servo engaged."
          );
      }

      ImGui::PopID();
  }

  // A state lamp with its label, the way every other indicator here is drawn.
  // ui::led throws a halo when lit, so it reads as emitting.
  Void driveLamp(Bool lit, ImU32 color, const Char* label)
  {
      const Float32 r = 4.0f * uiDpiScale;
      const ImVec2  at = ImGui::GetCursorScreenPos();
      const Float32 mid = at.y + (ImGui::GetTextLineHeight() * 0.5f);

      ui::led(ImGui::GetWindowDrawList(), ImVec2(at.x + r, mid), r, color, lit);

      ImGui::Dummy(ImVec2(r * 2.0f + (6.0f * uiDpiScale), ImGui::GetTextLineHeight()));
      ImGui::SameLine(0.0f, 0.0f);
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(color), "%s", label);
  }

  // ---- the car, seen from above ---------------------------------------------
  // A wireframe rather than a picture. It answers, at a glance, the two questions
  // somebody next to a powered car has: WHICH WAY ARE THE FRONT WHEELS POINTING,
  // and IS THE MOTOR LIVE. The numbers say both and nobody converts them in time.

  // How far the front wheels swing on screen at full lock. A TT-02's real
  // steering is around 28 degrees, which looks timid here; exaggerated to 34.
  constexpr Float32 CHASSIS_LOCK_DEG = 34.0f;

  // One stroke weight for the whole drivetrain: vary it and the eye reads the
  // heavier parts as nearer. Color carries the meaning here, weight carries none.
  constexpr Float32 WIRE_W = 1.0f;

  // The unfilled interior. Not quite the panel black, so a wheel crossing a beam
  // still occludes it.
  constexpr ImU32 WIRE_VOID = IM_COL32(0x08, 0x08, 0x08, 0xFF);

  // One wheel, rotated about its own center.
  Void drawWheel(ImDrawList* dl, const ImVec2& c, Float32 hw, Float32 hh, Float32 deg, ImU32 fill, ImU32 edge)
  {
      const Float32 r = deg * 3.14159265f / 180.0f;
      const Float32 cs = std::cos(r);
      const Float32 sn = std::sin(r);

      const ImVec2 corner[4] = {
          ImVec2(-hw, -hh), ImVec2(hw, -hh), ImVec2(hw, hh), ImVec2(-hw, hh)
      };

      ImVec2 p[4];
      for(Int32 i = 0; i < 4; ++i)
      {
          p[i] = ImVec2(
              c.x + (corner[i].x * cs) - (corner[i].y * sn),
              c.y + (corner[i].x * sn) + (corner[i].y * cs)
          );
      }

      // Outline over a near-black fill, not a solid: the fill only stops the beam
      // behind a wheel showing through. All the information is in the stroke.
      dl->AddQuadFilled(p[0], p[1], p[2], p[3], fill);
      dl->AddQuad(p[0], p[1], p[2], p[3], edge, WIRE_W);

      // A tread line down the middle, so a rotated wheel reads as rotated rather
      // than as a slightly different rectangle.
      const ImVec2 t0(c.x - (hh * 0.62f * -sn), c.y - (hh * 0.62f * cs));
      const ImVec2 t1(c.x + (hh * 0.62f * -sn), c.y + (hh * 0.62f * cs));
      dl->AddLine(t0, t1, edge, WIRE_W);
  }

  // The Drive view's layout grid: ONE right-hand column for the whole view, so
  // the controls form a column instead of a staircase.
  constexpr Float32 DRIVE_TAIL_W = 120.0f;

  // One width for every nudge button - the servo steps and the throttle steps -
  // so the two rows line up with each other as well as with everything else.
  constexpr Float32 DRIVE_STEP_W = 46.0f;

  // How far into its working range the throttle is, 0..1. ONE function: three
  // copies measured from a hardcoded 1500 rather than from the CALIBRATED idle,
  // so once idle was measured at 1541 the bar read 41% with the motor standing
  // still. Clamped: a disarmed ESC sits at neutral, BELOW idle.
  Float32 throttleFraction(Int32 us)
  {
      const Int32 span = driveEscMax - driveEscMin;
      if(span <= 0)
      {
          return 0.0f;   // the range collapses while limits are being edited
      }

      const Float32 f = static_cast<Float32>(us - driveEscMin)
                      / static_cast<Float32>(span);
      return (f < 0.0f) ? 0.0f : ((f > 1.0f) ? 1.0f : f);
  }

  // A section head: a rule, the name in the title face, and what it is for. The
  // captions under it are MUTED body text, so the heading needs the weight.
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

  // A reading pushed to the right-hand edge of the current line, so a number that
  // changes width does not shuffle everything after it.
  Void driveReading(ImU32 col, const Char* text)
  {
      // One spacing in from the edge. Flush against it the last glyph is clipped
      // by the panel's own clip rect - "lock to lock 1.10 s" lost its s.
      const Float32 pad = ImGui::GetStyle().ItemSpacing.x;
      const Float32 w = ImGui::CalcTextSize(text).x;
      const Float32 x = ImGui::GetCursorPosX()
                        + ImGui::GetContentRegionAvail().x - w - pad;

      ImGui::SameLine();
      ImGui::SetCursorPosX(x);
      ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(col), "%s", text);
  }

  // steerN is -1..+1 of THIS car's travel; powerN is 0..1 of its throttle range.
  // A DRIVETRAIN, not a car: two axle beams and the shaft between them, a wheel
  // on each end, no bodywork - the shell neither steers nor drives. A struct
  // rather than parameters because the longhand signature ran to 175 columns.
  struct ChassisFrame
  {
      ImVec2  p0{ 0.0f, 0.0f };
      Float32 w = 0.0f;
      Float32 h = 0.0f;
      Float32 steerN = 0.0f;   // -1 left, +1 right
      Float32 powerN = 0.0f;   // 0..1 of commanded throttle
      Bool    servoLive = false;
      Bool    armed = false;
  };

  // The lamp model is ten wide on both rows - the level and the pin it is
  // bound to - so the two are one type rather than two spellings of Int32*.
  using LampRow = Array<Int32, LAMP_N>;

  Void drawChassis(ImDrawList* dl, const ChassisFrame& f, const LampRow& lamp, const LampRow& pins)
  {
      const ImVec2& p0 = f.p0;
      const Float32 w = f.w;
      const Float32 h = f.h;
      const Float32 steerN = f.steerN;
      const Float32 powerN = f.powerN;
      const Bool    servoLive = f.servoLive;
      const Bool    armed = f.armed;
      const Int32* const lampPin = pins.data();
      const ImVec2 p1(p0.x + w, p0.y + h);

      // The well the drivetrain sits in. Black and square-cornered: this is a
      // screen the drawing is on, not a molded recess it sits in.
      dl->AddRectFilled(p0, p1, ui::ansi::BLACK, 0.0f);

      const ImVec2 mid((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);

      // Everything is derived from the wheelbase, so the drawing scales with the
      // panel rather than being a pile of tuned pixels.
      const Float32 byH = h * 0.62f;
      const Float32 byW = w * 0.46f;
      const Float32 base = (byH < byW) ? byH : byW;   // front axle to rear axle

      const Float32 axleF = mid.y - (base * 0.5f);
      const Float32 axleR = mid.y + (base * 0.5f);
      const Float32 track = base * 0.42f;             // center to wheel center

      const Float32 wheelH = base * 0.20f;
      const Float32 wheelW = base * 0.075f;

      const Float32 beamT = base * 0.045f;           // half-thickness of a beam
      const Float32 shaftT = base * 0.030f;

      // Structure is one color and one color only: anything that changes color in
      // this drawing is reporting something, and the frame reports nothing.
      const ImU32 frame = ui::ansi::CYAN;

      // ---- the lamps --------------------------------------------------------
      // All four corners, from the board's own answer, indicators outboard of the
      // FRONT wheels and tails outboard of the REAR. Drawn BEFORE the drivetrain
      // so a beam crossing one reads as behind the axle. A lamp with no LED is an
      // EMPTY ring, not a dark one - which is why the pin list is reported.
      {
          const Float32 lampR = base * 0.055f;
          const Float32 lampX = track + wheelW * 2.6f;
          const Float32 lampFY = axleF - wheelH * 0.35f;
          const Float32 lampRY = axleR + wheelH * 0.35f;

          // BRYELLOW rather than a true amber, and BRRED for the tails: the price
          // of a sixteen-color palette, and consistency beats accuracy here.
          const ImU32 unwired = ui::ansi::GRID;

          const auto oneLamp = [&](Float32 x, Float32 y, Int32 idx, ImU32 col)
          {
              const Bool wired = (lampPin[idx] >= 0);
              const Bool lit = wired && (lamp[idx] > 0);

              if(wired)
              {
                  ui::led(dl, ImVec2(x, y), lampR, col, lit);
              }
              dl->AddCircle(
                  ImVec2(x, y),
                  lampR,
                  wired ? ((col & 0x00FFFFFFu) | (0x60u << IM_COL32_A_SHIFT)) : unwired,
                  0,
                  WIRE_W
              );
          };

          oneLamp(mid.x - lampX, lampFY, 4, ui::ansi::BRYELLOW);   // indL
          oneLamp(mid.x + lampX, lampFY, 5, ui::ansi::BRYELLOW);   // indR
          oneLamp(mid.x - lampX, lampRY, 2, ui::ansi::BRRED);      // tailL
          oneLamp(mid.x + lampX, lampRY, 3, ui::ansi::BRRED);      // tailR
      }

      const Float32 g = armed ? powerN : 0.0f;

      // The shaft: an outline, and a fill that grows as a BAR rather than a tint -
      // you cannot tell 40% from 60% by hue. It fills from the rear axle forward.
      dl->AddRectFilled(
          ImVec2(mid.x - shaftT, axleF),
          ImVec2(mid.x + shaftT, axleR),
          WIRE_VOID,
          0.0f
      );

      if(g > 0.0f)
      {
          const Float32 fillTop = axleR - ((axleR - axleF) * g);
          dl->AddRectFilled(
              ImVec2(mid.x - shaftT, fillTop),
              ImVec2(mid.x + shaftT, axleR),
              ui::ansi::BRGREEN,
              0.0f
          );
      }

      dl->AddRect(
          ImVec2(mid.x - shaftT, axleF),
          ImVec2(mid.x + shaftT, axleR),
          armed ? ui::ansi::BRGREEN : frame,
          0.0f,
          0,
          WIRE_W
      );

      // ---- the two axle beams ---------------------------------------------
      const auto beam = [&](Float32 y)
      {
          dl->AddRectFilled(
              ImVec2(mid.x - track, y - beamT),
              ImVec2(mid.x + track, y + beamT),
              WIRE_VOID,
              0.0f
          );
          dl->AddRect(
              ImVec2(mid.x - track, y - beamT),
              ImVec2(mid.x + track, y + beamT),
              frame,
              0.0f,
              0,
              WIRE_W
          );
      };
      beam(axleF);
      beam(axleR);

      // A hub at each end, so a wheel reads as mounted on the beam. Eight segments,
      // not twelve: the facets are visible, which is what a wireframe looked like.
      const auto hub = [&](Float32 x, Float32 y)
      {
          dl->AddCircleFilled(ImVec2(x, y), beamT * 1.15f, WIRE_VOID, 8);
          dl->AddCircle(ImVec2(x, y), beamT * 1.15f, frame, 8, WIRE_W);
      };

      // ---- wheels -----------------------------------------------------------
      const Float32 deg = steerN * CHASSIS_LOCK_DEG;

      // Front: bright while the servo holds them, gray when released - a released
      // servo holds nothing, so drawing them straight would invent a fact.
      const ImU32 frontFill = WIRE_VOID;
      const ImU32 frontEdge = servoLive ? ui::ansi::BRCYAN : ui::ansi::IDLE;

      drawWheel(
          dl,
          ImVec2(mid.x - track, axleF),
          wheelW,
          wheelH,
          deg,
          frontFill,
          frontEdge
      );
      drawWheel(
          dl,
          ImVec2(mid.x + track, axleF),
          wheelW,
          wheelH,
          deg,
          frontFill,
          frontEdge
      );
      hub(mid.x - track, axleF);
      hub(mid.x + track, axleF);

      // Rear: the driven pair. They warm rather than spin - nothing here measures
      // speed, so brightness is a COMMAND and not a reading.
      const ImU32 rearFill = WIRE_VOID;
      const ImU32 rearEdge = armed ? ui::ansi::BRGREEN : ui::ansi::IDLE;

      drawWheel(
          dl,
          ImVec2(mid.x - track, axleR),
          wheelW,
          wheelH,
          0.0f,
          rearFill,
          rearEdge
      );
      drawWheel(
          dl,
          ImVec2(mid.x + track, axleR),
          wheelW,
          wheelH,
          0.0f,
          rearFill,
          rearEdge
      );
      hub(mid.x - track, axleR);
      hub(mid.x + track, axleR);

      // ---- labels -----------------------------------------------------------
      const ImU32 faint = ui::ansi::IDLE;

      const Char* const FRONT = "FRONT";
      const Char* const REAR = "REAR";
      dl->AddText(
          ImVec2(mid.x - (ImGui::CalcTextSize(FRONT).x * 0.5f), p0.y + (6.0f * uiDpiScale)),
          faint,
          FRONT
      );
      dl->AddText(
          ImVec2(mid.x - (ImGui::CalcTextSize(REAR).x * 0.5f), p1.y - (20.0f * uiDpiScale)),
          faint,
          REAR
      );

      // The angle beside the front axle: a number and a picture of the same
      // thing, because one is checkable and the other is fast.
      Array<Char, 24> deglabel;
      std::snprintf(deglabel.data(), deglabel.size(), "%+.0f deg", static_cast<Float64>(deg));
      dl->AddText(
          ImVec2(mid.x + track + (wheelW * 2.6f), axleF - (7.0f * uiDpiScale)),
          servoLive ? ui::ansi::BRYELLOW : faint,
          deglabel.data()
      );

      // A hard border rather than the bevelled inset the rest of the panel uses.
      // The inset is a molding, and this is a screen.
      dl->AddRect(p0, p1, ui::ansi::GRID, 0.0f, 0, WIRE_W);
  }

  // ============================================== the drive view ==
  // Steering on GP0 and the ESC on GP1, with sliders instead of typed numbers.
  // EVERY LIMIT HERE COMES FROM THE BOARD: the sliders are built from the range
  // the firmware reports, so safety survives the hub disconnecting.
  Void drawDriveBody(Float32 w, Float32 h)
  {
      // Scrolls with the wheel. Do NOT copy ImGuiWindowFlags_NoScrollWithMouse
      // from the map views - it is right there because the wheel ZOOMS.
      ImGui::BeginChild("##drive", ImVec2(w, h), ImGuiChildFlags_None, ImGuiWindowFlags_None);

      // The cut-out this view sits in, recessed like the other views. The bezel is
      // drawn LAST, over the content, so it shadows the top edge.
      const ImVec2 drivePanel0 = ImGui::GetCursorScreenPos();

      const Bool live = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

      if(!live)
      {
          driveServoHeld = false;
          driveEscHeld = false;
          driveKnown = false;

          const PicoState st = picoLink.state();

          ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED), "Pico not connected.");
          ui::screenInset(drivePanel0, ImVec2(drivePanel0.x + w, drivePanel0.y + h));

          // A board that is PRESENT but will not talk is a different problem from
          // a board that is not there, and the fix for it is not "try again".
          if(st == PicoState::PICO_STATE_ERROR)
          {
              ImGui::Spacing();
              ImGui::TextColored(
                  ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                  "The port is there and will not open."
              );
              ImGui::TextColored(
                  ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                  "Most often this is the firmware, not the cable.\n" "\n" "A sketch that never opens a serial console still\n" "enumerates a COM port - Windows keeps offering it,\n" "and every attempt to open it hangs. From out here\n" "that is indistinguishable from a dead board.\n" "\n" "This view needs the Debug / Blink firmware\n" "(firmware/app/main.c). Flashing a sketch from the\n" "Code view REPLACES it - they are separate programs\n" "and only one can be on the board at a time."
              );

              const Str err = picoLink.error();
              if(!err.empty())
              {
                  ImGui::TextColored(
                      ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                      " %s",
                      err.c_str()
                  );
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
          // There are TWO images: whatever is open in the Code view, and
          // `pico_debug`, which answers these commands. Build & Flash on a sketch
          // replaces pico_debug and every board view goes quiet.
          const Float64 nowAsk = ImGui::GetTime();
          if(nowAsk - driveAskedAt > 1.0)
          {
              driveAskedAt = nowAsk;
              pollPico("DRIVE");
          }

          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
              "The board is not answering drive commands."
          );
          ImGui::Spacing();
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              "This view needs the Debug / Blink firmware " "(firmware/app/main.c).\n" "\n" "If you have flashed a sketch from the Code view, " "that REPLACED it -\n" "the two are separate programs and only one can be " "on the board at a time."
          );
          ImGui::Spacing();

          drawDriveFlashButton();

          ui::screenInset(drivePanel0, ImVec2(drivePanel0.x + w, drivePanel0.y + h));
          ImGui::EndChild();
          return;
      }

      // Poll while the view is open, rather than subscribing on the board: the
      // traffic stops dead when the view is not on screen.
      const Float64 nowPoll = ImGui::GetTime();
      if(nowPoll - driveAskedAt > 0.25)
      {
          driveAskedAt = nowPoll;
          pollPico("DRIVE");
      }

      // The sweep runs here so it stops the moment the view is not drawn: a servo
      // cycling behind a tab nobody is looking at must not be possible.
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
                  driveSweepDir = -1;
              }
              else if(driveServoWant <= driveServoMin)
              {
                  driveServoWant = driveServoMin;
                  driveSweepDir = 1;
              }
              Array<Char, 32> cmd;
              std::snprintf(cmd.data(), cmd.size(), "SERVO %d", driveServoWant);
              sendPico(cmd.data());
          }
      }

      ImGui::Spacing();

      // ---- the car ---------------------------------------------------------
      // What the numbers below already say, in a form readable without converting.
      {
          // The well is NARROWER than the panel and centered: full width leaves the
          // car adrift in a meter of black, which reads as a rendering fault.
          const Float32 full = ImGui::GetContentRegionAvail().x;
          const Float32 want = 460.0f * uiDpiScale;
          const Float32 caw = (full < want) ? full : want;
          const Float32 cah = 250.0f * uiDpiScale;

          const ImVec2 here = ImGui::GetCursorScreenPos();
          const ImVec2 cp0(here.x + ((full - caw) * 0.5f), here.y);

          // The throttle as a fraction of ITS range. The TARGET rather than the
          // output - the bar below shows what is actually being given.
          const Float32 power = throttleFraction(driveEscT);

          drawChassis(
              ImGui::GetWindowDrawList(),
              ChassisFrame{ cp0, caw, cah, driveSteerNow, power, driveServoOn, driveArmed },
              boardLamp,
              boardLampPin
          );

          ImGui::Dummy(ImVec2(full, cah));
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "The car from above, drawn from what the BOARD reports.\n" "\n" "Front wheels turn with the steering. They go dark when the\n" "servo is released, because a released servo holds nothing and\n" "the wheels are wherever the ground last left them - drawing\n" "them straight would be the display inventing a fact.\n" "\n" "Rear wheels warm with throttle. They do not spin: nothing on\n" "this car measures speed yet, so brightness is a COMMAND and\n" "not a reading, and that difference is worth being able to see."
              );
          }
      }

      ImGui::Spacing();

      // ---- steering --------------------------------------------------------
      driveSection("Steering", "GP0");

      // Engaging is a deliberate act, like arming the ESC. The board comes up
      // released: with the horn a tooth out, 1500 us picks up the frame.
      if(driveServoOn)
      {
          ui::pushTint(ui::Tint::TINT_WARN);
          if(ui::iconButton(
              ui::Icon::ICON_MOTOR_STOP,
              "Release the servo",
              ImVec2(240.0f * uiDpiScale, 0.0f)
          ))
          {
              driveSweep = false;
              sendPico("SERVO OFF");
          }
          ui::popTint(ui::Tint::TINT_WARN);
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "Stops the pulse. The servo goes limp - no holding torque, no\n" "current, nothing to push against.\n" "\n" "This is the thing to reach for when it is leaning on the\n" "frame. A better number will not help; not being asked to hold\n" "a position at all is what helps."
              );
          }

          ImGui::SameLine();
          driveLamp(true, ui::sem::WARN, "driving");
      }

      else
      {
          if(ui::iconButton(
              ui::Icon::ICON_MOTOR_RUN,
              "Engage the servo",
              ImVec2(240.0f * uiDpiScale, 0.0f)
          ))
          {
              sendPico("SERVO ON");
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "Starts driving GP0, from neutral, slewed.\n" "\n" "Everything below is remembered while released and takes\n" "effect when you engage - so you can set a value first and\n" "then commit to it, rather than the other way round."
              );
          }

          ImGui::SameLine();
          driveLamp(false, ui::sem::MUTED, "released  -  the servo is limp");

          // A band across the controls it applies to, because what is explained is
          // that ALL of them are inert. ONE line, not a bordered placard.
          const ImVec2  a = ImGui::GetCursorScreenPos();
          const Float32 bh = ImGui::GetTextLineHeight() + 6.0f * uiDpiScale;
          ImDrawList*   dl = ImGui::GetWindowDrawList();
          dl->AddRectFilled(a, ImVec2(a.x + 3.0f * uiDpiScale, a.y + bh), ui::sem::WARN, 0.0f);

          ImGui::Dummy(ImVec2(0.0f, 3.0f * uiDpiScale));
          ImGui::Indent(10.0f);
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
              "Released - the controls below set a target only."
          );
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "The slider and the steps write a target that the board\n" "remembers. Nothing reaches the servo until you engage it,\n" "at which point it walks to that target at the Response\n" "rate rather than jumping."
              );
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
          Array<Char, 32> cmd;
          std::snprintf(cmd.data(), cmd.size(), "STEER %.3f", static_cast<Float64>(driveSteerWant));
          sendPico(cmd.data());
      }
      driveSteerHeld = ImGui::IsItemActive();
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "-1 is full lock one way, +1 the other, 0 is wheels straight.\n" "\n" "The two sides are scaled SEPARATELY, from the calibration. This\n" "car throws %d us one way from center and %d the other, so half\n" "left and half right are not the same number of microseconds -\n" "and anything that added a fixed amount to a midpoint would pull\n" "to one side every time it was asked for half.\n" "\n" "This is the command the autonomy layer should use. Nothing above\n" "the calibration needs to know what a microsecond is.",
              driveServoC - driveServoMin,
              driveServoMax - driveServoC
          );
      }

      ImGui::SameLine();
      if(ui::button("Straight", ImVec2(-FLT_MIN, 0.0f)))
      {
          driveSweep = false;
          driveSteerWant = 0.0f;
          sendPico("STEER 0");
      }

      // ---- how fast anything is allowed to move -------------------------
      // A slider rather than presets: the useful rate is found by moving it and
      // watching. LOGARITHMIC, because 1 to 20 is where "creeps" becomes "moves".
      {
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              "Response - how fast the servo may move"
          );

          // The reading goes on the HEAD's line rather than beside the slider,
          // which is what lets the slider run to the shared right edge.
          {
              const Int32 perSec = driveSlew * 50;
              const Int32 travel = driveServoMax - driveServoMin;

              Array<Char, 64> r;
              std::snprintf(
                  r.data(),
                  r.size(),
                  "%d us/s lock to lock %.2f s",
                  perSec,
                  (perSec > 0) ? (static_cast<Float64>(travel) / perSec) : 0.0
              );
              driveReading(ui::sem::MUTED, r.data());
          }

          ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
          if(ImGui::SliderInt(
              "##slew",
              &driveSlewWant,
              1,
              200,
              "%d us/tick",
              ImGuiSliderFlags_Logarithmic
          ))
          {
              Array<Char, 32> cmd;
              std::snprintf(cmd.data(), cmd.size(), "SLEW STEER %d", driveSlewWant);
              sendPico(cmd.data());
          }
          driveSlewHeld = ImGui::IsItemActive();

          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "Microseconds of pulse the board may move an output, per\n" "20 ms tick. It governs the steering AND the throttle.\n" "\n" "The scale is logarithmic: the interesting range is the\n" "bottom, where the difference between creeping and moving\n" "lives, and a linear track would bury it in the first tenth.\n" "\n" " ~2 creeps - slow enough to stop the moment a linkage\n" " binds, which is what finding an end stop wants\n" " 8 the default. A slider dragged end to end sweeps\n" " rather than flinging the servo at a stop\n" " ~40 lock to lock in about a fifth of a second, quick\n" " enough to correct a line\n" " 200 faster than the servo can physically follow, so the\n" " limit stops being this software and starts being\n" " the hardware\n" "\n" "Not saved by itself - Write to firmware, under Throttle\n" "range, is what keeps it across a reflash."
              );
          }

      }


      // ---- the calibration -------------------------------------------------
      // Three named points instead of a min/max pair: a car has three interesting
      // positions and only two are ends. Center is a MEASUREMENT - assuming it is
      // 1500 is what put a servo against a frame.
      if(ImGui::TreeNode("Calibration  -  the three numbers for THIS car"))
      {
          if(!calLoaded)
          {
              loadCalibration();
          }

      ImGui::Spacing();
      ImGui::TextColored(
          ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
          "Raw microseconds - for calibrating, not for driving"
      );

      ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
      if(ImGui::SliderInt("##servo", &driveServoWant, driveServoMin, driveServoMax, "%d us"))
      {
          Array<Char, 32> cmd;
          std::snprintf(cmd.data(), cmd.size(), "SERVO %d", driveServoWant);
          sendPico(cmd.data());
      }
      driveServoHeld = ImGui::IsItemActive();
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "1500 us is center. The range is %d-%d, which the BOARD sets.\n" "\n" "Deliberately narrower than the servo's own 1000-2000: a TT-02's\n" "steering binds against its linkage well before the servo's limits,\n" "and a servo pushing a stop stalls and cooks itself.\n" "\n" "Widen it in firmware once the real end stops are known - with the\n" "servo horn OFF, so being wrong costs nothing.",
              driveServoMin,
              driveServoMax
          );
      }

      ImGui::SameLine();
      if(ui::button("Center", ImVec2(-FLT_MIN, 0.0f)))
      {
          driveSweep = false;
          driveServoWant = 1500;
          sendPico("SERVO CENTER");
      }

      // ---- exact values, and steps --------------------------------------
      // A slider feels out where things are; a number says exactly where.
      const auto nudge = [](Int32 by)
      {
          driveSweep = false;
          driveServoWant += by;
          driveServoWant = clampInt(driveServoWant, driveServoMin, driveServoMax);
          Array<Char, 32> cmd;
          std::snprintf(cmd.data(), cmd.size(), "SERVO %d", driveServoWant);
          sendPico(cmd.data());
      };

      const Float32 bw = DRIVE_STEP_W * uiDpiScale;
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
              ImGui::SetTooltip(
                  "Sets the target. The servo is released, so\n" "nothing moves until you engage it."
              );
          }
      }

      ImGui::SameLine(0.0f, 12.0f);
      ImGui::BeginDisabled(!driveServoOn);
      if(ui::segmentedIconButton(
          driveSweep ? ui::Icon::ICON_PAUSE : ui::Icon::ICON_PLAY,
          "Sweep",
          driveSweep,
          ImVec2(110.0f * uiDpiScale, 0.0f)
      ))
      {
          driveSweep = !driveSweep;
          driveSweepNext = ImGui::GetTime();
      }
      ImGui::EndDisabled();
      if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      {
          ImGui::SetTooltip(
              "Walks the servo between its limits, back and forth.\n" "\n" "For watching the linkage move through its whole travel and seeing\n" "whether anything binds or fouls before you trust a number.\n" "\n" "Driven from the hub, not the board, and it stops the moment this\n" "view is not on screen - a servo cycling behind a tab nobody is\n" "looking at is exactly what should not be possible."
          );
      }

      // ONE status line for the whole steering section: what the board is actually
      // OUTPUTTING, which lags the slider. Showing both makes the ramp visible.
      {
          Array<Char, 96> st;
          if(driveServoOn)
          {
              std::snprintf(
                  st.data(),
                  st.size(),
                  "%+.2f %d us target %d us",
                  static_cast<Float64>(driveSteer),
                  driveServo,
                  driveServoT
              );
          }
          else
          {
              std::snprintf(
                  st.data(),
                  st.size(),
                  "%+.2f target %d us not driven",
                  static_cast<Float64>(driveSteer),
                  driveServoT
              );
          }
          ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED), "%s", st.data());
      }


          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
              "Servo horn OFF while you find these."
          );
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              "Engage, step until the output shaft reaches each end,\n" "and press Set to here. With the horn off, being wrong\n" "costs nothing at all."
          );
          ImGui::Spacing();

          calRow(
              "Max left",
              "Full lock one way. Where the STEERING stops, which is\n" "not where the servo stops - the linkage binds first,\n" "and the servo will happily keep pushing past it.",
              &calLeft
          );
          calRow(
              "Center",
              "Wheels straight ahead.\n" "\n" "The one that matters most and the one nobody measures.\n" "1500 us is the middle of the SERVO's range and says\n" "nothing about the CAR's - the horn only meets its\n" "spline at whole-tooth intervals, so straight-ahead\n" "lands wherever it lands.",
              &calCenter
          );
          calRow("Max right", "Full lock the other way.", &calRight);

          ImGui::Spacing();

          const Bool ordered = (calLeft < calCenter && calCenter < calRight);
          if(!ordered)
          {
              ImGui::TextColored(
                  ImGui::ColorConvertU32ToFloat4(ui::sem::BAD),
                  "Left must be below center, and center below right."
              );
              ImGui::TextColored(
                  ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                  "If your steering runs the other way, swap which end\n" "you call left - the firmware only needs the order."
              );
          }

          ImGui::BeginDisabled(!ordered || !driveKnown);
          if(ui::iconButton(
              ui::Icon::ICON_SEND,
              "Send to the board",
              ImVec2(200.0f * uiDpiScale, 0.0f),
              ui::Tint::TINT_WARN
          ))
          {
              Array<Char, 48> cmd;
              std::snprintf(cmd.data(), cmd.size(), "SERVOLIMITS %d %d", calLeft, calRight);
              sendPico(cmd.data());
              std::snprintf(cmd.data(), cmd.size(), "SERVOTRIM %d", calCenter);
              sendPico(cmd.data());
          }
          ImGui::EndDisabled();
          if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
          {
              ImGui::SetTooltip(
                  "Applies these to the RUNNING board so you can try\n" "them immediately.\n" "\n" "Lost on the next reboot - Write to firmware is what\n" "makes them stick."
              );
          }

          ImGui::SameLine();
          ImGui::BeginDisabled(!ordered);
          if(ui::iconButton(
              ui::Icon::ICON_SAVE,
              "Write to firmware",
              ImVec2(210.0f * uiDpiScale, 0.0f)
          ))
          {
              const Str path = steeringCalPath();
              const Str text = steeringCalText();
              Str       err;
              if(text.empty())
              {
                  // steeringCalText() has already said why. Writing here would
                  // put a truncated header on disk, which is how this was found.
                  LOG_WARN("drive", "refusing to write an incomplete cal.hxx");
              }
              else if(path.empty())
              {
                  LOG_WARN("drive", "no firmware directory - nothing written");
              }
              else if(sketch::save(path, text, err))
              {
                  calWritten = text;
                  calDirty = false;
                  LOG_INFO("drive", "wrote %s", path.c_str());
              }
              else
              {
                  LOG_WARN("drive", "could not write cal.h: %s", err.c_str());
              }
          }
          ImGui::EndDisabled();
          if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
          {
              ImGui::SetTooltip(
                  "Writes firmware/lib/chassis/cal.h - the three\n" "steering numbers AND the throttle range the\n" "board is currently using.\n" "\n" "main.c includes it, so these become the limits and\n" "the center the board comes up with. Reflash after\n" "writing or the board keeps running the old ones."
              );
          }

          // Whether what is on screen matches what the firmware would build with.
          // Silence would make "written" and "typed but not written" identical.
          {
              Int32 fl = 0;
              Int32 fc = 0;
              Int32 fr = 0;
              if(calWritten.empty()
                 || !readCalNumbers(calWritten, fl, fc, fr))
              {
                  ImGui::TextColored(
                      ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                      "cal.h could not be read."
                  );
              }
              else if(fl != calLeft || fc != calCenter || fr != calRight)
              {
                  ImGui::TextColored(
                      ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                      "cal.h still says %d / %d / %d.",
                      fl,
                      fc,
                      fr
                  );
              }
              else if(Int32 tl = 0, th = 0;
                      !readThrottleNumbers(calWritten, tl, th)
                      || tl != driveEscMin || th != driveEscMax)
              {
                  // The steering matches and the throttle does not - what a header
                  // written before the throttle was persisted looks like.
                  ImGui::TextColored(
                      ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
                      "Steering matches; the throttle range in " "cal.h does not."
                  );
              }
              else
              {
                  ImGui::TextColored(
                      ImGui::ColorConvertU32ToFloat4(ui::sem::GOOD),
                      "Matches cal.h - the firmware would " "build with these."
                  );
              }
          }

          if(ui::iconButton(ui::Icon::ICON_CODE, "Open cal.h", ImVec2(210.0f * uiDpiScale, 0.0f)))
          {
              const Str path = steeringCalPath();
              if(!path.empty())
              {
                  openCodeFile(path, "cal.h");
                  forceView = 3;
                  forceViewFrames = 4;
              }
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "Opens the generated header in the Code view, so\n" "the numbers are readable as code rather than only\n" "as boxes in a panel."
              );
          }

          if(driveKnown)
          {
              ImGui::TextColored(
                  ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
                  "The board is currently using %d / %d / %d.",
                  driveServoMin,
                  driveServoC,
                  driveServoMax
              );
          }

          ImGui::TreePop();
      }

      // ---- the raw limits, still reachable -------------------------------
      // Finding an end stop means pushing PAST where you think it is, and the
      // calibration above clamps to what the board already accepts.
      if(ImGui::TreeNode("Limits  -  widen to find the real end stops"))
      {
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
              "Take the servo horn OFF first."
          );
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              "The linkage binds before the servo's own limits do,\n" "and a servo pushing a stop stalls and cooks itself.\n" "With the horn off, being wrong costs nothing."
          );
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
          if(ui::iconButton(
              ui::Icon::ICON_SAVE,
              "Apply to the board",
              ImVec2(220.0f * uiDpiScale, 0.0f),
              ui::Tint::TINT_WARN
          ))
          {
              Array<Char, 48> cmd;
              std::snprintf(
                  cmd.data(),
                  cmd.size(),
                  "SERVOLIMITS %d %d",
                  driveLimitLo,
                  driveLimitHi
              );
              sendPico(cmd.data());
              driveLimitsDirty = false;
          }
          ImGui::EndDisabled();

          ImGui::SameLine();
          if(ui::button("Reset", ImVec2(100.0f * uiDpiScale, 0.0f)))
          {
              sendPico("SERVOLIMITS 1300 1700");
              driveLimitsDirty = false;
          }

          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              "The board clamps to 1000-2000 whatever is asked, and\n" "pulls the current target and center back inside a\n" "narrowed range.\n" "\n" "Widen here to go looking; record what you find in the\n" "Calibration block above, which is what gets written\n" "into the firmware."
          );
          ImGui::TreePop();
      }

      ImGui::Spacing();
      driveSection("Throttle", "GP1  (ESC)");

      // The percentage belongs up here with the other readings, not stranded
      // under the power bar where it was the only thing on its line.
      {
          const Float32 frac = throttleFraction(driveEsc);

          Array<Char, 32> r;
          std::snprintf(
              r.data(),
              r.size(),
              "%.0f%%",
              static_cast<Float64>((driveArmed ? frac : 0.0f) * 100.0f)
          );
          driveReading(driveArmed ? ui::sem::WARN : ui::sem::MUTED, r.data());
      }

      // Arming is a separate, deliberate act. The BOARD refuses throttle commands
      // regardless of what this checkbox says - this is the reminder, not the law.
      Bool arm = driveArmed;
      if(ui::checkbox("Arm the ESC", &arm))
      {
          sendPico(arm ? "ESC ARM" : "ESC DISARM");
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "The ESC ignores every throttle command until this is on.\n" "\n" "BEFORE ARMING:\n" " - the car on a stand, wheels off the ground\n" " - common ground between the Pico and the ESC (mandatory)\n" " - the BEC 5 V NOT connected while USB is\n" "\n" "Enforced on the BOARD, not here. This checkbox is the reminder."
          );
      }

      ImGui::BeginDisabled(!driveArmed);
      ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
      if(ImGui::SliderInt("##esc", &driveEscWant, driveEscMin, driveEscMax, "%d us"))
      {
          Array<Char, 32> cmd;
          std::snprintf(cmd.data(), cmd.size(), "ESC %d", driveEscWant);
          sendPico(cmd.data());
      }
      driveEscHeld = ImGui::IsItemActive();
      ImGui::EndDisabled();

      if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      {
          ImGui::SetTooltip(
              "1500 us is neutral. The range is %d-%d - forward only, and barely.\n" "\n" "1600 is a crawl on a bench. Reverse is not offered at all: a\n" "QuicRun needs a brake-then-reverse sequence, and getting that\n" "wrong on a stand is how a gearbox meets a workbench.\n" "\n" "The board ramps toward whatever you set rather than jumping to it.",
              driveEscMin,
              driveEscMax
          );
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
          driveEscWant = clampInt(driveEscWant, driveEscMin, driveEscMax);
          Array<Char, 32> cmd;
          std::snprintf(cmd.data(), cmd.size(), "ESC %d", driveEscWant);
          sendPico(cmd.data());
      };

      // A power bar, because a microsecond figure is not a sense of how much.
      // Segmented: ten blocks say "about a third" without implying more.
      {
          const Float32 barW = ImGui::GetContentRegionAvail().x - (120.0f * uiDpiScale);
          const Float32 barH = 10.0f * uiDpiScale;
          const ImVec2  b0 = ImGui::GetCursorScreenPos();
          ImDrawList*   dl = ImGui::GetWindowDrawList();

          const Float32 frac = throttleFraction(driveEsc);

          dl->AddRectFilled(
              b0,
              ImVec2(b0.x + barW, b0.y + barH),
              IM_COL32(0x14, 0x15, 0x18, 0xFF),
              2.0f
          );

          constexpr Int32 SEGS = 10;
          const Float32 segW = (barW - (2.0f * uiDpiScale) * (SEGS - 1)) / SEGS;
          for(Int32 i = 0; i < SEGS; ++i)
          {
              const Float32 at = static_cast<Float32>(i + 1) / SEGS;
              const Bool    on = driveArmed && (frac >= at - (0.5f / SEGS));
              const Float32 x = b0.x + (i * (segW + 2.0f * uiDpiScale));

              // The top of the range is amber: the last blocks are the ones worth
              // noticing before pressing anything else.
              const ImU32 lit = (i >= SEGS - 3) ? ui::sem::WARN : ui::sem::GOOD;
              dl->AddRectFilled(
                  ImVec2(x, b0.y),
                  ImVec2(x + segW, b0.y + barH),
                  on ? lit : IM_COL32(0x26, 0x28, 0x2C, 0xFF),
                  1.0f
              );
          }
          ui::screenInset(b0, ImVec2(b0.x + barW, b0.y + barH), 0.7f);

          ImGui::Dummy(ImVec2(barW, barH));
      }

      const Float32 escBw = DRIVE_STEP_W * uiDpiScale;

      ImGui::BeginDisabled(!driveArmed);
      for(Size i = 0; i < countOf(ESC_STEPS); ++i)
      {
          if(i > 0)
          {
              ImGui::SameLine(0.0f, 3.0f);
          }
          // Suffixed: several of these labels repeat the servo row's, and ImGui
          // derives a widget's identity from its label.
          if(ui::button(ESC_STEPS[i].label, ImVec2(escBw, 0.0f)))
          {
              nudgeEsc(ESC_STEPS[i].by);
          }
      }
      ImGui::EndDisabled();

      {
          Array<Char, 64> line;
          std::snprintf(
              line.data(),
              line.size(),
              "output %d us target %d us %s",
              driveEsc,
              driveEscT,
              driveArmed ? "ARMED" : "disarmed"
          );
          driveLamp(driveArmed, driveArmed ? ui::sem::BAD : ui::sem::MUTED, line.data());
      }

      // Response belongs WITH the thing it governs, not in a settings pile at the
      // top of the page.
      ImGui::Spacing();
      ImGui::TextColored(
          ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
          "Response - how fast the ESC may move"
      );

      {
          const Int32 perSec = driveEscSlew * 50;
          const Int32 span = driveEscMax - driveEscMin;

          Array<Char, 64> r;
          std::snprintf(
              r.data(),
              r.size(),
              "%d us/s idle to full %.2f s",
              perSec,
              (perSec > 0) ? (static_cast<Float64>(span) / perSec) : 0.0
          );
          driveReading(ui::sem::MUTED, r.data());
      }

      ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
      if(ImGui::SliderInt(
          "##slewesc",
          &driveEscSlewWant,
          1,
          200,
          "%d us/tick",
          ImGuiSliderFlags_Logarithmic
      ))
      {
          Array<Char, 40> cmd;
          std::snprintf(cmd.data(), cmd.size(), "SLEW THROTTLE %d", driveEscSlewWant);
          sendPico(cmd.data());
      }
      driveEscSlewHeld = ImGui::IsItemActive();

      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "How fast the ESC's pulse may move, per 20 ms tick. Separate\n" "from the steering, and it should be: a servo wants to arrive\n" "promptly, an ESC wants to be led there.\n" "\n" "This is the one that decides whether the car pulls away or\n" "lurches. Throttle slammed on spins the wheels; slammed off\n" "pitches the car onto its nose; and a brushed motor asked for\n" "a step change draws a spike the BEC feels.\n" "\n" "The usable range is only %d us wide (%d to %d), so a rate\n" "that feels gentle on the steering's %d us of travel crosses\n" "the whole throttle band in a fraction of the time.\n" "\n" "Not saved by itself - Write to firmware, under Throttle\n" "range, is what keeps it across a reflash.",
              driveEscMax - driveEscMin,
              driveEscMin,
              driveEscMax,
              calRight - calLeft
          );
      }

      if(ImGui::TreeNode("Throttle range  -  widen once the car is on a stand"))
      {
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::WARN),
              "Wheels off the ground before touching this."
          );
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              "The board will not go above 1700 whatever is asked,\n" "and reverse stays unreachable - a QuicRun needs a\n" "brake-then-reverse sequence, and getting that wrong on\n" "a stand is how a gearbox meets a workbench."
          );
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
          if(ui::iconButton(
              ui::Icon::ICON_SAVE,
              "Apply to the board##esc",
              ImVec2(220.0f * uiDpiScale, 0.0f),
              ui::Tint::TINT_WARN
          ))
          {
              Array<Char, 48> cmd;
              std::snprintf(
                  cmd.data(),
                  cmd.size(),
                  "ESCLIMITS %d %d",
                  driveEscLimitLo,
                  driveEscLimitHi
              );
              sendPico(cmd.data());
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

      // ---- keyboard drive --------------------------------------------------
      driveSection("Keyboard", "WASD, hold to drive");

      {
          const Bool ready = driveServoOn && driveArmed;

          if(ui::checkbox("Drive with WASD", &wasdOn))
          {
              // Turning it OFF must put the car down, not just stop listening.
              if(!wasdOn)
              {
                  wasdSentSteer = 0;
                  wasdSentEsc = 0;
                  sendPico("STEER 0");
                  sendPico("ESC NEUTRAL");
              }
          }
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "A / D steer to FULL LOCK while held\n" "W forward at the cap below\n" "S brake to neutral\n" "\n" "Hold to drive - releasing a key stops that axis. The keys do\n" "nothing while you are typing in the console, the code editor,\n" "or any text field, and nothing while the link is down.\n" "\n" "S is a BRAKE, not reverse. The drivetrain is forward-only and\n" "the throttle clamp turns any number below idle INTO idle, so\n" "there is no value that means backwards.\n" "\n" "The board stops itself if this window stops sending for 400 ms,\n" "which is what covers the app being minimised, hung or closed."
              );
          }

          // What is missing, named: enabling this does NOT arm the ESC or engage
          // the servo. SameLine only when there IS something to put there.
          if(!ready || wasdOn)
          {
              ImGui::SameLine();
          }
          if(!ready)
          {
              const Char* missing = (!driveServoOn && !driveArmed)
                                        ? "servo released, ESC disarmed"
                                    : (!driveServoOn) ? "servo released"
                                                      : "ESC disarmed";
              colored(ui::sem::WARN, "%s", missing);
          }
          else if(wasdOn)
          {
              colored(ui::sem::GOOD, "live");
          }

          ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED), "Forward cap");

          {
              Array<Char, 48> r;
              std::snprintf(r.data(), r.size(), "%d us above idle", wasdCapNow() - driveEscMin);
              driveReading(ui::sem::MUTED, r.data());
          }

          ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
          ImGui::SliderInt("##wasdcap", &wasdCapUs, driveEscMin, wasdCapCeil(), "%d us");
          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "How much throttle W asks for.\n" "\n" "Runs from the measured idle of %d up to %d and STOPS there -\n" "the ceiling is the slider's own end, not a default you can\n" "slide past. Nine microseconds, about a sixth of this car's\n" "%d us band.\n" "\n" "It is low because the keyboard is digital: W is fully on or\n" "fully off, with none of the feathering a trigger gives you, so\n" "the only thing keeping it sane is the number it slams to.\n" "\n" "Raising the ceiling is a code change, on purpose. Nothing you\n" "can do here while the car is moving raises it.",
                  driveEscMin,
                  wasdCapCeil(),
                  driveEscMax - driveEscMin
              );
          }
      }

      // ---- lights ----------------------------------------------------------
      // Its own section, at the end, not interleaved with the drivetrain.
      driveSection("Lights", "what the lamps do");

      // ---- when the tail lamps go out ------------------------------------
      // The same KIND of thing as Response: a judgment about where a boundary
      // sits. Idle is the pulse at which the motor sits still; this is how far
      // past that counts as going somewhere.
      {
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              "Tail lamps - how much throttle counts as moving"
          );

          {
              const Bool lit = (boardLamp[2] > 0) || (boardLamp[3] > 0);
              driveReading(lit ? ui::sem::BAD : ui::sem::MUTED, lit ? "lit" : "dark");
          }

          ImGui::SetNextItemWidth(-DRIVE_TAIL_W * uiDpiScale);
          if(ImGui::SliderInt("##lightsoff", &lightsOffWant, 0, 60, "%d us past idle"))
          {
              Array<Char, 40> cmd;
              std::snprintf(cmd.data(), cmd.size(), "LIGHTS OFFAT %d", lightsOffWant);
              sendPico(cmd.data());
          }
          lightsOffHeld = ImGui::IsItemActive();

          if(ImGui::IsItemHovered())
          {
              ImGui::SetTooltip(
                  "The tail lamps are lit whenever the car is not being driven,\n" "and go out once the throttle clears idle by this much.\n" "\n" "Zero means the lamps go out the instant the throttle leaves\n" "idle, which lights them off for a car that has not really\n" "pulled away. Wind it up until they stay on at a crawl and go\n" "out when the car actually goes somewhere.\n" "\n" "Mirrored for reverse: this far BELOW neutral counts as being\n" "driven backwards, and lights the reverse lamps instead.\n" "\n" "Idle here is %d us and full throttle is %d, so the whole\n" "usable range is %d us wide.\n" "\n" "Not saved to the board's calibration by itself - Write to\n" "firmware, under Throttle range, is what makes it stick.",
                  driveEscMin,
                  driveEscMax,
                  driveEscMax - driveEscMin
              );
          }

      }

      ImGui::Spacing();

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::Spacing();

      ImGui::TextColored(
          ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
          "Nothing here jumps. The board walks each output toward\n" "its target a few microseconds at a time, so a slider\n" "dragged end to end produces a sweep rather than a step."
      );

      // ---- the wiring this view assumes ----------------------------------
      // A signal wire in the wrong hole looks exactly like firmware that does not
      // work, and the two are debugged very differently.
      if(ImGui::TreeNode("Wiring"))
      {
          ImGui::TextColored(
              ImGui::ColorConvertU32ToFloat4(ui::sem::MUTED),
              " servo signal -> GP0\n" " ESC signal -> GP1\n" " ESC ground -> a Pico GND (mandatory)\n" "\n" "The ESC's BEC 5 V goes NOWHERE while USB is plugged\n" "in - two supplies fighting over one rail is how a\n" "Pico stops being a Pico.\n" "\n" "Nothing else is assumed connected. This view does not\n" "need the display or the ToF sensor."
          );
          ImGui::TreePop();
      }

      ui::screenInset(drivePanel0, ImVec2(drivePanel0.x + w, drivePanel0.y + h));
      ImGui::EndChild();
  }

  // A .bdoc, drawn either as the page it describes or as the source that describes
  // it - the SAME editor and file underneath, so one click says which is wrong.
  Void drawDocPane(Float32 w, Float32 h)
  {
      ImGui::BeginChild(
          "##docpane",
          ImVec2(w, h),
          ImGuiChildFlags_None,
          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
      );

      // A different document starts on the page. Somebody opening a pinout wants
      // the pinout, and the last file's mode is not a preference about this one.
      if(docModeFor != codePath)
      {
          docModeFor = codePath;
          docModeSource = false;
      }

      // ---- the toggle -------------------------------------------------------
      const Float32 bh = ImGui::GetFrameHeight();

      if(ui::iconButton(
          ui::Icon::ICON_REFERENCE,
          "Page",
          ImVec2(84.0f * uiDpiScale, bh),
          docModeSource ? ui::Tint::TINT_NONE : ui::Tint::TINT_GOOD
      ))
      {
          docModeSource = false;
      }
      ImGui::SameLine();
      if(ui::iconButton(
          ui::Icon::ICON_CODE,
          "Source",
          ImVec2(94.0f * uiDpiScale, bh),
          docModeSource ? ui::Tint::TINT_GOOD : ui::Tint::TINT_NONE
      ))
      {
          docModeSource = true;
      }

      // Reparsed only when the text changes. That is what makes editing the
      // source and watching the page update work, and it costs nothing.
      if(docParsedFrom != codeEditor.text())
      {
          docParsedFrom = codeEditor.text();

          // The document's OWN folder, so <Include file="..."/> resolves like a C
          // include. BOTH separators: "\/" is just '/' and never matches here.
          Str base = codePath;
          const Size cut = base.find_last_of("\\/");
          base = (cut == Str::npos) ? Str() : base.substr(0, cut);

          docParsed = refdoc::parse(docParsedFrom, base);
      }

      ImGui::SameLine();
      if(!docParsed.ok())
      {
          colored(ui::sem::BAD, "line %d: %s", docParsed.errorLine, docParsed.error.c_str());
      }
      else
      {
          // Style complaints, counted rather than listed: they are not errors,
          // the document renders.
          const Vec<Str> notes = refdoc::check(docParsed);
          if(notes.empty())
          {
              colored(ui::sem::MUTED, "%s", codeName.c_str());
          }
          else
          {
              colored(
                  ui::sem::WARN,
                  "%d style note%s",
                  static_cast<Int32>(notes.size()),
                  notes.size() == 1 ? "" : "s"
              );
              if(ImGui::IsItemHovered())
              {
                  Str all;
                  for(const Str& n : notes)
                  {
                      all += n;
                      all += "\n";
                  }
                  ImGui::SetTooltip("%s", all.c_str());
              }
          }
      }

      ImGui::Separator();

      const Float32 rest = h - bh - (ImGui::GetStyle().ItemSpacing.y * 3.0f);

      if(docModeSource)
      {
          ui::drawCode(codeView, codeEditor, ImVec2(w, std::max(60.0f, rest)), ImGui::GetTime());
      }
      else
      {
          // One renderer, shared. The page's frame - scroll, Ctrl+wheel zoom, drag
          // to pan - lives in refdoc, so a document behaves the same everywhere.
          refdoc::drawPage(docParsed, ImVec2(w, std::max(60.0f, rest)), docView, uiDpiScale);
      }

      ImGui::EndChild();
  }

  // Defined with the rest of the firmware UI several hundred lines below, beside
  // the board state and backup path it shares a subject with.
  Void drawFlashCatalog();

  Void drawViewBody(Int32 view, Float32 w, Float32 h)
  {
      const ImVec2 p0 = ImGui::GetCursorScreenPos();

      // Explicitly black rather than inherited: the map's background must never
      // pick up a tint or an alpha, it is what the point cloud is read against.
      ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::ansi::BLACK);
      ImGui::PushID(view);

      if(view == 0 || view == 1)
      {
          // is3D is set immediately before the draw, so both maps can be on screen
          // at once in the floating layout - each draw sees the flag it needs.
          radarView.is3D = (view == 1);

          ImGui::BeginChild(
              "##map",
              ImVec2(w, h),
              ImGuiChildFlags_None,
              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
          );
          radarView.draw(ImGui::GetContentRegionAvail());
          drawMapHud(p0, ImVec2(w, h));
          ImGui::GetWindowDrawList()->AddRect(
              p0,
              ImVec2(p0.x + w, p0.y + h),
              IM_COL32(0x3A, 0x3A, 0x3A, 0xFF),
              0.0f,
              0,
              1.0f
          );
          ImGui::EndChild();
      }
      else if(view == 2)
      {
          ImGui::BeginChild(
              "##recmap",
              ImVec2(w, h),
              ImGuiChildFlags_None,
              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
          );
          recView.draw(ImGui::GetContentRegionAvail());
          drawRecorderHud(p0, ImVec2(w, h));
          ImGui::GetWindowDrawList()->AddRect(
              p0,
              ImVec2(p0.x + w, p0.y + h),
              IM_COL32(0x3A, 0x3A, 0x3A, 0xFF),
              0.0f,
              0,
              1.0f
          );
          ImGui::EndChild();
      }
      else if(view == 3)
      {
          // Loaded on first sight rather than at startup: reading the sketch
          // library costs a directory scan, and most sessions never open it.
          if(!codeLoaded)
          {
              codeLoaded = true;

              // The first sketch, alphabetically. Each is a real file with its own
              // build target, so any of them beats an empty editor.
              const Vec<Str> first = sketch::list();
              if(!first.empty())
              {
                  openCodeFile(sketch::pathOf(first.front()), "sketches\\" + first.front());
              }
          }

          // Tree | splitter | editor. The tree's width is the user's, clamped so a
          // drag can neither squeeze the editor away nor strand the tree.
          const Float32 splitW = std::max(ImGui::GetStyle().ItemSpacing.x,
                                          8.0f * uiDpiScale);

          const Float32 treeW = codeTreeCollapsed
              ? ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.x * 2.0f
              : std::max(CODE_TREE_MIN_W * uiDpiScale,
                         std::min(
                             codeTreeLogicalW * uiDpiScale,
                             std::min(CODE_TREE_MAX_W * uiDpiScale, w * 0.6f)
                         ));

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

          const Float32 editW = std::max(120.0f, w - treeW - splitW);

          if(refdoc::isDocPath(codePath))
          {
              drawDocPane(editW, h);
          }
          else
          {
              ui::drawCode(codeView, codeEditor, ImVec2(editW, h), ImGui::GetTime());
          }
          handleCodeCommand();
      }
      else if(view == DRIVE_VIEW)
      {
          // Polled inside the body, so the fast LIGHTS poll only runs while
          // somebody is actually watching the lamps. TEMPORARY.
          pollLights();
          drawDriveBody(w, h);
      }
      else if(view == CUE_VIEW)
      {
          // Both polled inside the body, so the fast one only runs while
          // somebody is watching. The list is a no-op after the first call.
          pollCueList();
          pollCueState();
          drawCueBody(w, h);
      }
      else if(view == SOUND_VIEW)
      {
          pollSoundState();
          drawSoundBody(w, h);
      }
      else if(view == FLASH_VIEW)
      {
          drawFlashCatalog();
      }
      else
      {
          // Range is the terminal branch, so an index from a stale settings file
          // lands on a real view. Polled only while drawn - see pollTof().
          pollSensorList();
          pollTof(true);
          drawRangeBody(w, h);
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
      if(view == RANGE_VIEW)
      {
          return "Range";
      }
      if(view == DRIVE_VIEW)
      {
          return "Drive";
      }
      if(view == CUE_VIEW)
      {
          return "Cues";
      }
      if(view == SOUND_VIEW)
      {
          return "Sound";
      }
      if(view == FLASH_VIEW)
      {
          return "Flash";
      }
      return "Range";
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
      if(view == CUE_VIEW)
      {
          return ui::Icon::ICON_LAMP;
      }
      if(view == SOUND_VIEW)
      {
          return ui::Icon::ICON_SIGNAL;
      }
      if(view == FLASH_VIEW)
      {
          return ui::Icon::ICON_FLASH;
      }
      return ui::Icon::ICON_TOF;
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
          Array<Char, 48> label;
          iconTabLabel(label.data(), label.size(), viewName(v));

          const Bool open = ImGui::BeginTabItem(label.data(), nullptr, viewSel(v));
          tabIcon(viewIcon(v));
          if(open)
          {
              centralView = v;
              wsFocused = v;
              drawViewBody(v, mapW, viewH);
              ImGui::EndTabItem();
          }
      }

      // Right-aligned, so opening the console is where the tabs END rather than a
      // second row of chrome above them.
      if(ImGui::TabItemButton(
          consoleOpen ? " Console < " : " > Console ",
          ImGuiTabItemFlags_Trailing | ImGuiTabItemFlags_NoTooltip
      ))
      {
          consoleOpen = !consoleOpen;
          panelLayoutDirty = true;
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "The serial and build logs, in a column on the left.\n" "Ctrl+` toggles it."
          );
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

      // The bottom bar belongs to the VIEW above it: the map's controls are ABSENT
      // on a board tab rather than disabled, so the board gets the height back.
      if(ctrlH > 0.0f)
      {
          ImGui::BeginChild(
              "##controls",
              ImVec2(mapW, ctrlH),
              ImGuiChildFlags_None,
              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
          );
          drawCentralControls(centralView);
          ImGui::EndChild();
      }
  }

  // ==================================================================== vehicle
  // The Pico as the car's controller rather than as a serial port: the link and
  // its vocabulary, then what the board says about servo and ESC, or why not.

  Void drawPicoLinkBlock()
  {
      const ImGuiStyle& sty = ImGui::GetStyle();
      const PicoState   st = picoLink.state();
      const Bool        live = (st == PicoState::PICO_STATE_CONNECTED);
      const Bool        busy = live || st == PicoState::PICO_STATE_CONNECTING;
      const Float32       bh = ImGui::GetFrameHeight() * 1.2f;

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
          ui::combo(
              "##picoport",
              &picoIndex,
              picoItems.data(),
              static_cast<Int32>(picoItems.size())
          );
      }
      ImGui::EndDisabled();

      ImGui::SameLine();
      if(ui::iconButton(ui::Icon::ICON_REFRESH, "Refresh"))
      {
          refreshPicoPorts();
      }

      if(busy)
      {
          if(ui::iconButton(
              ui::Icon::ICON_PLUG_DISCONNECT,
              "Disconnect",
              ImVec2(-FLT_MIN, bh),
              ui::Tint::TINT_WARN
          ))
          {
              picoLink.disconnect();
          }
      }
      else
      {
          ImGui::BeginDisabled(picoIndex < 0);
          if(ui::iconButton(
              ui::Icon::ICON_PLUG_CONNECT,
              "Connect",
              ImVec2(-FLT_MIN, bh),
              ui::Tint::TINT_GOOD
          ))
          {
              connectPico();
          }
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
          ImGui::TableNextColumn();
          ImGui::TextDisabled("State");
          ImGui::TableNextColumn();
          colored(picoStateColor(st), "%s", picoStateText(st));

          const Str p = picoLink.port();
          keyValue("Port",    "%s",   p.empty() ? "--" : p.c_str());
          keyValue("Sent",    "%llu", picoLink.txLines());
          keyValue("Received","%llu", picoLink.rxLines());
          keyValue("Dropped", "%llu", picoLink.dropped());

          // A board that never answers has to read as deliberately silent rather
          // than broken, and the value alone carries that.
          const Float64 age = picoLink.lastRxAgeS();
          Array<Char, 48> ageS;
          picoAgeText(ageS.data(), ageS.size(), age);

          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextDisabled("Last line");
          ImGui::TableNextColumn();
          colored(
              (age >= 0.0 && age < 2.0) ? ui::sem::GOOD : (live ? ui::sem::WARN : ui::sem::MUTED),
              "%s",
              ageS.data()
          );

          ImGui::EndTable();
      }
  }

  Void drawPicoCommands()
  {
      const ImGuiStyle& sty = ImGui::GetStyle();
      const Bool        live = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

      ImGui::SeparatorText("Commands");

      ImGui::BeginDisabled(!live);
      if(ImGui::BeginTable("picocmd", 3, ImGuiTableFlags_SizingStretchSame))
      {
          auto cmd = [](ui::Icon ic, const Char* label, const Char* line)
          {
              ImGui::TableNextColumn();
              if(ui::iconButton(ic, label, ImVec2(-FLT_MIN, 0.0f)))
              {
                  sendPico(line);
              }
          };

          // Two vocabularies, both real: pico_debug answers PING/ID/STATUS/HELP/
          // LED, tt02_control only `?`. Half this grid comes back ERR either way.
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

      // Free-text line. Enter sends and keeps the cursor here. The width matches
      // what iconButton auto-sizes to, so the field ends where the button begins.
      const Float32 sendW = ImGui::CalcTextSize("Send").x + sty.FramePadding.x * 2.0f
                          + (ui::iconsReady()
                             ? ui::iconSize() + sty.ItemInnerSpacing.x : 0.0f);
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - sendW - sty.ItemSpacing.x);

      Bool fire = ImGui::InputTextWithHint("##picocmdline", "type a command",
                                           cmdBuf.data(), cmdBuf.size(),
                                           ImGuiInputTextFlags_EnterReturnsTrue);
      if(fire)
      {
          ImGui::SetKeyboardFocusHere(-1);
      }

      ImGui::SameLine();
      if(ui::iconButton(ui::Icon::ICON_SEND, "Send"))
      {
          fire = true;
      }

      if(fire)
      {
          sendPico(cmdBuf.data());
          cmdBuf[0] = '\0';
      }
      ImGui::EndDisabled();
  }

  // Servo and ESC pulse widths, when the firmware reports them. Nothing is
  // synthesised: with no S line the readouts say "--" and the note says why.
  Void drawControllerState()
  {
      ImGui::SeparatorText("Controller state");

      Array<Char, 24> servo = {'-', '-'};
      Array<Char, 24> esc = {'-', '-'};
      if(vehicleStatus.have)
      {
          std::snprintf(servo.data(), servo.size(), "%d", vehicleStatus.servoUs);
          std::snprintf(esc.data(),   sizeof(esc.data()),   "%d", vehicleStatus.escUs);
      }

      // Bounded rather than stretched: at full width a two-column table throws its
      // second value against the far edge and the pair stops reading as a pair.
      const Float32 tableW = std::min(520.0f * uiDpiScale, ImGui::GetContentRegionAvail().x);

      if(ImGui::BeginTable("pwm", 2, ImGuiTableFlags_SizingStretchSame, ImVec2(tableW, 0.0f)))
      {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          statCell(servo.data(), "servo GP0  us");
          ImGui::TableNextColumn();
          statCell(esc.data(),   "ESC GP1  us");
          ImGui::EndTable();
      }

      ImGui::Spacing();

      // Everything the board reports about itself, and "--" for everything it does
      // not. The reply row is the only thing saying which firmware is answering.
      if(ImGui::BeginTable("vehstat", 2, ImGuiTableFlags_SizingStretchProp, ImVec2(tableW, 0.0f)))
      {
          const BoardStatus brd = picoFlash.board();
          keyValue("Program", "%s", brd.program.empty() ? "--" : brd.program.c_str());

          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextDisabled("? reply");
          ImGui::TableNextColumn();
          if(vehicleStatus.have)
          {
              colored(ui::sem::GOOD, "S line");
          }
          else if(vehUnsupported)
          {
              colored(ui::sem::WARN, "no S line");
          }
          else
          {
              ImGui::TextUnformatted("--");
          }

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
          // Start from a clean slate: a stale brake hold or half-finished flash
          // carried in would look like the detector's first decision.
          autoLightState = lights::AutoState{};
          if(!autoLights)
          {
              lightInput = lights::Input{};
          }
      }
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Work the lamps out from what the car is doing.\n\n" "Steering past %.0f%% of lock indicates that way and holds until it\n" "unwinds past %.0f%%. Any drop in throttle lights the brakes for\n" "%.0f ms. Armed turns the tail lights on.\n\n" "Both are guesses from the outputs - there is no brake pedal and no\n" "indicator stalk on this car, and no encoder yet, so \"braking\" means\n" "the throttle was reduced rather than the car actually slowed.",
              static_cast<Float64>(lights::AutoConfig{}.turnOnAbs * 100.0f),
              static_cast<Float64>(lights::AutoConfig{}.turnOffAbs * 100.0f),
              lights::AutoConfig{}.brakeHoldS * 1000.0
          );
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

      // The controls below SHOW the decision while automatic is on. Left live they
      // would fight the detector, and a press would last exactly one frame.
      ImGui::BeginDisabled(autoLights);

      const Float32 w = ImGui::GetContentRegionAvail().x;
      const Float32 third = (w - ImGui::GetStyle().ItemSpacing.x * 2.0f) / 3.0f;
      const Float32 quarter = (w - ImGui::GetStyle().ItemSpacing.x * 3.0f) / 4.0f;

      ImGui::TextDisabled("Headlights");
      struct HeadOpt { const Char* label; lights::Head v; };
      static constexpr HeadOpt HEADS[3] = {
          { "Off", lights::Head::HEAD_OFF },
          { "DRL", lights::Head::HEAD_DRL },
          { "On",  lights::Head::HEAD_ON  },
      };
      // Off / dim / full, as three states of one lamp - which is what they are.
      static constexpr ui::Icon HEAD_ICONS[3] = {
          ui::Icon::ICON_STATUS_IDLE, ui::Icon::ICON_LAMP_DIM, ui::Icon::ICON_LAMP,
      };
      for(Int32 i = 0; i < 3; ++i)
      {
          if(i)
          {
              ImGui::SameLine();
          }
          if(ui::segmentedIconButton(
              HEAD_ICONS[i],
              HEADS[i].label,
              lightInput.head == HEADS[i].v,
              ImVec2(third, 0.0f)
          ))
          {
              lightInput.head = HEADS[i].v;
          }
      }

      ImGui::Spacing();
      ImGui::TextDisabled("Turn");
      struct TurnOpt { const Char* label; lights::Turn v; };
      static constexpr TurnOpt TURNS[4] = {
          { "Off",    lights::Turn::TURN_OFF    },
          { "Left",   lights::Turn::TURN_LEFT   },
          { "Right",  lights::Turn::TURN_RIGHT  },
          { "Hazard", lights::Turn::TURN_HAZARD },
      };
      for(Int32 i = 0; i < 4; ++i)
      {
          if(i)
          {
              ImGui::SameLine();
          }

          // Hazards get the warning glyph and the lamp's own amber; Left and Right
          // get neither, because the pack has no left/right arrow.
          const Bool sel = (lightInput.turn == TURNS[i].v);
          const ui::Tint t = (TURNS[i].v == lights::Turn::TURN_HAZARD)
                           ? ui::Tint::TINT_WARN : ui::Tint::TINT_NONE;

          Bool hit;
          if(TURNS[i].v == lights::Turn::TURN_HAZARD)
          {
              ui::pushTint(t);
              hit = ui::segmentedIconButton(
                  ui::Icon::ICON_HAZARD,
                  TURNS[i].label,
                  sel,
                  ImVec2(quarter, 0.0f)
              );
              ui::popTint(t);
          }
          else if(TURNS[i].v == lights::Turn::TURN_OFF)
          {
              hit = ui::segmentedIconButton(
                  ui::Icon::ICON_STATUS_IDLE,
                  TURNS[i].label,
                  sel,
                  ImVec2(quarter, 0.0f)
              );
          }
          else
          {
              hit = ui::segmentedButton(TURNS[i].label, sel, ImVec2(quarter, 0.0f));
          }

          if(hit)
          {
              lightInput.turn = TURNS[i].v;
          }
      }

      ImGui::Spacing();
      ui::checkbox("Brake", &lightInput.brake);
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Tail and brake are the same red lamp - 30%% and 100%%.\n" "With an indicator running, that side alternates and the\n" "other stays solid. Try Brake with Right."
          );
      }
      ImGui::SameLine();
      ui::checkbox("Reverse", &lightInput.reverse);

      ImGui::Spacing();
      if(ui::iconButton(ui::Icon::ICON_CLEAR, "All off"))
      {
          lightInput = lights::Input{};
      }

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
  // Load a different program onto the Pico on demand - a thin face over
  // firmware\build.bat / flash.bat / backup.bat, so there is one mechanism.

  Void sizeText(Char* buf, Size n, Int64 bytes)
  {
      if(bytes >= 1024 * 1024)
      {
          std::snprintf(buf, n, "%.1f MB", bytes / (1024.0 * 1024.0));
      }
      else
      {
          std::snprintf(buf, n, "%lld KB", (bytes + 512) / 1024);
      }
  }

  // The catalog's descriptions are paragraphs, not labels. A hover tooltip keeps
  // every row one predictable height.
  Void descriptionTooltip(const Str& text)
  {
      if(text.empty() || !ImGui::IsItemHovered())
      {
          return;
      }

      ImGui::BeginTooltip();
      ImGui::PushTextWrapPos(420.0f * uiDpiScale);
      ImGui::TextUnformatted(text.c_str());
      ImGui::PopTextWrapPos();
      ImGui::EndTooltip();
  }

  // ANSI coloring for the build log. The tag is colored and the rest of the line
  // by TOKEN: paths recede to gray, ninja's verbs come forward in bright white,
  // and error/warning are found wherever they appear. Six colors, each meaning
  // one thing, on the same black ground the serial console next door uses.
  struct LogRun
  {
      Size  begin;
      Size  end;
      ImU32 col;
  };

  [[nodiscard]] Bool containsCI(const Str& hay, const Char* needle)
  {
      const Size n = std::strlen(needle);
      if(n == 0 || hay.size() < n)
      {
          return false;
      }
      for(Size i = 0; i + n <= hay.size(); ++i)
      {
          if(_strnicmp(hay.c_str() + i, needle, n) == 0)
          {
              return true;
          }
      }
      return false;
  }

  [[nodiscard]] ImU32 flashTokenColor(const Str& tok, ImU32 fallback)
  {
      // Found anywhere, not just at the start of a line: "CMake Warning at ..."
      // and "range.cxx:41:9: error: ..." both carry the word in the middle.
      if(containsCI(tok, "error"))
      {
          return ui::ansi::BRRED;
      }
      if(containsCI(tok, "warning"))
      {
          return ui::ansi::BRYELLOW;
      }

      // A PATH RECEDES: every cmake line carries one and the verb in front of it is
      // what matters. A separator AND a dot - a slash alone is not a path.
      const Bool sep = tok.find('/') != Str::npos || tok.find('\\') != Str::npos;
      const Bool dot = tok.find('.') != Str::npos;
      if(sep && dot)
      {
          return ui::ansi::GRAY;
      }

      // 'pico2_w', 'rp2350-arm-s' - the VALUE in a cmake status line, which is
      // the one word on it that ever differs between two builds.
      Int32 quotes = 0;
      for(const Char c : tok)
      {
          if(c == '\'' || c == '"')
          {
              ++quotes;
          }
      }
      if(quotes >= 2)
      {
          return ui::ansi::BRYELLOW;
      }

      return fallback;
  }

  // Splits `s` into colored runs covering it end to end.
  Void flashRuns(const Str& s, Vec<LogRun>& out)
  {
      out.clear();
      if(s.empty())
      {
          return;
      }

      Size  i = 0;
      ImU32 body = ui::ansi::WHITE;
      ImU32 first = 0;          // 0 = no override for the first word

      if(s[0] == '[')
      {
          // The scripts write [conf ], [build], [ok   ], [error]; ninja writes
          // [1/3]. Both are a bracket at column zero, told apart by content.
          if(const Size close = s.find(']'); close != Str::npos && close <= 12)
          {
              Str tag = s.substr(1, close - 1);
              while(!tag.empty() && tag.back() == ' ')
              {
                  tag.pop_back();
              }

              ImU32 head = ui::ansi::BRCYAN;

              if(!tag.empty()
                 && tag.find_first_not_of("0123456789/") == Str::npos)
              {
                  // ninja's progress counter. Gray: it is a COUNTER, not a
                  // verdict, and it is the same on every successful build.
                  head = ui::ansi::GRAY;
                  first = ui::ansi::BRWHITE;   // ...but "Linking" is worth seeing
              }
              else if(_stricmp(tag.c_str(), "error") == 0
                   || _stricmp(tag.c_str(), "fail") == 0)
              {
                  head = ui::ansi::BRRED;

                  // The dim red, not the bright one: the line still reads as bad
                  // without becoming a block of shouting.
                  body = ui::ansi::RED;
              }
              else if(_stricmp(tag.c_str(), "ok") == 0)
              {
                  head = ui::ansi::BRGREEN;
              }
              else if(_stricmp(tag.c_str(), "start") == 0
                   || _stricmp(tag.c_str(), "busy") == 0
                   || _stricmp(tag.c_str(), "skip") == 0)
              {
                  head = ui::ansi::BRYELLOW;
              }

              out.push_back(LogRun{ 0, close + 1, head });
              i = close + 1;
          }
      }
      else if(s.rfind("-- ", 0) == 0)
      {
          // cmake's status prefix. Dim, because there are forty of them and the
          // marker is not the message.
          out.push_back(LogRun{ 0, 2, ui::ansi::GRAY });
          i = 2;

          // OURS. The messages this project's CMakeLists prints are the only lines
          // in a configure dump written by somebody who works on this car.
          if(s.rfind("-- bibo:", 0) == 0)
          {
              body = ui::ansi::BRCYAN;
          }
      }

      while(i < s.size())
      {
          Size b = i;
          while(i < s.size() && (s[i] == ' ' || s[i] == '\t'))
          {
              ++i;
          }
          if(i > b)
          {
              out.push_back(LogRun{ b, i, body });
          }
          if(i >= s.size())
          {
              break;
          }

          b = i;
          while(i < s.size() && s[i] != ' ' && s[i] != '\t')
          {
              ++i;
          }

          const Str  tok = s.substr(b, i - b);
          const ImU32 col = (first != 0) ? first : flashTokenColor(tok, body);
          first = 0;
          out.push_back(LogRun{ b, i, col });
      }
  }

  // The script output pane. A build prints a hundred lines and you want to watch
  // them arrive, so this scrolls - it is a log.
  Void drawFlashOutput(const Char* id, const ImVec2& size)
  {
      if(ui::iconButton(ui::Icon::ICON_CLEAR, "Clear"))
      {
          flashLog.clear();
      }
      ImGui::SameLine();
      ui::checkbox("Auto-scroll", &flashAutoscroll);
      ImGui::SameLine();
      {
          ScopedFont sf(ui::fonts.small);
          ImGui::AlignTextToFramePadding();
          ImGui::TextDisabled("%d lines", static_cast<Int32>(flashLog.size()));
      }

      // The same black ground and mono face as the serial console: the two panes
      // are tabs of one console and must not look like two programs.
      ImGui::PushStyleColor(ImGuiCol_ChildBg, ui::ansi::BLACK);

      ImGui::BeginChild(id, size, ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);
      {
          // Pushed and popped INSIDE the child: a font push that straddles
          // Begin/EndChild renders as a blank white window - see drawSerialConsole.
          ScopedFont sf(ui::fonts.mono);

          if(!flashLog.empty())
          {
              // Reused rather than built per line: the clipper tokenizes about
              // forty lines a frame, and none needs its own allocation.
              static Vec<LogRun> runs;

              // The runs butt against each other, so no horizontal item spacing.
              ImGui::PushStyleVar(
                  ImGuiStyleVar_ItemSpacing,
                  ImVec2(0.0f, ImGui::GetStyle().ItemSpacing.y)
              );

              ImGuiListClipper clipper;
              clipper.Begin(static_cast<Int32>(flashLog.size()));
              while(clipper.Step())
              {
                  for(Int32 r = clipper.DisplayStart; r < clipper.DisplayEnd; ++r)
                  {
                      const Str& s = flashLog[r];

                      flashRuns(s, runs);
                      if(runs.empty())
                      {
                          // A blank line still has to take a line's height, or a
                          // build log's paragraphing collapses.
                          ImGui::TextUnformatted("");
                          continue;
                      }

                      for(Size k = 0; k < runs.size(); ++k)
                      {
                          if(k != 0)
                          {
                              ImGui::SameLine(0.0f, 0.0f);
                          }
                          ImGui::PushStyleColor(ImGuiCol_Text, runs[k].col);
                          ImGui::TextUnformatted(
                              s.c_str() + runs[k].begin,
                              s.c_str() + runs[k].end
                          );
                          ImGui::PopStyleColor();
                      }
                  }
              }

              ImGui::PopStyleVar();
          }

          if(flashAutoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
          {
              ImGui::SetScrollHereY(1.0f);
          }
      }
      ImGui::EndChild();

      ImGui::PopStyleColor();
  }

  Void drawFlashControls()
  {
      const ImGuiStyle& sty = ImGui::GetStyle();
      const BoardStatus brd = picoFlash.board();
      const Bool        busy = picoFlash.busy();
      const Float32       bh = ImGui::GetFrameHeight() * 1.2f;

      // ---- board -----------------------------------------------------------
      ImGui::SeparatorText("Board");

      const Float32 refreshW = ImGui::CalcTextSize("Refresh").x + sty.FramePadding.x * 2.0f;

      {
          // BOOTSEL is a MODE, and forgetting you are in it is the commonest way
          // to be confused here: the COM port is gone and nothing answers.
          ScopedFont sf(ui::fonts.title);
          ImGui::AlignTextToFramePadding();

          if(brd.bootsel)
          {
              colored(ui::sem::WARN, "BOOTSEL  -  %s", brd.drive.c_str());
          }
          else if(brd.present)
          {
              colored(
                  ui::sem::GOOD,
                  "Running - %s",
                  brd.port.empty() ? "no serial port" : brd.port.c_str()
              );
          }
          else
          {
              colored(ui::sem::MUTED, "No board found");
          }
      }

      ImGui::SameLine(ImGui::GetContentRegionAvail().x - refreshW);
      if(ui::iconButton(ui::Icon::ICON_REFRESH, "Refresh"))
      {
          picoFlash.refreshBoard();
          picoFlash.refreshCatalog();
      }

      {
          // Always drawn, even with nothing to say: rendering it conditionally
          // makes every control below jump a line as the board enters BOOTSEL.
          ScopedFont sf(ui::fonts.small);

          if(brd.present && !brd.bootsel)
          {
              ImGui::TextDisabled(
                  "%s%s%s",
                  brd.chip.c_str(),
                  brd.program.empty() ? "" : " ",
                  brd.program.c_str()
              );
          }
          else if(brd.bootsel)
          {
              ImGui::TextDisabled("%s%sbootloader", brd.chip.c_str(), brd.chip.empty() ? "" : " ");
          }
          else
          {
              ImGui::TextDisabled("--");
          }
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

      // ---- backup -----------------------------------------------------------
      ImGui::SeparatorText("Backup");

      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::InputTextWithHint(
          "##backupout",
          "output .uf2 path",
          backupBuf.data(),
          backupBuf.size()
      );

      ImGui::BeginDisabled(busy || backupBuf[0] == '\0');
      if(ui::iconButton(ui::Icon::ICON_BACKUP, "Back up board flash", ImVec2(-FLT_MIN, bh)))
      {
          startBackup();
      }
      ImGui::EndDisabled();

      // ---- reboot -----------------------------------------------------------
      ImGui::Spacing();
      ImGui::SeparatorText("Reboot");

      const Float32 half = (ImGui::GetContentRegionAvail().x - sty.ItemSpacing.x) * 0.5f;

      ImGui::BeginDisabled(busy);
      if(ui::iconButton(ui::Icon::ICON_REBOOT, "To BOOTSEL", ImVec2(half, bh), ui::Tint::TINT_WARN))
      {
          picoLink.disconnect();
          releasePicoPortForBoardOp();
          picoFlash.rebootBootsel();
      }
      ImGui::SameLine();
      if(ui::iconButton(ui::Icon::ICON_REBOOT, "Normally", ImVec2(half, bh)))
      {
          releasePicoPortForBoardOp();
      }
          picoFlash.rebootNormal();
      ImGui::EndDisabled();

  }

  // ============================================== the flash view ==
  // The catalog, full width, as a central view - its rows never fitted the
  // sidebar. The modal comes WITH it: a popup's identity is its ID stack, so
  // OpenPopup and BeginPopupModal must be drawn by the same function.
  Void drawFlashCatalog()
  {
      const ImGuiStyle& sty = ImGui::GetStyle();
      const Bool        busy = picoFlash.busy();
      const Float32     bh = ImGui::GetFrameHeight() * 1.2f;

      // ---- catalog ----------------------------------------------------------
      ImGui::Spacing();
      ImGui::SeparatorText("Firmware");

      const Vec<FirmwareEntry>& cat = picoFlash.catalog();

      // OpenPopup is deferred out of the row's PushID scope: opening it inside the
      // row and beginning it outside would never match on the ID stack.
      Bool openConfirm = false;

      if(cat.empty())
      {
          colored(ui::sem::WARN, "catalog.txt: no entries");
      }

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
                  Array<Char, 32> sz;
                  sizeText(sz.data(), sz.size(), e.sizeBytes);
                  colored(ui::sem::GOOD, "%s   %s", sz.data(), e.builtAt.c_str());
              }
              else
              {
                  colored(ui::sem::WARN, "%s", e.buildable ? "not built yet" : "missing on disk");
              }

              // Which board the image is FOR, stated nowhere else: a .uf2 carries
              // none, and a Pico 2 W image on a plain Pico 2 runs with a dead LED.
              if(!e.board.empty())
              {
                  ImGui::SameLine();
                  colored(ui::sem::MUTED, "   %s", e.board.c_str());
              }
          }

          const Float32 half = (ImGui::GetContentRegionAvail().x - sty.ItemSpacing.x) * 0.5f;

          ImGui::BeginDisabled(busy || !e.buildable);
          if(ui::iconButton(ui::Icon::ICON_BUILD, "Build", ImVec2(half, bh)))
          {
              picoFlash.build(e.id);
          }
          ImGui::EndDisabled();
          if(!e.buildable && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
          {
              ImGui::SetTooltip("No source for this in the repo - the .uf2 is all there is.");
          }

          ImGui::SameLine();

          ImGui::BeginDisabled(busy || !e.present);
          if(ui::iconButton(
              ui::Icon::ICON_FLASH,
              "Flash...",
              ImVec2(half, bh),
              ui::Tint::TINT_WARN
          ))
          {
              confirmId = e.id;
              confirmName = e.name;
              confirmPath = e.uf2Path;
              openConfirm = true;
          }
          ImGui::EndDisabled();

          ImGui::PopID();
          ImGui::Separator();
      }

      if(openConfirm)
      {
          ImGui::OpenPopup("Flash this firmware?");
      }


      // ---- confirmation ---------------------------------------------------
      // Flashing is destructive and, for anything not in the catalog, permanent,
      // so it gets a modal that names the image and says so.
      const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
      ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
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
          ImGui::BulletText(
              "There is no undo: unless that firmware is in the catalog\n" "above, the only way back is a backup you took first."
          );
          ImGui::Spacing();
          ImGui::TextWrapped(
              "If you are unsure what is on the board, cancel and press " "\"Back up board flash\" first."
          );
          ImGui::PopTextWrapPos();

          ImGui::Separator();

          if(ui::button("Cancel", ImVec2(150.0f * uiDpiScale, bh)))
          {
              ImGui::CloseCurrentPopup();
          }

          ImGui::SameLine();
          // Red: this is the point of no return - it overwrites the board.
          if(ui::iconButton(
              ui::Icon::ICON_FLASH,
              "Flash it",
              ImVec2(260.0f * uiDpiScale, bh),
              ui::Tint::TINT_BAD
          ))
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

  // Board state, backup and reboot: status and one-shot actions, which is what a
  // narrow column is good for. THE CATALOG IS NOT HERE - it is the Flash view.
  Void sectionFirmware()
  {
      drawFlashControls();
  }

  // ==================================================================== console
  // Everything that streams, in one place: the serial line log and whatever the
  // flash/build scripts print. Each gets its OWN fixed-height scroll region - a
  // log that grows the scrolling page it sits on cannot be used.
  constexpr Float32 LOG_PANE_H = 260.0f;   // logical px; multiplied by uiDpiScale at use

  // ---- the console's palette ------------------------------------------------
  // A terminal, not a panel: pure black rather than the slate the rest of the hub
  // uses, because this is a screen and not a molded object. The colors are the
  // ANSI sixteen, so a line reads the way it would in any serial monitor.
  namespace ansi
  {

    constexpr ImU32 BLACK = IM_COL32(0x0A, 0x0A, 0x0A, 0xFF);   // the ground
    constexpr ImU32 RED = IM_COL32(0xCD, 0x32, 0x32, 0xFF);
    constexpr ImU32 GREEN = IM_COL32(0x3F, 0xC0, 0x50, 0xFF);
    constexpr ImU32 YELLOW = IM_COL32(0xCD, 0xAA, 0x2E, 0xFF);
    constexpr ImU32 BLUE = IM_COL32(0x40, 0x80, 0xD0, 0xFF);
    constexpr ImU32 MAGENTA = IM_COL32(0xB0, 0x5C, 0xC0, 0xFF);
    constexpr ImU32 CYAN = IM_COL32(0x35, 0xB5, 0xB5, 0xFF);
    constexpr ImU32 WHITE = IM_COL32(0xC8, 0xC8, 0xC8, 0xFF);
    constexpr ImU32 GRAY = IM_COL32(0x66, 0x66, 0x66, 0xFF);
    constexpr ImU32 BRIGHT = IM_COL32(0xEE, 0xEE, 0xEE, 0xFF);

  }

  // What color a line is, from what the board says. The firmware's vocabulary is
  // four words wide, and finding the one ERR in three hundred lines is the point.
  ImU32 consoleColor(const PicoLine& ln)
  {
      if(ln.poll)
      {
          // Chatter, when it is shown at all, is dim. It is context, not content.
          return ansi::GRAY;
      }
      if(ln.outgoing)
      {
          return ansi::CYAN;      // what WE said
      }

      const Char* t = ln.text.c_str();
      if(std::strncmp(t, "ERR", 3) == 0)
      {
          return ansi::RED;
      }
      if(std::strncmp(t, "OK", 2) == 0)
      {
          return ansi::GREEN;
      }
      if(std::strncmp(t, "INFO", 4) == 0)
      {
          return ansi::BLUE;
      }
      if(std::strncmp(t, "PONG", 4) == 0)
      {
          return ansi::GREEN;
      }
      return ansi::WHITE;
  }

  Void drawSerialConsole(const ImVec2& size)
  {
      if(ui::iconButton(ui::Icon::ICON_CLEAR, "Clear"))
      {
          picoLog.clear();
      }
      ImGui::SameLine();
      ui::checkbox("Auto-scroll", &logAutoscroll);
      ImGui::SameLine();

      // Off by default: the hub's DRIVE/LIGHTS/STATUS/TOF polls are sixteen lines
      // a second on their own, and they bury everything you typed.
      ui::checkbox("Polling", &logShowPoll);
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Show the traffic the hub generates on its own:\n" "DRIVE, LIGHTS, STATUS and TOF polls, and their\n" "replies. Anything you send by hand always shows."
          );
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
          ImGui::SetTooltip(
              "Copy every line currently shown - what the filter\n" "left, in the order it is on screen."
          );
      }

      ImGui::SetNextItemWidth(-FLT_MIN);
      ImGui::InputTextWithHint("##logfilter", "filter lines", filterBuf.data(), filterBuf.size());

      logShown.clear();
      for(Int32 i = 0; i < static_cast<Int32>(picoLog.size()); ++i)
      {
          if(!logShowPoll && picoLog[i].poll)
          {
              continue;
          }
          if(logMatches(picoLog[i]))
          {
              logShown.push_back(i);
          }
      }

      {
          ScopedFont sf(ui::fonts.small);
          const Int32 hidden = static_cast<Int32>(picoLog.size())
                             - static_cast<Int32>(logShown.size());
          if(hidden > 0)
          {
              ImGui::TextDisabled(
                  "%d of %d lines - %d hidden - %llu sent / %llu received",
                  static_cast<Int32>(logShown.size()),
                  static_cast<Int32>(picoLog.size()),
                  hidden,
                  picoLink.txLines(),
                  picoLink.rxLines()
              );
          }
          else
          {
              ImGui::TextDisabled(
                  "%d lines - %llu sent / %llu received",
                  static_cast<Int32>(picoLog.size()),
                  picoLink.txLines(),
                  picoLink.rxLines()
              );
          }
      }

      // ---- the screen -------------------------------------------------------
      ImGui::PushStyleColor(ImGuiCol_ChildBg, ansi::BLACK);
      ImGui::PushStyleColor(ImGuiCol_Header,        IM_COL32(0x2A, 0x44, 0x60, 0xFF));
      ImGui::PushStyleColor(ImGuiCol_HeaderHovered, IM_COL32(0x22, 0x36, 0x4E, 0xFF));
      ImGui::PushStyleColor(ImGuiCol_HeaderActive,  IM_COL32(0x2A, 0x44, 0x60, 0xFF));
      ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));

      ImGui::BeginChild(
          "##console",
          size,
          ImGuiChildFlags_None,
          ImGuiWindowFlags_HorizontalScrollbar
      );

      // The font is pushed and popped INSIDE the child: ImGui compares the style
      // and font stack sizes at Begin against End, and an unbalanced frame renders
      // as a blank window rather than as an error anybody can read.
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
                  const PicoLine& ln = picoLog[idx];

                  Array<Char, 512> buf;
                  std::snprintf(
                      buf.data(),
                      buf.size(),
                      "%8.2f %c %s",
                      ln.tS,
                      ln.outgoing ? '>' : '<',
                      ln.text.c_str()
                  );

                  // A Selectable rather than plain text, so lines can be
                  // highlighted and copied: click, shift-click, ctrl-click.
                  ImGui::PushStyleColor(ImGuiCol_Text, consoleColor(ln));
                  ImGui::PushID(idx);
                  const Bool wasSel = (logSel.count(idx) != 0);
                  if(ImGui::Selectable(buf.data(), wasSel, ImGuiSelectableFlags_AllowDoubleClick))
                  {
                      const ImGuiIO& io = ImGui::GetIO();
                      if(io.KeyShift && logSelAnchor >= 0)
                      {
                          // A run in SCREEN order, not log order: the filter can
                          // hide lines between the two clicks.
                          Int32 a = -1;
                          Int32 b = -1;
                          for(Int32 k = 0; k < static_cast<Int32>(logShown.size()); ++k)
                          {
                              if(logShown[k] == logSelAnchor)
                              {
                                  a = k;
                              }
                              if(logShown[k] == idx)
                              {
                                  b = k;
                              }
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
                              for(Int32 k = a; k <= b; ++k)
                              {
                                  logSel.insert(logShown[k]);
                              }
                          }
                      }
                      else if(io.KeyCtrl)
                      {
                          if(wasSel)
                          {
                              logSel.erase(idx);
                          }
                          else
                          {
                              logSel.insert(idx);
                          }
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

      // Ctrl+C copies the selection. Scoped to this child being focused so it does
      // not steal the shortcut from the editor.
      if(!logSel.empty()
         && ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows)
         && ImGui::IsKeyPressed(ImGuiKey_C)
         && ImGui::GetIO().KeyCtrl)
      {
          Str out;
          for(Int32 i : logShown)
          {
              if(logSel.count(i) == 0)
              {
                  continue;
              }
              out += picoLog[i].outgoing ? "> " : "< ";
              out += picoLog[i].text;
              out += "\n";
          }
          ImGui::SetClipboardText(out.c_str());
          LOG_INFO("console", "copied %d selected line(s)", static_cast<Int32>(logSel.size()));
      }

      // Sticks to the bottom only while the view already is at the bottom, so
      // scrolling up to read something does not yank you back.
      if(logAutoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
      {
          ImGui::SetScrollHereY(1.0f);
      }
      }

      ImGui::EndChild();

      ImGui::PopStyleVar();
      ImGui::PopStyleColor(4);
  }

  // The console column: the two logs, filling their own column on the left - a
  // whole column's height instead of the fifth the sidebar section gave them.
  Void drawConsoleColumn(Float32 w, Float32 h)
  {
      ImGui::BeginChild("##consolecol", ImVec2(w, h), ImGuiChildFlags_Borders);

      if(ImGui::BeginTabBar("##concoltabs"))
      {
          Array<Char, 40> lbA;
          const Bool tA = ImGui::BeginTabItem(
              iconTabLabel(lbA.data(), lbA.size(), "Pico serial"));
          tabIcon(ui::Icon::ICON_CONSOLE);
          if(tA)
          {
              // Zero height means "the rest of this child", so the log grows with
              // the window instead of being pinned to a constant.
              drawSerialConsole(ImVec2(0.0f, 0.0f));
              ImGui::EndTabItem();
          }

          Array<Char, 40> lbB;
          const Bool tB = ImGui::BeginTabItem(
              iconTabLabel(lbB.data(), lbB.size(), "Build / flash"));
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
  // frame, where the ID stack is stable.

  Void drawGlobalModals()
  {
      const Float32 bh = ImGui::GetFrameHeight() * 1.2f;

      // The SERIAL port. Over a wireless link picoLink.port() is "192.168.1.42:4242"
      // and a 1200-baud touch on that is not a thing: BOOTSEL always needs the cable.
      const Str linkPort = picoLink.wireless() ? Str() : picoLink.port();
      const Str bport = linkPort.empty()
          ? (picoIndex >= 0 && picoIndex < static_cast<Int32>(picoPorts.size())
                 ? picoPorts[picoIndex] : Str())
          : linkPort;

      if(openBootsel)
      {
          ImGui::OpenPopup("Reboot to BOOTSEL?");
          openBootsel = false;
      }

      const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
      ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
      if(ImGui::BeginPopupModal("Reboot to BOOTSEL?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
      {
          ImGui::PushTextWrapPos(420.0f * uiDpiScale);
          ImGui::TextWrapped(
              "This reboots %s into the RP2350 USB bootloader.",
              bport.empty() ? "the board" : bport.c_str()
          );
          ImGui::Spacing();
          ImGui::BulletText("Whatever the board is running stops immediately.");
          ImGui::BulletText("The serial link drops and the port disappears.");
          ImGui::BulletText("It remounts as the RP2350 mass-storage drive.");
          ImGui::BulletText(
              "It does not come back until a .uf2 is copied onto it,\n" "or the board is power-cycled."
          );
          ImGui::PopTextWrapPos();

          ImGui::Separator();

          if(ui::button("Cancel", ImVec2(150.0f * uiDpiScale, bh)))
          {
              ImGui::CloseCurrentPopup();
          }

          ImGui::SameLine();
          if(ui::iconButton(
              ui::Icon::ICON_REBOOT,
              "Reboot to BOOTSEL",
              ImVec2(260.0f * uiDpiScale, bh),
              ui::Tint::TINT_WARN
          ))
          {
              picoLink.disconnect();
              bootselOk = PicoLink::bootselTouch(bport);
              bootselDone = true;
              refreshPicoPorts();
              ImGui::CloseCurrentPopup();
          }
          ImGui::EndPopup();
      }
  }

  // ==================================================================== sidebar
  // One scrollable column, five collapsing sections, drawn in a fixed order - the
  // only place in the app that scrolls as a page.

  // The sections, and the three things you can do to them: reorder, tear off,
  // put back.
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
      {
          if(e.id == id)
          {
              return e;
          }
      }
      return SECTIONS[0];
  }

  // Moves the section at slot `from` to slot `to`, shifting the rest along. A
  // rotate, not a swap: the others keep their relative order.
  Void moveSection(Int32 from, Int32 to)
  {
      if(from == to || from < 0 || to < 0 || from >= SECTION_COUNT || to >= SECTION_COUNT)
      {
          return;
      }

      const Int32 moved = sectionOrder[from];
      if(from < to)
      {
          for(Int32 k = from; k < to; ++k)
          {
              sectionOrder[k] = sectionOrder[k + 1];
          }
      }
      else
      {
          for(Int32 k = from; k > to; --k)
          {
              sectionOrder[k] = sectionOrder[k - 1];
          }
      }

      sectionOrder[to] = moved;
      panelLayoutDirty = true;
  }

  // The tear-off button, at the right end of the row the header owns. Call it
  // immediately after the header, which must have had SetNextItemAllowOverlap().
  // SameLine, NOT SetCursorScreenPos: moving the cursor past the last submitted
  // item and ending the window without another is an ImGui assertion.
  Bool tearOffButton(Int32 id, Bool floating)
  {
      const Char*   lbl = floating ? "dock" : "float";
      const Float32 w = ImGui::CalcTextSize(lbl).x
                        + ImGui::GetStyle().FramePadding.x * 2.0f;

      const Float32 x = ImGui::GetContentRegionMax().x - w
                      - ImGui::GetStyle().FramePadding.x;

      ImGui::SameLine(x, 0.0f);
      ImGui::PushID(id);
      const Bool hit = ImGui::Button(lbl, ImVec2(w, 0.0f));
      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              floating ? "Put this panel back in the column" : "Tear this panel off into its own window"
          );
      }
      ImGui::PopID();
      return hit;
  }

  // The emergency stop. PINNED above the sidebar's scroll, so it is in the same
  // place in every view and at every scroll position. Deliberately NOT disabled
  // when the link is down: a stop you cannot press is worse than one that presses
  // and does nothing. One body, two ways in - the button and the space bar.
  Void emergencyStop(const Char* how)
  {
      // The hub's own idea of what it is asking for is reset too, or the sliders
      // fight the board back to the old target on the next poll.
      driveSweep = false;
      driveServoWant = 1500;
      driveEscWant = 1500;

      // Keyboard drive goes off, not just to neutral. A stop that leaves the
      // thing that was driving still armed is a stop you have to press twice.
      wasdOn = false;
      wasdSentSteer = 0;
      wasdSentEsc = 0;

      sendPico("STOP");
      LOG_WARN("drive", "emergency stop (%s)", how);
  }

  // SPACE, anywhere in the program.
  //
  // Space is ImGui's own "activate what the nav cursor is on" key, so a panicked
  // space with the cursor on the ARM checkbox would fire this stop AND tick that
  // box - and this runs at the top of frame(), so the box wins and re-arms it.
  // The key is therefore CLAIMED every frame: ImGui's navigation asks for it with
  // ImGuiKeyOwner_NoOwner and gets nothing once somebody owns it. No lock flags,
  // which would block this read too, and the claim must be renewed each frame.
  //
  // The one exception is TEXT ENTRY, and both flavors must be named: the editor
  // reads io.InputQueueCharacters directly, so it sets neither WantTextInput nor
  // an ActiveId. Dragging a slider is NOT an exception.
  Void updateEmergencyKey()
  {
      const ImGuiIO& io = ImGui::GetIO();

      // Any stable non-zero id; it only has to differ from ImGuiKeyOwner_NoOwner.
      ImGui::SetKeyOwner(ImGuiKey_Space, static_cast<ImGuiID>(0xE5709ABCu));

      const Bool typing = io.WantTextInput || codeView.focused;

      // Focus: ImGui clears its key state when the window loses it, so a space
      // held while alt-tabbing away does not arrive here later.
      if(io.AppFocusLost || typing)
      {
          return;
      }

      if(ImGui::IsKeyPressed(ImGuiKey_Space, false))
      {
          emergencyStop("space");
      }
  }

  Void drawEmergencyStop(Float32 width)
  {
      ui::pushTint(ui::Tint::TINT_BAD);
      if(ui::iconButton(
          ui::Icon::ICON_MOTOR_STOP,
          "STOP [SPACE]",
          ImVec2(width, ImGui::GetFrameHeight() * 2.0f)
      ))
      {
          emergencyStop("button");
      }
      ui::popTint(ui::Tint::TINT_BAD);

      if(ImGui::IsItemHovered())
      {
          ImGui::SetTooltip(
              "Everything off.\n" "\n" "The ESC to neutral and disarmed, the steering RELEASED, and any\n" "lamp being held on by hand handed back to the car.\n" "\n" "Released, not centered. Center is only a safe place to leave a\n" "servo if 1500 us is where the linkage wants to sit - if the horn\n" "is a tooth off its spline it is not, and centering would just be\n" "pushing somewhere else. Nothing to push with is the only stop\n" "that works on every car.\n" "\n" "Not slewed. A stop that eases in is not a stop.\n" "\n" "SPACE does the same thing from anywhere in the program, as long\n" "as this window has focus. The one place it does not is while you\n" "are typing - in the console, a text field, or the code editor -\n" "because there space is a character, and a stop that fired on\n" "every word typed would be switched off by the end of the day."
          );
      }
  }

  Void drawSidebar(Float32 width, Float32 height)
  {
      // An OUTER child holding the pinned button and the scrolling column. It MUST
      // be a child: ImGui puts the cursor back at the WINDOW's left content edge
      // after an item, so without one the column lands on the console.
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
      ImGui::BeginChild(
          "##sidebarcol",
          ImVec2(width, height),
          ImGuiChildFlags_None,
          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
      );
      ImGui::PopStyleVar();

      const Float32 stopH = ImGui::GetFrameHeight() * 2.0f;
      drawEmergencyStop(width);
      ImGui::Spacing();

      const Float32 rest = height - stopH - ImGui::GetStyle().ItemSpacing.y * 2.0f;

      ImGui::BeginChild(
          "##sidebar",
          ImVec2(width, std::max(40.0f, rest)),
          ImGuiChildFlags_None,
          ImGuiWindowFlags_None
      );

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
              {
                  ui::iconAt(ImGui::GetWindowDrawList(), e.icon,
                             ImVec2(
                                 a2.x + ImGui::GetStyle().FramePadding.x + ImGui::GetFontSize() * 1.35f,
                                 a2.y + ((b2.y - a2.y) - ui::iconSize()) * 0.5f
                             ));
              }

              if(tearOffButton(e.id, true))
              {
                  sectionFloating[e.id] = false;
                  panelLayoutDirty = true;
              }
              continue;
          }

          const Bool forced = (forceSection == e.id && forceTabFrames > 0);
          if(forced)
          {
              ImGui::SetNextItemOpen(true, ImGuiCond_Always);
          }

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
              ImGui::SetDragDropPayload("BIBO_SECTION", &slot, sizeof(Int32));
              ImGui::TextUnformatted(e.title);
              ImGui::EndDragDropSource();
          }
          if(ImGui::BeginDragDropTarget())
          {
              if(const ImGuiPayload* pl = ImGui::AcceptDragDropPayload("BIBO_SECTION"))
              {
                  dragFrom = *static_cast<const Int32*>(pl->Data);
                  dragTo = slot;
              }
              ImGui::EndDragDropTarget();
          }

          // Drawn over the header after the fact: CollapsingHeader takes a string,
          // so an image inside it means hand-rolling the whole widget.
          if(ui::iconsReady())
          {
              ui::iconAt(ImGui::GetWindowDrawList(), e.icon,
                         ImVec2(
                             hp.x + ImGui::GetStyle().FramePadding.x + ImGui::GetFontSize() * 1.35f,
                             hp.y + ((hq.y - hp.y) - ui::iconSize()) * 0.5f
                         ));
          }

          if(tearOffButton(e.id, false))
          {
              sectionFloating[e.id] = true;
              panelLayoutDirty = true;
          }

          // A named section is no use if it opened below the fold. Scrolling to
          // the header itself, not into its body, keeps the label on screen.
          if(forced)
          {
              ImGui::SetScrollHereY(0.0f);
          }

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
      {
          moveSection(dragFrom, dragTo);
      }
      ImGui::EndChild();

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
          {
              continue;
          }

          // First appearance only: after that ImGui's ini has the position the user
          // dragged it to, and forcing one would undo it on every launch.
          const ImGuiViewport* vp = ImGui::GetMainViewport();
          ImGui::SetNextWindowPos(ImVec2(
              vp->WorkPos.x + vp->WorkSize.x * 0.5f + static_cast<Float32>(slot) * 24.0f * uiDpiScale,
              vp->WorkPos.y + 80.0f * uiDpiScale + static_cast<Float32>(slot) * 24.0f * uiDpiScale
          ),
                                  ImGuiCond_FirstUseEver);
          ImGui::SetNextWindowSize(
              ImVec2(420.0f * uiDpiScale, 380.0f * uiDpiScale),
              ImGuiCond_FirstUseEver
          );
          ImGui::SetNextWindowSizeConstraints(
              ImVec2(240.0f * uiDpiScale, 120.0f * uiDpiScale),
              ImVec2(FLT_MAX, FLT_MAX)
          );

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
              panelLayoutDirty = true;
          }
      }
  }

  // The column's width for this frame, clamped to something usable. Pure, because
  // the layout needs it before the handle that adjusts it can be drawn.
  Float32 consoleWidth(Float32 availW)
  {
      const Float32 lo = CONSOLE_MIN_W * uiDpiScale;
      const Float32 hi = std::max(lo, availW * 0.50f);

      Float32 w = consoleLogicalW * uiDpiScale;
      if(w < lo)
      {
          w = lo;
      }
      if(w > hi)
      {
          w = hi;
      }
      return w;
  }

  // The handle on the console column's RIGHT edge. The sign is the opposite of
  // the sidebar's: this panel is on the left, so dragging right widens it.
  Void consoleSplitter(const ImVec2& at, Float32 h, Float32 thickness)
  {
      ImGui::SetCursorScreenPos(at);
      ImGui::InvisibleButton("##console-split", ImVec2(thickness, h));

      const Bool active = ImGui::IsItemActive();
      if(active || ImGui::IsItemHovered())
      {
          ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      }

      if(active)
      {
          consoleLogicalW += ImGui::GetIO().MouseDelta.x / uiDpiScale;
          consoleLogicalW = std::max(CONSOLE_MIN_W, std::min(1600.0f, consoleLogicalW));
          panelLayoutDirty = true;
      }

      if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
      {
          consoleLogicalW = CONSOLE_DEF_W;
          panelLayoutDirty = true;
      }

      const ImU32 col = active   ? ui::accent::CYAN
                      : ImGui::IsItemHovered() ? ui::accent::CYAN_HI
                      : IM_COL32(0x50, 0x58, 0x60, 0xFF);
      ImDrawList*   dl = ImGui::GetWindowDrawList();
      const Float32 cx = at.x + thickness * 0.5f;
      const Float32 cy = at.y + h * 0.5f;
      const Float32 r = 1.5f * uiDpiScale;
      for(Int32 k = -1; k <= 1; ++k)
      {
          dl->AddCircleFilled(
              ImVec2(cx, cy + static_cast<Float32>(k) * 6.0f * uiDpiScale),
              r,
              col,
              8
          );
      }
  }

  Float32 sidebarWidth(Float32 availW)
  {
      const Float32 lo = SIDEBAR_MIN_W * uiDpiScale;
      const Float32 hi = std::max(lo, availW * 0.62f);

      Float32 w = sidebarLogicalW * uiDpiScale;
      if(w < lo)
      {
          w = lo;
      }
      if(w > hi)
      {
          w = hi;
      }
      return w;
  }

  // The drag handle between the map and the column. The value it adjusts is in
  // LOGICAL pixels, so a drag feels the same at 100% and at 200%.
  Void sidebarSplitter(const ImVec2& at, Float32 h, Float32 thickness)
  {
      ImGui::SetCursorScreenPos(at);
      ImGui::InvisibleButton("##sidebar-split", ImVec2(thickness, h));

      const Bool active = ImGui::IsItemActive();
      if(active || ImGui::IsItemHovered())
      {
          ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      }

      if(active)
      {
          sidebarLogicalW -= ImGui::GetIO().MouseDelta.x / uiDpiScale;
          panelLayoutDirty = true;
      }

      // Double-click restores the default, which is the only way back from a
      // column dragged to a width you cannot grab the edge of.
      if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
      {
          sidebarLogicalW = 400.0f;
          panelLayoutDirty = true;
      }

      // The grip: three dots, only once it is worth noticing. A full-height handle
      // would be a bigger mark on the screen than the thing it resizes.
      const ImU32 col = active   ? ui::accent::CYAN
                      : ImGui::IsItemHovered() ? ui::accent::CYAN_HI
                      : IM_COL32(0x50, 0x58, 0x60, 0xFF);
      ImDrawList* dl = ImGui::GetWindowDrawList();
      const Float32 cx = at.x + thickness * 0.5f;
      const Float32 cy = at.y + h * 0.5f;
      const Float32 r = 1.5f * uiDpiScale;
      for(Int32 k = -1; k <= 1; ++k)
      {
          dl->AddCircleFilled(
              ImVec2(cx, cy + static_cast<Float32>(k) * 6.0f * uiDpiScale),
              r,
              col,
              8
          );
      }
  }

  // The drag handle between the Code view's file tree and the editor - the same
  // shape as sidebarSplitter(), but this panel is on the LEFT so the sign flips.
  Void codeTreeSplitter(const ImVec2& at, Float32 h, Float32 thickness)
  {
      ImGui::SetCursorScreenPos(at);
      ImGui::InvisibleButton("##codetree-split", ImVec2(thickness, h));

      const Bool active = ImGui::IsItemActive();
      if(active || ImGui::IsItemHovered())
      {
          ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      }

      if(active)
      {
          codeTreeLogicalW += ImGui::GetIO().MouseDelta.x / uiDpiScale;
          codeTreeLogicalW = std::max(CODE_TREE_MIN_W, std::min(CODE_TREE_MAX_W, codeTreeLogicalW));
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
      const Float32 r = 1.5f * uiDpiScale;
      for(Int32 k = -1; k <= 1; ++k)
      {
          dl->AddCircleFilled(
              ImVec2(cx, cy + static_cast<Float32>(k) * 6.0f * uiDpiScale),
              r,
              col,
              8
          );
      }
  }

}

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

    for(Int32 i = 0; i < RANGE_COUNT; ++i)
    {
        RANGE_ITEMS[i] = RANGES[i].label;
    }

    refreshPorts();
    refreshPicoPorts();
    applyRange();

    // The catalog is a couple of file stats; the board query spawns picotool and
    // is asynchronous, so neither delays the first frame.
    picoFlash.refreshCatalog();
    picoFlash.refreshBoard();
    defaultBackupName();

    // --connect [port] [baud] pins a specific port; --no-connect suppresses the
    // automatic attempt. With neither, we just connect.
    Bool suppress = false;

    for(Int32 i = 1; i < __argc; ++i)
    {
        if(std::strcmp(__argv[i], "--no-connect") == 0)
        {
            suppress = true;
            continue;
        }

        // --tab names a sidebar section and opens it. The old workspace and lidar
        // sub-tab names still map onto the section that holds that content.
        if(std::strcmp(__argv[i], "--map") == 0 && i + 1 < __argc)
        {
            for(Int32 m = 0; m < static_cast<Int32>(MapMode::MAP_MODE_COUNT); ++m)
            {
                if(_stricmp(__argv[i + 1], mapModeName(static_cast<MapMode>(m))) == 0)
                {
                    radarView.mode = static_cast<MapMode>(m);
                    radarView.is3D = false;
                    forceView = 0;
                    forceViewFrames = 4;
                    centralView = 0;
                }
            }
            continue;
        }

        // --scene <name> selects a 3D overlay AND switches to 3D, since asking
        // for one without the other is never what was meant.
        if(std::strcmp(__argv[i], "--scene") == 0 && i + 1 < __argc)
        {
            for(Int32 m = 0;
                m < static_cast<Int32>(scene3d::SceneMode::SCENE_MODE_COUNT); ++m)
            {
                if(_stricmp(
                    __argv[i + 1],
                    scene3d::sceneModeName(static_cast<scene3d::SceneMode>(m))
                ) == 0)
                {
                    radarView.scene = static_cast<scene3d::SceneMode>(m);
                    radarView.is3D = true;
                    forceView = 1;
                    forceViewFrames = 4;
                    centralView = 1;
                }
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
            else if(_stricmp(v, "sound") == 0 ||
                     _stricmp(v, "speaker") == 0 ||
                     _stricmp(v, "audio") == 0 ||
                     _stricmp(v, "dfplayer") == 0)
                     {
                         forceView = SOUND_VIEW;
                         forceViewFrames = 4;
                     }
            else if(_stricmp(v, "flash") == 0 ||
                     _stricmp(v, "firmware") == 0 ||
                     _stricmp(v, "catalog") == 0)
                     {
                         forceView = FLASH_VIEW;
                         forceViewFrames = 4;
                     }

            // The documents live in the Code tree now, so every word that meant
            // "the Reference view" means the Code view. Kept, not removed.
            else if(_stricmp(v, "reference") == 0 ||
                     _stricmp(v, "ref") == 0 ||
                     _stricmp(v, "docs") == 0)
                     {
                         forceView = 3;
                         forceViewFrames = 4;
                     }

            // Seed the live selection too, so the first frame reserves the
            // right bottom-bar height instead of the map's.
            if(forceView >= 0)
            {
                centralView = forceView;
            }
            continue;
        }

        if(std::strcmp(__argv[i], "--tab") == 0 && i + 1 < __argc)
        {
            struct TabName { const Char* name; Int32 sec; Int32 sub; };
            static constexpr TabName TAB_NAMES[] = {
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
            // "console" and "debug" no longer name a SECTION - the logs have their
            // own column - so they open that instead.
            if(_stricmp(__argv[i + 1], "console") == 0
               || _stricmp(__argv[i + 1], "debug") == 0)
            {
                consoleOpen = true;
                continue;
            }

            for(const TabName& t : TAB_NAMES)
            {
                if(_stricmp(__argv[i + 1], t.name) == 0)
                {
                    forceSection = t.sec;
                    forceSub = t.sub;
                    forceTabFrames = 4;
                }
            }
            continue;
        }

        // --range <meters> pins the view instead of auto-fitting.
        if(std::strcmp(__argv[i], "--range") == 0 && i + 1 < __argc)
        {
            const Float32 m = static_cast<Float32>(std::atof(__argv[i + 1]));
            for(Int32 k = 0; k < RANGE_COUNT; ++k)
            {
                if(RANGES[k].mm > 0.0f && std::fabs(RANGES[k].mm - m * 1000.0f) < 1.0f)
                {
                    rangeIndex = k;
                }
            }
            applyRange();
            continue;
        }

        if(std::strcmp(__argv[i], "--connect") != 0)
        {
            continue;
        }

        if(i + 1 < __argc && __argv[i + 1][0] != '-')
        {
            const Char* want = __argv[i + 1];
            Bool        found = false;
            for(Int32 p = 0; p < static_cast<Int32>(lidarPorts.size()); ++p)
            {
                if(_stricmp(lidarPorts[p].c_str(), want) == 0)
                {
                    portIndex = p;
                    found = true;
                }
            }

            // A named port that is NOT enumerated is offered anyway and allowed to
            // fail: falling back would make --connect COM7 report success against
            // whatever device happened to be selected.
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

    // Only when a port was actually identified. refreshPorts() leaves portIndex at
    // -1 when it cannot tell, and connecting anyway opens somebody else's device.
    if(!suppress && portIndex >= 0)
    {
        connect();
    }
    else if(!suppress)
    {
        LOG_INFO("lidar", "no RPLIDAR adapter found; not auto-connecting");
    }

    // The other half of "launched with no arguments, both devices connected",
    // which is what this app is documented to do. --no-connect suppresses both.
    if(!suppress)
    {
        connectPico();
    }
}

Void app::notifyDeviceChange()
{
    // Called from the window procedure, so it touches nothing but an atomic. The
    // rescan happens on the UI thread in pumpDeviceScan().
    deviceChangePending.store(true, std::memory_order_release);
}

Void app::setDpiScale(Float32 dpiScale)
{
    uiDpiScale = dpiScale > 0.0f ? dpiScale : 1.0f;
}

// Works the lamps out from the drive state, once a frame. The inputs are what the
// BOARD reports, not the sliders: the targets would flash the indicator early.
Void updateAutoLights()
{
    if(!autoLights)
    {
        return;
    }

    lights::Drive d;
    d.steer = driveSteer;
    d.throttleUs = driveEsc;
    d.armed = driveArmed;

    lightInput = lights::detect(autoLightState, d, ImGui::GetTime());
}

// WASD, polled once a frame. A/D steer to FULL LOCK on press and back to straight
// on release. W is forward at the cap below. S is a BRAKE, not reverse:
// chassis.h is forward-only and the throttle clamp turns anything below idle
// INTO idle, so no value means backwards.
//
// WHAT STOPS THIS WHILE SOMEBODY TYPES: not io.WantCaptureKeyboard, which
// ImGuiConfigFlags_NavEnableKeyboard makes true on nearly every frame, and not
// io.WantTextInput alone, because the editor reads io.InputQueueCharacters and
// sets neither it nor an ActiveId. So all three.
Void updateKeyboardDrive()
{
    const ImGuiIO& io = ImGui::GetIO();

    const Bool linkUp = (picoLink.state() == PicoState::PICO_STATE_CONNECTED);

    // Typing, dragging, or the editor has the keyboard.
    const Bool typing = io.WantTextInput
                     || codeView.focused
                     || ImGui::IsAnyItemActive();

    // ImGui clears its key-down array when the window loses focus, so the keys read
    // as released. While minimised main.cxx sleeps: that case is the deadman's.
    const Bool live = wasdOn && linkUp && !typing && !io.AppFocusLost;

    const Bool a = live && ImGui::IsKeyDown(ImGuiKey_A);
    const Bool d = live && ImGui::IsKeyDown(ImGuiKey_D);
    const Bool w = live && ImGui::IsKeyDown(ImGuiKey_W);
    const Bool sK = live && ImGui::IsKeyDown(ImGuiKey_S);

    // Both directions at once cancels rather than picking one.
    const Int32 wantSteer = (a == d) ? 0 : (a ? -1 : 1);

    // S wins over W. A brake that can be overridden by still holding the
    // throttle is not a brake.
    const Int32 wantEsc = (w && !sK) ? 1 : 0;

    if(wantSteer != wasdSentSteer)
    {
        wasdSentSteer = wantSteer;
        driveSteerWant = static_cast<Float32>(wantSteer);

        Array<Char, 32> cmd;
        std::snprintf(cmd.data(), cmd.size(), "STEER %d", wantSteer);
        pollPico(cmd.data());
    }

    if(wantEsc != wasdSentEsc)
    {
        wasdSentEsc = wantEsc;

        if(wantEsc == 0)
        {
            driveEscWant = 1500;
            pollPico("ESC NEUTRAL");
        }
        else
        {
            const Int32 us = wasdCapNow();
            driveEscWant = us;

            Array<Char, 32> cmd;
            std::snprintf(cmd.data(), cmd.size(), "ESC %d", us);
            pollPico(cmd.data());
        }
        wasdFedAt = ImGui::GetTime();
    }

    // ---- the keepalive ---------------------------------------------------
    // The board stops itself after DEADMAN_MS with nothing heard, and this sends
    // on key CHANGES - so holding W steadily would go quiet. Only while driving.
    if(live && wasdSentEsc != 0)
    {
        const Float64 now = ImGui::GetTime();
        if(now - wasdFedAt > 0.2)
        {
            wasdFedAt = now;

            const Int32 us = wasdCapNow();
            Array<Char, 32> cmd;
            std::snprintf(cmd.data(), cmd.size(), "ESC %d", us);
            pollPico(cmd.data());
        }
    }
}

Void app::frame()
{
    pumpData();
    updateAutoLights();

    // BEFORE the keyboard drive, so a stop in this frame is not undone by a
    // still-held W later in the same frame.
    updateEmergencyKey();
    updateKeyboardDrive();

    // Ctrl+` for the console. Checked before anything draws, so the layout below
    // measures the state it is about to render.
    if(ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_GraveAccent))
    {
        consoleOpen = !consoleOpen;
        panelLayoutDirty = true;
    }

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);

    ImGui::Begin(
        "##root",
        nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
    );

    const ImGuiStyle& sty = ImGui::GetStyle();

    // ---- the status bar's height, reserved now and DRAWN LAST ----------
    // Sized to whichever is taller, the type or the icons, plus its padding:
    // sizing to the text alone is what cropped the old lamps.
    const Float32 stripPad = 6.0f * uiDpiScale;
    const Float32 stripH = std::max(ImGui::GetTextLineHeight(), ui::iconSize())
                           + stripPad * 2.0f;

    // ---- map + sidebar ---------------------------------------------------
    // Everything above the bar lays out against the height the bar leaves.
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y -= stripH + sty.ItemSpacing.y;

    // The column's width is the user's - see sidebarSplitter() - and a LOGICAL
    // width, so its contents lay out against a number DPI and zoom do not move.
    const Float32 gap = std::max(sty.ItemSpacing.x, 8.0f * uiDpiScale);
    const Float32 sideW = sidebarWidth(avail.x);

    // The console column, when open, comes off the LEFT before anything else is
    // measured - so the central view narrows and the sidebar does not move.
    const Float32 consW = consoleOpen ? consoleWidth(avail.x) : 0.0f;
    const Float32 consGap = consoleOpen ? gap : 0.0f;

    // Sized for the view on screen, which is why centralView is kept across
    // frames. A view with no controls costs no height - the spacing goes too.
    const Float32 mapW = avail.x - sideW - gap - consW - consGap;
    const Float32 ctrlH = centralControlHeight(centralView, mapW);
    const Float32 mapH = avail.y - ctrlH - (ctrlH > 0.0f ? sty.ItemSpacing.y : 0.0f);

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
        // The central region MUST have its own child window: everything inside it
        // begins with a tab bar, after which ImGui puts the cursor back at the
        // WINDOW's left content edge. Zero padding, popped at once.
        ImGui::SetCursorScreenPos(ImVec2(midX, p0.y));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::BeginChild(
            "##centralcol",
            ImVec2(mapW, avail.y),
            ImGuiChildFlags_None,
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
        );
        ImGui::PopStyleVar();

        drawMapRegion(mapW, mapH, ctrlH);

        ImGui::EndChild();

        sidebarSplitter(ImVec2(midX + mapW, p0.y), avail.y, gap);
        ImGui::SetCursorScreenPos(ImVec2(midX + mapW + gap, p0.y));
    }

    drawSidebar(sideW, avail.y);

    // ---- the status bar, last ------------------------------------------
    // Pinned to the window's bottom edge rather than flowed after the sidebar:
    // the two columns above it do not necessarily end at the same y.
    {
        const ImVec2 wp = ImGui::GetWindowPos();
        const ImVec2 ws = ImGui::GetWindowSize();

        ImGui::SetCursorScreenPos(ImVec2(
            wp.x + sty.WindowPadding.x,
            wp.y + ws.y - sty.WindowPadding.y - stripH
        ));

        // A hairline above it, so the bar reads as chrome rather than as more
        // content that happens to be at the bottom.
        ImGui::GetWindowDrawList()->AddLine(
            ImVec2(wp.x, wp.y + ws.y - sty.WindowPadding.y - stripH - 1.0f),
            ImVec2(wp.x + ws.x, wp.y + ws.y - sty.WindowPadding.y - stripH - 1.0f),
            ImGui::GetColorU32(ImGuiCol_Separator)
        );

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f * uiDpiScale, stripPad));
        ImGui::BeginChild(
            "##statusbar",
            ImVec2(ws.x - sty.WindowPadding.x * 2.0f, stripH),
            ImGuiChildFlags_None,
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse
        );
        drawStatusBar();
        ImGui::EndChild();
        ImGui::PopStyleVar();
    }

    // ---- the recorder's frame source -----------------------------------
    // Live while idle or capturing, recorded while playing or scrubbed. Here
    // rather than in the tab body, so a recording keeps advancing off-tab.
    if(!recording.empty() && (recPlaying || recPendingSeek))
    {
        if(recPlaying)
        {
            recPlayS += static_cast<Float64>(ImGui::GetIO().DeltaTime);
            if(recPlayS >= recording.durationS())
            {
                recPlayS = recording.durationS();
                recPlaying = false;    // holds on the last frame
            }
        }
        recIndex = recording.indexAt(recPlayS);
        recPendingSeek = false;

        const rec::Rev& r = recording.at(recIndex);
        LidarFrame lf;
        lf.points = r.points;
        lf.hz = r.hz;
        for(const LidarPoint& p : lf.points)
        {
            if(p.distMm > 0.0f)
            {
                ++lf.validCount;
            }
        }
        recView.push(lf);
    }

    // The preselect has to persist a few frames: a tab bar only honors
    // SetSelected once it has laid its items out, which is not on frame one.
    if(forceTabFrames > 0)
    {
        --forceTabFrames;
    }

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

    // clangd holds an index of the whole firmware in memory and would otherwise
    // outlive the window - an orphan of a few hundred megabytes.
    lsp::stop();

    // Last, so anything the two lines above logged on their way out is in the
    // file before it closes.
    applog::shutdown();
}
