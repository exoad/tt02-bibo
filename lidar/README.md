# RPLIDAR C1

Working and validated. Live 360° scans on Windows, via a self-contained Dear
ImGui viewer that links Slamtec's SDK driver directly.

This gets repeated on **macOS** and on the **Orange Pi**, so everything below is
specific rather than approximate.

The viewer is **not here** - it is the whole project's application and lives at
`hub/`, which builds `bibo.exe`. This directory holds the sensor's own notes and
a standalone CLI tool.

```
lidar/
  bridge/   CLI tool: streams scan frames as text on stdout, no GUI
```

The SDK itself lives in `vendor/rplidar_sdk/` and is **gitignored**.

---

## The two facts that cost the most time

1. **460800 baud.** Not 115200. The SDK examples often default to 115200 and
   produce garbage or a failed connect. The C1 will not talk at the default.
2. **The motor does not spin until the SDK issues a start-scan command.** A
   stationary motor on USB connect is normal. The adapter LED *does* light
   immediately on connect — that is the sign power is reaching it.

## Connection

| | |
|---|---|
| **Port (this machine)** | **COM7** — device path `\\.\COM7` |
| **Baud** | **460800** |
| USB-serial chip | **CP2102** (Silicon Labs) |
| VID/PID | `VID_10C4&PID_EA60` |
| Model / firmware / hardware | `0x41` / 1.02 / rev 18 |
| Serial | `A11FE18AC1EA9ED2B29C92F522BB466C` |

**Driver:** Windows auto-installed it. **macOS needs the Silicon Labs VCP driver
installed manually** — without it the device does not enumerate at all and looks
dead. See [../docs/log.md](../docs/log.md).

**Connector (XH2.54-5P):** pin order is **VCC, TX, RX, GND** = red, yellow,
green, black. One of the five positions is unpopulated — correct, not a fault.

### Re-finding the port

The port number moves between USB sockets and reboots.

```powershell
Get-CimInstance Win32_PnPEntity | Where-Object { $_.Name -match 'CP210x' } |
  Select-Object Name, DeviceID
```

The viewer does this itself: it reads `HKLM\HARDWARE\DEVICEMAP\SERIALCOMM` and
picks the `Silabser*` entry. On this machine the other four ports are
`BthModem*` (Bluetooth), so a "highest-numbered port" guess would eventually pick
wrong.

macOS: `ls /dev/tty.* | grep -Ei 'usbserial|SLAB'`

---

## Build and run

### First time on a fresh machine

Both upstream dependencies are gitignored and must be fetched. From the **repo
root**:

```bash
git clone https://github.com/Slamtec/rplidar_sdk.git vendor/rplidar_sdk
git clone --depth 1 --branch v1.92.9 https://github.com/ocornut/imgui.git \
    hub/third_party/imgui
```

**Pin ImGui to `v1.92.9`.** Master is a moving target and 1.92 reworked the font
system — `PushFont` is now two-argument, and a base size is a *pre-scale* value
that ImGui multiplies by the global DPI factors. Code here depends on that.

### Windows

```bat
hub\build.bat
hub\build\bibo.exe
```

`build.bat clean` wipes the build tree first. It builds the SDK driver library if
missing, compiles ImGui + the win32/dx11 backends + `src\*.cpp` with `/MT` (to
match the SDK lib's static CRT), and links with `/LTCG` (the SDK lib is `/GL`).

**Auto-connects on launch** — no arguments needed, so double-clicking the exe
works.

| Flag | Effect |
|---|---|
| *(none)* | Auto-detect port, 460800, connect |
| `--connect [port] [baud]` | Pin a specific port/baud |
| `--no-connect` | Start disconnected |
| `--range <meters>` | Pin the view instead of auto-fitting |
| `--view <map\|pico>` | Central view to open on |
| `--map <mode>` | Map render mode, by its lower-case name (`points`, `rays`, `distance`, `contour`, `density`, `occupancy`, `clearance`, `motion`, `sectors`, `gaps`, `validity`, `sweep`, `walls`, `full`) |
| `--tab <system\|sensors\|vehicle\|firmware\|console>` | Sidebar section to expand and scroll to |

`--view` picks what the **center** shows; `--tab` picks what the **right sidebar**
scrolls to. They are independent, and no word means both.

### Central views

Top tabs above the center panel. The map is one view among several rather than
the only thing the window can show — the board tab is a live pinout that cannot
drift out of date the way a diagram in a document does, because the project's own
pin assignments are compiled in beside the physical pinout.

- **Map** — the lidar map, below.
- **Pico 2 W** — the board to scale (51 × 21 mm, 2.54 mm pitch), drawn as it
  looks: green solder mask, white silkscreen, gold castellations, black
  packages, the shield can over the radio. All 40 pads hoverable.

Because every pad on the real board is gold, the pin category (*assigned /
power-ground / free GPIO*) moved out of the pad fill and onto a short mask
stripe just inboard of each pad, plus the label outside the board.

**The board tab shows live state, not just a pinout:**

| Drawn | Means | Source |
|---|---|---|
| LED lit / dark / blinking | the actual LED state, animated at the reported rate | `INFO status ... led= blink_hz=` |
| LED hollow outline | nothing has told us what it is doing | — |
| green pip on the shield can | `cyw43=up`; red means FAILED | `INFO status ... cyw43=` |
| green ring on the USB shell | the CDC link is open | `PicoLink` |
| amber BOOTSEL button | the board is in the bootloader, so every other reading is stale | `picotool` / drive detect |
| green ring on pads 1 and 2 | GP0/GP1 are being driven, with the µs in the tooltip | `S` line from `tt02_control` |

Two rules the drawing follows:

- **Unknown is not off.** An LED nobody has asked about is drawn hollow; an LED
  reported off is drawn dark and filled. Defaulting unknowns to off would make a
  board nobody has talked to look like one that answered and said no.
- **`blink_hz` beats `led=`.** The firmware reports the LED's *instantaneous*
  level and its blink rate, so `led=off blink_hz=0.50` means it is blinking and
  the sample caught the dark half. Observed live on the bench, and the reason the
  parser treats a non-zero rate as authoritative.

The blink phase is **not** synced to the board — nothing tells us where in its
cycle it is. What the drawing claims is "it is blinking, at about this rate",
which is exactly what the reported state supports.

The app polls `STATUS` every 2 s, but only while this tab is on screen, and stops
permanently after one refusal — `tt02_control` answers `ERR bad command` and must
not be asked forever.

### Sensor telemetry (Sensors workspace)

Split across tabs rather than made scrollable — there is more telemetry than
fits at once, and nothing in this app scrolls.

- **Live** — rotation rate, points/revolution, in-spec %, nearest/mean/max range,
  a rotation-rate sparkline, and a 12-sector clearance chart.
- **Signal** — the revolution broken into *in spec / no return / < 50 mm /
  beyond 12 m*, which is what makes the in-spec percentage interpretable rather
  than merely low. Plus the per-measurement signal quality the device reports.
- **Scan** — the scan mode the SDK negotiated, its sample period and implied
  sample rate, the mode's own range ceiling, measured angular resolution,
  angular coverage, and a range histogram.
- **Device** — identity, health, and session counters (uptime, revolutions,
  measurements, dropped revolutions, average rate), plus a pre-heat indicator
  that goes green at 2 minutes.

**Observed: the C1 reports a constant quality of 47** for every in-spec return —
mean 47.0, range 47–47. The quality histogram is therefore a single bar. That
appears to be the device's behavior rather than a bug in the reader; it is worth
knowing before relying on quality to detect weak returns.

### Requirements

- Visual Studio 2022 with **MSVC v142** toolset (14.29) — the SDK `.vcxproj`
  files specify `v142`; v143 alone is not enough without retargeting
- Windows SDK (d3d11, dxgi, dwmapi)
- Dear ImGui **v1.92.9**, vendored at `hub/third_party/imgui`

### The CLI bridge (optional, no GUI)

```bat
lidar\bridge\build.bat
lidar\bridge\build\lidar_bridge.exe \\.\COM7 460800
```

Prints `INFO`/`HEALTH`, then one line per revolution:
`F <count> <freq_mHz> <a>,<d>,<q> ...` — angle in centi-degrees, distance in mm,
quality 0–63. A line starting with `q` on stdin, or EOF, requests a clean
shutdown, so it always stops the motor when its parent dies.

Launched with no stdin attached (e.g. redirected from `/dev/null`) it prints the
header and exits immediately with no frames. That is the EOF path working, not a
failure.

---

## The map

One self-contained exe (`hub/build/bibo.exe`). Links `rplidar_driver.lib` and runs the scan loop on a
worker thread — no helper process, no reimplemented serial protocol.

### Mouse

| Gesture | Action |
|---|---|
| Left / middle drag | Pan |
| Wheel | Zoom about the cursor |
| Ctrl / Shift + wheel | Fine / coarse zoom |
| Double-click left | Reset to auto-fit |
| Right drag | Measure — distance shown on the line |
| Hover | Bearing + range readout |

### The bottom bar belongs to the view

Each central view declares its own controls, and **none** is a valid answer. The
map's bar (render modes, grid/trail/labels, range) configures the map and nothing
else, so on a board tab it is not disabled — it is absent, and the board gets the
height back. A view that later grows a control declares its row count in
`centralControlRows()` and draws it in `drawCentralControls()`; no other code
changes.

### Render modes

Switched with the segmented row under the map. All five draw the same in-spec
window and the same furniture (rings, ticks, arrow, blind zone); only the marks
differ.

Fourteen of them. **Hover any button for what it draws and how to read it** — the
labels alone do not say, and were never going to. The strip keeps two rows and
scrolls sideways once cells would be too narrow to read rather than shrinking
them further. Each mode also tints the viewer background toward its own palette,
so the active mode is legible from the map and not only from the toggle.

| Mode | Marks | Reads |
|---|---|---|
| **Points** | flat dots, current revolution brightest | geometry, unweighted |
| **Rays** | a line from the sensor to each return | which bearings actually got a return, and where the shadows are |
| **Distance** | dots ramped by range | near/far structure at a glance |
| **Contour** | adjacent returns joined into polylines | surfaces as surfaces — a wall reads as a wall, not as the dots it produced |
| **Density** | hit counts in a fixed 6 cm world grid over a 40-revolution (~4 s) window | what is *stable*: a wall saturates to cream, something that moved leaves a low-count blue smear |
| **Occupancy** | the same grid, exponentially decayed (τ = 9 s) | short-term memory of the space, with the live revolution drawn over it |
| **Clearance** | smoothed free-space polygon, one radius per 3° bearing bin | how far it is safe to drive on each bearing — the only mode about the *empty* space |
| **Motion** | grid cells hit now whose whole neighborhood the memory map says was empty | what changed. The complement of Density: that shows what stayed put, this shows what did not |
| **Sectors** | twelve 30° wedges, each out to its nearest obstacle, numbers on a ring | the navigation view. Coarse on purpose — the resolution a steering decision works at |
| **Gaps** | openings wide enough to drive through, with their width in meters | follow-the-gap, drawn. Width is the chord at the opening's *nearest* edge, so it is what would actually have to fit |
| **Validity** | why each bearing failed: no return / under 0.05 m / past 12 m | explains the in-spec percentage instead of reporting it |
| **Sweep** | returns colored by position within the revolution | the scan takes ~100 ms, not zero. Makes skew from a moving robot visible |
| **Walls** | straight segments *fitted* to the returns, with lengths | one step past Contour: that joins the dots, this decides which runs actually are a straight surface. The landmarks a SLAM front end keys on |
| **Full** | every derived layer at once, with the numbers drawn on the map | the field display — for when the laptop is the only instrument you have and nothing may be hidden behind a panel or a hover |

**Every mode prints its own reading** in the HUD, under the mode name: the widest
gap and its bearing, the tightest sector, the full in-spec breakdown, the sweep
duration. A mode without a number is a picture.

Two notes on the ones that surprised me:

- **Sectors labels sit on a fixed ring, not at the end of their wedge.** The
  wedge length is the datum and stays true to scale — but with the sensor among
  clutter most sectors are a few centimeters and their wedges are a dozen pixels,
  which put every number in an unreadable pile at the origin.
- **A sector with nothing inside 12 m** would be drawn at 12 m, off screen at any
  normal zoom, silently deleting most of the rose. Those clamp to the view edge
  and get a *broken* arc, so "the wall is here" and "the view ends here" cannot
  be confused.

Four of these deliberately trade latency for stability, because a map that
flickers cannot be read:

- **Density** normalizes against a **fixed** count, never the current frame's
  maximum — normalizing against a running maximum would make the same count mean
  a different brightness from frame to frame, which is the one thing a heatmap
  may not do.
- **Clearance** closes instantly and opens slowly. An obstacle that appears two
  meters ahead is reported at two meters on the frame it appears; a free-space
  map that lags an obstacle is worse than none. A bin that gets no return for a
  single revolution **holds** rather than opening — at 3° bins and ~505
  points/rev that is four samples a bin, and an isolated empty bin is noise, not
  evidence. (At 1.5° it was two samples, a quarter of bins came up empty by
  chance, and every one drew a spike to the range ceiling.)
- **Motion** requires the whole 3×3 neighborhood to be cold, not just the cell.
  Angular jitter lands a stationary wall's returns in slightly different cells
  each revolution, so a bare cell-was-empty test lights up every wall edge
  permanently.
- **Contour** breaks a run when the gap between consecutive returns exceeds a
  threshold that **grows with range**. At 0.72° the arc between neighboring
  samples is ~13 mm at 1 m and ~150 mm at 12 m, so a fixed threshold either
  shatters every distant surface or bridges across nearby doorways.

The clearance polygon is drawn as a triangle fan from the sensor, not via
`AddConvexPolyFilled` — a room's profile is emphatically not convex, and that
call would fill straight across every doorway and alcove. It *is* star-shaped
about the sensor by construction (one radius per bearing), so the fan is the
exact shape rather than an approximation.

### What the map draws

- Scan points as small flat dots, current revolution brightest, previous four
  fading behind it
- **Blind zone** — hatched disc at the 0.05 m spec floor, drawn true to scale
- **Heading arrow** along bearing 0, matching the molded arrow on the housing
- Range rings stepped 1/2/5 × 10ⁿ, centered on the **sensor** so they stay correct
  when it is panned off-center, plus compass ticks and a scale bar
- **12 m envelope ring** marking the far end of the spec range

Angle 0 is up and increases clockwise. **This is the assumed convention and has
not been physically confirmed against the molded arrow yet** — see
[../docs/calibration.md](../docs/calibration.md).

### In-spec window: 0.05 – 12 m

Everything outside it is excluded from the map **and** from every readout.
Whatever is not counted is not drawn. Below the floor the returns are reflections
off the unit's own housing; above the ceiling they are unreliable.

`valid %` therefore means "in-spec", not "non-zero", and reads a few points lower
than a raw `dist > 0` count would.

---

## Before trusting measurements: pre-heat 2+ minutes

Run it with the motor spinning for at least two minutes first.

The scan core is a **time-of-flight** ranger — the C1M1 datasheet rev 1.1 (p. 4)
says "laser flight-of-time (TOF) ranging principle … combined with the
high-speed laser acquisition and high-precision fusion algorithm". Timing
references and detector chains drift with die temperature, so a cold reading
sits outside whatever point the unit was calibrated at.

This used to say "dToF … picosecond-scale". That was inferred from other
RPLIDAR models, not read from the C1's datasheet. The two-minute figure below is
measured from this unit and is unaffected; the explanation was the borrowed
part.

This matters for SLAM specifically: a drifting offset during the first minutes
corrupts early map geometry *relative to* later geometry, which is exactly the
error SLAM cannot reconcile.

Also skip the spin-up transient — the first ~15 revolutions run fast and sparse
(~355 points at ~14 Hz) before settling.

---

## Specifications

| | |
|---|---|
| Range | 0.05 – 12 m white, 6 m black |
| Accuracy | ±30 mm |
| Angular resolution | 0.72° |
| Scan rate | 10 Hz typical |
| Sample rate | 5 KHz |
| **Ambient tolerance** | **40 klux** |
| Ingress | IP54 |
| Mass | 110 g |
| Size | 55.6 × 55.6 × 41.3 mm |
| Current | ~230 mA |

Observed live: **9.8 Hz**, 508–513 points/revolution, ~5000 points/s, 73–83%
in-spec returns indoors, max indoor range ~7.6–7.9 m.

### 40 klux is the known obstacle to the outdoor goal

Direct sunlight exceeds it and degrades the sensor. Sidewalk operation is a
stated goal, so this decides how much of the outdoor plan survives.

Mitigations in rough priority order:

1. Operate in shade or overcast
2. Fuse a sunlight-tolerant single-point sensor (TF-Luna, 70 klux — but 0.2 m
   minimum range, so it does not replace the bumper ToFs)
3. Lean on wheel odometry to carry through dropouts
4. Optical bandpass filter
5. Hardware upgrade — last resort

**Characterise this now, not in month six.** Procedure in
[../docs/calibration.md](../docs/calibration.md).

---

## Known bugs in Slamtec's SDK

Both are in vendored upstream code and are worked around, not patched.

1. **`drv->connect()` reports success for a port that never opened.** In
   `sdk/src/sl_async_transceiver.cpp` an inner `Result<nullptr_t> ans` shadows
   the outer `u_result ans`, so an `open()` failure never reaches the caller. A
   nonexistent COM port therefore reports "wrong baud rate?", pointing at the
   wrong cause. The viewer probes the port with `CreateFileA` first so the three
   failure modes — missing port / port in use / wrong baud — report distinctly.

2. **The demo projects cannot build x64.** All four configurations hardcode the
   library search path to `output\win32\$(Configuration)` while `OutDir` uses
   `$(Platform)`, so an x64 build writes the lib to `output\x64\` and the linker
   looks in `output\win32\`. Fails with `LNK1181`. Build **Win32** for the demos.
   The driver library itself builds fine at x64, which is what the viewer links.

## Other gotchas

- **RoboStudio is Windows-only and did not detect the device** even with a valid
  COM port present and the SDK talking to that same port. The SDK command-line
  path is the working route.
- **Git Bash mangles `\\.\COM7` into `\.\COM7`.** `MSYS2_ARG_CONV_EXCL='*'` does
  not fix it. Run from `cmd.exe`/PowerShell or put the argument in a `.bat`. A
  bare relative `build.bat` is also not found from bash — use the absolute path.

---

## Porting to macOS / Orange Pi

Not yet run — a starting point, not a verified recipe.

1. **Install the Silicon Labs CP210x VCP driver on macOS.** Without it the device
   does not enumerate and looks dead.
2. **SDK demo** — the top-level Makefile picks the platform from `uname`, and the
   Windows-only x64 bug does not apply:
   ```bash
   cd vendor/rplidar_sdk && make
   ./output/Darwin/Release/ultra_simple --channel --serial /dev/tty.usbserial-XXXX 460800
   ```
3. **CLI bridge** — `lidar_bridge.cpp` uses only `<thread>`/`<chrono>` and the
   SDK, no Win32 calls, so it should compile as-is: `./lidar/bridge/build.sh`
4. **The viewer needs porting work.** `main.cpp` is Win32 + DirectX 11 and
   `build.bat` is MSVC-specific. ImGui ships `imgui_impl_osx` + `imgui_impl_metal`
   backends in `hub/third_party/imgui/backends/`, and `lidar_source.cpp` uses
   Win32 for `CreateFileA` port probing and `QueryDosDeviceA` enumeration, both
   of which need POSIX equivalents (`open()` and globbing `/dev/tty.*`).
   `radar.cpp`, `theme.cpp` and `app_ui.cpp` are portable as written.
