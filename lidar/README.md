# RPLIDAR C1

Working and validated. Live 360° scans on Windows, via a self-contained Dear
ImGui viewer that links Slamtec's SDK driver directly.

This gets repeated on **macOS** and on the **Orange Pi**, so everything below is
specific rather than approximate.

```
lidar/
  viewer/   Dear ImGui + DirectX 11 live point cloud viewer (the main tool)
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
    lidar/viewer/third_party/imgui
```

**Pin ImGui to `v1.92.9`.** Master is a moving target and 1.92 reworked the font
system — `PushFont` is now two-argument, and a base size is a *pre-scale* value
that ImGui multiplies by the global DPI factors. Code here depends on that.

### Windows

```bat
lidar\viewer\build.bat
lidar\viewer\build\rplidar_imgui.exe
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
| `--range <metres>` | Pin the view instead of auto-fitting |

### Requirements

- Visual Studio 2022 with **MSVC v142** toolset (14.29) — the SDK `.vcxproj`
  files specify `v142`; v143 alone is not enough without retargeting
- Windows SDK (d3d11, dxgi, dwmapi)
- Dear ImGui **v1.92.9**, vendored at `viewer/third_party/imgui`

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

## Viewer

One self-contained exe. Links `rplidar_driver.lib` and runs the scan loop on a
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

### What the map draws

- Scan points as small flat dots, current revolution brightest, previous four
  fading behind it
- **Blind zone** — hatched disc at the 0.05 m spec floor, drawn true to scale
- **Heading arrow** along bearing 0, matching the moulded arrow on the housing
- Range rings stepped 1/2/5 × 10ⁿ, centred on the **sensor** so they stay correct
  when it is panned off-centre, plus compass ticks and a scale bar
- **12 m envelope ring** marking the far end of the spec range

Angle 0 is up and increases clockwise. **This is the assumed convention and has
not been physically confirmed against the moulded arrow yet** — see
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

The scan core is **dToF** — it times picosecond-scale light return, and both the
reference oscillator and the detector propagation delay drift with die
temperature. Cold readings sit outside the calibrated point.

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
   backends in `viewer/third_party/imgui/backends/`, and `lidar_source.cpp` uses
   Win32 for `CreateFileA` port probing and `QueryDosDeviceA` enumeration, both
   of which need POSIX equivalents (`open()` and globbing `/dev/tty.*`).
   `radar.cpp`, `theme.cpp` and `app_ui.cpp` are portable as written.
