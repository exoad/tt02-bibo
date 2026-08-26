# CLion setup

There are **two CMake projects** in this repo and they cannot be one:

| | toolchain | runs on |
|---|---|---|
| `hub/` | MSVC x64, Win32 + DX11 | this machine |
| `firmware/` | `arm-none-eabi`, RP2350 | the car |

A single CMake configure has exactly one toolchain, so the cross-compiled half
cannot be a subdirectory of the native half. The root `CMakeLists.txt` therefore
covers the hub, and the firmware is **attached** beside it. Both then index at
once, and Ctrl-clicking `gpioOpen()` from a sketch lands in
`firmware/src/pico2w.h`.

---

## If every symbol is red

That is the symptom of CLion having opened this folder as a plain directory
rather than as a CMake project — it happens when the folder was first opened
before a `CMakeLists.txt` existed. CLion does not convert on its own.

**Fix:** right-click the root `CMakeLists.txt` in the Project tree →
**Load CMake Project**.

Nothing else is needed. CLion auto-detects the Visual Studio 2022 toolchain, and
`CMakePresets.json` supplies the rest.

---

## 1. The hub

Open the repo root. CLion reads `CMakePresets.json` and offers one profile:

- **Hub (MSVC x64)** → `build/cmake`

It needs CLion's **Visual Studio** toolchain, not MinGW; `hub/CMakeLists.txt`
stops with a clear error rather than half-configuring against anything else.

Targets you get: `tt02`, plus `test_editor`, `test_map_geometry` and
`test_lights` with runnable gutter arrows. `ctest` runs all three.

`hub/build.bat` is still the authoritative build. The CMake file uses the same
flags — `/MT`, `/W4`, `/LTCG`, the same wildcard over `src/` — so the two agree,
but if they ever disagree the script wins.

## 2. The firmware

**File → Attach Project…** → pick `firmware/CMakeLists.txt`, then choose the
preset:

- **Pico 2 W (RP2350, arm-none-eabi)** → `firmware/build`
- **Pico 2 W (Debug)** → `firmware/build-debug`, `-O0` with symbols

These mirror `firmware/build.bat`, including the three settings that took a day
to find and are documented at length in that script's header:

- `PICO_TOOLCHAIN_PATH=C:/msys64/mingw64` rather than putting `mingw64\bin` on
  `PATH`, which shadows ucrt64's runtime and makes the SDK's own host tools
  (`pioasm`) die silently mid-build
- `picotool_DIR` pointing at Raspberry Pi's **prebuilt** picotool, because the
  one the SDK builds from source here crashes with an access violation in every
  subcommand that touches an ELF
- Visual Studio's `ninja.exe`, because MSYS2 ships two incompatible CMakes and
  both fail on this machine

Indexing the SDK gives completion on `gpio_put`, `pwm_set_gpio_level` and the
rest — 191 translation units.

---

## 3. Run configurations

Committed under `.idea/runConfigurations/`, so they appear in the run dropdown:

| | what it does |
|---|---|
| **Flash sketch** | `firmware/build/sketch.uf2` → the board |
| **Flash pico_debug** | the bring-up image; this is the way back |
| **Back up board flash** | reads the board's flash to a `.uf2` in `vendor/` |
| **Build firmware (build.bat)** | the authoritative firmware build |
| **Build hub (build.bat)** | the authoritative hub build |
| **Style audit** | the naming rules from `docs/conventions.md`, as a gate |

All of them shell out to the existing `.bat` scripts rather than reimplementing
anything, so there is still exactly one build mechanism and one flash mechanism.

Flashing does not need the BOOTSEL button: `flash.ps1` touches the port at 1200
baud, which the Pico's USB reset interface reads as "reboot into the bootloader".

---

## 4. What is NOT set up

**On-chip debugging.** Stepping through firmware on the RP2350 needs a hardware
probe (a second Pico running `debugprobe`, or a CMSIS-DAP unit) wired to SWD.
There isn't one, so there is no OpenOCD configuration here. Adding one when a
probe exists is an Embedded GDB Server configuration, not a change to any of the
files above.

Until then, debugging is `serialPrintf()` and the hub's console — which is what
`pico_debug` and the USB CDC link are for.
