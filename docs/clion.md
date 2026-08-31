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
`firmware/lib/hal.h`.

---

## If every INCLUDE in firmware/ is red

Symptom, in `hal.h` or any firmware source:

```
#include "bibo.hxx"          Cannot find file 'bibo.hxx' in search paths: ...\firmware\app.
#include "hardware/adc.h"  Cannot find directory 'hardware' in search paths: ...
```

**Nothing is wrong with the code or the build.** `firmware/build/compile_commands.json`
lists ~208 translation units with every one of those paths on them. The file
simply belongs to no *loaded* project, so the parser falls back to searching the
file's own directory and nothing else.

`firmware/` is a **separate CMake project** and has to be attached - it
cross-compiles for `arm-none-eabi` while the root is MSVC x64, and one CMake
configure has exactly one toolchain.

**Fix:** `File | Attach Project…` → `firmware/CMakeLists.txt` → choose the
**Pico 2 W (RP2350, arm-none-eabi)** preset. That also gets you building and
flashing from the IDE.

**How to tell it worked.** The CMake tool window gains a second root next to the
hub's, and the run-configuration dropdown gains `sketch` and `pico_debug`. If
neither appears, the attach did not take — reattaching is harmless.

**What is still red if you do NOT attach**, and why each one is the same
problem wearing a different hat:

| symptom | cause |
|---|---|
| `Cannot find directory 'hardware'` / `'pico'` | no SDK include path |
| `Cannot resolve symbol 'printf'`, `'strcmp'`, `'atof'` | no C standard library include path either — the cross-compiler's, which only the project knows |
| `C-style cast is used instead of a C++ cast` inside a `.h` | an unowned `.h` is assumed to be C++. The headers are now listed as sources on their targets, so this resolves with the project attached |

The project types (`Int32`, `UInt16`, `Utf8`) come from `firmware/lib/types.h`,
which sits next to `hal.h` and is included by it. That one resolves even with no
project loaded, because a quoted include is searched relative to the including
file first — which is why it lives in the library rather than in a `shared/`
directory it was never actually shared with.

The library's own `"bibo.hxx"`, `"drivers/…"` and `"chassis/…"` includes are
different: they need `-Ifirmware/lib`, which comes from the project or from the
compile database. `firmware/.clangd` exists for exactly that gap. Everything
below needs the cross-toolchain as well, and that only comes from the project.

`firmware/.clangd` is a fallback for the same problem, pointing the parser at
that compile database directly. It helps where the IDE falls back to clangd; it
is not a substitute for attaching the project.

The database is `firmware/build/compile_commands.json`, and **`firmware\build.bat`
writes it**. If firmware includes are unresolved and attaching has not helped,
check that file exists — a `build.bat clean` deletes `build\` and it is not
recreated until the next configure. It carries `-Ifirmware/lib`, which is what
resolves `"bibo.hxx"` and the library's own `"drivers/…"` and `"chassis/…"`
includes.

## If every symbol is red

That is the symptom of CLion having opened this folder as a plain directory
rather than as a CMake project — it happens when the folder was first opened
before a `CMakeLists.txt` existed. CLion does not convert on its own.

**Fix:** right-click the root `CMakeLists.txt` in the Project tree →
**Load CMake Project**.

Nothing else is needed. CLion auto-detects the Visual Studio 2022 toolchain, and
`CMakePresets.json` supplies the rest.

---

## If `Int32` is red in firmware/ but the includes are fine

A different fault from the two above, and the giveaway is that it survives
attaching the project. `firmware/lib/types.hxx` fails on `#include <cstddef>`,
so every alias it defines — `Int32`, `UInt8`, `Bool`, `Void` — is an unknown
type name in every firmware file at once.

**The cause is that clangd will not run the cross-compiler.** The build
compiles firmware with `arm-none-eabi-g++`, so clangd correctly targets
`arm-none-eabi` — but the system headers for that target are wherever that
compiler keeps them, and the only way to find out is to ASK it. clangd will not
execute a compiler it has not been told is safe, which is a sensible default
when a `compile_commands.json` can name any binary on disk.

**Fix:** allow that one driver, by exact path.

```
--query-driver=C:/msys64/mingw64/bin/arm-none-eabi-g++.exe
```

Where it goes depends on the editor, because **it is a clangd command-line
flag and not a config key**. `firmware/.clangd` cannot carry it: clangd 18.1.8
answers `Unknown CompileFlags key 'QueryDriver'` and carries on without it.
Checked, rather than assumed.

- **VS Code** — `"clangd.arguments": ["--query-driver=C:/msys64/mingw64/bin/arm-none-eabi-g++.exe"]`
- **CLion** — Settings → Languages & Frameworks → C/C++ → Clangd, in the
  additional-flags box
- **Anything else** — wherever that editor spells "extra clangd arguments"

Use the exact path rather than a glob. The flag is a whitelist of programs
clangd may execute, and the one it needs is the compiler the build already
runs.

**It does not fail loudly.** Without the flag clangd still answers questions —
from an identifier index rather than a parsed AST — so completion returns
plausible-looking nonsense (`printf` and `define` offered after `dfplayer::`)
and nothing announces that the file never compiled. Found on 2026-08-31 while
wiring clangd into the Code view, where it showed up as a namespace offering a
handful of completions instead of its real members.

---

## 1. The hub

Open the repo root. CLion reads `CMakePresets.json` and offers one profile:

- **Hub (MSVC x64)** → `build/cmake`

It needs CLion's **Visual Studio** toolchain, not MinGW; `hub/CMakeLists.txt`
stops with a clear error rather than half-configuring against anything else.

Targets you get: `bibo`, plus `test_editor`, `test_map_geometry` and
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
rest — 208 translation units.

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
