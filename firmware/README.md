# firmware — Pico 2 W

`pico_debug` is the bring-up firmware: prove the toolchain, the USB link, and
the board, with the smallest thing that can be wrong. It blinks the LED and
answers commands over USB CDC.

This is the step *before* phase 2 (servo + ESC under code). See AGENTS.md.

---

## One-time setup

### 1. ARM toolchain — not installed on this machine yet

Everything else is already here (CMake 3.27, Ninja 1.12, Python 3.13). The only
missing piece is the cross-compiler. Pick either:

```powershell
# Option A - winget (installs Arm's official GNU toolchain)
winget install Arm.GnuArmEmbeddedToolchain
```

```bash
# Option B - MSYS2, which is already installed here
/c/msys64/usr/bin/pacman -S --needed mingw-w64-x86_64-arm-none-eabi-gcc
# then make sure /c/msys64/mingw64/bin is on PATH
```

Verify with `arm-none-eabi-gcc --version`. The toolchain is deliberately **not**
vendored — it is a compiler install, not a project dependency.

### 2. Pico SDK

Vendored like the other upstream clones, and gitignored:

```bash
git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git vendor/pico-sdk
cd vendor/pico-sdk
git submodule update --init --depth 1 lib/tinyusb lib/cyw43-driver lib/lwip
```

Pinned at **SDK 2.3.0**. `build.bat` points `PICO_SDK_PATH` at it, so no
environment variable is needed.

TinyUSB is required for USB CDC; the CYW43 driver is required to blink the LED
at all (see below).

---

## Build and flash

```bat
firmware\build.bat          REM add "clean" to wipe the build tree
firmware\flash.bat
```

`flash.bat` needs **no picotool**. The Pico SDK's USB stack exposes a reset
interface: opening its CDC port at **1200 baud** and closing reboots the board
into the UF2 bootloader, which mounts as a removable drive labelled `RPI-RP2`.
Copying the `.uf2` there flashes it and the board reboots itself.

If the board is already in BOOTSEL (or you held the button while plugging in),
the touch step is skipped automatically.

---

## The LED is on the wireless chip

On a **Pico 2 W** the user LED is **not** an RP2350 GPIO. It is driven by the
CYW43439, so it needs `cyw43_arch_init()` and `cyw43_arch_gpio_put()`. That is
why this firmware links `pico_cyw43_arch_none` just to blink.

On a non-W Pico the same LED is plain GPIO 25. Getting this wrong yields
firmware that runs perfectly and never blinks — which looks exactly like a dead
board.

`pico_cyw43_arch_none` brings the chip up without a networking stack. AP mode
and UDP arrive in phase 3; linking lwip now would cost flash and build time for
nothing.

---

## Serial protocol

USB CDC, newline-terminated ASCII, case-insensitive. Every command answers with
exactly one line beginning `OK` / `ERR` / `INFO` / `PONG`, so the host can tell a
silent board from a confused one.

| Command | Reply |
|---|---|
| `PING` | `PONG` |
| `ID` | `INFO id board=... sdk=... built=... uid=... cyw43=...` |
| `STATUS` | `INFO status up_ms=... led=... blink_hz=... cyw43=...` |
| `HELP` or `?` | several `INFO help ...` lines |
| `LED ON` / `LED OFF` | `OK led on` / `OK led off` |
| `LED BLINK <hz>` | `OK led blink <hz>` — `0` stops |
| `BOOTSEL` | reboots into the bootloader (no reply) |

Baud is irrelevant on CDC — **except 1200, which triggers the BOOTSEL reset.**
Do not use 1200 for normal traffic.

### Behaviour at power-on

Three fast flashes, then a slow 0.5 Hz heartbeat. That is deliberate: it is
proof of life *before* any host opens the port, so a board can be checked
without a laptop.

The `INFO ready ...` banner is printed when a host connects, not at boot —
anything written before the port is open is discarded by the USB stack.

### `cyw43=FAILED`

If `cyw43_arch_init()` fails the firmware still runs and still answers commands,
but the LED cannot light. It reports this rather than pretending. A board that
answers `PING` while reporting `cyw43=FAILED` is a very different problem from a
board that is silent.
