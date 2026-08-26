# tt02-auto

Self-driving 1/10 scale RC car on a Tamiya TT-02 chassis. Teach-and-repeat
autonomy: map an environment once by driving it manually, then localize against
that map and drive it autonomously.

Full project context, hardware inventory and architecture: **[docs/conventions.md](docs/conventions.md)**

## Status

| | |
|---|---|
| Chassis | Assembled, drives under radio control |
| RPLIDAR C1 | Working — live scans, see [lidar/README.md](lidar/README.md) |
| Pico 2 W | Toolchain, build, flash and flash-backup all working from this machine |
| Pico firmware | `pico_debug` flashed and answering; LED blinks under command |
| **Servo / ESC** | **Never moved under code. This is still the gate.** |

**Current gate: phase 2** — Pico replaces the receiver, USB serial commands drive
servo + ESC, watchdog added. Everything else waits on it.

One correction to the original project record: the board arrived carrying
firmware called `tt02_control` that emits 1500 µs neutral on two channels and
answers a `?` status command, so control firmware *does* exist and run. What has
never been demonstrated is a servo actually moving. Its source is on the MacBook,
not in this repo — only a read-back binary in `vendor/`. See
[docs/log.md](docs/log.md).

## Layout

```
hub/        THE APPLICATION. One executable, tt02.exe - the operator
            front end for the whole project. Lidar map, Pico link and
            command set, firmware build/flash/backup, console.
firmware/   Pico SDK C - runs on the car
  src/main.c    pico_debug: LED + serial command console
  catalog.txt   images the viewer's Firmware tab can flash
  build.bat / flash.bat / backup.bat
host/       Go - laptop-side command + telemetry
sim/        bicycle model, controller experiments, no hardware
data/       telemetry CSVs pulled off SD (gitignored)
lidar/      RPLIDAR C1 notes and a standalone CLI tool
  bridge/     scan frames as text on stdout; diagnostics without the GUI
docs/
  wiring.md       Pico pin map and wiring invariants
  calibration.md  lidar calibration procedures + measured values
  log.md          build log, failures first
vendor/     upstream clones and binaries (gitignored)
  rplidar_sdk/      Slamtec SDK
  pico-sdk/         Raspberry Pi Pico SDK 2.3.0
  picotool-2.3.0/   official prebuilt; the SDK-built one crashes here
  tt02_control-backup.uf2   read back off the board; ONLY copy on this machine
```

## Getting set up on a fresh machine

`vendor/` is gitignored, so clone the SDK before building anything under `lidar/`:

```bash
git clone https://github.com/Slamtec/rplidar_sdk.git vendor/rplidar_sdk
```

You will also want the Pico SDK and picotool to build firmware — see
[firmware/README.md](firmware/README.md), which lists the one-time toolchain
install and the exact clone commands.

Then:

```bat
hubuild.bat
hubuild	t02.exe
```

It auto-detects and connects both devices on launch. See
[lidar/README.md](lidar/README.md) for the lidar's COM port and the baud rate
that matters, and [firmware/README.md](firmware/README.md) for the Pico.

## The application

`hub/` builds **one executable, `tt02.exe`**, and it is the front end for the
whole project. Launch it and everything is reachable from that one window:

- **Lidar** — live map with pan/zoom/measure, plus scan telemetry
- **Vehicle** — the Pico link and its command set
- **Firmware** — build, flash and back up the board on demand, from a catalog
  (`firmware/catalog.txt`). It shells out to the same scripts that work from a
  terminal, so there is one flashing mechanism rather than two that can drift
- **Console** — everything the board and the build scripts say

Flashing needs no picotool for the happy path, and **backing up the board's
flash first is one click** — worth doing before loading anything you cannot
rebuild from source.

## Safety

- The car goes **on a stand, wheels off the ground**, for every first run of new
  code.
- Common ground between Pico and ESC is **mandatory**. Its absence looks like a
  software bug and is not one.
- Never connect BEC 5V to the Pico while USB is attached.
- The Flysky radio is the independent kill switch, on a separate band. It is the
  last link in the degradation chain and does not depend on any code in this repo.

## Licence

**This project is not open source.** Copyright (c) 2026 Jiaming Meng, all rights
reserved. The source is here for reference; no licence to use, copy, modify or
redistribute it is granted. See [COPYRIGHT](COPYRIGHT).

That covers the original work only. The dependencies and vendored assets each
carry their own terms, and two of them bind any distributed **binary**:

- **Fugue Icons** (CC BY 3.0) - the app's icons. Attribution is a *condition*,
  and it has to stay visible to anyone who receives a build.
- **Slamtec rplidar_driver** (BSD-2-Clause) - statically linked into `tt02.exe`,
  so its copyright notice has to ship with the binary.

Full inventory, and what each licence actually obliges:
**[THIRD_PARTY.md](THIRD_PARTY.md)**.

## Editor / IDE

CLion setup, and the reason there are two CMake projects rather than one:
**[docs/clion.md](docs/clion.md)**. If every symbol shows as unresolved, that
page opens with the fix.
