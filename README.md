# bibo

Self-driving 1/10 scale RC car on a Tamiya TT-02. Teach-and-repeat: map an
environment once by driving it manually, then localize against that map and
drive it autonomously.

**Current gate: phase 2** — the Pico replaces the receiver and drives servo and
ESC over USB serial. Everything else waits on it.

Full context, hardware and architecture: **[docs/conventions.md](docs/conventions.md)**

## Layout

```
hub/        the application - bibo.exe. Map, Pico link, firmware flashing, console
firmware/   Pico SDK C++ - runs on the car
lidar/      RPLIDAR C1 notes and a standalone CLI
host/       Go - laptop-side command + telemetry
sim/        bicycle model, no hardware
docs/       wiring, calibration, build log
vendor/     upstream clones (gitignored)
```

## Build

`vendor/` is gitignored, so clone the SDK first:

```bash
git clone https://github.com/Slamtec/rplidar_sdk.git vendor/rplidar_sdk
```

```bat
hub\build.bat && hub\build\bibo.exe
```

It auto-detects and connects both devices on launch. Firmware toolchain and the
one-time install: [firmware/README.md](firmware/README.md).

## Safety

- The car goes **on a stand, wheels off the ground**, for every first run of new code.
- Common ground between Pico and ESC is **mandatory**. Its absence looks like a
  software bug and is not one.
- Never connect BEC 5V to the Pico while USB is attached.
- The Flysky radio is the independent kill switch, on a separate band.

## Licence

**Not open source.** Copyright (c) 2026 Jiaming Meng, all rights reserved. See
[COPYRIGHT](COPYRIGHT).

Two dependencies bind any distributed **binary**: Fugue Icons (CC BY 3.0,
attribution must stay visible) and Slamtec rplidar_driver (BSD-2-Clause, notice
must ship). Full inventory: [THIRD_PARTY.md](THIRD_PARTY.md).
