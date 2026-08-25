# tt02-auto

Self-driving 1/10 scale RC car on a Tamiya TT-02 chassis. Teach-and-repeat
autonomy: map an environment once by driving it manually, then localize against
that map and drive it autonomously.

Full project context, hardware inventory and architecture: **[AGENTS.md](AGENTS.md)**

## Status

| | |
|---|---|
| Chassis | Assembled, drives under radio control |
| RPLIDAR C1 | Working — live scans, see [lidar/README.md](lidar/README.md) |
| **Pico** | **Nothing yet. The servo has never moved under code.** |

**Current gate: phase 2** — Pico replaces the receiver, USB serial commands drive
servo + ESC, watchdog added. Everything else waits on it. The lidar being done
early is fine, but it is not the critical path.

## Layout

```
firmware/   Pico SDK C - runs on the car
host/       Go - laptop-side command + telemetry
sim/        bicycle model, controller experiments, no hardware
data/       telemetry CSVs pulled off SD (gitignored)
lidar/      RPLIDAR C1 viewer + notes
  viewer/     Dear ImGui + DirectX 11 live point cloud viewer
  bridge/     CLI tool: scan frames as text on stdout
docs/
  wiring.md       Pico pin map and wiring invariants
  calibration.md  lidar calibration procedures + measured values
  log.md          build log, failures first
vendor/     upstream clones (gitignored)
  rplidar_sdk/  Slamtec SDK
```

## Getting set up on a fresh machine

`vendor/` is gitignored, so clone the SDK before building anything under `lidar/`:

```bash
git clone https://github.com/Slamtec/rplidar_sdk.git vendor/rplidar_sdk
```

Then build and run the viewer — see [lidar/README.md](lidar/README.md) for the
exact commands, the COM port, and the baud rate that matters.

## Safety

- The car goes **on a stand, wheels off the ground**, for every first run of new
  code.
- Common ground between Pico and ESC is **mandatory**. Its absence looks like a
  software bug and is not one.
- Never connect BEC 5V to the Pico while USB is attached.
- The Flysky radio is the independent kill switch, on a separate band. It is the
  last link in the degradation chain and does not depend on any code in this repo.
