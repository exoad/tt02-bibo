# tt02-auto — project context

Read this before working in this repo.

## Project

Self-driving 1/10 scale RC car on a Tamiya TT-02 chassis (kit 58631, 1999 Subaru
Impreza Monte-Carlo). Goal is **teach-and-repeat autonomy**: map an environment
once by driving it manually, then localize against that map and drive it
autonomously. Long-term target includes operating outdoors on sidewalks, not
just indoors.

Owner is new to robotics and embedded, with a strong systems software
background. Prefers C, C++, Go. Python only where unavoidable.

## Current state

The car is **assembled and drives under radio control**. Chassis built, bearings
installed, motor mounted and meshed, shell trimmed and painted.

The RPLIDAR C1 is **working** — see `lidar/README.md`.

The **Pico 2 W toolchain is working on this machine**: build, flash, and read
back the board's flash, all from `firmware/`. `pico_debug` is flashed and
answering over USB CDC, and the LED blinks under command (`cyw43=up`). The GUI
in `hub/` is the command hub for all of it.

**The servo has still never moved under code — phase 2 is still the gate.**
But one thing in the original record was wrong and is worth stating plainly:
the board arrived carrying firmware called **`tt02_control`** which emits
1500 µs neutral on two channels and answers a `?` status command. Control
firmware exists and runs; what has never been demonstrated is a servo actually
moving. Its source is on the MacBook, not in this repo — only a read-back
binary at `vendor/tt02_control-backup.uf2` (gitignored). See `docs/log.md`.

---

## Architecture — three tiers

### Pico 2 W, on the car, always running

The only thing that must never miss a deadline.

- PWM to servo (GP0) and ESC (GP1) at 50 Hz
- Watchdog: no valid command in 200 ms -> throttle zero
- ToF bumper sensors on I2C; hardware-level stop that overrides anything above
- Encoder pulse counting for odometry
- Optionally a reactive lidar layer: parse the C1's UART stream, bucket points
  into angular sectors, keep min distance per sector. This is feasible on a Pico.

**SLAM on the Pico is not** — 520 KB RAM, no filesystem. That is a correct
division of labor, not a limitation to engineer around.

### Orange Pi 4 Pro 6GB (ordered) — perception and planning

Laptop fills this role until it arrives.

- Lidar over USB, full scan
- Localization or online SLAM depending on mode
- Path planning
- Outputs steer/throttle at ~20 Hz over UDP to the Pico

### Tower with RTX 3060 12GB — offline only

Map post-processing, model training, Gazebo simulation. **Never in the control
loop.**

---

## Perception modes — these stack, they don't replace each other

1. **Reactive.** No map. Sector minimums from lidar plus ToF bumpers. Wall
   following, gap detection, emergency stop. Always available, works anywhere.
2. **Online SLAM.** Builds a map as it drives, pose relative to start. Works
   outdoors, degrades gracefully in dynamic environments.
3. **Localization against a prior map.** Best accuracy. Map built offline from a
   manual drive, AMCL localizes, Nav2 plans. Indoor and fixed-course only.

**Reactive always runs underneath the other two.**

## Degradation chain — the core safety property

- Planning fails -> localization holds last pose, car holds its line
- Localization fails -> reactive layer prevents collisions
- Link fails -> watchdog stops the car
- Everything fails -> Flysky FS-GT2 radio kill switch (separate band, independent)

**The Pico's job stays deliberately small. Resist moving logic onto it.**

---

## Link

Pico 2 W runs its own AP with WPA2. Laptop joins directly, no router.

**UDP, not TCP** — newest command wins; retransmitting a stale command is worse
than dropping it. Every packet carries a rolling counter and an HMAC; the Pico
rejects failed verification or replayed counters.

FHSS on the RC link is interference resistance, **not security**. The RC receiver
also cannot carry arbitrary data — it outputs servo PWM only.

## Power — two isolated domains

- **Car:** NiMH pack -> ESC -> motor. BEC 5V -> servo, receiver.
- **Compute:** separate supply -> SBC -> USB out to Pico and lidar.

Joined by exactly one wire: **common ground**. Signal and ground cross between
domains; power never does.

> A missing common ground presents as a code bug and is not one.

---

## Hardware, confirmed

**Chassis:** Tamiya TT-02, ~440 x 190 x 130 mm with shell, ~1.35 kg stock.
Chassis tub ~90-100 mm wide — that is the usable deck space.

**Motor:** Tamiya 540 Torque Tuned (RS-540SH-7525). Uses a **19T pinion** per the
motor's supplementary sheet, **not** the 22T in the main kit manual. The smaller
pinion requires the closer motor mount position. Following the main manual leads
to a mount position where the gears cannot mesh. This cost real debugging time.

**ESC:** Hobbywing QuicRun/THW 1060, 60 A, 5V/2A BEC. Deans male battery
connector. Set battery type to **NiMH, not LiPo** — LiPo mode cuts off early on
this pack. Motor wiring: ESC yellow (+) -> motor yellow, ESC blue (-) -> motor
green. If the motor spins backwards, swap them; harmless. A spare kit ESC is held
as a known-good swap for fault isolation.

**Servo:** Power HD 1501MG. 17 kg/cm @ 6V, 0.14 s/60deg, deadband <= 4 us. Cable
is black/white with a white stripe on one outer conductor = signal. Middle pin is
+5V (servo convention, always). Other outer is ground.

**Battery:** 2x Tenergy 7.2V 3800 mAh NiMH, 383 g each, Tamiya male connector.
Rapid charge 1900 mA x 2.1 hr — use the charger's 2A setting.

**Adapter:** needed Deans **FEMALE** to Tamiya **MALE**. The first one purchased
was the wrong direction; the correct one is in hand and works.

**Radio:** Flysky FS-GT2 + FS-GR3E receiver. CH1 = servo, CH2 = ESC. BIND/CH3 and
VCC unused. Receiver is powered by the ESC's BEC through the CH2 lead.

**Bearings:** Yeah Racing YBS-0022 — 12x 1150 (5x11x4) + 4x 1280 (8x12x3.5). Kit
positions are BB1 x4 = 1280 metal, BB2 x4 = 1150 plastic, BB3 x8 = 1050 plastic.
The eight 1050 (5x10) positions are **not** covered by the Yeah Racing set — kit
plastic bushings stay in those.

**Dampers:** Tamiya 54753 CVA Super-Mini. Use CVA springs with CVA dampers; do
not mix with kit friction-shock springs. Kit springs are stiffer because friction
shocks provide no real damping.

**Shell:** trimmed, body posts reamed, painted Tamiya PS-4 Blue on the inside
(correct — clear lexan becomes the glossy outer layer). Windows and light
sections were accidentally painted over in the first pass, then scraped clear and
re-masked. PS-12 Silver backing coat ordered and pending — it goes over the blue
once cured, blocks light transmission, and makes the color read opaque instead of
translucent. **Lidar hole not yet cut.**

### Compute and sensors

- **Pico 2 W** (RP2350, micro USB variant) — the car's control board
- **3x spare Picos** (mix RP2040/RP2350) — bench mule and spares. **Not a
  cluster**; microcontrollers can't share memory and the interconnect is slower
  than one SBC core. Don't build an architecture to justify them.
- **ESP32-DevKitC** (CH340C) — comms offload if measurement shows it's needed
- **MicroSD module** (SPI, headers unsoldered — needs a 6-pin header)
- **RPLIDAR C1** — in hand and working, see `lidar/README.md`
- **Orange Pi 4 Pro 6GB** — ordered
- Planned: 2x VL53L1X ToF, magnetic wheel encoder, IMU
- LED lighting: MIBIDAO pre-wired RC light pairs, 3-7V, resistors already inline.
  Plus a ULN2003 driver — the Pico's total GPIO current budget (~50 mA) can't
  drive ten LEDs directly even at 3.3V.

---

## The application — `hub/`

`hub/` builds **one executable, `tt02.exe`**, and it is the front end for the
whole project: the fused sensor view, the Pico link and its command set, an
on-demand firmware build/flash/backup suite, and a console.

It started life as a point-cloud viewer under `lidar/`, which is why the git
history calls it that. It is not a lidar accessory - it is the application, and
it sits at the top of the tree to say so. Anything new that an operator drives
belongs inside it rather than beside it as another executable.

Two design rules that matter more than they look:

**The map is a fused sensor view, not a lidar view.** The RPLIDAR C1 is the only
thing feeding it today, but the four VL53L1X ToF sensors are static,
single-point, fixed-bearing devices that describe the same world. The UI lists
them as unwired layers rather than pretending the scanner is the world model, so
adding one is filling in a row rather than redesigning a screen.

**The lidar-to-vehicle transform is NOT established.** The map is sensor-centred
because that is the only frame currently justified. `docs/calibration.md` §2 is
an empty table, and the convention that 0° is the moulded arrow on the housing is
recorded as *assumed, not confirmed*. Inventing a plausible mounting offset would
silently rotate every fused reading from every sensor added afterwards, in a way
that looks like a sensor fault rather than a bad constant. Measure it first.

**Flashing goes through the scripts in `firmware/`, never around them.** The GUI
shells out to `build.bat` / `flash.bat` / `backup.bat` so there is exactly one
flashing mechanism. A second implementation inside the GUI would drift from the
terminal path and the two would disagree at the worst possible moment.

---

## Build order — each phase must work before the next

1. Kit assembled, driving under radio control — **done**
2. **Pico replaces receiver. USB serial commands drive servo + ESC. Watchdog
   added. THIS IS THE CURRENT GATE AND IT IS NOT STARTED.** Needs nothing that
   isn't already on hand.
3. Untether: Pico 2 W AP mode, UDP + HMAC. Add display and SD telemetry logging.
4. ToF reflex layer — hardware-level stop overriding any higher command
5. Wheel odometry (magnet + hall sensor), closed-loop speed control
6. Lidar + ROS2. SLAM offline, AMCL localize, Nav2 plan.

**Do not wire the ESP32 in until phase 3 has measured an actual problem.** Two
unknowns at once — PWM timing and UART framing — makes debugging impossible.
Adding it later is a two-wire change and nothing from phase 2 is wasted.

### Note on sequencing

The lidar is a phase 6 component and it's working at phase 1. That's fine, it's
validated. But **phase 2 is the gate for everything and it's untouched.** If
asked what to work on, phase 2 is the answer.

---

## Conventions

- Firmware: C, Pico SDK, CMake, PIO for PWM generation
- Host: Go. ROS2 layer: C++ (rclcpp)
- **Control code thinks in normalized units (-1.0 to 1.0), never microseconds.**
  The microsecond mapping lives in exactly one function so calibration is a
  single-constant change.
- **Never write to SD inside the control loop** — RAM ring buffer, periodic flush.
- Owner maintains a personal header-only C library and specific naming
  conventions. **Ask before introducing dependencies or restructuring includes.**

### C++ style — Jack's C++ Style Guide

`hub/` follows it. Applied 2026-08-25; match it in new code rather than the
surrounding history.

| | |
|---|---|
| Variables, functions, members, params | `camelCase` — no `m_`, no `g_`, no trailing `_` |
| Types, aliases, concepts, template type params | `PascalCase` |
| `constexpr` constants, macros, non-type template params | `SCREAMING_SNAKE_CASE` — no `k` prefix |
| Enum members | `SCREAMING_SNAKE_CASE`, **prefixed with the enum name** — `MapMode::MAP_MODE_POINTS` |
| Namespaces | lowercase — `ui`, `board`, `app` |
| Headers | `.hpp`, `#pragma once`. `.h` only for headers that must also compile as C — that is `firmware/` and `shared/shared.h` |
| Vocabulary | one place: `shared/shared.hpp` for C++, `shared/shared.h` for C. Both are on the include path of every target |
| Braces | Allman, everywhere. **Never one-lined** — a body never shares a line with its head, however short |
| Aggregate rows | a table row like `{ Icon::ICON_RADAR, "radar" },` stays on one line. That is *data*, not a body |
| Standard library | use the `shared/shared.hpp` aliases — `Vec`, `Str`, `Map`, `Mutex`, `Opt`. Never `std::vector` in our own declarations |
| Aggregate init | designated initializers where the type has named members — `Vec3{ .x = 1.0f, .y = 0.0f }` |
| Control keywords | `if(cond)`, not `if (cond)` |
| Casts | named casts only. **No C-style casts** |

## The central region has two layouts

**Tabbed** — one view at a time, full width. The default, and what to use when
you are looking at one thing.

**Floating** — every view is a panel on a pannable, zoomable board. Drag a title
bar to move, the bottom-right grip to resize, double-click or the `-` to
collapse, `x` to remove; the header chips put one back. Wheel zooms about the
cursor, dragging empty space (or middle-dragging anywhere) pans.

Both render the **same** view bodies via `drawViewBody()`. A tab and a panel
showing "2D" are one picture, not two implementations of it.

Zoom is **optical**: the same content, bigger or smaller, never reflowed. A
zoomed panel raises `ui::dpiScale()` by the same factor for the duration of its
draw, so padding, radii, line thicknesses and the editor's character cell all
grow together, and `ui::fontScale()` carries the text along. The layout then
occupies the same *proportion* of a panel that is itself larger.

Doing nothing would have given reflow instead - a bigger panel is more pixels,
so the editor would show more columns at the same size and the map would fit a
wider range. That is showing more, not zooming.

The scale is saved and restored **around each panel**, never set globally: the
sidebar and the status bar live outside the canvas and must not move when the
board zooms.

Layout, canvas and every panel rect persist in `%LOCALAPPDATA%/tt02-auto/panels.txt`.
`--layout floating` / `--layout tabbed` picks one at startup.

**A panel is ONE child window, frame included.** ImGui renders a parent's whole
draw list first and every child window afterwards, so a frame drawn into the
canvas list sits under *every* panel's content no matter how recently it was
raised - the order is right and the layering is not. Wrapping each panel's
frame, title, handles and content in its own child makes it a unit, and children
are sorted by `BeginOrderWithinParent`, which is submission order. Submit in z
order and the panel stacks whole.

**Two widgets cannot share a rectangle.** An ImGui item that owns `ActiveId`
makes every later overlapping item non-hoverable, which is silent - the later
widget simply never fires. It cost two bugs here: a full-canvas background button
disabled panel dragging, and a full-width title bar disabled its own fold and
close buttons. Overlap deliberately, or not at all.

**Types come from `hub/src/shared.hpp`** — `Int32`, `Float32`, `Bool`, `Void`,
`Size`, `Str`, `UniqPtr<T>`, `Opt<T>`. They are `using` aliases, so they are the
same types third-party signatures use; the boundary needs no conversion. Bare
`long` is deliberately NOT aliased: it is 32-bit on Windows and 64-bit on the
Linux/macOS targets, so pinning it either way would be wrong on one of them.

Two guide rules this repo cannot follow, both for the same reason:

- **"Never MSVC."** The Slamtec SDK ships `.vcxproj` files pinned to toolset
  v142 and its driver library is built `/GL` for MSVC LTCG. `hub/` links that
  library directly. Moving to GCC/Clang is a port, not a style change.
- **The GCC/Clang warning flags** (`-Wold-style-cast`, `-Wshadow`,
  `-Wnon-virtual-dtor`) have no MSVC equivalent. `build.bat` uses `/W4` and the
  tree is warning-clean at it. The cast rule is held by
  `hub/tools/style_audit.py` rather than by the compiler.

  `build.bat` actually shipped `/W3` from the style refactor until 2026-08-25,
  so "warning-clean at /W4" was an aspiration for several weeks rather than a
  fact. It is a fact now: raising the flag surfaced 12 warnings — a vestigial
  parameter and eleven dead locals left behind by the rename — and all twelve
  are gone.

---

## Lighting behavior — for whenever it gets built

Real car lighting, since the point is legibility at a glance.

- **Headlights** white, on when armed. Optional dim/bright for DRL vs headlight.
- **Tail and brake are the same red lamp**, PWM at 30% for tail and 100% for
  brake. Brake is not a separate light on most cars.
- **Turn signals** amber, **1.5 Hz** (roughly 400 ms on / 267 ms off) — that's
  the legal standard and what reads as correct.
- **Turn signal overrides brake on that side.** Braking while signaling right
  means the right rear alternates bright/off while the left stays solid. That
  asymmetry is what makes it look real.
- **Hazards** are both signals in phase, not alternating.
- Front and rear on the same side must **share one global timer**, not per-LED
  timers, or they drift apart.
- **Hazards on watchdog fire.** This is the debugging payoff — a failsafe visible
  from across the room without a laptop.

The Impreza's rear cluster has the reverse lamp nested inside the indicator
housing, so per side rear: red main, amber indicator, small white reverse.

---

## Repo layout

```
firmware/   Pico SDK C - runs on the car
host/       Go - laptop-side command + telemetry
sim/        bicycle model, controller experiments, no hardware
data/       telemetry CSVs pulled off SD (gitignored)
lidar/      RPLIDAR C1 viewer + notes
docs/       wiring.md, calibration.md, log.md
vendor/     upstream clones (gitignored) - rplidar_sdk lives here
```

---

## The firmware library

```
firmware/
  lib/
    tt02.h            the ONE header an application includes
    hal.h             the board: gpio, pwm, i2c, spi, serial, led, time
    gfx.h             drawing into a Screen
    drivers/
      display.h       ST7789 / ST7735 panel
      range.h         VL53L1X
      storage.h       SD over SPI
    chassis/
      chassis.h       steering + throttle, in fractions not microseconds
      cal.h           GENERATED - this car's measured numbers
  app/
    main.c            the serial console
  scratch/
    sketch.c          the Code view's scratch slot
```

### Rules, enforced by `hub/tools/style_audit.py`

**Includes point strictly downward.**

| a file in | may include |
|---|---|
| `lib/` | `shared.h` |
| `lib/drivers/` | `hal.h` |
| `lib/chassis/` | `hal.h`, `chassis/cal.h` |
| `app/`, `scratch/` | `tt02.h` — and nothing else of ours |

`lib/gfx.h` is the one written-down exception: it draws into a `Screen`, so it
reaches sideways into `drivers/display.h`. The Pico SDK (`pico/`, `hardware/`)
is exempt everywhere — it is not ours and is not a layer, and `hal.h` exists
precisely to be the file that reaches into it.

A driver that needed another driver would be two things wearing one name. The
moment that is allowed, the folders stop meaning anything — which is why this is
a build failure and not a guideline.

**Application code includes `tt02.h` and nothing else.** Reaching past it to a
specific header still compiles and is still wrong: it makes every file's
dependencies something you have to read the top of the file to know, and a
header that moves then breaks callers that had no business naming it.

**Every public symbol carries its module's prefix** — `gpio`, `pwm`, `i2c`,
`spi`, `serial`, `led`, `tft`, `gfx`, `vl53`, `sd`, `drive` — so a call site
says which layer it reaches into without anyone looking it up.

**Safety lives in the module, not the caller.** `chassis.h` refuses throttle
until armed and returns `Bool`; it never prints. A console, a sketch and an
autonomy loop each carrying their own copy of that rule is three copies, and the
day one of them forgets is the day it matters.

**Header-only, `static inline`.** Deliberate on a microcontroller: the compiler
sees through `gpioWrite()` and emits the single store it actually is. The cost
is compile time rather than link time, which at this size is free. The state in
`chassis.h` is file-scope, so each firmware image must stay a single translation
unit — two would give you two chassis and one car.
