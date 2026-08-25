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

**No Pico work has been done. The servo has never moved under code.** That is
phase 2 below, and it is the gate for everything else.

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
