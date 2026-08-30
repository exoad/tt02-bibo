# bibo — project context

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

The **toolchain works on this machine** for both boards: build, flash, and read
back the flash, all from `firmware/`. There are two RP2350s now — a Pico 2 W
that is the breadboard mule and a plain Pico 2 that goes in the car. One
firmware builds for either; see [wiring.md](wiring.md).

**Phase 2 is done.** The servo moves under code, sweeps its full travel on
command, and is calibrated on this car: left 1230, **centre 1484**, right 1670.
The ESC is verified and idle is measured at 1541. Centre is not 1500 and the
throw is asymmetric, which is why every command above the calibration is a
fraction rather than a microsecond count.

Four LEDs are on the car — two tail lamps and one pair of indicators — driven
by a lighting model in `firmware/lib/lights.hxx` that computes ten lamps whether
or not an LED exists for each. Two of the four pins are borrowed, and **GP15 is
the wheel encoder's**, so it goes back when the Hall sensor is fitted.

Everything the library exposes is listed in
[firmware-api.md](firmware-api.md).

What is still missing is the **sensing**: no encoder, no IMU, nothing on the
I²C bus, and the lidar is not mounted. Nothing closes a loop yet, which is why
there is no PID.

The board arrived carrying firmware called **`tt02_control`**, which emits
1500 µs neutral on two channels and answers a `?` status command. Its source is
on the MacBook, not in this repo — only a read-back binary at
`vendor/tt02_control-backup.uf2` (gitignored). See `docs/log.md`.

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

**Motor:** Tamiya 540 Torque Tuned (RS-540SH-7525).

The motor's supplementary sheet calls for a **19T pinion**, **not** the 22T in
the main kit manual, and the smaller pinion needs the closer motor mount
position. Following the main manual leads to a mount position where the gears
cannot mesh, which cost real debugging time.

**Gearing: 17T pinion / 70T spur.** With the TT-02's 2.60:1 internal ratio that
is a **10.71:1** final drive, down from 9.58:1 on the 19T — **+11.8% torque at
the wheels, -10.5% top speed**.

Nothing in the firmware or the hub knows the gear ratio, and nothing should: the
ESC is commanded in microseconds and a gear change does not alter what a pulse
width means to it. Two things downstream of it are affected and are written down
here so they are not rediscovered:

- **`THROTTLE_CAL_MIN` is a measurement that this invalidates.** It is idle - the
  pulse at which the motor sits still and the next microsecond starts it turning
  - and it was found by winding the throttle up until the wheels moved. More
  reduction means the drivetrain breaks static friction at a *lower* pulse, so
  the old 1541 is probably now above the real idle. Re-measure on a stand.

  This is not cosmetic. `driveThrottleUs` clamps *upward* to `escMin`, and the
  deadman decides the car is being driven by `escTargetUs > escMinUs` - so a car
  that creeps at what the firmware calls idle is a car the deadman would not
  consider to be moving.

- **Where the encoder magnet goes.** On the wheel or axle, counts per wheel
  revolution are a fact about the wheel and no gear change ever touches them. On
  the spur or the motor, every ratio change silently invalidates the odometry
  calibration. The axle is the answer; this is worth deciding before it is
  glued, not after.

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
- **Magnetic wheel encoder** - Hall sensor and magnets in hand, not yet fitted.
  Its pin (GP15) is currently lent to a tail lamp.
- Planned: 2x VL53L1X ToF, IMU
- LED lighting: MIBIDAO pre-wired RC light pairs, 3-7V, resistors already inline.
  Plus a ULN2003 driver — the Pico's total GPIO current budget (~50 mA) can't
  drive ten LEDs directly even at 3.3V.

---

## The application — `hub/`

`hub/` builds **one executable, `bibo.exe`**, and it is the front end for the
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

- Firmware: **C++**, Pico SDK, CMake, PIO for PWM generation. It was C until
  2026-08-28; every `.c`/`.h` under `firmware/` is now `.cxx`/`.hxx` and the
  module prefixes became real namespaces
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
| Headers | `.hxx`, `#pragma once` — **no `#ifndef` guards**. There are no `.h` headers of ours left; `hub/src/resource.h` is the one exception, because `rc.exe` compiles it |
| Vocabulary | `shared/shared.hxx` for the hub, `firmware/lib/types.hxx` for the firmware. Two files kept in step by hand — not because one is C any more, but because the firmware is freestanding: no heap, no exceptions, no STL, so every template in `shared.hxx` is unusable there. Nothing ever includes both |
| Braces | Allman, everywhere. **Never one-lined** — a body never shares a line with its head, however short. That includes a body with no braces at all: `if(x) return;` is a body on its head's line |
| Namespaces | Allman brace, and the body **indented one level** — see below |
| Aggregate rows | a table row like `{ Icon::ICON_RADAR, "radar" },` stays on one line. That is *data*, not a body |
| Standard library | use the `shared/shared.hxx` aliases — `Vec`, `Str`, `Map`, `Mutex`, `Opt`. Never `std::vector` in our own declarations |
| Arrays | `Array<T, N>`, never `T name[N]`. A raw array decays to a pointer the moment it is passed and the size stops travelling with it |
| Time | `Clock`, `TimePoint`, `Millis`, `Duration`, and the `monoNow` / `elapsedMs` / `elapsedS` / `sleepMs` helpers. Never `std::chrono::steady_clock::now()` in our own code |
| Files | `InFile` / `OutFile`. Most file reading here is C stdio and needs no alias |
| `*` and `&` | bind to the **type**, not the name — `Char* p`, `const Str& s`. Never `Char *p` |
| Only TYPES are aliased | `std::move`, `std::min`, `std::sort`, `std::chrono::duration_cast` keep their own spelling. A function is not vocabulary |
| Aggregate init | designated initializers where the type has named members — `Vec3{ .x = 1.0f, .y = 0.0f }` |
| Control keywords | `if(cond)`, not `if (cond)` |
| Casts | named casts only. **No C-style casts** — see below |

### `Array<T, N>`, and the one place the vocabulary stops being free

Every alias in `shared.hxx` is a `using` declaration, so a `Float32` **is** a
`float` and handing one to ImGui is not a conversion. `Array<T, N>` is the
exception and the only one: `std::array<Char, 64>` is **not** `char[64]`, so
every C API it reaches gets `.data()` and `.size()`.

That was paid knowingly on 2026-08-30 — 175 declarations, 123 of them `Char`
scratch buffers feeding `snprintf` and ImGui. It is worth it because a raw
array decays to a pointer at the first call and its length becomes something
the caller has to know by other means, which is how a buffer and its `sizeof`
drift apart.

**`sizeof` is the trap.** For a one-byte element `sizeof(a)` and `a.size()`
agree; for anything wider they differ by `sizeof(T)`. Getting that backwards
divides a buffer by four and still compiles.

A `Char` buffer initialised from a string literal has no clean equivalent:
`= ""` becomes `{}` (identical zero-fill) and `= "--"` becomes `{'-', '-'}`,
but a long literal is better written as an explicit `snprintf` of the fallback
than as a list of character constants.

### There is no `Ptr<T>`, deliberately

An alias for a raw pointer was considered and rejected. `template<typename T>
using Ptr = T*;` reads well until `const` appears, and then it silently means
the wrong thing:

```cpp
const Ptr<Char>   p;   // Char* const  - a const POINTER to mutable chars
const Char*       q;   // what everyone writing the line above meant
```

`Ptr<const Char>` is the pointer-to-const one, and the two spellings differ by
where a word sits rather than by anything a reader would notice. Raw `*`
syntax has no such ambiguity.

It would not carry meaning either. Ownership is already spelled — `UniqPtr`,
`SharedPtr`, `WeakPtr` — so a bare `T*` in this codebase already means
"borrowed, may be null", and `Ptr<T>` would rename that without adding to it.
Where a pointer type genuinely deserves a name it gets a specific one:
`CharSeq` is `const Utf8*`, and it says what it is for.

### A namespace is a block, and its body is indented

```cpp
namespace ui
{

  Void draw()
  {
  }

}
```

not

```cpp
namespace ui {
Void draw()
{
}
}
```

**Two spaces per namespace level**, not four. Most of `firmware/lib` sits two
namespaces deep — `bibo::lights`, `bibo::gpio` — and at four spaces every real
line would start eight columns in before it had said anything. Two keeps the
nesting visible without spending the width.

The brace goes on its own line for the same reason it does everywhere else: a
namespace is a block, and this project does not have one brace rule for blocks
and a second one for the block that contains them all.

The honest reason this is written down rather than assumed: the tree was split
almost exactly in half. 45 files wrote `namespace ui {` and 46 wrote the brace
on its own line, and **not one file indented a body**. The half that was
already Allman only stayed that way by accident — `style_audit.py`'s check that
each module declares its namespace was anchored at column 0, so it enforced the
brace placement as a side effect while nobody had decided it.

Applied 2026-08-30, tree-wide: 32,722 lines, every one of them whitespace. It
is held by `hub/tools/style_audit.py` under `--- namespace layout ---`.

### File extensions ARE the language

| | |
|---|---|
| `.cxx` `.hxx` | C++. **Everything of ours** — `hub/`, `lidar/bridge/`, `shared/`, and `firmware/` since the 2026-08-28 conversion |
| `.c` `.h` | C. **Nothing of ours any more.** `hub/src/resource.h` is the single exception, and `rc.exe` compiles it |

`.cpp` and `.hpp` are not used. Neither is a bare `.h` outside `firmware/`, and
the style audit fails on all three.

The reason is not taste. `.h` was doing double duty as "a C header" and "a
header nobody thought about", and **most editors assume an unowned `.h` is
C++** — which is how a perfectly correct C header ends up underlined for using
`NULL` instead of `nullptr`, and how `(UInt8)` in a file that cannot contain a
C++ cast gets flagged as a C-style cast. Splitting the extensions means the
language is knowable from the filename by a tool that has loaded nothing.

`resource.h` is the one exception: `rc.exe` compiles it, which is neither a C
nor a C++ compiler, and renaming it would break the resource build to satisfy a
rule about headers.

### Named casts, everywhere the language has them

`static_cast`, `reinterpret_cast`, `const_cast`. Never `(Type) x`.

A C-style cast is four different casts wearing one spelling, and which one you
get depends on the types — so it silently becomes a `reinterpret_cast` the day
somebody changes a declaration three files away. The named forms say what was
meant and fail when the meaning stops being available.

MSVC has no `-Wold-style-cast`, so this is held by `hub/tools/style_audit.py`
rather than by the compiler.

C used to be carved out, because C has no named casts and banning `(Int64) x`
there would have banned casting. **That carve-out is spent.** The doc said
"when a file moves from C to C++, the waiver stops with it", and on
2026-08-30 the bill arrived: eight C-style casts, every one of them in
`firmware/`, converted. There are no C files left to waive.

**The rule is enforced on SYNTAX, not on a list of type names.** It used to
enumerate the types a cast could be *to*, which meant it only found casts to
types somebody had remembered to add — and it missed `(MINMAXINFO*)lparam`,
`(const RECT*)lparam` and `(sl_u32)baud` for as long as those existed. What
makes a cast recognisable is its shape, and the hard part — telling a type
from an expression — is settled by this project's own naming: types are
PascalCase, so `(Foo)x` is a cast and `(width) * 2` is arithmetic. Win32
shouts, the standard library uses `_t`, Slamtec uses `sl_`.

### A parameter list never wraps

In a **definition or declaration**. One line, however long:

```cpp
Void emitDiscs(ImDrawList* dl, const Dot* dots, Int32 count, Float32 r, ImU32 col, const ImVec2& uv)
```

not

```cpp
Void emitDiscs(ImDrawList* dl, const Dot* dots, Int32 count,
               Float32 r, ImU32 col, const ImVec2& uv)
```

A signature is a contract. One that has to be reassembled across lines before it
can be read is one people stop reading, and a parameter added to the second line
of a wrapped list is a parameter nobody reviewing the first line will see.

**Call sites may wrap and often should.** A call's arguments are expressions, and
an expression is allowed to be long — the rule is about the contract, not about
every parenthesis in the file.

The honest cost: 91 signatures were unwrapped when this rule was written and
**47 of them now exceed 100 columns**, the worst at 167. Those are not a
formatting problem, they are a design one — a function taking eleven parameters
was hard to read wrapped as well, and the wrapping was hiding it. Shortening
those signatures is real work and is not done.

### Application code uses the library, not libc

`firmware/lib` wraps the C standard library so the project has one vocabulary:

| instead of | use | from |
|---|---|---|
| `printf` | `serial::printf` | `hal.hxx` |
| `puts` / `fputs` | `serial::printLine` / `serial::print` | `hal.hxx` |
| `strcmp(a,b)==0` | `text::eq` | `text.hxx` |
| `strncmp(s,p,n)==0` | `text::starts` | `text.hxx` |
| `atoi` / `atof` | `text::toInt` / `text::toFloat` | `text.hxx` |
| `sscanf("%d %d")` | `text::twoInts` | `text.hxx` |
| `toupper` loop | `text::upper` | `text.hxx` |
| `snprintf` | `text::format` | `text.hxx` |

The wrappers cost nothing — they are macros or `static inline` and compile to
exactly what they wrap. **The point is that the seam is complete.** `hal.hxx` is
where the SDK's spelling stops and this project's begins, and a console calling
`printf()` directly was the one place reaching past it — sixty-two times. The
day the transport is not stdio (a UDP link, a log to the SD card, both at once)
that is sixty-two call sites to find instead of one definition to change.

`app/` and `sketches/` include **`../lib/bibo.hxx` and nothing else**, and name no
libc function at all. The style audit checks this and names the replacement.

**`lib/` is exempt, because it is where the wrapping happens.** `text.hxx` naming
`strtol` is the wrapper doing its job.

### `static inline` is `static`, in C++

`static` on a free function already gives it internal linkage, and a static
function is only emitted where it is used — so `inline` tells a C++ compiler
nothing it did not know.

**C is the exception and keeps `static inline`.** There it is the standard idiom
for a definition in a header: without `static` every translation unit emits a
copy and they collide at link time, and without `inline` the compiler is not
asked to inline it.

**This no longer applies to `firmware/`, because `firmware/` is no longer C.**
The C++ conversion dropped every one of them: there are now 329 plain
`static` definitions in `firmware/lib` and **zero** `static inline`. The only
occurrence left in the tree is the phrase inside `hal.hxx`'s own header
comment, describing a design the file no longer has.

This is worth knowing because most editors assume an unowned `.h` is C++ and
will flag every one of them as redundant. That is the editor being wrong about
the language, not the code being wrong — and it is the same misreading that
makes `NULL` and `(UInt8)` look like mistakes in a C header. Attaching the
firmware CMake project fixes all three at once; see [clion.md](clion.md).

## The central region is tabbed

One view at a time, full width, selected from the tab bar. That is the only
layout.

There used to be a second one - a "floating" mode where every view was a panel
on a pannable, zoomable board, with drag-to-move, a resize grip, fold and close
buttons and a z-order. It was removed on 2026-08-28, along with
`hub/src/workspace.cxx`, its header and its test. It worked; it was simply a
second way to look at the same pictures, and the cost of a second way is that
every new view has to be designed, laid out and debugged in both. The view
bodies were already shared through `drawViewBody()`, so removing the panels
removed a layout, not a feature.

What went with it: the `--layout` command-line switch, and the `L`, `C` and `P`
records in `%LOCALAPPDATA%/bibo/panels.txt`. Older files still parse -
unknown record letters are skipped - so a settings file written by a build that
had panels loads without complaint and simply forgets where they were.

Note that the **sidebar's** section tear-off is a different thing and is still
there: the `float` button on a section header pops that section out. It shares
no code with the removed workspace.

**A view is ONE child window, frame included.** ImGui renders a parent's whole
draw list first and every child window afterwards, so a frame drawn into the
parent list sits under *every* child's content no matter how recently it was
raised - the order is right and the layering is not. Wrapping a frame, title and
content in its own child makes it a unit.

**Two widgets cannot share a rectangle.** An ImGui item that owns `ActiveId`
makes every later overlapping item non-hoverable, which is silent - the later
widget simply never fires. It cost two bugs here: a full-canvas background button
disabled panel dragging, and a full-width title bar disabled its own fold and
close buttons. Overlap deliberately, or not at all.

**Types come from `hub/src/shared.hxx`** — `Int32`, `Float32`, `Bool`, `Void`,
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
- **Turn signals** amber, **100 flashes per minute** — 360 ms on / 240 ms off.
  Deliberately not 50/50: a slightly longer on than off is what real flasher
  cans do and what the eye expects.

  This said 1.5 Hz (400/267) until 2026-08-30 and that was wrong. 667 ms is
  89.96 flashes per minute, and the normally-closed band in SAE J945 has a
  **floor of 90** — so the "legal standard" figure was just under the legal
  minimum. The firmware holds the band with a `static_assert`; `hub/src/
  lights.hxx` mirrors the two numbers and must keep doing so, since the board
  decides what the lamps do and the hub only draws the same blink.
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
    bibo.hxx            the ONE header an application includes
    hal.hxx           the board: gpio, pwm, i2c, spi, serial, led, time
    gfx.hxx           drawing into a Screen
    status.hxx        the onboard LED as a readable signal
    drivers/
      display.hxx     ST7789 / ST7735 panel
      range.hxx       VL53L1X
      storage.hxx     SD over SPI
    chassis/
      chassis.hxx     steering + throttle, in fractions not microseconds
      cal.hxx         GENERATED - this car's measured numbers
  app/
    main.cxx          the serial console
  sketches/
    *.cxx             one file per sketch, each its own build target
```

### Rules, enforced by `hub/tools/style_audit.py`

**Includes point strictly downward.**

| a file in | may include |
|---|---|
| `lib/` | `types.hxx`, `hal.hxx` — hal is the floor everything stands on |
| `lib/drivers/` | `../hal.hxx` |
| `lib/chassis/` | `../hal.hxx`, `cal.hxx` |
| `app/`, `sketches/` | `../lib/bibo.hxx` — and nothing else of ours. The `../lib/` is part of the rule: the bare spelling compiles only because `-Ifirmware/lib` is set |

`lib/gfx.hxx` is the one written-down exception: it draws into a `Screen`, so it
reaches sideways into `drivers/display.hxx`. The Pico SDK (`pico/`, `hardware/`)
is exempt everywhere — it is not ours and is not a layer, and `hal.hxx` exists
precisely to be the file that reaches into it.

A driver that needed another driver would be two things wearing one name. The
moment that is allowed, the folders stop meaning anything — which is why this is
a build failure and not a guideline.

Every public function, setting and type is listed in
[firmware-api.md](firmware-api.md), including which of them persist across a
reflash and what the five parts of a properly-finished setting are.

**Application code includes `bibo.hxx` and nothing else.** Reaching past it to a
specific header still compiles and is still wrong: it makes every file's
dependencies something you have to read the top of the file to know, and a
header that moves then breaks callers that had no business naming it.

**Wrap the SDK once, in `hal.hxx`.** If application code is reaching for
`cyw43_arch_gpio_put`, `getchar_timeout_us` or `pico_get_unique_board_id`, the
HAL has a gap — fill it there rather than at the call site. And **derive
sentinels, never restate them**: `SERIAL_NONE` was written as `-1` because that
is what a sentinel looks like, the SDK says `-2`, and the result was a command
buffer filling with bytes nobody typed.

**Ask the SDK which board this is; never hard-code one.** The firmware builds
for two — `pico2_w` (the breadboard mule) and `pico2` (the car) — and the only
place that difference is allowed to appear is behind the HAL. `hal.hxx` switches
the LED on `CYW43_WL_GPIO_LED_PIN` versus `PICO_DEFAULT_LED_PIN`, both of which
come from the SDK's board header; `CMakeLists.txt` links the wireless arch on
`PICO_CYW43_SUPPORTED`, which the SDK lifts out of that same header. Matching on
a trailing `_w` in the board name would have been shorter and would have been a
rule this project invented about somebody else's naming.

The corollary is a wiring rule: **GP23, GP24, GP25 and GP29 are off limits.**
They are board functions on the Pico 2 and the wireless chip's own pins on the W,
so anything that claims one works on one board and quietly drives a chip select
on the other. Nothing here uses them, which is exactly what makes the two boards
drop-in for each other — see [wiring.md](wiring.md).

**Every module is a NAMESPACE** — `gpio`, `pwm`, `i2c`, `spi`, `serial`,
`led`, `radio`, `adc`, `watchdog`, `timing`, `board`, `tft`, `tof`, `sd`,
`drive`, `gfx`, `text`, `lights`, `cue`, `net`, `status` — all inside
`bibo`. A call site says which layer it reaches into without anyone looking
it up.

This used to be a spelling rule, because the library was C and C has no
namespaces: every symbol carried its module in its name, `gpioWrite`. Since
the C++ conversion the boundary is real, and the compiler enforces what a
prefix could only suggest. What the audit still checks is the thing the
compiler cannot: that each module HAS its namespace and that it is the one
everybody expects. A header that quietly stops declaring one still compiles
— its symbols simply move to the global namespace, one file at a time,
which is exactly how the prefixes decayed before anything checked them.

**Safety lives in the module, not the caller.** `chassis.hxx` refuses throttle
until armed and returns `Bool`; it never prints. A console, a sketch and an
autonomy loop each carrying their own copy of that rule is three copies, and the
day one of them forgets is the day it matters.

**Header-only, `static`.** Deliberate on a microcontroller: the compiler
sees through `gpioWrite()` and emits the single store it actually is. The cost
is compile time rather than link time, which at this size is free. The state in
`chassis.hxx` is file-scope, so each firmware image must stay a single translation
unit — two would give you two chassis and one car.
