# The firmware library

Everything the car can do, and the one include that gets you all of it.

```c
#include "../lib/bibo.hxx"     /* and nothing else of ours */
```

`bibo.hxx` is the whole library. Reaching past it to a specific header compiles
and is still wrong — see [conventions.md](conventions.md), and the style audit
that enforces it.

The library is header-only and every module holds **one copy** of its state, so
a module belongs to a single translation unit. That is deliberate: the car has
one drivetrain and one set of lamps, and a second instance of either would be a
bug wearing a design.

---

## What is a SETTING and what is a MEASUREMENT

The distinction runs through the whole library and decides where a number
lives.

A **measurement** is a fact about this particular car, found by moving it — the
pulse at which the wheels point straight, the pulse at which the motor starts
turning. There is no way to know it except by looking.

A **setting** is a choice about how the car should behave — how fast an output
may move, how much throttle counts as going somewhere. A different answer is
right on a bench than on the ground, and neither is wrong.

Both live in `lib/chassis/cal.h`, because that is the file that survives a
reflash, and both are settable at run time so they can be found without a
rebuild. The hub's Drive view writes that file; see **Persisting** below.

---

## Settings, and the whole surface for each

Every one of these has the same shape: a setter that validates, a getter or a
field in `driveRead()`, a serial command, a slider in the hub, and a macro in
`cal.h` that survives a reflash. If any of those five is missing for a number
you care about, that is a gap and not a style.

| Setting | Set with | Read from | Serial | cal.h |
|---|---|---|---|---|
| Steering response | `driveSetSteerSlew(us)` | `steerSlewUs` | `SLEW STEER <us>` | `SLEW_CAL_STEER` |
| Throttle response | `driveSetThrottleSlew(us)` | `throttleSlewUs` | `SLEW THROTTLE <us>` | `SLEW_CAL_THROTTLE` |
| Both at once | `driveSetSlew(us)` | — | `SLEW <us>` | — |
| Steering travel | `driveSetSteerLimits(lo,hi)` | `servoMinUs` / `servoMaxUs` | `SERVOLIMITS <lo> <hi>` | — |
| Straight ahead | `driveTrim(us)` | `centerUs` | `SERVOTRIM <us>` | `STEER_CAL_CENTER` |
| Throttle range | `driveSetThrottleLimits(lo,hi)` | `escMinUs` / `escMaxUs` | `ESCLIMITS <lo> <hi>` | `THROTTLE_CAL_MIN/MAX` |
| Lamps-off point | `lightsSetOffThreshold(us)` | `lightsOffThreshold()` | `LIGHTS OFFAT <us>` | `LIGHT_CAL_OFF_US` |

### Response rates — why there are two

`driveSetSteerSlew` and `driveSetThrottleSlew` are separate settings, in
microseconds of pulse per 20 ms tick, bounded by `SLEW_MIN_STEP` (1) and
`SLEW_MAX_STEP` (200).

They shared one number until 2026-08-28, and that was wrong in a way that only
shows up when you try to tune it: making the steering quick made the throttle
violent, and gentling the throttle made the steering vague. A servo should
arrive promptly — a steering correction that lands late is a correction applied
to a car that has already gone past the thing. An ESC should be led there:
throttle slammed on spins the wheels, slammed off pitches the car onto its
nose, and a brushed motor asked for a step change draws a spike the BEC feels.

The units are per-tick because that is what the code does; the hub and the
`SLEW` reply both also print µs/s and a full-travel time, because per-tick is a
number nobody has intuition about.

Note the ranges differ enormously. The steering has ~440 µs of travel and the
throttle ~59 µs, so a rate that feels gentle on the steering crosses the whole
throttle band in a fraction of the time.

---

## chassis — the drivetrain

`lib/chassis/chassis.h`. One servo, one ESC, and three rules it will not let a
caller break.

**The three rules**, enforced here so no caller can forget them:

1. **The steering is RELEASED at boot** — not neutral, released, no pulse at
   all. Driving neutral the instant USB power arrives assumes 1500 µs is a safe
   place for the linkage, and on a car whose horn is a tooth out the servo
   picks up the frame before anyone has typed anything.
2. **The ESC will not move unless armed.** `driveThrottleUs` returns `false`
   when it is not, rather than quietly doing nothing.
3. **Nothing jumps.** Every output is walked toward its target by `drivePump()`
   at the response rate above.

```c
Void driveOpen(Void);                  /* once, at startup                    */
Void drivePump(Void);                  /* often - this is what actually moves */
Void driveStop(Void);                  /* neutral, disarmed, released         */
DriveState driveRead(Void);            /* a snapshot, taken all at once       */
```

### Steering

```c
Void driveEngage(Bool on);             /* start/stop driving the pin          */
Void driveSteer(Float32 frac);         /* -1..+1 of THIS car's travel         */
Void driveCenter(Void);
Void driveSteerUs(Int32 us);           /* raw - for calibrating               */
```

`driveSteer` takes a **fraction**, and each side is scaled separately from the
calibration. This car throws 254 µs one way from centre and 186 the other, so
anything that added a fixed amount to a midpoint would pull to one side every
time it was asked for half. Nothing above the calibration should need to know
what a microsecond is.

### Throttle

```c
Void driveArm(Bool on);                /* also snaps the target to neutral    */
Bool driveThrottleUs(Int32 us);        /* false when not armed                */
Void driveThrottleNeutral(Void);
```

Forward only. `driveThrottleUs` clamps to `[escMin, escMax]` and the board
refuses anything below 1500. Reverse needs a brake-then-reverse sequence and is
not something to reach by editing a number.

Neutral is always reachable: `driveArm` and `driveThrottleNeutral` set the
target directly, and a disarmed ESC is walked to 1500 whatever the limits say.
So raising the idle floor tightens the driving range without locking out stop.

### DriveState

What `driveRead()` gives you. Note the pairs — several things have a *target*
and an *actual*, and the difference is the response rate.

| Field | Meaning |
|---|---|
| `servoUs` / `servoTargetUs` | actual output / where it is heading |
| `escUs` / `escTargetUs` | the same, for the throttle |
| `steerMilli` / `steerNowMilli` | the target as −1000..1000 / where the wheels ACTUALLY are |
| `steerSlewUs` / `throttleSlewUs` | the two response rates |
| `centerUs` | straight ahead, measured |
| `servoMinUs` / `servoMaxUs`, `escMinUs` / `escMaxUs` | the limits |
| `escArmed`, `servoLive` | is it allowed to move, is the pin driven |

**Read the ACTUAL one for anything that reports what the car is doing.** The
target and the actual disagree for about a second after every command, and
something following the target lights a lamp or draws a wheel before the car
has done the thing. That mistake is why `steerNowMilli` exists.

---

## lights — the lamps, and which pin each is on

`lib/lights.h`. **The output layer, and nothing else.** Ten lamps, a table
saying which GPIO shows which, and a way to write a set of brightnesses out. It
does not know why any lamp is lit — that is `cue.h`, below.

```c
Void    lightsOpen(Void);
Void    lightsWrite(const LampSet* s);        /* show this set of lamps       */
Void    lightsEnable(Bool on);                /* master switch                */
Void    lightsForceLamp(Int32 lamp);          /* hold one lit, for testing    */
Int32   lightsForcedLamp(Void);
LampSet lightsRead(Void);                     /* what is lit this instant     */
Bool    lightsLit(Lamp l);
```

The table is temporary; see [wiring.md](wiring.md) for the current binding. A
lamp with no pin is still in the model, still computed and still reported, so
wiring one later is filling in a table entry rather than writing a rule.

The two overrides — the master switch and the forced lamp — live in
`lightsWrite` rather than in the caller. A master switch that only worked when
the layer above remembered to ask is a master switch that, one day, does not.
The forced lamp is at the output for a second reason: it is a claim about the
hardware, not about what the car means, so it has to survive whatever the cue
layer is doing, including a one-shot cue that would otherwise take the channel
back off you mid-test.

---

## cue — what the car SAYS

`lib/cue.h`. Everything the car emits at a person: which way it is about to
turn, that it is not being driven, that it has stopped itself, that it has seen
you. Lamps today, a buzzer later, and the point of the module is that the day
the buzzer arrives nothing above it changes.

```c
Void    cueOpen(Void);
Void    cueTick(const CueInput* in);          /* often, from the main loop    */
Void    cueSolve(const CueInput* in, CueTurn turn, Bool blink, LampSet* out);

Bool    cueEmit(CueKind k);                   /* say something, now           */
Void    cueSilence(Void);                     /* stop mid-sentence            */
Bool    cueBusy(Void);
CueKind cueSpeaking(Void);
CueKind cueFind(CharSeq name);
CharSeq cueName(CueKind k);
CharSeq cueMeans(CueKind k);
CueTurn cueSide(Void);

Bool    cueSetMotionUs(Int32 us);             /* what counts as "moving"      */
Int32   cueMotionUs(Void);
```

`cueSolve` is pure — same input and clock, same answer, no hardware — so the
rules can be reasoned about and eventually tested on the host the way
`lib/text.h` is. The blink phase is passed in for the same reason.

Feed `cueTick` the **actual** servo and ESC output, not the targets. The slew
limiter means the two differ for about a second after every command, and a cue
should follow the car rather than the request.

### Why this is not just "lights"

Indicators and brake lights used to be rules inside `lights.h`, next to the pin
table. That reads fine until the car needs a second way to express something —
a horn, a chirp, a headlight flash — and then either the lighting file grows a
sound API or the sound grows its own copy of "is the car turning". Two files,
both deciding what the car means, is one too many.

So the split is by **job**, not by hardware. `cue.h` decides; `lights.h` emits.

### Channels

A cue reaches a person through one or more channels, never through individual
lamps: `CUE_CH_HEAD`, `CUE_CH_TAIL`, `CUE_CH_IND_L`, `CUE_CH_IND_R`,
`CUE_CH_REV`. "Flash the headlights" is the intent; "set lamp 0 and lamp 1" is
an implementation of it that stops being true the moment a lamp moves.

`CueStep` also carries a `CueTone`. **Nothing drives it** — there is no buzzer
on the car and no pin set aside for one, and `cueSoundWrite()` records the tone
and returns. It is declared now because the alternative is discovering later
that `CueStep` has no room for sound and revising every script that exists by
then.

### Two kinds of cue

**Continuous** — what the car is doing right now, recomputed every tick from
the drivetrain. These are the rules that used to live in `lights.h` and they
are unchanged.

**One-shot** — an utterance with a beginning and an end: a short script of
steps, each naming which channels are lit and for how long. Adding a cue is
adding a row to `CUE_SCRIPT`, not writing another state machine.

A one-shot cue **owns** every channel any of its steps mentions, for its whole
duration, including the steps where that channel is dark. Without that a flash
would be invisible whenever the headlights were already on — the cue's on-steps
would agree with the continuous state and its off-steps would be overwritten by
it. Channels a cue does not mention are left alone, which is what lets the
indicators go on blinking through a flash.

`cueEmit` restarts a cue that is already running rather than queueing behind
it. A cue is a statement about the car *right now*; "I have stopped myself" is
not worth hearing four hundred milliseconds late, and a queue would mean the
car finishing a pleasantry before mentioning a fault.

| cue | means | what it does |
|---|---|---|
| `flash` | I have seen you — after you | headlights, 90 ms on / 110 ms off, twice |
| `alert` | I have stopped myself | both indicator pairs and the tails, 160/160, three times |

`flash` is **two** flashes and not one, deliberately: a single 90 ms blip is
what a loose connection looks like, and the whole value of a cue is that it
reads as something the car meant. `alert` is slower and heavier so the two are
not confusable at a glance — one means "after you" and the other means
"something is wrong".

`alert` is what the **deadman** emits when it stops the car. By definition that
fires when the host has stopped listening, so a line in a console nobody is
reading is the one place that message must not only be.

**What the continuous rules claim, and how honest each one is:**

- **Tails** are lit when the car is not being driven. Not braking — there is no
  brake and no way to measure slowing until the encoder is on, so the lamp
  reports the one throttle fact available. A car standing still with the ESC
  disarmed therefore has its tails lit.
- **Indicators** follow the steering past a threshold. Also a guess: a real car
  indicates because somebody pushed a stalk. Wrong on a long sweeping bend,
  right in the case that matters.
- **Reverse** is written and unreachable, because the drivetrain is
  forward-only. It is there so the lighting is already correct the day reverse
  arrives rather than being the thing somebody remembers afterwards.
- **Head** is manual. Nothing the car knows implies darkness.

The indicators do **not** interrupt the tails. On a car whose rear indicator
and brake share one bulb they have to; this car has a separate LED for each,
with a second indicator pair going on the rear, so they never compete for one
lamp. Interrupting anyway made the brake light blink in antiphase to the signal
beside it, which is the one thing a brake light must not do. If a shared-bulb
cluster is ever fitted, that belongs in the **binding** - two lamps mapped to
one pin - not in the rules.

### From the console

```
CUE                  where it stands
CUE LIST             every cue this firmware knows, and what each one means
CUE <name>           say it now
CUE STOP             stop mid-sentence
```

`CUE` reports the **utterance** — which one, how far through it is, and what it
would be sounding if there were a buzzer. The lamps are reported by `LIGHTS`,
because "the headlights blinked" and "the car said *after you*" are different
facts and only one of them survives being read off a lamp level.

`LIGHTS OFFAT <us>` still sets the motion threshold and `LIGHTS` still reports
it as `off_us=`. The wire protocol did not move when the code did — the hub's
Drive view drives both, and a reorganisation that changed what a board answers
would be a reorganisation that broke a screen.

---

## status — the onboard LED

`lib/status.h`. The one lamp a person can read across a room, and the only
channel left when the car has driven itself under a table.

```c
Bool statusOpen(Void);                 /* false if the lamp did not come up   */
Void statusSolid(Bool on);
Void statusBlink(Float32 hz);          /* full cycles per second; 0 = solid   */
Void statusTick(Void);                 /* often, or the blink stalls          */
Void statusHello(Int32 flashes, UInt32 msEach);
```

Not on an interrupt, on purpose: an interrupt handler that drives a peripheral
over a bus is a good way to find out what your bus does when it is re-entered.

---

## hal — the SDK, spelled our way

`lib/hal.h`. Wrap the SDK **once**, here. If application code is reaching for
`cyw43_arch_gpio_put`, `getchar_timeout_us` or `pico_get_unique_board_id`, the
HAL has a gap — fill it there rather than at the call site.

`gpio*`, `pwm*`, `servo*`, `serial*`, `adc*`, `spi*`, `i2c*`, `watchdog*`,
`led*`, `sleepMs`/`sleepUs`, `nowMs`/`nowUs`, `boardId`, `rebootToBootsel`.

Two things worth knowing:

- **The LED is board-specific and the header hides it.** On the Pico 2 W it
  hangs off the CYW43439 and `ledOpen()` can fail; on a plain Pico 2 it is GP25
  and cannot. `ledBackend()` says which. The switch is on the SDK's own board
  header, not on anything this project has to remember to set.
- **Derive sentinels, never restate them.** `SERIAL_NONE` is
  `PICO_ERROR_TIMEOUT`. It was written as `-1` because that is what a sentinel
  looks like, the SDK says `-2`, and the result was a command buffer filling
  with bytes nobody typed.

## text, gfx, drivers

- `lib/text.h` — `textEq`, `textStarts`, `textWord`, `textInt`, `textFloat`,
  `textTwoInts`. Strict: a parse either consumes the whole thing or fails.
  `textWord` matches a **whole word**, which is what makes the command table
  order-independent. Host-tested in `firmware/tests/test_text.c`.
- `lib/gfx.h` — drawing into a `Screen`. The one file at lib root that reaches
  sideways into a driver, written down rather than special-cased.
- `lib/drivers/` — `range.h` (VL53L1X), `display.h` (TFT), `storage.h`
  (MicroSD). A driver may name `hal.h` and nothing else of ours: a driver that
  needed another driver would be two things wearing one name.

---

## Persisting a setting

Everything above is runtime state and is gone at the next reset. `cal.h` is
what survives, and the hub's Drive view writes it: **Write to firmware**, under
Throttle range.

That generator rewrites `cal.h` **whole**, so anything it does not print is
deleted. If you add a macro to `cal.h`, add it to `steeringCalText()` in
`hub/src/app_ui.cxx` in the same commit — `SLEW_CAL_STEP` was not printed for a
while, and one click of that button would have produced a header the firmware
could not compile against.

---

## Adding to the library

1. **Where does it go?** A driver for a chip goes in `drivers/`. Something
   about the drivetrain goes in `chassis/`. Something every program wants goes
   at lib root. If it needs another driver, it is two things.
2. **What may it include?** lib root may name `types.h` and `hal.h`; a driver
   or chassis file may name `../hal.h`. Anything else is a build failure, not a
   guideline — see `LAYERS` in `hub/tools/style_audit.py`.
3. **Add it to the umbrella.** `bibo.hxx`, and the audit's `LAYER_EXTRA`.
4. **If it is a setting, give it all five**: setter, reading, serial command,
   hub control, `cal.h` macro.
5. **Never restate a constant somebody else owns.** Derive it.
