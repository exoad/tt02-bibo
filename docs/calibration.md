# Calibration

Numbers measured from the real hardware. These are needed later and will not be
remembered — record them here as they are taken, including the ones that come
out badly.

**Status: only §6 (scan rate) has been taken** — it reads off the viewer live, so
it came for free during bring-up. §1–§5 all need physical measurement and the
tables below are procedure plus empty result columns.

Device identity, read over USB 2026-08-24: model `0x41`, firmware 1.02, hardware
rev 18, health 0, serial `A11FE18AC1EA9ED2B29C92F522BB466C`.

---

## Before measuring anything: pre-heat

**Run the lidar with the motor spinning for at least 2 minutes before trusting
any measurement.**

The C1's scan core is a time-of-flight ranger — the datasheet (rev 1.1, p. 4)
says "laser flight-of-time (TOF) ranging principle … combined with the
high-speed laser acquisition and high-precision fusion algorithm", and the cover
calls it Fusion Technology. Any timing reference and any detector chain drift
with die temperature, and a cold reading sits outside whatever point the unit
was calibrated at.

**"dToF" and "picosecond-scale" used to be written here and are not from the
datasheet** — they were inferred from how other RPLIDAR models work. The pre-heat
advice below is a MEASURED result from this unit, and stands on its own; only
the explanation for it was borrowed.

This matters beyond bench accuracy: a drifting offset during the first minutes
of a mapping run corrupts early map geometry *relative to* later geometry, which
is exactly the kind of error SLAM cannot reconcile.

---

## 1. Distance bias

Tape measure to a flat wall, sensor perpendicular. Spec is ±30 mm; a *consistent*
offset is a constant that can be subtracted in firmware.

| Actual | Reported | Error |
|---|---|---|
| 1.00 m | | |
| 2.00 m | | |
| 5.00 m | | |

Conclusion (constant offset / scale error / neither):

---

## 2. Zero-degree orientation

There is a directional arrow molded into the housing. Point it at a known
object and confirm which angle the viewer reports.

This is the first term of the lidar-to-vehicle transform, so getting it wrong
rotates the entire map.

- Angle reported for the arrow direction:
- Sense (does angle increase clockwise or anticlockwise, viewed from above):
- Offset to apply in firmware:

The viewer draws 0° pointing up with angle increasing clockwise, and draws a
heading arrow from the sensor origin along 0° for exactly this check.

---

## 3. Real minimum range

Walk an object in from 30 cm to 3 cm and find where it stops reporting.
Determines whether the bumper ToF sensors are actually necessary.

- Datasheet minimum: 0.05 m
- Measured dropout point:
- Behavior below it (no return / garbage return / clamped):

The viewer discards returns below 50 mm as housing reflections and draws a
hatched blind disc at that radius. If the measured minimum differs, that
constant (`kMinValidMm`) should be updated to match reality.

---

## 4. Dark surface range

Point at matte black and find where it drops out.

- Datasheet: 12 m white / 6 m black
- Measured on matte black:
- Measured on the test wall (for reference):

---

## 5. Sunlight behavior

**This is the outdoor blocker. Characterise it now, not in month six.**

The C1 tolerates 40 klux. Direct sunlight exceeds that and degrades it. Sidewalk
operation is a stated goal, so this measurement decides how much of the plan
survives.

| Condition | Max usable range | Valid return % | Notes |
|---|---|---|---|
| Indoors | | | |
| Outdoor, overcast | | | |
| Outdoor, shade | | | |
| Outdoor, direct sun | | | |

Mitigations, in rough priority order:

1. Operate in shade or overcast
2. Fuse a sunlight-tolerant single-point sensor (TF-Luna, 70 klux — but 0.2 m
   minimum range, so it does not replace the bumper ToFs)
3. Lean on wheel odometry to carry through dropouts
4. Optical bandpass filter
5. Hardware upgrade — last resort

---

## 6. Scan rate

Count complete revolutions per second.

- Datasheet: 10 Hz typical, 5 kHz sample rate, 0.72° angular resolution
- Measured rotation rate: **9.8 Hz** steady state _(2026-08-24)_
- Measured points per revolution: **508–513**
- Derived angular resolution (360 / points per rev): **~0.71°** — matches spec
- Measured sample rate: **~5000 points/s**
- In-spec valid returns: **73–83%** indoors; max indoor range seen ~7.6–7.9 m

Sampled after warm-up. Skip the spin-up transient: the first ~15 revolutions run
fast and sparse (~355 points at ~14 Hz) before settling, so anything measured in
that window is wrong in both directions at once.

The viewer reports rotation rate, points per revolution and points/second live,
so this is a read-off rather than a stopwatch exercise.

---

## Servo and ESC calibration (phase 2, not started)

To be filled in once the Pico drives the servo. Control code thinks in
normalized units (−1.0 to 1.0); the microsecond mapping lives in exactly one
function so calibration is a single-constant change.

| Quantity | Value |
|---|---|
| Servo center (µs) | |
| Servo full left (µs) | |
| Servo full right (µs) | |
| Mechanical steering limit (does full travel bind?) | |
| ESC neutral (µs) | |
| ESC full forward (µs) | |
| ESC full reverse (µs) | |
| ESC arming sequence required | |
