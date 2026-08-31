# Build log

Newest first. **Entries recording what failed are worth more than entries
recording success** — the failures are what cost time and what gets forgotten.

---

## 2026-08-28 - a second board, the lighting model, and four self-inflicted bugs

A long day. The parts that cost time are the four things that were wrong,
so those come first.

### The generator that ate cal.h

`steeringCalText()` builds the calibration header in a fixed `Char[2048]`,
and the header grew past it. `snprintf` **truncates silently** and returns
the length it *would* have written, which nothing was reading — so one click
of "Write to firmware" put a `cal.h` on disk that stopped mid-comment, and
the firmware then would not compile. The failure surfaced three steps away
from the button that caused it, in a file nobody had edited.

The buffer is bigger now, but that is not the fix: the return value is
checked, and on truncation the generator returns nothing and the caller
refuses to write. **No file at all beats half of one.** A fixed buffer
holding generated source is always one edit away from this.

Related, and found at the same time: the generator rewrites `cal.h` WHOLE,
and it had never printed `SLEW_CAL_STEP` — which `chassis.h` uses. Anybody
clicking that button since the slew step was added would have got a header
that could not compile. **A macro added to `cal.h` has to be added to the
generator in the same commit.**

### The cursor after an item is at the WINDOW's left edge

Twice. Repositioning a column and then drawing into it does not work: ImGui
puts the cursor back at the window's left content edge after every item, not
at wherever the caller moved it.

First the console column — the central tab bar landed correctly at the new x
and the view body underneath it drew 400 px to the left, on top of the
console. Then the pinned stop — it drew at the sidebar's x and the whole
scrolling column below it started at the far left of the app.

The fix both times is a child window, which makes "the window's left edge"
mean that column's left edge. **Reach for the child wrapper by default when
moving a column.**

### A ScopedFont that outlived its child

Pushed inside `BeginChild`, popped at end of scope — after `EndChild`. ImGui
compares the font and style stack sizes across Begin/End, and the unbalanced
frame renders as a **blank white window**, not as an error anybody can read.

### The throttle bar measured from 1500

Three places computed how far into its range the throttle was, and two used
a hardcoded 1500 rather than the calibrated idle. Correct until idle was
measured at 1541; from then on the bar showed four of ten blocks lit with
the motor standing still, and disagreed with the percentage printed directly
above it. **A number computed in three places is wrong in two of them
eventually, and the disagreement is the only symptom.**

---

### The car's own board

A second RP2350 — a plain Pico 2, no wireless — is now a build target and is
the board that goes in the vehicle. The Pico 2 W stays the breadboard mule.

Only the LED genuinely differs: on the W it hangs off the CYW43439 and needs
the chip up first, on the plain Pico 2 it is GP25. `hal.h` carries both
behind one API, chosen by the SDK's own board header rather than by anything
this repo has to remember to set. Every pin this project uses is identical on
both, which is what makes the harness carry over — and that holds only while
GP23/24/25/29 stay unused.

`build.bat` takes a board and gives each its own tree. A wrong image does not
announce itself: the RP2350 accepts either and a W image on the car's board
runs perfectly with a dead lamp.

### Lighting, as a library

Ten lamps, the rules that decide what each does, and a **separate table**
saying which GPIO shows which. The rules are permanent; the table is
temporary and is the only thing that changes when a wire moves — proven when
four LEDs went on and the change was four numbers.

Two lamps are on borrowed pins. **GP15 is the wheel encoder** and gives it
back the moment the Hall sensor is fitted; that is the one borrowing with a
deadline.

One rule was built and removed. On many cars the rear indicator and the brake
share a bulb, so the indicator must interrupt the brake to be seen — and that
interruption is what makes such a car read as a car. This car has separate
LEDs, so applied here it made the brake light blink in antiphase to the
signal beside it. **The test was inverted rather than deleted**, because a
rule that is deliberately gone needs a test saying so.

### Settings, properly

The steering and throttle shared one response rate. Tuning the steering quick
made the throttle violent; gentling the throttle made the steering vague. They
are separate settings now — the steering has 440 µs of travel and the throttle
59, so one number could never have suited both.

[firmware-api.md](firmware-api.md) is new and records the shape a finished
setting has: a setter that validates, a reading, a serial command, a hub
control, and a `cal.h` macro. **Missing any of the five is a gap, not a
style.**

---

## 2026-08-26 - the recorder

A fourth central tab, beside 2D / 3D / Pico 2 W: **Record**. Capture
revolutions, save, load, scrub, play.

### Why a recorder and not a mapper

What was asked for was "mapping or localization". A real map needs a POSE per
scan - translation as well as rotation - and there is none: GP15 is marked
"wheel encoder, not wired", and the scan-profile matcher written for World lock
recovers rotation only. A map built from unlocated scans is a confident-looking
picture that is wrong the moment the sensor moves, which is worse than no map.

The recorder is the part that is honest today, and it is also the part that has
to come first: **you cannot develop a mapper against a live sensor.** You develop
it against a recording you can replay a hundred times and get the same answer
every time. When the encoders are wired, the mapper reads these files.

The view is deliberately Points and nothing else. Every other mode is an
interpretation; what a recording must preserve, and show you it has preserved,
is the returns.

### A second RadarView, and a bug it exposed

The recorder has its own view so scrubbing does not disturb the live map. That
immediately broke `mapStateFor()`, which held ONE static accumulator and wiped it
whenever the owner changed - correct while there was one map on screen, and with
two it threw away the density and occupancy history on every tab switch, which is
exactly the history those modes exist to build. It is a fixed slot table now.

### The format is plain text, and smaller than the binary it replaced

It started as binary: two float32 per point. Then the requirement arrived that
these live on an SD card and should be readable. Measured against 48 real
revolutions before choosing:

| encoding | bytes/point | |
|---|---|---|
| binary, 2 x float32 | 8.00 | 100% |
| **text, absolute mm** | **6.65** | **83%** |
| text, delta-coded mm | ~5.9 | 74% |

**Text won outright.** The device reports whole millimeters - the fractional part
of every distance in a real capture is exactly 0.0000 - so `6789` carries
precisely what a float32 did, in four bytes, with none of the ambiguity. Angles
are centidegree deltas, which are two digits because the C1 steps ~0.72 deg.

Delta-coding the ranges would save another 8 points and cost the one property the
format exists for: you can open it and see that 6789 is a wall 6.8 m away. On a
card pulled out of a car at the side of a track, that is worth more than 8%.

The angles are kept per point rather than assumed uniform. The real spacing
wanders between 0.49 and 1.05 degrees - that is measured data, and discarding it
to save bytes is the same mistake as inventing data, run backwards.

The v1 binary reader is kept, read-only, so changing the format cost nobody a
recording.

### Verified rather than assumed

Recorded against the real C1, then parsed the file with an INDEPENDENT Python
reader - not the C++ one that wrote it:

- 49 revolutions, **all 50,026 tokens consumed, none left over**
- angles reconstruct from the deltas to 0.6..359.2 deg with **zero wraps inside a
  revolution**
- 508.5 points/revolution at 10.04 rev/s, matching the device's reported 9.8 Hz
- ranges 136..7415 mm, inside the 50..12000 spec window

And the UI cycle end to end: record, save, clear (status "nothing recorded", the
buttons correctly disabled), load ("loaded scan-...", 48 rev 4.7 s), play
(advancing through revolution 24 of 48).

**Caught by that test**: the first pass clicked coordinates I had guessed rather
than measured, so Clear and Load were never actually exercised - the status still
read "saved". Re-ran with positions read off a rendered frame.

---

## 2026-08-26 - World lock becomes a frame of reference

I had built the wrong thing. "Lock to car" was implemented as a **camera
preference** - pin the orbit target, disable panning. What was actually wanted is
a **frame of reference**:

- **Car lock** - the sensor's frame. The car is fixed and the room turns around
  it. This is what the view has always done.
- **World lock** - the room's frame. The room holds still, and turning the sensor
  visibly rotates the car.

The second one needs the sensor's heading, and nothing on this machine reports
it: no odometry, no IMU, no compass. I had written in a comment that the heading
half was "not faked in the meantime", which was the right call then and is not an
excuse now - it is recoverable from the scan alone.

### Heading by profile matching

`mapgeo::estimateHeading` cross-correlates the current 120-bin range profile
against a **reference** captured when the world frame was zeroed. Turn the
sensor and that profile slides along the bearing axis without changing shape;
the shift that best aligns them is the heading.

Against a fixed reference rather than frame-to-frame **on purpose**: integrating
per-frame deltas accumulates drift forever, matching an absolute reference does
not drift at all. The trade is that it fails when the room stops resembling the
reference - a failure you can see and reason about, where drift is one you
cannot.

Sub-bin refinement by parabola through the best cost and its neighbors, because
without it the heading quantises to 3 degrees and the world visibly snaps as you
turn.

**What it cannot do, stated in the header and reported on screen:**

- **Rotation only.** Translation is not estimated. Slide the sensor sideways and
  the profile changes shape rather than shifting.
- **A rotationally symmetric room has no answer.** A circular room costs the same
  at every shift.

So it returns a **confidence**, and the readout prints it: `world lock, 12 deg
(97%)`, or `heading unsure (18%)` below about a third. Separability against the
MEDIAN cost, not the worst - the worst shift is an outlier that flatters every
match, where the median is what a typical wrong answer costs.

**28 checks, 0 failed.** A 17-bin shift comes back as 51.0 deg, a half-bin
rotation resolves to 31.5 rather than snapping to 30, and **a circular room
scores 0.000**.

### One rotation, in the matrix

Everything drawn is in the sensor's frame, so putting it in the world's is a
single rotation about the vertical applied to the whole scene - as a model matrix
premultiplied into the MVP. The car, being fixed in the sensor frame, is then
what visibly turns. Zero in Car lock.

The sign is **derived, not guessed**: a return at bearing b lands at
(sin b, cos b), so increasing bearing sweeps forward toward right - clockwise from
above, a negative rotation about +z. A fixed feature whose measured bearing rises
by d has moved by -d, so undoing it is +d. If it turns out to spin the wrong way
on real hardware it is a one-character fix; I cannot rotate the sensor from here.

### And the buttons were broken too

The Car/World pair genuinely did not respond, and it was the **same ID conflict**
from earlier: two widgets sharing an identity confuses ImGui's active-ID
tracking, so the click landed on the wrong one. The two reports were one bug.

---

## 2026-08-26 - two buttons called "Car"

An ImGui ID conflict, reported from the app's own error checking.

**ImGui derives a widget's identity from its LABEL.** The 3D control bar had a
`Lock [Car][World]` pair and a `Show [Car][Sensor]` pair sitting side by side in
the same child window, both with a button called "Car" - so as far as ImGui was
concerned they were one widget. It flags that, and clicking one can drive the
other.

Fixed with named `PushID` scopes around each group - named rather than indexed,
so the two scopes cannot collide with each other the way the labels did.

**Caught a worse bug on the way in.** A patch script asserted partway through
and wrote nothing, but I had also made the matching edit by hand - which left
`drawEgoSwitch` holding a `PopID` with no `PushID`. An unbalanced ID stack is
strictly worse than a duplicate label, and the only reason it did not ship is
that the balance was checked rather than assumed.

Then swept the rest: two other labels repeat - "Back up board flash" and
"Auto-scroll" - and both are already safe, because every sidebar section body is
wrapped in `PushID(e.id)` and the pairs live in different sections. Only the
control bar had no such scope, which is exactly why it was the one that broke.

Also fixed a tooltip still claiming the car is 430 x 190 mm. It is 442 x 186.

---

## 2026-08-26 - the real TT-02 numbers, the sensor on the car, and lights

### Four of six dimensions were wrong

They were remembered, not looked up. The kit is **Tamiya 58631, "1/10 R/C 4WD
Subaru Impreza Monte-Carlo '99 (TT-02 Chassis)"** - which is this project's
actual car - and its published specification says:

| | had | Tamiya 58631 |
|---|---|---|
| Length | 430 | **442 mm** |
| Width | 190 | **186 mm** |
| Height | 135 | **140 mm** |
| Wheelbase | 257 | 257 |
| Tire | 64 x 26 | **69 x 27 mm** |

Tread is not published and the TT-02 offers two settings, so it is **derived**:
the stated 186 mm is measured across the tires, so center-to-center is
186 - 27 = **159 mm**, self-consistent with both published figures. If the car is
ever set to the wider tread this becomes wrong *visibly* - the wheels will sit
outside the shell.

They were also **duplicated** - one copy in radar.cpp, one in scene3d.cpp - which
is how they came to disagree with the kit and with each other. There is now one
`src/vehicle.hpp`, with the source URL beside the numbers, because a number
nobody can check is a number nobody can correct.

### The sensor, on the car

The scene is measured in the sensor's frame, so where the sensor sits on the car
is the relationship the entire picture is built on - and it was not drawn at all.
Now it is, at its mount, to scale: 55.6 mm against a 442 mm car.

Two consequences worth the change:

- **Return columns stop at the scan plane.** They were 90 mm tall because 90 mm
  looked right. They now rise to the height of the horizontal slice the beam
  actually sweeps, so the top of every column is a real surface and the sensor
  on the roof sits exactly level with them. One fewer invented number.
- The mount position, mount height and optical height are **all assumptions**,
  written down as such in vehicle.hpp with a warning, so measuring the real
  mount is a three-line edit and nothing downstream mistakes them for facts.

Plus a `Show [Car] [Sensor]` switch in both dimensions. Worth a control rather
than a constant because **SENSOR is currently the honest picture**: the C1 is on
a desk and there is no car, so a 442 mm shell drawn round a 56 mm puck is a
statement about the future.

### Lighting

`conventions.md` has carried a lighting specification since long before there was
anything to run it on. It is implemented now, in `src/lights.cpp`, as a pure
function: inputs and a clock in, ten lamp brightnesses out. No statics, no
device - the firmware will need exactly this logic, and porting it should be a
copy rather than a re-derivation from a photograph.

**tests/test_lights.cpp - 25 checks, 0 failed.** Three of the rules look right
and are not, which is precisely why they are tested rather than eyeballed:

- **The indicator overrides the brake on its own side.** Braking while
  signaling right means the right rear alternates and the left stays solid. Get
  it wrong and both stay solid, which looks fine and is wrong.
- **Hazards are both sides IN PHASE.** Alternating is what a film prop does.
- **Front and rear on one side share ONE clock.** Per-lamp timers drift, and
  drift is invisible for the first minute.

Also verified on screen, two shots half a flash apart: left rear solid red,
right rear dark with its amber lit.

The lamps are modelled on the car - headlights and front indicators on the nose,
and a rear cluster following the note in conventions.md: red main, amber indicator,
and the reverse lamp **nested inside the indicator housing** rather than sitting
beside it as a fourth unit. Each lamp is drawn twice, a dark lens always and a
lit face on top, because an unlit lamp is not invisible - it is dark glass, and
drawing nothing makes the car change shape every time it blinks.

A bench panel in the Vehicle section drives it. **Nothing is wired**: no LED
exists and the board has no lighting firmware. The panel says so.

---

## 2026-08-26 - the ride view

3D Full is now modelled on the display an autonomous car shows its
**passengers**, which is a different object from the four modes above it. Those
are instruments - a grid to measure against, a number per mode, a color per
meaning. This is a picture of the situation, and the rules that follow are:

- **No wireframe.** A grid of lines is how you read a value off a plot. It is
  not how you show somebody where the car is. The ground is a filled disc that
  fades out, and its edge is where the sensor stops seeing - the only boundary
  the view draws.
- **Almost no text.** One line: objects, and meters ahead.
- **Few colors.** Near-white for things that are there, one accent for the path,
  a dark ground. Waymo's screen is essentially two colors and reads from the
  back seat.
- **Soft and matte.** Detections are chamfered boxes - eight sides in plan
  instead of four, four extra triangles, and the whole difference between "a
  rounded object" and "a debug AABB".

### The one thing deliberately not copied

Waymo draws little pedestrians, cyclists and cars. It has a classifier that earns
those shapes. **A planar lidar with no classifier cannot tell a chair leg from an
ankle**, so every detection here is the same plain box, and the only distinction
drawn is in-path or not - two states, which is what this sensor can honestly
support. Drawing a person because something was person-sized would be the
display inventing one, which is the same line the column heights have held since
the 3D view existed.

Detections and the corridor come from the flat map's own fitter rather than a
second one here, so the two dimensions cannot disagree about what is out there.

### A material, rather than a lambert term

The flat `0.34 + 0.66*|n.l|` became key + hemispheric + fresnel rim. The sky
term is what stops a flat-shaded model reading as folded paper; the rim
separates the silhouette from a dark background without needing an outline.
Plus a contact shadow under the car - nothing else sells "this is sitting on
that floor", and without one the car appears to hover, which on a display whose
whole subject is where things are on the ground is the wrong impression.

### Caught in the screenshot

The ground gradient came out as a ring of flat wedges. `emitTri` paints a whole
triangle one color, and a gradient made of flat triangles is a gradient made of
bands. Added `emitTriC` with a color per vertex - the vertex format always had
the field, nothing was using it.

---

## 2026-08-26 - a depth buffer, and the filter that ate a hand

### The painter's algorithm had to go

Two reports, one cause:

- The car self-clipped at certain angles - panels punching through the
  windscreen and the boot.
- The car drew over point-cloud pins that were plainly in front of it.

The renderer decided visibility **per primitive**: project every face, sort by
centroid depth, draw back to front. That is exact for a flat ground and disjoint
columns. It is not merely inaccurate for a car, it is *impossible*: a body whose
greenhouse is inset into the shoulders and whose spoiler passes through the tail
has interpenetrating triangles, and interpenetrating triangles have **no correct
draw order at all**. No sort could ever have fixed it.

The second one was worse and dumber: the pins were drawn with immediate
`ImDrawList` calls that never entered the sort queue, so they were always behind
everything that did.

So the scene now renders through **D3D11 with a real depth buffer**
(`src/scene_gpu.{hpp,cpp}`), off-screen, composited back as one ImGui image.
Visibility is per pixel. The sort is gone.

Notes worth keeping:

- **Off-screen, not to the back buffer**, so the result layers with ImGui
  normally - the HUD, labels and floating panels draw over it knowing nothing
  about D3D.
- **Opaque triangles are grouped by texture; blended ones are not.** Opaque can
  be reordered freely because the depth buffer decides the result. Translucent
  cannot: a depth buffer picks one winner per pixel, and a see-through surface
  has no business being a winner - so those still sort back to front, and only
  those.
- **Cull NONE.** Trusting a downloaded mesh's winding was a bug waiting to
  happen for no gain at 2,088 triangles; the depth buffer resolves it.
- Every line - the ground grid, the pins - is a camera-facing quad scaled to
  hold a constant pixel width. A real line primitive is 1 px regardless of DPI
  and would not be depth-tested against the solid geometry.

### The chassis filter was wrong and is gone

Yesterday I suppressed returns inside the car's footprint, to stop pins standing
up through the model. It fixed a picture and broke a measurement: a hand cupped
around the sensor is well inside a 430 x 190 mm footprint, so the filter deleted
the easiest way there is to check the device is alive. It was reported within a
day, which is about how long a display that hides real returns deserves.

The depth buffer makes it unnecessary. Instead there is a **Car** toggle: the car
is a reference object, not data, and when it hides something you want to see the
answer is to hide the car - not the returns.

### Grids belong to the mode

Every mode got range rings because the first one did and the rest inherited it.
But a grid is a measuring instrument and which one is right depends on what is
being measured:

| | |
|---|---|
| **Radial** | Points, Clearance, Gaps, Full - where the quantity IS a range and a bearing |
| **Cartesian** | Walls, Corners - lengths and right angles. And Density and Motion, whose cells *are* a fixed 60 mm world grid: meter squares show the coordinate system the data actually lives in, where rings would be a second, unrelated one laid over it |
| **None** | Minimal |

A wall you are judging the straightness of, drawn against concentric circles, is
a straight edge being argued with by curves. The same rule now picks the 3D
ground grid.

### Minimal

A tenth flat mode, and the first one that is not an instrument. No grid, no
numbers, no legend, no nearest-return marker, no scale bar, no HUD line - the
room as one soft shape, the returns as light on its edge, the car in the middle.

It overrides the Grid/Labels/Nearest checkboxes rather than reading them. The
mode IS the statement "no chrome", and leaving a red marker on a display meant
for an audience because a debug toggle happened to be ticked would defeat it.
This is not Full with the labels off - it answers a different question. "What can
it see?" rather than "is bin 47 stale?".

### Full, in 3D

Drivable floor, walls as standing panels, returns as columns, the car, and the
figures. The numbers are handed in from the flat map's accumulators rather than
recomputed, so the two dimensions cannot disagree about them.

---

## 2026-08-25 - style audit, texturing, camera lock

### The style audit found something worse than style

`python hub/tools/style_audit.py` now enforces the table in conventions.md and exits
non-zero on a violation. Writing it turned up **151** of them - but the count is
not the finding. Almost all were in `hub/tests/`, which the rename pass of
earlier that day had skipped entirely, and the two hardware tests had **not
compiled since**: they still included `../src/lidar_source.h`, a file that no
longer existed. Weeks of "we have hardware tests" that could not be run.

Both are repaired and both build. `test_lidar_source.exe` passes **16/16 against
the real C1** - 9.84 Hz, 508 points a revolution, 71% valid, stop() in 1027 ms.

The rest of the pass: 15 C-style casts named, `g_` globals renamed,
`GetProcAddress` results through `reinterpret_cast`, `k`-prefixed constants and
snake_case locals in the tests brought into line. **0 violations now.**

Three standing exemptions are encoded in the tool with their reasons, so nobody
has to rediscover them: `resource.h` is compiled by `rc.exe`, `shared.hpp` is
where the aliases are defined, and Win32 ABI signatures use the OS's types.

### /W4 was aspirational

conventions.md said the tree was warning-clean at `/W4`. `build.bat` was passing
`/W3`. Raising it surfaced 12 warnings - a vestigial `dpi` parameter, and
**eleven dead `Impl* impl = pimpl;` locals in pico_link.cpp**, scar tissue from
the same rename pass that broke the tests. All twelve gone; the claim is true
now, and the discrepancy is written into conventions.md rather than quietly fixed.

### The 3D view

- **Returns inside the chassis are suppressed.** They were drawn as columns
  standing up *through* the car - a picture of the car impaled by its own housing
  reflections. Same rule the corridor already used. The count is reported rather
  than hidden: "326 returns (31 inside the chassis)".
- **Camera lock, car or world.** Locked to the car, the target is re-pinned every
  frame and panning is inert - that is what locked means, and a pan that silently
  un-locked it would be worse than one that does nothing. Today the two differ
  only in anchoring; once there is a pose estimate, CAR will inherit the car's
  heading and WORLD will hold a fixed bearing. The heading half is **not faked**
  in the meantime.
- **Textures.** `ImDrawList` can carry UVs, so faces now go through
  `PrimWriteVtx` with the model's own texture coordinates, batched by texture in
  depth order - grouping by texture first would have drawn the car in front of
  the columns occluding it. Affine interpolation, so a huge near triangle would
  warp; at this poly count it does not. The vertex color becomes a light level
  rather than a paint, so the flat-shading still carries the form.
  `ui::loadTexture()` reuses the WIC decode that already existed for the icons
  rather than adding a second one.

### The livery, and what it is not

`tools/livery.py` repaints the atlas into Subaru World Rally blue. It has to be a
script rather than an image edit: the atlas is a *palette*, so there is no bonnet
to paint - only the swatch the bonnet samples. It asks the mesh, samples the
`body` group's UVs, and picks the most common **chromatic** swatch, because the
grays that dominate that histogram are the underbody nobody ever sees. Remapped
by nearness and luminance-preserving, so the palette's baked shading survives.

**It is still a generic saloon in the right color, not an Impreza.** The closest
properly-licensed low-poly Impreza WRC is Aeroux's 368-triangle CC-BY model on
Sketchfab, which returns 401 on its download endpoint without an account. The
loader needs nothing but the file dropped into `assets/models/`.

---

## 2026-08-25 - a real car model, map tabs, and a frame ceiling

### The car is a downloaded model now

`assets/models/car.obj` - **"sedan-sports" from Kenney's Car Kit, CC0**. The
hand-lofted shell before it got a touring car's proportions roughly right and
never once looked like a car; there is no reason to keep approximating a thing
that exists under a public-domain license.

The loader is ~120 lines and deliberately not a general OBJ parser: it reads
`v`, `g` and triangular `f`, which is all this file contains, and skips anything
else rather than guessing. Color comes from the **group names** (`body`,
`spoiler`, `wheel-*`), not from the material - the kit textures everything from
one atlas, and a texture is the wrong answer for a schematic drawn in a terminal
palette anyway. Flat-shaded from face normals against a fixed world-space light,
back-face culled (2088 triangles down to ~1000), no per-triangle outline because
at that count an outline is a wireframe.

Two things worth writing down:

- **It is scaled PER AXIS to 430 x 190 x 135, not uniformly.** That is a
  deliberate ~15% distortion of the width. The corridor, the Fit erosion and the
  flat map's footprint all derive from those measured numbers, and a car drawn
  wider than the corridor beside it is a picture contradicting the measurement it
  sits next to.
- **Swapping two axes is a reflection.** Model space is x-right, y-up, z-length;
  scene space is x-right, y-forward, z-up. That swap flips triangle winding,
  which the back-face test depends on - so x is negated as well to put the
  determinant back to +1. Mirroring a symmetric car left-to-right is invisible.

The hand-built shell stays as a fallback. A missing asset must cost fidelity,
never the car: with nothing at the origin the scene loses its frame of reference.

Attribution is in `assets/ATTRIBUTION.md` even though CC0 does not require it -
knowing where a file came from is worth more than the license obliges.

### 2D and 3D are tabs

They were a segmented switch on the overlay strip, which put the control that
decides *which overlays exist* inside the strip of overlays it decides. They are
tabs now - `2D | 3D | Pico 2 W` - which is the level the choice actually sits at,
and the same relationship the board view already had to the map.

`centralView` gained a slot: 0 = flat map, 1 = scene, 2+ = a board. The mode
strip keys off it rather than off `radarView.is3D`, because the control bar's
HEIGHT is computed before the tab bar runs and its CONTENTS after, and
`centralView` is the value that is stable across both.

### 60 fps ceiling

`Present(1, 0)` syncs to the **monitor**, and this one is 240 Hz - so vsync alone
was drawing the UI at 240 fps. Nothing on screen changes faster than the lidar
produces it, which is 10 Hz: the other 230 frames a second were redrawing the
same revolution at four times the power draw, on a laptop that is going to be
sitting next to a car in a car park.

A `CreateWaitableTimerEx` with `HIGH_RESOLUTION` rather than `Sleep()`: the
default system timer resolution is 15.6 ms, so `Sleep(1)` between frames would
have hit 60 Hz by luck rather than by design, and would have silently changed
behavior on a machine where another process had raised the global timer
resolution. Falls back to `Sleep` where the flag is unsupported. Behind schedule
it resyncs to now rather than running a burst of uncapped frames to make up time
that is already spent.

### Also

The end-cap fans on the hand-built shell were outlining every triangle, which
drew a star across the nose and tail - the spokes are an artifact of how a cap is
triangulated, not lines that exist on the object. Caps are filled with no edge
now and the rim gets its outline from the section loop.

---

## 2026-08-25 - Motion fixed, the real TT-02, and a 3D view

### Motion was firing on stationary things

`neighborhoodCold()` used a fixed 3x3 - plus or minus one 60 mm cell. But the
gap between adjacent samples along a surface is `r x 0.72 deg`: **25 mm at 2 m
and 151 mm at 12 m**. Past about 6 m, consecutive returns on the SAME stationary
wall land further apart than the test can reach, each one finds nothing warm
nearby, and every one of them is reported as new. Forever. The symptom is
range-dependent, which is exactly what was described - near things behaved, far
things strobed.

The radius is now derived from the sampling geometry instead of picked:

    radius = 1.4 * (sample arc at this range) + 45 mm

with the 45 mm covering the +/-30 mm range spec and the phase drift between
revolutions - the motor is not locked to the sample clock, so a wall's returns
walk along it. That is 1 cell close in and 5 at the ceiling.

This deliberately makes the mode **less** sensitive with range, and it should be:
at 12 m the device cannot place a return to better than ~150 mm, so declaring
motion at 60 mm resolution out there was reporting precision that does not exist.
Also added an 8-revolution warm-up, because on the first sweep nothing has been
seen before, every return is legitimately new, and the whole map flashing looks
like a fault. A still room now reads blank, which is the correct output.

### The car is a real TT-02 now

| | was | now |
|---|---|---|
| Length | 400 mm | **430 mm** (Tamiya's stated overall) |
| Width | 190 mm | 190 mm |
| Wheelbase | 257 mm | 257 mm |
| Tread | implied ~164 mm | **162 mm**, explicit |
| Wheels | 64 x 26 mm | 64 x 26 mm |

The width being 190 mm two ways is not a coincidence: tread 162 + tire 26 = 188
across the rubber, and the body is 190. The widest part is the same either way.
Two caveats recorded in the source: **length depends on the body shell**, and the
sensor is still drawn at the chassis center because the C1 is not mounted yet.

The plan view is a touring silhouette now rather than a six-point wedge - widest
over the rear arches, waisted at the doors, tapering to a nose about a third of
its width, with a greenhouse outline so front and back are distinguishable.

### The 3D view

A second dimension, not a second projection: `2D` and `3D` are a switch at the
left of the overlay strip, and each carries **its own overlays**, because the
dimension decides which questions are askable.

- **Cloud** - returns as pins on the ground plane
- **Blocks** - returns as solid columns; at a low camera angle a run of them
  occludes what is behind it, which is what the sensor sees too
- **Walls** - fitted surfaces extruded into standing panels. This is where 3D
  earns its place: a wall you can orbit behind is a surface, where the same wall
  on the flat map is a line segment.
- **Fit** - the drivable floor, seen from where the car sits

Software projection into the same `ImDrawList`, not a second D3D11 pass. The
scene is a few thousand quads; the alternative is shaders, a depth buffer,
render-to-texture and a second resize path for something that would still be
sorted by hand. Depth is a painter's sort on face centroids - exact for a flat
ground, correct for disjoint columns, and the car's parts are individually
convex.

**Every column is the same height, and that is the important design decision.**
The C1 is a planar scanner: it measures one horizontal slice and knows nothing
whatsoever about height. Varying the height by range or by quality would be
inventing a third dimension out of a two-dimensional instrument, which is the one
thing a 3D view of a 2D sensor must not do. The car is the only filled object in
the scene; every return stays an outline, so you never have to wonder which of
the two the sensor actually saw.

Three things the screenshots caught: the HUD printed the flat map's mode name
over the scene; the control bar still offered Range/Trail/Labels, which mean
nothing when you are orbiting (it offers Reset camera and Top down instead); and
the 3D path skipped the nearest-return calculation, which blanked a telemetry
readout that is a property of the data and not of the projection.

---

## 2026-08-25 - fourteen overlays down to nine

The complaint was that most of them did not mean anything, and that Walls was
the kind of thing they should all be. That is a usable bar, so I applied it:

> **Does it decide something the dots do not already say?**

Seven failed it and were deleted, code and all.

| Cut | Why |
|---|---|
| Distance | Colored dots by range - but range **is** the radius. It encoded one variable twice. |
| Sectors | Clearance at 30 deg instead of 3. Same computation, strictly coarser. Never rendered legibly either; I chased that bug twice. |
| Occupancy | Density with a longer fade. Two modes for one idea. |
| Rays | Dots plus a line to the origin. Its stated value was Validity's job, done better. |
| Contour | Joins adjacent dots - the *input* to Walls, not a rival to it. Kept as a helper, since Full composites it. |
| Sweep | Skew diagnostic. The car does not move yet. |
| Validity | Real, but sensor health rather than scene, and the telemetry panel already reports in-spec %. |

`drawMarksRays/Distance/Occupancy/Sectors/Validity/Sweep` went, and with them
`collectRays`, `collectDotsRamp`, `emitDiscsC`, `emitSegs`, `segScratch`,
`clipSeg`, `CDot` and `Seg` - about 300 lines that existed only to feed them. The
strip now fits two rows with no horizontal scrollbar, which it had needed since
the type scale went up.

### Two new ones, to the same bar

**Corners.** Where two fitted walls meet at more than 35 deg. Worth its own mode
because of what a corner is FOR: a wall constrains two of the three numbers a
robot needs - it fixes your distance from it and your heading against it, and
says nothing about where you are ALONG it, because sliding a wall along itself
leaves it looking identical. A corner does not slide. It is a point landmark,
which is what a scan-matcher keys on.

**Fit.** Free space eroded by the chassis width - configuration space, done in
polar form. Clearance is a question about the sensor; this is a question about
the car, and they disagree constantly: a 150 mm slot between a chair leg and a
wall is free space and is not a route. Live, on a cluttered desk, it reported the
car fitting in **5.2 m2 of 15.3 m2 free (34%), 104 of 120 bearings blocked** -
which is the honest answer and the reason the mode exists.

Both are refactors as much as additions: the split-and-merge was pulled out of
`drawMarksWalls` into `fitWalls()` so Walls and Corners cannot disagree about
what a wall is.

### The geometry is tested now, and that was not optional

Corners ran on real hardware and found **nothing** - "no corners: 3 walls, none
meeting at more than 35 deg". Correct for a desk with three short surfaces on it,
and completely useless as verification: *a mode that finds nothing and a mode
that cannot find anything look identical.* The scene decides, and the scene was
not going to cooperate.

So the pure geometry moved to `src/map_geometry.{hpp,cpp}` - no ImGui, no
globals - and `tests/test_map_geometry.cpp` feeds it inputs a desk cannot
produce. **21 checks, 0 failed.** The half that matters is the rejections, since
a corner detector that fires on two unrelated walls is worse than one that never
fires:

- Two walls at 90 deg whose infinite lines cross far from either one -> rejected.
- **The same two walls, moved so their ends meet -> one corner.** That pair is
  the point: it proves the rejection is the endpoint rule and not a dead code
  path.
- A 150 mm slot blocks a 190 mm car (reach 922 mm, stopping just short of it);
  an 80 mm car passes the same slot (2960 mm).

`radar.cpp` now *calls* that code rather than keeping its own copy - a second
definition of "corner" is a second place for it to drift, which is the same
reason `fitWalls` was extracted in the first place.

---

## 2026-08-25 — scale, panels, and the field display

### Everything was too small

One knob, `ui::userScale()`, multiplied into the monitor's DPI before anything
derives a size from it. Fonts **and** geometry together - scaling only the type
would blow the layout apart, and the density the UI was tuned for is a *ratio*,
so it survives being multiplied. Default 120 %, `Ctrl +/-/0`, and an `A- 120% A+`
control in the status strip.

The control is visible rather than shortcut-only on purpose. "The UI is too
small" is a complaint about the app, and an app whose answer is a key combination
nobody is told about has not answered it.

Persisted to `%LOCALAPPDATA%	t02-auto\` via a new `settings::` - **not** next to
the exe, because `build.bat clean` deletes that directory and a preference that
resets on every rebuild is not a preference.

### Draggable panels

No docking: the vendored ImGui is master, not the docking branch, and
`IMGUI_HAS_DOCK` is not defined. Three things instead, all persisted:

- **A splitter** between map and column. Drags in *logical* pixels, so a drag
  feels the same at 100 % and 200 %; double-click restores the default, which is
  the only way back from a column dragged narrower than its own grab edge.
- **Reorder** by dragging a section header. A rotate, not a swap - dragging
  Console to the top should leave the others in their relative order.
- **Tear off** into a free-floating window, and dock it back. The row stays
  behind as a placeholder so the column never silently loses an entry, and the
  window's X docks rather than hides: an X that makes a feature unreachable is a
  trap.

### The white window, and four wrong theories

The app built clean and rendered a **blank white client area**. In order, and all
wrong: PrintWindow failing (disproved by clearing the bitmap to magenta first -
an unpainted bitmap saves as *transparent*, which every viewer shows as white and
which is indistinguishable from a real white window); DXGI occlusion (disproved
by raising the window topmost); the settings layer failing; the mode toggle
looping at the larger scale.

What settled it was **evidence, not reasoning**: 0.125 s of CPU over 5 s said
blocked, not spinning. Traces bracketing init, the loop, and then `app::frame()`
narrowed it to `EndChild()`. Enumerating the process's own windows found a
`#32770` behind the app:

> Assertion failed! ... Code uses SetCursorPos()/SetCursorScreenPos() to extend
> window/parent boundaries. Please submit an item e.g. Dummy() afterwards.

`build.bat` does not define `NDEBUG`, so ImGui's asserts are live - which is why
this was a catchable dialog and not silent corruption.

**The bug:** my tear-off button positioned itself with `SetCursorScreenPos` and
restored the cursor afterwards. Every section but the LAST has a following header
to grow the bounds back; the last one does not, so `EndChild` asserted. Fixed by
using `SameLine`, which keeps the button inside ImGui's own layout and cannot
desynchronise from it.

**Two process lessons.** A modal dialog owned by the app under test is invisible
to a `PrintWindow` capture and looks exactly like a rendering failure - worth
checking first, not fifth. And I reached for a full-screen `CopyFromScreen`
capture as a diagnostic, which grabbed the user's browser rather than the app;
that approach is off the table, and the file was discarded unread.

### Full mode becomes a field display

Three things only this mode computes, on top of the composite it already was:

- **The car, to scale.** TT-02 footprint - 400 x 190 mm, 257 mm wheelbase - with
  wheels and a nose, because which way it is facing is the reason to draw it. It
  vanishes at 12 m across, correctly: a vehicle outline that stays legible while
  the world zooms out is lying about how big the car is. The sensor sits at the
  chassis center because that is the only position currently *true* - the C1 is
  not mounted yet - and the drawing is written in terms of an offset so mounting
  it is changing one constant.
- **Objects.** Returns clustered by a gap threshold that grows with range (at
  0.72 deg, neighbors are 63 mm apart at 5 m and 151 mm at 12 m - one fixed
  number would either shred distant objects or weld near ones), then an oriented
  box per cluster by principal axis. Drawn as **corner brackets**, not closed
  rectangles: the lidar sees one face, so the box is a bound and not a shape, and
  open corners say bound. PCA rather than min-area calipers because the true
  minimum-area box of a partial outline is not more correct, just less stable
  frame to frame - which on a live display is the thing that matters.
  The ring is closed, so the scan is rotated to start at its largest gap; that
  removes the wrap-around case instead of special-casing it.
- **The corridor.** How far the car could drive straight before something enters
  the width it sweeps, colored green/amber/red.

**Two bugs the screenshots caught.** The corridor read `0.02 m` - returns *inside
the chassis* were being treated as obstacles. A return at 80 mm dead ahead is
inside the car, and once the C1 is mounted the bodywork will occlude that bearing
entirely; excluded now. And every box carrying a label turned the near field into
an unreadable pile, so labels are placed in priority order - in path, then
nearest - and one that would land on a label already down is dropped. The
corridor's own label claims its space first: it is the only number on the display
that is a driving decision rather than an observation.

Verified live: the map's `19 objects (3 in path) | 0.32 m ahead` and the
instrument panel's independently formatted `objects 19 / ahead 0.32 m` agree,
which is two code paths cross-checking rather than one being read twice.

---

## 2026-08-25 — the map goes to a terminal palette

Not a dial-back of the skeuomorphism this time. A replacement.

`ui::ansi` is the xterm sixteen, and it is the map's palette and nothing else's.
The chrome around it stays industrial slate, and the board view keeps `plot::` -
that panel is a picture of a physical object, and a photograph of a PCB has no
business being drawn in sixteen colors.

The map is a different kind of thing. It is a readout, and a readout wants
maximum separation between its few meanings with no ambiguity about which one you
are looking at. That is the exact problem saturated primaries on black were
designed for, and it is what the graphite treatment was working against: every
hue had to survive being laid over a tinted, unevenly lit ground, so none of them
could be itself.

### What went

| Removed | Why |
|---|---|
| `drawScreenBloom` | Sixteen stacked discs claiming a lamp behind the panel. |
| `drawGlass` | A vignette dimming the corners of a measurement display. |
| `ENGRAVE_SHADOW` + the two engraved primitives | A 1 px shadow under every ring and tick, to make flat lines look scribed. |
| The per-mode background tint | Fourteen different grounds; see below. |
| The plate bevel | Highlights on the box that exists to protect a number. |
| The map's `screenInset` bezel | Replaced by a 1 px rule. The board view keeps its bezel. |

### The background

All fourteen modes return `0x000000` now - a real black, not a dark blue standing
in for one. The tint told you which mode was active, but the mode toggle already
does that in words, and the cost was that no data color could be fully itself.
On a terminal palette the ground has one job: be the thing every other color is
maximally far from. Anything other than black is worse at it.

### The assignments

- **Distance: red near, yellow mid, green far.** A traffic light. This is also why
  the ramp no longer ends in cyan - cyan is the heading marker, and a far wall
  should not be the same color as the direction the car is pointing.
- Density blue -> magenta -> white; walls bright cyan; gaps bright green; motion
  bright red; clearance cyan; sweep cyan -> magenta -> yellow.
- The dead zone is red now instead of amber, dropping the recessed disc and the
  bevel rings entirely. Red is what this palette already spends on "there is
  nothing here", and the region is the transmitter's own shadow, not a hazard.
- The sensor is a ring and a crosshair. It marks 0,0 exactly without covering it,
  which the lit-component-in-a-socket did not.

**The one departure from the sixteen** is the grid: two neutral grays, `0x3A3A3A`
and `0x7F7F7F`, because a range grid at full brightness competes with the returns
it exists to measure. Their alpha then had to come down again after the first
screenshot - values tuned against a lit graphite substrate read brighter than the
data once the ground went black.

### Broke it twice on the way

Deleting the engraving block took `strokeRing` with it - the helper sat between
`ENGRAVE_SHADOW` and its own engraved wrapper, so a range-delete bounded by those
two landmarks swallowed the function every ring on the map calls. Restoring it
from the backup then put it back at the *original* line, which is below its first
caller; C++ needed it above. Same class of mistake as the earlier block
replacements that ate their neighbors, and the same lesson: bound a deletion by
what it *contains*, not by the comments on either side of it.

---

## 2026-08-25 — motor control, and the gradient pulled back

### The skeuomorphism was overdone

Pulled back hard, because the previous pass crossed from describing an object
into decorating one:

- The center bloom went from 16 discs at alpha 5 (a ~80/255 lift at the hub) to
  10 at alpha 2 (~20). It should hint that the panel is lit. What it was doing
  was a spotlight.
- The vignette dropped from 46 to 26.
- **The diagonal sheen is gone entirely.** A light streak across a measurement
  display is the exact point where skeuomorphism stops being a description and
  starts being an effect, and it was the most obviously fake thing on screen.

### Stopping the motor

`LidarSource::setMotorEnabled()` parks the rotor without dropping the link. It is
separate from `stop()` because they answer different questions: `stop()` means "I
am done with this device", this means "stop making noise and wearing the bearing,
I am still here". The worker stops the scan, waits 200 ms, then cuts the motor —
that order matters, since killing the motor under a running scan leaves the
device streaming into a stopped rotor.

**The first version of the button was wrong and testing caught it.** The quick
action was keyed on `isBusy()`, which is `state == SCANNING`. Pausing sets the
state to idle, so the control that would restart the motor turned into "Connect
lidar" — you could stop it and not start it again. It keys on a new
`connected()` (the worker holds the port) now, and the strip says **"Motor off"**
rather than "Not connected", because those are different situations and a lidar
that is attached-but-parked should not claim to be absent.

### Does closing the app stop the lidar? Measured, not assumed

| Exit | Bytes still streaming 1 s later |
|---|---|
| `WM_CLOSE` (the X, Alt+F4, logoff) | **0** |
| `TerminateProcess` (Task Manager, `Stop-Process -Force`, a crash) | **9744** |

So the clean path was already correct — `WM_ENDSESSION`, `WM_DESTROY`, a lifetime
guard and a final call in `WinMain` all route to the same idempotent shutdown.

**A force-kill cannot be fixed from inside the process.** `TerminateProcess` runs
no user-mode code by design; there is no handler to install. Two things were done
about it instead:

- The teardown now stops the motor **unconditionally** once the device has been
  opened, rather than only when `scanning` was true. With a pause control the
  motor can be off while scanning is true, or the scan stopped while the rotor
  coasts — and the cost of telling an already-stopped device to stop is nothing,
  while the cost of skipping it is a lidar spinning on a desk with no application
  attached.
- Connect now issues `stop()` **before** starting, so recovery from a killed
  session is deterministic rather than relying on `startScan` to reset a device
  that is already mid-scan.

---

## 2026-08-25 — the viewport becomes a screen, not a background

The map still read as a flat plot on near-black, which is what "not aligned with
the skeuomorphic design" was pointing at. Three changes, and none of them touches
a single return.

### The substrate is colored, and it is lit

Flat neutral black was the single thing making this look like a chart on a page.
A phosphor screen is never neutral: it is a dark, faintly **colored** panel
behind glass. Every mode's ground moved to a deep blue-graphite — the modes now
shift its hue rather than its idea.

`drawScreenBloom` then stacks sixteen very low-alpha discs outward from the
sensor, so the display is brightest where it is *driven* and falls off toward the
bezel. Together with the vignette already in `drawGlass` that is the same idea
worked from both ends — lit in the middle, dark at the edge — and it is what makes
the viewport read as a surface with a light behind it instead of a rectangle of
one color.

### The blind zone was a warning sticker

It was an amber disc, with amber hatching, and an amber rim: the visual language
of a hazard label. It was therefore the **loudest thing on the display**, despite
being the one region that never contains data.

It is the transmitter's own shadow. The honest depiction is a patch of screen
that is not driven — so it is now *darker* than the substrate, sunk into it with
the same inset bevel the viewport carries, machined with dim neutral hatching
rather than diagonal stripes. The amber survives only as a thin scribed line on
the boundary itself: a limit worth marking, not worth shouting.

The 12 m envelope at the other end of the range got the same restraint.

### The furniture belongs to the panel

Grid, major rings, axes and labels were neutral steel against what is now a blue
substrate, so they were subtly the wrong family. All warmed toward the ground
they sit on.

The returns are still untouched — see the note in the entry below on why
decorating a measurement is out of bounds.

---

## 2026-08-25 — Walls and Full

Fourteen map modes now. Both of the new ones are *calculated* rather than drawn.

### Walls — line fitting

Iterative end-point fit (split-and-merge) over each contiguous run of returns:
recursively split at the point furthest from the chord until every remaining
segment is straight, then keep the ones long enough to be a surface.

**The tolerance comes from the sensor, not from taste.** The C1 is specified to
+/-30 mm, so anything inside ~45 mm of a straight line *is* a straight line as far
as this device can tell, and splitting there would be fitting noise. Minimum
length 250 mm, minimum 6 points.

This is a real step past Contour, which only joins adjacent dots and will happily
trace a curve or a cloud. Walls asks the harder question and only draws what
answers yes, so what comes out is a landmark list. The raw returns stay visible
underneath, dim: a fitted wall is *inferred*, and the evidence has to sit beside
the inference.

Reads `2 walls fitted, longest 0.71 m at 135 deg` in a cluttered room — which is
the honest answer from a sensor sitting among desk objects, not a failure.

### Full — the field display

Every derived layer at once, with the numbers on the map: free-space polygon,
contour, the returns at full brightness, a ray to the widest drivable gap, and an
eight-row instrument panel down the right-hand side.

The case it exists for is standing next to the car outside with a laptop and
nothing else. No second screen, no panel to open, no hover to discover. That is
also why it is the only mode that deliberately duplicates what the sidebar
shows — out there, there may not be a sidebar in view.

It computes its figures **independently** of the telemetry path, which turned out
to be a free cross-check: rate, returns, in-spec, near, mean and furthest all
matched the Live panel exactly on the first run (9.8 Hz / 511 / 65% / 0.12 / 1.42
/ 7.39). Two separate code paths agreeing on six numbers is worth more than
either of them agreeing with itself.

The panel sits at the right edge, vertically centered — the only quarter of the
map the existing HUD does not already use.

---

## 2026-08-25 — the map's own drawing goes tactile

The viewer had a bezel and a lit hub but everything *inside* it was still flat
vector work. This pass takes the map's furniture the rest of the way — and
deliberately stops short of the data.

### Engraving

`strokeRingEngraved` and `lineEngraved` draw a dark line one pixel below the
bright one. That is how a line scribed into a panel reads: the groove shadows on
its lower edge and catches light on its upper. Applied to every range ring, every
compass tick and the scale bar, it is the single cheapest thing that stops the
furniture looking like a flat vector drawing.

The offset is **one physical pixel**, for the same reason the button bevel is:
scale it with DPI and it stops being a groove and becomes a second line.

### Glass

`drawGlass` puts a vignette into the corners and a soft sheen in the top-left,
as a panel-mounted screen has under room light.

Where it goes in the draw order is the whole design: **over the returns and the
sensor, under every number.** A vignette that dims a range label would be trading
legibility for an effect, which is the wrong way round on an instrument. Both
layers are held very low (46/255 and 12/255) for the same reason.

### Indicators emit, they do not draw

The heading arrow and the nearest-return marker are *indicators*, not
measurements, so they now get the lamp treatment the chrome uses — a wide dim
pass under a narrow bright one, and a halo plus a core for the nearest ring —
rather than being flat strokes like the grid they sit on.

### What was deliberately left flat: the returns

The dots, cells and rays are **measurements**, and this is where a skeuomorphic
pass has to stop. A glossy blob has a bright center and a soft edge, which reads
as a position and a confidence that the sensor never reported — decorating a
return would be inventing data. They keep the flat, honest disc they have always
had.

The derived overlays (clearance polygon, sector wedges, gap wedges) sit in
between: they are computed rather than measured, so they carry edge highlights,
but their geometry is still exact.

---

## 2026-08-25 — the status strip, and icons on the overlays

### The strip was cropped, not off-center

Reported as "not vertically centered". It was centered — it was **clipped**. The
strip child was sized to exactly one text line, and a lit `ui::led` throws a halo
out to ~2.6x its radius, so the top and bottom of every lamp was being cut off by
the child bounds. That reads as a misaligned dot, which is what it looked like.

Two fixes, and the first one was also asked for on its own:

- **The lamps are gone from the strip.** A lamp *and* a color-coded word said
  the same thing twice, which is what made the row feel heavy. An icon replaced
  the lamp and carries something the color does not: WHICH subsystem this is.
  Identity from the icon, state from the color, one channel each.
- **The strip is sized to `max(textLineHeight, iconSize)`**, not to the text
  alone. Sizing chrome to one of the two things inside it is what caused the
  original crop, and it would have come back the moment the icon scale changed.

The lamps stay everywhere else — the subsystem rows are table rows with real
height and their halos have always fitted.

### Icons on all twelve overlays

Previously skipped, with the reason given: twelve modes would need twelve
metaphors and most had no honest one. That was true of the *first* set of
candidates. A second search through Fugue turned up genuinely apt ones —
`layer-shape-polyline` for Contour, `chart-pie` for Sectors, `door-open` for
Gaps, `asterisk` for Rays, `clock-history` for Occupancy — so the objection no
longer applied and they are in.

Two details that matter more than the choice of picture:

- **Unselected icons are drawn at 130/255.** A row of twelve equally-loud icons
  competes with the selection, and the selection is the thing that has to be
  seen first.
- **The icon sits immediately left of the CENTERED label**, not out at the frame
  padding. These cells are wide; an icon pinned to the far margin reads as
  unrelated to the word in the middle of the same button. The quick-action
  buttons keep the margin placement, which is right for a toolbar key.

The `ICON_MODE_*` block is contiguous and in `MapMode` order, indexed
arithmetically rather than through a second table that could drift out of step —
with a `static_assert` holding that invariant.

---

## 2026-08-25 — an application icon, generated

`hub/assets/make_icon.ps1` renders `bibo.ico` at 16/20/24/32/48/64/128/256 —
graphite plate, top bevel, cyan radar rings with a sweep wedge, lit hub, and a
green status LED at sizes where it can be seen. **Generated rather than drawn in
an editor**, so the icon is made of the same parts as the UI and re-runs after a
theme change instead of drifting away from it.

Wired through `src/app.rc` (icon + VERSIONINFO), a `[rc]` step in `build.bat`,
and `LoadImageW` on the window class for the title bar and taskbar. The
VERSIONINFO carries the Fugue attribution in `LegalCopyright`, which puts it
somewhere a user of the *built binary* can see it — the CC BY license asks for
that and a file in the repo does not satisfy it on its own.

### Four things went wrong, and one of them mattered

1. **PowerShell has no inline `if` expression** in that position. `$(if ...)` is
   required; without it the width/height bytes silently never got written and the
   directory was corrupt.

2. **`return $out` unrolls the array.** PowerShell emits a `byte[]` as 25,000
   separate objects, so every consumer downstream saw `object[]` and the writer
   failed. `return ,$out` — the comma is load-bearing.

3. **PNG-compressed ICO entries are the wrong default.** They are legal since
   Vista and Explorer renders them, so the first version used PNG at every size.
   But GDI+ (`System.Drawing.Icon`) *cannot decode them*, which is how this was
   caught: the verification render failed while the file was perfectly valid.
   An icon that a whole class of tooling cannot read is a bad icon regardless of
   what the spec permits. Now: uncompressed DIB up to 64, PNG only at 128/256.

4. **`%SRC%` was never defined in build.bat.** I invented the variable; the .rc
   path resolved to `\app.rc`, `rc.exe` failed, and the build printed
   `[warn] rc.exe failed - building without an icon` and carried on — which is
   the behavior I wanted for a missing SDK and which neatly hid my own typo for
   two builds. The warning is still non-fatal (refusing to build over decoration
   is the wrong trade) but the path is now `%ROOT%src\app.rc`.

The structure is verified by parsing the finished `.ico` back: every directory
entry checked for a 40-byte BITMAPINFOHEADER, doubled height, 32bpp, and an
in-bounds offset.

---

## 2026-08-25 — the two viewports join the console

The chrome was tactile but the map and the board view were still flat areas
inside it, which made them read as *holes* in the design rather than as part of
it. Two new primitives fixed that, and both are deliberately the inverse of
something that already existed:

- **`ui::screenInset`** — a recessed bezel drawn just inside a region: shadow
  falling in from the top and left edges, light caught along the bottom and
  right. That is the exact inverse of the raised-key bevel, which is what makes
  a display and a button read as opposite mechanisms rather than as two
  rectangles. The inner shadow is a gradient, not a line, because a milled edge
  falls off over a couple of millimeters and at screen scale that is a few
  pixels. Applied to both central views.
- **`ui::plate`** — the raised-key treatment for custom-drawn chrome that is not
  an ImGui item. The map's HUD readout and the board's legend swatches now get
  the same bevel the buttons do, so a label sitting on the display belongs to
  the machine around it.

Inside the viewports:

- **The sensor hub** is drawn as a lit component seated in a socket — dark well,
  body, a rim arc catching light from above, then the emitter and its glow —
  rather than as two flat discs. It is the one piece of hardware *on* the
  display.
- **Map label plates** get a hairline of light on top and a dark seam below, at
  a lighter strength than the buttons. Deliberately lighter: there are dozens of
  these on screen at once and the full treatment turns the map into a wall of
  chrome.
- **Every indicator now goes through `ui::led`**, including the board's own LED,
  which previously had its own hand-rolled halo. It is the one real LED on
  screen and it should not be the odd one out.

---

## 2026-08-25 — denser, glossier, and icons everywhere

Three passes on the same brief, because the first attempt was too timid.

### Density

The reference tools are *dense*. Every spacing value came down —
`ItemSpacing` 7×5 → **5×3**, `FramePadding` 8×4 → **6×3**, `CellPadding` → 5×2,
`WindowPadding` → 6×5, scrollbar 11 → 10 px — and the type scale dropped a step
(body 17 → **15**, small 15 → **13**).

Shrinking type was the one to think about, because "don't make the text tiny"
was an earlier instruction and it still stands. 15 logical px lands at ~22
physical px on this display, which is comfortably readable; what it buys is
real — the sidebar now holds System, Sensors, the link block, all four telemetry
tabs, the sparkline, the clearance chart *and* the three collapsed sections
without scrolling. Previously it ran out of room at Telemetry.

### Gloss and lamps

Two additions that make the surfaces read as objects rather than rectangles:

- **Gloss.** A raised control now carries a soft sheen over its top half above
  the bevel — light falling on a molded surface, which is what "glossy charcoal"
  means. Kept at 15/255 because this is the first effect that tips into looking
  cheap.
- **`ui::led`.** A lit indicator throws a *halo* onto the panel around it plus an
  off-center hot spot, so it reads as emitting rather than as a colored dot
  painted on; an unlit one is a dark lamp in a recessed socket. Both states are
  drawn — a socket is always there — so they differ in more than brightness.
  Used on the status strip and on every subsystem row.

### Icons, properly spread

From 16 to **32**, and from 3 places to 9: central tabs, sidebar section
headers, quick actions, subsystem rows, sensor rows, telemetry tabs, console
tabs, and the flash controls.

**Not on the map-mode toggle**, deliberately. Twelve modes would need twelve
metaphors and most of them (Density, Validity, Sweep) have no honest one — a
wrong icon is worse than no icon, and those already carry hover tooltips that
say what they do.

### The bug that came back twice

Tab labels reserve room for their icon with leading spaces, because
`BeginTabItem` takes a string and there is no way to put a picture inside one
without reimplementing the widget. The first version hard-coded three spaces.
That worked, right up until the type scale changed in the same session — and the
icons landed on top of their labels, because icon size and space width move
independently.

It is computed now, from `CalcTextSize(" ")` and the live icon size. Guessing a
layout constant that depends on two other constants is not a shortcut; it is a
bug with a delay on it.

---

## 2026-08-25 — Dark Aero on an industrial slate console, and real icons

Back to dark, but **graphite, not black** — and that is the whole point rather
than a detail. UDK, Maya, 3ds Max, Blender 2.7x, Photoshop CS5/6 and the DAWs all
sit in a narrow low-contrast band because people stare at them for ten hours;
near-white type on pure black is the highest-contrast pairing available and the
worst one to work in. Measured off the running app: window **21%**, panel 23%,
button plate 29%, viewport 5%.

The viewport stays darker than the chrome — a display set into the casing rather
than part of it — but it is `#0E0F12`, not `#000`. Pure black against a 23% panel
is a harder edge than any of those tools would draw and it throws away the
low-contrast ground the rest of the theme exists to provide.

### Tactile controls

`ui::bevelRect` gives every control one pixel of light along its top edge and one
of shadow along its bottom, **inverted when pressed** so a key visibly sinks into
the panel instead of merely changing color. Buttons are raised keys; checkboxes
and text fields take the *inverted* bevel because they are wells milled into the
casing — that difference is what makes the two read as different mechanisms.

One pixel means one **physical** pixel. This is the only measurement in the theme
that deliberately ignores the DPI scale: a bevel that grows with DPI stops
reading as a machined edge and starts reading as a border.

Verified by sampling a vertical slice through a button rather than by eye:
`#57595E` top edge, `#494C51` plate, `#252628` bottom edge.

### Fugue Icons

[Fugue Icons 3.5.6](https://p.yusukekamiyamane.com/) by Yusuke Kamiyamane — the
icon set of exactly this era. 25 of its 3,570 in use, on the tabs, the section
headers and the quick actions.

- **CC BY 3.0, and attribution is a condition, not a courtesy.** It is in
  `README.md`, `hub/assets/ATTRIBUTION.md`, and the pack's own README shipped
  verbatim as `assets/icons/LICENSE.txt`. If an About box ever appears it belongs
  there too.
- **Decoded with WIC**, which ships with Windows, so no image library is
  vendored, built or licensed. The app is already Win32 + D3D11; the platform's
  own PNG decoder costs nothing.
- **One atlas texture**, not sixteen. Each separate texture is a draw-call
  boundary in ImGui's draw list, and a toolbar is a lot of them.
- **Integer scale only.** These are 16×16 pixel art hinted at that size, and the
  pack's 24/32 px bonus sets do not include any of the names used here — checked,
  not assumed. A 16 px source stretched to 24 is mush; 16 px next to larger text
  is merely small, which is what toolbars of that era looked like anyway.
- A missing asset folder degrades to no icons, never to a crash: the app's job is
  talking to a lidar.

Since chrome and viewport are both dark again, `ui::sem::` goes back to
forwarding to `ui::plot::` — one green means one thing everywhere. The two-stop
split below existed only while the chrome was light.

---

## 2026-08-25 — light chrome, dark viewport, and two palettes

The app is now **cream**, in the manner of NetBeans and the GNOME/Tango desktops
of the same years — the era this thing is dressed as was mostly a light era. The
lidar viewport stays **dark**, which is the usual arrangement for an engineering
tool: light chrome around a dark data surface. The Windows title bar is forced
light too (`DWMWA_USE_IMMERSIVE_DARK_MODE = FALSE`) — it follows the *system*
theme unless told otherwise, so on a dark-mode machine a light app was getting a
black title bar.

### The consequence nobody asks for up front: two grounds need two palettes

The same state — "scanning", "out of spec" — is now printed on two different
backgrounds. Tango's **dark** stops (`#4E9A06`, `#CE5C00`, `#A40000`) are what
read on cream; its **bright** stops (`#8AE234`, `#FCAF3E`, `#EF2929`) are what
read on black. Each is close to invisible on the other.

So the palette split by GROUND, not by meaning:

| | drawn on | stops |
|---|---|---|
| `ui::plot::` | the dark viewport — map, board view, HUD | bright |
| `ui::sem::` | the light chrome — sidebar, status strip | dark |

`sem::` used to simply alias `plot::`. It cannot any more. The failure mode is
nasty because it is quiet: pick the wrong one and the text is *technically
there*, just unreadable. `lidarStateColor()` therefore has a
`lidarStateColorOnViewport()` twin — same state, same hue, different stop —
because that one value is printed in both places.

### Pin categories were never statuses

The complaint was the orange pin names on the Pico board, and it was right: an
assigned pin was drawn in `sem::GOOD` and a power/ground pin in `sem::WARN`, so a
whole column of `GND / VBUS / VSYS / 3V3` labels sat there in **warning orange**
announcing a problem that did not exist. Power is not a warning. An assigned pin
is not "healthy".

What the three categories actually encode is one axis — how much this pin has to
do with the project — so `ui::pin` is now **one hue at three weights**: blue for
ours, mid gray for structural, dim gray for unused. No green, no orange, and the
column reads as a list instead of an alarm.

---

## 2026-08-25 — twelve map modes, each with a reading

Four more: **Sectors** (coarse 12-wedge rose), **Gaps** (openings wide enough to
drive through, with widths), **Validity** (why a bearing failed), **Sweep**
(position within the revolution). Twelve total.

Three things went in alongside them, and they matter more than the count:

- **Every mode prints its own number.** `RadarView::diag` is one line the active
  mode fills and the HUD renders under the mode name — widest gap and bearing,
  tightest sector, the in-spec breakdown, the sweep duration. A mode without a
  number is a picture.
- **Hover tooltips.** Twelve one-word labels, several of which say nothing to
  anyone who has not read the source. Each carries what it draws and how to read
  it, from a table beside the enum so a new mode cannot be added without one.
- **Per-mode backgrounds.** The viewer is no longer pure black in every mode; it
  tints toward the mode's own palette. Raw-geometry modes keep black, because
  nothing is being encoded by hue there and nothing should be implied.

### Three bugs, all of the same shape

Each was a case of drawing something true that could not be *seen*:

1. **Sectors vanished.** A sector with nothing inside 12 m was drawn at 12 m —
   off screen at any normal zoom, so eight of twelve wedges were invisible. They
   clamp to the view edge now and get a **broken** arc, so "the wall is here" and
   "the view ends here" stay distinguishable.

2. **Then they were still invisible**, and the instinct to reach for higher alpha
   was wrong. Instrumenting instead of guessing said `open=0`: nothing was
   clamped at all. With the sensor among desk clutter, *every* sector's nearest
   was 0.14–0.25 m, so the wedges were a dozen pixels long and their labels were
   a pile at the origin. Labels moved to a fixed ring with dividers; the wedge
   still carries magnitude, the ring carries the reading.

3. **The tooltip never appeared** with `ImGuiHoveredFlags_DelayNormal`. That flag
   also implies `Stationary`, which never satisfied for a cursor placed by
   `SetCursorPos` — which is how this is verified. Plain `IsItemHovered` works
   and ships. An unverifiable nicety loses to a tooltip that provably appears.

The mode strip keeps its two rows and grows a horizontal scrollbar once cells
would be narrower than a label, rather than shrinking them into ellipsis. The
layout and the strip share one `modeToggleScrolls()` predicate so they cannot
disagree about whether that scrollbar exists.

---

## 2026-08-25 — the palette goes Frutiger Aero

Geometry stayed (squared corners, outlined widgets, flat plates — the Unreal /
IntelliJ pass). Only the **color** changed, and it changed a lot: that era is
high chroma, and the muted mint-and-salmon it replaced is exactly what was making
the app read as a 2020s product rather than a panel.

Three anchors, everything derived from them:

| | | |
|---|---|---|
| **Cyan** | `#00A6E6` | selection, check marks, active tab, the heading arrow |
| **Green** | `#3FD35C` | connected, scanning, in spec, cyw43 up |
| **Amber** | `#F7A128` | waiting, degraded, out of spec, BOOTSEL, spec boundaries |

Red `#F0453B` for failure, steel `#8496A6` for idle.

**Cyan is deliberately not a status.** "The UI is pointing at this" and "this is
healthy" are different claims and must not share a color — which is why the
accent lives in its own `ui::accent` namespace rather than in `ui::plot`.

Three changes did most of the work, and none of them is an accent color:

- **Chrome is brushed steel, not neutral gray.** Every surface carries a few
  percent blue cast — what metal looks like under that era's cool light. Neutral
  `#262626` is a modern app's gray and reads as one.
- **Text is cool white** (`#DAE5F1`), not ImGui's stock neutral `0.90`. Single
  biggest shift in the whole pass for its size.
- **The distance ramp ends on the cyan**: amber near → lime → cyan far, so the
  far field is the sky end of an Aero gradient rather than an unrelated blue.

The blind-zone disc and the 12 m envelope moved from dusty rose to hazard amber,
joining `sem::WARN` — they are spec boundaries, which *is* a warning, so they
should not have had a private color in the first place.

The board view needed no changes: it draws a physical object (green mask, gold
pads) and its category stripes read `sem::` directly, so it followed on its own.

---

## 2026-08-25 — the bottom bar belongs to the view

The render-mode toggle and the map controls were drawn below the tab bar
unconditionally, so the Pico 2 W tab inherited Points/Rays/Density and
Grid/Trail/Labels — controls that configure the map and nothing else. Each view
now declares its own bar, and **zero rows is a valid declaration**: the board tab
has no bar at all rather than a disabled one, and gets the height back (the board
draws visibly larger for it).

The layout has to reserve that height *before* the tab bar has run, so the
selected view is kept across frames rather than queried. Switching tabs therefore
sizes the bar from the outgoing view for exactly one frame — checked by clicking
between the tabs at runtime, not just by launching into each with `--view`, and
it is not perceptible at 240 fps. `--view` seeds the stored selection so even the
first frame is right.

---

## 2026-08-25 — `hub/` converted to Jack's C++ Style Guide

~4000 identifiers across 9,700 lines. Every phase was compiler-verified and the
app was run between phases, not just built.

What changed: functions/variables/members/params to `camelCase` (no `g_`, no
`m_`, no trailing `_`), `constexpr` constants to `SCREAMING_SNAKE` (no `k`
prefix), enum members to `SCREAMING_SNAKE` prefixed with the enum name, headers
to `.hpp`, `if (` to `if(`, all 251 C-style casts to `static_cast`, and the
builtin types to the new `hub/src/shared.hpp` alias layer.

### Three ways a mechanical rename goes wrong, in increasing order of nastiness

1. **It hits a third-party name and fails to compile.** Cheap. The first pass
   renamed `std::steady_clock`, `lock_guard` and `memory_order_release` because
   the foreign-identifier filter scanned the SDK and ImGui but not the standard
   library.

2. **It hits a name that only *looks* like an identifier.** `L` was a
   single-letter method on the board painter, so `L` -> `toPx` also rewrote every
   Win32 `L"..."` string prefix in main.cpp. Single-letter method names are worth
   avoiding for this reason alone; it got a real name (`toPx`) rather than a
   lowercased one.

3. **It merges two identifiers and still compiles.** This is the dangerous one.
   A local `impl` and a member `impl_` both mapped to `impl`, turning
   `Impl* impl = impl_;` into `Impl* impl = impl;` — a self-initialization that
   compiles clean and reads plausibly. Same class of bug hit `g_max_mm = max_mm`,
   which silently stopped publishing the max-range readout.

**Reverting via the inverse map does not undo (3).** The merge is not injective,
so the inverse restored one name where there had been two. Two more casualties
were found only by grepping for `x = x;` afterwards.

The fix for the second attempt: refuse any rename whose TARGET name already
exists anywhere in the project, and report the refusals rather than resolving
them silently. That turned 9 collisions into a decision instead of a bug.

### RadarView lost its trivial getters

A class cannot have both a member `radiusPx` and a method `radiusPx()`, and the
guide asks for public fields where a getter would only forward. So the cached
readouts — `centerPx`, `radiusPx`, `cursorValid`, `nearestMm`, `measuring` and
the rest — became public fields, matching `mode` and the overlay flags that were
always public. `visibleRangeMm()`, `toScreen()` and `toWorld()` compute something
and stayed methods.

### Casts needed a parser, not a regex

`(int)a + b` means `((int)a) + b`, because a C-style cast binds to the unary
expression that follows it. A regex grabbing "the rest of the expression" yields
`static_cast<int>(a + b)` — which compiles, type-checks, and is a different
number. The converter scans exactly one unary-expression with balanced brackets
and refuses anything it cannot parse. It refused 2 things, both function-pointer
typedefs (`typedef UINT (WINAPI* PFN)(HWND)`), correctly.

### Not done, and why

**MSVC stays.** The guide says never MSVC. The Slamtec SDK ships `.vcxproj`
files pinned to toolset v142 and its driver library is `/GL` LTCG, which `hub/`
links directly — so the guide's GCC/Clang flag set, `-Wold-style-cast` included,
is unavailable here. `/W4` is clean; the cast rule is held by review.

**Bare `long` is not aliased.** It is 32-bit on Windows and 64-bit on the
Linux/macOS targets in the porting plan, so `Int32` or `Int64` would each be
wrong on one of them. The uses read `%ld` through `sscanf`, where the spelling
has to match the format specifier regardless.

---

## 2026-08-25 — editor chrome, eight map modes, and a board that shows its state

### The UI is now Unreal/early-IntelliJ, not Material

Flat neutral grays, widgets **outlined**, corners nearly square (5 px → 2 px),
one accent blue used only where something is selected or set, and tighter
spacing. The ground stays pure black, which was asked for earlier and which both
reference tools are close to behind their panels anyway.

Two carry-overs were kept deliberately rather than reverted:

- **Containers still have no border.** The earlier complaint was about sidebars
  and boxes ringed inside other ringed boxes. A *widget* outline is a different
  statement — it says "this is a control" rather than "this is a box" — so
  `FrameBorderSize` came back to 1 while child and window borders stayed at 0.
- **The soft elevation shading was cut, not deleted.** Unreal's buttons are flat;
  early IntelliJ's have the faintest top-down fall. What is left is a hint of
  light on the upper edge and no base shadow at all. The convex modeling
  belonged to the Material design this replaced.

**The parts of the palette nobody looks at were still stock.** `PlotLines`,
`PlotHistogram`, `NavCursor`, `DragDropTarget` and `TextLink` were left at
ImGui's dark defaults, which are a **blue-violet and an orange** — a different
theme showing through wherever one appeared. The clearance-by-sector chart in the
telemetry panel was drawing orange bars in a blue-accented app and had been for
as long as that chart has existed. They are rare, and rare is exactly why they
got missed.

Also swept, all of it toward the same target:

- **Scrollbar** — 14 px → 11 px with a faintly visible track and a square gray
  grab. The stock 14 px bar is a touch-sized affordance; a tool that expects a
  mouse gives the space to the content.
- **Combos** — ImGui fills the arrow area with `ImGuiCol_Button`, so against a
  darker `FrameBg` the drop-down read as a separate button welded to the right of
  a field. Both reference tools draw the arrow *inside* the field; `ui::Combo`
  pushes the button colors to match the frame and is otherwise stock.
- **Tab strip** — `TabBarOverlineSize` 1 → 2. The selected tab is marked by its
  overline, and at 1 px against a 2 px-rounded tab it read as an anti-aliasing
  artifact rather than a deliberate mark.
- **The map's own chrome** — the label plates behind ring numbers and the board
  view's legend swatches had their own hard-coded 4 px and 30 % radii. Both now
  read `style.FrameRounding`, so the map cannot drift away from the app around
  it. A 4 px plate against 2 px chrome reads as a different toolkit.
- `CellPadding`, `TextDisabled` and `InputTextCursor` brought in line.

**The checkbox had to be hand-rolled.** `ImGui::Checkbox` sizes its box to the
whole frame height, which at this type scale is a ~28 px square; filled with an
accent and rounded, it was the single most off-style control in the app. The
replacement is a small outlined square centered in a taller row, and when checked
the box stays dark — the accent arrives as the tick and the outline rather than
flooding. It is still a real ImGui item (an `InvisibleButton` over box + label),
so hover, nav and `BeginDisabled` all behave.

Two things this shook out:

- `ImGui::FindRenderedTextEnd` is **imgui_internal only**. The rule it implements
  is one line (everything from the first `##` is an ID), so it was inlined rather
  than pulling in that header.
- Turning on frame borders made the sensor list read as a **column of text
  fields**, because every unselected row was a `Button` and now got outlined.
  Selected-only drawing with an accent left bar fixed it, and the same
  `SegmentedButton` now backs both the sensor rows and the mode toggle.

### Three more map modes — Contour, Clearance, Motion

Eight total; the toggle wraps to two rows past five. All three were chosen for
temporal stability, which is what makes Density readable.

**Clearance was wrong on the first attempt and the failure is worth recording.**
It drew radial spikes to the range ceiling in a dozen directions. Cause: 240
bearing bins against ~505 points a revolution is two samples a bin, so a quarter
of bins came up empty *by chance*, and an empty bin was being read as "clear to
12 m". Fixed by halving the bin count to 3° (four samples a bin) and, more
importantly, by treating a single empty bin as **no information** — it holds, and
only opens after four consecutive empty revolutions. An empty bin is genuinely
ambiguous: open space and a surface too dark to return look identical from here.

Motion requires the whole 3×3 neighborhood to be cold, not just the cell.
Angular jitter lands a stationary wall's returns in slightly different cells each
revolution, so a bare cell-was-empty test lights every wall edge permanently.

The clearance polygon uses a hand-written triangle fan, not
`AddConvexPolyFilled`: a room's profile is not convex and that call fills across
every doorway. It *is* star-shaped about the sensor by construction, so the fan
is exact rather than an approximation.

### The board tab shows the board, and what it is doing

Drawn as the object looks — green mask, white silkscreen, gold castellations,
black packages, shield can. Since every real pad is gold, the pin category moved
to a mask stripe inboard of each pad.

Live state is wired through from the link: LED (animated at the reported rate),
`cyw43` up/failed, link open, BOOTSEL, and GP0/GP1 driven with the µs in the
tooltip. Verified end to end against the board on COM10.

Two things the firmware taught us on the bench:

- **`blink_hz` beats `led=`.** A live `STATUS` returned `led=off blink_hz=0.50` —
  the firmware reports the LED's *instantaneous* level alongside its rate, so a
  non-zero rate means blinking whichever half the sample caught. The parser
  treats the rate as authoritative because of this reading.
- **Unknown is not off.** An LED nobody has asked about is drawn hollow, a
  reported-off LED dark and filled. Defaulting unknowns to off makes a board
  nobody has talked to look like one that answered and said no.

The blink phase is not synced to the board and does not pretend to be — nothing
reports where in its cycle it is.

**The Pico never auto-connected.** This log has claimed since the restructure
that the hub launches with no arguments and connects *both* devices. Only the
lidar was ever wired up; `ConnectPico()` existed but nothing called it at
startup. The board view made it obvious, since it has nothing live to show until
the link is open. Fixed — `--no-connect` still suppresses both.

`STATUS` is polled every 2 s, but only while the board tab is on screen, and it
stops permanently after one refusal: `tt02_control` answers `ERR bad command` and
must not be asked forever.

---

## 2026-08-25 — the center stops being only the map

Top tabs above the central panel, and a segmented mode row under it. The map is
now one central view among several and one of five render modes, rather than the
only thing the window can draw.

The board tab is a to-scale Pico 2 W with all 40 pads hoverable. It is a pinout
that **cannot go stale**: the project's own assignments are compiled in beside
the physical pinout, so it contradicts `docs/wiring.md` loudly if the two ever
diverge, instead of quietly being wrong the way a drawing in a document is.

Three things worth keeping:

- **Auto-fit was jittering** because the fitted range was a continuous function
  of the scene — a 95th-percentile distance that moved a few centimeters redrew
  every ring label. It now quantises to the same 1/2/5 × 10ⁿ ladder the rings
  use, grows immediately, and shrinks only below 55% of the current step. Reads a
  locked `10.0 m` across samples seconds apart; it was flipping 4.7 ↔ 10.0.
- **The status strip was sized off `GetFrameHeight()`** — the height of a
  *widget* — while containing only text. Sized to `GetTextLineHeight()` plus its
  own padding instead.
- **A hover highlight sized to its hit box.** The board's pin rows are given a
  generous 17.5 mm hit column so they are easy to point at; drawing the highlight
  at that width left a bar hanging in empty space beside the short names. The
  highlight is measured from the widest label per side now. Hit target and
  highlight are different questions and should not share a number.

`--tab map` and `--tab pico` were removed rather than kept working: `map` and
`pico` now name central views, and one word selecting two different things is a
trap. `--view` picks the center, `--tab` picks the sidebar section.

---

## 2026-08-25 — the viewer becomes the application

`lidar/viewer/` moved to `hub/` and now builds **`bibo.exe`**. The old path said
"a lidar accessory" while the thing had become the operator front end for the
whole project: sensor map, Pico link and command set, firmware build/flash/
backup, console. One executable, launched with no arguments, auto-connecting
both devices.

Two things this shook out, both broken for a while and unnoticed because nothing
exercised them:

- `hub/tests/build_test.bat` still pointed at `imgui_viewer/src/` and at
  `rplidar_sdk/` in the repo root. Both moved during the first restructure and
  the script never followed, so that test had been unbuildable since.
- `pico_flash.cpp` located the repo root by looking for `firmware/` **and**
  `lidar/`. It still worked by luck, since `lidar/` survives as the sensor's
  notes directory, but keying the root off one sensor's folder was fragile. Now
  `firmware/` + `hub/`.

The UI also lost every panel border. Structure comes from surface and spacing
now: panels lift a hair off black rather than being ringed. That change had to
touch the call sites too - 16 of them passed `ImGuiChildFlags_Borders`
explicitly, which overrides the style, and the first attempt at "borderless"
changed nothing visible until those were found.

---

## 2026-08-24 — Windows becomes the primary dev machine

Restructured the working directory into the `bibo` layout. The RPLIDAR work
done here was absorbed rather than redone: the viewer moved to `lidar/viewer/`,
the CLI bridge to `lidar/bridge/`, and the Slamtec SDK clone to
`vendor/rplidar_sdk/` (gitignored). Build scripts were repointed at the new SDK
path and the viewer was rebuilt from its new home to confirm nothing broke.

The MacBook `bibo` directory is superseded by this one.

### Pico 2 W bring-up — the board was never dead, and it was not empty

Connected the Pico 2 W to the Windows machine. It enumerated as **VID_2E8A
PID_0009** — a USB composite device with a CDC serial interface (COM10) and an
`MI_02 "Reset"` interface, which is the Pico SDK's USB reset endpoint.

**The board already had purposeful firmware on it.** `picotool info` named it
**`tt02_control`** — the car control firmware, written on the MacBook. It answers
`?` with:

```
S 736762 0 0 1500 1500 736761
OK
```

Field 1 is uptime in ms (samples 1 s apart increment by ~1015). Fields 4 and 5
are **1500 — servo and ESC pulse widths in microseconds**, i.e. neutral. Field 6
tracks uptime, probably a last-command or watchdog timestamp. `?` is its entire
vocabulary; everything else returns `ERR bad command`.

This is worth reconciling against the build order below, which records phase 2 as
untouched. Firmware that emits neutral PWM on two channels clearly exists. What
is *not* established is whether a servo has ever been attached and moved. Treat
"the servo has never moved under code" as still true and "no phase 2 firmware
exists" as false.

**Its source is not in this repo** — only on the MacBook. Before reflashing, the
board's flash was read back to `vendor/tt02_control-backup.uf2` (94 KB,
gitignored). That .uf2 is currently the only copy on this machine.

#### The board looked silent, and was not

First contact showed nothing: the port opened, writes appeared to succeed, and
no reply ever came. The writes were in fact failing with *"the semaphore timeout
period has expired"*.

**TinyUSB CDC will not accept OUT data until the host asserts DTR.** Without it
the device never drains its OUT endpoint and writes block until the driver gives
up. PowerShell's `SerialPort` defaults `DtrEnable` to `$false`, so a naive probe
looks exactly like dead hardware. Assert DTR.

#### Four toolchain failures before the first build

None of these produce a useful error message; three of them produce *no* message
at all, which is what made them expensive.

1. **MSYS2's `usr/bin` cmake is Cygwin-flavored.** It uses POSIX paths and
   cannot drive a native Windows ARM toolchain.
2. **MSYS2's `mingw64` cmake dies with `0xC0000135` (DLL not found)** because
   this MSYS2 install is only partially updated. Fixing that means a full
   `pacman -Syu`, which was not worth it — **Visual Studio ships a native cmake
   and ninja**, and those work.
3. **Putting `mingw64\bin` on PATH breaks the SDK's own host tools.** They are
   built with ucrt64's gcc; prepending mingw64 shadows ucrt64's runtime DLLs and
   `pioasm` then exits `0xC0000139` *silently*, mid-build, while running fine by
   hand. PATH is left alone and `PICO_TOOLCHAIN_PATH` points at the
   cross-compiler instead.
4. **The picotool the SDK builds from source is broken here.** It reports
   *"compiled without USB support"* and crashes with `0xC0000005` in every
   subcommand that opens an ELF — both `uf2 convert` and `coprodis`. Replaced
   with Raspberry Pi's **official prebuilt** picotool (v2.3.0, GNU-16.2.0) via
   `-Dpicotool_DIR=`.

#### `RP2350`, not `RPI-RP2`

The flash script never found the board in BOOTSEL because it looked for a
removable drive labeled `RPI-RP2`. **That is the RP2040 label. RP2350 — Pico 2
and Pico 2 W — labels its drive `RP2350`**, and enumerates as PID `0x000F`
rather than `0x0003`. Both labels are now accepted.

Related: the 1200-baud touch did **not** reboot `tt02_control` into BOOTSEL, so
that firmware likely has `PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE` off.
`picotool reboot -u -f` drives the vendor reset interface directly and worked.

#### The LED is on the wireless chip

On a Pico 2 W the user LED is **not** an RP2350 GPIO — it hangs off the CYW43439,
so `cyw43_arch_init()` is required before it will do anything. On a non-W Pico
the same LED is plain GPIO 25. Get this wrong and you get firmware that runs
perfectly and never blinks, which is indistinguishable from a dead board.

The firmware reports the result rather than assuming it: verified live as
`cyw43=up`, and the LED responds to `LED ON` / `LED BLINK <hz>`.

#### One self-inflicted debugging error worth remembering

`cmd /c "some.exe & echo RC=%errorlevel%"` reads a **stale** exit code —
`%errorlevel%` is expanded before the line runs. Several "RC=0" readings were
meaningless and briefly suggested picotool worked when it was crashing. Put
multi-step checks in a `.bat` with separate lines, or use delayed expansion.

### RPLIDAR appeared dead on macOS, worked immediately on Windows

**This was diagnosed as a hardware fault and a message was sent to the AliExpress
seller. That diagnosis was wrong.**

Symptoms on macOS: no LED, no USB enumeration, no motor. On Windows the same unit
enumerated instantly, the adapter LED lit on connect, and it streamed valid scans
on the first try.

Root cause is macOS-side, not the device:

- Missing Silicon Labs CP210x VCP driver (the adapter uses a **CP2102**; Windows
  auto-installed it, macOS does not)
- and/or insufficient USB-C port current through an adapter chain

> **Action item: send a retraction to the AliExpress seller** so a dispute is not
> left open against working hardware.

Two things that look like faults and are not:

- The motor does **not** spin until the SDK issues a start-scan command. A
  stationary motor before that point is normal.
- One of the five positions on the XH2.54-5P connector is unpopulated. Correct.

### RoboStudio does not detect the device

Windows-only, and it failed to detect the C1 even with a valid COM port present
and the SDK talking to that same port successfully. **The SDK command-line path
is the working route** — do not spend more time on RoboStudio.

### Three real bugs found in the process

1. **Slamtec's SDK x64 build is broken.** All four configurations of the demo
   `.vcxproj` files hardcode the library search path to
   `output\win32\$(Configuration)` while `OutDir` correctly uses `$(Platform)`,
   so an x64 build writes the lib to `output\x64\` and the linker looks in
   `output\win32\`. Fails with `LNK1181`. Build Win32 for the demos; the driver
   library itself builds fine at x64, which is what the viewer links.

2. **`drv->connect()` returns success for a port that never opened.** In
   `sdk/src/sl_async_transceiver.cpp` an inner `Result<nullptr_t> ans` shadows
   the outer `u_result ans`; the `channel->open()` failure lands in the shadowed
   copy and the outer stays `RESULT_OK`. Practical effect: a nonexistent COM port
   reports "wrong baud rate?", pointing at entirely the wrong cause. Worked
   around by probing the port with `CreateFileA` before handing it to the driver.

3. **Git Bash mangles the device path.** `\\.\COM7` becomes `\.\COM7` and the
   connect fails. `MSYS2_ARG_CONV_EXCL='*'` does *not* fix it. Put such arguments
   in a `.bat` file.

### Viewer data-quality lessons

Both were cases of the display contradicting itself, and both were only visible
by looking at the running app:

- The blind-zone disc was drawn with a minimum on-screen radius so it would not
  vanish, which made it cover ~200 mm against a real blind radius of 50 mm.
  Returns from a hand held near the unit landed *inside* a region labeled
  "cannot see here". Now drawn true to scale.
- Out-of-spec returns were drawn but not counted. The rule now is: **whatever is
  not counted is not drawn.** The in-spec window is 0.05–12 m.

---

## Earlier — chassis assembly

### Motor mount position differs from the main kit manual

The Tamiya 540 Torque Tuned uses a **19T pinion** per the *motor's supplementary
sheet*, not the **22T** shown in the main kit manual. The smaller pinion requires
the **closer** motor mount position.

Following the main manual puts the motor where the gears physically cannot mesh.
**This cost real debugging time** — the failure looks like a bad part or a
mis-assembled gearbox, not a documentation conflict. When two Tamiya documents
disagree, the part-specific sheet wins.

### Deans/Tamiya adapter ordered in the wrong direction

Needed **Deans female → Tamiya male**. Ordered the opposite first. The correct
one is in hand and works. Connector gender is not symmetric and the listings do
not make the direction obvious.

### Shell windows and light sections painted over

Painted Tamiya PS-4 Blue on the **inside** of the clear lexan, which is correct —
the lexan becomes the glossy outer layer. But the windows and light sections were
not masked on the first pass and got painted over. Scraped clear and re-masked.

PS-12 Silver backing coat is ordered and pending. It goes over the blue once
cured, blocks light transmission, and makes the color read opaque rather than
translucent.

**2026-08-26 — silver applied.** PS-12 sprayed over the blue across the whole
inside. Curing; leave it a day before handling. The masking held this time, so
the windows and light sections are still clear.

Lidar hole not yet cut. It has to be cut *after* the cure, not during: painted
lexan chips inward from a cut edge, and the chip takes the paint with it. Cut
from the outside with a body reamer, backed on the inside, and open the hole in
stages rather than in one pass.

---

## Open items

- [ ] Send the retraction to the AliExpress seller (see above)
- [ ] Run the lidar calibration measurements — see [calibration.md](calibration.md)
- [ ] Cut the lidar hole in the shell — after the silver has fully cured
- [x] Apply PS-12 Silver backing coat once the blue has cured — 2026-08-26
- [ ] Solder the 6-pin header onto the MicroSD module
- [ ] **Wire the wheel encoder to GP15 and give the pin back.** The Hall
      sensor is in hand. GP15 currently drives a tail lamp - see the binding
      table in `firmware/lib/lights.h` - and a pin cannot be an
      interrupt-driven input and an LED at the same time.
- [ ] Move the lamps to their permanent pins: GP2/GP3 indicators, GP6/GP7
      tails, GP8 both heads. Two numbers per lamp in that same table.
- [ ] The second pair of indicators, front and rear. Already in the model and
      computed; only the rear pair has no pin.
- [ ] Decide the right lock. `cal.h` says 1670 and the hub'''s stored
      calibration says 1660, so the next "Write to firmware" will move it.
- [ ] **Recover `tt02_control`'s source from the MacBook.** The board's original
      firmware now exists on this machine only as a read-back binary
      (`vendor/tt02_control-backup.uf2`, gitignored). Get the source into
      `firmware/` before that .uf2 is the last copy anywhere.
- [x] **Phase 2: make the servo move under code** — 2026-08-27. Steering sweeps
      its full travel on command, and the ESC is verified. Calibrated on this
      car: left 1230, **center 1484**, right 1670, in
      `firmware/lib/chassis/cal.h`.

      Center is 1484, not 1500. The servo was binding at what the firmware
      called neutral, and that is why it kept stalling against the frame.

      The throw is asymmetric — 254 µs one way, 186 µs the other. Anything that
      assumes ±X µs from center steers further one way than the other, so the
      next layer needs a normalized command that maps through the calibration
      rather than adding microseconds to a midpoint.

      The evening it took was a missing common ground, hidden by a breadboard
      power rail that is split in the middle. See the invariant in
      [wiring.md](wiring.md).
