# Build log

Newest first. **Entries recording what failed are worth more than entries
recording success** — the failures are what cost time and what gets forgotten.

---

## 2026-08-24 — Windows becomes the primary dev machine

Restructured the working directory into the `tt02-auto` layout. The RPLIDAR work
done here was absorbed rather than redone: the viewer moved to `lidar/viewer/`,
the CLI bridge to `lidar/bridge/`, and the Slamtec SDK clone to
`vendor/rplidar_sdk/` (gitignored). Build scripts were repointed at the new SDK
path and the viewer was rebuilt from its new home to confirm nothing broke.

The MacBook `tt02-auto` directory is superseded by this one.

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
  Returns from a hand held near the unit landed *inside* a region labelled
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
cured, blocks light transmission, and makes the colour read opaque rather than
translucent.

Lidar hole not yet cut.

---

## Open items

- [ ] Send the retraction to the AliExpress seller (see above)
- [ ] Run the lidar calibration measurements — see [calibration.md](calibration.md)
- [ ] Cut the lidar hole in the shell
- [ ] Apply PS-12 Silver backing coat once the blue has cured
- [ ] Solder the 6-pin header onto the MicroSD module
- [ ] **Phase 2: make the servo move under code.** This is the gate for the whole
      project and needs nothing that is not already on hand.
