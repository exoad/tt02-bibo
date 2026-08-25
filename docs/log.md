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

1. **MSYS2's `usr/bin` cmake is Cygwin-flavoured.** It uses POSIX paths and
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
removable drive labelled `RPI-RP2`. **That is the RP2040 label. RP2350 — Pico 2
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
- [ ] **Recover `tt02_control`'s source from the MacBook.** The board's original
      firmware now exists on this machine only as a read-back binary
      (`vendor/tt02_control-backup.uf2`, gitignored). Get the source into
      `firmware/` before that .uf2 is the last copy anywhere.
- [ ] **Phase 2: make the servo move under code.** Still the gate for the whole
      project. Note the correction above: control firmware emitting 1500 µs
      neutral on two channels already exists and runs — what has not been shown
      is a servo actually moving. Toolchain, build, flash and backup all work
      from this machine now, so the remaining work is wiring and testing, not
      setup.
