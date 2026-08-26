# Third-party components

Everything in this project that someone else wrote, what its licence is, and
what that licence obliges. The project's own code is **not** open source — see
[COPYRIGHT](COPYRIGHT) — but several things below carry conditions that survive
that, and two of them bind any distributed **binary**, not just the source.

Most of these are **cloned, not vendored**: `vendor/` and `hub/third_party/` are
gitignored, so this repository contains no third-party source. It does contain
third-party **assets** (icons, one 3D model), which is why the attribution
obligations below are live rather than theoretical.

---

## Obligations that bind a distributed binary

Two, and both are easy to satisfy and easy to forget.

**Fugue Icons — CC BY 3.0.** Attribution is a *condition* of the licence. It has
to be visible to anyone who receives a build of `tt02.exe`, not merely present
in the source tree. Today it lives in `hub/assets/ATTRIBUTION.md`,
`hub/assets/icons/LICENSE.txt` and the README. **If the app grows an About box,
it belongs there too** — that is the moment the current arrangement stops being
sufficient.

**Slamtec `rplidar_driver.lib` — BSD-2-Clause.** Statically linked into
`tt02.exe`. Clause 2 requires binary redistributions to reproduce the copyright
notice and the disclaimer "in the documentation and/or other materials provided
with the distribution". Shipping the .exe alone would breach it; shipping it
alongside this file, or a NOTICE containing the Slamtec text, satisfies it.

The Pico SDK (BSD-3) carries the same shape of obligation for a distributed
`.uf2`, which in practice nobody distributes — but it is the same rule.

---

## Software

| Component | Version | Licence | Where | In this repo? |
|---|---|---|---|---|
| [Dear ImGui](https://github.com/ocornut/imgui) | 1.92.9 | MIT | `hub/third_party/imgui` | no, cloned |
| [Slamtec rplidar_sdk](https://github.com/Slamtec/rplidar_sdk) | — | BSD-2-Clause | `vendor/rplidar_sdk` | no, cloned |
| [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) | — | BSD-3-Clause | `vendor/pico-sdk` | no, cloned |
| [picotool](https://github.com/raspberrypi/pico-sdk-tools) | 2.3.0 | BSD-3-Clause | `vendor/picotool-2.3.0` | no, downloaded |
| TinyUSB, cyw43-driver, lwIP | — | MIT / mixed | inside the Pico SDK | no |

**Dear ImGui** is copyright (c) 2014-2026 Omar Cornut. MIT: do what you like,
keep the notice.

**rplidar_sdk** is copyright (c) 2009-2014 RoboPeak Team and (c) 2014-2018
Shanghai Slamtec Co., Ltd. Two clauses, no endorsement clause — see the binary
obligation above.

**Pico SDK** is copyright (c) 2020 Raspberry Pi (Trading) Ltd.

## Assets — these ARE in this repository

| Component | Licence | Where |
|---|---|---|
| [Fugue Icons 3.5.6](https://p.yusukekamiyamane.com/) | **CC BY 3.0** | `hub/assets/icons/*.png` |
| [Kenney Car Kit 3.1](https://kenney.nl/assets/car-kit) | CC0 1.0 | `hub/assets/models/` |

68 of Fugue's 3,570 icons are vendored, unmodified, at their native 16x16.
© 2013 Yusuke Kamiyamane.

One model of the Car Kit's fifty (`sedan-sports.obj` → `car.obj`) plus the kit's
shared `colormap.png`. **The texture is modified** — repainted to SWRT blue by
`hub/tools/livery.py`. CC0 permits this without asking or crediting; it is
recorded because knowing where a file came from is worth more than the licence
obliges.

Full detail, including exactly what was changed and why the mesh is scaled
non-uniformly, is in [hub/assets/ATTRIBUTION.md](hub/assets/ATTRIBUTION.md).

**`hub/assets/tt02.ico` is not third-party.** It is generated from primitives by
`hub/assets/make_icon.ps1` in the app's own visual language.

## Source files under someone else's terms

**`firmware/src/shared.h`** — BSD-3-Clause, copyright (c) 2026 Jiaming Meng. From
[manbox](https://github.com/exoad/manbox). Same author as this project, but it is
published under its own licence and carries its own notice, which must be
retained. Two typedefs (`Utf16`, `Utf32`) are a local addition, marked as such in
the file.

## Fonts — used, not redistributed

The hub loads **Cascadia Mono**, **Consolas**, **Lucida Console** and
**Segoe UI** from `C:\Windows\Fonts` at runtime. None is copied, embedded or
shipped, so no font licence attaches to anything here. Every one degrades to
Dear ImGui's built-in font if absent.

---

## Hardware, for completeness

Not licences, but the datasheets these were built against:
Tamiya TT-02 (kit 58631), Slamtec RPLIDAR C1, Raspberry Pi Pico 2 W (RP2350),
Hobbywing QuicRun 1060, Power HD 1501MG, Flysky FS-GT2. See
[docs/wiring.md](docs/wiring.md).
