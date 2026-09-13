# Third-party components

Everything in this project that someone else wrote, what its licence is, and
what that licence obliges. The project's own code is **not** open source — see
[COPYRIGHT](COPYRIGHT) — but several things below carry conditions that survive
that, and some of them bind any distributed **binary**, not just the source.

These are **cloned, not vendored**: `vendor/` and `third_party/` are gitignored,
so this repository contains no third-party source apart from the one file under
*Source files under someone else's terms*.

---

## Obligations that bind a distributed binary

**Slamtec rplidar_sdk — BSD-2-Clause.** Statically linked into the pilot
(`libsl_lidar_sdk.a`, when `firmware/pilot` is configured with
`-DPILOT_RPLIDAR_SDK`). Clause 2 requires binary redistributions to reproduce
the copyright notice and the disclaimer "in the documentation and/or other
materials provided with the distribution". The pilot is built on the board and
handed to nobody; shipping it would need this file, or a NOTICE containing the
Slamtec text, beside it.

The Pico SDK (BSD-3) carries the same shape of obligation for a distributed
`.uf2`, which in practice nobody distributes — but it is the same rule.

---

## Software

| Component | Version | Licence | Where | In this repo? |
|---|---|---|---|---|
| [Dear ImGui](https://github.com/ocornut/imgui) | 1.92.9 | MIT | `third_party/imgui` | no, cloned |
| [stb_image](https://github.com/nothings/stb) | 2.30 | MIT **or** public domain | `third_party/stb` | no, cloned |
| [Slamtec rplidar_sdk](https://github.com/Slamtec/rplidar_sdk) | — | BSD-2-Clause | `vendor/rplidar_sdk`, `~/rplidar_sdk` on the board | no, cloned |
| [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) | — | BSD-3-Clause | `vendor/pico-sdk` | no, cloned |
| [picotool](https://github.com/raspberrypi/pico-sdk-tools) | 2.3.0 | BSD-3-Clause | `vendor/picotool-2.3.0` | no, downloaded |
| TinyUSB, cyw43-driver, lwIP | — | MIT / mixed | inside the Pico SDK | no |

**Dear ImGui** is copyright (c) 2014-2026 Omar Cornut. MIT: do what you like,
keep the notice.

**rplidar_sdk** is copyright (c) 2009-2014 RoboPeak Team and (c) 2014-2018
Shanghai Slamtec Co., Ltd. Two clauses, no endorsement clause — see the binary
obligation above.

**Pico SDK** is copyright (c) 2020 Raspberry Pi (Trading) Ltd.

**stb_image** is by Sean Barrett and is released under **two** licences, at the
user's choice: MIT, or public domain via the Unlicense. Either way there is
nothing to reproduce in a binary — unlike the SDKs above, it adds **no**
obligation to a distributed `bibo.exe`. It is recorded here because knowing
where a file came from is worth more than the licence obliges.

Only its **JPEG** decoder is compiled in (`STBI_ONLY_JPEG`, `STBI_NO_STDIO` in
`viewer/src/jpeg.cxx`). `CAMERA`'s codec byte defines `1 = JPEG` and nothing
else, so the other eight decoders would be eight more parsers reachable from a
network payload in exchange for no feature.

## Source files under someone else's terms

**`firmware/lib/shared.hxx`** — BSD-3-Clause, copyright (c) 2026 Jiaming Meng. From
[manbox](https://github.com/exoad/manbox). Same author as this project, but it is
published under its own licence and carries its own notice, which must be
retained. Two typedefs (`Utf16`, `Utf32`) are a local addition, marked as such in
the file.

---

## Hardware, for completeness

Not licences, but the datasheets these were built against:
Tamiya TT-02 (kit 58631), Slamtec RPLIDAR C1, Raspberry Pi Pico 2 W (RP2350),
Orange Pi 4 Pro, Hobbywing QuicRun 10BL160 G2, Power HD 1501MG, Flysky FS-GT2.
See [docs/wiring.md](docs/wiring.md).
