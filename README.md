# bibo

A self-driving 1/10 RC car on a Tamiya TT-02. Drive a route once, then let it
drive itself.

```bat
:: vendor/ is gitignored, so fetch the SDK first
git clone https://github.com/Slamtec/rplidar_sdk.git vendor/rplidar_sdk

hub\build.bat && hub\build\bibo.exe
```

It finds the lidar and the Pico on its own. Firmware toolchain:
[firmware/README.md](firmware/README.md). Everything else — hardware, wiring,
architecture, the build log — is [docs/conventions.md](docs/conventions.md).

## Before you power the car

- **On a stand, wheels off the ground**, for the first run of any new code.
- **Common ground between the Pico and the ESC.** Its absence looks like a
  software bug and is not one.
- **Never connect the BEC 5 V to the Pico while USB is attached.**
- The Flysky radio is the kill switch, on its own band.

## Licence

**Not open source.** Copyright (c) 2026 Jiaming Meng. See [COPYRIGHT](COPYRIGHT).

A distributed binary must carry two notices: Fugue Icons (CC BY 3.0) and Slamtec
rplidar_driver (BSD-2-Clause). [THIRD_PARTY.md](THIRD_PARTY.md).
