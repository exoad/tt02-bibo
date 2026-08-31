# bibo

A self-driving 1/10 RC car on a Tamiya TT-02. Drive a route once, then let it
drive itself.

```bat
:: vendor/ is gitignored, so fetch the SDK first
git clone https://github.com/Slamtec/rplidar_sdk.git vendor/rplidar_sdk

hub\build.bat && hub\build\bibo.exe
```

Wiring and safety: [docs/wiring.md](docs/wiring.md). Firmware toolchain:
[firmware/README.md](firmware/README.md). Everything else:
[docs/conventions.md](docs/conventions.md).

**Read the wiring notes before powering the car.**

**Not open source.** See [COPYRIGHT](COPYRIGHT) and
[THIRD_PARTY.md](THIRD_PARTY.md).
