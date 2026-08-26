@echo off
REM ===========================================================================
REM  Builds the Pico 2 W firmware. Output: build\pico_debug.uf2
REM
REM  Toolchain notes, learned the hard way on this machine:
REM
REM   * arm-none-eabi-gcc comes from MSYS2 (mingw64), installed once. It is a
REM     native Windows binary.
REM   * CMake and Ninja come from Visual Studio, NOT from MSYS2. MSYS2 ships two
REM     incompatible cmakes and both failed here: usr/bin/cmake is a Cygwin-style
REM     build that uses POSIX paths and cannot drive a native ARM toolchain, and
REM     mingw64/cmake died with 0xC0000135 (DLL not found) because this MSYS2
REM     install is only partially updated. VS's cmake is native, self-contained,
REM     and already present.
REM   * PATH is NOT globally modified. In particular, putting mingw64\bin ahead
REM     of msys64\usr\bin makes usr/bin binaries fail with 0xC0000139
REM     (entrypoint not found), because the two runtimes shadow each other.
REM     Only the ARM toolchain directory is added, and only for this build.
REM
REM  Usage:  firmware\build.bat  [clean]
REM ===========================================================================

setlocal EnableDelayedExpansion
set "HERE=%~dp0"
set "ROOT=%HERE%.."
set "BUILD=%HERE%build"

set "PICO_SDK_PATH=%ROOT%\vendor\pico-sdk"

set "VSROOT=C:\Program Files\Microsoft Visual Studio\2022\Community"
set "CMAKE=%VSROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NINJA=%VSROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
set "ARMBIN=C:\msys64\mingw64\bin"

if not exist "%PICO_SDK_PATH%\pico_sdk_init.cmake" (
    echo [error] pico-sdk not found at %PICO_SDK_PATH%
    echo         git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git vendor/pico-sdk
    echo         cd vendor/pico-sdk ^&^& git submodule update --init --depth 1 lib/tinyusb lib/cyw43-driver lib/lwip
    exit /b 1
)
if not exist "%ARMBIN%\arm-none-eabi-gcc.exe" (
    echo [error] arm-none-eabi-gcc not found at %ARMBIN%
    echo         See firmware\README.md for the one-line install.
    exit /b 1
)
if not exist "%CMAKE%" (
    echo [error] Visual Studio's cmake not found at:
    echo         %CMAKE%
    exit /b 1
)

REM PATH is deliberately NOT touched. The SDK builds its own host tools (pioasm,
REM picotool) with the first native g++ it finds, which on this machine is
REM ucrt64's. Prepending mingw64\bin then shadows ucrt64's runtime DLLs and those
REM tools die silently mid-build - pioasm exits non-zero with no message, while
REM running fine by hand. PICO_TOOLCHAIN_PATH tells the SDK where the ARM
REM cross-compiler is without putting its directory in front of anything.
set "PICO_TOOLCHAIN_PATH=C:/msys64/mingw64"

if /i "%~1"=="clean" (
    echo [clean] removing %BUILD%
    if exist "%BUILD%" rmdir /s /q "%BUILD%"
)

echo [conf ] cmake configure
REM picotool_DIR: use Raspberry Pi's OFFICIAL prebuilt picotool rather than the
REM one the SDK builds from source here. The locally-built one crashes with an
REM access violation (0xC0000005) in every subcommand that touches an ELF -
REM `uf2 convert` and `coprodis` both - and reports "compiled without USB
REM support". It is built by ucrt64 gcc; the official binary is GNU-16.2.0 with
REM USB support and works. Downloaded once into vendor/ (gitignored):
REM   https://github.com/raspberrypi/pico-sdk-tools/releases  (v2.3.0-1)
set "PICOTOOL_DIR=%ROOT%\vendor\picotool-2.3.0\picotool"

if not exist "%PICOTOOL_DIR%\picotool.exe" (
    echo [error] picotool not found at %PICOTOOL_DIR%
    echo         Download picotool-2.3.0-x64-win.zip from
    echo         https://github.com/raspberrypi/pico-sdk-tools/releases/tag/v2.3.0-1
    echo         and extract it to vendor\picotool-2.3.0\
    exit /b 1
)

"%CMAKE%" -S "%HERE%." -B "%BUILD%" -G Ninja ^
    -DCMAKE_MAKE_PROGRAM="%NINJA%" ^
    -Dpicotool_DIR="%PICOTOOL_DIR%" ^
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
if errorlevel 1 (
    echo [error] cmake configure failed
    exit /b 1
)

echo [build] compiling
"%CMAKE%" --build "%BUILD%"
if errorlevel 1 (
    echo [error] build failed
    exit /b 1
)

if not exist "%BUILD%\pico_debug.uf2" (
    echo [error] build reported success but pico_debug.uf2 is missing
    exit /b 1
)

echo.
echo [ok] %BUILD%\pico_debug.uf2
if exist "%BUILD%\sketch.uf2" echo [ok] %BUILD%\sketch.uf2
echo      Flash it with:  firmware\flash.bat
