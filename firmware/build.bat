@echo off
REM  Builds the firmware for one of the two boards.
REM
REM    firmware\build.bat                 pico2_w  -> build\pico_debug.uf2
REM    firmware\build.bat pico2           pico2    -> build-pico2\pico_debug.uf2
REM    firmware\build.bat clean           wipe, then build pico2_w
REM    firmware\build.bat clean pico2     wipe, then build pico2  (any order)
REM
REM  SEPARATE BUILD DIRECTORIES: changing PICO_BOARD invalidates most of a tree,
REM  so sharing one means a full rebuild (~190 TUs on the W) at every switch.
REM  build\ stays the W's, so existing paths and .clangd keep meaning what they did.
REM
REM  Toolchain traps on this machine:
REM   * arm-none-eabi-gcc is MSYS2's (mingw64), a native Windows binary. CMake and
REM     Ninja are Visual Studio's, NOT MSYS2's: usr/bin's cmake is Cygwin-style
REM     (POSIX paths, cannot drive a native ARM toolchain) and mingw64's dies with
REM     0xC0000135 (DLL not found) on this partially-updated install.
REM   * PATH is NOT globally modified: mingw64\bin ahead of msys64\usr\bin makes
REM     usr/bin binaries fail with 0xC0000139, the two runtimes shadowing.

setlocal EnableDelayedExpansion
set "HERE=%~dp0"
set "ROOT=%HERE%.."

REM ---- arguments, order-independent ----------------------------------------
REM Order matters nowhere: silently building the wrong board yields a working
REM image for the wrong chip, which flashes without complaint. TARGET builds one
REM image; without it every sketch in sketches/ is built alongside the car.
set "BOARD=pico2_w"
set "DOCLEAN="
set "TARGET="

for %%A in (%*) do (
    if /i "%%~A"=="clean" (
        set "DOCLEAN=1"
    ) else if /i "%%~A"=="pico2" (
        set "BOARD=pico2"
    ) else if /i "%%~A"=="pico2_w" (
        set "BOARD=pico2_w"
    ) else if /i "%%~A"=="pico_debug" (
        set "TARGET=pico_debug"
    ) else if exist "%HERE%sketches\%%~A.cxx" (
        REM Checked against the FILE, so a typo is caught here with a list of
        REM what exists rather than two minutes later as ninja "unknown target".
        set "TARGET=%%~A"
    ) else (
        echo [error] unknown argument "%%~A"
        echo         usage: build.bat [clean] [pico2^|pico2_w] [TARGET]
        echo.
        echo         TARGET is pico_debug, or the name of a sketch:
        for %%S in ("%HERE%sketches\*.cxx") do echo           %%~nS
        exit /b 1
    )
)

REM The W keeps the plain build\ directory; anything else gets a suffixed tree.
if /i "%BOARD%"=="pico2_w" (
    set "BUILD=%HERE%build"
) else (
    set "BUILD=%HERE%build-%BOARD%"
)

set "PICO_SDK_PATH=%ROOT%\vendor\pico-sdk"

call "%~dp0..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
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

REM PATH deliberately NOT touched: the SDK builds its host tools (pioasm,
REM picotool) with the first native g++ it finds - ucrt64's - and prepending
REM mingw64\bin shadows ucrt64's DLLs, so they die silently mid-build (pioasm
REM exits non-zero with no message, yet runs fine by hand). PICO_TOOLCHAIN_PATH
REM points the SDK at the cross-compiler instead.
set "PICO_TOOLCHAIN_PATH=C:/msys64/mingw64"

if defined DOCLEAN (
    echo [clean] removing %BUILD%
    if exist "%BUILD%" rmdir /s /q "%BUILD%"
)

echo [conf ] cmake configure for %BOARD%
REM picotool_DIR: Raspberry Pi's OFFICIAL prebuilt picotool, pinned at v2.3.0-1.
REM The one the SDK builds here (ucrt64 gcc) crashes with 0xC0000005 in every
REM subcommand that touches an ELF - `uf2 convert`, `coprodis` - and reports
REM "compiled without USB support". Downloaded once into vendor/ (gitignored):
REM   https://github.com/raspberrypi/pico-sdk-tools/releases  (v2.3.0-1)
set "PICOTOOL_DIR=%ROOT%\vendor\picotool-2.3.0\picotool"

if not exist "%PICOTOOL_DIR%\picotool.exe" (
    echo [error] picotool not found at %PICOTOOL_DIR%
    echo         Download picotool-2.3.0-x64-win.zip from
    echo         https://github.com/raspberrypi/pico-sdk-tools/releases/tag/v2.3.0-1
    echo         and extract it to vendor\picotool-2.3.0\
    exit /b 1
)

REM CMAKE_EXPORT_COMPILE_COMMANDS writes build\compile_commands.json, which
REM firmware/.clangd points editors at. Only CMakePresets.json set it, yet this
REM script is what runs and it recreates build\ on a clean - so the fallback for
REM editors WITHOUT the project loaded only worked once it was loaded.
"%CMAKE%" -S "%HERE%." -B "%BUILD%" -G Ninja ^
    -DPICO_BOARD=%BOARD% ^
    -DCMAKE_MAKE_PROGRAM="%NINJA%" ^
    -Dpicotool_DIR="%PICOTOOL_DIR%" ^
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ^
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
if errorlevel 1 (
    echo [error] cmake configure failed
    exit /b 1
)

if defined TARGET (
    echo [build] compiling %TARGET%
    "%CMAKE%" --build "%BUILD%" --target %TARGET%
) else (
    echo [build] compiling everything
    "%CMAKE%" --build "%BUILD%"
)
if errorlevel 1 (
    echo [error] build failed
    exit /b 1
)

REM Check the asked-for image by name: a build that "succeeded" while producing
REM no UF2 leaves the flash step downstream reaching for a stale file from a
REM previous build and reporting success having flashed the wrong program.
set "WANT=pico_debug"
if defined TARGET set "WANT=%TARGET%"

if not exist "%BUILD%\%WANT%.uf2" (
    echo [error] build reported success but %WANT%.uf2 is missing
    exit /b 1
)

echo.
echo [ok] %BUILD%\%WANT%.uf2   (%BOARD%)
if not defined TARGET (
    for %%S in ("%HERE%sketches\*.cxx") do (
        if exist "%BUILD%\%%~nS.uf2" echo [ok] %BUILD%\%%~nS.uf2   (%BOARD%)
    )
)
echo      Flash it with:  firmware\flash.bat "%BUILD%\%WANT%.uf2"
