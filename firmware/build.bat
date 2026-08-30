@echo off
REM ===========================================================================
REM  Builds the firmware for one of the two boards.
REM
REM    firmware\build.bat                 pico2_w  -> build\pico_debug.uf2
REM    firmware\build.bat pico2           pico2    -> build-pico2\pico_debug.uf2
REM    firmware\build.bat clean           wipe, then build pico2_w
REM    firmware\build.bat clean pico2     wipe, then build pico2  (any order)
REM
REM  SEPARATE BUILD DIRECTORIES, on purpose. Changing PICO_BOARD in one tree
REM  invalidates most of it, so sharing a directory would mean a full rebuild -
REM  and on the W that is the cyw43 stack, ~190 translation units - every single
REM  time you switched boards. Two trees cost disk, which is free, instead of
REM  minutes, which are not. build\ stays the W's so every existing path,
REM  note, .clangd and compile_commands.json keeps meaning what it meant.
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
REM ===========================================================================

setlocal EnableDelayedExpansion
set "HERE=%~dp0"
set "ROOT=%HERE%.."

REM ---- arguments, order-independent ----------------------------------------
REM Two flags in any order, because "clean pico2" and "pico2 clean" are the same
REM intent and having one of them silently build the wrong board would be a
REM genuinely expensive mistake - it produces a working image for the wrong
REM chip, which flashes without complaint.
REM A third argument, added when sketches stopped being one overwritten slot and
REM became one target per file: TARGET, which builds just that image. Without it
REM every sketch in sketches/ is built along with the car, and that grows with
REM the folder - which is the wrong thing to charge someone for pressing Build
REM on a one-file experiment.
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
        REM Checked against the FILE rather than taken on trust, so a typo is
        REM caught here with a list of what exists instead of surfacing two
        REM minutes later as ninja saying "unknown target".
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

REM The W keeps the plain build\ directory it has always had; anything else gets
REM its own suffixed tree.
if /i "%BOARD%"=="pico2_w" (
    set "BUILD=%HERE%build"
) else (
    set "BUILD=%HERE%build-%BOARD%"
)

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

if defined DOCLEAN (
    echo [clean] removing %BUILD%
    if exist "%BUILD%" rmdir /s /q "%BUILD%"
)

echo [conf ] cmake configure for %BOARD%
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

REM CMAKE_EXPORT_COMPILE_COMMANDS writes build\compile_commands.json, which is
REM what firmware/.clangd points every editor at. CMakePresets.json has always
REM set it and this script never did - and since this script is the one that
REM actually runs, and it deletes and recreates build\ on a clean, the database
REM did not exist unless somebody had separately configured through the IDE.
REM So the fallback that exists precisely for editors WITHOUT the project loaded
REM only worked once the project was loaded. It is set in both places now.
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

REM The image the caller actually asked for, checked by name. A build that
REM "succeeded" while producing no UF2 is the failure worth catching here: the
REM flash step downstream would otherwise reach for a stale file from a previous
REM build and report success having flashed the wrong program.
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
