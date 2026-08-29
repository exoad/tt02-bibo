# bibo - the root CMake project, for the IDE.
#
# ---------------------------------------------------------------------------
# WHY THIS IS ONLY THE HUB.
#
# There are two codebases here and they cannot share one CMake configure:
#
#   hub/       x64 MSVC, Win32 + DX11, runs on this machine
#   firmware/  arm-none-eabi, RP2350, runs on the car
#
# A single configure has ONE toolchain, so the cross-compiled half cannot be a
# subdirectory of the native half. This root therefore covers the hub, and the
# firmware is a project you attach beside it:
#
#   File | Attach Project...  ->  firmware/CMakeLists.txt
#   then pick the "Pico 2 W (RP2350, arm-none-eabi)" preset
#
# Both then index at once, and Ctrl-clicking gpioOpen() from a sketch lands in
# firmware/lib/hal.h the way it should.
#
# WITHOUT attaching, firmware/.clangd is the fallback: it points the parser at
# firmware/build/compile_commands.json, which firmware/build.bat now writes.
# That file carries -Ifirmware/lib, which is what makes "bibo.hxx" and the
# library's own "drivers/..." and "chassis/..." includes resolve.
#
# Neither of these replaces the .bat scripts. hub/build.bat and
# firmware/build.bat are still the authoritative builds and still the thing to
# run when something is being shipped or flashed.
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.21)

project(bibo CXX)

# Must be here and not only in hub/: CTest writes its test list per directory,
# and a subdirectory's tests are only registered with the root when the ROOT has
# enabled testing. Without this, `ctest` from the build directory builds the test
# binaries happily and then reports "No tests were found".
enable_testing()

add_subdirectory(hub)
