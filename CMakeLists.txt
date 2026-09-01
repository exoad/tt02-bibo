# bibo - the root CMake project, for the IDE. Covers the HUB only: one configure
# has one toolchain, and hub/ (x64 MSVC) and firmware/ (arm-none-eabi, RP2350)
# cannot share it. Attach the firmware beside it for cross-project indexing:
#   File | Attach Project... -> firmware/CMakeLists.txt, preset
#   "Pico 2 W (RP2350, arm-none-eabi)"
# Without attaching, firmware/.clangd points the parser at
# firmware/build/compile_commands.json (written by firmware/build.bat); its
# -Ifirmware/lib is what makes "bibo.hxx" and the library's own "drivers/..."
# includes resolve.
#
# Neither replaces hub/build.bat and firmware/build.bat - those are still the
# authoritative builds for anything shipped or flashed.

cmake_minimum_required(VERSION 3.21)

project(bibo CXX)

# Must be at the ROOT, not only in hub/: a subdirectory's tests register with
# CTest only when the root enabled testing. Without this, `ctest` builds the
# test binaries happily and then reports "No tests were found".
enable_testing()

add_subdirectory(hub)
