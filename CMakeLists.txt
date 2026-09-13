# bibo - the root CMake project, for the IDE. It builds nothing.
#
# CLion opens this directory as a CMake project, so one has to exist here. The
# firmware cannot be a subdirectory of it: firmware/ cross-compiles for
# arm-none-eabi, and one configure has exactly one toolchain. Attach it beside
# this one instead - File | Attach Project... -> firmware/CMakeLists.txt, preset
# "Pico 2 W (RP2350, arm-none-eabi)". See docs/hardware.md ("In CLion").

cmake_minimum_required(VERSION 3.21)

project(bibo CXX)
