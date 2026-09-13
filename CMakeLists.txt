# Exists so CLion can open the repository as a CMake project; it builds nothing.
# firmware/ cannot be a subdirectory: it cross-compiles for arm-none-eabi, and one
# configure has one toolchain. Attach it beside this one with File | Attach
# Project... -> firmware/CMakeLists.txt, preset "Pico 2 W (RP2350, arm-none-eabi)".

cmake_minimum_required(VERSION 3.21)
project(bibo CXX)
