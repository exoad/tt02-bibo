cmake_minimum_required(VERSION 3.13)

# Pico 2 W: RP2350 plus a CYW43439. The board matters here beyond pin naming -
# the user LED is on the wireless chip, so pico_cyw43_arch is required just to
# blink it. Overridable for the bench mule (e.g. -DPICO_BOARD=pico_w).
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2_w)
endif()

include(pico_sdk_import.cmake)

project(tt02_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_debug
    src/main.c
)

# pico_cyw43_arch_none: the wireless chip is brought up so its GPIO (and so the
# LED) works, but no networking stack is linked. AP mode + UDP arrive in phase 3;
# pulling lwip in now would cost flash and build time for nothing.
target_link_libraries(pico_debug
    pico_stdlib
    pico_cyw43_arch_none
    pico_unique_id
    hardware_pwm
    hardware_adc
)

# Debug console is USB CDC. UART stdio is off so GP0/GP1 stay free - they are
# the servo and ESC signal pins (docs/wiring.md).
pico_enable_stdio_usb(pico_debug 1)
pico_enable_stdio_uart(pico_debug 0)

# shared/ holds the project's C vocabulary (shared.h). One copy, used by the
# firmware and mirrored by shared/shared.hpp for the hub.
target_include_directories(pico_debug PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
)

pico_add_extra_outputs(pico_debug)

# ---------------------------------------------------------------------------
# sketch: the scratch program the hub's Code view edits and flashes.
#
# Deliberately linked against pico_stdlib ONLY. The point of this target is that
# a beginner's program builds in a couple of seconds and that a mistake in it
# cannot break the debug image sitting next to it. Adding cyw43 here would pull
# in the wireless stack, roughly triple the build, and buy nothing that a plain
# GPIO on a breadboard needs.
#
# USB stdio is on so printf() from a sketch lands in the hub's console. UART
# stdio stays off for the same reason as above: GP0/GP1 are the servo and ESC.
# ---------------------------------------------------------------------------
add_executable(sketch
    src/sketch.c
)

# hardware_pwm and hardware_adc are NOT in pico_stdlib, and pico2w.h wraps both.
# Linking them here rather than making the sketch ask keeps the promise that
# header makes: include it and everything in it works.
target_link_libraries(sketch
    pico_stdlib
    hardware_pwm
    hardware_adc
)

pico_enable_stdio_usb(sketch 1)
pico_enable_stdio_uart(sketch 0)

target_include_directories(sketch PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
)

pico_add_extra_outputs(sketch)
