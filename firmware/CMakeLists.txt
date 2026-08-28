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

# Headers are listed as SOURCES on purpose. CMake does not need them - it finds
# them through the include path - but an IDE decides what LANGUAGE a .h is by
# which target owns it, and an unowned .h is assumed to be C++. That is why
# hal.h showed "C-style cast is used instead of a C++ cast" on every (UInt8) in
# a file that cannot contain a C++ cast. Listing them says plainly: these are C,
# and they belong to this program.
set(TT02_LIB
    lib/tt02.h
    lib/types.h
    lib/text.h
    lib/hal.h
    lib/gfx.h
    lib/status.h
    lib/drivers/display.h
    lib/drivers/range.h
    lib/drivers/storage.h
    lib/chassis/chassis.h
    lib/chassis/cal.h
)

add_executable(pico_debug
    app/main.c
    ${TT02_LIB}
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
    hardware_watchdog
    hardware_spi
    hardware_i2c
)

# Debug console is USB CDC. UART stdio is off so GP0/GP1 stay free - they are
# the servo and ESC signal pins (docs/wiring.md).
pico_enable_stdio_usb(pico_debug 1)
pico_enable_stdio_uart(pico_debug 0)

# firmware/lib is the whole library: an app's "tt02.h" and the library's own
# "drivers/..." and "chassis/..." includes all resolve through this one entry.
# shared/ is no longer on the path - the C vocabulary moved to lib/types.h,
# because the firmware was its only caller.
target_include_directories(pico_debug PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
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
    scratch/sketch.c
    ${TT02_LIB}
)

# pico_stdlib does not carry PWM, ADC or the watchdog, and the Pico 2 W's own
# LED hangs off the CYW43439 rather than a GPIO - so the wireless arch has to be
# linked just to blink it. Linking all of them here rather than making each
# sketch ask keeps the promise tt02.h makes: include it and everything in it
# works.
#
# COST, stated honestly: cyw43_arch roughly doubles a CLEAN build of this target
# (86 -> ~190 translation units). Incremental builds are unaffected - editing
# sketch.c still recompiles one file - and a first-hour program that cannot
# blink the LED everybody can see is the worse trade.
#
# _none is the no-networking variant: the chip is brought up so its GPIO works,
# and no lwIP is linked. AP mode arrives in phase 3.
target_link_libraries(sketch
    pico_stdlib
    pico_cyw43_arch_none
    hardware_pwm
    hardware_adc
    hardware_watchdog
    hardware_spi
    hardware_i2c
)

pico_enable_stdio_usb(sketch 1)
pico_enable_stdio_uart(sketch 0)

target_include_directories(sketch PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)

pico_add_extra_outputs(sketch)
