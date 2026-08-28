cmake_minimum_required(VERSION 3.13)

# Two boards, one firmware.
#
#   pico2_w  the breadboard workhorse. RP2350 plus a CYW43439, and the user LED
#            hangs off that chip - so pico_cyw43_arch has to be linked just to
#            blink it.
#   pico2    the board that goes IN THE CAR. Same RP2350, same pinout for
#            everything this project uses, but no wireless chip: the LED is a
#            plain GPIO and cyw43 must NOT be linked.
#
# The default stays pico2_w because that is the board on the desk and the one
# every existing script, path and note already means. Build the car's image
# with:  firmware\build.bat pico2
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2_w)
endif()

include(pico_sdk_import.cmake)

project(tt02_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Whether THIS board has the wireless chip, asked of the SDK rather than
# pattern-matched off the board name. The board header carries
# `pico_board_cmake_set(PICO_CYW43_SUPPORTED, 1)` and the SDK lifts that into a
# CMake variable while loading it - so pico2_w answers yes, pico2 answers
# nothing, and a board nobody has thought about yet answers correctly on its
# own. Matching on a trailing "_w" would have been shorter and would have been
# a rule this project invented about somebody else's naming.
if(PICO_CYW43_SUPPORTED)
    set(TT02_LED_LIBS pico_cyw43_arch_none)
    message(STATUS "tt02: ${PICO_BOARD} - LED via CYW43439, linking cyw43_arch")
else()
    set(TT02_LED_LIBS "")
    message(STATUS "tt02: ${PICO_BOARD} - LED on a plain GPIO, no cyw43")
endif()

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

# TT02_LED_LIBS is pico_cyw43_arch_none on the W and empty on the plain Pico 2 -
# see the board test above. _none is the no-networking variant: the chip is
# brought up so its GPIO (and so the LED) works, and no lwIP is linked. AP mode
# + UDP arrive in phase 3; pulling lwip in now would cost flash and build time
# for nothing.
target_link_libraries(pico_debug
    pico_stdlib
    ${TT02_LED_LIBS}
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

# pico_stdlib does not carry PWM, ADC or the watchdog, and on the Pico 2 W the
# LED hangs off the CYW43439 rather than a GPIO - so there the wireless arch has
# to be linked just to blink it. Linking all of them here rather than making
# each sketch ask keeps the promise tt02.h makes: include it and everything in
# it works.
#
# COST, stated honestly: on the W, cyw43_arch roughly doubles a CLEAN build of
# this target (86 -> ~190 translation units). Incremental builds are unaffected
# - editing sketch.c still recompiles one file - and a first-hour program that
# cannot blink the LED everybody can see is the worse trade. The car's pico2
# build does not pay it at all: TT02_LED_LIBS is empty there.
target_link_libraries(sketch
    pico_stdlib
    ${TT02_LED_LIBS}
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
