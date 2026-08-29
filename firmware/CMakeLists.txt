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

# C is still listed, and has to be: the SDK, TinyUSB, lwIP and the cyw43 driver
# are all C and are compiled as part of these targets. What changed is that
# everything WE write is C++ now - see the note on the library below.
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# Whether THIS board has the wireless chip, asked of the SDK rather than
# pattern-matched off the board name. The board header carries
# `pico_board_cmake_set(PICO_CYW43_SUPPORTED, 1)` and the SDK lifts that into a
# CMake variable while loading it - so pico2_w answers yes, pico2 answers
# nothing, and a board nobody has thought about yet answers correctly on its
# own. Matching on a trailing "_w" would have been shorter and would have been
# a rule this project invented about somebody else's naming.
#
# Two answers, because the two programs want different things from the chip.
#
#   TT02_LED_LIBS  pico_cyw43_arch_none - the chip is brought up so its GPIO
#                  (and so the LED) works, and no networking is linked. This is
#                  what the sketch target gets: a first-hour program needs the
#                  LED and does not need lwIP tripling its build.
#
#   TT02_NET_LIBS  pico_cyw43_arch_lwip_poll - the same chip plus the IP stack,
#                  polled from the main loop rather than run off an interrupt or
#                  a second core. Poll mode is the right one here: this firmware
#                  is already one loop where one thing happens at a time, so
#                  there is nothing to make thread-safe and no callback can
#                  arrive in the middle of a servo update.
#
# TT02_WIRELESS is what net.hxx and main.cxx test. It comes from the SAME board
# question rather than from a hand-maintained list, so the plain Pico 2 compiles
# the stub half of net.hxx without anybody having to remember to say so.
if(PICO_CYW43_SUPPORTED)
    set(TT02_LED_LIBS pico_cyw43_arch_none)
    set(TT02_NET_LIBS pico_cyw43_arch_lwip_poll)
    set(TT02_NET_DEFS TT02_WIRELESS=1)
    message(STATUS "tt02: ${PICO_BOARD} - CYW43439 present, wireless link built")
else()
    set(TT02_LED_LIBS "")
    set(TT02_NET_LIBS "")
    set(TT02_NET_DEFS "")
    message(STATUS "tt02: ${PICO_BOARD} - LED on a plain GPIO, no cyw43")
endif()

# ---------------------------------------------------------------------------
# The library is C++.
#
# It was C, and the extension is what says so: .hxx and .cxx, per the manbox
# guide. That is not cosmetic - it is the reason this conversion happened. C has
# no namespaces, so every symbol carried its module in its NAME (gpioWrite,
# driveStop, lightsTick), which is the C answer to the same problem and reads as
# a prefix somebody remembered rather than a boundary the compiler knows about.
#
# Headers are still listed as SOURCES, but no longer to tell an IDE what
# language they are - .hxx answers that on its own. They are listed so the files
# show up as belonging to this target, and so a header-only library is visible
# in a project tree at all.
# ---------------------------------------------------------------------------
set(TT02_LIB
    lib/tt02.hxx
    lib/types.hxx
    lib/text.hxx
    lib/hal.hxx
    lib/gfx.hxx
    lib/status.hxx
    lib/lights.hxx
    lib/cue.hxx
    lib/net.hxx
    lib/drivers/display.hxx
    lib/drivers/range.hxx
    lib/drivers/storage.hxx
    lib/chassis/chassis.hxx
    lib/chassis/cal.hxx
)

add_executable(pico_debug
    app/main.cxx
    ${TT02_LIB}
)

# The debug image is the one that carries the wireless command link, so it gets
# TT02_NET_LIBS - the cyw43 arch WITH lwIP - rather than the LED-only variant.
# On the car's plain Pico 2 both are empty and net.hxx compiles to its stubs.
target_link_libraries(pico_debug
    pico_stdlib
    ${TT02_NET_LIBS}
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

target_compile_definitions(pico_debug PRIVATE ${TT02_NET_DEFS})

# firmware/lib is the whole library: an app's "tt02.hxx" and the library's own
# "drivers/..." and "chassis/..." includes all resolve through this one entry.
# shared/ is no longer on the path - the vocabulary moved to lib/types.hxx,
# because the firmware was its only caller.
#
# The second entry is not optional: lwIP is configured by a header called
# exactly lwipopts.h found on the include path, and it lives at the firmware
# root rather than in lib/ because it is a contract with somebody else's code
# rather than part of this library.
target_include_directories(pico_debug PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}"
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
    scratch/sketch.cxx
    ${TT02_LIB}
)

# pico_stdlib does not carry PWM, ADC or the watchdog, and on the Pico 2 W the
# LED hangs off the CYW43439 rather than a GPIO - so there the wireless arch has
# to be linked just to blink it. Linking all of them here rather than making
# each sketch ask keeps the promise tt02.hxx makes: include it and everything in
# it works.
#
# COST, stated honestly: on the W, cyw43_arch roughly doubles a CLEAN build of
# this target (86 -> ~190 translation units). Incremental builds are unaffected
# - editing sketch.cxx still recompiles one file - and a first-hour program that
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
