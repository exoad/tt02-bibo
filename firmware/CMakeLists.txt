cmake_minimum_required(VERSION 3.13)

# Two boards, one firmware.
#   pico2_w  breadboard workhorse. RP2350 + CYW43439; the user LED hangs off that
#            chip, so pico_cyw43_arch must be linked just to blink it.
#   pico2    the board in the car. Same RP2350 and pinout, no wireless chip: the
#            LED is a plain GPIO and cyw43 must NOT be linked.
# Default is pico2_w (the board on the desk). Car image: firmware\build.bat pico2
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2_w)
endif()

include(pico_sdk_import.cmake)

project(bibo_firmware C CXX ASM)

# C has to stay listed: the SDK, TinyUSB, lwIP and the cyw43 driver are C and
# compile as part of these targets. Everything we write is C++.
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# Ask the SDK whether this board has the wireless chip, rather than matching a
# trailing "_w" on the board name: the board header sets PICO_CYW43_SUPPORTED and
# the SDK lifts it into CMake, so an unfamiliar board still answers correctly.
#
#   BIBO_LED_LIBS  pico_cyw43_arch_none - chip up so its GPIO (the LED) works, no
#                  networking. Sketches get this; they don't need lwIP.
#   BIBO_NET_LIBS  ..._lwip_poll - chip plus IP stack, polled from the main loop.
#                  Poll mode fits: this firmware is one loop, so nothing needs to
#                  be thread-safe and no callback lands mid-servo-update.
#
# BIBO_WIRELESS (tested by net.hxx and main.cxx) comes from the same question, so
# the plain Pico 2 gets net.hxx's stub half with nobody having to remember.
if(PICO_CYW43_SUPPORTED)
    set(BIBO_LED_LIBS pico_cyw43_arch_none)
    set(BIBO_NET_LIBS pico_cyw43_arch_lwip_poll)
    set(BIBO_NET_DEFS BIBO_WIRELESS=1)
    message(STATUS "bibo: ${PICO_BOARD} - CYW43439 present, wireless link built")
else()
    set(BIBO_LED_LIBS "")
    set(BIBO_NET_LIBS "")
    set(BIBO_NET_DEFS "")
    message(STATUS "bibo: ${PICO_BOARD} - LED on a plain GPIO, no cyw43")
endif()

# The library, C++ and header-only (.hxx/.cxx per the manbox guide). Headers are
# listed as SOURCES so a header-only library is visible in the project tree at
# all and the files read as belonging to this target.
set(BIBO_LIB
    lib/bibo.hxx
    lib/shared.hxx
    lib/pins.hxx
    lib/sfx.hxx
    lib/sound.hxx
    lib/geom.hxx
    lib/kinematics.hxx
    lib/pursuit.hxx
    lib/plan.hxx
    lib/control.hxx
    lib/text.hxx
    lib/hal.hxx
    lib/gfx.hxx
    lib/status.hxx
    lib/lights.hxx
    lib/cue.hxx
    lib/net.hxx
    lib/drivers/dfplayer.hxx
    lib/drivers/dfplayer_proto.hxx
    lib/drivers/display.hxx
    lib/drivers/range.hxx
    lib/drivers/storage.hxx
    lib/chassis/chassis.hxx
    lib/chassis/cal.hxx
    lib/chassis/odom.hxx
)

# app: the car. Globbed rather than listed because app/ IS the product - every
# .cxx in it is part of the car by definition, so an explicit list buys nothing
# and costs a mystery the day somebody adds sound.cxx and it is silently not
# built. CONFIGURE_DEPENDS re-runs the glob when the directory changes, so a new
# file builds without a manual reconfigure.
file(GLOB BIBO_APP CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/app/*.cxx")

if(NOT BIBO_APP)
    message(FATAL_ERROR "bibo: firmware/app is empty - nothing to build")
endif()

add_executable(pico_debug
    ${BIBO_APP}
    ${BIBO_LIB}
)

# The debug image carries the wireless command link, so it takes BIBO_NET_LIBS
# (cyw43 arch WITH lwIP), not the LED-only variant. On the car's plain Pico 2
# both are empty and net.hxx compiles to its stubs.
target_link_libraries(pico_debug
    pico_stdlib
    ${BIBO_NET_LIBS}
    pico_unique_id
    hardware_pwm
    hardware_adc
    hardware_watchdog
    hardware_spi
    hardware_i2c
)

# Console is USB CDC. UART stdio off so GP0/GP1 stay free - they are the servo
# and ESC signal pins (docs/wiring.md).
pico_enable_stdio_usb(pico_debug 1)
pico_enable_stdio_uart(pico_debug 0)

target_compile_definitions(pico_debug PRIVATE ${BIBO_NET_DEFS})

# lib/ is the whole library: an app's "bibo.hxx" and the library's own
# "drivers/..." and "chassis/..." includes all resolve through that one entry.
# The second entry is not optional - lwIP is configured by a header named exactly
# lwipopts.h found on the include path, and it sits at the firmware root because
# it is a contract with somebody else's code, not part of this library.
target_include_directories(pico_debug PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

pico_add_extra_outputs(pico_debug)

# sketches: ONE TARGET PER FILE, and that is the point of this section.
#
# There used to be a single `sketch` target built from a scratch/sketch.cxx SLOT.
# The hub's Code view copied whatever you were editing into it before every
# build, so pressing Build & Flash silently destroyed the slot's contents - it
# held a working VL53L1X range view for weeks. Now every .cxx in sketches/ is its
# own executable named after the file (range-view.cxx -> range-view.uf2):
# nothing is overwritten and a sketch is still findable a month later.
#
#   app/        the car. Multi-file, one image, the finished thing.
#   sketches/   one file, one image, one question.
#
# Linked against pico_stdlib and the hardware units ONLY - no lwIP - so a sketch
# builds in seconds and a mistake in one cannot break the image next to it. USB
# stdio on for printf(); UART off because GP0/GP1 are the servo and the ESC.
file(GLOB BIBO_SKETCHES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/sketches/*.cxx")

foreach(SKETCH_SRC ${BIBO_SKETCHES})
    get_filename_component(SKETCH_NAME "${SKETCH_SRC}" NAME_WE)

    # A sketch named pico_debug.cxx would collide with the car's target, and
    # CMake's own error for that is opaque. Name the file and the fix.
    if(TARGET ${SKETCH_NAME})
        message(FATAL_ERROR
            "bibo: sketches/${SKETCH_NAME}.cxx collides with an existing "
            "target named ${SKETCH_NAME} - rename the file")
    endif()

    add_executable(${SKETCH_NAME}
        "${SKETCH_SRC}"
        ${BIBO_LIB}
    )

# pico_stdlib carries neither PWM, ADC nor the watchdog, and on the W the LED
# needs the cyw43 arch. Linking all of them here keeps bibo.hxx's promise:
# include it and everything in it works. Cost: on the W, cyw43_arch roughly
# doubles a CLEAN build (86 -> ~190 TUs); incremental builds are unaffected and
# the car's pico2 build does not pay it (BIBO_LED_LIBS is empty there).
    target_link_libraries(${SKETCH_NAME}
        pico_stdlib
        ${BIBO_LED_LIBS}
        hardware_pwm
        hardware_adc
        hardware_watchdog
        hardware_spi
        hardware_i2c
    )

    pico_enable_stdio_usb(${SKETCH_NAME} 1)
    pico_enable_stdio_uart(${SKETCH_NAME} 0)

    target_include_directories(${SKETCH_NAME} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    )

    pico_add_extra_outputs(${SKETCH_NAME})
endforeach()

list(LENGTH BIBO_SKETCHES BIBO_SKETCH_COUNT)
message(STATUS "bibo: ${BIBO_SKETCH_COUNT} sketch target(s) in sketches/")
