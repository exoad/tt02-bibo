cmake_minimum_required(VERSION 3.13)

# Two boards, one firmware; pico2_w is the default.
#   pico2_w  the desk board. RP2350 + CYW43439; the user LED hangs off that
#            chip, so pico_cyw43_arch_none is linked just to light it.
#   pico2    the board in the car. Same RP2350 and pinout, no wireless chip:
#            the LED is a plain GPIO and cyw43 must NOT be linked.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2_w)
endif()

include(pico_sdk_import.cmake)

project(bibo_firmware C CXX ASM)

# C stays listed: the SDK, TinyUSB and the cyw43 driver are C and compile as
# part of this target.
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# The board header, not the board's name, says whether the wireless chip is
# there: it sets PICO_CYW43_SUPPORTED and the SDK lifts it into CMake.
if(PICO_CYW43_SUPPORTED)
    set(BIBO_LED_LIBS pico_cyw43_arch_none)
    message(STATUS "bibo: ${PICO_BOARD} - LED on the CYW43439")
else()
    set(BIBO_LED_LIBS "")
    message(STATUS "bibo: ${PICO_BOARD} - LED on a plain GPIO, no cyw43")
endif()

# Header-only; listed so the headers show as part of the target.
set(BIBO_LIB
    lib/bibo.hxx
    lib/shared.hxx
    lib/hal.hxx
    lib/text.hxx
    lib/pins.hxx
    lib/status.hxx
    lib/chassis/cal.hxx
    lib/chassis/chassis.hxx
)

# app/ is the car: every .cxx in it is built, and CONFIGURE_DEPENDS picks up a
# new file without a manual reconfigure.
file(GLOB BIBO_APP CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/app/*.cxx")
if(NOT BIBO_APP)
    message(FATAL_ERROR "bibo: firmware/app is empty - nothing to build")
endif()

add_executable(pico_debug
    ${BIBO_APP}
    ${BIBO_LIB}
)

target_link_libraries(pico_debug
    pico_stdlib
    ${BIBO_LED_LIBS}
    pico_unique_id
    hardware_pwm
)

# Console on USB CDC. UART stdio off, so GP0/GP1 stay the servo and the ESC.
pico_enable_stdio_usb(pico_debug 1)
pico_enable_stdio_uart(pico_debug 0)

# lib/ resolves app/'s "bibo.hxx" and the library's own "chassis/..." includes.
target_include_directories(pico_debug PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)

pico_add_extra_outputs(pico_debug)

# encoder/ is the second Pico, the hall-sensor encoder node: the same SDK and
# board target, its own image. Built beside the car's for whichever board is
# configured, so `build.bat pico2` yields both.
file(GLOB BIBO_ENCODER CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/encoder/*.cxx")
add_executable(pico_encoder
    ${BIBO_ENCODER}
    lib/shared.hxx
    lib/hal.hxx
    lib/hall.hxx
)
target_link_libraries(pico_encoder
    pico_stdlib
    ${BIBO_LED_LIBS}
    pico_unique_id
    hardware_pwm
    hardware_gpio
)
pico_enable_stdio_usb(pico_encoder 1)
pico_enable_stdio_uart(pico_encoder 0)
target_include_directories(pico_encoder PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib")
pico_add_extra_outputs(pico_encoder)
