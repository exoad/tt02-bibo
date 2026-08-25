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
)

# Debug console is USB CDC. UART stdio is off so GP0/GP1 stay free - they are
# the servo and ESC signal pins (docs/wiring.md).
pico_enable_stdio_usb(pico_debug 1)
pico_enable_stdio_uart(pico_debug 0)

pico_add_extra_outputs(pico_debug)
