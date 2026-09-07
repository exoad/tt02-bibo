# The companion board's build. Host C++ only - no Pico SDK, no cross-compiler -
# so it builds with the g++ on the Orange Pi or any Linux box the same way the
# .bat files build it with MSVC on the laptop. Two toolchains, one source; the
# tests are what hold the two to the same answer.
#
#   cmake -S firmware/pilot -B build-pilot && cmake --build build-pilot -j
#   ctest --test-dir build-pilot --output-on-failure
#
# With the lidar SDK (see PILOT_RPLIDAR_SDK below) the same configure also
# builds the program itself and the probe:
#
#   cmake -S firmware/pilot -B build-pilot -DPILOT_RPLIDAR_SDK=$HOME/rplidar_sdk
#   cmake --build build-pilot -j
#   build-pilot/pilot --dry --seconds 12          # watch it decide, car untouched
#   build-pilot/pilot --arm                       # drive, until Ctrl-C
cmake_minimum_required(VERSION 3.22)
project(bibo_pilot CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# -Wno-unused-function for the same reason build_pilot_test.bat passes /wd4505:
# firmware/lib's headers define their functions `static`, so a consumer using a
# subset of a header trips it on every function it does not call.
add_compile_options(-Wall -Wextra -Wno-unused-function)

set(SHARED ${CMAKE_CURRENT_SOURCE_DIR}/../../shared)
set(FWLIB  ${CMAKE_CURRENT_SOURCE_DIR}/../lib)

# pilotlib, not pilot: `pilot` is the program (app/main.cxx, below) and the
# binary a person runs, so the library it is built from takes the longer name.
# Everything else here - the tests, the probe - links this.
add_library(
    pilotlib STATIC src/proto.cxx src/link.cxx src/autonomy.cxx src/reactive.cxx src/lidar.cxx
)
target_include_directories(pilotlib PUBLIC ${SHARED} ${CMAKE_CURRENT_SOURCE_DIR}/src ${FWLIB})
find_package(Threads REQUIRED)
target_link_libraries(pilotlib PUBLIC Threads::Threads)

# The lidar. Slamtec's SDK is not a package anything can find - it is a
# checkout built in place with its own makefiles - so the path to that checkout
# is handed in, and everything below is derived from its fixed layout:
#
#   cmake -S firmware/pilot -B build-pilot -DPILOT_RPLIDAR_SDK=$HOME/rplidar_sdk
#
# Unset, src/lidar.cxx compiles its refusing half and lidar_probe is not built;
# the tests are the same either way, because none of them needs a device.
#
# A PATH cache entry, not an option(): option() declares a BOOL, so ccmake
# showed this as a checkbox and ticking it set PILOT_RPLIDAR_SDK to ON - a
# "path" the checks below then failed on, with a message about a missing header
# in ON/sdk/include. Empty is still "no SDK", so the if() below is unchanged.
set(PILOT_RPLIDAR_SDK "" CACHE PATH "path to a built rplidar_sdk checkout, Linux only")
if(PILOT_RPLIDAR_SDK)
    set(RPLIDAR_INC ${PILOT_RPLIDAR_SDK}/sdk/include)
    set(RPLIDAR_LIB ${PILOT_RPLIDAR_SDK}/output/Linux/Release/libsl_lidar_sdk.a)

    # Checked at configure time, by name. A path with a typo in it would
    # otherwise define PILOT_HAVE_RPLIDAR, compile the real half, and fail at
    # link with a message about a symbol rather than about the path.
    if(NOT EXISTS ${RPLIDAR_INC}/sl_lidar_driver.h)
        message(FATAL_ERROR "no sl_lidar_driver.h in ${RPLIDAR_INC} - check PILOT_RPLIDAR_SDK")
    endif()
    if(NOT EXISTS ${RPLIDAR_LIB})
        message(FATAL_ERROR "${RPLIDAR_LIB} is missing - run make in the rplidar_sdk checkout")
    endif()

    # PRIVATE include, PUBLIC define. lidar.hxx names no SDK type, so nothing
    # linking pilotlib needs the SDK's headers; the define is public so a
    # consumer can #if on whether the lidar half of the library is real. SYSTEM
    # because the SDK's headers are not held to this build's -Wall -Wextra.
    target_compile_definitions(pilotlib PUBLIC PILOT_HAVE_RPLIDAR)
    target_include_directories(pilotlib SYSTEM PRIVATE ${RPLIDAR_INC})
    target_link_libraries(pilotlib PUBLIC ${RPLIDAR_LIB} Threads::Threads)

    add_executable(lidar_probe tools/lidar_probe.cxx)
    target_link_libraries(lidar_probe PRIVATE pilotlib)

    # The program. Only with the SDK, on purpose: without a lidar half that can
    # see, app/main.cxx would open the car and send it a stop once a tick, which
    # is a test of the link wearing the program's name. The source itself is
    # portable - it names nothing from the SDK - so the day there is a second
    # sensor this condition is the only thing that moves.
    add_executable(pilot app/main.cxx)
    target_link_libraries(pilot PRIVATE pilotlib)
endif()

enable_testing()
foreach(t proto pilot reactive)
    add_executable(test_${t} tests/test_${t}.cxx)
    target_link_libraries(test_${t} PRIVATE pilotlib)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()
