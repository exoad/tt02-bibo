# The companion board's build: host C++ only, no Pico SDK, so g++ on the Orange Pi or any
# Linux box builds it. tools\test.bat builds the suites with MSVC, and the tests hold the two
# toolchains to the same answer.
#
#   cmake -S firmware/pilot -B build-pilot && cmake --build build-pilot -j
#   ctest --test-dir build-pilot --output-on-failure
#
# Without the lidar SDK the pilot and the car programs build, but their lidar refuses at run
# time. With it they can see, and lidar_probe is built:
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

# -Wno-unused-function: a header that defines its functions `static`, as firmware/lib's do,
# trips it on every function a consumer does not call.
add_compile_options(-Wall -Wextra -Wno-unused-function)

set(FWLIB ${CMAKE_CURRENT_SOURCE_DIR}/../lib)

# pilotlib, because `pilot` is the program a person runs.
add_library(
    pilotlib STATIC
    src/proto.cxx src/link.cxx src/reactive.cxx src/lidar.cxx src/bibowire.cxx src/viewfeed.cxx
    src/trimfile.cxx src/carrules.cxx src/car.cxx src/chain.cxx src/tags.cxx
)
target_include_directories(pilotlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${FWLIB})
find_package(Threads REQUIRED)
target_link_libraries(pilotlib PUBLIC Threads::Threads)

# Slamtec's SDK is a checkout built in place with its own makefiles, not a package CMake can
# find, so its path is handed in and the paths below follow its layout. Unset, lidar.cxx
# compiles its refusing half, lidar_probe is not built, and the tests are the same, since none
# needs a device. A PATH cache entry, not option(): option() is a BOOL and would store ON.
set(PILOT_RPLIDAR_SDK "" CACHE PATH "path to a built rplidar_sdk checkout, Linux only")
if(PILOT_RPLIDAR_SDK)
    set(RPLIDAR_INC ${PILOT_RPLIDAR_SDK}/sdk/include)
    set(RPLIDAR_LIB ${PILOT_RPLIDAR_SDK}/output/Linux/Release/libsl_lidar_sdk.a)
    # Checked here, or a mistyped path fails at link naming a symbol instead of the path.
    if(NOT EXISTS ${RPLIDAR_INC}/sl_lidar_driver.h)
        message(FATAL_ERROR "no sl_lidar_driver.h in ${RPLIDAR_INC} - check PILOT_RPLIDAR_SDK")
    endif()
    if(NOT EXISTS ${RPLIDAR_LIB})
        message(FATAL_ERROR "${RPLIDAR_LIB} is missing - run make in the rplidar_sdk checkout")
    endif()
    # PRIVATE include: lidar.hxx names no SDK type. PUBLIC define, so a consumer can #if on a
    # real lidar. SYSTEM: the SDK's headers are not held to -Wall -Wextra.
    target_compile_definitions(pilotlib PUBLIC PILOT_HAVE_RPLIDAR)
    target_include_directories(pilotlib SYSTEM PRIVATE ${RPLIDAR_INC})
    target_link_libraries(pilotlib PUBLIC ${RPLIDAR_LIB} Threads::Threads)
    add_executable(lidar_probe tools/lidar_probe.cxx)
    target_link_libraries(lidar_probe PRIVATE pilotlib)
endif()

# The AprilTag detector (AprilRobotics/apriltag, BSD-2) is the same shape: a checkout built in
# place, handed in by path, Linux only. Unset, tags.cxx compiles its refusing half and the
# apriltag bundle refuses to start, saying so. With it the board also needs libjpeg-dev, since
# the camera speaks MJPEG and the detector wants grey pixels.
#
#   git clone --depth 1 https://github.com/AprilRobotics/apriltag $HOME/apriltag
#   cmake -S $HOME/apriltag -B $HOME/apriltag/build -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF
#   cmake --build $HOME/apriltag/build -j
#   cmake -S firmware/pilot -B build-pilot -DPILOT_APRILTAG=$HOME/apriltag ...
set(PILOT_APRILTAG "" CACHE PATH "path to a built apriltag checkout, Linux only")
if(PILOT_APRILTAG)
    set(APRILTAG_LIB ${PILOT_APRILTAG}/build/libapriltag.a)
    if(NOT EXISTS ${PILOT_APRILTAG}/apriltag.h)
        message(FATAL_ERROR "no apriltag.h in ${PILOT_APRILTAG} - check PILOT_APRILTAG")
    endif()
    if(NOT EXISTS ${APRILTAG_LIB})
        message(FATAL_ERROR "${APRILTAG_LIB} is missing - build the apriltag checkout first")
    endif()
    find_package(JPEG REQUIRED)
    target_compile_definitions(pilotlib PUBLIC PILOT_HAVE_APRILTAG)
    target_include_directories(pilotlib SYSTEM PRIVATE ${PILOT_APRILTAG})
    target_link_libraries(pilotlib PUBLIC ${APRILTAG_LIB} JPEG::JPEG m Threads::Threads)
endif()

# Built with or without the SDK, so CI compiles it; without the SDK lidar::open refuses and it
# exits before it opens the Pico.
add_executable(pilot app/main.cxx)
target_link_libraries(pilot PRIVATE pilotlib)

enable_testing()
# test_viewfeed is not in tools\test.bat: its real half is Linux sockets, and on MSVC every
# call refuses.
foreach(t proto pilot reactive bibowire viewfeed trimfile carrules car chain tags)
    add_executable(test_${t} tests/test_${t}.cxx)
    target_link_libraries(test_${t} PRIVATE pilotlib)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()

# Every programs/<name>.cxx is an executable <name>, built with or without the SDK so CI
# compiles every program. After the targets above, so a name clash is caught.
set(RESERVED_NAMES pilot pilotlib lidar_probe)
file(GLOB CAR_PROGRAMS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/programs/*.cxx)
foreach(src ${CAR_PROGRAMS})
    get_filename_component(name ${src} NAME_WE)
    if(TARGET ${name} OR name IN_LIST RESERVED_NAMES)
        message(FATAL_ERROR "programs/${name}.cxx: ${name} is taken - rename the file")
    endif()
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE pilotlib)
endforeach()
