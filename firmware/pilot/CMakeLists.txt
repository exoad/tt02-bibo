# The companion board's build. Host C++ only - no Pico SDK, no cross-compiler -
# so it builds with the g++ on the Orange Pi or any Linux box the same way the
# .bat files build it with MSVC on the laptop. Two toolchains, one source; the
# tests are what hold the two to the same answer.
#
#   cmake -S firmware/pilot -B build-pilot && cmake --build build-pilot -j
#   ctest --test-dir build-pilot --output-on-failure
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

add_library(pilot STATIC src/proto.cxx src/link.cxx src/autonomy.cxx src/reactive.cxx)
target_include_directories(pilot PUBLIC ${SHARED} ${CMAKE_CURRENT_SOURCE_DIR}/src ${FWLIB})
find_package(Threads REQUIRED)
target_link_libraries(pilot PUBLIC Threads::Threads)

enable_testing()
foreach(t proto pilot reactive)
    add_executable(test_${t} tests/test_${t}.cxx)
    target_link_libraries(test_${t} PRIVATE pilot)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()
