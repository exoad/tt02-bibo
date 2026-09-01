# bibo.exe - the operator hub. MSVC x64.
#
# build.bat REMAINS THE AUTHORITATIVE BUILD. This file exists so CLion and clangd
# can index the tree; without a CMakeLists anywhere CLion opens the repo as an
# EMPTY_MODULE and gives you syntax colouring and nothing else. It produces the
# same binary because it uses the same flags, wildcard, libraries, /MT and /LTCG.
# Shipping -> build.bat. Reading code -> this.

cmake_minimum_required(VERSION 3.21)

project(bibo CXX)

enable_language(RC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Hand this to CLion and clangd - the whole point of the file.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT MSVC)
    message(FATAL_ERROR
            "The hub is Win32 + DX11 + MSVC. There is no other target, and a "
            "half-configured one would be worse than a clear stop.")
endif()

# /MT is MANDATORY: rplidar_driver.lib is built against the static CRT and a
# mismatch is a wall of LNK2038 "RuntimeLibrary mismatch" errors, not a warning.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")

# The Slamtec driver, NOT built here: build.bat shells out to msbuild when it is
# missing, and a second mechanism would mean two subtly different .libs.
set(RPLIDAR_SDK "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/rplidar_sdk")
set(RPLIDAR_LIB "${RPLIDAR_SDK}/output/x64/Release/rplidar_driver.lib")

if(NOT EXISTS "${RPLIDAR_LIB}")
    message(FATAL_ERROR
            "rplidar_driver.lib is missing:\n  ${RPLIDAR_LIB}\n"
            "Run hub\\build.bat once - it builds the driver with msbuild and "
            "then this configure will succeed.")
endif()

# Dear ImGui. Listed, not globbed: imgui_demo.cpp is deliberately not built and
# a wildcard would quietly pull it back in.
set(IMGUI "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui")

# Not vendored (see .gitignore and the README's pin), so a fresh clone arrives
# here with the directory missing. Say so: the alternative is six "cannot open
# source file" errors that do not name the cause.
if(NOT EXISTS "${IMGUI}/imgui.cpp")
    message(FATAL_ERROR
            "Dear ImGui is not present:\n  ${IMGUI}\n"
            "Clone it first - see the pin in README.md:\n"
            "  git clone --depth 1 --branch v1.92.9 "
            "https://github.com/ocornut/imgui hub/third_party/imgui")
endif()

add_library(imgui STATIC
    "${IMGUI}/imgui.cpp"
    "${IMGUI}/imgui_draw.cpp"
    "${IMGUI}/imgui_tables.cpp"
    "${IMGUI}/imgui_widgets.cpp"
    "${IMGUI}/backends/imgui_impl_win32.cpp"
    "${IMGUI}/backends/imgui_impl_dx11.cpp"
)

target_include_directories(imgui PUBLIC "${IMGUI}" "${IMGUI}/backends")
target_compile_definitions(imgui PUBLIC _CRT_SECURE_NO_WARNINGS)

# The application. CONFIGURE_DEPENDS so a new source in src/ is picked up
# without editing this file - matching build.bat's wildcard, which is what keeps
# the two from drifting every time a file is added.
file(GLOB APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx")

if(NOT APP_SOURCES)
    message(FATAL_ERROR "No sources found in hub/src - is this the right directory?")
endif()

add_executable(bibo WIN32
    ${APP_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app.rc"
)

target_include_directories(bibo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
    "${RPLIDAR_SDK}/sdk/include"
    "${RPLIDAR_SDK}/sdk/src"
)

target_compile_options(bibo PRIVATE /W4)
target_compile_definitions(bibo PRIVATE _CRT_SECURE_NO_WARNINGS)

target_link_libraries(bibo PRIVATE
    imgui
    "${RPLIDAR_LIB}"
    d3d11 dxgi d3dcompiler dwmapi ws2_32
    kernel32 user32 gdi32 shell32 advapi32
    ole32 oleaut32 uuid imm32 setupapi
)

# /LTCG: rplidar_driver.lib is compiled with /GL, so without it the linker
# emits LNK4075 and restarts the whole link pass.
target_link_options(bibo PRIVATE /LTCG)

# No explicit entry point (build.bat pins one): WIN32 already selects
# /SUBSYSTEM:WINDOWS and MSVC picks WinMainCRTStartup. Deliberate, not forgotten.

# The exe expects assets/ beside it, exactly as build.bat's layout does.
add_custom_command(TARGET bibo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/assets"
            "$<TARGET_FILE_DIR:bibo>/assets"
    COMMENT "Copying assets beside the executable"
)

# The test binaries, so CLion lists and runs them. Same sources the tests\*.bat
# scripts compile; those stay authoritative from a terminal.
enable_testing()

function(add_hub_test name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
        "${IMGUI}")
    target_compile_definitions(${name} PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_options(${name} PRIVATE /W4)
    set_target_properties(${name} PROPERTIES EXCLUDE_FROM_ALL TRUE)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_hub_test(test_editor
    tests/test_editor.cpp src/editor.cpp src/syntax.cpp src/complete.cpp)

add_hub_test(test_map_geometry
    tests/test_map_geometry.cpp src/map_geometry.cpp)

add_hub_test(test_lights
    tests/test_lights.cpp src/lights.cpp)

add_hub_test(test_workspace
    tests/test_workspace.cpp src/workspace.cpp)

# The two HARDWARE tests. They need a device plugged in, so they are excluded
# from `ctest` - but they still need a target: a source belonging to NO target
# gets no include paths and no defines, and the IDE shows every symbol in it as
# unresolved.
add_hub_test(test_pico_link
    tests/test_pico_link.cpp src/pico_link.cpp)

add_hub_test(test_lidar_source
    tests/test_lidar_source.cpp src/lidar_source.cpp)

target_include_directories(test_lidar_source PRIVATE
    "${RPLIDAR_SDK}/sdk/include"
    "${RPLIDAR_SDK}/sdk/src"
)
target_link_libraries(test_lidar_source PRIVATE "${RPLIDAR_LIB}")
target_link_options(test_lidar_source PRIVATE /LTCG)

# Disabled in CTest: a green run only means "a device answered", so an unplugged
# machine would report a failure about the bench rather than the code.
set_tests_properties(test_pico_link    PROPERTIES DISABLED TRUE)
set_tests_properties(test_lidar_source PROPERTIES DISABLED TRUE)

# board_preview: standalone harness rendering the Pico board view to a PNG
# without the rest of the app. Targeted for the same reason as above.
add_executable(board_preview
    tests/board_preview/main.cpp
    src/board_view.cpp
    src/theme.cpp
)
set_target_properties(board_preview PROPERTIES EXCLUDE_FROM_ALL TRUE)
target_include_directories(board_preview PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
)
target_compile_definitions(board_preview PRIVATE _CRT_SECURE_NO_WARNINGS)
target_link_libraries(board_preview PRIVATE imgui d3d11 dxgi d3dcompiler)
