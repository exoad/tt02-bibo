# tt02.exe - the operator hub. MSVC x64.
#
# ---------------------------------------------------------------------------
# build.bat REMAINS THE AUTHORITATIVE BUILD.
#
# This file exists so CLion (and clangd, and anything else that reads
# compile_commands.json) can index the tree, complete a symbol and underline a
# real error. Without a CMakeLists anywhere, CLion opens this repo as an
# EMPTY_MODULE and gives you a text editor with syntax colouring and nothing
# else - which is what it was doing.
#
# It produces the same binary as build.bat because it uses the same flags, the
# same wildcard over src/, the same libraries and the same /MT and /LTCG. Where
# the two could drift, the comments say which one wins. If you are shipping,
# run build.bat; if you are reading code, use this.
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.21)

project(tt02 CXX)

enable_language(RC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Hand this to CLion and to clangd. Cheap, and it is the whole point of the file.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT MSVC)
    message(FATAL_ERROR
            "The hub is Win32 + DX11 + MSVC. There is no other target, and a "
            "half-configured one would be worse than a clear stop.")
endif()

# /MT is MANDATORY: rplidar_driver.lib is built against the static CRT and a
# mismatch is a wall of LNK2038 "RuntimeLibrary mismatch" errors, not a warning.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")

# ---------------------------------------------------------------------------
# The Slamtec driver. NOT built here.
#
# build.bat shells out to msbuild for it when it is missing, and that is the one
# mechanism. Rebuilding it a second way from CMake would mean two places that
# can produce a subtly different .lib.
# ---------------------------------------------------------------------------
set(RPLIDAR_SDK "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/rplidar_sdk")
set(RPLIDAR_LIB "${RPLIDAR_SDK}/output/x64/Release/rplidar_driver.lib")

if(NOT EXISTS "${RPLIDAR_LIB}")
    message(FATAL_ERROR
            "rplidar_driver.lib is missing:\n  ${RPLIDAR_LIB}\n"
            "Run hub\\build.bat once - it builds the driver with msbuild and "
            "then this configure will succeed.")
endif()

# ---------------------------------------------------------------------------
# Dear ImGui. Listed, not globbed: imgui_demo.cpp is deliberately not built and
# a wildcard would quietly pull it back in.
# ---------------------------------------------------------------------------
set(IMGUI "${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui")

# Not vendored into the repo (see .gitignore and the README's pin), so a fresh
# clone reaches here with the directory missing. Say so plainly: the alternative
# is six "cannot open source file" errors that do not name the cause.
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

# ---------------------------------------------------------------------------
# The application.
#
# CONFIGURE_DEPENDS so a new .cpp in src/ is picked up without editing this
# file - which is what build.bat's `src\*.cpp` does, and the property that
# keeps the two from drifting every time a file is added.
# ---------------------------------------------------------------------------
file(GLOB APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx")

if(NOT APP_SOURCES)
    message(FATAL_ERROR "No sources found in hub/src - is this the right directory?")
endif()

add_executable(tt02 WIN32
    ${APP_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app.rc"
)

target_include_directories(tt02 PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../shared"
    "${RPLIDAR_SDK}/sdk/include"
    "${RPLIDAR_SDK}/sdk/src"
)

target_compile_options(tt02 PRIVATE /W4)
target_compile_definitions(tt02 PRIVATE _CRT_SECURE_NO_WARNINGS)

target_link_libraries(tt02 PRIVATE
    imgui
    "${RPLIDAR_LIB}"
    d3d11 dxgi d3dcompiler dwmapi ws2_32
    kernel32 user32 gdi32 shell32 advapi32
    ole32 oleaut32 uuid imm32 setupapi
)

# /LTCG: rplidar_driver.lib is compiled with /GL, so without it the linker
# emits LNK4075 and restarts the whole link pass.
target_link_options(tt02 PRIVATE /LTCG)

# build.bat pins the entry point explicitly. CMake's WIN32 flag already selects
# /SUBSYSTEM:WINDOWS and MSVC then picks WinMainCRTStartup on its own, so this
# is here to say that the omission is deliberate rather than forgotten.

# The exe expects assets/ beside it, exactly as build.bat's layout does.
add_custom_command(TARGET tt02 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/assets"
            "$<TARGET_FILE_DIR:tt02>/assets"
    COMMENT "Copying assets beside the executable"
)

# ---------------------------------------------------------------------------
# The test binaries, so CLion lists and runs them.
#
# Same sources the tests\*.bat scripts compile. Those scripts stay the
# authoritative way to run them from a terminal; this just makes the green
# arrow in the gutter work.
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# The two HARDWARE tests. They need a device plugged in, so they are not part of
# `ctest` - but they still have to be in a target.
#
# A source that belongs to NO target gets no include paths and no defines, and
# the IDE renders every symbol in it unresolved. That is not a smaller problem
# than a failing build; it is the same problem, discovered later.
# ---------------------------------------------------------------------------
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

# Hardware tests are not registered with CTest: a green run means "a device
# answered", and an unplugged machine would report a failure that is about the
# bench rather than about the code.
set_tests_properties(test_pico_link    PROPERTIES DISABLED TRUE)
set_tests_properties(test_lidar_source PROPERTIES DISABLED TRUE)

# ---------------------------------------------------------------------------
# board_preview: a standalone harness that renders the Pico board view to a PNG
# without the rest of the app. Same reason as above - it is a real source and
# it needs a target.
# ---------------------------------------------------------------------------
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
