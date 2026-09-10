@echo off
setlocal

rem  bibo viewer :: MSVC x64. THE AUTHORITATIVE BUILD - there is no CMake
rem  project for the viewer and no second way to produce the exe.
rem
rem  Usage:  build.bat          incremental; the Dear ImGui objects are cached
rem          build.bat clean    wipe build\ first
rem
rem  /MT rather than /MD. It used to be mandatory, because rplidar_driver.lib is
rem  built against the static CRT and a mismatch is a wall of LNK2038. That
rem  library is GONE - under the new protocol the board owns the lidar and this
rem  program only ever receives scans over the network - so /MT is now a choice
rem  rather than a constraint, and it is kept for the reason that outlives the
rem  SDK: the exe runs on a machine with no redistributable installed.

set "ROOT=%~dp0"
set "BUILD=%ROOT%build"
set "OBJ=%BUILD%\obj"
set "IMGUI=%ROOT%..\third_party\imgui"

rem  stb_image, for the camera window's JPEG decode. Same arrangement as Dear
rem  ImGui: third_party\ is gitignored and this is CLONED, never vendored - see
rem  THIRD_PARTY.md, which records the version and the licence.
set "STB=%ROOT%..\third_party\stb"

rem  The board's source tree. bibowire.cxx is compiled INTO this exe rather than
rem  copied or reimplemented: docs/bibowire.md section 11 requires the viewer and
rem  the pilot to share the object file so the encoder and the decoder cannot
rem  drift into disagreeing about a field's offset while both still compile.
set "PILOT=%ROOT%..\firmware\pilot\src"

rem  BIBO_EXE_NAME overrides the output name, for one situation: the viewer is
rem  running and holding bibo.exe, so the link cannot replace it.
if not defined BIBO_EXE_NAME set "BIBO_EXE_NAME=bibo.exe"
set "EXE=%BUILD%\%BIBO_EXE_NAME%"

if /i "%~1"=="clean" (
    echo [clean] removing %BUILD%
    if exist "%BUILD%" rmdir /s /q "%BUILD%"
)

if not exist "%IMGUI%\imgui.cpp" (
    echo [error] Dear ImGui not found at %IMGUI%
    echo         It is gitignored - see .gitignore and THIRD_PARTY.md.
    exit /b 1
)

if not exist "%STB%\stb_image.h" (
    echo [error] stb_image.h not found at %STB%
    echo         It is gitignored - see .gitignore and THIRD_PARTY.md.
    echo         git clone --depth 1 https://github.com/nothings/stb third_party\stb
    exit /b 1
)

rem --- MSVC x64 env. find_vs.bat puts the VS Installer directory on PATH, which
rem  is what stops vcvarsall printing "vswhere.exe is not recognized" first.
echo [env] Visual Studio x64
call "%ROOT%..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
call "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" x64
if errorlevel 1 (
    echo [error] vcvarsall.bat failed
    exit /b 1
)

if not exist "%BUILD%" mkdir "%BUILD%"
if not exist "%OBJ%"   mkdir "%OBJ%"

set "CFLAGS=/nologo /c /EHsc /MT /O2 /std:c++20 /W4 /D_CRT_SECURE_NO_WARNINGS"
set "INC=/I"%IMGUI%" /I"%IMGUI%\backends" /I"%STB%" /I"%ROOT%src" /I"%ROOT%..\shared" /I"%PILOT%""

rem --- Dear ImGui core + the win32/dx11 backends, compiled once and cached.
rem  imgui_demo.cpp is NOT built: nothing here shows the demo window.
echo [imgui] core + win32/dx11 backends

if not exist "%OBJ%\imgui.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui.obj" "%IMGUI%\imgui.cpp"
    if errorlevel 1 exit /b 1
)
if not exist "%OBJ%\imgui_draw.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_draw.obj" "%IMGUI%\imgui_draw.cpp"
    if errorlevel 1 exit /b 1
)
if not exist "%OBJ%\imgui_tables.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_tables.obj" "%IMGUI%\imgui_tables.cpp"
    if errorlevel 1 exit /b 1
)
if not exist "%OBJ%\imgui_widgets.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_widgets.obj" "%IMGUI%\imgui_widgets.cpp"
    if errorlevel 1 exit /b 1
)
if not exist "%OBJ%\imgui_impl_win32.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_impl_win32.obj" "%IMGUI%\backends\imgui_impl_win32.cpp"
    if errorlevel 1 exit /b 1
)
if not exist "%OBJ%\imgui_impl_dx11.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_impl_dx11.obj" "%IMGUI%\backends\imgui_impl_dx11.cpp"
    if errorlevel 1 exit /b 1
)

rem --- the shared codec. NOT cached like the ImGui objects above: it is the one
rem  file in this build that also belongs to another program, and a stale copy of
rem  it is the exact failure the shared-object rule exists to prevent.
echo [codec] firmware\pilot\src\bibowire.cxx
cl %CFLAGS% %INC% /Fo"%OBJ%\\" "%PILOT%\bibowire.cxx"
if errorlevel 1 (
    echo [error] compiling bibowire.cxx
    exit /b 1
)

rem --- the viewer. Wildcard on purpose: every .cxx in src\ is built.
echo [app] src\*.cxx
cl %CFLAGS% %INC% /Fo"%OBJ%\\" "%ROOT%src\*.cxx"
if errorlevel 1 (
    echo [error] compiling src\*.cxx
    exit /b 1
)

rem --- link.
rem  No d3dcompiler.lib: there are no shaders in this program - the 3D view is a
rem  perspective divide feeding ImGui's draw list, and the only HLSL in the
rem  binary is the backend's, which ships precompiled.
rem  ws2_32.lib IS linked, ahead of its first caller: the protocol client that
rem  receives scans is being written now, and finding out at link time that the
rem  build never named a socket library is a minute nobody needs to spend.
echo [link] %EXE%
link /nologo /OUT:"%EXE%" /SUBSYSTEM:WINDOWS /ENTRY:WinMainCRTStartup ^
    "%OBJ%\*.obj" ^
    d3d11.lib dxgi.lib dwmapi.lib ws2_32.lib ^
    kernel32.lib user32.lib gdi32.lib shell32.lib advapi32.lib ^
    ole32.lib oleaut32.lib uuid.lib imm32.lib
if errorlevel 1 (
    echo [error] link failed
    exit /b 1
)

echo.
echo [ok] %EXE%
exit /b 0
