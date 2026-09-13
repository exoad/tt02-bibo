@echo off
setlocal

rem  bibo viewer, MSVC x64. The only build: there is no CMake project for the viewer.
rem
rem  Usage:  build.bat          incremental; the Dear ImGui objects are cached
rem          build.bat clean    wipe build\ first
rem
rem  The compiler flags are tools\find_vs.bat's, shared with tools\test.bat.

set "ROOT=%~dp0"
set "BUILD=%ROOT%build"
set "OBJ=%BUILD%\obj"
set "IMGUI=%ROOT%..\third_party\imgui"

rem  stb_image, for the camera window's JPEG decode.
set "STB=%ROOT%..\third_party\stb"

rem  bibowire.cxx is compiled from the pilot's tree, never copied, so the viewer and
rem  the pilot share one codec; see docs/bibowire.md section 12.
set "PILOT=%ROOT%..\firmware\pilot\src"

rem  BIBO_EXE_NAME renames the output, for when a running viewer holds bibo.exe.
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

echo [env] Visual Studio x64
call "%ROOT%..\tools\find_vs.bat" cl
if errorlevel 1 exit /b 1

if not exist "%BUILD%" mkdir "%BUILD%"
if not exist "%OBJ%"   mkdir "%OBJ%"

set "CFLAGS=%CLFLAGS% /c"
set "INC=/I"%IMGUI%" /I"%IMGUI%\backends" /I"%STB%" /I"%ROOT%src" /I"%ROOT%..\firmware\lib" /I"%PILOT%""

rem  imgui_demo.cpp is not built: nothing here shows the demo window.
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

rem  Not cached like the ImGui objects: the pilot compiles this file too, and a stale
rem  object would let the viewer and the pilot disagree about the wire.
echo [codec] firmware\pilot\src\bibowire.cxx
cl %CFLAGS% %INC% /Fo"%OBJ%\\" "%PILOT%\bibowire.cxx"
if errorlevel 1 (
    echo [error] compiling bibowire.cxx
    exit /b 1
)

rem  Every .cxx in src\ is built, so a new file needs no edit here.
echo [app] src\*.cxx
cl %CFLAGS% %INC% /Fo"%OBJ%\\" "%ROOT%src\*.cxx"
if errorlevel 1 (
    echo [error] compiling src\*.cxx
    exit /b 1
)

rem  No d3dcompiler.lib here: imgui_impl_dx11.cpp links it itself with a pragma.
rem  ws2_32.lib for link.cxx's sockets.
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
