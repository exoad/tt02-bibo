@echo off
setlocal

rem  RPLIDAR C1 - Dear ImGui viewer :: MSVC x64 build
rem  Usage:  build.bat [clean]    incremental (imgui objects are reused);
rem          "clean" wipes build\ first
rem  /MT is MANDATORY: rplidar_driver.lib is built with the static CRT and a
rem  mismatch produces a wall of LNK2038 "RuntimeLibrary mismatch" errors.

set "ROOT=%~dp0"
set "BUILD=%ROOT%build"
set "OBJ=%BUILD%\obj"
set "IMGUI=%ROOT%third_party\imgui"
set "SDK=%ROOT%..\vendor\rplidar_sdk"
set "SDKLIB=%SDK%\output\x64\Release\rplidar_driver.lib"
rem  BIBO_EXE_NAME overrides the output name, for one situation: the app is
rem  running and holding bibo.exe, so the link cannot replace it.
if not defined BIBO_EXE_NAME set "BIBO_EXE_NAME=bibo.exe"
set "EXE=%BUILD%\%BIBO_EXE_NAME%"

if /i "%~1"=="clean" (
    echo [clean] removing %BUILD%
    if exist "%BUILD%" rmdir /s /q "%BUILD%"
)

rem --- MSVC x64 env. find_vs.bat puts the VS Installer directory on PATH, which
rem  is what stops vcvarsall printing "vswhere.exe is not recognized" first.
echo [env] Visual Studio 2022 x64
call "%~dp0..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
call "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" x64
if errorlevel 1 (
    echo [error] vcvarsall.bat failed
    exit /b 1
)

rem --- rplidar SDK driver lib. Only the demo .vcxproj files have a broken x64
rem  library path; the driver project itself builds cleanly at x64.
if not exist "%SDKLIB%" (
    echo [sdk] building rplidar_driver x64/Release
    msbuild "%SDK%\workspaces\vc14\sdk_and_demo.sln" -t:rplidar_driver -p:Configuration=Release -p:Platform=x64 -m -v:minimal -nologo
    if errorlevel 1 (
        echo [error] failed to build rplidar_driver.lib
        exit /b 1
    )
)
if not exist "%SDKLIB%" (
    echo [error] missing %SDKLIB%
    exit /b 1
)

if not exist "%BUILD%" mkdir "%BUILD%"
if not exist "%OBJ%"   mkdir "%OBJ%"

rem --- flags ---
set "CFLAGS=/nologo /c /EHsc /MT /O2 /std:c++20 /W4 /D_CRT_SECURE_NO_WARNINGS"
set "INC=/I"%IMGUI%" /I"%IMGUI%\backends" /I"%ROOT%src" /I"%ROOT%..\shared" /I"%ROOT%..\firmware\pilot\src" /I"%SDK%\sdk\include" /I"%SDK%\sdk\src""

rem --- Dear ImGui core + backends, compiled once. imgui_demo.cpp is NOT built.
echo [imgui] core + win32/dx11 backends

if not exist "%OBJ%\imgui.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui.obj" "%IMGUI%\imgui.cpp"
    if errorlevel 1 (
        echo [error] imgui.cpp
        exit /b 1
    )
)
if not exist "%OBJ%\imgui_draw.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_draw.obj" "%IMGUI%\imgui_draw.cpp"
    if errorlevel 1 (
        echo [error] imgui_draw.cpp
        exit /b 1
    )
)
if not exist "%OBJ%\imgui_tables.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_tables.obj" "%IMGUI%\imgui_tables.cpp"
    if errorlevel 1 (
        echo [error] imgui_tables.cpp
        exit /b 1
    )
)
if not exist "%OBJ%\imgui_widgets.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_widgets.obj" "%IMGUI%\imgui_widgets.cpp"
    if errorlevel 1 (
        echo [error] imgui_widgets.cpp
        exit /b 1
    )
)
if not exist "%OBJ%\imgui_impl_win32.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_impl_win32.obj" "%IMGUI%\backends\imgui_impl_win32.cpp"
    if errorlevel 1 (
        echo [error] imgui_impl_win32.cpp
        exit /b 1
    )
)
if not exist "%OBJ%\imgui_impl_dx11.obj" (
    cl %CFLAGS% %INC% /Fo"%OBJ%\imgui_impl_dx11.obj" "%IMGUI%\backends\imgui_impl_dx11.cpp"
    if errorlevel 1 (
        echo [error] imgui_impl_dx11.cpp
        exit /b 1
    )
)

rem --- application sources. Wildcard on purpose: every .cxx in src\ is built.
echo [app] src\*.cxx
cl %CFLAGS% %INC% /Fo"%OBJ%\\" "%ROOT%src\*.cxx"
if errorlevel 1 (
    echo [error] compiling src\*.cxx
    exit /b 1
)

rem --- the reactive driver. ONE source, from the companion board's tree: it is
rem  pure C++ over shared.hxx, tested on the host, and the hub is its first
rem  binding to a real lidar and a real Pico. Not copied here - two copies of a
rem  controller agree until somebody fixes a sign in one of them.
echo [app] ..\firmware\pilot\src\reactive.cxx
cl %CFLAGS% %INC% /Fo"%OBJ%\\" "%ROOT%..\firmware\pilot\src\reactive.cxx"
if errorlevel 1 (
    echo [error] compiling reactive.cxx
    exit /b 1
)

rem --- resources: the icon and VERSIONINFO. rc.exe (Windows SDK, on PATH after
rem  vcvarsall) is not fatal if missing - the app just runs without an icon.
set "RES=%OBJ%\app.res"
echo [rc] src\app.rc
rc /nologo /fo "%RES%" "%ROOT%src\app.rc" >nul 2>&1
if errorlevel 1 (
    echo [warn] rc.exe failed - building without an icon
    set "RES="
)

rem --- link. /LTCG: rplidar_driver.lib is compiled with /GL, so without it the
rem  linker emits LNK4075 and restarts the link pass.
echo [link] %EXE%
link /nologo /LTCG /OUT:"%EXE%" /SUBSYSTEM:WINDOWS /ENTRY:WinMainCRTStartup ^
    "%OBJ%\*.obj" ^
    %RES% ^
    "%SDKLIB%" ^
    d3d11.lib dxgi.lib d3dcompiler.lib dwmapi.lib ws2_32.lib ^
    kernel32.lib user32.lib gdi32.lib shell32.lib advapi32.lib ^
    ole32.lib oleaut32.lib uuid.lib imm32.lib setupapi.lib
if errorlevel 1 (
    echo [error] link failed
    exit /b 1
)

echo.
echo [ok] %EXE%
exit /b 0
