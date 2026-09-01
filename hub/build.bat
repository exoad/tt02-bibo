@echo off
setlocal

rem ===========================================================================
rem  RPLIDAR C1 - Dear ImGui viewer :: MSVC x64 build
rem
rem  Usage:  build.bat            incremental (imgui objects are reused)
rem          build.bat clean      wipe build\ first
rem
rem  /MT is MANDATORY: rplidar_driver.lib is built with the static CRT and a
rem  mismatch produces a wall of LNK2038 "RuntimeLibrary mismatch" errors.
rem ===========================================================================

set "ROOT=%~dp0"
set "BUILD=%ROOT%build"
set "OBJ=%BUILD%\obj"
set "IMGUI=%ROOT%third_party\imgui"
set "SDK=%ROOT%..\vendor\rplidar_sdk"
set "SDKLIB=%SDK%\output\x64\Release\rplidar_driver.lib"
rem  BIBO_EXE_NAME overrides the output name. There for exactly one
rem  situation: the app is running and holding bibo.exe, so a link cannot
rem  replace it, and the alternative is killing somebody's live session
rem  just to verify a build.
if not defined BIBO_EXE_NAME set "BIBO_EXE_NAME=bibo.exe"
set "EXE=%BUILD%\%BIBO_EXE_NAME%"

if /i "%~1"=="clean" (
    echo [clean] removing %BUILD%
    if exist "%BUILD%" rmdir /s /q "%BUILD%"
)

rem --- MSVC x64 environment ---------------------------------------------------
rem  vcvarsall may print a harmless "'vswhere.exe' is not recognized" line.
echo [env] Visual Studio 2022 x64
call "%~dp0..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
call "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" x64
if errorlevel 1 (
    echo [error] vcvarsall.bat failed
    exit /b 1
)

rem --- rplidar SDK driver library --------------------------------------------
rem  Only the demo .vcxproj files have a broken x64 library path; the driver
rem  project itself builds cleanly at x64.
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

rem --- flags ------------------------------------------------------------------
set "CFLAGS=/nologo /c /EHsc /MT /O2 /std:c++20 /W4 /D_CRT_SECURE_NO_WARNINGS"
set "INC=/I"%IMGUI%" /I"%IMGUI%\backends" /I"%ROOT%src" /I"%ROOT%..\shared" /I"%SDK%\sdk\include" /I"%SDK%\sdk\src""

rem --- Dear ImGui core + backends (compiled once, reused afterwards) ----------
rem  imgui_demo.cpp is deliberately NOT built.
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

rem --- application sources ----------------------------------------------------
rem  Wildcard on purpose: every .cpp dropped into src\ is picked up.
echo [app] src\*.cxx
cl %CFLAGS% %INC% /Fo"%OBJ%\\" "%ROOT%src\*.cxx"
if errorlevel 1 (
    echo [error] compiling src\*.cxx
    exit /b 1
)

rem --- resources ---------------------------------------------------------------
rem  The icon and VERSIONINFO. rc.exe comes from the Windows SDK and is on PATH
rem  after vcvarsall. Not fatal if it is missing: the app runs without an icon,
rem  and refusing to build over decoration would be the wrong trade.
set "RES=%OBJ%\app.res"
echo [rc] src\app.rc
rc /nologo /fo "%RES%" "%ROOT%src\app.rc" >nul 2>&1
if errorlevel 1 (
    echo [warn] rc.exe failed - building without an icon
    set "RES="
)

rem --- link -------------------------------------------------------------------
rem  /LTCG: rplidar_driver.lib is compiled with /GL, so without it the linker
rem  emits LNK4075 and restarts the link pass.
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
