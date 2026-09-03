@echo off
REM Builds lidar_bridge.exe (x64) against the rplidar_sdk driver. Run: bridge\build.bat

setlocal
set HERE=%~dp0
set ROOT=%HERE%..\..\vendor
call "%~dp0..\..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
set "VS=%VSROOT%"

REM find_vs.bat has put the VS Installer directory on PATH, so vcvarsall no longer
REM prints "vswhere.exe is not recognized" on its way in.
call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [build] vcvarsall failed
  exit /b 1
)

REM 1. SDK driver static lib for x64 (only ultra_simple has the x64 lib-path bug).
msbuild "%ROOT%\rplidar_sdk\workspaces\vc14\sdk_and_demo.sln" ^
  -t:rplidar_driver -p:Configuration=Release -p:Platform=x64 -m -v:minimal -nologo
if errorlevel 1 (
  echo [build] driver build failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

REM 2. Compile the bridge. /MT to match the driver's MultiThreaded runtime.
cl /nologo /EHsc /O2 /MT /W4 /std:c++20 ^
  /I "%HERE%..\..\shared" ^
  /I "%ROOT%\rplidar_sdk\sdk\include" ^
  /I "%ROOT%\rplidar_sdk\sdk\src" ^
  "%HERE%lidar_bridge.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\lidar_bridge.exe" ^
  /link /LTCG "%ROOT%\rplidar_sdk\output\x64\Release\rplidar_driver.lib" ws2_32.lib
if errorlevel 1 (
  echo [build] bridge compile failed
  exit /b 1
)

echo [build] OK -^> %HERE%build\lidar_bridge.exe
