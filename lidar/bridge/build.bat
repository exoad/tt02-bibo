@echo off
REM Builds lidar_bridge.exe (x64) against the rplidar_sdk driver.
REM Run from anywhere: bridge\build.bat

setlocal
set HERE=%~dp0
REM The SDK lives under vendor/ at the repo root, two levels up from here.
set ROOT=%HERE%..\..\vendor
set VS=C:\Program Files\Microsoft Visual Studio\2022\Community

REM vcvarsall prints a harmless "'vswhere.exe' is not recognized" line when
REM vswhere is not on PATH; the environment is still set up correctly.
call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [build] vcvarsall failed
  exit /b 1
)

REM 1. Build the SDK driver static lib for x64.
REM    Only the ultra_simple project has the win32/x64 library-path bug, so the
REM    driver itself builds fine for x64 and we link it ourselves below.
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
