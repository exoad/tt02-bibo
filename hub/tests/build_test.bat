@echo off
REM Builds and (with "run") executes the LidarSource hardware test.
REM   tests\build_test.bat [run]   - compile, optionally run against COM7 @ 460800
REM The \\.\ device prefix is applied inside LidarSource, so only the bare port
REM name is passed here - which also keeps Git Bash from mangling backslashes.

setlocal
set HERE=%~dp0
set ROOT=%HERE%..\..
call "%~dp0..\..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
set "VS=%VSROOT%"

set PORT=COM7
set BAUD=460800

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

REM /MT matches the prebuilt driver lib's static CRT; anything else fails at
REM link time with CRT conflicts.
REM scanwire.cxx, from the board's tree: lidar_source.cxx is one object with the
REM network worker in it too, and that worker parses the feed with scanwire.
cl /nologo /EHsc /O2 /MT /W3 /std:c++20 ^
  /I"%HERE%..\..\shared" ^
  /I "%ROOT%\vendor\rplidar_sdk\sdk\include" ^
  /I "%ROOT%\vendor\rplidar_sdk\sdk\src" ^
  /I "%ROOT%\firmware\pilot\src" ^
  "%HERE%test_lidar_source.cxx" ^
  "%ROOT%\hub\src\lidar_source.cxx" ^
  "%ROOT%\hub\src\devlink.cxx" ^
  "%ROOT%\firmware\pilot\src\scanwire.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_lidar_source.exe" ^
  /link /LTCG "%ROOT%\vendor\rplidar_sdk\output\x64\Release\rplidar_driver.lib" ws2_32.lib advapi32.lib
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [test] built -^> %HERE%build\test_lidar_source.exe

REM An EARLY RETURN, not an if-block: `exit /b %errorlevel%` inside
REM `if ... ( ... )` expands when cmd PARSES the block, before the test has run.
REM All 11 scripts once exited 0 while printing OVERALL: FAIL.
if /i not "%~1"=="run" exit /b 0
echo.
"%HERE%build\test_lidar_source.exe" %PORT% %BAUD%
exit /b %errorlevel%
