@echo off
REM Builds and (with "run") executes the LidarSource-over-the-network test.
REM   tests\build_lidarnet_test.bat [run]
REM
REM "run" starts hub\tests\fake_scanfeed.py on the port below - no board, no
REM lidar, no network beyond localhost - then runs the test against it. The fake
REM serves the three sessions the test expects and exits on its own (or after
REM 90 s, so a crashed test leaves no server behind). Its log is what section 5
REM of the test reads.
REM
REM lidar_source.cxx is one object with both workers in it, so the SDK's driver
REM library links here even though nothing serial is started. /MT and /LTCG for
REM the same reason as build.bat. scanwire.cxx comes from the board's tree, as
REM in build.bat: the same object file parses on the hub and formats on the Pi.

setlocal
set HERE=%~dp0
set ROOT=%HERE%..\..
set SRC=%HERE%..\src
set PILOT=%ROOT%\firmware\pilot\src
set PORT=18011
set LOG=%HERE%build\fake_scanfeed.log

call "%~dp0..\..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
set "VS=%VSROOT%"

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%ROOT%\shared" ^
  /I"%SRC%" ^
  /I"%PILOT%" ^
  /I"%ROOT%\vendor\rplidar_sdk\sdk\include" ^
  /I"%ROOT%\vendor\rplidar_sdk\sdk\src" ^
  "%HERE%test_lidarnet.cxx" ^
  "%SRC%\lidar_source.cxx" ^
  "%SRC%\devlink.cxx" ^
  "%PILOT%\scanwire.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_lidarnet.exe" ^
  /link /LTCG "%ROOT%\vendor\rplidar_sdk\output\x64\Release\rplidar_driver.lib" ws2_32.lib advapi32.lib
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [test] built -^> %HERE%build\test_lidarnet.exe

REM An EARLY RETURN, not an if-block: `exit /b %errorlevel%` inside
REM `if ... ( ... )` expands when cmd PARSES the block, before the test has run.
REM All 11 scripts once exited 0 while printing OVERALL: FAIL.
if /i not "%~1"=="run" exit /b 0

if exist "%LOG%" del "%LOG%"
echo [test] starting fake_scanfeed.py on 127.0.0.1:%PORT%
start "" /b python "%HERE%fake_scanfeed.py" %PORT% --sessions junk,abrupt:5,err:3 --log "%LOG%" --timeout 90
echo.
"%HERE%build\test_lidarnet.exe" %PORT% "%LOG%"
exit /b %errorlevel%
