@echo off
REM Builds and (with "run") executes the LidarSource hardware test.
REM
REM   tests\build_test.bat        - compile only
REM   tests\build_test.bat run    - compile then run against COM7 @ 460800
REM
REM The \\.\ device prefix is applied inside LidarSource, so only the bare port
REM name is passed here - which also keeps Git Bash from mangling backslashes.

setlocal
set HERE=%~dp0
set ROOT=%HERE%..\..
set VS=C:\Program Files\Microsoft Visual Studio\2022\Community

set PORT=COM7
set BAUD=460800

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

REM /MT matches the MultiThreaded static CRT the prebuilt driver lib was
REM compiled with; anything else fails at link time with CRT conflicts.
cl /nologo /EHsc /O2 /MT /W3 /std:c++20 ^
  /I"%HERE%..\..\shared" ^
  /I "%ROOT%\vendor\rplidar_sdk\sdk\include" ^
  /I "%ROOT%\vendor\rplidar_sdk\sdk\src" ^
  "%HERE%test_lidar_source.cxx" ^
  "%ROOT%\hub\src\lidar_source.cxx" ^
  "%ROOT%\hub\src\devlink.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_lidar_source.exe" ^
  /link /LTCG "%ROOT%\vendor\rplidar_sdk\output\x64\Release\rplidar_driver.lib" ws2_32.lib advapi32.lib
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [test] built -^> %HERE%build\test_lidar_source.exe

REM An EARLY RETURN, not an if-block, and the reason is a cmd.exe
REM parsing rule that cost every one of these scripts its exit code:
REM `exit /b %errorlevel%` inside `if ... ( ... )` is expanded when
REM cmd parses the BLOCK, which is before the test has run. All 11
REM scripts exited 0 while printing OVERALL: FAIL, so not one of them
REM could ever have gated a commit. Out here the expansion happens
REM when the line is reached.
if /i not "%~1"=="run" exit /b 0
echo.
"%HERE%build\test_lidar_source.exe" %PORT% %BAUD%
exit /b %errorlevel%
