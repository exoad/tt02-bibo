@echo off
REM Builds and (with "run") executes the PicoLink hardware test.
REM   tests\build_pico_test.bat [run]   - compile, optionally run against COM10
REM devlink.cxx must stay in the source list: when it was added and this script
REM was not told, the whole suite failed on two unresolved symbols.
REM The port name is hardcoded in test_pico_link.cxx rather than passed on the
REM command line, because Git Bash mangles the \\.\ device prefix.
REM /MT matches the rest of this project (the rplidar driver lib is static-CRT).

setlocal
set HERE=%~dp0
set SRC=%HERE%..\src
call "%~dp0..\..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
set "VS=%VSROOT%"

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 ^
  /I"%HERE%..\..\shared" ^
  /I "%SRC%" ^
  "%HERE%test_pico_link.cxx" ^
  "%SRC%\pico_link.cxx" ^
  "%SRC%\devlink.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_pico_link.exe" ^
  /link kernel32.lib advapi32.lib setupapi.lib ws2_32.lib
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [test] built -^> %HERE%build\test_pico_link.exe

REM An EARLY RETURN, not an if-block: `exit /b %errorlevel%` inside
REM `if ... ( ... )` expands when cmd PARSES the block, before the test has run.
REM All 11 scripts once exited 0 while printing OVERALL: FAIL.
if /i not "%~1"=="run" exit /b 0
echo.
"%HERE%build\test_pico_link.exe"
exit /b %errorlevel%
