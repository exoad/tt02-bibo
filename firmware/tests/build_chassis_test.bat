@echo off
REM Builds and (with "run") executes the chassis safety test on the HOST.
REM   tests\build_chassis_test.bat [run]   - compile, optionally run
REM No board involved: BIBO_FAKE_HAL swaps hal.hxx's SDK half for
REM tests/fakes/hal.hxx, which records what would have reached the pins.

setlocal
set HERE=%~dp0

call "%~dp0..\..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
call "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
if errorlevel 1 (
  echo [error] vcvarsall.bat failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /DBIBO_FAKE_HAL ^
  /I"%HERE%..\lib" ^
  "%HERE%test_chassis.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_chassis.exe"
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [test] built -^> %HERE%build\test_chassis.exe

REM An EARLY RETURN, not an if-block: `exit /b %errorlevel%` inside
REM `if ... ( ... )` expands when cmd PARSES the block, before the test has
REM run - which is how these scripts used to exit 0 printing OVERALL: FAIL.
if /i not "%~1"=="run" exit /b 0
echo.
"%HERE%build\test_chassis.exe"
exit /b %errorlevel%
