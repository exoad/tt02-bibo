@echo off
REM Builds and (with "run") executes the lib/control.hxx tests.
REM   firmware\tests\build_control_test.bat [run]   - compile, optionally run
REM Compiled for the HOST with MSVC, not for the board: the header needs nothing
REM from the Pico SDK, only lib/shared.hxx, and that is worth keeping - code only
REM exercised by flashing a microcontroller is code nobody exercises.
REM MSVC takes the language from the file extension; a rename would change it.

setlocal
set HERE=%~dp0
call "%~dp0..\..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
set "VS=%VSROOT%"

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

cl /nologo /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\lib" ^
  "%HERE%test_control.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_control.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_control.exe

REM An EARLY RETURN, not an if-block: `exit /b %errorlevel%` inside
REM `if ... ( ... )` expands when cmd PARSES the block, before the test has run.
REM All 11 scripts once exited 0 while printing OVERALL: FAIL.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_control.exe"
exit /b %errorlevel%
