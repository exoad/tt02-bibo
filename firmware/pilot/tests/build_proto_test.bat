@echo off
REM Builds and (with "run") executes the line-protocol tests.
REM   tests\build_proto_test.bat [run]   - compile, optionally run
REM No hardware, no serial port and no Orange Pi: proto.cxx is pure string work
REM so it can be proved before the board it is for exists.

setlocal
set HERE=%~dp0
call "%~dp0..\..\..\tools\find_vs.bat"
if errorlevel 1 exit /b 1
set "VS=%VSROOT%"

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\..\..\shared" /I"%HERE%..\src" ^
  "%HERE%test_proto.cxx" ^
  "%HERE%..\src\proto.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_proto.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_proto.exe

REM An EARLY RETURN, not an if-block: `exit /b %errorlevel%` inside
REM `if ... ( ... )` expands when cmd PARSES the block, before the test has run,
REM so the script exits 0 no matter what the test printed.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_proto.exe"
exit /b %errorlevel%
