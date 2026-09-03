@echo off
REM Builds and (with "run") executes the reactive driving tests.
REM   tests\build_reactive_test.bat [run]   - compile, optionally run
REM No lidar, no car, no serial port and no Orange Pi: reactive.cxx is pure
REM arithmetic over an array, so the behaviour that would otherwise be proved by
REM driving at a wall is proved here instead.

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
if not exist "%HERE%build\reactive" mkdir "%HERE%build\reactive"

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\..\..\shared" /I"%HERE%..\src" ^
  "%HERE%test_reactive.cxx" ^
  "%HERE%..\src\reactive.cxx" ^
  /Fo"%HERE%build\reactive\\" ^
  /Fe"%HERE%build\test_reactive.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_reactive.exe

REM An EARLY RETURN, not an if-block - see build_proto_test.bat for why.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_reactive.exe"
exit /b %errorlevel%
