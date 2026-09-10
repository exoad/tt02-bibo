@echo off
REM Builds and (with "run") executes the bibowire wire format tests.
REM   tests\build_bibowire_test.bat [run]   - compile, optionally run
REM No hardware, no socket and no Orange Pi: bibowire.cxx is pure byte work and
REM pure arithmetic, the same object file the board's pilot and the Windows
REM viewer both compile, so it is proved here for both ends at once - and the
REM deadman is a pure function, so the safety property is exercised on a laptop
REM in microseconds rather than by sleeping next to a car.

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
  "%HERE%test_bibowire.cxx" ^
  "%HERE%..\src\bibowire.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_bibowire.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_bibowire.exe

REM An EARLY RETURN, not an if-block - see build_proto_test.bat for why.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_bibowire.exe"
exit /b %errorlevel%
