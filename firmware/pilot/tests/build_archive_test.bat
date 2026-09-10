@echo off
REM Builds and (with "run") executes the .bibo archive format tests.
REM   tests\build_archive_test.bat [run]   - compile, optionally run
REM No hardware, no socket and no Orange Pi: archive.cxx is pure byte work over
REM a source and a sink it is handed, so the whole suite runs against memory and
REM proves on a laptop what will run off the board's SD card - including the
REM case that matters most, a run whose power was pulled before the footer was
REM written, which is a thing you cannot ask a real card for on demand.

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
  "%HERE%test_archive.cxx" ^
  "%HERE%..\src\archive.cxx" ^
  "%HERE%..\src\bibowire.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_archive.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_archive.exe

REM An EARLY RETURN, not an if-block - see build_proto_test.bat for why.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_archive.exe"
exit /b %errorlevel%
