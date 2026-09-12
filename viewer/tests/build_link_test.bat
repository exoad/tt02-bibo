@echo off
REM Builds and (with "run") executes the viewer's link tests.
REM   viewer\tests\build_link_test.bat [run]   - compile, optionally run
REM
REM No socket and no board: the half of link.cxx this exercises is pure - bytes
REM and a millisecond in, decoded state out - which is the only half that can be
REM proved while the pilot that serves port 8020 is still being written. What is
REM NOT covered is listed at the top of test_link.cxx rather than left to be
REM assumed.
REM
REM ws2_32.lib is linked because link.cxx contains the socket half too; nothing
REM here calls it.

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

REM  ..\src FIRST, and that order is load-bearing: firmware\pilot\src has a
REM  link.hxx of its own (the Pico's carlink) and this file includes "link.hxx".
REM  With the board's directory ahead of the viewer's, this test silently
REM  compiled against the wrong header - it is viewer\build.bat's order, for the
REM  same reason.
if not exist "%HERE%..\..\third_party\stb\stb_image.h" (
  echo [test] stb_image.h not found - it is gitignored, see THIRD_PARTY.md
  echo        git clone --depth 1 https://github.com/nothings/stb third_party\stb
  exit /b 1
)

REM  jpeg.cxx is compiled in because the camera's DECODE is exercised here, on
REM  a real JPEG, with no board and no graphics device. That is the whole
REM  reason the decoder is its own module and not part of camera.cxx, which
REM  owns a D3D11 texture and could not be linked into a console test.
REM
REM  vlog.cxx is compiled in because link.cxx writes its round-trip log through
REM  it. Nothing here calls vlog::open, so every line it would write is a no-op.
REM
REM  settings.cxx is compiled in for its text half - the integer parser, the
REM  clamping and the key list. The suite reads one path that does not exist and
REM  never writes a settings file.
cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\src" /I"%HERE%..\..\shared" ^
  /I"%HERE%..\..\firmware\pilot\src" /I"%HERE%..\..\third_party\imgui" ^
  /I"%HERE%..\..\third_party\stb" ^
  "%HERE%test_link.cxx" ^
  "%HERE%..\src\link.cxx" ^
  "%HERE%..\src\vlog.cxx" ^
  "%HERE%..\src\jpeg.cxx" ^
  "%HERE%..\src\orient.cxx" ^
  "%HERE%..\src\settings.cxx" ^
  "%HERE%..\..\firmware\pilot\src\bibowire.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_link.exe" ^
  /link /SUBSYSTEM:CONSOLE ws2_32.lib
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_link.exe

REM An EARLY RETURN, not an if-block - see firmware\pilot\tests for why.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_link.exe"
exit /b %errorlevel%
