@echo off
REM Builds and (with "run") executes the map geometry tests.
REM   tests\build_geom_test.bat [run]   - compile, optionally run
REM No hardware and no ImGui: map_geometry is deliberately free of both, which
REM is the whole reason it was split out of radar.cxx.

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

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\..\shared" ^
  "%HERE%test_map_geometry.cxx" ^
  "%HERE%..\src\map_geometry.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_map_geometry.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_map_geometry.exe

REM An EARLY RETURN, not an if-block: `exit /b %errorlevel%` inside
REM `if ... ( ... )` expands when cmd PARSES the block, before the test has run.
REM All 11 scripts once exited 0 while printing OVERALL: FAIL.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_map_geometry.exe"
exit /b %errorlevel%
