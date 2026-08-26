@echo off
REM Builds and (with "run") executes the map geometry tests.
REM
REM   tests\build_geom_test.bat        - compile only
REM   tests\build_geom_test.bat run    - compile then run
REM
REM No hardware and no ImGui: map_geometry is deliberately free of both, which
REM is the whole reason it was split out of radar.cpp.

setlocal
set HERE=%~dp0
set VS=C:\Program Files\Microsoft Visual Studio\2022\Community

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  "%HERE%test_map_geometry.cpp" ^
  "%HERE%..\src\map_geometry.cpp" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_map_geometry.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_map_geometry.exe

if /I "%~1"=="run" (
  "%HERE%build\test_map_geometry.exe"
  exit /b %errorlevel%
)
exit /b 0
