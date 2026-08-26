@echo off
REM Builds and (with "run") executes the vehicle lighting tests.
REM
REM   tests\build_geom_test.bat        - compile only
REM   tests\build_geom_test.bat run    - compile then run
REM
REM No hardware and no ImGui: lights.cpp is a pure state machine, which
REM is what makes the docs/conventions.md rules checkable before any LED exists.

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
  /I"%HERE%..\..\shared" ^
  "%HERE%test_lights.cpp" ^
  "%HERE%..\src\lights.cpp" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_lights.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_lights.exe

if /I "%~1"=="run" (
  "%HERE%build\test_lights.exe"
  exit /b %errorlevel%
)
exit /b 0
