@echo off
REM Builds and (with "run") executes the PicoLink hardware test.
REM
REM   tests\build_pico_test.bat        - compile only
REM   tests\build_pico_test.bat run    - compile then run against COM10
REM
REM pico_link.cxx and the one thing it calls - dev::describe, which turns a lost
REM link into a sentence. Nothing else from the viewer. This test would not link
REM at all for a while: devlink.cxx was added and this script was not told, so
REM the whole suite failed on two unresolved symbols rather than on anything it
REM was testing.
REM The port name is hardcoded in test_pico_link.cxx rather than passed on the
REM command line, because Git Bash mangles the \\.\ device prefix.
REM
REM /MT matches the rest of this project (the rplidar driver lib is static-CRT).

setlocal
set HERE=%~dp0
set SRC=%HERE%..\src
set VS=C:\Program Files\Microsoft Visual Studio\2022\Community

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 ^
  /I"%HERE%..\..\shared" ^
  /I "%SRC%" ^
  "%HERE%test_pico_link.cxx" ^
  "%SRC%\pico_link.cxx" ^
  "%SRC%\devlink.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_pico_link.exe" ^
  /link kernel32.lib advapi32.lib setupapi.lib ws2_32.lib
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [test] built -^> %HERE%build\test_pico_link.exe

if /i "%~1"=="run" (
  echo.
  "%HERE%build\test_pico_link.exe"
  exit /b %errorlevel%
)

exit /b 0
