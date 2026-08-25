@echo off
REM Builds and (with "run") executes the PicoLink hardware test.
REM
REM   tests\build_pico_test.bat        - compile only
REM   tests\build_pico_test.bat run    - compile then run against COM10
REM
REM Only pico_link.cpp is linked in; the test pulls in nothing from the viewer.
REM The port name is hardcoded in test_pico_link.cpp rather than passed on the
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

cl /nologo /EHsc /O2 /MT /W4 /std:c++17 ^
  /I "%SRC%" ^
  "%HERE%test_pico_link.cpp" ^
  "%SRC%\pico_link.cpp" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_pico_link.exe" ^
  /link kernel32.lib advapi32.lib setupapi.lib
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
