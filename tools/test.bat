@echo off
setlocal

REM test.bat - builds one host test suite with MSVC and, given run, runs it.
REM
REM     tools\test.bat <suite> [run]
REM
REM Exits non-zero when the suite does not build or does not pass. Every check is
REM an early return on its own line: exit /b %errorlevel% inside an if-block
REM expands when cmd PARSES the block, before the test has run, which is how the
REM per-suite scripts this replaced once printed OVERALL: FAIL and exited 0.
REM
REM Each suite builds into build\<suite>\ beside its test file, so no two suites
REM share an object file.

set "ROOT=%~dp0.."
set "SUITE=%~1"
set "LIB=%ROOT%\firmware\lib"
set "PILOT=%ROOT%\firmware\pilot\src"
set "EXTRA="
set "LIBS="

set "FIRMWARE_SUITES=text pins dfplayer chassis control pursuit sfx"
set "PILOT_SUITES=proto pilot reactive scanwire bibowire trimfile"

for %%s in (%FIRMWARE_SUITES%) do if "%SUITE%"=="%%s" goto :firmware
for %%s in (%PILOT_SUITES%) do if "%SUITE%"=="%%s" goto :pilot
if "%SUITE%"=="link" goto :viewer

echo [error] no suite named "%SUITE%"
echo         usage: tools\test.bat ^<suite^> [run]
echo         suites: %FIRMWARE_SUITES% %PILOT_SUITES% link
exit /b 2

REM firmware\lib's host-buildable headers. Only chassis reaches into hal.hxx, and
REM BIBO_FAKE_HAL swaps its SDK half for firmware\tests\fakes\hal.hxx.
:firmware
set "TESTS=%ROOT%\firmware\tests"
set "INC=/I"%LIB%""
set "SRCS="%TESTS%\test_%SUITE%.cxx""
if "%SUITE%"=="chassis" set "EXTRA=/DBIBO_FAKE_HAL"
goto :build

REM One pilot module and its test. pilot is the exception: the refusing halves of
REM lidar.cxx and link.cxx, plus firmware\lib's pure headers - which define their
REM functions static, so /wd4505 for every one this program does not call.
:pilot
set "TESTS=%ROOT%\firmware\pilot\tests"
set "INC=/I"%ROOT%\shared" /I"%PILOT%""
set "SRCS="%TESTS%\test_%SUITE%.cxx" "%PILOT%\%SUITE%.cxx""
if not "%SUITE%"=="pilot" goto :build
set "INC=%INC% /I"%LIB%""
set "SRCS="%TESTS%\test_pilot.cxx" "%PILOT%\lidar.cxx" "%PILOT%\link.cxx""
set "EXTRA=/wd4505"
goto :build

REM viewer\src BEFORE firmware\pilot\src: both have a link.hxx, and this suite
REM means the viewer's. ws2_32.lib because link.cxx holds the socket half too.
:viewer
set "TESTS=%ROOT%\viewer\tests"
set "VSRC=%ROOT%\viewer\src"
set "IMGUI=%ROOT%\third_party\imgui"
set "STB=%ROOT%\third_party\stb"
if not exist "%IMGUI%\imgui.h" goto :nothirdparty
if not exist "%STB%\stb_image.h" goto :nothirdparty
set "INC=/I"%VSRC%" /I"%ROOT%\shared" /I"%PILOT%" /I"%IMGUI%" /I"%STB%""
set "SRCS="%TESTS%\test_link.cxx" "%VSRC%\link.cxx" "%VSRC%\vlog.cxx" "%VSRC%\jpeg.cxx""
set "SRCS=%SRCS% "%VSRC%\orient.cxx" "%VSRC%\settings.cxx" "%PILOT%\bibowire.cxx""
set "LIBS=ws2_32.lib"
goto :build

:build
set "OUT=%TESTS%\build\%SUITE%"
call "%~dp0find_vs.bat" cl
if errorlevel 1 exit /b 1
if not exist "%OUT%" mkdir "%OUT%"

cl %CLFLAGS% %EXTRA% %INC% %SRCS% /Fo"%OUT%\\" /Fe"%OUT%\test_%SUITE%.exe" /link /SUBSYSTEM:CONSOLE %LIBS%
if errorlevel 1 goto :nobuild
echo [ok] %OUT%\test_%SUITE%.exe

if not "%~2"=="run" exit /b 0
"%OUT%\test_%SUITE%.exe"
exit /b %errorlevel%

:nobuild
echo [error] suite %SUITE% did not build
exit /b 1

:nothirdparty
echo [error] third_party\imgui or third_party\stb is missing - both are cloned, see THIRD_PARTY.md
echo         git clone --depth 1 --branch v1.92.9 https://github.com/ocornut/imgui third_party\imgui
echo         git clone --depth 1 https://github.com/nothings/stb third_party\stb
exit /b 1
