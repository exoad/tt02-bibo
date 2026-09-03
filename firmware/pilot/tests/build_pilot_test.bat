@echo off
REM Builds and (with "run") executes the link and autonomy stub tests.
REM   tests\build_pilot_test.bat [run]   - compile, optionally run
REM Compiles firmware/lib's pure headers into a program that is NOT firmware,
REM which is the point: geom, kinematics, pursuit and control all claim to build
REM for the Pico, the Orange Pi and the host, and nothing else holds them to it.

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
if not exist "%HERE%build\pilot" mkdir "%HERE%build\pilot"

REM /wd4505 - "unreferenced function with internal linkage has been removed".
REM firmware/lib's headers define their functions `static`, so any consumer using
REM a SUBSET of a header trips this: 48 warnings here, none about anything wrong.
REM Named rather than hidden behind a blanket /W3. The real fix is `inline` in
REM those headers, but that is a change to firmware/lib, not to this script.
cl /nologo /EHsc /O2 /MT /W4 /wd4505 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\..\..\shared" ^
  /I"%HERE%..\src" ^
  /I"%HERE%..\..\lib" ^
  "%HERE%test_pilot.cxx" ^
  "%HERE%..\src\autonomy.cxx" ^
  "%HERE%..\src\link.cxx" ^
  /Fo"%HERE%build\pilot\\" ^
  /Fe"%HERE%build\test_pilot.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_pilot.exe

REM An EARLY RETURN, not an if-block - see build_proto_test.bat for why.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_pilot.exe"
exit /b %errorlevel%
