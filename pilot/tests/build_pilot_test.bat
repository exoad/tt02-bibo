@echo off
REM Builds and (with "run") executes the link and autonomy stub tests.
REM
REM   tests\build_pilot_test.bat        - compile only
REM   tests\build_pilot_test.bat run    - compile then run
REM
REM Compiles firmware/lib's pure headers into a program that is NOT firmware,
REM which is the point: geom, kinematics, pursuit and control each claim in
REM their own comments to build for the Pico, the Orange Pi and the host, and
REM nothing held them to it until this test existed.

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
if not exist "%HERE%build\pilot" mkdir "%HERE%build\pilot"

REM /wd4505 - "unreferenced function with internal linkage has been removed".
REM
REM firmware/lib's headers define their functions `static`, so every translation
REM unit gets a private copy and the compiler reports every one it did not use.
REM A consumer that uses a SUBSET of a header therefore always trips this: 48
REM warnings here, all of them this, none of them about anything wrong.
REM
REM Suppressed rather than worked around, and named rather than hidden behind a
REM blanket /W3. The underlying fix is `inline` instead of `static` in those
REM headers - which is what C++ has for header-only functions and what would let
REM the Pico share one copy too - but that is a change to firmware/lib and not
REM one to make from the pilot's build script.
cl /nologo /EHsc /O2 /MT /W4 /wd4505 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\..\shared" ^
  /I"%HERE%..\src" ^
  /I"%HERE%..\..\firmware\lib" ^
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

REM An EARLY RETURN, not an if-block - see build_proto_test.bat for the cmd.exe
REM parsing rule that cost every one of these scripts its exit code.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_pilot.exe"
exit /b %errorlevel%
