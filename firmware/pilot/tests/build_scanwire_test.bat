@echo off
REM Builds and (with "run") executes the scan feed wire format tests.
REM   tests\build_scanwire_test.bat [run]   - compile, optionally run
REM No hardware, no socket and no Orange Pi: scanwire.cxx is pure string work,
REM the same object file the board's scanfeed and the hub's lidar source both
REM compile, so it is proved here for both ends at once.

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
  "%HERE%test_scanwire.cxx" ^
  "%HERE%..\src\scanwire.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_scanwire.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_scanwire.exe

REM An EARLY RETURN, not an if-block - see build_proto_test.bat for why.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_scanwire.exe"
exit /b %errorlevel%
