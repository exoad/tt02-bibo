@echo off
REM Builds and (with "run") executes the line-protocol tests.
REM
REM   tests\build_proto_test.bat        - compile only
REM   tests\build_proto_test.bat run    - compile then run
REM
REM No hardware, no serial port and no Orange Pi. proto.cxx is pure string work
REM precisely so it can be finished and proved before the board it is for
REM exists - which is the whole argument for writing this layer first.

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
  /I"%HERE%..\..\shared" /I"%HERE%..\src" ^
  "%HERE%test_proto.cxx" ^
  "%HERE%..\src\proto.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_proto.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_proto.exe

REM An EARLY RETURN, not an if-block, and the reason is a cmd.exe parsing
REM rule that cost every one of these scripts its exit code: `exit /b
REM %errorlevel%` inside `if ... ( ... )` is expanded when cmd parses the
REM BLOCK, which is before the test has run. Out here the expansion happens
REM when the line is reached.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_proto.exe"
exit /b %errorlevel%
