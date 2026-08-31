@echo off
REM Builds and (with "run") executes the device-loss classification tests.
REM
REM   tests\build_devlink_test.bat        - compile only
REM   tests\build_devlink_test.bat run    - compile then run
REM
REM No ImGui and no hardware, but this one DOES talk to Windows: the whole point
REM of the rule is that it agrees with what the machine reports, so the port
REM enumeration is exercised for real rather than mocked.

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
  "%HERE%test_devlink.cxx" ^
  "%HERE%..\src\devlink.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_devlink.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_devlink.exe

REM An EARLY RETURN, not an if-block, and the reason is a cmd.exe
REM parsing rule that cost every one of these scripts its exit code:
REM `exit /b %errorlevel%` inside `if ... ( ... )` is expanded when
REM cmd parses the BLOCK, which is before the test has run. All 11
REM scripts exited 0 while printing OVERALL: FAIL, so not one of them
REM could ever have gated a commit. Out here the expansion happens
REM when the line is reached.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_devlink.exe"
exit /b %errorlevel%
