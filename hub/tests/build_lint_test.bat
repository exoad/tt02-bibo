@echo off
REM Builds and (with "run") executes the live style linter tests.
REM   tests\build_lint_test.bat [run]   - compile, optionally run
REM No hardware and no ImGui: lint.cxx is a pure state machine, which makes the
REM docs/conventions.md rules checkable.

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

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\..\shared" /I"%HERE%..\src" ^
  "%HERE%test_lint.cxx" ^
  "%HERE%..\src\lint.cxx" ^
  "%HERE%..\src\diagnostics.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_lint.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_lint.exe

REM An EARLY RETURN, not an if-block: `exit /b %errorlevel%` inside
REM `if ... ( ... )` expands when cmd PARSES the block, before the test has run.
REM All 11 scripts once exited 0 while printing OVERALL: FAIL.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_lint.exe"
exit /b %errorlevel%
