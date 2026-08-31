@echo off
REM Builds and (with "run") executes the lib/text.hxx parser tests.
REM
REM   firmware\tests\build_text_test.bat        - compile only
REM   firmware\tests\build_text_test.bat run    - compile then run
REM
REM Compiled for the HOST with MSVC, not for the board. text.hxx needs nothing
REM from the Pico SDK - only lib/types.hxx - and that is a property worth keeping:
REM a parser that can only be exercised by flashing a microcontroller is a
REM parser nobody exercises.
REM
REM forces C. These are .c files and MSVC decides the language from the
REM extension, but saying so keeps a rename from silently compiling C as C++,
REM where several things here mean something subtly different.

setlocal
set HERE=%~dp0
set VS=C:\Program Files\Microsoft Visual Studio\2022\Community

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"

cl /nologo /O2 /MT /W4 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\lib" ^
  "%HERE%test_sfx.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_sfx.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_sfx.exe

REM An EARLY RETURN, not an if-block, and the reason is a cmd.exe
REM parsing rule that cost every one of these scripts its exit code:
REM `exit /b %errorlevel%` inside `if ... ( ... )` is expanded when
REM cmd parses the BLOCK, which is before the test has run. All 11
REM scripts exited 0 while printing OVERALL: FAIL, so not one of them
REM could ever have gated a commit. Out here the expansion happens
REM when the line is reached.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_sfx.exe"
exit /b %errorlevel%
