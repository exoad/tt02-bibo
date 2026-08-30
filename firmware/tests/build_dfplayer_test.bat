@echo off
REM Builds and (with "run") executes the lib/driversdfplayer_proto.hxx parser tests.
REM
REM   firmware\tests\build_text_test.bat        - compile only
REM   firmware\tests\build_text_test.bat run    - compile then run
REM
REM Compiled for the HOST with MSVC, not for the board. driversdfplayer_proto.hxx needs nothing
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
  "%HERE%test_dfplayer.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_dfplayer.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_dfplayer.exe

if /I "%~1"=="run" (
  "%HERE%build\test_dfplayer.exe"
  exit /b %errorlevel%
)
exit /b 0
