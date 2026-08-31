@echo off
REM Builds and (with "run") executes the clangd client tests.
REM
REM   tests\build_lsp_test.bat        - compile only
REM   tests\build_lsp_test.bat run    - compile then run
REM
REM Talks to the REAL clangd and needs firmware\build\compile_commands.json.
REM The test SKIPS (exit 0) when either is absent - a missing toolchain is a
REM fact about the machine, not a defect in the code.
REM
REM No ImGui. lsp.cxx is Win32 and nothing else, which is what makes this
REM runnable from a console at all.

setlocal
set HERE=%~dp0
set VS=C:\Program Files\Microsoft Visual Studio\2022\Community

call "%VS%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [test] vcvarsall failed
  exit /b 1
)

if not exist "%HERE%build" mkdir "%HERE%build"
if not exist "%HERE%build\lsp" mkdir "%HERE%build\lsp"

cl /nologo /EHsc /O2 /MT /W4 /std:c++20 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\..\shared" /I"%HERE%..\src" ^
  "%HERE%test_lsp.cxx" ^
  "%HERE%..\src\lsp.cxx" ^
  "%HERE%..\src\json.cxx" ^
  "%HERE%..\src\complete.cxx" ^
  /Fo"%HERE%build\lsp\\" ^
  /Fe"%HERE%build\test_lsp.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_lsp.exe

REM An EARLY RETURN, not an if-block - see build_json_test.bat for why.
if /i not "%~1"=="run" exit /b 0
"%HERE%build\test_lsp.exe"
exit /b %errorlevel%
