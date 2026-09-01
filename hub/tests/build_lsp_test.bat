@echo off
REM Builds and (with "run") executes the clangd client tests.
REM   tests\build_lsp_test.bat [run]   - compile, optionally run
REM Talks to the REAL clangd and needs firmware\build\compile_commands.json; the
REM test SKIPS (exit 0) when either is absent - a missing toolchain is a fact
REM about the machine, not a defect. No ImGui: lsp.cxx is Win32 and nothing else.

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
