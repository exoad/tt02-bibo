@echo off
REM Builds and (with "run") executes the editor + syntax tests.
REM
REM   tests\build_editor_test.bat        - compile only
REM   tests\build_editor_test.bat run    - compile then run
REM
REM editor.cxx is ImGui-free by design, so the vim bindings, the auto-closing
REM braces and the auto-indent are all checkable without a window. syntax.cxx
REM includes imgui.h only for IM_COL32, hence the include path below.

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
  /I"%HERE%..\..\shared" ^
  /I"%HERE%..\third_party\imgui" ^
  "%HERE%test_editor.cxx" ^
  "%HERE%..\src\editor.cxx" ^
  "%HERE%..\src\syntax.cxx" ^
  "%HERE%..\src\complete.cxx" ^
  /Fo"%HERE%build\\" ^
  /Fe"%HERE%build\test_editor.exe" ^
  /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [test] compile failed
  exit /b 1
)

echo [ok] %HERE%build\test_editor.exe

if /I "%~1"=="run" (
  "%HERE%build\test_editor.exe"
  exit /b %errorlevel%
)
exit /b 0
