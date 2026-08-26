@echo off
REM Throwaway preview harness for hub\src\board_view.cpp.
REM
REM   build.bat        compile + link
REM   build.bat run    compile + link + run
REM
REM Not part of the application build; hub\build.bat never sees this directory.

setlocal
set HERE=%~dp0
set HUB=%HERE%..\..
set IMGUI=%HUB%\third_party\imgui
set OUT=%HERE%build

call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 (
  echo [preview] vcvarsall failed
  exit /b 1
)

if not exist "%OUT%" mkdir "%OUT%"

cl /nologo /EHsc /MT /O2 /std:c++20 /W4 /D_CRT_SECURE_NO_WARNINGS ^
  /I"%HERE%..\..\..\shared" ^
  /I "%IMGUI%" /I "%IMGUI%\backends" /I "%HUB%\src" ^
  "%HERE%main.cpp" ^
  "%HUB%\src\board_view.cpp" ^
  "%HUB%\src\theme.cpp" ^
  "%IMGUI%\imgui.cpp" "%IMGUI%\imgui_draw.cpp" ^
  "%IMGUI%\imgui_tables.cpp" "%IMGUI%\imgui_widgets.cpp" ^
  "%IMGUI%\backends\imgui_impl_win32.cpp" ^
  "%IMGUI%\backends\imgui_impl_dx11.cpp" ^
  /Fo"%OUT%\\" /Fe"%OUT%\board_preview.exe" ^
  /link user32.lib gdi32.lib shell32.lib dwmapi.lib imm32.lib
if errorlevel 1 (
  echo [preview] build failed
  exit /b 1
)

echo [preview] built -^> %OUT%\board_preview.exe

if /i "%~1"=="run" (
  "%OUT%\board_preview.exe"
  exit /b %errorlevel%
)

exit /b 0
