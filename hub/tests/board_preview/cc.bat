@echo off
setlocal
set ROOT=%~dp0..\..
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 exit /b 1
if not exist "%ROOT%\build\cc" mkdir "%ROOT%\build\cc"
cl /c /std:c++17 /EHsc /MT /W4 /I "%ROOT%\third_party\imgui" /I "%ROOT%\third_party\imgui\backends" /I "%ROOT%\src" "%ROOT%\src\board_view.cpp" /Fo:"%ROOT%\build\cc\\"
exit /b %errorlevel%
