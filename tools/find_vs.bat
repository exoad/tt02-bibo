@echo off
REM Sets VSROOT to a Visual Studio with the C++ toolset.
REM
REM     call "<repo>\tools\find_vs.bat"       VSROOT only
REM     call "<repo>\tools\find_vs.bat" cl    also the x64 cl environment and CLFLAGS
REM     if errorlevel 1 exit /b 1
REM
REM The cl form is where tools\test.bat and viewer\build.bat get their MSVC flags.
REM vswhere finds any edition: GitHub runners carry Enterprise, not Community.
REM No setlocal: this sets variables in its caller.
REM goto, not an if block: the closing paren of x86 in the Program Files path ends the block early.

set "VSROOT="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto :onpath
REM VsDevCmd.bat runs a bare vswhere.exe from this directory, which does not
REM resolve when NoDefaultCurrentDirectoryInExePath is set. PATH makes it resolve.
set "PATH=%PATH%;%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
REM -requires makes a machine without the C++ workload report missing here.
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%i"
goto :fallback

:onpath
REM No installer: try a vswhere on PATH, as runner images have. Errors go to nul so :nope reports once.
for /f "usebackq tokens=*" %%i in (`vswhere.exe -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VSROOT=%%i"

:fallback
if defined VSROOT goto :check
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" set "VSROOT=C:\Program Files\Microsoft Visual Studio\2022\Community"

:check
if not defined VSROOT goto :nope
if not exist "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" goto :gone
if not "%~1"=="cl" exit /b 0
call "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul
if errorlevel 1 goto :novcvars
REM /MT: the programs run on machines with no redistributable installed.
set "CLFLAGS=/nologo /EHsc /MT /O2 /std:c++20 /W4 /D_CRT_SECURE_NO_WARNINGS"
exit /b 0

:novcvars
echo [error] vcvarsall.bat x64 failed in %VSROOT%
exit /b 1

:gone
set "VSROOT="

:nope
echo [error] no Visual Studio with the C++ toolset found.
echo         looked for vswhere at: %VSWHERE%
echo         Install "Desktop development with C++", or set VSROOT yourself.
exit /b 1
