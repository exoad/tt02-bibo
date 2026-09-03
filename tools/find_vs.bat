@echo off
REM find_vs.bat - sets VSROOT to a Visual Studio with the C++ toolset.
REM
REM     call "<repo>\tools\find_vs.bat"
REM     if errorlevel 1 exit /b 1
REM     call "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" x64
REM
REM Twenty-two scripts hardcoded ...\2022\Community. GitHub's runners carry
REM Enterprise, so all of them failed there with "vcvarsall.bat is not
REM recognized" - a wrong path wearing the costume of a broken build.
REM
REM No setlocal: this sets a variable in its CALLER.
REM goto, not if(...): the path holds %ProgramFiles(x86)%, and the `)` in (x86)
REM closes a parenthesised block early - "\Microsoft was unexpected at this time".

set "VSROOT="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"

if not exist "%VSWHERE%" goto :onpath

REM vcvarsall's VsDevCmd.bat reads its banner version by pushd-ing into this
REM directory and running a bare vswhere.exe - which only resolves if cmd
REM searches the current directory, and NoDefaultCurrentDirectoryInExePath turns
REM that off. Result: "'vswhere.exe' is not recognized" on every build, followed
REM by a build that worked. Putting the directory on PATH makes the bare name
REM resolve; no setlocal here, so this reaches vcvarsall in the caller.
set "PATH=%PATH%;%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"

REM -requires, so a machine with only the C# workload reports missing here
REM rather than failing later with a stranger message.
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%i"
goto :fallback

:onpath
REM No installer copy - a vswhere on PATH (a runner image) is the next best
REM thing, and if there is none either, stay quiet: :nope reports it once,
REM instead of cmd's not-recognized line on top.
for /f "usebackq tokens=*" %%i in (`vswhere.exe -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VSROOT=%%i"

:fallback
if defined VSROOT goto :check
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" set "VSROOT=C:\Program Files\Microsoft Visual Studio\2022\Community"

:check
REM A path is not a compiler. Check the file every caller is about to run.
if not defined VSROOT goto :nope
if exist "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" exit /b 0
set "VSROOT="

:nope
echo [error] no Visual Studio with the C++ toolset found.
echo         looked for vswhere at: %VSWHERE%
echo         Install "Desktop development with C++", or set VSROOT yourself.
exit /b 1
