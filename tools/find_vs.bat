@echo off
REM ===========================================================================
REM  find_vs.bat - where is Visual Studio?
REM
REM      call "<repo>\tools\find_vs.bat"
REM      if errorlevel 1 exit /b 1
REM      call "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" x64
REM
REM  Sets VSROOT to an installation that actually has the C++ toolset, and
REM  exits 1 with a message if there is not one.
REM
REM  WHY THIS EXISTS
REM
REM  Twenty-two scripts had this line, character for character:
REM
REM      set VS=C:\Program Files\Microsoft Visual Studio\2022\Community
REM
REM  which is true on the desk it was written on and on nothing else. Community
REM  is one of four editions; the others install to \Professional, \Enterprise
REM  and \BuildTools, VS can be installed anywhere, and GitHub's windows runners
REM  carry Enterprise - so every one of those scripts failed there with
REM  "vcvarsall.bat is not recognized", which reads like a broken build rather
REM  than a wrong path. The whole tree was unbuildable on any machine but one,
REM  and that fact was invisible because there was only ever one machine.
REM
REM  ONE COPY, not twenty-two. The same argument as lib/boot.hxx: three sketches
REM  each had their own copy of the eight lines every program started with, and
REM  the copies had already drifted before anybody noticed. A path repeated in
REM  twenty-two files is twenty-two chances to fix twenty-one of them.
REM
REM  vswhere.exe ships with every VS since 2017 and lives at a FIXED path under
REM  Program Files (x86) - that is its whole purpose, to be the one thing you do
REM  not have to search for. -latest picks the newest install and -requires
REM  makes it pick one with the C++ tools, so a machine carrying only the C#
REM  workload is reported as missing rather than found and then failing later
REM  with a stranger message.
REM
REM  The hard-coded path is kept as a FALLBACK, not as the answer. If vswhere is
REM  somehow absent the old behaviour is still there, and if that path does not
REM  exist either this says so plainly instead of letting `cl` be "not
REM  recognized" three steps further on.
REM
REM  NO setlocal IN THIS FILE. It exists to set a variable in its CALLER, and
REM  setlocal would discard it at exit - the failure would be VSROOT coming back
REM  empty from a script that reported success.
REM ===========================================================================

REM  GOTO RATHER THAN if (...) BLOCKS, and this is not a style choice. The path
REM  to vswhere contains `%ProgramFiles(x86)%`, and cmd expands variables while
REM  it PARSES a parenthesised block - so the `)` inside `(x86)` closes the
REM  block early and the script dies with "\Microsoft was unexpected at this
REM  time". Labels have no such problem.

set "VSROOT="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"

if not exist "%VSWHERE%" goto :fallback

for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%i"

:fallback
REM The desk this was written on, kept so nothing that worked stops working.
if defined VSROOT goto :check
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" set "VSROOT=C:\Program Files\Microsoft Visual Studio\2022\Community"

:check
REM Found a path is not the same as found a compiler. Check the file every
REM caller is about to run, so a wrong answer fails HERE with a reason.
if not defined VSROOT goto :nope
if exist "%VSROOT%\VC\Auxiliary\Build\vcvarsall.bat" exit /b 0
set "VSROOT="

:nope
echo [error] no Visual Studio with the C++ toolset found.
echo         looked for vswhere at: %VSWHERE%
echo         Install "Desktop development with C++", or set VSROOT yourself.
exit /b 1
