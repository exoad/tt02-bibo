@echo off
setlocal EnableDelayedExpansion

REM  verify.bat - is firmware/ actually green?    usage: firmware\verify.bat
REM
REM  Exits 0 only when EVERY line below passed, so it can gate a commit or flash.
REM
REM  NO BACKTICKS IN REM LINES. cmd still tokenises them, and a backtick pair in
REM  one once broke the for /f over git further down.

set HERE=%~dp0
set ROOT=%HERE%..
set FAIL=0

for /f "delims=" %%s in ('git -C "%ROOT%" rev-parse HEAD 2^>nul') do set PINNED=%%s
if "%PINNED%"=="" set PINNED=(not a git checkout)

echo.
echo   bibo firmware verify
echo   commit %PINNED%
echo   ---------------------------------------------------------------

REM Both boards CLEAN: an incremental build skips untouched files, so a warning in
REM one is invisible.
call :board pico2_w ""
call :board pico2   "pico2"

REM Every suite tools\test.bat knows. A suite missing from this list is one nothing
REM runs, and the gate still says PASS the day it breaks.
for %%s in (text pins hall chassis encoder) do call :suite %%s
for %%s in (proto pilot reactive bibowire trimfile carrules car link) do call :suite %%s

python "%ROOT%\tools\style_audit.py" >nul 2>&1
if errorlevel 1 (
  echo   [FAIL] audit          violations - run tools\style_audit.py
  set FAIL=1
) else (
  echo   [ ok ] audit          0 violations
)

python "%ROOT%\tools\format.py" >nul 2>&1
if errorlevel 1 (
  echo   [FAIL] format         run tools\format.py to see it, --apply to fix it
  set FAIL=1
) else (
  echo   [ ok ] format         0 violations
)

for /f "delims=" %%s in ('git -C "%ROOT%" rev-parse HEAD 2^>nul') do set NOW=%%s
if not "%NOW%"=="%PINNED%" (
  echo   [FAIL] commit moved during the run - results describe %PINNED%
  set FAIL=1
)

echo   ---------------------------------------------------------------
if "%FAIL%"=="1" (
  echo   FAIL
  exit /b 1
)
echo   PASS
exit /b 0

REM  :board <label> <build.bat argument>
REM  find and findstr are called by ABSOLUTE PATH: with git's usr/bin ahead of
REM  System32, find reaches GNU find, which takes /c as a path and walks the whole
REM  C: drive. Errors are counted FIRST: a warning count from a failed build only
REM  measures how far the compiler got.
:board
set LABEL=%~1
set OUT=%TEMP%\bibo-verify-%LABEL%.txt
call "%HERE%build.bat" %~2 clean > "%OUT%" 2>&1

set ERRS=0
set WARNS=0
set IMGS=0
for /f %%c in ('%SystemRoot%\System32\findstr.exe /c:"error:" "%OUT%" ^| %SystemRoot%\System32\find.exe /c /v ""') do set ERRS=%%c
for /f %%c in ('%SystemRoot%\System32\findstr.exe /c:"warning:" "%OUT%" ^| %SystemRoot%\System32\find.exe /c /v ""') do set WARNS=%%c
for /f %%c in ('%SystemRoot%\System32\findstr.exe /b /c:"[ok]" "%OUT%" ^| %SystemRoot%\System32\find.exe /c /v ""') do set IMGS=%%c

if not "%ERRS%"=="0" (
  echo   [FAIL] %LABEL%  %ERRS% error^(s^) - see %OUT%
  set FAIL=1
  exit /b 0
)
if not "%WARNS%"=="0" (
  echo   [FAIL] %LABEL%  %WARNS% warning^(s^), %IMGS% image^(s^) - see %OUT%
  set FAIL=1
  exit /b 0
)
echo   [ ok ] %LABEL%  0 errors, 0 warnings, %IMGS% image^(s^)
exit /b 0

REM  :suite <name>
REM  Reads the EXIT CODE, never the output: a script can print FAIL and exit 0.
REM  errorlevel is copied to RC outside any block, because a variable inside an
REM  if-block expands when cmd parses the block, before the call has run.
:suite
call "%ROOT%\tools\test.bat" %1 run >nul 2>&1
set RC=%errorlevel%
if not "%RC%"=="0" (
  echo   [FAIL] suite %~1     exit %RC% - tools\test.bat %~1 run
  set FAIL=1
  exit /b 0
)
echo   [ ok ] suite %~1     exit 0
exit /b 0
