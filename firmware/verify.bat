@echo off
setlocal EnableDelayedExpansion

REM  verify.bat - is firmware/ actually green?    usage: firmware\verify.bat
REM
REM  Exits 0 only when EVERY line below passed, so it can gate a commit or flash.
REM
REM  Shaped by four ways of verifying this tree that all REPORTED SUCCESS while
REM  measuring nothing:
REM    1. Warnings counted from a build that had already failed - the compiler
REM       never reached the file, so "0 warnings" was true and meaningless.
REM       => errors are counted FIRST; warnings are reported only when errors=0.
REM    2. Results read from printed text while the exit code was always 0:
REM       `exit /b %errorlevel%` inside an if-block expands when cmd PARSES the
REM       block, before the test runs. All eleven scripts had it. => check EXIT
REM       CODES, never output, and use an early return rather than a block.
REM    3. A count taken twice against a moving branch => the commit is pinned at
REM       the top and compared at the bottom.
REM    4. An incremental build reporting 0 warnings because it never recompiled
REM       the file that warns. => both boards are built CLEAN.

set HERE=%~dp0
set ROOT=%HERE%..
set FAIL=0

for /f "delims=" %%s in ('git -C "%ROOT%" rev-parse HEAD 2^>nul') do set PINNED=%%s
if "%PINNED%"=="" set PINNED=(not a git checkout)

echo.
echo   bibo firmware verify
echo   commit %PINNED%
echo   ---------------------------------------------------------------

REM ---- 1. both boards, CLEAN, one capture each ------------------------------
REM Clean because an incremental build skips untouched files, so a warning in one
REM is invisible - how a raw NUL byte in main.cxx survived "0 warnings" builds.
call :board pico2_w ""
call :board pico2   "pico2"

REM ---- 2. the host tests, by EXIT CODE --------------------------------------
call :suite text     "%HERE%tests\build_text_test.bat"
call :suite pins     "%HERE%tests\build_pins_test.bat"
call :suite dfplayer "%HERE%tests\build_dfplayer_test.bat"
call :suite chassis  "%HERE%tests\build_chassis_test.bat"

REM control, pursuit and sfx were WRITTEN AND NEVER GATED - 71 checks sitting in
REM tests\ that nothing ran, so the day one broke the gate would still say PASS.
REM A test suite that is not in this list is a suite that does not exist.
call :suite control  "%HERE%tests\build_control_test.bat"
call :suite pursuit  "%HERE%tests\build_pursuit_test.bat"
call :suite sfx      "%HERE%tests\build_sfx_test.bat"

REM ---- 3. the style audit --------------------------------------------------
python "%ROOT%\hub\tools\style_audit.py" >nul 2>&1
if errorlevel 1 (
  echo   [FAIL] audit          violations - run hub\tools\style_audit.py
  set FAIL=1
) else (
  echo   [ ok ] audit          0 violations
)

REM ---- 4. did the tree move under us? --------------------------------------
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
REM
REM  find and findstr are called by ABSOLUTE PATH: with git's usr/bin ahead of
REM  System32, `find /c /v ""` reaches GNU find, which takes /c as a PATH and
REM  walks the whole C: drive. Same exposure for findstr, sort and more.
REM
REM  One capture. Errors decide pass/fail; the warning count prints only when
REM  there are no errors, since one from a failed build just measures how far
REM  the compiler got.
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

REM  :suite <name> <script>
REM
REM  Reads the EXIT CODE, not the output: until 2026-08-31 every one of these
REM  scripts printed OVERALL: FAIL while exiting 0.
:suite
call %2 run >nul 2>&1
if errorlevel 1 (
  echo   [FAIL] suite %~1     exit 1
  set FAIL=1
  exit /b 0
)
echo   [ ok ] suite %~1     exit 0
exit /b 0
