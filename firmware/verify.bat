@echo off
setlocal EnableDelayedExpansion

REM ===========================================================================
REM  verify.bat - is firmware/ actually green?
REM
REM    firmware\verify.bat
REM
REM  Exits 0 only when EVERY line below passed. Exits 1 otherwise, so it can
REM  gate a commit or a flash.
REM
REM  WHY THIS EXISTS, and why it is shaped the way it is.
REM
REM  Verifying this tree by hand went wrong four separate ways in one day, and
REM  every one of them REPORTED SUCCESS while measuring nothing:
REM
REM    1. A warning count taken from a build that had already failed. The
REM       compiler never reached the file, so "0 warnings" was true and
REM       meaningless. => errors are counted FIRST and warnings are only
REM       reported when errors is 0.
REM
REM    2. Test results read from printed text while the script's exit code was
REM       always 0 - `exit /b %errorlevel%` inside an if-block expands when cmd
REM       PARSES the block, before the test runs. All eleven scripts had it.
REM       => this file checks EXIT CODES, never output, and uses an early
REM       return rather than a block.
REM
REM    3. A count taken twice against a moving branch, answering about two
REM       different objects. => the commit is pinned once at the top and
REM       compared at the bottom.
REM
REM    4. An incremental build reporting 0 warnings because it had not
REM       recompiled the file that warns. => both boards are built CLEAN.
REM
REM  The rule underneath all four: measure the artefact, not the name of it,
REM  and prefer a check that fails loudly to one that passes quietly.
REM ===========================================================================

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
REM
REM Clean because an incremental build only recompiles what changed, so a
REM warning in an untouched file is invisible - which is exactly how a raw NUL
REM byte in main.cxx survived a run of "0 warnings" builds.
call :board pico2_w ""
call :board pico2   "pico2"

REM ---- 2. the host tests, by EXIT CODE --------------------------------------
call :suite text     "%HERE%tests\build_text_test.bat"
call :suite pins     "%HERE%tests\build_pins_test.bat"
call :suite dfplayer "%HERE%tests\build_dfplayer_test.bat"
call :suite chassis  "%HERE%tests\build_chassis_test.bat"

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

REM ===========================================================================
REM  :board <label> <build.bat argument>
REM
REM  find and findstr are called by ABSOLUTE PATH. `find /c /v ""` is the
REM  standard cmd line-count idiom and it is a loaded gun in a tree whose
REM  shell puts git's usr/bin ahead of System32: GNU find took /c as a PATH
REM  and started walking the whole C: drive. Same exposure for findstr,
REM  sort and more.
REM
REM  One capture. Errors decide pass or fail; the warning count is only printed
REM  when there are no errors, because a warning count from a failed build is
REM  a count of how far the compiler got.
REM ===========================================================================
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

REM ===========================================================================
REM  :suite <name> <script>
REM
REM  Runs it and reads the EXIT CODE. Not the output: every one of these
REM  scripts printed OVERALL: FAIL while exiting 0 until 2026-08-31, so any
REM  check that read the text agreed with a check that read nothing.
REM ===========================================================================
:suite
call %2 run >nul 2>&1
if errorlevel 1 (
  echo   [FAIL] suite %~1     exit 1
  set FAIL=1
  exit /b 0
)
echo   [ ok ] suite %~1     exit 0
exit /b 0
