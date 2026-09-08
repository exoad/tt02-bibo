@echo off
setlocal EnableDelayedExpansion

REM  verify.bat - is hub/ actually green?    usage: hub\verify.bat
REM
REM  The other half of firmware\verify.bat. That one answers for the boards; the
REM  hub had twelve test scripts in tests\ and nothing that ran them, so "the hub
REM  is green" meant whichever few someone remembered that day. A suite that is
REM  not in the list below is a suite that does not exist.
REM
REM  Exits 0 only when every line passed, so it can gate a commit.
REM
REM  The same four ways of measuring nothing that shaped firmware\verify.bat
REM  apply here, and the same answers:
REM    1. Warnings counted from a build that had already failed. => errors decide
REM       first; the warning count is only reported when the build succeeded.
REM    2. Results read from printed text while the exit code was always 0.
REM       => EXIT CODES, never output, and an early return rather than an
REM       if-block, since exit /b %errorlevel% inside a block expands when cmd
REM       PARSES the block, before the test has run.
REM    3. A count taken twice against a moving branch. => the commit is pinned
REM       at the top and compared at the bottom.
REM    4. An incremental build reporting 0 warnings because it never recompiled
REM       the file that warns. => the build here is CLEAN.
REM
REM  NO BACKTICKS IN THESE COMMENTS. cmd tokenises a REM line, and a backtick
REM  pair makes the for /f over git below try to run a command named after it.
REM
REM  Expect this to take several minutes: a clean hub build plus twelve
REM  compiles. That is the price of a gate that means something.

set HERE=%~dp0
set ROOT=%HERE%..
set FAIL=0

for /f "delims=" %%s in ('git -C "%ROOT%" rev-parse HEAD 2^>nul') do set PINNED=%%s
if "%PINNED%"=="" set PINNED=(not a git checkout)

echo.
echo   bibo hub verify
echo   commit %PINNED%
echo   ---------------------------------------------------------------

REM ---- 1. the application, CLEAN -------------------------------------------
REM Clean because an incremental build skips untouched files, so a warning in
REM one of them is invisible - the same way a NUL byte survived "0 warnings"
REM builds on the firmware side.
call :build

REM ---- 2. the host suites, by EXIT CODE ------------------------------------
call :suite devlink     "%HERE%tests\build_devlink_test.bat"     ""
call :suite diagnostics "%HERE%tests\build_diagnostics_test.bat" ""
call :suite editor      "%HERE%tests\build_editor_test.bat"      ""
call :suite face        "%HERE%tests\build_face_test.bat"        ""
call :suite geom        "%HERE%tests\build_geom_test.bat"        ""
call :suite json        "%HERE%tests\build_json_test.bat"        ""
call :suite lidarnet    "%HERE%tests\build_lidarnet_test.bat"    ""
call :suite lights      "%HERE%tests\build_lights_test.bat"      ""
call :suite lint        "%HERE%tests\build_lint_test.bat"        ""
call :suite lsp         "%HERE%tests\build_lsp_test.bat"         ""

REM The Pico suite's sections 1 and 2 need a board on COM10; its "host" argument
REM stops after section 0, which is the part that holds with nothing plugged in.
REM Gated in that mode rather than skipped, because the send accounting it
REM checks there is exactly the kind of arithmetic that breaks unnoticed.
call :suite pico        "%HERE%tests\build_pico_test.bat"        "host"

REM ---- 3. the suite that CANNOT run without hardware -----------------------
REM tests\build_test.bat drives a real RPLIDAR C1 on COM7 and has no host mode.
REM Compiling it is still a gate: it links lidar_source.cxx, and that file has
REM changed under it twice this month. Running it is a bench job, not a commit
REM gate, and saying so is better than a green line that measured nothing.
call :compileonly lidar "%HERE%tests\build_test.bat"

REM ---- 4. style ------------------------------------------------------------
python "%ROOT%\hub\tools\style_audit.py" >nul 2>&1
if errorlevel 1 (
  echo   [FAIL] audit          violations - run hub\tools\style_audit.py
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

REM ---- 5. did the tree move under us? --------------------------------------
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

REM  :build - the hub, clean, one capture.
REM
REM  find and findstr are called by ABSOLUTE PATH: with git's usr/bin ahead of
REM  System32, find /c /v "" reaches GNU find, which takes /c as a path and walks
REM  the whole C: drive.
:build
set OUT=%TEMP%\bibo-hub-verify-build.txt
call "%HERE%build.bat" clean > "%OUT%" 2>&1
if errorlevel 1 (
  echo   [FAIL] bibo.exe       build failed - see %OUT%
  set FAIL=1
  exit /b 0
)
set WARNS=0
for /f %%c in ('%SystemRoot%\System32\findstr.exe /c:": warning" "%OUT%" ^| %SystemRoot%\System32\find.exe /c /v ""') do set WARNS=%%c
if not "%WARNS%"=="0" (
  echo   [FAIL] bibo.exe       %WARNS% warning^(s^) - see %OUT%
  set FAIL=1
  exit /b 0
)
echo   [ ok ] bibo.exe       0 errors, 0 warnings
exit /b 0

REM  :suite <name> <script> <extra argument before run>
REM
REM  Reads the EXIT CODE, not the output: every one of these scripts printed
REM  OVERALL: FAIL while exiting 0 until the trap in the header was fixed.
:suite
call %2 %~3 run >nul 2>&1
if errorlevel 1 (
  echo   [FAIL] suite %~1     exit 1
  set FAIL=1
  exit /b 0
)
echo   [ ok ] suite %~1     exit 0
exit /b 0

REM  :compileonly <name> <script>
REM
REM  Without "run" these scripts compile and stop. The line says NEEDS HARDWARE
REM  rather than ok, so nobody reads it as "the lidar path is tested".
:compileonly
call %2 >nul 2>&1
if errorlevel 1 (
  echo   [FAIL] suite %~1     does not compile
  set FAIL=1
  exit /b 0
)
echo   [ -- ] suite %~1     compiles; running it needs a C1 on COM7
exit /b 0
