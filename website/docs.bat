@echo off
REM Open the firmware documentation in the default browser.
REM
REM This is what the hub's Docs button runs. It is a batch file rather than
REM three ShellExecute calls in app_ui.cxx so that the same thing happens
REM whether you press the button or run it from a shell.
REM
REM Rebuilds when there is nothing to serve OR when firmware/lib has changed
REM since the last build. It used to rebuild only in the first case, so the
REM press after editing a header served the PREVIOUS reference and said
REM nothing about it - a docs site quietly showing yesterday's signatures is
REM worse than a missing one, because a missing one sends you to the header.
REM
REM stale.mjs answers that question and exits 0 for "rebuild"; a full rebuild
REM on every press would put twenty seconds behind a button meant to open a
REM page.

setlocal
cd /d "%~dp0"

set PORT=4173

set NEEDS_BUILD=
if not exist ".output\public\index.html" set NEEDS_BUILD=1
if not defined NEEDS_BUILD (
    node stale.mjs >nul 2>&1
    if not errorlevel 1 set NEEDS_BUILD=1
)

if defined NEEDS_BUILD (
    echo Building the documentation, one moment...
    if not exist "node_modules" (
        call npm install --no-audit --no-fund
        if errorlevel 1 (
            echo.
            echo npm install failed. Node is required to build the docs.
            exit /b 1
        )
    )
    call npm run generate
    if errorlevel 1 (
        echo.
        echo The documentation build failed.
        exit /b 1
    )
)

REM Detached, so closing this window does not kill the server. If one is
REM already listening, serve.mjs exits 0 and the browser opens that one.
start "bibo docs" /min cmd /c "node serve.mjs %PORT%"

REM The server binds in well under a second, but the browser must not race it.
timeout /t 1 /nobreak >nul 2>&1

start "" "http://127.0.0.1:%PORT%/"
endlocal
