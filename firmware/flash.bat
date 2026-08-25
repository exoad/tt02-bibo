@echo off
REM Flashes build\pico_debug.uf2 to the Pico. See flash.ps1 for the mechanism.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" %*
