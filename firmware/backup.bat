@echo off
REM Reads the Pico's current flash back to a .uf2. See backup.ps1 for details.
REM Run this BEFORE flashing anything you cannot rebuild from source.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0backup.ps1" %*
