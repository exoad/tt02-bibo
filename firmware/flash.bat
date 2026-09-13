@echo off
REM firmware\flash.bat [file.uf2]  or  firmware\flash.bat backup [out.uf2] - see flash.ps1.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" %*
