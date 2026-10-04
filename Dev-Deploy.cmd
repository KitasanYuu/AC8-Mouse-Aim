@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Dev-Deploy.ps1" %*
if errorlevel 1 pause
