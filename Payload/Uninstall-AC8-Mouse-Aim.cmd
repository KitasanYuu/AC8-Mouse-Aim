@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Uninstall-AC8-Mouse-Aim.ps1"
if errorlevel 1 pause
