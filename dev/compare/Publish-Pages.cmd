@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Publish-Pages.ps1" %*
if errorlevel 1 pause
