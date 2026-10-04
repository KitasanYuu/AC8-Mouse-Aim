@echo off
rem Live flight reconstruction: receives the mod telemetry and opens the page.
rem Enable it in the game config: [control] telemetry_port=49731
title AC8 Telemetry
where py >NUL 2>&1
if not errorlevel 1 (
  py -3 "%~dp0telemetry\server.py" %*
) else (
  python "%~dp0telemetry\server.py" %*
)
