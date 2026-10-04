@echo off
setlocal EnableDelayedExpansion
rem Development script: works from the repository root, one level up.
cd /d "%~dp0.."
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Setup-Dependencies.ps1"
if errorlevel 1 exit /b 1
call "%~dp0msvc-env.cmd"
if errorlevel 1 exit /b 1
if not exist build mkdir build
if not exist build\lua mkdir build\lua

set "LUA_SOURCE=deps\ue4ss-source\deps\first\LuaRaw"
set "CORE_OBJECTS="
for %%F in ("%LUA_SOURCE%\src\*.c") do (
  cl /nologo /c /O2 /I"%LUA_SOURCE%\include" /Fo"build\lua\%%~nF.obj" "%%~fF"
  if errorlevel 1 exit /b 1
  if /I not "%%~nF"=="lua" if /I not "%%~nF"=="luac" set "CORE_OBJECTS=!CORE_OBJECTS! build\lua\%%~nF.obj"
)
link /nologo /OUT:build\lua.exe !CORE_OBJECTS! build\lua\lua.obj
if errorlevel 1 exit /b 1
link /nologo /OUT:build\luac.exe !CORE_OBJECTS! build\lua\luac.obj
if errorlevel 1 exit /b 1

for %%F in ("Payload\Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\Scripts\*.lua") do (
  build\luac.exe -p "%%~fF"
  if errorlevel 1 exit /b 1
)
build\lua.exe tests\rig_math_checks.lua
if errorlevel 1 exit /b 1
echo Lua syntax and rig math checks passed.
