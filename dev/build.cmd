@echo off
setlocal
rem Development script: works from the repository root, one level up.
cd /d "%~dp0.."

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Setup-Dependencies.ps1"
if errorlevel 1 exit /b 1

call "%~dp0msvc-env.cmd"
if errorlevel 1 exit /b 1

if not exist build mkdir build
lib /nologo /def:src\ue4ss_lua.def /machine:x64 /out:build\ue4ss_lua.lib
if errorlevel 1 exit /b 1

cl /nologo /std:c++20 /EHsc /O2 /MD /LD /Fo"build\\" ^
  /I"deps\ue4ss-source\deps\first\LuaMadeSimple\include" ^
  /I"deps\ue4ss-source\deps\first\LuaRaw\include" ^
  src\mouse_aim.cpp ^
  src\vendor\minhook\src\buffer.c ^
  src\vendor\minhook\src\hook.c ^
  src\vendor\minhook\src\trampoline.c ^
  src\vendor\minhook\src\hde\hde64.c ^
  /link /OUT:build\ac8_mouse_aim_010.dll /IMPLIB:build\ac8_mouse_aim_010.lib build\ue4ss_lua.lib
if errorlevel 1 exit /b 1

copy /y build\ac8_mouse_aim_010.dll "Payload\Game\Binaries\Win64\UE4SS\Mods\AC8MouseAim\Scripts\ac8_mouse_aim_010.dll" >nul
if errorlevel 1 exit /b 1
echo Built and copied ac8_mouse_aim_010.dll into Payload.

rem Development-only flight logic module, hot-loaded by Dev-Deploy -Live. Not shipped.
cl /nologo /std:c++20 /EHsc /O2 /MD /LD /Fo"build\\" src\flight_logic_dll.cpp ^
  /link /OUT:build\ac8_flight_logic.dll /IMPLIB:build\ac8_flight_logic.lib
if errorlevel 1 exit /b 1
