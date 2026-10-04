@echo off
setlocal
rem Development script: works from the repository root, one level up.
cd /d "%~dp0.."
call "%~dp0msvc-env.cmd"
if errorlevel 1 exit /b 1
if not exist build mkdir build
cl /nologo /std:c++20 /EHsc /Od /Fe:build\flight_math_checks.exe ^
  /Fo:build\flight_math_checks.obj /Fd:build\flight_math_checks.pdb ^
  tests\flight_math_checks.cpp
if errorlevel 1 exit /b 1
build\flight_math_checks.exe
if not "%errorlevel%"=="0" exit /b 1
echo Flight math checks passed.
rem Closed-loop simulation of the flight logic against the measured plant model.
cl /nologo /std:c++20 /EHsc /O2 /Fe:build\flight_sim.exe ^
  /Fo:build\flight_sim.obj /Fd:build\flight_sim.pdb tests\flight_sim.cpp
if errorlevel 1 exit /b 1
build\flight_sim.exe
if not "%errorlevel%"=="0" exit /b 1
