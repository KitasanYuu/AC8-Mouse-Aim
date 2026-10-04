@echo off
setlocal
rem Flight controller comparison: builds the bench, flies every controller through the
rem same scenarios and opens the viewer. Extra arguments go to the bench, for example
rem   compare.cmd variant="kd 0.1:pitch_kd=0.1" variant="level 10:level_per_deg=10"
rem Works from the repository root, two levels up.
cd /d "%~dp0..\.."
call dev\msvc-env.cmd
if errorlevel 1 exit /b 1
if not exist build mkdir build
cl /nologo /std:c++20 /EHsc /O2 /utf-8 /Fe:build\compare.exe /Fo:build\compare.obj /Fd:build\compare.pdb ^
  dev\compare\harness.cpp >build\compare_build.log 2>&1
if errorlevel 1 (type build\compare_build.log & exit /b 1)
build\compare.exe %*
if errorlevel 1 exit /b 1
if /i not "%NO_BROWSER%"=="1" start "" "%~dp0index.html"
