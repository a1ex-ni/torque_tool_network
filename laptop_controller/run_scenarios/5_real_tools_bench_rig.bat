@echo off
title Real tools - bench rig flange
rem No scaling; pick a Bench Rig flange. Close the Serial Monitor first.
call "%~dp0settings.bat"
echo Real tools - bench rig flange
echo.
%PY% dashboard.py --port %PORT%
echo.
pause
