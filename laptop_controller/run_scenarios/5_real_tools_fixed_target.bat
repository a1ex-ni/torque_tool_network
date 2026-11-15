@echo off
title Real tools - fixed target test
rem No scaling; pick a Fixed target test flange (50 N). Close the Serial Monitor first.
call "%~dp0settings.bat"
echo Real tools - fixed target test
echo.
%PY% dashboard.py --port %PORT%
echo.
pause
