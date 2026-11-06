@echo off
title Simulation - some bolts stall or over-tighten
rem No hardware. 12 tools; ~1 in 5 stalls, ~1 in 10 over-tightens.
call "%~dp0settings.bat"
echo Simulation - some bolts stall or over-tighten
echo.
%PY% dashboard.py --sim 12 --sim-stall 0.2 --sim-overshoot 0.1
echo.
pause
