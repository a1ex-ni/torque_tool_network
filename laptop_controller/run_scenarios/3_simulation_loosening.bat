@echo off
title Simulation - loosening and re-tightening
rem No hardware. 12 tools; bolts loosen and get re-tightened.
call "%~dp0settings.bat"
echo Simulation - loosening and re-tightening
echo.
%PY% dashboard.py --sim 12 --sim-stall 0 --sim-loosen 0.3
echo.
pause
