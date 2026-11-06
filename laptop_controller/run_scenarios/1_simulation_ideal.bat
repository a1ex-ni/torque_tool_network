@echo off
title Simulation - ideal run
rem No hardware. 32 simulated tools, every bolt reaches target.
call "%~dp0settings.bat"
echo Simulation - ideal run
echo.
%PY% simulate_ideal.py
echo.
pause
