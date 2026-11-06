@echo off
title Text controller (no dashboard)
rem Drive tools by typed commands. Close the Serial Monitor first.
call "%~dp0settings.bat"
echo Text controller (no dashboard)
echo.
%PY% laptop_controller.py --port %PORT%
echo.
pause
