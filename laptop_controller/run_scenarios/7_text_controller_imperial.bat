@echo off
title Text controller - imperial
rem Same as 6, in lbf.
call "%~dp0settings.bat"
echo Text controller - imperial
echo.
%PY% laptop_controller.py --port %PORT% --units imperial
echo.
pause
