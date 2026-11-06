@echo off
title First-time setup
rem Install Python packages (once per laptop).
call "%~dp0settings.bat"
echo First-time setup
echo.
%PY% -m pip install -r requirements.txt
echo.
pause
