@echo off
title Real tools - bench scale
rem Any API 6A flange scaled to BENCH_TARGET N. Close the Serial Monitor first.
call "%~dp0settings.bat"
echo Real tools - bench scale
echo.
%PY% dashboard.py --port %PORT% --bench-target %BENCH_TARGET%
echo.
pause
