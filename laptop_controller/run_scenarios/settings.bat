@echo off
rem Shared settings

rem Controller serial port
set PORT=COM4

rem Bench-scale load-cell target (N)
set BENCH_TARGET=1500

rem py if installed, else python
where py >nul 2>nul && (set PY=py) || (set PY=python)

rem Run from laptop_controller
cd /d "%~dp0.."
