@echo off
rem ============================================================
rem  Live Watch waveform monitor (OpenOCD TCL background reads)
rem  Double-click            -> auto: attach to running OpenOCD,
rem                             otherwise start OpenOCD with the
rem                             wireless DAPLink config
rem  Optional arguments:
rem    --attach              attach to VS Code OpenOCD only
rem    --demo                no hardware demo
rem    --check               resolve symbols only
rem    --var EXPR            add watched expression (repeatable)
rem    --rate 5              sample rate in Hz
rem ============================================================
setlocal
set "DIR=%~dp0"
set "PY=C:\Users\bbq25\anaconda3\python.exe"
if not exist "%PY%" set "PY=python"
if "%~1"=="" (
    "%PY%" "%DIR%livewatch_plot.py"
) else (
    "%PY%" "%DIR%livewatch_plot.py" %*
)
pause
exit /b 0
