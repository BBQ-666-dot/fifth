@echo off
rem ============================================================
rem  Live Watch waveform monitor - silent launcher (no console)
rem  Auto: attach to running OpenOCD, otherwise start it.
rem  Errors are shown as a message box by the tool itself.
rem ============================================================
setlocal
set "DIR=%~dp0"
set "PYW=C:\Users\bbq25\anaconda3\pythonw.exe"
if not exist "%PYW%" set "PYW=pythonw"
if "%~1"=="" (
    start "" "%PYW%" "%DIR%livewatch_plot.py"
) else (
    start "" "%PYW%" "%DIR%livewatch_plot.py" %*
)
exit /b 0
