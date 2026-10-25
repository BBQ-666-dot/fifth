@echo off
rem ============================================================
rem  Live plot of chassis.yaw_delta via DAPLink (OpenOCD TCL, no serial)
rem
rem  Double-click:
rem    - attach to a running VS Code OpenOCD if present
rem    - otherwise start OpenOCD with openocd_wireless_dap.cfg
rem
rem  Useful arguments:
rem    --rate 50                         sample rate (default 20 Hz)
rem    --cfg openocd_dap.cfg             use the wired CMSIS-DAP config
rem    --demo                            no-hardware demo
rem    --check                           resolve symbols only
rem    --standalone                      force starting its own OpenOCD
rem    --tcl 6666                        connect to a specific TCL port
rem ============================================================
setlocal
set "DIR=%~dp0"
set "PY=C:\Users\bbq25\anaconda3\python.exe"
if not exist "%PY%" set "PY=python"
"%PY%" "%DIR%livewatch_plot.py" --elf "%DIR%..\MDK-ARM\standard_tpye_c\standard_tpye_c.axf" --cfg openocd_wireless_dap.cfg --var chassis.yaw_delta --no-follow-vscode %*
pause
exit /b 0
