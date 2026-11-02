@echo off
rem Start the CTR V2 control centre (double-click in Windows Explorer)
cd /d "%~dp0"
python -m ctr_control
if errorlevel 1 pause
