@echo off
cd /d "%~dp0"
python run_realtime_sim.py
if errorlevel 1 (
    py run_realtime_sim.py
)
pause
