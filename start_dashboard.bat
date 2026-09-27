@echo off
echo ====================================================
echo Starting Bharat Electronics Limited (BEL)
echo Autonomous Fleet Hub ^& Digital Twin Command Center
echo Dashboard URL: http://localhost:8000
echo ====================================================

start "BEL-Fleet-Hub-Backend" cmd /k "cd /d %~dp0ref_sih_amr\dashboard\backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000"

timeout /t 2 /nobreak > nul
echo Opening Autonomous Fleet Hub in browser...
start http://localhost:8000

echo ====================================================
echo Unified Dashboard running at http://localhost:8000
echo ====================================================