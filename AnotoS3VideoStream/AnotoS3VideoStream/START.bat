@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
set "PYTHONUTF8=1"
if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" run.py %*
) else (
  python --version >nul 2>&1
  if not errorlevel 1 (
    python run.py %*
  ) else (
    py -3 run.py %*
  )
)
set "RC=%ERRORLEVEL%"
if not "%RC%"=="0" echo Execution failed. See the message above.
echo.
pause
exit /b %RC%
