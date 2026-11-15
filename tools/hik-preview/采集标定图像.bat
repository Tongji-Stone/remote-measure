@echo off
cd /d "%~dp0"
"%USERPROFILE%\Downloads\3DScanner-GrayCode-main\.venv\Scripts\python.exe" "%~dp0calibration_capture.py"
pause
