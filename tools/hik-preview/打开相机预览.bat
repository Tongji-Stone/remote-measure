@echo off
cd /d "%~dp0"
"%USERPROFILE%\Downloads\3DScanner-GrayCode-main\.venv\Scripts\python.exe" "%~dp0preview.py"
if errorlevel 1 pause
