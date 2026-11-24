@echo off
set PYTHONUTF8=1
cd /d "%~dp0"
"%USERPROFILE%\Downloads\3DScanner-GrayCode-main\.venv\Scripts\python.exe" "%~dp0automatic_object_capture.py" %*
pause
