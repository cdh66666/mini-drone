@echo off
cd /d "%~dp0"
set PY=python
if exist D:\Python311\python.exe set PY=D:\Python311\python.exe
start "" http://127.0.0.1:8771/
%PY% -I drone_server.py 8771
