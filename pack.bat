@echo off

nuitka --onefile --standalone --follow-imports ^
--include-package=PID_Helper ^
--include-module=serial.tools.list_ports ^
--include-module=parse ^
--output-dir=dist ^
One_click_Start.py

pause