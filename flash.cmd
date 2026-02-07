@echo off
REM Flash script for ESP32-P4 HelloWorld project

echo Setting up ESP-IDF environment...
call C:\Espressif\frameworks\esp-idf-v5.5.2\export.bat

echo.
echo Building and flashing project...
idf.py -p COM27 build flash monitor

pause
