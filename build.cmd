@echo off
REM Build script for ESP32-P4 HelloWorld project

echo Setting up ESP-IDF environment...
call C:\Espressif\frameworks\esp-idf-v5.5.2\export.bat

echo.
echo Building project...
idf.py build

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Build successful!
    echo To flash: idf.py -p COMx flash monitor
) else (
    echo.
    echo Build failed!
    exit /b 1
)
