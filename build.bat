@echo off
cd /d D:\code\ESP32-P4-NANO_Demo\ESP-IDF\02_HelloWorld
call C:\Espressif\frameworks\esp-idf-v5.5.2\export.bat
idf.py build
pause
