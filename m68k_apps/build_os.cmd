@echo off
REM Build M68K-OS Operating System
REM Requires m68k cross-compiler toolchain in PATH

echo ================================================
echo Building M68K-OS Operating System
echo ================================================
echo.

REM Try different M68K compiler prefixes
set M68K_PREFIX=

REM Check for m68k-elf (bare metal)
where m68k-elf-g++ >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set M68K_PREFIX=m68k-elf-
    goto :found
)

REM Check for m68k-linux-gnu (Linux cross-compiler via WSL or MSYS2)
where m68k-linux-gnu-g++ >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set M68K_PREFIX=m68k-linux-gnu-
    goto :found
)

REM Check for m68k-unknown-elf
where m68k-unknown-elf-g++ >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set M68K_PREFIX=m68k-unknown-elf-
    goto :found
)

REM No compiler found
echo ERROR: No M68K cross-compiler found in PATH!
echo.
echo Please install using WSL (Windows Subsystem for Linux):
echo.
echo   1. Install WSL (run in PowerShell as Admin):
echo      wsl --install -d Ubuntu
echo.
echo   2. Restart your computer, then open Ubuntu from Start menu
echo.
echo   3. In Ubuntu terminal, install the M68K toolchain:
echo      sudo apt update
echo      sudo apt install gcc-m68k-linux-gnu g++-m68k-linux-gnu
echo.
echo   4. Navigate to this directory in WSL:
echo      cd /mnt/d/code/ESP32-P4-NANO_Demo/ESP-IDF/02_HelloWorld/m68k_apps
echo.
echo   5. Run the build script:
echo      ./build_os.sh
echo.
echo Alternative: Use Docker with an M68K toolchain image
echo.
pause
exit /b 1

:found
echo Found M68K toolchain: %M68K_PREFIX%g++
%M68K_PREFIX%g++ --version | findstr /C:"g++"
echo.

REM Create output directory
if not exist "build" mkdir build

REM Compile M68K-OS
echo [1/3] Compiling m68k_os.cpp...
%M68K_PREFIX%g++ -c -O2 -m68000 -nostdlib -fno-exceptions -fno-rtti -ffreestanding m68k_os.cpp -o build/m68k_os.o
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Compilation failed!
    pause
    exit /b 1
)

echo [2/3] Linking...
%M68K_PREFIX%ld -T m68k.ld build/m68k_os.o -o build/m68k_os.elf
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Linking failed!
    pause
    exit /b 1
)

echo [3/3] Creating binary...
%M68K_PREFIX%objcopy -O binary build/m68k_os.elf build/OS.bin
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Binary creation failed!
    pause
    exit /b 1
)

echo.
echo ================================================
echo Build Successful!
echo ================================================
echo.
echo Output file: build\OS.bin
for %%F in (build\OS.bin) do echo File size: %%~zF bytes
echo.
echo To install the OS:
echo   1. Copy build\OS.bin to the root of your SD card
echo   2. The bootloader will auto-load it on boot
echo.
echo Or load manually in the BIOS:
echo   loadsd OS.bin
echo   reset
echo.

REM Show symbol table
echo Symbol table:
%M68K_PREFIX%nm build/m68k_os.elf | findstr /C:"_start" /C:"os_main" /C:"__bss"
echo.

pause
