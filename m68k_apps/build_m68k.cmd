@echo off
REM Build script for M68K test applications
REM Requires m68k cross-compiler toolchain in PATH

echo ================================================
echo Building M68K Test Applications
echo ================================================
echo.

REM Try different M68K compiler prefixes
set M68K_PREFIX=

REM Check for m68k-elf (bare metal)
where m68k-elf-gcc >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set M68K_PREFIX=m68k-elf-
    goto :found
)

REM Check for m68k-linux-gnu (Linux cross-compiler via WSL or MSYS2)
where m68k-linux-gnu-gcc >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set M68K_PREFIX=m68k-linux-gnu-
    goto :found
)

REM Check for m68k-unknown-elf
where m68k-unknown-elf-gcc >nul 2>&1
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
echo      ./build_m68k.sh
echo.
pause
exit /b 1

:found
echo Found M68K toolchain: %M68K_PREFIX%gcc
%M68K_PREFIX%gcc --version | findstr /C:"gcc"
echo.

REM Create output directory
if not exist "build" mkdir build

REM Build Test 1: Console Output
echo [1/3] Building test1_console.c...
%M68K_PREFIX%gcc -c -O2 -m68000 -nostdlib test1_console.c -o build/test1_console.o
if %ERRORLEVEL% NEQ 0 goto :error

%M68K_PREFIX%ld -T m68k.ld build/test1_console.o -o build/test1_console.elf
if %ERRORLEVEL% NEQ 0 goto :error

%M68K_PREFIX%objcopy -O binary build/test1_console.elf build/test1_console.bin
if %ERRORLEVEL% NEQ 0 goto :error
echo       Output: build/test1_console.bin
for %%F in (build\test1_console.bin) do echo       Size: %%~zF bytes
echo.

REM Build Test 2: Register and Memory Test
echo [2/3] Building test2_registers.c...
%M68K_PREFIX%gcc -c -O2 -m68000 -nostdlib test2_registers.c -o build/test2_registers.o
if %ERRORLEVEL% NEQ 0 goto :error

%M68K_PREFIX%ld -T m68k.ld build/test2_registers.o -o build/test2_registers.elf
if %ERRORLEVEL% NEQ 0 goto :error

%M68K_PREFIX%objcopy -O binary build/test2_registers.elf build/test2_registers.bin
if %ERRORLEVEL% NEQ 0 goto :error
echo       Output: build/test2_registers.bin
for %%F in (build\test2_registers.bin) do echo       Size: %%~zF bytes
echo.

REM Build Test 3: Assembly Test (if VASM is available)
where vasmm68k_mot >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo [3/3] Building test3_asm.s with VASM...
    vasmm68k_mot -Fbin -o build/test3_asm.bin test3_asm.s
    if %ERRORLEVEL% NEQ 0 goto :error
    echo       Output: build/test3_asm.bin
    for %%F in (build\test3_asm.bin) do echo       Size: %%~zF bytes
    echo.
) else (
    echo [3/3] Skipping test3_asm.s (VASM not found)
    echo       Install VASM to build assembly programs
    echo.
)

echo ================================================
echo Build completed successfully!
echo ================================================
echo.
echo Binary files are in the 'build' directory:
dir /B build\*.bin
echo.
echo To test on ESP32-P4:
echo   1. Copy .bin files to SD card
echo   2. In ESP32-P4 console: loadsd /sdcard/test1_console.bin
echo   3. Run: reset
echo   4. Run: run 10000
echo.
goto :end

:error
echo.
echo ================================================
echo BUILD FAILED!
echo ================================================
echo.
pause
exit /b 1

:end
pause
