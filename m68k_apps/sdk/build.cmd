@echo off
REM M68K SDK - Windows Build Script
REM
REM Usage:
REM   build.cmd              Build the SDK library
REM   build.cmd myapp.c      Build an application with the SDK
REM   build.cmd clean        Clean build artifacts
REM

setlocal
cd /d "%~dp0"

if "%1"=="clean" (
    echo Cleaning build artifacts...
    if exist build rmdir /s /q build
    exit /b 0
)

REM Create build directory
if not exist build mkdir build

REM Toolchain prefix - change if your toolchain is different
if "%CROSS%"=="" set CROSS=m68k-linux-gnu-

set CC=%CROSS%gcc
set AS=%CROSS%gcc
set AR=%CROSS%ar
set OBJCOPY=%CROSS%objcopy
set SIZE=%CROSS%size

set CFLAGS=-m68000 -nostdlib -ffreestanding -fno-builtin -Os -Wall -Wextra -Iinclude
set ASFLAGS=-m68000 -nostdlib -ffreestanding

echo === Building M68K SDK ===

REM Compile startup code
echo   Assembling crt0.S...
%AS% %ASFLAGS% -c crt0.S -o build\crt0.o
if errorlevel 1 goto :error

REM Compile SDK sources
for %%f in (src\*.c) do (
    echo   Compiling %%~nf.c...
    %CC% %CFLAGS% -c %%f -o build\sdk_%%~nf.o
    if errorlevel 1 goto :error
)

REM Archive into library
echo   Creating libm68k.a...
%AR% rcs build\libm68k.a build\sdk_*.o
if errorlevel 1 goto :error

echo SDK built successfully.
echo   Library: build\libm68k.a
echo   Startup: build\crt0.o

REM Build application if specified
if "%1"=="" goto :done

echo.
echo === Building Application: %1 ===

set APP_NAME=%~n1
%CC% %CFLAGS% -c %1 -o build\%APP_NAME%.o
if errorlevel 1 goto :error

%CC% -m68000 -nostdlib -ffreestanding -Tprogram.ld build\crt0.o build\%APP_NAME%.o -Lbuild -lm68k -lgcc -o build\%APP_NAME%.elf
if errorlevel 1 goto :error

%OBJCOPY% -O binary build\%APP_NAME%.elf build\%APP_NAME%.bin
if errorlevel 1 goto :error

%SIZE% build\%APP_NAME%.elf
echo.
echo Done! Binary: build\%APP_NAME%.bin
echo Load with:  load /sdcard/%APP_NAME%.bin
echo Run with:   loadrun /sdcard/%APP_NAME%.bin

:done
exit /b 0

:error
echo BUILD FAILED
exit /b 1
