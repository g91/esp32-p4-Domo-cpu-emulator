@echo off
REM Build M68K binaries using hand-assembled hex (no compiler needed)
REM This creates the test binaries directly from opcodes

echo ========================================
echo M68K Binary Builder (Manual Assembly)
echo ========================================
echo.

REM Create test1 - Simple counter (0 to 100)
echo Building test1_simple.bin...
(
    echo|set /p="0000FFFC00000010"
    echo|set /p="0000000000000000"
    echo|set /p="7000524"
0C4000646FF84E7160FE"
) > test1_simple.hex

REM Convert hex to binary using PowerShell
powershell -Command "$hex = Get-Content test1_simple.hex; $bytes = [byte[]]::new($hex.Length / 2); for ($i = 0; $i -lt $hex.Length; $i += 2) { $bytes[$i / 2] = [Convert]::ToByte($hex.Substring($i, 2), 16) }; [System.IO.File]::WriteAllBytes('test1_simple.bin', $bytes)"

echo Test1 created: test1_simple.bin
echo.

REM Create test2 - Register operations
echo Building test2_registers.bin...
(
    echo|set /p="0000FFFC00000010"
    echo|set /p="0000000000000000"
    echo|set /p="7001700252404E7160FE"
) > test2_registers.hex

powershell -Command "$hex = Get-Content test2_registers.hex; $bytes = [byte[]]::new($hex.Length / 2); for ($i = 0; $i -lt $hex.Length; $i += 2) { $bytes[$i / 2] = [Convert]::ToByte($hex.Substring($i, 2), 16) }; [System.IO.File]::WriteAllBytes('test2_registers.bin', $bytes)"

echo Test2 created: test2_registers.bin
echo.

REM Create test3 - Full instruction test
echo Building test3_full.bin...
(
    echo|set /p="0000FFFC00000010"
    echo|set /p="0000000000000000"
    echo|set /p="700070017002700370047005700670077200720172027203720472057206720791C091C191C291C391C491C591C691C74E714E714E714E714E714E714E7160FE"
) > test3_full.hex

powershell -Command "$hex = Get-Content test3_full.hex; $bytes = [byte[]]::new($hex.Length / 2); for ($i = 0; $i -lt $hex.Length; $i += 2) { $bytes[$i / 2] = [Convert]::ToByte($hex.Substring($i, 2), 16) }; [System.IO.File]::WriteAllBytes('test3_full.bin', $bytes)"

echo Test3 created: test3_full.bin
echo.

echo ========================================
echo Build complete! Created binaries:
echo   - test1_simple.bin (28 bytes)
echo   - test2_registers.bin (24 bytes)
echo   - test3_full.bin (92 bytes)
echo.
echo To test on ESP32-P4:
echo   1. Copy .bin files to SD card
echo   2. Connect to serial console
echo   3. Run: loadsd /sdcard/test1_simple.bin
echo   4. Run: reset
echo   5. Run: step or run 100
echo ========================================

pause
