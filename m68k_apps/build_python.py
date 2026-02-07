#!/usr/bin/env python3
"""
M68K Binary Builder - Python Version
Creates M68K test binaries from hand-assembled opcodes
No compiler needed!
"""

import os
import sys

def create_binary(filename, hex_string):
    """Convert hex string to binary file"""
    # Remove spaces and newlines
    hex_string = hex_string.replace(' ', '').replace('\n', '').replace('\r', '')
    
    # Convert to bytes
    binary_data = bytes.fromhex(hex_string)
    
    # Write to file
    with open(filename, 'wb') as f:
        f.write(binary_data)
    
    print(f"✓ Created {filename} ({len(binary_data)} bytes)")

def main():
    print("=" * 50)
    print("M68K Binary Builder (Manual Assembly)")
    print("=" * 50)
    print()
    
    # Test 1: Simple counter (0 to 100)
    print("Building test1_simple.bin...")
    create_binary('test1_simple.bin', '0000FFFC000000100000000000000000700052400C4000646FF84E7160FE')
    
    # Test 2: Register operations
    print("Building test2_registers.bin...")
    create_binary('test2_registers.bin', '0000FFFC0000001000000000000000007001700252404E7160FE')
    
    # Test 3: Full instruction test
    print("Building test3_full.bin...")
    create_binary('test3_full.bin', 
                  '0000FFFC000000100000000000000000' +
                  '7000700170027003700470057006700772007201720272037204720572067207' +
                  '91C091C191C291C391C491C591C691C7' +
                  '4E714E714E714E714E714E714E7160FE')
        7005                  ; MOVEQ #5, D5
        7006                  ; MOVEQ #6, D6
    print("  - test2_registers.bin (24 bytes)")
    print("  - test3_full.bin (92 bytes)")
    print()
    print("To test on ESP32-P4:")
    print("  1. Copy .bin files to SD card")
    print("  2. Connect to serial console")
    print("  3. Run: loadsd /sdcard/test1_simple.bin")
    print("  4. Run: reset")
    print("  5. Run: step or run 100")
    print("=" * 50)

if __name__ == '__main__':
    main()
