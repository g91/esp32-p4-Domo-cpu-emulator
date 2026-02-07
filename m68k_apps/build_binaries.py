#!/usr/bin/env python3
"""
M68K Binary Builder
Creates M68K test binaries from hand-assembled opcodes
No compiler needed!
"""

def create_binary(filename, hex_string):
    """Convert hex string to binary file"""
    hex_string = hex_string.replace(' ', '').replace('\n', '').replace('\r', '')
    binary_data = bytes.fromhex(hex_string)
    with open(filename, 'wb') as f:
        f.write(binary_data)
    print(f"✓ Created {filename} ({len(binary_data)} bytes)")

def main():
    print("=" * 50)
    print("M68K Binary Builder")
    print("=" * 50)
    print()
    
    # Test 1: Simple counter
    print("Building test1_simple.bin...")
    create_binary('test1_simple.bin', 
                  '0000FFFC000000100000000000000000700052400C4000646FF84E7160FE')
    
    # Test 2: Register operations
    print("Building test2_registers.bin...")
    create_binary('test2_registers.bin', 
                  '0000FFFC0000001000000000000000007001700252404E7160FE')
    
    # Test 3: Full instruction test
    print("Building test3_full.bin...")
    hex_data = '0000FFFC000000100000000000000000'
    hex_data += '7000700170027003700470057006700772007201720272037204720572067207'
    hex_data += '91C091C191C291C391C491C591C691C7'
    hex_data += '4E714E714E714E714E714E714E7160FE'
    create_binary('test3_full.bin', hex_data)
    
    print()
    print("=" * 50)
    print("Build complete!")
    print("  - test1_simple.bin (28 bytes)")
    print("  - test2_registers.bin (24 bytes)")
    print("  - test3_full.bin (92 bytes)")
    print()
    print("To test on ESP32-P4:")
    print("  1. Copy .bin files to SD card")
    print("  2. loadsd /sdcard/test1_simple.bin")
    print("  3. reset")
    print("  4. run 100")
    print("=" * 50)

if __name__ == '__main__':
    main()
