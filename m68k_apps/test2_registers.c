/* M68K Test Application 2: Register and Memory Test
 * Tests CPU registers and memory operations
 */

#define CONSOLE_OUT ((volatile unsigned char *)0x00F02000)

void print(const char *str) {
    while (*str) {
        *CONSOLE_OUT = *str++;
    }
}

// Print hex value
void print_hex(unsigned long value) {
    const char hex[] = "0123456789ABCDEF";
    *CONSOLE_OUT = '0';
    *CONSOLE_OUT = 'x';
    for (int i = 7; i >= 0; i--) {
        *CONSOLE_OUT = hex[(value >> (i * 4)) & 0xF];
    }
}

void _start(void) {
    volatile unsigned long *test_mem = (volatile unsigned long *)0x00100000;
    unsigned long reg_d0, reg_d1, reg_d2;
    
    print("=== M68K Test Application 2 ===\r\n");
    print("Register and Memory Test\r\n\r\n");
    
    // Test data registers
    print("Testing data registers:\r\n");
    reg_d0 = 0x12345678;
    reg_d1 = 0xAABBCCDD;
    reg_d2 = reg_d0 + reg_d1;
    
    print("D0 = ");
    print_hex(reg_d0);
    print("\r\n");
    
    print("D1 = ");
    print_hex(reg_d1);
    print("\r\n");
    
    print("D2 = D0 + D1 = ");
    print_hex(reg_d2);
    print("\r\n\r\n");
    
    // Test memory operations
    print("Testing memory operations:\r\n");
    *test_mem = 0xDEADBEEF;
    print("Wrote 0xDEADBEEF to address 0x00100000\r\n");
    
    unsigned long read_val = *test_mem;
    print("Read back: ");
    print_hex(read_val);
    print("\r\n");
    
    if (read_val == 0xDEADBEEF) {
        print("Memory test PASSED!\r\n");
    } else {
        print("Memory test FAILED!\r\n");
    }
    
    print("\r\n");
    
    // Test loop counter
    print("Loop counter test (0-9):\r\n");
    for (int i = 0; i < 10; i++) {
        *CONSOLE_OUT = '0' + i;
        *CONSOLE_OUT = ' ';
    }
    print("\r\n\r\n");
    
    print("All tests completed!\r\n");
    print("============================\r\n");
    
    // Infinite loop
    while (1) {
        // CPU idle
    }
}
