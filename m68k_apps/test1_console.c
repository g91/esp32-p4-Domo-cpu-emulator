/* M68K Test Application 1: Console Output
 * Tests console I/O by printing a message
 */

// Console output address
#define CONSOLE_OUT ((volatile unsigned char *)0x00F02000)

// Simple print function
void print(const char *str) {
    while (*str) {
        *CONSOLE_OUT = *str++;
    }
}

void _start(void) {
    // Print welcome message
    print("=== M68K Test Application 1 ===\r\n");
    print("Console I/O Test\r\n");
    print("This program is running on the M68K emulator!\r\n");
    print("\r\n");
    
    // Print some numbers
    print("Counting test:\r\n");
    for (int i = 0; i < 10; i++) {
        print("Count: ");
        // Simple number to ASCII conversion
        char num[4];
        num[0] = '0' + (i / 10);
        num[1] = '0' + (i % 10);
        num[2] = '\r';
        num[3] = '\n';
        if (num[0] == '0' && i < 10) {
            *CONSOLE_OUT = num[1];
        } else {
            *CONSOLE_OUT = num[0];
            *CONSOLE_OUT = num[1];
        }
        *CONSOLE_OUT = '\r';
        *CONSOLE_OUT = '\n';
    }
    
    print("\r\n");
    print("Test completed successfully!\r\n");
    print("=================================\r\n");
    
    // Loop forever
    while (1) {
        // Do nothing
    }
}
