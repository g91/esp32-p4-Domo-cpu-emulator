/*
 * M68K-OS: A Simple Operating System for the M68000 Processor
 * Version 1.0
 * 
 * Features:
 * - Command-line shell with basic commands
 * - Memory management
 * - File system access via bus controller
 * - Network stack access via bus controller
 * - Process management (single-tasking)
 */

// Ensure C linkage for entry point
extern "C" void _start(void);

// ============================================================================
// Hardware I/O Addresses (via Bus Controller)
// ============================================================================

#define CONSOLE_OUT     ((volatile unsigned char *)0x00F02000)
#define CONSOLE_IN      ((volatile unsigned char *)0x00F02004)
#define CONSOLE_STATUS  ((volatile unsigned char *)0x00F02008)

#define FS_COMMAND      ((volatile unsigned long *)0x00F01000)
#define FS_STATUS       ((volatile unsigned long *)0x00F01004)
#define FS_FILENAME     ((volatile char *)0x00F01100)
#define FS_BUFFER       ((volatile unsigned char *)0x00F01200)
#define FS_SIZE         ((volatile unsigned long *)0x00F01008)
#define FS_OFFSET       ((volatile unsigned long *)0x00F0100C)

#define NET_COMMAND     ((volatile unsigned long *)0x00F00000)
#define NET_STATUS      ((volatile unsigned long *)0x00F00004)
#define NET_SOCKET_ID   ((volatile unsigned long *)0x00F00008)
#define NET_ADDR_TYPE   ((volatile unsigned long *)0x00F0000C)
#define NET_ADDR_IP     ((volatile unsigned long *)0x00F00010)
#define NET_ADDR_PORT   ((volatile unsigned long *)0x00F00014)
#define NET_DATA_LEN    ((volatile unsigned long *)0x00F00018)
#define NET_DATA_PTR    ((volatile unsigned long *)0x00F0001C)
#define NET_FLAGS       ((volatile unsigned long *)0x00F00020)
#define NET_RESULT      ((volatile unsigned long *)0x00F00024)

// Network commands
#define NET_CMD_SOCKET      0x01
#define NET_CMD_BIND        0x02
#define NET_CMD_LISTEN      0x03
#define NET_CMD_ACCEPT      0x04
#define NET_CMD_CONNECT     0x05
#define NET_CMD_SEND        0x06
#define NET_CMD_RECV        0x07
#define NET_CMD_SENDTO      0x08
#define NET_CMD_RECVFROM    0x09
#define NET_CMD_CLOSE       0x0A
#define NET_CMD_PING        0x0E

// Socket types
#define SOCK_STREAM         0x01
#define SOCK_DGRAM          0x02
#define SOCK_RAW            0x03

#define TIMER_COUNT     ((volatile unsigned long *)0x00F05000)
#define TIMER_CONTROL   ((volatile unsigned long *)0x00F05004)

// Console status bits
#define CONSOLE_RX_READY    0x01
#define CONSOLE_TX_READY    0x02

// Filesystem commands
#define FS_CMD_OPEN         0x01
#define FS_CMD_CLOSE        0x02
#define FS_CMD_READ         0x03
#define FS_CMD_WRITE        0x04
#define FS_CMD_SEEK         0x05
#define FS_CMD_STAT         0x06
#define FS_CMD_READDIR      0x07
#define FS_CMD_MKDIR        0x08
#define FS_CMD_REMOVE       0x09

// ============================================================================
// String and Memory Functions
// ============================================================================

extern "C" {

void *memset(void *s, int c, unsigned long n) {
    unsigned char *p = (unsigned char *)s;
    while (n--) *p++ = (unsigned char)c;
    return s;
}

void *memcpy(void *dest, const void *src, unsigned long n) {
    unsigned char *d = (unsigned char *)dest;
    const unsigned char *s = (const unsigned char *)src;
    while (n--) *d++ = *s++;
    return dest;
}

int strcmp(const char *s1, const char *s2) {
    while (*s1 && (*s1 == *s2)) {
        s1++;
        s2++;
    }
    return *(unsigned char *)s1 - *(unsigned char *)s2;
}

int strncmp(const char *s1, const char *s2, unsigned long n) {
    while (n && *s1 && (*s1 == *s2)) {
        s1++;
        s2++;
        n--;
    }
    if (n == 0) return 0;
    return *(unsigned char *)s1 - *(unsigned char *)s2;
}

unsigned long strlen(const char *s) {
    unsigned long len = 0;
    while (*s++) len++;
    return len;
}

char *strcpy(char *dest, const char *src) {
    char *d = dest;
    while ((*d++ = *src++));
    return dest;
}

char *strncpy(char *dest, const char *src, unsigned long n) {
    char *d = dest;
    while (n && (*d++ = *src++)) n--;
    while (n--) *d++ = 0;
    return dest;
}

char *strcat(char *dest, const char *src) {
    char *d = dest;
    while (*d) d++;
    while ((*d++ = *src++));
    return dest;
}

char *strchr(const char *s, int c) {
    while (*s) {
        if (*s == c) return (char *)s;
        s++;
    }
    return 0;
}

char *strrchr(const char *s, int c) {
    const char *last = 0;
    while (*s) {
        if (*s == c) last = s;
        s++;
    }
    return (char *)last;
}

int isspace(int c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

int isdigit(int c) {
    return c >= '0' && c <= '9';
}

int isalpha(int c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

int tolower(int c) {
    if (c >= 'A' && c <= 'Z') return c + 32;
    return c;
}

int toupper(int c) {
    if (c >= 'a' && c <= 'z') return c - 32;
    return c;
}

int atoi(const char *s) {
    int n = 0;
    int neg = 0;
    while (isspace(*s)) s++;
    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') s++;
    while (isdigit(*s)) {
        n = n * 10 + (*s - '0');
        s++;
    }
    return neg ? -n : n;
}

// ============================================================================
// Software Division/Multiplication (M68000 lacks 32-bit math instructions)
// ============================================================================

// Unsigned 32-bit multiplication
unsigned long __mulsi3(unsigned long a, unsigned long b) {
    unsigned long result = 0;
    while (b) {
        if (b & 1) result += a;
        a <<= 1;
        b >>= 1;
    }
    return result;
}

// Unsigned 32-bit division
unsigned long __udivsi3(unsigned long num, unsigned long den) {
    if (den == 0) return 0;  // Division by zero
    unsigned long quot = 0;
    unsigned long bit = 1;
    
    // Normalize: shift divisor left until >= dividend
    while ((long)den > 0 && den < num) {
        den <<= 1;
        bit <<= 1;
    }
    
    // Divide
    while (bit) {
        if (num >= den) {
            num -= den;
            quot |= bit;
        }
        den >>= 1;
        bit >>= 1;
    }
    return quot;
}

// Unsigned 32-bit modulo (without using multiplication)
unsigned long __umodsi3(unsigned long num, unsigned long den) {
    if (den == 0) return 0;
    
    // Normalize
    unsigned long temp_den = den;
    while ((long)temp_den > 0 && temp_den < num) {
        temp_den <<= 1;
    }
    
    // Subtract shifted divisor repeatedly
    while (temp_den >= den) {
        if (num >= temp_den) {
            num -= temp_den;
        }
        temp_den >>= 1;
    }
    return num;
}

// Signed 32-bit division
long __divsi3(long num, long den) {
    int neg = 0;
    if (num < 0) { num = -num; neg = !neg; }
    if (den < 0) { den = -den; neg = !neg; }
    long result = (long)__udivsi3((unsigned long)num, (unsigned long)den);
    return neg ? -result : result;
}

// Signed 32-bit modulo
long __modsi3(long num, long den) {
    int neg = (num < 0);
    if (num < 0) num = -num;
    if (den < 0) den = -den;
    long result = (long)__umodsi3((unsigned long)num, (unsigned long)den);
    return neg ? -result : result;
}

} // extern "C"

// ============================================================================
// Console I/O Class
// ============================================================================

class Console {
public:
    static void putchar(char c) {
        *CONSOLE_OUT = c;
    }
    
    static void puts(const char *s) {
        while (*s) {
            if (*s == '\n') putchar('\r');
            putchar(*s++);
        }
    }
    
    static void print(const char *s) {
        puts(s);
    }
    
    static void println(const char *s) {
        puts(s);
        putchar('\r');
        putchar('\n');
    }
    
    static void print_hex(unsigned long val, int digits = 8) {
        const char hex[] = "0123456789ABCDEF";
        for (int i = digits - 1; i >= 0; i--) {
            putchar(hex[(val >> (i * 4)) & 0xF]);
        }
    }
    
    static void print_dec(long val) {
        if (val < 0) {
            putchar('-');
            val = -val;
        }
        if (val == 0) {
            putchar('0');
            return;
        }
        char buf[12];
        int i = 0;
        while (val > 0) {
            buf[i++] = '0' + (val % 10);
            val /= 10;
        }
        while (i > 0) {
            putchar(buf[--i]);
        }
    }
    
    static void printf(const char *fmt, ...) {
        // Simple printf implementation
        // Supports: %s, %d, %x, %c, %%
        __builtin_va_list args;
        __builtin_va_start(args, fmt);
        
        while (*fmt) {
            if (*fmt == '%') {
                fmt++;
                switch (*fmt) {
                    case 's': {
                        const char *s = __builtin_va_arg(args, const char *);
                        if (s) puts(s);
                        else puts("(null)");
                        break;
                    }
                    case 'd': {
                        int d = __builtin_va_arg(args, int);
                        print_dec(d);
                        break;
                    }
                    case 'x': {
                        unsigned int x = __builtin_va_arg(args, unsigned int);
                        print_hex(x, 8);
                        break;
                    }
                    case 'c': {
                        int c = __builtin_va_arg(args, int);
                        putchar(c);
                        break;
                    }
                    case '%':
                        putchar('%');
                        break;
                    default:
                        putchar('%');
                        putchar(*fmt);
                        break;
                }
            } else {
                if (*fmt == '\n') putchar('\r');
                putchar(*fmt);
            }
            fmt++;
        }
        
        __builtin_va_end(args);
    }
    
    static bool key_available() {
        return (*CONSOLE_STATUS & CONSOLE_RX_READY) != 0;
    }
    
    static char getchar() {
        while (!key_available()) {
            // Wait for input
        }
        return *CONSOLE_IN;
    }
    
    static int getline(char *buf, int maxlen) {
        int pos = 0;
        while (pos < maxlen - 1) {
            char c = getchar();
            
            if (c == '\r' || c == '\n') {
                putchar('\r');
                putchar('\n');
                break;
            }
            else if (c == '\b' || c == 0x7F) {
                if (pos > 0) {
                    pos--;
                    putchar('\b');
                    putchar(' ');
                    putchar('\b');
                }
            }
            else if (c == 0x03) {  // Ctrl+C
                puts("^C\r\n");
                pos = 0;
                break;
            }
            else if (c >= 0x20 && c < 0x7F) {
                buf[pos++] = c;
                putchar(c);
            }
        }
        buf[pos] = '\0';
        return pos;
    }
};

// ============================================================================
// Memory Manager
// ============================================================================

#define HEAP_START  0x00100000  // 1MB mark
#define HEAP_END    0x00E00000  // 14MB mark (leave 1MB for I/O)

class Memory {
private:
    static unsigned long heap_ptr;
    
public:
    static void init() {
        heap_ptr = HEAP_START;
    }
    
    static void *alloc(unsigned long size) {
        // Align to 4 bytes
        size = (size + 3) & ~3;
        
        if (heap_ptr + size > HEAP_END) {
            return 0;  // Out of memory
        }
        
        void *ptr = (void *)heap_ptr;
        heap_ptr += size;
        return ptr;
    }
    
    static void free(void *ptr) {
        // Simple allocator - no free support
        (void)ptr;
    }
    
    static unsigned long get_free() {
        return HEAP_END - heap_ptr;
    }
    
    static unsigned long get_used() {
        return heap_ptr - HEAP_START;
    }
    
    static unsigned long get_total() {
        return HEAP_END - HEAP_START;
    }
};

unsigned long Memory::heap_ptr = HEAP_START;

// ============================================================================
// Simple Filesystem API
// ============================================================================

class FileSystem {
public:
    static int open(const char *filename) {
        // Copy filename to FS buffer
        volatile char *fn = FS_FILENAME;
        while (*filename) *fn++ = *filename++;
        *fn = 0;
        
        // Issue open command
        *FS_COMMAND = FS_CMD_OPEN;
        
        // Wait for completion
        while (*FS_STATUS == 0);
        
        return (*FS_STATUS == 1) ? 0 : -1;
    }
    
    static void close() {
        *FS_COMMAND = FS_CMD_CLOSE;
        while (*FS_STATUS == 0);
    }
    
    static int read(void *buffer, unsigned long size) {
        *FS_SIZE = size;
        *FS_COMMAND = FS_CMD_READ;
        while (*FS_STATUS == 0);
        
        if (*FS_STATUS != 1) return -1;
        
        // Copy from FS buffer to user buffer
        memcpy(buffer, (void *)FS_BUFFER, size);
        return (int)*FS_SIZE;
    }
    
    static int write(const void *buffer, unsigned long size) {
        // Copy to FS buffer
        memcpy((void *)FS_BUFFER, buffer, size);
        *FS_SIZE = size;
        *FS_COMMAND = FS_CMD_WRITE;
        while (*FS_STATUS == 0);
        return (*FS_STATUS == 1) ? (int)size : -1;
    }
    
    static int list_dir(const char *path) {
        // Copy path to filename buffer
        volatile char *fn = FS_FILENAME;
        while (*path) *fn++ = *path++;
        *fn = 0;
        
        *FS_COMMAND = FS_CMD_READDIR;
        while (*FS_STATUS == 0);
        
        return (*FS_STATUS == 1) ? 0 : -1;
    }
};

// ============================================================================
// Shell Commands
// ============================================================================

class Shell {
private:
    static char cmd_buffer[256];
    static char *argv[16];
    static int argc;
    static bool running;
    static char current_dir[256];
    
    static void parse_command(char *line) {
        argc = 0;
        
        // Skip leading whitespace
        while (*line && isspace(*line)) line++;
        
        while (*line && argc < 16) {
            argv[argc++] = line;
            
            // Find end of argument
            while (*line && !isspace(*line)) line++;
            
            if (*line) {
                *line++ = '\0';
                while (*line && isspace(*line)) line++;
            }
        }
    }
    
    // Command handlers
    static void cmd_help() {
        Console::println("");
        Console::println("M68K-OS Command Reference");
        Console::println("=========================");
        Console::println("");
        Console::println("  help      - Show this help message");
        Console::println("  ver       - Display OS version");
        Console::println("  cls       - Clear the screen");
        Console::println("  mem       - Show memory information");
        Console::println("  regs      - Display CPU registers");
        Console::println("  echo      - Echo text to console");
        Console::println("  ls        - List directory contents");
        Console::println("  pwd       - Print working directory");
        Console::println("  cd        - Change directory");
        Console::println("  cat       - Display file contents");
        Console::println("  hex       - Hex dump memory");
        Console::println("  poke      - Write byte to memory");
        Console::println("  peek      - Read byte from memory");
        Console::println("  run       - Execute program at address");
        Console::println("  ping      - Ping a host (requires WiFi)");
        Console::println("  uptime    - Show system uptime");
        Console::println("  reboot    - Reboot the system");
        Console::println("");
    }
    
    static void cmd_version() {
        Console::println("");
        Console::println("  M68K-OS Version 1.0");
        Console::println("  (C) 2026 ESP32-P4 M68K Emulator Project");
        Console::println("  Build: " __DATE__ " " __TIME__);
        Console::println("");
        Console::println("  Hardware: Motorola 68000 CPU");
        Console::println("  Clock: 60 MHz (emulated)");
        Console::println("  Memory: 16 MB RAM");
        Console::println("");
    }
    
    static void cmd_clear() {
        // Send ANSI clear sequence
        Console::print("\033[2J\033[H");
    }
    
    static void cmd_memory() {
        Console::println("");
        Console::printf("  Total Memory:  %d bytes (%d KB)\n", 
                       Memory::get_total(), Memory::get_total() / 1024);
        Console::printf("  Used Memory:   %d bytes (%d KB)\n",
                       Memory::get_used(), Memory::get_used() / 1024);
        Console::printf("  Free Memory:   %d bytes (%d KB)\n",
                       Memory::get_free(), Memory::get_free() / 1024);
        Console::println("");
    }
    
    static void cmd_registers() {
        // Read CPU registers using inline assembly
        unsigned long d0, d1, d2, d3, d4, d5, d6, d7;
        unsigned long a0, a1, a2, a3, a4, a5, a6, sp;
        
        asm volatile (
            "move.l %%d0, %0\n"
            "move.l %%d1, %1\n"
            "move.l %%d2, %2\n"
            "move.l %%d3, %3\n"
            "move.l %%d4, %4\n"
            "move.l %%d5, %5\n"
            "move.l %%d6, %6\n"
            "move.l %%d7, %7\n"
            : "=m"(d0), "=m"(d1), "=m"(d2), "=m"(d3),
              "=m"(d4), "=m"(d5), "=m"(d6), "=m"(d7)
        );
        
        asm volatile (
            "move.l %%a0, %0\n"
            "move.l %%a1, %1\n"
            "move.l %%a2, %2\n"
            "move.l %%a3, %3\n"
            "move.l %%a4, %4\n"
            "move.l %%a5, %5\n"
            "move.l %%a6, %6\n"
            "move.l %%sp, %7\n"
            : "=m"(a0), "=m"(a1), "=m"(a2), "=m"(a3),
              "=m"(a4), "=m"(a5), "=m"(a6), "=m"(sp)
        );
        
        Console::println("");
        Console::println("  CPU Registers:");
        Console::printf("  D0=%x  D1=%x  D2=%x  D3=%x\n", d0, d1, d2, d3);
        Console::printf("  D4=%x  D5=%x  D6=%x  D7=%x\n", d4, d5, d6, d7);
        Console::printf("  A0=%x  A1=%x  A2=%x  A3=%x\n", a0, a1, a2, a3);
        Console::printf("  A4=%x  A5=%x  A6=%x  SP=%x\n", a4, a5, a6, sp);
        Console::println("");
    }
    
    static void cmd_echo() {
        for (int i = 1; i < argc; i++) {
            if (i > 1) Console::putchar(' ');
            Console::print(argv[i]);
        }
        Console::println("");
    }
    
    static void cmd_ls() {
        const char *path = (argc > 1) ? argv[1] : current_dir;
        Console::printf("\nDirectory: %s\n\n", path);
        
        // Request directory listing from bus controller
        if (FileSystem::list_dir(path) == 0) {
            // Files are returned in FS_BUFFER as newline-separated names
            char *ptr = (char *)FS_BUFFER;
            while (*ptr) {
                Console::print("  ");
                while (*ptr && *ptr != '\n') {
                    Console::putchar(*ptr++);
                }
                Console::println("");
                if (*ptr == '\n') ptr++;
            }
        } else {
            Console::println("  Error reading directory");
        }
        Console::println("");
    }
    
    static void cmd_pwd() {
        Console::printf("%s\n", current_dir);
    }
    
    static void cmd_cd() {
        if (argc < 2) {
            strcpy(current_dir, "/");
        } else if (strcmp(argv[1], "..") == 0) {
            // Go up one directory
            char *last = strrchr(current_dir, '/');
            if (last && last != current_dir) {
                *last = '\0';
            } else {
                strcpy(current_dir, "/");
            }
        } else if (argv[1][0] == '/') {
            // Absolute path
            strcpy(current_dir, argv[1]);
        } else {
            // Relative path
            if (current_dir[strlen(current_dir) - 1] != '/') {
                strcat(current_dir, "/");
            }
            strcat(current_dir, argv[1]);
        }
    }
    
    static void cmd_cat() {
        if (argc < 2) {
            Console::println("Usage: cat <filename>");
            return;
        }
        
        char filepath[256];
        if (argv[1][0] == '/') {
            strcpy(filepath, argv[1]);
        } else {
            strcpy(filepath, current_dir);
            if (filepath[strlen(filepath) - 1] != '/') {
                strcat(filepath, "/");
            }
            strcat(filepath, argv[1]);
        }
        
        if (FileSystem::open(filepath) == 0) {
            char buffer[256];
            int bytes;
            while ((bytes = FileSystem::read(buffer, sizeof(buffer) - 1)) > 0) {
                buffer[bytes] = '\0';
                Console::print(buffer);
            }
            FileSystem::close();
            Console::println("");
        } else {
            Console::printf("cat: %s: No such file\n", argv[1]);
        }
    }
    
    static void cmd_hexdump() {
        if (argc < 2) {
            Console::println("Usage: hex <address> [length]");
            return;
        }
        
        unsigned long addr = 0;
        unsigned long len = 256;
        
        // Parse hex address
        const char *p = argv[1];
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        while (*p) {
            addr <<= 4;
            if (*p >= '0' && *p <= '9') addr |= *p - '0';
            else if (*p >= 'A' && *p <= 'F') addr |= *p - 'A' + 10;
            else if (*p >= 'a' && *p <= 'f') addr |= *p - 'a' + 10;
            p++;
        }
        
        if (argc > 2) {
            len = atoi(argv[2]);
        }
        
        Console::println("");
        for (unsigned long i = 0; i < len; i += 16) {
            Console::print_hex(addr + i, 8);
            Console::print(": ");
            
            // Hex bytes
            for (int j = 0; j < 16 && (i + j) < len; j++) {
                unsigned char *ptr = (unsigned char *)(addr + i + j);
                Console::print_hex(*ptr, 2);
                Console::putchar(' ');
            }
            
            // ASCII
            Console::print(" |");
            for (int j = 0; j < 16 && (i + j) < len; j++) {
                unsigned char *ptr = (unsigned char *)(addr + i + j);
                char c = *ptr;
                Console::putchar((c >= 0x20 && c < 0x7F) ? c : '.');
            }
            Console::println("|");
        }
        Console::println("");
    }
    
    static void cmd_poke() {
        if (argc < 3) {
            Console::println("Usage: poke <address> <value>");
            return;
        }
        
        unsigned long addr = 0;
        unsigned long val = 0;
        
        // Parse hex address
        const char *p = argv[1];
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        while (*p) {
            addr <<= 4;
            if (*p >= '0' && *p <= '9') addr |= *p - '0';
            else if (*p >= 'A' && *p <= 'F') addr |= *p - 'A' + 10;
            else if (*p >= 'a' && *p <= 'f') addr |= *p - 'a' + 10;
            p++;
        }
        
        // Parse hex value
        p = argv[2];
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        while (*p) {
            val <<= 4;
            if (*p >= '0' && *p <= '9') val |= *p - '0';
            else if (*p >= 'A' && *p <= 'F') val |= *p - 'A' + 10;
            else if (*p >= 'a' && *p <= 'f') val |= *p - 'a' + 10;
            p++;
        }
        
        *(volatile unsigned char *)addr = (unsigned char)val;
        Console::printf("Wrote 0x%x to address 0x%x\n", val & 0xFF, addr);
    }
    
    static void cmd_peek() {
        if (argc < 2) {
            Console::println("Usage: peek <address>");
            return;
        }
        
        unsigned long addr = 0;
        const char *p = argv[1];
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        while (*p) {
            addr <<= 4;
            if (*p >= '0' && *p <= '9') addr |= *p - '0';
            else if (*p >= 'A' && *p <= 'F') addr |= *p - 'A' + 10;
            else if (*p >= 'a' && *p <= 'f') addr |= *p - 'a' + 10;
            p++;
        }
        
        unsigned char val = *(volatile unsigned char *)addr;
        Console::printf("[0x%x] = 0x%x\n", addr, val);
    }
    
    static void cmd_run() {
        if (argc < 2) {
            Console::println("Usage: run <address>");
            return;
        }
        
        unsigned long addr = 0;
        const char *p = argv[1];
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        while (*p) {
            addr <<= 4;
            if (*p >= '0' && *p <= '9') addr |= *p - '0';
            else if (*p >= 'A' && *p <= 'F') addr |= *p - 'A' + 10;
            else if (*p >= 'a' && *p <= 'f') addr |= *p - 'a' + 10;
            p++;
        }
        
        Console::printf("Executing code at 0x%x...\n", addr);
        
        // Call the address as a function
        typedef void (*func_t)(void);
        func_t func = (func_t)addr;
        func();
        
        Console::println("Returned from subroutine.");
    }
    
    static void cmd_ping() {
        if (argc < 2) {
            Console::println("Usage: ping <hostname or IP>");
            Console::println("Example: ping 8.8.8.8");
            return;
        }
        
        const char *host = argv[1];
        Console::printf("Pinging %s...\n", host);
        
        // Parse IP address (simple format: xxx.xxx.xxx.xxx)
        unsigned long ip = 0;
        int parts[4] = {0, 0, 0, 0};
        int part_idx = 0;
        const char *p = host;
        
        while (*p && part_idx < 4) {
            if (*p >= '0' && *p <= '9') {
                parts[part_idx] = parts[part_idx] * 10 + (*p - '0');
            } else if (*p == '.') {
                part_idx++;
            } else {
                Console::println("Invalid IP address format");
                return;
            }
            p++;
        }
        
        // Build IP address (network byte order for display, host order for sending)
        ip = (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8) | parts[3];
        
        // Set up ping parameters
        *NET_ADDR_IP = ip;
        *NET_DATA_LEN = 4;  // Number of pings
        
        // Issue ping command
        *NET_COMMAND = NET_CMD_PING;
        
        // Wait for completion (with timeout)
        int timeout = 10000;  // ~10 seconds
        while (*NET_STATUS == 0 && timeout > 0) {
            timeout--;
            for (volatile int i = 0; i < 100; i++);  // Small delay
        }
        
        if (timeout == 0) {
            Console::println("Ping timeout - no response");
            return;
        }
        
        // Check result
        int result = (int)*NET_RESULT;
        if (result >= 0) {
            Console::printf("Reply from %d.%d.%d.%d: %d ms\n",
                           parts[0], parts[1], parts[2], parts[3], result);
        } else {
            Console::printf("Ping failed (error %d)\n", result);
        }
    }
    
    static void cmd_uptime() {
        unsigned long ticks = *TIMER_COUNT;
        unsigned long secs = ticks / 1000;
        unsigned long mins = secs / 60;
        unsigned long hours = mins / 60;
        
        Console::printf("Uptime: %d:%02d:%02d (%d ticks)\n",
                       hours, mins % 60, secs % 60, ticks);
    }
    
    static void cmd_reboot() {
        Console::println("Rebooting system...");
        
        // Jump to reset vector
        typedef void (*reset_t)(void);
        reset_t reset = (reset_t)0;
        reset();
    }
    
public:
    static void init() {
        running = true;
        strcpy(current_dir, "/");
    }
    
    static void execute(char *line) {
        parse_command(line);
        
        if (argc == 0) return;
        
        // Match command
        if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0) {
            cmd_help();
        }
        else if (strcmp(argv[0], "ver") == 0 || strcmp(argv[0], "version") == 0) {
            cmd_version();
        }
        else if (strcmp(argv[0], "cls") == 0 || strcmp(argv[0], "clear") == 0) {
            cmd_clear();
        }
        else if (strcmp(argv[0], "mem") == 0 || strcmp(argv[0], "memory") == 0) {
            cmd_memory();
        }
        else if (strcmp(argv[0], "regs") == 0 || strcmp(argv[0], "registers") == 0) {
            cmd_registers();
        }
        else if (strcmp(argv[0], "echo") == 0) {
            cmd_echo();
        }
        else if (strcmp(argv[0], "ls") == 0 || strcmp(argv[0], "dir") == 0) {
            cmd_ls();
        }
        else if (strcmp(argv[0], "pwd") == 0) {
            cmd_pwd();
        }
        else if (strcmp(argv[0], "cd") == 0) {
            cmd_cd();
        }
        else if (strcmp(argv[0], "cat") == 0 || strcmp(argv[0], "type") == 0) {
            cmd_cat();
        }
        else if (strcmp(argv[0], "hex") == 0 || strcmp(argv[0], "dump") == 0) {
            cmd_hexdump();
        }
        else if (strcmp(argv[0], "poke") == 0) {
            cmd_poke();
        }
        else if (strcmp(argv[0], "peek") == 0) {
            cmd_peek();
        }
        else if (strcmp(argv[0], "run") == 0 || strcmp(argv[0], "exec") == 0) {
            cmd_run();
        }
        else if (strcmp(argv[0], "ping") == 0) {
            cmd_ping();
        }
        else if (strcmp(argv[0], "uptime") == 0) {
            cmd_uptime();
        }
        else if (strcmp(argv[0], "reboot") == 0 || strcmp(argv[0], "reset") == 0) {
            cmd_reboot();
        }
        else if (strcmp(argv[0], "exit") == 0 || strcmp(argv[0], "halt") == 0) {
            Console::println("System halted.");
            running = false;
        }
        else {
            Console::printf("Unknown command: %s\n", argv[0]);
            Console::println("Type 'help' for available commands.");
        }
    }
    
    static bool is_running() {
        return running;
    }
    
    static const char *get_cwd() {
        return current_dir;
    }
};

// Static member definitions
char Shell::cmd_buffer[256];
char *Shell::argv[16];
int Shell::argc = 0;
bool Shell::running = true;
char Shell::current_dir[256];

// ============================================================================
// Operating System Main Entry Point
// ============================================================================

void os_main() {
    // Initialize subsystems
    Memory::init();
    Shell::init();
    
    // Clear screen
    Console::print("\033[2J\033[H");
    
    // Boot banner
    Console::println("");
    Console::println("  =============================================");
    Console::println("  |                                           |");
    Console::println("  |     M68K-OS v1.0 - Motorola 68000 OS      |");
    Console::println("  |                                           |");
    Console::println("  |     Running on ESP32-P4 M68K Emulator     |");
    Console::println("  |             16MB RAM @ 60MHz              |");
    Console::println("  |                                           |");
    Console::println("  =============================================");
    Console::println("");
    Console::printf("  Memory: %d KB total, %d KB free\n", 
                   Memory::get_total() / 1024, Memory::get_free() / 1024);
    Console::println("");
    Console::println("  Type 'help' for available commands.");
    Console::println("");
    
    // Command prompt buffer
    char line[256];
    
    // Main shell loop
    while (Shell::is_running()) {
        // Display prompt
        Console::printf("m68k:%s> ", Shell::get_cwd());
        
        // Read command
        Console::getline(line, sizeof(line));
        
        // Execute command
        if (strlen(line) > 0) {
            Shell::execute(line);
        }
    }
    
    // System halted
    Console::println("");
    Console::println("System halted. Press reset to restart.");
    while (1) {
        // Halt loop
    }
}

// ============================================================================
// Startup Code (_start entry point)
// ============================================================================

extern "C" void _start(void) {
    // Initialize BSS section to zero
    // IMPORTANT: Limit to 14MB maximum to stay within 16MB physical RAM
    extern unsigned long __bss_start, __bss_end;
    unsigned long *bss = &__bss_start;
    unsigned long *bss_end_safe = &__bss_end;
    
    // Safety check: never clear beyond 14MB (0x00E00000)
    unsigned long max_bss_end = 0x00E00000;
    if ((unsigned long)bss_end_safe > max_bss_end) {
        bss_end_safe = (unsigned long *)max_bss_end;
    }
    
    while (bss < bss_end_safe) {
        *bss++ = 0;
    }
    
    // Jump to main OS
    os_main();
    
    // Should never return
    while (1);
}
