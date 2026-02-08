/*
 * Virtual Bus Controller for M68K/ARM Shared Resources
 * Provides DMA, memory-mapped I/O, and device access for both architectures
 */

#ifndef BUS_CONTROLLER_H
#define BUS_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Bus controller memory map
#define BUS_IO_BASE         0x00F00000  // I/O device base (M68K address space)
#define BUS_IO_SIZE         0x00100000  // 1MB I/O space

// Device offsets within I/O space
#define BUS_DEV_NETWORK     0x00000     // Network stack interface
#define BUS_DEV_FILESYSTEM  0x01000     // Filesystem interface
#define BUS_DEV_CONSOLE     0x02000     // Console I/O
#define BUS_DEV_DMA         0x03000     // DMA controller
#define BUS_DEV_IRQ         0x04000     // Interrupt controller
#define BUS_DEV_TIMER       0x05000     // Timer device
#define BUS_DEV_FPU         0x06000     // Floating point coprocessor
#define BUS_DEV_AUDIO       0x07000     // Sound Blaster 16 audio device
#define BUS_DEV_VIDEO       0x08000     // Video/GPU framebuffer device

// Network device registers (offset from BUS_DEV_NETWORK)
#define NET_REG_COMMAND     0x00    // Command register
#define NET_REG_STATUS      0x04    // Status register
#define NET_REG_SOCKET_ID   0x08    // Socket ID
#define NET_REG_ADDR_TYPE   0x0C    // Address type (IPv4/IPv6)
#define NET_REG_ADDR_IP     0x10    // IP address (4 bytes)
#define NET_REG_ADDR_PORT   0x14    // Port number
#define NET_REG_DATA_LEN    0x18    // Data length
#define NET_REG_DATA_PTR    0x1C    // Data buffer pointer (M68K address)
#define NET_REG_FLAGS       0x20    // Operation flags
#define NET_REG_RESULT      0x24    // Operation result

// Network commands
#define NET_CMD_SOCKET      0x01    // Create socket
#define NET_CMD_BIND        0x02    // Bind to address
#define NET_CMD_LISTEN      0x03    // Listen for connections
#define NET_CMD_ACCEPT      0x04    // Accept connection
#define NET_CMD_CONNECT     0x05    // Connect to remote
#define NET_CMD_SEND        0x06    // Send data
#define NET_CMD_RECV        0x07    // Receive data
#define NET_CMD_SENDTO      0x08    // Send UDP datagram
#define NET_CMD_RECVFROM    0x09    // Receive UDP datagram
#define NET_CMD_CLOSE       0x0A    // Close socket
#define NET_CMD_GETSOCKOPT  0x0B    // Get socket option
#define NET_CMD_SETSOCKOPT  0x0C    // Set socket option
#define NET_CMD_GETINFO     0x0D    // Get network info (IP, DNS, etc.)
#define NET_CMD_PING        0x0E    // ICMP ping

// Socket types
#define NET_SOCK_STREAM     0x01    // TCP
#define NET_SOCK_DGRAM      0x02    // UDP
#define NET_SOCK_RAW        0x03    // Raw socket

// Address families
#define NET_AF_INET         0x02    // IPv4
#define NET_AF_INET6        0x0A    // IPv6

// Status flags
#define NET_STATUS_READY    0x01    // Ready for operation
#define NET_STATUS_BUSY     0x02    // Operation in progress
#define NET_STATUS_ERROR    0x04    // Error occurred
#define NET_STATUS_CONNECTED 0x08   // Socket connected
#define NET_STATUS_LISTENING 0x10   // Socket listening

// Filesystem device registers (offset from BUS_DEV_FILESYSTEM)
#define FS_REG_COMMAND      0x00    // Command register
#define FS_REG_STATUS       0x04    // Status register
#define FS_REG_RESULT       0x08    // Result/error code
#define FS_REG_FILE_SIZE    0x0C    // File size (for stat/read)
#define FS_REG_FILE_POS     0x10    // File position (for seek)
#define FS_REG_PATH_BUF     0x100   // Path string buffer (256 bytes)
#define FS_REG_DATA_PTR     0x200   // Data buffer pointer (M68K address)
#define FS_REG_DATA_LEN     0x204   // Data length
#define FS_REG_FILE_MODE    0x208   // File open mode
#define FS_REG_FILE_HANDLE  0x20C   // File handle (index)

// Filesystem commands (matching m68k_os.cpp definitions)
#define FS_CMD_OPEN         0x01    // Open file
#define FS_CMD_CLOSE        0x02    // Close file
#define FS_CMD_READ         0x03    // Read from file
#define FS_CMD_WRITE        0x04    // Write to file
#define FS_CMD_SEEK         0x05    // Seek file position
#define FS_CMD_STAT         0x06    // Get file info
#define FS_CMD_READDIR      0x07    // List directory contents
#define FS_CMD_MKDIR        0x08    // Create directory
#define FS_CMD_REMOVE       0x09    // Delete file/directory

// Filesystem status flags (bit fields)
#define FS_STATUS_READY     0x01    // Device ready
#define FS_STATUS_COMPLETE  0x02    // Command complete
#define FS_STATUS_ERROR     0x04    // Error occurred
#define FS_STATUS_EOF       0x08    // End of file/directory

// Filesystem result codes (separate from status)
#define FS_RESULT_OK        0       // Success
#define FS_RESULT_ERROR     -1      // Generic error
#define FS_RESULT_NOTFOUND  -2      // File not found
#define FS_RESULT_EXISTS    -3      // File already exists
#define FS_RESULT_NOSPACE   -4      // No space left

// DMA controller registers
#define DMA_REG_CONTROL     0x00    // DMA control
#define DMA_REG_SRC_ADDR    0x04    // Source address
#define DMA_REG_DST_ADDR    0x08    // Destination address
#define DMA_REG_LENGTH      0x0C    // Transfer length
#define DMA_REG_STATUS      0x10    // Transfer status

// DMA commands
#define DMA_CMD_START       0x01    // Start transfer
#define DMA_CMD_ABORT       0x02    // Abort transfer
#define DMA_CMD_M68K_TO_ARM 0x10    // M68K memory -> ARM memory
#define DMA_CMD_ARM_TO_M68K 0x20    // ARM memory -> M68K memory

// ============================================================================
// FPU Coprocessor Registers (offset from BUS_DEV_FPU = 0x06000)
// ============================================================================
// IEEE 754 double-precision math coprocessor
// Usage: Write operands, write operation, read result
// All 64-bit values use two 32-bit registers (HI/LO, big-endian)

#define FPU_REG_OP_A_HI     0x00    // Operand A high 32 bits
#define FPU_REG_OP_A_LO     0x04    // Operand A low 32 bits
#define FPU_REG_OP_B_HI     0x08    // Operand B high 32 bits
#define FPU_REG_OP_B_LO     0x0C    // Operand B low 32 bits
#define FPU_REG_COMMAND      0x10    // Operation command (triggers execution)
#define FPU_REG_STATUS       0x14    // Status register
#define FPU_REG_RESULT_HI    0x18    // Result high 32 bits
#define FPU_REG_RESULT_LO    0x1C    // Result low 32 bits
#define FPU_REG_INT_RESULT   0x20    // Integer result (for FTOI, comparisons)
#define FPU_REG_CONTROL      0x24    // Control register (rounding mode, etc.)

// FPU operations (written to FPU_REG_COMMAND)
#define FPU_CMD_ADD          0x01    // result = A + B
#define FPU_CMD_SUB          0x02    // result = A - B
#define FPU_CMD_MUL          0x03    // result = A * B
#define FPU_CMD_DIV          0x04    // result = A / B
#define FPU_CMD_SQRT         0x05    // result = sqrt(A)
#define FPU_CMD_ABS          0x06    // result = |A|
#define FPU_CMD_NEG          0x07    // result = -A
#define FPU_CMD_SIN          0x08    // result = sin(A)
#define FPU_CMD_COS          0x09    // result = cos(A)
#define FPU_CMD_TAN          0x0A    // result = tan(A)
#define FPU_CMD_ATAN         0x0B    // result = atan(A)
#define FPU_CMD_ATAN2        0x0C    // result = atan2(A, B)
#define FPU_CMD_LOG          0x0D    // result = log(A) (natural)
#define FPU_CMD_LOG10        0x0E    // result = log10(A)
#define FPU_CMD_EXP          0x0F    // result = exp(A)
#define FPU_CMD_POW          0x10    // result = A^B
#define FPU_CMD_FMOD         0x11    // result = fmod(A, B)
#define FPU_CMD_FLOOR        0x12    // result = floor(A)
#define FPU_CMD_CEIL         0x13    // result = ceil(A)
#define FPU_CMD_ROUND        0x14    // result = round(A)
#define FPU_CMD_ITOF         0x15    // result = (double)INT_RESULT  (int to float)
#define FPU_CMD_FTOI         0x16    // INT_RESULT = (int)A          (float to int)
#define FPU_CMD_CMP          0x17    // INT_RESULT = compare(A, B) (-1,0,1)
#define FPU_CMD_ASIN         0x18    // result = asin(A)
#define FPU_CMD_ACOS         0x19    // result = acos(A)
#define FPU_CMD_SINH         0x1A    // result = sinh(A)
#define FPU_CMD_COSH         0x1B    // result = cosh(A)
#define FPU_CMD_TANH         0x1C    // result = tanh(A)
#define FPU_CMD_PI           0x20    // result = M_PI
#define FPU_CMD_E            0x21    // result = M_E

// FPU status bits
#define FPU_STATUS_READY     0x01    // FPU ready (always 1 on ESP32, instant execution)
#define FPU_STATUS_ZERO      0x02    // Result is zero
#define FPU_STATUS_NEG       0x04    // Result is negative
#define FPU_STATUS_INF       0x08    // Result is infinity
#define FPU_STATUS_NAN       0x10    // Result is NaN
#define FPU_STATUS_OVERFLOW  0x20    // Overflow occurred
#define FPU_STATUS_UNDERFLOW 0x40    // Underflow occurred
#define FPU_STATUS_DIVZERO   0x80    // Division by zero

// ============================================================================
// Audio Device Registers (offset from BUS_DEV_AUDIO = 0x07000)
// ============================================================================
// Sound Blaster 16 compatible audio output device
// M68K writes PCM samples to buffer, ESP32 plays via I2S/DAC

#define AUD_REG_COMMAND      0x00    // Audio command register
#define AUD_REG_STATUS       0x04    // Status register
#define AUD_REG_FORMAT       0x08    // Audio format (8/16 bit, mono/stereo)
#define AUD_REG_SAMPLE_RATE  0x0C    // Sample rate in Hz
#define AUD_REG_VOLUME       0x10    // Master volume (0-255)
#define AUD_REG_VOLUME_L     0x14    // Left channel volume (0-255)
#define AUD_REG_VOLUME_R     0x18    // Right channel volume (0-255)
#define AUD_REG_BUF_ADDR     0x1C    // M68K address of audio DMA buffer
#define AUD_REG_BUF_SIZE     0x20    // Buffer size in bytes
#define AUD_REG_BUF_POS      0x24    // Current playback position (read-only)
#define AUD_REG_CHANNELS     0x28    // Number of channels (1=mono, 2=stereo)
#define AUD_REG_BITS         0x2C    // Bits per sample (8 or 16)
#define AUD_REG_IRQ_AT       0x30    // Fire IRQ when buffer reaches this position
// SB16-style DSP registers (offset 0x100)
#define AUD_SB_RESET         0x100   // DSP reset (write 1 to reset)
#define AUD_SB_READ_DATA     0x104   // DSP read data port
#define AUD_SB_WRITE_CMD     0x108   // DSP write command/data port
#define AUD_SB_WRITE_STATUS  0x10C   // DSP write status (bit 7 = ready)
#define AUD_SB_READ_STATUS   0x110   // DSP read status (bit 7 = data available)
#define AUD_SB_MIXER_ADDR    0x114   // Mixer address port
#define AUD_SB_MIXER_DATA    0x118   // Mixer data port
// Audio buffer at offset 0x200 (2KB DMA buffer)
#define AUD_BUFFER_OFFSET    0x200
#define AUD_BUFFER_SIZE      2048

// Audio commands
#define AUD_CMD_INIT         0x01    // Initialize audio device
#define AUD_CMD_PLAY         0x02    // Start playback
#define AUD_CMD_STOP         0x03    // Stop playback
#define AUD_CMD_PAUSE        0x04    // Pause playback
#define AUD_CMD_RESUME       0x05    // Resume playback
#define AUD_CMD_SET_FORMAT   0x06    // Set audio format
#define AUD_CMD_BEEP         0x07    // Play a beep tone (freq in BUF_SIZE, dur in BUF_POS)
#define AUD_CMD_TONE         0x08    // Play a tone (freq in SAMPLE_RATE, dur in BUF_SIZE)

// Audio format values
#define AUD_FMT_U8_MONO     0x00    // 8-bit unsigned mono
#define AUD_FMT_U8_STEREO   0x01    // 8-bit unsigned stereo
#define AUD_FMT_S16_MONO    0x10    // 16-bit signed mono
#define AUD_FMT_S16_STEREO  0x11    // 16-bit signed stereo

// Audio status bits
#define AUD_STATUS_READY     0x01    // Device initialized
#define AUD_STATUS_PLAYING   0x02    // Currently playing
#define AUD_STATUS_PAUSED    0x04    // Playback paused
#define AUD_STATUS_BUF_EMPTY 0x08    // Buffer underrun
#define AUD_STATUS_IRQ       0x10    // IRQ pending
#define AUD_STATUS_ERROR     0x80    // Error occurred

// ============================================================================
// Video/GPU Device Registers (offset from BUS_DEV_VIDEO = 0x08000)
// ============================================================================
// Framebuffer-based video device with 2D acceleration
// Supports text mode, 8-bit indexed, 16-bit RGB565, 24-bit RGB

#define VID_REG_COMMAND      0x00    // Command register
#define VID_REG_STATUS       0x04    // Status register
#define VID_REG_MODE         0x08    // Video mode
#define VID_REG_WIDTH        0x0C    // Screen width in pixels
#define VID_REG_HEIGHT       0x10    // Screen height in pixels
#define VID_REG_BPP          0x14    // Bits per pixel
#define VID_REG_PITCH        0x18    // Bytes per scanline
#define VID_REG_FB_ADDR      0x1C    // Framebuffer address in M68K RAM
#define VID_REG_FB_SIZE      0x20    // Framebuffer size in bytes
#define VID_REG_CURSOR_X     0x24    // Text cursor X position
#define VID_REG_CURSOR_Y     0x28    // Text cursor Y position
#define VID_REG_FG_COLOR     0x2C    // Foreground color
#define VID_REG_BG_COLOR     0x30    // Background color
#define VID_REG_DRAW_X       0x34    // Draw X coordinate
#define VID_REG_DRAW_Y       0x38    // Draw Y coordinate
#define VID_REG_DRAW_W       0x3C    // Draw width
#define VID_REG_DRAW_H       0x40    // Draw height
#define VID_REG_DRAW_COLOR   0x44    // Draw color
#define VID_REG_SRC_X        0x48    // Blit source X
#define VID_REG_SRC_Y        0x4C    // Blit source Y
#define VID_REG_FONT_ADDR    0x50    // Font data address in M68K RAM
#define VID_REG_SCROLL_Y     0x54    // Vertical scroll offset
#define VID_REG_VBLANK       0x58    // VBlank counter (read-only, increments each frame)
// Palette registers at offset 0x100 (256 entries × 4 bytes = 1KB)
#define VID_PALETTE_OFFSET   0x100
#define VID_PALETTE_SIZE     256

// Video commands
#define VID_CMD_INIT         0x01    // Initialize video mode
#define VID_CMD_CLEAR        0x02    // Clear framebuffer
#define VID_CMD_FLIP         0x03    // Flip/refresh display from framebuffer
#define VID_CMD_FILL_RECT    0x04    // Fill rectangle (X,Y,W,H,COLOR)
#define VID_CMD_BLIT         0x05    // Blit rectangle (SRC_X,SRC_Y to X,Y, W×H)
#define VID_CMD_DRAW_LINE    0x06    // Draw line (X,Y to DRAW_W,DRAW_H, COLOR)
#define VID_CMD_DRAW_PIXEL   0x07    // Draw single pixel (X,Y,COLOR)
#define VID_CMD_DRAW_CHAR    0x08    // Draw character at cursor (char in DRAW_COLOR)
#define VID_CMD_DRAW_TEXT    0x09    // Draw text string from M68K address
#define VID_CMD_SET_PALETTE  0x0A    // Set palette entry (index in DRAW_X, RGB in DRAW_COLOR)
#define VID_CMD_SCROLL       0x0B    // Scroll framebuffer by SCROLL_Y pixels
#define VID_CMD_HLINE        0x0C    // Fast horizontal line
#define VID_CMD_VLINE        0x0D    // Fast vertical line
#define VID_CMD_CIRCLE       0x0E    // Draw circle (X,Y center, W=radius)
#define VID_CMD_FILL_CIRCLE  0x0F    // Fill circle

// Video modes
#define VID_MODE_TEXT_80x25  0x03    // CGA text mode (80×25, 16 colors)
#define VID_MODE_320x200x8   0x13    // VGA mode 13h (320×200, 256 colors)
#define VID_MODE_320x240x16  0x50    // 320×240, RGB565
#define VID_MODE_480x320x16  0x51    // 480×320, RGB565 (matches SPI LCD)
#define VID_MODE_640x480x8   0x52    // 640×480, 256 colors
#define VID_MODE_800x480x16  0x53    // 800×480, RGB565 (matches DSI LCD)

// Video status bits
#define VID_STATUS_READY     0x01    // Device initialized
#define VID_STATUS_VBLANK    0x02    // In vertical blanking period
#define VID_STATUS_BUSY      0x04    // GPU operation in progress
#define VID_STATUS_TEXT_MODE 0x08    // Text mode active
#define VID_STATUS_GFX_MODE  0x10    // Graphics mode active

// Bus transaction structure
typedef struct {
    uint32_t address;       // Target address
    uint32_t data;          // Data to write/read
    uint32_t size;          // Transaction size (1, 2, 4 bytes)
    bool is_write;          // true = write, false = read
    bool is_m68k;           // true = M68K access, false = ARM access
} bus_transaction_t;

// Network socket structure
typedef struct {
    int socket_fd;          // Underlying LWIP socket
    uint8_t type;           // SOCK_STREAM, SOCK_DGRAM
    uint8_t family;         // AF_INET, AF_INET6
    bool in_use;            // Socket allocated
    bool connected;         // TCP connection established
    bool listening;         // TCP listening
} bus_socket_t;

// Network operation structure
typedef struct {
    uint8_t command;        // Network command
    int socket_id;          // Socket ID (0-31)
    uint32_t addr_ip;       // IP address
    uint16_t addr_port;     // Port number
    uint32_t data_ptr;      // M68K pointer to data buffer
    uint32_t data_len;      // Length of data
    uint32_t flags;         // Operation flags
    int32_t result;         // Operation result
} network_op_t;

// DMA transfer structure
typedef struct {
    uint32_t src_addr;      // Source address
    uint32_t dst_addr;      // Destination address
    uint32_t length;        // Transfer length
    uint8_t direction;      // M68K->ARM or ARM->M68K
    bool in_progress;       // Transfer active
} dma_transfer_t;

// Application type
typedef enum {
    APP_TYPE_M68K,          // M68K binary
    APP_TYPE_ARM_NATIVE     // ARM native binary
} app_type_t;

// Application structure
typedef struct {
    app_type_t type;        // Application type
    uint32_t entry_point;   // Entry point address
    uint32_t stack_size;    // Stack size
    void *context;          // Application-specific context
    bool running;           // Application is running
    char name[32];          // Application name
} application_t;

/* ====== Bus Controller API ====== */

/**
 * Initialize the bus controller
 */
esp_err_t bus_controller_init(void);

/**
 * Handle bus I/O read from M68K
 */
uint32_t bus_io_read(uint32_t address, uint8_t size);

/**
 * Handle bus I/O write from M68K
 */
void bus_io_write(uint32_t address, uint32_t data, uint8_t size);

/**
 * Process pending bus operations
 */
void bus_controller_process(void);

/* ====== Network Stack API ====== */

/**
 * Initialize shared network stack
 */
esp_err_t network_stack_init(void);

/**
 * Process network commands from M68K
 */
int32_t network_process_command(network_op_t *op);

/**
 * Get network stack status
 */
bool network_stack_is_ready(void);

/* ====== DMA Controller API ====== */

/**
 * Initialize DMA controller
 */
esp_err_t dma_controller_init(void);

/**
 * Start DMA transfer between M68K and ARM memory spaces
 */
esp_err_t dma_transfer(uint32_t src, uint32_t dst, uint32_t len, uint8_t direction);

/**
 * Check if DMA transfer is complete
 */
bool dma_is_busy(void);

/* ====== Application Loader API ====== */

/**
 * Initialize application loader
 */
esp_err_t app_loader_init(void);

/**
 * Load and run M68K application
 */
esp_err_t app_run_m68k(const char *filename);

/**
 * Load and run ARM native application
 */
esp_err_t app_run_arm(void (*entry_point)(void), const char *name);

/**
 * Get current running application
 */
application_t* app_get_current(void);

/**
 * Stop current application
 */
void app_stop(void);

/* ====== System Call Interface for M68K ====== */

/**
 * Handle M68K TRAP instruction for system calls
 * TRAP #0 - Network operations
 * TRAP #1 - File I/O
 * TRAP #2 - Console I/O
 * TRAP #3 - DMA operations
 */
void bus_handle_trap(uint8_t trap_number);

#endif // BUS_CONTROLLER_H
