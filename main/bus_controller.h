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
