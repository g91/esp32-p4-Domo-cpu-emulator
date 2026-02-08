/*
 * Virtual Bus Controller Implementation
 * Shared networking, DMA, and application loader
 */

#include "bus_controller.h"
#include "m68k_emulator.h"
#include "wifi_ftp_server.h"
#include "lcd_console.h"
#include "ps2_keyboard.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "ping/ping_sock.h"

static const char *TAG = "BusCtrl";

// Maximum sockets
#define MAX_SOCKETS 32

// Socket table
static bus_socket_t socket_table[MAX_SOCKETS];
static SemaphoreHandle_t socket_mutex = NULL;

// DMA state
static dma_transfer_t dma_state;
static SemaphoreHandle_t dma_mutex = NULL;

// Current application
static application_t current_app;
static SemaphoreHandle_t app_mutex = NULL;

// Network device state
static struct {
    uint32_t command;
    uint32_t status;
    uint32_t socket_id;
    uint32_t addr_type;
    uint32_t addr_ip;
    uint32_t addr_port;
    uint32_t data_len;
    uint32_t data_ptr;
    uint32_t flags;
    int32_t result;
} net_device;

// Filesystem device state
#define MAX_OPEN_FILES 8
#define MAX_OPEN_DIRS 4
#define FS_BUF_SIZE 512
static struct {
    char path_buf[256];      // Path string buffer (at reg 0x100)
    uint8_t data_buf[FS_BUF_SIZE];  // Data buffer (at reg 0x200)
    uint32_t command;        // Current command
    uint32_t status;         // Status register (0=busy, 1=success, 2+=error)
    int32_t result;          // Result/error code
    uint32_t file_size;      // File size (for stat/read)
    uint32_t file_pos;       // File position
    uint32_t data_ptr;       // M68K data buffer pointer
    uint32_t data_len;       // Data length
    uint32_t file_mode;      // File open mode
    uint32_t file_handle;    // File handle index
    FILE* open_files[MAX_OPEN_FILES];
    DIR* open_dirs[MAX_OPEN_DIRS];
} fs_device;

/* ====== Bus Controller Implementation ====== */

esp_err_t bus_controller_init(void)
{
    ESP_LOGI(TAG, "Initializing bus controller...");
    
    // Initialize filesystem device state
    memset(&fs_device, 0, sizeof(fs_device));
    for (int i = 0; i < MAX_OPEN_FILES; i++) {
        fs_device.open_files[i] = NULL;
    }
    for (int i = 0; i < MAX_OPEN_DIRS; i++) {
        fs_device.open_dirs[i] = NULL;
    }
    
    // Create mutexes
    socket_mutex = xSemaphoreCreateMutex();
    dma_mutex = xSemaphoreCreateMutex();
    app_mutex = xSemaphoreCreateMutex();
    
    if (!socket_mutex || !dma_mutex || !app_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_FAIL;
    }
    
    // Initialize subsystems
    esp_err_t ret;
    
    ret = network_stack_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Network stack init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = dma_controller_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DMA controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = app_loader_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "App loader init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Bus controller initialized");
    ESP_LOGI(TAG, "  I/O Base: 0x%08X", BUS_IO_BASE);
    ESP_LOGI(TAG, "  Network device: 0x%08X", BUS_IO_BASE + BUS_DEV_NETWORK);
    ESP_LOGI(TAG, "  DMA controller: 0x%08X", BUS_IO_BASE + BUS_DEV_DMA);
    
    return ESP_OK;
}

uint32_t bus_io_read(uint32_t address, uint8_t size)
{
    // Check if address is in I/O range
    if (address < BUS_IO_BASE || address >= (BUS_IO_BASE + BUS_IO_SIZE)) {
        return 0xFFFFFFFF;
    }
    
    uint32_t offset = address - BUS_IO_BASE;
    uint32_t device = offset & 0xFF000;
    uint32_t reg = offset & 0x00FFF;
    
    // Console device (0x00F02000)
    if (device == BUS_DEV_CONSOLE) {
        if (reg == 0x04) {  // Console input register
            // Read character from PS/2 keyboard or stdin
            if (ps2_keyboard_available()) {
                return ps2_keyboard_read();
            }
            int ch = fgetc(stdin);
            return (ch == EOF) ? 0 : (uint8_t)ch;
        }
        else if (reg == 0x08) {  // Console status register
            // Bit 0: RX ready (character available)
            // Bit 1: TX ready (always ready)
            uint8_t status = 0x02;  // TX always ready
            if (ps2_keyboard_available()) {
                status |= 0x01;  // RX ready if keyboard has data
            }
            return status;
        }
        return 0;
    }
    
    // Filesystem device - reads
    // Protocol: STATUS at 0x004 (0=busy, 1=success, 2+=error)
    //   FS_BUFFER at 0x200+ served from local data_buf[]
    if (device == BUS_DEV_FILESYSTEM) {
        // FS_BUFFER reads (0x200-0x3FF) - return data from local buffer
        if (reg >= 0x200 && reg < 0x200 + FS_BUF_SIZE) {
            return fs_device.data_buf[reg - 0x200];
        }
        
        switch (reg) {
            case 0x00: return fs_device.command;
            case 0x04: return fs_device.status;    // FS_STATUS
            case 0x08: return fs_device.data_len;   // FS_SIZE
            case 0x0C: return fs_device.file_pos;   // FS_OFFSET
        }
        return 0;
    }
    
    // Network device
    if (device == BUS_DEV_NETWORK) {
        switch (reg) {
            case NET_REG_COMMAND:   return net_device.command;
            case NET_REG_STATUS:    return net_device.status;
            case NET_REG_SOCKET_ID: return net_device.socket_id;
            case NET_REG_ADDR_TYPE: return net_device.addr_type;
            case NET_REG_ADDR_IP:   return net_device.addr_ip;
            case NET_REG_ADDR_PORT: return net_device.addr_port;
            case NET_REG_DATA_LEN:  return net_device.data_len;
            case NET_REG_DATA_PTR:  return net_device.data_ptr;
            case NET_REG_FLAGS:     return net_device.flags;
            case NET_REG_RESULT:    return net_device.result;
        }
    }
    
    // DMA device
    if (device == BUS_DEV_DMA) {
        xSemaphoreTake(dma_mutex, portMAX_DELAY);
        uint32_t value = 0;
        switch (reg) {
            case DMA_REG_SRC_ADDR: value = dma_state.src_addr; break;
            case DMA_REG_DST_ADDR: value = dma_state.dst_addr; break;
            case DMA_REG_LENGTH:   value = dma_state.length; break;
            case DMA_REG_STATUS:   value = dma_state.in_progress ? 0x01 : 0x00; break;
        }
        xSemaphoreGive(dma_mutex);
        return value;
    }
    
    return 0;
}

void bus_io_write(uint32_t address, uint32_t data, uint8_t size)
{
    // Check if address is in I/O range
    if (address < BUS_IO_BASE || address >= (BUS_IO_BASE + BUS_IO_SIZE)) {
        return;
    }
    
    uint32_t offset = address - BUS_IO_BASE;
    uint32_t device = offset & 0xFF000;
    uint32_t reg = offset & 0x00FFF;
    
    // Filesystem device (0x00F01000)
    // Protocol: M68K OS writes path to 0x100+, then command to 0x000
    //   STATUS (0x004): 0 = busy/processing, 1 = success, 2+ = error
    //   Results go to FS_BUFFER at offset 0x200 (newline-separated for READDIR)
    if (device == BUS_DEV_FILESYSTEM) {
        // Path/filename buffer writes (0x100-0x1FF)
        if (reg >= 0x100 && reg < 0x200) {
            int idx = reg - 0x100;
            fs_device.path_buf[idx] = (char)(data & 0xFF);
            return;
        }
        
        // FS_BUFFER writes (0x200-0x3FF) - OS writes data here for FS_CMD_WRITE
        if (reg >= 0x200 && reg < 0x200 + FS_BUF_SIZE) {
            fs_device.data_buf[reg - 0x200] = (uint8_t)(data & 0xFF);
            return;
        }
        
        switch (reg) {
            case 0x00:  // FS_COMMAND
                fs_device.command = data;
                fs_device.status = 0;  // 0 = busy (OS polls until non-zero)
                ESP_LOGI(TAG, "FS: Command 0x%02X, path='%s'", data, fs_device.path_buf);
                
                switch (data) {
                    case FS_CMD_READDIR: {
                        // List directory: read all entries into local data_buf as newline-separated names
                        // M68K OS reads result from FS_BUFFER (I/O offset 0x200+) via bus_io_read
                        char full_path[300];
                        if (fs_device.path_buf[0] == '/' && fs_device.path_buf[1] == '\0') {
                            snprintf(full_path, sizeof(full_path), "/sdcard");
                        } else {
                            snprintf(full_path, sizeof(full_path), "/sdcard%s", fs_device.path_buf);
                        }
                        
                        DIR* dir = opendir(full_path);
                        if (dir) {
                            // Write filenames into local buffer (served via bus_io_read at offset 0x200)
                            memset(fs_device.data_buf, 0, FS_BUF_SIZE);
                            uint32_t buf_offset = 0;
                            uint32_t max_buf = FS_BUF_SIZE - 1;  // Leave room for null terminator
                            
                            struct dirent *entry;
                            int count = 0;
                            while ((entry = readdir(dir)) != NULL) {
                                const char *name = entry->d_name;
                                int name_len = strlen(name);
                                
                                if (buf_offset + name_len + 1 >= max_buf) break;
                                
                                memcpy(&fs_device.data_buf[buf_offset], name, name_len);
                                buf_offset += name_len;
                                fs_device.data_buf[buf_offset++] = '\n';
                                count++;
                                ESP_LOGI(TAG, "FS: [%d] %s", count, name);
                            }
                            // Null-terminate
                            fs_device.data_buf[buf_offset] = 0;
                            fs_device.data_len = buf_offset;
                            closedir(dir);
                            
                            fs_device.status = 1;  // 1 = success
                            ESP_LOGI(TAG, "FS: READDIR '%s' success: %d entries, %d bytes", full_path, count, buf_offset);
                        } else {
                            memset(fs_device.data_buf, 0, FS_BUF_SIZE);
                            fs_device.status = 2;  // error
                            ESP_LOGW(TAG, "FS: READDIR failed on '%s': %s", full_path, strerror(errno));
                        }
                        break;
                    }
                    
                    case FS_CMD_OPEN: {
                        char full_path[300];
                        snprintf(full_path, sizeof(full_path), "/sdcard%s%s",
                                 fs_device.path_buf[0] == '/' ? "" : "/",
                                 fs_device.path_buf);
                        
                        int slot = -1;
                        for (int i = 0; i < MAX_OPEN_FILES; i++) {
                            if (fs_device.open_files[i] == NULL) { slot = i; break; }
                        }
                        
                        if (slot >= 0) {
                            FILE *f = fopen(full_path, "rb");
                            if (f) {
                                fs_device.open_files[slot] = f;
                                fs_device.file_handle = slot;
                                fseek(f, 0, SEEK_END);
                                fs_device.file_size = ftell(f);
                                fseek(f, 0, SEEK_SET);
                                fs_device.status = 1;
                                ESP_LOGI(TAG, "FS: OPEN '%s' handle=%d size=%d", full_path, slot, fs_device.file_size);
                            } else {
                                fs_device.status = 2;
                                ESP_LOGW(TAG, "FS: OPEN '%s' failed: %s", full_path, strerror(errno));
                            }
                        } else {
                            fs_device.status = 2;
                            ESP_LOGW(TAG, "FS: OPEN - no free slots");
                        }
                        break;
                    }
                    
                    case FS_CMD_CLOSE: {
                        if (fs_device.file_handle < MAX_OPEN_FILES && 
                            fs_device.open_files[fs_device.file_handle]) {
                            fclose(fs_device.open_files[fs_device.file_handle]);
                            fs_device.open_files[fs_device.file_handle] = NULL;
                            ESP_LOGI(TAG, "FS: CLOSE handle=%d", fs_device.file_handle);
                        }
                        fs_device.status = 1;
                        break;
                    }
                    
                    case FS_CMD_READ: {
                        if (fs_device.file_handle < MAX_OPEN_FILES && 
                            fs_device.open_files[fs_device.file_handle]) {
                            uint32_t to_read = fs_device.data_len;
                            if (to_read > FS_BUF_SIZE) to_read = FS_BUF_SIZE;
                            
                            memset(fs_device.data_buf, 0, FS_BUF_SIZE);
                            size_t got = fread(fs_device.data_buf, 1, to_read, 
                                              fs_device.open_files[fs_device.file_handle]);
                            fs_device.data_len = got;
                            fs_device.status = 1;
                            ESP_LOGI(TAG, "FS: READ %d bytes", got);
                        } else {
                            fs_device.status = 2;
                        }
                        break;
                    }
                    
                    default:
                        ESP_LOGW(TAG, "FS: Unknown command 0x%02X", data);
                        fs_device.status = 2;  // error
                        break;
                }
                break;
                
            case 0x08:  // FS_SIZE
                fs_device.data_len = data;
                break;
            case 0x0C:  // FS_OFFSET
                fs_device.file_pos = data;
                break;
        }
        return;
    }
    
    // Console device (0x00F02000)
    if (device == BUS_DEV_CONSOLE) {
        if (reg == 0x00) {  // Console output register
            // Write character to LCD console
            char ch = (char)(data & 0xFF);
            
            // Debug: log first few console writes
            static int write_count = 0;
            if (write_count < 20) {
                ESP_LOGI(TAG, "Console write [%d]: 0x%02X '%c'", write_count, (uint8_t)ch, 
                         (ch >= 32 && ch < 127) ? ch : '.');
                write_count++;
            }
            
            if (lcd_console_is_initialized()) {
                lcd_console_putchar(ch);
            }
            // Also echo to UART for debugging
            putchar(ch);
            fflush(stdout);
        }
        return;
    }
    
    // Network device
    if (device == BUS_DEV_NETWORK) {
        switch (reg) {
            case NET_REG_COMMAND:
                net_device.command = data;
                // Execute command when written
                if (data != 0) {
                    network_op_t op = {
                        .command = net_device.command,
                        .socket_id = net_device.socket_id,
                        .addr_ip = net_device.addr_ip,
                        .addr_port = net_device.addr_port,
                        .data_ptr = net_device.data_ptr,
                        .data_len = net_device.data_len,
                        .flags = net_device.flags
                    };
                    net_device.result = network_process_command(&op);
                    net_device.command = 0;  // Clear command
                    net_device.status = NET_STATUS_READY;
                }
                break;
            case NET_REG_SOCKET_ID: net_device.socket_id = data; break;
            case NET_REG_ADDR_TYPE: net_device.addr_type = data; break;
            case NET_REG_ADDR_IP:   net_device.addr_ip = data; break;
            case NET_REG_ADDR_PORT: net_device.addr_port = data; break;
            case NET_REG_DATA_LEN:  net_device.data_len = data; break;
            case NET_REG_DATA_PTR:  net_device.data_ptr = data; break;
            case NET_REG_FLAGS:     net_device.flags = data; break;
        }
    }
    
    // DMA device
    if (device == BUS_DEV_DMA) {
        xSemaphoreTake(dma_mutex, portMAX_DELAY);
        switch (reg) {
            case DMA_REG_SRC_ADDR:
                dma_state.src_addr = data;
                break;
            case DMA_REG_DST_ADDR:
                dma_state.dst_addr = data;
                break;
            case DMA_REG_LENGTH:
                dma_state.length = data;
                break;
            case DMA_REG_CONTROL:
                if (data & DMA_CMD_START) {
                    uint8_t direction = (data & DMA_CMD_M68K_TO_ARM) ? DMA_CMD_M68K_TO_ARM : DMA_CMD_ARM_TO_M68K;
                    dma_transfer(dma_state.src_addr, dma_state.dst_addr, dma_state.length, direction);
                }
                break;
        }
        xSemaphoreGive(dma_mutex);
    }
}

void bus_controller_process(void)
{
    // Process any pending operations
    // This would be called periodically from main loop
}

/* ====== Network Stack Implementation ====== */

esp_err_t network_stack_init(void)
{
    ESP_LOGI(TAG, "Initializing shared network stack...");
    
    // Clear socket table
    memset(socket_table, 0, sizeof(socket_table));
    
    // Initialize network device registers
    memset(&net_device, 0, sizeof(net_device));
    net_device.status = NET_STATUS_READY;
    
    ESP_LOGI(TAG, "Network stack ready (max %d sockets)", MAX_SOCKETS);
    return ESP_OK;
}

static int allocate_socket(void)
{
    xSemaphoreTake(socket_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_SOCKETS; i++) {
        if (!socket_table[i].in_use) {
            socket_table[i].in_use = true;
            xSemaphoreGive(socket_mutex);
            return i;
        }
    }
    xSemaphoreGive(socket_mutex);
    return -1;  // No free sockets
}

static void free_socket(int id)
{
    if (id >= 0 && id < MAX_SOCKETS) {
        xSemaphoreTake(socket_mutex, portMAX_DELAY);
        if (socket_table[id].socket_fd >= 0) {
            lwip_close(socket_table[id].socket_fd);
        }
        memset(&socket_table[id], 0, sizeof(bus_socket_t));
        xSemaphoreGive(socket_mutex);
    }
}

// Ping callback data
// Ping callback data
typedef struct {
    int success_count;
    uint32_t total_time;
} ping_result_t;

// Ping callbacks (must be static C functions, not lambdas)
static void ping_on_success(esp_ping_handle_t hdl, void *args) {
    ping_result_t *result = (ping_result_t*)args;
    if (!result) return;
    
    uint32_t elapsed;
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed, sizeof(elapsed));
    result->success_count++;
    result->total_time += elapsed;
    ESP_LOGI(TAG, "Ping reply: time=%d ms", elapsed);
}

static void ping_on_timeout(esp_ping_handle_t hdl, void *args) {
    ESP_LOGW(TAG, "Ping timeout");
}

static void ping_on_end(esp_ping_handle_t hdl, void *args) {
    ping_result_t *result = (ping_result_t*)args;
    if (!result) return;
    ESP_LOGI(TAG, "Ping complete: %d replies received", result->success_count);
}

int32_t network_process_command(network_op_t *op)
{
    if (!op) return -1;
    
    ESP_LOGD(TAG, "Network command: %d, socket: %d", op->command, op->socket_id);
    
    switch (op->command) {
        case NET_CMD_SOCKET: {
            // Create socket
            int id = allocate_socket();
            if (id < 0) return -1;
            
            int domain = (net_device.addr_type == NET_AF_INET6) ? AF_INET6 : AF_INET;
            int type = (net_device.addr_type == NET_SOCK_DGRAM) ? SOCK_DGRAM : SOCK_STREAM;
            
            int fd = lwip_socket(domain, type, 0);
            if (fd < 0) {
                free_socket(id);
                return -1;
            }
            
            socket_table[id].socket_fd = fd;
            socket_table[id].type = net_device.addr_type;
            socket_table[id].family = net_device.addr_type;
            
            return id;  // Return socket ID
        }
        
        case NET_CMD_CONNECT: {
            // TCP connect
            if (op->socket_id < 0 || op->socket_id >= MAX_SOCKETS) return -1;
            if (!socket_table[op->socket_id].in_use) return -1;
            
            struct sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_port = htons(op->addr_port);
            addr.sin_addr.s_addr = op->addr_ip;
            
            int ret = lwip_connect(socket_table[op->socket_id].socket_fd,
                                   (struct sockaddr*)&addr, sizeof(addr));
            if (ret == 0) {
                socket_table[op->socket_id].connected = true;
            }
            return ret;
        }
        
        case NET_CMD_BIND: {
            // Bind socket
            if (op->socket_id < 0 || op->socket_id >= MAX_SOCKETS) return -1;
            if (!socket_table[op->socket_id].in_use) return -1;
            
            struct sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_port = htons(op->addr_port);
            addr.sin_addr.s_addr = (op->addr_ip == 0) ? INADDR_ANY : op->addr_ip;
            
            return lwip_bind(socket_table[op->socket_id].socket_fd,
                            (struct sockaddr*)&addr, sizeof(addr));
        }
        
        case NET_CMD_LISTEN: {
            // Listen for connections
            if (op->socket_id < 0 || op->socket_id >= MAX_SOCKETS) return -1;
            if (!socket_table[op->socket_id].in_use) return -1;
            
            int backlog = (op->flags > 0) ? op->flags : 5;
            int ret = lwip_listen(socket_table[op->socket_id].socket_fd, backlog);
            if (ret == 0) {
                socket_table[op->socket_id].listening = true;
            }
            return ret;
        }
        
        case NET_CMD_ACCEPT: {
            // Accept connection
            if (op->socket_id < 0 || op->socket_id >= MAX_SOCKETS) return -1;
            if (!socket_table[op->socket_id].in_use) return -1;
            
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            
            int client_fd = lwip_accept(socket_table[op->socket_id].socket_fd,
                                        (struct sockaddr*)&client_addr, &addr_len);
            if (client_fd < 0) return -1;
            
            // Allocate new socket for accepted connection
            int new_id = allocate_socket();
            if (new_id < 0) {
                lwip_close(client_fd);
                return -1;
            }
            
            socket_table[new_id].socket_fd = client_fd;
            socket_table[new_id].type = socket_table[op->socket_id].type;
            socket_table[new_id].family = socket_table[op->socket_id].family;
            socket_table[new_id].connected = true;
            
            // Store client address in device registers for M68K to read
            net_device.addr_ip = client_addr.sin_addr.s_addr;
            net_device.addr_port = ntohs(client_addr.sin_port);
            
            return new_id;
        }
        
        case NET_CMD_SEND: {
            // Send data
            if (op->socket_id < 0 || op->socket_id >= MAX_SOCKETS) return -1;
            if (!socket_table[op->socket_id].in_use) return -1;
            
            // Read data from M68K memory
            uint8_t *buffer = malloc(op->data_len);
            if (!buffer) return -1;
            
            for (uint32_t i = 0; i < op->data_len; i++) {
                buffer[i] = m68k_read_memory_8(op->data_ptr + i);
            }
            
            int sent = lwip_send(socket_table[op->socket_id].socket_fd,
                                 buffer, op->data_len, 0);
            free(buffer);
            return sent;
        }
        
        case NET_CMD_RECV: {
            // Receive data
            if (op->socket_id < 0 || op->socket_id >= MAX_SOCKETS) return -1;
            if (!socket_table[op->socket_id].in_use) return -1;
            
            uint8_t *buffer = malloc(op->data_len);
            if (!buffer) return -1;
            
            int received = lwip_recv(socket_table[op->socket_id].socket_fd,
                                     buffer, op->data_len, 0);
            
            if (received > 0) {
                // Write data to M68K memory
                for (int i = 0; i < received; i++) {
                    m68k_write_memory_8(op->data_ptr + i, buffer[i]);
                }
            }
            
            free(buffer);
            return received;
        }
        
        case NET_CMD_SENDTO: {
            // Send UDP datagram
            if (op->socket_id < 0 || op->socket_id >= MAX_SOCKETS) return -1;
            if (!socket_table[op->socket_id].in_use) return -1;
            
            uint8_t *buffer = malloc(op->data_len);
            if (!buffer) return -1;
            
            for (uint32_t i = 0; i < op->data_len; i++) {
                buffer[i] = m68k_read_memory_8(op->data_ptr + i);
            }
            
            struct sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_port = htons(op->addr_port);
            addr.sin_addr.s_addr = op->addr_ip;
            
            int sent = lwip_sendto(socket_table[op->socket_id].socket_fd,
                                   buffer, op->data_len, 0,
                                   (struct sockaddr*)&addr, sizeof(addr));
            free(buffer);
            return sent;
        }
        
        case NET_CMD_RECVFROM: {
            // Receive UDP datagram
            if (op->socket_id < 0 || op->socket_id >= MAX_SOCKETS) return -1;
            if (!socket_table[op->socket_id].in_use) return -1;
            
            uint8_t *buffer = malloc(op->data_len);
            if (!buffer) return -1;
            
            struct sockaddr_in addr;
            socklen_t addr_len = sizeof(addr);
            
            int received = lwip_recvfrom(socket_table[op->socket_id].socket_fd,
                                         buffer, op->data_len, 0,
                                         (struct sockaddr*)&addr, &addr_len);
            
            if (received > 0) {
                // Write data to M68K memory
                for (int i = 0; i < received; i++) {
                    m68k_write_memory_8(op->data_ptr + i, buffer[i]);
                }
                
                // Store sender address
                net_device.addr_ip = addr.sin_addr.s_addr;
                net_device.addr_port = ntohs(addr.sin_port);
            }
            
            free(buffer);
            return received;
        }
        
        case NET_CMD_CLOSE: {
            // Close socket
            free_socket(op->socket_id);
            return 0;
        }
        
        case NET_CMD_PING: {
            // ICMP ping - M68K sends IP already in network byte order (big-endian)
            // 192.168.0.90 = 0xC0A8005A (no conversion needed)
            uint32_t ip_addr = op->addr_ip;
            
            ESP_LOGI(TAG, "Ping command: IP=0x%08X = %d.%d.%d.%d",
                     ip_addr,
                     (ip_addr >> 24) & 0xFF, (ip_addr >> 16) & 0xFF,
                     (ip_addr >> 8) & 0xFF, ip_addr & 0xFF);
            
            esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
            ping_config.target_addr.u_addr.ip4.addr = ip_addr;
            ping_config.target_addr.type = IPADDR_TYPE_V4;
            ping_config.count = (op->data_len > 0) ? op->data_len : 4;
            ping_config.interval_ms = 1000;
            ping_config.timeout_ms = 5000;
            
            // Initialize result structure explicitly
            ping_result_t result;
            memset(&result, 0, sizeof(result));
            
            esp_ping_callbacks_t cbs;
            memset(&cbs, 0, sizeof(cbs));
            cbs.on_ping_success = ping_on_success;
            cbs.on_ping_timeout = ping_on_timeout;
            cbs.on_ping_end = ping_on_end;
            cbs.cb_args = &result;
            
            esp_ping_handle_t ping;
            esp_err_t ret = esp_ping_new_session(&ping_config, &cbs, &ping);
            if (ret == ESP_OK) {
                esp_ping_start(ping);
                vTaskDelay(pdMS_TO_TICKS(ping_config.count * 1200));  // Wait for all pings
                esp_ping_stop(ping);
                
                uint32_t transmitted, received, total_time_ms;
                esp_ping_get_profile(ping, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
                esp_ping_get_profile(ping, ESP_PING_PROF_REPLY, &received, sizeof(received));
                esp_ping_get_profile(ping, ESP_PING_PROF_TIMEGAP, &total_time_ms, sizeof(total_time_ms));
                
                esp_ping_delete_session(ping);
                
                ESP_LOGI(TAG, "Ping statistics: %d/%d received, avg_time=%d ms",
                         received, transmitted, result.success_count > 0 ? result.total_time / result.success_count : 0);
                
                if (result.success_count > 0) {
                    return result.total_time / result.success_count;  // Average RTT in ms
                } else {
                    return -1;  // No replies
                }
            }
            return -2;  // Ping session creation failed
        }
        
        case NET_CMD_GETINFO: {
            // Get network info (IP address, etc.)
            // WiFi disabled for ESP32-P4-NANO (no WiFi chip)
            return -1;  // Network not available
        }
    }
    
    return -1;  // Unknown command
}

bool network_stack_is_ready(void)
{
    return (net_device.status & NET_STATUS_READY) != 0;
}

/* ====== DMA Controller Implementation ====== */

esp_err_t dma_controller_init(void)
{
    ESP_LOGI(TAG, "Initializing DMA controller...");
    memset(&dma_state, 0, sizeof(dma_state));
    ESP_LOGI(TAG, "DMA controller ready");
    return ESP_OK;
}

esp_err_t dma_transfer(uint32_t src, uint32_t dst, uint32_t len, uint8_t direction)
{
    xSemaphoreTake(dma_mutex, portMAX_DELAY);
    
    if (dma_state.in_progress) {
        xSemaphoreGive(dma_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    dma_state.src_addr = src;
    dma_state.dst_addr = dst;
    dma_state.length = len;
    dma_state.direction = direction;
    dma_state.in_progress = true;
    
    ESP_LOGD(TAG, "DMA transfer: 0x%08X -> 0x%08X, %d bytes, dir=%d",
             src, dst, len, direction);
    
    // Perform transfer
    if (direction == DMA_CMD_M68K_TO_ARM) {
        // M68K memory -> ARM memory
        uint8_t *arm_ptr = (uint8_t*)dst;
        for (uint32_t i = 0; i < len; i++) {
            arm_ptr[i] = m68k_read_memory_8(src + i);
        }
    } else {
        // ARM memory -> M68K memory
        uint8_t *arm_ptr = (uint8_t*)src;
        for (uint32_t i = 0; i < len; i++) {
            m68k_write_memory_8(dst + i, arm_ptr[i]);
        }
    }
    
    dma_state.in_progress = false;
    xSemaphoreGive(dma_mutex);
    
    return ESP_OK;
}

bool dma_is_busy(void)
{
    bool busy;
    xSemaphoreTake(dma_mutex, portMAX_DELAY);
    busy = dma_state.in_progress;
    xSemaphoreGive(dma_mutex);
    return busy;
}

/* ====== Application Loader Implementation ====== */

esp_err_t app_loader_init(void)
{
    ESP_LOGI(TAG, "Initializing application loader...");
    memset(&current_app, 0, sizeof(current_app));
    ESP_LOGI(TAG, "Application loader ready");
    return ESP_OK;
}

esp_err_t app_run_m68k(const char *filename)
{
    xSemaphoreTake(app_mutex, portMAX_DELAY);
    
    if (current_app.running) {
        ESP_LOGW(TAG, "Application already running: %s", current_app.name);
        xSemaphoreGive(app_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Loading M68K application: %s", filename);
    
    // Load binary from SD card
    char filepath[256];
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    
    FILE *f = fopen(filepath, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open: %s", filepath);
        xSemaphoreGive(app_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Get file size
    fseek(f, 0, SEEK_END);
    size_t size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    ESP_LOGI(TAG, "Loading %d bytes...", size);
    
    // Load into M68K memory at address 0x00010000
    uint32_t load_addr = 0x00010000;
    uint8_t buffer[512];
    size_t offset = 0;
    
    while (offset < size) {
        size_t to_read = (size - offset > sizeof(buffer)) ? sizeof(buffer) : (size - offset);
        size_t read = fread(buffer, 1, to_read, f);
        
        for (size_t i = 0; i < read; i++) {
            m68k_write_memory_8(load_addr + offset + i, buffer[i]);
        }
        
        offset += read;
    }
    
    fclose(f);
    
    // Set up application context
    current_app.type = APP_TYPE_M68K;
    current_app.entry_point = load_addr;
    current_app.stack_size = 0x10000;  // 64KB stack
    current_app.running = true;
    strncpy(current_app.name, filename, sizeof(current_app.name) - 1);
    
    // Reset M68K CPU and set PC to entry point
    m68k_reset();
    m68k_set_reg(M68K_REG_PC, load_addr);
    m68k_set_reg(M68K_REG_SP, 0x00FFFFFC);  // Stack at top of RAM
    
    ESP_LOGI(TAG, "M68K application loaded: %s", filename);
    ESP_LOGI(TAG, "  Entry: 0x%08X", load_addr);
    ESP_LOGI(TAG, "  Stack: 0x%08X", 0x00FFFFFC);
    
    xSemaphoreGive(app_mutex);
    return ESP_OK;
}

esp_err_t app_run_arm(void (*entry_point)(void), const char *name)
{
    xSemaphoreTake(app_mutex, portMAX_DELAY);
    
    if (current_app.running) {
        ESP_LOGW(TAG, "Application already running: %s", current_app.name);
        xSemaphoreGive(app_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting ARM native application: %s", name);
    
    current_app.type = APP_TYPE_ARM_NATIVE;
    current_app.entry_point = (uint32_t)entry_point;
    current_app.running = true;
    strncpy(current_app.name, name, sizeof(current_app.name) - 1);
    
    xSemaphoreGive(app_mutex);
    
    // Run in separate task
    xTaskCreate((TaskFunction_t)entry_point, name, 8192, NULL, 5, NULL);
    
    return ESP_OK;
}

application_t* app_get_current(void)
{
    return &current_app;
}

void app_stop(void)
{
    xSemaphoreTake(app_mutex, portMAX_DELAY);
    
    if (current_app.running) {
        ESP_LOGI(TAG, "Stopping application: %s", current_app.name);
        
        if (current_app.type == APP_TYPE_M68K) {
            // Stop M68K execution
            m68k_reset();
        }
        
        current_app.running = false;
    }
    
    xSemaphoreGive(app_mutex);
}

/* ====== System Call Handler ====== */

void bus_handle_trap(uint8_t trap_number)
{
    switch (trap_number) {
        case 0: {
            // Network operations - registers contain parameters
            // This would be called from M68K emulator when TRAP #0 is executed
            ESP_LOGD(TAG, "TRAP #0: Network syscall");
            break;
        }
        
        case 1: {
            // File I/O
            ESP_LOGD(TAG, "TRAP #1: File I/O syscall");
            break;
        }
        
        case 2: {
            // Console I/O
            ESP_LOGD(TAG, "TRAP #2: Console I/O syscall");
            break;
        }
        
        case 3: {
            // DMA operations
            ESP_LOGD(TAG, "TRAP #3: DMA syscall");
            break;
        }
    }
}
