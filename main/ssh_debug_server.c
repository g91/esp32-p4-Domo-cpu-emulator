/*
 * SSH Debug Server for ESP32-P4 M68K Emulator
 * 
 * Provides telnet-based remote console access for debugging.
 * (Using telnet instead of full SSH for simplicity - can upgrade to libssh later)
 * 
 * Features:
 * - Remote console access on port 23 (telnet)
 * - Same commands as UART console
 * - Access M68K dump, crash, state commands
 * - Multiple simultaneous connections supported
 */

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "ssh_debug_server.h"
#include "m68k_emulator.h"

static const char *TAG = "SSH_DBG";

#define TELNET_PORT 23
#define MAX_CLIENTS 2
#define RX_BUFFER_SIZE 1024

typedef struct {
    int socket;
    bool active;
    char rx_buffer[RX_BUFFER_SIZE];
    int rx_pos;
} telnet_client_t;

static TaskHandle_t telnet_task_handle = NULL;
static bool server_running = false;
static int listen_socket = -1;
static telnet_client_t clients[MAX_CLIENTS] = {0};

// Send string to telnet client
static void telnet_send(int client_idx, const char *str) {
    if (clients[client_idx].active && clients[client_idx].socket >= 0) {
        send(clients[client_idx].socket, str, strlen(str), 0);
    }
}

// Send formatted string to telnet client
static void telnet_printf(int client_idx, const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    telnet_send(client_idx, buffer);
}

// Process command from telnet client
static void process_telnet_command(int client_idx, const char *cmd) {
    // Echo the command
    telnet_printf(client_idx, "%s\r\n", cmd);
    
    // Parse and execute command
    if (strlen(cmd) == 0) {
        return;
    }
    
    // Handle built-in telnet commands
    if (strcmp(cmd, "exit") == 0 || strcmp(cmd, "quit") == 0) {
        telnet_send(client_idx, "Goodbye!\r\n");
        clients[client_idx].active = false;
        return;
    }
    
    if (strcmp(cmd, "help") == 0) {
        telnet_send(client_idx, "\r\n");
        telnet_send(client_idx, "=== ESP32-P4 M68K Debug Console ===\r\n");
        telnet_send(client_idx, "\r\n");
        telnet_send(client_idx, "M68K Emulator Commands:\r\n");
        telnet_send(client_idx, "  crash          - Show crash context\r\n");
        telnet_send(client_idx, "  dump ADDR LEN  - Dump memory (addr in hex)\r\n");
        telnet_send(client_idx, "  state          - Show CPU state\r\n");
        telnet_send(client_idx, "  reset          - Reset M68K CPU\r\n");
        telnet_send(client_idx, "  run N          - Run N instructions\r\n");
        telnet_send(client_idx, "  step           - Execute one instruction\r\n");
        telnet_send(client_idx, "\r\n");
        telnet_send(client_idx, "Telnet Commands:\r\n");
        telnet_send(client_idx, "  help           - Show this help\r\n");
        telnet_send(client_idx, "  exit/quit      - Close connection\r\n");
        telnet_send(client_idx, "\r\n");
        return;
    }
    
    // Handle M68K commands
    if (strcmp(cmd, "crash") == 0) {
        // Call the function - it will print to stdout (captured in UART)
        m68k_show_crash_context();
        telnet_send(client_idx, "(Crash data sent to UART console - check serial monitor)\r\n");
        telnet_send(client_idx, "Note: Full crash details available in UART log\r\n");
        return;
    }
    
    if (strncmp(cmd, "dump ", 5) == 0) {
        uint32_t addr = 0;
        uint32_t length = 256;
        sscanf(cmd + 5, "%lx %lu", &addr, &length);
        
        // Call the function - output goes to stdout
        m68k_dump_memory(addr, length);
        telnet_send(client_idx, "(Memory dump sent to UART console - check serial monitor)\r\n");
        telnet_printf(client_idx, "Dumped %lu bytes from 0x%08lX\r\n", length, addr);
        return;
    }
    
    if (strcmp(cmd, "state") == 0) {
        char state_buf[1024];
        m68k_get_state(state_buf, sizeof(state_buf));
        telnet_printf(client_idx, "%s\r\n", state_buf);
        return;
    }
    
    if (strcmp(cmd, "reset") == 0) {
        m68k_reset();
        telnet_send(client_idx, "M68K CPU reset\r\n");
        return;
    }
    
    if (strncmp(cmd, "run ", 4) == 0) {
        int count = atoi(cmd + 4);
        if (count > 0) {
            m68k_run(count);
            telnet_printf(client_idx, "Executed %d instructions\r\n", count);
        }
        return;
    }
    
    if (strcmp(cmd, "step") == 0) {
        m68k_step();
        telnet_send(client_idx, "Step executed\r\n");
        return;
    }
    
    // Unknown command
    telnet_printf(client_idx, "Unknown command: %s\r\n", cmd);
    telnet_send(client_idx, "Type 'help' for available commands\r\n");
}

// Handle telnet client
static void handle_telnet_client(int client_idx) {
    char rx_buf[128];
    int len;
    
    while (clients[client_idx].active) {
        len = recv(clients[client_idx].socket, rx_buf, sizeof(rx_buf) - 1, 0);
        
        if (len <= 0) {
            // Connection closed or error
            clients[client_idx].active = false;
            break;
        }
        
        // Process received data
        for (int i = 0; i < len; i++) {
            char c = rx_buf[i];
            
            // Handle backspace
            if (c == '\b' || c == 127) {
                if (clients[client_idx].rx_pos > 0) {
                    clients[client_idx].rx_pos--;
                    telnet_send(client_idx, "\b \b"); // Erase character
                }
                continue;
            }
            
            // Handle newline
            if (c == '\r' || c == '\n') {
                clients[client_idx].rx_buffer[clients[client_idx].rx_pos] = '\0';
                telnet_send(client_idx, "\r\n");
                
                // Process command
                process_telnet_command(client_idx, clients[client_idx].rx_buffer);
                
                // Reset buffer
                clients[client_idx].rx_pos = 0;
                
                // Send prompt
                if (clients[client_idx].active) {
                    telnet_send(client_idx, "M68K-DBG> ");
                }
                continue;
            }
            
            // Add character to buffer
            if (clients[client_idx].rx_pos < RX_BUFFER_SIZE - 1 && c >= 32 && c < 127) {
                clients[client_idx].rx_buffer[clients[client_idx].rx_pos++] = c;
                // Echo character
                char echo[2] = {c, '\0'};
                telnet_send(client_idx, echo);
            }
        }
    }
    
    // Cleanup
    if (clients[client_idx].socket >= 0) {
        close(clients[client_idx].socket);
        clients[client_idx].socket = -1;
    }
    ESP_LOGI(TAG, "Client %d disconnected", client_idx);
}

// Telnet server task
static void telnet_server_task(void *pvParameters) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int client_sock;
    
    ESP_LOGI(TAG, "Telnet debug server starting on port %d", TELNET_PORT);
    
    // Create socket
    listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        server_running = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Set socket options
    int opt = 1;
    setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Bind socket
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TELNET_PORT);
    
    if (bind(listen_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(listen_socket);
        server_running = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Listen
    if (listen(listen_socket, 2) < 0) {
        ESP_LOGE(TAG, "Socket listen failed: errno %d", errno);
        close(listen_socket);
        server_running = false;
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Telnet debug server listening on port %d", TELNET_PORT);
    ESP_LOGI(TAG, "Connect with: telnet <ESP32_IP> 23");
    
    server_running = true;
    
    while (server_running) {
        // Accept new connection
        client_sock = accept(listen_socket, (struct sockaddr *)&client_addr, &addr_len);
        
        if (client_sock < 0) {
            if (server_running) {
                ESP_LOGE(TAG, "Accept failed: errno %d", errno);
            }
            continue;
        }
        
        // Find free client slot
        int client_idx = -1;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].active) {
                client_idx = i;
                break;
            }
        }
        
        if (client_idx < 0) {
            // No free slots
            const char *msg = "Server full. Try again later.\r\n";
            send(client_sock, msg, strlen(msg), 0);
            close(client_sock);
            ESP_LOGW(TAG, "Connection rejected - server full");
            continue;
        }
        
        // Setup client
        clients[client_idx].socket = client_sock;
        clients[client_idx].active = true;
        clients[client_idx].rx_pos = 0;
        
        ESP_LOGI(TAG, "Client %d connected from %s", client_idx, 
                 inet_ntoa(client_addr.sin_addr));
        
        // Send welcome message
        telnet_send(client_idx, "\r\n");
        telnet_send(client_idx, "=================================\r\n");
        telnet_send(client_idx, "  ESP32-P4 M68K Debug Console\r\n");
        telnet_send(client_idx, "=================================\r\n");
        telnet_send(client_idx, "\r\n");
        telnet_send(client_idx, "Type 'help' for commands\r\n");
        telnet_send(client_idx, "\r\n");
        telnet_send(client_idx, "M68K-DBG> ");
        
        // Handle client (blocking)
        handle_telnet_client(client_idx);
    }
    
    // Cleanup
    if (listen_socket >= 0) {
        close(listen_socket);
    }
    
    ESP_LOGI(TAG, "Telnet debug server stopped");
    vTaskDelete(NULL);
}

esp_err_t ssh_debug_server_init(void) {
    if (server_running) {
        ESP_LOGW(TAG, "SSH debug server already running");
        return ESP_OK;
    }
    
    // Initialize clients
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].active = false;
        clients[i].socket = -1;
        clients[i].rx_pos = 0;
    }
    
    // Create telnet server task
    BaseType_t ret = xTaskCreate(
        telnet_server_task,
        "telnet_server",
        8192,
        NULL,
        5,
        &telnet_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telnet server task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "SSH debug server initialized");
    return ESP_OK;
}

void ssh_debug_server_stop(void) {
    if (!server_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping SSH debug server...");
    server_running = false;
    
    // Close all client connections
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].active && clients[i].socket >= 0) {
            close(clients[i].socket);
            clients[i].socket = -1;
            clients[i].active = false;
        }
    }
    
    // Close listen socket to unblock accept()
    if (listen_socket >= 0) {
        close(listen_socket);
        listen_socket = -1;
    }
    
    // Wait for task to end
    if (telnet_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100));
        telnet_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "SSH debug server stopped");
}

bool ssh_debug_server_is_running(void) {
    return server_running;
}
