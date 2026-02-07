/*
 * WiFi Controller for ESP32-P4-NANO
 * Manages ESP32-C6 WiFi coprocessor via UART
 */

#include "wifi_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "wifi_controller";

// UART configuration for ESP32-C6 communication (ESP32-P4-NANO variant)
// Note: This uses the same UART as Bluetooth, so only one can be active at a time
#define WIFI_UART_PORT        UART_NUM_1
#define WIFI_UART_TX_PIN      38  // Same as Bluetooth - shared coprocessor
#define WIFI_UART_RX_PIN      39  // Same as Bluetooth
#define WIFI_UART_BAUD        115200
#define WIFI_UART_BUF_SIZE    1024

// Internal state
typedef struct {
    bool initialized;
    bool scanning;
    TaskHandle_t rx_task;
    SemaphoreHandle_t mutex;
    QueueHandle_t event_queue;
    
    // Network lists
    wifi_network_info_t scan_results[WIFI_MAX_NETWORKS];
    int scan_count;
    
    // Current connection
    wifi_status_t status;
    char current_ssid[WIFI_MAX_SSID_LEN + 1];
    char current_ip[16];
    int8_t current_rssi;
    
    // Callbacks
    wifi_scan_callback_t scan_callback;
    wifi_connection_callback_t connection_callback;
} wifi_controller_state_t;

static wifi_controller_state_t *wifi_state = NULL;

// Command protocol for ESP32-C6 communication
#define WIFI_CMD_INIT         0x10
#define WIFI_CMD_SCAN_START   0x11
#define WIFI_CMD_SCAN_STOP    0x12
#define WIFI_CMD_CONNECT      0x13
#define WIFI_CMD_DISCONNECT   0x14
#define WIFI_CMD_GET_STATUS   0x15

#define WIFI_EVT_SCAN_RESULT  0x20
#define WIFI_EVT_SCAN_DONE    0x21
#define WIFI_EVT_CONNECTED    0x22
#define WIFI_EVT_DISCONNECTED 0x23
#define WIFI_EVT_IP_ASSIGNED  0x24

// Protocol packet structure (same as Bluetooth)
typedef struct __attribute__((packed)) {
    uint8_t magic;      // 0xWF (0xF1) for WiFi
    uint8_t cmd;        // Command/event ID
    uint16_t length;    // Payload length
    uint8_t payload[];  // Variable payload
} wifi_packet_t;

#define WIFI_MAGIC 0xF1

// Payload structures
typedef struct __attribute__((packed)) {
    char ssid[WIFI_MAX_SSID_LEN + 1];
    char password[WIFI_MAX_PASS_LEN + 1];
} wifi_connect_payload_t;

typedef struct __attribute__((packed)) {
    char ssid[WIFI_MAX_SSID_LEN + 1];
    int8_t rssi;
    uint8_t auth_mode;
    uint8_t channel;
} wifi_scan_result_payload_t;

typedef struct __attribute__((packed)) {
    char ssid[WIFI_MAX_SSID_LEN + 1];
    char ip_address[16];
    int8_t rssi;
} wifi_connected_payload_t;

// Forward declarations
static void wifi_rx_task(void *arg);
static void process_packet(const wifi_packet_t *packet);
static esp_err_t send_command(uint8_t cmd, const void *payload, uint16_t length);

esp_err_t wifi_controller_init(void)
{
    if (wifi_state != NULL && wifi_state->initialized) {
        ESP_LOGW(TAG, "WiFi controller already initialized");
        return ESP_OK;
    }
    
    // Allocate state
    wifi_state = calloc(1, sizeof(wifi_controller_state_t));
    if (!wifi_state) {
        ESP_LOGE(TAG, "Failed to allocate WiFi controller state");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize status
    wifi_state->status = WIFI_STATUS_DISCONNECTED;
    
    // Create mutex
    wifi_state->mutex = xSemaphoreCreateMutex();
    if (!wifi_state->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(wifi_state);
        wifi_state = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Create event queue
    wifi_state->event_queue = xQueueCreate(32, sizeof(wifi_packet_t) + 256);
    if (!wifi_state->event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        vSemaphoreDelete(wifi_state->mutex);
        free(wifi_state);
        wifi_state = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Configure UART for ESP32-C6 communication
    const uart_config_t uart_config = {
        .baud_rate = WIFI_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_driver_install(WIFI_UART_PORT, WIFI_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = uart_param_config(WIFI_UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = uart_set_pin(WIFI_UART_PORT, WIFI_UART_TX_PIN, WIFI_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Set initialized flag before creating task to avoid race conditions
    wifi_state->initialized = true;
    
    // Start RX task
    BaseType_t task_ret = xTaskCreate(wifi_rx_task, "wifi_rx", 4096, NULL, 10, &wifi_state->rx_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    // Give RX task time to start
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Send initialization command to ESP32-C6
    ret = send_command(WIFI_CMD_INIT, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ESP32-C6 coprocessor not responding (expected on ESP32-P4-NANO)");
        // Don't fail initialization - WiFi commands will show appropriate errors
        ret = ESP_OK;
    }
    ESP_LOGI(TAG, "WiFi controller initialized (UART: TX=GPIO%d, RX=GPIO%d)", 
             WIFI_UART_TX_PIN, WIFI_UART_RX_PIN);
    return ESP_OK;
    
cleanup:
    // Set initialized to false to stop RX task loop
    if (wifi_state) {
        wifi_state->initialized = false;
    }
    
    // Wait for RX task to exit gracefully
    if (wifi_state && wifi_state->rx_task) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to exit
        vTaskDelete(wifi_state->rx_task);
        wifi_state->rx_task = NULL;
    }
    
    uart_driver_delete(WIFI_UART_PORT);
    
    if (wifi_state && wifi_state->event_queue) {
        vQueueDelete(wifi_state->event_queue);
    }
    if (wifi_state && wifi_state->mutex) {
        vSemaphoreDelete(wifi_state->mutex);
    }
    free(wifi_state);
    wifi_state = NULL;
    return ret;
}

void wifi_controller_deinit(void)
{
    if (!wifi_state) return;
    
    wifi_state->initialized = false;
    
    if (wifi_state->rx_task) {
        vTaskDelete(wifi_state->rx_task);
    }
    
    uart_driver_delete(WIFI_UART_PORT);
    
    if (wifi_state->event_queue) {
        vQueueDelete(wifi_state->event_queue);
    }
    
    if (wifi_state->mutex) {
        vSemaphoreDelete(wifi_state->mutex);
    }
    
    free(wifi_state);
    wifi_state = NULL;
    
    ESP_LOGI(TAG, "WiFi controller deinitialized");
}

bool wifi_controller_is_initialized(void)
{
    return wifi_state != NULL && wifi_state->initialized;
}

esp_err_t wifi_controller_scan(wifi_scan_callback_t callback)
{
    if (!wifi_controller_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(wifi_state->mutex, portMAX_DELAY);
    
    wifi_state->scan_callback = callback;
    wifi_state->scanning = true;
    wifi_state->scan_count = 0;
    
    esp_err_t ret = send_command(WIFI_CMD_SCAN_START, NULL, 0);
    
    xSemaphoreGive(wifi_state->mutex);
    return ret;
}

esp_err_t wifi_controller_connect(const char *ssid, const char *password, wifi_connection_callback_t callback)
{
    if (!wifi_controller_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (strlen(ssid) > WIFI_MAX_SSID_LEN || strlen(password) > WIFI_MAX_PASS_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(wifi_state->mutex, portMAX_DELAY);
    
    wifi_connect_payload_t payload = {0};
    strncpy(payload.ssid, ssid, WIFI_MAX_SSID_LEN);
    strncpy(payload.password, password, WIFI_MAX_PASS_LEN);
    
    wifi_state->connection_callback = callback;
    wifi_state->status = WIFI_STATUS_CONNECTING;
    strncpy(wifi_state->current_ssid, ssid, WIFI_MAX_SSID_LEN);
    
    esp_err_t ret = send_command(WIFI_CMD_CONNECT, &payload, sizeof(payload));
    
    xSemaphoreGive(wifi_state->mutex);
    return ret;
}

esp_err_t wifi_controller_disconnect(void)
{
    if (!wifi_controller_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return send_command(WIFI_CMD_DISCONNECT, NULL, 0);
}

wifi_status_t wifi_controller_get_status(void)
{
    if (!wifi_controller_is_initialized()) {
        return WIFI_STATUS_DISCONNECTED;
    }
    
    return wifi_state->status;
}

esp_err_t wifi_controller_get_ip(char *ip_str, size_t len)
{
    if (!wifi_controller_is_initialized() || !ip_str) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (wifi_state->status != WIFI_STATUS_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(wifi_state->mutex, portMAX_DELAY);
    strncpy(ip_str, wifi_state->current_ip, len - 1);
    ip_str[len - 1] = '\0';
    xSemaphoreGive(wifi_state->mutex);
    
    return ESP_OK;
}

int8_t wifi_controller_get_rssi(void)
{
    if (!wifi_controller_is_initialized()) {
        return 0;
    }
    
    return wifi_state->current_rssi;
}

int wifi_controller_get_scan_results(wifi_network_info_t *networks, int max_networks)
{
    if (!wifi_controller_is_initialized() || !networks) {
        return -1;
    }
    
    xSemaphoreTake(wifi_state->mutex, portMAX_DELAY);
    int count = wifi_state->scan_count;
    if (count > max_networks) {
        count = max_networks;
    }
    memcpy(networks, wifi_state->scan_results, count * sizeof(wifi_network_info_t));
    xSemaphoreGive(wifi_state->mutex);
    
    return count;
}

// Internal functions

static esp_err_t send_command(uint8_t cmd, const void *payload, uint16_t length)
{
    // Allow commands during initialization, but check basic state
    if (!wifi_state) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Build packet
    size_t packet_size = sizeof(wifi_packet_t) + length;
    wifi_packet_t *packet = malloc(packet_size);
    if (!packet) {
        return ESP_ERR_NO_MEM;
    }
    
    packet->magic = WIFI_MAGIC;
    packet->cmd = cmd;
    packet->length = length;
    
    if (length > 0 && payload) {
        memcpy(packet->payload, payload, length);
    }
    
    // Send packet
    int written = uart_write_bytes(WIFI_UART_PORT, packet, packet_size);
    esp_err_t ret = (written == packet_size) ? ESP_OK : ESP_FAIL;
    
    free(packet);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command 0x%02X", cmd);
    }
    
    return ret;
}

static void wifi_rx_task(void *arg)
{
    uint8_t *data = malloc(WIFI_UART_BUF_SIZE);
    if (!data) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "WiFi RX task started");
    
    while (wifi_state && wifi_state->initialized) {
        // Read data from UART
        int length = uart_read_bytes(WIFI_UART_PORT, data, WIFI_UART_BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (length > 0) {
            // Process received data
            int offset = 0;
            while (offset < length) {
                // Look for magic byte
                if (data[offset] != WIFI_MAGIC) {
                    offset++;
                    continue;
                }
                
                // Check if we have enough data for header
                if (offset + sizeof(wifi_packet_t) > length) {
                    break;
                }
                
                wifi_packet_t *packet = (wifi_packet_t *)&data[offset];
                size_t packet_size = sizeof(wifi_packet_t) + packet->length;
                
                // Check if we have the complete packet
                if (offset + packet_size > length) {
                    break;
                }
                
                // Process packet
                process_packet(packet);
                
                offset += packet_size;
            }
        }
    }
    
    free(data);
    ESP_LOGI(TAG, "WiFi RX task ended");
    vTaskDelete(NULL);
}

static void process_packet(const wifi_packet_t *packet)
{
    if (!wifi_state || !wifi_state->mutex) {
        return;
    }
    
    xSemaphoreTake(wifi_state->mutex, portMAX_DELAY);
    
    switch (packet->cmd) {
        case WIFI_EVT_SCAN_RESULT: {
            if (packet->length >= sizeof(wifi_scan_result_payload_t) && wifi_state->scan_count < WIFI_MAX_NETWORKS) {
                wifi_scan_result_payload_t *result = (wifi_scan_result_payload_t *)packet->payload;
                wifi_network_info_t *network = &wifi_state->scan_results[wifi_state->scan_count];
                
                strncpy(network->ssid, result->ssid, WIFI_MAX_SSID_LEN);
                network->ssid[WIFI_MAX_SSID_LEN] = '\0';
                network->rssi = result->rssi;
                network->auth_mode = result->auth_mode;
                network->channel = result->channel;
                
                wifi_state->scan_count++;
                ESP_LOGD(TAG, "Found network: %s (RSSI: %d dBm)", network->ssid, network->rssi);
            }
            break;
        }
        
        case WIFI_EVT_SCAN_DONE: {
            wifi_state->scanning = false;
            ESP_LOGI(TAG, "WiFi scan completed (%d networks found)", wifi_state->scan_count);
            
            if (wifi_state->scan_callback) {
                wifi_state->scan_callback(wifi_state->scan_results, wifi_state->scan_count);
            }
            break;
        }
        
        case WIFI_EVT_CONNECTED: {
            if (packet->length >= sizeof(wifi_connected_payload_t)) {
                wifi_connected_payload_t *conn = (wifi_connected_payload_t *)packet->payload;
                
                wifi_state->status = WIFI_STATUS_CONNECTED;
                strncpy(wifi_state->current_ip, conn->ip_address, sizeof(wifi_state->current_ip) - 1);
                wifi_state->current_ip[sizeof(wifi_state->current_ip) - 1] = '\0';
                wifi_state->current_rssi = conn->rssi;
                
                ESP_LOGI(TAG, "WiFi connected to %s (IP: %s, RSSI: %d dBm)", 
                         conn->ssid, conn->ip_address, conn->rssi);
                
                if (wifi_state->connection_callback) {
                    wifi_state->connection_callback(WIFI_STATUS_CONNECTED, conn->ip_address);
                }
            }
            break;
        }
        
        case WIFI_EVT_DISCONNECTED: {
            wifi_state->status = WIFI_STATUS_DISCONNECTED;
            memset(wifi_state->current_ip, 0, sizeof(wifi_state->current_ip));
            wifi_state->current_rssi = 0;
            
            ESP_LOGI(TAG, "WiFi disconnected");
            
            if (wifi_state->connection_callback) {
                wifi_state->connection_callback(WIFI_STATUS_DISCONNECTED, NULL);
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown WiFi event: 0x%02X", packet->cmd);
            break;
    }
    
    xSemaphoreGive(wifi_state->mutex);
}