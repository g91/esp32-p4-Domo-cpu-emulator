/*
 * Bluetooth Controller for ESP32-P4-NANO
 * Manages ESP32-C6 Bluetooth coprocessor via UART
 */

#include "bt_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "bt_controller";

// UART configuration for ESP32-C6 communication (ESP32-P4-NANO variant)
#define BT_UART_PORT        UART_NUM_1
#define BT_UART_TX_PIN      38  // Now free for UART after LCD RST moved to GPIO6
#define BT_UART_RX_PIN      39  // Now free for UART
#define BT_UART_BAUD        115200
#define BT_UART_BUF_SIZE    1024

// Internal state
typedef struct {
    bool initialized;
    bool scanning;
    TaskHandle_t rx_task;
    SemaphoreHandle_t mutex;
    QueueHandle_t event_queue;
    
    // Device lists
    bt_device_info_t discovered_devices[BT_MAX_DEVICES];
    int discovered_count;
    
    // Callbacks
    bt_scan_callback_t scan_callback;
    bt_connection_callback_t connection_callback;
    bt_mouse_callback_t mouse_callback;
    bt_keyboard_callback_t keyboard_callback;
    bt_gamepad_callback_t gamepad_callback;
    
    // Scan state
    uint32_t scan_start_time;
    uint32_t scan_duration;
} bt_controller_state_t;

static bt_controller_state_t *bt_state = NULL;

// Command protocol for ESP32-C6 communication
#define BT_CMD_INIT         0x01
#define BT_CMD_SCAN_START   0x02
#define BT_CMD_SCAN_STOP    0x03
#define BT_CMD_CONNECT      0x04
#define BT_CMD_DISCONNECT   0x05
#define BT_CMD_GET_STATUS   0x06

#define BT_EVT_DEVICE_FOUND 0x10
#define BT_EVT_CONNECTED    0x11
#define BT_EVT_DISCONNECTED 0x12
#define BT_EVT_MOUSE        0x20
#define BT_EVT_KEYBOARD     0x21
#define BT_EVT_GAMEPAD      0x22

// Protocol packet structure
typedef struct __attribute__((packed)) {
    uint8_t magic;      // 0xBT (0xB7)
    uint8_t cmd;        // Command/event ID
    uint16_t length;    // Payload length
    uint8_t payload[];  // Variable payload
} bt_packet_t;

#define BT_MAGIC 0xB7

// Forward declarations
static void bt_rx_task(void *arg);
static void process_packet(const bt_packet_t *packet);
static esp_err_t send_command(uint8_t cmd, const void *payload, uint16_t length);

esp_err_t bt_controller_init(void)
{
    if (bt_state != NULL && bt_state->initialized) {
        ESP_LOGW(TAG, "Bluetooth controller already initialized");
        return ESP_OK;
    }
    
    // Allocate state
    bt_state = calloc(1, sizeof(bt_controller_state_t));
    if (!bt_state) {
        ESP_LOGE(TAG, "Failed to allocate Bluetooth controller state");
        return ESP_ERR_NO_MEM;
    }
    
    // Create mutex
    bt_state->mutex = xSemaphoreCreateMutex();
    if (!bt_state->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(bt_state);
        bt_state = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Create event queue
    bt_state->event_queue = xQueueCreate(32, sizeof(bt_packet_t) + 256);
    if (!bt_state->event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        vSemaphoreDelete(bt_state->mutex);
        free(bt_state);
        bt_state = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Configure UART for ESP32-C6 communication
    const uart_config_t uart_config = {
        .baud_rate = BT_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_driver_install(BT_UART_PORT, BT_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = uart_param_config(BT_UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = uart_set_pin(BT_UART_PORT, BT_UART_TX_PIN, BT_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Start RX task
    BaseType_t task_ret = xTaskCreate(bt_rx_task, "bt_rx", 4096, NULL, 10, &bt_state->rx_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    // Send initialization command to ESP32-C6
    ret = send_command(BT_CMD_INIT, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP32-C6: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    bt_state->initialized = true;
    ESP_LOGI(TAG, "Bluetooth controller initialized (UART: TX=GPIO%d, RX=GPIO%d)", 
             BT_UART_TX_PIN, BT_UART_RX_PIN);
    return ESP_OK;
    
cleanup:
    if (bt_state->rx_task) {
        vTaskDelete(bt_state->rx_task);
    }
    uart_driver_delete(BT_UART_PORT);
    if (bt_state->event_queue) {
        vQueueDelete(bt_state->event_queue);
    }
    if (bt_state->mutex) {
        vSemaphoreDelete(bt_state->mutex);
    }
    free(bt_state);
    bt_state = NULL;
    return ret;
}

void bt_controller_deinit(void)
{
    if (!bt_state) return;
    
    bt_state->initialized = false;
    
    if (bt_state->rx_task) {
        vTaskDelete(bt_state->rx_task);
    }
    
    uart_driver_delete(BT_UART_PORT);
    
    if (bt_state->event_queue) {
        vQueueDelete(bt_state->event_queue);
    }
    
    if (bt_state->mutex) {
        vSemaphoreDelete(bt_state->mutex);
    }
    
    free(bt_state);
    bt_state = NULL;
    
    ESP_LOGI(TAG, "Bluetooth controller deinitialized");
}

esp_err_t bt_scan_start(uint32_t duration_sec, bt_scan_callback_t callback)
{
    if (!bt_state || !bt_state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(bt_state->mutex, portMAX_DELAY);
    
    if (bt_state->scanning) {
        xSemaphoreGive(bt_state->mutex);
        ESP_LOGW(TAG, "Scan already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear discovered devices
    memset(bt_state->discovered_devices, 0, sizeof(bt_state->discovered_devices));
    bt_state->discovered_count = 0;
    
    bt_state->scanning = true;
    bt_state->scan_callback = callback;
    bt_state->scan_start_time = (uint32_t)(esp_timer_get_time() / 1000000);
    bt_state->scan_duration = duration_sec;
    
    xSemaphoreGive(bt_state->mutex);
    
    // Send scan start command
    struct {
        uint32_t duration;
    } scan_params = { .duration = duration_sec };
    
    esp_err_t ret = send_command(BT_CMD_SCAN_START, &scan_params, sizeof(scan_params));
    if (ret != ESP_OK) {
        bt_state->scanning = false;
        ESP_LOGE(TAG, "Failed to start scan: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Bluetooth scan started (duration: %lu sec)", (unsigned long)duration_sec);
    }
    
    return ret;
}

esp_err_t bt_scan_stop(void)
{
    if (!bt_state || !bt_state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!bt_state->scanning) {
        return ESP_OK;
    }
    
    bt_state->scanning = false;
    
    esp_err_t ret = send_command(BT_CMD_SCAN_STOP, NULL, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth scan stopped");
    }
    
    return ret;
}

bool bt_is_scanning(void)
{
    return bt_state && bt_state->scanning;
}

int bt_get_discovered_devices(bt_device_info_t *devices, int max_devices)
{
    if (!bt_state || !devices || max_devices <= 0) {
        return 0;
    }
    
    xSemaphoreTake(bt_state->mutex, portMAX_DELAY);
    
    int count = bt_state->discovered_count < max_devices ? bt_state->discovered_count : max_devices;
    memcpy(devices, bt_state->discovered_devices, count * sizeof(bt_device_info_t));
    
    xSemaphoreGive(bt_state->mutex);
    
    return count;
}

esp_err_t bt_connect(const uint8_t address[6], bt_connection_callback_t callback)
{
    if (!bt_state || !bt_state->initialized || !address) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bt_state->connection_callback = callback;
    
    esp_err_t ret = send_command(BT_CMD_CONNECT, address, 6);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Connecting to %02X:%02X:%02X:%02X:%02X:%02X",
                 address[0], address[1], address[2], address[3], address[4], address[5]);
    }
    
    return ret;
}

esp_err_t bt_disconnect(const uint8_t address[6])
{
    if (!bt_state || !bt_state->initialized || !address) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = send_command(BT_CMD_DISCONNECT, address, 6);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Disconnecting from %02X:%02X:%02X:%02X:%02X:%02X",
                 address[0], address[1], address[2], address[3], address[4], address[5]);
    }
    
    return ret;
}

int bt_get_connected_devices(bt_device_info_t *devices, int max_devices)
{
    if (!bt_state || !devices || max_devices <= 0) {
        return 0;
    }
    
    xSemaphoreTake(bt_state->mutex, portMAX_DELAY);
    
    int count = 0;
    for (int i = 0; i < BT_MAX_DEVICES && count < max_devices; i++) {
        if (bt_state->discovered_devices[i].is_valid && 
            bt_state->discovered_devices[i].state == BT_STATE_CONNECTED) {
            devices[count++] = bt_state->discovered_devices[i];
        }
    }
    
    xSemaphoreGive(bt_state->mutex);
    
    return count;
}

void bt_register_mouse_callback(bt_mouse_callback_t callback)
{
    if (bt_state) {
        bt_state->mouse_callback = callback;
    }
}

void bt_register_keyboard_callback(bt_keyboard_callback_t callback)
{
    if (bt_state) {
        bt_state->keyboard_callback = callback;
    }
}

void bt_register_gamepad_callback(bt_gamepad_callback_t callback)
{
    if (bt_state) {
        bt_state->gamepad_callback = callback;
    }
}

esp_err_t bt_get_device_info(const uint8_t address[6], bt_device_info_t *info)
{
    if (!bt_state || !address || !info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(bt_state->mutex, portMAX_DELAY);
    
    for (int i = 0; i < BT_MAX_DEVICES; i++) {
        if (bt_state->discovered_devices[i].is_valid &&
            memcmp(bt_state->discovered_devices[i].address, address, 6) == 0) {
            *info = bt_state->discovered_devices[i];
            xSemaphoreGive(bt_state->mutex);
            return ESP_OK;
        }
    }
    
    xSemaphoreGive(bt_state->mutex);
    return ESP_ERR_NOT_FOUND;
}

void bt_clear_device_list(void)
{
    if (!bt_state) return;
    
    xSemaphoreTake(bt_state->mutex, portMAX_DELAY);
    memset(bt_state->discovered_devices, 0, sizeof(bt_state->discovered_devices));
    bt_state->discovered_count = 0;
    xSemaphoreGive(bt_state->mutex);
}

bool bt_controller_is_initialized(void)
{
    return bt_state && bt_state->initialized;
}

const char* bt_get_status_string(void)
{
    if (!bt_state) {
        return "Not initialized";
    }
    
    if (bt_state->scanning) {
        return "Scanning...";
    }
    
    int connected_count = 0;
    for (int i = 0; i < BT_MAX_DEVICES; i++) {
        if (bt_state->discovered_devices[i].is_valid && 
            bt_state->discovered_devices[i].state == BT_STATE_CONNECTED) {
            connected_count++;
        }
    }
    
    static char status[64];
    snprintf(status, sizeof(status), "Ready (%d device%s connected)", 
             connected_count, connected_count == 1 ? "" : "s");
    return status;
}

// Internal functions

static esp_err_t send_command(uint8_t cmd, const void *payload, uint16_t length)
{
    if (!bt_state || !bt_state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Build packet
    uint8_t buffer[sizeof(bt_packet_t) + length];
    bt_packet_t *packet = (bt_packet_t *)buffer;
    packet->magic = BT_MAGIC;
    packet->cmd = cmd;
    packet->length = length;
    
    if (payload && length > 0) {
        memcpy(packet->payload, payload, length);
    }
    
    // Send via UART
    int written = uart_write_bytes(BT_UART_PORT, buffer, sizeof(bt_packet_t) + length);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

static void bt_rx_task(void *arg)
{
    uint8_t buffer[1024];
    
    ESP_LOGI(TAG, "Bluetooth RX task started");
    
    while (bt_state && bt_state->initialized) {
        int len = uart_read_bytes(BT_UART_PORT, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Process received data
            // Simple packet parser (in real implementation, need proper framing)
            if (len >= sizeof(bt_packet_t)) {
                bt_packet_t *packet = (bt_packet_t *)buffer;
                if (packet->magic == BT_MAGIC) {
                    process_packet(packet);
                }
            }
        }
    }
    
    ESP_LOGI(TAG, "Bluetooth RX task stopped");
    vTaskDelete(NULL);
}

static void process_packet(const bt_packet_t *packet)
{
    if (!packet) return;
    
    switch (packet->cmd) {
        case BT_EVT_DEVICE_FOUND: {
            if (packet->length >= sizeof(bt_device_info_t)) {
                bt_device_info_t *device = (bt_device_info_t *)packet->payload;
                
                xSemaphoreTake(bt_state->mutex, portMAX_DELAY);
                
                // Add or update device
                int slot = -1;
                for (int i = 0; i < BT_MAX_DEVICES; i++) {
                    if (memcmp(bt_state->discovered_devices[i].address, device->address, 6) == 0) {
                        slot = i;
                        break;
                    }
                    if (slot == -1 && !bt_state->discovered_devices[i].is_valid) {
                        slot = i;
                    }
                }
                
                if (slot >= 0) {
                    bt_state->discovered_devices[slot] = *device;
                    bt_state->discovered_devices[slot].is_valid = true;
                    bt_state->discovered_devices[slot].last_seen = (uint32_t)(esp_timer_get_time() / 1000000);
                    
                    if (slot >= bt_state->discovered_count) {
                        bt_state->discovered_count = slot + 1;
                    }
                    
                    // Call scan callback
                    if (bt_state->scan_callback) {
                        bt_state->scan_callback(&bt_state->discovered_devices[slot]);
                    }
                }
                
                xSemaphoreGive(bt_state->mutex);
            }
            break;
        }
        
        case BT_EVT_CONNECTED:
        case BT_EVT_DISCONNECTED: {
            if (packet->length >= 6) {
                const uint8_t *address = packet->payload;
                bool connected = (packet->cmd == BT_EVT_CONNECTED);
                
                xSemaphoreTake(bt_state->mutex, portMAX_DELAY);
                
                for (int i = 0; i < BT_MAX_DEVICES; i++) {
                    if (memcmp(bt_state->discovered_devices[i].address, address, 6) == 0) {
                        bt_state->discovered_devices[i].state = connected ? BT_STATE_CONNECTED : BT_STATE_DISCONNECTED;
                        
                        if (bt_state->connection_callback) {
                            bt_state->connection_callback(&bt_state->discovered_devices[i], connected);
                        }
                        break;
                    }
                }
                
                xSemaphoreGive(bt_state->mutex);
            }
            break;
        }
        
        case BT_EVT_MOUSE: {
            if (packet->length >= sizeof(bt_mouse_event_t) && bt_state->mouse_callback) {
                bt_mouse_event_t *event = (bt_mouse_event_t *)packet->payload;
                bt_state->mouse_callback(event);
            }
            break;
        }
        
        case BT_EVT_KEYBOARD: {
            if (packet->length >= sizeof(bt_keyboard_event_t) && bt_state->keyboard_callback) {
                bt_keyboard_event_t *event = (bt_keyboard_event_t *)packet->payload;
                bt_state->keyboard_callback(event);
            }
            break;
        }
        
        case BT_EVT_GAMEPAD: {
            if (packet->length >= sizeof(bt_gamepad_event_t) && bt_state->gamepad_callback) {
                bt_gamepad_event_t *event = (bt_gamepad_event_t *)packet->payload;
                bt_state->gamepad_callback(event);
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown event: 0x%02X", packet->cmd);
            break;
    }
}
