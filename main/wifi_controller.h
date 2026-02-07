/*
 * WiFi Controller for ESP32-P4-NANO
 * Manages ESP32-C6 WiFi coprocessor via UART
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_MAX_NETWORKS 32
#define WIFI_MAX_SSID_LEN 32
#define WIFI_MAX_PASS_LEN 64

// WiFi network information
typedef struct {
    char ssid[WIFI_MAX_SSID_LEN + 1];
    int8_t rssi;
    uint8_t auth_mode;
    uint8_t channel;
} wifi_network_info_t;

// WiFi connection status
typedef enum {
    WIFI_STATUS_DISCONNECTED = 0,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_CONNECTION_FAILED
} wifi_status_t;

// Callback function types
typedef void (*wifi_scan_callback_t)(const wifi_network_info_t *networks, int count);
typedef void (*wifi_connection_callback_t)(wifi_status_t status, const char *ip_address);

/**
 * @brief Initialize WiFi controller (ESP32-C6 coprocessor via UART)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_init(void);

/**
 * @brief Deinitialize WiFi controller
 */
void wifi_controller_deinit(void);

/**
 * @brief Check if WiFi controller is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool wifi_controller_is_initialized(void);

/**
 * @brief Start WiFi network scan
 * 
 * @param callback Callback function for scan results (optional)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_scan(wifi_scan_callback_t callback);

/**
 * @brief Connect to WiFi network
 * 
 * @param ssid Network SSID
 * @param password Network password
 * @param callback Connection status callback (optional)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_connect(const char *ssid, const char *password, wifi_connection_callback_t callback);

/**
 * @brief Disconnect from WiFi network
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_controller_disconnect(void);

/**
 * @brief Get current WiFi status
 * 
 * @return Current WiFi status
 */
wifi_status_t wifi_controller_get_status(void);

/**
 * @brief Get current IP address
 * 
 * @param ip_str Buffer to store IP address (min 16 bytes)
 * @param len Buffer length
 * @return ESP_OK if connected and IP available, error code otherwise
 */
esp_err_t wifi_controller_get_ip(char *ip_str, size_t len);

/**
 * @brief Get RSSI of current connection
 * 
 * @return RSSI value in dBm, or 0 if not connected
 */
int8_t wifi_controller_get_rssi(void);

/**
 * @brief Get last scan results
 * 
 * @param networks Buffer to store network information
 * @param max_networks Maximum number of networks to return
 * @return Number of networks found, or negative error code
 */
int wifi_controller_get_scan_results(wifi_network_info_t *networks, int max_networks);

#ifdef __cplusplus
}
#endif