/*
 * WiFi and FTP Server for ESP32-P4
 * Provides WiFi connectivity and FTP access to SD card
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize WiFi and connect to default network
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_init_sta(void);

/**
 * @brief Connect to a specific WiFi network
 * 
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ftp_wifi_connect(const char *ssid, const char *password);

/**
 * @brief Scan for available WiFi networks
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_scan(void);

/**
 * @brief Check if WiFi is connected
 * 
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * @brief Get IP address as string
 * 
 * @param ip_str Buffer to store IP address (min 16 bytes)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_get_ip(char *ip_str, size_t len);

/**
 * @brief Initialize FTP server
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ftp_server_init(void);

/**
 * @brief Stop FTP server
 */
void ftp_server_stop(void);

/**
 * @brief Check if FTP server is running
 * 
 * @return true if running, false otherwise
 */
bool ftp_server_is_running(void);

#ifdef __cplusplus
}
#endif
