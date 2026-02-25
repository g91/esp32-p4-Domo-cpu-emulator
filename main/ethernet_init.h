/*
 * Ethernet Driver for ESP32-P4-NANO
 * IP101GRI PHY via internal EMAC (RMII interface)
 * RJ45 100M Ethernet port
 */

#pragma once

#include "esp_err.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Ethernet hardware and connect to network
 * 
 * Sets up the ESP32-P4 internal EMAC with IP101GRI PHY.
 * Ethernet will auto-negotiate and get IP via DHCP when cable is plugged in.
 * 
 * @note Call after esp_netif_init() and esp_event_loop_create_default()
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ethernet_init(void);

/**
 * @brief Deinitialize Ethernet and release resources
 */
void ethernet_deinit(void);

/**
 * @brief Check if Ethernet link is up (cable connected)
 * 
 * @return true if link is up, false otherwise
 */
bool ethernet_is_link_up(void);

/**
 * @brief Check if Ethernet has an IP address
 * 
 * @return true if connected with IP, false otherwise
 */
bool ethernet_is_connected(void);

/**
 * @brief Get Ethernet IP address as string
 * 
 * @param ip_str Buffer to store IP address (min 16 bytes)
 * @param len Buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ethernet_get_ip(char *ip_str, size_t len);

/**
 * @brief Get the Ethernet netif handle
 * 
 * @return esp_netif_t pointer or NULL
 */
esp_netif_t *ethernet_get_netif(void);

#ifdef __cplusplus
}
#endif
