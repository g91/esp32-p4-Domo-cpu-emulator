/*
 * SSH Debug Server for ESP32-P4
 * Provides remote console access for debugging M68K emulator
 */

#ifndef SSH_DEBUG_SERVER_H
#define SSH_DEBUG_SERVER_H

#include "esp_err.h"

// Initialize SSH debug server (starts automatically with WiFi)
esp_err_t ssh_debug_server_init(void);

// Stop SSH debug server
void ssh_debug_server_stop(void);

// Check if SSH server is running
bool ssh_debug_server_is_running(void);

#endif // SSH_DEBUG_SERVER_H
