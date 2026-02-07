/*
 * Bluetooth Controller for ESP32-P4-NANO
 * Manages ESP32-C6 Bluetooth coprocessor via UART
 * Supports BLE scanning, HID devices (mice, keyboards, controllers)
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum devices that can be tracked
#define BT_MAX_DEVICES 16

// Device types
typedef enum {
    BT_DEVICE_TYPE_UNKNOWN = 0,
    BT_DEVICE_TYPE_KEYBOARD,
    BT_DEVICE_TYPE_MOUSE,
    BT_DEVICE_TYPE_GAMEPAD,
    BT_DEVICE_TYPE_JOYSTICK,
    BT_DEVICE_TYPE_AUDIO,
    BT_DEVICE_TYPE_OTHER
} bt_device_type_t;

// Connection state
typedef enum {
    BT_STATE_DISCONNECTED = 0,
    BT_STATE_CONNECTING,
    BT_STATE_CONNECTED,
    BT_STATE_PAIRED
} bt_connection_state_t;

// Bluetooth device info
typedef struct {
    uint8_t address[6];           // MAC address
    char name[64];                // Device name
    int8_t rssi;                  // Signal strength
    bt_device_type_t type;        // Device type
    bt_connection_state_t state;  // Connection state
    bool is_valid;                // Entry is valid
    uint32_t last_seen;           // Timestamp
} bt_device_info_t;

// HID mouse event
typedef struct {
    int16_t x;          // X movement
    int16_t y;          // Y movement
    int8_t wheel;       // Wheel movement
    uint8_t buttons;    // Button state (bit mask)
} bt_mouse_event_t;

// HID keyboard event
typedef struct {
    uint8_t modifiers;  // Modifier keys (Ctrl, Shift, Alt, etc.)
    uint8_t keys[6];    // Pressed keys (HID keycodes)
} bt_keyboard_event_t;

// HID gamepad event
typedef struct {
    int16_t left_x;     // Left stick X
    int16_t left_y;     // Left stick Y
    int16_t right_x;    // Right stick X
    int16_t right_y;    // Right stick Y
    uint8_t triggers;   // Trigger buttons
    uint16_t buttons;   // Button state
} bt_gamepad_event_t;

// Callback function types
typedef void (*bt_scan_callback_t)(const bt_device_info_t *device);
typedef void (*bt_connection_callback_t)(const bt_device_info_t *device, bool connected);
typedef void (*bt_mouse_callback_t)(const bt_mouse_event_t *event);
typedef void (*bt_keyboard_callback_t)(const bt_keyboard_event_t *event);
typedef void (*bt_gamepad_callback_t)(const bt_gamepad_event_t *event);

/**
 * @brief Initialize Bluetooth controller
 * 
 * Initializes communication with ESP32-C6 coprocessor
 * For ESP32-P4-NANO: UART communication
 * For ESP32-P4-WIFI6: SDIO communication
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bt_controller_init(void);

/**
 * @brief Deinitialize Bluetooth controller
 */
void bt_controller_deinit(void);

/**
 * @brief Start Bluetooth scan
 * 
 * Scans for nearby Bluetooth devices
 * 
 * @param duration_sec Scan duration in seconds (0 = continuous)
 * @param callback Callback for each discovered device (optional)
 * @return ESP_OK on success
 */
esp_err_t bt_scan_start(uint32_t duration_sec, bt_scan_callback_t callback);

/**
 * @brief Stop Bluetooth scan
 * 
 * @return ESP_OK on success
 */
esp_err_t bt_scan_stop(void);

/**
 * @brief Check if scan is active
 * 
 * @return true if scanning
 */
bool bt_is_scanning(void);

/**
 * @brief Get list of discovered devices
 * 
 * @param devices Array to store device info
 * @param max_devices Maximum number of devices to return
 * @return Number of devices returned
 */
int bt_get_discovered_devices(bt_device_info_t *devices, int max_devices);

/**
 * @brief Connect to a Bluetooth device
 * 
 * @param address Device MAC address
 * @param callback Connection state callback (optional)
 * @return ESP_OK on success
 */
esp_err_t bt_connect(const uint8_t address[6], bt_connection_callback_t callback);

/**
 * @brief Disconnect from a Bluetooth device
 * 
 * @param address Device MAC address
 * @return ESP_OK on success
 */
esp_err_t bt_disconnect(const uint8_t address[6]);

/**
 * @brief Get list of connected devices
 * 
 * @param devices Array to store device info
 * @param max_devices Maximum number of devices to return
 * @return Number of connected devices
 */
int bt_get_connected_devices(bt_device_info_t *devices, int max_devices);

/**
 * @brief Register mouse event callback
 * 
 * @param callback Callback function
 */
void bt_register_mouse_callback(bt_mouse_callback_t callback);

/**
 * @brief Register keyboard event callback
 * 
 * @param callback Callback function
 */
void bt_register_keyboard_callback(bt_keyboard_callback_t callback);

/**
 * @brief Register gamepad event callback
 * 
 * @param callback Callback function
 */
void bt_register_gamepad_callback(bt_gamepad_callback_t callback);

/**
 * @brief Get device info by address
 * 
 * @param address Device MAC address
 * @param info Pointer to store device info
 * @return ESP_OK if found
 */
esp_err_t bt_get_device_info(const uint8_t address[6], bt_device_info_t *info);

/**
 * @brief Clear device list
 */
void bt_clear_device_list(void);

/**
 * @brief Check if Bluetooth controller is initialized
 * 
 * @return true if initialized
 */
bool bt_controller_is_initialized(void);

/**
 * @brief Get Bluetooth controller status string
 * 
 * @return Status string
 */
const char* bt_get_status_string(void);

#ifdef __cplusplus
}
#endif
