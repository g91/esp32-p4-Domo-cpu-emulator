/*
 * PS/2 Keyboard Driver for ESP32-P4
 * Handles PS/2 keyboard input via GPIO bit-banging
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PS/2 GPIO configuration for ESP32-P4-NANO
// Using available GPIOs with interrupt capability
#define PS2_CLK_PIN     5   // GPIO5 (TOUCH_CHANNEL3) - PS/2 Clock
#define PS2_DATA_PIN    23  // GPIO23 (ADC1_CHANNEL7) - PS/2 Data

// Keyboard buffer size
#define PS2_BUFFER_SIZE 64

// Special key codes returned by ps2_keyboard_read()
#define PS2_KEY_NONE        0x00
#define PS2_KEY_ESCAPE      0x1B
#define PS2_KEY_BACKSPACE   0x08
#define PS2_KEY_TAB         0x09
#define PS2_KEY_ENTER       0x0D
#define PS2_KEY_DELETE      0x7F

// Arrow keys (using values above 0x80)
#define PS2_KEY_UP          0x80
#define PS2_KEY_DOWN        0x81
#define PS2_KEY_LEFT        0x82
#define PS2_KEY_RIGHT       0x83
#define PS2_KEY_HOME        0x84
#define PS2_KEY_END         0x85
#define PS2_KEY_PAGEUP      0x86
#define PS2_KEY_PAGEDOWN    0x87
#define PS2_KEY_INSERT      0x88

// Function keys
#define PS2_KEY_F1          0x90
#define PS2_KEY_F2          0x91
#define PS2_KEY_F3          0x92
#define PS2_KEY_F4          0x93
#define PS2_KEY_F5          0x94
#define PS2_KEY_F6          0x95
#define PS2_KEY_F7          0x96
#define PS2_KEY_F8          0x97
#define PS2_KEY_F9          0x98
#define PS2_KEY_F10         0x99
#define PS2_KEY_F11         0x9A
#define PS2_KEY_F12         0x9B

// Callback type for key events
typedef void (*ps2_key_callback_t)(uint8_t key, bool pressed);

/**
 * @brief Initialize the PS/2 keyboard driver
 * 
 * Sets up GPIO interrupts for CLK and DATA pins
 * 
 * @return ESP_OK on success
 */
esp_err_t ps2_keyboard_init(void);

/**
 * @brief Deinitialize the PS/2 keyboard driver
 */
void ps2_keyboard_deinit(void);

/**
 * @brief Read a character from the keyboard buffer
 * 
 * Non-blocking read. Returns 0 if no character available.
 * 
 * @return ASCII character, special key code, or 0 if none available
 */
uint8_t ps2_keyboard_read(void);

/**
 * @brief Read a character from the keyboard (blocking)
 * 
 * Blocks until a key is pressed or timeout occurs.
 * 
 * @param timeout_ms Timeout in milliseconds (0 = wait forever)
 * @return ASCII character, special key code, or 0 on timeout
 */
uint8_t ps2_keyboard_read_blocking(uint32_t timeout_ms);

/**
 * @brief Check if keyboard data is available
 * 
 * @return true if there is data in the buffer
 */
bool ps2_keyboard_available(void);

/**
 * @brief Clear the keyboard buffer
 */
void ps2_keyboard_clear(void);

/**
 * @brief Set keyboard LED state
 * 
 * @param scroll_lock Scroll Lock LED
 * @param num_lock Num Lock LED  
 * @param caps_lock Caps Lock LED
 * @return ESP_OK on success
 */
esp_err_t ps2_keyboard_set_leds(bool scroll_lock, bool num_lock, bool caps_lock);

/**
 * @brief Register a callback for key events
 * 
 * @param callback Function to call on key press/release
 */
void ps2_keyboard_set_callback(ps2_key_callback_t callback);

/**
 * @brief Check if keyboard is initialized
 * 
 * @return true if initialized
 */
bool ps2_keyboard_is_initialized(void);

/**
 * @brief Get keyboard state (modifier keys)
 * 
 * @param shift Pointer to store Shift state
 * @param ctrl Pointer to store Ctrl state
 * @param alt Pointer to store Alt state
 */
void ps2_keyboard_get_modifiers(bool *shift, bool *ctrl, bool *alt);

#ifdef __cplusplus
}
#endif
