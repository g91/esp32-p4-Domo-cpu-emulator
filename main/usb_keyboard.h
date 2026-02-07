/*
 * USB Host HID Keyboard Driver
 * For ESP32-P4-NANO USB Type-A port (USB OTG 2.0 HS)
 */

#ifndef USB_KEYBOARD_H
#define USB_KEYBOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Initialize USB Host and HID keyboard driver
esp_err_t usb_keyboard_init(void);

// Check if keyboard has data available
bool usb_keyboard_available(void);

// Read a character from the keyboard (non-blocking, returns 0 if no data)
uint8_t usb_keyboard_read(void);

// Check if USB keyboard is initialized
bool usb_keyboard_is_initialized(void);

// Deinitialize USB keyboard
void usb_keyboard_deinit(void);

#endif // USB_KEYBOARD_H
