/*
 * USB Host HID Mouse Driver
 * For ESP32-P4-NANO USB Type-A port (USB OTG 2.0 HS)
 * Works alongside USB keyboard - shares USB HID host infrastructure
 */

#ifndef USB_MOUSE_H
#define USB_MOUSE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Mouse state (relative movement + buttons)
typedef struct {
    int8_t dx;        // Relative X movement (-127 to +127)
    int8_t dy;        // Relative Y movement (-127 to +127)
    uint8_t buttons;  // Button state: bit0=left, bit1=right, bit2=middle
} usb_mouse_report_t;

// Check if USB mouse is connected and has data
bool usb_mouse_available(void);

// Read the latest mouse report (non-blocking, returns false if no data)
bool usb_mouse_read(usb_mouse_report_t *report);

// Check if mouse is connected
bool usb_mouse_is_connected(void);

// Called internally by USB HID host when a mouse report is received
// Do not call this directly - it's called from usb_keyboard.c HID callback
void usb_mouse_process_report(const uint8_t *data, size_t length);

// Called when mouse device connects/disconnects
void usb_mouse_set_connected(bool connected);

#endif // USB_MOUSE_H
