/*
 * USB Host HID Mouse Driver Implementation
 * Receives mouse reports from the shared USB HID host (via usb_keyboard.c)
 * Provides a simple queue-based interface for mouse movement
 * 
 * Also provides weak stubs for USB keyboard functions so code can
 * reference them without requiring the full USB HID host library.
 * When usb_keyboard.c is compiled (with USB HID host component), 
 * its strong symbols override these weak stubs.
 */

#include "usb_mouse.h"
#include "usb_keyboard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "USB_MOUSE";

#define MOUSE_QUEUE_SIZE 32

static QueueHandle_t mouse_queue = NULL;
static bool mouse_connected = false;
static bool mouse_initialized = false;

// Auto-initialize the mouse queue on first use
static void ensure_initialized(void) {
    if (!mouse_initialized) {
        mouse_queue = xQueueCreate(MOUSE_QUEUE_SIZE, sizeof(usb_mouse_report_t));
        if (mouse_queue) {
            mouse_initialized = true;
            ESP_LOGI(TAG, "Mouse report queue created");
        }
    }
}

void usb_mouse_set_connected(bool connected) {
    ensure_initialized();
    mouse_connected = connected;
    if (connected) {
        ESP_LOGI(TAG, ">>> USB Mouse connected and ready <<<");
    } else {
        ESP_LOGI(TAG, "USB Mouse disconnected");
    }
}

bool usb_mouse_is_connected(void) {
    return mouse_connected;
}

bool usb_mouse_available(void) {
    if (!mouse_initialized || !mouse_queue) return false;
    return uxQueueMessagesWaiting(mouse_queue) > 0;
}

bool usb_mouse_read(usb_mouse_report_t *report) {
    if (!mouse_initialized || !mouse_queue || !report) return false;
    return xQueueReceive(mouse_queue, report, 0) == pdTRUE;
}

void usb_mouse_process_report(const uint8_t *data, size_t length) {
    ensure_initialized();
    
    if (!mouse_queue || length < 3) return;
    
    // Standard HID mouse report:
    // Byte 0: Buttons (bit0=left, bit1=right, bit2=middle)
    // Byte 1: X displacement (signed)
    // Byte 2: Y displacement (signed)
    // Byte 3: Wheel (optional, signed)
    
    usb_mouse_report_t report;
    report.buttons = data[0] & 0x07;
    report.dx = (int8_t)data[1];
    report.dy = (int8_t)data[2];
    
    // Only queue if there's actual movement or button change
    if (report.dx != 0 || report.dy != 0 || report.buttons != 0) {
        xQueueSend(mouse_queue, &report, 0);
    }
}

// ============================================================================
// Weak stubs for USB keyboard functions
// These are overridden when usb_keyboard.c is compiled with the USB HID component
// ============================================================================

__attribute__((weak)) esp_err_t usb_keyboard_init(void) {
    ESP_LOGW(TAG, "USB keyboard not available (HID host component not installed)");
    return ESP_ERR_NOT_SUPPORTED;
}

__attribute__((weak)) bool usb_keyboard_available(void) {
    return false;
}

__attribute__((weak)) uint8_t usb_keyboard_read(void) {
    return 0;
}

__attribute__((weak)) bool usb_keyboard_is_initialized(void) {
    return false;
}

__attribute__((weak)) void usb_keyboard_deinit(void) {
}
