/*
 * USB Host HID Keyboard Driver Implementation
 * For ESP32-P4-NANO USB Type-A port (USB OTG 2.0 HS)
 * Based on esp32-quake example and usb_host_hid component
 */

#include "usb_keyboard.h"
#include "ps2_keyboard.h"   // Shared key defines (PS2_KEY_F1, PS2_KEY_UP, etc.)
#include "usb_mouse.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"

static const char *TAG = "USB_KBD";

#define KEY_QUEUE_SIZE 128

// Keyboard state
static bool initialized = false;
static QueueHandle_t key_queue = NULL;
static hid_host_device_handle_t kbd_handle = NULL;
static hid_host_device_handle_t mouse_handle = NULL;

// USB HID keycodes to ASCII/special key translation table
// Uses PS2_KEY_* defines from ps2_keyboard.h for function/special keys
// so all keyboard drivers produce the same key codes
static const uint8_t hid_to_ascii[] = {
    0,   0,   0,   0,   'a', 'b', 'c', 'd', // 0x00-0x07
    'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', // 0x08-0x0F
    'm', 'n', 'o', 'p', 'q', 'r', 's', 't', // 0x10-0x17
    'u', 'v', 'w', 'x', 'y', 'z', '1', '2', // 0x18-0x1F
    '3', '4', '5', '6', '7', '8', '9', '0', // 0x20-0x27
    '\r',PS2_KEY_ESCAPE, PS2_KEY_BACKSPACE, '\t',' ', '-', '=', '[', // 0x28-0x2F
    ']', '\\', '#', ';', '\'','`', ',', '.', // 0x30-0x37
    '/', 0,                                   // 0x38-0x39 (CapsLock)
    PS2_KEY_F1,  PS2_KEY_F2,  PS2_KEY_F3,  PS2_KEY_F4,   // 0x3A-0x3D
    PS2_KEY_F5,  PS2_KEY_F6,  PS2_KEY_F7,  PS2_KEY_F8,   // 0x3E-0x41
    PS2_KEY_F9,  PS2_KEY_F10, PS2_KEY_F11, PS2_KEY_F12,  // 0x42-0x45
    0,   0,   0,                              // 0x46-0x48 (PrtSc, ScrLk, Pause)
    PS2_KEY_INSERT, PS2_KEY_HOME, PS2_KEY_PAGEUP,         // 0x49-0x4B
    PS2_KEY_DELETE, PS2_KEY_END,  PS2_KEY_PAGEDOWN,       // 0x4C-0x4E
    PS2_KEY_RIGHT,  PS2_KEY_LEFT, PS2_KEY_DOWN, PS2_KEY_UP, // 0x4F-0x52
};

// Shift key translations
static const uint8_t hid_to_ascii_shift[] = {
    0,   0,   0,   0,   'A', 'B', 'C', 'D', // 0x00-0x07
    'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', // 0x08-0x0F
    'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', // 0x10-0x17
    'U', 'V', 'W', 'X', 'Y', 'Z', '!', '@', // 0x18-0x1F
    '#', '$', '%', '^', '&', '*', '(', ')', // 0x20-0x27
    '\r',PS2_KEY_ESCAPE, PS2_KEY_BACKSPACE, '\t',' ', '_', '+', '{', // 0x28-0x2F
    '}', '|', '~', ':', '"', '~', '<', '>', // 0x30-0x37
    '?', 0,                                   // 0x38-0x39 (CapsLock)
    PS2_KEY_F1,  PS2_KEY_F2,  PS2_KEY_F3,  PS2_KEY_F4,   // 0x3A-0x3D
    PS2_KEY_F5,  PS2_KEY_F6,  PS2_KEY_F7,  PS2_KEY_F8,   // 0x3E-0x41
    PS2_KEY_F9,  PS2_KEY_F10, PS2_KEY_F11, PS2_KEY_F12,  // 0x42-0x45
    0,   0,   0,                              // 0x46-0x48 (PrtSc, ScrLk, Pause)
    PS2_KEY_INSERT, PS2_KEY_HOME, PS2_KEY_PAGEUP,         // 0x49-0x4B
    PS2_KEY_DELETE, PS2_KEY_END,  PS2_KEY_PAGEDOWN,       // 0x4C-0x4E
    PS2_KEY_RIGHT,  PS2_KEY_LEFT, PS2_KEY_DOWN, PS2_KEY_UP, // 0x4F-0x52
};

// Previous HID report state for key-change detection
// USB keyboards send all currently-pressed keys in every report.
// Without tracking, held keys get re-enqueued every report cycle (~8ms),
// flooding the queue and drowning out new keypresses.
static uint8_t prev_keys[6] = {0};
static uint8_t prev_modifier = 0;

// Check if a keycode was already pressed in the previous report
static bool was_key_pressed(uint8_t keycode) {
    for (int i = 0; i < 6; i++) {
        if (prev_keys[i] == keycode) return true;
    }
    return false;
}

// HID interface callback - handles keyboard input reports
static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                       const hid_host_interface_event_t event,
                                       void *arg)
{
    uint8_t data[64] = { 0 };
    size_t data_length = 0;

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                                  data, 64, &data_length));
        
        // Check if this is a mouse report
        if (hid_device_handle == mouse_handle) {
            usb_mouse_process_report(data, data_length);
            break;
        }
        
        if (data_length >= 8) {
            // HID keyboard report: Byte 0 = modifiers, Bytes 2-7 = keycodes
            uint8_t modifier = data[0];
            bool shift = (modifier & 0x22) != 0; // Left or Right Shift
            
            // Only process NEWLY pressed keys (not held from previous report)
            for (int i = 2; i < 8; i++) {
                uint8_t keycode = data[i];
                if (keycode == 0) break;
                if (keycode == 1) continue;  // Error rollover
                
                // Skip keys that were already pressed in the previous report
                if (was_key_pressed(keycode)) continue;
                
                // This is a NEW keypress - convert and enqueue
                uint8_t ascii = 0;
                if (keycode < sizeof(hid_to_ascii)) {
                    ascii = shift ? hid_to_ascii_shift[keycode] : hid_to_ascii[keycode];
                }
                
                // Debug: log keycode and resulting ASCII
                ESP_LOGI(TAG, "HID key 0x%02X -> ASCII 0x%02X ('%c')", keycode, ascii, (ascii >= 0x20 && ascii < 0x7F) ? ascii : '?');
                
                if (ascii != 0 && key_queue != NULL) {
                    xQueueSend(key_queue, &ascii, 0);
                }
            }
            
            // Save current report as previous for next comparison
            memcpy(prev_keys, &data[2], 6);
            prev_modifier = modifier;
        }
        break;
        
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HID Device disconnected");
        if (hid_device_handle == kbd_handle) {
            kbd_handle = NULL;
        }
        if (hid_device_handle == mouse_handle) {
            mouse_handle = NULL;
            usb_mouse_set_connected(false);
        }
        break;
        
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "HID transfer error");
        break;
        
    default:
        break;
    }
}

// HID Host Device event callback
static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                    const hid_host_driver_event_t event,
                                    void *arg)
{
    hid_host_dev_params_t dev_params;
    
    ESP_LOGI(TAG, "HID Device event received: %d", event);
    
    switch (event) {
    case HID_HOST_DRIVER_EVENT_CONNECTED:
        // Get device parameters
        ESP_LOGI(TAG, "HID Device connecting - getting parameters...");
        if (hid_host_device_get_params(hid_device_handle, &dev_params) == ESP_OK) {
            ESP_LOGI(TAG, "HID Device connected:");
            ESP_LOGI(TAG, "  Address: %d", dev_params.addr);
            ESP_LOGI(TAG, "  Interface: %d", dev_params.iface_num);
            ESP_LOGI(TAG, "  Protocol: %d", dev_params.proto);
            ESP_LOGI(TAG, "  SubClass: %d", dev_params.sub_class);
            
            bool is_mouse = (dev_params.proto == HID_PROTOCOL_MOUSE);
            
            // Configure device with interface callback
            const hid_host_device_config_t dev_config = {
                .callback = hid_host_interface_callback,
                .callback_arg = NULL
            };
            
            // Open the device
            ESP_LOGI(TAG, "Opening HID device (%s)...", is_mouse ? "mouse" : "keyboard");
            if (hid_host_device_open(hid_device_handle, &dev_config) == ESP_OK) {
                ESP_LOGI(TAG, "✓ USB HID device opened successfully");
                
                if (is_mouse) {
                    mouse_handle = hid_device_handle;
                } else {
                    kbd_handle = hid_device_handle;
                }
                
                // Start receiving reports
                ESP_LOGI(TAG, "Starting HID device...");
                if (hid_host_device_start(hid_device_handle) == ESP_OK) {
                    if (is_mouse) {
                        usb_mouse_set_connected(true);
                    } else {
                        ESP_LOGI(TAG, ">>> USB Keyboard ready for input! <<<");
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to start HID device");
                }
            } else {
                ESP_LOGE(TAG, "Failed to open HID device");
            }
        } else {
            ESP_LOGE(TAG, "Failed to get device parameters");
        }
        break;
        
    default:
        ESP_LOGW(TAG, "Unhandled HID event: %d", event);
        break;
    }
}

// USB lib task - handles USB Host library events
static void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGW(TAG, "No more clients");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All devices freed");
        }
    }
}

// Initialize USB Host and HID keyboard driver
esp_err_t usb_keyboard_init(void)
{
    ESP_LOGI(TAG, "Starting USB keyboard initialization...");
    
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    // Create key queue
    ESP_LOGI(TAG, "Creating key queue (size=%d)...", KEY_QUEUE_SIZE);
    key_queue = xQueueCreate(KEY_QUEUE_SIZE, sizeof(uint8_t));
    if (key_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create key queue");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Key queue created successfully");
    
    // Install USB Host driver
    ESP_LOGI(TAG, "Installing USB Host driver...");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    
    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB Host install failed: %s", esp_err_to_name(ret));
        vQueueDelete(key_queue);
        key_queue = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "USB Host driver installed successfully");
    
    // Create USB Host library task
    ESP_LOGI(TAG, "Creating USB event handler task...");
    if (xTaskCreate(usb_lib_task, "usb_events", 4096, NULL, 10, NULL) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create USB event task");
        usb_host_uninstall();
        vQueueDelete(key_queue);
        key_queue = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "USB event task created successfully");
    
    // Install HID Host driver
    ESP_LOGI(TAG, "Installing HID Host driver...");
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = NULL
    };

    ret = hid_host_install(&hid_host_driver_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID Host install failed: %s", esp_err_to_name(ret));
        usb_host_uninstall();
        vQueueDelete(key_queue);
        key_queue = NULL;
        return ret;
    }
    ESP_LOGI(TAG, "HID Host driver installed successfully");
    
    initialized = true;
    ESP_LOGI(TAG, "=== USB keyboard driver ready - waiting for device ===");
    ESP_LOGI(TAG, ">>> Please connect USB keyboard to Type-A USB port <<<");
    return ESP_OK;
}

// Check if keyboard has data available
bool usb_keyboard_available(void)
{
    if (!initialized || key_queue == NULL) {
        return false;
    }
    return uxQueueMessagesWaiting(key_queue) > 0;
}

// Read a character from the keyboard (non-blocking)
uint8_t usb_keyboard_read(void)
{
    if (!initialized || key_queue == NULL) {
        return 0;
    }
    
    uint8_t key = 0;
    xQueueReceive(key_queue, &key, 0); // Non-blocking read
    return key;
}

// Check if USB keyboard is initialized
bool usb_keyboard_is_initialized(void)
{
    return initialized;
}

// Deinitialize USB keyboard
void usb_keyboard_deinit(void)
{
    if (!initialized) {
        return;
    }
    
    if (kbd_handle != NULL) {
        hid_host_device_close(kbd_handle);
        kbd_handle = NULL;
    }
    
    hid_host_uninstall();
    usb_host_uninstall();
    
    if (key_queue != NULL) {
        vQueueDelete(key_queue);
        key_queue = NULL;
    }
    
    initialized = false;
    ESP_LOGI(TAG, "USB keyboard driver deinitialized");
}
