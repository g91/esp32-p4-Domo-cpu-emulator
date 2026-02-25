/*
 *  input_esp32.cpp - Keyboard and mouse input for ESP-IDF
 *  BasiliskII ESP32-P4 Port
 *
 *  Phase 1: Stub implementation - input will be wired in Phase 5
 *  ADB functions are implemented in adb.cpp, not here.
 */

#include "sysdeps.h"
#include "prefs.h"
#include "main.h"

#define DEBUG 0
#include "debug.h"

static const char *TAG = "B2_INPUT";

/*
 *  Input initialization
 */
bool InputInit(void)
{
    ESP_LOGI(TAG, "Input initialized (stub)");
    return true;
}

/*
 *  Input deinitialization
 */
void InputExit(void)
{
    ESP_LOGI(TAG, "Input shutdown");
}

/*
 *  Poll input devices (called from main loop)
 *  Phase 5 will read from USB keyboard queue
 */
void InputInterrupt(void)
{
    // TODO Phase 5:
    //   Read keys from usb_keyboard queue
    //   Convert to Mac ADB keycodes
    //   Call ADBKeyDown/ADBKeyUp via adb.cpp
}

/*
 *  Set screen dimensions for coordinate mapping
 */
void InputSetScreenSize(int width, int height)
{
    (void)width;
    (void)height;
}

/*
 *  USB HID keycode to Mac ADB keycode translation table
 *  Will be populated in Phase 5
 */
static const uint8_t usb_to_mac_keycode[256] = { 0 };

/*
 *  Convert USB HID keycode to Mac ADB keycode
 */
int USBToMacKeycode(int usb_key)
{
    if (usb_key < 0 || usb_key >= 256) return -1;
    return usb_to_mac_keycode[usb_key];
}

/*
 *  USB HID keycode to Mac ADB keycode translation table
 *  Will be populated in Phase 5
 */
static const uint8_t usb_to_mac_keycode[256] = { 0 };

/*
 *  Convert USB HID keycode to Mac ADB keycode
 */
int USBToMacKeycode(int usb_key)
{
    if (usb_key < 0 || usb_key >= 256) return -1;
    return usb_to_mac_keycode[usb_key];
}
