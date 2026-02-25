/*
 * ESP32 Native OS - Windows 3.1-style Desktop Environment
 * Runs natively on ESP32-P4 Xtensa LX7 dual-core @ 400MHz
 * Uses video_card.c GPU for 480x320 RGB565 display
 * Supports USB HID + PS/2 mouse and keyboard
 */

#ifndef ESP32_OS_H
#define ESP32_OS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Main entry point - boots the Windows 3.1-style desktop.
 * Blocks until the user exits (Ctrl+Alt+Del or power cycle).
 * Must be called from a FreeRTOS task with at least 16KB stack.
 */
void esp32_os_run(void *arg);

#ifdef __cplusplus
}
#endif

#endif // ESP32_OS_H
