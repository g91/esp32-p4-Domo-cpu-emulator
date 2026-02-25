/*
 *  boot_gui_stub.cpp - Stub implementation for boot GUI
 *  BasiliskII ESP32-P4 Port
 *
 *  On ESP32-P4 we skip the boot GUI and use hardcoded/prefs values.
 */

#include "sysdeps.h"
#include "boot_gui.h"
#include "prefs.h"
#include "prefs_items.h"

bool BootGUI_Init(void) { return true; }
void BootGUI_Run(void) {}

const char* BootGUI_GetDiskPath(void)
{
    const char *path = PrefsFindString("disk");
    return path ? path : "";
}

const char* BootGUI_GetCDROMPath(void)
{
    const char *path = PrefsFindString("cdrom");
    return path ? path : "";
}

uint32_t BootGUI_GetRAMSize(void)
{
    int32 size = PrefsFindInt32("ramsize");
    return size > 0 ? (uint32_t)size : 8 * 1024 * 1024;
}

int BootGUI_GetRAMSizeMB(void)
{
    return (int)(BootGUI_GetRAMSize() / (1024 * 1024));
}

const char* BootGUI_GetWiFiSSID(void) { return ""; }
const char* BootGUI_GetWiFiPassword(void) { return ""; }
bool BootGUI_GetWiFiAutoConnect(void) { return false; }
bool BootGUI_GetAudioEnabled(void) { return false; }  // Disable audio on ESP32-P4
bool BootGUI_IsWiFiConnected(void) { return false; }
uint32_t BootGUI_GetWiFiIP(void) { return 0; }
