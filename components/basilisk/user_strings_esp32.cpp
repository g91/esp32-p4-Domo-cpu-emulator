/*
 *  user_strings_esp32.cpp - ESP32-specific user interface strings
 *
 *  BasiliskII ESP32 Port
 */

#include "sysdeps.h"
#include "user_strings.h"

// Platform-specific string definitions
user_string_def platform_strings[] = {
    {STR_ESP32_MENU_TITLE, "BasiliskII ESP32"},
    {STR_ESP32_ROM_LOAD_ERR, "Cannot load ROM file from SD card"},
    {STR_ESP32_RAM_ALLOC_ERR, "Cannot allocate Mac RAM from PSRAM"},
    {STR_ESP32_SD_INIT_ERR, "Cannot initialize SD card"},
    {STR_ESP32_DISK_OPEN_ERR, "Cannot open disk image"},
    {-1, NULL}
};
