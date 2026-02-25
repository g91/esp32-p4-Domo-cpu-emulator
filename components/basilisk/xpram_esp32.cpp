/*
 *  xpram_esp32.cpp - XPRAM (NVRAM) handling for ESP-IDF
 *  BasiliskII ESP32-P4 Port
 *
 *  Stores Mac PRAM settings in a file on SD card
 */

#include "sysdeps.h"
#include "xpram.h"

#include <stdio.h>

#define XPRAM_FILE "/sdcard/mac/BasiliskII_XPRAM"

// XPRAM data (256 bytes)
uint8 XPRAM[XPRAM_SIZE];

/*
 *  Load XPRAM from file
 */
void LoadXPRAM(const char *vmdir)
{
    UNUSED(vmdir);
    memset(XPRAM, 0, XPRAM_SIZE);
    
    FILE *f = fopen(XPRAM_FILE, "rb");
    if (f) {
        fread(XPRAM, 1, XPRAM_SIZE, f);
        fclose(f);
        ESP_LOGI("B2_XPRAM", "XPRAM loaded from %s", XPRAM_FILE);
    } else {
        ESP_LOGI("B2_XPRAM", "No XPRAM file found, using defaults");
    }
}

/*
 *  Save XPRAM to file
 */
void SaveXPRAM(void)
{
    FILE *f = fopen(XPRAM_FILE, "wb");
    if (f) {
        fwrite(XPRAM, 1, XPRAM_SIZE, f);
        fclose(f);
    }
}

/*
 *  Delete XPRAM file
 */
void ZapPRAM(void)
{
    memset(XPRAM, 0, XPRAM_SIZE);
    SaveXPRAM();
}
