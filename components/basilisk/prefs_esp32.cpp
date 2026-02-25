/*
 *  prefs_esp32.cpp - Preferences handling for ESP-IDF
 *  BasiliskII ESP32-P4 Port
 *
 *  Hardcodes Quadra 900 configuration (68040, 32-bit, color)
 */

#include "sysdeps.h"
#include "prefs.h"

// Platform-specific preferences items
prefs_desc platform_prefs_items[] = {
    {NULL, TYPE_END, false, NULL}
};

/*
 *  Platform-specific preference defaults
 *  Configures as Quadra 900 with 68040 for Mac OS 7.1-8.1
 */
void AddPlatformPrefsDefaults(void)
{
    // Quadra 900 / 68040 configuration
    PrefsAddInt32("ramsize", 8 * 1024 * 1024);  // 8MB RAM
    PrefsAddInt32("modelid", 14);                // Quadra 900 (Gestalt ID)
    PrefsAddInt32("cpu", 4);                     // 68040
    PrefsAddBool("fpu", false);                  // No FPU emulation
    PrefsAddBool("nosound", true);               // Disable sound (no codec)
    PrefsAddBool("nogui", true);                 // No GUI
    
    // Screen: 480x320, 8-bit color (matches SPI LCD)
    PrefsAddString("screen", "win/480/320/8");
    
    // ROM file on SD card
    PrefsAddString("rom", "/sdcard/mac/Q650.ROM");
    
    // Hard disk image on SD card
    PrefsAddString("disk", "/sdcard/mac/Macintosh.dsk");
    
    // Frame skip (1 = every frame, 4 = every 4th frame)
    PrefsAddInt32("frameskip", 2);
    
    // Boot from first available drive
    PrefsAddInt32("bootdrive", 0);
    PrefsAddInt32("bootdriver", 0);
}

/*
 *  Load preferences from file (no-op on ESP32, use hardcoded defaults)
 */
void LoadPrefs(const char *vmdir)
{
    UNUSED(vmdir);
    // Preferences are set by AddPlatformPrefsDefaults()
}

/*
 *  Save preferences (no-op)
 */
void SavePrefs(void)
{
    // Nothing to save
}

/*
 *  Delete preferences (no-op)
 */
void DeletePrefs(void)
{
}
