/*
 *  sys_esp32.cpp - System I/O functions for ESP-IDF
 *  BasiliskII ESP32-P4 Port
 *
 *  Handles disk/CD-ROM/floppy file I/O via POSIX FILE* on SDMMC filesystem
 */

#include "sysdeps.h"
#include "prefs.h"
#include "sys.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#define DEBUG 0
#include "debug.h"

static const char *TAG = "B2_SYS";

/*
 *  File handle structure using POSIX FILE
 */
struct mac_file_handle {
    FILE *f;
    bool read_only;
    loff_t file_size;
    bool is_floppy;
    bool is_cdrom;
    bool dirty;         // Has pending writes
};

// Track all open handles for periodic flush
#define MAX_FILE_HANDLES 16
static mac_file_handle *open_handles[MAX_FILE_HANDLES] = {0};
static int num_open_handles = 0;

/*
 *  Initialize system I/O
 */
void SysInit(void)
{
    ESP_LOGI(TAG, "System I/O initialized (POSIX/SDMMC)");
}

/*
 *  Deinitialize system I/O
 */
void SysExit(void)
{
    // Flush and close all open handles
    for (int i = 0; i < num_open_handles; i++) {
        if (open_handles[i] && open_handles[i]->f) {
            if (open_handles[i]->dirty) {
                fflush(open_handles[i]->f);
            }
        }
    }
    ESP_LOGI(TAG, "System I/O shutdown");
}

/*
 *  Open file
 */
void *Sys_open(const char *name, bool read_only)
{
    if (!name || !name[0]) return NULL;
    
    ESP_LOGI(TAG, "Opening: %s (%s)", name, read_only ? "RO" : "RW");
    
    FILE *f = fopen(name, read_only ? "rb" : "r+b");
    if (!f) {
        ESP_LOGW(TAG, "Cannot open: %s", name);
        return NULL;
    }
    
    // Get file size
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    mac_file_handle *fh = (mac_file_handle *)malloc(sizeof(mac_file_handle));
    if (!fh) {
        fclose(f);
        return NULL;
    }
    
    fh->f = f;
    fh->read_only = read_only;
    fh->file_size = size;
    fh->dirty = false;
    
    // Detect floppy by size
    fh->is_floppy = (size == 400*1024 || size == 800*1024 || 
                     size == 1440*1024 || size == 2880*1024);
    fh->is_cdrom = false;
    
    // Check for .iso extension
    int nlen = strlen(name);
    if (nlen > 4) {
        const char *ext = name + nlen - 4;
        if (strcasecmp(ext, ".iso") == 0 || strcasecmp(ext, ".cdr") == 0) {
            fh->is_cdrom = true;
            fh->is_floppy = false;
        }
    }
    
    // Track handle for flush
    if (num_open_handles < MAX_FILE_HANDLES) {
        open_handles[num_open_handles++] = fh;
    }
    
    ESP_LOGI(TAG, "Opened: %s, size=%ld, floppy=%d, cdrom=%d", 
             name, size, fh->is_floppy, fh->is_cdrom);
    
    return fh;
}

/*
 *  Close file
 */
void Sys_close(void *arg)
{
    mac_file_handle *fh = (mac_file_handle *)arg;
    if (!fh) return;
    
    if (fh->f) {
        if (fh->dirty) fflush(fh->f);
        fclose(fh->f);
    }
    
    // Remove from tracking
    for (int i = 0; i < num_open_handles; i++) {
        if (open_handles[i] == fh) {
            open_handles[i] = open_handles[--num_open_handles];
            break;
        }
    }
    
    free(fh);
}

/*
 *  Read from file
 */
size_t Sys_read(void *arg, void *buffer, loff_t offset, size_t length)
{
    mac_file_handle *fh = (mac_file_handle *)arg;
    if (!fh || !fh->f) return 0;
    
    if (fseek(fh->f, offset, SEEK_SET) != 0) return 0;
    return fread(buffer, 1, length, fh->f);
}

/*
 *  Write to file
 */
size_t Sys_write(void *arg, void *buffer, loff_t offset, size_t length)
{
    mac_file_handle *fh = (mac_file_handle *)arg;
    if (!fh || !fh->f || fh->read_only) return 0;
    
    if (fseek(fh->f, offset, SEEK_SET) != 0) return 0;
    size_t written = fwrite(buffer, 1, length, fh->f);
    if (written > 0) fh->dirty = true;
    return written;
}

/*
 *  Get file size
 */
loff_t SysGetFileSize(void *arg)
{
    mac_file_handle *fh = (mac_file_handle *)arg;
    if (!fh) return 0;
    return fh->file_size;
}

/*
 *  Eject (no-op for file images)
 */
void SysEject(void *arg)
{
    UNUSED(arg);
}

/*
 *  Format (not supported)
 */
bool SysFormat(void *arg)
{
    UNUSED(arg);
    return false;
}

/*
 *  Check if file is read-only
 */
bool SysIsReadOnly(void *arg)
{
    mac_file_handle *fh = (mac_file_handle *)arg;
    if (!fh) return true;
    return fh->read_only;
}

/*
 *  Check if this is a fixed disk (not ejectable)
 */
bool SysIsFixedDisk(void *arg)
{
    mac_file_handle *fh = (mac_file_handle *)arg;
    if (!fh) return true;
    return !fh->is_floppy && !fh->is_cdrom;
}

/*
 *  Check if disk is a CD-ROM
 */
bool SysIsDiskInserted(void *arg)
{
    mac_file_handle *fh = (mac_file_handle *)arg;
    return fh != NULL && fh->f != NULL;
}

/*
 *  Prevent/allow medium removal (no-op)
 */
void SysPreventRemoval(void *arg)
{
    UNUSED(arg);
}

void SysAllowRemoval(void *arg)
{
    UNUSED(arg);
}

/*
 *  Get CD-ROM TOC (stub)
 */
bool SysCDReadTOC(void *arg, uint8 *toc)
{
    UNUSED(arg);
    UNUSED(toc);
    return false;
}

/*
 *  Get CD-ROM position (stub)
 */
bool SysCDGetPosition(void *arg, uint8 *pos)
{
    UNUSED(arg);
    UNUSED(pos);
    return false;
}

/*
 *  Play CD-ROM audio (stub)
 */
bool SysCDPlay(void *arg, uint8 start_m, uint8 start_s, uint8 start_f,
               uint8 end_m, uint8 end_s, uint8 end_f)
{
    UNUSED(arg);
    return false;
}

/*
 *  Pause CD-ROM audio (stub)
 */
bool SysCDPause(void *arg)
{
    UNUSED(arg);
    return false;
}

/*
 *  Resume CD-ROM audio (stub)
 */
bool SysCDResume(void *arg)
{
    UNUSED(arg);
    return false;
}

/*
 *  Stop CD-ROM audio (stub)
 */
bool SysCDStop(void *arg, uint8 lead_out_m, uint8 lead_out_s, uint8 lead_out_f)
{
    UNUSED(arg);
    return false;
}

/*
 *  Scan for CD-ROM drives (stub)
 */
void SysCDScan(void)
{
}

/*
 *  Periodic flush - write dirty buffers to SD card
 *  Called from main emulator loop every 2 seconds
 */
void Sys_periodic_flush(void)
{
    for (int i = 0; i < num_open_handles; i++) {
        mac_file_handle *fh = open_handles[i];
        if (fh && fh->f && fh->dirty) {
            fflush(fh->f);
            fh->dirty = false;
        }
    }
}
