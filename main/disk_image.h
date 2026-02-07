/*
 * Virtual Disk Image Manager
 * Supports floppy and hard disk images for i8086 emulator
 */

#ifndef DISK_IMAGE_H
#define DISK_IMAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of mounted disks
#define MAX_DISK_DRIVES     4

// Drive types
typedef enum {
    DISK_TYPE_NONE = 0,
    DISK_TYPE_FLOPPY_360K,   // 360KB 5.25" floppy
    DISK_TYPE_FLOPPY_720K,   // 720KB 3.5" floppy
    DISK_TYPE_FLOPPY_1M2,    // 1.2MB 5.25" floppy
    DISK_TYPE_FLOPPY_1M44,   // 1.44MB 3.5" floppy
    DISK_TYPE_FLOPPY_2M88,   // 2.88MB 3.5" floppy
    DISK_TYPE_HARD_DISK      // Hard disk (any size)
} disk_type_t;

// Disk geometry
typedef struct {
    uint16_t cylinders;
    uint8_t heads;
    uint8_t sectors_per_track;
    uint32_t total_sectors;
    uint32_t bytes_per_sector;
} disk_geometry_t;

// Disk information
typedef struct {
    bool mounted;
    disk_type_t type;
    char filename[256];
    FILE *file;
    disk_geometry_t geometry;
    uint64_t size_bytes;
    bool read_only;
    uint32_t read_count;
    uint32_t write_count;
} disk_info_t;

/**
 * Initialize disk image system
 * @return ESP_OK on success
 */
esp_err_t disk_init(void);

/**
 * Mount disk image file
 * @param drive Drive number (0-3: A,B,C,D)
 * @param filename Path to disk image file on SD card
 * @param read_only Mount as read-only
 * @return ESP_OK on success
 */
esp_err_t disk_mount(uint8_t drive, const char *filename, bool read_only);

/**
 * Unmount disk
 * @param drive Drive number
 * @return ESP_OK on success
 */
esp_err_t disk_unmount(uint8_t drive);

/**
 * Create new blank disk image
 * @param filename Output filename
 * @param type Disk type (determines size/geometry)
 * @return ESP_OK on success
 */
esp_err_t disk_create_image(const char *filename, disk_type_t type);

/**
 * Create custom sized hard disk image
 * @param filename Output filename
 * @param size_mb Size in megabytes
 * @return ESP_OK on success
 */
esp_err_t disk_create_hd_image(const char *filename, uint32_t size_mb);

/**
 * Get disk information
 * @param drive Drive number
 * @param info Pointer to info structure to fill
 * @return ESP_OK if drive is mounted
 */
esp_err_t disk_get_info(uint8_t drive, disk_info_t *info);

/**
 * Check if drive is mounted
 * @param drive Drive number
 * @return true if mounted
 */
bool disk_is_mounted(uint8_t drive);

/**
 * Read sector(s) from disk
 * @param drive Drive number
 * @param lba Logical block address (sector number)
 * @param count Number of sectors to read
 * @param buffer Buffer to read into (must be count * 512 bytes)
 * @return Number of sectors read, or -1 on error
 */
int disk_read_sectors(uint8_t drive, uint32_t lba, uint16_t count, uint8_t *buffer);

/**
 * Write sector(s) to disk
 * @param drive Drive number
 * @param lba Logical block address
 * @param count Number of sectors to write
 * @param buffer Data to write (must be count * 512 bytes)
 * @return Number of sectors written, or -1 on error
 */
int disk_write_sectors(uint8_t drive, uint32_t lba, uint16_t count, const uint8_t *buffer);

/**
 * Convert CHS (Cylinder/Head/Sector) to LBA
 * @param drive Drive number
 * @param cylinder Cylinder number
 * @param head Head number
 * @param sector Sector number (1-based)
 * @return LBA sector number, or -1 on error
 */
int32_t disk_chs_to_lba(uint8_t drive, uint16_t cylinder, uint8_t head, uint8_t sector);

/**
 * Auto-detect disk type from file size
 * @param filename Image filename
 * @return Detected disk type
 */
disk_type_t disk_detect_type(const char *filename);

/**
 * Get disk type name string
 * @param type Disk type
 * @return Human-readable type name
 */
const char *disk_type_name(disk_type_t type);

/**
 * List all mounted disks
 * @param buffer Buffer to write list (one line per disk)
 * @param size Buffer size
 */
void disk_list(char *buffer, size_t size);

/**
 * Flush all disk write caches
 */
void disk_sync_all(void);

/**
 * Cleanup and unmount all disks
 */
void disk_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // DISK_IMAGE_H
