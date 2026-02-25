/*
 * Virtual Disk Image Manager Implementation
 * Handles disk image files for i8086 emulator
 */

#include "disk_image.h"
#include <string.h>
#include <sys/stat.h>
#include "esp_log.h"

static const char *TAG = "DiskImg";

// Disk drive table
static disk_info_t s_drives[MAX_DISK_DRIVES];

// Standard floppy geometries
static const disk_geometry_t floppy_geometries[] = {
    {0, 0, 0, 0, 512},                           // DISK_TYPE_NONE
    {40, 2, 9, 720, 512},                        // 360KB
    {80, 2, 9, 1440, 512},                       // 720KB
    {80, 2, 15, 2400, 512},                      // 1.2MB
    {80, 2, 18, 2880, 512},                      // 1.44MB
    {80, 2, 36, 5760, 512},                      // 2.88MB
};

esp_err_t disk_init(void)
{
    memset(s_drives, 0, sizeof(s_drives));
    ESP_LOGI(TAG, "Disk image system initialized");
    return ESP_OK;
}

esp_err_t disk_mount(uint8_t drive, const char *filename, bool read_only)
{
    if (drive >= MAX_DISK_DRIVES) {
        ESP_LOGE(TAG, "Invalid drive number: %d", drive);
        return ESP_ERR_INVALID_ARG;
    }

    if (!filename) {
        ESP_LOGE(TAG, "No filename specified");
        return ESP_ERR_INVALID_ARG;
    }

    // Unmount if already mounted
    if (s_drives[drive].mounted) {
        disk_unmount(drive);
    }

    // Open file
    const char *mode = read_only ? "rb" : "r+b";
    FILE *f = fopen(filename, mode);
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s", filename);
        return ESP_FAIL;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    uint64_t size = ftell(f);
    fseek(f, 0, SEEK_SET);

    // Detect disk type from size
    disk_type_t type = disk_detect_type(filename);
    
    disk_info_t *disk = &s_drives[drive];
    disk->mounted = true;
    disk->type = type;
    strncpy(disk->filename, filename, sizeof(disk->filename) - 1);
    disk->file = f;
    disk->size_bytes = size;
    disk->read_only = read_only;
    disk->read_count = 0;
    disk->write_count = 0;

    // Set geometry
    if (type >= DISK_TYPE_FLOPPY_360K && type <= DISK_TYPE_FLOPPY_2M88) {
        disk->geometry = floppy_geometries[type];
    } else {
        // Hard disk - try to read geometry from BPB in boot sector
        uint32_t total_sectors = size / 512;
        disk->geometry.bytes_per_sector = 512;
        disk->geometry.total_sectors = total_sectors;

        // Defaults
        uint16_t spt = 63;
        uint16_t heads = 16;

        // Read boot sector to check for BPB/MBR geometry
        uint8_t bootsect[512];
        if (fread(bootsect, 512, 1, f) == 1) {
            fseek(f, 0, SEEK_SET);
            bool found_geometry = false;

            // Check if this is a VBR (Volume Boot Record) with BPB
            // VBRs start with JMP (0xEB xx 0x90 or 0xE9 xx xx) and have BPB
            bool looks_like_vbr = (bootsect[0] == 0xEB || bootsect[0] == 0xE9);
            uint16_t bpb_bps = bootsect[0x0B] | (bootsect[0x0C] << 8);
            uint16_t bpb_spt = bootsect[0x18] | (bootsect[0x19] << 8);
            uint16_t bpb_heads = bootsect[0x1A] | (bootsect[0x1B] << 8);

            if (looks_like_vbr && bpb_bps == 512 &&
                bpb_spt >= 1 && bpb_spt <= 63 &&
                bpb_heads >= 1 && bpb_heads <= 255) {
                // Valid BPB geometry found
                spt = bpb_spt;
                heads = bpb_heads;
                found_geometry = true;
                ESP_LOGI(TAG, "  Geometry from BPB: H=%d S=%d", heads, spt);
            }

            // If not a VBR, check for MBR partition table
            if (!found_geometry && bootsect[510] == 0x55 && bootsect[511] == 0xAA) {
                // Scan all 4 partition entries for a valid one
                for (int p = 0; p < 4 && !found_geometry; p++) {
                    uint8_t *part = &bootsect[0x1BE + p * 16];
                    uint8_t boot_ind = part[0];
                    uint8_t part_type = part[4];

                    if ((boot_ind == 0x80 || boot_ind == 0x00) && part_type != 0) {
                        // Extract geometry from partition end CHS
                        uint8_t end_head = part[5];
                        uint8_t end_sec = part[6] & 0x3F;
                        if (end_sec > 0 && end_head > 0) {
                            heads = end_head + 1;
                            spt = end_sec;
                            found_geometry = true;
                            ESP_LOGI(TAG, "  Geometry from MBR part%d: H=%d S=%d", p, heads, spt);
                        }
                    }
                }
            }

            if (!found_geometry) {
                ESP_LOGI(TAG, "  Using default geometry: H=%d S=%d", heads, spt);
            }
        } else {
            fseek(f, 0, SEEK_SET);
        }

        disk->geometry.sectors_per_track = spt;
        disk->geometry.heads = heads;
        disk->geometry.cylinders = total_sectors / (spt * heads);
        if (disk->geometry.cylinders == 0 && total_sectors > 0) {
            disk->geometry.cylinders = 1;
        }
    }

    ESP_LOGI(TAG, "Mounted drive %d: %s (%s, %llu bytes, %s)",
             drive, filename, disk_type_name(type), size,
             read_only ? "RO" : "RW");
    ESP_LOGI(TAG, "  Geometry: C=%d H=%d S=%d",
             disk->geometry.cylinders,
             disk->geometry.heads,
             disk->geometry.sectors_per_track);

    return ESP_OK;
}

esp_err_t disk_unmount(uint8_t drive)
{
    if (drive >= MAX_DISK_DRIVES) {
        return ESP_ERR_INVALID_ARG;
    }

    disk_info_t *disk = &s_drives[drive];
    if (!disk->mounted) {
        return ESP_OK;
    }

    if (disk->file) {
        fclose(disk->file);
    }

    ESP_LOGI(TAG, "Unmounted drive %d (%u reads, %u writes)",
             drive, disk->read_count, disk->write_count);

    memset(disk, 0, sizeof(disk_info_t));
    return ESP_OK;
}

esp_err_t disk_create_image(const char *filename, disk_type_t type)
{
    if (type < DISK_TYPE_FLOPPY_360K || type > DISK_TYPE_FLOPPY_2M88) {
        ESP_LOGE(TAG, "Invalid floppy type for create_image");
        return ESP_ERR_INVALID_ARG;
    }

    const disk_geometry_t *geom = &floppy_geometries[type];
    uint32_t size = geom->total_sectors * geom->bytes_per_sector;

    FILE *f = fopen(filename, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to create %s", filename);
        return ESP_FAIL;
    }

    // Write zeros
    uint8_t buffer[512] = {0};
    for (uint32_t i = 0; i < geom->total_sectors; i++) {
        if (fwrite(buffer, 512, 1, f) != 1) {
            fclose(f);
            remove(filename);
            return ESP_FAIL;
        }
    }

    fclose(f);
    ESP_LOGI(TAG, "Created %s (%u KB, %s)", filename, size / 1024, disk_type_name(type));
    return ESP_OK;
}

esp_err_t disk_create_hd_image(const char *filename, uint32_t size_mb)
{
    if (size_mb == 0 || size_mb > 2048) {
        ESP_LOGE(TAG, "Invalid HD size: %u MB (max 2048)", size_mb);
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(filename, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to create %s", filename);
        return ESP_FAIL;
    }

    // Write zeros in 64KB chunks
    uint8_t *buffer = malloc(65536);
    if (!buffer) {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }
    memset(buffer, 0, 65536);

    uint32_t total_chunks = (size_mb * 1024) / 64;
    for (uint32_t i = 0; i < total_chunks; i++) {
        if (fwrite(buffer, 65536, 1, f) != 1) {
            free(buffer);
            fclose(f);
            remove(filename);
            return ESP_FAIL;
        }
    }

    free(buffer);
    fclose(f);
    ESP_LOGI(TAG, "Created HD image %s (%u MB)", filename, size_mb);
    return ESP_OK;
}

esp_err_t disk_get_info(uint8_t drive, disk_info_t *info)
{
    if (drive >= MAX_DISK_DRIVES || !info) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_drives[drive].mounted) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(info, &s_drives[drive], sizeof(disk_info_t));
    return ESP_OK;
}

bool disk_is_mounted(uint8_t drive)
{
    return (drive < MAX_DISK_DRIVES && s_drives[drive].mounted);
}

int disk_read_sectors(uint8_t drive, uint32_t lba, uint16_t count, uint8_t *buffer)
{
    if (drive >= MAX_DISK_DRIVES || !buffer) {
        return -1;
    }

    disk_info_t *disk = &s_drives[drive];
    if (!disk->mounted || !disk->file) {
        return -1;
    }

    // Check bounds
    if (lba + count > disk->geometry.total_sectors) {
        ESP_LOGW(TAG, "Read beyond disk: LBA %u count %u (max %u)",
                 lba, count, disk->geometry.total_sectors);
        return -1;
    }

    // Seek and read
    long offset = lba * 512L;
    if (fseek(disk->file, offset, SEEK_SET) != 0) {
        return -1;
    }

    size_t bytes_to_read = count * 512;
    size_t bytes_read = fread(buffer, 1, bytes_to_read, disk->file);
    
    disk->read_count += count;
    return bytes_read / 512;
}

int disk_write_sectors(uint8_t drive, uint32_t lba, uint16_t count, const uint8_t *buffer)
{
    if (drive >= MAX_DISK_DRIVES || !buffer) {
        return -1;
    }

    disk_info_t *disk = &s_drives[drive];
    if (!disk->mounted || !disk->file) {
        return -1;
    }

    if (disk->read_only) {
        ESP_LOGW(TAG, "Write attempt to read-only disk %d", drive);
        return -1;
    }

    // Check bounds
    if (lba + count > disk->geometry.total_sectors) {
        ESP_LOGW(TAG, "Write beyond disk: LBA %u count %u", lba, count);
        return -1;
    }

    // Seek and write
    long offset = lba * 512L;
    if (fseek(disk->file, offset, SEEK_SET) != 0) {
        return -1;
    }

    size_t bytes_to_write = count * 512;
    size_t bytes_written = fwrite(buffer, 1, bytes_to_write, disk->file);
    
    if (bytes_written == bytes_to_write) {
        fflush(disk->file);
        disk->write_count += count;
    }

    return bytes_written / 512;
}

int32_t disk_chs_to_lba(uint8_t drive, uint16_t cylinder, uint8_t head, uint8_t sector)
{
    if (drive >= MAX_DISK_DRIVES) {
        return -1;
    }

    disk_info_t *disk = &s_drives[drive];
    if (!disk->mounted) {
        return -1;
    }

    disk_geometry_t *geom = &disk->geometry;

    // Sector is 1-based in CHS
    if (sector == 0 || sector > geom->sectors_per_track) {
        return -1;
    }
    if (head >= geom->heads) {
        return -1;
    }

    uint32_t lba = (cylinder * geom->heads + head) * geom->sectors_per_track + (sector - 1);

    // Validate against total sectors (more reliable than cylinder count)
    if (lba >= geom->total_sectors) {
        return -1;
    }
    return lba;
}

disk_type_t disk_detect_type(const char *filename)
{
    struct stat st;
    if (stat(filename, &st) != 0) {
        return DISK_TYPE_NONE;
    }

    uint64_t size = st.st_size;

    // Check floppy sizes
    if (size == 360 * 1024) return DISK_TYPE_FLOPPY_360K;
    if (size == 720 * 1024) return DISK_TYPE_FLOPPY_720K;
    if (size == 1200 * 1024) return DISK_TYPE_FLOPPY_1M2;
    if (size == 1440 * 1024) return DISK_TYPE_FLOPPY_1M44;
    if (size == 2880 * 1024) return DISK_TYPE_FLOPPY_2M88;

    // Everything else is hard disk
    if (size > 0) return DISK_TYPE_HARD_DISK;

    return DISK_TYPE_NONE;
}

const char *disk_type_name(disk_type_t type)
{
    switch (type) {
        case DISK_TYPE_FLOPPY_360K:  return "360KB Floppy";
        case DISK_TYPE_FLOPPY_720K:  return "720KB Floppy";
        case DISK_TYPE_FLOPPY_1M2:   return "1.2MB Floppy";
        case DISK_TYPE_FLOPPY_1M44:  return "1.44MB Floppy";
        case DISK_TYPE_FLOPPY_2M88:  return "2.88MB Floppy";
        case DISK_TYPE_HARD_DISK:    return "Hard Disk";
        default:                     return "None";
    }
}

void disk_list(char *buffer, size_t size)
{
    if (!buffer || size == 0) return;

    int pos = 0;
    pos += snprintf(buffer + pos, size - pos, "Drive Type        Size      File                         R/W Stats\n");
    pos += snprintf(buffer + pos, size - pos, "===== =========== ========= ============================ ==========\n");

    for (int i = 0; i < MAX_DISK_DRIVES && pos < size - 100; i++) {
        disk_info_t *disk = &s_drives[i];
        if (disk->mounted) {
            char size_str[20];
            if (disk->size_bytes < 1024 * 1024) {
                snprintf(size_str, sizeof(size_str), "%lluKB", disk->size_bytes / 1024);
            } else {
                snprintf(size_str, sizeof(size_str), "%lluMB", disk->size_bytes / (1024 * 1024));
            }

            char drive_letter = 'A' + i;
            pos += snprintf(buffer + pos, size - pos, 
                            "%c:    %-11s %-9s %-28s %"PRIu32"R/%"PRIu32"W\n",
                            drive_letter,
                            disk_type_name(disk->type),
                            size_str,
                            disk->filename,
                            disk->read_count,
                            disk->write_count);
        }
    }

    if (pos < size) {
        buffer[pos] = '\0';
    }
}

void disk_sync_all(void)
{
    for (int i = 0; i < MAX_DISK_DRIVES; i++) {
        if (s_drives[i].mounted && s_drives[i].file) {
            fflush(s_drives[i].file);
        }
    }
}

void disk_deinit(void)
{
    for (int i = 0; i < MAX_DISK_DRIVES; i++) {
        disk_unmount(i);
    }
    ESP_LOGI(TAG, "Disk system shut down");
}
