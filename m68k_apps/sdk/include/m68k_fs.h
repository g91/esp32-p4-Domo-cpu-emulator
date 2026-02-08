/*
 * M68K SDK - Filesystem API
 * =========================
 * File and directory operations via the bus controller filesystem device.
 * The host ESP32-P4 maps these to the SD card via FATFS.
 *
 * Note: Only one file can be open at a time through this API.
 * The filesystem device uses a 512-byte buffer for transfers.
 */

#ifndef M68K_FS_H
#define M68K_FS_H

#include "m68k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Open a file by path. Returns 0 on success, -1 on error.
 * Path is relative to /sdcard on the ESP32 (e.g. "/programs/test.bin").
 * Only one file can be open at a time. */
int fs_open(const char *path);

/* Close the currently open file. */
void fs_close(void);

/* Read up to 'size' bytes from the open file into 'buf'.
 * Returns number of bytes read, or -1 on error.
 * Maximum single read is 512 bytes (FS_BUF_SIZE). For larger reads, call
 * repeatedly. Returns 0 at end-of-file. */
int fs_read(void *buf, uint32_t size);

/* Write up to 'size' bytes from 'buf' to the open file.
 * Returns number of bytes written, or -1 on error.
 * Maximum single write is 512 bytes. */
int fs_write(const void *buf, uint32_t size);

/* Read an entire file into memory. Opens, reads in 512-byte chunks, closes.
 * Returns total bytes read, or -1 on error. */
int32_t fs_read_file(const char *path, void *buf, uint32_t max_size);

/* Write an entire buffer to a file. Opens, writes in 512-byte chunks, closes.
 * Returns total bytes written, or -1 on error. */
int32_t fs_write_file(const char *path, const void *buf, uint32_t size);

/* List directory contents. Entries are written to 'buf' as newline-separated
 * names. Returns 0 on success, -1 on error. */
int fs_listdir(const char *path, char *buf, uint32_t buf_size);

/* Create a directory. Returns 0 on success, -1 on error. */
int fs_mkdir(const char *path);

/* Remove a file or empty directory. Returns 0 on success, -1 on error. */
int fs_remove(const char *path);

/* Get file size of the currently open file. Returns size in bytes. */
uint32_t fs_filesize(void);

#ifdef __cplusplus
}
#endif

#endif /* M68K_FS_H */
