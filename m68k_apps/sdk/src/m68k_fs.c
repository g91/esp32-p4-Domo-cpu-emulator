/*
 * M68K SDK - Filesystem Implementation
 */

#include "m68k_fs.h"
#include "m68k_io.h"
#include "m68k_string.h"

/* Internal: write a path string to the FS filename register */
static void fs_set_path(const char *path) {
    volatile char *dst = (volatile char *)FS_FILENAME;
    while (*path)
        *dst++ = *path++;
    *dst = 0;
}

/* Internal: wait for FS device to complete command */
static uint32_t fs_wait(void) {
    while (IO_READ32(FS_STATUS) == FS_STATUS_BUSY) { /* spin */ }
    return IO_READ32(FS_STATUS);
}

/* Internal: copy bytes from FS data buffer to user buffer */
static void fs_copy_from_buf(void *dst, uint32_t len) {
    uint8_t *d = (uint8_t *)dst;
    for (uint32_t i = 0; i < len; i++)
        d[i] = IO_READ8(FS_BUFFER + i);
}

/* Internal: copy bytes from user buffer to FS data buffer */
static void fs_copy_to_buf(const void *src, uint32_t len) {
    const uint8_t *s = (const uint8_t *)src;
    for (uint32_t i = 0; i < len; i++)
        IO_WRITE8(FS_BUFFER + i, s[i]);
}

int fs_open(const char *path) {
    fs_set_path(path);
    IO_WRITE32(FS_COMMAND, FS_CMD_OPEN);
    return (fs_wait() == FS_STATUS_OK) ? 0 : -1;
}

void fs_close(void) {
    IO_WRITE32(FS_COMMAND, FS_CMD_CLOSE);
    fs_wait();
}

int fs_read(void *buf, uint32_t size) {
    if (size > FS_BUF_SIZE)
        size = FS_BUF_SIZE;

    IO_WRITE32(FS_SIZE, size);
    IO_WRITE32(FS_COMMAND, FS_CMD_READ);

    if (fs_wait() != FS_STATUS_OK)
        return -1;

    uint32_t got = IO_READ32(FS_SIZE);
    fs_copy_from_buf(buf, got);
    return (int)got;
}

int fs_write(const void *buf, uint32_t size) {
    if (size > FS_BUF_SIZE)
        size = FS_BUF_SIZE;

    fs_copy_to_buf(buf, size);
    IO_WRITE32(FS_SIZE, size);
    IO_WRITE32(FS_COMMAND, FS_CMD_WRITE);

    return (fs_wait() == FS_STATUS_OK) ? (int)size : -1;
}

int32_t fs_read_file(const char *path, void *buf, uint32_t max_size) {
    if (fs_open(path) != 0)
        return -1;

    uint8_t *dst = (uint8_t *)buf;
    uint32_t total = 0;
    uint8_t chunk[FS_BUF_SIZE];

    while (total < max_size) {
        uint32_t want = max_size - total;
        if (want > FS_BUF_SIZE)
            want = FS_BUF_SIZE;

        int got = fs_read(chunk, want);
        if (got <= 0)
            break;

        m68k_memcpy(dst + total, chunk, got);
        total += got;
    }

    fs_close();
    return (int32_t)total;
}

int32_t fs_write_file(const char *path, const void *buf, uint32_t size) {
    /* Need to set up for write mode - open the path first */
    fs_set_path(path);
    IO_WRITE32(FS_COMMAND, FS_CMD_OPEN);
    if (fs_wait() != FS_STATUS_OK)
        return -1;

    const uint8_t *src = (const uint8_t *)buf;
    uint32_t total = 0;

    while (total < size) {
        uint32_t chunk_size = size - total;
        if (chunk_size > FS_BUF_SIZE)
            chunk_size = FS_BUF_SIZE;

        int wrote = fs_write(src + total, chunk_size);
        if (wrote <= 0)
            break;

        total += wrote;
    }

    fs_close();
    return (int32_t)total;
}

int fs_listdir(const char *path, char *buf, uint32_t buf_size) {
    fs_set_path(path);
    IO_WRITE32(FS_COMMAND, FS_CMD_READDIR);

    if (fs_wait() != FS_STATUS_OK)
        return -1;

    /* Copy directory listing from FS data buffer */
    uint32_t len = IO_READ32(FS_SIZE);
    if (len > buf_size - 1)
        len = buf_size - 1;

    fs_copy_from_buf(buf, len);
    buf[len] = '\0';
    return 0;
}

int fs_mkdir(const char *path) {
    fs_set_path(path);
    IO_WRITE32(FS_COMMAND, FS_CMD_MKDIR);
    return (fs_wait() == FS_STATUS_OK) ? 0 : -1;
}

int fs_remove(const char *path) {
    fs_set_path(path);
    IO_WRITE32(FS_COMMAND, FS_CMD_REMOVE);
    return (fs_wait() == FS_STATUS_OK) ? 0 : -1;
}

uint32_t fs_filesize(void) {
    return IO_READ32(FS_SIZE);
}
