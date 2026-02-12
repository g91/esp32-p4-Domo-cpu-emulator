/*
 * DOS INT 21h Function Handlers
 * Complete implementation of DOS services for i8086 emulator
 */

#ifndef I8086_DOS_H
#define I8086_DOS_H

#include <stdint.h>
#include <stdbool.h>
#include "i8086_cpu.h"
#include "disk_image.h"

#ifdef __cplusplus
extern "C" {
#endif

// DOS function numbers
#define DOS_TERMINATE               0x00
#define DOS_CHAR_INPUT              0x01
#define DOS_CHAR_OUTPUT             0x02
#define DOS_DIRECT_CONS_IO          0x06
#define DOS_STRING_OUTPUT           0x09
#define DOS_BUFFERED_INPUT          0x0A
#define DOS_GET_STDIN_STATUS        0x0B
#define DOS_FLUSH_BUFFER            0x0C
#define DOS_DISK_RESET              0x0D
#define DOS_SELECT_DISK             0x0E
#define DOS_OPEN_FILE_FCB           0x0F
#define DOS_CLOSE_FILE_FCB          0x10
#define DOS_FIND_FIRST_FCB          0x11
#define DOS_FIND_NEXT_FCB           0x12
#define DOS_DELETE_FILE_FCB         0x13
#define DOS_SEQ_READ_FCB            0x14
#define DOS_SEQ_WRITE_FCB           0x15
#define DOS_CREATE_FILE_FCB         0x16
#define DOS_RENAME_FILE_FCB          0x17
#define DOS_GET_CURRENT_DISK        0x19
#define DOS_SET_DTA_ADDRESS         0x1A
#define DOS_GET_DEFAULT_DRIVE       0x1B
#define DOS_SET_INTERRUPT_VECTOR    0x25
#define DOS_CREATE_PSP              0x26
#define DOS_RANDOM_READ_FCB         0x21
#define DOS_RANDOM_WRITE_FCB        0x22
#define DOS_GET_FILE_SIZE_FCB       0x23
#define DOS_SET_RANDOM_RECORD       0x24
#define DOS_PARSE_FILENAME          0x29
#define DOS_GET_DATE                0x2A
#define DOS_SET_DATE                0x2B
#define DOS_GET_TIME                0x2C
#define DOS_SET_TIME                0x2D
#define DOS_GET_VERIFY_FLAG         0x2E
#define DOS_GET_DTA_ADDRESS         0x2F
#define DOS_GET_DOS_VERSION         0x30
#define DOS_KEEP_PROCESS            0x31
#define DOS_GET_CTRL_BREAK_FLAG     0x33
#define DOS_GET_INDOS_FLAG          0x34
#define DOS_GET_INTERRUPT_VECTOR    0x35
#define DOS_GET_DISK_FREE_SPACE     0x36
#define DOS_GET_COUNTRY_INFO        0x38
#define DOS_MKDIR                   0x39
#define DOS_RMDIR                   0x3A
#define DOS_CHDIR                   0x3B
#define DOS_CREATE_FILE             0x3C
#define DOS_OPEN_FILE               0x3D
#define DOS_CLOSE_FILE              0x3E
#define DOS_READ_FILE               0x3F
#define DOS_WRITE_FILE              0x40
#define DOS_DELETE_FILE             0x41
#define DOS_LSEEK                   0x42
#define DOS_GET_SET_FILE_ATTR       0x43
#define DOS_IOCTL                   0x44
#define DOS_DUP_HANDLE              0x45
#define DOS_FORCE_DUP_HANDLE        0x46
#define DOS_GET_CURRENT_DIR         0x47
#define DOS_ALLOCATE_MEMORY         0x48
#define DOS_FREE_MEMORY             0x49
#define DOS_RESIZE_MEMORY           0x4A
#define DOS_EXEC                    0x4B
#define DOS_EXIT                    0x4C
#define DOS_GET_RETURN_CODE         0x4D
#define DOS_FIND_FIRST              0x4E
#define DOS_FIND_NEXT               0x4F
#define DOS_SET_PSP                 0x50
#define DOS_GET_PSP                 0x51
#define DOS_GET_LIST_OF_LISTS       0x52
#define DOS_RENAME_FILE             0x56
#define DOS_GET_SET_FILE_DATE_TIME  0x57
#define DOS_GET_SET_ALLOC_STRATEGY  0x58
#define DOS_GET_EXTENDED_ERROR      0x59
#define DOS_CREATE_TEMP_FILE        0x5A
#define DOS_CREATE_NEW_FILE         0x5B
#define DOS_LOCK_UNLOCK_FILE        0x5C
#define DOS_NETWORK_FUNCTIONS       0x5E
#define DOS_NETWORK_REDIRECTION     0x5F
#define DOS_GET_QUALIFIED_FILENAME  0x60
#define DOS_GET_ADDRESS_OF_NLS      0x65
#define DOS_GET_SET_CODE_PAGE       0x66
#define DOS_SET_HANDLE_COUNT        0x67
#define DOS_COMMIT_FILE             0x68
#define DOS_EXTENDED_OPEN_CREATE    0x6C

// DOS error codes
#define DOS_ERR_NONE                0x00
#define DOS_ERR_INVALID_FUNCTION    0x01
#define DOS_ERR_FILE_NOT_FOUND      0x02
#define DOS_ERR_PATH_NOT_FOUND      0x03
#define DOS_ERR_TOO_MANY_OPEN_FILES 0x04
#define DOS_ERR_ACCESS_DENIED       0x05
#define DOS_ERR_INVALID_HANDLE      0x06
#define DOS_ERR_MCB_DESTROYED       0x07
#define DOS_ERR_INSUFFICIENT_MEMORY 0x08
#define DOS_ERR_INVALID_MEMORY_BLOCK 0x09
#define DOS_ERR_INVALID_ENVIRONMENT 0x0A
#define DOS_ERR_INVALID_FORMAT      0x0B
#define DOS_ERR_INVALID_ACCESS      0x0C
#define DOS_ERR_INVALID_DATA        0x0D
#define DOS_ERR_INVALID_DRIVE       0x0F
#define DOS_ERR_NO_MORE_FILES       0x12
#define DOS_ERR_WRITE_PROTECT       0x13
#define DOS_ERR_DISK_FULL           0x27

// File handle state
#define MAX_OPEN_FILES 20

typedef struct {
    bool in_use;
    int disk_num;           // Disk number (0-3)
    uint32_t file_offset;   // Current file position
    uint32_t file_size;     // File size
    char filename[260];     // Full path
    uint8_t attributes;     // File attributes
    uint16_t date;          // Last modified date
    uint16_t time;          // Last modified time
    bool is_device;         // True for CON:, PRN:, etc.
} dos_file_handle_t;

// PSP (Program Segment Prefix) structure
typedef struct {
    uint16_t segment;       // Segment address of PSP
    uint16_t memory_top;    // Top of memory for this program
    uint32_t dta_address;   // Current DTA address
    uint16_t parent_psp;    // Parent PSP segment
} dos_psp_t;

/**
 * Initialize DOS environment
 */
void dos_init(void);

/**
 * Handle DOS INT 21h
 * @return true if interrupt was handled
 */
bool dos_int21h_handler(void);

/**
 * Set DOS error code
 */
void dos_set_error(uint8_t error_code);

/**
 * Get DOS error information
 */
void dos_get_extended_error(uint16_t *ax, uint8_t *bh, uint8_t *bl, uint8_t *ch);

#ifdef __cplusplus
}
#endif

#endif // I8086_DOS_H
