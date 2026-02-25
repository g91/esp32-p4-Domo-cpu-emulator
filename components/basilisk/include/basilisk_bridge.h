/*
 *  basilisk_bridge.h - C API bridge for BasiliskII ESP32-P4 component
 *
 *  This header provides a C-compatible interface for calling BasiliskII
 *  functions from the main ESP-IDF console application (hello_world_main.c).
 */

#ifndef BASILISK_BRIDGE_H
#define BASILISK_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  Initialize and start the Macintosh emulator.
 *  Creates a FreeRTOS task that loads ROM, allocates RAM,
 *  initializes all subsystems, and starts 68k CPU emulation.
 *
 *  Requires:
 *    - SD card mounted at /sdcard
 *    - /sdcard/mac/Q650.ROM (Quadra 650 ROM, ~1MB)
 *    - /sdcard/mac/Macintosh.dsk (HFS disk image)
 *
 *  Returns 0 on success, -1 on failure.
 */
int basilisk_init_and_run(void);

/*
 *  Stop the Macintosh emulator.
 *  Signals the CPU loop to exit, waits for cleanup, then deletes the task.
 */
void basilisk_stop(void);

/*
 *  Check if the emulator is currently running.
 */
bool basilisk_is_running(void);

/*
 *  Get current emulator performance (Instructions Per Second).
 */
uint32_t basilisk_get_ips(void);

#ifdef __cplusplus
}
#endif

#endif /* BASILISK_BRIDGE_H */
