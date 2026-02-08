# M68K Software Development Kit (SDK) v1.0

A complete SDK for writing programs that run on the M68K (Motorola 68000) emulator hosted on the ESP32-P4. Provides clean C APIs for all bus controller devices: console, filesystem, networking, FPU, audio, video, DMA, timers, and interrupts.

## Quick Start

### Prerequisites
- **m68k-linux-gnu-gcc** cross-compiler (or m68k-elf-gcc)
  - Linux: `sudo apt install gcc-m68k-linux-gnu`
  - Windows: Download from [gcc-m68k-build releases](https://github.com/vintagepc/gcc-m68k-build/releases)
- Set `CROSS` env var if your prefix differs: `export CROSS=m68k-elf-`

### Build the SDK
```bash
# Linux/macOS
cd m68k_apps/sdk
./build.sh

# Windows
cd m68k_apps\sdk
build.cmd
```

### Build an Application
```bash
# Linux/macOS
./build.sh examples/hello_sdk.c

# Windows
build.cmd examples\hello_sdk.c
```

### Run on ESP32-P4
Copy `build/hello_sdk.bin` to the SD card, then in the serial console:
```
loadrun /sdcard/hello_sdk.bin
```

Or load and run separately:
```
load /sdcard/hello_sdk.bin
run
```

## Project Structure

```
sdk/
├── include/              Headers
│   ├── m68k_sdk.h        Master include (use this!)
│   ├── m68k_types.h      Fixed-width types, macros
│   ├── m68k_io.h         Bus register addresses & constants
│   ├── m68k_string.h     String/memory utilities
│   ├── m68k_console.h    Console I/O
│   ├── m68k_fs.h         Filesystem access
│   ├── m68k_net.h        TCP/UDP networking
│   ├── m68k_fpu.h        FPU math operations
│   ├── m68k_audio.h      SB16-style audio
│   ├── m68k_video.h      Framebuffer video/GPU
│   └── m68k_system.h     DMA, Timer, IRQ, Heap
├── src/                  Implementations
│   ├── m68k_string.c     String funcs + software math (__divsi3 etc.)
│   ├── m68k_console.c    Console I/O implementation
│   ├── m68k_fs.c         Filesystem implementation
│   ├── m68k_net.c        Network socket implementation
│   ├── m68k_fpu.c        FPU bridge implementation
│   ├── m68k_audio.c      Audio device implementation
│   ├── m68k_video.c      Video/GPU implementation
│   └── m68k_system.c     DMA/Timer/IRQ/Heap implementation
├── examples/             Example programs
│   ├── hello_sdk.c       Multi-feature demo
│   ├── http_get.c        HTTP client demo
│   └── gfx_demo.c        Graphics demo
├── crt0.S                C runtime startup assembly
├── program.ld            Linker script (load @ 0x100000)
├── Makefile              GNU Make build system
├── build.sh              Linux/macOS build script
├── build.cmd             Windows build script
└── README.md             This file
```

## API Reference

### Include Everything
```c
#include "m68k_sdk.h"     /* Pulls in all SDK headers */
```

---

### Console I/O (`m68k_console.h`)

Output text to UART and LCD simultaneously.

| Function | Description |
|----------|-------------|
| `con_putchar(c)` | Write a single character |
| `con_print(str)` | Write a string |
| `con_println(str)` | Write a string + newline |
| `con_printf(fmt, ...)` | Formatted output (%s, %d, %u, %x, %c) |
| `con_print_dec(val)` | Print signed integer |
| `con_print_hex(val)` | Print hex with 0x prefix |
| `con_getchar()` | Read one character (blocking) |
| `con_key_available()` | Check if key is ready (non-blocking) |
| `con_getline(buf, size)` | Read a line with editing |
| `con_clear()` | Clear screen |
| `con_gotoxy(x, y)` | Move cursor |
| `con_set_color(fg, bg)` | Set ANSI colors |
| `con_reset_color()` | Reset to defaults |

**Color constants:** `CON_COLOR_BLACK`, `CON_COLOR_RED`, `CON_COLOR_GREEN`, `CON_COLOR_YELLOW`, `CON_COLOR_BLUE`, `CON_COLOR_MAGENTA`, `CON_COLOR_CYAN`, `CON_COLOR_WHITE`

---

### Filesystem (`m68k_fs.h`)

Access files on the ESP32-P4's SD card.

| Function | Description |
|----------|-------------|
| `fs_open(path, mode)` | Open file (returns handle) |
| `fs_close(handle)` | Close file |
| `fs_read(handle, buf, size)` | Read up to 512 bytes |
| `fs_write(handle, buf, size)` | Write data |
| `fs_read_file(path, buf, max)` | Read entire file |
| `fs_write_file(path, data, len)` | Write entire file |
| `fs_listdir(path, buf, size)` | List directory entries |
| `fs_mkdir(path)` | Create directory |
| `fs_remove(path)` | Delete file |
| `fs_filesize(path)` | Get file size in bytes |

**Modes:** `FS_MODE_READ`, `FS_MODE_WRITE`, `FS_MODE_APPEND`

---

### Networking (`m68k_net.h`)

BSD-style TCP/UDP sockets via the bus controller.

| Function | Description |
|----------|-------------|
| `net_socket(domain, type, proto)` | Create socket |
| `net_connect(sock, ip, port)` | Connect to server |
| `net_bind(sock, ip, port)` | Bind to local address |
| `net_listen(sock, backlog)` | Listen for connections |
| `net_accept(sock)` | Accept incoming connection |
| `net_send(sock, data, len)` | Send data (TCP) |
| `net_recv(sock, buf, len)` | Receive data (TCP) |
| `net_sendto(sock, data, len, ip, port)` | Send datagram (UDP) |
| `net_recvfrom(sock, buf, len)` | Receive datagram (UDP) |
| `net_close(sock)` | Close socket |
| `net_ping(ip)` | Ping an IP address |

**Helper:** `NET_IP(a,b,c,d)` — pack IP address into uint32_t

---

### FPU Math (`m68k_fpu.h`)

64-bit floating-point math via the bus controller's FPU device.

| Function | Description |
|----------|-------------|
| `fpu_from_int(n)` | Convert integer to fpu_double_t |
| `fpu_to_int(d)` | Convert to integer (truncate) |
| `fpu_add(a, b)` | Addition |
| `fpu_sub(a, b)` | Subtraction |
| `fpu_mul(a, b)` | Multiplication |
| `fpu_div(a, b)` | Division |
| `fpu_sqrt(a)` | Square root |
| `fpu_sin(a)` | Sine (radians) |
| `fpu_cos(a)` | Cosine (radians) |
| `fpu_pow(a, b)` | Power (a^b) |
| `fpu_abs(a)` / `fpu_neg(a)` | Absolute value / negate |
| `fpu_floor(a)` / `fpu_ceil(a)` | Floor / ceiling |
| `fpu_cmp(a, b)` | Compare (-1, 0, 1) |
| `fpu_pi()` / `fpu_e()` | Mathematical constants |
| `fpu_print(label, val)` | Print to console |

All math is done on the ESP32-P4 RISC-V host via memory-mapped I/O — the M68K side just reads/writes registers.

---

### Audio (`m68k_audio.h`)

SB16-style audio with tone generation and musical note support.

| Function | Description |
|----------|-------------|
| `audio_init()` | Initialize audio subsystem |
| `audio_beep(freq, duration_ms)` | Play a tone |
| `audio_tone(freq)` | Start continuous tone |
| `audio_set_volume(vol)` | Set volume (0-255) |
| `audio_play()` / `audio_stop()` | Play/stop |
| `audio_pause()` / `audio_resume()` | Pause/resume |
| `audio_play_note(note, octave, ms)` | Play musical note |
| `audio_note_freq(note, octave)` | Get frequency for note |

**Musical note constants:** `NOTE_C` through `NOTE_B` (0-11)

---

### Video / GPU (`m68k_video.h`)

Framebuffer video with 2D drawing primitives.

| Function | Description |
|----------|-------------|
| `video_init(mode)` | Initialize display mode |
| `video_clear(color)` | Clear framebuffer |
| `video_flip()` | Swap/display buffer |
| `video_pixel(x, y, color)` | Set pixel |
| `video_line(x1,y1, x2,y2, color)` | Draw line |
| `video_rect(x1,y1, x2,y2, color)` | Draw rectangle outline |
| `video_fill_rect(x1,y1, x2,y2, c)` | Filled rectangle |
| `video_circle(cx, cy, r, color)` | Circle outline |
| `video_fill_circle(cx, cy, r, c)` | Filled circle |
| `video_hline(x1, x2, y, color)` | Horizontal line |
| `video_vline(x, y1, y2, color)` | Vertical line |
| `video_scroll(dx, dy)` | Scroll framebuffer |
| `video_draw_char(c)` | Draw character at cursor |
| `video_set_cursor(x, y)` | Set text cursor |
| `video_set_colors(fg, bg)` | Set text colors |
| `video_blit(sx,sy, dx,dy, w,h)` | Block copy |
| `video_wait_vblank()` | Wait for vertical blank |

**Color helper:** `RGB565(r, g, b)` — pack 8-bit RGB into 16-bit color  
**Predefined:** `COLOR_BLACK`, `COLOR_WHITE`, `COLOR_RED`, `COLOR_GREEN`, `COLOR_BLUE`, `COLOR_CYAN`, `COLOR_MAGENTA`, `COLOR_YELLOW`

---

### System (`m68k_system.h`)

DMA, timers, interrupts, and memory management.

| Function | Description |
|----------|-------------|
| `dma_m68k_to_arm(src, dst, len)` | DMA from M68K to ARM |
| `dma_arm_to_m68k(src, dst, len)` | DMA from ARM to M68K |
| `dma_is_busy()` | Check DMA status |
| `timer_ticks()` | Read system timer |
| `delay_ms(ms)` | Busy-wait delay |
| `irq_enable(mask)` | Enable interrupts |
| `irq_disable(mask)` | Disable interrupts |
| `irq_pending()` | Check pending interrupts |
| `irq_ack(mask)` | Acknowledge interrupts |
| `heap_init(start, end)` | Initialize allocator |
| `heap_alloc(size)` | Allocate memory |
| `heap_free(ptr)` | Free memory (no-op in bump) |
| `heap_free_space()` | Bytes remaining |
| `heap_total_space()` | Total heap size |

---

### String Utilities (`m68k_string.h`)

Standard C string/memory functions for freestanding environment.

**Memory:** `m68k_memset`, `m68k_memcpy`, `m68k_memmove`, `m68k_memcmp`  
**String:** `m68k_strlen`, `m68k_strcpy`, `m68k_strncpy`, `m68k_strcmp`, `m68k_strncmp`, `m68k_strcat`, `m68k_strncat`, `m68k_strchr`, `m68k_strrchr`, `m68k_strstr`  
**Char:** `m68k_isspace`, `m68k_isdigit`, `m68k_isalpha`, `m68k_toupper`, `m68k_tolower`, etc.  
**Convert:** `m68k_atoi`, `m68k_strtoul`  
**Format:** `m68k_snprintf` (supports %s, %d, %u, %x, %X, %c, %p, %%)

Also provides GCC intrinsics `__mulsi3`, `__divsi3`, `__modsi3`, `__udivsi3`, `__umodsi3` for M68000 software multiply/divide.

## Memory Map

```
0x00000000 - 0x000FFFFF : Reserved (OS / vectors)
0x00100000 - 0x007FFFFF : Program code + data + heap (7MB)
0x00800000               : Stack (grows down)
0x00F00000 - 0x00F08FFF : Bus controller I/O devices
  +0x00000 : Network
  +0x01000 : Filesystem
  +0x02000 : Console
  +0x03000 : DMA
  +0x04000 : IRQ Controller
  +0x05000 : Timer
  +0x06000 : FPU
  +0x07000 : Audio (SB16)
  +0x08000 : Video / GPU
```

## Writing Your Own Program

1. Create a `.c` file with `#include "m68k_sdk.h"` and a `main()` function
2. Build: `./build.sh your_program.c` (or `build.cmd your_program.c`)
3. Copy `build/your_program.bin` to the SD card
4. Run: `loadrun /sdcard/your_program.bin`

```c
#include "m68k_sdk.h"

int main(void) {
    con_println("Hello from M68K!");
    
    /* Play a tone */
    audio_init();
    audio_beep(440, 500);
    
    /* Read the timer */
    con_printf("Ticks: %u\n", timer_ticks());
    
    return 0;
}
```

## License

Part of the ESP32-P4 Dual CPU Emulator project.
