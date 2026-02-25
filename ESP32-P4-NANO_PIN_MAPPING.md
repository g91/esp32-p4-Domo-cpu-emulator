# ESP32-P4-NANO Pin Mapping

## Board Comparison

| Feature | ESP32-P4-WIFI6 | ESP32-P4-NANO | Notes |
|---------|----------------|---------------|-------|
| **WiFi** | ESP32-C6 on SDIO Slot 1 | None | NANO requires external WiFi |
| **SDMMC** | Slot 0 (SD card) | Slot 0 (SD card only) | Same pins, no SDIO conflict |

## Pin Assignments for ESP32-P4-NANO

### SPI LCD Display (ST7796S 480x320)
```
MOSI     = GPIO27 (USBIP1_P1)
CLK      = GPIO24 (USBIP1_N0)
CS       = GPIO22
DC/RS    = GPIO4  (TOUCH_CHANNEL2)
RST      = GPIO38
Backlight= GPIO48 (PWM)
```

### MIPI DSI Display (7" 800x480)
```
DSI Lanes  = Built-in ESP32-P4 DSI pins (hardwired)
Backlight  = GPIO48 (PWM)
LDO Power  = Channel 3, 2.5V (auto-configured)
```

### SD Card (SDMMC Slot 0, 4-bit mode)
```
CLK  = GPIO43
CMD  = GPIO44
D0   = GPIO39
D1   = GPIO40
D2   = GPIO41
D3   = GPIO42
LDO  = Channel 4 (required for ESP32-P4)
```

### PS/2 Keyboard
```
CLK  = GPIO5  (TOUCH_CHANNEL3)
DATA = GPIO23 (ADC1_CHANNEL7)
```

### I2C Bus (Touch, Audio Codec, etc.)
```
SDA = GPIO7 (TOUCH_CHANNEL5)
SCL = GPIO8 (TOUCH_CHANNEL6)
```

### I2S Audio (if codec available)
```
MCLK = GPIO33
SCLK = GPIO32
DOUT = GPIO25 (USBIP1_P0)
LRCK = GPIO26 (USBIP1_N1)
DIN  = GPIO36
PA_CTRL = GPIO53 (ADC2_CHANNEL6)
```

## Key Differences from WIFI6 Board

1. **No WiFi Companion Chip**: NANO doesn't have ESP32-C6, so SDIO Slot 1 is unused
2. **Different GPIO Range**: NANO uses GPIOs 4-48 primarily (smaller form factor)
3. **Backlight Changed**: GPIO48 instead of GPIO46 (WIFI6) or GPIO26 (previous DSI)
4. **PS/2 Pins Changed**: GPIO5/23 instead of GPIO28/29
5. **SPI LCD Pins**: Completely different pinout from WIFI6 board

## Important Notes

- **LDO Power Control**: ESP32-P4 requires LDO initialization for SD card (channel 4) and DSI display (channel 3)
- **WiFi Support**: Requires external module via ESP-HOSTED framework or SPI-based WiFi module
- **Touch Controller**: Uses I2C bus on GPIO7 (SDA) and GPIO8 (SCL)
- **Display Selection**: Choose SPI or DSI by editing `main/CMakeLists.txt` (link `lcd_console.c` OR `lcd_console_dsi.c`)

## Available GPIOs on NANO (from pinout diagram)

- Left header: GPIO7, GPIO8, GPIO23, GPIO5, GPIO20, GPIO21, GPIO25, GPIO26, GPIO32
- Right header: GPIO0, GPIO1, GPIO3, GPIO2, GPIO6, GPIO53, GPIO48, GPIO45, GPIO46, GPIO47, GPIO54, GPIO22, GPIO24, GPIO27, GPIO4, GPIO38, GPIO37, GPIO33, GPIO36
- UART: GPIO37 (TXD), GPIO38 (RXD)
- USB: GPIO24-27 (USBIP1 pins)
- ADC: Multiple ADC channels available
- Touch: GPIO5, GPIO7, GPIO8, GPIO20, GPIO21, GPIO23 (TOUCH_CHANNEL labels)

## Build Configuration

No changes needed to `sdkconfig` - pin assignments are in source code:
- [main/lcd_console.c](main/lcd_console.c) - SPI LCD pins
- [main/lcd_console_dsi.c](main/lcd_console_dsi.c) - DSI LCD backlight
- [main/ps2_keyboard.h](main/ps2_keyboard.h) - PS/2 pins
- [main/hello_world_main.c](main/hello_world_main.c) - SD card, I2C, I2S pins
