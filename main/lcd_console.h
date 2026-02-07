/*
 * LCD Console Driver for ESP32-P4
 * Mirrors serial console output to LCD display
 */

#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// LCD display configuration (4" 480x320 SPI ST7796S)
#define LCD_H_RES           480
#define LCD_V_RES           320
#define LCD_BIT_PER_PIXEL   16

// Console configuration (larger screen = more text)
#define CONSOLE_FONT_WIDTH      8
#define CONSOLE_FONT_HEIGHT     16
#define CONSOLE_COLS            (LCD_H_RES / CONSOLE_FONT_WIDTH)     // 60 columns
#define CONSOLE_ROWS            (LCD_V_RES / CONSOLE_FONT_HEIGHT)    // 20 rows

// Colors (RGB888)
#define CONSOLE_BG_COLOR        0x000000    // Black background
#define CONSOLE_FG_COLOR        0x00FF00    // Green text (classic terminal)
#define CONSOLE_CURSOR_COLOR    0xFFFFFF    // White cursor

/**
 * @brief Initialize the LCD console
 * 
 * Sets up MIPI DSI, LCD panel, and text rendering buffer
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lcd_console_init(void);

/**
 * @brief Deinitialize the LCD console
 */
void lcd_console_deinit(void);

/**
 * @brief Print a string to the LCD console
 * 
 * Supports basic escape sequences for newline, carriage return, etc.
 * 
 * @param str String to print
 */
void lcd_console_print(const char *str);

/**
 * @brief Print a single character to the LCD console
 * 
 * @param c Character to print
 */
void lcd_console_putchar(char c);

/**
 * @brief Clear the LCD console screen
 */
void lcd_console_clear(void);

/**
 * @brief Set cursor position
 * 
 * @param col Column (0-based)
 * @param row Row (0-based)
 */
void lcd_console_set_cursor(int col, int row);

/**
 * @brief Get current cursor position
 * 
 * @param col Pointer to store column
 * @param row Pointer to store row
 */
void lcd_console_get_cursor(int *col, int *row);

/**
 * @brief Scroll the console up by one line
 */
void lcd_console_scroll_up(void);

/**
 * @brief Set console colors
 * 
 * @param fg_color Foreground color (RGB888)
 * @param bg_color Background color (RGB888)
 */
void lcd_console_set_colors(uint32_t fg_color, uint32_t bg_color);

/**
 * @brief Enable/disable cursor blinking
 * 
 * @param enable true to enable cursor
 */
void lcd_console_cursor_enable(bool enable);

/**
 * @brief Force a display refresh
 * 
 * Normally the console auto-refreshes, but this can be called
 * to force an immediate update
 */
void lcd_console_refresh(void);

/**
 * @brief Check if LCD console is initialized
 * 
 * @return true if initialized
 */
bool lcd_console_is_initialized(void);

/**
 * @brief Get LCD pin configuration for diagnostic purposes
 * 
 * @param rst_pin Pointer to store reset pin number
 * @param bl_pin Pointer to store backlight pin number  
 * @param mosi_pin Pointer to store MOSI pin number
 * @param clk_pin Pointer to store clock pin number
 */
void lcd_console_get_pins(int *rst_pin, int *bl_pin, int *mosi_pin, int *clk_pin);

/**
 * @brief Draw raw RGB565 pixel data to the LCD
 * 
 * Used by VGA emulation to output framebuffer directly to LCD.
 * Data must be in DMA-capable memory or will be copied internally.
 * 
 * @param x_start Start X coordinate
 * @param y_start Start Y coordinate
 * @param x_end End X coordinate (exclusive)
 * @param y_end End Y coordinate (exclusive)
 * @param data RGB565 pixel data
 */
void lcd_console_draw_raw(int x_start, int y_start, int x_end, int y_end,
                          const void *data);

#ifdef __cplusplus
}
#endif
