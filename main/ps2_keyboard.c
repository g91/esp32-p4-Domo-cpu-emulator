/*
 * PS/2 Keyboard Driver for ESP32-P4
 * Handles PS/2 keyboard input via GPIO bit-banging
 */

#include "ps2_keyboard.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"  // For ets_printf (ISR-safe printing)

static const char *TAG = "ps2_kbd";

// PS/2 protocol state machine
typedef enum {
    PS2_STATE_IDLE,
    PS2_STATE_DATA,
    PS2_STATE_PARITY,
    PS2_STATE_STOP
} ps2_state_t;

// Keyboard driver state
typedef struct {
    // GPIO pins
    gpio_num_t clk_pin;
    gpio_num_t data_pin;
    
    // Receive state machine
    ps2_state_t state;
    uint8_t bit_count;
    uint8_t data_byte;
    uint8_t parity;
    
    // Scancode processing
    bool extended;      // E0 prefix received
    bool release;       // F0 (break) prefix received
    
    // Modifier key states
    bool shift_left;
    bool shift_right;
    bool ctrl_left;
    bool ctrl_right;
    bool alt_left;
    bool alt_right;
    bool caps_lock;
    bool num_lock;
    
    // Character buffer
    uint8_t buffer[PS2_BUFFER_SIZE];
    volatile int buf_head;
    volatile int buf_tail;
    
    // Synchronization
    SemaphoreHandle_t data_ready;
    SemaphoreHandle_t mutex;
    
    // Callback
    ps2_key_callback_t callback;
    
    // Flags
    bool initialized;
    
    // Last clock time for timeout detection
    int64_t last_clock_time;
} ps2_keyboard_t;

static ps2_keyboard_t *kbd = NULL;

// Forward declarations
static void process_scancode(uint8_t scancode);
static void buffer_put(uint8_t c);
static void IRAM_ATTR gpio_isr_handler(void *arg);

// PS/2 Set 2 scancode to ASCII lookup table (unshifted)
static const uint8_t scancode_to_ascii[] = {
    [0x00] = 0,     [0x01] = PS2_KEY_F9,  [0x02] = 0,     [0x03] = PS2_KEY_F5,
    [0x04] = PS2_KEY_F3, [0x05] = PS2_KEY_F1, [0x06] = PS2_KEY_F2, [0x07] = PS2_KEY_F12,
    [0x08] = 0,     [0x09] = PS2_KEY_F10, [0x0A] = PS2_KEY_F8, [0x0B] = PS2_KEY_F6,
    [0x0C] = PS2_KEY_F4, [0x0D] = '\t',  [0x0E] = '`',   [0x0F] = 0,
    [0x10] = 0,     [0x11] = 0,     [0x12] = 0,     [0x13] = 0,     // Alt
    [0x14] = 0,     [0x15] = 'q',   [0x16] = '1',   [0x17] = 0,     // Ctrl
    [0x18] = 0,     [0x19] = 0,     [0x1A] = 'z',   [0x1B] = 's',
    [0x1C] = 'a',   [0x1D] = 'w',   [0x1E] = '2',   [0x1F] = 0,
    [0x20] = 0,     [0x21] = 'c',   [0x22] = 'x',   [0x23] = 'd',
    [0x24] = 'e',   [0x25] = '4',   [0x26] = '3',   [0x27] = 0,
    [0x28] = 0,     [0x29] = ' ',   [0x2A] = 'v',   [0x2B] = 'f',
    [0x2C] = 't',   [0x2D] = 'r',   [0x2E] = '5',   [0x2F] = 0,
    [0x30] = 0,     [0x31] = 'n',   [0x32] = 'b',   [0x33] = 'h',
    [0x34] = 'g',   [0x35] = 'y',   [0x36] = '6',   [0x37] = 0,
    [0x38] = 0,     [0x39] = 0,     [0x3A] = 'm',   [0x3B] = 'j',
    [0x3C] = 'u',   [0x3D] = '7',   [0x3E] = '8',   [0x3F] = 0,
    [0x40] = 0,     [0x41] = ',',   [0x42] = 'k',   [0x43] = 'i',
    [0x44] = 'o',   [0x45] = '0',   [0x46] = '9',   [0x47] = 0,
    [0x48] = 0,     [0x49] = '.',   [0x4A] = '/',   [0x4B] = 'l',
    [0x4C] = ';',   [0x4D] = 'p',   [0x4E] = '-',   [0x4F] = 0,
    [0x50] = 0,     [0x51] = 0,     [0x52] = '\'',  [0x53] = 0,
    [0x54] = '[',   [0x55] = '=',   [0x56] = 0,     [0x57] = 0,
    [0x58] = 0,     [0x59] = 0,     [0x5A] = '\r',  [0x5B] = ']',   // Enter, Caps Lock
    [0x5C] = 0,     [0x5D] = '\\',  [0x5E] = 0,     [0x5F] = 0,
    [0x60] = 0,     [0x61] = 0,     [0x62] = 0,     [0x63] = 0,
    [0x64] = 0,     [0x65] = 0,     [0x66] = PS2_KEY_BACKSPACE, [0x67] = 0,
    [0x68] = 0,     [0x69] = '1',   [0x6A] = 0,     [0x6B] = '4',   // Numpad
    [0x6C] = '7',   [0x6D] = 0,     [0x6E] = 0,     [0x6F] = 0,
    [0x70] = '0',   [0x71] = '.',   [0x72] = '2',   [0x73] = '5',
    [0x74] = '6',   [0x75] = '8',   [0x76] = PS2_KEY_ESCAPE, [0x77] = 0,   // Num Lock
    [0x78] = PS2_KEY_F11, [0x79] = '+', [0x7A] = '3',  [0x7B] = '-',
    [0x7C] = '*',   [0x7D] = '9',   [0x7E] = 0,     [0x7F] = 0,
};

// Shifted characters
static const uint8_t scancode_to_ascii_shifted[] = {
    [0x00] = 0,     [0x01] = PS2_KEY_F9,  [0x02] = 0,     [0x03] = PS2_KEY_F5,
    [0x04] = PS2_KEY_F3, [0x05] = PS2_KEY_F1, [0x06] = PS2_KEY_F2, [0x07] = PS2_KEY_F12,
    [0x08] = 0,     [0x09] = PS2_KEY_F10, [0x0A] = PS2_KEY_F8, [0x0B] = PS2_KEY_F6,
    [0x0C] = PS2_KEY_F4, [0x0D] = '\t',  [0x0E] = '~',   [0x0F] = 0,
    [0x10] = 0,     [0x11] = 0,     [0x12] = 0,     [0x13] = 0,
    [0x14] = 0,     [0x15] = 'Q',   [0x16] = '!',   [0x17] = 0,
    [0x18] = 0,     [0x19] = 0,     [0x1A] = 'Z',   [0x1B] = 'S',
    [0x1C] = 'A',   [0x1D] = 'W',   [0x1E] = '@',   [0x1F] = 0,
    [0x20] = 0,     [0x21] = 'C',   [0x22] = 'X',   [0x23] = 'D',
    [0x24] = 'E',   [0x25] = '$',   [0x26] = '#',   [0x27] = 0,
    [0x28] = 0,     [0x29] = ' ',   [0x2A] = 'V',   [0x2B] = 'F',
    [0x2C] = 'T',   [0x2D] = 'R',   [0x2E] = '%',   [0x2F] = 0,
    [0x30] = 0,     [0x31] = 'N',   [0x32] = 'B',   [0x33] = 'H',
    [0x34] = 'G',   [0x35] = 'Y',   [0x36] = '^',   [0x37] = 0,
    [0x38] = 0,     [0x39] = 0,     [0x3A] = 'M',   [0x3B] = 'J',
    [0x3C] = 'U',   [0x3D] = '&',   [0x3E] = '*',   [0x3F] = 0,
    [0x40] = 0,     [0x41] = '<',   [0x42] = 'K',   [0x43] = 'I',
    [0x44] = 'O',   [0x45] = ')',   [0x46] = '(',   [0x47] = 0,
    [0x48] = 0,     [0x49] = '>',   [0x4A] = '?',   [0x4B] = 'L',
    [0x4C] = ':',   [0x4D] = 'P',   [0x4E] = '_',   [0x4F] = 0,
    [0x50] = 0,     [0x51] = 0,     [0x52] = '"',   [0x53] = 0,
    [0x54] = '{',   [0x55] = '+',   [0x56] = 0,     [0x57] = 0,
    [0x58] = 0,     [0x59] = 0,     [0x5A] = '\r',  [0x5B] = '}',
    [0x5C] = 0,     [0x5D] = '|',   [0x5E] = 0,     [0x5F] = 0,
    [0x60] = 0,     [0x61] = 0,     [0x62] = 0,     [0x63] = 0,
    [0x64] = 0,     [0x65] = 0,     [0x66] = PS2_KEY_BACKSPACE, [0x67] = 0,
    [0x68] = 0,     [0x69] = '1',   [0x6A] = 0,     [0x6B] = '4',
    [0x6C] = '7',   [0x6D] = 0,     [0x6E] = 0,     [0x6F] = 0,
    [0x70] = '0',   [0x71] = '.',   [0x72] = '2',   [0x73] = '5',
    [0x74] = '6',   [0x75] = '8',   [0x76] = PS2_KEY_ESCAPE, [0x77] = 0,
    [0x78] = PS2_KEY_F11, [0x79] = '+', [0x7A] = '3',  [0x7B] = '-',
    [0x7C] = '*',   [0x7D] = '9',   [0x7E] = 0,     [0x7F] = 0,
};

// Extended scancodes (E0 prefix)
static uint8_t process_extended_scancode(uint8_t scancode) {
    switch (scancode) {
        case 0x75: return PS2_KEY_UP;
        case 0x72: return PS2_KEY_DOWN;
        case 0x6B: return PS2_KEY_LEFT;
        case 0x74: return PS2_KEY_RIGHT;
        case 0x6C: return PS2_KEY_HOME;
        case 0x69: return PS2_KEY_END;
        case 0x7D: return PS2_KEY_PAGEUP;
        case 0x7A: return PS2_KEY_PAGEDOWN;
        case 0x70: return PS2_KEY_INSERT;
        case 0x71: return PS2_KEY_DELETE;
        case 0x5A: return '\r';  // Numpad Enter
        case 0x4A: return '/';   // Numpad /
        default: return 0;
    }
}

// Add character to buffer
static void buffer_put(uint8_t c) {
    if (kbd == NULL) return;
    
    int next_head = (kbd->buf_head + 1) % PS2_BUFFER_SIZE;
    if (next_head != kbd->buf_tail) {  // Not full
        kbd->buffer[kbd->buf_head] = c;
        kbd->buf_head = next_head;
        xSemaphoreGive(kbd->data_ready);
    }
}

// Process a received scancode
static void process_scancode(uint8_t scancode) {
    if (kbd == NULL) return;
    
    // Handle prefix bytes
    if (scancode == 0xE0) {
        kbd->extended = true;
        return;
    }
    
    if (scancode == 0xF0) {
        kbd->release = true;
        return;
    }
    
    // Ignore certain codes
    if (scancode == 0xAA || scancode == 0xFA || scancode == 0xFE) {
        // Self-test passed, ACK, Resend - ignore
        kbd->extended = false;
        kbd->release = false;
        return;
    }
    
    bool is_release = kbd->release;
    bool is_extended = kbd->extended;
    kbd->release = false;
    kbd->extended = false;
    
    // Handle modifier keys
    if (!is_extended) {
        switch (scancode) {
            case 0x12:  // Left Shift
                kbd->shift_left = !is_release;
                return;
            case 0x59:  // Right Shift
                kbd->shift_right = !is_release;
                return;
            case 0x14:  // Left Ctrl
                kbd->ctrl_left = !is_release;
                return;
            case 0x11:  // Left Alt
                kbd->alt_left = !is_release;
                return;
            case 0x58:  // Caps Lock
                if (!is_release) {
                    kbd->caps_lock = !kbd->caps_lock;
                }
                return;
        }
    } else {
        switch (scancode) {
            case 0x14:  // Right Ctrl
                kbd->ctrl_right = !is_release;
                return;
            case 0x11:  // Right Alt
                kbd->alt_right = !is_release;
                return;
        }
    }
    
    // Only process key presses, not releases (for typing)
    if (is_release) {
        if (kbd->callback) {
            uint8_t key = is_extended ? process_extended_scancode(scancode) : 
                          (scancode < sizeof(scancode_to_ascii) ? scancode_to_ascii[scancode] : 0);
            kbd->callback(key, false);
        }
        return;
    }
    
    // Convert scancode to ASCII
    uint8_t ascii = 0;
    
    if (is_extended) {
        ascii = process_extended_scancode(scancode);
    } else if (scancode < sizeof(scancode_to_ascii)) {
        bool shift = kbd->shift_left || kbd->shift_right;
        bool caps = kbd->caps_lock;
        
        // Determine effective shift state
        // For letters, Caps Lock inverts the shift state
        uint8_t base_char = scancode_to_ascii[scancode];
        if (base_char >= 'a' && base_char <= 'z') {
            if (shift ^ caps) {
                ascii = scancode_to_ascii_shifted[scancode];
            } else {
                ascii = base_char;
            }
        } else {
            // For non-letters, only shift affects it
            if (shift) {
                ascii = scancode_to_ascii_shifted[scancode];
            } else {
                ascii = scancode_to_ascii[scancode];
            }
        }
    }
    
    // Add to buffer if valid
    if (ascii != 0) {
        buffer_put(ascii);
        
        if (kbd->callback) {
            kbd->callback(ascii, true);
        }
    }
}

// GPIO ISR handler for clock falling edge
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    if (kbd == NULL) return;
    
    int64_t now = esp_timer_get_time();
    
    // Reset state machine on timeout (> 2ms between bits for compatibility)
    if (now - kbd->last_clock_time > 2000) {
        kbd->state = PS2_STATE_IDLE;
        kbd->bit_count = 0;
        kbd->data_byte = 0;
        kbd->parity = 0;
    }
    kbd->last_clock_time = now;
    
    // Read data bit
    int data_bit = gpio_get_level(kbd->data_pin);
    
    switch (kbd->state) {
        case PS2_STATE_IDLE:
            // Start bit should be 0
            if (data_bit == 0) {
                kbd->state = PS2_STATE_DATA;
                kbd->bit_count = 0;
                kbd->data_byte = 0;
                kbd->parity = 0;
            }
            break;
            
        case PS2_STATE_DATA:
            // Data bits (LSB first)
            kbd->data_byte |= (data_bit << kbd->bit_count);
            kbd->parity ^= data_bit;
            kbd->bit_count++;
            if (kbd->bit_count == 8) {
                kbd->state = PS2_STATE_PARITY;
            }
            break;
            
        case PS2_STATE_PARITY:
            // Parity bit (odd parity)
            kbd->parity ^= data_bit;
            kbd->state = PS2_STATE_STOP;
            break;
            
        case PS2_STATE_STOP:
            // Stop bit should be 1
            if (data_bit == 1 && kbd->parity == 1) {
                // Valid byte received - process it
                // Queue it for processing in task context
                process_scancode(kbd->data_byte);
            }
            kbd->state = PS2_STATE_IDLE;
            break;
    }
}

esp_err_t ps2_keyboard_init(void) {
    if (kbd != NULL && kbd->initialized) {
        ESP_LOGW(TAG, "PS/2 keyboard already initialized");
        return ESP_OK;
    }
    
    // Allocate state
    kbd = calloc(1, sizeof(ps2_keyboard_t));
    if (kbd == NULL) {
        ESP_LOGE(TAG, "Failed to allocate keyboard state");
        return ESP_ERR_NO_MEM;
    }
    
    kbd->clk_pin = PS2_CLK_PIN;
    kbd->data_pin = PS2_DATA_PIN;
    kbd->state = PS2_STATE_IDLE;
    
    // Create semaphores
    kbd->data_ready = xSemaphoreCreateBinary();
    kbd->mutex = xSemaphoreCreateMutex();
    if (kbd->data_ready == NULL || kbd->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        ps2_keyboard_deinit();
        return ESP_ERR_NO_MEM;
    }
    
    // Configure clock pin (input with pull-up, interrupt on falling edge)
    gpio_config_t clk_conf = {
        .pin_bit_mask = (1ULL << kbd->clk_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t ret = gpio_config(&clk_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CLK pin: %s", esp_err_to_name(ret));
        ps2_keyboard_deinit();
        return ret;
    }
    
    // Configure data pin (input with pull-up)
    gpio_config_t data_conf = {
        .pin_bit_mask = (1ULL << kbd->data_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&data_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DATA pin: %s", esp_err_to_name(ret));
        ps2_keyboard_deinit();
        return ret;
    }
    
    // Install GPIO ISR service
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        ps2_keyboard_deinit();
        return ret;
    }
    
    // Add ISR handler for clock pin
    ret = gpio_isr_handler_add(kbd->clk_pin, gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        ps2_keyboard_deinit();
        return ret;
    }
    
    kbd->initialized = true;
    kbd->last_clock_time = esp_timer_get_time();
    
    // Essential: Give keyboard time to power up and stabilize
    // This fixes the issue when serial monitor isn't connected
    ESP_LOGI(TAG, "Waiting for PS/2 keyboard stabilization...");
    vTaskDelay(pdMS_TO_TICKS(500));  // 500ms for better compatibility
    
    ESP_LOGI(TAG, "PS/2 keyboard initialized (CLK=GPIO%d, DAT=GPIO%d)", 
             kbd->clk_pin, kbd->data_pin);
    
    return ESP_OK;
}

void ps2_keyboard_deinit(void) {
    if (kbd == NULL) return;
    
    if (kbd->initialized) {
        gpio_isr_handler_remove(kbd->clk_pin);
    }
    
    if (kbd->data_ready) {
        vSemaphoreDelete(kbd->data_ready);
    }
    if (kbd->mutex) {
        vSemaphoreDelete(kbd->mutex);
    }
    
    free(kbd);
    kbd = NULL;
    
    ESP_LOGI(TAG, "PS/2 keyboard deinitialized");
}

uint8_t ps2_keyboard_read(void) {
    if (kbd == NULL || !kbd->initialized) return 0;
    
    xSemaphoreTake(kbd->mutex, portMAX_DELAY);
    
    uint8_t c = 0;
    if (kbd->buf_head != kbd->buf_tail) {
        c = kbd->buffer[kbd->buf_tail];
        kbd->buf_tail = (kbd->buf_tail + 1) % PS2_BUFFER_SIZE;
    }
    
    xSemaphoreGive(kbd->mutex);
    return c;
}

uint8_t ps2_keyboard_read_blocking(uint32_t timeout_ms) {
    if (kbd == NULL || !kbd->initialized) return 0;
    
    // Check if data already available
    uint8_t c = ps2_keyboard_read();
    if (c != 0) return c;
    
    // Wait for data
    TickType_t timeout = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(kbd->data_ready, timeout) == pdTRUE) {
        c = ps2_keyboard_read();
    }
    
    return c;
}

bool ps2_keyboard_available(void) {
    if (kbd == NULL) return false;
    return kbd->buf_head != kbd->buf_tail;
}

void ps2_keyboard_clear(void) {
    if (kbd == NULL) return;
    
    xSemaphoreTake(kbd->mutex, portMAX_DELAY);
    kbd->buf_head = 0;
    kbd->buf_tail = 0;
    xSemaphoreGive(kbd->mutex);
}

esp_err_t ps2_keyboard_set_leds(bool scroll_lock, bool num_lock, bool caps_lock) {
    // TODO: Implement LED control (requires sending data to keyboard)
    // This involves pulling CLK low, then sending 0xED followed by LED byte
    ESP_LOGW(TAG, "LED control not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

void ps2_keyboard_set_callback(ps2_key_callback_t callback) {
    if (kbd != NULL) {
        kbd->callback = callback;
    }
}

bool ps2_keyboard_is_initialized(void) {
    return kbd != NULL && kbd->initialized;
}

void ps2_keyboard_get_modifiers(bool *shift, bool *ctrl, bool *alt) {
    if (kbd == NULL) {
        if (shift) *shift = false;
        if (ctrl) *ctrl = false;
        if (alt) *alt = false;
        return;
    }
    
    if (shift) *shift = kbd->shift_left || kbd->shift_right;
    if (ctrl) *ctrl = kbd->ctrl_left || kbd->ctrl_right;
    if (alt) *alt = kbd->alt_left || kbd->alt_right;
}
