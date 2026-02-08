#
# M68K SDK - Makefile
#
# Usage:
#   make                    - Build the SDK library (libm68k.a)
#   make APP=myapp.c        - Build an application linked with the SDK
#   make clean              - Remove build artifacts
#
# The toolchain prefix can be overridden:
#   make CROSS=m68k-elf-
#
# Output:
#   build/libm68k.a         - Static library of all SDK modules
#   build/crt0.o            - Startup code (link first)
#   build/<app>.bin          - Raw binary (when APP= specified)
#

# Toolchain
CROSS   ?= m68k-linux-gnu-
CC       = $(CROSS)gcc
AS       = $(CROSS)gcc
AR       = $(CROSS)ar
OBJCOPY  = $(CROSS)objcopy
SIZE     = $(CROSS)size

# Directories
SDK_DIR   = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR   = $(SDK_DIR)src
INC_DIR   = $(SDK_DIR)include
BUILD_DIR = build

# Flags
CFLAGS  = -m68000 -nostdlib -ffreestanding -fno-builtin \
          -Os -Wall -Wextra -I$(INC_DIR)
ASFLAGS = -m68000 -nostdlib -ffreestanding
LDFLAGS = -m68000 -nostdlib -ffreestanding -T$(SDK_DIR)program.ld -lgcc

# SDK source files
SDK_SRCS = $(SRC_DIR)/m68k_string.c  \
           $(SRC_DIR)/m68k_console.c \
           $(SRC_DIR)/m68k_fs.c      \
           $(SRC_DIR)/m68k_net.c     \
           $(SRC_DIR)/m68k_fpu.c     \
           $(SRC_DIR)/m68k_audio.c   \
           $(SRC_DIR)/m68k_video.c   \
           $(SRC_DIR)/m68k_system.c

SDK_OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/sdk_%.o,$(SDK_SRCS))
CRT0_OBJ = $(BUILD_DIR)/crt0.o

# Default target: build library
.PHONY: all clean app

all: $(BUILD_DIR)/libm68k.a $(CRT0_OBJ)
	@echo "SDK built successfully."
	@echo "  Library: $(BUILD_DIR)/libm68k.a"
	@echo "  Startup: $(BUILD_DIR)/crt0.o"

# Build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile SDK sources
$(BUILD_DIR)/sdk_%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup code
$(CRT0_OBJ): $(SDK_DIR)crt0.S | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

# Archive into static library
$(BUILD_DIR)/libm68k.a: $(SDK_OBJS)
	$(AR) rcs $@ $^

# Build an application: make APP=myapp.c
ifdef APP
APP_NAME = $(basename $(notdir $(APP)))
APP_OBJ  = $(BUILD_DIR)/$(APP_NAME).o
APP_ELF  = $(BUILD_DIR)/$(APP_NAME).elf
APP_BIN  = $(BUILD_DIR)/$(APP_NAME).bin

app: $(APP_BIN)
	@echo "Application built: $(APP_BIN)"
	@$(SIZE) $(APP_ELF)

$(APP_OBJ): $(APP) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(APP_ELF): $(CRT0_OBJ) $(APP_OBJ) $(BUILD_DIR)/libm68k.a
	$(CC) $(LDFLAGS) $(CRT0_OBJ) $(APP_OBJ) -L$(BUILD_DIR) -lm68k -o $@

$(APP_BIN): $(APP_ELF)
	$(OBJCOPY) -O binary $< $@
else
app:
	@echo "Usage: make APP=your_program.c app"
endif

clean:
	rm -rf $(BUILD_DIR)
