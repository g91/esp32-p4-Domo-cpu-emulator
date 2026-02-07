# Quick Start - M68K Test Applications

## ✅ What's Ready Now

Three M68K test binaries are compiled and ready to run:
- `m68k_apps/test1_simple.bin` - Counter test (30 bytes)
- `m68k_apps/test2_registers.bin` - Register test (26 bytes)
- `m68k_apps/test3_full.bin` - Full instruction test (80 bytes)

## 🚀 How to Test (3 Steps)

### Step 1: Flash Firmware
```cmd
idf.py -p COM3 flash monitor
```

### Step 2: Upload Binaries to SD Card

**Option A - Via FTP:**
```
# In serial console
m68k> connect YourWiFi YourPassword

# Then use FTP client
ftp://[ESP32_IP]:21
# Upload .bin files from m68k_apps/ to /sdcard/
```

**Option B - Direct SD Card:**
- Remove SD card from board
- Copy .bin files to root
- Reinsert SD card

### Step 3: Run Tests
```
m68k> loadsd /sdcard/test1_simple.bin
m68k> reset
m68k> run 100
m68k> state
```

Expected: `D0 = 0x00000064` (100 in hex)

## 📖 Full Documentation

- **[DEVELOPMENT_SUMMARY.md](DEVELOPMENT_SUMMARY.md)** - Complete overview
- **[m68k_apps/README.md](m68k_apps/README.md)** - M68K app details
- **[riscv_apps/README.md](riscv_apps/README.md)** - Native test app
- **[M68K_COMPILER_SETUP.md](M68K_COMPILER_SETUP.md)** - Compiler setup (optional)

## 🔧 Build System

**No compiler needed for provided binaries!**

To rebuild binaries:
```cmd
cd m68k_apps
python build_binaries.py
```

To compile C source (requires m68k-elf-gcc):
```cmd
cd m68k_apps
build_m68k.cmd
```
