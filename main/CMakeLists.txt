idf_component_register(SRCS "amiga.c" "atari_st.c" "usb_keyboard.c" "usb_mouse.c" "i8086_pc.c" "ssh_debug_server.c" "wifi_controller.c" "bt_controller.c" "lcd_console.c" "hello_world_main.c" "m68k_emulator.c" "bus_controller.c" "i8086_emulator.c" "disk_image.c" "i8086_cpu.c" "ps2_keyboard.c" "wifi_ftp_server.c"
                            "mos6502_cpu.c" "mos6502_emulator.c"
                            "video_card.c" "video_card_3d.c"
                    INCLUDE_DIRS "."
                    REQUIRES console nvs_flash fatfs sdmmc driver esp_lcd esp_timer spi_flash lwip esp_wifi esp_event esp_netif bt usb)
