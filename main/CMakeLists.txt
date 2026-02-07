idf_component_register(SRCS "ssh_debug_server.c" "wifi_controller.c" "bt_controller.c" "lcd_console.c" "hello_world_main.c" "m68k_emulator.c" "bus_controller.c" "i8086_emulator.c" "disk_image.c" "i8086_cpu.c" "ps2_keyboard.c" "wifi_ftp_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES console nvs_flash fatfs sdmmc driver esp_lcd esp_timer spi_flash lwip esp_wifi esp_event esp_netif bt)
