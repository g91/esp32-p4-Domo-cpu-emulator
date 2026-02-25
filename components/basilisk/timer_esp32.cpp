/*
 *  timer_esp32.cpp - Timer functions for ESP-IDF
 *  BasiliskII ESP32-P4 Port
 */

#include "sysdeps.h"
#include "timer.h"

uint64 GetTicks_usec(void)
{
    return (uint64)esp_timer_get_time();
}

void Delay_usec(uint64 usec)
{
    if (usec > 0) {
        vTaskDelay(pdMS_TO_TICKS(usec / 1000));
    }
}

// Timer idle function (called from CPU loop)
void idle_wait(void)
{
    vTaskDelay(1);
}
