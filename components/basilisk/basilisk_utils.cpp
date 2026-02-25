/*
 *  basilisk_utils.cpp - Utility functions for BasiliskII ESP-IDF port
 *
 *  Provides: basilisk_log(), basilisk_strdup()
 */

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cstdlib>
#include "esp_log.h"

static const char *TAG = "B2";

/*
 *  Logging function used by write_log macro (redirected from sysdeps.h)
 */
extern "C" void basilisk_log(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // Strip trailing newline for ESP_LOGI (it adds its own)
    size_t len = strlen(buf);
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
        buf[--len] = '\0';
    }

    if (len > 0) {
        ESP_LOGI(TAG, "%s", buf);
    }
}

/*
 *  strdup implementation (safe for all allocators)
 */
extern "C" char *basilisk_strdup(const char *s)
{
    if (!s) return NULL;
    size_t len = strlen(s) + 1;
    char *copy = (char *)malloc(len);
    if (copy) {
        memcpy(copy, s, len);
    }
    return copy;
}
