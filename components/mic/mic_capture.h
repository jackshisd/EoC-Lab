#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Blocking capture (existing behavior).
esp_err_t mic_capture_to_file(const char *path, int seconds, int *out_seconds);

// Async capture helpers.
esp_err_t mic_capture_start(const char *path, int seconds);
bool mic_capture_is_running(void);
esp_err_t mic_capture_wait(int *out_seconds, TickType_t timeout);
