#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Blocking capture (existing behavior).
esp_err_t mic_capture_to_file(const char *path, int seconds, int *out_seconds);
const char *mic_capture_last_stage(void);
esp_err_t mic_capture_contact_init(void);
bool mic_capture_contact_is_ready(void);
esp_err_t mic_capture_contact_to_file(const char *path, int seconds, int *out_seconds);
esp_err_t mic_capture_contact_debug_to_file(const char *path, int seconds, int *out_seconds);
const char *mic_capture_contact_debug_last_stage(void);
int mic_capture_contact_debug_last_reg(void);
void mic_capture_contact_set_status_callback(void (*callback)(const char *status));

// Async capture helpers.
esp_err_t mic_capture_start(const char *path, int seconds);
bool mic_capture_is_running(void);
esp_err_t mic_capture_wait(int *out_seconds, TickType_t timeout);
TickType_t mic_capture_last_progress_tick(void);
esp_err_t mic_capture_contact_start(const char *path, int seconds);
bool mic_capture_contact_is_running(void);
esp_err_t mic_capture_contact_wait(int *out_seconds, TickType_t timeout);
TickType_t mic_capture_contact_last_progress_tick(void);
