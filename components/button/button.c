#include "button.h"

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "oled_ssd1306.h"

#define BUTTON_GPIO GPIO_NUM_45
#define DEBOUNCE_MS 30
#define LONG_PRESS_MS 500

static const char *TAG = "button";

static volatile bool s_paused = false;
static volatile bool s_recording = false;
static volatile TickType_t s_record_start_tick = 0;
static char s_status_line[64] = "Ready";
static TickType_t s_oled_next_retry_tick = 0;
static uint8_t s_oled_fail_count = 0;
static char s_oled_last_text[64];

static void s_append_event_log(const char *fmt, ...)
{
    FILE *f = fopen("/sdcard/rec_event.txt", "a");
    if (f == NULL) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    vfprintf(f, fmt, args);
    va_end(args);
    fputc('\n', f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
}

static void s_log_oled_error_to_sd(esp_err_t err)
{
    FILE *f = fopen("/sdcard/oled_err.txt", "a");
    if (f == NULL) {
        return;
    }
    fprintf(f, "oled write failed: %s (%d)\n", esp_err_to_name(err), (int)err);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
}

// Logs button state changes to the console.
static void s_log_info(const char *text)
{
    ESP_LOGI(TAG, "%s", text);
}

static void s_handle_short_press(void)
{
    if (s_recording) {
        s_paused = !s_paused;
        s_log_info(s_paused ? "Paused" : "Recording");
        s_append_event_log("button short press -> %s", s_paused ? "paused" : "resumed");
    }
}

static void s_handle_long_press(void)
{
    if (s_recording) {
        s_recording = false;
        s_paused = false;
        s_log_info("Recording stopped");
        s_append_event_log("button long press -> stop");
    } else {
        s_recording = true;
        s_paused = false;
        s_record_start_tick = xTaskGetTickCount();
        s_log_info("Recording started");
        s_append_event_log("button long press -> start");
    }
}

// Updates the OLED with timer/status once per second.
static void s_oled_task(void *arg)
{
    (void)arg;
    char buffer[64];
    while (true) {
        if (s_oled_next_retry_tick != 0 && xTaskGetTickCount() < s_oled_next_retry_tick) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        esp_err_t ret;
        if (s_recording) {
            TickType_t elapsed_ticks = xTaskGetTickCount() - s_record_start_tick;
            uint32_t elapsed_seconds = (uint32_t)(elapsed_ticks / configTICK_RATE_HZ);
            uint32_t hours = elapsed_seconds / 3600;
            uint32_t minutes = (elapsed_seconds % 3600) / 60;
            uint32_t seconds = elapsed_seconds % 60;
            const char *line2 = s_paused ? "Paused" : "Recording";
            snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu\n%s",
                     (unsigned long)hours,
                     (unsigned long)minutes,
                     (unsigned long)seconds,
                     line2);
            if (strcmp(buffer, s_oled_last_text) == 0) {
                ret = ESP_OK;
            } else {
                ret = oled_ssd1306_display_text(buffer);
                if (ret == ESP_OK) {
                    snprintf(s_oled_last_text, sizeof(s_oled_last_text), "%s", buffer);
                }
            }
        } else {
            if (strcmp(s_status_line, s_oled_last_text) == 0) {
                ret = ESP_OK;
            } else {
                ret = oled_ssd1306_display_text(s_status_line);
                if (ret == ESP_OK) {
                    snprintf(s_oled_last_text, sizeof(s_oled_last_text), "%s", s_status_line);
                }
            }
        }
        if (ret != ESP_OK) {
            s_log_oled_error_to_sd(ret);
            s_oled_fail_count++;
            if (s_oled_fail_count >= 3) {
                s_oled_next_retry_tick = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
                s_oled_fail_count = 0;
            }
        } else {
            s_oled_fail_count = 0;
            s_oled_next_retry_tick = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Handles debounced button presses and toggles recording state.
static void s_button_task(void *arg)
{
    (void)arg;
    bool last_level = true;
    TickType_t press_tick = 0;

    while (true) {
        bool level = gpio_get_level(BUTTON_GPIO);
        if (level != last_level) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            level = gpio_get_level(BUTTON_GPIO);
            if (level != last_level) {
                last_level = level;
                if (!level) {
                    press_tick = xTaskGetTickCount();
                } else {
                    TickType_t held = xTaskGetTickCount() - press_tick;
                    if (held >= pdMS_TO_TICKS(LONG_PRESS_MS)) {
                        s_handle_long_press();
                    } else {
                        s_handle_short_press();
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Initializes GPIO and background tasks.
void button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    xTaskCreate(s_button_task, "button_task", 4096, NULL, 10, NULL);
    xTaskCreate(s_oled_task, "oled_task", 4096, NULL, 5, NULL);
}

// Sets the idle OLED display lines shown when not recording.
void button_set_idle_display(const char *line1, const char *line2)
{
    if (line1 == NULL) {
        line1 = "";
    }
    if (line2 == NULL) {
        line2 = "";
    }
    snprintf(s_status_line, sizeof(s_status_line), "%s\n%s", line1, line2);
}

void button_force_idle(void)
{
    s_recording = false;
    s_paused = false;
    s_record_start_tick = 0;
    s_log_info("Recording forced idle");
    s_append_event_log("button force idle");
}

// Returns whether recording is currently paused.
bool button_is_paused(void)
{
    return s_paused;
}

// Returns whether recording is currently active.
bool button_is_recording(void)
{
    return s_recording;
}

void button_trigger_short_press(void)
{
    s_handle_short_press();
}

void button_trigger_long_press(void)
{
    s_handle_long_press();
}
