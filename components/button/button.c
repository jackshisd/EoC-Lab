#include "button.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "oled_ssd1306.h"

#define BUTTON_GPIO GPIO_NUM_1
#define BUZZER_GPIO GPIO_NUM_2
#define DEBOUNCE_MS 30
#define LONG_PRESS_MS 500
#define BUZZER_PULSE_MS 50
#define BUZZER_FREQ_HZ 2000
#define BUZZER_DUTY_RES LEDC_TIMER_10_BIT

static const char *TAG = "button";

static volatile bool s_paused = false;
static volatile bool s_recording = false;
static volatile TickType_t s_record_start_tick = 0;
static char s_status_line[64] = "Ready";

// Logs button state changes to the console.
static void s_log_info(const char *text)
{
    ESP_LOGI(TAG, "%s", text);
}

// Plays a short buzzer pulse for feedback.
static void s_buzzer_pulse(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (1 << BUZZER_DUTY_RES) / 2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(BUZZER_PULSE_MS));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Updates the OLED with timer/status once per second.
static void s_oled_task(void *arg)
{
    (void)arg;
    char buffer[64];
    while (true) {
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
            oled_ssd1306_display_text(buffer);
        } else {
            oled_ssd1306_display_text(s_status_line);
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
                        if (s_recording) {
                            s_recording = false;
                            s_paused = false;
                            s_log_info("Recording stopped");
                        } else {
                            s_recording = true;
                            s_paused = false;
                            s_record_start_tick = xTaskGetTickCount();
                            s_log_info("Recording started");
                        }
                    } else {
                        if (s_recording) {
                            s_paused = !s_paused;
                            s_log_info(s_paused ? "Paused" : "Recording");
                        }
                    }
                    s_buzzer_pulse();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Initializes GPIO, buzzer, and background tasks.
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

    ledc_timer_config_t buzzer_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = BUZZER_DUTY_RES,
        .freq_hz = BUZZER_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&buzzer_timer);

    ledc_channel_config_t buzzer_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&buzzer_channel);

    xTaskCreate(s_button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(s_oled_task, "oled_task", 2048, NULL, 5, NULL);
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
