#include "camera_ov2640.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "button.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "img_converters.h"

void camera_ov2640_get_default_pins(camera_ov2640_pins_t *pins)
{
    if (!pins) {
        return;
    }

    *pins = (camera_ov2640_pins_t){
        .pin_d0 = 8,
        .pin_d1 = 9,
        .pin_d2 = 10,
        .pin_d3 = 11,
        .pin_d4 = 12,
        .pin_d5 = 13,
        .pin_d6 = 14,
        .pin_d7 = 17,
        .pin_pclk = 18,
        .pin_vsync = 21,
        .pin_href = 46,
        .pin_xclk = 3,
        .pin_sda = 47,
        .pin_scl = 48,
        .pin_pwdn = -1,
        .pin_reset = -1,
    };
}

#define VIDEO_FRAME_SIZE   FRAMESIZE_QQVGA
#define VIDEO_JPEG_QUALITY 12
#define VIDEO_XCLK_HZ      10000000
#define VIDEO_FLUSH_BYTES  (4 * 1024 * 1024)

static const char *TAG = "example";

typedef struct {
    char path[64];
} camera_task_args_t;

static TaskHandle_t s_camera_task = NULL;
static bool s_camera_ready = false;
static uint8_t *s_black_jpeg = NULL;
static size_t s_black_jpeg_len = 0;
static bool s_psram_ok = false;

// Returns the resolution for the selected camera frame size.
static void s_frame_size_to_dim(framesize_t size, int *width, int *height)
{
    switch (size) {
        case FRAMESIZE_VGA:
            *width = 640;
            *height = 480;
            break;
        default:
            *width = 640;
            *height = 480;
            break;
    }
}

// Builds a black JPEG frame that can be reused while paused.
static esp_err_t s_prepare_black_frame(framesize_t size, int quality)
{
    int width = 0;
    int height = 0;
    s_frame_size_to_dim(size, &width, &height);
    if (width <= 0 || height <= 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t rgb_bytes = (size_t)width * (size_t)height * 2;
    uint8_t *rgb = heap_caps_malloc(rgb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rgb) {
        rgb = malloc(rgb_bytes);
    }
    if (!rgb) {
        return ESP_ERR_NO_MEM;
    }
    memset(rgb, 0, rgb_bytes);

    uint8_t *jpeg = NULL;
    size_t jpeg_len = 0;
    bool ok = fmt2jpg(rgb, rgb_bytes, width, height, PIXFORMAT_RGB565, quality, &jpeg, &jpeg_len);
    free(rgb);
    if (!ok || !jpeg || jpeg_len == 0) {
        if (jpeg) {
            free(jpeg);
        }
        return ESP_FAIL;
    }

    if (s_black_jpeg) {
        free(s_black_jpeg);
    }
    s_black_jpeg = jpeg;
    s_black_jpeg_len = jpeg_len;
    return ESP_OK;
}

// Records MJPEG frames to a file while the main recorder is active.
static void s_camera_record_task(void *arg)
{
    camera_task_args_t *args = (camera_task_args_t *)arg;
    FILE *f = fopen(args->path, "wb");
    if (!f) {
        int err = errno;
        ESP_LOGE(TAG, "Failed to open video file %s (errno=%d: %s)", args->path, err, strerror(err));
        s_camera_task = NULL;
        free(args);
        vTaskDelete(NULL);
        return;
    }

    size_t bytes_since_flush = 0;
    uint32_t bad_jpeg_count = 0;
    uint32_t good_frame_count = 0;
    while (button_is_recording()) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (button_is_paused() && s_black_jpeg && s_black_jpeg_len > 0) {
            fwrite(s_black_jpeg, 1, s_black_jpeg_len, f);
            bytes_since_flush += s_black_jpeg_len;
        } else {
            if (fb->len < 4 || fb->buf[0] != 0xFF || fb->buf[1] != 0xD8 ||
                fb->buf[fb->len - 2] != 0xFF || fb->buf[fb->len - 1] != 0xD9) {
                bad_jpeg_count++;
                if ((bad_jpeg_count % 50) == 1) {
                    ESP_LOGW(TAG, "Skipping bad JPEG frame (SOI/EOI), count=%u", (unsigned)bad_jpeg_count);
                }
                esp_camera_fb_return(fb);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            fwrite(fb->buf, 1, fb->len, f);
            bytes_since_flush += fb->len;
            good_frame_count++;
            if ((good_frame_count % 50) == 0) {
                ESP_LOGI(TAG, "Wrote %u frames (%u bad)", (unsigned)good_frame_count, (unsigned)bad_jpeg_count);
            }
        }

        esp_camera_fb_return(fb);

        if (bytes_since_flush >= VIDEO_FLUSH_BYTES) {
            fflush(f);
            fsync(fileno(f));
            bytes_since_flush = 0;
        }
    }

    fflush(f);
    fsync(fileno(f));
    fclose(f);
    s_camera_task = NULL;
    free(args);
    vTaskDelete(NULL);
}

void camera_app_log_i2c_levels(void)
{
    camera_ov2640_pins_t pins;
    camera_ov2640_get_default_pins(&pins);

    gpio_reset_pin(pins.pin_sda);
    gpio_reset_pin(pins.pin_scl);
    gpio_set_direction(pins.pin_sda, GPIO_MODE_INPUT);
    gpio_set_direction(pins.pin_scl, GPIO_MODE_INPUT);

    int sda = gpio_get_level(pins.pin_sda);
    int scl = gpio_get_level(pins.pin_scl);
    ESP_LOGI(TAG, "I2C idle levels: SDA=%d SCL=%d (expect 1)", sda, scl);
}

// Initializes the OV2640 camera with the project pin map.
esp_err_t camera_app_init(void)
{
    camera_ov2640_pins_t pins;
    camera_ov2640_get_default_pins(&pins);
    gpio_reset_pin(pins.pin_sda);
    gpio_reset_pin(pins.pin_scl);
    if (pins.pin_pwdn >= 0) {
        gpio_reset_pin(pins.pin_pwdn);
        gpio_set_direction(pins.pin_pwdn, GPIO_MODE_OUTPUT);
        gpio_set_level(pins.pin_pwdn, 0);
    }
    if (pins.pin_reset >= 0) {
        gpio_reset_pin(pins.pin_reset);
        gpio_set_direction(pins.pin_reset, GPIO_MODE_OUTPUT);
        gpio_set_level(pins.pin_reset, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(pins.pin_reset, 1);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    bool psram_ok = false;
#if CONFIG_SPIRAM
    psram_ok = true;
#endif
    s_psram_ok = psram_ok;
    if (!psram_ok) {
        ESP_LOGW(TAG, "PSRAM not initialized; using DRAM framebuffer and smaller frame size");
    }

    camera_config_t config = {
        .pin_pwdn = pins.pin_pwdn,
        .pin_reset = pins.pin_reset,
        .pin_xclk = pins.pin_xclk,
        .pin_sccb_sda = pins.pin_sda,
        .pin_sccb_scl = pins.pin_scl,
        .pin_d7 = pins.pin_d7,
        .pin_d6 = pins.pin_d6,
        .pin_d5 = pins.pin_d5,
        .pin_d4 = pins.pin_d4,
        .pin_d3 = pins.pin_d3,
        .pin_d2 = pins.pin_d2,
        .pin_d1 = pins.pin_d1,
        .pin_d0 = pins.pin_d0,
        .pin_vsync = pins.pin_vsync,
        .pin_href = pins.pin_href,
        .pin_pclk = pins.pin_pclk,
        .xclk_freq_hz = VIDEO_XCLK_HZ,
        .ledc_timer = LEDC_TIMER_1,
        .ledc_channel = LEDC_CHANNEL_1,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = psram_ok ? VIDEO_FRAME_SIZE : FRAMESIZE_QVGA,
        .jpeg_quality = VIDEO_JPEG_QUALITY,
        .fb_count = psram_ok ? 3 : 1,
        .fb_location = psram_ok ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = I2C_NUM_0,
    };

    esp_err_t ret = esp_camera_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed (%s)", esp_err_to_name(ret));
        return ret;
    }
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        sensor->set_pixformat(sensor, PIXFORMAT_JPEG);
        sensor->set_framesize(sensor, VIDEO_FRAME_SIZE);
        sensor->set_quality(sensor, VIDEO_JPEG_QUALITY);
    }

    // Warm-up: discard initial frames that are often corrupted.
    for (int i = 0; i < 10; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    if (s_psram_ok) {
        ret = s_prepare_black_frame(VIDEO_FRAME_SIZE, VIDEO_JPEG_QUALITY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Black frame init failed (%s)", esp_err_to_name(ret));
            return ret;
        }
    } else {
        ESP_LOGW(TAG, "Skipping black frame without PSRAM");
    }

    s_camera_ready = true;
    return ESP_OK;
}

bool camera_app_is_ready(void)
{
    return s_camera_ready;
}

bool camera_app_is_recording(void)
{
    return s_camera_task != NULL;
}

esp_err_t camera_app_start_record(const char *path)
{
    if (!s_camera_ready || s_camera_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    camera_task_args_t *args = malloc(sizeof(camera_task_args_t));
    if (!args) {
        return ESP_ERR_NO_MEM;
    }
    strncpy(args->path, path, sizeof(args->path));
    args->path[sizeof(args->path) - 1] = '\0';

    if (xTaskCreate(s_camera_record_task, "camera_record", 4096, args, 5, &s_camera_task) != pdPASS) {
        free(args);
        s_camera_task = NULL;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void camera_app_wait_for_stop(void)
{
    while (s_camera_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
