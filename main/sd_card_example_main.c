/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// This example uses SDMMC peripheral to communicate with SD card.

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "button.h"
#include "mic_capture.h"
#include "oled_ssd1306.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "camera_ov2640.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "sd_test_io.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#define EXAMPLE_MAX_CHAR_SIZE    64

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"
#define EXAMPLE_IS_UHS1    (CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR50 || CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_DDR50)

#define VIDEO_FRAME_SIZE   FRAMESIZE_QQVGA
#define VIDEO_JPEG_QUALITY 12
#define VIDEO_XCLK_HZ      10000000
#define CAMERA_SCCB_SDA    47
#define CAMERA_SCCB_SCL    48
#define OLED_I2C_SDA       41
#define OLED_I2C_SCL       42
#define I2C_SHARED_FREQ_HZ 100000
#define VIDEO_FLUSH_BYTES  (4 * 1024 * 1024)

#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
const char* names[] = {"CLK", "CMD", "D0", "D1", "D2", "D3"};
const int pins[] = {4, 5, 6, 7, 15, 16};

const int pin_count = sizeof(pins)/sizeof(pins[0]);

#if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
const int adc_channels[] = {CONFIG_EXAMPLE_ADC_PIN_CLK,
                            CONFIG_EXAMPLE_ADC_PIN_CMD,
                            CONFIG_EXAMPLE_ADC_PIN_D0,
                            CONFIG_EXAMPLE_ADC_PIN_D1,
                            CONFIG_EXAMPLE_ADC_PIN_D2,
                            CONFIG_EXAMPLE_ADC_PIN_D3
                            };
#endif //CONFIG_EXAMPLE_ENABLE_ADC_FEATURE

pin_configuration_t config = {
    .names = names,
    .pins = pins,
#if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
    .adc_channels = adc_channels,
#endif
};
#endif //CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS

/* TinyUSB descriptors */
#define EPNUM_MSC            1
#define TUSB_DESC_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

enum {
    ITF_NUM_MSC = 0,
    ITF_NUM_TOTAL
};

enum {
    EDPT_CTRL_OUT = 0x00,
    EDPT_CTRL_IN  = 0x80,

    EDPT_MSC_OUT  = 0x01,
    EDPT_MSC_IN   = 0x81,
};

static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,
    .idProduct = 0x4002,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static uint8_t const msc_fs_configuration_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 64),
};

#if (TUD_OPT_HIGH_SPEED)
static const tusb_desc_device_qualifier_t device_qualifier = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0
};

static uint8_t const msc_hs_configuration_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 512),
};
#endif

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },
    "TinyUSB",
    "TinyUSB Device",
    "123456",
};

static tinyusb_msc_storage_handle_t s_storage_hdl;
static tinyusb_config_t s_tusb_cfg;
static bool s_usb_active;

typedef struct {
    char path[EXAMPLE_MAX_CHAR_SIZE];
} camera_task_args_t;

static TaskHandle_t s_camera_task = NULL;
static bool s_camera_ready = false;
static uint8_t *s_black_jpeg = NULL;
static size_t s_black_jpeg_len = 0;
static bool s_psram_ok = false;

// Writes a test string to a file on the SD card.
static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

// Reads a line from a file on the SD card.
static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

// Initializes the SDMMC host/slot and returns a ready card handle.
static esp_err_t s_storage_init_sdmmc(sdmmc_card_t **card)
{
    esp_err_t ret = ESP_OK;
    bool host_init = false;
    sdmmc_card_t *sd_card = NULL;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
#if CONFIG_EXAMPLE_SDMMC_SPEED_HS
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR50
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_SDR50;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_DDR50
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_DDR50;
#endif

#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
#if EXAMPLE_IS_UHS1
    slot_config.flags |= SDMMC_SLOT_FLAG_UHS1;
#endif
    slot_config.width = 4;

#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
    slot_config.clk = 4;
    slot_config.cmd = 5;
    slot_config.d0 = 6;
    slot_config.d1 = 7;
    slot_config.d2 = 15;
    slot_config.d3 = 16;
#endif

    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    sd_card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));
    if (!sd_card) {
        return ESP_ERR_NO_MEM;
    }

    ret = (*host.init)();
    if (ret != ESP_OK) {
        goto clean;
    }
    host_init = true;

    ret = sdmmc_host_init_slot(host.slot, (const sdmmc_slot_config_t *)&slot_config);
    if (ret != ESP_OK) {
        goto clean;
    }

    while (sdmmc_card_init(&host, sd_card)) {
        ESP_LOGE(TAG, "Insert uSD card. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    sdmmc_card_print_info(stdout, sd_card);
    *card = sd_card;
    return ESP_OK;

clean:
    if (host_init) {
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
            host.deinit_p(host.slot);
        } else {
            (*host.deinit)();
        }
    }
    if (sd_card) {
        free(sd_card);
    }
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
#endif
    return ret;
}

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

// Initializes the OV2640 camera with the project pin map.
static esp_err_t s_camera_init(void)
{
    camera_ov2640_pins_t pins;
    camera_ov2640_get_default_pins(&pins);
    pins.pin_sda = CAMERA_SCCB_SDA;
    pins.pin_scl = CAMERA_SCCB_SCL;
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


static bool s_wait_for_mount(const char *path, int timeout_ms)
{
    struct stat st;
    const int step_ms = 50;
    int waited = 0;
    while (waited < timeout_ms) {
        if (stat(path, &st) == 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        waited += step_ms;
    }
    return false;
}

// Switches TinyUSB MSC mount point between USB and app.
static esp_err_t s_switch_mount(tinyusb_msc_mount_point_t mount_point)
{
    return tinyusb_msc_set_storage_mount_point(s_storage_hdl, mount_point);
}

// Starts the TinyUSB MSC driver if not already running.
static esp_err_t s_usb_start(void)
{
    if (s_usb_active) {
        return ESP_OK;
    }
    esp_err_t ret = tinyusb_driver_install(&s_tusb_cfg);
    if (ret == ESP_OK) {
        s_usb_active = true;
        ESP_LOGI(TAG, "USB MSC ready");
    }
    return ret;
}

// Stops the TinyUSB MSC driver if running.
static void s_usb_stop(void)
{
    if (!s_usb_active) {
        return;
    }
    esp_err_t ret = tinyusb_driver_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "USB uninstall failed (%s)", esp_err_to_name(ret));
        return;
    }
    s_usb_active = false;
    ESP_LOGI(TAG, "USB MSC stopped");
}

static void s_i2c_log_pin_levels(const camera_ov2640_pins_t *pins)
{
    if (!pins) {
        return;
    }

    gpio_reset_pin(pins->pin_sda);
    gpio_reset_pin(pins->pin_scl);
    gpio_set_direction(pins->pin_sda, GPIO_MODE_INPUT);
    gpio_set_direction(pins->pin_scl, GPIO_MODE_INPUT);

    int sda = gpio_get_level(pins->pin_sda);
    int scl = gpio_get_level(pins->pin_scl);
    ESP_LOGI(TAG, "I2C idle levels: SDA=%d SCL=%d (expect 1)", sda, scl);
}


// Initializes peripherals and handles record/USB switching loop.
void app_main(void)
{
    esp_err_t ret;
    camera_ov2640_pins_t pins;
    camera_ov2640_get_default_pins(&pins);
    s_i2c_log_pin_levels(&pins);

    ret = i2c_bus_init(I2C_NUM_1, OLED_I2C_SDA, OLED_I2C_SCL, I2C_SHARED_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed (%s)", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Initializing SD card");
    if (s_camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera disabled");
    }
    if (oled_ssd1306_init() != ESP_OK) {
        ESP_LOGE(TAG, "OLED init failed");
    }
    button_init();

    sdmmc_card_t *card = NULL;
    ret = s_storage_init_sdmmc(&card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SD card (%s)", esp_err_to_name(ret));
        return;
    }

    tinyusb_msc_storage_config_t storage_cfg = {
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB,
        .fat_fs = {
            .base_path = MOUNT_POINT,
            .config.max_files = 5,
            .format_flags = 0,
        },
        .medium.card = card,
    };

    ESP_ERROR_CHECK(tinyusb_msc_new_storage_sdmmc(&storage_cfg, &s_storage_hdl));

    s_tusb_cfg = (tinyusb_config_t)TINYUSB_DEFAULT_CONFIG();
    s_tusb_cfg.descriptor.device = &descriptor_config;
    s_tusb_cfg.descriptor.full_speed_config = msc_fs_configuration_desc;
    s_tusb_cfg.descriptor.string = string_desc_arr;
    s_tusb_cfg.descriptor.string_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]);
#if (TUD_OPT_HIGH_SPEED)
    s_tusb_cfg.descriptor.high_speed_config = msc_hs_configuration_desc;
    s_tusb_cfg.descriptor.qualifier = &device_qualifier;
#endif

    ESP_ERROR_CHECK(s_usb_start());
    ESP_LOGI(TAG, "Exposing SD card over USB");
    ESP_ERROR_CHECK(s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_USB));

    uint32_t file_index = 1;
    while (true) {
        while (!button_is_recording()) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        ESP_LOGI(TAG, "Disabling USB and mounting SD card for recording");
        s_usb_stop();
        ret = s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_APP);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to mount to app (%s)", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        char mic_path[EXAMPLE_MAX_CHAR_SIZE];
        char video_path[EXAMPLE_MAX_CHAR_SIZE];
        snprintf(mic_path, sizeof(mic_path), MOUNT_POINT"/mic_%04u.wav", (unsigned)file_index);
        // Use 8.3-compatible name to avoid FATFS EINVAL when LFN is disabled.
        snprintf(video_path, sizeof(video_path), MOUNT_POINT"/VID%04u.MJP", (unsigned)file_index);

        if (!s_wait_for_mount(MOUNT_POINT, 2000)) {
            ESP_LOGE(TAG, "Mount not ready for %s", MOUNT_POINT);
        }

        if (s_camera_ready && s_camera_task == NULL) {
            camera_task_args_t *args = malloc(sizeof(camera_task_args_t));
            if (args) {
                strncpy(args->path, video_path, sizeof(args->path));
                args->path[sizeof(args->path) - 1] = '\0';
                if (xTaskCreate(s_camera_record_task, "camera_record", 4096, args, 5, &s_camera_task) != pdPASS) {
                    free(args);
                }
            }
        }
        int captured_seconds = 0;
        ret = mic_capture_to_file(mic_path, 0, &captured_seconds);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Mic capture failed");
        } else {
            char line1[32];
            const char *filename = strrchr(mic_path, '/');
            if (filename != NULL) {
                filename++;
            } else {
                filename = mic_path;
            }
            snprintf(line1, sizeof(line1), "Recorded %ds at", captured_seconds);
            button_set_idle_display(line1, filename);
            file_index++;
        }

        ESP_LOGI(TAG, "Exposing SD card over USB");
        while (s_camera_task != NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_ERROR_CHECK(s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_USB));
        ESP_ERROR_CHECK(s_usb_start());
    }
}
