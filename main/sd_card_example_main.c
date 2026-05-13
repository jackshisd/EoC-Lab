/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// This example uses SDMMC peripheral to communicate with SD card.

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "button.h"
#include "ble_trigger.h"
#include "mic_capture.h"
#include "oled_ssd1306.h"
#include "camera_ov2640.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "sd_test_io.h"
#include "esp_system.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#define EXAMPLE_MAX_CHAR_SIZE    64

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"
#define EXAMPLE_IS_UHS1    (CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR50 || CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_DDR50)
#define ENABLE_TINYUSB_MSC 1
#define SHOW_RESET_CAUSE_ON_OLED 1
#define CONTACT_MIC_DEBUG_MODE 0
#define CONTACT_MIC_DEBUG_SECONDS 10
#define CONTACT_MIC_DEBUG_WAV_PATH MOUNT_POINT"/es8388.wav"
#define STANDARD_MIC_DEBUG_WAV_PATH MOUNT_POINT"/stdmic.wav"
#define ENABLE_CONTACT_MIC_RECORDING 1
#define RECORDING_STALL_TIMEOUT_MS 15000

#define OLED_I2C_SDA       1
#define OLED_I2C_SCL       2
#define I2C_SHARED_FREQ_HZ 100000


#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
const char* names[] = {"CLK", "CMD", "D0", "D1", "D2", "D3"};
const int pins[] = {6, 7, 5, 4, 16, 15};

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

static const char *s_reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXT";
        case ESP_RST_SW: return "SW";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        default: return "UNKNOWN";
    }
}

static void s_append_event_log(const char *fmt, ...)
{
    FILE *f = fopen(MOUNT_POINT"/rec_event.txt", "a");
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
    slot_config.clk = 6;
    slot_config.cmd = 7;
    slot_config.d0 = 5;
    slot_config.d1 = 4;
    slot_config.d2 = 16;
    slot_config.d3 = 15;
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
#if ENABLE_TINYUSB_MSC
    return tinyusb_msc_set_storage_mount_point(s_storage_hdl, mount_point);
#else
    (void)mount_point;
    return ESP_OK;
#endif
}

// Starts the TinyUSB MSC driver if not already running.
static esp_err_t s_usb_start(void)
{
#if !ENABLE_TINYUSB_MSC
    return ESP_OK;
#else
    if (s_usb_active) {
        return ESP_OK;
    }
    esp_err_t ret = tinyusb_driver_install(&s_tusb_cfg);
    if (ret == ESP_OK) {
        s_usb_active = true;
        ESP_LOGI(TAG, "USB MSC ready");
    }
    return ret;
#endif
}

// Stops the TinyUSB MSC driver if running.
static void s_usb_stop(void)
{
#if !ENABLE_TINYUSB_MSC
    return;
#else
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
#endif
}

static esp_err_t s_mount_sd_card_debug(sdmmc_card_t **out_card)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
    slot_config.clk = 6;
    slot_config.cmd = 7;
    slot_config.d0 = 5;
    slot_config.d1 = 4;
    slot_config.d2 = 16;
    slot_config.d3 = 15;
#endif
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    return esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, out_card);
}

static void s_write_contact_debug_result(esp_err_t ret, int captured_seconds)
{
    FILE *f = fopen(MOUNT_POINT"/contact_result.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to write contact_result.txt");
        return;
    }

    fprintf(f, "result=%s\n", esp_err_to_name(ret));
    fprintf(f, "captured_seconds=%d\n", captured_seconds);
    fprintf(f, "wav_path=%s\n", CONTACT_MIC_DEBUG_WAV_PATH);
    fclose(f);
}

static void s_write_dual_mic_debug_result(esp_err_t std_ret, int std_seconds,
                                          esp_err_t contact_ret, int contact_seconds)
{
    FILE *f = fopen(MOUNT_POINT"/contact_result.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to write contact_result.txt");
        return;
    }

    fprintf(f, "standard_result=%s\n", esp_err_to_name(std_ret));
    fprintf(f, "standard_captured_seconds=%d\n", std_seconds);
    fprintf(f, "standard_wav_path=%s\n", STANDARD_MIC_DEBUG_WAV_PATH);
    fprintf(f, "contact_result=%s\n", esp_err_to_name(contact_ret));
    fprintf(f, "contact_captured_seconds=%d\n", contact_seconds);
    fprintf(f, "contact_wav_path=%s\n", CONTACT_MIC_DEBUG_WAV_PATH);
    fclose(f);
}

static void s_build_i2c_scan_summary(char *out, size_t out_size)
{
    if (out_size == 0) {
        return;
    }

    snprintf(out, out_size, "I2C:none");
    if (!i2c_bus_is_init()) {
        return;
    }

    esp_err_t lock_ret = i2c_bus_lock(pdMS_TO_TICKS(200));
    if (lock_ret != ESP_OK) {
        snprintf(out, out_size, "I2C:lockfail");
        return;
    }

    size_t used = (size_t)snprintf(out, out_size, "I2C:");
    int shown = 0;
    bool truncated = false;
    i2c_port_t port = i2c_bus_get_port();

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(20));
        i2c_cmd_link_delete(cmd);

        if (ret != ESP_OK) {
            continue;
        }

        if (shown >= 4) {
            truncated = true;
            break;
        }

        int written = snprintf(out + used, out_size - used, "%s%02X",
                               (shown == 0) ? "" : ",", addr);
        if (written < 0 || (size_t)written >= out_size - used) {
            truncated = true;
            break;
        }
        used += (size_t)written;
        shown++;
    }

    if (shown == 0) {
        snprintf(out, out_size, "I2C:none");
    } else if (truncated && used < out_size) {
        snprintf(out + used, out_size - used, "+");
    }

    i2c_bus_unlock();
}

static void s_oled_show_contact_status(bool oled_ready, const char *status)
{
    if (!oled_ready) {
        return;
    }

    char i2c_line[24];
    char oled_msg[64];
    s_build_i2c_scan_summary(i2c_line, sizeof(i2c_line));
    snprintf(oled_msg, sizeof(oled_msg), "%s\n%s", i2c_line, status);
    oled_ssd1306_display_text(oled_msg);
}

static bool s_contact_oled_ready;

static void s_contact_oled_status_callback(const char *status)
{
    s_oled_show_contact_status(s_contact_oled_ready, status);
}

static void s_run_contact_mic_debug_mode(void)
{
    ESP_LOGW(TAG, "CONTACT_MIC_DEBUG_MODE enabled; skipping normal recorder features");

    bool oled_ready = false;
    esp_err_t ret = i2c_bus_init(I2C_NUM_1, OLED_I2C_SDA, OLED_I2C_SCL, I2C_SHARED_FREQ_HZ);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Contact mic debug I2C init failed (%s)", esp_err_to_name(ret));
    }

    if (oled_ssd1306_init() == ESP_OK) {
        oled_ready = true;
        s_contact_oled_ready = true;
        mic_capture_contact_set_status_callback(s_contact_oled_status_callback);
        s_oled_show_contact_status(oled_ready, "dual mic test");
    }

    sdmmc_card_t *card = NULL;
    ret = s_mount_sd_card_debug(&card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Contact mic debug SD mount failed (%s)", esp_err_to_name(ret));
        s_oled_show_contact_status(oled_ready, "sd mount fail");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    sdmmc_card_print_info(stdout, card);

    s_oled_show_contact_status(oled_ready, "codec boot init");
    ret = mic_capture_contact_init();
    if (ret != ESP_OK) {
        char status[32];
        int fail_reg = mic_capture_contact_debug_last_reg();
        if (fail_reg >= 0 && strcmp(mic_capture_contact_debug_last_stage(), "codec fail") == 0) {
            snprintf(status, sizeof(status), "fail:codec %02X", fail_reg);
        } else {
            snprintf(status, sizeof(status), "fail:%s",
                     mic_capture_contact_debug_last_stage());
        }
        s_oled_show_contact_status(oled_ready, status);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    int standard_seconds = 0;
    int contact_seconds = 0;
    button_force_idle();
    button_trigger_long_press();

    esp_err_t std_ret = mic_capture_start(STANDARD_MIC_DEBUG_WAV_PATH, CONTACT_MIC_DEBUG_SECONDS);
    esp_err_t contact_ret = ESP_OK;
    if (std_ret == ESP_OK) {
        contact_ret = mic_capture_contact_start(CONTACT_MIC_DEBUG_WAV_PATH, CONTACT_MIC_DEBUG_SECONDS);
    }

    if (std_ret == ESP_OK) {
        std_ret = mic_capture_wait(&standard_seconds, portMAX_DELAY);
    }
    if (contact_ret == ESP_OK) {
        contact_ret = mic_capture_contact_wait(&contact_seconds, portMAX_DELAY);
    }
    button_force_idle();

    ESP_LOGI(TAG, "Dual mic debug complete: standard=%s(%d) contact=%s(%d)",
             esp_err_to_name(std_ret), standard_seconds,
             esp_err_to_name(contact_ret), contact_seconds);
    s_write_dual_mic_debug_result(std_ret, standard_seconds, contact_ret, contact_seconds);

    if (std_ret == ESP_OK && contact_ret == ESP_OK) {
        s_oled_show_contact_status(oled_ready, "test finished");
    } else {
        char status[32];
        int fail_reg = mic_capture_contact_debug_last_reg();
        if (std_ret != ESP_OK) {
            snprintf(status, sizeof(status), "fail:std mic");
        } else if (fail_reg >= 0 && strcmp(mic_capture_contact_debug_last_stage(), "codec fail") == 0) {
            snprintf(status, sizeof(status), "fail:codec %02X", fail_reg);
        } else {
            snprintf(status, sizeof(status), "fail:%s",
                     mic_capture_contact_debug_last_stage());
        }
        s_oled_show_contact_status(oled_ready, status);
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void s_handle_record_failure(const char *line1, const char *line2,
                                    const char *reason, uint32_t *failure_count)
{
    if (failure_count != NULL) {
        (*failure_count)++;
    }

    ESP_LOGW(TAG, "Recording recovery: %s (count=%lu)", reason,
             (unsigned long)(failure_count ? *failure_count : 0));
    s_append_event_log("main: recovery begin reason=%s count=%lu", reason,
                       (unsigned long)(failure_count ? *failure_count : 0));

    button_force_idle();
    button_set_idle_display(line1, line2);

    camera_app_wait_for_stop();
    s_append_event_log("main: recovery camera stop done");
    if (mic_capture_is_running()) {
        int discarded_seconds = 0;
        esp_err_t mic_ret = mic_capture_wait(&discarded_seconds, pdMS_TO_TICKS(5000));
        s_append_event_log("main: recovery mic stop %s seconds=%d",
                           esp_err_to_name(mic_ret), discarded_seconds);
    }
#if ENABLE_CONTACT_MIC_RECORDING
    if (mic_capture_contact_is_running()) {
        int discarded_seconds = 0;
        esp_err_t contact_ret = mic_capture_contact_wait(&discarded_seconds, pdMS_TO_TICKS(5000));
        s_append_event_log("main: recovery contact stop %s seconds=%d",
                           esp_err_to_name(contact_ret), discarded_seconds);
    }
#endif

    s_usb_stop();
    esp_err_t ret = s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_APP);
    s_append_event_log("main: recovery switch app %s", esp_err_to_name(ret));
    bool mount_ready = s_wait_for_mount(MOUNT_POINT, 2000);
    s_append_event_log("main: recovery app mount ready=%d", mount_ready);

#if ENABLE_TINYUSB_MSC
    if (failure_count != NULL && *failure_count >= 3) {
        ret = s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_USB);
        s_append_event_log("main: recovery switch usb %s", esp_err_to_name(ret));
        ret = s_usb_start();
        s_append_event_log("main: recovery usb start %s", esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(300));
        s_usb_stop();
        ret = s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_APP);
        s_append_event_log("main: recovery switch app retry %s", esp_err_to_name(ret));
        mount_ready = s_wait_for_mount(MOUNT_POINT, 2000);
        s_append_event_log("main: recovery app mount retry ready=%d", mount_ready);
    }
#endif
}

static const char *s_basename_or_default(const char *path, const char *fallback)
{
    if (fallback == NULL) {
        fallback = "unnamed file";
    }
    if (path == NULL || path[0] == '\0') {
        return fallback;
    }
    const char *filename = strrchr(path, '/');
    if (filename != NULL) {
        filename++;
        if (filename[0] != '\0') {
            return filename;
        }
    }
    return path[0] != '\0' ? path : fallback;
}

static bool s_file_exists(const char *path)
{
    struct stat st;
    return path != NULL && stat(path, &st) == 0;
}

static bool s_progress_stalled(TickType_t last_tick, TickType_t timeout_ticks, TickType_t now)
{
    return last_tick != 0 && (now - last_tick) > timeout_ticks;
}

static const char *s_mic_stage_label(const char *stage)
{
    if (stage == NULL) {
        return "mic fail";
    }
    if (strcmp(stage, "read fail") == 0) {
        return "mic read fail";
    }
    if (strcmp(stage, "write fail") == 0) {
        return "mic write fail";
    }
    if (strcmp(stage, "flush fail") == 0) {
        return "mic flush fail";
    }
    if (strcmp(stage, "seek fail") == 0) {
        return "mic seek fail";
    }
    if (strcmp(stage, "file fail") == 0) {
        return "mic file fail";
    }
    if (strcmp(stage, "alloc fail") == 0) {
        return "mic alloc fail";
    }
    if (strcmp(stage, "i2s init fail") == 0 || strcmp(stage, "i2s en fail") == 0 ||
        strcmp(stage, "i2s new fail") == 0) {
        return "mic i2s fail";
    }
    return "mic fail";
}

static const char *s_contact_stage_label(const char *stage)
{
    if (stage == NULL) {
        return "contact fail";
    }
    if (strcmp(stage, "codec fail") == 0 || strcmp(stage, "codec init") == 0) {
        return "codec fail";
    }
    if (strcmp(stage, "read fail") == 0) {
        return "contact read fail";
    }
    if (strcmp(stage, "write fail") == 0) {
        return "contact write fail";
    }
    if (strcmp(stage, "flush fail") == 0) {
        return "contact flush";
    }
    if (strcmp(stage, "seek fail") == 0) {
        return "contact seek";
    }
    if (strcmp(stage, "file fail") == 0) {
        return "contact file";
    }
    if (strcmp(stage, "alloc fail") == 0) {
        return "contact alloc";
    }
    if (strcmp(stage, "i2s init fail") == 0 || strcmp(stage, "i2s en fail") == 0 ||
        strcmp(stage, "i2s new fail") == 0) {
        return "contact i2s";
    }
    return "contact fail";
}


// Initializes peripherals and handles record/USB switching loop.
void app_main(void)
{
#if CONTACT_MIC_DEBUG_MODE
    s_run_contact_mic_debug_mode();
    return;
#endif

    esp_err_t ret;
    ESP_LOGI(TAG, "Bring-up: start");

    ESP_LOGI(TAG, "Bring-up: camera_app_log_i2c_levels begin");
    camera_app_log_i2c_levels();
    ESP_LOGI(TAG, "Bring-up: camera_app_log_i2c_levels done");

    ESP_LOGI(TAG, "Bring-up: i2c_bus_init begin");
    ret = i2c_bus_init(I2C_NUM_1, OLED_I2C_SDA, OLED_I2C_SCL, I2C_SHARED_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed (%s)", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Bring-up: i2c_bus_init done");

    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Bring-up: camera_app_init begin");
    if (camera_app_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera disabled");
    }
    ESP_LOGI(TAG, "Bring-up: camera_app_init done");

    ESP_LOGI(TAG, "Bring-up: oled_ssd1306_init begin");
    if (oled_ssd1306_init() != ESP_OK) {
        ESP_LOGE(TAG, "OLED init failed");
    } else {
        s_contact_oled_ready = true;
        mic_capture_contact_set_status_callback(s_contact_oled_status_callback);
#if SHOW_RESET_CAUSE_ON_OLED
        char boot_msg[64];
        snprintf(boot_msg, sizeof(boot_msg), "Boot\n%s", s_reset_reason_name(esp_reset_reason()));
        oled_ssd1306_display_text(boot_msg);
        vTaskDelay(pdMS_TO_TICKS(1500));
#endif
    }
    ESP_LOGI(TAG, "Bring-up: oled_ssd1306_init done");

#if ENABLE_CONTACT_MIC_RECORDING
    ESP_LOGI(TAG, "Bring-up: mic_capture_contact_init begin");
    if (s_contact_oled_ready) {
        s_oled_show_contact_status(true, "codec boot init");
    }
    ret = mic_capture_contact_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Contact codec init failed (%s)", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Bring-up: mic_capture_contact_init done");
#endif

    ESP_LOGI(TAG, "Bring-up: button_init begin");
    button_init();
    ESP_LOGI(TAG, "Bring-up: button_init done");

    ESP_LOGI(TAG, "Bring-up: ble_trigger_init begin");
    ble_trigger_init();
    ESP_LOGI(TAG, "Bring-up: ble_trigger_init done");

    sdmmc_card_t *card = NULL;
    ESP_LOGI(TAG, "Bring-up: s_storage_init_sdmmc begin");
    ret = s_storage_init_sdmmc(&card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SD card (%s)", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Bring-up: s_storage_init_sdmmc done");
    s_append_event_log("boot: reset_reason=%d", (int)esp_reset_reason());

    tinyusb_msc_storage_config_t storage_cfg = {
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB,
        .fat_fs = {
            .base_path = MOUNT_POINT,
            .config.max_files = 5,
            .format_flags = 0,
        },
        .medium.card = card,
    };

    ESP_LOGI(TAG, "Bring-up: tinyusb_msc_new_storage_sdmmc begin");
    ESP_ERROR_CHECK(tinyusb_msc_new_storage_sdmmc(&storage_cfg, &s_storage_hdl));
    ESP_LOGI(TAG, "Bring-up: tinyusb_msc_new_storage_sdmmc done");

    s_tusb_cfg = (tinyusb_config_t)TINYUSB_DEFAULT_CONFIG();
    s_tusb_cfg.descriptor.device = &descriptor_config;
    s_tusb_cfg.descriptor.full_speed_config = msc_fs_configuration_desc;
    s_tusb_cfg.descriptor.string = string_desc_arr;
    s_tusb_cfg.descriptor.string_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]);
#if (TUD_OPT_HIGH_SPEED)
    s_tusb_cfg.descriptor.high_speed_config = msc_hs_configuration_desc;
    s_tusb_cfg.descriptor.qualifier = &device_qualifier;
#endif

    ESP_LOGI(TAG, "Bring-up: s_switch_mount APP begin");
    ESP_ERROR_CHECK(s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_APP));
    ESP_LOGI(TAG, "Bring-up: s_switch_mount APP done");
#if ENABLE_TINYUSB_MSC
    ESP_LOGI(TAG, "Bring-up: s_usb_start begin");
    ESP_ERROR_CHECK(s_usb_start());
    ESP_LOGI(TAG, "Bring-up: s_usb_start done");
    ESP_LOGI(TAG, "Exposing SD card over USB");
    ESP_LOGI(TAG, "Bring-up: s_switch_mount USB begin");
    ESP_ERROR_CHECK(s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_USB));
    ESP_LOGI(TAG, "Bring-up: s_switch_mount USB done");
#endif

    uint32_t file_index = 1;
    uint32_t consecutive_record_failures = 0;
    while (true) {
        while (!button_is_recording()) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        s_append_event_log("main: recording loop start");

        ESP_LOGI(TAG, "Disabling USB and mounting SD card for recording");
        s_usb_stop();
        ret = s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_APP);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to mount to app (%s)", esp_err_to_name(ret));
            s_handle_record_failure("Recording failed", "sd mount failed",
                                    "switch mount app failed", &consecutive_record_failures);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        char mic_path[EXAMPLE_MAX_CHAR_SIZE];
        char contact_path[EXAMPLE_MAX_CHAR_SIZE];
        char video_path[EXAMPLE_MAX_CHAR_SIZE];
        char timestamp[16];
        char contact_timestamp[16];
        bool use_index_name = true;
        if (ble_trigger_get_timestamp(timestamp, sizeof(timestamp))) {
            // 8.3-safe names using YYMMDDHH (hour resolution) for both media types.
            snprintf(mic_path, sizeof(mic_path), MOUNT_POINT"/%s.WAV", timestamp);
            snprintf(contact_timestamp, sizeof(contact_timestamp), "%s", timestamp);
            if (contact_timestamp[0] != '\0') {
                contact_timestamp[0] = 'C';
            }
            snprintf(contact_path, sizeof(contact_path), MOUNT_POINT"/%s.WAV", contact_timestamp);
            snprintf(video_path, sizeof(video_path), MOUNT_POINT"/%s.MJP", timestamp);
            use_index_name = false;
        } else {
            snprintf(mic_path, sizeof(mic_path), MOUNT_POINT"/mic_%04u.wav", (unsigned)file_index);
            snprintf(contact_path, sizeof(contact_path), MOUNT_POINT"/CT%04u.WAV", (unsigned)file_index);
            // Use 8.3-compatible name to avoid FATFS EINVAL when LFN is disabled.
            snprintf(video_path, sizeof(video_path), MOUNT_POINT"/VID%04u.MJP", (unsigned)file_index);
        }

        if (!s_wait_for_mount(MOUNT_POINT, 2000)) {
            ESP_LOGE(TAG, "Mount not ready for %s", MOUNT_POINT);
            s_handle_record_failure("Recording failed", "sd not ready",
                                    "mount path not ready", &consecutive_record_failures);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (camera_app_is_ready() && !camera_app_is_recording()) {
            esp_err_t cam_ret = camera_app_start_record(video_path);
            if (cam_ret != ESP_OK && !use_index_name) {
                ESP_LOGW(TAG, "Timestamped video name failed (%s); using index", esp_err_to_name(cam_ret));
                snprintf(video_path, sizeof(video_path), MOUNT_POINT"/VID%04u.MJP", (unsigned)file_index);
                use_index_name = true;
                cam_ret = camera_app_start_record(video_path);
            }
            if (cam_ret != ESP_OK) {
                ESP_LOGE(TAG, "Camera record start failed (%s)", esp_err_to_name(cam_ret));
                s_append_event_log("main: camera_app_start_record failed %s", esp_err_to_name(cam_ret));
                s_handle_record_failure("Recording failed", "camera start failed",
                                        "camera start failed", &consecutive_record_failures);
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
        }
        int captured_seconds = 0;
        int contact_captured_seconds = 0;
        ret = mic_capture_start(mic_path, 0);
        if (ret != ESP_OK && !use_index_name) {
            ESP_LOGW(TAG, "Timestamped mic name failed (%s); using index", esp_err_to_name(ret));
            snprintf(mic_path, sizeof(mic_path), MOUNT_POINT"/mic_%04u.wav", (unsigned)file_index);
            snprintf(contact_path, sizeof(contact_path), MOUNT_POINT"/CT%04u.WAV", (unsigned)file_index);
            use_index_name = true;
            ret = mic_capture_start(mic_path, 0);
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Mic capture start failed");
            s_append_event_log("main: mic_capture_start failed %s", esp_err_to_name(ret));
            s_handle_record_failure("Recording failed", "mic start failed",
                                    "mic start failed", &consecutive_record_failures);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

#if ENABLE_CONTACT_MIC_RECORDING
        ret = mic_capture_contact_start(contact_path, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Contact mic capture start failed");
            s_append_event_log("main: mic_capture_contact_start failed %s", esp_err_to_name(ret));
            s_handle_record_failure("Recording failed", "contact start failed",
                                    "contact mic start failed", &consecutive_record_failures);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
#endif

        bool mic_stalled = false;
        bool contact_stalled = false;
        const TickType_t stall_timeout_ticks = pdMS_TO_TICKS(RECORDING_STALL_TIMEOUT_MS);
        while (mic_capture_is_running()
#if ENABLE_CONTACT_MIC_RECORDING
               || mic_capture_contact_is_running()
#endif
        ) {
            const TickType_t now = xTaskGetTickCount();
            if (mic_capture_is_running() &&
                s_progress_stalled(mic_capture_last_progress_tick(), stall_timeout_ticks, now)) {
                mic_stalled = true;
                s_append_event_log("main: mic stalled, forcing stop");
                button_force_idle();
                break;
            }
#if ENABLE_CONTACT_MIC_RECORDING
            if (mic_capture_contact_is_running() &&
                s_progress_stalled(mic_capture_contact_last_progress_tick(), stall_timeout_ticks, now)) {
                contact_stalled = true;
                s_append_event_log("main: contact stalled, forcing stop");
                button_force_idle();
                break;
            }
#endif
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        ret = mic_capture_wait(&captured_seconds, pdMS_TO_TICKS(1000));
        esp_err_t contact_ret = ESP_OK;
#if ENABLE_CONTACT_MIC_RECORDING
        contact_ret = mic_capture_contact_wait(&contact_captured_seconds, pdMS_TO_TICKS(1000));
#endif
        if (mic_stalled || ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Mic capture stalled");
            s_append_event_log("main: mic capture stall timeout");
            s_handle_record_failure("mic stalled", s_basename_or_default(mic_path, "unnamed file"),
                                    "mic capture stalled", &consecutive_record_failures);
#if ENABLE_CONTACT_MIC_RECORDING
        } else if (contact_stalled || contact_ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Contact mic capture stalled");
            s_append_event_log("main: contact capture stall timeout");
            s_handle_record_failure("contact stalled",
                                    s_basename_or_default(mic_path, "unnamed file"),
                                    "contact mic capture stalled", &consecutive_record_failures);
#endif
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Mic capture failed");
            s_append_event_log("main: mic_capture_wait failed %s", esp_err_to_name(ret));
            if (s_file_exists(mic_path)) {
                const char *filename = s_basename_or_default(mic_path, "unnamed file");
                s_handle_record_failure(s_mic_stage_label(mic_capture_last_stage()), filename,
                                        "mic capture reported fail after file save",
                                        &consecutive_record_failures);
            } else {
                s_handle_record_failure(s_mic_stage_label(mic_capture_last_stage()), "no mic file",
                                        "mic capture failed", &consecutive_record_failures);
            }
#if ENABLE_CONTACT_MIC_RECORDING
        } else if (contact_ret != ESP_OK) {
            ESP_LOGE(TAG, "Contact mic capture failed");
            s_append_event_log("main: mic_capture_contact_wait failed %s", esp_err_to_name(contact_ret));
            s_handle_record_failure(s_contact_stage_label(mic_capture_contact_debug_last_stage()),
                                    s_basename_or_default(mic_path, "unnamed file"),
                                    "contact mic capture failed", &consecutive_record_failures);
#endif
        } else {
            char line1[32];
            const char *filename = s_basename_or_default(mic_path, "unnamed file");
            snprintf(line1, sizeof(line1), "Saved %ds in", captured_seconds);
            button_set_idle_display(line1, filename[0] != '\0' ? filename : "unnamed file");
            if (use_index_name) {
                file_index++;
            }
            consecutive_record_failures = 0;
            s_append_event_log("main: mic_capture_wait ok seconds=%d file=%s", captured_seconds, filename);
#if ENABLE_CONTACT_MIC_RECORDING
            s_append_event_log("main: contact_capture_wait ok seconds=%d file=%s",
                               contact_captured_seconds, contact_path);
#endif
        }

        camera_app_wait_for_stop();
        s_append_event_log("main: camera_app_wait_for_stop done button=%d", button_is_recording());
#if ENABLE_TINYUSB_MSC
        ESP_LOGI(TAG, "Exposing SD card over USB");
        ESP_ERROR_CHECK(s_switch_mount(TINYUSB_MSC_STORAGE_MOUNT_USB));
        ESP_ERROR_CHECK(s_usb_start());
#endif
        s_append_event_log("main: returned to idle");
    }
}
