#include "mic_capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <stdarg.h>

#include "esp_log.h"
#include "button.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "oled_ssd1306.h"

#define I2S_SAMPLE_RATE_HZ 16000 // Sample rate
#define I2S_BCLK_IO        38 // Bit clock
#define I2S_WS_IO          40 // Also known as LRCK
#define I2S_DIN_IO         39 // Microphone data input
#define MIC_GAIN_MULT      2  // Microphone gain multiplier
#define MIC_CHANNELS       2  // Stereo capture (left + right)

#define CONTACT_CODEC_I2C_PORT I2C_NUM_1
#define CONTACT_CODEC_I2C_SDA  1
#define CONTACT_CODEC_I2C_SCL  2
#define CONTACT_CODEC_I2C_HZ   100000
#define CONTACT_ES8388_ADDR0   0x10
#define CONTACT_ES8388_ADDR1   0x11
#define CONTACT_I2S_MCLK_IO    41
#define CONTACT_I2S_DIN_IO     42
#define CONTACT_I2S_BCLK_IO    43
#define CONTACT_I2S_WS_IO      44
#define CONTACT_BITS_PER_SAMPLE 16
#define CONTACT_MIC_CHANNELS   1

static const char *TAG = "mic";

typedef struct {
    char path[128];
    int seconds;
} mic_capture_args_t;

static TaskHandle_t s_mic_task = NULL;
static volatile bool s_mic_running = false;
static volatile int s_mic_last_seconds = 0;
static volatile esp_err_t s_mic_last_result = ESP_OK;

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

// Applies software gain with clipping.
static int32_t s_apply_gain(int32_t sample)
{
#if MIC_GAIN_MULT > 1
    int64_t v = (int64_t)sample * MIC_GAIN_MULT;
    if (v > INT32_MAX) {
        return INT32_MAX;
    }
    if (v < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)v;
#else
    return sample;
#endif
}

// Logs an info message (and optionally OLED if enabled).
static void s_log_info(const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ESP_LOGI(TAG, "%s", buf);
    //oled_ssd1306_display_text(buf);
}

// Logs an error message (and shows it on OLED).
static void s_log_error(const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ESP_LOGE(TAG, "%s", buf);
    oled_ssd1306_display_text(buf);
}

static void s_mic_task_entry(void *arg)
{
    mic_capture_args_t *args = (mic_capture_args_t *)arg;
    int captured_seconds = 0;
    esp_err_t result = mic_capture_to_file(args->path, args->seconds, &captured_seconds);
    s_mic_last_seconds = captured_seconds;
    s_mic_last_result = result;
    s_mic_running = false;
    s_mic_task = NULL;
    free(args);
    vTaskDelete(NULL);
}

esp_err_t mic_capture_start(const char *path, int seconds)
{
    if (s_mic_running) {
        return ESP_ERR_INVALID_STATE;
    }

    mic_capture_args_t *args = (mic_capture_args_t *)calloc(1, sizeof(*args));
    if (args == NULL) {
        return ESP_ERR_NO_MEM;
    }

    strlcpy(args->path, path, sizeof(args->path));
    args->seconds = seconds;
    s_mic_running = true;
    s_mic_last_seconds = 0;
    s_mic_last_result = ESP_OK;

    if (xTaskCreate(s_mic_task_entry, "mic_record", 4096, args, 5, &s_mic_task) != pdPASS) {
        s_mic_running = false;
        free(args);
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool mic_capture_is_running(void)
{
    return s_mic_running;
}

esp_err_t mic_capture_wait(int *out_seconds, TickType_t timeout)
{
    const TickType_t start = xTaskGetTickCount();
    while (s_mic_running) {
        if (timeout != portMAX_DELAY && (xTaskGetTickCount() - start) > timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (out_seconds != NULL) {
        *out_seconds = s_mic_last_seconds;
    }
    return s_mic_last_result;
}

// Writes a 16-bit little-endian value to a file.
static void s_write_le16(FILE *f, uint16_t value)
{
    uint8_t b[2] = {value & 0xff, (value >> 8) & 0xff};
    fwrite(b, 1, sizeof(b), f);
}

// Writes a 32-bit little-endian value to a file.
static void s_write_le32(FILE *f, uint32_t value)
{
    uint8_t b[4] = {
        value & 0xff,
        (value >> 8) & 0xff,
        (value >> 16) & 0xff,
        (value >> 24) & 0xff
    };
    fwrite(b, 1, sizeof(b), f);
}

// Checks if the path ends with .wav.
static bool s_has_wav_extension(const char *path)
{
    const char *dot = strrchr(path, '.');
    if (dot == NULL) {
        return false;
    }
    if (strlen(dot) != 4) {
        return false;
    }
    return (tolower((unsigned char)dot[1]) == 'w' &&
            tolower((unsigned char)dot[2]) == 'a' &&
            tolower((unsigned char)dot[3]) == 'v');
}

// Writes/updates a PCM WAV header.
static void s_write_wav_header(FILE *f, uint32_t sample_rate_hz, uint16_t bits_per_sample,
                               uint16_t channels, uint32_t data_bytes)
{
    const uint32_t byte_rate = sample_rate_hz * channels * (bits_per_sample / 8);
    const uint16_t block_align = channels * (bits_per_sample / 8);
    const uint32_t riff_size = 36 + data_bytes;

    fwrite("RIFF", 1, 4, f);
    s_write_le32(f, riff_size);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    s_write_le32(f, 16);
    s_write_le16(f, 1);
    s_write_le16(f, channels);
    s_write_le32(f, sample_rate_hz);
    s_write_le32(f, byte_rate);
    s_write_le16(f, block_align);
    s_write_le16(f, bits_per_sample);
    fwrite("data", 1, 4, f);
    s_write_le32(f, data_bytes);
}

// Captures I2S audio to a file; stops on button or after N seconds.
esp_err_t mic_capture_to_file(const char *path, int seconds, int *out_seconds)
{
    esp_err_t ret;
    i2s_chan_handle_t rx_handle = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        s_log_error("I2S channel create (%s)", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DIN_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        s_log_error("I2S init std mode (%s)", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        return ret;
    }

    ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        s_log_error("I2S enable (%s)", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        return ret;
    }

    s_log_info("Waiting for long press");
    while (!button_is_recording()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    s_log_info("Recording started");

    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        s_log_error("Open failed %s (%d)", path, errno);
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        return ESP_FAIL;
    }

    const bool write_wav = s_has_wav_extension(path);
    const bool stop_on_button = (seconds <= 0);
    if (!stop_on_button && seconds < 1) {
        seconds = 1;
    }

    const size_t bytes_per_sample = 4;
    const size_t bytes_per_frame = bytes_per_sample * MIC_CHANNELS;
    const size_t frames_per_chunk = 512;
    const int flush_interval_ms = 1000;
    uint32_t next_flush_ms = flush_interval_ms;
    const size_t chunk_bytes = frames_per_chunk * bytes_per_frame;
    uint8_t *buffer = (uint8_t *)malloc(chunk_bytes);
    if (buffer == NULL) {
        s_log_error("Audio buffer alloc failed");
        fclose(f);
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        return ESP_ERR_NO_MEM;
    }

    size_t total_frames = stop_on_button ? SIZE_MAX : (size_t)I2S_SAMPLE_RATE_HZ * (size_t)seconds;
    size_t captured_frames = 0;
    if (write_wav) {
        s_write_wav_header(f, I2S_SAMPLE_RATE_HZ, 32, MIC_CHANNELS, 0);
    }
    while (captured_frames < total_frames) {
        if (stop_on_button && !button_is_recording()) {
            s_log_info("Stop requested");
            break;
        }
        size_t bytes_read = 0;
        size_t frames_to_read = frames_per_chunk;
        if (captured_frames + frames_to_read > total_frames) {
            frames_to_read = total_frames - captured_frames;
        }
        size_t bytes_to_read = frames_to_read * bytes_per_frame;

        ret = i2s_channel_read(rx_handle, buffer, bytes_to_read, &bytes_read, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            s_log_error("I2S read failed (%s)", esp_err_to_name(ret));
            s_append_event_log("mic error: i2s_read %s", esp_err_to_name(ret));
            break;
        }
        if (bytes_read > 0) {
            if (button_is_paused()) {
                memset(buffer, 0, bytes_read);
            } else {
                int32_t *samples = (int32_t *)buffer;
                size_t count = bytes_read / sizeof(int32_t);
                for (size_t i = 0; i < count; ++i) {
                    samples[i] = s_apply_gain(samples[i]);
                }
            }
            fwrite(buffer, 1, bytes_read, f);
            captured_frames += bytes_read / bytes_per_frame;
        }

        if (write_wav && (captured_frames * 1000 / I2S_SAMPLE_RATE_HZ) >= next_flush_ms) {
            const uint32_t data_bytes = (uint32_t)(captured_frames * bytes_per_frame);
            fflush(f);
            fsync(fileno(f));
            fseek(f, 0, SEEK_SET);
            s_write_wav_header(f, I2S_SAMPLE_RATE_HZ, 32, MIC_CHANNELS, data_bytes);
            fseek(f, 0, SEEK_END);
            next_flush_ms += flush_interval_ms;
        }
    }

    if (write_wav) {
        const uint32_t data_bytes = (uint32_t)(captured_frames * bytes_per_frame);
        fseek(f, 0, SEEK_SET);
        s_write_wav_header(f, I2S_SAMPLE_RATE_HZ, 32, MIC_CHANNELS, data_bytes);
    }

    free(buffer);
    fclose(f);
    i2s_channel_disable(rx_handle);
    i2s_del_channel(rx_handle);

    int captured_seconds = (int)(captured_frames / I2S_SAMPLE_RATE_HZ);
    if (out_seconds != NULL) {
        *out_seconds = captured_seconds;
    }
    s_log_info("Captured %d sec to %s", captured_seconds, path);
    s_append_event_log("mic stop: ret=%s seconds=%d button=%d path=%s",
                       esp_err_to_name(ret), captured_seconds, button_is_recording(), path);
    return ret;
}

static bool s_contact_codec_i2c_ready = false;
static uint8_t s_contact_codec_addr = 0;
static const char *s_contact_last_stage = "idle";
static int s_contact_last_reg = -1;
static void (*s_contact_status_callback)(const char *status) = NULL;

int mic_capture_contact_debug_last_reg(void)
{
    return s_contact_last_reg;
}

void mic_capture_contact_set_status_callback(void (*callback)(const char *status))
{
    s_contact_status_callback = callback;
}

static int16_t s_apply_contact_gain(int16_t sample)
{
#if MIC_GAIN_MULT > 1
    int32_t v = (int32_t)sample * MIC_GAIN_MULT;
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)v;
#else
    return sample;
#endif
}

static esp_err_t s_contact_codec_i2c_init(void)
{
    if (s_contact_codec_i2c_ready) {
        return ESP_OK;
    }

    if (!i2c_bus_is_init()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (i2c_bus_get_port() != CONTACT_CODEC_I2C_PORT) {
        return ESP_ERR_INVALID_STATE;
    }

    s_contact_codec_i2c_ready = true;
    return ESP_OK;
}

static esp_err_t s_contact_codec_write_reg(uint8_t addr_7bit, uint8_t reg, uint8_t data)
{
    uint8_t payload[2] = {reg, data};
    esp_err_t ret = i2c_bus_lock(pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_write_to_device(i2c_bus_get_port(), addr_7bit, payload, sizeof(payload), pdMS_TO_TICKS(100));
    i2c_bus_unlock();
    return ret;
}

static esp_err_t s_contact_es8388_init_at_addr(uint8_t addr_7bit)
{
    static const uint8_t init_seq[][2] = {
        {0x19, 0x04},
        {0x01, 0x50},
        {0x02, 0x00},
        {0x35, 0xA0},
        {0x37, 0xD0},
        {0x39, 0xD0},
        {0x08, 0x00},
        {0x04, 0xC0},
        {0x00, 0x12},
        {0x17, 0x18},
        {0x18, 0x02},
        {0x26, 0x00},
        {0x27, 0x90},
        {0x2A, 0x90},
        {0x2B, 0x80},
        {0x2D, 0x00},
        {0x2E, 0x1E},
        {0x2F, 0x1E},
        {0x30, 0x00},
        {0x31, 0x00},
        {0x04, 0x3C},
        {0x03, 0xFF},
        {0x09, 0xBB},
        {0x0A, 0x00},
        {0x0B, 0x02},
        {0x0C, 0x0C},
        {0x0D, 0x02},
        {0x10, 0x00},
        {0x11, 0x00},
        {0x03, 0x09},
    };

    s_contact_last_reg = -1;
    for (size_t i = 0; i < sizeof(init_seq) / sizeof(init_seq[0]); ++i) {
        s_contact_last_reg = init_seq[i][0];
        esp_err_t ret = ESP_FAIL;
        int attempts = (i == 0) ? 3 : 1;
        for (int attempt = 0; attempt < attempts; ++attempt) {
            ret = s_contact_codec_write_reg(addr_7bit, init_seq[i][0], init_seq[i][1]);
            if (ret == ESP_OK) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    s_contact_last_reg = -1;
    return ESP_OK;
}

static esp_err_t s_contact_es8388_init(void)
{
    esp_err_t ret = s_contact_codec_i2c_init();
    if (ret != ESP_OK) {
        return ret;
    }

    const uint8_t codec_addrs[] = {CONTACT_ES8388_ADDR0, CONTACT_ES8388_ADDR1};
    for (int attempt = 0; attempt < 10; ++attempt) {
        for (size_t i = 0; i < sizeof(codec_addrs) / sizeof(codec_addrs[0]); ++i) {
            ret = s_contact_es8388_init_at_addr(codec_addrs[i]);
            if (ret == ESP_OK) {
                s_contact_codec_addr = codec_addrs[i];
                ESP_LOGI(TAG, "ES8388 contact codec initialized at 0x%02X on attempt %d",
                         s_contact_codec_addr, attempt + 1);
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    return ret;
}

static esp_err_t s_contact_es8388_init_until_ready(void)
{
    esp_err_t ret = ESP_FAIL;
    int retry_count = 0;
    while (true) {
        ret = s_contact_es8388_init();
        if (ret == ESP_OK) {
            if (s_contact_status_callback != NULL) {
                s_contact_status_callback("codec ok");
            }
            return ESP_OK;
        }
        retry_count++;
        if (s_contact_status_callback != NULL) {
            char status[32];
            snprintf(status, sizeof(status), "retry %d reg %02X",
                     retry_count,
                     (s_contact_last_reg >= 0) ? s_contact_last_reg : 0);
            s_contact_status_callback(status);
        }
        ESP_LOGW(TAG, "ES8388 init retry: %s reg=0x%02X",
                 esp_err_to_name(ret),
                 (s_contact_last_reg >= 0) ? s_contact_last_reg : 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void s_contact_write_stats(esp_err_t ret, size_t captured_frames, int captured_seconds,
                                  int32_t peak_left, int32_t peak_right,
                                  uint32_t nonzero_left, uint32_t nonzero_right)
{
    FILE *f = fopen("/sdcard/contact_stats.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to write contact_stats.txt");
        return;
    }

    fprintf(f, "result=%s\n", esp_err_to_name(ret));
    fprintf(f, "captured_seconds=%d\n", captured_seconds);
    fprintf(f, "captured_frames=%zu\n", captured_frames);
    fprintf(f, "sample_rate_hz=%d\n", I2S_SAMPLE_RATE_HZ);
    fprintf(f, "channels=%d\n", CONTACT_MIC_CHANNELS);
    fprintf(f, "bits_per_sample=%d\n", CONTACT_BITS_PER_SAMPLE);
    fprintf(f, "codec_addr_7bit=0x%02X\n", s_contact_codec_addr);
    fprintf(f, "peak_left=%ld\n", (long)peak_left);
    fprintf(f, "peak_right=%ld\n", (long)peak_right);
    fprintf(f, "nonzero_left=%lu\n", (unsigned long)nonzero_left);
    fprintf(f, "nonzero_right=%lu\n", (unsigned long)nonzero_right);
    fclose(f);
}

esp_err_t mic_capture_contact_debug_to_file(const char *path, int seconds, int *out_seconds)
{
    s_contact_last_stage = "start";
    s_contact_last_reg = -1;

    if (seconds < 1) {
        seconds = 10;
    }

    esp_err_t ret;
    i2s_chan_handle_t rx_handle = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        s_contact_last_stage = "i2s new fail";
        ESP_LOGE(TAG, "Contact I2S channel create failed (%s)", esp_err_to_name(ret));
        return ret;
    }

    s_contact_last_stage = "codec init";
    vTaskDelay(pdMS_TO_TICKS(300));
    ret = s_contact_es8388_init_until_ready();
    if (ret != ESP_OK) {
        s_contact_last_stage = "codec fail";
        ESP_LOGE(TAG, "ES8388 init failed (%s)", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        return ret;
    }

    if (s_contact_status_callback != NULL) {
        s_contact_status_callback("codec ok");
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = CONTACT_I2S_MCLK_IO,
            .bclk = CONTACT_I2S_BCLK_IO,
            .ws = CONTACT_I2S_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din = CONTACT_I2S_DIN_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    s_contact_last_stage = "i2s std init";
    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        s_contact_last_stage = "i2s init fail";
        ESP_LOGE(TAG, "Contact I2S init failed (%s)", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        return ret;
    }

    s_contact_last_stage = "i2s enable";
    ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        s_contact_last_stage = "i2s en fail";
        ESP_LOGE(TAG, "Contact I2S enable failed (%s)", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        return ret;
    }

    s_contact_last_stage = "file open";
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        s_contact_last_stage = "file fail";
        ESP_LOGE(TAG, "Failed to open %s (%d)", path, errno);
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        return ESP_FAIL;
    }

    const size_t bytes_per_sample = CONTACT_BITS_PER_SAMPLE / 8;
    const size_t bytes_per_frame = bytes_per_sample * CONTACT_MIC_CHANNELS;
    const size_t frames_per_chunk = 512;
    const size_t chunk_bytes = frames_per_chunk * bytes_per_frame;
    const size_t total_frames = (size_t)I2S_SAMPLE_RATE_HZ * (size_t)seconds;
    uint8_t *buffer = (uint8_t *)malloc(chunk_bytes);
    if (buffer == NULL) {
        s_contact_last_stage = "alloc fail";
        fclose(f);
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        return ESP_ERR_NO_MEM;
    }

    s_contact_last_stage = "recording";
    s_write_wav_header(f, I2S_SAMPLE_RATE_HZ, CONTACT_BITS_PER_SAMPLE, CONTACT_MIC_CHANNELS, 0);

    size_t captured_frames = 0;
    int32_t peak_left = 0;
    int32_t peak_right = 0;
    uint32_t nonzero_left = 0;
    uint32_t nonzero_right = 0;

    while (captured_frames < total_frames) {
        size_t bytes_read = 0;
        size_t frames_to_read = frames_per_chunk;
        if (captured_frames + frames_to_read > total_frames) {
            frames_to_read = total_frames - captured_frames;
        }
        const size_t bytes_to_read = frames_to_read * bytes_per_frame;

        ret = i2s_channel_read(rx_handle, buffer, bytes_to_read, &bytes_read, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            s_contact_last_stage = "read fail";
            ESP_LOGE(TAG, "Contact I2S read failed (%s)", esp_err_to_name(ret));
            break;
        }

        int16_t *samples = (int16_t *)buffer;
        size_t count = bytes_read / sizeof(int16_t);
        for (size_t i = 0; i < count; i++) {
            int16_t left = s_apply_contact_gain(samples[i]);
            samples[i] = left;

            int32_t abs_left = left < 0 ? -(int32_t)left : (int32_t)left;
            if (abs_left > peak_left) {
                peak_left = abs_left;
            }
            if (left != 0) {
                nonzero_left++;
            }
        }

        fwrite(buffer, 1, bytes_read, f);
        captured_frames += bytes_read / bytes_per_frame;
    }

    const uint32_t data_bytes = (uint32_t)(captured_frames * bytes_per_frame);
    fseek(f, 0, SEEK_SET);
    s_write_wav_header(f, I2S_SAMPLE_RATE_HZ, CONTACT_BITS_PER_SAMPLE, CONTACT_MIC_CHANNELS, data_bytes);

    free(buffer);
    fclose(f);
    i2s_channel_disable(rx_handle);
    i2s_del_channel(rx_handle);

    int captured_seconds = (int)(captured_frames / I2S_SAMPLE_RATE_HZ);
    if (out_seconds != NULL) {
        *out_seconds = captured_seconds;
    }

    s_contact_write_stats(ret, captured_frames, captured_seconds,
                          peak_left, peak_right, nonzero_left, nonzero_right);
    s_contact_last_stage = (ret == ESP_OK) ? "done" : s_contact_last_stage;
    ESP_LOGI(TAG, "Contact capture finished: %s, seconds=%d, peakL=%ld, peakR=%ld",
             esp_err_to_name(ret), captured_seconds, (long)peak_left, (long)peak_right);
    return ret;
}
const char *mic_capture_contact_debug_last_stage(void)
{
    return s_contact_last_stage;
}
