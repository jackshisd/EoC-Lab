#include "ble_trigger.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#endif

#define ENABLE_BLE_TRIGGER 1
#define BLE_TRIGGER_DEBOUNCE_MS 500
#define BLE_SCAN_INTERVAL_UNITS 0x0080
#define BLE_SCAN_WINDOW_UNITS   0x0008

static const char *TAG = "ble_trigger";

#if CONFIG_BT_NIMBLE_ENABLED
static uint8_t s_ble_own_addr_type;
static int64_t s_ble_last_trigger_us = 0;
static portMUX_TYPE s_ble_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_ble_timestamp[32] = {0};
static bool s_ble_timestamp_valid = false;
static const uint8_t s_ble_prefix_be[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static bool s_ble_uuid_match_prefix(const uint8_t be[16])
{
    for (int i = 0; i < 8; ++i) {
        if (be[i] != s_ble_prefix_be[i]) {
            return false;
        }
    }
    return true;
}

static bool s_ble_extract_trigger(const ble_uuid128_t *adv_uuid, bool *out_long, char *out_ts, size_t out_ts_size)
{
    if (out_ts_size < 9) {
        return false;
    }

    uint8_t be[16];
    for (int i = 0; i < 16; ++i) {
        be[i] = adv_uuid->value[15 - i];
    }

    if (!s_ble_uuid_match_prefix(be)) {
        return false;
    }

    if (be[8] == 0x01) {
        *out_long = false;
    } else if (be[8] == 0x02) {
        *out_long = true;
    } else {
        return false;
    }

    char digits[13];
    int pos = 0;
    for (int i = 10; i < 16; ++i) {
        int hi = (be[i] >> 4) & 0x0F;
        int lo = be[i] & 0x0F;
        if (hi > 9 || lo > 9) {
            return false;
        }
        digits[pos++] = (char)('0' + hi);
        digits[pos++] = (char)('0' + lo);
    }
    digits[12] = '\0';

    // Build an 8.3-safe timestamp: MMDDHHMM (month/day/hour/minute).
    snprintf(out_ts, out_ts_size, "%.8s", digits + 2);
    return true;
}

static void s_ble_store_timestamp(const char *timestamp)
{
    portENTER_CRITICAL(&s_ble_lock);
    strncpy(s_ble_timestamp, timestamp, sizeof(s_ble_timestamp) - 1);
    s_ble_timestamp[sizeof(s_ble_timestamp) - 1] = '\0';
    s_ble_timestamp_valid = true;
    portEXIT_CRITICAL(&s_ble_lock);
}

static void s_ble_handle_trigger(bool is_long)
{
    int64_t now_us = esp_timer_get_time();
    if ((now_us - s_ble_last_trigger_us) < (BLE_TRIGGER_DEBOUNCE_MS * 1000LL)) {
        return;
    }
    s_ble_last_trigger_us = now_us;
    if (is_long) {
        button_trigger_long_press_with_reason("BLE stop");
    } else {
        button_trigger_short_press();
    }
}

static int s_ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_DISC) {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
            return 0;
        }
        bool found_short = false;
        bool found_long = false;
        for (int i = 0; i < fields.num_uuids128; ++i) {
            bool is_long = false;
            char timestamp[32];
            if (s_ble_extract_trigger(&fields.uuids128[i], &is_long, timestamp, sizeof(timestamp))) {
                s_ble_store_timestamp(timestamp);
                if (is_long) {
                    found_long = true;
                } else {
                    found_short = true;
                }
            }
        }
        if (found_long) {
            s_ble_handle_trigger(true);
        } else if (found_short) {
            s_ble_handle_trigger(false);
        }
    }
    return 0;
}

static void s_ble_start_scan(void)
{
    struct ble_gap_disc_params params = {
        .itvl = BLE_SCAN_INTERVAL_UNITS,
        .window = BLE_SCAN_WINDOW_UNITS,
        .filter_duplicates = 0,
        .passive = 1,
    };
    int rc = ble_gap_disc(s_ble_own_addr_type, BLE_HS_FOREVER, &params, s_ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE scan start failed (%d)", rc);
    }
}

static void s_ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_ble_own_addr_type);
    s_ble_start_scan();
}

static void s_ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}
#endif

void ble_trigger_init(void)
{
#if CONFIG_BT_NIMBLE_ENABLED && ENABLE_BLE_TRIGGER
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed (%s)", esp_err_to_name(err));
        return;
    }

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed (%s)", esp_err_to_name(err));
        return;
    }
    ble_hs_cfg.sync_cb = s_ble_on_sync;
    nimble_port_freertos_init(s_ble_host_task);
    ESP_LOGI(TAG, "BLE trigger scan started");
#elif CONFIG_BT_NIMBLE_ENABLED
    ESP_LOGW(TAG, "BLE trigger disabled by ENABLE_BLE_TRIGGER=0");
#else
    ESP_LOGW(TAG, "BLE trigger disabled (CONFIG_BT_NIMBLE_ENABLED=0)");
#endif
}

bool ble_trigger_get_timestamp(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) {
        return false;
    }
    bool have_ts = false;
    portENTER_CRITICAL(&s_ble_lock);
    if (s_ble_timestamp_valid) {
        strncpy(out, s_ble_timestamp, out_size - 1);
        out[out_size - 1] = '\0';
        s_ble_timestamp_valid = false;
        have_ts = true;
    }
    portEXIT_CRITICAL(&s_ble_lock);
    return have_ts;
}
