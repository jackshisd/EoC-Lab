#include "i2c_bus.h"

#include "esp_log.h"
#include "freertos/semphr.h"

static const char *TAG = "i2c_bus";

static SemaphoreHandle_t s_i2c_mutex;
static bool s_i2c_inited;
static i2c_port_t s_i2c_port = I2C_NUM_0;

esp_err_t i2c_bus_init(i2c_port_t port, int sda, int scl, uint32_t clk_hz)
{
    if (s_i2c_inited) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_hz,
    };

    esp_err_t ret = i2c_param_config(port, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    s_i2c_mutex = xSemaphoreCreateMutex();
    if (!s_i2c_mutex) {
        return ESP_ERR_NO_MEM;
    }

    s_i2c_port = port;
    s_i2c_inited = true;
    ESP_LOGI(TAG, "I2C bus init on port %d SDA=%d SCL=%d @%u Hz", port, sda, scl, (unsigned)clk_hz);
    return ESP_OK;
}

bool i2c_bus_is_init(void)
{
    return s_i2c_inited;
}

i2c_port_t i2c_bus_get_port(void)
{
    return s_i2c_port;
}

esp_err_t i2c_bus_lock(TickType_t timeout)
{
    if (!s_i2c_inited || !s_i2c_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTake(s_i2c_mutex, timeout) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

void i2c_bus_unlock(void)
{
    if (s_i2c_mutex) {
        xSemaphoreGive(s_i2c_mutex);
    }
}
