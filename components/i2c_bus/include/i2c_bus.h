#pragma once

#include <stdbool.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t i2c_bus_init(i2c_port_t port, int sda, int scl, uint32_t clk_hz);
bool i2c_bus_is_init(void);
i2c_port_t i2c_bus_get_port(void);
esp_err_t i2c_bus_lock(TickType_t timeout);
void i2c_bus_unlock(void);
