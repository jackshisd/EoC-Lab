#ifndef CAMERA_OV2640_H
#define CAMERA_OV2640_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    int pin_d0;
    int pin_d1;
    int pin_d2;
    int pin_d3;
    int pin_d4;
    int pin_d5;
    int pin_d6;
    int pin_d7;
    int pin_pclk;
    int pin_vsync;
    int pin_href;
    int pin_xclk;
    int pin_sda;
    int pin_scl;
    int pin_pwdn;
    int pin_reset;
} camera_ov2640_pins_t;

void camera_ov2640_get_default_pins(camera_ov2640_pins_t *pins);

void camera_app_log_i2c_levels(void);
esp_err_t camera_app_init(void);
bool camera_app_is_ready(void);
bool camera_app_is_recording(void);
esp_err_t camera_app_start_record(const char *path);
void camera_app_wait_for_stop(void);

#ifdef __cplusplus
}
#endif

#endif
