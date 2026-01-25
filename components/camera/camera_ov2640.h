#ifndef CAMERA_OV2640_H
#define CAMERA_OV2640_H

#ifdef __cplusplus
extern "C" {
#endif

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
} camera_ov2640_pins_t;

void camera_ov2640_get_default_pins(camera_ov2640_pins_t *pins);

#ifdef __cplusplus
}
#endif

#endif
