idf_component_register(SRCS "oled_ssd1306.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver i2c_bus)
