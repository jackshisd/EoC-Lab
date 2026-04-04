#pragma once

#include <stdbool.h>

void button_init(void);
bool button_is_paused(void);
bool button_is_recording(void);
void button_set_idle_display(const char *line1, const char *line2);
void button_force_idle(void);
void button_trigger_short_press(void);
void button_trigger_long_press(void);
