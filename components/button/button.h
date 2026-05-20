#pragma once

#include <stdbool.h>
#include <stdint.h>

void button_init(void);
bool button_is_paused(void);
bool button_is_recording(void);
void button_set_idle_display(const char *line1, const char *line2);
void button_force_idle(void);
void button_trigger_short_press(void);
void button_trigger_long_press(void);
void button_trigger_long_press_with_reason(const char *reason);
uint32_t button_get_record_elapsed_seconds(void);
void button_restart_recording_with_elapsed(uint32_t elapsed_seconds);
