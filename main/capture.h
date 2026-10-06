#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t capture_init(void);
/* Standalone shots end at flash capacity rather than a fixed timeout. */
/* Serialize USB and TCP command handlers, including board diagnostics. */
bool capture_control_lock(void);
void capture_control_unlock(void);
bool capture_start(void);
/* Discard samples after checking timing; retained flash recording is untouched. */
bool capture_start_timing_test(uint32_t seconds);
bool capture_start_direct_test(uint32_t seconds);
bool capture_start_wifi(void);
bool capture_start_browser(void);
bool capture_start_shot(uint32_t pre_ms, uint32_t post_ms, uint32_t threshold_mg, const char *metadata);
bool capture_trigger_shot(void);
bool capture_shot_active(void);
bool capture_shot_ready(void);
uint32_t capture_shot_elapsed_ms(void);
const char *capture_shot_state(void);
int capture_shot_error(void);
void capture_stop(void);
bool capture_running(void);
bool capture_wifi_active(void);
bool capture_browser_active(void);
uint8_t capture_overrun_mask(void);
