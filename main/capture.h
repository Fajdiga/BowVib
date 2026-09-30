#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t capture_init(void);
/* Serialize USB and TCP command handlers, including board diagnostics. */
bool capture_control_lock(void);
void capture_control_unlock(void);
bool capture_start(void);
bool capture_start_wifi(void);
bool capture_start_browser(void);
void capture_stop(void);
bool capture_running(void);
bool capture_wifi_active(void);
bool capture_browser_active(void);
uint8_t capture_overrun_mask(void);
