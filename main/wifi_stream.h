#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define BOWVIB_WIFI_SSID "BowVib-IMU"
#define BOWVIB_WIFI_PASSWORD "BowVib320x"
#define BOWVIB_WIFI_PORT 3333

esp_err_t wifi_stream_start(void);
bool wifi_stream_ready(void);
bool wifi_stream_client_connected(void);
bool wifi_stream_send(const void *data, size_t len);
bool wifi_stream_send_text(const char *text);
