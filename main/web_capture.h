#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

esp_err_t web_capture_start(void);
bool web_capture_client_connected(void);
bool web_capture_send(const void *data, size_t len);
