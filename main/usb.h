/*
 * usb.h — USB-Serial-JTAG command + binary-data channel.
 * Console logging is disabled; UART0 pins are used for sensor chip-selects.
 */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t usb_start(void); /* install driver and spawn command task */

/* Thread-safe writes (safe to call from capture task too). */
bool usb_send(const void *data, size_t len);
int  usb_printf(const char *fmt, ...);
