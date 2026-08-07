/*
 * usb.h — USB-Serial-JTAG command + binary-data channel.
 * (Console/ESP_LOG stays on UART0; this native "USB" port is the app interface.)
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

void usb_start(void);   /* install driver, print banner, spawn command task */

/* Thread-safe writes (safe to call from capture task too). */
void usb_send(const void *data, size_t len);
int  usb_printf(const char *fmt, ...);
