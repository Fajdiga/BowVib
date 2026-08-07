/*
 * usb.c — USB-Serial-JTAG command shell + binary dump.
 *
 * Line-based ASCII commands in, framed binary out (DUMP). Same COM port the
 * PC tool (tools/capture.py) talks to. Text responses use usb_printf; the
 * multi-MB DUMP payload uses usb_send in chunks.
 */
#include "usb.h"
#include "capture.h"
#include "iis3dwb10is.h"

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "usb";

static SemaphoreHandle_t s_tx_lock;
static iis3dwb10is_odr_t s_odr = IIS3DWB10IS_ODR_20K;
static iis3dwb10is_fs_t  s_fs  = IIS3DWB10IS_FS_200G;

/* ---- thread-safe writers ---------------------------------------------- */
void usb_send(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    size_t off = 0;
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    while (off < len) {
        size_t chunk = len - off > 4096 ? 4096 : len - off;
        size_t w = usb_serial_jtag_write_bytes(p + off, chunk, pdMS_TO_TICKS(1000));
        if (w == 0) {
            break;   /* timeout: drop the rest */
        }
        off += w;
    }
    xSemaphoreGive(s_tx_lock);
}

int usb_printf(const char *fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    usb_send(buf, strlen(buf));
    return r;
}

/* ---- little-endian pack helpers for the DUMP frame -------------------- */
static void put_u16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

static uint32_t odr_hz(iis3dwb10is_odr_t o)
{
    switch (o) {
    case IIS3DWB10IS_ODR_40K: return 40000;
    case IIS3DWB10IS_ODR_80K: return 80000;
    case IIS3DWB10IS_ODR_20K: return 20000;
    default:                  return 0;
    }
}

static void do_dump(void)
{
    uint32_t bytes = capture_bytes();
    if (bytes == 0) {
        usb_printf("ERR nothing to dump (run START first)\n");
        return;
    }

    uint8_t hdr[32];
    memset(hdr, 0, sizeof(hdr));
    hdr[0] = 'B'; hdr[1] = 'O'; hdr[2] = 'W'; hdr[3] = 'V';
    put_u16(&hdr[4], 1);                       /* version */
    put_u32(&hdr[6], odr_hz(capture_last_odr()));
    put_u16(&hdr[10], (uint16_t)(capture_last_fs() == IIS3DWB10IS_FS_50G ? 50 :
                                 capture_last_fs() == IIS3DWB10IS_FS_100G ? 100 : 200));
    put_u16(&hdr[12], FIFO_ROW_BYTES);
    put_u32(&hdr[14], capture_rows());         /* sample_count */
    put_u16(&hdr[18], capture_overrun() ? 1 : 0);
    /* hdr[20..31] reserved = 0 */

    usb_send(hdr, sizeof(hdr));                /* 32-byte header */
    usb_send(capture_buf(), bytes);            /* raw rows */
}

static void do_status(void)
{
    usb_printf("STATUS running=%d odr_hz=%lu fs=%u psram=%lu cap_bytes=%lu "
               "rows=%lu overrun=%d mglsb=%.3f\n",
               (int)capture_running(),
               (unsigned long)odr_hz(capture_last_odr()),
               (unsigned)(capture_last_fs() == IIS3DWB10IS_FS_50G ? 50 :
                          capture_last_fs() == IIS3DWB10IS_FS_100G ? 100 : 200),
               (unsigned long)capture_psram_size(),
               (unsigned long)capture_capacity(),
               (unsigned long)capture_rows(), (int)capture_overrun(),
               iis3dwb10is_fs_to_mglsb(s_fs));
}

static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t");
    if (!cmd) {
        return;
    }
    if (!strcmp(cmd, "START")) {
        capture_start_store(s_odr, s_fs);
    } else if (!strcmp(cmd, "STOP")) {
        capture_stop();
    } else if (!strcmp(cmd, "STREAM")) {
        capture_start_stream(s_odr, s_fs);
    } else if (!strcmp(cmd, "DUMP")) {
        do_dump();
    } else if (!strcmp(cmd, "STATUS")) {
        do_status();
    } else if (!strcmp(cmd, "SET")) {
        char *k = strtok(NULL, " \t");
        char *v = strtok(NULL, " \t");
        if (k && v) {
            long n = strtol(v, NULL, 10);
            if (!strcmp(k, "ODR")) {
                if      (n == 20000) s_odr = IIS3DWB10IS_ODR_20K;
                else if (n == 40000) s_odr = IIS3DWB10IS_ODR_40K;
                else if (n == 80000) s_odr = IIS3DWB10IS_ODR_80K;
                else { usb_printf("ERR ODR must be 20000|40000|80000\n"); return; }
                usb_printf("OK ODR=%ld (applies on next START)\n", n);
            } else if (!strcmp(k, "FS")) {
                if      (n ==  50) s_fs = IIS3DWB10IS_FS_50G;
                else if (n == 100) s_fs = IIS3DWB10IS_FS_100G;
                else if (n == 200) s_fs = IIS3DWB10IS_FS_200G;
                else { usb_printf("ERR FS must be 50|100|200\n"); return; }
                usb_printf("OK FS=%ld g (applies on next START)\n", n);
            } else {
                usb_printf("ERR SET needs ODR or FS\n");
            }
        } else {
            usb_printf("ERR SET ODR|FS <value>\n");
        }
    } else if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) {
        usb_printf("BowVib cmds: START STOP DUMP STREAM STATUS "
                   "SET ODR 20000|40000|80000 SET FS 50|100|200 HELP\n");
    } else {
        usb_printf("ERR unknown cmd '%s' (HELP for list)\n", cmd);
    }
}

static void usb_task(void *arg)
{
    (void)arg;
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 4096,
        .rx_buffer_size = 4096,
    };
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed");
        vTaskDelete(NULL);
    }

    usb_printf("\nBowVib ready. IIS3DWB10IS SPI capture. cmds: START STOP DUMP "
               "STREAM STATUS HELP\n");
    do_status();

    char line[64];
    int pos = 0;
    while (1) {
        uint8_t b;
        int r = usb_serial_jtag_read_bytes(&b, 1, pdMS_TO_TICKS(100));
        if (r <= 0) {
            continue;
        }
        if (b == '\r' || b == '\n') {
            if (pos > 0) {
                line[pos] = '\0';
                handle_line(line);
            }
            pos = 0;
        } else if (pos < (int)sizeof(line) - 1) {
            line[pos++] = (char)b;
        }
    }
}

void usb_start(void)
{
    s_tx_lock = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(usb_task, "usb", 4096, NULL, 5, NULL, 0);
}
