#include "usb.h"
#include "capture.h"
#include "led_status.h"
#include "lsm6dsv320x.h"
#include "wifi_stream.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_tx_lock;

bool usb_send(const void *data, size_t len)
{
    if (!data || !s_tx_lock) return false;
    const uint8_t *p = (const uint8_t *)data;
    size_t off = 0;
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    while (off < len) {
        const size_t chunk = len - off > 4096U ? 4096U : len - off;
        const int written = usb_serial_jtag_write_bytes(p + off, chunk,
                                                        pdMS_TO_TICKS(1000));
        if (written <= 0) break;
        off += (size_t)written;
    }
    xSemaphoreGive(s_tx_lock);
    return off == len;
}

int usb_printf(const char *fmt, ...)
{
    char buf[192];
    va_list args;
    va_start(args, fmt);
    const int written = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (written > 0) usb_send(buf, (size_t)written < sizeof(buf) ?
                              (size_t)written : sizeof(buf) - 1U);
    return written;
}

static void status(void)
{
    const uint8_t mask = lsm6dsv320x_present_mask();
    char names[8];
    char ids[32];
    char freq_fine[48];
    size_t pos = 0;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        if (mask & (1U << i)) {
            names[pos++] = (char)('A' + i);
            names[pos++] = ',';
        }
    }
    if (pos) names[--pos] = '\0';
    else { names[0] = 'n'; names[1] = 'o'; names[2] = 'n'; names[3] = 'e'; names[4] = '\0'; }
    pos = 0;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        const int n = snprintf(ids + pos, sizeof(ids) - pos, "%s%c:%02X",
                               i ? "," : "", (char)('A' + i),
                               lsm6dsv320x_whoami(i));
        if (n <= 0 || (size_t)n >= sizeof(ids) - pos) break;
        pos += (size_t)n;
    }
    pos = 0;
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        const int8_t fine = lsm6dsv320x_internal_freq_fine(i);
        const int n = snprintf(freq_fine + pos, sizeof(freq_fine) - pos,
                               "%s%c:%d", i ? "," : "",
                               (char)('A' + i), fine == INT8_MIN ? 999 : fine);
        if (n <= 0 || (size_t)n >= sizeof(freq_fine) - pos) break;
        pos += (size_t)n;
    }
    usb_printf("STATUS running=%d sensors=4 present_mask=%02X present=%s whoami=%s "
               "odr_hz=%u fs_g=%u fifo=on spi_hz=10000000 freq_fine=%s\n",
               capture_running() ? 1 : 0, mask, names, ids,
               LSM6DSV320X_ODR_HZ, LSM6DSV320X_FS_G, freq_fine);
}

static void handle_line_unlocked(char *line)
{
    char *cmd = strtok(line, " \t");
    if (!cmd) return;
    if (capture_running()) {
        if (!capture_wifi_active()) {
            /* Text replies would corrupt the active USB binary stream. */
            if (!strcmp(cmd, "STOP")) capture_stop();
        } else if (!strcmp(cmd, "STATUS")) {
            status();
        } else {
            usb_printf("ERR busy\n");
        }
        return;
    }
    if (!strcmp(cmd, "STATUS")) {
        status();
    } else if (!strcmp(cmd, "SCAN")) {
        if (capture_running()) {
            usb_printf("ERR busy\n");
        } else {
            const uint8_t mask = lsm6dsv320x_rescan();
            usb_printf("SCAN present_mask=%02X\n", mask);
            status();
        }
    } else if (!strcmp(cmd, "LEDTEST")) {
        if (capture_running()) {
            usb_printf("ERR busy\n");
        } else {
            usb_printf("LEDTEST BEGIN pins=3,4,5 active_level=%d\n",
                       1);
            const esp_err_t err = led_status_test(2);
            if (err == ESP_OK) usb_printf("LEDTEST DONE\n");
            else usb_printf("ERR led_test=%s\n", esp_err_to_name(err));
        }
    } else if (!strcmp(cmd, "CSATEST")) {
        if (capture_running()) {
            usb_printf("ERR busy\n");
        } else {
            usb_printf("CSATEST BEGIN pin=GPIO16 transitions=10 interval_ms=1000\n");
            const esp_err_t err = lsm6dsv320x_cs_a_test(10, 1000);
            if (err == ESP_OK) usb_printf("CSATEST DONE restored=1 level=HIGH\n");
            else usb_printf("ERR cs_a_test=%s\n", esp_err_to_name(err));
        }
    } else if (!strcmp(cmd, "START")) {
        if (capture_running()) {
            usb_printf("ERR busy\n");
        } else if (lsm6dsv320x_present_mask() == 0U) {
            usb_printf("ERR no_sensors present_mask=%02X\n",
                       lsm6dsv320x_present_mask());
        } else {
            usb_printf("CAPTURE START present_mask=%02X odr_hz=%u fs_g=%u order=1234\n",
                       lsm6dsv320x_present_mask(), LSM6DSV320X_ODR_HZ,
                       LSM6DSV320X_FS_G);
            if (!capture_start()) usb_printf("ERR capture_start_failed\n");
        }
    } else if (!strcmp(cmd, "STOP")) {
        if (capture_running()) capture_stop();
        else usb_printf("ERR not_running\n");
    } else if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) {
        usb_printf("BowVib commands: START STOP STATUS SCAN LEDTEST CSATEST HELP\n");
        usb_printf("START streams detected HG FIFO channels A-D, 7680 Hz, +/-320 g.\n");
        usb_printf("SCAN retries all four chip selects; LEDTEST flashes GPIO3,4,5.\n");
        usb_printf("CSATEST toggles CS_A/GPIO16 every second for 10 transitions.\n");
    } else {
        usb_printf("ERR unknown command\n");
    }
}

static void handle_line(char *line)
{
    if (!capture_control_lock()) {
        usb_printf("ERR capture_unavailable\n");
        return;
    }
    handle_line_unlocked(line);
    capture_control_unlock();
}

static void usb_task(void *arg)
{
    (void)arg;
    usb_printf("\nBowVib ready. LSM6DSV320X x4, 320 g, 7680 Hz.\n");
    if (wifi_stream_ready()) {
        usb_printf("Wi-Fi AP: %s password=%s TCP=192.168.4.1:%u\n",
                   BOWVIB_WIFI_SSID, BOWVIB_WIFI_PASSWORD, BOWVIB_WIFI_PORT);
    } else {
        usb_printf("Wi-Fi AP startup failed\n");
    }
    status();

    char line[64];
    size_t pos = 0;
    for (;;) {
        uint8_t byte;
        const int n = usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;
        if (byte == '\r' || byte == '\n') {
            if (pos) {
                line[pos] = '\0';
                handle_line(line);
            }
            pos = 0;
        } else if (pos < sizeof(line) - 1U) {
            line[pos++] = (char)byte;
        }
    }
}

esp_err_t usb_start(void)
{
    s_tx_lock = xSemaphoreCreateMutex();
    if (!s_tx_lock) return ESP_ERR_NO_MEM;
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 4096,
        .rx_buffer_size = 4096,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_OK && xTaskCreate(usb_task, "usb", 4096, NULL, 5, NULL) != pdPASS) {
        usb_serial_jtag_driver_uninstall();
        err = ESP_ERR_NO_MEM;
    }
    if (err != ESP_OK) {
        vSemaphoreDelete(s_tx_lock);
        s_tx_lock = NULL;
    }
    return err;
}
