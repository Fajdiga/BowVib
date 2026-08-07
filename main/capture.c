/*
 * capture.c — PSRAM capture engine for IIS3DWB10IS.
 */
#include "capture.h"
#include "usb.h"

#include <string.h>
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "capture";

/* Up to this many bytes of PSRAM are used for the STORE buffer. Tunable. */
#define CAP_MAX_BYTES   (8u * 1024u * 1024u)
#define STREAM_ROWS     200u          /* rows per STREAM chunk (2000 bytes) */

typedef enum { CAP_IDLE, CAP_STORE, CAP_STREAM } cap_mode_t;

static uint8_t  *s_buf;               /* PSRAM STORE buffer */
static uint32_t  s_cap_bytes;         /* capacity, bytes */
static uint32_t  s_write_idx;         /* valid bytes */
static uint32_t  s_rows;
static uint32_t  s_psram_size;
static bool      s_overrun;

static volatile cap_mode_t s_mode = CAP_IDLE;
static volatile bool       s_stop_req = false;
static TaskHandle_t s_task;

static int64_t s_store_start_us;
static uint32_t s_store_dur_ms;
static iis3dwb10is_odr_t s_last_odr = IIS3DWB10IS_ODR_20K;
static iis3dwb10is_fs_t  s_last_fs  = IIS3DWB10IS_FS_200G;

/* STREAM scratch (internal RAM). */
static uint8_t s_stream_scratch[STREAM_ROWS * FIFO_ROW_BYTES];

/* ---- accessors --------------------------------------------------------- */
uint8_t  *capture_buf(void)        { return s_buf; }
uint32_t  capture_bytes(void)      { return s_write_idx; }
uint32_t  capture_rows(void)       { return s_rows; }
uint32_t  capture_capacity(void)   { return s_cap_bytes; }
bool      capture_overrun(void)    { return s_overrun; }
uint32_t  capture_duration_ms(void){ return s_store_dur_ms; }
iis3dwb10is_odr_t capture_last_odr(void)   { return s_last_odr; }
iis3dwb10is_fs_t  capture_last_fs(void)    { return s_last_fs; }
uint32_t  capture_psram_size(void) { return s_psram_size; }
bool      capture_running(void)    { return s_mode != CAP_IDLE; }

/* ---- INT1 → task (IRAM) ------------------------------------------------ */
static void IRAM_ATTR on_int1(void *arg)
{
    (void)arg;
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &hpw);   /* counting notify: consumed by task */
    portYIELD_FROM_ISR(hpw);
}

/* ---- task -------------------------------------------------------------- */
static void finish_store(bool full)
{
    s_store_dur_ms = (uint32_t)((esp_timer_get_time() - s_store_start_us) / 1000);
    iis3dwb10is_stop();
    s_mode = CAP_IDLE;
    usb_printf("CAPTURE %s rows=%lu bytes=%lu overrun=%d odr=%u fs=%u dur_ms=%lu\n",
               full ? "DONE" : "STOPPED",
               (unsigned long)s_rows, (unsigned long)s_write_idx,
               (int)s_overrun, (unsigned)s_last_odr, (unsigned)s_last_fs,
               (unsigned long)s_store_dur_ms);
}

static void capture_task(void *arg)
{
    (void)arg;
    while (1) {
        /* Block until INT1 (or a stop/timeout while running). */
        if (s_mode == CAP_IDLE) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));   /* wake on wtm / fallback */

        if (s_mode == CAP_STORE) {
            uint16_t lvl = iis3dwb10is_fifo_level();
            if (lvl) {
                uint32_t free_rows = (s_cap_bytes - s_write_idx) / FIFO_ROW_BYTES;
                uint16_t n = (lvl > free_rows) ? (uint16_t)free_rows : lvl;
                if (n) {
                    iis3dwb10is_fifo_read(s_buf + s_write_idx, n);
                    s_write_idx += (uint32_t)n * FIFO_ROW_BYTES;
                    s_rows += n;
                    if (iis3dwb10is_fifo_overrun()) {
                        s_overrun = true;
                    }
                }
            }
            if (s_stop_req || s_write_idx + FIFO_ROW_BYTES > s_cap_bytes) {
                finish_store(!s_stop_req);
                s_stop_req = false;
            }
        } else if (s_mode == CAP_STREAM) {
            uint16_t lvl = iis3dwb10is_fifo_level();
            uint16_t n = (lvl > STREAM_ROWS) ? STREAM_ROWS : lvl;
            if (n) {
                iis3dwb10is_fifo_read(s_stream_scratch, n);
                usb_send(s_stream_scratch, (size_t)n * FIFO_ROW_BYTES);
                if (iis3dwb10is_fifo_overrun()) {
                    s_overrun = true;
                }
            }
            if (s_stop_req) {
                iis3dwb10is_stop();
                s_mode = CAP_IDLE;
                s_stop_req = false;
                usb_printf("STREAM STOPPED overrun=%d\n", (int)s_overrun);
            }
        }
    }
}

/* ---- control ----------------------------------------------------------- */
void capture_start_store(iis3dwb10is_odr_t odr, iis3dwb10is_fs_t fs)
{
    if (!s_buf) {
        return;
    }
    if (s_mode != CAP_IDLE) {                 /* keep SPI single-threaded */
        usb_printf("ERR busy, send STOP first\n");
        return;
    }
    s_last_odr = odr;
    s_last_fs = fs;
    s_write_idx = 0;
    s_rows = 0;
    s_overrun = false;
    s_stop_req = false;
    s_store_start_us = esp_timer_get_time();
    s_mode = CAP_STORE;
    iis3dwb10is_start(odr, fs);
    xTaskNotifyGive(s_task);
    usb_printf("STORE START odr=%u fs=%u cap_bytes=%lu\n",
               (unsigned)odr, (unsigned)fs, (unsigned long)s_cap_bytes);
}

void capture_start_stream(iis3dwb10is_odr_t odr, iis3dwb10is_fs_t fs)
{
    if (s_mode != CAP_IDLE) {
        usb_printf("ERR busy, send STOP first\n");
        return;
    }
    s_last_odr = odr;
    s_last_fs = fs;
    s_overrun = false;
    s_stop_req = false;
    s_mode = CAP_STREAM;
    usb_printf("STREAM START odr=%u fs=%u (binary follows; send STOP to end)\n",
               (unsigned)odr, (unsigned)fs);
    iis3dwb10is_start(odr, fs);
    xTaskNotifyGive(s_task);   /* capture task is IDLE until now → text precedes binary */
}

void capture_stop(void)
{
    if (s_mode != CAP_IDLE) {
        s_stop_req = true;
        xTaskNotifyGive(s_task);
    }
}

esp_err_t capture_init(void)
{
    size_t psz = esp_psram_get_size();
    if (psz == 0) {
        ESP_LOGE(TAG, "no PSRAM detected — this firmware needs octal PSRAM");
        return ESP_ERR_NO_MEM;
    }
    s_psram_size = (uint32_t)psz;

    uint32_t cap = (uint32_t)(psz / 10u) * 9u;        /* 90 % */
    if (cap > CAP_MAX_BYTES) {
        cap = CAP_MAX_BYTES;
    }
    cap -= (cap % FIFO_ROW_BYTES);                    /* whole rows */
    s_buf = (uint8_t *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!s_buf) {
        ESP_LOGE(TAG, "PSRAM malloc of %lu bytes failed", (unsigned long)cap);
        return ESP_ERR_NO_MEM;
    }
    s_cap_bytes = cap;
    s_write_idx = 0;
    s_rows = 0;
    s_overrun = false;

    xTaskCreatePinnedToCore(capture_task, "capture", 4096, NULL,
                            configMAX_PRIORITIES - 2, &s_task, 1);
    iis3dwb10is_set_int1_callback(on_int1, NULL);

    ESP_LOGI(TAG, "PSRAM %lu KB, capture buffer %lu KB (%lu rows, ~%lu s @20kHz)",
             (unsigned long)(psz / 1024), (unsigned long)(cap / 1024),
             (unsigned long)(cap / FIFO_ROW_BYTES),
             (unsigned long)(cap / FIFO_ROW_BYTES / 20000));
    return ESP_OK;
}
