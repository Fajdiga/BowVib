/*
 * capture.c — PSRAM capture engine for IIS3DWB10IS.
 */
#include "capture.h"
#include "usb.h"

#include <string.h>
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "capture";

/* Up to this many bytes of PSRAM are used for the STORE buffer. Tunable. */
#define CAP_MAX_BYTES   (8u * 1024u * 1024u)
#define STREAM_ROWS     400u          /* rows per STREAM chunk (4000 bytes) */
#define STREAM_BUFFERS  3u            /* overlap SPI reads with USB writes */

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
static QueueHandle_t s_stream_free;
static QueueHandle_t s_stream_ready;

static int64_t s_store_start_us;
static uint32_t s_store_dur_ms;
static iis3dwb10is_odr_t s_last_odr = IIS3DWB10IS_ODR_40K;
static iis3dwb10is_fs_t  s_last_fs  = IIS3DWB10IS_FS_50G;

/* STREAM buffers stay in internal RAM for SPI DMA. */
static uint8_t s_stream_buf[STREAM_BUFFERS][STREAM_ROWS * FIFO_ROW_BYTES];
static bool     s_direct_stream_active;
static uint8_t  s_direct_stream_index;
static uint16_t s_direct_stream_rows;

typedef struct {
    uint8_t  index;
    uint16_t rows;
} stream_block_t;

static uint32_t odr_to_hz(iis3dwb10is_odr_t odr)
{
    switch (odr) {
    case IIS3DWB10IS_ODR_2P5K: return 2500U;
    case IIS3DWB10IS_ODR_5K:   return 5000U;
    case IIS3DWB10IS_ODR_10K:  return 10000U;
    case IIS3DWB10IS_ODR_20K:  return 20000U;
    case IIS3DWB10IS_ODR_40K:  return 40000U;
    case IIS3DWB10IS_ODR_80K:  return 80000U;
    default:                   return 0U;
    }
}

static uint16_t fs_to_g(iis3dwb10is_fs_t fs)
{
    return fs == IIS3DWB10IS_FS_50G ? 50U :
           fs == IIS3DWB10IS_FS_100G ? 100U : 200U;
}

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
    usb_printf("CAPTURE %s rows=%lu bytes=%lu overrun=%d "
               "odr_hz=%lu fs_g=%u dur_ms=%lu\n",
               full ? "DONE" : "STOPPED",
               (unsigned long)s_rows, (unsigned long)s_write_idx,
               (int)s_overrun, (unsigned long)odr_to_hz(s_last_odr),
               (unsigned)fs_to_g(s_last_fs),
               (unsigned long)s_store_dur_ms);
}

static void stream_tx_task(void *arg)
{
    (void)arg;
    stream_block_t block;
    while (1) {
        if (xQueueReceive(s_stream_ready, &block, portMAX_DELAY) == pdTRUE) {
            usb_send(s_stream_buf[block.index],
                     (size_t)block.rows * FIFO_ROW_BYTES);
            xQueueSend(s_stream_free, &block.index, portMAX_DELAY);
        }
    }
}

static bool direct_store_sample(void)
{
    if (s_write_idx + FIFO_ROW_BYTES > s_cap_bytes) {
        return false;
    }
    uint8_t row[FIFO_ROW_BYTES];
    if (!iis3dwb10is_direct_read_row(row)) {
        s_overrun = true;
        return false;
    }
    memcpy(s_buf + s_write_idx, row, FIFO_ROW_BYTES);
    s_write_idx += FIFO_ROW_BYTES;
    s_rows++;
    return true;
}

static bool direct_stream_sample(void)
{
    if (!s_direct_stream_active) {
        if (xQueueReceive(s_stream_free, &s_direct_stream_index,
                          pdMS_TO_TICKS(1)) != pdTRUE) {
            s_overrun = true;
            return false;
        }
        s_direct_stream_rows = 0;
        s_direct_stream_active = true;
    }
    uint8_t *dst = s_stream_buf[s_direct_stream_index] +
                   (size_t)s_direct_stream_rows * FIFO_ROW_BYTES;
    if (!iis3dwb10is_direct_read_row(dst)) {
        s_overrun = true;
        return false;
    }
    s_direct_stream_rows++;
    if (s_direct_stream_rows == STREAM_ROWS) {
        stream_block_t block = {
            .index = s_direct_stream_index,
            .rows = s_direct_stream_rows,
        };
        if (xQueueSend(s_stream_ready, &block, 0) != pdTRUE) {
            xQueueSend(s_stream_free, &s_direct_stream_index, portMAX_DELAY);
            s_overrun = true;
        }
        s_direct_stream_active = false;
        s_direct_stream_rows = 0;
    }
    return true;
}

/* INT1 is the preferred wake source.  If the board's INT1 trace is not
   populated, XLDA polling keeps the direct path functional instead of
   freezing at zero samples.  The status bit is checked immediately before
   each DMA read, so this does not duplicate samples. */
static void direct_poll_fallback(bool stream)
{
    const int64_t deadline = esp_timer_get_time() + 10000; /* 10 ms */
    while (!s_stop_req && iis3dwb10is_direct_mode() &&
           esp_timer_get_time() < deadline) {
        if (iis3dwb10is_data_ready()) {
            if (stream ? !direct_stream_sample() : !direct_store_sample()) {
                break;
            }
        } else {
            esp_rom_delay_us(2);
        }
    }
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

        uint32_t events = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        const bool got_events = events != 0;

        if (s_mode == CAP_STORE) {
            if (iis3dwb10is_direct_mode()) {
                /* One INT1 event corresponds to one fresh direct-register
                   sample.  The SPI transaction itself is queued to the DMA
                   engine; no FIFO status polling is involved. */
                while (events-- && !s_stop_req) {
                    if (!direct_store_sample()) {
                        break;
                    }
                }
                if (!got_events && !s_stop_req) {
                    direct_poll_fallback(false);
                }
            } else {
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
            }
            if (s_stop_req || s_write_idx + FIFO_ROW_BYTES > s_cap_bytes) {
                finish_store(!s_stop_req);
                s_stop_req = false;
            }
        } else if (s_mode == CAP_STREAM) {
            if (iis3dwb10is_direct_mode()) {
                /* Keep a partially-filled USB block across interrupt wakeups
                   so the host receives efficient bursts instead of one USB
                   write per sample. */
                while (events-- && !s_stop_req) {
                    if (!direct_stream_sample()) {
                        break;
                    }
                }
                if (!got_events && !s_stop_req) {
                    direct_poll_fallback(true);
                }
            } else {
                /* FIFO mode: once woken, keep draining all rows currently
                   available.  High ODRs therefore use DMA bulk reads and do
                   not require one interrupt or USB write per sample. */
                while (!s_stop_req) {
                    uint16_t lvl = iis3dwb10is_fifo_level();
                    uint16_t n = (lvl > STREAM_ROWS) ? STREAM_ROWS : lvl;
                    if (!n) {
                        break;
                    }
                    uint8_t index;
                    if (xQueueReceive(s_stream_free, &index,
                                      pdMS_TO_TICKS(1)) != pdTRUE) {
                        if (iis3dwb10is_fifo_overrun()) {
                            s_overrun = true;
                        }
                        continue;
                    }
                    iis3dwb10is_fifo_read(s_stream_buf[index], n);
                    stream_block_t block = { .index = index, .rows = n };
                    if (xQueueSend(s_stream_ready, &block, 0) != pdTRUE) {
                        /* Should be unreachable: ready and free have equal depth. */
                        xQueueSend(s_stream_free, &index, portMAX_DELAY);
                        s_overrun = true;
                    }
                    if (iis3dwb10is_fifo_overrun()) {
                        s_overrun = true;
                    }
                }
            }
            if (s_stop_req) {
                iis3dwb10is_stop();
                if (s_direct_stream_active) {
                    if (s_direct_stream_rows) {
                        stream_block_t block = {
                            .index = s_direct_stream_index,
                            .rows = s_direct_stream_rows,
                        };
                        xQueueSend(s_stream_ready, &block, portMAX_DELAY);
                    } else {
                        xQueueSend(s_stream_free, &s_direct_stream_index, portMAX_DELAY);
                    }
                    s_direct_stream_active = false;
                    s_direct_stream_rows = 0;
                }
                /* Keep the textual STOP reply behind all queued binary data. */
                while (uxQueueMessagesWaiting(s_stream_free) < STREAM_BUFFERS) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
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
    if (!iis3dwb10is_direct_mode()) {
        xTaskNotifyGive(s_task);              /* FIFO may already contain rows */
    }
    usb_printf("STORE START odr_hz=%lu fs_g=%u cap_bytes=%lu\n",
               (unsigned long)odr_to_hz(odr), (unsigned)fs_to_g(fs),
               (unsigned long)s_cap_bytes);
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
    s_direct_stream_active = false;
    s_direct_stream_rows = 0;
    s_mode = CAP_STREAM;
    usb_printf("STREAM START odr_hz=%lu fs_g=%u "
               "(binary follows; send STOP to end)\n",
               (unsigned long)odr_to_hz(odr), (unsigned)fs_to_g(fs));
    iis3dwb10is_start(odr, fs);
    if (!iis3dwb10is_direct_mode()) {
        xTaskNotifyGive(s_task);   /* FIFO may already contain rows */
    }
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

    s_stream_free = xQueueCreate(STREAM_BUFFERS, sizeof(uint8_t));
    s_stream_ready = xQueueCreate(STREAM_BUFFERS, sizeof(stream_block_t));
    if (!s_stream_free || !s_stream_ready) {
        ESP_LOGE(TAG, "failed to allocate STREAM queues");
        return ESP_ERR_NO_MEM;
    }
    for (uint8_t i = 0; i < STREAM_BUFFERS; ++i) {
        xQueueSend(s_stream_free, &i, portMAX_DELAY);
    }

    xTaskCreatePinnedToCore(capture_task, "capture", 4096, NULL,
                            configMAX_PRIORITIES - 2, &s_task, 1);
    xTaskCreatePinnedToCore(stream_tx_task, "stream_tx", 4096, NULL,
                            configMAX_PRIORITIES - 3, NULL, 0);
    iis3dwb10is_set_int1_callback(on_int1, NULL);

    ESP_LOGI(TAG, "PSRAM %lu KB, capture buffer %lu KB (%lu rows, ~%lu s @40kHz)",
             (unsigned long)(psz / 1024), (unsigned long)(cap / 1024),
             (unsigned long)(cap / FIFO_ROW_BYTES),
             (unsigned long)(cap / FIFO_ROW_BYTES / 40000));
    return ESP_OK;
}
