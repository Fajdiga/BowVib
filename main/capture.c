#include "capture.h"
#include "lsm6dsv320x.h"
#include "usb.h"
#include "wifi_stream.h"
#include "web_capture.h"

#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define CHUNK_ROWS LSM6DSV320X_FIFO_ROWS_PER_READ
#define WORD_BYTES LSM6DSV320X_FIFO_WORD_BYTES
#define SAMPLE_BYTES 6U
#define FRAME_HEADER_BYTES 7U
#define WIFI_TX_BUFFER_BYTES (32U * 1024U)
/* Stay ahead of HTTP/TCP sends while draining; the 1 ms delay below gives
   command and network tasks regular time to run. */
#define CAPTURE_TASK_PRIORITY (tskIDLE_PRIORITY + 6U)

static volatile bool s_running;
static volatile bool s_stop_requested;
static volatile bool s_wifi_output;
static volatile bool s_browser_output;
static int64_t s_duration_limit_us;
static TaskHandle_t s_task;
static SemaphoreHandle_t s_control_lock;
static TaskHandle_t s_wifi_tx_task;
static StreamBufferHandle_t s_wifi_tx_buffer;
static StaticStreamBuffer_t s_wifi_tx_buffer_control;
static uint8_t s_wifi_tx_storage[WIFI_TX_BUFFER_BYTES];
static volatile uint32_t s_wifi_enqueued_bytes;
static volatile uint32_t s_wifi_completed_bytes;
static uint32_t s_counts[LSM6DSV320X_COUNT];
static volatile uint8_t s_overrun_mask;
static int64_t s_started_us;
static uint8_t s_fifo_raw[CHUNK_ROWS * WORD_BYTES];
static uint8_t s_frame[FRAME_HEADER_BYTES + CHUNK_ROWS * SAMPLE_BYTES];

static bool capture_send(const void *data, size_t len, TickType_t wait)
{
    if (s_wifi_output) {
        if (len > FRAME_HEADER_BYTES + CHUNK_ROWS * SAMPLE_BYTES) return false;
        if (wait == 0 && xStreamBufferSpacesAvailable(s_wifi_tx_buffer) < len) {
            s_overrun_mask |= 0x80U; /* RAM-to-Wi-Fi queue overflow */
            s_stop_requested = true;
            return false;
        }
        if (xStreamBufferSend(s_wifi_tx_buffer, data, len, wait) != len) {
            s_overrun_mask |= 0x80U; /* RAM-to-Wi-Fi queue overflow */
            s_stop_requested = true;
            return false;
        }
        s_wifi_enqueued_bytes += len;
    } else {
        if (!usb_send(data, len)) {
            s_overrun_mask |= 0x80U; /* Output transport failure. */
            s_stop_requested = true;
            return false;
        }
    }
    return true;
}

static void wifi_tx_task(void *arg)
{
    (void)arg;
    uint8_t data[2048];
    for (;;) {
        /* Coalesce the 1 ms FIFO polls into fewer, larger network writes. */
        const size_t len = xStreamBufferReceive(s_wifi_tx_buffer, data,
                                                sizeof(data), pdMS_TO_TICKS(5));
        if (len == 0U) continue;
        const bool sent = s_browser_output ? web_capture_send(data, len) :
                                            wifi_stream_send(data, len);
        if (!sent) {
            s_overrun_mask |= 0x80U;
            s_stop_requested = true;
        }
        s_wifi_completed_bytes += len;
    }
}

static void send_data_frame(unsigned imu, const uint8_t *fifo, uint16_t rows)
{
    uint16_t kept = 0;
    s_frame[0] = 'I'; s_frame[1] = 'M'; s_frame[2] = '4'; s_frame[3] = 'D';
    s_frame[4] = (uint8_t)imu;
    for (uint16_t row = 0; row < rows; ++row) {
        const uint8_t *src = fifo + (size_t)row * WORD_BYTES;
        if ((src[0] & 0xF8U) != (0x1DU << 3)) {
            continue;
        }
        memcpy(s_frame + FRAME_HEADER_BYTES + (size_t)kept * SAMPLE_BYTES,
               src + 1, SAMPLE_BYTES);
        ++kept;
    }
    if (kept == 0) return;
    s_frame[5] = (uint8_t)(kept & 0xFFU);
    s_frame[6] = (uint8_t)(kept >> 8);
    const size_t frame_len = FRAME_HEADER_BYTES + (size_t)kept * SAMPLE_BYTES;
    if (!capture_send(s_frame, frame_len, 0)) return;
    s_counts[imu] += kept;
}

static void send_end_frame(void)
{
    uint8_t end[4U + LSM6DSV320X_COUNT * 4U + 4U + 1U];
    const uint32_t duration_ms = (uint32_t)((esp_timer_get_time() - s_started_us) / 1000);
    memcpy(end, "IM4E", 4);
    for (unsigned i = 0; i < LSM6DSV320X_COUNT; ++i) {
        const uint32_t n = s_counts[i];
        end[4U + i * 4U] = (uint8_t)n;
        end[5U + i * 4U] = (uint8_t)(n >> 8);
        end[6U + i * 4U] = (uint8_t)(n >> 16);
        end[7U + i * 4U] = (uint8_t)(n >> 24);
    }
    const unsigned off = 4U + LSM6DSV320X_COUNT * 4U;
    end[off] = (uint8_t)duration_ms;
    end[off + 1U] = (uint8_t)(duration_ms >> 8);
    end[off + 2U] = (uint8_t)(duration_ms >> 16);
    end[off + 3U] = (uint8_t)(duration_ms >> 24);
    end[off + 4U] = s_overrun_mask;
    (void)capture_send(end, sizeof(end), portMAX_DELAY);
    /* Keep capture busy until the queued footer has passed through TX. */
    while (s_wifi_output && s_wifi_completed_bytes != s_wifi_enqueued_bytes) {
        vTaskDelay(1);
    }
    s_wifi_output = false;
    s_browser_output = false;
}

static void capture_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!s_running) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        if (s_duration_limit_us > 0 &&
            esp_timer_get_time() - s_started_us >= s_duration_limit_us) {
            s_stop_requested = true;
        }
        for (unsigned imu = 0; imu < LSM6DSV320X_COUNT; ++imu) {
            if (!(lsm6dsv320x_present_mask() & (1U << imu))) continue;
            uint16_t level = lsm6dsv320x_fifo_level(imu);
            if (lsm6dsv320x_fifo_overrun(imu)) {
                s_overrun_mask |= (uint8_t)(1U << imu);
            }
            while (level && !s_stop_requested) {
                const uint16_t n = level > CHUNK_ROWS ? CHUNK_ROWS : level;
                if (lsm6dsv320x_fifo_read(imu, s_fifo_raw, n) != ESP_OK) {
                    s_overrun_mask |= (uint8_t)(1U << imu);
                    break;
                }
                send_data_frame(imu, s_fifo_raw, n);
                level = (uint16_t)(level - n);
            }
        }
        if (s_stop_requested) {
            lsm6dsv320x_capture_stop();
            /* Flush the finite tail before reporting completion. */
            for (unsigned imu = 0; imu < LSM6DSV320X_COUNT; ++imu) {
                if (!(lsm6dsv320x_present_mask() & (1U << imu))) continue;
                uint16_t level = lsm6dsv320x_fifo_level(imu);
                while (level) {
                    const uint16_t n = level > CHUNK_ROWS ? CHUNK_ROWS : level;
                    if (lsm6dsv320x_fifo_read(imu, s_fifo_raw, n) != ESP_OK) break;
                    send_data_frame(imu, s_fifo_raw, n);
                    level = (uint16_t)(level - n);
                }
            }
            lsm6dsv320x_capture_finish();
            send_end_frame();
            s_stop_requested = false;
            s_running = false;
            continue;
        }
        /* Yield every 1 ms so lower-priority TCP/USB work and the idle task run;
           the FIFOs absorb this bounded interval while all four are drained. */
        vTaskDelay(1);
    }
}

esp_err_t capture_init(void)
{
    s_control_lock = xSemaphoreCreateMutex();
    if (!s_control_lock) return ESP_ERR_NO_MEM;
    s_wifi_tx_buffer = xStreamBufferCreateStatic(
        WIFI_TX_BUFFER_BYTES, 1U, s_wifi_tx_storage, &s_wifi_tx_buffer_control);
    if (!s_wifi_tx_buffer) return ESP_ERR_NO_MEM;
    BaseType_t ok = xTaskCreate(wifi_tx_task, "wifi_tx", 4096, NULL,
                                CAPTURE_TASK_PRIORITY, &s_wifi_tx_task);
    if (ok != pdPASS) return ESP_ERR_NO_MEM;
    ok = xTaskCreate(capture_task, "capture", 4096, NULL,
                     CAPTURE_TASK_PRIORITY, &s_task);
    if (ok != pdPASS) {
        vTaskDelete(s_wifi_tx_task);
        s_wifi_tx_task = NULL;
    }
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

bool capture_control_lock(void)
{
    return s_control_lock && xSemaphoreTake(s_control_lock, portMAX_DELAY) == pdTRUE;
}

void capture_control_unlock(void)
{
    xSemaphoreGive(s_control_lock);
}

bool capture_start(void)
{
    if (!s_task || s_running || lsm6dsv320x_present_mask() == 0U) return false;
    s_wifi_output = false;
    s_browser_output = false;
    s_duration_limit_us = 0;
    memset(s_counts, 0, sizeof(s_counts));
    s_overrun_mask = 0;
    s_stop_requested = false;
    lsm6dsv320x_capture_start();
    s_started_us = esp_timer_get_time();
    s_running = true;
    xTaskNotifyGive(s_task);
    return true;
}

bool capture_start_wifi(void)
{
    if (!s_task || s_running || lsm6dsv320x_present_mask() == 0U ||
        !wifi_stream_client_connected()) return false;
    s_wifi_output = true;
    s_browser_output = false;
    s_duration_limit_us = 0;
    memset(s_counts, 0, sizeof(s_counts));
    s_overrun_mask = 0;
    /* The previous capture waited for TX completion; no reset of a stream
       buffer with a blocked reader is needed. */
    s_stop_requested = false;
    lsm6dsv320x_capture_start();
    s_started_us = esp_timer_get_time();
    s_running = true;
    xTaskNotifyGive(s_task);
    return true;
}

bool capture_start_browser(void)
{
    if (!s_task || s_running || lsm6dsv320x_present_mask() == 0U ||
        !web_capture_client_connected()) return false;
    s_wifi_output = true;
    s_browser_output = true;
    s_duration_limit_us = 10 * 1000 * 1000;
    memset(s_counts, 0, sizeof(s_counts));
    s_overrun_mask = 0;
    s_stop_requested = false;
    lsm6dsv320x_capture_start();
    s_started_us = esp_timer_get_time();
    s_running = true;
    xTaskNotifyGive(s_task);
    return true;
}

void capture_stop(void)
{
    if (s_running) s_stop_requested = true;
}

bool capture_running(void)
{
    return s_running;
}

bool capture_wifi_active(void)
{
    return s_running && s_wifi_output;
}

uint8_t capture_overrun_mask(void)
{
    return s_overrun_mask;
}

bool capture_browser_active(void)
{
    return s_running && s_browser_output;
}
