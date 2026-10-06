#include "capture.h"
#include "lsm6dsv320x.h"
#include "usb.h"
#include "wifi_stream.h"
#include "web_capture.h"
#include "shot_store.h"
#include "capture_timing.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

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
#define HIGH_G_SATURATION_COUNTS ((LSM6DSV320X_FS_G * 995000U) / 10417U)
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
static uint8_t s_frame[FRAME_HEADER_BYTES + 4U + CHUNK_ROWS * 7U];
static volatile bool s_shot_output, s_trigger_requested;
static volatile int64_t s_trigger_us;
static uint32_t s_pre_ms, s_post_ms, s_threshold_mg;
static uint8_t s_saturation_mask;
static volatile esp_err_t s_shot_error;
static bool s_save_full_session;
static int64_t s_next_clock_us;
static uint32_t s_clock_ticks[4], s_hw_ticks[4];
static int64_t s_clock_us[4];
static bool s_hw_valid[4];
static uint8_t s_hw_tag[4];
typedef struct {
    uint32_t ticks;
    uint8_t xyz[6], tag;
    bool data, time;
} pending_slot_t;
static pending_slot_t s_pending[4];
static uint32_t s_sample_ticks[CHUNK_ROWS];
static bool s_timing_test;
static uint32_t s_gaps[4], s_slot_errors[4], s_pair_errors[4];
static uint16_t s_high_water[4];
typedef struct {
    uint32_t index, before, after, clock_age_us;
    int32_t drdy_delta_us;
    uint8_t slot_step;
} timing_gap_t;
static timing_gap_t s_gap_detail[4][8];

#define DRDY_RING_SIZE 512U
typedef struct { int64_t us; uint32_t seq; int16_t xyz[3]; bool valid; } drdy_edge_t;
static volatile drdy_edge_t s_drdy_edges[4][DRDY_RING_SIZE];
static volatile uint32_t s_drdy_count[4], s_drdy_min[4], s_drdy_max[4], s_drdy_late[4];
static volatile int64_t s_drdy_last[4];
static volatile bool s_drdy_active;
static uint32_t s_first_edge[4];
static int32_t s_edge_match_us[4];
static volatile bool s_direct_test, s_direct_shot;
static uint32_t s_direct_consumed[4];
static volatile uint8_t s_direct_error;
static int64_t s_direct_sums[4][3];
static uint32_t s_direct_backlog[4];

static void IRAM_ATTR drdy_isr(void *arg)
{
    if (!s_drdy_active) return;
    const unsigned imu = (unsigned)(uintptr_t)arg;
    const int64_t now = esp_timer_get_time();
    if (s_drdy_last[imu]) {
        const uint32_t delta = (uint32_t)(now - s_drdy_last[imu]);
        if (delta < s_drdy_min[imu]) s_drdy_min[imu] = delta;
        if (delta > s_drdy_max[imu]) s_drdy_max[imu] = delta;
        if (delta > 200U) ++s_drdy_late[imu];
    }
    s_drdy_last[imu] = now;
    const uint32_t seq = s_drdy_count[imu] + 1U;
    volatile drdy_edge_t *edge = &s_drdy_edges[imu][seq & (DRDY_RING_SIZE - 1U)];
    if (s_direct_test || s_direct_shot) {
        int16_t xyz[3];
        edge->valid = !s_direct_error && lsm6dsv320x_direct_read_isr(imu, xyz);
        if (edge->valid) {
            edge->xyz[0] = xyz[0]; edge->xyz[1] = xyz[1]; edge->xyz[2] = xyz[2];
        } else s_direct_error |= 1U << imu;
    }
    edge->us = now; edge->seq = seq;
    s_drdy_count[imu] = seq;
}

static int32_t drdy_interval(unsigned imu, uint32_t index, uint32_t ticks)
{
    if (!s_first_edge[imu]) {
        const int32_t fine = lsm6dsv320x_internal_freq_fine(imu);
        const int64_t estimate = s_clock_us[imu] +
            ((int64_t)ticks - s_clock_ticks[imu]) * INT64_C(10000000000) /
            (46080 * (int64_t)(10000 + (fine == INT8_MIN ? 0 : 13 * fine)));
        int64_t best = INT64_MAX;
        uint32_t chosen = 0;
        const uint32_t last = s_drdy_count[imu];
        const uint32_t first = last > DRDY_RING_SIZE - 1U ? last - DRDY_RING_SIZE + 1U : 1U;
        for (uint32_t seq = first; seq <= last; ++seq) {
            const volatile drdy_edge_t *e = &s_drdy_edges[imu][seq & (DRDY_RING_SIZE - 1U)];
            const int64_t error = e->us - estimate;
            const int64_t magnitude = error < 0 ? -error : error;
            if (e->seq == seq && magnitude < best) { best = magnitude; chosen = seq; }
        }
        if (!chosen || best > 60 || chosen <= index) return -1;
        s_first_edge[imu] = chosen - index;
        s_edge_match_us[imu] = (int32_t)(s_drdy_edges[imu][chosen & (DRDY_RING_SIZE - 1U)].us - estimate);
    }
    const uint32_t seq = s_first_edge[imu] + index;
    const volatile drdy_edge_t *e = &s_drdy_edges[imu][seq & (DRDY_RING_SIZE - 1U)];
    const volatile drdy_edge_t *p = &s_drdy_edges[imu][(seq - 1U) & (DRDY_RING_SIZE - 1U)];
    if (e->seq != seq || p->seq != seq - 1U) return -1;
    return (int32_t)(e->us - p->us);
}

static esp_err_t record_clocks(void)
{
    for (unsigned imu = 0; imu < LSM6DSV320X_COUNT; ++imu) {
        if (!(lsm6dsv320x_present_mask() & (1U << imu))) continue;
        uint32_t ticks, span;
        int64_t midpoint;
        esp_err_t err = lsm6dsv320x_clock_read(imu, &ticks, &midpoint, &span);
        if (err == ESP_OK) {
            s_clock_ticks[imu] = ticks; s_clock_us[imu] = midpoint;
            if (!s_timing_test) err = shot_store_clock(esp_timer_get_time(), imu, ticks, midpoint, span);
        }
        if (err != ESP_OK) return err;
    }
    s_next_clock_us = esp_timer_get_time() + 250000;
    return ESP_OK;
}

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

static void store_timed_frame(unsigned imu, uint16_t rows)
{
    if (!rows) return;
    if (s_timing_test) { s_counts[imu] += rows; return; }
    const int64_t now = esp_timer_get_time();
    uint16_t trigger_row = UINT16_MAX;
    for (uint16_t row = 0; row < rows; ++row) {
        int16_t xyz[3];
        memcpy(xyz, s_frame + 11U + row * 7U, sizeof(xyz));
        uint64_t norm = 0;
        for (unsigned axis = 0; axis < 3; ++axis) {
            const int32_t v = xyz[axis];
            if (v >= (int32_t)HIGH_G_SATURATION_COUNTS || v <= -(int32_t)HIGH_G_SATURATION_COUNTS)
                s_saturation_mask |= (uint8_t)(1U << imu);
            norm += (int64_t)v * v;
        }
        if (capture_threshold_reached(norm, s_threshold_mg) && !s_trigger_us && !s_stop_requested) {
            const int32_t fine = lsm6dsv320x_internal_freq_fine(imu);
            const int64_t estimate = s_direct_shot ? s_started_us + s_sample_ticks[row] : s_clock_us[imu] +
                ((int64_t)s_sample_ticks[row] - s_clock_ticks[imu]) * INT64_C(10000000000) /
                (46080 * (int64_t)(10000 + (fine == INT8_MIN ? 0 : 13 * fine)));
            s_trigger_us = estimate > s_started_us ? estimate : s_started_us;
            s_trigger_requested = false;
            trigger_row = row;
        }
    }
    memcpy(s_frame, "IM4S", 4); s_frame[4] = imu;
    s_frame[5] = rows & 255U; s_frame[6] = rows >> 8;
    memcpy(s_frame + 7, s_sample_ticks, 4);
    esp_err_t err = shot_store_append(s_frame, 11U + rows * 7U, now, s_counts[imu]);
    if (err == ESP_OK && trigger_row != UINT16_MAX)
        err = shot_store_mark_trigger(now, imu, s_counts[imu] + trigger_row, s_threshold_mg);
    if (err != ESP_OK) { s_shot_error = err; s_overrun_mask |= 0x80U; s_stop_requested = true; }
    else s_counts[imu] += rows;
}

static void send_data_frame(unsigned imu, const uint8_t *fifo, uint16_t rows, uint16_t remaining_rows)
{
    (void)remaining_rows;
    uint16_t kept = 0;
    if (s_shot_output) {
        pending_slot_t *slot = &s_pending[imu];
        for (uint16_t row = 0; row < rows; ++row) {
            const uint8_t *src = fifo + (size_t)row * WORD_BYTES;
            const uint8_t kind = src[0] >> 3, tag = (src[0] >> 1) & 3U;
            if (kind != 0x04U && kind != 0x1DU) { s_overrun_mask |= 1U << imu; continue; }
            /* Either ordering and a split SPI chunk are supported. Never attach
               a timestamp from another time slot or invent a missing sample. */
            if ((slot->data || slot->time) &&
                (slot->tag != tag || (kind == 0x04U ? slot->time : slot->data))) {
                s_overrun_mask |= 1U << imu;
                ++s_pair_errors[imu];
                slot->data = slot->time = false;
            }
            slot->tag = tag;
            if (kind == 0x04U) { memcpy(&slot->ticks, src + 1, 4); slot->time = true; }
            else { memcpy(slot->xyz, src + 1, 6); slot->data = true; }
            if (!slot->data || !slot->time) continue;
            const uint32_t ticks = slot->ticks;
            const uint32_t delta = ticks - s_hw_ticks[imu];
            const int32_t edge_delta = s_timing_test ? drdy_interval(imu, s_counts[imu] + kept, ticks) : -1;
            if (s_hw_valid[imu]) {
                if (ticks <= s_hw_ticks[imu] || delta < 4U || delta > 8U) {
                    if (s_timing_test && s_gaps[imu] < 8U)
                        s_gap_detail[imu][s_gaps[imu]] = (timing_gap_t){
                            .index = s_counts[imu] + kept, .before = s_hw_ticks[imu], .after = ticks,
                            .clock_age_us = (uint32_t)(esp_timer_get_time() - s_clock_us[imu]),
                            .drdy_delta_us = edge_delta,
                            .slot_step = (slot->tag - s_hw_tag[imu]) & 3U,
                        };
                    ++s_gaps[imu];
                }
                if (((slot->tag - s_hw_tag[imu]) & 3U) != 1U) ++s_slot_errors[imu];
            }
            if (s_hw_valid[imu] && (ticks <= s_hw_ticks[imu] || delta < 4U || delta > 8U ||
                                   ((slot->tag - s_hw_tag[imu]) & 3U) != 1U)) {
                s_overrun_mask |= 1U << imu;
            }
            /* A large interval starts a new frame with an absolute timestamp;
               no one-byte truncation, even on a damaged recording. */
            if (kept && delta > 255U) { store_timed_frame(imu, kept); kept = 0; }
            memcpy(s_frame + 11U + kept * 7U, slot->xyz, 6);
            s_frame[11U + kept * 7U + 6U] = kept ? delta : 0;
            s_sample_ticks[kept++] = ticks;
            s_hw_ticks[imu] = ticks; s_hw_valid[imu] = true;
            s_hw_tag[imu] = slot->tag;
            slot->data = slot->time = false;
        }
        store_timed_frame(imu, kept);
        return;
    }
    memcpy(s_frame, "IM4D", 4); s_frame[4] = imu;
    for (uint16_t row = 0; row < rows; ++row) {
        const uint8_t *src = fifo + (size_t)row * WORD_BYTES;
        if ((src[0] >> 3) != 0x1DU) continue;
        memcpy(s_frame + FRAME_HEADER_BYTES + (size_t)kept++ * SAMPLE_BYTES, src + 1, SAMPLE_BYTES);
    }
    if (!kept) return;
    s_frame[5] = kept & 255U; s_frame[6] = kept >> 8;
    if (capture_send(s_frame, FRAME_HEADER_BYTES + kept * SAMPLE_BYTES, 0)) s_counts[imu] += kept;
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

static void direct_test_poll(void)
{
    if (s_direct_error) s_stop_requested = true;
    if (esp_timer_get_time() - s_started_us >= s_duration_limit_us) s_stop_requested = true;
    if (s_stop_requested) {
        s_drdy_active = false;
        lsm6dsv320x_direct_release();
        lsm6dsv320x_capture_stop(); lsm6dsv320x_capture_finish();
        (void)lsm6dsv320x_drdy_monitor(false);
    }
    for (unsigned imu = 0; imu < 4; ++imu) {
        const uint32_t last = s_drdy_count[imu];
        const uint32_t backlog = last - s_counts[imu];
        if (backlog > s_direct_backlog[imu]) s_direct_backlog[imu] = backlog;
        if (backlog >= DRDY_RING_SIZE) { s_direct_error |= 1U << imu; s_counts[imu] = last; }
        while (s_counts[imu] < last) {
            const uint32_t seq = s_counts[imu] + 1U;
            const volatile drdy_edge_t *edge = &s_drdy_edges[imu][seq & (DRDY_RING_SIZE - 1U)];
            if (edge->seq != seq || !edge->valid) s_direct_error |= 1U << imu;
            else for (unsigned axis = 0; axis < 3; ++axis) s_direct_sums[imu][axis] += edge->xyz[axis];
            ++s_counts[imu];
        }
        if (s_drdy_late[imu]) s_direct_error |= 1U << imu;
    }
    if (s_stop_requested) {
        for (unsigned imu = 0; imu < 4; ++imu)
            usb_printf("DIRECT sensor=%u count=%lu edges=%lu late=%lu min_us=%lu max_us=%lu backlog=%lu mean_xyz=%ld,%ld,%ld\n",
                imu, (unsigned long)s_counts[imu], (unsigned long)s_drdy_count[imu],
                (unsigned long)s_drdy_late[imu], (unsigned long)s_drdy_min[imu],
                (unsigned long)s_drdy_max[imu], (unsigned long)s_direct_backlog[imu],
                (long)(s_direct_sums[imu][0] / (s_counts[imu] ? s_counts[imu] : 1U)),
                (long)(s_direct_sums[imu][1] / (s_counts[imu] ? s_counts[imu] : 1U)),
                (long)(s_direct_sums[imu][2] / (s_counts[imu] ? s_counts[imu] : 1U)));
        usb_printf("TIMING DONE elapsed_ms=%lu quality=%u spi_read_max_us=%lu\n",
            (unsigned long)((esp_timer_get_time() - s_started_us) / 1000), s_direct_error,
            (unsigned long)lsm6dsv320x_direct_read_max_us());
        s_direct_test = false; s_running = false; s_stop_requested = false;
    } else vTaskDelay(1);
}

static void direct_shot_poll(void)
{
    if (s_trigger_requested && !s_trigger_us && !s_stop_requested) {
        s_trigger_us = esp_timer_get_time(); s_trigger_requested = false;
    }
    if (s_trigger_us && esp_timer_get_time() - s_trigger_us >= (int64_t)s_post_ms * 1000)
        s_stop_requested = true;
    if (shot_store_near_capacity()) {
        if (!s_trigger_us) s_save_full_session = true;
        else s_overrun_mask |= 0x40U;
        s_stop_requested = true;
    }
    if (s_direct_error) { s_overrun_mask |= s_direct_error; s_stop_requested = true; }
    const bool finishing = s_stop_requested;
    if (finishing) {
        /* Freeze the producer before touching the driver or draining its tail. */
        s_drdy_active = false;
        lsm6dsv320x_direct_release();
        lsm6dsv320x_capture_stop(); lsm6dsv320x_capture_finish();
        (void)lsm6dsv320x_drdy_monitor(false);
    }
    for (unsigned imu = 0; imu < 4; ++imu) {
        const uint32_t last = s_drdy_count[imu];
        const uint32_t backlog = last - s_direct_consumed[imu];
        if (backlog > s_direct_backlog[imu]) s_direct_backlog[imu] = backlog;
        if (backlog >= DRDY_RING_SIZE) {
            s_direct_error |= 1U << imu; s_direct_consumed[imu] = last;
        }
        uint16_t rows = 0;
        while (s_direct_consumed[imu] < last) {
            const uint32_t seq = s_direct_consumed[imu] + 1U;
            /* Copy all fields before the ISR can reuse this slot. */
            const drdy_edge_t edge = s_drdy_edges[imu][seq & (DRDY_RING_SIZE - 1U)];
            if (edge.seq != seq || s_drdy_edges[imu][seq & (DRDY_RING_SIZE - 1U)].seq != seq ||
                s_drdy_count[imu] - s_direct_consumed[imu] >= DRDY_RING_SIZE ||
                !edge.valid || edge.us < s_started_us ||
                edge.us - s_started_us > UINT32_MAX) {
                s_direct_error |= 1U << imu; break;
            }
            const uint32_t stamp = (uint32_t)(edge.us - s_started_us);
            const uint32_t delta = rows ? stamp - s_sample_ticks[rows - 1U] : 0;
            if (rows && (rows == CHUNK_ROWS || !delta || delta > 255U)) {
                store_timed_frame(imu, rows); rows = 0;
            }
            s_sample_ticks[rows] = stamp;
            memcpy(s_frame + 11U + rows * 7U, edge.xyz, 6);
            s_frame[17U + rows * 7U] = rows ? (uint8_t)delta : 0;
            ++rows; ++s_direct_consumed[imu];
        }
        store_timed_frame(imu, rows);
        if (s_drdy_late[imu]) s_direct_error |= 1U << imu;
    }
    s_overrun_mask |= s_direct_error;
    if (s_direct_error) {
        s_shot_error = ESP_ERR_INVALID_RESPONSE;
        s_save_full_session = true;
        s_stop_requested = true;
    }
    if (finishing) {
        const esp_err_t err = shot_store_finish(s_started_us, s_trigger_us, esp_timer_get_time(),
            s_overrun_mask, s_saturation_mask, s_save_full_session);
        if (err != ESP_OK) s_shot_error = err;
        for (unsigned imu = 0; imu < 4; ++imu)
            usb_printf("SAVED DIRECT sensor=%u count=%lu edges=%lu late=%lu min_us=%lu max_us=%lu backlog=%lu\n",
                imu, (unsigned long)s_counts[imu], (unsigned long)s_drdy_count[imu],
                (unsigned long)s_drdy_late[imu], (unsigned long)s_drdy_min[imu],
                (unsigned long)s_drdy_max[imu], (unsigned long)s_direct_backlog[imu]);
        usb_printf("SAVED DIRECT DONE quality=%u spi_read_max_us=%lu\n", s_overrun_mask,
            (unsigned long)lsm6dsv320x_direct_read_max_us());
        s_direct_shot = s_shot_output = s_running = s_stop_requested = false;
    } else vTaskDelay(2);
}

static void capture_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!s_running) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        if (s_direct_test) { direct_test_poll(); continue; }
        if (s_direct_shot) { direct_shot_poll(); continue; }
        if (s_duration_limit_us > 0 && !(s_shot_output && s_trigger_us) &&
            esp_timer_get_time() - s_started_us >= s_duration_limit_us) {
            if (s_shot_output) s_save_full_session = true;
            s_stop_requested = true;
        }
        if (s_shot_output && s_trigger_requested && !s_trigger_us && !s_stop_requested) {
            s_trigger_us = esp_timer_get_time();
            s_trigger_requested = false;
        }
        if (s_shot_output && s_trigger_us &&
            esp_timer_get_time() - s_trigger_us >= (int64_t)s_post_ms * 1000) s_stop_requested = true;
        if (s_shot_output && !s_stop_requested && esp_timer_get_time() >= s_next_clock_us) {
            esp_err_t err = record_clocks();
            if (err != ESP_OK) { s_shot_error = err; s_overrun_mask |= 0x80U; s_stop_requested = true; }
        }
        for (unsigned imu = 0; imu < LSM6DSV320X_COUNT; ++imu) {
            if (!(lsm6dsv320x_present_mask() & (1U << imu))) continue;
            uint16_t level = lsm6dsv320x_fifo_level(imu);
            if (level > s_high_water[imu]) s_high_water[imu] = level;
            if (lsm6dsv320x_fifo_overrun(imu)) {
                s_overrun_mask |= (uint8_t)(1U << imu);
            }
            while (level && !s_stop_requested) {
                const uint16_t n = level > CHUNK_ROWS ? CHUNK_ROWS : level;
                if (lsm6dsv320x_fifo_read(imu, s_fifo_raw, n) != ESP_OK) {
                    s_overrun_mask |= (uint8_t)(1U << imu);
                    break;
                }
                send_data_frame(imu, s_fifo_raw, n, level - n);
                level = (uint16_t)(level - n);
            }
        }
        if (s_shot_output && !s_timing_test && !s_stop_requested && shot_store_near_capacity()) {
            if (!s_trigger_us) s_save_full_session = true;
            else s_overrun_mask |= 0x40U; /* Capacity ended the post-window early. */
            s_stop_requested = true;
        }
        if (s_stop_requested) {
            if (s_shot_output) {
                esp_err_t err = record_clocks(); /* Before stopping the sensor clock. */
                if (err != ESP_OK) { s_shot_error = err; s_overrun_mask |= 0x80U; }
            }
            lsm6dsv320x_capture_stop();
            /* Flush the finite tail before reporting completion. */
            for (unsigned imu = 0; imu < LSM6DSV320X_COUNT; ++imu) {
                if (!(lsm6dsv320x_present_mask() & (1U << imu))) continue;
                uint16_t level = lsm6dsv320x_fifo_level(imu);
                while (level) {
                    const uint16_t n = level > CHUNK_ROWS ? CHUNK_ROWS : level;
                    if (lsm6dsv320x_fifo_read(imu, s_fifo_raw, n) != ESP_OK) {
                        s_overrun_mask |= (uint8_t)(1U << imu);
                        break;
                    }
                    send_data_frame(imu, s_fifo_raw, n, level - n);
                    level = (uint16_t)(level - n);
                }
            }
            lsm6dsv320x_capture_finish();
            if (s_shot_output) {
                for (unsigned imu = 0; imu < LSM6DSV320X_COUNT; ++imu)
                    if (s_pending[imu].data) s_overrun_mask |= 1U << imu;
                const esp_err_t err = s_timing_test ? ESP_OK : shot_store_finish(s_started_us, s_trigger_us,
                    esp_timer_get_time(), s_overrun_mask, s_saturation_mask, s_save_full_session);
                if (err != ESP_OK) s_shot_error = err;
                s_shot_output = false;
                if (s_timing_test) {
                    s_drdy_active = false;
                    (void)lsm6dsv320x_drdy_monitor(false);
                    for (unsigned imu = 0; imu < 4; ++imu)
                        usb_printf("TIMING sensor=%u count=%lu gaps=%lu slots=%lu pairs=%lu high_water=%u fifo_error=%u\n",
                            imu, (unsigned long)s_counts[imu], (unsigned long)s_gaps[imu],
                            (unsigned long)s_slot_errors[imu], (unsigned long)s_pair_errors[imu],
                            s_high_water[imu], lsm6dsv320x_fifo_overrun(imu));
                    for (unsigned imu = 0; imu < 4; ++imu)
                        usb_printf("TIMING DRDY sensor=%u count=%lu paired_count=%lu late=%lu min_us=%lu max_us=%lu first_match_us=%ld\n",
                            imu, (unsigned long)s_drdy_count[imu],
                            (unsigned long)(s_first_edge[imu] ? s_drdy_count[imu] - s_first_edge[imu] + 1U : 0),
                            (unsigned long)s_drdy_late[imu], (unsigned long)s_drdy_min[imu],
                            (unsigned long)s_drdy_max[imu], (long)s_edge_match_us[imu]);
                    for (unsigned imu = 0; imu < 4; ++imu)
                        for (unsigned j = 0; j < s_gaps[imu] && j < 8U; ++j) {
                            const timing_gap_t *g = &s_gap_detail[imu][j];
                            usb_printf("TIMING GAP sensor=%u index=%lu ticks=%lu:%lu slot_step=%u clock_age_us=%lu drdy_delta_us=%ld\n",
                                imu, (unsigned long)g->index, (unsigned long)g->before,
                                (unsigned long)g->after, g->slot_step, (unsigned long)g->clock_age_us, (long)g->drdy_delta_us);
                        }
                    usb_printf("TIMING DONE elapsed_ms=%lu quality=%u error=%d\n",
                        (unsigned long)((esp_timer_get_time() - s_started_us) / 1000), s_overrun_mask, s_shot_error);
                    s_timing_test = false;
                }
            } else send_end_frame();
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
    const gpio_config_t drdy_pins = {
        .pin_bit_mask = (1ULL << 20) | (1ULL << 21) | (1ULL << 22) | (1ULL << 23),
        .mode = GPIO_MODE_INPUT, .intr_type = GPIO_INTR_POSEDGE,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t err = gpio_config(&drdy_pins);
    if (err != ESP_OK) return err;
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (err != ESP_OK) return err;
    for (unsigned imu = 0; imu < 4; ++imu) {
        err = gpio_isr_handler_add((gpio_num_t)(20 + imu), drdy_isr, (void *)(uintptr_t)imu);
        if (err != ESP_OK) return err;
    }
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
    if (lsm6dsv320x_capture_start(false, &s_started_us) != ESP_OK) return false;
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
    if (lsm6dsv320x_capture_start(false, &s_started_us) != ESP_OK) { s_wifi_output = false; return false; }
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
    if (lsm6dsv320x_capture_start(false, &s_started_us) != ESP_OK) { s_wifi_output = s_browser_output = false; return false; }
    s_running = true;
    xTaskNotifyGive(s_task);
    return true;
}

bool capture_start_shot(uint32_t pre_ms, uint32_t post_ms, uint32_t threshold_mg, const char *metadata)
{
    if (!s_task || s_running || !lsm6dsv320x_present_mask() || threshold_mg > 320000 ||
        (threshold_mg && threshold_mg < 20)) return false;
    s_shot_error = shot_store_begin(pre_ms, post_ms, metadata);
    if (s_shot_error != ESP_OK) return false;
    s_pre_ms = pre_ms; s_post_ms = post_ms; s_threshold_mg = threshold_mg;
    s_timing_test = false;
    s_wifi_output = false; s_browser_output = false; s_shot_output = true;
    s_trigger_us = 0; s_trigger_requested = false; s_saturation_mask = 0;
    s_save_full_session = false;
    s_duration_limit_us = 0;
    s_overrun_mask = 0; s_stop_requested = false;
    memset(s_counts, 0, sizeof(s_counts));
    memset(s_pending, 0, sizeof(s_pending));
    memset(s_hw_valid, 0, sizeof(s_hw_valid));
    memset(s_clock_us, 0, sizeof(s_clock_us));
    s_drdy_active = false;
    memset((void *)s_drdy_edges, 0, sizeof(s_drdy_edges));
    memset((void *)s_drdy_count, 0, sizeof(s_drdy_count));
    memset((void *)s_drdy_last, 0, sizeof(s_drdy_last));
    memset((void *)s_drdy_min, 0xff, sizeof(s_drdy_min));
    memset((void *)s_drdy_max, 0, sizeof(s_drdy_max));
    memset((void *)s_drdy_late, 0, sizeof(s_drdy_late));
    memset(s_direct_consumed, 0, sizeof(s_direct_consumed));
    memset(s_direct_backlog, 0, sizeof(s_direct_backlog));
    s_direct_error = 0; s_direct_shot = true;
    s_shot_error = lsm6dsv320x_direct_start(&s_started_us);
    if (s_shot_error != ESP_OK) {
        lsm6dsv320x_direct_release();
        lsm6dsv320x_capture_stop(); lsm6dsv320x_capture_finish();
        (void)lsm6dsv320x_drdy_monitor(false); s_direct_shot = false;
        (void)shot_store_finish(0, 0, 0, 0, 0, false);
        s_shot_output = false;
        return false;
    }
    s_running = true; s_drdy_active = true;
    xTaskNotifyGive(s_task);
    return true;
}

bool capture_start_timing_test(uint32_t seconds)
{
    if (!s_task || s_running || !lsm6dsv320x_present_mask() || seconds < 1 || seconds > 600) return false;
    s_timing_test = true; s_shot_output = true; s_wifi_output = s_browser_output = false;
    s_trigger_us = 0; s_trigger_requested = false; s_threshold_mg = 0;
    s_stop_requested = false; s_overrun_mask = s_saturation_mask = 0;
    s_duration_limit_us = (int64_t)seconds * 1000000;
    memset(s_counts, 0, sizeof(s_counts)); memset(s_pending, 0, sizeof(s_pending));
    memset(s_hw_valid, 0, sizeof(s_hw_valid)); memset(s_clock_us, 0, sizeof(s_clock_us));
    memset(s_gaps, 0, sizeof(s_gaps)); memset(s_slot_errors, 0, sizeof(s_slot_errors));
    memset(s_pair_errors, 0, sizeof(s_pair_errors)); memset(s_high_water, 0, sizeof(s_high_water));
    memset((void *)s_drdy_edges, 0, sizeof(s_drdy_edges));
    memset((void *)s_drdy_count, 0, sizeof(s_drdy_count));
    memset((void *)s_drdy_last, 0, sizeof(s_drdy_last));
    memset((void *)s_drdy_min, 0xff, sizeof(s_drdy_min));
    memset((void *)s_drdy_max, 0, sizeof(s_drdy_max));
    memset((void *)s_drdy_late, 0, sizeof(s_drdy_late));
    memset(s_first_edge, 0, sizeof(s_first_edge)); memset(s_edge_match_us, 0, sizeof(s_edge_match_us));
    s_drdy_active = true;
    s_shot_error = lsm6dsv320x_drdy_monitor(true);
    if (s_shot_error == ESP_OK) s_shot_error = lsm6dsv320x_capture_start(true, &s_started_us);
    if (s_shot_error == ESP_OK) s_shot_error = record_clocks();
    if (s_shot_error != ESP_OK) {
        lsm6dsv320x_capture_stop(); lsm6dsv320x_capture_finish();
        s_drdy_active = false; (void)lsm6dsv320x_drdy_monitor(false);
        s_timing_test = s_shot_output = false; return false;
    }
    s_running = true; xTaskNotifyGive(s_task); return true;
}

bool capture_trigger_shot(void)
{
    if (!capture_shot_ready()) return false;
    s_trigger_requested = true;
    return true;
}

bool capture_start_direct_test(uint32_t seconds)
{
    if (!s_task || s_running || !lsm6dsv320x_present_mask() || seconds < 1 || seconds > 600) return false;
    s_drdy_active = false;
    memset((void *)s_drdy_edges, 0, sizeof(s_drdy_edges));
    memset((void *)s_drdy_count, 0, sizeof(s_drdy_count));
    memset((void *)s_drdy_last, 0, sizeof(s_drdy_last));
    memset((void *)s_drdy_min, 0xff, sizeof(s_drdy_min));
    memset((void *)s_drdy_max, 0, sizeof(s_drdy_max));
    memset((void *)s_drdy_late, 0, sizeof(s_drdy_late));
    memset(s_counts, 0, sizeof(s_counts)); memset(s_direct_sums, 0, sizeof(s_direct_sums));
    memset(s_direct_backlog, 0, sizeof(s_direct_backlog));
    s_direct_error = 0; s_direct_test = true; s_timing_test = s_shot_output = false;
    s_wifi_output = s_browser_output = false; s_stop_requested = false;
    s_duration_limit_us = (int64_t)seconds * 1000000;
    s_shot_error = lsm6dsv320x_direct_start(&s_started_us);
    if (s_shot_error != ESP_OK) {
        lsm6dsv320x_direct_release(); lsm6dsv320x_capture_stop(); lsm6dsv320x_capture_finish();
        (void)lsm6dsv320x_drdy_monitor(false); s_direct_test = false; return false;
    }
    s_running = true; s_drdy_active = true; xTaskNotifyGive(s_task); return true;
}
bool capture_shot_active(void) { return s_running && s_shot_output && !s_timing_test; }
bool capture_shot_ready(void)
{
    const int64_t elapsed = esp_timer_get_time() - s_started_us;
    return capture_shot_active() && !s_trigger_us && !s_stop_requested &&
        elapsed >= 0 && !shot_store_near_capacity();
}
uint32_t capture_shot_elapsed_ms(void)
{
    if (!capture_shot_active()) return 0;
    const int64_t elapsed = esp_timer_get_time() - s_started_us;
    return elapsed > 0 ? (uint32_t)(elapsed / 1000) : 0;
}
const char *capture_shot_state(void)
{
    if (capture_shot_active()) return s_trigger_us ? "post" : "armed";
    if (shot_store_corrupt()) return "corrupt";
    if (s_shot_error != ESP_OK) return "error";
    return shot_store_saved() ? "saved" : "idle";
}
int capture_shot_error(void) { return (int)s_shot_error; }

void capture_stop(void)
{
    if (s_running) {
        if (s_shot_output && s_trigger_us) s_overrun_mask |= 0x40U; /* Post-window cancelled. */
        s_stop_requested = true;
    }
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
