/*
 * usb.c — USB-Serial-JTAG command shell + binary dump.
 *
 * Line-based ASCII commands in, framed binary out (DUMP). Same COM port the
 * PC tool (inspect_raw.py) talks to. Text responses use usb_printf; the
 * multi-MB DUMP payload uses usb_send in chunks.
 */
#include "usb.h"
#include "capture.h"
#include "iis3dwb10is.h"

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "usb";

static SemaphoreHandle_t s_tx_lock;
/* Validated lossless default: 80 kS/s captured to PSRAM, then dumped. */
static iis3dwb10is_odr_t s_odr = IIS3DWB10IS_ODR_80K;
static iis3dwb10is_fs_t  s_fs  = IIS3DWB10IS_FS_50G;
static iis3dwb10is_bits_t s_bits = IIS3DWB10IS_BITS_20;

#define RAW_COMPARE_SAMPLES   100U
#define RAW_SETTLE_MS          50U
#define RAW_READY_POLL_US      10U
#define RAW_READY_TIMEOUT_US 20000U
static uint8_t s_raw_fifo[RAW_COMPARE_SAMPLES * FIFO_ROW_BYTES];

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
    case IIS3DWB10IS_ODR_2P5K: return 2500;
    case IIS3DWB10IS_ODR_5K:   return 5000;
    case IIS3DWB10IS_ODR_10K:  return 10000;
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
    put_u16(&hdr[20], (uint16_t)iis3dwb10is_output_bits());
    /* hdr[22..31] reserved = 0 */

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
               iis3dwb10is_fs_to_mglsb_bits(s_fs, s_bits));
    usb_printf("STATUS output_bits=%u\n", (unsigned)s_bits);
    usb_printf("STATUS spi_khz=%d\n", iis3dwb10is_spi_khz());
}

static void do_temperature(void)
{
    if (capture_running()) {
        usb_printf("ERR busy, send STOP before TEMP\n");
        return;
    }

    int32_t sum = 0;
    iis3dwb10is_set_output_bits(IIS3DWB10IS_BITS_20);
    iis3dwb10is_start(IIS3DWB10IS_ODR_80K, IIS3DWB10IS_FS_50G);
    vTaskDelay(pdMS_TO_TICKS(50));
    for (unsigned sample = 0; sample < 4U; ++sample) {
        int16_t raw = 0;
        iis3dwb10is_temperature_read(&raw, NULL);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    iis3dwb10is_stop();
    iis3dwb10is_set_output_bits(s_bits);

    const float raw_average = (float)sum / 4.0f;
    usb_printf("TEMP raw=%.2f celsius=%.3f samples=4 sensitivity=200_lsb_per_c\n",
               raw_average, 25.0f + raw_average / 200.0f);
}

static int32_t get_i32_le(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)u;
}

static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t get_i20_le(const uint8_t *p)
{
    uint32_t u = ((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                  ((uint32_t)p[2] << 16)) & 0x000FFFFFU;
    if (u & 0x00080000U) {
        u |= 0xFFF00000U;
    }
    return (int32_t)u;
}

static bool wait_for_direct_sample(void)
{
    for (unsigned elapsed = 0; elapsed < RAW_READY_TIMEOUT_US;
         elapsed += RAW_READY_POLL_US) {
        if ((iis3dwb10is_read(REG_STATUS_REG) & 0x01U) != 0U) {
            return true;
        }
        esp_rom_delay_us(RAW_READY_POLL_US);
    }
    return false;
}

static bool wait_for_fifo_samples(uint16_t wanted, uint16_t *level)
{
    for (unsigned tries = 0; tries < 50U; ++tries) {
        *level = iis3dwb10is_fifo_level();
        if (*level >= wanted) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

static void print_raw_average(const char *mode, const int64_t sum[3],
                              float mglsb)
{
    const double x = (double)sum[0] / RAW_COMPARE_SAMPLES;
    const double y = (double)sum[1] / RAW_COMPARE_SAMPLES;
    const double z = (double)sum[2] / RAW_COMPARE_SAMPLES;
    const double gx = x * mglsb / 1000.0;
    const double gy = y * mglsb / 1000.0;
    const double gz = z * mglsb / 1000.0;
    const double magnitude = sqrt(gx * gx + gy * gy + gz * gz);

    usb_printf("RAW %s avg_counts x=%.2f y=%.2f z=%.2f samples=%u\n",
               mode, x, y, z, RAW_COMPARE_SAMPLES);
    usb_printf("RAW %s avg_g x=%+.6f y=%+.6f z=%+.6f magnitude=%.6f "
               "mglsb=%.3f\n", mode, gx, gy, gz, magnitude, mglsb);
}

static bool prepare_direct(iis3dwb10is_bits_t bits, uint8_t *discard,
                           uint16_t bytes)
{
    iis3dwb10is_set_output_bits(bits);
    iis3dwb10is_start(IIS3DWB10IS_ODR_10K, IIS3DWB10IS_FS_50G);
    vTaskDelay(pdMS_TO_TICKS(RAW_SETTLE_MS));

    /* Reading one complete settled value clears any stale XLDA/output value.
       Every value collected after this point must assert a new data-ready. */
    if (!wait_for_direct_sample()) {
        return false;
    }
    iis3dwb10is_read_burst(REG_OUTX_L_A, discard, bytes);
    return true;
}

static bool raw_direct16(void)
{
    uint8_t raw[6] = { 0 };
    int64_t sum[3] = { 0 };
    if (!prepare_direct(IIS3DWB10IS_BITS_16, raw, sizeof(raw))) {
        usb_printf("RAW ERROR mode=direct16 reason=settle_timeout\n");
        iis3dwb10is_stop();
        return false;
    }

    for (unsigned sample = 0; sample < RAW_COMPARE_SAMPLES; ++sample) {
        if (!wait_for_direct_sample()) {
            usb_printf("RAW ERROR mode=direct16 reason=data_ready_timeout "
                       "sample=%u\n", sample);
            iis3dwb10is_stop();
            return false;
        }
        iis3dwb10is_read_burst(REG_OUTX_L_A, raw, sizeof(raw));
        sum[0] += get_i16_le(&raw[0]);
        sum[1] += get_i16_le(&raw[2]);
        sum[2] += get_i16_le(&raw[4]);
    }
    iis3dwb10is_stop();

    usb_printf("RAW direct16 bytes=%02X %02X|%02X %02X|%02X %02X "
               "sample=last\n", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
    print_raw_average("direct16", sum,
                      iis3dwb10is_fs_to_mglsb_bits(IIS3DWB10IS_FS_50G,
                                                   IIS3DWB10IS_BITS_16));
    return true;
}

static bool raw_direct20(void)
{
    uint8_t raw[12] = { 0 };
    int64_t sum[3] = { 0 };
    if (!prepare_direct(IIS3DWB10IS_BITS_20, raw, sizeof(raw))) {
        usb_printf("RAW ERROR mode=direct20 reason=settle_timeout\n");
        iis3dwb10is_stop();
        return false;
    }

    for (unsigned sample = 0; sample < RAW_COMPARE_SAMPLES; ++sample) {
        if (!wait_for_direct_sample()) {
            usb_printf("RAW ERROR mode=direct20 reason=data_ready_timeout "
                       "sample=%u\n", sample);
            iis3dwb10is_stop();
            return false;
        }
        iis3dwb10is_read_burst(REG_OUTX_L_A, raw, sizeof(raw));
        sum[0] += get_i32_le(&raw[0]);
        sum[1] += get_i32_le(&raw[4]);
        sum[2] += get_i32_le(&raw[8]);
    }
    iis3dwb10is_stop();

    usb_printf("RAW direct20 bytes=%02X %02X %02X %02X|"
               "%02X %02X %02X %02X|%02X %02X %02X %02X sample=last\n",
               raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6],
               raw[7], raw[8], raw[9], raw[10], raw[11]);
    print_raw_average("direct20", sum,
                      iis3dwb10is_fs_to_mglsb(IIS3DWB10IS_FS_50G));
    return true;
}

static bool raw_fifo20(void)
{
    int64_t sum[3] = { 0 };
    uint16_t startup_rows = 0;
    uint16_t level = 0;
    iis3dwb10is_set_output_bits(IIS3DWB10IS_BITS_20);
    iis3dwb10is_start(IIS3DWB10IS_ODR_20K, IIS3DWB10IS_FS_50G);
    vTaskDelay(pdMS_TO_TICKS(RAW_SETTLE_MS));

    if (iis3dwb10is_fifo_overrun()) {
        usb_printf("RAW ERROR mode=fifo20 reason=fifo_startup_overrun\n");
        iis3dwb10is_stop();
        return false;
    }
    startup_rows = iis3dwb10is_fifo_level();
    iis3dwb10is_fifo_flush_running();
    if (!wait_for_fifo_samples(RAW_COMPARE_SAMPLES, &level)) {
        usb_printf("RAW ERROR mode=fifo20 reason=fifo_fresh_timeout\n");
        iis3dwb10is_stop();
        return false;
    }
    iis3dwb10is_fifo_read(s_raw_fifo, RAW_COMPARE_SAMPLES);
    iis3dwb10is_stop();

    for (unsigned sample = 0; sample < RAW_COMPARE_SAMPLES; ++sample) {
        const uint8_t *row = &s_raw_fifo[sample * FIFO_ROW_BYTES];
        if (row[0] != 0x10U) {
            usb_printf("RAW ERROR mode=fifo20 reason=bad_tag tag=%02X sample=%u\n",
                       row[0], sample);
            return false;
        }
        sum[0] += get_i20_le(&row[1]);
        sum[1] += get_i20_le(&row[4]);
        sum[2] += get_i20_le(&row[7]);
    }

    const uint8_t *raw = &s_raw_fifo[(RAW_COMPARE_SAMPLES - 1U) * FIFO_ROW_BYTES];
    usb_printf("RAW fifo20 bytes=%02X|%02X %02X %02X|%02X %02X %02X|"
               "%02X %02X %02X sample=last startup_flushed=%u level=%u\n",
               raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6],
               raw[7], raw[8], raw[9], (unsigned)startup_rows, (unsigned)level);
    print_raw_average("fifo20", sum,
                      iis3dwb10is_fs_to_mglsb(IIS3DWB10IS_FS_50G));
    return true;
}

/* Compare native layouts at a fixed +/-50 g. Each mode settles, discards old
   output, and averages 100 fresh samples. Shell capture settings are preserved. */
static void do_raw(const char *mode)
{
    if (capture_running()) {
        usb_printf("ERR busy, send STOP first\n");
        return;
    }
    if (!mode) {
        mode = "ALL";
    }
    const bool all = !strcmp(mode, "ALL");
    if (!all && strcmp(mode, "DIRECT16") && strcmp(mode, "DIRECT20") &&
        strcmp(mode, "FIFO20")) {
        usb_printf("ERR RAW mode must be ALL|DIRECT16|DIRECT20|FIFO20\n");
        return;
    }

    usb_printf("RAW BEGIN mode=%s fs=50 settle_ms=%u samples=%u\n", mode,
               RAW_SETTLE_MS, RAW_COMPARE_SAMPLES);
    if (all || !strcmp(mode, "DIRECT16")) raw_direct16();
    if (all || !strcmp(mode, "DIRECT20")) raw_direct20();
    if (all || !strcmp(mode, "FIFO20"))   raw_fifo20();
    iis3dwb10is_set_output_bits(s_bits);
    usb_printf("RAW END\n");
}

/* Hardware diagnostic: direct output and one packed row from the same instant. */
static void do_probe(void)
{
    if (capture_running()) {
        usb_printf("ERR busy, send STOP first\n");
        return;
    }

    uint8_t direct[12] = { 0 };
    uint8_t row[FIFO_ROW_BYTES] = { 0 };
    iis3dwb10is_start(s_odr, s_fs);
    const int64_t rate_start_us = esp_timer_get_time();
    esp_rom_delay_us(10000);
    uint16_t level_10ms = iis3dwb10is_fifo_level();
    int64_t time_10ms = esp_timer_get_time() - rate_start_us;
    esp_rom_delay_us(5000);
    uint16_t level_15ms = iis3dwb10is_fifo_level();
    int64_t time_15ms = esp_timer_get_time() - rate_start_us;
    esp_rom_delay_us(5000);
    uint16_t level_20ms = iis3dwb10is_fifo_level();
    int64_t time_20ms = esp_timer_get_time() - rate_start_us;
    uint8_t ctrl1 = iis3dwb10is_read(REG_CTRL1);
    uint8_t ctrl2 = iis3dwb10is_read(REG_CTRL2);
    uint8_t ctrl3 = iis3dwb10is_read(REG_CTRL3);
    uint8_t ctrl4 = iis3dwb10is_read(REG_CTRL4);
    uint8_t pll1 = iis3dwb10is_read(REG_PLL_CTRL1);
    uint8_t pll2 = iis3dwb10is_read(REG_PLL_CTRL2);
    uint8_t int_ctrl1 = iis3dwb10is_read(0x0C);
    uint8_t int_ctrl2 = iis3dwb10is_read(REG_INT_CTRL2);
    uint8_t status = iis3dwb10is_read(REG_STATUS_REG);
    uint8_t fifo_ctrl3 = iis3dwb10is_read(REG_FIFO_CTRL3);
    uint16_t level = level_20ms;
    iis3dwb10is_read_burst(REG_OUTX_L_A, direct, sizeof(direct));
    if (iis3dwb10is_direct_mode()) {
        (void)iis3dwb10is_direct_read_row(row);
    } else if (level) {
        iis3dwb10is_fifo_read(row, 1);
    }
    iis3dwb10is_stop();

    usb_printf("PROBE regs ctrl1=%02X ctrl2=%02X ctrl3=%02X ctrl4=%02X "
               "pll1=%02X pll2=%02X "
               "int1=%02X int2=%02X status=%02X int1_gpio=%d "
               "fifo_ctrl3=%02X level=%u rate10=%u@%lldus rate15=%u@%lldus "
               "rate20=%u@%lldus steady_rate=%.1f\n",
               ctrl1, ctrl2, ctrl3, ctrl4, pll1, pll2,
               int_ctrl1, int_ctrl2, status,
               gpio_get_level(IIS3DWB10IS_PIN_INT1), fifo_ctrl3, (unsigned)level,
               (unsigned)level_10ms, time_10ms,
               (unsigned)level_15ms, time_15ms,
               (unsigned)level_20ms, time_20ms,
               (double)(level_20ms - level_10ms) * 1000000.0 /
               (double)(time_20ms - time_10ms));
    usb_printf("PROBE direct x=%ld y=%ld z=%ld raw="
               "%02X%02X%02X%02X %02X%02X%02X%02X %02X%02X%02X%02X\n",
               (long)get_i32_le(&direct[0]), (long)get_i32_le(&direct[4]),
               (long)get_i32_le(&direct[8]),
               direct[3], direct[2], direct[1], direct[0],
               direct[7], direct[6], direct[5], direct[4],
               direct[11], direct[10], direct[9], direct[8]);
    usb_printf("PROBE fifo=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
               row[0], row[1], row[2], row[3], row[4],
               row[5], row[6], row[7], row[8], row[9]);
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
    } else if (!strcmp(cmd, "TEMP")) {
        do_temperature();
    } else if (!strcmp(cmd, "PROBE")) {
        do_probe();
    } else if (!strcmp(cmd, "RAW")) {
        do_raw(strtok(NULL, " \t"));
    } else if (!strcmp(cmd, "SET")) {
        char *k = strtok(NULL, " \t");
        char *v = strtok(NULL, " \t");
        if (k && v) {
            long n = strtol(v, NULL, 10);
            if (!strcmp(k, "ODR")) {
                if (s_bits == IIS3DWB10IS_BITS_16 && n > 10000L) {
                    usb_printf("ERR ODR above 10000 requires BITS 20 (16-bit mode is not FIFO-compatible)\n");
                    return;
                }
                if      (n ==  2500) s_odr = IIS3DWB10IS_ODR_2P5K;
                else if (n ==  5000) s_odr = IIS3DWB10IS_ODR_5K;
                else if (n == 10000) s_odr = IIS3DWB10IS_ODR_10K;
                else if (n == 20000) s_odr = IIS3DWB10IS_ODR_20K;
                else if (n == 40000) s_odr = IIS3DWB10IS_ODR_40K;
                else if (n == 80000) s_odr = IIS3DWB10IS_ODR_80K;
                else { usb_printf("ERR ODR must be 2500|5000|10000|20000|40000|80000\n"); return; }
                usb_printf("OK ODR=%ld (applies on next START)\n", n);
            } else if (!strcmp(k, "FS")) {
                if      (n ==  50) s_fs = IIS3DWB10IS_FS_50G;
                else if (n == 100) s_fs = IIS3DWB10IS_FS_100G;
                else if (n == 200) s_fs = IIS3DWB10IS_FS_200G;
                else { usb_printf("ERR FS must be 50|100|200\n"); return; }
                usb_printf("OK FS=%ld g (applies on next START)\n", n);
            } else if (!strcmp(k, "BITS")) {
                if (n != 16 && n != 20) {
                    usb_printf("ERR BITS must be 16|20\n");
                    return;
                }
                if (n == 16 && odr_hz(s_odr) > 10000U) {
                    usb_printf("ERR BITS 16 requires ODR <= 10000 (16-bit mode is not FIFO-compatible)\n");
                    return;
                }
                s_bits = (n == 16) ? IIS3DWB10IS_BITS_16 : IIS3DWB10IS_BITS_20;
                iis3dwb10is_set_output_bits(s_bits);
                usb_printf("OK BITS=%ld (applies on next START)\n", n);
            } else {
                usb_printf("ERR SET needs ODR, FS, or BITS\n");
            }
        } else {
            usb_printf("ERR SET ODR|FS <value>\n");
        }
    } else if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) {
        usb_printf("BowVib cmds: START STOP DUMP STREAM STATUS TEMP "
                   "PROBE RAW [ALL|DIRECT16|DIRECT20|FIFO20] "
                   "SET ODR 2500|5000|10000|20000|40000|80000 "
                   "SET FS 50|100|200 SET BITS 16|20 HELP\n");
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

    usb_printf("\nBowVib ready. IIS3DWB10IS SPI capture. cmds: RAW START STOP DUMP "
               "STREAM STATUS TEMP HELP\n");
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
