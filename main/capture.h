/*
 * capture.h — PSRAM capture engine for IIS3DWB10IS.
 *
 * Two modes:
 *   STORE   fill a large PSRAM buffer from the FIFO until full, then stop.
 *           (primary: matches "store onboard until memory full, a few seconds")
 *   STREAM  drain the FIFO straight to USB continuously (fallback).
 *
 * The FIFO watermark interrupt (INT1) wakes the capture task, which bulk-reads
 * rows directly into PSRAM (or the USB scratch for STREAM).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "iis3dwb10is.h"

esp_err_t capture_init(void);                         /* alloc PSRAM, INT1 cb, task */

void capture_start_store(iis3dwb10is_odr_t odr, iis3dwb10is_fs_t fs);   /* fill PSRAM until full */
void capture_start_stream(iis3dwb10is_odr_t odr, iis3dwb10is_fs_t fs);  /* live → USB */
void capture_stop(void);                                 /* user/USB initiated stop */
bool capture_running(void);

/* Accessors used by DUMP / STATUS. */
uint8_t  *capture_buf(void);
uint32_t  capture_bytes(void);      /* valid bytes (= rows * 10) */
uint32_t  capture_rows(void);
uint32_t  capture_capacity(void);   /* buffer capacity, bytes */
bool      capture_overrun(void);
uint32_t  capture_duration_ms(void);/* last STORE wall time */
iis3dwb10is_odr_t capture_last_odr(void);
iis3dwb10is_fs_t  capture_last_fs(void);
uint32_t  capture_psram_size(void); /* detected PSRAM size, bytes */
