/*
 * iis3dwb10is.h — minimal ST IIS3DWB10IS driver (SPI), ESP-IDF.
 *
 * Register map / bit fields from ST's platform-independent driver
 * (iis3dwb10is_reg.h) and datasheet DS14585. Hand-rolled minimal driver:
 * direct register reads/writes over SPI, bit-7 = read direction, IF
 * auto-increment (CTRL3.if_inc) for multi-byte / FIFO bursts.
 *
 * Sensor: 3-axis vibration accelerometer, FS ±50/±100/±200 g, 20-bit/axis,
 * continuous-mode ODR up to 80 kHz (>10 kHz bandwidth). Output is read
 * through the 2048-slot FIFO (10-byte rows: tag + X/Y/Z, each 3-byte LE).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"

/* ---- Wiring (ESP32-S3-DevKitC-1, SPI2_HOST) ---------------------------- */
#define IIS3DWB10IS_SPI_HOST   SPI2_HOST
#define IIS3DWB10IS_PIN_SCK    12
#define IIS3DWB10IS_PIN_MISO   13
#define IIS3DWB10IS_PIN_MOSI   11
#define IIS3DWB10IS_PIN_CS     10
#define IIS3DWB10IS_PIN_INT1   4
#define IIS3DWB10IS_SPI_HZ     (8 * 1000 * 1000)   /* 8 MHz; chip supports ~10 MHz */

/* ---- Register addresses (from iis3dwb10is_reg.h) ----------------------- */
#define REG_FIFO_CTRL1    0x05U
#define REG_FIFO_CTRL2    0x06U
#define REG_FIFO_CTRL3    0x07U
#define REG_INT_CTRL2     0x0DU   /* INT1 event routing */
#define REG_WHO_AM_I      0x0FU
#define REG_CTRL1         0x10U   /* odr_xl[3:0], burst_cfg[7:4] */
#define REG_CTRL2         0x11U   /* fs_xl[6:5] */
#define REG_CTRL3         0x12U   /* sw_reset/if_inc/fifo_en/bdu/boot */
#define REG_CTRL4         0x13U   /* axis enables [4:2] */
#define REG_STATUS_REG    0x1EU
#define REG_FIFO_STATUS1  0x4CU
#define REG_FIFO_STATUS2  0x4DU
#define REG_FIFO_OUT      0x40U   /* FIFO_DATA_OUT_TAG; auto-inc streams rows */

#define IIS3DWB10IS_ID            0x50U
#define FIFO_DEPTH        2048U
#define FIFO_ROW_BYTES    10U
#define FIFO_WATERMARK    512U    /* IRQ at ~512 rows; drained well before 2048 */

/* ---- Bit helpers -------------------------------------------------------- */
#define CTRL3_SW_RESET    0x01U
#define CTRL3_IF_INC      0x04U
#define CTRL3_FIFO_EN     0x10U
#define CTRL3_BDU         0x40U
#define CTRL4_AXES        0x1CU          /* X|Y|Z enable, bits 2..4 */
#define INT1_FIFO_TH      0x10U          /* route FIFO watermark to INT1 */

/* ---- ODR / FS encodings (CTRL1 low nibble = odr_xl; CTRL2 bits 5..6 = fs_xl) */
typedef enum {
    IIS3DWB10IS_ODR_IDLE = 0x00,
    IIS3DWB10IS_ODR_20K  = 0x05,   /* BW = 10 kHz (ODR/2) */
    IIS3DWB10IS_ODR_40K  = 0x06,   /* BW > 10 kHz */
    IIS3DWB10IS_ODR_80K  = 0x07,   /* BW > 10 kHz */
} iis3dwb10is_odr_t;

typedef enum {
    IIS3DWB10IS_FS_50G  = 0,
    IIS3DWB10IS_FS_100G = 1,
    IIS3DWB10IS_FS_200G = 2,
} iis3dwb10is_fs_t;

/* mg per LSB (20-bit) from iis3dwb10is_from_fsXXg_to_mg() */
float iis3dwb10is_fs_to_mglsb(iis3dwb10is_fs_t fs);

/* ---- API ---------------------------------------------------------------- */
esp_err_t iis3dwb10is_init(void);                 /* bus + whoami + reset + idle config */
bool      iis3dwb10is_whoami_ok(void);            /* read WHO_AM_I, compare to 0x50 */

uint8_t   iis3dwb10is_read(uint8_t reg);          /* single-byte register read */
void      iis3dwb10is_write(uint8_t reg, uint8_t val);
void      iis3dwb10is_read_burst(uint8_t reg, uint8_t *dst, uint16_t len);

void      iis3dwb10is_start(iis3dwb10is_odr_t odr, iis3dwb10is_fs_t fs);
void      iis3dwb10is_stop(void);
uint16_t  iis3dwb10is_fifo_level(void);           /* rows available in FIFO */
bool      iis3dwb10is_fifo_overrun(void);         /* latched overrun flag */
void      iis3dwb10is_fifo_clear_overrun(void);
void      iis3dwb10is_fifo_read(uint8_t *dst, uint16_t nrows);  /* bulk rows into dst (any RAM) */

/* Used by the capture engine to wake on the INT1 watermark IRQ. */
typedef void (*iis3dwb10is_int1_cb_t)(void *arg);
void      iis3dwb10is_set_int1_callback(iis3dwb10is_int1_cb_t cb, void *arg);
