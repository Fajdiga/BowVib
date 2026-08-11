/*
 * iis3dwb10is.h — minimal ST IIS3DWB10IS driver (SPI), ESP-IDF.
 *
 * Register map / bit fields from ST's platform-independent driver
 * (iis3dwb10is_reg.h) and datasheet DS14585. Hand-rolled minimal driver:
 * direct register reads/writes over SPI, bit-7 = read direction, IF
 * auto-increment (CTRL3.if_inc) for multi-byte / FIFO bursts.
 *
 * Sensor: 3-axis vibration accelerometer, FS ±50/±100/±200 g, 20-bit/axis,
 * continuous-mode ODR up to 80 kHz (>10 kHz bandwidth). Low ODR captures use
 * INT1 data-ready plus direct 20-bit reads; high ODR captures use the
 * 2048-slot FIFO (10-byte rows: tag + X/Y/Z, each 3-byte LE).
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
/* This PCB is reliable at 8 MHz; 10 MHz and above corrupt status/data reads. */
#define IIS3DWB10IS_SPI_HZ     (8 * 1000 * 1000)

/* ---- Register addresses (from iis3dwb10is_reg.h) ----------------------- */
#define REG_RAM_ACCESS    0x01U   /* sw_por bit 0 */
#define REG_FIFO_CTRL1    0x05U
#define REG_FIFO_CTRL2    0x06U
#define REG_FIFO_CTRL3    0x07U
#define REG_INT_CTRL2     0x0DU   /* INT1 event routing */
#define REG_WHO_AM_I      0x0FU
#define REG_CTRL1         0x10U   /* odr_xl[3:0], burst_cfg[7:4] */
#define REG_CTRL2         0x11U   /* fs_xl[6:5] */
#define REG_CTRL3         0x12U   /* sw_reset/if_inc/fifo_en/bdu/boot */
#define REG_CTRL4         0x13U   /* axis enables [4:2] */
#define REG_ST_CTRL       0x17U   /* self-test [2:0], LPF1 bandwidth [7:4] */
#define REG_STATUS_REG    0x1EU
#define REG_OUT_TEMP_L    0x22U   /* 16-bit LE, 200 LSB/deg C, zero near 25 C */
#define REG_OUTX_L_A      0x24U   /* 12-byte direct 20-bit XYZ output */
#define REG_FIFO_STATUS1  0x4CU
#define REG_FIFO_STATUS2  0x4DU
#define REG_FIFO_OUT      0x40U   /* FIFO_DATA_OUT_TAG; auto-inc streams rows */

#define IIS3DWB10IS_ID            0x50U
#define FIFO_DEPTH        2048U
#define FIFO_ROW_BYTES    10U
#define FIFO_WATERMARK    512U    /* IRQ at ~512 rows; drained well before 2048 */

/* ---- Bit helpers -------------------------------------------------------- */
#define CTRL3_SW_RESET    0x01U
#define RAM_ACCESS_SW_POR 0x01U
#define CTRL3_IF_INC      0x04U
#define CTRL3_FIFO_EN     0x10U
#define CTRL3_BDU         0x40U
#define CTRL4_ROUNDING_16 0x80U         /* ROUNDING=10: 16-bit output mode */
#define CTRL4_AXES        0x1CU          /* X|Y|Z enable, bits 2..4 */
#define INT1_DRDY_XL      0x01U          /* route accelerometer data-ready to INT1 */
#define INT1_FIFO_TH      0x10U          /* route FIFO watermark to INT1 */
#define ST_CTRL_LPF1_AUTO 0x80U          /* ODR-dependent; 20 kHz at 40/80 kS/s */

/* ---- ODR / FS encodings (CTRL1 low nibble = odr_xl; CTRL2 bits 5..6 = fs_xl) */
typedef enum {
    IIS3DWB10IS_ODR_IDLE = 0x00,
    IIS3DWB10IS_ODR_2P5K = 0x02,   /* 2.5 kS/s */
    IIS3DWB10IS_ODR_5K   = 0x03,   /* 5 kS/s */
    IIS3DWB10IS_ODR_10K  = 0x04,   /* 10 kS/s */
    IIS3DWB10IS_ODR_20K  = 0x05,   /* 20 kS/s */
    IIS3DWB10IS_ODR_40K  = 0x06,   /* 40 kS/s */
    IIS3DWB10IS_ODR_80K  = 0x07,   /* 80 kS/s */
} iis3dwb10is_odr_t;

typedef enum {
    IIS3DWB10IS_FS_50G  = 0,
    IIS3DWB10IS_FS_100G = 1,
    IIS3DWB10IS_FS_200G = 2,
} iis3dwb10is_fs_t;

typedef enum {
    IIS3DWB10IS_BITS_20 = 20,
    IIS3DWB10IS_BITS_16 = 16,
} iis3dwb10is_bits_t;

/* mg per LSB (20-bit) from iis3dwb10is_from_fsXXg_to_mg() */
float iis3dwb10is_fs_to_mglsb(iis3dwb10is_fs_t fs);
float iis3dwb10is_fs_to_mglsb_bits(iis3dwb10is_fs_t fs, iis3dwb10is_bits_t bits);

/* ---- API ---------------------------------------------------------------- */
esp_err_t iis3dwb10is_init(void);                 /* bus + whoami + reset + idle config */
bool      iis3dwb10is_whoami_ok(void);            /* read WHO_AM_I, compare to 0x50 */

uint8_t   iis3dwb10is_read(uint8_t reg);          /* single-byte register read */
void      iis3dwb10is_write(uint8_t reg, uint8_t val);
void      iis3dwb10is_read_burst(uint8_t reg, uint8_t *dst, uint16_t len);

void      iis3dwb10is_start(iis3dwb10is_odr_t odr, iis3dwb10is_fs_t fs);
void      iis3dwb10is_stop(void);
void      iis3dwb10is_set_output_bits(iis3dwb10is_bits_t bits);
iis3dwb10is_bits_t iis3dwb10is_output_bits(void);
bool      iis3dwb10is_direct_mode(void);           /* low ODR: INT1 + direct 20-bit reads */
bool      iis3dwb10is_data_ready(void);            /* XLDA fallback if INT1 is unwired */
void      iis3dwb10is_temperature_read(int16_t *raw, float *celsius);
uint16_t  iis3dwb10is_fifo_level(void);           /* rows available in FIFO */
bool      iis3dwb10is_fifo_overrun(void);         /* latched overrun flag */
void      iis3dwb10is_fifo_clear_overrun(void);
void      iis3dwb10is_fifo_flush_running(void);   /* flush/restart FIFO without stopping ODR */
void      iis3dwb10is_fifo_read(uint8_t *dst, uint16_t nrows);  /* bulk rows into dst (any RAM) */

/* Read one 20-bit XYZ sample from OUTX_L_A..OUTZ_HH_A with SPI DMA and pack
   it into the same 10-byte tag/X/Y/Z row used by the FIFO stream. */
bool      iis3dwb10is_direct_read_row(uint8_t *dst);

/* Used by the capture engine to wake on the INT1 watermark IRQ. */
typedef void (*iis3dwb10is_int1_cb_t)(void *arg);
void      iis3dwb10is_set_int1_callback(iis3dwb10is_int1_cb_t cb, void *arg);
